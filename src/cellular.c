/**
 * Cellular MQTT Reporting (SIM7670G modem-side AT+CMQTT*)
 *
 * See cellular.h.
 */

#include "cellular.h"
#include "config.h"
#include "modem_uart.h"
#include "gnss.h"
#include "device_json.h"
#include "report_throttle.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "CELLULAR";

#define CELLULAR_AT_RESP_BUF_SIZE 256

static char s_client_id[32] = "";

// A single pending report, handed from cellular_publish() (called on
// report_task's schedule) to cellular_task (its own background task) via
// s_report_queue. Heap-allocated per report - see cellular_publish() and
// cellular_free_pending(). Carries the phone count and a GNSS fix snapshot
// (already polled by report_task, embedded by value - no allocation/
// lifetime concerns), not a finished payload - the JSON is built later, in
// cellular_task, from this snapshot, right before the actual send.
typedef struct {
    char *topic;
    int phone_count;
    gnss_fix_t fix;
    int qos;
} cellular_pending_report_t;

// Length 1: only the freshest un-sent report is ever kept - see
// cellular_publish()'s "replace, don't queue" handling.
static QueueHandle_t s_report_queue = NULL;

// Bring-up is layered (PDP -> MQTT service start -> MQTT client acquire ->
// MQTT connect) and each layer is idempotent/skippable once up, so a
// reconnect after a publish failure doesn't necessarily replay the whole
// sequence - see cellular_mark_disconnected().
static bool s_pdp_up = false;
static bool s_mqtt_started = false;
static bool s_mqtt_acquired = false;
static bool s_mqtt_connected = false;

static time_t s_next_reconnect_attempt = 0;

// State of the last report actually *sent* (not just queued/attempted) -
// see report_throttle.h. Shared logic, but this instance is private to the
// cellular-MQTT transport - cellular_http.c (mutually exclusive at runtime)
// keeps its own.
static report_throttle_state_t s_throttle;

/**
 * AT+CEREG? - confirm the modem has actually joined the LTE network before
 * attempting PDP/MQTT bring-up, matching the pre-flight step SIMCom's MQTT
 * AT command manual documents ("ensure GPRS network is available") before
 * any MQTT-related operations. <stat> 1 (registered, home) or 5 (registered,
 * roaming) means attached; anything else and AT+CGDCONT/AT+CMQTTSTART are
 * likely to fail or hang waiting on a network that isn't there yet.
 */
static bool cellular_network_registered(void) {
    char resp[CELLULAR_AT_RESP_BUF_SIZE];
    if (!modem_uart_send_at("AT+CEREG?", resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS)) {
        return false;
    }

    // Response looks like "+CEREG: <n>,<stat>[,...]" - only <stat> matters.
    char *p = strstr(resp, "+CEREG:");
    if (p == NULL) {
        return false;
    }
    char *comma = strchr(p, ',');
    if (comma == NULL) {
        return false;
    }
    int stat = atoi(comma + 1);
    return stat == 1 || stat == 5;
}

/**
 * AT+CGDCONT + AT+CGACT - define the PDP context against CELLULAR_APN and
 * actually activate it. No PDP auth is sent - CELLULAR_APN (a T-Mobile MVNO
 * SIM) doesn't require a username/password, see plans/4g-integration.md's
 * "Resolved decisions".
 *
 * This code has gone back and forth on whether AT+NETOPEN belongs here
 * (removed, then re-added, see git history/plans/4g-integration.md's
 * round-by-round log) without ever finding the actual bug: AT+CGDCONT only
 * *defines* a PDP context, it doesn't activate it, and this code never sent
 * the separate activation command - AT+CGACT=1,1 - at all. Found via the
 * SIM7672X/SIM7652X MQTT(S) Application Note's process flowchart (the
 * closest board-specific reference found - hosted on Waveshare's own wiki
 * page for this exact board), which shows AT+CGDCONT -> AT+CGACT -> (verify)
 * -> AT+CMQTTSTART, and never uses AT+NETOPEN at all. AT+CMQTTSTART is
 * documented to be *able* to activate the PDP context itself, but that
 * clearly wasn't sufficient on its own for AT+CMQTTACCQ/CONNECT to then
 * succeed - dropping AT+NETOPEN (again) in favor of the explicit AT+CGACT
 * step this more specific document shows and this code was actually
 * missing.
 */
static bool cellular_bring_up_pdp(void) {
    if (s_pdp_up) {
        return true;
    }

    if (!cellular_network_registered()) {
        ESP_LOGW(TAG, "Not registered on the LTE network yet (AT+CEREG?) - waiting for next attempt");
        return false;
    }

    char cmd[80];
    char resp[CELLULAR_AT_RESP_BUF_SIZE];

    snprintf(cmd, sizeof(cmd), "AT+CGDCONT=1,\"IP\",\"%s\"", CELLULAR_APN);
    if (!modem_uart_send_at(cmd, resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS) ||
        strstr(resp, "OK") == NULL) {
        ESP_LOGW(TAG, "AT+CGDCONT failed: %s", resp);
        return false;
    }

    if (!modem_uart_send_at("AT+CGACT=1,1", resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS) ||
        strstr(resp, "OK") == NULL) {
        ESP_LOGW(TAG, "AT+CGACT=1,1 failed: %s", resp);
        return false;
    }

    // Diagnostic only, not a gate - confirms the PDP context actually got
    // an IP (logged, not checked/required, since a missing/malformed
    // response here shouldn't block bring-up on its own).
    if (modem_uart_send_at("AT+CGPADDR=1", resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS)) {
        ESP_LOGI(TAG, "AT+CGPADDR: %s", resp);
    }

    ESP_LOGI(TAG, "PDP context defined and activated (APN=%s)", CELLULAR_APN);
    s_pdp_up = true;
    return true;
}

bool cellular_pdp_ensure_up(void) {
    return cellular_bring_up_pdp();
}

bool cellular_pdp_is_up(void) {
    return s_pdp_up;
}

/**
 * AT+CMQTTSTART + AT+CMQTTACCQ + AT+CMQTTCONNECT - start the modem's MQTT
 * service (this is what actually activates the PDP context - see
 * cellular_bring_up_pdp()), acquire a client, and connect to
 * MQTT_BROKER_HOST/_PORT. The !s_mqtt_started/!s_mqtt_acquired guards don't
 * currently skip anything in practice - cellular_mark_disconnected() resets
 * both on any failure, and a success never re-enters this function (see
 * cellular_ensure_connected()'s early return), so every call runs the full
 * sequence including the defensive teardown below. Left in place as a
 * cheap safety net rather than removed, in case that failure-handling
 * logic becomes more targeted later.
 */
static bool cellular_mqtt_session_connect(void) {
    char cmd[160];
    char resp[CELLULAR_AT_RESP_BUF_SIZE];

    if (!s_mqtt_started) {
        // Defensive teardown before the first AT+CMQTTSTART each boot. The
        // modem has its own power/reset domain, independent of the ESP32's
        // - across a dev cycle of repeated ESP32 reflashes, the modem
        // itself typically stays powered the whole time. If an earlier
        // attempt got partway through the connect sequence (client
        // acquired, maybe even connected) and was never cleanly torn down
        // (e.g. an ESP32 reset/reflash mid-session), the modem still holds
        // that state even though s_mqtt_started/s_mqtt_acquired here are
        // freshly false - confirmed on hardware twice now: AT+CMQTTSTART
        // on an already-started service, then (once that was fixed)
        // AT+CMQTTACCQ on an already-acquired client index, both failing
        // immediately with plain ERROR (~200ms response, too fast to be a
        // real operation - a state-precondition rejection, not an actual
        // failed attempt). Tear down in the proper order the manual
        // documents (AT+CMQTTDISC -> AT+CMQTTREL -> AT+CMQTTSTOP) before
        // starting fresh, covering both symptoms already hit plus the
        // equivalent one AT+CMQTTCONNECT would hit if a client were left
        // connected. Each step is best-effort/result-ignored - "nothing to
        // tear down" just errors harmlessly, same as AT+CMQTTSTOP already
        // did when nothing was started.
        char teardown_resp[CELLULAR_AT_RESP_BUF_SIZE];
        char teardown_cmd[32];
        // <timeout> is documented range 60-180s ("0" is described as a
        // "use default" sentinel in the SIM7500/7600/7800 manual, but
        // that's outside the stated range and unconfirmed for this
        // chip - use a real in-range value instead of relying on that).
        snprintf(teardown_cmd, sizeof(teardown_cmd), "AT+CMQTTDISC=%d,60", CELLULAR_MQTT_CLIENT_INDEX);
        modem_uart_send_at(teardown_cmd, teardown_resp, sizeof(teardown_resp), CELLULAR_AT_TIMEOUT_MS);
        snprintf(teardown_cmd, sizeof(teardown_cmd), "AT+CMQTTREL=%d", CELLULAR_MQTT_CLIENT_INDEX);
        modem_uart_send_at(teardown_cmd, teardown_resp, sizeof(teardown_resp), CELLULAR_AT_TIMEOUT_MS);
        modem_uart_send_at("AT+CMQTTSTOP", teardown_resp, sizeof(teardown_resp), CELLULAR_AT_TIMEOUT_MS);

        // Documented max response time is 120s (activating the PDP context
        // can take a while on a cold/weak-signal attach) - far longer than
        // the usual short AT exchange.
        if (!modem_uart_send_at("AT+CMQTTSTART", resp, sizeof(resp), CELLULAR_CMQTTSTART_TIMEOUT_MS) ||
            strstr(resp, "OK") == NULL) {
            ESP_LOGW(TAG, "AT+CMQTTSTART failed: %s", resp);
            return false;
        }
        s_mqtt_started = true;
    }

    if (!s_mqtt_acquired) {
        // <server_type> is documented as optional, defaulting to 0 (TCP) -
        // dropped the explicit trailing ",0" this code used to always send.
        // Both MQTT AT command manuals' own worked examples omit it too
        // (e.g. AT+CMQTTACCQ=0,"client test0") despite documenting it as a
        // legal explicit value - on the chance this firmware's parser
        // doesn't treat "explicitly passed default" the same as "omitted"
        // for an optional trailing parameter.
        snprintf(cmd, sizeof(cmd), "AT+CMQTTACCQ=%d,\"%s\"", CELLULAR_MQTT_CLIENT_INDEX, s_client_id);
        if (!modem_uart_send_at(cmd, resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS) ||
            strstr(resp, "OK") == NULL) {
            ESP_LOGW(TAG, "AT+CMQTTACCQ failed: %s", resp);
            return false;
        }
        s_mqtt_acquired = true;
    }

    snprintf(cmd, sizeof(cmd), "AT+CMQTTCONNECT=%d,\"tcp://%s:%d\",%d,1",
             CELLULAR_MQTT_CLIENT_INDEX, MQTT_BROKER_HOST, MQTT_BROKER_PORT, MQTT_KEEPALIVE_SEC);
    // Connect can take a while (network attach + broker handshake), so give
    // it more headroom than the usual short AT exchange.
    if (!modem_uart_send_at(cmd, resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS * 3) ||
        strstr(resp, "OK") == NULL) {
        ESP_LOGW(TAG, "AT+CMQTTCONNECT failed: %s", resp);
        return false;
    }

    ESP_LOGI(TAG, "Cellular MQTT connected to %s:%d", MQTT_BROKER_HOST, MQTT_BROKER_PORT);
    return true;
}

/**
 * Mark the MQTT session down and start a reconnect backoff window. Resets
 * every layered "already done" flag (s_pdp_up/s_mqtt_started/
 * s_mqtt_acquired), not just s_mqtt_connected - forcing the next attempt to
 * replay the whole sequence, including the defensive AT+CMQTTDISC/REL/STOP
 * teardown in cellular_mqtt_session_connect(), rather than skipping
 * straight to AT+CMQTTCONNECT.
 *
 * Earlier versions of this function deliberately left those flags alone,
 * on the theory that a CMQTTCONNECT failure alone doesn't mean the PDP
 * context or the MQTT service itself dropped. That theory is exactly what
 * the AT+CMQTTSTART/AT+CMQTTACCQ failures hit during hardware testing
 * disproved: the modem's own state can end up not matching what these
 * flags assume for reasons that have nothing to do with why the *previous*
 * step here failed (it has its own power/reset domain, independent of the
 * ESP32's - see cellular_mqtt_session_connect()'s teardown comment). A
 * failed CMQTTCONNECT could just as easily leave a client in a state where
 * blindly retrying CMQTTCONNECT hits that same class of stale-state ERROR
 * - the defensive teardown that fixed AT+CMQTTSTART/ACCQ applies just as
 * much to recovering from a failed attempt mid-session as it does to
 * recovering from a previous boot's leftover state, so every reconnect now
 * gets the same "assume nothing, tear down and rebuild" treatment. The
 * extra AT round-trips this costs per retry are cheap relative to
 * CELLULAR_RECONNECT_BACKOFF_SEC's pacing (5 minutes).
 */
static void cellular_mark_disconnected(void) {
    s_pdp_up = false;
    s_mqtt_started = false;
    s_mqtt_acquired = false;
    s_mqtt_connected = false;
    s_next_reconnect_attempt = time(NULL) + CELLULAR_RECONNECT_BACKOFF_SEC;
    ESP_LOGW(TAG, "Cellular MQTT down, backing off %ds before retrying",
             CELLULAR_RECONNECT_BACKOFF_SEC);
}

static bool cellular_ensure_connected(void) {
    if (s_mqtt_connected) {
        return true;
    }
    if (time(NULL) < s_next_reconnect_attempt) {
        // Deliberately logged every call (not just on the backoff
        // transition) - this is what makes it visible on the serial console
        // that report_task() is still trying rather than silently stuck,
        // which matters a lot while the AT+CMQTT* sequence above is still
        // unverified against real hardware (see plans/4g-integration.md).
        ESP_LOGI(TAG, "Still backing off, retrying cellular MQTT in %llds",
                 (long long)(s_next_reconnect_attempt - time(NULL)));
        return false;
    }

    ESP_LOGI(TAG, "Attempting cellular MQTT bring-up...");
    if (!cellular_bring_up_pdp() || !cellular_mqtt_session_connect()) {
        cellular_mark_disconnected();
        return false;
    }

    s_mqtt_connected = true;
    return true;
}

/**
 * Write an AT command, wait for the modem's ">" prompt, then send the raw
 * data that follows it. Per SIMCom's MQTT AT command manual, this is the
 * documented two-phase exchange AT+CMQTTTOPIC/AT+CMQTTPAYLOAD both use:
 * command line -> ">" prompt -> raw data -> OK. This code originally sent
 * the command line and the raw data back-to-back with no wait for the
 * prompt in between, which was likely contributing (along with the
 * interleaving bug fixed alongside this) to the corruption seen during
 * hardware testing. Caller must already hold the UART lock
 * (modem_uart_lock()) - this does NOT take/release it itself, so a
 * multi-step exchange can hold the lock across every step as one atomic
 * unit - see cellular_do_publish().
 */
static bool cellular_write_at_with_prompt(const char *at_cmd, const char *data, size_t data_len,
                                           char *resp, size_t resp_size, int timeout_ms) {
    char line[64];
    int n = snprintf(line, sizeof(line), "%s\r\n", at_cmd);
    modem_uart_write_raw(line, n);

    if (!modem_uart_wait_for_prompt(resp, resp_size, timeout_ms)) {
        ESP_LOGW(TAG, "%s: no '>' prompt: %s", at_cmd, resp);
        return false;
    }

    modem_uart_write_raw(data, data_len);

    int got = modem_uart_read_response(resp, resp_size, timeout_ms);
    if (got <= 0 || strstr(resp, "OK") == NULL) {
        ESP_LOGW(TAG, "%s failed: %s", at_cmd, got > 0 ? resp : "(no response)");
        return false;
    }
    return true;
}

/**
 * Write a plain AT command with no data phase (AT+CMQTTPUB - the topic and
 * payload were already staged by cellular_write_at_with_prompt() above).
 * Caller must already hold the UART lock.
 */
static bool cellular_write_at_plain_locked(const char *at_cmd, char *resp, size_t resp_size,
                                            int timeout_ms) {
    char line[64];
    int n = snprintf(line, sizeof(line), "%s\r\n", at_cmd);
    modem_uart_write_raw(line, n);

    int got = modem_uart_read_response(resp, resp_size, timeout_ms);
    if (got <= 0 || strstr(resp, "OK") == NULL) {
        ESP_LOGW(TAG, "%s failed: %s", at_cmd, got > 0 ? resp : "(no response)");
        return false;
    }
    return true;
}

/**
 * The actual send: the AT+CMQTTTOPIC/PAYLOAD/PUB exchange, held under one
 * continuous UART lock (this used to lock/unlock per step, leaving gaps
 * where another AT-issuing caller could interleave mid-sequence - the
 * "unexpected CGNSSINFO response: ...reports/ESP32_..." garbage seen during
 * hardware testing was exactly that race, back when GNSS still polled on
 * its own independent timer - see gnss.h). Assumes the session is already
 * connected (cellular_task calls cellular_ensure_connected() first) -
 * doesn't attempt to connect, and doesn't update connection state on
 * failure. That's the caller's job (see cellular_task) - keeps this
 * function a pure "try the exchange, report whether it worked".
 */
static bool cellular_do_publish(const char *topic, const char *payload, int qos) {
    size_t topic_len = strlen(topic);
    size_t payload_len = strlen(payload);

    if (!modem_uart_lock(CELLULAR_AT_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "Timed out waiting for UART lock for publish");
        return false;
    }

    // A stale trailing URC from an earlier command (e.g. AT+CMQTTSTART's
    // "OK" can arrive before its "+CMQTTSTART: 0" URC does) could otherwise
    // get misread as part of this sequence's first response.
    modem_uart_flush_input();

    char cmd[64];
    char resp[CELLULAR_AT_RESP_BUF_SIZE];

    snprintf(cmd, sizeof(cmd), "AT+CMQTTTOPIC=%d,%d", CELLULAR_MQTT_CLIENT_INDEX, (int)topic_len);
    if (!cellular_write_at_with_prompt(cmd, topic, topic_len, resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS)) {
        modem_uart_unlock();
        return false;
    }

    snprintf(cmd, sizeof(cmd), "AT+CMQTTPAYLOAD=%d,%d", CELLULAR_MQTT_CLIENT_INDEX, (int)payload_len);
    if (!cellular_write_at_with_prompt(cmd, payload, payload_len, resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS)) {
        modem_uart_unlock();
        return false;
    }

    snprintf(cmd, sizeof(cmd), "AT+CMQTTPUB=%d,%d,60", CELLULAR_MQTT_CLIENT_INDEX, qos);
    if (!cellular_write_at_plain_locked(cmd, resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS)) {
        modem_uart_unlock();
        return false;
    }

    modem_uart_unlock();

    ESP_LOGI(TAG, "Published %d bytes to %s over cellular", (int)payload_len, topic);
    return true;
}

static void cellular_free_pending(cellular_pending_report_t *item) {
    if (item == NULL) {
        return;
    }
    free(item->topic);
    free(item);
}

/**
 * Background task: blocks on the report queue (zero CPU while idle). Each
 * time a report arrives:
 *   1. cellular_ensure_connected() - paced by CELLULAR_RECONNECT_BACKOFF_SEC
 *      internally, so calling it every cycle (~every REPORT_INTERVAL_SEC)
 *      costs nothing extra while backing off (returns fast, no AT traffic).
 *   2. If connected: report_throttle_should_send() (report_throttle.h)
 *      decides whether item->fix/phone_count (already polled by
 *      report_task, once per report cycle - cellular_task itself never
 *      touches GNSS, see gnss.h) differ enough from the last one actually
 *      *sent* to be worth it (or enough time has passed - see config.h's
 *      CELLULAR_POSITION_UNCHANGED_THRESHOLD_M/CELLULAR_HEARTBEAT_INTERVAL_SEC).
 *      If so, build the payload (device_json_build_minimal(), from
 *      item->fix) and publish it. A failure here marks the session down
 *      (cellular_mark_disconnected()) so the next cycle's
 *      cellular_ensure_connected() call retries from scratch.
 * Runs independently of report_task - a slow/failed send here never delays
 * report_task's own loop (print_summary(), the next report cycle, the next
 * GNSS poll).
 */
static void cellular_task(void *pvParameters) {
    (void)pvParameters;
    cellular_pending_report_t *item = NULL;

    while (1) {
        if (xQueueReceive(s_report_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (cellular_ensure_connected()) {
            if (!report_throttle_should_send(&s_throttle, &item->fix, item->phone_count)) {
                ESP_LOGI(TAG, "Skipping cellular send: position/count unchanged (last sent %llds ago)",
                         (long long)(time(NULL) - s_throttle.last_sent_time));
            } else {
                char *payload = device_json_build_minimal(item->fix.valid, item->fix.latitude, item->fix.longitude,
                                                            item->fix.fix_time, item->phone_count, s_client_id);
                if (payload == NULL) {
                    ESP_LOGE(TAG, "Failed to build cellular report JSON");
                } else if (strlen(payload) > CELLULAR_MQTT_MAX_PAYLOAD_LEN) {
                    // The modem's documented AT+CMQTTPAYLOAD limit is
                    // firmware-dependent and hasn't been confirmed on this
                    // board yet (see plans/4g-integration.md's "Data usage
                    // considerations") - guard against sending something the
                    // modem would reject or mangle.
                    ESP_LOGW(TAG, "Payload too large for cellular MQTT (%d > %d bytes) - skipping",
                             (int)strlen(payload), CELLULAR_MQTT_MAX_PAYLOAD_LEN);
                } else if (!cellular_do_publish(item->topic, payload, item->qos)) {
                    cellular_mark_disconnected();
                } else {
                    // Record what actually went out -
                    // report_throttle_should_send() compares the *next*
                    // cycle's fix/phone_count against this, not against
                    // whatever the previous cycle merely considered.
                    report_throttle_record_sent(&s_throttle, &item->fix, item->phone_count);
                }
                free(payload);
            }
        }
        // else: cellular_ensure_connected() already logged its own reason
        // (backing off / bring-up failed) - this report is simply dropped,
        // same as any other un-sent report in this freshest-only design;
        // the next queued report (~REPORT_INTERVAL_SEC later) tries again.

        cellular_free_pending(item);
        item = NULL;
    }
}

bool cellular_start_task(const char *client_id) {
    strncpy(s_client_id, client_id, sizeof(s_client_id) - 1);
    s_client_id[sizeof(s_client_id) - 1] = '\0';

    // UART setup and the initial "AT" handshake are shared with gnss.c over
    // the modem's single AT command port - see modem_uart.h. Safe to call
    // even if gnss_init() already brought the link up. Still synchronous
    // here (unlike the PDP+MQTT bring-up below) - matches gnss_init()'s
    // existing boot-time wait for the same shared UART, so this doesn't add
    // a new race: whichever of gnss_init()/cellular_start_task() runs first
    // in run_normal_mode() does the real work, the other just gets the
    // cached result.
    if (!modem_uart_init()) {
        return false;
    }

    s_report_queue = xQueueCreate(1, sizeof(cellular_pending_report_t *));
    if (s_report_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create cellular report queue");
        return false;
    }

    // Deliberately does NOT attempt the PDP+MQTT bring-up here, even though
    // this function itself is still synchronous. Doing it here would block
    // whoever calls cellular_start_task() for cellular_ensure_connected()'s
    // worst case (AT+CEREG?/CGDCONT/CMQTTSTART/ACCQ/CONNECT each timing out
    // - CMQTTSTART alone up to 120s per its documented max response time -
    // roughly 170s total) on top of modem_uart_init()'s own up-to-50s
    // AT-sync wait - stalling BLE scanning and all logging for minutes on a
    // boot where the modem doesn't attach quickly. The first real attempt
    // happens lazily on cellular_task's first queued report instead, which
    // runs on its own task, so a slow/failed attempt there costs nothing
    // else. Mirrors mqtt_report_start_task()'s async connect on the WiFi
    // side.
    ESP_LOGI(TAG, "Cellular UART ready, MQTT bring-up deferred to first publish");

    xTaskCreate(cellular_task, "cellular_task", 4096, NULL, 3, NULL);
    return true;
}

bool cellular_is_connected(void) {
    return s_mqtt_connected;
}

bool cellular_publish(const char *topic, int phone_count, gnss_fix_t fix, int qos) {
    if (s_report_queue == NULL) {
        ESP_LOGW(TAG, "cellular_publish() called before cellular_start_task()");
        return false;
    }

    cellular_pending_report_t *item = malloc(sizeof(cellular_pending_report_t));
    char *topic_copy = strdup(topic);
    if (item == NULL || topic_copy == NULL) {
        ESP_LOGE(TAG, "Out of memory queuing cellular report");
        free(item);
        free(topic_copy);
        return false;
    }
    item->topic = topic_copy;
    item->phone_count = phone_count;
    item->fix = fix;
    item->qos = qos;

    // Keep only the freshest report - if one's already queued and hasn't
    // been picked up by cellular_task yet, replace (and free) it rather
    // than blocking or growing a backlog. A stale device count isn't worth
    // sending once a newer one exists.
    cellular_pending_report_t *old = NULL;
    if (xQueueReceive(s_report_queue, &old, 0) == pdTRUE) {
        cellular_free_pending(old);
    }

    if (xQueueSend(s_report_queue, &item, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to queue cellular report");
        cellular_free_pending(item);
        return false;
    }

    return true;
}
