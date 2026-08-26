/**
 * Cellular HTTP Reporting (SIM7670G modem-side AT+HTTP*), legacy JSON format
 *
 * See cellular_http.h.
 */

#include "cellular_http.h"
#include "cellular.h"
#include "config.h"
#include "modem_uart.h"
#include "report_throttle.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"

static const char *TAG = "CELLULAR_HTTP";

#define CELLULAR_HTTP_AT_RESP_BUF_SIZE 256

// A single pending report, handed from cellular_http_publish() (called on
// report_task's schedule) to cellular_http_task (its own background task)
// via s_report_queue. Unlike cellular.c's cellular_pending_report_t, the
// JSON body itself is already built (device_json_build() needs
// device_list_t under lock - see cellular_http_should_send()'s doc comment
// for why that build has to happen in report_task(), not here) - this
// struct just carries it across the queue and owns it until sent or
// dropped.
typedef struct {
    char *topic;      // logged only, see cellular_http_publish()
    char *json_body;  // owned - device_json_build() output, freed after use
    int phone_count;
    gnss_fix_t fix;
} cellular_http_pending_report_t;

// Length 1: only the freshest un-sent report is ever kept - see
// cellular_http_publish()'s "replace, don't queue" handling.
static QueueHandle_t s_report_queue = NULL;

// Own reconnect backoff, independent of cellular.c's - the two transports
// are mutually exclusive at runtime, so there's no coordination needed, but
// each still needs its own pacing state.
static time_t s_next_reconnect_attempt = 0;

// State of the last report actually *sent* - see report_throttle.h. Own
// instance, independent of cellular.c's (mutually exclusive transports).
static report_throttle_state_t s_throttle;

/**
 * Write a plain AT command and read its response, assuming the UART lock is
 * already held (see cellular_http_do_post()). Modeled on cellular.c's
 * cellular_write_at_plain_locked().
 */
static bool cellular_http_at_locked(const char *cmd, char *resp, size_t resp_size, int timeout_ms) {
    char line[192];
    int n = snprintf(line, sizeof(line), "%s\r\n", cmd);
    modem_uart_write_raw(line, n);

    int got = modem_uart_read_response(resp, resp_size, timeout_ms);
    if (got <= 0 || strstr(resp, "OK") == NULL) {
        ESP_LOGW(TAG, "%s failed: %s", cmd, got > 0 ? resp : "(no response)");
        return false;
    }
    return true;
}

/**
 * AT+HTTPDATA's two-phase exchange: command line -> wait for the modem's
 * "ready for data" token -> raw body bytes -> OK. Modeled on cellular.c's
 * cellular_write_at_with_prompt(), generalized to whatever token the
 * command actually waits for (SIM7670G's AT+HTTPDATA is documented
 * (SIMCom HTTP(S) application note) to send "DOWNLOAD" here, not the bare
 * ">" AT+CMQTTTOPIC/PAYLOAD use - unconfirmed on this exact board, same
 * "needs on-device verification" caveat as the rest of this file, see
 * cellular_http.h). Assumes the UART lock is already held.
 */
static bool cellular_http_write_data_locked(const char *at_cmd, const char *token,
                                             const char *data, size_t data_len,
                                             char *resp, size_t resp_size, int timeout_ms) {
    char line[64];
    int n = snprintf(line, sizeof(line), "%s\r\n", at_cmd);
    modem_uart_write_raw(line, n);

    if (!modem_uart_wait_for_token(resp, resp_size, timeout_ms, token)) {
        ESP_LOGW(TAG, "%s: no '%s' prompt: %s", at_cmd, token, resp);
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
 * Parse the HTTP status code out of an "+HTTPACTION: <method>,<status>,
 * <datalen>" URC line. Same atoi(comma + 1) style as
 * cellular.c's cellular_network_registered() - atoi() stops at the next
 * comma on its own, no need to isolate the field first.
 * @return the parsed status code, or -1 if the line couldn't be parsed
 */
static int cellular_http_parse_action_status(const char *resp) {
    const char *p = strstr(resp, "+HTTPACTION:");
    if (p == NULL) {
        return -1;
    }
    const char *comma = strchr(p, ',');
    if (comma == NULL) {
        return -1;
    }
    return atoi(comma + 1);
}

/**
 * The actual POST: the full AT+HTTPINIT..AT+HTTPACTION..AT+HTTPTERM
 * exchange, held under one continuous UART lock (same reasoning as
 * cellular.c's cellular_do_publish() - avoids gnss.c's AT+CGNSSINFO poll
 * interleaving mid-exchange, including across the potentially long wait for
 * the +HTTPACTION URC). Always attempts AT+HTTPTERM before returning,
 * whether or not everything before it succeeded, so a failure partway
 * through never leaves the modem's HTTP service stuck open for the next
 * attempt - same lesson cellular_mqtt_session_connect()'s defensive
 * teardown already encodes for AT+CMQTT*.
 * Assumes the PDP context is already up (cellular_http_task calls
 * cellular_pdp_ensure_up() first) - doesn't bring it up itself.
 */
static bool cellular_http_do_post(const char *json_body) {
    size_t body_len = strlen(json_body);
    char cmd[256];
    char resp[CELLULAR_HTTP_AT_RESP_BUF_SIZE];
    bool ok = true;
    int status_code = -1;

    if (!modem_uart_lock(CELLULAR_AT_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "Timed out waiting for UART lock for HTTP POST");
        return false;
    }

    // A stale trailing URC from an earlier command could otherwise get
    // misread as part of this sequence's first response - same reasoning as
    // cellular_do_publish()'s equivalent flush.
    modem_uart_flush_input();

    // Defensive teardown before init, in case a previous attempt (or a
    // previous ESP32 boot - the modem has its own power/reset domain,
    // independent of the ESP32's) left the HTTP service open. Best-effort,
    // result ignored - "nothing to tear down" just errors harmlessly.
    cellular_http_at_locked("AT+HTTPTERM", resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS);

    if (!cellular_http_at_locked("AT+HTTPINIT", resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS)) {
        ok = false;
    }

    if (ok && !cellular_http_at_locked("AT+HTTPPARA=\"CID\",1", resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS)) {
        ok = false;
    }

#if API_SKIP_CERT_CHECK
    // Skip TLS certificate verification on the modem side, mirroring
    // API_SKIP_CERT_CHECK's existing (previously unused) intent for the
    // WiFi/esp_http_client path. Exact AT+CSSLCFG syntax/semantics are
    // unconfirmed on this board - best-effort, warn rather than abort, since
    // a wrong command here shouldn't block an otherwise-working plain-HTTP
    // deployment.
    if (ok && strncmp(API_URL, "https://", 8) == 0) {
        if (!cellular_http_at_locked("AT+CSSLCFG=\"authmode\",0,0", resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "AT+CSSLCFG failed (needs on-device verification) - continuing anyway");
        }
        if (!cellular_http_at_locked("AT+HTTPSSL=1", resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "AT+HTTPSSL=1 failed - HTTPS POST may fail");
        }
    }
#endif

    if (ok) {
        snprintf(cmd, sizeof(cmd), "AT+HTTPPARA=\"URL\",\"%s\"", API_URL);
        if (!cellular_http_at_locked(cmd, resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS)) {
            ok = false;
        }
    }

    if (ok && !cellular_http_at_locked("AT+HTTPPARA=\"CONTENT\",\"application/json\"",
                                        resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS)) {
        ok = false;
    }

    if (ok) {
        snprintf(cmd, sizeof(cmd), "AT+HTTPDATA=%d,%d", (int)body_len, CELLULAR_AT_TIMEOUT_MS / 1000);
        if (!cellular_http_write_data_locked(cmd, "DOWNLOAD", json_body, body_len,
                                              resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS)) {
            ok = false;
        }
    }

    if (ok && !cellular_http_at_locked("AT+HTTPACTION=1", resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS)) {
        ok = false;
    }

    if (ok) {
        // AT+HTTPACTION's own OK (just consumed above) arrives immediately -
        // the actual result is this later, unsolicited "+HTTPACTION: ..."
        // URC, once the HTTP request itself completes against the far end.
        if (!modem_uart_wait_for_urc(resp, sizeof(resp), CELLULAR_HTTP_ACTION_TIMEOUT_MS, "+HTTPACTION:")) {
            ESP_LOGW(TAG, "No +HTTPACTION response within %dms", CELLULAR_HTTP_ACTION_TIMEOUT_MS);
            ok = false;
        } else {
            status_code = cellular_http_parse_action_status(resp);
            if (status_code < 200 || status_code >= 300) {
                ESP_LOGW(TAG, "AT+HTTPACTION reported HTTP status %d: %s", status_code, resp);
                ok = false;
            }
        }
    }

    // Always terminate, regardless of where the sequence above failed (if
    // it did) - see this function's own doc comment.
    cellular_http_at_locked("AT+HTTPTERM", resp, sizeof(resp), CELLULAR_AT_TIMEOUT_MS);

    modem_uart_unlock();

    if (ok) {
        ESP_LOGI(TAG, "Posted %d bytes to %s over cellular HTTP (status %d)",
                 (int)body_len, API_URL, status_code);
    }
    return ok;
}

static void cellular_http_free_pending(cellular_http_pending_report_t *item) {
    if (item == NULL) {
        return;
    }
    free(item->topic);
    free(item->json_body);
    free(item);
}

/**
 * Bring up the PDP context, paced by s_next_reconnect_attempt - same
 * backoff idiom as cellular.c's cellular_ensure_connected(), just against
 * cellular_pdp_ensure_up() (shared with cellular.c) instead of the full
 * MQTT session bring-up. Cheap to call every cycle once up (idempotent).
 */
static bool cellular_http_ensure_pdp_up(void) {
    if (time(NULL) < s_next_reconnect_attempt) {
        ESP_LOGI(TAG, "Still backing off, retrying cellular HTTP PDP bring-up in %llds",
                 (long long)(s_next_reconnect_attempt - time(NULL)));
        return false;
    }

    if (!cellular_pdp_ensure_up()) {
        s_next_reconnect_attempt = time(NULL) + CELLULAR_RECONNECT_BACKOFF_SEC;
        ESP_LOGW(TAG, "Cellular HTTP PDP bring-up failed, backing off %ds before retrying",
                 CELLULAR_RECONNECT_BACKOFF_SEC);
        return false;
    }

    return true;
}

/**
 * Background task: blocks on the report queue (zero CPU while idle). Each
 * time a report arrives:
 *   1. cellular_http_ensure_pdp_up() - paced internally, cheap while
 *      backing off.
 *   2. If up: report_throttle_should_send() re-checks item->fix/
 *      phone_count against the last one actually sent (report_task()
 *      already did this same check with a slightly staler fix before
 *      paying for device_json_build() - see cellular_http_should_send()'s
 *      doc comment - this is the authoritative re-check).
 *   3. If still worth sending: cellular_http_do_post(). A failure here sets
 *      a fresh backoff window, same as cellular.c's cellular_mark_disconnected()
 *      does for MQTT - it does NOT reset the shared PDP-up flag, since an
 *      HTTP-layer failure doesn't necessarily mean the PDP context itself
 *      dropped (only cellular_pdp_ensure_up() failing on a later call
 *      should conclude that).
 * item->json_body is freed unconditionally afterwards (sent, throttled, or
 * failed) - cellular_http_publish() transferred ownership when it queued
 * this item.
 */
static void cellular_http_task(void *pvParameters) {
    (void)pvParameters;
    cellular_http_pending_report_t *item = NULL;

    while (1) {
        if (xQueueReceive(s_report_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (cellular_http_ensure_pdp_up()) {
            if (!report_throttle_should_send(&s_throttle, &item->fix, item->phone_count)) {
                ESP_LOGI(TAG, "Skipping cellular HTTP send: position/count unchanged (last sent %llds ago)",
                         (long long)(time(NULL) - s_throttle.last_sent_time));
            } else if (!cellular_http_do_post(item->json_body)) {
                s_next_reconnect_attempt = time(NULL) + CELLULAR_RECONNECT_BACKOFF_SEC;
                ESP_LOGW(TAG, "Cellular HTTP POST failed, backing off %ds before retrying",
                         CELLULAR_RECONNECT_BACKOFF_SEC);
            } else {
                report_throttle_record_sent(&s_throttle, &item->fix, item->phone_count);
            }
        }
        // else: cellular_http_ensure_pdp_up() already logged its own reason
        // (backing off / bring-up failed) - this report is simply dropped,
        // same as cellular.c's equivalent path; the next queued report
        // (~REPORT_INTERVAL_SEC later) tries again.

        cellular_http_free_pending(item);
        item = NULL;
    }
}

bool cellular_http_start_task(const char *client_id) {
    // UART setup and the initial "AT" handshake are shared with gnss.c/
    // cellular.c over the modem's single AT command port - see
    // modem_uart.h. Safe to call even if already brought up by one of them.
    if (!modem_uart_init()) {
        return false;
    }

    s_report_queue = xQueueCreate(1, sizeof(cellular_http_pending_report_t *));
    if (s_report_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create cellular HTTP report queue");
        return false;
    }

    // Deliberately does NOT attempt PDP bring-up here - same reasoning as
    // cellular_start_task(): the first real attempt happens lazily on
    // cellular_http_task's first queued report instead, so a slow/failed
    // attempt never stalls boot.
    ESP_LOGI(TAG, "Cellular HTTP UART ready for %s, PDP bring-up deferred to first publish", client_id);

    xTaskCreate(cellular_http_task, "cellular_http_task", 4096, NULL, 3, NULL);
    return true;
}

bool cellular_http_is_connected(void) {
    return cellular_pdp_is_up();
}

bool cellular_http_should_send(int phone_count, gnss_fix_t fix) {
    return report_throttle_should_send(&s_throttle, &fix, phone_count);
}

bool cellular_http_publish(const char *topic, char *json_body, int phone_count, gnss_fix_t fix) {
    if (json_body == NULL) {
        return false;
    }

    if (s_report_queue == NULL) {
        ESP_LOGW(TAG, "cellular_http_publish() called before cellular_http_start_task()");
        free(json_body);
        return false;
    }

    cellular_http_pending_report_t *item = malloc(sizeof(cellular_http_pending_report_t));
    char *topic_copy = (topic != NULL) ? strdup(topic) : NULL;
    if (item == NULL) {
        ESP_LOGE(TAG, "Out of memory queuing cellular HTTP report");
        free(topic_copy);
        free(json_body);
        return false;
    }
    item->topic = topic_copy;
    item->json_body = json_body; // ownership transferred
    item->phone_count = phone_count;
    item->fix = fix;

    // Keep only the freshest report - same "replace, don't queue" handling
    // as cellular.c's cellular_publish().
    cellular_http_pending_report_t *old = NULL;
    if (xQueueReceive(s_report_queue, &old, 0) == pdTRUE) {
        cellular_http_free_pending(old);
    }

    if (xQueueSend(s_report_queue, &item, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to queue cellular HTTP report");
        cellular_http_free_pending(item);
        return false;
    }

    return true;
}
