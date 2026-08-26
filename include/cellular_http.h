/**
 * Cellular HTTP Reporting (SIM7670G modem-side AT+HTTP*), legacy JSON format
 *
 * Alternative to cellular.c's MQTT-over-cellular transport
 * (REPORT_TRANSPORT_CELLULAR_MQTT) for the same metered SIM -
 * REPORT_TRANSPORT_CELLULAR_HTTP instead POSTs the legacy full-device JSON
 * (device_json_build(), see device_json.h - the same shape/endpoint
 * http_client.c's now-unused http_send_devices() used over WiFi) to API_URL
 * using the modem's own AT+HTTP* command set, over the same shared UART as
 * cellular.c's AT+CMQTT* (modem_uart.h) - not esp_http_client, which needs a
 * PPP/TCP-IP stack this project deliberately doesn't bring up over cellular
 * (see plans/4g-integration.md's transport decision).
 *
 * The two cellular transports are mutually exclusive (REPORT_TRANSPORT) -
 * only one of cellular_start_task()/cellular_http_start_task() ever runs -
 * but this module still shares cellular.c's PDP bring-up
 * (cellular_pdp_ensure_up()) rather than duplicating the AT+CGDCONT/CGACT
 * sequence, since that part has nothing MQTT-specific about it.
 *
 * Unlike MQTT, there's no persistent "session" here: each report runs its
 * own AT+HTTPINIT -> ... -> AT+HTTPTERM cycle from scratch (see
 * cellular_http.c), same as http_send_devices()'s one-shot esp_http_client
 * calls did.
 *
 * Sending runs on its own background task (cellular_http_task, started by
 * cellular_http_start_task()), decoupled from whichever task calls
 * cellular_http_publish() (report_task() in practice) - same "hand off via
 * a length-1 queue and return immediately" shape as cellular.c.
 *
 * Because this transport's payload (the full per-device JSON) is much
 * larger than cellular MQTT's minimal one, and building it means locking
 * device_list and iterating every device, report_task() uses
 * cellular_http_should_send() as a cheap pre-check *before* paying for that
 * build - see its own doc comment.
 *
 * The AT+HTTP* sequence here is written against SIMCom's HTTP(S)
 * application note, same "first cut, not yet validated against this exact
 * board" caveat cellular.h documents for AT+CMQTT* - see
 * plans/4g-integration.md.
 */

#ifndef CELLULAR_HTTP_H
#define CELLULAR_HTTP_H

#include <stdbool.h>

#include "gnss.h"

/**
 * Bring up the shared modem UART (if not already done by gnss_init()/
 * cellular_start_task()) and the pending-report queue
 * cellular_http_publish()/cellular_http_task use, then start
 * cellular_http_task. Does NOT attempt PDP bring-up itself before
 * returning - same reasoning as cellular_start_task(): that happens lazily,
 * on the first queued report, so a slow/failed attempt doesn't stall boot.
 * @param client_id This sensor's device ID - logged only; the JSON body's
 *                   own device_id field is already baked in by
 *                   device_json_build() before cellular_http_publish() is
 *                   called
 * @return true if the modem UART, queue, and task all started successfully
 */
bool cellular_http_start_task(const char *client_id);

/**
 * @return true if the PDP context is currently up. There's no persistent
 *         HTTP "session" the way cellular_is_connected() reports one for
 *         MQTT - this is only ever used for report_task()'s "not sent"
 *         diagnostic log line.
 */
bool cellular_http_is_connected(void);

/**
 * Cheap, synchronous pre-check for whether the current fix/phone_count
 * differs enough from the last report actually sent to be worth building a
 * report for at all - see report_throttle.h. Unlike cellular_publish()'s
 * minimal payload (cheap to always build, so cellular.c's own
 * report_throttle_should_send() call happens only inside cellular_task,
 * after queuing), this transport's payload
 * (device_json_build()) requires locking device_list and iterating every
 * device - report_task() calls this *before* paying for that build, so a
 * throttled cycle costs nothing beyond this check. cellular_http_publish()
 * re-checks with the fresher fix right before actually sending, so this
 * pre-check only needs to be approximately right, not authoritative.
 */
bool cellular_http_should_send(int phone_count, gnss_fix_t fix);

/**
 * Hand off a pre-built device_json_build() payload to be POSTed via
 * AT+HTTP*. TAKES OWNERSHIP of json_body - it is freed internally in every
 * case (sent, throttled, or replaced by a fresher report before being
 * picked up). Caller must not free it after this call, regardless of the
 * return value. Returns immediately; the actual PDP bring-up (if needed)
 * and AT+HTTPINIT/PARA/DATA/ACTION/TERM exchange happen later,
 * asynchronously, on cellular_http_task.
 * @param topic Logged only - HTTP has no MQTT topic concept, this is just
 *              for symmetry with cellular_publish()'s log lines
 * @param json_body Heap string from device_json_build() - ownership
 *                   transferred to this call
 * @param phone_count Current phone count, for report_throttle's decision
 * @param fix GNSS fix already polled by the caller, passed by value
 * @return true if handed off to the queue - NOT whether it was delivered
 */
bool cellular_http_publish(const char *topic, char *json_body, int phone_count, gnss_fix_t fix);

#endif // CELLULAR_HTTP_H
