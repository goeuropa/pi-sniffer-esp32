/**
 * Cellular MQTT Reporting (SIM7670G modem-side AT+CMQTT*)
 *
 * Brings up the PDP context against CELLULAR_APN and opens a long-lived MQTT
 * session against MQTT_BROKER_HOST/MQTT_BROKER_PORT using the modem's
 * built-in AT+CMQTT* command set - deliberately not PPP/CMUX/esp_netif_t,
 * see the transport decision in plans/4g-integration.md. Shares the modem's
 * single AT command port with gnss.c via modem_uart.h.
 *
 * The whole PDP+MQTT bring-up sequence and the AT+CMQTT* command set here
 * are written against SIMCom's documented A76XX/SIM76XX MQTT(TCP)
 * application note, which this project hasn't yet had a chance to validate
 * against the actual on-board SIM7670G - treat the exact AT sequence as a
 * first cut that will likely need adjustment once tested on real hardware.
 *
 * Sending runs on its own background task (cellular_task, started by
 * cellular_start_task()), decoupled from whichever task calls
 * cellular_publish() (report_task() in practice). cellular_publish() only
 * hands the latest report off via a length-1 queue and returns immediately -
 * it does not wait for (or reflect) the actual AT+CMQTT* exchange. If a new
 * report arrives before the previous one was sent, it replaces it - only the
 * freshest pending report is ever attempted.
 *
 * cellular_publish() takes a phone count and a GNSS fix, not a finished
 * payload - the actual JSON (device_json_build_minimal()) is built later,
 * inside cellular_task, from that snapshot, right before the actual send.
 * cellular_task itself never touches GNSS: the fix comes from
 * report_task()'s own once-per-cycle gnss_poll_now() call (main.c), the
 * only GNSS poll site in the firmware - see gnss.h.
 *
 * Not every queued report is actually sent over the air: cellular_task's
 * report_throttle_should_send() (report_throttle.h) skips a send when the
 * position hasn't moved more than CELLULAR_POSITION_UNCHANGED_THRESHOLD_M
 * and the phone count hasn't changed since the last report actually sent -
 * but never skips more than CELLULAR_HEARTBEAT_INTERVAL_SEC in a row, so a
 * stationary unit still checks in hourly. Cuts needless AT+CMQTT*
 * traffic/SIM data; WiFi reporting (mqtt_report.c) is unaffected.
 *
 * cellular_pdp_ensure_up() (PDP bring-up only, no MQTT) is also exposed for
 * cellular_http.c (REPORT_TRANSPORT_CELLULAR_HTTP) to reuse - the two
 * transports are mutually exclusive at runtime, so there's no bring-up race
 * between them, just a shared "define+activate the PDP context" step
 * neither needs to duplicate.
 */

#ifndef CELLULAR_H
#define CELLULAR_H

#include <stdbool.h>

#include "gnss.h"

/**
 * Bring up the shared modem UART (if not already done by gnss_init()) and
 * the pending-report queue cellular_publish()/cellular_task use to hand off
 * reports, then start cellular_task. The only cellular-specific call the
 * rest of the firmware needs to make - callers don't need to know anything
 * about cellular init/retry/PDP/MQTT internals beyond this one call.
 *
 * Does NOT attempt the PDP+MQTT bring-up itself before returning - this
 * runs synchronously during boot, and that bring-up sequence's worst-case
 * timeouts (~170s, dominated by AT+CMQTTSTART's documented 120s max) would
 * stall BLE scanning/all logging if attempted here.
 * The first connection attempt happens lazily inside cellular_task, on its
 * first queued report, same as ongoing reconnects - see cellular_publish().
 * @param client_id This sensor's device ID, used as the MQTT client ID
 * @return true if the modem UART, queue, and task all started successfully
 *         (NOT whether cellular has actually connected yet - see above)
 */
bool cellular_start_task(const char *client_id);

/**
 * @return true if the modem currently has a live MQTT session with the broker
 */
bool cellular_is_connected(void);

/**
 * Hand off a report to be published over the modem's AT+CMQTT* MQTT session.
 * Returns as soon as the report is queued - the connect/reconnect (subject
 * to CELLULAR_RECONNECT_BACKOFF_SEC), the should-send decision
 * (report_throttle_should_send(), report_throttle.h), payload build, and
 * AT+CMQTTTOPIC/PAYLOAD/PUB exchange all happen later, asynchronously, on
 * cellular_task. If a report
 * is already queued and not yet sent, it's replaced by this one - only the
 * most recent report is ever attempted, never a backlog.
 * @param phone_count Current phone count (device_summary_t.phones) - the
 *                     actual payload is built later, at send time, not here
 * @param fix GNSS fix to report alongside phone_count, already polled by
 *            the caller (report_task(), once per report cycle - see
 *            gnss.h) - passed by value and copied into the queued item, so
 *            the caller doesn't need it to outlive this call
 * @return true if handed off to the queue - NOT whether it was delivered;
 *         delivery success/failure is logged separately, from cellular_task
 */
bool cellular_publish(const char *topic, int phone_count, gnss_fix_t fix, int qos);

/**
 * Define and activate the PDP context against CELLULAR_APN (AT+CGDCONT +
 * AT+CGACT), if not already up. Idempotent/cheap to call every cycle once
 * up. Shared by cellular_task internally and by cellular_http.c
 * (REPORT_TRANSPORT_CELLULAR_HTTP) - the MQTT-specific bring-up
 * (AT+CMQTTSTART/ACCQ/CONNECT) stays private to this file, only the PDP
 * layer underneath it is exposed.
 * @return true once the PDP context is up (was already up, or just brought
 *         up successfully); false if not yet registered on the network
 *         (AT+CEREG?) or a bring-up AT command failed
 */
bool cellular_pdp_ensure_up(void);

/**
 * Passive read of whether the PDP context is currently up - unlike
 * cellular_pdp_ensure_up(), never attempts to bring it up. For status/log
 * lines only (e.g. cellular_http_is_connected()).
 */
bool cellular_pdp_is_up(void);

#endif // CELLULAR_H
