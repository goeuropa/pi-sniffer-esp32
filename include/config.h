/**
 * ESP32 BLE Sniffer Configuration
 * Modify these settings to match your environment
 */

#ifndef CONFIG_H
#define CONFIG_H

// Deployment-specific network endpoints (API_URL, MQTT_BROKER_*,
// CELLULAR_APN) live in secrets.h, which is gitignored - not checked into
// source control. If this doesn't compile because secrets.h is missing,
// copy include/secrets.h.example to include/secrets.h and fill in your own
// values.
#include "secrets.h"

// ============================================================================
// WiFi Configuration
// ============================================================================
// WiFi credentials are now configured via captive portal
// On first boot or double-press BOOT button, device creates "DC Sniffer" AP
// Connect to it and open http://192.168.4.1 to configure WiFi

#define WIFI_MAX_RETRY      10

// ============================================================================
// Reporting Configuration
// ============================================================================
// Which transport report_task() publishes device reports over, selected per
// deployment - some units have WiFi, some are permanently off-grid on
// cellular only. Exactly one of these ever runs: run_normal_mode() only
// starts the one background task (mqtt_report_task, cellular_task, or
// cellular_http_task) this setting actually needs, never more than one (see
// plans/4g-integration.md's "Resolved decisions").
typedef enum {
    REPORT_TRANSPORT_WIFI,
    REPORT_TRANSPORT_CELLULAR_MQTT,      // MQTT over cellular (AT+CMQTT*, cellular.c)
    REPORT_TRANSPORT_CELLULAR_HTTP,      // HTTP POST over cellular (AT+HTTP*, cellular_http.c) -
                                          // legacy full-device JSON format, see cellular_http.h
} report_transport_t;

#define REPORT_TRANSPORT           REPORT_TRANSPORT_CELLULAR_HTTP

// Disable sending device reports entirely (set to 1 to disable, 0 to
// enable) - applies to whichever REPORT_TRANSPORT is selected above, not
// just one transport.
#define DISABLE_API_SEND    0

// MQTT broker - both the WiFi path (esp-mqtt, needs a full URI) and the
// cellular path (AT+CMQTTCONNECT, needs host/port split out) connect here.
// Publicly reachable hostname (not a LAN-only address) - needed for the
// cellular path, which isn't on the same network as a local broker would
// be. Plain (unencrypted) MQTT on the standard port for now; TLS/auth is
// still an open question, see plans/4g-integration.md.
// MQTT_BROKER_HOST/PORT/URI are defined in secrets.h (gitignored) - see
// this file's top comment.

// Full publish topic is "<prefix>/<device_id>" - one topic per unit, same
// JSON body device_json_build() produces today regardless of transport.
#define MQTT_TOPIC_PREFIX           "reports"
#define MQTT_QOS                    1
#define MQTT_KEEPALIVE_SEC          60

// How often mqtt_report_task's outer loop re-kicks wifi_connect() once
// wifi_manager.c's own fast burst-retry (WIFI_MAX_RETRY, back-to-back) has
// already given up. A brief WiFi blip still recovers in seconds via that
// existing fast retry, unchanged - this is only the paced fallback for a
// genuinely down AP/outage, so the device doesn't sit permanently stuck the
// way it could before (wifi_manager.c never retried again on its own).
#define WIFI_RECONNECT_INTERVAL_SEC 300

// ============================================================================
// Cellular Configuration (SIM7670G modem-side AT+CMQTT*)
// ============================================================================
// Enable the cellular MQTT transport (cellular.c). Independent of
// ENABLE_GNSS - both share the modem's UART via modem_uart.h - but there's
// no point bringing up the modem's data connection on boards/deployments
// that only need WiFi.
#define ENABLE_CELLULAR              1

// CELLULAR_APN is defined in secrets.h (gitignored) - see this file's top
// comment. No PDP username/password required for this deployment's SIM.

#define CELLULAR_AT_TIMEOUT_MS       10000
#define CELLULAR_MQTT_CLIENT_INDEX   0

// AT+CMQTTSTART's documented max response time is 12000ms per the
// SIM7672X/SIM7652X MQTT(S) Application Note (the closest board-specific
// reference found - hosted on Waveshare's own wiki page for this board).
// An earlier version of this comment cited 120000ms from the older
// SIM7500/7600/7800 MQTT AT command manual, which isn't specific to this
// chip - corrected. Still longer than CELLULAR_AT_TIMEOUT_MS since it can
// activate the PDP context on a cold/weak-signal attach.
#define CELLULAR_CMQTTSTART_TIMEOUT_MS 12000

// Outer-loop reconnect pacing, matching WIFI_RECONNECT_INTERVAL_SEC - cellular
// has no independent fast-retry layer the way WiFi's event handler provides
// (nothing signals a drop between publish attempts), so this is the only
// reconnect pacing cellular_task has.
#define CELLULAR_RECONNECT_BACKOFF_SEC 300

// Conservative placeholder cap on a single AT+CMQTTPAYLOAD - the modem's
// actual documented/firmware limit hasn't been confirmed on this board yet.
// See "Data usage considerations" in plans/4g-integration.md - a full
// MAX_DEVICES report can be tens of KB, likely needing payload
// shrinking/compression for cellular-connected units well before this cap
// is the binding constraint.
#define CELLULAR_MQTT_MAX_PAYLOAD_LEN 4096

// Skip a cellular send when nothing meaningful changed since the last one
// actually sent - phone count identical and position (gnss_fix_t.latitude/
// longitude, the *raw* unrounded floats, not the ~11m-rounded values that
// go into the payload - see device_json_build_minimal()) moved less than
// CELLULAR_POSITION_UNCHANGED_THRESHOLD_M - to cut needless AT+CMQTT*
// traffic (and SIM data) for a stationary, steady-count unit. Still forces
// a send at least every CELLULAR_HEARTBEAT_INTERVAL_SEC regardless, so a
// unit that never moves/changes still proves it's alive on a predictable
// cadence rather than going silent indefinitely. WiFi reporting is
// unaffected - see report_throttle_should_send() in report_throttle.c.
//
// 5m (the original value) turned out to be inside the SIM7670G's own
// stationary GNSS noise floor - confirmed on hardware: consecutive sends
// with *identical* rounded lat/lon (e.g. two reports both showing
// 47.5853,-122.0644 a few minutes apart) still triggered a send, meaning
// the raw fix wandered past 5m and back within the same ~11m rounding
// bucket purely from receiver jitter, not real movement. Raised well above
// that noise floor so it only fires on an actual position change.
#define CELLULAR_POSITION_UNCHANGED_THRESHOLD_M 20.0f
#define CELLULAR_HEARTBEAT_INTERVAL_SEC          3600

// ----------------------------------------------------------------------
// Cellular HTTP reporting (SIM7670G modem-side AT+HTTP*, cellular_http.c) -
// alternative to REPORT_TRANSPORT_CELLULAR_MQTT (MQTT) above for the same
// metered SIM. POSTs the legacy full-device JSON (device_json_build(), see
// device_json.h) to API_URL (defined in secrets.h, gitignored - see this
// file's top comment), using the modem's own AT+HTTP* command set instead
// of esp_http_client (which needs a PPP/TCP-IP stack this project
// deliberately doesn't bring up over cellular - see
// plans/4g-integration.md's transport decision). API_URL/API_TIMEOUT_MS are
// also still referenced by http_client.c's now-unused WiFi/esp_http_client
// path (http_send_devices(), no longer called by report_task()) - kept as
// one shared endpoint/timeout rather than duplicated, since both are
// posting the same JSON shape to the same place. Throttled by the same
// CELLULAR_POSITION_UNCHANGED_THRESHOLD_M/CELLULAR_HEARTBEAT_INTERVAL_SEC
// above (see report_throttle.h) - the legacy payload is much larger than
// cellular MQTT's minimal one, so sending it unconditionally every
// REPORT_INTERVAL_SEC isn't viable on a metered plan.
#define API_TIMEOUT_MS      10000
// Skip SSL certificate verification (e.g. for self-signed certs) - honored
// by cellular_http.c (AT+CSSLCFG); http_client.c's dead WiFi path always
// verifies via esp_crt_bundle_attach and ignores this.
#define API_SKIP_CERT_CHECK 1

// AT+HTTPACTION's result arrives later as an unsolicited "+HTTPACTION" URC,
// not synchronously with the command's own OK - this bounds how long
// cellular_http_task waits for that URC before giving up. Needs on-device
// verification, same as CELLULAR_CMQTTSTART_TIMEOUT_MS was - starting
// conservative per SIMCom's HTTP(S) application note's documented worst case
// (a slow far-end server/DNS lookup).
#define CELLULAR_HTTP_ACTION_TIMEOUT_MS 120000

// ============================================================================
// BLE Scanning Configuration
// ============================================================================
// Duration of each scan cycle in seconds
#define SCAN_DURATION_SEC       10

// How often to send data to the API (seconds)
#define REPORT_INTERVAL_SEC     30

// Maximum number of devices to track
#define MAX_DEVICES             128

// Maximum age before removing a device from tracking (seconds)
#define MAX_DEVICE_AGE_SEC      300

// A device needs at least this many observations before it counts toward
// device_get_summary()'s totals/categories. A single stray packet (seen
// once, then never again) is weak evidence of a real nearby device - within
// one SCAN_DURATION_SEC window a genuinely present device is almost always
// seen more than once, given typical BLE advertising intervals - so
// requiring a second sighting filters out one-off noise without meaningfully
// delaying when a real device gets counted. Still shown in the per-device
// log listing either way, just excluded from the category counts.
#define DEVICE_MIN_SEEN_COUNT_FOR_SUMMARY 2

// A device whose most recent RSSI is weaker (more negative) than this dBm
// value is excluded from device_get_summary()'s totals/categories. Signals
// this weak are at or below typical BLE receiver sensitivity (roughly -95 to
// -100 dBm), where bit errors are common and a misdecoded MAC address can
// masquerade as a brand-new "device" - in practice showing up as a
// low-confidence chain of single-packet sightings that inflates the phone
// count. Still shown in the per-device log listing either way, just
// excluded from the category counts.
#define MIN_RSSI_FOR_SUMMARY -105

// ============================================================================
// RSSI to Distance Configuration
// ============================================================================
// RSSI value at 1 meter distance (calibrate for your environment)
// Typical values: -50 to -70 dBm
#define RSSI_ONE_METER          -80

// Path loss exponent (2.0-4.0)
// Lower = indoor/cluttered, Higher = outdoor/open
// Typical: 2.0-2.5 indoor, 3.0-4.0 outdoor
#define PATH_LOSS_EXPONENT      2.5f

// Maximum reasonable distance in meters (cap unrealistic values)
#define MAX_DISTANCE_METERS     50.0f

// Distance-bucket boundaries (meters) for the per-category breakdown in
// print_summary() (main.c) - see device_get_distance_summary() in device.c.
// Buckets are [0, DISTANCE_BUCKET_NEAR_M), [DISTANCE_BUCKET_NEAR_M,
// DISTANCE_BUCKET_MID_M), [DISTANCE_BUCKET_MID_M, +inf).
#define DISTANCE_BUCKET_NEAR_M   5.0f
#define DISTANCE_BUCKET_MID_M    10.0f

// ============================================================================
// Device Identification
// ============================================================================
// Enable name resolution via BLE scan response
#define ENABLE_NAME_RESOLUTION  1

// Enable device categorization based on manufacturer data
#define ENABLE_CATEGORIZATION   1

// Decode Apple's Continuity protocol manufacturer data (AirPods, HomeKit,
// Handoff, Nearby Info, etc.) for better name/category detection
#define ENABLE_APPLE_HEURISTICS 1

// ============================================================================
// MAC-Rotation Packing Configuration
// ============================================================================
// Detect when a device's BLE MAC address has likely rotated (iOS/Android
// random-MAC privacy feature) so the same physical device isn't double
// counted as two separate devices.
#define ENABLE_MAC_PACKING             1

// Below this gap (seconds), a lone single-observation device is treated as
// noise from the same burst rather than a real device.
#define MAC_PACK_BLIP_MIN_GAP_SEC      2

// Above this gap (seconds), a lone single-observation device is treated as
// unrelated rather than a MAC rotation.
#define MAC_PACK_BLIP_MAX_GAP_SEC      90

// Time constant (seconds) for the exponential decay used to turn the gap
// between an old device's last sighting and a new device's first sighting
// into a confidence score.
#define MAC_PACK_TIME_CONSTANT_SEC     30.0f

// Minimum confidence required to record a superseded-by link.
#define MAC_PACK_PROBABILITY_THRESHOLD 0.05f

// Minimum number of matching bytes required for mac_pack_payload_matches()
// to treat two devices' identical raw stream payload as decisive
// same-device evidence. Guards against matching on short/low-entropy
// payloads (e.g. a 1-byte status code that different real devices could
// plausibly share by coincidence) - 3+ bytes of matching pseudo-random
// session/auth content is a much safer bar.
#define MAC_PACK_MIN_PAYLOAD_MATCH_LEN 3

// Maximum number of distinct Apple Continuity message types tracked
// concurrently per device (see stream_window_t in device.h). Sized to the
// number of Continuity streams a single iPhone/iPad realistically emits at
// once (Nearby Info, Proximity Pairing, Instant Hotspot, HomeKit, etc.);
// when full, the least-recently-seen type is evicted to make room for a
// new one.
#define MAC_PACK_MAX_TRACKED_STREAMS    4

// ============================================================================
// GNSS Configuration (SIM7670G onboard modem)
// ============================================================================
// Poll the SIM7670G's GNSS receiver over UART via AT commands and log the
// fix (lat/long/height/time) on a regular interval.
#define ENABLE_GNSS              1

// gnss_poll_now() is called exactly once per report cycle, synchronously,
// from report_task() (main.c) - there's no independent GNSS polling task,
// so this happens on report_task's own thread, once every
// REPORT_INTERVAL_SEC, whichever transport (WiFi or cellular) is actually
// configured to send.

// UART peripheral and pins used to talk to the modem's AT command port.
// Board docs give GPIO17/18 for this link but swapped from our original
// guess: GPIO18 is the ESP32-S3's TX (into the modem's RX), GPIO17 is the
// ESP32-S3's RX (from the modem's TX). UART_NUM_1, 115200 baud, 1024-byte
// RX buffer confirmed correct.
#define GNSS_UART_NUM            UART_NUM_1
#define GNSS_UART_TX_PIN         18   // ESP32-S3 -> modem RX
#define GNSS_UART_RX_PIN         17   // modem TX -> ESP32-S3
#define GNSS_UART_BAUD           115200

// Time to let the modem finish booting before sending the first AT command
// (seconds). Combo LTE+GNSS modems can be slow off a cold boot - this is
// just the *initial* wait; GNSS_AT_SYNC_ATTEMPTS/GNSS_AT_SYNC_RETRY_MS below
// add further patience on top of it.
#define GNSS_POWERON_WAIT_SEC    10

// How many times to retry "AT", GNSS_AT_SYNC_RETRY_MS apart, before giving
// up on the modem during gnss_init(). At the defaults this allows up to
// GNSS_POWERON_WAIT_SEC + (20 * 2s) = ~50s total for the modem to answer.
#define GNSS_AT_SYNC_ATTEMPTS    20
#define GNSS_AT_SYNC_RETRY_MS    2000

// Discipline the ESP32's system clock (settimeofday()) from each GNSS fix's
// own UTC date/time, so units with no WiFi/SNTP still get a correct wall
// clock - see gnss_poll_now()'s system-clock-sync step. Skip the actual
// settimeofday() call (and its log line) if the system clock is already
// within this many seconds of the GPS time, so a clock already kept
// accurate (e.g. by SNTP) isn't needlessly nudged every report cycle.
#define GNSS_CLOCK_SYNC_THRESHOLD_SEC 2

// ============================================================================
// Debugging
// ============================================================================
// Log individual device discoveries - the per-packet "Device: ..." line in
// on_device_discovered() (main.c). Off by default: at any real device
// density this is by far the noisiest log source, one line per BLE
// advertisement seen. Independent of ble_scanner.c's own (lower-level, less
// detailed) discovery line, which this same flag also gates.
#define DEBUG_DEVICE_DISCOVERY  0

// Append each device's per-Continuity-type stream windows (see
// stream_window_t in device.h) to its line in the device summary, e.g.
// "streams: 0c[150s-90s] 10[150s-1s]". Lets you directly see, for two
// devices that mac_pack chose not to link, whether they share a stream
// type with overlapping activity (a real conflict) or genuinely disjoint
// stream types/windows (a possible missed ragged-handover link) - useful
// when investigating a device count that looks too high or too low.
#define DEBUG_STREAM_WINDOWS    1

// Only log devices categorized as CATEGORY_PHONE - suppresses the per-scan
// "Device: ..." discovery line (main.c) and the per-device row in the
// summary listing (main.c) for everything else (tvs, beacons, tablets,
// appliances, unknown, etc.), for a much shorter log when all you care about
// is phones. Does not affect the summary's aggregate category counts
// (Total/Phones/Computers/.../Beacons/Other) - those stay in the log as a
// compact crowd overview regardless of this setting.
#define PHONES_ONLY              0

#endif // CONFIG_H
