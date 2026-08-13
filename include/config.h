/**
 * ESP32 BLE Sniffer Configuration
 * Modify these settings to match your environment
 */

#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// WiFi Configuration
// ============================================================================
// WiFi credentials are now configured via captive portal
// On first boot or double-press BOOT button, device creates "DC Sniffer" AP
// Connect to it and open http://192.168.4.1 to configure WiFi

#define WIFI_MAX_RETRY      10

// ============================================================================
#define API_URL             "https://3334.xomnghien.com/api/devices"
#define API_TIMEOUT_MS      10000
// Skip SSL certificate verification (e.g. for self-signed certs)
#define API_SKIP_CERT_CHECK 1
// Disable sending device data to the API (set to 1 to disable, 0 to enable)
#define DISABLE_API_SEND    1

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
#define MIN_RSSI_FOR_SUMMARY -100

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

// How often to poll AT+CGNSSINFO and log the result (seconds).
#define GNSS_POLL_INTERVAL_SEC   30

// ============================================================================
// Debugging
// ============================================================================
// Enable verbose logging
#define DEBUG_LOGGING           1

// Log individual device discoveries
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
