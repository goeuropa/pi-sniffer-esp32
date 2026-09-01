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

#define REPORT_TRANSPORT           REPORT_TRANSPORT_WIFI

// Disable sending device reports entirely (set to 1 to disable, 0 to enable)
#define DISABLE_API_SEND    0

// Default API endpoint placeholder if needed by legacy HTTP implementations
#ifndef API_URL
#define API_URL             "your_api_endpoint_here"
#endif

// MQTT broker - both the WiFi path (esp-mqtt, needs a full URI) and the
// cellular path (AT+CMQTTCONNECT, needs host/port split out) connect here.
#define MQTT_TOPIC_PREFIX           "reports"
#define MQTT_QOS                    1
#define MQTT_KEEPALIVE_SEC          60

#define WIFI_RECONNECT_INTERVAL_SEC 300

// ============================================================================
// Cellular Configuration (SIM7670G modem-side AT+CMQTT*)
// ============================================================================
#define ENABLE_CELLULAR             1
#define CELLULAR_AT_TIMEOUT_MS       10000
#define CELLULAR_MQTT_CLIENT_INDEX   0
#define CELLULAR_CMQTTSTART_TIMEOUT_MS 12000
#define CELLULAR_RECONNECT_BACKOFF_SEC 300
#define CELLULAR_MQTT_MAX_PAYLOAD_LEN 4096
#define CELLULAR_POSITION_UNCHANGED_THRESHOLD_M 20.0f
#define CELLULAR_HEARTBEAT_INTERVAL_SEC          3600

#define API_TIMEOUT_MS      10000
#define API_SKIP_CERT_CHECK 1
#define CELLULAR_HTTP_ACTION_TIMEOUT_MS 120000

// ============================================================================
// BLE Scanning Configuration
// ============================================================================
#define SCAN_DURATION_SEC       10
#define REPORT_INTERVAL_SEC     30
#define MAX_DEVICES             128
#define MAX_DEVICE_AGE_SEC      300
#define DEVICE_MIN_SEEN_COUNT_FOR_SUMMARY 2
#define MIN_RSSI_FOR_SUMMARY -105

// ============================================================================
// RSSI to Distance Configuration
// ============================================================================
#define RSSI_ONE_METER          -80
#define PATH_LOSS_EXPONENT      2.5f
#define MAX_DISTANCE_METERS     50.0f
#define DISTANCE_BUCKET_NEAR_M   5.0f
#define DISTANCE_BUCKET_MID_M    10.0f

// ============================================================================
// Device Identification
// ============================================================================
#define ENABLE_NAME_RESOLUTION  1
#define ENABLE_CATEGORIZATION   1
#define ENABLE_APPLE_HEURISTICS 1

// ============================================================================
// MAC-Rotation Packing Configuration
// ============================================================================
#define ENABLE_MAC_PACKING             1
#define MAC_PACK_BLIP_MIN_GAP_SEC      2
#define MAC_PACK_BLIP_MAX_GAP_SEC      90
#define MAC_PACK_TIME_CONSTANT_SEC     30.0f
#define MAC_PACK_PROBABILITY_THRESHOLD 0.05f
#define MAC_PACK_MIN_PAYLOAD_MATCH_LEN 3
#define MAC_PACK_MAX_TRACKED_STREAMS    4

// ============================================================================
// GNSS Configuration (SIM7670G onboard modem)
// ============================================================================
#define ENABLE_GNSS              1
#define GNSS_UART_NUM            UART_NUM_1
#define GNSS_UART_TX_PIN         18   // ESP32-S3 -> modem RX
#define GNSS_UART_RX_PIN         17   // modem TX -> ESP32-S3
#define GNSS_UART_BAUD           115200
#define GNSS_POWERON_WAIT_SEC    10
#define GNSS_AT_SYNC_ATTEMPTS    20
#define GNSS_AT_SYNC_RETRY_MS    2000
#define GNSS_CLOCK_SYNC_THRESHOLD_SEC 2

// ============================================================================
// Debugging
// ============================================================================
#define DEBUG_DEVICE_DISCOVERY  0
#define DEBUG_STREAM_WINDOWS    1
#define PHONES_ONLY             0

#endif // CONFIG_H