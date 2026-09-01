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
#define API_URL             "your_api_endpoint_here"
#define API_TIMEOUT_MS      10000
// Skip SSL certificate verification (e.g. for self-signed certs)
#define API_SKIP_CERT_CHECK 1
// Disable sending device data to the API (set to 1 to disable, 0 to enable)
#define DISABLE_API_SEND    0

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

// ============================================================================
// Debugging
// ============================================================================
// Enable verbose logging
#define DEBUG_LOGGING           1

// Log individual device discoveries
#define DEBUG_DEVICE_DISCOVERY  1

#endif // CONFIG_H
