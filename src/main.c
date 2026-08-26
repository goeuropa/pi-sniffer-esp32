/**
 * ESP32 BLE Sniffer - Main Application
 * 
 * A crowd-counting system that scans for nearby BLE devices and reports
 * device data to a REST API endpoint. Based on the pi-sniffer project,
 * optimized for ESP32 microcontrollers.
 * 
 * Features:
 * - BLE device scanning with manufacturer data extraction
 * - RSSI filtering using Kalman filter for distance estimation
 * - Device categorization (phones, computers, wearables, etc.)
 * - REST API reporting of device data
 * - WiFi provisioning via captive portal
 * - Double-press BOOT button to enter config mode
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_sntp.h"

#include "config.h"
#include "wifi_manager.h"
#include "wifi_provision.h"
#include "button_handler.h"
#include "ble_scanner.h"
#include "http_client.h"
#include "device.h"
#include "device_json.h"
#include "mac_pack.h"
#include "gnss.h"
#include "mqtt_report.h"
#include "cellular.h"
#include "cellular_http.h"

static const char *TAG = "BLE_SNIFFER";

// Global device list
static device_list_t device_list;

// Guards device_list against concurrent access between on_device_discovered()
// (called from the BLE stack's own GAP callback task while a scan is live -
// see ble_scanner.c's gap_event_handler) and report_task() (cleanup/
// mac_pack/summary/JSON build). Previously this was done by having
// report_task() wait for ble_scanner_get_status() != SCANNER_RUNNING before
// touching device_list - a timing proxy, not a real lock, and one that could
// silently starve report_task forever: SCAN_DURATION_SEC+1 (11s) is an exact
// multiple of report_task's 1s poll, so it sampled the same fixed phase
// every scan cycle - if that phase never landed in the ~1s STOPPED gap
// between scans, report_task's report (including print_summary()) simply
// never ran, deterministically, not intermittently. A real mutex removes the
// dependency on scan timing entirely.
static SemaphoreHandle_t device_list_mutex = NULL;

// Device ID for this sensor (derived from WiFi MAC)
static char device_id[32];

// Last report time
static time_t last_report_time = 0;

// Config mode flag
static bool config_mode = false;

/**
 * Callback handler for BLE device discoveries
 */
static void on_device_discovered(const uint8_t *mac,
                                  int8_t rssi,
                                  address_type_t addr_type,
                                  const char *name,
                                  uint16_t manufacturer_id,
                                  int8_t tx_power,
                                  const uint8_t *manufacturer_payload,
                                  uint8_t manufacturer_payload_len) {
    // Runs on the BLE stack's own GAP callback task (see ble_scanner.c) -
    // take the lock for the whole function, not just device_update(), since
    // `device` below points into device_list and stays read from until the
    // end (see device_list_mutex's doc comment for why this replaced the old
    // "wait for the scan to be idle" approach).
    if (device_list_mutex == NULL || xSemaphoreTake(device_list_mutex, portMAX_DELAY) != pdTRUE) {
        return;
    }

    // Add or update device in our tracking list
    ble_device_t *device = device_update(&device_list, mac, rssi, addr_type,
                                          name, manufacturer_id, tx_power,
                                          manufacturer_payload, manufacturer_payload_len);

    if (device == NULL) {
        ESP_LOGW(TAG, "Device list full, could not add device");
        xSemaphoreGive(device_list_mutex);
        return;
    }

#if PHONES_ONLY
    if (device->category != CATEGORY_PHONE) {
        xSemaphoreGive(device_list_mutex);
        return;
    }
#endif

#if DEBUG_DEVICE_DISCOVERY
    char mfg_buf[8];

    // Hex-dump the raw manufacturer-specific payload (beyond just the 2-byte
    // company ID) so unrecognized devices - e.g. a Samsung TV that only
    // reports as "phone" - can be investigated and correlated against real
    // captured samples, the same way the original pi-sniffer's manufacturer
    // heuristics were built up from observed byte patterns.
    char payload_hex[48] = "";
    if (manufacturer_payload != NULL && manufacturer_payload_len > 0) {
        uint8_t n = manufacturer_payload_len;
        if (n > 15) n = 15;  // cap so the log line stays a manageable length
        for (uint8_t i = 0; i < n; i++) {
            char byte_str[4];
            snprintf(byte_str, sizeof(byte_str), "%02X ", manufacturer_payload[i]);
            strncat(payload_hex, byte_str, sizeof(payload_hex) - strlen(payload_hex) - 1);
        }
    }

    ESP_LOGI(TAG, "Device: %s RSSI:%d Dist:%.1fm Cat:%s Addr:%s Mfg:%s Name:%s Payload:[%s]",
             device->mac_str, rssi, device->distance,
             category_to_string(device->category),
             addr_type == ADDRESS_TYPE_PUBLIC ? "public" : (addr_type == ADDRESS_TYPE_RANDOM ? "random" : "unknown"),
             manufacturer_to_string(device->has_manufacturer_data, device->manufacturer_id, mfg_buf, sizeof(mfg_buf)),
             device->name[0] ? device->name : "?",
             payload_hex);
#endif

    xSemaphoreGive(device_list_mutex);
}

/**
 * Initialize SNTP for time synchronization
 */
static void init_sntp(void) {
    ESP_LOGI(TAG, "Initializing SNTP");
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    esp_sntp_init();
    
    // Wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    
    while (timeinfo.tm_year < (2020 - 1900) && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for time sync... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        time(&now);
        localtime_r(&now, &timeinfo);
    }
    
    if (timeinfo.tm_year >= (2020 - 1900)) {
        ESP_LOGI(TAG, "Time synchronized: %s", asctime(&timeinfo));
    } else {
        ESP_LOGW(TAG, "Time sync failed, using system time");
    }
}

/**
 * Waits for the initial WiFi connection in the background (doesn't block
 * boot the way init_sntp() used to as part of run_normal_mode()'s old
 * synchronous wifi_wait_connected() call), then runs SNTP once and
 * self-deletes. Polls wifi_is_connected() rather than wifi_wait_connected()
 * - the latter's underlying event group's WIFI_FAIL_BIT is never cleared
 * once set (see wifi_manager.c), so calling it repeatedly in a loop after
 * the first failed attempt would spin without ever actually waiting.
 * SNTP only needs to run once - esp_sntp_init() keeps itself synced going
 * forward on its own (SNTP_OPMODE_POLL), independent of WiFi reconnects
 * mqtt_report_task handles separately.
 */
static void sntp_wait_task(void *pvParameters) {
    (void)pvParameters;
    while (!wifi_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
    init_sntp();
    vTaskDelete(NULL);
}

/**
 * Log one category's line in print_summary()'s per-category breakdown,
 * e.g. "  Phones:      14 [0-5m:  5, 5-10m:  6, 10m+:  3]". Label is
 * left-padded to the width of the longest category name ("Headphones:") and
 * every count is right-aligned to a fixed width, so rows line up with each
 * other regardless of label length or how many digits each count has.
 */
static void log_category_summary(const char *label, int total, const distance_breakdown_t *db) {
    ESP_LOGI(TAG, "  %-11s %3d [0-%.0fm:%3d, %.0f-%.0fm:%3d, %.0fm+:%3d]",
             label, total,
             DISTANCE_BUCKET_NEAR_M, db->near,
             DISTANCE_BUCKET_NEAR_M, DISTANCE_BUCKET_MID_M, db->mid,
             DISTANCE_BUCKET_MID_M, db->far);
}

// One entry per active device being printed - just enough to sort by
// category without touching device_list's own array order (device slot
// indices are relied on elsewhere - device_find()/device_update() etc. -
// so the listing sorts a separate index list instead of the array itself).
typedef struct {
    int index;
    device_category_t category;
} summary_sort_entry_t;

/**
 * Sorts print_summary()'s per-device listing by category, in device_category_t's
 * own declared order (device.h) - not the summary breakdown's print order
 * above it, just whatever groups same-category devices together.
 */
static int summary_sort_entry_cmp(const void *a, const void *b) {
    const summary_sort_entry_t *ea = (const summary_sort_entry_t *)a;
    const summary_sort_entry_t *eb = (const summary_sort_entry_t *)b;
    if (ea->category != eb->category) {
        return (int)ea->category - (int)eb->category;
    }
    return ea->index - eb->index; // stable tie-break: original order within a category
}

static void print_summary(void) {
    device_summary_t summary;
    device_get_summary(&device_list, &summary);

    device_distance_summary_t distances;
    device_get_distance_summary(&device_list, &distances);

    ESP_LOGI(TAG, "=== Device Summary ===");
    ESP_LOGI(TAG, "Total: %d devices", summary.total_devices);
    log_category_summary("Phones:", summary.phones, &distances.phones);
    log_category_summary("Computers:", summary.computers, &distances.computers);
    log_category_summary("Tablets:", summary.tablets, &distances.tablets);
    log_category_summary("Watches:", summary.watches, &distances.watches);
    log_category_summary("Wearables:", summary.wearables, &distances.wearables);
    log_category_summary("Headphones:", summary.headphones, &distances.headphones);
    log_category_summary("Speakers:", summary.speakers, &distances.speakers);
    log_category_summary("Beacons:", summary.beacons, &distances.beacons);
    log_category_summary("Other:", summary.other, &distances.other);
    ESP_LOGI(TAG, "----------------------");

    // Sort the per-device listing below by category instead of raw
    // array-slot order, so entries of the same type print together. Sorts
    // a separate index list rather than device_list.devices itself - see
    // summary_sort_entry_t.
    static summary_sort_entry_t sorted[MAX_DEVICES];
    int sorted_count = 0;
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (!device_list.devices[i].active) {
            continue;
        }
#if PHONES_ONLY
        if (device_list.devices[i].category != CATEGORY_PHONE) {
            continue;
        }
#endif
        sorted[sorted_count].index = i;
        sorted[sorted_count].category = device_list.devices[i].category;
        sorted_count++;
    }
    qsort(sorted, sorted_count, sizeof(sorted[0]), summary_sort_entry_cmp);

    time_t now = time(NULL);
    for (int si = 0; si < sorted_count; si++) {
        const ble_device_t *device = &device_list.devices[sorted[si].index];
        char mfg_buf[8];
        char superseded_buf[32] = "";
        if (device->has_superseded_by) {
            char superseded_mac[18];
            mac_to_string(device->superseded_by, superseded_mac);
            snprintf(superseded_buf, sizeof(superseded_buf), "  -> %s (p=%.2f)",
                     superseded_mac, device->superseded_probability);
        }
        char window_buf[16];
        snprintf(window_buf, sizeof(window_buf), "%lds-%lds",
                 (long)(now - device->first_seen), (long)(now - device->last_seen));

        char tx_buf[5];
        if (device->tx_power == TX_POWER_UNKNOWN) {
            snprintf(tx_buf, sizeof(tx_buf), "?");
        } else {
            snprintf(tx_buf, sizeof(tx_buf), "%d", device->tx_power);
        }

#if DEBUG_STREAM_WINDOWS
        char streams_buf[128] = "";
        if (device->stream_count > 0) {
            int used = snprintf(streams_buf, sizeof(streams_buf), "  streams:");
            for (int s = 0; s < device->stream_count && used > 0 && (size_t)used < sizeof(streams_buf); s++) {
                used += snprintf(streams_buf + used, sizeof(streams_buf) - used, " %02x[%lds-%lds]",
                                  device->streams[s].type,
                                  (long)(now - device->streams[s].first_seen),
                                  (long)(now - device->streams[s].last_seen));
            }
        }
#else
        static const char streams_buf[1] = "";
#endif

        ESP_LOGI(TAG, "  %s  %-5s  RSSI:%4d  TX:%4s  dist:%5.1fm  seen:%3lu %9s  %-20s  %s%s%s",
                 device->mac_str,
                 category_to_string(device->category),
                 device->raw_rssi,
                 tx_buf,
                 device->distance,
                 (unsigned long)device->seen_count,
                 window_buf,
                 manufacturer_to_string(device->has_manufacturer_data, device->manufacturer_id, mfg_buf, sizeof(mfg_buf)),
                 device->name[0] ? device->name : "",
                 superseded_buf,
                 streams_buf);
    }
    ESP_LOGI(TAG, "======================");
}

/**
 * Button event handler - double-press enters config mode
 */
static void on_button_event(button_event_t event) {
    switch (event) {
        case BUTTON_EVENT_DOUBLE_PRESS:
            ESP_LOGW(TAG, "Double-press detected! Entering config mode...");
            // Clear credentials and reboot into config mode
            wifi_provision_clear_credentials();
            vTaskDelay(500 / portTICK_PERIOD_MS);
            wifi_provision_reboot();
            break;
            
        case BUTTON_EVENT_LONG_PRESS:
            ESP_LOGW(TAG, "Long press detected! Clearing credentials...");
            wifi_provision_clear_credentials();
            ESP_LOGI(TAG, "Credentials cleared. Rebooting...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            wifi_provision_reboot();
            break;
            
        case BUTTON_EVENT_SINGLE_PRESS:
            // Print summary on single press. print_summary() itself doesn't
            // lock (see device_list_mutex's doc comment) - report_task()
            // holds it across a wider block, so every caller takes it.
            if (device_list_mutex != NULL && xSemaphoreTake(device_list_mutex, portMAX_DELAY) == pdTRUE) {
                print_summary();
                xSemaphoreGive(device_list_mutex);
            }
            break;
            
        default:
            break;
    }
}

/**
 * Report task - sends device data to REST API periodically
 */
static void report_task(void *pvParameters) {
    while (1) {
        time_t now = time(NULL);

        bool time_to_report = (now - last_report_time >= REPORT_INTERVAL_SEC);

        // Runs every second on schedule regardless of BLE scan state -
        // device_list_mutex (not scan timing) is what keeps this safe
        // against on_device_discovered() running concurrently on the BLE
        // stack's own task. See device_list_mutex's doc comment: the old
        // "wait for the scan to be idle" gate that used to live here could
        // silently starve this whole block forever, which is exactly what
        // it was doing.
        if (time_to_report) {
            if (device_list_mutex == NULL || xSemaphoreTake(device_list_mutex, portMAX_DELAY) != pdTRUE) {
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                continue;
            }

            // Clean up stale devices
            int removed = device_cleanup(&device_list, MAX_DEVICE_AGE_SEC);
            if (removed > 0) {
                ESP_LOGI(TAG, "Removed %d stale devices", removed);
            }

#if ENABLE_MAC_PACKING
            // Detect MAC-rotated devices so they aren't double-counted
            mac_pack_run(&device_list);
#endif

            // Print summary
            print_summary();

#if !DISABLE_API_SEND
            // Phone count needs device_list - compute it while still
            // holding the lock. device_json_build_minimal() below doesn't
            // touch device_list itself, just this already-computed count.
            device_summary_t summary;
            device_get_summary(&device_list, &summary);

            // REPORT_TRANSPORT_CELLULAR_HTTP's payload (device_json_build(),
            // the legacy full-device format) is the one exception that does
            // need device_list itself, under this same lock - unlike the
            // minimal payload above, it iterates every device. Only pay for
            // that expensive build when cellular_http_should_send() (a
            // cheap synchronous pre-check against the last report actually
            // sent - see report_throttle.h) says this cycle is even a
            // candidate to send; cellular_http_publish() re-checks
            // authoritatively, with a fresher fix, right before actually
            // sending - see cellular_http.h.
            char *cellular_http_json_body = NULL;
#if ENABLE_CELLULAR
            if (REPORT_TRANSPORT == REPORT_TRANSPORT_CELLULAR_HTTP) {
                gnss_fix_t fix_for_gate = {0};
#if ENABLE_GNSS
                gnss_get_last_fix(&fix_for_gate); // last-known fix is fine for this
                                                   // gate - the fresh poll below is
                                                   // what actually gets sent
#endif
                if (cellular_http_should_send(summary.phones, fix_for_gate)) {
                    cellular_http_json_body = device_json_build(&device_list, device_id);
                }
            }
#endif
#endif // !DISABLE_API_SEND

            // Everything from here on works from independent snapshots
            // (summary, cellular_http_json_body, and the GNSS fix below),
            // not device_list itself - release the lock now rather than
            // holding it through the GNSS poll/network queuing/logging.
            xSemaphoreGive(device_list_mutex);

#if !DISABLE_API_SEND
            // Poll GNSS synchronously, once per report cycle, regardless of
            // which transport REPORT_TRANSPORT actually sends over - this
            // is the *only* GNSS poll site in the firmware (no independent
            // polling task - see gnss.h), so both WiFi and cellular reports
            // get a recent position/GPS-timestamp, not just cellular.
            // fix.valid/latitude/longitude stay at the last known values
            // (see gnss_parse_cgnssinfo()) even when this particular poll
            // comes back with no current fix - device_json_build_minimal()
            // below uses that, plus a system-clock fallback, when GNSS
            // isn't available (ENABLE_GNSS=0, or no fix yet).
            gnss_fix_t fix = {0};
#if ENABLE_GNSS
            gnss_poll_now();
            gnss_get_last_fix(&fix);
#endif

            // Both transports are pure hand-offs to their own background
            // task (mqtt_report_task / cellular_task, see mqtt_report.h /
            // cellular.h) - report_task() never waits on the actual network
            // exchange. WiFi and cellular are mutually exclusive
            // (REPORT_TRANSPORT), so exactly one of these runs.
            char topic[64];
            snprintf(topic, sizeof(topic), "%s/%s", MQTT_TOPIC_PREFIX, device_id);

            bool wifi_queued = false;
            bool cellular_queued = false;

            switch (REPORT_TRANSPORT) {
                case REPORT_TRANSPORT_WIFI:
                    if (mqtt_report_is_connected()) {
                        char *json_body = device_json_build_minimal(fix.valid, fix.latitude, fix.longitude,
                                                                      fix.fix_time, summary.phones, device_id);
                        if (json_body != NULL) {
                            wifi_queued = mqtt_report_publish(topic, json_body, MQTT_QOS);
                            free(json_body);
                        }
                    }
                    break;

                case REPORT_TRANSPORT_CELLULAR_MQTT:
#if ENABLE_CELLULAR
                    cellular_queued = cellular_publish(topic, summary.phones, fix, MQTT_QOS);
#else
                    ESP_LOGW(TAG, "REPORT_TRANSPORT is cellular-only but ENABLE_CELLULAR=0");
#endif
                    break;

                case REPORT_TRANSPORT_CELLULAR_HTTP:
#if ENABLE_CELLULAR
                    if (cellular_http_json_body != NULL) {
                        cellular_queued = cellular_http_publish(topic, cellular_http_json_body,
                                                                 summary.phones, fix);
                        cellular_http_json_body = NULL; // ownership transferred either way
                    } else {
                        ESP_LOGI(TAG, "Skipping cellular HTTP report: throttled");
                    }
#else
                    ESP_LOGW(TAG, "REPORT_TRANSPORT is cellular-HTTP but ENABLE_CELLULAR=0");
#endif
                    break;
            }

            if (wifi_queued) {
                ESP_LOGI(TAG, "Report queued for WiFi send (%s)", topic);
            } else if (cellular_queued) {
                ESP_LOGI(TAG, "Report queued for cellular send (%s)", topic);
            } else {
                // Both tasks already log their own reason (backing off /
                // bring-up failed / AT or publish failure) when they're
                // the transport in play - spell out both states here too
                // rather than a generic "not sent" that leaves it
                // ambiguous which transport (or both) was the problem.
                ESP_LOGW(TAG, "Report not sent (wifi_mqtt=%s cellular_mqtt=%s cellular_http=%s)",
                         mqtt_report_is_connected() ? "up" : "down",
#if ENABLE_CELLULAR
                         cellular_is_connected() ? "up" : "down",
                         cellular_http_is_connected() ? "up" : "down"
#else
                         "disabled",
                         "disabled"
#endif
                         );
            }

            // Safety net: cellular_http_json_body is only non-NULL when
            // REPORT_TRANSPORT_CELLULAR_HTTP built it above and the switch
            // case's cellular_http_publish() call didn't run/transfer
            // ownership for some reason (e.g. a future early-return added
            // between the build and the switch) - avoid a silent leak of a
            // potentially large full-device JSON string.
            if (cellular_http_json_body != NULL) {
                free(cellular_http_json_body);
            }
#else
            // device_list_mutex is already released unconditionally above,
            // regardless of DISABLE_API_SEND.
            ESP_LOGI(TAG, "Reporting disabled (DISABLE_API_SEND), skipping send");
#endif

            last_report_time = now;
        }
        
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * Scan task - continuously scans for BLE devices
 */
static void scan_task(void *pvParameters) {
    while (1) {
        // Start a scan cycle
        if (ble_scanner_get_status() != SCANNER_RUNNING) {
            ESP_LOGI(TAG, "Starting BLE scan for %d seconds", SCAN_DURATION_SEC);
            ble_scanner_start(SCAN_DURATION_SEC, on_device_discovered);
        }

        // Wait for scan to complete
        vTaskDelay((SCAN_DURATION_SEC + 1) * 1000 / portTICK_PERIOD_MS);
    }
}

/**
 * Run provisioning mode - WiFi configuration portal
 */
static void run_provisioning_mode(void) {
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "   CONFIGURATION MODE");
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "Connect to WiFi: %s", PROVISION_AP_SSID);
    ESP_LOGI(TAG, "Then open: http://192.168.4.1");
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=================================");
    
    // Start the captive portal
    if (!wifi_provision_start_portal()) {
        ESP_LOGE(TAG, "Failed to start provisioning portal!");
        return;
    }
    
    // Wait indefinitely - the portal will reboot when configured
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * Run normal operation mode - BLE scanning and reporting
 */
static void run_normal_mode(wifi_credentials_t *creds) {
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "   ESP32 BLE Sniffer Starting");
    ESP_LOGI(TAG, "=================================");
    
    // Initialize device list
    device_list_init(&device_list);

    // Guards device_list - must exist before ble_scanner_init()/scan_task
    // start delivering scan results (see device_list_mutex's doc comment).
    device_list_mutex = xSemaphoreCreateMutex();
    if (device_list_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create device_list_mutex!");
        return;
    }

    // Initialize WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    if (!wifi_manager_init()) {
        ESP_LOGE(TAG, "WiFi init failed!");
        return;
    }
    
    // Connect to configured WiFi. wifi_connect() itself is non-blocking -
    // it just kicks off esp_wifi_start(); actual association happens
    // asynchronously via wifi_manager.c's event handler. Nothing in boot
    // waits for it to finish: mqtt_report_task (started below) owns
    // watching/retrying that connection from here on, on its own schedule.
    ESP_LOGI(TAG, "Connecting to WiFi: %s", creds->ssid);
    wifi_connect(creds->ssid, creds->password);

    // Device ID from the WiFi MAC address - available as soon as the driver
    // is up, doesn't need association to complete, so this no longer
    // depends on (or waits for) a successful connection.
    char mac_str[18];
    if (wifi_get_mac(mac_str)) {
        // Create device ID from MAC (use last 6 hex digits without colons)
        // mac_str format: "XX:XX:XX:XX:XX:XX"
        device_id[0] = 'E';
        device_id[1] = 'S';
        device_id[2] = 'P';
        device_id[3] = '3';
        device_id[4] = '2';
        device_id[5] = '_';
        // Copy last 6 hex chars (positions 9,10,12,13,15,16)
        device_id[6] = mac_str[9];
        device_id[7] = mac_str[10];
        device_id[8] = mac_str[12];
        device_id[9] = mac_str[13];
        device_id[10] = mac_str[15];
        device_id[11] = mac_str[16];
        device_id[12] = '\0';
    } else {
        strcpy(device_id, "ESP32_UNKNOWN");
    }
    ESP_LOGI(TAG, "Device ID: %s", device_id);

    // Time sync needs WiFi actually connected - waits for that in the
    // background instead of blocking boot the way it used to.
    xTaskCreate(sntp_wait_task, "sntp_wait_task", 3072, NULL, 2, NULL);

    // Initialize HTTP client (legacy REST path - kept but unused by
    // report_task(), see device_json.h / plans/4g-integration.md)
    http_client_init();

#if !DISABLE_API_SEND
    // Initialize MQTT reporting per REPORT_TRANSPORT (config.h). WiFi and
    // cellular are mutually exclusive - only the one background task the
    // configured transport actually needs is started, never both. Gated
    // the same as report_task()'s send below - with reporting disabled
    // there's no point spinning up a task that just retries against
    // WiFi/the broker forever.
    if (REPORT_TRANSPORT == REPORT_TRANSPORT_WIFI) {
        // mqtt_report_start_task() owns WiFi's (re)connection and the MQTT
        // client from here on, on its own background task - see
        // mqtt_report.h.
        mqtt_report_start_task(creds->ssid, creds->password, device_id);
    }

#if ENABLE_CELLULAR
    if (REPORT_TRANSPORT == REPORT_TRANSPORT_CELLULAR_MQTT) {
        // Cellular MQTT reporting (SIM7670G modem-side AT+CMQTT*) - actual
        // sending runs on cellular_task, its own background task -
        // report_task() only ever hands off the latest report via
        // cellular_publish(), never waits on the AT exchange itself. main.c
        // doesn't need to know anything about cellular init/retry beyond
        // this one call - failures are logged from within cellular.c,
        // matching mqtt_report_start_task() above.
        cellular_start_task(device_id);
    }
    if (REPORT_TRANSPORT == REPORT_TRANSPORT_CELLULAR_HTTP) {
        // Cellular HTTP reporting (SIM7670G modem-side AT+HTTP*, legacy
        // full-device JSON format) - same "own background task, main.c
        // doesn't need to know cellular internals" shape as
        // cellular_start_task() above, see cellular_http.h.
        cellular_http_start_task(device_id);
    }
#endif
#endif // !DISABLE_API_SEND

    // Initialize button handler for runtime double-press detection
    button_handler_init();
    button_handler_register_callback(on_button_event);
    
    // Initialize BLE scanner
    ESP_LOGI(TAG, "Initializing BLE scanner...");
    if (!ble_scanner_init()) {
        ESP_LOGE(TAG, "BLE scanner init failed!");
        return;
    }

#if ENABLE_GNSS
    // Power on GNSS (SIM7670G onboard modem) - independent of WiFi/BLE and
    // of REPORT_TRANSPORT/ENABLE_CELLULAR. No polling task started here:
    // GNSS is polled synchronously by report_task(), once per report cycle
    // (see gnss.h) - not on its own timer - so it runs regardless of
    // whether reporting ends up going over WiFi or cellular.
    ESP_LOGI(TAG, "Initializing GNSS...");
    if (!gnss_init()) {
        ESP_LOGW(TAG, "GNSS init failed, continuing without GNSS");
    }
#endif

    ESP_LOGI(TAG, "Starting scan and report tasks...");
    
    // Initialize last report time
    last_report_time = time(NULL);
    
    // Create scan task
    xTaskCreate(scan_task, "scan_task", 4096, NULL, 5, NULL);
    
    // Create report task
    xTaskCreate(report_task, "report_task", 8192, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "   BLE Sniffer Running!");
    ESP_LOGI(TAG, "   Scan interval: %d sec", SCAN_DURATION_SEC);
    ESP_LOGI(TAG, "   Report interval: %d sec", REPORT_INTERVAL_SEC);
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "   Double-press BOOT button");
    ESP_LOGI(TAG, "   to enter config mode");
    ESP_LOGI(TAG, "=================================");
}

/**
 * Main application entry point
 */
void app_main(void) {
    // The runtime log level defaults to INFO (CONFIG_LOG_DEFAULT_LEVEL_INFO
    // in sdkconfig) even though DEBUG-level logs are compiled in
    // (CONFIG_LOG_MAXIMUM_LEVEL_DEBUG) - override individual tags here to
    // surface their ESP_LOGD output without changing the default for
    // everything else. Use esp_log_level_set("*", ESP_LOG_DEBUG) instead to
    // raise every tag at once.
    // esp_log_level_set("APPLE_HEUR", ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   ESP32 BLE Sniffer v1.1");
    ESP_LOGI(TAG, "========================================");

    // Initialize provisioning system (NVS)
    if (!wifi_provision_init()) {
        ESP_LOGE(TAG, "Failed to initialize provisioning!");
        return;
    }
    
    // Check for double-press at boot to enter config mode
    // Give user 3 seconds to double-press the BOOT button
    config_mode = button_check_double_press_boot(3000);
    
    // Also enter config mode if no credentials are stored
    if (!config_mode && !wifi_provision_has_credentials()) {
        ESP_LOGI(TAG, "No WiFi credentials found, entering config mode");
        config_mode = true;
    }
    
    if (config_mode) {
        // Run WiFi provisioning portal
        run_provisioning_mode();
    } else {
        // Load credentials and run normal mode
        wifi_credentials_t creds;
        if (wifi_provision_load_credentials(&creds)) {
            run_normal_mode(&creds);
        } else {
            ESP_LOGE(TAG, "Failed to load credentials!");
            config_mode = true;
            run_provisioning_mode();
        }
    }
}
