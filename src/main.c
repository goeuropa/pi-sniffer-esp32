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
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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
#include "mac_pack.h"

static const char *TAG = "BLE_SNIFFER";

// Global device list
static device_list_t device_list;

// Device ID for this sensor (derived from WiFi MAC or platformio.ini)
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
    // Add or update device in our tracking list
    ble_device_t *device = device_update(&device_list, mac, rssi, addr_type,
                                          name, manufacturer_id, tx_power,
                                          manufacturer_payload, manufacturer_payload_len);
    
    if (device == NULL) {
        ESP_LOGW(TAG, "Device list full, could not add device");
        return;
    }
    
#if DEBUG_LOGGING
    char mfg_buf[8];

    // Hex-dump the raw manufacturer-specific payload
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
 * Print device summary to log
 */
static void print_summary(void) {
    device_summary_t summary;
    device_get_summary(&device_list, &summary);

    ESP_LOGI(TAG, "=== Device Summary ===");
    ESP_LOGI(TAG, "Total: %d devices", summary.total_devices);
    ESP_LOGI(TAG, "  Phones: %d", summary.phones);
    ESP_LOGI(TAG, "  Computers: %d", summary.computers);
    ESP_LOGI(TAG, "  Tablets: %d", summary.tablets);
    ESP_LOGI(TAG, "  Watches: %d", summary.watches);
    ESP_LOGI(TAG, "  Wearables: %d", summary.wearables);
    ESP_LOGI(TAG, "  Headphones: %d", summary.headphones);
    ESP_LOGI(TAG, "  Speakers: %d", summary.speakers);
    ESP_LOGI(TAG, "  Beacons: %d", summary.beacons);
    ESP_LOGI(TAG, "  Other: %d", summary.other);
    ESP_LOGI(TAG, "----------------------");

    time_t now = time(NULL);
    for (int i = 0; i < MAX_DEVICES; i++) {
        const ble_device_t *device = &device_list.devices[i];
        if (!device->active) {
            continue;
        }
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

        ESP_LOGI(TAG, "  %s  %-5s  RSSI:%4d  TX:%4s  dist:%5.1fm  seen:%3lu  window:%9s  %-20s  %s%s",
                 device->mac_str,
                 category_to_string(device->category),
                 device->raw_rssi,
                 tx_buf,
                 device->distance,
                 (unsigned long)device->seen_count,
                 window_buf,
                 manufacturer_to_string(device->has_manufacturer_data, device->manufacturer_id, mfg_buf, sizeof(mfg_buf)),
                 device->name[0] ? device->name : "",
                 superseded_buf);
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
            print_summary();
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
        
        if (now - last_report_time >= REPORT_INTERVAL_SEC) {
            int removed = device_cleanup(&device_list, MAX_DEVICE_AGE_SEC);
            if (removed > 0) {
                ESP_LOGI(TAG, "Removed %d stale devices", removed);
            }

#if ENABLE_MAC_PACKING
            mac_pack_run(&device_list);
#endif

            print_summary();
            
#if !DISABLE_API_SEND
            if (wifi_is_connected()) {
                ESP_LOGI(TAG, "Sending device data to API...");
                if (http_send_devices(&device_list, device_id)) {
                    ESP_LOGI(TAG, "Data sent successfully");
                } else {
                    ESP_LOGW(TAG, "Failed to send data");
                }
            } else {
                ESP_LOGW(TAG, "WiFi not connected, skipping report");
            }
#else
            ESP_LOGI(TAG, "API reporting disabled (DISABLE_API_SEND), skipping send");
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
        if (ble_scanner_get_status() != SCANNER_RUNNING) {
            ESP_LOGI(TAG, "Starting BLE scan for %d seconds", SCAN_DURATION_SEC);
            ble_scanner_start(SCAN_DURATION_SEC, on_device_discovered);
        }

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
    
    if (!wifi_provision_start_portal()) {
        ESP_LOGE(TAG, "Failed to start provisioning portal!");
        return;
    }
    
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
    
    device_list_init(&device_list);
    
    ESP_LOGI(TAG, "Initializing WiFi...");
    if (!wifi_manager_init()) {
        ESP_LOGE(TAG, "WiFi init failed!");
        return;
    }
    
    ESP_LOGI(TAG, "Connecting to WiFi: %s", creds->ssid);
    wifi_connect(creds->ssid, creds->password);
    
    if (wifi_wait_connected(30000)) {
        ESP_LOGI(TAG, "WiFi connected!");
        
        // --- USTALANIE NAZWY URZĄDZENIA (DEVICE ID) ---
#ifdef DEVICE_NAME
        // Użyj własnej nazwy z zdefiniowanej w platformio.ini
        snprintf(device_id, sizeof(device_id), "%s", DEVICE_NAME);
#else
        // Domyślnie utwórz nazwę na podstawie adresu MAC
        char mac_str[18];
        if (wifi_get_mac(mac_str)) {
            device_id[0] = 'E';
            device_id[1] = 'S';
            device_id[2] = 'P';
            device_id[3] = '3';
            device_id[4] = '2';
            device_id[5] = '_';
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
#endif
        ESP_LOGI(TAG, "Device ID: %s", device_id);
        
        init_sntp();
    } else {
        ESP_LOGW(TAG, "WiFi connection failed!");
        ESP_LOGW(TAG, "Check your credentials or double-press BOOT to reconfigure");
        strcpy(device_id, "ESP32_OFFLINE");
    }
    
    http_client_init();
    
    button_handler_init();
    button_handler_register_callback(on_button_event);
    
    ESP_LOGI(TAG, "Initializing BLE scanner...");
    if (!ble_scanner_init()) {
        ESP_LOGE(TAG, "BLE scanner init failed!");
        return;
    }
    
    ESP_LOGI(TAG, "Starting scan and report tasks...");
    
    last_report_time = time(NULL);
    
    xTaskCreate(scan_task, "scan_task", 4096, NULL, 5, NULL);
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
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "   ESP32 BLE Sniffer v1.1");
    ESP_LOGI(TAG, "========================================");

    if (!wifi_provision_init()) {
        ESP_LOGE(TAG, "Failed to initialize provisioning!");
        return;
    }
    
    config_mode = button_check_double_press_boot(3000);
    
    if (!config_mode && !wifi_provision_has_credentials()) {
        ESP_LOGI(TAG, "No WiFi credentials found, entering config mode");
        config_mode = true;
    }
    
    if (config_mode) {
        run_provisioning_mode();
    } else {
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