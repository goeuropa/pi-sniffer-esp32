/**
 * WiFi MQTT Reporting
 *
 * See mqtt_report.h.
 */

#include "mqtt_report.h"
#include "config.h"
#include "wifi_manager.h"

#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "mqtt_client.h"
#include "esp_log.h"

static const char *TAG = "MQTT_REPORT";

// Copied in by mqtt_report_start_task() - the outer loop needs these for
// every wifi_connect() re-kick across the task's lifetime, not just the
// first attempt.
static char s_ssid[33] = "";
static char s_password[65] = "";
static char s_client_id[32] = "";

static esp_mqtt_client_handle_t s_client = NULL;
static volatile bool s_mqtt_connected = false;

// A single pending report, handed from mqtt_report_publish() (called on
// report_task's schedule) to mqtt_report_task (its own background task) via
// s_report_queue - same shape/semantics as cellular.c's pending-report queue.
typedef struct {
    char *topic;
    char *payload;
    int qos;
} wifi_pending_report_t;

// Length 1: only the freshest un-sent report is ever kept.
static QueueHandle_t s_report_queue = NULL;

// How often the outer loop polls connection state while not (yet) connected.
// Cheap and frequent, so a recovery (wifi_manager's fast burst succeeding,
// or esp-mqtt's own background reconnect succeeding) is noticed quickly -
// separate from WIFI_RECONNECT_INTERVAL_SEC, which only gates how often we
// *actively* re-kick a stuck WiFi connection (see below).
#define MQTT_REPORT_POLL_MS 2000

// How long the inner loop's queue receive blocks between connection-state
// rechecks - responsive enough to notice a drop without busy-polling.
#define MQTT_REPORT_QUEUE_POLL_MS 5000

static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                                int32_t event_id, void *event_data) {
    (void)handler_args;
    (void)base;
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Connected to broker %s", MQTT_BROKER_URI);
            s_mqtt_connected = true;
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "Disconnected from broker");
            s_mqtt_connected = false;
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGW(TAG, "MQTT error (type=%d)", event->error_handle ? event->error_handle->error_type : -1);
            break;
        default:
            break;
    }
}

static void wifi_free_pending(wifi_pending_report_t *item) {
    if (item == NULL) {
        return;
    }
    free(item->topic);
    free(item->payload);
    free(item);
}

/**
 * Create and start the esp-mqtt client, once. A no-op on every call after
 * the first - esp-mqtt manages its own reconnect internally once started,
 * so there's normally no need to stop/recreate it, just wait for
 * MQTT_EVENT_CONNECTED again.
 */
static bool mqtt_client_ensure_started(void) {
    if (s_client != NULL) {
        return true;
    }

    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.client_id = s_client_id,
        .session.keepalive = MQTT_KEEPALIVE_SEC,
    };

    s_client = esp_mqtt_client_init(&cfg);
    if (s_client == NULL) {
        ESP_LOGE(TAG, "esp_mqtt_client_init failed");
        return false;
    }

    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);

    esp_err_t err = esp_mqtt_client_start(s_client);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_mqtt_client_start failed: %s", esp_err_to_name(err));
        s_client = NULL;
        return false;
    }

    ESP_LOGI(TAG, "MQTT client started, connecting to %s", MQTT_BROKER_URI);
    return true;
}

/**
 * Background task: outer loop gets WiFi + the MQTT client connected
 * (retrying every WIFI_RECONNECT_INTERVAL_SEC once wifi_manager.c's own
 * fast burst-retry has given up), inner loop serves the report queue while
 * connected. Runs independently of report_task - a slow WiFi/MQTT
 * connect/reconnect here never delays report_task's own loop
 * (print_summary(), the next report cycle, cellular sending).
 */
static void mqtt_report_task(void *pvParameters) {
    (void)pvParameters;
    TickType_t last_reconnect_attempt = 0;

    while (1) {
        // --- Outer loop: get WiFi + MQTT connected ---
        while (!(wifi_is_connected() && s_mqtt_connected)) {
            if (!wifi_is_connected()) {
                TickType_t now = xTaskGetTickCount();
                if (wifi_get_status() == WIFI_STATUS_ERROR &&
                    (now - last_reconnect_attempt) >= pdMS_TO_TICKS(WIFI_RECONNECT_INTERVAL_SEC * 1000)) {
                    // wifi_manager.c's own fast burst-retry already gave up
                    // (WIFI_MAX_RETRY exhausted) - re-kick it, which resets
                    // retry_count and restarts that fast retry. Paced to
                    // WIFI_RECONNECT_INTERVAL_SEC so a genuinely down AP
                    // doesn't get hammered.
                    ESP_LOGW(TAG, "WiFi retry exhausted, re-kicking connection (retrying every %ds)...",
                             WIFI_RECONNECT_INTERVAL_SEC);
                    wifi_connect(s_ssid, s_password);
                    last_reconnect_attempt = now;
                }
            } else {
                // WiFi is up - make sure the MQTT client exists and is
                // trying to connect; no-op after the first successful call.
                mqtt_client_ensure_started();
            }
            vTaskDelay(pdMS_TO_TICKS(MQTT_REPORT_POLL_MS));
        }

        ESP_LOGI(TAG, "WiFi + MQTT ready");

        // --- Inner loop: serve the report queue while connected ---
        wifi_pending_report_t *item = NULL;
        while (wifi_is_connected() && s_mqtt_connected) {
            if (xQueueReceive(s_report_queue, &item, pdMS_TO_TICKS(MQTT_REPORT_QUEUE_POLL_MS)) != pdTRUE) {
                continue; // no report yet - loop back and recheck connection state
            }

            int msg_id = esp_mqtt_client_publish(s_client, item->topic, item->payload, 0, item->qos, 0);
            if (msg_id < 0) {
                ESP_LOGW(TAG, "Publish to %s failed", item->topic);
                wifi_free_pending(item);
                item = NULL;
                break; // drop back to the outer loop to reconnect
            }

            ESP_LOGI(TAG, "Published %d bytes to %s over WiFi (msg_id=%d)",
                     (int)strlen(item->payload), item->topic, msg_id);
            wifi_free_pending(item);
            item = NULL;
        }
    }
}

bool mqtt_report_start_task(const char *ssid, const char *password, const char *client_id) {
    strncpy(s_ssid, ssid, sizeof(s_ssid) - 1);
    s_ssid[sizeof(s_ssid) - 1] = '\0';
    strncpy(s_password, password, sizeof(s_password) - 1);
    s_password[sizeof(s_password) - 1] = '\0';
    strncpy(s_client_id, client_id, sizeof(s_client_id) - 1);
    s_client_id[sizeof(s_client_id) - 1] = '\0';

    s_report_queue = xQueueCreate(1, sizeof(wifi_pending_report_t *));
    if (s_report_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create WiFi report queue");
        return false;
    }

    xTaskCreate(mqtt_report_task, "mqtt_report_task", 4096, NULL, 3, NULL);
    return true;
}

bool mqtt_report_is_connected(void) {
    return wifi_is_connected() && s_mqtt_connected;
}

bool mqtt_report_publish(const char *topic, const char *payload, int qos) {
    if (s_report_queue == NULL) {
        ESP_LOGW(TAG, "mqtt_report_publish() called before mqtt_report_start_task()");
        return false;
    }

    wifi_pending_report_t *item = malloc(sizeof(wifi_pending_report_t));
    char *topic_copy = strdup(topic);
    char *payload_copy = strdup(payload);
    if (item == NULL || topic_copy == NULL || payload_copy == NULL) {
        ESP_LOGE(TAG, "Out of memory queuing WiFi report");
        free(item);
        free(topic_copy);
        free(payload_copy);
        return false;
    }
    item->topic = topic_copy;
    item->payload = payload_copy;
    item->qos = qos;

    // Keep only the freshest report - see cellular_publish()'s identical
    // policy in cellular.c for the rationale.
    wifi_pending_report_t *old = NULL;
    if (xQueueReceive(s_report_queue, &old, 0) == pdTRUE) {
        wifi_free_pending(old);
    }

    if (xQueueSend(s_report_queue, &item, 0) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to queue WiFi report");
        wifi_free_pending(item);
        return false;
    }

    return true;
}
