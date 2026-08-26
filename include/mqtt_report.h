/**
 * WiFi MQTT Reporting
 *
 * Publishes device reports over MQTT via the WiFi esp_netif_t, using
 * ESP-IDF's built-in esp-mqtt component (esp_mqtt_client). This is the
 * WiFi-side half of unifying reporting onto MQTT - see
 * plans/4g-integration.md - the cellular-side half is cellular.c, which
 * publishes the same payload/topic through the modem's AT+CMQTT* commands
 * instead of a local IP stack.
 *
 * Sending runs on its own background task (mqtt_report_task, started by
 * mqtt_report_start_task()), decoupled from whichever task calls
 * mqtt_report_publish() (report_task() in practice) - mirrors cellular.c's
 * cellular_task exactly. mqtt_report_publish() only hands the latest report
 * off via a length-1 queue and returns immediately; if a new report arrives
 * before the previous one was sent, it replaces it - only the freshest
 * pending report is ever attempted.
 *
 * The task also owns getting (and keeping) WiFi + the MQTT client
 * connected: an outer loop retries every WIFI_RECONNECT_INTERVAL_SEC once
 * wifi_manager.c's own fast burst-retry has given up (a brief WiFi blip
 * still recovers in seconds via that existing mechanism, unchanged), and an
 * inner loop serves the report queue while connected, dropping back to the
 * outer loop on any publish failure or detected disconnect.
 */

#ifndef MQTT_REPORT_H
#define MQTT_REPORT_H

#include <stdbool.h>

/**
 * Start the background task that connects WiFi + the MQTT client and serves
 * the report queue. Copies ssid/password/client_id into its own storage
 * (needed for repeated wifi_connect() re-kicks across the task's lifetime),
 * so the caller's copies don't need to outlive this call.
 * @param ssid WiFi network name (from the loaded provisioning credentials)
 * @param password WiFi password
 * @param client_id This sensor's device ID, used as the MQTT client ID
 * @return true if the task was created
 */
bool mqtt_report_start_task(const char *ssid, const char *password, const char *client_id);

/**
 * @return true if WiFi is associated AND the MQTT client currently has a
 *         live broker connection
 */
bool mqtt_report_is_connected(void);

/**
 * Hand off a report to be published over MQTT. Returns as soon as the
 * report is queued - the actual esp_mqtt_client_publish() call happens
 * later, asynchronously, on mqtt_report_task. If a report is already queued
 * and not yet sent, it's replaced (freed) by this one - only the most
 * recent report is ever attempted, never a backlog.
 * @return true if handed off to the queue - NOT whether it was delivered;
 *         delivery success/failure is logged separately, from mqtt_report_task
 */
bool mqtt_report_publish(const char *topic, const char *payload, int qos);

#endif // MQTT_REPORT_H
