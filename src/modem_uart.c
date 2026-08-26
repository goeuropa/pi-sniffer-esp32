/**
 * Shared UART/AT-command transport to the SIM7670G modem
 *
 * See modem_uart.h. Only compiled for the target (ESP-IDF); nothing here is
 * pure-logic enough to be worth host-testing the way gnss.c's parsing half
 * is - it's all UART/FreeRTOS primitives.
 */

#include "modem_uart.h"
#include "config.h"

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "MODEM_UART";

static SemaphoreHandle_t s_uart_mutex = NULL;
static bool s_initialized = false;
static bool s_synced = false;

bool modem_uart_lock(int timeout_ms) {
    if (s_uart_mutex == NULL) {
        return false;
    }
    return xSemaphoreTake(s_uart_mutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void modem_uart_unlock(void) {
    if (s_uart_mutex != NULL) {
        xSemaphoreGive(s_uart_mutex);
    }
}

void modem_uart_write_raw(const void *buf, size_t len) {
    // Logs every byte written to the modem, regardless of caller -
    // modem_uart_send_at() and cellular.c's lower-level multi-step
    // exchanges (topic/payload raw data included) all funnel through this
    // one function, so this is the single place that captures the full AT
    // transcript. %.*s rather than %s since buf isn't necessarily
    // NUL-terminated (raw topic/payload data isn't).
    ESP_LOGI(TAG, "AT TX: %.*s", (int)len, (const char *)buf);
    uart_write_bytes(GNSS_UART_NUM, buf, len);
}

int modem_uart_read_response(char *resp, size_t resp_size, int timeout_ms) {
    size_t total = 0;
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start) < timeout_ticks && total < resp_size - 1) {
        int n = uart_read_bytes(GNSS_UART_NUM, (uint8_t *)resp + total,
                                 resp_size - 1 - total, pdMS_TO_TICKS(200));
        if (n > 0) {
            total += n;
            resp[total] = '\0';
            if (strstr(resp, "OK\r\n") != NULL || strstr(resp, "ERROR") != NULL) {
                break;
            }
        }
    }
    resp[total] = '\0';

    if (total == 0) {
        ESP_LOGI(TAG, "AT RX: (no response)");
        return -1;
    }
    ESP_LOGI(TAG, "AT RX: %s", resp);
    return (int)total;
}

bool modem_uart_wait_for_prompt(char *resp, size_t resp_size, int timeout_ms) {
    size_t total = 0;
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while ((xTaskGetTickCount() - start) < timeout_ticks && total < resp_size - 1) {
        int n = uart_read_bytes(GNSS_UART_NUM, (uint8_t *)resp + total,
                                 resp_size - 1 - total, pdMS_TO_TICKS(200));
        if (n > 0) {
            total += n;
            resp[total] = '\0';
            if (strchr(resp, '>') != NULL) {
                ESP_LOGI(TAG, "AT RX: %s", resp);
                return true;
            }
            if (strstr(resp, "ERROR") != NULL) {
                ESP_LOGI(TAG, "AT RX: %s", resp);
                return false;
            }
        }
    }
    resp[total] = '\0';
    ESP_LOGI(TAG, "AT RX: %s", total > 0 ? resp : "(no response, no '>' prompt)");
    return false;
}

void modem_uart_flush_input(void) {
    ESP_LOGI(TAG, "AT: flushing stale input");
    uart_flush_input(GNSS_UART_NUM);
}

bool modem_uart_send_at(const char *cmd, char *resp, size_t resp_size, int timeout_ms) {
    if (!modem_uart_lock(timeout_ms)) {
        ESP_LOGW(TAG, "Timed out waiting for UART lock: %s", cmd);
        resp[0] = '\0';
        return false;
    }

    char line[160];
    int cmd_len = snprintf(line, sizeof(line), "%s\r\n", cmd);
    uart_flush_input(GNSS_UART_NUM);
    modem_uart_write_raw(line, cmd_len);

    int n = modem_uart_read_response(resp, resp_size, timeout_ms);

    modem_uart_unlock();
    return n > 0;
}

bool modem_uart_init(void) {
    if (s_initialized) {
        return s_synced;
    }
    s_initialized = true;

    s_uart_mutex = xSemaphoreCreateMutex();
    if (s_uart_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create UART mutex");
        return false;
    }

    uart_config_t uart_config = {
        .baud_rate = GNSS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(GNSS_UART_NUM, 1024, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return false;
    }
    err = uart_param_config(GNSS_UART_NUM, &uart_config);
    if (err == ESP_OK) {
        err = uart_set_pin(GNSS_UART_NUM, GNSS_UART_TX_PIN, GNSS_UART_RX_PIN,
                            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "UART config failed: %s", esp_err_to_name(err));
        uart_driver_delete(GNSS_UART_NUM);
        return false;
    }

    ESP_LOGI(TAG, "Waiting %ds for modem to boot...", GNSS_POWERON_WAIT_SEC);
    vTaskDelay(pdMS_TO_TICKS(GNSS_POWERON_WAIT_SEC * 1000));

    // Combo LTE+GNSS modems can take a while to come up from a cold boot
    // (well past the initial GNSS_POWERON_WAIT_SEC), and if the board gates
    // modem power behind a physical switch/jumper rather than a PWRKEY GPIO,
    // AT will simply never respond until that's addressed. Be patient here -
    // GNSS_AT_SYNC_ATTEMPTS attempts, GNSS_AT_SYNC_RETRY_MS apart - and log
    // every attempt so it's obvious whether the modem is silent (no bytes at
    // all - likely unpowered or wrong pins) or almost there (garbled/partial
    // bytes - likely wrong baud, or a boot banner not yet followed by "OK").
    char resp[512];
    for (int attempt = 0; attempt < GNSS_AT_SYNC_ATTEMPTS; attempt++) {
        bool got = modem_uart_send_at("AT", resp, sizeof(resp), 1000);
        if (got && strstr(resp, "OK") != NULL) {
            s_synced = true;
            break;
        }
        if (got) {
            char hex[3 * 32 + 1] = "";
            int n = (int)strlen(resp);
            int shown = n < 32 ? n : 32;
            for (int i = 0; i < shown; i++) {
                snprintf(hex + i * 3, 4, "%02X ", (uint8_t)resp[i]);
            }
            ESP_LOGW(TAG, "AT attempt %d/%d: got %d byte(s), no OK: [%s] \"%s\"",
                     attempt + 1, GNSS_AT_SYNC_ATTEMPTS, n, hex, resp);
        } else {
            ESP_LOGW(TAG, "AT attempt %d/%d: no bytes received", attempt + 1, GNSS_AT_SYNC_ATTEMPTS);
        }
        vTaskDelay(pdMS_TO_TICKS(GNSS_AT_SYNC_RETRY_MS));
    }

    if (!s_synced) {
        ESP_LOGE(TAG, "Modem did not respond to AT after %ds total - check GNSS_UART_TX_PIN/"
                 "GNSS_UART_RX_PIN in config.h, and check the board for a physical 4G/modem "
                 "power switch or DIP switch that must be enabled",
                 GNSS_POWERON_WAIT_SEC + (GNSS_AT_SYNC_ATTEMPTS * GNSS_AT_SYNC_RETRY_MS) / 1000);
        return false;
    }

    // Disable command echo (ATE0). With it on (the default), the modem
    // echoes back every byte it receives - including large raw-data
    // payloads sent via AT+CMQTTTOPIC/AT+CMQTTPAYLOAD (see cellular.c). A
    // real device-count JSON report can run well over 1KB, comfortably
    // overflowing a response buffer sized for a normal short AT reply -
    // observed on hardware as an AT+CMQTTPAYLOAD "failure" that's actually
    // just the echoed payload filling the read buffer before the real OK
    // arrives, with the unread remainder leaking into whichever AT exchange
    // reads the UART next (gnss_task's AT+CGNSSINFO poll, in practice).
    // Best-effort - if ATE0 itself fails for some reason, echo just stays
    // on and this class of corruption remains possible.
    if (!modem_uart_send_at("ATE0", resp, sizeof(resp), 2000) || strstr(resp, "OK") == NULL) {
        ESP_LOGW(TAG, "ATE0 (disable echo) failed: %s - command echo may still be on", resp);
    } else {
        ESP_LOGI(TAG, "Command echo disabled (ATE0)");
    }

    // Enable verbose CME error codes (AT+CMEE=2). Without this, most
    // AT+CMQTT* failures just report a bare "ERROR" with no detail; with
    // it, failures report "+CME ERROR: <description>" instead, which the
    // AT RX logging (modem_uart_read_response()) now captures directly -
    // closes a real diagnostic gap hit repeatedly during cellular hardware
    // testing (see plans/4g-integration.md). Best-effort, same as ATE0.
    if (!modem_uart_send_at("AT+CMEE=2", resp, sizeof(resp), 2000) || strstr(resp, "OK") == NULL) {
        ESP_LOGW(TAG, "AT+CMEE=2 (verbose errors) failed: %s - error codes may stay terse", resp);
    } else {
        ESP_LOGI(TAG, "Verbose CME error codes enabled (AT+CMEE=2)");
    }

    return true;
}
