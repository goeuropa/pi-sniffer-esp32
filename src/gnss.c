/**
 * GNSS Implementation (SIM7670G onboard modem)
 *
 * Talks to the modem's AT command port over UART to power on the GNSS
 * receiver and periodically poll it for a fix.
 */

#include "gnss.h"
#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// ----------------------------------------------------------------------
// Pure parsing/time helpers - no ESP-IDF dependency, so this half of the
// file can be compiled and unit tested on the host (see test/test_gnss.c).
// ----------------------------------------------------------------------

// +CGNSSINFO's documented field layout is
// mode,GPS-SVs,GLONASS-SVs,[GALILEO-SVs],BEIDOU-SVs,lat,N/S,lon,E/W,date,
// UTC-time,alt,speed,course,PDOP,HDOP,VDOP,[NoSV] - but the exact field
// count varies by SIMCom firmware/manual revision (18 fields with GALILEO
// and NoSV per the SIM767XX manual, 16 without per the A76XX manual), and
// real hardware has been observed reporting a no-fix line with only 9
// fields (trailing empties apparently truncated rather than padded out).
// Rather than hardcode a field count, locate the stable lat,N/S,lon,E/W
// block by content (the N/S and E/W indicators are unambiguous single
// characters) so this works across firmware variants and isn't sensitive to
// how many satellite-count or DOP fields surround it.
#define GNSS_CGNSSINFO_MAX_FIELDS 24

/**
 * Split buf in place on commas into up to max_fields fields, replacing each
 * comma with '\0'. Unlike strtok, this preserves empty fields (",,," yields
 * three empty fields), which AT+CGNSSINFO relies on for its no-fix response.
 */
static int split_fields(char *buf, char *fields[], int max_fields) {
    int count = 0;
    char *p = buf;
    fields[count++] = p;
    while (*p != '\0' && count < max_fields) {
        if (*p == ',') {
            *p = '\0';
            fields[count++] = p + 1;
        }
        p++;
    }
    return count;
}

/**
 * Convert a UTC calendar date/time to time_t without relying on timegm()
 * (not universally available on ESP-IDF's newlib) or the local timezone.
 * Uses Howard Hinnant's days_from_civil algorithm.
 */
static time_t utc_mktime(int year, int month, int day, int hour, int min, int sec) {
    year -= (month <= 2) ? 1 : 0;
    long era = (year >= 0 ? year : year - 399) / 400;
    unsigned yoe = (unsigned)(year - era * 400);              // [0, 399]
    unsigned mp = (unsigned)(month + (month > 2 ? -3 : 9));    // [0, 11]
    unsigned doy = (153 * mp + 2) / 5 + (unsigned)day - 1;     // [0, 365]
    unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;      // [0, 146096]
    long days = era * 146097L + (long)doe - 719468L;           // days since 1970-01-01

    return (time_t)(days * 86400L + hour * 3600L + min * 60L + sec);
}

/**
 * Parse a CGNSSINFO <date> (ddmmyy) and <UTC-time> (hhmmss.ss) field pair
 * into a UTC time_t. Returns 0 if either field is too short to parse.
 */
static time_t parse_gnss_datetime(const char *date_field, const char *time_field) {
    if (strlen(date_field) < 6 || strlen(time_field) < 6) {
        return 0;
    }

    char tmp[3] = {0};

    tmp[0] = date_field[0]; tmp[1] = date_field[1];
    int day = atoi(tmp);
    tmp[0] = date_field[2]; tmp[1] = date_field[3];
    int month = atoi(tmp);
    tmp[0] = date_field[4]; tmp[1] = date_field[5];
    int year = atoi(tmp) + 2000;

    tmp[0] = time_field[0]; tmp[1] = time_field[1];
    int hour = atoi(tmp);
    tmp[0] = time_field[2]; tmp[1] = time_field[3];
    int minute = atoi(tmp);
    tmp[0] = time_field[4]; tmp[1] = time_field[5];
    int second = atoi(tmp);

    return utc_mktime(year, month, day, hour, minute, second);
}

bool gnss_parse_cgnssinfo(const char *line, gnss_fix_t *out) {
    if (line == NULL || out == NULL) {
        return false;
    }

    // Skip a leading "+CGNSSINFO:" prefix (and any whitespace after it), if present -
    // callers may pass either the raw modem line or just the field list.
    const char *prefix = strstr(line, "CGNSSINFO:");
    const char *start = (prefix != NULL) ? prefix + strlen("CGNSSINFO:") : line;
    while (*start == ' ') {
        start++;
    }

    char buf[160];
    size_t len = strlen(start);
    if (len >= sizeof(buf)) {
        len = sizeof(buf) - 1;
    }
    memcpy(buf, start, len);
    buf[len] = '\0';
    // Trim trailing CR/LF left over from the modem's line terminator.
    while (len > 0 && (buf[len - 1] == '\r' || buf[len - 1] == '\n')) {
        buf[--len] = '\0';
    }

    char *fields[GNSS_CGNSSINFO_MAX_FIELDS];
    int count = split_fields(buf, fields, GNSS_CGNSSINFO_MAX_FIELDS);
    if (count < 2) {
        return false;
    }

    // <mode> is empty on a no-fix response, whatever the firmware's field
    // count/padding for the rest of the line happens to be.
    bool has_fix = (fields[0][0] != '\0');
    out->has_fix = has_fix;
    out->last_poll_time = time(NULL);

    if (!has_fix) {
        // No fix yet - leave any previously-known position/valid flag alone.
        return true;
    }

    // Locate the lat,N/S,lon,E/W block: scan for a lone "N"/"S" field
    // preceded by a numeric-looking field (the latitude) - this is the same
    // regardless of how many satellite-count fields (GPS/GLONASS/GALILEO/
    // BEIDOU) precede it. Start at 2 so the mode field itself can't match.
    int ns_idx = -1;
    for (int i = 2; i + 2 < count; i++) {
        bool is_ns = (fields[i][0] == 'N' || fields[i][0] == 'S') && fields[i][1] == '\0';
        if (is_ns && strpbrk(fields[i - 1], "0123456789") != NULL) {
            ns_idx = i;
            break;
        }
    }
    if (ns_idx < 0) {
        // Claimed a fix but the position block couldn't be found - treat as
        // an unrecognized/malformed line rather than guessing.
        return false;
    }

    int lat_idx = ns_idx - 1;
    int lon_idx = ns_idx + 1;
    int ew_idx = ns_idx + 2;
    bool is_ew = (fields[ew_idx][0] == 'E' || fields[ew_idx][0] == 'W') && fields[ew_idx][1] == '\0';
    if (!is_ew) {
        return false;
    }

    // date/UTC-time/altitude follow E/W, but may be absent on some
    // firmware/truncated lines - treat missing fields as empty rather than
    // erroring, so at least lat/lon/hemisphere still come through.
    int date_idx = ew_idx + 1;
    int time_idx = ew_idx + 2;
    int alt_idx = ew_idx + 3;
    const char *date_field = (date_idx < count) ? fields[date_idx] : "";
    const char *time_field = (time_idx < count) ? fields[time_idx] : "";
    const char *alt_field = (alt_idx < count) ? fields[alt_idx] : "";

    float lat = strtof(fields[lat_idx], NULL);
    if (fields[ns_idx][0] == 'S') {
        lat = -lat;
    }

    float lon = strtof(fields[lon_idx], NULL);
    if (fields[ew_idx][0] == 'W') {
        lon = -lon;
    }

    // Sum whatever satellite-count fields precede lat (GPS/GLONASS/
    // [GALILEO]/BEIDOU - the exact set varies by firmware) as a best-effort
    // visible-satellite count; a trailing NoSV field isn't reliably present.
    int svs_sum = 0;
    for (int i = 1; i < lat_idx; i++) {
        svs_sum += atoi(fields[i]);
    }

    out->valid = true;
    out->latitude = lat;
    out->longitude = lon;
    out->altitude_m = strtof(alt_field, NULL);
    out->num_satellites = (uint8_t)svs_sum;
    out->fix_time = parse_gnss_datetime(date_field, time_field);

    return true;
}

// ----------------------------------------------------------------------
// UART / AT command driver - ESP-IDF only, not compiled into host tests.
// ----------------------------------------------------------------------
#ifdef ESP_PLATFORM

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "GNSS";

static gnss_fix_t s_last_fix = {0};
static portMUX_TYPE s_fix_lock = portMUX_INITIALIZER_UNLOCKED;

#define GNSS_AT_BUF_SIZE 512
#define GNSS_AT_TIMEOUT_MS 5000

/**
 * Write an AT command (CR/LF appended) and read back the modem's response
 * until "OK"/"ERROR" is seen or the timeout elapses.
 * @return number of bytes read into resp (0-terminated), or -1 on failure/timeout
 */
static int gnss_send_at(const char *cmd, char *resp, size_t resp_size, int timeout_ms) {
    char line[128];
    int cmd_len = snprintf(line, sizeof(line), "%s\r\n", cmd);
    uart_flush_input(GNSS_UART_NUM);
    uart_write_bytes(GNSS_UART_NUM, line, cmd_len);

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
        return -1;
    }
    return (int)total;
}

bool gnss_init(void) {
    uart_config_t uart_config = {
        .baud_rate = GNSS_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_driver_install(GNSS_UART_NUM, GNSS_AT_BUF_SIZE * 2, 0, 0, NULL, 0);
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
    char resp[GNSS_AT_BUF_SIZE];
    bool synced = false;
    for (int attempt = 0; attempt < GNSS_AT_SYNC_ATTEMPTS; attempt++) {
        int n = gnss_send_at("AT", resp, sizeof(resp), 1000);
        if (n > 0 && strstr(resp, "OK") != NULL) {
            synced = true;
            break;
        }
        if (n > 0) {
            char hex[3 * 32 + 1] = "";
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
    if (!synced) {
        ESP_LOGE(TAG, "Modem did not respond to AT after %ds total - check GNSS_UART_TX_PIN/"
                 "GNSS_UART_RX_PIN in config.h, and check the board for a physical 4G/modem "
                 "power switch or DIP switch that must be enabled",
                 GNSS_POWERON_WAIT_SEC + (GNSS_AT_SYNC_ATTEMPTS * GNSS_AT_SYNC_RETRY_MS) / 1000);
        return false;
    }

    if (gnss_send_at("AT+CGNSSPWR=1", resp, sizeof(resp), GNSS_AT_TIMEOUT_MS) < 0 ||
        strstr(resp, "OK") == NULL) {
        ESP_LOGE(TAG, "AT+CGNSSPWR=1 failed: %s", resp);
        return false;
    }

    ESP_LOGI(TAG, "GNSS powered on");
    return true;
}

static void gnss_update_fix(const gnss_fix_t *fix) {
    taskENTER_CRITICAL(&s_fix_lock);
    s_last_fix = *fix;
    taskEXIT_CRITICAL(&s_fix_lock);
}

bool gnss_get_last_fix(gnss_fix_t *out) {
    if (out == NULL) {
        return false;
    }
    taskENTER_CRITICAL(&s_fix_lock);
    *out = s_last_fix;
    taskEXIT_CRITICAL(&s_fix_lock);
    return out->valid;
}

static void gnss_task(void *pvParameters) {
    char resp[GNSS_AT_BUF_SIZE];

    while (1) {
        int n = gnss_send_at("AT+CGNSSINFO", resp, sizeof(resp), GNSS_AT_TIMEOUT_MS);
        if (n > 0) {
            // Read the current fix (preserves last-known position/valid flag
            // across polls that don't have a fix - see gnss_parse_cgnssinfo).
            gnss_fix_t fix;
            taskENTER_CRITICAL(&s_fix_lock);
            fix = s_last_fix;
            taskEXIT_CRITICAL(&s_fix_lock);

            if (gnss_parse_cgnssinfo(resp, &fix)) {
                gnss_update_fix(&fix);

                if (fix.has_fix) {
                    char time_buf[24] = "?";
                    struct tm tm_utc;
                    if (fix.fix_time > 0 && gmtime_r(&fix.fix_time, &tm_utc) != NULL) {
                        strftime(time_buf, sizeof(time_buf), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
                    }
                    ESP_LOGI(TAG, "GNSS fix: lat=%.6f lon=%.6f alt=%.1fm sats=%d time=%s",
                              fix.latitude, fix.longitude, fix.altitude_m,
                              fix.num_satellites, time_buf);
                } else {
                    ESP_LOGI(TAG, "GNSS: no fix yet");
                }
            } else {
                ESP_LOGW(TAG, "GNSS: unexpected CGNSSINFO response: %s", resp);
            }
        } else {
            ESP_LOGW(TAG, "GNSS: AT+CGNSSINFO timed out");
        }

        vTaskDelay(pdMS_TO_TICKS(GNSS_POLL_INTERVAL_SEC * 1000));
    }
}

void gnss_start_task(void) {
    xTaskCreate(gnss_task, "gnss_task", 4096, NULL, 3, NULL);
}

#endif // ESP_PLATFORM
