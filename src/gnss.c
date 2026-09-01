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

#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "modem_uart.h"

static const char *TAG = "GNSS";

static gnss_fix_t s_last_fix = {0};
static portMUX_TYPE s_fix_lock = portMUX_INITIALIZER_UNLOCKED;

#define GNSS_AT_BUF_SIZE 512
#define GNSS_AT_TIMEOUT_MS 5000

bool gnss_init(void) {
    // UART setup and the initial "AT" handshake are shared with cellular.c
    // over the modem's single AT command port - see modem_uart.h. Safe to
    // call even if cellular.c has already brought the link up.
    if (!modem_uart_init()) {
        return false;
    }

    char resp[GNSS_AT_BUF_SIZE];
    if (!modem_uart_send_at("AT+CGNSSPWR=1", resp, sizeof(resp), GNSS_AT_TIMEOUT_MS) ||
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

/**
 * Discipline the ESP32's system clock from a GNSS fix's own UTC date/time,
 * so units with no WiFi (hence no SNTP - see init_sntp() in main.c) still
 * get a correct wall clock, and any unit gets one faster/more reliably than
 * waiting on SNTP at all. Called from gnss_poll_now() whenever a fix
 * carries a parsed fix_time.
 *
 * Sanity-checks the date first (same tm_year idiom init_sntp() already uses
 * to detect an unsynced clock) rather than trusting a malformed
 * AT+CGNSSINFO date field outright - settimeofday() with garbage would be
 * worse than not syncing at all. Skips the actual settimeofday() call (and
 * its log line) once the clock is already within
 * GNSS_CLOCK_SYNC_THRESHOLD_SEC of GPS time, so a clock that's already
 * accurate (e.g. via SNTP) isn't needlessly nudged every report cycle.
 */
static void gnss_maybe_sync_system_clock(time_t gps_time) {
    struct tm tm_utc;
    if (gmtime_r(&gps_time, &tm_utc) == NULL || tm_utc.tm_year + 1900 < 2024) {
        ESP_LOGW(TAG, "GNSS: implausible fix date, not syncing system clock");
        return;
    }

    time_t now = time(NULL);
    long delta = (long)(gps_time - now);
    if (labs(delta) < GNSS_CLOCK_SYNC_THRESHOLD_SEC) {
        return;
    }

    struct timeval tv = { .tv_sec = gps_time, .tv_usec = 0 };
    if (settimeofday(&tv, NULL) == 0) {
        ESP_LOGI(TAG, "System clock synced from GPS (was off by %lds)", delta);
    } else {
        ESP_LOGW(TAG, "settimeofday() from GPS fix failed");
    }
}

bool gnss_poll_now(void) {
    char resp[GNSS_AT_BUF_SIZE];

    bool got = modem_uart_send_at("AT+CGNSSINFO", resp, sizeof(resp), GNSS_AT_TIMEOUT_MS);
    if (!got) {
        ESP_LOGW(TAG, "GNSS: AT+CGNSSINFO timed out");
        return false;
    }

    // Read the current fix (preserves last-known position/valid flag
    // across polls that don't have a fix - see gnss_parse_cgnssinfo).
    gnss_fix_t fix;
    taskENTER_CRITICAL(&s_fix_lock);
    fix = s_last_fix;
    taskEXIT_CRITICAL(&s_fix_lock);

    if (!gnss_parse_cgnssinfo(resp, &fix)) {
        ESP_LOGW(TAG, "GNSS: unexpected CGNSSINFO response: %s", resp);
        return false;
    }

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

        if (fix.fix_time > 0) {
            gnss_maybe_sync_system_clock(fix.fix_time);
        }
    } else {
        ESP_LOGI(TAG, "GNSS: no fix yet");
    }

    return true;
}

#endif // ESP_PLATFORM
