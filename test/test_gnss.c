/**
 * Host-side unit tests for the AT+CGNSSINFO parser in gnss.c.
 *
 * Not part of the ESP-IDF/PlatformIO firmware build. Compile and run with a
 * plain host compiler, e.g.:
 *
 *   gcc -I../include -o /tmp/test_gnss test_gnss.c ../src/gnss.c -lm
 *   /tmp/test_gnss
 */

#include "gnss.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

static int failures = 0;

#define CHECK(cond, msg)                                              \
    do {                                                              \
        if (!(cond)) {                                                \
            printf("FAIL: %s (%s:%d)\n", msg, __FILE__, __LINE__);    \
            failures++;                                               \
        } else {                                                      \
            printf("ok:   %s\n", msg);                                \
        }                                                              \
    } while (0)

static int approx(float a, float b, float eps) {
    return fabsf(a - b) <= eps;
}

int main(void) {
    // A valid fix, 18-field SIM767XX-manual layout (with GALILEO-SVs and a
    // trailing NoSV), decimal-degree lat/lon as returned by CGNSSINFO (not
    // the ddmm.mmmmmm format CGPSINFO uses). Southern/western hemisphere so
    // sign-flip handling is exercised too.
    {
        gnss_fix_t fix = {0};
        bool ok = gnss_parse_cgnssinfo(
            "+CGNSSINFO: 2,09,05,00,00,47.123456,S,122.654321,W,100826,182231.50,123.4,0.0,0.0,1.1,0.8,0.7,8",
            &fix);
        CHECK(ok, "18-field fix line parses");
        CHECK(fix.has_fix, "18-field fix line reports has_fix");
        CHECK(fix.valid, "18-field fix line reports valid");
        CHECK(approx(fix.latitude, -47.123456f, 0.0001f), "latitude sign-flipped for S");
        CHECK(approx(fix.longitude, -122.654321f, 0.0001f), "longitude sign-flipped for W");
        CHECK(approx(fix.altitude_m, 123.4f, 0.01f), "altitude parsed");
        // Best-effort satellite count: sum of the SV-count fields preceding
        // lat (GPS=09, GLONASS=05, GALILEO=00, BEIDOU=00), not a trailing
        // NoSV field (not reliably present across firmware variants).
        CHECK(fix.num_satellites == 14, "num_satellites summed from SV-count fields");

        struct tm expected = {0};
        expected.tm_year = 2026 - 1900;
        expected.tm_mon = 8 - 1;
        expected.tm_mday = 10;
        expected.tm_hour = 18;
        expected.tm_min = 22;
        expected.tm_sec = 31;
        time_t expected_time = timegm(&expected);
        CHECK(fix.fix_time == expected_time, "fix_time parsed from ddmmyy + hhmmss.ss (UTC)");
    }

    // Northern/eastern hemisphere - no sign flip.
    {
        gnss_fix_t fix = {0};
        bool ok = gnss_parse_cgnssinfo(
            "+CGNSSINFO: 3,09,05,00,00,31.222177,N,121.354376,E,131117,091918.00,32.9,0.0,255.0,1.1,0.8,0.7,14",
            &fix);
        CHECK(ok, "N/E fix line parses");
        CHECK(fix.has_fix, "N/E fix line reports has_fix");
        CHECK(approx(fix.latitude, 31.222177f, 0.0001f), "latitude positive for N");
        CHECK(approx(fix.longitude, 121.354376f, 0.0001f), "longitude positive for E");
    }

    // 16-field A76XX-manual layout (no GALILEO-SVs, no trailing NoSV) - a
    // different SIMCom firmware/manual revision from the 18-field case
    // above. Must parse via the same lat/N-S/lon/E-W scan, not a fixed index.
    {
        gnss_fix_t fix = {0};
        bool ok = gnss_parse_cgnssinfo(
            "+CGNSSINFO: 2,09,05,00,31.222177,N,121.354376,E,131117,091918.00,32.9,0.0,255.0,1.1,0.8,0.7",
            &fix);
        CHECK(ok, "16-field (no GALILEO) fix line parses");
        CHECK(fix.has_fix, "16-field fix line reports has_fix");
        CHECK(approx(fix.latitude, 31.222177f, 0.0001f), "16-field layout: latitude parsed");
        CHECK(approx(fix.longitude, 121.354376f, 0.0001f), "16-field layout: longitude parsed");
        CHECK(approx(fix.altitude_m, 32.9f, 0.01f), "16-field layout: altitude parsed");
    }

    // No-fix response, fully padded to 18 empty fields (as documented in
    // the SIM767XX manual).
    {
        gnss_fix_t fix = {0};
        fix.valid = true;       // simulate a previously-known fix
        fix.latitude = 1.0f;
        fix.longitude = 2.0f;
        bool ok = gnss_parse_cgnssinfo("+CGNSSINFO:,,,,,,,,,,,,,,,,,", &fix);
        CHECK(ok, "fully-padded no-fix line parses without error");
        CHECK(!fix.has_fix, "fully-padded no-fix line reports has_fix=false");
        CHECK(fix.valid, "no-fix line preserves previously-known valid flag");
        CHECK(approx(fix.latitude, 1.0f, 0.0001f), "no-fix line preserves previously-known latitude");
    }

    // No-fix response as actually observed on hardware: only 9 fields
    // (trailing empties truncated rather than padded to the full count).
    {
        gnss_fix_t fix = {0};
        bool ok = gnss_parse_cgnssinfo("+CGNSSINFO: ,,,,,,,,", &fix);
        CHECK(ok, "hardware-observed short no-fix line parses without error");
        CHECK(!fix.has_fix, "hardware-observed short no-fix line reports has_fix=false");
    }

    // Malformed line: too few fields to even plausibly contain a fix -
    // must not crash, must fail cleanly.
    {
        gnss_fix_t fix = {0};
        bool ok = gnss_parse_cgnssinfo("+CGNSSINFO: 2,09,05", &fix);
        CHECK(!ok, "truncated line claiming a fix but missing the position block is rejected");
    }

    // A stray extra trailing field (firmware quirk / future field addition)
    // must not break parsing of the fields we do understand.
    {
        gnss_fix_t fix = {0};
        bool ok = gnss_parse_cgnssinfo(
            "+CGNSSINFO: 2,09,05,00,00,47.1,N,122.6,W,100826,182231.50,123.4,0.0,0.0,1.1,0.8,0.7,8,9",
            &fix);
        CHECK(ok, "line with a stray extra trailing field still parses");
        CHECK(fix.has_fix, "line with a stray extra trailing field reports has_fix");
        CHECK(approx(fix.latitude, 47.1f, 0.0001f), "line with a stray extra trailing field: latitude parsed");
    }

    // NULL-safety.
    {
        CHECK(!gnss_parse_cgnssinfo(NULL, NULL), "NULL line/out rejected without crashing");
    }

    printf("\n%d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
