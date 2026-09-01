/**
 * Host-side unit tests for report_throttle.c's should-send/heartbeat logic -
 * extracted from cellular.c so both cellular MQTT and cellular HTTP
 * reporting (cellular_http.c) share one implementation, see
 * report_throttle.h.
 *
 * Not part of the ESP-IDF/PlatformIO firmware build. Compile and run with a
 * plain host compiler, e.g.:
 *
 *   gcc -I../include -o /tmp/test_report_throttle test_report_throttle.c \
 *       ../src/report_throttle.c -lm
 *   /tmp/test_report_throttle
 */

#include "report_throttle.h"
#include "config.h" // CELLULAR_HEARTBEAT_INTERVAL_SEC / CELLULAR_POSITION_UNCHANGED_THRESHOLD_M

#include <stdio.h>
#include <string.h>

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

static gnss_fix_t make_fix(bool valid, float lat, float lon) {
    gnss_fix_t fix = {0};
    fix.valid = valid;
    fix.latitude = lat;
    fix.longitude = lon;
    return fix;
}

int main(void) {
    // First-ever report is always worth sending, regardless of state.
    {
        report_throttle_state_t state = {0};
        gnss_fix_t fix = make_fix(true, 47.5853f, -122.0644f);
        CHECK(report_throttle_should_send(&state, &fix, 3) == true,
              "first-ever report always sends");
    }

    // Unchanged position/phone count, well within CELLULAR_HEARTBEAT_INTERVAL_SEC
    // of the last send, does not send again.
    {
        report_throttle_state_t state = {0};
        gnss_fix_t fix = make_fix(true, 47.5853f, -122.0644f);
        report_throttle_record_sent(&state, &fix, 3);
        CHECK(report_throttle_should_send(&state, &fix, 3) == false,
              "unchanged fix/phone_count does not re-send immediately");
    }

    // Phone count change always sends, even with identical position.
    {
        report_throttle_state_t state = {0};
        gnss_fix_t fix = make_fix(true, 47.5853f, -122.0644f);
        report_throttle_record_sent(&state, &fix, 3);
        CHECK(report_throttle_should_send(&state, &fix, 4) == true,
              "phone count change always sends");
    }

    // Gaining/losing a fix (valid flips) always sends, even at the same
    // last-known coordinates.
    {
        report_throttle_state_t state = {0};
        gnss_fix_t fix_valid = make_fix(true, 47.5853f, -122.0644f);
        report_throttle_record_sent(&state, &fix_valid, 3);
        gnss_fix_t fix_lost = make_fix(false, 47.5853f, -122.0644f);
        CHECK(report_throttle_should_send(&state, &fix_lost, 3) == true,
              "losing a fix always sends");

        report_throttle_state_t state2 = {0};
        gnss_fix_t fix_novalid = make_fix(false, 0.0f, 0.0f);
        report_throttle_record_sent(&state2, &fix_novalid, 3);
        CHECK(report_throttle_should_send(&state2, &fix_valid, 3) == true,
              "gaining a fix always sends");
    }

    // Movement below CELLULAR_POSITION_UNCHANGED_THRESHOLD_M (20m) does not
    // trigger a send; movement past it does.
    {
        report_throttle_state_t state = {0};
        gnss_fix_t origin = make_fix(true, 47.58530f, -122.06440f);
        report_throttle_record_sent(&state, &origin, 3);

        // ~0.00005 degrees of latitude is roughly 5.5m - well under threshold.
        gnss_fix_t nearby = make_fix(true, 47.58535f, -122.06440f);
        CHECK(report_throttle_should_send(&state, &nearby, 3) == false,
              "small movement under threshold does not send");

        // ~0.001 degrees of latitude is roughly 111m - well past threshold.
        gnss_fix_t far = make_fix(true, 47.58630f, -122.06440f);
        CHECK(report_throttle_should_send(&state, &far, 3) == true,
              "movement past CELLULAR_POSITION_UNCHANGED_THRESHOLD_M sends");
    }

    // Heartbeat: even with nothing changed, a send is forced once
    // CELLULAR_HEARTBEAT_INTERVAL_SEC has elapsed since the last one.
    {
        report_throttle_state_t state = {0};
        gnss_fix_t fix = make_fix(true, 47.5853f, -122.0644f);
        report_throttle_record_sent(&state, &fix, 3);
        // Simulate time passing by rewinding last_sent_time directly - this
        // is exactly what report_throttle_should_send() compares against
        // time(NULL), so backdating it is equivalent to waiting for real.
        state.last_sent_time -= CELLULAR_HEARTBEAT_INTERVAL_SEC + 1;
        CHECK(report_throttle_should_send(&state, &fix, 3) == true,
              "heartbeat interval elapsed forces a send even if unchanged");
    }

    printf("\n%d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
