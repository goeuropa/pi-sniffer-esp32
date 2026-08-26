/**
 * Delta/heartbeat send throttling, shared by both cellular transports
 *
 * See report_throttle.h. Pure logic - no FreeRTOS/ESP-IDF includes - so it
 * builds and runs the same way on the host (test/test_report_throttle.c) as
 * it does cross-compiled into the firmware.
 */

#include "report_throttle.h"
#include "config.h"

#include <math.h>

/**
 * Straight-line ground distance between two lat/lon points, in meters.
 * Equirectangular approximation (flat-earth, scaled by degrees-to-meters at
 * the mean latitude) rather than full haversine - plenty accurate at the
 * few-meters scale CELLULAR_POSITION_UNCHANGED_THRESHOLD_M cares about, and
 * cheap enough to call every cycle.
 */
static float report_throttle_distance_m(float lat1, float lon1, float lat2, float lon2) {
    const float deg_to_rad = 0.017453293f; // pi / 180
    const float m_per_deg_lat = 111320.0f; // ~constant; longitude scales by cos(lat)
    float mean_lat_rad = (lat1 + lat2) * 0.5f * deg_to_rad;
    float dlat_m = (lat2 - lat1) * m_per_deg_lat;
    float dlon_m = (lon2 - lon1) * m_per_deg_lat * cosf(mean_lat_rad);
    return sqrtf(dlat_m * dlat_m + dlon_m * dlon_m);
}

bool report_throttle_should_send(const report_throttle_state_t *state,
                                  const gnss_fix_t *fix, int phone_count) {
    if (!state->last_sent_ever) {
        return true;
    }

    if (time(NULL) - state->last_sent_time >= CELLULAR_HEARTBEAT_INTERVAL_SEC) {
        return true;
    }

    if (phone_count != state->last_sent_phone_count) {
        return true;
    }

    if (fix->valid != state->last_sent_position_valid) {
        return true; // gained or lost a known position since the last send
    }

    if (fix->valid) {
        float moved_m = report_throttle_distance_m(state->last_sent_lat, state->last_sent_lon,
                                                     fix->latitude, fix->longitude);
        if (moved_m > CELLULAR_POSITION_UNCHANGED_THRESHOLD_M) {
            return true;
        }
    }

    return false;
}

void report_throttle_record_sent(report_throttle_state_t *state,
                                  const gnss_fix_t *fix, int phone_count) {
    state->last_sent_ever = true;
    state->last_sent_time = time(NULL);
    state->last_sent_phone_count = phone_count;
    state->last_sent_position_valid = fix->valid;
    state->last_sent_lat = fix->latitude;
    state->last_sent_lon = fix->longitude;
}
