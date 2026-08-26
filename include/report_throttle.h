/**
 * Delta/heartbeat send throttling, shared by both cellular transports
 *
 * Extracted out of cellular.c (where it started as cellular_should_send())
 * once REPORT_TRANSPORT_CELLULAR_HTTP (cellular_http.c) needed the exact
 * same decision - skip a send when nothing meaningful changed since the
 * last one actually went out, but never skip more than
 * CELLULAR_HEARTBEAT_INTERVAL_SEC in a row. Cellular MQTT and cellular HTTP
 * are mutually exclusive transports (REPORT_TRANSPORT), so there's no
 * concurrent access to worry about - each just keeps its own
 * report_throttle_state_t instance, since "last sent" means something
 * different per transport.
 *
 * Pure logic, no FreeRTOS/ESP-IDF includes - host-testable, see
 * test/test_report_throttle.c.
 */

#ifndef REPORT_THROTTLE_H
#define REPORT_THROTTLE_H

#include <stdbool.h>
#include <time.h>

#include "gnss.h"

/**
 * State of the last report actually *sent* (not just considered/attempted)
 * by one transport. Compared against each new cycle's fix/phone_count by
 * report_throttle_should_send() to decide whether this one is worth sending
 * too - mirrors exactly what went into that last payload, not just the raw
 * GNSS state, so "unchanged" always means "unchanged from what the server
 * was last told". Zero-initialize (or use a `static` instance, which
 * zero-inits automatically) before first use.
 */
typedef struct {
    bool last_sent_ever;
    bool last_sent_position_valid;
    float last_sent_lat;
    float last_sent_lon;
    int last_sent_phone_count;
    time_t last_sent_time;
} report_throttle_state_t;

/**
 * Decide whether this cycle's fix/phone_count is worth a send, per
 * CELLULAR_POSITION_UNCHANGED_THRESHOLD_M/CELLULAR_HEARTBEAT_INTERVAL_SEC
 * (config.h) - Ian's explicit direction to skip a send when nothing
 * meaningful changed since the last one actually went out, but never skip
 * more than an hour's worth in a row. Compares against state (what was last
 * actually published, per report_throttle_record_sent()), not the previous
 * cycle's fix - a run of cycles that are each individually unchanged from
 * the one before, but drift past the threshold cumulatively, still triggers
 * a send.
 * Always sends: the very first report ever, whenever the phone count
 * differs, whenever "do we have a known position at all" flips (gained or
 * lost a fix since the last send), or whenever more than
 * CELLULAR_HEARTBEAT_INTERVAL_SEC has elapsed since the last send
 * regardless of anything else.
 */
bool report_throttle_should_send(const report_throttle_state_t *state,
                                  const gnss_fix_t *fix, int phone_count);

/**
 * Record that a report was actually sent - call this only after a
 * successful send, never after one that was skipped or failed. Updates
 * state so the *next* call to report_throttle_should_send() compares
 * against what actually went out, not against whatever this cycle merely
 * considered.
 */
void report_throttle_record_sent(report_throttle_state_t *state,
                                  const gnss_fix_t *fix, int phone_count);

#endif // REPORT_THROTTLE_H
