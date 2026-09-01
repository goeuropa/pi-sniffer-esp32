/**
 * Device Report JSON Payload Builder
 *
 * Builds the JSON body reported to the server, shared by every transport -
 * the legacy REST path (http_client.c), WiFi MQTT (mqtt_report.c), and
 * cellular MQTT (cellular.c) all publish/POST the same payload shape, per
 * plans/4g-integration.md's "one payload/topic schema, one code path
 * server-side" goal.
 */

#ifndef DEVICE_JSON_H
#define DEVICE_JSON_H

#include <stdbool.h>
#include <time.h>

#include "device.h"

/**
 * Build the device-report JSON payload for this sensor's current device
 * list. Not used by the live report path (see device_json_build_minimal())
 * - kept in the tree in case full per-device detail is wanted again later
 * (e.g. a local-network-only debug mode).
 * @param list Pointer to the device list
 * @param device_id Unique identifier for this sensor
 * @return Newly allocated, NUL-terminated JSON string (caller must free()),
 *         or NULL on allocation failure
 */
char *device_json_build(const device_list_t *list, const char *device_id);

/**
 * Build the minimal report payload - position and phone count, not
 * per-device detail. Shared by both transports: report_task() (main.c)
 * polls GNSS once per report cycle, synchronously, regardless of which
 * transport (WiFi or cellular) is actually configured to send, and passes
 * that same fix through to whichever one it is - see
 * plans/4g-integration.md. Doesn't touch device_list_t directly -
 * phone_count is the caller's already-computed device_summary_t.phones.
 * @param has_fix Whether a GNSS fix is currently (or was ever) available
 *                (gnss_fix_t.valid, from gnss_get_last_fix()) - sticky, so
 *                this stays true (with the last known position) even
 *                across polls that come back with no current fix
 * @param latitude Decimal degrees, signed (+N/-S) - ignored if !has_fix
 * @param longitude Decimal degrees, signed (+E/-W) - ignored if !has_fix
 * @param fix_time UTC time of that GNSS fix (gnss_fix_t.fix_time), used as
 *                  the payload's "timestamp" field (an ISO 8601 string, not
 *                  a number) whenever has_fix and fix_time is set - i.e. the
 *                  timestamp reported is the GPS's own clock, not the
 *                  ESP32's. Falls back to the ESP32's system clock (still
 *                  ISO 8601) when there's no fix yet (e.g. ENABLE_GNSS=0,
 *                  or no fix acquired yet) or the modem's CGNSSINFO
 *                  date/time fields were unparseable (gnss_parse_cgnssinfo()
 *                  leaves fix_time at 0 in that case) - this doubles as the
 *                  "GNSS isn't available" fallback: last known position (if
 *                  any) plus current system time.
 * @param phone_count Current phone count (device_summary_t.phones)
 * @param device_id Unique identifier for this sensor
 * @return Newly allocated, NUL-terminated JSON string (caller must free()),
 *         or NULL on allocation failure
 */
char *device_json_build_minimal(bool has_fix, float latitude, float longitude,
                                 time_t fix_time, int phone_count, const char *device_id);

#endif // DEVICE_JSON_H
