/**
 * GNSS Interface (SIM7670G onboard modem)
 *
 * Polls the SIM7670G's GNSS receiver over UART via AT commands
 * (AT+CGNSSPWR, AT+CGNSSINFO) and keeps track of the most recent fix.
 * Also disciplines the ESP32's system clock (settimeofday()) from each
 * fix's own UTC date/time - see gnss_poll_now() - so units with no WiFi
 * (hence no SNTP) still get a correct wall clock.
 *
 * Deliberately has no independent polling task/timer - gnss_poll_now()
 * must be called explicitly by whoever needs a fresh fix. It shares the
 * modem's single AT command port with cellular.c (see modem_uart.h); the
 * sole caller is report_task() (main.c), once per report cycle,
 * unconditionally (whenever ENABLE_GNSS=1) regardless of which transport
 * (WiFi or cellular) is actually configured to send - keeping GNSS and
 * cellular AT-command activity to one call site apiece rather than
 * interleaved on independent schedules (an earlier independent GNSS timer
 * caused exactly that during hardware testing - see
 * plans/4g-integration.md). cellular_task itself never touches GNSS: it
 * receives whatever fix report_task already polled, passed through
 * cellular_publish().
 */

#ifndef GNSS_H
#define GNSS_H

#include <stdbool.h>
#include <stdint.h>
#include <time.h>

/**
 * Most recently polled GNSS fix.
 */
typedef struct {
    bool     valid;          // true once a fix has ever been acquired
    bool     has_fix;        // true if the *last* poll returned a fix
    float    latitude;       // decimal degrees, signed (+N/-S)
    float    longitude;      // decimal degrees, signed (+E/-W)
    float    altitude_m;     // MSL altitude, meters
    uint8_t  num_satellites; // satellites used in the last fix (NoSV)
    time_t   fix_time;       // UTC time of the fix, from the modem's date+time fields
    time_t   last_poll_time; // local time we last attempted a poll
} gnss_fix_t;

/**
 * Configure the UART link to the modem and power on GNSS.
 * @return true on success (modem responded to AT and GNSS power-on succeeded)
 */
bool gnss_init(void);

/**
 * Poll AT+CGNSSINFO once, synchronously, updating the cached fix
 * (gnss_get_last_fix()) and logging the result. Call this explicitly right
 * before you need a fresh fix - there's no background task doing this on a
 * timer (see this header's top comment for why). Also disciplines the
 * system clock from the fix's UTC date/time when one comes back valid and
 * plausible - see gnss_maybe_sync_system_clock() in gnss.c.
 * @return true if the AT+CGNSSINFO exchange itself completed (regardless of
 *         whether it reported an actual fix - check gnss_get_last_fix()
 *         separately for that)
 */
bool gnss_poll_now(void);

/**
 * Copy out the most recently polled fix.
 * @return true if a fix has ever been acquired (mirrors out->valid)
 */
bool gnss_get_last_fix(gnss_fix_t *out);

/**
 * Parse a single AT+CGNSSINFO response line, e.g.:
 *   +CGNSSINFO: 2,09,05,00,00,47.12345,N,122.12345,W,100826,182231.00,123.4,0.0,0.0,1.1,0.8,0.7,8
 * or the no-fix form:
 *   +CGNSSINFO:,,,,,,,,,,,,,,,,,
 *
 * Exposed for unit testing - does not touch the UART.
 *
 * @param line Response line (with or without leading "+CGNSSINFO:" prefix)
 * @param out  Filled in on success; out->has_fix reflects whether the line
 *             carried a fix. out->valid/latitude/longitude/altitude_m/
 *             num_satellites/fix_time are only updated when out->has_fix.
 * @return true if the line was a well-formed CGNSSINFO response (fix or no-fix),
 *         false if it couldn't be parsed at all (wrong field count, etc.)
 */
bool gnss_parse_cgnssinfo(const char *line, gnss_fix_t *out);

#endif // GNSS_H
