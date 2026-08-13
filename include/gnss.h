/**
 * GNSS Interface (SIM7670G onboard modem)
 *
 * Polls the SIM7670G's GNSS receiver over UART via AT commands
 * (AT+CGNSSPWR, AT+CGNSSINFO) and keeps track of the most recent fix.
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
 * Start the background task that polls AT+CGNSSINFO on
 * GNSS_POLL_INTERVAL_SEC and logs the result.
 */
void gnss_start_task(void);

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
