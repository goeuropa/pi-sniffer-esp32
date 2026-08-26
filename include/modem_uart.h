/**
 * Shared UART/AT-command transport to the SIM7670G modem
 *
 * The modem exposes one AT command port to the ESP32-S3, and more than one
 * module needs to issue short request/response AT exchanges on it - gnss.c
 * (AT+CGNSSINFO polling) and cellular.c (AT+CGDCONT/AT+CEREG?/AT+CMQTT*).
 * Rather than CMUX (see plans/4g-integration.md's transport decision), this
 * module owns the UART and serializes access with a mutex so callers'
 * request/response exchanges interleave safely on the wire instead of
 * corrupting each other's responses.
 *
 * modem_uart_send_at() covers the common case (write a command line, read
 * back the response). A few AT+CMQTT* commands need a two-phase exchange -
 * send a command line, then immediately follow with a fixed number of raw
 * bytes with no line terminator, then read the response - so the lower-level
 * lock/write_raw/read_response primitives are exposed too, for callers that
 * need to hold the UART across such a sequence.
 */

#ifndef MODEM_UART_H
#define MODEM_UART_H

#include <stddef.h>
#include <stdbool.h>

/**
 * Configure the UART peripheral (GNSS_UART_NUM/_TX_PIN/_RX_PIN/_BAUD from
 * config.h - shared by every AT-command caller, not GNSS-specific despite
 * the config names) and wait for the modem to answer "AT". Safe to call more
 * than once (e.g. from both gnss_init() and cellular_start_task()) - only the
 * first call does the work, later calls just return the cached result.
 * @return true once the modem has responded OK to "AT"
 */
bool modem_uart_init(void);

/**
 * Write an AT command (CR/LF appended) and read back the modem's response
 * until "OK"/"ERROR" is seen or the timeout elapses. Takes the UART lock for
 * the duration of the call, so it interleaves safely with other callers.
 * @return true if a response was read (check resp for OK/ERROR yourself),
 *         false on timeout with no bytes at all
 */
bool modem_uart_send_at(const char *cmd, char *resp, size_t resp_size, int timeout_ms);

/**
 * Take/release the UART lock for a multi-step exchange (e.g. AT+CMQTTTOPIC's
 * "command line, then raw literal bytes, then OK" flow) that
 * modem_uart_send_at() can't express as a single call. Callers must pair
 * every successful lock with an unlock.
 * @return true if the lock was acquired within timeout_ms
 */
bool modem_uart_lock(int timeout_ms);
void modem_uart_unlock(void);

/**
 * Write raw bytes with no line terminator added. Caller must hold the UART
 * lock (modem_uart_lock()).
 */
void modem_uart_write_raw(const void *buf, size_t len);

/**
 * Read back a response until "OK"/"ERROR" is seen or the timeout elapses.
 * Caller must hold the UART lock (modem_uart_lock()).
 * @return number of bytes read into resp (0-terminated), or -1 on timeout
 *         with no bytes at all
 */
int modem_uart_read_response(char *resp, size_t resp_size, int timeout_ms);

/**
 * Read bytes until a ">" prompt byte is seen (the modem signaling it's
 * ready for a raw-data argument - AT+CMQTTTOPIC/AT+CMQTTPAYLOAD both work
 * this way per SIMCom's MQTT AT command manual: command line, wait for ">",
 * then the raw data, then OK), "ERROR" is seen, or the timeout elapses.
 * Caller must hold the UART lock (modem_uart_lock()).
 * @return true if the prompt was seen, false on ERROR or timeout
 */
bool modem_uart_wait_for_prompt(char *resp, size_t resp_size, int timeout_ms);

/**
 * Discard any bytes currently sitting in the UART RX buffer. Useful before
 * starting a multi-step locked exchange (modem_uart_lock() + several
 * write_raw/read_response calls) to clear out a stale trailing URC from a
 * previous command (e.g. AT+CMQTTSTART's "OK" can arrive before its
 * "+CMQTTSTART: 0" URC does) that could otherwise get read as part of the
 * next command's response. modem_uart_send_at() already does this
 * internally before every single-shot command; multi-step callers need to
 * do it themselves once, up front. Caller must hold the UART lock
 * (modem_uart_lock()).
 */
void modem_uart_flush_input(void);

#endif // MODEM_UART_H
