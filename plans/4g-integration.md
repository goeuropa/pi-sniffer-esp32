# 4G Integration

## Goal

Use the SIM7670G modem's cellular data connection to upload people-count
reports to the server, so the sniffer doesn't depend on WiFi being available
at its deployment location. As part of this, **unify all reporting (WiFi and
cellular) onto MQTT** rather than running REST for WiFi units and MQTT for
cellular units side by side — one payload/topic schema, one code path
server-side.

## Status: firmware implemented, untested on hardware; server side not started

Plan items 1, 2, and 4 (cellular AT-command MQTT client, WiFi moved onto
MQTT, config additions) are implemented and build cleanly (`pio run -e
esp32s3` succeeds; `test/test_gnss.c`'s 27 parser tests still pass after the
UART refactor). None of it has been exercised against real hardware or a
real broker yet — see "Needs on-device validation" below before treating
this as working. Item 3 (server-side MQTT subscriber) is **not started**:
[server/index.js](../server/index.js) is still the plain REST-only Express
app described below.

Current `config.h` is set to test cellular reporting specifically:
`DISABLE_API_SEND=0`, `ENABLE_CELLULAR=1`, `REPORT_TRANSPORT=
REPORT_TRANSPORT_CELLULAR_MQTT` (WiFi still connects for provisioning/SNTP,
just isn't used for reporting), `MQTT_BROKER_HOST=api.mercerfamily.org:1883`
(publicly reachable, needed since the cellular link isn't on the same
network as a LAN-only broker would be).

**Scope change**: the report payload itself is now minimal for both WiFi
and cellular - device ID, timestamp, phone count, and position only, not
the full per-device list. See item 5 below for detail; this also
substantially defused several of the cellular debugging rounds below, which
were largely payload-size-driven.

### Firmware implementation

- [include/modem_uart.h](../include/modem_uart.h) /
  [src/modem_uart.c](../src/modem_uart.c) — the shared UART/AT transport,
  extracted out of `gnss.c` (which owned the UART directly before). Mutex-
  serializes AT exchanges so `gnss.c` and `cellular.c` interleave safely on
  the modem's single AT port instead of requiring CMUX. `gnss.c` was
  refactored to call this instead of owning the UART itself; its parsing
  logic ([gnss_parse_cgnssinfo()](../src/gnss.c)) is untouched.
  - **`gnss.c` no longer polls on its own timer.** The independent
    `gnss_task`/`GNSS_POLL_INTERVAL_SEC` background poll (originally the
    only way GNSS was ever exercised) is gone - it was the root cause of
    several rounds of AT-command interleaving with `cellular.c`'s own
    traffic on the shared UART (the mutex prevented byte-level corruption,
    but the two tasks still competed for the UART on unrelated schedules).
    Replaced with `gnss_poll_now()`, a synchronous single-shot poll -
    **UPDATE**: the call site moved again since this was first written.
    It briefly lived in `cellular_task` (immediately before each cellular
    send attempt); Ian then asked for GNSS to run regardless of which
    transport is configured, with no separate task either way. The poll
    site is now `report_task()` (main.c), once per report cycle,
    unconditionally whenever `ENABLE_GNSS=1` - still exactly one call site,
    just upstream of both transports instead of inside cellular's. GNSS
    also now disciplines the ESP32's system clock (`settimeofday()`) from
    each fix's own UTC time - see `gnss_maybe_sync_system_clock()` in
    `gnss.c` - so units with no WiFi/SNTP still get a correct wall clock.
    Consequence: WiFi reports now include position too (both transports
    share `device_json_build_minimal()` - see below), not just cellular.
- [include/cellular.h](../include/cellular.h) /
  [src/cellular.c](../src/cellular.c) — PDP context + modem-side MQTT
  session (`AT+CEREG?`, `AT+CGDCONT`, `AT+NETOPEN`, `AT+CMEE=2`,
  `AT+CMQTTSTART/ACCQ/CONNECT/TOPIC/PAYLOAD/PUB` — see round 7 below for
  `AT+NETOPEN`/`AT+CMEE=2`, both round-trip additions), kept connected
  across report cycles with reconnect/backoff on failure
  (`CELLULAR_RECONNECT_BACKOFF_SEC`). Gated by `ENABLE_CELLULAR` in
  `config.h` (currently `1` — enabled for testing). `cellular_start_task()`
  is the only cellular-specific call `main.c` makes (mirrors
  `mqtt_report_start_task()`'s single-entry-point shape on the WiFi side) -
  it brings up the shared UART and a pending-report queue synchronously,
  then starts `cellular_task` and returns; `main.c` doesn't check its
  return value or otherwise know anything about cellular internals, same as
  the WiFi call site. The PDP+MQTT bring-up itself is deferred further
  still, to `cellular_task`'s first queued report (see below), not
  attempted synchronously during boot (its ~170s worst-case timeout chain
  would otherwise stall BLE scanning and
  all logging - this was a real bug caught during initial hardware testing,
  see git history).
  - **Sending runs on its own task, decoupled from `report_task()`.**
    `cellular_publish()` doesn't perform the AT exchange itself - it hands
    off `topic`/`phone_count`/`qos`, plus a GNSS fix already polled by
    `report_task()` (see above - `cellular_task` itself never touches
    GNSS anymore), into a length-1 "keep only the freshest" queue and
    returns immediately (no payload string - see below). `cellular_task`
    blocks on that queue; for each report it calls
    `cellular_ensure_connected()` (internally paced to
    `CELLULAR_RECONNECT_BACKOFF_SEC`, 5 minutes), and if connected (and
    `cellular_should_send()` decides the fix/phone count changed enough to
    be worth it - see `config.h`'s
    `CELLULAR_POSITION_UNCHANGED_THRESHOLD_M`/`CELLULAR_HEARTBEAT_INTERVAL_SEC`)
    builds the payload fresh from that snapshot and does the
    `AT+CMQTTTOPIC/PAYLOAD/PUB` exchange.
    Replaced an earlier explicit outer(connect)/inner(serve queue) loop
    shape - no longer needed once `cellular_ensure_connected()`'s own
    backoff gate does the same pacing job on every queue-driven call. A
    slow/hung cellular send never delays `report_task()`'s own loop
    (`print_summary()`, the next report cycle, WiFi sending).
- [include/device_json.h](../include/device_json.h) /
  [src/device_json.c](../src/device_json.c) — **UPDATE**: this used to
  describe two minimal builders (one WiFi, position-less; one cellular,
  with position). Now there's just one: `device_json_build_minimal()`
  (device ID, ISO-8601 timestamp - GPS-sourced when available, system-clock
  fallback otherwise - phone count, `lat`/`lon`), shared by both
  transports. `report_task()` builds it once per cycle from the fix it just
  polled and hands the same payload to whichever transport
  `REPORT_TRANSPORT` selects. `device_json_build()` (the original full
  per-device payload, extracted out of `http_client.c`) is kept in the file
  but no longer called from the report path - see item 5 in the Plan
  section below.
- [include/mqtt_report.h](../include/mqtt_report.h) /
  [src/mqtt_report.c](../src/mqtt_report.c) — WiFi-side MQTT via ESP-IDF's
  built-in `esp-mqtt` (`esp_mqtt_client`) component over the WiFi
  `esp_netif_t`. Now gets the exact same treatment as `cellular.c`:
  `mqtt_report_publish()` is a hand-off to its own "keep only the freshest"
  queue, and a dedicated `mqtt_report_task` (started via
  `mqtt_report_start_task()`) owns an outer loop (wait for WiFi + the MQTT
  client to be connected, re-kicking `wifi_connect()` every
  `WIFI_RECONNECT_INTERVAL_SEC` (5 minutes) once `wifi_manager.c`'s own fast
  burst-retry has given up - that fast retry itself is unchanged, this
  layers on top of it) and an inner loop (serve the queue, drop back to the
  outer loop on a publish failure or detected disconnect). Neither
  `run_normal_mode()`'s boot sequence nor `report_task()` block on WiFi
  association or MQTT connect anymore - `wifi_wait_connected()`'s old 30s
  blocking call at boot is gone; `init_sntp()` moved to its own small
  `sntp_wait_task` that waits for WiFi in the background instead.
- [src/main.c](../src/main.c)'s `report_task()` now builds one JSON payload
  per cycle and hands it off to WiFi-MQTT and/or cellular-MQTT's queues
  depending on `REPORT_TRANSPORT` (config.h) — REST (`http_send_devices()`)
  is no longer called from the report path, though `http_client.c` is left
  in the tree rather than deleted. Both transports being symmetric hand-offs
  now, the report log is symmetric too: `"Report queued for WiFi send"` /
  `"Report queued for cellular send"` - actual delivery success/failure is
  always logged separately and asynchronously, from each task itself.
- [src/CMakeLists.txt](../src/CMakeLists.txt) — added the four new source
  files and the `mqtt` component to `REQUIRES`.

### Needs on-device validation

- **GNSS-driven-by-cellular restructure** - `gnss_poll_now()` (called from
  `cellular_task`) hasn't caused any new interleaving symptoms in testing
  so far (bring-up got cleanly through `AT+CGDCONT`/`AT+CMQTTSTART` with no
  `GNSS:`-flavored corruption), but hasn't been specifically confirmed
  either - still worth checking: `GNSS: ...` log lines should only ever
  appear immediately around cellular `AT+CMQTT*` activity, including during
  a `"Still backing off..."` cycle (not connected), never on an independent
  cadence; WiFi-only report cycles should show no GNSS activity at all.
- **The exact `AT+CMQTT*` sequence in `cellular.c`** — originally written
  against SIMCom's A76XX/SIM76XX MQTT(TCP) application note from memory;
  since cross-checked directly against SIMCom's official
  `SIM7500_SIM7600_SIM7800 Series_MQTT_AT Command Manual_V1.00` (the closest
  available primary source - not SIM7670G-specific, but the same CMQTT
  command family), which caught several real discrepancies below. Firmware
  quirks specific to the SIM7670G (exact response text, timing) are still
  unconfirmed and expected to need further iteration.
  - **Found and fixed on first hardware test (round 1)**: `cellular_do_publish()`
    was locking/unlocking the shared modem UART separately for each of
    `AT+CMQTTTOPIC`/`AT+CMQTTPAYLOAD`/`AT+CMQTTPUB`, instead of holding one
    continuous lock across all three. That left two windows where
    `gnss_task`'s periodic `AT+CGNSSINFO` poll could interleave mid-sequence
    and corrupt both command streams on the wire - observed directly as
    `GNSS: unexpected CGNSSINFO response: AT+CGNSSINFO\nOK\nreports/ESP32_...SSINFO\nERROR`.
    Fixed by holding the UART lock for the whole TOPIC/PAYLOAD/PUB sequence.
  - **Found via the official manual (round 2), while investigating the
    `AT+CMQTTSTART` `ERROR` that round 1's fix then surfaced**:
    - `AT+NETOPEN` was being called before `AT+CMQTTSTART` in
      `cellular_bring_up_pdp()`. The manual's documented process (§1.2) is
      just `AT+CGDCONT` then `AT+CMQTTSTART` - **no `AT+NETOPEN` at all**.
      `AT+CMQTTSTART` activates the PDP context itself; `AT+NETOPEN` opens
      a *different* application (the TCPIP AT command set) that appears to
      compete for the same underlying PDP/socket resource - almost
      certainly the actual cause of the `AT+CMQTTSTART ERROR`. Removed.
      **Reversed in round 7** - this theory was never actually confirmed
      (what fixed `AT+CMQTTSTART` was round 3's defensive `AT+CMQTTSTOP`,
      not this removal), and the manual it was based on isn't SIM7670G-specific.
      `AT+NETOPEN` is back, see below.
    - `AT+CMQTTSTART`'s documented max response time is **120000ms** -
      `CELLULAR_AT_TIMEOUT_MS` (10s) was nowhere near enough. New
      `CELLULAR_CMQTTSTART_TIMEOUT_MS` (120000ms) used for that command
      specifically.
    - `AT+CMQTTTOPIC`/`AT+CMQTTPAYLOAD` are documented as a **two-phase
      exchange**: send the command line, wait for a `">"` prompt byte, *then*
      send the raw data - not "command line immediately followed by raw
      bytes" as this code assumed. Confirmed against the manual's own worked
      example transcript. New `modem_uart_wait_for_prompt()` in
      `modem_uart.c`, used by a new `cellular_write_at_with_prompt()`.
    - Added an `AT+CEREG?` LTE-registration check before `AT+CGDCONT`,
      matching the manual's documented pre-flight step ("ensure GPRS network
      is available") - `AT+CGDCONT`/`AT+CMQTTSTART` are liable to fail or
      hang if issued before the modem has actually joined the network.
    - Added `modem_uart_flush_input()` (new, in `modem_uart.c`), called once
      at the start of `cellular_do_publish()`'s locked sequence, to discard
      any stale trailing URC bytes from a previous command (e.g.
      `AT+CMQTTSTART`'s `OK` can arrive before its `+CMQTTSTART: 0` URC
      does - confirmed in the manual's own example transcripts) that could
      otherwise get misread as part of the next response.
  - **Round 2 tested on hardware - `AT+CMQTTSTART` still failed**, but
    differently: an *immediate* (~200ms) plain `ERROR`, not a timeout -
    too fast to be a real network operation, more consistent with a
    state-precondition rejection. Round 3 hypothesis: the SIM7670G has its
    own power/reset domain, independent of the ESP32 - across a dev cycle
    of repeated ESP32 reflashes this session, the modem itself very likely
    never power-cycled. If an earlier attempt (before round 2's fixes) got
    as far as `AT+CMQTTSTART` actually succeeding on the modem and was
    never cleanly `AT+CMQTTSTOP`'d, the modem still considers MQTT started
    even though `s_mqtt_started` resets to `false` on every ESP32 boot -
    `AT+CMQTTSTART` on an already-started service is a known SIMCom gotcha
    and fits the instant-rejection timing well. **Fix**: `cellular.c` now
    sends a best-effort `AT+CMQTTSTOP` (result ignored - harmless error if
    it wasn't actually started) immediately before the first
    `AT+CMQTTSTART` each boot, forcing a known baseline.
  - Diagnostic gap noted for a possible round 4: `modem_uart_read_response()`
    stops as soon as it sees `ERROR`, so a more specific trailing
    `+CMQTTSTART: <err>` URC (see the manual's `<err>` code table - e.g. `9`
    = "network not opened") - if the modem sends one - isn't currently
    captured. Not fixed yet (would need `cellular_mqtt_session_connect()`'s
    `AT+CMQTTSTART` call to move off `modem_uart_send_at()` onto the
    lower-level lock/write/read primitives to append a supplementary read
    without racing another task for the UART lock in between) - worth doing
    if round 3's fix doesn't resolve it and better diagnostics are needed.
  - **Round 3 tested on hardware - `AT+CMQTTSTART` succeeded** (confirms
    the "already started on the modem, needs a stop first" hypothesis), and
    got as far as a real `AT+CMQTTPAYLOAD` with the actual ~1.7KB device
    report JSON - which then failed, differently again, with the *echoed
    payload itself* showing up as the "failure" text, and its unread tail
    leaking into `gnss_task`'s next `AT+CGNSSINFO` poll (`GNSS: unexpected
    CGNSSINFO response: ...` containing JSON fragments). Round 4 root
    cause: command echo (`ATE1`, the modem's default) was never disabled -
    the modem echoes every byte it's sent, including large raw-data
    payloads from `AT+CMQTTTOPIC`/`AT+CMQTTPAYLOAD`. `CELLULAR_AT_RESP_BUF_SIZE`
    (256 bytes) fills with echoed payload well before the real `OK` arrives,
    so the read looks like a failure even when the command likely
    succeeded - and the unread remainder of the echo sits in the UART
    driver's buffer until the *next* AT exchange reads it, corrupting
    whichever one that happens to be. Ian asked whether serializing GNSS
    polling around cellular sends would fix this instead - it wouldn't:
    the lock already correctly serializes writes between the two tasks, so
    timing/ordering isn't the problem; an oversized echo would overflow
    cellular's own response buffer and corrupt cellular's *own* next
    read regardless of what else is or isn't running concurrently. **Fix**:
    `modem_uart_init()` now sends `ATE0` once, right after confirming the
    modem is responsive, before returning to any caller - eliminates the
    echo entirely rather than working around its size.
  - **Round 4 tested on hardware - progress, but a new failure**:
    `AT+CMQTTSTART` now succeeds reliably (confirms round 3's fix), and
    bring-up got past it cleanly with no echo-related corruption observed.
    `AT+CMQTTACCQ` then failed, the same way `AT+CMQTTSTART` originally
    did - immediate (~600ms) plain `ERROR`, consistent with the *same* bug
    class: the client index (0) was very likely already acquired by the
    modem from an earlier, uncleanly-terminated test session (same
    persistent-modem-state root cause as round 3, just a different acquired
    resource this time). **Round 5 fix**: generalized round 3's single
    defensive `AT+CMQTTSTOP` into the full teardown order the manual
    documents - `AT+CMQTTDISC` → `AT+CMQTTREL` → `AT+CMQTTSTOP`, all
    best-effort/result-ignored - before the first `AT+CMQTTSTART` each
    boot. Covers the `AT+CMQTTACCQ` failure just hit, and preemptively
    covers the equivalent `AT+CMQTTCONNECT` failure (an already-connected
    client) before it's had a chance to show up as its own round.
  - **Round 6 (not yet hardware-tested)**: Ian pointed out round 5's
    defensive teardown only ran once, at boot (`cellular_mark_disconnected()`
    deliberately left `s_pdp_up`/`s_mqtt_started`/`s_mqtt_acquired` alone on
    a failure, so a retry mid-session skipped straight back to
    `AT+CMQTTCONNECT`) - but the exact same "modem's actual state doesn't
    match what these flags assume" problem that hit `AT+CMQTTSTART` and
    `AT+CMQTTACCQ` could just as easily happen after a failed
    `AT+CMQTTCONNECT` retried mid-session, not just after a fresh boot.
    `cellular_mark_disconnected()` now resets all four flags (not just
    `s_mqtt_connected`), so **every** reconnect attempt - first one at boot
    or any later retry - replays the full sequence including the
    `AT+CMQTTDISC`/`REL`/`STOP` teardown, rather than only the first one.
  - Round 6's fix has not been confirmed tested on hardware either way
    (no result reported) - round 7 below was prompted by independent
    research, not a confirmed round 6 failure.
  - **Round 7**: Ian found a different documented working sequence
    (`AT+CMEE=2` → `AT+NETOPEN` → `AT+CMQTTSTART` → `AT+CMQTTREL=0` →
    `AT+CMQTTACCQ`) that puts `AT+NETOPEN` back in, contradicting round 2's
    removal. Re-examining round 2: the theory that `AT+NETOPEN` conflicts
    with `AT+CMQTTSTART` was never actually confirmed - what fixed
    `AT+CMQTTSTART` was round 3's defensive `AT+CMQTTSTOP`, added *after*
    the `AT+NETOPEN` removal, and the SIM7500/7600/7800 manual that removal
    was based on isn't SIM7670G-specific. The `AT+CMQTTACCQ`/`CONNECT`
    failures since removing it are also consistent with that same manual's
    error code `9` ("network not opened"). Two changes:
    - `AT+CMEE=2` (verbose CME error codes) added to `modem_uart_init()`,
      alongside `ATE0` - pure upside regardless of the `AT+NETOPEN`
      question, and directly closes the "can't see the real error code"
      diagnostic gap noted after round 3. Failures should now show
      `+CME ERROR: <description>` instead of bare `ERROR`, visible via the
      `AT RX:` logging added afterward.
    - `AT+NETOPEN` restored to `cellular_bring_up_pdp()`, right after
      `AT+CGDCONT`, with the same "already opened" tolerance the original
      (pre-round-2) code had. Round 6's full defensive teardown before
      `AT+CMQTTSTART` is kept as-is - more thorough than the found
      sequence's single `AT+CMQTTREL=0`, and not in conflict with it.
  - **Round 7 tested on hardware - `AT+NETOPEN` succeeded (`OK` +
    `+NETOPEN: 0`), `AT+CMQTTSTART` succeeded fresh (`+CMQTTSTART: 0`), but
    `AT+CMQTTACCQ` still failed** - bare `ERROR`, no code at all, and
    critically **no `+CME ERROR:` text despite `AT+CMEE=2` being
    confirmed active** for this run. Also newly visible (thanks to the
    round-6 AT logging): the defensive teardown's `AT+CMQTTREL`/
    `AT+CMQTTSTOP` calls both came back `+CMQTTxxx: 0,19` → `ERROR` - a
    real, informative error code we hadn't seen before, but not decodable
    without the `<err>` table.
  - **Round 8**: found `SIM7672X_SIM7652X_Series_MQTT(S)_Application_Note_V1.00.pdf`,
    hosted on Waveshare's own wiki page for *this exact board*
    (`files.waveshare.com/wiki/ESP32-S3-SIM7670G-4G/...`) - the closest
    board-specific reference found yet, versus the SIM7500/7600/7800
    manual used for rounds 1-7 (a different, merely-adjacent chip family).
    Two findings changed everything:
    - **Its `<err>` code table decodes code `19` as "client is used"** -
      confirms the modem's client-index state really is stale/stuck, as
      rounds 5-6 suspected, but also means round 6's full teardown
      (`DISC`→`REL`→`STOP`) isn't actually clearing it - `AT+CMQTTREL`
      itself reports "client is used" when trying to release it, which is
      why `AT+CMQTTACCQ` keeps failing afterward: nothing ever actually
      gets released. (Codes 7/8/9 - "network open/close fail"/"network not
      opened" - the theory rounds 2/7's `AT+NETOPEN` back-and-forth was
      chasing - never actually appeared in any response.)
    - **Its bring-up flowchart never uses `AT+NETOPEN` at all**, and
      includes a step this code has *never* sent: `AT+CGDCONT` (define the
      PDP context) → **`AT+CGACT=1,1`** (actually *activate* it) → verify →
      `AT+CMQTTSTART`. `AT+CGDCONT` alone only defines the context: it was
      never actually activated by anything other than `AT+CMQTTSTART`'s own
      documented-but-apparently-insufficient implicit activation.
    - Also corrects `AT+CMQTTSTART`'s documented max response time: **12000ms**
      per this board-specific note, not the `120000ms` rounds 2-7 used from
      the other manual (`CELLULAR_CMQTTSTART_TIMEOUT_MS` updated).
    - **Fix**: `cellular_bring_up_pdp()` drops `AT+NETOPEN` (again - third
      reversal on this specific command, now backed by much stronger
      board-specific evidence and a coherent explanation of *why* previous
      rounds never worked) and adds the missing `AT+CGACT=1,1`, plus a
      best-effort diagnostic `AT+CGPADDR=1` (logs the assigned IP, doesn't
      gate). Also switched the defensive `AT+CMQTTDISC=<idx>,0` teardown
      call to `AT+CMQTTDISC=<idx>,60` - `0` is described as a "use default"
      sentinel in the *other* manual, but that's outside its own documented
      60-180s range and was never confirmed valid for this firmware.
  - Not yet re-tested on hardware - round 8's fix should confirm
    `AT+CMQTTACCQ`/`AT+CMQTTCONNECT` finally succeed and a publish reaches
    the broker end-to-end. If `AT+CMQTTACCQ` still fails after this, the
    modem's client-index state may need a real power cycle (not just an
    ESP32 reflash) to clear - the modem has its own power/reset domain that
    nothing in this firmware's AT sequence has touched.
  - **Round 9**: Ian passed along further advice specific to
    SIM7500/SIM7600-family firmware that `AT+CMQTTACCQ`'s optional trailing
    `<server_type>` parameter (`0` = TCP, the value this code always sent
    explicitly) can itself trigger a syntax error on some firmware when
    passed rather than omitted - both MQTT AT manuals' own worked examples
    omit it (`AT+CMQTTACCQ=0,"client test0"`) despite documenting it as a
    legal explicit value. Cheap to try alongside round 8's fix, so
    `cellular_mqtt_session_connect()`'s `AT+CMQTTACCQ` call now sends
    `AT+CMQTTACCQ=<idx>,"<client_id>"` with no trailing `,0`. Not yet
    tested on hardware. If `AT+CMQTTACCQ` still fails after rounds 8+9
    together, the modem's client-index state most likely does need a real
    power cycle to clear, per round 8's caveat.
- **`MQTT_BROKER_HOST`/`MQTT_BROKER_URI` are now set to `api.mercerfamily.org:1883`**
  (was a `192.168.1.100` placeholder) — but nothing has confirmed a broker
  is actually listening there yet, or that it's reachable unencrypted on
  1883 from outside Ian's LAN. Worth checking before relying on a "no
  reports arrived" result meaning the firmware side is broken.
- **`CELLULAR_MQTT_MAX_PAYLOAD_LEN` (4096 bytes) is an unconfirmed guess** —
  the modem's real single-publish limit isn't documented for this
  board/firmware; a full `MAX_DEVICES` report can run tens of KB, so this
  will likely bind before the modem's real limit does regardless (see item 5
  below).
- **Cellular reconnect can't detect a fully-reset modem or a PDP context
  dropped out from under a "connected" session** — `cellular_mark_disconnected()`
  in `cellular.c` only backs off the MQTT-connect step; a follow-up would add
  periodic `AT+CEREG?`/`AT+CMQTTSTART?` state polling to close that gap.
- **`mqtt_report_task`'s WiFi reconnect layering (`mqtt_report.c`) is new
  and untested on hardware** — worth confirming on a real WiFi drop that (a)
  a brief blip still recovers in a few seconds via `wifi_manager.c`'s
  existing fast burst-retry, unchanged, and (b) a genuinely down AP shows
  the "WiFi retry exhausted, re-kicking connection" log every
  `WIFI_RECONNECT_INTERVAL_SEC` (5 min) rather than getting stuck the way it
  could before this session (`wifi_manager.c` used to give up permanently
  after `WIFI_MAX_RETRY`, never retrying again on its own).

### UART contention with GNSS polling — resolved

The SIM7670G exposes one AT port to the ESP32-S3, and `gnss.c` already owns
it for `AT+CGNSSINFO` polling. Three ways to get cellular data without the
two stepping on each other were evaluated:

1. **CMUX (3GPP TS 27.010) software multiplexing** — `AT+CMUX=0` splits the
   single UART into virtual channels (DLCIs): one dedicated to the PPP data
   session, another left free for AT commands (GNSS polling included) at the
   same time. Gives a real `esp_netif_t`/IP stack on the ESP32-S3 via
   ESP-IDF's `esp_modem` component, so any existing socket/HTTP client code
   works unmodified. Cost: `esp_modem`'s CMUX client plus RTS/CTS flow
   control wiring, more moving parts to get right.
2. **Modem-side sockets (no PPP at all)** — keep the UART in plain AT command
   mode permanently (as it is today) and use the SIM7670G's built-in
   HTTP(S)/MQTT(S)/TCP/UDP AT commands (`AT+CMQTT*` etc.) to do the upload
   directly through the modem, interleaved with `AT+CGNSSINFO` polls on the
   same command channel. No IP stack on the ESP32-S3 side at all.
3. **USB instead of UART** — the SIM7670G exposes separate AT/NMEA/modem
   virtual COM ports over USB, with RNDIS/ECM giving a free-standing network
   interface. Not applicable here: this board's SIM7670G is wired to the
   ESP32-S3 over UART, not driven from a USB host like a Pi/Linux box.

**Decision: go with option 2** (modem-side AT-command sockets), not
`esp_modem`/PPP/CMUX, and **use MQTT rather than HTTP** for the upload
itself. It fits the architecture already in place — `gnss.c`'s
`gnss_send_at()` pattern extends directly to issuing `AT+CMQTT*` commands on
the same UART GNSS already uses, with no second network stack, no
CMUX/flow-control wiring, and no new ESP-IDF component. MQTT over the
cellular link also has some practical advantages over HTTP+`AT+HTTPACTION`
for this use case: a long-lived connection (no re-registering the PDP
context/re-connecting per report), smaller framing overhead per publish,
and QoS options if delivery confirmation matters for reports sent while
off-grid. Revisit CMUX only if something later needs a real `esp_netif_t`
on the ESP32-S3 side (e.g. OTA firmware updates over cellular, or arbitrary
outbound sockets) that the modem's built-in AT command set can't cover.

### Plan

1. ✅ **Add an AT-command MQTT client to the modem driver** — done, see
   [cellular.c](../src/cellular.c) / [modem_uart.c](../src/modem_uart.c) in
   "Firmware implementation" above. UART contention with `gnss.c` is handled
   by `modem_uart.c`'s mutex rather than CMUX, as decided.
2. ✅ **Move WiFi reporting onto MQTT too** — done, see
   [mqtt_report.c](../src/mqtt_report.c) and `report_task()` in
   [main.c](../src/main.c). `report_task()` picks WiFi-MQTT vs.
   cellular-MQTT (mutually exclusive - never both) per `REPORT_TRANSPORT`.
   **UPDATE**: location fields from [gps-integration.md](gps-integration.md)
   are now folded into the JSON payload for both transports, not just
   cellular - `report_task()` polls GNSS once per cycle and both transports
   share `device_json_build_minimal()` (see "Firmware implementation"
   above) - this was TODO when the line above was first written.
3. **Not started. Server side needs an MQTT subscriber, replacing the
   current REST route.** [server/index.js](../server/index.js) is currently
   a plain Express app with a `POST /api/devices` REST route and no MQTT
   broker — this is new server-side work, not just a firmware change. Plan:
   - **Dev/testing**: point the firmware at the local MQTT broker Ian
     already has running for early end-to-end testing, before standing up
     a production broker.
   - **Production**: run a broker (e.g. Mosquitto, or a hosted one) and add
     a subscriber in the server that writes incoming reports through the
     same storage/downstream logic the REST route uses today, so only how
     reports arrive changes.
   - **Delete the `POST /api/devices` REST route** once the MQTT subscriber
     is confirmed working end-to-end — no dual-path ingestion to maintain.
   - [server/simulated_device.js](../server/simulated_device.js) currently
     POSTs to `/api/devices` over HTTP for local testing — needs to be
     switched to publish over MQTT (to the same local broker) so it stays
     useful as a test harness once the REST route is gone.
4. ✅ **Config additions** (`include/config.h`) — done: `CELLULAR_APN`,
   `MQTT_BROKER_HOST`/`_PORT`/`_URI` (placeholder values, see "Needs
   on-device validation"), `MQTT_TOPIC_PREFIX`, and `REPORT_TRANSPORT`.
   **UPDATE**: `REPORT_TRANSPORT` originally had a third value,
   `WIFI_WITH_CELLULAR_FALLBACK`; Ian later asked for WiFi and cellular to
   be strictly mutually exclusive, so that's gone - just `WIFI_ONLY` /
   `CELLULAR_ONLY` now, and `run_normal_mode()` only starts the one
   background task (`mqtt_report_task` or `cellular_task`) the selected
   value actually needs.
5. ✅ **Data usage considerations — resolved by a scope change, not payload
   tuning.** Ian decided the network should never carry full per-device
   detail at all - **both WiFi and cellular now report a minimal payload**
   (`device_json_build_minimal()` in `device_json.c`): just `device_id`,
   `timestamp`, `phones` (phone count from `device_summary_t.phones`), and
   `lat`/`lon` (from `gnss_get_last_fix()`, `null` if no fix yet). Measured
   ~114 bytes with a fix, ~84 bytes without - independent of how many BLE
   devices are nearby, unlike the old per-device payload. The full
   per-device payload (`device_json_build()`, MAC/RSSI/distance/category
   per device) is no longer called from `report_task()` but is left in the
   tree in case per-device detail is wanted again later (e.g. a
   local-network-only debug mode) - same precedent as `http_client.c`'s
   unused legacy REST path.

   **Concrete budget, updated**: at ~114 bytes/report and the current
   `REPORT_INTERVAL_SEC` (30s, ~86,400 reports/month), that's roughly
   **~10MB/month** - trivially inside the SpeedTalk SIM's $7/2GB budget,
   regardless of how many BLE devices are nearby (previously the binding
   constraint scaled with device count, up to ~1.7GB/month worst case).
   `REPORT_INTERVAL_SEC` no longer needs revisiting for data-budget reasons.
6. **Power**: the SIM7670G's cellular radio draws meaningfully more current
   than GNSS-only operation, especially during a cold registration/attach —
   relevant if the target deployment is battery/solar powered rather than
   mains.

## Resolved decisions

- **APN/carrier**: deployment SIM uses APN `mnet`, MCC 310 / MNC 240 — a
  T-Mobile US MVNO block, no username/password required. PDP context is
  therefore just `AT+CGDCONT=1,"IP","mnet"`, no `AT+CGAUTH` needed. The SIM
  is activated with SpeedTalk Mobile (that MVNO) on a $7/2GB plan — see
  "Data usage considerations" (item 5) for what that budget means for
  `REPORT_INTERVAL_SEC`/payload size.
- **Production MQTT broker**: still not formally decided, but firmware is
  currently pointed at `api.mercerfamily.org:1883` (plain/unencrypted) for
  testing — needed a publicly reachable host rather than a LAN-only address
  since the cellular link isn't on Ian's home network. Revisit
  self-hosted-vs-managed and TLS/auth once WiFi-MQTT and cellular-MQTT are
  both confirmed working end-to-end against it.
- **Topic/payload schema**: single topic per unit (e.g. `reports/<unit-id>`),
  publishing the same JSON body `http_send_devices()` builds today — no new
  topic-per-device hierarchy. Minimal server-side change: the MQTT subscriber
  parses the same payload shape the REST route parses today.
- **Transport role (WiFi vs. cellular)**: per-deployment selection, not a
  single global policy — `config.h` needs a build/runtime choice of
  WiFi-only / cellular-only / WiFi-with-cellular-fallback per unit, matching
  that some units are permanently off-grid on cellular only while others stay
  WiFi-only. `report_task()` reads this setting rather than assuming one
  policy for all units.

## Open questions

- Auth/TLS for the production broker (`AT+CMQTTSSLCONNECTCFG`, cert
  provisioning) — deferred along with the broker hosting decision above.
