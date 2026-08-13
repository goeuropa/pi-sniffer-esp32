# GPS/GNSS Integration

## Goal

Attach a location to each people-count report, so crowd density readings from
a mobile/portable sniffer unit can be plotted on a map rather than assumed to
come from a fixed location.

## Status: fix acquisition done, not yet wired into reporting

### Done

- **[gnss.h](../include/gnss.h) / [gnss.c](../src/gnss.c)** — driver for the
  onboard SIM7670G modem's GNSS receiver (Waveshare ESP32-S3 SIM7670G 4G
  board). Talks to the modem over UART using AT commands:
  - `gnss_init()` brings up the UART, waits for the modem to boot, syncs on
    plain `AT`, then sends `AT+CGNSSPWR=1` to power on the GNSS receiver.
  - `gnss_start_task()` runs a background FreeRTOS task that polls
    `AT+CGNSSINFO` every `GNSS_POLL_INTERVAL_SEC` and keeps the most recent
    fix in `s_last_fix`.
  - `gnss_get_last_fix()` returns a copy of the last fix (lat/lon/altitude/
    satellite count/fix time), thread-safe via a critical section.
  - `gnss_parse_cgnssinfo()` is a pure parser (no ESP-IDF dependency), unit
    tested in [test/test_gnss.c](../test/test_gnss.c). It's deliberately
    tolerant of firmware variation in `+CGNSSINFO`'s field count (SIM767XX
    vs A76XX manuals disagree, and real hardware has been seen truncating
    the no-fix response) by locating the lat/N-S/lon/E-W block by content
    instead of a fixed field index.
- Wired into [src/main.c](../src/main.c): `gnss_init()` +
  `gnss_start_task()` are started alongside BLE scanning. Fixes are logged
  (`ESP_LOGI(TAG, "GNSS fix: lat=... lon=... ...")`) but not yet consumed by
  anything else.

### Not done yet

1. **Include the last fix in each report.** `report_task()` in
   [src/main.c](../src/main.c) currently builds a device-count payload via
   `http_send_devices()` ([src/mac_pack.c](../src/mac_pack.c) /
   [include/mac_pack.h](../include/mac_pack.h) handle MAC-rotation packing
   before that count is computed) with no location attached. Plan:
   - Call `gnss_get_last_fix()` once per report cycle, alongside the
     existing `device_cleanup()` / `mac_pack_run()` / summary steps.
   - Add `latitude`, `longitude`, `altitude_m`, `num_satellites`, and
     `fix_time` (or a `has_fix: false` flag when no fix is available yet)
     to the JSON body `http_send_devices()` posts to the REST API.
   - Decide the no-fix behavior: send the report without a location (fine
     for a stationary unit that hasn't acquired a fix yet), vs. falling back
     to the last known fix even if it's stale — mark it as stale via
     `fix_time` so the server can decide how much to trust it.
2. **Server-side**: extend whatever consumes the REST payload (see
   [server/](../server/)) to store lat/lon per report and expose it for
   mapping/plotting alongside the people-count time series.
3. **Config surfacing**: `GNSS_POLL_INTERVAL_SEC` and friends already live in
   [include/config.h](../include/config.h) — no changes expected there
   beyond maybe decoupling the GNSS poll interval from `REPORT_INTERVAL_SEC`
   if we want fresher fixes than the report cadence.

## Open questions

- Precision/rounding of lat/lon in the outgoing payload (privacy vs. utility
  — do we want to truncate to ~4 decimal places, ~11m, for public-facing
  reports?).
- Whether to report device counts *without* GPS on boards that don't have
  the SIM7670G (e.g. plain ESP32-WROOM-32) — the field should presumably be
  omitted entirely rather than zeroed, so the server can distinguish "no GPS
  hardware" from "no fix yet."
