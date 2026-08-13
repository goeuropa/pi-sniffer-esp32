# 4G Integration

## Goal

Use the SIM7670G modem's cellular data connection to upload people-count
reports to the server, so the sniffer doesn't depend on WiFi being available
at its deployment location. As part of this, **unify all reporting (WiFi and
cellular) onto MQTT** rather than running REST for WiFi units and MQTT for
cellular units side by side — one payload/topic schema, one code path
server-side.

## Status: not started

The SIM7670G's UART/AT interface is already in use for GNSS (see
[gps-integration.md](gps-integration.md), [src/gnss.c](../src/gnss.c)), so
the modem itself is reachable from the firmware. Actual data-mode networking
over the modem hasn't been built yet.

### Current reporting path (WiFi-only)

- [src/main.c](../src/main.c)'s `report_task()` gates sending on
  `wifi_is_connected()` and posts via `http_send_devices()` over WiFi only.
- If WiFi isn't connected, the report is logged and silently skipped
  (`ESP_LOGW(TAG, "WiFi not connected, skipping report")`) — no fallback
  transport exists.

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

1. **Add an AT-command MQTT client to the modem driver**, alongside
   `gnss.c`'s existing `gnss_send_at()` — likely a new
   `cellular.c`/`cellular.h` sharing the same UART/AT port:
   - `AT+CGDCONT` / `AT+NETOPEN` (or SIM7670G-equivalent) to bring up the
     PDP context against the deployment SIM's APN.
   - `AT+CMQTTSTART`, `AT+CMQTTACCQ`, `AT+CMQTTCONNECT` to open a session
     against the broker, then `AT+CMQTTTOPIC` + `AT+CMQTTPAYLOAD` +
     `AT+CMQTTPUB` per report to publish the device-count JSON body — the
     same payload `http_send_devices()` builds today for the WiFi/REST path,
     just published instead of POSTed.
   - Keep the MQTT session connected across report cycles rather than
     reconnecting each time (reconnect/backoff only on failure) — avoids
     re-running the PDP+MQTT-connect handshake every `REPORT_INTERVAL_SEC`.
   - Interleave with `gnss.c`'s `AT+CGNSSINFO` polls on the same command
     channel — both are short request/response AT exchanges, so they just
     need to share the UART sequentially (e.g. a mutex/lock around
     `gnss_send_at()`-style calls) rather than requiring CMUX.
2. **Move WiFi reporting onto MQTT too**, so both transports publish the
   same way instead of WiFi staying on REST while only cellular gets MQTT:
   - WiFi units have a real IP stack already, so they don't need the
     modem's AT-command MQTT path — use ESP-IDF's `esp-mqtt`
     (`esp_mqtt_client`) component directly over the WiFi `esp_netif_t`.
   - Cellular units go through the modem's `AT+CMQTT*` path from step 1
     (no local IP stack on the ESP32-S3 side, see the CMUX/no-PPP decision
     above).
   - Both publish the same JSON payload (whatever `http_send_devices()`
     builds today, extended with location per
     [gps-integration.md](gps-integration.md)) to the same topic scheme, so
     the transport is just an implementation detail per board/deployment,
     not something the server needs to know about.
   - `report_task()` picks WiFi-MQTT vs. cellular-MQTT per deployment/
     connectivity rather than choosing between REST and MQTT.
3. **Server side needs an MQTT subscriber, replacing the current REST
   route.** [server/index.js](../server/index.js) is currently a plain
   Express app with a `POST /api/devices` REST route and no MQTT broker —
   this is new server-side work, not just a firmware change. Plan:
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
4. **Config additions** (`include/config.h`): APN, PDP auth (if the
   carrier's SIM requires it), MQTT broker host/port/credentials and topic
   name, and a way to select "WiFi only" / "cellular only" / "WiFi with
   cellular fallback" per deployment — some units may be permanently
   off-grid on cellular only, others WiFi-only as today.
5. **Data usage considerations**: cellular data isn't free/unlimited like
   WiFi typically is for this project. Worth revisiting `REPORT_INTERVAL_SEC`
   and payload size (see the location fields planned in
   [gps-integration.md](gps-integration.md)) once cellular is the transport,
   and possibly compressing/shrinking the device-count JSON payload for
   cellular-connected units.
6. **Power**: the SIM7670G's cellular radio draws meaningfully more current
   than GNSS-only operation, especially during a cold registration/attach —
   relevant if the target deployment is battery/solar powered rather than
   mains.

## Open questions

- Production MQTT broker choice/hosting once past local testing
  (self-hosted alongside [server/](../server/) vs. a managed broker) and
  auth/TLS (`AT+CMQTTSSLCONNECTCFG`, cert provisioning) requirements.
- Topic/payload schema — reuse the existing `/api/devices` JSON body as the
  publish payload, or design a topic-per-device-id scheme.
- Which APN/carrier the deployment SIM will use (affects the PDP context
  config and whether authentication is needed).
- Whether to keep WiFi as the primary transport with cellular as fallback,
  or move fully to cellular for off-grid deployments.
