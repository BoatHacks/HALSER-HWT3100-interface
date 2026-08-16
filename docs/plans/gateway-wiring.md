# Implementation Plan: gateway.cpp wiring (core data path + calibration UI + serial log)

## Overview

Wire together the pieces built so far (`HWT3100SerialIO`,
`ParseHWT3100Line`) into a working SensESP application: HWT3100 →
calibration offset → N2K PGN 127250 + SignalK `navigation.headingMagnetic`,
plus the calibration-command and serial-log UI features, using SensESP's
existing config REST API (not a custom HTTP/WebSocket endpoint — see
below).

## Relevant SPEC/ARCHITECTURE Sections

SPEC.md §2, §5, §6, §7, §8; ARCHITECTURE.md §2 (all components), §6.

## Two implementation-time architecture corrections

Both discovered by reading SensESP 3.2.0's actual vendored source, not
assumed from its docs:

1. **No WebSocket support.** SensESP's `HTTPServer` wraps ESP-IDF's
   native `esp_http_server`, not `ESPAsyncWebServer`/`AsyncWebSocket` as
   ARCHITECTURE.md originally guessed.
2. **No public extension point for custom HTTP handlers at all.**
   `SensESPApp::http_server_` is `protected` with no getter anywhere in
   the public API; every `add_handler()` call in the library is internal
   to the framework's own `build()` step.

Resolution (user decision): reuse SensESP's existing, already-public
config REST API instead of a custom endpoint.

- **Calibration commands** → three `PersistingObservableValue<bool>`
  "trigger" config items. Setting one to `true` (via the auto-generated
  config UI) fires the corresponding `CalibrationCommandHandler` method,
  then immediately resets itself to `false` so it can be re-triggered.
  Known trade-off: if the device crashes/loses power between the `true`
  write and the `false` reset, the *next* boot's initial config-load
  emit could re-fire the command. Accepted because HWT3100 calibration
  commands are safe to resend (unlike `AT+MODE`) — see SPEC.md §1.2/§2
  for why that distinction matters here.
- **Serial log** → a small custom class (`SerialTerminal`) implementing
  `Saveable`+`Serializable` directly (not `FileSystemSaveable`), so
  `load()`/`save()`/`clear()` stay no-ops — nothing here ever touches
  flash, which matters for a high-churn buffer that would otherwise wear
  it out. `to_json()` serializes the last 30 raw lines; `from_json()` is
  left at `Serializable`'s default (`return false`), making writes a
  no-op — this is a read-only view surfaced through the config GET path.

## Explicitly deferred (not part of this change)

- **Stale-data fault indication** (RGB LED + SignalK notification, SPEC
  §6) — needs its own small design pass (what LED color/pattern, exact
  SignalK notification path/format) rather than bolting it on
  speculatively here. `ExpiringValue::to_n2k()` already handles the N2K
  side (transmits `N2kDoubleNA` when stale) as a side effect of reusing
  the parent's pattern, so that part of §6 is covered; the LED/SignalK
  notification part is not.
- Live-hardware verification of the whole pipeline — no HWT3100 module
  or N2K bus available in this environment; verification here is limited
  to a clean `pio run -e halser` build.

## Test Strategy

`pio run -e halser` (compiles against the real ESP32/SensESP framework —
this is where a wiring mistake, missing include, or API misuse would
surface) plus `pio test -e native` (confirms the parser is untouched).
No hardware-in-the-loop test is possible in this environment.

## Implementation Steps

- [x] `calibration_offset.h` — pure offset-application function
- [x] `n2k_senders.h` — `ExpiringValue<T>` + `N2kHeadingSender` (PGN
      127250 only, adapted from the parent firmware)
- [x] `hwt3100_calibration_commands.h` — `CalibrationCommandHandler`
- [x] `serial_terminal.h/.cpp` — `SerialTerminal` (config-REST-exposed
      ring buffer)
- [x] `gateway.cpp` — full wiring
- [x] Update ARCHITECTURE.md/SPEC.md for the transport correction and
      the deferred fault-indication scope
- [x] Verify: `pio run -e halser`, `pio test -e native`

## Files to Create/Modify

- `src/calibration_offset.h` (new)
- `src/n2k_senders.h` (new)
- `src/hwt3100_calibration_commands.h` (new)
- `src/serial_terminal.h` / `.cpp` (new)
- `src/gateway.cpp` (rewrite from stub)
- `ARCHITECTURE.md`, `SPEC.md` (transport correction, scope notes)
