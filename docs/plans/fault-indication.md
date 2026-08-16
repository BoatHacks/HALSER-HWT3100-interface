# Implementation Plan: Stale-data fault indication

## Overview

Completes SPEC.md §6's fault-handling requirement: when the HWT3100 goes
stale, actively signal it via SignalK notification (the LED half is
dropped — see below). N2K's half was already covered for free via
`ExpiringValue`.

## Relevant SPEC/ARCHITECTURE Sections

SPEC.md §6, §9.1; ARCHITECTURE.md §2, §6.

## Two more implementation-time findings

1. **The RGB LED is already claimed by SensESP.** `platformio.ini`'s
   `-D PIN_RGB_LED=8` build flag (inherited from the parent firmware)
   makes `sensesp_app.h` auto-instantiate an `RGBSystemStatusLed` on
   GPIO8, unconditionally writing WiFi/WebSocket connection-state colors
   every 5ms via its own internal timer. `gateway.cpp` was *also*
   separately driving the same physical LED with its own
   `Adafruit_NeoPixel` instance since the gateway-wiring change — a
   latent conflict, not something introduced by this change. Checked
   for a pause/override hook to time-share the LED; none exists in
   SensESP's public API (`BaseSystemStatusLed::show()` has no way to be
   suspended). User decision: drop the firmware's own `Adafruit_NeoPixel`
   entirely; SensESP's status LED keeps sole ownership of GPIO8. Fault
   indication becomes SignalK-notification-only — no dedicated LED
   color for it.
2. **SensESP has no built-in "send a notification" helper** — only
   `SKPrefixListener`, for *receiving* `notifications.*`. Verified the
   underlying mechanism is generic, though: `SKDeltaQueue` sweeps
   `SKEmitter::get_sources()` unconditionally and calls
   `as_signalk_json()` on each (the same mechanism `SKOutput<T>` itself
   uses). A custom class subclassing `sensesp::SKEmitter` directly and
   overriding `as_signalk_json()` to build a `{state, message}` object is
   the same generic extension point, not a hack layered on top of it.

## Approach

- `signalk_notification.h`: `SKNotification`, a minimal `SKEmitter`
  subclass. `Set(state, message)` updates in-memory fields; overridden
  `as_signalk_json()` builds `{"path": ..., "value": {"state": ...,
  "message": ...}}` per the SignalK notification schema. Path:
  `notifications.navigation.headingMagnetic` — nested under the related
  data path, per SignalK convention.
- `gateway.cpp`: a periodic check (piggybacking on the existing 100ms
  N2K send loop rather than adding a second timer) reads
  `heading_sender->heading_.is_valid()` and calls
  `fault_notification->Set(...)` accordingly — `"alarm"` when stale,
  `"normal"` otherwise. Gated by `signalk_enabled`: when the user has
  disabled SignalK output, the notification is held at `"normal"`
  regardless of actual staleness, consistent with "disabled" meaning "no
  SignalK output," full stop, rather than a partial output.
- Remove `Adafruit_NeoPixel` from `gateway.cpp` and its `lib_deps` entry
  in `platformio.ini` (FastLED stays — it's SensESP's own dependency for
  `CRGB`/`system_status_led.h`, unrelated to our removed usage).

## Test Strategy

No new pure logic to unit test — this is UI/wiring glue around
`ExpiringValue::is_valid()`, which is already exercised indirectly.
Verification: `pio run -e halser` (confirms `SKNotification` compiles
against SensESP's actual `SKEmitter`/`ArduinoJson` types) and
`pio test -e native` (regression check, unaffected by this change). No
live SignalK server was available to confirm the notification's JSON
shape is accepted/displayed correctly by a real server — flagged as an
open question.

## Implementation Steps

- [x] Remove gateway.cpp's own `Adafruit_NeoPixel` usage
- [x] Remove `Adafruit NeoPixel` from platformio.ini lib_deps
- [x] `signalk_notification.h`: `SKNotification`
- [x] Wire into `gateway.cpp`: create the notification, update it from
      the periodic send loop
- [x] Update SPEC.md/ARCHITECTURE.md
- [x] Verify: `pio run -e halser`, `pio test -e native`

## Files to Create/Modify

- `src/signalk_notification.h` (new)
- `src/gateway.cpp`
- `platformio.ini`
- `SPEC.md`, `ARCHITECTURE.md`
