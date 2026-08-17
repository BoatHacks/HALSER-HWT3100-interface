# Implementation Plan: The three remaining post-MVP items

## Overview

SPEC §9.2 listed three deferred items. All three implemented in this
pass:

1. `AT+UART` (runtime baud switching) exposed as config.
2. Baud-rate auto-detection.
3. Raw magnetic field as a SignalK output.

Items 1 and 2 turned out to share one underlying piece of state (what
baud are the firmware and module actually speaking) and were unified
into a single persisted config value rather than built as two separate
bolt-ons.

## Approach: baud auto-detection + runtime switching

- `hwt3100_uart_command.h/.cpp`: pure, unit-tested `FormatUartCommand()`
  — snaps a requested baud to the nearest of `{9600, 115200, 460800}`
  (tie toward the lower rate), formats the matching `AT+UART=<n>`
  index, returns the baud actually selected.
- `HWT3100SerialIO::DetectBaud()` — synchronous, blocking, tries
  candidate bauds in order (115200 recommended, then 9600 factory
  default, then 460800), opening the port at each and listening up to
  a timeout for a line that parses via `ParseHWT3100Line()`. Sends no
  commands — passive discovery only. Must run before `Begin()` starts
  the background read task (no concurrent `Serial1` access at that
  point).
- `HWT3100SerialIO::SetBaudRate()` — sends `AT+UART=<n>` at the current
  rate, a fixed settle delay, then reconfigures `Serial1` to the new
  rate.
- `gateway.cpp`: single persisted `hwt3100_baud`, starting at
  `halser::kBaudUnknown` (`-1`). At boot: if unknown, run detection and
  persist the result (no command sent — this is exactly what the
  module already is); if known, `Begin()` directly at that rate. A
  `connect_to()` consumer, attached *after* any startup detection
  `set()`, handles future explicit config changes by calling
  `SetBaudRate()` — the ordering is what keeps passive discovery from
  ever triggering an unwanted `AT+UART` command (same trick §8.2b's
  `AT+PRATE` query/reply wiring already uses).

**This changes SPEC's hardware setup instructions**: the "required
pre-wiring `AT+UART=1`" step becomes "recommended" — auto-detection
resolves the chicken-and-egg problem that made it mandatory before.
Still recommended for a faster first boot (skips ~1-2s of failed
detection attempts at slower candidate rates).

**Known, accepted limitation**: `SetBaudRate()` reconfigures `Serial1`
from the main-loop thread while the background read task may
concurrently be reading from it. Narrow race, only during an explicit,
rare user-triggered switch — documented (SPEC §11, ARCHITECTURE §2.2d)
rather than solved with added synchronization complexity.

## Approach: raw magnetic field SignalK output

Three new `SKOutputFloat`s: `sensors.hwt3100.magneticField.x/y/z`.
No standard SignalK path exists for this (unlike heading/rate of turn's
spec-defined `navigation.*` keys), so a `sensors.*` custom path was
used — SignalK's own convention for exactly this kind of non-standard
sensor data. No N2K equivalent was added: no standard PGN exists for
raw magnetometer data, and a made-up proprietary one would have no real
consumer on the bus (unlike PGN 130850/130851, which exists to
interoperate with a real, if reverse-engineered, MFD protocol).

Values are raw, uncalibrated sensor counts — no unit published (no
documented counts-to-µT factor), no calibration offset applied (offset
correction is heading-specific, SPEC §3). Gated behind a dedicated
toggle in addition to the SignalK master enable flag, off by default.

## Test Strategy

`test/test_hwt3100_uart_command/test_hwt3100_uart_command.cpp` — 9
cases: exact matches for all three rates, snap-to-nearest at both ends
of the range, both tie-break boundaries (verifying "toward the lower
rate"), and extreme (0, very large) inputs.

`DetectBaud()`/`SetBaudRate()` are not unit-tested directly — both are
inherently I/O-bound (blocking reads/writes against `Serial1`, timing-
dependent), same category as the rest of `HWT3100SerialIO`, which has
never had host-side unit tests for its hardware-facing half either
(only the pure parser/format functions it delegates to are tested).

`pio run -e halser` (build) and `pio test -e native` (52/52, up from
43) both verified.

## Files Modified

- `src/hwt3100_uart_command.h` / `.cpp` (new)
- `test/test_hwt3100_uart_command/test_hwt3100_uart_command.cpp` (new)
- `src/hwt3100_serial.h` / `.cpp`
- `src/gateway.cpp`
- `src/halser_const.h`
- `platformio.ini`
- `SPEC.md` §1.2, §5.2, §8.2c, §9.1/§9.2, §11
- `ARCHITECTURE.md` §2.1, §2.2d, §2.7, §9
