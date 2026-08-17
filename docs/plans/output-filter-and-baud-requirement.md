# Implementation Plan: AT+FILT config + 115200 baud / TTL-only requirements

## Overview

Two additions from user direction:

1. Expose the HWT3100's `AT+FILT` output smoothing filter as persisted
   config (previously listed as deferred, SPEC §9.2).
2. Document two hardware prerequisites that were previously implicit/
   missing: this firmware only supports the HWT3100-**TTL** variant, and
   the module must be reconfigured to 115200 baud (`AT+UART=1`) *before*
   wiring it to HALSER — this firmware talks to it at 115200, not the
   module's factory-default 9600.

## AT+FILT

Per the manual §5.3.1: `AT+FILT=<n>`, `n` in `[1, 999]` sets filter
strength (smaller = smoother); `n = 0` or `1000` both mean "no filter"
(0 is the module's own default) — these aren't out-of-range inputs to
reject, they're valid commands with that specific meaning.

- `FormatFilterCommand()` (`hwt3100_filter_command.h/.cpp`) — pure,
  unit-tested function: clamps to `[0, 1000]`, formats
  `"AT+FILT=<n>\r\n"`, returns the clamped value actually written.
- `HWT3100SerialIO::SetOutputFilter(int)` — the second (and last)
  parameterized write method on the class, alongside the existing
  three-value `SendCommand(HWT3100Command)`. Still not a raw-text
  backdoor: the only text that can reach the wire through this method is
  `AT+FILT=<clamped integer>\r\n`.
- Persisted as a single `PersistingObservableValue<int>` config item
  (`/hwt3100/output_filter`, default 0), sent to the module once at
  boot and again on every config change — unlike the one-shot
  calibration-trigger config items (§8.2), this isn't a fire-and-reset
  trigger, since the module can't report its current filter setting
  back and a reboot would otherwise silently drop back to 0.

## Baud rate / model requirement

Discovered (from the user, reading the manual) that `AT+UART=1` sets
115200 baud, and this firmware requires that setting rather than the
module's factory-default 9600. `kHWT3100DefaultBaud`
(`halser_const.h`) changed from 9600 to 115200.

This creates a chicken-and-egg dependency the docs need to be explicit
about: the firmware can't send `AT+UART=1` itself, because it can't
talk to the module (at 115200) until the module is already at 115200.
So this must happen as a one-time manual step — module connected to a
PC/USB-TTL adapter at its *current* (9600) baud, `AT+UART=1` sent
there — *before* the module is ever wired to HALSER.

Also documented: this firmware supports the HWT3100-**TTL** variant
only, not HWT3100-232/485 (different physical layer, incompatible with
HALSER's UART terminal block).

## Test Strategy

`test/test_hwt3100_filter_command/test_hwt3100_filter_command.cpp` — 7
cases: a typical mid-range value, both "no filter" sentinels (0 and
1000, asserted as valid/unchanged, not clamped-as-if-invalid),
below/above-range clamping, and both boundary values (1, 999).

`pio run -e halser` (build) and `pio test -e native` (30/30, up from
23) both verified.

## Files Modified

- `src/hwt3100_filter_command.h` / `.cpp` (new)
- `test/test_hwt3100_filter_command/test_hwt3100_filter_command.cpp` (new)
- `src/hwt3100_serial.h` / `.cpp`
- `src/gateway.cpp`
- `src/halser_const.h`
- `platformio.ini`
- `SPEC.md` §1.2, §8.2/§8.2a, §9.1/§9.2
- `ARCHITECTURE.md` §5
