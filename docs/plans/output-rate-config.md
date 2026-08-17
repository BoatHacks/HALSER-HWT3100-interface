# Implementation Plan: AT+PRATE config, with auto-detection at startup

## Overview

Expose the HWT3100's `AT+PRATE` output data rate as persisted config,
per user direction — with a twist requested alongside it: if the
persisted value is still unknown at startup, query the sensor
(`AT+PRATE=?`) rather than forcing an assumed default.

## Why not just default to something, like AT+FILT does

`AT+FILT`'s default (0 = off) is always safe to force. `AT+PRATE=0`
is not: it's the module's *single-return mode*, which stops the
continuous unsolicited data stream this firmware's entire read
pipeline is built around parsing. There is no polling/request-response
mechanism implemented anywhere in this firmware, so forcing an
assumed rate (especially 0, but even an arbitrary nonzero guess) risks
silently overwriting a rate someone may have already configured for a
reason. Querying first and learning the real value avoids that.

## Approach

- `hwt3100_prate_command.h/.cpp` — two pure, unit-tested functions:
  - `FormatPrateCommand(int, buf, len)`: clamps to `{0} u [10, 10000]`
    (values in `(0, 10)` round *up* to 10, not down to 0 — rounding
    into "silence the stream" would be a much bigger surprise than
    rounding into the nearest valid periodic rate).
  - `ParsePrateReply(line, *out)`: parses the module's `+PRATE=<n>`
    query response.
  - `kPrateUnknown = -1`: sentinel for "not yet known," outside the
    real `{0} u [10, 10000]` domain.
- `HWT3100SerialIO` gets two more write methods (alongside
  `SetOutputFilter`): `SetOutputRate(int)` and `QueryOutputRate()`
  (fixed `"AT+PRATE=?\r\n"`, no parameter).
- `gateway.cpp`:
  - New persisted config item, default `kPrateUnknown`.
  - A second consumer attached to the existing `raw_line_producer`
    (already flowing to the serial terminal) tries `ParsePrateReply()`
    on every line; while the config value is still `kPrateUnknown`, a
    match is accepted as the module's answer and persisted via
    `output_prate->set()`.
  - At boot: if still `kPrateUnknown`, send the query; otherwise
    (re-)apply the known value directly — mirroring `AT+FILT`'s
    apply-at-boot-and-on-change pattern.
  - The same `connect_to()` used for config-UI-driven changes also
    fires when the learned value gets persisted, so it re-sends
    `AT+PRATE=<n>` right back — a harmless echo confirming what the
    module just reported, not a second, different write path.

## Test Strategy

`test/test_hwt3100_prate_command/test_hwt3100_prate_command.cpp` — 13
cases: format clamping (typical value, 0, negative, small-positive
round-up, over-max, both boundaries) and reply parsing (well-formed,
zero, trailing CRLF, rejecting an ordinary heading data line, rejecting
a malformed reply, rejecting null arguments).

`pio run -e halser` (build) and `pio test -e native` (43/43, up from
30) both verified.

## Files Modified

- `src/hwt3100_prate_command.h` / `.cpp` (new)
- `test/test_hwt3100_prate_command/test_hwt3100_prate_command.cpp` (new)
- `src/hwt3100_serial.h` / `.cpp`
- `src/gateway.cpp`
- `platformio.ini`
- `SPEC.md` §8.2, §8.2b, §9.1/§9.2
- `ARCHITECTURE.md` §2.2c, §6, §9
