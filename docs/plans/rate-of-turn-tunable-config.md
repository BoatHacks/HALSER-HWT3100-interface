# Implementation Plan: Rate-of-turn window/min-span as runtime config

## Overview

SPEC §11 flagged the rate-of-turn sliding window (2000ms) and minimum
span (500ms) as reasonable defaults, not derived from a real
helm/autopilot's sensitivity requirements, needing real-hardware
tuning. Rather than guess better constants, expose both as config so
tuning doesn't require a firmware rebuild each time.

## Approach

- `RateOfTurnEstimator` (`rate_of_turn.h/.cpp`) gets two new public
  setters, `SetWindowMs(unsigned long)`/`SetMinSpanMs(unsigned long)`
  — the existing `window_ms_`/`min_span_ms_` members were already
  plain (non-`const`) fields, so no internal restructuring needed;
  `GetRateOfTurn()` already reads them fresh on every call.
- `gateway.cpp`: two persisted config items,
  `rate_of_turn_window_ms` (default 2000, bounds e.g. 500-10000) and
  `rate_of_turn_min_span_ms` (default 500, bounds e.g. 100-5000).
  Applied at boot and on every config change (plain defaults, no
  unknown-sentinel/discovery needed — unlike `AT+FILT`/`AT+PRATE`,
  there's no hardware round-trip or existing state to preserve, just a
  firmware-side number).
- `min_span_ms` is clamped to never exceed `window_ms` wherever either
  is applied: a min-span larger than the window it's measured within
  can never be satisfied (the window caps the maximum possible span),
  which would silently make `GetRateOfTurn()` always return "not
  available" — a foot-gun worth guarding against rather than
  documenting as a way to misconfigure the device into silence.

## Test Strategy

Two new tests in `test/test_rate_of_turn/test_rate_of_turn.cpp`:
verify `SetWindowMs()` actually changes which samples are included
(a sample that's in-window at the default 2000ms falls out after
narrowing the window to e.g. 1000ms), and verify `SetMinSpanMs()`
changes whether `GetRateOfTurn()` accepts a given sample spacing.
`pio run -e halser` (build) and `pio test -e native` (up from 52) both
verified.

## Files Modified

- `src/rate_of_turn.h` / `.cpp`
- `test/test_rate_of_turn/test_rate_of_turn.cpp`
- `src/gateway.cpp`
- `SPEC.md` §7, §11
- `ARCHITECTURE.md` §2.4a
