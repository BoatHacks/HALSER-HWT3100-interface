# Implementation Plan: Precision-9 N2K identity + computed rate of turn

## Overview

Reworks the N2K interface to present as a B&G Precision-9 (device
identity cloned from `htool/ESP32_Precision-9_compass_CMPS14`) and adds
PGN 127251 (Rate of Turn), computed from a sliding window of heading
readings rather than sensed or faked.

## Relevant SPEC/ARCHITECTURE Sections

SPEC.md §1.2, §1.3, §3, §5.1, §9.1, §10, §11.

## Approach

- `RateOfTurnEstimator` (new, `rate_of_turn.h/.cpp`): pure logic, no
  Arduino dependency, following the same split established for
  `ParseHWT3100Line()` — a ring buffer of (heading, timestamp) samples
  over a trailing window; `GetRateOfTurn()` returns the wraparound-
  corrected angular difference between the oldest and newest sample in
  the window, divided by elapsed time, as radians/second. Requires a
  minimum elapsed span (not just "≥2 samples") before returning a value,
  to avoid amplifying noise from two samples a few milliseconds apart.
  Unit tested on the host (`pio test -e native`).
- Window length: 2000ms, minimum span: 500ms — chosen as reasonable
  defaults (SPEC §11 flags these as needing real-hardware tuning later).
- `N2kRateOfTurnSender` added to `n2k_senders.h`, same
  `ExpiringValue`-backed pattern as `N2kHeadingSender` — "not available"
  when the estimator doesn't have enough history yet, exactly like any
  other stale/unavailable reading (SPEC §6).
- Device identity constants (product info, device function/class,
  manufacturer code) moved from placeholder TODOs in `halser_const.h` to
  the Precision-9's actual values. The "unique number" passed to
  `SetDeviceInformation()` is NOT cloned — kept as the MAC-derived value
  already used, per SPEC §10's design decision.
- `gateway.cpp`: feed every corrected heading into the
  `RateOfTurnEstimator`, add the PGN 127251 sender + its own config
  toggle, gated by the master N2K switch like PGN 127250.

## Test Strategy

`RateOfTurnEstimator` unit tests (native): steady heading (rate ≈ 0),
constant clockwise turn, constant counter-clockwise turn, a turn crossing
the 0°/360° wrap boundary in both directions, too few samples (not
enough history), samples that don't yet span the minimum time window.
Plus `pio run -e halser` for the real build and `pio test -e native` for
the parser (regression check).

## Implementation Steps

- [x] `rate_of_turn.h/.cpp` + unit tests
- [x] `n2k_senders.h`: add `N2kRateOfTurnSender`
- [x] `halser_const.h`: Precision-9 identity constants
- [x] `gateway.cpp`: wire the estimator + PGN 127251 sender + config
- [x] Update ARCHITECTURE.md
- [x] Verify: `pio run -e halser`, `pio test -e native`

## Files to Create/Modify

- `src/rate_of_turn.h` / `.cpp` (new)
- `test/test_rate_of_turn/test_rate_of_turn.cpp` (new)
- `src/n2k_senders.h`
- `src/halser_const.h`
- `src/gateway.cpp`
- `ARCHITECTURE.md`, `SPEC.md`
