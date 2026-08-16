# Implementation Plan: Windowed least-squares rate-of-turn (vs. two-point difference)

## Overview

Replaces `RateOfTurnEstimator`'s two-point (oldest/newest-in-window)
finite-difference calculation with a windowed least-squares slope over
every buffered sample in the window, per user direction after
researching how rate of turn is computed elsewhere.

## Relevant SPEC/ARCHITECTURE Sections

SPEC.md §3, §10; ARCHITECTURE.md §2.4a.

## Research findings

- Marine gyrocompass rate-of-turn indicators (patent literature):
  low-pass-filtered derivative of heading, filter time constant tuned to
  vessel size/sea state.
- pypilot (real open-source marine autopilot): computes heading rate as
  the derivative of its already-filtered pose estimate, with an explicit
  discussion that differentiating a *good* filtered estimate beats
  raw-signal-then-filter.
- SignalK's own `signalk-derived-data` plugin has no rate-of-turn
  calculator at all — nothing to borrow there.
- General numerical-differentiation literature: two-point finite
  differences are known to amplify noise; the standard fix for noisy
  sampled data is a windowed least-squares (or higher-order polynomial)
  fit, using the slope of the fitted line/curve as the derivative.

The last point directly applied to our existing implementation: the
ring buffer already stores up to 64 samples per window, but the old
`GetRateOfTurn()` only ever used 2 of them (whichever were oldest and
newest within the window), discarding the rest.

## Approach

`GetRateOfTurn()` now accumulates `Σt, Σh, Σt², Σth` across every sample
from the first-in-window through newest, then solves the standard
two-variable least-squares normal equations for the slope
(`(nΣth − ΣtΣh) / (nΣt² − (Σt)²)`). Heading unwrapping (shortest signed
angular delta, one step at a time) now runs across the whole included
range, not just between the two endpoints, so multi-sample sequences
that cross the 0/360 boundary still fit correctly. `t` values are kept
relative to the first included sample's timestamp (not raw `millis()`)
for numerical stability; accumulation uses `double` even though inputs/
outputs stay `float`, a standard precision-preserving pattern for sums
of squared values.

Backward compatibility: a least-squares fit through exactly 2 points has
one mathematically unique solution — the line through them — so this is
provably identical to the old two-point method whenever the window
contains exactly 2 samples. Every pre-existing 2-sample unit test's
expected value is therefore unchanged.

## Test Strategy

Added two new tests exercising the ≥3-sample case specifically:
- `test_regression_uses_all_samples_not_just_endpoints`: a 3-point
  sequence with an interior sample off the line connecting the
  endpoints. Asserts the result is measurably different from what the
  old two-point method would give (a naive endpoints-only calculation),
  confirming the interior sample actually influences the fit.
- `test_wraparound_regression_multiple_samples`: 3 collinear samples
  crossing 0°/360°, verifying per-step unwrapping across the whole
  window (not just the endpoints) still produces the correct linear
  slope.

All pre-existing tests re-verified unchanged (no expected-value edits
needed). `pio run -e halser` confirms the real build still compiles;
`pio test -e native` — 23/23 tests pass (was 21; +2 new).

## Implementation Steps

- [x] Rewrite `RateOfTurnEstimator::GetRateOfTurn()` for windowed
      least-squares
- [x] Add the two new regression-specific tests
- [x] Update SPEC.md/ARCHITECTURE.md
- [x] Verify: `pio run -e halser`, `pio test -e native`

## Files to Create/Modify

- `src/rate_of_turn.cpp`
- `test/test_rate_of_turn/test_rate_of_turn.cpp`
- `SPEC.md`, `ARCHITECTURE.md`
