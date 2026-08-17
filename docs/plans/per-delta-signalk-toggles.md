# Implementation Plan: Per-delta SignalK enable/disable toggles

## Overview

User request: extend the config UI with individual enable/disable
switches for each SignalK delta output, matching the N2K side's
existing master-plus-per-PGN pattern (`n2k_enabled` +
`n2k_heading_pgn_enabled`/`n2k_rate_of_turn_pgn_enabled`).

## Approach

Two new config items, `sk_heading_enabled` and
`sk_rate_of_turn_enabled` (both default `true`), gating
`navigation.headingMagnetic`/`navigation.rateOfTurn` in addition to the
existing `signalk_enabled` master switch — same
`if (signalk_enabled->get() && <per_output>->get())` pattern already
used for N2K's `n2k_enabled->get() && n2k_heading_pgn_enabled->get()`.

`raw_mag_field_enabled` already existed as this project's first
per-delta toggle (added when the raw magnetic field output itself was
implemented); this change makes it consistent with the other two
deltas rather than a one-off, and gives every SignalK output the same
shape of control the N2K PGNs already have.

## Test Strategy

No new pure logic to unit-test — this is config wiring + a boolean
gate, same category as the existing N2K per-PGN toggles (never
unit-tested either). `pio run -e halser` (build) and
`pio test -e native` (52/52, unaffected) both verified.

## Files Modified

- `src/gateway.cpp`
- `SPEC.md` §7
- `ARCHITECTURE.md` §2.7
