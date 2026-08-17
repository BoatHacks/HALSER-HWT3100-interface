# Implementation Plan: Publish navigation.rateOfTurn to SignalK

## Overview

`navigation.rateOfTurn` (rad/s, +ve = starboard) is a standard SignalK
key. This firmware already computes rate of turn for PGN 127251 but
never published it as a SignalK delta — an oversight, not a deliberate
SPEC decision (unlike the raw-magnetic-field omission, which is
intentional per SPEC §5.2/§10).

## Approach

Added a second `SKOutputFloat` in `gateway.cpp`, mirroring the existing
`sk_heading_output` pattern: `SKMetadata("rad/s", ..., kHeadingTimeoutSeconds)`
for the same `meta.timeout` staleness mechanism (SPEC §6), set inside
the existing `heading_producer` consumer lambda right where
`rate_of_turn_sender->rate_of_turn_.update()` already happens — same
gate (`RateOfTurnEstimator::GetRateOfTurn()` returning true) and same
`signalk_enabled` flag as the heading output.

## Files Modified

- `src/gateway.cpp`
- `SPEC.md` §5.2
- `ARCHITECTURE.md` §1 diagram, §2.7
