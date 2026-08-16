# Implementation Plan: Replace SignalK notification with meta.timeout; fix degrees/radians bug

## Overview

Replaces the active `notifications.*` alarm (docs/plans/fault-indication.md)
with SignalK's spec-defined `meta.timeout` advisory mechanism, per user
direction after checking the SignalK spec directly. Also fixes a real
degrees/radians unit bug in both N2K and SignalK heading output, found
while wiring up `meta.timeout`'s companion `units` field.

## Relevant SPEC/ARCHITECTURE Sections

SPEC.md §3, §5.1, §5.2, §6, §10, §11; ARCHITECTURE.md §2.7.

## Findings

- SignalK spec: `meta.timeout` (seconds) — "tells the consumer how long
  it should consider the value valid." Published once as metadata, not
  per-delta. SensESP's `SKMetadata` already has a `timeout_` field.
- Trade-off: per an open SignalK server RFC, `meta.timeout` isn't
  enforced/checked by most consumers today (informational only). Traded
  the guaranteed-actionable active alarm for the spec-idiomatic
  mechanism — user's explicit choice.
- Real bug: `HeadingReading.heading` is degrees internally (matches the
  HWT3100's wire format and the rate-of-turn wraparound math), but both
  `SetN2kPGN127250` and `navigation.headingMagnetic` require radians —
  neither output was converting. Caught because attaching a `"rad"`
  units label to a still-in-degrees value would have been actively
  wrong, not just an omission.

## Approach

- Remove `SKNotification`/`signalk_notification.h` entirely.
- `sk_heading_output` gets an `SKMetadata("rad", "", "", "",
  kHeadingTimeoutSeconds)` (5.0s, matching `N2kHeadingSender`'s
  `ExpiringValue` window) at construction.
- Convert degrees → radians at both output boundaries (N2K's
  `heading_sender->heading_.update()`, SignalK's `sk_heading_output->set()`)
  — not upstream, since `HeadingReading` and `RateOfTurnEstimator` both
  correctly assume degrees internally.

## Test Strategy

`pio run -e halser` (confirms `SKMetadata` usage compiles against
SensESP's real types) and `pio test -e native` (regression check,
unaffected). No live SignalK server available to confirm `meta.timeout`
is actually honored by a given consumer — flagged in SPEC.md §11, same
caveat that applied to the notification it replaces.

## Implementation Steps

- [x] Remove `signalk_notification.h`, its `gateway.cpp` usage
- [x] Add `SKMetadata` (units + timeout) to `sk_heading_output`
- [x] Fix degrees→radians conversion at both N2K and SignalK boundaries
- [x] Update SPEC.md/ARCHITECTURE.md
- [x] Verify: `pio run -e halser`, `pio test -e native`

## Files to Create/Modify

- `src/signalk_notification.h` (deleted)
- `src/gateway.cpp`
- `SPEC.md`, `ARCHITECTURE.md`
