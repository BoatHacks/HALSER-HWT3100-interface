# Implementation Plan: Bus-sourced magnetic variation + true heading

## Overview

Follow-up from "why are Deviation/Variation empty" — this firmware has
no GPS/geomagnetic model of its own, but another N2K device (GPS,
chartplotter) often already broadcasts PGN 127258 (Magnetic
Variation). Listening for it lets this firmware fill PGN 127250's own
Variation field with real data instead of always N/A, and compute
`navigation.headingTrue`.

## Approach

- `magnetic_variation_listener.h/.cpp`: `MagneticVariationListener :
  public tNMEA2000::tMsgHandler`, filtered to PGN 127258, same
  self-attach pattern as `MfdCalibrationBridge`. On a valid message
  (`ParseN2kMagneticVariation` succeeds, value isn't `N2kDoubleNA`),
  updates an externally-owned `ExpiringValue<float>*`.
- `N2kHeadingSender` (`n2k_senders.h`) gets a second member,
  `ExpiringValue<float> variation_`, with a longer expiry than heading
  (variation changes on a geographic timescale, not per-second — a
  short expiry would flap the field on and off between an
  infrequently-rebroadcasting source's updates). `send()` passes
  `variation_.to_n2k()` instead of the hardcoded `N2kDoubleNA` for the
  Variation parameter.
- `gateway.cpp`: constructs `MagneticVariationListener` pointed at
  `heading_sender->variation_`, right after `nmea2000->Open()` (same
  spot `MfdCalibrationBridge` is constructed). In the existing
  heading-reading consumer lambda, after computing `heading_rad`: if
  `heading_sender->variation_.is_valid()`, compute
  `heading_true_rad = wrap(heading_rad + variation_rad)` and publish
  to a new `navigation.headingTrue` SKOutputFloat, gated by both
  `signalk_enabled` and a new `sk_heading_true_enabled` toggle
  (default on).
- Sign convention: SignalK/N2K both use Easterly-positive variation, so
  `true = magnetic + variation` needs no sign flip crossing between the
  two specs.
- Deliberately one-directional: never transmits PGN 127258 itself (no
  honest way to originate a variation value), and doesn't re-publish
  the raw variation as its own SignalK delta (would be a redundant
  second source for data a legitimate device on the bus already
  provides).

## Test Strategy

No new pure logic warranting host-side unit tests — `HandleMsg()` is a
thin wrapper around the library's own `ParseN2kMagneticVariation`
(already exercised by the library's own tests, not this project's to
re-test), and the true-heading wrap-into-`[0,2π)` math mirrors
`ApplyCalibrationOffset`'s already-tested degree-wrapping pattern
closely enough that a from-scratch test doesn't add much confidence
per the effort spent. `pio run -e halser` (build) and
`pio test -e native` (unaffected) both verified.

## Files Modified

- `src/magnetic_variation_listener.h` / `.cpp` (new)
- `src/n2k_senders.h`
- `src/gateway.cpp`
- `SPEC.md` §5.1, §5.1a (new), §5.2, §7, §9.1
- `ARCHITECTURE.md` §2.4, §2.4b (new), §2.7
