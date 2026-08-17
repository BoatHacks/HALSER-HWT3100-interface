# Implementation Plan: Declare PGN 127250/127251 to ExtendTransmitMessages()

## Overview

User observed the firmware's advertised "Supported PGNs" (via PGN
126464, "PGN List — Transmit") only listed the NMEA2000-library's own
automatic boilerplate: 60928 (address claim), 126993 (heartbeat),
126996 (product info), 126998 (config info) — missing 127250 (Vessel
Heading) and 127251 (Rate of Turn), which this firmware actually
transmits every 100ms.

## Root Cause

`gateway.cpp` never called `tNMEA2000::ExtendTransmitMessages()`. Per
the library's own docs: "Library responds automatically to PGN 126464
request about transmit or receive messages. With this function you
extend library list of messages your device own logic sends" — and
"some devices refuses to handle PGNs from devices which does not list
them on transmit messages." So this wasn't just a cosmetic omission:
some MFDs may not recognize this device as a heading/rate-of-turn
source even though the data is genuinely on the bus. The parent
firmware (`HALSER-default-firmware`) has the same gap — not something
introduced by this project, but worth fixing here regardless.

## Approach

Added a file-scope `const unsigned long kTransmitMessages[] PROGMEM =
{127250L, 127251L, 0}` (0-terminated per the library's convention;
`PROGMEM` is a no-op on ESP32 but included to match the documented API
exactly) and `nmea2000->ExtendTransmitMessages(kTransmitMessages)`
right after `SetMode()`, before `Open()`.

Deliberately *not* added: PGN 130850 (MFD calibration command, which
this device *receives*, not transmits) or 130851 (the calibration
acknowledgment, sent only conditionally in reply to 130850, not
periodically broadcast) — `ExtendTransmitMessages()` is for PGNs a
device actively/periodically transmits as its own data, not
reply-only proprietary messages.

## Verification

`pio run -e halser` (build) and `pio test -e native` (52/52) both
verified. Not independently unit-testable — this is ESP32/library
device-setup wiring, not pure logic.

## Files Modified

- `src/gateway.cpp`
- `SPEC.md` §5.1
- `ARCHITECTURE.md` §2.4
