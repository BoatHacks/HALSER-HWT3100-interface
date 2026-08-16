# Implementation Plan: MFD-triggered calibration (PGN 130850/130851)

## Overview

Lets a compatible MFD (B&G/Simrad/Navico, given the Precision-9 identity
emulation, SPEC §1.2/§5.1) start/stop the HWT3100's on-module magnetic
calibration over the N2K bus, matching how
`htool/ESP32_Precision-9_compass_CMPS14` does it — the user's explicit
direction, after that project was already used as the source for this
firmware's N2K device identity.

## Relevant SPEC/ARCHITECTURE Sections

SPEC.md §1.2, §5.1, §8.2, §10; ARCHITECTURE.md §2.2, §2.6.

## Important caveat, stated up front

**PGN 130850/130851 ("Simnet: Event Command" / its ack) is an
undocumented, reverse-engineered proprietary Navico/Simnet message, not
a published NMEA 2000 spec.** Everything below — the byte layout, the
meaning of `DEVICE_ID`, the exact command values — comes from reading
the reference implementation's source, not from an official
specification, and none of it has been verified against real B&G/Simrad
MFD hardware or a real N2K bus in this environment (none is available).
This is a best-effort, documented port, not a guarantee of compatibility.

## Findings (from `htool/ESP32_Precision-9_compass_CMPS14`)

Incoming PGN 130850, parsed starting at byte index 2 (skipping the
2-byte manufacturer-code/industry-code header standard to proprietary
PGNs):

| Byte (in order) | Reference name | Meaning |
|---|---|---|
| 1 | `Command1` | Compared against a device-identifying constant (`DEVICE_ID = 24` in the reference — see SPEC §10 for the decision to copy this value as-is despite not knowing what it represents) |
| 2, 3 | `Command2`, `Command3` | Read but not checked by the reference implementation |
| 4 | `Command4` | Must equal `18` — the "this is a calibration command" marker |
| 5 | `CalibrationStopStart` | `0` = start calibration, `1` = stop calibration |

On a match, the reference sends an acknowledgment on PGN 130851 with
this exact byte payload: `0x41, 0x9f, DEVICE_ID, 0xff, 0xff, 18,
CalibrationStopStart, 0x00, 0xff, 0xff, 0xff, 0xff` (12 bytes — this PGN
is fast-packet, not single-frame; confirmed the NMEA2000-library already
handles fast-packet transparently based on message length, since the
existing `N2kGNSSSender` (PGN 129029, ~43 bytes) already relies on
exactly this behavior).

## Approach

- `MfdCalibrationBridge` (new): a `tNMEA2000::tMsgHandler` subclass
  (the library's class-based extension point for receiving specific
  PGNs — chosen over the alternative single-function-pointer
  `SetMsgHandler` API specifically because it doesn't require routing
  through file-scope global state to reach `CalibrationCommandHandler`).
  Filters to PGN 130850 via the handler's own constructor argument.
  Parses the fields above; on a `Command4==18` match, calls the
  *existing* `CalibrationCommandHandler::StartCalibration()`/
  `EndCalibration()` (SPEC §8.2, ARCHITECTURE §2.2) — this MFD path
  reuses the same allowlisted write chokepoint the web UI calibration
  triggers already use, not a new one. Sends the PGN 130851 ack
  afterward.
- No `ClearCalibration()` MFD path exists — the reference implementation
  doesn't have one either (only start/stop). `AT+CALI=2` (clear) stays
  web-UI-only.
- `gateway.cpp` constructs `MfdCalibrationBridge` after
  `CalibrationCommandHandler`, passing both `nmea2000` and the handler —
  it needs both to attach itself to the bus and to dispatch calibration
  actions.

## Test Strategy

No pure logic to unit test in isolation here (this is N2K message
parsing tied to `tN2kMsg`, which has no meaningful native/host
representation) — verification is `pio run -e halser` (confirms the
`tMsgHandler` subclass compiles against the real library and that
`GetByte`/`AddByte`/fast-packet sending work as expected structurally)
and `pio test -e native` (regression check). Real verification — does an
actual B&G/Simrad MFD's calibration screen actually talk to this
firmware — needs real hardware, which isn't available here; flagged in
SPEC.md §11.

## Implementation Steps

- [x] `mfd_calibration_bridge.h/.cpp`
- [x] Wire into `gateway.cpp`
- [x] Update SPEC.md/ARCHITECTURE.md
- [x] Verify: `pio run -e halser`, `pio test -e native`

## Files to Create/Modify

- `src/mfd_calibration_bridge.h` (new)
- `src/mfd_calibration_bridge.cpp` (new)
- `src/gateway.cpp`
- `SPEC.md`, `ARCHITECTURE.md`
