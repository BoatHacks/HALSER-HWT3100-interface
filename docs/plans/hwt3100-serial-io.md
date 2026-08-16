# Implementation Plan: HWT3100 hardware serial I/O

## Overview

The hardware-facing half of ARCHITECTURE.md §2.1: a dedicated FreeRTOS
task that owns `Serial1`, reads bytes into lines, calls the already-tested
`ParseHWT3100Line()` (docs/plans/hwt3100-serial-parser.md), and marshals
results to the main SensESP loop. Also implements the *only* write path
to the HWT3100 — `SendCommand(HWT3100Command)` — with no code path that
accepts anything but the three closed enum values.

## Relevant SPEC/ARCHITECTURE Sections

- SPEC.md §1.2 (wire format, `AT+MODE` hazard), §2 (domain rule: never
  send `AT+MODE`), §8.2 (calibration command allowlist)
- ARCHITECTURE.md §2.1 (component split), §6 (security: single write
  chokepoint), §7 (file structure)

## Approach

- `HWT3100SerialIO` owns `Serial1` exclusively. Constructed with
  references to two `sensesp::TaskQueueProducer` instances (owned by the
  caller, `gateway.cpp`, since they need the main event loop) — one for
  parsed `HeadingReading`s, one for raw lines (for the serial terminal,
  §8.1) — so results cross the FreeRTOS task boundary the same way the
  parent firmware's NMEA0183 pipeline does.
- Raw lines are marshaled as a small fixed-size POD (`HWT3100RawLine`),
  not `Arduino::String`, to avoid heap allocation across the task
  boundary — `SafeQueue`'s own docs call out heap fragmentation as a
  correctness risk for cross-task data.
- `timestamp` is stamped with `millis()` in the read task, immediately
  after a successful parse — this is why `ParseHWT3100Line()` deliberately
  doesn't set it itself (see the earlier plan).
- `SendCommand()` switches on the closed `HWT3100Command` enum with no
  `default:` case, so adding an enum value without updating the command
  table is a compiler warning, not a silent gap.

## Test Strategy

The parsing logic already has host-side unit tests. This component is
Arduino/FreeRTOS-dependent (owns a real `HardwareSerial`, spawns a real
task) and can't run in the `native` test environment — verification here
is a successful `pio run -e halser` build (compiles against the real
ESP32 framework) plus manual hardware-in-the-loop testing once wired into
`gateway.cpp` and connected to a real HWT3100 module (tracked as a
follow-up, not part of this change since `gateway.cpp` doesn't
instantiate any components yet).

## Implementation Steps

- [x] Add `HWT3100RawLine` to `hwt3100_types.h`
- [x] Add `src/hwt3100_serial.h/.cpp`
- [x] Verify `pio run -e halser` still builds
- [x] Update ARCHITECTURE.md if the implementation diverged from the plan

## Files to Create/Modify

- `src/hwt3100_types.h` (add `HWT3100RawLine`)
- `src/hwt3100_serial.h` (new)
- `src/hwt3100_serial.cpp` (new)
