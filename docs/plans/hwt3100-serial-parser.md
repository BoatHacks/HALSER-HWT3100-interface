# Implementation Plan: HWT3100 ASCII line parser

## Overview

Parse the HWT3100's default ASCII-mode data stream
(`Magx=<int>,y=<int>,z=<int>,w=<float>\r\n`) into a `HeadingReading`. This
is the first piece of ARCHITECTURE.md §2.1 (HWT3100 Serial I/O) — the
read/parse half, kept separate from the hardware serial I/O plumbing so
the parsing logic itself is unit-testable without a real board.

## Relevant SPEC/ARCHITECTURE Sections

- SPEC.md §1.2 (confirmed wire format), §3 (data model), §11 (open
  question: real-hardware parsing edge cases still need verification)
- ARCHITECTURE.md §2.1 (HWT3100 Serial I/O), §3 (data model in code)

## Approach

Split what ARCHITECTURE.md called `hwt3100_serial.h/.cpp` into two
pieces:

- `hwt3100_types.h` — `HeadingReading` and `HWT3100Command`, no Arduino
  dependency.
- `hwt3100_parser.h/.cpp` — a pure function, `ParseHWT3100Line()`, that
  takes a line of text and fills a `HeadingReading` (heading + raw
  magnetic field only — no `timestamp`, since that's a hardware clock
  read the caller supplies, not something the parser should own). No
  Arduino dependency either, so it can be unit tested on the host without
  a board.
- The hardware-facing half (owning `Serial1`, buffering bytes into
  lines, calling the parser, and the `SendCommand()` write path) is
  deliberately **not** part of this change — it's a separate, larger
  piece involving the FreeRTOS task/`TaskQueueProducer` plumbing
  ARCHITECTURE.md describes, better done as its own plan once this parser
  is solid.

This is a documentation-affecting change: ARCHITECTURE.md §7 (File
Structure) named a single `hwt3100_serial.h/.cpp`; splitting out
`hwt3100_types.h`/`hwt3100_parser.h/.cpp` is an implementation refinement
worth reflecting there.

## Test Strategy

Unit tests (PlatformIO's `native` test environment + Unity, no board
required) covering:
- A well-formed line, positive values
- Negative `x`/`y`/`z` values (manual documents a sign-prefixed format)
- `w` with a decimal component
- Missing/malformed fields (should fail cleanly, not garbage-parse)
- Leading/trailing whitespace or a stray `\r` (serial framing noise)

Real-hardware verification (does the module actually emit exactly this
format, always in this field order) stays an open question per SPEC.md
§11 — these tests validate the parser against the documented/SDK-derived
format, not against a live module.

## Implementation Steps

- [x] Add `hwt3100_types.h`
- [x] Add `hwt3100_parser.h/.cpp`
- [x] Add `native` PlatformIO test environment + Unity tests
- [x] Update ARCHITECTURE.md §7 for the file split
- [x] Verify: `pio test -e native`, `pio run` (main firmware env still
      builds)

## Files to Create/Modify

- `src/hwt3100_types.h` (new)
- `src/hwt3100_parser.h` (new)
- `src/hwt3100_parser.cpp` (new)
- `test/test_hwt3100_parser/test_hwt3100_parser.cpp` (new)
- `platformio.ini` (add `[env:native]`)
- `ARCHITECTURE.md` (§7 file structure update)
