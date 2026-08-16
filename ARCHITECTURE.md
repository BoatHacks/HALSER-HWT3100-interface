# HALSER-HWT3100-interface Architecture

See SPEC.md for requirements and rationale; this document covers how the
firmware is built.

## 1. Overview

```
                     UART (GPIO2 TX / GPIO3 RX), 9600 baud
                     HALSER "UART" terminal block,
                     RX-select jumper on "U"
HWT3100-TTL  ◄────────────────────────────────────► Serial1
   ▲  (read: continuous ASCII lines)   (write: AT+CALI commands only)
   │                                                       │
   │                                                       ▼
   │                                      HWT3100 ASCII line parser
   │                              "Magx=<n>,y=<n>,z=<n>,w=<n.n>\r\n"
   │                                                       │
   │                                            HeadingReading (heading,
   │                                            magX/Y/Z, timestamp)
   │                                                       │
   │                             ┌──────────────────┬──────┴────────────┐
   │                             ▼                  ▼                   ▼
   │                   Calibration offset    Live serial terminal   (raw lines
   │                   applied (once)        tap (text display)     also feed here)
   │                             │            read-only display
   │             ┌───────────────┴───────────────┐  │
   │             ▼                                ▼  ▼
   │   N2K sender (PGN 127250,            SignalK delta sender      Web UI (WebSocket)
   │   ExpiringValue pattern)              (SensESP, WiFi)
   │             │                                │
   │             ▼                                ▼
   │ tNMEA2000_esp32 (TWAI, GPIO4/5)      SignalK server (WiFi)
   │
   └──────────────── Calibration command handler ◄──── Web UI (named actions,
                      (fixed AT+CALI list, §2.2)        not free text)
```

Single ESP32-C3 firmware, no boot-mode routing (unlike the parent
HALSER-default-firmware — no test-jig requirement, see SPEC §10 Design
Decisions). One FreeRTOS task reads the HWT3100's ASCII text stream; the
main SensESP event loop drives calibration, N2K sending, SignalK delta
publishing, and (occasionally, on user action) calibration command
writes.

The HWT3100 is a **compass, not an IMU** — it reports only magnetic
heading and raw magnetic field (X/Y/Z), never pitch/roll (SPEC §1.2, §3).
Its default **ASCII mode** already streams everything this firmware
needs as plain text and accepts the calibration commands as plain `AT+`
text — the firmware never switches the module into Modbus mode and never
implements `AT+MODE` (SPEC §1.2, §2). This is not a feature gap; it's
deliberate and permanent (SPEC §9.3).

## 2. System Components

### 2.1 HWT3100 Serial I/O (`hwt3100_serial.h/.cpp`, plus `hwt3100_parser.h/.cpp`)

Owns the UART1 (`Serial1`) handle exclusively — no other component ever
touches `Serial1` directly. Two directions, both gated:

- **Read** (continuous, dedicated FreeRTOS task, mirroring the parent
  firmware's `NMEA0183IOTask` pattern so reads aren't blocked by WiFi/N2K
  work on the main loop): accumulates bytes until `\r\n`, then hands the
  line to `ParseHWT3100Line()` — a pure function, in its own file
  (`hwt3100_parser.h/.cpp`) with no Arduino dependency, that parses
  `Magx=<int>,y=<int>,z=<int>,w=<float>` into a `HeadingReading` (§3) and
  is unit-tested on the host (`pio test -e native`,
  docs/plans/hwt3100-serial-parser.md) rather than only against real
  hardware. `hwt3100_serial.h/.cpp` itself is just the thin hardware
  wrapper around it: byte accumulation, calling the parser, setting
  `timestamp` from `millis()` (deliberately not the parser's job — a
  hardware clock read has no business in a function meant to be testable
  without a board), and marshaling to the main loop via
  `TaskQueueProducer`, same mechanism as the parent's NMEA0183 pipeline.
  No binary framing/checksum — this is line-based ASCII text parsing,
  simpler than the parent firmware's NMEA 0183 sentence parsers. Also
  taps every raw line to the serial terminal broadcaster (2.5). The two
  `TaskQueueProducer` instances (one for `HeadingReading`, one for the
  raw-line tap — a fixed-size `HWT3100RawLine` POD, not `Arduino::String`,
  to keep the cross-task queue allocation-free) are constructed by, and
  owned by, `gateway.cpp` (2.6) and passed into `HWT3100SerialIO` by
  pointer — `HWT3100SerialIO` has no business knowing about the main
  event loop needed to construct one.
- **Write** (only from the calibration command handler, 2.2): exposes a
  single entry point, `SendCommand(HWT3100Command cmd)`, where
  `HWT3100Command` is a closed enum with exactly three values —
  `kStartCalibration`, `kEndCalibration`, `kClearCalibration` — mapping to
  `AT+CALI=1`, `AT+CALI=0`, `AT+CALI=2` respectively (SPEC §8.2). There is
  no method on this class that accepts raw bytes or arbitrary text, and
  **no enum value for `AT+MODE` exists at all** — this isn't a value
  that's excluded from a table, it's a value that was never added to the
  enum's definition. Extending the enum to add it would require a
  deliberate code change touching this file directly (SPEC §1.2, §2).

### 2.2 Calibration Command Handler (`hwt3100_calibration_commands.h/.cpp`)

Receives named calibration actions from the web UI (three buttons: Start
Calibration, End Calibration, Clear Calibration) and maps each to an
`HWT3100Command` enum value, then calls `HWT3100SerialIO::SendCommand`
(2.1). This is the *only* component with access to `SendCommand` — the
serial terminal (2.5) is display-only and has no reference to it. Holding
the write capability in one small, auditable component (rather than
spreading "can write to the sensor" across the codebase) keeps the
allowlist enforcement easy to verify by reading one file.

### 2.3 Calibration Offset

Applies the configured heading offset to each `HeadingReading` before it
reaches either output path (SPEC §2, §10 Design Decisions). Pure function
of (raw reading, offset) → corrected reading; no state beyond the offset
itself, which lives in the config system (2.6). Distinct from 2.2
(in-place calibration commands): this is a software-only correction
applied to every reading; 2.2 drives the HWT3100's own on-module magnetic
calibration procedure via serial commands. The two are related in purpose
(both about getting accurate heading out of the module) but don't share
code — one never touches the serial link, the other only does.

### 2.4 N2K Sender (`n2k_senders.h`)

Reuses the parent firmware's `N2kHeadingSender` (PGN 127250) as-is,
unmodified — it already exists, already uses the `ExpiringValue<T>`
pattern, and already does exactly what SPEC §6 (stale-data behavior)
asks: `ExpiringValue::to_n2k()` returns `N2kDoubleNA` once a value goes
stale, satisfying the "transmit N2K not-available values" decision for
free. No PGN 127257 (Attitude) sender exists — the hardware can't
supply pitch/roll, so there's nothing to send it with (SPEC §5.1, §10).

Has its own enable/disable flag (config, 2.6) checked before `send()` is
called from the periodic send loop, independent of the `ExpiringValue`
staleness mechanism — "disabled" and "stale" are different states
(disabled = never sends; stale = sends N/A values).

### 2.5 Serial Terminal Broadcaster

Receives raw lines from 2.1 and forwards them to the web UI as plain
text, per SPEC §8.1 — no hex/decoded toggle needed since the wire format
is already human-readable ASCII (unlike the binary-protocol assumption in
an earlier draft of this design). Pushes to connected browsers over a
dedicated WebSocket endpoint (2.7 covers why a dedicated endpoint, not
SensESP's own channel). Display-only, per §2.1/§2.2 — this component
never gets a reference to `SendCommand`.

### 2.6 Configuration / Web UI Wiring

Uses SensESP's `ConfigItem` + `PersistingObservableValue` pattern (as the
parent firmware does for NMEA 0183 bit rate) for every SPEC §7 config
value: N2K master enable, PGN 127250 enable, SignalK enable, calibration
offset, WiFi/SignalK connection (SensESP's own built-in config UI). The
calibration *command* action buttons (§8.2/2.2) live here too, as a UI
concern, but the actual command dispatch is 2.2's responsibility, not
this component's.

### 2.7 SignalK Delta Sender

Publishes `navigation.headingMagnetic` via SensESP's existing
SignalK/WiFi transport, gated by the SignalK master enable flag (2.6).
Uses the same corrected `HeadingReading` as the N2K sender (2.4) — single
source of truth, per SPEC §2. Raw magnetic field is not published here
(SPEC §5.2, §10) — it's diagnostic-only, visible via 2.5.

## 3. Data Models

See SPEC.md §3 for the conceptual model. In code:

```cpp
struct HeadingReading {
  float heading;      // degrees, 0-360, magnetic, offset-corrected
  int16_t mag_x;       // raw magnetic field X, diagnostic use only
  int16_t mag_y;       // raw magnetic field Y, diagnostic use only
  int16_t mag_z;       // raw magnetic field Z, diagnostic use only
  unsigned long timestamp;  // millis() of last successful sensor read
};

struct CalibrationOffset {
  float heading_offset = 0.0f;
};

enum class HWT3100Command {
  kStartCalibration,   // AT+CALI=1
  kEndCalibration,     // AT+CALI=0
  kClearCalibration,   // AT+CALI=2
};
```

No pitch/roll fields exist, deliberately (SPEC §3, §9.3) — the HWT3100
cannot produce them. `HWT3100Command` has exactly three values; there is
no value for `AT+MODE` (SPEC §1.2, §2).

## 4. Technology Stack

Same as the parent firmware (see its AGENTS.md), plus one addition:

| Layer | Choice | Why |
|---|---|---|
| Framework | Arduino (ESP32-C3), SensESP 3.2.0 | Reused for consistency with the HALSER family; gets WiFi, web UI, SignalK client, OTA for free. |
| N2K | ttlappalainen/NMEA2000-library + NMEA2000_twai | Same as parent; reuses the parent's existing `N2kHeadingSender`/`SetN2kPGN127250` as-is. |
| Serial parsing | Custom (this project) | Simple line-based ASCII parsing (`Magx=<n>,y=<n>,z=<n>,w=<n.n>\r\n`) — no library needed, comparable effort to the parent's NMEA 0183 sentence parsers but simpler (no checksum). |
| Live terminal transport | `ESPAsyncWebServer`/`AsyncWebSocket` (bundled with SensESP's dependencies) | SensESP is itself built on ESPAsyncWebServer; adding one more `AsyncWebSocket` endpoint alongside it avoids pulling in a second web/socket stack. To be confirmed against the exact SensESP 3.2.0 dependency tree during implementation. |
| RGB LED | Adafruit NeoPixel | Reused as-is for fault indication (SPEC §6). |

## 5. Integration Points

- **HWT3100-TTL module** — UART1, `Serial1`, GPIO2 TX / GPIO3 RX, 9600
  baud (module default), wired to HALSER's UART terminal block with the
  RX-select jumper on "U" (confirmed against Hat Labs' HALSER hardware
  docs — the board has one shared UART peripheral muxed by that jumper
  across NMEA0183/RS-232/UART connectors). Protocol confirmed from the
  vendor manual + SDK: ASCII mode, comma-delimited text lines for data,
  `AT+`-prefixed text commands for calibration (SPEC §1.2). Exact parsing
  edge cases remain an open question for implementation-time hardware
  testing (SPEC §11).
- **NMEA 2000 bus** — via `tNMEA2000_esp32`, GPIO4 TX / GPIO5 RX (TWAI),
  unchanged from the parent firmware.
- **SignalK server** — via SensESP's WiFi + mDNS discovery + WebSocket
  delta client, unchanged from the parent firmware's mechanism.
- **Browser (web UI)** — SensESP's own config UI, plus the dedicated
  WebSocket endpoint for the serial terminal (2.5, 2.7).

## 6. Security Considerations

Same trust model as the parent firmware: a device on a private boat LAN,
where physical/network access already implies a high level of trust.
SensESP's existing defaults (WiFi credentials in NVS, whatever auth
SensESP's config UI provides out of the box) are sufficient — no
additional auth/encryption work scoped for this project (per SPEC's user
decision).

The hardware-specific rule that *is* a hard requirement, not a
convention: **the firmware must never send `AT+MODE` to the HWT3100**
(SPEC §1.2, §2) — `AT+MODE=1` has bricked a real unit. Unlike an earlier
draft of this design (which treated this as "writes are allowed, but
filtered"), the actual enforcement here is stronger: the firmware simply
has no functional need for Modbus mode, so nothing implements it.

- `Serial1` is owned exclusively by `HWT3100SerialIO` (2.1); no other
  component ever gets a reference to it.
- `HWT3100SerialIO::SendCommand()` is the *only* method that writes to
  `Serial1`, and it takes a closed `HWT3100Command` enum with exactly
  three values (2.1, §3) — `kStartCalibration`, `kEndCalibration`,
  `kClearCalibration`. There is no overload, no debug backdoor, no
  "advanced mode" that accepts raw text, and **no enum value that maps to
  `AT+MODE` at all** — not "filtered out," simply never defined.
- The calibration command handler (2.2) is the only component holding a
  reference to `SendCommand`; the serial terminal (2.5) and everything
  else remain read-only, with no path to `Serial1` at all.
- Because the enum is exhaustive and small (three values, all mapping to
  known-safe `AT+CALI` variants), a code reviewer can verify the entire
  write surface by reading `HWT3100Command`'s definition and
  `SendCommand()`'s lookup table — both fit on one screen.

## 7. File Structure

```
src/
  main.cpp                            — entry point, run_hwt3100_gateway()
  halser_const.h                       — pin assignments (reused from
                                        parent: GPIO2/3 UART, GPIO4/5 CAN,
                                        GPIO8 LED, GPIO9 button)
  hwt3100_types.h                       — HeadingReading, HWT3100Command
                                        (§3). No Arduino dependency —
                                        shared by every component below.
  hwt3100_parser.h/.cpp                 — ParseHWT3100Line(): pure ASCII
                                        line → HeadingReading parsing,
                                        no Arduino dependency, unit
                                        tested via `pio test -e native`
                                        (docs/plans/hwt3100-serial-parser.md)
  hwt3100_serial.h/.cpp                 — Serial1 owner (2.1): read task
                                        (buffers bytes into lines, calls
                                        ParseHWT3100Line()) +
                                        SendCommand() write path
                                        (HWT3100Command enum + lookup
                                        table; no AT+MODE). Split out from
                                        the parser so the parsing logic
                                        itself doesn't need a board to
                                        test — this file is the
                                        hardware-facing half, not yet
                                        implemented.
  hwt3100_calibration_commands.h/.cpp     — named calibration actions →
                                        HWT3100Command, calls
                                        SendCommand() (2.2); the only
                                        component that does
  calibration_offset.h                    — heading offset application
                                        (2.3)
  n2k_senders.h                           — PGN 127250 sender only,
                                        reused from the parent's
                                        ExpiringValue pattern (2.4)
  serial_terminal.h/.cpp                  — raw line buffering, WebSocket
                                        broadcast (2.5), read-only, no
                                        reference to SendCommand()
  gateway.h/.cpp                          — SensESP app wiring: config
                                        items, sender instantiation, main
                                        event loop (equivalent to the
                                        parent's nmea_gateway.h/.cpp)
test/
  test_hwt3100_parser/                    — Unity tests for
                                        ParseHWT3100Line(), run via the
                                        native (host, no board) PlatformIO
                                        environment
docs/
  plans/                                 — per-feature implementation
                                        plans (see
                                        IMPLEMENTATION_CHECKLIST.md)
SPEC.md
ARCHITECTURE.md
platformio.ini
```

`test_mode.h/.cpp` and the GPIO0 boot-routing logic from the parent
firmware are not carried over (SPEC §10 Design Decisions — no test-jig
requirement). The inherited NMEA 0183/gateway source from the fork
(`nmea_gateway.h/.cpp`, sentence-parser usage) gets removed once this
structure is in place — it served as pattern reference during SPEC/
ARCHITECTURE drafting but the code itself has no direct reuse (different
sensor, different input format).

## 8. Deployment

Unchanged from the parent firmware: `pio run -t upload` over USB for
initial flash, SensESP's built-in OTA for subsequent updates. Runs
standalone on the ESP32-C3 — no external services beyond WiFi + SignalK
server (both optional/independently toggleable per SPEC §7) and the N2K
bus (also optional/toggleable).

## 9. Future Considerations

- Post-MVP config (`AT+UART`, `AT+PRATE`, `AT+FILT` — SPEC §9.2) would
  extend `HWT3100Command` with more enum values mapping to more `AT+`
  commands, following the same closed-enum pattern established in 2.1 —
  the architecture doesn't need to change shape to accommodate this, and
  each addition stays auditable the same way.
- If raw magnetic field ever gets a real consumer (SPEC §5.2, §9.2), it
  flows through the same `HeadingReading`-based pipeline already carrying
  it for diagnostics — no new sensor-side plumbing needed, just a new
  sender/delta path consuming fields that already exist in the struct.
- **`AT+MODE` and Modbus mode are not future considerations** — SPEC §9.3
  treats this as a permanent, deliberate exclusion, not a deferred
  feature. If a future need for higher throughput or Modbus-specific
  functionality ever arises, treat it as reopening a closed safety
  decision requiring fresh review, not a routine roadmap item.
