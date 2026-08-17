# HALSER-HWT3100-interface Architecture

See SPEC.md for requirements and rationale; this document covers how the
firmware is built.

## 1. Overview

```
                     UART (GPIO2 TX / GPIO3 RX), 115200 baud
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
   │                   Calibration offset    Serial log ring buffer  (raw lines
   │                   applied (once)        (30 lines, config REST  also feed here)
   │                             │            API, §2.5)
   │                             ├─────────────────┐
   │                             ▼                  ▼
   │                   RateOfTurnEstimator   SignalK delta sender
   │                   (sliding window,      (navigation.headingMagnetic,
   │                   §2.4a)                navigation.rateOfTurn,
   │                             │           SensESP, WiFi)
   │             ┌───────────────┴───────┐          │
   │             ▼                       ▼          ▼
   │   N2K senders: PGN 127250    PGN 127251  SignalK server (WiFi)
   │   (heading), ExpiringValue   (rate of turn),
   │   pattern                    ExpiringValue pattern
   │             │                       │
   │             ▼                       ▼
   │        tNMEA2000_esp32 (TWAI, GPIO4/5) — identifies as
   │        B&G Precision-9 (§5, §10)
   │
   └──────────────── Calibration command handler ◄──── Web UI (config
                      (fixed AT+CALI list, §2.2)        REST toggles,
                                                         not free text)
```

Not pictured above (to keep the diagram readable): the SignalK delta
sender's `meta.timeout` (2.7) is set once at construction, not updated
per-tick like the N2K senders — SignalK staleness is advisory metadata,
not an active per-cycle signal. No RGB LED involvement anywhere in this
firmware's own code (2.9) — SensESP already owns the board's one LED.

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

On the N2K bus, this firmware presents as a **B&G Precision-9** — its
device-identity fields (product info, device function/class,
manufacturer code) are cloned from `htool/ESP32_Precision-9_compass_CMPS14`,
an open-source reference implementation for a different sensor doing the
same thing. Identity emulation is scoped to what makes the device
*recognizable*, not to fabricating data: rate of turn (PGN 127251) is
genuinely computed from heading history (2.4a), and attitude (PGN
127257, which the reference sends) is still not implemented, because
there's no honest source for it on this hardware (SPEC §1.2, §5.1, §9.3,
§10).

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
- **Write**: `SendCommand(HWT3100Command cmd)`, where `HWT3100Command`
  is a closed enum with exactly three values — `kStartCalibration`,
  `kEndCalibration`, `kClearCalibration` — mapping to `AT+CALI=1`,
  `AT+CALI=0`, `AT+CALI=2` respectively (SPEC §8.2, used only from the
  calibration command handler, 2.2). Plus `SetOutputFilter()` (2.2b),
  `SetOutputRate()`/`QueryOutputRate()` (2.2c), and `SetBaudRate()`
  (2.2d) — each a single bounded, clamped numeric write, not a raw-text
  path. There is no method on this class that accepts raw bytes or
  arbitrary text, and **no enum value for `AT+MODE` exists at all** —
  this isn't a value that's excluded from a table, it's a value that
  was never added to the enum's definition. Extending the enum to add
  it would require a deliberate code change touching this file
  directly (SPEC §1.2, §2).
- **Baud detection** (`DetectBaud()`, 2.2d): the one method on this
  class that writes nothing at all — pure passive listening at each
  candidate rate in turn, used only before `Begin()` starts the
  background read task (no concurrent access to `Serial1` from two
  places at once at that point).

### 2.2 Calibration Command Handler (`hwt3100_calibration_commands.h/.cpp`)

Receives named calibration actions from two trigger sources — the web UI
(three config toggles: Start, End, Clear) and the N2K bus, via
`MfdCalibrationBridge` (2.2a) — and maps each to an `HWT3100Command`
enum value, then calls `HWT3100SerialIO::SendCommand` (2.1). This is the
*only* component with access to `SendCommand` — the serial terminal
(2.5) is display-only and has no reference to it, and `MfdCalibrationBridge`
reaches `SendCommand` only through this handler, not directly. Holding
the write capability in one small, auditable component regardless of
how many trigger sources exist (rather than spreading "can write to the
sensor" across the codebase) keeps the allowlist enforcement easy to
verify by reading one file.

### 2.2a MFD Calibration Bridge (`mfd_calibration_bridge.h/.cpp`)

Lets a compatible MFD start/stop calibration over the N2K bus (SPEC
§8.3), matching how `htool/ESP32_Precision-9_compass_CMPS14` does it —
user-requested feature parity with the same reference project used for
the Precision-9 device identity (2.6, §5). Subclasses
`tNMEA2000::tMsgHandler` (chosen over the library's alternative
single-function-pointer `SetMsgHandler` API specifically to avoid
routing through file-scope global state to reach
`CalibrationCommandHandler`), filtered to PGN 130850 via the handler's
own constructor argument, and self-attaches to the `tNMEA2000` instance
it's constructed with.

**PGN 130850/130851 is an undocumented, reverse-engineered proprietary
Navico/Simnet message, not a published NMEA 2000 spec** — every byte
offset, the `DEVICE_ID=24` comparison value, and the acknowledgment's
exact payload come from reading the reference implementation's source,
not an official specification, and none of it has been verified against
real MFD hardware (SPEC §11). On a match (`Command1==24`,
`Command4==18`), dispatches to `CalibrationCommandHandler::StartCalibration()`/
`EndCalibration()` (2.2) — reusing the existing allowlisted chokepoint,
not a second write path — then replies with a PGN 130851 acknowledgment
built via `tN2kMsg::Init()`/`AddByte()` (fast-packet, >8 bytes; the
NMEA2000-library already handles this transparently based on message
length, the same way the parent firmware's PGN 129029 GNSS sender does).

### 2.2b Output Filter (`hwt3100_filter_command.h/.cpp`)

`FormatFilterCommand()` (SPEC §8.2a) — pure, unit-tested function that
clamps an integer to `[0, 999]` and formats `"AT+FILT=<n>\r\n"`, used by
`HWT3100SerialIO::SetOutputFilter()` (2.1, §6). Config wiring lives in
`gateway.cpp` (2.6): a single persisted `PersistingObservableValue<int>`
sent to the module once at boot and again on every config change —
unlike 2.2's one-shot calibration triggers, this isn't fire-and-reset,
since the module can't report its current filter setting back and a
reboot would otherwise silently drop it to `0`.

### 2.2c Output Rate (`hwt3100_prate_command.h/.cpp`)

`FormatPrateCommand()`/`ParsePrateReply()` (SPEC §8.2b) — pure,
unit-tested functions used by `HWT3100SerialIO::SetOutputRate()`/
`QueryOutputRate()` (2.1, §6). Config wiring in `gateway.cpp` (2.6)
follows 2.2b's apply-at-boot-and-on-change pattern, with one addition:
the persisted default is `halser::kPrateUnknown` (`-1`), not a real
rate, since forcing an assumed default the way 2.2b does could
silently overwrite whatever the module was already configured with —
and if wrong in the `0` (single-return) direction, would silence the
module's stream entirely. At boot, if the value is still
`kPrateUnknown`, `gateway.cpp` sends the query instead of a set
command; a second consumer attached to `raw_line_producer` (the same
producer the serial terminal, 2.5, reads from) tries
`ParsePrateReply()` on every incoming line and, while still
`kPrateUnknown`, treats a match as the module's answer — persisting it
via `output_prate->set()`, which in turn re-applies it through the
same `connect_to()` used for config-UI-driven changes (a harmless echo
of what the module just reported).

### 2.2d UART Baud Rate (`hwt3100_uart_command.h/.cpp`)

`FormatUartCommand()` (SPEC §8.2c) — pure, unit-tested function that
snaps a requested baud to the nearest of `{9600, 115200, 460800}`
(ties toward the lower rate) and formats the matching `AT+UART=<n>`
index, used by `HWT3100SerialIO::SetBaudRate()`. Unlike 2.2b/2.2c, this
config item does double duty: `gateway.cpp`'s persisted
`hwt3100_baud` represents both "what baud are we currently on" and
"what baud do you want," same unknown-sentinel-then-learn pattern as
2.2c (`halser::kBaudUnknown = -1`), but discovery here is
`HWT3100SerialIO::DetectBaud()` (2.1) rather than a query/reply — a
synchronous, blocking scan through candidate rates (115200, 9600,
460800 in order) run once at boot, *before* `Begin()` starts the
background read task, since it isn't safe to call concurrently with
it. The first candidate that produces at least one line parsing via
`ParseHWT3100Line()` is adopted and persisted; detection sends no
`AT+UART` — it's passive listening, discovering what the module
already is, not changing it.

Once a baud is known (learned or explicitly set), changing
`hwt3100_baud` through the config UI triggers `SetBaudRate()`: send
`AT+UART=<n>` at the *current* rate, a fixed settle delay, then
reconfigure `Serial1` to the new rate — same `connect_to()`-after-
`set()` ordering trick as 2.2c uses to avoid a discovery-time `set()`
looping back into an unwanted write (SPEC §8.2c documents the
`connect_to()` wiring is attached *after* any startup-detection
`set()`, so the initial discovery never itself fires a command).

**Known limitation, documented rather than solved**: `SetBaudRate()`
reconfigures `Serial1` from the main-loop thread while
`HWT3100SerialIO`'s background read task may concurrently be calling
`serial_.available()`/`read()` on the same object. This is a narrow,
rare race (only during an explicit, infrequent user-triggered baud
change, not during normal operation) accepted rather than solved with
additional synchronization, consistent with this codebase's practice
of flagging real limitations rather than adding untested complexity to
paper over them (SPEC §11).

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

### 2.4 N2K Senders (`n2k_senders.h`)

- `N2kHeadingSender` — PGN 127250, adapted from the parent firmware's
  version. Uses the `ExpiringValue<T>` pattern, which already does
  exactly what SPEC §6 (stale-data behavior) asks: `to_n2k()` returns
  `N2kDoubleNA` once a value goes stale, satisfying the "transmit N2K
  not-available values" decision for free.
- `N2kRateOfTurnSender` — PGN 127251, same `ExpiringValue<T>` pattern,
  fed by `RateOfTurnEstimator` (2.4a) rather than directly by the
  HWT3100 — the hardware has no gyroscope (SPEC §1.3, §5.1, §10).
  "Not available" covers both staleness and "not enough heading history
  yet" identically, since both mean `ExpiringValue::update()` hasn't
  been called recently.

No PGN 127257 (Attitude) sender exists, still — the hardware can't
supply pitch/roll, and unlike rate of turn there's no honest way to
derive it from data this hardware has (SPEC §5.1, §9.3, §10).

Each sender has its own enable/disable flag (config, 2.6) checked before
`send()` is called from the periodic send loop, independent of the
`ExpiringValue` staleness mechanism — "disabled" and "stale" are
different states (disabled = never sends; stale = sends N/A values).

Both PGNs are also declared to `tNMEA2000::ExtendTransmitMessages()`
(a file-scope `const unsigned long[]`, `{127250L, 127251L, 0}`,
0-terminated per the library's convention — must outlive the call
since the library stores the pointer, not a copy) right after
`SetMode()`/before `Open()` in `gateway.cpp`'s N2K setup. Without this,
the library only reports its own automatic boilerplate PGNs (address
claim, heartbeat, product/config info) in response to PGN 126464 ("PGN
List — Transmit") queries — a real gap that went unnoticed until
directly observed against real hardware/tooling, not something caught
by the native unit tests (this is device-setup wiring, not pure logic,
so it was never in scope for them).

### 2.4a Rate of Turn Estimator (`rate_of_turn.h/.cpp`, class `RateOfTurnEstimator`)

Pure logic, no Arduino dependency (same split rationale as the HWT3100
parser, 2.1) — unit tested on the host via `pio test -e native`. Holds a
fixed-size ring buffer of recent (heading, timestamp) samples and, on
request, fits a **least-squares line through every sample within a
trailing time window** and returns its slope, as radians/second —
`GetRateOfTurn()` walks the window from oldest to newest, accumulating
`Σt, Σh, Σt², Σth` and solving the standard two-variable normal
equations, rather than differencing just the two endpoint samples.

This replaced an earlier two-point-difference version after research
into how others compute rate of turn (marine gyrocompass patents,
pypilot's own heading-rate derivation, general numerical-differentiation
literature) converged on the same point: finite differences amplify
noise, and a windowed regression is the standard fix, using data the
ring buffer was already storing but the two-point version discarded.
The two are mathematically identical whenever exactly two samples fall
in the window — a line through two points has only one possible slope —
so every existing two-sample unit test kept its expected value
unchanged; new tests cover the ≥3-sample case specifically (a noisy
interior sample measurably pulling the fitted slope away from what a
pure endpoint difference would give, and multi-sample 0/360 wraparound).

Heading is unwrapped once per step across the *entire* included range
(not just at the two endpoints) before fitting, using the same shortest-
signed-angular-delta trick as before — each successive sample's raw
heading is compared to the previous one, and the (possibly several)
individual unwrap steps accumulate into a continuous value the linear
fit can safely operate on.

Returns "not available" (no value) when there isn't yet at least a
minimum time span of history within the window (even if ≥2 samples exist
in the buffer overall) — see SPEC §11 for why the window (2000ms) and
minimum span (500ms) are flagged as needing real-hardware tuning rather
than being load-bearing constants.

Fed from the same `LambdaConsumer` in `gateway.cpp` (2.6) that applies
the calibration offset and updates the heading sender — every corrected
`HeadingReading` becomes one sample. This keeps rate-of-turn computation
downstream of calibration (so a heading offset change doesn't look like
an instantaneous "turn" to the estimator) without adding a second
observer of the raw `TaskQueueProducer`.

### 2.5 Serial Terminal (`serial_terminal.h/.cpp`, class `SerialTerminal`)

**Implementation-time correction**: this is no longer a WebSocket
broadcaster. Reading SensESP 3.2.0's actual vendored source turned up two
things ARCHITECTURE.md had gotten wrong from the outside: (1) SensESP's
`HTTPServer` wraps ESP-IDF's native `esp_http_server`, not
`ESPAsyncWebServer`/`AsyncWebSocket`; (2) more fundamentally, there is no
public way to add a custom HTTP handler at all — `SensESPApp`'s
`http_server_` is `protected` with no accessor anywhere in the public
API, and every `add_handler()` call in the library happens internally
during the framework's own `build()`. See
docs/plans/gateway-wiring.md for the full finding.

Resolution: `SerialTerminal` is a fixed-size ring buffer (last 30 raw
lines) exposed through SensESP's existing, already-public **config REST
API** — it implements `Saveable`+`Serializable` directly (not
`FileSystemSaveable`, so `load()`/`save()`/`clear()` stay no-ops and
nothing here ever hits flash) and is registered via `ConfigItem()` like
any other config value. `to_json()` serializes the buffer; `from_json()`
is left at `Serializable`'s default (`return false`), making writes a
no-op — a read-only view. `AddLine()` is called from a plain
`LambdaConsumer` set up in gateway.cpp (2.6), not by this class reading
off a `TaskQueueProducer` itself, keeping it a plain buffer with no
FreeRTOS/event-loop concerns of its own. Display-only, per §2.1/§2.2 —
this component never gets a reference to `SendCommand`.

The trade-off: this reads as config fields in SensESP's auto-generated
config UI, not a dedicated live-scrolling terminal widget. Accepted per
the user decision recorded in docs/plans/gateway-wiring.md.

### 2.6 Configuration / Web UI Wiring (`gateway.cpp`)

Uses SensESP's `ConfigItem` + `PersistingObservableValue` pattern (as the
parent firmware does for NMEA 0183 bit rate) for every SPEC §7 config
value: N2K master enable, PGN 127250 enable, SignalK enable, calibration
offset, WiFi/SignalK connection (SensESP's own built-in config UI).

Calibration *commands* (§8.2/2.2) use the same mechanism, reframed as
one-shot triggers: each is a `PersistingObservableValue<bool>` that,
when set `true` (via the config UI), fires the corresponding
`CalibrationCommandHandler` method and then immediately resets itself to
`false` so it can be triggered again. Trade-off, accepted: if the device
loses power between the `true` write and the `false` reset, the next
boot's initial config-load emit could replay the command — acceptable
because HWT3100 calibration commands are safe to resend, unlike
`AT+MODE` (SPEC §1.2, §2). `gateway.cpp` constructs and owns the two
`TaskQueueProducer`s (2.1) and the `SerialTerminal`/calibration-trigger
config items; the actual command dispatch stays 2.2's responsibility, not
this component's.

`MfdCalibrationBridge` (2.2a) is a second trigger source for the same
2.2 handler, constructed once after it — no config item of its own
(SPEC §8.3), since it's triggered by N2K bus traffic, not a web UI
action.

### 2.7 SignalK Delta Sender

Publishes `navigation.headingMagnetic`, `navigation.rateOfTurn`, and
(if `raw_mag_field_enabled`) `sensors.hwt3100.magneticField.x/y/z` via
SensESP's existing SignalK/WiFi transport, gated by the SignalK master
enable flag (2.6). Uses the same corrected `HeadingReading` as the N2K
sender (2.4) — single source of truth, per SPEC §2. Heading/rate-of-
turn are in radians (rad and rad/s respectively), converted from the
firmware's internal degrees representation right at this boundary
(SPEC §3, §5.2 — an earlier version of the heading sender sent raw
degrees, a real bug caught while wiring up the `meta.timeout` units
field below). `navigation.rateOfTurn` is only set when
`RateOfTurnEstimator` actually has a value (SPEC §5.1) — same gate as
the N2K PGN 127251 sender, same sign convention (positive = starboard).

The raw magnetic field outputs are the one exception to "converted at
the output boundary": `mag_x/y/z` are published as-is, the module's
raw sensor counts, with no unit in their `SKMetadata` (the manual
doesn't document a counts-to-µT conversion factor) and no calibration
offset applied (§2.3 only corrects `heading`, per SPEC §3's note that
the raw field is diagnostic-only). Gated behind its own toggle in
addition to the SignalK master flag (SPEC §5.2) — off by default,
unlike heading/rate-of-turn.

All outputs are constructed with an `SKMetadata` carrying the
appropriate `units` (`"rad"` / `"rad/s"` / none) and `timeout_=5.0`
(matching the N2K senders' own `ExpiringValue` window) — this
`timeout_` field is SPEC §6's entire staleness mechanism on the
SignalK side. No separate fault-indication component exists: SensESP's
`SKMetadata` already has a first-class `timeout_` field, so this is
one constructor argument, not new code. (An earlier version of this
architecture had a dedicated `SKNotification`/`SKEmitter` subclass
sending an active `notifications.*` alarm; replaced once `meta.timeout`
was confirmed as the SignalK-spec-defined mechanism for exactly this —
SPEC §10 covers the trade-off.)

### 2.9 No Dedicated Fault LED

`gateway.cpp` does not drive the RGB LED. SensESP auto-instantiates its
own `RGBSystemStatusLed` on GPIO8 from the `PIN_RGB_LED` build flag
(`sensesp_app.h`), writing WiFi/WebSocket connection-state colors
unconditionally every 5ms via its own internal timer — with no exposed
pause/override hook. This was already true before this component was
added (a latent conflict from the gateway-wiring change, where
`gateway.cpp` separately ran its own `Adafruit_NeoPixel` on the same
physical pin); adding fault-indication color to that fight rather than
resolving it wasn't an option. See §6 and SPEC §10 for the decision to
drop the firmware's own LED use entirely rather than remove
`PIN_RGB_LED` and lose SensESP's connection-status display.

## 3. Data Models

See SPEC.md §3 for the conceptual model. In code:

```cpp
struct HeadingReading {
  float heading = 0.0f;   // degrees, 0-360, magnetic, offset-corrected
  int32_t mag_x = 0;      // raw magnetic field X, diagnostic use only
  int32_t mag_y = 0;      // raw magnetic field Y, diagnostic use only
  int32_t mag_z = 0;      // raw magnetic field Z, diagnostic use only
  unsigned long timestamp = 0;  // millis() of last successful sensor read
};

enum class HWT3100Command {
  kStartCalibration,   // AT+CALI=1
  kEndCalibration,     // AT+CALI=0
  kClearCalibration,   // AT+CALI=2
};

struct HWT3100RawLine {  // raw serial line for the terminal (§2.5)
  static constexpr size_t kMaxLength = 128;
  char text[kMaxLength] = {0};
};
```

No separate `CalibrationOffset` struct exists in code — the offset is a
single `float`, held directly by a `PersistingObservableValue<float>` in
`gateway.cpp` (2.6); a one-field wrapper struct would have added
indirection without buying anything.

No pitch/roll fields exist, deliberately (SPEC §3, §9.3) — the HWT3100
cannot produce them. `HWT3100Command` has exactly three values; there is
no value for `AT+MODE` (SPEC §1.2, §2).

## 4. Technology Stack

Same as the parent firmware (see its AGENTS.md), plus one addition:

| Layer | Choice | Why |
|---|---|---|
| Framework | Arduino (ESP32-C3), SensESP 3.2.0 | Reused for consistency with the HALSER family; gets WiFi, web UI, SignalK client, OTA for free. |
| N2K | ttlappalainen/NMEA2000-library + NMEA2000_twai | Same as parent; adapts the parent's `N2kHeadingSender`/`SetN2kPGN127250` and adds `SetN2kPGN127251` for rate of turn. |
| N2K device identity | Cloned from `htool/ESP32_Precision-9_compass_CMPS14` | Presents as a B&G Precision-9 rather than a generic device — SPEC §1.2, §5.1, §10. Product info + device function/class + manufacturer code cloned verbatim; the "unique number" is deliberately not, to avoid N2K address-claim collisions between multiple installs. |
| Serial parsing | Custom (this project) | Simple line-based ASCII parsing (`Magx=<n>,y=<n>,z=<n>,w=<n.n>\r\n`) — no library needed, comparable effort to the parent's NMEA 0183 sentence parsers but simpler (no checksum). |
| Rate of turn | Custom (this project) | No library needed — a sliding-window derivative of heading readings is a small, self-contained computation (2.4a); no existing library targets "derive ROT from a compass with no gyroscope." |
| Serial log transport | SensESP's existing config REST API (no new dependency) | SensESP's `HTTPServer` wraps ESP-IDF's native `esp_http_server`, not `ESPAsyncWebServer` as first assumed, and exposes no public hook for custom HTTP/WebSocket endpoints at all. Reusing the already-public config GET/PUT mechanism (§2.5) needed nothing new; a dedicated WebSocket endpoint would have needed a second, hand-rolled `esp_http_server` instance. |
| RGB LED | Not used by this firmware's own code | SensESP auto-instantiates its own `RGBSystemStatusLed` on GPIO8 (from the `PIN_RGB_LED` build flag) with no exposed pause/share hook. `gateway.cpp` briefly ran a second, conflicting `Adafruit_NeoPixel` driver on the same pin (from the gateway-wiring change); removed once the conflict was found (§2.9, SPEC §10). |

## 5. Integration Points

- **HWT3100-TTL module** (TTL variant only — see SPEC §1.2) — UART1,
  `Serial1`, GPIO2 TX / GPIO3 RX, **115200 baud**, NOT the module's
  factory-default 9600 (`kHWT3100DefaultBaud`, halser_const.h). The
  module must be reconfigured to 115200 via `AT+UART=1` *before* wiring
  to HALSER (SPEC §1.2) — this firmware can't perform that
  reconfiguration itself, since it can't talk to the module until the
  module is already at the rate the firmware expects. Wired to HALSER's
  UART terminal block with the RX-select jumper on "U" (confirmed
  against Hat Labs' HALSER hardware docs — the board has one shared
  UART peripheral muxed by that jumper
  across NMEA0183/RS-232/UART connectors). Protocol confirmed from the
  vendor manual + SDK: ASCII mode, comma-delimited text lines for data,
  `AT+`-prefixed text commands for calibration (SPEC §1.2). Exact parsing
  edge cases remain an open question for implementation-time hardware
  testing (SPEC §11).
- **NMEA 2000 bus** — via `tNMEA2000_esp32`, GPIO4 TX / GPIO5 RX (TWAI),
  same transport as the parent firmware; device identity presented on
  the bus is the B&G Precision-9 clone (§1, §4), not a HALSER identity.
- **SignalK server** — via SensESP's WiFi + mDNS discovery + WebSocket
  delta client, unchanged from the parent firmware's mechanism.
- **Browser (web UI)** — SensESP's own config UI only. The serial log
  (2.5) and calibration commands (2.2, 2.6) both ride the same existing
  config REST API rather than a separate endpoint (see §4).

## 6. Security Considerations

Same trust model as the parent firmware: a device on a private boat LAN,
where physical/network access already implies a high level of trust.
SensESP's existing defaults (WiFi credentials in NVS, whatever auth
SensESP's config UI provides out of the box) are sufficient — no
additional auth/encryption work scoped for this project (per SPEC's user
decision).

The hardware-specific rule that *is* a hard requirement, not a
convention: **the firmware must never send `AT+MODE`, `AT+MRATE`, or
`AT+ID` to the HWT3100** (SPEC §1.2, §2, §9.3) — `AT+MODE=1` has
confirmed-bricked a real unit; `AT+MRATE`/`AT+ID` are excluded
precautionarily (unconfirmed risk) and because both are meaningless
without Modbus mode anyway. Unlike an earlier draft of this design
(which treated `AT+MODE` as "writes are allowed, but filtered"), the
actual enforcement here is stronger: the firmware simply has no
functional need for any of these three, so nothing implements them.

- `Serial1` is owned exclusively by `HWT3100SerialIO` (2.1); no other
  component ever gets a reference to it.
- `HWT3100SerialIO` exposes exactly four write methods, and they are
  the *only* code in this firmware that writes to `Serial1`:
  - `SendCommand()` takes a closed `HWT3100Command` enum with exactly
    three values (2.1, §3) — `kStartCalibration`, `kEndCalibration`,
    `kClearCalibration`. No overload, no debug backdoor, no "advanced
    mode" that accepts raw text, and **no enum value that maps to
    `AT+MODE`, `AT+MRATE`, or `AT+ID` at all** — not "filtered out,"
    simply never defined.
  - `SetOutputFilter(int)` (§2.2b) constructs `"AT+FILT=<n>\r\n"` with
    `n` clamped to `[0, 999]` by `FormatFilterCommand()` before
    anything reaches the wire (SPEC §8.2a).
  - `SetOutputRate(int)` (§2.2c) constructs `"AT+PRATE=<n>\r\n"` with
    `n` clamped by `FormatPrateCommand()` to `{0} u [10, 10000]` (SPEC
    §8.2b).
  - `QueryOutputRate()` (§2.2c) sends the single fixed string
    `"AT+PRATE=?\r\n"` — no parameter at all.
  All four are bounded, closed-domain writes, not a raw-text backdoor.
- The calibration command handler (2.2) and the output-filter/rate
  config wiring (2.2b, 2.2c, gateway.cpp) are the only components
  holding a reference to these write methods; the serial terminal (2.5)
  and everything else remain read-only, with no path to `Serial1` at
  all.
- Because the enum and both numeric commands' valid ranges are small,
  closed, and clamped, a code reviewer can verify the entire write
  surface by reading `HWT3100Command`'s definition, `SendCommand()`'s
  lookup table, `FormatFilterCommand()`, and `FormatPrateCommand()` —
  all fit on one screen combined.

## 7. File Structure

```
src/
  main.cpp                            — entry point, run_hwt3100_gateway()
  halser_const.h                       — pin assignments (reused from
                                        parent: GPIO2/3 UART, GPIO4/5 CAN,
                                        GPIO9 button — no GPIO8 LED
                                        constant, see §2.9) + N2K device
                                        identity constants cloned from the
                                        Precision-9 reference (§1, §4)
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
                                        table; no AT+MODE). The
                                        hardware-facing half, split out
                                        from the parser so the parsing
                                        logic itself doesn't need a board
                                        to test.
  hwt3100_calibration_commands.h          — CalibrationCommandHandler
                                        (2.2): named calibration actions
                                        → SendCommand(); the only
                                        component that calls it. Small
                                        enough to stay header-only (each
                                        method is one call through).
  mfd_calibration_bridge.h/.cpp           — MfdCalibrationBridge (2.2a):
                                        N2K PGN 130850/130851 handler,
                                        reverse-engineered proprietary
                                        protocol (docs/plans/mfd-calibration.md),
                                        dispatches through 2.2, not a
                                        second write path
  calibration_offset.h                    — heading offset application
                                        (2.3), a pure function
  rate_of_turn.h/.cpp                     — RateOfTurnEstimator (2.4a):
                                        pure sliding-window ROT
                                        computation, no Arduino
                                        dependency, unit tested via
                                        `pio test -e native`
                                        (docs/plans/precision9-rate-of-turn.md)
  n2k_senders.h                           — ExpiringValue<T> +
                                        N2kHeadingSender (PGN 127250) +
                                        N2kRateOfTurnSender (PGN 127251),
                                        adapted from the parent's
                                        pattern (2.4)
  serial_terminal.h/.cpp                  — SerialTerminal (2.5): a
                                        30-line ring buffer exposed via
                                        SensESP's config REST API, not a
                                        WebSocket (see §4 for why)
  gateway.h/.cpp                          — SensESP app wiring: config
                                        items, sender instantiation, main
                                        event loop (equivalent to the
                                        parent's nmea_gateway.h/.cpp)
test/
  test_hwt3100_parser/                    — Unity tests for
                                        ParseHWT3100Line(), run via the
                                        native (host, no board) PlatformIO
                                        environment
  test_rate_of_turn/                      — Unity tests for
                                        RateOfTurnEstimator, same native
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

- `AT+UART` (2.2d), raw magnetic field as a SignalK output (2.7), and
  baud auto-detection (2.1, 2.2d) are all implemented as of this
  version — SPEC §9.2's post-MVP list is now empty. Any further AT
  command exposed as config (`AT+PRATE`'s sibling `AT+ID`/`AT+MRATE`
  are permanently excluded, not deferred — see below) would follow the
  same pattern established for `AT+FILT`/`AT+PRATE`/`AT+UART` (2.2b,
  2.2c, 2.2d): a small pure format/parse function plus one or two
  dedicated `HWT3100SerialIO` methods.
- **`AT+MODE` and Modbus mode are not future considerations** — SPEC §9.3
  treats this as a permanent, deliberate exclusion, not a deferred
  feature. If a future need for higher throughput or Modbus-specific
  functionality ever arises, treat it as reopening a closed safety
  decision requiring fresh review, not a routine roadmap item.
- `RateOfTurnEstimator`'s window (2000ms) and minimum span (500ms) are
  compile-time constants in `gateway.cpp`, not config items — SPEC §11
  flags them as likely needing real-hardware tuning. If that tuning
  turns out to need per-installation adjustment (rather than a one-time
  firmware-wide constant change), exposing them as config values follows
  the same `PersistingObservableValue` pattern already used everywhere
  else in `gateway.cpp` — no architectural change needed, just wiring.
- If a future need for LED fault indication becomes strong enough to
  revisit (SPEC §10 accepted losing it as the cost of keeping SensESP's
  connection-status LED), the two real options are removing
  `PIN_RGB_LED` to take back the pin entirely, or patching/forking
  SensESP to add a pause hook to `BaseSystemStatusLed` — both are
  bigger changes than this project's scope, not something to reach for
  casually.
