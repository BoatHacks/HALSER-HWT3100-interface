# HALSER-HWT3100-interface Specification

## 1. Introduction

### 1.1 Purpose

HALSER-HWT3100-interface is ESP32-C3 firmware that reads magnetic heading
data from a WitMotion HWT3100-TTL fluxgate electronic compass module over
a serial (UART) link, and makes that data available to a boat's instrument
network in two independent ways:

- as **NMEA 2000 (N2K)** messages on the vessel's CAN bus, for chartplotters
  and other N2K instruments, and
- as **SignalK deltas**, sent over WiFi to a SignalK server, for
  chartplotter apps (e.g. OpenCPN), dashboards, and logging.

Either output can be enabled or disabled independently — the firmware isn't
built around one being "the real product" and the other a bonus.

On the N2K side, the firmware presents itself as a **B&G Precision-9**
compass (device-identity fields, PGN set) rather than as a generic/custom
device — see §1.2, §5.1, §10 for what that means concretely and why.

It is part of the HALSER firmware family and is forked from
`HALSER-default-firmware` (an NMEA 0183→N2K gateway), which is the
reference for coding patterns (SensESP setup, N2K sender/value-expiry
patterns, project layout) but is otherwise a different product: this
project's input is a single compass sensor over serial, not an NMEA 0183
feed, and it carries no production test-jig requirement.

### 1.2 Background

The WitMotion HWT3100-TTL/232 is a **fluxgate 3-axis electronic compass**,
not a full IMU/AHRS — per its manual (§5.4.3 register table), it outputs
only raw magnetic field (X/Y/Z) and a derived magnetic heading angle
(`YAW`). **It has no accelerometer or gyroscope, and does not output
pitch or roll.** An earlier draft of this document assumed a full
attitude sensor (heading/pitch/roll); that assumption was wrong for this
specific product and has been corrected throughout.

The module has three communication modes: **ASCII** (the power-on
default), **Modbus**, and **Modbus active-output**. Verified against the
manual and the vendor's own Arduino SDK example (`wit_c_sdk.c`):

- In ASCII mode, the module continuously streams plain-text lines of the
  form `Magx=<int>,y=<int>,z=<int>,w=<decimal>\r\n` — `x`/`y`/`z` are raw
  magnetic field readings, `w` is heading in degrees (one decimal place).
  This is confirmed by the vendor SDK's ASCII line parser, which looks
  for exactly this comma-delimited format. **This alone is sufficient for
  every data requirement this firmware has** — no Modbus needed.
- ASCII mode also accepts a documented set of `AT+`-prefixed text
  commands (§5.3.1 of the manual) for calibration and configuration —
  see §2 and §8.2 — without leaving ASCII mode.
- Default baud rate is **9600 bps**, configurable to 115200 or 460800 via
  `AT+UART`.

> **⚠️ Model restriction:** this firmware works with the **HWT3100-TTL**
> variant only — not HWT3100-232 (RS-232) or HWT3100-485 (RS-485). Those
> variants use different physical-layer signaling that HALSER's UART
> terminal block doesn't support (SPEC.md §4, and the wiring guidance
> that follows this note).

> **⚠️ Recommended pre-wiring setup:** the module ships at 9600 baud;
> **115200 is recommended** for headroom with `AT+FILT`/`AT+PRATE`
> traffic. As of §8.2c, the firmware auto-detects whichever baud the
> module is actually on at boot (trying 115200 first, then 9600, then
> 460800) and can switch it live via config — so pre-configuring
> `AT+UART=1` on a PC/USB-TTL adapter before wiring to HALSER is no
> longer *required*, just a way to skip the extra detection latency on
> first boot (§8.2c covers the actual timing cost).

> **⚠️ Hardware hazard:** sending `AT+MODE=1` switches the module out of
> ASCII mode into Modbus mode. Per a firsthand report, this has bricked a
> real unit. **This firmware must never send `AT+MODE` (in either
> direction) under any circumstance.** Unlike the earlier draft of this
> document, this is no longer "avoid it via an allowlist while doing
> something else that needs it" — the firmware has **no functional need
> for Modbus mode at all**: ASCII mode's default streaming output already
> provides 100% of the data this project needs, and ASCII mode's AT
> commands already provide 100% of the calibration/config this project
> needs. `AT+MODE` is simply never implemented anywhere in this
> codebase — not gated, not allowlisted-out, just absent. See §2 and §8.

> **⚠️ Precautionary exclusion:** `AT+MRATE` and `AT+ID` are also never
> implemented — **not** because either is confirmed to brick a unit like
> `AT+MODE`, but because it's currently *unknown* whether they carry the
> same risk, and this firmware has no functional need for either one to
> take that chance. (`AT+ID` sets the module's Modbus address — an
> address that's meaningless without also using `AT+MODE` to enter
> Modbus mode, which is already permanently excluded above. `AT+MRATE`
> configures Modbus active-output timing — same dependency on Modbus
> mode.) If a real functional need for either ever arises, that requires
> new hardware-verified research first, the same bar `AT+MODE` would
> need to clear — not just adding a case to a switch statement.

All N2K PGNs applicable to the data this firmware actually produces
(heading and computed rate of turn — see §3) are implemented and
individually selectable in the config UI — see §5.1 and §7.

**B&G Precision-9 emulation.** On the N2K bus, this firmware identifies
itself as a B&G Precision-9 compass — same product info (model ID,
software/model version, manufacturer's model serial code, product code),
same NMEA 2000 device function/class, same manufacturer code — rather
than as a generic/custom device. This is a deliberate choice, not an
accident of copying example code: some MFDs/autopilots key certain
features off recognized product identities, and presenting as a known,
already-supported compass is more likely to "just work" with them than a
novel device announcing itself honestly as a HALSER prototype. The
reference implementation, `htool/ESP32_Precision-9_compass_CMPS14`
(an open-source ESP32 project doing the same thing for a CMPS14
9-DOF sensor), is the source for these exact field values — see §10 for
which fields are cloned verbatim vs. which are deliberately not (the
"unique number" field, which isn't part of what makes a device
*recognizable* as a Precision-9 to anything listening on the bus).

Emulating the identity does **not** mean emulating data this hardware
doesn't have: PGN 127257 (Attitude) is still not implemented (§9.3) since
the HWT3100 has no accelerometer to derive heel/trim from, honestly or
otherwise. PGN 127251 (Rate of Turn) *is* newly implemented, but as a
genuinely computed value (§1.3, §3, §5.1) — a sliding-window derivative
of real heading readings — not a copied fingerprint with no data behind
it.

### 1.3 Terminology

- **Heading** — magnetic heading angle, as reported by the HWT3100's
  `w` field / `YAW` register.
- **Magnetic field reading** — the raw 3-axis magnetic field vector
  (`x`/`y`/`z`) the HWT3100 measures and derives heading from. Exposed for
  diagnostics/calibration verification; not itself a vessel instrument
  reading with an established N2K/SignalK representation (see §5, §10
  Design Decisions).
- **HWT3100** — shorthand for the WitMotion HWT3100-TTL/232 module, the
  firmware's sole sensor input.
- **Calibration offset** — a user-configurable correction applied to raw
  HWT3100 heading readings, to compensate for the module not being
  mounted perfectly aligned with the vessel's fore-aft axis.
- **On-module calibration** — the HWT3100's own magnetic-field
  calibration procedure (rotate the module through a full circle while
  the sensor learns its magnetic environment), triggered via `AT+CALI`
  commands (§8.2). Distinct from the offset above: on-module calibration
  corrects the sensor's own magnetic measurement; the offset corrects for
  mounting misalignment on top of that.
- **Stale** — sensor data that has aged past its expected update interval
  without a fresh reading, and is no longer trustworthy.
- **Rate of turn (computed)** — the rate the vessel's heading is
  changing, in radians/second, positive = turning to starboard. Not
  sensed directly (the HWT3100 has no gyroscope) — derived by comparing
  the oldest and newest heading readings within a trailing time window
  (§3, §10). "Computed" is part of the name deliberately, to keep this
  distinct from a real gyroscope-sensed rate of turn wherever this spec
  discusses it.

## 2. Domain Rules

- **The firmware must never send `AT+MODE` (any value) to the HWT3100,
  under any circumstance.** `AT+MODE=1` has bricked a real unit (§1.2).
  The firmware has no functional need for Modbus mode, so this isn't an
  allowlist exclusion applied to an otherwise-general write path — it's
  simply a command this codebase never implements, anywhere, for any
  reason. See §8 for how the calibration-command write path is scoped to
  make this structural.
- Heading is referenced to magnetic north as read by the HWT3100 (the
  module does not do true-heading correction; any magnetic variation
  correction, if needed, happens downstream in SignalK/N2K consumers).
- Heading is relative to the module's mounted orientation, adjusted by
  the configured calibration offset (§1.3) before transmission.
- The calibration offset is applied in firmware, once, before data is
  handed to either the N2K or SignalK output path — both outputs must
  always see the same, already-corrected value.

## 3. Data Model

### HeadingReading (MVP)

| Field       | Type  | Notes                                              |
|-------------|-------|-----------------------------------------------------|
| heading     | float | Magnetic heading, **degrees**, 0–360, offset-corrected — this is the internal representation throughout the firmware's pipeline (matches the HWT3100's own wire format and the rate-of-turn math, §3). N2K and SignalK both require radians (§5.1, §5.2); that conversion happens once, at each output's boundary, not here. |
| magX, magY, magZ | int | Raw magnetic field components, as reported by the module (diagnostic use — see §5, §10) |
| timestamp   | uint  | Millis of last successful sensor read (for staleness) |

### CalibrationOffset (config, persisted)

| Field         | Type  | Default |
|---------------|-------|---------|
| headingOffset | float | 0.0     |

No pitch/roll/gyro/accel/quaternion fields exist because the HWT3100
doesn't produce them (§1.2) — this isn't a deferred scope decision, it's
a hardware capability limit. If a future project needs full attitude
(pitch/roll), it needs a different sensor; extending this data model
won't get there.

### Rate of turn (computed, not persisted)

Not a field on `HeadingReading` — it's derived from a trailing window of
`HeadingReading.heading` values (§1.3), each already timestamped for
this purpose. Conceptually:

| Field | Type | Notes |
|---|---|---|
| rateOfTurn | float, radians/second | least-squares slope of heading vs. time over every sample in the window (wraparound-unwrapped), positive = turning to starboard — see §10 for why a windowed fit rather than a two-point difference |

Requires at least two samples spanning a minimum elapsed time within the
window to produce a value at all (§10) — below that, it's "not
available," same as any other not-yet-valid reading (§6).

## 4. Sources / Inputs

- **Single source**: the HWT3100-TTL module, connected via UART to the
  ESP32-C3, GPIO2 TX / GPIO3 RX (HALSER's UART terminal block, RX-select
  jumper on "U" — confirmed against Hat Labs' HALSER hardware docs; see
  ARCHITECTURE.md §5). 115200 bps, requiring the module to be
  pre-configured via `AT+UART=1` before wiring (§1.2) — not the
  module's factory-default 9600.
- There is no secondary/fallback sensor. If the HWT3100 stops sending
  data, the firmware does not substitute another source — it marks the
  data stale (see §6).

## 5. API Specification

### 5.1 N2K Output

This firmware presents itself on the N2K bus as a **B&G Precision-9
compass**, cloning the device-identity fields (product info, device
function/class, manufacturer code) from a known working open-source
implementation (`htool/ESP32_Precision-9_compass_CMPS14`) — see §1.2 for
why, and §10 for exactly which fields are cloned vs. derived locally.

Two PGNs are implemented:

- **PGN 127250 — Vessel Heading**: real data. Radians, per
  `SetN2kPGN127250`'s documented parameter — converted from the
  firmware's internal degrees representation (§3) at this output
  boundary, same as SignalK's (§5.2).
- **PGN 127251 — Rate of Turn**: *computed*, not sensed — the HWT3100
  has no gyroscope, so this is derived from a sliding window of recent
  heading readings (§1.3, §3). This is genuinely-derived data (the
  actual rate the heading has been changing), not a fabricated/NA
  placeholder sent only to pad out the PGN fingerprint. Already in
  radians/second natively — no conversion needed at this boundary.

**PGN 127257 (Attitude) remains not implemented** — that decision doesn't
change here. The Precision-9 reference implementation sends it (it reads
heel/trim from a real accelerometer-equipped IMU), but the HWT3100 has no
accelerometer; there is no honest way to derive attitude the way rate of
turn can be derived from heading history. Emulating the Precision-9's
*identity* doesn't require emulating data this hardware can't produce
(§9.3 still applies).

Both PGNs are independently selectable in the config UI on top of the
master N2K enable/disable switch (§7).

Requirement: transmit at a reasonable update rate for helm/autopilot
displays, and transmit N2K "not available" values when stale (§6) —
this applies to both PGNs, including when there isn't yet enough heading
history to compute a rate of turn. Exact transmission rate and field
encoding are worked out in ARCHITECTURE.md.

### 5.2 SignalK Output

Heading delta published as `navigation.headingMagnetic`, in **radians**
per the SignalK spec (all SignalK angles are SI units — degrees is not
valid here, unlike the firmware's internal representation, §3). Rate of
turn is published as `navigation.rateOfTurn`, also a standard SignalK
key (rad/s, positive = starboard — same sign convention as PGN 127251,
§5.1), sourced from the same `RateOfTurnEstimator` output.

Raw magnetic field is published as three independently-toggleable
SignalK deltas: `sensors.hwt3100.magneticField.x/y/z`. There's no
established standard SignalK path for a raw 3-axis magnetic field
vector on a vessel (unlike heading/rate of turn, which have
spec-defined `navigation.*` keys) — this is a custom path under the
`sensors.*` namespace SignalK reserves for exactly this kind of
non-standard sensor data. Values are the module's raw, uncalibrated
sensor counts (same `mag_x/y/z` fields already visible via the serial
terminal, §8.1) — no unit is published in `SKMetadata` for them, since
the HWT3100 manual doesn't document a counts-to-µT conversion factor.
No corresponding N2K message was added: there is no standard PGN for
raw magnetometer data, and inventing a proprietary one would have no
real consumer on the bus (unlike PGN 130850/130851 in §8.3, which
exists because a real, if reverse-engineered, MFD protocol to
interoperate with already exists). Gated behind both the SignalK
master enable flag (§7) and its own dedicated toggle, off by default —
diagnostic data, not something every install needs streamed.

Staleness is surfaced via `meta.timeout` (§6), a spec-defined advisory
value published once as metadata — not a per-update flag on the delta
itself, and not an active alarm/notification.

## 6. Fault Handling / Persistence

- **Stale sensor data**: on the N2K side, the firmware transmits "not
  available" values on PGN 127250/127251 rather than omitting them
  (`ExpiringValue`, §10) — an active signal, not silence. On the
  SignalK side, staleness is **not** an active alarm — the
  `navigation.headingMagnetic` output carries `meta.timeout` (5 seconds,
  matching `ExpiringValue`'s own expiry window), the SignalK-spec-defined
  advisory mechanism for this (§5.2, §10): the firmware states how long a
  value should be considered valid, and any spec-aware consumer computes
  staleness itself from that plus the delta's own timestamp. **No
  dedicated RGB LED fault color, and no SignalK notification** — both
  were in earlier drafts of this section; see §10 for why each was
  dropped (LED: already fully claimed by SensESP with no way to share
  it; notification: `meta.timeout` is the more spec-idiomatic mechanism,
  chosen deliberately over an active alarm).
- **Persistence**: the calibration offset (heading) and WiFi/N2K
  enable-disable configuration must survive a restart. Live sensor
  readings are ephemeral and are not persisted.

## 7. Configuration

User-tunable, exposed via the SensESP web UI (reusing the parent
firmware's pattern):

- Enable/disable N2K output (independently of SignalK output).
- Enable/disable PGN 127250 specifically (independent of the master N2K
  switch — see §5.1 for why this matters more as the PGN set grows).
- Enable/disable SignalK output (independently of N2K output).
- Calibration offset: heading.
- WiFi credentials / SignalK server connection (standard SensESP config).

## 8. User Interface

In addition to the standard SensESP config UI (WiFi, calibration offset,
per-output/per-PGN toggles — §7), the web UI includes:

### 8.1 Serial Terminal (Monitor)

A small live serial terminal/monitor showing the raw data arriving from
the HWT3100 over UART — the `Magx=...,y=...,z=...,w=...` text lines
(§1.2). This is a diagnostic tool for the person installing/wiring the
module: it lets them confirm the sensor is actually sending data, at the
expected baud rate, before trusting the parsed heading value — useful
when tracking down wiring faults or a wrong baud rate.

Requirements:

- Shows recent incoming serial data — implemented as the last 30 raw
  lines, refreshed whenever the browser reads the corresponding config
  value (SensESP's config UI re-fetches periodically/on demand). This is
  a revision from the original "streamed, not polled" requirement: a
  working implementation-time investigation of SensESP 3.2.0 found no
  public way to push data to the browser at all (no WebSocket support,
  no custom-HTTP-handler extension point) — see ARCHITECTURE.md §2.5,
  §4 for the finding, and docs/plans/gateway-wiring.md for the decision
  to build this on SensESP's existing config REST API instead. Good
  enough for the diagnostic use case (§8.1's purpose is confirming the
  sensor is wired and talking, not a scrolling live feed).
- **Read-only as a monitor**: the terminal view itself has no free-text
  send box and no code path for transmitting arbitrary bytes. It is a
  passive tap on the incoming stream (see ARCHITECTURE §2.1, §6). Sending
  to the module happens only through the separate, constrained mechanism
  in §8.2 — never through this general-purpose terminal.
- Display format: since the wire format is already plain ASCII text (not
  a binary protocol), showing the raw text lines as received is
  sufficient — a hex-dump view has less value here than it would for a
  binary protocol, but doesn't hurt as a fallback for diagnosing
  corrupted/garbled bytes (e.g. wrong baud rate).

### 8.2 In-Place Calibration Commands

The web UI exposes the HWT3100's own on-module magnetic-field calibration
procedure as named actions for a fixed, known set of commands — not as
free-text command entry. Implemented as three boolean config-toggle
items (SensESP's config REST API has no "button" primitive — see §8.1's
note on why this rides the config API rather than a dedicated UI
control): setting one to true fires the corresponding command and it
immediately resets itself to false. See ARCHITECTURE.md §2.6 for the
mechanism and the accepted reboot-replay trade-off.

SensESP's `UIButton` class (documented as creating a real button in the
web UI's "Control" tab) was investigated as a nicer alternative to the
check-a-box-then-Save mechanic, since that flow reads as confusing —
checking a box looks like changing a persistent setting, not firing a
one-shot action. It's not usable: neither the vendored SensESP version
this project pins nor the current upstream `main` branch has any
server-side HTTP handler that serves or consumes `UIButton`'s registry
— the class exists, but nothing on the backend wires it to the web UI.
Given that, the three items keep the config-toggle mechanism, with
titles (`Calibration 1/3: Start` etc.) and identically-worded
descriptions making the "check + Save, then it un-checks itself"
behavior explicit on all three rather than assuming it's obvious from
the first one alone.

The full set of relevant commands, from the HWT3100-TTL/232 manual §5.3.1
(all plain ASCII text, terminated `\r\n`, all operate without leaving
ASCII mode):

| Command | Effect |
|---|---|
| `AT+CALI=1` | Start magnetic-field calibration (rotate module 2-3 full turns) |
| `AT+CALI=0` | End calibration |
| `AT+CALI=2` | Clear magnetic-field calibration offset (reset) |

(`AT+UART` also exists in the manual but isn't a calibration command
and isn't in scope for §8.2 — noted here for completeness, not
implemented (deferred, §9.2). `AT+ID` and `AT+MRATE` are permanently
excluded, not deferred — see the precautionary-exclusion callout in
§1.2 and §9.3. `AT+FILT` and `AT+PRATE` *are* implemented, but as
persisted config rather than calibration actions — see below.)

### 8.2a Output Filter (`AT+FILT`)

Per the manual's AT-command table: `AT+FILT=0` closes/disables the
module's output smoothing filter (module default); `AT+FILT=<n>` with
`n` in `[1, 999]` sets the filter strength — smaller values smooth
more. There is no `AT+FILT=1000` in the documented command set.

Exposed as a single persisted integer config value (`0-999`,
clamped), sent to the module once at boot and again on every config
change — unlike §8.2's calibration actions, this isn't a one-shot
trigger, since the module doesn't report its current filter setting
back and a reboot would otherwise silently revert it to `0`. Still
goes through `HWT3100SerialIO`'s allowlisted write surface: the value
is clamped to `[0, 999]` before it's ever formatted into a command
string, so this doesn't reopen the door to sending arbitrary text
(ARCHITECTURE.md §6).

### 8.2b Output Rate (`AT+PRATE`)

Per the manual's AT-command table: `AT+PRATE=0` sets the module to
**single-return mode**; `AT+PRATE=<n>` with `n` in `[10, 10000]` sets a
periodic output interval in ms. Values in `(0, 10)` clamp up to `10`
(the lowest valid periodic interval), not down to `0` — collapsing a
small positive request into "stop streaming entirely" would be far
more surprising than rounding up.

**Recommended minimum: `100` (10 datagrams/second).** Below that, both
`RateOfTurnEstimator`'s windowed regression (§1.3, §3 — the default
window is 2000ms/500ms min-span, ARCHITECTURE §2.4a) and the
N2K/SignalK heading refresh rate lose meaningful resolution for a
helm/autopilot display. The config UI's description field states this;
it isn't enforced as a hard floor (`10` remains the lowest value the
clamp will actually accept), since a slower rate is still valid — just
not recommended.

> **⚠️ `AT+PRATE=0` silences the module's continuous ASCII stream.**
> This firmware's entire read pipeline (§2, ARCHITECTURE §2.1) is
> built around parsing unsolicited `Magx=...` lines as they arrive —
> there is no request/response polling implemented anywhere. Setting
> the rate to `0` via config does not crash anything, but it does stop
> all heading data from ever arriving again until the rate is changed
> back. The config UI's description field carries this warning; it is
> not otherwise blocked, since a user might have a legitimate reason to
> silence the stream temporarily (e.g. before disconnecting the module
> for field service).

Exposed as a single persisted integer config value, same
apply-at-boot-and-on-change pattern as §8.2a's output filter — with one
difference: **the persisted default is a sentinel meaning "unknown"
(`-1`, `halser::kPrateUnknown`), not a real rate.** Forcing an
assumed default the way §8.2a does for `AT+FILT` would risk silently
overwriting whatever rate the module was already configured with
(possibly by someone else, before this firmware was ever installed) —
and if that assumed default happened to be wrong in the `0` direction,
it would be actively harmful per the warning above. So at boot, if the
persisted value is still `kPrateUnknown`, the firmware sends
`AT+PRATE=?` instead of a real command, and learns the module's actual
current rate from its `+PRATE=<n>` reply — which arrives on the same
raw-line stream as ordinary heading data (there is no separate reply
channel) and is parsed there. The learned value is then persisted
(so future boots skip the query and just (re-)apply the known rate,
same as §8.2a) and echoed back to the module as a confirming
`AT+PRATE=<n>` (harmless — the module is already at that rate).

Requirements:

- **Allowlist-gated, and additionally scoped to ASCII-mode-only
  commands**: the firmware ships with the fixed list above, hardcoded.
  The write path only ever transmits a command from that list; there is
  no code path that accepts or forwards arbitrary/user-supplied command
  text to the module. `AT+MODE` is not on the list, is not a calibration
  command, and must never be added (§1.2, §2).
- This is a distinct feature from §8.1's terminal — the terminal remains
  read-only; calibration commands go through their own named-action UI,
  so the two don't get conflated into one general "send anything" box.

### 8.2c Baud Rate: Auto-Detection and Runtime Switching (`AT+UART`)

Per the manual's AT-command table: `AT+UART=0/1/2` sets the module to
9600/115200/460800 baud respectively, replying `OK`. Two related
capabilities, combined into one persisted config value rather than
two separate features, because they share the same underlying state
(what baud are the firmware and module actually speaking):

**Auto-detection.** The persisted value starts unknown
(`halser::kBaudUnknown = -1`, same sentinel pattern as §8.2b's output
rate). At boot, if still unknown, the firmware tries candidate bauds
in order — **115200 (recommended), 9600 (factory default), 460800**
— opening `Serial1` at each and listening up to a fixed timeout for at
least one line that parses as valid HWT3100 output (`ParseHWT3100Line`,
§3). The first candidate that produces a valid line is adopted: the
value is persisted (so later boots skip straight to it) and the read
task starts communicating at that rate. No `AT+UART` command is sent
during detection — this is passive listening, not reconfiguration; the
firmware is discovering what the module already is, not changing it.
If none of the candidates work within their timeouts, detection fails
for this boot: the firmware falls back to reading at the recommended
115200 without persisting a value, so the next boot retries detection
fresh rather than getting stuck on a guess.

**Runtime switching.** Once a baud is known (whether learned via
detection or set explicitly), changing the persisted value through the
config UI to a different one of the three supported rates sends
`AT+UART=<n>` at the *current* baud, waits briefly for the module to
apply it, then reconfigures the firmware's own `Serial1` to the new
rate to keep talking — same "apply on config change" pattern as
§8.2a/§8.2b, except this one also has to reconfigure the transport
itself, not just a module-side setting. A value outside the three
supported rates is snapped to the nearest one rather than rejected —
see ARCHITECTURE.md §6 for why "clamp to nearest," not "reject," is
this codebase's consistent numeric-command validation strategy.

> **⚠️ Unverified timing detail (§11 Open Questions):** the manual
> doesn't specify whether the module's `OK` reply to `AT+UART` is sent
> at the *old* baud (before switching) or the *new* one (after). The
> firmware doesn't try to read that reply at all — it sends the
> command, waits a fixed settle delay, and reconfigures its own UART
> unconditionally — so this ambiguity doesn't block the switch, but it
> does mean a failed/garbled switch currently has no confirmation
> beyond "does data resume arriving afterward."

### 8.3 MFD-Triggered Calibration (N2K)

In addition to the web UI (§8.2), a compatible chartplotter/MFD can
start and stop the same on-module calibration over the N2K bus — user
requirement, matching how `htool/ESP32_Precision-9_compass_CMPS14`
(the same reference used for the Precision-9 device identity, §1.2,
§5.1) does it for its own sensor.

**This uses PGN 130850 (command) / PGN 130851 (acknowledgment), an
undocumented, reverse-engineered proprietary Navico/Simnet message —
not a published NMEA 2000 specification.** Everything about its byte
layout comes from reading the reference implementation's source, not
from an official spec, and none of it has been verified against real
B&G/Simrad MFD hardware or a live N2K bus (§10, §11). This is a
best-effort compatibility port, offered because the user explicitly
requested feature parity with that reference project — not a claim that
it's guaranteed to work with a real MFD.

The MFD-triggered path dispatches to the exact same
`CalibrationCommandHandler` (§8.2, ARCHITECTURE.md §2.2) the web UI
uses — same allowlisted write chokepoint, same two-command scope (start,
stop; no MFD-triggered "clear," matching the reference implementation,
which has none either).

## 9. MVP Scope

### 9.1 MVP Features

- Read heading (and raw magnetic field, for diagnostics) from the HWT3100
  over serial, in the module's default ASCII mode.
- Apply a configurable heading calibration offset.
- Transmit heading via PGN 127250 (Vessel Heading), toggleable, plus a
  master N2K enable/disable switch.
- Compute and transmit rate of turn via PGN 127251, from a sliding
  window of heading readings (§1.3, §3, §5.1, §10), independently
  toggleable.
- Present as a B&G Precision-9 on the N2K bus (device-identity fields —
  §1.2, §5.1, §10), not a generic/custom device identity.
- Transmit heading via SignalK deltas (`navigation.headingMagnetic`,
  SensESP/WiFi), independently toggleable.
- Detect stale sensor data and indicate it: N2K "not available" values
  (active) + SignalK `meta.timeout` (advisory, spec-idiomatic) — no LED,
  no active SignalK notification, see §6, §10.
- Live serial terminal in the web UI showing raw HWT3100 serial traffic
  (§8.1).
- In-place calibration commands: named, allowlisted `AT+CALI` actions to
  drive the HWT3100's own on-module magnetic-field calibration (§8.2).
- MFD-triggered calibration start/stop over N2K (§8.3), reverse-engineered
  proprietary protocol, unverified against real hardware.
- On-module output smoothing filter (`AT+FILT`), persisted config
  (§8.2a).
- On-module output data rate (`AT+PRATE`), persisted config, learned
  from the module at boot if not yet known (§8.2b).
- UART baud rate: auto-detected at boot if not yet known, switchable
  live via config (`AT+UART`, §8.2c).
- Raw magnetic field as SignalK deltas
  (`sensors.hwt3100.magneticField.x/y/z`), independently toggleable
  (§5.2).
- OTA firmware upgrades (reusing SensESP's built-in OTA support).

### 9.2 Post-MVP / Deferred

None remaining as of this version — the three items previously listed
here (`AT+UART` runtime baud switching, raw magnetic field as a
SignalK output, baud-rate auto-detection) are now implemented; see
§9.1, §8.2c, §5.2.

### 9.3 Out of Scope (Not Deferred — Hardware Limitation)

- **Pitch, roll, full attitude, PGN 127257.** The HWT3100-TTL/232 is a
  compass, not an IMU/AHRS — it cannot produce this data (§1.2, §3).
  This is not something a future version of this firmware can add; it
  would require a different sensor.
- **Modbus mode, `AT+MODE`, anything requiring it — including
  `AT+MRATE` and `AT+ID`.** Not a capability gap to fill later — a
  deliberate, permanent exclusion (§1.2, §2). `AT+MODE` is excluded
  because it's confirmed to brick a real unit; `AT+MRATE`/`AT+ID` are
  excluded precautionarily (unconfirmed risk) and because both are
  meaningless without Modbus mode anyway.

## 10. Design Decisions

- **Both outputs are first-class, independently toggleable**, rather than
  one being primary — this is a deliberate departure from a "N2K gateway
  with SignalK bonus" framing, because the intended deployments may only
  have one of the two networks present on a given vessel.
- **Calibration offset is applied once, in firmware, before either output
  path** — rejected the alternative of letting each output (N2K vs
  SignalK) apply its own correction, since that risks the two outputs
  disagreeing about vessel heading.
- **No production test-jig mode** — unlike the parent HALSER-default-firmware,
  this board/firmware doesn't have that requirement, so the GPIO0
  boot-routing pattern from the parent is not carried over.
- **Reuse SensESP** rather than a lighter custom WiFi/SignalK client, for
  consistency with the rest of the HALSER firmware family and to get
  WiFi provisioning, web UI, mDNS SignalK discovery, and OTA support for
  free.
- **PGN 127257 (Attitude) is not implemented, at all, rather than sent
  with pitch/roll as N2K "not available"** — considered sending Attitude
  with NA pitch/roll to "cover both PGNs anyway," but rejected it: a
  device advertising an Attitude PGN implies attitude sensing capability
  to anything listening on the bus, which would be actively misleading
  for hardware that is a compass, not an AHRS.
- **Raw magnetic field is diagnostic-only for MVP, not a SignalK/N2K
  output** — there's no standard SignalK path for a raw magnetometer
  vector on a vessel and no N2K PGN that fits it either; inventing a
  custom representation for data nobody's asked to consume yet isn't
  worth doing speculatively. It's still visible in the serial terminal
  (§8.1) for anyone who wants it for debugging/calibration verification.
- **Allow serial writes for calibration, but only via a fixed allowlist
  of named commands, never free-text passthrough** — the in-place
  calibration workflow genuinely needs to send commands to the module, so
  a blanket "never write" rule (an earlier draft's design) didn't survive
  contact with that requirement. Rejected a blocklist approach (allow
  everything except dangerous commands) because it only stops commands
  someone thought to list; an allowlist stops everything by default and
  only positively-vetted commands can ever be sent.
- **`AT+MODE` is excluded by never being implemented, not by allowlist
  filtering** — once it became clear the firmware has no functional need
  for Modbus mode at all (§1.2), the safest design isn't "allow writes,
  but filter out the dangerous one" — it's "the dangerous one was never a
  capability of this codebase in the first place." A command that
  doesn't exist in the code can't be sent by a bug, typo, or future
  well-meaning addition, the way an excluded-from-an-allowlist command
  theoretically still could be if someone edited the list without
  re-reading this document.
- **Clone the Precision-9's product info, device function/class, and
  manufacturer code verbatim; do NOT clone its hardcoded "unique
  number"** — the identity fields (model ID "Precision-9 Compass",
  software/model version, product code, device function/class,
  manufacturer code) are what make a listening MFD/autopilot recognize
  the device; the "unique number" is purely an N2K bus address-claim
  differentiator with no bearing on recognizability. The reference
  project hardcodes one fixed value; reusing that same constant across
  every install of this firmware would create a real (if unlikely)
  address-claim collision if two "clones" ever ended up on the same
  physical N2K bus. Instead, the unique number is derived from this
  board's own MAC address (matching the pattern the parent
  HALSER-default-firmware already uses for its own N2K identity) — this
  differs from a byte-for-byte behavioral clone, but only in the one
  field that was never part of "looking like a Precision-9" to begin
  with.
- **PGN 127257 (Attitude) still isn't implemented, even under
  "emulate the Precision-9"** — the reference project sends it using
  real heel/trim from a 9-DOF IMU (CMPS14). The HWT3100 has no
  accelerometer, so there's no honest data to put in that PGN; sending
  it anyway (fabricated or NA) would misrepresent capability the device
  identity now specifically implies it has (a Precision-9 model number
  *does* legitimately send attitude). Identity emulation intentionally
  doesn't extend to data emulation.
- **Rate of turn is computed from a heading sliding window, not
  fabricated as NA** — the user's explicit direction, and the one place
  this firmware synthesizes a PGN's data rather than either sending real
  sensor output or omitting the PGN.
- **The window's rate is a least-squares slope over every sample in the
  window, not a two-point endpoint difference** — an explicit
  improvement made after researching how others compute rate of turn
  (marine gyrocompass patents, pypilot's own heading-rate derivation,
  and the general numerical-differentiation literature). The original
  version of this estimator only used the oldest and newest sample in
  the window, discarding everything buffered in between; finite
  differences of that kind are well known to amplify sensor noise, and
  discarding the interior samples wasted data the ring buffer was
  already storing anyway. A windowed least-squares fit uses all of it,
  is strictly more robust to noise, and is mathematically identical to
  the old two-point method whenever only two samples fall in the window
  (a straight line through two points has one possible slope), so this
  didn't regress the common case — it only changed behavior when three
  or more samples are available, which is exactly where it should help.
  See ARCHITECTURE.md §2.4a for the window length and minimum sample-span
  chosen, and for how heading is unwrapped across the whole window (not
  just at the two endpoints) so the fit isn't corrupted by the 0/360
  boundary.
- **No dedicated LED for fault indication; SensESP's connection-status
  LED keeps sole ownership of GPIO8** — an earlier draft of §6 required
  RGB LED fault indication. Implementation found `gateway.cpp` was
  already (accidentally, from the gateway-wiring change) running its own
  `Adafruit_NeoPixel` on the same physical LED SensESP's
  `RGBSystemStatusLed` auto-claims via the `PIN_RGB_LED` build flag —
  two independent drivers on one WS2812-style addressable LED, a real
  timing/protocol conflict, not just a "who wins" question. Checked
  SensESP's public API for a way to pause/share it; none exists.
  Rejected running our own LED with `PIN_RGB_LED` removed (loses
  SensESP's free WiFi/WebSocket status blinking, and this was a closer
  call than it might sound — the user picked keeping SensESP's LED).
  Fault indication is SignalK-notification-only as a result.
- **SignalK staleness is `meta.timeout`, not an active notification** —
  an earlier draft implemented an active `notifications.*` alarm via a
  custom `SKEmitter` subclass (verified against SensESP's real sweep
  mechanism, not guessed). Replaced it after checking the SignalK spec
  directly: `meta.timeout` is the spec-defined, purpose-built mechanism
  for exactly this ("tell the consumer how long to consider the value
  valid"), published once as metadata rather than requiring the device
  to actively track and declare an alarm state. Trade-off accepted
  knowingly: per an open SignalK server RFC, `meta.timeout` isn't
  universally enforced by consumers today, so this is a bet on
  spec-correctness over guaranteed-actionable-everywhere. SensESP's
  `SKMetadata` already has a first-class `timeout_` field — no custom
  code needed, unlike the notification it replaced.
- **Fixed a real degrees/radians bug found while wiring up
  `meta.timeout`'s units field** — `navigation.headingMagnetic` and
  `SetN2kPGN127250`'s `Heading` parameter both require radians; this
  firmware's `HeadingReading.heading` is degrees throughout its internal
  pipeline (§3), and both N2K and SignalK output had been sending raw
  degrees, uncoverted, since the outputs were first wired up. Caught only
  because attaching a `"rad"` units label to a value that was actually in
  degrees would have been actively wrong, not just incomplete — a
  reminder that adding metadata is itself a form of verification.
- **MFD calibration (§8.3) copies the reference implementation's
  `DEVICE_ID=24` verbatim, despite not knowing what it represents** —
  user's explicit choice, over making it configurable or dropping the
  check entirely. Rejected making it configurable-with-unknown-default
  (adds a setting nobody yet knows the correct value for) and rejected
  dropping the check (if it does mean something like an N2K device
  instance, ignoring it could mean responding to a command meant for a
  different device on a multi-compass bus). Copying the reference's
  exact value is the most direct interpretation of "match the reference
  implementation."
- **MFD calibration dispatches through the existing
  `CalibrationCommandHandler`, not a separate write path** — this N2K
  command path and the web UI's config-toggle path both ultimately need
  to call `HWT3100SerialIO::SendCommand()`; giving the N2K handler its
  own direct access would create a second place capable of reaching that
  chokepoint, undermining the "one auditable caller" property established
  in §10 for the web UI path. Reusing the same handler keeps that
  property intact regardless of how many trigger sources exist.

## 11. Open Questions

- **Whether `AT+UART`'s `OK` reply arrives at the old baud or the new
  one (§8.2c)** — the manual doesn't say. The firmware doesn't rely on
  reading it either way (it sends the command, waits a fixed settle
  delay, then reconfigures its own UART unconditionally), so this
  doesn't block the switch, but it does mean there's currently no
  positive confirmation the switch actually succeeded beyond "does
  valid data resume arriving afterward." Also unverified: the exact
  settle delay needed between sending the command and the module
  actually being ready at the new rate — chosen as a reasonable
  fixed value, not derived from a datasheet timing spec.
- **The entire PGN 130850/130851 MFD-calibration mechanism (§8.3) is
  unverified against real hardware** — no B&G/Simrad MFD or live N2K bus
  was available in this environment. The byte layout, `DEVICE_ID`'s true
  meaning, the priority value used for the acknowledgment, and even
  whether a real MFD's calibration screen actually sends this exact PGN
  in this exact shape all rest entirely on one third-party reference
  implementation's reverse-engineering. Needs real-hardware testing
  before relying on this operationally.
- Exact field ordering/formatting edge cases in the HWT3100's ASCII
  output line (negative-number formatting, whether the module always
  sends all four fields in the same order, behavior at start-up before
  the first full line arrives) — the format is confirmed at a high level
  from the manual + vendor SDK, but real-hardware testing during
  implementation should verify parsing handles the actual byte stream
  correctly.
- Whether `AT+CALI=0` (end calibration) also persists/saves the result,
  or whether a separate save step exists — the manual doesn't explicitly
  say persistence is automatic. Needs confirming during implementation
  (e.g. by power-cycling the module after calibrating and checking
  whether the calibration held).
- `meta.timeout`'s actual effect was never verified against a live
  SignalK server or a real consumer app — no server was available in
  this environment, and (per the SignalK server RFC noted in §10)
  enforcement isn't universal across the ecosystem regardless. Worth
  confirming what the intended consumer(s) for a given install actually
  do with it before relying on this as the only staleness signal.
- The rate-of-turn sliding window length and minimum sample-span (chosen
  in ARCHITECTURE.md §2 as reasonable defaults, not derived from a real
  helm/autopilot's actual sensitivity requirements) may need tuning once
  this runs against real hardware and a real display — too short and the
  reading stays jittery/noisy, too long and it lags real turns. No live
  hardware was available to tune this empirically during this pass.

Resolved during implementation (see ARCHITECTURE.md and
docs/plans/gateway-wiring.md, docs/plans/fault-indication.md for
detail): N2K transmission rate for PGN 127250/127251 (100ms, matching
the parent firmware's `N2kHeadingSender` interval); serial-log transport
(SensESP's config REST API, not a WebSocket — SensESP 3.2.0 has no public
extension point for custom HTTP endpoints at all); calibration-command UI
mechanism (boolean config-toggle triggers, not dedicated buttons — same
underlying reason); fault-indication mechanism (SignalK `meta.timeout`,
no active notification, no LED — SensESP already owns the only RGB LED
with no sharing hook); a degrees/radians unit bug in both N2K and
SignalK heading output, found and fixed while wiring up `meta.timeout`'s
units field.
