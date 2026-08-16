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
| heading     | float | Magnetic heading, degrees, 0–360, offset-corrected  |
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
| rateOfTurn | float, radians/second | (newest heading − oldest heading in window) / elapsed seconds, wraparound-corrected, positive = turning to starboard |

Requires at least two samples spanning a minimum elapsed time within the
window to produce a value at all (§10) — below that, it's "not
available," same as any other not-yet-valid reading (§6).

## 4. Sources / Inputs

- **Single source**: the HWT3100-TTL module, connected via UART to the
  ESP32-C3, GPIO2 TX / GPIO3 RX (HALSER's UART terminal block, RX-select
  jumper on "U" — confirmed against Hat Labs' HALSER hardware docs; see
  ARCHITECTURE.md §5). Default baud 9600 bps (§1.2).
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

- **PGN 127250 — Vessel Heading**: real data, unchanged from before this
  revision.
- **PGN 127251 — Rate of Turn**: *computed*, not sensed — the HWT3100
  has no gyroscope, so this is derived from a sliding window of recent
  heading readings (§1.3, §3). This is genuinely-derived data (the
  actual rate the heading has been changing), not a fabricated/NA
  placeholder sent only to pad out the PGN fingerprint.

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

Heading delta published as `navigation.headingMagnetic`, consistent with
the SignalK spec's navigation group. Raw magnetic field readings are not
published as SignalK deltas in MVP — there's no established SignalK path
for a raw 3-axis magnetic field vector on a vessel, and no known
consumer for one (§10 Design Decisions); they remain visible only via the
serial terminal (§8.1) and internally for the calibration workflow (§8.2).

When data is stale, the firmware should not keep republishing the last
value silently — see §6 (Fault Handling) for how staleness is surfaced.

## 6. Fault Handling / Persistence

- **Stale sensor data**: if the HWT3100 stops producing readings within
  the expected interval, the firmware actively signals a fault rather
  than going silent — via RGB LED status color and a SignalK notification
  at minimum, and by transmitting N2K "not available" values on PGN
  127250 rather than omitting it (resolved design decision, §10 — the
  parent firmware's existing `ExpiringValue` pattern already does this).
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

The full set of relevant commands, from the HWT3100-TTL/232 manual §5.3.1
(all plain ASCII text, terminated `\r\n`, all operate without leaving
ASCII mode):

| Command | Effect |
|---|---|
| `AT+CALI=1` | Start magnetic-field calibration (rotate module 2-3 full turns) |
| `AT+CALI=0` | End calibration |
| `AT+CALI=2` | Clear magnetic-field calibration offset (reset) |

(`AT+UART`, `AT+PRATE`, `AT+ID`, `AT+FILT` also exist in the manual but
aren't calibration commands and aren't in scope for §8.2 — they're
noted here for completeness, not implemented in MVP.)

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
- Detect stale sensor data and actively indicate the fault (LED + SignalK
  notification + N2K "not available" values, at minimum).
- Live serial terminal in the web UI showing raw HWT3100 serial traffic
  (§8.1).
- In-place calibration commands: named, allowlisted `AT+CALI` actions to
  drive the HWT3100's own on-module magnetic-field calibration (§8.2).
- OTA firmware upgrades (reusing SensESP's built-in OTA support).

### 9.2 Post-MVP / Deferred

- Any of `AT+UART` (runtime baud switching), `AT+PRATE` (return rate),
  `AT+FILT` (output smoothing filter) exposed as config — all documented,
  all ASCII-mode-safe, just not needed for MVP.
- Raw magnetic field as a SignalK delta or a custom N2K message, if a
  real consumer need shows up (§5.2, §10) — currently diagnostic-only.
- Auto-detection of the HWT3100's baud rate. Deferred — MVP hardcodes
  9600 (the documented default); auto-detect adds complexity not needed
  for a fixed hardware pairing.

### 9.3 Out of Scope (Not Deferred — Hardware Limitation)

- **Pitch, roll, full attitude, PGN 127257.** The HWT3100-TTL/232 is a
  compass, not an IMU/AHRS — it cannot produce this data (§1.2, §3).
  This is not something a future version of this firmware can add; it
  would require a different sensor.
- **Modbus mode, `AT+MODE`, anything requiring it.** Not a capability
  gap to fill later — a deliberate, permanent exclusion (§1.2, §2).

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
  sensor output or omitting the PGN. Chose a trailing-window derivative
  (newest heading − oldest heading in the window, wraparound-corrected,
  divided by elapsed time) over a simple frame-to-frame delta because
  raw frame-to-frame differences amplify sensor/quantization noise into
  a jittery rate signal; averaging over a window trades a little
  responsiveness for a much more usable value on a helm/autopilot
  display. See ARCHITECTURE.md §2 for the window length and minimum
  sample-span chosen.

## 11. Open Questions

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
- **Fault indication (RGB LED + SignalK notification) for stale sensor
  data is not yet implemented** — §6/§9.1 describe it as an MVP
  requirement, but gateway.cpp currently only initializes the LED
  without driving it, and no SignalK notification is sent. The N2K side
  of §6 is already covered (§10: `ExpiringValue::to_n2k()` sends
  `N2kDoubleNA` when stale, as a side effect of reusing the parent's
  pattern). Needs its own small design pass (LED color/pattern, SignalK
  notification path/format) — see docs/plans/gateway-wiring.md.
- The rate-of-turn sliding window length and minimum sample-span (chosen
  in ARCHITECTURE.md §2 as reasonable defaults, not derived from a real
  helm/autopilot's actual sensitivity requirements) may need tuning once
  this runs against real hardware and a real display — too short and the
  reading stays jittery/noisy, too long and it lags real turns. No live
  hardware was available to tune this empirically during this pass.

Resolved during implementation (see ARCHITECTURE.md and
docs/plans/gateway-wiring.md for detail): N2K transmission rate for PGN
127250 (100ms, matching the parent firmware's `N2kHeadingSender`
interval); serial-log transport (SensESP's config REST API, not a
WebSocket — SensESP 3.2.0 has no public extension point for custom HTTP
endpoints at all); calibration-command UI mechanism (boolean
config-toggle triggers, not dedicated buttons — same underlying reason).
