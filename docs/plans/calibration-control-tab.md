# Calibration buttons on the Control tab

## Problem

The three calibration actions (start/end/clear, SPEC.md §8.2) were
implemented as boolean config-toggle items — check a box, hit Save, it
un-checks itself — because SensESP's `UIButton` class, despite its doc
comment claiming it "creates a button in the Control tab," had no
backend or frontend wiring behind it in either the vendored 3.5.0
release or upstream `main` (`docs/plans/uibutton-investigation.md`).
Confirmed both by static analysis and on real hardware.

That gap has since been fixed in `BoatHacks/SensESP` (a fork of
`SignalK/SensESP`): a new `/api/buttons` HTTP handler plus a Control-tab
frontend page. A PR carrying the fix upstream has been filed but not
yet merged.

## Decision

1. **Depend on `BoatHacks/SensESP` instead of upstream, temporarily.**
   `platformio.ini`'s `lib_deps` now points at
   `https://github.com/BoatHacks/SensESP.git` with a comment marking it
   as a stopgap pending the upstream PR merging, at which point this
   reverts to `SignalK/SensESP @ ^3.2.0` (or whatever version range
   includes the fix).

2. **Replace the three config-toggle checkboxes with real `UIButton`s.**
   `sensesp::UIButton::add("hwt3100_calibration_start", "Start
   Calibration")->attach(...)` etc., each lambda calling straight into
   the existing `CalibrationCommandHandler` (unchanged — `StartCalibration()`
   / `EndCalibration()` / `ClearCalibration()`). No persisted state,
   no config item: a click either fires the command or (if missed) is
   simply not fired — nothing to replay on the next boot, unlike the
   old `PersistingObservableValue<bool>` trigger pattern.
   `must_confirm` is left at its default (`true`) for all three, since
   each changes the module's on-module calibration state and is
   annoying to redo if clicked by accident.

3. **Status display: a `StatusPageItem`, not a synchronous response.**
   The user's ask was "user pushes 'start', ... status line changes to
   serial response content" — ideally shown right on the Control tab,
   inline with the click. `UIButton`'s contract (even in the fixed
   fork) is fire-and-forget: `POST /api/buttons/<name>` triggers
   `notify()` and returns, with no mechanism to carry the module's
   reply back to the caller. Building that would mean a second round
   of changes to `BoatHacks/SensESP` (extend the button handler and
   `UIButton` itself to support a response value, extend the Control
   page to display it) — another dependency on code this project
   doesn't have push access to merge upstream on its own timeline.

   Chosen instead: a `StatusPageItem<String>` ("HWT3100 Calibration
   Reply") on the existing, unmodified Status page. `gateway.cpp` adds
   a third consumer on the existing `raw_line_producer` (the same
   stream `SerialTerminal` and the `AT+PRATE=?` reply parser already
   read from, SPEC.md §8.1/§8.2b) that checks each incoming line with
   `halser::IsCalibrationReply()` and, on a match, updates the status
   item to that line's text. This requires zero further SensESP
   changes — `StatusPageItem`/`/api/info` already work today, unlike
   `UIButton`.

   Trade-off, accepted: this is a best-effort correlation, not a
   guaranteed request/response pairing. If two calibration commands
   fire close together, or a reply line is dropped/garbled, the status
   item could show a stale or slightly-wrong-attribution reply. Good
   enough for "did something happen and what does the module think
   happened," not a hard guarantee tied to one specific click.

## `IsCalibrationReply()` — how replies are recognized

Per the manual's AT-command table, the module's replies to the three
`AT+CALI` commands are fixed, human-readable text (not a structured
`+CALI=...` format like `AT+PRATE=?`'s `+PRATE=<n>`):

| Command | Reply text |
|---|---|
| `AT+CALI=1` | `Calibrating` |
| `AT+CALI=0` | `Calibration completed` |
| `AT+CALI=2` | `Reset mag offset param` |

`hwt3100_calibration_reply.h/.cpp` exports these three strings as
constants and an `IsCalibrationReply(const char*)` exact-match
predicate — pure, dependency-free logic, unit-tested in
`test/test_hwt3100_calibration_reply/` (`pio test -e native`), same
pattern as `hwt3100_prate_command.h`'s `ParsePrateReply()`.

## Not done

- No synchronous click-to-response UI (see trade-off above) — would
  need further upstream SensESP work.
- No new SensESP frontend/backend changes of any kind — this feature
  only consumes `BoatHacks/SensESP`'s already-merged `UIButton` fix and
  already-existing `StatusPageItem`/Status page.
