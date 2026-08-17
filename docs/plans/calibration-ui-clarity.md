# Implementation Plan: Clarify calibration action checkboxes

## Overview

User feedback: the three calibration triggers (§8.2) — check a box,
then Save, then it un-checks itself — read as confusing, since a
checkbox visually implies a persistent setting rather than a one-shot
action.

## Investigated: SensESP's `UIButton`

`UIButton` (`sensesp/ui/ui_button.h`) is documented as creating a real
button in the web UI's "Control" tab, notifying observers on click —
exactly what this use case wants. Investigated as a replacement for
the checkbox mechanism.

**Finding: not usable.** Checked both the vendored SensESP version this
project pins (resolved to 3.5.0) and the current upstream `main` branch
on GitHub. Neither has any server-side HTTP handler — in
`config_handler.cpp`, `app_command_handler.cpp`, or
`base_command_handler.cpp` — that serves or consumes `UIButton`'s
static registry (`get_ui_buttons()`). The class itself exists and
self-registers into a `std::map`, but nothing on the backend exposes
that map to the web UI or handles a click. Could not verify the
frontend's expectations directly (the embedded JS in `js_sensesp.h` is
gzip-compressed in the vendored copy), but the total absence of any
matching route on the backend is conclusive enough: wiring app code
against `UIButton` would silently do nothing when clicked.

## Approach taken (user-selected)

Keep the checkbox/config-toggle mechanism (the only actually-working
option), and make the confusing part explicit instead of implicit:

- Titles changed to `Calibration 1/3: Start`, `2/3: End`, `3/3: Clear`
  — the numbering makes the three read as one related set (previously:
  `Start Magnetic Calibration` / `End Magnetic Calibration` / `Clear
  Magnetic Calibration`, each title-cased identically but with no
  visual grouping cue).
- Every description now states the same mechanic explicitly and
  identically: "One-shot action, not a setting: check the box and Save
  to send `AT+CALI=<n>`... it un-checks itself once sent." Previously
  only the first item's description hinted at this ("Set true to
  send..."), leaving the reader to infer the same behavior applied to
  the other two.
- Schema field titles changed from generic `Start`/`End`/`Clear`
  (redundant with the item title) to the literal command being sent
  (`Send AT+CALI=1` etc.), so the checkbox itself states what it does.

No functional/write-path change — `WireCalibrationTrigger` and
`CalibrationCommandHandler` are untouched.

## Files Modified

- `src/gateway.cpp`
- `SPEC.md` §8.2
