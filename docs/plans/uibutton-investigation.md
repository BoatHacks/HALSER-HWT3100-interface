# Investigation: SensESP UIButton live-hardware probe (temporary, complete)

## Result

**Confirmed on real hardware.** Flashed `v0.2.1` and checked every
page of the actual served web UI (Status, System, Log, WiFi, SignalK,
Configuration) — the "TEST: Ping SignalK server" button never appeared
anywhere. Not just "renders but does nothing on click": it doesn't
render at all, which is the stronger and more conclusive result, and
matches the static-analysis finding exactly (no page in the actual
served frontend bundle has any code path that reads a dynamic
list of commands/buttons).

The probe block has been removed from `gateway.cpp` now that the test
is done (this doc originally described it as in-place). See the
upstream issue write-up (delivered to the user, not committed to this
repo) for the full report, now updated with this confirmation.

## Overview

Static analysis (backend route grep + decompiling the actual served
frontend bundle) found no wiring at all between `sensesp::UIButton`
and the web UI — see the upstream issue write-up produced alongside
this. Per user request, added a real, on-hardware test to confirm this
empirically rather than rely solely on source inspection.

## Approach

`gateway.cpp` gets a `UIButton::add("uibutton_probe", ...)` whose
callback fires an `HTTPClient` GET to the SignalK server's own base
URL (`http://<sk_host>:<sk_port>/signalk`), using the same
`SKWSClient` the app already uses for its WebSocket connection
(`sensesp_app->get_ws_client()->get_server_address()`/
`get_server_port()`) so no new config is needed. Both the outgoing
request and its HTTP status are logged via `ESP_LOGI` on the device
itself, and the SignalK server's own access log is the independent
confirmation: if a GET to `/signalk` never shows up there no matter
how the button is clicked in the web UI, that's a live-hardware
confirmation of the "UIButton does nothing" finding.

This is explicitly temporary — not gated behind config, not documented
as a feature, marked for deletion in-line once the test is done
(either way: confirms the bug for the upstream issue, or disproves the
static analysis and means the calibration-checkbox decision needs
revisiting).

## Verification

`pio run -e halser` — builds clean (pulls in `HTTPClient` as a new
dependency; flash usage 84.5% -> 87.1%, still well within budget).
`pio test -e native` — unaffected, 43/43 (gateway.cpp isn't part of
the native build).

## Files Modified

- `src/gateway.cpp` (temporary block, clearly marked for removal)
