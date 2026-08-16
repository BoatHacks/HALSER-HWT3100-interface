# Implementation Checklist - Quick Reference

**Before implementing a feature, work through this in order:**

## Phase 1: Explore
- [ ] Read the relevant issue/task fully
- [ ] Read the relevant sections of `SPEC.md`
- [ ] Read the relevant sections of `ARCHITECTURE.md`
- [ ] Explore existing code before writing anything — check the parent
      firmware (`HALSER-default-firmware`) for reusable patterns
      (`ExpiringValue`, `ConfigItem`/`PersistingObservableValue`,
      `TaskQueueProducer`-based I/O tasks) before writing something new

## Phase 2: Plan
- [ ] Think through the approach and alternatives
- [ ] Write a short implementation plan in `docs/plans/` (see
      `docs/plans/TEMPLATE.md`)
- [ ] Identify test scenarios up front, including hardware-in-the-loop
      cases (this is embedded firmware — some things can only really be
      verified against a real HWT3100 module and a real N2K/SignalK
      consumer)

## Phase 3: Test
- [ ] Where logic can be isolated from hardware (e.g. the ASCII line
      parser, the calibration offset math, the `HWT3100Command` lookup
      table), write it so it *can* be unit tested even if this project
      doesn't have a test harness set up yet
- [ ] For anything that can't be meaningfully unit tested (serial I/O
      timing, actual N2K bus behavior), plan the manual verification step
      instead — write down what "working" looks like before implementing

## Phase 4: Implement
- [ ] Write code to satisfy the plan
- [ ] Build frequently (`pio run`) while working, not just at the end
- [ ] **Never add a code path that writes arbitrary/unvalidated bytes or
      text to the HWT3100 serial link.** Any new write capability must
      go through `HWT3100SerialIO::SendCommand()`'s closed
      `HWT3100Command` enum (ARCHITECTURE.md §2.1, §6) — extend the enum
      deliberately, never add a string-accepting write path. If a change
      would add or touch anything resembling `AT+MODE`, stop and treat it
      as reopening a closed safety decision (SPEC.md §1.2, §9.3), not a
      routine addition.

## Phase 5: Verify
- [ ] Check edge cases, not just the happy path — the HWT3100 losing
      power/wiring, malformed serial lines, WiFi/SignalK server
      unreachable, N2K bus not present
- [ ] Confirm the change matches `SPEC.md`
- [ ] Confirm the change follows `ARCHITECTURE.md`
- [ ] If hardware is available: verify against a real HWT3100 module,
      not just against the documented protocol — SPEC.md §11 notes real
      hardware may reveal parsing edge cases the manual/vendor SDK don't
      cover

## Phase 6: Document & Commit
- [ ] Update SPEC.md/ARCHITECTURE.md if this change altered what they
      describe
- [ ] Remove any temporal language from comments ("new", "recently
      added") — comments should read correctly a year from now
- [ ] Firmware builds cleanly (`pio run`)
- [ ] Commit with a message that explains *why*, referencing the issue

---

## Common Mistakes to Avoid

**Don't:**
- Jump straight to coding before reading SPEC.md/ARCHITECTURE.md
- Add pitch/roll/attitude support "just in case" — the hardware can't
  provide it (SPEC.md §1.2, §9.3); don't build toward data that will
  never arrive
- Loosen the `HWT3100Command` write path to accept strings/arbitrary
  bytes "for flexibility" — that's exactly the design the allowlist
  exists to prevent (SPEC.md §10, ARCHITECTURE.md §6)
- Leave SPEC.md/ARCHITECTURE.md stale after a change that contradicts
  them (this project has already been burned once by stale assumptions
  about the hardware — see SPEC.md's revision history in git log)

**Do:**
- Explore before planning, plan before coding
- Write down the plan somewhere reviewable, even briefly
- Verify against the docs, not just against your own memory of the task
- When in doubt about the HWT3100's actual behavior, prefer real
  hardware testing over assumptions from the manual — the manual and the
  vendor's bundled SDK example didn't fully agree with each other either
