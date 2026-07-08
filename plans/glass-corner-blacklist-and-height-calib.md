# Deferred work — glass-corner blacklist bugs + per-goal height calibration

Two independent pieces of work deferred out of session 6. Both are **designed/diagnosed, NOT built**.
Referenced from PROGRESS.md.

---

## 1. Glass-corner blacklist bugs (diagnosed on flight `20260707_151030`, confirmed in `frontier_planner.select()`)

Symptom: at a glass/collider wall the drone fired standoff stops "like crazy" yet never retired the
goal. It re-committed to the SAME unreachable far corner `[-1.0589, 7.5521]` (z=7.55, map edge — the
drone never gets past z=6.38), "blacklisting" it three times with "2 total" never growing. Two bugs:

### Bug A (dominant) — the verify/reposition target bypasses the blacklist
When no frontier is reachable, `select()` enters `verifying=True` and returns `farthest_free` as the
goal **with no `_excluded()` check** (frontier_planner.py:272-275), then holds that fixed `verify_target`
until the drone physically reaches it (line 270). The corner is "free" on the map but sits behind a
glass/collider wall → verify never completes, the goal never changes. The 2-bump blacklist DOES fire
against it but is a **no-op**: the verify branch never consults the blacklist, and `committed_goal` is
already `None` during verify, so "drop the commitment" does nothing. The drone chases an unreachable
corner forever. **The verification mechanism has no concept that its target may be unreachable.**

**Fix directions (pick with the user):** give the verify/reposition target an unreachable-escape —
when the drone bump-blacklists the region it is verifying toward, set `verifying=False`, exclude that
corner, and recompute `farthest_free` EXCLUDING blacklisted regions (or declare done if none remain).
The verify branch must consult `_excluded()` / abandon a target the drone keeps bumping.

### Bug B — the 2-bump latch can't re-arm at a clearance stand-off
Once SLAM maps the wall, the clearance stand-off (0.6) fires BEFORE the ram guard can accrue, pinning
the drone in a tight `REPLAN→ORIENT(0°)→ADVANCE(a hair)→standoff-stop` loop at clr≈0.57. That path
never reverses and never moves > `goal_reach_dist`, so `rearm_bump_if_disengaged` never re-arms → every
standoff contact is a MISSED-BUMP and the counter is stuck at 0. Even if Bug A were fixed, the counter
could not reach 2 here.

**Fix directions (pick with the user):** make an advance-blocked contact count even when pinned at a
stand-off — e.g. a "pinned at stand-off" detector (N consecutive standoff-stops at ~the same pose = a
bump), a small back-off on the standoff stop (also re-arms the latch + gives SLAM parallax), or re-arm
on a standoff-stop event rather than only on reverse/displacement.

---

## 2. Part 3 — per-goal `CALIBRATING_HEIGHT` (semi-planned, getting urgent)

Add a `CALIBRATING_HEIGHT` state triggered **only** when the frontier goal genuinely changes
(`dist(leg_goal, _leg_goal_prev) > goal_assoc_dist` at REPLAN; the first post-prelude goal does NOT
re-calibrate), **not** on PLAN-LOST. It re-runs the Part-2 two-phase ascend→descend to re-tap the
ceiling, re-nulls `target_altitude_y` to re-latch the baseline (undoing per-leg downward drift), then
hands to ORIENT for the new goal. ASCEND/DESCEND get a `_post_ascend` return field so the same states
serve both the prelude (→BASELINE_NUDGE→REPLAN) and CALIBRATING_HEIGHT (→ORIENT). Gate with a
`calibrate_on_goal_change` config flag.

---

## Verification (either piece)
- Extend the relevant `--self-test` (frontier_planner for the blacklist/verify bugs; autopilot for
  CALIBRATING_HEIGHT state routing) and keep ALL module self-tests green.
- Re-fly `autopilot.py --explore --log`: (1) the drone RETIRES an unreachable glass corner and moves
  on instead of ramming it forever; (2) height re-taps the ceiling on a genuine goal change only.
