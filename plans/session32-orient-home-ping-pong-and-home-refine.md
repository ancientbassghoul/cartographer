# Session 32 — ORIENT_HOME real-angle convergence + HOME_REFINE position tightening + DOCK_FLOOR settle-gate

## Context

Flight `20260720_223555` reached `ORIENT_HOME` (the "face the take-off heading before docking"
postlude leg) and got stuck ping-ponging turn commands forever, starting at `22:55:37.228`:

```
22:55:37.261  turn +30 deg (err +15.1)
22:55:44.262  turn -30 deg (err -17.4)
22:55:49.967  turn +30 deg (err +16.9)
22:55:56.011  turn -30 deg (err -15.5)
22:56:02.357  turn +30 deg (err +17.2)
22:56:09.731  turn -30 deg (err -15.8)
22:56:16.569  turn +30 deg (err +15.9)
```
(log runs out here — the drone was still oscillating and never reached `DOCK_FLOOR`.)

**Root cause**: `ORIENT_HOME`'s `PLAN` phase ran the live bearing error through `_quantize_turn`,
which snaps it to the nearest whole multiple of `turn_step_deg` (config: `30.0`) — it can *only*
ever command 0° or ±30°, never anything in between. `_turn_steps` (the code that actually flies the
turn) already supports an arbitrary continuous angle — the quantization was a self-imposed
restriction, not a platform limitation. Once the residual error lands near half a step (~15°,
exactly what the log shows), each open-loop 30° turn overshoots to the *other* side by a similar
margin, and the "done" check (`round(be/30) == 0`, i.e. `|be| < 15`) sits right on that same
knife-edge — so it can loop forever, alternating sides, never both landing inside the dead zone
AND stopping.

This is a different design point from the other two `_quantize_turn` call sites (leg `ORIENT` and
`RETURN_TO_ORIGIN`'s aim-at-origin): those don't require an exact final heading — they turn
approximately toward a goal and then just advance, self-correcting on the next re-plan/re-aim
regardless of residual error. `ORIENT_HOME` is the only one that gates real progress on hitting a
tight implicit heading tolerance, so it's the only one that could pathologically loop. Scope: kept
to `ORIENT_HOME` only; those two other sites are untouched.

If `ORIENT_HOME` had converged, the pre-existing flow was: `ORIENT_HOME` → `DOCK_FLOOR` (pulsed
micro-descent) → `LOW_STANDOFF` (up-nudge) → `DONE` (terminal hover). This session inserts a new
`HOME_REFINE` stage between `ORIENT_HOME` and `DOCK_FLOOR` (operator ask, once heading is nailed,
also tighten the resting *position* against the true origin before descending), and separately
fixes `DOCK_FLOOR`'s own inter-pulse wait to use a real settle-gate (operator ask).

## Design

### 1. ORIENT_HOME — turn by the real (clamped, not quantized) angle + explicit tolerance + give-up cap

- Drop `_quantize_turn` for this call site: `theta = be`, clamped to `±turn_step_deg` (same
  SLAM-survives-a-small-turn rationale as the other two sites, kept). Pure precision improvement —
  `_turn_steps` already scales continuously with `theta`.
- New `orient_home_tol_deg` (default `5.0`) replaces the old `abs(theta) < 1e-6` check: "facing the
  take-off heading" now means `abs(be) <= orient_home_tol_deg`, decoupled from the turn-magnitude
  clamp.
- New `orient_home_max_s` (default `60.0`) give-up cap, same "proceed HERE, VISIBLE, no silent
  fallback" idiom as `home_max_s`. On expiry, falls through to `HOME_REFINE` from wherever the
  heading ended up.

### 2. HOME_REFINE — tighten position after orientation (new state)

Inserted between `ORIENT_HOME` and `DOCK_FLOOR` (all of `ORIENT_HOME`'s exits, including the new
cap, now go to `HOME_REFINE`; `HOME_REFINE` is the only thing that transitions to `DOCK_FLOOR`).
Mirrors `ORIENT_HOME`'s own `PLAN → PUSH → SETTLE → PLAN` loop shape:

- **PLAN**: `reached = dist(pos, [0,0])`. If `<= home_fine_reach_dist` (`0.15`) → `DOCK_FLOOR`.
  Else compute the bearing-to-origin the same way `RETURN_TO_ORIGIN` does
  (`atan2(0-pos.x, 0-pos.z)`, wrapped against live `heading_deg`), and pick ONE of 4 fixed pushes
  by body-frame quadrant (45°/135° splits; sign convention `+X = right`, `+yaw = right`,
  `+joy_horizontal = right` cross-checked against `flight_playbook.json`/`perception_worker.py` —
  all one consistent "right"): forward/backward full throttle (`trigger`/`reverse` = 1.0, ramps as
  usual) held `home_refine_fwd_s` (`0.32`); left/right strafe full throttle (`joy_horizontal` =
  ±1.0, never ramped) held `home_refine_strafe_s` (`0.16`).
- **PUSH**: play the one-step `RecipePlayer` to completion.
- **SETTLE**: the exact `_settle_poll(require_fast=False, min_frames=settle_fresh_frames,
  max_hold_s=None)` call `ORIENT_HOME`/`RETURN_TO_ORIGIN` already use — re-measure before ever
  chaining another push.
- Give-up cap `home_refine_max_s` (default `45.0`), same VISIBLE idiom.
- Added to `POSTLUDE_STATES` (SLAM-loss during a refine push gets the dedicated
  `POSTLUDE_LOST_HOLD`, same as every other postlude stage) and to `_step_postlude_lost`'s resume
  dispatch. NOT gated on clearance/ring (matches the literal spec — fixed durations, no obstacle
  check; origin proximity is generally open since it's the take-off spot, but this is a live-fly
  watch item).

### 3. DOCK_FLOOR — real settle-gate between descent pulses

`PULSE` (a `dock_pulse_s` DOWN pulse) was followed by `REST`, which waited a **fixed** `dock_rest_s`
then read whatever `pos_y` happened to be in `plan` at that instant — the same stale-frame shape
session 24's settle-gate rewrite eliminated everywhere else, just never applied to `DOCK_FLOOR`
(added later, mirroring `ASCEND`'s own `PULSE/REST/LATCH`). Fixed, scoped to `DOCK_FLOOR`'s
`_dock_phase` only (`ASCEND`'s `_ascend_phase` mirrors the same shape but is untouched — not asked
for, flagged here as the same class of gap for a future session):

- `PULSE → REST` now also opens a settle-gate (`_settle_begin`).
- `REST` waits on `_settle_poll` (6 fresh frames, same call as everywhere else); once settled, runs
  the existing valid-pose / dZ-sample / stall-count / phase-transition logic unchanged, now off a
  provably-fresh post-pulse pose. If the pose still isn't trustworthy once settled, re-opens the
  gate and keeps waiting (`dock_max_s` still backstops a truly dead pipeline).
- `dock_rest_s` became fully unused — removed from `__init__` and `config.yaml`.

## Files touched

- `autopilot.py`: `__init__` config reads (`orient_home_tol_deg`, `orient_home_max_s`,
  `home_fine_reach_dist`, `home_refine_fwd_s`, `home_refine_strafe_s`, `home_refine_max_s`; removed
  `dock_rest_s`), state-var init (`_orient_home_t0`, `_home_refine_phase`, `_home_refine_t0`),
  `ORIENT_HOME` handler rewrite, new `HOME_REFINE` handler, `DOCK_FLOOR`'s `PULSE`/`REST` rewrite,
  `POSTLUDE_STATES` + `_step_postlude_lost` resume dispatch, self-test block (see below).
- `config.yaml`: new knobs listed above; `dock_rest_s` entry removed (comment left explaining why).

## Self-tests added

- `ORIENT_HOME real-angle convergence`: starts the bearing error just past HALF a `turn_step_deg`
  (the exact knife-edge shape of the diagnosed bug) with a REALISTIC noisy open-loop turn (actual
  rotation = 1.4× the commanded angle, never exact) — asserts it still converges to `HOME_REFINE`
  within a bounded number of turn+settle cycles (currently converges in 2) instead of oscillating
  indefinitely. Plus a dedicated `orient_home_max_s` cap test (pose stays invalid forever → proceeds
  anyway, VISIBLE).
- `HOME_REFINE`: a quadrant-pick test (4 synthetic pos/heading combos, one per
  forward/backward/strafe-left/strafe-right, asserting both the picked recipe name AND the actual
  emitted control field/sign); a convergence test (simulated position genuinely closes on each
  forward push → reaches `DOCK_FLOOR`); and a `home_refine_max_s` cap test (position never improves
  → proceeds anyway, VISIBLE).
- Updated three pre-existing postlude self-tests to account for the new `HOME_REFINE` hop in the
  state sequence (`_is_subsequence` checks are order-only so most needed no change; two needed
  `home_refine_max_s`/`orient_home_max_s` = 0 overrides so synthetic non-physical drive loops that
  never actually move the drone don't burn their whole tick budget waiting on a give-up cap that
  isn't what they're testing).

`python autopilot.py --self-test`: ALL PASS (includes all pre-existing suites, unaffected).

## Status

BUILT — self-tests green. **No live hardware in this dev environment — live-fly confirmation is
still pending**, stacked on top of the already-pending session 20b-31 checklist (none of that has
flown yet either). `home_refine_fwd_s`/`home_refine_strafe_s`/`home_fine_reach_dist` are
freshly-specified magnitudes (not yet observed live) — watch closely on the first live docking, per
this codebase's own pattern of retuning such constants after first live observation (e.g.
`backoff_hold_s`).
