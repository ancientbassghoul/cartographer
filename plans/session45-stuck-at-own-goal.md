# Session 45 — the "stuck next to its own goal" stall: four defects, all fixed

## Origin

The operator flagged the end of flight `20260901_112227`: the drone sat still for minutes "near its
goal," and asked why neither of the two guards he expected had caught it — (1) a goal too close to
the drone getting blacklisted, (2) a goal being marked reached so the mission moves on.

Answer: **one did not exist, the other could not run.** Two further defects explained why the flight
could not make progress or retire the goal by any other route either.

## Diagnosis (from the flight's own log + timeline, not inference)

- Final state: `pos=[-0.7592, -0.0389]`, `goal=[-0.5464, -0.2674]` → **dist 0.312**, with
  `goal_reach_dist` = **1.0**. It was three times deeper inside "reached" than the threshold.
- **221 seconds** inside 1.0u of the goal (`t=1756522.3 → 1756743.3`), status `OK` for ~2300 of
  those ticks — so it was not merely blind.
- Cycle: `SLAM_HOLD` →(15s forced hop)→ `REPLAN` → `ORIENT` (turn +30°) → `PARALLAX_PUSH` →
  `SLAM_HOLD` ~30ms later, forever. **The drone turned every ~25s and never translated.**
- Whole-flight state histogram: `ORIENT` 39, `SLAM_HOLD` 32, `HOLD_LOST` 23, `ADVANCE` **5**.
- Goals-DB at the end: the hammered disc `[-0.566, -0.318]` sat at **`picks=1`** after ~7 commits.
- The goal jittered between commits by less than `goal_area_radius` (0.5): `[-0.5664,-0.3182]`,
  `[-0.6265,-0.0794]`, `[-0.4492,-0.3091]`, `[-0.5488,-0.2807]`, `[-0.5464,-0.2674]` — one disc.

### Defect 1 — the planner will commit a goal the drone is standing on (did not exist)
`_select_reachable` had **no minimum-distance check at all**, and two existing behaviours actively
steer into it: the utility divides by distance (`goal_dist_weight`), so a frontier on top of the
drone scores *highest*, and the clearance inset walks the goal *toward* the drone.

### Defect 2 — "goal reached" was only tested inside ADVANCE (existed, could not run)
`autopilot.py`'s `goal reached (d=...)` branch lives in the `ADVANCE` handler. The stall never
entered `ADVANCE`, so a goal 0.31u away was never evaluated.

### Defect 3 — PARALLAX_PUSH excluded from the forced-hop grace window (**root cause**)
`_enter()` cleared `_slam_slow_hop_deadline` for anything outside `("ADVANCE","ORIENT")`, and
`PARALLAX_PUSH`'s slow-check was a bare `if self._slam_slow:` with no `_slam_slow_hop_active()`
guard (unlike `ADVANCE`). So under chronically slow SLAM every forced hop died the instant it
entered the push — making the session-35/43 rescue a no-op for any off-axis goal.

### Defect 4 — the pick-dedup starved the loop-blacklist
`same_goal_as_last_pick = (not pick_moved) and (prev_goal is None)`. `prev_goal` is set only when a
hop is **judged**; no hop ever completed, so every pick was suppressed → `picks` stuck at 1 while
the loop guard needs `> goal_loop_min_picks` (2). The one mechanism that could have retired the goal
was starved by the exact condition it exists to catch — the same class of starvation the code
already documents fixing once (20260720), via a different path.

## Built

1. **`frontier_planner._select_reachable`** — reject a candidate whose **post-inset committed**
   point is within `goal_reach_dist` of `pos`: soft-blacklist it (`permanent=False`, reason
   `"reached"`) and try the next-best, same bounded pattern session 33 established. Soft/round scope
   so `_whitelist_round()` restores it on the next corner arrival; genuine recurrence is escalated by
   the (now unstarved) loop guard. Surfaced as a `TOO-CLOSE …` select event through a new transient
   `last_select_events`, drained in `perception_worker` after `select()` (rides the next plan, like
   the bump receipts). **Also fixed a bug introduced while writing this**: the drop filter must key on
   the RAW centroid (`remaining` is keyed on raw centroids) while the blacklist keys on the
   POST-INSET point — captured as `raw_goal`; blacklisting the raw centroid instead would leave the
   adjusted point committable, which is precisely the session-33 defeat mechanism.
2. **`autopilot.step()`** — a state-independent "goal already reached" check, placed **before** the
   `SLAM_HOLD` handler (it had to move there: `SLAM_HOLD` returns early, and that is exactly where
   the flight was parked). Retires the leg to `SETTLE`→`REPLAN` from wherever it sits.
3. **`_enter()` + PARALLAX_PUSH** — added `"PARALLAX_PUSH"` to the grace-window exemption and gave
   its slow-check the `not self._slam_slow_hop_active(now)` guard, matching `ADVANCE`.
4. **Bounded pick-dedup** — `goal_dedup_max_hold_s` (new, 20.0s). Past that much *continuous*
   same-goal re-committing with no hop judged, the leg is circling rather than orienting and the
   pick registers for real. A duration rather than a count, deliberately: the observed loop
   re-committed only once per ~25s, so a small count bound would have taken many minutes to arm.

### The review catch that mattered (external plan review, verified in code before accepting)

The first draft let the reached check fire in **any** non-excluded state. `_enter("SETTLE")` resets
`_settle_t0`, **`_settle_ok = 0`**, `_settle_last_fid` and restamps the settle gate — and the drone
does not move during a settle, so the reached condition stays true and it would have re-fired every
tick, never completing the frame gate: **an infinite SETTLE hover, strictly worse than the stall
being fixed.** `_REACHED_EXCLUDED_STATES` therefore excludes `SETTLE`/`REPLAN` (plus the TRIM pair,
recovery and postlude sets). Verified by reverting the exclusion: the regression test flips to FAIL.

## Verified

New tests, each **proven to catch its defect** by reverting the fix and confirming a FAIL (session
39/44 practice), then restoring:

| Revert | Test that flipped to FAIL |
|---|---|
| PARALLAX_PUSH grace | `forced hop survives PARALLAX_PUSH=False` |
| SETTLE exclusion | `SETTLE-trap avoided=False` |
| dedup bound | `dedup bounded past it=False` |
| state-independent reached | `reached retires from any parked state=False` |
| planner too-close | all four `(opt3)` planner tests FAIL |

Also covered: a genuinely far goal is untouched; a blind (`plan_valid=False`) tick never retires a
goal off a frozen pose; a slow push with NO grace window still holds exactly as before; the dedup
still suppresses within its window; all-candidates-too-close lands on the **corner tour**, not a
premature `done`.

Two pre-existing test fixtures had to move because they placed the goal at *exactly*
`goal_reach_dist` (the planner's commitment-switch test at 0.4 = its default; `_plan_be` at 1.0 =
config's). Both were re-pointed to genuinely-distant goals with the original intent preserved
(utilities recomputed for the switch_factor=1.5 hold-vs-switch assertions).

`python autopilot.py --self-test`, `frontier_planner.py`, `flight_replay.py`, `map_store.py`,
`ground_grid.py`: **ALL PASS, 0 failures.** `perception_worker.py`/`io_bridge.py` cannot self-test
in this environment (no torch / NDIlib); both were `py_compile`-checked, and the
`perception_worker` change is a 9-line drain with no new imports.

## Next

**LIVE-FLY** (`python fly.py` — full stack). Watch for:
- The drone no longer re-committing a goal it is sitting on; a `TOO-CLOSE goal=… d=… (already
  standing on it)` line from perception when it tries.
- A `goal already reached (d=…) while parked in <state>` line retiring a leg from a hold.
- `PARALLAX_PUSH` actually **translating** during a forced hop under slow SLAM, instead of bouncing
  to `SLAM_HOLD` in ~30ms.
- The Goals DB pick count on a hammered disc climbing past 1, and the loop blacklist actually firing.
- **Downstream:** TRIM should become able to fire again — it never did in that flight because its
  gate needs `not self._slam_slow` **and** `st in {SETTLE, ADVANCE}`, and SLAM ran 1300-2200ms with
  those states nearly never active. No TRIM change was made; if it still never fires once the drone
  is moving again, that is the next thing to look at.
- **Watch for over-rejection**: with `goal_reach_dist` 1.0, a tight room could see several
  candidates soft-rejected in a row and fall through to the corner tour earlier than before. It is
  round-scoped and self-healing, but worth eyeballing on the first flight.
