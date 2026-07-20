# Session 33 — fix the goal-loop bug that survives its own blacklist (clearance-inset escapes exclusion)

## Origin

The operator flagged flight `20260721_005658`: the end of the timeline showed an endless "goal reached →
re-picked → reached again" loop, and the Goals DB showed the disc at `[-0.445, 4.582]` with **49 picks** —
despite the loop guard correctly firing and permanently blacklisting it after just the 3rd pick
(`LOOP-BLACKLIST goal=[-0.445, 4.582] picks=3 -> reselecting`, `t=00:58:49.163`).

## Diagnosis

Traced the root cause end to end in `frontier_planner.py`:

- `FrontierPlanner._select_reachable()` filters candidate frontiers by `_excluded()`, checking each
  frontier's **raw** centroid against the blacklist. It then calls `_choose()` to pick the best raw
  candidate, and runs that through the injected `_clearance_fn` (`ground_grid.inset_to_clearance`, wired up
  in `perception_worker.py`), which walks the goal **back toward the drone's current position** until it
  finds the first free, buffered cell. That adjusted point — not the raw frontier centroid — is what
  actually gets committed, published, and fed to `register_goal_pick` as `pick_goal`.
- **The exclusion check never re-ran against the adjusted point.** In this flight the drone was pinned in
  one spot with its entire reachable free space in that direction pinched into one small pocket — exactly
  where the already-dead frontier sat. Every "different" (technically not-excluded) raw frontier candidate
  the utility function turned up got inset right back onto that same dead cell. So the blacklist never
  actually stopped anything: each cycle, REPLAN committed a "new" goal that was really the same spot, and
  since it was already almost on top of the drone (`goal_reach_dist: 1.0` is generous) it was instantly
  "reached" → SETTLE → REPLAN again. Confirmed via the timeline: `plan_goal` drifted by millimeters cycle to
  cycle (`-0.4308, 4.5832` → `-0.4282, 4.5835` → `-0.4272, 4.5836` → …), and the Goals DB disc's `is_corner`
  flag stayed `False` the entire time — ruling out the corner-sweep tour (`_pick_sweep_corner`, which
  *deliberately* bypasses `_excluded` — a different, intentional escape hatch, not this bug) as the source.

This was a genuine defect in the loop/2-bump/stall blacklist's guarantee: it only ever protected against
re-selecting a dead **raw frontier**, never against the **committed, clearance-adjusted** goal landing back
on one.

## Built

**`frontier_planner.py`** — rewrote `_select_reachable()`: after the clearance-inset adjustment, if the
adjusted goal is itself `_excluded()`, the candidate is dropped and `_choose()` retries on the remaining
non-excluded frontiers, looping until either a genuinely clear candidate is found or the list is exhausted
(returns `None`, exactly as if nothing had been reachable that cycle — `select()`'s existing corner-sweep /
done fallback takes over unchanged). Bounded: each pass removes at least the rejected candidate. No change
to `_choose()`, `_excluded()`, or the corner-sweep path.

**`perception_worker.py`** — defense in depth (visibility, not a fallback): at the `register_goal_pick` call
site, if the picked goal is *still* `is_excluded()` at pick time, a structured `WARNING: pick landed on an
ALREADY-excluded goal...` message is pushed through `pipe.last_planner_event` — the SAME mechanism
`LOOP-BLACKLIST`/`CORNER-GIVEUP`/`BUMP` events already use — so it shows up as a `planner_event` in both the
console log and the timeline/`flight_replay.py` debugger, not just a bare `print()`. After the fix above
this should never fire; if it ever does, it's an immediate, loud signal that some other path is bypassing
exclusion, instead of quietly repeating this same 49-pick loop.

## Self-tests

Added two cases to `frontier_planner.py`'s self-test, next to the existing `(opt) clearance inset` tests:
- `(opt2)` a permanently-blacklisted "dead" point plus two frontier candidates, one whose injected
  `clearance_fn` collapses onto the dead point (would normally win on utility) and one that stays genuinely
  clear — `select()` must drop the first and commit the second, never the dead spot.
- `(opt2)` when the ONLY reachable candidate's inset collapses onto the dead spot, `select()` must report no
  commit (falls through to `done`) rather than ever committing there.

`python frontier_planner.py --self-test`, `python autopilot.py --self-test`, `python perception_worker.py
--self-test`, and `python ground_grid.py --self-test`: **ALL PASS**.

## Verification

No hardware/GPU in this environment — self-tests only. Live-fly checklist for next flight: does a drone
pinned into a corner/pocket with no genuinely reachable free space now correctly fall through to the
corner-sweep tour (or `STUCK`/`done`) instead of looping on a dead goal; does the Goals DB's pick count for
any blacklisted disc stay flat once it's permanently blacklisted (not keep climbing like the 49-pick case);
watch the console/replay debugger for the new WARNING event — it should never print.
