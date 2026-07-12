# All-corners verification sweep + post-mission floor-dock postlude (+ SLAM/parallax backlog)

_Session-10 plan. **Parts A + B BUILT (session 10), all offline self-tests green, live-fly pending.**
The two Deferred items (SLAM-loss investigation, parallax-strafe on turns) are NOT built. Referenced from
PROGRESS.md._

## Context

Session-9's flight was good, but reconstruction is **uneven**: the drone flew one main diagonal, so
occupancy is dense near that line and thin at the two off-path corners
(`DEBUG_IMAGES/mission_complete__mapping_so_so.png`). Done-verification currently flies to ONE opposite
corner (`ground_grid.sweep_corner`) then declares done. The operator wants (A) the verification stage to
**tour ALL room corners** (opposite first, then the farthest unvisited, then the last) so every corner
reconstructs well, and (B) after the tour, a **post-mission floor-dock postlude** instead of a hover at
mapping height: fly home to the take-off origin, descend to the floor, nudge up to a low stand-off, then
stand by.

Two operator observations are **captured for later, NOT built here** (Deferred): the plan is lost a lot
(SLAM choking?), and after a turn SLAM often drops before any parallax — a small strafe to complement each
turn may help. Build A + B first; then discuss the Deferred items.

## Part A — all-corners verification tour

Generalize the single-corner sweep (built session 9: `frontier_planner.select` +
`ground_grid.sweep_corner` + `perception_worker._plan_payload`) into a corner **tour**.

### A1. `ground_grid.py` — `bbox_corners(inset=1.0)` (replaces `sweep_corner`)
Return the (up to) 4 inset corners of the known bbox (`free ∪ occ`), each pulled `inset` toward center
per axis, reusing `sweep_corner`'s per-axis logic: an axis narrower than `2*inset` collapses to its
**midpoint** (a corridor → 2 end-corners), corners **deduped**, canonical order (SW, SE, NW, NE). `[]`
when no known cells. Remove `sweep_corner`; fold its self-test into `bbox_corners` tests.

### A2. `frontier_planner.py` — multi-corner tour in `select()`
- Signature: `select(frontiers, pos, heading_deg=None, sweep_corners=None)` (a LIST of corners).
- New state `self._swept_corners = []` (positions of corners already reached/retired; persists for the
  flight, self-corrects if the bbox grows beyond `assoc_dist`). Keep `sweeping` / `sweep_target`.
- Helpers:
  - `_corner_visited(c)` = any stored pos within `assoc_dist` of `c`.
  - **`_pick_sweep_corner(corners, pos)` = the FARTHEST-from-`pos` corner with `not _corner_visited(c)`
    — and NOTHING ELSE. It MUST NOT consult `_excluded()`. Corner targets are never suppressed by old
    frontier blacklists** (operator's explicit requirement). Farthest-first ⇒ opposite corner first,
    then the far one of the rest, then the last.
- Rework the "nothing reachable" branch:
  - **Auto-mark**: any corner within `goal_reach_dist` of `pos` → append to `_swept_corners` (covers the
    start corner + any passed).
  - If `sweeping` and NOT reached → keep returning `sweep_target`.
  - If `sweeping` and reached → mark it visited, clear target, do the existing whitelist-round + frontier
    retry from this vantage; then fall through to pick the next.
  - **Pick next** via `_pick_sweep_corner`; if one exists → cache + return (`sweeping=True`). If none →
    whitelist-retry once (not on a just-blacklisted tick); else `done=True` if `_ever_had_frontiers or
    _swept_corners`, else the startup transient idle.
- **Unreachable-corner retirement WITHOUT `_excluded` filtering**: in `note_wall_hit`, when a bump
  2-bump-blacklists a region matching the current `sweep_target` (within `assoc_dist`), mark that corner
  **visited** (append to `_swept_corners`) and clear `sweeping`. A corner the drone actively fails to
  reach during THIS tour is retired via the event-driven bump — never via a stale blacklist filter. This
  preserves termination while honoring "corners ignore `_excluded`".
- Invariant preserved: never a resting `goal=None, done=False` except the startup tick.

### A3. `perception_worker.py`
When `not any_reachable`, compute `corners = self.ground.bbox_corners(self.reposition_inset)` and pass
`sweep_corners=corners`. Keep the one-shot `SWEEPING to corner …` log (name the target + corners left).

## Part B — post-mission floor-dock postlude (`autopilot.py` + `flow_contact_detector.py`)

When the tour is fully exhausted (planner returns `done=True`), REPLAN routes to the postlude instead of
a static DONE hover: **RETURN_TO_ORIGIN → DOCK_FLOOR → LOW_STANDOFF → DONE**.

### B1. FLOOR contact detector (`flow_contact_detector.py`)
Add a `CMD_DOWN` mode that mirrors CEILING for downward motion: signal = `|dy_med|` (vertical flow), ref
`_ref_down`, event kind `"FLOOR"` — the downward vertical flow collapses to ~0 at floor contact. Add the
`CMD_DOWN` constant, the `_signal_cfg`/`update` cases, `_ref_down` init, and a self-test (descending flow
collapses → FLOOR). Update `autopilot._detector_command`: `joy_vertical > 0` → `CMD_DOWN`. In the explore
loop's verdict block (autopilot.py ~1961), derive `floor_contact` (v.contact & kind FLOOR & command
CMD_DOWN) and pass it into `ctrl.step(...)` (new `floor_contact=False` param).

### B2. New states in `ExploreController.step`
- **`RETURN_TO_ORIGIN`** — home to the take-off origin `[0, 0]` (SLAM frame) at the current mapping
  height. Self-contained mini orient+advance loop: compute bearing to `[0,0]` from `plan.pos`/`heading`,
  turn in ≤`turn_step_deg` steps (reuse `_build_turn`), advance with the forward preset + the **clearance
  stand-off** + altitude lock. Reach → `DOCK_FLOOR`. Bounded: a homing time cap / repeated wall-block →
  log "couldn't reach origin, docking here" → `DOCK_FLOOR` (NO SILENT FALLBACK).
- **`DOCK_FLOOR`** — a controlled **stepped/pulsed** descent that MIRRORS the gentle two-phase ascent.
  **A continuous hold-down is FORBIDDEN**: rapid downward acceleration stretches vertical visual features
  and chokes SLAM right at mission end. Reuse the ascent pattern DOWNWARD — a short DOWN micro-pulse
  (`joy_vertical: +1`), then PAUSE/settle to let SLAM catch its breath and read the pose, verify
  `floor_contact`, and repeat until the FLOOR latch OR the `dock_max_s` safety cap (log + proceed).
  Parameterize the ascent knobs for descent (`dock_pulse_s` / `dock_rest_s`, analogous to
  `ascend_micro_pulse_s` / `ascend_rest_s`). → `LOW_STANDOFF`.
- **`LOW_STANDOFF`** — a short UP nudge (`joy_vertical: -1`) for a tunable `floor_standoff_nudge` (a
  general platform behavior param — a duration or a small pulse count) to clear the ground safely.
  → `DONE`.
- **`DONE`** — one-shot log `EXPLORE COMPLETE -> STANDBY AT LOW HEIGHT`, hold neutral (as today).
- REPLAN done-branch: `_enter("RETURN_TO_ORIGIN")` instead of `_enter("DONE")`. Config knobs
  (`autonomy.explore`): `floor_standoff_nudge`, `dock_pulse_s`, `dock_rest_s`, `dock_max_s`,
  `home_reach_dist`, `home_max_s`.

### B3. Self-tests
- `flow_contact_detector`: a descending stream whose `|dy_med|` collapses → one FLOOR contact.
- `autopilot`: drive `done=True` → visits `RETURN_TO_ORIGIN → DOCK_FLOOR → LOW_STANDOFF → DONE`, with
  the pulsed down-command in DOCK_FLOOR, the up-nudge in LOW_STANDOFF, floor_contact/timeout terminating
  each stage.

## Difficulties / considerations
- **Corners ignore blacklists** (A2): termination relies on the fresh 2-bump retirement of the *current*
  corner (not `_excluded` filtering), so a genuinely walled-off corner still ends the tour.
- **Bbox grows** → corner drift; visited-match uses `assoc_dist`; growth beyond it re-visits (acceptable).
- **Homing to origin** with only "fly-toward-aim" control = a multi-leg turn/advance; the clearance
  stand-off keeps it off walls, a time cap bounds it.
- **FLOOR detection is NEW and unvalidated** (unlike CEILING/WALL which were flight-validated). The
  `dock_max_s` cap is the fail-safe; watch the first live dock closely. De-risk fallback if it never
  latches: a fixed pulsed-descent count instead of flow detection.
- More legs/turns (tour + homing) = more SLAM-loss chances → ties into the Deferred items.

## Deferred (captured — NOT built now)
1. **Plan lost too often (SLAM choking?)** — investigate from logs: `slam_ms` spikes (compute
   contention) vs. turn-induced tracking loss vs. the ~2 Hz pose rate. Quantify frequency + which
   maneuvers (esp. turns). Decide mitigations. Investigation only, no code yet.
2. **Parallax-strafe on turns** — after an open-loop turn SLAM often drops before the drone translates,
   so the parallax push never gets a chance. Add a small `joy_horizontal` STRAFE alongside each turn for
   translational parallax so SLAM keeps tracking through the rotation. Design AFTER Parts A + B are
   confirmed good live.

## Files to modify
- `ground_grid.py` — `bbox_corners()` (remove `sweep_corner`), self-tests.
- `frontier_planner.py` — `_swept_corners` + `_corner_visited`/`_pick_sweep_corner` (no `_excluded`),
  multi-corner `select()`, corner retirement in `note_wall_hit`, self-tests.
- `perception_worker.py` — pass `bbox_corners` as `sweep_corners`.
- `flow_contact_detector.py` — `CMD_DOWN`/FLOOR mode + self-test.
- `autopilot.py` — `_detector_command` DOWN, `floor_contact` derivation + `step` param, the
  RETURN_TO_ORIGIN/DOCK_FLOOR(pulsed)/LOW_STANDOFF states, REPLAN done→postlude, config knobs, self-tests.
- `config.yaml` — `floor_standoff_nudge`, `dock_pulse_s`, `dock_rest_s`, `dock_max_s`, `home_reach_dist`,
  `home_max_s`.

## Verification
1. Offline self-tests (all pass): `ground_grid.py`, `frontier_planner.py`, `flow_contact_detector.py`,
   `autopilot.py`, `flight_replay.py` `--self-test` (run from `venv`).
2. Live fly (`Xlab.exe` → io_bridge → perception `--no-display` → visualizer → `autopilot.py --explore
   --log`, `m`): when frontiers exhaust, confirm the tour visits opposite → farthest-unvisited → last
   corner; then the postlude homes to origin, descends to the floor in gentle pulses (FLOOR latch),
   nudges up, and logs `STANDBY AT LOW HEIGHT`. Open the `*_timeline.html`: the path reaches all four
   corners and ends at the origin near the floor; occupancy thickens at the previously-thin corners.
