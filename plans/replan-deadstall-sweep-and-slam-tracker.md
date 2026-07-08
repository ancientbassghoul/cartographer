# Fix the REPLAN dead-stall (bbox diagonal sweep) + move SLAM tracker into the replay HTML

_Session-9 plan. Designed, not yet built. Resume here for item 2._

## Context

The session-8 flight (`20260708_195009`) ended by "doing nothing in a loop." Root cause (the known
**item 2** dead-stall): at `19:57:06` perception's planner returned `goal=None, done=False`. The
autopilot's REPLAN branch (`autopilot.py:1401-1403`) has **no handling for that terminal case** — it
deliberately idles ("frontiers forming / verification choosing a corner"), assuming it is transient. It
wasn't: the drone hovered ~54 s with no translation, SLAM drifted, the plan went `PLAN-LOST` →
`HOLD_LOST` (indefinite hover).

Why the planner returned `goal=None`: the **done-verification stage exists but was silently bypassed.**
`select()` only *starts* verification when the transition gate (`frontier_planner.py:313`) passes —
`farthest_free` non-None, not excluded, **and > `verify_min_dist` (0.6) away**. That gate failed, so
`select()` fell through to the silent-idle branch (`frontier_planner.py:332`: `if frontiers: return
None, False`) — neither verifying nor declaring done. A silent do-nothing, which also violates the
project's NO-SILENT-FALLBACK rule.

**Operator's direction (supersedes the "room was fully mapped" read):** the room is only *mildly*
mapped — borders fine, interior under-covered — and the `farthest_free` / `verify_min_dist` ("too near")
machinery is too fragile. Replace it with a deterministic **bounding-box diagonal sweep**: take the room
bbox, find the corner nearest the drone, target the **opposite corner inset 1 unit on each axis** (so
it's reachable), and fly there; if the traverse surfaces no new frontier, declare **DONE** (just print
it — no Phase-3 handoff yet). Also: **move the `[SLAM_TRACKER]` stream out of the terminal into the
replay HTML**, in a distinct color.

Intended outcome: the explorer never idles forever — it either flies a full interior diagonal that
exposes more of the room, or declares a clean, visible DONE; and the SLAM per-pose stream lives in the
visual debugger, not the terminal.

---

## Part 1 — Replace fragile verify with a bounding-box diagonal sweep

### 1a. `ground_grid.py` — add `sweep_corner(pos, inset=1.0)`
New method returning the world `[x, z]` sweep target:
- bbox from **known cells** (`free ∪ occ` from `classify_dense()`), converting `ix0/iz0` + shape to
  world extents the same way `farthest_free` does (`(idx + origin + 0.5) * self.cell`).
- Four bbox corners; pick the one **nearest** `pos`; the **opposite** (diagonal) corner is the raw
  target. Inset each axis **independently** toward the box center via a per-axis helper:
  - if `span_axis >= 2*inset`: `t = far - inset` (i.e. `x_max - inset` when the opposite corner is at
    `x_max`, else `x_min + inset`).
  - if `span_axis < 2*inset` (axis too narrow for a dual-side stand-off — e.g. a corridor's short
    axis): **clamp to that axis's midpoint** `(min+max)/2` (maximally clear of both walls). This
    prevents the overshoot bug where `x_min + inset` on a 1.5 m span pushes the target past `x_max`
    into unmapped territory.
- Return `None` only when **both** axes are degenerate (`span < 2*inset` on X **and** Z → box too small
  to sweep) or there are no known cells. A corridor (one wide axis) still yields a valid target: inset
  along the long axis, midpoint on the short axis. (`inset` is a general stand-off-scale param, not a
  room answer — same class as the existing `reposition_inset` / `goal_reach_dist`.)

### 1b. `frontier_planner.py` — route "nothing reachable" through the sweep target
Rework `select()` (lines ~273-334) and rename the verify state to a **sweep** target:
- Keep normal frontier selection first (`_select_reachable`) unchanged.
- When nothing is reachable: use the passed-in **sweep corner** (from perception, see 1c) as the
  frozen, cached target (reuse the existing `verifying` / `verify_target` "cache once, stays static
  while flying it" mechanism — rename to `sweeping` / `sweep_target`). Return it as `goal, done=False`.
- **Delete the fragile gates:** the `verify_min_dist` "too near" skip and the `farthest_free`-argmax
  reliance. The only terminal outcomes now:
  - a reachable frontier appears mid-sweep → abandon the sweep, resume selection (keep existing
    "resume" behavior; covers the c3 self-test).
  - sweep target **reached** (`_d(pos, sweep_target) <= goal_reach_dist`) with still no reachable
    frontier → `done=True`.
  - sweep target **blacklisted** (2 bumps → unreachable wall) and no other sweep corner → `done=True`.
  - no sweep corner available (degenerate bbox) → `done=True`.
- Guarantee: `select()` **never** returns `goal=None, done=False` as a resting state (only a momentary
  startup tick before the first frontiers form, which the autopilot guard in 1d bounds).
- Keep the 2-bump blacklist, `_whitelist_round`-on-arrival retry, commitment, and utility logic as-is.
- Update the self-tests (cases c, c2, c3, c4, i, A1-A3) to the sweep semantics (opposite-corner target
  instead of `farthest_free`-argmax; drop the "too near → done" case c4-near).

### 1c. `perception_worker.py` — compute + pass the sweep corner
In `_plan_payload` (around lines 373-384): when `not any_reachable(fr)`, compute
`sweep = self.ground.sweep_corner(pos, inset=<sweep inset>)` and pass it to `planner.select(...)` in
place of `farthest`. Keep the one-shot `[perception] planner: … SWEEPING via corner …` log (rename from
the verify log) and the `done` transition log.

### 1d. `autopilot.py` — kill the silent idle; visible DONE
- REPLAN branch (lines 1366-1403): the `plan.get("done")` path already routes to `DONE`
  (1368-1371) — keep it. **Remove/replace the silent infinite-idle comment path (1401-1403):** if a
  `goal=None, done=False` plan persists in REPLAN beyond a **bounded window** (a general robustness
  timeout, e.g. `no_goal_idle_s`, NOT a room value), emit a visible log + telemetry flag and hold —
  never a dark infinite idle. With the new planner this should essentially never fire (goal=None ⇒
  done), so it is a fail-visible backstop, consistent with NO SILENT FALLBACK.
- DONE state (line 1609): add a **one-shot** `[autopilot][explore] EXPLORE COMPLETE — <n_frontiers>
  frontiers, sweep traverse found nothing new -> DONE (hover)` log + a `done` telemetry flag; then hold
  neutral as today. No Phase-3 handoff (per operator: "just print DONE").

---

## Part 2 — Move `[SLAM_TRACKER]` from the terminal into the replay HTML (distinct color)

### 2a. `autopilot.py` (lines 1729-1755)
- **Remove** `print(sline, flush=True)` and the `diag.line(sline)` terminal/.log writes.
- Instead emit a compact **timeline record** for each accepted pose via `diag.timeline({...})` — e.g.
  `{"t_wall", "t_mono", "ev_kind": "slam", "slam": "<dx/dy/dYaw> [mode] - Latency <gap> (build <ms>)",
  "frame_id", "slam_ms"}` — so it lands in `*_timeline.jsonl` for the replay. Keep it gated on a new
  `frame_id` (once per publish), as now.

### 2b. `flight_replay.py`
- Add CSS class `#events .slam { color: #2fd4d4; }` (teal — distinct from the existing plan-purple,
  miss-amber, blacklist-red) near lines 111-117.
- In the JS event-log builder, render records whose `ev_kind === "slam"` (or that carry a `slam` field)
  with the `slam` class, using the `slam` string as the message. They interleave chronologically with
  the `[autopilot][explore]` state events by `t_mono`.

---

## Part 3 — Next-phase notes (NOT built here; for the operator's decision)

The operator's idea: a **ground-level interior mapping pass**, then fly those inner paths **with SLAM
off + detection on**. Recommendation (also recorded in `PROGRESS.md` Future):

- The GPU-time split is already the documented plan ("Map mode = SLAM only; a future Scan mode pauses
  SLAM to run the cascade"). A dense **low-altitude interior traverse** to enrich geometry is worth it —
  more parallax + coverage of actual room contents improves the localization map.
- **But a blind SLAM-off flight can't hold position** (no pose feedback → open-loop drift). Prefer one
  of:
  - **(a) Offline detection on the recorded map-mode footage** — the cascade already runs on recorded
    hi-res frames (Phase-1 E2E did exactly this); no second flight, no GPU contention. Simplest, and it
    reuses the map-mode poses for 3D lift.
  - **(b) Temporally-interleaved Scan mode** — SLAM navigates/relocalizes → pause SLAM → run the
    cascade on a captured hi-res frame → resume SLAM. This is the already-deferred "Scan mode
    (360° cascade with SLAM/GPU temporal separation)."
- Start with (a) to validate detection→3D-lift on the interior footage before investing in (b).

---

## Files to modify
- `ground_grid.py` — add `sweep_corner()`.
- `frontier_planner.py` — sweep-target logic in `select()`; rename verify→sweep state; update self-tests.
- `perception_worker.py` — compute/pass sweep corner in `_plan_payload`.
- `autopilot.py` — REPLAN bounded-idle backstop + DONE one-shot log/flag; remove SLAM_TRACKER terminal
  print, emit timeline record.
- `flight_replay.py` — `.slam` CSS + render `ev_kind:"slam"` records in the event log.
- `PROGRESS.md` — collapse item 2 into the "tried that" narrative; record Part-3 next-phase notes;
  update Next/Milestones.

## Verification
1. Offline self-tests (must pass before any live fly):
   `venv\Scripts\python.exe frontier_planner.py --self-test` (updated sweep cases),
   `ground_grid.py --self-test` (add `sweep_corner` assertions: square-bbox opposite corner with
   1-unit inset; **corridor case** — narrow axis clamps to midpoint, wide axis insets normally, target
   stays inside bounds; both-axes-degenerate → None), `autopilot.py --self-test`.
2. Regenerate the replay from the existing timeline to confirm the HTML still renders:
   `python flight_replay.py OUTPUT/diag/20260708_195009_timeline.jsonl --open`.
3. Live fly (`Xlab.exe` → io_bridge → perception `--no-display` → visualizer → `autopilot.py --explore
   --log`, press `m`): confirm that when frontiers run out the drone flies the diagonal to the
   opposite-corner sweep target (not an idle hover), and either resumes on a new frontier or prints
   `EXPLORE COMPLETE -> DONE`. Open the new `*_timeline.html`: the `[SLAM_TRACKER]` lines appear in the
   event log in teal, and the autopilot **terminal** no longer prints them.
