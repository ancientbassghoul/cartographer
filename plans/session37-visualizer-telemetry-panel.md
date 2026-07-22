# Replace visualizer.py's disabled depth panel with live autopilot telemetry

## Context

The project is a monocular-SLAM autonomous-exploration drone stack (`io_bridge.py` <-> Unity,
`perception_worker.py` = SLAM + planning, `autopilot.py` = the flight FSM, `visualizer.py` = the
live read-only dashboard). The depth panel (bottom-left of the dashboard) has shown a "DEPTH
DISABLED" placeholder since DA-V2 depth was removed (2026-07-07) — `render_depth_panel()` in
`visualizer.py:79-86`.

The operator wants that dead space replaced with telemetry that's currently invisible during a
live flight:
- drone height (actual) + desired height
- plan status
- the current autopilot FSM state (`ADVANCE`, `TRIM`, `SETTLE`, ...)

**Why this data isn't already flowing to the visualizer:** `visualizer.py` only subscribes to
`perception_state_port` (:5603 — `pose`/`map`/`plan`/`target` topics published by
`perception_worker.py`). The FSM state string (`self.state` inside `ExploreController`, e.g.
`ADVANCE`/`TRIM`/`SETTLE`/`BACKOFF`/...) is never published there — it only rides
`TOPIC_CONTROL` on `autonomy_control_port` (:5606), which `autopilot.py`'s `run_explore()`
publishes every tick (20 Hz) via `_full_vector()` (`autopilot.py:88-99`) and which today only
`io_bridge.py` subscribes to (to actually drive Unity). Similarly the "desired height" —
`ExploreController.target_altitude_y` (the live-calibrated, self-latched altitude-lock hold
target, `autopilot.py:911`) — is a private attribute of the FSM object inside `run_explore()`
and is never published anywhere. The *actual* current height (`pos_y`) and plan status
(`plan_valid`, `done`, `n_frontiers`, `forward_clearance_dist`, `bearing_err`,
`n_blacklisted`) already ride the existing `plan` topic `visualizer.py` subscribes to
(`perception_worker.py:342-368`) — no change needed there.

ZeroMQ PUB/SUB supports multiple independent subscribers to one bound PUB port for free — this
is the exact pattern already used for the frame bus ("an extra subscriber is free and never
steals frames"). So the plan is: (1) add `target_altitude_y` to the payload `autopilot.py`
already publishes on `TOPIC_CONTROL`, and (2) have `visualizer.py` add a second, independent
`StateSubscriber` connected to `autonomy_control_port` to receive `state` + `target_altitude_y`
continuously, alongside its existing subscription for `pos_y`/plan status.

## Isolation: worked in a separate git worktree

Built on branch `worktree-visualizer-telemetry-panel` (worktree at
`.claude/worktrees/visualizer-telemetry-panel/`), branched from `leg-hops-and-goal-commit-fix`@
`801e2f8`, because another Claude session was concurrently working the main checkout on the
session-36 visual-recovery line of work. Merged `new-visual-recovery` (the visual-recovery
branch, once its work landed as a real commit) into this branch afterward via a clean
fast-forward + stash-pop (only conflict was this same paragraph in `PROGRESS.md`, both sessions
having written a "session 36" entry — resolved by renumbering this one to session 37).

## Changes

### 1. `autopilot.py` — publish `target_altitude_y` on `TOPIC_CONTROL`

- `_full_vector(active, seq, now, state)` (line 88) gains an optional trailing parameter:
  `_full_vector(active, seq, now, state, target_altitude_y=None)`, adding
  `v["target_altitude_y"] = (round(float(target_altitude_y), 4) if target_altitude_y is not None else None)`
  to the returned dict. All other existing call sites keep working unchanged (default `None`)
  since only `run_explore` (the autonomous explore loop, the only place `target_altitude_y`
  exists) needs to pass it.
- In `run_explore()`'s `publish(active, state)` closure, pass `ctrl.target_altitude_y` through
  to `_full_vector` — `ctrl` (the `ExploreController` instance) is already in the closure's
  scope. `None` before the first calibration latches it — matching `plan_valid`-style "not
  available yet" semantics already used elsewhere, not a silent fallback.

### 2. `visualizer.py` — new telemetry panel replacing the depth placeholder

- **New subscriber**: in `run()`, read `ctrl_port = cfg["network"]["autonomy_control_port"]`
  and open a second `frame_bus.StateSubscriber(ctrl_port, topics=[frame_bus.TOPIC_CONTROL])`
  alongside the existing `state_sub`/`frame_sub`. Drained to freshest each loop iteration the
  same way `state_sub` is drained, storing the payload on `dash.control`.
- **`Dashboard`**: `self.control = None` in `__init__`; a `"control"` branch in `update()` sets
  `self.control = payload` (topic string from `TOPIC_CONTROL` decodes to `"control"`, matching
  the existing `pose`/`map`/`target`/`plan` dispatch pattern).
- **`render_depth_panel` replaced** with `render_telemetry_panel(control, plan, w=PANEL_W,
  h=PANEL_H)`:
  - No data yet (`control is None`): same placeholder style as the other "waiting" panels
    (`"waiting for autopilot on the control bus..."`).
  - Otherwise renders, in the existing font/style (0.45+ scale `cv2.putText`, one line per
    reading): `STATE: <state>` (raw FSM state string, always shown verbatim — no hardcoded
    good/bad state list to keep in sync); `HEIGHT pos_y=.. desired=.. delta=..` (either side
    `--` when `None`, matching the existing `bearing_err`/`clear` `--` convention in
    `overlay_plan`); `PLAN valid/STALE done=.. frontiers=.. blacklisted=.. bearing_err=..
    clear=..` (the same `plan` payload fields `overlay_plan`/`render_status` already consume,
    just surfaced permanently instead of only on the transient map overlay text).
- `Dashboard.render()` swaps in `render_telemetry_panel(self.control, self.plan)`.
- Module docstring's layout diagram, the `PANEL_W`/`PANEL_H` comment, and the `argparse`
  description string updated (no longer say "depth").
- `self.depth`/`render_depth_panel` removed as dead code, along with the always-`None` `depth`
  parameter `render_status` carried only for signature symmetry.

## Verification

- `python autopilot.py --self-test`: ALL PASS (the new `_full_vector` parameter is additive;
  existing positional 4-arg call sites, including self-test helpers, stay valid).
- No `--self-test` exists for `visualizer.py` (pure display module, no prior self-test
  scaffold). Verified instead with a standalone script calling `render_telemetry_panel` /
  `Dashboard.update` / `Dashboard.render` directly against synthetic control/plan payloads
  (no-control placeholder, control-without-plan, a valid plan, a `PLAN-STALE` plan) — all
  composed without error at the expected panel shape.
- Not yet watched on a real flight (no hardware in this dev environment) — see PROGRESS.md
  "Next".
