# Session 41 — telemetry panel: live distance-to-goal

## Origin

Operator ask, no diagnosed bug behind it: while watching the visualizer's telemetry panel (session
37) — FSM state, height, plan status — he wanted the drone's live distance to its current goal
visible too, only while the plan is actually valid (not during `PLAN-STALE`).

## Built

`render_telemetry_panel` (`visualizer.py`) already receives the full `TOPIC_PLAN` payload, which
already carries `pos` (`[x, z]` world-space drone position) and `goal` (`[x, z]` world-space chosen
frontier goal, `None` when there's no active goal, e.g. `DONE`) — both already consumed the same
way for the map panel's goal marker/clearance ray (`overlay_plan`). No new bus field, no
`perception_worker.py`/`autopilot.py` change: purely a display computation inside the existing
`plan_valid` branch. Added one line, `GOAL     dist=<value>u`, computed as
`np.hypot(pos[0]-goal[0], pos[1]-goal[1])`; prints `--` if either `pos` or `goal` is `None` (NO
SILENT FALLBACK — mirrors every other reading in this panel). Stale-plan branch is untouched, so
the new line simply doesn't render during `PLAN-STALE`, per the ask.

## Verified

No self-test scaffold exists for `visualizer.py` (session 37 established the pattern of an ad-hoc
smoke check instead). Ran `render_telemetry_panel` directly against four synthetic payloads:
valid-plan-with-goal, valid-plan-with-`goal=None` (the `DONE` case), `plan_valid=False` (stale),
and `control=None` (no-autopilot placeholder) — all four composed without error, and the `--`
fallback confirmed for the no-goal case. **NEXT = LIVE-FLY** (folded into the standing session
36-40 live-fly checklist, nothing new to add there beyond "confirm the GOAL line tracks distance
shrinking as the drone advances toward a frontier").
