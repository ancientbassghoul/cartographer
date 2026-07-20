# Session 28 — TRIM resume settle-gate + blacklist re-validation, clearance-fan hit-fraction vote

## Origin

The operator flew `20260720_135307` on the session-27 build ("exceptional apart from one bug that put the
drone into a loop") and asked for three things to be diagnosed off the raw `_timeline.jsonl` (never the
console `.log`), per the project's established method: read line-by-line, form a hypothesis, verify it
against actual field values before presenting it.

## Bug 1 — goal_db picks/strikes/bumps jumping together at 14:13:38.995 (EXPLAINED, no code change)

At 14:13:38.995 the goals-DB disc for corner `[4.65, -4.45]` jumped `picks 3->4`, `strikes 1->0`,
`bumps 1->2` (triggering a PERMANENT 2-bump blacklist) all in one telemetry tick, and the console-style
`planner: BUMP ... -> BLACKLIST PERMANENT` line then appeared 33 times in a row (14:13:38.995 ->
14:13:39.780).

Root cause, verified against the diag `slam_start`/`slam_finish` records: two consecutive SLAM solves took
**10.48s** (frame_id 70624) and **9.13s** (frame_id 71296) back to back — perception was blind for ~19.6s
straight. `perception_worker.py`'s `run_live()` main loop is **fully synchronous**: it drains
`TOPIC_AUTOPILOT_EVENT` then calls `pipe.step()` (the blocking SLAM solve); `pipe.step()` is the ONLY thing
that publishes a fresh `TOPIC_PLAN` (carrying `goal_db`/`planner_event`). While blocked, perception can't
drain new pulses or publish anything — meanwhile the autopilot kept running in real time and legitimately
fired a REPLAN-judged hop-progress pulse (14:13:17.767, fresh SLAM-confirmed pos: `strikes 1->0`,
`picks 3->4`) and a live flow-based WALL-contact bump (14:13:19.388, `bumps 1->2` -> 2-bump PERMANENT
blacklist) — both correctly queued and applied to `self.planner` the instant `pipe.step()` finally
returned, 19s later, batching two genuinely independent real-time events into one visible tick. The
33x-repeated BUMP line is the same mechanism at smaller scale: `frame_id`/`cap_ts` were IDENTICAL across
all 33 timeline rows (perception published that plan exactly once; the autopilot just re-logs its
still-held `last_plan` on every one of its own ~30Hz control ticks until perception's next, much slower
publish arrives ~0.8s later).

**Verdict:** the blacklist DECISION was correct in both instances; only its *visibility* was
batched/delayed by perception's synchronous solve loop. This is almost certainly also the root cause
enabling bug 3's "can't react while blind" class of problem below — but making SLAM solving async is a
much bigger architectural change, out of scope for this session.

## Bug 2 — drone keeps flying toward the just-blacklisted goal (FIXED)

`self.leg_goal` (autopilot's committed goal) never changed for the rest of the flight after the blacklist
— confirmed unchanged in the timeline for ~2 more minutes. It is only ever refreshed inside a REPLAN
transition, and the flight never reached REPLAN again.

Trace: the one queued chance (`self._settle_to = "REPLAN"`, set at `autopilot.py:2235` when SLAM_HOLD's
status recovers to OK) got preempted by the whitelisted-state height-TRIM trigger (`autopilot.py:2301-2324`,
fires on SETTLE/ADVANCE), which hijacked the very SETTLE that was about to resolve to REPLAN. TRIM's
ring-gate read itself boxed in (fwd+back+sides all "blocked" — see the clearance-vote fix below for why
that judgment itself may have been unreliable) and aborted via `_trim_exit()`, whose own docstring said
plainly: *"RESTORE the committed goal snapshotted on entry ... never a fresh planner pick."* It blindly
re-committed `leg_goal` back to the corner that had, moments earlier, been permanently 2-bump-blacklisted —
because the snapshot (`_trim_resume_goal`) predated the blacklist. SLAM then died for good ~4s later, so no
further REPLAN ever arrived to correct it.

`_trim_resume_goal` was the ONLY place in the codebase with this restore-a-stashed-goal-without-revalidating
pattern (`_step_calib_lost`/`_blind_contact_backoff` both resume to a STATE, not a stashed goal, and
naturally flow through the normal SETTLE->REPLAN convergence).

A follow-up question sharpened the fix: **why isn't a REPLAN-class transition conditioned on a SLAM frame
captured strictly after the last command, the way the session-24 settle-gate already requires for the
normal SETTLE->REPLAN path?** Answer: it already is, for the normal path — but `_trim_exit()`'s abort
branch bypassed that gate entirely, computing its re-aim bearing off whatever `plan.get("pos")` happened to
be sitting in the CURRENT snapshot and jumping to ORIENT the same tick.

### Fix

`autopilot.py`:
- `_trim_exit(now, plan, msg)` no longer resolves instantly. It stashes the exit message
  (`self._trim_exit_msg`), calls `self._settle_gate_begin(now)` (the existing session-24 freshness+dwell
  primitive), and enters a new state `TRIM_RESUME_WAIT`.
- New `elif st == "TRIM_RESUME_WAIT":` branch (in `step()`, right after the `TRIM` block): holds neutral
  until `self._settle_gate_poll(now)` clears, then calls the new `_trim_resolve_resume(now, plan)`.
- New `_trim_resolve_resume(now, plan)`: re-checks `_trim_resume_goal` against the NOW-current
  `plan.get("blacklist")`/`blacklist_permanent` (already-published live data, `autopilot.py:130-131`)
  within `self.goal_area_radius` (existing 0.5u knob). If the goal is alive and pos/heading are available,
  restores it and re-aims ORIENT exactly as before — just off a provably fresh, post-TRIM pose. If the goal
  died (permanently blacklisted) or pos/heading are unavailable, falls through to
  `self._settle_to = "REPLAN"; self._enter("SETTLE", now)` — the SAME convergence a genuinely new leg uses.
  A goal that is only *soft*-blacklisted (this round's exclusion, not permanent) is still restored —
  only a permanent kill should stop the restore.
- This fallback also naturally preserves Trap B's original intent ("TRIM must not pollute the goals-DB")
  for the common case where the goal survives unchanged: REPLAN's own session-24 pick-dedup
  (`same_goal_as_last_pick`, `autopilot.py:2810-2820`) suppresses the pick when the freshly-read
  `plan.get("goal")` matches the preserved one, since `_hop_start_goal` was already cleared at TRIM entry.

Self-tests added to the existing HEIGHT-TRIM block: (e) ring-blocked abort now gates through
`TRIM_RESUME_WAIT` before resolving to ORIENT; (f) the full climb path likewise; (g) a preserved goal that
gets permanently blacklisted while `TRIM_RESUME_WAIT` is pending falls through to SETTLE->REPLAN instead of
being restored; (h) a *soft* blacklist on the same goal still restores it normally. All pass —
`python autopilot.py --self-test` is fully green.

## Bug 3 — plan-stale -> fallback -> spin -> stuck against the wall (DOCUMENTED ONLY, no fix)

Per the operator's explicit request: no fix until they have a working visualizer clip to verify an
orientation-during-"lost" observation. Two findings recorded for that session:
- The stuck-detector's inputs (`_hop_start_goal`/`_hop_start_dist`, `_settle_gate_poll`) are all keyed off
  the last SLAM-confirmed pose — the same blind spot as bug 1, so a real physical spin between frozen SLAM
  ticks is invisible to it by construction (supports the operator's hypothesis (a): the stuck-detector
  didn't fire because the plan never actually went "lost" from its perspective, or genuinely can't see a
  blind spin).
- The visualizer's heading arrow (`visualizer.py:188-189`) is driven by `plan["heading_deg"]` — the SAME
  field feeding the timeline log — so if perception really is synchronously blocked during a
  PLAN-LOST/HOLD_LOST stretch (bug 1), that arrow should be frozen too, not showing live orientation. What
  looked like "correct orientation" during "lost" was very likely the raw NDI video panel (visual judgment
  of the room), not a computed signal — meaning hypothesis (b) would need NEW SLAM-independent
  instrumentation (natural home: `flow_contact_detector.py`, already used for wall/backwall contact) rather
  than reusing an existing field. To be confirmed against the actual clip before any design is finalized.

## Fix 2 — clearance(): minimum hit-fraction vote before calling a direction blocked

While investigating whether TRIM's "ring blocked fwd+back+sides" judgment (bug 2) was trustworthy, traced
`map_store.py`'s `clearance()`: it casts a fan of rays (config.yaml: `clearance_fan_n: 10`,
`clearance_fan_deg: 8.0` -> 10 rays across a ±8°/16°-total cone) and returned the **MIN (nearest) hit
across the whole fan** — ONE ray touching one persistently-observed (`min_count=2`) but spatially-isolated
voxel was enough to call an entire direction "blocked," with no vote/fraction check at all. The operator
independently flagged this exact gap after reviewing the (locally messy/sparse) reconstructed point cloud.

### Fix

`map_store.py`, `clearance()`: now collects ALL ray hits (not just the min), then a direction counts as
blocked only once `len(hits) >= min_hit_fraction * fan_n`; below that, too few rays confirm a wall to trust
it over sparse-reconstruction noise, and the direction reads OPEN. New parameter `min_hit_fraction: float
= 0.0` — default preserves EXACT prior behavior (any single hit still counts), so nothing regresses
silently for an un-updated caller. When a direction IS judged blocked, the reported distance is still
`min(hits)` — same conservative distance as before, just gated by the vote first.

Wired a new `clearance_min_hit_fraction` config knob (`perception_worker.py` alongside
`clearance_fan_n`/`clearance_fan_deg`/`clearance_min_count`; `config.yaml` under `autonomy.explore`,
starting value **0.3** — the operator's own "less than a third -> probably open" framing). Passed through
both call sites (the forward stand-off AND the 8-direction ring), so ONE knob covers TRIM, PARALLAX_PUSH,
and the forward-cruise stand-off together, since they all share this one function.

This is a genuine tradeoff against the reason MIN-over-fan was originally chosen (protects against a
thin/off-center wall a single ray could "thread"). **Watch live for BOTH directions of failure**:
false-opens (ramming a genuinely thin wall) and whether the false-blocks the operator observed actually go
away.

New `run_self_test()` added to `map_store.py` (it had none before) + a `--self-test` CLI flag, mirroring
the pattern used in `frontier_planner.py`/`autopilot.py`. Cases: (a) one isolated but min_count-qualified
hit among a 5-ray fan, `min_hit_fraction=0.3` -> ignored (open); (b) a second hit on a different ray closer
in -> blocked, reported distance is still the MIN of the confirming hits; (c)/(c2) default (or omitted)
`min_hit_fraction=0.0` -> single-hit-blocks behavior unchanged (regression guard). All pass —
`python map_store.py --self-test` is fully green.

## Verification

Self-tests only — no hardware/GPU in this environment to live-fly. `python autopilot.py --self-test`,
`python map_store.py --self-test`, `python perception_worker.py --self-test`, `python frontier_planner.py
--self-test`, and `python flight_replay.py --self-test` are all green as of this session.

**Live-fly checklist for next flight** (folds into the existing sessions 20b-27 checklist):
- After a goal gets 2-bump-blacklisted, does the drone reach REPLAN and re-target promptly instead of
  riding the dead goal — watch for the (new) `TRIM_RESUME_WAIT` state in the replay debugger and a
  `... -> settle -> replan (preserved goal ... was blacklisted while trimming)` event line if TRIM happens
  to interrupt right after a blacklist.
- Does TRIM/ring-blocked judgment stop false-firing on sparse point-cloud noise (fewer
  `"ring blocked fwd+back+sides -> skip trim (pray)"` events where the room clearly has room), AND does it
  still correctly stop for a genuine thin/off-axis wall (no new wall-rams)? `clearance_min_hit_fraction:
  0.3` is a first guess, expect to retune after watching one flight.
