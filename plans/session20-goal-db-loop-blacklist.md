# Session 20 — De-commit the hops + a persistent goals database (loop-blacklist) + corner-goal safety

## Context

Branch `leg-hops-and-goal-commit-fix` (= `main` + the first session-20 commit) flew badly, while `session19`
flies fast and smooth for one reason: **SLAM is allowed to re-pick its goal freely** instead of the drone
hardening its life by committing to one distant goal. But letting SLAM pick freely re-introduces **goal
ping-pong** — the planner oscillates between a few nearby goals, the drone circles a small area, and the safety
watchers go blind (the 2-bump `note_wall_hit` counter resets on every goal change — the "counter defeated" bug
from `blacklist-region-and-counter.md`).

Operator's decisions (locked): **keep the 40-tick hop cadence**, remove only the *commitment*; keep the
leg-stall guard as a safety; keep `forward_throttle: 1.0`; loop rule = **picks ≥ 3 AND any pair of pick-time
drone locations < 1u**; the goals DB **persists the whole flight, never reset mid-flight**; `main` stays
untouched (all work on `leg-hops-and-goal-commit-fix`).

## What "de-commit the hops" means (the key clarification)

The hop cadence stays: ADVANCE flies `hop_ticks` (40) ticks → SETTLE (a fresh-frame SLAM breather). What dies is
the *resume-the-old-goal* behavior. Previously the post-hop SETTLE routed back to `ADVANCE` toward the same,
unreached `leg_goal` (`_settle_to = "ADVANCE"`). Now it routes to **`REPLAN`**, which re-reads SLAM's *current*
goal; if SLAM re-picked a different goal while the drone advanced, the drone adopts the new one — re-orient
(**with the parallax scout** for an off-axis goal) → hop — instead of stubbornly finishing the old leg.

## Changes

### 1. `autopilot.py` — hop routes to REPLAN; region-gated leg-stall tracker; far-corner flag

- ADVANCE hop branch: `self._settle_to = "ADVANCE"` → **`"REPLAN"`** (+ updated event text). A corner cruise is
  therefore itself chunked into hops, so a frontier found en route is adopted at the next hop's REPLAN — this
  is what makes SLAM "allowed to find goals on the way to a corner" (Part 2.1), for free.
- REPLAN: capture `prev_leg_goal` before overwriting `leg_goal`; set `self._leg_is_corner =
  bool(plan.get("goal_is_corner"))`; reset the leg-stall trackers (`_leg_best_dist`/`_leg_progress_t`) **only on
  a genuine goal-region change** (`_dist(prev, new) > goal_assoc_dist`). Without this gate, re-planning every
  hop would wipe the stall clock each hop and the kept leg-stall guard could never fire.
- `_register_bump` FAR-CORNER guard: if `_leg_is_corner` and `_dist(pos, leg_goal) > corner_no_blacklist_dist`
  (1.0), return early (stash a MISSED-BUMP note, no pulse). This blocks BOTH the region blacklist and the corner
  retirement in `note_wall_hit`, and — because the leg-stall guard also bumps through this choke point — it
  protects a far corner from the leg-stall guard too (Part 2.2).
- New knobs read from config: `_leg_region_dist` (= `goal_assoc_dist`), `corner_no_blacklist_dist`.

### 2. `frontier_planner.py` — the persistent goals database + circling-loop blacklist

- New PERSISTENT state (never reset mid-flight): `_goal_db` (entries `{center:[x,z], picks:int,
  drone_locs:[[x,z],…]}`), `_last_pick_center` (a ~2 Hz cursor to avoid double-counting a held goal),
  `last_loop_event` (transient, for the caller to log).
- `_commit(goal, pos)`: after committing, register a PICK **only when the goal enters a different disc than the
  last pick** (`_d > goal_area_radius`), so holding one goal across the 2 Hz selects counts once while a genuine
  goal-switch (incl. ping-pong) counts each time.
- `_register_pick`: match the goal to an existing disc (within `goal_area_radius` 0.5) → increment its pick
  count + append the pick-time drone `pos` (bounded by `goal_db_maxlocs`); else add a new disc. Then `_check_loop`.
- `_check_loop`: `picks > goal_loop_min_picks` (≥3) AND any two `drone_locs` within `goal_loop_pos_dist` (<1u)
  → `_blacklist_goal(permanent=True)` (the SAME store `_excluded` filters), drop the commitment if it's this
  region, set `last_loop_event`. Idempotent (skips an already-dead region).
- `_select_reachable`: bounded retry — a commit can loop-blacklist its own pick and drop the commitment; then
  re-filter `_excluded` and pick the next reachable frontier. Corners still bypass this path (never in the DB).

### 3. `perception_worker.py` — publish + surface

- `_plan_payload`: publish `goal_is_corner = bool(planner.sweeping)`; after `select()`, if
  `planner.last_loop_event` is set, print a `LOOP-BLACKLIST …` line and set `last_planner_event` so it rides the
  next plan's timeline (the blacklist itself is reflected in the same payload's `blacklist`).

### 4. `config.yaml` — new general SLAM-unit knobs (autonomy.explore)

`goal_area_radius: 0.5`, `goal_loop_min_picks: 2`, `goal_loop_pos_dist: 1.0`, `goal_db_maxlocs: 12`,
`corner_no_blacklist_dist: 1.0`. `forward_throttle: 1.0` and `hop_ticks: 40` unchanged.

## Review of the blacklist instruments (operator's explicit ask)

- **2-bump `note_wall_hit`** — physical contact; still correct for a rammed wall, but its counter resets on goal
  change → blind to ping-pong.
- **Goals-DB loop-blacklist (new)** — accumulates *picks* across goal changes via the 0.5u disc, immune to that
  reset; the precise patch for the ping-pong hole free re-picking opens.
- Both write the **same** permanent `_blacklist` (radius 1.0) that `_excluded` filters; `goal_area_radius` 0.5
  (DB dedup) < `blacklist_radius` 1.0 (exclusion), so one loop-blacklist also suppresses the shifted
  micro-frontiers around it (the region intent of `blacklist-region-and-counter.md`).
- Corners bypass the DB and (when far) the bump — a far corner is never wrongly retired; a corner is retired
  only by a 2-bump earned while essentially on it (<1u).

## Self-tests (all green)

- `frontier_planner.py --self-test`: (db1) ping-pong from one spot → A blacklisted on the 3rd pick; (db2)
  ping-pong from spread-out spots → NOT a loop; (db3) holding one goal across ticks = one pick; (db4) a corner
  never enters the DB; (db5) DB persists across a goal switch. Existing corner/blacklist tests unchanged.
- `autopilot.py --self-test`: rewrote the session-20 hop test → HOPS-NO-COMMITMENT (hop→REPLAN route; REPLAN
  adopts a re-picked off-axis goal with the parallax scout; region-gated tracker persist/reset; leg-stall still
  fires; far/near corner bump guard).
- Full suite green: autopilot, frontier_planner, io_bridge, flight_replay, ground_grid, flow_contact_detector.

## Live-fly verification (pending)

`python fly.py`, press `m`, watch the replay/command timeline for: (a) each hop re-picking (ORIENT + parallax
between hops, goal changes honored, no stubborn cruise to a stale goal); (b) a ping-pong loop retiring in a
handful of picks via `LOOP-BLACKLIST` (no minutes-long circling, no "counter defeated" thrash); (c) a far corner
surviving a transient near-start stall while a frontier found en route to a corner is investigated.

---

## Session 20b — per-hop progress + strikes (the first live fly of the above froze)

**What broke.** Flight `20260716_140437` froze re-picking one goal `[-1.8929,-0.45]` forever, `leg STALL … 75.0s`
every leg, never blacklisting. Three faults: (1) the session-20 leg-stall guard fired the INSTANT ADVANCE began
(its stall clock never reset across same-region re-picks; the drone was farther than its stale best-dist), so
ADVANCE bailed to SETTLE **before emitting a forward command** — the drone never moved. (2) The 2-bump latch
can't re-arm on a frozen drone → stuck at 1. (3) The goals-DB counted a pick only on a DIFFERENT disc, so the
same goal re-picked every leg stayed at `picks=1`. (The one goal that DID blacklist, `[-1.81,-2.65]`, was 2.20u
from the next pick — correctly outside the 1.0 radius; different goals along one wall, each stalled once.)

**Rebuild (operator's tightened rules).**
- **A stall is a MEASURED CONSEQUENCE, not a precondition.** Removed the leg-stall guard + `_leg_best_dist`/
  `_leg_progress_t` + region-gate. On ADVANCE entry the autopilot snapshots `_hop_start_dist`/`_hop_start_goal`;
  at the next REPLAN it judges the finished hop: closed ≥ `hop_progress_eps` (0.2) OR now within `goal_reach_dist`
  ⇒ progress, else a STALL. A plan-loss / SLAM-choke hold (`_enter` HOLD_LOST/SLAM_HOLD) clears the pending eval.
- **The goals-DB is fed by the AUTOPILOT, once per leg.** At each REPLAN commit the autopilot stashes a combined
  pulse `{pick_goal, pick_pos, prev_goal, prev_progressed, prev_strike_eligible}` (`take_pick_pulse`); run_explore
  publishes it on `TOPIC_AUTOPILOT_EVENT` (mirror of the bump pulse); `perception_worker` drains it into
  `planner.register_hop_outcome` (strikes) + `planner.register_goal_pick` (loop). Removed the `_commit`-side
  registration + `_last_pick_center`.
- **Three complementary blacklist guards (operator-confirmed), all → the same permanent `_blacklist`, none block
  flight:** 2-bump (physical contact), STRIKES (`register_hop_outcome`: reset on progress, +1 on stall, blacklist
  at `goal_strike_limit`=2), PICKS-LOOP (`register_goal_pick`: >`goal_loop_min_picks` picks with ALL drone-locs
  inside one `goal_loop_pos_dist`-wide cluster — max pairwise spread ≤ it). A FAR corner (`strike_eligible=False`
  when `_leg_is_corner` and `dist > corner_no_blacklist_dist`) is exempt from strike + bump.
- **Loop-rule tightening (first fly of 20b false-blacklisted goals).** The loop was "**any pair** of drone-locs
  <1u", which false-fired on a legit MARCHING approach to a far goal over several short (<1u) hops (adjacent
  picks close, but the trail spans >1u). Tightened to "**ALL** drone-locs within one 1u cluster (max spread ≤
  `goal_loop_pos_dist`)" — genuine circling keeps every pick in the small ball; a marching approach spans >1u and
  is left to the STALL/strike guard (which covers "approached then stuck"). No coverage lost.
- **Goals DB in the replay debugger.** `planner.goal_db_snapshot()` (center / picks / strikes / the actual
  DRONE LOCATIONS at each pick / blacklisted) → `payload["goal_db"]` → `_timeline_step_record` → a draggable
  floating **Goals DB** table in `flight_replay.py`: a goal row (center · picks · strikes · status · cluster
  `spread`) with one indented sub-row per pick showing the drone's X,Z — so a loop blacklist can be verified by
  eye. Updated in `render()`; older logs without `goal_db` degrade gracefully.

**Files:** `autopilot.py` (per-hop snapshot, pulse, remove leg-stall guard), `frontier_planner.py`
(`register_hop_outcome`/`register_goal_pick`/`goal_db_snapshot`, `_db_entry`/`_db_blacklist`, strikes),
`perception_worker.py` (drain + publish), `flight_replay.py` (floating table), `config.yaml`
(`hop_progress_eps` 0.2, `goal_strike_limit` 2; removed the orphaned `ram_progress_eps`). New knobs are general
SLAM-unit params (HARD RULE). **All 6 module self-tests green** (HOPS+PER-HOP-STRIKE; planner db1–db5 strike/loop;
flight_replay goal_db). **LIVE-FLY PENDING.**
