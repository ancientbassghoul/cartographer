# Cartographer — State (read this first)

This is the cheap, first-read resume file. Full session-by-session history and the
presentation narrative live in `PROGRESS.md`; full per-session technical design/trace lives in
`plans/*.md`. Read those only when you need depth this file doesn't have.

## What this project is
Assessment task: from the black-box **XLAB** Unity sim's single monocular drone feed,
autonomously map the room and report the 3D location of a target object (+ uncertainty).
Phases: 1 Human Recon → 2 Autonomous Survey → 3 Localize & Report → GUI. Grading = internal
consistency (metric scale and compute efficiency NOT graded). Local on an RTX 3080 Laptop (16 GB).

Five processes over a ZMQ bus: `io_bridge.py` (NDI capture + control to Unity), `perception_worker.py`
(MASt3R-SLAM → voxel map + 2D grid, publishes pose/map/plan/target), `visualizer.py` (read-only
dashboard), `object_worker.py` (3-stage target-detection cascade), `autopilot.py` (CPU-only flight
controller / FSM). One-command launch: `python fly.py`. Full architecture + run procedure + key
technical facts (control mechanic, build quirks, world-frame convention): `PROGRESS.md`'s
`## Architecture`, `## What's built`, and `## Reference — don't re-derive` sections.

## Current status
Branch **`all-bets-are-off`**. **Session 59 is BUILT, GATED and FLOWN**
(`OUTPUT/diag/20260904_103342_*`, ~37 min, 2026-09-04). All 9 suites green at HEAD.

Session 59's results: the 23-minute corner lockup **did not recur** — zero `ALREADY-excluded`
warnings against 29 the flight before, and no drop→re-commit loop. Caveat kept deliberately:
`CORNER-RETIRE-EN-ROUTE` never fired because no corner went dead mid-tour, so the fix is *not
contradicted* rather than *confirmed*. The camera trigger **did** fire, once and correctly
(`LIVE=21 ratio=1.00 over 3.0s`) — the first time the camera rather than the map asked for a maneuver.

**Session 60's spec is written and verified-parsing but NOT YET BUILT** — that is the next action.

`main` is unaffected and sits at session 43 (confirmed tolerable live-fly on 2026-09-01), with one
open problem: height.

## STILL OPEN, THE DOMINANT PROBLEM: why does SLAM choke
Nothing in session 59 addresses this — it's damage control around the choke, not a cure. The
2026-09-04 flight is the best measurement yet — **30 of its 47 minutes were spent blind**
(`HOLD_LOST` 1348 s + `FALLBACK` 471 s):

| flight min | frames | median | p90 | max |
|---|---|---|---|---|
| 0–5 | 151 | 791 ms | 2 829 | 13 970 |
| 5–10 | 72 | 1 404 | 9 960 | 16 479 |
| 10–15 | 56 | 2 436 | 10 860 | 37 197 |
| 15–20 | 22 | 12 682 | 22 133 | 58 061 |
| 20–25 | 16 | 5 056 | 41 969 | **71 774** |
| 25–30 | 86 | 1 783 | 4 402 | 37 209 |
| 40–45 | 78 | 1 816 | 3 012 | 31 399 |

Worst wait between two solved frames: **72.9 s** at flight-minute 24.6. Nuance worth keeping: the
median degrades 16× over the first 20 minutes but then **partially recovers** (min 25-30 back to
1783 ms, again at 40-45) — evidence *against* the simplest "keyframe graph grows monotonically"
theory. And **session 58 removed ~230 wasted SIFT matches per flight and the choke persisted
unchanged**, which weakens (does not kill) the LKG-debug-window lead. Three theories already ruled
out: autopilot loop rate (32-38.5Hz throughout), Unity focus loss (re-chokes ~2 frames after
refocus), and the clearance raycast (times outside `slam_ms`; `clearance_fan_deg`/`fan_n` unchanged
since session 12, six weeks of 300-500ms medians followed). Untested leads: MASt3R-SLAM's own
workload growing with the keyframe graph/retrieval DB, its backend optimization thread, GPU
contention from the visualizer's `--record` MP4 encode, and the LKG debug window (`cv2.imshow` + PNG
writes).

## >>> IMMEDIATE NEXT: BUILD session 60 <<<
```
python sonnet_runner.py --plan C:\Users\owner\.claude\plans\valiant-waddling-spark.md
```
7 chunks, parse-verified, all 29 source anchors confirmed. Chunk 7 copies the spec to
`plans/session60-spec.md`, so afterwards the repo holds its own record.

**What it does, and why.** Diagnosed off `OUTPUT/diag/20260904_103342_*`, whose centrepiece is a
**15.3-minute PLAN-STALE** during which SLAM was *not* choked (median `slam_ms` 1902) — it was
tracking-lost. Four findings:

1. **F_LKG cannot refresh.** The autopilot reconstructs it by looking the plan's `frame_id` up in a
   160-slot ring, but the plan naming a frame arrives ~15 s after that frame went past. Ring span
   ≈17.6 s for 71 MB; median age-out shortfall **0.55 s** (min 6 ids, max 1143); **34 age-outs**.
   Fix: **perception publishes the frame it tracked on**; the ring and all its age-out machinery are
   deleted. Note the timing that makes this bite — 66% of `OK` periods last exactly the 3.0 s
   `plan_timeout_s`, and those blips are when F_LKG is meant to refresh.
2. **One wall contact can permanently blacklist a goal.** `rearm_bump_if_disengaged` re-arms on any
   `reverse > 0`, which a back-off always commands — so the latch meant to make one contact count
   once is defeated by the back-off itself (pulses #3/#4 4.3 s apart → `BLACKLIST PERMANENT`).
   Fix: the latch ignores our own `BACKOFF`/`BLIND_BACKOFF` reverse. **`backoff_hold_s` deliberately
   unchanged** — operator accepts the double back-offs for now.
3. **The 15° probe has never recovered SLAM** — 0 recoveries in 9.4 min of exposure across 139 flight
   logs, vs FALLBACK's 11 in 61.5 min. Mechanism: the probe only *rotates*, so it cannot return to a
   viewpoint the drone has drifted from. Meanwhile the blind sweep matched **98 times**, including 11
   consecutive `EQUAL` verdicts over 5.6 s, and swept straight through it into `STUCK` 34 s before
   recovery. Fix: **delete the probe**; `PLAN-STALE` → 12 s grace → FALLBACK, with a **servo** inside
   FALLBACK that nudges forward/back to `EQUAL`, holds for **3 solved frames** (a count, not a timer —
   it self-calibrates to any solve latency), freezes the sweep budget while servoing, and never
   exhausts to `STUCK`.
4. **The probe's grace notice printed 413 times** — latched.

Also: the **LKG panel moves into the visualizer** (stacked, new left column, grey when idle) so it
lands in the `--record` MP4; the standalone `cv2` window is retired. **This is NOT a SLAM-choke
mitigation** — the match, the composition and the PNG writes all still happen; only `imshow` moves,
and an encode + IPC hop are added, so total work goes slightly *up*. The only real choke experiment is
a flight with `visrec_debug_window: false`, and it is mutually exclusive with having the panel.

Watch on the flight AFTER session 60 is built:

1. **Zero** `F_LKG AGE-OUT` lines — the message no longer exists — and `src=slam:<id>` advancing
   during the ~3 s `OK` blips, which is exactly where it used to freeze.
2. No two bump pulses from one contact; no permanent blacklist off a single wall touch.
3. No `VISUAL_RECOVERY` anywhere; a `SERVO` phase engaging on a sustained match with
   `_fallback_cum_deg` frozen; FALLBACK never reaching `STUCK`.
4. The LKG panel visible in the dashboard and grey when idle.
5. Kill switches: `use_visual_matching: false` disables all matching; `visrec_debug_window: false`
   disables the canvas.
6. Flights end by **manual stop**, as every flight has; no bounded-survey mechanism exists.
7. **Carry forward every still-unconfirmed session-49-to-57 item below.**

**Operator's decision rule (2026-09-04):** if this flight is clean, stop here and ship the
Blender/PLY presentation work. If goal problems recur, **rebuild goal management from scratch**
against a written behaviour spec — the operator's own judgement is that it is over-complicated, and
the evidence agrees: two independent death registries (`_blacklist` with soft/permanent/active, and
`_swept_corners` which deliberately ignores the first), four mechanisms writing the first (2-bump,
stall, loop, stagnation), a third `_goal_db` disc structure, plus `corner_no_blacklist_dist` /
`corner_giveup_limit` / clearance-inset carve-outs. Sessions 52, 58 and 59 each patched a *different*
hole in the same invariant and one broke another. Before any rewrite, build the carve-out inventory
("this exemption exists because flight X did Y") so nothing hard-won is dropped by accident.

**SESSION-61 CANDIDATES (deferred; none are in session 60's spec):**
- **Adaptive back-off strength** (operator's idea, 2026-09-04) — scale the back-off to the measured
  clearance DEFICIT instead of a fixed `backoff_hold_s`. A single duration cannot serve the observed
  trigger range 0.25 … 1.20 (sizing for the worst overshoots the mildest ~6×), which is why
  back-to-back back-offs persist. Caveat for whoever builds it: post-back-off clearance readings are
  heavily contaminated by SLAM re-localisation (timeline shows `0.25 → 7.30` and `7.88 → 0.25`), so a
  closed loop must NOT naively trust the next `forward_clearance_dist`.
- **Close session 47's dead re-solve gate.** `_backoff_resolve_since` (budget
  `backoff_resolve_budget_s: 12.0`) is armed by `_step_backoff` on completion but only ever *checked*
  inside `_maybe_loss_snapshot_backoff`; session 57 moved the PLAN-LOST path onto
  `_step_lost_recovery`, which never checks it. Dead on that path since session 57.
- **Staleness UI** — operator wants to discuss it first. `TOPIC_CONTROL` carries no plan status/age,
  so the visualizer keeps drawing its last plan as `PLAN valid` with a healthy goal while the
  autopilot has been blind for 20s+. FALLBACK ran 439s on the diagnosing flight — 234s under
  PLAN-LOST, 205s under PLAN-STALE, **0 ticks** under OK. The state label was the only honest field
  on screen.
- **Bump-pulse latency** — a blacklisted goal retired only via the 2-bump rule takes 10-18s to
  become a blacklist the autopilot can see (only rides the next published plan; `perception_worker.
  run()` blocks 8-10s per SLAM solve), and **this cost scales with SLAM latency** — the 2026-09-04
  flight measured solves up to 71.8s, which would stretch one bump to well over a minute. Three
  candidate designs sketched, none built:
  `plans/session58-lkg-window-discipline-and-dead-goal-guard.md`'s "Session-59 design sketch"
  section.

### Session 56/57's watch list (still current except the corner item above, confirmed 2026-09-03)
Full design/replay-arithmetic in `plans/session56-settle-gate-currency-and-lkg-freeze.md` and
`plans/session57-planlost-recovery-and-direction-aware-lkg.md`:

- A `[VISREC]` line reading `closer=LKG` while a back-off was pending → the hold should fire and
  **no** BACKOFF should follow (the operator-reported 08:51 symptom). Not yet observed maturing on a
  real flight — session 58's grace fix is what should finally let a decision-bearing match happen.
- `size=` versus `scale=` on the same lines — does the spread ratio hold steady where `scale` swung
  `0.27 → 1.85` within one second on the diagnosing flight?
- `LOST_VISUAL_HOLD` notices should appear and NOT repeat every tick.
- No two back-offs inside one loss episode should land closer together than `loss_backoff_grace_s`.
- Corner-tour goals should draw **blue** on the map panel; frontier goals stay yellow.
- If `diag.ply_sequence` is turned on, `OUTPUT/diag/<ts>_plyseq/` should fill, `markers.json` should
  be written, and the first five markers should sit at identical world coordinates in an early and a
  late frame. (Off by default — leave off for an ordinary flight, ~1.2GB/flight.)

1. **`SLAM_HOLD_FORCED_HOP`/`SETTLE_DEADBAND` should become rare, not the normal exit.** Before
   this session every gate release came from the 15s dead-band escape; now most holds should
   resolve in roughly one capture-gap-plus-one-solve (~3.5s median / ~5s p75 on the diagnosing flight).
2. **No 3.5s `ADVANCE`↔`SLAM_HOLD` limit cycle.** If it appears, the release-grace stamp isn't
   landing — check it's stamped AFTER `_enter`, not before.
3. **`[VISREC]` bursts gone.** A held-still loss should log a handful of matches, not hundreds/minute.
   Zero `scale=1.00 inliers=7xx contained=True` self-matches anywhere in the log.
4. **The LKG debug window shows a frozen reference with growing `age=` during a loss, never a live
   view.** `LKG=` on the telemetry panel should read `slam:<id>` normally and go red
   (`LKG=STALE x<n>`) only when genuinely degraded.
5. **TRIM fires while parked in `SLAM_HOLD`**, not just from `SETTLE`/`ADVANCE`.
6. **Everything from sessions 49-55 is still itself unconfirmed** (their flights kept getting cut
   short by the bugs sessions 53-56 fixed) — re-watch on this same flight:
   - Session 55 crash-survivability: periodic livemap checkpoints, multiple `"map"` timeline
     records, and (if you deliberately kill perception/visualizer mid-flight) the crash-recovery
     prompt + `salvage_flight.py` actually working.
   - `TRIM` exits within ~3s of its pulse even at 1500-2700ms SLAM latency (a `FORCED after N.Ns`
     log line means the backstop fired, not a bug, but means the primary fix isn't landing).
   - `SLAM_HOLD`↔`HOLD_LOST` limit cycle still can't persist past ~18s (regression check only).
   - `VISUAL_RECOVERY` actually executes a probe turn (never yet observed on a real flight);
     `TRIM enter (DOWN)` firing (every flight so far only showed sag/UP); no back-off suppressed
     notice unless one was genuinely about to fire; no goal committed inside a permanently
     blacklisted region; the LKG debug window drawing inliers + saving PNGs.
   - Session 50: `SETTLE gate blocked ... -> forcing REPLAN` in place of a 91.6s park. If it fires
     OFTEN, that's an honest signal SLAM is chronically slow, not a bug in the fix.
   - Session 51: pure waste removal — expect **zero** decision changes vs. earlier flights.

### `main` branch — next after that: diagnose the HEIGHT issue
A 2026-09-01 live flight confirmed sessions 20-43 fly *tolerably* (operator's own call). Height is
still off — not yet diagnosed. Start by opening the flight replay debugger and comparing `pos_y` vs
`target_altitude_y` across the flight; check whether `trim_pulse_s` (currently `0.01`, much shorter
than the `0.16` session 40 tuned) is even correcting meaningfully, before assuming it's TRIM.

## Standing rules
CLAUDE.md carries the durable rules for this repo (NO SILENT FALLBACKS, image-integrity guardrail,
NO MANUAL-FLIGHT DATA LEAKAGE into autonomy limits, the PROGRESS.md/STATE.md update-then-commit-then-push
rule, task-list discipline) — read it, don't re-derive it here. One live exception on record: branch
`all-bets-are-off` session 44 hardcoded two TRIM pos_y thresholds, an explicit operator-approved
override of the no-leakage rule, scoped to that branch only.

Full session-by-session history and presentation narrative: `PROGRESS.md`.
Full per-session technical design/trace: `plans/*.md`.
