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
Branch **`all-bets-are-off`**. **Session 60 is BUILT and GATED, NOT YET FLOWN** — all 9 self-test
suites green at HEAD, spec archived at `plans/session60-spec.md`.

Session 60 was diagnosed off session 59's own flight (`OUTPUT/diag/20260904_103342_*`, ~37 min,
2026-09-04), whose centrepiece was a **15.3-minute PLAN-STALE** during which SLAM was *not* choked
(median `slam_ms` 1902) — it was tracking-lost, i.e. recovery logic failing, not SLAM speed. Four
fixes went in: F_LKG moved to perception (the reconstruct-from-ID ring, 34 age-outs/median 0.55s
shortfall last flight, is deleted); a back-off's own reverse could re-arm the bump latch it was
supposed to be exempt from, letting one wall contact permanently blacklist a goal (fixed); the 15°
rotation probe (0 recoveries in 9.4 min across 139 logs) is deleted in favour of a `SERVO` phase
inside `FALLBACK` that steers to the F_LKG viewpoint instead of just spinning; and the LKG debug
canvas moved from a standalone `cv2` window into a new visualizer dashboard column. Full detail:
`plans/session60-spec.md`; concise narrative: `PROGRESS.md` session 60.

`main` is unaffected and sits at session 43 (confirmed tolerable live-fly on 2026-09-01), with one
open problem: height.

## STILL OPEN, THE DOMINANT PROBLEM: why does SLAM choke
Nothing in sessions 59-60 addresses this — it's damage control around the choke, not a cure. The
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

## >>> IMMEDIATE NEXT: FLY session 60, then watch for <<<

1. **Zero** `F_LKG AGE-OUT` lines (34 last flight) — the message no longer exists.
2. `src=slam:<id>` in the telemetry `LKG=` field advancing during the ~3 s `OK` blips, which is
   exactly where it used to freeze.
3. No two bump pulses from one contact; no permanent blacklist off a single wall touch.
4. No `VISUAL_RECOVERY` anywhere in the log; a `SERVO` phase engaging on a sustained match with
   `_fallback_cum_deg` frozen while it holds; FALLBACK never reaching `STUCK` (only the corner
   give-up path still can).
5. The LKG panel visible in the dashboard's new leftmost column, and grey (not black) when idle.
6. Kill switches, if either half needs isolating: `use_visual_matching: false`
   (`autonomy.explore`) disables all SIFT matching; `visrec_debug_window: false` disables the
   canvas build/publish/save entirely — the one real SLAM-choke experiment this session enables.
7. Flights still end by **manual stop**; no bounded-survey mechanism exists.
8. **Carry forward every still-unconfirmed session-49-to-57 item below** (unchanged from session 59).

**Operator's decision rule (2026-09-04), unchanged:** if this flight is clean, stop here and ship the
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
4. **RESOLVED by session 60, not open.** This item asked whether `LKG=` would go red
   (`LKG=STALE x<n>`) only when genuinely degraded — session 60 made that condition structurally
   impossible (F_LKG is now published straight from perception, no reconstruct-from-ID ring to age
   out) and removed the indicator entirely, and moved the window itself into the visualizer's LKG
   panel. Watched fresh at IMMEDIATE NEXT items 1-2 and 5 above.
5. **TRIM fires while parked in `SLAM_HOLD`**, not just from `SETTLE`/`ADVANCE`.
6. **Everything from sessions 49-55 is still itself unconfirmed** (their flights kept getting cut
   short by the bugs sessions 53-56 fixed) — re-watch on this same flight:
   - Session 55 crash-survivability: periodic livemap checkpoints, multiple `"map"` timeline
     records, and (if you deliberately kill perception/visualizer mid-flight) the crash-recovery
     prompt + `salvage_flight.py` actually working.
   - `TRIM` exits within ~3s of its pulse even at 1500-2700ms SLAM latency (a `FORCED after N.Ns`
     log line means the backstop fired, not a bug, but means the primary fix isn't landing).
   - `SLAM_HOLD`↔`HOLD_LOST` limit cycle still can't persist past ~18s (regression check only).
   - `VISUAL_RECOVERY` executing a probe turn: **moot** — session 60 deleted the probe STATE outright
     (it never once recovered SLAM in 139 logs; see PROGRESS.md session 60), so this no longer applies.
     Still open: `TRIM enter (DOWN)` firing (every flight so far only showed sag/UP); no back-off
     suppressed notice unless one was genuinely about to fire; no goal committed inside a permanently
     blacklisted region; the LKG canvas drawing inliers + saving PNGs (now via the visualizer panel).
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
