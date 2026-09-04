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
Branch **`all-bets-are-off`**. **Session 58 is BUILT and FLOWN** (`OUTPUT/diag/20260903_234939_*`,
47 min, 2026-09-04) — all 4 chunks of `plans/session58-spec.md` applied, all **9** self-test suites
green at HEAD. **Both of its fixes verified live**: `[VISREC]` matches inside the loss grace fell
from 233/253 (92%) to **45/1454 (3%)**, and the dead-goal drop fired correctly every time.

**But that flight ended badly** — the drone rammed glass at bounding-box corner `[-1.5, -3.9]` for
**23.3 minutes** (45 legs), and session 58's own bump guard was half the cause. **Session 59's spec
is written and verified-parsing but NOT YET BUILT** — that is the next action, below.

`main` is unaffected and sits at session 43 (confirmed tolerable live-fly on 2026-09-01), with one
open problem: height.

## >>> IMMEDIATE NEXT: BUILD session 59 <<<
```
python sonnet_runner.py --plan C:\Users\owner\.claude\plans\valiant-waddling-spark.md
```
5 chunks, parse-verified, all source anchors confirmed to resolve. Chunk 5 copies the spec to
`plans/session59-spec.md`, so after the run the repo holds its own record.

**Why it exists.** Flying session 58 confirmed both its fixes, then ran the drone into glass at
corner `[-1.5, -3.9]` for 23.3 minutes. Three findings:

1. **A committed sweep corner is never re-checked.** The tour committed to the corner at 00:11:06;
   the LOOP guard permanently blacklisted it at 00:13:01; `select()` returns an already-committed
   `sweep_target` unconditionally, forever, with no `_excluded_permanent` re-check. Session 52 built
   exactly that guard, but it lives in `_pick_sweep_corner`, only reached when `sweeping` is
   **False** — so it never ran once (zero `CORNER-SKIP` lines, 29 × `WARNING: pick landed on an
   ALREADY-excluded goal`). The only remaining escape was `note_wall_hit`'s 2-bump, and **session 58's
   own guard blocked it**, so the corner could never be retired and the tour never advanced. The
   result was a 51-millisecond loop repeated for 23 minutes: session 58's drop and the planner's
   re-emit fighting each other.
2. **The `closer` verdict can veto a back-off but never request one.** 496 `closer=LIVE` verdicts,
   **zero** actions — `_step_lost_recovery` returns early unless the *map's* cached clearance already
   proposes a back-off, and on glass the map reads clear forever. Every back-off that did fire cited
   `closer=UNKNOWN`. 555 of 1454 matches were computed in `FALLBACK`, which returns before any
   consumer runs. Session 59 adds an operator-specified hysteresis trigger (3 s confident window,
   ≥66% LIVE over LIVE+EQUAL+LKG, UNKNOWN discarded), kill switch
   `use_visual_backoff_trigger: false`.
3. **Deferred, operator wants to discuss first:** the staleness UI. `TOPIC_CONTROL` carries no plan
   status/age, so the visualizer keeps drawing its last plan as `PLAN valid` with a healthy goal while
   the autopilot has been blind for 20 s. FALLBACK ran 439 s this flight — 234 s under PLAN-LOST,
   205 s under PLAN-STALE, **0 ticks** under OK. The state label was the only honest field on screen.

Watch these on the flight AFTER session 59 is built:

1. **The corner tour advances** — expect a `CORNER-RETIRE-EN-ROUTE` line and the next corner
   committed within one plan publish. Expect **zero** `WARNING: pick landed on an ALREADY-excluded
   goal` lines (29 last flight).
2. **No drop→re-commit loop** — no `already PERMANENTLY blacklisted` line followed within a second by
   an `ORIENT` toward that same goal. If they still disagree, session 59's `DEAD_GOAL_RECOMMIT`
   notice must say so on the panel.
3. **The camera fires at least once** — back-off lines citing a `LIVE=/EQUAL=/LKG=` tally rather than
   only `closer=UNKNOWN`. None appearing is information, not necessarily a bug.
4. **No BACKOFF↔FALLBACK oscillation** (the trap session 59 guards against).
5. Regression: session 58's grace fix holds — `[VISREC]` inside a loss grace stays near zero.
6. Flights end by **manual stop**, as every flight has; no bounded-survey mechanism exists.
7. **Carry forward every still-unconfirmed session-49-to-57 item below.**

**Operator's decision rule (2026-09-04):** if that flight is clean, stop here and ship the
Blender/PLY presentation work. If goal problems recur, **rebuild goal management from scratch**
against a written behaviour spec — the operator's own judgement is that it is over-complicated, and
the evidence agrees: two independent death registries (`_blacklist` with soft/permanent/active, and
`_swept_corners` which deliberately ignores the first), four mechanisms writing the first (2-bump,
stall, loop, stagnation), a third `_goal_db` disc structure, plus `corner_no_blacklist_dist` /
`corner_giveup_limit` / clearance-inset carve-outs. Sessions 52, 58 and 59 each patched a *different*
hole in the same invariant and one broke another. Before any rewrite, build the carve-out inventory
("this exemption exists because flight X did Y") so nothing hard-won is dropped by accident.

### Session 56/57's watch list (still current except item 1 above, confirmed 2026-09-03)
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

**SESSION-60 CANDIDATE (deferred twice now): bump-pulse latency.** A blacklisted goal retired only via the 2-bump
rule takes 10-18s to become a blacklist the autopilot can see, because the pulse only rides the next
published plan and `perception_worker.run()` blocks 8-10s per SLAM solve — on the diagnosing flight
this stretched one goal's retirement to 4min10s across three glass rams. Design sketch (three
candidates, none built, operator's call needed before any code):
`plans/session58-lkg-window-discipline-and-dead-goal-guard.md`'s "Session-59 design sketch" section.
**This cost SCALES with SLAM latency** — the 10-18s figure came from 8-10s solves; the 2026-09-04
flight measured solves up to 71.8s, which would stretch one bump to well over a minute.

**STILL OPEN, THE DOMINANT PROBLEM: why does SLAM choke.** The 2026-09-04 flight is the best
measurement yet — **30 of its 47 minutes were spent blind** (`HOLD_LOST` 1348 s + `FALLBACK` 471 s):

| flight min | frames | median | p90 | max |
|---|---|---|---|---|
| 0–5 | 151 | 791 ms | 2 829 | 13 970 |
| 5–10 | 72 | 1 404 | 9 960 | 16 479 |
| 10–15 | 56 | 2 436 | 10 860 | 37 197 |
| 15–20 | 22 | 12 682 | 22 133 | 58 061 |
| 20–25 | 16 | 5 056 | 41 969 | **71 774** |
| 25–30 | 86 | 1 783 | 4 402 | 37 209 |
| 40–45 | 78 | 1 816 | 3 012 | 31 399 |

Worst wait between two solved frames: **72.9 s** at flight-minute 24.6. The median degrades 16× over
the first 20 minutes — but then **partially recovers** (min 25-30 back to 1783 ms, again at 40-45).
That is evidence *against* the simplest "keyframe graph grows monotonically" theory in the untested
leads below, and is the most useful new datum this flight produced. **Session 58 removed ~230 wasted
SIFT matches per flight and the choke persisted unchanged**, which weakens (does not kill — the
window is still built, just rarer) the LKG-window lead below.

Older framing, kept for context (plateaus at ~2000ms for minutes, worst gap
90.8s on the session-52 flight): **Three** theories now ruled out — autopilot loop rate (measured
32-38.5Hz throughout), Unity focus loss (tested directly, re-chokes ~2 frames after refocus), and
**the clearance raycast** (session 57: `slam_ms` times *only* `slam.process(rgb)`
(`perception_worker.py:218-220`) while the clearance fan runs afterwards in `_plan_payload`, outside
that window; and `clearance_fan_deg`/`fan_n` last changed at session 12, after which flights ran at
300-500ms medians for six weeks — do not re-derive this). Untested leads: MASt3R-SLAM's own workload
growing with the keyframe graph/retrieval DB, its backend optimization thread, GPU contention from
the visualizer's `--record` MP4 encode, and the session-49 LKG debug window (`cv2.imshow` + PNG
writes; medians stepped from ~400ms to ~1300-1700ms on 2026-09-01 between the 17:22 and 21:48
flights, which brackets when that window was built — correlation only, one flight with
`visrec_debug_window: false` would settle it). **Session 58 removed ~230 wasted SIFT matches/flight**
(Finding 1's grace-bypass fix) — the next flight is a cheap natural experiment on the LKG-window lead
above; a null result (choke persists unchanged) proves nothing on its own since the window itself is
still built, just rarer, but a clear drop in choke frequency would be suggestive.

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
