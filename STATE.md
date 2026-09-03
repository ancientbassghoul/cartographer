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
Branch **`all-bets-are-off`**. **Session 57 is BUILT, LIVE-FLY PENDING** — all 10 chunks of
`plans/session57-spec.md` applied (chunk 4's fix turned out to need a companion chunk 5 to close a
trap it exposed; the spec file's own chunk numbering was renumbered in place to 1-10, see
`plans/session57-planlost-recovery-and-direction-aware-lkg.md`). All **9** self-test suites are green
at HEAD (`visualizer.py` joined the gate this session with its first `--self-test` entry point).
Session 56 (BUILT and FLOWN on 2026-09-03, `OUTPUT/diag/20260903_083329_*`) is folded into this
session's watch list below — its own fixes still haven't been re-flown in isolation. `main` is
unaffected and sits at session 43 (confirmed tolerable live-fly on 2026-09-01), with one open
problem: height.

## >>> IMMEDIATE NEXT: live-fly session 57 <<<
Full design/traps/files-touched: `plans/session57-planlost-recovery-and-direction-aware-lkg.md`.
**What it fixes and why** — the operator saw the LKG window show a reference frame plainly closer
than the live frame while the drone backed off anyway. Of the diagnosing flight's 86 loss episodes
only **30 ever ran a visual match**; 56 ran none, because the one-shot ticket that gated matching was
almost always spent before any picture was taken. PLAN-LOST/NO-PLAN now always waits 12s, then always
looks; the cached clearance only marks a back-off *pending*; a three-way inlier-spread `closer`
verdict adjudicates it; and firing a back-off restarts the wait.

Watch these, in priority order, on the first live flight:

1. **Every** loss episode should now show `[VISREC]` lines after 12s, including clear-front ones.
   Diagnosing flight: 56 of 86 episodes ran zero matches. Expect that count to go to **zero**.
2. A `[VISREC]` line reading `closer=LKG` while a back-off was pending → the hold should fire and
   **no** BACKOFF should follow. This is the operator-reported 08:51 symptom, directly.
3. `size=` versus `scale=` on the same lines — does the spread ratio hold steady where `scale` swung
   `0.27 → 1.85` within one second on the diagnosing flight?
4. `LOST_VISUAL_HOLD` notices should appear and NOT repeat every tick.
5. No two back-offs inside one loss episode should land closer together than `loss_backoff_grace_s` —
   the restamp is what enforces this now that the one-shot ticket is gone from this path.
6. Corner-tour goals should draw **blue** on the map panel; frontier goals stay yellow.
7. If `diag.ply_sequence` is turned on, `OUTPUT/diag/<ts>_plyseq/` should fill, `markers.json` should
   be written, and the first five markers should sit at identical world coordinates in an early and a
   late frame. (Off by default — leave off for an ordinary flight, ~1.2GB/flight.)
8. **Session 56's own watch list below still applies in full** — this is still its first live flight.

### Session 56's watch list (still current — its fixes were flown once, on 2026-09-03)
Full design/replay-arithmetic in `plans/session56-settle-gate-currency-and-lkg-freeze.md`:

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

**STILL OPEN, TOP OF THE LIST: why does SLAM choke** (plateaus at ~2000ms for minutes, worst gap
90.8s on the session-52 flight)? **Three** theories now ruled out — autopilot loop rate (measured
32-38.5Hz throughout), Unity focus loss (tested directly, re-chokes ~2 frames after refocus), and
**the clearance raycast** (session 57: `slam_ms` times *only* `slam.process(rgb)`
(`perception_worker.py:218-220`) while the clearance fan runs afterwards in `_plan_payload`, outside
that window; and `clearance_fan_deg`/`fan_n` last changed at session 12, after which flights ran at
300-500ms medians for six weeks — do not re-derive this). Untested leads: MASt3R-SLAM's own workload
growing with the keyframe graph/retrieval DB, its backend optimization thread, GPU contention from
the visualizer's `--record` MP4 encode, and the session-49 LKG debug window (`cv2.imshow` + PNG
writes; medians stepped from ~400ms to ~1300-1700ms on 2026-09-01 between the 17:22 and 21:48
flights, which brackets when that window was built — correlation only, one flight with
`visrec_debug_window: false` would settle it).

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
