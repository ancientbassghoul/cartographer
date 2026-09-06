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

### >>> THE PROBLEM IS AN IN-FLIGHT ACCUMULATOR. EVERYTHING ELSE IS SECONDARY. <<<
Operator's conclusion, 2026-09-06, and the data supports it. `tracker_ms` is **flat at ~250-400 ms**
across a deterministic replay of the EXACT frames a flight consumed, and **2000-6000 ms** on the live
flight those same frames came from. Every live flight starts at ~370 ms with the drone on the ground,
then steps up **7-17x within two minutes of takeoff** and never returns:

| run | start | -> | -> | -> | end |
|---|---|---|---|---|---|
| replay, window ON | 253 | 236 | 814 | 268 | 267 |
| replay, window OFF | 254 | 269 | 353 | 360 | 394 |
| **live, window ON** | 363 | **6240** | 5035 | 2722 | 2668 |
| **live, window OFF** | 382 | **4549** | 2530 | 3127 | 3127 |

**Ruled out:** the frames (replay uses the identical sequence); any of the session 63-66 speed work
(the step change is identical with the window ON and OFF, backend inline or threaded); and plain
GPU contention with Unity (real — replays run 1.0-1.4 s/frame vs 2.4-7.3 live — but constant across
every flight, so it cannot explain a step change at takeoff or a 2x gap between two same-config
flights). **Something changes state once the vehicle is airborne, in a process whose inputs are
provably identical.**

**NEXT STEP — instrument `tracker.track()` and fly once.** Log three numbers per call: the GN
**iteration count**, the **convergence reason** (`rel_error` / `delta_norm` / hit the `max_iters: 50`
ceiling), and the **match/inlier count** the solve started from. No behavioural change, and the
frame-exact replay of that flight is a matched control. Decisive either way:
- **Iterations climb at takeoff** -> the accumulator is the warm-start feedback loop (worse tracking
  -> worse `idx_f2k` seed -> more iterations -> slower -> worse tracking). Real knobs: `max_iters`,
  `rel_error`, or reseeding `idx_f2k` from retrieval instead of the previous frame.
- **Iterations flat while `tracker_ms` rises** -> the cost is not in the solver; it is process-level
  state a replay never accumulates (CUDA allocator fragmentation, retrieval-DB growth, sim
  contention scaling with map size). The same log eliminates the solver and redirects the hunt.

Full write-up with every table, and this plan in section 03: `plans/slam_report.html`.

**Do not resume the constant-factor optimisation work before this is understood.** Sessions 63-66
produced real, measured improvements that do not touch this, and chasing more of them is yak-shaving.

Branch **`all-bets-are-off`**. **Session 65 (bounded global-opt window) is BUILT, GATED, FLOWN and
BENCHED. Session 66 made offline replay frame-exact and fixed the window's real cost.**
`config.yaml` currently ships **`backend_window_mode: "OFF"`, `backend_window_kf: 0`** — set back
to OFF at the operator's request so a flight runs exactly as it did before the speed work
(`backend_async` was already `false`). The window is a real ~10% improvement on replay (evidence
below) and can be re-enabled with `"ON"` / `30`, but it is NOT the fix for the accumulator above. Session 65's design/mechanics: `plans/session65-spec.md`.
Session 66's measurements and replay mechanics: `plans/session66-frame-exact-replay-and-window-syncs.md`.

**>>> THE BIGGEST THING LEARNED THIS SESSION — offline replay had NEVER reproduced live flight. <<<**
SLAM runs at ~0.2 Hz, so `io_bridge` hands it the newest frame and DROPS the backlog. Measured on
flight `20260906_144839`: the gap between consecutively PROCESSED frames is a median of **112** NDI
frames (p90 624, max 1735) at ~58 fps capture — live sees snapshots ~1.9 s apart. Replaying a
recording at the default `--stride 3` puts frames 17 ms apart, so the match fraction never falls
below `match_frac_thresh: 0.333` and almost no keyframes form (measured: **3 keyframes in 200
frames**). No fixed stride reproduces live, because the real gaps are wildly uneven (median 76, p90
543, max 1510 on the session-66 flight). **Any bench, and `plans/slam_report.html`'s claim that
resolution work is "a bench experiment, not a flight", must account for this.**

The fix, and the tool to use from now on:
- `rec_frame` is now a flight-CSV column (it already rode `TOPIC_POSE` but never reached disk).
  `io_bridge` writes EVERY NDI frame to `flight_<ts>.mp4`, so it IS that file's frame number.
- **Press `r` in the io_bridge window on every flight.** Then replay it exactly:
  `venv\Scripts\python.exe perception_worker.py --video OUTPUTlight_<ts>.mp4 --frame-list
  OUTPUT\diag\<ts>_perception.csv --no-display --log --out <dir>`
  This is a **controlled A/B** — same frames, same order — which no pair of live flights can be.

**Window verdict — SETTLED, `ON W=30` ships.** Frame-exact A/B on `flight_20260906_165141` (319
frames; every run 64 kf / 38 RELOC, so genuinely matched), run as an **ABBA set** (ON,OFF,OFF,ON) to
cancel the run-order drift described below. Drift-cancelled: loop **364.3 s vs 405.9 s (ON 10.3%
faster)**, backend **233.2 s vs 265.1 s (12.0%)**. Median `backend_ms` on global solves:

| total kf | ON | OFF | |
|---|---|---|---|
| 0-9 | 1704 | 1697 | identical — the window is a no-op below `W` |
| 10-19 | 2241 | 2231 | identical |
| 20-29 | 2571 | 2196 | 17% worse (boundary: anchors appear, little cut yet) |
| 30-39 | 2201 | 3816 | **42% better** |
| 40-49 | 1931 | 4018 | **52% better** |
| 50-59 | 2316 | 4651 | **50% better** |
| 60-69 | 1896 | 4168 | **55% better** |

`ON` is flat (~1700-2600 ms) where `OFF` climbs 1697 -> 4651; the ordering holds in each of the four
individual runs, not just the means. **The whole-flight number is diluted** — this flight spends most
of its frames under 30 kf where the window does nothing — so the gain GROWS with flight length. Getting
here required a session-66 fix: the selection was asking CUDA four questions per solve (`ii.max()`, a
boolean-indexed `anchor_idx`, its `.numel()`, `mask.all()`), each a device->host sync draining the
queue and costing ~1.8 s per solve even below the window. The edge lists are ~1.3 KB, so they are now
pulled to the host once and every decision made there (0-9 kf: 2517 -> 749 ms).

**CAVEAT — measurement order is a large confound here.** Identical `OFF` code ran 409.2 s in run
position 1 and 496.8 s in position 2 (up to 4.5x slower at low keyframe counts). That is the same
signature originally mis-attributed to the window itself. **Never compare two sequential runs on this
machine without controlling order** — use ABBA (A,B,B,A) so a linear drift cancels. An ABBA set was
run to settle the verdict; see IMMEDIATE NEXT #0.

**The anchor leak is CONFIRMED on real data and is the deferred CUDA `num_fix` trigger.** At W=30:
`solve_kf` max **42** (= 30 window + **12 anchors**), `anchors>1` on **25 of 102** solve frames,
`anchor_drift` up to **0.489**. The `anchored` policy discards that motion at write-back, so those
anchors are optimised and the answer thrown away — a 43% overshoot of the bound, paid for and unused.
Fix design (make `num_fix` a kernel parameter; anchors are always the lowest sorted indices, so "fix
the first k" is exactly "fix all anchors", no reordering) is in `plans/session65-spec.md`'s deferred
section. Requires rebuilding the CUDA extension.

**Still true and unaddressed:** SLAM is slow. The flown `ON W=30` flight ran 2.74 s/frame with backend
at 64% of the loop, and `tracker_ms` grew 330 -> ~1800 ms within the flight again (third occurrence,
still unexplained — see `plans/slam_report.html` section 07).

## THE SLAM CHOKE — MEASURED (session 62). Stop guessing; the numbers are in.
`OUTPUT/diag/20260905_113348_perception.csv`, 446 frames, 35 min, voxels 3 220 → 386 558, keyframes
1 → 82. Phase closure residual median 0.1 ms, so the split is trustworthy. Of **2 010 s** in the loop:

| phase | total | share | what it is |
|---|---|---|---|
| **`backend_ms`** | **1 113 s** | **55.4 %** | `_run_backend()`: retrieval update + `add_factors` + `solve_GN_rays()` |
| `track_ms` | 602 s | 29.9 % | frame construction + the INIT/TRACKING/RELOC branch |
| `map_pub_ms` | 93 s | 4.6 % | `_map_payload` + publish (`topdown_summary` is O(map)) |
| `integrate_ms` | 39 s | 1.9 % | `mapstore.add_pose` + `mapstore.integrate` + `ground.integrate` |
| `plan_ms` | 29 s | 1.4 % | `_plan_payload` (13 raycasts, frontiers, `planner.select`) + publish |
| `pose_ms` + `kf_download_ms` + `publish_ms` | 2.9 s | 0.1 % | — |

Net throughput: **0.22 Hz** (446 processed frames in 35 min).

**On keyframe frames** (77 of 446): median `slam_ms` 9 818 ms = `backend_ms` 8 082 (**82 %**) +
`integrate_ms` 339 (3 %). Ordinary frames: 1 652 ms with `backend_ms` 0. So the ~6× keyframe tax is
almost entirely the backend, and this repo *chose* that — `slam_engine.py:38` records the deliberate
collapse of upstream MASt3R-SLAM's separate backend **process** into this one, which is what put
global optimization on the frame-critical path.

**Two findings that survived the measurement and still need explaining:**
- **`map_pub_ms`, not `integrate_ms`, is the CPU cost that scales with map size** — 4 → 70 → 138 →
  231 → **1 238 ms** across voxel buckets up to 386 k. Cheap to fix (pure CPU, no CUDA sharing) and
  worth more than `integrate` + `plan` combined.
- **`track_ms` is non-monotonic.** TRACKING-only frames: 367 ms (24 kf) → 1 820-3 078 ms (61-74 kf) →
  back down to 1 368 ms (82 kf). It rises 8× then partially *falls* while the keyframe graph keeps
  growing — the same shape as the old table's puzzling minute-25-30 recovery. This flight also ended
  in a hardware crash, so GPU/VRAM pressure is a live suspect. Current instrumentation cannot split
  it further; a finer split inside the tracker would be needed.

**Ruled out by this measurement:** CPU-side map integration as the driver of the choke (1.9 %), and
with it the premise of the original async-refactor proposal. Ruled out earlier: autopilot loop rate
(32-38.5 Hz throughout), Unity focus loss, and the clearance raycast.

**The historical degradation table** (autopilot-side `slam_ms`, 2026-09-04 flight, 30 of 47 minutes
blind — `HOLD_LOST` 1348 s + `FALLBACK` 471 s) is kept because it shows the *time* behaviour the
single-flight attribution above cannot:

| flight min | frames | median | p90 | max |
|---|---|---|---|---|
| 0–5 | 151 | 791 ms | 2 829 | 13 970 |
| 5–10 | 72 | 1 404 | 9 960 | 16 479 |
| 10–15 | 56 | 2 436 | 10 860 | 37 197 |
| 15–20 | 22 | 12 682 | 22 133 | 58 061 |
| 20–25 | 16 | 5 056 | 41 969 | **71 774** |
| 25–30 | 86 | 1 783 | 4 402 | 37 209 |
| 40–45 | 78 | 1 816 | 3 012 | 31 399 |

Worst wait between two solved frames: **72.9 s** at flight-minute 24.6.

**The three parked cures, now ranked by measured share** (full designs in `plans/session62-spec.md`):
- **Stage A — move `_run_backend()` off the frame-critical path** into a backend thread inside
  `slam_engine.py`. Targets **55 %**. The only change that can move `slam_ms` itself. Risky:
  `factor_graph`, `keyframes` and `states` are shared CUDA-backed structures on one stream.
- **Stage B — decouple `TOPIC_PLAN` publishing from the SLAM cadence.** Today a plan can only be
  published from inside `step()`, so `PLAN_PUB_INTERVAL = 0.5 s` is a lie: during a 12 s solve the
  autopilot gets no plan and no forward clearance at all. Must carry explicit `pose_age_s` /
  `pose_frame_id`.
- **Stage C — the original tracking/mapping thread split.** Targets **3.3 %**. If ever built:
  `MapStore._grow` *reallocates* `_count`/`_color_sum`, so a concurrent raycast is a **torn** read,
  not a stale one (needs a copy-on-write snapshot swap, not a "lightweight" lock); `clearance` and
  `planner.select` must stay on one side of the fence or a single `_plan_payload` mixes two map
  snapshots; and `flight_replay.py` / `salvage_flight.py` / the timeline all assume plan-frame
  correspondence.
- **New, cheap, unranked before:** `_map_payload`/`topdown_summary` at 4.6 % and growing.

## >>> IMMEDIATE NEXT <<<

0. **>>> DECIDE the bounded window's default, then fly it. <<<** Session 66 already did the bench
   the old item-0 described, and did it better: the **frame-exact replay** (`--frame-list`) is a
   controlled A/B that the old fixed-`--stride` procedure could not be — see the status section for
   why any fixed stride misrepresents live. What remains:
   (a) **DONE — the ABBA set settled it and `ON W=30` is now the shipped default** (see the verdict
   table in the status section). What remains is to **fly it**: every number above is replay, and a
   replay cannot show how a cheaper backend changes where the autopilot decides to go.
   (b) **Then decide the `num_fix` CUDA work** on the anchor-leak evidence in the status section
   (25 of 102 solves with `anchors>1`, max 12, a 43% overshoot of the bound). It is the difference
   between a window that truly bounds the solve and one that leaks — design in
   `plans/session65-spec.md`'s deferred section; needs a CUDA extension rebuild
   (`build_mast3r_slam.bat`), which is the risk.
   (c) `SHADOW` mode and `free`/`strict` policies are built and gated but still **never exercised** —
   worth one frame-exact run each if the window ships ON.
   (d) **Always press `r` when the io_bridge window opens.** Without the recording a flight cannot be
   replayed, and every future SLAM-speed lever (bf16, attention, resolution, RELOC throttling) needs
   frame-exact replay to be testable at all.

1. **>>> SLAM SPEED: the target is now `tracker_ms` + `infer_ms` (~72% of the loop). <<<**
   Session 64 settled the backend question and it is CLOSED: threading it works mechanically
   (`backend_ms` on the frame path 1634s -> 0s, no failures, no clobbers) but on a single GPU it does
   not remove the work, it lets the tracker compete with it -- `backend_thread_ms` p90 14.5s -> 26.2s
   once contention was removed, and both flights in that state lost tracking within minutes while the
   earlier, accidentally-serialised one flew 98 minutes clean. **`backend_async: false` is the default
   on evidence.** Do not re-open it without a 20+ minute flight; the median frame carries no backend
   cost in either mode (it fires on ~1 frame in 6), so short flights and medians cannot measure it.
   **Where the time actually is** (flight `20260906_000915`, `track_ms` split, session 63):

   | mode | `track_ms` | dominated by |
   |---|---|---|
   | TRACKING | 1109 ms | `tracker_ms` **945 ms (85%)** -- `tracker.track()` |
   | RELOC | 1783 ms | `infer_ms` **1762 ms (99%)** -- `_mast3r_inference_mono` |

   `frame_ms` is 5-8 ms; frame construction is nothing. Both hot paths are MASt3R ViT-Large forward
   passes at `img_size=512`. **Unexplained and high-value: `tracker_ms` grew 405 -> 5177 ms over one
   flight** with nothing in the tracker changing. Candidate causes, none tested: the warm-start
   `idx_f2k` degrading as tracking gets worse (more solver iterations), VRAM pressure (peak 7.35 GB of
   16 GB), thermal, or the retrieval DB. Attack plan in `plans/session64-spec.md`'s closing section
   and the shareable report at `plans/slam_report.html`.
2. **Parallax-push measurement: BUILT (watch-only) and PARKED at step 3.** Steps 1-2 are in:
   `traveled`, net cycle drift, distinct-pose count and a three-state verdict (moved / stuck /
   **unknown**) now ride the push-done event, the timeline row and the telemetry panel. Threshold is a
   fraction of `parallax_push_dist` (`push_stuck_drift_frac: 0.4`), never an absolute, since SLAM units
   have no metric scale. **Nothing acts on the verdict.** Validation: replaying the operator's corner
   trap gives three consecutive `stuck` verdicts on exactly the looping cycles and none on the free
   ones; flight `20260905_184034` was a clean negative control (13 moved, 7 unknown, **zero stuck** on a
   flight where the drone was never trapped). Caveat seen once: a `drift 6.245u` reading, almost
   certainly a RELOC pose jump rather than real motion -- if step 3 is ever built, drift must be robust
   to that. **Step 3 (trigger the existing guarded forward escape `reposition_fwd`) is deliberately NOT
   built** — operator's call: slow SLAM outranks it.
2. **FALLBACK reordering: BUILT + FLOWN 2026-09-05 — watch it.** The ladder is now
   `INITIAL_WAIT -> BACKOFF -> BACKOFF_WAIT -> TURN -> PUSH -> WAIT_POST -> TURN -> ...`, once per
   episode, aborting on `backwall_contact`. Flying it found and fixed two deeper bugs: `wants_visual_match`
   had the SIFT matcher switched OFF during PLAN-STALE (40% of a FALLBACK episode -- the root cause of
   SERVO never once working since session 60 built it), and the SERVO exit bailed on ONE bad sample
   against an entry gate needing 3 over 1.5s (`servo_lost_grace_s` now mirrors the entry). Step 0b also
   had to be re-sized: it first played the `back_off` recipe (reverse 0.2 for 0.3s, ~1/17th the sweep's
   own backward push) and was invisible in flight; it now uses `fallback_push_fwd_back_s` at full
   reverse. **Watch for:** `FALLBACK SERVO: held EQUAL for N solved frames` -- the intended give-up
   path, which fired for the FIRST time ever on `20260905_155834`. If it becomes common, revisit whether
   `servo_hold_frames: 3` (~38s at that flight's `slam_ms`) is too patient. Also watch the new telemetry
   panel rows, and whether the back-off is now visible from the cockpit.

3. **Review the session-61 panel watch list against the three 2026-09-05 flights.** Session 61 is
   flown but its items below were never checked off. Also unreviewed: `diag.ply_sequence` is ON and
   `OUTPUT/diag/20260905_113348_plyseq/` did fill — the marker-stability check was never run.
4. **Read the timing report after every flight from now on** — `fly.py` now passes `--log`, so the
   CSV always exists. `venv\Scripts\python.exe perception_timing_report.py` (no argument = newest).
5. **Decide on a choke cure** using the ranked stages above. Nothing is committed to yet.
6. **Housekeeping, offered and not yet done:** three July orphan flights (`20260720_133111`,
   `_135245`, `_135307`) still trip `fly.py`'s crash-recovery prompt at every launch. Moving them to
   `OUTPUT/diag/_orphans_2026-07/` silences it permanently and reversibly — the detector globs that
   directory non-recursively.

### Session 61's panel watch list (BUILT + FLOWN 2026-09-05, not yet reviewed)

1. **The LIVE half of the LKG panel moves continuously**; the F_LKG half changes whenever
   telemetry's `src=slam:<id>` changes — no more ~50s/8-solve freeze (the 22:40:29 failure).
2. Telemetry's `LKG=<src> age=<n>s` — the age should RESET to ~0 every time SLAM solves a fresh
   frame, not climb unbounded.
3. Green inlier lines drawn on the panel, steady at ~2 Hz once a loss episode matures past the 12s
   `loss_backoff_grace_s` window; DURING that grace the pair should still be live, with
   `lines=none (loss grace N/12.0s)` on screen — never a blank.
4. The info block is COMPLETE: `closer`, `scale`, `size`, `src`, `age` all legible, nothing clipped
   off the right edge; yellow "F_LKG (reference)"/"LIVE" labels visible on the panel.
5. **The panel must never grey out during `PLAN-LOST`/`PLAN-STALE`.** `LKG_CANVAS_STALE_S` is now 5
   minutes, so this should be near-impossible to observe; the guard exists as a dead-publisher
   backstop.
6. Kill switches, unchanged in meaning: `visrec_debug_window: false` still kills compose + publish +
   PNG saving outright (the one real SLAM-choke experiment); `use_visual_matching: false` now leaves
   an EXPLAINED idle panel (a startup line states why) instead of a silent grey one.
7. Flights still end by **manual stop**; no bounded-survey mechanism exists.
8. **`diag.ply_sequence` is ON** — `OUTPUT/diag/<ts>_plyseq/` should hold one `.ply` per fused SLAM
   frame + a `markers.json` (~1.2GB/flight, capped at `ply_sequence_max=2000`); the first five markers
   should sit at identical world coordinates in an early and a late frame.

### Carry forward, unchanged priority
Folded from session 60's now-flown watch list — see `PROGRESS.md`'s session 60/61/62 entries for what
those flights confirmed.

- **The SLAM choke is now MEASURED, not mysterious** (see the section above) — what remains open is
  choosing and building a cure, not diagnosing it.
- **FALLBACK's `SERVO` phase (session 60) is still unobserved on a flight.** The reordering in
  IMMEDIATE NEXT #1 touches this same ladder, so watch for both together.
- **Bump-pulse latency** unresolved — a blacklisted goal takes 10-18s to become visible to the
  autopilot (rides the next published plan; scales with SLAM latency). Three candidate designs
  sketched, none built: `plans/session58-lkg-window-discipline-and-dead-goal-guard.md`'s
  "Session-59 design sketch" section.
- **Staleness UI** — operator wants to discuss before it's built. Concrete case on record: at
  22:40:29 the top status strip showed `SLAM=TRACKING kf=27 slam=2921.5ms` from 15s earlier while
  perception was mid-solve on frame #153 (`slam_ms=15472.2`) — the only stale-looking field left in
  the dashboard now that the LKG panel is fixed. Session 62's Stage B (explicit `pose_age_s` /
  `pose_frame_id` on the plan) is the plumbing this would render.
- **Goal-management rewrite decision rule** (below, unchanged) still applies if goal problems recur.
- Future ideas, not yet built: **adaptive back-off strength** and closing session 47's **dead
  `_backoff_resolve_since` gate** (both detailed below).
- Everything under "Session 56/57's watch list" below is still current and unconfirmed.

**Operator's decision rule (2026-09-04), unchanged:** if flights are clean from here, stop and ship
the Blender/PLY presentation work. If goal problems recur, **rebuild goal management from scratch**
against a written behaviour spec — the operator's own judgement is that it is over-complicated, and
the evidence agrees: two independent death registries (`_blacklist` with soft/permanent/active, and
`_swept_corners` which deliberately ignores the first), four mechanisms writing the first (2-bump,
stall, loop, stagnation), a third `_goal_db` disc structure, plus `corner_no_blacklist_dist` /
`corner_giveup_limit` / clearance-inset carve-outs. Sessions 52, 58 and 59 each patched a *different*
hole in the same invariant and one broke another. Before any rewrite, build the carve-out inventory
("this exemption exists because flight X did Y") so nothing hard-won is dropped by accident.

**FUTURE CANDIDATES (deferred; not addressed by session 60 or 61):**
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

### Session 56/57's watch list (still current — the corner/bump-latch item resolved 2026-09-04, see `PROGRESS.md`)
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
