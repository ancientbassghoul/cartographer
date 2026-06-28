# Cartographer — Progress & Resume Handoff

_Last updated: 2026-06-29. Read this first when resuming with fresh context. **Where we are:** the
optical-flow CEILING+WALL detector is built/validated/wired; the autopilot is now a generic MISSION
RUNNER (`run_mission` + editable `mission_demo.json`) using `flight_playbook.json` recipes. **Live
status (2026-06-29): the FULL `mission_demo.json` FLEW LIVE — arm + takeoff + ascend + turn + forward,
all over the bus, working.** Earlier the fabricated takeoff duration (3.0s, just under the real 3.25s
climb) kept the drone grounded; the playbook was then HONESTLY RE-MEASURED from `mono_ts` (see the
FRAME-RATE bullet's 2026-06-29 CORRECTION) and the demo mission now works end-to-end. **Open tuning:**
turns overshoot ~90° slightly (as predicted — autopilot snaps yaw to 1.0 with no ramp; trim turn
`duration_s` toward ~1.85s); the USER is hand-tuning these. **Note:** bus-arming worked this run (the
old "ARM-over-bus broken" issue did NOT block it — possibly the failed takeoff masked a working arm
before; treat as resolved-pending-reconfirm). Active plan
`C:\Users\owner\.claude\plans\hazy-exploring-pascal.md`. **The "## NEXT" section is currently OPEN —
the user is deciding direction.**_

## What this project is (1 paragraph)
Assessment task: from the black-box **XLAB** Unity sim's single monocular drone feed, autonomously
**map the room and report the 3D location of a target object** (+ uncertainty). Three phases: Phase 1
Human Recon → Phase 2 Autonomous Survey → Phase 3 Localize & Report; then a GUI ("make it look like
an app"). Grading = internal consistency (metric scale NOT required); compute efficiency NOT graded.
All local on an RTX 3080 Laptop (16 GB).

## Where we are (2026-06-27)
**Phase 1** (the user's framing) = fly manually + SLAM-map the lab + record the flight path + detect
the target + report its 3D location. Every core piece is built and verified on hardware:
- **Sensing/transport** (io_bridge + frame bus), **SLAM + voxel map + live dashboard**, **depth** —
  all done and signed off live (see Milestones).
- **Target detector** — the hard problem; SOLVED by a **3-stage cascade** after every single-shot
  engine failed. Built, generalized, committed.
- **3D lift + consensus** — back-project each detection into the voxel map → robust 3D estimate.
  Built and verified offline; detector-agnostic.

The cascade is now **wired into the live app** (object_worker P4, `object_mode=CASCADE`), target
auto-classified at designation, verified offline E2E (`confident` 3D estimate, peak VRAM 9.68 GB).

**Two live runs done; the second one settled the architecture.** Run 1 (2026-06-26) flew smooth and
detection fired, but surfaced 3 issues (see "## Live-flight fixes"): B/C are fixed; A (detection
cadence) was confirmed by Run 2's `--log` CSVs to be **GPU contention** — see "## The GPU choke".
That finding is decisive: **SLAM and the cascade cannot share the one GPU**, which can't be patched —
it is solved architecturally by moving to **Phase 2 autonomy** and its Map/Scan state machine that
**temporally separates** the two GPU-heavy modes. We are now starting Phase 2.

## Live-flight fixes (2026-06-26 / -27)
The live 4-process runs revealed three problems (annotated shots in `DEBUG_IMAGES/`):
- **A. Detection slower than 2 s → DIAGNOSED: GPU contention.** Confirmed by Run 2's `--log` CSVs — the
  cascade and SLAM fight over the single GPU. Full diagnosis in "## The GPU choke" below. Not patchable;
  resolved architecturally by Phase 2 (Map/Scan temporal separation).
- **B. Flight path froze / didn't persist** — **FIXED**. Root cause: `perception_worker` recorded the
  trajectory **only on keyframes** (~20/flight, with a ~1.5 s keyframe-creation stall every ~52
  frames). Now records the camera pose **every frame** + publishes `TOPIC_MAP` on a 0.5 s timer (not
  only per keyframe), trajectory capped to 1500 pts for the bus. Verified: `traj_poses` = every
  processed frame (was 1/keyframe). Residual ~1.9 fps base rate is SLAM throughput (tied to A).
- **C. Target marker misplaced — actually TWO RIFLES → MULTI-TARGET (FIXED).** The lift CSV showed two
  near-tied clusters: `[0.03,0.03,3.38]` (55 hits, frames→source ~1287) and `[0.68,0.03,5.03]` (51
  hits, source ~2419). First read them as accurate-vs-inaccurate (far ray hitting the wall) and added a
  1/distance weighting "fix". **The user then revealed the lab has TWO rifles** (source frames ~1100 &
  ~2300) — the two clusters are the **two real rifles**, not one good + one bad. So the weighting was
  misguided (it just switched which rifle was reported). **Reverted it; replaced with multi-target:**
  `TargetEstimator.estimate_all()` returns EVERY well-supported instance via iterative peel-off
  mode-seeking (`CLUSTER_RADIUS` 0.30 u extent; `MIN_INSTANCE` 8 raw hits to report — filters
  incidental 4-5-hit blips; `MIN_CLUSTER` 3 only gates the `confident` flag). `TOPIC_TARGET` now carries
  a `{"targets":[...]}` list; the visualizer marks each (T0/T1…), and the offline export writes all
  instances + a `.ply` (cloud + green flight path + magenta target points). Verified: self-test (2
  instances + a blip + scatter → exactly 2) and replaying the real hits → **both rifles**
  `[0.025,0.025,3.425]` and `[0.675,0.025,5.025]`, each confident.

**Diagnostic logging:** `--log` on object_worker + perception_worker writes CSVs to `OUTPUT/diag/<ts>/`
(`object` = detection cadence/timing; `perception` = per-frame SLAM/loop timing; `lift` = per-hit
geometry + estimate). Used to diagnose B/C offline and to confirm the GPU choke (A) live.

## The GPU choke (decisive finding — why Phase 2)
The second live run with `--log` proved the **cascade and SLAM cannot coexist on the one GPU** (RTX
3080 Laptop, 16 GB). Both are GPU-heavy and run continuously in separate processes, so they interleave
on the same device and starve each other. Observed:
- **Perception collapsed to ~0.29 fps** with a **RELOC spiral** — SLAM kept dropping out of TRACKING
  because it couldn't get GPU cycles fast enough to match consecutive frames.
- **Poses went stale** (multi-second gaps between fresh poses), so detections lifted against
  out-of-date camera geometry → hits **scattered** instead of clustering on the rifle.
- The biggest hit cluster fell **below `MIN_INSTANCE`**, so the estimator reported **no rifle marks**,
  and the flight path stuttered (the dense-trajectory B-fix masks but can't cure the underlying stall).
- VRAM was **not** the limiter (peak ~9.7 / 16 GB, coexistence headroom confirmed offline) — the
  bottleneck is **compute/scheduling contention**, not memory.

**Conclusion:** no amount of cadence tuning fixes simultaneous SLAM + cascade on one GPU. The fix is
structural — never run them at the same time. Phase 2's **Map/Scan state machine** enforces exactly
that: in **Map** mode SLAM owns the GPU and explores; in **Scan** mode the drone stops, SLAM pauses,
and the cascade owns the GPU to fire at 45° increments. This is the architectural reason Phase 1 hands
off to Phase 2 rather than getting another patch.

## Phase 2 foundation — autonomous takeoff + ceiling-stall (BUILT this session, 2026-06-27)
First autonomy capability per the plan (`mossy-harbor`): a minimal closed loop **arm → gentle ascend →
detect the ceiling via a self-calibrating SLAM-stall → hold**. Proves programmatic control + SLAM pose
feedback; it is the skeleton for the later Map/Scan machine. Built + verified OFFLINE; **live dry-run +
closed-loop are the pending step** (need hardware / the user to fly).
- **HARD RULE added** (`CLAUDE.md` "CRITICAL AUTONOMY STANDARD" + memory `no-manual-flight-data-leakage`):
  NO manual-flight data leakage — every autonomous limit must be a GENERAL self-calibrating signal
  computed LIVE, never a constant lifted from a recorded flight. The ceiling detector obeys it: it
  calibrates its OWN `rise_rate_ref` live and compares the current vertical rate to it as a **scale-free
  ratio**. No constant encodes the ceiling.
- **`autopilot.py` (P5, NEW):** `CeilingStallDetector` (self-calibrating, scale-free) + a state machine.
  `--self-test` (7 synthetic cases incl. slow/large/small scale + ~2 fps sparse cadence + RELOC-suppress
  → ALL PASS), `--dry-run` (SUB TOPIC_POSE only, log the verdict while the user flies up — validation
  only), default = closed loop (ARM→ASCEND→HOLD), `--max-ascend-s` optional SAFETY abort (reported as a
  NON-detection, never a ceiling). Altitude = `−camera_center.Y`; stall logic trusts only `mode==TRACKING`.
- **Control bus:** `frame_bus.TOPIC_CONTROL` on `network.autonomy_control_port` (:5606). `autopilot` PUBs
  the FULL desired control vector at 20 Hz; `io_bridge` SUBs and applies it into `control_state` **ONLY
  while `autonomy_active`** (toggled by the `m` key; surfaced on the HUD + status as MANUAL/AUTO/AUTO(STALE)).
  **Safety, NO SILENT FALLBACKS:** any manual flight key **aborts** to manual instantly; a command older
  than `cmd_timeout_s` (0.5 s) **zeroes** the autonomous controls (no run-away on a dropped link).
- **`config.yaml` `autonomy:`** — general params only: `enable_key, ascend_cmd, arm_pulse_s, rate_window_s
  (1.5 s, sized for ~2 fps poses), stall_frac (0.15 ratio), rate_noise_floor (0.02 = "vertically stopped"),
  ceiling_stall_seconds (1.2 s), cmd_timeout_s`. None encodes a room/flight answer.
- **Verified offline:** detector self-test ALL PASS; control-bus round-trip + overlay apply/stale/abort/
  enable-toggle all PASS; full closed loop driven by a synthetic rise→plateau pose stream went
  **ARMING → ASCEND (ratio≈1.0 rising) → ratio collapses at the plateau → CEILING-STALL → HOLD**, ending
  neutral.

### Dry-run #1 (live) → detector FIX (2026-06-27, plan `i-don-t-think-it-piped-bee`)
First live dry-run FAILED two ways: (1) it fired `CEILING-STALL` **mid-air at alt≈1.5** when the manual
climb merely *paused* (the real ceiling was alt≈2.3) — a pure vertical-rate stall can't tell "pilot
stopped commanding up" from "hit the ceiling"; (2) the verdict **latched forever** (stayed CEILING-STALL
even while climbing again). **Fix (built + verified offline):**
- **Commanded-ascent gate + non-latching verdict** (`autopilot.py`): `CeilingStallDetector.update()` now
  takes `commanded_ascending`; a stall only counts WHILE ascent is commanded, and each commanded stretch
  is a fresh EPISODE (calibration + ceiling reset when the command stops → no stale latch, no pause
  false-fire). New label `NOT-ASCENDING`. Gating on the COMMAND is structural, not flight data (HARD RULE OK).
  Commanded signal: dry-run = pilot `joy_vertical==ascend_cmd`; closed loop = the autopilot's ASCEND state.
- **Plumbing:** `io_bridge` already put `controls` in the frame meta; `perception_worker` now **forwards
  `controls` + `rec_frame` into `TOPIC_POSE`**. (NO SILENT FALLBACK: if `controls` is absent the dry-run
  warns loudly and stays quiet — restart perception after this change.)
- **Recording-synced frame logging** (user request): `io_bridge` keeps a recording-relative `rec_frame`
  counter (0 at each `r`-record start, +1 per written video frame, `None` otherwise) in the meta;
  `autopilot --log` prefixes every line with it and writes `OUTPUT/diag/<ts>_autopilot.{log,csv}`. To see
  a log line's visual, scrub `OUTPUT/flight_<ts>.mp4` to that frame number. No overlay tool (not wanted).
- **Self-test now 10 cases ALL PASS** incl. the reproduced bug. BUT the self-test used DENSE/synthetic
  poses (20 fps + a fabricated "~2 fps" case) — which did NOT reflect the real feed, so it passed while
  the real thing failed again (below). Lesson: validate detectors against REAL captured data, not
  hand-made streams.

### Dry-run #2 (live) → the rate primitive is WRONG; PIVOT to vision (2026-06-27)
The fixed detector **still failed — it never armed once.** Real log/CSV
(`OUTPUT/diag/20260627_110606_autopilot.*`, 127 poses): `vert_rate` is `None` and `rise_ref=0` on nearly
every row, including ~18 s where the pilot held up and parked at the ceiling (`cmd=1`, `alt`≈2.279,
rows 108–114). Root cause, confirmed from the data + a code read:
- **Live SLAM pose rate is only ~1 Hz and DROPS to ~0.27 Hz at the ceiling** (rows 108–114 are ~108 NDI
  frames ≈ 3.6 s apart). SLAM runs every frame at ~1 s/frame and the frame bus is newest-wins (CONFLATE),
  so poses are inherently sparse + irregular. The least-squares rate needs ≥2 samples inside `rate_window_s`
  (1.5 s); at ~0.27 Hz the window holds ONE sample → rate `None` forever → never armed → never fired.
- **The slowdown at the ceiling happened in `TRACKING`, NOT RELOC** (`tracking_ok`=1 every plateau row;
  alt valid). So it is NOT a relocalization "death spiral" (a Gemini hypothesis the user raised) — likely
  just MASt3R per-frame cost rising on a near, low-texture surface. Either way it's **moot**: the fix is
  to stop depending on the slow SLAM pose for this primitive.
- **Pilot flies in pulses** (press `e` → coast up → release → fall back: rows 49–65 are a ballistic arc
  with `cmd=0`), so `commanded` is intermittent at sampled poses; the gate was right but the rate
  primitive is the wrong tool for ~1 Hz pulsed data.
**Decision (with the user):** abandon the SLAM-pose *rate/plateau* primitive. The right "am I still
moving?" signal is in the **camera image** at 30 fps (the sample `Sample_Drone_Interface.py` confirms the
ONLY Unity telemetry is `time` — everything must come from vision). Move ceiling/wall detection to
**optical flow** (dense, scale-free, SLAM-independent). The Gemini "Recovery Mode" (toggle RELOC / pause
mapping) is deferred to the later MAP-QUALITY work, not the detector.

### HARD RULE — refined boundary (with the user, 2026-06-27)
The discriminator is **room-specific answer (forbidden as a baked constant) vs platform/signal behavior
(legitimate to learn + use)**. LEGITIMATE: optical-flow SIGNATURES ("wall contact → image flows
vertically ~50 px/s", "ceiling → vertical flow → 0") and drone CONTROL DYNAMICS / maneuver magnitudes
("~N presses of `s` backs off a wall", arm key pattern). NON-LEGITIMATE: this room's geometry ("176
frames forward to the wall", "ceiling at Y=−2.3", "stop after 4.2 s"), which must be detected LIVE.
Prefer relative/self-calibrating use of even legitimate signatures where easy. (To be folded into
`CLAUDE.md` "CRITICAL AUTONOMY STANDARD" + memory `no-manual-flight-data-leakage` during implementation.)

### What's BUILT so far for autonomy (state at this handoff)
- `autopilot.py` (P5): **REWRITTEN** to consume the frame bus + the optical-flow contact detector (see
  "### Optical-flow contact detector"). The old SLAM-pose `CeilingStallDetector` is **GONE** (retired).
- `frame_bus.TOPIC_CONTROL` + `io_bridge` autonomy apply/abort/timeout + HUD flag — work (offline-tested).
  `io_bridge.AUTONOMY_FIELDS` now also includes `btnCdown` so the autopilot can pulse the attitude reset
  ('c') before a forward push.
- `io_bridge` `rec_frame` recording counter + `--log-keys` keystroke CSV + `autopilot --log` rec_frame-
  prefixed `.log`/`.csv` — work. KEEP these.

### Optical-flow contact detector — BUILT + validated + wired (2026-06-28, plan `hazy-exploring-pascal`)
Replaces the twice-failed SLAM-pose ceiling detector. Two events, ONE self-calibrating mechanism (a
collapse of the relevant flow signal while the matching command is held), all CPU-only (Farneback):
- **CEILING** (ascent commanded): vertical flow `|dy_med|` collapses from its live ascent level → ~0.
- **WALL** (forward commanded): the looming radial `expansion` collapses from its live free-forward
  level → ~0 — forward progress stopped. ONE signal unifies the user's freeze-OR-climb OR: a textureless
  wall freezes the image (`mag→0`), a textured wall shows a slow vertical climb; either way looming dies.
  (Root cause of the earlier miss: the analyzer's wall heuristic required `mag>0.2`, excluding freeze.)
- **Acceleration dead-zone guard** (per the user): a push from rest has flow≈0 for the first frames
  (inertia). Guards: (1) `arm_blank_s` onset blanking, (2) sustained `contact_seconds`, (3) the airborne
  latch below.
- **LIVE DRY-RUN #1 (2026-06-28, `20260628_002855_autopilot.log`) → ARMING FIX.** WALL fired perfectly
  live (looming→11.3→collapse→WALL). **CEILING never fired** — stuck in `PRE-MOTION`. Root cause: arming
  + the reference were **per-episode**; a later UP-press began with the drone already parked at the
  ceiling (frozen) → new episode reset the ref → it needs to *see* motion to arm but it's frozen →
  PRE-MOTION forever. **Fix:** arming is now a **flight-level `airborne` latch** (flips True the first
  time any motion exceeds the floor — takeoff) and the per-command reference (`ref_up`/`ref_fwd`,
  running-max) **persists across episodes**. PRE-MOTION is now TAKEOFF-only; once airborne, `commanded +
  flow collapsed (signal < stall_frac·ref AND < floor), held` → CONTACT — even on a push that *starts*
  blocked (re-pressing UP at the ceiling uses the takeoff-learned `ref_up`). Removed the per-episode
  windowed `calib_window_s`. (`armed`→`airborne` in the verdict/CSV.) **Replaying the failed flight
  `flight_20260628_002918` through the fix: CEILING now fires @1299 (was stuck), WALL @2709** — both
  correct. Self-test gained the frozen-start-after-takeoff + never-airborne cases (ALL PASS); 214625
  re-validate unchanged.
- **`flow_contact_detector.py` (NEW):** `FlowContactDetector.update(t, frame, command)` → `FlowVerdict`;
  resolution-normalized to `flow_long_side` (ratios are scale-free → HARD-RULE-safe). `--self-test`
  (synthetic signal streams incl. the dead-zone case, scale-free, episode-reset — ALL PASS) +
  `--validate --video --keys` (replays a real flight frame-by-frame vs labeled ground-truth ranges).
- **VALIDATED on `flight_20260627_214625` (real data, the dry-run #2 lesson):** CEILING@1543,1873
  (labels 1514-1551, 1845-1884); WALL@2993,6294 (labels 2966-3425, 6285-6514); **no contact within 12f
  of any push onset** (dead-zone holds). `learn_to_fly` (wall heuristic fixed) also flags both walls.
- **`flight_playbook.json` + `flight_playbook.py` (NEW):** control recipes as DATA (platform dynamics).
  presets (ascend/forward/hold), recipes (arm, reset_attitude='c', turn_left/turn_right, back_off), rule
  `reset_attitude_before_forward`. `RecipePlayer` steps a recipe over time; autopilot overlays it onto
  the 20 Hz control vector. **Recipes are now DATA-DRIVEN** (rebuilt 2026-06-28 from `learn_to_fly` on a
  clean labeled flight `flight_20260628_092640`, arm from `214625`):
  - **arm** = tap `btnARMdown` 0.25s → release 0.2s → hold 1.6s (the real DOUBLE-press; was a single hold).
  - **turn** = one continuous `yaw=±1` for ~3.5s → `btnCdown` reset. KEY: io_bridge does NOT decay yaw on
    release (it latches until `c`), so the pilot's "pulse then coast" is really continuous yaw until `c`;
    measured press-start→`c` ≈ 3.5s, very consistent. Open-loop; angle approximate; tunable. (Closed-loop
    yaw-from-flow rejected — textureless surfaces + no FOV/scale.)
  - **back_off** = hold `reverse` 0.7 for 0.6s (median of the 5 clean continuous-press back-offs; the
    multi-press set only cross-checked — it just sums to a bigger retreat). reverse 0.7 ≈ the impulse of
    a manual ramped `s`-hold (io_bridge applies the autopilot value directly, no ramp).
  - Schema documented in the JSON `_comment` (presets vs recipes vs rules; step = fields+`duration_s`;
    empty step = release; field meanings). autopilot `--self-test` updated for the multi-step arm.
- **`autopilot.py`:** SUB frame bus (`:5601`), not `TOPIC_POSE`. `--dry-run` derives the command from
  frame-meta `controls` and logs verdicts (NO controls). **LIVE dry-run flew CEILING + WALL accurately
  (2026-06-28)** — then the per-episode arming bug was found + fixed (see above).
- **MISSION RUNNER (`run_mission`, 2026-06-28):** replaced the hardcoded ARM→ASCEND→… state machine with
  a generic runner that flies an EDITABLE JSON script (`--mission`, default `mission_demo.json`). Step
  grammar: a playbook recipe name (`arm`/`takeoff`/`turn_left`/`turn_right`/`back_off`/`reset_attitude`),
  an until-keyword (`ascend_until_ceiling`, `forward_until_wall` — preset + flow detector; forward
  auto-prepends `reset_attitude`), or `{"rest": N}`. `rest_between_s` auto-inserted between consecutive
  non-rest steps; `max_contact_s` aborts the mission to HOLD if an until-contact step doesn't fire
  (non-detection). **The detector command is derived from the PUBLISHED control vector and fed every
  frame**, so the airborne latch + refs build during `takeoff` → a later `ascend_until_ceiling` fires
  even if the drone drifted up to the ceiling during the pause (frozen-start fix at mission level).
  Default mission = arm→takeoff→rest 3s→ascend_until_ceiling→turn_right→forward_until_wall→turn_left→
  forward_until_wall. Unknown step name → fail-fast. `config.yaml autonomy.flow` holds detector params;
  dead SLAM-pose params removed.
- **AUTONOMY GATE (fix, 2026-06-28 live run `112357`):** first live mission did NOTHING — the drone never
  armed. Cause: the mission started the instant autopilot launched, but io_bridge applies no autonomy
  command until the operator presses `m`; `arm`+`takeoff` elapsed BEFORE handover, so `btnARMdown` was
  never applied (`btnARMdown` itself is correct — sample maps `1`→btnARMdown; the autopilot sets it
  directly). FIX: the runner now HOLDS at the current step until io_bridge reports autonomy ON
  (`controls.autonomy != "MANUAL"`, read from the frame meta), publishing neutral while it waits, and
  PAUSES (restarts the current step) if autonomy is aborted. So you can press `m` whenever; the mission
  starts cleanly from `arm`. **Offline self-tests ALL PASS; LIVE mission re-run pending.**
- **`flight_playbook.json` `takeoff` recipe (NEW):** `joy_vertical:-1` (launch through the countdown) →
  short coast → `joy_vertical:+1` (arrest). Tunable. (Durations corrected below.)
- **FRAME-RATE BUG → all recipe durations were ~1.92x too long (fix, 2026-06-28 live run `121624`).**
  Live no-arm mission flew but every maneuver was ~2x (turns ~180° not 90°, takeoff near ceiling). The
  `--log` cmd CSV proved the autopilot executes recipe SECONDS correctly, so the error was in DERIVING
  them: the NDI/recording is **~58 fps, not 30**, but io_bridge tagged the mp4 `30.0` and `learn_to_fly`
  computed durations as `frames/30` → 1.92x inflation. Confirmed via the keys log's real `mono_ts`
  (turn press→`c` ≈ 2.0s, takeoff `e` ≈ 3.0s). FIXES: (b) `learn_to_fly.extract_maneuvers` reports real
  `duration_s` from `mono_ts` (frames kept only for scrubbing); (c) `io_bridge` tags the recording at the
  measured `cap_fps` (~58), not 30.

  **CORRECTION (2026-06-29): claim (a) above was FALSE.** The 2026-06-28 "fix" did NOT recompute the
  playbook from `mono_ts` — it edited `learn_to_fly.py` (b) but never re-ran it, and the playbook numbers
  were **hand-entered by dividing the old inflated values by ~1.92 and rounding** (3.0, 0.3, 0.2, etc.).
  The artifacts in `OUTPUT/learn/` still read `fps=30.0` with no `duration_s`, proving no re-run. The
  fabricated takeoff `3.0s` was just-under the real climb and the drone failed to leave the ground (dry-run
  `20260628_182159`); a manual probe found `3.1s` works. **Now properly re-measured** (`learn_to_fly` re-run
  on `flight_20260628_092640`+`092608_keys` and `flight_20260627_214625`+`214613_keys`, plus direct
  `mono_ts` analysis of the keys CSVs): **takeoff up 3.25 / coast 0.55 / arrest 0.17s** (the real `e`-hold,
  3.0 fails / 3.1 works / 3.25 measured), **turn 2.0s** (24 clean spin=press→`c` cycles, median 2.016,
  range 1.84–2.16; key-hold ~0.6s is NOT the turn — yaw latches until `c`), **back_off 0.3s** (median of
  5 continuous back-off presses; multi-tap ones reached ~0.6s/~2× retreat), **arm 0.11/0.08/0.80** (the one
  real double-press). All sourced in `flight_playbook.json` `_comment`. **LIVE re-run DONE (2026-06-29):
  the full `mission_demo.json` flew — arm + takeoff + ascend + turn + forward all worked over the bus.**
  Takeoff is now clean. Turns overshoot ~90° slightly, exactly as predicted (autopilot snaps yaw to 1.0
  with no ramp vs the pilot's ~0.33s ramp → rotates a bit more/sec); the user is hand-tuning turn
  `duration_s` toward ~1.85s. No code action needed there.

### Flight-characterization toolkit — BUILT + bootstrap-verified (2026-06-27, plan `i-don-t-think-it-piped-bee`)
The learning step before redesigning the ceiling detector. Capture manual flights as *video +
frame-synced keystrokes*, analyze offline to learn what optical flow does under each condition.
- **`io_bridge.py --log-keys` (NEW arg):** writes every keyboard EDGE (auto-repeat `down`s suppressed)
  to `OUTPUT/diag/<ts>_keys.csv` as `rec_frame, mono_ts, key, action`, stamped with the recording-
  relative `rec_frame` so the key log aligns 1:1 with `flight_<ts>.mp4`. Logging only — NO control-path
  change. (`DroneControl.current_rec_frame` set each main-loop iter; `_keys_down` set drops repeats.)
- **`learn_to_fly.py` (NEW, offline, CPU-only — OpenCV+numpy, no torch):** video + keys CSV (`--keys`
  now REQUIRED — the flow-only bootstrap mode was retired after it served its purpose; the tool's job is
  to CORRELATE flow with the commands that drove it) → per-frame dense Farneback flow scalars
  (`dy_med`/`dx_med`/`mag_mean`/`expansion`+framediff) timeline CSV + characterization JSON
  (`command_segments`, `candidate_states.{ceiling_contact,wall_contact}`, `maneuvers`). Heuristics are
  RELATIVE/self-calibrating + clearly labeled (HARD RULE: learning only, nothing baked back). Flow
  computed on a long-side-320 downscale (analyzer speed only; disclosed).
- **BOOTSTRAP VERIFIED** on `OUTPUT/flight_20260627_110627.mp4` (flow-only, no keys; artifacts in
  `OUTPUT/learn/`): **`|dy_med|` mean 0.452 / p90 0.810 during the climb vs EXACTLY 0.000 across the
  whole parked-at-ceiling plateau (rec_frame 1910–2467)** — the optical-flow primitive cleanly separates
  "moving up" from "at the ceiling", which the SLAM-pose rate could NOT. Ceiling-contact candidate fired
  at the climb→plateau transition [1866,1904]. The `--keys` path also smoke-tested (synthetic edges):
  arm extracted as tap(1f)+hold(10f), segments split by motion command, pre-record edges dropped.
- **HARD RULE refined in `CLAUDE.md` + memory `no-manual-flight-data-leakage`:** discriminator is
  room-specific ANSWER (forbidden as a baked constant) vs platform/signal BEHAVIOR (flow signatures,
  control-response magnitudes — legitimate to learn + use, prefer relative).

## The detector: a 3-stage CASCADE (the solution)
Single-shot engines all conflated *"where is a candidate"* with *"is this THE target"* and over-fired
(see "## Detector history"). The cascade separates propose from verify. It is **generalized** by an
`AssetClass` — no hardcoded target names:

- **Stage 1 — propose:** GroundingDINO (text phrase) + OWLv2 image-guided (reference crop), pooled,
  low thresholds, per-source NMS. (Stage-1 recall ceiling = 1.00 on both test targets.)
- **Stage 2 — verify:** DINOv2 ViT-S/14 global crop-embedding cosine vs the reference; ref AND each
  candidate crop **letterboxed** (aspect-preserving, never squashed) to 224². Gate = the asset
  class's DINOv2 threshold.
- **Stage 3 — geometric gate (per asset class):** `2D_PLANAR` → SIFT+RANSAC homography **HARD gate**;
  `3D_GEOMETRY` → LightGlue inliers **SOFT bonus** (never vetoes). Survivors ranked by **DINOv2
  cosine first**, geometry as tie-break.

`AssetClass` params live in `cascade_detector.ASSET_CLASS_PARAMS` (the only per-class config):
`2D_PLANAR` → DINOv2 ≥0.33 + SIFT hard; `3D_GEOMETRY` → DINOv2 ≥0.40 + LightGlue soft.

**Results** (9 poster + 13 rifle positives, 15 negatives — `OUTPUT/cascade/`): the **3D rifle, which
every prior engine failed** (best 0.23 good / 0.93 FP), reaches **0.77 good / 0.00 FP**; poster 0.33
good / 0.00 FP. **0 false positives across all negatives** — the asymmetric gate proved its worth:
even when the looser planar DINOv2 0.33 leaked negatives into Stage 2, the SIFT hard gate held final
FP at 0. **Warm per-frame ~1.55 s** (GD 0.44 + OWLv2 0.84 + DINOv2 0.08 + geom 0.18; all resident).

**Diagnostic scripts (kept, reproducible):** `cascade_detector.py` (CLI single-target or
`--targets cascade_targets.yaml` batch) + `cascade_report.py` (embedded HTML funnel). The earlier
`benchmark_detectors.py`/`benchmark_report.py` (5-engine bake-off) remain as the evidence base.

## Live integration (in progress this session)
Goal: cascade running live in `object_worker` (P4) firing every ~2 s, feeding the existing 3D lift +
consensus; target auto-classified at designation with user override. **The 3D side is unchanged** —
the lift only consumes `found/frame_id/center/target_label` from `TOPIC_DETECTION`, which the payload
still provides. Built:
- `cascade_detector.LiveCascade` — all cascade models resident; `set_target(ref,text,asset_class)` +
  `detect(frame)`. Explicit `torch.cuda.empty_cache()` after each init stage + per-frame (first live
  VRAM-coexistence test).
- `object_worker.py` — the three dead detectors (Qwen/OWLv2/old-cascade) **removed**; new
  `CascadeDetector` wraps `LiveCascade`; `object_mode` is **`CASCADE`** only; loads the designated
  target from `target.yaml`; payload + overlay carry `asset_class`.
- `target_classifier.py` (new) — one-time Qwen2.5-VL pass on the crop → suggested text phrase +
  PLANAR/3D class. **Designation-only**; the flight path carries no VLM.
- `make_target.py` — crops at **native resolution** (fidelity), runs the classifier, lets the user
  confirm/override label + class, writes `target.yaml` `{reference_crop, text, asset_class}`.

**Verification status (2026-06-26):** ✅ classifier sanity (rifle→3D_GEOMETRY, poster→2D_PLANAR,
labels correct — needed a "judge the medium not the depicted content" prompt or it called the poster
SOLID). ✅ **offline E2E** on the rifle flight (`flight_20260622_183816.mp4`, single-process SLAM +
cascade + lift + consensus): rifle found 119/188, 114 lifted to map hits, estimator
**`confident=True` @ [0.025, 0.025, 3.425], radial_rms 0.14u, spread_p90 0.22u**, **peak VRAM 9.68 GB
/ 16** (coexistence headroom confirmed), ~2 s/detection. ⏳ Live 4-process run pending (user flies).
Note: two hit clusters appeared (estimate settled on the denser earlier one at z≈3.4; near-approach
hits sat at ~[0.7, 0, 5.0]) — mode-seeking is internally consistent, but worth an eye on which is the
true rifle. Not chased (per "don't over-tune the honest read").

## ⚠️ Binding rules (from `cartographer/CLAUDE.md`)
- **NO SILENT FALLBACKS.** No auto-failover / hidden try-except downgrades. Fail-fast OR set a
  visible, logged, UI-surfaced state flag (`tracking_mode`, `object_mode`, `asset_class`). Any
  fallback must be approved before coding.
- **Image integrity:** no undisclosed downscaling; maximize source fidelity into each model and log
  every resize (letterbox→224², OWLv2→960², etc.).
- **Always start work with a TaskCreate list.** Never commit unless the user explicitly asks.
- **Checkpoint at milestone boundaries;** the user reviews each step.

## Architecture & data flow
4 processes over a ZMQ bus (frame bus = CONFLATE newest-wins; state bus = multipart `[topic][json]`):
- **P1 `io_bridge.py`** — NDI capture + 60 Hz TCP control + keyboard. Publishes the 512×288 transport
  stream (:5601), a **hi-res 720p stream (:5605)** for detection, and status/`space`-capture events.
- **P2 `perception_worker.py`** — SLAM (MASt3R) every frame + DA-V2 depth (throttled) in ONE CUDA
  context → voxel `MapStore`; publishes TOPIC_POSE/DEPTH/MAP (:5603). SUBs TOPIC_DETECTION, **lifts**
  each detection (`ingest_detection` ray-casts the center pixel into the map), feeds
  `target_estimator`, publishes TOPIC_TARGET.
- **P3 `visualizer.py`** — read-only dashboard: [status | input | depth+bar | top-down map + live
  camera track + magenta TARGET marker].
- **P4 `object_worker.py`** — the cascade detector; SUBs the hi-res stream, publishes TOPIC_DETECTION
  (:5604) with bbox/center scaled to 512×288 transport.

**KEEP — detector-agnostic, verified, reused by any detector (don't break):** the **3D lift** in
`perception_worker.ingest_detection` (pose ring + `MapStore.raycast`; geometry confirmed, center ray
≈[0,0,1]), **`target_estimator.py`** (mode-seeking cluster consensus + uncertainty), the hi-res
`:5605` stream, the `TOPIC_DETECTION`/`TOPIC_TARGET` bus contract, the visualizer target marker, and
`--debug-lift`.

## Environment & build (don't re-derive)
```
D:\EXTEND\C2_SIM\XLAB\
├── XLAB\          ← black-box sim (READ-ONLY). Xlab.exe, Sample_Drone_Interface.py, OUTPUT\*.mp4
└── cartographer\  ← our repo (this dir). Sim referenced as ../XLAB/
```
- **One venv:** `cartographer\venv` (Python 3.11.9, torch 2.5.1+cu121). All processes run from it.
- Re-validate models: `venv\Scripts\python.exe smoke_test_models.py` (DA-V2 + Qwen) and
  `smoke_test_slam.py` (MASt3R two-view).
- **lietorch is a PATCHED LOCAL build** at `third_party/lietorch` — upstream pip/git segfaults on CUDA
  group ops (`const scalar_t*` kernels miscompiled by nvcc 12.1 + MSVC 14.36). Rebuild via
  `build_lietorch.bat`; the tracked fix is `lietorch_windows_const_fix.patch`. NEVER `pip install`
  upstream lietorch. Validate: `lietorch_probe.py` ("ALL LIETORCH CASES PASSED").
- **MASt3R-SLAM** rebuild: `build_mast3r_slam.bat` then `build_mast3r_slam_step23.bat` (the int64_t
  kernel patch must already be present in `third_party/MASt3R-SLAM/.../backend/src/*.cu`).
- `.gitignore` excludes `venv/`, `third_party/`, `test_assets/`, `OUTPUT/`, weights.

## Key technical facts (don't re-derive)
- **Sim protocol** (`../XLAB/Sample_Drone_Interface.py`): Python is the TCP **SERVER**
  (127.0.0.1:65432); Unity connects as client. 60 Hz length-prefixed `control_state` JSON
  (trigger/reverse fwd-back, joy_horizontal strafe, joy_vertical altitude, yaw, pitch). Only
  telemetry back = `time`. Video = **NDI** 1280×720@30 BGRA. Keys: 1=arm, w/s, a/d strafe, e/f up/down,
  arrows yaw/pitch, b=land, c=reset cam; `space`=full-res capture, `g`=object-detect event. ('o','f' taken.)
- **MASt3R-SLAM API** (import ONLY these — never `mast3r_slam.visualization`, needs absent pyimgui):
  from `mast3r_slam.config` → `load_config`, `config`; `mast3r_utils` → `load_mast3r`,
  `mast3r_inference_mono`, `mast3r_symmetric_inference`; `frame` → `create_frame`. RGB = float32 [0,1]
  HxWx3. `os.chdir(REPO)` before loading (repo uses relative paths). Reference loop: `third_party/
  MASt3R-SLAM/main.py`; our streaming wrapper: `slam_engine.py` (INIT→TRACKING→backend+retrieval).
- **Driving the loop single-process:** mirror main.py but skip viz and use an `InProcessManager` shim
  (real `mp.Manager()` deadlocks on Windows). World pts = `kf.T_WC.act(kf.X_canon)`, conf-filter
  `kf.get_average_conf()`. Recover the 4×4 pose via **Act3 on origin+unit axes — NOT `T_WC.matrix()`**
  (matrix() routes through Act4 and under patched lietorch corrupts the frame pose, killing keyframes).
- **Ray geometry (lift):** camera per-pixel rays = normalized `X_canon` (intrinsics fixed → cached on
  `SlamEngine.ray_field`); world ray = `pose[:3,:3] @ ray_cam`; raycast skip 0.25u. Verified center
  ray ≈[0,0,1].
- **Resolution:** transport 512×288 (16:9). MASt3R's own resize makes 512×288 from 1280×720 — do NOT
  anamorphically squash. The cascade runs on the **hi-res** (720p/native) stream; its box is scaled to
  512×288 by `object_worker.Pipeline._to_transport` for the lift.

## Files in repo
- `config.yaml` — all settings (paths via ../XLAB/, ports, resolution, model ids, thresholds,
  `runtime.object_mode: CASCADE`).
- **io/transport:** `frame_bus.py` (+ `TOPIC_CONTROL`), `io_bridge.py` (+ autonomy apply/abort/timeout,
  `--log-keys` frame-synced keystroke CSV), `test_frame_subscriber.py`.
- **autonomy (Phase 2):** `autopilot.py` (P5: MISSION RUNNER `run_mission` over frame bus + flow detector;
  `--self-test` / `--dry-run` / `--mission <file>` / `--max-contact-s` / `--log` [writes `*_autopilot.csv`
  verdicts + `*_autopilot_cmd.csv` published commands]); `flow_contact_detector.py` (self-calibrating
  CEILING/WALL; `--self-test` / `--validate`); `flight_playbook.json` + `flight_playbook.py` (control
  recipes + `RecipePlayer`); mission scripts `mission_demo.json` (full), `mission_noarm.json` (skip arm —
  manual arm), `mission_basic.json` (skip arm+takeoff), `mission_arm_test.json`; `learn_to_fly.py`
  (offline optical-flow characterizer; maneuver durations from real `mono_ts` → `OUTPUT/learn/*`).
- **perception/SLAM:** `perception_worker.py`, `slam_engine.py`, `slam_offline.py`, `map_store.py`,
  `target_estimator.py`, `visualizer.py`; diagnostics `lietorch_probe.py`, `slam_match_probe.py`.
- **detection:** `object_worker.py` (live, CASCADE), `cascade_detector.py` (+`LiveCascade`),
  `cascade_report.py`, `cascade_targets.yaml`, `target_classifier.py`, `make_target.py`,
  `benchmark_detectors.py`, `benchmark_report.py`, `annotate_targets.py`. `target.yaml` = the
  designated target (written by make_target).
- **build/env:** `build_lietorch.bat`, `lietorch_windows_const_fix.patch`, `build_mast3r_slam*.bat`,
  `smoke_test_models.py`, `smoke_test_slam.py`, `third_party/` (gitignored).

## Live-run launch procedure (4 processes)
0. Designate the target once: `venv\Scripts\python.exe make_target.py [--video <recon.mp4>]` → crop +
   confirm class/label → writes `target.yaml`. Validate: `object_worker.py --self-test`.
1. Kill stray `perception_worker`/`object_worker`/`visualizer` (a stray PUB on :5603/:5604 makes a
   worker fail-fast on bind).
2. Xlab.exe → T1 `python io_bridge.py` (arm with 1; Admin if the keyboard hook is dead).
3. T2 `python perception_worker.py --no-display` (SLAM+depth+map+3D lift; SUB :5604, PUB :5603).
4. T3 `python object_worker.py` (cascade ~0.5 Hz; PUB TOPIC_DETECTION :5604; `--no-display` for headless).
5. T4 `python visualizer.py` (dashboard + magenta TARGET marker + uncertainty).
6. **(Phase 2)** T5 `python autopilot.py --dry-run --log` (observe only) or `python autopilot.py --log`
   (fly the mission `mission_demo.json`; press `m` on the io_bridge window to enable autonomy, any flight
   key aborts). Needs ONLY io_bridge running — it SUBs the frame bus (:5601) + drives TOPIC_CONTROL;
   CPU-only (Farneback), no SLAM/cascade concurrent. Detector = `flow_contact_detector.py`; control
   recipes = `flight_playbook.json`; mission script = `mission_demo.json` (`--mission <file>` to swap).
- VRAM budget: perception ~7.6 GB + cascade ~1.5 GB ≈ 9.1 GB / 16 (Qwen classifier is
  designation-only, not concurrent). autopilot (P5) is CPU-only (no GPU).
- **Offline E2E (no hardware):** `perception_worker.py --video <flight.mp4> --detect --debug-lift
  --no-display` runs SLAM + cascade + lift + consensus in one process. Rifle flight with the target
  in view: `OUTPUT/flight_20260622_183816.mp4`. Poster flight: `flight_20260621_120829.mp4`.

## Milestones
- **M1 env + all models on GPU** ✅
- **M2 io_bridge + frame_bus** ✅ (hardware-verified)
- **M3 depth (DA-V2) overlay** ✅ (live wall/glass fly-through signed off). Finding: DA-V2 reads glass
  as open air → an M5 glass detector must make SLAM-stall authoritative.
- **M4 SLAM + voxel map + map_store + live perception_worker + live dashboard** ✅ (fly-a-loop signed off).
- **Target detection (cascade) + 3D localize** — detector ✅; live integration ✅; two live runs ✅;
  flight-path (B) + target-placement (C) bugs ✅ fixed; detection cadence (A) ✅ diagnosed = GPU choke
  (see "## The GPU choke"). ← Phase-1 capstone; closed, hands off to Phase 2 by design.
- **DEFERRED to Phase 2:** glass + opening detectors (navigation safety, only autonomy needs them);
  live point-cloud save / 3D flight replay; then autonomy (planner, explore) + Phase-3 report polish + GUI.

## Detector history (compressed — don't re-try these)
Every learned/VLM single-shot detector was inadequate for this small-object, mural/clutter-competing,
low-texture-3D task: **Qwen2.5-VL-3B 4-bit** (non-deterministic, degenerates to `!!!` at 720p, boxes
murals not the poster); **OWLv2 image-guided** as a gate (scores any framed rectangle ~the same);
**Qwen→OWLv2 cascade** (recall too low); and a 5-engine benchmark (SIFT/LightGlue/DINOv2-dense/OWLv2/
Qwen) confirming the planar poster is solvable but the **3D rifle is unsolved by any single engine**.
That benchmark is the evidence base that motivated the verify-cascade above. Full numbers in
`OUTPUT/benchmark/` and git history.

## NEXT (OPEN — user is deciding direction, 2026-06-29)
The Phase-2 autonomy SKELETON is now flying end-to-end: **the full `mission_demo.json` (arm → takeoff →
ascend_until_ceiling → turn → forward_until_wall …) flew live over the bus.** The previously-blocking
issues are cleared: the fabricated-duration takeoff is fixed (honest re-measure), and bus-arming worked.
The remaining open item is small and OWNED BY THE USER: turns overshoot ~90° slightly (trim turn
`duration_s` toward ~1.85s — a one-value tune, NOT a code bug).

**No single forced next step.** Candidate directions to pick from when resuming (the roadmap toward the
Map/Scan machine that keeps SLAM + cascade off the GPU at the same time):
- **Map mode** — frontier-based exploration. `MapStore` needs a free/unknown/occupied layer (currently
  occupied-only) so the planner can choose where to fly next; the mission runner is the executor.
- **Scan mode** — stop, pause SLAM, rotate 360° in 45° steps firing the cascade (temporal GPU separation;
  the decisive Phase-1 finding, see "## The GPU choke").
- **Mission polish** — `back_off` after the final wall (one-line `mission_demo.json` edit); richer step
  grammar; recovery on a non-detection (currently aborts to HOLD).
- **Phase-3 report + GUI** — once survey coverage is good.

**ARM-over-bus — looks RESOLVED (reconfirm):** previously the drone wouldn't arm from a bus-driven
`btnARMdown` (only manual `1` worked), so we flew `mission_noarm`. On 2026-06-29 the full `mission_demo`
(which starts with the `arm` recipe) armed and flew over the bus. The arm recipe barely changed
(0.1→0.11s), so the earlier failure may have been MASKED by the broken takeoff (arm worked, but the
grounded drone looked un-armed). Treat as working; reconfirm on the next clean run before deleting the
`mission_noarm.json` / `mission_basic.json` fallbacks.

**HARD RULE (codified):** room-specific answers detected LIVE; platform/signal characteristics (flow
signatures, the `c`-before-forward rule, control magnitudes) are legitimate. In `CLAUDE.md` + memory.

**Deferred:** turn-angle precision (open-loop ~90° for now); a `back_off` after the final wall (one-line
`mission_demo.json` edit); Gemini's SLAM "Recovery Mode" → MAP-QUALITY; then the roadmap — **Map mode**
(frontier extraction; `MapStore` needs a free/unknown/occupied layer, currently occupied-only) ↔ **Scan
mode** (stop, pause SLAM, 360° in 45° steps firing the cascade) so SLAM + cascade never run together. The
mission runner + flow detector + playbook are the primitives that machine builds on.
