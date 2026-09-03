# Cartographer — Progress & Full History

**Read `STATE.md` first** — it's the concise, cheap first-read resume file (current status, the
live watch list, standing-rules pointer). This file is the full session-by-session history and
presentation record; read it when you need the "why" behind a past decision that `STATE.md`
compressed away. Full per-session technical design/trace lives in `plans/*.md`, linked below.

_Last updated **2026-09-03**, branch `all-bets-are-off`, session 56 continued: restructured this
file — split the concise current-state/resume-pointer content into `STATE.md` (the new first-read
file) and compressed the session log below from full multi-paragraph traces (never actually done,
despite CLAUDE.md's own long-standing rule to keep it this way) into one-liners, each still ending
in a `plans/*.md` pointer for full detail. No code changed this session._

## Session Log (newest first)

- **56** — SLAM was slow the whole flight; the settle gate needed 6 *consecutive* fast frames
  (arithmetically unreachable), so every hold escaped via the 15s dead-band instead of the gate.
  Separately, F_LKG's cache trusted `plan_valid` regardless of age, and on a ring age-out silently
  substituted the *live* frame for the frozen reference — so visual recovery matched a frame
  against itself, manufacturing false "too close" evidence that drove BACKOFF. Fixed: the settle
  gate now demands one solve *captured after* the gate opened (currency, not speed) + a
  release-grace stamp against a resulting limit cycle; F_LKG now requires `status=="OK"` and never
  fakes a substitute — degradation is visible instead (`LKG=STALE`, counters). Also made TRIM
  reachable directly from `SLAM_HOLD`. All 8 self-test suites green; one honest gap: no
  end-to-end `run_explore` test covers the exact site the age-out bug lived in.
  `plans/session56-settle-gate-currency-and-lkg-freeze.md`
- **55** — A live flight was lost to a GPU-driver TDR bugcheck (Unity on the iGPU, not SLAM/CUDA)
  mid-run. Investigation found almost everything had already survived (flush()'d per write); what
  broke was the MP4's missing frame index and a map backdrop only ever written at shutdown. Built
  `salvage_flight.py` (repairs the MP4 losslessly by borrowing a byte-identical VOL header from
  another finalized recording) + periodic map/log checkpoints (H1/H2/H3) so future crashes lose
  less. `plans/session55-crash-survivability.md`
- **54** — TRIM could enter under slow SLAM (session 52) but its WAIT-exit still required
  `not self._slam_slow` — arithmetically unreachable at a sustained ~2700ms, so TRIM parked 74s
  with no exit. Dropped the stale conjunct + added a bounded forced-exit backstop. Audit found 3
  more states with the identical unbounded-fast-streak-only exit shape (`CALIB_LOST_HOLD`,
  `CALIB_ESCAPE`, `POSTLUDE_LOST_HOLD`) — recorded as backlog, not fixed this session.
  `plans/session54-trim-wait-no-exit.md`
- **53** — `SLAM_HOLD`'s forced-hop rescue (15s) never fired because its clock reset on every
  re-entry, so a `HOLD_LOST`/`SLAM_HOLD` bounce never accumulated toward the 15s bar — 43 bounces,
  2m21s hover. Added a separate episode clock that only resets on genuine recovery.
  `plans/session53-slamhold-limit-cycle.md`
- **52** — Four stacked defects kept the 15° visual-recovery probe from ever executing on a real
  flight (a one-shot flag spent too early, `PLAN-LOST` had no dispatch case for an in-progress
  probe, F_LKG cached by tick-freshness instead of frame id). Fixed all four + gave the probe its
  own latch. Also removed a stale `not self._slam_slow` guard starving TRIM, and scoped a corner
  blacklist carve-out to soft/round-only after it let an already-permanently-blacklisted corner
  get re-flown. `plans/session52-lkg-recovery-unreachable.md`
- **50-51** — `SETTLE` had no upper bound on slow-but-alive SLAM (every sibling recovery state
  did) — 91.6s park. Gave it the same session-43 forced-resolve rule. Separately measured the
  autopilot loop rate (38.5Hz — not the SLAM-choke cause) and ruled out Unity-focus-loss as the
  choke theory (re-chokes ~2 frames after refocus) — the choke itself stays open, see `STATE.md`.
  Also cut wasted SIFT recomputation during a held-still loss (~380 matches/loss → ~24) via two
  gates + a lazy per-reference memo; deliberately zero behavior change.
  `plans/session50-settle-dead-band-escape.md`, `plans/session51-visual-match-on-demand.md`
- **49** — A wall got re-attacked across 5 legs / 8 minutes because three independent stall guards
  were each starved (a judged-hop precondition that never fired; a goals-DB disc frozen at
  creation while the live goal drifted off it; a `_best_dist` field read/reset everywhere but
  written nowhere). Unified the drifting disc, wrote `_best_dist` at commit, added a
  2-consecutive-no-progress stagnation blacklist. Also added an LKG debug window (F_LKG vs. live +
  drawn RANSAC inliers) for visual-recovery diagnosis. `plans/session49-goal-stagnation-and-lkg-window.md`
- **48** — Back-off fired ~70ms after `PLAN-LOST`, into losses that self-heal in <1.1s seven times
  out of eight (measured across 128 flight logs) — reacting to a blip, not to being stuck. Added a
  12s grace window a loss must outlive before it earns a physical reaction.
  `plans/session48-loss-recovery-grace.md`
- **47** — The back-off "loop" traced to SLAM never getting a chance to re-lock after a back-off
  (SETTLE's exit needed 6 fast frames while SLAM solved at 3.5s+, no timeout) — each `OK`/
  `PLAN-LOST` flip re-armed a check that never saw improved evidence. Built a post-backoff
  SLAM re-solve gate, unified the two back-off trigger paths onto one wedge counter, and made
  BACKOFF stop on `backwall_contact` instead of grinding a full 2s reverse into a wall.
  `plans/session47-post-backoff-slam-resolve-gate.md`
- **46** — Drone physically wedged (displacement-per-push collapsed to 0.000u). `BACKOFF`'s own
  body was wiped by a `PLAN-LOST` flicker one tick after entry (never got to command reverse), and
  `FALLBACK`'s escape sweep dispatched only from `PLAN-STALE`, never `PLAN-LOST`. Gave `BACKOFF`
  status-ownership, made `FALLBACK` dispatchable from `PLAN-LOST` too, added an escalation counter
  after repeated failed blind-backoff reflexes. `plans/session46-wedged-corner-escalation.md`
  (live-fly: chunk 1 confirmed, chunks 2-3 unreachable until session 47)
- **45** — Drone sat still 221s just 0.31u from its own goal because no guard applied: the planner
  had no minimum-pick-distance check, "goal reached" only lived inside `ADVANCE` (never entered),
  `PARALLAX_PUSH` was missing from the forced-hop grace window, and pick-dedup starved the loop
  blacklist since no hop ever completed. Fixed all four; an external review caught that a
  state-independent reached-check would have infinite-looped inside `SETTLE` — excluded it.
  `plans/session45-stuck-at-own-goal.md`
- **44** — Replaced TRIM's live-calibrated sag/high band with two hardcoded absolute `pos_y`
  thresholds — an explicit, flagged, operator-approved exception to the NO-MANUAL-FLIGHT-DATA-LEAKAGE
  rule, scoped to this branch only. Also gave `TRIM_RESUME_WAIT` the forced-resolve rescue
  `SLAM_HOLD` already had (was hanging forever under sustained slow-but-OK SLAM).
  `plans/session44-hardcoded-height-trim-thresholds.md`
- **43** (`main`) — Simplified `SLAM_HOLD`'s forced-hop rule to fire on any sustained OK-status
  hold, not just a plain mid-leg one — a recovery-settle hold used to be exempt and could wait out
  a 31.5s settle-gate oscillation instead. `plans/session43-slam-hold-forced-hop-simplification.md`
  — live-fly on 2026-09-01 confirmed sessions 20-43 fly tolerably; height still open.
- **42** — `VISUAL_RECOVERY`'s 15° probe had never once executed live — dispatched only from
  `PLAN-STALE`, but every real loss opened as `PLAN-LOST`, which had no in-progress-probe handling
  and swept it back to `HOLD_LOST` every time. Scoped the hand-off to `PLAN-STALE` deliberately
  (`PLAN-LOST` = a perception throughput problem, hold still; `PLAN-STALE` = perception alive but
  not tracking, a different viewpoint is a coherent remedy).
  `plans/session42-plan-lost-visual-recovery-scoping.md`
- **41** — Added a live distance-to-goal readout to the visualizer telemetry panel (operator ask,
  no bug behind it). `plans/session41-visualizer-goal-distance.md`
- **40** — Replaced TRIM's pitch-aim+forward-push+ring-gate mechanism with a direct short vertical
  pulse, reusing `DOCK_FLOOR`'s already-validated pulse→settle-gate pattern instead of the
  continuous push session 14 had found choked SLAM. `plans/session40-trim-vertical-pulse.md`
- **39** — Removed `RETURN_TO_ORIGIN`'s `BACKOFF` sub-phase (operator's call — homing always
  orients first) and fixed `DONE` missing from `POSTLUDE_STATES`, which let a plan-loss after
  mission completion resurrect the entire explore FSM instead of quietly resuming `DONE`.
  `plans/session39-return-to-origin-backoff-removal-and-done-loss-fix.md`
- **38** — Added `desired_height_override_y`, an explicit opt-in operator override of the
  live-calibrated flying height (for repeatable test flights) — flagged as a deliberate, visible
  exception to NO-MANUAL-FLIGHT-DATA-LEAKAGE. `plans/session38-desired-height-override.md`
- **37** — Replaced the visualizer's dead depth panel with a live FSM-state/height/plan telemetry
  panel. `plans/session37-visualizer-telemetry-panel.md`
- **36** — Built image-based visual recovery for `PLAN-STALE`: a loss-instant SIFT/RANSAC check
  against a cached "last known good" frame, plus a first-class 15°-step turn probe before falling
  back to blind `FALLBACK`. Default off. `plans/session36-visual-recovery-15deg-probe.md`
- **35** — `_recovering` could never clear because the confirm-distance anchor it needed got wiped
  on every hop boundary, not just a genuine recovery — simplified to clear on a settled `OK`.
  Added a config-switched alternative to the SLAM-slow step-back (forced-hop-toward-goal after
  30s), default off. `plans/session35-slam-slow-strategy-switch-and-recovering-fix.md`
- **34** — Nothing watched wall proximity while the drone held (only `ADVANCE` ran the clearance
  stand-off, and being close to a wall degrades SLAM into holds). Added two proactive checks: back
  off the instant a post-recovery settle-gate clears, and cache the last-known-good clearance
  every tick so a fresh loss can react immediately instead of waiting out a blind period.
  `plans/session34-proactive-clearance-while-blind.md`
- **33** — A permanently-blacklisted goal kept getting re-picked because the clearance inset (which
  walks a goal back toward the drone) ran AFTER the exclusion check, so an inset point could land
  back on a dead cell unchecked — 49 picks despite a correct 3-pick blacklist. Re-check exclusion
  against the post-inset point. `plans/session33-goal-loop-clearance-inset-fix.md`
- **32** — `ORIENT_HOME` quantized its turn to whole 30° steps even for a small residual error, so
  it could oscillate ±30° forever around a ~15° true heading error. Turn by the real (clamped)
  bearing error instead + an explicit convergence tolerance and give-up cap. Also added
  `HOME_REFINE` (position-tightening via short pulses) and a real settle-gate for `DOCK_FLOOR`.
  `plans/session32-orient-home-ping-pong-and-home-refine.md`
- **31** — Killed REWIND (operator: never once saw it help — gated off by default) and rebuilt
  `FALLBACK` as a simple wait→turn 15°→push-fresh-random-direction→wait sweep, replacing a
  locked-direction search that tested badly live. `plans/session31-rewind-off-simple-fallback-sweep.md`
- **30** — BACKOFF's reaction was structurally too slow — io_bridge's throttle smoothing (shared
  with manual flight) ate a chunk of its short reaction window, and manual testing showed the
  platform itself needs ~2s of held reverse to actually move. Rebuilt BACKOFF as a phase-timer with
  a hard thrust-gate override and a full-magnitude 2s reverse. `plans/session30-backoff-hard-gate.md`
- **29** — A ~113s stuck episode traced to `FALLBACK`'s push direction being picked from a stale
  pre-loss ring snapshot that never refreshed, repeatedly pushing back into the same wall as
  heading drifted. Replaced with a direction-cycling sweep (shuffled queue, live wall-contact
  early-exit). Also fixed a fallback-attempts cap only checked from one re-entry path, and added a
  Clearance-detail debugger tab. `plans/session29-clearance-tab-direction-cycling-fallback.md`
- **28** — The drone kept flying at a goal it had JUST blacklisted because TRIM's abort path
  restored a pre-blacklist goal snapshot and re-aimed without ever going back through REPLAN's
  blacklist check. Rebuilt TRIM's exit to hand off through a settle-gated `TRIM_RESUME_WAIT` that
  re-validates the goal. Also tightened clearance's ray-vote from MIN-over-fan to a hit-fraction
  vote. `plans/session28-trim-resume-gate-clearance-vote.md`
- **27** — Live flights never actually exported the SLAM point cloud or the recorded MP4 cleanly,
  because `fly.py` hard-terminated perception/visualizer, skipping their `finally:` blocks. Gave
  both a graceful stop-file shutdown. `plans/session27-video-recording-pointcloud-export-graceful-shutdown.md`
- **26** — Homing had no back-off reaction to a wall (only explore's `ADVANCE` did) — added one.
  Also fixed a settle-gate that could "pass" having seen zero frames captured after the maneuver
  it was judging, a postlude recovery-streak gate that could never satisfy under flickering SLAM,
  and a pick-dedup bug suppressing every one of 40+ genuinely-completed hops as sub-steps of one
  leg. `plans/session26-homing-backoff-settle-freshness-pick-dedup.md`
- **25** — Diagnosed 7 operator-flagged issues from one flight's raw timeline; fixed 3 real bugs (a
  lossy single-slot event mailbox that could silently drop a bump/strike message during a slow
  solve; blind wall contact during a hold never reacting since the back-off check only ran inside
  `ADVANCE`; a stepback counter that reset on every hold re-entry so it could never escalate).
  Added manual TRIM key-macros, a goals-DB mechanism-split schema, clickable event-log navigation.
  `plans/session25-trim-macros-recovery-fixes-goaldb-schema-debugger-nav.md`
- **24** — Rebuilt the settle-gate as a rolling two-gate design (freshness + physical-motion-dwell,
  decoupled) after finding a single streak counter conflated "SLAM is healthy" with "the airframe
  has rested long enough." Also fixed `LOOP-BLACKLIST` firing on a multi-step turn's own re-orient
  sub-steps, and made the far-corner exemption bounded/proximity-scaled instead of a flat distance
  with an unbounded counter. `plans/session24-settle-gate-pick-dedup-corner-giveup.md`
- **23** — The flow-based BACKWALL detector was detection-only — logged but never acted on — so
  `PARALLAX_PUSH` kept picking "backward" off a stale/unmapped ring reading and running its full
  reverse timer into a wall it could clearly sense. Wired `backwall_contact` into a real decision.
  `plans/session23-backwall-reaction-and-parallax-retry.md`
- **22** — A per-goal ceiling re-tap firing in a SLAM-hostile corner caused a ~2.25min calibration
  death-loop ending with the drone glued to the ceiling. Root cause: the periodic re-tap assumed
  height needed re-measuring per goal, but SLAM's height read is actually stable within a flight.
  Made the first-takeoff calibration THE flight reference (periodic re-tap off by default), made
  TRIM bidirectional (was UP-only). `plans/session22-fixed-height-ref-and-bidirectional-trim.md`
- **21** — Restored the periodic height re-calibration + gradual TRIM + height debugger panel that
  session 17 had deleted (believing the sag was purely the unset `triggerDown` boolean) — live
  flights proved real sag remained. `plans/session21-restore-height-calib-and-trim.md`
- **20 (REV)** — De-committed per-hop goal targets (post-hop re-reads SLAM's current pick via
  REPLAN instead of finishing a stale committed leg) to fix goal ping-pong while keeping SLAM free
  to re-pick; added a persistent goals database (0.5u discs, pick-count-based permanent
  loop-blacklist); made far corners exempt from bump/blacklist. `plans/session20-goal-db-loop-blacklist.md`
- **20b** — A goal kept re-picking itself forever because the leg-stall guard fired the instant
  `ADVANCE` began (before any command was even sent), and neither blacklist path caught a
  stationary re-pick. Made "stall" a measured consequence (distance closed since entry) and split
  the goals-DB guard into three complementary triggers (2-bump / strikes / clustered-picks loop).
- **18** — Autonomous flight was height-erratic (hard brake + pitch-up on every stop) while manual
  felt smooth — traced to the autopilot bypassing io_bridge's own ramp/gate smoothing entirely.
  Made throttle a ramp target through the existing loop (yaw/pitch deliberately left un-ramped,
  since the sim's yaw is duration-not-magnitude). Also fixed a height-median ingesting ~25
  duplicate stale samples per real SLAM frame. `plans/session18-command-smoothing-and-height-median.md`
- **17 — THE BIG ONE** — Months of "crawl" and height sag traced to Unity gating real thrust on a
  `triggerDown`/`reverseDown` BOOLEAN the autopilot had never set (only driving the analog value),
  so every autonomous forward/reverse ran with the gas effectively unpressed. Fixed centrally in
  `_full_vector`; deleted the now-pointless periodic height-recalibration + TRIM machinery it had
  been fighting a self-inflicted sag with. `plans/session17-triggerdown-and-height-simplification.md`
- **16** — Return-to-origin fell apart (spun, thrashed, no still windows for SLAM) — generalized
  session 15's settle gate into a shared primitive and put a settle between every recovery/homing
  action; built the full return-to-origin ending (`ORIENT_HOME`, `POSTLUDE_LOST_HOLD`, graceful
  dock). `plans/session16-settle-between-stages-and-return-to-origin.md`
- **15** — Six fixes off one TRIM flight: inverted TRIM pitch sign, a bounded `CALIB_ESCAPE` for an
  endless calibration retry loop, a SETTLE that could fly on a stale pose (now requires 6 frames
  captured after settle began), plus smaller fixes. `plans/session15-trim-and-settle-fixes.md`
- **14** — Built a gradual height TRIM using the sim's pitch-aim (pitch up + push forward = gentle
  climb with SLAM-friendly parallax, unlike a discrete full-thrust vertical pulse that chokes
  SLAM), triggered when `pos_y` sinks past a calibrated ceiling-relative threshold. Diagnosed (not
  built) the return-to-origin ending and a 2-minute glass-wall bounce. `plans/gradual-height-trim.md`
- **13** — A transient `PLAN-LOST` during a per-goal ceiling re-tap erased all memory of being
  mid-calibration, skipping the descend and leaving the drone glued to the ceiling for the rest of
  the flight. Built `CALIB_LOST_HOLD` to survive a loss mid-calibration and redo it once SLAM
  genuinely recovers. `plans/crystalline-swimming-floyd.md`
- **12** — A full-magnitude strafe into an unmapped wall scraped and spun the drone, killing SLAM
  (invisible in the pose log, since the frozen pose masked the real rotation) — then the recovery
  FSM could never terminate because a flickering SLAM status reset its progress/give-up counters
  every ~3s. Throttled the strafe, added a "don't trust re-lock until confirmed motion" rule, made
  recovery persist across the flicker. `plans/strafe-throttle-and-recovery-loop.md`
- **11** — The height-calibration bug was judging the result AT the ceiling tap instead of after
  the drone settled, against too few samples to define "normal." Built `CALIB_VERIFY`: judge the
  settled result against a continuously-updated baseline of normal flying altitude, frozen during
  calibration. Also fixed a replay-timestamp bug and added the one-command `fly.py` launcher.
  `plans/height-calib-state-gate-and-slam-debug.md`
- **10** — Reconstruction was uneven (drone only flew one diagonal) — generalized the single
  opposite-corner sweep into an all-corners tour, and added a floor-dock postlude (mirrors the
  two-phase ceiling ascent) ending in a graceful hover instead of a ceiling hang.
- **9** — The done-verification stage could silently fall through to an idle no-op if a fragile
  distance gate failed. Replaced with a deterministic bbox-diagonal sweep that always yields either
  a new frontier or a visible DONE. Also built per-goal height re-calibration.
- **8** — "Turns are broken" turned out to be a logging lie — the timeline logged perception's
  async plan instead of what the controller actually acted on, and SLAM's ~2Hz heading update made
  a real ~1s turn look motionless in the log. Fixed the log to show the committed goal + explicit
  staleness.
- **7** — A glass corner trapped the drone forever via two coupled bugs — the verify-target
  fallback never consulted the blacklist, and the clearance stand-off wasn't triggering the
  bump-latch re-arm. Fixed both; added a clearance buffer so committed goals aren't pinned to the
  wall itself.
- **6** — Blacklists fired falsely because the "ram guard" assumed a fixed nominal speed the
  drone's actual crawl never reached. Made it self-calibrating (measure real free-flight speed
  live, fire only well below that).
- **5** — Removed all depth-based height logic (freed the GPU for SLAM) and replaced a continuous
  full-thrust ceiling ascent (built momentum, hit the ceiling hard) with a two-phase micro-pulse
  approach.
- **4** — An unreachable-goal watchdog was a time-accumulator gated on SLAM health, which stayed
  frozen (never fired) in exactly the glass-wall pockets where SLAM stays healthy on a false track.
  Replaced with an event-driven 2-bump blacklist, immune to SLAM-clock health.
- **3** — Built `flight_replay.py`, the animated HTML flight debugger.
- **2** — Corrected an earlier wrong mental model of "glass-stuck" (SLAM actually stays healthy
  looking through glass — the real problem is an invisible collider) and fixed a blind 360°
  startup sweep + added a position-space ram guard.

### Earlier (2026-06-27 → 07-05)
- **Ceiling detector v1** (SLAM-pose rate/plateau) **failed twice live** — monocular pose is only
  ~1 Hz, so the rate window never armed. **Lesson: validate detectors on REAL captured data, not
  synthetic streams.** → pivoted to `flow_contact_detector.py` (CPU optical-flow, self-calibrating):
  CEILING = vertical flow collapses while ascending; WALL = radial looming collapses while moving
  forward. Validated on real flights.
- **Turns vs SLAM:** closed-loop-on-heading thrashed (heading goes stale mid-spin); a "pulsed" yaw
  was wrong (yaw latches). Settled on **open-loop quantized turns clamped to ≤45°**.
- **Ramming a wall kills monocular SLAM** (no parallax freezes the image); reversing a dead track
  can't revive it. → the **forward-clearance stand-off** (SLAM raycast) is the primary wall stop.
- **Frontier planner** (`frontier_planner.py`): utility selection + strong commitment +
  done-verification — fixed goal thrash and false "mission complete".
- **Control-space SLAM-loss recovery**: `PLAN-LOST` → hard hover-hold; `PLAN-STALE` → replay the
  inverse of recent maneuvers; history empty → a bounded ≤45° fallback sweep → STUCK.
- **The unreachable-goal saga**: several dead ends (position-conditioned watchdog ping-ponging,
  round-based blacklist, distance-stagnation timer) before session 4's event-driven 2-bump rule
  finally held.

_Older per-flight watch checklists for sessions 2-20 (what to confirm live on each specific flight)
are superseded by the 2026-09-01 confirmation that sessions 20-43 fly tolerably as a bundle — full
text recoverable from git history of this file if ever needed._

---

## Next (resume after a context clear)

**Moved to `STATE.md`** — read that file first; it has the live watch list for session 56's
pending live-fly and the `main`-branch HEIGHT-issue pointer, kept current instead of duplicated here.

---

## Future (backlog)
- **Session 56 backlog — recorded, deliberately not implemented (operator's explicit call to defer):**
  - **`visrec_min_inliers`=12 is too low to trust a homography *scale* for a physical BACKOFF
    reaction.** The `17:21:40.711` BACKOFF on the diagnosing flight was decided on ~19 inliers whose
    scale swung 0.65→27.4 within half a second. Session 56's chunks 4+5 removed the *fake* reference
    that made a bad match look self-consistent; they did not raise the trust bar on a real match.
  - `SETTLE`'s dead-band escape measures `self.t_state`, not an episode clock (`autopilot.py:4217`) —
    the same shape session 53 already fixed for `SLAM_HOLD`. Low priority now that the escape stops
    being the *normal* path (session 56's currency gate should make it rare).
  - Session 54's four streak-gated states (see the dead-band class entry right below) are untouched
    by session 56: `CALIB_LOST_HOLD`, `CALIB_ESCAPE` (still has no bounded escape at all),
    `POSTLUDE_LOST_HOLD`, and the legacy `use_slam_stepback_on_slow=True` arm.
  - **GATE A allows exactly one visual match per loss episode** (session 50/51) — correct for session
    48's 2.4s median loss, blind for this flight's 71s and 89s ones. Needs its own session and a
    decision about what a *second* look during a long loss is even for.
  - **The SLAM choke itself is still undiagnosed** (max solve 90.8s on this flight; leading theory
    below is MASt3R-SLAM's workload growing with the keyframe graph). Session 56 is damage control
    around the choke, not a cure — see `STATE.md`'s "STILL OPEN, TOP OF THE LIST".
  - **A `run_explore`-level integration test is missing.** Session 56's revert-proof exercise found
    that reverting the F_LKG ring age-out fix at its actual integration site (inside `run_explore`,
    not unit-testable the way the FSM `step()` helpers are) produced zero self-test failures — the
    four `F_LKG AGE-OUT` self-test blocks cover the surrounding mechanics only. Worth a harness that
    can drive `run_explore` itself if this bug class recurs. See
    `plans/session56-settle-gate-currency-and-lkg-freeze.md`'s revert-proof matrix for the full trace.
- **The `_slam_fast_streak` dead-band class — three remaining sites (found session 54, not fixed).**
  Same signature each time: the ONLY exit predicate is `_slam_fast_streak >= N` or `not
  self._slam_slow`, with no wall-clock cap and no `has_any_capture` blackout guard — grep for that
  pattern to find more. All three are dispatched ABOVE `step()`'s status router and own every status
  themselves, so unlike `TRIM` (fixed session 54) not even a genuine `PLAN-LOST` can rescue them:

  | State | Exit predicate | Why it can't fire at ~2700ms/frame |
  |---|---|---|
  | `CALIB_LOST_HOLD` (`_step_calib_lost`, `autopilot.py:1680`) | `_slam_fast_streak >= calib_lost_recover_frames` (6) + `status == "OK"` | `_slam_fast_streak` resets to 0 on every slow frame, pinned at 0. `calib_gate_max_s` bounds only the downstream comfort sub-gate, never reached. Docstring: "No time cap — the SLAM frame stream is the liveness signal (operator ask)" — that premise assumes slow eventually turns fast; this flight class disproves it. |
  | `CALIB_ESCAPE` (`_step_calib_escape`, `autopilot.py:1791`) | `_slam_fast_streak >= calib_escape_ok_frames` (12) + `status == "OK"` + `_calib_slam_comfortable()` | Same, stricter (12 fast frames + a latency-average bar). Comment: "no extra bound needed here". |
  | `POSTLUDE_LOST_HOLD` (`_step_postlude_lost`, `autopilot.py:1879-1881`) | `_slam_fast_streak >= required_streak` + `status == "OK"` | `postlude_recover_budget_s` (30s) relaxes the required streak 6→1 but NOT the speed bar — a fast frame is still required and there are none. Residual, not absent. Hangs at mission end, near the ground. |

  Plus one **latent** instance (off by default, so not yet bitten anyone): the `SLAM_HOLD` legacy
  `use_slam_stepback_on_slow=True` arm — the sessions-43/53 rescue lives only in the `if not
  self.use_slam_stepback_on_slow:` branch; the legacy arm returns "keep holding" with no clock at
  all. The un-fixed twin of a known-fixed bug, waiting for whoever flips that flag.

  Each fix should mirror `TRIM`'s session-54 idiom (or `TRIM_RESUME_WAIT`'s session-44 original): a
  bounded forced exit on `slam_slow_hop_after_s`, guarded by `has_any_capture` so a total capture
  blackout still isn't papered over by a wall clock. `CALIB_LOST_HOLD`/`CALIB_ESCAPE` both have an
  explicit "operator ask: no cap" comment on record — re-confirm the premise with the operator before
  changing either, since the no-cap choice was deliberate at the time, not an oversight (unlike
  `TRIM`'s, which was). See `plans/session54-trim-wait-no-exit.md` for the full audit table this was
  drawn from.
- **Session-22 — fixed height reference + BIDIRECTIONAL TRIM; mid-flight re-tap RETIRED — BUILT
  (`leg-hops-and-goal-commit-fix`), all 6 module self-tests green, UNCOMMITTED, LIVE-FLY PENDING**
  (`plans/session22-fixed-height-ref-and-bidirectional-trim.md`): the 20260717_004418 calibration death-loop +
  glued-at-ceiling diagnosis; `calibrate_on_goal_change: false` (first-takeoff `desired_y` = THE flight
  reference — SLAM Y stability confirmed in the log); TRIM DOWN (`trim_high_ratio` 0.2, mirrored pitch +1.0);
  `trim_aim_s` automatic (0.5 s platform constant); SLAM-COMFORT gate (`calib_slam_avg_ms` 666 on a 10-frame
  healthy-latency window, `calib_gate_max_s` 30 → escalate WITHOUT redo); Y-DRIFT audit posture (re-enable +
  `calib_cooldown_s` 600; non-first PASS logs the ceiling movement); PASS latches `target_altitude_y`; LOUD
  once-per-flight median-vs-desired disagreement notice; debugger trim-at-high/low band. **NEXT = live-fly.**
- **Session-21 — periodic height re-calibration + gradual TRIM + height debugger RESTORED — BUILT, then the
  re-tap RETIRED by session 22 after its live-fly (kept configurable)**
  (`plans/session21-restore-height-calib-and-trim.md`): goal-change re-tap (configurable `calib_cooldown_s` 60 s,
  `calib_goal_change_dist` 1.0); live refs ceiling_y/desired_y/delta re-measured each CALIB_VERIFY PASS; TRIM
  (pitch-aim + forward climb, ring-gated, goal-preserving) on `pos_y > ceiling + 1.2·delta` in SETTLE/ADVANCE;
  HEIGHT panel (pos_y/ceiling/desired/delta/trim-at/median/active). Session-20b integrations: recalib pulse is
  hop-outcome-only; TRIM clears the pending hop eval. Review hardening: never-calibrated allowed; None-guarded
  trigger; phase-relative WAIT gate; θ≈0 'c'-only resume. **NEXT = live-fly (with the 20b checklist).**
- **Session-20 REV — de-commit hops + persistent goals DB + corner safety — BUILT (`leg-hops-and-goal-commit-fix`),
  all 6 module self-tests green, UNCOMMITTED, LIVE-FLY PENDING** (`plans/session20-goal-db-loop-blacklist.md`):
  hop→REPLAN (re-pick every hop, adopt SLAM's new goal with parallax; `_settle_to="REPLAN"`); `frontier_planner`
  persistent `_goal_db` (goals-as-0.5u-discs, pick count + pick-time drone locs; ≥3 picks with any pair of
  drone-locs <1u → permanent loop-blacklist via the same store; NEVER reset mid-flight); `_register_bump`
  far-corner guard (`corner_no_blacklist_dist` 1.0); region-gated leg-stall tracker reset; kept the leg-stall
  guard + `forward_throttle 1.0`. New knobs under `autonomy.explore`. **NEXT = live-fly + watch LOOP-BLACKLIST +
  far-corner survival.** FOLLOW-UP: return-to-origin (orient-to-north + gentle stepped descent) still pre-existing.
- **Session-18 command smoothing + height-median — BUILT, self-tests green, LIVE-FLY PENDING**
  (`plans/session18-command-smoothing-and-height-median.md`): autopilot trigger/reverse/yaw/pitch are now RAMP
  TARGETS io_bridge's 60 Hz loop chases (manual constants) → smoothed flight + smooth release; `--log-commands`
  re-added (always-on via fly.py); height-median ingests one reading per FRESH SLAM frame after the first calib
  (frame dedup, `MAPPING_ALT_STATES` retired). **NEXT = live-fly + re-tune throttle knobs AND maneuver durations
  (smoothing attenuates short pulses).**
- **Session-17 triggerDown fix + height simplification — BUILT, all six module self-tests green, LIVE-FLY PENDING**
  (`plans/session17-triggerdown-and-height-simplification.md`): Unity gates thrust on the
  `triggerDown`/`reverseDown` BOOLEAN (autopilot never set it → the "crawl"). Added them to `AUTONOMY_FIELDS` +
  `_neutralize_autonomy` (io_bridge) and DERIVED centrally in `autopilot._full_vector` from the analog value;
  deleted the periodic re-calibration trigger + all of TRIM (state/trigger/exit/vars/config/self-test); kept the
  first calibration + flight-height median + calib recovery. **NEXT = live-fly + re-tune speed knobs.**
- **Wall-hit-triggered re-calibration — FUTURE (the next thing to build after session 17 flies).** The drone
  holds altitude on its own EXCEPT it climbs uncontrollably when flying forward/strafe INTO a wall (reverse
  doesn't). Session 17 kept the `CALIBRATING_HEIGHT` machinery + flight-height median (both unwired now)
  specifically so a wall-contact event can trigger a re-calibration judged against the median. To wire: on a
  forward/strafe wall-contact event, `self._recalibrating = True; self._enter("CALIBRATING_HEIGHT")`.
- **Session-16 settle-between-stages + return-to-origin — BUILT (committed 44b4fa6), LIVE-FLY PENDING**
  (`plans/session16-settle-between-stages-and-return-to-origin.md`): confirmed on the same flight as session 17.
- **Session-15 six fixes — BUILT, all module self-tests green, LIVE-FLY PENDING**
  (`plans/session15-trim-and-settle-fixes.md`): TRIM pitch sign (`trim_pitch_up=-1.0`); calibration
  escape/STUCK guard (`CALIB_ESCAPE` + `_calib_fail_escalate`, config `calib_escape_*`); SETTLE fresh-frame
  gate (`settle_fresh_frames`, `_SETTLE_EXEMPT_NXT`); CALIB_VERIFY no-fly-on-stale (`TIMEOUT_FAIL`→escalate);
  debugger live height numbers (`alt_*` + `_alt_median`); fly.py `NEW_CONSOLE`.
- **Settle between every recovery action + full return-to-origin ending — BUILT (session 16), LIVE-FLY PENDING**
  (`plans/session16-settle-between-stages-and-return-to-origin.md`): shared `_settle_begin`/`_settle_poll` gate;
  a settle between REWIND inverse maneuvers and between spin FALLBACK attempts (resolves the parked
  "reverse-without-settling"); `home_reach_dist` 0.5, `ORIENT_HOME`, `POSTLUDE_LOST_HOLD` (dock survives a SLAM
  loss), `_POSTLUDE_NOLOCK` (no floor re-inflation), homing `TURN→SETTLE→ADVANCE→SETTLE`. Knobs
  `recovery_settle_frames`/`recovery_settle_max_s`.
- **Gradual height TRIM — BUILT (session 14), pitch sign fixed session 15, LIVE-FLY PENDING**
  (`plans/gradual-height-trim.md`): PITCH-aim + forward climb between calibrations; 3-value capture
  (ceiling_y/desired_y/delta) at CALIB_VERIFY; `pos_y > ceiling_y + 1.2*delta` trigger in SETTLE/ADVANCE;
  ring-gated (reverse/strafe reposition, else abort); goal-preserving exit. Config knobs `trim_*`.
- **Return-to-origin + graceful dock — BUILT (session 16), LIVE-FLY PENDING**
  (`plans/return-to-origin-and-graceful-dock.md` = diagnosis of record; built per
  `plans/session16-settle-between-stages-and-return-to-origin.md`): home at altitude (`home_reach_dist` 0.5),
  `ORIENT_HOME` to the recorded take-off heading, `POSTLUDE_LOST_HOLD` (DOCK survives a SLAM loss),
  `_POSTLUDE_NOLOCK` kills the floor-level altitude-lock re-inflation.
- **Glass-wall bounce (blacklist region + counter) — DIAGNOSED (session 14), plan written, NOT BUILT**
  (`plans/blacklist-region-and-counter.md`): widen the blacklist region past the frontier spacing + per-region
  bump tallies (stop the `counter defeated` thrash).
- **Calibration survives a plan loss — BUILT (session 13), LIVE-PROVEN (session 14 flight, 3× recover)**
  (`plans/crystalline-swimming-floyd.md`): `CALIB_LOST_HOLD` + `_calib_interrupted`; redo on a 6-fast-frame +
  `status==OK` SLAM-pulse recovery, one DOWN bump if stuck, `status==OK`-gated exit (anti-flicker).
- **Height calibration — BUILT + FLEW (session 11), NOT confirmed good** (`plans/height-calib-state-gate-and-slam-debug.md`):
  state-gated `CALIB_VERIFY`/`ASCEND_ESCAPE`/`CALIB_TRANSLATE`. The operator is dissecting the `20260712`
  flight log; expect follow-up questions on whether the low-drone occupancy poisoning is actually solved.
- **Paired SLAM logging + timestamp fix — DONE (session 11)**; `fly.py` one-command launcher — DONE.
- **REPLAN dead-stall (item 2)** — no infinite idle when the planner returns no goal. Designed:
  `plans/replan-deadstall-sweep-and-slam-tracker.md` (bbox diagonal sweep + SLAM_TRACKER → replay HTML).
- **Per-goal height calibration (item 1)** — BUILT session 9, live-fly pending
  (`plans/glass-corner-blacklist-and-height-calib.md`).
- **Glass-corner blacklist escape (Bug A+B)** — built session 7, still needs a clean live confirm.
- **Phase-2b — dense low-altitude interior mapping, then detection.** Operator idea: map the inner
  room near ground level so the target can be found there later. Recommendation (see item-2 plan
  Part 3): a low-altitude interior traverse is worth it for denser geometry, but a *blind SLAM-off*
  flight drifts (no pose feedback). For detection, prefer **(a) offline cascade on the recorded
  map-mode footage** (reuses map-mode poses; no GPU contention — start here) or **(b) a temporally
  interleaved Scan mode** (SLAM navigate → pause → detect → resume), NOT a pure SLAM-off pass.
- Deferred: Scan mode (360° cascade with SLAM/GPU temporal separation); a glass-window altitude
  descend-probe; Phase-3 report polish + GUI.

---

## What this project is
Assessment task: from the black-box **XLAB** Unity sim's single monocular drone feed, autonomously
map the room and report the 3D location of a target object (+ uncertainty). Phases: 1 Human Recon →
2 Autonomous Survey → 3 Localize & Report → GUI. Grading = internal consistency (metric scale and
compute efficiency NOT graded). Local on an RTX 3080 Laptop (16 GB).

## Architecture (processes over a ZMQ bus)
- **P1 `io_bridge.py`** — NDI capture + 60 Hz TCP control to Unity + keyboard. Publishes 512×288
  transport frames (:5601) + hi-res 720p (:5605). Applies the autopilot's control ONLY while autonomy
  is ON (toggle `m`; any manual key aborts).
- **P2 `perception_worker.py`** — MASt3R-SLAM every frame → `MapStore` voxel map + `GroundGrid` 2D
  free/unknown/occupied. Publishes TOPIC_POSE/MAP/PLAN/TARGET (:5603); lifts detections into the map.
  (DA-V2 depth removed in session 5.)
- **P3 `visualizer.py`** — read-only dashboard (input | top-down map + path + frontiers/goal + target).
- **P4 `object_worker.py`** — 3-stage cascade detector; publishes TOPIC_DETECTION (:5604).
- **P5 `autopilot.py`** — CPU-only flight controller (optical-flow CEILING/WALL detector + playbook
  recipes). Modes: `--dry-run`, `--mission`, `--explore` (Map mode).
- **GPU note:** SLAM and the detection cascade **cannot share the GPU** (compute contention → SLAM
  RELOC spiral). Phase-2 separates them in time (Map mode = SLAM only; a future Scan mode pauses SLAM
  to run the cascade).

## What's built
**Phase 1 (done, hardware-verified):** io_bridge + bus + dashboard; SLAM + voxel map; **target
detector** = 3-stage cascade (GroundingDINO+OWLv2 propose → DINOv2 verify → SIFT/LightGlue geom gate)
— solved a small-object, mural-cluttered task that **every single-shot/VLM engine failed** (Qwen2.5-VL,
OWLv2, dense DINOv2/SIFT/LightGlue); **3D lift + consensus** (`target_estimator`) → confident
multi-target estimate. Offline E2E confirmed.

**Phase 2 — Map-mode explorer (`autopilot.py --explore`), flies live:**
- `ground_grid.py` — 2D grid + frontier extraction from SLAM points.
- `perception` publishes **TOPIC_PLAN** (pose/heading/goal/bearing/done + forward clearance + ring);
  goal = frontier planner pick; `plan_valid=false` when SLAM not TRACKING.
- `ExploreController`: **ARM → TAKEOFF → ASCEND (two-phase) → DESCEND → CALIB_VERIFY → BASELINE_NUDGE →**
  leg loop **REPLAN → ORIENT (open-loop ≤45° turn) → ADVANCE (forward until the clearance stand-off / flow
  WALL / self-calibrating ram guard) → SETTLE**; a per-goal **CALIBRATING_HEIGHT** re-tap routes ASCEND →
  DESCEND → **CALIB_VERIFY** (state-gated judge vs the frozen flying-height baseline; a sunk result →
  **ASCEND_ESCAPE → CALIB_TRANSLATE →** re-tap, session 11); a plan loss DURING any re-tap diverts to
  **CALIB_LOST_HOLD** (survive the loss → redo the calibration on a 6-fast-frame + `status==OK` SLAM-pulse
  recovery, one DOWN bump if stuck; session 13), and 3 consecutive failed calibrations divert to
  **CALIB_ESCAPE** (ring push + hold for SLAM → retry; 3 more → STUCK; session 15); a gradual-height **TRIM**
  (session 14) fires from SETTLE/ADVANCE when `pos_y` sinks past `ceiling_y + 1.2*delta` — ring-gated PITCH-aim
  + forward climb (`trim_pitch_up=-1.0`), goal-preserving; a leg **SETTLE waits for 6 fresh post-settle SLAM
  frames** before flying (session 15); on `done` the **floor-dock postlude
  RETURN_TO_ORIGIN → DOCK_FLOOR (two-phase pulsed descent) → LOW_STANDOFF → DONE** (session 10);
  control-space **recovery** on SLAM loss; **STUCK** hold; event-driven 2-bump blacklist for unreachable
  goals. **Ram guard is self-calibrating**; the clearance stand-off is the primary wall stop. (The ORIENT
  open-loop turn works — session-8 re-fly.)
- `flight_playbook.json` + `RecipePlayer` — control recipes as data (the tunable durations).
- `fly.py` — one-command stack launcher (perception + autopilot + visualizer + io_bridge + Xlab in separate
  windows), a graceful stop-file shutdown so the autopilot flushes its replay map backdrop, then auto-compiles
  + opens the flight report. The autopilot honours `--stop-file <path>` (polled `_FileStopEvent` → clean exit).

*(Sessions 8-13's build history — turn-logging fix, REPLAN dead-stall sweep, the all-corners tour +
floor-dock, height-calibration state-gating, `CALIB_LOST_HOLD` — is in the Session Log above; this
section is current-state only, not a second retelling.)*

---

## Reference — don't re-derive

### Drone control mechanic
Yaw is a **"fly toward your aim"** scheme: yaw moves an aim crosshair, forward thrust flies toward it;
a **SUSTAINED yaw hold then `c` (reset)** rotates the body (turn ANGLE = hold duration) — **confirmed
live in session 8** (the drone visibly turns). NB: the SLAM *heading* in the log lags the turn badly
(pose is ~2 Hz and monocular SLAM barely resolves pure rotation), so a real turn looks motionless in the
timeline until the drone translates — do NOT read that as "the drone didn't turn". io_bridge applies
autopilot values directly (no ramp); yaw latches until `c`. `joy_vertical` is a **DISCRETE −1/0/+1 axis**
(up/down = full thrust, can't be throttled); trigger & reverse ARE continuous 0–1. The only Unity
telemetry back is `time` — everything else is vision. Calibration: ~90° at yaw 1.0 for ~1.625 s.

### Environment & build
- Tree: `D:\EXTEND\C2_SIM\XLAB\` → `XLAB\` (read-only sim: Xlab.exe, Sample_Drone_Interface.py,
  OUTPUT\*.mp4) + `cartographer\` (our repo). One venv `cartographer\venv` (py 3.11.9,
  torch 2.5.1+cu121) — run everything from it.
- **lietorch is a PATCHED LOCAL build** (`third_party/lietorch`) — NEVER pip-install upstream.
- MASt3R-SLAM rebuild: `build_mast3r_slam.bat` then `build_mast3r_slam_step23.bat`.
- SLAM quirks (`slam_engine.py`): `os.chdir` into the SLAM repo before loading; recover the 4×4 pose
  via **Act3 on origin+unit axes, NOT `T_WC.matrix()`** (matrix() corrupts the pose under patched
  lietorch).

### Key technical facts
- **Sim protocol** (`Sample_Drone_Interface.py`): Python is the TCP **server** (127.0.0.1:65432);
  Unity connects in. 60 Hz `control_state` JSON (trigger/reverse, joy_horizontal strafe, joy_vertical
  altitude [−1 up/+1 down], yaw, pitch). Video = NDI 1280×720@30. Keys: 1=arm, w/s, a/d strafe,
  e/f up/down, arrows yaw/pitch, b=land, c=reset attitude, space=full-res capture, g=detect.
- **Resolution:** transport 512×288 (16:9, never squash); the cascade runs on the hi-res (:5605) stream.
- **Ray lift:** world ray = `pose[:3,:3] @ ray_cam`; center ray ≈ [0,0,1]; raycast skip 0.25 u.
- **World frame is +Y DOWN** (camera convention) — a sinking drone has an INCREASING `pos_y`.
- **Recording is ~58 fps, not 30** — durations must come from keystroke `mono_ts`, never frame counts.

### Run procedure
1. Designate target once: `venv\Scripts\python.exe make_target.py` → `target.yaml`.
2. **One command: `python fly.py`** — spawns perception `--no-display` + autopilot
   `--explore --log --stop-file` + visualizer + io_bridge (separate windows) + `Xlab.exe`; press `m` on
   io_bridge to hand over; press ENTER in the launcher to stop CLEANLY (drops the stop-file so the autopilot
   flushes its replay map backdrop) → auto-compiles + opens the report. Manual equivalent: `Xlab.exe` →
   `python io_bridge.py` → `python perception_worker.py --no-display` → `python visualizer.py` →
   `python autopilot.py --explore --log`; press `m` to hand over.
3. Offline self-tests: `autopilot.py --self-test`, `flow_contact_detector.py --self-test`,
   `frontier_planner.py --self-test`, `ground_grid.py --self-test`, `perception_worker.py --self-test`.
   Offline SLAM+map E2E: `perception_worker.py --video OUTPUT\flight_<ts>.mp4 --no-display`.
4. Diagnostics: `--log` → `OUTPUT/diag/<ts>_autopilot.{log,csv}` + `<ts>_timeline.{jsonl,html}`
   (open the HTML in a browser).

---

## Standing rules (every change)
- **NO SILENT FALLBACKS:** fail-fast OR set a visible/logged/HUD flag; any fallback approved first.
- **NO manual-flight data leakage:** every autonomous limit is a LIVE self-calibrating signal;
  platform/signal characteristics (flow signatures, control magnitudes, turn calibration, the ~1 s
  healthy-SLAM compute time) are legitimate — this room's geometry is not. **One documented,
  operator-approved exception:** branch `all-bets-are-off`, session 44 — TRIM's trigger is two
  hardcoded absolute pos_y numbers, flagged and knowingly overridden by the operator. Scoped to
  that branch; re-decide before merging to `main`.
- Image integrity (no undisclosed downscaling); start multi-step work with a TaskCreate list;
  self-test offline before live.
- **Before a commit** (whenever the user asks to commit): update `PROGRESS.md` and `STATE.md` to
  reflect current state, THEN commit, THEN push. See CLAUDE.md for the full rule.
