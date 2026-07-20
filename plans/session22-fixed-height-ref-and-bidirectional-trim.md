# Session 22 — Fixed height reference + bidirectional TRIM; retire the mid-flight ceiling re-tap

## Diagnosis (flight `20260717_004418`, ~00:53–00:56)

The operator hit a loop: height calibration → plan lost near its end → long hold → redo → … → CALIB_ESCAPE →
STUCK (~2¼ min of nothing). The log indicts the mid-flight re-tap design itself:

1. **The re-tap is a SLAM-killer run in SLAM-hostile places.** The goal-change trigger fired in a corner already
   dropping the plan in plain flight; the vertical ASCEND then lost the plan on EVERY attempt. Each failed tap
   also threw the height around (y bounced -1.98 → -2.31 → -1.74 → -2.30 → -1.95 → -2.33).
2. **An interrupted calibration leaves the drone AT THE CEILING** (ASCEND ran, DESCEND never did). From mid-
   flight on the drone sat at y≈-2.30 ≈ ceiling (-2.32) — glued HIGH — while `desired_y` was **-1.855**.
3. **Nothing could bring a too-high drone down** — the altitude lock injects UP only; TRIM climbed only.
4. **The rolling median follows the error** (drifted -1.80 → -2.26 ingesting the glued-high samples) — it cannot
   be the reference.
5. **SLAM height is STABLE within a flight** (operator's hypothesis, confirmed): pos_y readings stayed
   consistent across every loss/re-lock; the first calibration's `desired_y` stayed valid all flight.
6. **The redo gate was too weak**: "6 fresh frames <1000ms" passed on 616–797ms alive-but-MARGINAL frames, and
   the redo died seconds into every ASCEND.

## What was built (operator's design)

1. **Periodic re-tap RETIRED** (`calibrate_on_goal_change: false`; code kept). The FIRST-takeoff calibration —
   run when SLAM is healthiest — is THE height reference for the whole flight.
2. **Bidirectional TRIM** — the only in-flight height corrector:
   - too LOW (sagged): `pos_y > ceiling_y + trim_sag_ratio·delta` → TRIM UP (as before);
   - too HIGH (glued near the ceiling — THE observed failure): `pos_y < desired_y − trim_high_ratio·delta`
     (new knob, 0.2) → **TRIM DOWN**: same machine, pitch aim +1.0 (mirrored sign) + forward push = gradual
     descent. Ring gate / guards / WAIT gate / goal-preserving exit unchanged; events carry the direction.
   - `trim_aim_s` is now AUTOMATIC (0.5 s platform constant — io_bridge ramps the aim ±0.05/tick @60 Hz, ±1.0
     saturates in ~0.33 s; the aim is then HELD through the whole push). Config knob removed.
3. **SLAM-COMFORT gate** (operator idea A): rolling average of HEALTHY-frame latencies (`_slam_ms_win`, window
   `calib_slam_avg_window`=10, ingested in `_update_slam` per fresh frame). A calibration REDO
   (CALIB_LOST_HOLD), ESCAPE retry, or (re-enabled) periodic tap additionally requires avg < `calib_slam_avg_ms`
   (666) once the window is full (part-filled passes; the FIRST prelude calibration is ungated). Bounded: a redo
   gated past `calib_gate_max_s` (30 s) counts ONE failed attempt via `_calib_fail_escalate(allow_redo=False)` —
   never launching an ASCEND into uncomfortable SLAM, while the streak still reaches CALIB_ESCAPE (relocate) /
   STUCK. A deferred periodic tap is noted on the leg event (visible).
4. **Y-DRIFT AUDIT posture** (operator idea B): re-enable later with `calib_cooldown_s: 600` to turn rare taps
   into drift measurements — every non-first CALIB_VERIFY PASS logs
   `Y-DRIFT check: ceiling_y moved {±x.xxx}u since the first calibration` (`_first_ceiling_y` baseline).
5. **Unifications + backstop**: CALIB_VERIFY PASS latches `target_altitude_y = settled_y` (the ADVANCE altitude
   lock holds the same verified height TRIM defends); once per flight, the median wandering > delta from
   `desired_y` raises a LOUD `HEIGHT-REFERENCE DISAGREEMENT` notice (take_notice → run_explore print; display-
   only — if SLAM Y ever DOES drift, the operator sees it).
6. **Debugger**: HEIGHT panel shows the full band — `trim-at-high` (desired − 0.2·delta) and `trim-at-low`
   (ceiling + 1.2·delta); pos_y reddens outside the band on EITHER side.

## Self-tests (all 6 module suites green)

`SESSION-22`: TRIM DOWN fires below the high threshold with a POSITIVE pitch and re-aims the preserved goal;
in-band pos_y fires neither; comfort gate holds on a full 800ms window → times out into one counted fail (still
holding) → releases when 300ms frames pull the average down; the shipped default keeps a >1u goal change on the
normal ORIENT path; a PASS latches target_altitude_y + the drift baseline; a second PASS logs the Y-DRIFT line;
the median-disagreement notice pops exactly once. (Plus the existing HEIGHT-TRIM / PERIODIC-RECALIB suites.)

## Live-fly verification (pending)

The drone holds ~`desired_y` all flight. A glued-high episode (post-recovery / wall-climb) triggers
`TRIM enter (DOWN)` at the next settle and pos_y returns into the band. NO `CALIBRATING_HEIGHT` after the first
one. The HEIGHT panel keeps pos_y between `trim-at-high` and `trim-at-low`; no calibration loops anywhere.
