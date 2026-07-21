# Session 36 — image-based visual recovery on PLAN-STALE: loss-instant check + 15° rotational probe

> Filed as **session 36** in PROGRESS.md's sequence (a prior, unrelated "session 35" — the SLAM-slow
> strategy switch + `_recovering` fix — is already committed). The design below is internally labeled
> **"session 35 ALT"** throughout the code (comments, config keys, self-test names) because it was drafted
> as the alternative to a parallel Sonnet-authored plan before either landed; that label is kept verbatim
> in-code for traceability back to the design discussion, not because it is actually the 35th session.

> **This is the ALTERNATIVE to `mighty-churning-hamming.md` (the Sonnet plan).** Same operator vision —
> *the live NDI image tells us why tracking dropped and what to do about it* — and it now keeps the full
> decision tree including the **15° rotational turn-probe (Step 2c)** as a first-class mechanism that runs
> BEFORE any blind FALLBACK. Where this still differs from the Sonnet plan is flagged **[DIVERGES]**.
> Everything else is deliberately shared (config-gating, a new CPU-only module, LOUD give-up logging, a
> replay panel, the self-test culture).

## Context

FALLBACK (the blind wait→turn→push sweep, session 31) recovers from a PLAN-STALE unreliably and is
SLAM-hostile. A stale plan almost always means we got too close to something, or turned to a bad
angle/perspective — and the answer is in the live image we've never used for recovery.

**Framing this plan is built on [DIVERGES]:** session 34 already added the "too close" idea in *geometric*
form — `_maybe_loss_snapshot_backoff` (`autopilot.py:2110`, "Idea B") runs a ONE-SHOT check at the first
stale tick and backs off if the cached forward clearance reads too close. But it keys entirely on
`forward_clearance_dist` from the frozen map (`autopilot.py:2308`): **if SLAM never integrated the wall we
flew into, the cached clearance reads CLEAR, Idea B returns `None`, and today we fall to a blind FALLBACK.**
The image is the only signal that can catch that. So the visual loss-instant check is wired **as a second
signal at the exact decision point Idea B already owns**, covering the case geometry can't — and only when
*it* is inconclusive do we enter the 15° visual probe, and only when the probe is exhausted do we fall to
FALLBACK.

## The decision tree (operator's vision, fully preserved)

```
PLAN-STALE (first tick of the loss episode)
│
├─ Step 1  cached-clearance check (session-34 Idea B, UNCHANGED) — too close by geometry → BACKOFF
│
├─ Step 2  loss-instant visual check vs F_LKG (the frame cached the instant before tracking dropped)
│   ├─ 2a  matched & CONTAINED (F_live is a zoomed-in crop of F_LKG)      → BACKOFF
│   ├─ 2b  matched & PLANAR-LIKE (high homography inlier ratio, nose-to-a-flat-surface) → BACKOFF
│   └─ else (understandable 3D scene, or no match) → enter Step 2c
│
└─ Step 2c  15° ROTATIONAL VISUAL PROBE  (VISUAL_RECOVERY state)   [first-class, runs BEFORE FALLBACK]
     repeat:
       • open-loop TURN pulse of visrec_turn_step_deg (15°), then a brief settle
       • grab F_live, SIFT+RANSAC-homography match vs F_LKG
       • MATCHED (inliers ≥ visrec_min_inliers):
            s = sqrt(|det(H[:2,:2])|)
            s ≥ visrec_close_scale (1.15)  → closer/zoomed-in → BACKOFF (reuse _register_bump + BACKOFF)
            s <  visrec_close_scale         → farther/same → WAIT for SLAM to re-anchor (bounded)
       • NO MATCH → continue to the next 15° step
     EXHAUSTED (cum rotation ≥ visrec_max_rotation_deg, never re-acquired F_LKG):
       → LOUD diag_event "visual turn search exhausted" → fall through to FALLBACK
```

`visrec_close_scale` (1.15) and every other threshold is a general ratio/count, never a room answer
(CLAUDE.md autonomy standard); the scale test is self-relative (F_live vs F_LKG), per the best-practice
rider.

## Shared with the Sonnet plan (unchanged, it got these right)

- **Reuse the validated classical-CV primitive** `benchmark_detectors.SiftDetector` (`benchmark_detectors.py:161`
  — cv2 SIFT + ratio-test + RANSAC homography, `uses_gpu=False`, `SIFT_MIN_INLIERS`). **Copy** the ~25
  CPU-only lines into the new module; do NOT `import benchmark_detectors` (its top level pulls torch /
  LightGlue for the GPU engines). Keep `visual_recovery.py` CPU-only.
- **CPU-only on purpose**: SLAM keeps trying to relocalize every stale frame (not idle), so a GPU-heavy
  matcher would contend with the very relocalization we're waiting on. SIFT on the 512×288 transport frame is
  tens of ms; the drone is hovering/turning slowly during recovery, so a slower tick is fine. (LightGlue
  escalation stays a noted future option, not built.)
- **New module owns all image handling**; `ExploreController` stays a pure state machine fed small verdict
  values, exactly like it already consumes `wall_contact`/`backwall_contact` from `flow_contact_detector.py`.
  Keeps CV out of the FSM's fast synthetic self-tests.
- **Config-gated** via `use_visual_recovery_on_stale`, mirroring `use_rewind_on_stale` (`autopilot.py:754`).
  **[DIVERGES]: default `false`** (Sonnet's block sets it `true`). It ships live-fly-untested; every prior
  PROGRESS.md session lands "BUILT, live-fly PENDING", and `use_rewind_on_stale` set the precedent that a new
  stale-recovery path defaults off.
- **No silent fallback / no room-answer constants**; every give-up (no LKG, exhausted probe, wait-timeout) is
  a **LOUD `diag_event`** so it lands in the replay timeline (session-25 clickable event log), not just the
  console.
- **Frame source**: the transport stream already flowing into `run_explore` every tick (`autopilot.py:4099`)
  — no new subscription, no hi-res plumbing. Do NOT replace F_LKG with a keyframe ring buffer — a single
  cached last-known-good frame, per the operator's structure.
- A small **`flight_replay.py` floating panel** (same pattern as the session-29 Clearance tab).

## Design

### New module: `visual_recovery.py` (mirrors `flow_contact_detector.py` conventions)

- CPU-only SIFT matcher copied from `SiftDetector` (drop the `uses_gpu`/torch branch).
- `class VisualRecoveryProbe`:
  - `update_reference(frame, tracked: bool)` — caches `frame.copy()` as F_LKG whenever `tracked` is True.
    Called every tick from `run_explore`, gated on the SAME `plan.get("plan_valid")` boundary Idea B's own
    cache uses (`autopilot.py:2299`). Cheap (a copy); SIFT runs only at match time. *(Honest caveat: the
    plan lags frames by up to ~1s, so "F_LKG" is "frame at the last valid plan" — a close-enough proxy, the
    same imprecision Idea B's cached pose already accepts.)*
  - `match(frame) -> VisualMatch` (dataclass) — SIFT+RANSAC of `frame` vs F_LKG. Fields: `has_lkg`,
    `matched` (inliers ≥ `visrec_min_inliers`, reuse the validated 12), `inliers`, `contained` (warp
    F_live's 4 corners through H into F_LKG coords; all inside its bounds + `visrec_contain_margin_frac` →
    2a), `planar_like` (inlier ratio ≥ `visrec_planar_inlier_ratio` → 2b), `scale`
    (`sqrt(|det(H[:2,:2])|)`, the 2c close signal; only meaningful when `matched`).
  - `--self-test`: deterministic cv2-drawn synthetic pairs — contained (zoom-in), planar (flat texture),
    genuine-parallax (matched, scale≈1), no-overlap (no match). Same self-test culture as
    `flow_contact_detector.py`.

### `autopilot.py` — `ExploreController`

- New config flag `use_visual_recovery_on_stale` (default `false`, `autonomy.explore`) + the knobs below.
- **Step 2 (loss-instant), in `_maybe_loss_snapshot_backoff` (`autopilot.py:2110`)**: gains a `visual_match`
  param, threaded from both call sites (`autopilot.py:1440`, `:2386`). After the existing clearance clause
  (Step 1) declines, a new clause: `matched and (contained or planar_like)` → the identical
  `_register_bump` + `BACKOFF` action already there (`autopilot.py:2124-2133`), tagged "visual too-close @
  loss". If the visual check is also inconclusive AND `use_visual_recovery_on_stale`, hand into the probe
  (below) instead of returning `None`; else return `None` (today's fall-through to FALLBACK).
- **Step 2c — new state `"VISUAL_RECOVERY"`**, added to `_RECOVERY_STATES` (`autopilot.py:3790`) so the
  existing generic top-of-`step()` OK-convergence (status OK → SETTLE → REPLAN) resumes it for free, exactly
  like HOLD_LOST/REWIND/FALLBACK — this is also how a SLAM re-anchor during WAIT breaks the probe out with no
  bespoke polling.
- **New handler `_step_visual_recovery(now, plan, visual_match)`**, phase-timer style like
  `_step_fallback_sweep` (`self._visrec_phase`, build-if-`None` player pattern so a HOLD_LOST flicker cleanly
  restarts just the current sub-step; `_visrec_cum_deg` persists across the flicker like `_fallback_cum_deg`):
  - **TURN** (entry + after each no-match): build a `visrec_turn_step_deg` (15°) turn player
    (`self._build_turn`), play it, then a brief lost-SLAM settle (`_settle_begin`/`_settle_poll`, bounded by
    `recovery_settle_max_s`) so SLAM gets a still window and the match frame is clean → **MATCH**. Add 15° to
    `_visrec_cum_deg`; if `≥ visrec_max_rotation_deg` → LOUD `diag_event` "visual turn search exhausted" →
    `_enter_fallback_sweep`.
  - **MATCH**: consume the freshest `visual_match`:
    - `not matched` → back to **TURN** (next 15° step).
    - `matched and scale ≥ visrec_close_scale` → `_register_bump` + `BACKOFF` ("visual probe closer @ +N°").
    - `matched and scale < visrec_close_scale` → **WAIT_RECOVER**.
  - **WAIT_RECOVER**: hover; the generic OK-convergence breaks out the instant SLAM re-anchors. Bounded by
    `visrec_wait_recover_s` (30.0) — on timeout with no recovery → LOUD `diag_event` → **STUCK**
    ("re-acquired F_LKG farther, waited Ns, no re-anchor → STUCK").

### `run_explore` (`autopilot.py:~4099-4133`)

Build a `VisualRecoveryProbe` when the flag is on. Every tick: `probe.update_reference(frame, plan_valid)`.
On a stale tick (or while in VISUAL_RECOVERY), `vm = probe.match(frame)` and thread it into `ctrl.step(...)`
as a new `visual_match=None` kwarg (exactly like `wall_contact`, `autopilot.py:4131`). Pop/print/diag-log
the LOUD notices (reuse `take_notice`/`diag_event` plumbing).

### `flight_replay.py`

New "Visual Recovery" floating panel (session-29 Clearance-tab pattern: button + draggable panel), populated
per-step from a `visual_recovery_detail` field on the step record (phase, last `VisualMatch` verdict
incl. scale, F_LKG cache age). Makes every branch — especially the LOUD exhausted/timeout events —
scrubbable/inspectable.

### Config (`config.yaml`, `autonomy.explore`)

```
use_visual_recovery_on_stale: false   # off by default; live-fly-untested, mirrors use_rewind_on_stale
visrec_min_inliers: 12                 # reuse the project's already-validated SIFT/RANSAC inlier threshold
visrec_planar_inlier_ratio: 0.85       # inlier fraction to call a match "planar-like" (Step 2b)
visrec_contain_margin_frac: 0.02       # slack (fraction of frame) for the corner-containment test (Step 2a)
visrec_close_scale: 1.15               # homography linear scale >= this => zoomed-in => closer (Step 2c BACKOFF)
visrec_turn_step_deg: 15.0             # the probe's discrete rotation step (operator's exact value)
visrec_max_rotation_deg: 720.0         # cumulative probe budget before "exhausted" -> FALLBACK
visrec_wait_recover_s: 30.0            # bounded wait for a SLAM re-anchor after a farther re-match
```
(A dedicated `visrec_turn_step_deg`/`visrec_max_rotation_deg` keeps the 15° structure independent of
FALLBACK's own 22.5° `recovery_turn_step_deg`, per "do not alter the 15° step turn structure".)

## Files touched

- **New**: `visual_recovery.py` (CPU-only SIFT matcher + `VisualRecoveryProbe` + `--self-test`).
- `autopilot.py`: `_maybe_loss_snapshot_backoff` (visual clause + `visual_match` param at both call sites),
  new `VISUAL_RECOVERY` state + `_step_visual_recovery` + `_RECOVERY_STATES` entry, `run_explore` (probe
  lifecycle + per-tick reference cache + match threading + LOUD notices), config read, self-test additions.
- `flight_replay.py`: new floating panel + per-step `visual_recovery_detail` field.
- `config.yaml`: the knobs above.
- `PROGRESS.md`: session-36 entry once built; `plans/session36-visual-recovery-15deg-probe.md` as the plan
  of record (this file, moved in).

## Verification

- `python visual_recovery.py --self-test` — contained/planar/parallax/no-match classification + scale sign
  on synthetic pairs.
- `python autopilot.py --self-test` — new VISUAL_RECOVERY FSM cases (flag on):
  - Step 2 loss-instant: cached clearance CLEAR + `contained` → BACKOFF (Idea-B gap covered); + `planar_like`
    → BACKOFF; clearance ALREADY close → the clearance clause still wins first (visual never consulted).
  - Step 2c probe: parallax (no match) → TURN → re-match `scale≥1.15` → BACKOFF; → re-match `scale<1.15` →
    WAIT_RECOVER → SLAM OK breaks out; → WAIT_RECOVER timeout → STUCK; turn-budget exhausted with no re-match
    → LOUD event → FALLBACK hand-off.
  - **Regression**: flag `false` reproduces today's `_maybe_loss_snapshot_backoff` + FALLBACK behavior
    byte-for-byte (visual code never runs).
- `python flight_replay.py` on a self-test-produced log — the panel renders; LOUD events are clickable.
- **LIVE-FLY PENDING** afterward (no hardware here — same caveat as every PROGRESS.md session). First live
  watch: does a nose-to-an-unmapped-wall loss BACK OFF at the loss instant; does the 15° probe re-acquire
  F_LKG and act on scale correctly; does exhaustion fall to FALLBACK cleanly; any BACKOFF off a bad match
  (the accepted Idea-B-class risk).

## What still differs from the Sonnet plan (so the choice stays explicit)

Both plans now build the same 15° turn-probe decision tree. This ALT differs only in:
1. **[DIVERGES] Wiring the loss-instant check into session-34's `_maybe_loss_snapshot_backoff`** (one shared
   decision point, clearance-first then visual) rather than a parallel `_enter_stale_recovery` dispatch —
   makes the visual check the direct complement that fills Idea B's documented unmapped-wall gap.
2. **[DIVERGES] Config defaults `false`** (Sonnet: `true`), consistent with `use_rewind_on_stale` and the
   live-fly-untested convention.
3. **[DIVERGES] Dedicated `visrec_turn_step_deg: 15.0` / `visrec_max_rotation_deg`** knobs (Sonnet reuses the
   22.5° `recovery_turn_step_deg` and 720° `fallback_max_rotation_deg`), honoring the exact 15° step as a
   first-class, independent structure.

Pick this ALT for the tighter integration with the already-tested loss-instant path, the off-by-default
safety, and the exact 15° structure; pick Sonnet's for reusing the existing turn/rotation knobs and shipping
the path on by default.

---

## BUILD RESULT (session 36, 2026-07-21)

Built as designed above, with two real bugs found + fixed during implementation (not anticipated by the
plan):

1. **The visual "too-close" clause (Step 2) was originally gated only by `visual_match is not None`, not by
   `use_visual_recovery_on_stale`.** In practice this was harmless (`run_explore` only ever builds the probe
   / computes a match when the flag is on), but it broke the stated regression invariant ("flag false
   reproduces today's behavior byte-for-byte") as an explicit property of `_maybe_loss_snapshot_backoff`
   itself rather than an implicit contract with its one caller. Fixed: the entire visual path (Step 2 AND
   2c) is now gated behind one `if not self.use_visual_recovery_on_stale: return None` at the top.
2. **A TRUE-fresh VISUAL_RECOVERY entry inherited a stale `self._player`** left over from whatever maneuver
   was mid-flight when the loss hit (e.g. an in-progress ORIENT turn) — the TURN phase's build-if-`None`
   check saw a non-`None` (but unrelated, possibly already-mid-step) player and reused it instead of building
   a fresh 15° turn. Fixed: `_enter_visual_recovery` clears `self._player = None` on a true-fresh episode
   (mirrors how the BACKOFF/SETTLE branches already do this before their own `_enter()` calls).

`python visual_recovery.py --self-test`, `python autopilot.py --self-test`, `python flight_replay.py
--self-test`: ALL PASS (plus `perception_worker.py`, `frontier_planner.py`, `map_store.py`, `io_bridge.py`,
`flow_contact_detector.py` — unaffected, confirmed regression-free). **NEXT = LIVE-FLY** (flag is off by
default — flip `use_visual_recovery_on_stale: true` in config.yaml for the first test flight).
