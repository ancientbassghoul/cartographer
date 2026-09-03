# Session 57 — PLAN-LOST recovery rewrite + direction-aware LKG matching

## Origin

Diagnosed off flight `OUTPUT/diag/20260903_083329_autopilot.log` (23 min, 86 loss episodes) —
session 56's own SLAM_HOLD fix had just been flown and worked (fast, efficient), but the operator
watched the LKG debug window show a reference frame plainly *closer* than the live frame while the
drone backed off anyway.

**Finding 1 — the camera is only consulted when the map already said we're close.** Of 86 loss
episodes, 30 ran LKG matching (435 `[VISREC]` lines, mean 14.5/episode) and **56 ran none at all** (1
line in total). The discriminator is not the plan status — it is whether the one-shot ticket
`_loss_snapshot_checked` was *deferred* or *spent*. In `run_explore` the match block runs *before*
`ctrl.step()`, so on the first tick of a loss `visual_match` is structurally `None`; the
`_would_react` predicate can therefore only be decided by the cached clearance. Clear ahead → the
ticket is spent on that same tick before any SIFT ever runs → the camera is never consulted again for
the rest of the episode. Close ahead → the 12s loss-recovery grace defers without spending → matching
runs every 0.5s for the whole window. **The one case where the picture is the only evidence available
is exactly the case that never looks.**

**Finding 2 — direction is never tested.** The visual back-off trigger fired on
`matched and (contained or planar_like)`. `planar_like` is a pure inlier-ratio test — it means "flat
surface", not "closer" — and is completely direction-blind. Real lines from the flight:

```
08:42:05.063 [HOLD_LOST] [VISREC] matched=True inliers=94 contained=False planar_like=True scale=0.66
08:51:44.313 [HOLD_LOST] [VISREC] matched=True inliers=45 contained=False planar_like=True scale=0.32
08:51:45.914 [HOLD_LOST] [VISREC] matched=True inliers=40 contained=False planar_like=True scale=0.35
```

`scale < 1` means the live view is a *shrunk* view of F_LKG — the drone is **farther** from that
surface than F_LKG was. The operator watched the LKG debug window show a reference frame plainly
closer than the live frame while the drone backed off anyway.

**Finding 3 — `scale` is too noisy to be the direction estimator.** Same flight, within one second on
~30 inliers, `scale` read `1.44 → 1.31 → 0.53 → 0.59 → 0.67 → 1.33 → 1.85`. It is
`sqrt(|det(H[:2,:2])|)` of a projective homography, which conflates perspective foreshortening and
blows up on a weak fit. This session adds an inlier-**spread** ratio instead (RMS distance of the
RANSAC inlier keypoints from their centroid, live vs. F_LKG), and keeps `scale` logged beside it so
the next flight compares both estimators on real data.

**The new rule (operator-approved).** For `PLAN-LOST`/`NO-PLAN`: always wait 12s unconditionally;
past 12s always run LKG matching whatever the map remembers; the remembered clearance decides only
whether a back-off is *pending*, never whether we look; a three-way size verdict adjudicates the
pending back-off; and firing a back-off restarts the 12s wait, which is what stops it repeating.

## Design

### 1. Direction-aware `closer` verdict (`visual_recovery.py` — C1-C4)

`VisualMatch` gains `spread_lkg`/`spread_live`/`size_ratio`/`closer` (after `scale`, `debug_image`
still last). `VisualRecoveryProbe.__init__` gains `size_ratio_hi=1.25`, `size_ratio_lo=0.80`,
`size_min_inliers=20`. A new static `_inlier_spread(points)` returns the RMS distance of RANSAC-inlier
points from their centroid (`None` for <2 points or a non-finite result) — a LINEAR measure of screen
area spanned, far less sensitive to one stray inlier than the homography determinant. Once `matched`
and both spreads are usable (`spread_lkg > 1e-6`), `size_ratio = spread_live / spread_lkg` and:

| Condition | `closer` |
|---|---|
| `inliers < size_min_inliers` | `"UNKNOWN"` |
| `size_ratio > size_ratio_hi` | `"LIVE"` (live frame is magnified → we moved closer) |
| `size_ratio < size_ratio_lo` | `"LKG"` (F_LKG is the closer one → we are already farther) |
| otherwise | `"EQUAL"` |

Every earlier return path (no F_LKG, no descriptors, <4 good matches, no homography, below
`min_inliers`) leaves `closer == "UNKNOWN"`. The debug banner now prints `size=<ratio> closer=<verdict>`
beside the existing `scale=` field so both estimators are visible on the same real flight.

### 2. `_step_lost_recovery` replaces the one-shot ticket for PLAN-LOST/NO-PLAN (`autopilot.py` — C5-C8)

New method `_step_lost_recovery(plan, now, visual_match, status)` owns the whole PLAN-LOST/NO-PLAN
loss-instant decision:

1. No episode stamp (`_loss_episode_t0 is None`) → nothing to time, fall through.
2. `waited < loss_backoff_grace_s` → emit `LOSS_GRACE` once, hold. The wait is now **unconditional** —
   the notice text no longer claims "too-close evidence", since the camera hasn't been consulted yet.
3. The cached clearance (`_last_good_clearance <= stop_clearance_dist`) decides only whether a
   back-off is **pending**. Not pending → hold (matching still runs every tick past the grace, per
   `wants_visual_match`'s new clause — this is what fixes Finding 1).
4. `verdict = visual_match.closer` (or `"UNKNOWN"` if no match yet).
5. `verdict == "LKG"` → emit `LOST_VISUAL_HOLD` once, hold **indefinitely** (exit only via SLAM
   returning `OK`, or a fresh plan routing to `PLAN-STALE`'s own probe path). No back-off — this is
   the exact 08:51 symptom, fixed directly.
6. `"LIVE"`/`"EQUAL"`/`"UNKNOWN"` → the back-off proceeds (weak CV must never veto a physical
   reaction), **and restamps `_loss_episode_t0 = now`**, clearing both one-shot notice flags. This
   restamp is what stops a back-off repeating — after the push the drone is farther from the surface,
   so the next matured window reads `closer="LKG"` and step 5 holds.

`_maybe_loss_snapshot_backoff` is now reached **only** for `status == "PLAN-STALE"` — its Step 1/2/2c
tree is otherwise unchanged and keeps serving the visual-probe hand-off.

`wants_visual_match(now=None, status=None)` (both default `None`, so every existing caller is
unaffected) gains one more `True` condition: `status in ("PLAN-LOST", "NO-PLAN")` and a matured
episode (`now - _loss_episode_t0 >= loss_backoff_grace_s`). `_visrec_should_match` threads `now=`/
`status=` through to it; GATE B is otherwise untouched.

### 3. Direction gate on the PLAN-STALE visual trigger (Finding 2, the remaining path)

The PLAN-STALE visual back-off trigger (`matched and (contained or planar_like)`) gained the same
`closer in ("LIVE", "EQUAL", "UNKNOWN")` conjunct `_step_lost_recovery` uses, applied at both the
decision site and its `_would_react` predicate twin, so the two stay in lockstep (a stale drift here
would silently arm a reaction the action site then declines to take).

### 4. Blue corner-tour goals (`visualizer.py`)

`perception_worker.py` already publishes `goal_is_corner = bool(planner.sweeping)` (session 20).
`overlay_plan` now draws the goal star pure BLUE `(255, 0, 0)` when `plan.get("goal_is_corner")` is
true, else the existing YELLOW `(0, 255, 255)` — a missing key defaults to frontier/yellow. Gave
`visualizer.py` its first `--self-test` entry point (synthetic `overlay_plan` colour checks, no
window/socket/hardware), which also joins it to the runner's `DEFAULT_SUITES` gate.

### 5. Binary PLY + marker clusters (`map_store.py` — C9)

`MapStore.save_ply` gains `markers=None, binary=False`. `markers` is a list of
`((x, y, z), (r, g, b))` pairs, each written as a 7-point cluster (centre + ±`voxel_size` along each
axis) so it's visible/selectable in Blender; written LAST, after occupancy/trajectory/targets.
`binary=True` writes `format binary_little_endian 1.0` with the identical float-xyz + uchar-rgb vertex
layout, ~3x smaller / ~10x faster. A call with `markers=None, binary=False` is byte-identical to the
pre-session-57 ASCII output (self-test `ascii_unchanged`).

### 6. Per-frame PLY sequence (`perception_worker.py` — C10-C11)

Module-level `PLY_MARKER_COLORS` (5 fixed colours). New `Pipeline` attributes `ply_markers` (append-
only, capped list of dicts), `ply_seq_failures` (count), `ply_seq_degraded` (sticky flag). New
`_record_ply_marker(goal_xz, pos_y)` freezes the first N distinct *committed* goals (dedupe on
change, not on every tick a goal is held) as alignment anchors: `{goal_index, goal_xz, xyz=[goal_x,
pos_y, goal_z], rgb, first_frame=None}`. New module function `_write_frame_ply(pipe, seq_dir,
frame_idx, min_count=2)` writes one binary `.ply` per fused SLAM frame (`frame_{idx:05d}.ply`),
including every marker recorded so far, stamping `first_frame` on any marker still `None`; raises
`OSError` on failure — the caller (`run_live`) owns the loud-and-counted handling, incrementing
`ply_seq_failures` and latching `ply_seq_degraded` (NO SILENT FALLBACKS: a write failure is visible,
never swallowed). `markers.json` (`{"markers": pipe.ply_markers}`) is rewritten whenever a new marker
is appended and once more at shutdown. Config keys `diag.ply_sequence` (off by default, ~1.2GB/flight),
`diag.ply_sequence_markers` (5), `diag.ply_sequence_max` (2000, mirrors `visrec_save_max`'s bounded-
disk rule).

## Traps caught

- **The one-shot ticket was implicitly acting as the 15° probe's grace gate.** `_step_stale`'s
  `if not self._loss_snapshot_checked:` guard used to be spent almost immediately on a PLAN-LOST-
  opened episode, which meant `_step_stale` skipped the *ungated* Step-2c hand-off to
  `_enter_visual_recovery(...)` and the grace-checking late entry (`_maybe_enter_visual_probe`)
  handled the probe instead. Removing the ticket from the PLAN-LOST path (so `_step_lost_recovery` now
  owns that path entirely) exposed that ungated hand-off: a probe that TURNS could now fire at
  t=0.1s of a loss, before the drone has any business moving. Caught by
  `SESSION-52 chunk 4 visual-probe reachability` failing on `probe waits out the grace before
  turning`; fixed by requiring `_maybe_enter_visual_probe`'s existing grace condition
  (`_loss_episode_t0 is not None and now - _loss_episode_t0 < loss_backoff_grace_s`, reused verbatim,
  not re-derived) explicitly at the `_maybe_loss_snapshot_backoff` hand-off itself, before it enters
  `VISUAL_RECOVERY`. Two `SESSION-47 post-backoff re-solve gate` sub-assertions also failed as a direct
  consequence — they exercised `_backoff_resolve_since`, which session 57 replaced on the PLAN-LOST
  path with the `_loss_episode_t0` restamp; re-scoped those four sub-assertions to drive a PLAN-STALE
  loss instead (where `_backoff_resolve_since` is untouched), added one new sub-assertion for
  PLAN-LOST's replacement protection, and did not touch the five sub-assertions that already passed.
  **Lesson worth the words: a one-shot latch that gates several consumers is load-bearing for all of
  them, and removing it from one path silently un-gates the others** — grep for every reader of a
  one-shot flag before retiring it from any one of its callers.

Two ideas were considered and dropped during design, recorded here so they are not re-proposed:

- **A general CV veto over clearance decisions.** Unsound: LKG matching is not continuously active
  (it only runs on demand, gated by GATE A/B and now the loss-episode grace), so it structurally
  cannot continuously veto anything. A veto needs continuous evidence to be trustworthy as a veto.
- **Any raycast change.** Investigated and cleared as a candidate cause of the SLAM choke.
  `MapStore.clearance` is a flat HORIZONTAL fan with the Y component hardcoded to zero, 10 rays over
  ±8°, clipped at `clearance_max_range` 10.0 (ring 1.5), marched at `voxel_size × 0.5` = 2.5cm. It is
  not a cone or a frustum and cannot see floor or ceiling. It also cannot be the SLAM bottleneck:
  `slam_ms` times only `slam.process(rgb)` (`perception_worker.py:218-220`), and the clearance fan runs
  afterwards in `_plan_payload`, outside that window.

## Files touched

- `visual_recovery.py` — `VisualMatch` new fields, `VisualRecoveryProbe.__init__` new params,
  `_inlier_spread` static method, `match()`'s spread/ratio/verdict computation, debug banner `size=`/
  `closer=` text, self-test cases `(57-1)`–`(57-6)`.
- `autopilot.py` — `_lost_hold_noticed` field, `visrec_size_ratio_hi`/`_lo`/`_min_inliers` config
  reads, `_step_lost_recovery`, `_maybe_loss_snapshot_backoff` rescoped to `PLAN-STALE`-only + its
  Finding-2 direction conjunct, `wants_visual_match`'s new PLAN-LOST/NO-PLAN clause,
  `_visrec_should_match`'s `now=`/`status=` threading, the probe-grace fix at the Step-2c hand-off, and
  self-test blocks `SESSION-57 CONFIG WIRING`, `SESSION-57 PLAN-LOST RECOVERY`,
  `SESSION-57 PLAN-LOST ALWAYS LOOKS`, `SESSION-57 STALE DIRECTION GATE`,
  `SESSION-57 chunk 5 probe_inside_grace_holds`, `SESSION-57 chunk 5 no_second_grace_variant`.
- `visualizer.py` — `overlay_plan`'s blue-corner-goal branch, `run_self_test` + `--self-test` flag
  (new entry point, joins the runner's `DEFAULT_SUITES`).
- `map_store.py` — `save_ply`'s `markers=`/`binary=` params, marker-cluster + binary-header writers,
  self-test cases for ASCII-unchanged, binary header/payload size, marker points/colour, `markers=None`
  no-op.
- `perception_worker.py` — `PLY_MARKER_COLORS`, `Pipeline.ply_markers`/`ply_seq_failures`/
  `ply_seq_degraded`, `_record_ply_marker`, `_write_frame_ply`, the `markers.json` sidecar writer, the
  `run_live` call site + loud/counted failure handling, config reads for the three new `diag.ply_*`
  keys, `SESSION-57 PLY SEQUENCE` self-test block.
- `config.yaml` — `autonomy.explore.visrec_size_ratio_hi`/`_lo`/`_min_inliers`,
  `diag.ply_sequence`/`_markers`/`_max`.

## Verification

`python autopilot.py --self-test`, `python frontier_planner.py --self-test`,
`python visual_recovery.py --self-test`, `python flight_replay.py --self-test`,
`python ground_grid.py --self-test`, `python map_store.py --self-test`,
`python salvage_flight.py --self-test`, `venv\Scripts\python perception_worker.py --self-test`,
`venv\Scripts\python visualizer.py --self-test` — **all 9 suites, ALL PASS, 0 failures**, run clean
(no reverts in place) as the final check before this documentation chunk.

**Live-fly is the next step.** This session was built and gated entirely offline, per the global
spec's rule 9 (do not commit/stage/mutate git state) and the runner's own discipline of never flying
between chunks. The `_step_lost_recovery` rewrite, the `closer` verdict, and the blue corner goals
have never been exercised against a real loss episode — see `STATE.md`'s watch list for exactly what
to check on the first flight.
