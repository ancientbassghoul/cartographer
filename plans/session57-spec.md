# SESSION 57 — Sonnet-Ready Implementation Specification
# PLAN-LOST recovery rewrite · direction-aware LKG matching · blue corner goals · per-frame PLY sequence

Run with: `python sonnet_runner.py --plan plans/session57-spec.md`

---

## EXECUTION GUIDELINES (read before every chunk)

1. **Implement ONLY the current chunk.** Do not start, preview, refactor or "improve" any other
   chunk. Earlier chunks are already applied on disk — do not re-verify or re-implement them.
2. **Signatures are contracts.** Use the exact names, parameter names, parameter order, defaults,
   types and return shapes given in `SHARED CONTRACTS`. No renames, no extra parameters, no changed
   return shapes. If a contract looks wrong, implement it as written and say so in your report.
3. **No architectural changes.** Do not restructure the FSM, do not move functions between modules,
   do not introduce new classes, threads, processes or dependencies beyond what a chunk names.
4. **Anchors are source strings, not line numbers.** `autopilot.py` is ~8.8k lines and a bare `Read`
   truncates it — use `Grep` to locate each anchor string, then `Edit`.
5. **NO SILENT FALLBACKS** (`CLAUDE.md`). Never swallow an error into a default. A degraded path must
   set an explicit visible state flag, emit a CRITICAL log line, and be counted. Prefer raising over
   absorbing.
6. **IMAGE INTEGRITY** (`CLAUDE.md`). Never resize, crop, downscale or re-encode a frame. The
   zero-pad-never-scale behaviour in `visual_recovery.py` is load-bearing.
7. **NO MANUAL-FLIGHT DATA LEAKAGE** (`CLAUDE.md`). Every new constant here is a general ratio,
   duration or floor. Do not introduce any value derived from a specific flight or room.
8. **Comment in the surrounding style.** This codebase carries dense "why", not "what", comments,
   each tagged with its session number. Tag new blocks `Session 57:` and say *why*, citing the
   evidence in `MISSION CONTEXT`. Match the local density; no banner comments.
9. **Do not commit, stage, stash, or otherwise mutate git state.**
10. **The gate runs eight suites under the project venv after every chunk** — `autopilot.py`,
    `frontier_planner.py`, `visual_recovery.py`, `flight_replay.py`, `ground_grid.py`,
    `map_store.py`, `salvage_flight.py`, `perception_worker.py`. All eight are green at the start of
    this session. Breaking any one of them halts the run, not just the module you edited.
11. **Finish by running the self-test commands the chunk names**, and report the full PASS/FAIL list
    verbatim. A chunk that leaves any suite failing is not complete.
12. **Every chunk must change at least one file.** An empty diff is treated as a failed chunk.

---

## MISSION CONTEXT (why this work exists)

Diagnosed off flight `OUTPUT/diag/20260903_083329_autopilot.log` (23 min, 86 loss episodes).

**Finding 1 — the camera is only consulted when the map already said we're close.**
Of 86 loss episodes, 30 ran LKG matching (435 `[VISREC]` lines, mean 14.5/episode) and **56 ran none
at all** (1 line in total). The discriminator is not the plan status — it is whether the one-shot
ticket `_loss_snapshot_checked` was *deferred* or *spent*. In `run_explore` the match block runs
*before* `ctrl.step()`, so on the first tick of a loss `visual_match` is structurally `None`; the
`_would_react` predicate can therefore only be decided by the cached clearance. Clear ahead → the
ticket is spent on that same tick before any SIFT ever runs → the camera is never consulted again for
the rest of the episode. Close ahead → the 12 s loss-recovery grace defers without spending →
matching runs every 0.5 s for the whole window. **The one case where the picture is the only evidence
available is exactly the case that never looks.**

**Finding 2 — direction is never tested.** The visual back-off trigger fires on
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
blows up on a weak fit. This spec adds an inlier-**spread** ratio instead, and keeps `scale` logged
beside it so the next flight compares both estimators on real data.

**The new rule (operator-approved).** For `PLAN-LOST`/`NO-PLAN`: always wait 12 s unconditionally;
past 12 s always run LKG matching whatever the map remembers; the remembered clearance decides only
whether a back-off is *pending*, never whether we look; a three-way size verdict adjudicates the
pending back-off; and **firing a back-off restarts the 12 s wait**, which is what stops it repeating.

---

## SHARED CONTRACTS

Everything in this section is fixed. Do not deviate.

### C1 — `visual_recovery.VisualMatch` (extended dataclass)

Existing fields unchanged. Append exactly these four, in this order, after `scale`:

```python
@dataclass
class VisualMatch:
    has_lkg: bool
    matched: bool = False
    inliers: int = 0
    contained: bool = False
    planar_like: bool = False
    scale: float | None = None
    # --- session 57 ---
    spread_lkg: float | None = None    # RMS px distance of INLIER keypoints from their centroid, in F_LKG
    spread_live: float | None = None   # same, in the live frame
    size_ratio: float | None = None    # spread_live / spread_lkg; None unless both spreads are usable
    closer: str = "UNKNOWN"            # "LIVE" | "LKG" | "EQUAL" | "UNKNOWN"
    debug_image: "np.ndarray | None" = None
```

`debug_image` MUST remain the last field.

### C2 — `visual_recovery.VisualRecoveryProbe.__init__` (extended)

```python
def __init__(self, *, min_inliers=SIFT_MIN_INLIERS, planar_inlier_ratio=0.85,
             contain_margin_frac=0.02,
             size_ratio_hi: float = 1.25, size_ratio_lo: float = 0.80,
             size_min_inliers: int = 20):
```

Store as `self.size_ratio_hi = float(size_ratio_hi)`, `self.size_ratio_lo = float(size_ratio_lo)`,
`self.size_min_inliers = int(size_min_inliers)`. Existing params keep their names and defaults.

### C3 — `visual_recovery.VisualRecoveryProbe._inlier_spread` (new, static)

```python
@staticmethod
def _inlier_spread(points: "np.ndarray") -> float | None:
    """RMS distance of `points` (N,2 float) from their centroid, in pixels.

    Returns None when fewer than 2 points, or when the result is not finite.
    A LINEAR measure of how much screen area the matched features span -- directly comparable
    to the homography `scale`, but far less sensitive to one stray inlier than a hull area.
    """
```

### C4 — `closer` verdict rule (inside `VisualRecoveryProbe.match`)

Applied only once `out.matched` is True and both spreads are non-None and `spread_lkg > 1e-6`:

| Condition | `closer` |
|---|---|
| `inliers < self.size_min_inliers` | `"UNKNOWN"` |
| `size_ratio > self.size_ratio_hi` | `"LIVE"` |
| `size_ratio < self.size_ratio_lo` | `"LKG"` |
| otherwise | `"EQUAL"` |

Every earlier return path (no F_LKG, no descriptors, <4 good matches, no homography, below
`min_inliers`) leaves `closer == "UNKNOWN"`, `size_ratio is None`, both spreads `None`.

### C5 — `autopilot.ExploreController` new state and config

One new field, declared beside the existing `self._loss_grace_noticed`:

```python
self._lost_hold_noticed = False    # session 57: one-shot for the "LKG is closer -> holding" notice
```

New config values, read in the same block that reads `use_visual_recovery_on_stale`, via `e.get(...)`
on the `autonomy.explore` dict:

```python
self.visrec_size_ratio_hi = float(e.get("visrec_size_ratio_hi", 1.25))
self.visrec_size_ratio_lo = float(e.get("visrec_size_ratio_lo", 0.80))
self.visrec_size_min_inliers = int(e.get("visrec_size_min_inliers", 20))
```

### C6 — `autopilot.ExploreController._step_lost_recovery` (new method)

```python
def _step_lost_recovery(self, plan, now, visual_match, status):
    """Session 57: the WHOLE PLAN-LOST/NO-PLAN loss-instant decision, replacing the one-shot
    ticket path for these statuses. PLAN-STALE still uses `_maybe_loss_snapshot_backoff`.

    Returns the usual (fields: dict, state: str, event: str | None) triple to hand straight
    back to the caller, or None to fall through to the caller's plain hard hover-hold.
    """
```

Exact decision order — implement precisely this, no reordering:

1. `if self._loss_episode_t0 is None: return None` — no episode stamp, nothing to time.
2. `waited = now - self._loss_episode_t0`.
   `if waited < self.loss_backoff_grace_s:` — emit the grace notice once (guarded by
   `self._loss_grace_noticed`, set it True) via `self.note_timeout("LOSS_GRACE", <text>, now)`, then
   `return None`. The text MUST no longer claim there is too-close evidence — the wait is now
   unconditional. Use exactly:
   ```python
   f"LOSS-RECOVERY GRACE: holding still for {self.loss_backoff_grace_s:.0f}s before any reaction "
   f"(96.9% of held-still losses resolve inside that window). Looking at the camera after that."
   ```
3. `pending = (self._last_good_clearance is not None
              and self._last_good_clearance <= self.stop_clearance_dist)`.
   `if not pending: return None` — nothing to act on; the drone holds. Matching still runs (see C7).
4. `verdict = visual_match.closer if visual_match is not None else "UNKNOWN"`.
5. `if verdict == "LKG":` — the camera is confident the drone is already farther than F_LKG was.
   Emit the hold notice once (guarded by `self._lost_hold_noticed`, set it True) via
   `self.note_timeout("LOST_VISUAL_HOLD", <text>, now)`, naming `visual_match.size_ratio` and
   `visual_match.inliers`, then `return None`. **No back-off.** This hold is indefinite by design:
   the exit is SLAM returning OK, or perception publishing a `plan_valid=False` plan that routes to
   PLAN-STALE and its existing probe path.
6. Otherwise (`"LIVE"`, `"EQUAL"`, `"UNKNOWN"`) the pending back-off proceeds, mirroring the existing
   geometric branch in `_maybe_loss_snapshot_backoff`:
   ```python
   self._register_bump(dict(plan, pos=self._last_good_pos),
                       "clearance stand-off (stale pose @ loss)")
   # Session 57: taking a PHYSICAL action restarts the wait. This is what stops a back-off
   # repeating -- it replaces the one-shot ticket's re-fire protection. After the push the drone
   # is farther from the surface, so when the next window matures the match reads closer="LKG"
   # and step 5 holds. Restamping also re-arms both notices, since this is a NEW wait window.
   self._loss_episode_t0 = now
   self._loss_grace_noticed = False
   self._lost_hold_noticed = False
   ```
   then `if self.backoff_on_standoff:` → `return self._arm_loss_backoff(now, <why>)`;
   else → `self._enter("SETTLE", now)`; `return {}, "SETTLE", <event text>`.
   The `<why>` / `<event text>` MUST name the clearance value, the verdict, and `size_ratio`
   (or `n/a`), so the log line is self-explaining.

**Note the ordering trap:** the restamp must happen BEFORE `_arm_loss_backoff` returns, and must not
be undone anywhere. `_arm_loss_backoff` may escalate to the FALLBACK sweep instead of BACKOFF (its
`_blind_contact_reacts` wedge counter) — the restamp is correct in both cases, because both are
physical actions.

### C7 — `autopilot.ExploreController.wants_visual_match` (extended signature)

```python
def wants_visual_match(self, now=None, status=None):
```

Both parameters default to `None` so every existing caller and self-test keeps working unchanged.
Return True if ANY of:

- `not self._loss_snapshot_checked` (unchanged)
- `self._visrec_phase == "MATCH"` (unchanged)
- **session 57:** `status in ("PLAN-LOST", "NO-PLAN")` **and** `now is not None` **and**
  `self._loss_episode_t0 is not None` **and**
  `(now - self._loss_episode_t0) >= self.loss_backoff_grace_s`

### C8 — `autopilot._visrec_should_match` (extended signature)

```python
def _visrec_should_match(ctrl, *, needs_match, has_frame, loss_edge, moved_since_match,
                         memo, memo_age_s, now=None, status=None):
```

The only behavioural change: call `ctrl.wants_visual_match(now=now, status=status)` instead of
`ctrl.wants_visual_match()`. GATE B logic is otherwise untouched.

### C9 — `map_store.MapStore.save_ply` (extended signature)

```python
def save_ply(self, path, min_count: int = 1, trajectory=True, targets=None,
             markers=None, binary: bool = False):
    """...
    `markers` (session 57): optional list of ((x, y, z), (r, g, b)) pairs. Each is written as a
    7-point cluster -- the centre plus +/- self.voxel_size along each axis -- so it is visible and
    selectable in Blender. Written LAST, after occupancy/trajectory/targets.
    `binary` (session 57): write `format binary_little_endian 1.0` instead of ASCII. Identical
    vertex layout (float x,y,z + uchar red,green,blue), ~3x smaller and ~10x faster to write.
    """
```

Existing parameter names, order and defaults unchanged. ASCII output for a call with
`markers=None, binary=False` MUST be byte-identical to today's.

### C10 — `perception_worker` PLY-sequence contracts

Marker colour table (fixed, index-ordered), at module level:

```python
PLY_MARKER_COLORS = [(255, 0, 0), (0, 0, 255), (255, 255, 0), (0, 255, 255), (255, 128, 0)]
```

`Pipeline` new attributes, declared in `__init__`:

```python
self.ply_markers = []          # list[dict], record shape below; append-only, capped
self.ply_seq_failures = 0      # count of frame-PLY writes that raised
self.ply_seq_degraded = False  # sticky: True on the first failure, never cleared
```

Marker record shape (one dict per captured goal):

```python
{"goal_index": int,            # 0-based, == index into PLY_MARKER_COLORS
 "goal_xz": [float, float],    # the published goal, verbatim
 "xyz": [float, float, float], # world point: [goal_x, pos_y_at_capture, goal_z]
 "rgb": [int, int, int],
 "first_frame": int | None}    # frame index of the first PLY it was written into; None until written
```

New `Pipeline` method:

```python
def _record_ply_marker(self, goal_xz, pos_y) -> None:
    """Session 57: freeze the first N distinct committed goals as PLY alignment anchors.

    No-op when: the list is already at `self.ply_sequence_markers`, `goal_xz` is None, `pos_y` is
    None, or `goal_xz` equals the most recently recorded goal (dedupe on CHANGE, so a goal held
    across many plans is recorded once). Once appended a record is NEVER modified except for
    `first_frame`, which the writer stamps on the first PLY that carries it.
    """
```

New module-level function:

```python
def _write_frame_ply(pipe, seq_dir, frame_idx: int, min_count: int = 2):
    """Session 57: write ONE .ply for the current fused map state.

    Path: `seq_dir / f"frame_{frame_idx:05d}.ply"`. Binary. Includes trajectory and every marker
    recorded so far; stamps `first_frame` on any marker whose value is still None. Returns the
    written Path. Raises OSError on failure -- the CALLER owns the loud-and-counted handling,
    mirroring the periodic-checkpoint pattern in `run_live`.
    """
```

Sidecar, rewritten whenever a new marker is appended and once more at shutdown:
`seq_dir / "markers.json"` containing `{"markers": pipe.ply_markers}`, `json.dump(..., indent=2)`.

### C11 — new config keys

`config.yaml`, under `autonomy.explore`, immediately after the existing `visrec_close_scale` line so
the `visrec_*` block stays contiguous:

```yaml
    visrec_size_ratio_hi: 1.25    # session 57: inlier-SPREAD ratio (live/LKG) above this => the live frame is
                                  #   MAGNIFIED => we moved CLOSER => a pending back-off is justified.
    visrec_size_ratio_lo: 0.80    # session 57: below this (= 1/1.25) the LKG frame is the closer one => we are
                                  #   already farther than the evidence describes => HOLD, never back off.
    visrec_size_min_inliers: 20   # session 57: a direction verdict resting on fewer inliers than this is "not
                                  #   sure" -> UNKNOWN, which never suppresses a back-off (weak CV must not veto).
```

`config.yaml`, under the `diag:` block beside `livemap_checkpoint_period_s`:

```yaml
  ply_sequence: false         # session 57: write one .ply per fused SLAM frame, for a Blender build-up animation.
                              #   OFF by default -- roughly 1.2 GB per flight.
  ply_sequence_markers: 5     # session 57: freeze the first N committed goals into EVERY ply as alignment anchors.
  ply_sequence_max: 2000      # session 57: hard file cap per flight (mirrors visrec_save_max's bounded-disk rule).
```

### C12 — self-test conventions

This repo's suites are plain `check(name, bool)` accumulators printing `PASS`/`FAIL` per block, run
via `python <module>.py --self-test`. Follow the existing block style exactly: a leading comment
naming the session and the defect, one `check(...)` per assertion, and the block's aggregate folded
into the module's overall `ok`. Name new blocks `SESSION-57 <TOPIC>`.

---

## CHUNK 1 — inlier-spread measure and the `closer` verdict

**Module Objective.** Give `VisualMatch` a robust, direction-aware answer to "which frame do the
matched features occupy more screen space in?". Pure measurement — no caller changes.

**Required Context/Dependencies.** None (first chunk). Contracts C1, C2, C3, C4.

**Target Files.** `visual_recovery.py` only.

**Strict Interfaces.**

- Extend the `VisualMatch` dataclass exactly per **C1**.
- Extend `VisualRecoveryProbe.__init__` exactly per **C2**.
- Add `VisualRecoveryProbe._inlier_spread` exactly per **C3**.
- Inside `match()`, compute the spreads from the RANSAC inliers already in hand — no second detector
  pass, no re-running SIFT:
  ```python
  src_in = src.reshape(-1, 2)[mask]      # F_LKG inlier points
  dst_in = dst.reshape(-1, 2)[mask]      # live inlier points
  ```
  then `out.spread_lkg = self._inlier_spread(src_in)`, `out.spread_live = self._inlier_spread(dst_in)`,
  then `out.size_ratio` and `out.closer` per **C4**. Place this block AFTER the existing
  `out.planar_like` assignment and BEFORE the existing `out.scale` try/except, so no existing
  statement moves.
- Extend `_compose_debug`'s `line2` f-string to append `size={ratio_txt} closer={out.closer}`, where
  `ratio_txt` is `f"{out.size_ratio:.2f}"` or `"n/a"`. Do NOT remove `scale=` — both must show.
- Update the `VisualMatch` and `match()` docstrings to describe the new fields and say *why* spread
  was chosen over `scale` (Finding 3).

**Acceptance Tests.** New block `SESSION-57 SPREAD RATIO` in `run_self_test`, built on the existing
`_textured_image(w, h, seed, n_shapes)` helper. Assert:

1. `zoom_in` — reference `_textured_image(seed=7)`; live = the same content magnified into a
   same-size canvas via `cv2.warpAffine` with a known scale > 1 (do NOT resize the source asset).
   Expect `matched is True`, `size_ratio > 1.0`, `closer == "LIVE"`.
2. `zoom_out` — the inverse warp of the same pair. Expect `size_ratio < 1.0`, `closer == "LKG"`.
3. `identical` — reference matched against itself. Expect `size_ratio` within `[0.95, 1.05]` and
   `closer == "EQUAL"`.
4. `unmatched_is_unknown` — match against a frame with no correspondence. Expect `matched is False`,
   `size_ratio is None`, `closer == "UNKNOWN"`, both spreads `None`.
5. `low_inliers_is_unknown` — a probe built with `size_min_inliers=10_000`; a normally-matching pair
   must yield `closer == "UNKNOWN"` while `size_ratio` is still populated.
6. `spread_none_on_degenerate` — `_inlier_spread` returns `None` for a 1-point array and for an
   empty array.

**Verify.** `python visual_recovery.py --self-test` — 0 failures.

---

## CHUNK 2 — wire the thresholds from config, and log the verdict

**Module Objective.** Make the new measure configurable and visible. No decision logic yet.

**Required Context/Dependencies.** Chunk 1. Contracts C5 (the three `visrec_size_*` reads only),
C11 (the three `visrec_size_*` keys only).

**Target Files.** `config.yaml`, `autopilot.py`.

**Strict Interfaces.**

- `config.yaml`: add the three `visrec_size_*` keys exactly per **C11**.
- `autopilot.py`, `ExploreController.__init__`: read the three values per **C5**, immediately after
  the existing `self.visrec_contain_margin_frac` assignment.
- `autopilot.py`, the `VisualRecoveryProbe(...)` construction in `run_explore` — anchor on the source
  string `contain_margin_frac=ctrl.visrec_contain_margin_frac) if ctrl.use_visual_recovery_on_stale else None` —
  pass the three new kwargs through:
  `size_ratio_hi=ctrl.visrec_size_ratio_hi, size_ratio_lo=ctrl.visrec_size_ratio_lo,
   size_min_inliers=ctrl.visrec_size_min_inliers`.
- The `[VISREC]` log line — anchor on `f"planar_like={visual_match.planar_like} scale={scale_txt}"` —
  append `f" size={size_txt} closer={visual_match.closer}"`, where `size_txt` mirrors the existing
  `scale_txt` pattern (`f"{visual_match.size_ratio:.2f}"` or `"n/a"`).
- Extend the `vlabel` tuple (anchor: `vlabel = (visual_match.has_lkg, visual_match.matched,`) with
  `visual_match.closer`, so a verdict *change* defeats the 0.5 s log dedupe and is never swallowed.

**Acceptance Tests.** New block `SESSION-57 CONFIG WIRING` in `autopilot.py`:

1. `defaults_present` — a controller built from the repo's own `config.yaml` has
   `visrec_size_ratio_hi == 1.25`, `visrec_size_ratio_lo == 0.80`, `visrec_size_min_inliers == 20`.
2. `overrides_honoured` — a cfg dict with different values produces those values on the controller.
3. `lo_below_hi` — assert `visrec_size_ratio_lo < visrec_size_ratio_hi` as a sanity invariant.

**Verify.** `python autopilot.py --self-test`, `python visual_recovery.py --self-test`.

---

## CHUNK 3 — let a matured PLAN-LOST episode actually run the match

**Module Objective.** Open the gate so LKG matching runs on EVERY loss episode past the grace, not
only on episodes that happen to hold the one-shot ticket. Without this, chunk 4 is dead code.

**Required Context/Dependencies.** Chunks 1-2. Contracts C7, C8.

**Target Files.** `autopilot.py`.

**Strict Interfaces.**

- Extend `ExploreController.wants_visual_match` exactly per **C7**. Its docstring currently documents
  "exactly two consumers"; there are now three. The CALLER CONTRACT paragraph ("this is an
  AND-narrowing of run_explore's existing status gate, NEVER a replacement for it") must be preserved
  in meaning — the new clause is an additional narrow permission, not a replacement.
- Extend `_visrec_should_match` exactly per **C8**.
- Update the single call site in `run_explore` — anchor on
  `moved_since_match=visrec_moved_since_match, memo=visrec_memo,` — to also pass `now=now, status=status`.

**Acceptance Tests.** New block `SESSION-57 PLAN-LOST ALWAYS LOOKS` in `autopilot.py`:

1. `spent_ticket_clear_front_still_looks` — controller with `_loss_snapshot_checked = True`,
   `_loss_episode_t0 = t0`, called as `wants_visual_match(now=t0 + grace + 0.1, status="PLAN-LOST")`
   → **True**. (Today this returns False — the defect being fixed.)
2. `inside_grace_does_not_look` — same controller at `now = t0 + grace - 0.1` → **False**.
3. `no_args_is_unchanged` — `wants_visual_match()` with a spent ticket and no `_visrec_phase` →
   **False**, proving backward compatibility for every existing caller.
4. `plan_stale_unaffected` — `status="PLAN-STALE"` past the grace with a spent ticket → **False**.
5. `no_episode_stamp` — `_loss_episode_t0 = None` past any time → **False**.
6. `should_match_threads_args` — `_visrec_should_match(..., now=..., status="PLAN-LOST")` returns
   True for a matured episode with `memo=None`, and False for the same call at `status="OK"`.

**Verify.** `python autopilot.py --self-test`. Confirm no previously-passing block regressed —
`wants_visual_match` is consumed by several older blocks.

---

## CHUNK 4 — the PLAN-LOST recovery rule itself

**Module Objective.** Replace the one-shot-ticket decision path for `PLAN-LOST`/`NO-PLAN` with the
uniform wait → always look → three-way verdict rule, where firing a back-off restarts the wait.
This is the load-bearing chunk.

**Required Context/Dependencies.** Chunks 1-3 (`visual_match.closer` is populated, and a match is
actually computed for a matured PLAN-LOST episode). Contracts C5, C6.

**Target Files.** `autopilot.py`.

**Strict Interfaces.**

- `ExploreController.__init__`: add `self._lost_hold_noticed = False` per **C5**.
- `reset_leg` — anchor on
  `self._loss_episode_t0 = None         # session 48: and any in-flight loss-recovery grace window` —
  also clear `self._lost_hold_noticed = False`.
- The fresh-loss-edge block — anchor on
  `self._visrec_probe_armed = True       # session 52 (chunk 4): re-arm the probe's late entry` —
  also reset `self._lost_hold_noticed = False`, so each episode starts clean.
- Add `ExploreController._step_lost_recovery` exactly per **C6**. Place it immediately before
  `_maybe_loss_snapshot_backoff` so the two loss paths read together.
- Rewire the two PLAN-LOST call sites, both inside the `if status in ("PLAN-LOST", "NO-PLAN"):`
  branch of `step()`:
  - **Fresh-entry site** — anchor on the source string
    `snap = self._maybe_loss_snapshot_backoff(plan, now, visual_match, status=status)`. Replace the
    whole enclosing `if not self._loss_snapshot_checked:` guard and its body with an unconditional:
    ```python
    snap = self._step_lost_recovery(plan, now, visual_match, status)
    if snap is not None:
        return snap
    ```
  - **Per-tick site** — anchor on the source string
    `deferred = self._maybe_loss_snapshot_backoff(plan, now, visual_match, status=status)`. Same
    replacement, keeping the local name `deferred`.
  The ticket is no longer consulted or spent on this path. Do NOT delete `_loss_snapshot_checked` —
  `_step_stale` (PLAN-STALE) still owns it, unchanged.
- `_maybe_loss_snapshot_backoff` is otherwise **untouched**, but update its docstring to state that
  it is now reached only for `status == "PLAN-STALE"`, and why.

**Acceptance Tests.** New block `SESSION-57 PLAN-LOST RECOVERY` in `autopilot.py`. Drive
`ctrl.step(...)` with synthetic plans exactly as the existing session-52 probe tests do:

1. `inside_grace_holds` — clear-front PLAN-LOST inside the grace → `HOLD_LOST`, no BACKOFF, and the
   grace notice fired exactly once across three consecutive ticks.
2. `clear_front_past_grace_holds` — `_last_good_clearance = 3.0` past the grace → `HOLD_LOST`, no
   BACKOFF (nothing pending), and **no** hold-notice (that notice is only for the LKG verdict).
3. `verdict_live_backs_off` — `_last_good_clearance = 0.5`, match with `closer="LIVE"` → `BACKOFF`.
4. `verdict_equal_backs_off` — same but `closer="EQUAL"` → `BACKOFF`.
5. `verdict_unknown_backs_off` — same but `closer="UNKNOWN"` → `BACKOFF` (weak CV must never veto).
6. `verdict_lkg_holds` — same but `closer="LKG"` → `HOLD_LOST`, no BACKOFF, hold notice fired once.
7. `lkg_hold_is_indefinite` — the `closer="LKG"` controller still holds after 60 s of further ticks.
8. `backoff_restamps_the_wait` — after a `closer="LIVE"` back-off, `_loss_episode_t0` equals the
   back-off's `now`, and both `_loss_grace_noticed` and `_lost_hold_noticed` are back to False.
9. `restamp_blocks_immediate_refire` — continuing to tick that controller with the SAME evidence
   produces no second BACKOFF until a further `loss_backoff_grace_s` has elapsed.
10. `plan_stale_path_untouched` — a PLAN-STALE loss still routes through
    `_maybe_loss_snapshot_backoff` and still spends `_loss_snapshot_checked`.
11. `episode_edge_resets` — a genuine `OK` followed by a fresh loss clears `_lost_hold_noticed`.

**Verify.** `python autopilot.py --self-test`, `python visual_recovery.py --self-test`,
`python frontier_planner.py --self-test`, `python flight_replay.py --self-test`.

---

## CHUNK 5 — direction gate on the PLAN-STALE visual trigger

**Module Objective.** Fix Finding 2 on the remaining path. One conjunct, applied twice.

**Required Context/Dependencies.** Chunks 1-2.

**Target Files.** `autopilot.py`.

**Strict Interfaces.**

- Anchor on the source string
  `if visual_match is not None and visual_match.matched and (visual_match.contained or visual_match.planar_like):`
  and add `and visual_match.closer == "LIVE"` as a further conjunct.
- Anchor on the source string `or (self.use_visual_recovery_on_stale and visual_match is not None`
  (inside the `_would_react` predicate) and add the identical conjunct to its visual clause, so the
  grace/gate arming and the action agree. **These two MUST stay in lockstep** — if the predicate says
  a reaction is coming and the action then declines, the episode silently stalls.
- Add a `Session 57:` comment at both sites explaining that `planar_like` means "flat surface", not
  "closer", citing the `08:51:44 ... planar_like=True scale=0.32` evidence.

**Do NOT touch** `_step_visual_recovery`'s MATCH phase (anchor:
`if vm.scale is not None and vm.scale >= self.visrec_close_scale:`). That path has never executed on
a real flight; changing an unobserved mechanism in the same session would make the next flight
unreadable.

**Acceptance Tests.** New block `SESSION-57 STALE DIRECTION GATE` in `autopilot.py`:

1. `planar_far_no_backoff` — PLAN-STALE, `matched=True, planar_like=True, closer="LKG"`, clearance
   clear → no BACKOFF.
2. `planar_near_backs_off` — same but `closer="LIVE"` → BACKOFF.
3. `contained_far_no_backoff` — `contained=True, closer="LKG"` → no BACKOFF.
4. `would_react_agrees` — `_would_react`'s visual clause is False for `closer="LKG"` and True for
   `closer="LIVE"`, given otherwise-identical evidence.

**Verify.** `python autopilot.py --self-test`.

---

## CHUNK 6 — corner (bounding-box) goals drawn blue, and a visualizer self-test

**Module Objective.** Distinguish bbox corner-tour goals from frontier goals on the map panel, and
give `visualizer.py` the self-test entry point it currently lacks.

**Required Context/Dependencies.** None — `perception_worker.py` already publishes
`payload["goal_is_corner"] = bool(self.planner.sweeping)`. Nothing upstream changes.

**Target Files.** `visualizer.py`, `sonnet_runner.py`.

**Strict Interfaces.**

- In `overlay_plan`, anchor on the source string
  `cv2.drawMarker(img, (int(gu[0]), int(gv[0])), (0, 255, 255), cv2.MARKER_STAR, 18, 2)` and select
  the colour from the plan flag:
  ```python
  goal_bgr = (255, 0, 0) if plan.get("goal_is_corner") else (0, 255, 255)   # corner = BLUE, frontier = yellow
  ```
  Marker type, size and thickness unchanged. Do not change any other colour in the file.
- Update `overlay_plan`'s docstring, which currently says `the current goal (yellow star)`.
- **Add a `--self-test` entry point.** `visualizer.py` currently has an argparse that REJECTS
  `--self-test`. Follow `visual_recovery.py`'s pattern exactly: an `argparse` flag, a
  `run_self_test()` returning a bool, printing one `PASS`/`FAIL` line per check plus an
  `ALL PASS` / summary line, and `sys.exit(0 if ok else 1)`. It must not import or open any window.
- **Register the module with the gate.** In `sonnet_runner.py`, add `"visualizer.py"` to
  `DEFAULT_SUITES` and delete the sentence in the comment above it that explains why it was absent.
  This is the one chunk permitted to edit the runner, and it must do so — every later chunk depends
  on the gate covering this module.

**Acceptance Tests.** New block `SESSION-57 CORNER GOAL COLOUR` in `visualizer.py`. Call
`overlay_plan` on a small synthetic image with minimal `plan` / `m` dicts and inspect drawn pixels:

1. `corner_goal_is_blue` — `goal_is_corner=True` puts pure `(255, 0, 0)` pixels on the canvas and no
   `(0, 255, 255)` star pixels.
2. `frontier_goal_is_yellow` — `goal_is_corner=False` puts `(0, 255, 255)` pixels and no `(255, 0, 0)`.
3. `missing_flag_defaults_yellow` — a plan dict with no `goal_is_corner` key behaves as frontier.

**Verify.** `python visualizer.py --self-test` — 0 failures — and
`python sonnet_runner.py --plan plans/session57-spec.md --list` to prove the runner still parses.

---

## CHUNK 7 — binary PLY writer with marker clusters

**Module Objective.** Teach `MapStore.save_ply` to emit binary PLY and to embed marker clusters.
Pure writer work — no caller changes.

**Required Context/Dependencies.** None. Contract C9.

**Target Files.** `map_store.py`.

**Strict Interfaces.**

- Extend `save_ply` exactly per **C9**. Anchor on
  `def save_ply(self, path, min_count: int = 1, trajectory=True, targets=None):`.
- Marker expansion: for each `((x, y, z), (r, g, b))`, emit 7 points — the centre plus
  `±self.voxel_size` along each of X, Y and Z — all in that marker's colour. Append AFTER the
  existing targets block so existing point ordering is untouched.
- Binary output: header lines `ply`, `format binary_little_endian 1.0`, then the same
  `element vertex N` / `property float x|y|z` / `property uchar red|green|blue` / `end_header`
  sequence, written as ASCII bytes with `\n` line endings; then the packed records. Use a single
  `np.zeros(N, dtype=np.dtype([("x","<f4"),("y","<f4"),("z","<f4"),("red","u1"),("green","u1"),("blue","u1")]))`
  structured array and one `.tobytes()` write — do not loop per point.
- Open the file `"wb"` for binary; keep `"w", encoding="utf-8"` for ASCII. The ASCII branch's
  formatting (`f"{x:.4f} {y:.4f} {z:.4f} {int(r)} {int(g)} {int(b)}\n"`) must not change.

**Acceptance Tests.** New block `SESSION-57 BINARY PLY` in `map_store.py`'s self-test:

1. `ascii_unchanged` — for a fixed synthetic map, `save_ply(p, binary=False)` still produces the
   documented ASCII header and one `x y z r g b` line per vertex, with the vertex count matching
   `len(occupied) + len(trajectory)`. State plainly in your report how you established this baseline.
2. `binary_header_and_count` — the binary file starts with `b"ply\nformat binary_little_endian 1.0\n"`
   and its `element vertex N` matches the ASCII file's N for the same arguments.
3. `binary_payload_size` — file size == header length + `N * 15` bytes (3 float32 + 3 uint8).
4. `markers_add_seven_points_each` — `markers=[((1,2,3),(255,0,0)), ((4,5,6),(0,0,255))]` raises the
   vertex count by exactly 14 versus `markers=None`.
5. `marker_colour_present` — parsing the binary payload back, exactly 7 vertices carry `(255,0,0)`
   and 7 carry `(0,0,255)` beyond whatever the base map contained.
6. `markers_none_is_noop` — `markers=None` and `markers=[]` produce identical output.

**Verify.** `python map_store.py --self-test` — 0 failures — plus `python autopilot.py --self-test`
to prove nothing upstream regressed.

---

## CHUNK 8 — per-SLAM-frame PLY sequence in perception

**Module Objective.** Emit one PLY per fused SLAM frame with frozen goal anchors, plus the sidecar
manifest, behind an off-by-default config flag, with loud counted failure handling.

**Required Context/Dependencies.** Chunk 7 (`save_ply` accepts `markers=` and `binary=`).
Contracts C10, C11 (the `diag:` keys).

**Target Files.** `config.yaml`, `perception_worker.py`.

**Strict Interfaces.**

- `config.yaml`: add the three `diag:` keys per **C11**.
- Module level: add `PLY_MARKER_COLORS` per **C10**, beside the other module constants.
- `Pipeline.__init__`: add the three attributes per **C10**, plus
  `self.ply_sequence_markers = int((cfg.get("diag") or {}).get("ply_sequence_markers", 5))`,
  following the existing config-read style in that constructor.
- Add `Pipeline._record_ply_marker` per **C10**, and call it from `_plan_payload` immediately after
  the existing goal assignment — anchor on
  `payload["goal"] = [round(float(goal[0]), 4), round(float(goal[1]), 4)]` — as
  `self._record_ply_marker(payload["goal"], payload.get("pos_y"))`. `pos_y` is assigned earlier in
  the same function, so it is available.
- Add module-level `_write_frame_ply` per **C10**.
- In `run_live`: read the config near the existing `livemap_checkpoint_period_s` read, create
  `seq_dir = out_dir / f"{ts}_plyseq"` (only when enabled, `mkdir(parents=True, exist_ok=True)`), and
  keep a local `ply_frame_idx = 0`. Hook the writer immediately after the LIVE `pipe.step` call —
  anchor on `_, _, panel, map_updated = pipe.step(frame, meta, state_pub, show)` **inside `run_live`**,
  NOT the identical line inside the offline runner. Check the enclosing function before editing.
  ```python
  if ply_sequence and map_updated and ply_frame_idx < ply_sequence_max:
      try:
          _write_frame_ply(pipe, seq_dir, ply_frame_idx)
          ply_frame_idx += 1
      except OSError as exc:
          pipe.ply_seq_failures += 1
          pipe.ply_seq_degraded = True
          print(f"*** CRITICAL: frame PLY #{ply_frame_idx} FAILED ({exc}) -- "
                f"{pipe.ply_seq_failures} failure(s) so far; the flight continues but the PLY "
                f"sequence is INCOMPLETE ***", flush=True)
  ```
  Mirror the existing periodic-checkpoint handler's shape exactly. Log once when the cap is first
  reached, following `visrec_cap_logged`'s precedent.
- Write `markers.json` per **C10** whenever `_record_ply_marker` actually appends, and once more in
  `run_live`'s `finally` block beside the final `_checkpoint_livemap` call.

**Acceptance Tests.** New block `SESSION-57 PLY SEQUENCE` in `perception_worker.py`, extending the
duck-typed Pipeline substitute already used by `_self_test_checkpoint_livemap`. Use a temp dir, as
that helper already does — leave no files in `OUTPUT/diag/`.

1. `marker_dedupes_on_change` — recording the same goal three times appends one record; a different
   goal appends a second.
2. `marker_cap_respected` — with `ply_sequence_markers = 2`, a third distinct goal is ignored.
3. `marker_skips_none` — `goal_xz=None` or `pos_y=None` appends nothing.
4. `marker_xyz_shape` — the record's `xyz` is `[goal_x, pos_y, goal_z]` in that order.
5. `frame_ply_written` — `_write_frame_ply` produces a non-empty `frame_00000.ply` whose header says
   `binary_little_endian`.
6. `markers_identical_across_frames` — write frames 0 and 1 with no new markers between them, parse
   both, and assert the marker vertices are at byte-identical coordinates. **This is the property the
   whole feature exists for** — Blender alignment depends on it.
7. `first_frame_stamped_once` — after writing frames 0 and 1, every marker's `first_frame` is 0.
8. `sidecar_matches` — `markers.json` parses and its records equal `pipe.ply_markers`.
9. `failure_is_counted` — force an `OSError` (a directory that does not exist, or a read-only path)
   and assert `ply_seq_failures == 1` and `ply_seq_degraded is True`.
10. `disabled_writes_nothing` — with `ply_sequence: false` the sequence directory is never created.

**Verify.** `python perception_worker.py --self-test` and `python map_store.py --self-test`.
(The gate runs these under the project venv; torch is required and is present there.)

---

## CHUNK 9 — documentation and resume state

**Module Objective.** Leave the tree self-describing so the next session can resume cold from
`STATE.md` alone. Mandated by `CLAUDE.md` — a plan is not complete until this is done.

**Required Context/Dependencies.** Chunks 1-8 applied.

**Target Files.** `plans/session57-planlost-recovery-and-direction-aware-lkg.md` (new),
`PROGRESS.md`, `STATE.md`.

**Strict Interfaces.**

- **New plan file.** Follow the structure of `plans/session56-settle-gate-currency-and-lkg-freeze.md`:
  Origin (the flight, the three findings with their real log lines and the 30-vs-56 episode counts),
  Design (one section per contract group), Traps caught, Files touched, Verification. Record the two
  ideas that were considered and dropped, so they are not re-proposed:
  - a general CV veto over clearance decisions — unsound, because LKG matching is not continuously
    active and therefore cannot continuously veto anything;
  - any raycast change — investigated and cleared. `MapStore.clearance` is a flat HORIZONTAL fan with
    the Y component hardcoded to zero, 10 rays over ±8°, clipped at `clearance_max_range` 10.0 (ring
    1.5), marched at `voxel_size × 0.5` = 2.5 cm. It is not a cone or a frustum and cannot see floor
    or ceiling. It also cannot be the SLAM bottleneck: `slam_ms` times only `slam.process(rgb)`, and
    the clearance fan runs afterwards in `_plan_payload`, outside that window.
- **PROGRESS.md.** Add a session-57 entry in the file's established VERY CONCISE narrative voice —
  "We wanted X. We tried Y. It failed because Z. So we tried W." Detailed design stays in the plan
  file, referenced by name, NOT inlined. Also record the two diagnosed-but-unbuilt findings, each
  with its arithmetic intact and each carrying this warning:

  > **Re-evaluate after session 57's PLAN-LOST rule has flown.** The cached clearance no longer
  > decides whether we look and no longer fires a back-off by itself — it only marks one *pending*,
  > which the camera adjudicates over a 12 s wait. Both may be defanged or moot. Do not schedule
  > either until a post-session-57 flight shows they still bite.

  1. *Capture-age / abandoned heading.* Frame #367 was captured at `cap_ts=48347.718`, reconstructed
     from frame #368 (`cap_ts=48364.906`, `slam_ms=557.9`, solved 08:51:24.530) as **08:51:06.8**.
     SLAM ground on it for 16.5 s publishing nothing, so status read PLAN-LOST from 08:51:09.4 while
     the drone hovered blind; at 08:51:23.6 the answer landed and status flipped to `OK` **on the
     arrival of a 17-second-old answer** (`OK` means "a message arrived within `plan_timeout_s`",
     never "the information is recent"). The drone then turned **−30°**, advanced, and backed off on
     that plan's `clearance 0.60` — measured by the raycast fan along the **pre-turn heading**. While
     hovering, a stale clearance is not very wrong; it turns wrong the moment the drone acts on it,
     which here meant turning first. The finding is therefore not "17.7 s old" but **"measured at a
     heading we have since abandoned, and nothing re-checks that."**
  2. *`_last_good_*` cache currency.* The cache is gated on `plan_valid` alone. `plan_valid` is
     SLAM's "I was tracking" flag and says nothing about age, while PLAN-LOST is a pure age verdict —
     so during a loss the last plan still reads `plan_valid=True` and the cache is re-written every
     tick with a fresh `now`. `_last_good_t` is therefore always ~0, which is why the log prints
     `stale pose, 0.0s old` — not "fresh" but "we touched this variable 0.0 s ago". It is a
     cache-WRITE time, not a capture time; session 52 §7 flagged exactly this trap. Same shape as
     session 56's F_LKG fix (`status == "OK"` was the missing conjunct), one field over.
- **STATE.md.** Keep it ~150-200 lines. Set status to session 57, BUILT, LIVE-FLY PENDING, and
  replace the resume pointer with this watch list, in priority order:
  1. **Every** loss episode now shows `[VISREC]` lines after 12 s, including clear-front ones. Last
     flight: 56 of 86 episodes ran zero matches. Expect that count to go to **zero**.
  2. A `[VISREC]` line reading `closer=LKG` while a back-off was pending → the hold fires and **no**
     BACKOFF follows. This is the operator-reported 08:51 symptom, directly.
  3. `size=` versus `scale=` on the same lines — does the spread ratio hold steady where `scale` swung
     `0.27 → 1.85` within one second?
  4. `LOST_VISUAL_HOLD` notices appear and do **not** repeat every tick.
  5. No two back-offs inside one loss episode closer together than `loss_backoff_grace_s` — the
     restamp is what enforces this now that the one-shot ticket is gone from this path.
  6. Corner-tour goals draw **blue** on the map panel; frontier goals stay yellow.
  7. `OUTPUT/diag/<ts>_plyseq/` fills (only if `diag.ply_sequence` is turned on), `markers.json` is
     written, and the first five markers sit at identical world coordinates in an early and a late
     frame.
  8. **Session 56's own watch list still applies in full** — this is still its first live flight.
  Carry forward the unchanged open problem: **why SLAM chokes** (medians stepped from ~400 ms to
  ~1300-1700 ms on 2026-09-01, between the 17:22 and 21:48 flights). Record that the raycast is now
  ruled out on two independent grounds, so nobody re-derives it.

**Acceptance Tests.** Documentation only — no code, no self-tests. Verify by reading back:

1. `STATE.md` is under 220 lines and names session 57 as current.
2. `PROGRESS.md` contains both deferred findings, each with its numbers and its re-evaluate warning.
3. `plans/session57-planlost-recovery-and-direction-aware-lkg.md` exists and records both dropped
   ideas with their reasons.
4. Nothing in `PROGRESS.md`'s session log inlines implementation detail belonging in the plan file.

**Verify.** Run the whole gate one final time with no reverts in place — all nine suites, 0 failures.
