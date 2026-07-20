# Session 29 — clearance details debugger tab + direction-cycling blind recovery sweep

## Origin

The operator flew `20260720_180112` on the session-28 build and asked about a stuck episode at 18:08:33:
plan recovered from PLAN-LOST, went PLAN-STALE one frame later, and the drone spent ~113s bouncing
HOLD_LOST↔FALLBACK before giving up into STUCK, where it then sat for ~83 more recorded seconds looking
(by SLAM solve-time alone) healthy. Three questions, diagnosed off the raw `_timeline.jsonl`:

1. Why didn't clearance stop this, and could the debugger show the raw ray-hit picture instead of just the
   post-vote outcome?
2. `pos`/`heading` were frozen the whole 113s (SLAM blind) but the physical drone was visibly turning,
   pushing, and getting straightened back out by a wall on repeat — why?
3. It eventually reached STUCK next to what looked like healthy SLAM (solve times back to normal) and just
   sat there — why didn't the recovery convergence pull it back out?

## Diagnosis

**(1)** `ring_clear`/`forward_clearance_dist` only ever exposed the post-vote distance or `None` — not the
hit-count/fraction/closest/farthest behind the judgment. Nothing to fix logically; built the requested tab.

**(2)** `_begin_fallback()` always turned the same fixed `+recovery_turn_step_deg` (15°) each attempt — a
deliberate unidirectional sweep meant to eventually face away from any one wall — but picked the PUSH
direction from `self._last_ring`, a snapshot frozen from *before* SLAM was lost and never refreshed for the
rest of the blind episode. Across 31 attempts (465° of commanded turning), the push kept reusing one
increasingly-stale "which way is open" judgment in body-relative terms. If the drone is genuinely pinned
near a wall, this can repeatedly push it right back into the same wall, and a wall contact naturally
re-aligns (straightens) the nose — erasing the turn sweep's progress each cycle. This matched the operator's
account exactly.

**(3)** FALLBACK hit `fallback_max_attempts` and entered STUCK — a *designed* terminal hold: the generic
recovery convergence (`autopilot.py`, the `st in _RECOVERY_STATES` branch) would pull it back out the
instant `status` reads OK again, so staying in STUCK for ~83s means status genuinely never got there. The
misleading "healthy" appearance: SLAM's solve TIMES looked normal (400ms–2s, no more session-28-style
multi-second stalls), but every one of ~40 solves in that window reported `dx:+0.00 dy:+0.00` — far more
consistent with SLAM stuck in a non-tracking/relocalizing mode repeating the same frozen pose than with 83s
of genuinely-reacquired tracking. STUCK's own "logging paused" design is exactly why the jsonl goes dark
right when it would be most useful. **Operator decision: leave STUCK's logging as-is; no fix this session.**

A THIRD, previously-undiscovered issue surfaced while rewriting the FALLBACK direction logic:
`fallback_max_attempts` (config: 16) was only ever checked from FALLBACK's own internal "settle then
continue" path — NOT from the top-level PLAN-STALE entry paths that `_step_stale` re-enters through every
time the connection flickers back through HOLD_LOST first (which is exactly what this flight did, bouncing
HOLD_LOST↔FALLBACK). So the cap was silently bypassed for the entire episode; the drone actually reached 31
attempts against a configured cap of 16, and STUCK only fired once a later cycle happened to stay in
PLAN-STALE long enough to reach the one code path that checked it.

## Built — Part A: Clearance details tab

- `map_store.py`, `MapStore.clearance()`: new `detail: bool = False` param. Collects all ray hits (already
  done since the session-28 `min_hit_fraction` fix) and, when `detail=True`, returns
  `{"dist", "n_hits", "n_rays", "fraction", "min_dist", "max_dist", "blocked"}` instead of a bare
  float/None. `dist`/`blocked` match exactly what a plain call would return/act on; the raw stats fields
  are independent of the vote, so the tab shows WHY, not just the outcome. Well-formed even for an
  empty/unusable map (all-zero row, never `None`). New self-tests (d)/(e)/(f) in `run_self_test()`.
- `perception_worker.py`, `_plan_payload()`: computes `clearance_detail` at the 4 cardinal offsets (fwd/
  back/left/right) using the SAME parameters the ring/TRIM/PARALLAX_PUSH/FALLBACK already consult
  (`ring_max_range`, not the longer forward-cruise range) and publishes it on `TOPIC_PLAN`.
- `autopilot.py`, `_timeline_step_record()`: threads `clearance_detail` onto every timeline record
  alongside the existing `ring_clear`.
- `flight_replay.py`: new floating `#clrdet` panel (toggle button "Clearance", draggable header, close
  button) mirroring the existing Goals DB panel's shell/pattern exactly — a 4-row table (Forward/Backward/
  Left/Right) × (hits/rays, fraction, closest, farthest, judged). New self-test assertion (`c_clr`).

All Part-A self-tests green: `map_store.py --self-test`, `perception_worker.py --self-test`,
`flight_replay.py --self-test`.

## Built — Part B: direction-cycling blind recovery sweep

**File: `autopilot.py` only.**

- New per-episode state: `_fallback_dir_queue` (shuffled remaining directions this lap),
  `_fallback_dir_current`, `_fallback_dir_opposite` (bool), `_fallback_dir_tries_left`, `_fallback_laps`,
  `_fallback_push_dirn` (the resolved, opposite-flip-applied direction of the IN-FLIGHT attempt — read by
  the live-contact check below). New knob `fallback_dir_tries` (config default 4). New helper
  `_reset_fallback_dir_search()`, called everywhere `_fallback_attempts` is reset (a fresh loss episode,
  a confirmed recovery, `reset_leg()`'s manual-takeover reset) so a new blind episode always starts a fresh
  shuffled lap.
- `_begin_fallback()` rewritten: cycles a shuffled `[forward, backward, left, right]` queue — each
  direction gets `fallback_dir_tries` attempts, then its OPPOSITE gets `fallback_dir_tries` more, before
  moving to the next direction. Forward IS now a candidate (operator's explicit call: "we might as well be
  with our back to the wall and a push forward will save us" — unlike normal scouting, there's no live
  signal while blind saying the back is any safer than the front). Both exhaustion checks — the
  pre-existing attempt-count cap AND the new two-full-passes cap — are now unified INSIDE
  `_begin_fallback`, the one place every caller funnels through (fixing the silently-bypassed-cap issue
  found above). Forward pushes use `forward_preset` with `trigger` throttled down to `reverse_throttle`
  (not full cruise speed) — gentler than a normal ADVANCE push, matching the existing backward/side push
  magnitudes already used here.
- FALLBACK state handler: a live SLAM-independent `wall_contact` (forward push) or `backwall_contact`
  (backward push) now ends that attempt EARLY (still counts as a spent try) instead of grinding out the
  full recipe duration into a wall it can't see — checked only on the recipe's LAST step (the push),
  via `RecipePlayer.i`. No equivalent live signal exists for left/right, so those always run their full
  budget.
- `config.yaml`: added `fallback_dir_tries: 4`; raised `fallback_max_attempts` 16 → 70 (2 laps × 4
  directions × 2 phases × 4 tries = 64 worst-case attempts under the new search — the old 16 would have cut
  the 2-lap search off long before it could complete on its own terms; 70 leaves it as a backstop instead).
- New self-test block "FALLBACK direction-cycling search": (a) a direction exhausts its try budget then
  flips to its opposite; (b) a live contact on the matching direction ends an attempt early, a MISMATCHED
  contact (e.g. backwall during a forward push) does NOT; (c) 2 complete laps with no recovery → STUCK with
  a lap-based reason string, distinct from the plain attempt-count wording; (d) a fresh loss episode resets
  stale mid-search state (queue/laps) before the next attempt runs. Two PRE-EXISTING tests updated for the
  new mechanism (forward pushes are no longer forbidden; the turn-order test seeds a deterministic
  direction instead of relying on the now-removed `_last_ring`-based pick).

`python autopilot.py --self-test`: **ALL PASS** (full suite, including the two updated pre-existing tests
and the new direction-cycling block).

## Verification

No hardware/GPU in this environment — self-tests only. Live-fly checklist for the next flight:
- Does the Clearance tab's per-direction stats explain a future `"ring blocked"` judgment at a glance
  instead of requiring hand-derivation from raw voxel data.
- Does a blind recovery episode visibly cycle through different push directions (watch the FALLBACK event
  lines: `push {dirn} [{direction} N/tries, lap L/2]`) instead of repeating the same one; does the
  straighten-against-the-wall loop the operator saw actually stop recurring.
- Does a live wall/backwall contact visibly cut a phase short (shorter FALLBACK attempts than the full
  `fallback_retreat_s` duration when a contact is present).
- Total search time on a real stuck episode, to retune `fallback_dir_tries`/`fallback_max_attempts` — the
  64-attempt worst case is an unverified estimate against real settle/turn durations.
