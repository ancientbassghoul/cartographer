# Plan — SLAM debug logging + state-gated height-calibration fix + timeline offset (session 11)

## Context

Two live-flight problems + one cosmetic catch, surfaced by session-10/11 test flights:

1. **Visibility gap.** During latency spikes we cannot tell if the SLAM pipeline is running
   sequentially or overlapping. We want paired, high-visibility START/FINISH indicators keyed on a
   shared frame `seq_id` so a spike/overlap is instantly obvious — **rendered in the Replay/Debugger
   HTML** (the terminals must stay clean), matching the existing timeline-telemetry pattern.

2. **Height-calibration poisoning (the real bug).** On flight `20260709_122349` a **per-goal**
   `CALIBRATING_HEIGHT` re-tap fired at `12:25:50.569`. The drone ascended to the ceiling
   (`pos_y ≈ -2.30`), did its brief `DESCEND`, but async SLAM only caught up mid-move and the drone
   sank to **`pos_y = -1.768`** (`12:26:00.744`, in `PARALLAX_PUSH`, which does not enforce the
   altitude lock) and stayed ~0.5u low. Because `ground_grid` builds occupancy from a height slab
   **relative to the live camera Y** (`ground_grid.py:103-106`), a low drone corrupts
   clearance/occupancy → it "clips standoffs and blacklists valid frontiers."

   **User's architectural realignment (supersedes the original 6 rules):** judge a calibration's
   *result after it ends*, against a **continuous, state-gated rolling baseline of normal flying
   altitude** — not the ceiling-tap median (too few samples; can't represent "normal ceiling height").
   One master ingestion rule handles prelude and per-goal identically. On failure the drone must
   **climb to clean airspace BEFORE sliding sideways** (never translate while sunk/corrupted).

3. **1 ms timeline skew.** `[SLAM_TRACKER]` payload `t_mono` is snapshotted at loop top
   (`autopilot.py:2019`) while `t_wall` is written later (`:2058`/`:2183`) — a benign single-frame
   poll effect. Confirmed **no compounding offset** (each row snapshots its own values; replay sorts
   by `t_mono`). Fix = unify the capture instant.

**No-leakage compliance:** every new threshold is GENERAL/relative (settle window, margin vs the LIVE
median, a 1.0-unit displacement) — none encodes this room's answer.

---

## Workstream 1 — Paired SLAM logging in the Replay HTML (NOT the console)

The paired frame logs are synthesized by the **autopilot** into the timeline JSONL (reusing the
existing `ev_kind:"slam"` pattern at `autopilot.py:2039-2069`) and rendered by **`flight_replay.py`**.
No printing to the `perception_worker` terminal. This composes with the `cap_ts` plumb (2a): the
autopilot already fires once per fresh `frame_id` and already computes the pose delta — extend that one
block to emit **two** records instead of one, keyed on `frame_id` as `seq_id`:

- **START (orange)** — `ev_kind:"slam_start"`, `t_mono = cap_ts` (frame capture = pipeline ingress):
  `[SLAM_ENGINE] [START] Frame #<seq_id> ingested for computation.`
- **FINISH (green)** — `ev_kind:"slam_finish"`, `t_mono = cap_ts + slam_ms/1000` (compute done):
  `[SLAM_TRACKER] [FINISH] Pose update accepted for Frame #<seq_id> (dx: <+.2f> dy: <+.2f>) Latency: <slam_ms:.0f>ms.`

Because both record times sit in the shared machine-monotonic domain, the `[cap_ts, cap_ts+slam_ms]`
span is poll-independent; **overlapping spans across frames = the multi-threaded-overlap signal**, and
a long span = a latency spike — both obvious in the browser.

- **Coordinate clarity (Trap A):** `dx/dy` are horizontal floor motion = world **X and Z** (vertical
  is **Y**). Reuse the block's existing `_slam_pos` (prev x,z) delta.
- **Boot/recovery guard (Trap A):** the existing null-guard already handles it — when `_slam_pos is
  None` (first frame, or the frame TRACKING comes back online) emit `dx: —, dy: —` and seed the
  tracker; no `TypeError`. **Do NOT clamp the recovery delta** — after a SLAM loss+recover, log the
  true massive jump (the drift scale is useful data).
- **`flight_replay.py`:** update the SLAM filter (`:166`) + CSS (`:118`) — replace the single teal
  `.slam` with distinct high-visibility `.slam_start` (orange) and `.slam_finish` (green) rows so
  START/FINISH pairs and overlaps read at a glance. Old single-`ev_kind:"slam"` record is replaced by
  the pair.

---

## Workstream 2 — State-gated height calibration (core fix)

All in **`autopilot.py`** (`ExploreController`), plus a one-line plumb in `perception_worker.py`.

### 2a. Plumb the camera capture timestamp (`t_historical_state`) → autopilot
`io_bridge.py:594` already stamps `meta["mono_ts"] = time.monotonic()`; the autopilot issues commands
on the SAME `time.monotonic()` (`:2010/2019`) — same-machine monotonic clocks compare directly. In
`perception_worker._plan_payload` (`:316-333`) add `"cap_ts": meta.get("mono_ts")` so it rides EVERY
`TOPIC_PLAN` (valid or not). This one field serves BOTH the START/FINISH records (WS1) and the
settlement gate (2c). Also surface `cap_ts` in the autopilot timeline rows for correlation.

### 2b. Master baseline `_mapping_altitude_history` (continuous, state-gated, frozen during calib)
- New `self._mapping_altitude_history = collections.deque(maxlen=mapping_alt_history_len)` (~200) and
  flag `self._calib_active` (both flight-level, persist across `reset_leg` like `target_altitude_y`).
- **Ingest** near the top of `step()` (using CURRENT `self.state`): append `plan["pos_y"]` **only when**
  `not self._calib_active` AND `self.state in MAPPING_ALT_STATES` AND the plan is healthy (`plan_valid`,
  `pos_y is not None`, `not self._slam_slow`).
  `MAPPING_ALT_STATES = {"ADVANCE","ORIENT","PARALLAX_PUSH","SETTLE"}` (steady horizontal flight at
  mapping height; excludes every vertical/calibration/recovery/dock state). (`EXPLORE` in the user's
  note is the *mode*; these are its states.)
- **Freeze**: set `_calib_active = True` the moment a calibration starts — at prelude `TAKEOFF` entry
  (`:1306`) AND `CALIBRATING_HEIGHT` entry (`:1317`). Stays `True` through
  ASCEND→DESCEND→VERIFY(→ESCAPE→TRANSLATE→retry); cleared ONLY at the explicit **height-OK** transition
  (2d). On the initial prelude the history is empty → the guard can't judge → VERIFY passes immediately
  and unfreezes; the leg loop then fills the baseline for later per-goal re-taps.

### 2c. Descend-issue timestamp + settlement gate (with None guard — Trap B)
- Record `self._descend_issue_t = now` when the DESCEND recipe is first created (`:1436-1437`).
- Gate, guarding a missing/dropped-frame timestamp (comparing `None >= float` would crash):
  ```
  cap_ts = plan.get("cap_ts")
  if cap_ts is None or cap_ts < self._descend_issue_t + calib_settle_gate_s:
      # hold — wait for a settled AND populated telemetry frame
  ```
  (`calib_settle_gate_s` default 1.0 s — frame CAPTURED ≥1 s after descend went out ⇒ dynamics settled,
  latency backlog cleared.)

### 2d. New `CALIB_VERIFY` state (post-descend; replaces the direct DESCEND→SETTLE route)
On `DESCEND` done (`:1439`), for BOTH prelude and per-goal, route to `CALIB_VERIFY` (carry the existing
`_settle_to` — `REPLAN` per-goal, `BASELINE_NUDGE` prelude). `CALIB_VERIFY` holds neutral (no vertical
command, so the TRUE settled altitude is observable). Each healthy tick:
1. Settlement gate not met (2c) → keep holding.
2. **Insufficient baseline** (`len(history) < calib_min_baseline_samples`) → **PASS** (can't judge;
   logged — covers the prelude).
3. `med = median(history)`, `y = plan["pos_y"]` (+Y DOWN: larger = spatially lower).
   - **Init mask (rule 3):** `y < med` (spatially HIGHER) → ignore, keep waiting.
   - **Low-height fail (rule 4):** `y > med + calib_low_height_margin` (significantly lower) → **FAIL**.
   - Else (`med ≤ y ≤ med+margin`) → **PASS**.
4. **Verify timeout** `calib_verify_max_s` (safety) → PASS with a visible logged warning (persistent
   "higher than median" = no sink). No silent fallback.

- **PASS = explicit height-OK:** log `height OK`, clear `_calib_active`, clear `_recalibrating`,
  `_enter(self._settle_to)`.
- **FAIL:** bump a bounded retry counter; if `< calib_max_retries` → **`_enter("ASCEND_ESCAPE")`**
  (`_calib_active` stays True); else abandon the retry with a visible logged warning → `_settle_to`.

### 2e. Vertical-then-horizontal escape (Trap C): `ASCEND_ESCAPE` → `CALIB_TRANSLATE` → re-calibrate
Never slide sideways while sunk at a corrupted low height (risks clipping low furniture/walls).
- **`ASCEND_ESCAPE` (new):** a bounded vertical climb into clean airspace to clear local low obstacles —
  reuse the ASCEND UP-pulse climb until ceiling-gain flattens / flow CEILING (bounded by `ascend_max_s`);
  records **no** tap and **no** latch (purely to gain altitude). Done → `CALIB_TRANSLATE`.
- **`CALIB_TRANSLATE` (new):** clean horizontal translation **1.0 SLAM unit from the CURRENT position**
  (`calib_retry_translate_dist`), now in clean high airspace. Reuse `BASELINE_NUDGE` machinery: pick the
  roomier axis from `plan["clearance_ring"]` (`_ring_get`), distance-quantized off the live pose,
  clearance-guarded + time cap; boxed on all axes → proceed anyway (logged). Done → `CALIBRATING_HEIGHT`
  (→ ASCEND→DESCEND→CALIB_VERIFY). Bounded by `calib_max_retries`.

### 2f. Retire the superseded ascend-time reject
Remove `_ceiling_taps`, `_is_low_object_tap` (`:908-922`), and the `CALIB_NUDGE` state + its ascend-time
trigger (`:1354-1364`, `:1413-1431`) — the low-ceiling-feature case is now handled by
CALIB_VERIFY→ASCEND_ESCAPE→CALIB_TRANSLATE (judge after the routine ends, climb, displace 1.0u, re-run).
Update the `--self-test` `_run_calib` harness (`:2720-2740`) to exercise the new flow.

### New config (all GENERAL/relative — `flight_playbook.json` explore block)
`calib_settle_gate_s` 1.0 · `calib_low_height_margin` 0.3 · `calib_min_baseline_samples` ~10 ·
`mapping_alt_history_len` ~200 · `calib_verify_max_s` ~5.0 · `calib_retry_translate_dist` 1.0 ·
reuse `calib_max_retries` 2, `ascend_max_s`. (`calibrate_on_goal_change`/cooldown/goal-change-dist
unchanged.)

---

## Workstream 3 — Timeline 1 ms offset (cosmetic; verified non-compounding)

In `autopilot.py` at `:2019` capture wall-clock alongside `now`
(`now = time.monotonic(); now_wall = datetime.now()`) and use `now_wall.strftime("%H:%M:%S.%f")[:-3]`
for BOTH the SLAM rows (`:2058`) and the step row (`:2183`) so `t_wall`/`t_mono` share one instant.
One-line comment: the earlier skew was a benign single-frame poll effect, not a compounding tracking
offset (replay still sorts by `t_mono`).

---

## Files to modify
- `autopilot.py` — paired `slam_start`/`slam_finish` timeline records (extend the SLAM_TRACKER block);
  `_mapping_altitude_history` + `_calib_active` + gated ingest; `_descend_issue_t` + None-guarded
  settlement gate; new `CALIB_VERIFY`, `ASCEND_ESCAPE`, `CALIB_TRANSLATE` states; DESCEND reroute;
  remove `_ceiling_taps`/`_is_low_object_tap`/`CALIB_NUDGE`; unify `t_wall`/`t_mono`; update `--self-test`.
- `perception_worker.py` — add `cap_ts` to `_plan_payload` (one line).
- `flight_replay.py` — `.slam_start`(orange)/`.slam_finish`(green) filter + CSS + render.
- `flight_playbook.json` — new explore-block calib params.
- `PROGRESS.md` — session log one-liners once built.

## Build order
plumb `cap_ts` (2a) → calibration states (2b–2f) → replay records/render (WS1) → timeline unify (WS3)
→ update `--self-test` → offline self-tests → live-fly.

## Verification
1. **Offline self-tests (before live):** `autopilot.py --self-test` (new calib flow),
   `perception_worker.py --self-test`, `flow_contact_detector.py --self-test`,
   `frontier_planner.py --self-test`, `ground_grid.py --self-test`.
2. **Paired HTML logging:** run `autopilot.py --explore --log` (or a replay), open `*_timeline.html`;
   confirm each `[SLAM_ENGINE] [START] Frame #N` (orange) is bracketed by its `[SLAM_TRACKER] [FINISH]
   ...Frame #N... Latency:` (green); a latency spike shows a long START→FINISH span and overlaps read
   at a glance. Force a SLAM loss+recover and confirm the true large `dx/dy` recovery jump is logged
   (not clamped); confirm no crash on the first/recovery frame (`_slam_pos is None` → `dx: —`).
3. **Calibration guard (offline logic in `--self-test`):** prime `_mapping_altitude_history` to a
   flying-height median; drive a DESCEND that settles significantly lower with `cap_ts` past the gate →
   assert FAIL → ASCEND_ESCAPE → CALIB_TRANSLATE → re-CALIBRATING_HEIGHT; a settle at flying height →
   PASS/height-OK (`_calib_active` cleared); empty baseline (prelude) → PASS immediately; `cap_ts=None`
   → held (no crash).
4. **Timeline offset:** brief `--explore --log`; confirm a SLAM row's `t_wall` matches its `t_mono`
   instant (no systematic 1 ms lead) and the replay HTML still renders.
5. **Live-fly** full stack (`Xlab.exe` → io_bridge → perception `--no-display` → visualizer →
   `autopilot.py --explore --log`, press `m`): trigger a per-goal re-calibration; confirm the drone no
   longer settles low — a bad result climbs, slides 1.0u in clean airspace, retries; occupancy stays
   clean. Open `*_timeline.html` for the CALIB_VERIFY/ASCEND_ESCAPE/CALIB_TRANSLATE events + clean
   altitude + the START/FINISH latency spans.
