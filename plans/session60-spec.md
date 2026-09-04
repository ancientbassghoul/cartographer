# SESSION 60 — Sonnet-Ready Implementation Specification
# F_LKG owned by perception · bump-latch fix · delete the probe, servo to F_LKG · LKG panel in the visualizer

Run with: `python sonnet_runner.py --plan C:\Users\owner\.claude\plans\valiant-waddling-spark.md`

---

## EXECUTION GUIDELINES (read before every chunk)

1. **Implement ONLY the current chunk.** Do not start, preview, refactor or "improve" any other
   chunk. Earlier chunks are already applied on disk — do not re-verify or re-implement them.
2. **Signatures are contracts.** Use the exact names, parameter names, parameter order, defaults,
   types and return shapes given in `SHARED CONTRACTS`. No renames, no extra parameters, no changed
   return shapes. If a contract looks wrong, implement it as written and say so in your report.
3. **No architectural changes.** Do not restructure the FSM, do not move functions between modules,
   do not introduce new classes, threads, processes or dependencies beyond what a chunk names.
4. **Anchors are source strings, not line numbers.** `autopilot.py` is ~11k lines and a bare `Read`
   truncates it — use `Grep` to locate each anchor string, then `Edit`.
5. **NO SILENT FALLBACKS** (`CLAUDE.md`). Never swallow an error into a default. A degraded or
   suppressed path must set an explicit visible state flag, emit a log line, and be counted.
6. **IMAGE INTEGRITY** (`CLAUDE.md`). Never resize, crop or re-encode a frame that feeds a model. The
   ONE display-only scale in this session is disclosed in C9 — do not add others.
7. **NO MANUAL-FLIGHT DATA LEAKAGE** (`CLAUDE.md`). Every new constant here is a general count,
   ratio or duration. Do not introduce any value derived from a specific flight or room.
8. **Comment in the surrounding style.** Dense "why", not "what", each tagged with its session
   number. Tag new blocks `Session 60:` and cite the evidence in `MISSION CONTEXT`.
9. **Do not commit, stage, stash, or otherwise mutate git state.**
10. **The gate runs nine suites under the project venv after every chunk** — `autopilot.py`,
    `frontier_planner.py`, `visual_recovery.py`, `flight_replay.py`, `ground_grid.py`,
    `map_store.py`, `salvage_flight.py`, `perception_worker.py`, `visualizer.py`. All nine are green
    at the start of this session. Breaking any one halts the run.
11. **Finish by running the self-test commands the chunk names**, and report the full PASS/FAIL list
    verbatim.
12. **Every chunk must change at least one file.** An empty diff is treated as a failed chunk.

---

## MISSION CONTEXT (why this work exists)

Diagnosed off flight `OUTPUT/diag/20260904_103342_autopilot.log` (~37 min), the first live fly of
session 59. Its centrepiece is a **15.3-minute PLAN-STALE** during which SLAM was *not* choked
(169 frames, median `slam_ms` 1902) — it was tracking-lost. So this is recovery logic failing, not
SLAM speed.

**Finding A — F_LKG cannot refresh, because the autopilot reconstructs it from an ID it no longer
holds.** F_LKG must be the exact frame SLAM's plan was computed from. The autopilot keeps
`_lkg_ring`, a `deque(maxlen=160)` of `(frame_id, frame)`, and looks the arriving plan's `frame_id`
up in it. The ring exists only because the plan saying *"frame #1000 was good"* arrives ~15 s AFTER
frame #1000 went past. Measured: NDI ids run at 59.9/s; the ring spans ~1056 ids ≈ **17.6 s** for
**71 MB**; and the age-out shortfall — how far below the ring's oldest the plan frame had fallen —
was **median 33 ids ≈ 0.55 s** (min 6, max 1143). **34 age-outs.** The median missed by half a second.
Session 56's guard then correctly refuses to substitute the live frame (that substitution produced the
`inliers=732 scale=1.00 contained=True` self-match), so the reference silently stays stale.

Note the timing that makes this bite: `_plan_status` declares `PLAN-LOST` at `plan_age > 3.0 s`, and
across 85 `OK` periods this flight the median lasted **3.03 s** — 66% were bare `plan_timeout_s`
blips between slow solves. Those blips are exactly the windows in which F_LKG is meant to refresh.

**Finding B — one wall contact can permanently blacklist a goal.** Two back-off *movements* fired
back-to-back, twice (`10:40:10`→`10:40:28`, and `10:47:19`→`10:47:23`, 4.3 s apart). The maneuver is
NOT at fault: `_step_backoff` is session 30's phase timer (`backoff_hold_s: 1.0`,
`backoff_reverse_mag: 1.0`), a value session 52 deliberately CUT from 2.0 after measuring it "TOO
STRONG", and it performed exactly as tuned (clearance `0.25 → 0.95`, +0.70). Two real causes:
`stop_clearance_dist = 1.25` is both trigger and release (no hysteresis), so starting deep guarantees
a re-fire; and `rearm_bump_if_disengaged` re-arms the bump latch on ANY `reverse > 0` — which a
back-off always commands. So the latch whose entire purpose is *"one continuous contact = one bump"*
is defeated by the back-off itself:

```
10:47:19.124  BUMP pulse #3  goal=[3.9553, -2.65]
10:47:23.425  BUMP pulse #4  goal=[3.9553, -2.65]      <- 4.3s later, SAME contact
10:47:36.976  PLANNER: BUMP count=2/2 -> BLACKLIST PERMANENT
```

Operator's decision: **leave `backoff_hold_s` alone** (*"I'll live with the double backoffs"*); fix
only the latch. Adaptive back-off strength is recorded as a future idea, not built.

**Finding C — the 15° rotation probe has never once recovered SLAM.** Across **139 flight logs**,
per minute of actual exposure:

| state | minutes | recoveries | rec/min |
|---|---|---|---|
| `HOLD_LOST` (hold still) | 185.9 | 2495 | 13.42 |
| `FALLBACK` (turn + push) | 61.5 | 11 | 0.18 |
| **`VISUAL_RECOVERY` (probe)** | **9.4** | **0** | **0.00** |

The exposure is small, so "0" alone is not proof — the **mechanism** is what settles it: the probe
only ROTATES. If the drone has drifted from where F_LKG was taken, spinning on the spot cannot
reproduce that view. (Do NOT read the `HOLD_LOST` row as "holding is 75× better" — it is confounded:
HOLD_LOST catches every loss including the ~66% 3-second blips, while the probe and sweep only see
losses that already survived 12 s of holding.) Last flight:

```
10:50:34  probe's first turn (cum 15/720)
10:58:36  "visual turn search exhausted (720° with no F_LKG re-acquire) -> FALLBACK sweep"
            ^ two full circles, 465 comparisons, ZERO matches
11:01:26  FALLBACK (turns AND pushes) starts matching — 98 successful matches follow
11:04:06  >>> 11 consecutive EQUAL verdicts, 37-50 inliers, over 5.6 seconds <<<
11:04:58  "FALLBACK sweep exhausted (720° over 32 cycles) -> STUCK"
11:05:32  SLAM recovers
```

Verdicts across those 98: **LKG 56, EQUAL 18, UNKNOWN 24**. The drone found the view, swept straight
through it, and hit STUCK 34 s before recovery. Nothing consumed any of it — FALLBACK returns before
any consumer runs.

**Finding D — the probe's grace notice printed 413 times**, once per tick at ~32 Hz, unlike every
other notice in the file.

---

## SHARED CONTRACTS

Read before every chunk.

### C1 — new `config.yaml` keys

Under `network:` (following the per-PUB-binds-its-own-port convention; 5607/5608 are free):
```yaml
    lkg_frame_port: 5607          # perception -> autopilot: the frame SLAM last TRACKED on (F_LKG)
    visrec_canvas_port: 5608      # autopilot -> visualizer: composed F_LKG|LIVE debug canvas
```

Under `autonomy.explore:` — replaces `use_visual_recovery_on_stale` (see C5) and adds the servo knobs:
```yaml
    use_visual_matching: true     # session 60: build the SIFT matcher at all (the F_LKG probe STATE is gone;
                                  #   the matcher still feeds the HOLD_LOST back-off trigger + the FALLBACK servo)
    servo_min_window_s: 1.5       # sustained-confidence window before the servo may act on a verdict
    servo_min_samples: 3          # confident verdicts required in that window (UNKNOWN never counts)
    servo_hold_frames: 3          # SLAM frames CAPTURED at the held pose that must be SOLVED before resuming
```

### C2 — `PerceptionPipeline` publishes F_LKG (`perception_worker.py`)

`_plan_payload` already computes the authoritative condition
(`valid = (res.mode == "TRACKING") and pos is not None and heading_deg is not None`). Stash it so
`run()` can act on it — add to `__init__` and set inside `_plan_payload`:

```python
self.last_plan_valid = False    # bool: the solve just published was a genuine TRACKING solve
```

`run()` constructs `lkg_pub = frame_bus.FramePublisher(lkg_port)` beside `state_pub`, and immediately
after the existing `pipe.step(frame, meta, state_pub, show)` call publishes when
`pipe.last_plan_valid` is True: `lkg_pub.publish(frame, meta)`. `meta` already carries `frame_id` and
`mono_ts`. Close it in the shutdown path beside `state_pub`. `FramePublisher` sets `CONFLATE=1`,
which is correct here — the subscriber only ever wants the newest F_LKG.

### C3 — autopilot consumes F_LKG, and the ring is DELETED (`autopilot.py`)

`run_explore` constructs `lkg_sub = frame_bus.FrameSubscriber(lkg_port)`. Each tick, drain it
non-blocking and, on a frame, store it directly:

```python
got_lkg = lkg_sub.recv(timeout_ms=0)
if got_lkg is not None and visrec_probe is not None:
    _f, _m = got_lkg
    visrec_probe.update_reference(_f, True, src=f"slam:{_m.get('frame_id')}")
```

**DELETE entirely** (age-out is now structurally impossible, so none of this is re-tuned — it goes):
`_lkg_ring`, `_lkg_ageout_last_log`, the `LEGACY LIVE-FRAME mode` startup branch,
`_visrec_should_cache_reference`, and the `ExploreController` fields `visrec_lkg_ring_len`,
`visrec_lkg_ageouts`, `visrec_lkg_degraded`, `visrec_lkg_ageout_log_interval_s` — plus their
`config.yaml` keys and the whole `*** F_LKG AGE-OUT` block.

`_full_vector`'s `visrec_lkg` payload becomes `{"src": <str>}` only (drop `degraded`/`ageouts`).
`visualizer.py`'s `LKG=` row simplifies to the source label — **remove the red `LKG=STALE x<n>`
indicator**, because the condition it reported can no longer occur. Do not leave a dead indicator.

### C4 — `ExploreController.rearm_bump_if_disengaged` (`autopilot.py`)

Signature unchanged. Only the `backward` clause changes:

```python
backward = (float((active or {}).get("reverse", 0.0) or 0.0) > 0.0
            and self.state not in ("BACKOFF", "BLIND_BACKOFF"))
```

Keep the displacement clause EXACTLY as is — the existing comment warns a SLAM-frozen pose stalls
displacement at 0, which is why the command clause has to exist at all while blind.

### C5 — the `VISUAL_RECOVERY` STATE is deleted; the MATCHER is KEPT

**Read this twice.** `visual_recovery.VisualRecoveryProbe` — the SIFT matcher class — **STAYS**. It
feeds session 59's HOLD_LOST back-off trigger and the new servo. Do not touch `visual_recovery.py`.

Delete from `autopilot.py`: the `VISUAL_RECOVERY` state and its members `_step_visual_recovery`,
`_enter_visual_recovery`, `_maybe_enter_visual_probe`, `_reset_visual_recovery`, and the fields
`_visrec_phase`, `_visrec_phase_t0`, `_visrec_cum_deg`, `_visrec_wait_t0`, `_visrec_probe_armed`.
Remove `"VISUAL_RECOVERY"` from `_RECOVERY_STATES`, the `_visrec_phase == "MATCH"` clause from
`wants_visual_match`, and the probe-in-MATCH force condition from `_visrec_should_match`. Delete the
`config.yaml` keys `visrec_turn_step_deg`, `visrec_max_rotation_deg`, `visrec_wait_recover_s`,
`visrec_close_scale`. **Keep** `visrec_size_ratio_hi/lo`, `visrec_size_min_inliers`,
`visrec_match_min_interval_s`, `visrec_debug_window`, `visrec_save_max`.

`use_visual_recovery_on_stale` is **renamed** `use_visual_matching` (default **true**) and now gates
only the matcher's construction (`VisualRecoveryProbe(...) if ctrl.use_visual_matching else None`).

Every site that routed to the probe routes to `_enter_fallback_sweep` instead, **preserving the 12 s
loss grace** that `_maybe_enter_visual_probe` performed (a sweep MOVES, and 96.9% of losses resolve
inside that window). **Latch the grace notice** — it printed 413 times last flight (Finding D).

### C6 — the FALLBACK servo phase (`autopilot.py`)

A new value of the EXISTING `_fallback_phase` — **not** a new top-level state. Sessions 46 and 52 each
had to fix bugs where a fresh recovery state was wiped by the status router one tick after entry; a
phase inside FALLBACK inherits the ownership FALLBACK already has over every status.

New `ExploreController` fields:
```python
self._servo_cap_floor = None     # float | None: cap_ts floor stamped when EQUAL is first reached
self._servo_frames_seen = 0      # int: solved frames CAPTURED at/after that floor
```

Dispatched from `_step_fallback_sweep` before the existing phase ladder. Rules:

- Enter `"SERVO"` on a **sustained confident match**: `visual_match.matched` and
  `closer != "UNKNOWN"`, sustained over `servo_min_window_s` with at least `servo_min_samples`
  confident verdicts. Reuse session 59's `VisualDirectionTally` for the counting.
- While in `"SERVO"`, **do not advance `_fallback_cum_deg`** — finding the view must never push the
  drone toward exhaustion.
- `closer == "LIVE"` (too close) → play the `back_off` recipe (the project's designated small
  backward nudge), then re-evaluate.
- `closer == "LKG"` (too far) → play the new `nudge_forward` recipe (C7), then re-evaluate. **No cap
  on forward travel** (operator's call): F_LKG was captured from a pose the drone physically occupied
  and tracked from, so servoing toward it is servoing toward known-flyable space.
- `closer == "EQUAL"` → **hold**. On first reaching EQUAL, stamp `_servo_cap_floor` from the live
  plan's `cap_ts` and zero `_servo_frames_seen`; thereafter count each solved plan whose `cap_ts >=
  _servo_cap_floor`. At `servo_hold_frames` with no recovery, clear the servo state and resume the
  sweep. A frame COUNT, not a timer, so it self-calibrates to any solve latency (3 s or 70 s). Reuse
  the `_backoff_resolve_since` `cap >= floor` pattern for the floor test.
- Match lost / `UNKNOWN` → clear the servo state and resume the sweep where it left off.

**FALLBACK never exhausts to STUCK.** Remove the `_fallback_cum_deg >= fallback_max_rotation_deg ->
STUCK` transition; the sweep cycles indefinitely (reset `_fallback_cum_deg` and continue). Operator:
*"continue forever… either it'd recover, or I'd stop it manually."* Delete
`fallback_max_rotation_deg` from `config.yaml`. `STUCK` stays reachable from the corner-give-up path.

**Session 59's `_fire_visual_backoff` stays wired in `HOLD_LOST` ONLY.** Remove its FALLBACK
dispatch — inside the sweep the servo owns the verdict. HOLD_LOST is the hold-still state that
produced 2495 of the fleet's ~2500 recoveries and must not gain a servo.

**Known limitation, record it in a comment, do not try to solve it:** the servo controls ONE axis —
distance along the viewing direction. `EQUAL` means "right distance", not "right place".

### C7 — new `flight_playbook.json` recipe

```json
"nudge_forward": [{"trigger": 0.55, "duration_s": 0.3}]
```

Mirrors `back_off` (`reverse 0.7, 0.3 s`) in the forward direction; `presets.forward` is a continuous
drive, not a nudge. Add a one-line note in the `_comment` DERIVATION block stating it is the forward
counterpart of `back_off`, a platform control dynamic, TUNABLE live, NOT a room distance.

### C8 — stacked debug canvas (`visual_recovery.py`)

`_compose_debug` gains a keyword-only `stacked: bool = False` (default preserves today's side-by-side
exactly). When `stacked=True`: F_LKG on top, live below, banner strip above both, and the RANSAC
inlier correspondences drawn manually — `cv2.drawMatches` only composes side-by-side, so for each
inlier draw a line from `(x_lkg, y_lkg)` to `(x_live, y_live + h_lkg)` in the body. Zero-pad to equal
width if the two differ; **never scale** (IMAGE INTEGRITY).

### C9 — the LKG panel (`autopilot.py`, `visualizer.py`)

Autopilot publishes the composed stacked canvas on `visrec_canvas_port` wherever it currently calls
`_visrec_debug_sink`, and **the standalone `cv2` window is retired** — delete the `imshow`/`waitKey`
path, `VISREC_WINDOW`, `_visrec_close_window`, `visrec_window_open`, `visrec_window_failed` and the
`finally`-block `destroyWindow`. **PNG evidence saving to `OUTPUT/diag/<ts>_visrec/` is UNCHANGED**,
including `visrec_save_failed` and the save cap.

> **Do NOT describe this as a SLAM-choke mitigation** in any comment or doc. The SIFT match, the
> canvas composition and the PNG writes all still happen; only `imshow`/`waitKey` leaves the
> autopilot, and an encode + IPC hop + decode are ADDED. Total work goes slightly UP. The only real
> choke experiment is a flight with `visrec_debug_window: false`.

`visualizer.py` subscribes on `visrec_canvas_port` and renders a **new leftmost column**: width
`PANEL_W`, height `MAP_SIZE` (`= PANEL_H*2 + GAP`), so it matches the existing column's geometry.
Composed width becomes `PANEL_W + GAP + PANEL_W + GAP + MAP_SIZE`; height unchanged.
`_open_video_writer` derives its size from these constants, so the MP4 follows automatically. Show a
**grey** placeholder (not black — it must read "idle", not "signal lost") whenever no canvas has
arrived.

**IMAGE INTEGRITY disclosure (the one permitted scale this session):** the canvas is 512 px wide and
the column is 416, so the visualizer scales it to fit, preserving aspect and letterboxing. This is
**display-only**, touches no model input, and the full-resolution canvas still goes to the PNGs
unchanged. State this in a comment at the scaling site.

### C10 — self-test conventions

`autopilot.py` / `visualizer.py` / `perception_worker.py`: `ok = ok and <case>_ok` accumulators with
one `print(f"[self-test] {'PASS' if X else 'FAIL'}  ...")` per case. Name new blocks
`SESSION-60 <TOPIC>`.

---

## CHUNK 1 — perception publishes the frame SLAM tracked on

**Module Objective.** Put F_LKG at its source. Purely additive — nothing consumes it yet, so this
chunk must not change any flight behaviour.

**Required Context/Dependencies.** None. Contracts **C1** (the `lkg_frame_port` key only), **C2**.

**Target Files.** `config.yaml`, `perception_worker.py`.

**Strict Interfaces.** Add the `lkg_frame_port: 5607` key per C1. Add
`self.last_plan_valid = False` to `PerceptionPipeline.__init__` and set it inside `_plan_payload`
from the existing `valid` local (anchor: `valid = (res.mode == "TRACKING") and pos is not None`).
In `run()`, build `lkg_pub` beside `state_pub` (anchor:
`state_pub = frame_bus.StatePublisher(pstate_port)`) and publish after the `pipe.step(...)` call when
`pipe.last_plan_valid`. Close it in the shutdown path.

**Anchor warning:** `_, _, panel, map_updated = pipe.step(frame, meta, state_pub, show)` occurs
**TWICE** — once in the live `run()` and once in `run_offline_video()`, an offline replay path that
must NOT publish. Anchor on the unique line immediately above the live one,
`n_ply_markers_before = len(pipe.ply_markers)`, and edit only there.

Comment the *why*: the plan naming a frame arrives ~15 s after that frame went past, so the autopilot
cannot reliably still hold it (Finding A).

**Acceptance Tests.** New block `SESSION-60 F_LKG SOURCE` in `perception_worker.py`'s self-test:
`last_plan_valid` is False on a fresh pipeline; a `_plan_payload` call with a TRACKING result and a
usable pose/heading sets it True; one with `mode != "TRACKING"`, or a missing pose, or a missing
heading, leaves it False.

**Verify.** `venv\Scripts\python.exe perception_worker.py --self-test`.

---

## CHUNK 2 — the autopilot consumes it, and the ring is deleted

**Module Objective.** Replace the reconstruct-from-ID machinery with a single stored frame. Age-out
becomes structurally impossible, so its entire apparatus is removed rather than re-tuned.

**Required Context/Dependencies.** Chunk 1. Contracts **C3**.

**Target Files.** `autopilot.py`, `visualizer.py`, `config.yaml`.

**Strict Interfaces.** Exactly per **C3**. Anchors: `_lkg_ring = (collections.deque(maxlen=`;
`if _lkg_ring is not None and frame is not None and meta.get("frame_id") is not None:`;
`*** F_LKG AGE-OUT`; `def _visrec_should_cache_reference`; `self.visrec_lkg_ring_len = int(`;
`v["visrec_lkg"] = visrec_lkg`; and in `visualizer.py` `lkg_txt = (f"LKG=STALE x{`.

**Semantic change to state in a comment:** F_LKG previously cached only while the AUTOPILOT's
`status == "OK"`, which folds in the 3 s plan-freshness timeout. It now means *"the last frame SLAM
successfully tracked on"*, full stop — more faithful to `visual_recovery.py`'s own definition; the
3 s rule was an accident of transport latency. Operator-approved 2026-09-04.

**Acceptance Tests.** New block `SESSION-60 F_LKG CONSUMER`: `visrec_lkg` telemetry carries `src` and
no longer carries `degraded`/`ageouts`; a controller built from the repo's `config.yaml` has none of
the four deleted attributes (`hasattr` is False for each); `visualizer.render_telemetry_panel`
renders with a `visrec_lkg` of `{"src": "slam:42"}` and with `None`, without raising.

**Verify.** `venv\Scripts\python.exe autopilot.py --self-test`,
`venv\Scripts\python.exe visualizer.py --self-test`.

---

## CHUNK 3 — one contact, one bump

**Module Objective.** Stop a back-off's own reverse from re-arming the latch that exists to make one
continuous contact count once.

**Required Context/Dependencies.** Chunks 1-2. Contract **C4**.

**Target Files.** `autopilot.py`.

**Strict Interfaces.** Exactly per **C4**. Anchor:
`backward = float((active or {}).get("reverse", 0.0) or 0.0) > 0.0`. Comment the evidence: pulses #3
and #4 4.3 s apart on one contact reached `BLACKLIST PERMANENT` (Finding B). Do NOT change
`backoff_hold_s`, `stop_clearance_dist`, or the back-off maneuver.

**Acceptance Tests.** New block `SESSION-60 BUMP LATCH`: with `_bump_armed=False` and an anchor set,
a `reverse=1.0` command while `state == "BACKOFF"` does NOT re-arm; the same while `state ==
"BLIND_BACKOFF"` does NOT re-arm; the same while `state == "PARALLAX_PUSH"` DOES re-arm; and
displacement `> goal_reach_dist` re-arms regardless of state (the SLAM-freeze-safe clause).

**Verify.** `venv\Scripts\python.exe autopilot.py --self-test`.

---

## CHUNK 4 — delete the probe STATE, keep the matcher

**Module Objective.** Remove `VISUAL_RECOVERY` and route `PLAN-STALE` → 12 s grace → `FALLBACK`.

**Required Context/Dependencies.** Chunks 1-3. Contract **C5**.

**Target Files.** `autopilot.py`, `config.yaml`.

**Strict Interfaces.** Exactly per **C5**. **Re-read C5's first paragraph before deleting anything** —
`visual_recovery.py` and its `VisualRecoveryProbe` class must not be touched. Anchors:
`def _step_visual_recovery`; `def _enter_visual_recovery`; `def _maybe_enter_visual_probe`;
`_RECOVERY_STATES = {`; `if not self.use_visual_recovery_on_stale:`;
`size_min_inliers=ctrl.visrec_size_min_inliers) if ctrl.use_visual_recovery_on_stale else None`.
Preserve the 12 s grace at every site that routed to the probe, and latch its notice so it prints
once per episode instead of 413 times.

**SCOPE WARNING — this is a wide rename, not a two-line one.** `use_visual_recovery_on_stale` appears
**21 times** in `autopilot.py`: the `__init__` read, two `if not self...` guards
(`_maybe_enter_visual_probe`, which is deleted, and `_maybe_loss_snapshot_backoff`, which stays), the
matcher-construction ternary, several docstrings/comments, and **eight self-test fixtures**
(`cfg_vr`, `cfg_gate52`, `cfg_probe_on`, `cfg_probe_off`, `cfg_probe_grace`, `cfg57_stale`,
`cfg_dir57`, and the session-38 isolation at `cfg["autonomy"]["explore"][...] = False`). Grep for all
21 and handle each.

Several self-test BLOCKS exercise the probe specifically and assert behaviour this chunk removes —
they cannot be made to pass and must be **retired**, each with a comment naming session 60 and the
reason. Do NOT weaken them into vacuous passes, and do NOT delete assertions that still describe live
behaviour (e.g. the session-38 config-isolation property, which should be re-pointed at
`use_visual_matching`).

**Acceptance Tests.** New block `SESSION-60 PROBE REMOVED`: `"VISUAL_RECOVERY"` is not in
`_RECOVERY_STATES`; a controller has no `_step_visual_recovery`/`_maybe_enter_visual_probe`
attributes; `use_visual_matching` defaults True from the repo config and gates matcher construction;
a `PLAN-STALE` tick inside the grace holds still and does NOT enter FALLBACK, and past the grace it
DOES; the grace notice is emitted once, not per tick.

**Verify.** `venv\Scripts\python.exe autopilot.py --self-test`,
`venv\Scripts\python.exe visual_recovery.py --self-test`.

---

## CHUNK 5 — servo back to F_LKG inside FALLBACK

**Module Objective.** Let the sweep stop and steer to the F_LKG viewpoint when it finds it, hold for
three solved frames, and never exhaust to STUCK.

**Required Context/Dependencies.** Chunk 4. Contracts **C1** (servo keys), **C6**, **C7**.

**Target Files.** `autopilot.py`, `config.yaml`, `flight_playbook.json`.

**Strict Interfaces.** Exactly per **C6** and **C7**. Anchors: `def _step_fallback_sweep`;
`if self._fallback_cum_deg >= self.fallback_max_rotation_deg:`; and session 59's FALLBACK dispatch of
`_fire_visual_backoff` (remove it; keep the HOLD_LOST one).

**Acceptance Tests.** New block `SESSION-60 FALLBACK SERVO`:

1. a sustained run of confident `LIVE` verdicts enters `"SERVO"` and commands **reverse**;
2. a sustained run of `LKG` enters `"SERVO"` and commands **forward**;
3. `EQUAL` holds — no drive fields — and stamps `_servo_cap_floor`;
4. `servo_hold_frames` solved plans with `cap_ts >= _servo_cap_floor` resume the sweep; fewer do not;
5. plans with `cap_ts` BELOW the floor never count toward it;
6. `_fallback_cum_deg` does not advance while in `"SERVO"`;
7. a run of `UNKNOWN` never enters `"SERVO"`;
8. exceeding the old `fallback_max_rotation_deg` no longer reaches `STUCK` — the sweep continues;
9. `_fire_visual_backoff` still fires from `HOLD_LOST` and no longer from `FALLBACK`.

Update or retire the existing session-31 exhaustion assertions that assert `STUCK` (anchors:
`exhaust_ok = (s == "STUCK" and cex._fallback_cum_deg`, and the `cf.fallback_max_rotation_deg = 2 *`
setups) — they assert behaviour this chunk deliberately removes. Retire them with a comment naming
session 60; do not weaken them into vacuous passes.

**Verify.** `venv\Scripts\python.exe autopilot.py --self-test`.

---

## CHUNK 6 — the LKG panel in the visualizer

**Module Objective.** Put the stacked F_LKG/LIVE view into the dashboard so it lands in the recording,
and retire the separate window.

**Required Context/Dependencies.** Chunks 1-5. Contracts **C1** (`visrec_canvas_port`), **C8**, **C9**.

**Target Files.** `visual_recovery.py`, `autopilot.py`, `visualizer.py`, `config.yaml`.

**Strict Interfaces.** Exactly per **C8** and **C9**. Anchors: `def _compose_debug`;
`cv2.imshow(VISREC_WINDOW, canvas)`; `VISREC_WINDOW = `; `PANEL_W, PANEL_H = 416, 234`;
`left = np.vstack([frame_p, col_gap, tel_p])`; `width = PANEL_W + GAP + MAP_SIZE`. Keep the PNG
saving path and its degradation flag untouched. Re-read C9's boxed warning before writing any comment
about this change.

**Acceptance Tests.** New block `SESSION-60 LKG PANEL`: `_compose_debug(..., stacked=True)` returns a
canvas taller than wide with height `>= h_lkg + h_live` and width `== max(w_lkg, w_live)`, and
`stacked=False` reproduces today's shape exactly; the visualizer composes with a canvas present and
with none (grey placeholder) without raising, and the composed width equals
`PANEL_W + GAP + PANEL_W + GAP + MAP_SIZE` in both cases.

**Verify.** `venv\Scripts\python.exe visual_recovery.py --self-test`,
`venv\Scripts\python.exe visualizer.py --self-test`,
`venv\Scripts\python.exe autopilot.py --self-test`.

---

## CHUNK 7 — documentation, resume state, and archive this spec

**Module Objective.** Leave the tree self-describing (CLAUDE.md's two closing steps) and keep this
spec in the repo.

**Required Context/Dependencies.** Chunks 1-6.

**Target Files.** `PROGRESS.md`, `STATE.md`, `plans/session60-spec.md` (new).

**Strict Interfaces.**

1. Copy `C:\Users\owner\.claude\plans\valiant-waddling-spark.md` verbatim to `plans/session60-spec.md`.
2. `PROGRESS.md` — one concise narrative session-60 entry in the house voice. Must record: F_LKG moved
   to perception (34 age-outs, median shortfall 0.55 s); one contact could permanently blacklist a
   goal; the probe never recovered SLAM in 139 logs and was deleted in favour of a servo inside
   FALLBACK; and the LKG panel moved into the visualizer. Reference `plans/session60-spec.md`.
3. `STATE.md` — keep ~150-200 lines. Replace the session-59 immediate-next block with session 60's
   watch list: **zero** `F_LKG AGE-OUT` lines (34 last flight — the message no longer exists);
   `src=slam:<id>` advancing during the ~3 s `OK` blips; no two bump pulses on one contact; no
   `VISUAL_RECOVERY` anywhere; a `SERVO` phase engaging on a sustained match with `_fallback_cum_deg`
   frozen; FALLBACK never reaching `STUCK`; the LKG panel visible and grey when idle. Record the kill
   switches: `use_visual_matching: false` disables all matching; `visrec_debug_window: false` disables
   the canvas.
4. `STATE.md` — carry forward, unchanged in priority: the **SLAM choke** as the dominant open problem
   (with the per-5-minute table already there), **bump-pulse latency**, the **staleness UI**, and the
   **goal-management rewrite** decision rule. Add to "Future ideas": **adaptive back-off strength**
   scaled to the clearance deficit (operator's idea — a fixed duration cannot serve a trigger range of
   0.25 … 1.20; note that post-back-off clearance readings are contaminated by SLAM re-localisation,
   so a closed loop must not naively trust the next reading), and **closing session 47's dead
   `_backoff_resolve_since` gate** on the PLAN-LOST path.

**Acceptance Tests.** None (documentation only). The nine-suite gate must still pass.

**Verify.** `venv\Scripts\python.exe autopilot.py --self-test`. Then confirm by inspection that
`STATE.md` alone is enough to resume cold.
