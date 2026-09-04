# SESSION 58 — Sonnet-Ready Implementation Specification
# Honour the loss grace before looking · LKG window becomes loss-only · dead-goal bump guard

Run with: `python sonnet_runner.py --plan plans/session58-spec.md`

---

## EXECUTION GUIDELINES (read before every chunk)

1. **Implement ONLY the current chunk.** Do not start, preview, refactor or "improve" any other
   chunk. Earlier chunks are already applied on disk — do not re-verify or re-implement them.
2. **Signatures are contracts.** Use the exact names, parameter names, parameter order, defaults,
   types and return shapes given in `SHARED CONTRACTS`. No renames, no extra parameters, no changed
   return shapes. If a contract looks wrong, implement it as written and say so in your report.
3. **No architectural changes.** Do not restructure the FSM, do not move functions between modules,
   do not introduce new classes, threads, processes or dependencies beyond what a chunk names.
4. **Anchors are source strings, not line numbers.** `autopilot.py` is ~10.7k lines and a bare `Read`
   truncates it — use `Grep` to locate each anchor string, then `Edit`.
5. **NO SILENT FALLBACKS** (`CLAUDE.md`). Never swallow an error into a default. A degraded path must
   set an explicit visible state flag, emit a CRITICAL log line, and be counted. Prefer raising over
   absorbing.
6. **IMAGE INTEGRITY** (`CLAUDE.md`). Never resize, crop, downscale or re-encode a frame. The
   zero-pad-never-scale behaviour in `visual_recovery.py` is load-bearing.
7. **NO MANUAL-FLIGHT DATA LEAKAGE** (`CLAUDE.md`). This session introduces NO new constants at all.
   Do not add any value derived from a specific flight or room.
8. **Comment in the surrounding style.** This codebase carries dense "why", not "what", comments,
   each tagged with its session number. Tag new blocks `Session 58:` and say *why*, citing the
   evidence in `MISSION CONTEXT`. Match the local density; no banner comments.
9. **Do not commit, stage, stash, or otherwise mutate git state.**
10. **The gate runs nine suites under the project venv after every chunk** — `autopilot.py`,
    `frontier_planner.py`, `visual_recovery.py`, `flight_replay.py`, `ground_grid.py`,
    `map_store.py`, `salvage_flight.py`, `perception_worker.py`, `visualizer.py`. All nine are green
    at the start of this session. Breaking any one of them halts the run, not just the module you
    edited.
11. **Finish by running the self-test commands the chunk names**, and report the full PASS/FAIL list
    verbatim. A chunk that leaves any suite failing is not complete.
12. **Every chunk must change at least one file.** An empty diff is treated as a failed chunk.

---

## MISSION CONTEXT (why this work exists)

Diagnosed off flight `OUTPUT/diag/20260903_223345_autopilot.log` (7 min, 25 loss episodes) — the
first live fly of session 57. Session 57's headline fix **worked**: all 25 loss episodes ran LKG
matching (the previous flight's 56-of-86 zero-match count went to 0 of 25). Flying it surfaced three
new defects, all fixed here. Full design record: `plans/session58-lkg-window-discipline-and-dead-goal-guard.md`.

**Finding 1 — the 12 s grace is announced but not honoured for the camera.** Every episode reads:

```
22:39:40.399  plan status: PLAN-LOST
22:39:40.399  *** LOSS-RECOVERY GRACE: holding still for 12s ... Looking at the camera after that. ***
22:39:40.490  [VISREC] has_lkg=True matched=True inliers=75 ... closer=EQUAL     <- 91 ms later
```

`ExploreController.wants_visual_match` short-circuits on `not self._loss_snapshot_checked` **before**
it reaches session 57's grace clause. That one-shot ticket is re-armed to `False` at every loss edge
(anchor: `self._loss_snapshot_checked = False`) and session 57 removed the only code that ever spent
it on this path (`_maybe_loss_snapshot_backoff` is now `PLAN-STALE`-only). So on PLAN-LOST the ticket
is permanently `False`, the first clause always returns `True`, and **session 57's grace clause is
unreachable dead code**. Measured: **233 of 253 matches (92 %) ran inside a grace window no consumer
could read**; of the 20 that matured, **zero** produced a decision (0 `LOST_VISUAL_HOLD`, 0
loss-instant back-offs). Only 4 of 25 episodes outlived the grace at all (durations: median 4.1 s,
p90 13.1 s). That is a full SIFT + BFMatcher + RANSAC every 0.5 s on the CPU while SLAM is fighting to
relocalize — the exact contention `visual_recovery.py`'s docstring says the module is CPU-only to
avoid, and a live suspect for the still-open SLAM-choke question in `STATE.md`.

**Finding 2 — the LKG debug window opens at arm and never closes.** Not a NO-PLAN first frame: the
first `[VISREC]` of the flight is 79 s after takeoff, and `match()` returns before composing anything
while `_lkg is None`. The cause is session 49's **idle refresh** in `run_explore`, whose condition is
only `visrec_debug_window and _lkg is not None and 0.5 s elapsed` — no loss gate at all. It fires
from the first `plan_valid` plan (at arm) for the rest of the flight.

**Finding 3 — the window flickers because the idle canvas is `F_LKG | F_LKG`.**
`_compose_debug(self, frame, out, ...)` takes the **live** frame first; the idle branch hands it
`visrec_probe._lkg`. With no keypoints it does `np.hstack([pad(lkg), pad(frame)])`, so the idle canvas
draws the reference twice and labels the right half "LIVE". During a loss two independent 0.5 s
timers race into the same window — the match path (live frame + drawn inliers) and the idle path (a
second copy of F_LKG) — so the right pane alternates at ~2 Hz between two different images. Half the
frames the operator uses to judge "is F_LKG closer than live?" show F_LKG on both sides.

**Finding 4 — a permanently-blacklisted goal can still be bumped.** 1.0 s after
`BLACKLIST PERMANENT` retired goal `[3.9, -3.6]`, the autopilot fired bump pulse #2 at that same dead
goal and backed off against it:

```
22:40:24.170  PLANNER: BUMP goal=[3.9, -3.6] count=2/2 -> BLACKLIST PERMANENT (1 total) -> reselecting
22:40:25.165  BUMP pulse #2 goal=[3.9, -3.6] (clearance stand-off (post-recovery settle) -> planner)
22:40:25.165  BACKOFF: SLAM settled after 1.0s but clearance 0.97 <= 1.25 -> standoff back off
22:40:26.133  PLANNER: BUMP goal=[3.9, -3.6] count=1/2 (armed)      <- re-arms on a dead region
```

The planner never re-selected it — the autopilot's own `leg_goal` was still the dead goal, and
`_register_bump` checks `leg_goal is None` and the far-corner guard but **never the live blacklist**,
even though the arrays ride every plan and session 28 already built exactly that re-check for the TRIM
resume path. The plan in hand at `22:40:25.165` already carried
`blacklist=[[3.9,-3.6]], blacklist_permanent=[True]`, so the guard specified below is confirmed to
fire on the real trace. In the replay this renders as an `active` marker (drawn from `leg_goal` by
`_timeline_goals`) sitting on its own `blacklist_permanent` ring — what the operator read as "the goal
got selected again in the same place".

**Out of scope, deferred to session 59** (recorded in the design doc, do not build): the bump pulse
takes 10-18 s to become a blacklist the autopilot can see, because it only rides the next published
plan and `perception_worker.run()` is `recv -> drain -> pipe.step()` with 8-10 s SLAM solves.

---

## SHARED CONTRACTS

Read this section before every chunk. **No new config keys are added this session.**

### C1 — `ExploreController.wants_visual_match` (restructured body)

Signature is UNCHANGED: `def wants_visual_match(self, now=None, status=None):`. The body becomes:

```python
if self._visrec_phase == "MATCH":
    return True
if status in ("PLAN-LOST", "NO-PLAN"):
    return (now is not None and self._loss_episode_t0 is not None
            and (now - self._loss_episode_t0) >= self.loss_backoff_grace_s)
return not self._loss_snapshot_checked
```

Behaviour table (the contract every assertion tests against):

| `_visrec_phase` | `status` | ticket `_loss_snapshot_checked` | episode age | result |
|---|---|---|---|---|
| `"MATCH"` | any | any | any | `True` |
| any | `"PLAN-LOST"`/`"NO-PLAN"` | **ignored** | `>= loss_backoff_grace_s` | `True` |
| any | `"PLAN-LOST"`/`"NO-PLAN"` | **ignored** | `< grace`, or `_loss_episode_t0 is None`, or `now is None` | `False` |
| any | anything else, incl. `None` | `False` (armed) | n/a | `True` |
| any | anything else, incl. `None` | `True` (spent) | n/a | `False` |

The ticket is deliberately NOT consulted on the PLAN-LOST/NO-PLAN row — that is the whole fix.

### C2 — `ExploreController.visrec_window_open` (new instance flag)

```python
self.visrec_window_open = False   # session 58: an LKG canvas is currently SHOWN in the OS window
```

Declared beside the two existing degradation flags (anchor:
`self.visrec_save_failed = False      # imwrite/makedirs raised -> saving disabled, window continues`).
Distinct from `visrec_window_failed` (a permanent degradation) — this one is episode-scoped state and
flips both ways.

### C3 — `autopilot._visrec_close_window` (new module-level function)

```python
def _visrec_close_window(ctrl, diag):
    """Close the LKG debug window at the end of a loss episode. No-op unless one is open."""
```

Returns `None`. Sibling of `_visrec_debug_sink`, placed immediately after it. Behaviour:

- Returns immediately unless `ctrl.visrec_window_open` is True.
- Calls `cv2.destroyWindow(VISREC_WINDOW)`; clears `ctrl.visrec_window_open = False` on success.
- On exception: sets `ctrl.visrec_window_failed = True`, clears `ctrl.visrec_window_open = False`,
  `print(...)` + `diag.line(...)` ONE CRITICAL line (NO SILENT FALLBACK — mirror the wording style of
  `_visrec_debug_sink`'s existing window-failure line), and does not re-raise.

### C4 — `ExploreController._goal_is_blacklisted` (new method)

```python
def _goal_is_blacklisted(self, plan, goal):
    """True when `goal` sits inside a PERMANENTLY blacklisted region of the live plan."""
```

Returns `bool`. Lifted VERBATIM from the predicate already inside `_trim_resolve_resume` (anchor:
`dead = g is not None and any(`) — same arrays (`plan.get("blacklist")` /
`plan.get("blacklist_permanent")`, the ones `_timeline_goals` and the visualizer already read), same
radius (`self.goal_area_radius`), same soft-vs-permanent rule (**only** `permanent` entries count).
Returns `False` when `goal is None`. Placed immediately before `_register_bump`.

### C5 — PNG evidence predicate (`run_explore` local)

`decision` currently reads `loss_edge or ctrl._visrec_phase == "MATCH"`. After chunk 1 a `loss_edge`
tick never runs a match, so this would silently stop writing PLAN-LOST evidence. It becomes
**the first match of each loss episode, or a probe re-match**, using a new `run_explore` local
`visrec_episode_saved` (bool, initialised `False`, set `False` on every `loss_edge`, set `True` after
a canvas is saved).

### C6 — self-test conventions

This repo's suites are `ok = ok and <case>_ok` accumulators printing one
`print(f"[self-test] {'PASS' if X else 'FAIL'}  ...")` line per case, run via
`python <module>.py --self-test`. Follow the existing block style exactly: a leading comment naming
the session and the defect, then per-case asserts folded into the module's overall `ok`. Name new
blocks `SESSION-58 <TOPIC>`.

---

## CHUNK 1 — honour the loss grace before looking

**Module Objective.** Make session 57's grace clause reachable, so LKG matching runs only when a
consumer can read the result. Removes ~92 % of the flight's SIFT load.

**Required Context/Dependencies.** None (first chunk). Contract C1.

**Target Files.** `autopilot.py` only.

**Strict Interfaces.**

- Restructure `ExploreController.wants_visual_match` exactly per **C1**. Anchor on the source string
  `if (not self._loss_snapshot_checked) or self._visrec_phase == "MATCH":`.
- Update its docstring. It currently claims the session-57 clause "re-opens the camera for every
  MATURED episode regardless of the ticket's state" — true in intent, false in effect, because the
  ticket clause ran first. Say plainly that PLAN-LOST/NO-PLAN never spends the ticket, so consulting
  it there made the grace unreachable, and cite Finding 1's numbers (233 of 253 matches).
  **Preserve the CALLER CONTRACT paragraph's meaning** — this is still an AND-narrowing of
  `run_explore`'s status gate, never a replacement for it; a fresh controller still reads `True` for
  a non-loss status, which is why `needs_match` remains load-bearing.
- Thread `now`/`status` into the **memo-reuse** call site so a match cached from a PREVIOUS episode
  cannot be replayed into `visual_match` during a grace. Anchor:
  `if not do_match and needs_match and visrec_memo is not None and ctrl.wants_visual_match():`
  → `... and ctrl.wants_visual_match(now=now, status=status):`.
  (`_step_lost_recovery` already returns before reading `visual_match` during the wait, so the only
  observable effect is that it stays `None` — which is what the code claims it does.)

**Do NOT** change `_visrec_should_match`, `_step_lost_recovery`, `loss_backoff_grace_s`, or the
`_loss_snapshot_checked` re-arm at the loss edge.

**Acceptance Tests.** New block `SESSION-58 GRACE BEFORE LOOKING` in `autopilot.py`'s
`run_explore_self_test`, placed immediately after the existing `SESSION-57 PLAN-LOST ALWAYS LOOKS`
block. Reuse the existing `_mk_gate_ctrl(one_shot_spent=..., phase=...)` and `_gate(c, **kw)` helpers
defined in that function. Every case uses the REAL flight condition — the ticket **ARMED**
(`one_shot_spent=False`) — which is what the session-57 block never covered:

1. `armed_inside_grace_does_not_look` — `one_shot_spent=False`, `_loss_episode_t0 = t0`,
   `wants_visual_match(now=t0 + grace - 0.1, status="PLAN-LOST")` → **False**.
   *(Today this returns True — the defect being fixed.)*
2. `armed_matured_looks` — same controller at `now = t0 + grace + 0.1` → **True**.
3. `armed_probe_still_looks` — `_mk_gate_ctrl(one_shot_spent=False, phase="MATCH")` with
   `_loss_episode_t0 = t0`, called inside the grace → **True** (the probe must never be gated by it).
4. `armed_stale_unaffected` — `status="PLAN-STALE"` with the ticket armed → **True**.
5. `armed_no_plan_matches_plan_lost` — repeat case 1 and case 2 with `status="NO-PLAN"`; same answers.
6. `armed_gate_blocks_compute` — `_gate(c, memo=None, now=t0 + grace - 0.1, status="PLAN-LOST")` is
   **False** and `_gate(c, memo=None, now=t0 + grace + 0.1, status="PLAN-LOST")` is **True**, proving
   the predicate reaches the actual compute decision.

**Regression requirement.** All six existing `SESSION-57 PLAN-LOST ALWAYS LOOKS` assertions must
still pass **unmodified** — they build with `one_shot_spent=True`, which is exactly why this defect
shipped. Do not edit that block. Several older blocks also consume `wants_visual_match` (anchors:
`predicate_ok = (armed.wants_visual_match() is True`, `healthy_ok = (fresh.wants_visual_match() is True`,
`gate2_armed_ok`, `probe1_tick1_ok`) — confirm each still passes rather than adjusting it.

**Verify.** `venv\Scripts\python.exe autopilot.py --self-test` — 0 failures.

---

## CHUNK 2 — the LKG window becomes loss-only and stops flickering

**Module Objective.** Delete the idle refresh (killing both the arm-time pop-up and the two-image
flicker), scope the window to a loss episode, and keep the PNG evidence alive after chunk 1.

**Required Context/Dependencies.** Chunk 1. Contracts C2, C3, C5.

**Target Files.** `autopilot.py` only.

**Strict Interfaces.**

- **Delete the idle-refresh branch entirely.** Anchor on
  `elif (ctrl.visrec_debug_window and visrec_probe._lkg is not None` and remove that `elif` and its
  whole body (the `why` / `idle_canvas` / `_visrec_debug_sink` / `visrec_last_idle_show = now` lines).
  Also delete the now-unused local `visrec_last_idle_show` and its initialiser (anchor:
  `visrec_last_idle_show = 0.0 # throttle for the idle "reference only" refresh (session 49)`).
  Leave `visrec_probe._compose_debug` itself untouched — `match()`'s four failure return paths still
  call it, so it is not dead code.
- Add `ctrl.visrec_window_open` per **C2**.
- In `_visrec_debug_sink`, set `ctrl.visrec_window_open = True` immediately after the
  `cv2.waitKey(1)` line (i.e. only on a successful show). On the exception path, leave it `False`.
- Add `_visrec_close_window` per **C3**, immediately after `_visrec_debug_sink`.
- In `run_explore`, compute the recovery edge beside the existing loss edge. Anchor:
  `loss_edge = loss_now and visrec_prev_status not in ("PLAN-LOST", "NO-PLAN", "PLAN-STALE")` —
  add on the following line:
  ```python
  recover_edge = (not loss_now) and visrec_prev_status in ("PLAN-LOST", "NO-PLAN", "PLAN-STALE")
  ```
  and call `_visrec_close_window(ctrl, diag)` on that edge, guarded by `ctrl.visrec_debug_window`.
- Apply **C5**: add the `visrec_episode_saved` local (initialise beside `visrec_saved`), clear it on
  `loss_edge`, and change the `decision` line (anchor:
  `decision = loss_edge or ctrl._visrec_phase == "MATCH"`) to save on the first match of an episode
  or a probe re-match. Set `visrec_episode_saved = True` where `visrec_saved` is already incremented.
- Comment the *why* on both halves: the window's appearance is now a SIGNAL that a loss outlived the
  grace (chunk 1 makes that rare — 4 of 25 episodes on the diagnosing flight), and the `decision`
  redefinition exists because chunk 1 makes `loss_edge` ticks match-free.

**Do NOT** touch the `finally`-block teardown (anchor: `cv2.destroyWindow(VISREC_WINDOW)` inside
`finally`) — it stays as the flight-end backstop. **Do NOT** change `visrec_debug_window`'s default
or its `config.yaml` value.

**Acceptance Tests.** Extend the existing block that already monkey-patches `cv2.imshow` /
`cv2.imwrite` (anchor: `# (b) WINDOW FAILURE IS ISOLATED AND LOUD:`) with a new block
`SESSION-58 LKG WINDOW SCOPE`, reusing its `_FakeDiag`, `canvas49` and `import cv2 as _cv2mod`
aliasing pattern:

1. `window_open_flag_set` — a controller with `visrec_debug_window = True` and a working `imshow`:
   `_visrec_debug_sink(...)` leaves `visrec_window_open is True`.
2. `window_open_flag_clear_on_close` — then `_visrec_close_window(ctrl, diag)` leaves
   `visrec_window_open is False` (patch `_cv2mod.destroyWindow` to a no-op lambda so the test needs
   no display).
3. `close_is_noop_when_never_opened` — a controller with `visrec_window_open is False`:
   `_visrec_close_window` does not raise, does not call `destroyWindow` (assert via a counter on the
   patched lambda), and leaves `visrec_window_failed is False`.
4. `close_failure_is_loud_and_isolated` — `destroyWindow` patched to raise: `visrec_window_failed`
   becomes `True`, `visrec_window_open` becomes `False`, and nothing propagates.
5. `window_failure_still_blocks_open_flag` — with `imshow` patched to raise (the existing case (b)
   setup), `visrec_window_open` stays `False` while the PNG is still written.

**Verify.** `venv\Scripts\python.exe autopilot.py --self-test`,
`venv\Scripts\python.exe visual_recovery.py --self-test`.

---

## CHUNK 3 — a permanently-blacklisted goal can no longer be bumped

**Module Objective.** Stop the autopilot bumping, and backing off against, a goal the planner already
retired. Extract the dead-goal predicate that already exists rather than writing a second one.

**Required Context/Dependencies.** Chunks 1-2. Contract C4.

**Target Files.** `autopilot.py` only.

**Strict Interfaces.**

- Add `ExploreController._goal_is_blacklisted` exactly per **C4**.
- **Refactor `_trim_resolve_resume` to call it**, replacing its inline `bl` / `perm` / `dead` block
  (anchor: `dead = g is not None and any(`) with `dead = self._goal_is_blacklisted(plan, g)`. This
  must be behaviour-identical: the existing assertions `blacklist_reroute_ok` and
  `soft_blacklist_still_restores` (anchors of those names) must still pass untouched.
- **Guard `_register_bump`.** After the existing `if self.leg_goal is None: return` and BEFORE the
  `if self._leg_is_corner:` far-corner guard, insert:
  ```python
  if self._goal_is_blacklisted(plan, self.leg_goal):
      self._missed_bump = (f"{reason} (goal {self.leg_goal} is already PERMANENTLY blacklisted "
                           f"— no pulse; the planner retired it before this contact)")
      return
  ```
  This covers all eight call sites uniformly. The `MISSED-BUMP` marker is already drained and logged
  in `run_explore` (anchor: `missed = ctrl.take_missed_bump()`), so the suppression is visible, never
  silent. **Perform NO FSM mutation here** — this method exists to stash a pulse, and its eight
  callers each own their own control flow.
- **Drop the dead commitment at the path that produced the observed event.** In the SLAM_HOLD
  settle-resume branch, anchor on:
  ```python
  if nxt == "SETTLE":
      clr = plan.get("forward_clearance_dist")
  ```
  Insert a check **before** the clearance test: when `self.leg_goal is not None` and
  `self._goal_is_blacklisted(plan, self.leg_goal)`, clear `self.leg_goal = None`, set
  `self._settle_to = "REPLAN"`, `self._enter("SETTLE", now)`, and return
  `({}, "SETTLE", <event>)` where the event names the blacklist and the waited time — the same
  convergence `_trim_resolve_resume` already uses for a goal that died mid-TRIM. Cite the
  `22:40:25.165` trace in the comment.

**Do NOT** add an autopilot-side refusal to the REPLAN goal-commit path (anchor:
`self.leg_goal = list(plan["goal"])`). `perception_worker` already guards a pick landing on an
excluded goal, and refusing here could deadlock if the planner keeps handing the same goal back.

**Acceptance Tests.** New block `SESSION-58 DEAD-GOAL BUMP GUARD` in `autopilot.py`. Reuse the
existing `_mk_plan(...)` helper's `blacklist=` / `blacklist_permanent=` kwargs (anchor:
`blacklist=None, blacklist_permanent=None):`) and the bump-latch patterns near the anchor
`cr._register_bump({"pos": anchor}, "flow WALL contact")`:

1. `perm_blacklisted_goal_emits_no_pulse` — controller with `leg_goal = [3.0, 0.0]`,
   `_bump_armed = True`; `_register_bump(plan_with_perm_blacklist_at_3_0, "clearance stand-off")`
   → `take_bump_pulse()` returns `(None, ...)` and `take_missed_bump()` returns a string containing
   `"blacklisted"`.
2. `soft_blacklisted_goal_still_bumps` — same setup with `blacklist_permanent=[False]` → a pulse IS
   emitted (mirrors `soft_blacklist_still_restores`; soft entries are retryable by design).
3. `clean_goal_still_bumps` — no blacklist arrays at all → a pulse IS emitted (regression guard: the
   guard must not fire on the ordinary path).
4. `guard_respects_radius` — a permanent blacklist point FARTHER than `goal_area_radius` from
   `leg_goal` → a pulse IS emitted.
5. `slam_hold_dead_goal_replans_instead_of_backing_off` — build a controller, `leg_goal = [3.0, 0.0]`,
   `_enter_slam_hold("SETTLE", t0, "test")`, fill `settle_fresh_frames` via `_update_slam` so the gate
   clears (follow the existing `cb24` recipe at anchor
   `cb24._enter_slam_hold("SETTLE", 0.0, "test")      # gate opens at t=0.0`), then step once with a
   plan whose `forward_clearance_dist` is BELOW `stop_clearance_dist` **and** which permanently
   blacklists `[3.0, 0.0]`. Expect state `"SETTLE"` with `_settle_to == "REPLAN"`, `leg_goal is None`,
   and **no** `"BACKOFF"`.
6. `slam_hold_live_goal_still_backs_off` — the identical setup with NO blacklist arrays → `"BACKOFF"`,
   proving the clearance stand-off path is untouched for live goals.

**Verify.** `venv\Scripts\python.exe autopilot.py --self-test`,
`venv\Scripts\python.exe frontier_planner.py --self-test`.

---

## CHUNK 4 — documentation and resume state

**Module Objective.** Leave the tree self-describing so the next session can resume cold from
`STATE.md` alone (CLAUDE.md's two mandatory closing steps).

**Required Context/Dependencies.** Chunks 1-3.

**Target Files.** `PROGRESS.md`, `STATE.md`.

**Strict Interfaces.**

- `PROGRESS.md` — add one **concise, narrative** session-58 entry in the existing house voice ("We
  wanted X. We tried Y. It failed because Z. So we tried W."). No implementation detail — that lives
  in `plans/session58-*.md`, which the entry must reference by name. It MUST record that session 57's
  zero-match fix was **verified in flight** (56/86 zero-match episodes → 0/25) before this session's
  three defects are described. Fold in the settled session-56/57 watch-list items this flight
  answered, in the same one-liner "tried that" style.
- `STATE.md` — keep it ~150-200 lines. Replace the session-57 `>>> IMMEDIATE NEXT <<<` block and its
  watch list with session 58's. The new watch list, in priority order:
  1. **No `[VISREC]` line within 12 s of its episode's `PLAN-LOST` stamp.** Per-flight match count
     should fall roughly an order of magnitude from 253.
  2. The LKG window is **absent at arm**, appears only on a matured loss, and closes on recovery.
  3. The window never shows two identical panes.
  4. `OUTPUT/diag/<ts>_visrec/` still fills — one PNG per matured episode plus every probe re-match.
     An empty directory means C5 regressed.
  5. No `BUMP pulse` or `BACKOFF` naming a goal already logged `BLACKLIST PERMANENT`; a suppressed one
     must show a `MISSED-BUMP` line instead.
  6. `SLAM_HOLD` → `SETTLE` → `REPLAN` still converges normally for goals that are NOT blacklisted.
  7. Carry forward every still-unconfirmed session-49-to-57 item that this flight did not settle.
- `STATE.md` — add **Finding 4 (bump-pulse latency)** as the top session-59 item, one short paragraph
  pointing at the design sketch in `plans/session58-lkg-window-discipline-and-dead-goal-guard.md`.
  Keep the existing "STILL OPEN: why does SLAM choke" section, and add to its untested-leads list that
  session 58 removed ~230 wasted SIFT matches per flight, so this flight is a cheap natural experiment
  on that lead (a null result proves nothing).
- Do NOT put the session-59 design sketch itself in `STATE.md` — it stays in the design doc.

**Acceptance Tests.** None (documentation only). The nine-suite gate must still pass.

**Verify.** `venv\Scripts\python.exe autopilot.py --self-test`. Then confirm by inspection that
`STATE.md` alone is sufficient to resume cold: current status, what was built, what to watch on the
next flight, and what is next.
