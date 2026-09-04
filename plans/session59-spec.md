# SESSION 59 — Sonnet-Ready Implementation Specification
# Retire the dead sweep corner · camera-triggered back-off

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
4. **Anchors are source strings, not line numbers.** `autopilot.py` is ~10.8k lines and a bare `Read`
   truncates it — use `Grep` to locate each anchor string, then `Edit`.
5. **NO SILENT FALLBACKS** (`CLAUDE.md`). Never swallow an error into a default. A degraded or
   suppressed path must set an explicit visible state flag, emit a log line, and be counted. Prefer
   raising over absorbing.
6. **IMAGE INTEGRITY** (`CLAUDE.md`). Never resize, crop, downscale or re-encode a frame.
7. **NO MANUAL-FLIGHT DATA LEAKAGE** (`CLAUDE.md`). Every new constant here is a general ratio,
   duration or count. Do not introduce any value derived from a specific flight or room.
8. **Comment in the surrounding style.** This codebase carries dense "why", not "what", comments,
   each tagged with its session number. Tag new blocks `Session 59:` and say *why*, citing the
   evidence in `MISSION CONTEXT`. Match the local density; no banner comments.
9. **Do not commit, stage, stash, or otherwise mutate git state.**
10. **The gate runs nine suites under the project venv after every chunk** — `autopilot.py`,
    `frontier_planner.py`, `visual_recovery.py`, `flight_replay.py`, `ground_grid.py`,
    `map_store.py`, `salvage_flight.py`, `perception_worker.py`, `visualizer.py`. All nine are green
    at the start of this session. Breaking any one halts the run, not just the module you edited.
11. **Finish by running the self-test commands the chunk names**, and report the full PASS/FAIL list
    verbatim. A chunk that leaves any suite failing is not complete.
12. **Every chunk must change at least one file.** An empty diff is treated as a failed chunk.

---

## MISSION CONTEXT (why this work exists)

Diagnosed off flight `OUTPUT/diag/20260903_234939_autopilot.log` (47 min) — the first live fly of
session 58. Session 58's fixes **verified**: `[VISREC]` inside the loss grace fell from 233/253 (92 %)
to 45/1454 (3 %), and the dead-goal drop fires correctly. But the flight ended with the drone ramming
a glass wall at bounding-box corner `[-1.5, -3.9]` for **23.3 minutes**, and session 58 turned out to
have closed the last escape hatch on a pre-existing trap.

### Finding 1 — a committed sweep corner is never re-checked, so the tour can never advance

1. `00:11:06` — the sweep tour commits to the corner. `sweeping = True`, `sweep_target` cached.
2. `00:13:01` — the LOOP guard permanently blacklists that same disc
   (`PLANNER: LOOP-BLACKLIST goal=[-1.5, -3.9] picks=3`).
3. `select()` returns the cached target **unconditionally, forever** — no `_excluded_permanent`
   re-check. Session 52 built exactly that guard, but it lives in `_pick_sweep_corner`, which is only
   reached when `sweeping` is **False**. It never ran once: the log has zero `CORNER-SKIP` lines and
   **29** × `WARNING: pick landed on an ALREADY-excluded goal=[-1.5, -3.9] -> blacklist bypassed
   (sweep-tour CORNER target)`.
4. Only three things clear a `sweep_target`: physically reaching it (impossible — behind glass),
   `note_wall_hit`'s 2-bump, or `force_retire_corner` (reachable only via the FAR-corner give-up path;
   the drone was close, so it was skipped).
5. **Session 58 blocked the 2-bump.** `_register_bump`'s new guard returns early when the goal is
   already permanently blacklisted, so no pulse is published, `note_wall_hit` never runs, and
   `_mark_corner_visited` never fires.

The result is a 51-millisecond loop, repeated for 23 minutes — session 58's drop and the planner's
re-emit fighting each other:

```
00:33:00.142  SETTLE: leg_goal [-1.5,-3.9] was already PERMANENTLY blacklisted -> settle -> replan
00:33:00.175  REPLAN
00:33:00.193  ORIENT: advance toward goal [-1.5, -3.9]
```

45 legs (31 advance + 14 parallax) committed to a goal the planner had already condemned.

### Finding 2 — the `closer` verdict can veto a back-off but never request one

496 `closer=LIVE` verdicts across the flight; **zero** drove any action. Every back-off that fired
cited `closer=UNKNOWN`. `_step_lost_recovery` returns `None` unless the **cached map clearance**
already proposes a back-off (`pending`); the camera only adjudicates that proposal. On glass the map
reads clear forever, so `pending` is never true and the verdict is unreachable — exactly backwards,
since the camera is the only sensor that sees glass at all (this flight logged **zero** flow-WALL
fires against three confirmed glass rams). 555 of 1454 matches were computed in `FALLBACK`, which
returns before `_step_lost_recovery` is ever reached.

The operator's rule for fixing this, verbatim: *"I perform a backoff manuever, only if the VR window
episode is alive for more than 3 (configurable) seconds, and the percentage of 'live - closer' calls
over 'equal' (I ignore 'unknown's) is higher than 66% (configurable)."*

`LKG` is placed in the denominator alongside `EQUAL` (operator-reviewed): `LKG` is positive evidence
we are **farther** than F_LKG, so excluding it would let alternating LIVE/LKG frames read as 100 %
LIVE and back off against the camera's own contrary evidence.

### Context — the platform underneath

`HOLD_LOST` 1348 s + `FALLBACK` 471 s = **30 of the 47 minutes blind**. Median `slam_ms` degrades 791
→ 12 682 ms over the first 20 minutes, worst single solve **71 774 ms**, worst wait between solved
frames **72.9 s** — then *partially recovers* (min 25-30 back to 1783 ms; min 40-45 to 1816 ms).
Nothing in this session addresses that; it is recorded in chunk 5 as the dominant open problem.

### Deferred, do NOT build here

- **Staleness UI** — `TOPIC_CONTROL` carries no plan status/age, so the visualizer draws its last plan
  as `PLAN valid` while the autopilot is blind. Operator wants to discuss it first.
- **Bump-pulse latency** — a bump takes 10-18 s to become a blacklist (worse as SLAM slows).

---

## SHARED CONTRACTS

Read this section before every chunk.

### C1 — `FrontierPlanner._retire_corner_unseen` (new private method, `frontier_planner.py`)

```python
def _retire_corner_unseen(self, c, tag: str) -> None:
```

Retires corner `c` without the drone ever reaching it. Returns `None`. Behaviour — **exactly** what
`_pick_sweep_corner`'s existing permanent-dead block already does, extracted verbatim:

```python
e = self._db_entry([float(c[0]), float(c[1])])
e["corner_giveups"] += 1
e["is_corner"] = True
self._mark_corner_visited(c)
self.last_select_events.append(
    f"{tag} goal=[{float(c[0]):.3f}, {float(c[1]):.3f}] inside a PERMANENT dead "
    f"zone -> force-retired, tour advances")
```

**CRITICAL — this method MUST NOT set `self._gave_up_corner`.** It is tempting to mirror
`force_retire_corner`, which does set it, but that flag means "we ABANDONED a corner we never got
close to" (the far-corner give-up cap). A corner inside a *proven permanent dead zone* is a
**confirmed-unreachable** retirement, not a give-up. Session 52 made this call deliberately and locked
it — `frontier_planner.py` already asserts
`check("(52-corner-4) _gave_up_corner is NOT set by a permanent-dead skip", p._gave_up_corner is False)`
— so setting it breaks a passing test and halts this chunk. Worse, the flag propagates
`perception_worker.py` (`payload["corner_giveup_stuck"] = bool(self.planner._gave_up_corner)`) →
`autopilot.py`'s REPLAN done branch, which routes the mission into terminal **STUCK** instead of
**RETURN_TO_ORIGIN**, ending the flight in a dead hold rather than the graceful postlude.

`tag` is the event prefix: `"CORNER-SKIP"` at the selection-time site (preserving the existing string
that three self-tests match on) and `"CORNER-RETIRE-EN-ROUTE"` at the new en-route site.

### C2 — `ExploreController` new instance state (`autopilot.py`)

Declared in `__init__` beside the existing session-58 flag `self.visrec_window_open`:

```python
self._dead_goal_dropped = None       # list[float] | None -- [x,z] the last dead-goal drop discarded
self._dead_goal_recommits = 0        # consecutive REPLAN commits landing back inside that region
self._dead_goal_notice_fired = False # so the CRITICAL notice prints exactly ONCE per flight
self._vis_tally = VisualDirectionTally()   # session 59, see C4
```

`_dead_goal_dropped` / `_dead_goal_recommits` / `_dead_goal_notice_fired` are also cleared by
`reset_leg()` (autonomy pause), beside the existing `self._blind_contact_reacts = 0`.

### C3 — `ExploreController` new config attributes (`autopilot.py.__init__`)

Read from `e` (the `autonomy.explore` dict) immediately after the existing
`self.visrec_lkg_ageout_log_interval_s` assignment:

```python
self.use_visual_backoff_trigger = bool(e.get("use_visual_backoff_trigger", True))
self.visual_backoff_min_window_s = float(e.get("visual_backoff_min_window_s", 3.0))
self.visual_backoff_live_ratio = float(e.get("visual_backoff_live_ratio", 0.66))
self.visual_backoff_min_samples = int(e.get("visual_backoff_min_samples", 5))
self.dead_goal_recommit_notice_after = int(e.get("dead_goal_recommit_notice_after", 3))
```

### C4 — `VisualDirectionTally` (new module-level PLAIN class, `autopilot.py`)

Placed immediately before `class ExploreController`. Pure bookkeeping — no I/O, no clock of its own,
no config. `now` is always passed in.

**Use a plain class with an explicit `__init__`, NOT a dataclass.** `autopilot.py` does not import
`dataclasses` (its imports are `argparse, collections, copy, json, math, os, random, time`) and
defines only plain classes; `@dataclass` would require a new import for no benefit. Do not add one.

```python
class VisualDirectionTally:
    """Rolling per-episode tally of VisualMatch.closer verdicts (session 59)."""

    def __init__(self):
        self.t0 = None       # float | None -- monotonic time of the first COUNTED verdict
        self.live = 0        # int
        self.equal = 0       # int
        self.lkg = 0         # int

    def reset(self) -> None: ...
    def add(self, verdict, now) -> None: ...        # verdict: str | None, now: float
    @property
    def samples(self) -> int: ...
    @property
    def ratio(self): ...                            # float | None
    def window_s(self, now) -> float: ...
    def summary(self) -> str: ...
```

Exact behaviour:

| member | contract |
|---|---|
| `reset()` | sets `t0=None`, `live=equal=lkg=0` |
| `add(verdict, now)` | `verdict` in `{"LIVE","EQUAL","LKG"}` increments that counter and, **if `t0 is None`, stamps `t0 = now`**. Any other value — `"UNKNOWN"`, `None`, unknown strings — is **ignored entirely**: no counter moves and `t0` is NOT stamped. |
| `samples` | `self.live + self.equal + self.lkg` |
| `ratio` | `self.live / self.samples`, or `None` when `samples == 0` |
| `window_s(now)` | `0.0` when `t0 is None`, else `now - self.t0` |
| `summary()` | `f"LIVE={self.live} EQUAL={self.equal} LKG={self.lkg} samples={self.samples} ratio={r}"` where `r` is `f"{self.ratio:.2f}"` or `"n/a"` |

### C5 — `ExploreController._visual_backoff_due` (new method, `autopilot.py`)

```python
def _visual_backoff_due(self, now) -> bool:
```

Returns `True` iff **all** hold, in this order (short-circuit on the first failure):

1. `self.use_visual_backoff_trigger` is True
2. `self._vis_tally.t0 is not None`
3. `self._vis_tally.window_s(now) >= self.visual_backoff_min_window_s`
4. `self._vis_tally.samples >= self.visual_backoff_min_samples`
5. `self._vis_tally.ratio >= self.visual_backoff_live_ratio`

Pure predicate — reads state only, mutates nothing.

### C6 — `ExploreController._fire_visual_backoff` (new method, `autopilot.py`)

```python
def _fire_visual_backoff(self, now, from_state: str) -> tuple[dict, str, str]:
```

Returns the standard `(fields, state, event)` triple for the caller to return directly. Behaviour:

```python
tally = self._vis_tally.summary()
window = self._vis_tally.window_s(now)
self._vis_tally.reset()
self._loss_episode_t0 = now          # a physical action restarts the wait (session 57 convention)
self._loss_grace_noticed = False
self._lost_hold_noticed = False
self._player = None
self._backoff_t0 = now
self._enter("BACKOFF", now)
return {}, "BACKOFF", (f"visual back-off: {tally} over {window:.1f}s "
                       f"(>= {self.visual_backoff_min_window_s:.1f}s, ratio >= "
                       f"{self.visual_backoff_live_ratio:.2f}) from {from_state} -> standoff back off")
```

**CRITICAL — this MUST NOT touch `self._blind_contact_reacts`, and MUST NOT call
`_arm_loss_backoff`.** `_arm_loss_backoff` increments that wedge counter and escalates to `FALLBACK`
at `blind_contact_escalate_after`; routing the visual trigger through it would let a back-off fired
*from* FALLBACK re-escalate *into* FALLBACK. Entering `BACKOFF` directly, plus the `reset()` above
(which forces a whole fresh confident-evidence window before the next fire), is what bounds it.

### C7 — new `config.yaml` keys

Added under `autonomy.explore`, immediately after the existing `visrec_lkg_ageout_log_interval_s`
entry, each with a comment in the surrounding style stating it is a general parameter, not a room
answer:

```yaml
    use_visual_backoff_trigger: true   # session 59: let the CAMERA request a back-off, not merely veto one
    visual_backoff_min_window_s: 3.0   # confident-evidence window that must elapse before it may fire
    visual_backoff_live_ratio: 0.66    # LIVE / (LIVE + EQUAL + LKG) needed to call it "closing"
    visual_backoff_min_samples: 5      # confident verdicts required (UNKNOWN never counts)
    dead_goal_recommit_notice_after: 3 # consecutive drop -> re-commit cycles before a CRITICAL notice
```

### C8 — self-test conventions

- `frontier_planner.py`: a local `check(name, cond)` closure inside `run_self_test()` printing
  `[frontier_planner][self-test] PASS|FAIL  <name>`, folded into `ok`. Name new cases
  `(59-corner-N) <description>`.
- `autopilot.py`: `ok = ok and <case>_ok` accumulators with one
  `print(f"[self-test] {'PASS' if X else 'FAIL'}  ...")` per case. Name new blocks
  `SESSION-59 <TOPIC>`.

---

## CHUNK 1 — retire a condemned sweep corner en route

**Module Objective.** Make `select()` re-check its already-committed `sweep_target` against the
permanent blacklist, retire it if dead, and advance the tour on the same tick. This is the root-cause
fix for the 23-minute lockup.

**Required Context/Dependencies.** None (first chunk). Contracts **C1**, **C8**.

**Target Files.** `frontier_planner.py` only.

**Strict Interfaces.**

1. Add `_retire_corner_unseen(self, c, tag)` exactly per **C1**, placed immediately after
   `_mark_corner_visited`. Re-read C1's CRITICAL paragraph before writing it.
2. Refactor `_pick_sweep_corner` to call it. Anchor on the source string
   `if self._excluded_permanent(c):` and replace that block's five statements with
   `self._retire_corner_unseen(c, "CORNER-SKIP")`, keeping the surrounding `for` / `continue`
   structure untouched. This must be behaviour-identical — three existing tests match on the exact
   string `"CORNER-SKIP"`.
3. Add the en-route re-check in `select()`. Anchor on the source string
   `if self.sweeping and self.sweep_target is not None:` and restructure that block to:
   ```python
   if self.sweeping and self.sweep_target is not None:
       if self._excluded_permanent(self.sweep_target):
           # Session 59: <why -- cite Finding 1>
           self._retire_corner_unseen(self.sweep_target, "CORNER-RETIRE-EN-ROUTE")
           self.sweeping = False
           self.sweep_target = None
           # fall through to _pick_sweep_corner below -- the tour advances THIS tick
       elif self._d(pos, self.sweep_target) > self.goal_reach_dist:
           return list(self.sweep_target), len(frontiers), False
       else:
           <the existing reached-branch body, moved here VERBATIM and unchanged>
   ```
   The dead branch must **fall out of the if/elif/else**, not return, so control reaches the existing
   `nxt = self._pick_sweep_corner(corners, pos)` below. Do not reorder or edit the reached-branch body.

**Do NOT** touch `force_retire_corner`, `_excluded`, `_excluded_permanent`, `_blacklist_goal`, or any
blacklist-writing mechanism. Do NOT set `_gave_up_corner` anywhere.

**Acceptance Tests.** New block in `run_self_test()`, after the existing `(52-corner-*)` cases:

1. `(59-corner-1)` — a corner committed while ALIVE, then permanently blacklisted mid-tour, is
   retired on the next `select()`: build a planner, `select(..., sweep_corners=corners)` to commit a
   target, assert `p.sweeping is True`; then `p._blacklist_goal(p.sweep_target, permanent=True)`;
   then `select()` again from a pos far from every corner. Assert the returned goal is **not** the
   retired corner, `p._corner_visited(<retired>)` is True, and
   `any("CORNER-RETIRE-EN-ROUTE" in s for s in p.last_select_events)`.
2. `(59-corner-2)` — **`p._gave_up_corner is False`** after that en-route retirement. This is the
   direct analogue of `(52-corner-4)` and the guard against the STUCK-instead-of-RETURN_TO_ORIGIN
   ending.
3. `(59-corner-3)` — a **soft** blacklist on the committed target does NOT retire it: same setup with
   `permanent=False`, assert the returned goal is still that corner and no
   `"CORNER-RETIRE-EN-ROUTE"` event was appended.
4. `(59-corner-4)` — the tour ADVANCES: with two unvisited corners and the committed one blacklisted
   permanently, the goal returned by the same `select()` call is the OTHER corner (not `None`, not
   the dead one).
5. `(59-corner-5)` — all corners permanently dead while committed to one: `select()` retires it and
   still reports `done` via the existing path, without looping.

**Regression requirement.** Every existing `(52-corner-*)` assertion must still pass **unmodified**.

**Verify.** `venv\Scripts\python.exe frontier_planner.py --self-test` — 0 failures.

---

## CHUNK 2 — let a corner still bump, and make a planner/autopilot disagreement loud

**Module Objective.** Undo the half of session 58's guard that removed the corner's last escape, and
make the drop→re-commit loop impossible to miss if the two components ever disagree again.

**Required Context/Dependencies.** Chunk 1. Contracts **C2**, **C3** (the
`dead_goal_recommit_notice_after` key only), **C7** (that key only), **C8**.

**Target Files.** `autopilot.py`, `config.yaml`.

**Strict Interfaces.**

1. **Narrow the session-58 bump guard.** In `_register_bump`, anchor on the source string
   `if self._goal_is_blacklisted(plan, self.leg_goal):` and change the condition to
   `if self._goal_is_blacklisted(plan, self.leg_goal) and not self._leg_is_corner:`. Update the
   comment: session 59 found this guard also suppressed the pulse that reaches `note_wall_hit`'s
   corner-retirement branch — the 2-bump was a sweep corner's last escape, and blocking it cost 23
   minutes. Frontier goals keep the session-58 behaviour exactly.
2. **Record the dropped goal.** In the SLAM_HOLD settle-resume dead-goal branch, anchor on
   `dead_goal = self.leg_goal`, and after `self.leg_goal = None` add
   `self._dead_goal_dropped = list(dead_goal)`.
3. **Detect the re-commit.** At the REPLAN goal-commit site, anchor on the source string
   `self.leg_goal = list(plan["goal"])`, and immediately AFTER it add: if `self._dead_goal_dropped`
   is not None and `self._dist(self.leg_goal, self._dead_goal_dropped) <= self.goal_area_radius`,
   increment `self._dead_goal_recommits`; otherwise set `self._dead_goal_recommits = 0` and
   `self._dead_goal_dropped = None`. When the counter reaches
   `self.dead_goal_recommit_notice_after` and `not self._dead_goal_notice_fired`, set that flag True
   and call `self.note_timeout("DEAD_GOAL_RECOMMIT", <text>, now)` — the text must name the goal, the
   count, and state plainly that the planner is re-emitting a goal the autopilot just rejected as
   permanently blacklisted.
4. Add the config attribute per **C3** and the `config.yaml` key per **C7** (this one key only).
5. Add the three `_dead_goal_*` fields per **C2** to `__init__` and to `reset_leg()`.

**Do NOT** add any local suppression of the goal itself — the planner owns goal selection, and chunk
1 is the fix. This chunk only restores the escape and makes the disagreement visible.

**Acceptance Tests.** New block `SESSION-59 DEAD-GOAL DISAGREEMENT` in `autopilot.py`:

1. `corner_still_bumps` — controller with `leg_goal` set, `_bump_armed = True`, `_leg_is_corner = True`,
   `_register_bump(<plan permanently blacklisting that goal>)` → `take_bump_pulse()` returns the goal
   (a pulse WAS emitted).
2. `frontier_still_suppressed` — identical but `_leg_is_corner = False` → no pulse, and
   `take_missed_bump()` contains `"blacklisted"` (session 58 regression guard).
3. `recommit_counts_and_notices` — set `_dead_goal_dropped`, then drive
   `dead_goal_recommit_notice_after` REPLAN commits onto the same point; assert the notice fires
   exactly once (a further commit does not re-fire) and names the goal.
4. `recommit_resets_on_different_goal` — a commit outside `goal_area_radius` zeroes
   `_dead_goal_recommits` and clears `_dead_goal_dropped`.
5. `reset_leg_clears_state` — after `reset_leg()`, all three `_dead_goal_*` fields are back to their
   initial values.

**Verify.** `venv\Scripts\python.exe autopilot.py --self-test`,
`venv\Scripts\python.exe frontier_planner.py --self-test`.

---

## CHUNK 3 — the visual direction tally (pure measurement, no wiring)

**Module Objective.** Add the hysteresis accumulator and its predicate. Nothing consumes them yet —
this chunk must not change any flight behaviour.

**Required Context/Dependencies.** Chunks 1-2. Contracts **C3**, **C4**, **C5**, **C7**, **C8**.

**Target Files.** `autopilot.py`, `config.yaml`.

**Strict Interfaces.**

1. Add the `VisualDirectionTally` plain class exactly per **C4**, immediately before
   `class ExploreController`. Do **not** add a `dataclasses` import — see C4.
2. Add `self._vis_tally = VisualDirectionTally()` per **C2**, and clear it in `reset_leg()`
   (`self._vis_tally.reset()`).
3. Add the four `visual_backoff*` config attributes per **C3** and the four `config.yaml` keys per
   **C7**.
4. Add `_visual_backoff_due(self, now)` exactly per **C5**, placed immediately after
   `wants_visual_match`.

**Do NOT** call `add()`, `_visual_backoff_due` or `_fire_visual_backoff` from anywhere in this chunk.
No FSM edits. A behavioural diff on this chunk is a failure.

**Acceptance Tests.** New block `SESSION-59 VISUAL TALLY` in `autopilot.py`:

1. `unknown_is_ignored` — `add("UNKNOWN", 100.0)` and `add(None, 100.0)` on a fresh tally leave
   `samples == 0`, `ratio is None`, `t0 is None`, `window_s(200.0) == 0.0`.
2. `t0_stamps_on_first_counted` — after `add("EQUAL", 50.0)`, `t0 == 50.0`; a later `add` does not
   move it.
3. `ratio_math` — 7×`LIVE`, 2×`EQUAL`, 1×`LKG` → `samples == 10` and `ratio == 0.7`.
4. `due_all_conditions` — with defaults (3.0 s / 0.66 / 5 samples): a tally of 8 LIVE + 2 EQUAL
   stamped at `t0` is due at `t0 + 3.1` and **not** due at `t0 + 2.9`.
5. `due_needs_ratio` — 5 LIVE + 5 EQUAL (ratio 0.50) at `t0 + 10.0` is **not** due.
6. `due_needs_samples` — 3 LIVE + 0 others at `t0 + 10.0` is **not** due (under `min_samples`).
7. `due_respects_flag` — the same due tally with `use_visual_backoff_trigger = False` is **not** due.
8. `defaults_from_config` — a controller built from the repo's own `config.yaml` reads
   `use_visual_backoff_trigger is True`, `3.0`, `0.66`, `5`.
9. `reset_clears` — `reset()` returns every field to its initial value.

**Verify.** `venv\Scripts\python.exe autopilot.py --self-test`.

---

## CHUNK 4 — wire the trigger into HOLD_LOST and FALLBACK

**Module Objective.** Feed the tally, and let it fire a back-off from the two states where the
verdict is currently computed and discarded. This is the only chunk that changes flight behaviour.

**Required Context/Dependencies.** Chunk 3 (`VisualDirectionTally`, `_visual_backoff_due`, the config
attributes). Contract **C6**.

**Target Files.** `autopilot.py`.

**Strict Interfaces.**

1. Add `_fire_visual_backoff(self, now, from_state)` exactly per **C6**, immediately after
   `_visual_backoff_due`. Re-read C6's CRITICAL paragraph before writing it.
2. **Reset on a fresh loss edge.** Anchor on the **unique** source string
   `self._loss_episode_t0 = now          # session 48: the loss-recovery grace window starts HERE`
   (inside the `if lost and not self._was_lost:` block) and add `self._vis_tally.reset()` beside it.
   Do NOT anchor on `self._loss_snapshot_checked = False` — that string occurs three times in the
   file (`__init__`, a docstring, and this block).
3. **Feed the tally every lost tick.** Immediately after that `if lost and not self._was_lost:` block
   and its companion `if not lost:` block, add: while `lost`, call
   `self._vis_tally.add(visual_match.closer if visual_match is not None else None, now)`. `add()`
   already ignores `None` and `"UNKNOWN"`, so no extra guard is needed.
4. **Fire from FALLBACK.** Anchor on the source string `if st == "FALLBACK":` **inside the
   `if status in ("PLAN-LOST", "NO-PLAN"):` branch** (there are two matches for this string in the
   file — the other is in a TRIM-trigger context near the top of `step()`; use surrounding context to
   pick the right one). Immediately BEFORE the existing
   `return self._step_fallback_sweep(...)`, add:
   ```python
   if self._visual_backoff_due(now):
       return self._fire_visual_backoff(now, "FALLBACK")
   ```
5. **Fire from HOLD_LOST.** In the same PLAN-LOST branch, in the `st == "HOLD_LOST"` path, add the
   same two lines with `from_state="HOLD_LOST"` immediately BEFORE the existing
   `deferred = self._step_lost_recovery(plan, now, visual_match, status)` call, so a confident visual
   verdict is acted on before the map-clearance path gets its turn.

**Do NOT** touch `_step_lost_recovery`, `_maybe_loss_snapshot_backoff`, `_arm_loss_backoff`,
`_blind_contact_reacts`, `blind_contact_escalate_after`, or `_step_fallback_sweep`'s body. Do NOT
change `wants_visual_match` or `_visrec_should_match` — the tally is fed from whatever verdicts those
already produce.

**Acceptance Tests.** New block `SESSION-59 VISUAL BACKOFF WIRING` in `autopilot.py`. Use the
existing `_drive`/`_mk_plan` helpers where they fit; otherwise drive `step()` directly:

1. `fires_from_hold_lost` — a controller parked in `HOLD_LOST` under `status="PLAN-LOST"`, stepped
   with `visual_match.closer == "LIVE"` for longer than `visual_backoff_min_window_s`, transitions to
   `"BACKOFF"` and the event text contains `"visual back-off"`.
2. `fires_from_fallback` — same, parked in `FALLBACK`, reaches `"BACKOFF"`.
3. `no_fire_on_equal` — the identical drive with `closer == "EQUAL"` throughout never leaves the hold
   state.
4. `no_fire_on_unknown` — the identical drive with `closer == "UNKNOWN"` throughout never fires and
   leaves `_vis_tally.samples == 0`.
5. `wedge_counter_untouched` — capture `_blind_contact_reacts` before a visual-triggered back-off and
   assert it is unchanged after. **This is the BACKOFF↔FALLBACK loop guard.**
6. `does_not_refire_immediately` — after firing, the tally is empty, so a single further `LIVE` tick
   cannot fire again (`_visual_backoff_due` is False).
7. `flag_off_is_inert` — the full `fires_from_hold_lost` drive with
   `use_visual_backoff_trigger = False` never leaves the hold state.
8. `loss_edge_resets_tally` — a tally filled during one episode is empty on the first tick of the
   next fresh loss edge.

**Verify.** `venv\Scripts\python.exe autopilot.py --self-test`. Confirm no previously-passing block
regressed — the PLAN-LOST branch is exercised by many older blocks.

---

## CHUNK 5 — documentation, resume state, and archive this spec

**Module Objective.** Leave the tree self-describing so the next session can resume cold from
`STATE.md` alone (CLAUDE.md's two mandatory closing steps), and keep this spec in the repo.

**Required Context/Dependencies.** Chunks 1-4.

**Target Files.** `PROGRESS.md`, `STATE.md`, `plans/session59-spec.md` (new).

**Strict Interfaces.**

1. **Archive this spec.** Copy `C:\Users\owner\.claude\plans\valiant-waddling-spark.md` verbatim to
   `plans/session59-spec.md`. Do not edit the content while copying.
2. **`PROGRESS.md`** — add one concise, narrative session-59 entry in the existing house voice ("We
   wanted X. We tried Y. It failed because Z. So we tried W."). No implementation detail. It MUST
   record: session 58's grace fix **verified in flight** (233/253 → 45/1454 wasted matches); that
   session 58's bump guard **closed a sweep corner's last escape**, costing 23.3 minutes and 45 legs;
   and that the camera can now request a back-off, not merely veto one. Reference
   `plans/session59-spec.md`.
3. **`STATE.md`** — keep it ~150-200 lines. Replace the session-58 `>>> IMMEDIATE NEXT <<<` block and
   watch list with session 59's:
   1. **The corner tour advances.** Expect a `CORNER-RETIRE-EN-ROUTE` line and the next corner
      committed within one plan publish. Expect **zero** `WARNING: pick landed on an
      ALREADY-excluded goal` lines (29 last flight).
   2. **No drop→re-commit loop** — no `already PERMANENTLY blacklisted` line followed within a second
      by an `ORIENT` toward that same goal. If they still disagree, chunk 2's `DEAD_GOAL_RECOMMIT`
      notice must say so.
   3. **The camera fires at least once** — back-off lines citing a `LIVE=/EQUAL=/LKG=` tally rather
      than only `closer=UNKNOWN`. None appearing is information, not necessarily a bug: check whether
      a 0.66 ratio was ever actually reached.
   4. **No BACKOFF↔FALLBACK oscillation.** Kill switch if it misbehaves in the air:
      `use_visual_backoff_trigger: false` — config only, no code change, no re-run of this spec.
   5. Regression: session 58's grace fix holds — `[VISREC]` inside a loss grace stays near zero.
   6. The flight ends by **manual stop**, as every flight has; no bounded-survey mechanism exists.
4. **`STATE.md` — name the SLAM choke the dominant open problem**, at the top, with this flight's
   measured profile:

   | flight min | frames | median | p90 | max |
   |---|---|---|---|---|
   | 0–5 | 151 | 791 ms | 2 829 | 13 970 |
   | 5–10 | 72 | 1 404 | 9 960 | 16 479 |
   | 10–15 | 56 | 2 436 | 10 860 | 37 197 |
   | 15–20 | 22 | 12 682 | 22 133 | 58 061 |
   | 20–25 | 16 | 5 056 | 41 969 | **71 774** |
   | 25–30 | 86 | 1 783 | 4 402 | 37 209 |
   | 40–45 | 78 | 1 816 | 3 012 | 31 399 |

   Worst wait between two solved frames: **72.9 s** at flight-minute 24.6. Record the nuance
   explicitly — the median degrades 16× over the first 20 minutes but then **partially recovers**,
   which is evidence *against* the "keyframe graph grows monotonically" theory on the untested-leads
   list. Also note session 58 removed ~230 wasted SIFT matches per flight and the choke persisted,
   which weakens the LKG-window lead.
5. **`STATE.md` — carry forward the two deferred items** as the session-60 candidates: the
   **staleness UI** (operator wants to discuss it first — `TOPIC_CONTROL` carries no plan status/age,
   so the panel reads `PLAN valid` while blind) and the **bump-pulse latency** (10-18 s, and it scales
   with SLAM latency). Also record the operator's standing question of whether goal management should
   be **rebuilt from scratch** against a written behaviour spec, with the decision rule: if this
   flight is clean, ship the Blender/PLY presentation work; if goal problems recur, rewrite.

**Acceptance Tests.** None (documentation only). The nine-suite gate must still pass.

**Verify.** `venv\Scripts\python.exe autopilot.py --self-test`. Then confirm by inspection that
`STATE.md` alone is enough to resume cold, and that `plans/session59-spec.md` exists and matches this
file.
