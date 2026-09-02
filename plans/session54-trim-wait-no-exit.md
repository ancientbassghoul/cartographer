# Session 54 — TRIM's WAIT phase has no exit under slow SLAM

## Context

Flight `20260902_155916` (session-53 live-fly, 5m28s) died parked in `TRIM`.

Session 53's own fix **worked** — the log shows the new episode clock naming both timers
(`SLAM_HOLD still waiting (this hold 7.8s / episode 15.0s) ... forcing one hop`), twice, exactly as
designed. The limit cycle is gone.

What happened instead:

```
16:03:30.073  REPLAN: SLAM_HOLD still waiting (this hold 15.0s / episode 15.0s) -> forcing one hop
16:03:30.103  ORIENT: turn +0 deg -> advance toward goal [1.9347, 1.1348]
16:03:30.321  ADVANCE: turn complete -> ADVANCE
16:03:30.356  TRIM: TRIM enter (UP): sag pos_y=-1.738 >= trim_sag_trigger_y=-1.750 -> pulse up
16:03:30.356  [CMD] state=TRIM fields={"joy_vertical": -1}     <- the pulse
16:03:30.417  [CMD] state=TRIM fields={}                       <- pulse released, 61ms later
   ... 26 more SLAM frames, all healthy, plan status OK, no further state change ...
16:04:44.294  (operator kills the run)
```

**73.9 seconds parked in `TRIM`**, holding neutral, on a flight that only flew for 5.5 minutes. The
trim itself fired correctly and did its job — it just never left the state. It also ate the
session-43 forced hop that had been granted 0.3 s earlier, so that rescue bought nothing.

### Root cause

`autopilot.py:4232-4239`, the `TRIM` handler's `WAIT` sub-phase:

```python
healthy = plan.get("plan_valid") and plan.get("pos_y") is not None and not self._slam_slow
ready = (self._trim_cmd_t0 is not None and cap_ts is not None
         and cap_ts >= self._trim_cmd_t0 + self.trim_settle_s and healthy)
```

`_slam_slow` is `_slam_ms_latest >= slam_slow_ms` (1000 ms). Over the last two minutes of this flight
SLAM solved at a flat **~2700 ms/frame**, so `_slam_slow` was `True` on every frame and `healthy`
could never become `True`. There is **no wall-clock cap on `TRIM`**, so the state had no other way
out — and it is in `_REACHED_EXCLUDED_STATES`, so the session-45 "goal already reached" retire could
not fire either.

Nor could the status dispatch rescue it: frames arrived every ~2.86 s, just inside `plan_timeout_s`
(3.0 s), so plan status stayed `OK` for all 74 seconds — never `PLAN-LOST` (→ `HOLD_LOST`), never
`PLAN-STALE` (→ `_step_stale`). **Too slow to satisfy the gate, too alive to be rescued** — verbatim
the dead band of session 50 (`SETTLE`, 91.6 s), session 44 (`TRIM_RESUME_WAIT`, ~35 s) and session
53 (`SLAM_HOLD`, 2m21s).

Two things made it reachable now:

1. **Session 52 opened the door and left the exit locked.** It removed `and not self._slam_slow` from
   TRIM's *trigger* (`autopilot.py:3255`), arguing that `pos_y` is the slowest-varying quantity SLAM
   publishes, so a stale-but-slow reading is still a trustworthy height reading. Its comment claims
   "the other six `_slam_slow` gates in this file guard MOTION/HEADING decisions and are untouched;
   only this height READING gate is removed." That claim is **wrong** — line 4235 is *also* a
   height-reading gate (its only consumer is the `TRIM done ... post pos_y=` log line). Session 52
   made TRIM enterable under slow SLAM without making it exitable.
2. **`TRIM` is the only wait of its family with no backstop.** Every sibling that reads `pos_y`
   behind a `not self._slam_slow` gate already has an explicit cap and says so in its own comment —
   `ASCEND` (`ascend_max_s`, "the cap is the backstop"), `CALIB_VERIFY` (`calib_verify_max_s`),
   `DOCK_FLOOR` (`dock_max_s`). `TRIM` has none.

## The fix (both halves — operator's call)

### 1. Drop the `_slam_slow` conjunct from TRIM WAIT's `healthy` (`autopilot.py:4235`)

```python
healthy = plan.get("plan_valid") and plan.get("pos_y") is not None
```

The real freshness guarantee is the line below it — `cap_ts >= self._trim_cmd_t0 + trim_settle_s`
proves the frame was *captured* after the pulse settled, on the same monotonic clock. `_slam_slow`
says nothing about whether `pos_y` is right, only about how long the solve took. This is session
52's own argument, applied to the exit it missed. Extend that comment block at `autopilot.py:3246`
to say the exit gate is now covered too, so the next reader isn't misled by the "other six gates"
sentence again.

Consequence to state plainly in the code comment: this flight would have left TRIM in ~3 s.

### 2. Give the WAIT phase the bounded forced exit every sibling already has

Mirror `TRIM_RESUME_WAIT`'s session-44 rescue verbatim (`autopilot.py:4262-4272`) — same knob, same
guard, same logging discipline:

```python
else:                      # not ready
    waited = now - self._trim_cmd_t0
    has_any_capture = any(cap_ts is not None for _, cap_ts in self._slam_hist)
    if waited >= self.slam_slow_hop_after_s and has_any_capture:
        y_txt = f"{plan['pos_y']:+.3f}" if plan.get("pos_y") is not None else "unavailable"
        msg = (f"TRIM done ({self._trim_dir}): FORCED after {waited:.1f}s waiting for a post-trim "
               f"frame (last pos_y={y_txt}, slam_ms={...})")
        event = self._trim_exit(now, plan, msg)
        self.note_timeout("TRIM_WAIT_FORCED", event, now, loud=False)
        return active, self.state, event
```

Reuse `slam_slow_hop_after_s` (15.0 s) rather than adding a knob — sessions 43/44/50 all reuse it for
exactly this, and a new `trim_max_s` would be a fourth name for one idea.

`has_any_capture` is load-bearing and must not be dropped: a wall clock must never paper over
perception producing *nothing* (session 43's rule). `_trim_exit` hands off to `TRIM_RESUME_WAIT`,
which has its own session-44 escape, and `_trim_resolve_resume` already degrades gracefully to
`SETTLE`→`REPLAN` when the pose is unavailable — so forcing here on a merely-slow plan is safe.

Net: exits in ~3 s normally, 15 s worst case, never infinite.

### 3. Self-tests (`autopilot.py`, `--self-test`)

The existing session-52 block (`~6455-6500`) proves TRIM *enters* under slow SLAM and never checked
that it *exits* — that is precisely the gap. Add, reusing the helpers already there (`_mk_trim52`,
`_fill_slam`, `_tplan`, `_run_trim_to_exit`):

- **(54-trim-1)** TRIM entered under sustained slow SLAM (`slam_ms=2700` on every frame, advancing
  `cap_ts`) reaches `TRIM_RESUME_WAIT` — the direct regression test for this flight.
- **(54-trim-2)** No regression under fast SLAM: the normal path still exits via the `ready` branch,
  and does **not** report itself as forced (`last_timeout` kind is not `TRIM_WAIT_FORCED`).
- **(54-trim-3)** The blackout guard holds: with `cap_ts` `None` on every frame (`_slam_hist` carries
  no capture times), TRIM does **not** force-exit even past `slam_slow_hop_after_s` — it keeps
  holding. This is the one that stops the cap from becoming a blind timer.
- **(54-trim-4)** The forced path still reaches `ORIENT` at the *preserved* goal (Trap B intact) by
  running `_run_trim_resume_wait` after the forced exit.

Per the standing rule, prove each new assertion against its own defect on a scratch copy: revert
fix 1 → (54-trim-1) alone fails; revert fix 2 → (54-trim-1) fails on the blackout variant; drop
`has_any_capture` → (54-trim-3) fails.

Then run the full suite: `python autopilot.py --self-test`, `python frontier_planner.py --self-test`,
`python visual_recovery.py --self-test`, `python flight_replay.py --self-test`.

## Backlog to record in PROGRESS.md — the rest of the class

A systematic audit of every state in `step()` and the `_step_*` helpers found **three more genuine
instances plus one latent**, all with the identical signature: *the only exit predicate is
`_slam_fast_streak >= N` or `not _slam_slow`, with no wall clock and no blackout guard.* All three
are **worse than TRIM** in one respect — they are dispatched *above* the status router and own every
status themselves, so not even a real `PLAN-LOST` can rescue them.

| State | Exit predicate | Why it can't fire | Note |
|---|---|---|---|
| `CALIB_LOST_HOLD` (`autopilot.py:1680`) | `_slam_fast_streak >= calib_lost_recover_frames` (6) + `status == "OK"` | `_slam_fast_streak` resets to 0 on every frame ≥ `slam_slow_ms`, so at 2700 ms it is pinned at 0. `calib_gate_max_s` bounds only the *comfort* sub-gate, which is downstream and never entered. | Docstring states "No time cap — the SLAM frame stream is the liveness signal (operator ask)". That premise assumes slow ⇒ eventually fast; this flight disproves it. Needs the operator's call, not a unilateral fix. |
| `CALIB_ESCAPE` (`autopilot.py:1791`) | `_slam_fast_streak >= calib_escape_ok_frames` (12) + `status == "OK"` + `_calib_slam_comfortable()` | Same, and stricter — 12 fast frames plus a latency-average bar. | Comment says "no extra bound needed here". |
| `POSTLUDE_LOST_HOLD` (`autopilot.py:1879-1881`) | `_slam_fast_streak >= required_streak` + `status == "OK"` | `postlude_recover_budget_s` (30 s) relaxes the streak 6 → 1 but **not** the speed bar; a fast frame is still required, and there are none. | Residual, not absent. Hovers at mission end, near the ground — the worst place to hang. |
| `SLAM_HOLD` legacy arm (`use_slam_stepback_on_slow=True`) | — | The sessions-43/53 rescue lives only in the `if not self.use_slam_stepback_on_slow:` branch; the legacy arm returns "keep holding" with no clock. | **Latent** — default is `False`, so off. The un-fixed twin of a known-fixed bug. |

Deliberately unbounded and **not** bugs: `HOLD_LOST` (documented hard hover), `STUCK` (terminal park),
`DONE`, `WARMUP`. Grey area: `REPLAN`'s no-goal branch is unbounded but is a *planner* liveness wait,
fail-visible via `no_goal_stall` / `NO_GOAL_IDLE` — different class.

Record this table in PROGRESS.md's backlog as one named item ("the `_slam_fast_streak` dead-band
class — three remaining sites") with the grep signature, so the next occurrence is a known item
rather than another surprise mid-flight.

## Also record: the disease behind all four

SLAM latency on this flight degrades **monotonically with map size** — mean ms per 20 frames:

```
frames    0- 19 :   489      frames  100-119 :   866
frames   20- 39 :   462      frames  120-139 :  3844
frames   40- 59 :   452      frames  140-159 :  2468
frames   60- 79 :   485      frames  160-179 :  3282
frames   80- 99 :   702      frames  180-199 :  2716
```

Flight median **1184 ms** — i.e. `_slam_slow` was true for *more than half the flight*, and
essentially all of it after ~2 minutes. This is direct support for the "MASt3R-SLAM's own workload
grows with the keyframe graph / retrieval DB" lead already recorded as the top open question in
PROGRESS.md, and it distinguishes that lead from the two already-ruled-out candidates (autopilot loop
rate, Unity focus). Every one of these dead-band bugs is a symptom of it. Worth adding these numbers
to that open-question section; **not fixed here**.

## Files touched

- `autopilot.py` — the `elif st == "TRIM":` handler (~4207-4247); the session-52 comment block at
  ~3246; new self-test cases beside the session-52 TRIM block (~6455-6500).
- `PROGRESS.md` — session-54 narrative, refreshed "Next" resume pointer, the backlog table above, the
  SLAM-degradation numbers.
- `plans/session54-trim-wait-no-exit.md` — this trace (moved out of the scratch plan file).

No `config.yaml` change: `slam_slow_hop_after_s` is reused, not added to.

## Verification

1. `python autopilot.py --self-test` — 0 failures, including the four new TRIM cases.
2. `python frontier_planner.py --self-test`, `python visual_recovery.py --self-test`,
   `python flight_replay.py --self-test` — 0 failures (untouched, but the standing rule runs them).
3. Revert-proof each new assertion on a scratch copy per the table in §3, confirming **only** the
   intended test fails, then restore byte-identical from backup.
4. **Live-fly** (`python fly.py`). Success looks like: a `TRIM enter` line followed within ~3 s by
   `TRIM done (...) -> wait for a fresh post-trim frame before resuming` **even while `slam_ms` reads
   1500-2700 ms**, then `ORIENT` at the preserved goal. If the forced path fires instead
   (`FORCED after 15.0s`), that is the backstop doing its job and is not a bug — but it means fix 1
   isn't landing, so check `cap_ts` is advancing.
5. Session 52's watch list is *still* unconfirmed (this flight was cut short again, at 5m28s):
   `VISUAL_RECOVERY` reaching a MATCH verdict, `TRIM enter (DOWN)` under slow SLAM, no evidence-free
   `BACK-OFF SUPPRESSED`, no goal inside a permanently blacklisted region. Carry it forward.
