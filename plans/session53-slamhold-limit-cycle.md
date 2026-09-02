# Session 53 — the SLAM_HOLD ↔ HOLD_LOST limit cycle: a rescue clock that can never accumulate

## Context

Flight `20260902_143207` (the session-52 live-fly) ended in a **2 minute 21 second hover loop** the
drone never escaped on its own — the operator killed the run. From `14:34:55.739` to the end of the
log the FSM oscillated `SLAM_HOLD` → `HOLD_LOST` → `SLAM_HOLD` **43 times** at a dead-steady ~3.3 s
period, emitting nothing but the same two lines. No rescue fired, no escalation, no back-off, no
probe. The drone hovered in place and burned the flight.

The escape hatch for exactly this situation already exists — session 43's forced hop
(`slam_slow_hop_after_s`) — and it **fired correctly once**, at `14:34:32.547`, producing a real
maneuver (`ORIENT` +30° → `PARALLAX_PUSH` → `SETTLE`, ~3 s of genuine motion). Then it never fired
again for the remaining 2m21s. This plan restores it.

## What the log says

| measurement (over the 43-cycle loop) | value |
|---|---|
| SLAM solve time | median **3155 ms** (min 947, max 3556) — 3.2× over `slam_slow_ms` = 1000 |
| frames fast enough to count toward the settle gate | **1 of 44**, and it is the last line of the log |
| SLAM inter-frame arrival | median **3297 ms** |
| `OK` → `PLAN-LOST` dwell | median **3.02 s** (min 2.99, max 3.03) — pinned to `plan_timeout_s` = 3.0 |
| `SLAM_HOLD` entries | 46 · `HOLD_LOST` 45 · **`PLAN-STALE` 0** · `NO-PLAN` 0 |
| forced hops (`SLAM_HOLD_FORCED_HOP`) | **1**, all flight |

**Three independent axes, and only one is the pathology.** `slam_ms` (throughput) is the disease:
median 3155 ms. Plan status is a *derived consequence* — a 3155 ms solve exceeds `plan_timeout_s`,
so the plan ages out between frames and the status flip-flops. `PLAN-STALE` occurred **zero times
all flight**: `plan_valid` was true on every frame, so **SLAM never lost tracking** — it tracked
perfectly, just slowly. This is not a tracking loss and not a relocalization problem.

## Root cause

A **rescue timer reset by the very oscillation it exists to bound.**

One revolution of the cycle:

1. A SLAM frame lands → `plan status: OK`. The drone is in `HOLD_LOST`, a `_RECOVERY_STATE`, so the
   step() top (`autopilot.py:3030`) calls `_enter_slam_hold("SETTLE", now, …)`.
2. `_enter_slam_hold` (`autopilot.py:2301`) does **`self._slam_hold_start = now`** — unconditionally,
   on every entry.
3. Inside `SLAM_HOLD` the settle gate needs `settle_fresh_frames` (6) consecutive frames under
   `slam_slow_ms` (1000 ms). At a median 3155 ms **not one frame qualifies** — arithmetically
   unreachable, the same shape as session 50's SETTLE dead band.
4. So it falls through to the session-43 rescue (`autopilot.py:3123`):
   `waited = now - self._slam_hold_start`, fire when `waited >= slam_slow_hop_after_s` (**15.0 s**).
5. **3.02 s later** the plan ages past `plan_timeout_s` (3.0 s) → `PLAN-LOST` → `_enter("HOLD_LOST")`.
6. ~0.2 s later the next frame lands → back to step 1, and `_slam_hold_start` is **re-stamped**.

`waited` therefore tops out at ~3.0 s against a 15.0 s bar. **The rescue is not too slow — its clock
is structurally incapable of ever reaching the threshold.** SLAM is too slow to hold a plan valid for
15 continuous seconds, but fast enough to keep re-locking, so no rescue can accumulate: a dead band
between `plan_timeout_s` and `slam_slow_hop_after_s`.

The one hop that *did* fire at `14:34:32` was luck — it followed a stretch where SLAM ran 550–1400 ms,
so a single `SLAM_HOLD` happened to survive 15 s uninterrupted.

### This lesson is already written in this file, and was applied to the wrong field

`autopilot.py:926-933`, the comment block for `_slam_stepback_count`:

> PERSISTS across a PLAN-LOST/HOLD_LOST bounce within one bad SLAM patch … a solve slow enough to
> trip this almost always exceeds `plan_timeout_s` before it finishes, so the FSM bounces
> HOLD_LOST -> OK -> a FRESH SLAM_HOLD every time; resetting this on every fresh hold entry (the old
> behavior) meant the escalation could never reach its cap **in exactly the scenario it exists to
> bound**. Reset ONLY on a genuinely trusted recovery … NOT on every `_enter_slam_hold`.

A verbatim description of this bug. `_slam_hold_start` is declared on the **very next line (935)** and
never received the same treatment — `_enter_slam_hold`'s docstring spells out why the counter must
persist while the line right below it resets the clock. This is the session-43/44/50 family's fourth
instance: a bounded give-up whose bound is measured on a clock the bounce keeps zeroing.

## The fix

Give the hold a **SLAM-slow episode clock that survives the bounce**, mirroring
`_slam_stepback_count`'s documented persistence rule exactly. Scope is **`SLAM_HOLD` only**
(operator's call — see Backlog).

**`autopilot.py`, four touch points:**

1. **New field `_slam_hold_episode_t0`** beside `_slam_hold_start` (`:935`), with a comment block
   pointing at this flight and at the `_slam_stepback_count` precedent directly above it.
2. **`_enter_slam_hold` (`:2301`)** — stamp it **only when it is currently `None`**, leaving
   `_slam_hold_start = now` untouched. Deliberately a new field rather than repurposing
   `_slam_hold_start`: that one still feeds the `"SLAM settled after {waited:.1f}s"` /
   `"still waiting after {waited:.1f}s"` log text as *this hold's* wait, and silently widening it to
   episode-wide would change numbers the operator reads on every flight. Extend the docstring, which
   already explains the persistence rule for the counter, to cover the clock.
3. **The rescue check (`:3123`, `:3143`)** — measure `waited` from the **episode** clock. The log line
   reports **both** (`"this hold 3.0s / episode 15.2s"`) so the bounce is visible on the face of the
   log rather than inferred. The `has_any_capture` guard is **kept unchanged**: a wall clock must
   never paper over a total capture blackout (NO SILENT FALLBACK). It passes here — every frame
   carries a real `cap_ts`.
4. **Reset sites** — exactly the boundaries `_slam_stepback_count` already uses, no new ones:
   - the `REPLAN` handler (`:3630`) — trusted recovery / materially new goal;
   - `reset_leg()` (`:1314`) — autonomy pause / leg interruption;
   - `SLAM_HOLD`'s settle-gate-clear branch (`:3074`) — the hold genuinely settled, episode over.

**Predicted effect on this flight:** the hop reaches its bar roughly every 15 s of episode time, each
hop costing ~3 s of real motion and resetting the clock via `REPLAN` — about **8 hops across the
window that produced 0**. The one hop that did fire proves the maneuver completes and spends its
grace correctly (session 45 already fixed `PARALLAX_PUSH` missing from `_enter`'s exemption list), so
this restores a known-working escape rather than inventing one.

**No new config knobs.** `slam_slow_hop_after_s` (15.0) is reused as-is.

**No escalation is built** (operator's call). The hop is the only lever that might unstick a slow
solve — it moves the drone and re-exposes geometry — and this flight has no evidence for choosing an
N. Judge from the live-fly whether a cap is needed.

## Verification

- Self-tests: `python autopilot.py --self-test`, plus `frontier_planner.py`, `visual_recovery.py`,
  `flight_replay.py`.
- **New self-test**, modelled directly on the existing `SLAM-STEPBACK counter PERSISTENCE` test
  (`autopilot.py:7786-7838`), which already drives the precise `OK`/`PLAN-LOST` bounce needed:
  drive `status="OK"` with `slam_ms=3155` for 3.0 s → assert `SLAM_HOLD` and **no** hop; flip to
  `PLAN-LOST` for one tick → assert `HOLD_LOST` and the episode clock **preserved**; flip back to `OK`
  → assert a fresh `SLAM_HOLD` that still preserves it; repeat until episode time ≥ 15 s → assert the
  forced hop **fires** (state `REPLAN`, `SLAM_HOLD_FORCED_HOP` noted). Then assert `REPLAN` clears it.
- **Prove the test against the defect** (house rule): revert the fix on a scratch copy and confirm the
  new assertion FAILS — specifically on *the hop never firing*, not on some incidental state, so it
  isolates this defect the way session 51's re-asserted GATE A test does.
- Replay `20260902_143207` through `flight_replay.py` to confirm the timeline renders the hops.
- **LIVE-FLY**: during a slow-SLAM stretch, expect repeated `SLAM_HOLD still waiting … forcing one hop`
  lines naming both clocks, instead of a silent hover; confirm the loop cannot persist past ~18 s.

## Backlog (recorded, not built)

- **The two sibling rescues share the identical hazard** and were deliberately left alone this session:
  `SETTLE`'s session-50 escape reads `now - self.t_state` (`:4152`), `TRIM_RESUME_WAIT`'s session-44
  escape reads `_settle_gate_t0` with a `t_state` fallback (`:4228`) — **both re-stamped on re-entry**,
  so both starve the same way if a bad patch bounces them. Neither manifested on this flight. Note in
  `PROGRESS.md` so the next session can find it.
- **Why does SLAM choke?** Still the open top-of-list question. This flight adds a data point: median
  **3155 ms**, worse than session 50's ~2000 ms plateau and session 52's 1943 ms, with `plan_valid`
  true throughout — so whatever it is, it degrades *throughput* without ever costing *tracking*.

## Closing steps (non-negotiable, per CLAUDE.md)

1. Update `PROGRESS.md` — fold this into the concise narrative, refresh the "Next" resume pointer,
   reference the new `plans/session53-slamhold-limit-cycle.md`, and record both backlog items above.
2. Leave the tree and `PROGRESS.md` clean and self-describing for a cold resume.
