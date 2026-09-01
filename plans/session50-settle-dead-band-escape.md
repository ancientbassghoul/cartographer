# Session 50 — SETTLE's slow-SLAM dead band: alive, tracking, and wedged for 91.6 seconds

## Origin

The operator flagged the tail of flight `20260901_172217`: "SLAM was stuck on SETTLE at around 17:27.
I've never seen it stuck on settle for so long — normally it falls to SLAM_HOLD."

## The trace

`SETTLE` from **17:25:22.774 → 17:26:54.390 — 91.6 seconds, 3531 ticks**, plan status `OK` the whole
time (the previous status line was `OK` at 17:25:04.985). Valid pose, valid goal, drone parked.

SLAM across that window:

```
frames=42   min=1834ms   max=2093ms   mean=1995ms
frames under slam_slow_ms (1000): 0
```

Not one frame under the threshold — consistently **~2x over**, and eerily flat (1978 / 1992 / 1995 /
2003 …), the signature of a system settled into a stable bad equilibrium rather than noisy contention.

`SETTLE`'s only exit is `_settle_gate_poll()`, which needs `settle_fresh_frames` (6) **consecutive**
frames under `slam_slow_ms` (1000 ms). At a steady 1995 ms that is not "slow to clear" — it is
**arithmetically impossible**. Zero of 42 frames qualified.

## Why nothing rescued it

`ORIENT` and `ADVANCE` both carry an explicit slow-SLAM bail-out:

```python
if self._slam_slow and not self._slam_slow_hop_active(now):
    return self._enter_slam_hold(...)      # autopilot.py, ORIENT and ADVANCE handlers
```

**`SETTLE` had no such check**, and its own comment explains the assumption that failed:

```python
# No timeout on a gated settle: if SLAM stops delivering, the plan status goes STALE/LOST and the
# step() top diverts to recovery.
```

SLAM never *stopped* delivering. It delivered like clockwork, every ~2.2 s. So the drone landed in
the **dead band between two thresholds**:

| | needs | actual | verdict |
|---|---|---|---|
| settle-gate | a frame < **1000 ms** | ~1995 ms | never clears |
| `PLAN-LOST` | silence > **3.0 s** (`plan_timeout_s`) | frames every ~2.2 s | never trips |

Too slow to proceed, too alive to be rescued. And `SLAM_HOLD`'s 15 s forced-hop escape — which
**worked in this same flight** at `17:25:19.985` — was unreachable, because you can only get it from
inside `SLAM_HOLD`, and `SETTLE` never diverts there.

It escaped only by accident: the last frame landed at `17:26:51.423`, then a **3.006 s** gap finally
tipped `plan_age` past `plan_timeout_s` → `PLAN-LOST` at `17:26:54.429` → `HOLD_LOST`.

This is the same bug class session 44 fixed for `TRIM_RESUME_WAIT` ("unlike `SLAM_HOLD`, never had a
timeout for sustained slowness, so it just hung") — fixed there, never applied to `SETTLE`.

## Built

Session 43's rule, verbatim, in `SETTLE`: **plan OK + stuck past `slam_slow_hop_after_s` → proceed
anyway, loudly.** Reuses the existing knob (15 s) rather than adding another — it is the same concept
(`SLAM_HOLD` and `TRIM_RESUME_WAIT` already share it), and one fewer number to tune.

- Anchored on `now - self.t_state` (time in `SETTLE` itself), not the gate window, which can be
  carried over from an antecedent `SLAM_HOLD` (session 24) and would otherwise double-count.
- **Stamps `_slam_slow_hop_deadline` AFTER `_enter(nxt)`**, the identical ordering trap `SLAM_HOLD`'s
  hop documents: `_enter()` wipes that deadline for any state outside
  `("ADVANCE", "ORIENT", "PARALLAX_PUSH")`, and `nxt` is normally `REPLAN`. Without the grace the very
  next `ORIENT`/`ADVANCE` would re-divert into `SLAM_HOLD` on the same slow frames and the forced
  proceed would buy nothing.
- **One guard kept**, mirroring `SLAM_HOLD`'s: a wall clock cannot distinguish "slow but alive" from a
  total capture blackout (perception producing nothing timestamped). At least one entry in the current
  SLAM window must carry a real `cap_ts`, so a forced proceed never flies on zero live data.
- Deliberately **not** a divert to `SLAM_HOLD`: `SETTLE` is frequently entered *from* `SLAM_HOLD`
  (`_slam_resume="SETTLE"`, gate window deliberately carried over), so a naive divert would ping-pong
  the two at tick rate. The escape belongs *in* `SETTLE`.
- Reaching the handler already implies status `OK` (the step() top diverts every non-OK status for a
  non-exempt state — which is precisely how this flight eventually left `SETTLE`), so no status recheck.

## Verified

`python autopilot.py --self-test`: **ALL PASS**. New block `SESSION-50 SETTLE slow-SLAM dead-band
escape` reproduces the flight exactly — a SLAM stream that is alive, tracking, status `OK`, publishing
a fresh frame every tick, at **1995 ms** each:

| assertion | |
|---|---|
| holds before the timeout | the gate is still allowed to do its job |
| forces the wedge out past it | `REPLAN` reached instead of hanging |
| arms the hop grace | or the next `ORIENT`/`ADVANCE` re-diverts and nothing is gained |
| healthy SLAM still exits via the normal gate | the escape is a backstop, never the primary path |
| capture blackout still holds | a wall clock must not paper over perception saying *nothing* |

Proof-of-defect: disabling the escape on a scratch copy flips the wedge assertions to FAIL (the
controller never leaves `SETTLE`) while the healthy-SLAM and blackout cases stay green. Restored.

## Next

**LIVE-FLY.** Watch for `SETTLE gate blocked N.Ns by slow-but-ALIVE SLAM ... -> forcing REPLAN` in
place of a multi-minute park. If it fires *often*, that is not a bug in this fix — it is the honest
signal that SLAM is chronically running over `slam_slow_ms`, and the thing to chase is the choke
itself (still open — see session 51's negative results).
