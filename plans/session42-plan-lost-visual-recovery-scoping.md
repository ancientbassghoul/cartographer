# Session 42 — scope PLAN-LOST out of the VISUAL_RECOVERY hand-off

## Origin

Operator ask, following on from the session-36/41 visual-recovery discussion: is it true that
`VISUAL_RECOVERY` stays "live" even after entry, still caching fresh frames while a loss is
ongoing? That question led to pulling the actual `20260721_233244` / `20260722_124351` flight
logs, which turned up something neither side expected.

## Diagnosis

Every `VISUAL_RECOVERY` entry in both flights (62 total) was triggered by `PLAN-LOST`, none by
`PLAN-STALE` — and every single one lasted exactly one tick before reverting to `HOLD_LOST`,
still under `PLAN-LOST`. Example: `slam_ms` sat frozen at a perfectly normal 358.5ms the whole
time (not a slow solve at the moment of entry) while `plan_age_s` climbed past the 3.0s timeout,
`VISUAL_RECOVERY` logged "15° visual recovery probe" for one tick, then immediately fell back to
`HOLD_LOST`. Root cause: `_step_visual_recovery` (the actual TURN→MATCH→WAIT_RECOVER machinery)
is only ever dispatched from `_step_stale`, itself only reached `if status == "PLAN-STALE"`. The
top-level `PLAN-LOST`/`NO-PLAN` branch has no equivalent dispatch for an in-progress
`VISUAL_RECOVERY`, so the tick after a PLAN-LOST-triggered entry, it falls into the generic "not
HOLD_LOST -> enter HOLD_LOST" logic and gets swept straight back out. In practice, the probe has
never actually executed a turn on a real flight.

## Decision

The operator's argument, which this evidence supports: `PLAN-LOST` means perception itself
stopped publishing — a throughput/backlog problem (session 28 diagnosed exactly this: a
synchronous SLAM solve blocking the loop for 9-10s), not a "this viewpoint is confusing"
problem — so the only sound universal remedy is to hold still and wait for perception to catch
up, never to actively search. `PLAN-STALE` (perception alive, SLAM explicitly reports
not-tracking) is the case where a different viewpoint is a coherent remedy, and stays the only
entry into `VISUAL_RECOVERY`. Operator confirmed: remove the hand-off for `PLAN-LOST`/`NO-PLAN`,
but explicitly **keep** the two one-shot BACKOFF reactions (cached-clearance-too-close, and
visual-too-close via a contained/planar-like F_LKG match) available for `PLAN-LOST` — those are
defensive reactions to an already-known reading, not an active search, and were never affected by
the dispatch bug above.

## Built

`_maybe_loss_snapshot_backoff` (`autopilot.py`) gained a `status=None` parameter. Steps 1/2
(the two BACKOFF reactions) are unchanged and fire for any status. Only the final hand-off —
`_enter_visual_recovery(...)` — is now gated on `status == "PLAN-STALE"`; otherwise it returns
`None`, which the caller already treats as "fall through to the plain hard-hover-hold." The two
call sites now pass the real status through: the top-level `PLAN-LOST`/`NO-PLAN` branch passes
`status=status`; `_step_stale` (only ever reached when `status == "PLAN-STALE"`) passes the
literal `status="PLAN-STALE"`. Updated the function's docstring and the class-level session-35-ALT
design comment to record the new scoping and why. No change to `_step_visual_recovery`, the
BACKOFF paths, `use_visual_recovery_on_stale` gating, or flicker-resume behavior (a probe that
started under PLAN-STALE and later flickers to PLAN-LOST already diverts to `HOLD_LOST` via a
separate, already-spent-ticket path, untouched by this fix).

Added a new self-test case, `plan_lost_no_probe_ok`, mirroring the existing "both loss-instant
checks inconclusive" case but under `status="PLAN-LOST"` instead of `"PLAN-STALE"` — asserts the
result is now `HOLD_LOST` with the "HARD HOVER-HOLD" event text, never `VISUAL_RECOVERY`. Folded
into the `VISUAL RECOVERY 15° probe` block's aggregate + print line. `python autopilot.py
--self-test`: the rewritten block PASSES (`plan-lost-no-probe=True`, all prior sub-checks
unchanged); the same two pre-existing, unrelated failures from sessions 39-41 (`explore
ALTITUDE-LOCK`, `explore PRELUDE arm+takeoff+...`) remain, unaffected by this change.

**NEXT = LIVE-FLY** — confirm a future `PLAN-LOST` episode holds cleanly in `HOLD_LOST` with no
`VISUAL_RECOVERY` flicker in the log, and that a genuine `PLAN-STALE` episode still reaches
`VISUAL_RECOVERY` and this time executes a real turn (still unobserved on a real flight — the
probe has never yet run to completion in practice).
