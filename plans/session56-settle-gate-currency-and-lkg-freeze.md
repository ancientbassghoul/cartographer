# Session 56 — settle-gate currency proof + F_LKG freeze

## Origin

Source flight `OUTPUT/diag/20260902_165340_autopilot.log` (~34 min, salvaged per session 55's tooling
after a GPU-driver TDR bugcheck). SLAM was chronically slow for the whole flight: **n=391 solves,
median 1699 ms, p75 3227 ms, p95 17867 ms, max 90774 ms**, against `slam_slow_ms=1000`. The shared
settle gate required 6 *consecutive* frames under that bar — arithmetically unreachable once p25 is
already 1251 ms — so every `SLAM_HOLD`/`SETTLE`/`TRIM_RESUME_WAIT` exit in this flight came from the
15 s `slam_slow_hop_after_s` dead-band escape, not the gate itself. The drone paid roughly 15 s of
hover for every ~2 s of real motion.

Three consequences, each confirmed from the log before any code was touched:

1. **TRIM starved on reachability, not on height.** `_TRIM_TRIGGER_STATES = {"SETTLE","ADVANCE"}`
   (pre-session-56). The drone never entered either state between `17:16:36.961` and `17:20:51.888` —
   TRIM fired 3 ms after `ADVANCE` was finally reached.
2. **F_LKG was silently replaced by the live frame.** The reference cache was gated on `plan_valid`
   read off a plan of *any* age; `PLAN-LOST` is a pure age verdict, so a stale plan still reads
   `plan_valid=True` and the reference was re-stored every tick of the loss. Past ~16 s the true frame
   aged out of the 160-entry ring and the live frame was substituted. Fingerprint at `17:00:32.140`:
   `inliers=732 contained=True planar_like=True scale=1.00` — a frame matched against itself. The old
   one-shot warning for this fired exactly once, at `16:56:07`, then went silent for the rest of the
   flight.
3. **Visual matching was silent when needed, runaway when not.** GATE A permits one match per loss
   *edge*; the 71 s and 89 s blackouts each got exactly one look. Meanwhile the per-tick reference
   re-store nulled the SIFT memos, permanently forcing GATE B open — 438 full SIFT matches in the
   `17:22` minute alone, feeding a BACKOFF decided on ~19 inliers whose homography scale swung
   0.65→27.4 within half a second.

## Design

### 1. Currency replaces speed as the settle gate's proof (`_slam_gate_since`, `_settle_gate_begin`, `_settle_gate_poll`)

The gate's old job was "prove the last 6 solves were fast." The new job is "prove *one* solve landed
of a frame captured at or after the gate opened" — a currency proof, not a throughput proof. A slow
solve is still evidence the *pose* is trustworthy; it just took longer to arrive. `_slam_gate_since`
is the monotonic floor: `None` means satisfied, non-`None` means still waiting on a fresh-enough
solve. `_update_slam` is the sole site that clears it (`cap >= floor`). The physical-motion half of
the gate (`_settle_gate_t0` / `settle_gate_s`) is untouched — both gates must still clear.

`_settle_gate_begin` takes `new_floor` (default `True`) so ordinary entries stamp a fresh floor, while
`_enter_slam_hold` re-entering an *already-open* bad-SLAM episode can pass `new_floor=False` and leave
the existing floor alone — the same "don't let a bounce zero the clock it's supposed to survive"
lesson session 53 already applied to the episode clock, one field over.

### 2. Release grace, so the faster gate can't become a limit cycle

A currency-only gate now releases in roughly one capture-gap-plus-one-solve instead of 15 s — which
means the *next* frame is very likely to still be slow. Without a grace window, that next slow frame
would immediately re-trip whatever diverted into `SLAM_HOLD` in the first place, trading a 15 s stall
for a fast oscillation. Every gated release (`SLAM_HOLD`, `SETTLE`, `TRIM_RESUME_WAIT`) now stamps
`_slam_slow_hop_deadline = now + slam_slow_hop_grace_s` (reusing the existing 8 s knob, not a new
one) at the moment it resumes, buying real flight time before slow-SLAM logic can re-arm.

### 3. F_LKG is frozen to a CURRENT, valid plan (`_visrec_should_cache_reference`)

```python
def _visrec_should_cache_reference(status, plan):
    return status == "OK" and bool(plan.get("plan_valid"))
```

`status == "OK"` is the missing half — `plan_valid` alone doesn't distinguish a fresh plan from a
stale one still inside its `plan_valid` window. Gating cache writes on both closes the self-match hole
directly: a `PLAN-LOST` episode can no longer re-store a stale-but-valid-looking plan's frame as if it
were current.

### 4. No fake reference on ring age-out — visible degradation instead

When the plan's `frame_id` isn't in the 160-entry ring (SLAM solve latency has outrun the ring depth),
the old code substituted the live frame under a `"live(aged-out)"` label — which is exactly the
self-match hazard in a different guise, just with an honest-looking label. The fix: don't store
*anything* that tick. Keep whichever reference is already held — an OLD true reference beats a FAKE
current one. `visrec_lkg_ageouts` counts it, `visrec_lkg_degraded` latches `True` (sticky, mirrors
`visrec_window_failed`/`visrec_save_failed`), and a rate-limited (`visrec_lkg_ageout_log_interval_s`,
5.0 s — the one new knob this session) CRITICAL line names the plan's `frame_id`, the ring's actual
span, and the currently-held `src`. The visualizer's telemetry panel surfaces the same state as
`LKG=slam:<id>` normally and `LKG=STALE x<ageouts>` in red when degraded — never silent.

### 5. TRIM is reachable from `SLAM_HOLD`

`_TRIM_TRIGGER_STATES` gains `"SLAM_HOLD"`. Height correction no longer has to wait for the drone to
reach `SETTLE`/`ADVANCE` before a sagging read can be corrected — directly closes the `17:16:36.961`–
`17:20:51.888` starvation this session's own diagnosis found. The TRIM entry preserves
`_slam_hold_episode_t0` and `_recovering`/`_history_broken` exactly as a mid-hold TRIM already did from
other trigger states, and honours the pre-trim `_slam_resume` target on return.

## Traps caught

- **Ordering trap on the release-grace stamp.** `_enter(nxt, now)` wipes `_slam_slow_hop_deadline` for
  any state outside `{"ADVANCE","ORIENT","PARALLAX_PUSH"}`, and `nxt` here is usually `SETTLE`/`REPLAN`
  — so the grace stamp has to happen *after* `_enter`, never before, or it is silently discarded on
  the very tick it's meant to protect. This is the identical shape as the forced-hop's own established
  ordering trap (`autopilot.py:3212` / `:4223-4228`); the fix follows the same pattern rather than
  reinventing one.
- **"Old-but-true" vs "current-but-fake."** The natural instinct when a plan frame ages out of the
  ring is to substitute *something* rather than storing nothing. That something (the live frame) is
  precisely the failure mode being fixed. The design deliberately keeps a stale reference over a fake
  fresh one, and makes the staleness visible (counter, sticky flag, rate-limited log, telemetry panel)
  rather than papering over it — the CLAUDE.md "no silent fallback, visible degraded-state flag"
  pattern applied directly.
- **TRIM-from-SLAM_HOLD must not look like a trust restoration.** A height correction is not evidence
  the pose recovered — only `SETTLE`'s own settle-gate-based trust check may clear
  `_recovering`/`_history_broken`. Verified by a dedicated self-test
  (`episode_clock_and_trust_survive`) that a TRIM detour from `SLAM_HOLD` leaves both untouched.

## Revert-proof matrix

Every load-bearing fix was reverted on a scratch backup of `autopilot.py` (byte-for-byte restored and
SHA-256-verified after each), full self-test suite run against the reverted line, confirmed only the
targeted assertion(s) failed, then restored before touching the next site.

| # | Reverted | Site | Result |
|---|---|---|---|
| 1 | Currency proof (`fresh_ok = True`, unconditional) | `_settle_gate_poll` (`autopilot.py:2324`) | `SESSION-56 GATE CURRENCY` fails on `pre_gate_capture_does_not_release=False` exactly, plus 5 other pre-existing gate tests that share this mechanism (`SETTLE fresh-frame gate`, `settle-gate two-gate design`, `SLAM step-back`, `SESSION-50 SETTLE dead-band escape`, `HEIGHT-TRIM`) — expected, since they all poll the same shared gate. |
| 2 | Release-grace stamp after a gated `SLAM_HOLD` resume | `autopilot.py:3293` | `SESSION-56 RELEASE GRACE` fails cleanly and *only* there (`slam_hold_release_stamps_grace=False`, `no_instant_redivert=False`, `grace_expires=False`); no other block moved. |
| 3 | `status == "OK"` conjunct dropped from `_visrec_should_cache_reference` | `autopilot.py:4807` | `SESSION-56 F_LKG FREEZE` fails cleanly and only there: `plan_lost_with_stale_valid_does_not_cache=False`, every other sub-check unaffected. |
| 4 | Ring age-out branch removed (falls back to storing the live frame, the pre-session-56 defect) | `autopilot.py:5283-5310` (inside `run_explore`) | **Full suite stayed ALL PASS.** See "Gap found" below — this is a real, honest finding, not a clean pass. |
| 5 | `"SLAM_HOLD"` dropped from `_TRIM_TRIGGER_STATES` | `autopilot.py:4749` | `SESSION-56 TRIM FROM SLAM_HOLD` fails cleanly on 4 of its 5 sub-assertions (`trims_from_slam_hold`, `episode_clock_and_trust_survive`, `resume_target_is_honoured`, `no_reentry_within_one_solve`); `hold_lost_still_excluded` correctly stays green (untouched by this trigger-state change, as it should). |

All five scratch edits were restored from a pre-edit backup and SHA-256-verified byte-identical
(`10dd1617ed85af1c7756dd30023bc560f00e9559f9e10aa1df2cd659a408ca91`) before moving to the next.
`visual_recovery.py` was not edited this pass; its own `(56-lkg-1)` case (a `tracked=False` call
must leave `_lkg`/`_lkg_src`/`_lkg_feats` untouched) covers the probe side directly and was left
standing rather than re-proven, since that contract predates this session (session 52) — only the new
`_lkg_t` age-timestamp field is new here, and it has no failure mode a revert would usefully isolate
(it is write-only, read only by the debug banner's cosmetic age display).

### Gap found, recorded rather than silently accepted

Reverting the ring age-out branch inside `run_explore` (#4 above) produced **zero self-test failures**.
The four `SESSION-56 F_LKG AGE-OUT` blocks that exist test the *mechanics* around the fix — the
counter/flag pair behave like their `visrec_window_failed`/`visrec_save_failed` siblings, the
telemetry payload shape round-trips, the panel renders both with and without the payload, and the
retired `"live(aged-out)"` label no longer appears anywhere in the repo's own source — but none of
them drive `run_explore`'s live loop with a `_lkg_ring` and a plan `frame_id` that misses it, because
`_lkg_ring` is local to `run_explore` and that function isn't unit-testable the way the FSM `step()`
helpers are. The fix is correct by inspection (mirrors the already-proven `_visrec_should_cache_reference`
pattern one call site over) and the flight's own fingerprint evidence supports it, but there is
currently **no automated assertion that would catch a regression at the exact integration site the bug
lived in**. Flagged here rather than claimed as covered — worth a `run_explore`-level harness test in
a future session if this class of bug recurs.

## Replay arithmetic — Step 7.2

Recomputed what the currency-based gate (§1) plus its release grace (§2) would have produced for the
five observed dead-band-escape holds, using each hold's own logged episode-start time (event
timestamp minus the logged episode duration) and the flight's own solve-latency distribution (median
1699 ms, p75 3227 ms) plus a roughly constant capture-gap component, giving "one capture gap + one
solve ≈ 3.5 s median, ~5 s p75" per the session's own measurement:

| Observed hold | Escaped at (old, 15 s/89 s dead-band) | Episode start (back-computed) | New median (start + 3.5 s) | New p75 (start + 5 s) | Time saved (median) |
|---|---|---|---|---|---|
| 15.0 s | `17:03:56.860` | `17:03:41.860` | `17:03:45.360` | `17:03:46.860` | 11.5 s |
| 15.8 s | `17:16:34.156` | `17:16:18.356` | `17:16:21.856` | `17:16:23.356` | 12.3 s |
| 21.4 s | `17:14:45.198` | `17:14:23.798` | `17:14:27.298` | `17:14:28.798` | 17.9 s |
| 74.5 s | `17:17:53.759` | `17:16:39.259` | `17:16:42.759` | `17:16:44.259` | 71.0 s |
| 89.2 s | `17:20:51.634` | `17:19:22.434` | `17:19:25.934` | `17:19:27.434` | 85.7 s |

Across just these five samples: old total hover **215.9 s**, new total (median) **17.5 s**, new total
(p75) **25.0 s** — roughly a 90% cut in dead-time on the flight's five worst episodes, without ever
requiring SLAM to actually get faster. (The underlying SLAM choke itself is untouched by this session
— see PROGRESS.md's backlog.)

## Files touched

- `autopilot.py` — `_slam_gate_since` field, `_settle_gate_begin`/`_settle_gate_poll` currency
  rewrite, release-grace stamps at all three gated-release sites, `visrec_lkg_ageouts` /
  `visrec_lkg_degraded` fields, `_visrec_should_cache_reference`, the `run_explore` ring
  resolution block, `_TRIM_TRIGGER_STATES`, `_full_vector`'s `visrec_lkg` kwarg, the `visrec` timeline
  record's `lkg_src`/`lkg_ageouts` keys, and the new self-test blocks (`SESSION-56 GATE CURRENCY`,
  `SESSION-56 RELEASE GRACE`, `SESSION-56 F_LKG FREEZE`, `SESSION-56 F_LKG AGE-OUT` ×4,
  `SESSION-56 TRIM FROM SLAM_HOLD` ×5).
- `visual_recovery.py` — `_lkg_t` age timestamp, age display on the debug banner, docstring updates
  removing the retired `"live(aged-out)"` src value, new `(56-lkg-1)` self-test case.
- `visualizer.py` — `LKG=` segment on the telemetry panel's SLAM row.
- `flight_replay.py` — carries the new `lkg_src`/`lkg_ageouts` timeline keys through to the replay
  HTML (covered by the existing visual-recovery panel self-test case).
- `config.yaml` — one new knob, `visrec_lkg_ageout_log_interval_s: 5.0`.

## Verification

1. `python autopilot.py --self-test`, `python visual_recovery.py --self-test`,
   `python frontier_planner.py --self-test`, `python flight_replay.py --self-test`,
   `python ground_grid.py --self-test`, `python map_store.py`,
   `python salvage_flight.py --self-test`, `venv\Scripts\python perception_worker.py --self-test` —
   **all 8, ALL PASS, 0 failures**, run clean (no reverts in place) as the final check.
2. Revert-proof matrix above — 4 of 5 fixes isolate cleanly to their own new assertions; 1 (`run_explore`
   ring age-out) has no direct integration-level test and is recorded as a real gap, not claimed as
   proven.
3. **Live-fly is the next step** — see PROGRESS.md's watch list. Per the global spec's rule 8, the
   drone was **not** flown between chunks 2 and 3 during this build, so the release-grace fix (§2) has
   never been exercised against a real 3.5 s limit-cycle risk; this is the first live flight that will
   test it.
