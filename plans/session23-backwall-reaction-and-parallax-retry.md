# Session 23 — wire the flow BACKWALL detector into a real decision + PARALLAX_PUSH side-retry

## Diagnosis (flight `20260717_102403`, loop starts `10:27:09.454`)

`flow_contact_detector.py` already implements `BACKWALL` as the exact mirror of `CEILING`/`WALL`: while a
`reverse` command is held, the contraction-magnitude flow signal is tracked against a live per-flight
reference; a collapse to ~0 for `contact_seconds` (0.8s) means "sent reverse, but the image shows we stopped
moving = backed into something." But `autopilot.py` computed this verdict and threw it away — explicitly
commented `"DETECTION-ONLY this session (no control reaction yet)"` in two places, and
`ExploreController.step()` didn't even accept a `backwall_contact` argument.

The flight log is a clean recorded instance of exactly this gap: at `10:27:08.691` the drone ORIENTed away
from a wall SLAM hadn't mapped behind it yet, so the clearance ring at 180° read `None` ("open") and
`PARALLAX_PUSH` picked BACKWARD. The detector fired twice — `10:27:23.086` and `10:27:35.542`:
`"BACKWALL contact (reverse into a wall; detection-only — no reaction yet)"`. Ground truth: position barely
moved across 4 push attempts over ~30s (`[-1.68,1.22]` → `[-1.74,1.40]`), heading swung wildly (consistent
with the airframe being deflected by contact, not a commanded turn), SLAM died twice, and every single push
ended `"(timer)"` — it silently ran the full `parallax_push_s` (2.0s) of reverse thrust into the wall every
attempt, then turned and repeated, until an unrelated height-TRIM branch happened to break the cycle.

## What was built

1. **`ExploreController.step()`** gained a `backwall_contact=False` parameter (mirrors `ceiling_contact`).
   `run_explore`'s per-frame detector block now promotes the previously-log-only `now_backwall` reading into
   a real flag threaded into `ctrl.step()`.
2. **`PARALLAX_PUSH`'s direction pick was extracted into `_pick_ring_direction(ring, plan,
   force_no_backward=False)`** — the same "backward if pushable, else the roomier side (with the D2
   scrape-guard reposition-forward branch), else give up" logic that used to run only once at entry. It's now
   also called **mid-push** whenever a backward push is aborted by either the ring guard OR a live
   `backwall_contact` (passing `force_no_backward=True`, since the episode just proved backward bad
   regardless of what the ring says): if a side has room, hand off to it **in the same episode** (no
   settle/replan/re-turn — reuses the existing D2 reposition→strafe hand-off pattern); only if both sides are
   also blocked does it bail to `SETTLE → REPLAN`, logging *"no room for backwards parallax push"* and marking
   a missed-bump. This also upgrades the *pre-existing* ring-based mid-push block, which previously bailed
   straight to settle/replan without ever trying a side.
3. **Give-up MEMORY**: a full give-up (backward excluded/blocked AND both sides blocked) latches
   `_parallax_back_blocked` + the drone's position (`_parallax_back_blocked_anchor`). The NEXT direction pick
   — even a leg later, after settle→replan→re-orient — won't retry backward near that spot just because the
   ring's "open" reading hasn't changed (SLAM still hasn't mapped what's behind); this is what stops the
   "give up → re-orient → ring says clear → try backward again → blocked again" ping-pong one level up.
   Cleared once the drone has moved `parallax_min_clear` (0.7u) away from the anchor — SLAM-freeze-safe (a
   missing/frozen pose never clears it), mirroring the existing `rearm_bump_if_disengaged` anchor-distance
   pattern.
4. **`REVERSE_PROBE`** (the *other* spot flagged detection-only — live by default via
   `reverse_probe_on_wall: true`) now ends its fixed-duration `reverse_probe` recipe EARLY on a live
   `backwall_contact` instead of only its natural timeout.
5. Not touched (documented follow-up, not scope-crept in): `BACKOFF` and the ring-picked backward push inside
   `CALIB_ESCAPE`/`_begin_fallback` (FALLBACK) also command `reverse` but run a short bounded-duration recipe
   rather than an open-ended timed hold — lower risk, not implicated by the log.

## Self-tests added (all in `autopilot.py --self-test`)

- PARALLAX-PUSH backward-blocked retry: flow contact → hands off to a side in-place (no re-orient); both
  sides also tight → real give-up (missed-bump text checked); the *existing* ring-only mid-push block also
  now retries a side instead of bailing straight to settle/replan.
- PARALLAX-PUSH give-up MEMORY: latches on give-up, suppresses a nearby re-pick, clears once moved away.
- REVERSE-PROBE-BACKWALL: a live contact ends the probe well before its 4.0s fixed duration.

All 6 module self-test suites green (`python autopilot.py --self-test`, `python flow_contact_detector.py
--self-test`).

**NEXT = LIVE-FLY** alongside the still-pending sessions 20b/21/22 checklist. Watch for a
`parallax backward blocked (...) -> strafe_...` or `-> no room back/left/right either` log line (instead of
the old silent `"(timer)"` grind) whenever SLAM hasn't mapped what's behind a parallax scout.
