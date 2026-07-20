# Session 27 — visualizer video recording, SLAM point-cloud export on quit, graceful shutdown for all three processes

Two new features requested, plus a bug found (and fixed) in the first feature's own shutdown path.

## 1 — Video recording (`visualizer.py`)

`visualizer.py` already owns no GPU (its own docstring: "pure display") and composes one BGR image
per tick with plain numpy/OpenCV drawing before `cv2.imshow`. Recording is just writing that same
already-composed image to a `cv2.VideoWriter` — zero additional GPU work, by construction.

Added `--record`/`--record-fps` (default 15). Since the render loop's tick rate isn't fixed
(`state_sub.recv(timeout_ms=30)` + `cv2.waitKey(15)`), writes are wall-clock throttled
(`time.monotonic() - last_write >= 1.0/record_fps`) so playback approximates real elapsed time
regardless of render jitter. Output: `OUTPUT/diag/<ts>_visualizer.mp4` (`mp4v`, no new dependency —
`opencv-python`, already required for `cv2.imshow`, bundles the FFmpeg backend).

## 2 — SLAM point-cloud export on quit (`perception_worker.py`)

`map_store.py` already had a working `MapStore.save_ply()` — a Blender-loadable ASCII `.ply` (voxel
occupancy in true color + flight path in green + targets in magenta) — plus `save_npz`/
`render_topdown` siblings, proven by the **offline** `--video` export path
(`run_offline_video()`, ~line 839-843). The format problem was already solved; the actual gap was
that **none of it ever ran for a live flight**: `run_live()`'s `finally:` never called any export
function, and even if it did, it would rarely fire — `fly.py` launches `perception_worker.py
--no-display` (no window, no `'q'`-quit path) and hard-`terminate()`s it on stop, which skips
`finally` entirely. Exactly the problem `autopilot.py` already solved for itself with a `--stop-file`
sentinel (`_FileStopEvent`) it polls each loop iteration, letting it exit normally instead of relying
on a Ctrl+C a `CREATE_NEW_CONSOLE` child can't even receive from its parent on Windows.

Gave `perception_worker.py` the same `--stop-file` mechanism (inline poll at the top of `run_live()`'s
`while True:`, mirroring autopilot's `_FileStopEvent` pattern), and wired the same three
already-proven export calls into its `finally:` block, `<ts>`-prefixed into `OUTPUT/diag/` (where the
live diag CSVs already land) instead of `<stem>`-prefixed into `OUTPUT/`. One `ts` (generated once at
`run_live()`'s top) is now shared between the diag CSVs (`pipe.enable_diag(ts=ts)`) and the map export.

## 3 — Bug found in #1's own shutdown path: corrupted MP4

After wiring `fly.py` to launch perception with its new `--stop-file` (sequenced *after* autopilot's
own graceful stop completes — autopilot still needs perception's published pose/plan while flying its
last leg), `visualizer.py --record` was left in the generic `processes` list that `fly.py` just
`terminate()`s — the exact same class of bug #2 above just fixed, freshly introduced in the very same
session by not extending the same reasoning to the video writer.

**Reproduced directly** (not just theorized): hard-terminating a `cv2.VideoWriter`-holding process
mid-write (`Popen.terminate()` — exactly what `fly.py`'s teardown does) leaves the MP4 with `mdat` but
NO `moov` atom (the movie header / frame index — without it no player can decode the file, regardless
of how much raw frame data sits in `mdat`). Confirmed via manual MP4 box parsing:
```
clean.mp4  (writer.release() called):  ftyp, free, mdat(67428B), moov(1714B)
killed.mp4 (hard-terminated mid-write): ftyp, free, mdat(0B)                    <- no moov
```
Fix: gave `visualizer.py` the identical `--stop-file` pattern (poll at loop top, `break` -> `finally:`
-> `writer.release()`). Re-verified end-to-end against the real module: hard-killing without a
stop-file reproduces the corrupt structure again (`mdat(0B)`, no `moov`); signaling the stop-file makes
it exit cleanly (`return code 0`) and produces a valid file (`mdat(72035B), moov(954B)`).

`fly.py` now tracks all three of `autopilot`/`perception`/`visualizer` as named variables (not the
generic hard-terminated `processes` list), each given its own sentinel + `wait(timeout=15)` +
`terminate()`-fallback, autopilot first (needs perception alive), then perception, then visualizer
(order-independent — nothing depends on visualizer staying alive).

## Status

All three pieces built + self-verified (compile/import checks, `perception_worker.py --self-test`,
`autopilot.py --self-test` all green, and the MP4 corruption fix specifically reproduced + confirmed
fixed via direct hard-kill tests against the real module — see above). Not yet confirmed against an
actual live flight with real camera frames (this environment has no GPU/drone hardware) — the operator
flew a session right after this was built; results to fold into the next session's log per the
operator's own account it was "exceptional" apart from a new, separate bug to be diagnosed next.
