# Session 55 — crash survivability

## Trigger

An unattended flight (`20260902_165340`, ~34 min) ended when the machine bugchecked:
`0x116 VIDEO_TDR_ERROR` at ~17:28 (Windows System event log, `WER-SystemErrorReporting` 1001). SLAM
itself was never at risk — `slam_engine.py:71` hard-asserts `cuda:0` — the TDR was the Intel iGPU
choking on `Xlab.exe`'s render load; `HKCU\Software\Microsoft\DirectX\UserGpuPreferences` was empty, so
Unity got whatever Optimus defaults to (the iGPU on this laptop). Operational mitigation (no code):
Settings → Graphics → force `Xlab.exe` to the RTX 3080 ("High performance"). That reduces crash
*frequency*; this session is about surviving the next one regardless of cause.

## What was actually lost vs. what survived

A bugcheck skips every process's `finally` — none of `fly.py`'s three graceful-stop sentinels fired.
Despite that, most of the 34-minute flight survived, because the text log / CSVs / timeline already
`flush()` per write (`diag_log.py:33`, `autopilot.py` `AutopilotLog.line()`/`.timeline()`). What was
actually lost or broken:

1. **The dashboard MP4's index.** `visualizer.py`'s `cv2.VideoWriter` only writes a `moov` atom (frame
   index) in `writer.release()` (`visualizer.py:588`), reachable only on a normal loop exit. A killed
   recording has `ftyp/free/mdat(size=0)` — the frame data is intact, just unplayable.
2. **The replay's map backdrop.** Emitted exactly once, in `finally` (`autopilot.py`, pre-session-55
   line ~5348) — a crashed flight's timeline had **zero** `"map"` records, so a report built from it
   renders on a blank scene.
3. **Perception's entire voxel map + point cloud.** `MapStore` accumulates in memory the whole flight;
   `save_npz`/`save_ply`/`render_topdown` only ran in `run_live`'s `finally` (`perception_worker.py`).
   This flight got lucky — perception's process happened to still be exiting cleanly when it mattered
   least, and its export ran. Nothing about that was guaranteed.

Everything durable was durable only against a **process kill**, not a **reboot** — `flush()` reaches
the OS page cache, not the platter; there was not one `os.fsync()` anywhere in the codebase before this
session.

## Part 0/1 — `salvage_flight.py` (new tool)

Recovers a flight after the fact. Writes only new files; never opens an original for writing.

- **MP4 repair** (`repair_mp4`): the payload in `mdat` is undamaged. The blocker wasn't a missing frame
  index for stream-copy — it's that MP4-contained MPEG-4 Part 2 stores its VOL header (width/height/
  profile) **once**, in `moov`'s `esds` box, never repeated in `mdat`. `ffmpeg -c copy` on the bare
  payload fails with `Picture size 0x0 is invalid` — confirmed empirically, including against the real
  crashed file. Fix: `find_reference_vol_header` pulls the identical VOL header (48 bytes, byte-for-byte
  reproducible) out of ANY other finalized `*_visualizer.mp4` at the same resolution — legitimate reuse,
  not a guess: `visualizer.py`'s dashboard is a **fixed pixel layout**
  (`PANEL_W`/`GAP`/`MAP_SIZE`/`STATUS_H`), not room/flight data, so two recordings at the same
  resolution have byte-identical VOL headers by construction. Prepend it, remux with `-c copy` — no
  re-encode, no frame loss. Verified end-to-end on flight `20260902_165340`: 17,437 frames recovered,
  visually confirmed (dashboard, telemetry, top-down map all intact).
- **Map-backdrop reconstruction** (`reconstruct_map_from_livemap`): when a timeline has zero `"map"`
  records and a sibling `*_livemap.npz` exists, mark OCC from the voxel centers (real structure) and
  FREE from the trajectory (the drone was physically there) — and nothing else. Deliberately does
  **not** replay `GroundGrid.integrate()`'s raycast carving (the npz has no per-frame point↔pose
  association; inventing one would fabricate free-space claims never actually observed — CLAUDE.md).
  Tagged `map_source: RECONSTRUCTED_FROM_LIVEMAP_NPZ`.
- **Torn-tail trim** (`check_and_trim_trailing_garbage`): every line is individually `flush()`'d, so a
  crash can only tear the *last* line. Detected and dropped in a new `.salvaged.jsonl`; a malformed line
  anywhere else is real corruption and raises (matches `flight_replay.load_timeline`'s own contract).
- Per operator direction: **no downsampling, no trimming of any field** — the salvaged report is full
  fidelity, same as `flight_replay.py` already produces for a healthy flight.
- `--all` scans `OUTPUT/diag/` for orphans (jsonl with no report; un-finalized mp4 with no `.recovered`
  sibling yet).
- `--self-test`: reproduces the real corruption shape deterministically (a real finalized clip,
  mdat-size patched to 0, moov dropped) rather than racing cv2's own GC-triggered `release()`; also
  covers the codec guard, the trim/no-trim boundary, and honest-vs-fabricated map reconstruction.

## Part 2 — in-flight hardening (H1/H2/H3)

New `diag:` config block (`config.yaml`) — all periodic, all general durability params, none room/
flight-specific:

- **H1** (`autopilot.py`, `timeline_map_period_s`, default 10s): the SAME `_downsample_map(last_ground)`
  call the shutdown path already made, now also on a timer inside the loop — `last_ground` was already
  tracked live every step, so this was a small, low-risk addition. `flight_replay.js` already selects
  the newest map at/before the cursor, so periodic records are a strict improvement (the map now
  visibly evolves in the replay too).
- **H2** (`perception_worker.py`, `_checkpoint_livemap`, `livemap_checkpoint_period_s`, default 60s):
  the three shutdown-only export calls hoisted into a reusable helper, called periodically AND at clean
  shutdown, writing to `*_tmp.*` + `os.replace()` (atomic on the same volume, POSIX and Windows) so a
  crash mid-write can't corrupt the *previous* good checkpoint. The periodic call is wrapped in
  `try/except OSError` — a checkpoint failure (e.g., Windows `PermissionError` if another process has
  the file open) must degrade loudly, never take the whole flight down; the final `finally` checkpoint
  is deliberately left unprotected, since a failure there should be as visible as possible.
- **H3** (`diag_log.py` + `autopilot.py`, `log_fsync_period_s`, default 2s): `DiagLog.fsync()` /
  `AutopilotLog.fsync()` — periodic, NOT per-record (the timeline runs ~30-50 Hz on a 300 MB/flight
  file). A failure latches per-sink (`_fsync_failed`) and is surfaced once via a CRITICAL print;
  `flush()` keeps working regardless — only the reboot-safety margin degrades, not correctness.

All three are self-tested: H1 via the existing `_downsample_map` machinery (periodic gating itself is
inline in `run_explore`'s ZMQ loop, not unit-testable in isolation — verified live-fly only); H2 via a
duck-typed `Pipeline` stand-in proving atomic replace-not-merge semantics; H3 via monkeypatching
`os.fsync` to prove the failure/latch/silence contract without corrupting the test's own file handles.

## Part 3 — `fly.py`: run manifest + auto-recovery

A `<launch_ts>_run.json` written `"in_progress"` at launch, marked `"completed"` only by the SAME
teardown a hard crash always skips — a reliable crash signal for the *next* launch. On startup, before
touching anything else, `fly.py` scans for `"in_progress"` manifests (plus, independently, any
un-finalized `*_visualizer.mp4` with no `.recovered.mp4` yet, covering flights that predate this
manifest) and OFFERS — never silently runs — `salvage_flight.py` on each. `salvage_flight.py`, on a
successful salvage, marks the matching manifest `"salvaged"` so it stops being re-flagged.

Verified against the real repo: a fresh scan found not just `20260902_165340` (correctly *not*
re-flagged, since it already has a `.recovered.mp4`) but **three older, previously-unnoticed crashed
flights** (`20260720_133111`, `20260720_135245`, `20260720_135307`) — genuine orphans from before this
session, never salvaged. Left for the operator to run `salvage_flight.py --all` on at their convenience
— not auto-run.

## Explicitly not done (per operator direction)

- No video segmentation (rejected in favor of the repair-tool approach).
- No trimming/downsampling of `goal_db`/`clearance_detail`/`visual_recovery_detail` or any other
  timeline field, in salvage or otherwise — kept exactly as recorded so the raw log stays directly
  diagnosable.

## Verification status

- `python salvage_flight.py --self-test`, `autopilot.py --self-test`, `perception_worker.py
  --self-test`, `ground_grid.py --self-test`, `map_store.py --self-test`, `flight_replay.py
  --self-test`: **all green.**
- `python salvage_flight.py 20260902_165340`: full real-flight salvage, verified end-to-end (video
  playback spot-checked visually, map reconstruction inspected, originals confirmed byte-identical
  before/after).
- **NOT yet verified live-fly**: H1/H2/H3's actual periodic firing during a real flight, and the
  fly.py crash-recovery prompt on a real killed run. Next step: a short flight, then
  `Stop-Process -Force` on `perception_worker.py`/`visualizer.py` mid-flight (mirrors
  `plans/session27-*.md`'s own reproduction), confirm a checkpointed livemap + periodic map records
  exist, then relaunch `fly.py` and confirm the recovery prompt fires.
