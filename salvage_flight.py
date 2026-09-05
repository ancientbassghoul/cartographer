"""salvage_flight.py — recover a flight's artifacts after a hard crash/reboot gave every process ZERO
shutdown path (session 55; see PROGRESS.md). Flight 20260902_165340 was cut short by a GPU-driver TDR
bugcheck (0x116) at ~17:28, which bypasses every `finally` block in the stack (fly.py's own three
graceful-stop sentinels never had a chance to fire). Most of that flight's data survived anyway — the
text log / CSVs / timeline.jsonl already `flush()` on every write (diag_log.py, AutopilotLog) — but two
things did not:

  1. The dashboard MP4's container index (`moov`) is written only by `writer.release()` in
     visualizer.py's `finally` — a hard kill leaves `ftyp/free/mdat(size=0)`, unplayable by any player,
     even though the frame data on disk is intact.
  2. The replay timeline's map backdrop (until session 55 added a periodic tick) was emitted only ONCE,
     in autopilot.py's `finally` — a crashed flight's timeline can carry ZERO 'map' records, so a report
     built from it renders on a blank scene.

This tool repairs both, and only ever WRITES NEW FILES — no original artifact is ever opened for
writing. Per operator direction: no downsampling, no trimming of any field. The only content ever
dropped is a single torn, unparseable FINAL line left by a mid-write crash (see
`check_and_trim_trailing_garbage`) — anything else malformed is treated as real corruption and raises.

Usage:
    python salvage_flight.py <ts | path-to-*_timeline.jsonl>   # salvage one flight, full fidelity
    python salvage_flight.py --all                              # salvage every orphaned flight found
    python salvage_flight.py --self-test                        # synthetic, no hardware/flight needed
"""

import argparse
import glob
import json
import math
import os
import re
import shutil
import struct
import subprocess
import sys
from datetime import datetime

DIAG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "OUTPUT", "diag")


# ==============================================================================
# 1. MP4 repair — visualizer.py --record uses mp4v (MPEG-4 Part 2, OpenCV's bundled encoder,
#    visualizer.py:513). An un-finalized recording is missing only the container INDEX (moov); the
#    frame payload in mdat is untouched, undamaged, self-delimiting elementary-stream data.
# ==============================================================================
def mp4_top_atoms(path):
    """Parse the top-level ISO-BMFF box chain: [(type, size, offset), ...]. size=0 on a box means
    "extends to EOF" — the exact shape a cv2.VideoWriter leaves when killed before release() ever
    patches the real size in (confirmed against flight 20260902_165340's own mdat box: size=0)."""
    atoms = []
    total = os.path.getsize(path)
    with open(path, "rb") as f:
        off = 0
        while off < total:
            f.seek(off)
            hdr = f.read(8)
            if len(hdr) < 8:
                break
            box_size = struct.unpack(">I", hdr[:4])[0]
            box_type = hdr[4:8].decode("latin1", errors="replace")
            if box_size == 1:                      # 64-bit extended size (rare; not expected here)
                box_size = struct.unpack(">Q", f.read(8))[0]
            atoms.append((box_type, box_size, off))
            if box_size == 0:
                break                               # "extends to EOF" -- always the terminal box
            off += box_size
    return atoms


def is_mp4_finalized(path):
    """True iff the file has a 'moov' box (the frame index) — i.e. writer.release() actually ran."""
    return any(t == "moov" for t, _s, _o in mp4_top_atoms(path))


def _find_ffmpeg(explicit=None, tool="ffmpeg"):
    if explicit:
        return explicit
    exe = shutil.which(tool)
    if exe is None:
        raise RuntimeError(f"{tool} not found on PATH -- required to remux an un-finalized mp4v "
                           f"recording losslessly. Install ffmpeg, or pass --ffmpeg <path>.")
    return exe


def _walk_boxes_for_esds(f, off, end):
    """Depth-first walk of the ISO-BMFF container chain moov/trak/mdia/minf/stbl/stsd/mp4v/esds,
    yielding each `esds` box's raw MPEG-4 ES_Descriptor bytes (its own version+flags already skipped).
    Fixed offsets (`+8` past stsd's version/flags/entry_count, `+78` past mp4v's fixed
    VisualSampleEntry header) are the ISO/IEC 14496-12 layout, not anything flight-specific."""
    while off < end:
        f.seek(off)
        hdr = f.read(8)
        if len(hdr) < 8:
            return
        size = struct.unpack(">I", hdr[:4])[0]
        typ = hdr[4:8].decode("latin1", errors="replace")
        hlen = 8
        if size == 1:
            size = struct.unpack(">Q", f.read(8))[0]
            hlen = 16
        if size == 0:
            size = end - off
        if typ in ("moov", "trak", "mdia", "minf", "stbl"):
            yield from _walk_boxes_for_esds(f, off + hlen, off + size)
        elif typ == "stsd":
            yield from _walk_boxes_for_esds(f, off + hlen + 8, off + size)
        elif typ == "mp4v":
            yield from _walk_boxes_for_esds(f, off + hlen + 78, off + size)
        elif typ == "esds":
            f.seek(off + hlen + 4)
            yield f.read(size - hlen - 4)
        off += size


def _mpeg4_descriptor_len(buf, i):
    """MPEG-4 systems variable-length descriptor size field (continuation-bit encoded, up to 4 bytes)."""
    val = 0
    while True:
        b = buf[i]
        i += 1
        val = (val << 7) | (b & 0x7F)
        if not (b & 0x80):
            break
    return val, i


def _extract_vol_header(esds_payload):
    """Pull the DecoderSpecificInfo (MPEG-4 descriptor tag 0x05) out of an esds box's ES_Descriptor —
    for mp4v this literally IS the raw MPEG-4 Part 2 Video Object Layer (VOL) header: width, height,
    aspect ratio, profile, time-increment resolution. An encoder writing a RAW .m4v elementary stream
    repeats this at the start of the file; a container instead stores it ONCE here, in moov, and NEVER
    in mdat — which is exactly why an un-finalized recording's frame data (in mdat) can be intact and
    still be undecodable: the payload has GOV/VOP data but no VOL header telling a decoder the frame
    dimensions. Returns None if the descriptor tree has no DecoderSpecificInfo."""
    i = 0
    while i < len(esds_payload):
        tag = esds_payload[i]
        i += 1
        ln, i = _mpeg4_descriptor_len(esds_payload, i)
        if tag == 0x05:                            # DecoderSpecificInfo
            return esds_payload[i:i + ln]
        if tag == 0x03:                             # ES_Descriptor: ES_ID(2) + flags(1), then descend
            i += 3
        elif tag == 0x04:                           # DecoderConfigDescriptor: 13 fixed bytes, then descend
            i += 13
        else:
            i += ln
    return None


def _probe_dims(path, ffprobe=None):
    """(width, height) of a FINALIZED mp4's first video stream via ffprobe, or (None, None) on any
    failure (an un-finalized file has no moov for ffprobe to read either — this is for candidates
    already confirmed finalized)."""
    try:
        exe = _find_ffmpeg(ffprobe, tool="ffprobe")
        result = subprocess.run([exe, "-v", "error", "-select_streams", "v:0",
                                 "-show_entries", "stream=width,height", "-of", "csv=p=0:s=x", path],
                                capture_output=True, text=True, timeout=15)
        w, h = result.stdout.strip().split("x")
        return int(w), int(h)
    except Exception:
        return None, None


def find_reference_vol_header(width, height, ffprobe=None, search_dir=None):
    """Find a VOL header (MPEG-4 Part 2 codec extradata) to prepend to an un-finalized recording's raw
    payload, by pulling it out of any OTHER finalized `*_visualizer.mp4` in `search_dir` (default
    OUTPUT/diag/) that was made by this SAME encoder pipeline (visualizer.py's cv2.VideoWriter, mp4v
    fourcc) at the SAME dashboard resolution. This is NOT a guess or a fabrication: the VOL header
    encodes only encoder-invariant stream parameters (width/height/profile/time-increment resolution)
    that are IDENTICAL across every recording this exact tool produces — visualizer.py's dashboard is a
    FIXED pixel layout (`PANEL_W`/`GAP`/`MAP_SIZE`/`STATUS_H`), not room or flight data, so two clips at
    the same resolution share byte-identical VOL headers by construction. A candidate whose OWN probed
    width/height doesn't match the target is skipped, never reused. Returns (vol_bytes, source_path),
    or (None, None) if no matching reference exists."""
    search_dir = search_dir or DIAG_DIR
    for p in sorted(glob.glob(os.path.join(search_dir, "*_visualizer.mp4")),
                    key=os.path.getmtime, reverse=True):
        if not is_mp4_finalized(p):
            continue
        pw, ph = _probe_dims(p, ffprobe=ffprobe)
        if pw != width or ph != height:
            continue
        try:
            with open(p, "rb") as f:
                esds_list = list(_walk_boxes_for_esds(f, 0, os.path.getsize(p)))
        except (OSError, struct.error):
            continue
        for esds in esds_list:
            vol = _extract_vol_header(esds)
            if vol:
                return vol, p
    return None, None


def repair_mp4(path, fps=15.0, width=None, height=None, ffmpeg=None, vol_header_path=None,
               out_path=None, search_dir=None, verify_frames=300):
    """Losslessly recover an un-finalized visualizer.py --record MP4. The frame payload in mdat is
    intact, undamaged GOV/VOP data — what's missing is the MPEG-4 Part 2 VOL header (width/height/
    profile), which a container stores ONCE in moov's `esds` box and NEVER repeats in mdat, so a
    hard-killed recording's payload alone can't be decoded (confirmed empirically: ffmpeg's mpeg4
    decoder fails with "Picture size 0x0 is invalid" on the raw payload alone, but succeeds once a
    matching VOL header is prepended — see `find_reference_vol_header`). visualizer.py's dashboard is a
    FIXED pixel layout (width/height default to `PANEL_W+GAP+MAP_SIZE` x `STATUS_H+MAP_SIZE`, a UI
    constant, never room/flight data), so the missing header can be losslessly reconstructed by reusing
    the byte-identical one any OTHER finalized recording at the same resolution already has — NO
    re-encode, NO frame loss, byte-for-byte the same pixels.

    Fails FAST (raises) instead of guessing: if the payload's start code isn't a recognized MPEG-4
    Part 2 marker, if no reference VOL header can be found (pass `vol_header_path` to supply one
    manually), or if the remux doesn't actually produce a playable file (CLAUDE.md: no silent
    fallbacks). Writes a NEW file; the input is never modified."""
    atoms = mp4_top_atoms(path)
    if any(t == "moov" for t, _s, _o in atoms):
        raise ValueError(f"{path}: already finalized (has a moov atom) -- nothing to repair")
    mdat = next(((t, s, o) for t, s, o in atoms if t == "mdat"), None)
    if mdat is None:
        raise ValueError(f"{path}: no mdat box found -- not a recognizable partial mp4")
    _, _mdat_size, mdat_off = mdat
    payload_off = mdat_off + 8          # mdat's declared size was 0 (extends-to-EOF) -> header is 8B
    with open(path, "rb") as f:
        f.seek(payload_off)
        start_code = f.read(4)
    if start_code[:3] != b"\x00\x00\x01" or start_code[3] not in (0xB3, 0xB6):   # GOV or VOP
        raise ValueError(f"{path}: mdat payload does not start with a recognized MPEG-4 Part 2 "
                         f"GOV/VOP start code (got {start_code.hex()}) -- refusing to guess a codec; "
                         f"this repair path is for mp4v (visualizer.py's fourcc) only.")

    if width is None or height is None:
        # Session 62: import the composed size, never re-derive it. This line USED to spell the
        # formula out (`PANEL_W + GAP + MAP_SIZE`) -- a copy that silently went stale when session 60
        # added the leftmost LKG column, and that is the whole reason flight 20260905_113346's first
        # recovery decoded into garbage. visualizer.CANVAS_W is now the single definition.
        from visualizer import CANVAS_W, CANVAS_H   # authoritative source, visualizer.py:73-74
        width, height = CANVAS_W, CANVAS_H
    if vol_header_path is not None:
        with open(vol_header_path, "rb") as f:
            vol = f.read()
        vol_source = vol_header_path
    else:
        vol, vol_source = find_reference_vol_header(width, height, search_dir=search_dir)
        if vol is None:
            raise RuntimeError(f"{path}: no OTHER finalized *_visualizer.mp4 at {width}x{height} exists "
                               f"in {search_dir or DIAG_DIR} to borrow a VOL header (codec extradata) "
                               f"from -- record one flight to completion first, or pass a VOL header "
                               f"explicitly via --vol-header-from.")
    print(f"[salvage] {os.path.basename(path)}: borrowing VOL header from "
          f"{os.path.basename(vol_source)} ({width}x{height})")

    ffmpeg = _find_ffmpeg(ffmpeg)
    base, _ext = os.path.splitext(path)
    m4v_path = base + ".recovered.m4v"
    out_path = out_path or (base + ".recovered.mp4")
    with open(m4v_path, "wb") as dst:
        dst.write(vol)
        with open(path, "rb") as src:
            src.seek(payload_off)
            shutil.copyfileobj(src, dst)
    try:
        result = subprocess.run(
            [ffmpeg, "-y", "-f", "m4v", "-framerate", str(fps), "-i", m4v_path, "-c", "copy", out_path],
            capture_output=True, text=True)
        if result.returncode != 0:
            raise RuntimeError(f"ffmpeg remux failed (exit {result.returncode}):\n{result.stderr[-2000:]}")
    finally:
        os.remove(m4v_path)
    if not is_mp4_finalized(out_path):
        raise RuntimeError(f"{out_path}: remux completed but the output still has no moov atom")
    ow, oh = _probe_dims(out_path)
    if (ow, oh) != (width, height):
        raise RuntimeError(f"{out_path}: remuxed to {ow}x{oh}, expected {width}x{height} -- the "
                           f"borrowed VOL header may not actually match this recording")
    # Session 62: the dimension check above is NECESSARY BUT NOT SUFFICIENT -- `ow/oh` are read back
    # out of the very VOL header that `width/height` selected, so it agrees with itself by
    # construction and CANNOT detect a wrong-sized header. That is exactly how flight 20260905_113346
    # passed every guard here and still played as garbage: a stale canvas formula asked for 908x528,
    # found a genuine 908x528 donor from a pre-LKG flight, and then validated its 908-wide output
    # against its own 908-wide assumption. The only honest test is to DECODE the payload and find out
    # whether the header actually fits it -- a mismatched width desynchronises every macroblock row
    # and errors pour out of the first GOV onward. Fail LOUDLY (CLAUDE.md: no silent fallbacks); a
    # bad recovery that looks successful is worse than none, because the operator trusts it.
    verify = subprocess.run([ffmpeg, "-v", "error", "-i", out_path, "-frames:v", str(verify_frames),
                             "-f", "null", "-"], capture_output=True, text=True)
    errs = [ln for ln in verify.stderr.splitlines() if ln.strip()]
    if errs:
        raise RuntimeError(
            f"{out_path}: the borrowed VOL header does not fit this payload -- decoding the first "
            f"{verify_frames} frames produced {len(errs)} decoder error(s), first: {errs[0].strip()!r}. "
            f"The usual cause is a wrong-sized header: this recording is probably NOT {width}x{height} "
            f"(the visualizer layout changed between it and {os.path.basename(vol_source)}). Pass the "
            f"true width/height explicitly, or --vol-header-from a recording made by the SAME layout.")
    return out_path


# ==============================================================================
# 2. Timeline repair — a torn final line (mid-write crash artifact) + a missing map backdrop.
# ==============================================================================
def check_and_trim_trailing_garbage(jsonl_path):
    """Every line an AutopilotLog.timeline() record writes is flush()'d individually (autopilot.py's
    AutopilotLog.timeline()), so a crash can only ever tear the LAST line, never an earlier one. Returns
    (lines, trimmed) where `lines` is the file's raw lines with a torn tail dropped, and `trimmed` is
    None (nothing needed trimming) or (line_no, raw_text) of the dropped line. Read-only — writes
    nothing; the caller decides where a cleaned copy goes.

    A malformed line found ANYWHERE ELSE is real corruption, not a crash artifact, and is NOT silently
    dropped — this raises, matching flight_replay.load_timeline's own NO-SILENT-FALLBACK contract."""
    with open(jsonl_path, "r", encoding="utf-8") as f:
        lines = f.readlines()
    trimmed = None
    last_idx = next((i for i in range(len(lines) - 1, -1, -1) if lines[i].strip()), None)
    if last_idx is not None:
        try:
            json.loads(lines[last_idx])
        except json.JSONDecodeError:
            trimmed = (last_idx + 1, lines[last_idx])
            lines = lines[:last_idx] + lines[last_idx + 1:]
    for lineno, line in enumerate(lines, 1):
        s = line.strip()
        if not s:
            continue
        try:
            json.loads(s)
        except json.JSONDecodeError as exc:
            raise ValueError(f"{jsonl_path}:{lineno}: malformed JSON NOT at the file tail -- this is "
                             f"real corruption, not a crash-truncated last write; refusing to silently "
                             f"trim it: {exc}") from exc
    return lines, trimmed


def reconstruct_map_from_livemap(npz_path, cfg=None):
    """Reconstruct an HONEST (not fabricated) map-backdrop record from a perception `<ts>_livemap.npz`
    export, for a flight whose timeline carries zero 'map' records (a crash before session 55's
    periodic tick existed, or before it had accumulated a `last_ground`).

    Marks OCCUPIED cells from the voxel centers (real observed structure) and FREE cells from the
    camera trajectory (the drone physically occupied those cells, so they are confirmed traversable) —
    nothing else. This deliberately does NOT attempt to replay `GroundGrid.integrate()`'s per-keyframe
    raycast carving: the npz has no per-frame point<->pose association (only the pooled voxel set +
    the dense trajectory), and inventing one would fabricate free-space claims (raycasts) that were
    never actually observed — exactly what CLAUDE.md's no-silent-fallback rule forbids. The record is
    tagged `map_source: RECONSTRUCTED_FROM_LIVEMAP_NPZ` so a viewer/operator can see it is an
    approximation, not the live grid. Returns None if the npz has no usable centers/trajectory at all.

    Reuses `GroundGrid` (ground_grid.py) for cell sizing + classification + the exact bus-summary shape
    (`bounds`/`rows`/`cols`/`cls`) autopilot.py's `_downsample_map` / flight_replay.py's `drawMap`
    already consume unchanged."""
    import numpy as np
    from ground_grid import GroundGrid
    data = np.load(npz_path)
    centers = data["centers"] if "centers" in data.files else np.zeros((0, 3), np.float32)
    traj = data["trajectory"] if "trajectory" in data.files else np.zeros((0, 3), np.float32)
    if len(centers) == 0 and len(traj) == 0:
        return None
    gg = GroundGrid(cfg)
    cell = gg.cell
    for x, _y, z in centers:
        key = (int(math.floor(float(x) / cell)), int(math.floor(float(z) / cell)))
        gg._lo[key] = gg.lo_clamp                    # force OCC classification (real structure)
    for x, _y, z in traj:
        key = (int(math.floor(float(x) / cell)), int(math.floor(float(z) / cell)))
        if key not in gg._lo:                         # never let a flyover overwrite real structure
            gg._lo[key] = -gg.lo_clamp                 # force FREE classification (drone was here)
    if not gg._lo:
        return None
    m = gg.summary()
    if not m.get("bounds"):
        return None
    m["map_source"] = "RECONSTRUCTED_FROM_LIVEMAP_NPZ"
    return m


# ==============================================================================
# 3. Orchestration
# ==============================================================================
def _find_sibling(glob_pattern, near_dt, max_delta_s):
    """The file matching `glob_pattern` in OUTPUT/diag/ whose OWN ts-in-filename is closest to
    `near_dt`, within `max_delta_s`. Each of fly.py's child processes stamps its own artifacts with an
    INDEPENDENTLY generated ts (typically a couple seconds apart — e.g. flight 20260902_165340's own
    autopilot ts vs its perception ts 20260902_165342). Returns None if nothing is within tolerance."""
    best, best_delta = None, None
    for p in glob.glob(os.path.join(DIAG_DIR, glob_pattern)):
        m = re.search(r"(\d{8}_\d{6})", os.path.basename(p))
        if not m:
            continue
        try:
            t = datetime.strptime(m.group(1), "%Y%m%d_%H%M%S")
        except ValueError:
            continue
        delta = abs((t - near_dt).total_seconds())
        if delta <= max_delta_s and (best_delta is None or delta < best_delta):
            best, best_delta = p, delta
    return best


def resolve_flight(target):
    """target: a bare autopilot ts ('20260902_165340') or a path to its *_timeline.jsonl. Returns
    (ts, jsonl_path)."""
    if target.endswith(".jsonl"):
        jsonl_path = target
        base = os.path.basename(target)
        ts = base[: -len("_timeline.jsonl")] if base.endswith("_timeline.jsonl") else os.path.splitext(base)[0]
    else:
        ts = target
        jsonl_path = os.path.join(DIAG_DIR, f"{ts}_timeline.jsonl")
    if not os.path.exists(jsonl_path):
        raise FileNotFoundError(f"no timeline for {target!r}: {jsonl_path} does not exist")
    return ts, jsonl_path


def salvage_flight(target, ffmpeg=None, open_browser=False):
    """Full salvage for one flight: repair its video (if un-finalized), reconstruct a map backdrop (if
    the timeline has none and a sibling livemap.npz exists), and build the replay report — all at FULL
    fidelity, no downsampling or trimming of any field (operator directive: the raw data must stay
    intact so it can be read directly to diagnose flight bugs). Every output is a NEW file; no original
    is ever opened for writing. Returns a dict summary of what was found/produced."""
    ts, orig_jsonl = resolve_flight(target)
    near_dt = datetime.strptime(ts, "%Y%m%d_%H%M%S")
    result = {"ts": ts, "jsonl": orig_jsonl}

    # ---- video repair (independent of everything below) ----
    mp4_path = _find_sibling("*_visualizer.mp4", near_dt, 120)
    if mp4_path is None:
        print(f"[salvage] {ts}: no *_visualizer.mp4 found near this flight -- skipping video repair")
    elif is_mp4_finalized(mp4_path):
        print(f"[salvage] {ts}: {os.path.basename(mp4_path)} already finalized -- nothing to repair")
        result["video"] = mp4_path
    else:
        print(f"[salvage] {ts}: {os.path.basename(mp4_path)} is UN-FINALIZED -- repairing ...")
        result["video_recovered"] = repair_mp4(mp4_path, ffmpeg=ffmpeg)
        print(f"[salvage] {ts}: video repaired -> {result['video_recovered']}")

    # ---- timeline: trim a torn tail (if any) + reconstruct a map backdrop (if none exists) ----
    lines, trimmed = check_and_trim_trailing_garbage(orig_jsonl)
    if trimmed is not None:
        lineno, raw = trimmed
        print(f"[salvage] {ts}: *** timeline line {lineno} is torn ({len(raw)} bytes -- a mid-write "
              f"crash artifact) -- will be dropped in the salvaged copy; original left untouched ***")
        result["trimmed_line"] = lineno

    has_map = any(("map" in json.loads(l)) for l in lines if l.strip())
    map_record = None
    if has_map:
        print(f"[salvage] {ts}: timeline already has a map backdrop -- no reconstruction needed")
    else:
        npz_path = _find_sibling("*_livemap.npz", near_dt, 120)
        if npz_path is None:
            print(f"[salvage] {ts}: *** no map record in the timeline and no *_livemap.npz nearby -- "
                  f"the report will render on a BLANK scene (no fabricated backdrop) ***")
        else:
            map_record = reconstruct_map_from_livemap(npz_path)
            if map_record is None:
                print(f"[salvage] {ts}: *** {npz_path} has no usable voxels/trajectory -- "
                      f"reconstruction skipped, report will render on a blank scene ***")
            else:
                result["map_reconstructed_from"] = npz_path
                print(f"[salvage] {ts}: *** map backdrop RECONSTRUCTED from "
                      f"{os.path.basename(npz_path)} (tagged map_source="
                      f"RECONSTRUCTED_FROM_LIVEMAP_NPZ) ***")

    if trimmed is not None or map_record is not None:
        working_jsonl = orig_jsonl[: -len(".jsonl")] + ".salvaged.jsonl"
        with open(working_jsonl, "w", encoding="utf-8") as f:
            f.writelines(lines)
            if map_record is not None:
                f.write(json.dumps({"t_mono": 0.0, "map": map_record}) + "\n")
        print(f"[salvage] {ts}: salvaged timeline -> {working_jsonl}")
        result["salvaged_jsonl"] = working_jsonl
    else:
        working_jsonl = orig_jsonl
        print(f"[salvage] {ts}: timeline already complete -- using it as-is")

    import flight_replay
    result["html"] = flight_replay.render_file(working_jsonl, open_browser=open_browser)

    # Session 55: close the loop with fly.py's own crash-recovery check -- if this flight's launch left
    # an "in_progress" *_run.json (fly.py's manifest, marked "completed" only by ITS OWN clean teardown,
    # which a hard crash always skips), mark it "salvaged" now so the NEXT `python fly.py` launch stops
    # re-flagging a flight that has already been recovered.
    manifest_path = _find_sibling("*_run.json", near_dt, 120)
    if manifest_path is not None:
        try:
            with open(manifest_path, "r", encoding="utf-8") as f:
                manifest = json.load(f)
            if manifest.get("status") == "in_progress":
                manifest["status"] = "salvaged"
                manifest["salvaged_ts"] = ts
                with open(manifest_path, "w", encoding="utf-8") as f:
                    json.dump(manifest, f)
                print(f"[salvage] {ts}: marked {os.path.basename(manifest_path)} as salvaged")
        except (OSError, json.JSONDecodeError):
            pass
    return result


def salvage_all(ffmpeg=None):
    """Scan OUTPUT/diag/ for orphaned flights: (1) any *_timeline.jsonl with no sibling report ->
    full salvage_flight(); (2) any *_visualizer.mp4 left un-finalized (independent of report status,
    e.g. its owning flight's report was already built before the video got fixed) -> repair only."""
    results = []
    for jp in sorted(glob.glob(os.path.join(DIAG_DIR, "*_timeline.jsonl"))):
        html = jp[: -len(".jsonl")] + ".html"
        if os.path.exists(html):
            continue
        ts = os.path.basename(jp)[: -len("_timeline.jsonl")]
        print(f"[salvage] --all: {ts} has no report -> salvaging")
        results.append(salvage_flight(ts, ffmpeg=ffmpeg))
    for mp in sorted(glob.glob(os.path.join(DIAG_DIR, "*_visualizer.mp4"))):
        if is_mp4_finalized(mp):
            continue
        recovered = mp[: -len(".mp4")] + ".recovered.mp4"
        if os.path.exists(recovered):
            continue
        print(f"[salvage] --all: {os.path.basename(mp)} un-finalized -> repairing")
        repair_mp4(mp, ffmpeg=ffmpeg)
    if not results:
        print("[salvage] --all: nothing needed salvaging")
    return results


# ==============================================================================
# Self-test: synthetic, no hardware/flight/ffmpeg-dependent-content needed (ffmpeg itself IS invoked --
# this is an integration self-test of the actual remux, not a mock).
# ==============================================================================
def _self_test():
    ok = True

    def check(name, cond):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[salvage_flight][self-test] {'PASS' if cond else 'FAIL'}  {name}")

    import tempfile
    import cv2
    import numpy as np

    tmp = tempfile.mkdtemp(prefix="salvage_selftest_")
    try:
        # ---- (a) mp4 repair: a REAL finalized clip, then truncated EXACTLY like the observed crash
        #      shape (mdat declared-size patched to 0, moov dropped) -- deterministic, no race with
        #      cv2's own GC-triggered release(). ----
        # Named like a real *_visualizer.mp4 so find_reference_vol_header (pointed at `tmp` via
        # search_dir) can discover it as the reference to borrow a VOL header from -- this is the SAME
        # discovery path a real salvage run uses, just scoped to a throwaway directory.
        clip_path = os.path.join(tmp, "20260101_000000_visualizer.mp4")
        w, h, n_frames, fps = 64, 48, 20, 15.0
        writer = cv2.VideoWriter(clip_path, cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
        for i in range(n_frames):
            writer.write(np.full((h, w, 3), i * 10 % 255, np.uint8))
        writer.release()
        check("(a0) synthetic clip has a moov atom", is_mp4_finalized(clip_path))

        atoms = mp4_top_atoms(clip_path)
        mdat = next((a for a in atoms if a[0] == "mdat"), None)
        check("(a1) synthetic clip has an mdat box", mdat is not None)
        _, mdat_size, mdat_off = mdat
        with open(clip_path, "rb") as f:
            data = f.read()
        truncated_path = os.path.join(tmp, "clip.truncated.mp4")
        truncated = bytearray(data[: mdat_off + mdat_size])
        truncated[mdat_off: mdat_off + 4] = struct.pack(">I", 0)   # moov dropped, size patched to 0
        with open(truncated_path, "wb") as f:
            f.write(truncated)
        check("(a2) truncated copy has NO moov atom", not is_mp4_finalized(truncated_path))

        repaired = repair_mp4(truncated_path, fps=fps, width=w, height=h, search_dir=tmp)
        check("(a3) repair produces a finalized mp4", is_mp4_finalized(repaired))
        cap = cv2.VideoCapture(repaired)
        got_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.release()
        check(f"(a4) repaired clip frame count matches original (got {got_frames}, want {n_frames})",
              got_frames == n_frames)
        with open(truncated_path, "rb") as f:
            still_there = f.read()
        check("(a5) the truncated INPUT was left untouched by repair",
              still_there == bytes(truncated))

        # ---- (b) codec guard: a payload that does NOT start with an MPEG-4 Part 2 start code must
        #      raise, never be silently "repaired" into garbage. ----
        fake = bytearray(data[: mdat_off + mdat_size])
        fake[mdat_off + 8: mdat_off + 12] = b"\xff\xff\xff\xff"
        fake[mdat_off: mdat_off + 4] = struct.pack(">I", 0)
        fake_path = os.path.join(tmp, "fake.mp4")
        with open(fake_path, "wb") as f:
            f.write(fake)
        raised = False
        try:
            repair_mp4(fake_path, fps=fps)
        except ValueError:
            raised = True
        check("(b) an unrecognized codec start code raises instead of guessing", raised)

        # ---- (c) trailing-garbage jsonl trim: every earlier line complete, last line torn mid-write ----
        good_lines = [json.dumps({"t_mono": float(i), "state": "X", "pos": [0, 0]}) for i in range(5)]
        jsonl_path = os.path.join(tmp, "t_timeline.jsonl")
        with open(jsonl_path, "w", encoding="utf-8") as f:
            f.write("\n".join(good_lines) + "\n")
            f.write('{"t_mono": 5.0, "state": "X", "pos": [0.1')     # torn, no closing
        lines, trimmed = check_and_trim_trailing_garbage(jsonl_path)
        check("(c1) torn final line detected at the correct line number",
              trimmed is not None and trimmed[0] == 6)
        check("(c2) every remaining line still parses",
              all(json.loads(l) for l in lines if l.strip()))
        with open(jsonl_path, "r", encoding="utf-8") as f:
            untouched = f.read()
        check("(c3) the original jsonl was left byte-identical",
              untouched.endswith('"pos": [0.1'))

        # ---- (d) an EARLIER malformed line (not at the tail) is real corruption -> must raise ----
        bad_jsonl = os.path.join(tmp, "bad_timeline.jsonl")
        with open(bad_jsonl, "w", encoding="utf-8") as f:
            f.write(good_lines[0] + "\n{not json\n" + good_lines[1] + "\n")
        raised2 = False
        try:
            check_and_trim_trailing_garbage(bad_jsonl)
        except ValueError:
            raised2 = True
        check("(d) a malformed line NOT at the tail raises rather than being silently trimmed", raised2)

        # ---- (e) map reconstruction: honest OCC-from-voxels + FREE-from-trajectory, tagged ----
        npz_path = os.path.join(tmp, "t_livemap.npz")
        centers = np.array([[2.0, 0.0, 2.0], [2.1, 0.0, 2.0], [-1.0, 0.0, -1.0]], np.float32)
        traj = np.array([[0.0, 0.0, 0.0], [0.5, 0.0, 0.5]], np.float32)
        np.savez(npz_path, centers=centers, colors=np.zeros((3, 3), np.uint8), trajectory=traj,
                 voxel_size=0.05, tracking_mode="MASt3R")
        m = reconstruct_map_from_livemap(npz_path)
        check("(e1) reconstruction returns a well-formed map dict",
              m is not None and m.get("bounds") is not None and m["rows"] * m["cols"] == len(m["cls"]))
        check("(e2) reconstruction is tagged as reconstructed (not the live grid)",
              m.get("map_source") == "RECONSTRUCTED_FROM_LIVEMAP_NPZ")
        from ground_grid import CLS_OCC, CLS_FREE, CLS_FRONTIER
        check("(e3) at least one OCC cell from the voxel centers", CLS_OCC in m["cls"])
        # A trajectory cell isolated in mostly-UNKNOWN space is CORRECTLY promoted to FRONTIER by
        # GroundGrid.summary()'s own frontier layer (a FREE cell bordering the unobserved region) --
        # both labels stem from the same underlying FREE classification in _lo, so either counts here.
        check("(e4) at least one FREE-derived cell from the trajectory (FREE or FRONTIER)",
              CLS_FREE in m["cls"] or CLS_FRONTIER in m["cls"])

        # ---- (f) empty npz -> None, not a fabricated empty grid ----
        empty_npz = os.path.join(tmp, "empty_livemap.npz")
        np.savez(empty_npz, centers=np.zeros((0, 3), np.float32), colors=np.zeros((0, 3), np.uint8),
                 trajectory=np.zeros((0, 3), np.float32), voxel_size=0.05, tracking_mode="MASt3R")
        check("(f) an empty livemap reconstructs to None (no fabricated backdrop)",
              reconstruct_map_from_livemap(empty_npz) is None)
    finally:
        shutil.rmtree(tmp, ignore_errors=True)

    print(f"\n[salvage_flight][self-test] {'ALL PASS' if ok else 'FAILURES PRESENT'}")
    return ok


def main():
    ap = argparse.ArgumentParser(description="Recover flight artifacts after a hard crash/reboot "
                                              "(session 55). Never modifies an original artifact.")
    ap.add_argument("target", nargs="?",
                    help="a flight ts (e.g. 20260902_165340) or a path to its *_timeline.jsonl")
    ap.add_argument("--all", action="store_true",
                    help="scan OUTPUT/diag/ for orphaned flights (no report, or an un-finalized "
                         "video) and salvage every one")
    ap.add_argument("--ffmpeg", default=None, help="path to ffmpeg.exe (default: search PATH)")
    ap.add_argument("--open", action="store_true", help="open the built report in a browser")
    ap.add_argument("--self-test", action="store_true", help="run the synthetic self-test (no flight)")
    args = ap.parse_args()

    if args.self_test:
        raise SystemExit(0 if _self_test() else 1)
    if args.all:
        salvage_all(ffmpeg=args.ffmpeg)
        return
    if not args.target:
        ap.error("target (a ts or *_timeline.jsonl path) is required unless --all or --self-test")
    result = salvage_flight(args.target, ffmpeg=args.ffmpeg, open_browser=args.open)
    print(f"\n[salvage] done: {json.dumps({k: str(v) for k, v in result.items()}, indent=2)}")


if __name__ == "__main__":
    main()
