#!/usr/bin/env python
"""backend_window_bench.py -- Session 65 (C7): does bounding the global-optimisation window degrade
trajectory consistency?

`plans/session65-spec.md`'s outcome for this session is a config-gated bounded window PLUS an offline
bench answering the report's own question (`plans/slam_report.html`): does `backend_ms` flatten
against keyframe count while trajectory consistency holds? This module is the "holds" half -- it
turns two `*_livemap.npz` exports (written by `MapStore.save_npz`, read via
`perception_worker.py --video ... --log`, one per `backend_window_mode`) into numbers: trajectory
RMSE/max-deviation and voxel-occupancy IoU.

Pure NumPy + stdlib -- no torch, no project import -- so it runs standalone on any machine that can
read a `.npz`, exactly like `perception_timing_report.py` stays import-clean of torch for the CSV
side of the same bench.

**No Sim3 alignment, and this is deliberate, not an oversight.** Two runs being compared are the SAME
recorded video at the SAME `--stride`/`--max-frames`, so they produce the same frame count in the same
order -- there is no independent camera trajectory to align, only two estimates of one. And
`num_fix = 1` (`gn_kernels.cu:1155`) pins the SAME oldest keyframe's pose in every solve of every run,
windowed or not (see `slam_window.py`'s module docstring: the pin always lands on the lowest sorted
index, which for `OFF` is keyframe 0). Both runs therefore already share one common, un-drifted anchor
pose -- the one thing a similarity alignment would otherwise be solving for -- so aligning would only
hide a real divergence behind a fitted rotation/scale, not remove a spurious one.
"""

import argparse
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent

_REQUIRED_KEYS = ("trajectory", "centers", "voxel_size")


def load_run(npz_path) -> dict:
    """Load one `*_livemap.npz` export (`MapStore.save_npz`) into a plain dict.

    Returns {"trajectory": (N,3) float32, "centers": (M,3) float32, "voxel_size": float,
    "path": str}. `np.load` on a nonexistent file already raises `FileNotFoundError` with the path in
    it, so that case needs no extra handling here. A required key missing from the archive raises
    `KeyError` naming BOTH the key and the file -- the bare `KeyError(key)` a raw `NpzFile[key]` access
    would raise doesn't say which of possibly two files being compared is the broken one.
    """
    path = str(npz_path)
    with np.load(path) as data:
        missing = [k for k in _REQUIRED_KEYS if k not in data.files]
        if missing:
            raise KeyError(f"{path}: missing key(s) {missing}")
        return {
            "trajectory": np.asarray(data["trajectory"], dtype=np.float32),
            "centers": np.asarray(data["centers"], dtype=np.float32),
            "voxel_size": float(data["voxel_size"]),
            "path": path,
        }


def _per_pose_distance(a, b):
    """Euclidean distance per row of two (N,3) arrays, float64 throughout for a stable RMSE."""
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)
    if len(a) != len(b):
        raise ValueError(f"trajectory length mismatch: {len(a)} vs {len(b)}")
    if len(a) == 0:
        return np.zeros(0, dtype=np.float64)
    return np.linalg.norm(a - b, axis=1)


def trajectory_rmse(a, b) -> tuple:
    """RMSE of per-pose Euclidean distance between two (N,3) trajectories, and N.

    Raises ValueError naming BOTH lengths on a mismatch -- two runs of the same video at the same
    `--stride`/`--max-frames` must produce the same frame count; a mismatch means the bench was run
    wrong (different video, different stride), and silently truncating to the shorter one would hide
    exactly that mistake."""
    d = _per_pose_distance(a, b)
    if d.size == 0:
        return 0.0, 0
    return float(np.sqrt(np.mean(d ** 2))), int(d.size)


def occupancy_overlap(a_centers, b_centers, voxel_size: float) -> tuple:
    """Voxel-occupancy IoU between two point sets, at integer keys quantised by `voxel_size`.

    Returns (IoU, n_a, n_b) where n_a/n_b are the number of DISTINCT occupied voxels in each set
    (not the raw point count -- `MapStore.occupied()` already de-duplicates to one center per voxel,
    but this function re-quantises independently so it works on any (M,3) point array, not just a
    MapStore export). Both sets empty -> IoU 1.0 (no disagreement is possible); exactly one empty ->
    IoU 0.0. `voxel_size <= 0` raises ValueError -- a zero or negative bucket size would either divide
    by zero or silently invert every comparison."""
    if voxel_size <= 0:
        raise ValueError(f"voxel_size must be > 0, got {voxel_size!r}")

    def _keys(centers):
        centers = np.asarray(centers, dtype=np.float64)
        if centers.size == 0:
            return set()
        idx = np.round(centers / voxel_size).astype(np.int64)
        return set(map(tuple, idx.tolist()))

    a_keys, b_keys = _keys(a_centers), _keys(b_centers)
    n_a, n_b = len(a_keys), len(b_keys)
    union = a_keys | b_keys
    if not union:
        return 1.0, n_a, n_b
    inter = a_keys & b_keys
    return len(inter) / len(union), n_a, n_b


def compare(baseline_npz, candidate_npz) -> str:
    """Multi-line human-readable comparison of two `*_livemap.npz` exports: pose/voxel counts,
    trajectory RMSE + max per-pose deviation, and occupancy IoU. No Sim3 alignment -- see the module
    docstring for why that is unnecessary here, not merely skipped. Raises ValueError if the two
    runs' `voxel_size` differ: `occupancy_overlap`'s quantisation only means the same thing when both
    sides used the same bucket size, and a silent mismatch would produce a number that looks like an
    IoU but isn't one."""
    base = load_run(baseline_npz)
    cand = load_run(candidate_npz)
    if base["voxel_size"] != cand["voxel_size"]:
        raise ValueError(
            f"voxel_size mismatch: {base['path']}={base['voxel_size']!r} vs "
            f"{cand['path']}={cand['voxel_size']!r} -- IoU is not comparable across bucket sizes")

    rmse, n = trajectory_rmse(base["trajectory"], cand["trajectory"])
    d = _per_pose_distance(base["trajectory"], cand["trajectory"])
    max_dev = float(d.max()) if d.size else 0.0
    iou, n_a, n_b = occupancy_overlap(base["centers"], cand["centers"], base["voxel_size"])

    lines = [
        f"baseline:  {base['path']}  ({len(base['trajectory'])} poses, {n_a} voxels)",
        f"candidate: {cand['path']}  ({len(cand['trajectory'])} poses, {n_b} voxels)",
        f"trajectory RMSE: {rmse:.6f} over {n} poses  (max per-pose deviation: {max_dev:.6f})",
        f"occupancy IoU: {iou:.4f}  (baseline {n_a} voxels, candidate {n_b} voxels, "
        f"voxel_size={base['voxel_size']:g})",
    ]
    return "\n".join(lines)


def run_self_test() -> None:
    import shutil
    import tempfile

    ok = True

    def check(name, cond):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[bench][self-test] {'PASS' if cond else 'FAIL'}  {name}")

    tmp_dir = tempfile.mkdtemp(prefix="backend_window_bench_selftest_")
    try:
        def write_npz(path, trajectory, centers, voxel_size=0.1, tracking_mode="MASt3R"):
            colors = np.zeros((len(centers), 3), dtype=np.uint8)
            np.savez(path, centers=np.asarray(centers, dtype=np.float32),
                      colors=colors, trajectory=np.asarray(trajectory, dtype=np.float32),
                      voxel_size=voxel_size, tracking_mode=tracking_mode)

        # 1. Self-compare: RMSE exactly 0.0, IoU exactly 1.0.
        traj_a = np.array([[0.0, 0.0, 0.0], [1.0, 1.0, 1.0], [2.0, 2.0, 2.0]], dtype=np.float32)
        centers_a = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], dtype=np.float32)
        path_a = Path(tmp_dir) / "a.npz"
        write_npz(path_a, traj_a, centers_a)

        run_a = load_run(path_a)
        check("load_run -- round-trips trajectory/centers/voxel_size/path",
              run_a["trajectory"].shape == (3, 3) and run_a["centers"].shape == (3, 3)
              and run_a["voxel_size"] == 0.1 and run_a["path"] == str(path_a))

        rmse_self, n_self = trajectory_rmse(run_a["trajectory"], run_a["trajectory"])
        check("trajectory_rmse -- self-compare gives exactly 0.0", rmse_self == 0.0 and n_self == 3)
        iou_self, na_self, nb_self = occupancy_overlap(run_a["centers"], run_a["centers"],
                                                         run_a["voxel_size"])
        check("occupancy_overlap -- self-compare gives exactly 1.0", iou_self == 1.0
              and na_self == 3 and nb_self == 3)

        report_self = compare(path_a, path_a)
        check("compare -- self-compare report shows RMSE 0.000000 and IoU 1.0000",
              "RMSE: 0.000000" in report_self and "IoU: 1.0000" in report_self)

        # 2. Constant offset: RMSE equals that constant within 1e-6.
        offset = 1.0
        traj_b = traj_a + np.array([offset, 0.0, 0.0], dtype=np.float32)
        path_b = Path(tmp_dir) / "b.npz"
        write_npz(path_b, traj_b, centers_a)
        rmse_off, n_off = trajectory_rmse(traj_a, traj_b)
        check(f"trajectory_rmse -- constant offset {offset} -> RMSE matches within 1e-6 "
              f"(got {rmse_off!r})", abs(rmse_off - offset) < 1e-6 and n_off == 3)

        # 3. Differing lengths -> ValueError naming both lengths.
        raised_len = None
        try:
            trajectory_rmse(np.zeros((3, 3)), np.zeros((5, 3)))
        except ValueError as e:
            raised_len = str(e)
        check(f"trajectory_rmse -- length mismatch raises ValueError naming both lengths "
              f"(got {raised_len!r})",
              raised_len is not None and "3" in raised_len and "5" in raised_len)

        # 4. Disjoint voxel sets -> IoU 0.0; a strict subset -> the exact expected IoU.
        disjoint_a = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32)
        disjoint_b = np.array([[10.0, 0.0, 0.0], [11.0, 0.0, 0.0]], dtype=np.float32)
        iou_disjoint, _, _ = occupancy_overlap(disjoint_a, disjoint_b, 1.0)
        check(f"occupancy_overlap -- disjoint sets -> IoU 0.0 (got {iou_disjoint!r})",
              iou_disjoint == 0.0)

        superset = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]], dtype=np.float32)
        subset = np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]], dtype=np.float32)
        iou_subset, n_super, n_sub = occupancy_overlap(superset, subset, 1.0)
        check(f"occupancy_overlap -- strict subset -> IoU == n_sub/n_super == 2/3 "
              f"(got {iou_subset!r})", abs(iou_subset - (2.0 / 3.0)) < 1e-9
              and n_super == 3 and n_sub == 2)

        # 5. voxel_size <= 0 -> ValueError.
        raised_vs = False
        try:
            occupancy_overlap(disjoint_a, disjoint_b, 0.0)
        except ValueError:
            raised_vs = True
        check("occupancy_overlap -- voxel_size <= 0 raises ValueError", raised_vs)

        # 6. Mismatched voxel_size between two runs -> ValueError, via compare().
        path_c = Path(tmp_dir) / "c.npz"
        write_npz(path_c, traj_a, centers_a, voxel_size=0.2)
        raised_mismatch = False
        try:
            compare(path_a, path_c)
        except ValueError:
            raised_mismatch = True
        check("compare -- mismatched voxel_size raises ValueError", raised_mismatch)

        # 7. A missing trajectory key -> KeyError naming the key and the file.
        path_missing = Path(tmp_dir) / "missing_traj.npz"
        np.savez(path_missing, centers=centers_a, voxel_size=0.1)
        raised_key = None
        try:
            load_run(path_missing)
        except KeyError as e:
            raised_key = str(e)
        # KeyError's str() is repr(args[0]), which re-escapes backslashes -- match the filename
        # rather than the raw Windows path so this doesn't depend on that escaping.
        check(f"load_run -- missing trajectory key raises KeyError naming the key and the file "
              f"(got {raised_key!r})",
              raised_key is not None and "trajectory" in raised_key and path_missing.name in raised_key)

        # 8. A nonexistent file -> FileNotFoundError (np.load's own, not swallowed/reinterpreted).
        raised_fnf = False
        try:
            load_run(Path(tmp_dir) / "does_not_exist.npz")
        except FileNotFoundError:
            raised_fnf = True
        check("load_run -- nonexistent file raises FileNotFoundError", raised_fnf)
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    print(f"\n[self-test] {'ALL PASS' if ok else 'FAILURES PRESENT'}")
    if not ok:
        sys.exit(1)


def main() -> None:
    ap = argparse.ArgumentParser(
        description="Session 65 (C7) -- compare two offline SLAM runs' trajectory + occupancy "
                     "(*_livemap.npz exports from perception_worker.py --video ... --log).")
    ap.add_argument("baseline", nargs="?", default=None, help="baseline *_livemap.npz")
    ap.add_argument("candidate", nargs="?", default=None, help="candidate *_livemap.npz")
    ap.add_argument("--self-test", action="store_true", help="run the synthetic self-test")
    args = ap.parse_args()

    if args.self_test:
        run_self_test()
        return

    if args.baseline is None or args.candidate is None:
        ap.error("baseline and candidate are required unless --self-test is given")

    print(compare(args.baseline, args.candidate))


if __name__ == "__main__":
    main()
