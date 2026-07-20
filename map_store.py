"""map_store.py — Milestone 4: fuse SLAM poses + pointmaps into a persistent map.

`MapStore` is the downstream sink the plan calls for: it consumes the per-keyframe
**world-space pointmaps + camera poses** that MASt3R-SLAM produces and accumulates them
into a **sparse voxel/occupancy grid** (`map.voxel_size`, default 5 cm) plus the camera
**trajectory**. From that it renders a top-down (X-Z) occupancy map and can export the
voxel cloud.

Why voxels (not the raw 2 M-point cloud `slam_offline.py` dumps): a streaming live run
re-emits overlapping pointmaps every keyframe, so raw accumulation explodes and double-
counts. Voxel hashing gives a **bounded, deduplicated, multiply-observed** map — and the
per-voxel observation `count` is exactly the confidence signal the M5 opening/glass
analyzers and the Phase-3 report want.

Design boundary (deliberate): this module is **transport-agnostic** — pure numpy in,
numpy/PNG out. No ZMQ, no torch, no SLAM imports. In the live system (M4 step 2) it runs
*in-process inside* `perception_worker` (pointmaps are ~440 K floats/keyframe — far too big
for the JSON state bus), and only the compact rendered map / occupancy summary is published
onward to the visualizer. Keeping it pure also makes it offline-testable against the
`slam_offline.py` `.npz` export (see `__main__`).

NO SILENT FALLBACKS (per CLAUDE.md): inputs are validated and bad shapes raise; there is no
hidden downgrade. `tracking_mode` is carried through and surfaced so a degraded SLAM state
is visible in any map the store renders.
"""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parent


class MapStore:
    """Sparse voxel-occupancy map + camera trajectory, built incrementally.

    Internally the occupancy grid is held in dense, append-only numpy arrays keyed by a
    dict from integer voxel index -> row, so `integrate()` can be called once per keyframe
    in a live loop without re-touching prior voxels. Per voxel we keep an observation
    `count` and a running color sum (mean color read out on demand).
    """

    def __init__(self, voxel_size: float, tracking_mode: str = "MASt3R"):
        assert voxel_size > 0, "voxel_size must be positive"
        self.voxel_size = float(voxel_size)
        self.tracking_mode = tracking_mode

        # voxel index (ix,iy,iz) tuple -> row in the parallel arrays below
        self._row_of: dict[tuple, int] = {}
        self._keys: list[tuple] = []          # row -> (ix,iy,iz)
        self._count = np.zeros(0, np.int64)    # row -> observation count
        self._color_sum = np.zeros((0, 3), np.float64)  # row -> summed RGB
        self._cap = 0                          # allocated capacity of the arrays

        self.trajectory: list[np.ndarray] = []  # camera centers in world coords
        self.n_points_seen = 0                   # raw points integrated (pre-voxelization)

    # ------------------------------------------------------------------ ingest
    def _grow(self, extra: int):
        """Ensure capacity for `extra` more voxels (amortized doubling)."""
        need = len(self._keys) + extra
        if need <= self._cap:
            return
        new_cap = max(need, max(self._cap * 2, 1024))
        c = np.zeros(new_cap, np.int64)
        c[: self._cap] = self._count
        cs = np.zeros((new_cap, 3), np.float64)
        cs[: self._cap] = self._color_sum
        self._count, self._color_sum, self._cap = c, cs, new_cap

    def integrate(self, points_world: np.ndarray, colors: np.ndarray | None = None):
        """Fold one batch of world-space points (N,3) [+ uint8 RGB (N,3)] into the grid.

        Points are voxelized; repeat observations of the same voxel increment its count
        and accumulate color. Returns the number of voxels touched by this batch.
        """
        pts = np.asarray(points_world, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] != 3:
            raise ValueError(f"points_world must be (N,3), got {pts.shape}")
        if len(pts) == 0:
            return 0
        finite = np.isfinite(pts).all(axis=1)
        pts = pts[finite]
        if colors is not None:
            colors = np.asarray(colors)
            if colors.shape[0] != len(finite):
                raise ValueError("colors length must match points length")
            colors = colors[finite].astype(np.float64)
        if len(pts) == 0:
            return 0
        self.n_points_seen += len(pts)

        vidx = np.floor(pts / self.voxel_size).astype(np.int64)
        uniq, inv = np.unique(vidx, axis=0, return_inverse=True)
        batch_count = np.bincount(inv, minlength=len(uniq)).astype(np.int64)
        if colors is not None:
            batch_color = np.stack(
                [np.bincount(inv, weights=colors[:, c], minlength=len(uniq)) for c in range(3)],
                axis=1,
            )
        else:
            batch_color = np.zeros((len(uniq), 3), np.float64)

        self._grow(len(uniq))
        for i in range(len(uniq)):
            key = (int(uniq[i, 0]), int(uniq[i, 1]), int(uniq[i, 2]))
            row = self._row_of.get(key)
            if row is None:
                row = len(self._keys)
                self._row_of[key] = row
                self._keys.append(key)
            self._count[row] += batch_count[i]
            self._color_sum[row] += batch_color[i]
        return len(uniq)

    def add_pose(self, camera_center_world):
        """Append a camera center (world coords, shape (3,)) to the trajectory."""
        c = np.asarray(camera_center_world, dtype=np.float32).reshape(3)
        self.trajectory.append(c)

    # ------------------------------------------------------------------ readout
    def __len__(self):
        return len(self._keys)

    def occupied(self, min_count: int = 1):
        """Return (centers Mx3 float32, colors Mx3 uint8) for voxels seen >= min_count."""
        n = len(self._keys)
        if n == 0:
            return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8)
        keys = np.asarray(self._keys, dtype=np.int64)
        count = self._count[:n]
        keep = count >= min_count
        keys, count = keys[keep], count[keep]
        centers = ((keys + 0.5) * self.voxel_size).astype(np.float32)
        with np.errstate(invalid="ignore"):
            colors = (self._color_sum[:n][keep] / count[:, None]).clip(0, 255).astype(np.uint8)
        return centers, colors

    def raycast(self, origin, direction, max_range: float = 15.0, step_frac: float = 0.5,
                min_count: int = 1, skip: float = 0.0):
        """March a world-space ray through the occupancy grid.

        Returns (hit_center (3,) float32, distance float) for the first voxel seen
        >= `min_count` along the ray, or None if nothing is hit within `max_range`.

        `step_frac` is the march step as a fraction of voxel_size (<= 0.5 avoids tunneling
        through a one-voxel-thick wall); `skip` skips the first `skip` world units (e.g. to
        ignore voxels right at the camera). Uses the occupancy hash for O(1) per-step lookup;
        a ray only does max_range/step lookups so it is cheap at detection cadence.
        """
        origin = np.asarray(origin, np.float64).reshape(3)
        d = np.asarray(direction, np.float64).reshape(3)
        dn = np.linalg.norm(d)
        if dn < 1e-9 or not self._row_of:
            return None
        d = d / dn
        step = self.voxel_size * float(step_frac)
        t = float(skip)
        last_key = None
        while t <= max_range:
            p = origin + d * t
            key = (int(np.floor(p[0] / self.voxel_size)),
                   int(np.floor(p[1] / self.voxel_size)),
                   int(np.floor(p[2] / self.voxel_size)))
            if key != last_key:
                row = self._row_of.get(key)
                if row is not None and self._count[row] >= min_count:
                    center = ((np.asarray(key, np.float64) + 0.5) * self.voxel_size).astype(np.float32)
                    return center, float(t)
                last_key = key
            t += step
        return None

    def clearance(self, origin, heading_deg, fan_deg: float = 15.0, fan_n: int = 3,
                  skip: float = 0.25, min_count: int = 2, max_range: float = 10.0,
                  min_hit_fraction: float = 0.0, detail: bool = False):
        """Forward stand-off distance to the nearest mapped obstacle ahead, for "stop before you ram a
        wall" navigation. Casts a small FAN of GROUND-PLANE rays (Y component zeroed, so they stay at the
        camera's height and read vertical walls, not the floor/ceiling) spread over +/- `fan_deg` around
        `heading_deg`, and returns the NEAREST hit distance (SLAM units), or None if nothing is hit within
        `max_range` (or the map is empty). Heading convention matches `heading_from_pose`: 0 = +Z,
        +90 = +X. Reuses `raycast` (a non-normalized direction is fine).

        `min_hit_fraction` (session 28, default 0.0 = exact prior behavior): a direction only counts as
        BLOCKED once at least this FRACTION of the fan's rays hit something within range — below it, the
        hits are treated as sparse-reconstruction noise and the direction reads OPEN (None). At 0.0 a
        SINGLE ray hit is enough (the original MIN-over-fan design, chosen because it's robust to a
        thin/off-center wall a single ray could otherwise thread between). Raising it trades that
        protection for robustness against the opposite failure: an isolated, spatially-noisy voxel (still
        passing the per-voxel `min_count` observation filter) falsely reading an entire direction as
        blocked. When the direction IS judged blocked, the reported distance is still the MIN (nearest)
        hit among ALL rays that hit — same conservative distance as before, just gated by the vote first.

        `detail` (session 29, default False = exact prior return shape): when True, return a stats dict
        `{"dist", "n_hits", "n_rays", "fraction", "min_dist", "max_dist", "blocked"}` instead of a bare
        float/None — the raw ray-hit picture behind the vote (for the replay debugger's Clearance tab),
        not just its outcome. `dist`/`blocked` are exactly what a `detail=False` call would have returned/
        acted on (`blocked` is `dist is not None`). `n_rays` is `fan_n` even when the map/origin/heading are
        unusable, so a caller always gets a well-formed row; `n_hits`/`fraction`/`min_dist`/`max_dist` are
        computed over EVERY ray that hit within range, independent of the `min_hit_fraction` vote."""
        n = max(int(fan_n), 1)
        if origin is None or heading_deg is None or not self._row_of:
            return ({"dist": None, "n_hits": 0, "n_rays": n, "fraction": 0.0,
                     "min_dist": None, "max_dist": None, "blocked": False} if detail else None)
        h0 = np.radians(float(heading_deg))
        offs = np.zeros(1) if n == 1 else np.linspace(-np.radians(float(fan_deg)),
                                                       np.radians(float(fan_deg)), n)
        hits = []
        for a in offs:
            h = h0 + a
            hit = self.raycast(origin, (float(np.sin(h)), 0.0, float(np.cos(h))),
                               max_range=max_range, min_count=min_count, skip=skip)
            if hit is not None:
                hits.append(hit[1])
        blocked = bool(hits) and len(hits) >= min_hit_fraction * n
        dist = min(hits) if blocked else None
        if not detail:
            return dist
        return {"dist": dist, "n_hits": len(hits), "n_rays": n,
                "fraction": round(len(hits) / n, 3), "min_dist": (min(hits) if hits else None),
                "max_dist": (max(hits) if hits else None), "blocked": blocked}

    def trajectory_array(self):
        return (np.asarray(self.trajectory, dtype=np.float32)
                if self.trajectory else np.zeros((0, 3), np.float32))

    def stats(self, min_count: int = 1):
        centers, _ = self.occupied(min_count)
        n = len(self._keys)
        counts = self._count[:n]
        return {
            "tracking_mode": self.tracking_mode,
            "voxel_size": self.voxel_size,
            "n_points_seen": int(self.n_points_seen),
            "n_voxels": int(n),
            "n_voxels_kept": int(len(centers)),
            "max_obs": int(counts.max()) if n else 0,
            "mean_obs": float(counts.mean()) if n else 0.0,
            "traj_poses": len(self.trajectory),   # per-frame camera centers (dense path)
            "compression_x": round(self.n_points_seen / max(n, 1), 1),
        }

    # ------------------------------------------------------------------ live summary
    def topdown_summary(self, grid: int = 200, pad: float = 0.06, min_count: int = 1):
        """Compact top-down (X-Z) occupancy summary for the live state bus.

        Rasterizes the occupied voxels onto a `grid`x`grid` ground plane using the SAME
        robust 1st/99th-pct bounds + axis convention as `render_topdown` (X right, +Z up),
        and returns ONLY the occupied cells (sparse) plus the trajectory, already in
        pixel row/col space (v=0 at the top, +Z up) so a viewer can plot them directly with
        no knowledge of world coords. Per cell we keep the count-weighted mean color.

        Transport-agnostic: numpy out; the caller serializes for the bus. Each summary is a
        self-contained snapshot of the whole map, so a late-joining subscriber catches up
        fully on the next publish (no incremental state to miss).
        """
        grid = int(grid)
        out = {
            "grid": grid, "tracking_mode": self.tracking_mode,
            "voxel_size": self.voxel_size, "n_voxels_kept": 0,
            "bounds": None, "span_world": 0.0,
            "cells_u": np.zeros(0, np.int32), "cells_v": np.zeros(0, np.int32),
            "cells_rgb": np.zeros((0, 3), np.uint8),
            "traj_u": np.zeros(0, np.int32), "traj_v": np.zeros(0, np.int32),
        }
        n = len(self._keys)
        if n == 0:
            return out
        keys = np.asarray(self._keys, dtype=np.int64)
        count = self._count[:n]
        keep = count >= min_count
        if not keep.any():
            return out
        keys, count = keys[keep], count[keep]
        csum = self._color_sum[:n][keep]
        centers = (keys + 0.5) * self.voxel_size
        color = (csum / count[:, None]).clip(0, 255)  # count-weighted mean RGB (float)

        X, Z = centers[:, 0], centers[:, 2]
        xlo, xhi = np.percentile(X, 1), np.percentile(X, 99)
        zlo, zhi = np.percentile(Z, 1), np.percentile(Z, 99)
        span = max(xhi - xlo, zhi - zlo, 1e-6)
        cx, cz = (xlo + xhi) / 2, (zlo + zhi) / 2
        half = span * (0.5 + pad)
        x0, z0 = cx - half, cz - half
        scale = (grid - 1) / (2 * half)

        def to_cell(x, z):
            u = np.clip((x - x0) * scale, 0, grid - 1).astype(np.int64)
            vraw = np.clip((z - z0) * scale, 0, grid - 1).astype(np.int64)
            return u, (grid - 1) - vraw  # flip so +Z reads "up", matching render_topdown

        u, v = to_cell(X, Z)
        lin = v * grid + u
        uniq, invix = np.unique(lin, return_inverse=True)
        cell_cnt = np.bincount(invix, weights=count, minlength=len(uniq))
        cell_rgb = np.stack(
            [np.bincount(invix, weights=color[:, c] * count, minlength=len(uniq))
             for c in range(3)],
            axis=1,
        ) / cell_cnt[:, None]

        out["cells_v"] = (uniq // grid).astype(np.int32)
        out["cells_u"] = (uniq % grid).astype(np.int32)
        out["cells_rgb"] = cell_rgb.clip(0, 255).astype(np.uint8)
        out["n_voxels_kept"] = int(len(centers))
        out["bounds"] = [float(x0), float(x0 + 2 * half), float(z0), float(z0 + 2 * half)]
        out["span_world"] = float(2 * half)

        traj = self.trajectory_array()
        if len(traj):
            # The trajectory is now per-FRAME (dense), so cap the bus payload by striding to at most
            # TRAJ_MAX points (shape preserved). The full dense path still goes to the .npz export.
            TRAJ_MAX = 1500
            if len(traj) > TRAJ_MAX:
                traj = traj[np.linspace(0, len(traj) - 1, TRAJ_MAX).astype(int)]
            tu, tv = to_cell(traj[:, 0], traj[:, 2])
            out["traj_u"], out["traj_v"] = tu.astype(np.int32), tv.astype(np.int32)
        return out

    # ------------------------------------------------------------------ export
    def save_npz(self, path, min_count: int = 1):
        centers, colors = self.occupied(min_count)
        np.savez(path, centers=centers, colors=colors,
                 trajectory=self.trajectory_array(),
                 voxel_size=self.voxel_size, tracking_mode=self.tracking_mode)

    def save_ply(self, path, min_count: int = 1, trajectory=True, targets=None):
        """Write a viewable ASCII .ply point cloud: the voxel occupancy (true colors), plus the
        flight path as GREEN points and any target instances as large MAGENTA points. Opens in
        MeshLab/CloudCompare. `targets` = optional list of world (3,) points."""
        centers, colors = self.occupied(min_count)
        xyz = [centers.astype(np.float32)]
        rgb = [colors.astype(np.uint8)]
        if trajectory:
            traj = self.trajectory_array()
            if len(traj):
                xyz.append(traj.astype(np.float32))
                rgb.append(np.tile(np.array([0, 255, 0], np.uint8), (len(traj), 1)))   # path = green
        if targets:
            tp = np.asarray(targets, np.float32).reshape(-1, 3)
            xyz.append(tp)
            rgb.append(np.tile(np.array([255, 0, 255], np.uint8), (len(tp), 1)))        # target = magenta
        P = np.concatenate(xyz, axis=0)
        C = np.concatenate(rgb, axis=0)
        with open(path, "w", encoding="utf-8") as f:
            f.write("ply\nformat ascii 1.0\n")
            f.write(f"element vertex {len(P)}\n")
            f.write("property float x\nproperty float y\nproperty float z\n")
            f.write("property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
            for (x, y, z), (r, g, b) in zip(P, C):
                f.write(f"{x:.4f} {y:.4f} {z:.4f} {int(r)} {int(g)} {int(b)}\n")

    def render_topdown(self, out_path=None, size=900, pad=0.06, min_count: int = 1,
                       point_px: int = 1, targets=None):
        """Render an X-Z (ground-plane) top-down occupancy map with the trajectory.

        Camera convention: X right, Y down, Z forward => X-Z is the horizontal plane.
        Robust 1st/99th-percentile bounds keep outliers from squashing the view. Returns
        the rendered BGR image (and writes it if `out_path` is given). `targets` is an optional
        list of world (3,) points (e.g. the estimated target) drawn as labeled markers.
        """
        centers, colors = self.occupied(min_count)
        traj = self.trajectory_array()
        img = np.full((size, size, 3), 18, np.uint8)
        if len(centers) == 0:
            if out_path:
                cv2.imwrite(str(out_path), img)
            return img

        X, Z = centers[:, 0], centers[:, 2]
        xlo, xhi = np.percentile(X, 1), np.percentile(X, 99)
        zlo, zhi = np.percentile(Z, 1), np.percentile(Z, 99)
        span = max(xhi - xlo, zhi - zlo, 1e-6)
        cx, cz = (xlo + xhi) / 2, (zlo + zhi) / 2
        half = span * (0.5 + pad)

        def to_px(x, z):
            u = (x - (cx - half)) / (2 * half) * (size - 1)
            v = (z - (cz - half)) / (2 * half) * (size - 1)
            return np.clip(u, 0, size - 1).astype(int), np.clip(v, 0, size - 1).astype(int)

        u, v = to_px(X, Z)
        vy = size - 1 - v  # flip so +Z reads "up"
        bgr = colors[:, ::-1]
        if point_px <= 1:
            img[vy, u] = bgr
        else:
            r = point_px // 2
            for du in range(-r, r + 1):
                for dv in range(-r, r + 1):
                    uu = np.clip(u + du, 0, size - 1)
                    vv = np.clip(vy + dv, 0, size - 1)
                    img[vv, uu] = bgr

        if len(traj) > 1:
            tu, tv = to_px(traj[:, 0], traj[:, 2])
            path = np.stack([tu, size - 1 - tv], axis=1).astype(np.int32)
            cv2.polylines(img, [path], False, (0, 0, 255), 2, cv2.LINE_AA)
            cv2.circle(img, tuple(path[0]), 6, (0, 255, 0), -1)    # start
            cv2.circle(img, tuple(path[-1]), 6, (0, 255, 255), -1)  # end

        if targets:
            multi = len(targets) > 1
            for i, tw in enumerate(targets):
                tw = np.asarray(tw, np.float64).reshape(3)
                tu, tv = to_px(np.array([tw[0]]), np.array([tw[2]]))
                px, py = int(tu[0]), int(size - 1 - int(tv[0]))
                cv2.drawMarker(img, (px, py), (255, 0, 255), cv2.MARKER_TILTED_CROSS, 22, 2)
                cv2.circle(img, (px, py), 11, (255, 0, 255), 2)
                cv2.putText(img, f"TARGET {i}" if multi else "TARGET", (px + 14, py - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        cv2.putText(img, f"top-down X-Z  {len(centers)} voxels @ {self.voxel_size:g}u  "
                    f"{len(traj)} traj-pts  mode={self.tracking_mode}  ~{2*half:.2f}u across",
                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(img, "traj: green=start yellow=end (red path)", (10, size - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1)
        if out_path:
            cv2.imwrite(str(out_path), img)
        return img


# ==============================================================================
# Self-test: synthetic voxels (no SLAM, no hardware) — clearance()'s min_hit_fraction vote
# (session 28).
# ==============================================================================
def run_self_test():
    ok = True

    def check(name, cond):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[map_store][self-test] {'PASS' if cond else 'FAIL'}  {name}")

    origin = (0.0, 0.0, 0.0)
    # A wide, few-ray fan so each ray's straight-line path is unambiguous: offs = [-40,-20,0,20,40] deg
    # around heading_deg=0 (+Z). A point placed exactly ON one ray's axis is hit ONLY by that ray -- the
    # others diverge by >0.7u at these ranges, nowhere near a single 0.1u voxel.
    fan_kw = dict(heading_deg=0.0, fan_deg=40.0, fan_n=5, skip=0.0, min_count=2, max_range=5.0)

    def _add_twice(store, pt):
        store.integrate(np.array([pt], np.float64))
        store.integrate(np.array([pt], np.float64))   # 2nd observation -> passes min_count=2

    # (a) ONE isolated (but min_count-qualified) hit, on-axis at Z=2.0 -> only 1/5 rays (20%) confirm it.
    #     min_hit_fraction=0.3 (>=1.5 hits needed) -> too few -> treated as OPEN (None), not a false block.
    sa = MapStore(voxel_size=0.1)
    _add_twice(sa, (0.0, 0.0, 2.0))
    isolated_ignored = sa.clearance(origin, min_hit_fraction=0.3, **fan_kw) is None
    check("(a) isolated single-ray hit below min_hit_fraction -> ignored (open)", isolated_ignored)

    # (b) a SECOND hit along the +20deg ray, CLOSER (t=1.5) -> 2/5 rays (40%) >= 0.3 -> blocked, and the
    #     reported distance is still the MIN across all confirming hits (1.5, not the on-axis 2.0).
    sb = MapStore(voxel_size=0.1)
    _add_twice(sb, (0.0, 0.0, 2.0))                          # on-axis (0 deg), dist 2.0
    h20 = np.radians(20.0)
    _add_twice(sb, (1.5 * np.sin(h20), 0.0, 1.5 * np.cos(h20)))  # +20 deg ray, dist 1.5
    d = sb.clearance(origin, min_hit_fraction=0.3, **fan_kw)
    enough_hits_blocks = d is not None and abs(d - 1.5) < 0.05
    check(f"(b) 2/5 rays hit -> blocked, MIN distance reported (dist={d})", enough_hits_blocks)

    # (c) min_hit_fraction=0.0 (the default) on the SAME single-hit setup from (a) -> unchanged prior
    #     behavior: a single ray hit is still enough to call it blocked (regression guard).
    default_unchanged = sa.clearance(origin, min_hit_fraction=0.0, **fan_kw) is not None
    check("(c) default min_hit_fraction=0.0 -> single-hit-blocks behavior unchanged", default_unchanged)
    # same check with the parameter omitted entirely (its own default)
    default_omitted = sa.clearance(origin, **fan_kw) is not None
    check("(c2) parameter omitted -> same as 0.0 (single hit still blocks)", default_omitted)

    # (d) session 29: detail=True on the (a) setup (isolated 1/5-ray hit, min_hit_fraction=0.3) -> a
    #     well-formed stats dict that reports the RAW ray picture (n_hits=1, fraction=0.2) even though the
    #     vote judges it OPEN (blocked=False, dist=None) -- the tab shows why, not just the outcome.
    da = sa.clearance(origin, min_hit_fraction=0.3, detail=True, **fan_kw)
    detail_open = (isinstance(da, dict) and da["blocked"] is False and da["dist"] is None
                  and da["n_hits"] == 1 and da["n_rays"] == 5 and abs(da["fraction"] - 0.2) < 1e-9
                  and abs(da["min_dist"] - 2.0) < 0.05 and abs(da["max_dist"] - 2.0) < 0.05)
    check(f"(d) detail=True on a below-vote hit -> raw stats + blocked=False ({da})", detail_open)

    # (e) detail=True on the (b) setup (2/5 rays hit, distances 2.0 and 1.5) -> blocked=True, dist is the
    #     MIN (1.5), but min_dist/max_dist still span BOTH hits (1.5 and 2.0).
    db = sb.clearance(origin, min_hit_fraction=0.3, detail=True, **fan_kw)
    detail_blocked = (isinstance(db, dict) and db["blocked"] is True and abs(db["dist"] - 1.5) < 0.05
                      and db["n_hits"] == 2 and db["n_rays"] == 5 and abs(db["fraction"] - 0.4) < 1e-9
                      and abs(db["min_dist"] - 1.5) < 0.05 and abs(db["max_dist"] - 2.0) < 0.05)
    check(f"(e) detail=True on a confirmed block -> raw stats span all hits ({db})", detail_blocked)

    # (f) detail=True on an empty/unusable map (no voxels at all) -> a well-formed all-zero row, not None.
    se = MapStore(voxel_size=0.1)
    df = se.clearance(origin, min_hit_fraction=0.3, detail=True, **fan_kw)
    detail_empty_ok = (isinstance(df, dict) and df["blocked"] is False and df["dist"] is None
                       and df["n_hits"] == 0 and df["n_rays"] == 5 and df["fraction"] == 0.0
                       and df["min_dist"] is None and df["max_dist"] is None)
    check(f"(f) detail=True on an empty map -> well-formed all-zero row, not None ({df})", detail_empty_ok)

    print(f"\n[map_store][self-test] {'ALL PASS' if ok else 'FAILURES PRESENT'}")
    return ok


# ==============================================================================
# Offline validation: rebuild the voxel map from a slam_offline .npz export.
#
# This proves the fusion + render path against the *same* 2.08 M-point cloud the offline
# SLAM run produced, with no hardware/SLAM needed. `--chunks` splits the cloud and feeds
# it through integrate() in pieces to exercise the incremental (streaming) path exactly
# as the live keyframe loop will.
# ==============================================================================
def main():
    ap = argparse.ArgumentParser(description="Offline: build a voxel MapStore from a SLAM .npz")
    ap.add_argument("--npz", default=str(REPO / "OUTPUT" / "flight_20260621_120829_map.npz"),
                    help="slam_offline export with points/colors/trajectory")
    ap.add_argument("--voxel-size", type=float, default=None,
                    help="override map.voxel_size (default: read config.yaml, fallback 0.05)")
    ap.add_argument("--min-count", type=int, default=2,
                    help="drop voxels seen fewer than this many times (denoise)")
    ap.add_argument("--chunks", type=int, default=8,
                    help="split the cloud into N batches to simulate streaming keyframes")
    ap.add_argument("--out", default=None, help="output basename (default: <npz stem>_voxmap)")
    ap.add_argument("--self-test", action="store_true", help="run the synthetic self-test (no hardware)")
    args = ap.parse_args()

    if args.self_test:
        raise SystemExit(0 if run_self_test() else 1)

    voxel_size = args.voxel_size
    if voxel_size is None:
        try:
            import yaml
            with open(REPO / "config.yaml", "r", encoding="utf-8") as f:
                voxel_size = float(yaml.safe_load(f)["map"]["voxel_size"])
        except Exception:
            voxel_size = 0.05

    npz_path = Path(args.npz)
    assert npz_path.exists(), f"npz not found: {npz_path}"
    data = np.load(npz_path)
    points, colors = data["points"], data["colors"]
    traj = data["trajectory"]
    print(f"[map_store] loaded {npz_path.name}: {len(points)} pts, {len(traj)} kf, "
          f"voxel_size={voxel_size:g}")

    store = MapStore(voxel_size)
    t0 = time.time()
    splits = np.array_split(np.arange(len(points)), max(args.chunks, 1))
    for i, idx in enumerate(splits):
        touched = store.integrate(points[idx], colors[idx])
        print(f"[map_store]   chunk {i+1}/{len(splits)}: +{len(idx)} pts -> "
              f"{touched} voxels touched, {len(store)} total")
    for c in traj:
        store.add_pose(c)
    dt = time.time() - t0

    s = store.stats(min_count=args.min_count)
    print(f"[map_store] built in {dt:.1f}s | {s}")

    out_base = args.out or str(npz_path.with_name(npz_path.stem.replace("_map", "") + "_voxmap"))
    png = out_base + "_topdown.png"
    store.render_topdown(png, min_count=args.min_count)
    print(f"[map_store] top-down -> {png}")
    store.save_npz(out_base + ".npz", min_count=args.min_count)
    print(f"[map_store] voxel map -> {out_base}.npz")
    print("[map_store] OK")


if __name__ == "__main__":
    main()
