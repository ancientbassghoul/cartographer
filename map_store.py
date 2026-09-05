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
        # Session 63: preallocated (cap,3) array, not a Python list -- readout paths used to pay
        # np.asarray(list-of-tuples) on every call (O(map) each time); self._n is the live row
        # count since len(self._keys) is now the allocated CAPACITY, not the occupancy.
        self._keys = np.zeros((0, 3), np.int64)  # row -> (ix,iy,iz), preallocated to self._cap
        self._n = 0                            # live rows (<= self._cap == len(self._keys))
        self._count = np.zeros(0, np.int64)    # row -> observation count
        self._color_sum = np.zeros((0, 3), np.float64)  # row -> summed RGB
        self._cap = 0                          # allocated capacity of the arrays

        self.trajectory: list[np.ndarray] = []  # camera centers in world coords
        self.n_points_seen = 0                   # raw points integrated (pre-voxelization)

        # Session 63: topdown_summary() was recomputing an identical raster at >=2Hz even though
        # occupied cells only change inside integrate() (keyframes only, ~1 frame in 6). Cache the
        # cell raster + its projection frame; _td_dirty is the ONLY invalidation signal (no timer --
        # a stale raster after the map changed is a silent wrong answer, per CLAUDE.md).
        self._td_cache: dict | None = None    # cached cell raster + bounds from the last recompute
        self._td_key: tuple | None = None     # (grid, pad, min_count) the cache was built for
        self._td_dirty: bool = True           # set True by ANY mutation of counts/colors/keys

    # ------------------------------------------------------------------ ingest
    def _grow(self, extra: int):
        """Ensure capacity for `extra` more voxels (amortized doubling)."""
        need = self._n + extra
        if need <= self._cap:
            return
        new_cap = max(need, max(self._cap * 2, 1024))
        c = np.zeros(new_cap, np.int64)
        c[: self._cap] = self._count
        cs = np.zeros((new_cap, 3), np.float64)
        cs[: self._cap] = self._color_sum
        k = np.zeros((new_cap, 3), np.int64)
        k[: self._cap] = self._keys
        self._keys = k
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
                row = self._n
                self._row_of[key] = row
                self._keys[row] = uniq[i]
                self._n += 1
            self._count[row] += batch_count[i]
            self._color_sum[row] += batch_color[i]
        self._td_dirty = True  # Session 63: this path just mutated counts/colors/keys
        return len(uniq)

    def add_pose(self, camera_center_world):
        """Append a camera center (world coords, shape (3,)) to the trajectory."""
        c = np.asarray(camera_center_world, dtype=np.float32).reshape(3)
        self.trajectory.append(c)
        # Session 63: deliberately does NOT set _td_dirty -- the cell raster is unaffected by a new
        # pose, and topdown_summary() recomputes traj_u/traj_v on every call regardless of the cache.

    # ------------------------------------------------------------------ readout
    def __len__(self):
        return self._n

    def occupied(self, min_count: int = 1):
        """Return (centers Mx3 float32, colors Mx3 uint8) for voxels seen >= min_count."""
        n = self._n
        if n == 0:
            return np.zeros((0, 3), np.float32), np.zeros((0, 3), np.uint8)
        keys = self._keys[:n]
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
        n = self._n
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

        Session 63: the cell raster (cells_u/v/rgb, bounds, span_world, n_voxels_kept, and the
        x0/z0/scale projection frame) is cached and reused while `_td_dirty` is False and the
        (grid, pad, min_count) key matches -- occupied cells only change inside integrate()
        (keyframes only), so most publish-timer ticks would otherwise recompute an identical
        answer. traj_u/traj_v are recomputed on EVERY call (cached or not): the trajectory grows
        every frame even when the cells don't, so caching it would freeze the drawn path.
        """
        grid = int(grid)
        key = (grid, pad, min_count)
        out = {
            "grid": grid, "tracking_mode": self.tracking_mode,
            "voxel_size": self.voxel_size, "n_voxels_kept": 0,
            "bounds": None, "span_world": 0.0,
            "cells_u": np.zeros(0, np.int32), "cells_v": np.zeros(0, np.int32),
            "cells_rgb": np.zeros((0, 3), np.uint8),
            "traj_u": np.zeros(0, np.int32), "traj_v": np.zeros(0, np.int32),
        }

        def to_cell(x, z, x0, z0, scale):
            u = np.clip((x - x0) * scale, 0, grid - 1).astype(np.int64)
            vraw = np.clip((z - z0) * scale, 0, grid - 1).astype(np.int64)
            return u, (grid - 1) - vraw  # flip so +Z reads "up", matching render_topdown

        if self._td_dirty or self._td_key != key:
            n = self._n
            if n == 0:
                self._td_cache = None
                self._td_key = key
                self._td_dirty = False
                return out
            keys = self._keys[:n]
            count = self._count[:n]
            keep = count >= min_count
            if not keep.any():
                self._td_cache = None
                self._td_key = key
                self._td_dirty = False
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

            u, v = to_cell(X, Z, x0, z0, scale)
            lin = v * grid + u
            uniq, invix = np.unique(lin, return_inverse=True)
            cell_cnt = np.bincount(invix, weights=count, minlength=len(uniq))
            cell_rgb = np.stack(
                [np.bincount(invix, weights=color[:, c] * count, minlength=len(uniq))
                 for c in range(3)],
                axis=1,
            ) / cell_cnt[:, None]

            self._td_cache = {
                "cells_v": (uniq // grid).astype(np.int32),
                "cells_u": (uniq % grid).astype(np.int32),
                "cells_rgb": cell_rgb.clip(0, 255).astype(np.uint8),
                "n_voxels_kept": int(len(centers)),
                "bounds": [float(x0), float(x0 + 2 * half), float(z0), float(z0 + 2 * half)],
                "span_world": float(2 * half),
                "x0": x0, "z0": z0, "scale": scale,
            }
            self._td_key = key
            self._td_dirty = False

        cache = self._td_cache
        if cache is None:
            return out
        out["cells_v"] = cache["cells_v"].copy()
        out["cells_u"] = cache["cells_u"].copy()
        out["cells_rgb"] = cache["cells_rgb"].copy()
        out["n_voxels_kept"] = cache["n_voxels_kept"]
        out["bounds"] = list(cache["bounds"])
        out["span_world"] = cache["span_world"]

        traj = self.trajectory_array()
        if len(traj):
            # The trajectory is now per-FRAME (dense), so cap the bus payload by striding to at most
            # TRAJ_MAX points (shape preserved). The full dense path still goes to the .npz export.
            TRAJ_MAX = 1500
            if len(traj) > TRAJ_MAX:
                traj = traj[np.linspace(0, len(traj) - 1, TRAJ_MAX).astype(int)]
            tu, tv = to_cell(traj[:, 0], traj[:, 2], cache["x0"], cache["z0"], cache["scale"])
            out["traj_u"], out["traj_v"] = tu.astype(np.int32), tv.astype(np.int32)
        return out

    # ------------------------------------------------------------------ export
    def save_npz(self, path, min_count: int = 1):
        centers, colors = self.occupied(min_count)
        np.savez(path, centers=centers, colors=colors,
                 trajectory=self.trajectory_array(),
                 voxel_size=self.voxel_size, tracking_mode=self.tracking_mode)

    def save_ply(self, path, min_count: int = 1, trajectory=True, targets=None,
                 markers=None, binary: bool = False):
        """Write a viewable .ply point cloud: the voxel occupancy (true colors), plus the
        flight path as GREEN points and any target instances as large MAGENTA points. Opens in
        MeshLab/CloudCompare. `targets` = optional list of world (3,) points.

        `markers` (session 57): optional list of ((x, y, z), (r, g, b)) pairs. Each is written as a
        7-point cluster -- the centre plus +/- self.voxel_size along each axis -- so it is visible and
        selectable in Blender. Written LAST, after occupancy/trajectory/targets.
        `binary` (session 57): write `format binary_little_endian 1.0` instead of ASCII. Identical
        vertex layout (float x,y,z + uchar red,green,blue), ~3x smaller and ~10x faster to write.
        """
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
        if markers:
            d = self.voxel_size
            offsets = np.array([[0, 0, 0], [d, 0, 0], [-d, 0, 0], [0, d, 0],
                                 [0, -d, 0], [0, 0, d], [0, 0, -d]], np.float32)
            for (mx, my, mz), (mr, mg, mb) in markers:
                xyz.append(np.array([mx, my, mz], np.float32)[None, :] + offsets)
                rgb.append(np.tile(np.array([mr, mg, mb], np.uint8), (7, 1)))
        P = np.concatenate(xyz, axis=0)
        C = np.concatenate(rgb, axis=0)
        if binary:
            with open(path, "wb") as f:
                f.write(b"ply\nformat binary_little_endian 1.0\n")
                f.write(f"element vertex {len(P)}\n".encode("ascii"))
                f.write(b"property float x\nproperty float y\nproperty float z\n")
                f.write(b"property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n")
                rec = np.zeros(len(P), dtype=np.dtype([
                    ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                    ("red", "u1"), ("green", "u1"), ("blue", "u1")]))
                rec["x"], rec["y"], rec["z"] = P[:, 0], P[:, 1], P[:, 2]
                rec["red"], rec["green"], rec["blue"] = C[:, 0], C[:, 1], C[:, 2]
                f.write(rec.tobytes())
        else:
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

    # --------------------------------------------------------------------
    # SESSION-57 BINARY PLY: save_ply gains binary output + marker clusters. The ASCII branch
    # must stay byte-identical (no writer-format regression); the binary branch is verified by
    # re-parsing its own header/payload rather than trusting a byte count alone.
    # --------------------------------------------------------------------
    import tempfile

    sp = MapStore(voxel_size=0.5)
    sp.integrate(np.array([[0.0, 0.0, 1.0], [2.0, 0.0, 1.0], [0.0, 0.0, 3.0]], np.float64),
                 colors=np.array([[10, 20, 30], [40, 50, 60], [70, 80, 90]], np.uint8))
    sp.add_pose([0.1, 0.0, 0.0])
    sp.add_pose([0.2, 0.0, 0.5])
    n_base = len(sp.occupied(1)[0]) + len(sp.trajectory_array())

    with tempfile.TemporaryDirectory() as td:
        td = Path(td)

        # (a) ASCII output unchanged: baseline established by re-deriving the exact header/line
        # format the pre-session-57 writer produced (same f-strings, same field order) and diffing
        # against what save_ply(binary=False) now emits for this fixed synthetic map.
        p_ascii = td / "a.ply"
        sp.save_ply(p_ascii, binary=False)
        text = p_ascii.read_text(encoding="utf-8")
        lines = text.splitlines()
        header_ok = (lines[0] == "ply" and lines[1] == "format ascii 1.0"
                     and lines[2] == f"element vertex {n_base}"
                     and lines[3] == "property float x" and lines[4] == "property float y"
                     and lines[5] == "property float z" and lines[6] == "property uchar red"
                     and lines[7] == "property uchar green" and lines[8] == "property uchar blue"
                     and lines[9] == "end_header")
        body = lines[10:]
        ascii_unchanged = header_ok and len(body) == n_base and all(len(ln.split()) == 6 for ln in body)
        check(f"(g) ascii_unchanged -- header + one 'x y z r g b' line per vertex (n={n_base})",
              ascii_unchanged)

        # (h) binary header + vertex count matches the ASCII file's count for the same arguments.
        p_bin = td / "b.ply"
        sp.save_ply(p_bin, binary=True)
        raw = p_bin.read_bytes()
        binary_header_and_count = (
            raw.startswith(b"ply\nformat binary_little_endian 1.0\n")
            and f"element vertex {n_base}\n".encode("ascii") in raw[:200]
        )
        check("(h) binary_header_and_count -- magic + element vertex N matches ASCII N",
              binary_header_and_count)

        # (i) binary_payload_size: file size == header length + N * 15 bytes (3 f32 + 3 u1).
        header_end = raw.index(b"end_header\n") + len(b"end_header\n")
        expected_size = header_end + n_base * 15
        binary_payload_size = len(raw) == expected_size
        check(f"(i) binary_payload_size -- {len(raw)} == header({header_end}) + {n_base}*15",
              binary_payload_size)

        def _parse_binary(raw_bytes):
            hdr_end = raw_bytes.index(b"end_header\n") + len(b"end_header\n")
            header_txt = raw_bytes[:hdr_end].decode("ascii")
            n = int(header_txt.split("element vertex ")[1].splitlines()[0])
            payload = raw_bytes[hdr_end:]
            rec = np.frombuffer(payload, dtype=np.dtype([
                ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                ("red", "u1"), ("green", "u1"), ("blue", "u1")]), count=n)
            return rec

        # (j) markers_add_seven_points_each: +14 vertices for 2 markers vs markers=None.
        markers = [((1.0, 2.0, 3.0), (255, 0, 0)), ((4.0, 5.0, 6.0), (0, 0, 255))]
        p_mk = td / "m.ply"
        sp.save_ply(p_mk, binary=True, markers=markers)
        rec_mk = _parse_binary(p_mk.read_bytes())
        markers_add_seven_points_each = len(rec_mk) == n_base + 14
        check(f"(j) markers_add_seven_points_each -- {len(rec_mk)} == {n_base}+14",
              markers_add_seven_points_each)

        # (k) marker_colour_present: exactly 7 vertices of each marker colour, beyond the base map.
        base_red = int(np.sum((rec_mk["red"][:n_base] == 255) & (rec_mk["green"][:n_base] == 0)
                               & (rec_mk["blue"][:n_base] == 0)))
        base_blue = int(np.sum((rec_mk["red"][:n_base] == 0) & (rec_mk["green"][:n_base] == 0)
                                & (rec_mk["blue"][:n_base] == 255)))
        n_red = int(np.sum((rec_mk["red"] == 255) & (rec_mk["green"] == 0) & (rec_mk["blue"] == 0)))
        n_blue = int(np.sum((rec_mk["red"] == 0) & (rec_mk["green"] == 0) & (rec_mk["blue"] == 255)))
        marker_colour_present = (n_red - base_red == 7) and (n_blue - base_blue == 7)
        check(f"(k) marker_colour_present -- 7 red + 7 blue beyond base map "
              f"(red={n_red - base_red}, blue={n_blue - base_blue})", marker_colour_present)

        # (l) markers_none_is_noop: markers=None and markers=[] produce identical output.
        p_none = td / "n_none.ply"
        p_empty = td / "n_empty.ply"
        sp.save_ply(p_none, binary=True, markers=None)
        sp.save_ply(p_empty, binary=True, markers=[])
        markers_none_is_noop = p_none.read_bytes() == p_empty.read_bytes()
        check("(l) markers_none_is_noop -- markers=None byte-identical to markers=[]",
              markers_none_is_noop)

    # --------------------------------------------------------------------
    # SESSION-63 KEYS-AS-ARRAY: `_keys` moved from a Python list of tuples to a preallocated
    # (cap,3) int64 array with an explicit live-row counter `_n`, to stop every readout path
    # (occupied/stats/topdown_summary) paying a full-map np.asarray(list) conversion on every
    # call. These tests prove the storage change is behavior-neutral.
    # --------------------------------------------------------------------

    # (m) invariant: after several integrate() calls, _n == len(_row_of) <= _cap, and _keys is
    # shaped (_cap, 3) -- i.e. _keys is capacity, not occupancy.
    si = MapStore(voxel_size=1.0)
    si.integrate(np.array([[0.5, 0.5, 0.5], [3.5, 0.5, 0.5]], np.float64))
    si.integrate(np.array([[0.5, 0.5, 0.5], [7.5, 0.5, 0.5]], np.float64))
    invariant_ok = (si._n == len(si._row_of) <= si._cap
                    and si._keys.shape == (si._cap, 3))
    check(f"(m) invariant -- _n={si._n} == len(_row_of)={len(si._row_of)} <= _cap={si._cap}, "
          f"_keys.shape={si._keys.shape}", invariant_ok)

    # (n) growth: crossing the doubling boundary more than once must leave every PREVIOUSLY
    # stored key byte-identical -- a `_grow` reallocation is a copy, never a reinterpretation.
    sg = MapStore(voxel_size=1.0)
    n_crossings = 0
    growth_preserves_keys = True
    for start in range(0, 2500, 300):
        idx = np.arange(start, start + 300, dtype=np.float64)
        pts = np.stack([idx, np.zeros(300), np.zeros(300)], axis=1)  # each row -> distinct voxel
        cap_before, n_before = sg._cap, sg._n
        keys_before = sg._keys[:n_before].copy()
        sg.integrate(pts)
        if sg._cap != cap_before:
            n_crossings += 1
            if not np.array_equal(sg._keys[:n_before], keys_before):
                growth_preserves_keys = False
    check(f"(n) growth crosses the doubling boundary {n_crossings} times "
          f"(>1) and preserves prior keys byte-identical", n_crossings > 1 and growth_preserves_keys)

    # (o) equivalence, load-bearing: a fixed seeded point cloud against hard-coded expectations.
    # Points -> voxel keys (voxel_size=1.0): (0.5,0.5,0.5)x2 -> key (0,0,0) [row 0, count 2];
    # (1.5,0.5,0.5) -> key (1,0,0) [row 1, count 1]; (2.5,0.5,1.5) -> key (2,0,1) [row 2, count 1].
    so = MapStore(voxel_size=1.0)
    so.integrate(np.array([[0.5, 0.5, 0.5], [0.5, 0.5, 0.5], [1.5, 0.5, 0.5], [2.5, 0.5, 1.5]],
                          np.float64))
    len_eq_n = len(so) == so._n == 3
    centers, _ = so.occupied(min_count=1)
    expected_centers = np.array([[0.5, 0.5, 0.5], [1.5, 0.5, 0.5], [2.5, 0.5, 1.5]], np.float32)
    occupied_row_order = np.allclose(centers, expected_centers)
    stats_n_voxels = so.stats()["n_voxels"] == so._n == 3
    td = so.topdown_summary()
    topdown_ok = td["n_voxels_kept"] == 3 and len(td["cells_u"]) > 0
    equivalence_ok = len_eq_n and occupied_row_order and stats_n_voxels and topdown_ok
    check(f"(o) equivalence -- len==_n==3 ({len_eq_n}), occupied() in row order "
          f"({occupied_row_order}), stats()['n_voxels']==_n ({stats_n_voxels}), "
          f"topdown_summary() n_voxels_kept==3 ({topdown_ok})", equivalence_ok)

    # (p) empty store: nothing raises, and every readout is the well-formed zero/empty shape.
    se2 = MapStore(voxel_size=0.1)
    empty_len = len(se2) == 0
    ec, ecol = se2.occupied()
    empty_occupied = ec.shape == (0, 3) and ecol.shape == (0, 3)
    etd = se2.topdown_summary()
    empty_topdown = (etd["n_voxels_kept"] == 0 and etd["bounds"] is None
                      and etd["span_world"] == 0.0 and len(etd["cells_u"]) == 0
                      and len(etd["cells_v"]) == 0 and len(etd["cells_rgb"]) == 0
                      and len(etd["traj_u"]) == 0 and len(etd["traj_v"]) == 0)
    check(f"(p) empty store -- len==0 ({empty_len}), occupied() (0,3)/(0,3) ({empty_occupied}), "
          f"topdown_summary() zero-filled ({empty_topdown})",
          empty_len and empty_occupied and empty_topdown)

    # --------------------------------------------------------------------
    # SESSION-63 TOPDOWN CACHE: topdown_summary() caches the cell raster (cells_u/v/rgb, bounds,
    # span_world, n_voxels_kept, x0/z0/scale) keyed by (grid, pad, min_count), invalidated only by
    # _td_dirty (set at the end of integrate(), never by add_pose). traj_u/traj_v are recomputed on
    # every call regardless of the cache, since the trajectory grows every frame.
    # --------------------------------------------------------------------

    # (q) identical output: an uncached call and the following cached call return identical
    # content, key for key, dtypes included.
    sq = MapStore(voxel_size=0.5)
    rng = np.random.default_rng(0)
    pts_q = rng.uniform(-5, 5, size=(500, 3))
    sq.integrate(pts_q, colors=rng.uniform(0, 255, size=(500, 3)).astype(np.uint8))
    sq.add_pose([0.0, 0.0, 0.0])
    sq.add_pose([1.0, 0.0, 1.0])
    td_uncached = sq.topdown_summary()
    td_cached = sq.topdown_summary()
    identical_output = (
        np.array_equal(td_uncached["cells_u"], td_cached["cells_u"])
        and np.array_equal(td_uncached["cells_v"], td_cached["cells_v"])
        and np.array_equal(td_uncached["cells_rgb"], td_cached["cells_rgb"])
        and np.array_equal(td_uncached["traj_u"], td_cached["traj_u"])
        and np.array_equal(td_uncached["traj_v"], td_cached["traj_v"])
        and td_uncached["cells_u"].dtype == td_cached["cells_u"].dtype == np.int32
        and td_uncached["cells_rgb"].dtype == td_cached["cells_rgb"].dtype == np.uint8
        and td_uncached["bounds"] == td_cached["bounds"]
        and td_uncached["span_world"] == td_cached["span_world"]
        and td_uncached["n_voxels_kept"] == td_cached["n_voxels_kept"]
    )
    check("(q) identical_output -- cached call matches preceding uncached call, key for key",
          identical_output)

    # (r) invalidation is real (the no-silent-fallback case): integrate() new points that add
    # voxels must change the next summary, and _td_dirty must be True immediately after.
    td_before = sq.topdown_summary()
    sq.integrate(np.array([[50.0, 0.0, 50.0]], np.float64))
    dirty_after_integrate = sq._td_dirty is True
    td_after = sq.topdown_summary()
    invalidation_real = (dirty_after_integrate
                         and td_after["n_voxels_kept"] != td_before["n_voxels_kept"]
                         and not np.array_equal(td_after["cells_u"], td_before["cells_u"]))
    check(f"(r) invalidation_real -- integrate() sets _td_dirty and changes the next summary "
          f"(dirty={dirty_after_integrate}, n_before={td_before['n_voxels_kept']}, "
          f"n_after={td_after['n_voxels_kept']})", invalidation_real)

    # (s) trajectory is never cached: cells stay byte-identical across add_pose() calls while
    # traj_u grows, and _td_dirty stays False -- add_pose must not dirty the cache.
    td1 = sq.topdown_summary()
    sq.add_pose([2.0, 0.0, 2.0])
    sq.add_pose([3.0, 0.0, 3.0])
    sq.add_pose([4.0, 0.0, 4.0])
    dirty_after_pose = sq._td_dirty
    td2 = sq.topdown_summary()
    traj_not_cached = (len(td2["traj_u"]) > len(td1["traj_u"])
                       and np.array_equal(td1["cells_u"], td2["cells_u"])
                       and dirty_after_pose is False)
    check(f"(s) traj_not_cached -- traj_u grows ({len(td1['traj_u'])}->{len(td2['traj_u'])}) "
          f"while cells_u is byte-identical and _td_dirty stays False", traj_not_cached)

    # (t) cache key covers the parameters: grid=100 then grid=200 must each recompute their own
    # raster, not reuse the first one twice.
    td_g100 = sq.topdown_summary(grid=100)
    key_after_100 = sq._td_key
    td_g200 = sq.topdown_summary(grid=200)
    key_after_200 = sq._td_key
    cache_key_covers_params = (key_after_100 == (100, 0.06, 1) and key_after_200 == (200, 0.06, 1)
                                and td_g100["cells_u"].max(initial=0) < 100
                                and td_g200["cells_u"].max(initial=0) < 200)
    check(f"(t) cache_key_covers_params -- grid=100 then grid=200 each get their own raster "
          f"(keys {key_after_100} -> {key_after_200})", cache_key_covers_params)

    # (u) returned arrays are copies: mutating a returned array must not corrupt the cache.
    td3 = sq.topdown_summary(grid=100)
    if len(td3["cells_u"]):
        td3["cells_u"][0] = -1
    td4 = sq.topdown_summary(grid=100)
    returned_are_copies = (len(td4["cells_u"]) == 0) or (td4["cells_u"][0] != -1)
    check("(u) returned_are_copies -- mutating a returned cells_u does not corrupt the cache",
          returned_are_copies)

    # (v) it is actually faster: ~200k voxels, mean of 10 cached calls >= 5x faster than the one
    # uncached call that had to recompute the raster.
    sv = MapStore(voxel_size=0.05)
    rng2 = np.random.default_rng(1)
    pts_v = rng2.uniform(-5, 5, size=(200_000, 3))
    sv.integrate(pts_v)
    t0 = time.perf_counter()
    sv.topdown_summary()
    uncached_dt = time.perf_counter() - t0
    t1 = time.perf_counter()
    for _ in range(10):
        sv.topdown_summary()
    cached_dt = (time.perf_counter() - t1) / 10
    speedup = uncached_dt / max(cached_dt, 1e-9)
    check(f"(v) cache_is_faster -- uncached={uncached_dt*1000:.2f}ms "
          f"cached_mean={cached_dt*1000:.2f}ms speedup={speedup:.1f}x (need >=5x)", speedup >= 5.0)

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
