"""ground_grid.py — Phase-2 Map mode: a 2D free/unknown/occupied ground layer + frontiers.

`MapStore` (the existing voxel map) is **occupied-only** — it answers "where is structure?" but not
"where have I confirmed empty space?" or "where haven't I looked yet?". Frontier-based exploration
needs all three. `GroundGrid` is that layer: a top-down (X-Z) **log-odds occupancy grid**, built
incrementally from the SAME per-keyframe SLAM data MapStore already consumes (camera center + world
pointmap), and queried for **frontiers** — FREE cells on the boundary of the UNOBSERVED region, which
is exactly where the planner should send the drone next (including the door/wall holes the user flagged).

Design boundary (mirrors map_store.py): **transport-agnostic** — pure numpy/scipy in, numpy out. No
ZMQ, no torch, no SLAM imports. It runs in-process inside `perception_worker`; only a compact rasterized
summary is published onward. Dict-keyed sparse storage keeps it bounded and offline-testable.

NO SILENT FALLBACKS (per CLAUDE.md): bad shapes raise; nothing is silently dropped or downgraded.

HARD RULE (CLAUDE.md "CRITICAL AUTONOMY STANDARD"): nothing here encodes THIS room's answer. The grid
is built live from SLAM points; the height slab is **relative to the live camera Y and the per-keyframe
vertical span** (scale-free); thresholds/inflation are general platform/robustness params from config.
A floor stays at floor world-Y because the points are WORLD-frame (SLAM already applied orientation),
so the slab is robust to camera pitch/roll — we slice in world coords, not camera coords.
"""

import argparse
import math
import os

import cv2
import numpy as np
from scipy import ndimage

# Classification ids used in the compact raster summary (and the visualizer overlay).
CLS_UNKNOWN = 0
CLS_FREE = 1
CLS_OCC = 2
CLS_FRONTIER = 3


def explore_cfg(cfg: dict) -> dict:
    """Pull the autonomy.explore block (general params only — see HARD RULE)."""
    return ((cfg or {}).get("autonomy") or {}).get("explore") or {}


class GroundGrid:
    """Incremental 2D (X-Z) log-odds occupancy grid + frontier extraction.

    Cells are integer `(ix, iz) = floor(x / cell), floor(z / cell)`; per cell we keep a clamped
    log-odds value (positive = occupied evidence, negative = free evidence). A cell absent from the
    dict is **UNOBSERVED** (the exploration frontier forms at the FREE/UNOBSERVED boundary).
    """

    def __init__(self, cfg: dict | None = None, **overrides):
        e = explore_cfg(cfg)
        g = lambda k, d: overrides.get(k, e.get(k, d))
        self.cell = float(g("ground_cell_size", 0.10))
        assert self.cell > 0, "ground_cell_size must be positive"
        self.height_band_frac = float(g("height_band_frac", 0.20))
        self.lo_hit = float(g("logodds_hit", 0.85))
        self.lo_miss = float(g("logodds_miss", 0.40))
        self.lo_clamp = float(g("logodds_clamp", 4.0))
        # Defaults: a SINGLE clean carve (lo_miss) classifies FREE and a single hit (lo_hit) OCC —
        # appropriate for this sim where raycast carving is geometrically reliable (no lidar noise);
        # repeated observations only reinforce. Stray single-cell frontiers are filtered downstream
        # by obstacle inflation + min_frontier_cells.
        self.free_thresh = float(g("free_thresh", -0.3))   # lo <= this  => FREE
        self.occ_thresh = float(g("occ_thresh", 0.4))      # lo >= this  => OCC
        self.min_frontier_cells = int(g("min_frontier_cells", 6))
        self.obstacle_inflation = int(g("obstacle_inflation", 2))
        assert self.free_thresh < self.occ_thresh, "free_thresh must be < occ_thresh"

        self._lo: dict[tuple, float] = {}     # (ix,iz) -> clamped log-odds
        self.n_keyframes_integrated = 0

    # ------------------------------------------------------------------ ingest
    @staticmethod
    def _line_cells(x0, z0, x1, z1):
        """Integer DDA: every cell on the line (x0,z0)->(x1,z1), INCLUSIVE of both ends.

        Vectorized in numpy (one sampling per cell along the dominant axis). Returns (M,2) int.
        """
        n = int(max(abs(x1 - x0), abs(z1 - z0)))
        if n == 0:
            return np.array([[x0, z0]], dtype=np.int64)
        t = np.arange(n + 1) / n
        xs = np.rint(x0 + (x1 - x0) * t).astype(np.int64)
        zs = np.rint(z0 + (z1 - z0) * t).astype(np.int64)
        return np.stack([xs, zs], axis=1)

    def integrate(self, camera_center, points_world):
        """Fold one keyframe (camera center (3,) + world points (N,3)) into the grid.

        Points are filtered to a height slab around the camera, deduped to unique ground cells
        (so we cast a few hundred rays, not ~10^4 — perf), each endpoint marked OCC and the
        camera->endpoint ray carved FREE (endpoint excluded). Returns #endpoint cells touched.
        """
        cam = np.asarray(camera_center, dtype=np.float64).reshape(3)
        pts = np.asarray(points_world, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] != 3:
            raise ValueError(f"points_world must be (N,3), got {pts.shape}")
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) == 0:
            return 0

        # --- height slab (world-frame; relative to live camera Y + per-keyframe vertical span) ---
        y = pts[:, 1]
        vspan = float(np.percentile(y, 98) - np.percentile(y, 2))
        band = self.height_band_frac * vspan
        if band > 0:
            keep = np.abs(y - cam[1]) <= band
            pts = pts[keep]
        if len(pts) == 0:
            return 0

        cx = int(math.floor(cam[0] / self.cell))
        cz = int(math.floor(cam[2] / self.cell))
        endpoints = np.floor(pts[:, [0, 2]] / self.cell).astype(np.int64)
        endpoints = np.unique(endpoints, axis=0)        # dedupe -> one ray per unique cell

        for ex, ez in endpoints:
            if ex == cx and ez == cz:
                self._bump((cx, cz), self.lo_hit)       # degenerate: endpoint at camera
                continue
            line = self._line_cells(cx, cz, int(ex), int(ez))
            for fx, fz in line[:-1]:                     # carve free up to (not incl.) the endpoint
                self._bump((int(fx), int(fz)), -self.lo_miss)
            self._bump((int(ex), int(ez)), self.lo_hit)  # endpoint = occupied evidence
        self.n_keyframes_integrated += 1
        return len(endpoints)

    def _bump(self, key, delta):
        lo = self._lo.get(key, 0.0) + delta
        self._lo[key] = max(-self.lo_clamp, min(self.lo_clamp, lo))

    # ------------------------------------------------------------------ readout
    def __len__(self):
        return len(self._lo)

    def _dense(self):
        """Rasterize the dict to dense arrays over the observed bounding box.

        Returns (known, lo, ix0, iz0): `known` bool (cell observed), `lo` float log-odds (0 where
        unknown), and the integer origin (ix0,iz0) so a cell (ix,iz) maps to [iz-iz0, ix-ix0].
        """
        if not self._lo:
            return (np.zeros((0, 0), bool), np.zeros((0, 0), np.float32), 0, 0)
        keys = np.asarray(list(self._lo.keys()), dtype=np.int64)
        vals = np.asarray(list(self._lo.values()), dtype=np.float32)
        ix0, iz0 = int(keys[:, 0].min()), int(keys[:, 1].min())
        w = int(keys[:, 0].max()) - ix0 + 1
        h = int(keys[:, 1].max()) - iz0 + 1
        known = np.zeros((h, w), bool)
        lo = np.zeros((h, w), np.float32)
        rr = keys[:, 1] - iz0
        cc = keys[:, 0] - ix0
        known[rr, cc] = True
        lo[rr, cc] = vals
        return known, lo, ix0, iz0

    def classify_dense(self):
        """Dense classification masks over the observed bbox.

        Returns dict(free, occ, unknown_observed, not_known, ix0, iz0, shape). `not_known` =
        never-observed cells (the true exploration boundary lives between FREE and `not_known`).
        """
        known, lo, ix0, iz0 = self._dense()
        free = known & (lo <= self.free_thresh)
        occ = known & (lo >= self.occ_thresh)
        return {
            "free": free, "occ": occ, "not_known": ~known,
            "ix0": ix0, "iz0": iz0, "shape": known.shape,
        }

    def inflated_occ(self, occ):
        """OCC dilated by `obstacle_inflation` cells (drone-clearance margin)."""
        if self.obstacle_inflation <= 0 or occ.size == 0 or not occ.any():
            return occ
        return ndimage.binary_dilation(occ, iterations=self.obstacle_inflation)

    def frontier_mask(self, c=None):
        """Boolean mask of frontier cells: FREE, adjacent (4-nbr) to a never-observed cell, and
        NOT inside the inflated-occupied margin (so a centroid never lands on an unreachable wall)."""
        c = c or self.classify_dense()
        free, not_known = c["free"], c["not_known"]
        if free.size == 0 or not free.any():
            return np.zeros_like(free)
        # 4-connectivity dilation of the unobserved region; frontier = free touching it.
        cross = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], bool)
        near_unknown = ndimage.binary_dilation(not_known, structure=cross) & ~not_known
        fm = free & near_unknown
        fm &= ~self.inflated_occ(c["occ"])
        return fm

    def frontiers(self):
        """Frontier clusters as world points. Returns a list of dicts sorted by size desc:
        {"center": [X, Z], "size": n_cells} for each connected component >= min_frontier_cells."""
        c = self.classify_dense()
        fm = self.frontier_mask(c)
        if fm.size == 0 or not fm.any():
            return []
        # 8-connectivity: frontier cells run along diagonal "staircase" edges, so a 4-connectivity
        # label would shatter one frontier into many sub-min fragments.
        labels, n = ndimage.label(fm, structure=np.ones((3, 3), bool))
        out = []
        for lab in range(1, n + 1):
            rr, cc = np.where(labels == lab)
            if len(rr) < self.min_frontier_cells:
                continue
            ix = cc.mean() + c["ix0"]
            iz = rr.mean() + c["iz0"]
            out.append({"center": [float((ix + 0.5) * self.cell),
                                   float((iz + 0.5) * self.cell)],
                        "size": int(len(rr))})
        out.sort(key=lambda d: d["size"], reverse=True)
        return out

    def farthest_free(self, pos, margin=0.0, exclude=None):
        """World [X, Z] of the FREE (confirmed-empty) cell farthest (Euclidean) from `pos`, or None if
        no free cells. The planner uses this for done-verification / reposition: when no frontiers are
        reachable, the farthest known free point is the best place to fly to look for uncharted territory.

        `margin` (SLAM units) pulls the returned point back toward `pos` by min(margin, 0.5*dist), so the
        target sits ALMOST in the corner rather than hard against the wall — otherwise the raw farthest
        free cell is inside the forward stand-off shell and the drone (which stops `stop_clearance_dist`
        short of walls) can never reach it. A general stand-off-scale value (like `stop_clearance_dist` /
        `goal_reach_dist`), NOT a room answer.

        `exclude(world_xy) -> bool` (optional) skips FREE cells inside a dead-goal (blacklisted) region, so
        the reposition/verify target ESCAPES a corner the drone keeps bumping instead of re-picking it
        forever (Bug A). We scan candidates farthest-first and return the first non-excluded one, so the
        common case (farthest corner is fine) costs a single predicate call. Returns None when EVERY free
        cell is excluded — the caller (`select`) then correctly declares done."""
        c = self.classify_dense()
        free = c["free"]
        if free.size == 0 or not free.any():
            return None
        rr, cc = np.where(free)
        X = (cc + c["ix0"] + 0.5) * self.cell      # cell-center world coords (matches frontiers())
        Z = (rr + c["iz0"] + 0.5) * self.cell
        d2 = (X - float(pos[0])) ** 2 + (Z - float(pos[1])) ** 2
        fx = fz = None
        for i in np.argsort(d2)[::-1]:             # farthest-first; stop at the first non-excluded cell
            px, pz = float(X[i]), float(Z[i])
            if exclude is not None and exclude([px, pz]):
                continue
            fx, fz = px, pz
            break
        if fx is None:
            return None                            # every FREE cell is blacklisted -> done
        if margin > 0.0:
            dist = math.hypot(fx - float(pos[0]), fz - float(pos[1]))
            if dist > 1e-9:
                pull = min(margin, 0.5 * dist) / dist          # unit step toward pos, clamped to half-way
                fx -= (fx - float(pos[0])) * pull
                fz -= (fz - float(pos[1])) * pull
        return [fx, fz]

    def bbox_corners(self, inset=1.0):
        """The (up to) 4 inset corners of the KNOWN bounding box (FREE ∪ OCC), each pulled `inset` toward
        the box center per axis so it stays reachable inside the forward stand-off shell. Used by the
        ALL-CORNERS verification TOUR (session 10): the planner flies to each in turn so every corner of
        the room reconstructs densely (not just the one main diagonal). Generalizes the old single
        opposite-corner `sweep_corner`.

        Per-axis logic (unchanged from the old sweep_corner):
          * span >= 2*inset -> the two extremes pull `inset` toward center (a wall stand-off both sides).
          * span <  2*inset -> the axis is too narrow for a dual-side stand-off (a corridor's short axis)
            -> BOTH extremes collapse to the axis MIDPOINT (maximally clear of both walls; never the
            overshoot where `x_min + inset` on a 1.5 u span lands past `x_max`).
        Corners are emitted in canonical order (SW, SE, NW, NE) then DEDUPED, so a corridor (one narrow
        axis) yields its 2 end-corners and a tiny box its 1 center. Returns [] when no cells are known.
        `inset` is a general stand-off-scale param (like `goal_reach_dist`), NOT a room answer."""
        c = self.classify_dense()
        known = c["free"] | c["occ"]
        if known.size == 0 or not known.any():
            return []
        rr, cc = np.where(known)
        ix0, iz0 = c["ix0"], c["iz0"]
        x_min = (int(cc.min()) + ix0 + 0.5) * self.cell   # cell-center world coords (matches frontiers())
        x_max = (int(cc.max()) + ix0 + 0.5) * self.cell
        z_min = (int(rr.min()) + iz0 + 0.5) * self.cell
        z_max = (int(rr.max()) + iz0 + 0.5) * self.cell

        def axis_targets(lo, hi):
            if (hi - lo) < 2.0 * inset:                    # too narrow for a dual-side stand-off -> midpoint
                m = 0.5 * (lo + hi)
                return m, m
            return lo + inset, hi - inset

        xlo, xhi = axis_targets(x_min, x_max)
        zlo, zhi = axis_targets(z_min, z_max)
        ordered = [[xlo, zlo], [xhi, zlo], [xlo, zhi], [xhi, zhi]]   # SW, SE, NW, NE
        out = []
        for p in ordered:
            if not any(abs(p[0] - q[0]) < 1e-9 and abs(p[1] - q[1]) < 1e-9 for q in out):
                out.append([float(p[0]), float(p[1])])
        return out

    def inset_to_clearance(self, goal, pos, buffer):
        """Pull `goal` back along the goal->pos axis to the first point that is map-validated FREE AND has
        `buffer` (SLAM units) of clearance from any occupied cell. Returns the adjusted [X, Z], the
        unchanged `goal` if it already clears, or None if no FREE buffered point exists on the segment.

        A frontier centroid sits on the FREE/unobserved boundary, so it can land right against an
        obstacle/corner and stall the drone. Insetting keeps the committed goal reachable. `buffer` is a
        general stand-off-scale distance validated against the LIVE map (OCC dilated by ceil(buffer/cell)
        cells), never a precomputed coordinate."""
        c = self.classify_dense()
        free, occ = c["free"], c["occ"]
        if free.size == 0 or not free.any():
            return None
        ix0, iz0 = c["ix0"], c["iz0"]
        h, w = free.shape
        rad = int(math.ceil(buffer / self.cell)) if buffer > 0.0 else 0
        near_occ = (ndimage.binary_dilation(occ, iterations=rad) if rad > 0 and occ.any()
                    else np.zeros_like(free))

        def _ok(wx, wz):
            ix = int(math.floor(wx / self.cell)) - ix0     # array is [iz-iz0, ix-ix0] (row=Z, col=X)
            iz = int(math.floor(wz / self.cell)) - iz0
            if ix < 0 or iz < 0 or ix >= w or iz >= h:
                return False
            return bool(free[iz, ix]) and not bool(near_occ[iz, ix])

        gx, gz = float(goal[0]), float(goal[1])
        px, pz = float(pos[0]), float(pos[1])
        if _ok(gx, gz):
            return [gx, gz]
        dist = math.hypot(gx - px, gz - pz)
        if dist < 1e-9:
            return None
        step = 0.5 * self.cell                             # sub-cell stride so no cell is skipped
        for k in range(1, int(dist / step) + 1):
            t = (k * step) / dist                          # fraction from goal toward pos
            if _ok(gx + (px - gx) * t, gz + (pz - gz) * t):
                return [gx + (px - gx) * t, gz + (pz - gz) * t]
        return None

    # ------------------------------------------------------------------ bus summary
    def summary(self, raster: int = 160):
        """Compact rasterized class grid for the state bus / visualizer.

        Returns dict(bounds=[x0,x1,z0,z1], raster=R, rows, cols, cls=[int...]) where `cls` is a
        flat row-major grid of CLS_* ids over the observed bounds, downsampled to fit `raster` on
        the long side (nearest, so labels are preserved). Bounds are in WORLD X-Z; v=0 is +Z up to
        match render conventions. Empty grid -> bounds None.
        """
        c = self.classify_dense()
        h, w = c["shape"]
        if h == 0 or w == 0:
            return {"bounds": None, "raster": 0, "rows": 0, "cols": 0, "cls": []}
        fm = self.frontier_mask(c)
        grid = np.full((h, w), CLS_UNKNOWN, np.uint8)
        grid[c["free"]] = CLS_FREE
        grid[c["occ"]] = CLS_OCC
        grid[fm] = CLS_FRONTIER

        # Flip rows so +Z reads up (matches map_store render), then downsample to <= raster.
        grid = grid[::-1, :]
        scale = min(1.0, raster / max(h, w))
        if scale < 1.0:
            rows = max(1, int(round(h * scale)))
            cols = max(1, int(round(w * scale)))
            ri = np.clip((np.arange(rows) / scale).astype(int), 0, h - 1)
            ci = np.clip((np.arange(cols) / scale).astype(int), 0, w - 1)
            grid = grid[np.ix_(ri, ci)]
        rows, cols = grid.shape
        ix0, iz0 = c["ix0"], c["iz0"]
        bounds = [ix0 * self.cell, (ix0 + w) * self.cell,
                  iz0 * self.cell, (iz0 + h) * self.cell]
        return {"bounds": [float(b) for b in bounds], "raster": int(raster),
                "rows": int(rows), "cols": int(cols), "cls": grid.reshape(-1).tolist()}


    # ------------------------------------------------------------------ render (offline inspection)
    # BGR per class id, drawn at CLS_* index: unknown=dark, free=gray, occ=brick, frontier=cyan.
    _CLS_BGR = np.array([[28, 28, 28], [120, 120, 120], [40, 40, 200], [255, 255, 0]], np.uint8)

    def render_overlay(self, out_path=None, size=720, frontiers=True, goal=None, pos=None):
        """Render the classified ground grid (+ optional frontier centroids, goal, drone pos) to a
        BGR image. World +X right, +Z up. Self-contained — for offline `--video` inspection."""
        c = self.classify_dense()
        h, w = c["shape"]
        if h == 0 or w == 0:
            img = np.full((size, size, 3), 18, np.uint8)
            if out_path:
                cv2.imwrite(str(out_path), img)
            return img
        fm = self.frontier_mask(c)
        grid = np.full((h, w), CLS_UNKNOWN, np.uint8)
        grid[c["free"]] = CLS_FREE
        grid[c["occ"]] = CLS_OCC
        grid[fm] = CLS_FRONTIER
        bgr = self._CLS_BGR[grid]              # (h,w,3)
        bgr = bgr[::-1, :, :]                  # flip rows so +Z is up
        scale = max(1, size // max(h, w))
        img = cv2.resize(bgr, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)

        ix0, iz0 = c["ix0"], c["iz0"]
        def to_px(X, Z):
            u = int((X / self.cell - ix0) * scale)
            v = int((h - 1 - (Z / self.cell - iz0)) * scale)
            return u, v

        if frontiers:
            for f in self.frontiers():
                u, v = to_px(*f["center"])
                cv2.circle(img, (u, v), 5, (255, 255, 0), 1)
        if goal is not None:
            u, v = to_px(goal[0], goal[1])
            cv2.drawMarker(img, (u, v), (0, 255, 255), cv2.MARKER_STAR, 16, 2)
        if pos is not None:
            u, v = to_px(pos[0], pos[1])
            cv2.circle(img, (u, v), 4, (0, 255, 0), -1)
        cv2.putText(img, f"ground grid {w}x{h}@{self.cell:g}u  free={int(c['free'].sum())} "
                    f"occ={int(c['occ'].sum())} frontier_cells={int(fm.sum())}",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        if out_path:
            cv2.imwrite(str(out_path), img)
        return img


# ==============================================================================
# Self-test: synthetic camera + a wall of points -> free corridor / occ wall / unknown-behind /
# frontier ring; dedupe path matches per-point carving; inflation drops wall-hugging frontiers.
# ==============================================================================
def _build_walled_room():
    """A camera at the origin looking +Z; a wall plane at z=2 spanning x in [-1.5,1.5], y near
    camera height. Returns (cam, points_world)."""
    cam = np.array([0.0, 0.0, 0.0])
    xs = np.linspace(-1.5, 1.5, 240)
    ys = np.linspace(-0.1, 0.1, 5)            # within the height band around cam Y=0
    X, Y = np.meshgrid(xs, ys)
    Z = np.full_like(X, 2.0)
    pts = np.stack([X.ravel(), Y.ravel(), Z.ravel()], axis=1)
    return cam, pts


def run_self_test():
    ok = True

    def check(name, cond):
        nonlocal ok
        ok = ok and bool(cond)
        print(f"[ground_grid][self-test] {'PASS' if cond else 'FAIL'}  {name}")

    def close(a, b, tol=1e-6):
        return a is not None and abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol

    gg = GroundGrid(None, ground_cell_size=0.10, obstacle_inflation=2, min_frontier_cells=4)
    cam, pts = _build_walled_room()
    touched = gg.integrate(cam, pts)
    c = gg.classify_dense()
    check("integrate touched endpoint cells", touched > 0)

    # A cell midway between camera and wall (z~1.0) should be FREE; a cell at the wall (z~2.0) OCC;
    # a cell behind the wall (z~2.5) should be UNOBSERVED.
    def cls_at(x, z):
        ix = int(math.floor(x / gg.cell)) - c["ix0"]
        iz = int(math.floor(z / gg.cell)) - c["iz0"]
        if not (0 <= iz < c["shape"][0] and 0 <= ix < c["shape"][1]):
            return "UNKNOWN"   # outside the observed bbox = never observed = unknown
        if c["occ"][iz, ix]:
            return "OCC"
        if c["free"][iz, ix]:
            return "FREE"
        if c["not_known"][iz, ix]:
            return "UNKNOWN"
        return "UNCERTAIN"

    check("corridor cell (0,1.0) is FREE", cls_at(0.0, 1.0) == "FREE")
    check("wall cell (0,2.0) is OCC", cls_at(0.0, 2.0) == "OCC")
    check("behind-wall cell (0,2.6) is UNKNOWN", cls_at(0.0, 2.6) == "UNKNOWN")

    fr = gg.frontiers()
    check("at least one frontier cluster found", len(fr) >= 1)
    # Frontiers should sit at the FREE/UNKNOWN seam (around the wall edges / sides), not at the wall.
    check("frontier centers exist with size>=min", all(f["size"] >= gg.min_frontier_cells for f in fr))

    # Dedupe path correctness: integrating the SAME points twice (lots of duplicate endpoint cells)
    # must produce the identical grid as integrating the deduped set once would — i.e. unique()
    # inside integrate makes duplicate points within a keyframe a no-op on the count of rays.
    gg2 = GroundGrid(None, ground_cell_size=0.10, obstacle_inflation=2, min_frontier_cells=4)
    cam2, pts2 = _build_walled_room()
    pts2_dup = np.concatenate([pts2, pts2], axis=0)        # exact duplicates
    n1 = gg2.integrate(cam2, pts2_dup)
    gg3 = GroundGrid(None, ground_cell_size=0.10, obstacle_inflation=2, min_frontier_cells=4)
    n2 = gg3.integrate(cam2, pts2)
    same = (gg2._lo == gg3._lo)
    check("endpoint dedupe: duplicate points within a keyframe don't change the grid", same and n1 == n2)

    # Obstacle inflation drops frontier cells that would sit within the margin of the wall.
    gg_inf = GroundGrid(None, ground_cell_size=0.10, obstacle_inflation=5, min_frontier_cells=4)
    gg_inf.integrate(cam, pts)
    fm_small = gg.frontier_mask()
    fm_big = gg_inf.frontier_mask()
    check("larger obstacle_inflation yields no more frontier cells", fm_big.sum() <= fm_small.sum())

    # farthest_free: returns a FREE cell far from the camera; None on an empty grid.
    ff = gg.farthest_free([cam[0], cam[2]])
    check("farthest_free returns a free cell well away from the camera",
          ff is not None and cls_at(ff[0], ff[1]) == "FREE" and math.hypot(ff[0] - cam[0], ff[1] - cam[2]) > 1.0)
    check("farthest_free is None on an empty grid", GroundGrid(None).farthest_free([0.0, 0.0]) is None)
    # margin pulls the returned point inward toward pos (reachable "almost in the corner"): the inset
    # point is closer to the camera than the raw corner, by ~margin, and still on the same bearing.
    ff_in = gg.farthest_free([cam[0], cam[2]], margin=0.3)
    d_raw = math.hypot(ff[0] - cam[0], ff[1] - cam[2])
    d_in = math.hypot(ff_in[0] - cam[0], ff_in[1] - cam[2])
    check("farthest_free(margin) pulls the target inward by ~margin",
          ff_in is not None and abs((d_raw - d_in) - 0.3) < 1e-6)

    # farthest_free(exclude): blacklisting the raw farthest cell's region forces a DIFFERENT free cell
    # (the Bug-A escape-recompute); excluding everything returns None so the planner declares done.
    ex_r = 0.5
    ff_ex = gg.farthest_free([cam[0], cam[2]], exclude=lambda p: math.hypot(p[0] - ff[0], p[1] - ff[1]) <= ex_r)
    check("farthest_free(exclude) skips the blacklisted region -> a different free cell",
          ff_ex is not None and cls_at(ff_ex[0], ff_ex[1]) == "FREE"
          and math.hypot(ff_ex[0] - ff[0], ff_ex[1] - ff[1]) > ex_r)
    check("farthest_free(exclude-all) is None", gg.farthest_free([cam[0], cam[2]], exclude=lambda p: True) is None)

    # bbox_corners: the (up to) 4 inset corners of the KNOWN bbox (SW, SE, NW, NE), with per-axis midpoint
    # clamp on axes too narrow for a dual-side stand-off, deduped. Build controlled FREE rectangles via _lo
    # (1u cells so cell-center world coords are trivial: x = (ix+0.5), z = (iz+0.5)).
    def _free_rect(nx, nz, cell=1.0):
        g = GroundGrid(None, ground_cell_size=cell)
        for ix in range(nx):
            for iz in range(nz):
                g._lo[(ix, iz)] = -5.0                  # well below free_thresh -> FREE
        return g

    def has_corner(corners, pt):
        return any(close(c, pt) for c in corners)

    # (a) wide box (x:0.5..5.5, z:0.5..7.5), inset 1u: 4 corners SW[1.5,1.5] SE[4.5,1.5] NW[1.5,6.5]
    #     NE[4.5,6.5], in canonical order.
    gs = _free_rect(6, 8)
    corners = gs.bbox_corners(inset=1.0)
    check("bbox_corners returns 4 inset corners of a wide box",
          len(corners) == 4 and close(corners[0], [1.5, 1.5]) and close(corners[1], [4.5, 1.5])
          and close(corners[2], [1.5, 6.5]) and close(corners[3], [4.5, 6.5]))
    # (b) CORRIDOR (x:0.5..1.5 span 1.0 < 2*inset; z:0.5..11.5 span 11.0): the narrow X clamps to its
    #     MIDPOINT (1.0) so SW==SE and NW==NE dedupe -> exactly the 2 end-corners [1.0,1.5] and [1.0,10.5],
    #     both strictly inside the bbox (the overshoot bug this guards against).
    gc = _free_rect(2, 12)
    cc2 = gc.bbox_corners(inset=1.0)
    check("bbox_corners corridor: narrow axis -> midpoint, dedupes to 2 end-corners",
          len(cc2) == 2 and has_corner(cc2, [1.0, 1.5]) and has_corner(cc2, [1.0, 10.5]))
    check("bbox_corners corridor: end-corners stay inside the known bbox",
          all(0.5 <= p[0] <= 1.5 and 0.5 <= p[1] <= 11.5 for p in cc2))
    # (c) both axes degenerate (1x1 known) -> a single center corner; empty grid -> [].
    check("bbox_corners both-axes-degenerate -> single center corner",
          _free_rect(1, 1).bbox_corners() == [[0.5, 0.5]])
    check("bbox_corners empty grid -> []", GroundGrid(None).bbox_corners() == [])

    # inset_to_clearance: an already-clear FREE goal is unchanged; a wall-hugging goal is pulled back along
    # the goal->pos axis to a FREE buffered cell; an impossibly large buffer yields None (no such cell).
    free_goal = [0.0, 1.0]
    same = gg.inset_to_clearance(free_goal, [cam[0], cam[2]], buffer=0.05)
    check("inset_to_clearance leaves an already-clear FREE goal unchanged",
          same is not None and abs(same[0] - free_goal[0]) < 1e-9 and abs(same[1] - free_goal[1]) < 1e-9)
    inset = gg.inset_to_clearance([0.0, 2.0], [cam[0], cam[2]], buffer=gg.obstacle_inflation * gg.cell)
    check("inset_to_clearance pulls a wall-hugging goal back to a FREE buffered cell",
          inset is not None and cls_at(inset[0], inset[1]) == "FREE")
    check("inset_to_clearance returns None under an impossibly large buffer",
          gg.inset_to_clearance([0.0, 2.0], [cam[0], cam[2]], buffer=100.0) is None)

    # Summary raster is well-formed and label-preserving in size.
    s = gg.summary(raster=64)
    check("summary raster well-formed", s["bounds"] is not None and s["rows"] * s["cols"] == len(s["cls"])
          and max(s["cls"]) <= CLS_FRONTIER)

    print(f"\n[ground_grid][self-test] {'ALL PASS' if ok else 'FAILURES PRESENT'}")
    return ok


def main():
    ap = argparse.ArgumentParser(description="GroundGrid (2D free/unknown/occupied + frontiers)")
    ap.add_argument("--self-test", action="store_true", help="run the synthetic self-test (no hardware)")
    args = ap.parse_args()
    if args.self_test:
        raise SystemExit(0 if run_self_test() else 1)
    ap.print_help()


if __name__ == "__main__":
    main()
