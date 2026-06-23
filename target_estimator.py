"""target_estimator.py — aggregate per-detection 3D hits into one target estimate.

Each time the object detector finds the target and the lift back-projects its center pixel
into the voxel map (see `perception_worker` + `MapStore.raycast`), we get one world-space
**hit point**. This module accumulates those hits and produces the Phase-3 deliverable: a
single estimated 3D **position** plus an **uncertainty** describing how tightly the
independent detections agree.

Robustness: a stray detection (wrong object) or a ray that grazes the wrong wall can throw
an outlier hit far from the cluster. We therefore report a **median** position (per-axis,
outlier-resistant) and derive uncertainty from the **inliers** — hits within a MAD-scaled
radius of the median — while still surfacing the raw/inlier counts so a sparse or
disagreeing estimate is never silently presented as confident.

Transport-agnostic: pure numpy in, plain dict out (the caller serializes for the bus). No
ZMQ / torch / SLAM imports, so it is unit-testable offline (see `__main__`).
"""

import numpy as np


class TargetEstimator:
    """Online accumulator of 3D target hits -> robust position + uncertainty.

    Estimation is **spatial-consensus / mode-seeking**, not a plain median: jittery 2D
    detections back-project to rays that fan through a cluttered room and hit the wrong wall,
    so a large fraction of hits can be wrong (a median breaks down past 50% outliers). Instead
    we find the **densest cluster** of hits (where many independent rays agree) and report its
    centre; everything outside the consensus radius is discarded as a wrong-wall ray.
    """

    CLUSTER_RADIUS = 0.30   # world units: hits within this of the cluster centre are inliers
    MIN_CLUSTER = 3         # inliers needed before the estimate is called `confident`
    TIGHT_RMS = 0.30        # in-cluster radial RMS must be below this to be `confident`

    def __init__(self, label: str = "", max_hits: int = 500):
        self.label = label
        self.max_hits = int(max_hits)
        self._hits: list[np.ndarray] = []        # world (3,) per accepted detection
        self._frame_ids: list[int] = []
        self.n_found = 0     # detections that reported the target visible
        self.n_hits = 0      # of those, how many produced a map hit (ray hit a voxel)
        self.n_miss = 0      # found but the ray hit nothing in the map

    def add_found_no_hit(self, frame_id=None):
        """Record a detection that saw the target but whose ray hit no map voxel."""
        self.n_found += 1
        self.n_miss += 1

    def add(self, point_world, frame_id=None):
        """Record a detection whose back-projected ray hit a map voxel at `point_world`."""
        p = np.asarray(point_world, np.float64).reshape(3)
        if not np.isfinite(p).all():
            return
        self.n_found += 1
        self.n_hits += 1
        self._hits.append(p)
        self._frame_ids.append(frame_id)
        if len(self._hits) > self.max_hits:        # keep the most recent (drift-friendly)
            self._hits.pop(0)
            self._frame_ids.pop(0)

    def estimate(self) -> dict | None:
        """Current best estimate, or None if there are no map hits yet.

        Finds the densest cluster of hits (mode-seeking): seed = the hit with the most
        neighbours within CLUSTER_RADIUS, refined once around the cluster centroid; the cluster
        members are the inliers. Returns position (cluster median) + uncertainty (in-cluster
        radial_rms, std_xyz, spread_p90), counts, cluster_frac, and a coarse `confident` flag.
        """
        if not self._hits:
            return None
        P = np.asarray(self._hits, np.float64)
        if len(P) == 1:
            inl = np.array([True])
        else:
            # Pairwise distances -> densest seed -> cluster, refined once around its centroid.
            d = np.linalg.norm(P[:, None, :] - P[None, :, :], axis=2)
            seed = int(np.argmax((d <= self.CLUSTER_RADIUS).sum(axis=1)))
            inl = d[seed] <= self.CLUSTER_RADIUS
            centroid = P[inl].mean(axis=0)
            refined = np.linalg.norm(P - centroid, axis=1) <= self.CLUSTER_RADIUS
            if refined.any():
                inl = refined
        Pin = P[inl]
        pos = np.median(Pin, axis=0)
        rin = np.linalg.norm(Pin - pos, axis=1)
        rms = float(np.sqrt(np.mean(rin ** 2))) if len(rin) else 0.0
        n_inl = int(inl.sum())
        return {
            "label": self.label,
            "position": [round(float(v), 4) for v in pos],
            "n_found": self.n_found,
            "n_hits": self.n_hits,
            "n_inliers": n_inl,
            "n_miss": self.n_miss,
            "cluster_frac": round(n_inl / max(self.n_hits, 1), 3),
            "radial_rms": round(rms, 4),
            "std_xyz": [round(float(s), 4) for s in Pin.std(axis=0)],
            "spread_p90": round(float(np.percentile(rin, 90)) if len(rin) else 0.0, 4),
            "confident": bool(n_inl >= self.MIN_CLUSTER and rms <= self.TIGHT_RMS),
        }


if __name__ == "__main__":
    # Offline self-test: a tight cluster of 8 good hits + 7 SCATTERED wrong-wall outliers (a
    # majority that is NOT clustered). Mode-seeking must lock onto the cluster — a plain median
    # would be dragged off by the outlier majority.
    rng = np.random.default_rng(0)
    est = TargetEstimator(label="test target")
    true = np.array([2.0, -0.5, 3.0])
    for i in range(8):
        est.add(true + rng.normal(0, 0.05, 3), frame_id=i)
    for j in range(7):                                  # scattered wrong-wall rays (majority!)
        est.add(rng.uniform(-6, 6, 3), frame_id=100 + j)
    est.add_found_no_hit(frame_id=200)                  # found but no map hit
    e = est.estimate()
    print("[target_estimator] estimate:", e)
    err = np.linalg.norm(np.array(e["position"]) - true)
    assert err < 0.15, f"position off by {err:.3f} (consensus failed against the outlier majority?)"
    assert e["n_hits"] == 15 and e["n_miss"] == 1
    assert e["n_inliers"] >= 7 and e["radial_rms"] < 0.15 and e["confident"]
    print(f"[target_estimator] OK  (pos err {err*1000:.0f} mm, cluster {e['n_inliers']}/{e['n_hits']} "
          f"hits, frac {e['cluster_frac']}, confident={e['confident']})")
