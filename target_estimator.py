"""target_estimator.py — aggregate per-detection 3D hits into target estimate(s).

Each time the object detector finds the target and the lift back-projects its center pixel into the
voxel map (see `perception_worker` + `MapStore.raycast`), we get one world-space **hit point**. This
module accumulates those hits and produces the Phase-3 deliverable: the estimated 3D **position(s)**
plus **uncertainty**.

**Multi-instance:** the designated object can appear MORE THAN ONCE in the lab (e.g. two rifles at two
locations). So we report **every well-supported instance** — `estimate_all()` returns a list of
clusters, each its own position + uncertainty + hit count. `estimate()` keeps returning the single
best (most-supported) cluster for back-compat / single-target callers.

Robustness: a stray detection or a ray grazing the wrong wall throws an isolated outlier hit. We find
**spatial-consensus clusters** (mode-seeking): each instance is the densest group of hits within
`CLUSTER_RADIUS`; a cluster must have at least `MIN_CLUSTER` hits to count as real (so a 1-2 hit
wrong-wall ray is filtered, while a genuine second instance with many hits is reported). Position is
the cluster **median** (per-axis, outlier-resistant).

Thresholds (well-conditioned: intra-instance spread ~0.1-0.15u vs inter-instance gaps ~1u+):
  * CLUSTER_RADIUS = spatial extent of ONE instance; a hit farther than this from every known cluster
    seeds a new one.
  * MIN_INSTANCE   = minimum RAW hit count to REPORT a cluster as a distinct target. Filters incidental
    small clusters of early/inaccurate hits (observed: the real rifles had 50+ hits each while
    transient blips had 4-5 — a ~10x gap). Existence is by count, NOT proximity, so a real-but-distant
    instance seen many times is still reported.
  * MIN_CLUSTER    = support for the `confident` flag (a cluster can be reported yet flagged tentative).

Transport-agnostic: pure numpy in, plain dict(s) out (the caller serializes for the bus). No
ZMQ / torch / SLAM imports, so it is unit-testable offline (see `__main__`).
"""

import numpy as np


class TargetEstimator:
    """Online accumulator of 3D target hits -> robust position(s) + uncertainty via mode-seeking."""

    CLUSTER_RADIUS = 0.30   # world units: hits within this of a cluster centre are inliers / one instance
    MIN_INSTANCE = 8        # raw inliers needed to REPORT a cluster as a distinct target (filters blips)
    MIN_CLUSTER = 3         # raw inliers needed for the `confident` flag
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

    def _cluster_dict(self, Pin: np.ndarray) -> dict:
        """Build the position + uncertainty record for one cluster's inlier points."""
        pos = np.median(Pin, axis=0)
        rin = np.linalg.norm(Pin - pos, axis=1)
        rms = float(np.sqrt(np.mean(rin ** 2))) if len(rin) else 0.0
        n_inl = int(len(Pin))
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

    def estimate_all(self) -> list[dict]:
        """All well-supported instances, sorted by support (most hits first).

        Iterative peel-off mode-seeking: find the densest cluster within CLUSTER_RADIUS (refined once
        around its centroid); if it has >= MIN_CLUSTER hits, emit it and remove its inliers; repeat on
        the remainder until no remaining cluster meets MIN_CLUSTER. Deterministic; degrades to a single
        cluster when there is one instance; isolated wrong-wall rays (below MIN_CLUSTER) are dropped.
        """
        if not self._hits:
            return []
        P = np.asarray(self._hits, np.float64)
        remaining = np.ones(len(P), bool)
        out = []
        while remaining.sum() >= self.MIN_INSTANCE:
            idxs = np.where(remaining)[0]
            Pr = P[idxs]
            d = np.linalg.norm(Pr[:, None, :] - Pr[None, :, :], axis=2)
            within = d <= self.CLUSTER_RADIUS
            seed = int(np.argmax(within.sum(axis=1)))          # densest by RAW count (existence)
            inl = within[seed]
            centroid = Pr[inl].mean(axis=0)
            refined = np.linalg.norm(Pr - centroid, axis=1) <= self.CLUSTER_RADIUS
            if refined.any():
                inl = refined
            if int(inl.sum()) < self.MIN_INSTANCE:
                break               # densest remaining cluster is a blip -> stop (rest are smaller)
            out.append(self._cluster_dict(Pr[inl]))
            remaining[idxs[inl]] = False
        out.sort(key=lambda c: -c["n_inliers"])
        return out

    def estimate(self) -> dict | None:
        """The single best (most-supported) instance, or None — back-compat single-target view."""
        all_ = self.estimate_all()
        return all_[0] if all_ else None


if __name__ == "__main__":
    # Offline self-test: TWO real instances (12 + 9 tight hits), ONE incidental blip (4 hits, below
    # MIN_INSTANCE), and 5 SCATTERED wrong-wall outliers. Peel-off must return EXACTLY the 2 real
    # instances (tight + confident) and drop both the 4-hit blip and the scatter.
    rng = np.random.default_rng(0)
    est = TargetEstimator(label="test target")
    A = np.array([2.0, -0.5, 3.0])      # instance 1 (more hits)
    B = np.array([-1.5, 0.2, 6.0])      # instance 2
    C = np.array([4.0, 1.0, 1.0])       # incidental blip (4 hits < MIN_INSTANCE=8 -> dropped)
    for i in range(12):
        est.add(A + rng.normal(0, 0.05, 3), frame_id=i)
    for i in range(9):
        est.add(B + rng.normal(0, 0.05, 3), frame_id=100 + i)
    for i in range(4):
        est.add(C + rng.normal(0, 0.05, 3), frame_id=200 + i)
    for j in range(5):                   # scattered wrong-wall rays (each isolated)
        est.add(rng.uniform(-9, 9, 3), frame_id=300 + j)
    est.add_found_no_hit(frame_id=400)

    ests = est.estimate_all()
    print(f"[target_estimator] {len(ests)} reported instance(s):")
    for k, e in enumerate(ests):
        print(f"   #{k}: pos={e['position']} n_inliers={e['n_inliers']} rms={e['radial_rms']} "
              f"confident={e['confident']}")
    assert len(ests) == 2, f"expected 2 instances, got {len(ests)} (blip/scatter not filtered?)"
    pA, pB = np.array(ests[0]["position"]), np.array(ests[1]["position"])
    assert np.linalg.norm(pA - A) < 0.1, f"instance 1 off: {pA}"
    assert np.linalg.norm(pB - B) < 0.1, f"instance 2 off: {pB}"
    assert ests[0]["n_inliers"] == 12 and ests[1]["n_inliers"] == 9
    assert all(e["confident"] for e in ests)
    assert est.estimate()["n_inliers"] == 12                 # back-compat: best instance
    assert ests[0]["n_hits"] == 30 and ests[0]["n_miss"] == 1
    print(f"[target_estimator] OK  (2 real instances localized; 4-hit blip + 5 scattered dropped; "
          f"best n_inliers={ests[0]['n_inliers']})")
