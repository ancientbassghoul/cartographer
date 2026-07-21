"""visual_recovery.py — CPU-only SIFT-based visual loss-recovery probe (session 35 ALT).

The operator's vision: "the live NDI image tells us why tracking dropped and what to do about it." A
PLAN-STALE almost always means the drone got too close to something, or turned to a bad angle — and the
answer is in the live image, never used for recovery before this module.

Caches the last frame SLAM was actually TRACKING on ("F_LKG" — last known good), then matches a live
frame against it with the SAME validated classical-CV primitive already used elsewhere in this project
(`benchmark_detectors.SiftDetector`: cv2 SIFT + Lowe ratio test + RANSAC homography, `SIFT_MIN_INLIERS`
confidence). The ~25 lines are COPIED here (not imported from benchmark_detectors, whose top level pulls
torch/LightGlue for the GPU engines this module deliberately has no reason to load).

================================ CPU-ONLY, ON PURPOSE ================================
SLAM keeps trying to relocalize every stale frame — it is NOT idle while we're deciding what to do. A
GPU-heavy matcher (LightGlue) would contend with the very relocalization we're waiting on. SIFT on the
512x288 transport frame is tens of ms; the drone is hovering/turning slowly during recovery, so a slower
tick here is an acceptable trade. A GPU escalation (LightGlue) stays a noted future option, not built.
========================================================================================

Consumed by `autopilot.py`'s `ExploreController`, gated behind `use_visual_recovery_on_stale`
(config.yaml `autonomy.explore`, default OFF — mirrors `use_rewind_on_stale`'s precedent that a new
stale-recovery path ships live-fly-untested). `ExploreController` stays a pure state machine fed small
verdict values (this module owns ALL image handling), exactly like it already consumes
`wall_contact`/`backwall_contact` from `flow_contact_detector.py`.
"""

import argparse
import time
from dataclasses import dataclass

import cv2
import numpy as np

SIFT_RATIO = 0.75          # Lowe ratio test (matches benchmark_detectors.SiftDetector)
SIFT_MIN_INLIERS = 12      # RANSAC inliers to call it a find (matches benchmark_detectors.SiftDetector)


@dataclass
class VisualMatch:
    """One verdict from matching a live frame against F_LKG (the cached last-known-good frame)."""
    has_lkg: bool                  # a reference frame exists to match against at all
    matched: bool = False          # inliers >= min_inliers
    inliers: int = 0
    contained: bool = False        # F_live is a zoomed-in crop of F_LKG (2a: nose closer to the same surface)
    planar_like: bool = False      # high inlier ratio -> nose-to-a-flat-surface (2b)
    scale: float | None = None     # homography linear scale (LKG->live); only meaningful when matched


class VisualRecoveryProbe:
    """Caches F_LKG (the most recent frame SLAM was TRACKING on) and matches later frames against it.
    See the module docstring for why this is CPU-only SIFT, copied (not imported) from
    `benchmark_detectors.SiftDetector`."""

    def __init__(self, *, min_inliers=SIFT_MIN_INLIERS, planar_inlier_ratio=0.85, contain_margin_frac=0.02):
        self.min_inliers = int(min_inliers)
        self.planar_inlier_ratio = float(planar_inlier_ratio)
        self.contain_margin_frac = float(contain_margin_frac)
        self.sift = cv2.SIFT_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_L2)
        self._lkg = None    # cached BGR frame (last known good — SLAM was TRACKING when it was captured)

    def update_reference(self, frame, tracked):
        """Cache `frame` as F_LKG whenever `tracked` is True. Called every tick from run_explore, gated on
        the same `plan.get("plan_valid")` boundary session-34's own cached-pose snapshot uses — so "F_LKG"
        is really "frame at the last valid plan", the same close-enough proxy that cache already accepts
        (the plan can lag frames by up to ~1s). Cheap (a copy); SIFT only runs at match() time."""
        if tracked and frame is not None:
            self._lkg = frame.copy()

    def _keypoints(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        return self.sift.detectAndCompute(gray, None)

    def match(self, frame) -> VisualMatch:
        """SIFT+RANSAC-homography match of `frame` against the cached F_LKG. H maps LKG -> live (same
        src/dst convention as benchmark_detectors.SiftDetector: src=reference keypoints, dst=frame
        keypoints), so `scale = sqrt(|det(H[:2,:2])|)` reads >1 when the live frame shows a MAGNIFIED
        (closer) view of what F_LKG covered — the natural "moved closer" signal. `contained` needs the
        OPPOSITE direction (does the live frame's own extent sit entirely inside a crop of F_LKG?), so it
        explicitly inverts H rather than reusing it raw."""
        if self._lkg is None or frame is None:
            return VisualMatch(has_lkg=self._lkg is not None)
        kp0, des0 = self._keypoints(self._lkg)
        kp1, des1 = self._keypoints(frame)
        out = VisualMatch(has_lkg=True)
        if des0 is None or des1 is None or len(kp0) < 2 or len(kp1) < 2:
            return out
        good = []
        for m_n in self.matcher.knnMatch(des0, des1, k=2):
            if len(m_n) == 2 and m_n[0].distance < SIFT_RATIO * m_n[1].distance:
                good.append(m_n[0])
        if len(good) < 4:
            return out
        src = np.float32([kp0[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)   # F_LKG points
        dst = np.float32([kp1[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)   # F_live points
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
        if H is None or mask is None:
            return out
        mask = mask.ravel().astype(bool)
        inliers = int(mask.sum())
        out.inliers = inliers
        out.matched = inliers >= self.min_inliers
        if not out.matched:
            return out
        out.planar_like = (inliers / float(len(good))) >= self.planar_inlier_ratio
        try:
            out.scale = float(np.sqrt(abs(np.linalg.det(H[:2, :2]))))
        except (np.linalg.LinAlgError, ValueError):
            out.scale = None
        try:
            h_live, w_live = frame.shape[:2]
            h_lkg, w_lkg = self._lkg.shape[:2]
            H_inv = np.linalg.inv(H)                       # live -> LKG (the direction "contained" needs)
            corners = np.float32([[0, 0], [w_live, 0], [w_live, h_live], [0, h_live]]).reshape(-1, 1, 2)
            warped = cv2.perspectiveTransform(corners, H_inv).reshape(-1, 2)
            mx, my = self.contain_margin_frac * w_lkg, self.contain_margin_frac * h_lkg
            out.contained = bool(np.all(warped[:, 0] >= -mx) and np.all(warped[:, 0] <= w_lkg + mx)
                                  and np.all(warped[:, 1] >= -my) and np.all(warped[:, 1] <= h_lkg + my))
        except np.linalg.LinAlgError:
            out.contained = False       # a singular H (degenerate match) can't be inverted -> not contained
        return out


# ==============================================================================
# Self-test: deterministic synthetic frame pairs (no hardware, no recorded flight).
# ==============================================================================
def _textured_image(w=480, h=360, seed=0, n_shapes=90):
    """A procedurally-textured BGR image with plenty of SIFT-friendly corners (random rects + circles over
    a mild noise floor), so warps/crops of it produce enough stable keypoints to match reliably."""
    rng = np.random.RandomState(seed)
    img = (rng.randint(20, 60, (h, w, 3))).astype(np.uint8)   # low-amplitude background texture
    for _ in range(n_shapes):
        x, y = int(rng.randint(0, w)), int(rng.randint(0, h))
        r = int(rng.randint(10, 34))
        color = tuple(int(c) for c in rng.randint(0, 255, 3))
        if rng.rand() < 0.5:
            cv2.rectangle(img, (x, y), (min(w - 1, x + r), min(h - 1, y + r)), color, -1)
        else:
            cv2.circle(img, (x, y), r, color, -1)
    return img


def run_self_test():
    ok = True

    def case(name, good):
        nonlocal ok
        ok = ok and good
        print(f"[self-test] {'PASS' if good else 'FAIL'}  {name}")

    base = _textured_image(seed=1)
    h, w = base.shape[:2]

    # 1. CONTAINED (zoom-in): F_live is a magnified central crop of F_LKG -> matched, contained, scale>=1.
    probe = VisualRecoveryProbe()
    probe.update_reference(base, True)
    cx0, cy0, cx1, cy1 = w // 4, h // 4, 3 * w // 4, 3 * h // 4
    crop = base[cy0:cy1, cx0:cx1]
    zoomed = cv2.resize(crop, (w, h), interpolation=cv2.INTER_LINEAR)
    vm_zoom = probe.match(zoomed)
    case(f"contained/zoom-in -> matched+contained+scale>=1.15 "
         f"(matched={vm_zoom.matched} contained={vm_zoom.contained} scale={vm_zoom.scale})",
         vm_zoom.matched and vm_zoom.contained and vm_zoom.scale is not None and vm_zoom.scale >= 1.15)

    # 2. PLANAR-LIKE (nose-to-a-flat-surface): a mild GLOBAL perspective warp of the full frame -> a single
    #    homography explains nearly every match -> high inlier ratio, no crop -> not contained.
    probe2 = VisualRecoveryProbe()
    probe2.update_reference(base, True)
    src_pts = np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    dst_pts = np.float32([[0.06 * w, 0.03 * h], [0.97 * w, 0.0], [1.0 * w, 0.98 * h], [0.02 * w, 1.0 * h]])
    Hp = cv2.getPerspectiveTransform(src_pts, dst_pts)
    planar_live = cv2.warpPerspective(base, Hp, (w, h), borderMode=cv2.BORDER_REPLICATE)
    vm_planar = probe2.match(planar_live)
    case(f"planar-like (flat-surface warp) -> matched+planar_like, not contained "
         f"(matched={vm_planar.matched} planar_like={vm_planar.planar_like} contained={vm_planar.contained})",
         vm_planar.matched and vm_planar.planar_like and not vm_planar.contained)

    # 3. GENUINE PARALLAX: two independently-shifted regions (simulates two depth planes moving
    #    differently) -> matched (enough total inliers), but NO single homography fits both regions well
    #    -> inlier ratio stays below the planar threshold, and scale reads ~1 (no overall zoom).
    probe3 = VisualRecoveryProbe()
    probe3.update_reference(base, True)
    parallax_live = base.copy()
    left = base[:, : w // 2]
    right = base[:, w // 2:]
    shifted_left = np.roll(left, 6, axis=1)
    shifted_right = np.roll(right, -18, axis=0)
    parallax_live[:, : w // 2] = shifted_left
    parallax_live[:, w // 2:] = shifted_right
    vm_parallax = probe3.match(parallax_live)
    case(f"genuine parallax (two depth planes) -> matched, NOT planar_like, scale~=1 "
         f"(matched={vm_parallax.matched} planar_like={vm_parallax.planar_like} scale={vm_parallax.scale})",
         vm_parallax.matched and not vm_parallax.planar_like
         and vm_parallax.scale is not None and 0.7 <= vm_parallax.scale <= 1.3)

    # 4. NO-OVERLAP: a wholly unrelated textured image -> no match.
    probe4 = VisualRecoveryProbe()
    probe4.update_reference(base, True)
    unrelated = _textured_image(seed=99)
    vm_none = probe4.match(unrelated)
    case(f"no-overlap (unrelated frame) -> not matched (inliers={vm_none.inliers})", not vm_none.matched)

    # 5. update_reference only caches on tracked=True; no reference -> has_lkg False, no crash.
    probe5 = VisualRecoveryProbe()
    vm_empty = probe5.match(base)
    probe5.update_reference(base, False)      # NOT tracked -> must NOT cache
    vm_still_empty = probe5.match(base)
    case("update_reference ignores tracked=False (no cache -> has_lkg stays False)",
         not vm_empty.has_lkg and not vm_still_empty.has_lkg)
    probe5.update_reference(base, True)
    vm_now = probe5.match(base)
    case("update_reference caches on tracked=True (self-match -> has_lkg + matched)",
         vm_now.has_lkg and vm_now.matched)

    print(f"\n[self-test] {'ALL PASS' if ok else 'FAILURES PRESENT'}")
    return ok


def main():
    ap = argparse.ArgumentParser(description="CPU-only SIFT visual loss-recovery probe")
    ap.add_argument("--self-test", action="store_true", help="synthetic frame-pair validation, no hardware")
    args = ap.parse_args()
    if args.self_test:
        raise SystemExit(0 if run_self_test() else 1)
    ap.error("nothing to do: pass --self-test")


if __name__ == "__main__":
    main()
