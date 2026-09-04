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

# Session 49: operator-visible debug canvas (F_LKG | LIVE + drawn RANSAC inliers), built only when
# match(..., debug=True) is passed -- see VisualRecoveryProbe._compose_debug.
BANNER_H = 34          # px of black header strip; NEVER overlays image content
BANNER_FONT = cv2.FONT_HERSHEY_SIMPLEX


@dataclass
class VisualMatch:
    """One verdict from matching a live frame against F_LKG (the cached last-known-good frame).

    Session 57: `scale` (the homography's linear determinant) is a poor direction estimator on its
    own -- diagnosed off a real flight where it read 1.44 -> 1.31 -> 0.53 -> 0.59 -> 0.67 -> 1.33 ->
    1.85 within one second on ~30 inliers, because it conflates perspective foreshortening with a
    weak-fit homography. `spread_lkg`/`spread_live`/`size_ratio`/`closer` add a LINEAR, RANSAC-inlier-
    only measure (RMS distance from centroid) of how much screen area the matched features actually
    span in each frame -- far less sensitive to one stray inlier, and directly answers "which frame is
    the feature cluster bigger in" without going through a projective determinant at all. `scale` is
    kept (never removed) so the two estimators can be compared side by side on real flights."""
    has_lkg: bool                  # a reference frame exists to match against at all
    matched: bool = False          # inliers >= min_inliers
    inliers: int = 0
    contained: bool = False        # F_live is a zoomed-in crop of F_LKG (2a: nose closer to the same surface)
    planar_like: bool = False      # high inlier ratio -> nose-to-a-flat-surface (2b)
    scale: float | None = None     # homography linear scale (LKG->live); only meaningful when matched
    # --- session 57 ---
    spread_lkg: float | None = None    # RMS px distance of INLIER keypoints from their centroid, in F_LKG
    spread_live: float | None = None   # same, in the live frame
    size_ratio: float | None = None    # spread_live / spread_lkg; None unless both spreads are usable
    closer: str = "UNKNOWN"            # "LIVE" | "LKG" | "EQUAL" | "UNKNOWN"
    debug_image: "np.ndarray | None" = None   # BGR canvas (F_LKG | live + inliers), only when debug=True


class VisualRecoveryProbe:
    """Caches F_LKG (the most recent frame SLAM was TRACKING on) and matches later frames against it.
    See the module docstring for why this is CPU-only SIFT, copied (not imported) from
    `benchmark_detectors.SiftDetector`."""

    def __init__(self, *, min_inliers=SIFT_MIN_INLIERS, planar_inlier_ratio=0.85,
                 contain_margin_frac=0.02,
                 size_ratio_hi: float = 1.25, size_ratio_lo: float = 0.80,
                 size_min_inliers: int = 20):
        self.min_inliers = int(min_inliers)
        self.planar_inlier_ratio = float(planar_inlier_ratio)
        self.contain_margin_frac = float(contain_margin_frac)
        # --- session 57: inlier-spread direction verdict thresholds, see `match()`'s `closer` field ---
        self.size_ratio_hi = float(size_ratio_hi)
        self.size_ratio_lo = float(size_ratio_lo)
        self.size_min_inliers = int(size_min_inliers)
        self.sift = cv2.SIFT_create()
        self.matcher = cv2.BFMatcher(cv2.NORM_L2)
        self._lkg = None    # cached BGR frame (last known good — SLAM was TRACKING when it was captured)
        # Session 52: which frame `_lkg` actually IS -- "live" (the tick's own frame, legacy behaviour) or
        # "slam:<frame_id>" (the exact frame the plan was computed from, pulled from run_explore's ring). A
        # LABEL ONLY: never read by match(), just surfaced on the debug banner. Session 56: an aged-out plan
        # frame no longer degrades this to a fake "live" reference (see run_explore) -- the caller simply
        # does not call update_reference that tick, so `_lkg_src` keeps naming whatever reference is still
        # held, and `_lkg_t` (below) lets the banner show how OLD it now is.
        self._lkg_src = "none"
        self._lkg_t = None   # monotonic timestamp of the last successful update_reference (age display)
        # Session 51: LAZY, memoised SIFT for the REFERENCE half of a match. `None` = not computed yet;
        # once computed it holds the (keypoints, descriptors) tuple for the CURRENT `_lkg` and is reused
        # until a new reference replaces it. The flag is the TUPLE SLOT, never the descriptors: a
        # featureless reference legitimately yields (kp, None), and keying on the descriptors would
        # recompute it forever.
        self._lkg_feats = None

    def update_reference(self, frame, tracked, src: str = "live"):
        """Cache `frame` as F_LKG whenever `tracked` is True. Called every tick from run_explore, gated on
        the same `plan.get("plan_valid")` boundary session-34's own cached-pose snapshot uses.

        Session 52: `frame` is now WHICHEVER frame the caller resolved as the true reference -- run_explore
        keeps a short ring of recent (frame_id, frame) pairs and, when possible, passes the EXACT frame the
        SLAM plan was computed from (this flight measured solve latency up to 14.9s, so "the live frame at
        this tick" and "the frame the plan describes" can be seconds apart). `src` is a caller-supplied
        LABEL for which frame this is ("slam:<frame_id>" | "live") -- stored verbatim for the debug banner
        (`_compose_debug`) and never consulted by matching logic. Cheap (a copy); SIFT only runs at
        match() time.

        Session 51: returns True when it actually STORED a new reference (so run_explore can invalidate
        its own per-tick match memo on that exact edge rather than duplicating this condition), else
        False. Storing also drops the cached reference features — see `_lkg_feats`.

        Session 56: a `tracked=False` call (e.g. the caller's plan frame aged out of its ring) touches
        NEITHER `_lkg`, `_lkg_src`, `_lkg_t` NOR `_lkg_feats` -- the previous reference is kept exactly as
        it was, memoised features included, so a run of age-outs costs nothing extra and never churns the
        SIFT memo."""
        if tracked and frame is not None:
            self._lkg = frame.copy()
            self._lkg_feats = None      # new reference -> the memoised SIFT no longer describes it
            self._lkg_src = str(src)
            self._lkg_t = time.monotonic()
            return True
        return False

    def _reference_keypoints(self):
        """The CURRENT reference's (keypoints, descriptors), computed at most once per reference.

        Session 51: `match()` used to recompute SIFT on `self._lkg` on EVERY call. F_LKG only changes in
        `update_reference`, and during a loss it never changes at all (it only caches while `plan_valid`),
        so a 12s loss-recovery grace at ~32Hz was repeating the identical computation hundreds of times.
        The live frame's SIFT still runs per call -- that frame really is new every time."""
        if self._lkg_feats is None:
            self._lkg_feats = self._keypoints(self._lkg)
        return self._lkg_feats

    def _keypoints(self, bgr):
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY) if bgr.ndim == 3 else bgr
        return self.sift.detectAndCompute(gray, None)

    @staticmethod
    def _inlier_spread(points: "np.ndarray") -> float | None:
        """RMS distance of `points` (N,2 float) from their centroid, in pixels.

        Returns None when fewer than 2 points, or when the result is not finite.
        A LINEAR measure of how much screen area the matched features span -- directly comparable
        to the homography `scale`, but far less sensitive to one stray inlier than a hull area.
        """
        if points is None or len(points) < 2:
            return None
        centroid = points.mean(axis=0)
        d = np.sqrt(np.mean(np.sum((points - centroid) ** 2, axis=1)))
        val = float(d)
        return val if np.isfinite(val) else None

    @staticmethod
    def _pad_to_height(img, target_h):
        """Zero-pad `img` at the bottom to `target_h` rows — NEVER resize/scale (IMAGE INTEGRITY,
        CLAUDE.md). A no-op when `img` is already tall enough."""
        h = img.shape[0]
        if h >= target_h:
            return img
        pad_shape = (target_h - h,) + img.shape[1:]
        return np.vstack([img, np.zeros(pad_shape, dtype=img.dtype)])

    @staticmethod
    def _pad_to_width(img, target_w):
        """Zero-pad `img` at the right to `target_w` columns — NEVER resize/scale (IMAGE INTEGRITY,
        CLAUDE.md). A no-op when `img` is already wide enough. Session 60 (C8): the STACKED layout
        pads columns (not rows) since the two frames sit one above the other."""
        w = img.shape[1]
        if w >= target_w:
            return img
        pad_shape = (img.shape[0], target_w - w) + img.shape[2:]
        return np.hstack([img, np.zeros(pad_shape, dtype=img.dtype)])

    def _compose_debug(self, frame, out, kp0=None, kp1=None, good=None, mask=None, banner=None,
                       *, stacked: bool = False):
        """Build the operator canvas: F_LKG | live, with the RANSAC INLIER correspondences drawn when
        a homography survived, and a two-line header.

        IMAGE INTEGRITY (CLAUDE.md): NOTHING here resizes, crops or downscales either frame.

        `stacked` (session 60, C8; keyword-only, default False preserves the original layout exactly):
          - False (default): SIDE-BY-SIDE, F_LKG (left) | live (right) via `cv2.drawMatches`. Canvas
            width is exactly w_lkg + w_live; a height mismatch is zero-padded (never scaled). This is
            still the shape saved to the PNG evidence trail — UNCHANGED.
          - True: STACKED, F_LKG (top) / live (bottom) — the visualizer's LKG panel column (see
            visualizer.py PANEL_W x MAP_SIZE) is tall and narrow, not wide and short, so a second
            canvas in this orientation is composed for that column only. `cv2.drawMatches` only ever
            builds a side-by-side canvas, so the inlier lines are drawn by hand here: for each inlier,
            a line from (x_lkg, y_lkg) to (x_live, y_live + h_lkg). A width mismatch is zero-padded
            (never scaled).

        Returns a BGR ndarray.
        """
        lkg = self._lkg
        if stacked:
            w_max = max(lkg.shape[1], frame.shape[1])
            top = self._pad_to_width(lkg, w_max)
            bot = self._pad_to_width(frame, w_max)
            body = np.vstack([top, bot])
            if kp0 is not None and kp1 is not None and good is not None and mask is not None:
                h_lkg_body = lkg.shape[0]
                for m, keep in zip(good, mask):
                    if not keep:
                        continue
                    x0, y0 = kp0[m.queryIdx].pt
                    x1, y1 = kp1[m.trainIdx].pt
                    cv2.line(body, (int(round(x0)), int(round(y0))),
                            (int(round(x1)), int(round(y1)) + h_lkg_body), (0, 255, 0), 1, cv2.LINE_AA)
        elif kp0 is not None and kp1 is not None and good is not None and mask is not None:
            # cv2.drawMatches already allocates a (max(h1,h2), w1+w2) canvas and top-left-aligns each
            # image into its half — the same zero-pad-not-scale behaviour _pad_to_height gives the
            # fallback branch below, done internally.
            body = cv2.drawMatches(lkg, kp0, frame, kp1, good, None,
                                   matchesMask=mask.astype(np.uint8).tolist(),
                                   flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        else:
            h_max = max(lkg.shape[0], frame.shape[0])
            body = np.hstack([self._pad_to_height(lkg, h_max), self._pad_to_height(frame, h_max)])
        banner_strip = np.zeros((BANNER_H, body.shape[1], 3), dtype=np.uint8)
        scale_txt = f"{out.scale:.2f}" if out.scale is not None else "n/a"
        ratio_txt = f"{out.size_ratio:.2f}" if out.size_ratio is not None else "n/a"
        line1 = banner or ""
        age_txt = f"{time.monotonic() - self._lkg_t:.1f}s" if self._lkg_t is not None else "n/a"
        line2 = (f"has_lkg={out.has_lkg} matched={out.matched} inliers={out.inliers} "
                 f"contained={out.contained} planar_like={out.planar_like} scale={scale_txt} "
                 f"size={ratio_txt} closer={out.closer} "
                 f"lkg_src={self._lkg_src} age={age_txt}")
        cv2.putText(banner_strip, line1, (6, 13), BANNER_FONT, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(banner_strip, line2, (6, 29), BANNER_FONT, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
        canvas = np.vstack([banner_strip, body])
        if stacked:
            h_lkg = lkg.shape[0]
            cv2.putText(canvas, "F_LKG (reference)", (6, BANNER_H + h_lkg - 8), BANNER_FONT, 0.45,
                       (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(canvas, "LIVE", (6, canvas.shape[0] - 8), BANNER_FONT, 0.45,
                       (0, 255, 255), 1, cv2.LINE_AA)
        else:
            w_lkg = lkg.shape[1]
            cv2.putText(canvas, "F_LKG (reference)", (6, canvas.shape[0] - 8), BANNER_FONT, 0.45,
                       (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(canvas, "LIVE", (w_lkg + 6, canvas.shape[0] - 8), BANNER_FONT, 0.45,
                       (0, 255, 255), 1, cv2.LINE_AA)
        return canvas

    def match(self, frame, debug: bool = False, banner: str | None = None) -> VisualMatch:
        """SIFT+RANSAC-homography match of `frame` against the cached F_LKG. H maps LKG -> live (same
        src/dst convention as benchmark_detectors.SiftDetector: src=reference keypoints, dst=frame
        keypoints), so `scale = sqrt(|det(H[:2,:2])|)` reads >1 when the live frame shows a MAGNIFIED
        (closer) view of what F_LKG covered — the natural "moved closer" signal. `contained` needs the
        OPPOSITE direction (does the live frame's own extent sit entirely inside a crop of F_LKG?), so it
        explicitly inverts H rather than reusing it raw.

        Session 49: `debug=True` (default False, zero extra cost) additionally composes an operator-
        readable canvas (F_LKG | live + drawn inlier correspondences, see `_compose_debug`) on
        `out.debug_image` — on EVERY return path that has an F_LKG, including every failure path (no
        descriptors, <4 good matches, a degenerate homography, or a match below `min_inliers`), since
        seeing exactly where/why the match failed is the whole point of the window. `banner` is an
        optional caller-supplied context line (e.g. FSM state + plan status) rendered verbatim on the
        canvas; it never affects the match itself.

        Session 57: once matched, also computes `spread_lkg`/`spread_live` (RMS distance of the RANSAC
        INLIER points from their centroid, in each frame) and derives `size_ratio = spread_live /
        spread_lkg` and a `closer` verdict ("LIVE" | "LKG" | "EQUAL" | "UNKNOWN"). See the `VisualMatch`
        docstring for why this linear spread measure was added alongside `scale` rather than replacing
        it."""
        if self._lkg is None or frame is None:
            return VisualMatch(has_lkg=self._lkg is not None)
        kp0, des0 = self._reference_keypoints()      # session 51: memoised per reference, not per call
        kp1, des1 = self._keypoints(frame)
        out = VisualMatch(has_lkg=True)
        if des0 is None or des1 is None or len(kp0) < 2 or len(kp1) < 2:
            if debug:
                out.debug_image = self._compose_debug(frame, out, banner=banner)
            return out
        good = []
        for m_n in self.matcher.knnMatch(des0, des1, k=2):
            if len(m_n) == 2 and m_n[0].distance < SIFT_RATIO * m_n[1].distance:
                good.append(m_n[0])
        if len(good) < 4:
            if debug:
                out.debug_image = self._compose_debug(frame, out, banner=banner)
            return out
        src = np.float32([kp0[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)   # F_LKG points
        dst = np.float32([kp1[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)   # F_live points
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)
        if H is None or mask is None:
            if debug:
                out.debug_image = self._compose_debug(frame, out, banner=banner)
            return out
        mask = mask.ravel().astype(bool)
        inliers = int(mask.sum())
        out.inliers = inliers
        out.matched = inliers >= self.min_inliers
        if not out.matched:
            if debug:
                out.debug_image = self._compose_debug(frame, out, kp0=kp0, kp1=kp1, good=good, mask=mask,
                                                       banner=banner)
            return out
        out.planar_like = (inliers / float(len(good))) >= self.planar_inlier_ratio
        # Session 57: direction verdict from RANSAC-inlier spread (see VisualMatch docstring, Finding 3
        # in the session-57 spec -- `scale` alone is too noisy to gate a physical back-off on).
        src_in = src.reshape(-1, 2)[mask]      # F_LKG inlier points
        dst_in = dst.reshape(-1, 2)[mask]      # live inlier points
        out.spread_lkg = self._inlier_spread(src_in)
        out.spread_live = self._inlier_spread(dst_in)
        if (out.spread_lkg is not None and out.spread_live is not None and out.spread_lkg > 1e-6):
            out.size_ratio = out.spread_live / out.spread_lkg
            if inliers < self.size_min_inliers:
                out.closer = "UNKNOWN"
            elif out.size_ratio > self.size_ratio_hi:
                out.closer = "LIVE"
            elif out.size_ratio < self.size_ratio_lo:
                out.closer = "LKG"
            else:
                out.closer = "EQUAL"
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
        if debug:
            out.debug_image = self._compose_debug(frame, out, kp0=kp0, kp1=kp1, good=good, mask=mask,
                                                   banner=banner)
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

    # ---- SESSION 49 — debug canvas (F_LKG | LIVE + drawn inliers) --------------------------------------
    # Operator ask: see what the CV probe is matching against, in its own window. All image handling
    # stays in this module per its own docstring rule.

    # 6. NO-RESIZE CONTRACT: a genuine match (the CONTAINED case, so it exercises the cv2.drawMatches
    #    branch) -> the canvas is EXACTLY w_lkg + w_live wide and max(h_lkg, h_live) + BANNER_H tall.
    #    IMAGE INTEGRITY (CLAUDE.md): this is the assertion that proves nothing was resized anywhere.
    probe6 = VisualRecoveryProbe()
    probe6.update_reference(base, True)
    vm6 = probe6.match(zoomed, debug=True)
    shape6 = None if vm6.debug_image is None else vm6.debug_image.shape
    case(f"debug=True (matched case) -> canvas is EXACTLY w_lkg+w_live wide, h_max+BANNER_H tall "
         f"(shape={shape6})",
         vm6.debug_image is not None
         and vm6.debug_image.shape[1] == base.shape[1] + zoomed.shape[1]
         and vm6.debug_image.shape[0] == max(base.shape[0], zoomed.shape[0]) + BANNER_H)

    # 7. DEFAULT PATH PAYS NOTHING: debug=False -> debug_image is None, and debug rendering never
    #    perturbs the verdict (every OTHER field is identical to the debug=True call on the same input).
    probe7 = VisualRecoveryProbe()
    probe7.update_reference(base, True)
    vm7_debug = probe7.match(zoomed, debug=True)
    vm7_default = probe7.match(zoomed)
    case("debug=False -> debug_image is None, and verdict fields are IDENTICAL to the debug=True call "
         "(debug rendering never perturbs the match)",
         vm7_default.debug_image is None
         and vm7_default.has_lkg == vm7_debug.has_lkg
         and vm7_default.matched == vm7_debug.matched
         and vm7_default.inliers == vm7_debug.inliers
         and vm7_default.contained == vm7_debug.contained
         and vm7_default.planar_like == vm7_debug.planar_like
         and vm7_default.scale == vm7_debug.scale)

    # 8. FAILURE PATHS STILL DRAW: an unrelated image (not matched) still composes a canvas -- seeing
    #    exactly where/why the match FAILED is the whole point of the window.
    probe8 = VisualRecoveryProbe()
    probe8.update_reference(base, True)
    vm8 = probe8.match(_textured_image(seed=99), debug=True)
    case(f"failure path (unrelated image, not matched) still draws a debug canvas "
         f"(matched={vm8.matched}, debug_image={'set' if vm8.debug_image is not None else 'None'})",
         vm8.matched is False and vm8.debug_image is not None)

    # 9. NO REFERENCE: nothing cached yet -> debug=True must not crash, and there is nothing to draw.
    probe9 = VisualRecoveryProbe()
    vm9 = probe9.match(base, debug=True)
    case("no F_LKG cached -> debug=True yields debug_image=None (nothing to compare against), no crash",
         vm9.debug_image is None and vm9.has_lkg is False)

    # 10. BANNER: caller-supplied context line renders without raising; banner=None is also safe, and
    #     the banner text never changes the canvas SIZE (only its header pixels).
    probe10 = VisualRecoveryProbe()
    probe10.update_reference(base, True)
    vm10a = probe10.match(zoomed, debug=True, banner="HOLD_LOST / PLAN-LOST")
    vm10b = probe10.match(zoomed, debug=True, banner=None)
    case("banner text renders without raising; banner=None is safe; banner never changes canvas size",
         vm10a.debug_image is not None and vm10b.debug_image is not None
         and vm10a.debug_image.shape == vm10b.debug_image.shape)

    # ---- SESSION-60 LKG PANEL — C8: _compose_debug gains stacked=True (F_LKG-over-LIVE) for the
    # visualizer's tall/narrow panel column, WITHOUT disturbing the default side-by-side shape that
    # still feeds the PNG evidence trail (unchanged). ------------------------------------------------
    probe11 = VisualRecoveryProbe()
    probe11.update_reference(base, True)
    vm11 = probe11.match(zoomed, debug=True)
    canvas_default = probe11._compose_debug(zoomed, vm11)
    canvas_explicit_false = probe11._compose_debug(zoomed, vm11, stacked=False)
    case(f"(60-1) stacked=False reproduces today's side-by-side shape exactly "
         f"(default={canvas_default.shape} explicit={canvas_explicit_false.shape})",
         canvas_default.shape == canvas_explicit_false.shape
         and canvas_default.shape[1] == base.shape[1] + zoomed.shape[1])

    canvas_stacked = probe11._compose_debug(zoomed, vm11, stacked=True)
    h_lkg11, w_lkg11 = base.shape[:2]
    h_live11, w_live11 = zoomed.shape[:2]
    case(f"(60-2) stacked=True composes F_LKG-over-LIVE: taller than wide, height >= h_lkg+h_live, "
         f"width == max(w_lkg, w_live) (shape={canvas_stacked.shape})",
         canvas_stacked.shape[0] > canvas_stacked.shape[1]
         and canvas_stacked.shape[0] >= h_lkg11 + h_live11
         and canvas_stacked.shape[1] == max(w_lkg11, w_live11))

    # ---- SESSION 51 — lazy, memoised SIFT on the REFERENCE half -----------------------------------------
    # match() used to recompute SIFT on _lkg every single call; during a loss the reference is frozen
    # (update_reference only caches while plan_valid), so a 12s grace at ~32Hz repeated it ~380 times.

    class _CountingProbe(VisualRecoveryProbe):
        """Counts _keypoints() calls so the memo can be asserted on behaviour, not on internals."""
        def __init__(self, **kw):
            super().__init__(**kw)
            self.kp_calls = 0

        def _keypoints(self, bgr):
            self.kp_calls += 1
            return super()._keypoints(bgr)

    # 11. The reference's SIFT is computed ONCE and reused; only the live frame's is paid per call.
    p11 = _CountingProbe()
    p11.update_reference(base, True)
    p11.match(zoomed)
    after_first = p11.kp_calls               # reference + live = 2
    p11.match(zoomed)
    p11.match(zoomed)
    after_three = p11.kp_calls               # + live only, twice = 4
    case(f"reference SIFT memoised: 1st match costs 2 keypoint passes, each later match costs 1 "
         f"(calls after 1={after_first}, after 3={after_three})",
         after_first == 2 and after_three == 4)

    # 12. A NEW reference invalidates the memo (otherwise we would match against a stale frame's features).
    p11.update_reference(_textured_image(seed=7), True)
    before_new = p11.kp_calls
    p11.match(zoomed)
    case(f"a new update_reference re-computes the reference features "
         f"(+{p11.kp_calls - before_new} passes, expected 2)", p11.kp_calls - before_new == 2)

    # 13. THE TRAP: a FEATURELESS reference yields (kp, None) legitimately. The memo flag is the tuple
    #     slot, so it must NOT recompute forever just because the descriptors are None.
    p13 = _CountingProbe()
    p13.update_reference(np.zeros((120, 160, 3), dtype=np.uint8), True)   # flat black -> no descriptors
    p13.match(base)
    blank_first = p13.kp_calls
    p13.match(base)
    blank_second = p13.kp_calls
    case(f"featureless reference (des=None) is still memoised, not recomputed forever "
         f"(after 1={blank_first}, after 2={blank_second})",
         p13._lkg_feats is not None and (blank_second - blank_first) <= 1)

    # 14. update_reference reports whether it actually STORED (run_explore keys its own memo off this).
    p14 = VisualRecoveryProbe()
    stored_no = p14.update_reference(base, False)
    stored_none = p14.update_reference(None, True)
    stored_yes = p14.update_reference(base, True)
    case("update_reference returns True only when it really cached a new reference",
         stored_no is False and stored_none is False and stored_yes is True)

    # ---- SESSION 52 — F_LKG selected by frame IDENTITY, not by tick freshness ---------------------------
    # `src` is a caller-supplied LABEL (run_explore resolves it from a short frame-id ring against the
    # plan's own frame_id) that must be visible for debugging but must NEVER influence matching.

    # (52-lkg-1) src is recorded and defaults safely; a tracked=False call touches neither _lkg nor _lkg_src.
    frameA, frameB, frameC = _textured_image(seed=21), _textured_image(seed=22), _textured_image(seed=23)
    p_lkg1 = VisualRecoveryProbe()
    p_lkg1.update_reference(frameA, True)
    case(f"(52-lkg-1a) default src is 'live' (src={p_lkg1._lkg_src!r})", p_lkg1._lkg_src == "live")
    p_lkg1.update_reference(frameB, True, src="slam:42")
    case(f"(52-lkg-1b) explicit src is stored verbatim (src={p_lkg1._lkg_src!r})",
         p_lkg1._lkg_src == "slam:42")
    p_lkg1.update_reference(frameC, False, src="slam:43")
    lkg_still_frameB = bool(np.array_equal(p_lkg1._lkg, frameB))
    case(f"(52-lkg-1c) tracked=False stores NEITHER frame nor src "
         f"(lkg_still_frameB={lkg_still_frameB}, src={p_lkg1._lkg_src!r})",
         lkg_still_frameB and p_lkg1._lkg_src == "slam:42")

    # (52-lkg-2) src never affects matching: two probes fed the IDENTICAL reference/live pair, differing
    # only in the src label, must agree on every match verdict field.
    p_lkg2_live = VisualRecoveryProbe()
    p_lkg2_live.update_reference(base, True, src="live")
    p_lkg2_slam = VisualRecoveryProbe()
    p_lkg2_slam.update_reference(base, True, src="slam:9")
    vm_lkg2_live = p_lkg2_live.match(zoomed)
    vm_lkg2_slam = p_lkg2_slam.match(zoomed)
    case("(52-lkg-2) src label never affects the match verdict (live vs slam:9 agree in full)",
         vm_lkg2_live.matched == vm_lkg2_slam.matched
         and vm_lkg2_live.inliers == vm_lkg2_slam.inliers
         and vm_lkg2_live.contained == vm_lkg2_slam.contained
         and vm_lkg2_live.planar_like == vm_lkg2_slam.planar_like
         and vm_lkg2_live.scale == vm_lkg2_slam.scale)

    # (52-lkg-3) the debug banner names the reference frame's src. cv2.putText draws to pixels (not
    # OCR-able), so intercept the text ARGUMENTS the same way the module composes them, capturing exactly
    # what _compose_debug hands to cv2.putText.
    p_lkg3 = VisualRecoveryProbe()
    p_lkg3.update_reference(base, True, src="slam:777")
    _put_text_calls = []
    _real_put_text = cv2.putText

    def _spy_put_text(img, text, *rest, **kw):
        _put_text_calls.append(text)
        return _real_put_text(img, text, *rest, **kw)

    cv2.putText = _spy_put_text
    try:
        vm_lkg3 = p_lkg3.match(zoomed, debug=True)
    finally:
        cv2.putText = _real_put_text
    banner_has_src = (vm_lkg3.debug_image is not None
                       and any("slam:777" in t for t in _put_text_calls))
    case(f"(52-lkg-3) debug banner names the reference src (lkg_src={p_lkg3._lkg_src!r})", banner_has_src)

    # ---- SESSION 56 F_LKG AGE-OUT — a tracked=False call (the caller's plan frame aged out of its ring)
    # must KEEP the previous reference untouched, not silently substitute whatever frame WAS live. Also
    # proves the memoised SIFT features on the kept reference are not churned by the no-op call.
    p56_1 = VisualRecoveryProbe()
    p56_1.update_reference(frameA, True)          # store frame A as the reference
    p56_1.match(frameB)                            # force the reference SIFT memo to be computed
    memo_before = p56_1._lkg_feats
    stored_56 = p56_1.update_reference(frameB, False)   # caller's age-out branch: tracked=False
    lkg_still_frameA = bool(np.array_equal(p56_1._lkg, frameA))
    memo_unchanged = p56_1._lkg_feats is memo_before
    case(f"(56-lkg-1) update_reference_false_keeps_previous: tracked=False returns False, keeps F_LKG "
         f"unchanged, and does NOT invalidate the reference SIFT memo "
         f"(stored={stored_56}, lkg_still_frameA={lkg_still_frameA}, memo_unchanged={memo_unchanged})",
         stored_56 is False and lkg_still_frameA and memo_unchanged)

    # ---- SESSION 57 SPREAD RATIO — direction-aware "closer" verdict from RANSAC-inlier spread, in
    # place of the noisy homography `scale` (Finding 3: same flight, same second, ~30 inliers, `scale`
    # read 1.44 -> 1.31 -> 0.53 -> 0.59 -> 0.67 -> 1.33 -> 1.85). ------------------------------------------

    ref57 = _textured_image(seed=7)
    h57, w57 = ref57.shape[:2]
    center57 = (w57 / 2.0, h57 / 2.0)

    # 57-1/2. zoom_in / zoom_out: a KNOWN-scale affine warp about the frame centre (no source resize --
    # cv2.warpAffine renders the magnified/shrunk content directly into a same-size canvas).
    zoom_factor = 1.6
    M_in = cv2.getRotationMatrix2D(center57, 0, zoom_factor)
    live_zoom_in = cv2.warpAffine(ref57, M_in, (w57, h57), borderMode=cv2.BORDER_REPLICATE)
    M_out = cv2.getRotationMatrix2D(center57, 0, 1.0 / zoom_factor)
    live_zoom_out = cv2.warpAffine(ref57, M_out, (w57, h57), borderMode=cv2.BORDER_REPLICATE)

    probe57_in = VisualRecoveryProbe()
    probe57_in.update_reference(ref57, True)
    vm57_in = probe57_in.match(live_zoom_in)
    case(f"(57-1) zoom_in -> matched, size_ratio>1.0, closer=LIVE "
         f"(matched={vm57_in.matched} size_ratio={vm57_in.size_ratio} closer={vm57_in.closer})",
         vm57_in.matched and vm57_in.size_ratio is not None and vm57_in.size_ratio > 1.0
         and vm57_in.closer == "LIVE")

    probe57_out = VisualRecoveryProbe()
    probe57_out.update_reference(ref57, True)
    vm57_out = probe57_out.match(live_zoom_out)
    case(f"(57-2) zoom_out -> matched, size_ratio<1.0, closer=LKG "
         f"(matched={vm57_out.matched} size_ratio={vm57_out.size_ratio} closer={vm57_out.closer})",
         vm57_out.matched and vm57_out.size_ratio is not None and vm57_out.size_ratio < 1.0
         and vm57_out.closer == "LKG")

    # 57-3. identical: reference matched against itself -> size_ratio ~1, closer=EQUAL.
    probe57_id = VisualRecoveryProbe()
    probe57_id.update_reference(ref57, True)
    vm57_id = probe57_id.match(ref57)
    case(f"(57-3) identical frame -> size_ratio in [0.95, 1.05], closer=EQUAL "
         f"(size_ratio={vm57_id.size_ratio} closer={vm57_id.closer})",
         vm57_id.size_ratio is not None and 0.95 <= vm57_id.size_ratio <= 1.05
         and vm57_id.closer == "EQUAL")

    # 57-4. unmatched_is_unknown: no correspondence at all -> size_ratio/spreads None, closer=UNKNOWN.
    probe57_un = VisualRecoveryProbe()
    probe57_un.update_reference(ref57, True)
    vm57_un = probe57_un.match(_textured_image(seed=123))
    case(f"(57-4) unmatched -> matched=False, size_ratio=None, closer=UNKNOWN, spreads None "
         f"(matched={vm57_un.matched} size_ratio={vm57_un.size_ratio} closer={vm57_un.closer} "
         f"spread_lkg={vm57_un.spread_lkg} spread_live={vm57_un.spread_live})",
         vm57_un.matched is False and vm57_un.size_ratio is None and vm57_un.closer == "UNKNOWN"
         and vm57_un.spread_lkg is None and vm57_un.spread_live is None)

    # 57-5. low_inliers_is_unknown: an absurdly high size_min_inliers forces UNKNOWN even on a clean
    # match, while size_ratio itself is still populated (weak CV must not fabricate certainty).
    probe57_lo = VisualRecoveryProbe(size_min_inliers=10_000)
    probe57_lo.update_reference(ref57, True)
    vm57_lo = probe57_lo.match(live_zoom_in)
    case(f"(57-5) size_min_inliers=10000 -> closer=UNKNOWN despite a normal match, size_ratio populated "
         f"(matched={vm57_lo.matched} closer={vm57_lo.closer} size_ratio={vm57_lo.size_ratio})",
         vm57_lo.matched and vm57_lo.closer == "UNKNOWN" and vm57_lo.size_ratio is not None)

    # 57-6. spread_none_on_degenerate: _inlier_spread returns None for <2 points.
    one_pt = np.array([[1.0, 2.0]])
    empty_pt = np.zeros((0, 2))
    case("(57-6) _inlier_spread(1 point)=None, _inlier_spread(empty)=None",
         VisualRecoveryProbe._inlier_spread(one_pt) is None
         and VisualRecoveryProbe._inlier_spread(empty_pt) is None)

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
