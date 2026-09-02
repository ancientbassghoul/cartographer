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
    """One verdict from matching a live frame against F_LKG (the cached last-known-good frame)."""
    has_lkg: bool                  # a reference frame exists to match against at all
    matched: bool = False          # inliers >= min_inliers
    inliers: int = 0
    contained: bool = False        # F_live is a zoomed-in crop of F_LKG (2a: nose closer to the same surface)
    planar_like: bool = False      # high inlier ratio -> nose-to-a-flat-surface (2b)
    scale: float | None = None     # homography linear scale (LKG->live); only meaningful when matched
    debug_image: "np.ndarray | None" = None   # BGR canvas (F_LKG | live + inliers), only when debug=True


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
        # Session 52: which frame `_lkg` actually IS -- "live" (the tick's own frame, legacy behaviour),
        # "slam:<frame_id>" (the exact frame the plan was computed from, pulled from run_explore's ring),
        # or "live(aged-out)" (the plan's frame_id fell off the ring -- degraded to live, logged LOUD by
        # the caller). A LABEL ONLY: never read by match(), just surfaced on the debug banner.
        self._lkg_src = "none"
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
        LABEL for which frame this is ("slam:<frame_id>" | "live" | "live(aged-out)") -- stored verbatim for
        the debug banner (`_compose_debug`) and never consulted by matching logic. Cheap (a copy); SIFT only
        runs at match() time.

        Session 51: returns True when it actually STORED a new reference (so run_explore can invalidate
        its own per-tick match memo on that exact edge rather than duplicating this condition), else
        False. Storing also drops the cached reference features — see `_lkg_feats`."""
        if tracked and frame is not None:
            self._lkg = frame.copy()
            self._lkg_feats = None      # new reference -> the memoised SIFT no longer describes it
            self._lkg_src = str(src)
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
    def _pad_to_height(img, target_h):
        """Zero-pad `img` at the bottom to `target_h` rows — NEVER resize/scale (IMAGE INTEGRITY,
        CLAUDE.md). A no-op when `img` is already tall enough."""
        h = img.shape[0]
        if h >= target_h:
            return img
        pad_shape = (target_h - h,) + img.shape[1:]
        return np.vstack([img, np.zeros(pad_shape, dtype=img.dtype)])

    def _compose_debug(self, frame, out, kp0=None, kp1=None, good=None, mask=None, banner=None):
        """Build the operator canvas: F_LKG (left) | live (right), with the RANSAC INLIER
        correspondences drawn when a homography survived, and a two-line header.

        IMAGE INTEGRITY (CLAUDE.md): NOTHING here resizes, crops or downscales either frame. The canvas
        width is exactly w_lkg + w_live. If the two frames differ in height (they do not in this
        pipeline — both are the 512x288 transport frame) the SHORTER one is zero-PADDED, never scaled.

        Returns a BGR ndarray.
        """
        lkg = self._lkg
        if kp0 is not None and kp1 is not None and good is not None and mask is not None:
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
        line1 = banner or ""
        line2 = (f"has_lkg={out.has_lkg} matched={out.matched} inliers={out.inliers} "
                 f"contained={out.contained} planar_like={out.planar_like} scale={scale_txt} "
                 f"lkg_src={self._lkg_src}")
        cv2.putText(banner_strip, line1, (6, 13), BANNER_FONT, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(banner_strip, line2, (6, 29), BANNER_FONT, 0.4, (200, 200, 200), 1, cv2.LINE_AA)
        canvas = np.vstack([banner_strip, body])
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
        canvas; it never affects the match itself."""
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
