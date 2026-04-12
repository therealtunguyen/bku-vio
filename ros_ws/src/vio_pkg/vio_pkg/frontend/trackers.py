from typing import List, Tuple

import cv2
import numpy as np

from .interfaces import FrontendConfig, IFeatureTracker


class KLTTracker(IFeatureTracker):
    """
    Pyramidal Lucas-Kanade tracker with forward-backward consistency check.

    The forward-backward check tracks points A->B then B->A and rejects
    any point whose round-trip pixel error exceeds *fb_threshold*.  This
    catches tracking failures that the LK status flag alone misses.
    """

    def __init__(self, config: FrontendConfig):
        self._config = config
        self._lk_params = dict(
            winSize=(config.klt_win_size, config.klt_win_size),
            maxLevel=config.klt_max_level,
            criteria=(
                cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
                30,
                0.01,
            ),
        )

    def track(
        self,
        image_prev: np.ndarray,
        image_curr: np.ndarray,
        pts_prev: List[np.ndarray],
    ) -> Tuple[List[np.ndarray], List[int]]:
        if not pts_prev:
            return [], []

        pts_arr = np.array(pts_prev, dtype=np.float32).reshape(-1, 1, 2)

        # --- Forward pass: prev -> curr ---
        pts_fwd, status_fwd, _ = cv2.calcOpticalFlowPyrLK(
            image_prev, image_curr, pts_arr, None, **self._lk_params
        )

        # --- Backward pass: curr -> prev ---
        pts_bwd, status_bwd, _ = cv2.calcOpticalFlowPyrLK(
            image_curr, image_prev, pts_fwd, None, **self._lk_params
        )

        # --- Forward-backward error (L2 per point) ---
        fb_error = np.linalg.norm(
            pts_arr.reshape(-1, 2) - pts_bwd.reshape(-1, 2), axis=1
        )

        # --- Build output ---
        tracked_pts: List[np.ndarray] = []
        status: List[int] = []
        for i in range(len(pts_prev)):
            good = (
                status_fwd[i][0] == 1
                and status_bwd[i][0] == 1
                and fb_error[i] < self._config.fb_threshold
            )
            tracked_pts.append(pts_fwd[i].ravel().astype(np.float32))
            status.append(1 if good else 0)

        return tracked_pts, status
