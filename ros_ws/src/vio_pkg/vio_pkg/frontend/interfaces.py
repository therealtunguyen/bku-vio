from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np


@dataclass
class FrontendConfig:
    """Configuration DTO for all frontend components."""

    # --- Detection ---
    max_features: int = 200
    grid_rows: int = 8
    grid_cols: int = 8
    quality_level: float = 0.01
    min_distance: float = 10.0

    # --- Harris-specific ---
    harris_block_size: int = 2
    harris_k: float = 0.04

    # --- KLT tracking ---
    klt_win_size: int = 21
    klt_max_level: int = 3
    fb_threshold: float = 1.0        # max round-trip pixel error for F-B check

    # --- Feature track lifecycle ---
    max_track_length: int = 15       # mature a track after this many frames
    min_track_length: int = 3        # discard lost tracks shorter than this

    # --- RANSAC outlier rejection ---
    ransac_threshold: float = 1.0    # reprojection threshold for findFundamentalMat


class IFeatureDetector(ABC):
    """Strategy interface for corner/keypoint detectors."""

    @abstractmethod
    def detect(
        self,
        image: np.ndarray,
        mask: Optional[np.ndarray] = None,
    ) -> List[np.ndarray]:
        """
        Detect keypoints in *image*.

        :param image: Grayscale image (H x W, uint8).
        :param mask: Optional binary mask (same size as image).
                     Pixels set to 255 are eligible for detection; 0 means skip.
        :return: List of detected points, each as np.ndarray([x, y]).
        """
        pass


class IFeatureTracker(ABC):
    """Strategy interface for optical-flow / feature trackers."""

    @abstractmethod
    def track(
        self,
        image_prev: np.ndarray,
        image_curr: np.ndarray,
        pts_prev: List[np.ndarray],
    ) -> Tuple[List[np.ndarray], List[int]]:
        """
        Track *pts_prev* from *image_prev* to *image_curr*.

        :param image_prev: Previous grayscale frame.
        :param image_curr: Current grayscale frame.
        :param pts_prev:   Points to track, each as np.ndarray([x, y]).
        :return: Tuple of:
                   - tracked_pts: predicted positions in *image_curr*
                   - status: 1 = successfully tracked, 0 = lost
        """
        pass
