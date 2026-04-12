from typing import List, Optional

import cv2
import numpy as np

from .interfaces import FrontendConfig, IFeatureDetector


class ShiTomasiDetector(IFeatureDetector):
    """
    Shi-Tomasi corner detector via cv2.goodFeaturesToTrack.

    Preferred default for MSCKF — directly maximises trackability
    (smallest eigenvalue of the gradient matrix).
    """

    def __init__(self, config: FrontendConfig):
        self._config = config

    def detect(
        self,
        image: np.ndarray,
        mask: Optional[np.ndarray] = None,
    ) -> List[np.ndarray]:
        corners = cv2.goodFeaturesToTrack(
            image,
            maxCorners=self._config.max_features,
            qualityLevel=self._config.quality_level,
            minDistance=self._config.min_distance,
            mask=mask,
        )
        if corners is None:
            return []
        return [c.ravel().astype(np.float32) for c in corners]


class HarrisDetector(IFeatureDetector):
    """
    Harris corner detector via cv2.goodFeaturesToTrack with useHarrisDetector=True.

    Interchangeable with ShiTomasiDetector thanks to the Strategy interface.
    """

    def __init__(self, config: FrontendConfig):
        self._config = config

    def detect(
        self,
        image: np.ndarray,
        mask: Optional[np.ndarray] = None,
    ) -> List[np.ndarray]:
        corners = cv2.goodFeaturesToTrack(
            image,
            maxCorners=self._config.max_features,
            qualityLevel=self._config.quality_level,
            minDistance=self._config.min_distance,
            mask=mask,
            blockSize=self._config.harris_block_size,
            useHarrisDetector=True,
            k=self._config.harris_k,
        )
        if corners is None:
            return []
        return [c.ravel().astype(np.float32) for c in corners]
