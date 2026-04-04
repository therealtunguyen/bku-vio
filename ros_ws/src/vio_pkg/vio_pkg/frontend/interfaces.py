from abc import ABC, abstractmethod
import numpy as np
from typing import List, Tuple

class IFeatureDetector(ABC):
    @abstractmethod
    def detect(self, image: np.ndarray) -> List[np.ndarray]:
        """
        Detect keypoints in the given image.
        :param image: Input image
        :return: List of keypoints (e.g., [x, y])
        """
        pass

class IFeatureTracker(ABC):
    @abstractmethod
    def track(self, image_prev: np.ndarray, image_curr: np.ndarray, 
              pts_prev: List[np.ndarray]) -> Tuple[List[np.ndarray], List[int]]:
        """
        Track keypoints from the previous image to the current image.
        :param image_prev: Previous image
        :param image_curr: Current image
        :param pts_prev: Keypoints in the previous image
        :return: Tuple of (tracked_keypoints, status_flags)
        """
        pass
