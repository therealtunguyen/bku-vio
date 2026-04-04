import numpy as np
from typing import List, Tuple
from .interfaces import IFeatureTracker

class KLTTracker(IFeatureTracker):
    def track(self, image_prev: np.ndarray, image_curr: np.ndarray, 
              pts_prev: List[np.ndarray]) -> Tuple[List[np.ndarray], List[int]]:
        # TODO: Implement optical flow tracking using cv2.calcOpticalFlowPyrLK
        pass
