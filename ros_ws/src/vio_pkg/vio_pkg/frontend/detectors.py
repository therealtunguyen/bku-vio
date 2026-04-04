import numpy as np
from typing import List
from .interfaces import IFeatureDetector

class HarrisDetector(IFeatureDetector):
    def detect(self, image: np.ndarray) -> List[np.ndarray]:
        # TODO: Implement Harris corner detection + grid sampling (8x8)
        pass
