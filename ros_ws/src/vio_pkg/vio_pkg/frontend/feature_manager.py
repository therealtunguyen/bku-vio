import numpy as np
from typing import List
from ..utils.common import FeatureTrack
from .interfaces import IFeatureDetector, IFeatureTracker

class FeatureManager:
    def __init__(self, detector: IFeatureDetector, tracker: IFeatureTracker):
        self.detector = detector
        self.tracker = tracker
        self.active_tracks: List[FeatureTrack] = [] # List of currently active FeatureTracks
        self.mature_tracks: List[FeatureTrack] = [] # List of features ready for the backend
        self.next_feature_id = 0

    def process_image(self, timestamp: float, image: np.ndarray, current_camera_pose) -> List[FeatureTrack]:
        """
        Main entry point for processing a new image.
        - Track existing features using self.tracker
        - Detect new features using self.detector (using grid-based sampling)
        - Update active_tracks and manage IDs
        - Return mature tracks that are lost or tracked long enough
        """
        # TODO: Implement tracking logic
        # TODO: Implement RANSAC outlier rejection (e.g. cv2.findFundamentalMat)
        # TODO: Implement ID assignment and grid-based new feature detection
        
        mature_features_for_backend = self.mature_tracks.copy()
        self.mature_tracks.clear()
        
        return mature_features_for_backend
