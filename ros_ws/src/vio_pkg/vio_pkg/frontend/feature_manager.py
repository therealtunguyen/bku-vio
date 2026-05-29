from typing import List, Optional, Tuple

import cv2
import numpy as np

from ..utils.common import CameraPose, FeatureTrack
from .interfaces import FrontendConfig, IFeatureDetector, IFeatureTracker


class FeatureManager:
    """
    Facade that orchestrates the full visual-frontend pipeline:

        detect (first frame)
            |
        track -> F-B reject -> RANSAC reject -> update tracks
            |                                        |
        new-feature detection in depleted cells   mature tracks -> backend

    Design patterns:
    - **Facade**: single `process_image()` entry point hides the pipeline.
    - **Strategy**: detector and tracker are injected; swappable at construction.
    - **DI / DTO**: `FrontendConfig` carries all tunable params.
    """

    def __init__(
        self,
        detector: IFeatureDetector,
        tracker: IFeatureTracker,
        config: FrontendConfig,
    ):
        self._detector = detector
        self._tracker = tracker
        self._config = config

        self.active_tracks: List[FeatureTrack] = []
        self.mature_tracks: List[FeatureTrack] = []
        self.next_feature_id: int = 0

        self._prev_image: Optional[np.ndarray] = None

    # ------------------------------------------------------------------
    # Public entry point
    # ------------------------------------------------------------------

    def process_image(
        self,
        timestamp: float,
        image: np.ndarray,
        current_camera_pose: CameraPose,
    ) -> List[FeatureTrack]:
        """
        Process a new frame.

        :param timestamp:           Frame timestamp (seconds).
        :param image:               Grayscale image (H x W, uint8).
        :param current_camera_pose: Camera pose after IMU propagation.
        :return: Mature feature tracks ready for the MSCKF backend.
        """
        if self._prev_image is None:
            self._handle_first_frame(image, current_camera_pose)
        else:
            self._handle_subsequent_frame(image, current_camera_pose)

        self._prev_image = image

        mature_out = self.mature_tracks.copy()
        self.mature_tracks.clear()
        return mature_out

    # ------------------------------------------------------------------
    # First-frame initialisation
    # ------------------------------------------------------------------

    def _handle_first_frame(
        self,
        image: np.ndarray,
        camera_pose: CameraPose,
    ) -> None:
        """Detect features across the full image; no tracking yet."""
        new_pts = self._detector.detect(image, mask=None)
        for pt in new_pts[: self._config.max_features]:
            self.active_tracks.append(
                FeatureTrack(
                    feature_id=self._assign_id(),
                    observations=[pt],
                    camera_states=[camera_pose],
                )
            )

    # ------------------------------------------------------------------
    # Subsequent frames
    # ------------------------------------------------------------------

    def _handle_subsequent_frame(
        self,
        image: np.ndarray,
        camera_pose: CameraPose,
    ) -> None:
        """Track -> reject outliers -> update tracks -> detect new features."""
        # 1. Track all active features forward one frame.
        prev_pts = [t.observations[-1] for t in self.active_tracks]
        tracked_pts, status = self._tracker.track(
            self._prev_image, image, prev_pts
        )

        # 2. Split into survived (tracked OK) and lost.
        survived_tracks, survived_pts, lost_tracks = self._split_by_status(
            self.active_tracks, tracked_pts, status
        )

        # 3. RANSAC on survived tracks to reject geometric outliers.
        if len(survived_pts) >= 8:
            survived_tracks, survived_pts = self._ransac_reject(
                [t.observations[-1] for t in survived_tracks],
                survived_pts,
                survived_tracks,
            )

        # 4. Update survived tracks; mature those that hit max_track_length.
        still_active: List[FeatureTrack] = []
        for track, pt in zip(survived_tracks, survived_pts):
            track.observations.append(pt)
            track.camera_states.append(camera_pose)
            if len(track.observations) >= self._config.max_track_length:
                self.mature_tracks.append(track)
            else:
                still_active.append(track)

        # 5. Mature lost tracks that have enough observations.
        for track in lost_tracks:
            if len(track.observations) >= self._config.min_track_length:
                self.mature_tracks.append(track)

        # 6. Detect new features in depleted grid cells.
        self.active_tracks = still_active
        self._detect_new_features(image, camera_pose)

    # ------------------------------------------------------------------
    # RANSAC outlier rejection
    # ------------------------------------------------------------------

    def _ransac_reject(
        self,
        pts_prev: List[np.ndarray],
        pts_curr: List[np.ndarray],
        tracks: List[FeatureTrack],
    ) -> Tuple[List[FeatureTrack], List[np.ndarray]]:
        """Remove geometric outliers via the fundamental matrix (RANSAC)."""
        p1 = np.array(pts_prev, dtype=np.float64)
        p2 = np.array(pts_curr, dtype=np.float64)

        _, mask = cv2.findFundamentalMat(
            p1, p2, cv2.FM_RANSAC, self._config.ransac_threshold
        )

        if mask is None:
            return tracks, pts_curr

        inlier_tracks: List[FeatureTrack] = []
        inlier_pts: List[np.ndarray] = []
        for i, (track, pt) in enumerate(zip(tracks, pts_curr)):
            if mask[i][0] == 1:
                inlier_tracks.append(track)
                inlier_pts.append(pt)

        return inlier_tracks, inlier_pts

    # ------------------------------------------------------------------
    # Grid-based new feature detection
    # ------------------------------------------------------------------

    def _detect_new_features(
        self,
        image: np.ndarray,
        camera_pose: CameraPose,
    ) -> None:
        """Detect new features in grid cells that are below capacity."""
        if len(self.active_tracks) >= self._config.max_features:
            return

        mask = self._build_occupancy_mask(image.shape)
        new_pts = self._detector.detect(image, mask=mask)

        budget = self._config.max_features - len(self.active_tracks)
        for pt in new_pts[:budget]:
            self.active_tracks.append(
                FeatureTrack(
                    feature_id=self._assign_id(),
                    observations=[pt],
                    camera_states=[camera_pose],
                )
            )

    def _build_occupancy_mask(self, image_shape: tuple) -> np.ndarray:
        """
        Return a binary mask where grid cells with fewer than
        *features_per_cell* existing features are marked 255 (detect here).
        """
        h, w = image_shape[:2]
        cell_h = max(1, h // self._config.grid_rows)
        cell_w = max(1, w // self._config.grid_cols)
        features_per_cell = max(
            1,
            self._config.max_features
            // (self._config.grid_rows * self._config.grid_cols),
        )

        grid_count = np.zeros(
            (self._config.grid_rows, self._config.grid_cols), dtype=int
        )
        for track in self.active_tracks:
            pt = track.observations[-1]
            col = min(int(pt[0] / cell_w), self._config.grid_cols - 1)
            row = min(int(pt[1] / cell_h), self._config.grid_rows - 1)
            grid_count[row, col] += 1

        mask = np.zeros((h, w), dtype=np.uint8)
        for r in range(self._config.grid_rows):
            for c in range(self._config.grid_cols):
                if grid_count[r, c] < features_per_cell:
                    y1 = r * cell_h
                    y2 = (r + 1) * cell_h if r < self._config.grid_rows - 1 else h
                    x1 = c * cell_w
                    x2 = (c + 1) * cell_w if c < self._config.grid_cols - 1 else w
                    mask[y1:y2, x1:x2] = 255

        return mask

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _split_by_status(
        tracks: List[FeatureTrack],
        tracked_pts: List[np.ndarray],
        status: List[int],
    ) -> Tuple[List[FeatureTrack], List[np.ndarray], List[FeatureTrack]]:
        """Partition tracks into (survived, survived_pts, lost)."""
        survived_tracks: List[FeatureTrack] = []
        survived_pts: List[np.ndarray] = []
        lost_tracks: List[FeatureTrack] = []

        for track, pt, s in zip(tracks, tracked_pts, status):
            if s == 1:
                survived_tracks.append(track)
                survived_pts.append(pt)
            else:
                lost_tracks.append(track)

        return survived_tracks, survived_pts, lost_tracks

    def _assign_id(self) -> int:
        fid = self.next_feature_id
        self.next_feature_id += 1
        return fid

    def reset(self) -> None:
        """Clear temporal frontend state after a sensor timestamp discontinuity."""
        self.active_tracks.clear()
        self.mature_tracks.clear()
        self._prev_image = None
