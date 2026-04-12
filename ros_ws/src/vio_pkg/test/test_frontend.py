"""
Standalone smoke test for the visual frontend.
No ROS required — run directly with Python 3.

Usage (inside the container, after colcon build + source):
    cd /home/ubuntu/VIO/ros_ws
    python3 src/vio_pkg/test/test_frontend.py
"""

import sys
import numpy as np
import cv2

# ---------------------------------------------------------------------------
# Make the package importable without installing it
# ---------------------------------------------------------------------------
sys.path.insert(0, "src/vio_pkg")

from vio_pkg.utils.common import CameraPose
from vio_pkg.frontend.interfaces import FrontendConfig
from vio_pkg.frontend.detectors import ShiTomasiDetector, HarrisDetector
from vio_pkg.frontend.trackers import KLTTracker
from vio_pkg.frontend.feature_manager import FeatureManager

PASS = "\033[92mPASS\033[0m"
FAIL = "\033[91mFAIL\033[0m"

def check(label: str, condition: bool, detail: str = ""):
    status = PASS if condition else FAIL
    print(f"  [{status}] {label}" + (f" — {detail}" if detail else ""))
    return condition


def make_test_image(seed: int = 0) -> np.ndarray:
    """Generate a 480x752 synthetic grayscale image with random texture."""
    rng = np.random.default_rng(seed)
    img = rng.integers(40, 220, (480, 752), dtype=np.uint8)
    # Add some structure so the detector finds real corners
    for _ in range(200):
        x = rng.integers(10, 740)
        y = rng.integers(10, 470)
        cv2.rectangle(img, (x, y), (x + rng.integers(5, 30), y + rng.integers(5, 30)),
                      int(rng.integers(0, 255)), -1)
    return img


def dummy_pose(t: float = 0.0) -> CameraPose:
    return CameraPose(
        timestamp=t,
        position=np.zeros(3),
        quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
    )


# ---------------------------------------------------------------------------
# Test 1: ShiTomasiDetector
# ---------------------------------------------------------------------------
def test_shi_tomasi():
    print("\n--- Test 1: ShiTomasiDetector ---")
    cfg = FrontendConfig(max_features=150)
    det = ShiTomasiDetector(cfg)
    img = make_test_image(seed=1)

    pts = det.detect(img)
    check("Returns a list", isinstance(pts, list))
    check("Detects > 0 points", len(pts) > 0, f"{len(pts)} points")
    check("Respects max_features", len(pts) <= cfg.max_features)
    check("Each point is np.ndarray shape (2,)", all(p.shape == (2,) for p in pts))

    # With a mask that blocks the left half
    mask = np.zeros_like(img)
    mask[:, img.shape[1] // 2:] = 255
    pts_masked = det.detect(img, mask=mask)
    if pts_masked:
        check("Mask respected (all x >= width/2)",
              all(p[0] >= img.shape[1] // 2 for p in pts_masked),
              f"{len(pts_masked)} points in masked region")
    else:
        check("Mask respected (no points returned in empty half)", True)


# ---------------------------------------------------------------------------
# Test 2: HarrisDetector
# ---------------------------------------------------------------------------
def test_harris():
    print("\n--- Test 2: HarrisDetector ---")
    cfg = FrontendConfig(max_features=100)
    det = HarrisDetector(cfg)
    img = make_test_image(seed=2)

    pts = det.detect(img)
    check("Returns a list", isinstance(pts, list))
    check("Detects > 0 points", len(pts) > 0, f"{len(pts)} points")
    check("Respects max_features", len(pts) <= cfg.max_features)


# ---------------------------------------------------------------------------
# Test 3: KLTTracker — with identical images (expect 100% track rate)
# ---------------------------------------------------------------------------
def test_klt_identical():
    print("\n--- Test 3: KLTTracker (identical frames — expect all tracked) ---")
    cfg = FrontendConfig(fb_threshold=1.0)
    det = ShiTomasiDetector(cfg)
    tracker = KLTTracker(cfg)

    img = make_test_image(seed=3)
    pts = det.detect(img)

    tracked, status = tracker.track(img, img, pts)
    check("Returns same count as input", len(tracked) == len(pts))
    check("Status list matches", len(status) == len(pts))
    survived = sum(status)
    check("High track rate on identical frames (>80%)",
          survived / max(len(pts), 1) > 0.8,
          f"{survived}/{len(pts)} tracked")


# ---------------------------------------------------------------------------
# Test 4: KLTTracker — with shifted image (simulates camera motion)
# ---------------------------------------------------------------------------
def test_klt_shifted():
    print("\n--- Test 4: KLTTracker (3px shifted frame) ---")
    cfg = FrontendConfig(fb_threshold=1.5)
    det = ShiTomasiDetector(cfg)
    tracker = KLTTracker(cfg)

    img1 = make_test_image(seed=4)
    # Shift the image 3 pixels to the right (simulate camera motion)
    M = np.float32([[1, 0, 3], [0, 1, 0]])
    img2 = cv2.warpAffine(img1, M, (img1.shape[1], img1.shape[0]))

    pts = det.detect(img1)
    tracked, status = tracker.track(img1, img2, pts)
    survived = sum(status)
    check("Tracks survive a small shift (>50%)",
          survived / max(len(pts), 1) > 0.5,
          f"{survived}/{len(pts)} tracked")


# ---------------------------------------------------------------------------
# Test 5: FeatureManager — first frame initialises active tracks
# ---------------------------------------------------------------------------
def test_feature_manager_first_frame():
    print("\n--- Test 5: FeatureManager (first frame) ---")
    cfg = FrontendConfig(max_features=100)
    fm = FeatureManager(ShiTomasiDetector(cfg), KLTTracker(cfg), cfg)

    img = make_test_image(seed=5)
    pose = dummy_pose(0.0)
    mature = fm.process_image(0.0, img, pose)

    check("No mature features on first frame", len(mature) == 0)
    check("Active tracks initialised", len(fm.active_tracks) > 0,
          f"{len(fm.active_tracks)} active")
    check("Respects max_features", len(fm.active_tracks) <= cfg.max_features)
    check("Track IDs are unique",
          len({t.feature_id for t in fm.active_tracks}) == len(fm.active_tracks))


# ---------------------------------------------------------------------------
# Test 6: FeatureManager — multi-frame tracking
# ---------------------------------------------------------------------------
def test_feature_manager_multi_frame():
    print("\n--- Test 6: FeatureManager (multi-frame lifecycle) ---")
    cfg = FrontendConfig(
        max_features=80,
        max_track_length=5,   # short for testing
        min_track_length=2,
    )
    fm = FeatureManager(ShiTomasiDetector(cfg), KLTTracker(cfg), cfg)

    img = make_test_image(seed=6)
    M = np.float32([[1, 0, 2], [0, 1, 0]])  # 2px shift each frame

    all_mature = []
    for i in range(8):
        shifted = cv2.warpAffine(img, np.float32([[1, 0, i * 2], [0, 1, 0]]),
                                 (img.shape[1], img.shape[0]))
        pose = dummy_pose(float(i) * 0.033)
        mature = fm.process_image(float(i) * 0.033, shifted, pose)
        all_mature.extend(mature)

    check("Active tracks non-empty after 8 frames",
          len(fm.active_tracks) > 0, f"{len(fm.active_tracks)} active")
    check("Some tracks matured over 8 frames",
          len(all_mature) > 0, f"{len(all_mature)} matured")
    check("Mature tracks have >= min_track_length observations",
          all(len(t.observations) >= cfg.min_track_length for t in all_mature))
    check("Mature tracks have matching camera_states",
          all(len(t.observations) == len(t.camera_states) for t in all_mature))
    check("Feature IDs are globally unique across all matured tracks",
          len({t.feature_id for t in all_mature}) == len(all_mature))


# ---------------------------------------------------------------------------
# Test 7: Grid occupancy mask coverage
# ---------------------------------------------------------------------------
def test_grid_occupancy():
    print("\n--- Test 7: Grid-based feature distribution ---")
    cfg = FrontendConfig(max_features=200, grid_rows=4, grid_cols=4)
    fm = FeatureManager(ShiTomasiDetector(cfg), KLTTracker(cfg), cfg)

    img = make_test_image(seed=7)
    fm.process_image(0.0, img, dummy_pose())

    # Check how many grid cells have at least one feature
    h, w = img.shape[:2]
    cell_h = h // cfg.grid_rows
    cell_w = w // cfg.grid_cols
    occupied = set()
    for t in fm.active_tracks:
        pt = t.observations[-1]
        col = min(int(pt[0] / cell_w), cfg.grid_cols - 1)
        row = min(int(pt[1] / cell_h), cfg.grid_rows - 1)
        occupied.add((row, col))

    total_cells = cfg.grid_rows * cfg.grid_cols
    coverage = len(occupied) / total_cells
    check("Grid coverage >= 50% of cells",
          coverage >= 0.5, f"{len(occupied)}/{total_cells} cells occupied")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("=" * 55)
    print("  VIO Frontend Smoke Tests")
    print("=" * 55)

    test_shi_tomasi()
    test_harris()
    test_klt_identical()
    test_klt_shifted()
    test_feature_manager_first_frame()
    test_feature_manager_multi_frame()
    test_grid_occupancy()

    print("\n" + "=" * 55)
    print("  Done.")
    print("=" * 55)
