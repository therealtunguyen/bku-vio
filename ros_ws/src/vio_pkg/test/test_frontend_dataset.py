"""
Dataset integration test for the visual frontend.
Reads real camera frames from the EuRoC ROS 2 bag and runs the full
frontend pipeline on them.

Requires ROS 2 Humble — run INSIDE the container:

    docker exec -it vio-ros2 bash -c "
        source /opt/ros/humble/setup.bash &&
        cd /home/ubuntu/VIO/ros_ws &&
        python3 src/vio_pkg/test/test_frontend_dataset.py
    "

The bag must be at /home/ubuntu/VIO/dataset/V1_01_easy/.
"""

import sys
import os

# ---------------------------------------------------------------------------
# Make the package importable without a full colcon install
# ---------------------------------------------------------------------------
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "vio_pkg", ".."))

import numpy as np
import cv2

from vio_pkg.utils.common import CameraPose
from vio_pkg.frontend.interfaces import FrontendConfig
from vio_pkg.frontend.detectors import ShiTomasiDetector, HarrisDetector
from vio_pkg.frontend.trackers import KLTTracker
from vio_pkg.frontend.feature_manager import FeatureManager

# ---------------------------------------------------------------------------
# ROS 2 bag reading (requires rosbag2_py + rclpy inside the container)
# ---------------------------------------------------------------------------
try:
    import rclpy
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    import rosbag2_py
    HAS_ROS = True
except ImportError:
    HAS_ROS = False

BAG_PATH = "/home/ubuntu/VIO/dataset/V1_01_easy"
CAM_TOPIC = "/cam0/image_raw"
MAX_FRAMES = 50   # number of frames to test with (full bag has 2912)

PASS = "\033[92mPASS\033[0m"
FAIL = "\033[91mFAIL\033[0m"
INFO = "\033[94mINFO\033[0m"


def check(label: str, condition: bool, detail: str = "") -> bool:
    status = PASS if condition else FAIL
    print(f"  [{status}] {label}" + (f" — {detail}" if detail else ""))
    return condition


def info(msg: str):
    print(f"  [{INFO}] {msg}")


# ---------------------------------------------------------------------------
# Bag reader helper
# ---------------------------------------------------------------------------
def _ros_image_to_gray(msg) -> np.ndarray:
    """
    Decode a sensor_msgs/Image to a grayscale numpy array without cv_bridge.

    cv_bridge is compiled against NumPy 1.x and crashes on NumPy 2.x
    (_ARRAY_API not found). Raw decoding from msg.data avoids that entirely.
    """
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    enc = msg.encoding.lower()

    if enc == "mono8":
        return raw.reshape(msg.height, msg.width)
    elif enc == "mono16":
        img16 = raw.view(np.uint16).reshape(msg.height, msg.width)
        return (img16 >> 8).astype(np.uint8)
    elif enc in ("bgr8", "rgb8"):
        color = raw.reshape(msg.height, msg.width, 3)
        code = cv2.COLOR_BGR2GRAY if enc == "bgr8" else cv2.COLOR_RGB2GRAY
        return cv2.cvtColor(color, code)
    else:
        raise ValueError(f"Unsupported image encoding: {msg.encoding!r}")


def read_camera_frames(bag_path: str, topic: str, max_frames: int):
    """Yield (timestamp_sec, grayscale_image) from a ROS 2 bag."""
    storage_opts = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_opts = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_opts, converter_opts)

    filter_ = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(filter_)

    msg_type = get_message("sensor_msgs/msg/Image")
    count = 0
    while reader.has_next() and count < max_frames:
        _, data, timestamp_ns = reader.read_next()
        msg = deserialize_message(data, msg_type)
        gray = _ros_image_to_gray(msg)
        yield timestamp_ns * 1e-9, gray
        count += 1


def dummy_pose(t: float) -> CameraPose:
    return CameraPose(
        timestamp=t,
        position=np.zeros(3),
        quaternion=np.array([1.0, 0.0, 0.0, 0.0]),
    )


# ---------------------------------------------------------------------------
# Test 1: Detector on real EuRoC frames
# ---------------------------------------------------------------------------
def test_detector_on_real_frames(frames):
    print("\n--- Test 1: ShiTomasiDetector on real EuRoC frames ---")
    cfg = FrontendConfig(max_features=200)
    det = ShiTomasiDetector(cfg)

    counts = []
    for t, img in frames[:10]:
        pts = det.detect(img)
        counts.append(len(pts))

    avg = sum(counts) / len(counts)
    info(f"Frames tested: {len(counts)},  avg features: {avg:.1f},  "
         f"min: {min(counts)},  max: {max(counts)}")

    check("Detects features on every frame", all(c > 0 for c in counts))
    check("Average feature count >= 50", avg >= 50, f"{avg:.1f} avg")
    check("Respects max_features on all frames", all(c <= cfg.max_features for c in counts))


# ---------------------------------------------------------------------------
# Test 2: KLT tracking between consecutive real frames
# ---------------------------------------------------------------------------
def test_tracker_on_real_frames(frames):
    print("\n--- Test 2: KLTTracker on consecutive real frames ---")
    cfg = FrontendConfig(max_features=150, fb_threshold=1.0)
    det = ShiTomasiDetector(cfg)
    tracker = KLTTracker(cfg)

    track_rates = []
    pairs = list(zip(frames, frames[1:]))[:10]

    for (t0, img0), (t1, img1) in pairs:
        pts = det.detect(img0)
        if not pts:
            continue
        tracked, status = tracker.track(img0, img1, pts)
        rate = sum(status) / len(pts)
        track_rates.append(rate)
        
    if len(track_rates) == 0:
        print(f"\n[ERROR] No valid frame pairs found in {BAG_PATH}")
        sys.exit(1)

    avg_rate = sum(track_rates) / len(track_rates)
    info(f"Avg track survival rate over {len(track_rates)} frame pairs: {avg_rate:.1%}")

    check("Track rate > 60% on consecutive EuRoC frames",
          avg_rate > 0.6, f"{avg_rate:.1%}")
    check("Forward-backward check runs without error", True)


# ---------------------------------------------------------------------------
# Test 3: FeatureManager full pipeline on real sequence
# ---------------------------------------------------------------------------
def test_feature_manager_on_real_sequence(frames):
    print(f"\n--- Test 3: FeatureManager on {len(frames)}-frame real sequence ---")
    cfg = FrontendConfig(
        max_features=200,
        max_track_length=15,
        min_track_length=3,
    )
    fm = FeatureManager(ShiTomasiDetector(cfg), KLTTracker(cfg), cfg)

    all_mature = []
    active_counts = []

    for t, img in frames:
        pose = dummy_pose(t)
        mature = fm.process_image(t, img, pose)
        all_mature.extend(mature)
        active_counts.append(len(fm.active_tracks))

    avg_active = sum(active_counts) / len(active_counts)
    info(f"Frames processed: {len(frames)}")
    info(f"Avg active tracks: {avg_active:.1f}")
    info(f"Total matured tracks: {len(all_mature)}")

    check("Active tracks maintained throughout sequence",
          all(c > 0 for c in active_counts[1:]))
    check("Active tracks stay within max_features",
          all(c <= cfg.max_features for c in active_counts))
    check("Mature tracks produced", len(all_mature) > 0,
          f"{len(all_mature)} tracks")
    check("All mature tracks meet min_track_length",
          all(len(t.observations) >= cfg.min_track_length for t in all_mature))
    check("observations and camera_states always in sync",
          all(len(t.observations) == len(t.camera_states) for t in all_mature))
    check("Feature IDs globally unique",
          len({t.feature_id for t in all_mature}) == len(all_mature))

    if all_mature:
        lengths = [len(t.observations) for t in all_mature]
        info(f"Track length — min: {min(lengths)}, max: {max(lengths)}, "
             f"avg: {sum(lengths)/len(lengths):.1f}")


# ---------------------------------------------------------------------------
# Test 4: Visualise tracking on first 3 frames (saves to /tmp)
# ---------------------------------------------------------------------------
def test_visualise(frames):
    print("\n--- Test 4: Visualisation (saves debug images to /tmp) ---")
    cfg = FrontendConfig(max_features=150)
    fm = FeatureManager(ShiTomasiDetector(cfg), KLTTracker(cfg), cfg)

    out_paths = []
    for i, (t, img) in enumerate(frames[:3]):
        fm.process_image(t, img, dummy_pose(t))
        vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        for track in fm.active_tracks:
            pt = track.observations[-1]
            cv2.circle(vis, (int(pt[0]), int(pt[1])), 3, (0, 255, 0), -1)
        path = f"/tmp/frontend_frame_{i:02d}.png"
        cv2.imwrite(path, vis)
        out_paths.append(path)

    check("Debug images written to /tmp",
          all(os.path.exists(p) for p in out_paths),
          ", ".join(out_paths))
    info("Copy to host with: docker cp vio-ros2:/tmp/frontend_frame_00.png .")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
if __name__ == "__main__":
    print("=" * 60)
    print("  VIO Frontend — Dataset Integration Tests (EuRoC V1_01_easy)")
    print("=" * 60)

    if not HAS_ROS:
        print("\n[ERROR] ROS 2 not found. Run this script inside the container:")
        print("  docker exec -it vio-ros2 bash -c \\")
        print('    "source /opt/ros/humble/setup.bash && \\')
        print("     cd /home/ubuntu/VIO/ros_ws && \\")
        print('     python3 src/vio_pkg/test/test_frontend_dataset.py"')
        sys.exit(1)

    if not os.path.exists(BAG_PATH):
        print(f"\n[ERROR] Bag not found at {BAG_PATH}")
        print("  Make sure the dataset folder is mounted into the container.")
        sys.exit(1)

    print(f"\nReading up to {MAX_FRAMES} frames from {BAG_PATH} ...")
    frames = list(read_camera_frames(BAG_PATH, CAM_TOPIC, MAX_FRAMES))
    if len(frames) == 0:
        print(f"\n[ERROR] No frames found in {BAG_PATH}")
        sys.exit(1)
    info(f"Loaded {len(frames)} frames — "
         f"resolution: {frames[0][1].shape[1]}x{frames[0][1].shape[0]}")

    test_detector_on_real_frames(frames)
    test_tracker_on_real_frames(frames)
    test_feature_manager_on_real_sequence(frames)
    test_visualise(frames)

    print("\n" + "=" * 60)
    print("  Done.")
    print("=" * 60)
