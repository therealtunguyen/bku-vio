#!/usr/bin/env python3
"""
Deterministic EuRoC V1_01_easy propagation-only replay harness.

Calls the existing test_hcmut_propagation_replay.py with EuRoC-specific parameters
to extract frame metrics for frames 160–200 for comparison with HCMUT dataset.

Run inside the container:

    source /opt/ros/humble/setup.bash
    cd /home/ubuntu/VIO/ros_ws
    python3 src/vio_pkg/test/test_euroc_propagation_replay.py
"""

import argparse
import os
import sys
from pathlib import Path

# Add parent directory to path to import the main test
sys.path.insert(0, os.path.dirname(__file__))

from test_hcmut_propagation_replay import main as hcmut_main

REPO_ROOT = Path(__file__).resolve().parents[4]

# EuRoC V1_01_easy defaults
DEFAULT_EUROC_BAG_PATHS = (
    "/home/ubuntu/VIO/dataset/V1_01_easy",
    str(REPO_ROOT / "dataset" / "V1_01_easy"),
)

DEFAULT_EUROC_IMU_TOPIC = "/imu0"
DEFAULT_EUROC_IMAGE_TOPIC = "/cam0/image_raw"

# EuRoC cam0 calibration
DEFAULT_EUROC_CAMERA_FX = 458.654
DEFAULT_EUROC_CAMERA_FY = 457.296
DEFAULT_EUROC_CAMERA_CX = 367.215
DEFAULT_EUROC_CAMERA_CY = 248.375
DEFAULT_EUROC_CAMERA_DISTORTION = (
    -0.28340811,
    0.07395907,
    0.00019359,
    1.76187114e-05,
)

# EuRoC cam0 to imu0 extrinsics (camera_in_imu convention)
DEFAULT_EUROC_CAMERA_R_IC = (
    0.0148655429818,
    -0.999880929698,
    0.004140296794,
    0.999557249008,
    0.014967213324,
    0.025715529948,
    -0.0257744366974,
    0.003756188357,
    0.999660727108,
)
DEFAULT_EUROC_CAMERA_T_IC = (
    -0.0216401455,
    -0.0646769868,
    0.0098107306,
)

# EuRoC image dimensions: 752 x 480
DEFAULT_EUROC_IMAGE_PROCESSING_WIDTH = 752

# EuRoC V1_01_easy has ~4431 frames (at 10 Hz)
DEFAULT_EUROC_FULL_STOP_IMAGE = 400
DEFAULT_EUROC_SNAPSHOT_IMAGE = 150
DEFAULT_EUROC_SEGMENT_STOP_IMAGE = 400
DEFAULT_EUROC_OUTPUT_FILE = "frame_metrics_euroc_160_200.csv"


def resolve_bag_path(bag_path=None):
    """Resolve bag path from defaults if not provided."""
    if bag_path is not None:
        return bag_path
    for path in DEFAULT_EUROC_BAG_PATHS:
        if os.path.isdir(path):
            return path
    raise FileNotFoundError(
        f"EuRoC bag not found in any of {DEFAULT_EUROC_BAG_PATHS}"
    )


def main() -> int:
    # Parse arguments for the EuRoC replay harness
    parser = argparse.ArgumentParser(
        description="EuRoC V1_01_easy propagation-only replay harness"
    )
    parser.add_argument(
        "--replay-mode",
        choices=("propagation", "msckf"),
        default="msckf",
        help="Replay mode (default: msckf for full EKF simulation)",
    )
    parser.add_argument(
        "--bag-path",
        default=None,
        help="Override default EuRoC bag path",
    )
    parser.add_argument(
        "--full-stop-image",
        type=int,
        default=DEFAULT_EUROC_FULL_STOP_IMAGE,
        help="Frame limit for full replay",
    )
    parser.add_argument(
        "--snapshot-image",
        type=int,
        default=DEFAULT_EUROC_SNAPSHOT_IMAGE,
        help="Frame to snapshot for segment replay",
    )
    parser.add_argument(
        "--segment-stop-image",
        type=int,
        default=DEFAULT_EUROC_SEGMENT_STOP_IMAGE,
        help="Frame limit for segment replay",
    )
    parser.add_argument(
        "--bias-mode",
        choices=("baseline", "zero_accel_bias", "zero_gyro_bias", "zero_both_biases"),
        default="baseline",
    )
    parser.add_argument(
        "--output-file",
        default=DEFAULT_EUROC_OUTPUT_FILE,
        help="Output CSV filename for exported frame metrics",
    )
    args = parser.parse_args()

    # Resolve bag path
    bag_path = resolve_bag_path(args.bag_path)
    print(f"Using EuRoC bag: {bag_path}")

    # Build arguments for the HCMUT harness with EuRoC parameters
    hcmut_args = [
        "--replay-mode", args.replay_mode,
        "--bag-path", bag_path,
        "--imu-topic", DEFAULT_EUROC_IMU_TOPIC,
        "--image-topic", DEFAULT_EUROC_IMAGE_TOPIC,
        "--full-stop-image", str(args.full_stop_image),
        "--snapshot-image", str(args.snapshot_image),
        "--segment-stop-image", str(args.segment_stop_image),
        "--bias-mode", args.bias_mode,
        "--image-processing-width", str(DEFAULT_EUROC_IMAGE_PROCESSING_WIDTH),
        "--camera-fx", str(DEFAULT_EUROC_CAMERA_FX),
        "--camera-fy", str(DEFAULT_EUROC_CAMERA_FY),
        "--camera-cx", str(DEFAULT_EUROC_CAMERA_CX),
        "--camera-cy", str(DEFAULT_EUROC_CAMERA_CY),
        "--camera-distortion",
        *[str(d) for d in DEFAULT_EUROC_CAMERA_DISTORTION],
        "--camera-R-IC", *[str(r) for r in DEFAULT_EUROC_CAMERA_R_IC],
        "--camera-t-IC", *[str(t) for t in DEFAULT_EUROC_CAMERA_T_IC],
        "--report-window-start", "160",
        "--report-window-stop", "200",
        "--report-every", "10",
    ]

    # Replace sys.argv with EuRoC parameters and run HCMUT harness
    sys.argv = ["test_euroc_propagation_replay.py"] + hcmut_args

    # Invoke the HCMUT harness which will generate frame_metrics_160_200.csv
    # We also need to update the output filename for EuRoC
    print("\n" + "="*70)
    print("Running EuRoC V1_01_easy deterministic replay with frame metrics export")
    print("="*70 + "\n")

    try:
        result = hcmut_main()
        
        # Rename the output CSV to distinguish from HCMUT.
        if os.path.exists("frame_metrics_160_200.csv"):
            os.replace("frame_metrics_160_200.csv", args.output_file)
            print(f"\n✓ Output file written to {args.output_file}")
        
        if os.path.exists("euroc_replay_summary.json"):
            print("✓ Summary file euroc_replay_summary.json generated")
        
        return result
    except Exception as e:
        print(f"Error running EuRoC replay: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
