#!/usr/bin/env python3
"""
Export M4 VIO evaluation topics from a ROS 2 bag to TUM and run evo.

Run inside the ROS 2 container after sourcing ROS:
    python3 /home/ubuntu/VIO/tools/evaluate_m4.py \
        /home/ubuntu/VIO/results/vio_run_001 \
        /home/ubuntu/VIO/results/m4_eval
"""

from __future__ import annotations

import argparse
import sqlite3
import subprocess
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


ODOM_TOPIC = "/vio/odometry"
GT_TOPIC = "/vio/gt_path"


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def pose_to_tum_line(timestamp: float, pose) -> str:
    p = pose.position
    q = pose.orientation
    return (
        f"{timestamp:.9f} "
        f"{p.x:.9f} {p.y:.9f} {p.z:.9f} "
        f"{q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n"
    )


def bag_databases(bag_dir: Path) -> list[Path]:
    db_paths = sorted(bag_dir.glob("*.db3"))
    if not db_paths:
        raise FileNotFoundError(f"No .db3 files found under {bag_dir}")
    return db_paths


def export_tum(bag_dir: Path, output_dir: Path) -> tuple[Path, Path]:
    output_dir.mkdir(parents=True, exist_ok=True)
    vio_path = output_dir / "vio_odom.tum"
    gt_path = output_dir / "vio_gt_path.tum"

    odom_lines: list[str] = []
    gt_by_timestamp: dict[float, str] = {}
    gt_loaded = False

    for db_path in bag_databases(bag_dir):
        with sqlite3.connect(db_path) as conn:
            rows = conn.execute(
                "select id, name, type from topics where name in (?, ?)",
                (ODOM_TOPIC, GT_TOPIC),
            ).fetchall()
            if not rows:
                continue
            topic_by_id = {row[0]: row[1] for row in rows}
            msg_type_by_topic = {
                row[1]: get_message(row[2])
                for row in rows
            }
            placeholders = ",".join("?" for _ in topic_by_id)
            query = (
                "select topic_id, data from messages "
                f"where topic_id in ({placeholders}) order by timestamp"
            )
            for topic_id, data in conn.execute(query, tuple(topic_by_id)):
                topic = topic_by_id[topic_id]
                if topic == GT_TOPIC and gt_loaded:
                    continue
                msg = deserialize_message(data, msg_type_by_topic[topic])
                if topic == ODOM_TOPIC:
                    timestamp = stamp_to_sec(msg.header.stamp)
                    odom_lines.append(pose_to_tum_line(timestamp, msg.pose.pose))
                elif topic == GT_TOPIC:
                    for pose_stamped in msg.poses:
                        timestamp = stamp_to_sec(pose_stamped.header.stamp)
                        gt_by_timestamp[timestamp] = pose_to_tum_line(
                            timestamp,
                            pose_stamped.pose,
                        )
                    gt_loaded = True

    vio_path.write_text("".join(odom_lines), encoding="utf-8")
    gt_path.write_text(
        "".join(gt_by_timestamp[key] for key in sorted(gt_by_timestamp)),
        encoding="utf-8",
    )

    print(f"Exported {len(odom_lines)} odometry poses to {vio_path}")
    print(f"Exported {len(gt_by_timestamp)} GT poses to {gt_path}")
    return vio_path, gt_path


def run_evo(output_dir: Path, vio_path: Path, gt_path: Path) -> None:
    for result_name in ("ape_aligned.zip", "rpe_1m_aligned.zip"):
        result_path = output_dir / result_name
        if result_path.exists():
            result_path.unlink()

    commands = [
        [
            "evo_ape",
            "tum",
            str(gt_path),
            str(vio_path),
            "-a",
            "--pose_relation",
            "trans_part",
            "--save_results",
            str(output_dir / "ape_aligned.zip"),
        ],
        [
            "evo_rpe",
            "tum",
            str(gt_path),
            str(vio_path),
            "-a",
            "--delta",
            "1",
            "--delta_unit",
            "m",
            "--pose_relation",
            "trans_part",
            "--save_results",
            str(output_dir / "rpe_1m_aligned.zip"),
        ],
    ]
    for command in commands:
        print("\n$ " + " ".join(command))
        subprocess.run(command, check=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_dir", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--skip-evo", action="store_true")
    args = parser.parse_args()

    vio_path, gt_path = export_tum(args.bag_dir, args.output_dir)
    if not args.skip_evo:
        run_evo(args.output_dir, vio_path, gt_path)


if __name__ == "__main__":
    main()
