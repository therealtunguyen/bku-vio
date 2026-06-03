#!/usr/bin/env python3
"""Run four replay onset comparisons and write a deterministic JSON artifact."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

TEST_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TEST_DIR))

import test_hcmut_propagation_replay as replay
from test_euroc_propagation_replay import (
    DEFAULT_EUROC_BAG_PATHS,
    DEFAULT_EUROC_CAMERA_CX,
    DEFAULT_EUROC_CAMERA_CY,
    DEFAULT_EUROC_CAMERA_DISTORTION,
    DEFAULT_EUROC_CAMERA_FX,
    DEFAULT_EUROC_CAMERA_FY,
    DEFAULT_EUROC_CAMERA_R_IC,
    DEFAULT_EUROC_CAMERA_T_IC,
    DEFAULT_EUROC_IMAGE_PROCESSING_WIDTH,
    DEFAULT_EUROC_IMAGE_TOPIC,
    DEFAULT_EUROC_IMU_TOPIC,
)

WINDOW_START = 160
WINDOW_STOP = 220
SNAPSHOT_FRAME = 150
DEFAULT_OUTPUT_PATH = "/tmp/vio_onset_window_report.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default=DEFAULT_OUTPUT_PATH)
    return parser.parse_args()


def _resolve_euroc_bag_path() -> str:
    for candidate in DEFAULT_EUROC_BAG_PATHS:
        if os.path.isdir(candidate):
            return candidate
    raise FileNotFoundError(f"EuRoC bag not found in {DEFAULT_EUROC_BAG_PATHS}")


def _build_runner(
    *,
    replay_mode: str,
    runner_kwargs: dict[str, object],
    image_processing_width: int,
    camera_fx: float,
    camera_fy: float,
    camera_cx: float,
    camera_cy: float,
    camera_distortion,
    camera_r_ic,
    camera_t_ic,
    collect_feature_diagnostics: bool = False,
):
    if replay_mode == "propagation":
        return replay.PropagationReplay(**runner_kwargs)
    if replay_mode != "msckf":
        raise ValueError(f"Unsupported replay mode: {replay_mode}")

    return replay.MsckfReplay(
        **runner_kwargs,
        image_processing_width=image_processing_width,
        camera_fx=camera_fx,
        camera_fy=camera_fy,
        camera_cx=camera_cx,
        camera_cy=camera_cy,
        camera_distortion=camera_distortion,
        camera_extrinsics_convention="camera_in_imu",
        camera_R_IC=np.asarray(camera_r_ic, dtype=np.float64).reshape(3, 3),
        camera_t_IC=np.asarray(camera_t_ic, dtype=np.float64),
        min_triangulation_parallax_deg=replay.DEFAULT_MIN_TRIANGULATION_PARALLAX_DEG,
        max_batch_dx_pos_norm=0.5,
        max_batch_dx_vel_norm=1.0,
        max_batch_dx_bias_norm=replay.DEFAULT_MAX_BATCH_DX_BIAS_NORM,
        collect_feature_diagnostics=collect_feature_diagnostics,
    )


def _run_diagnostics(
    *,
    replay_mode: str,
    bag_path: str,
    imu_topic: str,
    image_topic: str,
    image_processing_width: int,
    camera_fx: float,
    camera_fy: float,
    camera_cx: float,
    camera_cy: float,
    camera_distortion,
    camera_r_ic,
    camera_t_ic,
    stop_processed_image: int,
) -> dict[str, object]:
    events = replay._read_replay_events(
        bag_path,
        imu_topic=imu_topic,
        image_topic=image_topic,
    )
    runner_kwargs = {
        "imu_init_samples": replay.DEFAULT_IMU_INIT_SAMPLES,
        "max_imu_init_gap": replay.DEFAULT_MAX_IMU_INIT_GAP,
        "max_frame_gap": replay.DEFAULT_MAX_FRAME_GAP,
        "max_imu_dt": replay.DEFAULT_MAX_IMU_DT,
    }
    full_runner = _build_runner(
        replay_mode=replay_mode,
        runner_kwargs=runner_kwargs,
        image_processing_width=image_processing_width,
        camera_fx=camera_fx,
        camera_fy=camera_fy,
        camera_cx=camera_cx,
        camera_cy=camera_cy,
        camera_distortion=camera_distortion,
        camera_r_ic=camera_r_ic,
        camera_t_ic=camera_t_ic,
    )
    metrics, snapshot, snapshot_event_index = replay._run_replay(
        events,
        start_index=0,
        stop_processed_image=stop_processed_image,
        snapshot_frame=SNAPSHOT_FRAME,
        runner=full_runner,
        bias_mode="baseline",
    )
    if snapshot is None or snapshot_event_index is None:
        raise RuntimeError(f"Snapshot frame {SNAPSHOT_FRAME} was not reached")

    segment_runner = _build_runner(
        replay_mode=replay_mode,
        runner_kwargs=runner_kwargs,
        image_processing_width=image_processing_width,
        camera_fx=camera_fx,
        camera_fy=camera_fy,
        camera_cx=camera_cx,
        camera_cy=camera_cy,
        camera_distortion=camera_distortion,
        camera_r_ic=camera_r_ic,
        camera_t_ic=camera_t_ic,
    )
    if replay_mode == "propagation":
        segment_runner = replay.PropagationReplay.from_snapshot(
            snapshot,
            **runner_kwargs,
        )
    else:
        segment_runner = replay.MsckfReplay.from_snapshot(
            snapshot,
            **runner_kwargs,
            image_processing_width=image_processing_width,
            camera_fx=camera_fx,
            camera_fy=camera_fy,
            camera_cx=camera_cx,
            camera_cy=camera_cy,
            camera_distortion=camera_distortion,
            camera_extrinsics_convention="camera_in_imu",
            camera_R_IC=np.asarray(camera_r_ic, dtype=np.float64).reshape(3, 3),
            camera_t_IC=np.asarray(camera_t_ic, dtype=np.float64),
            min_triangulation_parallax_deg=replay.DEFAULT_MIN_TRIANGULATION_PARALLAX_DEG,
            max_batch_dx_pos_norm=0.5,
            max_batch_dx_vel_norm=1.0,
            max_batch_dx_bias_norm=replay.DEFAULT_MAX_BATCH_DX_BIAS_NORM,
        )
    segment_metrics, _, _ = replay._run_replay(
        events,
        start_index=snapshot_event_index,
        stop_processed_image=stop_processed_image,
        snapshot_frame=None,
        runner=segment_runner,
        bias_mode="baseline",
    )
    replay._verify_segment_matches(
        metrics,
        segment_metrics,
        start_frame=SNAPSHOT_FRAME,
        stop_frame=stop_processed_image,
    )

    discontinuities = max(
        (metric.discontinuity_count for metric in metrics),
        default=0,
    )
    segment_discontinuities = max(
        (metric.discontinuity_count for metric in segment_metrics),
        default=0,
    )
    if discontinuities != 0 or segment_discontinuities != 0:
        raise RuntimeError(
            "Unexpected discontinuities in deterministic replay: "
            f"full={discontinuities}, segment={segment_discontinuities}"
        )

    diagnostics = replay.build_onset_window_diagnostics(
        metrics,
        start_frame=WINDOW_START,
        stop_frame=WINDOW_STOP,
        vel_norm_limit=replay.DRIFT_THRESHOLD,
    )
    diagnostics.update(
        {
            "replay_mode": replay_mode,
            "processed_frames": len(metrics),
            "discontinuities": discontinuities,
            "segment_processed_frames": len(segment_metrics),
            "segment_replay_matches": True,
            "snapshot_frame": SNAPSHOT_FRAME,
            "stop_processed_image": int(stop_processed_image),
        }
    )
    return diagnostics


def main() -> int:
    args = parse_args()

    hcmut_kwargs = {
        "bag_path": replay._resolve_bag_path(None),
        "imu_topic": replay.DEFAULT_IMU_TOPIC,
        "image_topic": replay.DEFAULT_IMAGE_TOPIC,
        "image_processing_width": replay.DEFAULT_IMAGE_PROCESSING_WIDTH,
        "camera_fx": replay.DEFAULT_CAMERA_FX,
        "camera_fy": replay.DEFAULT_CAMERA_FY,
        "camera_cx": replay.DEFAULT_CAMERA_CX,
        "camera_cy": replay.DEFAULT_CAMERA_CY,
        "camera_distortion": replay.DEFAULT_CAMERA_DISTORTION,
        "camera_r_ic": replay.DEFAULT_CAMERA_R_IC,
        "camera_t_ic": replay.DEFAULT_CAMERA_T_IC,
        "stop_processed_image": 260,
    }
    euroc_kwargs = {
        "bag_path": _resolve_euroc_bag_path(),
        "imu_topic": DEFAULT_EUROC_IMU_TOPIC,
        "image_topic": DEFAULT_EUROC_IMAGE_TOPIC,
        "image_processing_width": DEFAULT_EUROC_IMAGE_PROCESSING_WIDTH,
        "camera_fx": DEFAULT_EUROC_CAMERA_FX,
        "camera_fy": DEFAULT_EUROC_CAMERA_FY,
        "camera_cx": DEFAULT_EUROC_CAMERA_CX,
        "camera_cy": DEFAULT_EUROC_CAMERA_CY,
        "camera_distortion": DEFAULT_EUROC_CAMERA_DISTORTION,
        "camera_r_ic": DEFAULT_EUROC_CAMERA_R_IC,
        "camera_t_ic": DEFAULT_EUROC_CAMERA_T_IC,
        "stop_processed_image": 400,
    }

    hcmut_propagation = _run_diagnostics(
        replay_mode="propagation",
        **hcmut_kwargs,
    )
    hcmut_msckf = _run_diagnostics(
        replay_mode="msckf",
        **hcmut_kwargs,
    )
    euroc_propagation = _run_diagnostics(
        replay_mode="propagation",
        **euroc_kwargs,
    )
    euroc_msckf = _run_diagnostics(
        replay_mode="msckf",
        **euroc_kwargs,
    )

    report = {
        "window": {"start_frame": WINDOW_START, "stop_frame": WINDOW_STOP},
        "hcmut": {
            "propagation": hcmut_propagation,
            "msckf": hcmut_msckf,
        },
        "euroc": {
            "propagation": euroc_propagation,
            "msckf": euroc_msckf,
        },
        "investigation": replay.classify_onset_comparison(
            propagation=hcmut_propagation,
            msckf=hcmut_msckf,
        ),
    }

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"onset_window_report={output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
