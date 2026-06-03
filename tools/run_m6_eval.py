#!/usr/bin/env python3
"""Run fixed M6 evaluation cases and capture machine-readable summaries."""

from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import signal
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


DEFAULT_DATASET_ROOT = Path("/home/ubuntu/VIO/dataset")
DEFAULT_WORKSPACE_ROOT = Path("/home/ubuntu/VIO/ros_ws")
DEFAULT_REPO_ROOT = Path("/home/ubuntu/VIO")
DEFAULT_ACCEPTANCE_CONTRACT = Path(__file__).with_name("m6_acceptance.json")
RESULT_TOPICS_WITH_GT = ("/vio/odometry", "/vio/gt_path")
RESULT_TOPICS_SMOKE = ("/vio/odometry",)

DEFAULT_ACCEPTANCE_CONTRACT_DATA: dict[str, Any] = {
    "schema_version": 1,
    "euroc_v101_easy": {
        "runtime": {
            "timed_out": False,
            "worker_crashes": 0,
            "image_queue_full": 0,
            "large_forward_gap_skips": 0,
            "large_image_timestamp_gaps": 0,
            "imu_init_resets": 0,
            "minimum_odometry_poses": 2800,
        },
        "metrics": {
            "ape_translation_rmse_m_max": 1.0,
            "baseline_regression_warning_rmse_m_max": 0.25,
            "require_rpe_translation_rmse": True,
            "alignment": "se3",
            "scale_correction": False,
        },
        "replay": {
            "minimum_processed_frames": 400,
            "discontinuities": 0,
            "segment_replay_matches": True,
        },
    },
    "hcmut_d455_smoke": {
        "runtime": {
            "timed_out": False,
            "worker_crashes": 0,
            "image_queue_full": 0,
            "large_image_timestamp_gaps": 0,
            "frame_gap_resets": 0,
            "imu_init_resets": 0,
            "minimum_accepted_updates": 1,
        },
        "replay": {
            "minimum_processed_frames": 260,
            "discontinuities": 0,
            "segment_replay_matches": True,
            "require_onset_report": True,
            "require_classification": True,
            "accuracy_gate": False,
        },
    },
    "report": {
        "required_case_fields": [
            "case",
            "config",
            "run",
            "runtime_health",
            "checks",
            "ok",
            "summary_path",
        ],
        "required_overall_fields": [
            "schema_version",
            "generated_at_utc",
            "contract_path",
            "cases",
            "replay",
            "checks",
            "ok",
        ],
    },
}

CASE_CONFIGS: dict[str, dict[str, Any]] = {
    "euroc_v101_easy": {
        "name": "euroc_v101_easy",
        "launch_file": "vio_system.launch.py",
        "metadata_relpath": "V1_01_easy/metadata.yaml",
        "bag_relpath": "V1_01_easy",
        "image_topic": "/cam0/image_raw",
        "gt_expected": True,
        "log_stem": "m6_euroc_v101_easy",
        "record_topics": RESULT_TOPICS_WITH_GT,
        "dataset_arg_name": "dataset_dir",
        "dataset_arg_kind": "root",
        "static_launch_args": (
            ("bag_rate", "0.10"),
            ("enable_rviz", "false"),
            ("publish_debug_image", "false"),
            ("image_processing_width", "640"),
            ("log_tracked_frames", "false"),
            ("runtime_diagnostics_enabled", "true"),
            ("diagnostics_log_every_n_frames", "10"),
        ),
        "expected_paths": ("V1_01_easy", "V1_01_easy/metadata.yaml"),
    },
    "hcmut_d455_smoke": {
        "name": "hcmut_d455_smoke",
        "launch_file": "vio_hcmut.launch.py",
        "metadata_relpath": "vio_hcmut_dataset/metadata.yaml",
        "bag_relpath": "vio_hcmut_dataset",
        "image_topic": "/camera/camera/color/image_raw",
        "gt_expected": False,
        "log_stem": "m6_hcmut_d455_smoke",
        "record_topics": RESULT_TOPICS_SMOKE,
        "dataset_arg_name": "bag_path",
        "dataset_arg_kind": "bag",
        "static_launch_args": (("bag_rate", "0.5"),),
        "expected_paths": ("vio_hcmut_dataset", "vio_hcmut_dataset/metadata.yaml"),
    },
}


def get_case_config(case_name: str) -> dict[str, Any]:
    if case_name not in CASE_CONFIGS:
        raise KeyError(f"Unknown case: {case_name}")
    return dict(CASE_CONFIGS[case_name])


def apply_launch_arg_overrides(
    case: dict[str, Any],
    overrides: tuple[str, ...] | list[str],
) -> dict[str, Any]:
    launch_args = dict(case["static_launch_args"])
    for item in overrides:
        if "=" not in item:
            raise ValueError(f"Launch override must use NAME=VALUE syntax: {item!r}")
        name, value = item.split("=", 1)
        if not name or not value:
            raise ValueError(f"Launch override must use NAME=VALUE syntax: {item!r}")
        launch_args[name] = value
    updated = dict(case)
    updated["static_launch_args"] = tuple(launch_args.items())
    return updated


def resolve_requested_frames(
    configured_frames: int | None,
    *,
    expected_frames: int,
) -> int:
    if configured_frames is None:
        return expected_frames
    if configured_frames < 1:
        raise ValueError("--stop-after-processed-frames must be positive")
    return configured_frames


def _parse_scalar(text: str) -> str:
    value = text.strip()
    if value[:1] in {"'", '"'} and value[-1:] == value[:1]:
        return value[1:-1]
    return value


def _load_topic_counts(metadata_path: Path) -> dict[str, int]:
    if not metadata_path.is_file():
        raise FileNotFoundError(f"Missing rosbag metadata: {metadata_path}")

    lines = metadata_path.read_text(encoding="utf-8").splitlines()
    in_topics = False
    current_name: str | None = None
    current_count: int | None = None
    topic_counts: dict[str, int] = {}

    def commit() -> None:
        nonlocal current_name, current_count
        if current_name is not None and current_count is not None:
            topic_counts[current_name] = current_count
        current_name = None
        current_count = None

    for raw_line in lines:
        stripped = raw_line.strip()
        indent = len(raw_line) - len(raw_line.lstrip(" "))
        if not in_topics:
            if stripped == "topics_with_message_count:":
                in_topics = True
            continue

        if indent <= 2 and stripped and not stripped.startswith("-"):
            commit()
            break

        if stripped.startswith("- "):
            commit()
            stripped = stripped[2:].strip()
            if stripped.startswith("message_count:"):
                current_count = int(_parse_scalar(stripped.split(":", 1)[1]))
            continue

        if stripped.startswith("message_count:"):
            current_count = int(_parse_scalar(stripped.split(":", 1)[1]))
            continue

        if stripped.startswith("name:"):
            current_name = _parse_scalar(stripped.split(":", 1)[1])

    commit()
    return topic_counts


def read_expected_message_count(metadata_path: Path | str, topic_name: str) -> int:
    topic_counts = _load_topic_counts(Path(metadata_path))
    if topic_name not in topic_counts:
        raise KeyError(f"Topic {topic_name!r} not found in {metadata_path}")
    return topic_counts[topic_name]


def _read_bag_duration_seconds(metadata_path: Path) -> float | None:
    match = re.search(
        r"duration:\s*\n\s+nanoseconds:\s*(\d+)",
        metadata_path.read_text(encoding="utf-8"),
    )
    if not match:
        return None
    return int(match.group(1)) / 1_000_000_000.0


def _parse_number(value: str) -> float | int:
    if re.fullmatch(r"-?\d+", value):
        return int(value)
    return float(value)


def extract_runtime_diagnostics(log_text: str) -> dict[str, float | int]:
    matches = re.findall(r"Runtime diagnostics:\s*(.*)", log_text)
    if not matches:
        return {}

    latest = matches[-1]
    diagnostics: dict[str, float | int] = {}
    patterns = {
        "frame": r"\bframe=(\d+)",
        "scale": r"\bscale=([-+0-9.eE]+)",
        "queue_current": r"\bqueue=(\d+)/(\d+)",
        "received": r"\breceived=(\d+)",
        "enqueued": r"\benqueued=(\d+)",
        "dropped": r"\bdropped=(\d+)",
        "dt_img_s": r"\bdt_img=([-+0-9.eE]+)s",
        "imu_count": r"\bimu_count=(\d+)",
        "imu_span_s": r"\bimu_span=([-+0-9.eE]+)s",
        "imu_large_dt_skips": r"\bimu_large_dt_skips=(\d+)",
        "imu_max_dt_s": r"\bimu_max_dt=([-+0-9.eE]+)s",
        "decode_resize_ms": r"\bdecode_resize=([-+0-9.eE]+)ms",
        "propagate_ms": r"\bpropagate=([-+0-9.eE]+)ms",
        "frontend_ms": r"\bfrontend=([-+0-9.eE]+)ms",
        "backend_ms": r"\bbackend=([-+0-9.eE]+)ms",
        "publish_ms": r"\bpublish=([-+0-9.eE]+)ms",
        "total_ms": r"\btotal=([-+0-9.eE]+)ms",
        "vel_norm": r"\bvel_norm=([-+0-9.eE]+)",
        "accel_bias_norm": r"\|ba\|=([-+0-9.eE]+)",
        "gyro_bias_norm": r"\|bg\|=([-+0-9.eE]+)",
    }
    for key, pattern in patterns.items():
        match = re.search(pattern, latest)
        if not match:
            continue
        if key == "queue_current":
            diagnostics["queue_current"] = int(match.group(1))
            diagnostics["queue_max"] = int(match.group(2))
            diagnostics["queue_size"] = diagnostics["queue_current"]
            diagnostics["queue_capacity"] = diagnostics["queue_max"]
        else:
            diagnostics[key] = _parse_number(match.group(1))
    return diagnostics


def extract_runtime_health(log_text: str) -> dict[str, Any]:
    samples = [
        extract_runtime_diagnostics(f"Runtime diagnostics: {payload}")
        for payload in re.findall(r"Runtime diagnostics:\s*(.*)", log_text)
    ]
    samples = [sample for sample in samples if sample]
    queue_warning_count = len(re.findall(r"image queue full", log_text, flags=re.I))
    forward_gap_skips = len(
        re.findall(r"Skipping visual update after large forward image gap:", log_text)
    )
    large_image_timestamp_gaps = len(
        re.findall(r"Large image timestamp gap:", log_text)
    )
    frame_gap_matches = re.findall(
        (
            r"Resetting temporal state after image timestamp discontinuity: "
            r"(?:kind=[^,]+,\s*)?dt_img=([-+0-9.eE]+)s"
        ),
        log_text,
    )
    frame_gap_resets = len(frame_gap_matches)
    backward_jump_resets = sum(float(value) <= 0.0 for value in frame_gap_matches)
    forward_gap_resets = sum(float(value) > 0.0 for value in frame_gap_matches)
    imu_init_resets = len(
        re.findall(r"Resetting IMU init buffer after timestamp discontinuity", log_text)
    )
    intentional_shutdown = _is_intentional_launch_shutdown(log_text)
    ordered_worker_crashes = len(re.findall(r"Ordered VIO Worker Crashed:", log_text))
    generic_worker_crash = not intentional_shutdown and bool(
        re.search(r"(process has died|Traceback)", log_text)
    )
    worker_crashes = ordered_worker_crashes or int(generic_worker_crash)

    first_nonrecoverable = next(
        (
            sample
            for sample in samples
            if int(sample.get("dropped", 0)) > 0
            or (
                int(sample.get("queue_capacity", 0)) > 0
                and int(sample.get("queue_size", 0))
                >= int(sample["queue_capacity"]) - 1
            )
        ),
        {},
    )
    latest = samples[-1] if samples else {}
    peak_queue_size = max(
        (int(sample.get("queue_size", 0)) for sample in samples),
        default=0,
    )
    peak_queue_capacity = max(
        (int(sample.get("queue_capacity", 0)) for sample in samples),
        default=0,
    )

    reasons: list[str] = []
    if not samples:
        reasons.append("missing runtime diagnostics")
    if queue_warning_count > 0:
        reasons.append(f"image queue full detected ({queue_warning_count})")
    if forward_gap_skips > 0:
        reasons.append(f"large forward gap skips detected ({forward_gap_skips})")
    if large_image_timestamp_gaps > 0:
        reasons.append(
            f"large image timestamp gaps detected ({large_image_timestamp_gaps})"
        )
    if frame_gap_resets > 0:
        reasons.append(f"frame-gap resets detected ({frame_gap_resets})")
    if imu_init_resets > 0:
        reasons.append(f"imu init resets detected ({imu_init_resets})")
    if worker_crashes > 0:
        reasons.append(f"worker crashes detected ({worker_crashes})")
    if int(latest.get("dropped", 0)) > 0:
        reasons.append(f"dropped frames detected ({int(latest.get('dropped', 0))})")

    return {
        "ok": not reasons,
        "reasons": reasons,
        "runtime_sample_count": len(samples),
        "image_queue_full": queue_warning_count,
        "large_forward_gap_skips": forward_gap_skips,
        "large_image_timestamp_gaps": large_image_timestamp_gaps,
        "frame_gap_resets": frame_gap_resets,
        "backward_jump_resets": backward_jump_resets,
        "forward_gap_resets": forward_gap_resets,
        "imu_init_resets": imu_init_resets,
        "worker_crashes": worker_crashes,
        "peak_queue_size": peak_queue_size,
        "peak_queue_capacity": peak_queue_capacity,
        "latest_dropped": int(latest.get("dropped", 0)),
        "latest_frame": int(latest.get("frame", -1)),
        "first_nonrecoverable_queue_frame": int(first_nonrecoverable.get("frame", -1)),
        "first_nonrecoverable_queue_size": int(
            first_nonrecoverable.get("queue_size", -1)
        ),
        "latest_runtime_diagnostics": latest,
    }


def extract_evo_metrics(output_text: str) -> dict[str, float]:
    metrics: dict[str, float] = {}
    for metric in ("rmse", "mean", "median", "std", "min", "max", "sse"):
        match = re.search(
            rf"(?im)^\s*{metric}\s+(?:\S+\s+)?([-+0-9.eE]+)\s*$",
            output_text,
        )
        if not match:
            match = re.search(
                rf"(?im)\b{metric}\b\s*[:=]\s*([-+0-9.eE]+)",
                output_text,
            )
        if match:
            metrics[metric] = float(match.group(1))
    return metrics


def _is_intentional_launch_shutdown(log_text: str) -> bool:
    return (
        "process has finished cleanly [pid" in log_text
        and "user interrupted with ctrl-c (SIGINT)" in log_text
        and "sending signal 'SIGINT' to process" in log_text
        and "KeyboardInterrupt" in log_text
    )


def check_hcmut_smoke_log(log_text: str) -> dict[str, Any]:
    reasons: list[str] = []
    large_image_timestamp_gaps = len(
        re.findall(r"Large image timestamp gap:", log_text)
    )
    frame_gap_matches = re.findall(
        (
            r"Resetting temporal state after image timestamp discontinuity: "
            r"(?:kind=[^,]+,\s*)?dt_img=([-+0-9.eE]+)s"
        ),
        log_text,
    )
    frame_gap_resets = len(frame_gap_matches)
    backward_jump_resets = sum(float(value) <= 0.0 for value in frame_gap_matches)
    forward_gap_resets = sum(float(value) > 0.0 for value in frame_gap_matches)
    imu_init_resets = len(
        re.findall(
            r"Resetting IMU init buffer after timestamp discontinuity",
            log_text,
        )
    )
    image_queue_full = len(re.findall(r"image queue full", log_text, flags=re.I))
    intentional_shutdown = _is_intentional_launch_shutdown(log_text)
    crash_signature = ""
    crash_match = re.search(r"Ordered VIO Worker Crashed:\s*(.+)", log_text)
    if crash_match:
        crash_signature = crash_match.group(1).strip()

    peak_vel_norm = 0.0
    for match in re.finditer(r"\bvel_norm=([-+0-9.eE]+)", log_text):
        peak_vel_norm = max(peak_vel_norm, float(match.group(1)))

    worker_crash = not intentional_shutdown and bool(
        crash_signature
        or re.search(
            r"(process has died|Traceback)",
            log_text,
        )
    )

    if worker_crash:
        reasons.append("worker crash detected")
    if large_image_timestamp_gaps > 0:
        reasons.append(
            f"large image timestamp gap detected ({large_image_timestamp_gaps})"
        )
    if frame_gap_resets > 0:
        reasons.append(f"frame-gap reset detected ({frame_gap_resets})")
    if imu_init_resets > 0:
        reasons.append(f"imu init reset detected ({imu_init_resets})")
    if image_queue_full > 0:
        reasons.append(f"image queue full detected ({image_queue_full})")

    msckf_present = "MSCKF diagnostics:" in log_text
    if not msckf_present:
        reasons.append("missing MSCKF diagnostics")

    accepted_updates = 0
    for match in re.finditer(r"MSCKF diagnostics:\s*(.*)", log_text):
        accepted_match = re.search(r"\baccepted=(\d+)", match.group(1))
        if accepted_match:
            accepted_updates = max(accepted_updates, int(accepted_match.group(1)))
    if accepted_updates <= 0:
        reasons.append("no accepted MSCKF updates observed")

    return {
        "ok": not reasons,
        "reasons": reasons,
        "intentional_shutdown": intentional_shutdown,
        "worker_crash": worker_crash,
        "crash_signature": crash_signature,
        "large_image_timestamp_gaps": large_image_timestamp_gaps,
        "frame_gap_resets": frame_gap_resets,
        "backward_jump_resets": backward_jump_resets,
        "forward_gap_resets": forward_gap_resets,
        "imu_init_resets": imu_init_resets,
        "image_queue_full": image_queue_full,
        "msckf_present": msckf_present,
        "accepted_updates": accepted_updates,
        "peak_vel_norm": peak_vel_norm,
    }


def _shell_prefix(workspace_root: Path) -> str:
    setup_bash = workspace_root / "install" / "setup.bash"
    if setup_bash.is_file():
        return f"source {shlex.quote(str(setup_bash))} && "
    return ""


def _spawn_logged_process(
    command: list[str],
    log_path: Path,
    workspace_root: Path,
    repo_root: Path,
) -> tuple[subprocess.Popen[str], Any]:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    shell_command = _shell_prefix(workspace_root) + shlex.join(command)
    log_file = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        ["bash", "-lc", shell_command],
        cwd=repo_root,
        stdout=log_file,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )
    return process, log_file


def _terminate_process(process: subprocess.Popen[str], grace_seconds: float = 10.0) -> None:
    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    deadline = time.time() + grace_seconds
    while time.time() < deadline:
        if process.poll() is not None:
            return
        time.sleep(0.2)
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return


def _launch_command(
    case: dict[str, Any],
    dataset_root: Path,
    workspace_root: Path,
    expected_frames: int,
) -> list[str]:
    launch_path = workspace_root / "src" / "vio_pkg" / "launch" / case["launch_file"]
    if not launch_path.is_file():
        raise FileNotFoundError(f"Missing launch file: {launch_path}")

    command = ["ros2", "launch", "vio_pkg", case["launch_file"]]
    dataset_value = dataset_root if case["dataset_arg_kind"] == "root" else dataset_root / case["bag_relpath"]
    command.append(f'{case["dataset_arg_name"]}:={dataset_value}')
    command.extend(f"{key}:={value}" for key, value in case["static_launch_args"])
    command.append(f"stop_after_processed_frames:={expected_frames}")
    return command


def _recorder_command(output_dir: Path, topics: tuple[str, ...]) -> list[str]:
    return ["ros2", "bag", "record", "-o", str(output_dir), *topics]


def _run_logged_command(
    command: list[str],
    log_path: Path,
    workspace_root: Path,
    repo_root: Path,
) -> subprocess.CompletedProcess[str]:
    shell_command = _shell_prefix(workspace_root) + shlex.join(command)
    completed = subprocess.run(
        ["bash", "-lc", shell_command],
        cwd=repo_root,
        capture_output=True,
        text=True,
    )
    log_path.write_text(
        f"$ {shell_command}\n\n{completed.stdout}\n{completed.stderr}",
        encoding="utf-8",
    )
    return completed


def _launch_log_indicates_node_finished(log_path: Path) -> bool:
    if not log_path.is_file():
        return False
    log_text = log_path.read_text(encoding="utf-8", errors="replace")
    return (
        "Stopping node: Reached stop_after_processed_frames=" in log_text
        or "process has finished cleanly" in log_text
        or "Ordered VIO Worker Crashed" in log_text
    )


def _run_case_orchestration(
    case: dict[str, Any],
    dataset_root: Path,
    workspace_root: Path,
    repo_root: Path,
    results_root: Path,
    expected_frames: int,
    duration_seconds: float | None,
) -> dict[str, Any]:
    output_bag_dir = results_root / case["log_stem"]
    if output_bag_dir.exists():
        raise FileExistsError(
            f"Refusing to overwrite existing recorded bag directory: {output_bag_dir}"
        )
    run_logs_dir = results_root / "run_logs"
    record_log = run_logs_dir / f'{case["log_stem"]}_record.log'
    launch_log = run_logs_dir / f'{case["log_stem"]}_launch.log'

    recorder, record_handle = _spawn_logged_process(
        _recorder_command(output_bag_dir, case["record_topics"]),
        record_log,
        workspace_root,
        repo_root,
    )
    time.sleep(2.0)

    launch_process, launch_handle = _spawn_logged_process(
        _launch_command(case, dataset_root, workspace_root, expected_frames),
        launch_log,
        workspace_root,
        repo_root,
    )

    timeout_seconds = None
    if duration_seconds is not None:
        rate = dict(case["static_launch_args"]).get("bag_rate")
        try:
            timeout_seconds = max(120.0, (duration_seconds / float(rate)) + 60.0)
        except (TypeError, ValueError, ZeroDivisionError):
            timeout_seconds = None

    timed_out = False
    node_finished = False
    try:
        deadline = None if timeout_seconds is None else time.time() + timeout_seconds
        while True:
            if _launch_log_indicates_node_finished(launch_log):
                node_finished = True
                _terminate_process(launch_process)
                launch_returncode = launch_process.wait(timeout=30)
                break

            launch_returncode = launch_process.poll()
            if launch_returncode is not None:
                break

            if deadline is not None and time.time() >= deadline:
                timed_out = True
                launch_returncode = None
                _terminate_process(launch_process)
                break

            time.sleep(1.0)
    finally:
        _terminate_process(recorder)
        record_returncode = recorder.wait(timeout=30)
        record_handle.close()
        launch_handle.close()

    return {
        "bag_dir": str(output_bag_dir),
        "record_log": str(record_log),
        "launch_log": str(launch_log),
        "launch_returncode": launch_returncode,
        "record_returncode": record_returncode,
        "node_finished": node_finished,
        "timed_out": timed_out,
    }


def _run_evo(
    case: dict[str, Any],
    repo_root: Path,
    workspace_root: Path,
    results_root: Path,
    bag_dir: Path,
) -> dict[str, Any]:
    if not case["gt_expected"]:
        return {"skipped": True, "reason": "ground truth not expected"}

    eval_dir = results_root / f'{case["log_stem"]}_eval'
    eval_dir.mkdir(parents=True, exist_ok=True)
    eval_log = results_root / "run_logs" / f'{case["log_stem"]}_evo.log'
    command = [
        sys.executable,
        str(repo_root / "tools" / "evaluate_m4.py"),
        str(bag_dir),
        str(eval_dir),
    ]
    completed = _run_logged_command(command, eval_log, workspace_root, repo_root)
    combined_output = completed.stdout + "\n" + completed.stderr
    evaluation_summary_path = eval_dir / "evaluation_summary.json"
    evaluation_summary: dict[str, Any] = {}
    if evaluation_summary_path.is_file():
        evaluation_summary = json.loads(
            evaluation_summary_path.read_text(encoding="utf-8")
        )
    compatibility_metrics = (
        evaluation_summary.get("evo", {}).get("ape_translation", {})
        or extract_evo_metrics(combined_output)
    )
    return {
        "skipped": False,
        "returncode": completed.returncode,
        "log_path": str(eval_log),
        "output_dir": str(eval_dir),
        "evaluation_summary_path": str(evaluation_summary_path),
        "evaluation_summary": evaluation_summary,
        "metrics": compatibility_metrics,
    }


def _case_summary_path(results_root: Path, case_name: str) -> Path:
    return results_root / f"{case_name}_summary.json"


def _validate_case_paths(case: dict[str, Any], dataset_root: Path, workspace_root: Path, repo_root: Path) -> dict[str, str]:
    checks = {
        "workspace_root": str(workspace_root),
        "repo_root": str(repo_root),
    }
    for relpath in case["expected_paths"]:
        checks[relpath] = str(dataset_root / relpath)
    return checks


def load_acceptance_contract(
    path: Path | str = DEFAULT_ACCEPTANCE_CONTRACT,
) -> dict[str, Any]:
    contract_path = Path(path)
    if contract_path.is_file():
        return json.loads(contract_path.read_text(encoding="utf-8"))
    return json.loads(json.dumps(DEFAULT_ACCEPTANCE_CONTRACT_DATA))


def evaluate_euroc_case(
    summary: dict[str, Any],
    contract: dict[str, Any],
) -> dict[str, bool]:
    runtime = contract["runtime"]
    metrics = contract["metrics"]
    health = summary.get("runtime_health", {})
    evaluation = summary.get("evo", {}).get("evaluation_summary", {})
    evo = evaluation.get("evo", {})
    ape = evo.get("ape_translation", {})
    rpe = evo.get("rpe_translation_1m", {})
    return {
        "not_timed_out": summary.get("run", {}).get("timed_out") is runtime["timed_out"],
        "worker_crashes": health.get("worker_crashes") == runtime["worker_crashes"],
        "image_queue_full": health.get("image_queue_full") == runtime["image_queue_full"],
        "large_forward_gap_skips": (
            health.get("large_forward_gap_skips") == runtime["large_forward_gap_skips"]
        ),
        "large_image_timestamp_gaps": (
            health.get("large_image_timestamp_gaps")
            == runtime["large_image_timestamp_gaps"]
        ),
        "imu_init_resets": health.get("imu_init_resets") == runtime["imu_init_resets"],
        "minimum_odometry_poses": (
            evaluation.get("odometry_poses", 0) >= runtime["minimum_odometry_poses"]
        ),
        "evo_returncode": summary.get("evo", {}).get("returncode") == 0,
        "ape_translation_rmse": (
            ape.get("rmse", float("inf")) <= metrics["ape_translation_rmse_m_max"]
        ),
        "alignment": ape.get("alignment") == metrics["alignment"],
        "scale_correction": ape.get("scale_correction") is metrics["scale_correction"],
        "rpe_translation_present": (
            (not metrics.get("require_rpe_translation_rmse", False))
            or "rmse" in rpe
        ),
    }


def collect_euroc_warnings(
    summary: dict[str, Any],
    contract: dict[str, Any],
) -> list[str]:
    metrics = contract["metrics"]
    warning_threshold = metrics.get("baseline_regression_warning_rmse_m_max")
    if warning_threshold is None:
        return []
    rmse = (
        summary.get("evo", {})
        .get("evaluation_summary", {})
        .get("evo", {})
        .get("ape_translation", {})
        .get("rmse")
    )
    if rmse is None or rmse <= warning_threshold:
        return []
    return [
        (
            "APE translation RMSE exceeds regression warning threshold "
            f"({rmse:.6f} > {warning_threshold:.6f})"
        )
    ]


def evaluate_hcmut_case(
    summary: dict[str, Any],
    contract: dict[str, Any],
) -> dict[str, bool]:
    runtime = contract["runtime"]
    health = summary.get("runtime_health", {})
    smoke = summary.get("smoke_check", {})
    return {
        "not_timed_out": summary.get("run", {}).get("timed_out") is runtime["timed_out"],
        "worker_crashes": health.get("worker_crashes") == runtime["worker_crashes"],
        "image_queue_full": health.get("image_queue_full") == runtime["image_queue_full"],
        "large_image_timestamp_gaps": (
            health.get("large_image_timestamp_gaps")
            == runtime["large_image_timestamp_gaps"]
        ),
        "frame_gap_resets": health.get("frame_gap_resets") == runtime["frame_gap_resets"],
        "imu_init_resets": health.get("imu_init_resets") == runtime["imu_init_resets"],
        "minimum_accepted_updates": (
            smoke.get("accepted_updates", 0) >= runtime["minimum_accepted_updates"]
        ),
    }


def _extract_replay_case_payload(payload: dict[str, Any], case_name: str) -> dict[str, Any]:
    case_payload = payload.get(case_name, {})
    if isinstance(case_payload.get("msckf"), dict):
        return case_payload["msckf"]
    return case_payload


def _extract_replay_classification(payload: dict[str, Any]) -> str:
    investigation = payload.get("investigation", {})
    if investigation.get("classification"):
        return str(investigation["classification"])
    for case_name in ("hcmut", "euroc"):
        case_payload = payload.get(case_name, {})
        if case_payload.get("classification"):
            return str(case_payload["classification"])
        if case_payload.get("classification_hint"):
            return str(case_payload["classification_hint"])
    return ""


def evaluate_replay_report(
    payload: dict[str, Any],
    contract: dict[str, Any],
) -> dict[str, bool]:
    euroc = _extract_replay_case_payload(payload, "euroc")
    hcmut = _extract_replay_case_payload(payload, "hcmut")
    euroc_contract = contract["euroc_v101_easy"]["replay"]
    hcmut_contract = contract["hcmut_d455_smoke"]["replay"]
    classification = _extract_replay_classification(payload)
    return {
        "euroc_processed_frames": (
            euroc.get("processed_frames", 0) >= euroc_contract["minimum_processed_frames"]
        ),
        "euroc_discontinuities": (
            euroc.get("discontinuities") == euroc_contract["discontinuities"]
        ),
        "euroc_segment_replay": (
            euroc.get("segment_replay_matches")
            is euroc_contract["segment_replay_matches"]
        ),
        "hcmut_processed_frames": (
            hcmut.get("processed_frames", 0) >= hcmut_contract["minimum_processed_frames"]
        ),
        "hcmut_discontinuities": (
            hcmut.get("discontinuities") == hcmut_contract["discontinuities"]
        ),
        "hcmut_segment_replay": (
            hcmut.get("segment_replay_matches")
            is hcmut_contract["segment_replay_matches"]
        ),
        "onset_report_present": (
            not hcmut_contract.get("require_onset_report", False)
            or bool(payload.get("hcmut")) or bool(payload.get("window"))
        ),
        "hcmut_classification": (
            not hcmut_contract.get("require_classification", False)
            or bool(classification)
        ),
    }


def run_replay_comparison(
    *,
    repo_root: Path,
    workspace_root: Path,
    results_root: Path,
    contract: dict[str, Any],
) -> dict[str, Any]:
    output_path = results_root / "hcmut_euroc_onset_report.json"
    log_path = results_root / "run_logs" / "hcmut_euroc_onset_report.log"
    command = [
        sys.executable,
        str(workspace_root / "src" / "vio_pkg" / "test" / "tools" / "compare_onset_windows.py"),
        "--output",
        str(output_path),
    ]
    completed = _run_logged_command(command, log_path, workspace_root, repo_root)
    payload: dict[str, Any] = {}
    if output_path.is_file():
        payload = json.loads(output_path.read_text(encoding="utf-8"))
    checks = evaluate_replay_report(payload, contract)
    return {
        "ok": completed.returncode == 0 and bool(payload) and all(checks.values()),
        "returncode": completed.returncode,
        "log_path": str(log_path),
        "report_path": str(output_path),
        "report": payload,
        "checks": checks,
    }


def build_overall_report(
    *,
    contract_path: str,
    summaries: list[dict[str, Any]],
    replay: dict[str, Any],
    required_case_fields: tuple[str, ...] | list[str] = (),
    required_overall_fields: tuple[str, ...] | list[str] = (),
) -> dict[str, Any]:
    checks = {
        "cases": all(bool(summary.get("ok", False)) for summary in summaries),
        "replay": bool(replay.get("ok", False)),
        "case_required_fields": all(
            all(key in summary for key in required_case_fields) for summary in summaries
        ),
    }
    report = {
        "schema_version": 1,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "contract_path": contract_path,
        "cases": summaries,
        "replay": replay,
        "checks": checks,
        "ok": False,
    }
    checks["required_fields"] = all(key in report for key in required_overall_fields)
    report["ok"] = all(checks.values())
    return report


def run_case(
    case_name: str,
    args: argparse.Namespace,
    contract_data: dict[str, Any],
) -> dict[str, Any]:
    case = apply_launch_arg_overrides(get_case_config(case_name), tuple(args.launch_arg))
    dataset_root = args.dataset_root.resolve()
    workspace_root = args.workspace_root.resolve()
    repo_root = args.repo_root.resolve()
    results_root = args.results_root.resolve()
    results_root.mkdir(parents=True, exist_ok=True)
    (results_root / "run_logs").mkdir(parents=True, exist_ok=True)

    metadata_path = dataset_root / case["metadata_relpath"]
    expected_frames = read_expected_message_count(metadata_path, case["image_topic"])
    requested_frames = resolve_requested_frames(
        args.stop_after_processed_frames,
        expected_frames=expected_frames,
    )
    duration_seconds = _read_bag_duration_seconds(metadata_path)
    summary: dict[str, Any] = {
        "case": case["name"],
        "config": case,
        "paths": _validate_case_paths(case, dataset_root, workspace_root, repo_root),
        "metadata_path": str(metadata_path),
        "expected_frames": expected_frames,
        "requested_frames": requested_frames,
        "duration_seconds": duration_seconds,
        "run": {"skipped": args.skip_run},
        "evo": {"skipped": True},
    }

    if not args.skip_run:
        run_summary = _run_case_orchestration(
            case=case,
            dataset_root=dataset_root,
            workspace_root=workspace_root,
            repo_root=repo_root,
            results_root=results_root,
            expected_frames=requested_frames,
            duration_seconds=duration_seconds,
        )
        summary["run"] = {"skipped": False, **run_summary}

        launch_log_text = Path(run_summary["launch_log"]).read_text(
            encoding="utf-8",
            errors="replace",
        )
        summary["runtime_diagnostics"] = extract_runtime_diagnostics(launch_log_text)
        summary["runtime_health"] = extract_runtime_health(launch_log_text)
        if case["name"] == "hcmut_d455_smoke":
            summary["smoke_check"] = check_hcmut_smoke_log(launch_log_text)
    else:
        summary["runtime_diagnostics"] = {}
        summary["runtime_health"] = {
            "ok": False,
            "reasons": ["run skipped; no log available"],
        }
        if case["name"] == "hcmut_d455_smoke":
            summary["smoke_check"] = {
                "ok": False,
                "reasons": ["run skipped; no log available"],
            }

    if not args.skip_evo and case["gt_expected"]:
        bag_dir = Path(summary["run"].get("bag_dir", results_root / case["log_stem"]))
        if bag_dir.exists():
            summary["evo"] = _run_evo(
                case=case,
                repo_root=repo_root,
                workspace_root=workspace_root,
                results_root=results_root,
                bag_dir=bag_dir,
            )
        else:
            summary["evo"] = {
                "skipped": True,
                "reason": f"missing bag dir: {bag_dir}",
            }
    elif case["gt_expected"]:
        summary["evo"] = {"skipped": True, "reason": "skip-evo requested"}
    else:
        summary["evo"] = {"skipped": True, "reason": "ground truth not expected"}

    if case["name"] == "euroc_v101_easy":
        summary["checks"] = evaluate_euroc_case(summary, contract_data["euroc_v101_easy"])
        summary["warnings"] = collect_euroc_warnings(
            summary,
            contract_data["euroc_v101_easy"],
        )
    else:
        summary["checks"] = evaluate_hcmut_case(summary, contract_data["hcmut_d455_smoke"])
        summary["warnings"] = []
    summary["ok"] = all(summary["checks"].values())

    summary_path = _case_summary_path(results_root, case["name"])
    summary["summary_path"] = str(summary_path)
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    return summary


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--case",
        choices=("euroc_v101_easy", "hcmut_d455_smoke", "all"),
        required=True,
    )
    parser.add_argument("--results-root", type=Path, required=True)
    parser.add_argument("--dataset-root", type=Path, default=DEFAULT_DATASET_ROOT)
    parser.add_argument("--workspace-root", type=Path, default=DEFAULT_WORKSPACE_ROOT)
    parser.add_argument("--repo-root", type=Path, default=DEFAULT_REPO_ROOT)
    parser.add_argument("--skip-run", action="store_true")
    parser.add_argument("--skip-evo", action="store_true")
    parser.add_argument("--skip-replay", action="store_true")
    parser.add_argument("--skip-summary", action="store_true")
    parser.add_argument(
        "--launch-arg",
        action="append",
        default=[],
        metavar="NAME=VALUE",
        help="Override one launch argument and record the effective value in the summary.",
    )
    parser.add_argument(
        "--stop-after-processed-frames",
        type=int,
        default=None,
        help="Request a bounded diagnostic slice instead of the metadata image count.",
    )
    parser.add_argument(
        "--allow-failures",
        action="store_true",
        help="Write reports but return zero for diagnostic matrix runs.",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    contract = load_acceptance_contract()
    results_root = args.results_root.resolve()
    results_root.mkdir(parents=True, exist_ok=True)
    (results_root / "run_logs").mkdir(parents=True, exist_ok=True)
    selected_cases = (
        list(CASE_CONFIGS)
        if args.case == "all"
        else [args.case]
    )
    summaries = [run_case(case_name, args, contract) for case_name in selected_cases]
    replay = (
        {"ok": False, "skipped": True, "reason": "skip-replay requested"}
        if args.skip_replay
        else run_replay_comparison(
            repo_root=args.repo_root.resolve(),
            workspace_root=args.workspace_root.resolve(),
            results_root=results_root,
            contract=contract,
        )
    )
    report = build_overall_report(
        contract_path=str(DEFAULT_ACCEPTANCE_CONTRACT),
        summaries=summaries,
        replay=replay,
        required_case_fields=contract["report"]["required_case_fields"],
        required_overall_fields=contract["report"]["required_overall_fields"],
    )
    report_path = results_root / "m6_report.json"
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True), encoding="utf-8")
    if not args.skip_summary:
        print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["ok"] or args.allow_failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
