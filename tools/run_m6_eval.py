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
from pathlib import Path
from typing import Any


DEFAULT_DATASET_ROOT = Path("/home/ubuntu/VIO/dataset")
DEFAULT_WORKSPACE_ROOT = Path("/home/ubuntu/VIO/ros_ws")
DEFAULT_REPO_ROOT = Path("/home/ubuntu/VIO")
RESULT_TOPICS_WITH_GT = ("/vio/odometry", "/vio/gt_path")
RESULT_TOPICS_SMOKE = ("/vio/odometry",)

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
            ("bag_rate", "0.1"),
            ("enable_rviz", "false"),
            ("publish_debug_image", "false"),
            ("log_tracked_frames", "false"),
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
    return {
        "skipped": False,
        "returncode": completed.returncode,
        "log_path": str(eval_log),
        "output_dir": str(eval_dir),
        "metrics": extract_evo_metrics(combined_output),
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


def run_case(case_name: str, args: argparse.Namespace) -> dict[str, Any]:
    case = CASE_CONFIGS[case_name]
    dataset_root = args.dataset_root.resolve()
    workspace_root = args.workspace_root.resolve()
    repo_root = args.repo_root.resolve()
    results_root = args.results_root.resolve()
    results_root.mkdir(parents=True, exist_ok=True)
    (results_root / "run_logs").mkdir(parents=True, exist_ok=True)

    metadata_path = dataset_root / case["metadata_relpath"]
    expected_frames = read_expected_message_count(metadata_path, case["image_topic"])
    duration_seconds = _read_bag_duration_seconds(metadata_path)
    summary: dict[str, Any] = {
        "case": case["name"],
        "config": get_case_config(case["name"]),
        "paths": _validate_case_paths(case, dataset_root, workspace_root, repo_root),
        "metadata_path": str(metadata_path),
        "expected_frames": expected_frames,
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
            expected_frames=expected_frames,
            duration_seconds=duration_seconds,
        )
        summary["run"] = {"skipped": False, **run_summary}

        launch_log_text = Path(run_summary["launch_log"]).read_text(
            encoding="utf-8",
            errors="replace",
        )
        summary["runtime_diagnostics"] = extract_runtime_diagnostics(launch_log_text)
        if case["name"] == "hcmut_d455_smoke":
            summary["smoke_check"] = check_hcmut_smoke_log(launch_log_text)
    else:
        summary["runtime_diagnostics"] = {}
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

    summary_path = _case_summary_path(results_root, case["name"])
    summary["summary_path"] = str(summary_path)
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    return summary


def parse_args() -> argparse.Namespace:
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
    parser.add_argument("--skip-summary", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    selected_cases = (
        list(CASE_CONFIGS)
        if args.case == "all"
        else [args.case]
    )
    summaries = [run_case(case_name, args) for case_name in selected_cases]
    if not args.skip_summary:
        print(json.dumps(summaries, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
