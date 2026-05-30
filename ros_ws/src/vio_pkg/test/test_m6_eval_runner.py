"""
Pure-Python contract tests for the planned M6 evaluation runner helpers.

These tests intentionally import ``tools/run_m6_eval.py`` by file path so they
do not depend on package installation or ROS being available in the test
environment.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


REPO_ROOT = Path(__file__).resolve().parents[4]
RUNNER_PATH = REPO_ROOT / "tools" / "run_m6_eval.py"


def load_run_m6_eval_module():
    if not RUNNER_PATH.is_file():
        raise AssertionError(
            f"Expected planned runner at {RUNNER_PATH}, but the file does not exist."
        )

    spec = importlib.util.spec_from_file_location("run_m6_eval_under_test", RUNNER_PATH)
    if spec is None or spec.loader is None:
        raise AssertionError(f"Could not create import spec for {RUNNER_PATH}.")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    "case_name",
    (
        "euroc_v101_easy",
        "hcmut_d455_smoke",
    ),
)
def test_get_case_config_accepts_known_case_names(case_name):
    module = load_run_m6_eval_module()

    case_config = module.get_case_config(case_name)

    assert isinstance(case_config, dict)
    assert case_config["name"] == case_name


def test_get_case_config_rejects_unknown_case_name():
    module = load_run_m6_eval_module()

    with pytest.raises((KeyError, ValueError)):
        module.get_case_config("not_a_real_case")


def test_get_case_config_uses_headless_euroc_automation_profile():
    module = load_run_m6_eval_module()

    case_config = module.get_case_config("euroc_v101_easy")
    launch_args = dict(case_config["static_launch_args"])

    assert launch_args["bag_rate"] == "0.1"
    assert launch_args["enable_rviz"] == "false"
    assert launch_args["publish_debug_image"] == "false"
    assert launch_args["log_tracked_frames"] == "false"


def test_read_expected_message_count_reads_synthetic_rosbag2_metadata(tmp_path):
    module = load_run_m6_eval_module()
    metadata_path = tmp_path / "metadata.yaml"
    metadata_path.write_text(
        """
rosbag2_bagfile_information:
  version: 5
  storage_identifier: sqlite3
  duration:
    nanoseconds: 123456789
  starting_time:
    nanoseconds_since_epoch: 1716000000000000000
  message_count: 579
  topics_with_message_count:
    - topic_metadata:
        name: /cam0/image_raw
        type: sensor_msgs/msg/Image
        serialization_format: cdr
      message_count: 123
    - topic_metadata:
        name: /imu0
        type: sensor_msgs/msg/Imu
        serialization_format: cdr
      message_count: 456
""".strip(),
        encoding="utf-8",
    )

    assert module.read_expected_message_count(metadata_path, "/cam0/image_raw") == 123
    assert module.read_expected_message_count(metadata_path, "/imu0") == 456


def test_extract_runtime_diagnostics_parses_queue_counts_and_timings():
    module = load_run_m6_eval_module()
    log_text = (
        "[INFO] [1716000000.123456789] [vio_system]: Runtime diagnostics: "
        "frame=40, shape=(423, 752), scale=0.587500, queue=3/50, received=41, "
        "enqueued=41, dropped=0, dt_img=0.0322s, imu_count=14, imu_span=0.0318s, "
        "imu_large_dt_skips=0, imu_max_dt=0.0049s, decode_resize=2.3ms, "
        "propagate=1.4ms, frontend=6.8ms, backend=1.9ms, publish=0.7ms, "
        "total=13.4ms, pos=[0.1 0.2 0.3], vel_norm=0.165, |ba|=0.024, |bg|=0.002"
    )

    diagnostics = module.extract_runtime_diagnostics(log_text)

    assert diagnostics["frame"] == 40
    assert diagnostics["queue_size"] == 3
    assert diagnostics["queue_capacity"] == 50
    assert diagnostics["received"] == 41
    assert diagnostics["enqueued"] == 41
    assert diagnostics["dropped"] == 0
    assert diagnostics["dt_img_s"] == pytest.approx(0.0322)
    assert diagnostics["imu_count"] == 14
    assert diagnostics["imu_span_s"] == pytest.approx(0.0318)
    assert diagnostics["imu_large_dt_skips"] == 0
    assert diagnostics["imu_max_dt_s"] == pytest.approx(0.0049)
    assert diagnostics["decode_resize_ms"] == pytest.approx(2.3)
    assert diagnostics["propagate_ms"] == pytest.approx(1.4)
    assert diagnostics["frontend_ms"] == pytest.approx(6.8)
    assert diagnostics["backend_ms"] == pytest.approx(1.9)
    assert diagnostics["publish_ms"] == pytest.approx(0.7)
    assert diagnostics["total_ms"] == pytest.approx(13.4)
    assert diagnostics["vel_norm"] == pytest.approx(0.165)
    assert diagnostics["accel_bias_norm"] == pytest.approx(0.024)
    assert diagnostics["gyro_bias_norm"] == pytest.approx(0.002)


def test_extract_evo_metrics_parses_representative_evo_ape_output():
    module = load_run_m6_eval_module()
    output_text = """
APE w.r.t. translation part (m)
(with SE(3) Umeyama alignment)

       max	0.168742
      mean	0.072345
    median	0.069871
       min	0.012004
      rmse	0.081234
       sse	1.978654
       std	0.036789
""".strip()

    metrics = module.extract_evo_metrics(output_text)

    assert metrics["rmse"] == pytest.approx(0.081234)
    assert metrics["mean"] == pytest.approx(0.072345)
    assert metrics["median"] == pytest.approx(0.069871)
    assert metrics["std"] == pytest.approx(0.036789)
    assert metrics["min"] == pytest.approx(0.012004)
    assert metrics["max"] == pytest.approx(0.168742)
    assert metrics["sse"] == pytest.approx(1.978654)


def test_check_hcmut_smoke_log_accepts_clean_smoke_run():
    module = load_run_m6_eval_module()
    log_text = """
[INFO] [1716000000.050000000] [vio_system]: MSCKF diagnostics: mature=4, triangulated=4, accepted=2, batch_rejected=0, gated_out=0, invalid=0, triangulation_failed=0, rejected_ill_conditioned=0, rejected_unreasonable_dx=0, skipped=0, rows=54, dx_pos=1.000e-03, dx_vel=2.000e-03, dx_bg=3.000e-05, dx_ba=4.000e-05, innovation_cond=1.200e+00, pos=[0.0 0.0 0.0], vel=[0.0 0.0 0.0], ba=[0.0 0.0 0.0], bg=[0.0 0.0 0.0]
[INFO] [1716000000.100000000] [vio_system]: Runtime diagnostics: frame=40, shape=(423, 752), scale=0.587500, queue=0/50, received=41, enqueued=41, dropped=0, dt_img=0.0322s, imu_count=14, imu_span=0.0318s, imu_large_dt_skips=0, imu_max_dt=0.0049s, decode_resize=2.3ms, propagate=1.4ms, frontend=6.8ms, backend=1.9ms, publish=0.7ms, total=13.4ms, pos=[0.1 0.2 0.3], vel_norm=0.165, |ba|=0.024, |bg|=0.002
[INFO] [1716000000.200000000] [vio_system]: Runtime diagnostics: frame=80, shape=(423, 752), scale=0.587500, queue=0/50, received=81, enqueued=81, dropped=0, dt_img=0.0321s, imu_count=14, imu_span=0.0318s, imu_large_dt_skips=0, imu_max_dt=0.0049s, decode_resize=2.4ms, propagate=1.5ms, frontend=6.7ms, backend=2.0ms, publish=0.7ms, total=13.3ms, pos=[0.2 0.1 0.4], vel_norm=0.172, |ba|=0.025, |bg|=0.002
""".strip()

    result = module.check_hcmut_smoke_log(log_text)

    assert result["ok"] is True
    assert result["accepted_updates"] == 2
    assert result["reasons"] == []


def test_check_hcmut_smoke_log_ignores_intentional_shutdown_after_clean_playback():
    module = load_run_m6_eval_module()
    log_text = """
[INFO] [ros2-1]: process has finished cleanly [pid 2440]
[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
[INFO] [vio_system_node-2]: sending signal 'SIGINT' to process[vio_system_node-2]
[vio_system_node-2] Traceback (most recent call last):
[vio_system_node-2] KeyboardInterrupt
[ERROR] [vio_system_node-2]: process has died [pid 2442, exit code -2, cmd '/home/ubuntu/VIO/ros_ws/install/vio_pkg/lib/vio_pkg/vio_system_node']
[INFO] [1716000000.050000000] [vio_system]: MSCKF diagnostics: mature=4, triangulated=4, accepted=2, batch_rejected=0, gated_out=0, invalid=0, triangulation_failed=0, rejected_ill_conditioned=0, rejected_unreasonable_dx=0, skipped=0, rows=54, dx_pos=1.000e-03, dx_vel=2.000e-03, dx_bg=3.000e-05, dx_ba=4.000e-05, innovation_cond=1.200e+00, pos=[0.0 0.0 0.0], vel=[0.0 0.0 0.0], ba=[0.0 0.0 0.0], bg=[0.0 0.0 0.0]
""".strip()

    result = module.check_hcmut_smoke_log(log_text)

    assert result["ok"] is True
    assert result["worker_crash"] is False
    assert result["reasons"] == []


@pytest.mark.parametrize(
    "log_text",
    (
        """
[INFO] [1716000000.100000000] [vio_system]: Runtime diagnostics: frame=40, shape=(423, 752), scale=0.587500, queue=0/50, received=41, enqueued=41, dropped=2, dt_img=0.0322s, imu_count=14, imu_span=0.0318s, imu_large_dt_skips=0, imu_max_dt=0.0049s, decode_resize=2.3ms, propagate=1.4ms, frontend=6.8ms, backend=1.9ms, publish=0.7ms, total=13.4ms, pos=[0.1 0.2 0.3], vel_norm=0.165, |ba|=0.024, |bg|=0.002
""".strip(),
        """
[WARN] [1716000000.100000000] [vio_system]: Large image timestamp gap: 2.5310s
[INFO] [1716000000.200000000] [vio_system]: Runtime diagnostics: frame=80, shape=(423, 752), scale=0.587500, queue=0/50, received=81, enqueued=81, dropped=0, dt_img=0.0321s, imu_count=14, imu_span=0.0318s, imu_large_dt_skips=0, imu_max_dt=0.0049s, decode_resize=2.4ms, propagate=1.5ms, frontend=6.7ms, backend=2.0ms, publish=0.7ms, total=13.3ms, pos=[0.2 0.1 0.4], vel_norm=0.172, |ba|=0.025, |bg|=0.002
""".strip(),
        """
Traceback (most recent call last):
  File "/home/ubuntu/VIO/ros_ws/install/vio_pkg/lib/vio_pkg/vio_node", line 1, in <module>
RuntimeError: backend worker crashed
""".strip(),
    ),
)
def test_check_hcmut_smoke_log_rejects_failure_sentinels(log_text):
    module = load_run_m6_eval_module()

    result = module.check_hcmut_smoke_log(log_text)

    assert result["ok"] is False
    assert result["reasons"]
