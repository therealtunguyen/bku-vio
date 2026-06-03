"""
Pure-Python contract tests for the planned M6 evaluation runner helpers.

These tests intentionally import ``tools/run_m6_eval.py`` by file path so they
do not depend on package installation or ROS being available in the test
environment.
"""

from __future__ import annotations

import importlib.util
import json
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


def test_get_case_config_uses_conservative_euroc_automation_profile():
    module = load_run_m6_eval_module()

    case_config = module.get_case_config("euroc_v101_easy")
    launch_args = dict(case_config["static_launch_args"])

    assert launch_args["bag_rate"] == "0.10"
    assert launch_args["enable_rviz"] == "false"
    assert launch_args["publish_debug_image"] == "false"
    assert launch_args["image_processing_width"] == "640"
    assert launch_args["log_tracked_frames"] == "false"
    assert launch_args["runtime_diagnostics_enabled"] == "true"
    assert launch_args["diagnostics_log_every_n_frames"] == "10"


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


def test_extract_runtime_health_reports_queue_overflow_and_first_nonrecoverable_sample():
    module = load_run_m6_eval_module()
    log_text = """
[INFO] Runtime diagnostics: frame=300, queue=3/50, received=304, enqueued=304, dropped=0, total=120.0ms
[INFO] Runtime diagnostics: frame=310, queue=49/50, received=360, enqueued=360, dropped=0, total=740.0ms
[WARN] Image queue full. Dropped stale frame.
[WARN] Skipping visual update after large forward image gap: dt_img=0.1000s
[INFO] Runtime diagnostics: frame=320, queue=50/50, received=422, enqueued=422, dropped=2, total=910.0ms
""".strip()

    health = module.extract_runtime_health(log_text)

    assert health["ok"] is False
    assert health["image_queue_full"] == 1
    assert health["large_forward_gap_skips"] == 1
    assert health["peak_queue_size"] == 50
    assert health["peak_queue_capacity"] == 50
    assert health["latest_dropped"] == 2
    assert health["first_nonrecoverable_queue_frame"] == 310
    assert health["first_nonrecoverable_queue_size"] == 49


def test_extract_runtime_health_accepts_clean_diagnostic_log():
    module = load_run_m6_eval_module()
    log_text = """
[INFO] Runtime diagnostics: frame=300, queue=3/50, received=304, enqueued=304, dropped=0, total=120.0ms
[INFO] Runtime diagnostics: frame=310, queue=1/50, received=314, enqueued=314, dropped=0, total=140.0ms
""".strip()

    health = module.extract_runtime_health(log_text)

    assert health["ok"] is True
    assert health["runtime_sample_count"] == 2
    assert health["first_nonrecoverable_queue_frame"] == -1


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


def test_apply_launch_arg_overrides_replaces_existing_values_and_appends_new_values():
    module = load_run_m6_eval_module()
    case = module.get_case_config("euroc_v101_easy")

    updated = module.apply_launch_arg_overrides(
        case,
        ("bag_rate=0.10", "image_processing_width=480", "diagnostics_log_every_n_frames=1"),
    )

    launch_args = dict(updated["static_launch_args"])
    assert launch_args["bag_rate"] == "0.10"
    assert launch_args["image_processing_width"] == "480"
    assert launch_args["diagnostics_log_every_n_frames"] == "1"


def test_apply_launch_arg_overrides_rejects_malformed_value():
    module = load_run_m6_eval_module()

    with pytest.raises(ValueError, match="NAME=VALUE"):
        module.apply_launch_arg_overrides(
            module.get_case_config("euroc_v101_easy"),
            ("bag_rate",),
        )


def test_requested_frame_limit_defaults_to_metadata_count_and_accepts_positive_override():
    module = load_run_m6_eval_module()

    assert module.resolve_requested_frames(None, expected_frames=2912) == 2912
    assert module.resolve_requested_frames(300, expected_frames=2912) == 300

    with pytest.raises(ValueError, match="positive"):
        module.resolve_requested_frames(0, expected_frames=2912)


def test_evaluate_euroc_case_requires_clean_runtime_pose_count_and_ape():
    module = load_run_m6_eval_module()
    contract = {
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
            "require_rpe_translation_rmse": True,
            "alignment": "se3",
            "scale_correction": False,
        },
    }
    summary = {
        "run": {"timed_out": False},
        "runtime_health": {
            "worker_crashes": 0,
            "image_queue_full": 0,
            "large_forward_gap_skips": 0,
            "large_image_timestamp_gaps": 0,
            "imu_init_resets": 0,
        },
        "evo": {
            "returncode": 0,
            "evaluation_summary": {
                "odometry_poses": 2891,
                "evo": {
                    "ape_translation": {
                        "rmse": 0.134976,
                        "alignment": "se3",
                        "scale_correction": False,
                    },
                    "rpe_translation_1m": {"rmse": 0.022},
                },
            },
        },
    }

    checks = module.evaluate_euroc_case(summary, contract)

    assert all(checks.values())


def test_evaluate_hcmut_case_requires_clean_runtime_and_accepted_update():
    module = load_run_m6_eval_module()
    contract = {
        "runtime": {
            "timed_out": False,
            "worker_crashes": 0,
            "image_queue_full": 0,
            "large_image_timestamp_gaps": 0,
            "frame_gap_resets": 0,
            "imu_init_resets": 0,
            "minimum_accepted_updates": 1,
        }
    }
    summary = {
        "run": {"timed_out": False},
        "runtime_health": {
            "worker_crashes": 0,
            "image_queue_full": 0,
            "large_image_timestamp_gaps": 0,
            "frame_gap_resets": 0,
            "imu_init_resets": 0,
        },
        "smoke_check": {
            "accepted_updates": 2,
        },
    }

    checks = module.evaluate_hcmut_case(summary, contract)

    assert all(checks.values())


def test_build_overall_report_is_red_when_any_case_is_red():
    module = load_run_m6_eval_module()

    report = module.build_overall_report(
        contract_path="tools/m6_acceptance.json",
        summaries=[{"case": "euroc_v101_easy", "ok": False}],
        replay={"ok": True},
        required_case_fields=("case", "ok"),
        required_overall_fields=("schema_version", "checks"),
    )

    assert report["ok"] is False


def test_evaluate_replay_report_requires_deterministic_rows_and_classification():
    module = load_run_m6_eval_module()
    contract = {
        "euroc_v101_easy": {
            "replay": {
                "minimum_processed_frames": 400,
                "discontinuities": 0,
                "segment_replay_matches": True,
            },
        },
        "hcmut_d455_smoke": {
            "replay": {
                "minimum_processed_frames": 260,
                "discontinuities": 0,
                "segment_replay_matches": True,
                "require_onset_report": True,
                "require_classification": True,
            },
        },
    }
    payload = {
        "euroc": {
            "msckf": {
                "processed_frames": 400,
                "discontinuities": 0,
                "segment_replay_matches": True,
            },
        },
        "hcmut": {
            "msckf": {
                "processed_frames": 260,
                "discontinuities": 0,
                "segment_replay_matches": True,
            },
        },
        "investigation": {"classification": "propagation_side"},
    }

    checks = module.evaluate_replay_report(payload, contract)

    assert all(checks.values())


def test_run_replay_comparison_reads_payload_and_evaluates_checks(tmp_path, monkeypatch):
    module = load_run_m6_eval_module()
    results_root = tmp_path / "results"
    (results_root / "run_logs").mkdir(parents=True)
    contract = {
        "euroc_v101_easy": {
            "replay": {
                "minimum_processed_frames": 10,
                "discontinuities": 0,
                "segment_replay_matches": True,
            },
        },
        "hcmut_d455_smoke": {
            "replay": {
                "minimum_processed_frames": 10,
                "discontinuities": 0,
                "segment_replay_matches": True,
                "require_onset_report": True,
                "require_classification": True,
            },
        },
    }

    def fake_run(command, log_path, workspace_root, repo_root):
        payload = {
            "euroc": {
                "msckf": {
                    "processed_frames": 10,
                    "discontinuities": 0,
                    "segment_replay_matches": True,
                }
            },
            "hcmut": {
                "msckf": {
                    "processed_frames": 10,
                    "discontinuities": 0,
                    "segment_replay_matches": True,
                }
            },
            "investigation": {"classification": "mixed_or_visual_side"},
        }
        output_index = command.index("--output") + 1
        Path(command[output_index]).write_text(
            json.dumps(payload),
            encoding="utf-8",
        )
        return module.subprocess.CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(module, "_run_logged_command", fake_run)

    replay = module.run_replay_comparison(
        repo_root=tmp_path,
        workspace_root=tmp_path,
        results_root=results_root,
        contract=contract,
    )

    assert replay["ok"] is True
    assert replay["checks"]["hcmut_classification"] is True
    assert Path(replay["report_path"]).is_file()


def test_load_acceptance_contract_falls_back_to_builtin_when_file_is_missing(tmp_path):
    module = load_run_m6_eval_module()

    contract = module.load_acceptance_contract(tmp_path / "missing.json")

    assert contract["schema_version"] == 1
    assert "euroc_v101_easy" in contract


def test_main_returns_zero_when_allow_failures_is_set(tmp_path, monkeypatch):
    module = load_run_m6_eval_module()
    results_root = tmp_path / "results"
    contract = {
        "schema_version": 1,
        "report": {
            "required_case_fields": ["case", "ok"],
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

    def fake_run_case(case_name, args, contract_data):
        return {"case": case_name, "ok": False, "summary_path": str(results_root / "case.json")}

    monkeypatch.setattr(module, "load_acceptance_contract", lambda path=None: contract)
    monkeypatch.setattr(module, "run_case", fake_run_case)
    monkeypatch.setattr(
        module,
        "run_replay_comparison",
        lambda **kwargs: {"ok": False, "checks": {}},
    )

    exit_code = module.main(
        [
            "--case",
            "euroc_v101_easy",
            "--results-root",
            str(results_root),
            "--allow-failures",
            "--skip-replay",
            "--skip-summary",
        ]
    )

    assert exit_code == 0
    assert (results_root / "m6_report.json").is_file()


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


def test_check_hcmut_smoke_log_reports_peak_velocity_and_crash_signature():
    module = load_run_m6_eval_module()
    log_text = """
    [vio_system_node]: Runtime diagnostics: frame=650, vel_norm=158.679, dt_img=0.0333s
    [vio_system_node]: Ordered VIO Worker Crashed: OpenCV(4.13.0) calcOpticalFlowPyrLK
    Traceback (most recent call last):
    """

    smoke = module.check_hcmut_smoke_log(log_text)

    assert smoke["worker_crash"] is True
    assert smoke["peak_vel_norm"] == 158.679
    assert "OpenCV(4.13.0) calcOpticalFlowPyrLK" in smoke["crash_signature"]


def test_check_hcmut_smoke_log_counts_backward_gap_resets_separately():
    module = load_run_m6_eval_module()
    log_text = """
    [vio_system_node]: Resetting temporal state after image timestamp discontinuity: dt_img=-372864616.9668s, reset_count=1
    [vio_system_node]: Runtime diagnostics: frame=10, vel_norm=0.002, dt_img=372864617.0501s
    [vio_system_node]: MSCKF diagnostics: mature=0, accepted=0
    """

    smoke = module.check_hcmut_smoke_log(log_text)

    assert smoke["frame_gap_resets"] == 1
    assert smoke["backward_jump_resets"] == 1
    assert smoke["peak_vel_norm"] == 0.002


def test_check_hcmut_smoke_log_accepts_reset_lines_with_gap_kind_prefix():
    module = load_run_m6_eval_module()
    log_text = """
    [vio_system_node]: Resetting temporal state after image timestamp discontinuity: kind=backward_jump, dt_img=-372864616.9668s, reset_count=1
    [vio_system_node]: Runtime diagnostics: frame=10, vel_norm=0.002, dt_img=372864617.0501s
    [vio_system_node]: MSCKF diagnostics: mature=0, accepted=0
    """

    smoke = module.check_hcmut_smoke_log(log_text)

    assert smoke["frame_gap_resets"] == 1
    assert smoke["backward_jump_resets"] == 1
    assert smoke["forward_gap_resets"] == 0


def test_check_hcmut_smoke_log_rejects_large_gap_warning_even_without_reset():
    module = load_run_m6_eval_module()
    log_text = """
[WARN] [1716000000.100000000] [vio_system]: Large image timestamp gap: 2.5310s
[INFO] [1716000000.150000000] [vio_system]: MSCKF diagnostics: mature=4, triangulated=4, accepted=2, batch_rejected=0, gated_out=0, invalid=0, triangulation_failed=0, rejected_ill_conditioned=0, rejected_unreasonable_dx=0, skipped=0, rows=54, dx_pos=1.000e-03, dx_vel=2.000e-03, dx_bg=3.000e-05, dx_ba=4.000e-05, innovation_cond=1.200e+00, pos=[0.0 0.0 0.0], vel=[0.0 0.0 0.0], ba=[0.0 0.0 0.0], bg=[0.0 0.0 0.0]
[INFO] [1716000000.200000000] [vio_system]: Runtime diagnostics: frame=80, shape=(423, 752), scale=0.587500, queue=0/50, received=81, enqueued=81, dropped=0, dt_img=0.0321s, imu_count=14, imu_span=0.0318s, imu_large_dt_skips=0, imu_max_dt=0.0049s, decode_resize=2.4ms, propagate=1.5ms, frontend=6.7ms, backend=2.0ms, publish=0.7ms, total=13.3ms, pos=[0.2 0.1 0.4], vel_norm=0.172, |ba|=0.025, |bg|=0.002
""".strip()

    result = module.check_hcmut_smoke_log(log_text)

    assert result["ok"] is False
    assert "large image timestamp gap detected (1)" in result["reasons"]


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
