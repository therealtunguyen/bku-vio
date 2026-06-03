"""
Pure-Python contract tests for ``tools/evaluate_m4.py``.

These tests import the script by file path and avoid ROS runtime dependencies.
"""

from __future__ import annotations

import importlib.util
import json
import zipfile
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
EVALUATE_M4_PATH = REPO_ROOT / "tools" / "evaluate_m4.py"


def load_evaluate_m4_module():
    if not EVALUATE_M4_PATH.is_file():
        raise AssertionError(
            f"Expected evaluator at {EVALUATE_M4_PATH}, but the file does not exist."
        )

    spec = importlib.util.spec_from_file_location("evaluate_m4_under_test", EVALUATE_M4_PATH)
    if spec is None or spec.loader is None:
        raise AssertionError(f"Could not create import spec for {EVALUATE_M4_PATH}.")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_evo_zip(path: Path, *, title: str, rmse: float) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(path, "w") as archive:
        archive.writestr(
            "info.json",
            json.dumps({"title": title, "label": "translation"}),
        )
        archive.writestr(
            "stats.json",
            json.dumps({"rmse": rmse, "mean": rmse / 2.0}),
        )


def test_read_evo_stats_reads_stats_and_alignment_from_saved_result_zip(tmp_path):
    module = load_evaluate_m4_module()
    zip_path = tmp_path / "ape_aligned.zip"
    write_evo_zip(
        zip_path,
        title="APE w.r.t. translation part (m)\n(with SE(3) Umeyama alignment)",
        rmse=0.123,
    )

    stats = module.read_evo_stats(zip_path)

    assert stats["rmse"] == 0.123
    assert stats["alignment"] == "se3"
    assert stats["scale_correction"] is False
    assert "info" in stats


def test_run_evo_returns_structured_ape_and_rpe_stats(tmp_path, monkeypatch):
    module = load_evaluate_m4_module()
    output_dir = tmp_path / "eval"
    vio_path = tmp_path / "vio.tum"
    gt_path = tmp_path / "gt.tum"
    vio_path.write_text("", encoding="utf-8")
    gt_path.write_text("", encoding="utf-8")

    def fake_run(command, check):
        result_path = Path(command[command.index("--save_results") + 1])
        title = (
            "APE w.r.t. translation part (m)\n(with SE(3) Umeyama alignment)"
            if command[0] == "evo_ape"
            else "RPE w.r.t. translation part (m)\n(with SE(3) Umeyama alignment)"
        )
        rmse = 0.5 if command[0] == "evo_ape" else 0.05
        write_evo_zip(result_path, title=title, rmse=rmse)
        return module.subprocess.CompletedProcess(command, 0)

    monkeypatch.setattr(module.subprocess, "run", fake_run)

    summary = module.run_evo(output_dir, vio_path, gt_path)

    assert summary["ape_translation"]["rmse"] == 0.5
    assert summary["rpe_translation_1m"]["rmse"] == 0.05


def test_main_writes_machine_readable_evaluation_summary(tmp_path, monkeypatch):
    module = load_evaluate_m4_module()
    bag_dir = tmp_path / "bag"
    output_dir = tmp_path / "eval"
    bag_dir.mkdir()

    monkeypatch.setattr(
        module,
        "export_tum",
        lambda bag, out: (
            out / "vio_odom.tum",
            out / "vio_gt_path.tum",
            2891,
            2912,
        ),
    )
    monkeypatch.setattr(
        module,
        "run_evo",
        lambda out, vio, gt: {
            "ape_translation": {"rmse": 0.123},
            "rpe_translation_1m": {"rmse": 0.045},
        },
    )

    exit_code = module.main([str(bag_dir), str(output_dir)])

    summary_path = output_dir / "evaluation_summary.json"
    summary = json.loads(summary_path.read_text(encoding="utf-8"))

    assert exit_code == 0
    assert summary["odometry_poses"] == 2891
    assert summary["gt_poses"] == 2912
    assert summary["evo"]["ape_translation"]["rmse"] == 0.123
