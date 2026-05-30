#!/usr/bin/env python3
"""Plot a VIO trajectory against ground truth from TUM-format files.

Usage:
    python3 tools/plot_trajectory.py \
        --gt   results/euroc_V1_01_easy/eval/vio_gt_path.tum \
        --vio  results/euroc_V1_01_easy/eval/vio_odom.tum \
        --out  results/euroc_V1_01_easy/plots/trajectory_xy.png \
        --title "EuRoC V1_01_easy - BKU-VIO baseline"

Input format (TUM): one pose per line, whitespace-separated:
    timestamp tx ty tz qx qy qz qw

Notes:
    - This script does NOT do SE(3) alignment; for aligned trajectories,
      use `evo_traj tum --ref <gt> <vio> -a --plot --save_plot <out.pdf>`.
      This script is for the quick "raw vs gt" overlay used in the paper.
    - Produces both .png (raster, for slides) and .pdf (vector, for paper)
      when --out ends in .png; the .pdf sibling is written automatically.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # headless-safe; required inside Docker without DISPLAY
import matplotlib.pyplot as plt
import numpy as np


def load_tum(path: Path) -> np.ndarray:
    """Return Nx3 (x, y, z) from a TUM trajectory file."""
    if not path.exists():
        raise FileNotFoundError(f"TUM file not found: {path}")
    data = np.loadtxt(path, ndmin=2)
    if data.size == 0:
        raise ValueError(f"TUM file is empty: {path}")
    if data.shape[1] < 4:
        raise ValueError(
            f"TUM file {path} has {data.shape[1]} columns; expected >= 8"
        )
    return data[:, 1:4]


def plot_xy(gt_xyz: np.ndarray, vio_xyz: np.ndarray, title: str, out: Path) -> None:
    fig, ax = plt.subplots(figsize=(6.0, 5.5))
    ax.plot(gt_xyz[:, 0], gt_xyz[:, 1], color="#444", linewidth=1.4, label="Ground truth")
    ax.plot(vio_xyz[:, 0], vio_xyz[:, 1], color="#1f77b4", linewidth=1.2, label="BKU-VIO")
    ax.scatter(gt_xyz[0, 0], gt_xyz[0, 1], color="green", marker="o", s=30, zorder=5, label="Start")
    ax.scatter(gt_xyz[-1, 0], gt_xyz[-1, 1], color="red", marker="x", s=40, zorder=5, label="End (GT)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linestyle=":", linewidth=0.5, alpha=0.7)
    ax.legend(loc="best", fontsize=9)
    fig.tight_layout()

    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200)
    pdf_sibling = out.with_suffix(".pdf")
    fig.savefig(pdf_sibling)
    plt.close(fig)
    print(f"Wrote {out}")
    print(f"Wrote {pdf_sibling}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--gt", type=Path, required=True, help="Ground-truth TUM file")
    parser.add_argument("--vio", type=Path, required=True, help="VIO odometry TUM file")
    parser.add_argument("--out", type=Path, required=True, help="Output image path (.png)")
    parser.add_argument("--title", type=str, default="VIO trajectory vs ground truth")
    args = parser.parse_args()

    gt_xyz = load_tum(args.gt)
    vio_xyz = load_tum(args.vio)
    print(f"GT  poses: {len(gt_xyz)} from {args.gt}")
    print(f"VIO poses: {len(vio_xyz)} from {args.vio}")
    plot_xy(gt_xyz, vio_xyz, args.title, args.out)


if __name__ == "__main__":
    main()
