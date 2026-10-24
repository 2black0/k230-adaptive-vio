#!/usr/bin/env python3
"""k230-levio — plot a TUM trajectory and, when ground truth is given, run evo.

Headless by design: the figure is written to a PNG, no window is opened.

Outputs, into --out-dir:
    trajectory.png      top-down estimated vs ground-truth path
    evo/ate.zip         APE result (evo archive, re-readable by `evo_res`)
    evo/rpe.zip         RPE result
    evo/ate.pdf         APE plot

Usage:
    python3 tools/plot_trajectory.py --est run/trajectory.tum \
        --gt data/MH01/groundtruth.txt --out-dir run

    python3 tools/plot_trajectory.py --est run/trajectory.tum --out-dir run

SPDX-License-Identifier: MIT
"""

import argparse
import os
import shutil
import subprocess
import sys


def read_tum(path):
    """Returns (t, xyz) with xyz an N x 3 list. Skips comments and blanks."""
    ts, xyz = [], []
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 4:
                continue
            try:
                ts.append(float(parts[0]))
                xyz.append((float(parts[1]), float(parts[2]), float(parts[3])))
            except ValueError:
                continue
    return ts, xyz


def plot(est_path, gt_path, out_dir, label):
    try:
        import matplotlib
        matplotlib.use("Agg")           # no display, no window
        import matplotlib.pyplot as plt
    except ImportError:
        print("[plot] matplotlib not installed, skipping trajectory.png", file=sys.stderr)
        return None

    _, est_xyz = read_tum(est_path)
    if not est_xyz:
        print(f"[plot] {est_path} has no pose rows", file=sys.stderr)
        return None

    fig, ax = plt.subplots(figsize=(7, 7))

    if gt_path and os.path.isfile(gt_path):
        _, gt_xyz = read_tum(gt_path)
        if gt_xyz:
            ax.plot([p[0] for p in gt_xyz], [p[2] for p in gt_xyz],
                    color="black", linewidth=1.6, label="ground truth")
        else:
            print(f"[plot] {gt_path} has no pose rows", file=sys.stderr)

    ax.plot([p[0] for p in est_xyz], [p[2] for p in est_xyz],
            color="#1f77b4", linewidth=1.2, label=label)
    ax.scatter([est_xyz[0][0]], [est_xyz[0][2]], color="green", s=30,
               zorder=5, label="start")

    # Equal aspect: a monocular scale error is much easier to read when the
    # axes are not free to stretch independently.
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("z [m]")
    ax.set_title(f"{label} — top-down (x vs z)")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    out = os.path.join(out_dir, "trajectory.png")
    fig.tight_layout()
    fig.savefig(out, dpi=120)
    plt.close(fig)
    return out


def run_evo(est_path, gt_path, out_dir, label):
    """Runs evo_ape/evo_rpe when ground truth and evo are both available."""
    if not gt_path or not os.path.isfile(gt_path):
        print("[evo] no ground truth given, skipping ATE/RPE", file=sys.stderr)
        return {}

    if shutil.which("evo_ape") is None:
        print("[evo] evo_ape not on PATH (pip install evo), skipping", file=sys.stderr)
        return {}

    evo_dir = os.path.join(out_dir, "evo")
    os.makedirs(evo_dir, exist_ok=True)

    # -a : SE(3) alignment; -s : also solve scale. -s is mandatory for a
    # monocular run, whose trajectory is only defined up to scale.
    cmds = {
        "ate": ["evo_ape", "tum", gt_path, est_path, "-a", "-s",
                "--save_results", os.path.join(evo_dir, "ate.zip"),
                "--save_plot", os.path.join(evo_dir, "ate.pdf"),
                "--no_warnings"],
        "rpe": ["evo_rpe", "tum", gt_path, est_path, "-a", "-s",
                "--delta", "1", "--delta_unit", "f",
                "--save_results", os.path.join(evo_dir, "rpe.zip"),
                "--no_warnings"],
    }

    stats = {}
    for name, cmd in cmds.items():
        try:
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
        except (OSError, subprocess.TimeoutExpired) as exc:
            print(f"[evo] {name} failed to run: {exc}", file=sys.stderr)
            continue
        if res.returncode != 0:
            print(f"[evo] {name} exited {res.returncode}\n{res.stderr.strip()}",
                  file=sys.stderr)
            continue
        for line in res.stdout.splitlines():
            parts = line.split()
            if len(parts) == 2 and parts[0] in ("rmse", "mean", "median", "max", "min", "std"):
                stats[f"{name}_{parts[0]}"] = float(parts[1])

    # Machine-readable copy, so the runner and the aggregation scripts do not
    # have to scrape the plotter's stdout.
    if stats:
        with open(os.path.join(evo_dir, "ate.csv"), "w") as f:
            f.write("metric,value\n")
            for key in ("ate_rmse", "ate_mean", "ate_median", "ate_max", "ate_std",
                        "rpe_rmse", "rpe_mean"):
                if key in stats:
                    f.write(f"{key},{stats[key]:.6f}\n")

    return stats


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--est", required=True, help="estimated trajectory, TUM format")
    ap.add_argument("--gt", default=None, help="ground-truth trajectory, TUM format")
    ap.add_argument("--out-dir", required=True, help="directory for the plots")
    ap.add_argument("--label", default="estimated", help="legend label for the estimate")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    png = plot(args.est, args.gt, args.out_dir, args.label)
    if png:
        print(f"[plot] wrote {png}")

    stats = run_evo(args.est, args.gt, args.out_dir, args.label)
    if "ate_rmse" in stats:
        print(f"[evo] ATE RMSE (SE3 + scale): {stats['ate_rmse']:.4f} m")
        print(f"[evo] ATE mean/max          : {stats['ate_mean']:.4f} / {stats['ate_max']:.4f} m")

    return 0


if __name__ == "__main__":
    sys.exit(main())
