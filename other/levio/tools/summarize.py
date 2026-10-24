#!/usr/bin/env python3
"""k230-levio — collect every completed run under results/ into one table.

Reads each run directory the harness and run_bench.sh produce and stitches the
per-run files into a single CSV plus a markdown table per dataset and target:

    results/<target>/<sequence>/summary.csv    fps, latency, cpu, rss
    results/<target>/<sequence>/evo/ate.csv    evo ATE/RPE against ground truth
    results/<target>/<sequence>/run.json       dataset, mode, verdict, binary

Usage:
    python3 tools/summarize.py                       # everything under results/
    python3 tools/summarize.py --target board
    python3 tools/summarize.py --out results/summary.csv

SPDX-License-Identifier: MIT
"""

import argparse
import csv
import json
import os
import statistics
import sys
from pathlib import Path

# Column order of the combined CSV.
FIELDS = [
    "target", "dataset", "sequence", "mode", "verdict",
    "frames", "frames_dropped", "keyframes",
    "throughput_fps", "fps_mean", "fps_p95",
    "latency_mean_ms", "latency_p95_ms",
    "cpu_pct_mean", "rss_peak_mb", "wall_time_s",
    "ate_rmse", "ate_mean", "ate_max", "rpe_rmse",
    "binary_sha256",
]

# Rendered in the markdown table; the rest stay in the CSV.
TABLE_FIELDS = [
    "sequence", "frames", "keyframes", "throughput_fps",
    "latency_mean_ms", "latency_p95_ms", "cpu_pct_mean", "rss_peak_mb", "ate_rmse",
]


def read_kv_csv(path):
    """Reads a two-column metric,value file, ignoring '#' comment lines."""
    out = {}
    if not Path(path).is_file():
        return out
    with open(path, newline="") as f:
        for row in csv.reader(f):
            if len(row) >= 2 and not row[0].startswith("#"):
                out[row[0].strip()] = row[1].strip()
    return out


def as_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def fmt(value, digits=3):
    return f"{value:.{digits}f}" if isinstance(value, float) else ""


def dataset_of(run_dir, run_json):
    """run.json is authoritative; fall back to the sequence-name convention."""
    dataset = (run_json or {}).get("dataset")
    if dataset == "euroc-mav" or dataset == "euroc":
        return "euroc"
    if dataset == "tum-vi" or dataset == "tumvi":
        return "tumvi"
    name = run_dir.name
    if name.startswith("room"):
        return "tumvi"
    if name.startswith("MH") or (name[:1] == "V" and name[1:2].isdigit()):
        return "euroc"
    return "unknown"


def collect(results_root, target_filter):
    rows = []
    if not results_root.is_dir():
        return rows

    for target_dir in sorted(p for p in results_root.iterdir() if p.is_dir()):
        target = target_dir.name
        if target_filter and target != target_filter:
            continue
        for run_dir in sorted(p for p in target_dir.iterdir() if p.is_dir()):
            metrics = read_kv_csv(run_dir / "summary.csv")
            if not metrics:
                continue                      # not a finished run

            evo = read_kv_csv(run_dir / "evo" / "ate.csv")
            run_json = None
            if (run_dir / "run.json").is_file():
                try:
                    with open(run_dir / "run.json") as f:
                        run_json = json.load(f)
                except (OSError, ValueError):
                    run_json = None

            latency_mean = as_float(metrics.get("latency_mean_us"))
            latency_p95 = as_float(metrics.get("latency_p95_us"))

            rows.append({
                "target": target,
                "dataset": dataset_of(run_dir, run_json),
                "sequence": run_dir.name,
                "mode": metrics.get("mode", (run_json or {}).get("mode", "")),
                "verdict": (run_json or {}).get("verdict", ""),
                "frames": as_float(metrics.get("frames")),
                "frames_dropped": as_float(metrics.get("frames_dropped")),
                "keyframes": as_float(metrics.get("keyframes")),
                "throughput_fps": as_float(metrics.get("throughput_fps")),
                "fps_mean": as_float(metrics.get("fps_mean")),
                "fps_p95": as_float(metrics.get("fps_p95")),
                "latency_mean_ms": latency_mean / 1e3 if latency_mean is not None else None,
                "latency_p95_ms": latency_p95 / 1e3 if latency_p95 is not None else None,
                "cpu_pct_mean": as_float(metrics.get("cpu_pct_mean")),
                "rss_peak_mb": as_float(metrics.get("rss_peak_mb")),
                "wall_time_s": as_float(metrics.get("wall_time_s")),
                "ate_rmse": as_float(evo.get("ate_rmse")),
                "ate_mean": as_float(evo.get("ate_mean")),
                "ate_max": as_float(evo.get("ate_max")),
                "rpe_rmse": as_float(evo.get("rpe_rmse")),
                "binary_sha256": (run_json or {}).get("binary_sha256", ""),
            })
    return rows


def print_table(rows, dataset, target, budget_ms=50.0):
    subset = [r for r in rows if r["dataset"] == dataset and r["target"] == target]
    if not subset:
        return

    label = {"euroc": "EuRoC MAV", "tumvi": "TUM-VI"}.get(dataset, dataset)
    print(f"\n### {label} — {target}\n")
    print("| " + " | ".join(TABLE_FIELDS) + " |")
    print("|" + "---|" * len(TABLE_FIELDS))

    for r in sorted(subset, key=lambda r: (r["dataset"], r["sequence"])):
        cells = []
        for f in TABLE_FIELDS:
            v = r[f]
            if isinstance(v, str):
                cells.append(v)
            elif f in ("frames", "keyframes"):
                cells.append(f"{v:.0f}" if isinstance(v, float) else "")
            elif f == "ate_rmse":
                cells.append(f"{v:.3f}" if isinstance(v, float) else "—")
            else:
                cells.append(fmt(v, 2 if f in ("throughput_fps", "cpu_pct_mean") else 1))
        print("| " + " | ".join(cells) + " |")

    ates = [r["ate_rmse"] for r in subset if r["ate_rmse"] is not None]
    thrs = [r["throughput_fps"] for r in subset if r["throughput_fps"] is not None]
    lats = [r["latency_p95_ms"] for r in subset if r["latency_p95_ms"] is not None]
    kfs = [r["keyframes"] / r["frames"] * 100 for r in subset
           if r["keyframes"] is not None and r["frames"]]

    print()
    if ates:
        print(f"- ATE RMSE: mean {statistics.mean(ates):.3f} m, median "
              f"{statistics.median(ates):.3f} m, min {min(ates):.3f}, max {max(ates):.3f}")
    if thrs:
        print(f"- Throughput: mean {statistics.mean(thrs):.1f} fps, min {min(thrs):.1f}")
    if lats:
        # The realtime question is about the tail, not the mean.
        over = sum(1 for v in lats if v > budget_ms)
        print(f"- p95 latency > {budget_ms:.0f} ms on {over}/{len(lats)} sequences")
    if kfs:
        print(f"- Keyframes: {statistics.mean(kfs):.0f}% of frames on average")

    missing = [r["sequence"] for r in subset if r["ate_rmse"] is None]
    if missing:
        print(f"- No ATE (missing/failed evo): {', '.join(missing)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--results", default="results", help="results root (default: results)")
    ap.add_argument("--target", default="", help="host | board (default: all)")
    ap.add_argument("--out", default="results/summary.csv", help="combined CSV path")
    ap.add_argument("--budget-ms", type=float, default=50.0,
                    help="per-frame budget for the deadline-miss count (default 50)")
    args = ap.parse_args()

    rows = collect(Path(args.results), args.target)
    if not rows:
        print("no completed runs found under %s" % args.results, file=sys.stderr)
        return 1

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS, extrasaction="ignore")
        w.writeheader()
        for r in sorted(rows, key=lambda r: (r["dataset"], r["target"], r["sequence"])):
            w.writerow({k: ("" if r[k] is None else r[k]) for k in FIELDS})

    for dataset in ("euroc", "tumvi"):
        for target in ("host", "board"):
            print_table(rows, dataset, target, args.budget_ms)

    print(f"\ncombined CSV: {out}  ({len(rows)} runs)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
