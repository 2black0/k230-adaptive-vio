#!/usr/bin/env python3
"""Render the representative-sequence trajectory figures.

Two figures: EuRoC-MAV (MH05, V101, V203) and TUM-VI (Room1, Room5). Rows are
sequences, columns are methods. Each panel draws the ground truth in grey
underneath one estimate coloured by its point-wise position error after a rigid
SE(3) alignment without scale. Panel ATE values are taken from Table 4
(paper_metrics.accuracy_table) and asserted against a recomputation from the
same trajectories. Writes scripts/paper/out/figures/fig_traj_euroc_sel_k230.pdf
and scripts/paper/out/figures/fig_traj_tumvi_sel_k230.pdf.
"""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from evo.core import sync
from matplotlib.collections import LineCollection
from matplotlib.colors import LogNorm

import paper_metrics as pm

OUT = Path(__file__).resolve().parent / "paper" / "out" / "figures"
EUROC_SEQS = ["MH05", "V101", "V203"]
TUMVI_SEQS = ["room1", "room5"]

METHODS = ["eqvio", "levio", "larvio", "openvins", "ours"]
LABELS = {"eqvio": "Default EqVIO", "levio": "LEVIO", "larvio": "LARVIO",
          "openvins": "OpenVINS", "ours": "SC-FBC"}
TABLE_KEY = {"eqvio": "EqVIO", "levio": "LEVIO", "larvio": "LARVIO",
             "openvins": "OpenVINS", "ours": "Ours"}
NORM = LogNorm(vmin=0.05, vmax=5.0)
CMAP = "viridis"


def seq_paths(seq):
    """(trajectory.tum, groundtruth.tum) per method, from results/ only."""
    if seq.startswith("room"):
        dataset, ddir = "TUM-VI", seq
    else:
        dataset, ddir = "EuRoC-MAV", pm.EUROC_DIR[seq]
    paths = {}
    for key, method in TABLE_KEY.items():
        root, sub = pm.SOURCES[method]
        if method == "OpenVINS" and dataset == "TUM-VI":
            sub = "confirmation"  # Table 4 reads the post-diagnosis TUM-VI run
        base = pm.RESULTS / root / dataset / ddir / sub
        paths[key] = (base / "trajectory.tum", base / "groundtruth.tum")
    return paths


def matched_error(traj, gtp):
    """Return aligned xy, per-point APE, GT xy and ATE RMSE on matched stamps."""
    est, gt = pm.load_xyt(traj), pm.load_xyt(gtp)
    # Paced runs record their evaluation tolerance (10 ms); external runs use TOL.
    metadata = Path(traj).with_name("run.json")
    tolerance = pm.TOL
    if metadata.exists():
        record = pm.json_load(metadata)
        for value in record.values():
            if isinstance(value, dict) and "association_tolerance_s" in value:
                tolerance = value["association_tolerance_s"]
    ei, gi = sync.matching_time_indices(est[:, 0], gt[:, 0], max_diff=tolerance)
    rot, trans = pm.umeyama_no_scale(est[ei, 1:4], gt[gi, 1:4])
    p = est[ei, 1:4] @ rot.T + trans
    err = np.linalg.norm(p - gt[gi, 1:4], axis=1)
    return p[:, :2], err, gt[:, 1:3], float(np.sqrt(np.mean(err ** 2)))


def table_ate(tables, seq, m):
    """ATE as printed in Table 4 (the table dict is keyed by raw seq names)."""
    dataset = "TUM-VI" if seq.startswith("room") else "EuRoC-MAV"
    key = f"R{seq[4:]}" if seq.startswith("room") else seq
    block = tables[dataset][TABLE_KEY[m]]
    if key in block:
        return block[key][0]
    return block[seq][0]


def panel(ax, xy, err, gt_xy, ate):
    ax.plot(gt_xy[:, 0], gt_xy[:, 1], color="#bdbdbd", lw=2.2, zorder=1, solid_capstyle="round")
    seg = np.stack([xy[:-1], xy[1:]], axis=1)
    lc = LineCollection(seg, cmap=CMAP, norm=NORM, lw=1.0, zorder=2)
    lc.set_array(err[1:])
    ax.add_collection(lc)
    lo, hi = gt_xy.min(0), gt_xy.max(0)
    pad = 0.15 * (hi - lo).max()
    ax.set_xlim(lo[0] - pad, hi[0] + pad)
    ax.set_ylim(lo[1] - pad, hi[1] + pad)
    ax.set_aspect("equal", adjustable="datalim")
    out = np.mean((xy < lo - pad).any(1) | (xy > hi + pad).any(1))
    diverged = ate > 100.0
    note = f"ATE {ate:.3f} m" if not diverged else "ATE > 100 m"
    if 0.02 < out <= 0.95:
        note += f"\n{out:.0%} out of view"
    if diverged or out > 0.95:
        ax.text(0.5, 0.45, "diverged" if diverged else f"{out:.0%}\nout of view",
                transform=ax.transAxes, ha="center", va="center",
                fontsize=9, color="#b2182b", style="italic")
    ax.text(0.03, 0.97, note, transform=ax.transAxes, va="top", ha="left", fontsize=7,
            color="#b2182b" if ate > 1.0 else "black",
            zorder=10, bbox=dict(boxstyle="round,pad=0.2", fc="white", ec="none", alpha=1.0))
    ax.set_xticks([]); ax.set_yticks([])
    return lc


def render(tables, seqs, out):
    plt.rcParams.update({"font.family": "serif", "font.size": 8, "axes.titlesize": 9})
    fig, axes = plt.subplots(len(seqs), len(METHODS), figsize=(7.16, 1.44 * len(seqs) + 0.32),
                             squeeze=False)
    fig.subplots_adjust(left=0.07, right=0.90, top=0.90, bottom=0.03, wspace=0.05, hspace=0.06)
    lc = None
    for r, seq in enumerate(seqs):
        paths = seq_paths(seq)
        for c, m in enumerate(METHODS):
            ax = axes[r, c]
            xy, err, gt_xy, ate = matched_error(*paths[m])
            tab = table_ate(tables, seq, m)
            # Panels print the table value; the recomputed one must agree with it.
            ok = abs(ate - tab) <= max(0.002, 1e-4 * tab)
            print(f"{seq:6s} {m:9s} table={tab:<12.4f} recomputed={ate:<12.4f} {'OK' if ok else 'MISMATCH'}")
            assert ok, f"{seq} {m}: figure data does not match Table 4"
            lc = panel(ax, xy, err, gt_xy, tab)
            if r == 0:
                ax.set_title(LABELS[m], fontweight="bold" if m == "ours" else "normal")
        axes[r, 0].set_ylabel(seq.capitalize() if seq.startswith("room") else seq, fontsize=8.5)
    cax = fig.add_axes([0.915, 0.08, 0.015, 0.80])
    fig.colorbar(lc, cax=cax).set_label("Position error (m), log scale")
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    print("wrote", out)


def main():
    tables = pm.accuracy_table()
    render(tables, EUROC_SEQS, OUT / "fig_traj_euroc_sel_k230.pdf")
    render(tables, TUMVI_SEQS, OUT / "fig_traj_tumvi_sel_k230.pdf")


if __name__ == "__main__":
    main()
