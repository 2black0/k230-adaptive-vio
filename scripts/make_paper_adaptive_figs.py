#!/usr/bin/env python3
"""Figures for the adaptive-feature subsection.

fig_tradeoff.pdf  Accuracy against pipeline P95 for five paced EqVIO policies
                  on EuRoC-MAV, with external methods as context points.
fig_temporal.pdf  Four aligned per-frame signals for EuRoC-MAV MH02 and
                  TUM-VI Room1, as in the manuscript.

Reads only from k230-adaptive-vio/results/. Writes to
scripts/paper/out/figures/fig_tradeoff.pdf and fig_temporal.pdf.
"""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from evo.core import sync
from matplotlib.patches import ConnectionPatch, Rectangle

import paper_metrics as pm

OUT = Path(__file__).resolve().parent / "paper" / "out" / "figures"
TEMPORAL_RUNS = [
    ("EuRoC-MAV MH02", pm.RESULTS / "SC-FBC/EuRoC-MAV/MH_02_easy/primary"),
    ("TUM-VI Room1", pm.RESULTS / "SC-FBC/TUM-VI/room1/primary"),
]

CONFIG_STYLE = {
    "Default EqVIO": {"color": "#1f77b4", "marker": "o"},
    "Fixed-Threshold": {"color": "#d97b16", "marker": "s"},
    "Auto-Hold": {"color": "#9467bd", "marker": "^"},
    "Dual-Loop": {"color": "#7f7f7f", "marker": "D"},
    "SC-FBC": {"color": "#2ca02c", "marker": "*"},
}

# EuRoC-MAV mean per-sequence pipeline P95 (ms) and offline ATE (m).
CONTEXT = pm.CONTEXT

DEADLINE_MS = 50.0
ZOOM_PAD_MS, ZOOM_PAD_LOG = 3.5, 1.14


def base_style():
    plt.rcParams.update({
        "font.family": "serif", "font.size": 9, "axes.titlesize": 10,
        "axes.labelsize": 9, "legend.fontsize": 8, "axes.linewidth": 0.6,
        "xtick.labelsize": 8, "ytick.labelsize": 8,
    })


# inset label offsets: name -> (dx_pt, dy_pt, ha)
INSET_OFF = {
    "SC-FBC": (0, -13, "center"),
    "Dual-Loop": (0, 8, "center"),
    "Fixed-Threshold": (7, 0, "left"),
    "Auto-Hold": (0, -13, "center"),
    "Default EqVIO": (0, 8, "center"),
}
CTX_OFF = {"LEVIO": (6, -1, "left"), "LARVIO": (-6, -1, "right"),
           "OpenVINS": (-6, -1, "right")}


def load_euroc_agg():
    """Raw five-arm run.json means (the published figure predates the
    Auto-Hold P95 table override, so it plots the un-rounded campaign mean)."""
    agg = {}
    for label, locator in pm.ABLATION_CONFIGS:
        feats, ates, p95s = [], [], []
        for seq in pm.EUROC_SEQS:
            cells, path = pm._run_json(label, "EuRoC-MAV", seq, locator)
            feats.append(cells["poses"])
            ates.append(cells["ate"])
            p95s.append(cells["pipe"])
        agg[label] = {"p95": float(np.mean(p95s)), "ate_mean": float(np.mean(ates))}
    return agg


def _plot_configs(ax, agg, size_scale=1.0, label=False):
    for name, style in CONFIG_STYLE.items():
        lat, ate = agg[name]["p95"], agg[name]["ate_mean"]
        big = style["marker"] == "*"
        ax.scatter(lat, ate, s=(140 if big else 65) * size_scale,
                   color=style["color"], marker=style["marker"],
                   edgecolors="black", linewidths=0.5, zorder=4,
                   label=None if label else name)
        if label:
            dx, dy, ha = INSET_OFF[name]
            ax.annotate("Fixed-\nThreshold" if name == "Fixed-Threshold" else name.replace(" ", "\n"), (lat, ate), textcoords="offset points",
                        xytext=(dx, dy), ha=ha, fontsize=6.2, color=style["color"],
                        weight="bold" if big else "normal")


def tradeoff():
    base_style()
    agg = load_euroc_agg()
    fig, ax = plt.subplots(figsize=(3.5, 3.4))

    # Context methods (offline), small grey labels.
    for name, (lat, ate) in CONTEXT.items():
        ax.scatter(lat, ate, s=42, facecolors="none", edgecolors="#777777",
                   linewidths=1.0, zorder=2)
        dx, dy, ha = CTX_OFF[name]
        ax.annotate(name, (lat, ate), textcoords="offset points",
                    xytext=(dx, dy), fontsize=6, color="#666666", ha=ha,
                    va="center")
    _plot_configs(ax, agg)

    ax.set_yscale("log")
    ax.set_xlim(18, 185)
    ax.set_ylim(0.24, 6.0)
    ax.set_xlabel("Processing P95 latency (ms)")
    ax.set_ylabel("ATE (m)")
    ax.grid(True, which="both", ls=":", lw=0.4, alpha=0.6)

    # Zoom inset over the EqVIO cluster.
    lats = [agg[c]["p95"] for c in CONFIG_STYLE]
    ates = [agg[c]["ate_mean"] for c in CONFIG_STYLE]
    axins = ax.inset_axes([0.05, 0.54, 0.45, 0.42])
    _plot_configs(axins, agg, size_scale=0.9, label=True)
    axins.set_xlim(min(lats) - 2.0, max(lats) + 2.5)
    axins.set_ylim(min(ates) - 0.012, max(ates) + 0.012)
    axins.set_xticks([25, 28, 31, 34])
    axins.tick_params(labelsize=6)
    axins.grid(True, ls=":", lw=0.3, alpha=0.5)
    for spine in axins.spines.values():
        spine.set_linewidth(0.8)
    # Zoom box padded around the markers, not the inset's tight data limits.
    x0, x1 = min(lats) - ZOOM_PAD_MS, max(lats) + ZOOM_PAD_MS
    y0, y1 = min(ates) / ZOOM_PAD_LOG, max(ates) * ZOOM_PAD_LOG
    ax.add_patch(Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec="black", lw=0.8, zorder=5))
    # Leaders join matching corners: box top-left/right -> inset bottom-left/right.
    for bx, ix in ((x0, 0.0), (x1, 1.0)):
        ax.add_artist(ConnectionPatch(xyA=(bx, y1), coordsA=ax.transData, xyB=(ix, 0.0),
                                      coordsB=axins.transAxes, color="black", lw=0.7, zorder=1))

    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.20), ncol=5,
              frameon=False, fontsize=6.8, handletextpad=0.1, columnspacing=0.5)
    OUT.mkdir(parents=True, exist_ok=True)
    out = OUT / "fig_tradeoff.pdf"
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out}")


def run_series(folder):
    """Per-frame controller, latency and aligned position error of one paced run."""
    with (folder / "controller.csv").open(newline="") as f:
        ctrl = list(csv.DictReader(f))
    with (folder / "raw/realtime.csv").open(newline="") as f:
        rt = list(csv.DictReader(f))
    assert len(ctrl) == len(rt)
    t0 = float(ctrl[0]["stamp"])
    s = {k: np.array([float(r[c]) for r in ctrl]) for k, c in
         [("n", "accepted"), ("tau", "floor"), ("cap", "current_cap"), ("short", "starving")]}
    s["t"] = np.array([float(r["stamp"]) for r in ctrl]) - t0
    s["L"] = np.array([float(r["finish_s"]) - float(r["arrival_s"]) for r in rt]) * 1e3
    # Same alignment and association as Fig. 4 and Table 4 (SE(3), no scale).
    record = json.loads((folder / "run.json").read_text())
    est, gt = pm.load_xyt(folder / "trajectory.tum"), pm.load_xyt(folder / "groundtruth.tum")
    ei, gi = sync.matching_time_indices(est[:, 0], gt[:, 0],
                                        max_diff=record["accuracy"]["association_tolerance_s"])
    rot, trans = pm.umeyama_no_scale(est[ei, 1:4], gt[gi, 1:4])
    err = np.linalg.norm(est[ei, 1:4] @ rot.T + trans - gt[gi, 1:4], axis=1)
    s["te"], s["err"] = est[ei, 0] - t0, err
    s["ate"] = float(np.sqrt(np.mean(err ** 2)))
    assert abs(s["ate"] - record["accuracy"]["ate_rmse_m"]) < 1e-3, (folder, s["ate"])
    return s


def temporal_2col(out=OUT / "fig_temporal.pdf"):
    """Same four panels for both datasets; rows share their y-axis."""
    base_style()
    data = [(title, run_series(folder)) for title, folder in TEMPORAL_RUNS]
    fig, axes = plt.subplots(4, 2, figsize=(3.5, 4.9), sharey="row", sharex="col")
    lgnd = dict(frameon=False, fontsize=6.3, handlelength=1.3, borderaxespad=0.2,
                labelspacing=0.2, columnspacing=0.7)
    for c, (title, s) in enumerate(data):
        t = s["t"]
        ax = axes[0, c]
        ax.set_title(title, fontsize=8)
        ax.fill_between(t, 0, 0.08, where=s["short"] > 0, transform=ax.get_xaxis_transform(),
                        color="#d62728", alpha=0.6, lw=0, step="mid", label="shortfall")
        ax.plot(t, s["n"], color="#26863b", lw=0.45, alpha=0.75, label=r"accepted $n_k$")
        ax.step(t, s["tau"], where="post", color="#08306b", lw=1.0, label=r"online $\tau_k$")
        ax.axhline(8, color="#d97b16", ls="--", lw=0.8, label=r"fixed $\tau=8$")
        ax = axes[1, c]
        ax.step(t, s["cap"], where="post", color="#171717", lw=0.6)
        ax = axes[2, c]
        ax.plot(t, s["L"], color="#1f77b4", lw=0.5)
        ax.axhline(DEADLINE_MS, color="#d62728", ls="--", lw=0.9, label="50 ms frame interval")
        late = s["L"] > DEADLINE_MS
        ax.plot(t[late], s["L"][late], "o", ms=3, mfc="none", mec="#d62728", mew=0.8)
        ax.text(0.98, 0.95, f"{int(late.sum())} of {len(t)} > 50 ms", transform=ax.transAxes,
                ha="right", va="top", fontsize=6.3, color="#d62728")
        ax = axes[3, c]
        ax.plot(s["te"], s["err"], color="#8c564b", lw=0.6)
        ax.text(0.98, 0.95, f"ATE {s['ate']:.3f} m", transform=ax.transAxes,
                ha="right", va="top", fontsize=6.3)
        ax.set_xlabel("Time (s)")
        print(f"{title}: frames={len(t)} late={int(late.sum())} maxL={s['L'].max():.1f} "
              f"shortfall={100 * s['short'].mean():.1f}% ATE={s['ate']:.4f}")
    for r, (label, lim) in enumerate([("Observations", (0, 13.5)), (r"Cap $M_k$", (0, 55)),
                                      (r"$L_k$ (ms)", (0, 68)), ("Pos. err (m)", (0, None))]):
        axes[r, 0].set_ylabel(label)
        axes[r, 0].set_ylim(*lim)
    for ax in axes.flat:
        ax.grid(True, ls=":", lw=0.4, alpha=0.6)
    handles, labels = axes[0, 0].get_legend_handles_labels()
    h2, l2 = axes[2, 0].get_legend_handles_labels()
    handles, labels = handles + h2, labels + l2
    fig.legend(handles, labels, loc="upper center", ncol=3, bbox_to_anchor=(0.56, 1.0), **lgnd)
    for ax, (_, s) in zip(axes[3], data):  # ticks only within each sequence's duration
        ax.set_xticks(np.arange(0, s["t"][-1] + 1, 50))
        ax.set_xlim(-5, s["t"][-1] + 5)
    fig.subplots_adjust(left=0.15, right=0.975, top=0.885, bottom=0.085, hspace=0.12, wspace=0.14)
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(out)
    plt.close(fig)
    print(f"wrote {out}")


def main():
    tradeoff()
    temporal_2col()


if __name__ == "__main__":
    main()
