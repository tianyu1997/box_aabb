#!/usr/bin/env python3
"""Generate paper figures from experiment JSON results.

Usage:
    python build_paper_figures.py [--source DIR] [--output DIR]

Outputs PDF figures into doc/figures/:
    fig_threads.pdf       — Strong scaling bar chart (data: threads_iiwa14_far.json)
    fig_pathopt.pdf       — Path-optimisation stages (data: pathopt_iiwa14_far.json)
    fig_marcucci.pdf      — Per-query latency bar chart (data: marcucci.json)
    fig_dof_scaling_build_query.pdf — DoF-scaling build/query/size summary
    fig_2dof_forest.pdf   — 2-DoF box forest visualisation (synthetic, illustrates concept)

System architecture figure (fig_arch.pdf) is rendered from TikZ in the paper directly.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# IEEE-style figure defaults: serif font, ~3.4 inch single column / 7.16 inch double column
plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["DejaVu Serif", "Times New Roman", "Liberation Serif"],
    "font.size": 9,
    "axes.titlesize": 9,
    "axes.labelsize": 9,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "legend.fontsize": 8,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "pdf.fonttype": 42,  # editable text in PDF
})

# IEEE column widths (inches)
COL_W = 3.4
DBL_W = 7.16


def fig_threads(src_dir: Path, out_dir: Path) -> None:
    data = json.loads((src_dir / "threads_iiwa14_far.json").read_text())
    rows = data["results"]
    threads = [r["n_threads"] for r in rows]
    times = [r["avg_total_time_ms"] for r in rows]
    speedup = [r["speedup_vs_1t"] for r in rows]

    fig, ax1 = plt.subplots(figsize=(COL_W, 2.2), constrained_layout=True)
    bars = ax1.bar(
        [str(t) for t in threads], times, color="#4C72B0", width=0.55, label="time"
    )
    ax1.set_xlabel("threads")
    ax1.set_ylabel("avg wall time (ms)", color="#4C72B0")
    ax1.tick_params(axis="y", labelcolor="#4C72B0")
    ax1.grid(axis="y", linestyle=":", alpha=0.5)
    for bar, t in zip(bars, times):
        ax1.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 8,
                 f"{t:.0f}", ha="center", va="bottom", fontsize=7,
                 color="#4C72B0")

    ax2 = ax1.twinx()
    ax2.spines["top"].set_visible(False)
    ax2.plot(range(len(threads)), speedup, "o-", color="#C44E52",
             linewidth=1.5, markersize=4, label="speed-up")
    ax2.plot(range(len(threads)), threads, "k--", linewidth=0.8,
             alpha=0.5, label="ideal")
    ax2.set_ylabel("speed-up vs 1 thread", color="#C44E52")
    ax2.tick_params(axis="y", labelcolor="#C44E52")
    ax2.set_ylim(0, max(threads) + 0.5)

    fig.savefig(out_dir / "fig_threads.pdf")
    plt.close(fig)


def fig_pathopt(src_dir: Path, out_dir: Path) -> None:
    data = json.loads((src_dir / "pathopt_iiwa14_far.json").read_text())
    rows = data["results"]
    combos = [r["combo"] for r in rows]
    lengths = [r["avg_opt_length"] for r in rows]
    ratios = [r["length_ratio"] for r in rows]
    times = [r["avg_opt_time_ms"] for r in rows]

    fig, ax1 = plt.subplots(figsize=(COL_W, 2.3), constrained_layout=True)
    x = np.arange(len(combos))
    bars = ax1.bar(x, lengths, color="#55A868", width=0.55)
    ax1.set_xticks(x)
    ax1.set_xticklabels(combos, rotation=15, ha="right")
    ax1.set_ylabel("avg path length (rad)")
    ax1.grid(axis="y", linestyle=":", alpha=0.5)
    for bar, ratio, t in zip(bars, ratios, times):
        ax1.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 0.02,
                 f"{ratio:.2f}\n({t:.2f} ms)", ha="center", va="bottom",
                 fontsize=7)

    ax1.set_ylim(0, max(lengths) * 1.35)
    ax1.set_title("Path length & optimisation time per stage")
    fig.savefig(out_dir / "fig_pathopt.pdf")
    plt.close(fig)


def fig_marcucci(src_dir: Path, out_dir: Path) -> None:
    data = json.loads((src_dir / "marcucci.json").read_text())
    trials = data["trials"]
    # Aggregate per-query latency across seeds (median)
    queries = trials[0]["queries"]
    pair_names = [f"{q['from']}\u2192{q['to']}" for q in queries]

    times_per_pair = [[] for _ in queries]
    for trial in trials:
        for i, q in enumerate(trial["queries"]):
            times_per_pair[i].append(q["t_s"] * 1000.0)  # to ms

    medians = [float(np.median(ts)) for ts in times_per_pair]
    p25 = [float(np.percentile(ts, 25)) for ts in times_per_pair]
    p75 = [float(np.percentile(ts, 75)) for ts in times_per_pair]
    err_low = [m - lo for m, lo in zip(medians, p25)]
    err_hi = [hi - m for m, hi in zip(medians, p75)]

    fig, ax = plt.subplots(figsize=(COL_W, 2.2), constrained_layout=True)
    x = np.arange(len(pair_names))
    bars = ax.bar(x, medians, yerr=[err_low, err_hi], color="#8172B2",
                  capsize=3, width=0.6, error_kw={"linewidth": 0.8})
    ax.set_xticks(x)
    ax.set_xticklabels(pair_names, rotation=20, ha="right")
    ax.set_ylabel("query latency (ms)")
    ax.grid(axis="y", linestyle=":", alpha=0.5)
    for bar, m in zip(bars, medians):
        ax.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 5,
                f"{m:.0f}", ha="center", va="bottom", fontsize=7)
    ax.set_title(f"Marcucci cabinet, {len(trials)} seeds (median, IQR)")
    fig.savefig(out_dir / "fig_marcucci.pdf")
    plt.close(fig)


def fig_2dof_forest(out_dir: Path) -> None:
    """Synthetic illustration of a 2-DoF box forest covering C-free."""
    rng = np.random.default_rng(7)
    fig, ax = plt.subplots(figsize=(COL_W, COL_W * 0.85),
                           constrained_layout=True)

    # Joint-space domain [-pi, pi] x [-pi, pi]
    pi = np.pi
    ax.set_xlim(-pi, pi)
    ax.set_ylim(-pi, pi)
    ax.set_aspect("equal")
    ax.set_xlabel("$q_1$ (rad)")
    ax.set_ylabel("$q_2$ (rad)")

    # C-obstacle regions (illustrative)
    obstacles = [
        (-2.5, -2.5, 1.4, 1.6),
        (0.5, -1.0, 1.8, 1.4),
        (-1.5, 1.3, 2.6, 1.2),
        (1.8, 1.2, 1.0, 1.5),
    ]
    for ox, oy, ow, oh in obstacles:
        ax.add_patch(plt.Rectangle((ox, oy), ow, oh, facecolor="#E0E0E0",
                                   edgecolor="#888", linewidth=0.6,
                                   hatch="///"))

    # Synthesise non-overlapping safe boxes via simple grid + filtering.
    boxes = []
    for x in np.arange(-pi, pi, 0.6):
        for y in np.arange(-pi, pi, 0.6):
            w = 0.5 + rng.uniform(-0.1, 0.15)
            h = 0.5 + rng.uniform(-0.1, 0.15)
            cx, cy = x + w / 2, y + h / 2
            inside_obs = any(
                ox <= cx <= ox + ow and oy <= cy <= oy + oh
                for ox, oy, ow, oh in obstacles
            )
            if not inside_obs and x + w < pi and y + h < pi:
                boxes.append((x, y, w, h))

    for bx, by, bw, bh in boxes:
        ax.add_patch(plt.Rectangle((bx, by), bw, bh, facecolor="#B5DDF0",
                                   edgecolor="#1F77B4", linewidth=0.4,
                                   alpha=0.85))

    # Start, goal, and a sample path through box centres.
    qs = (-2.6, 2.6)
    qg = (2.6, -2.6)
    ax.plot(*qs, "g*", markersize=10, label="$q_s$")
    ax.plot(*qg, "r*", markersize=10, label="$q_g$")

    path = [qs, (-1.5, 0.5), (-0.2, 0.2), (1.5, -0.5), qg]
    px, py = zip(*path)
    ax.plot(px, py, "-", color="#D62728", linewidth=1.4, alpha=0.9,
            label="optimised path")

    ax.legend(loc="lower left", fontsize=7, framealpha=0.85)
    ax.set_title("2-DoF box forest cover of $\\mathcal{C}_\\mathrm{free}$")

    fig.savefig(out_dir / "fig_2dof_forest.pdf")
    plt.close(fig)


def fig_dof_scaling_build_query(paper_dir: Path, out_dir: Path) -> None:
    path = paper_dir / "dof_scaling.json"
    if not path.exists():
        return
    data = json.loads(path.read_text())
    cells = sorted(
        data.get("cells", []),
        key=lambda cell: (cell.get("scene_family", ""), cell.get("active_dof", 0)),
    )
    if not cells:
        return

    grouped: dict[str, list[dict]] = {}
    for cell in cells:
        family = cell.get("scene_family") or cell.get("scene_name", "")
        grouped.setdefault(family, []).append(cell)

    fig, axes = plt.subplots(1, 3, figsize=(DBL_W, 2.25), constrained_layout=True)
    metrics = [
        ("median_grow_time_ms", "Build med. (ms)"),
        ("median_total_time_ms", "Total med. (ms)"),
        ("median_n_boxes", "Boxes med."),
    ]
    colors = {
        "iiwa14_far": "#4C72B0",
        "iiwa14_narrow": "#C44E52",
    }
    markers = {
        "iiwa14_far": "o",
        "iiwa14_narrow": "s",
    }

    for ax, (metric, ylabel) in zip(axes, metrics):
        for family, family_cells in grouped.items():
            family_cells = sorted(family_cells, key=lambda cell: cell.get("active_dof", 0))
            dof = [cell["active_dof"] for cell in family_cells]
            values = [cell["summary"][metric] for cell in family_cells]
            ax.plot(
                dof,
                values,
                marker=markers.get(family, "o"),
                color=colors.get(family, "#55A868"),
                linewidth=1.6,
                markersize=4,
                label=family.replace("iiwa14_", ""),
            )
        ax.set_xlabel("active DoF")
        ax.set_ylabel(ylabel)
        ax.grid(axis="y", linestyle=":", alpha=0.5)
        ax.set_xticks(sorted({cell["active_dof"] for cell in cells}))

    axes[0].legend(loc="upper left", frameon=False)

    fig.savefig(out_dir / "fig_dof_scaling_build_query.pdf")
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source",
        default="experiments/results_nightly/full",
        help="directory containing *.json results",
    )
    parser.add_argument(
        "--paper-source",
        default="experiments/results_paper",
        help="directory containing paper-only JSON results",
    )
    parser.add_argument(
        "--output",
        default="doc/figures",
        help="directory to write PDF figures",
    )
    args = parser.parse_args()

    src_dir = Path(args.source)
    paper_dir = Path(args.paper_source)
    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    fig_threads(src_dir, out_dir)
    fig_pathopt(src_dir, out_dir)
    fig_marcucci(src_dir, out_dir)
    fig_2dof_forest(out_dir)
    fig_dof_scaling_build_query(paper_dir, out_dir)

    print(f"Figures written to: {out_dir}")
    for p in sorted(out_dir.glob("*.pdf")):
        print(f"  {p.name}  ({p.stat().st_size // 1024} KB)")


if __name__ == "__main__":
    main()
