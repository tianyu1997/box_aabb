"""
viz_aabb_comparison.py — Post-hoc visualization for exp_aabb_sampling results

Reads ``results.json`` produced by ``exp_aabb_sampling.py`` and generates
additional analysis plots:

  1. Radar chart:  tightness × speed × safety per method
  2. Box-size scaling curve: how does each method's quality degrade with
     larger intervals?
  3. Per-link bar chart: side-by-side AABB widths (method vs MC)

Usage:
  python experiments/viz_aabb_comparison.py results_aabb_sampling/results.json
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Dict, List

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import FancyBboxPatch
except ImportError:
    print("matplotlib is required for this script.")
    sys.exit(1)


COLORS = {
    "critical": "#2196F3",
    "random":   "#FF9800",
    "interval": "#4CAF50",
}


def load_results(path: str) -> List[dict]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


# ════════════════════════════════════════════════════════════════════════
#  1. Radar chart: tightness / speed / safety
# ════════════════════════════════════════════════════════════════════════

def radar_chart(data: List[dict], save_dir: str):
    """One radar chart per (robot, box) grouping."""

    groups: Dict[tuple, List[dict]] = {}
    for d in data:
        groups.setdefault((d["robot"], d["box"]), []).append(d)

    for (robot, box), entries in groups.items():
        methods = [e["method"] for e in entries]
        # Axes: tightness (1/ratio, higher=better), speed (1/time), safety (0 or 1)
        max_time = max(e["time_ms"] for e in entries) + 1e-6

        tightness = [1.0 / max(e["total_volume_ratio"], 1e-6) for e in entries]
        speed = [1.0 - e["time_ms"] / max_time for e in entries]
        safety = [1.0 if e["all_safe"] else 0.0 for e in entries]

        # Normalize tightness to [0,1]
        max_t = max(tightness) + 1e-9
        tightness = [t / max_t for t in tightness]

        categories = ["Tightness", "Speed", "Safety"]
        N = len(categories)
        angles = [n / float(N) * 2 * np.pi for n in range(N)]
        angles += angles[:1]  # close

        fig, ax = plt.subplots(figsize=(6, 6), subplot_kw=dict(polar=True))
        for i, m in enumerate(methods):
            vals = [tightness[i], speed[i], safety[i]]
            vals += vals[:1]
            ax.plot(angles, vals, "o-", linewidth=2, color=COLORS.get(m, "gray"),
                    label=m)
            ax.fill(angles, vals, alpha=0.1, color=COLORS.get(m, "gray"))

        ax.set_xticks(angles[:-1])
        ax.set_xticklabels(categories, fontsize=11)
        ax.set_ylim(0, 1.05)
        ax.set_title(f"{robot} | box={box}", fontsize=13, pad=20)
        ax.legend(loc="upper right", bbox_to_anchor=(1.3, 1.1))
        plt.tight_layout()

        fpath = os.path.join(save_dir, f"radar_{robot.lower()}_{box}.png")
        plt.savefig(fpath, dpi=150)
        print(f"  Radar chart saved: {fpath}")
        plt.close(fig)


# ════════════════════════════════════════════════════════════════════════
#  2. Scaling curve: volume ratio vs box size
# ════════════════════════════════════════════════════════════════════════

def scaling_curve(data: List[dict], save_dir: str):
    """For each robot, plot volume_ratio vs box_name for each method."""

    robots: Dict[str, Dict[str, Dict[str, dict]]] = {}
    for d in data:
        robots.setdefault(d["robot"], {}).setdefault(d["box"], {})[d["method"]] = d

    BOX_ORDER = ["small", "medium", "large", "full"]

    for robot, box_data in robots.items():
        ordered_boxes = [b for b in BOX_ORDER if b in box_data]
        methods = sorted({m for bd in box_data.values() for m in bd})

        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))
        fig.suptitle(f"{robot}: Quality vs Box Size", fontsize=14)

        x = np.arange(len(ordered_boxes))

        for method in methods:
            ratios = []
            times = []
            for bn in ordered_boxes:
                entry = box_data[bn].get(method)
                if entry:
                    ratios.append(entry["total_volume_ratio"])
                    times.append(entry["time_ms"])
                else:
                    ratios.append(np.nan)
                    times.append(np.nan)

            ax1.plot(x, ratios, "o-", color=COLORS.get(method, "gray"),
                     label=method, linewidth=2, markersize=6)
            ax2.plot(x, times, "s-", color=COLORS.get(method, "gray"),
                     label=method, linewidth=2, markersize=6)

        ax1.axhline(y=1.0, color="red", ls="--", lw=0.8)
        ax1.set_xticks(x)
        ax1.set_xticklabels(ordered_boxes)
        ax1.set_ylabel("Volume Ratio (method / MC truth)")
        ax1.set_title("Tightness")
        ax1.legend()

        ax2.set_xticks(x)
        ax2.set_xticklabels(ordered_boxes)
        ax2.set_ylabel("Time (ms)")
        ax2.set_title("Computation Time")
        ax2.set_yscale("log")
        ax2.legend()

        plt.tight_layout()
        fpath = os.path.join(save_dir, f"scaling_{robot.lower()}.png")
        plt.savefig(fpath, dpi=150)
        print(f"  Scaling curve saved: {fpath}")
        plt.close(fig)


# ════════════════════════════════════════════════════════════════════════
#  3. Per-link AABB width comparison
# ════════════════════════════════════════════════════════════════════════

def perlink_width(data: List[dict], save_dir: str):
    """Bar chart: per-link AABB width for each axis, grouped by method."""

    groups: Dict[tuple, List[dict]] = {}
    for d in data:
        groups.setdefault((d["robot"], d["box"]), []).append(d)

    for (robot, box), entries in groups.items():
        if not entries or not entries[0].get("links"):
            continue

        n_links = len(entries[0]["links"])
        methods = [e["method"] for e in entries]
        axes_names = ["x", "y", "z"]

        fig, axs = plt.subplots(3, 1, figsize=(max(10, n_links * 1.2), 10),
                                sharex=True)
        fig.suptitle(f"{robot} | box={box} — Per-link AABB width", fontsize=13)

        for ai, aname in enumerate(axes_names):
            ax = axs[ai]
            x = np.arange(n_links)
            w = 0.8 / (len(methods) + 1)

            # MC truth
            mc_widths = []
            for li in range(n_links):
                lr = entries[0]["links"][li]
                mc_widths.append(lr["mc_max"][ai] - lr["mc_min"][ai])
            ax.bar(x - 0.4 + w / 2, mc_widths, w, label="MC truth",
                   color="lightgray", edgecolor="gray")

            for mi, method in enumerate(methods):
                entry = entries[mi]
                widths = []
                for li in range(n_links):
                    lr = entry["links"][li]
                    widths.append(lr["method_max"][ai] - lr["method_min"][ai])
                ax.bar(x - 0.4 + (mi + 1.5) * w, widths, w,
                       label=method, color=COLORS.get(method, "gray"),
                       alpha=0.85)

            ax.set_ylabel(f"{aname}-width (m)")
            if ai == 0:
                ax.legend(fontsize=8, ncol=5)

        link_labels = [f"L{entries[0]['links'][i]['link_idx']}"
                       for i in range(n_links)]
        axs[-1].set_xticks(x)
        axs[-1].set_xticklabels(link_labels, fontsize=9)
        axs[-1].set_xlabel("Link")

        plt.tight_layout()
        fpath = os.path.join(save_dir, f"perlink_width_{robot.lower()}_{box}.png")
        plt.savefig(fpath, dpi=150)
        print(f"  Per-link width plot saved: {fpath}")
        plt.close(fig)


# ════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Visualize AABB comparison results")
    parser.add_argument("results", type=str,
                        help="Path to results.json from exp_aabb_sampling.py")
    parser.add_argument("--save-dir", type=str, default=None,
                        help="Output directory (default: same as results.json)")
    args = parser.parse_args()

    data = load_results(args.results)
    save_dir = args.save_dir or os.path.dirname(args.results) or "."
    os.makedirs(save_dir, exist_ok=True)

    print(f"Loaded {len(data)} trial results")
    print(f"Output directory: {save_dir}\n")

    print("Radar charts ...")
    radar_chart(data, save_dir)

    print("\nScaling curves ...")
    scaling_curve(data, save_dir)

    print("\nPer-link width charts ...")
    perlink_width(data, save_dir)

    print("\nDone.")


if __name__ == "__main__":
    main()
