#!/usr/bin/env python3
"""
viz_exp2_stats.py — Exp2 结果统计可视化

Usage:
    python viz/viz_exp2_stats.py result/<dir>/paths.json
"""

import argparse
import json
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np


def load_data(json_path):
    with open(json_path) as f:
        data = json.load(f)
    return data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("json_path", help="paths.json from exp2")
    parser.add_argument("--log", default=None, help="exp2 stderr log for build times")
    parser.add_argument("--save", default=None, help="output PNG path")
    args = parser.parse_args()

    data = load_data(args.json_path)
    paths = data["paths"]
    queries = data["queries"]
    n_pairs = len(queries)

    # Group by seed
    seeds = sorted(set(p["seed"] for p in paths))
    n_seeds = len(seeds)

    # Per-pair metrics
    pair_labels = [q["label"] for q in queries]
    pair_lengths = {i: [] for i in range(n_pairs)}
    pair_success = {i: 0 for i in range(n_pairs)}

    for p in paths:
        pi = p["pair_idx"]
        if p["success"]:
            pair_lengths[pi].append(p["path_length"])
            pair_success[pi] += 1

    # --- Parse build times from log if available ---
    build_times = []
    seed_islands = []
    seed_boxes = []

    if args.log and os.path.exists(args.log):
        with open(args.log) as f:
            for line in f:
                if "build=" in line and "seed=" in line:
                    # e.g. "  seed=0  build=9.91s  boxes=6096"
                    parts = line.strip().split()
                    for part in parts:
                        if part.startswith("build="):
                            build_times.append(float(part.replace("build=", "").replace("s", "")))
                        if part.startswith("boxes="):
                            seed_boxes.append(int(part.replace("boxes=", "")))
                if "coverage:" in line and "islands=" in line:
                    parts = line.strip().split()
                    for part in parts:
                        if part.startswith("islands="):
                            seed_islands.append(int(part.replace("islands=", "")))

    # --- Create figure ---
    fig = plt.figure(figsize=(18, 12), facecolor='white')
    fig.suptitle(f'Exp2 E2E Planning Results  ({n_seeds} seeds, {n_pairs} pairs)',
                 fontsize=16, fontweight='bold', y=0.98)

    gs = gridspec.GridSpec(2, 3, hspace=0.35, wspace=0.3,
                           left=0.06, right=0.97, top=0.92, bottom=0.06)

    colors = plt.cm.Set2(np.linspace(0, 1, n_pairs))

    # --- Panel 1: Path Length per pair ---
    ax1 = fig.add_subplot(gs[0, 0])
    bp_data = [pair_lengths[i] for i in range(n_pairs)]
    bp = ax1.boxplot(bp_data, labels=pair_labels, patch_artist=True, widths=0.6)
    for patch, color in zip(bp['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.7)
    ax1.set_ylabel('Path Length (rad)')
    ax1.set_title('Path Length Distribution')
    ax1.grid(True, alpha=0.3)
    # Add median values
    for i, med in enumerate(bp['medians']):
        y = med.get_ydata()[0]
        ax1.annotate(f'{y:.2f}', xy=(i + 1, y), xytext=(0, 8),
                     textcoords='offset points', ha='center', fontsize=9, color='red')

    # --- Panel 2: Success Rate ---
    ax2 = fig.add_subplot(gs[0, 1])
    sr = [100 * pair_success[i] / n_seeds for i in range(n_pairs)]
    bars = ax2.bar(pair_labels, sr, color=colors, alpha=0.8, edgecolor='black', linewidth=0.5)
    ax2.set_ylabel('Success Rate (%)')
    ax2.set_title('Success Rate per Query Pair')
    ax2.set_ylim(0, 110)
    ax2.axhline(y=100, color='green', linestyle='--', alpha=0.5)
    ax2.grid(True, alpha=0.3, axis='y')
    for bar, val in zip(bars, sr):
        ax2.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 2,
                 f'{val:.0f}%', ha='center', fontsize=10, fontweight='bold')

    # --- Panel 3: Build Time per seed ---
    ax3 = fig.add_subplot(gs[0, 2])
    if build_times:
        ax3.bar(range(len(build_times)), build_times, color='steelblue', alpha=0.8,
                edgecolor='black', linewidth=0.5)
        ax3.set_xlabel('Seed')
        ax3.set_ylabel('Build Time (s)')
        ax3.set_title(f'Build Time (med={np.median(build_times):.2f}s)')
        ax3.set_xticks(range(len(build_times)))
        ax3.grid(True, alpha=0.3, axis='y')
        for i, t in enumerate(build_times):
            ax3.text(i, t + 0.1, f'{t:.1f}s', ha='center', fontsize=9)
    else:
        ax3.text(0.5, 0.5, 'No log data\n(use --log)', transform=ax3.transAxes,
                 ha='center', va='center', fontsize=14, color='gray')
        ax3.set_title('Build Time')

    # --- Panel 4: Islands per seed ---
    ax4 = fig.add_subplot(gs[1, 0])
    if seed_islands:
        ax4.bar(range(len(seed_islands)), seed_islands, color='coral', alpha=0.8,
                edgecolor='black', linewidth=0.5)
        ax4.set_xlabel('Seed')
        ax4.set_ylabel('# Islands')
        ax4.set_title(f'Island Count (med={int(np.median(seed_islands))})')
        ax4.set_xticks(range(len(seed_islands)))
        ax4.grid(True, alpha=0.3, axis='y')
        for i, n in enumerate(seed_islands):
            ax4.text(i, n + 1, str(n), ha='center', fontsize=9)
    else:
        ax4.text(0.5, 0.5, 'No log data', transform=ax4.transAxes,
                 ha='center', va='center', fontsize=14, color='gray')
        ax4.set_title('Island Count')

    # --- Panel 5: Boxes per seed ---
    ax5 = fig.add_subplot(gs[1, 1])
    if seed_boxes:
        ax5.bar(range(len(seed_boxes)), seed_boxes, color='mediumpurple', alpha=0.8,
                edgecolor='black', linewidth=0.5)
        ax5.set_xlabel('Seed')
        ax5.set_ylabel('# Final Boxes')
        ax5.set_title(f'Final Box Count (med={int(np.median(seed_boxes))})')
        ax5.set_xticks(range(len(seed_boxes)))
        ax5.grid(True, alpha=0.3, axis='y')
        for i, n in enumerate(seed_boxes):
            ax5.text(i, n + 50, str(n), ha='center', fontsize=9)
    else:
        ax5.text(0.5, 0.5, 'No log data', transform=ax5.transAxes,
                 ha='center', va='center', fontsize=14, color='gray')
        ax5.set_title('Final Box Count')

    # --- Panel 6: Per-seed path length heatmap ---
    ax6 = fig.add_subplot(gs[1, 2])
    # Build matrix: rows=pairs, cols=seeds
    mat = np.full((n_pairs, n_seeds), np.nan)
    for p in paths:
        if p["success"]:
            mat[p["pair_idx"], seeds.index(p["seed"])] = p["path_length"]

    im = ax6.imshow(mat, aspect='auto', cmap='YlOrRd', interpolation='nearest')
    ax6.set_xticks(range(n_seeds))
    ax6.set_xticklabels([f's{s}' for s in seeds])
    ax6.set_yticks(range(n_pairs))
    ax6.set_yticklabels(pair_labels)
    ax6.set_xlabel('Seed')
    ax6.set_title('Path Length Heatmap')
    fig.colorbar(im, ax=ax6, shrink=0.8)
    # Annotate
    for i in range(n_pairs):
        for j in range(n_seeds):
            v = mat[i, j]
            if not np.isnan(v):
                ax6.text(j, i, f'{v:.1f}', ha='center', va='center', fontsize=8,
                         color='white' if v > np.nanmedian(mat) else 'black')

    # --- Save ---
    out_dir = os.path.dirname(args.json_path)
    save_path = args.save or os.path.join(out_dir, 'stats.png')
    fig.savefig(save_path, dpi=150, bbox_inches='tight')
    print(f"Saved: {save_path}")
    plt.close()


if __name__ == "__main__":
    main()
