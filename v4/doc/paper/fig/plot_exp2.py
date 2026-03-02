#!/usr/bin/env python3
"""Generate Exp 2 bar chart (path length + solver time) for the IROS paper.

Style follows gcs-science-robotics/reproduction/prm_comparison/helpers.py:
  - Grouped bars with error bars, black edge, value labels rotated 90°
  - Two vertically stacked subplots: path length (top) and solver time (bottom)
  - Grid on, legend above top subplot

Data source: tab:exp2 in main.tex (already filled).
"""

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ── Data from main.tex tab:exp2 ──────────────────────────────────────────────
pairs = ["Task 1", "Task 2", "Task 3", "Task 4", "Task 5"]

# SBF-GCS (10 seeds, mean values from table)
sbf_pathlen = np.array([3.134, 2.193, 2.985, 3.744, 2.252])
sbf_time    = np.array([41.8,  20.9,  37.4,  46.5,  39.5])   # ms

# IRIS-GCS (10 repeats, mean values from table)
iris_pathlen = np.array([2.874, 1.917, 2.675, 3.456, 1.908])
iris_time    = np.array([25.4,   6.9,  34.7,  48.5,  32.8])  # ms

# ── Colours (matching notebook style: blue for ours, orange for baseline) ────
c_sbf  = [i / 255 for i in (0,   0,   255, 255)]   # blue
c_iris = [i / 255 for i in (255, 191,   0, 255)]   # amber/orange


def _make_bars(ax, sbf_vals, iris_vals, round_to=2):
    width = 0.3
    ticks = np.arange(1, len(pairs) + 1)

    bar_opts = dict(zorder=3, edgecolor="k", linewidth=0.6)
    text_opts = dict(va="bottom", ha="center", fontsize=7, rotation=90)

    ax.bar(ticks - width / 2, sbf_vals, width,
           label="SBF-GCS (ours)", color=c_sbf, **bar_opts)
    ax.bar(ticks + width / 2, iris_vals, width,
           label="IRIS-NP-GCS",   color=c_iris, **bar_opts)

    offset = max(max(sbf_vals), max(iris_vals)) / 50
    sbf_lbl  = np.round(sbf_vals,  round_to) if round_to else sbf_vals.astype(int)
    iris_lbl = np.round(iris_vals, round_to) if round_to else iris_vals.astype(int)

    for i, x in enumerate(ticks):
        ax.text(x - width / 2, sbf_vals[i] + offset,  sbf_lbl[i],  **text_opts)
        ax.text(x + width / 2, iris_vals[i] + offset, iris_lbl[i], **text_opts)

    ax.set_xticks(ticks)
    ax.set_xticklabels(pairs)
    ax.grid(True, axis="y", alpha=0.3)


# ── Figure ───────────────────────────────────────────────────────────────────
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(4.5, 5.0), sharex=True)

# Top: path length
_make_bars(ax1, sbf_pathlen, iris_pathlen, round_to=2)
ax1.set_ylabel("Path length (rad)")
ax1.set_ylim([0, max(max(sbf_pathlen), max(iris_pathlen)) * 1.35])
ax1.tick_params(labelbottom=False)
ax1.legend(loc="upper center", bbox_to_anchor=(0.5, 1.18),
           ncol=2, fancybox=True, fontsize=8)

# Bottom: solver time
_make_bars(ax2, sbf_time, iris_time, round_to=0)
ax2.set_xlabel("")
ax2.set_ylabel("GCS solver time (ms)")
ax2.set_ylim([0, max(max(sbf_time), max(iris_time)) * 1.45])

plt.subplots_adjust(hspace=0.08)
plt.tight_layout()

out = "/home/tian/桌面/box_aabb/v4/doc/paper/fig/fig_exp2_bars.pdf"
fig.savefig(out, bbox_inches="tight")
print(f"Saved → {out}")
plt.close()
