#!/usr/bin/env python3
"""Generate one figure per width-bin split sweep file.

Inputs:
  experiments/results_iiwa14_final/s0c_width_sub_grid_split/W*.json

Outputs:
  doc/figures/fig3_width_sweep_W*.{pdf,png}
"""

import json
from pathlib import Path
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from plot_common import *


def _load(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _ordered_width_files(root: Path):
    p = root / "experiments" / "results_iiwa14_final" / "s0c_width_sub_grid_split"
    names = [
        "W1_0.10_0.20.json",
        "W2_0.20_0.30.json",
        "W3_0.30_0.40.json",
        "W4_0.40_0.50.json",
    ]
    files = [p / n for n in names if (p / n).exists()]
    if len(files) != 4:
        missing = [str(p / n) for n in names if not (p / n).exists()]
        raise FileNotFoundError("Missing split files: " + ", ".join(missing))
    return files


def _extract(rows):
    # LinkIAABB curve: volume/time vs sub
    iaabb = sorted(
        [r for r in rows if r["envelope"] == "LinkIAABB"],
        key=lambda r: int(r["subdivisions"]),
    )
    subs = [int(r["subdivisions"]) for r in iaabb]
    iaabb_vol = [float(r["volume_mean"]) for r in iaabb]
    iaabb_time = [float(r["total_us_mean"]) for r in iaabb]

    # LinkIAABB_Grid curves by delta
    grid = [r for r in rows if r["envelope"] == "LinkIAABB_Grid"]
    deltas = sorted({float(r["grid_delta"]) for r in grid})
    grid_vol = {}
    grid_time = {}
    for d in deltas:
        rs = sorted(
            [r for r in grid if abs(float(r["grid_delta"]) - d) < 1e-9],
            key=lambda r: int(r["subdivisions"]),
        )
        grid_vol[d] = [float(r["volume_mean"]) for r in rs]
        grid_time[d] = [float(r["total_us_mean"]) for r in rs]
    return subs, deltas, iaabb_vol, iaabb_time, grid_vol, grid_time


def plot_one(width_name: str, width_lo: float, width_hi: float, rows):
    subs, deltas, iaabb_vol, iaabb_time, grid_vol, grid_time = _extract(rows)

    fig, axes = plt.subplots(1, 2, figsize=(DOUBLE_COL, 2.5), sharex=True)

    axes[0].plot(subs, iaabb_vol, marker="o", color=PAL[0], linewidth=1.4, label="LinkIAABB")
    axes[1].plot(subs, iaabb_time, marker="o", color=PAL[0], linewidth=1.4, label="LinkIAABB")

    for i, d in enumerate(deltas):
        c = PAL[(i + 1) % len(PAL)]
        label = f"Grid, delta={d:.2f}"
        axes[0].plot(subs, grid_vol[d], marker="s", color=c, linewidth=1.1, label=label)
        axes[1].plot(subs, grid_time[d], marker="s", color=c, linewidth=1.1, label=label)

    axes[0].set_title("Envelope volume vs sub")
    axes[0].set_ylabel("Volume (m^3)")
    axes[1].set_title("Runtime vs sub")
    axes[1].set_ylabel("Time (us)")

    for ax in axes:
        ax.set_xlabel("subdivisions (sub)")
        ax.set_xticks(subs)

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=3, framealpha=0.9, bbox_to_anchor=(0.5, 1.10))
    fig.suptitle(f"Width bin [{width_lo:.2f}, {width_hi:.2f}] rad ({width_name})", fontsize=9, fontweight="bold", y=1.15)
    fig.tight_layout()
    savefig(fig, f"fig3_width_sweep_{width_name}")


def main():
    setup_ieee_style()
    root = Path(__file__).resolve().parent.parent
    for fp in _ordered_width_files(root):
        data = _load(fp)
        rows = data["rows"]
        meta = data.get("meta", {})
        bins = meta.get("width_bins", [])
        if bins:
            b = bins[0]
            w_name = b["name"]
            w_lo = float(b["lo"])
            w_hi = float(b["hi"])
        else:
            w_name = fp.stem
            # fallback parse from name
            parts = w_name.split("_")
            w_lo = float(parts[1])
            w_hi = float(parts[2])
        plot_one(w_name, w_lo, w_hi, rows)

    print("Done: 4 split sweep figures")


if __name__ == "__main__":
    main()
