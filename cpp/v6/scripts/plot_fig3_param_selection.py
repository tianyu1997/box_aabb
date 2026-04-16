#!/usr/bin/env python3
"""Fig 3 parameter preselection figures (CritSample).

Outputs:
  - fig3_sub_sensitivity.{pdf,png}
  - fig3_grid_sensitivity.{pdf,png}
"""

import json
from pathlib import Path
import sys
import os

sys.path.insert(0, os.path.dirname(__file__))
from plot_common import *


def _load_json(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _resolve_s0_path(root: Path) -> Path:
    candidates = [
        root / "experiments" / "results_iiwa14_final" / "s0_param_selection" / "results.json",
        root / "experiments" / "results" / "s0_param_selection" / "results.json",
    ]
    for c in candidates:
        if c.exists():
            return c
    raise FileNotFoundError("No S0 parameter-selection results found in expected locations")


def _kb(x_bytes: float) -> float:
    return float(x_bytes) / 1024.0


def plot_sub_sensitivity(rows, meta):
    sub_rows = sorted([r for r in rows if r.get("study") == "sub"], key=lambda r: int(r["subdivisions"]))

    subs = [int(r["subdivisions"]) for r in sub_rows]
    vols = [float(r["volume_mean"]) for r in sub_rows]
    times = [float(r["total_us_mean"]) for r in sub_rows]

    fig, ax1 = plt.subplots(1, 1, figsize=(SINGLE_COL, 2.6))
    ax2 = ax1.twinx()

    l1 = ax1.plot(subs, vols, marker="o", color=PAL[0], label="Volume (m^3)")
    l2 = ax2.plot(subs, times, marker="s", color=PAL[3], label="Time (us)")

    ax1.set_xlabel("subdivisions (sub)")
    ax1.set_ylabel("Volume (m^3)")
    ax2.set_ylabel("Time (us)")
    ax1.set_title(f"CritSample + LinkIAABB_Grid (delta={meta['fixed_delta_for_sub_sweep']:.3f} m)")
    ax1.set_xticks(subs)

    lines = l1 + l2
    labels = [x.get_label() for x in lines]
    ax1.legend(lines, labels, loc="best", framealpha=0.9)

    fig.tight_layout()
    savefig(fig, "fig3_sub_sensitivity")


def plot_grid_sensitivity(rows, meta):
    grid_rows = sorted([r for r in rows if r.get("study") == "grid"], key=lambda r: float(r["grid_delta"]))

    deltas = [float(r["grid_delta"]) for r in grid_rows]
    vols = [float(r["volume_mean"]) for r in grid_rows]
    times = [float(r["total_us_mean"]) for r in grid_rows]
    cache_kb = [_kb(float(r.get("cache_payload_bytes_mean", 0.0))) for r in grid_rows]

    fig, axes = plt.subplots(1, 2, figsize=(DOUBLE_COL, 2.4))

    axes[0].plot(deltas, vols, marker="o", color=PAL[0], label="Volume (m^3)")
    ax0b = axes[0].twinx()
    ax0b.plot(deltas, times, marker="s", color=PAL[3], label="Time (us)")
    axes[0].set_xlabel("voxel delta (m)")
    axes[0].set_ylabel("Volume (m^3)")
    ax0b.set_ylabel("Time (us)")
    axes[0].set_title(f"Envelope quality/speed (sub={meta['recommended_subdivisions']})")

    axes[1].plot(deltas, cache_kb, marker="^", color=PAL[2], label="Grid cache payload (KiB)")
    axes[1].set_xlabel("voxel delta (m)")
    axes[1].set_ylabel("Cache payload (KiB)")
    axes[1].set_title("Grid cache footprint")

    for ax in axes:
        ax.set_xticks(deltas)

    handles = []
    labels = []
    for h in axes[0].get_lines() + ax0b.get_lines() + axes[1].get_lines():
        handles.append(h)
        labels.append(h.get_label())
    fig.legend(handles, labels, loc="upper center", ncol=3, framealpha=0.9, bbox_to_anchor=(0.5, 1.08))

    fig.tight_layout()
    savefig(fig, "fig3_grid_sensitivity")


def main():
    setup_ieee_style()
    root = Path(__file__).resolve().parent.parent
    s0_path = _resolve_s0_path(root)
    data = _load_json(s0_path)
    rows = data.get("rows", [])
    meta = data.get("meta", {})

    plot_sub_sensitivity(rows, meta)
    plot_grid_sensitivity(rows, meta)
    print("Done: parameter selection figures")


if __name__ == "__main__":
    main()
