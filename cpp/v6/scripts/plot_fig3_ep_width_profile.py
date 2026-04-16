#!/usr/bin/env python3
"""Fig 3 endpoint-source width profile figures.

Outputs:
  - fig3_ep_width_sources.{pdf,png}
  - fig3_ep_width_gap.{pdf,png}
"""

import json
from pathlib import Path
import os
import sys

sys.path.insert(0, os.path.dirname(__file__))
from plot_common import *


def _load_json(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _resolve_path(root: Path) -> Path:
    candidates = [
        root / "experiments" / "results_iiwa14_final" / "s0_ep_width_profile" / "results.json",
        root / "experiments" / "results" / "s0_ep_width_profile" / "results.json",
    ]
    for p in candidates:
        if p.exists():
            return p
    raise FileNotFoundError("No s0_ep_width_profile/results.json found")


def _parse(data):
    rows = data["rows"]
    widths = sorted({(r["width_bin"], float(r["width_lo"]), float(r["width_hi"])) for r in rows}, key=lambda x: x[1])
    sources = ["IFK", "CritSample", "Analytical", "GCPC"]
    by = {(r["width_bin"], r["endpoint"]): r for r in rows}
    return widths, sources, by


def _extent_gap_norm(r):
    v = np.array(r["extent_gap_vs_analytical_mean"], dtype=float)
    return float(np.linalg.norm(v))


def plot_sources(widths, sources, by):
    x = np.arange(len(widths))
    labels = [f"[{lo:.1f},{hi:.1f}]" for _, lo, hi in widths]

    fig, axes = plt.subplots(1, 2, figsize=(DOUBLE_COL, 2.5), sharex=True)

    for i, s in enumerate(sources):
        ys_v = []
        ys_t = []
        for name, _, _ in widths:
            r = by.get((name, s))
            ys_v.append(float(r["ep_volume_mean"]) if r else np.nan)
            ys_t.append(float(r["ep_time_us_mean"]) if r else np.nan)

        axes[0].plot(x, ys_v, marker="o", color=PAL[i], label=s)
        axes[1].plot(x, ys_t, marker="s", color=PAL[i], label=s)

    axes[0].set_title("Endpoint IAABB volume")
    axes[0].set_ylabel("Volume (m^3)")
    axes[1].set_title("Endpoint runtime")
    axes[1].set_ylabel("Time (us)")

    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.set_xlabel("Interval-width bin (rad)")

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, framealpha=0.9, bbox_to_anchor=(0.5, 1.08))
    fig.suptitle("Endpoint-source profile by interval width (IIWA14)", fontsize=9, fontweight="bold", y=1.12)
    fig.tight_layout()
    savefig(fig, "fig3_ep_width_sources")


def plot_gap(widths, sources, by):
    x = np.arange(len(widths))
    labels = [f"[{lo:.1f},{hi:.1f}]" for _, lo, hi in widths]

    fig, axes = plt.subplots(1, 2, figsize=(DOUBLE_COL, 2.5), sharex=True)

    for i, s in enumerate(sources):
        ys_vg = []
        ys_eg = []
        for name, _, _ in widths:
            r = by.get((name, s))
            ys_vg.append(float(r["vol_gap_vs_analytical_mean"]) if r else np.nan)
            ys_eg.append(_extent_gap_norm(r) if r else np.nan)

        axes[0].plot(x, ys_vg, marker="o", color=PAL[i], label=s)
        axes[1].plot(x, ys_eg, marker="^", color=PAL[i], label=s)

    axes[0].axhline(0.0, color="black", linewidth=0.7)
    axes[0].set_title("Volume gap vs Analytical")
    axes[0].set_ylabel("Delta volume (m^3)")
    axes[1].set_title("Extent-gap norm vs Analytical")
    axes[1].set_ylabel("L2 norm (m)")

    for ax in axes:
        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.set_xlabel("Interval-width bin (rad)")

    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=4, framealpha=0.9, bbox_to_anchor=(0.5, 1.08))
    fig.suptitle("Endpoint-source gap profile by interval width (IIWA14)", fontsize=9, fontweight="bold", y=1.12)
    fig.tight_layout()
    savefig(fig, "fig3_ep_width_gap")


def main():
    setup_ieee_style()
    root = Path(__file__).resolve().parent.parent
    path = _resolve_path(root)
    data = _load_json(path)
    widths, sources, by = _parse(data)
    plot_sources(widths, sources, by)
    plot_gap(widths, sources, by)
    print("Done: EP width profile figures")


if __name__ == "__main__":
    main()
