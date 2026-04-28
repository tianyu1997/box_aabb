#!/usr/bin/env python3
"""Generate fig_table3_ep_width_profile.{pdf,png}.

Three-panel visual summary accompanying Table III:
    (a) envelope volume, (b) endpoint runtime, (c) max negative gap vs reference.
"""

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RESULTS = (
    ROOT
    / "experiments"
    / "results_iiwa14_final"
    / "s0_ep_width_profile_mc_density_seed100_absneg"
    / "results.json"
)
OUT_DIR = ROOT / "doc"

SOURCES = ["IFK", "CritSample", "Analytical", "MC"]
COLORS = {
    "IFK": "#1f77b4",
    "CritSample": "#2ca02c",
    "Analytical": "#d62728",
    "MC": "#7f7f7f",
}
MARKERS = {
    "IFK": "o",
    "CritSample": "s",
    "Analytical": "^",
    "MC": "x",
}


def main():
    with RESULTS.open("r", encoding="utf-8") as f:
        data = json.load(f)
    rows = data["rows"]
    bins = sorted(
        {(r["width_bin"], float(r["width_lo"]), float(r["width_hi"])) for r in rows},
        key=lambda t: t[1],
    )
    bin_labels = [f"[{lo},{hi}]" for _, lo, hi in bins]
    bin_names = [n for n, _, _ in bins]

    by = {(r["width_bin"], r["endpoint"]): r for r in rows}

    xs = np.arange(len(bins))
    fig, axes = plt.subplots(1, 3, figsize=(13.5, 3.6))

    # (a) volume
    for src in SOURCES:
        ys = [by.get((b, src), {"ep_volume_mean": np.nan})["ep_volume_mean"] for b in bin_names]
        axes[0].plot(xs, ys, marker=MARKERS[src], color=COLORS[src], label=src, linewidth=1.4)
    axes[0].set_yscale("log")
    axes[0].set_ylabel("envelope volume (mean)")
    axes[0].set_title("(a) Envelope volume")
    axes[0].set_xticks(xs)
    axes[0].set_xticklabels(bin_labels, rotation=15)
    axes[0].grid(True, alpha=0.3)

    # (b) runtime
    for src in SOURCES:
        ys = [by.get((b, src), {"ep_time_us_mean": np.nan})["ep_time_us_mean"] for b in bin_names]
        axes[1].plot(xs, ys, marker=MARKERS[src], color=COLORS[src], label=src, linewidth=1.4)
    axes[1].set_yscale("log")
    axes[1].set_ylabel("endpoint runtime ($\\mu$s)")
    axes[1].set_title("(b) Endpoint runtime")
    axes[1].set_xticks(xs)
    axes[1].set_xticklabels(bin_labels, rotation=15)
    axes[1].grid(True, which="both", alpha=0.3)
    axes[1].legend(fontsize=8, loc="best", ncol=2)

    # (c) max negative gap
    for src in SOURCES:
        ys = [by.get((b, src), {"max_negative_gap_abs": np.nan})["max_negative_gap_abs"] for b in bin_names]
        axes[2].plot(xs, ys, marker=MARKERS[src], color=COLORS[src], label=src, linewidth=1.4)
    axes[2].set_ylabel("max neg gap vs reference (rad)")
    axes[2].set_title("(c) Max negative gap")
    axes[2].set_xticks(xs)
    axes[2].set_xticklabels(bin_labels, rotation=15)
    axes[2].grid(True, alpha=0.3)

    for ax in axes:
        ax.set_xlabel("interval-width bin (rad)")

    plt.tight_layout()
    pdf = OUT_DIR / "fig_table3_ep_width_profile.pdf"
    png = OUT_DIR / "fig_table3_ep_width_profile.png"
    fig.savefig(pdf)
    fig.savefig(png, dpi=200)
    print(f"Saved {pdf}")
    print(f"Saved {png}")


if __name__ == "__main__":
    main()
