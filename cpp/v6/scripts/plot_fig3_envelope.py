#!/usr/bin/env python3
"""Fig 3: IIWA14 envelope tightness and signed gap figures.

Outputs:
  - fig3_envelope_tightness.{pdf,png}
  - fig3_signed_gap.{pdf,png}
"""

import json
import os
import sys
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))
from plot_common import *


def _load_json(path: Path):
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _fmt_time_us(t_us: float) -> str:
    if t_us >= 100.0:
        return f"{t_us:.0f}"
    if t_us >= 10.0:
        return f"{t_us:.1f}"
    return f"{t_us:.2f}"


def plot_envelope_grouped(rows):
    endpoints = ["IFK", "CritSample", "Analytical", "GCPC", "MC"]
    sub_groups = [
        (1, ["LinkIAABB", "LinkIAABB_Grid", "Hull16_Grid"]),
        (4, ["LinkIAABB", "LinkIAABB_Grid"]),
        (8, ["LinkIAABB", "LinkIAABB_Grid"]),
    ]
    env_label = {
        "LinkIAABB": "LinkIAABB",
        "LinkIAABB_Grid": "LinkIAABB+Grid",
        "Hull16_Grid": "Hull16+Grid",
    }

    lookup = {}
    for r in rows:
        key = (r["endpoint"], r["envelope"], int(r.get("subdivisions", 1)))
        lookup[key] = r

    fig, axes = plt.subplots(1, 3, figsize=(DOUBLE_COL, 2.4), sharey=True)

    for ax, (sub, envs) in zip(axes, sub_groups):
        x = np.arange(len(endpoints))
        width = 0.20 if len(envs) == 3 else 0.30
        center = (len(envs) - 1) / 2.0

        for i, env in enumerate(envs):
            vals = []
            errs = []
            times = []
            for ep in endpoints:
                row = lookup.get((ep, env, sub))
                vals.append(row["volume_mean"] if row else 0.0)
                errs.append(row["volume_std"] if row else 0.0)
                times.append(row["time_us_mean"] if row else 0.0)

            xpos = x + (i - center) * width
            ax.bar(
                xpos,
                vals,
                width,
                yerr=errs,
                capsize=2,
                color=PAL[i],
                hatch=HATCHES[i],
                edgecolor="black",
                linewidth=0.5,
                label=env_label[env],
                zorder=3,
            )

            for j, ep in enumerate(endpoints):
                if vals[j] <= 0.0:
                    continue
                ax.text(
                    xpos[j],
                    vals[j] + errs[j] + 0.01,
                    f"{_fmt_time_us(times[j])}us",
                    ha="center",
                    va="bottom",
                    fontsize=5,
                    rotation=70,
                )

        ax.set_title(f"sub={sub}", fontsize=8, fontweight="bold")
        ax.set_xticks(x)
        ax.set_xticklabels(endpoints, rotation=0)
        ax.set_xlabel("Endpoint")
        ax.set_ylim(bottom=0.0)

    axes[0].set_ylabel("Envelope Volume (m^3)")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=3, framealpha=0.9, bbox_to_anchor=(0.5, 1.08))
    fig.suptitle("IIWA14 Envelope Tightness by Subdivision", fontsize=9, fontweight="bold", y=1.14)
    fig.tight_layout()
    savefig(fig, "fig3_envelope_tightness")


def plot_signed_gap(gap_data):
    robot_data = gap_data["robots"]["iiwa14"]
    methods = ["IFK", "CritSample", "Analytical", "GCPC"]
    axes_names = ["x", "y", "z"]

    fig, axs = plt.subplots(1, 2, figsize=(DOUBLE_COL, 2.4), sharey=False)

    x = np.arange(len(methods))
    width = 0.24

    # (a) mean signed extent gap
    for ai, axis_name in enumerate(axes_names):
        vals = []
        for m in methods:
            vals.append(robot_data[m]["extent_gap"]["mean"][ai])
        axs[0].bar(
            x + (ai - 1) * width,
            vals,
            width,
            color=PAL[ai],
            hatch=HATCHES[ai],
            edgecolor="black",
            linewidth=0.5,
            label=axis_name,
            zorder=3,
        )
    axs[0].set_title("(a) Mean signed extent gap", fontsize=8)
    axs[0].set_xticks(x)
    axs[0].set_xticklabels(methods)
    axs[0].set_ylabel("Gap (m)")

    # (b) most negative extent gap
    for ai, axis_name in enumerate(axes_names):
        vals = []
        for m in methods:
            vals.append(robot_data[m]["extent_gap"]["max_negative"][ai])
        axs[1].bar(
            x + (ai - 1) * width,
            vals,
            width,
            color=PAL[ai],
            hatch=HATCHES[ai],
            edgecolor="black",
            linewidth=0.5,
            label=axis_name,
            zorder=3,
        )
    axs[1].set_title("(b) Most negative extent gap", fontsize=8)
    axs[1].set_xticks(x)
    axs[1].set_xticklabels(methods)
    axs[1].set_ylabel("Gap (m)")
    axs[1].axhline(0.0, color="black", linewidth=0.6)

    handles, labels = axs[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=3, framealpha=0.9, bbox_to_anchor=(0.5, 1.06))
    fig.suptitle("MC-baseline Signed Extent Gap (LinkIAABB)", fontsize=9, fontweight="bold", y=1.13)
    fig.tight_layout()
    savefig(fig, "fig3_signed_gap")


def main():
    setup_ieee_style()

    root = Path(__file__).resolve().parent.parent
    s1_path = root / "experiments" / "results_iiwa14_final" / "s1_envelope_tightness" / "results.json"
    gap_path = root / "experiments" / "results_iiwa14_final" / "s1_mc_linkiaabb_gap_results.json"

    s1 = _load_json(s1_path)
    rows = [r for r in s1.get("rows", []) if r.get("robot") == "iiwa14"]
    plot_envelope_grouped(rows)

    gap = _load_json(gap_path)
    plot_signed_gap(gap)
    print("Done: Fig 3 + gap")


if __name__ == "__main__":
    main()
