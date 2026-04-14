"""Analyze split_compare.csv — BEST_TIGHTEN vs ROUND_ROBIN comparison.

Usage:
    python viz/analyze_split_compare.py results/split_compare.csv

Produces:
    results/split_bt_vs_rr_volume.png   — paired volume scatter + ratio hist
    results/split_bt_vs_rr_time.png     — paired time scatter + ratio hist
    results/split_bt_vs_rr_fk.png       — paired FK-calls scatter + ratio hist
    results/split_summary.txt           — summary statistics table
"""

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load_and_pair(csv_path: str) -> pd.DataFrame:
    """Load CSV and create paired rows (BT vs RR on same seed)."""
    df = pd.read_csv(csv_path)
    # Pivot so each row is one (robot, hw, scene, seed)
    rr = df[df["split_order"] == "ROUND_ROBIN"].copy()
    bt = df[df["split_order"] == "BEST_TIGHTEN"].copy()

    keys = ["robot", "half_width", "scene", "seed"]
    merged = rr.merge(bt, on=keys, suffixes=("_rr", "_bt"))
    return merged


def scatter_with_ratio(ax_scatter, ax_hist, x, y, xlabel, ylabel, title):
    """Left: scatter x vs y with y=x line.  Right: ratio histogram."""
    mask = (x > 0) & (y > 0)
    x, y = x[mask], y[mask]
    if len(x) == 0:
        ax_scatter.text(0.5, 0.5, "No data (all zero)", transform=ax_scatter.transAxes,
                        ha="center", va="center")
        ax_hist.text(0.5, 0.5, "No data", transform=ax_hist.transAxes,
                     ha="center", va="center")
        return

    ax_scatter.scatter(x, y, s=8, alpha=0.3, edgecolors="none")
    lo = min(x.min(), y.min()) * 0.9
    hi = max(x.max(), y.max()) * 1.1
    ax_scatter.plot([lo, hi], [lo, hi], "k--", lw=0.8, label="y=x")
    ax_scatter.set_xlabel(xlabel)
    ax_scatter.set_ylabel(ylabel)
    ax_scatter.set_title(title)
    ax_scatter.set_aspect("equal", adjustable="datalim")
    ax_scatter.legend(fontsize=8)

    ratio = y / x
    ax_hist.hist(ratio, bins=40, edgecolor="black", linewidth=0.5, alpha=0.7)
    med = np.median(ratio)
    ax_hist.axvline(med, color="red", ls="--", lw=1.2, label=f"median={med:.3f}")
    ax_hist.axvline(1.0, color="black", ls=":", lw=0.8)
    ax_hist.set_xlabel(f"{ylabel} / {xlabel}")
    ax_hist.set_ylabel("count")
    ax_hist.set_title(f"Ratio histogram (median={med:.3f})")
    ax_hist.legend(fontsize=8)


def make_metric_figure(merged, metric, label, out_dir):
    """Create a 2-panel figure (scatter + ratio hist) for one metric."""
    robots = merged["robot"].unique()
    n = len(robots)
    fig, axes = plt.subplots(n, 2, figsize=(12, 5 * n), squeeze=False)

    for i, robot in enumerate(robots):
        sub = merged[merged["robot"] == robot]
        x = sub[f"{metric}_rr"].values.astype(float)
        y = sub[f"{metric}_bt"].values.astype(float)
        scatter_with_ratio(
            axes[i, 0], axes[i, 1],
            x, y,
            f"ROUND_ROBIN {label}", f"BEST_TIGHTEN {label}",
            f"{robot}: {label}"
        )

    fig.tight_layout()
    out = out_dir / f"split_bt_vs_rr_{metric}.png"
    fig.savefig(str(out), dpi=150)
    plt.close(fig)
    print(f"  Saved {out}")


def make_hw_breakdown(merged, out_dir):
    """Bar chart of median volume ratio grouped by half_width and robot."""
    robots = merged["robot"].unique()
    fig, axes = plt.subplots(1, len(robots), figsize=(6 * len(robots), 5),
                             squeeze=False)
    for i, robot in enumerate(robots):
        sub = merged[(merged["robot"] == robot) &
                     (merged["success_bt"] == 1) &
                     (merged["success_rr"] == 1)]
        if sub.empty:
            continue
        sub = sub.copy()
        sub["vol_ratio"] = sub["box_volume_bt"] / sub["box_volume_rr"]
        grp = sub.groupby("half_width")["vol_ratio"].median()
        axes[0, i].bar(range(len(grp)), grp.values, tick_label=[
            f"{h:.3f}" for h in grp.index])
        axes[0, i].axhline(1.0, color="black", ls=":", lw=0.8)
        axes[0, i].set_xlabel("half_width")
        axes[0, i].set_ylabel("median(vol_BT / vol_RR)")
        axes[0, i].set_title(f"{robot}: Volume Ratio by half_width")
        axes[0, i].tick_params(axis="x", rotation=45)

    fig.tight_layout()
    out = out_dir / "split_bt_vs_rr_hw_breakdown.png"
    fig.savefig(str(out), dpi=150)
    plt.close(fig)
    print(f"  Saved {out}")


def summary_table(merged, out_dir):
    """Print and save summary statistics."""
    lines = []
    lines.append("=" * 80)
    lines.append("Split Strategy Comparison: BEST_TIGHTEN vs ROUND_ROBIN")
    lines.append("=" * 80)

    for robot in merged["robot"].unique():
        sub = merged[merged["robot"] == robot]
        lines.append(f"\n--- {robot} ({len(sub)} paired samples) ---")

        # Success rates
        succ_rr = sub["success_rr"].sum()
        succ_bt = sub["success_bt"].sum()
        lines.append(f"  Success: RR={succ_rr}/{len(sub)}  BT={succ_bt}/{len(sub)}")

        # Both-success subset
        both = sub[(sub["success_bt"] == 1) & (sub["success_rr"] == 1)]
        if len(both) == 0:
            lines.append("  (no both-success pairs)")
            continue

        # Volume
        vol_ratio = both["box_volume_bt"] / both["box_volume_rr"]
        lines.append(f"  Volume ratio (BT/RR): "
                     f"median={vol_ratio.median():.4f}  "
                     f"mean={vol_ratio.mean():.4f}  "
                     f"std={vol_ratio.std():.4f}")
        lines.append(f"    BT>RR: {(vol_ratio > 1).sum()}/{len(both)}  "
                     f"  RR>BT: {(vol_ratio < 1).sum()}/{len(both)}  "
                     f"  equal: {(vol_ratio == 1).sum()}/{len(both)}")

        # Time
        time_ratio = both["time_us_bt"] / both["time_us_rr"]
        lines.append(f"  Time ratio (BT/RR): "
                     f"median={time_ratio.median():.4f}  "
                     f"mean={time_ratio.mean():.4f}")
        lines.append(f"    avg_time: RR={both['time_us_rr'].mean():.1f}us  "
                     f"BT={both['time_us_bt'].mean():.1f}us")

        # FK calls
        fk_ratio = both["n_fk_bt"] / both["n_fk_rr"]
        lines.append(f"  FK ratio (BT/RR): "
                     f"median={fk_ratio.median():.4f}  "
                     f"mean={fk_ratio.mean():.4f}")
        lines.append(f"    avg_fk: RR={both['n_fk_rr'].mean():.1f}  "
                     f"BT={both['n_fk_bt'].mean():.1f}")

        # Depth
        lines.append(f"  Depth: RR_avg={both['depth_rr'].mean():.2f}  "
                     f"BT_avg={both['depth_bt'].mean():.2f}")

        # LECT nodes
        lines.append(f"  LECT nodes: RR_avg={both['lect_nodes_rr'].mean():.1f}  "
                     f"BT_avg={both['lect_nodes_bt'].mean():.1f}")

        # Per half_width breakdown
        lines.append(f"\n  Per half_width median vol ratio:")
        for hw, grp in both.groupby("half_width"):
            vr = grp["box_volume_bt"] / grp["box_volume_rr"]
            lines.append(f"    hw={hw:.3f}: vol_ratio={vr.median():.4f}  "
                         f"(n={len(grp)})")

    text = "\n".join(lines)
    print(text)
    out = out_dir / "split_summary.txt"
    out.write_text(text)
    print(f"\n  Saved {out}")


def main():
    if len(sys.argv) < 2:
        csv_path = "results/split_compare.csv"
    else:
        csv_path = sys.argv[1]

    out_dir = Path(csv_path).parent
    print(f"Loading {csv_path} ...")
    merged = load_and_pair(csv_path)
    print(f"  {len(merged)} paired (RR, BT) samples\n")

    make_metric_figure(merged, "box_volume", "box volume", out_dir)
    make_metric_figure(merged, "time_us", "time (us)", out_dir)
    make_metric_figure(merged, "n_fk", "FK calls", out_dir)
    make_hw_breakdown(merged, out_dir)
    print()
    summary_table(merged, out_dir)


if __name__ == "__main__":
    main()
