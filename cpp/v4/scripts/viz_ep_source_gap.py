"""
viz_ep_source_gap.py — Signed gap visualisation with Analytical baseline.

Reads the output directory produced by exp_ep_source_gap and generates:
  1. gap_dist.html        — per-robot, per-source signed-gap distribution
  2. gap_by_link.html     — median signed gap by link (grouped by robot/source)
  3. gap_by_axis.html     — per-axis signed-gap distribution, split by robot
  4. gap_vs_width.html    — mean signed gap vs. width_frac, split by robot/source
  5. neg_gap_heatmap.html — negative-gap frequency heatmap, split by robot/source

Usage:
    python scripts/viz_ep_source_gap.py <results_dir>

Dependencies:
    pip install plotly pandas numpy
"""

import argparse
import os
import numpy as np
import pandas as pd
import plotly.graph_objects as go
from plotly.subplots import make_subplots

# ── Style constants ────────────────────────────────────────────────────────────

SOURCE_META = {
    "CritSample": {"label": "CritSample",  "color": "#e83030"},
    "IFK":        {"label": "IFK",         "color": "#f5a623"},
    "MC":         {"label": "MC",          "color": "#4a90d9"},
}
SOURCE_ORDER = ["CritSample", "IFK", "MC"]

AXIS_COLORS = {"x": "#e74c3c", "y": "#27ae60", "z": "#2980b9"}

WIDTH_BANDS = {
    "narrow": (0.0,  0.19),
    "medium": (0.19, 0.45),
    "wide":   (0.45, 1.01),
}


def classify_width(w: float) -> str:
    if w < 0.19:
        return "narrow"
    if w < 0.45:
        return "medium"
    return "wide"


def save_html(fig: go.Figure, path: str) -> None:
    fig.write_html(path, include_plotlyjs="cdn")
    print(f"  Saved: {path}")


# ── 1. gap_dist.html — overall gap distribution per source ────────────────────

def plot_gap_dist(df: pd.DataFrame, out_dir: str) -> None:
    robots = sorted(df["robot"].unique())
    fig = make_subplots(
        rows=1,
        cols=len(robots),
        subplot_titles=[f"robot: {robot}" for robot in robots],
        shared_yaxes=True,
    )
    for col_idx, robot in enumerate(robots, start=1):
        sub_robot = df[df["robot"] == robot]
        for src in SOURCE_ORDER:
            meta = SOURCE_META[src]
            sub = sub_robot[sub_robot["source"] == src]["gap_signed_m"]
            fig.add_trace(go.Box(
                y=sub,
                name=meta["label"],
                marker_color=meta["color"],
                boxmean="sd",
                jitter=0.3,
                pointpos=-1.8,
                legendgroup=src,
                showlegend=(col_idx == 1),
            ), row=1, col=col_idx)

    fig.update_layout(
        title="Signed gap relative to Analytical baseline<br>"
              "<sub>gap_signed > 0: method looser than Analytical; gap_signed < 0: method tighter than Analytical</sub>",
        yaxis_title="gap_signed (m)",
        yaxis_zeroline=True,
        yaxis_zerolinecolor="#999",
        plot_bgcolor="white",
        paper_bgcolor="white",
        legend_title="Compared source",
    )
    save_html(fig, os.path.join(out_dir, "gap_dist.html"))


# ── 2. gap_by_link.html — median gap per active link per source ───────────────

def plot_gap_by_link(df: pd.DataFrame, out_dir: str) -> None:
    fig = go.Figure()
    robots = sorted(df["robot"].unique())
    for robot in robots:
        robot_df = df[df["robot"] == robot]
        links = sorted(robot_df["link"].unique())
        for src in SOURCE_ORDER:
            meta = SOURCE_META[src]
            sub = robot_df[robot_df["source"] == src]
            medians = [sub[sub["link"] == lnk]["gap_signed_m"].median() for lnk in links]
            fig.add_trace(go.Bar(
                x=[f"{robot}:L{l}" for l in links],
                y=medians,
                name=f"{robot} / {meta['label']}",
                marker_color=meta["color"],
            ))

    fig.update_layout(
        title="Median signed gap by link",
        yaxis_title="median gap_signed (m)",
        barmode="group",
        plot_bgcolor="white",
        paper_bgcolor="white",
        legend_title="Robot / source",
        yaxis_zeroline=True,
        yaxis_zerolinecolor="#999",
    )
    save_html(fig, os.path.join(out_dir, "gap_by_link.html"))


# ── 3. gap_by_axis.html — per-axis per-source gap box plots ──────────────────

def plot_gap_by_axis(df: pd.DataFrame, out_dir: str) -> None:
    axes = ["x", "y", "z"]
    robots = sorted(df["robot"].unique())
    fig = make_subplots(
        rows=len(robots), cols=3,
        subplot_titles=[f"{robot} / {axis}" for robot in robots for axis in axes],
        shared_yaxes=True,
    )
    for row_idx, robot in enumerate(robots, start=1):
        for col_idx, ax in enumerate(axes, start=1):
            for src in SOURCE_ORDER:
                meta = SOURCE_META[src]
                sub = df[
                    (df["robot"] == robot)
                    & (df["source"] == src)
                    & (df["axis"] == ax)
                ]["gap_signed_m"]
                show_legend = (row_idx == 1 and col_idx == 1)
                fig.add_trace(
                    go.Box(
                        y=sub,
                        name=meta["label"],
                        marker_color=meta["color"],
                        boxmean=True,
                        showlegend=show_legend,
                        legendgroup=src,
                    ),
                    row=row_idx, col=col_idx,
                )

    fig.update_layout(
        title="Signed gap distribution by axis and robot",
        plot_bgcolor="white",
        paper_bgcolor="white",
        legend_title="Compared source",
        yaxis_title="gap_signed (m)",
    )
    for row_idx in range(1, len(robots) + 1):
        for col_idx in range(1, 4):
            fig.update_yaxes(zeroline=True, zerolinecolor="#999", row=row_idx, col=col_idx)

    save_html(fig, os.path.join(out_dir, "gap_by_axis.html"))


# ── 4. gap_vs_width.html — gap vs. width_frac scatter ────────────────────────

def plot_gap_vs_width(df: pd.DataFrame, out_dir: str) -> None:
    fig = go.Figure()
    robots = sorted(df["robot"].unique())
    for robot in robots:
        robot_df = df[df["robot"] == robot]
        for src in SOURCE_ORDER:
            meta = SOURCE_META[src]
            sub = robot_df[robot_df["source"] == src]
            grp = sub.groupby(["robot", "trial"]).agg(
                mean_gap=("gap_signed_m", "mean"),
                width_frac=("width_frac", "first"),
            ).reset_index()

            fig.add_trace(go.Scatter(
                x=grp["width_frac"],
                y=grp["mean_gap"],
                mode="markers",
                name=f"{robot} / {meta['label']} mean",
                marker=dict(color=meta["color"], size=5, opacity=0.5),
            ))

            grp_sorted = grp.sort_values("width_frac")
            rolling = grp_sorted["mean_gap"].rolling(window=10, center=True, min_periods=1).mean()
            fig.add_trace(go.Scatter(
                x=grp_sorted["width_frac"],
                y=rolling,
                mode="lines",
                name=f"{robot} / {meta['label']} trend",
                line=dict(color=meta["color"], width=2),
            ))

    fig.update_layout(
        title="Mean signed gap vs. C-space box width fraction",
        xaxis_title="width_frac",
        yaxis_title="mean gap_signed (m)",
        plot_bgcolor="white",
        paper_bgcolor="white",
        legend_title="Robot / source / stat",
        yaxis_zeroline=True,
        yaxis_zerolinecolor="#999",
    )
    save_html(fig, os.path.join(out_dir, "gap_vs_width.html"))


# ── 5. neg_gap_heatmap.html — negative-gap frequency for CritSample ──────────

def plot_neg_gap_heatmap(df: pd.DataFrame, out_dir: str) -> None:
    axes = ["x", "y", "z"]
    robots = sorted(df["robot"].unique())
    fig = make_subplots(
        rows=len(robots),
        cols=len(SOURCE_ORDER),
        subplot_titles=[f"{robot} / {src}" for robot in robots for src in SOURCE_ORDER],
    )

    for row_idx, robot in enumerate(robots, start=1):
        for col_idx, src in enumerate(SOURCE_ORDER, start=1):
            sub = df[(df["robot"] == robot) & (df["source"] == src)].copy()
            if sub.empty:
                continue
            links = sorted(sub["link"].unique())
            z = np.zeros((len(links), len(axes)))
            for li, lnk in enumerate(links):
                for di, ax in enumerate(axes):
                    cell = sub[(sub["link"] == lnk) & (sub["axis"] == ax)]["gap_signed_m"]
                    z[li, di] = (cell < 0).mean() if len(cell) else 0.0

            fig.add_trace(
                go.Heatmap(
                    z=z,
                    x=axes,
                    y=[f"link {l}" for l in links],
                    colorscale="Reds",
                    zmin=0,
                    zmax=1,
                    showscale=(row_idx == 1 and col_idx == len(SOURCE_ORDER)),
                    colorbar=dict(title="neg freq", tickformat=".0%"),
                ),
                row=row_idx,
                col=col_idx,
            )

    fig.update_layout(
        title="Fraction of intervals where signed gap < 0<br>"
              "<sub>Negative means tighter than Analytical on that side</sub>",
        plot_bgcolor="white",
        paper_bgcolor="white",
    )
    save_html(fig, os.path.join(out_dir, "neg_gap_heatmap.html"))


# ── Summary statistics ────────────────────────────────────────────────────────

def print_summary(df: pd.DataFrame) -> None:
    print("\n── Summary ──────────────────────────────────────────────────────")
    print(f"  Trials per robot: {df['trial'].nunique()}")
    for robot in sorted(df["robot"].unique()):
        print(f"  robot={robot}")
        robot_df = df[df["robot"] == robot]
        for width_band in ["all", "narrow", "medium", "wide"]:
            band_df = robot_df if width_band == "all" else robot_df[robot_df["width_band"] == width_band]
            if band_df.empty:
                continue
            print(f"    width_band={width_band}")
            for src in SOURCE_ORDER:
                sub = band_df[band_df["source"] == src]["gap_signed_m"]
                if sub.empty:
                    continue
                neg_frac = (sub < 0).mean()
                pos_frac = (sub > 0).mean()
                max_abs_idx = sub.abs().idxmax()
                max_abs = sub.loc[max_abs_idx]
                print(
                    f"      {src:10s} mean={sub.mean():+.4f}m  p50={sub.median():+.4f}m  "
                    f"min={sub.min():+.4f}m  max={sub.max():+.4f}m  max_abs={max_abs:+.4f}m  "
                    f"neg_frac={neg_frac:.2%}  pos_frac={pos_frac:.2%}"
                )
    print()


# ── CLI ───────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Visualise exp_ep_source_gap results.")
    parser.add_argument("results_dir",
                        help="Path to the exp_ep_source_gap output directory.")
    args = parser.parse_args()

    csv_path = os.path.join(args.results_dir, "results.csv")
    if not os.path.isfile(csv_path):
        raise SystemExit(f"ERROR: {csv_path} not found")

    print(f"Loading {csv_path} …")
    df = pd.read_csv(csv_path)
    df["width_band"] = df["width_frac"].apply(classify_width)

    print_summary(df)

    out_dir = args.results_dir
    print("Generating plots …")
    plot_gap_dist(df, out_dir)
    plot_gap_by_link(df, out_dir)
    plot_gap_by_axis(df, out_dir)
    plot_gap_vs_width(df, out_dir)
    plot_neg_gap_heatmap(df, out_dir)

    print(f"\nAll plots written to: {out_dir}")


if __name__ == "__main__":
    main()
