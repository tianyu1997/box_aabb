#!/usr/bin/env python3
"""Generate paper-ready figures from experiment JSON data.

Usage (from v5/):
    python python/scripts/gen_figures.py [--data-dir experiments/results]

Requires: pip install plotly kaleido
"""

import argparse
import json
import logging
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np

logger = logging.getLogger(__name__)

# ── Unified color schemes ────────────────────────────────────

PLANNER_COLORS = {
    "SBF-GCPC-Grid":       "#1f77b4",
    "SBF-Analytical-Box":  "#ff7f0e",
    "SBF-IFK-Box":         "#2ca02c",
    "RRTConnect":          "#d62728",
    "RRT*":                "#9467bd",
    "BIT*":                "#8c564b",
}

ENVELOPE_COLORS = {
    "LinkIAABB":      "#4c78a8",
    "LinkIAABB_Grid": "#f58518",
    "Hull16_Grid":    "#54a24b",
}

_PAPER_FONT = dict(family="serif", size=10)
_SINGLE_COL = dict(width=504, height=360)   # 3.5in × 2.5in @ 144dpi
_DOUBLE_COL = dict(width=1008, height=432)   # 7in × 3in @ 144dpi


def _get_color(name: str) -> str:
    """Look up color, fall back to a hash-based color."""
    if name in PLANNER_COLORS:
        return PLANNER_COLORS[name]
    if name in ENVELOPE_COLORS:
        return ENVELOPE_COLORS[name]
    h = hash(name) % 360
    return f"hsl({h}, 60%, 50%)"


def _save_figure(fig, out_dir: str, basename: str):
    """Save figure as PDF (vector) and PNG (300 DPI)."""
    os.makedirs(out_dir, exist_ok=True)
    html_path = os.path.join(out_dir, f"{basename}.html")
    fig.write_html(html_path)
    logger.info("Wrote %s", html_path)
    try:
        pdf_path = os.path.join(out_dir, f"{basename}.pdf")
        fig.write_image(pdf_path)
        logger.info("Wrote %s", pdf_path)

        png_path = os.path.join(out_dir, f"{basename}.png")
        fig.write_image(png_path, scale=2)  # ~300 DPI
        logger.info("Wrote %s", png_path)
    except (ImportError, ValueError) as e:
        logger.warning("Static image export failed (install kaleido): %s", e)


def load_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


# ── Figure 1: Envelope Timing Bar Chart (S2) ────────────────

def fig_envelope_timing(s2_data: dict):
    """Grouped bar: endpoint sources × envelope types, y = total time (μs).
    Uses panda robot data only.
    """
    import plotly.graph_objects as go

    rows = s2_data.get("rows", [])
    # Filter to panda (more interesting for paper)
    rows = [r for r in rows if r.get("robot") == "panda"]
    if not rows:
        logger.warning("No S2 panda rows for timing figure")
        return None

    # Group by endpoint
    endpoints = []
    seen = set()
    for r in rows:
        ep = r.get("endpoint", "")
        if ep not in seen:
            endpoints.append(ep)
            seen.add(ep)

    envelopes = []
    seen = set()
    for r in rows:
        env = r.get("envelope", "")
        if env not in seen:
            envelopes.append(env)
            seen.add(env)

    # Build lookup
    lookup = {(r["endpoint"], r["envelope"]): r for r in rows}

    fig = go.Figure()
    for env in envelopes:
        totals = []
        for ep in endpoints:
            r = lookup.get((ep, env))
            totals.append(r["total_us_mean"] if r else 0)
        fig.add_trace(go.Bar(
            name=env, x=endpoints, y=totals,
            marker_color=_get_color(env),
        ))

    fig.update_layout(
        barmode="group",
        xaxis_title="Endpoint Source",
        yaxis_title="Total Time (μs)",
        font=_PAPER_FONT,
        **_SINGLE_COL,
        margin=dict(l=60, r=20, t=30, b=50),
        legend=dict(x=0.02, y=0.98, bgcolor="rgba(255,255,255,0.8)"),
    )
    return fig


# ── Figure 2: E2E Planning Heatmap (S3) ─────────────────────

def fig_e2e_heatmap(s3_path: str):
    """Heatmap: rows = configs, cols = scenes. Color = planning_time_s.
    Annotation = success_rate%.
    """
    import plotly.graph_objects as go

    from sbf5_bench.runner import ExperimentResults

    results = ExperimentResults.load(s3_path)
    if not results.trials:
        return None

    scenes = sorted({t.scene for t in results.trials})
    planners = sorted({t.planner for t in results.trials})

    groups = {}
    for t in results.trials:
        groups.setdefault((t.planner, t.scene), []).append(t)

    z = []  # planning time (log-friendly)
    text = []  # success rate annotations
    for planner in planners:
        row_z = []
        row_t = []
        for scene in scenes:
            trials = groups.get((planner, scene), [])
            if trials:
                times = [t.metrics.planning_time_s for t in trials]
                n_ok = sum(1 for t in trials if t.planning_result.success)
                pct = 100.0 * n_ok / len(trials)
                avg_time = np.mean(times)
                row_z.append(max(avg_time, 1e-4))  # floor for log
                row_t.append(f"{pct:.0f}%")
            else:
                row_z.append(None)
                row_t.append("")
        z.append(row_z)
        text.append(row_t)

    scene_labels = [s.replace("_", " ") for s in scenes]
    planner_labels = planners

    fig = go.Figure(data=go.Heatmap(
        z=z, x=scene_labels, y=planner_labels,
        text=text, texttemplate="%{text}", textfont=dict(size=9),
        colorscale="Viridis", reversescale=True,
        colorbar=dict(title="Time (s)"),
    ))

    fig.update_layout(
        font=_PAPER_FONT,
        **_DOUBLE_COL,
        margin=dict(l=140, r=20, t=30, b=50),
        xaxis=dict(side="bottom"),
    )
    return fig


# ── Figure 3: Baseline Pareto Plot (S4) ─────────────────────

def fig_baseline_pareto(s4_path: str):
    """Scatter: x = planning_time, y = path_length.
    Each point = one planner (mean ± std error bars).
    """
    import plotly.graph_objects as go

    from sbf5_bench.runner import ExperimentResults

    results = ExperimentResults.load(s4_path)
    if not results.trials:
        return None

    groups = {}
    for t in results.trials:
        groups.setdefault(t.planner, []).append(t)

    fig = go.Figure()
    for planner, trials in sorted(groups.items()):
        ok = [t for t in trials if t.planning_result.success]
        if not ok:
            continue
        times = [t.metrics.planning_time_s for t in ok]
        lengths = [t.metrics.path_length for t in ok]

        fig.add_trace(go.Scatter(
            x=[np.mean(times)], y=[np.mean(lengths)],
            error_x=dict(type="data", array=[np.std(times)], visible=True),
            error_y=dict(type="data", array=[np.std(lengths)], visible=True),
            mode="markers+text",
            marker=dict(size=10, color=_get_color(planner)),
            text=[planner], textposition="top center",
            textfont=dict(size=8),
            name=planner,
            showlegend=True,
        ))

    fig.update_layout(
        xaxis_title="Planning Time (s)",
        yaxis_title="Path Length",
        font=_PAPER_FONT,
        width=504, height=432,  # 3.5in × 3in
        margin=dict(l=60, r=20, t=30, b=50),
        legend=dict(x=0.02, y=0.98, bgcolor="rgba(255,255,255,0.8)"),
    )
    return fig


# ── Figure 4: Scalability Line Plots (S5) ───────────────────

def fig_scalability(s5_data: dict):
    """3 subplots: (a) time vs DOF, (b) success vs obstacles, (c) success vs max_boxes.

    S5 data format from run_s5():
      time_vs_dof:      {"2": {time_mean, time_std, success_rate}, "7": ...}
      success_vs_obstacles: {"1": {time_mean, time_std, success_rate}, "2": ...}
      success_vs_max_boxes: {"50": ..., "100": ...}
    """
    import plotly.graph_objects as go
    from plotly.subplots import make_subplots

    fig = make_subplots(
        rows=1, cols=3,
        subplot_titles=["(a) DOF", "(b) Obstacles", "(c) Budget"],
    )

    color = "#1f77b4"

    # (a) time vs DOF
    dof_data = s5_data.get("time_vs_dof", {})
    dofs = sorted(int(k) for k in dof_data.keys())
    times = [dof_data[str(d)]["time_mean"] for d in dofs]
    stds = [dof_data[str(d)]["time_std"] for d in dofs]
    fig.add_trace(go.Bar(
        x=[str(d) for d in dofs], y=times,
        error_y=dict(type="data", array=stds, visible=True),
        marker_color=color, name="SBF", showlegend=True,
    ), row=1, col=1)

    # (b) success vs obstacle count
    obs_data = s5_data.get("success_vs_obstacles", {})
    n_obs = sorted(int(k) for k in obs_data.keys())
    obs_success = [obs_data[str(n)]["success_rate"] for n in n_obs]
    obs_times = [obs_data[str(n)]["time_mean"] for n in n_obs]
    fig.add_trace(go.Scatter(
        x=n_obs, y=obs_success, mode="lines+markers",
        line=dict(color=color), marker=dict(color=color, size=6),
        name="Success %", showlegend=False,
    ), row=1, col=2)

    # (c) success vs max boxes
    box_data = s5_data.get("success_vs_max_boxes", {})
    budgets = sorted(int(k) for k in box_data.keys())
    box_success = [box_data[str(b)]["success_rate"] for b in budgets]
    fig.add_trace(go.Scatter(
        x=budgets, y=box_success, mode="lines+markers",
        line=dict(color=color), marker=dict(color=color, size=6),
        name="Success %", showlegend=False,
    ), row=1, col=3)

    fig.update_xaxes(title_text="DOF", row=1, col=1)
    fig.update_yaxes(title_text="Time (s)", row=1, col=1)
    fig.update_xaxes(title_text="# Obstacles", row=1, col=2)
    fig.update_yaxes(title_text="Success (%)", range=[0, 105], row=1, col=2)
    fig.update_xaxes(title_text="Max Boxes", row=1, col=3)
    fig.update_yaxes(title_text="Success (%)", range=[0, 105], row=1, col=3)

    fig.update_layout(
        font=_PAPER_FONT,
        **_DOUBLE_COL,
        margin=dict(l=60, r=20, t=40, b=50),
    )
    return fig


# ── Main ─────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Generate paper figures")
    parser.add_argument("--data-dir", default="experiments/results",
                        help="Root directory with experiment data")
    parser.add_argument("--output-dir", default=None,
                        help="Output dir (default: paper/fig)")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")

    data_dir = args.data_dir
    out_dir = args.output_dir or os.path.join(
        os.path.dirname(__file__), "..", "..", "paper", "fig")

    # Fig 1: Envelope Timing (S2)
    s2_path = os.path.join(data_dir, "s2_envelope_timing", "results.json")
    if os.path.exists(s2_path):
        s2_data = load_json(s2_path)
        fig = fig_envelope_timing(s2_data)
        if fig:
            _save_figure(fig, out_dir, "fig1_envelope_timing")
    else:
        logger.warning("S2 data not found: %s", s2_path)

    # Fig 2: E2E Heatmap (S3)
    s3_path = os.path.join(data_dir, "s3_e2e", "results.json")
    if os.path.exists(s3_path):
        fig = fig_e2e_heatmap(s3_path)
        if fig:
            _save_figure(fig, out_dir, "fig2_e2e_heatmap")
    else:
        logger.warning("S3 data not found: %s", s3_path)

    # Fig 3: Baseline Pareto (S4)
    s4_path = os.path.join(data_dir, "s4_baselines", "results.json")
    if os.path.exists(s4_path):
        fig = fig_baseline_pareto(s4_path)
        if fig:
            _save_figure(fig, out_dir, "fig3_baseline_pareto")
    else:
        logger.warning("S4 data not found: %s", s4_path)

    # Fig 4: Scalability (S5)
    s5_path = os.path.join(data_dir, "s5_scalability", "results.json")
    if os.path.exists(s5_path):
        s5_data = load_json(s5_path)
        fig = fig_scalability(s5_data)
        if fig:
            _save_figure(fig, out_dir, "fig4_scalability")
    else:
        logger.warning("S5 data not found: %s", s5_path)

    logger.info("Done. Figures → %s", out_dir)


if __name__ == "__main__":
    main()
