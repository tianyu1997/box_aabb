"""
sbf5_bench/report.py — Result reporting (Markdown + LaTeX tables)

Generates summary tables and optional Plotly figures from
ExperimentResults.
"""

from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np

from .runner import ExperimentResults


def summary_table(results: ExperimentResults) -> str:
    """Generate a Markdown summary table.

    Aggregates across seeds: mean ± std for numeric columns,
    success rate as percentage.
    """
    if not results.trials:
        return "No trials."

    # Group by (scene, planner)
    groups: Dict[tuple, list] = {}
    for t in results.trials:
        key = (t.scene, t.planner)
        groups.setdefault(key, []).append(t)

    header = (
        "| Planner | Scene | Success% | Time(s) | PathLen "
        "| Smooth | Clearance | Boxes |"
    )
    sep = "|---|---|---|---|---|---|---|---|"
    lines = [header, sep]

    for (scene, planner), trials in sorted(groups.items()):
        n = len(trials)
        n_success = sum(1 for t in trials if t.planning_result.success)
        success_pct = 100.0 * n_success / n if n > 0 else 0.0

        times = [t.metrics.planning_time_s for t in trials]
        lengths = [t.metrics.path_length for t in trials
                   if t.planning_result.success]
        smooths = [t.metrics.smoothness_mean for t in trials
                   if t.planning_result.success]
        clears = [t.metrics.clearance_min for t in trials
                  if t.planning_result.success
                  and np.isfinite(t.metrics.clearance_min)]
        boxes = [t.planning_result.nodes_explored for t in trials
                 if t.planning_result.success]

        def _fmt(vals):
            if not vals:
                return "—"
            m = np.mean(vals)
            s = np.std(vals)
            return f"{m:.3f}±{s:.3f}"

        boxes_str = str(int(np.mean(boxes))) if boxes else "—"

        lines.append(
            f"| {planner} | {scene} | {success_pct:.0f}% "
            f"| {_fmt(times)} | {_fmt(lengths)} "
            f"| {_fmt(smooths)} | {_fmt(clears)} | {boxes_str} |"
        )

    return "\n".join(lines)


def latex_table(results: ExperimentResults) -> str:
    """Generate a LaTeX booktabs table (paper-ready).

    Requires: \\usepackage{booktabs} in preamble.
    """
    if not results.trials:
        return "% No trials."

    groups: Dict[tuple, list] = {}
    for t in results.trials:
        key = (t.scene, t.planner)
        groups.setdefault(key, []).append(t)

    lines = [
        r"\begin{table}[ht]",
        r"\centering",
        r"\caption{SBF v5 Benchmark Results}",
        r"\label{tab:benchmark}",
        r"\begin{tabular}{llcccccc}",
        r"\toprule",
        (r"Scene & Planner & Success\% & Time (s) & PathLen "
         r"& Smooth & Clearance & Boxes \\"),
        r"\midrule",
    ]

    for (scene, planner), trials in sorted(groups.items()):
        n = len(trials)
        n_success = sum(1 for t in trials if t.planning_result.success)
        success_pct = 100.0 * n_success / n if n > 0 else 0.0

        times = [t.metrics.planning_time_s for t in trials]
        lengths = [t.metrics.path_length for t in trials
                   if t.planning_result.success]
        smooths = [t.metrics.smoothness_mean for t in trials
                   if t.planning_result.success]
        clears = [t.metrics.clearance_min for t in trials
                  if t.planning_result.success
                  and np.isfinite(t.metrics.clearance_min)]
        boxes = [t.planning_result.nodes_explored for t in trials
                 if t.planning_result.success]

        def _fmt(vals, prec=2):
            if not vals:
                return "---"
            m = np.mean(vals)
            return f"{m:.{prec}f}"

        def _fmt_pm(vals, prec=2):
            if not vals:
                return "---"
            m = np.mean(vals)
            s = np.std(vals)
            return f"${m:.{prec}f} \\pm {s:.{prec}f}$"

        scene_esc = scene.replace("_", r"\_")
        clear_str = _fmt(clears) if clears else "---"
        boxes_str = str(int(np.mean(boxes))) if boxes else "---"

        lines.append(
            f"  {scene_esc} & {planner} & {success_pct:.1f} "
            f"& {_fmt_pm(times)} & {_fmt_pm(lengths)} "
            f"& {_fmt_pm(smooths)} & {clear_str} & {boxes_str} \\\\"
        )

    lines += [
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
    ]
    return "\n".join(lines)


def plot_comparison(results: ExperimentResults):
    """Plotly bar chart: planners × metrics (optional dependency)."""
    try:
        import plotly.graph_objects as go
    except ImportError:
        raise ImportError("plotly is required for plot_comparison()")

    groups: Dict[tuple, list] = {}
    for t in results.trials:
        key = (t.scene, t.planner)
        groups.setdefault(key, []).append(t)

    planners_set = sorted({t.planner for t in results.trials})
    scenes_set = sorted({t.scene for t in results.trials})

    fig = go.Figure()
    for planner in planners_set:
        times = []
        labels = []
        for scene in scenes_set:
            key = (scene, planner)
            trials = groups.get(key, [])
            if trials:
                avg_time = np.mean([t.metrics.planning_time_s for t in trials])
                times.append(avg_time)
            else:
                times.append(0)
            labels.append(scene)

        fig.add_trace(go.Bar(name=planner, x=labels, y=times))

    fig.update_layout(
        title="Planning Time by Planner × Scene",
        xaxis_title="Scene",
        yaxis_title="Time (s)",
        barmode="group",
    )
    return fig


def plot_time_vs_dof(results: ExperimentResults):
    """Planning time vs DOF scatter (optional dependency)."""
    try:
        import plotly.graph_objects as go
    except ImportError:
        raise ImportError("plotly is required for plot_time_vs_dof()")

    # Infer DOF from path shape
    planner_data: Dict[str, tuple] = {}
    for t in results.trials:
        if t.planning_result.success and t.planning_result.path is not None:
            dof = t.planning_result.path.shape[1]
            planner_data.setdefault(t.planner, ([], []))
            planner_data[t.planner][0].append(dof)
            planner_data[t.planner][1].append(t.metrics.planning_time_s)

    fig = go.Figure()
    for planner, (dofs, times) in planner_data.items():
        fig.add_trace(go.Scatter(
            x=dofs, y=times, mode="markers", name=planner))

    fig.update_layout(
        title="Planning Time vs DOF",
        xaxis_title="DOF",
        yaxis_title="Time (s)",
    )
    return fig


# ──────────────────────────────────────────────────────────────
# Phase T: Paper-ready LaTeX table generators
# ──────────────────────────────────────────────────────────────

def envelope_volume_table(s1_data: dict) -> str:
    """Generate Table 1 LaTeX: envelope volume comparison.

    Args:
        s1_data: dict with keys "rows" — list of dicts, each having:
            robot, endpoint, envelope, volume_mean, ratio_pct, safe (bool)
    """
    rows = s1_data.get("rows", [])
    lines = [
        r"\begin{table}[t]",
        r"\centering",
        r"\caption{Envelope volume comparison across pipeline configurations.",
        r"$V$ = mean envelope volume.",
        r"$R$ = compression ratio vs IFK-LinkIAABB baseline.}",
        r"\label{tab:envelope_volume}",
        r"\begin{tabular}{lllccc}",
        r"\toprule",
        r"Robot & Endpoint & Envelope & $V$ & $R$ (\%) & Safe? \\",
        r"\midrule",
    ]

    for r in rows:
        robot = r.get("robot", "")
        ep = r.get("endpoint", "")
        env = r.get("envelope", "").replace("_", r"\_")
        vol = r.get("volume_mean", 0.0)
        ratio = r.get("ratio_pct", 0.0)
        safe = r.get("safe", False)

        vol_str = f"{vol:.4g}"
        ratio_str = f"{ratio:.1f}"
        safe_str = r"\cmark" if safe else r"\xmark"

        lines.append(f"  {robot} & {ep} & {env} & {vol_str} & {ratio_str} & {safe_str} \\\\")

    lines += [
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
    ]
    return "\n".join(lines)


def timing_table(s2_data: dict) -> str:
    """Generate Table 2 LaTeX: computation timing per box.

    Args:
        s2_data: dict with "rows" — list of dicts, each having:
            robot, endpoint, envelope, ep_us_mean, env_us_mean, total_us_mean, speedup
    """
    rows = s2_data.get("rows", [])
    lines = [
        r"\begin{table}[t]",
        r"\centering",
        r"\caption{Envelope computation time ($\mu$s) per box.}",
        r"\label{tab:timing}",
        r"\begin{tabular}{llrrrr}",
        r"\toprule",
        r"Endpoint & Envelope & EP ($\mu$s) & Env ($\mu$s) & Total ($\mu$s) & Speedup \\",
        r"\midrule",
    ]

    totals = [r.get("total_us_mean", float("inf")) for r in rows
              if r.get("total_us_mean") is not None]
    best_total = min(totals) if totals else None

    for r in rows:
        ep = r.get("endpoint", "")
        env = r.get("envelope", "").replace("_", r"\_")
        ep_us = r.get("ep_us_mean", 0)
        env_us = r.get("env_us_mean", 0)
        total_us = r.get("total_us_mean", 0)
        speedup = r.get("speedup", 1.0)

        total_str = f"{total_us:.0f}"
        if best_total is not None and total_us == best_total:
            total_str = r"\textbf{" + total_str + "}"

        speedup_str = f"{speedup:.1f}$\\times$"
        lines.append(
            f"  {ep} & {env} & {ep_us:.0f} & {env_us:.0f} & {total_str} & {speedup_str} \\\\"
        )

    lines += [
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
    ]
    return "\n".join(lines)


def e2e_table(s3_results: ExperimentResults) -> str:
    """Generate Table 3 LaTeX: end-to-end planning (double-column).

    Groups trials by (planner, scene) and shows success rate + planning time.
    """
    if not s3_results.trials:
        return "% No trials."

    # Discover scenes and planners
    scenes = sorted({t.scene for t in s3_results.trials})
    planners = sorted({t.planner for t in s3_results.trials})

    groups: Dict[tuple, list] = {}
    for t in s3_results.trials:
        groups.setdefault((t.planner, t.scene), []).append(t)

    n_scenes = len(scenes)
    col_spec = "ll" + "c" * n_scenes
    scene_headers = " & ".join(s.replace("_", r"\_") for s in scenes)

    lines = [
        r"\begin{table*}[t]",
        r"\centering",
        r"\caption{Planning performance across configurations and scenes.}",
        r"\label{tab:e2e}",
        r"\begin{tabular}{" + col_spec + "}",
        r"\toprule",
        r"Endpoint & Envelope & " + scene_headers + r" \\",
        r"\midrule",
        r"\multicolumn{" + str(n_scenes + 2) + r"}{c}{\textit{Success Rate (\%)}} \\",
        r"\midrule",
    ]

    # Success rate rows
    for planner in planners:
        parts = planner.split("-", 2)
        ep_name = parts[1] if len(parts) > 1 else planner
        env_name = parts[2].replace("_", r"\_") if len(parts) > 2 else ""
        cells = []
        for scene in scenes:
            trials = groups.get((planner, scene), [])
            if trials:
                n_success = sum(1 for t in trials if t.planning_result.success)
                pct = 100.0 * n_success / len(trials)
                cells.append(f"{pct:.0f}")
            else:
                cells.append("---")
        lines.append(f"  {ep_name} & {env_name} & " + " & ".join(cells) + r" \\")

    lines.append(r"\midrule")
    lines.append(
        r"\multicolumn{" + str(n_scenes + 2)
        + r"}{c}{\textit{Planning Time (s)}} \\"
    )
    lines.append(r"\midrule")

    # Planning time rows
    for planner in planners:
        parts = planner.split("-", 2)
        ep_name = parts[1] if len(parts) > 1 else planner
        env_name = parts[2].replace("_", r"\_") if len(parts) > 2 else ""
        cells = []
        for scene in scenes:
            trials = groups.get((planner, scene), [])
            if trials:
                times = [t.metrics.planning_time_s for t in trials]
                cells.append(f"{np.mean(times):.2f}")
            else:
                cells.append("---")
        lines.append(f"  {ep_name} & {env_name} & " + " & ".join(cells) + r" \\")

    lines += [
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table*}",
    ]
    return "\n".join(lines)


def baseline_table(s4_results: ExperimentResults) -> str:
    """Generate Table 4 LaTeX: baseline comparison.

    Groups trials by planner, reports success%, time, path length, smoothness.
    """
    if not s4_results.trials:
        return "% No trials."

    groups: Dict[str, list] = {}
    for t in s4_results.trials:
        groups.setdefault(t.planner, []).append(t)

    lines = [
        r"\begin{table}[t]",
        r"\centering",
        r"\caption{SBF vs sampling-based planners on representative scenes.}",
        r"\label{tab:baselines}",
        r"\begin{tabular}{lcccc}",
        r"\toprule",
        r"Planner & Success\% & Time (s) & PathLen & Smooth \\",
        r"\midrule",
    ]

    # Collect aggregated stats for bold detection
    planner_stats = {}
    for planner, trials in sorted(groups.items()):
        n = len(trials)
        n_success = sum(1 for t in trials if t.planning_result.success)
        success_pct = 100.0 * n_success / n if n > 0 else 0.0
        times = [t.metrics.planning_time_s for t in trials]
        lengths = [t.metrics.path_length for t in trials
                   if t.planning_result.success]
        smooths = [t.metrics.smoothness_mean for t in trials
                   if t.planning_result.success]
        planner_stats[planner] = {
            "success_pct": success_pct,
            "time": np.mean(times) if times else float("inf"),
            "pathlen": np.mean(lengths) if lengths else float("inf"),
            "smooth": np.mean(smooths) if smooths else float("inf"),
        }

    # Determine bests
    best_success = max(s["success_pct"] for s in planner_stats.values())
    best_time = min(s["time"] for s in planner_stats.values())
    best_pathlen = min(s["pathlen"] for s in planner_stats.values())
    best_smooth = min(s["smooth"] for s in planner_stats.values())

    for planner in sorted(groups.keys()):
        s = planner_stats[planner]

        def _bold_if(val, best, fmt):
            formatted = fmt.format(val)
            if abs(val - best) < 1e-9:
                return r"\textbf{" + formatted + "}"
            return formatted

        succ_str = _bold_if(s["success_pct"], best_success, "{:.0f}")
        time_str = _bold_if(s["time"], best_time, "{:.2f}")
        path_str = _bold_if(s["pathlen"], best_pathlen, "{:.1f}")
        smooth_str = _bold_if(s["smooth"], best_smooth, "{:.2f}")

        lines.append(
            f"  {planner} & {succ_str} & {time_str} & {path_str} & {smooth_str} \\\\"
        )

    lines += [
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
    ]
    return "\n".join(lines)

