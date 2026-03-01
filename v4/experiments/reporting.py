"""
experiments/reporting.py — 论文实验结果报告生成

对应论文实验:
  Table I  : Exp1 Region 生成效率与覆盖率 (Coverage)
  Table II : Exp2 端到端 GCS 规划性能 (E2E)
  Table III: Exp3 增量更新效率 (Incremental)
  Fig. 2   : 覆盖率随时间增长曲线
  Fig. 4   : Region 生成时间曲线 (E2E)
  Fig. 5   : 增量更新与热启动柱状图

用法:
    from experiments.reporting import generate_report
    generate_report("output/raw/paper_exp1_coverage.json")
"""

from __future__ import annotations

import json
import logging
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

logger = logging.getLogger(__name__)

OUTPUT_DIR = Path(__file__).resolve().parent / "output"
TABLES_DIR = OUTPUT_DIR / "tables"
FIGURES_DIR = OUTPUT_DIR / "figures"


# ═══════════════════════════════════════════════════════════════════════════
# Data loading
# ═══════════════════════════════════════════════════════════════════════════

def load_results(path: str | Path) -> dict:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _group_by(results: List[dict],
              keys: Sequence[str]) -> Dict[str, List[dict]]:
    """Group result dicts by composite key."""
    groups = defaultdict(list)
    for r in results:
        key = " | ".join(str(r.get(k, "")) for k in keys)
        groups[key].append(r)
    return dict(groups)


def _stats(values: List[float]) -> Dict[str, float]:
    arr = np.array(values)
    return {
        "n": len(arr),
        "mean": float(np.mean(arr)),
        "std": float(np.std(arr)),
        "median": float(np.median(arr)),
        "min": float(np.min(arr)),
        "max": float(np.max(arr)),
        "p5": float(np.percentile(arr, 5)),
        "p95": float(np.percentile(arr, 95)),
    }


# ═══════════════════════════════════════════════════════════════════════════
# LaTeX table generation
# ═══════════════════════════════════════════════════════════════════════════

def generate_latex_table(data: dict,
                         group_keys: Sequence[str] = ("scene", "planner"),
                         metrics: Sequence[str] = ("planning_time",
                                                    "cost", "success"),
                         caption: str = "",
                         label: str = "") -> str:
    """Generate LaTeX tabular from experiment results."""
    results = data["results"]
    groups = _group_by(results, group_keys)

    # header
    n_metrics = len(metrics)
    col_spec = "l" * len(group_keys) + "r" * n_metrics
    lines = [
        r"\begin{table}[htbp]",
        r"\centering",
        f"\\caption{{{caption}}}" if caption else "",
        f"\\label{{{label}}}" if label else "",
        f"\\begin{{tabular}}{{{col_spec}}}",
        r"\toprule",
    ]

    # header row
    header_parts = list(group_keys) + [m.replace("_", " ") for m in metrics]
    lines.append(" & ".join(f"\\textbf{{{h}}}" for h in header_parts)
                 + r" \\")
    lines.append(r"\midrule")

    # data rows
    for key, trials in sorted(groups.items()):
        key_parts = key.split(" | ")
        row_parts = list(key_parts)

        for metric in metrics:
            values = [t.get(metric) for t in trials
                      if t.get(metric) is not None]
            if metric == "success":
                n_success = sum(1 for v in values if v)
                row_parts.append(f"{n_success}/{len(values)}")
            elif values:
                s = _stats([float(v) for v in values
                            if not (isinstance(v, float) and np.isnan(v))])
                row_parts.append(f"{s['mean']:.3f} $\\pm$ {s['std']:.3f}")
            else:
                row_parts.append("--")

        lines.append(" & ".join(row_parts) + r" \\")

    lines.extend([
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
    ])

    return "\n".join(line for line in lines if line)


def save_latex_table(data: dict, filename: str, **kwargs) -> Path:
    """Generate and save LaTeX table."""
    TABLES_DIR.mkdir(parents=True, exist_ok=True)
    tex = generate_latex_table(data, **kwargs)
    out = TABLES_DIR / filename
    out.write_text(tex, encoding="utf-8")
    logger.info("Saved LaTeX table → %s", out)
    return out


# ═══════════════════════════════════════════════════════════════════════════
# Figure generation (matplotlib)
# ═══════════════════════════════════════════════════════════════════════════

def plot_comparison_violin(data: dict,
                           metric: str = "planning_time",
                           output_name: str = "comparison_violin.pdf"):
    """Violin plot comparing planners across scenes."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        logger.warning("matplotlib not available — skipping plot")
        return None

    FIGURES_DIR.mkdir(parents=True, exist_ok=True)

    results = data["results"]
    groups = _group_by(results, ["planner"])

    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    names = sorted(groups.keys())
    plot_data = []
    for name in names:
        vals = [r.get(metric, float("nan")) for r in groups[name]]
        vals = [v for v in vals if not np.isnan(v)]
        plot_data.append(vals)

    if all(len(d) > 0 for d in plot_data):
        parts = ax.violinplot(plot_data, showmeans=True, showmedians=True)
        ax.set_xticks(range(1, len(names) + 1))
        ax.set_xticklabels(names, rotation=30, ha="right")

    ax.set_ylabel(metric.replace("_", " ").title())
    ax.set_title(f"{data.get('experiment', 'Experiment')} — {metric}")
    ax.grid(True, alpha=0.3)

    out = FIGURES_DIR / output_name
    fig.tight_layout()
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    logger.info("Saved figure → %s", out)
    return out


def plot_reuse_curves(data: dict,
                      output_name: str = "forest_reuse_curves.pdf"):
    """Plot cumulative time vs K for forest reuse experiment."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        logger.warning("matplotlib not available — skipping plot")
        return None

    FIGURES_DIR.mkdir(parents=True, exist_ok=True)

    results = data["results"]
    groups = _group_by(results, ["planner"])

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    for name in sorted(groups.keys()):
        trials = groups[name]
        # group by K
        k_groups = defaultdict(list)
        for t in trials:
            k = t.get("K")
            if k is not None:
                k_groups[k].append(t.get("cumulative_time", 0))

        ks = sorted(k_groups.keys())
        means_cum = [np.mean(k_groups[k]) for k in ks]
        means_avg = [np.mean(k_groups[k]) / k for k in ks]

        ax1.plot(ks, means_cum, "o-", label=name)
        ax2.plot(ks, means_avg, "s-", label=name)

    ax1.set_xlabel("K (number of queries)")
    ax1.set_ylabel("Cumulative time (s)")
    ax1.set_title("Cumulative Planning Time vs K")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    ax2.set_xlabel("K (number of queries)")
    ax2.set_ylabel("Avg time per query (s)")
    ax2.set_title("Average Per-Query Time vs K")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    out = FIGURES_DIR / output_name
    fig.tight_layout()
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    logger.info("Saved figure → %s", out)
    return out


def plot_scalability(data: dict,
                     output_name: str = "scalability.pdf"):
    """Plot time vs obstacle count and vs DOF."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        logger.warning("matplotlib not available — skipping plot")
        return None

    FIGURES_DIR.mkdir(parents=True, exist_ok=True)

    results = data["results"]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 5))

    # time vs obstacle count (grouped by planner, for Panda only)
    panda_results = [r for r in results if r.get("robot") == "panda"]
    groups = _group_by(panda_results, ["planner"])
    for name in sorted(groups.keys()):
        trials = groups[name]
        obs_groups = defaultdict(list)
        for t in trials:
            n_obs = t.get("n_obstacles")
            if n_obs is not None:
                obs_groups[n_obs].append(t.get("planning_time", 0))

        xs = sorted(obs_groups.keys())
        ys = [np.mean(obs_groups[x]) for x in xs]
        ax1.plot(xs, ys, "o-", label=name)

    ax1.set_xlabel("Number of obstacles")
    ax1.set_ylabel("Planning time (s)")
    ax1.set_title("Time vs Obstacles (Panda 7-DOF)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # time vs DOF (fixed obstacle count)
    target_obs = 8
    dof_results = [r for r in results if r.get("n_obstacles") == target_obs]
    groups = _group_by(dof_results, ["planner"])
    for name in sorted(groups.keys()):
        trials = groups[name]
        dof_groups = defaultdict(list)
        for t in trials:
            dof = t.get("n_dof")
            if dof is not None:
                dof_groups[dof].append(t.get("planning_time", 0))

        xs = sorted(dof_groups.keys())
        ys = [np.mean(dof_groups[x]) for x in xs]
        ax2.bar([str(x) for x in xs], ys, label=name, alpha=0.7)

    ax2.set_xlabel("Degrees of Freedom")
    ax2.set_ylabel("Planning time (s)")
    ax2.set_title(f"Time vs DOF ({target_obs} obstacles)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    out = FIGURES_DIR / output_name
    fig.tight_layout()
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    logger.info("Saved figure → %s", out)
    return out


# ═══════════════════════════════════════════════════════════════════════════
# Unified report generator
# ═══════════════════════════════════════════════════════════════════════════

def generate_report(result_path: str | Path) -> None:
    """Generate all applicable tables and figures for a result file."""
    data = load_results(result_path)
    exp_name = data.get("experiment", "")

    print(f"\n=== Generating report for {exp_name} ===")
    print(f"  Trials: {data.get('n_trials', len(data.get('results', [])))}")

    if "coverage" in exp_name:
        # Exp 1: Coverage → Table I, Fig. 2
        generate_table_I(data)
        generate_fig_2(data)

    elif "e2e_gcs" in exp_name:
        # Exp 2: E2E → Table II, Fig. 4
        generate_table_II(data)
        generate_fig_4(data)

    elif "incremental" in exp_name:
        # Exp 3: Incremental → Table III, Fig. 5
        generate_table_III(data)
        generate_fig_5(data)

    # Legacy fallbacks
    elif "main_comparison" in exp_name:
        save_latex_table(data, "table1_main_comparison.tex",
                         caption="Main comparison of planning methods",
                         label="tab:main_comparison")
        plot_comparison_violin(data, output_name="fig2_main_comparison.pdf")

    print("  Done.")


# ═══════════════════════════════════════════════════════════════════════════
# Table II: E2E GCS Planning Performance (Exp 2)
# ═══════════════════════════════════════════════════════════════════════════

def generate_table_II(data: dict) -> Path:
    """Table II: 方法 × 场景 × 建图时间/查询时间/摊还时间/路径长度/认证."""
    TABLES_DIR.mkdir(parents=True, exist_ok=True)
    results = data["results"]

    # Separate first_query and amortized
    first_q = [r for r in results if r.get("mode") == "first_query"]
    amort = [r for r in results if r.get("mode") == "amortized"]

    groups_fq = _group_by(first_q, ["scene", "planner"])
    groups_am = _group_by(amort, ["scene", "planner"])

    lines = [
        r"\begin{table*}[htbp]",
        r"\centering",
        r"\caption{End-to-end GCS planning performance on Marcucci benchmark scenes. "
        r"Build: region generation time (computed once, reused across queries). "
        r"Query$_1$: single GCS solve time. "
        r"Query$_{10}$: amortized query time over $K\!=\!10$ queries. "
        r"$L$: C-space path length, Cert: certified collision-free regions.}",
        r"\label{tab:e2e_gcs}",
        r"\begin{tabular}{ll r rrr rr c}",
        r"\toprule",
        r"\textbf{Method} & \textbf{Scene} & \textbf{SR (\%)} & "
        r"\textbf{Build (s)} & \textbf{Query$_1$ (s)} & "
        r"\textbf{Query$_{10}$ (s)} & "
        r"\textbf{$L$ (rad)} & \textbf{Smooth} & \textbf{Cert} \\",
        r"\midrule",
    ]

    for key in sorted(groups_fq.keys()):
        trials_fq = groups_fq[key]
        trials_am = groups_am.get(key, [])
        parts = key.split(" | ")
        scene, planner = parts[0], parts[1]

        # Success rate
        successes = [r for r in trials_fq if r.get("success")]
        sr = len(successes) / max(len(trials_fq), 1) * 100

        # Build time (same across trials for one seed; report median)
        builds = [r.get("build_time", float("nan")) for r in trials_fq]
        builds = [b for b in builds if not np.isnan(b)]
        tb = _stats(builds) if builds else {"median": float("nan")}

        # Query time (pure GCS solve)
        queries = [r.get("query_time", r.get("wall_clock", float("nan")))
                   for r in trials_fq]
        queries = [q for q in queries if not np.isnan(q)]
        tq1 = _stats(queries) if queries else {"median": float("nan")}

        # Amortized query time (per query)
        times_am = [r.get("query_time", r.get("amortized_time", float("nan")))
                    for r in trials_am]
        times_am = [t for t in times_am if not np.isnan(t)]
        tq10 = _stats(times_am) if times_am else {"median": float("nan")}

        # Path length
        lengths = [r.get("path_length", float("nan")) for r in trials_fq
                    if r.get("success")]
        lengths = [l for l in lengths if not np.isnan(l)]
        pl = _stats(lengths) if lengths else {"median": float("nan")}

        # Smoothness
        smooths = [r.get("path_smoothness", float("nan")) for r in trials_fq
                    if r.get("success")]
        smooths = [s for s in smooths if not np.isnan(s)]
        sm = _stats(smooths) if smooths else {"median": float("nan")}

        # Certified
        cert = "\\checkmark" if any(r.get("certified") for r in trials_fq) \
            else "\\texttimes"

        row = (f"{planner} & {scene} & {sr:.0f} & "
               f"{tb['median']:.3f} & {tq1['median']:.3f} & "
               f"{tq10['median']:.3f} & "
               f"{pl['median']:.2f} & {sm['median']:.3f} & {cert}")
        lines.append(row + r" \\")

    lines.extend([
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table*}",
    ])

    tex = "\n".join(lines)
    out = TABLES_DIR / "table_II_e2e_gcs.tex"
    out.write_text(tex, encoding="utf-8")
    logger.info("Saved Table II → %s", out)
    return out


# ═══════════════════════════════════════════════════════════════════════════
# Table III: Incremental Update Efficiency (Exp 3)
# ═══════════════════════════════════════════════════════════════════════════

def generate_table_III(data: dict) -> Path:
    """Table III: SBF 增量 vs C-IRIS 全量 + HCACHE 对比 (含 build_time)."""
    TABLES_DIR.mkdir(parents=True, exist_ok=True)
    results = data["results"]

    # 3a: incremental vs full rebuild
    results_3a = [r for r in results if r.get("update_mode") is not None]
    # 3b: cold/warm/cross
    results_3b = [r for r in results if r.get("condition") in
                  ("cold", "warm", "cross_scene")]

    lines = [
        r"\begin{table}[htbp]",
        r"\centering",
        r"\caption{Incremental update efficiency. "
        r"Build: initial region generation (same as Exp~1/2). "
        r"Update: incremental update (SBF) or full rebuild (C-IRIS). "
        r"Bottom: HCACHE warmstart effect on build time.}",
        r"\label{tab:incremental}",
        r"\begin{tabular}{l l rrr}",
        r"\toprule",
    ]

    # ── Part A: Incremental ──
    lines.append(r"\multicolumn{5}{l}{\textit{(a) Incremental update}} \\")
    lines.append(r"\textbf{Condition} & \textbf{Method} & "
                 r"\textbf{Build (s)} & \textbf{Update (s)} & "
                 r"\textbf{Success} \\")
    lines.append(r"\midrule")

    groups_3a = _group_by(results_3a, ["condition", "planner"])
    for key in sorted(groups_3a.keys()):
        trials = groups_3a[key]
        parts = key.split(" | ")
        condition, planner = parts[0], parts[1]

        build_times = [r.get("build_time", 0) for r in trials]
        bt = _stats(build_times) if build_times else {"median": 0}
        update_times = [r.get("update_time", 0) for r in trials]
        ut = _stats(update_times) if update_times else {"median": 0}
        successes = sum(1 for r in trials if r.get("success_after"))
        n = len(trials)

        lines.append(
            f"{condition} & {planner} & {bt['median']:.3f} & "
            f"{ut['median']:.4f} & "
            f"{successes}/{n}" + r" \\")

    # ── Part B: HCACHE ──
    lines.append(r"\midrule")
    lines.append(r"\multicolumn{5}{l}{\textit{(b) HCACHE warmstart}} \\")
    lines.append(r"\textbf{Condition} & \textbf{Scene} & "
                 r"\textbf{Build (s)} & \textbf{Total (s)} & "
                 r"\textbf{FK (s)} \\")
    lines.append(r"\midrule")

    groups_3b = _group_by(results_3b, ["condition", "scene"])
    for key in sorted(groups_3b.keys()):
        trials = groups_3b[key]
        parts = key.split(" | ")
        condition, scene = parts[0], parts[1]

        build_times = [r.get("build_time", 0) for r in trials]
        bt = _stats(build_times) if build_times else {"median": 0}
        total_times = [r.get("total_time", 0) for r in trials]
        fk_times = [r.get("fk_time", 0) for r in trials]
        tt = _stats(total_times) if total_times else {"median": 0}
        ft = _stats(fk_times) if fk_times else {"median": 0}

        lines.append(
            f"{condition} & {scene} & {bt['median']:.3f} & "
            f"{tt['median']:.4f} & "
            f"{ft['median']:.4f}" + r" \\")

    lines.extend([
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
    ])

    tex = "\n".join(lines)
    out = TABLES_DIR / "table_III_incremental.tex"
    out.write_text(tex, encoding="utf-8")
    logger.info("Saved Table III → %s", out)
    return out


# ═══════════════════════════════════════════════════════════════════════════
# Table I: Coverage Efficiency (Exp 1)
# ═══════════════════════════════════════════════════════════════════════════

def generate_table_I(data: dict) -> Path:
    """Table I: 固定时间预算下 region 数 + 覆盖率."""
    TABLES_DIR.mkdir(parents=True, exist_ok=True)
    results = data["results"]

    # Filter main results (not ablation)
    main = [r for r in results
            if r.get("mode") not in ("strategy_ablation", "seed_strategy")]

    lines = [
        r"\begin{table}[htbp]",
        r"\centering",
        r"\caption{Region generation efficiency and free-space coverage "
        r"under fixed time budgets. Coverage estimated by Monte Carlo "
        r"($10^5$ samples).}",
        r"\label{tab:coverage}",
        r"\begin{tabular}{l l r rr c}",
        r"\toprule",
        r"\textbf{Method} & \textbf{Budget} & \textbf{Scene} & "
        r"\textbf{\#Reg} & \textbf{Cov (\%)} & \textbf{Cert} \\",
        r"\midrule",
    ]

    groups = _group_by(main, ["planner", "time_budget", "scene"])
    for key in sorted(groups.keys()):
        trials = groups[key]
        parts = key.split(" | ")
        planner = parts[0]
        budget = parts[1]
        scene = parts[2]

        n_regs = [r.get("n_regions", 0) for r in trials]
        covs = [r.get("coverage_rate", 0) for r in trials]
        nr = _stats(n_regs) if n_regs else {"median": 0}
        cv = _stats([c * 100 for c in covs]) if covs else {"median": 0}

        cert = "\\checkmark" if any(r.get("certified") for r in trials) \
            else "\\texttimes"

        lines.append(
            f"{planner} & {budget}s & {scene} & "
            f"{nr['median']:.0f} & {cv['median']:.1f} & {cert}"
            + r" \\")

    lines.extend([
        r"\bottomrule",
        r"\end{tabular}",
        r"\end{table}",
    ])

    tex = "\n".join(lines)
    out = TABLES_DIR / "table_I_coverage.tex"
    out.write_text(tex, encoding="utf-8")
    logger.info("Saved Table I → %s", out)
    return out


# ═══════════════════════════════════════════════════════════════════════════
# Fig. 4: Region generation time curve (Exp 2)
# ═══════════════════════════════════════════════════════════════════════════

def generate_fig_4(data: dict) -> Optional[Path]:
    """Fig. 4: 横轴 region 数 N, 纵轴累计生成时间 (log scale)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        logger.warning("matplotlib not available — skipping Fig. 4")
        return None

    FIGURES_DIR.mkdir(parents=True, exist_ok=True)
    results = data["results"]
    first_q = [r for r in results if r.get("mode") == "first_query"]

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), sharey=True)

    scenes = sorted(set(r.get("scene", "") for r in first_q))
    planners = sorted(set(r.get("planner", "") for r in first_q))

    colors = {"SBF-GCS": "#1f77b4", "IRIS-NP-GCS": "#ff7f0e",
              "C-IRIS-GCS": "#2ca02c", "PRM": "#d62728"}
    markers = {"SBF-GCS": "o", "IRIS-NP-GCS": "s",
               "C-IRIS-GCS": "^", "PRM": "D"}

    for ax_idx, scene in enumerate(scenes[:3]):
        ax = axes[ax_idx]
        scene_results = [r for r in first_q if r.get("scene") == scene]

        for planner in planners:
            pr = [r for r in scene_results if r.get("planner") == planner]
            if not pr:
                continue
            # Sort by nodes_explored (number of regions)
            pr.sort(key=lambda r: r.get("nodes_explored", 0))
            ns = [r.get("nodes_explored", 0) for r in pr]
            ts = [r.get("wall_clock", 0) for r in pr]

            ax.plot(ns, ts, marker=markers.get(planner, "o"),
                    color=colors.get(planner, None),
                    label=planner, alpha=0.7, markersize=4)

        ax.set_xlabel("Number of regions ($N$)")
        ax.set_title(scene.replace("marcucci_", "").capitalize())
        ax.set_yscale("log")
        ax.grid(True, alpha=0.3)
        if ax_idx == 0:
            ax.set_ylabel("Cumulative generation time (s)")

    axes[-1].legend(loc="upper left", fontsize=8)
    fig.suptitle("Fig. 4: Region Generation Time", fontsize=12)
    fig.tight_layout()

    out = FIGURES_DIR / "fig4_region_time.pdf"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    logger.info("Saved Fig. 4 → %s", out)
    return out


# ═══════════════════════════════════════════════════════════════════════════
# Fig. 5: Incremental update bar chart (Exp 3)
# ═══════════════════════════════════════════════════════════════════════════

def generate_fig_5(data: dict) -> Optional[Path]:
    """Fig. 5: Cold / Warm / Incremental 柱状图."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        logger.warning("matplotlib not available — skipping Fig. 5")
        return None

    FIGURES_DIR.mkdir(parents=True, exist_ok=True)
    results = data["results"]

    # 2a: incremental vs full rebuild
    results_2a = [r for r in results if r.get("update_mode") is not None]
    # 2b: cold/warm/cross
    results_2b = [r for r in results if r.get("condition") in
                  ("cold", "warm", "cross_scene")]

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.5))

    # ── Left: Incremental update ──
    if results_2a:
        groups = _group_by(results_2a, ["condition", "planner"])
        conditions = sorted(set(r.get("condition", "") for r in results_2a))
        planners_2a = sorted(set(r.get("planner", "") for r in results_2a))

        x = np.arange(len(conditions))
        width = 0.35

        for i, planner in enumerate(planners_2a):
            vals = []
            for cond in conditions:
                key = f"{cond} | {planner}"
                trials = groups.get(key, [])
                times = [r.get("update_time", 0) for r in trials]
                vals.append(np.median(times) if times else 0)
            ax1.bar(x + i * width, vals, width, label=planner,
                    alpha=0.85)

        ax1.set_xticks(x + width / 2)
        ax1.set_xticklabels(conditions, rotation=15, ha="right")
        ax1.set_ylabel("Update time (s)")
        ax1.set_yscale("log")
        ax1.set_title("(a) Incremental update vs full rebuild")
        ax1.legend(fontsize=8)
        ax1.grid(True, alpha=0.3, axis="y")

    # ── Right: HCACHE warmstart ──
    if results_2b:
        conds = ["cold", "warm", "cross_scene"]
        scenes_2b = sorted(set(r.get("scene", "") for r in results_2b))

        x = np.arange(len(conds))
        width = 0.25

        for i, scene in enumerate(scenes_2b[:3]):
            vals = []
            for cond in conds:
                trials = [r for r in results_2b
                          if r.get("condition") == cond
                          and r.get("scene") == scene]
                times = [r.get("fk_time", r.get("total_time", 0))
                         for r in trials]
                vals.append(np.median(times) if times else 0)
            ax2.bar(x + i * width, vals, width, label=scene, alpha=0.85)

        ax2.set_xticks(x + width)
        ax2.set_xticklabels(conds)
        ax2.set_ylabel("FK time (s)")
        ax2.set_yscale("log")
        ax2.set_title("(b) HCACHE cold / warm / cross-scene")
        ax2.legend(fontsize=8)
        ax2.grid(True, alpha=0.3, axis="y")

    fig.suptitle("Fig. 5: Incremental Update & Persistence", fontsize=12)
    fig.tight_layout()

    out = FIGURES_DIR / "fig5_incremental.pdf"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    logger.info("Saved Fig. 5 → %s", out)
    return out


# ═══════════════════════════════════════════════════════════════════════════
# Fig. 2: Coverage vs time curve (Exp 1)
# ═══════════════════════════════════════════════════════════════════════════

def generate_fig_2(data: dict) -> Optional[Path]:
    """Fig. 2: 覆盖率随时间增长 (半对数图)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        logger.warning("matplotlib not available — skipping Fig. 2")
        return None

    FIGURES_DIR.mkdir(parents=True, exist_ok=True)
    results = data["results"]

    # Main results only
    main = [r for r in results
            if r.get("mode") not in ("strategy_ablation", "seed_strategy")]

    scenes = sorted(set(r.get("scene", "") for r in main))
    planners = sorted(set(r.get("planner", "") for r in main))
    budgets = sorted(set(r.get("time_budget", 0) for r in main))

    colors = {"SBF-IFK": "#1f77b4", "IRIS-NP": "#ff7f0e",
              "C-IRIS": "#2ca02c"}

    fig, axes = plt.subplots(1, min(len(scenes), 3),
                              figsize=(15, 4.5), sharey=True)
    if len(scenes) == 1:
        axes = [axes]

    for ax_idx, scene in enumerate(scenes[:3]):
        ax = axes[ax_idx]
        scene_results = [r for r in main if r.get("scene") == scene]

        for planner in planners:
            pr = [r for r in scene_results if r.get("planner") == planner]
            if not pr:
                continue

            # Group by time budget
            budget_vals = {}
            for b in budgets:
                br = [r for r in pr if r.get("time_budget") == b]
                covs = [r.get("coverage_rate", 0) * 100 for r in br]
                if covs:
                    budget_vals[b] = (np.median(covs),
                                       np.percentile(covs, 25),
                                       np.percentile(covs, 75))

            if budget_vals:
                bs = sorted(budget_vals.keys())
                meds = [budget_vals[b][0] for b in bs]
                q25 = [budget_vals[b][1] for b in bs]
                q75 = [budget_vals[b][2] for b in bs]

                color = colors.get(planner, None)
                ax.plot(bs, meds, "o-", label=planner, color=color)
                ax.fill_between(bs, q25, q75, alpha=0.2, color=color)

        ax.set_xlabel("Time budget (s)")
        ax.set_title(scene.replace("marcucci_", "").capitalize())
        ax.set_xscale("log")
        ax.grid(True, alpha=0.3)
        if ax_idx == 0:
            ax.set_ylabel("Coverage (%)")

    axes[-1].legend(loc="lower right", fontsize=8)
    fig.suptitle("Fig. 2: Free-Space Coverage Efficiency", fontsize=12)
    fig.tight_layout()

    out = FIGURES_DIR / "fig2_coverage.pdf"
    fig.savefig(out, dpi=200, bbox_inches="tight")
    plt.close(fig)
    logger.info("Saved Fig. 2 → %s", out)
    return out


if __name__ == "__main__":
    import sys
    logging.basicConfig(level=logging.INFO)
    if len(sys.argv) < 2:
        print("Usage: python -m experiments.reporting <result_json>")
        sys.exit(1)
    generate_report(sys.argv[1])
