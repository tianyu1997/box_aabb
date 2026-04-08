"""
sbf5_bench/stats.py — Statistical significance analysis

Provides Wilcoxon signed-rank tests for pairwise planner comparisons
and LaTeX annotation helpers.
"""

from __future__ import annotations

import json
import os
import re
from typing import Dict, List, Optional, Tuple

import numpy as np

from .runner import ExperimentResults


def pairwise_significance(
    results: ExperimentResults,
    metric: str = "planning_time_s",
    alpha: float = 0.05,
) -> Dict[Tuple[str, str], dict]:
    """Wilcoxon signed-rank test between all planner pairs.

    For each pair of planners, we align trials by (scene, seed) and
    compare the specified metric.

    Returns:
        dict[(planner_a, planner_b)] -> {
            "statistic": W,
            "p_value": p,
            "significant": p < alpha,
            "effect_direction": "a < b" | "a > b" | "ns",
            "n_pairs": int,
        }
    """
    from scipy.stats import wilcoxon

    # Group by (planner, scene, seed)
    data: Dict[Tuple[str, str, int], float] = {}
    for t in results.trials:
        val = _extract_metric(t, metric)
        if val is not None:
            data[(t.planner, t.scene, t.seed)] = val

    planners = sorted({k[0] for k in data})
    output = {}

    for i, pa in enumerate(planners):
        for pb in planners[i + 1:]:
            # Find aligned pairs
            vals_a, vals_b = [], []
            scenes_seeds = {(k[1], k[2]) for k in data if k[0] == pa}
            scenes_seeds &= {(k[1], k[2]) for k in data if k[0] == pb}

            for sc, sd in sorted(scenes_seeds):
                va = data.get((pa, sc, sd))
                vb = data.get((pb, sc, sd))
                if va is not None and vb is not None:
                    vals_a.append(va)
                    vals_b.append(vb)

            if len(vals_a) < 5:
                # Not enough paired observations
                output[(pa, pb)] = {
                    "statistic": None,
                    "p_value": 1.0,
                    "significant": False,
                    "effect_direction": "ns",
                    "n_pairs": len(vals_a),
                }
                continue

            diffs = np.array(vals_a) - np.array(vals_b)
            # Remove zeros (ties) for Wilcoxon
            nonzero = diffs[diffs != 0]
            if len(nonzero) < 5:
                output[(pa, pb)] = {
                    "statistic": None,
                    "p_value": 1.0,
                    "significant": False,
                    "effect_direction": "ns",
                    "n_pairs": len(vals_a),
                }
                continue

            stat, p = wilcoxon(nonzero)
            sig = bool(p < alpha)
            if sig:
                direction = "a < b" if np.median(diffs) < 0 else "a > b"
            else:
                direction = "ns"

            output[(pa, pb)] = {
                "statistic": float(stat),
                "p_value": float(p),
                "significant": sig,
                "effect_direction": direction,
                "n_pairs": len(vals_a),
            }

    return output


def annotate_table_significance(
    latex_str: str,
    sig_results: Dict[Tuple[str, str], dict],
    reference_planner: str = "SBF-GCPC-LinkIAABB_Grid",
) -> str:
    """Add $^*$ markers to LaTeX table for statistically significant
    improvements over the reference planner.

    Scans each row for a planner name and appends $^*$ if that planner
    is significantly different from reference_planner.
    """
    sig_planners = set()
    for (pa, pb), info in sig_results.items():
        if not info["significant"]:
            continue
        if pa == reference_planner:
            sig_planners.add(pb)
        elif pb == reference_planner:
            sig_planners.add(pa)

    if not sig_planners:
        return latex_str

    lines = latex_str.split("\n")
    out = []
    for line in lines:
        modified = line
        for planner in sig_planners:
            # Match planner name in a table row (before first &)
            if planner in line and "&" in line:
                modified = modified.replace(
                    planner, planner + r"$^*$", 1)
                break
        out.append(modified)
    return "\n".join(out)


def save_significance(
    sig_results: Dict[Tuple[str, str], dict],
    path: str,
) -> None:
    """Save significance results to JSON."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    # Convert tuple keys to strings
    serializable = {}
    for (pa, pb), info in sig_results.items():
        key = f"{pa} vs {pb}"
        serializable[key] = info
    with open(path, "w", encoding="utf-8") as f:
        json.dump(serializable, f, indent=2)


def _extract_metric(trial, metric: str):
    """Extract a metric value from a TrialResult."""
    if metric == "planning_time_s":
        return trial.metrics.planning_time_s
    if metric == "path_length":
        if trial.planning_result.success:
            return trial.metrics.path_length
        return None
    if metric == "smoothness_mean":
        if trial.planning_result.success:
            return trial.metrics.smoothness_mean
        return None
    if metric == "success":
        return 1.0 if trial.planning_result.success else 0.0
    # Generic fallback: try metrics dict
    d = trial.metrics.to_dict()
    return d.get(metric)
