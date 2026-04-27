"""Compute statistical comparisons (Wilcoxon, 95 % CI) for the v7 paper.

Compares SBF vs OMPL-RRTConnect and SBF vs Drake-GCS on the IIWA14
shelf scene over 20 paired seeds, on both total wall time and path
length. Emits a LaTeX table and a macros file that the paper inputs.
"""
from __future__ import annotations
import json
import math
from pathlib import Path
from statistics import median

import numpy as np
from scipy import stats


HERE = Path(__file__).resolve().parent
SRC = HERE.parent / "experiments" / "results_nightly" / "full"
OUT_TEX = HERE.parent / "doc" / "generated" / "tab_stats.tex"
OUT_MAC = HERE.parent / "doc" / "generated" / "stats_macros.tex"


def load_trials(name: str, time_key: str, length_key: str):
    """Return ordered (time_ms, length) lists, one per seed (skip failures)."""
    data = json.loads((SRC / name).read_text())
    times, lengths = [], []
    for tr in data["trials"]:
        if not tr.get("success", False):
            continue
        t = tr.get(time_key)
        L = tr.get(length_key)
        if t is None or L is None:
            continue
        times.append(float(t))
        lengths.append(float(L))
    return np.array(times), np.array(lengths)


def bootstrap_ci(x: np.ndarray, n_boot: int = 10000, alpha: float = 0.05):
    """Percentile bootstrap 95 % CI for the median."""
    rng = np.random.default_rng(0)
    n = len(x)
    if n == 0:
        return float("nan"), float("nan")
    boots = np.empty(n_boot)
    for i in range(n_boot):
        boots[i] = np.median(rng.choice(x, size=n, replace=True))
    lo = float(np.quantile(boots, alpha / 2))
    hi = float(np.quantile(boots, 1 - alpha / 2))
    return lo, hi


def wilcoxon_paired(a: np.ndarray, b: np.ndarray):
    """Two-sided Wilcoxon signed-rank on paired samples (truncated to min len)."""
    n = min(len(a), len(b))
    if n < 5:
        return float("nan"), float("nan")
    a, b = a[:n], b[:n]
    diff = a - b
    if np.all(diff == 0):
        return float("nan"), 1.0
    res = stats.wilcoxon(a, b, alternative="two-sided", zero_method="wilcox")
    return float(res.statistic), float(res.pvalue)


def hodges_lehmann(a: np.ndarray, b: np.ndarray) -> float:
    """Hodges-Lehmann estimator of location shift (median of pairwise diffs)."""
    diffs = (a[:, None] - b[None, :]).ravel()
    return float(np.median(diffs))


def fmt_p(p: float) -> str:
    if math.isnan(p):
        return "n/a"
    if p < 1e-3:
        return "$<10^{-3}$"
    if p < 1e-2:
        return f"${p:.3f}$"
    return f"${p:.2f}$"


def main() -> None:
    sbf_t, sbf_L = load_trials("main_iiwa14_far.json", "total_time_ms", "opt_length")
    ompl_t, ompl_L = load_trials("ompl_iiwa14_far.json", "total_time_ms", "path_length")
    drk_t, drk_L = load_trials("drake_iiwa14_far.json", "total_time_ms", "path_length")

    # Paired stats
    pairs = [
        ("SBF vs OMPL-RRTC", sbf_t, ompl_t, sbf_L, ompl_L),
        ("SBF vs Drake-GCS", sbf_t, drk_t, sbf_L, drk_L),
    ]
    rows = []
    for label, a_t, b_t, a_L, b_L in pairs:
        wt, pt = wilcoxon_paired(a_t, b_t)
        wL, pL = wilcoxon_paired(a_L, b_L)
        hl_t = hodges_lehmann(a_t, b_t)
        hl_L = hodges_lehmann(a_L, b_L)
        rows.append((label, hl_t, pt, hl_L, pL))

    # Per-method medians and 95 % CI on the IIWA14 scene
    methods = [
        ("SBF (ours)", sbf_t, sbf_L),
        ("OMPL-RRTC", ompl_t, ompl_L),
        ("Drake-GCS", drk_t, drk_L),
    ]
    summary_rows = []
    for name, t, L in methods:
        t_lo, t_hi = bootstrap_ci(t)
        L_lo, L_hi = bootstrap_ci(L)
        summary_rows.append(
            (name, len(t), median(t), t_lo, t_hi, median(L), L_lo, L_hi)
        )

    # Emit LaTeX table
    lines = [
        r"\begin{table}[t]",
        r"  \centering",
        r"  \caption{Per-seed statistical comparison on the IIWA14 shelf scene"
        r" (20 seeds). Top: median wall time and path length with 95\,\% bootstrap"
        r" confidence intervals. Bottom: paired two-sided Wilcoxon signed-rank tests"
        r" with the Hodges--Lehmann (HL) location shift; positive HL means SBF is"
        r" larger. On this single-shelf scene OMPL is significantly faster than SBF"
        r" because a single straight-line attempt suffices, and Drake-GCS produces a"
        r" significantly shorter path because it solves a global convex relaxation;"
        r" SBF is between the two on time and slightly longer on length. The"
        r" multi-query and dense-cabinet results in \Cref{tab:marcucci} and"
        r" \Cref{tab:irisnp_iso} reverse this picture.}",
        r"  \label{tab:stats}",
        r"  \resizebox{\columnwidth}{!}{%",
        r"  \begin{tabular}{lrcccc}",
        r"    \toprule",
        r"    Method & $n$ & Time [ms] & 95\,\% CI & Length [rad] & 95\,\% CI \\",
        r"    \midrule",
    ]
    for name, n, mt, t_lo, t_hi, mL, L_lo, L_hi in summary_rows:
        lines.append(
            f"    {name} & {n} & ${mt:.1f}$ & $[{t_lo:.1f},{t_hi:.1f}]$ "
            f"& ${mL:.2f}$ & $[{L_lo:.2f},{L_hi:.2f}]$ \\\\"
        )
    lines += [
        r"    \midrule",
        r"    \multicolumn{6}{l}{\emph{Paired Wilcoxon (HL shift, two-sided $p$):}} \\",
    ]
    for label, hl_t, pt, hl_L, pL in rows:
        lines.append(
            f"    {label} & & HL$_t={hl_t:+.1f}$ & {fmt_p(pt)} "
            f"& HL$_L={hl_L:+.2f}$ & {fmt_p(pL)} \\\\"
        )
    lines += [
        r"    \bottomrule",
        r"  \end{tabular}}",
        r"\end{table}",
        "",
    ]
    OUT_TEX.write_text("\n".join(lines))

    # Emit macros so the prose can cite individual numbers safely
    mac = []
    sbf_t_lo, sbf_t_hi = bootstrap_ci(sbf_t)
    sbf_L_lo, sbf_L_hi = bootstrap_ci(sbf_L)

    def add(name, value):
        mac.append(rf"\newcommand{{\{name}}}{{{value}}}")

    add("statSbfTimeMed", f"{median(sbf_t):.1f}")
    add("statSbfTimeCI", f"[{sbf_t_lo:.1f},{sbf_t_hi:.1f}]")
    add("statSbfLenMed", f"{median(sbf_L):.2f}")
    add("statSbfLenCI", f"[{sbf_L_lo:.2f},{sbf_L_hi:.2f}]")
    add("statSbfVsOmplPt", fmt_p(rows[0][2]).strip("$"))
    add("statSbfVsOmplPL", fmt_p(rows[0][4]).strip("$"))
    add("statSbfVsDrkPt", fmt_p(rows[1][2]).strip("$"))
    add("statSbfVsDrkPL", fmt_p(rows[1][4]).strip("$"))
    OUT_MAC.write_text("\n".join(mac) + "\n")
    print(f"Wrote {OUT_TEX} and {OUT_MAC}")


if __name__ == "__main__":
    main()
