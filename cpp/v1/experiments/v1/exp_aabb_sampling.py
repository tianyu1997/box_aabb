"""
exp_aabb_sampling.py — Critical Sampling vs Random Sampling vs Interval FK
            for computing C-space-box interval AABBs

Experiment design
=================
Three methods for computing the AABB envelope of each robot link over a
C-space box [q_lo, q_hi]:

  1. **Critical**  – Enumerate gradient-zero critical points (boundary
     combinations, kπ/2 values, coupled-joint constraints, manifold
     sampling) + L-BFGS-B local optimisation.
     Produces *tight* AABBs; most expensive.

  2. **Random**    – Uniform random sampling inside the box + boundary
     corners + L-BFGS-B optimisation.
     Cheaper per sample; quality depends on sample budget.

  3. **Interval**  – Pure interval arithmetic FK (conservative enclosure,
     no sampling).  Fastest; guaranteed to *contain* the true reachable
     set but may over-approximate significantly.

Ground truth
------------
We run the **random** sampling method with a very large sample budget
(``N_GT_SAMPLES``, default 100 000) + L-BFGS-B optimisation.  Because
the optimiser pushes extremes beyond what raw MC can find, this produces
a tighter inner approximation than naïve uniform Monte-Carlo.  We then
measure each method against this ground truth:

  - **Volume ratio** = method_volume / GT_volume
    (1.0 = perfect; >1 = conservative; <1 = tighter than GT reference)
  - **Excess** = (method_volume - GT_volume) / GT_volume
  - **Safety** = does the method AABB fully contain the GT AABB
    on every axis?  (Only interval FK is *guaranteed* safe.)
  - **Time** = wall-clock computation time

Box sizes
---------
  - small:  ±0.1 rad  (typical leaf node in SBF tree)
  - medium: ±0.5 rad
  - large:  ±1.0 rad
  - full:   full joint limits

Robots
------
  - 2DOF planar (configs/2dof_planar.json)
  - Panda 7DOF  (configs/panda.json)

Usage
-----
  cd c:\\Users\\TIAN\\Documents\\safeboxforest
  python experiments/exp_aabb_sampling.py [--quick] [--save-dir DIR]

Requirements
------------
  numpy, scipy, matplotlib  (all in conda base)
  v4 src must be importable — the script adds it to sys.path.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# ── path setup ──────────────────────────────────────────────────────────
_HERE = Path(__file__).resolve().parent
_V4_SRC = Path(r"c:\Users\TIAN\Documents\box_aabb\v4\src")
if str(_V4_SRC) not in sys.path:
    sys.path.insert(0, str(_V4_SRC))
_V4_ROOT = _V4_SRC.parent
if str(_V4_ROOT) not in sys.path:
    sys.path.insert(0, str(_V4_ROOT))

from aabb.robot import Robot, load_robot
from aabb.calculator import AABBCalculator
from aabb.interval_fk import compute_interval_aabb
from aabb.models import AABBEnvelopeResult, LinkAABBInfo

# ════════════════════════════════════════════════════════════════════════
#  Configuration
# ════════════════════════════════════════════════════════════════════════

N_GT_SAMPLES = 100_000      # samples for ground truth (random + L-BFGS-B)
N_RANDOM_SAMPLES = 10_000   # samples for 'random' strategy
N_SEEDS = 50                # repeated trials for timing stability

BOX_SIZES = {
    "small":  0.1,   # ±0.1 rad
    "medium": 0.5,
    "large":  1.0,
    "full":   None,  # full joint-limit range
}

METHODS = ["critical", "random", "interval"]


# ════════════════════════════════════════════════════════════════════════
#  Data classes
# ════════════════════════════════════════════════════════════════════════

@dataclass
class LinkResult:
    link_idx: int
    link_name: str
    method_min: List[float]   # AABB min from method
    method_max: List[float]
    gt_min: List[float]       # AABB min from ground truth
    gt_max: List[float]
    method_volume: float = 0.0
    gt_volume: float = 0.0
    volume_ratio: float = 0.0
    excess_pct: float = 0.0
    is_safe: bool = True      # method AABB fully contains GT AABB?


@dataclass
class TrialResult:
    robot_name: str
    box_name: str
    method: str
    intervals: List[Tuple[float, float]]
    n_samples: int = 0
    time_s: float = 0.0
    links: List[LinkResult] = field(default_factory=list)
    total_method_vol: float = 0.0
    total_gt_vol: float = 0.0
    total_volume_ratio: float = 0.0
    all_safe: bool = True
    # Cross-method containment: sampling ⊆ interval per-link
    containment_ok: bool = True
    containment_violations: List[str] = field(default_factory=list)


# ════════════════════════════════════════════════════════════════════════
#  Ground truth via high-sample-count random + L-BFGS-B optimisation
# ════════════════════════════════════════════════════════════════════════

def compute_ground_truth(
    robot: Robot,
    intervals: List[Tuple[float, float]],
    n_samples: int = N_GT_SAMPLES,
) -> Tuple[Dict[int, Tuple[np.ndarray, np.ndarray]], float]:
    """Run the random sampling method with a large sample budget as GT.

    Uses the same AABBCalculator + RandomStrategy pipeline (uniform
    sampling + L-BFGS-B) — this produces tighter inner-approximation
    AABBs than naïve MC because the optimiser pushes each extreme.

    Returns:
        (gt_dict, elapsed_s)
        gt_dict: link_idx (1-based) → (min_xyz, max_xyz)
    """
    calc = AABBCalculator(robot, robot_name=robot.name)
    t0 = time.perf_counter()
    env_result, _ = run_method(robot, calc, intervals, "random",
                               n_random=n_samples)
    elapsed = time.perf_counter() - t0

    gt: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
    for aabb_info in env_result.link_aabbs:
        if aabb_info.is_zero_length:
            continue
        gt[aabb_info.link_index] = (
            np.array(aabb_info.min_point),
            np.array(aabb_info.max_point),
        )
    return gt, elapsed


# ════════════════════════════════════════════════════════════════════════
#  Run a single method
# ════════════════════════════════════════════════════════════════════════

def run_method(
    robot: Robot,
    calc: AABBCalculator,
    intervals: List[Tuple[float, float]],
    method: str,
    n_random: int = N_RANDOM_SAMPLES,
) -> Tuple[AABBEnvelopeResult, float]:
    """Run one AABB computation and return (result, elapsed_sec)."""

    t0 = time.perf_counter()

    if method == "interval":
        result = calc.compute_envelope(
            intervals,
            method="interval",
        )
    elif method == "critical":
        result = calc.compute_envelope(
            intervals,
            method="numerical",
            sampling="critical",
        )
    elif method == "random":
        result = calc.compute_envelope(
            intervals,
            method="numerical",
            sampling="random",
            n_random_samples=n_random,
        )
    else:
        raise ValueError(f"Unknown method: {method}")

    elapsed = time.perf_counter() - t0
    return result, elapsed


# ════════════════════════════════════════════════════════════════════════
#  Compare methods for one (robot, box_size) pair
# ════════════════════════════════════════════════════════════════════════

def make_intervals(robot: Robot, half_width: Optional[float]) -> List[Tuple[float, float]]:
    """Create symmetric intervals centered at 0, clipped to joint limits."""
    n = robot.n_joints
    if robot.joint_limits is not None:
        jl = robot.joint_limits
    else:
        jl = [(-math.pi, math.pi)] * n

    if half_width is None:
        # full range
        return [(float(lo), float(hi)) for lo, hi in jl]

    result = []
    for lo, hi in jl:
        mid = (lo + hi) / 2.0
        r = min(half_width, (hi - lo) / 2.0)
        result.append((mid - r, mid + r))
    return result


def compare_one(
    robot: Robot,
    box_name: str,
    half_width: Optional[float],
    methods: List[str],
    n_random: int,
    n_gt: int,
) -> List[TrialResult]:
    """Run all methods on one (robot, box) and return TrialResults."""

    intervals = make_intervals(robot, half_width)
    calc = AABBCalculator(robot, robot_name=robot.name)

    # Ground truth: high-sample random + L-BFGS-B
    print(f"    GT (random {n_gt:,} + L-BFGS-B) ... ", end="", flush=True)
    gt, gt_elapsed = compute_ground_truth(robot, intervals, n_samples=n_gt)
    print(f"done ({gt_elapsed*1000:.0f} ms)")

    results: List[TrialResult] = []

    for method in methods:
        print(f"    {method:12s} ... ", end="", flush=True)
        env_result, elapsed = run_method(robot, calc, intervals, method,
                                         n_random=n_random)

        trial = TrialResult(
            robot_name=robot.name,
            box_name=box_name,
            method=method,
            intervals=intervals,
            n_samples=env_result.n_samples_evaluated,
            time_s=elapsed,
        )

        # Per-link comparison against GT
        for aabb_info in env_result.link_aabbs:
            li = aabb_info.link_index
            if aabb_info.is_zero_length:
                continue
            if li not in gt:
                continue

            gt_min, gt_max = gt[li]
            m_min = np.array(aabb_info.min_point)
            m_max = np.array(aabb_info.max_point)

            m_size = np.maximum(m_max - m_min, 1e-12)
            gt_size = np.maximum(gt_max - gt_min, 1e-12)
            m_vol = float(np.prod(m_size))
            gt_vol = float(np.prod(gt_size))

            safe = bool(np.all(m_min <= gt_min + 1e-6) and
                        np.all(m_max >= gt_max - 1e-6))

            lr = LinkResult(
                link_idx=li,
                link_name=aabb_info.link_name,
                method_min=m_min.tolist(),
                method_max=m_max.tolist(),
                gt_min=gt_min.tolist(),
                gt_max=gt_max.tolist(),
                method_volume=m_vol,
                gt_volume=gt_vol,
                volume_ratio=m_vol / gt_vol if gt_vol > 1e-15 else float("inf"),
                excess_pct=(m_vol - gt_vol) / gt_vol * 100.0 if gt_vol > 1e-15 else 0.0,
                is_safe=safe,
            )
            trial.links.append(lr)

        trial.total_method_vol = sum(lr.method_volume for lr in trial.links)
        trial.total_gt_vol = sum(lr.gt_volume for lr in trial.links)
        if trial.total_gt_vol > 1e-15:
            trial.total_volume_ratio = trial.total_method_vol / trial.total_gt_vol
        trial.all_safe = all(lr.is_safe for lr in trial.links)

        tag = "✓ safe" if trial.all_safe else "✗ UNSAFE"
        print(f"{elapsed*1000:8.1f} ms | vol_ratio={trial.total_volume_ratio:.4f} "
              f"| {tag}")

        results.append(trial)

    # ── Cross-method containment check ────────────────────────────────
    # For each link present in BOTH a sampling result and the interval
    # result, verify that sampling_AABB ⊆ interval_AABB.
    interval_trial = next((r for r in results if r.method == "interval"), None)
    if interval_trial is not None:
        # Build per-link index for interval result
        ivl_by_link: Dict[int, LinkResult] = {
            lr.link_idx: lr for lr in interval_trial.links
        }
        for trial in results:
            if trial.method == "interval":
                continue
            for lr in trial.links:
                if lr.link_idx not in ivl_by_link:
                    continue
                ivl_lr = ivl_by_link[lr.link_idx]
                for ax, ax_name in enumerate(["x", "y", "z"]):
                    # sampling_min should be >= interval_min
                    if lr.method_min[ax] < ivl_lr.method_min[ax] - 1e-6:
                        msg = (f"L{lr.link_idx} {ax_name}_min: "
                               f"{trial.method}={lr.method_min[ax]:.6f} < "
                               f"interval={ivl_lr.method_min[ax]:.6f}")
                        trial.containment_violations.append(msg)
                    # sampling_max should be <= interval_max
                    if lr.method_max[ax] > ivl_lr.method_max[ax] + 1e-6:
                        msg = (f"L{lr.link_idx} {ax_name}_max: "
                               f"{trial.method}={lr.method_max[ax]:.6f} > "
                               f"interval={ivl_lr.method_max[ax]:.6f}")
                        trial.containment_violations.append(msg)
            trial.containment_ok = len(trial.containment_violations) == 0
            if not trial.containment_ok:
                print(f"    ⚠ {trial.method} CONTAINMENT VIOLATIONS "
                      f"({len(trial.containment_violations)}):")
                for v in trial.containment_violations[:5]:
                    print(f"      {v}")
                if len(trial.containment_violations) > 5:
                    print(f"      ... and {len(trial.containment_violations)-5} more")
            else:
                print(f"    ✓ {trial.method} ⊆ interval: verified on "
                      f"{sum(1 for lr in trial.links if lr.link_idx in ivl_by_link)} "
                      f"shared links")

    return results


# ════════════════════════════════════════════════════════════════════════
#  Pretty-print summary tables
# ════════════════════════════════════════════════════════════════════════

def print_summary_table(all_results: List[TrialResult]) -> str:
    """Print a Markdown-style summary table; return as string."""

    lines: List[str] = []
    lines.append("")
    lines.append("=" * 90)
    lines.append("  AABB Sampling Method Comparison — Summary")
    lines.append("=" * 90)

    # Group by (robot, box)
    groups: Dict[Tuple[str, str], List[TrialResult]] = {}
    for tr in all_results:
        key = (tr.robot_name, tr.box_name)
        groups.setdefault(key, []).append(tr)

    for (rname, bname), trials in groups.items():
        # Find shared links across all methods for fair comparison
        link_sets = [set(lr.link_idx for lr in tr.links) for tr in trials]
        shared_links = link_sets[0]
        for ls in link_sets[1:]:
            shared_links &= ls

        lines.append(f"\n  Robot: {rname}  |  Box: {bname}  "
                      f"|  Shared links: {sorted(shared_links)}")
        lines.append(f"  {'Method':12s} | {'Samples':>8s} | {'Time (ms)':>10s} "
                      f"| {'Vol Ratio':>10s} | {'Shared VR':>10s} "
                      f"| {'Safe':>5s} | {'⊆ IFK':>6s}")
        lines.append("  " + "-" * 85)
        for tr in trials:
            # Shared-link volume ratio (only links common to ALL methods)
            shared_method = sum(lr.method_volume for lr in tr.links
                                if lr.link_idx in shared_links)
            shared_gt = sum(lr.gt_volume for lr in tr.links
                            if lr.link_idx in shared_links)
            shared_vr = shared_method / shared_gt if shared_gt > 1e-15 else 0.0

            excess = (tr.total_volume_ratio - 1.0) * 100.0
            safe_str = "✓" if tr.all_safe else "✗"
            cont_str = ("✓" if tr.containment_ok else "✗"
                        ) if tr.method != "interval" else "—"
            lines.append(
                f"  {tr.method:12s} | {tr.n_samples:8d} | "
                f"{tr.time_s*1000:10.1f} | "
                f"{tr.total_volume_ratio:10.4f} | "
                f"{shared_vr:10.4f} | {safe_str:>5s} | {cont_str:>6s}"
            )

    # Per-link detail for last group
    lines.append("\n" + "-" * 90)
    lines.append("  Per-link detail (last robot/box combination)")
    lines.append("-" * 90)

    last_key = list(groups.keys())[-1]
    last_trials = groups[last_key]
    # Header
    link_names = [lr.link_name for lr in last_trials[0].links]
    for li_idx, lname in enumerate(link_names):
        lines.append(f"\n  {lname}:")
        lines.append(f"    {'Method':12s} | {'GT Vol':>12s} | {'Method Vol':>12s} "
                      f"| {'Ratio':>8s} | {'Excess':>8s} | {'Safe':>5s}")
        for tr in last_trials:
            if li_idx < len(tr.links):
                lr = tr.links[li_idx]
                lines.append(
                    f"    {tr.method:12s} | {lr.gt_volume:12.6e} | "
                    f"{lr.method_volume:12.6e} | "
                    f"{lr.volume_ratio:8.4f} | "
                    f"{lr.excess_pct:7.1f}% | "
                    f"{'✓' if lr.is_safe else '✗':>5s}"
                )

    txt = "\n".join(lines)
    print(txt)
    return txt


# ════════════════════════════════════════════════════════════════════════
#  Plotting
# ════════════════════════════════════════════════════════════════════════

def plot_results(all_results: List[TrialResult], save_dir: Optional[str] = None):
    """Generate comparison plots with matplotlib."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("[WARN] matplotlib not available — skipping plots")
        return

    # Group by robot
    robots: Dict[str, Dict[str, List[TrialResult]]] = {}
    for tr in all_results:
        robots.setdefault(tr.robot_name, {}).setdefault(tr.box_name, []).append(tr)

    for rname, box_groups in robots.items():
        n_boxes = len(box_groups)
        fig, axes = plt.subplots(1, 3, figsize=(16, 5))
        fig.suptitle(f"{rname}: Critical vs Random vs Interval FK", fontsize=14)

        box_names = list(box_groups.keys())
        method_names = METHODS
        colors = {"critical": "#2196F3", "random": "#FF9800", "interval": "#4CAF50"}

        # ── Plot 1: Volume ratio ──
        ax = axes[0]
        x = np.arange(n_boxes)
        w = 0.25
        for mi, method in enumerate(method_names):
            vals = []
            for bn in box_names:
                trials = box_groups[bn]
                tr = next((t for t in trials if t.method == method), None)
                vals.append(tr.total_volume_ratio if tr else 0)
            ax.bar(x + mi * w, vals, w, label=method, color=colors[method])
        ax.set_xticks(x + w)
        ax.set_xticklabels(box_names, fontsize=9)
        ax.set_ylabel("Volume Ratio (method / GT)")
        ax.set_title("Tightness (lower = better)")
        ax.axhline(y=1.0, color="red", linestyle="--", linewidth=0.8, label="ideal (1.0)")
        ax.legend(fontsize=8)
        ax.set_ylim(bottom=0)

        # ── Plot 2: Time (ms, log scale) ──
        ax = axes[1]
        for mi, method in enumerate(method_names):
            vals = []
            for bn in box_names:
                trials = box_groups[bn]
                tr = next((t for t in trials if t.method == method), None)
                vals.append(tr.time_s * 1000 if tr else 0.01)
            ax.bar(x + mi * w, vals, w, label=method, color=colors[method])
        ax.set_xticks(x + w)
        ax.set_xticklabels(box_names, fontsize=9)
        ax.set_ylabel("Time (ms)")
        ax.set_title("Computation Time")
        ax.set_yscale("log")
        ax.legend(fontsize=8)

        # ── Plot 3: Per-link volume ratio (last box size) ──
        ax = axes[2]
        last_bn = box_names[-1]
        last_trials = box_groups[last_bn]
        n_links = max(len(tr.links) for tr in last_trials) if last_trials else 0
        ref_trial = max(last_trials, key=lambda t: len(t.links)) if last_trials else None
        x2 = np.arange(n_links)
        w2 = 0.25
        for mi, method in enumerate(method_names):
            tr = next((t for t in last_trials if t.method == method), None)
            if tr is None:
                continue
            vals = [lr.volume_ratio for lr in tr.links]
            if len(vals) < n_links:
                vals += [0.0] * (n_links - len(vals))
            ax.bar(x2 + mi * w2, vals[:n_links], w2, label=method, color=colors[method])
        ax.set_xticks(x2 + w2)
        link_labels = [f"L{ref_trial.links[i].link_idx}" for i in range(n_links)] if ref_trial and ref_trial.links else []
        ax.set_xticklabels(link_labels, fontsize=8)
        ax.set_ylabel("Volume Ratio")
        ax.set_title(f"Per-Link Ratio (box={last_bn})")
        ax.axhline(y=1.0, color="red", linestyle="--", linewidth=0.8)
        ax.legend(fontsize=8)

        plt.tight_layout()

        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
            fpath = os.path.join(save_dir, f"aabb_comparison_{rname.lower()}.png")
            plt.savefig(fpath, dpi=150)
            print(f"  Plot saved: {fpath}")
        plt.close(fig)

    # ── Heatmap: excess % per (link, method) for each box ──
    for rname, box_groups in robots.items():
        for bn, trials in box_groups.items():
            n_links = max(len(tr.links) for tr in trials) if trials else 0
            if n_links == 0:
                continue
            # Find trial with most links for labels
            ref_trial = max(trials, key=lambda t: len(t.links))
            data = np.zeros((len(trials), n_links))
            y_labels = []
            for ti, tr in enumerate(trials):
                y_labels.append(tr.method)
                for li, lr in enumerate(tr.links):
                    data[ti, li] = lr.excess_pct

            fig, ax = plt.subplots(figsize=(max(8, n_links), 3))
            im = ax.imshow(data, aspect="auto", cmap="YlOrRd")
            ax.set_xticks(range(n_links))
            x_labels = [f"L{ref_trial.links[i].link_idx}" for i in range(n_links)]
            ax.set_xticklabels(x_labels, fontsize=8)
            ax.set_yticks(range(len(trials)))
            ax.set_yticklabels(y_labels, fontsize=9)
            ax.set_title(f"{rname} | box={bn} — Excess volume %")
            fig.colorbar(im, ax=ax, label="Excess %")

            # Annotate cells
            for ti in range(len(trials)):
                for li in range(n_links):
                    val = data[ti, li]
                    color = "white" if val > 50 else "black"
                    ax.text(li, ti, f"{val:.1f}", ha="center", va="center",
                            fontsize=7, color=color)

            plt.tight_layout()
            if save_dir:
                fpath = os.path.join(save_dir,
                                     f"heatmap_{rname.lower()}_{bn}.png")
                plt.savefig(fpath, dpi=150)
                print(f"  Heatmap saved: {fpath}")
            plt.close(fig)

    # ── Axis-level comparison: per-axis error for ONE robot/box ──
    for rname, box_groups in robots.items():
        last_bn = list(box_groups.keys())[-1]
        last_trials = box_groups[last_bn]
        if not last_trials:
            continue

        # Use trial with the most links as reference
        ref_trial = max(last_trials, key=lambda t: len(t.links))
        n_links = len(ref_trial.links)
        if n_links == 0:
            continue
        axes_names = ["x", "y", "z"]

        fig, axs = plt.subplots(3, 1, figsize=(12, 9), sharex=True)
        fig.suptitle(f"{rname} | box={last_bn} — Per-axis AABB bounds", fontsize=13)

        for ai, aname in enumerate(axes_names):
            ax = axs[ai]
            for tr in last_trials:
                n_tr_links = len(tr.links)
                if n_tr_links == 0:
                    continue
                gt_lo = [tr.links[li].gt_min[ai] for li in range(n_tr_links)]
                gt_hi = [tr.links[li].gt_max[ai] for li in range(n_tr_links)]
                m_lo = [tr.links[li].method_min[ai] for li in range(n_tr_links)]
                m_hi = [tr.links[li].method_max[ai] for li in range(n_tr_links)]

                xs = range(n_tr_links)
                ls = "-" if tr.method != "interval" else "--"
                c = colors[tr.method]
                ax.plot(xs, m_lo, ls, color=c, alpha=0.7, label=f"{tr.method} lo")
                ax.plot(xs, m_hi, ls, color=c, alpha=0.7, label=f"{tr.method} hi")

            # GT band from ref trial
            gt_lo = [ref_trial.links[li].gt_min[ai] for li in range(n_links)]
            gt_hi = [ref_trial.links[li].gt_max[ai] for li in range(n_links)]
            ax.fill_between(range(n_links), gt_lo, gt_hi,
                            alpha=0.15, color="gray", label="GT")
            ax.set_ylabel(f"{aname} (m)")
            if ai == 0:
                ax.legend(fontsize=7, ncol=4, loc="upper right")

        link_labels = [f"L{ref_trial.links[i].link_idx}" for i in range(n_links)]
        axs[-1].set_xticks(range(n_links))
        axs[-1].set_xticklabels(link_labels, fontsize=8)
        axs[-1].set_xlabel("Link")

        plt.tight_layout()
        if save_dir:
            fpath = os.path.join(save_dir, f"axis_bounds_{rname.lower()}_{last_bn}.png")
            plt.savefig(fpath, dpi=150)
            print(f"  Axis-bounds plot saved: {fpath}")
        plt.close(fig)


# ════════════════════════════════════════════════════════════════════════
#  Random sample count sweep (random method only)
# ════════════════════════════════════════════════════════════════════════

def sweep_random_samples(
    robot: Robot,
    half_width: float,
    sample_counts: List[int],
    n_gt: int = N_GT_SAMPLES,
    save_dir: Optional[str] = None,
):
    """Sweep n_random_samples and plot convergence of volume ratio."""
    intervals = make_intervals(robot, half_width)
    calc = AABBCalculator(robot, robot_name=robot.name)

    gt, _ = compute_ground_truth(robot, intervals, n_samples=n_gt)

    # Also run critical and interval as reference lines
    env_crit, _ = run_method(robot, calc, intervals, "critical")
    env_ivl, _ = run_method(robot, calc, intervals, "interval")

    def _compute_ratio(env_result: AABBEnvelopeResult) -> float:
        """Volume ratio, only counting links present in the result."""
        total_vol = 0.0
        gt_total = 0.0
        for aabb_info in env_result.link_aabbs:
            li = aabb_info.link_index
            if aabb_info.is_zero_length or li not in gt:
                continue
            gt_min, gt_max = gt[li]
            size = np.array(aabb_info.max_point) - np.array(aabb_info.min_point)
            gt_size = np.maximum(gt_max - gt_min, 1e-12)
            total_vol += float(np.prod(np.maximum(size, 1e-12)))
            gt_total += float(np.prod(gt_size))
        return total_vol / gt_total if gt_total > 1e-15 else 1.0

    crit_ratio = _compute_ratio(env_crit)
    ivl_ratio = _compute_ratio(env_ivl)
    print(f"    Reference — critical: {crit_ratio:.4f} | interval: {ivl_ratio:.4f}")

    ratios = []
    times = []
    for ns in sample_counts:
        t0 = time.perf_counter()
        env, _ = run_method(robot, calc, intervals, "random", n_random=ns)
        elapsed = time.perf_counter() - t0

        ratio = _compute_ratio(env)
        ratios.append(ratio)
        times.append(elapsed * 1000)
        print(f"    n_samples={ns:8d} | ratio={ratio:.4f} | {elapsed*1000:.0f} ms")

    # Plot convergence
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax1 = plt.subplots(figsize=(10, 5))
        ax1.plot(sample_counts, ratios, "o-", color="#FF9800", label="Random",
                 linewidth=2, markersize=5)
        ax1.axhline(y=1.0, color="red", linestyle="--", linewidth=0.8,
                    label="ideal (1.0)")
        ax1.axhline(y=crit_ratio, color="#2196F3", linestyle="-.",
                    linewidth=1.5, label=f"Critical ({crit_ratio:.4f})")
        ax1.axhline(y=ivl_ratio, color="#4CAF50", linestyle=":",
                    linewidth=1.5, label=f"Interval ({ivl_ratio:.4f})")
        ax1.set_xlabel("Random samples")
        ax1.set_ylabel("Volume ratio (method / GT)")
        ax1.set_xscale("log")
        ax1.set_title(f"{robot.name}: Random sampling convergence (box ±{half_width} rad)")
        ax1.legend(loc="upper left")

        ax2 = ax1.twinx()
        ax2.plot(sample_counts, times, "s--", color="#9C27B0", alpha=0.6, label="Time (ms)")
        ax2.set_ylabel("Time (ms)")
        ax2.legend(loc="upper right")

        plt.tight_layout()
        if save_dir:
            os.makedirs(save_dir, exist_ok=True)
            fpath = os.path.join(save_dir,
                                 f"random_convergence_{robot.name.lower()}.png")
            plt.savefig(fpath, dpi=150)
            print(f"  Convergence plot saved: {fpath}")
        plt.close(fig)
    except ImportError:
        pass


# ════════════════════════════════════════════════════════════════════════
#  Save raw results to JSON
# ════════════════════════════════════════════════════════════════════════

def save_results_json(all_results: List[TrialResult], path: str):
    """Serialize results to JSON."""
    data = []
    for tr in all_results:
        entry = {
            "robot": tr.robot_name,
            "box": tr.box_name,
            "method": tr.method,
            "n_samples": tr.n_samples,
            "time_ms": tr.time_s * 1000,
            "total_volume_ratio": tr.total_volume_ratio,
            "total_method_vol": tr.total_method_vol,
            "total_gt_vol": tr.total_gt_vol,
            "all_safe": tr.all_safe,
            "containment_ok": tr.containment_ok,
            "containment_violations": tr.containment_violations,
            "links": [
                {
                    "link_idx": lr.link_idx,
                    "method_min": lr.method_min,
                    "method_max": lr.method_max,
                    "gt_min": lr.gt_min,
                    "gt_max": lr.gt_max,
                    "volume_ratio": lr.volume_ratio,
                    "excess_pct": lr.excess_pct,
                    "is_safe": lr.is_safe,
                }
                for lr in tr.links
            ],
        }
        data.append(entry)

    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    print(f"  Results saved: {path}")


# ════════════════════════════════════════════════════════════════════════
#  Main
# ════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Compare critical/random/interval AABB methods")
    parser.add_argument("--quick", action="store_true",
                        help="Quick mode: fewer samples, fewer box sizes")
    parser.add_argument("--save-dir", type=str, default=None,
                        help="Directory to save plots and results")
    parser.add_argument("--robots", nargs="+",
                        default=["2dof_planar", "panda"],
                        help="Robot names to test")
    parser.add_argument("--n-gt", type=int, default=N_GT_SAMPLES,
                        help=f"Ground truth samples (default {N_GT_SAMPLES})")
    parser.add_argument("--n-random", type=int, default=N_RANDOM_SAMPLES,
                        help=f"Random method samples (default {N_RANDOM_SAMPLES})")
    parser.add_argument("--sweep", action="store_true",
                        help="Also run random-sample-count sweep")
    args = parser.parse_args()

    if args.quick:
        box_sizes = {"small": 0.1, "large": 1.0}
        n_gt = 20_000
        n_random = 2_000
    else:
        box_sizes = BOX_SIZES
        n_gt = args.n_gt
        n_random = args.n_random

    save_dir = args.save_dir or str(_HERE / "results_aabb_sampling")

    all_results: List[TrialResult] = []

    for robot_name in args.robots:
        print(f"\n{'='*70}")
        print(f"  Robot: {robot_name}")
        print(f"{'='*70}")

        try:
            robot = load_robot(robot_name)
        except FileNotFoundError:
            print(f"  [SKIP] Robot config not found: {robot_name}")
            continue

        print(f"  {robot.n_joints} DOF | joints: {robot.name}")
        if robot.joint_limits:
            for i, (lo, hi) in enumerate(robot.joint_limits):
                print(f"    q{i}: [{lo:.4f}, {hi:.4f}]")

        for bname, hw in box_sizes.items():
            print(f"\n  --- Box: {bname} (±{hw if hw else 'full'} rad) ---")
            trials = compare_one(
                robot, bname, hw,
                methods=METHODS,
                n_random=n_random,
                n_gt=n_gt,
            )
            all_results.extend(trials)

    # Summary
    summary = print_summary_table(all_results)

    # Save JSON
    json_path = os.path.join(save_dir, "results.json")
    save_results_json(all_results, json_path)

    # Save summary text
    summary_path = os.path.join(save_dir, "summary.txt")
    os.makedirs(os.path.dirname(summary_path), exist_ok=True)
    with open(summary_path, "w", encoding="utf-8") as f:
        f.write(summary)
    print(f"  Summary saved: {summary_path}")

    # Plots
    print("\nGenerating plots ...")
    plot_results(all_results, save_dir=save_dir)

    # Optional: random convergence sweep
    if args.sweep:
        print("\n" + "=" * 70)
        print("  Random sample count sweep")
        print("=" * 70)
        sweep_counts = [100, 500, 1_000, 2_000, 5_000, 10_000,
                        20_000, 50_000, 100_000]
        for robot_name in args.robots:
            try:
                robot = load_robot(robot_name)
            except FileNotFoundError:
                continue
            print(f"\n  {robot.name}: sweeping random samples (box ±0.5 rad)")
            sweep_random_samples(robot, 0.5, sweep_counts,
                                 n_gt=n_gt, save_dir=save_dir)

    print("\nDone.")


if __name__ == "__main__":
    main()
