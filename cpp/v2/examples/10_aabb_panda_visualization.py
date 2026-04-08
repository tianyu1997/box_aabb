#!/usr/bin/env python3
"""
Example 10: Panda 7-DOF AABB Envelope Comparison (Interactive HTML)
====================================================================

Compares three AABB estimation methods for a Panda 7-DOF robot arm:
  1. Interval FK         - Conservative interval arithmetic on DH chain
  2. Critical Sampling   - Gradient-zero critical-point enumeration
  3. Random Sampling     - Uniform random samples + boundary corners

Outputs are saved to:  safeboxforest/v2/results/aabb_panda_<timestamp>/
  - aabb_panda_interval.html
  - aabb_panda_critical.html
  - aabb_panda_random.html
  - aabb_panda_comparison.html  (three-panel side-by-side)

Usage:
    python 10_aabb_panda_visualization.py [--seed SEED]
"""
from __future__ import annotations

import sys
import time
import argparse
from pathlib import Path

import numpy as np

# Allow importing from the sibling python/ directory
sys.path.insert(0, str(Path(__file__).parent.parent / "python"))

from forest_viz import (
    PANDA_DH_PARAMS,
    PANDA_JOINT_LIMITS,
    PANDA_TOOL_D,
    dh_fk_positions,
    compute_aabbs_interval_fk,
    compute_aabbs_critical,
    compute_aabbs_random,
    random_joint_subintervals,
    random_workspace_obstacles,
    build_3d_aabb_figure,
    build_3d_comparison_figure,
    make_output_dir,
    _LINK_COLORS_DEFAULT,
)

#  Tunable parameters 
N_ARM_DISPLAY   = 40      # ghost arm configurations shown in visualizations
N_RANDOM        = 5000    # random samples for the random-sampling method
SKIP_FRAMES     = {0}     # frame indices hidden from AABB display (0 = base)
# 


def fk(q: np.ndarray) -> np.ndarray:
    """Forward kinematics wrapper: q(7,)  positions (9, 3)."""
    return dh_fk_positions(PANDA_DH_PARAMS, q)


def generate(seed: int | None = None) -> None:
    rng = np.random.default_rng(seed)

    # --- Random scene --------------------------------------------------------
    q_intervals = random_joint_subintervals(PANDA_JOINT_LIMITS, rng,
                                            width_lo=0.3, width_hi=0.6)
    obstacles   = random_workspace_obstacles(rng, n=3)

    mid_q   = np.array([(lo + hi) / 2 for lo, hi in q_intervals])
    mid_pos = fk(mid_q)

    # Sample ghost arm configurations
    arm_samples = [
        fk(np.array([rng.uniform(lo, hi) for lo, hi in q_intervals]))
        for _ in range(N_ARM_DISPLAY)
    ]

    print("Joint intervals (rad):")
    for i, (lo, hi) in enumerate(q_intervals):
        print(f"  q{i+1}: [{lo:.3f}, {hi:.3f}]")
    print()

    # --- Compute AABBs -------------------------------------------------------
    print("Computing Interval FK ... ", end="", flush=True)
    t0 = time.perf_counter()
    aabbs_iv = compute_aabbs_interval_fk(PANDA_DH_PARAMS, q_intervals)
    t_iv = time.perf_counter() - t0
    print(f"{t_iv*1000:.0f} ms")

    print("Computing Critical sampling ... ", end="", flush=True)
    t0 = time.perf_counter()
    aabbs_cr = compute_aabbs_critical(PANDA_DH_PARAMS, q_intervals)
    t_cr = time.perf_counter() - t0
    print(f"{t_cr*1000:.0f} ms")

    print(f"Computing Random sampling ({N_RANDOM} pts) ... ", end="", flush=True)
    t0 = time.perf_counter()
    aabbs_rn = compute_aabbs_random(fk, q_intervals, n_samples=N_RANDOM)
    t_rn = time.perf_counter() - t0
    print(f"{t_rn*1000:.0f} ms")

    methods_data = [
        ("Interval FK (Conservative)", aabbs_iv, "#4CAF50", t_iv),
        ("Critical Sampling (Tight)",  aabbs_cr, "#2196F3", t_cr),
        (f"Random Sampling ({N_RANDOM})", aabbs_rn, "#FF9800", t_rn),
    ]

    # --- Save outputs --------------------------------------------------------
    out_dir = make_output_dir(Path(__file__).parent.parent / "results",
                              prefix="aabb_panda")
    print(f"\nOutput directory: {out_dir}\n")

    labels = ["interval", "critical", "random"]
    for (name, aabbs, color, tc), label in zip(methods_data, labels):
        fig = build_3d_aabb_figure(
            method_name=name, aabbs=aabbs,
            arm_samples_pos=arm_samples, mid_pos=mid_pos,
            obstacles=obstacles, method_color=color, t_comp=tc,
            link_colors=_LINK_COLORS_DEFAULT, skip_frames=SKIP_FRAMES,
        )
        fname = f"aabb_panda_{label}.html"
        fig.write_html(str(out_dir / fname), include_plotlyjs="cdn")
        print(f"  Saved: {fname}")

    fig_cmp = build_3d_comparison_figure(
        methods_data=methods_data,
        arm_samples_pos=arm_samples, mid_pos=mid_pos,
        obstacles=obstacles,
        link_colors=_LINK_COLORS_DEFAULT, skip_frames=SKIP_FRAMES,
    )
    fig_cmp.write_html(str(out_dir / "aabb_panda_comparison.html"),
                       include_plotlyjs="cdn")
    print("  Saved: aabb_panda_comparison.html")
    print(f"\nDone. Output: {out_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Panda AABB comparison (HTML)")
    parser.add_argument("--seed", type=int, default=None,
                        help="RNG seed for reproducible scene (default: random)")
    args = parser.parse_args()
    generate(seed=args.seed)
