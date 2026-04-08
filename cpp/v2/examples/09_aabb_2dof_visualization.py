#!/usr/bin/env python3
"""
Example 09: 2-DOF Planar Robot AABB Comparison (matplotlib PNG)
===============================================================

Compares three AABB methods for a 2-joint planar robot:
  1. Interval FK       - conservative interval arithmetic
  2. Critical Sampling - pi/2 boundary angles
  3. Random Sampling   - uniform random + corner samples

Outputs are saved to:  safeboxforest/v2/results/aabb_2dof_<timestamp>/
  - aabb_2dof_comparison.png

Usage:
    python 09_aabb_2dof_visualization.py [--seed SEED]
"""
from __future__ import annotations

import sys
import math
import time
import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mp

sys.path.insert(0, str(Path(__file__).parent.parent / "python"))

from forest_viz import (
    compute_aabbs_interval_planar_2dof,
    compute_aabbs_critical_2dof,
    compute_aabbs_random,
    random_joint_subintervals,
    draw_aabb_2d,
    draw_arm_2d,
    aabb_volume,
    make_output_dir,
)

#  Robot parameters 
L1, L2 = 1.0, 0.8   # link lengths (m)

JOINT_LIMITS_2DOF = [
    (-math.pi, math.pi),
    (-math.pi, math.pi),
]

N_RANDOM      = 2000
N_ARM_DISPLAY = 60
# 


def fk_2dof(q: np.ndarray) -> np.ndarray:
    """
    2-DOF planar FK.  q shape (2,)
    Returns (3, 2): [base, elbow, ee] in 2D.
    """
    q1, q2 = float(q[0]), float(q[1])
    x1 = L1 * math.cos(q1)
    y1 = L1 * math.sin(q1)
    x2 = x1 + L2 * math.cos(q1 + q2)
    y2 = y1 + L2 * math.sin(q1 + q2)
    return np.array([[0.0, 0.0], [x1, y1], [x2, y2]])


def _fk_for_random(q):
    """Wrapper that returns (3, 3) so compute_aabbs_random works with x_min/y_min keys."""
    pos_2d = fk_2dof(q)
    return np.hstack([pos_2d, np.zeros((3, 1))])


def generate(seed: int | None = None) -> None:
    rng = np.random.default_rng(seed)

    # --- Random joint intervals within limits --------------------------------
    q_intervals = random_joint_subintervals(JOINT_LIMITS_2DOF, rng,
                                            width_lo=0.2, width_hi=0.5)
    print("Joint intervals (rad):")
    for i, (lo, hi) in enumerate(q_intervals):
        print(f"  q{i+1}: [{lo:.3f}, {hi:.3f}]")

    mid_q = np.array([(lo + hi) / 2 for lo, hi in q_intervals])

    # --- Compute AABBs -------------------------------------------------------
    print("Computing Interval FK ...    ", end="", flush=True)
    t0 = time.perf_counter()
    aabbs_iv = compute_aabbs_interval_planar_2dof((L1, L2), q_intervals)
    t_iv = time.perf_counter() - t0
    print(f"{t_iv*1000:.1f} ms")

    print("Computing Critical sampling ... ", end="", flush=True)
    t0 = time.perf_counter()
    aabbs_cr = compute_aabbs_critical_2dof((L1, L2), q_intervals)
    t_cr = time.perf_counter() - t0
    print(f"{t_cr*1000:.1f} ms")

    print(f"Computing Random sampling ({N_RANDOM} pts) ... ", end="", flush=True)
    t0 = time.perf_counter()
    aabbs_rn = compute_aabbs_random(_fk_for_random, q_intervals, n_samples=N_RANDOM)
    t_rn = time.perf_counter() - t0
    print(f"{t_rn*1000:.1f} ms")

    methods = [
        ("Interval FK\n(Conservative)", aabbs_iv, "#4CAF50", t_iv),
        ("Critical Sampling\n(Tight)",  aabbs_cr, "#2196F3", t_cr),
        (f"Random ({N_RANDOM})",        aabbs_rn, "#FF9800", t_rn),
    ]
    frame_colors = ["#E53935", "#1565C0"]  # Elbow, EE

    # --- Plot ----------------------------------------------------------------
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    fig.suptitle(
        f"2-DOF Planar Robot  AABB Comparison\n"
        f"q1 in [{q_intervals[0][0]:.2f},{q_intervals[0][1]:.2f}]  "
        f"q2 in [{q_intervals[1][0]:.2f},{q_intervals[1][1]:.2f}]",
        fontsize=12,
    )

    for ax, (title, aabbs, color, tc) in zip(axes, methods):
        ax.set_aspect("equal")
        ax.set_title(f"{title}\n({tc*1000:.1f} ms)", fontsize=10)
        ax.axhline(0, color="#ccc", lw=0.5)
        ax.axvline(0, color="#ccc", lw=0.5)
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")

        # Ghost arms
        for _ in range(N_ARM_DISPLAY):
            q_r = np.array([rng.uniform(lo, hi) for lo, hi in q_intervals])
            draw_arm_2d(ax, fk_2dof(q_r), alpha=0.08, color="#999")

        # Midpoint arm (bold)
        pos_mid = fk_2dof(mid_q)
        ax.plot(pos_mid[:, 0], pos_mid[:, 1], "o-", color="#333",
                linewidth=2.5, markersize=5, zorder=5, label="Mid config")

        # AABBs  one per link frame (frame 1, 2)
        link_keys = ["x", "y", "z"]  # dim 0=x, 1=y
        for i, aabb in enumerate(aabbs):
            lc = frame_colors[i % len(frame_colors)]
            vol = aabb.get("x_max", 0) - aabb.get("x_min", 0)   # 2D "area"
            area = ((aabb["x_max"] - aabb["x_min"]) *
                    (aabb["y_max"] - aabb["y_min"]))
            draw_aabb_2d(ax, aabb, color=lc, edgecolor=color, linewidth=1.6,
                         label=f"Link {i+1} area={area:.4f}")

        ax.legend(fontsize=8, loc="upper right")
        # Auto-range with padding
        p = 0.15
        ax.set_xlim(-(L1+L2+p), L1+L2+p)
        ax.set_ylim(-(L1+L2+p), L1+L2+p)

    plt.tight_layout()

    out_dir = make_output_dir(Path(__file__).parent.parent / "results",
                              prefix="aabb_2dof")
    fpath = out_dir / "aabb_2dof_comparison.png"
    plt.savefig(str(fpath), dpi=150, bbox_inches="tight")
    plt.close()
    print(f"\nSaved: {fpath}")
    print(f"Done.  Output: {out_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="2-DOF AABB comparison (PNG)")
    parser.add_argument("--seed", type=int, default=None,
                        help="RNG seed (default: random)")
    args = parser.parse_args()
    generate(seed=args.seed)
