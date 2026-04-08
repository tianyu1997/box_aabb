#!/usr/bin/env python3
"""
Example 12: 2-DOF Planar Robot OBB vs AABB Comparison (matplotlib PNG)
=======================================================================

Compares the 2D OBB (Oriented Bounding Box) against the three AABB methods
for a 2-joint planar robot:
  1. AABB Interval FK   - conservative interval arithmetic
  2. AABB Critical      - pi/2 critical-angle enumeration
  3. AABB Random        - uniform random samples
  4. OBB Critical (PCA) - PCA on critical-angle point cloud (tightest)

Layout: 2 rows x 3 cols
  Top row: per-method AABB (link 1 + link 2), individual subplots
  Bottom panel: AABB Critical vs OBB Critical side-by-side (zoomed detail)

Outputs:  safeboxforest/v2/results/obb_2dof_<timestamp>/
  - obb_2dof_comparison.png    (full 4-method panel)
  - obb_2dof_detail.png        (AABB Critical vs OBB Critical detail)

Usage:
    python 12_obb_2dof_visualization.py [--seed SEED]
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
from matplotlib.patches import FancyArrowPatch

sys.path.insert(0, str(Path(__file__).parent.parent / "python"))

from forest_viz import (
    compute_aabbs_interval_planar_2dof,
    compute_aabbs_critical_2dof,
    compute_aabbs_random,
    compute_obbs_critical_2dof,
    _obb2d_corners,
    obb_area_2d,
    aabb_volume,
    random_joint_subintervals,
    draw_aabb_2d,
    draw_obb_2d,
    draw_arm_2d,
    make_output_dir,
)

# ── Robot parameters ─────────────────────────────────────────────────────────
L1, L2 = 1.0, 0.8
JOINT_LIMITS_2DOF = [(-math.pi, math.pi), (-math.pi, math.pi)]
N_RANDOM      = 2000
N_ARM_DISPLAY = 80
LINK_COLORS   = ["#E53935", "#1565C0"]   # link 1, link 2
# ─────────────────────────────────────────────────────────────────────────────


def fk_2dof(q: np.ndarray) -> np.ndarray:
    """Returns (3, 2): [base, elbow, EE]."""
    q1, q2 = float(q[0]), float(q[1])
    x1 = L1 * math.cos(q1);          y1 = L1 * math.sin(q1)
    x2 = x1 + L2 * math.cos(q1+q2); y2 = y1 + L2 * math.sin(q1+q2)
    return np.array([[0., 0.], [x1, y1], [x2, y2]])


def _fk_3d(q):
    """Adapter: returns (3, 3) so compute_aabbs_random works."""
    pos = fk_2dof(q)
    return np.hstack([pos, np.zeros((3, 1))])


def _draw_obb_axes(ax, obb2d, color, scale=0.18):
    """Draw the two principal axes of a 2D OBB as arrows at its center."""
    c   = obb2d["center"]
    axs = obb2d["axes"]
    h   = obb2d["half"]
    wc  = c[0]*axs[0] + c[1]*axs[1]
    for j in range(2):
        length = min(float(h[j]), scale)
        dx, dy = float(axs[j, 0]) * length, float(axs[j, 1]) * length
        ax.annotate("", xy=(wc[0]+dx, wc[1]+dy), xytext=(wc[0], wc[1]),
                    arrowprops=dict(arrowstyle="->", color=color,
                                   lw=1.4, mutation_scale=10))


def _aabb_area_2d(aabb):
    return (aabb["x_max"] - aabb["x_min"]) * (aabb["y_max"] - aabb["y_min"])


def _setup_ax(ax, q_intervals, title, tc_ms):
    p = 0.15
    ax.set_aspect("equal")
    ax.set_title(f"{title}\n({tc_ms:.1f} ms)", fontsize=10, pad=4)
    ax.axhline(0, color="#ddd", lw=0.5, zorder=0)
    ax.axvline(0, color="#ddd", lw=0.5, zorder=0)
    ax.set_xlabel("x (m)", fontsize=8)
    ax.set_ylabel("y (m)", fontsize=8)
    ax.set_xlim(-(L1+L2+p), L1+L2+p)
    ax.set_ylim(-(L1+L2+p), L1+L2+p)
    ax.tick_params(labelsize=7)


def generate(seed: int | None = None) -> None:
    rng = np.random.default_rng(seed)

    # --- Joint intervals ----------------------------------------------------
    q_intervals = random_joint_subintervals(JOINT_LIMITS_2DOF, rng,
                                            width_lo=0.2, width_hi=0.5)
    mid_q = np.array([(lo+hi)/2 for lo, hi in q_intervals])

    print("Joint intervals (rad):")
    for i, (lo, hi) in enumerate(q_intervals):
        print(f"  q{i+1}: [{lo:.3f}, {hi:.3f}]")
    print()

    # --- Compute envelopes --------------------------------------------------
    print("Computing AABB Interval FK   ... ", end="", flush=True)
    t0 = time.perf_counter()
    aabbs_iv = compute_aabbs_interval_planar_2dof((L1, L2), q_intervals)
    t_iv = time.perf_counter() - t0
    print(f"{t_iv*1000:.2f} ms")

    print("Computing AABB Critical       ... ", end="", flush=True)
    t0 = time.perf_counter()
    aabbs_cr = compute_aabbs_critical_2dof((L1, L2), q_intervals)
    t_cr = time.perf_counter() - t0
    print(f"{t_cr*1000:.2f} ms")

    print(f"Computing AABB Random ({N_RANDOM} pts)  ... ", end="", flush=True)
    t0 = time.perf_counter()
    aabbs_rn = compute_aabbs_random(_fk_3d, q_intervals, n_samples=N_RANDOM)
    t_rn = time.perf_counter() - t0
    print(f"{t_rn*1000:.2f} ms")

    print("Computing OBB  Critical (PCA) ... ", end="", flush=True)
    t0 = time.perf_counter()
    obbs_cr = compute_obbs_critical_2dof((L1, L2), q_intervals)
    t_obb = time.perf_counter() - t0
    print(f"{t_obb*1000:.2f} ms")

    # --- Summary table -------------------------------------------------------
    print()
    print(f"{'Link':<22}  {'AABB_IV':>9}  {'AABB_CR':>9}  {'OBB_CR':>9}  {'OBB/AABB_CR':>11}")
    print("-" * 68)
    for a_iv, a_cr, o in zip(aabbs_iv, aabbs_cr, obbs_cr):
        a_iv_a = _aabb_area_2d(a_iv)
        a_cr_a = _aabb_area_2d(a_cr)
        o_a    = obb_area_2d(o)
        ratio = o_a / a_cr_a if a_cr_a > 1e-10 else float("nan")
        print(f"{a_cr['label']:<22}  {a_iv_a:>9.4f}  {a_cr_a:>9.4f}"
              f"  {o_a:>9.4f}  {ratio:>11.3f}")
    tot_aabb = sum(_aabb_area_2d(a) for a in aabbs_cr)
    tot_obb  = sum(obb_area_2d(o)   for o in obbs_cr)
    if tot_aabb > 0:
        print(f"\nOBB total area reduction vs AABB Critical: {100*(1-tot_obb/tot_aabb):.1f}%")
    print()

    # --- Ghost arm cloud (shared across subplots) ----------------------------
    arm_samples = [
        fk_2dof(np.array([rng.uniform(lo, hi) for lo, hi in q_intervals]))
        for _ in range(N_ARM_DISPLAY)
    ]
    pos_mid = fk_2dof(mid_q)

    # ─── Figure 1: 4-method comparison (2 x 2 layout) ────────────────────────
    fig, axes = plt.subplots(2, 2, figsize=(12, 11))
    supq = (f"2-DOF Planar Robot  —  OBB vs AABB Envelope Comparison\n"
            f"q1 \u2208 [{q_intervals[0][0]:.2f}, {q_intervals[0][1]:.2f}] rad,  "
            f"q2 \u2208 [{q_intervals[1][0]:.2f}, {q_intervals[1][1]:.2f}] rad")
    fig.suptitle(supq, fontsize=12, y=0.995)

    methods = [
        ("AABB Interval FK\n(Conservative)", aabbs_iv, "#4CAF50", t_iv*1000,
         "aabb"),
        ("AABB Critical\n(Tight)",           aabbs_cr, "#2196F3", t_cr*1000,
         "aabb"),
        (f"AABB Random ({N_RANDOM})",        aabbs_rn, "#FF9800", t_rn*1000,
         "aabb"),
        ("OBB Critical (PCA)\n(Tighter)",   obbs_cr,  "#9C27B0", t_obb*1000,
         "obb"),
    ]

    for idx, (title, envs, edge_color, tc_ms, kind) in enumerate(methods):
        ax = axes[idx // 2][idx % 2]
        _setup_ax(ax, q_intervals, title, tc_ms)

        # Ghost arms
        for pos in arm_samples:
            draw_arm_2d(ax, pos, alpha=0.06, color="#999", linewidth=0.8)

        # Mid config arm
        ax.plot(pos_mid[:, 0], pos_mid[:, 1], "o-", color="#222",
                linewidth=2.5, markersize=5, zorder=6, label="Mid config")

        # Envelopes
        for i, env in enumerate(envs):
            lc = LINK_COLORS[i % len(LINK_COLORS)]
            if kind == "aabb":
                area = _aabb_area_2d(env)
                draw_aabb_2d(ax, env, color=lc, edgecolor=edge_color,
                             linewidth=1.8,
                             label=f"Link {i+1}  area={area:.4f} m\u00b2")
            else:
                area = obb_area_2d(env)
                draw_obb_2d(ax, env, color=lc, edgecolor=edge_color,
                            linewidth=1.8,
                            label=f"Link {i+1}  area={area:.4f} m\u00b2")
                _draw_obb_axes(ax, env, edge_color, scale=0.15)

        ax.legend(fontsize=8, loc="upper right",
                  framealpha=0.9, borderpad=0.5)

    plt.tight_layout(rect=[0, 0, 1, 0.985])

    out_dir = make_output_dir(Path(__file__).parent.parent / "results",
                              prefix="obb_2dof")
    fpath1 = out_dir / "obb_2dof_comparison.png"
    fig.savefig(str(fpath1), dpi=150, bbox_inches="tight")
    plt.close(fig)
    print(f"Saved: {fpath1}")

    # ─── Figure 2: detail AABB Critical vs OBB Critical ──────────────────────
    fig2, (ax_a, ax_o) = plt.subplots(1, 2, figsize=(11, 6))
    fig2.suptitle(
        "2-DOF Planar Robot  —  AABB Critical vs OBB Critical (detail)\n"
        f"q1 \u2208 [{q_intervals[0][0]:.2f}, {q_intervals[0][1]:.2f}],  "
        f"q2 \u2208 [{q_intervals[1][0]:.2f}, {q_intervals[1][1]:.2f}]",
        fontsize=12,
    )

    for ax, (envs, edge_color, tc_ms, kind, title) in zip(
        [ax_a, ax_o],
        [
            (aabbs_cr, "#2196F3", t_cr*1000, "aabb",
             f"AABB Critical  (area={sum(_aabb_area_2d(a) for a in aabbs_cr):.4f} m\u00b2)"),
            (obbs_cr,  "#9C27B0", t_obb*1000, "obb",
             f"OBB Critical / PCA  (area={sum(obb_area_2d(o) for o in obbs_cr):.4f} m\u00b2)"),
        ],
    ):
        _setup_ax(ax, q_intervals, title, tc_ms)
        for pos in arm_samples:
            draw_arm_2d(ax, pos, alpha=0.07, color="#aaa", linewidth=0.8)
        ax.plot(pos_mid[:, 0], pos_mid[:, 1], "o-", color="#222",
                linewidth=2.8, markersize=6, zorder=6, label="Mid config")
        for i, env in enumerate(envs):
            lc = LINK_COLORS[i % len(LINK_COLORS)]
            if kind == "aabb":
                area = _aabb_area_2d(env)
                # Draw individual link box
                draw_aabb_2d(ax, env, color=lc, alpha=0.18,
                             edgecolor=edge_color, linewidth=2.0,
                             label=f"Link {i+1}  {area:.4f} m\u00b2")
                # Add dimension annotations
                x0, x1 = env["x_min"], env["x_max"]
                y0, y1 = env["y_min"], env["y_max"]
                ax.annotate("", xy=(x1, (y0+y1)/2), xytext=(x0, (y0+y1)/2),
                            arrowprops=dict(arrowstyle="<->", color=lc, lw=1.2))
                ax.annotate("", xy=((x0+x1)/2, y1), xytext=((x0+x1)/2, y0),
                            arrowprops=dict(arrowstyle="<->", color=lc, lw=1.2))
            else:
                area = obb_area_2d(env)
                draw_obb_2d(ax, env, color=lc, alpha=0.18,
                            edgecolor=edge_color, linewidth=2.0,
                            label=f"Link {i+1}  {area:.4f} m\u00b2")
                _draw_obb_axes(ax, env, edge_color, scale=0.20)
        ax.legend(fontsize=9, loc="upper right", framealpha=0.9)

    fig2.tight_layout()
    fpath2 = out_dir / "obb_2dof_detail.png"
    fig2.savefig(str(fpath2), dpi=150, bbox_inches="tight")
    plt.close(fig2)
    print(f"Saved: {fpath2}")
    print(f"\nDone. Output: {out_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="2-DOF planar OBB vs AABB comparison (PNG)")
    parser.add_argument("--seed", type=int, default=42,
                        help="RNG seed (default: 42)")
    args = parser.parse_args()
    generate(seed=args.seed)
