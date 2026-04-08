#!/usr/bin/env python3
"""
Example 11: Panda 7-DOF OBB vs AABB Envelope Comparison (Interactive HTML)
===========================================================================

Compares the tight OBB (Oriented Bounding Box) envelope against the three
AABB methods from Example 10:
  1. AABB Interval FK  - conservative AABB via interval arithmetic (fast)
  2. AABB Critical     - gradient-zero critical-point enumeration (tight)
  3. AABB Random       - uniform random samples (reference)
  4. OBB Critical      - PCA on critical-point cloud 鈫?tight OBB per link

The OBB is computed by enumerating the same critical joint angles as the
critical AABB, collecting the point cloud {parent, child} endpoint positions
for each link, and finding the minimum bounding OBB via PCA.  This is
guaranteed to be tighter (鈮? than the critical AABB in volume.

Outputs saved to:  safeboxforest/v2/results/obb_panda_<timestamp>/
  - obb_panda_obb.html          (OBB Critical standalone)
  - obb_panda_aabb_cr.html      (AABB Critical standalone)
  - obb_panda_2panel.html       (AABB Critical  vs  OBB Critical)
  - obb_panda_4panel.html       (all three AABB methods  +  OBB Critical)

Usage:
    python 11_obb_panda_visualization.py [--seed SEED] [--samples N]
"""
from __future__ import annotations

import sys
import time
import argparse
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "python"))

from forest_viz import (
    PANDA_DH_PARAMS,
    PANDA_JOINT_LIMITS,
    dh_fk_positions,
    compute_aabbs_interval_fk,
    compute_aabbs_critical,
    compute_aabbs_random,
    compute_obbs_critical,
    aabb_volume,
    obb_volume,
    random_joint_subintervals,
    random_workspace_obstacles,
    build_3d_aabb_figure,
    build_3d_obb_figure,
    build_3d_obb_aabb_comparison_figure,
    make_output_dir,
    _LINK_COLORS_DEFAULT,
)

# 鈹€鈹€ Tunable parameters 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
N_ARM_DISPLAY = 40          # ghost arm samples shown in each panel
N_RANDOM      = 5000        # random-sampling method sample count
SKIP_FRAMES   = {0}         # frame indices hidden (0 = base origin)
# 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€


def fk(q: np.ndarray) -> np.ndarray:
    return dh_fk_positions(PANDA_DH_PARAMS, q)


def generate(seed: int | None = None, n_random: int = N_RANDOM) -> None:
    rng = np.random.default_rng(seed)

    # --- Random scene --------------------------------------------------------
    q_intervals = random_joint_subintervals(PANDA_JOINT_LIMITS, rng,
                                            width_lo=0.3, width_hi=0.6)
    obstacles   = random_workspace_obstacles(rng, n=3)

    mid_q   = np.array([(lo + hi) / 2 for lo, hi in q_intervals])
    mid_pos = fk(mid_q)
    arm_samples = [
        fk(np.array([rng.uniform(lo, hi) for lo, hi in q_intervals]))
        for _ in range(N_ARM_DISPLAY)
    ]

    print("Joint intervals (rad):")
    for i, (lo, hi) in enumerate(q_intervals):
        print(f"  q{i+1}: [{lo:.3f}, {hi:.3f}]")
    print()

    # --- Compute envelopes ---------------------------------------------------
    print("Computing AABB Interval FK   ... ", end="", flush=True)
    t0 = time.perf_counter()
    aabbs_iv = compute_aabbs_interval_fk(PANDA_DH_PARAMS, q_intervals)
    t_iv = time.perf_counter() - t0
    v_iv = sum(aabb_volume(a) for a in aabbs_iv if a["frame"] not in SKIP_FRAMES)
    print(f"{t_iv*1000:.1f} ms   total vol = {v_iv:.4f} m^3")

    print("Computing AABB Critical       ... ", end="", flush=True)
    t0 = time.perf_counter()
    aabbs_cr = compute_aabbs_critical(PANDA_DH_PARAMS, q_intervals)
    t_cr = time.perf_counter() - t0
    v_cr = sum(aabb_volume(a) for a in aabbs_cr if a["frame"] not in SKIP_FRAMES)
    print(f"{t_cr*1000:.1f} ms   total vol = {v_cr:.4f} m^3")

    print(f"Computing AABB Random ({n_random} pts)  ... ", end="", flush=True)
    t0 = time.perf_counter()
    aabbs_rn = compute_aabbs_random(fk, q_intervals, n_samples=n_random)
    t_rn = time.perf_counter() - t0
    v_rn = sum(aabb_volume(a) for a in aabbs_rn if a["frame"] not in SKIP_FRAMES)
    print(f"{t_rn*1000:.1f} ms   total vol = {v_rn:.4f} m^3")

    print("Computing OBB  Critical (PCA) ... ", end="", flush=True)
    t0 = time.perf_counter()
    obbs_cr = compute_obbs_critical(PANDA_DH_PARAMS, q_intervals)
    t_obb = time.perf_counter() - t0
    v_obb = sum(obb_volume(o) for o in obbs_cr if o["frame"] not in SKIP_FRAMES)
    print(f"{t_obb*1000:.1f} ms   total vol = {v_obb:.4f} m^3")

    # --- Per-link comparison -------------------------------------------------
    print()
    print(f"{'Link':<8}  {'AABB_IV':>9}  {'AABB_CR':>9}  {'OBB_CR':>9}  {'OBB/AABB_CR':>11}")
    print("-" * 58)
    for a_iv, a_cr, o in zip(aabbs_iv, aabbs_cr, obbs_cr):
        if a_cr["frame"] in SKIP_FRAMES:
            continue
        av = aabb_volume(a_cr)
        ov = obb_volume(o)
        ratio = ov / av if av > 1e-10 else float("nan")
        print(f"{a_cr['label']:<8}  {aabb_volume(a_iv):>9.4f}  {av:>9.4f}"
              f"  {ov:>9.4f}  {ratio:>11.3f}")
    print()
    if v_cr > 0:
        print(f"OBB Critical volume reduction vs AABB Critical: "
              f"{100*(1 - v_obb/v_cr):.1f}%")
    print()

    # --- Save outputs --------------------------------------------------------
    out_dir = make_output_dir(Path(__file__).parent.parent / "results",
                              prefix="obb_panda")
    print(f"Output directory: {out_dir}\n")

    # Standalone OBB Critical figure
    fig_obb = build_3d_obb_figure(
        method_name="Critical PCA (OBB)",
        obbs=obbs_cr,
        arm_samples_pos=arm_samples, mid_pos=mid_pos,
        obstacles=obstacles, method_color="#9C27B0", t_comp=t_obb,
        link_colors=_LINK_COLORS_DEFAULT, skip_frames=SKIP_FRAMES,
    )
    fig_obb.write_html(str(out_dir / "obb_panda_obb.html"), include_plotlyjs="cdn")
    print("  Saved: obb_panda_obb.html")

    # Standalone AABB Critical figure (for direct comparison)
    fig_aabb = build_3d_aabb_figure(
        method_name="Critical Sampling (AABB)",
        aabbs=aabbs_cr,
        arm_samples_pos=arm_samples, mid_pos=mid_pos,
        obstacles=obstacles, method_color="#2196F3", t_comp=t_cr,
        link_colors=_LINK_COLORS_DEFAULT, skip_frames=SKIP_FRAMES,
    )
    fig_aabb.write_html(str(out_dir / "obb_panda_aabb_cr.html"), include_plotlyjs="cdn")
    print("  Saved: obb_panda_aabb_cr.html")

    # 2-panel: AABB Critical vs OBB Critical
    fig_2 = build_3d_obb_aabb_comparison_figure(
        panels=[
            {"name": "AABB 鈥?Critical Sampling (tight)",
             "envelopes": aabbs_cr, "color": "#2196F3", "t_comp": t_cr, "kind": "aabb"},
            {"name": "OBB  鈥?Critical + PCA (tighter)",
             "envelopes": obbs_cr,  "color": "#9C27B0", "t_comp": t_obb, "kind": "obb"},
        ],
        arm_samples_pos=arm_samples, mid_pos=mid_pos,
        obstacles=obstacles, link_colors=_LINK_COLORS_DEFAULT,
        skip_frames=SKIP_FRAMES,
        title="<b>Panda 7-DOF 鈥?AABB Critical vs OBB Critical Envelope</b>",
    )
    fig_2.write_html(str(out_dir / "obb_panda_2panel.html"), include_plotlyjs="cdn")
    print("  Saved: obb_panda_2panel.html")

    # 4-panel: all three AABB methods + OBB Critical
    fig_4 = build_3d_obb_aabb_comparison_figure(
        panels=[
            {"name": "AABB Interval FK",
             "envelopes": aabbs_iv, "color": "#4CAF50", "t_comp": t_iv, "kind": "aabb"},
            {"name": "AABB Critical",
             "envelopes": aabbs_cr, "color": "#2196F3", "t_comp": t_cr, "kind": "aabb"},
            {"name": f"AABB Random ({n_random})",
             "envelopes": aabbs_rn, "color": "#FF9800", "t_comp": t_rn, "kind": "aabb"},
            {"name": "OBB Critical (PCA)",
             "envelopes": obbs_cr,  "color": "#9C27B0", "t_comp": t_obb, "kind": "obb"},
        ],
        arm_samples_pos=arm_samples, mid_pos=mid_pos,
        obstacles=obstacles, link_colors=_LINK_COLORS_DEFAULT,
        skip_frames=SKIP_FRAMES,
        title="<b>Panda 7-DOF 鈥?All AABB Methods + OBB Critical Envelope Comparison</b>",
    )
    fig_4.write_html(str(out_dir / "obb_panda_4panel.html"), include_plotlyjs="cdn")
    print("  Saved: obb_panda_4panel.html")

    print(f"\nDone. Output: {out_dir}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Panda OBB vs AABB envelope comparison (interactive HTML)")
    parser.add_argument("--seed",    type=int, default=42,
                        help="RNG seed for reproducible scene (default: 42)")
    parser.add_argument("--samples", type=int, default=N_RANDOM,
                        help=f"Random-sampling count (default: {N_RANDOM})")
    args = parser.parse_args()
    generate(seed=args.seed, n_random=args.samples)
