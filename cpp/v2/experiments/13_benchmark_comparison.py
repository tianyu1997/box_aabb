#!/usr/bin/env python3
"""
Experiment 13 — Comprehensive Benchmark: AABB vs OBB, IFK vs Critical
======================================================================

Compares three bounding-box computation methods across two robots and three
interval-width regimes.

Robots
------
  2D   — 2-DOF planar (L1=1.0 m, L2=0.8 m)
  Panda — 7-DOF Franka Emika Panda (DH model)

Methods
-------
  AABB_IFK      Interval Forward Kinematics (conservative, analytic)
  AABB_Critical Critical-angle enumeration AABB (tighter)
  OBB_Critical  PCA on critical-angle cloud (tightest)

Interval widths (per joint)
---------------------------
  small   [0.05, 0.15] rad
  medium  [0.25, 0.50] rad
  large   [0.70, 1.40] rad

Protocol
--------
  N_TRIALS  = 50  independent random intervals (different seeds)
  N_REPEATS = 20  timing repeats per interval (take median)
  Results   : mean ± std over all trials, per robot × width × method

Outputs
-------
  results/benchmark_<ts>/benchmark_results.md   — full markdown report
  results/benchmark_<ts>/benchmark_raw.csv      — raw data for further analysis
"""
from __future__ import annotations

import sys
import math

# Set UTF-8 output on Windows (prevents GBK codec errors for any remaining
# unicode in print statements; markdown is written to file with utf-8 anyway)
try:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass
import time
import csv
import datetime
import argparse
import statistics
from pathlib import Path
from typing import NamedTuple

import numpy as np

sys.path.insert(0, str(Path(__file__).parent.parent / "python"))

from forest_viz import (
    PANDA_DH_PARAMS,
    PANDA_JOINT_LIMITS,
    compute_aabbs_interval_fk,
    compute_aabbs_critical,
    compute_obbs_interval_fk,
    compute_obbs_critical,
    compute_aabbs_interval_planar_2dof,
    compute_aabbs_critical_2dof,
    compute_obbs_interval_fk_2dof,
    compute_obbs_critical_2dof,
    aabb_volume,
    obb_volume,
    obb_area_2d,
    make_output_dir,
)

# ── Robot config ─────────────────────────────────────────────────────────────
L1, L2 = 1.0, 0.8
JOINT_LIMITS_2DOF = [(-math.pi, math.pi), (-math.pi, math.pi)]

# ── Experiment parameters ────────────────────────────────────────────────────
N_TRIALS   = 30
N_REPEATS  = 5    # Panda Critical calls can exceed 500ms; median of 5 is stable

WIDTHS = {
    "small":  (0.05, 0.15),
    "medium": (0.25, 0.50),
    "large":  (0.70, 1.40),
}

# ─────────────────────────────────────────────────────────────────────────────


def _aabb_area_2d(aabb: dict) -> float:
    return max(0.0, aabb["x_max"] - aabb["x_min"]) * max(0.0, aabb["y_max"] - aabb["y_min"])


def _total_area_2d(boxes: list) -> float:
    return sum(_aabb_area_2d(b) for b in boxes)


def _total_obb_area_2d(obbs: list) -> float:
    return sum(obb_area_2d(o) for o in obbs)


def _total_aabb_vol(boxes: list) -> float:
    return sum(aabb_volume(b) for b in boxes)


def _total_obb_vol(obbs: list) -> float:
    return sum(obb_volume(o) for o in obbs)


def _random_intervals(joint_limits: list, rng: np.random.Generator,
                      width_lo: float, width_hi: float) -> list:
    """Sample a random subinterval of each joint within its limits."""
    result = []
    for lo_lim, hi_lim in joint_limits:
        w = rng.uniform(width_lo, width_hi)
        center = rng.uniform(lo_lim + w / 2, hi_lim - w / 2)
        result.append((center - w / 2, center + w / 2))
    return result


def _timed(fn, n_repeats: int):
    """Call fn() n_repeats times; return (result_of_first_call, median_ms)."""
    times = []
    result = None
    for i in range(n_repeats):
        t0 = time.perf_counter()
        r = fn()
        times.append((time.perf_counter() - t0) * 1000.0)
        if i == 0:
            result = r
    return result, statistics.median(times)


# ─────────────────────────────────────────────────────────────────────────────
#  Per-trial runner
# ─────────────────────────────────────────────────────────────────────────────

class TrialResult(NamedTuple):
    robot:      str
    width_key:  str
    method:     str
    trial:      int
    time_ms:    float
    size:       float    # total area (2D) or total volume (3D, m³)


def run_2d_trial(trial: int, width_key: str, width_lo: float, width_hi: float,
                 seed: int) -> list[TrialResult]:
    rng = np.random.default_rng(seed)
    q_iv = _random_intervals(JOINT_LIMITS_2DOF, rng, width_lo, width_hi)
    rows: list[TrialResult] = []

    # AABB_IFK
    boxes, ms = _timed(
        lambda: compute_aabbs_interval_planar_2dof((L1, L2), q_iv), N_REPEATS)
    rows.append(TrialResult("2D", width_key, "AABB_IFK",      trial, ms,
                            _total_area_2d(boxes)))

    # AABB_Critical
    boxes, ms = _timed(
        lambda: compute_aabbs_critical_2dof((L1, L2), q_iv), N_REPEATS)
    rows.append(TrialResult("2D", width_key, "AABB_Critical",  trial, ms,
                            _total_area_2d(boxes)))

    # OBB_IFK
    obbs, ms = _timed(
        lambda: compute_obbs_interval_fk_2dof((L1, L2), q_iv), N_REPEATS)
    rows.append(TrialResult("2D", width_key, "OBB_IFK",        trial, ms,
                            _total_obb_area_2d(obbs)))

    # OBB_Critical
    obbs, ms = _timed(
        lambda: compute_obbs_critical_2dof((L1, L2), q_iv), N_REPEATS)
    rows.append(TrialResult("2D", width_key, "OBB_Critical",   trial, ms,
                            _total_obb_area_2d(obbs)))

    return rows


def run_panda_trial(trial: int, width_key: str, width_lo: float, width_hi: float,
                    seed: int) -> list[TrialResult]:
    rng = np.random.default_rng(seed)
    q_iv = _random_intervals(PANDA_JOINT_LIMITS, rng, width_lo, width_hi)
    rows: list[TrialResult] = []

    # AABB_IFK
    boxes, ms = _timed(
        lambda: compute_aabbs_interval_fk(PANDA_DH_PARAMS, q_iv), N_REPEATS)
    rows.append(TrialResult("Panda", width_key, "AABB_IFK",     trial, ms,
                            _total_aabb_vol(boxes)))

    # AABB_Critical
    boxes, ms = _timed(
        lambda: compute_aabbs_critical(PANDA_DH_PARAMS, q_iv), N_REPEATS)
    rows.append(TrialResult("Panda", width_key, "AABB_Critical", trial, ms,
                            _total_aabb_vol(boxes)))

    # OBB_IFK
    obbs, ms = _timed(
        lambda: compute_obbs_interval_fk(PANDA_DH_PARAMS, q_iv), N_REPEATS)
    rows.append(TrialResult("Panda", width_key, "OBB_IFK",       trial, ms,
                            _total_obb_vol(obbs)))

    # OBB_Critical
    obbs, ms = _timed(
        lambda: compute_obbs_critical(PANDA_DH_PARAMS, q_iv), N_REPEATS)
    rows.append(TrialResult("Panda", width_key, "OBB_Critical",  trial, ms,
                            _total_obb_vol(obbs)))

    return rows


# ─────────────────────────────────────────────────────────────────────────────
#  Statistics helpers
# ─────────────────────────────────────────────────────────────────────────────

def _stats(values: list[float]):
    """Return (mean, std, min, max) for a list."""
    n = len(values)
    if n == 0:
        return 0.0, 0.0, 0.0, 0.0
    mu   = sum(values) / n
    std  = math.sqrt(sum((v - mu)**2 for v in values) / max(n - 1, 1))
    return mu, std, min(values), max(values)


def _collect(rows: list[TrialResult], robot: str, width_key: str, method: str):
    return [r for r in rows
            if r.robot == robot and r.width_key == width_key
            and r.method == method]


# ─────────────────────────────────────────────────────────────────────────────
#  Report generation
# ─────────────────────────────────────────────────────────────────────────────

def _fmt(mu, std):
    """Format mean ± std with appropriate precision."""
    if mu < 0.001:
        return f"{mu:.2e} ± {std:.2e}"
    if mu < 1:
        return f"{mu:.4f} ± {std:.4f}"
    return f"{mu:.3f} ± {std:.3f}"


def build_markdown(rows: list[TrialResult], ts: str) -> str:
    methods = ["AABB_IFK", "AABB_Critical", "OBB_IFK", "OBB_Critical"]
    width_keys = list(WIDTHS.keys())

    lines = []
    lines.append("# Bounding-Box Benchmark Report")
    lines.append("")
    lines.append(f"**Date:** {ts[:4]}-{ts[4:6]}-{ts[6:8]}  "
                 f"{ts[9:11]}:{ts[11:13]}:{ts[13:15]}")
    lines.append("")
    lines.append("## Experiment Setup")
    lines.append("")
    lines.append(f"| Parameter | Value |")
    lines.append(f"|-----------|-------|")
    lines.append(f"| N_TRIALS  | {N_TRIALS} (independent random intervals) |")
    lines.append(f"| N_REPEATS | {N_REPEATS} (timing repeats per trial, median taken) |")
    lines.append(f"| 2D robot  | 2-DOF planar, L1={L1} m, L2={L2} m |")
    lines.append(f"| 3D robot  | Franka Emika Panda (7-DOF, DH model) |")
    lines.append("")
    lines.append("| Width regime | Per-joint width range (rad) |")
    lines.append("|-------------|----------------------------|")
    for wk, (wlo, whi) in WIDTHS.items():
        lines.append(f"| {wk:6s}       | [{wlo}, {whi}] |")
    lines.append("")

    for robot, size_unit in [("2D", "m²"), ("Panda", "m³")]:
        lines.append(f"---")
        lines.append("")
        lines.append(f"## Robot: {robot}")
        lines.append("")

        # ── Timing table ──────────────────────────────────────────────────
        lines.append(f"### {robot} — Computation Time (ms, median over N_REPEATS)")
        lines.append("")
        hdr = "| Width   | " + " | ".join(f"{m:>20}" for m in methods) + " |"
        lines.append(hdr)
        sep = "|---------| " + " | ".join("-" * 20 for _ in methods) + " |"
        lines.append(sep)
        for wk in width_keys:
            cells = []
            for m in methods:
                data = _collect(rows, robot, wk, m)
                mu, std, lo, hi = _stats([r.time_ms for r in data])
                cells.append(f"{_fmt(mu, std):>20}")
            lines.append(f"| {wk:7s} | " + " | ".join(cells) + " |")
        lines.append("")

        # ── Size table ────────────────────────────────────────────────────
        lines.append(f"### {robot} — Total Envelope Size ({size_unit})")
        lines.append("")
        lines.append(hdr)
        lines.append(sep)
        for wk in width_keys:
            cells = []
            for m in methods:
                data = _collect(rows, robot, wk, m)
                mu, std, lo, hi = _stats([r.size for r in data])
                cells.append(f"{_fmt(mu, std):>20}")
            lines.append(f"| {wk:7s} | " + " | ".join(cells) + " |")
        lines.append("")

        # ── Reduction ratio table ─────────────────────────────────────────
        lines.append(f"### {robot} — Size Reduction vs AABB_IFK (%)")
        lines.append("")
        ratio_methods = ["AABB_Critical", "OBB_IFK", "OBB_Critical"]
        hdr2 = "| Width   | " + " | ".join(f"{m:>20}" for m in ratio_methods) + " |"
        sep2 = "|---------| " + " | ".join("-" * 20   for _ in ratio_methods) + " |"
        lines.append(hdr2)
        lines.append(sep2)
        for wk in width_keys:
            base_data  = _collect(rows, robot, wk, "AABB_IFK")
            base_sizes = [r.size for r in base_data]
            cells = []
            for m in ratio_methods:
                data = _collect(rows, robot, wk, m)
                ratios = []
                for rd, bs in zip(data, base_data):
                    if bs.size > 1e-12:
                        ratios.append(100.0 * (1.0 - rd.size / bs.size))
                    else:
                        ratios.append(0.0)
                mu, std, lo, hi = _stats(ratios)
                cells.append(f"{mu:>8.2f}% ± {std:.2f}%    ")
            lines.append(f"| {wk:7s} | " + " | ".join(cells) + " |")
        lines.append("")

        # ── OBB vs AABB_Critical ratio ────────────────────────────────────
        lines.append(f"### {robot} — OBB_Critical Size Reduction vs AABB_Critical (%)")
        lines.append("")
        lines.append("| Width   |   OBB/AABB_CR mean size ratio   | Reduction (%)           |")
        lines.append("|---------|----------------------------------|-------------------------|")
        for wk in width_keys:
            cr_data  = _collect(rows, robot, wk, "AABB_Critical")
            obb_data = _collect(rows, robot, wk, "OBB_Critical")
            ratios   = []
            for rc, ro in zip(cr_data, obb_data):
                if rc.size > 1e-12:
                    ratios.append(ro.size / rc.size)
            mu_r, std_r, _, _ = _stats(ratios)
            reduc = [(1 - r) * 100 for r in ratios]
            mu_d, std_d, _, _ = _stats(reduc)
            lines.append(f"| {wk:7s} | {mu_r:8.4f} ± {std_r:.4f}              "
                         f"| {mu_d:7.2f}% ± {std_d:.2f}%      |")
        lines.append("")

        # ── Timing speed-up: Critical vs IFK ────────────────────────────
        lines.append(f"### {robot} — Speed-up: AABB_Critical vs AABB_IFK")
        lines.append("")
        lines.append("| Width   | AABB_IFK (ms)       | AABB_Critical (ms)  | Speed-up factor |")
        lines.append("|---------|---------------------|---------------------|-----------------|")
        for wk in width_keys:
            ifk_data = _collect(rows, robot, wk, "AABB_IFK")
            cr_data  = _collect(rows, robot, wk, "AABB_Critical")
            mu_i, std_i, _, _ = _stats([r.time_ms for r in ifk_data])
            mu_c, std_c, _, _ = _stats([r.time_ms for r in cr_data])
            speedup = mu_i / mu_c if mu_c > 1e-9 else float("nan")
            lines.append(f"| {wk:7s} | {_fmt(mu_i, std_i):>19} "
                         f"| {_fmt(mu_c, std_c):>19} | {speedup:>15.2f}x |")
        lines.append("")

    # ── Per-width cross-robot summary ─────────────────────────────────────────
    lines.append("---")
    lines.append("")
    lines.append("## Cross-Robot Summary Table")
    lines.append("")
    lines.append("Mean total envelope size and computation time across all trials.")
    lines.append("")
    lines.append("| Robot | Width  | Method        |  Time (ms)  |  Size (unit) | IFK reduce % | CR reduce % |")
    lines.append("|-------|--------|---------------|-------------|--------------|-------------|-------------|")
    for robot, size_unit in [("2D", "m²"), ("Panda", "m³")]:
        for wk in width_keys:
            for m in methods:
                data = _collect(rows, robot, wk, m)
                mu_t, std_t, _, _ = _stats([r.time_ms for r in data])
                mu_s, std_s, _, _ = _stats([r.size    for r in data])
                # reduction vs IFK
                base = _collect(rows, robot, wk, "AABB_IFK")
                if base and base[0].size > 1e-12:
                    red_ifk  = [(1 - rd.size / bs.size) * 100
                                for rd, bs in zip(data, base)
                                if bs.size > 1e-12]
                    mu_ri = _stats(red_ifk)[0] if red_ifk else 0.0
                else:
                    mu_ri = 0.0
                # reduction vs Critical (shown for OBB methods)
                cr_base = _collect(rows, robot, wk, "AABB_Critical")
                if cr_base and m in ("OBB_IFK", "OBB_Critical"):
                    red_cr = [(1 - rd.size / bs.size) * 100
                              for rd, bs in zip(data, cr_base)
                              if bs.size > 1e-12]
                    mu_rc = _stats(red_cr)[0] if red_cr else 0.0
                elif m == "AABB_IFK":
                    mu_rc = float("nan")
                else:
                    mu_rc = 0.0
                ri_str = f"{mu_ri:+.1f}%" if not math.isnan(mu_ri) else "  ref  "
                rc_str = f"{mu_rc:+.1f}%" if not math.isnan(mu_rc) else "  ref  "
                lines.append(f"| {robot:5s} | {wk:6s} | {m:13s} "
                             f"| {mu_t:9.4f}   | {mu_s:10.5f}   "
                             f"| {ri_str:>11} | {rc_str:>11} |")
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("## Notes")
    lines.append("")
    lines.append(
        "- **AABB_IFK** uses conservative interval arithmetic; it always over-approximates.\n"
        "- **AABB_Critical** enumerates critical angles (k\u00b7\u03c0/2) and joint boundaries;\n"
        "  it gives a tighter box but is not guaranteed to be conservative (it is correct\n"
        "  for the discrete critical sample set).\n"
        "- **OBB_IFK** applies the same interval-FK projections as AABB_IFK but onto a\n"
        "  link-aligned frame; shares the same over-approximation bias and speed as AABB_IFK.\n"
        "- **OBB_Critical** applies PCA to the same critical-angle point cloud;\n"
        "  the resulting oriented bounding box is always \u2264 AABB_Critical in volume.\n"
        "- Times reported are **median** of N_REPEATS calls per trial (removes JIT/cache effects).\n"
        "- Sizes are **summed over all links** (and tool for Panda).\n"
        "- 2D size unit: m\u00b2 (total cross-sectional area of all link bounding shapes).\n"
        "- 3D size unit: m\u00b3 (total volume of all link bounding shapes).\n"
    )

    lines.append(f"*Generated by `experiments/13_benchmark_comparison.py`  "
                 f"— {ts}*")
    return "\n".join(lines)


# ─────────────────────────────────────────────────────────────────────────────
#  Main
# ─────────────────────────────────────────────────────────────────────────────

def main():
    global N_TRIALS, N_REPEATS
    parser = argparse.ArgumentParser(description="Bounding-box benchmark")
    parser.add_argument("--seed",    type=int, default=0,
                        help="Base RNG seed (default: 0)")
    parser.add_argument("--trials",  type=int, default=N_TRIALS,
                        help=f"Number of trials per condition (default: {N_TRIALS})")
    parser.add_argument("--repeats", type=int, default=N_REPEATS,
                        help=f"Timing repeats per trial (default: {N_REPEATS})")
    args = parser.parse_args()

    N_TRIALS  = args.trials
    N_REPEATS = args.repeats

    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    base_seed = args.seed

    all_rows: list[TrialResult] = []

    total_conditions = 2 * len(WIDTHS) * 3  # 2 robots × 3 widths × 3 methods
    done = 0

    print(f"\n{'='*70}")
    print(f"  Bounding-Box Benchmark  --  {N_TRIALS} trials x {N_REPEATS} repeats")
    print(f"{'='*70}\n")

    for wk, (wlo, whi) in WIDTHS.items():
        print(f"[Width: {wk:6s}  ({wlo:.2f}–{whi:.2f} rad)]")

        # 2D robot
        print(f"  Robot: 2D   [", end="", flush=True)
        t_start = time.perf_counter()
        for t in range(N_TRIALS):
            seed = base_seed * 100_000 + hash(("2D", wk, t)) % 100_000
            rows = run_2d_trial(t, wk, wlo, whi, seed)
            all_rows.extend(rows)
            if (t + 1) % max(1, N_TRIALS // 10) == 0:
                print(".", end="", flush=True)
        elapsed = time.perf_counter() - t_start
        print(f"]  OK  ({elapsed:.1f} s, {N_TRIALS} trials)")

        # Panda robot
        print(f"  Robot: Panda[", end="", flush=True)
        t_start = time.perf_counter()
        for t in range(N_TRIALS):
            seed = base_seed * 100_000 + hash(("Panda", wk, t)) % 100_000
            rows = run_panda_trial(t, wk, wlo, whi, seed)
            all_rows.extend(rows)
            if (t + 1) % max(1, N_TRIALS // 10) == 0:
                print(".", end="", flush=True)
        elapsed = time.perf_counter() - t_start
        print(f"]  OK  ({elapsed:.1f} s, {N_TRIALS} trials)")
        print()

    # ── Quick console summary ─────────────────────────────────────────────────
    print(f"\n{'='*70}")
    print("  RESULTS SUMMARY")
    print(f"{'='*70}")
    for robot, size_unit in [("2D", "m2"), ("Panda", "m3")]:
        print(f"\n  Robot: {robot}")
        print(f"  {'Method':<18} {'small time':>15} {'medium time':>15} {'large time':>15}")
        print(f"  {'-'*18} {'-'*15} {'-'*15} {'-'*15}")
        for m in ["AABB_IFK", "AABB_Critical", "OBB_IFK", "OBB_Critical"]:
            cells = []
            for wk in ["small", "medium", "large"]:
                data = _collect(all_rows, robot, wk, m)
                mu, std, _, _ = _stats([r.time_ms for r in data])
                cells.append(f"{mu:6.3f}+/-{std:.3f}")
            print(f"  {m:<18} {cells[0]:>15} {cells[1]:>15} {cells[2]:>15}")
        print()
        print(f"  {'Method':<18} {'small size':>15} {'medium size':>15} {'large size':>15}  ({size_unit})")
        print(f"  {'-'*18} {'-'*15} {'-'*15} {'-'*15}")
        for m in ["AABB_IFK", "AABB_Critical", "OBB_IFK", "OBB_Critical"]:
            cells = []
            for wk in ["small", "medium", "large"]:
                data = _collect(all_rows, robot, wk, m)
                mu, std, _, _ = _stats([r.size for r in data])
                cells.append(f"{mu:6.4f}+/-{std:.4f}")
            print(f"  {m:<18} {cells[0]:>15} {cells[1]:>15} {cells[2]:>15}")

    # ── Save outputs ──────────────────────────────────────────────────────────
    out_dir = make_output_dir(
        Path(__file__).parent.parent / "results", prefix="benchmark")

    # CSV
    csv_path = out_dir / "benchmark_raw.csv"
    with open(str(csv_path), "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["robot", "width", "method", "trial",
                         "time_ms", "size"])
        for r in all_rows:
            writer.writerow([r.robot, r.width_key, r.method, r.trial,
                             f"{r.time_ms:.6f}", f"{r.size:.8f}"])
    print(f"\nRaw data saved: {csv_path}")

    # Markdown
    md_path = out_dir / "benchmark_results.md"
    md = build_markdown(all_rows, ts)
    with open(str(md_path), "w", encoding="utf-8") as f:
        f.write(md)
    print(f"Report saved:   {md_path}")
    print(f"\nDone.  Output directory: {out_dir}\n")


if __name__ == "__main__":
    main()
