#!/usr/bin/env python3
"""Endpoint-source width profile with MC width-proportional density.

Protocol:
- Width bins: [0.001,0.05], [0.05,0.1], [0.1,0.2], [0.2,0.5]
- Seeds per bin: default 100
- Sources: IFK, CritSample, Analytical, MC
- MC baseline: per-seed MC sample count proportional to geometric mean width:
    n_mc = clip(round(rho * V_box^{1/d}), min_samples, max_samples)
  This ensures MC cost scales linearly with joint-interval width,
  avoiding the w^d dynamic range of raw-volume scaling.

Output:
  experiments/results_iiwa14_final/s0_ep_width_profile_mc_density_seed100_absneg/results.json
"""

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import sbf6 as sbf6


WIDTH_BINS = [
    ("W1_0.001_0.05", 0.001, 0.05),
    ("W2_0.05_0.1", 0.05, 0.10),
    ("W3_0.1_0.2", 0.10, 0.20),
    ("W4_0.2_0.5", 0.20, 0.50),
]

SOURCES = ["IFK", "CritSample", "Analytical", "MC"]
GAP_REFERENCE_SOURCES = ("CritSample", "Analytical", "MC")


def _random_intervals(robot, rng, width_lo, width_hi):
    lims = robot.joint_limits().limits
    intervals = []
    for lim in lims:
        width = rng.uniform(width_lo, width_hi)
        lo = rng.uniform(lim.lo, max(lim.lo, lim.hi - width))
        hi = min(lo + width, lim.hi)
        intervals.append(sbf6.Interval(float(lo), float(hi)))
    return intervals


def _paired_centers(robot, rng, max_width: float):
    centers = []
    for lim in robot.joint_limits().limits:
        span = float(lim.hi - lim.lo)
        margin = min(0.5 * max_width, 0.45 * span)
        lo = float(lim.lo) + margin
        hi = float(lim.hi) - margin
        if hi <= lo:
            lo = float(lim.lo)
            hi = float(lim.hi)
        centers.append(float(rng.uniform(lo, hi)))
    return centers


def _paired_intervals(robot, centers, width):
    intervals = []
    for center, lim in zip(centers, robot.joint_limits().limits):
        span = float(lim.hi - lim.lo)
        w = min(float(width), 0.95 * span)
        lo = max(float(lim.lo), float(center) - 0.5 * w)
        hi = min(float(lim.hi), float(center) + 0.5 * w)
        if hi - lo < w:
            if lo <= float(lim.lo):
                hi = min(float(lim.hi), lo + w)
            elif hi >= float(lim.hi):
                lo = max(float(lim.lo), hi - w)
        intervals.append(sbf6.Interval(float(lo), float(hi)))
    return intervals


def _extent_from_iaabbs(flat_iaabbs):
    arr = np.asarray(flat_iaabbs, dtype=float)
    if arr.size == 0:
        z = np.array([0.0, 0.0, 0.0], dtype=float)
        return z, z
    arr = arr.reshape((-1, 6))
    lo = np.min(arr[:, 0:3], axis=0)
    hi = np.max(arr[:, 3:6], axis=0)
    return lo, hi


def _make_ep_cfg(source_name: str, n_mc_samples: int = 1000,
                 critical_combo_cap: int = 8192,
                 analytical_max_phase: int = 3,
                 bypass_narrow_skip: bool = False):
    cfg = sbf6.EndpointSourceConfig()
    cfg.source = {
        "IFK": sbf6.EndpointSource.IFK,
        "CritSample": sbf6.EndpointSource.CritSample,
        "Analytical": sbf6.EndpointSource.Analytical,
        "MC": sbf6.EndpointSource.MC,
    }[source_name]
    if source_name == "CritSample":
        # CritSample is deterministic: it enumerates lo/hi and k*pi/2 candidates.
        # The binding field is named n_samples_crit, but it is a combo cap here.
        cfg.n_samples_crit = int(critical_combo_cap)
    if source_name == "MC":
        cfg.n_samples_crit = int(n_mc_samples)
    if source_name == "Analytical":
        cfg.max_phase_analytical = int(analytical_max_phase)
        cfg.bypass_narrow_skip = bypass_narrow_skip
    return cfg


def _joint_box_volume(intervals) -> float:
    vol = 1.0
    for itv in intervals:
        vol *= max(0.0, float(itv.hi - itv.lo))
    return float(vol)


def main():
    parser = argparse.ArgumentParser(description="Run EP width profile with MC fixed density")
    parser.add_argument("--seeds", type=int, default=100, help="Seeds per width bin")
    parser.add_argument(
        "--rho",
        type=float,
        default=None,
        help=(
            "MC sampling density in samples per rad. If omitted, auto-calibrate so "
            "a reference box of width 0.35 rad per joint maps to --ref-samples samples."
        ),
    )
    parser.add_argument("--ref-samples", type=int, default=2000000,
                        help="MC samples for reference width 0.35 (for auto rho)")
    parser.add_argument("--min-samples", type=int, default=1000)
    parser.add_argument("--max-samples", type=int, default=10000000)
    parser.add_argument("--bypass-narrow-skip", action="store_true",
                        help="Force Analytical to run all phases even on narrow intervals")
    parser.add_argument("--base-seed", type=int, default=6100,
                        help="Base RNG seed (per-bin seed = base-seed + bin_index)")
    parser.add_argument("--critical-combo-cap", "--crit-samples", dest="critical_combo_cap",
                        type=int, default=8192,
                        help="Maximum deterministic lo/hi and k*pi/2 candidate tuples for CritSample")
    parser.add_argument("--analytical-max-phase", type=int, default=3,
                        help="Analytical phase limit for this Exp.1 comparison")
    parser.add_argument("--independent-bins", action="store_true",
                        help="Use the legacy protocol with independent random boxes per width bin")
    parser.add_argument(
        "--out",
        type=str,
        default="experiments/results_iiwa14_final/s0_ep_width_profile_mc_density_seed100_absneg/results.json",
    )
    args = parser.parse_args()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    robot = sbf6.Robot.from_json("data/iiwa14.json")
    n_dof = len(robot.joint_limits().limits)

    if args.rho is None:
        ref_width = 0.35
        rho = float(args.ref_samples) / ref_width
        rho_mode = f"auto_calibrated_from_ref_width_{ref_width}_to_{args.ref_samples}"
    else:
        rho = float(args.rho)
        rho_mode = "user_provided"

    rows = []

    max_width = max(w_hi for _, _, w_hi in WIDTH_BINS)
    paired_rng = np.random.RandomState(args.base_seed)
    paired_centers = [_paired_centers(robot, paired_rng, max_width) for _ in range(args.seeds)]

    for bi, (bin_name, w_lo, w_hi) in enumerate(WIDTH_BINS):
        rng = np.random.RandomState(args.base_seed + bi)
        representative_width = float(w_hi)

        acc = {
            s: {"vol": [], "time_us": [], "dvol_mc": [], "max_neg_abs": []}
            for s in SOURCES
        }
        mc_samples_used = []

        trials = []
        for seed_index in range(args.seeds):
            if args.independent_bins:
                intervals = _random_intervals(robot, rng, w_lo, w_hi)
            else:
                intervals = _paired_intervals(robot, paired_centers[seed_index], representative_width)
            box_vol = _joint_box_volume(intervals)
            geo_mean_width = box_vol ** (1.0 / n_dof)
            n_mc = int(np.clip(round(rho * geo_mean_width), args.min_samples, args.max_samples))
            mc_samples_used.append(n_mc)
            trials.append({"intervals": intervals, "n_mc": n_mc, "values": {}, "extents": {}})

        for src in SOURCES:
            for trial in trials:
                cfg = _make_ep_cfg(src, n_mc_samples=trial["n_mc"],
                                   critical_combo_cap=args.critical_combo_cap,
                                   analytical_max_phase=args.analytical_max_phase,
                                   bypass_narrow_skip=args.bypass_narrow_skip)
                info = sbf6.compute_endpoint_iaabb_info(
                    robot,
                    trial["intervals"],
                    cfg,
                )
                lo, hi = _extent_from_iaabbs(info["endpoint_iaabbs"])
                trial["values"][src] = {
                    "volume": float(info["volume_sum"]),
                    "time_us": float(info["ep_time_us"]),
                }
                trial["extents"][src] = (lo, hi)

        for trial in trials:
            trial_values = trial["values"]
            trial_extent = trial["extents"]
            reference_extents = [
                trial_extent[src][1] - trial_extent[src][0]
                for src in GAP_REFERENCE_SOURCES
                if src in trial_extent
            ]
            if "MC" not in trial_values or not reference_extents:
                continue

            mc_vol = trial_values["MC"]["volume"]
            reference_extent = np.maximum.reduce(reference_extents)

            for src, vals in trial_values.items():
                cur_lo, cur_hi = trial_extent[src]
                cur_extent = cur_hi - cur_lo
                extent_gap = cur_extent - reference_extent
                neg = extent_gap[extent_gap < 0.0]
                max_neg_abs = float(np.max(np.abs(neg))) if neg.size > 0 else 0.0

                acc[src]["vol"].append(vals["volume"])
                acc[src]["time_us"].append(vals["time_us"])
                acc[src]["dvol_mc"].append(vals["volume"] - mc_vol)
                acc[src]["max_neg_abs"].append(max_neg_abs)

        for src in SOURCES:
            if len(acc[src]["vol"]) == 0:
                continue
            rows.append(
                {
                    "width_bin": bin_name,
                    "width_lo": float(w_lo),
                    "width_hi": float(w_hi),
                    "endpoint": src,
                    "n_seeds": int(len(acc[src]["vol"])),
                    "ep_volume_mean": float(np.mean(acc[src]["vol"])),
                    "ep_time_us_mean": float(np.mean(acc[src]["time_us"])),
                    "ep_time_us_median": float(np.median(acc[src]["time_us"])),
                    "dvol_vs_mc_mean": float(np.mean(acc[src]["dvol_mc"])),
                    "max_negative_gap_abs": float(np.max(acc[src]["max_neg_abs"])),
                }
            )

        print(
            f"[done] {bin_name}: seeds={args.seeds}, "
            f"MC n_samples mean={np.mean(mc_samples_used):.1f}, "
            f"min={int(np.min(mc_samples_used))}, max={int(np.max(mc_samples_used))}"
        )

    payload = {
        "meta": {
            "n_seeds_per_bin": int(args.seeds),
            "gap_reference": "per_axis_max_extent_of_CritSample_Analytical_MC",
            "maxNeg_definition": "absolute largest negative gap (0 if no negative)",
            "mc_sampling_mode": "geomean_width_proportional",
            "mc_density_rho": float(rho),
            "mc_density_unit": "samples_per_rad_geomean_width",
            "mc_density_mode": rho_mode,
            "mc_min_samples": int(args.min_samples),
            "mc_max_samples": int(args.max_samples),
            "critical_combo_cap": int(args.critical_combo_cap),
            "analytical_max_phase": int(args.analytical_max_phase),
            "width_bins": [
                {"name": name, "lo": lo, "hi": hi}
                for name, lo, hi in WIDTH_BINS
            ],
            "sources": SOURCES,
            "bypass_narrow_skip": args.bypass_narrow_skip,
            "interval_protocol": "legacy_independent_bins" if args.independent_bins else "paired_centers_scaled_width_hi",
            "analytical_timing_protocol": "single_real_call_median",
        },
        "rows": rows,
    }

    with out_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    print(f"Saved: {out_path}")
    print(f"Rows: {len(rows)}")


if __name__ == "__main__":
    main()
