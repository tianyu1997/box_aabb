#!/usr/bin/env python3
"""Endpoint-source width profile with MC fixed volumetric density.

Protocol:
- Width bins: [0.001,0.05], [0.05,0.1], [0.1,0.2], [0.2,0.5]
- Seeds per bin: default 100
- Sources: IFK, CritSample, Analytical, GCPC, MC
- MC baseline: per-seed MC sample count computed from joint-box volume:
    n_mc = clip(round(rho * volume_joint_box), min_samples, max_samples)

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
import sbf5


WIDTH_BINS = [
    ("W1_0.001_0.05", 0.001, 0.05),
    ("W2_0.05_0.1", 0.05, 0.10),
    ("W3_0.1_0.2", 0.10, 0.20),
    ("W4_0.2_0.5", 0.20, 0.50),
]

SOURCES = ["IFK", "CritSample", "Analytical", "GCPC", "MC"]


def _find_gcpc_cache_path(robot_key: str) -> str | None:
    candidates = [
        os.path.join("data", f"{robot_key}.gcpc"),
        os.path.join("data", f"{robot_key}_5000.gcpc"),
        os.path.join("data", f"{robot_key}_500.gcpc"),
    ]
    for p in candidates:
        if os.path.exists(p):
            return p
    return None


def _random_intervals(robot, rng, width_lo, width_hi):
    lims = robot.joint_limits().limits
    intervals = []
    for lim in lims:
        width = rng.uniform(width_lo, width_hi)
        lo = rng.uniform(lim.lo, max(lim.lo, lim.hi - width))
        hi = min(lo + width, lim.hi)
        intervals.append(sbf5.Interval(float(lo), float(hi)))
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
                 bypass_narrow_skip: bool = False):
    cfg = sbf5.EndpointSourceConfig()
    cfg.source = {
        "IFK": sbf5.EndpointSource.IFK,
        "CritSample": sbf5.EndpointSource.CritSample,
        "Analytical": sbf5.EndpointSource.Analytical,
        "GCPC": sbf5.EndpointSource.GCPC,
        "MC": sbf5.EndpointSource.MC,
    }[source_name]
    if source_name == "MC":
        cfg.n_samples_crit = int(n_mc_samples)
    if source_name in ("Analytical", "GCPC"):
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
            "MC sampling density in samples per rad^d. If omitted, auto-calibrate so "
            "a reference box of width 0.35 rad per joint maps to 50000 samples."
        ),
    )
    parser.add_argument("--min-samples", type=int, default=2000)
    parser.add_argument("--max-samples", type=int, default=200000)
    parser.add_argument("--bypass-narrow-skip", action="store_true",
                        help="Force Analytical/GCPC to run all phases even on narrow intervals")
    parser.add_argument(
        "--out",
        type=str,
        default="experiments/results_iiwa14_final/s0_ep_width_profile_mc_density_seed100_absneg/results.json",
    )
    args = parser.parse_args()

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    robot = sbf5.Robot.from_json("data/iiwa14.json")
    n_dof = len(robot.joint_limits().limits)

    if args.rho is None:
        ref_width = 0.35
        ref_joint_box_volume = ref_width ** n_dof
        rho = 50000.0 / ref_joint_box_volume
        rho_mode = "auto_calibrated_from_ref_width_0.35_to_50000"
    else:
        rho = float(args.rho)
        rho_mode = "user_provided"

    gcpc_cache = None
    gcpc_path = _find_gcpc_cache_path("iiwa14")
    if gcpc_path:
        gcpc_cache = sbf5.GcpcCache.load(gcpc_path)

    rows = []

    for bi, (bin_name, w_lo, w_hi) in enumerate(WIDTH_BINS):
        rng = np.random.RandomState(6100 + bi)

        acc = {
            s: {"vol": [], "time_us": [], "dvol_mc": [], "max_neg_abs": []}
            for s in SOURCES
        }
        mc_samples_used = []

        for _ in range(args.seeds):
            intervals = _random_intervals(robot, rng, w_lo, w_hi)
            box_vol = _joint_box_volume(intervals)
            n_mc = int(np.clip(round(rho * box_vol), args.min_samples, args.max_samples))
            mc_samples_used.append(n_mc)

            trial = {}
            trial_extent = {}

            for src in SOURCES:
                if src == "GCPC" and gcpc_cache is None:
                    continue
                cfg = _make_ep_cfg(src, n_mc_samples=n_mc,
                                   bypass_narrow_skip=args.bypass_narrow_skip)
                info = sbf5.compute_endpoint_iaabb_info(
                    robot,
                    intervals,
                    cfg,
                    gcpc_cache if src == "GCPC" else None,
                )
                lo, hi = _extent_from_iaabbs(info["endpoint_iaabbs"])
                trial[src] = {
                    "volume": float(info["volume_sum"]),
                    "time_us": float(info["ep_time_us"]),
                }
                trial_extent[src] = (lo, hi)

            if "MC" not in trial:
                continue

            mc_vol = trial["MC"]["volume"]
            mc_lo, mc_hi = trial_extent["MC"]
            mc_extent = mc_hi - mc_lo

            for src, vals in trial.items():
                cur_lo, cur_hi = trial_extent[src]
                cur_extent = cur_hi - cur_lo
                extent_gap = cur_extent - mc_extent
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
            "gap_reference": "MC",
            "maxNeg_definition": "absolute largest negative gap (0 if no negative)",
            "mc_sampling_mode": "fixed_joint_box_volume_density",
            "mc_density_rho": float(rho),
            "mc_density_unit": "samples_per_rad_pow_dof",
            "mc_density_mode": rho_mode,
            "mc_min_samples": int(args.min_samples),
            "mc_max_samples": int(args.max_samples),
            "width_bins": [
                {"name": name, "lo": lo, "hi": hi}
                for name, lo, hi in WIDTH_BINS
            ],
            "sources": SOURCES,
            "bypass_narrow_skip": args.bypass_narrow_skip,
        },
        "rows": rows,
    }

    with out_path.open("w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    print(f"Saved: {out_path}")
    print(f"Rows: {len(rows)}")


if __name__ == "__main__":
    main()
