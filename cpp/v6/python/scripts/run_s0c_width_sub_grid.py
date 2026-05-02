#!/usr/bin/env python3
"""S0c: Width × Subdivision × Grid-resolution sweep (paper-facing 10 specs).

This script evaluates envelope behavior under different interval widths,
subdivision counts, and the retained Hull16-grid resolutions.

Outputs:
    experiments/results_iiwa14_final/s0c_width_sub_grid/results.json
"""

import argparse
import json
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import sbf6 as sbf6


BIN_SCHEMES = {
    "legacy": [
        ("W1_0.10_0.20", 0.10, 0.20),
        ("W2_0.20_0.30", 0.20, 0.30),
        ("W3_0.30_0.40", 0.30, 0.40),
        ("W4_0.40_0.50", 0.40, 0.50),
    ],
    "exp1": [
        ("W1_0.001_0.05", 0.001, 0.05),
        ("W2_0.05_0.1", 0.05, 0.10),
        ("W3_0.1_0.2", 0.10, 0.20),
        ("W4_0.2_0.5", 0.20, 0.50),
    ],
}

SUB_VALUES = [1, 2, 4, 8]
GRID_DELTAS = [0.02, 0.04, 0.06, 0.08]

PAPER_VARIANTS = [
    {"endpoint": "CritSample", "envelope": "LinkIAABB", "subdivisions": 1, "grid_delta": None},
    {"endpoint": "CritSample", "envelope": "LinkIAABB", "subdivisions": 2, "grid_delta": None},
    {"endpoint": "CritSample", "envelope": "LinkIAABB", "subdivisions": 4, "grid_delta": None},
    {"endpoint": "CritSample", "envelope": "LinkIAABB", "subdivisions": 8, "grid_delta": None},
    {"endpoint": "CritSample", "envelope": "Hull16_Grid", "subdivisions": 1, "grid_delta": 0.02},
    {"endpoint": "CritSample", "envelope": "Hull16_Grid", "subdivisions": 1, "grid_delta": 0.04},
    {"endpoint": "CritSample", "envelope": "Hull16_Grid", "subdivisions": 1, "grid_delta": 0.06},
    {"endpoint": "CritSample", "envelope": "Hull16_Grid", "subdivisions": 1, "grid_delta": 0.08},
    {"endpoint": "IFK", "envelope": "LinkIAABB", "subdivisions": 4, "grid_delta": None},
    {"endpoint": "IFK", "envelope": "Hull16_Grid", "subdivisions": 1, "grid_delta": 0.04},
]


def random_intervals(robot, rng, width_lo, width_hi):
    lims = robot.joint_limits().limits
    intervals = []
    for lim in lims:
        width = rng.uniform(width_lo, width_hi)
        lo = rng.uniform(lim.lo, max(lim.lo, lim.hi - width))
        hi = min(lo + width, lim.hi)
        intervals.append(sbf6.Interval(float(lo), float(hi)))
    return intervals


def paired_centers(robot, rng, max_width):
    centers = []
    for lim in robot.joint_limits().limits:
        span = float(lim.hi - lim.lo)
        margin = min(0.5 * float(max_width), 0.45 * span)
        lo = float(lim.lo) + margin
        hi = float(lim.hi) - margin
        if hi <= lo:
            lo = float(lim.lo)
            hi = float(lim.hi)
        centers.append(float(rng.uniform(lo, hi)))
    return centers


def paired_intervals(robot, centers, width):
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


def make_env_cfg(env_name, sub, delta=None):
    cfg = sbf6.EnvelopeTypeConfig()
    if env_name == "LinkIAABB":
        cfg.type = sbf6.EnvelopeType.LinkIAABB
    elif env_name == "Hull16_Grid":
        cfg.type = sbf6.EnvelopeType.Hull16_Grid
    else:
        raise ValueError(f"Unsupported envelope: {env_name}")
    cfg.n_subdivisions = int(sub)
    if delta is not None:
        cfg.grid_config.voxel_delta = float(delta)
    return cfg


def make_ep_cfg(endpoint_name):
    cfg = sbf6.EndpointSourceConfig()
    mapping = {
        "IFK": sbf6.EndpointSource.IFK,
        "CritSample": sbf6.EndpointSource.CritSample,
    }
    if endpoint_name not in mapping:
        raise ValueError(f"Unsupported endpoint source: {endpoint_name}")
    cfg.source = mapping[endpoint_name]
    if endpoint_name == "CritSample":
        # CritSample is deterministic lo/hi + k*pi/2 enumeration; no cap.
        cfg.n_samples_crit = 0
    return cfg


def run_one(robot, boxes, ep_cfg, env_cfg, repeats):
    volumes = []
    endpoint_us = []
    envelope_us = []
    total_us = []
    cache_bricks = []
    cache_voxels = []
    cache_payload = []

    for rep in range(repeats):
        for bi, b in enumerate(boxes):
            ep_info = sbf6.compute_endpoint_iaabb_info(robot, b, ep_cfg)
            env_info = sbf6.compute_envelope_info(robot, b, ep_cfg, env_cfg, None)
            ep_time_us = float(ep_info["ep_time_us"])
            env_time_us = float(env_info["env_time_us"])
            endpoint_us.append(ep_time_us)
            envelope_us.append(env_time_us)
            total_us.append(ep_time_us + env_time_us)
            if rep == 0:
                volumes.append(float(env_info["volume"]))
                cache_bricks.append(float(env_info.get("grid_num_bricks", 0)))
                cache_voxels.append(float(env_info.get("grid_num_voxels", 0)))
                cache_payload.append(float(env_info.get("grid_cache_payload_bytes", 0.0)))

    return {
        "volume_mean": float(np.mean(volumes)),
        "volume_std": float(np.std(volumes)),
        "endpoint_us_mean": float(np.mean(endpoint_us)),
        "endpoint_us_std": float(np.std(endpoint_us)),
        "envelope_us_mean": float(np.mean(envelope_us)),
        "envelope_us_std": float(np.std(envelope_us)),
        "total_us_mean": float(np.mean(total_us)),
        "total_us_std": float(np.std(total_us)),
        "cache_bricks_mean": float(np.mean(cache_bricks)),
        "cache_voxels_mean": float(np.mean(cache_voxels)),
        "cache_payload_bytes_mean": float(np.mean(cache_payload)),
    }


def main():
    parser = argparse.ArgumentParser(description="Run width×sub×grid sweep")
    parser.add_argument("--n-boxes", type=int, default=300)
    parser.add_argument("--repeats", type=int, default=10)
    parser.add_argument(
        "--bin-scheme",
        type=str,
        default="legacy",
        choices=sorted(BIN_SCHEMES),
        help="Which width-bin layout to use",
    )
    parser.add_argument(
        "--width-bin",
        type=str,
        default="all",
        help="Run only one width bin by name or 'all'",
    )
    parser.add_argument("--base-seed", type=int, default=6100)
    parser.add_argument("--independent-bins", action="store_true",
                        help="Use legacy independent random intervals instead of the Exp.1 paired protocol")
    parser.add_argument("--out", type=str,
                        default="experiments/results_iiwa14_final/s0c_width_sub_grid/results.json")
    args = parser.parse_args()

    out_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    robot = sbf6.Robot.from_json("data/iiwa14.json")

    rows = []
    t0 = time.time()

    selected_bins = BIN_SCHEMES[args.bin_scheme]
    if args.width_bin != "all":
        selected_bins = [w for w in selected_bins if w[0] == args.width_bin]
        if not selected_bins:
            raise ValueError(
                f"Unknown width bin '{args.width_bin}'. "
                f"Available: {[w[0] for w in BIN_SCHEMES[args.bin_scheme]]}"
            )

    max_width = max(w_hi for _, _, w_hi in BIN_SCHEMES[args.bin_scheme])
    paired_rng = np.random.RandomState(args.base_seed)
    centers = [paired_centers(robot, paired_rng, max_width) for _ in range(args.n_boxes)]

    for wi, (w_name, w_lo, w_hi) in enumerate(selected_bins):
        rng = np.random.RandomState(args.base_seed + wi)
        if args.independent_bins:
            boxes = [random_intervals(robot, rng, w_lo, w_hi) for _ in range(args.n_boxes)]
        else:
            boxes = [paired_intervals(robot, centers[i], float(w_hi)) for i in range(args.n_boxes)]

        for spec in PAPER_VARIANTS:
            endpoint = str(spec["endpoint"])
            envelope = str(spec["envelope"])
            subdivisions = int(spec["subdivisions"])
            delta = spec["grid_delta"]
            ep_cfg = make_ep_cfg(endpoint)
            env_cfg = make_env_cfg(envelope, subdivisions, delta)
            stats = run_one(robot, boxes, ep_cfg, env_cfg, args.repeats)
            rows.append({
                "width_bin": w_name,
                "width_lo": float(w_lo),
                "width_hi": float(w_hi),
                "endpoint": endpoint,
                "endpoint_source": endpoint,
                "envelope": envelope,
                "subdivisions": subdivisions,
                "grid_delta": None if delta is None else float(delta),
                "n_boxes": int(args.n_boxes),
                "repeats": int(args.repeats),
                **stats,
            })
            if delta is None:
                print(f"[done] {w_name} {endpoint} {envelope} sub={subdivisions}")
            else:
                print(f"[done] {w_name} {endpoint} {envelope} sub={subdivisions} delta={float(delta):.2f}")

    payload = {
        "meta": {
            "robot": "iiwa14",
            "endpoint": "mixed_Crit_main_plus_IFK_controls",
            "bin_scheme": args.bin_scheme,
            "interval_protocol": "legacy_independent_bins" if args.independent_bins else "paired_centers_scaled_width_hi",
            "base_seed": int(args.base_seed),
            "width_bins": [
                {"name": n, "lo": lo, "hi": hi}
                for (n, lo, hi) in selected_bins
            ],
            "width_bin_mode": args.width_bin,
            "sub_values": SUB_VALUES,
            "grid_deltas": GRID_DELTAS,
            "paper_variants": PAPER_VARIANTS,
            "n_boxes": int(args.n_boxes),
            "repeats": int(args.repeats),
            "elapsed_s": float(time.time() - t0),
        },
        "rows": rows,
    }

    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, indent=2)

    print(f"Saved: {out_path}")
    print(f"Rows: {len(rows)}")


if __name__ == "__main__":
    main()
