#!/usr/bin/env python3
"""S0c: Width × Subdivision × Grid-resolution sweep (CritSample).

This script evaluates envelope behavior under different interval widths,
subdivision counts, and grid resolutions.

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
import sbf5


WIDTH_BINS = [
    ("W1_0.10_0.20", 0.10, 0.20),
    ("W2_0.20_0.30", 0.20, 0.30),
    ("W3_0.30_0.40", 0.30, 0.40),
    ("W4_0.40_0.50", 0.40, 0.50),
]

SUB_VALUES = [1, 2, 4, 8]
GRID_DELTAS = [0.02, 0.04, 0.06, 0.08]


def random_intervals(robot, rng, width_lo, width_hi):
    lims = robot.joint_limits().limits
    intervals = []
    for lim in lims:
        width = rng.uniform(width_lo, width_hi)
        lo = rng.uniform(lim.lo, max(lim.lo, lim.hi - width))
        hi = min(lo + width, lim.hi)
        intervals.append(sbf5.Interval(float(lo), float(hi)))
    return intervals


def make_env_cfg(env_name, sub, delta=None):
    cfg = sbf5.EnvelopeTypeConfig()
    if env_name == "LinkIAABB":
        cfg.type = sbf5.EnvelopeType.LinkIAABB
    elif env_name == "LinkIAABB_Grid":
        cfg.type = sbf5.EnvelopeType.LinkIAABB_Grid
    else:
        raise ValueError(f"Unsupported envelope: {env_name}")
    cfg.n_subdivisions = int(sub)
    if delta is not None:
        cfg.grid_config.voxel_delta = float(delta)
    return cfg


def run_one(robot, boxes, ep_cfg, env_cfg, repeats):
    volumes = []
    total_us = []
    cache_bricks = []
    cache_voxels = []
    cache_payload = []

    for bi, b in enumerate(boxes):
        for rep in range(repeats):
            info = sbf5.compute_envelope_info(robot, b, ep_cfg, env_cfg, None)
            total_us.append(float(info["total_time_us"]))
            if rep == 0:
                volumes.append(float(info["volume"]))
                cache_bricks.append(float(info.get("grid_num_bricks", 0)))
                cache_voxels.append(float(info.get("grid_num_voxels", 0)))
                cache_payload.append(float(info.get("grid_cache_payload_bytes", 0.0)))

    return {
        "volume_mean": float(np.mean(volumes)),
        "volume_std": float(np.std(volumes)),
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
        "--width-bin",
        type=str,
        default="all",
        help="Run only one width bin by name (e.g., W1_0.10_0.20) or 'all'",
    )
    parser.add_argument("--out", type=str,
                        default="experiments/results_iiwa14_final/s0c_width_sub_grid/results.json")
    args = parser.parse_args()

    out_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    robot = sbf5.Robot.from_json("data/iiwa14.json")

    ep_cfg = sbf5.EndpointSourceConfig()
    ep_cfg.source = sbf5.EndpointSource.CritSample

    rows = []
    t0 = time.time()

    selected_bins = WIDTH_BINS
    if args.width_bin != "all":
        selected_bins = [w for w in WIDTH_BINS if w[0] == args.width_bin]
        if not selected_bins:
            raise ValueError(
                f"Unknown width bin '{args.width_bin}'. "
                f"Available: {[w[0] for w in WIDTH_BINS]}"
            )

    for wi, (w_name, w_lo, w_hi) in enumerate(selected_bins):
        rng = np.random.RandomState(4200 + wi)
        boxes = [random_intervals(robot, rng, w_lo, w_hi) for _ in range(args.n_boxes)]

        # LinkIAABB: sweep sub only
        for sub in SUB_VALUES:
            env_cfg = make_env_cfg("LinkIAABB", sub, None)
            stats = run_one(robot, boxes, ep_cfg, env_cfg, args.repeats)
            rows.append({
                "width_bin": w_name,
                "width_lo": float(w_lo),
                "width_hi": float(w_hi),
                "endpoint": "CritSample",
                "envelope": "LinkIAABB",
                "subdivisions": int(sub),
                "grid_delta": None,
                "n_boxes": int(args.n_boxes),
                "repeats": int(args.repeats),
                **stats,
            })
            print(f"[done] {w_name} LinkIAABB sub={sub}")

        # LinkIAABB_Grid: sweep sub × delta
        for sub in SUB_VALUES:
            for delta in GRID_DELTAS:
                env_cfg = make_env_cfg("LinkIAABB_Grid", sub, delta)
                stats = run_one(robot, boxes, ep_cfg, env_cfg, args.repeats)
                rows.append({
                    "width_bin": w_name,
                    "width_lo": float(w_lo),
                    "width_hi": float(w_hi),
                    "endpoint": "CritSample",
                    "envelope": "LinkIAABB_Grid",
                    "subdivisions": int(sub),
                    "grid_delta": float(delta),
                    "n_boxes": int(args.n_boxes),
                    "repeats": int(args.repeats),
                    **stats,
                })
                print(f"[done] {w_name} LinkIAABB_Grid sub={sub} delta={delta:.2f}")

    payload = {
        "meta": {
            "robot": "iiwa14",
            "endpoint": "CritSample",
            "width_bins": [
                {"name": n, "lo": lo, "hi": hi}
                for (n, lo, hi) in selected_bins
            ],
            "width_bin_mode": args.width_bin,
            "sub_values": SUB_VALUES,
            "grid_deltas": GRID_DELTAS,
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
