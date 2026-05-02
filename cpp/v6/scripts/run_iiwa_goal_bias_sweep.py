#!/usr/bin/env python3
"""Sweep RRT goal bias on the IIWA14 combined scene and rank settings.

The sweep reuses the paper's IIWA14 combined scene, five canonical seed
configurations, and five benchmark query pairs. Candidates are ranked by:
1. Query success rate (higher is better)
2. Median build time (lower is better)
3. Median successful-query time (lower is better)
4. Median successful path length (lower is better)
"""

import argparse
import json
import os
import statistics
import sys
import time
from pathlib import Path

import numpy as np

from run_online_query_comparison import (
    DEFAULT_GOAL_BIAS,
    IIWA_CONFIGS,
    QUERY_PAIRS,
    SBF_BUILD_DIR,
    SBF_DATA_DIR,
    make_combined_obstacles,
)


def _median_or_nan(values):
    return float(statistics.median(values)) if values else float("nan")


def _mean_or_nan(values):
    return float(statistics.fmean(values)) if values else float("nan")


def _load_sbf6():
    if SBF_BUILD_DIR not in sys.path:
        sys.path.insert(0, SBF_BUILD_DIR)
    import _sbf6_cpp as sbf6
    return sbf6


def run_bias(goal_bias, seeds, timeout_ms, max_boxes, bridge_threads):
    sbf6 = _load_sbf6()
    robot = sbf6.Robot.from_json(os.path.join(SBF_DATA_DIR, "iiwa14.json"))
    obstacles = make_combined_obstacles()
    seed_points = [IIWA_CONFIGS[k] for k in ["AS", "TS", "CS", "LB", "RB"]]

    build_times = []
    build_boxes = []
    query_times = []
    path_lengths = []
    success_count = 0
    total_queries = 0
    per_seed = []

    for seed in range(seeds):
        cfg = sbf6.SBFPlannerConfig()
        cfg.grower.timeout_ms = timeout_ms
        cfg.grower.max_boxes = max_boxes
        cfg.grower.post_connect_extra_boxes = 4000
        cfg.grower.rng_seed = seed * 1000 + 42
        cfg.grower.n_threads = 5
        cfg.grower.bridge_n_threads = bridge_threads
        cfg.grower.max_consecutive_miss = 2000
        cfg.grower.rrt_goal_bias = goal_bias
        cfg.grower.rrt_step_ratio = 0.05
        cfg.grower.ffb_config.max_depth = 300
        cfg.coarsen.target_boxes = 300
        cfg.coarsen.score_threshold = 500
        cfg.lect_no_cache = True

        planner = sbf6.SBFPlanner(robot, cfg)
        t0 = time.perf_counter()
        planner.build_coverage(obstacles, timeout_ms, seed_points)
        build_time = time.perf_counter() - t0
        n_boxes = planner.n_boxes()

        seed_result = {
            "seed": seed,
            "build_time_s": build_time,
            "n_boxes": n_boxes,
            "queries": [],
        }
        build_times.append(build_time)
        build_boxes.append(n_boxes)

        for label, start_name, goal_name in QUERY_PAIRS:
            total_queries += 1
            q_s = IIWA_CONFIGS[start_name]
            q_g = IIWA_CONFIGS[goal_name]
            t1 = time.perf_counter()
            result = planner.query(q_s, q_g)
            query_time = time.perf_counter() - t1
            success = bool(result.success)
            if success:
                success_count += 1
                query_times.append(query_time)
                path_lengths.append(float(result.path_length))
            seed_result["queries"].append({
                "label": label,
                "success": success,
                "query_time_s": query_time,
                "path_length": float(result.path_length) if success else None,
                "planning_time_ms": float(result.planning_time_ms),
            })

        per_seed.append(seed_result)

    summary = {
        "goal_bias": goal_bias,
        "n_seeds": seeds,
        "success_rate": success_count / max(total_queries, 1),
        "median_build_time_s": _median_or_nan(build_times),
        "mean_build_time_s": _mean_or_nan(build_times),
        "median_boxes": _median_or_nan(build_boxes),
        "median_query_time_s": _median_or_nan(query_times),
        "median_path_length": _median_or_nan(path_lengths),
        "successful_queries": success_count,
        "total_queries": total_queries,
        "per_seed": per_seed,
    }
    return summary


def rank_key(result):
    median_query = result["median_query_time_s"]
    median_path = result["median_path_length"]
    if np.isnan(median_query):
        median_query = float("inf")
    if np.isnan(median_path):
        median_path = float("inf")
    return (
        -result["success_rate"],
        result["median_build_time_s"],
        median_query,
        median_path,
        abs(result["goal_bias"] - DEFAULT_GOAL_BIAS),
    )


def parse_args():
    parser = argparse.ArgumentParser(
        description="Sweep IIWA14 combined-scene goal bias settings"
    )
    parser.add_argument(
        "--goal-biases",
        type=float,
        nargs="+",
        default=[0.02, 0.05, 0.08, 0.10, 0.12, 0.15, 0.20, 0.30],
        help="Candidate goal bias values to evaluate",
    )
    parser.add_argument("--seeds", type=int, default=5)
    parser.add_argument("--timeout-ms", type=float, default=60000.0)
    parser.add_argument("--max-boxes", type=int, default=200000)
    parser.add_argument("--bridge-threads", type=int, default=16)
    parser.add_argument(
        "--output-json",
        default=None,
        help="Optional JSON path. Defaults to experiments/results_goal_bias/iiwa14_goal_bias_sweep_<ts>.json",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    root = Path(__file__).resolve().parents[1]
    out_dir = root / "experiments" / "results_goal_bias"
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.output_json:
        output_json = Path(args.output_json)
    else:
        ts = time.strftime("%Y%m%d_%H%M%S")
        output_json = out_dir / f"iiwa14_goal_bias_sweep_{ts}.json"

    results = []
    print(f"IIWA14 goal-bias sweep on combined scene: {args.goal_biases}")
    print(f"Seeds={args.seeds}, timeout_ms={args.timeout_ms}, max_boxes={args.max_boxes}")
    for goal_bias in args.goal_biases:
        print(f"\n[goal_bias={goal_bias:.3f}] running...")
        result = run_bias(
            goal_bias=goal_bias,
            seeds=args.seeds,
            timeout_ms=args.timeout_ms,
            max_boxes=args.max_boxes,
            bridge_threads=args.bridge_threads,
        )
        results.append(result)
        print(
            "  success={:.1%} build_med={:.3f}s query_med={} path_med={}".format(
                result["success_rate"],
                result["median_build_time_s"],
                "nan" if np.isnan(result["median_query_time_s"]) else f"{result['median_query_time_s']:.3f}s",
                "nan" if np.isnan(result["median_path_length"]) else f"{result['median_path_length']:.3f}",
            )
        )

    ranked = sorted(results, key=rank_key)
    best = ranked[0] if ranked else None
    payload = {
        "meta": {
            "scene": "iiwa14_combined",
            "query_pairs": [label for label, _, _ in QUERY_PAIRS],
            "default_goal_bias": DEFAULT_GOAL_BIAS,
            "goal_biases": args.goal_biases,
            "n_seeds": args.seeds,
            "timeout_ms": args.timeout_ms,
            "max_boxes": args.max_boxes,
            "bridge_threads": args.bridge_threads,
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
        },
        "ranked_results": ranked,
        "recommended": best,
    }
    output_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")

    print("\nRanking:")
    for idx, result in enumerate(ranked, start=1):
        query_med = (
            "nan"
            if np.isnan(result["median_query_time_s"])
            else f"{result['median_query_time_s']:.3f}s"
        )
        print(
            f"  {idx}. bias={result['goal_bias']:.3f} "
            f"success={result['success_rate']:.1%} "
            f"build_med={result['median_build_time_s']:.3f}s "
            f"query_med={query_med}"
        )
    if best is not None:
        print(
            "\nRecommended goal bias: {:.3f} "
            "(success={:.1%}, median build={:.3f}s, median query={})".format(
                best["goal_bias"],
                best["success_rate"],
                best["median_build_time_s"],
                "nan" if np.isnan(best["median_query_time_s"]) else f"{best['median_query_time_s']:.3f}s",
            )
        )
    print(f"Results written to {output_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())