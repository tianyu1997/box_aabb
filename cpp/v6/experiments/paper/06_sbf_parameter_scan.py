#!/usr/bin/env python3
"""Focused parameter scan for the paper SBF multi-goal grower."""
from __future__ import annotations

import argparse
import importlib.util
import json
import statistics
import sys
import time
from itertools import product
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import PAPER_THREADS, ROOT, add_common_args, mode_args, require_python_extension, write_json


AUTHORITATIVE_SCRIPT = ROOT / "scripts" / "run_online_query_comparison.py"


def csv_floats(raw: str) -> list[float]:
    return [float(part.strip()) for part in raw.split(",") if part.strip()]


def csv_ints(raw: str) -> list[int]:
    return [int(part.strip()) for part in raw.split(",") if part.strip()]


def mean(values: list[float]) -> float | None:
    return float(statistics.fmean(values)) if values else None


def median(values: list[float]) -> float | None:
    return float(statistics.median(values)) if values else None


def load_authoritative_module() -> Any:
    spec = importlib.util.spec_from_file_location("v6_online_query", AUTHORITATIVE_SCRIPT)
    if spec is None or spec.loader is None:
        raise ImportError(f"unable to load {AUTHORITATIVE_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def summarize_rows(rows: list[dict[str, Any]]) -> tuple[list[dict[str, Any]], dict[str, Any] | None]:
    by_config: dict[tuple[float, float, int], list[dict[str, Any]]] = {}
    for row in rows:
        key = (
            float(row["goal_bias"]),
            float(row["unexplored_sample_prob"]),
            int(row["max_consecutive_miss"]),
        )
        by_config.setdefault(key, []).append(row)

    summaries: list[dict[str, Any]] = []
    for (goal_bias, unexplored, max_miss), config_rows in sorted(by_config.items()):
        total_queries = 5 * len(config_rows)
        successes = sum(int(row["successes"]) for row in config_rows)
        build_samples = [float(row["build_s"]) for row in config_rows]
        query_samples = [float(row["query_time_ms_sum"]) / 1000.0 for row in config_rows]
        box_samples = [float(row["n_boxes"]) for row in config_rows]
        summaries.append({
            "goal_bias": goal_bias,
            "unexplored_sample_prob": unexplored,
            "max_consecutive_miss": max_miss,
            "seeds": len(config_rows),
            "successes": successes,
            "total_queries": total_queries,
            "success_rate": float(successes) / total_queries if total_queries else 0.0,
            "build_s_mean": mean(build_samples),
            "build_s_median": median(build_samples),
            "query_s_sum_mean": mean(query_samples),
            "query_s_sum_median": median(query_samples),
            "n_boxes_mean": mean(box_samples),
            "n_boxes_median": median(box_samples),
        })

    ranked = sorted(
        summaries,
        key=lambda item: (
            -float(item["success_rate"]),
            float(item["query_s_sum_mean"] if item["query_s_sum_mean"] is not None else 1e99),
            float(item["build_s_mean"] if item["build_s_mean"] is not None else 1e99),
            float(item["n_boxes_mean"] if item["n_boxes_mean"] is not None else 1e99),
        ),
    )
    return summaries, (ranked[0] if ranked else None)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--goal-bias", default="0.2,0.4,0.6")
    parser.add_argument("--unexplored", default="0.5,0.3,0.2")
    parser.add_argument("--max-miss", default="5000,10000")
    parser.add_argument("--max-boxes", type=int, default=32000)
    parser.add_argument("--ffb-depth", type=int, default=220)
    parser.add_argument("--threads", type=int, default=PAPER_THREADS)
    args = parser.parse_args()

    python_dir = require_python_extension(args)
    if str(python_dir) not in sys.path:
        sys.path.insert(0, str(python_dir))
    if str(ROOT / "python") not in sys.path:
        sys.path.insert(0, str(ROOT / "python"))
    import _sbf6_cpp as sbf5

    module = load_authoritative_module()
    seeds, timeout, mode = mode_args(args, quick_seeds=1, full_seeds=3,
                                     quick_timeout=12, full_timeout=60)
    out_path = args.out_dir / "sbf_parameter_scan.json"

    combos = list(product(csv_floats(args.goal_bias),
                          csv_floats(args.unexplored),
                          csv_ints(args.max_miss)))
    if args.dry_run:
        print(json.dumps({"mode": mode, "seeds": seeds, "timeout": timeout,
                          "combos": combos, "out": str(out_path)}, indent=2))
        return 0

    robot = sbf5.Robot.from_json(str(ROOT / "data" / "iiwa14.json"))
    obstacles = module.make_combined_obstacles()
    seed_points = [module.IIWA_CONFIGS[k] for k in module.DEFAULT_SBF_SEED_ORDER]

    rows: list[dict[str, Any]] = []
    for goal_bias, unexplored, max_miss in combos:
        for seed in range(seeds):
            print(
                "[scan] "
                f"seed={seed} goal_bias={goal_bias:g} "
                f"unexplored={unexplored:g} max_miss={max_miss}"
            )
            config = sbf5.SBFPlannerConfig()
            module.apply_paper_sbf_architecture(
                config,
                seed=seed,
                grow_timeout_ms=float(timeout) * 1000.0,
                max_boxes=args.max_boxes,
                post_connect_extra_boxes=1000,
                n_threads=args.threads,
                ffb_depth=args.ffb_depth,
                goal_bias=goal_bias,
                lect_no_cache=True,
            )
            config.grower.unexplored_sample_prob = unexplored
            config.grower.max_consecutive_miss = max_miss
            planner = sbf5.SBFPlanner(robot, config)
            t0 = time.perf_counter()
            planner.build_coverage(obstacles, timeout * 1000, seed_points)
            build_s = time.perf_counter() - t0
            query_rows = []
            for label, start_name, goal_name in module.QUERY_PAIRS:
                result = planner.query(module.IIWA_CONFIGS[start_name],
                                       module.IIWA_CONFIGS[goal_name])
                query_rows.append({
                    "name": label,
                    "success": bool(result.success),
                    "planning_time_ms": float(result.planning_time_ms),
                    "path_length": float(result.path_length) if result.success else None,
                })
            rows.append({
                "seed": seed,
                "goal_bias": goal_bias,
                "unexplored_sample_prob": unexplored,
                "max_consecutive_miss": max_miss,
                "build_s": build_s,
                "n_boxes": int(planner.n_boxes()),
                "successes": sum(1 for row in query_rows if row["success"]),
                "query_time_ms_sum": sum(row["planning_time_ms"] for row in query_rows),
                "queries": query_rows,
            })

    summary, best_config = summarize_rows(rows)

    payload = {
        "experiment": "sbf_parameter_scan",
        "mode": mode or "default-full",
        "seeds": seeds,
        "timeout_s": timeout,
        "max_boxes": args.max_boxes,
        "ffb_depth": args.ffb_depth,
        "threads": args.threads,
        "selection_rule": "max_success_rate_then_min_mean_query_sum_then_min_mean_build_time_then_min_mean_box_count",
        "summary": summary,
        "best_config": best_config,
        "rows": rows,
    }
    write_json(out_path, payload)
    print(f"[write] {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
