#!/usr/bin/env python3
"""Paper Exp. 3 — Marcucci combined-scene SBF workload.

Environment: Marcucci combined IIWA14 scene only.  The five JSON files in
`experiments/configs/marcucci/` are the canonical query pairs in the same
16-obstacle combined scene; the paper reports them as one workload, not as
separate sub-scenes.

Outputs:
    - experiments/results_paper/marcucci_combined.json
    - optional diagnostic outputs selected by --output-name

This v6 wrapper uses the authoritative build_coverage + cached-query protocol
implemented in scripts/run_online_query_comparison.py and emits the current
paper JSON schema for the SBF row only.
"""
from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import ROOT, add_common_args, mode_args, require_python_extension, write_json


AUTHORITATIVE_SCRIPT = ROOT / "scripts" / "run_online_query_comparison.py"
PYTHON_EXTENSION_DIR = ROOT / "build" / "python"


def mean(values: list[float]) -> float | None:
    return (sum(values) / len(values)) if values else None


def median(values: list[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = len(ordered) // 2
    if len(ordered) % 2 == 1:
        return ordered[index]
    return 0.5 * (ordered[index - 1] + ordered[index])


def ensure_python_paths() -> None:
    for candidate in (PYTHON_EXTENSION_DIR, ROOT / "python"):
        text = str(candidate)
        if candidate.exists() and text not in sys.path:
            sys.path.insert(0, text)


def load_authoritative_module() -> Any:
    ensure_python_paths()
    spec = importlib.util.spec_from_file_location(
        "v6_authoritative_online_query_comparison",
        AUTHORITATIVE_SCRIPT,
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"Unable to load authoritative script at {AUTHORITATIVE_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def normalize_v6_authoritative_sbf(
    build_results: list[dict[str, Any]],
    query_results: dict[str, list[dict[str, Any]]],
    *,
    seeds: int,
    architecture: dict[str, Any] | None = None,
) -> dict[str, Any]:
    query_order = list(query_results)
    trials_by_seed: dict[int, dict[str, Any]] = {}
    build_samples: list[float] = []

    for row in build_results:
        seed = int(row["seed"])
        build_s = float(row["build_time_s"])
        build_samples.append(build_s)
        trials_by_seed[seed] = {
            "seed": seed,
            "seed_index": seed,
            "build_s": build_s,
            "n_boxes": int(row["n_boxes"]),
            "unique_box_count": int(row.get("unique_box_count", row["n_boxes"])),
            "duplicate_box_count": int(row.get("duplicate_box_count", 0)),
            "box_volume_sum": float(row.get("box_volume_sum", 0.0)),
            "dedup_box_volume_sum": float(row.get("dedup_box_volume_sum", row.get("box_volume_sum", 0.0))),
            "duplicate_box_volume_sum": float(row.get("duplicate_box_volume_sum", 0.0)),
            "queries": [],
        }

    queries_summary: list[dict[str, Any]] = []
    for label in query_order:
        start_name, goal_name = label.split("->", 1)
        rows = query_results.get(label, [])
        successes = [row for row in rows if row.get("success")]
        for row in rows:
            seed = int(row["seed"])
            trial = trials_by_seed.setdefault(
                seed,
                {
                    "seed": seed,
                    "seed_index": seed,
                    "build_s": None,
                    "n_boxes": None,
                    "unique_box_count": None,
                    "duplicate_box_count": None,
                    "box_volume_sum": None,
                    "dedup_box_volume_sum": None,
                    "duplicate_box_volume_sum": None,
                    "queries": [],
                },
            )
            query_payload = {
                "from": start_name,
                "to": goal_name,
                "t_s": float(row["time_s"]),
                "ok": bool(row["success"]),
                "length": float(row["path_length"]) if row.get("success") else 0.0,
            }
            planning_time_ms = row.get("planning_time_ms")
            if planning_time_ms is not None:
                query_payload["planning_time_ms"] = float(planning_time_ms)
            trial["queries"].append(query_payload)

        success_values = [1.0 if row.get("success") else 0.0 for row in rows]
        queries_summary.append(
            {
                "name": label,
                "sr": mean(success_values),
                "t_med_s": median([float(row["time_s"]) for row in successes]) if successes else None,
                "len_med": median([float(row["path_length"]) for row in successes]) if successes else None,
            }
        )

    ordered_trials = [trials_by_seed[seed] for seed in sorted(trials_by_seed)]
    dedup_box_volume_samples = [
        float(row.get("dedup_box_volume_sum", row.get("box_volume_sum", 0.0)))
        for row in build_results
    ]
    unique_box_count_samples = [float(row.get("unique_box_count", row["n_boxes"])) for row in build_results]
    return {
        "experiment": "marcucci",
        "robot": "iiwa14",
        "scene": "marcucci_combined",
        "source_protocol": "v6_authoritative_build_coverage_query",
        "source_script": str(AUTHORITATIVE_SCRIPT),
        "seeds": seeds,
        "params": architecture or {},
        "build": {
            "mean_s": mean(build_samples),
            "median_s": median(build_samples),
            "mean_unique_box_count": mean(unique_box_count_samples),
            "median_unique_box_count": median(unique_box_count_samples),
            "mean_dedup_box_volume_sum": mean(dedup_box_volume_samples),
            "median_dedup_box_volume_sum": median(dedup_box_volume_samples),
        },
        "queries": queries_summary,
        "trials": ordered_trials,
    }

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--threads", type=int, default=5)
    parser.add_argument("--bridge-threads", type=int, default=16)
    parser.add_argument("--env", default="hull16_grid",
                        help="deprecated no-op kept for CLI compatibility; Exp.3 uses the authoritative v6 build/query protocol")
    parser.add_argument("--n-sub", type=int, default=1,
                        help="deprecated no-op kept for CLI compatibility; Exp.3 uses the authoritative v6 build/query protocol")
    parser.add_argument("--ffb-depth", type=int, default=300)
    parser.add_argument("--max-boxes", type=int, default=200000)
    parser.add_argument("--bridge-boxes", type=int, default=4000)
    parser.add_argument("--goal-bias", type=float, default=0.1)
    parser.add_argument("--unexplored", type=float, default=0.7)
    parser.add_argument("--max-miss", type=int, default=2000)
    parser.add_argument("--enable-partitioned", action="store_true",
                        help="diagnostic: enable the partitioned shared-LECT grower")
    parser.add_argument("--partitioned-box-budget-per-tree", type=int, default=0,
                        help="diagnostic: per-root box budget for partitioned shared-LECT; 0 derives from the existing post-connect budget")
    parser.add_argument("--disable-coordinated", action="store_true",
                        help="diagnostic: disable coordinated multi-goal growth")
    parser.add_argument("--output-name", default="marcucci_combined.json",
                        help="result JSON filename under --out-dir")
    parser.add_argument("--point-bridge-timeout-ms", type=float, default=20000.0)
    parser.add_argument("--no-point-bridge", action="store_true",
                        help="disable point-level RRT bridge fallback")
    parser.add_argument("--include-anytime", action="store_true",
                        help="deprecated no-op retained for CLI compatibility; the v6 wrapper emits only the SBF row")
    args = parser.parse_args()

    global PYTHON_EXTENSION_DIR
    PYTHON_EXTENSION_DIR = require_python_extension(args)

    seeds, timeout, _mode = mode_args(args, quick_seeds=3, full_seeds=10,
                                      quick_timeout=30, full_timeout=60)
    out_path = args.out_dir / args.output_name
    if args.dry_run:
        print(
            f"$ {sys.executable} {AUTHORITATIVE_SCRIPT} --seeds {seeds} --json {out_path}"
        )
        print(f"[dry-run] would write {out_path}")
        return

    module = load_authoritative_module()
    build_results, query_results = module.run_sbf_experiment(
        int(seeds),
        grow_timeout_ms=float(timeout) * 1000.0,
        max_boxes=int(args.max_boxes),
        post_connect_extra_boxes=int(args.bridge_boxes),
        n_threads=int(args.threads),
        bridge_n_threads=int(args.bridge_threads),
        ffb_depth=int(args.ffb_depth),
        goal_bias=float(args.goal_bias),
        unexplored_sample_prob=float(args.unexplored),
        max_consecutive_miss=int(args.max_miss),
        enable_partitioned_lect_parallel=bool(args.enable_partitioned),
        partitioned_box_budget_per_tree=int(args.partitioned_box_budget_per_tree),
        enable_coordinated_multi_goal=not bool(args.disable_coordinated),
    )
    architecture = module.paper_sbf_architecture_summary()
    architecture.update({
        "grow_timeout_ms": float(timeout) * 1000.0,
        "max_boxes": int(args.max_boxes),
        "post_connect_extra_boxes": int(args.bridge_boxes),
        "n_threads": int(args.threads),
        "bridge_n_threads": int(args.bridge_threads),
        "ffb_depth": int(args.ffb_depth),
        "goal_bias": float(args.goal_bias),
        "unexplored_sample_prob": float(args.unexplored),
        "max_consecutive_miss": int(args.max_miss),
        "enable_partitioned_lect_parallel": bool(args.enable_partitioned),
        "partitioned_box_budget_per_tree": int(args.partitioned_box_budget_per_tree),
        "enable_coordinated_multi_goal": not bool(args.disable_coordinated),
    })
    payload = normalize_v6_authoritative_sbf(
        build_results,
        query_results,
        seeds=int(seeds),
        architecture=architecture,
    )
    write_json(out_path, payload)
    print(f"[write] {out_path}")


if __name__ == "__main__":
    main()
