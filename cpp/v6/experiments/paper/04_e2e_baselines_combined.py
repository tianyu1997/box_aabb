#!/usr/bin/env python3
"""Paper Exp. 4 — Marcucci combined-scene SBF workload.

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
import subprocess
import sys
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parent))
from common import (
    PAPER_THREADS,
    PAPER_STATISTICS_POLICY,
    ROOT,
    add_common_args,
    current_cpu_affinity,
    mode_args,
    require_python_extension,
    write_json,
)


AUTHORITATIVE_SCRIPT = ROOT / "scripts" / "run_online_query_comparison.py"
PYTHON_EXTENSION_DIR = ROOT / "build-release" / "python"
BASELINE_WRAPPER = ROOT / "experiments" / "paper" / "04_baselines_marcucci.py"


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


def run_exp4_baselines(args: argparse.Namespace, mode_flag: str) -> None:
    if args.skip_baselines:
        return
    cmd: list[str | Path] = [
        sys.executable,
        BASELINE_WRAPPER,
        mode_flag,
        "--out-dir",
        args.out_dir,
        "--methods",
        args.baseline_methods,
        "--logical-threads",
        str(PAPER_THREADS),
        "--bitstar-budget-s",
        str(args.bitstar_budget_s),
        "--prm-build-budget-s",
        str(args.prm_build_budget_s),
        "--prm-query-budget-s",
        str(args.prm_query_budget_s),
        "--iris-zo-query-time-limit-s",
        str(args.iris_zo_query_time_limit_s),
    ]
    if args.build_dir is not None:
        cmd += ["--build-dir", args.build_dir]
    if args.allow_debug_build:
        cmd.append("--allow-debug-build")
    if args.dry_run:
        cmd.append("--dry-run")
        print("$", " ".join(str(part) for part in cmd))
        return
    subprocess.run([str(part) for part in cmd], check=True, cwd=ROOT)


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
            "prebridge_time_s": float(row.get("prebridge_time_s", 0.0)),
            "prebridge_added_boxes": int(row.get("prebridge_added_boxes", 0)),
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
                    "prebridge_time_s": None,
                    "prebridge_added_boxes": None,
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
    prebridge_time_samples = [float(row.get("prebridge_time_s", 0.0)) for row in build_results]
    prebridge_added_samples = [float(row.get("prebridge_added_boxes", 0)) for row in build_results]
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
            "mean_prebridge_time_s": mean(prebridge_time_samples),
            "median_prebridge_time_s": median(prebridge_time_samples),
            "mean_prebridge_added_boxes": mean(prebridge_added_samples),
            "median_prebridge_added_boxes": median(prebridge_added_samples),
            "mean_dedup_box_volume_sum": mean(dedup_box_volume_samples),
            "median_dedup_box_volume_sum": median(dedup_box_volume_samples),
        },
        "queries": queries_summary,
        "trials": ordered_trials,
    }

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    add_common_args(parser)
    parser.add_argument("--threads", type=int, default=PAPER_THREADS)
    parser.add_argument("--bridge-threads", type=int, default=PAPER_THREADS)
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
    parser.add_argument("--point-bridge-timeout-ms", type=float, default=200.0)
    parser.add_argument("--no-point-bridge", action="store_true",
                        help="disable point-level RRT bridge fallback")
    parser.add_argument("--prebridge-query-pairs", action=argparse.BooleanOptionalAction, default=None,
                        help="override whether prebridge query pairs are run during build")
    parser.add_argument("--prebridge-per-pair-timeout-ms", type=float, default=None,
                        help="override prebridge timeout per query pair in milliseconds")
    parser.add_argument("--prebridge-max-pairs-per-call", type=int, default=None,
                        help="override max bridge candidates per prebridge call")
    parser.add_argument("--prebridge-max-query-pairs", type=int, default=None,
                        help="override how many query pairs are prebridged during build")
    parser.add_argument("--include-anytime", action="store_true",
                        help="deprecated no-op retained for CLI compatibility; the v6 wrapper emits only the SBF row")
    parser.add_argument("--skip-baselines", action="store_true",
                        help="run only the Exp.4 SBF row; by default this entry script also runs Exp.4 baselines")
    parser.add_argument("--baseline-methods", default="iris_np,ompl")
    parser.add_argument("--bitstar-budget-s", type=float, default=10.0)
    parser.add_argument("--prm-build-budget-s", type=float, default=10.0)
    parser.add_argument("--prm-query-budget-s", type=float, default=2.0)
    parser.add_argument("--iris-zo-query-time-limit-s", type=float, default=120.0)
    args = parser.parse_args()

    global PYTHON_EXTENSION_DIR
    PYTHON_EXTENSION_DIR = require_python_extension(args)

    seeds, timeout, _mode = mode_args(args, quick_seeds=3, full_seeds=10,
                                      quick_timeout=30, full_timeout=60)
    mode_flag = "--quick" if args.quick else "--full"
    out_path = args.out_dir / args.output_name
    if args.dry_run:
        print(
            f"$ {sys.executable} {AUTHORITATIVE_SCRIPT} --seeds {seeds} --json {out_path}"
        )
        print(f"[dry-run] would write {out_path}")
        run_exp4_baselines(args, mode_flag)
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
        prebridge_query_pairs=(
            module.DEFAULT_SBF_PREBRIDGE_QUERY_PAIRS
            if args.prebridge_query_pairs is None else bool(args.prebridge_query_pairs)
        ),
        prebridge_per_pair_timeout_ms=(
            module.DEFAULT_SBF_PREBRIDGE_PER_PAIR_TIMEOUT_MS
            if args.prebridge_per_pair_timeout_ms is None else float(args.prebridge_per_pair_timeout_ms)
        ),
        prebridge_max_pairs_per_call=(
            module.DEFAULT_SBF_PREBRIDGE_MAX_PAIRS_PER_CALL
            if args.prebridge_max_pairs_per_call is None else int(args.prebridge_max_pairs_per_call)
        ),
        prebridge_max_query_pairs=(
            module.DEFAULT_SBF_PREBRIDGE_MAX_QUERY_PAIRS
            if args.prebridge_max_query_pairs is None else int(args.prebridge_max_query_pairs)
        ),
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
        "prebridge_query_pairs": (
            module.DEFAULT_SBF_PREBRIDGE_QUERY_PAIRS
            if args.prebridge_query_pairs is None else bool(args.prebridge_query_pairs)
        ),
        "prebridge_per_pair_timeout_ms": (
            module.DEFAULT_SBF_PREBRIDGE_PER_PAIR_TIMEOUT_MS
            if args.prebridge_per_pair_timeout_ms is None else float(args.prebridge_per_pair_timeout_ms)
        ),
        "prebridge_max_pairs_per_call": (
            module.DEFAULT_SBF_PREBRIDGE_MAX_PAIRS_PER_CALL
            if args.prebridge_max_pairs_per_call is None else int(args.prebridge_max_pairs_per_call)
        ),
        "prebridge_max_query_pairs": (
            module.DEFAULT_SBF_PREBRIDGE_MAX_QUERY_PAIRS
            if args.prebridge_max_query_pairs is None else int(args.prebridge_max_query_pairs)
        ),
        "logical_threads": int(args.threads),
        "resource_policy": {
            "logical_threads": int(args.threads),
            "bridge_threads": int(args.bridge_threads),
            "cpu_affinity": current_cpu_affinity(),
            "seed_execution": "serial",
        },
        "statistical_policy": PAPER_STATISTICS_POLICY["exp4"],
        "cpu_affinity": current_cpu_affinity(),
        "seed_execution": "serial",
    })
    payload = normalize_v6_authoritative_sbf(
        build_results,
        query_results,
        seeds=int(seeds),
        architecture=architecture,
    )
    payload["resource_policy"] = architecture["resource_policy"]
    payload["statistical_policy"] = PAPER_STATISTICS_POLICY["exp4"]
    write_json(out_path, payload)
    print(f"[write] {out_path}")
    run_exp4_baselines(args, mode_flag)


if __name__ == "__main__":
    main()
