#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import os
from statistics import mean
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Any

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from common import (
    RESULTS_PAPER,
    ROOT,
    add_logical_threads_arg,
    add_mode_args,
    aggregate_method_trials,
    marcucci_workload,
    ordered_parallel_map,
    resolve_experiment_binary,
    resolve_mode,
    write_json,
)

def detect_baseline_bin() -> Path:
    return resolve_experiment_binary("baseline_ompl")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    add_mode_args(parser)
    add_logical_threads_arg(parser)
    parser.add_argument("--out-dir", type=Path, default=RESULTS_PAPER)
    parser.add_argument("--baseline-bin", type=Path, default=None)
    parser.add_argument(
        "--bitstar-budget-s",
        type=float,
        default=1.0,
        help="fixed wall-clock budget for the BIT* comparison",
    )
    parser.add_argument(
        "--methods",
        default="prm,bitstar_budget",
        help="comma-separated subset of prm, bitstar_budget",
    )
    return parser.parse_args()


def normalize_methods(raw: str) -> list[str]:
    allowed = {"prm", "bitstar_budget"}
    methods = [part.strip() for part in raw.split(",") if part.strip()]
    unknown = [method for method in methods if method not in allowed]
    if unknown:
        raise ValueError(f"unknown methods: {unknown}")
    return methods


def method_output_name(method: str) -> str:
    return {
        "prm": "marcucci_ompl_prm.json",
        "bitstar_budget": "marcucci_ompl_bitstar_budget.json",
    }[method]


def method_payload_name(method: str) -> str:
    return {
        "prm": "ompl_prm",
        "bitstar_budget": "ompl_bitstar_budget",
    }[method]


def current_cpu_affinity() -> list[int] | None:
    if not hasattr(os, "sched_getaffinity"):
        return None
    return sorted(int(cpu) for cpu in os.sched_getaffinity(0))


def run_trial(
    *,
    baseline_bin: Path,
    workload_item: dict[str, Any],
    method: str,
    seed: int,
    timeout_s: float,
) -> dict[str, Any]:
    planner = "prm" if method == "prm" else "bit_star"
    cmd = [
        str(baseline_bin),
        f"--scene={workload_item['path']}",
        "--seeds=1",
        f"--seed-base={42 + seed}",
        f"--timeout={timeout_s}",
        f"--planner={planner}",
        "--no-simplify",
    ]

    with tempfile.TemporaryDirectory(prefix="marcucci_ompl_") as tmpdir:
        out_path = Path(tmpdir) / "result.json"
        cmd.append(f"--out={out_path}")
        grace_s = min(0.1, max(0.01, 0.1 * timeout_s))
        try:
            completed = subprocess.run(
                cmd,
                check=True,
                cwd=ROOT,
                capture_output=True,
                text=True,
                timeout=timeout_s + grace_s,
            )
        except subprocess.TimeoutExpired:
            return {
                "query": workload_item["label"],
                "file": workload_item["file"],
                "seed": seed,
                "success": False,
                "time_s": None,
                "build_time_s": timeout_s if planner == "prm" else None,
                "path_length": None,
                "planner": planner,
                "status": "ExternalTimeout",
                "budget_s": timeout_s,
                "note": f"{planner} exceeded the wall-clock budget before returning a comparable result",
            }

        payload = json.loads(out_path.read_text())

    trial = payload["trials"][0]
    success = bool(trial.get("success"))
    build_time_ms = trial.get("build_time_ms")
    query_time_ms = trial.get("query_time_ms")
    if planner == "prm":
        time_s = (float(query_time_ms) / 1000.0) if success and query_time_ms is not None else None
        build_time_s = (float(trial["total_time_ms"]) / 1000.0) if trial.get("total_time_ms") is not None else None
    else:
        time_s = (float(trial["total_time_ms"]) / 1000.0) if success and trial.get("total_time_ms") is not None else None
        build_time_s = (float(build_time_ms) / 1000.0) if build_time_ms is not None else None
    return {
        "query": workload_item["label"],
        "file": workload_item["file"],
        "seed": seed,
        "success": success,
        "time_s": time_s,
        "build_time_s": build_time_s,
        "path_length": float(trial["path_length"]) if success and trial.get("path_length") is not None else None,
        "planner": planner,
        "status": trial.get("status"),
        "budget_s": timeout_s,
        "note": None if success else f"{planner} failed to solve within the timeout",
    }


def run_seed_task(task: tuple[int, dict[str, Any], list[dict[str, Any]]]) -> dict[str, Any]:
    seed, args_dict, workload = task
    baseline_bin = Path(args_dict["baseline_bin"])
    method = str(args_dict["method"])
    timeout_s = float(args_dict["timeout_s"])

    queries = []
    for workload_item in workload:
        query_timeout_s = timeout_s
        if method == "bitstar_budget":
            query_timeout_s = float(args_dict["bitstar_budget_s"])
        queries.append(
            run_trial(
                baseline_bin=baseline_bin,
                workload_item=workload_item,
                method=method,
                seed=seed,
                timeout_s=query_timeout_s,
            )
        )

    build_samples = [float(query["build_time_s"]) for query in queries if query.get("build_time_s") is not None]

    return {
        "seed": seed,
        "build_s": float(mean(build_samples)) if build_samples else (0.0 if method != "prm" else None),
        "queries": queries,
    }


def run_method(
    *,
    method: str,
    args: argparse.Namespace,
    quick: bool,
    seeds: int,
    timeout_s: int,
    workload: list[dict[str, Any]],
) -> dict[str, Any]:
    task_args = {
        "baseline_bin": str((args.baseline_bin or detect_baseline_bin()).resolve()),
        "method": method,
        "timeout_s": timeout_s,
        "bitstar_budget_s": args.bitstar_budget_s,
    }
    tasks = [(seed, task_args, workload) for seed in range(seeds)]
    seed_trials = ordered_parallel_map(
        run_seed_task,
        tasks,
        max_workers=max(1, min(int(args.logical_threads), seeds)),
    )

    params: dict[str, Any] = {
        "baseline_bin": task_args["baseline_bin"],
        "planner": "prm" if method == "prm" else "bit_star",
        "no_simplify": True,
        "logical_threads": int(args.logical_threads),
        "cpu_affinity": current_cpu_affinity(),
        "timeout_s": timeout_s,
    }
    if method == "bitstar_budget":
        params["budget_source"] = "fixed_wall_clock"
        params["budget_metric"] = f"{float(args.bitstar_budget_s):g}s_path_quality"
        params["bitstar_budget_s"] = float(args.bitstar_budget_s)
    if method == "prm":
        params["build_metric"] = "mean_per_seed_roadmap_build_time_across_query_runs"
        params["query_metric"] = "query_only_solve_time_after_roadmap_build"

    return aggregate_method_trials(
        method=method_payload_name(method),
        scene="iiwa14_marcucci_combined",
        quick=quick,
        seeds=seeds,
        params=params,
        seed_trials=seed_trials,
    )


def main() -> int:
    args = parse_args()
    quick, seeds, timeout_s = resolve_mode(args)
    methods = normalize_methods(args.methods)
    workload = marcucci_workload()

    if args.dry_run:
        preview = {
            "methods": methods,
            "baseline_bin": str((args.baseline_bin or detect_baseline_bin()).resolve()),
            "out_dir": str(args.out_dir),
            "quick": quick,
            "seeds": seeds,
            "timeout_s": timeout_s,
            "bitstar_budget_s": args.bitstar_budget_s,
            "logical_threads": int(args.logical_threads),
            "cpu_affinity": current_cpu_affinity(),
        }
        print(json.dumps(preview, indent=2))
        return 0

    for method in methods:
        payload = run_method(
            method=method,
            args=args,
            quick=quick,
            seeds=seeds,
            timeout_s=timeout_s,
            workload=workload,
        )
        out_path = args.out_dir / method_output_name(method)
        write_json(out_path, payload)
        print(f"[marcucci_ompl_baselines] wrote {out_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())