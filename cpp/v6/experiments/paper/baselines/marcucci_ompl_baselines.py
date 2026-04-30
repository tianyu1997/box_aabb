#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import tempfile
from pathlib import Path
from statistics import mean
from typing import Any

HERE = Path(__file__).resolve().parent
PAPER_DIR = HERE.parent
for candidate in (HERE, PAPER_DIR):
    text = str(candidate)
    if text not in sys.path:
        sys.path.insert(0, text)

from common import (
    OUT_DEFAULT,
    PAPER_STATISTICS_POLICY,
    ROOT,
    add_logical_threads_arg,
    add_mode_args,
    aggregate_method_trials,
    apply_cpu_affinity,
    current_cpu_affinity,
    marcucci_workload,
    resolve_experiment_binary,
    resolve_mode,
    write_json,
)


def detect_baseline_bin(args: argparse.Namespace) -> Path:
    return resolve_experiment_binary(
        "baseline_ompl",
        requested_build_dir=args.build_dir,
        allow_debug=bool(args.allow_debug_build),
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    add_mode_args(parser)
    add_logical_threads_arg(parser)
    parser.add_argument("--cpu-affinity", default=None)
    parser.add_argument("--out-dir", type=Path, default=OUT_DEFAULT)
    parser.add_argument("--build-dir", type=Path, default=None)
    parser.add_argument("--allow-debug-build", action="store_true")
    parser.add_argument("--baseline-bin", type=Path, default=None)
    parser.add_argument("--bitstar-budget-s", type=float, default=10.0)
    parser.add_argument("--prm-build-budget-s", type=float, default=10.0)
    parser.add_argument("--prm-query-budget-s", type=float, default=2.0)
    parser.add_argument("--methods", default="prm,bitstar_budget")
    parser.add_argument(
        "--simplify-prm",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="include OMPL simplifySolution in the PRM query timing record",
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


def run_trial(
    *,
    baseline_bin: Path,
    workload_item: dict[str, Any],
    method: str,
    seed: int,
    timeout_s: float,
    simplify: bool,
    prm_build_budget_s: float,
    prm_query_budget_s: float,
) -> dict[str, Any]:
    planner = "prm" if method == "prm" else "bit_star"
    cmd = [
        str(baseline_bin),
        f"--scene={workload_item['path']}",
        "--seeds=1",
        f"--seed-base={42 + seed}",
        f"--timeout={timeout_s}",
        f"--planner={planner}",
    ]
    if planner == "prm":
        cmd.extend(
            [
                f"--prm-build={float(prm_build_budget_s)}",
                f"--prm-query={float(prm_query_budget_s)}",
            ]
        )
    if not simplify:
        cmd.append("--no-simplify")

    with tempfile.TemporaryDirectory(prefix="marcucci_ompl_v6_") as tmpdir:
        out_path = Path(tmpdir) / "result.json"
        cmd.append(f"--out={out_path}")
        grace_s = min(0.1, max(0.01, 0.1 * timeout_s))
        if planner == "prm" and simplify:
            grace_s += 2.0
        try:
            subprocess.run(
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
        if build_time_ms is not None:
            build_time_s = float(build_time_ms) / 1000.0
        else:
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
                simplify=bool(args_dict["simplify"]),
                prm_build_budget_s=float(args_dict["prm_build_budget_s"]),
                prm_query_budget_s=float(args_dict["prm_query_budget_s"]),
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
    baseline_bin = (args.baseline_bin or detect_baseline_bin(args)).resolve()
    task_args = {
        "baseline_bin": str(baseline_bin),
        "method": method,
        "timeout_s": timeout_s,
        "bitstar_budget_s": args.bitstar_budget_s,
        "prm_build_budget_s": args.prm_build_budget_s,
        "prm_query_budget_s": args.prm_query_budget_s,
        "simplify": bool(args.simplify_prm) if method == "prm" else False,
    }
    tasks = [(seed, task_args, workload) for seed in range(seeds)]
    seed_trials = [run_seed_task(task) for task in tasks]

    params: dict[str, Any] = {
        "baseline_bin": task_args["baseline_bin"],
        "planner": "prm" if method == "prm" else "bit_star",
        "no_simplify": not bool(task_args["simplify"]),
        "postprocess": "ompl_simplify_solution" if task_args["simplify"] else None,
        "logical_threads": int(args.logical_threads),
        "cpu_affinity": current_cpu_affinity(),
        "seed_execution": "serial",
        "timeout_s": timeout_s,
    }
    if method == "bitstar_budget":
        params["budget_source"] = "fixed_wall_clock"
        params["budget_metric"] = f"{float(args.bitstar_budget_s):g}s_path_quality"
        params["bitstar_budget_s"] = float(args.bitstar_budget_s)
    if method == "prm":
        params["build_metric"] = "mean_per_seed_roadmap_build_time_across_query_runs"
        params["query_metric"] = PAPER_STATISTICS_POLICY["exp4"]["prm_query_metric"]
        params["prm_build_budget_s"] = float(args.prm_build_budget_s)
        params["prm_query_budget_s"] = float(args.prm_query_budget_s)

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
    apply_cpu_affinity(args.cpu_affinity)
    quick, seeds, timeout_s = resolve_mode(args)
    methods = normalize_methods(args.methods)
    workload = marcucci_workload()

    if args.dry_run:
        preview = {
            "methods": methods,
            "baseline_bin": str((args.baseline_bin or detect_baseline_bin(args)).resolve()),
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