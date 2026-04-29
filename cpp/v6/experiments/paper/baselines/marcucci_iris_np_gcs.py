#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PAPER_DIR = HERE.parent
for candidate in (HERE, PAPER_DIR):
    text = str(candidate)
    if text not in sys.path:
        sys.path.insert(0, text)

from _drake_gcs_regions import build_robot_diagram_checker
from _iris_region_baselines import DEFAULT_IRIS_REGION_BASELINE, add_shared_iris_args, run_region_baseline
from common import (
    PAPER_STATISTICS_POLICY,
    add_mode_args,
    aggregate_method_trials,
    apply_cpu_affinity,
    current_cpu_affinity,
    marcucci_workload,
    resolve_mode,
    resolve_output,
    write_json,
)


DEFAULT_IRIS_NP = {
    "iteration_limit": 10,
    "termination_threshold": -1.0,
    "relative_termination_threshold": 2e-2,
    "num_collision_infeasible_samples": 1,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    add_mode_args(parser)
    add_shared_iris_args(parser)
    parser.add_argument("--cpu-affinity", default=None)
    parser.add_argument("--iteration-limit", type=int, default=DEFAULT_IRIS_NP["iteration_limit"])
    parser.add_argument(
        "--relative-termination-threshold",
        type=float,
        default=DEFAULT_IRIS_NP["relative_termination_threshold"],
    )
    return parser.parse_args()


def make_build_regions(args: argparse.Namespace):
    def build_regions(seed: int, seed_configs, budget_s: float):
        import time
        from pydrake.all import IrisNp, IrisOptions

        robot_diagram, plant, checker = build_robot_diagram_checker(
            edge_step_size=DEFAULT_IRIS_REGION_BASELINE["edge_step_size"],
            env_padding=DEFAULT_IRIS_REGION_BASELINE["env_padding"],
            self_padding=DEFAULT_IRIS_REGION_BASELINE["self_padding"],
        )
        context = robot_diagram.CreateDefaultContext()
        plant_context = plant.GetMyContextFromRoot(context)

        opts = IrisOptions()
        opts.require_sample_point_is_contained = True
        opts.iteration_limit = int(args.iteration_limit)
        opts.termination_threshold = DEFAULT_IRIS_NP["termination_threshold"]
        opts.relative_termination_threshold = float(args.relative_termination_threshold)
        opts.num_collision_infeasible_samples = DEFAULT_IRIS_NP["num_collision_infeasible_samples"]
        opts.random_seed = int(seed)

        regions = []
        timings: list[float] = []
        failures: list[dict] = []
        cumulative = 0.0
        for name, q in seed_configs:
            if cumulative >= budget_s:
                break
            plant.SetPositions(plant_context, q)
            t0 = time.perf_counter()
            try:
                region = IrisNp(plant, plant_context, opts)
                dt = time.perf_counter() - t0
                regions.append(region)
                timings.append(float(dt))
                cumulative += dt
            except Exception as exc:
                dt = time.perf_counter() - t0
                timings.append(float(dt))
                cumulative += dt
                failures.append({"seed_name": name, "note": str(exc)})
        return regions, timings, failures, checker

    return build_regions


def run_seed_trial(task: tuple[int, dict, list[dict]]) -> dict:
    seed, args_dict, workload = task
    args = argparse.Namespace(**args_dict)
    build_regions = make_build_regions(args)
    return run_region_baseline(
        seed=seed,
        workload=workload,
        budget_s=args.budget_s,
        build_regions=build_regions,
        failure_note="IRIS-NP+GCS query failed",
    )


def main() -> int:
    args = parse_args()
    apply_cpu_affinity(args.cpu_affinity)
    quick, seeds, _ = resolve_mode(args)
    workload = marcucci_workload()
    out_path = resolve_output(args, "marcucci_iris_np_gcs.json")
    params = {
        "budget_s": args.budget_s,
        "logical_threads": args.logical_threads,
        "cpu_affinity": current_cpu_affinity(),
        "seed_execution": "serial",
        "edge_step_size": DEFAULT_IRIS_REGION_BASELINE["edge_step_size"],
        "env_padding": DEFAULT_IRIS_REGION_BASELINE["env_padding"],
        "self_padding": DEFAULT_IRIS_REGION_BASELINE["self_padding"],
        "iteration_limit": args.iteration_limit,
        "termination_threshold": DEFAULT_IRIS_NP["termination_threshold"],
        "relative_termination_threshold": args.relative_termination_threshold,
        "num_collision_infeasible_samples": DEFAULT_IRIS_NP["num_collision_infeasible_samples"],
        "require_sample_point_is_contained": True,
        "collision_validation": PAPER_STATISTICS_POLICY["exp4"]["iris_collision_validation"],
        "path_repair": PAPER_STATISTICS_POLICY["exp4"]["iris_path_repair"],
    }

    if args.dry_run:
        print({"out": str(out_path), "params": params})
        return 0

    task_args = {
        "budget_s": args.budget_s,
        "iteration_limit": args.iteration_limit,
        "relative_termination_threshold": args.relative_termination_threshold,
        "logical_threads": args.logical_threads,
    }
    seed_trials = [run_seed_trial((seed, task_args, workload)) for seed in range(seeds)]
    payload = aggregate_method_trials(
        method="iris_np_gcs",
        scene="iiwa14_marcucci_combined",
        quick=quick,
        seeds=seeds,
        params=params,
        seed_trials=seed_trials,
    )
    write_json(out_path, payload)
    print(f"[marcucci_iris_np_gcs] wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())