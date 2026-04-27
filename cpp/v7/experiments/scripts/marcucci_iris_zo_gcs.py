#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if str(HERE) not in sys.path:
    sys.path.insert(0, str(HERE))

from _drake_gcs_regions import build_robot_diagram_checker, joint_limit_domain
from _iris_region_baselines import (
    DEFAULT_IRIS_REGION_BASELINE,
    add_shared_iris_args,
    run_region_baseline,
)
from common import (
    add_mode_args,
    aggregate_method_trials,
    marcucci_workload,
    resolve_mode,
    resolve_output,
    write_json,
)


DEFAULT_IRIS_ZO = {
    "bisection_steps": 10,
    "num_particles": 2000,
    "tau": 0.5,
    "delta": 0.04,
    "epsilon": 1e-2,
    "max_iterations": 5,
    "max_iterations_separating_planes": 20,
    "mixing_steps": 10,
    "configuration_space_margin": 1e-2,
    "relative_termination_threshold": 2e-2,
    "parallelism": 2,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    add_mode_args(parser)
    add_shared_iris_args(parser)
    parser.add_argument("--num-particles", type=int, default=DEFAULT_IRIS_ZO["num_particles"])
    parser.add_argument("--epsilon", type=float, default=DEFAULT_IRIS_ZO["epsilon"])
    parser.add_argument("--max-iterations", type=int, default=DEFAULT_IRIS_ZO["max_iterations"])
    parser.add_argument("--delta", type=float, default=DEFAULT_IRIS_ZO["delta"])
    return parser.parse_args()


def make_iriszo_options(args: argparse.Namespace, *, seed: int):
    from pydrake.common import Parallelism
    from pydrake.planning import IrisZoOptions

    opts = IrisZoOptions()
    opts.bisection_steps = DEFAULT_IRIS_ZO["bisection_steps"]
    sampled = opts.sampled_iris_options
    sampled.num_particles = int(args.num_particles)
    sampled.tau = DEFAULT_IRIS_ZO["tau"]
    sampled.delta = float(args.delta)
    sampled.epsilon = float(args.epsilon)
    sampled.max_iterations = int(args.max_iterations)
    sampled.max_iterations_separating_planes = DEFAULT_IRIS_ZO["max_iterations_separating_planes"]
    sampled.mixing_steps = DEFAULT_IRIS_ZO["mixing_steps"]
    sampled.configuration_space_margin = DEFAULT_IRIS_ZO["configuration_space_margin"]
    sampled.relative_termination_threshold = DEFAULT_IRIS_ZO["relative_termination_threshold"]
    sampled.require_sample_point_is_contained = True
    sampled.random_seed = int(seed)
    try:
        sampled.parallelism = Parallelism(max(1, int(args.logical_threads)))
    except Exception:
        pass
    return opts


def make_build_regions(args: argparse.Namespace):
    def build_regions(seed: int, seed_configs, budget_s: float):
        import time
        from pydrake.all import Hyperellipsoid, IrisZo

        _, _, checker = build_robot_diagram_checker(
            edge_step_size=DEFAULT_IRIS_REGION_BASELINE["edge_step_size"],
            env_padding=DEFAULT_IRIS_REGION_BASELINE["env_padding"],
            self_padding=DEFAULT_IRIS_REGION_BASELINE["self_padding"],
        )
        domain = joint_limit_domain("iiwa14")
        options = make_iriszo_options(args, seed=seed)

        regions = []
        timings: list[float] = []
        failures: list[dict] = []
        cumulative = 0.0
        for name, q in seed_configs:
            if cumulative >= budget_s:
                break
            ellipsoid = Hyperellipsoid.MakeHypersphere(1e-3, q)
            t0 = time.perf_counter()
            try:
                region = IrisZo(checker, ellipsoid, domain, options)
                dt = time.perf_counter() - t0
                regions.append(region)
                timings.append(float(dt))
                cumulative += dt
            except Exception as exc:
                dt = time.perf_counter() - t0
                timings.append(float(dt))
                cumulative += dt
                failures.append({"seed_name": name, "note": str(exc)})
        return regions, timings, failures

    return build_regions


def main() -> int:
    args = parse_args()
    quick, seeds, _ = resolve_mode(args)
    workload = marcucci_workload()
    out_path = resolve_output(args, "marcucci_iris_zo_gcs.json")
    params = {
        "budget_s": args.budget_s,
        "logical_threads": args.logical_threads,
        "edge_step_size": DEFAULT_IRIS_REGION_BASELINE["edge_step_size"],
        "env_padding": DEFAULT_IRIS_REGION_BASELINE["env_padding"],
        "self_padding": DEFAULT_IRIS_REGION_BASELINE["self_padding"],
        "bisection_steps": DEFAULT_IRIS_ZO["bisection_steps"],
        "num_particles": args.num_particles,
        "tau": DEFAULT_IRIS_ZO["tau"],
        "delta": args.delta,
        "epsilon": args.epsilon,
        "max_iterations": args.max_iterations,
        "max_iterations_separating_planes": DEFAULT_IRIS_ZO["max_iterations_separating_planes"],
        "mixing_steps": DEFAULT_IRIS_ZO["mixing_steps"],
        "configuration_space_margin": DEFAULT_IRIS_ZO["configuration_space_margin"],
        "relative_termination_threshold": DEFAULT_IRIS_ZO["relative_termination_threshold"],
        "parallelism": max(1, int(args.logical_threads)),
    }

    if args.dry_run:
        print({"out": str(out_path), "params": params})
        return 0

    build_regions = make_build_regions(args)
    seed_trials = [
        run_region_baseline(
            seed=seed,
            workload=workload,
            budget_s=args.budget_s,
            build_regions=build_regions,
            failure_note="IRIS-ZO+GCS query failed",
        )
        for seed in range(seeds)
    ]
    payload = aggregate_method_trials(
        method="iris_zo_gcs",
        scene="iiwa14_marcucci_combined",
        quick=quick,
        seeds=seeds,
        params=params,
        seed_trials=seed_trials,
    )
    write_json(out_path, payload)
    print(f"[marcucci_iris_zo_gcs] wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())