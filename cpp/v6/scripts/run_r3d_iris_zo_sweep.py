"""R3-D1: IRIS-ZO parameter sweep.

Run pydrake.planning.IrisZo with several (num_particles, epsilon, max_iter)
combinations on the Marcucci-16 scene to test whether the default-params
SR=40% can be lifted toward SR>=90%. Same seed list as run_iris_zo.

Output:
  cpp/v6/experiments/results_r3d/D1_iris_zo_sweep.json
"""
import json
import os
import sys
import time
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from run_baselines import (  # noqa: E402
    _get_robot_diagram_checker_cached,
    _get_plant_cached,
    _resolve_query_pairs,
    run_iris_gcs,
    IIWA_CONFIGS,
    IIWA_JOINT_LIMITS,
    QUERY_PAIRS,
    logger,
)
import numpy as np  # noqa: E402


def run_one(num_particles, epsilon, max_iter, seed=0, budget_s=2400.0):
    from pydrake.planning import IrisZo, IrisZoOptions
    from pydrake.geometry.optimization import HPolyhedron, Hyperellipsoid

    rdiag, rplant, sg_checker = _get_robot_diagram_checker_cached()
    diagram, plant, checker = _get_plant_cached()
    pairs = _resolve_query_pairs(None)

    iris_seeds = [(name, IIWA_CONFIGS[name])
                  for name in ("AS", "TS", "CS", "LB", "RB")]
    for label, s, g in QUERY_PAIRS:
        mid = (IIWA_CONFIGS[s] + IIWA_CONFIGS[g]) / 2.0
        if not checker.check_config(mid):
            iris_seeds.append((f"mid_{label}", mid))

    lows = np.array([lo for lo, _ in IIWA_JOINT_LIMITS])
    highs = np.array([hi for _, hi in IIWA_JOINT_LIMITS])
    domain = HPolyhedron.MakeBox(lows, highs)

    opts = IrisZoOptions()
    opts.bisection_steps = 10
    sio = opts.sampled_iris_options
    sio.num_particles = int(num_particles)
    sio.tau = 0.5
    sio.delta = 5e-2
    sio.epsilon = float(epsilon)
    sio.max_iterations = int(max_iter)
    sio.max_iterations_separating_planes = 20
    sio.mixing_steps = 10
    sio.configuration_space_margin = 1e-2
    sio.relative_termination_threshold = 2e-2
    sio.require_sample_point_is_contained = True
    sio.random_seed = int(seed)
    try:
        from pydrake.common import Parallelism
        sio.parallelism = Parallelism(2)
    except Exception:
        pass

    regions, timings = [], []
    cumulative = 0.0
    for name, q in iris_seeds:
        if cumulative >= budget_s:
            break
        ellip = Hyperellipsoid.MakeHypersphere(1e-3, q)
        t0 = time.perf_counter()
        try:
            r = IrisZo(sg_checker, ellip, domain, opts)
            dt = time.perf_counter() - t0
            regions.append(r)
            timings.append(dt)
            cumulative += dt
            logger.info(f"  IRIS-ZO[{num_particles},{epsilon},{max_iter}] "
                        f"seed={name} dt={dt:.1f}s cum={cumulative:.1f}s")
        except Exception as exc:
            dt = time.perf_counter() - t0
            cumulative += dt
            logger.warning(f"  IRIS-ZO seed={name} FAILED ({exc})")

    qtimes, qlens, ok = [], [], 0
    for _, qs, qg in pairs:
        if not regions:
            break
        try:
            r = run_iris_gcs(qs, qg, plant, diagram, regions, seed=seed)
        except Exception as exc:
            logger.warning(f"  GCS query failed: {exc}")
            continue
        if r["success"]:
            ok += 1
            qtimes.append(r["time_s"])
            qlens.append(r["path_length"])

    return {
        "num_particles": num_particles,
        "epsilon": epsilon,
        "max_iter": max_iter,
        "seed": seed,
        "build_s": float(cumulative),
        "n_regions": len(regions),
        "per_region_s": [float(t) for t in timings],
        "query_path_rad_mean": float(np.mean(qlens)) if qlens else None,
        "query_time_s_mean": float(np.mean(qtimes)) if qtimes else None,
        "sr": 100.0 * ok / max(1, len(pairs)),
        "n_queries": len(pairs),
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--out", default="cpp/v6/experiments/results_r3d/D1_iris_zo_sweep.json")
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    sweep = [
        # (num_particles, epsilon, max_iter)
        (1000, 1e-2, 3),     # baseline (already in B4_iris_zo.json) — repro check
        (2000, 1e-2, 5),     # more particles + iterations
        (4000, 5e-3, 5),     # tighter epsilon, more particles
        (2000, 1e-2, 10),    # many iterations
    ]
    cells = []
    for np_, eps, mi in sweep:
        logger.info(f"=== sweep cell np={np_} eps={eps} max_iter={mi} ===")
        try:
            r = run_one(np_, eps, mi, seed=args.seed)
            cells.append(r)
            logger.info(f"  → SR={r['sr']:.0f}% build={r['build_s']:.1f}s "
                        f"regions={r['n_regions']} path={r['query_path_rad_mean']}")
        except Exception as exc:
            logger.error(f"  cell failed: {exc}")
            cells.append({"num_particles": np_, "epsilon": eps,
                          "max_iter": mi, "error": str(exc)})

    out = {"sweep": sweep, "cells": cells}
    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(out, f, indent=2)
    logger.info(f"Wrote {args.out}")


if __name__ == "__main__":
    main()
