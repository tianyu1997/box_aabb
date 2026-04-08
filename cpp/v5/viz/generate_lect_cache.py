"""
Generate deep LECT caches for robots via obstacle-free tree warmup.

Expands random C-space paths to the target depth.  LECT data (envelopes,
split dims, Z4 cache) is obstacle-independent — no obstacles are needed.

Usage:
    cd v5
    set PYTHONPATH=python;build_x64\\Release
    python viz/generate_lect_cache.py --robot data/iiwa14.json
    python viz/generate_lect_cache.py --robot data/panda.json
"""
import argparse
import os
import time


def parse_args():
    p = argparse.ArgumentParser(description="Generate deep LECT cache")
    p.add_argument("--robot", type=str, required=True,
                   help="Path to robot JSON (e.g. data/iiwa14.json)")
    p.add_argument("--max_depth", type=int, default=30,
                   help="LECT max split depth")
    p.add_argument("--n_paths", type=int, default=5000,
                   help="Random C-space paths to expand")
    p.add_argument("--seed", type=int, default=42,
                   help="RNG seed for path sampling")
    return p.parse_args()


def main():
    args = parse_args()

    from sbf5 import (
        Robot, SBFPlanner, SBFPlannerConfig,
        EndpointSource, EnvelopeType, SplitOrder,
    )

    robot = Robot.from_json(args.robot)
    print(f"Robot: {robot.name()} ({robot.n_joints()} DOF, "
          f"{robot.n_active_links()} active links)")
    print(f"Fingerprint: {robot.fingerprint():016x}")

    config = SBFPlannerConfig()
    config.endpoint_source.source = EndpointSource.CritSample
    config.endpoint_source.n_samples_crit = 50
    config.envelope_type.type = EnvelopeType.LinkIAABB
    config.split_order = SplitOrder.BEST_TIGHTEN
    config.z4_enabled = True

    cache_dir = config.lect_cache_dir
    print(f"Cache dir: {cache_dir}")
    print(f"Max depth: {args.max_depth}")
    print(f"Paths: {args.n_paths}")
    print(f"Seed: {args.seed}")
    print()

    planner = SBFPlanner(robot, config)
    t0 = time.perf_counter()
    new_nodes = planner.warmup_lect(args.max_depth, args.n_paths, args.seed)
    t1 = time.perf_counter()

    fp_hex = f"{robot.fingerprint():016x}"
    cache_path = os.path.join(cache_dir, f"{robot.name()}_{fp_hex}.lect")
    sz = os.path.getsize(cache_path) if os.path.exists(cache_path) else 0

    print(f"New nodes: {new_nodes}")
    print(f"Time: {1000*(t1-t0):.0f}ms")
    print(f"Cache: {cache_path} ({sz/1024:.0f}KB)")


if __name__ == "__main__":
    main()
