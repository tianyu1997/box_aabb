#!/usr/bin/env python3
"""
Example 6: Multi-Pair Planning & Caching
==========================================

Demonstrates:
  - Building a forest covering multiple start-goal pairs
  - Querying different pairs from the same forest
  - Saving and loading the HierAABBTree cache
  - Performance comparison: cold vs cached planning

Usage:
    python 06_multi_pair_cache.py
"""
import numpy as np
import time
import os

try:
    import pysbf2
except ImportError:
    raise ImportError("pysbf2 not found. Build with -DSBF_WITH_PYTHON=ON.")


def random_config(robot, rng):
    """Sample a random configuration within joint limits."""
    limits = robot.joint_limits()
    q = np.zeros(robot.n_joints())
    for i, iv in enumerate(limits.limits):
        q[i] = rng.uniform(iv.lo, iv.hi)
    return q


def main():
    # ─── Setup ────────────────────────────────────────────────────────────
    robot = pysbf2.Robot.from_json("../../configs/panda.json")
    obstacles = [
        pysbf2.Obstacle(np.array([0.5, 0.0, 0.4]),
                         np.array([0.05, 0.3, 0.2]), "shelf"),
        pysbf2.Obstacle(np.array([0.0, 0.4, 0.3]),
                         np.array([0.15, 0.05, 0.15]), "wall"),
    ]

    checker = pysbf2.AabbCollisionChecker(robot, obstacles)
    rng = np.random.default_rng(42)

    # Generate collision-free start-goal pairs
    print("=== Generating Start-Goal Pairs ===")
    pairs = []
    while len(pairs) < 5:
        s = random_config(robot, rng)
        g = random_config(robot, rng)
        if checker.check_config(s) and checker.check_config(g):
            pairs.append((s, g))
    print(f"Generated {len(pairs)} collision-free pairs")

    # ─── 1. Multi-Pair Build ─────────────────────────────────────────────
    print("\n=== Multi-Pair Forest Build ===")
    config = pysbf2.make_panda_config(seed=42)
    planner = pysbf2.SBFPlanner(robot, obstacles, config)

    t0 = time.time()
    planner.build_multi(pairs, n_random_boxes=3000, timeout=120.0)
    build_time = time.time() - t0

    print(f"Built in {build_time:.2f}s")
    print(f"Forest: {planner.forest().n_boxes()} boxes")
    print(f"Tree nodes: {planner.tree().n_nodes()}")
    print(f"Total FK calls: {planner.tree().total_fk_calls()}")

    # ─── 2. Query All Pairs ──────────────────────────────────────────────
    print("\n=== Querying All Pairs ===")
    for i, (s, g) in enumerate(pairs):
        t0 = time.time()
        result = planner.query(s, g, timeout=5.0)
        dt = time.time() - t0
        status = "OK" if result.success else "FAIL"
        cost = f"{result.cost:.3f}" if result.success else "N/A"
        print(f"  Pair {i}: {status}, cost={cost}, "
              f"time={dt:.3f}s")

    # ─── 3. Save Cache ───────────────────────────────────────────────────
    print("\n=== Cache Persistence ===")
    cache_path = "panda_tree.hcache"

    t0 = time.time()
    planner.tree().save(cache_path)
    save_time = time.time() - t0

    file_size = os.path.getsize(cache_path)
    print(f"Saved to {cache_path}")
    print(f"  Size: {file_size / 1024:.1f} KB")
    print(f"  Time: {save_time:.3f}s")

    # ─── 4. Load Cache ───────────────────────────────────────────────────
    print("\n=== Load Cache ===")

    # Full load
    t0 = time.time()
    tree_loaded = pysbf2.HierAABBTree.load(cache_path, robot)
    load_time = time.time() - t0
    print(f"Full load: {tree_loaded.n_nodes()} nodes in {load_time:.3f}s")

    # mmap load (lazy)
    t0 = time.time()
    tree_mmap = pysbf2.HierAABBTree.load_mmap(cache_path, robot)
    mmap_time = time.time() - t0
    print(f"mmap load: {tree_mmap.n_nodes()} nodes in {mmap_time:.3f}s")
    tree_mmap.close_mmap()

    # ─── 5. Cached vs Cold Comparison ────────────────────────────────────
    print("\n=== Cold vs Cached Planning ===")
    test_pair = pairs[0]

    # Cold start (new planner, no cache)
    t0 = time.time()
    cold_result = pysbf2.plan_once(
        robot, obstacles, test_pair[0], test_pair[1], config)
    cold_time = time.time() - t0

    # Warm start (pre-built forest)
    t0 = time.time()
    warm_result = planner.query(test_pair[0], test_pair[1], timeout=5.0)
    warm_time = time.time() - t0

    print(f"  Cold planning:  {cold_time:.3f}s "
          f"({'OK' if cold_result.success else 'FAIL'})")
    print(f"  Warm query:     {warm_time:.3f}s "
          f"({'OK' if warm_result.success else 'FAIL'})")
    if cold_time > 0:
        print(f"  Speedup:        {cold_time / max(warm_time, 1e-6):.1f}x")

    # Clean up
    if os.path.exists(cache_path):
        os.remove(cache_path)
        print(f"\nCleaned up {cache_path}")


if __name__ == "__main__":
    main()
