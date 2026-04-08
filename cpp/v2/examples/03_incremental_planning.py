#!/usr/bin/env python3
"""
Example 3: Incremental Planning — Dynamic Obstacles
=====================================================

Demonstrates:
  - Building a forest once
  - Adding/removing obstacles at runtime
  - Re-growing the forest incrementally
  - Re-planning after environment changes

Usage:
    python 03_incremental_planning.py
"""
import numpy as np
import time

try:
    import pysbf2
except ImportError:
    raise ImportError("pysbf2 not found. Build with -DSBF_WITH_PYTHON=ON.")


def main():
    # ─── Setup ────────────────────────────────────────────────────────────
    robot = pysbf2.Robot.from_json("../../configs/panda.json")
    start = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    goal  = np.array([2.0,  0.5, -1.0, -1.5,   0.5, 1.0,  -0.5])

    initial_obstacles = [
        pysbf2.Obstacle(np.array([0.5, 0.0, 0.4]),
                         np.array([0.05, 0.3, 0.2]), "shelf"),
    ]

    config = pysbf2.make_panda_config(seed=42)
    config.max_boxes = 500

    # ─── Phase 1: Initial Build & Plan ───────────────────────────────────
    print("=" * 60)
    print("Phase 1: Initial Planning")
    print("=" * 60)

    planner = pysbf2.SBFPlanner(robot, initial_obstacles, config)

    t0 = time.time()
    planner.build(start, goal, timeout=30.0)
    build_time = time.time() - t0

    print(f"Forest built: {planner.forest().n_boxes()} boxes "
          f"in {build_time:.2f}s")

    result1 = planner.query(start, goal, timeout=5.0)
    print(f"Plan result: {'SUCCESS' if result1.success else 'FAIL'}")
    if result1.success:
        print(f"  Waypoints: {result1.n_waypoints()}, "
              f"Cost: {result1.cost:.4f}")

    # ─── Phase 2: Add New Obstacle ───────────────────────────────────────
    print(f"\n{'='*60}")
    print("Phase 2: Obstacle Added")
    print("=" * 60)

    new_obstacle = pysbf2.Obstacle(
        np.array([0.3, 0.2, 0.5]),
        np.array([0.12, 0.12, 0.12]),
        "moving_box"
    )
    print(f"Adding obstacle: {new_obstacle.name} "
          f"at {new_obstacle.center}")

    boxes_before = planner.forest().n_boxes()
    t0 = time.time()
    planner.add_obstacle(new_obstacle)
    invalidate_time = time.time() - t0

    boxes_after = planner.forest().n_boxes()
    invalidated = boxes_before - boxes_after
    print(f"Boxes invalidated: {invalidated} "
          f"({boxes_before} → {boxes_after}) in {invalidate_time:.3f}s")

    # Try planning without regrowth
    result2 = planner.query(start, goal, timeout=5.0)
    print(f"Plan without regrow: {'SUCCESS' if result2.success else 'FAIL'}")

    # ─── Phase 3: Regrow ─────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print("Phase 3: Regrow Forest")
    print("=" * 60)

    t0 = time.time()
    n_regrown = planner.regrow(n_target=300, timeout=15.0)
    regrow_time = time.time() - t0

    print(f"Regrew {n_regrown} boxes in {regrow_time:.2f}s")
    print(f"Forest now has {planner.forest().n_boxes()} boxes")

    result3 = planner.query(start, goal, timeout=5.0)
    print(f"Plan after regrow: {'SUCCESS' if result3.success else 'FAIL'}")
    if result3.success:
        print(f"  Waypoints: {result3.n_waypoints()}, "
              f"Cost: {result3.cost:.4f}")

    # ─── Phase 4: Remove Obstacle ────────────────────────────────────────
    print(f"\n{'='*60}")
    print("Phase 4: Obstacle Removed")
    print("=" * 60)

    planner.remove_obstacle("moving_box")
    print("Removed 'moving_box'")

    # Regrow again (more free space available)
    n_regrown2 = planner.regrow(n_target=500, timeout=15.0)
    print(f"Regrew {n_regrown2} more boxes, "
          f"total: {planner.forest().n_boxes()}")

    result4 = planner.query(start, goal, timeout=5.0)
    print(f"Plan after removal: {'SUCCESS' if result4.success else 'FAIL'}")
    if result4.success:
        print(f"  Waypoints: {result4.n_waypoints()}, "
              f"Cost: {result4.cost:.4f}")

    # ─── Phase 5: Multiple Updates ───────────────────────────────────────
    print(f"\n{'='*60}")
    print("Phase 5: Rapid Updates")
    print("=" * 60)

    # Simulate a moving obstacle
    for step in range(5):
        offset = 0.1 * step
        obs_name = f"dynamic_{step}"
        obs = pysbf2.Obstacle(
            np.array([0.3 + offset, 0.0, 0.4]),
            np.array([0.08, 0.08, 0.08]),
            obs_name
        )
        planner.add_obstacle(obs)
        planner.regrow(100, timeout=3.0)
        result = planner.query(start, goal, timeout=2.0)
        status = "OK" if result.success else "FAIL"
        print(f"  Step {step}: +{obs_name}, "
              f"boxes={planner.forest().n_boxes()}, "
              f"plan={status}")

    # ─── Summary ─────────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print("Summary")
    print("=" * 60)
    print(f"Final forest: {planner.forest().n_boxes()} boxes")
    print(f"Total volume: {planner.forest().total_volume():.4f}")


if __name__ == "__main__":
    main()
