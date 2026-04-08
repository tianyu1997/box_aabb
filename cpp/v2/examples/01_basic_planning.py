#!/usr/bin/env python3
"""
Example 1: Basic Motion Planning
=================================

Demonstrates the most common workflow:
  1. Load a robot model
  2. Define obstacles
  3. Plan a collision-free path
  4. Inspect the result

Usage:
    python 01_basic_planning.py
"""
import numpy as np

try:
    import pysbf2
except ImportError:
    raise ImportError(
        "pysbf2 not found. Build with -DSBF_WITH_PYTHON=ON and "
        "add the build directory to PYTHONPATH."
    )


def main():
    # ─── 1. Load Robot ────────────────────────────────────────────────────
    robot = pysbf2.Robot.from_json("../../configs/panda.json")
    print(f"Robot: {robot.name()}, {robot.n_joints()} joints")
    print(f"Joint limits:")
    for i, iv in enumerate(robot.joint_limits().limits):
        print(f"  Joint {i}: [{iv.lo:.4f}, {iv.hi:.4f}]")

    # ─── 2. Define Obstacles ─────────────────────────────────────────────
    obstacles = [
        pysbf2.Obstacle(
            np.array([0.5, 0.0, 0.4]),
            np.array([0.05, 0.3, 0.2]),
            "shelf"
        ),
        pysbf2.Obstacle(
            np.array([0.0, 0.5, 0.3]),
            np.array([0.15, 0.05, 0.15]),
            "pillar"
        ),
        pysbf2.Obstacle(
            np.array([0.3, -0.3, 0.7]),
            np.array([0.1, 0.1, 0.05]),
            "ceiling_block"
        ),
    ]
    print(f"\n{len(obstacles)} obstacles defined")

    # ─── 3. Define Start and Goal ────────────────────────────────────────
    start = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    goal  = np.array([2.0,  0.5, -1.0, -1.5,   0.5, 1.0,  -0.5])

    # Verify both are collision-free
    checker = pysbf2.AabbCollisionChecker(robot, obstacles)
    assert checker.check_config(start), "Start is in collision!"
    assert checker.check_config(goal),  "Goal is in collision!"
    print(f"Start and goal verified collision-free")

    # ─── 4. Configure ────────────────────────────────────────────────────
    config = pysbf2.make_panda_config(seed=42)
    config.max_boxes = 500
    config.shortcut_max_iters = 200
    print(f"\nConfig: max_boxes={config.max_boxes}, "
          f"seed={config.seed}")

    # ─── 5. Plan ─────────────────────────────────────────────────────────
    print("\nPlanning...")
    result = pysbf2.plan_once(robot, obstacles, start, goal, config)

    # ─── 6. Inspect Result ───────────────────────────────────────────────
    print(f"\n{'='*50}")
    print(f"  Success:          {result.success}")
    print(f"  Path waypoints:   {result.n_waypoints()}")
    print(f"  Path cost (L2):   {result.cost:.4f}")
    print(f"  Planning time:    {result.planning_time:.3f} s")
    print(f"  First sol. time:  {result.first_solution_time:.3f} s")
    print(f"  Collision checks: {result.collision_checks}")
    print(f"  Nodes explored:   {result.nodes_explored}")
    print(f"{'='*50}")

    if result.success:
        path = result.path
        print(f"\nPath shape: {path.shape}")
        print(f"Start: {path[0, :]}")
        print(f"Goal:  {path[-1, :]}")

        # Verify path is collision-free
        checker2 = pysbf2.AabbCollisionChecker(robot, obstacles)
        all_free = all(
            checker2.check_config(path[i, :])
            for i in range(path.shape[0])
        )
        print(f"All waypoints collision-free: {all_free}")

    # ─── 7. Phase Timing ────────────────────────────────────────────────
    if result.phase_times:
        print(f"\nPhase timing breakdown:")
        for phase, t in sorted(result.phase_times.items()):
            print(f"  {phase:20s}: {t:.4f} s")


if __name__ == "__main__":
    main()
