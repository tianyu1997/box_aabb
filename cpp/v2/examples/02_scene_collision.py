#!/usr/bin/env python3
"""
Example 2: Scene Management & Collision Checking
=================================================

Demonstrates:
  - Creating and managing obstacle scenes
  - Checking individual configurations
  - Checking line segments
  - Checking boxes (interval sets)
  - Using the Scene class for dynamic obstacle management
  - Saving/loading obstacle scenes as JSON

Usage:
    python 02_scene_collision.py
"""
import numpy as np

try:
    import pysbf2
except ImportError:
    raise ImportError("pysbf2 not found. Build with -DSBF_WITH_PYTHON=ON.")


def main():
    # ─── Load Robot ───────────────────────────────────────────────────────
    robot = pysbf2.Robot.from_json("../../configs/panda.json")

    # ─── 1. Obstacle Creation ────────────────────────────────────────────
    print("=== Obstacle Creation ===")
    obs1 = pysbf2.Obstacle(
        np.array([0.5, 0.0, 0.5]),
        np.array([0.1, 0.3, 0.05]),
        "table_top"
    )
    print(f"Obstacle: {obs1}")
    print(f"  Lower corner: {obs1.lo()}")
    print(f"  Upper corner: {obs1.hi()}")

    # ─── 2. Collision Checker ────────────────────────────────────────────
    print("\n=== Collision Checking ===")
    obstacles = [
        obs1,
        pysbf2.Obstacle(np.array([0.0, 0.4, 0.3]),
                         np.array([0.2, 0.05, 0.3]),
                         "wall"),
        pysbf2.Obstacle(np.array([-0.3, 0.0, 0.1]),
                         np.array([0.15, 0.15, 0.1]),
                         "base_box"),
    ]

    checker = pysbf2.AabbCollisionChecker(robot, obstacles)
    print(f"Checking with {checker.n_obs()} obstacles, "
          f"{checker.n_aabb_slots()} AABB slots")

    # Check specific configurations
    q_home = np.zeros(robot.n_joints())
    q_test = np.array([0.5, -0.3, 0.0, -1.5, 0.0, 1.2, 0.0])
    q_stretch = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    for name, q in [("home", q_home), ("test", q_test), ("stretch", q_stretch)]:
        free = checker.check_config(q)
        print(f"  Config '{name}': {'FREE' if free else 'COLLISION'}")

    # Check a line segment
    print(f"\nSegment check (home → test):")
    seg_free = checker.check_segment(q_home, q_test, step=0.02)
    print(f"  {'FREE' if seg_free else 'COLLISION'}")

    # Check a box (interval FK)
    print(f"\nBox check (small box around home):")
    intervals = [pysbf2.Interval(q_home[i] - 0.05, q_home[i] + 0.05)
                 for i in range(robot.n_joints())]
    box_free = checker.check_box(intervals)
    print(f"  {'FREE' if box_free else 'COLLISION'}")

    print(f"\nTotal collision checks performed: {checker.n_checks()}")
    checker.reset_counter()
    print(f"After reset: {checker.n_checks()}")

    # ─── 3. Scene Management ────────────────────────────────────────────
    print("\n=== Scene Management ===")
    scene = pysbf2.Scene()
    print(f"Empty scene: {scene.n_obstacles()} obstacles")

    # Add obstacles one by one
    scene.add_obstacle(pysbf2.Obstacle(
        np.array([0.3, 0.0, 0.5]),
        np.array([0.05, 0.05, 0.05]),
        "cube_A"
    ))
    scene.add_obstacle(pysbf2.Obstacle(
        np.array([0.6, 0.0, 0.3]),
        np.array([0.1, 0.1, 0.1]),
        "cube_B"
    ))
    scene.add_obstacle(pysbf2.Obstacle(
        np.array([0.0, 0.5, 0.4]),
        np.array([0.08, 0.08, 0.2]),
        "pillar_C"
    ))
    print(f"After adding 3: {scene.n_obstacles()} obstacles")

    # Remove by name
    scene.remove_obstacle("cube_A")
    print(f"After removing cube_A: {scene.n_obstacles()} obstacles")

    # Access remaining
    for obs in scene.obstacles():
        print(f"  {obs.name}: center={obs.center}, half={obs.half_sizes}")

    # Clear all
    scene.clear()
    print(f"After clear: {scene.n_obstacles()} obstacles")

    # ─── 4. Batch from list ──────────────────────────────────────────────
    print("\n=== Batch Scene ===")
    batch = [
        pysbf2.Obstacle(np.array([0.4, 0.0, h * 0.2]),
                         np.array([0.05, 0.05, 0.05]),
                         f"layer_{h}")
        for h in range(5)
    ]
    scene2 = pysbf2.Scene(batch)
    print(f"Batch scene: {scene2.n_obstacles()} obstacles")

    # ─── 5. JSON I/O ────────────────────────────────────────────────────
    print("\n=== JSON I/O ===")
    pysbf2.io.save_obstacles_json("test_scene.json", batch)
    print("Saved to test_scene.json")

    loaded = pysbf2.io.load_obstacles_json("test_scene.json")
    print(f"Loaded {len(loaded)} obstacles from JSON")
    for obs in loaded:
        print(f"  {obs.name}: center={obs.center}")


if __name__ == "__main__":
    main()
