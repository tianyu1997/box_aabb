#!/usr/bin/env python3
"""
Example 5: Low-Level Pipeline — Full Control
==============================================

Demonstrates the individual steps of the planning pipeline:
  1. Robot + collision checker setup
  2. HierAABBTree creation
  3. ForestGrower — box growth
  4. Coarsening — merge boxes
  5. Graph search — Dijkstra
  6. Path smoothing

This gives you full control over every stage, unlike the
high-level SBFPlanner.plan() call.

Usage:
    python 05_low_level_pipeline.py
"""
import numpy as np
import time

try:
    import pysbf2
except ImportError:
    raise ImportError("pysbf2 not found. Build with -DSBF_WITH_PYTHON=ON.")


def main():
    # ─── 1. Robot + Scene ─────────────────────────────────────────────────
    print("=== Step 1: Setup ===")
    robot = pysbf2.Robot.from_json("../../configs/panda.json")
    obstacles = [
        pysbf2.Obstacle(np.array([0.5, 0.0, 0.4]),
                         np.array([0.05, 0.3, 0.2]), "shelf"),
    ]
    checker = pysbf2.AabbCollisionChecker(robot, obstacles)
    print(f"Robot: {robot.name()}, {robot.n_joints()} DOF")
    print(f"Obstacles: {checker.n_obs()}")

    start = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
    goal  = np.array([2.0,  0.5, -1.0, -1.5,   0.5, 1.0,  -0.5])

    # ─── 2. Create Tree & Forest ─────────────────────────────────────────
    print("\n=== Step 2: Create Tree & Forest ===")
    tree = pysbf2.HierAABBTree(robot, initial_cap=128)
    forest = pysbf2.SafeBoxForest(robot.n_joints(), robot.joint_limits())
    print(f"Tree nodes: {tree.n_nodes()}")
    print(f"Forest boxes: {forest.n_boxes()}")

    # ─── 3. Configure & Grow ─────────────────────────────────────────────
    print("\n=== Step 3: Forest Growth ===")
    config = pysbf2.make_panda_config(seed=42)
    config.max_boxes = 300

    grower = pysbf2.ForestGrower(robot, checker, tree, forest, config)

    t0 = time.time()
    success = grower.grow(start, goal)
    grow_time = time.time() - t0

    print(f"Growth {'succeeded' if success else 'failed'}")
    print(f"  Boxes grown: {forest.n_boxes()}")
    print(f"  Tree nodes:  {tree.n_nodes()}")
    print(f"  FK calls:    {tree.total_fk_calls()}")
    print(f"  Time:        {grow_time:.2f}s")

    # ─── 4. Coarsen ──────────────────────────────────────────────────────
    print("\n=== Step 4: Coarsening ===")
    t0 = time.time()
    cr = pysbf2.coarsen_greedy(
        forest, checker,
        target_boxes=0,
        max_rounds=100,
        adjacency_tol=1e-10
    )
    coarsen_time = time.time() - t0

    print(f"  Boxes: {cr.boxes_before} → {cr.boxes_after}")
    print(f"  Merges: {cr.merges_performed} in {cr.rounds} rounds")
    print(f"  Time: {coarsen_time:.2f}s")

    # ─── 5. Rebuild Adjacency ────────────────────────────────────────────
    print("\n=== Step 5: Rebuild Adjacency ===")
    forest.rebuild_adjacency(tol=1e-8)
    adj = forest.adjacency()
    print(f"  Nodes: {len(adj)}")
    total_edges = sum(len(v) for v in adj.values()) // 2
    print(f"  Edges: {total_edges}")

    # ─── 6. Find Islands ─────────────────────────────────────────────────
    print("\n=== Step 6: Connectivity Check ===")
    boxes = forest.boxes()
    all_ids = list(boxes.keys())
    islands = pysbf2.find_islands(adj, all_ids)
    print(f"  Islands: {len(islands)}")
    for isl_id, members in list(islands.items())[:5]:
        print(f"    Island {isl_id}: {len(members)} boxes")

    # ─── 7. Graph Search ─────────────────────────────────────────────────
    print("\n=== Step 7: Graph Search ===")

    # Find boxes containing start and goal
    start_box = forest.find_containing(start)
    goal_box = forest.find_containing(goal)

    if start_box is None:
        start_box = forest.find_nearest(start)
        print(f"  Start not inside any box, using nearest: {start_box.id}")
    else:
        print(f"  Start in box {start_box.id}")

    if goal_box is None:
        goal_box = forest.find_nearest(goal)
        print(f"  Goal not inside any box, using nearest: {goal_box.id}")
    else:
        print(f"  Goal in box {goal_box.id}")

    start_ids = {start_box.id}
    goal_ids = {goal_box.id}

    dr = pysbf2.dijkstra_center_distance(adj, boxes, start_ids, goal_ids)
    print(f"  Path found: {dr.found}")
    if dr.found:
        print(f"  Box sequence length: {len(dr.path)}")
        print(f"  Graph cost: {dr.total_cost:.4f}")

        # Extract waypoints
        waypoints = pysbf2.extract_waypoints(dr.path, boxes, start, goal)
        print(f"  Waypoints: {len(waypoints)}")
        raw_length = pysbf2.PathSmoother.path_length(waypoints)
        print(f"  Raw path length: {raw_length:.4f}")

        # ─── 8. Path Smoothing ───────────────────────────────────────────
        print("\n=== Step 8: Path Smoothing ===")
        smoother = pysbf2.PathSmoother(checker, 0.05)

        # Stage 1: Random shortcutting
        t0 = time.time()
        shortened = smoother.shortcut(waypoints, max_iters=200)
        t1 = time.time()
        print(f"  Shortcut: {len(waypoints)} → {len(shortened)} waypoints "
              f"({t1-t0:.3f}s)")
        print(f"  Length: {raw_length:.4f} → "
              f"{pysbf2.PathSmoother.path_length(shortened):.4f}")

        # Stage 2: Box-aware shortcutting
        t0 = time.time()
        box_short = smoother.box_aware_shortcut(shortened, forest,
                                                  max_iters=100)
        t1 = time.time()
        print(f"  Box-aware: {len(shortened)} → {len(box_short)} waypoints "
              f"({t1-t0:.3f}s)")
        print(f"  Length: {pysbf2.PathSmoother.path_length(box_short):.4f}")

        # Stage 3: Moving average
        t0 = time.time()
        smooth = smoother.smooth_moving_average(box_short, forest,
                                                  window=5, iterations=3)
        t1 = time.time()
        print(f"  Moving avg: {len(box_short)} → {len(smooth)} waypoints "
              f"({t1-t0:.3f}s)")
        print(f"  Length: {pysbf2.PathSmoother.path_length(smooth):.4f}")

        # Stage 4: Resample
        resampled = smoother.resample(smooth, resolution=0.02)
        print(f"  Resampled: {len(resampled)} waypoints "
              f"(resolution=0.02)")
        final_length = pysbf2.PathSmoother.path_length(resampled)
        print(f"  Final length: {final_length:.4f}")

    # ─── 9. Envelope Computer (LEGACY) ───────────────────────────────────
    # NOTE: IntervalFKEnvelopeComputer is a legacy API retained for backward
    # compatibility.  New C++ code should use FrameStore + collision_policy.h.
    print("\n=== Bonus: Envelope Computation (Legacy API) ===")
    env_comp = pysbf2.IntervalFKEnvelopeComputer(robot)
    intervals = [pysbf2.Interval(start[i] - 0.1, start[i] + 0.1)
                 for i in range(robot.n_joints())]

    env = env_comp.compute_envelope(intervals)
    print(f"  Full:  valid={env.valid}, "
          f"links={env.n_link_slots}, ee={env.n_ee_slots}")

    # Incremental after narrowing joint 3
    intervals2 = list(intervals)
    intervals2[3] = pysbf2.Interval(start[3] - 0.05, start[3] + 0.05)
    env2 = env_comp.compute_envelope_incremental(
        env.fk_state, intervals2, changed_dim=3)
    print(f"  Incr:  valid={env2.valid}, "
          f"links={env2.n_link_slots}, ee={env2.n_ee_slots}")

    # ─── 10. Random Sampling ─────────────────────────────────────────────
    print("\n=== Bonus: Sampling ===")
    sampler = pysbf2.RootSampler(robot.joint_limits(), seed=123)
    samples = [sampler.sample_uniform() for _ in range(1000)]
    n_free = sum(1 for q in samples if checker.check_config(q))
    print(f"  1000 random samples: {n_free} collision-free "
          f"({100*n_free/1000:.1f}%)")

    guided = [sampler.sample_guided(goal, 0.6) for _ in range(1000)]
    n_free_g = sum(1 for q in guided if checker.check_config(q))
    dist_to_goal = [np.linalg.norm(q - goal) for q in guided]
    print(f"  1000 guided samples: {n_free_g} free, "
          f"mean dist to goal={np.mean(dist_to_goal):.3f}")


if __name__ == "__main__":
    main()
