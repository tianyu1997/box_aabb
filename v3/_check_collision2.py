"""Check SBF Panda path for collisions and q6 behavior — using SBFAdapter (pipeline)."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
os.chdir(os.path.dirname(__file__))

import numpy as np
from aabb.robot import load_robot
from forest.collision import CollisionChecker
from experiments.scenes import load_scenes
from experiments.runner import load_scene_from_config, create_planner

# Load scene
scenes = load_scenes(["panda_10obs_moderate"])
sc = scenes[0]
robot, scene_obj, query_pairs = load_scene_from_config(sc)
checker = CollisionChecker(robot=robot, scene=scene_obj)

q_start, q_goal = query_pairs[0]
print(f"q_start = {q_start}")
print(f"q_goal  = {q_goal}")
print(f"q_start[6] = {q_start[6]:.4f}, q_goal[6] = {q_goal[6]:.4f}")
print(f"Obstacles: {len(scene_obj.get_obstacles())}")

# Run SBF via adapter (same as benchmark)
planner = create_planner({"type": "SBF", "method": "dijkstra"})
planner.setup(robot, scene_obj, {"seed": 0, "max_boxes": 2000})
result = planner.plan(q_start, q_goal, timeout=30.0)
print(f"\nSBF result: success={result.success}, cost={result.cost:.4f}")
print(f"  path shape = {result.path.shape}")
print(f"  planning_time = {result.planning_time:.4f}s")

path = result.path
print(f"\n=== q6 (joint 7) in SBF path ===")
print(f"  min={path[:,6].min():.4f} max={path[:,6].max():.4f} std={path[:,6].std():.6f}")
for i, wp in enumerate(path):
    print(f"  wp[{i:2d}] q6={wp[6]:+.4f}  q_full={np.array2string(wp, precision=3, suppress_small=True)}")

# Check each waypoint for collision
print(f"\n=== Waypoint collision check ===")
n_collisions = 0
for i, wp in enumerate(path):
    col = checker.check_config_collision(wp)
    if col:
        print(f"  COLLISION at waypoint {i}: q6={wp[6]:.4f}")
        n_collisions += 1
print(f"  {n_collisions}/{len(path)} waypoints in collision")

# Check interpolated segments for collision
print(f"\n=== Segment collision check (fine interpolation, resolution=0.02) ===")
n_seg_collisions = 0
n_total_checks = 0
for i in range(len(path) - 1):
    q0, q1 = path[i], path[i+1]
    dist = np.linalg.norm(q1 - q0)
    n_steps = max(int(dist / 0.02) + 1, 2)
    seg_collisions = 0
    for t in np.linspace(0, 1, n_steps):
        q = q0 + t * (q1 - q0)
        n_total_checks += 1
        if checker.check_config_collision(q):
            seg_collisions += 1
    if seg_collisions > 0:
        print(f"  COLLISION in segment {i}->{i+1}: {seg_collisions}/{n_steps} points, dist={dist:.4f}")
        # show where collision starts
        for t in np.linspace(0, 1, n_steps):
            q = q0 + t * (q1 - q0)
            if checker.check_config_collision(q):
                print(f"    first collision at t={t:.3f}, q6={q[6]:.4f}")
                break
        n_seg_collisions += 1
print(f"  {n_seg_collisions}/{len(path)-1} segments have collisions")
print(f"  Total interpolation checks: {n_total_checks}")

# Compare path length in 7D vs 6D
diffs = np.diff(path, axis=0)
dist_7d = np.sum(np.sqrt(np.sum(diffs**2, axis=1)))
dist_6d = np.sum(np.sqrt(np.sum(diffs[:, :6]**2, axis=1)))
q6_abs = np.sum(np.abs(diffs[:, 6]))
print(f"\n=== Path length comparison ===")
print(f"  7D path length: {dist_7d:.4f}")
print(f"  6D path length: {dist_6d:.4f}")
print(f"  q6 abs movement: {q6_abs:.4f}")
