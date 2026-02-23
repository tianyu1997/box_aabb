"""Check SBF Panda path for collisions and q6 behavior."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))

import numpy as np
from aabb.robot import load_robot
from forest.scene import Scene
from forest.collision import CollisionChecker
from experiments.scenes import load_scenes
from experiments.runner import load_scene_from_config

# Load scene
scenes = load_scenes(["panda_10obs_moderate"])
sc = scenes[0]
robot = load_robot("panda")

# Build scene using runner's logic
_, scene_obj, query_pairs = load_scene_from_config(sc)
checker = CollisionChecker(robot=robot, scene=scene_obj)

q_start = np.array(sc["q_start"], dtype=np.float64)
q_goal = np.array(sc["q_goal"], dtype=np.float64)
print(f"q_start = {q_start}")
print(f"q_goal  = {q_goal}")
print(f"q_start[6] = {q_start[6]:.4f}")
print(f"q_goal[6]  = {q_goal[6]:.4f}")

# Create SBFPlanner with correct scene
from planner.sbf_planner import SBFPlanner as _SBF

sbf = _SBF(robot=robot, scene=scene_obj)
result = sbf.plan(q_start, q_goal, seed=0)
print(f"\nSBF result: success={result.success}, cost={result.cost:.4f}")
print(f"  path shape = {result.path.shape}")
print(f"  planning_time = {result.planning_time:.4f}s")

path = result.path
print(f"\n=== q6 (joint 7) in SBF path ===")
print(f"  min={path[:,6].min():.4f} max={path[:,6].max():.4f} std={path[:,6].std():.6f}")
print(f"  all q6 values: {[f'{v:.4f}' for v in path[:, 6]]}")

# Check each waypoint for collision
print(f"\n=== Waypoint collision check ===")
n_collisions = 0
for i, wp in enumerate(path):
    col = checker.check_config_collision(wp)
    if col:
        print(f"  COLLISION at waypoint {i}: q={wp}")
        n_collisions += 1
print(f"  {n_collisions}/{len(path)} waypoints in collision")

# Check interpolated segments for collision
print(f"\n=== Segment collision check (fine interpolation) ===")
n_seg_collisions = 0
n_total_checks = 0
for i in range(len(path) - 1):
    q0, q1 = path[i], path[i+1]
    dist = np.linalg.norm(q1 - q0)
    n_steps = max(int(dist / 0.05) + 1, 2)
    seg_has_collision = False
    for t in np.linspace(0, 1, n_steps):
        q = q0 + t * (q1 - q0)
        n_total_checks += 1
        if checker.check_config_collision(q):
            if not seg_has_collision:
                print(f"  COLLISION in segment {i}->{i+1}: t={t:.3f}, q={q}, dist={dist:.4f}")
                seg_has_collision = True
                n_seg_collisions += 1
print(f"  {n_seg_collisions}/{len(path)-1} segments have collisions")
print(f"  Total interpolation checks: {n_total_checks}")

# Compare path length in 7D vs 6D
diffs = np.diff(path, axis=0)
dist_7d = np.sum(np.sqrt(np.sum(diffs**2, axis=1)))
dist_6d = np.sum(np.sqrt(np.sum(diffs[:, :6]**2, axis=1)))
print(f"\n=== Path length comparison ===")
print(f"  7D path length: {dist_7d:.4f}")
print(f"  6D path length: {dist_6d:.4f}")
print(f"  q6 contribution: {dist_7d - dist_6d:.4f}")
