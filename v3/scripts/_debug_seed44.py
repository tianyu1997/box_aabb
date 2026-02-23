"""Debug seed 44 collision issue."""
import sys
sys.path.insert(0, 'src')

import numpy as np
from aabb.robot import load_robot
from forest.collision import CollisionChecker
from forest.scene import Scene
from baselines.rrt_family import plan_rrt_connect

robot = load_robot('2dof_planar')
q_start = np.array([0.8 * np.pi, 0.2])
q_goal = np.array([-0.7 * np.pi, -0.4])
jl = robot.joint_limits
period = 2 * np.pi
half = np.pi

# Reproduce scene generation for seed 44
rng = np.random.default_rng(44)
max_trials = 500
scene = None
for trial_i in range(max_trials):
    sc = Scene()
    for i in range(8):
        cx = float(rng.uniform(-1.8, 1.8))
        cy = float(rng.uniform(-1.8, 1.8))
        w = float(rng.uniform(0.3, 0.8))
        h = float(rng.uniform(0.3, 0.8))
        sc.add_obstacle([cx - w / 2, cy - h / 2],
                        [cx + w / 2, cy + h / 2], name=f"obs_{i}")
    checker = CollisionChecker(robot=robot, scene=sc)
    if checker.check_config_collision(q_start):
        continue
    if checker.check_config_collision(q_goal):
        continue
    if not checker.check_segment_collision(q_start, q_goal, 0.03):
        continue
    if not checker.check_segment_collision(q_start, q_goal, 0.03, period=period):
        continue
    res = plan_rrt_connect(
        q_start, q_goal, jl, checker,
        timeout=2.0, step_size=0.3, resolution=0.05,
        seed=trial_i + 7777, period=period)
    if not res['success']:
        continue
    scene = sc
    print(f"Scene found at trial {trial_i}")
    break

if scene is None:
    print("No scene found!")
    sys.exit(1)

checker = CollisionChecker(robot=robot, scene=scene)

# The raw pipeline waypoints (from debug output)
wps = [
    np.array([+2.51327412, +0.200]),
    np.array([+2.614, +0.785]),
    np.array([+2.749, +1.571]),
    np.array([+2.945, +2.356]),
    np.array([+3.142, +3.142]),
    np.array([-2.356, -0.785]),
    np.array([-2.199, -0.400]),
]

print('\n=== Config collision check for each waypoint ===')
for i, q in enumerate(wps):
    col = checker.check_config_collision(q)
    print(f'  wp[{i}] [{q[0]:+.4f},{q[1]:+.4f}] config_col={col}')

print('\n=== Segment collision checks ===')
for i in range(len(wps) - 1):
    q1, q2 = wps[i], wps[i + 1]
    euc_col = checker.check_segment_collision(q1, q2, 0.03)
    geo_col = checker.check_segment_collision(q1, q2, 0.03, period=period)
    euc_diff = q2 - q1
    geo_diff = ((euc_diff + half) % period) - half
    wraps = any(abs(euc_diff[k]) > half for k in range(len(euc_diff)))
    print(f'  seg[{i}] euc_col={euc_col} geo_col={geo_col} wraps={wraps}')

# Fine scan of seg[5]
print('\n=== Fine scan of seg[5] ===')
q1, q2 = wps[5], wps[6]
diff = ((q2 - q1) + half) % period - half
dist = float(np.linalg.norm(diff))
n = max(2, int(dist / 0.003) + 1)
n_col = 0
for k in range(n + 1):
    t = k / n
    q = q1 + t * diff
    q = ((q + half) % period) - half
    if checker.check_config_collision(q):
        n_col += 1
        if n_col <= 5:
            print(f'  t={t:.4f} q=[{q[0]:+.4f},{q[1]:+.4f}] COLLISION')
print(f'  Total: {n_col}/{n+1} points in collision')

# Check: does the SBF checker and bench checker agree?
print('\n=== SBF adapter vs bench checker comparison ===')
from baselines import SBFAdapter
sbf = SBFAdapter(method="dijkstra")
sbf_cfg = {'seed': 44, 'max_boxes': 400, 'max_consecutive_miss': 50}
sbf.setup(robot, scene, sbf_cfg)
ad_checker = sbf._checker

# Compare on a few test points
test_pts = [wps[5], wps[6], (wps[5] + wps[6]) / 2]
for i, q in enumerate(test_pts):
    bc = checker.check_config_collision(q)
    ac = ad_checker.check_config_collision(q)
    print(f'  pt[{i}] [{q[0]:+.4f},{q[1]:+.4f}] bench={bc} adapter={ac} agree={bc==ac}')

# Compare segment check
bc_seg = checker.check_segment_collision(wps[5], wps[6], 0.03, period=period)
ac_seg = ad_checker.check_segment_collision(wps[5], wps[6], 0.03, period=period)
print(f'  seg[5] bench_geo_col={bc_seg} adapter_geo_col={ac_seg}')
