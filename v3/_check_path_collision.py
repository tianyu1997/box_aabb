"""Verify planner paths are collision-free after fixes."""
import sys, numpy as np
sys.path.insert(0, 'src')

from aabb.robot import load_robot
from planner.pipeline import PandaGCSConfig, build_panda_scene, set_verbose
from forest.collision import CollisionChecker
from baselines import SBFAdapter
from baselines.ompl_adapter import OMPLPlanner

set_verbose(False)

def check_path(path, checker, label, resolution=0.05, period=None):
    if path is None:
        print(f"  {label}: path=None")
        return
    n_pts = len(path)
    n_seg = n_pts - 1
    n_coll = 0
    for i in range(n_seg):
        if checker.check_segment_collision(path[i], path[i+1], resolution, period=period):
            n_coll += 1
    status = "COLLISION" if n_coll > 0 else "CLEAN"
    print(f"  {label}: {n_pts} pts, {n_seg} segs, {n_coll} colliding -> {status}")

# ── Panda 7DOF ──
print("=" * 60)
print("Panda 7DOF (seed=42)")
print("=" * 60)
robot = load_robot('panda')
rng = np.random.default_rng(1042)
pcfg = PandaGCSConfig(); pcfg.n_obstacles = 5
q_s = np.array([0.5, -1.2, 0.5, -2.5, 0.5, 0.8, 1.5])
q_g = np.array([-2.0, 1.2, -1.8, -0.5, -2.0, 3.0, -1.8])
scene = build_panda_scene(rng, pcfg, robot=robot, q_start=q_s, q_goal=q_g)
checker = CollisionChecker(robot=robot, scene=scene)

# SBF
import copy
scene_sbf = copy.deepcopy(scene)
ada = SBFAdapter(method='dijkstra')
ada.setup(robot, scene_sbf, {
    'ffb_min_edge': 0.1, 'max_boxes': 1500,
    'seed': 42, 'no_cache': True,
})
res = ada.plan(q_s, q_g, timeout=10)
if res.success:
    check_path(res.path, checker, "SBF-Dijkstra")
    print(f"    time={res.planning_time:.3f}s, cost={res.cost:.3f}")
else:
    print("  SBF: FAILED")

# RRT Connect
print()
planner = OMPLPlanner(algorithm='RRTConnect')
planner.setup(robot, scene, {'seed': 42, 'step_size': 0.5})
res = planner.plan(q_s, q_g, timeout=10)
if res.success:
    check_path(res.path, checker, "RRTConnect")
    print(f"    time={res.planning_time:.3f}s, cost={res.cost:.3f}")
else:
    print("  RRTConnect: FAILED")

# BIT*
print()
planner2 = OMPLPlanner(algorithm='BITstar')
planner2.setup(robot, scene, {'seed': 42, 'step_size': 0.5})
res2 = planner2.plan(q_s, q_g, timeout=10)
if res2.success:
    check_path(res2.path, checker, "BITstar")
    print(f"    time={res2.planning_time:.3f}s, cost={res2.cost:.3f}")
else:
    print("  BITstar: FAILED")

print("\nDone.")
