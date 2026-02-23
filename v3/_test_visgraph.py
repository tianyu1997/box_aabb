"""Test full pipeline with VisGraph fallback."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
os.chdir(os.path.dirname(__file__))

import logging
import numpy as np
from experiments.scenes import load_scenes
from experiments.runner import load_scene_from_config
from planner.pipeline import (
    grow_and_prepare, PandaGCSConfig,
    run_method_with_bridge, _solve_method_dijkstra,
    run_method_visgraph,
)
from forest.collision import CollisionChecker

logging.basicConfig(level=logging.WARNING)

scenes = load_scenes(["panda_10obs_moderate"])
sc = scenes[0]
robot, scene_obj, qps = load_scene_from_config(sc)
q_start, q_goal = qps[0]
ndim = robot.n_joints

cfg = PandaGCSConfig()
cfg.seed = 0
cfg.max_boxes = 5000

print("=== Growing forest ===")
prep = grow_and_prepare(robot, scene_obj, cfg, q_start, q_goal, ndim)

print(f"\n=== Trying Dijkstra + bridge ===")
raw = run_method_with_bridge(
    _solve_method_dijkstra, 'Dijkstra', prep, cfg, q_start, q_goal, ndim)

if raw and raw.get('success'):
    print(f"Dijkstra succeeded: {len(raw['waypoints'])} wp, cost={raw['cost']:.4f}")
else:
    print("Dijkstra failed, trying VisGraph fallback...")
    
    cc = CollisionChecker(robot=robot, scene=scene_obj)
    raw = run_method_visgraph(prep, cfg, q_start, q_goal, cc, ndim)
    
    if raw and raw.get('success'):
        wps = raw['waypoints']
        cost = raw['cost']
        print(f"\nVisGraph found path: {len(wps)} waypoints, cost={cost:.4f}")
        n_col = sum(1 for wp in wps if cc.check_config_collision(np.asarray(wp)))
        print(f"  Waypoints in collision: {n_col}/{len(wps)}")
        n_seg = 0
        for i in range(len(wps) - 1):
            if cc.check_segment_collision(np.asarray(wps[i]), np.asarray(wps[i+1]), 0.02):
                n_seg += 1
        print(f"  Segments in collision: {n_seg}/{len(wps)-1}")
    else:
        print("VisGraph also failed.")
