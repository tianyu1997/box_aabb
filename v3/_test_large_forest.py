"""Test SBF with large forest."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
os.chdir(os.path.dirname(__file__))

import numpy as np
from experiments.scenes import load_scenes
from experiments.runner import load_scene_from_config
from planner.pipeline import (
    grow_and_prepare, PandaGCSConfig, find_box_containing,
    _build_adjacency_and_islands,
    run_method_with_bridge, _solve_method_dijkstra,
)
from forest.collision import CollisionChecker

scenes = load_scenes(["panda_10obs_moderate"])
sc = scenes[0]
robot, scene_obj, qps = load_scene_from_config(sc)
q_start, q_goal = qps[0]

cfg = PandaGCSConfig()
cfg.seed = 0
cfg.max_boxes = 5000
ndim = robot.n_joints

prep = grow_and_prepare(robot, scene_obj, cfg, q_start, q_goal, ndim)

# Try planning
raw = run_method_with_bridge(
    _solve_method_dijkstra, 'Dijkstra', prep, cfg, q_start, q_goal, ndim)

if raw and raw.get('success'):
    wps = raw['waypoints']
    cost = raw['cost']
    print(f"Path found: {len(wps)} waypoints, cost={cost:.4f}")
    cc = CollisionChecker(robot=robot, scene=scene_obj)
    n_col = sum(1 for wp in wps if cc.check_config_collision(np.asarray(wp)))
    print(f"  Waypoints in collision: {n_col}/{len(wps)}")
    n_seg = 0
    for i in range(len(wps) - 1):
        if cc.check_segment_collision(np.asarray(wps[i]), np.asarray(wps[i+1]), 0.02):
            n_seg += 1
    print(f"  Segments in collision: {n_seg}/{len(wps)-1}")
else:
    print("No path found")
    # Check connectivity
    boxes = prep['boxes']
    src = find_box_containing(q_start, boxes)
    tgt = find_box_containing(q_goal, boxes)
    adj, uf, islands = _build_adjacency_and_islands(boxes, period=None)
    print(f"  src={src} (in box: {boxes[src].contains(q_start) if src else False})")
    print(f"  tgt={tgt} (in box: {boxes[tgt].contains(q_goal) if tgt else False})")
    print(f"  islands: {len(islands)}")
    if src and tgt:
        print(f"  connected: {uf.same(src, tgt)}")
        # Check island sizes
        for isl_id, isl_members in islands.items():
            if src in isl_members or tgt in isl_members:
                print(f"  island with {'src' if src in isl_members else 'tgt'}: "
                      f"{len(isl_members)} boxes")
