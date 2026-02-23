"""Test with much larger forest to achieve connectivity."""
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
    find_box_containing, _build_adjacency_and_islands,
)
from forest.collision import CollisionChecker

logging.basicConfig(level=logging.WARNING)

scenes = load_scenes(["panda_10obs_moderate"])
sc = scenes[0]
robot, scene_obj, qps = load_scene_from_config(sc)
q_start, q_goal = qps[0]
ndim = robot.n_joints

for max_b in [10000, 20000]:
    print(f"\n{'='*60}")
    print(f"=== max_boxes={max_b} ===")
    print(f"{'='*60}")
    
    cfg = PandaGCSConfig()
    cfg.seed = 0
    cfg.max_boxes = max_b
    
    prep = grow_and_prepare(robot, scene_obj, cfg, q_start, q_goal, ndim,
                            no_cache=True)
    
    boxes = prep['boxes']
    src = find_box_containing(q_start, boxes)
    tgt = find_box_containing(q_goal, boxes)
    adj, uf, islands = _build_adjacency_and_islands(boxes, period=None)
    n_islands = len(islands)
    connected = uf.same(src, tgt) if src and tgt else False
    src_in = boxes[src].contains(q_start) if src in boxes else False
    tgt_in = boxes[tgt].contains(q_goal) if tgt in boxes else False
    print(f"  boxes={len(boxes)}, islands={n_islands}, connected={connected}")
    print(f"  src={src}(in={src_in}), tgt={tgt}(in={tgt_in})")
    
    if connected:
        print("  Forest is connected! Trying Dijkstra...")
        raw = run_method_with_bridge(
            _solve_method_dijkstra, 'Dijkstra', prep, cfg, q_start, q_goal, ndim)
        if raw and raw.get('success'):
            wps = raw['waypoints']
            cost = raw['cost']
            print(f"  Path found: {len(wps)} waypoints, cost={cost:.4f}")
            cc = CollisionChecker(robot=robot, scene=scene_obj)
            n_col = sum(1 for wp in wps if cc.check_config_collision(np.asarray(wp)))
            print(f"  Waypoints in collision: {n_col}/{len(wps)}")
            n_seg = sum(1 for i in range(len(wps)-1)
                       if cc.check_segment_collision(np.asarray(wps[i]),
                                                     np.asarray(wps[i+1]), 0.02))
            print(f"  Segments in collision: {n_seg}/{len(wps)-1}")
            break
    else:
        # Check island sizes
        island_sizes = sorted([len(isl) for isl in islands], reverse=True)
        print(f"  Island sizes: {island_sizes[:5]}")
