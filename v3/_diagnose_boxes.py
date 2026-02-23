"""Diagnose SBF box safety: are 'safe' boxes really collision-free?"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
os.chdir(os.path.dirname(__file__))

import numpy as np
from aabb.robot import load_robot
from forest.collision import CollisionChecker
from experiments.scenes import load_scenes
from experiments.runner import load_scene_from_config, create_planner
from planner.pipeline import (
    grow_and_prepare, run_method_with_bridge, _solve_method_dijkstra,
    PandaGCSConfig, make_planner_config, find_box_containing,
    _build_adjacency_and_islands,
)
from planner.sbf_planner import SBFPlanner

# Load scene
scenes = load_scenes(["panda_10obs_moderate"])
sc = scenes[0]
robot, scene_obj, query_pairs = load_scene_from_config(sc)
checker = CollisionChecker(robot=robot, scene=scene_obj)

q_start, q_goal = query_pairs[0]
print(f"q_start = {np.array2string(q_start, precision=3)}")
print(f"q_goal  = {np.array2string(q_goal, precision=3)}")

# Build forest via pipeline
cfg = PandaGCSConfig()
cfg.seed = 0
ndim = robot.n_joints
prep = grow_and_prepare(robot, scene_obj, cfg, q_start, q_goal, ndim)
boxes = prep['boxes']
forest_obj = prep['forest_obj']
planner = prep['planner']
period = planner._period  # None for Panda

# Build adjacency  
adj, uf, islands = _build_adjacency_and_islands(boxes, period=period)
src = find_box_containing(q_start, boxes)
tgt = find_box_containing(q_goal, boxes)

print(f"\n=== Forest info ===")
print(f"  Total boxes: {len(boxes)}")
print(f"  ndim: {ndim}")
print(f"  planner._period: {planner._period}")
print(f"  src box: {src}")
print(f"  tgt box: {tgt}")

# Check which boxes contain start/goal
for bid, box in boxes.items():
    lo = np.array([l for l, h in box.joint_intervals])
    hi = np.array([h for l, h in box.joint_intervals])
    if np.all(q_start >= lo - 1e-9) and np.all(q_start <= hi + 1e-9):
        print(f"  q_start inside box {bid}")
    if np.all(q_goal >= lo - 1e-9) and np.all(q_goal <= hi + 1e-9):
        print(f"  q_goal inside box {bid}")

# Check box safety: sample random configs and compare with interval check
print(f"\n=== Box safety validation ===")
rng = np.random.default_rng(42)
n_false_negatives = 0
for bid, box in list(boxes.items()):
    lo = np.array([l for l, h in box.joint_intervals])
    hi = np.array([h for l, h in box.joint_intervals])
    
    # Interval check
    box_col = checker.check_box_collision(box.joint_intervals)
    
    if box_col:
        continue  # Box marked unsafe, skip
    
    # Box marked safe — verify with sampling
    n_samples = 200
    n_col = 0
    first_col_config = None
    for _ in range(n_samples):
        q = rng.uniform(lo, hi)
        if checker.check_config_collision(q):
            n_col += 1
            if first_col_config is None:
                first_col_config = q.copy()
    
    if n_col > 0:
        n_false_negatives += 1
        print(f"  FALSE NEGATIVE: box {bid} marked SAFE but {n_col}/{n_samples} samples collide!")
        print(f"    box lo = {np.array2string(lo, precision=3)}")
        print(f"    box hi = {np.array2string(hi, precision=3)}")
        print(f"    box size = {np.array2string(hi - lo, precision=3)}")
        print(f"    first colliding config = {np.array2string(first_col_config, precision=3)}")
        
        # Check which links collide
        positions = robot.get_link_positions(first_col_config)
        obstacles = scene_obj.get_obstacles()
        for li in range(1, len(positions)):
            p_s = positions[li - 1]
            p_e = positions[li]
            link_min = np.minimum(p_s, p_e)
            link_max = np.maximum(p_s, p_e)
            for obs in obstacles:
                from forest.collision import aabb_overlap
                o_min = obs.min_point
                o_max = obs.max_point
                if aabb_overlap(link_min, link_max, o_min, o_max):
                    print(f"      link {li}: [{np.array2string(link_min, precision=4)}, {np.array2string(link_max, precision=4)}]")
                    print(f"        overlaps obs: [{np.array2string(o_min, precision=4)}, {np.array2string(o_max, precision=4)}]")

if n_false_negatives == 0:
    print(f"  All safe boxes verified correct ({len(boxes)} boxes checked)")
else:
    print(f"\n  !!! {n_false_negatives} false negatives found !!!")

# Now run Dijkstra and check path details
print(f"\n=== Dijkstra path analysis ===")
from planner.pipeline import (
    _dijkstra_box_graph, _shortcut_box_sequence,
    _build_transition_waypoints, _pull_tight_in_bounds,
    _refine_path_in_boxes, _geometric_shortcut,
)

box_seq, raw_dist = _dijkstra_box_graph(boxes, adj, src, tgt, period=None, q_goal=q_goal)
print(f"  Dijkstra box_seq: {box_seq}")
short_seq = _shortcut_box_sequence(box_seq, adj)
print(f"  Shortcut box_seq: {short_seq}")

# Build transition waypoints
waypoints, wp_bounds = _build_transition_waypoints(
    short_seq, boxes, q_start, q_goal, period=None)
print(f"\n  Transition waypoints ({len(waypoints)}):")
for i, (wp, (lo, hi)) in enumerate(zip(waypoints, wp_bounds)):
    print(f"    wp[{i}] = {np.array2string(wp, precision=4)}")
    print(f"      bounds lo = {np.array2string(lo, precision=4)}")
    print(f"      bounds hi = {np.array2string(hi, precision=4)}")
    in_bounds = np.all(wp >= lo - 1e-9) and np.all(wp <= hi + 1e-9)
    print(f"      in_bounds: {in_bounds}")

# Pull-tight
wps_tight = _pull_tight_in_bounds(waypoints, wp_bounds, period=None)
print(f"\n  After pull-tight:")
for i, wp in enumerate(wps_tight):
    lo, hi = wp_bounds[i]
    in_bounds = np.all(wp >= lo - 1e-9) and np.all(wp <= hi + 1e-9)
    col = checker.check_config_collision(wp)
    print(f"    wp[{i}] = {np.array2string(wp, precision=4)}  in_bounds={in_bounds}  collision={col}")

# SOCP refine
refined_wps, refined_cost = _refine_path_in_boxes(
    wps_tight, wp_bounds, q_start, q_goal, ndim, period=None)
print(f"\n  After SOCP (cost={refined_cost:.4f}):")
for i, wp in enumerate(refined_wps):
    lo, hi = wp_bounds[i]
    in_bounds = np.all(wp >= lo - 1e-9) and np.all(wp <= hi + 1e-9)
    col = checker.check_config_collision(wp)
    print(f"    wp[{i}] = {np.array2string(wp, precision=4)}  in_bounds={in_bounds}  collision={col}")

# Geometric shortcut
short_wps, short_cost = _geometric_shortcut(refined_wps, boxes, short_seq, period=None)
print(f"\n  After geometric shortcut (cost={short_cost:.4f}, {len(short_wps)} wps):")
for i, wp in enumerate(short_wps):
    col = checker.check_config_collision(wp)
    print(f"    wp[{i}] = {np.array2string(wp, precision=4)}  collision={col}")

# Check each segment
print(f"\n=== Segment analysis ===")
for i in range(len(short_wps) - 1):
    q0, q1 = short_wps[i], short_wps[i+1]
    dist = np.linalg.norm(q1 - q0)
    seg_col = checker.check_segment_collision(q0, q1, 0.02)
    
    # Check if segment is inside any box in sequence
    from planner.pipeline import _segment_in_box
    in_any_box = False
    for bid in short_seq:
        if _segment_in_box(q0, q1, boxes[bid], n_samples=20):
            in_any_box = True
            print(f"  Segment {i}->{i+1}: dist={dist:.4f}, collision={seg_col}, in_box={bid}")
            
            # If segment is in a box and collides, check the box
            if seg_col:
                box = boxes[bid]
                lo = np.array([l for l, h in box.joint_intervals])
                hi = np.array([h for l, h in box.joint_intervals])
                box_col = checker.check_box_collision(box.joint_intervals)
                print(f"    Box {bid}: safe={not box_col}")
                print(f"    Box lo = {np.array2string(lo, precision=4)}")
                print(f"    Box hi = {np.array2string(hi, precision=4)}")
                
                # Check if both endpoints are in the box
                q0_in = np.all(q0 >= lo - 1e-9) and np.all(q0 <= hi + 1e-9)
                q1_in = np.all(q1 >= lo - 1e-9) and np.all(q1 <= hi + 1e-9)
                print(f"    q0 in box: {q0_in}")
                print(f"    q1 in box: {q1_in}")
                
                # Which dims are out?
                for d in range(ndim):
                    if q0[d] < lo[d] - 1e-9 or q0[d] > hi[d] + 1e-9:
                        print(f"      q0 dim {d}: {q0[d]:.4f} not in [{lo[d]:.4f}, {hi[d]:.4f}]")
                    if q1[d] < lo[d] - 1e-9 or q1[d] > hi[d] + 1e-9:
                        print(f"      q1 dim {d}: {q1[d]:.4f} not in [{lo[d]:.4f}, {hi[d]:.4f}]")
            break
    
    if not in_any_box:
        print(f"  Segment {i}->{i+1}: dist={dist:.4f}, collision={seg_col}, NOT in any box!")
        # Check each endpoint in each box
        for bid in short_seq:
            box = boxes[bid]
            lo = np.array([l for l, h in box.joint_intervals])
            hi = np.array([h for l, h in box.joint_intervals])
            q0_in = np.all(q0 >= lo - 1e-9) and np.all(q0 <= hi + 1e-9)
            q1_in = np.all(q1 >= lo - 1e-9) and np.all(q1 <= hi + 1e-9)
            if q0_in or q1_in:
                print(f"    Box {bid}: q0_in={q0_in}, q1_in={q1_in}")
                for d in range(ndim):
                    if q0[d] < lo[d] - 1e-9 or q0[d] > hi[d] + 1e-9:
                        print(f"      q0 dim {d}: {q0[d]:.4f} not in [{lo[d]:.4f}, {hi[d]:.4f}]")
                    if q1[d] < lo[d] - 1e-9 or q1[d] > hi[d] + 1e-9:
                        print(f"      q1 dim {d}: {q1[d]:.4f} not in [{lo[d]:.4f}, {hi[d]:.4f}]")
