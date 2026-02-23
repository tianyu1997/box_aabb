"""Diagnose anchor box corruption through grow + coarsen stages."""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'src'))
os.chdir(os.path.dirname(__file__))

import numpy as np
from experiments.scenes import load_scenes
from experiments.runner import load_scene_from_config
from planner.pipeline import (
    grow_forest, PandaGCSConfig, find_box_containing,
    _build_adjacency_and_islands, make_planner_config,
)
from planner.sbf_planner import SBFPlanner
from forest.coarsen import coarsen_forest

scenes = load_scenes(["panda_10obs_moderate"])
sc = scenes[0]
robot, scene_obj, qps = load_scene_from_config(sc)
q_start, q_goal = qps[0]
ndim = robot.n_joints

cfg = PandaGCSConfig()
cfg.seed = 0
cfg.max_boxes = 5000

planner_cfg = make_planner_config(cfg)
planner = SBFPlanner(robot=robot, scene=scene_obj, config=planner_cfg)

# Run grow_forest (which creates anchors + grows boxes)
boxes_export, forest_obj, grow_detail = grow_forest(
    planner, q_start, q_goal, cfg.seed,
    cfg.max_consecutive_miss, ndim,
    max_boxes=cfg.max_boxes)

# --- Stage 1: After grow, before coarsen ---
print("\n=== Stage 1: After grow, before coarsen ===")
print(f"Total boxes in forest: {len(forest_obj.boxes)}")
print(f"boxes_export has box 0: {0 in boxes_export}")
print(f"forest.boxes has box 0: {0 in forest_obj.boxes}")
print(f"forest.boxes has box 1: {1 in forest_obj.boxes}")

for anchor_id in [0, 1]:
    qx = q_start if anchor_id == 0 else q_goal
    label = "q_start" if anchor_id == 0 else "q_goal"
    if anchor_id in forest_obj.boxes:
        box = forest_obj.boxes[anchor_id]
        print(f"\n  Box {anchor_id} intervals:")
        for d, (lo, hi) in enumerate(box.joint_intervals):
            in_d = lo - 1e-10 <= qx[d] <= hi + 1e-10
            print(f"    dim {d}: [{lo:.6f}, {hi:.6f}]  {label}[{d}]={qx[d]:.6f}  {'OK' if in_d else 'OUT!'}")
        print(f"  contains({label}): {box.contains(qx)}")
        print(f"  vol: {box.volume:.2e}")
    else:
        print(f"\n  Box {anchor_id} NOT in forest!")
        # Find which box (if any) contains the query
        found = None
        for bid, b in forest_obj.boxes.items():
            if b.contains(qx):
                found = bid
                break
        if found is not None:
            print(f"    But box {found} contains {label}")
        else:
            print(f"    No box contains {label}!")
            # Find nearest
            near_id = find_box_containing(qx, forest_obj.boxes)
            near_box = forest_obj.boxes[near_id]
            print(f"    Nearest: box {near_id}, distance={near_box.distance_to_config(qx):.6f}")

# Check connectivity before coarsen
adj_pre, uf_pre, islands_pre = _build_adjacency_and_islands(
    forest_obj.boxes, period=None)
# Find src/tgt  
src_pre = find_box_containing(q_start, forest_obj.boxes)
tgt_pre = find_box_containing(q_goal, forest_obj.boxes)
n_islands_pre = len(islands_pre)
connected_pre = uf_pre.same(src_pre, tgt_pre) if src_pre and tgt_pre else False
print(f"\n  Pre-coarsen: {n_islands_pre} islands, src={src_pre}, tgt={tgt_pre}, connected={connected_pre}")

# --- Stage 2: Coarsen ---
print("\n=== Stage 2: Coarsening ===")
coarsen_stats = coarsen_forest(
    tree=planner.hier_tree, forest=forest_obj,
    obstacles=planner.obstacles, safety_margin=0.0,
    max_rounds=cfg.coarsen_max_rounds,
)
print(f"Coarsen: {coarsen_stats.n_before} -> {coarsen_stats.n_after} boxes "
      f"({coarsen_stats.n_merges} merges in {coarsen_stats.n_rounds} rounds)")

# --- Stage 3: After coarsen ---
print("\n=== Stage 3: After coarsen ===")
print(f"Total boxes in forest: {len(forest_obj.boxes)}")
print(f"forest.boxes has box 0: {0 in forest_obj.boxes}")
print(f"forest.boxes has box 1: {1 in forest_obj.boxes}")

for anchor_id in [0, 1]:
    qx = q_start if anchor_id == 0 else q_goal
    label = "q_start" if anchor_id == 0 else "q_goal"
    if anchor_id in forest_obj.boxes:
        box = forest_obj.boxes[anchor_id]
        print(f"\n  Box {anchor_id} intervals (post-coarsen):")
        for d, (lo, hi) in enumerate(box.joint_intervals):
            in_d = lo - 1e-10 <= qx[d] <= hi + 1e-10
            print(f"    dim {d}: [{lo:.6f}, {hi:.6f}]  {label}[{d}]={qx[d]:.6f}  {'OK' if in_d else 'OUT!'}")
        print(f"  contains({label}): {box.contains(qx)}")
        print(f"  vol: {box.volume:.2e}")
    else:
        print(f"\n  Box {anchor_id} NOT in forest after coarsen!")
        found = None
        for bid, b in forest_obj.boxes.items():
            if b.contains(qx):
                found = bid
                break
        if found is not None:
            print(f"    But box {found} contains {label}")
            box = forest_obj.boxes[found]
            print(f"    vol: {box.volume:.2e}")
        else:
            print(f"    No box contains {label}!")
            near_id = find_box_containing(qx, forest_obj.boxes)
            near_box = forest_obj.boxes[near_id]
            print(f"    Nearest: box {near_id}, distance={near_box.distance_to_config(qx):.6f}")

# Check connectivity after coarsen
boxes_final = forest_obj.boxes
adj_post, uf_post, islands_post = _build_adjacency_and_islands(
    boxes_final, period=None)
src_post = find_box_containing(q_start, boxes_final)
tgt_post = find_box_containing(q_goal, boxes_final)
in_src = boxes_final[src_post].contains(q_start) if src_post in boxes_final else False
in_tgt = boxes_final[tgt_post].contains(q_goal) if tgt_post in boxes_final else False
n_islands_post = len(islands_post)
connected_post = uf_post.same(src_post, tgt_post) if src_post and tgt_post else False
print(f"\n  Post-coarsen: {n_islands_post} islands, "
      f"src={src_post}(in={in_src}), tgt={tgt_post}(in={in_tgt}), "
      f"connected={connected_post}")
