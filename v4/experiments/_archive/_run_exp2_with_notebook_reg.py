import pickle
import sys
import os
import json
import time

sys.path.insert(0, "/home/tian/桌面/box_aabb/gcs-science-robotics")
sys.path.insert(0, "/home/tian/桌面/box_aabb/v4")

from experiments.paper_exp2_e2e_gcs import SCENE_NAMES, ARTIFACTS_DIR
from experiments.marcucci_scenes import load_marcucci_scene
from experiments.runner import load_scene_from_config
from baselines.iris_np_gcs import IRISNPGCSPlanner

def main():
    scene_name = "shelves"
    
    # 1. Load notebook pre-generated regions
    reg_path = "/home/tian/桌面/box_aabb/gcs-science-robotics/data/prm_comparison/IRIS.reg"
    with open(reg_path, "rb") as f:
        notebook_regions_dict = pickle.load(f)
    
    notebook_regions = list(notebook_regions_dict.values())
    print(f"Loaded {len(notebook_regions)} regions from notebook IRIS.reg")

    # 2. Setup scene
    scene_cfg = load_marcucci_scene(scene_name, robot="iiwa14")
    robot, scene, query_pairs = load_scene_from_config(scene_cfg)
    
    print(f"Scene: {scene_name}, {len(scene_cfg['obstacles'])} obstacles")
    
    # 3. Create dummy planner and inject regions
    planner = IRISNPGCSPlanner()
    planner.setup(robot, scene, {"seed": 0})
    
    # Need to build plant for the planner first, but we can do it manually or let plan() do it
    # We will spoof the regions
    planner._regions = notebook_regions
    planner._built = True # Prevent it from generating new regions
    
    # 4. Run tests
    print("\nTesting original notebook queries (adjacent regions):")
    notebook_pairs = [
        ("AS", "TS"), ("TS", "CS"), ("CS", "LB"),
        ("LB", "RB"), ("RB", "AS"),
    ]
    
    # Get seed points to reconstruct these queries
    from experiments.marcucci_scenes import get_iiwa14_seed_points
    pts = get_iiwa14_seed_points()
    
    for s_name, g_name in notebook_pairs:
        qs, qg = pts[s_name], pts[g_name]
        print(f"  {s_name} -> {g_name}...", end=" ")
        
        try:
            res = planner.plan(qs, qg, timeout=60.0)
            if res.success:
                print(f"SUCCESS (time={res.planning_time:.3f}s)")
            else:
                print(f"FAIL: {res.reason}")
        except Exception as e:
            print(f"ERROR: {e}")

    print("\nTesting our original query pair (LB -> TS, cross regions):")
    # From origin marcucci_scenes: [("LB", "TS"), ...] -> this is what failed in out exp2
    try:
        qs, qg = pts["LB"], pts["TS"]
        print(f"  LB -> TS...", end=" ")
        res = planner.plan(qs, qg, timeout=60.0)
        if res.success:
            print(f"SUCCESS (time={res.planning_time:.3f}s)")
        else:
            print(f"FAIL: {res.reason}")
    except Exception as e:
        print(f"ERROR: {e}")
        
    print("\nTest finished")

if __name__ == "__main__":
    main()
