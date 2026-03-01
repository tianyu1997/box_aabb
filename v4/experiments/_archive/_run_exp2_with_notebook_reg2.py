import pickle
import sys
import os
import time
import numpy as np

sys.path.insert(0, "/home/tian/桌面/box_aabb/gcs-science-robotics")
sys.path.insert(0, "/home/tian/桌面/box_aabb/v4")

from experiments.marcucci_scenes import load_marcucci_scene
from experiments.runner import load_scene_from_config
from baselines.iris_np_gcs import IRISNPGCSPlanner

class DummyPlanner(IRISNPGCSPlanner):
    def plan(self, start: np.ndarray, goal: np.ndarray, timeout: float = 60.0):
        # Only setup drake plant if not
        if self._plant is None:
            self._build_drake_plant()
        
        # Don't generate regions, we just use self._regions
        q_start = np.asarray(start, dtype=np.float64)
        q_goal = np.asarray(goal, dtype=np.float64)
        
        path = self._solve_gcs(q_start, q_goal, self._regions)
        if path is not None and len(path) > 1:
            return type('Obj', (object,), {'success': True, 'planning_time': 0.1, 'path': path, 'cost': 1.0})
        else:
            return type('Obj', (object,), {'success': False, 'planning_time': 0.1, 'reason': 'GCS failed'})

def main():
    scene_name = "shelves"
    
    # Load notebook pre-generated regions
    reg_path = "/home/tian/桌面/box_aabb/gcs-science-robotics/data/prm_comparison/IRIS.reg"
    with open(reg_path, "rb") as f:
        notebook_regions_dict = pickle.load(f)
    print(f"Loaded {len(notebook_regions_dict)} regions from notebook")

    # Load scene
    scene_cfg = load_marcucci_scene(scene_name, robot="iiwa14")
    robot, scene, _ = load_scene_from_config(scene_cfg)
    
    planner = DummyPlanner()
    planner.setup(robot, scene, {"seed": 0})
    planner._regions = list(notebook_regions_dict.values())
    planner._built = True

    from experiments.marcucci_scenes import get_iiwa14_seed_points
    pts = get_iiwa14_seed_points()
    
    notebook_pairs = [
        ("AS", "TS"), ("TS", "CS"), ("CS", "LB"),
        ("LB", "RB"), ("RB", "AS"),
    ]
    print("\nTesting adjacent pairs:")
    for s_name, g_name in notebook_pairs:
        qs, qg = pts[s_name], pts[g_name]
        print(f"  {s_name} -> {g_name}...", end=" ")
        try:
            res = planner.plan(qs, qg)
            print(f"SUCCESS" if res.success else f"FAIL: {getattr(res,'reason','?')}")
        except Exception as e:
            print(f"ERROR: {e}")

    print("\nTesting cross pair (LB -> TS):")
    try:
        qs, qg = pts["LB"], pts["TS"]
        print(f"  LB -> TS...", end=" ")
        res = planner.plan(qs, qg)
        print(f"SUCCESS" if res.success else f"FAIL: {getattr(res,'reason','?')}")
    except Exception as e:
        print(f"ERROR: {e}")

if __name__ == "__main__":
    main()
