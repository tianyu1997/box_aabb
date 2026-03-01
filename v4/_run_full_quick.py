"""Full quick experiment — all scenes, SBF-GCS + PRM, 5 seeds."""
import sys, logging, time
sys.path.insert(0, 'src')
sys.path.insert(0, 'experiments')

logging.basicConfig(level=logging.WARNING)
for name in ['forest', 'forest.hier_aabb_tree', 'forest.gcs', 'forest.builder',
             'baselines', 'baselines.iris_np_gcs', 'planner', 'planner.pipeline']:
    logging.getLogger(name).setLevel(logging.ERROR)

from runner import create_planner, load_scene_from_config
from scenes import load_all_marcucci_scenes
from paper_exp2_e2e_gcs import run_single_query, compute_path_length
import numpy as np

# Only test working planners
PLANNERS = [
    {"type": "SBF", "method": "gcs", "name": "SBF-GCS"},
    {"type": "PRM", "n_samples": 1000, "k_neighbors": 12, "name": "PRM"},
]

N_SEEDS = 5
results_all = []

scene_cfgs = load_all_marcucci_scenes(robot="iiwa14")
for scene_cfg in scene_cfgs:
    scene_name = scene_cfg["name"]
    robot, scene, query_pairs = load_scene_from_config(scene_cfg)
    print(f"\n{'='*50}")
    print(f"Scene: {scene_name} ({len(scene.get_obstacles())} obstacles)")
    
    for pcfg in PLANNERS:
        pname = pcfg["name"]
        times = []
        lengths = []
        successes = 0
        
        for seed in range(N_SEEDS):
            planner = create_planner(pcfg)
            planner.setup(robot, scene, {"seed": seed})
            qi = seed % len(query_pairs)
            q_s, q_g = query_pairs[qi]
            
            result, metrics = run_single_query(planner, q_s, q_g, timeout=30)
            times.append(metrics["wall_clock"])
            lengths.append(metrics["path_length"])
            if result.success:
                successes += 1
        
        sr = successes / N_SEEDS * 100
        t_med = np.median(times)
        l_med = np.median(lengths)
        print(f"  {pname:14s} | SR={sr:5.1f}% | t_med={t_med:.3f}s | path_med={l_med:.3f}")
        results_all.append({
            "scene": scene_name, "planner": pname,
            "success_rate": sr, "time_median": t_med, "path_median": l_med,
        })

print(f"\n{'='*70}")
print("SUMMARY TABLE (link_radii AABB inflation, no obstacle inflation)")
print(f"{'='*70}")
print(f"{'Scene':20s} | {'Planner':14s} | {'SR':>5s} | {'Time':>8s} | {'Path':>8s}")
print("-"*70)
for r in results_all:
    print(f"{r['scene']:20s} | {r['planner']:14s} | {r['success_rate']:5.1f}% | {r['time_median']:.3f}s   | {r['path_median']:.3f}")
print(f"{'='*70}")
