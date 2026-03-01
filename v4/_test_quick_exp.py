"""Quick pipeline test — 1 scene, SBF only, 1 seed, 1 trial."""
import sys, logging, time
sys.path.insert(0, 'src')
sys.path.insert(0, 'experiments')

# Suppress verbose output
logging.basicConfig(level=logging.WARNING)
for name in ['forest', 'forest.hier_aabb_tree', 'forest.gcs', 'forest.builder',
             'baselines', 'baselines.iris_np_gcs', 'planner']:
    logging.getLogger(name).setLevel(logging.ERROR)

from runner import create_planner, load_scene_from_config
from scenes import load_marcucci_scene
from paper_exp2_e2e_gcs import run_single_query, run_amortized_queries, compute_path_length

# Load scene
scene_cfg = load_marcucci_scene('shelves', robot='iiwa14')
robot, scene, query_pairs = load_scene_from_config(scene_cfg)
print(f"Scene: {scene_cfg['name']}, {len(scene.get_obstacles())} obs, {len(query_pairs)} queries")
print(f"Robot: {robot.name}, link_radii={robot.link_radii}")

# === SBF-GCS ===
print("\n=== SBF-GCS ===")
pcfg = {"type": "SBF", "method": "gcs", "name": "SBF-GCS"}
planner = create_planner(pcfg)
planner.setup(robot, scene, {"seed": 0})

q_s, q_g = query_pairs[0]
result, metrics = run_single_query(planner, q_s, q_g, timeout=30)
print(f"First query: success={result.success}, wall={metrics['wall_clock']:.3f}s, "
      f"path_len={metrics['path_length']:.3f}")

# Amortized
print("Running amortized queries (K=5)...")
amor_list, amor_summary = run_amortized_queries(planner, query_pairs, K=5, timeout=30)
print(f"Amortized: success_rate={amor_summary['success_rate']:.0%}, "
      f"amortized_time={amor_summary['amortized_time']:.3f}s")

# === PRM ===
print("\n=== PRM ===")
pcfg2 = {"type": "PRM", "n_samples": 500, "k_neighbors": 10, "name": "PRM"}
planner2 = create_planner(pcfg2)
planner2.setup(robot, scene, {"seed": 0})

result2, metrics2 = run_single_query(planner2, q_s, q_g, timeout=30)
print(f"First query: success={result2.success}, wall={metrics2['wall_clock']:.3f}s, "
      f"path_len={metrics2['path_length']:.3f}")

# === Summary ===
print("\n" + "="*60)
print("Method         | Success | Time    | Path Length")
print("-"*60)
print(f"SBF-GCS        | {result.success!s:7s} | {metrics['wall_clock']:.3f}s  | {metrics['path_length']:.3f}")
print(f"PRM            | {result2.success!s:7s} | {metrics2['wall_clock']:.3f}s  | {metrics2['path_length']:.3f}")
print(f"SBF-GCS (amor) | {amor_summary['success_rate']:.0%}    | {amor_summary['amortized_time']:.3f}s  | -")
print("="*60)
print("Done!")
