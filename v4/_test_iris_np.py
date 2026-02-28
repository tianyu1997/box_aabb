"""快速测试 IRIS-NP plant 构建和 region 生成"""
import sys, logging
from pathlib import Path
logging.basicConfig(level=logging.INFO)
_ROOT = Path(__file__).resolve().parent
sys.path.insert(0, str(_ROOT / 'src'))
sys.path.insert(0, str(_ROOT / 'experiments'))

from experiments.runner import load_scene_from_config
from experiments.scenes import load_marcucci_scene
from baselines.iris_np_gcs import IRISNPGCSPlanner

scene_cfg = load_marcucci_scene("shelves", robot="iiwa14")
robot, scene, query_pairs = load_scene_from_config(scene_cfg)
q_start, q_goal = query_pairs[0]

planner = IRISNPGCSPlanner(n_iris_seeds=3, max_iterations=3)
planner.setup(robot, scene, {"seed": 0, "max_time": 30})
result = planner.plan(q_start, q_goal, timeout=30)

print(f"\n=== RESULT ===")
print(f"Regions: {len(planner._regions)}")
print(f"Success: {result.success}")
