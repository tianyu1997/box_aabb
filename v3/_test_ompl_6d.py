"""Quick test: verify OMPL bridge runs in 6D for Panda."""
import sys, json, subprocess, time
from pathlib import Path

_ROOT = Path(__file__).resolve().parent
_SRC = _ROOT / "src"
sys.path.insert(0, str(_SRC))
sys.path.insert(0, str(_ROOT))

import numpy as np
from experiments.scenes import load_scenes

scene_cfg = load_scenes(["panda_10obs_moderate"])[0]
from experiments.runner import load_scene_from_config
robot, scene, query_pairs = load_scene_from_config(scene_cfg)
q_start, q_goal = query_pairs[0]

obs_list = []
for obs in scene.get_obstacles():
    obs_list.append({
        "min_point": obs.min_point.tolist(),
        "max_point": obs.max_point.tolist(),
        "name": obs.name,
    })

problem = {
    "q_start": q_start.tolist(),
    "q_goal": q_goal.tolist(),
    "obstacles": obs_list,
    "algorithms": ["RRTConnect"],
    "timeout": 30.0,
    "trials": 1,
    "seed": 42,
    "step_size": 0.5,
}

bridge_path = "/mnt/c/Users/TIAN/Documents/box_aabb/v2/examples/ompl_bridge.py"

print("Running OMPL bridge via WSL...")
t0 = time.perf_counter()
proc = subprocess.run(
    ["wsl", "-e", "bash", "-c", f"python3 {bridge_path}"],
    input=json.dumps(problem),
    capture_output=True, text=True,
    encoding='utf-8', errors='replace',
    timeout=60,
)
dt = time.perf_counter() - t0
print(f"  WSL time: {dt:.2f}s, exit code: {proc.returncode}")

if proc.stderr:
    print(f"  stderr: {proc.stderr[:500]}")

if proc.returncode != 0:
    print(f"  FAILED (exit code {proc.returncode})")
    print(f"  stdout: {proc.stdout[:500]}")
    sys.exit(1)

try:
    raw = json.loads(proc.stdout)
except json.JSONDecodeError as e:
    print(f"  JSON parse error: {e}")
    print(f"  stdout first 500 chars: {proc.stdout[:500]}")
    sys.exit(1)

data = raw.get("RRTConnect", {})
summary = data.get("summary", {})
trials = data.get("trials", [])

print(f"\n=== OMPL-RRTConnect Result ===")
print(f"  Success: {summary.get('n_success')}/{summary.get('n_trials')}")
if trials:
    t = trials[0]
    print(f"  Plan time: {t.get('plan_time_s', 0):.4f}s")
    print(f"  First solution time: {t.get('first_solution_time', 0):.4f}s")
    print(f"  Collision checks: {t.get('n_collision_checks', 0)}")
    print(f"  Path length: {t.get('path_length', 'N/A')}")
    print(f"  Raw path length: {t.get('raw_path_length', 'N/A')}")
    print(f"  N waypoints: {t.get('n_waypoints', 0)}")

# Check waypoint dimensions
wps = data.get("best_waypoints", [])
if wps:
    print(f"\n  Waypoint dims: {len(wps[0])} (should be 7)")
    wp0 = np.array(wps[0])
    wpN = np.array(wps[-1])
    print(f"  First wp: {np.array2string(wp0, precision=3)}")
    print(f"  Last  wp: {np.array2string(wpN, precision=3)}")
    print(f"  q_start:  {np.array2string(q_start, precision=3)}")
    print(f"  q_goal:   {np.array2string(q_goal, precision=3)}")
    
    # Check collision
    from forest.collision import CollisionChecker
    checker = CollisionChecker(robot=robot, scene=scene)
    n_coll = sum(1 for w in wps if checker.check_config_collision(np.array(w)))
    print(f"\n  Waypoints in collision: {n_coll}/{len(wps)}")

print("\nDone.")
