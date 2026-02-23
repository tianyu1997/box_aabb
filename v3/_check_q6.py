"""Quick check: is joint 7 (q6) constant in SBF paths?"""
import json, numpy as np

with open('experiments/output/raw/panda_10obs_comparison.json') as f:
    data = json.load(f)

results = data['results']
print("=== SBF-Dijkstra paths: joint 7 (q6) analysis ===")
for e in results:
    if e.get('planner') == 'SBF-Dijkstra' and e.get('path'):
        path = np.array(e['path'])
        seed = e['seed']
        print(f"seed {seed}: shape={path.shape}")
        print(f"  q6: min={path[:,6].min():.4f} max={path[:,6].max():.4f} std={path[:,6].std():.6f}")
        print(f"  start={path[0,:]}")
        print(f"  goal ={path[-1,:]}")
        # Distance contribution from q6 vs total
        diffs = np.diff(path, axis=0)
        dist_total = np.sum(np.sqrt(np.sum(diffs**2, axis=1)))
        dist_q6 = np.sum(np.abs(diffs[:, 6]))
        print(f"  total L2 dist={dist_total:.4f}, q6 abs dist={dist_q6:.6f}")
        print()

print("\n=== RRT-Connect paths: joint 7 (q6) analysis ===")
for e in results:
    if e.get('planner') == 'RRT-Connect' and e.get('path'):
        path = np.array(e['path'])
        seed = e['seed']
        diffs = np.diff(path, axis=0)
        dist_total = np.sum(np.sqrt(np.sum(diffs**2, axis=1)))
        dist_q6 = np.sum(np.abs(diffs[:, 6]))
        print(f"seed {seed}: shape={path.shape}, q6_std={path[:,6].std():.4f}, total_dist={dist_total:.4f}, q6_dist={dist_q6:.4f}")

print("\n=== BIT* paths: joint 7 (q6) analysis ===")
for e in results:
    if e.get('planner') == 'BIT*' and e.get('path'):
        path = np.array(e['path'])
        seed = e['seed']
        diffs = np.diff(path, axis=0)
        dist_total = np.sum(np.sqrt(np.sum(diffs**2, axis=1)))
        dist_q6 = np.sum(np.abs(diffs[:, 6]))
        print(f"seed {seed}: shape={path.shape}, q6_std={path[:,6].std():.4f}, total_dist={dist_total:.4f}, q6_dist={dist_q6:.4f}")

# Check start/goal for seed 0
print("\n=== Start/Goal configs (seed 0) ===")
print(f"metadata: {data.get('metadata', {}).get('scene', 'N/A')}")
for e in results:
    if e.get('seed') == 0:
        path = np.array(e['path']) if e.get('path') else None
        print(f"  {e['planner']}: start={path[0] if path is not None else 'N/A'}, goal={path[-1] if path is not None else 'N/A'}")
