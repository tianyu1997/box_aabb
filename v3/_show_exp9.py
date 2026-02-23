"""Read experiment 9 results and print summary table."""
import json
import numpy as np
from collections import defaultdict

data = json.load(open('experiments/output/raw/exp9_ffb_min_edge_sweep.json'))
groups = defaultdict(list)
for t in data['results']:
    groups[(t['scene'], t['planner'])].append(t)

header = f"{'Scene':25s}  {'Planner':15s}  {'ok':>6s}  {'cost':>8s}  {'grow_ms':>8s}  {'boxes':>6s}"
print(header)
print('-' * len(header))

prev_scene = None
for (scene, planner), trials in sorted(groups.items()):
    if scene != prev_scene and prev_scene is not None:
        print()
    prev_scene = scene

    successes = sum(1 for t in trials if t.get('success'))
    costs = [t['cost'] for t in trials if t.get('success')]
    grow_times = [t.get('grow_ms', 0) for t in trials]
    boxes = [t.get('n_boxes', 0) for t in trials]
    avg_cost = np.mean(costs) if costs else float('nan')
    avg_grow = np.mean(grow_times)
    avg_boxes = np.mean(boxes)
    print(f"{scene:25s}  {planner:15s}  {successes:>2}/{len(trials):<3}  {avg_cost:7.3f}  {avg_grow:7.0f}ms  {avg_boxes:5.0f}")
