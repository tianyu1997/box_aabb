import json, numpy as np
from collections import defaultdict

with open('experiments/output/raw/exp10_sampling_ratio_sweep.json') as f:
    data = json.load(f)
trials = data.get('trials', data.get('results', []))
print(f"Total trials: {len(trials)}")

groups = defaultdict(lambda: {'ok':0,'n':0,'costs':[],'grows':[],'boxes':[]})
for t in trials:
    key = (t['scene'], t['planner'])
    g = groups[key]
    g['n'] += 1
    if t.get('success'):
        g['ok'] += 1
        g['costs'].append(t.get('cost', float('inf')))
    g['grows'].append(t.get('grow_ms', 0))
    g['boxes'].append(t.get('n_boxes', 0))

for (sc, pl), g in sorted(groups.items()):
    ac = np.mean(g['costs']) if g['costs'] else float('nan')
    ag = np.mean(g['grows']) if g['grows'] else float('nan')
    ab = np.mean(g['boxes']) if g['boxes'] else 0
    print(f"  {sc:25s}  {pl:20s}  ok={g['ok']:2d}/{g['n']}  "
          f"cost={ac:7.3f}  grow={ag:6.0f}ms  boxes={ab:.0f}")
