import csv, statistics
from collections import defaultdict

data = []
with open('v4_h6_results.csv', encoding='utf-8-sig') as f:
    for row in csv.DictReader(f):
        data.append(row)

# Group by (trial, width) and compare GCPC vs Analytical
by_tw = defaultdict(dict)
for r in data:
    key = (int(r['trial']), float(r['width']))
    by_tw[key][r['source']] = float(r['volume'])

max_gcpc_looser = 0  # GCPC volume > Analytical (bad)
max_gcpc_tighter = 0  # GCPC volume < Analytical (good)
n_gcpc_looser = 0
n_gcpc_tighter = 0
n_equal = 0
worst_trial = None

for (trial, width), vols in by_tw.items():
    if 'GCPC' not in vols or 'Analytical' not in vols:
        continue
    g = vols['GCPC']
    a = vols['Analytical']
    if a == 0: continue
    diff_pct = (g - a) / a * 100
    if diff_pct > 1e-6:
        n_gcpc_looser += 1
        if diff_pct > max_gcpc_looser:
            max_gcpc_looser = diff_pct
            worst_trial = (trial, width, g, a, diff_pct)
    elif diff_pct < -1e-6:
        n_gcpc_tighter += 1
        if abs(diff_pct) > max_gcpc_tighter:
            max_gcpc_tighter = abs(diff_pct)
    else:
        n_equal += 1

total = n_gcpc_looser + n_gcpc_tighter + n_equal
print(f'=== GCPC vs Analytical Volume Analysis ===')
print(f'Total trial-width pairs: {total}')
print(f'GCPC tighter (smaller vol): {n_gcpc_tighter} ({n_gcpc_tighter/total*100:.1f}%)')
print(f'GCPC looser  (larger vol):  {n_gcpc_looser} ({n_gcpc_looser/total*100:.1f}%)')
print(f'Equal:                      {n_equal} ({n_equal/total*100:.1f}%)')
print(f'Max GCPC tighter: {max_gcpc_tighter:.4f}%')
print(f'Max GCPC looser:  {max_gcpc_looser:.4f}%')
if worst_trial:
    print(f'Worst looser trial: trial={worst_trial[0]}, w={worst_trial[1]:.3f}, GCPC={worst_trial[2]:.6f}, Anal={worst_trial[3]:.6f}, diff={worst_trial[4]:.4f}%')
