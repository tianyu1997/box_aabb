import csv
from collections import defaultdict

rows = []
with open('v4_h6f_results.csv', encoding='utf-8-sig') as f:
    for r in csv.DictReader(f):
        rows.append(r)

# Group by trial
trials = defaultdict(dict)
for r in rows:
    key = int(r['trial'])
    src = r['source']
    trials[key][src] = r

widths_data = defaultdict(lambda: {'gcpc_t': [], 'anal_t': [], 'gcpc_v': [], 'anal_v': []})

for trial_id, sources in trials.items():
    if 'GCPC' not in sources or 'Analytical' not in sources:
        continue
    w = float(sources['GCPC']['width'])
    w_key = round(w, 3)
    widths_data[w_key]['gcpc_t'].append(float(sources['GCPC']['time_ms']))
    widths_data[w_key]['anal_t'].append(float(sources['Analytical']['time_ms']))
    widths_data[w_key]['gcpc_v'].append(float(sources['GCPC']['volume']))
    widths_data[w_key]['anal_v'].append(float(sources['Analytical']['volume']))

print(f"{'w':>6} | {'GCPC_vol':>10} {'Anal_vol':>10} {'vol_diff%':>9} | {'GCPC_ms':>8} {'Anal_ms':>8} {'ratio':>6}")
for w in sorted(widths_data.keys()):
    d = widths_data[w]
    n = len(d['gcpc_t'])
    gv = sum(d['gcpc_v']) / n
    av = sum(d['anal_v']) / n
    gt = sum(d['gcpc_t']) / n
    at = sum(d['anal_t']) / n
    vd = (gv - av) / av * 100 if av > 0 else 0
    r = gt / at if at > 0 else 0
    mark = ' ***' if r > 1.0 else ''
    print(f'{w:6.3f} | {gv:10.4f} {av:10.4f} {vd:9.4f}% | {gt:8.2f} {at:8.2f} {r:6.3f}{mark}')

# Per-trial volume check: count cases where GCPC is looser
n_looser = 0
max_looser = 0
n_tighter = 0
n_total = 0
for trial_id, sources in trials.items():
    if 'GCPC' not in sources or 'Analytical' not in sources:
        continue
    gv = float(sources['GCPC']['volume'])
    av = float(sources['Analytical']['volume'])
    n_total += 1
    if gv > av * 1.0001:  # GCPC looser by > 0.01%
        n_looser += 1
        diff_pct = (gv - av) / av * 100
        if diff_pct > max_looser:
            max_looser = diff_pct
    elif av > gv * 1.0001:  # GCPC tighter by > 0.01%
        n_tighter += 1

print(f"\nVolume check: {n_total} trials")
print(f"  GCPC looser (>0.01%): {n_looser}/{n_total} ({100*n_looser/n_total:.1f}%), max={max_looser:.4f}%")
print(f"  GCPC tighter (>0.01%): {n_tighter}/{n_total} ({100*n_tighter/n_total:.1f}%)")
