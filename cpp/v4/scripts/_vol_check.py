import csv, statistics
from collections import defaultdict

data = []
with open('v4_h6_results.csv', encoding='utf-8-sig') as f:
    for row in csv.DictReader(f):
        data.append(row)

by_sw = defaultdict(list)
by_st = defaultdict(list)
for r in data:
    w = float(r['width'])
    by_sw[(r['source'], w)].append(float(r['volume']))
    by_st[(r['source'], w)].append(r)

widths = sorted(set(float(r['width']) for r in data))
print(f'{"w":>5} | {"GCPC_vol":>10} {"Anal_vol":>10} {"diff%":>8} | {"GCPC_ms":>8} {"Anal_ms":>8} {"ratio":>6}')
print('-'*80)
for w in widths:
    g_vol = statistics.mean(by_sw[('GCPC', w)])
    a_vol = statistics.mean(by_sw[('Analytical', w)])
    diff = (g_vol - a_vol) / a_vol * 100 if a_vol > 0 else 0
    g_ms = statistics.mean([float(r['time_ms']) for r in by_st[('GCPC', w)]])
    a_ms = statistics.mean([float(r['time_ms']) for r in by_st[('Analytical', w)]])
    ratio = g_ms / a_ms if a_ms > 0 else 0
    print(f'{w:5.3f} | {g_vol:10.4f} {a_vol:10.4f} {diff:8.4f}% | {g_ms:8.2f} {a_ms:8.2f} {ratio:6.3f}')

all_g_vol = [float(r['volume']) for r in data if r['source'] == 'GCPC']
all_a_vol = [float(r['volume']) for r in data if r['source'] == 'Analytical']
print(f'\nOverall mean vol: GCPC={statistics.mean(all_g_vol):.6f}, Anal={statistics.mean(all_a_vol):.6f}')

# Per-trial volume comparison (same trial, same width)
max_diff = 0
for w in widths:
    g_trials = sorted(by_st[('GCPC', w)], key=lambda r: int(r['trial']))
    a_trials = sorted(by_st[('Analytical', w)], key=lambda r: int(r['trial']))
    for gt, at in zip(g_trials, a_trials):
        gv = float(gt['volume'])
        av = float(at['volume'])
        if av > 0:
            d = abs(gv - av) / av * 100
            if d > max_diff:
                max_diff = d

print(f'Max per-trial vol diff: {max_diff:.6f}%')
