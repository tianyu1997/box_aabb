"""Per-trial paired GCPC/Analytical ratio analysis — removes system load variance"""
import csv
from collections import defaultdict
import math

rows = list(csv.DictReader(open('v4_h6g_results.csv', 'r', encoding='utf-8-sig')))
trials = defaultdict(dict)
for r in rows:
    trials[int(r['trial'])][r['source']] = r

# Per-trial ratio
width_ratios = defaultdict(list)
for tid, src in trials.items():
    if 'GCPC' not in src or 'Analytical' not in src:
        continue
    w = round(float(src['GCPC']['width']), 3)
    gt = float(src['GCPC']['time_ms'])
    at = float(src['Analytical']['time_ms'])
    if at > 0:
        width_ratios[w].append(gt / at)

print(f"{'w':>6} | {'mean_ratio':>10} {'median':>7} {'std':>6} | {'n<1':>4}/{' n':>3} | {'p5':>5} {'p95':>5}")
for w in sorted(width_ratios):
    ratios = width_ratios[w]
    n = len(ratios)
    mean = sum(ratios) / n
    ratios_sorted = sorted(ratios)
    median = ratios_sorted[n // 2]
    std = math.sqrt(sum((r - mean)**2 for r in ratios) / n)
    n_faster = sum(1 for r in ratios if r < 1.0)
    p5 = ratios_sorted[max(0, int(0.05 * n))]
    p95 = ratios_sorted[min(n-1, int(0.95 * n))]
    mark = ' ***' if mean > 1.0 else ''
    print(f"{w:6.3f} | {mean:10.4f} {median:7.4f} {std:6.4f} | {n_faster:4d}/{n:3d} | {p5:5.3f} {p95:5.3f}{mark}")

# Overall paired geometric mean ratio
all_ratios = []
for w in sorted(width_ratios):
    all_ratios.extend(width_ratios[w])
gmean = math.exp(sum(math.log(r) for r in all_ratios) / len(all_ratios))
print(f"\nOverall geometric mean ratio: {gmean:.4f}")
print(f"  GCPC faster in {sum(1 for r in all_ratios if r < 1)} / {len(all_ratios)} trials")
