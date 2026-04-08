#!/usr/bin/env python3
"""Analyze exp_freeze_sources output CSV."""
import csv
from collections import defaultdict

rows = list(csv.DictReader(open('exp_freeze_sources_output.csv', encoding='utf-16')))

stats = defaultdict(lambda: {
    'vol_d': [], 'vol_f': [], 't_d': [], 't_f': [],
    'ratio': [], 'fd': []
})

for r in rows:
    key = (r['source'], r['width'])
    if r['mode'] == 'direct':
        stats[key]['vol_d'].append(float(r['volume']))
        stats[key]['t_d'].append(float(r['time_ms']))
    else:
        stats[key]['vol_f'].append(float(r['volume']))
        stats[key]['t_f'].append(float(r['time_ms']))
        stats[key]['ratio'].append(float(r['ratio']))
        stats[key]['fd'].append(int(r['freeze_depth']))

def avg(lst):
    return sum(lst) / len(lst) if lst else 0

# Header
print("{:<12} {:>5} {:>5} {:>12} {:>12} {:>7} {:>8} {:>8} {:>8}".format(
    'Source', 'Width', 'AvgFD', 'Vol_direct', 'Vol_frozen',
    'Ratio', 'T_d(ms)', 'T_f(ms)', 'Speedup'))
print('-' * 87)

for src in ['IFK', 'CritSample', 'Analytical', 'GCPC']:
    for w in ['0.050', '0.100', '0.150', '0.200', '0.300', '0.500', '0.700', '1.000']:
        key = (src, w)
        if key not in stats:
            continue
        s = stats[key]
        if not s['ratio']:
            continue
        r = avg(s['ratio'])
        fd_val = avg(s['fd'])
        vd = avg(s['vol_d'])
        vf = avg(s['vol_f'])
        td = avg(s['t_d'])
        tf = avg(s['t_f'])
        spd = td / tf if tf > 1e-6 else 0
        print("{:<12} {:>5} {:>5.1f} {:>12.6e} {:>12.6e} {:>7.3f} {:>8.4f} {:>8.4f} {:>7.2f}x".format(
            src, w, fd_val, vd, vf, r, td, tf, spd))
    print()

# Overall summary per source
print("=" * 87)
print("OVERALL SUMMARY (averaged across all widths)")
print("=" * 87)
print("{:<12} {:>10} {:>10} {:>14} {:>14}".format(
    'Source', 'Avg Ratio', 'Med Ratio', 'AvgTime_d(ms)', 'AvgTime_f(ms)'))
print('-' * 64)

for src in ['IFK', 'CritSample', 'Analytical', 'GCPC']:
    all_ratios = []
    all_td = []
    all_tf = []
    for w in ['0.050', '0.100', '0.150', '0.200', '0.300', '0.500', '0.700', '1.000']:
        key = (src, w)
        if key in stats:
            all_ratios.extend(stats[key]['ratio'])
            all_td.extend(stats[key]['t_d'])
            all_tf.extend(stats[key]['t_f'])
    if all_ratios:
        med = sorted(all_ratios)[len(all_ratios) // 2]
        print("{:<12} {:>10.3f} {:>10.3f} {:>14.4f} {:>14.4f}".format(
            src, avg(all_ratios), med, avg(all_td), avg(all_tf)))

# Freeze depth distribution
print()
print("FREEZE DEPTH DISTRIBUTION:")
fd_counts = defaultdict(int)
for key, s in stats.items():
    for fd in s['fd']:
        fd_counts[fd] += 1
for fd in sorted(fd_counts.keys()):
    print("  fd={}: {} samples".format(fd, fd_counts[fd]))
