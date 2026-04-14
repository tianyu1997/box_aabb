#!/usr/bin/env python3
"""Fig 7: Optimisation ablation — grouped bar chart.

Shows build time and query time for all ablation configs.
Uses real data from exp5 JSON output when available.
"""
import sys, os, json
sys.path.insert(0, os.path.dirname(__file__))
from plot_common import *

setup_ieee_style()

# Try loading real exp5 data
results_new = os.path.join(os.path.dirname(__file__), '..', 'experiments', 'results_new')
json_path = os.path.join(results_new, 'exp5_ablation_10seed.json')
if not os.path.exists(json_path):
    json_path = '/tmp/exp5_quick.json'

if os.path.exists(json_path):
    with open(json_path) as f:
        d = json.load(f)
    configs = [r['name'] for r in d['results']]
    build_time = [r['build_median'] for r in d['results']]
    query_time = [r['query_median'] for r in d['results']]
    build_q25  = [r['build_q25'] for r in d['results']]
    build_q75  = [r['build_q75'] for r in d['results']]
    query_q25  = [r['query_q25'] for r in d['results']]
    query_q75  = [r['query_q75'] for r in d['results']]
    p4_on = [r['P4'] for r in d['results']]
    n_seeds = d.get('n_seeds', '?')
    print(f'Loaded exp5 data: {json_path} ({n_seeds} seeds, {len(configs)} configs)')
else:
    print('No exp5 JSON found, using hardcoded data')
    configs = ['Baseline', '+P0', '+P2', '+P4', '+P0+P2', '+P2+P4', 'ALL ON']
    build_time = [14.73, 14.84, 11.69, 2.72, 11.92, 2.76, 2.78]
    query_time = [0.913, 0.931, 0.926, 0.111, 0.932, 0.117, 0.114]
    build_q25 = build_q75 = query_q25 = query_q75 = None
    p4_on = [False, False, False, True, False, True, True]

# Shorten config names for display
short_names = []
for c in configs:
    c = c.replace('(all OFF)', '').replace('(multi-RRT)', '')
    c = c.replace('(parallel brg)', '').replace('(connect-stop)', '')
    c = c.replace('ALL ON (P0+P2+P4)', 'ALL ON')
    c = c.replace('ALL ON - Z4', 'ALL−Z4')
    c = c.replace('ALL ON / RoundRobin', 'ALL/RR')
    c = c.replace('ALL ON / WidestFirst', 'ALL/WF')
    c = c.replace('ALL ON / BT-v2', 'ALL/BT2')
    short_names.append(c.strip())

x = np.arange(len(configs))
width = 0.35

fig, ax1 = plt.subplots(figsize=(DOUBLE_COL, 2.8))

# Error bars if available
build_err = None
query_err = None
if build_q25 and build_q75:
    build_err = [[bt - bq25 for bt, bq25 in zip(build_time, build_q25)],
                 [bq75 - bt for bt, bq75 in zip(build_time, build_q75)]]
if query_q25 and query_q75:
    query_err = [[qt - qq25 for qt, qq25 in zip(query_time, query_q25)],
                 [qq75 - qt for qt, qq75 in zip(query_time, query_q75)]]

bars1 = ax1.bar(x - width/2, build_time, width,
                yerr=build_err, capsize=2,
                color=PAL[0], edgecolor='black', linewidth=0.5,
                label='Build Time', zorder=3)
bars2 = ax1.bar(x + width/2, query_time, width,
                yerr=query_err, capsize=2,
                color=PAL[1], edgecolor='black', linewidth=0.5,
                label='Query Time', zorder=3)

ax1.set_xlabel('Optimisation Configuration')
ax1.set_ylabel('Time (s)')
ax1.set_xticks(x)
ax1.set_xticklabels(short_names, rotation=35, ha='right', fontsize=6)
ax1.legend(loc='upper right', framealpha=0.9)
ax1.set_ylim(bottom=0)

# Highlight P4-ON configs
for i, p4 in enumerate(p4_on):
    if p4:
        bars1[i].set_edgecolor('red')
        bars1[i].set_linewidth(1.5)
        bars2[i].set_edgecolor('red')
        bars2[i].set_linewidth(1.5)

# Speedup annotations
baseline_b, baseline_q = build_time[0], query_time[0]
for i in range(1, len(configs)):
    speedup_b = baseline_b / max(build_time[i], 0.001)
    if speedup_b > 1.3:
        ax1.text(x[i] - width/2, build_time[i] + max(build_time) * 0.02,
                 f'{speedup_b:.1f}×', ha='center', va='bottom',
                 fontsize=5, fontweight='bold', color='darkblue')

ax1.set_title('Ablation Study (IIWA14, combined scene)',
              fontsize=8, fontweight='bold')
fig.tight_layout()
savefig(fig, 'fig7_ablation')
print('Done: Fig 7')
