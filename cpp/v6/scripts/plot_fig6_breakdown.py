#!/usr/bin/env python3
"""Fig 6: Build time breakdown — grouped bar chart with per-phase timing.

Uses real data from exp6_build_timing JSON output (BuildTimingProfile).
Falls back to estimated data if JSON not available.
"""
import sys, os, json
sys.path.insert(0, os.path.dirname(__file__))
from plot_common import *

setup_ieee_style()

# Try loading real exp6 data
results_new = os.path.join(os.path.dirname(__file__), '..', 'experiments', 'results_new')
json_path = os.path.join(results_new, 'exp6_timing_10seed.json')
if not os.path.exists(json_path):
    json_path = '/tmp/exp6_quick.json'

if os.path.exists(json_path):
    with open(json_path) as f:
        d = json.load(f)
    # Compute median of each phase across seeds
    import statistics
    fields = ['lect_ms', 'grow_ms', 'coarsen1_ms', 'bridge_ms', 'coarsen2_ms', 'adjacency_ms']
    medians = {}
    for fld in fields:
        vals = [r[fld] for r in d['build_results']]
        medians[fld] = statistics.median(vals)
    # Convert to seconds
    phases = ['LECT', 'Grow\n(RRT)', 'Coarsen', 'Bridge', 'Adjacency']
    all_on_times = [
        medians['lect_ms'] / 1000,
        medians['grow_ms'] / 1000,
        (medians['coarsen1_ms'] + medians['coarsen2_ms']) / 1000,
        medians['bridge_ms'] / 1000,
        medians['adjacency_ms'] / 1000,
    ]
    n_seeds = d.get('n_seeds', '?')
    print(f'Loaded exp6 data: {json_path} ({n_seeds} seeds)')
    print(f'  Phase medians (ms): {medians}')
else:
    print('No exp6 JSON found, using estimated data')
    phases = ['LECT', 'Grow\n(RRT)', 'Coarsen', 'Bridge', 'Adjacency']
    all_on_times = [0.08, 1.45, 0.80, 0.10, 0.20]

x = np.arange(len(phases))
width = 0.5

fig, ax = plt.subplots(figsize=(SINGLE_COL, 2.4))

colors = [PAL[0], PAL[1], PAL[2], PAL[4], PAL[5]]
bars = ax.bar(x, all_on_times, width,
              color=colors, edgecolor='black', linewidth=0.5, zorder=3)

ax.set_xlabel('Build Phase')
ax.set_ylabel('Time (s)')
ax.set_xticks(x)
ax.set_xticklabels(phases, fontsize=6.5)
if max(all_on_times) / max(0.001, min(v for v in all_on_times if v > 0)) > 10:
    ax.set_yscale('log')
    ax.set_ylim(max(0.001, min(v for v in all_on_times if v > 0) * 0.5),
                max(all_on_times) * 3)

# Percentage labels
total = sum(all_on_times)
for i, val in enumerate(all_on_times):
    if val > 0 and total > 0:
        pct = val / total * 100
        ypos = val * 1.3 if ax.get_yscale() == 'log' else val + total * 0.02
        ax.text(x[i], ypos, f'{pct:.0f}%', ha='center', va='bottom', fontsize=5.5)

ax.set_title('Build Time Breakdown (IIWA14, combined scene)',
             fontsize=8, fontweight='bold')
fig.tight_layout()
savefig(fig, 'fig6_build_breakdown')
print('Done: Fig 6')
