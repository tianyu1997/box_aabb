#!/usr/bin/env python3
"""Fig 7: Optimisation ablation — grouped bar chart (v2).

Shows build time, query time, and path length for all ablation configs.
Uses real data from exp5 JSON output.
"""
import sys, os, json
sys.path.insert(0, os.path.dirname(__file__))
from plot_common import *

setup_ieee_style()

# Load exp5 data
json_path = os.path.join(os.path.dirname(__file__), '..', 'doc', 'results_ablation_v2.json')
if not os.path.exists(json_path):
    print(f'ERROR: {json_path} not found. Run exp5_ablation first.')
    sys.exit(1)

with open(json_path) as f:
    d = json.load(f)

results = d['results']
configs = [r['name'] for r in results]
build_time = [r['build_median'] for r in results]
query_time = [r['query_median'] for r in results]
path_len   = [r['len_median'] for r in results]
boxes      = [r['boxes'] for r in results]
sr         = [r['sr'] for r in results]
n_seeds = d.get('n_seeds', '?')
print(f'Loaded exp5 data: {json_path} ({n_seeds} seeds, {len(configs)} configs)')

# Group labels and colours
# Group: baseline=0, split=1, ablation=2
group_idx = [0, 1, 1, 2, 2, 2, 2]  # Full, RR, WF, noCM, noPromo, noCoarsen, noPathOpt
group_colors_build = [PAL[0], PAL[2], PAL[2], PAL[3], PAL[3], PAL[3], PAL[3]]
group_colors_query = [PAL[1], PAL[4], PAL[4], PAL[5], PAL[5], PAL[5], PAL[5]]

x = np.arange(len(configs))
width = 0.28

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(DOUBLE_COL, 2.6),
                                gridspec_kw={'width_ratios': [3, 2]})

# ── Left panel: Build & Query time ──
bars_b = ax1.bar(x - width/2, build_time, width,
                 color=[group_colors_build[i] for i in range(len(configs))],
                 edgecolor='black', linewidth=0.5, label='Build', zorder=3)
bars_q = ax1.bar(x + width/2, query_time, width,
                 color=[group_colors_query[i] for i in range(len(configs))],
                 edgecolor='black', linewidth=0.5, label='Query', zorder=3)

# Highlight baseline
bars_b[0].set_edgecolor('red')
bars_b[0].set_linewidth(1.5)
bars_q[0].set_edgecolor('red')
bars_q[0].set_linewidth(1.5)

ax1.set_ylabel('Time (s)')
ax1.set_xticks(x)
ax1.set_xticklabels(configs, rotation=30, ha='right', fontsize=6)
ax1.legend(loc='upper right', framealpha=0.9, fontsize=6)
ax1.set_ylim(bottom=0)

# Annotate build time values
for i, bt in enumerate(build_time):
    ax1.text(x[i] - width/2, bt + 0.05, f'{bt:.2f}',
             ha='center', va='bottom', fontsize=5, color='darkblue')

ax1.set_title('Build / Query Time', fontsize=7, fontweight='bold')

# ── Right panel: Path length & Box count ──
ax2b = ax2.twinx()
bars_l = ax2.bar(x - width/2, path_len, width,
                 color=PAL[0], edgecolor='black', linewidth=0.5,
                 label='Path Len', zorder=3, alpha=0.8)
bars_x = ax2b.bar(x + width/2, boxes, width,
                  color=PAL[6] if len(PAL) > 6 else 'gray',
                  edgecolor='black', linewidth=0.5,
                  label='Boxes', zorder=3, alpha=0.7)

ax2.set_ylabel('Path Length (rad)')
ax2b.set_ylabel('Box Count')
ax2.set_xticks(x)
ax2.set_xticklabels(configs, rotation=30, ha='right', fontsize=6)
ax2.set_ylim(bottom=0)
ax2b.set_ylim(bottom=0)

# Combined legend
lines1, labels1 = ax2.get_legend_handles_labels()
lines2, labels2 = ax2b.get_legend_handles_labels()
ax2.legend(lines1 + lines2, labels1 + labels2, loc='upper right',
           framealpha=0.9, fontsize=6)

ax2.set_title('Path Length / Box Count', fontsize=7, fontweight='bold')

fig.suptitle(f'Ablation Study (IIWA14, combined, {n_seeds} seeds)',
             fontsize=8, fontweight='bold')
fig.tight_layout()
savefig(fig, 'fig7_ablation')
print('Done: Fig 7')
