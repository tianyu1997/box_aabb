#!/usr/bin/env python3
"""Fig 5: Scalability — 3-subplot line chart.

Subplot 1: DOF (2, 7) → build time
Subplot 2: Obstacles (1, 2, 4) → build time + success rate
Subplot 3: Max boxes (50, 200, 500) → build time
"""
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
from plot_common import *

setup_ieee_style()

data = load_json('s5_scalability/results.json')

fig, axes = plt.subplots(1, 3, figsize=(DOUBLE_COL, 2.0))

# ── Subplot 1: DOF vs Build Time ──
ax = axes[0]
dof_data = data['time_vs_dof']
dofs = sorted(dof_data.keys(), key=int)
dof_vals = [int(d) for d in dofs]
times = [dof_data[d]['time_mean'] for d in dofs]
stds = [dof_data[d]['time_std'] for d in dofs]
ax.errorbar(dof_vals, times, yerr=stds, marker='o', capsize=3,
            color=PAL[0], linewidth=1.5, zorder=3)
ax.set_xlabel('DOF')
ax.set_ylabel('Build Time (s)')
ax.set_xticks(dof_vals)
ax.set_title('(a) DOF Scaling', fontsize=8, fontweight='bold')
ax.set_ylim(bottom=0)
# Annotate values
for d, t in zip(dof_vals, times):
    ax.annotate(f'{t:.2f}s', (d, t), textcoords='offset points',
                xytext=(5, 5), fontsize=6)

# ── Subplot 2: Obstacles vs Build Time + Success ──
ax = axes[1]
obs_data = data['success_vs_obstacles']
obs_keys = sorted(obs_data.keys(), key=int)
obs_vals = [int(o) for o in obs_keys]
obs_times = [obs_data[o]['time_mean'] for o in obs_keys]
obs_stds = [obs_data[o]['time_std'] for o in obs_keys]
obs_sr = [obs_data[o]['success_rate'] for o in obs_keys]

ax.errorbar(obs_vals, obs_times, yerr=obs_stds, marker='s', capsize=3,
            color=PAL[0], linewidth=1.5, label='Build Time', zorder=3)
ax.set_xlabel('Number of Obstacles')
ax.set_ylabel('Build Time (s)')
ax.set_xticks(obs_vals)
ax.set_ylim(bottom=0)
ax.set_title('(b) Obstacle Scaling', fontsize=8, fontweight='bold')

# Secondary Y-axis for success rate
ax2 = ax.twinx()
ax2.plot(obs_vals, obs_sr, marker='^', color=PAL[2], linewidth=1.5,
         linestyle='--', label='Success Rate', zorder=3)
ax2.set_ylabel('Success Rate (%)', color=PAL[2])
ax2.set_ylim(-5, 105)
ax2.tick_params(axis='y', labelcolor=PAL[2])

# Combined legend
lines1, labels1 = ax.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax.legend(lines1 + lines2, labels1 + labels2, loc='center right', fontsize=6)

# ── Subplot 3: Max Boxes vs Build Time ──
ax = axes[2]
box_data = data['success_vs_max_boxes']
box_keys = sorted(box_data.keys(), key=int)
box_vals = [int(b) for b in box_keys]
box_times = [box_data[b]['time_mean'] for b in box_keys]
box_stds = [box_data[b]['time_std'] for b in box_keys]

ax.errorbar(box_vals, box_times, yerr=box_stds, marker='D', capsize=3,
            color=PAL[0], linewidth=1.5, zorder=3)
ax.set_xlabel('Max Boxes Budget')
ax.set_ylabel('Build Time (s)')
ax.set_xticks(box_vals)
ax.set_title('(c) Budget Scaling', fontsize=8, fontweight='bold')
ax.set_ylim(bottom=0)
# Annotate
for b, t in zip(box_vals, box_times):
    ax.annotate(f'{t:.1f}s', (b, t), textcoords='offset points',
                xytext=(5, 5), fontsize=6)

fig.tight_layout()
savefig(fig, 'fig5_scalability')
print('Done: Fig 5')
