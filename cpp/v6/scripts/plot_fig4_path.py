#!/usr/bin/env python3
"""Fig 4: Query path visualization — joint-space trajectory profiles.

Shows the 5 benchmark query paths (IIWA14 combined scene) as joint
trajectories over waypoint index, plus a summary path-length comparison.
"""
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
from plot_common import *

setup_ieee_style()

# Query data from v5v6 report (v6, seed 0, ALL ON)
queries = [
    {'name': 'AS→TS', 'length': 3.099, 'time': 0.125, 'waypoints': 4},
    {'name': 'TS→CS', 'length': 5.116, 'time': 0.324, 'waypoints': 6},
    {'name': 'CS→LB', 'length': 5.169, 'time': 0.265, 'waypoints': 6},
    {'name': 'LB→RB', 'length': 4.223, 'time': 0.468, 'waypoints': 7},
    {'name': 'RB→AS', 'length': 1.908, 'time': 0.024, 'waypoints': 3},
]

# Named configurations (from marcucci_scenes.h)
configs = {
    'AS': [-1.57, 0.1,  0.0, -1.2, 0.0,  1.6, 0.0],   # Above Shelf
    'TS': [-0.76, 0.6,  0.0, -0.7, 0.0,  2.0, 0.0],    # Top Shelf
    'CS': [ 0.0,  0.9,  0.0, -0.5, 0.0,  2.1, 0.0],    # Center Shelf
    'LB': [-0.78, 0.44, 0.0, -1.65, 0.0, 2.5, 0.79],   # Left Bin
    'RB': [ 0.78, 0.44, 0.0, -1.65, 0.0, 2.5, -0.79],  # Right Bin
}

fig, axes = plt.subplots(1, 2, figsize=(DOUBLE_COL, 2.4),
                         gridspec_kw={'width_ratios': [2, 1]})

# ── Left: Path length & query time bar chart ──
ax = axes[0]
names = [q['name'] for q in queries]
lengths = [q['length'] for q in queries]
times = [q['time'] for q in queries]
x = np.arange(len(queries))
width = 0.35

bars1 = ax.bar(x - width/2, lengths, width, color=PAL[0],
               edgecolor='black', linewidth=0.5, label='Path Length (rad)', zorder=3)
ax.set_ylabel('Path Length (rad)')
ax.set_xlabel('Query Pair')
ax.set_xticks(x)
ax.set_xticklabels(names, fontsize=7)
ax.set_ylim(0, 6.5)

ax2 = ax.twinx()
bars2 = ax2.bar(x + width/2, times, width, color=PAL[1],
                edgecolor='black', linewidth=0.5, label='Query Time (s)', zorder=3)
ax2.set_ylabel('Query Time (s)', color=PAL[1])
ax2.tick_params(axis='y', labelcolor=PAL[1])
ax2.set_ylim(0, 0.6)

lines1, labels1 = ax.get_legend_handles_labels()
lines2, labels2 = ax2.get_legend_handles_labels()
ax.legend(lines1 + lines2, labels1 + labels2, loc='upper left', fontsize=6)

# Annotate
for i, (l, t) in enumerate(zip(lengths, times)):
    ax.text(x[i] - width/2, l + 0.1, f'{l:.2f}', ha='center', fontsize=5.5, color='darkblue')
    ax2.text(x[i] + width/2, t + 0.01, f'{t:.3f}s', ha='center', fontsize=5.5, color='darkorange')

ax.set_title('(a) Query Performance Summary', fontsize=8, fontweight='bold')

# ── Right: Joint configuration radar / start-goal heatmap ──
ax = axes[1]
joint_names = [f'J{i+1}' for i in range(7)]

# Plot start/goal configs as a heatmap-like comparison
query_names = ['AS→TS', 'CS→LB', 'LB→RB']  # Subset for clarity
sel_queries = [
    ('AS', 'TS'),
    ('CS', 'LB'),
    ('LB', 'RB'),
]

# Show joint deltas
deltas = []
labels = []
for start, goal in sel_queries:
    d = [abs(configs[goal][j] - configs[start][j]) for j in range(7)]
    deltas.append(d)
    labels.append(f'{start}→{goal}')

deltas = np.array(deltas)
im = ax.imshow(deltas.T, aspect='auto', cmap='YlOrRd', interpolation='nearest')
ax.set_xticks(range(len(labels)))
ax.set_xticklabels(labels, fontsize=6)
ax.set_yticks(range(7))
ax.set_yticklabels(joint_names, fontsize=6)
ax.set_title('(b) Joint Displacement', fontsize=8, fontweight='bold')

# Annotate cells
for i in range(len(labels)):
    for j in range(7):
        val = deltas[i][j]
        if val > 0.01:
            ax.text(i, j, f'{val:.2f}', ha='center', va='center',
                    fontsize=5, color='white' if val > 1.0 else 'black')

plt.colorbar(im, ax=ax, shrink=0.8, label='Δrad')

fig.tight_layout()
savefig(fig, 'fig4_query_paths')
print('Done: Fig 4')
