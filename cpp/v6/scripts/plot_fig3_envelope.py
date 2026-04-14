#!/usr/bin/env python3
"""Fig 3: Envelope tightness comparison — grouped bar chart.

X-axis: Endpoint source (IFK, CritSample, Analytical, GCPC)
Groups: Envelope type (LinkIAABB, LinkIAABB_Grid, Hull16_Grid)
Y-axis left: Volume (m³)
Robot: Panda 7-DOF
"""
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
from plot_common import *

setup_ieee_style()

data = load_json('s1_envelope_tightness/results.json')
rows = [r for r in data['rows'] if r['robot'] == 'panda']

# Organize data
endpoints = ['IFK', 'CritSample', 'Analytical', 'GCPC']
envelopes = ['LinkIAABB', 'LinkIAABB_Grid', 'Hull16_Grid']
envelope_labels = ['LinkIAABB', 'LinkIAABB+Grid', 'Hull16+Grid']

vol = {ep: {} for ep in endpoints}
std = {ep: {} for ep in endpoints}
time_us = {ep: {} for ep in endpoints}

for r in rows:
    ep, env = r['endpoint'], r['envelope']
    if ep in vol and env in envelopes:
        vol[ep][env] = r['volume_mean']
        std[ep][env] = r['volume_std']
        time_us[ep][env] = r['time_us_mean']

x = np.arange(len(endpoints))
n_env = len(envelopes)
width = 0.22

fig, ax1 = plt.subplots(figsize=(SINGLE_COL, 2.2))

for i, env in enumerate(envelopes):
    vals = [vol[ep].get(env, 0) for ep in endpoints]
    errs = [std[ep].get(env, 0) for ep in endpoints]
    bars = ax1.bar(x + (i - 1) * width, vals, width,
                   yerr=errs, capsize=2,
                   color=PAL[i], hatch=HATCHES[i],
                   edgecolor='black', linewidth=0.5,
                   label=envelope_labels[i], zorder=3)

ax1.set_xlabel('Endpoint Source')
ax1.set_ylabel('Envelope Volume (m³)')
ax1.set_xticks(x)
ax1.set_xticklabels(endpoints)
ax1.legend(loc='upper right', framealpha=0.9)
ax1.set_ylim(bottom=0)

# Add timing annotations on top
for i, env in enumerate(envelopes):
    for j, ep in enumerate(endpoints):
        t = time_us[ep].get(env, 0)
        v = vol[ep].get(env, 0)
        s = std[ep].get(env, 0)
        if t > 0:
            label = f'{t:.0f}' if t >= 10 else f'{t:.1f}'
            ax1.text(j + (i - 1) * width, v + s + 0.01,
                     f'{label}μs', ha='center', va='bottom',
                     fontsize=5, rotation=45)

ax1.set_title('Panda 7-DOF Envelope Tightness', fontsize=9, fontweight='bold')
fig.tight_layout()
savefig(fig, 'fig3_envelope_tightness')
print('Done: Fig 3')
