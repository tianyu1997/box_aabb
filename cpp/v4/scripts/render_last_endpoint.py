"""render_last_endpoint.py — 渲染 results/small_ep_viz 的最后一个 endpoint iAABB"""
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'python'))
import numpy as np
import plotly.graph_objects as go
from sbf4_viz.ep_iaabb_viz import load_ep_iaabb_comparison, EP_METHOD_STYLES, _ep_wireframe, _ep_mesh

json_path = sys.argv[1] if len(sys.argv) > 1 else 'results/small_ep_viz/ep_iaabb_comparison.json'
out_html  = sys.argv[2] if len(sys.argv) > 2 else os.path.join(os.path.dirname(json_path), 'last_endpoint_small.html')

data = load_ep_iaabb_comparison(json_path)
n_ep = data.n_endpoints
last_idx = n_ep - 1

print(f'n_endpoints={n_ep}, visualizing endpoint[{last_idx}]')
for i, iv in enumerate(data.box_intervals):
    print(f'  j{i}: [{iv[0]:.4f}, {iv[1]:.4f}]  w={iv[1]-iv[0]:.4f} rad')
print(f'Last endpoint [{last_idx}]:')
for src in data.sources:
    ep = src.endpoint_iaabbs[last_idx]
    size = ep.hi - ep.lo
    print(f'  {src.name:14s}: size={[round(float(x),4) for x in size]}  vol={ep.volume:.6e}')

fig = go.Figure()
pos = data.link_positions
fig.add_trace(go.Scatter3d(x=pos[:,0], y=pos[:,1], z=pos[:,2],
    mode='lines+markers', line=dict(color='rgb(40,40,40)', width=5),
    marker=dict(size=5, color='rgb(40,40,40)'), name='Robot Arm',
    legendgroup='arm', showlegend=True))

for src in data.sources:
    ep = src.endpoint_iaabbs[last_idx]
    style = EP_METHOD_STYLES.get(src.name, {'label': src.name, 'wire': 'gray', 'fill': 'rgba(128,128,128,0.1)'})
    group = f'ep_{src.name}'
    fig.add_trace(_ep_mesh(ep.lo, ep.hi, color_fill=style['fill'], name=style['label'], legendgroup=group, showlegend=True))
    fig.add_trace(_ep_wireframe(ep.lo, ep.hi, color=style['wire'], width=3, name=style['label'], legendgroup=group, showlegend=False))

if last_idx < len(pos):
    ep_c = pos[last_idx]
    fig.add_trace(go.Scatter3d(x=[ep_c[0]], y=[ep_c[1]], z=[ep_c[2]],
        mode='markers', marker=dict(size=8, color='black', symbol='cross'),
        name=f'EP[{last_idx}] center', showlegend=True))

biv = data.box_intervals
w_str = ' | '.join(f'j{i}:{iv[1]-iv[0]:.3f}r' for i, iv in enumerate(biv))
fig.update_layout(
    title=f'末端 Endpoint [{last_idx}] iAABB — 小区间 (frac=5%)<br><sup>{w_str}</sup>',
    legend=dict(title='Source (click to toggle)', itemsizing='constant'),
    scene=dict(xaxis_title='X (m)', yaxis_title='Y (m)', zaxis_title='Z (m)', aspectmode='data'),
    margin=dict(l=0, r=0, t=90, b=0),
)
fig.write_html(out_html)
print(f'Saved: {os.path.abspath(out_html)}')
