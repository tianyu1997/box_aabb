"""Scene (obstacle) visualisation — Plotly 3D.

迁移自 v3 sbf_viz/scene_viz.py（API 无命名变更）
"""

import numpy as np
import plotly.graph_objects as go
from typing import List, Optional
from .load_data import SceneData, Obstacle

# Obstacle colour palette
_OBS_COLORS = [
    "rgba(214,39,40,0.25)",   # red
    "rgba(255,187,120,0.25)", # peach
    "rgba(152,78,163,0.25)",  # purple
    "rgba(255,127,0,0.25)",   # orange
]
_OBS_WIRE_COLORS = [
    "rgb(214,39,40)",
    "rgb(255,187,120)",
    "rgb(152,78,163)",
    "rgb(255,127,0)",
]


def _obs_mesh(obs: Obstacle, color: str, name: str,
              showlegend: bool = True) -> go.Mesh3d:
    lo = obs.center - obs.half_sizes
    hi = obs.center + obs.half_sizes
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    verts_x = [x0, x0, x1, x1, x0, x0, x1, x1]
    verts_y = [y0, y1, y1, y0, y0, y1, y1, y0]
    verts_z = [z0, z0, z0, z0, z1, z1, z1, z1]
    i = [0, 0, 0, 0, 4, 4, 2, 2, 0, 0, 1, 1]
    j = [1, 2, 4, 5, 5, 6, 3, 7, 1, 3, 2, 6]
    k = [2, 3, 5, 6, 6, 7, 7, 6, 5, 4, 6, 5]
    return go.Mesh3d(
        x=verts_x, y=verts_y, z=verts_z,
        i=i, j=j, k=k,
        color=color,
        flatshading=True,
        name=name,
        showlegend=showlegend,
    )


def _obs_wireframe(obs: Obstacle, color: str, width: float = 2,
                   name: str = "") -> go.Scatter3d:
    lo = obs.center - obs.half_sizes
    hi = obs.center + obs.half_sizes
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    nan = None
    xs = [x0,x1,nan, x0,x1,nan, x0,x1,nan, x0,x1,nan,
          x0,x0,nan, x1,x1,nan, x1,x1,nan, x0,x0,nan,
          x0,x0,nan, x1,x1,nan, x1,x1,nan, x0,x0]
    ys = [y0,y0,nan, y1,y1,nan, y0,y0,nan, y1,y1,nan,
          y0,y1,nan, y0,y1,nan, y0,y1,nan, y0,y1,nan,
          y0,y0,nan, y0,y0,nan, y1,y1,nan, y1,y1]
    zs = [z0,z0,nan, z0,z0,nan, z1,z1,nan, z1,z1,nan,
          z0,z0,nan, z0,z0,nan, z1,z1,nan, z1,z1,nan,
          z0,z1,nan, z0,z1,nan, z0,z1,nan, z0,z1]
    return go.Scatter3d(
        x=xs, y=ys, z=zs,
        mode="lines",
        line=dict(color=color, width=width),
        name=name,
        showlegend=False,
        connectgaps=False,
    )


def obstacle_traces(obs: Obstacle, idx: int = 0) -> List:
    """Return [mesh, wireframe] traces for one obstacle."""
    fc = _OBS_COLORS[idx % len(_OBS_COLORS)]
    wc = _OBS_WIRE_COLORS[idx % len(_OBS_WIRE_COLORS)]
    name = obs.name if obs.name else f"obs_{idx}"
    return [
        _obs_mesh(obs, fc, name),
        _obs_wireframe(obs, wc, name=name),
    ]


def plot_scene_3d(data: SceneData,
                  title: str = "Scene",
                  show: bool = True,
                  save_html: Optional[str] = None) -> go.Figure:
    """
    Visualise obstacles in 3D.

    Parameters
    ----------
    data : SceneData
        Loaded from export_scene_json().
    """
    fig = go.Figure()
    for i, obs in enumerate(data.obstacles):
        for t in obstacle_traces(obs, i):
            fig.add_trace(t)

    fig.update_layout(
        title=f"{title}  ({len(data.obstacles)} obstacles)",
        scene=dict(
            xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)",
            aspectmode="data",
        ),
        margin=dict(l=0, r=0, t=40, b=0),
    )
    if save_html:
        fig.write_html(save_html)
    if show:
        fig.show()
    return fig


def scene_traces(data: SceneData) -> List:
    """Return all obstacle traces for embedding in combined figures."""
    traces = []
    for i, obs in enumerate(data.obstacles):
        traces.extend(obstacle_traces(obs, i))
    return traces
