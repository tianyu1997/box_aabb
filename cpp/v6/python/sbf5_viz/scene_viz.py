"""Scene (obstacle) visualisation — Plotly 3D."""

import numpy as np
import plotly.graph_objects as go
from typing import List

from .load_data import SceneData, Obstacle

_OBS_COLORS = [
    ("rgba(214,39,40,0.25)",   "rgb(214,39,40)"),
    ("rgba(255,187,120,0.25)", "rgb(255,187,120)"),
    ("rgba(152,78,163,0.25)",  "rgb(152,78,163)"),
    ("rgba(255,127,0,0.25)",   "rgb(255,127,0)"),
]


def _obs_mesh(lo, hi, color: str, name: str,
              showlegend: bool = True) -> go.Mesh3d:
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    return go.Mesh3d(
        x=[x0, x0, x1, x1, x0, x0, x1, x1],
        y=[y0, y1, y1, y0, y0, y1, y1, y0],
        z=[z0, z0, z0, z0, z1, z1, z1, z1],
        i=[0, 0, 0, 0, 4, 4, 2, 2, 0, 0, 1, 1],
        j=[1, 2, 4, 5, 5, 6, 3, 7, 1, 3, 2, 6],
        k=[2, 3, 5, 6, 6, 7, 7, 6, 5, 4, 6, 5],
        color=color,
        flatshading=True,
        name=name,
        showlegend=showlegend,
    )


def _obs_wireframe(lo, hi, color: str, width: float = 2) -> go.Scatter3d:
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
        name="",
        showlegend=False,
        connectgaps=False,
    )


def obstacle_traces(obs: Obstacle, idx: int = 0) -> List:
    """Return [mesh, wireframe] traces for one obstacle."""
    lo = obs.center - obs.half_sizes
    hi = obs.center + obs.half_sizes
    fill_c, wire_c = _OBS_COLORS[idx % len(_OBS_COLORS)]
    return [
        _obs_mesh(lo, hi, fill_c, f"obs_{idx}"),
        _obs_wireframe(lo, hi, wire_c),
    ]


def scene_traces(data: SceneData) -> List:
    """Return all obstacle traces for embedding in combined figures."""
    traces = []
    for i, obs in enumerate(data.obstacles):
        traces.extend(obstacle_traces(obs, i))
    return traces


def plot_scene_3d(scene_data: SceneData) -> go.Figure:
    """3D obstacles (grey semi-transparent)."""
    fig = go.Figure()
    for t in scene_traces(scene_data):
        fig.add_trace(t)

    fig.update_layout(
        title=f"Scene ({len(scene_data.obstacles)} obstacles)",
        scene=dict(
            xaxis_title="X", yaxis_title="Y", zaxis_title="Z",
            aspectmode="data",
        ),
        margin=dict(l=0, r=0, t=40, b=0),
    )
    return fig


def add_scene_traces(fig: go.Figure, scene_data: SceneData) -> None:
    """Add obstacle traces to an existing figure."""
    for t in scene_traces(scene_data):
        fig.add_trace(t)
