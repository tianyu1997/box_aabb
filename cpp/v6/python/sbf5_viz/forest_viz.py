"""C-space forest box visualisation — 2D/3D projections."""

import numpy as np
import plotly.graph_objects as go
from typing import List, Optional

from .load_data import ForestData


def _rect_trace(lo_x, hi_x, lo_y, hi_y,
                color: str = "rgba(31,119,180,0.25)",
                line_color: str = "rgb(31,119,180)",
                name: str = "",
                showlegend: bool = False) -> go.Scatter:
    """A single 2D rectangle as a closed polygon trace."""
    return go.Scatter(
        x=[lo_x, hi_x, hi_x, lo_x, lo_x],
        y=[lo_y, hi_y, hi_y, lo_y, lo_y],
        fill="toself",
        fillcolor=color,
        line=dict(color=line_color, width=1),
        mode="lines",
        name=name,
        showlegend=showlegend,
    )


def plot_forest_2d(forest_data: ForestData,
                   dim_x: int = 0, dim_y: int = 1) -> go.Figure:
    """2D projection: select two joint axes, draw box rectangles."""
    fig = go.Figure()
    for i, fb in enumerate(forest_data.boxes):
        ivs = fb.intervals  # (n_dims, 2)
        fig.add_trace(_rect_trace(
            ivs[dim_x, 0], ivs[dim_x, 1],
            ivs[dim_y, 0], ivs[dim_y, 1],
            color="rgba(31,119,180,0.20)",
            line_color="rgb(31,119,180)",
            name=f"box {fb.id}",
            showlegend=(i < 10),  # legend for first 10 only
        ))

    fig.update_layout(
        title=f"Forest 2D (dims {dim_x},{dim_y}) — {forest_data.n_boxes} boxes",
        xaxis_title=f"q[{dim_x}]",
        yaxis_title=f"q[{dim_y}]",
        xaxis=dict(scaleanchor="y", scaleratio=1),
        margin=dict(l=50, r=20, t=40, b=40),
    )
    return fig


def plot_forest_3d(forest_data: ForestData,
                   dims: Optional[List[int]] = None) -> go.Figure:
    """3D projection onto three joint axes (requires >= 3 DOF)."""
    if dims is None:
        dims = [0, 1, 2]
    d0, d1, d2 = dims

    fig = go.Figure()
    for fb in forest_data.boxes:
        ivs = fb.intervals
        lo = [ivs[d0, 0], ivs[d1, 0], ivs[d2, 0]]
        hi = [ivs[d0, 1], ivs[d1, 1], ivs[d2, 1]]
        x0, y0, z0 = lo
        x1, y1, z1 = hi
        fig.add_trace(go.Mesh3d(
            x=[x0, x0, x1, x1, x0, x0, x1, x1],
            y=[y0, y1, y1, y0, y0, y1, y1, y0],
            z=[z0, z0, z0, z0, z1, z1, z1, z1],
            i=[0, 0, 0, 0, 4, 4, 2, 2, 0, 0, 1, 1],
            j=[1, 2, 4, 5, 5, 6, 3, 7, 1, 3, 2, 6],
            k=[2, 3, 5, 6, 6, 7, 7, 6, 5, 4, 6, 5],
            color="rgba(31,119,180,0.10)",
            flatshading=True,
            name=f"box {fb.id}",
            showlegend=False,
        ))

    fig.update_layout(
        title=f"Forest 3D (dims {d0},{d1},{d2}) — {forest_data.n_boxes} boxes",
        scene=dict(
            xaxis_title=f"q[{d0}]",
            yaxis_title=f"q[{d1}]",
            zaxis_title=f"q[{d2}]",
            aspectmode="data",
        ),
        margin=dict(l=0, r=0, t=40, b=0),
    )
    return fig


def plot_path_on_forest(forest_data: ForestData,
                        path: List[np.ndarray],
                        dim_x: int = 0, dim_y: int = 1) -> go.Figure:
    """Overlay path waypoints on the 2D forest projection."""
    fig = plot_forest_2d(forest_data, dim_x, dim_y)

    # Path line
    xs = [p[dim_x] for p in path]
    ys = [p[dim_y] for p in path]
    fig.add_trace(go.Scatter(
        x=xs, y=ys,
        mode="lines+markers",
        line=dict(color="red", width=3),
        marker=dict(size=6, color="red"),
        name="path",
        showlegend=True,
    ))

    # Start / goal markers
    if len(path) >= 2:
        fig.add_trace(go.Scatter(
            x=[path[0][dim_x]], y=[path[0][dim_y]],
            mode="markers",
            marker=dict(size=12, color="green", symbol="star"),
            name="start",
        ))
        fig.add_trace(go.Scatter(
            x=[path[-1][dim_x]], y=[path[-1][dim_y]],
            mode="markers",
            marker=dict(size=12, color="purple", symbol="diamond"),
            name="goal",
        ))

    return fig
