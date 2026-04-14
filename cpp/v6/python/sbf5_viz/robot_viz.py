"""Robot FK arm visualisation — Plotly 3D."""

import numpy as np
import plotly.graph_objects as go
from typing import List, Optional

from .load_data import RobotData

LINK_COLORS = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
    "#9467bd", "#8c564b", "#e377c2", "#7f7f7f",
    "#bcbd22", "#17becf", "#aec7e8", "#ffbb78",
]


def _arm_trace(positions: np.ndarray,
               color: str = "#1f77b4",
               width: float = 5,
               name: str = "arm",
               opacity: float = 1.0) -> go.Scatter3d:
    """Scatter3d trace for one arm configuration (link chain)."""
    return go.Scatter3d(
        x=positions[:, 0], y=positions[:, 1], z=positions[:, 2],
        mode="lines+markers",
        line=dict(color=color, width=width),
        marker=dict(size=3, color=color),
        name=name,
        opacity=opacity,
        showlegend=True,
    )


def _joint_markers(positions: np.ndarray,
                   color: str = "black",
                   size: int = 4) -> go.Scatter3d:
    """Joint position markers."""
    return go.Scatter3d(
        x=positions[:, 0], y=positions[:, 1], z=positions[:, 2],
        mode="markers",
        marker=dict(size=size, color=color, symbol="circle"),
        name="joints",
        showlegend=False,
    )


def robot_traces(positions: np.ndarray,
                 color: str = "#1f77b4",
                 name: str = "arm",
                 opacity: float = 1.0,
                 width: float = 5) -> List[go.Scatter3d]:
    """Return [arm_line, joint_markers] traces for embedding in other figures."""
    return [
        _arm_trace(positions, color=color, width=width,
                   name=name, opacity=opacity),
        _joint_markers(positions, color=color),
    ]


def plot_robot_3d(robot_data: RobotData,
                  config_idx: int = 0) -> go.Figure:
    """Plot a single configuration's FK link chain in 3D."""
    fig = go.Figure()
    cfg = robot_data.configs[config_idx]
    for t in robot_traces(cfg.link_positions):
        fig.add_trace(t)

    fig.update_layout(
        title=f"{robot_data.name} — config {config_idx}",
        scene=dict(
            xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)",
            aspectmode="data",
        ),
        margin=dict(l=0, r=0, t=40, b=0),
    )
    return fig


def plot_robot_multi_configs(robot_data: RobotData,
                             indices: Optional[List[int]] = None) -> go.Figure:
    """Overlay multiple configurations with opacity gradient."""
    fig = go.Figure()
    if indices is None:
        indices = list(range(len(robot_data.configs)))

    n = len(indices)
    for i, ci in enumerate(indices):
        cfg = robot_data.configs[ci]
        alpha = 0.2 + 0.8 * (i / max(n - 1, 1)) if n > 1 else 1.0
        color = LINK_COLORS[i % len(LINK_COLORS)]
        for t in robot_traces(cfg.link_positions, color=color,
                              name=f"q[{ci}]", opacity=alpha):
            fig.add_trace(t)

    fig.update_layout(
        title=f"{robot_data.name} — {n} configs",
        scene=dict(
            xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)",
            aspectmode="data",
        ),
        legend=dict(x=0.01, y=0.99),
        margin=dict(l=0, r=0, t=40, b=0),
    )
    return fig
