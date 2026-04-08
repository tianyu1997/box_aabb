"""Robot FK arm visualisation — Plotly 3D.

迁移自 v3 sbf_viz/robot_viz.py（API 无命名变更）
"""

import numpy as np
import plotly.graph_objects as go
from typing import List, Optional
from .load_data import RobotData, RobotConfig

# Qualitative colours for links
LINK_COLORS = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
    "#9467bd", "#8c564b", "#e377c2", "#7f7f7f",
    "#bcbd22", "#17becf", "#aec7e8", "#ffbb78",
]


def _arm_trace(positions: np.ndarray,
               color: str = "#1f77b4",
               width: float = 6,
               name: str = "arm",
               opacity: float = 1.0,
               showlegend: bool = True,
               dash: Optional[str] = None) -> go.Scatter3d:
    """Create a Scatter3d trace for one arm configuration (link chain)."""
    return go.Scatter3d(
        x=positions[:, 0], y=positions[:, 1], z=positions[:, 2],
        mode="lines+markers",
        line=dict(color=color, width=width, dash=dash),
        marker=dict(size=3, color=color),
        name=name,
        opacity=opacity,
        showlegend=showlegend,
    )


def _joint_markers(positions: np.ndarray,
                   color: str = "black",
                   size: int = 5,
                   name: str = "joints") -> go.Scatter3d:
    """Mark joint positions as spheres."""
    return go.Scatter3d(
        x=positions[:, 0], y=positions[:, 1], z=positions[:, 2],
        mode="markers",
        marker=dict(size=size, color=color, symbol="circle"),
        name=name,
        showlegend=False,
    )


def plot_robot_3d(data: RobotData,
                  config_indices: Optional[List[int]] = None,
                  title: str = "Robot FK",
                  show: bool = True,
                  save_html: Optional[str] = None) -> go.Figure:
    """
    Visualise robot arm at one or more configurations.

    Parameters
    ----------
    data : RobotData
        Loaded from export_robot_json().
    config_indices : list of int, optional
        Which configs to plot. None = all.
    title : str
        Figure title.
    show : bool
        Whether to call fig.show().
    save_html : str, optional
        If set, save as standalone HTML.

    Returns
    -------
    go.Figure
    """
    fig = go.Figure()

    if config_indices is None:
        config_indices = list(range(len(data.configs)))

    n = len(config_indices)
    for i, ci in enumerate(config_indices):
        cfg = data.configs[ci]
        alpha = 0.3 + 0.7 * (i / max(n - 1, 1)) if n > 1 else 1.0
        color = LINK_COLORS[i % len(LINK_COLORS)]
        name = f"q[{ci}]"

        fig.add_trace(_arm_trace(
            cfg.link_positions, color=color, width=5,
            name=name, opacity=alpha,
        ))
        fig.add_trace(_joint_markers(
            cfg.link_positions, color=color, size=4,
        ))

    fig.update_layout(
        title=title,
        scene=dict(
            xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)",
            aspectmode="data",
        ),
        legend=dict(x=0.01, y=0.99),
        margin=dict(l=0, r=0, t=40, b=0),
    )

    if save_html:
        fig.write_html(save_html)
    if show:
        fig.show()
    return fig


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
