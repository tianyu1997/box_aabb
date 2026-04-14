"""Multi-method / multi-planner comparison visualisation."""

import numpy as np
import plotly.graph_objects as go
from typing import Dict, List, Optional
from dataclasses import dataclass

from .load_data import EnvelopeData, ForestData
from .envelope_viz import _link_color, _box_mesh, _box_wireframe

# Distinct colours for different methods
_METHOD_COLORS = [
    ("rgba(31,119,180,0.15)",  "rgb(31,119,180)"),   # blue
    ("rgba(255,127,14,0.15)",  "rgb(255,127,14)"),    # orange
    ("rgba(44,160,44,0.15)",   "rgb(44,160,44)"),     # green
    ("rgba(214,39,40,0.15)",   "rgb(214,39,40)"),     # red
    ("rgba(148,103,189,0.15)", "rgb(148,103,189)"),   # purple
]

_PATH_COLORS = [
    "rgb(31,119,180)", "rgb(255,127,14)", "rgb(44,160,44)",
    "rgb(214,39,40)", "rgb(148,103,189)", "rgb(140,86,75)",
]


def compare_envelopes(envelopes: Dict[str, EnvelopeData]) -> go.Figure:
    """Multi-method envelope comparison (overlaid with legend groups).

    Parameters
    ----------
    envelopes : dict
        {method_name: EnvelopeData} — each rendered with distinct colour.
    """
    fig = go.Figure()

    for mi, (method_name, env) in enumerate(envelopes.items()):
        fill_c, wire_c = _METHOD_COLORS[mi % len(_METHOD_COLORS)]
        first_trace = True
        for ebox in env.boxes:
            for liabb in ebox.links:
                fig.add_trace(_box_mesh(
                    liabb.lo, liabb.hi, fill_c,
                    name=method_name,
                    showlegend=first_trace,
                ))
                fig.add_trace(_box_wireframe(
                    liabb.lo, liabb.hi, wire_c, width=1,
                    name=method_name,
                ))
                first_trace = False

    fig.update_layout(
        title="Envelope Comparison",
        scene=dict(
            xaxis_title="X", yaxis_title="Y", zaxis_title="Z",
            aspectmode="data",
        ),
        margin=dict(l=0, r=0, t=40, b=0),
    )
    return fig


@dataclass
class PlanResult:
    """Lightweight container for a planner result."""
    path: List[np.ndarray]   # list of config waypoints
    name: str = ""


def compare_planners(results: Dict[str, PlanResult],
                     forest_data: Optional[ForestData] = None,
                     dim_x: int = 0, dim_y: int = 1) -> go.Figure:
    """Multi-planner path comparison on 2D forest projection.

    Parameters
    ----------
    results : dict
        {planner_name: PlanResult}
    forest_data : ForestData, optional
        If provided, draw forest boxes as background.
    """
    fig = go.Figure()

    # Background forest
    if forest_data is not None:
        for fb in forest_data.boxes:
            ivs = fb.intervals
            fig.add_trace(go.Scatter(
                x=[ivs[dim_x, 0], ivs[dim_x, 1], ivs[dim_x, 1],
                   ivs[dim_x, 0], ivs[dim_x, 0]],
                y=[ivs[dim_y, 0], ivs[dim_y, 0], ivs[dim_y, 1],
                   ivs[dim_y, 1], ivs[dim_y, 0]],
                fill="toself",
                fillcolor="rgba(200,200,200,0.15)",
                line=dict(color="rgba(150,150,150,0.3)", width=0.5),
                mode="lines",
                showlegend=False,
            ))

    # Paths
    for pi, (pname, pr) in enumerate(results.items()):
        color = _PATH_COLORS[pi % len(_PATH_COLORS)]
        xs = [p[dim_x] for p in pr.path]
        ys = [p[dim_y] for p in pr.path]
        fig.add_trace(go.Scatter(
            x=xs, y=ys,
            mode="lines+markers",
            line=dict(color=color, width=3),
            marker=dict(size=5, color=color),
            name=pname,
        ))

    fig.update_layout(
        title="Planner Comparison",
        xaxis_title=f"q[{dim_x}]",
        yaxis_title=f"q[{dim_y}]",
        xaxis=dict(scaleanchor="y", scaleratio=1),
        margin=dict(l=50, r=20, t=40, b=40),
    )
    return fig
