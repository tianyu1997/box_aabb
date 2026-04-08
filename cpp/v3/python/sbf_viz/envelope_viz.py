"""AABB envelope visualisation — Plotly 3D wireframes & transparent boxes."""

import numpy as np
import plotly.graph_objects as go
from typing import List, Optional, Tuple
from .load_data import EnvelopeData, AABB

# Per-link colours (transparent fill + opaque wireframe)
_LINK_COLORS = [
    ("rgba(31,119,180,0.12)",  "rgb(31,119,180)"),   # blue
    ("rgba(255,127,14,0.12)",  "rgb(255,127,14)"),    # orange
    ("rgba(44,160,44,0.12)",   "rgb(44,160,44)"),     # green
    ("rgba(214,39,40,0.12)",   "rgb(214,39,40)"),     # red
    ("rgba(148,103,189,0.12)", "rgb(148,103,189)"),   # purple
    ("rgba(140,86,75,0.12)",   "rgb(140,86,75)"),     # brown
    ("rgba(227,119,194,0.12)", "rgb(227,119,194)"),   # pink
    ("rgba(127,127,127,0.12)", "rgb(127,127,127)"),   # grey
]


def _aabb_mesh(lo, hi, color_fill: str, name: str,
               showlegend: bool = True) -> go.Mesh3d:
    """Build a semi-transparent Mesh3d box from lo/hi corners."""
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
        color=color_fill,
        flatshading=True,
        name=name,
        showlegend=showlegend,
    )


def _aabb_wireframe(lo, hi, color_line: str, width: float = 2,
                    name: str = "", showlegend: bool = False) -> go.Scatter3d:
    """Build a wireframe for an AABB (12 edges)."""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    # 12 edges traced as a single polyline with None breaks
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
        line=dict(color=color_line, width=width),
        name=name,
        showlegend=showlegend,
        connectgaps=False,
    )


def aabb_traces(aabb: AABB, link_color_idx: int = 0,
                prefix: str = "", show_fill: bool = True,
                show_wire: bool = True,
                showlegend: bool = True) -> List:
    """Return Plotly traces for a single AABB."""
    fill_c, wire_c = _LINK_COLORS[link_color_idx % len(_LINK_COLORS)]
    label = f"{prefix}link {aabb.link}" + (f" seg {aabb.seg}" if aabb.seg >= 0 else "")
    traces = []
    if show_fill:
        traces.append(_aabb_mesh(aabb.lo, aabb.hi, fill_c, label, showlegend))
    if show_wire:
        traces.append(_aabb_wireframe(aabb.lo, aabb.hi, wire_c, name=label))
    return traces


def plot_envelope_3d(data: EnvelopeData,
                     node_idx: int = 0,
                     show_full_aabb: bool = True,
                     show_sub_aabb: bool = True,
                     title: str = "Envelope AABBs",
                     show: bool = True,
                     save_html: Optional[str] = None) -> go.Figure:
    """
    Visualise AABB envelope for one node.

    Parameters
    ----------
    data : EnvelopeData
    node_idx : int
        Index into data.nodes (not the C++ node_idx).
    show_full_aabb : bool
        Show full per-link AABBs.
    show_sub_aabb : bool
        Show subdivided sub-AABBs.
    """
    fig = go.Figure()
    node = data.nodes[node_idx]

    # Map link index → colour index
    link_ids = sorted(set(a.link for a in node.link_aabbs))
    link2ci = {lid: i for i, lid in enumerate(link_ids)}

    if show_full_aabb:
        for a in node.link_aabbs:
            for t in aabb_traces(a, link2ci.get(a.link, 0), prefix="AABB "):
                fig.add_trace(t)

    if show_sub_aabb:
        for a in node.sub_aabbs:
            for t in aabb_traces(a, link2ci.get(a.link, 0), prefix="Sub ",
                                 showlegend=(a.seg == 0)):
                fig.add_trace(t)

    fig.update_layout(
        title=title,
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


def envelope_traces(aabbs: List[AABB],
                    prefix: str = "",
                    show_fill: bool = True,
                    show_wire: bool = True) -> List:
    """Build traces for a list of AABBs (for embedding in combined figures)."""
    link_ids = sorted(set(a.link for a in aabbs))
    link2ci = {lid: i for i, lid in enumerate(link_ids)}
    traces = []
    seen = set()
    for a in aabbs:
        ci = link2ci.get(a.link, 0)
        sl = a.link not in seen
        seen.add(a.link)
        traces.extend(aabb_traces(a, ci, prefix, show_fill, show_wire, sl))
    return traces
