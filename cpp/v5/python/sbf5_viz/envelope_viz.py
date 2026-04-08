"""iAABB envelope visualisation — Plotly 3D wireframes & transparent boxes."""

import numpy as np
import plotly.graph_objects as go
from typing import List, Optional

from .load_data import EnvelopeData, LinkIAABB

# Per-link colours: (fill_rgba, wire_rgb)
_LINK_COLORS = [
    ("rgba(31,119,180,0.12)",  "rgb(31,119,180)"),
    ("rgba(255,127,14,0.12)",  "rgb(255,127,14)"),
    ("rgba(44,160,44,0.12)",   "rgb(44,160,44)"),
    ("rgba(214,39,40,0.12)",   "rgb(214,39,40)"),
    ("rgba(148,103,189,0.12)", "rgb(148,103,189)"),
    ("rgba(140,86,75,0.12)",   "rgb(140,86,75)"),
    ("rgba(227,119,194,0.12)", "rgb(227,119,194)"),
    ("rgba(127,127,127,0.12)", "rgb(127,127,127)"),
]


def _box_mesh(lo, hi, color_fill: str, name: str,
              showlegend: bool = True) -> go.Mesh3d:
    """Semi-transparent Mesh3d box from lo/hi corners."""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    return go.Mesh3d(
        x=[x0, x0, x1, x1, x0, x0, x1, x1],
        y=[y0, y1, y1, y0, y0, y1, y1, y0],
        z=[z0, z0, z0, z0, z1, z1, z1, z1],
        i=[0, 0, 0, 0, 4, 4, 2, 2, 0, 0, 1, 1],
        j=[1, 2, 4, 5, 5, 6, 3, 7, 1, 3, 2, 6],
        k=[2, 3, 5, 6, 6, 7, 7, 6, 5, 4, 6, 5],
        color=color_fill,
        flatshading=True,
        name=name,
        showlegend=showlegend,
    )


def _box_wireframe(lo, hi, color_line: str, width: float = 2,
                   name: str = "") -> go.Scatter3d:
    """Wireframe for a box (12 edges with None breaks)."""
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
        line=dict(color=color_line, width=width),
        name=name,
        showlegend=False,
        connectgaps=False,
    )


def _link_color(link_idx: int):
    """Return (fill, wire) color pair for a link index."""
    return _LINK_COLORS[link_idx % len(_LINK_COLORS)]


def plot_envelope_wireframe(envelope_data: EnvelopeData,
                            link_filter: Optional[List[int]] = None) -> go.Figure:
    """Each link iAABB as wireframe box."""
    fig = go.Figure()
    seen_links = set()
    for ebox in envelope_data.boxes:
        for liabb in ebox.links:
            if link_filter and liabb.link_idx not in link_filter:
                continue
            _, wire_c = _link_color(liabb.link_idx)
            sl = liabb.link_idx not in seen_links
            seen_links.add(liabb.link_idx)
            fig.add_trace(_box_wireframe(
                liabb.lo, liabb.hi, wire_c, width=2,
                name=f"link {liabb.link_idx}",
            ))
            if sl:
                # Add invisible mesh just for legend entry
                fig.add_trace(go.Scatter3d(
                    x=[None], y=[None], z=[None],
                    mode="markers",
                    marker=dict(size=0, color=wire_c),
                    name=f"link {liabb.link_idx}",
                    showlegend=True,
                ))

    fig.update_layout(
        title=f"Envelope Wireframe ({envelope_data.method})",
        scene=dict(
            xaxis_title="X", yaxis_title="Y", zaxis_title="Z",
            aspectmode="data",
        ),
        margin=dict(l=0, r=0, t=40, b=0),
    )
    return fig


def plot_envelope_filled(envelope_data: EnvelopeData,
                         opacity: float = 0.15) -> go.Figure:
    """Semi-transparent filled boxes (paper-quality)."""
    fig = go.Figure()
    seen_links = set()
    for ebox in envelope_data.boxes:
        for liabb in ebox.links:
            fill_c, wire_c = _link_color(liabb.link_idx)
            # Override alpha with requested opacity
            base = fill_c.replace("0.12", str(opacity))
            sl = liabb.link_idx not in seen_links
            seen_links.add(liabb.link_idx)
            fig.add_trace(_box_mesh(
                liabb.lo, liabb.hi, base,
                name=f"link {liabb.link_idx}", showlegend=sl,
            ))
            fig.add_trace(_box_wireframe(
                liabb.lo, liabb.hi, wire_c, width=1,
            ))

    fig.update_layout(
        title=f"Envelope Filled ({envelope_data.method})",
        scene=dict(
            xaxis_title="X", yaxis_title="Y", zaxis_title="Z",
            aspectmode="data",
        ),
        margin=dict(l=0, r=0, t=40, b=0),
    )
    return fig


def add_envelope_traces(fig: go.Figure,
                        envelope_data: EnvelopeData,
                        name: str = "envelope",
                        visible: bool = True) -> None:
    """Add envelope traces to an existing figure (for combined views)."""
    seen_links = set()
    for ebox in envelope_data.boxes:
        for liabb in ebox.links:
            fill_c, wire_c = _link_color(liabb.link_idx)
            sl = liabb.link_idx not in seen_links
            seen_links.add(liabb.link_idx)
            fig.add_trace(_box_mesh(
                liabb.lo, liabb.hi, fill_c,
                name=f"{name} link {liabb.link_idx}",
                showlegend=sl,
            ))
            fig.add_trace(_box_wireframe(
                liabb.lo, liabb.hi, wire_c, width=1,
                name=f"{name} link {liabb.link_idx}",
            ))
    if not visible:
        for t in fig.data:
            if name in (t.name or ""):
                t.visible = False
