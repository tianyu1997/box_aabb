"""Endpoint iAABB comparison visualisation — Plotly 3D wireframes.

每个 joint endpoint 位置区间框的三维可视化，支持多 Stage-1 源并排对比：
    IFK / CritSample / Analytical / GCPC

用法:
    data = load_ep_iaabb_comparison("path/to/ep_iaabb_comparison.json")
    fig  = plot_ep_iaabb_comparison(data)
    fig.show()
"""

from __future__ import annotations

import json
import numpy as np
import plotly.graph_objects as go
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# ─── Method style palette ────────────────────────────────────────────────────
EP_METHOD_STYLES: Dict[str, Dict[str, str]] = {
    "IFK":        {"label": "IFK",        "wire": "rgb(232,48,48)",   "fill": "rgba(232,48,48,0.07)"},
    "CritSample": {"label": "CritSample", "wire": "rgb(74,144,217)",  "fill": "rgba(74,144,217,0.07)"},
    "Analytical": {"label": "Analytical", "wire": "rgb(245,166,35)",  "fill": "rgba(245,166,35,0.07)"},
    "GCPC":       {"label": "GCPC",       "wire": "rgb(126,211,33)",  "fill": "rgba(126,211,33,0.07)"},
}

# Default style for unknown source names (cycle through these)
_FALLBACK_COLORS = [
    ("rgb(150,150,150)", "rgba(150,150,150,0.07)"),
    ("rgb(200,100,200)", "rgba(200,100,200,0.07)"),
    ("rgb(100,200,200)", "rgba(100,200,200,0.07)"),
    ("rgb(200,200,100)", "rgba(200,200,100,0.07)"),
]


def _style_for(name: str, fallback_idx: int = 0) -> Dict[str, str]:
    if name in EP_METHOD_STYLES:
        return EP_METHOD_STYLES[name]
    wire, fill = _FALLBACK_COLORS[fallback_idx % len(_FALLBACK_COLORS)]
    return {"label": name, "wire": wire, "fill": fill}


# ─── Data classes ─────────────────────────────────────────────────────────────
@dataclass
class EndpointIAABB:
    endpoint: int
    lo: np.ndarray   # shape (3,)
    hi: np.ndarray   # shape (3,)

    @property
    def center(self) -> np.ndarray:
        return (self.lo + self.hi) * 0.5

    @property
    def volume(self) -> float:
        d = self.hi - self.lo
        return float(np.prod(np.maximum(d, 0.0)))


@dataclass
class EpIAABBSource:
    name: str
    endpoint_iaabbs: List[EndpointIAABB]   # length = n_endpoints


@dataclass
class EpIAABBComparison:
    robot_name: str
    n_joints: int
    n_active_links: int
    n_endpoints: int
    link_radii: np.ndarray          # shape (n_active_links,)
    active_link_map: List[int]      # length n_active_links
    box_intervals: List[Tuple[float, float]]  # list of (lo, hi) per joint
    link_positions: np.ndarray      # shape (n_joints+1, 3)  FK at box center
    sources: List[EpIAABBSource]


# ─── Loader ──────────────────────────────────────────────────────────────────
def load_ep_iaabb_comparison(json_path: str) -> EpIAABBComparison:
    """Load the JSON produced by ``export_ep_iaabb_comparison_json``."""
    with open(json_path, "r", encoding="utf-8") as f:
        d = json.load(f)

    robot_name    = d["robot_name"]
    n_joints      = d["n_joints"]
    n_active      = d["n_active_links"]
    n_endpoints   = d["n_endpoints"]
    link_radii    = np.asarray(d.get("link_radii", []), dtype=float)
    alm           = d.get("active_link_map", [])
    box_intervals = [(iv[0], iv[1]) for iv in d["box_intervals"]]

    ra = d.get("robot_arm", {})
    link_positions = np.asarray(ra.get("link_positions", []), dtype=float)

    sources: List[EpIAABBSource] = []
    for src_name, src_data in d.get("methods", {}).items():
        ep_list: List[EndpointIAABB] = []
        for ep in src_data.get("endpoint_iaabbs", []):
            ep_list.append(EndpointIAABB(
                endpoint=ep["endpoint"],
                lo=np.asarray(ep["lo"], dtype=float),
                hi=np.asarray(ep["hi"], dtype=float),
            ))
        sources.append(EpIAABBSource(name=src_name, endpoint_iaabbs=ep_list))

    return EpIAABBComparison(
        robot_name=robot_name,
        n_joints=n_joints,
        n_active_links=n_active,
        n_endpoints=n_endpoints,
        link_radii=link_radii,
        active_link_map=alm,
        box_intervals=box_intervals,
        link_positions=link_positions,
        sources=sources,
    )


# ─── Wireframe helper ────────────────────────────────────────────────────────
def _ep_wireframe(
    lo: np.ndarray,
    hi: np.ndarray,
    color: str,
    width: float,
    name: str,
    legendgroup: str,
    showlegend: bool,
) -> go.Scatter3d:
    """12-edge wireframe box as a single Scatter3d trace (None-break trick)."""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    N = None
    xs = [x0,x1,N, x0,x1,N, x0,x1,N, x0,x1,N,
          x0,x0,N, x1,x1,N, x1,x1,N, x0,x0,N,
          x0,x0,N, x1,x1,N, x1,x1,N, x0,x0]
    ys = [y0,y0,N, y1,y1,N, y0,y0,N, y1,y1,N,
          y0,y1,N, y0,y1,N, y0,y1,N, y0,y1,N,
          y0,y0,N, y0,y0,N, y1,y1,N, y1,y1]
    zs = [z0,z0,N, z0,z0,N, z1,z1,N, z1,z1,N,
          z0,z0,N, z0,z0,N, z1,z1,N, z1,z1,N,
          z0,z1,N, z0,z1,N, z0,z1,N, z0,z1]
    return go.Scatter3d(
        x=xs, y=ys, z=zs,
        mode="lines",
        line=dict(color=color, width=width),
        name=name,
        legendgroup=legendgroup,
        showlegend=showlegend,
        connectgaps=False,
    )


def _ep_mesh(
    lo: np.ndarray,
    hi: np.ndarray,
    color_fill: str,
    name: str,
    legendgroup: str,
    showlegend: bool,
) -> go.Mesh3d:
    """Semi-transparent Mesh3d box."""
    x0, y0, z0 = lo
    x1, y1, z1 = hi
    vx = [x0, x0, x1, x1, x0, x0, x1, x1]
    vy = [y0, y1, y1, y0, y0, y1, y1, y0]
    vz = [z0, z0, z0, z0, z1, z1, z1, z1]
    ii = [0, 0, 0, 0, 4, 4, 2, 2, 0, 0, 1, 1]
    jj = [1, 2, 4, 5, 5, 6, 3, 7, 1, 3, 2, 6]
    kk = [2, 3, 5, 6, 6, 7, 7, 6, 5, 4, 6, 5]
    return go.Mesh3d(
        x=vx, y=vy, z=vz,
        i=ii, j=jj, k=kk,
        color=color_fill,
        flatshading=True,
        name=name,
        legendgroup=legendgroup,
        showlegend=showlegend,
    )


# ─── Main visualisation ───────────────────────────────────────────────────────
def plot_ep_iaabb_comparison(
    data: EpIAABBComparison,
    title: str = "Endpoint iAABB Comparison",
    wire_width: float = 2.0,
    show_fill: bool = True,
    show: bool = False,
    save_html: Optional[str] = None,
) -> go.Figure:
    """
    3D comparison of endpoint iAABBs from multiple Stage-1 sources.

    Each source forms a toggleable legend group.  All endpoint boxes for that
    source share the same colour; clicking the legend entry hides/shows them.

    Parameters
    ----------
    data        : loaded via ``load_ep_iaabb_comparison()``
    title       : figure title
    wire_width  : wireframe line width
    show_fill   : render semi-transparent mesh fill in addition to wireframes
    show        : call ``fig.show()`` immediately
    save_html   : optional path to write an HTML file

    Returns
    -------
    go.Figure
    """
    fig = go.Figure()

    # ── Robot arm skeleton ──────────────────────────────────────────────────
    if data.link_positions is not None and len(data.link_positions) > 0:
        pos = data.link_positions
        fig.add_trace(go.Scatter3d(
            x=pos[:, 0], y=pos[:, 1], z=pos[:, 2],
            mode="lines+markers",
            line=dict(color="rgb(60,60,60)", width=4),
            marker=dict(size=5, color="rgb(60,60,60)"),
            name="Robot Arm (FK center)",
            legendgroup="robot_arm",
            showlegend=True,
        ))

    # ── Per-source endpoint iAABBs ──────────────────────────────────────────
    for s_idx, src in enumerate(data.sources):
        style = _style_for(src.name, s_idx)
        wire_c = style["wire"]
        fill_c = style["fill"]
        label  = style["label"]
        group  = f"ep_iaabb_{src.name}"

        for ep_idx, ep_box in enumerate(src.endpoint_iaabbs):
            # Only the very first box in the group shows a legend entry
            show_legend = (ep_idx == 0)
            box_label   = label if show_legend else ""

            if show_fill:
                fig.add_trace(_ep_mesh(
                    ep_box.lo, ep_box.hi,
                    color_fill=fill_c,
                    name=box_label,
                    legendgroup=group,
                    showlegend=show_legend,
                ))
                # subsequent boxes in fill should not show legend again
                show_legend = False

            fig.add_trace(_ep_wireframe(
                ep_box.lo, ep_box.hi,
                color=wire_c,
                width=wire_width,
                name=box_label,
                legendgroup=group,
                showlegend=show_legend,
            ))

    # ── Endpoint center markers ─────────────────────────────────────────────
    # Mark the FK center of each endpoint across all sources (for reference)
    if data.link_positions is not None and len(data.link_positions) >= data.n_endpoints:
        ep_pos = data.link_positions[:data.n_endpoints]
        fig.add_trace(go.Scatter3d(
            x=ep_pos[:, 0], y=ep_pos[:, 1], z=ep_pos[:, 2],
            mode="markers",
            marker=dict(size=4, color="rgb(80,80,80)", symbol="cross"),
            name="Endpoint Centers",
            legendgroup="ep_centers",
            showlegend=True,
        ))

    fig.update_layout(
        title=dict(text=title, font=dict(size=16)),
        legend=dict(
            title="Source  (click to toggle)",
            itemsizing="constant",
            tracegroupgap=4,
        ),
        scene=dict(
            xaxis_title="X (m)",
            yaxis_title="Y (m)",
            zaxis_title="Z (m)",
            aspectmode="data",
        ),
        margin=dict(l=0, r=0, t=50, b=0),
    )

    if save_html:
        fig.write_html(save_html)
        print(f"[ep_iaabb_viz] Saved: {save_html}")

    if show:
        fig.show()

    return fig
