"""
Combined multi-layer viewer — unified Robot + Envelope + Voxel + Scene.

Reads a snapshot JSON (exported by sbf::viz::export_snapshot_json)
and overlays all layers in a single interactive Plotly figure.

迁移自 v3 sbf_viz/combined_viz.py
v4 术语更新: sub_aabbs → link_iaabbs_sub, link_aabbs → link_iaabbs
"""

import numpy as np
import plotly.graph_objects as go
from typing import Optional, List, Dict, Any

from .load_data import (
    SnapshotData, SceneData, Obstacle, IAABB,
    load_snapshot_data,
)
from .robot_viz import robot_traces
from .envelope_viz import iaabb_traces, envelope_traces
from .voxel_viz import voxel_scatter_trace
from .scene_viz import scene_traces


def _parse_snapshot_robot(rd: Dict[str, Any]):
    """Extract link positions from the robot section of a snapshot."""
    return np.array(rd["link_positions"])


def _parse_snapshot_envelope(ed: Dict[str, Any]):
    """Extract iAABB lists from the envelope section of a snapshot."""
    link_iaabbs = [
        IAABB(link=a["link"], lo=np.array(a["lo"]), hi=np.array(a["hi"]))
        for a in ed.get("link_iaabbs", [])
    ]
    link_iaabbs_sub = [
        IAABB(link=a["link"], lo=np.array(a["lo"]), hi=np.array(a["hi"]),
              seg=a.get("seg", -1))
        for a in ed.get("link_iaabbs_sub", [])
    ]
    return link_iaabbs, link_iaabbs_sub


def _parse_snapshot_voxel(vd: Dict[str, Any]):
    """Extract voxel centres + delta from voxel section of a snapshot."""
    centres = np.array(vd.get("centres", []))
    if len(centres) == 0:
        centres = np.empty((0, 3))
    delta = vd.get("delta", 0.02)
    return centres, delta


def plot_snapshot_3d(data,
                     show_robot: bool = True,
                     show_full_link_iaabb: bool = False,
                     show_link_iaabb: bool = True,
                     show_robot_voxel: bool = True,
                     show_obstacle_voxel: bool = True,
                     show_obstacles: bool = True,
                     title: str = "SafeBoxForest Snapshot",
                     show: bool = True,
                     save_html: Optional[str] = None) -> go.Figure:
    """
    Unified visualisation of all SBF layers.

    Parameters
    ----------
    data : SnapshotData or str (path to snapshot JSON).
    show_robot : bool
        Show FK arm chain.
    show_full_link_iaabb : bool
        Show per-link full iAABBs.
    show_link_iaabb : bool
        Show subdivided link iAABBs.
    show_robot_voxel : bool
        Show robot envelope voxel cloud.
    show_obstacle_voxel : bool
        Show obstacle voxel cloud.
    show_obstacles : bool
        Show obstacle iAABBs.
    """
    if isinstance(data, str):
        data = load_snapshot_data(data)

    fig = go.Figure()

    # ── Robot arm ──
    if show_robot and data.robot:
        positions = _parse_snapshot_robot(data.robot)
        for t in robot_traces(positions, color="#1f77b4", name="arm", width=6):
            fig.add_trace(t)

    # ── Envelope iAABBs ──
    if data.envelope:
        link_iaabbs, link_iaabbs_sub = _parse_snapshot_envelope(data.envelope)
        if show_full_link_iaabb:
            for t in envelope_traces(link_iaabbs, prefix="Full iAABB "):
                fig.add_trace(t)
        if show_link_iaabb:
            for t in envelope_traces(link_iaabbs_sub, prefix="iAABB "):
                fig.add_trace(t)

    # ── Robot voxels ──
    if show_robot_voxel and data.robot_voxel:
        centres, delta = _parse_snapshot_voxel(data.robot_voxel)
        if len(centres) > 0:
            fig.add_trace(voxel_scatter_trace(
                centres, delta,
                color="rgba(44,160,44,0.30)",
                name=f"robot voxels ({len(centres):,})",
            ))

    # ── Obstacle voxels ──
    if show_obstacle_voxel and data.obstacle_voxel:
        centres, delta = _parse_snapshot_voxel(data.obstacle_voxel)
        if len(centres) > 0:
            fig.add_trace(voxel_scatter_trace(
                centres, delta,
                color="rgba(214,39,40,0.30)",
                name=f"obs voxels ({len(centres):,})",
            ))

    # ── Scene obstacles ──
    if show_obstacles:
        for t in scene_traces(data.scene):
            fig.add_trace(t)

    # ── Layout ──
    fig.update_layout(
        title=title,
        scene=dict(
            xaxis_title="X (m)", yaxis_title="Y (m)", zaxis_title="Z (m)",
            aspectmode="data",
        ),
        legend=dict(x=0.01, y=0.99),
        margin=dict(l=0, r=0, t=40, b=0),
        updatemenus=[
            dict(
                type="buttons",
                direction="left",
                x=0.01, y=0.01,
                xanchor="left", yanchor="bottom",
                buttons=[
                    dict(label="All On",
                         method="update",
                         args=[{"visible": True}]),
                    dict(label="Robot Only",
                         method="update",
                         args=[{"visible": [
                             t.name is not None and ("arm" in str(t.name) or "joint" in str(t.name))
                             for t in fig.data
                         ]}]),
                    dict(label="Voxels Only",
                         method="update",
                         args=[{"visible": [
                             t.name is not None and "voxel" in str(t.name)
                             for t in fig.data
                         ]}]),
                ],
            ),
        ],
    )

    if save_html:
        fig.write_html(save_html)
    if show:
        fig.show()
    return fig
