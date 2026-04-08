"""Combined multi-layer viewer — Robot + Envelope + Scene + Forest."""

import plotly.graph_objects as go

from .load_data import SnapshotData
from .robot_viz import robot_traces
from .envelope_viz import add_envelope_traces
from .scene_viz import add_scene_traces


def plot_combined(snapshot: SnapshotData,
                  show_robot: bool = True,
                  show_envelope: bool = True,
                  show_scene: bool = True,
                  show_forest: bool = False) -> go.Figure:
    """Multi-layer 3D view with Plotly toggle buttons.

    Parameters
    ----------
    snapshot : SnapshotData
        Loaded via load_snapshot().
    show_robot, show_envelope, show_scene, show_forest : bool
        Initial visibility of each layer.
    """
    fig = go.Figure()

    # ── Robot arm ──
    if show_robot and snapshot.robot and snapshot.robot.configs:
        cfg = snapshot.robot.configs[0]
        for t in robot_traces(cfg.link_positions, name="arm", width=6):
            fig.add_trace(t)

    # ── Envelope iAABBs ──
    if show_envelope and snapshot.envelope:
        add_envelope_traces(fig, snapshot.envelope, name="envelope")

    # ── Scene obstacles ──
    if show_scene and snapshot.scene:
        add_scene_traces(fig, snapshot.scene)

    # ── Forest C-space boxes (3D projection, first 3 dims) ──
    if show_forest and snapshot.forest:
        from .forest_viz import plot_forest_3d
        # We add forest box meshes directly
        for fb in snapshot.forest.boxes:
            ivs = fb.intervals
            if len(ivs) < 3:
                continue
            lo = [ivs[0, 0], ivs[1, 0], ivs[2, 0]]
            hi = [ivs[0, 1], ivs[1, 1], ivs[2, 1]]
            x0, y0, z0 = lo
            x1, y1, z1 = hi
            fig.add_trace(go.Mesh3d(
                x=[x0, x0, x1, x1, x0, x0, x1, x1],
                y=[y0, y1, y1, y0, y0, y1, y1, y0],
                z=[z0, z0, z0, z0, z1, z1, z1, z1],
                i=[0, 0, 0, 0, 4, 4, 2, 2, 0, 0, 1, 1],
                j=[1, 2, 4, 5, 5, 6, 3, 7, 1, 3, 2, 6],
                k=[2, 3, 5, 6, 6, 7, 7, 6, 5, 4, 6, 5],
                color="rgba(100,200,100,0.08)",
                flatshading=True,
                name="forest",
                showlegend=False,
            ))

    # ── Layout + toggle buttons ──
    fig.update_layout(
        title="SafeBoxForest Snapshot",
        scene=dict(
            xaxis_title="X", yaxis_title="Y", zaxis_title="Z",
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
                             "arm" in (t.name or "") or "joint" in (t.name or "")
                             for t in fig.data
                         ]}]),
                    dict(label="Envelope Only",
                         method="update",
                         args=[{"visible": [
                             "envelope" in (t.name or "")
                             for t in fig.data
                         ]}]),
                    dict(label="Scene Only",
                         method="update",
                         args=[{"visible": [
                             "obs" in (t.name or "")
                             for t in fig.data
                         ]}]),
                ],
            ),
        ],
    )
    return fig
