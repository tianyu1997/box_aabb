"""Sparse voxel point-cloud visualisation (Phase P4)."""

import plotly.graph_objects as go

from .load_data import VoxelData


def plot_voxel_scatter(voxel_data: VoxelData,
                       opacity: float = 0.3,
                       marker_size: float = 2.0) -> go.Figure:
    """Sparse voxel centres as 3D scatter plot."""
    fig = go.Figure()

    if voxel_data.centres.shape[0] > 0:
        fig.add_trace(go.Scatter3d(
            x=voxel_data.centres[:, 0],
            y=voxel_data.centres[:, 1],
            z=voxel_data.centres[:, 2],
            mode="markers",
            marker=dict(size=marker_size, opacity=opacity,
                        color="rgb(44,160,44)"),
            name="voxel",
        ))

    fig.update_layout(
        title=f"Voxel Grid (delta={voxel_data.delta}, "
              f"n={voxel_data.total_occupied})",
        scene=dict(
            xaxis_title="X", yaxis_title="Y", zaxis_title="Z",
            aspectmode="data",
        ),
        margin=dict(l=0, r=0, t=40, b=0),
    )
    return fig


def add_voxel_traces(fig: go.Figure, voxel_data: VoxelData,
                     opacity: float = 0.3,
                     marker_size: float = 2.0) -> None:
    """Add voxel scatter traces to an existing figure."""
    if voxel_data.centres.shape[0] == 0:
        return

    fig.add_trace(go.Scatter3d(
        x=voxel_data.centres[:, 0],
        y=voxel_data.centres[:, 1],
        z=voxel_data.centres[:, 2],
        mode="markers",
        marker=dict(size=marker_size, opacity=opacity,
                    color="rgb(44,160,44)"),
        name="voxel",
    ))
