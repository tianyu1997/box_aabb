"""Sparse voxel grid visualisation — Plotly 3D scatter & isosurface."""

import numpy as np
import plotly.graph_objects as go
from typing import List, Optional, Tuple
from .load_data import VoxelData, VoxelCentresData, voxel_data_to_centres


def _voxel_scatter(centres: np.ndarray,
                   delta: float,
                   color: str = "rgba(44,160,44,0.35)",
                   name: str = "voxels",
                   showlegend: bool = True,
                   marker_size: float = 0) -> go.Scatter3d:
    """
    Render occupied voxels as a 3D scatter plot.
    Marker size defaults to approximate voxel edge length in points.
    """
    if marker_size <= 0:
        marker_size = max(2.0, min(6.0, delta * 150))  # heuristic
    return go.Scatter3d(
        x=centres[:, 0], y=centres[:, 1], z=centres[:, 2],
        mode="markers",
        marker=dict(
            size=marker_size,
            color=color,
            opacity=0.4,
            symbol="square",
        ),
        name=name,
        showlegend=showlegend,
    )


def _voxel_cube_mesh(centres: np.ndarray, delta: float,
                     color: str = "rgba(44,160,44,0.15)",
                     name: str = "voxel cubes",
                     max_cubes: int = 3000) -> Optional[go.Mesh3d]:
    """
    Render voxels as tiny cubes (Mesh3d). For up to max_cubes voxels;
    beyond that, scatter is more practical.
    """
    n = min(len(centres), max_cubes)
    if n == 0:
        return None
    h = delta / 2.0
    # Build vertex arrays for n cubes (8 verts each)
    all_x, all_y, all_z = [], [], []
    all_i, all_j, all_k = [], [], []
    for idx in range(n):
        cx, cy, cz = centres[idx]
        base = idx * 8
        for dx in (-h, h):
            for dy in (-h, h):
                for dz in (-h, h):
                    all_x.append(cx + dx)
                    all_y.append(cy + dy)
                    all_z.append(cz + dz)
        # 12 triangles per cube
        faces_i = [0, 0, 0, 0, 4, 4, 2, 2, 0, 0, 1, 1]
        faces_j = [1, 2, 4, 5, 5, 6, 3, 7, 1, 3, 2, 6]
        faces_k = [2, 3, 5, 6, 6, 7, 7, 6, 5, 4, 6, 5]
        all_i.extend([f + base for f in faces_i])
        all_j.extend([f + base for f in faces_j])
        all_k.extend([f + base for f in faces_k])

    return go.Mesh3d(
        x=all_x, y=all_y, z=all_z,
        i=all_i, j=all_j, k=all_k,
        color=color,
        flatshading=True,
        name=name,
        showlegend=True,
    )


def voxel_scatter_trace(centres: np.ndarray,
                        delta: float,
                        color: str = "rgba(44,160,44,0.35)",
                        name: str = "voxels") -> go.Scatter3d:
    """Public helper for embedding in combined figures."""
    return _voxel_scatter(centres, delta, color=color, name=name)


def plot_voxel_3d(data,
                  mode: str = "scatter",
                  color: str = "rgba(44,160,44,0.35)",
                  title: str = "Voxel Grid",
                  show: bool = True,
                  save_html: Optional[str] = None,
                  max_cubes: int = 3000) -> go.Figure:
    """
    Visualise a voxel grid.

    Parameters
    ----------
    data : VoxelData or VoxelCentresData
        Loaded from export_voxel_json() or export_voxel_centres_json().
    mode : str
        "scatter" = point cloud, "cubes" = tiny Mesh3d cubes.
    """
    fig = go.Figure()

    if isinstance(data, VoxelCentresData):
        centres = data.centres
        delta   = data.delta
    elif isinstance(data, VoxelData):
        centres = voxel_data_to_centres(data)
        delta   = data.delta
    else:
        raise TypeError(f"Expected VoxelData or VoxelCentresData, got {type(data)}")

    if len(centres) == 0:
        fig.update_layout(title=title + " (empty)")
        return fig

    if mode == "cubes" and len(centres) <= max_cubes:
        mesh = _voxel_cube_mesh(centres, delta, color=color)
        if mesh:
            fig.add_trace(mesh)
    else:
        fig.add_trace(_voxel_scatter(centres, delta, color=color))

    # Add annotation
    n_occ = len(centres)
    vol = n_occ * delta**3
    fig.update_layout(
        title=f"{title}  ({n_occ:,} voxels, Δ={delta:.3f} m, vol={vol:.4f} m³)",
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


def plot_voxel_comparison(grids: List[Tuple[np.ndarray, float, str, str]],
                          title: str = "Voxel Comparison",
                          show: bool = True,
                          save_html: Optional[str] = None) -> go.Figure:
    """
    Overlay multiple voxel grids for comparison.

    Parameters
    ----------
    grids : list of (centres, delta, color, name) tuples.
    """
    fig = go.Figure()
    for centres, delta, color, name in grids:
        if len(centres) > 0:
            fig.add_trace(_voxel_scatter(centres, delta, color=color, name=name))
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
