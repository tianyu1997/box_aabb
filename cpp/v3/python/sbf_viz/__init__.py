# SafeBoxForest v3 — Python Visualization Package
#
# Renders JSON exports from the C++ sbf::viz module as interactive 3D HTML
# using Plotly, or static figures via Matplotlib.
#
# Usage:
#   1. C++ side: call sbf::viz::export_*_json() to produce JSON files.
#   2. Python side: import sbf_viz and call the appropriate viewer.
#
# Sub-modules:
#   load_data     — JSON loading + lightweight data classes
#   robot_viz     — 3D arm / FK link-chain visualisation
#   envelope_viz  — AABB wireframes & filled boxes
#   voxel_viz     — Sparse voxel scatter / brick rendering
#   scene_viz     — Obstacle boxes
#   combined_viz  — Unified multi-layer viewer
#   envelope_comparison_viz — Multi-method envelope comparison with toggles
#
# Requirements: plotly, numpy  (pip install plotly numpy)

from .load_data import (
    load_robot_data,
    load_envelope_data,
    load_voxel_data,
    load_voxel_centres_data,
    load_scene_data,
    load_snapshot_data,
)
from .robot_viz import plot_robot_3d
from .envelope_viz import plot_envelope_3d
from .voxel_viz import plot_voxel_3d
from .scene_viz import plot_scene_3d
from .combined_viz import plot_snapshot_3d
from .envelope_comparison_viz import plot_envelope_comparison, load_envelope_comparison

__all__ = [
    "load_robot_data",
    "load_envelope_data",
    "load_voxel_data",
    "load_voxel_centres_data",
    "load_scene_data",
    "load_snapshot_data",
    "plot_robot_3d",
    "plot_envelope_3d",
    "plot_voxel_3d",
    "plot_scene_3d",
    "plot_snapshot_3d",
    "load_envelope_comparison",
    "plot_envelope_comparison",
]
