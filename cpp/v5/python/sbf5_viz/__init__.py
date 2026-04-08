# SafeBoxForest v5 — Python Visualization Package (sbf5_viz)
#
# Renders JSON exports from the C++ sbf::viz module as interactive 3D HTML
# using Plotly.
#
# Usage:
#   python -m sbf5_viz snapshot.json --html output.html
#
# Sub-modules:
#   load_data       — JSON loading + data classes
#   robot_viz       — 3D arm / FK link-chain
#   envelope_viz    — iAABB wireframes & filled boxes
#   forest_viz      — C-space box 2D/3D projection
#   scene_viz       — Obstacle boxes
#   combined_viz    — Unified multi-layer viewer
#   comparison_viz  — Multi-method/planner comparison
#   cli             — Command-line entry point
#
# Requirements: plotly >= 5.0, numpy >= 1.20

from .load_data import (
    load_json,
    load_robot_data,
    load_envelope_data,
    load_scene_data,
    load_forest_data,
    load_snapshot,
    load_voxel_data,
    load_voxel_centres,
    RobotData,
    EnvelopeData,
    SceneData,
    ForestData,
    SnapshotData,
    VoxelData,
)
from .robot_viz import plot_robot_3d, plot_robot_multi_configs
from .envelope_viz import plot_envelope_wireframe, plot_envelope_filled
from .forest_viz import plot_forest_2d, plot_forest_3d, plot_path_on_forest
from .scene_viz import plot_scene_3d
from .combined_viz import plot_combined
from .comparison_viz import compare_envelopes, compare_planners
from .voxel_viz import plot_voxel_scatter, add_voxel_traces

__all__ = [
    # loaders
    "load_json",
    "load_robot_data",
    "load_envelope_data",
    "load_scene_data",
    "load_forest_data",
    "load_snapshot",
    "load_voxel_data",
    "load_voxel_centres",
    # data classes
    "RobotData",
    "EnvelopeData",
    "SceneData",
    "ForestData",
    "SnapshotData",
    "VoxelData",
    # viz
    "plot_robot_3d",
    "plot_robot_multi_configs",
    "plot_envelope_wireframe",
    "plot_envelope_filled",
    "plot_forest_2d",
    "plot_forest_3d",
    "plot_path_on_forest",
    "plot_scene_3d",
    "plot_combined",
    "compare_envelopes",
    "compare_planners",
    "plot_voxel_scatter",
    "add_voxel_traces",
]
