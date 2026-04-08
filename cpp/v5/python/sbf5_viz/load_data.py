"""JSON loading utilities and data classes for sbf5_viz.

Reads JSON files exported by sbf::viz (Phase I VizExporter).
v5 JSON schema:
  - robot:    {name, n_joints, link_radii, configs: [{q, link_positions}]}
  - envelope: {method, boxes: [{box_id, intervals, links: [{link_idx, aabb}]}]}
  - scene:    {obstacles: [{center, half_sizes}]}
  - forest:   {n_boxes, total_volume, boxes: [{id, intervals, volume}]}
  - snapshot: {robot, envelope, scene, forest}
"""

import json
import numpy as np
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any


# ─── Data classes ────────────────────────────────────────────────────────────

@dataclass
class RobotConfig:
    """FK result for one configuration."""
    q: np.ndarray                     # (n_joints,)
    link_positions: np.ndarray        # (n_active, 3)


@dataclass
class RobotData:
    """Robot model + FK for multiple configurations."""
    name: str
    n_joints: int
    link_radii: List[float]
    configs: List[RobotConfig]


@dataclass
class LinkIAABB:
    """One link's interval-arithmetic AABB within a C-space box."""
    link_idx: int
    lo: np.ndarray   # (3,)
    hi: np.ndarray   # (3,)


@dataclass
class EnvelopeBox:
    """Envelope data for one C-space box (all link iAABBs)."""
    box_id: int
    intervals: np.ndarray             # (n_dims, 2)
    links: List[LinkIAABB]


@dataclass
class EnvelopeData:
    """Collection of envelope boxes."""
    method: str
    boxes: List[EnvelopeBox]


@dataclass
class Obstacle:
    """Single box obstacle."""
    center: np.ndarray    # (3,)
    half_sizes: np.ndarray  # (3,)


@dataclass
class SceneData:
    """Obstacle collection."""
    obstacles: List[Obstacle]


@dataclass
class ForestBox:
    """One C-space box in the forest."""
    id: int
    intervals: np.ndarray   # (n_dims, 2)
    volume: float


@dataclass
class ForestData:
    """Forest structure (C-space boxes)."""
    n_boxes: int
    total_volume: float
    boxes: List[ForestBox]


@dataclass
class SnapshotData:
    """Combined snapshot: robot + envelope + scene + forest."""
    robot: Optional[RobotData] = None
    envelope: Optional[EnvelopeData] = None
    scene: Optional[SceneData] = None
    forest: Optional[ForestData] = None


@dataclass
class VoxelData:
    """Sparse voxel grid data from voxel_centres JSON."""
    delta: float
    n_bricks: int = 0
    total_occupied: int = 0
    centres: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))


# ─── JSON loaders ────────────────────────────────────────────────────────────

def load_json(path: str) -> dict:
    """Load a JSON file and return raw dict."""
    with open(path) as f:
        return json.load(f)


def _parse_robot(d: dict) -> RobotData:
    configs = []
    for c in d.get("configs", []):
        configs.append(RobotConfig(
            q=np.array(c["q"]),
            link_positions=np.array(c["link_positions"]),
        ))
    return RobotData(
        name=d.get("name", ""),
        n_joints=d.get("n_joints", 0),
        link_radii=d.get("link_radii", []),
        configs=configs,
    )


def _parse_envelope(d: dict) -> EnvelopeData:
    boxes = []
    for bd in d.get("boxes", []):
        links = []
        for ld in bd.get("links", []):
            aabb = ld["aabb"]   # [[lo_x, hi_x], [lo_y, hi_y], [lo_z, hi_z]]
            lo = np.array([aabb[0][0], aabb[1][0], aabb[2][0]])
            hi = np.array([aabb[0][1], aabb[1][1], aabb[2][1]])
            links.append(LinkIAABB(link_idx=ld["link_idx"], lo=lo, hi=hi))
        boxes.append(EnvelopeBox(
            box_id=bd.get("box_id", 0),
            intervals=np.array(bd.get("intervals", [])),
            links=links,
        ))
    return EnvelopeData(
        method=d.get("method", ""),
        boxes=boxes,
    )


def _parse_scene(d: dict) -> SceneData:
    obs = []
    for o in d.get("obstacles", []):
        obs.append(Obstacle(
            center=np.array(o["center"]),
            half_sizes=np.array(o["half_sizes"]),
        ))
    return SceneData(obstacles=obs)


def _parse_forest(d: dict) -> ForestData:
    boxes = []
    for bd in d.get("boxes", []):
        boxes.append(ForestBox(
            id=bd.get("id", 0),
            intervals=np.array(bd.get("intervals", [])),
            volume=bd.get("volume", 0.0),
        ))
    return ForestData(
        n_boxes=d.get("n_boxes", 0),
        total_volume=d.get("total_volume", 0.0),
        boxes=boxes,
    )


def load_robot_data(path: str) -> RobotData:
    """Load robot JSON → RobotData."""
    return _parse_robot(load_json(path))


def load_envelope_data(path: str) -> EnvelopeData:
    """Load envelope JSON → EnvelopeData."""
    return _parse_envelope(load_json(path))


def load_scene_data(path: str) -> SceneData:
    """Load scene JSON → SceneData."""
    return _parse_scene(load_json(path))


def load_forest_data(path: str) -> ForestData:
    """Load forest JSON → ForestData."""
    return _parse_forest(load_json(path))


def load_snapshot(path: str) -> SnapshotData:
    """Load snapshot JSON → SnapshotData (all layers)."""
    d = load_json(path)
    snap = SnapshotData()
    if "robot" in d:
        snap.robot = _parse_robot(d["robot"])
    if "envelope" in d:
        snap.envelope = _parse_envelope(d["envelope"])
    if "scene" in d:
        snap.scene = _parse_scene(d["scene"])
    if "forest" in d:
        snap.forest = _parse_forest(d["forest"])
    return snap


def load_voxel_data(data: dict) -> VoxelData:
    """Load VoxelData from a voxel_centres JSON dict."""
    centres = np.array(data.get("centres", []))
    if centres.ndim == 1 and len(centres) == 0:
        centres = np.empty((0, 3))
    return VoxelData(
        delta=data.get("delta", 0.02),
        n_bricks=data.get("n_bricks", 0),
        total_occupied=data.get("n_points", data.get("total_occupied", 0)),
        centres=centres,
    )


def load_voxel_centres(path: str) -> VoxelData:
    """Load a voxel_centres JSON file → VoxelData."""
    return load_voxel_data(load_json(path))
