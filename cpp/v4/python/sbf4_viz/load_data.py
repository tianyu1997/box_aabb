"""JSON loading utilities and lightweight data classes for sbf4_viz.

迁移自 v3 sbf_viz/load_data.py
v4 术语更新: sub_aabbs → link_iaabbs_sub, link_aabbs → link_iaabbs
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
    link_positions: np.ndarray        # (n_links+1, 3)


@dataclass
class RobotData:
    """Robot model + FK for multiple configurations."""
    name: str
    n_joints: int
    link_radii: np.ndarray            # (n_links,)
    active_link_map: np.ndarray       # (n_active,) int
    joint_limits: np.ndarray          # (n_joints, 2)
    configs: List[RobotConfig]


@dataclass
class IAABB:
    """Interval-arithmetic axis-aligned bounding box."""
    link: int
    lo: np.ndarray   # (3,)
    hi: np.ndarray   # (3,)
    seg: int = -1    # -1 = full link, >=0 = sub-segment index


@dataclass
class EnvelopeNode:
    """Envelope data for one C-space node."""
    node_idx: int
    box_intervals: Optional[np.ndarray]  # (n_dims, 2) or None
    link_iaabbs: List[IAABB]             # full link iAABBs
    link_iaabbs_sub: List[IAABB]         # subdivided link iAABBs


@dataclass
class EnvelopeData:
    """Collection of envelope nodes."""
    n_active_links: int
    link_radii: np.ndarray
    n_sub: int
    nodes: List[EnvelopeNode]


@dataclass
class VoxelBrick:
    """One 8³ brick with hex-encoded words."""
    coord: np.ndarray   # (3,) int
    popcount: int
    words: List[int]    # 8 × uint64


@dataclass
class VoxelData:
    """Sparse voxel grid data (brick-level)."""
    delta: float
    safety_pad: float
    n_bricks: int
    total_occupied: int
    bricks: List[VoxelBrick]


@dataclass
class VoxelCentresData:
    """Sparse voxel grid data (explicit centres)."""
    delta: float
    n_occupied: int
    centres: np.ndarray   # (N, 3)


@dataclass
class Obstacle:
    """Single box obstacle."""
    name: str
    center: np.ndarray    # (3,)
    half_sizes: np.ndarray  # (3,)


@dataclass
class SceneData:
    """Obstacle collection."""
    obstacles: List[Obstacle]


@dataclass
class SnapshotData:
    """Combined snapshot: robot + envelope + voxel + scene."""
    robot: Dict[str, Any]
    envelope: Dict[str, Any]
    robot_voxel: Dict[str, Any]
    obstacle_voxel: Dict[str, Any]
    scene: SceneData


# ─── Brick → centre reconstruction ──────────────────────────────────────────

def brick_to_centres(brick: VoxelBrick, delta: float) -> np.ndarray:
    """Reconstruct occupied voxel centres from a brick's word data."""
    pts = []
    bx, by, bz = brick.coord
    for z in range(8):
        w = brick.words[z]
        while w:
            bit = (w & -w).bit_length() - 1  # ctz
            w &= w - 1
            lx = bit % 8
            ly = bit // 8
            cx = (bx * 8 + lx + 0.5) * delta
            cy = (by * 8 + ly + 0.5) * delta
            cz = (bz * 8 + z  + 0.5) * delta
            pts.append([cx, cy, cz])
    return np.array(pts) if pts else np.empty((0, 3))


def voxel_data_to_centres(vd: VoxelData) -> np.ndarray:
    """Reconstruct all occupied centres from VoxelData."""
    all_pts = [brick_to_centres(b, vd.delta) for b in vd.bricks]
    return np.vstack(all_pts) if all_pts else np.empty((0, 3))


# ─── JSON loaders ────────────────────────────────────────────────────────────

def load_robot_data(path: str) -> RobotData:
    with open(path) as f:
        d = json.load(f)
    configs = []
    for c in d.get("configs", []):
        configs.append(RobotConfig(
            q=np.array(c["q"]),
            link_positions=np.array(c["link_positions"]),
        ))
    return RobotData(
        name=d["name"],
        n_joints=d["n_joints"],
        link_radii=np.array(d.get("link_radii", [])),
        active_link_map=np.array(d.get("active_link_map", []), dtype=int),
        joint_limits=np.array(d.get("joint_limits", [])),
        configs=configs,
    )


def load_envelope_data(path: str) -> EnvelopeData:
    with open(path) as f:
        d = json.load(f)

    nodes = []
    for nd in d.get("nodes", []):
        la = [IAABB(link=a["link"], lo=np.array(a["lo"]), hi=np.array(a["hi"]))
              for a in nd.get("link_iaabbs", [])]
        sa = [IAABB(link=a["link"], lo=np.array(a["lo"]), hi=np.array(a["hi"]),
                    seg=a.get("seg", -1))
              for a in nd.get("link_iaabbs_sub", [])]
        bi = np.array(nd["box_intervals"]) if "box_intervals" in nd else None
        nodes.append(EnvelopeNode(
            node_idx=nd["node_idx"],
            box_intervals=bi,
            link_iaabbs=la,
            link_iaabbs_sub=sa,
        ))

    return EnvelopeData(
        n_active_links=d["n_active_links"],
        link_radii=np.array(d["link_radii"]),
        n_sub=d["n_sub"],
        nodes=nodes,
    )


def load_voxel_data(path: str) -> VoxelData:
    with open(path) as f:
        d = json.load(f)
    bricks = []
    for b in d.get("bricks", []):
        bricks.append(VoxelBrick(
            coord=np.array(b["coord"], dtype=int),
            popcount=b["popcount"],
            words=[int(w, 16) for w in b["words"]],
        ))
    return VoxelData(
        delta=d["delta"],
        safety_pad=d.get("safety_pad", 0.0),
        n_bricks=d["n_bricks"],
        total_occupied=d["total_occupied"],
        bricks=bricks,
    )


def load_voxel_centres_data(path: str) -> VoxelCentresData:
    with open(path) as f:
        d = json.load(f)
    return VoxelCentresData(
        delta=d["delta"],
        n_occupied=d["n_occupied"],
        centres=np.array(d["centres"]) if d["centres"] else np.empty((0, 3)),
    )


def load_scene_data(path: str) -> SceneData:
    with open(path) as f:
        d = json.load(f)
    obs = [Obstacle(name=o["name"],
                    center=np.array(o["center"]),
                    half_sizes=np.array(o["half_sizes"]))
           for o in d.get("obstacles", [])]
    return SceneData(obstacles=obs)


def load_snapshot_data(path: str) -> SnapshotData:
    with open(path) as f:
        d = json.load(f)

    # Parse scene
    sobs = [Obstacle(name=o["name"],
                     center=np.array(o["center"]),
                     half_sizes=np.array(o["half_sizes"]))
            for o in d.get("scene", {}).get("obstacles", [])]

    return SnapshotData(
        robot=d.get("robot", {}),
        envelope=d.get("envelope", {}),
        robot_voxel=d.get("robot_voxel", {}),
        obstacle_voxel=d.get("obstacle_voxel", {}),
        scene=SceneData(obstacles=sobs),
    )
