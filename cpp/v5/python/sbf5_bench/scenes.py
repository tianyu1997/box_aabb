"""
sbf5_bench/scenes.py — Standard benchmark scenes

Defines reusable test scenarios (robot + obstacles + start/goal)
for reproducible experiments.
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np

# Default data directory (relative to this package → ../../data)
_PACKAGE_DIR = os.path.dirname(os.path.abspath(__file__))
DATA_DIR = os.path.normpath(os.path.join(_PACKAGE_DIR, "..", "..", "data"))


@dataclass
class BenchmarkScene:
    """A complete benchmark scenario definition."""

    name: str
    robot_json: str               # relative to DATA_DIR or absolute path
    obstacles: List[Dict]         # [{"center": [...], "half_sizes": [...]}, ...]
    start: np.ndarray
    goal: np.ndarray
    description: str = ""

    def get_robot_path(self) -> str:
        """Resolve robot JSON to absolute path."""
        if os.path.isabs(self.robot_json):
            return self.robot_json
        return os.path.join(DATA_DIR, self.robot_json)

    def make_obstacles(self):
        """Convert obstacle dicts to sbf5.Obstacle list."""
        import sbf5
        obs_list = []
        for o in self.obstacles:
            if "center" in o and "half_sizes" in o:
                c = np.asarray(o["center"], dtype=np.float64)
                h = np.asarray(o["half_sizes"], dtype=np.float64)
                obs_list.append(sbf5.Obstacle(
                    float(c[0] - h[0]), float(c[1] - h[1]), float(c[2] - h[2]),
                    float(c[0] + h[0]), float(c[1] + h[1]), float(c[2] + h[2]),
                ))
            elif "lo" in o and "hi" in o:
                lo = o["lo"]
                hi = o["hi"]
                obs_list.append(sbf5.Obstacle(
                    float(lo[0]), float(lo[1]), float(lo[2]),
                    float(hi[0]), float(hi[1]), float(hi[2]),
                ))
        return obs_list

    def make_robot(self):
        """Load sbf5.Robot from the scene's robot_json."""
        import sbf5
        return sbf5.Robot.from_json(self.get_robot_path())


# ──────────────────────────────────────────────────────────────
# Built-in scenes
# ──────────────────────────────────────────────────────────────

SCENES: Dict[str, BenchmarkScene] = {
    "2dof_simple": BenchmarkScene(
        name="2dof_simple",
        robot_json="2dof_planar.json",
        obstacles=[
            {"center": [0.0, 1.5, 0.0], "half_sizes": [0.3, 0.15, 0.3]},
        ],
        start=np.array([0.5, 0.5]),
        goal=np.array([2.0, 1.0]),
        description="2DOF planar arm, single obstacle above workspace",
    ),
    "2dof_narrow": BenchmarkScene(
        name="2dof_narrow",
        robot_json="2dof_planar.json",
        obstacles=[
            {"center": [0.0, 1.8, 0.0], "half_sizes": [0.8, 0.1, 0.8]},
            {"center": [0.0, 1.2, 0.0], "half_sizes": [0.8, 0.1, 0.8]},
        ],
        start=np.array([0.5, 0.3]),
        goal=np.array([2.5, 1.5]),
        description="2DOF narrow passage between two walls",
    ),
    "panda_tabletop": BenchmarkScene(
        name="panda_tabletop",
        robot_json="panda.json",
        obstacles=[
            {"center": [0.4, 0.0, 0.4], "half_sizes": [0.15, 0.15, 0.02]},
        ],
        start=np.array([0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0]),
        goal=np.array([0.5, -0.3, 0.0, -1.5, 0.0, 1.2, 0.0]),
        description="7DOF Panda with near-field shelf obstacle",
    ),
    "2dof_cluttered": BenchmarkScene(
        name="2dof_cluttered",
        robot_json="2dof_planar.json",
        obstacles=[
            {"center": [0.0, 1.5, 0.0], "half_sizes": [0.15, 0.15, 0.15]},
            {"center": [-0.8, 1.0, 0.0], "half_sizes": [0.12, 0.12, 0.12]},
        ],
        start=np.array([0.5, 0.3]),
        goal=np.array([2.5, 1.5]),
        description="2DOF cluttered: 2 obstacles in workspace",
    ),
    "panda_shelf": BenchmarkScene(
        name="panda_shelf",
        robot_json="panda.json",
        obstacles=[
            {"center": [0.35, 0.0, 0.55], "half_sizes": [0.25, 0.30, 0.02]},
        ],
        start=np.array([0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0]),
        goal=np.array([0.8, 0.5, 0.0, -1.0, 0.0, 1.5, 0.5]),
        description="7DOF Panda reaching under a shelf",
    ),
    "panda_multi_obstacle": BenchmarkScene(
        name="panda_multi_obstacle",
        robot_json="panda.json",
        obstacles=[
            {"center": [0.4, 0.0, 0.4], "half_sizes": [0.15, 0.15, 0.02]},
            {"center": [0.3, 0.2, 0.35], "half_sizes": [0.08, 0.08, 0.08]},
        ],
        start=np.array([0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0]),
        goal=np.array([1.0, -0.5, 0.0, -2.0, 0.0, 1.8, 0.0]),
        description="7DOF Panda with two near-field obstacles",
    ),
}


# ──────────────────────────────────────────────────────────────
# Marcucci Science Robotics scenes (iiwa14)
# ──────────────────────────────────────────────────────────────

def _iiwa_config(vals):
    """Helper: create 7-DOF config array."""
    return np.array(vals, dtype=np.float64)


# Predefined iiwa14 configurations
_IIWA_C  = _iiwa_config([0.0, 0.2, 0.0, -2.09, 0.0, -0.3, np.pi / 2])
_IIWA_AS = _iiwa_config([6.42e-05, 0.4719533, -0.0001493, -0.6716735, 0.0001854, 0.4261696, 1.5706922])
_IIWA_TS = _iiwa_config([-1.55e-04, 0.3972726, 0.0002196, -1.3674756, 0.0002472, -0.1929518, 1.5704688])
_IIWA_CS = _iiwa_config([-1.76e-04, 0.6830279, 0.0002450, -1.6478229, 2.09e-05, -0.7590545, 1.5706263])
_IIWA_LB = _iiwa_config([1.3326656, 0.7865932, 0.3623384, -1.4916529, -0.3192509, 0.9217325, 1.7911904])
_IIWA_RB = _iiwa_config([-1.3324624, 0.7866478, -0.3626562, -1.4916528, 0.3195340, 0.9217833, 1.3502090])


def _make_shelves_obstacles():
    """Shelves: 5 obstacles at origin (0.85, 0, 0.4)."""
    ox, oy, oz = 0.85, 0.0, 0.4
    obs = []
    def add(lx, ly, lz, fx, fy, fz):
        obs.append({"center": [ox+lx, oy+ly, oz+lz], "half_sizes": [fx/2, fy/2, fz/2]})
    add( 0,  0.292,  0,       0.3,  0.016, 0.783)
    add( 0, -0.292,  0,       0.3,  0.016, 0.783)
    add( 0,  0,      0.3995,  0.3,  0.6,   0.016)
    add( 0,  0,     -0.13115, 0.3,  0.6,   0.016)
    add( 0,  0,      0.13115, 0.3,  0.6,   0.016)
    return obs


def _make_bins_obstacles():
    """Bins: 10 obstacles (2 bins × 5 faces), yaw=90°."""
    obs = []
    def add_bin(bx, by, bz):
        def add(lx, ly, lz, fx, fy, fz):
            # yaw=90°: body(lx,ly,lz)→world(-ly,lx,lz), size(fx,fy,fz)→world(fy,fx,fz)
            obs.append({"center": [bx-ly, by+lx, bz+lz], "half_sizes": [fy/2, fx/2, fz/2]})
        add( 0.22,  0,     0.105,  0.05, 0.63, 0.21)
        add(-0.22,  0,     0.105,  0.05, 0.63, 0.21)
        add( 0,     0.29,  0.105,  0.49, 0.05, 0.21)
        add( 0,    -0.29,  0.105,  0.49, 0.05, 0.21)
        add( 0,     0,     0.0075, 0.49, 0.63, 0.015)
    add_bin(0, -0.6, 0)
    add_bin(0,  0.6, 0)
    return obs


def _make_table_obstacles():
    """Table: 1 obstacle at (0.4, 0, -0.25), size 2.5×2.5×0.2."""
    return [{"center": [0.4, 0.0, -0.25], "half_sizes": [1.25, 1.25, 0.1]}]


def _make_combined_obstacles():
    """Combined: shelves(5) + bins(10) + table(1) = 16 obstacles."""
    return _make_shelves_obstacles() + _make_bins_obstacles() + _make_table_obstacles()


# 5 canonical query pairs: AS→TS→CS→LB→RB→AS
IIWA_COMBINED_QUERIES = [
    ("AS->TS", _IIWA_AS, _IIWA_TS),
    ("TS->CS", _IIWA_TS, _IIWA_CS),
    ("CS->LB", _IIWA_CS, _IIWA_LB),
    ("LB->RB", _IIWA_LB, _IIWA_RB),
    ("RB->AS", _IIWA_RB, _IIWA_AS),
]

# Register combined scene (using first query pair AS→TS as default)
SCENES["iiwa_combined"] = BenchmarkScene(
    name="iiwa_combined",
    robot_json="iiwa14.json",
    obstacles=_make_combined_obstacles(),
    start=_IIWA_AS,
    goal=_IIWA_TS,
    description="IIWA14 combined Marcucci scene (16 obstacles, 5 query pairs)",
)


def get_scene(name: str) -> BenchmarkScene:
    """Retrieve a built-in scene by name."""
    if name not in SCENES:
        raise KeyError(
            f"Unknown scene '{name}'. Available: {list(SCENES.keys())}")
    return SCENES[name]


def list_scenes() -> List[str]:
    """List all available scene names."""
    return list(SCENES.keys())
