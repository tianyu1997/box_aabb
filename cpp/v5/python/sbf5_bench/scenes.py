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


def get_scene(name: str) -> BenchmarkScene:
    """Retrieve a built-in scene by name."""
    if name not in SCENES:
        raise KeyError(
            f"Unknown scene '{name}'. Available: {list(SCENES.keys())}")
    return SCENES[name]


def list_scenes() -> List[str]:
    """List all available scene names."""
    return list(SCENES.keys())
