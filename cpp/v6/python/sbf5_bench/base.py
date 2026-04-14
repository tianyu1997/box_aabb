"""
sbf5_bench/base.py — BasePlanner ABC + PlanningResult

Unified interface for all planners (SBF, OMPL, IRIS-GCS, etc.)
and a standard result format for metric evaluation.

Migrated from: v3/src/baselines/base.py
"""

from __future__ import annotations

import abc
from dataclasses import dataclass, field
from typing import Dict, List, Optional

import numpy as np


@dataclass
class PlanningResult:
    """Unified result format for all planning methods."""

    success: bool = False
    path: Optional[np.ndarray] = None  # (N, DOF) waypoints
    cost: float = 0.0                  # path length (L2 joint space)
    planning_time_s: float = 0.0       # wall-clock seconds
    collision_checks: int = 0
    nodes_explored: int = 0
    metadata: Dict = field(default_factory=dict)

    @staticmethod
    def failure(reason: str = "",
                planning_time_s: float = 0.0) -> "PlanningResult":
        """Convenience constructor for failed results."""
        return PlanningResult(
            success=False,
            cost=float("inf"),
            planning_time_s=planning_time_s,
            metadata={"reason": reason} if reason else {},
        )

    @property
    def n_waypoints(self) -> int:
        if self.path is None:
            return 0
        return self.path.shape[0]

    def to_dict(self) -> dict:
        """Serialize to plain dict (JSON-safe if path converted)."""
        d = {
            "success": self.success,
            "cost": self.cost,
            "planning_time_s": self.planning_time_s,
            "n_waypoints": self.n_waypoints,
            "collision_checks": self.collision_checks,
            "nodes_explored": self.nodes_explored,
        }
        d.update(self.metadata)
        return d


class BasePlanner(abc.ABC):
    """Abstract base for all planners.

    Lifecycle::

        planner = SomePlanner()
        planner.setup(robot, scene, config)
        r1 = planner.plan(start, goal)
        r2 = planner.plan(start2, goal2)   # if supports_reuse
        planner.reset()
    """

    @abc.abstractmethod
    def setup(self, robot, scene, config: Optional[dict] = None) -> None:
        """Initialize planner with robot + scene + optional config."""

    @abc.abstractmethod
    def plan(self, start: np.ndarray, goal: np.ndarray,
             timeout: float = 30.0) -> PlanningResult:
        """Execute planning from start to goal."""

    def reset(self) -> None:
        """Reset internal state (default: no-op)."""

    @property
    @abc.abstractmethod
    def name(self) -> str:
        """Unique planner name for reports."""

    @property
    def supports_reuse(self) -> bool:
        """Whether successive plan() calls can reuse internal structures."""
        return False
