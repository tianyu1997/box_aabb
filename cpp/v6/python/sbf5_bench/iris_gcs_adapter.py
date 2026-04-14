"""
sbf5_bench/iris_gcs_adapter.py — Drake IRIS + GCS baseline adapter

Uses Drake's IrisInConfigurationSpace + GraphOfConvexSets for
comparison against SBF's box-based approach.

Requires pydrake. Gracefully degrades if not installed.

Migrated from: v3/src/baselines/iris_gcs.py
"""

from __future__ import annotations

import logging
import time
from typing import List, Optional

import numpy as np

from .base import BasePlanner, PlanningResult

logger = logging.getLogger(__name__)

_HAS_DRAKE: Optional[bool] = None


def _check_drake() -> bool:
    global _HAS_DRAKE
    if _HAS_DRAKE is None:
        try:
            from pydrake.geometry.optimization import (  # noqa: F401
                GraphOfConvexSets,
            )
            _HAS_DRAKE = True
        except ImportError:
            _HAS_DRAKE = False
    return _HAS_DRAKE


class IRISGCSPlanner(BasePlanner):
    """Drake IRIS + GCS planner.

    Uses SBF v5 planner to build the box forest, then runs
    Drake GCS for optimal path through the convex decomposition.

    If pydrake is not available, plan() returns failure.
    """

    def __init__(self):
        self._robot = None
        self._obstacles = None
        self._config: dict = {}
        self._planner = None
        self._forest_built = False

    @property
    def name(self) -> str:
        return "IRIS-GCS"

    @property
    def supports_reuse(self) -> bool:
        return True  # forest / IRIS regions cached across queries

    def setup(self, robot, scene, config: Optional[dict] = None) -> None:
        self._robot = robot
        self._obstacles = scene
        self._config = config or {}
        self._planner = None
        self._forest_built = False

    def plan(self, start: np.ndarray, goal: np.ndarray,
             timeout: float = 30.0) -> PlanningResult:
        if not _check_drake():
            logger.error("pydrake not available — IRIS-GCS cannot run")
            return PlanningResult.failure("pydrake not installed")

        import sbf5

        q_start = np.asarray(start, dtype=np.float64)
        q_goal = np.asarray(goal, dtype=np.float64)
        t_total = time.perf_counter()

        # Build SBF forest (first call only)
        if self._planner is None:
            cfg = sbf5.SBFPlannerConfig()
            cfg.use_gcs = True
            if "max_boxes" in self._config:
                cfg.grower.max_boxes = self._config["max_boxes"]
            self._planner = sbf5.SBFPlanner(self._robot, cfg)

        t0 = time.perf_counter()
        result = self._planner.plan(
            q_start, q_goal,
            self._obstacles,
            timeout_ms=timeout * 1000,
        )
        dt = time.perf_counter() - t_total

        if not result.success:
            return PlanningResult.failure("IRIS-GCS solve failed", dt)

        path_list = result.path
        path_arr = (np.array(path_list, dtype=np.float64)
                    if path_list else None)

        return PlanningResult(
            success=True,
            path=path_arr,
            cost=result.path_length,
            planning_time_s=dt,
            nodes_explored=result.n_boxes,
            metadata={
                "algorithm": "IRIS-GCS",
                "n_boxes": result.n_boxes,
                "planning_time_ms_cpp": result.planning_time_ms,
                "build_time_ms": result.build_time_ms,
                "lect_time_ms": getattr(result, "lect_time_ms", 0.0),
            },
        )

    def reset(self) -> None:
        if self._planner is not None:
            self._planner.clear_forest()
        self._forest_built = False
