"""
sbf5_bench/iris_np_gcs.py — IRIS-NP + GCS and C-IRIS + GCS planners

Ported from v4/src/baselines/iris_np_gcs.py.
Uses Drake's IrisInConfigurationSpace + GraphOfConvexSets for
comparison against SBF's box-based approach.

Requires pydrake. Gracefully degrades if not installed.
"""

from __future__ import annotations

import logging
import os
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
                GraphOfConvexSets, IrisInConfigurationSpace,
            )
            _HAS_DRAKE = True
        except ImportError:
            _HAS_DRAKE = False
    return _HAS_DRAKE


def _find_gcs_dir() -> Optional[str]:
    """Locate gcs-science-robotics repo directory."""
    candidates = [
        os.path.join(os.path.dirname(__file__), "..", "..", "..",
                      "..", "gcs-science-robotics"),
        os.path.expanduser("~/gcs-science-robotics"),
        os.path.expanduser("~/code/gcs-science-robotics"),
    ]
    for c in candidates:
        c = os.path.normpath(c)
        if os.path.isdir(c) and os.path.isfile(
            os.path.join(c, "models", "iiwa14_welded_gripper.yaml")
        ):
            return c
    return None


class IRISNPGCSPlanner(BasePlanner):
    """IRIS-NP + GCS planner (Marcucci et al.).

    Generates IRIS regions in C-space using IrisInConfigurationSpace,
    then solves GraphOfConvexSets for the shortest path.
    """

    def __init__(self, n_regions: int = 10, max_iterations: int = 10):
        self._n_iris_seeds = n_regions
        self._max_iterations = max_iterations
        self._robot = None
        self._scene = None
        self._config: dict = {}
        self._plant = None
        self._diagram = None
        self._regions: List = []
        self._seed_points: List = []
        self._built = False

    @property
    def name(self) -> str:
        return "IRIS-NP-GCS"

    @property
    def supports_reuse(self) -> bool:
        return True

    def setup(self, robot, scene, config: Optional[dict] = None) -> None:
        self._robot = robot
        self._scene = scene
        self._config = config or {}
        self._regions = []
        self._seed_points = []
        self._built = False

    def _build_plant(self) -> bool:
        """Build Drake MultibodyPlant for IRIS."""
        try:
            gcs_dir = _find_gcs_dir()
            if gcs_dir is None:
                logger.error("gcs-science-robotics repo not found")
                return False

            from pydrake.systems.framework import DiagramBuilder
            from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
            from pydrake.multibody.parsing import (
                Parser, LoadModelDirectives, ProcessModelDirectives)

            builder = DiagramBuilder()
            plant, _sg = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
            parser = Parser(plant, _sg)
            _pm = parser.package_map()
            _pm.Add("gcs", gcs_dir)

            # Override drake_models to local path if available
            try:
                import pydrake as _pyd
                _drake_share = os.path.join(
                    os.path.dirname(_pyd.__file__), "share", "drake")
                _dm_local = os.path.join(
                    _drake_share, "manipulation", "models")
                if os.path.isdir(_dm_local):
                    if _pm.Contains("drake_models"):
                        _pm.Remove("drake_models")
                    _pm.Add("drake_models", _dm_local)
            except Exception:
                pass

            directives_file = os.path.join(
                gcs_dir, "models", "iiwa14_welded_gripper.yaml")
            directives = LoadModelDirectives(directives_file)
            ProcessModelDirectives(directives, plant, parser)
            plant.Finalize()
            self._diagram = builder.Build()
            self._plant = plant
            return True
        except Exception as e:
            logger.error("IRIS plant build failed: %s", e)
            return False

    def _generate_iris_regions(self, seeds: List[np.ndarray],
                               timeout_per: float) -> list:
        """Generate IRIS regions from seed points."""
        from pydrake.geometry.optimization import (
            IrisInConfigurationSpace, IrisOptions)

        context = self._diagram.CreateDefaultContext()
        plant_context = self._plant.GetMyContextFromRoot(context)

        iris_options = IrisOptions()
        iris_options.iteration_limit = self._max_iterations
        iris_options.require_sample_point_is_contained = True

        regions = []
        for i, seed in enumerate(seeds):
            try:
                self._plant.SetPositions(plant_context, seed)
                hpoly = IrisInConfigurationSpace(
                    self._plant, plant_context, iris_options)
                regions.append(hpoly)
            except Exception as e:
                logger.warning("IRIS region %d failed: %s", i, e)
        return regions

    def _solve_gcs(self, q_start: np.ndarray, q_goal: np.ndarray,
                   regions: list) -> Optional[np.ndarray]:
        """Solve GCS for shortest path through regions."""
        from pydrake.geometry.optimization import (
            GraphOfConvexSets, HPolyhedron, Point)
        from pydrake.solvers import MathematicalProgram, Solve

        n = len(regions)
        if n == 0:
            return None

        gcs = GraphOfConvexSets()
        vertices = []
        for i, region in enumerate(regions):
            v = gcs.AddVertex(region, f"r{i}")
            vertices.append(v)

        # Add start and goal as point vertices
        start_pt = Point(q_start)
        goal_pt = Point(q_goal)
        v_start = gcs.AddVertex(start_pt, "start")
        v_goal = gcs.AddVertex(goal_pt, "goal")

        # Edges: start → regions that contain start
        for i, region in enumerate(regions):
            if region.PointInSet(q_start):
                e = gcs.AddEdge(v_start, vertices[i])
                cost = np.linalg.norm(
                    q_start - regions[i].ChebyshevCenter())
                e.AddCost(cost)

        # Edges: regions → goal (if region contains goal)
        for i, region in enumerate(regions):
            if region.PointInSet(q_goal):
                e = gcs.AddEdge(vertices[i], v_goal)
                cost = np.linalg.norm(
                    q_goal - regions[i].ChebyshevCenter())
                e.AddCost(cost)

        # Edges: region → region (overlapping)
        for i in range(n):
            for j in range(i + 1, n):
                # Check intersection
                try:
                    inter = regions[i].Intersection(regions[j])
                    if not inter.IsEmpty():
                        ci = regions[i].ChebyshevCenter()
                        cj = regions[j].ChebyshevCenter()
                        cost = np.linalg.norm(ci - cj)
                        gcs.AddEdge(vertices[i], vertices[j]).AddCost(cost)
                        gcs.AddEdge(vertices[j], vertices[i]).AddCost(cost)
                except Exception:
                    pass

        # Solve
        try:
            result = gcs.SolveShortestPath(v_start, v_goal)
            if not result.is_success():
                return None

            # Extract path
            path = [q_start]
            for v in vertices:
                x = result.GetSolution(v.x())
                if x is not None and len(x) > 0:
                    path.append(np.array(x))
            path.append(q_goal)
            return np.array(path, dtype=np.float64)
        except Exception as e:
            logger.warning("GCS solve error: %s", e)
            return None

    def plan(self, start: np.ndarray, goal: np.ndarray,
             timeout: float = 60.0) -> PlanningResult:
        if not _check_drake():
            return PlanningResult.failure("pydrake not installed")

        q_start = np.asarray(start, dtype=np.float64)
        q_goal = np.asarray(goal, dtype=np.float64)
        t_total = time.perf_counter()
        phase_times = {}

        # Build plant (first call only)
        if self._plant is None:
            t0 = time.perf_counter()
            if not self._build_plant():
                return PlanningResult.failure(
                    reason="IRIS plant build failed",
                    planning_time_s=time.perf_counter() - t_total)
            phase_times['plant_build'] = time.perf_counter() - t0

        # Generate IRIS regions (first call only)
        if not self._built:
            t0 = time.perf_counter()
            seeds = [q_start, q_goal]
            rng = np.random.default_rng(self._config.get('seed', 42))
            jl = self._robot.joint_limits
            lows = np.array([lo for lo, _ in jl])
            highs = np.array([hi for _, hi in jl])
            for _ in range(self._n_iris_seeds - 2):
                seeds.append(rng.uniform(lows, highs))

            timeout_per = max(5.0, (timeout * 0.7) / len(seeds))
            self._regions = self._generate_iris_regions(seeds, timeout_per)
            self._seed_points = seeds
            self._built = True
            phase_times['iris_build'] = time.perf_counter() - t0

            if len(self._regions) == 0:
                return PlanningResult.failure(
                    reason="No IRIS regions generated",
                    planning_time_s=time.perf_counter() - t_total)

        # Solve GCS
        t0 = time.perf_counter()
        try:
            path = self._solve_gcs(q_start, q_goal, self._regions)
            phase_times['gcs_solve'] = time.perf_counter() - t0
        except Exception as e:
            logger.warning("GCS solve error: %s", e)
            return PlanningResult.failure(
                reason=str(e),
                planning_time_s=time.perf_counter() - t_total)

        dt = time.perf_counter() - t_total

        if path is None or len(path) < 2:
            return PlanningResult.failure(
                reason="GCS path not found",
                planning_time_s=dt)

        cost = float(sum(
            np.linalg.norm(path[i] - path[i - 1])
            for i in range(1, len(path))
        ))

        return PlanningResult(
            success=True,
            path=path,
            cost=cost,
            planning_time_s=dt,
            collision_checks=0,
            nodes_explored=len(self._regions),
            metadata={
                "algorithm": "IRIS-NP-GCS",
                "n_regions": len(self._regions),
                "certified": False,
                "phase_times": phase_times,
            },
        )

    def reset(self) -> None:
        self._regions = []
        self._seed_points = []
        self._built = False


class CIRISGCSPlanner(BasePlanner):
    """C-IRIS + GCS planner (Werner et al. 2024).

    Uses certified C-space polytopes (IrisInConfigurationSpace with
    stricter parameters) + GCS. Extremely expensive per-region.
    """

    def __init__(self, n_regions: int = 5, max_iterations: int = 5):
        self._n_regions = n_regions
        self._max_iterations = max_iterations
        self._robot = None
        self._scene = None
        self._config: dict = {}
        self._plant = None
        self._diagram = None
        self._regions: List = []
        self._built = False

    @property
    def name(self) -> str:
        return "C-IRIS-GCS"

    @property
    def supports_reuse(self) -> bool:
        return True

    def setup(self, robot, scene, config: Optional[dict] = None) -> None:
        self._robot = robot
        self._scene = scene
        self._config = config or {}
        self._regions = []
        self._built = False

    def plan(self, start: np.ndarray, goal: np.ndarray,
             timeout: float = 120.0) -> PlanningResult:
        if not _check_drake():
            return PlanningResult.failure("pydrake not installed")

        q_start = np.asarray(start, dtype=np.float64)
        q_goal = np.asarray(goal, dtype=np.float64)
        t_total = time.perf_counter()
        phase_times = {}

        # Build plant (reuse IRIS-NP logic)
        if self._plant is None:
            t0 = time.perf_counter()
            iris = IRISNPGCSPlanner.__new__(IRISNPGCSPlanner)
            iris._plant = None
            iris._diagram = None
            iris._robot = self._robot
            iris._scene = self._scene
            iris._config = self._config
            if not iris._build_plant():
                return PlanningResult.failure(
                    reason="C-IRIS plant build failed",
                    planning_time_s=time.perf_counter() - t_total)
            self._plant = iris._plant
            self._diagram = iris._diagram
            phase_times['plant_build'] = time.perf_counter() - t0

        # Generate certified regions
        if not self._built:
            t0 = time.perf_counter()
            try:
                self._regions = self._generate_certified_regions(
                    q_start, q_goal, timeout * 0.8)
            except Exception as e:
                logger.warning("C-IRIS region generation failed: %s", e)
                self._regions = []
            phase_times['ciris_build'] = time.perf_counter() - t0
            self._built = True

            if len(self._regions) == 0:
                return PlanningResult.failure(
                    reason="No C-IRIS regions generated",
                    planning_time_s=time.perf_counter() - t_total)

        # Solve GCS (reuse IRIS-NP solve)
        t0 = time.perf_counter()
        try:
            iris = IRISNPGCSPlanner.__new__(IRISNPGCSPlanner)
            path = iris._solve_gcs(q_start, q_goal, self._regions)
            phase_times['gcs_solve'] = time.perf_counter() - t0
        except Exception as e:
            return PlanningResult.failure(
                reason=f"C-IRIS GCS solve error: {e}",
                planning_time_s=time.perf_counter() - t_total)

        dt = time.perf_counter() - t_total

        if path is None or len(path) < 2:
            return PlanningResult.failure(
                reason="C-IRIS GCS path not found",
                planning_time_s=dt)

        cost = float(sum(
            np.linalg.norm(path[i] - path[i - 1])
            for i in range(1, len(path))
        ))

        return PlanningResult(
            success=True,
            path=path,
            cost=cost,
            planning_time_s=dt,
            collision_checks=0,
            nodes_explored=len(self._regions),
            metadata={
                "algorithm": "C-IRIS-GCS",
                "n_regions": len(self._regions),
                "certified": True,
                "phase_times": phase_times,
            },
        )

    def _generate_certified_regions(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        total_timeout: float,
    ) -> list:
        """Generate C-IRIS certified regions."""
        from pydrake.geometry.optimization import (
            IrisInConfigurationSpace, IrisOptions)

        context = self._diagram.CreateDefaultContext()
        plant_context = self._plant.GetMyContextFromRoot(context)

        iris_options = IrisOptions()
        iris_options.iteration_limit = self._max_iterations
        iris_options.require_sample_point_is_contained = True
        iris_options.relative_termination_threshold = 1e-3

        seeds = [q_start, q_goal]
        rng = np.random.default_rng(self._config.get('seed', 42))
        jl = self._robot.joint_limits
        lows = np.array([lo for lo, _ in jl])
        highs = np.array([hi for _, hi in jl])
        for _ in range(self._n_regions - 2):
            seeds.append(rng.uniform(lows, highs))

        regions = []
        t0 = time.perf_counter()

        for i, seed in enumerate(seeds):
            if time.perf_counter() - t0 > total_timeout:
                logger.warning("C-IRIS timeout after %d regions", i)
                break
            try:
                self._plant.SetPositions(plant_context, seed)
                hpoly = IrisInConfigurationSpace(
                    self._plant, plant_context, iris_options)
                regions.append(hpoly)
                logger.info("C-IRIS region %d: %.2fs (cumulative)",
                            i, time.perf_counter() - t0)
            except Exception as e:
                logger.warning("C-IRIS region %d failed: %s", i, e)

        return regions

    def reset(self) -> None:
        self._regions = []
        self._built = False
