"""
sbf5_bench/sbf_adapter.py — SBF v5 → BasePlanner adapter

Wraps the C++ SBFPlanner (via pybind11 bindings) into the
unified BasePlanner interface for benchmarking.
"""

from __future__ import annotations

import time
from typing import Optional

import numpy as np

from .base import BasePlanner, PlanningResult


class SBFPlannerAdapter(BasePlanner):
    """Adapter for SBF v5 C++ planner."""

    def __init__(
        self,
        endpoint_source: str = "IFK",
        envelope_type: str = "LinkIAABB",
        use_gcs: bool = False,
        gcpc_cache_path: Optional[str] = None,
        extra_config: Optional[dict] = None,
    ):
        self._endpoint_source = endpoint_source
        self._envelope_type = envelope_type
        self._use_gcs = use_gcs
        self._gcpc_cache_path = gcpc_cache_path
        self._extra_config = extra_config or {}
        self._planner = None
        self._robot = None
        self._obstacles = None
        self._gcpc_cache = None

    @property
    def name(self) -> str:
        gcs_tag = "-GCS" if self._use_gcs else ""
        return f"SBF-{self._endpoint_source}-{self._envelope_type}{gcs_tag}"

    @property
    def supports_reuse(self) -> bool:
        return True  # build() + query() pattern

    def setup(self, robot, scene, config: Optional[dict] = None) -> None:
        import sbf5

        self._robot = robot
        self._obstacles = scene  # list of Obstacle

        cfg = sbf5.SBFPlannerConfig()
        cfg.use_gcs = self._use_gcs

        # Endpoint source
        src_map = {
            "IFK":        sbf5.EndpointSource.IFK,
            "CritSample": sbf5.EndpointSource.CritSample,
            "Analytical": sbf5.EndpointSource.Analytical,
            "GCPC":       sbf5.EndpointSource.GCPC,
        }
        cfg.endpoint_source.source = src_map[self._endpoint_source]

        # Envelope type
        env_map = {
            "LinkIAABB":      sbf5.EnvelopeType.LinkIAABB,
            "LinkIAABB_Grid": sbf5.EnvelopeType.LinkIAABB_Grid,
            "Hull16_Grid":    sbf5.EnvelopeType.Hull16_Grid,
        }
        cfg.envelope_type.type = env_map[self._envelope_type]

        # Merge extra_config and per-call config
        c = {**self._extra_config, **(config or {})}
        if "max_boxes" in c:
            cfg.grower.max_boxes = c["max_boxes"]
        if "n_roots" in c:
            cfg.grower.max_boxes = c["n_roots"]
        if "timeout_ms" in c:
            cfg.grower.timeout_ms = c["timeout_ms"]
        if "rrt_goal_bias" in c:
            cfg.grower.rrt_goal_bias = c["rrt_goal_bias"]
        if "rng_seed" in c:
            cfg.grower.rng_seed = c["rng_seed"]
        if "mode" in c:
            cfg.grower.mode = c["mode"]
        if "target_boxes" in c:
            cfg.coarsen.target_boxes = c["target_boxes"]
        if "shortcut_max_iters" in c:
            cfg.smoother.shortcut_max_iters = c["shortcut_max_iters"]
        if "n_subdivisions" in c:
            cfg.envelope_type.n_subdivisions = c["n_subdivisions"]
        if "n_samples_crit" in c:
            cfg.endpoint_source.n_samples_crit = c["n_samples_crit"]
        if "max_phase_analytical" in c:
            cfg.endpoint_source.max_phase_analytical = c["max_phase_analytical"]

        self._planner = sbf5.SBFPlanner(robot, cfg)

        # GCPC cache — must load before planner uses it
        if self._endpoint_source == "GCPC" and self._gcpc_cache_path:
            self._gcpc_cache = sbf5.GcpcCache.load(self._gcpc_cache_path)
            cfg.endpoint_source.set_gcpc_cache(self._gcpc_cache)
            # Recreate planner with updated config
            self._planner = sbf5.SBFPlanner(robot, cfg)

    def plan(self, start: np.ndarray, goal: np.ndarray,
             timeout: float = 30.0) -> PlanningResult:
        if self._planner is None:
            return PlanningResult.failure("planner not set up")

        q_start = np.asarray(start, dtype=np.float64)
        q_goal = np.asarray(goal, dtype=np.float64)

        t0 = time.perf_counter()
        result = self._planner.plan(
            q_start, q_goal,
            self._obstacles,
            timeout_ms=timeout * 1000,
        )
        dt = time.perf_counter() - t0

        if not result.success:
            return PlanningResult.failure(
                reason="SBF plan failed",
                planning_time_s=dt,
            )

        path_list = result.path  # list of numpy arrays
        path_arr = np.array(path_list, dtype=np.float64) if path_list else None

        return PlanningResult(
            success=True,
            path=path_arr,
            cost=result.path_length,
            planning_time_s=dt,
            nodes_explored=result.n_boxes,
            metadata={
                "n_boxes": result.n_boxes,
                "n_coarsen_merges": result.n_coarsen_merges,
                "planning_time_ms_cpp": result.planning_time_ms,
                "envelope_volume_total": result.envelope_volume_total,
                "build_time_ms": result.build_time_ms,
                "lect_time_ms": result.lect_time_ms,
                "endpoint_source": self._endpoint_source,
                "envelope_type": self._envelope_type,
                "use_gcs": self._use_gcs,
            },
        )

    def plan_incremental(self, start: np.ndarray, goal: np.ndarray,
                         obstacles_new, timeout: float = 30.0) -> PlanningResult:
        """Re-plan after obstacle perturbation, reusing existing forest."""
        if self._planner is None:
            return PlanningResult.failure("planner not set up")

        q_start = np.asarray(start, dtype=np.float64)
        q_goal = np.asarray(goal, dtype=np.float64)

        t0 = time.perf_counter()
        result = self._planner.plan(
            q_start, q_goal,
            obstacles_new,
            timeout_ms=timeout * 1000,
        )
        dt = time.perf_counter() - t0

        if not result.success:
            return PlanningResult.failure(
                reason="SBF incremental plan failed",
                planning_time_s=dt,
            )

        path_list = result.path
        path_arr = np.array(path_list, dtype=np.float64) if path_list else None

        return PlanningResult(
            success=True,
            path=path_arr,
            cost=result.path_length,
            planning_time_s=dt,
            nodes_explored=result.n_boxes,
            metadata={
                "n_boxes": result.n_boxes,
                "incremental": True,
                "planning_time_ms_cpp": result.planning_time_ms,
                "build_time_ms": result.build_time_ms,
            },
        )

    def reset(self) -> None:
        if self._planner is not None:
            self._planner.clear_forest()
