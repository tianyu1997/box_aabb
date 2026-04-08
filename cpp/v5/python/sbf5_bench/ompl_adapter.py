"""
sbf5_bench/ompl_adapter.py — OMPL planner via WSL subprocess

Calls OMPL C++ solvers through a WSL subprocess bridge.
Supports: RRT, RRTConnect, RRTstar, InformedRRTstar, BITstar, ABITstar.

Migrated from: v3/src/baselines/ompl_adapter.py
"""

from __future__ import annotations

import json
import logging
import subprocess
import tempfile
import time
from pathlib import Path
from typing import Optional

import numpy as np

from .base import BasePlanner, PlanningResult

logger = logging.getLogger(__name__)

OMPL_ALGORITHMS = [
    "RRT", "RRTConnect", "RRTstar",
    "InformedRRTstar", "BITstar", "ABITstar",
]


class OMPLPlanner(BasePlanner):
    """OMPL C++ planner via WSL subprocess bridge.

    Args:
        algorithm: OMPL planner name (must be in OMPL_ALGORITHMS).
        wsl_binary: WSL-side path to the OMPL bridge executable/script.
    """

    def __init__(self, algorithm: str = "RRTConnect",
                 wsl_binary: str = "sbf_ompl_bridge"):
        if algorithm not in OMPL_ALGORITHMS:
            raise ValueError(
                f"Unknown algorithm '{algorithm}'. "
                f"Supported: {OMPL_ALGORITHMS}")
        self._algo = algorithm
        self._binary = wsl_binary
        self._robot_data = None
        self._scene_data = None
        self._config: dict = {}

    @property
    def name(self) -> str:
        return f"OMPL-{self._algo}"

    @property
    def supports_reuse(self) -> bool:
        return False

    def setup(self, robot, scene, config: Optional[dict] = None) -> None:
        self._robot_data = robot
        self._scene_data = scene
        self._config = config or {}

    def plan(self, start: np.ndarray, goal: np.ndarray,
             timeout: float = 30.0) -> PlanningResult:
        q_start = np.asarray(start, dtype=np.float64)
        q_goal = np.asarray(goal, dtype=np.float64)

        # Serialize obstacles
        obs_list = []
        if self._scene_data is not None:
            for obs in self._scene_data:
                mn = obs.min_point()
                mx = obs.max_point()
                obs_list.append({
                    "min_point": mn.tolist(),
                    "max_point": mx.tolist(),
                })

        problem = {
            "algorithm": self._algo,
            "q_start": q_start.tolist(),
            "q_goal": q_goal.tolist(),
            "obstacles": obs_list,
            "timeout": timeout,
            "seed": self._config.get("seed", 42),
        }

        # Add joint limits if available
        if self._robot_data is not None:
            try:
                problem["joint_lo"] = self._robot_data.joint_lo().tolist()
                problem["joint_hi"] = self._robot_data.joint_hi().tolist()
            except AttributeError:
                pass

        t0 = time.perf_counter()

        try:
            with tempfile.NamedTemporaryFile(
                mode='w', suffix='.json', delete=False
            ) as f:
                json.dump(problem, f)
                tmp_path = f.name

            # Convert Windows path to WSL path
            wsl_path = subprocess.check_output(
                ["wsl", "wslpath", tmp_path],
                timeout=10,
            ).decode().strip()

            result_raw = subprocess.check_output(
                ["wsl", self._binary, wsl_path],
                timeout=timeout + 30,
            )
            result = json.loads(result_raw)

        except FileNotFoundError:
            dt = time.perf_counter() - t0
            logger.error("WSL not available")
            return PlanningResult.failure("WSL not available", dt)
        except subprocess.TimeoutExpired:
            dt = time.perf_counter() - t0
            logger.warning("OMPL bridge timed out for %s", self._algo)
            return PlanningResult.failure("OMPL timeout", dt)
        except (subprocess.CalledProcessError, json.JSONDecodeError) as e:
            dt = time.perf_counter() - t0
            logger.warning("OMPL bridge error for %s: %s", self._algo, e)
            return PlanningResult.failure(f"OMPL error: {e}", dt)
        finally:
            Path(tmp_path).unlink(missing_ok=True)

        dt = time.perf_counter() - t0

        if not result.get("success", False):
            return PlanningResult.failure("OMPL solve failed", dt)

        path_raw = result.get("path", [])
        path = np.array(path_raw, dtype=np.float64) if path_raw else None

        return PlanningResult(
            success=True,
            path=path,
            cost=result.get("cost", 0.0),
            planning_time_s=result.get("time", dt),
            collision_checks=result.get("collision_checks", 0),
            nodes_explored=result.get("nodes", 0),
            metadata={
                "algorithm": self.name,
                "wall_time": dt,
                "bridge_time_s": result.get("time", None),
                "cache_hits": result.get("cache_hits", 0),
                "cache_misses": result.get("cache_misses", 0),
            },
        )

    def reset(self) -> None:
        pass
