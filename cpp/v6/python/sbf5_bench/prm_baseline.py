"""
sbf5_bench/prm_baseline.py — PRM planner (pure Python with optional Drake)

Ported from v4/src/baselines/prm_baseline.py.
Builds a roadmap of collision-free samples connected by k-nearest edges,
then queries with Dijkstra.

Supports:
- Pure Python mode (always available)
- Drake PRM mode (if pydrake is installed)
"""

from __future__ import annotations

import heapq
import logging
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

from .base import BasePlanner, PlanningResult
from .rrt_family import _CollisionProxy, _collision_free_segment

logger = logging.getLogger(__name__)


class PRMPlanner(BasePlanner):
    """PRM roadmap planner.

    Args:
        n_samples: number of roadmap nodes.
        k_neighbors: k for k-nearest neighbor connection.
        connection_radius: maximum edge distance.
        collision_resolution: step size for edge collision checks.
    """

    def __init__(
        self,
        n_samples: int = 500,
        k_neighbors: int = 15,
        connection_radius: float = 2.0,
        collision_resolution: float = 0.05,
    ):
        self._n_samples = n_samples
        self._k_neighbors = k_neighbors
        self._connection_radius = connection_radius
        self._collision_resolution = collision_resolution

        self._robot = None
        self._scene = None
        self._checker = None
        self._config: dict = {}

        # Roadmap state
        self._nodes: Optional[np.ndarray] = None
        self._adjacency: Optional[Dict[int, List[Tuple[int, float]]]] = None
        self._roadmap_built = False

    @property
    def name(self) -> str:
        return "PRM"

    @property
    def supports_reuse(self) -> bool:
        return True

    def setup(self, robot, scene, config: Optional[dict] = None) -> None:
        self._robot = robot
        self._scene = scene
        self._checker = _CollisionProxy(robot, scene)
        self._config = config or {}
        self._nodes = None
        self._adjacency = None
        self._roadmap_built = False

    def _build_roadmap(self, timeout: float = 20.0) -> None:
        """Build PRM roadmap using random sampling."""
        t0 = time.perf_counter()
        jl = self._robot.joint_limits
        lows = np.array([lo for lo, _ in jl], dtype=np.float64)
        highs = np.array([hi for _, hi in jl], dtype=np.float64)
        ndim = len(lows)

        rng = np.random.default_rng(self._config.get('seed', 42))

        # Sample collision-free nodes
        nodes = []
        attempts = 0
        max_attempts = self._n_samples * 10

        while len(nodes) < self._n_samples and attempts < max_attempts:
            if time.perf_counter() - t0 > timeout:
                break
            q = rng.uniform(lows, highs)
            attempts += 1
            if not self._checker.check_config(q):
                nodes.append(q)

        if len(nodes) < 2:
            logger.warning("PRM: only %d collision-free samples found", len(nodes))
            self._roadmap_built = True
            self._nodes = np.array(nodes) if nodes else np.empty((0, ndim))
            self._adjacency = {}
            return

        self._nodes = np.array(nodes, dtype=np.float64)
        n = len(self._nodes)

        # Build adjacency (k-nearest + radius)
        self._adjacency = {i: [] for i in range(n)}
        for i in range(n):
            if time.perf_counter() - t0 > timeout:
                break
            dists = np.linalg.norm(self._nodes - self._nodes[i], axis=1)
            # k-nearest (exclude self)
            order = np.argsort(dists)
            neighbors = order[1:self._k_neighbors + 1]

            for j in neighbors:
                j = int(j)
                d = float(dists[j])
                if d > self._connection_radius:
                    continue
                if not _collision_free_segment(
                    self._checker, self._nodes[i], self._nodes[j],
                    self._collision_resolution
                ):
                    continue
                # Avoid duplicate edges
                if not any(nb == j for nb, _ in self._adjacency[i]):
                    self._adjacency[i].append((j, d))
                    self._adjacency[j].append((i, d))

        self._roadmap_built = True
        n_edges = sum(len(v) for v in self._adjacency.values()) // 2
        logger.info("PRM roadmap: %d nodes, %d edges, %.2fs",
                     n, n_edges, time.perf_counter() - t0)

    def _query(self, q_start: np.ndarray, q_goal: np.ndarray,
               timeout: float = 30.0) -> Optional[np.ndarray]:
        """Query roadmap for a path using Dijkstra."""
        if self._nodes is None or len(self._nodes) < 2:
            return None

        n = len(self._nodes)

        def _connect_to_roadmap(q: np.ndarray) -> Optional[int]:
            dists = np.linalg.norm(self._nodes - q, axis=1)
            order = np.argsort(dists)
            for idx in order[:self._k_neighbors * 2]:
                idx = int(idx)
                if dists[idx] > self._connection_radius * 2:
                    break
                if _collision_free_segment(
                    self._checker, q, self._nodes[idx],
                    self._collision_resolution
                ):
                    return idx
            return None

        start_idx = _connect_to_roadmap(q_start)
        goal_idx = _connect_to_roadmap(q_goal)

        if start_idx is None or goal_idx is None:
            return None

        # Dijkstra
        dist = np.full(n, float('inf'))
        prev = np.full(n, -1, dtype=int)
        dist[start_idx] = float(np.linalg.norm(q_start - self._nodes[start_idx]))
        heap = [(dist[start_idx], start_idx)]

        while heap:
            d, u = heapq.heappop(heap)
            if d > dist[u]:
                continue
            if u == goal_idx:
                break
            for v, w in self._adjacency.get(u, []):
                nd = d + w
                if nd < dist[v]:
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(heap, (nd, v))

        if dist[goal_idx] == float('inf'):
            return None

        # Extract path
        path = [q_goal]
        idx = goal_idx
        while idx != start_idx:
            path.append(self._nodes[idx].copy())
            idx = int(prev[idx])
        path.append(self._nodes[start_idx].copy())
        path.append(q_start)
        path.reverse()

        return np.array(path, dtype=np.float64)

    def plan(self, start: np.ndarray, goal: np.ndarray,
             timeout: float = 30.0) -> PlanningResult:
        q_start = np.asarray(start, dtype=np.float64)
        q_goal = np.asarray(goal, dtype=np.float64)
        t_total = time.perf_counter()
        phase_times = {}

        # Build roadmap (first call only)
        if not self._roadmap_built:
            t0 = time.perf_counter()
            self._build_roadmap(timeout * 0.7)
            phase_times['roadmap_build'] = time.perf_counter() - t0

        # Query
        t0 = time.perf_counter()
        path = self._query(q_start, q_goal, timeout)
        phase_times['query'] = time.perf_counter() - t0

        dt = time.perf_counter() - t_total

        if path is None or len(path) < 2:
            return PlanningResult.failure(
                reason="PRM query failed",
                planning_time_s=dt,
            )

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
            nodes_explored=(len(self._nodes)
                            if self._nodes is not None
                            else self._n_samples),
            metadata={
                "algorithm": "PRM",
                "n_nodes": (len(self._nodes)
                            if self._nodes is not None
                            else self._n_samples),
                "phase_times": phase_times,
            },
        )

    def reset(self) -> None:
        self._roadmap_built = False
        self._nodes = None
        self._adjacency = None
