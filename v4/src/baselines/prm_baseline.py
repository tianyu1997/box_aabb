"""
baselines/prm_baseline.py — PRM 基线规划器

基于 Marcucci et al. (2024) 的 PRM 对比实现
(reproduction/prm_comparison/planning.py).

PRM 作为经典 multi-query 基线:
- 同样支持持久化与多查询摊还 (supports_reuse=True)
- 但不提供凸 region, 路径是分段线性
- 采用采样碰撞检测 (非认证)

两种实现模式:
1. Drake PRM (pydrake PRMPlanner) — 用于与 Marcucci 论文精确对比
2. 纯 Python PRM — 回退实现, 使用与 SBF 相同的碰撞检测后端
"""

from __future__ import annotations

import logging
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

from .base import BasePlanner, PlanningResult

logger = logging.getLogger(__name__)


class PRMPlanner(BasePlanner):
    """PRM baseline planner.

    roadmap 构建一次后可复用于多次查询.
    """

    def __init__(
        self,
        n_samples: int = 2000,
        k_neighbors: int = 15,
        connection_radius: float = 2.0,
        collision_resolution: float = 0.05,
        seed: int = 42,
        use_drake: bool = True,
    ):
        self._n_samples = n_samples
        self._k_neighbors = k_neighbors
        self._connection_radius = connection_radius
        self._collision_resolution = collision_resolution
        self._seed = seed
        self._use_drake = use_drake

        self._robot = None
        self._scene = None
        self._config: dict = {}
        self._roadmap_built = False
        self._nodes: Optional[np.ndarray] = None
        self._adjacency: Optional[Dict[int, List[Tuple[int, float]]]] = None
        self._checker = None

        # Drake mode
        self._drake_prm = None
        self._drake_available = False

    @property
    def name(self) -> str:
        return "PRM"

    @property
    def supports_reuse(self) -> bool:
        return True

    def setup(self, robot, scene, config: dict) -> None:
        self._robot = robot
        self._scene = scene
        self._config = config
        self._roadmap_built = False
        self._nodes = None
        self._adjacency = None
        self._drake_prm = None

        if self._use_drake:
            try:
                self._setup_drake_prm()
                self._drake_available = True
            except Exception as e:
                logger.info(f"Drake PRM not available ({e}), "
                            f"falling back to Python PRM")
                self._drake_available = False

        if not self._drake_available:
            from forest.collision import CollisionChecker
            self._checker = CollisionChecker(robot=robot, scene=scene)

    def _setup_drake_prm(self):
        """Setup Drake-based PRM planner."""
        import os
        gcs_dir = None
        candidates = [
            os.path.join(os.path.dirname(__file__), "..", "..", "..",
                         "gcs-science-robotics"),
            os.path.expanduser("~/桌面/box_aabb/gcs-science-robotics"),
        ]
        for c in candidates:
            c = os.path.abspath(c)
            if os.path.isdir(c):
                gcs_dir = c
                break

        if gcs_dir is None:
            raise FileNotFoundError("gcs-science-robotics not found")

        # 尝试导入 Drake PRM
        from pydrake.all import (
            PRMPlanner as DrakePRM,
            HolonomicKinematicPlanningSpace,
            JointLimits,
            VoxelizedEnvironmentCollisionChecker,
            RobotDiagramBuilder,
            LoadModelDirectives,
            ProcessModelDirectives,
        )

        builder = RobotDiagramBuilder(time_step=0.0)
        builder.parser().package_map().Add("gcs", gcs_dir)

        directives_file = os.path.join(
            gcs_dir, "models",
            "iiwa14_spheres_collision_welded_gripper.yaml")
        directives = LoadModelDirectives(directives_file)
        ProcessModelDirectives(directives, builder.parser())

        builder.plant().Finalize()
        joint_limits = JointLimits(builder.plant())
        diagram = builder.Build()

        iiwa_idx = builder.plant().GetModelInstanceByName("iiwa")
        wsg_idx = builder.plant().GetModelInstanceByName("wsg")

        collision_checker = VoxelizedEnvironmentCollisionChecker(
            model=diagram,
            robot_model_instances=[iiwa_idx, wsg_idx],
            edge_step_size=self._collision_resolution,
            env_collision_padding=0.01,
            self_collision_padding=0.01,
        )
        collision_checker.VoxelizeEnvironment(
            (2.0, 2.0, 2.0), 0.02)

        self._drake_planning_space = HolonomicKinematicPlanningSpace(
            collision_checker, joint_limits, 0.5, self._seed)
        self._drake_prm_class = DrakePRM
        logger.info("Drake PRM initialized successfully")

    def _build_roadmap_drake(self, timeout: float = 60.0):
        """Build roadmap using Drake's PRM."""
        from pydrake.all import PRMPlanner as DrakePRM

        t0 = time.perf_counter()
        self._drake_roadmap = DrakePRM.GrowRoadmap(
            self._n_samples, self._drake_planning_space, self._seed)
        self._roadmap_built = True
        logger.info(f"Drake PRM roadmap built: {self._n_samples} nodes, "
                     f"{time.perf_counter() - t0:.2f}s")

    def _build_roadmap_python(self, timeout: float = 60.0):
        """Build roadmap using Python implementation."""
        rng = np.random.default_rng(self._seed)
        jl = self._robot.joint_limits
        lows = np.array([lo for lo, _ in jl])
        highs = np.array([hi for _, hi in jl])
        ndim = len(jl)

        # ── 1) Sample collision-free configurations ──
        nodes_list = []
        t0 = time.perf_counter()
        max_attempts = self._n_samples * 5

        for _ in range(max_attempts):
            if len(nodes_list) >= self._n_samples:
                break
            if time.perf_counter() - t0 > timeout * 0.5:
                break
            q = rng.uniform(lows, highs)
            if not self._checker.check_config_collision(q):
                nodes_list.append(q)

        self._nodes = np.array(nodes_list, dtype=np.float64)
        n = len(self._nodes)

        # ── 2) Connect neighbors ──
        self._adjacency = {i: [] for i in range(n)}
        for i in range(n):
            if time.perf_counter() - t0 > timeout:
                break
            dists = np.linalg.norm(self._nodes - self._nodes[i], axis=1)
            dists[i] = float('inf')

            # k-nearest + radius
            neighbors = np.argsort(dists)[:self._k_neighbors]
            for j in neighbors:
                j = int(j)
                d = float(dists[j])
                if d > self._connection_radius:
                    continue
                # Check edge collision-free
                if not self._checker.check_segment_collision(
                        self._nodes[i], self._nodes[j],
                        self._collision_resolution):
                    self._adjacency[i].append((j, d))
                    self._adjacency[j].append((i, d))

        self._roadmap_built = True
        logger.info(f"Python PRM roadmap: {n} nodes, "
                     f"{sum(len(v) for v in self._adjacency.values()) // 2} edges, "
                     f"{time.perf_counter() - t0:.2f}s")

    def _query_python(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        timeout: float = 30.0,
    ) -> Optional[np.ndarray]:
        """Query the roadmap for a path."""
        import heapq

        n = len(self._nodes)

        # Connect start and goal to roadmap
        def _connect_to_roadmap(q: np.ndarray) -> Optional[int]:
            dists = np.linalg.norm(self._nodes - q, axis=1)
            order = np.argsort(dists)
            for idx in order[:self._k_neighbors * 2]:
                idx = int(idx)
                if dists[idx] > self._connection_radius * 2:
                    break
                if not self._checker.check_segment_collision(
                        q, self._nodes[idx], self._collision_resolution):
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

        # ── 1) Build roadmap (first call only) ──
        if not self._roadmap_built:
            t0 = time.perf_counter()
            try:
                if self._drake_available:
                    self._build_roadmap_drake(timeout * 0.7)
                else:
                    self._build_roadmap_python(timeout * 0.7)
            except Exception as e:
                logger.error(f"PRM roadmap build failed: {e}")
                return PlanningResult.failure(
                    planning_time=time.perf_counter() - t_total,
                    reason=f"Roadmap build failed: {e}")
            phase_times['roadmap_build'] = time.perf_counter() - t0

        # ── 2) Query ──
        t0 = time.perf_counter()
        if self._drake_available:
            try:
                from pydrake.all import PRMPlannerQueryParameters
                params = PRMPlannerQueryParameters()
                result, runtime = self._drake_prm_class.TimedPlanLazy(
                    q_start, q_goal, params,
                    self._drake_planning_space, self._drake_roadmap)
                phase_times['query'] = runtime

                if result.has_solution():
                    path = np.array(result.path(), dtype=np.float64)
                    if path.ndim == 1:
                        path = path.reshape(-1, len(q_start))
                else:
                    path = None
            except Exception as e:
                logger.warning(f"Drake PRM query failed: {e}")
                path = None
                phase_times['query'] = time.perf_counter() - t0
        else:
            path = self._query_python(q_start, q_goal, timeout)
            phase_times['query'] = time.perf_counter() - t0

        dt = time.perf_counter() - t_total

        if path is None or len(path) < 2:
            return PlanningResult.failure(
                planning_time=dt,
                nodes_explored=self._n_samples if self._roadmap_built else 0,
                reason="PRM query failed")

        cost = float(sum(
            np.linalg.norm(path[i] - path[i - 1])
            for i in range(1, len(path))
        ))

        return PlanningResult(
            success=True,
            path=path,
            cost=cost,
            planning_time=dt,
            first_solution_time=dt,
            collision_checks=0,
            nodes_explored=self._n_samples if self._roadmap_built else 0,
            phase_times=phase_times,
            metadata={
                "algorithm": "PRM",
                "n_nodes": len(self._nodes) if self._nodes is not None else self._n_samples,
                "drake_mode": self._drake_available,
            },
        )

    def reset(self) -> None:
        self._roadmap_built = False
        self._nodes = None
        self._adjacency = None
        self._drake_roadmap = None
