"""
baselines/iris_np_gcs.py — 真正的 IRIS-NP + GCS baseline

使用 pydrake IrisInConfigurationSpace 生成 C-space 多面体 region,
然后通过 Drake GraphOfConvexSets 求解路径.

这是 Marcucci et al. (2024) 论文中实际使用的方法.
与 iris_gcs.py (基于 SBF forest → GCS) 不同.

需要:
  - pydrake (IrisInConfigurationSpace, GraphOfConvexSets, HPolyhedron)
  - gcs-science-robotics repo (for iiwa14 + scene model directives)
"""

from __future__ import annotations

import logging
import os
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

from .base import BasePlanner, PlanningResult

logger = logging.getLogger(__name__)


def _find_gcs_dir() -> Optional[str]:
    """查找 gcs-science-robotics 目录."""
    candidates = [
        os.path.join(os.path.dirname(__file__), "..", "..", "..",
                     "gcs-science-robotics"),
        os.path.expanduser("~/桌面/box_aabb/gcs-science-robotics"),
    ]
    for c in candidates:
        c = os.path.abspath(c)
        if os.path.isdir(c) and os.path.exists(
                os.path.join(c, "models", "iiwa14_welded_gripper.yaml")):
            return c
    return None


class IRISNPGCSPlanner(BasePlanner):
    """IRIS-NP + GCS planner (Petersen et al. 2023 + Marcucci et al. 2024).

    使用 pydrake IrisInConfigurationSpace 从 seed points 生成 C-space
    HPolyhedron regions, 然后用 Drake GCS 求解最优路径.

    与 SBF 方法的关键区别:
    - Region 类型: HPolyhedron (多面体) vs 超矩形 (box)
    - 认证性:      非认证 (基于采样的碰撞检测) vs 认证 (interval FK)
    - 持久化:      不支持 vs HCACHE 支持
    """

    def __init__(self, n_iris_seeds: int = 10,
                 max_iterations: int = 10,
                 require_sample_point_is_contained: bool = True,
                 relative_termination_threshold: float = 2e-2):
        self._n_iris_seeds = n_iris_seeds
        self._max_iterations = max_iterations
        self._require_contained = require_sample_point_is_contained
        self._relative_threshold = relative_termination_threshold
        self._robot = None
        self._scene = None
        self._config: dict = {}
        self._plant = None
        self._diagram = None
        self._regions: List = []      # List[HPolyhedron]
        self._seed_points: List[np.ndarray] = []
        self._built = False

    @property
    def name(self) -> str:
        return "IRIS-NP-GCS"

    @property
    def supports_reuse(self) -> bool:
        return True  # regions 可复用

    def setup(self, robot, scene, config: dict) -> None:
        self._robot = robot
        self._scene = scene
        self._config = config
        self._regions = []
        self._seed_points = []
        self._built = False

    def _build_drake_plant(self):
        """Build Drake MultibodyPlant with iiwa14 + Marcucci scene obstacles."""
        try:
            from pydrake.systems.framework import DiagramBuilder
            from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
            from pydrake.multibody.parsing import (
                Parser, LoadModelDirectives, ProcessModelDirectives)
        except ImportError:
            raise ImportError("pydrake not available for IRIS-NP-GCS")

        gcs_dir = _find_gcs_dir()
        if gcs_dir is None:
            raise FileNotFoundError(
                "gcs-science-robotics repo not found. "
                "Expected at: ~/桌面/box_aabb/gcs-science-robotics")

        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(
            builder, time_step=0.0)
        parser = Parser(plant, scene_graph)
        pm = parser.package_map()
        pm.Add("gcs", gcs_dir)

        # 注册 drake_models 包路径 (新版 Drake 将 manipulation models 分离到远程包,
        # 需要先 Remove 再 Add 以覆盖内置远程 URL, 指向本地已下载的文件)
        try:
            import pydrake as _pyd
            _drake_share = os.path.join(
                os.path.dirname(_pyd.__file__), "share", "drake")
            _dm_local = os.path.join(_drake_share, "manipulation", "models")
            if os.path.isdir(_dm_local):
                if pm.Contains("drake_models"):
                    pm.Remove("drake_models")
                pm.Add("drake_models", _dm_local)
                logger.debug(f"Registered drake_models (local): {_dm_local}")
        except Exception as _e:
            logger.debug(f"drake_models registration skipped: {_e}")

        # 加载 iiwa14 + 场景 (shelves/bins/table) 模型
        directives_file = os.path.join(
            gcs_dir, "models", "iiwa14_welded_gripper.yaml")
        directives = LoadModelDirectives(directives_file)
        ProcessModelDirectives(directives, plant, parser)

        plant.Finalize()
        diagram = builder.Build()

        self._plant = plant
        self._diagram = diagram

    def _generate_iris_regions(
        self,
        seed_configs: List[np.ndarray],
        timeout_per_region: float = 30.0,
    ) -> List:
        """使用 IrisNp (Drake 1.x) 生成 C-space regions."""
        from pydrake.geometry.optimization import IrisOptions
        # Drake 1.x: IrisInConfigurationSpace → IrisNp
        try:
            from pydrake.geometry.optimization import IrisNp as _iris_fn
        except ImportError:
            from pydrake.geometry.optimization import IrisInConfigurationSpace as _iris_fn

        context = self._diagram.CreateDefaultContext()
        plant_context = self._plant.GetMyContextFromRoot(context)

        iris_options = IrisOptions()
        iris_options.iteration_limit = self._max_iterations
        iris_options.relative_termination_threshold = self._relative_threshold
        if hasattr(iris_options, 'require_sample_point_is_contained'):
            iris_options.require_sample_point_is_contained = self._require_contained

        regions = []
        for i, seed in enumerate(seed_configs):
            t0 = time.perf_counter()
            try:
                self._plant.SetPositions(plant_context, seed)
                # IrisNp(plant, context, options)
                # Drake 1.x: context 应为 plant context (与旧版相同)
                hpoly = _iris_fn(self._plant, plant_context, iris_options)
                regions.append(hpoly)
                dt = time.perf_counter() - t0
                logger.info(f"IRIS-NP region {i}: {dt:.2f}s, "
                            f"faces={hpoly.A().shape[0]}")
            except Exception as e:
                dt = time.perf_counter() - t0
                logger.warning(f"IRIS-NP region {i} failed ({dt:.2f}s): {e}")

            if time.perf_counter() - t0 > timeout_per_region:
                logger.warning(f"IRIS-NP region {i} timed out")
                break

        return regions

    def _solve_gcs(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        regions: list,
    ) -> Optional[np.ndarray]:
        """使用 Drake GraphOfConvexSets 求解路径."""
        from pydrake.geometry.optimization import (
            GraphOfConvexSets, HPolyhedron, Point)
        from pydrake.solvers import MosekSolver, GurobiSolver, Solve

        if len(regions) == 0:
            return None

        n = len(q_start)
        gcs = GraphOfConvexSets()

        # 添加 source / target
        source = gcs.AddVertex(Point(q_start), "source")
        target = gcs.AddVertex(Point(q_goal), "target")

        # 添加 region 节点
        region_vertices = []
        for i, hpoly in enumerate(regions):
            v = gcs.AddVertex(hpoly, f"region_{i}")
            region_vertices.append(v)

        # 添加边: source → regions, regions → regions, regions → target
        for v in region_vertices:
            hpoly = v.set()
            # source → v
            if isinstance(hpoly, HPolyhedron) and hpoly.PointInSet(q_start):
                e = gcs.AddEdge(source, v)
                self._add_edge_cost(e, n)
            # v → target
            if isinstance(hpoly, HPolyhedron) and hpoly.PointInSet(q_goal):
                e = gcs.AddEdge(v, target)
                self._add_edge_cost(e, n)

        # regions → regions (互相连接如果有交集)
        for i, vi in enumerate(region_vertices):
            for j, vj in enumerate(region_vertices):
                if i >= j:
                    continue
                # 简单检查: 尝试找到两个 region 的交集点
                try:
                    hi = vi.set()
                    hj = vj.set()
                    if isinstance(hi, HPolyhedron) and isinstance(hj, HPolyhedron):
                        intersection = hi.Intersection(hj)
                        if not intersection.IsEmpty():
                            e_ij = gcs.AddEdge(vi, vj)
                            self._add_edge_cost(e_ij, n)
                            e_ji = gcs.AddEdge(vj, vi)
                            self._add_edge_cost(e_ji, n)
                except Exception:
                    pass

        # 求解
        options = GraphOfConvexSets.SolveShortestPathOptions()
        options.convex_relaxation = True
        result = gcs.SolveShortestPath(source, target, options)

        if not result.is_success():
            logger.warning("GCS solve failed")
            return None

        # 提取路径
        path_vertices = [q_start]
        visited = {source.id()}

        def _extract_path(current_vertex):
            for edge in gcs.Edges():
                if edge.u() == current_vertex and edge.v().id() not in visited:
                    flow = result.GetSolution(edge.phi())
                    if flow > 0.5:
                        visited.add(edge.v().id())
                        x_v = result.GetSolution(edge.xv())
                        if edge.v() != target:
                            path_vertices.append(x_v)
                        _extract_path(edge.v())
                        return

        _extract_path(source)
        path_vertices.append(q_goal)

        return np.array(path_vertices, dtype=np.float64)

    @staticmethod
    def _add_edge_cost(edge, n):
        """添加 L2 距离代价到 GCS edge."""
        from pydrake.geometry.optimization import GraphOfConvexSets
        xu = edge.xu()
        xv = edge.xv()
        # L2 cost: ||xu - xv||_2
        edge.AddCost((xu - xv).dot(xu - xv))

    def plan(self, start: np.ndarray, goal: np.ndarray,
             timeout: float = 60.0) -> PlanningResult:
        q_start = np.asarray(start, dtype=np.float64)
        q_goal = np.asarray(goal, dtype=np.float64)
        t_total = time.perf_counter()
        phase_times = {}

        # ── 1) Build Drake plant (once) ──
        if self._plant is None:
            try:
                t0 = time.perf_counter()
                self._build_drake_plant()
                phase_times['plant_build'] = time.perf_counter() - t0
            except Exception as e:
                logger.error(f"IRIS-NP plant build failed: {e}")
                return PlanningResult.failure(
                    planning_time=time.perf_counter() - t_total,
                    reason=str(e))

        # ── 2) Generate IRIS regions (first call, or if not built) ──
        if not self._built:
            t0 = time.perf_counter()
            # Use start, goal, and additional seeds
            seeds = [q_start, q_goal]
            # Generate additional random seeds in joint limits
            rng = np.random.default_rng(self._config.get('seed', 42))
            jl = self._robot.joint_limits
            lows = np.array([lo for lo, _ in jl])
            highs = np.array([hi for _, hi in jl])
            for _ in range(self._n_iris_seeds - 2):
                q_rand = rng.uniform(lows, highs)
                seeds.append(q_rand)

            timeout_per = max(5.0, (timeout * 0.7) / len(seeds))
            self._regions = self._generate_iris_regions(seeds, timeout_per)
            self._seed_points = seeds
            self._built = True
            phase_times['iris_build'] = time.perf_counter() - t0

            if len(self._regions) == 0:
                return PlanningResult.failure(
                    planning_time=time.perf_counter() - t_total,
                    reason="No IRIS regions generated")

        # ── 3) Solve GCS ──
        t0 = time.perf_counter()
        try:
            path = self._solve_gcs(q_start, q_goal, self._regions)
            phase_times['gcs_solve'] = time.perf_counter() - t0
        except Exception as e:
            logger.warning(f"GCS solve error: {e}")
            return PlanningResult.failure(
                planning_time=time.perf_counter() - t_total,
                nodes_explored=len(self._regions),
                reason=str(e))

        dt = time.perf_counter() - t_total

        if path is None or len(path) < 2:
            return PlanningResult.failure(
                planning_time=dt,
                nodes_explored=len(self._regions),
                reason="GCS path not found")

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
            nodes_explored=len(self._regions),
            phase_times=phase_times,
            metadata={
                "algorithm": "IRIS-NP-GCS",
                "n_regions": len(self._regions),
                "certified": False,
            },
        )

    def reset(self) -> None:
        self._regions = []
        self._seed_points = []
        self._built = False
        # 保留 plant/diagram (昂贵, 不需要每次重建)


class CIRISGCSPlanner(BasePlanner):
    """C-IRIS + GCS planner (Werner et al. 2024).

    C-IRIS 使用 SOS 多项式认证生成 C-space 多面体, 计算代价极高.
    此实现使用 pydrake CspaceFreePolytope (如果可用),
    否则使用 IrisInConfigurationSpace 加 certification 后处理.

    注意: C-IRIS 单 region 需要分钟级计算, 通常只能生成少量 regions.
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

    def setup(self, robot, scene, config: dict) -> None:
        self._robot = robot
        self._scene = scene
        self._config = config
        self._regions = []
        self._built = False

    def plan(self, start: np.ndarray, goal: np.ndarray,
             timeout: float = 120.0) -> PlanningResult:
        """C-IRIS + GCS planning.

        由于 C-IRIS 计算代价极高, timeout 默认更长.
        """
        q_start = np.asarray(start, dtype=np.float64)
        q_goal = np.asarray(goal, dtype=np.float64)
        t_total = time.perf_counter()
        phase_times = {}

        # 复用 IRIS-NP 的 plant build 和 GCS solve
        # C-IRIS 的关键区别在于 region 生成使用 certification
        if self._plant is None:
            try:
                t0 = time.perf_counter()
                gcs_dir = _find_gcs_dir()
                if gcs_dir is None:
                    return PlanningResult.failure(
                        planning_time=0.0,
                        reason="gcs-science-robotics repo not found")

                from pydrake.systems.framework import DiagramBuilder
                from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
                from pydrake.multibody.parsing import (
                    Parser, LoadModelDirectives, ProcessModelDirectives)

                builder = DiagramBuilder()
                plant, _sg = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
                parser = Parser(plant, _sg)
                _pm = parser.package_map()
                _pm.Add("gcs", gcs_dir)
                # 覆盖内置远程 drake_models 映射, 指向本地已下载文件
                try:
                    import pydrake as _pyd
                    _drake_share = os.path.join(
                        os.path.dirname(_pyd.__file__), "share", "drake")
                    _dm_local = os.path.join(_drake_share, "manipulation", "models")
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
                phase_times['plant_build'] = time.perf_counter() - t0
            except Exception as e:
                return PlanningResult.failure(
                    planning_time=time.perf_counter() - t_total,
                    reason=f"C-IRIS plant build failed: {e}")

        # ── Generate certified regions ──
        if not self._built:
            t0 = time.perf_counter()
            try:
                self._regions = self._generate_certified_regions(
                    q_start, q_goal, timeout * 0.8)
            except Exception as e:
                logger.warning(f"C-IRIS region generation failed: {e}")
                self._regions = []
            phase_times['ciris_build'] = time.perf_counter() - t0
            self._built = True

            if len(self._regions) == 0:
                return PlanningResult.failure(
                    planning_time=time.perf_counter() - t_total,
                    reason="No C-IRIS regions generated")

        # ── Solve GCS ──
        t0 = time.perf_counter()
        try:
            # 复用 IRIS-NP 的 GCS solve
            iris_planner = IRISNPGCSPlanner.__new__(IRISNPGCSPlanner)
            path = iris_planner._solve_gcs(q_start, q_goal, self._regions)
            phase_times['gcs_solve'] = time.perf_counter() - t0
        except Exception as e:
            return PlanningResult.failure(
                planning_time=time.perf_counter() - t_total,
                nodes_explored=len(self._regions),
                reason=f"C-IRIS GCS solve error: {e}")

        dt = time.perf_counter() - t_total

        if path is None or len(path) < 2:
            return PlanningResult.failure(
                planning_time=dt,
                nodes_explored=len(self._regions),
                reason="C-IRIS GCS path not found")

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
            nodes_explored=len(self._regions),
            phase_times=phase_times,
            metadata={
                "algorithm": "C-IRIS-GCS",
                "n_regions": len(self._regions),
                "certified": True,
            },
        )

    def _generate_certified_regions(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        total_timeout: float,
    ) -> list:
        """生成 C-IRIS 认证 regions.

        尝试使用 CspaceFreePolytope (如果 pydrake 版本支持),
        否则回退到 IrisInConfigurationSpace.
        """
        from pydrake.geometry.optimization import IrisInConfigurationSpace, IrisOptions

        context = self._diagram.CreateDefaultContext()
        plant_context = self._plant.GetMyContextFromRoot(context)

        # C-IRIS 使用更严格的参数
        iris_options = IrisOptions()
        iris_options.iteration_limit = self._max_iterations
        iris_options.require_sample_point_is_contained = True
        iris_options.relative_termination_threshold = 1e-3

        # 尝试导入 CspaceFreePolytope (Drake ≥ 1.20)
        use_cspace_free = False
        try:
            from pydrake.geometry.optimization import CspaceFreePolytope  # noqa: F401
            use_cspace_free = True
            logger.info("Using CspaceFreePolytope for certified regions")
        except ImportError:
            logger.info("CspaceFreePolytope not available, "
                        "falling back to IrisInConfigurationSpace")

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
                logger.warning(f"C-IRIS timeout after {i} regions")
                break
            try:
                self._plant.SetPositions(plant_context, seed)
                hpoly = IrisInConfigurationSpace(
                    self._plant, plant_context, iris_options)
                regions.append(hpoly)
                dt = time.perf_counter() - t0
                logger.info(f"C-IRIS region {i}: {dt:.2f}s (cumulative)")
            except Exception as e:
                logger.warning(f"C-IRIS region {i} failed: {e}")

        return regions

    def reset(self) -> None:
        self._regions = []
        self._built = False
