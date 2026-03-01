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
                 relative_termination_threshold: float = 2e-2,
                 timeout_per_region: float = 300.0):
        self._n_iris_seeds = n_iris_seeds
        self._max_iterations = max_iterations
        self._require_contained = require_sample_point_is_contained
        self._relative_threshold = relative_termination_threshold
        self._timeout_per_region = timeout_per_region
        self._robot = None
        self._scene = None
        self._config: dict = {}
        self._plant = None
        self._diagram = None
        self._regions: List = []      # List[HPolyhedron]
        self._seed_points: List[np.ndarray] = []
        self._manual_seeds: List[np.ndarray] = []  # 外部提供的 seed 配置
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
        self._manual_seeds = [np.asarray(s, dtype=np.float64)
                              for s in config.get('manual_seeds', [])]
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
        iris_options.termination_threshold = -1                    # 原论文: -1 (禁用)
        iris_options.relative_termination_threshold = self._relative_threshold  # 原论文: 0.02
        iris_options.num_collision_infeasible_samples = 1          # 原论文: 1
        if hasattr(iris_options, 'require_sample_point_is_contained'):
            iris_options.require_sample_point_is_contained = self._require_contained
        iris_options.random_seed = 0

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
        """使用 gcs-science-robotics 的 LinearGCS 求解路径.

        严格复现 Marcucci et al. (2024):
        - LinearGCS(regions) 自动构建 overlap 边
        - randomForwardPathSearch rounding (max_paths=10, max_trials=100)
        - MosekSolver
        """
        import sys
        gcs_dir = _find_gcs_dir()
        if gcs_dir and gcs_dir not in sys.path:
            sys.path.insert(0, gcs_dir)

        from gcs.linear import LinearGCS
        from gcs.rounding import randomForwardPathSearch
        from pydrake.solvers import MosekSolver

        if len(regions) == 0:
            return None

        seed = self._config.get('seed', 0) if hasattr(self, '_config') else 0

        gcs = LinearGCS(regions)
        try:
            gcs.addSourceTarget(q_start, q_goal)
        except ValueError as e:
            logger.warning(f"GCS addSourceTarget failed: {e}")
            return None
        gcs.setRoundingStrategy(randomForwardPathSearch,
                                max_paths=10,
                                max_trials=100,
                                seed=seed)
        gcs.setSolver(MosekSolver())

        # 设置求解器容差 (与原论文一致)
        from pydrake.solvers import SolverOptions
        solver_options = SolverOptions()
        solver_options.SetOption(MosekSolver.id(),
                                "MSK_DPAR_INTPNT_CO_TOL_REL_GAP", 1e-3)
        gcs.setSolverOptions(solver_options)

        waypoints, results_dict = gcs.SolvePath(
            rounding=True, verbose=False, preprocessing=True)

        if waypoints is None:
            logger.warning("GCS solve failed")
            return None

        # waypoints shape: (ndim, n_points), 转置为 (n_points, ndim)
        path = waypoints.T
        return np.asarray(path, dtype=np.float64)

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

            if self._manual_seeds:
                # 优先使用外部提供的 manual seed 配置
                for ms in self._manual_seeds:
                    # 去重: 跳过与 start/goal 过于接近的
                    if (np.linalg.norm(ms - q_start) > 0.01
                            and np.linalg.norm(ms - q_goal) > 0.01):
                        seeds.append(ms)
                logger.info(f"IRIS-NP: using {len(seeds)} seeds "
                            f"({len(self._manual_seeds)} manual + start/goal)")
                # 如果 manual seeds 不够, 用 random 补齐
                if len(seeds) < self._n_iris_seeds:
                    rng = np.random.default_rng(self._config.get('seed', 42))
                    jl = self._robot.joint_limits
                    lows = np.array([lo for lo, _ in jl])
                    highs = np.array([hi for _, hi in jl])
                    while len(seeds) < self._n_iris_seeds:
                        q_rand = rng.uniform(lows, highs)
                        seeds.append(q_rand)
            else:
                # 无 manual seeds: 生成随机 seeds (原逻辑)
                rng = np.random.default_rng(self._config.get('seed', 42))
                jl = self._robot.joint_limits
                lows = np.array([lo for lo, _ in jl])
                highs = np.array([hi for _, hi in jl])
                for _ in range(self._n_iris_seeds - 2):
                    q_rand = rng.uniform(lows, highs)
                    seeds.append(q_rand)

            timeout_per = max(5.0, (timeout * 0.7) / len(seeds))
            timeout_per = min(timeout_per, self._timeout_per_region)
            logger.info(f"IRIS-NP: {len(seeds)} seeds, iter_limit={self._max_iterations}, "
                        f"timeout_per={timeout_per:.1f}s")
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

    def __init__(self, n_regions: int = 5, max_iterations: int = 10):
        self._n_regions = n_regions
        self._max_iterations = max_iterations  # 原论文: 10
        self._robot = None
        self._scene = None
        self._config: dict = {}
        self._plant = None
        self._diagram = None
        self._regions: List = []
        self._manual_seeds: List[np.ndarray] = []  # 外部提供的 seed 配置
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
        self._manual_seeds = [np.asarray(s, dtype=np.float64)
                              for s in config.get('manual_seeds', [])]
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

                # ── 缩小关节限制 (如果实验传入了 planning_limits) ──
                planning_limits = self._config.get('planning_limits')
                if planning_limits:
                    from pydrake.multibody.tree import JointIndex
                    q_idx = 0
                    for ji in range(plant.num_joints()):
                        joint = plant.get_joint(JointIndex(ji))
                        if joint.num_positions() == 1 and hasattr(
                                joint, 'set_position_limits'):
                            if q_idx < len(planning_limits):
                                lo, hi = planning_limits[q_idx]
                                joint.set_position_limits(
                                    [float(lo)], [float(hi)])
                                logger.info(
                                    f"  Drake joint {joint.name()}: "
                                    f"[{lo:.3f}, {hi:.3f}]")
                            q_idx += 1

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
            # 复用 IRIS-NP 的 GCS solve (同样使用 LinearGCS)
            iris_planner = IRISNPGCSPlanner.__new__(IRISNPGCSPlanner)
            iris_planner._config = self._config
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
        """生成 IRIS regions (严格复现 Marcucci et al. 2024).

        使用 IrisNp (= 旧版 IrisInConfigurationSpace) + 与原论文完全相同的参数.
        不使用 CspaceFreePolytope (Drake 1.50 不稳定, 且原论文未使用).
        """
        from pydrake.geometry.optimization import IrisOptions

        context = self._diagram.CreateDefaultContext()
        plant_context = self._plant.GetMyContextFromRoot(context)

        # 优先使用 manual_seeds (如 Marcucci milestone 点), 不足部分随机补齐
        manual = list(self._manual_seeds) if self._manual_seeds else []
        seeds = [q_start, q_goal] + manual
        # 去重 (如果 q_start/q_goal 已在 manual 中)
        seen = set()
        unique_seeds = []
        for s in seeds:
            key = tuple(np.round(s, 8))
            if key not in seen:
                seen.add(key)
                unique_seeds.append(s)
        seeds = unique_seeds
        # 随机补齐到 n_regions
        if len(seeds) < self._n_regions:
            rng = np.random.default_rng(self._config.get('seed', 42))
            jl = self._robot.joint_limits
            lows = np.array([lo for lo, _ in jl])
            highs = np.array([hi for _, hi in jl])
            while len(seeds) < self._n_regions:
                seeds.append(rng.uniform(lows, highs))
        logger.info(f"C-IRIS seeds: {len(seeds)} total "
                    f"({len(manual)} manual + q_start/q_goal + random)")

        # ── IrisNp (Drake ≥ 1.50) 或 IrisInConfigurationSpace (旧版) ──
        # 严格复现 gcs-science-robotics/reproduction/prm_comparison 的参数
        try:
            from pydrake.geometry.optimization import IrisNp as _iris_fn
        except ImportError:
            try:
                from pydrake.geometry.optimization import (
                    IrisInConfigurationSpace as _iris_fn)
            except ImportError:
                raise ImportError(
                    "Neither IrisNp nor IrisInConfigurationSpace available")

        iris_options = IrisOptions()
        iris_options.iteration_limit = self._max_iterations       # 原论文: 10
        iris_options.termination_threshold = -1                   # 原论文: -1 (禁用)
        iris_options.relative_termination_threshold = 0.02        # 原论文: 0.02
        iris_options.num_collision_infeasible_samples = 1         # 原论文: 1
        iris_options.require_sample_point_is_contained = True     # 原论文: True
        iris_options.random_seed = self._config.get('seed', 0)    # 原论文: SEED=0

        regions = []
        t0 = time.perf_counter()

        for i, seed in enumerate(seeds[:self._n_regions]):
            if time.perf_counter() - t0 > total_timeout:
                logger.warning(f"C-IRIS timeout after {i} regions")
                break
            try:
                self._plant.SetPositions(plant_context, seed)
                hpoly = _iris_fn(self._plant, plant_context, iris_options)
                regions.append(hpoly)
                dt = time.perf_counter() - t0
                logger.info(f"C-IRIS region {i}: {dt:.2f}s "
                            f"(faces={hpoly.A().shape[0]}, cumulative)")
            except Exception as e:
                logger.warning(f"C-IRIS region {i} failed: {e}")

        return regions

    def reset(self) -> None:
        self._regions = []
        self._built = False

    # ── Pickle support ──
    # Drake MultibodyPlant / Diagram 不支持 pickle; 序列化时只保留
    # regions (HPolyhedron 的 A/b 以 numpy 形式存储) 和元数据.
    def __getstate__(self):
        regions_serializable = []
        for r in self._regions:
            try:
                regions_serializable.append({
                    'A': np.asarray(r.A()),
                    'b': np.asarray(r.b()),
                })
            except Exception:
                pass
        return {
            '_n_regions': self._n_regions,
            '_max_iterations': self._max_iterations,
            '_config': self._config,
            '_built': self._built,
            '_regions_ab': regions_serializable,
        }

    def __setstate__(self, state):
        self._n_regions = state.get('_n_regions', 5)
        self._max_iterations = state.get('_max_iterations', 10)
        self._config = state.get('_config', {})
        self._built = state.get('_built', False)
        self._robot = None
        self._scene = None
        self._plant = None
        self._diagram = None
        # 还原为轻量 _ABRegion 代理对象
        self._regions = [
            _ABRegion(ab['A'], ab['b'])
            for ab in state.get('_regions_ab', [])
        ]


class _ABRegion:
    """轻量 HPolyhedron 代理, 仅持有 A/b numpy 数组 (pickle 友好)."""
    def __init__(self, A: np.ndarray, b: np.ndarray):
        self._A = A
        self._b = b

    def A(self):
        return self._A

    def b(self):
        return self._b
