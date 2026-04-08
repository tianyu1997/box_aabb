# Phase M: Baselines + Metrics + Experiment 框架

> 依赖: Phase J (Python Bindings) + Phase K (可视化, 可选)
> 状态: ✅ 已完成 (2026-04-04)
> 产出: `python/sbf5_bench/` 包 — 基线规划器适配 + 路径质量评估 + 实验编排
> 预计: ~1200 LOC (Python)

---

## 目标

为论文实验构建完整的基准对比框架：
1. **Baselines**: OMPL (WSL subprocess) + IRIS-GCS (Drake) + 原始 RRT
2. **Metrics**: 路径长度、光滑度、间隙(clearance)、计算时间等
3. **Experiment Runner**: 多场景 × 多规划器 × 多随机种子 → 聚合结果

### 模块结构

```
python/sbf5_bench/
├── __init__.py
├── base.py              # BasePlanner ABC + PlanningResult
├── sbf_adapter.py       # SBF v5 → BasePlanner 适配器
├── ompl_adapter.py      # OMPL (WSL) → BasePlanner
├── iris_gcs_adapter.py  # IRIS + GCS (Drake) → BasePlanner
├── metrics.py           # 路径质量指标
├── runner.py            # 实验编排 + 聚合
├── scenes.py            # 标准测试场景库
└── report.py            # 结果表格 + LaTeX 输出
```

---

## Step M1: BasePlanner 抽象

### 文件
`python/sbf5_bench/base.py`

### 接口
```python
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import List, Optional, Dict
import numpy as np
import time

@dataclass
class PlanningResult:
    success: bool = False
    path: List[np.ndarray] = field(default_factory=list)
    cost: float = 0.0
    planning_time_s: float = 0.0
    collision_checks: int = 0
    nodes_explored: int = 0
    metadata: Dict = field(default_factory=dict)

    @staticmethod
    def failure(reason: str = "") -> "PlanningResult":
        return PlanningResult(success=False, metadata={"reason": reason})

    @property
    def n_waypoints(self) -> int:
        return len(self.path)

    def to_dict(self) -> dict:
        return {
            "success": self.success,
            "cost": self.cost,
            "planning_time_s": self.planning_time_s,
            "n_waypoints": self.n_waypoints,
            "collision_checks": self.collision_checks,
            "nodes_explored": self.nodes_explored,
            **self.metadata,
        }


class BasePlanner(ABC):
    @abstractmethod
    def setup(self, robot, scene, config: dict = None):
        """初始化规划器."""

    @abstractmethod
    def plan(self, start: np.ndarray, goal: np.ndarray,
             timeout: float = 30.0) -> PlanningResult:
        """执行规划."""

    def reset(self):
        """可选: 重置状态."""

    @property
    @abstractmethod
    def name(self) -> str:
        """规划器唯一名称."""

    @property
    def supports_reuse(self) -> bool:
        """是否支持多次 plan() 复用 (roadmap 类)."""
        return False
```

### 迁移来源
| 源文件 |
|--------|
| `v3/src/baselines/base.py` |

---

## Step M2: SBF v5 适配器

### 文件
`python/sbf5_bench/sbf_adapter.py`

### 接口
```python
from sbf5_bench.base import BasePlanner, PlanningResult
import sbf5

class SBFPlannerAdapter(BasePlanner):
    def __init__(self, use_gcs: bool = False):
        self._use_gcs = use_gcs
        self._planner = None

    @property
    def name(self) -> str:
        return "SBF-GCS" if self._use_gcs else "SBF-Dijkstra"

    @property
    def supports_reuse(self) -> bool:
        return True  # build() + query() 模式

    def setup(self, robot, scene, config=None):
        cfg = sbf5.SBFPlannerConfig()
        cfg.use_gcs = self._use_gcs
        if config:
            if "max_boxes" in config:
                cfg.grower.max_boxes = config["max_boxes"]
            if "n_roots" in config:
                cfg.grower.n_roots = config["n_roots"]
        self._planner = sbf5.SBFPlanner(robot, cfg)
        self._obstacles = scene

    def plan(self, start, goal, timeout=30.0):
        t0 = time.perf_counter()
        result = self._planner.plan(start, goal, self._obstacles,
                                    timeout_ms=timeout * 1000)
        dt = time.perf_counter() - t0
        return PlanningResult(
            success=result.success,
            path=[np.array(w) for w in result.path],
            cost=result.path_length,
            planning_time_s=dt,
            metadata={"n_boxes": result.n_boxes},
        )
```

---

## Step M3: OMPL 适配器 (WSL)

### 文件
`python/sbf5_bench/ompl_adapter.py`

### 设计
```python
import subprocess, json, tempfile

OMPL_ALGORITHMS = [
    "RRT", "RRTConnect", "RRTstar",
    "InformedRRTstar", "BITstar", "ABITstar",
]

class OMPLPlanner(BasePlanner):
    """通过 WSL subprocess 调用 OMPL C++ solver."""

    def __init__(self, algorithm: str = "RRTConnect",
                 wsl_binary: str = "sbf_ompl_bridge"):
        assert algorithm in OMPL_ALGORITHMS
        self._algo = algorithm
        self._binary = wsl_binary

    @property
    def name(self) -> str:
        return f"OMPL-{self._algo}"

    def setup(self, robot, scene, config=None):
        self._robot_data = robot
        self._scene_data = scene

    def plan(self, start, goal, timeout=30.0):
        # 序列化问题到 JSON
        problem = {
            "algorithm": self._algo,
            "robot": self._robot_data,   # serialized
            "obstacles": self._scene_data,
            "start": start.tolist(),
            "goal": goal.tolist(),
            "timeout": timeout,
        }

        with tempfile.NamedTemporaryFile(mode='w', suffix='.json', delete=False) as f:
            json.dump(problem, f)
            f.flush()
            # WSL 路径转换
            wsl_path = subprocess.check_output(
                ["wsl", "wslpath", f.name]
            ).decode().strip()

        # 调用 WSL OMPL bridge
        try:
            result_raw = subprocess.check_output(
                ["wsl", self._binary, wsl_path],
                timeout=timeout + 10
            )
            result = json.loads(result_raw)
            return PlanningResult(
                success=result["success"],
                path=[np.array(w) for w in result.get("path", [])],
                cost=result.get("cost", 0.0),
                planning_time_s=result.get("time", 0.0),
                collision_checks=result.get("collision_checks", 0),
            )
        except (subprocess.TimeoutExpired, subprocess.CalledProcessError):
            return PlanningResult.failure("OMPL timeout or crash")
```

### OMPL Bridge (C++, WSL 端)
- 独立的小型 C++ 程序, 链接 OMPL
- 读取 JSON → 配置 StateSpace + CollisionChecker → 调用 planner → 输出 JSON
- **不在 v5 核心仓库中** — 仅提供接口规范

### 迁移来源
| 源文件 |
|--------|
| `v3/src/baselines/ompl_adapter.py` |

---

## Step M4: IRIS-GCS 适配器

### 文件
`python/sbf5_bench/iris_gcs_adapter.py`

### 设计
```python
class IRISGCSPlanner(BasePlanner):
    """Drake IRIS region decomposition + GCS solver."""

    def __init__(self):
        self._drake_available = False
        try:
            import pydrake
            self._drake_available = True
        except ImportError:
            pass

    @property
    def name(self) -> str:
        return "IRIS-GCS"

    def setup(self, robot, scene, config=None):
        if not self._drake_available:
            raise RuntimeError("Drake not available")
        self._robot = robot
        self._scene = scene
        self._iris_regions = None  # lazy compute

    def plan(self, start, goal, timeout=30.0):
        from pydrake.geometry.optimization import (
            IrisInConfigurationSpace,
            GraphOfConvexSets,
            HPolyhedron,
            Point,
        )

        t0 = time.perf_counter()

        # 1. IRIS: 从种子配置生成凸区域
        if self._iris_regions is None:
            seeds = self._generate_seeds()
            self._iris_regions = []
            for seed in seeds:
                region = IrisInConfigurationSpace(
                    self._plant, self._context, seed,
                    IrisOptions(iteration_limit=10)
                )
                self._iris_regions.append(region)

        # 2. GCS: 构建图 + 求解
        gcs = GraphOfConvexSets()
        v_start = gcs.AddVertex(Point(start), "start")
        v_goal  = gcs.AddVertex(Point(goal),  "goal")

        region_verts = []
        for i, region in enumerate(self._iris_regions):
            v = gcs.AddVertex(region, f"iris_{i}")
            region_verts.append(v)

        # 添加边 (region 间有交集 → 连边)
        # ...

        result = gcs.SolveShortestPath(v_start, v_goal)
        dt = time.perf_counter() - t0

        if result.is_success():
            path = self._extract_path(result, gcs)
            return PlanningResult(
                success=True, path=path,
                cost=result.get_optimal_cost(),
                planning_time_s=dt,
            )
        return PlanningResult.failure("GCS solve failed")

    @property
    def supports_reuse(self) -> bool:
        return True  # IRIS regions cached
```

### 迁移来源
| 源文件 |
|--------|
| `v3/src/baselines/iris_gcs.py` |

---

## Step M5: 路径质量指标

### 文件
`python/sbf5_bench/metrics.py`

### 指标定义
```python
@dataclass
class PathMetrics:
    path_length: float = 0.0          # L2 distance (geodesic-aware)
    direct_distance: float = 0.0       # ||start - goal||₂
    efficiency: float = 0.0            # direct / path_length
    smoothness_mean: float = 0.0       # mean angle change between segments
    smoothness_max: float = 0.0        # max angle change
    clearance_min: float = 0.0         # min work-space distance to obstacles
    clearance_avg: float = 0.0         # avg work-space distance
    joint_range_usage: List[float] = field(default_factory=list)  # per-joint %
    n_waypoints: int = 0
```

### 计算函数
```python
def compute_path_length(path: List[np.ndarray],
                        joint_wrapping: List[bool] = None) -> float:
    """L2 path length, 支持周期关节 geodesic distance."""

def compute_smoothness(path: List[np.ndarray]) -> Tuple[float, float]:
    """返回 (mean_angle_change, max_angle_change)."""

def compute_clearance(path: List[np.ndarray],
                      robot, obstacles,
                      resolution: int = 20) -> Tuple[float, float]:
    """work-space 距离: FK 每个 link → 最近障碍物表面."""

def compute_joint_range_usage(path: List[np.ndarray],
                              joint_limits: List[Tuple[float, float]]) -> List[float]:
    """per-joint max-min / range."""

def evaluate_result(result: PlanningResult,
                    robot, obstacles) -> PathMetrics:
    """一次性计算所有指标."""

def compare_results(results: Dict[str, PlanningResult],
                    robot, obstacles) -> Dict[str, PathMetrics]:
    """多规划器横向比较."""

def format_comparison_table(metrics: Dict[str, PathMetrics]) -> str:
    """格式化为 Markdown/LaTeX 表格."""
```

### 迁移来源
| 源文件 |
|--------|
| `v3/src/planner/metrics.py` (~400 LOC) |

---

## Step M6: 标准测试场景库

### 文件
`python/sbf5_bench/scenes.py`

### 场景定义
```python
@dataclass
class BenchmarkScene:
    name: str
    robot_json: str                    # 相对路径
    obstacles: List[Dict]              # [{center, half_sizes}]
    start: np.ndarray
    goal: np.ndarray
    description: str = ""

# 内置场景
SCENES = {
    "2dof_simple": BenchmarkScene(
        name="2dof_simple",
        robot_json="data/2dof_planar.json",
        obstacles=[{"center": [1.0, 0.0, 0.0], "half_sizes": [0.2, 0.2, 0.2]}],
        start=np.array([0.1, 0.2]),
        goal=np.array([2.5, 1.5]),
        description="2DOF planar arm, single obstacle",
    ),
    "2dof_narrow": BenchmarkScene(
        name="2dof_narrow",
        robot_json="data/2dof_planar.json",
        obstacles=[
            {"center": [0.8, 0.0, 0.0], "half_sizes": [0.3, 1.0, 1.0]},
            {"center": [0.8, 0.0, 0.0], "half_sizes": [1.0, 0.3, 1.0]},
        ],
        start=np.array([0.5, 0.5]),
        goal=np.array([2.5, 2.0]),
        description="2DOF narrow passage",
    ),
    "panda_tabletop": BenchmarkScene(
        name="panda_tabletop",
        robot_json="data/panda.json",
        obstacles=[
            {"center": [0.5, 0.0, 0.4], "half_sizes": [0.3, 0.3, 0.02]},
            {"center": [0.5, 0.0, 0.2], "half_sizes": [0.05, 0.05, 0.2]},
        ],
        start=np.zeros(7),
        goal=np.array([0.5, -0.3, 0.0, -1.5, 0.0, 1.2, 0.0]),
        description="7DOF Panda with table + pillar",
    ),
}

def get_scene(name: str) -> BenchmarkScene:
    return SCENES[name]

def list_scenes() -> List[str]:
    return list(SCENES.keys())
```

---

## Step M7: 实验编排器

### 文件
`python/sbf5_bench/runner.py`

### 接口
```python
@dataclass
class ExperimentConfig:
    scenes: List[str]                  # scene names
    planners: List[BasePlanner]        # planner instances
    n_trials: int = 10                 # per (scene, planner)
    seeds: List[int] = None            # if None, range(n_trials)
    timeout: float = 30.0
    output_dir: str = "results/"

@dataclass
class TrialResult:
    scene: str
    planner: str
    seed: int
    trial_idx: int
    planning_result: PlanningResult
    metrics: PathMetrics

@dataclass
class ExperimentResults:
    trials: List[TrialResult]
    config: dict
    timestamp: str

    def save(self, path: str):
        """保存为 JSON."""

    @staticmethod
    def load(path: str) -> "ExperimentResults":
        """从 JSON 加载."""

    def summary_df(self):
        """返回 pandas DataFrame 汇总."""


def run_experiment(config: ExperimentConfig) -> ExperimentResults:
    """执行完整实验.

    for scene in scenes:
        for planner in planners:
            planner.setup(robot, obstacles)
            for seed in seeds:
                result = planner.plan(start, goal, timeout)
                metrics = evaluate_result(result, robot, obstacles)
                trials.append(TrialResult(...))
    """
```

### 特性
- 每个 trial 独立 (seed 控制随机性)
- 结果自动持久化 (JSON, 支持 NaN/inf)
- 支持断点续跑 (检查已有结果 → 跳过)

---

## Step M8: 结果报告

### 文件
`python/sbf5_bench/report.py`

### 接口
```python
def summary_table(results: ExperimentResults) -> str:
    """Markdown 汇总表格:
    | Planner | Scene | Success% | Time(s) | PathLen | Smooth | Clearance |
    """

def latex_table(results: ExperimentResults) -> str:
    """LaTeX booktabs 表格 (论文可用)."""

def plot_comparison(results: ExperimentResults) -> go.Figure:
    """Plotly bar chart: planners × metrics."""

def plot_time_vs_dof(results: ExperimentResults) -> go.Figure:
    """规划时间 vs DOF 曲线 (多场景)."""
```

---

## 测试

### 文件
`python/tests/test_bench.py`

| 用例 | 描述 |
|------|------|
| `test_planning_result_roundtrip` | PlanningResult → to_dict() → 重建 |
| `test_path_metrics_known` | 直线路径 → efficiency == 1.0 |
| `test_sbf_adapter_plan` | SBFPlannerAdapter + 2dof_simple → success |
| `test_runner_single_trial` | 单 scene × 单 planner × 1 trial 完成 |
| `test_summary_table_format` | summary_table 输出包含 header row |

---

## 依赖

| 包 | 必须? | 用途 |
|----|-------|------|
| numpy | 是 | 数组 |
| sbf5 | 是 | C++ 绑定 (Phase J) |
| plotly | 可选 | 可视化报告 |
| pandas | 可选 | DataFrame 汇总 |
| pydrake | 可选 | IRIS-GCS baseline |

---

## 验收标准

- [x] `SBFPlannerAdapter.plan()` 通过 2dof_simple 场景
- [x] `compute_path_length()` 对已知路径返回正确值
- [x] `run_experiment()` 框架完成 (save/load roundtrip 验证通过)
- [x] `summary_table()` 输出可读的 Markdown 表格
- [x] `latex_table()` 输出可编译的 LaTeX 代码
- [x] OMPL adapter 可降级 (WSL 不可用时返回 failure)
- [x] 18/18 pytest 测试通过
