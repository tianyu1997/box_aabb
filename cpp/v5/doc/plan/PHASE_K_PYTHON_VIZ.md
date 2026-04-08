# Phase K: Python 可视化 (sbf5_viz)

> 依赖: Phase I (VizExporter) 或 Phase J (Python Bindings) — 至少一个
> 状态: ✅ 已完成 (2026-04-04)
> 产出: `python/sbf5_viz/` 包 — Plotly 交互式 3D 可视化
> 实际: ~750 LOC (8 个模块 + tests)

---

## 目标

构建 Python 可视化包，支持：
1. **离线模式**: 读取 Phase I 导出的 JSON → 生成 Plotly HTML (无需编译 pybind11)
2. **在线模式**: 通过 Phase J 的 Python bindings 直接获取数据 → 实时渲染
3. **论文配图**: 可视化比较多种 envelope 方法、多规划器结果

### 与 v4 的差异
| 方面 | v4 (`sbf4_viz`) | v5 (`sbf5_viz`) |
|------|-----------------|-----------------|
| 数据来源 | 仅 JSON 文件 | JSON 或 pybind11 |
| 框架 | Plotly | Plotly (不变) |
| Voxel 可视化 | 是 (BitBrick hex) | Phase N 后支持 |
| 模块数 | 8 | 8 (重构) |

---

## 模块结构

```
python/sbf5_viz/
├── __init__.py          # 公共 API
├── load_data.py         # JSON 加载 + data classes
├── robot_viz.py         # FK link-chain 3D 渲染
├── envelope_viz.py      # iAABB box wireframe/filled
├── forest_viz.py        # C-space box 可视化 (2D/3D 投影)
├── scene_viz.py         # 障碍物 box 渲染
├── combined_viz.py      # 多层叠加统一视图
├── comparison_viz.py    # 多方法/多规划器对比
└── cli.py               # 命令行入口
```

---

## Step K1: 数据加载层

### 文件
`python/sbf5_viz/load_data.py`

### Data Classes
```python
from dataclasses import dataclass, field
from typing import List, Optional
import numpy as np

@dataclass
class RobotData:
    name: str
    n_joints: int
    link_radii: List[float]
    configs: List[dict]        # [{q: ndarray, link_positions: ndarray}]

@dataclass
class EnvelopeData:
    method: str
    boxes: List[dict]          # [{box_id, intervals, links: [{link_idx, aabb}]}]

@dataclass
class SceneData:
    obstacles: List[dict]      # [{center, half_sizes}]

@dataclass
class ForestData:
    n_boxes: int
    boxes: List[dict]          # [{id, intervals, volume}]

@dataclass
class SnapshotData:
    robot: Optional[RobotData] = None
    envelope: Optional[EnvelopeData] = None
    scene: Optional[SceneData] = None
    forest: Optional[ForestData] = None
```

### 加载函数
```python
def load_json(path: str) -> dict:
    """加载 JSON (by Phase I exporter)."""

def load_snapshot(path: str) -> SnapshotData:
    """加载 snapshot JSON → SnapshotData."""

def from_planner(planner, robot, obstacles, start, goal) -> SnapshotData:
    """在线模式: 从 pybind11 SBFPlanner 直接获取数据."""
```

### 迁移来源
| 源文件 |
|--------|
| `v4/python/sbf4_viz/load_data.py` |

---

## Step K2: Robot 可视化

### 文件
`python/sbf5_viz/robot_viz.py`

### 接口
```python
def plot_robot_3d(robot_data: RobotData, config_idx: int = 0) -> go.Figure:
    """绘制单个配置的 FK link chain (3D scatter + lines)."""

def plot_robot_multi_configs(robot_data: RobotData,
                             indices: List[int] = None) -> go.Figure:
    """叠加多组配置 (半透明渐变)."""
```

### 渲染方式
- 每个 link: 球体 (关节) + 圆柱 (连杆)
- 多 config: 用 opacity 0.2~1.0 渐变
- 颜色方案: 沿 link index 变化 (Plotly `Viridis`)

---

## Step K3: Envelope 可视化

### 文件
`python/sbf5_viz/envelope_viz.py`

### 接口
```python
def plot_envelope_wireframe(envelope_data: EnvelopeData,
                            link_filter: List[int] = None) -> go.Figure:
    """每个 link iAABB 画 wireframe box."""

def plot_envelope_filled(envelope_data: EnvelopeData,
                         opacity: float = 0.15) -> go.Figure:
    """半透明 filled box (适合论文截图)."""

def add_envelope_traces(fig: go.Figure,
                        envelope_data: EnvelopeData,
                        name: str = "envelope",
                        visible: bool = True) -> None:
    """向已有 fig 添加 envelope traces (用于 combined)."""
```

### 渲染方式
- wireframe: `go.Mesh3d` 12 条边
- filled: `go.Mesh3d` with opacity
- 每个 link 独立颜色

---

## Step K4: Forest 可视化

### 文件
`python/sbf5_viz/forest_viz.py`

### 接口
```python
def plot_forest_2d(forest_data: ForestData,
                   dim_x: int = 0, dim_y: int = 1) -> go.Figure:
    """2D 投影: 选择两个 joint 轴画 box 矩形."""

def plot_forest_3d(forest_data: ForestData,
                   dims: List[int] = [0, 1, 2]) -> go.Figure:
    """3D 投影 (限 ≥3 DOF)."""

def plot_path_on_forest(forest_data: ForestData,
                        path: List[np.ndarray],
                        dim_x: int = 0, dim_y: int = 1) -> go.Figure:
    """叠加 path waypoints 在 forest 上."""
```

### 特性
- 2DOF 机器人: 直接画全部 box (无投影损失)
- 高维: 投影到选定 2/3 维, box 边被截断
- 路径: 折线 + waypoint 标记

---

## Step K5: Scene 可视化

### 文件
`python/sbf5_viz/scene_viz.py`

### 接口
```python
def plot_scene_3d(scene_data: SceneData) -> go.Figure:
    """3D 障碍物 box (灰色半透明)."""

def add_scene_traces(fig: go.Figure, scene_data: SceneData) -> None:
    """向已有 fig 添加 obstacle traces."""
```

---

## Step K6: 组合视图

### 文件
`python/sbf5_viz/combined_viz.py`

### 接口
```python
def plot_combined(snapshot: SnapshotData,
                  show_robot: bool = True,
                  show_envelope: bool = True,
                  show_scene: bool = True,
                  show_forest: bool = False) -> go.Figure:
    """多层叠加 3D 视图, 每层可通过 Plotly 按钮 toggle."""
```

### 特性
- Plotly `updatemenus` 按钮: Toggle robot / envelope / scene / forest 层
- 等比例坐标轴 (`aspectmode='data'`)
- 统一 colorscale

---

## Step K7: 对比可视化

### 文件
`python/sbf5_viz/comparison_viz.py`

### 接口
```python
def compare_envelopes(envelopes: Dict[str, EnvelopeData]) -> go.Figure:
    """多种 envelope 方法并排/叠加对比 (论文用)."""

def compare_planners(results: Dict[str, PlanResult],
                     forest_data: ForestData) -> go.Figure:
    """多规划器路径叠加 (SBF vs OMPL vs GCS)."""
```

### 迁移来源
| 源文件 |
|--------|
| `v4/python/sbf4_viz/envelope_comparison_viz.py` |
| `v4/python/sbf4_viz/ep_iaabb_viz.py` |

---

## Step K8: CLI 入口

### 文件
`python/sbf5_viz/cli.py`

### 用法
```bash
# 离线: 从 JSON 生成 HTML
python -m sbf5_viz output/snapshot.json --html output/viz.html

# 在线: 从 robot JSON + plan 实时可视化
python -m sbf5_viz data/2dof_planar.json --plan --start "0.1,0.2" --goal "2.0,1.5"
```

### 实现
```python
import argparse

def main():
    parser = argparse.ArgumentParser(description="SBF v5 Visualizer")
    parser.add_argument("input", help="JSON file or robot JSON")
    parser.add_argument("--html", default=None, help="Output HTML path")
    parser.add_argument("--plan", action="store_true", help="Run planner online")
    parser.add_argument("--start", help="Start config (comma-separated)")
    parser.add_argument("--goal", help="Goal config (comma-separated)")
    args = parser.parse_args()

    if args.plan:
        # 在线模式: import sbf5, plan, visualize
        ...
    else:
        # 离线模式: load JSON, visualize
        snapshot = load_snapshot(args.input)
        fig = plot_combined(snapshot)
        if args.html:
            fig.write_html(args.html)
        else:
            fig.show()
```

---

## 依赖

| 包 | 版本 | 用途 |
|----|------|------|
| plotly | ≥ 5.0 | 3D 交互式渲染 |
| numpy | ≥ 1.20 | 数组操作 |
| sbf5 (可选) | — | 在线模式 (Phase J) |

### `python/sbf5_viz/requirements.txt`
```
plotly>=5.0
numpy>=1.20
```

---

## 测试

### 文件
`python/tests/test_viz.py`

| 用例 | 描述 |
|------|------|
| `test_load_snapshot_json` | 加载样例 JSON → SnapshotData 字段正确 |
| `test_robot_viz_returns_figure` | `plot_robot_3d()` → 返回 go.Figure |
| `test_combined_viz_layers` | `plot_combined()` → Figure 有 ≥ 2 traces |
| `test_cli_offline` | CLI 读取 JSON → 生成 HTML 文件 |

---

## 验收标准

- [ ] `python -m sbf5_viz snapshot.json --html out.html` 生成可交互 HTML
- [ ] HTML 打开后可 rotate/zoom, 有 toggle 按钮
- [ ] 2DOF forest 可视化正确显示全部 box
- [ ] 论文级配图: 分辨率 ≥ 1080p, 字体清晰, legend 可读
- [ ] 无 sbf5 pybind11 时离线模式仍可工作 (仅依赖 JSON)
