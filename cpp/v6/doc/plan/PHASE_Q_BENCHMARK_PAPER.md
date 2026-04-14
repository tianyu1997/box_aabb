# Phase Q: Benchmark 运行 + 论文就绪

> 依赖: Phase O (Python 验证) + Phase P (功能补全)
> 状态: 🔲 未开始
> 产出: 完整 benchmark 数据 + LaTeX 表格 + 论文配图 + 新场景
> 预计: ~400 LOC (Python 场景/脚本) + 实验运行

---

## 目标

在全部代码、测试、Python bindings 就绪后，运行真实 benchmark 实验，产出论文所需的：
1. 多场景 × 多规划器 × 多 seed 的量化结果
2. LaTeX 格式的汇总表格 (booktabs)
3. 交互式 Plotly 配图 + 静态 PNG 导出
4. 端到端 C++ → JSON → Python → HTML 可视化验证

---

## Step Q1: 扩充 BenchmarkScene 库

### 文件
修改: `python/sbf5_bench/scenes.py`

### 新增场景

```python
# 追加到 SCENES dict

"2dof_cluttered": BenchmarkScene(
    name="2dof_cluttered",
    robot_json="data/2dof_planar.json",
    obstacles=[
        {"center": [0.8, 0.3, 0.0], "half_sizes": [0.15, 0.15, 0.15]},
        {"center": [1.2, -0.2, 0.0], "half_sizes": [0.1, 0.1, 0.1]},
        {"center": [1.5, 0.5, 0.0], "half_sizes": [0.2, 0.1, 0.1]},
        {"center": [0.5, -0.5, 0.0], "half_sizes": [0.1, 0.2, 0.1]},
    ],
    start=np.array([0.3, 0.3]),
    goal=np.array([2.8, 2.5]),
    description="2DOF cluttered: 4 obstacles in workspace",
),

"panda_shelf": BenchmarkScene(
    name="panda_shelf",
    robot_json="data/panda.json",
    obstacles=[
        # 书架结构: 两个竖板 + 两个横板
        {"center": [0.5, -0.3, 0.5], "half_sizes": [0.02, 0.2, 0.3]},
        {"center": [0.5, 0.3, 0.5],  "half_sizes": [0.02, 0.2, 0.3]},
        {"center": [0.5, 0.0, 0.3],  "half_sizes": [0.02, 0.3, 0.02]},
        {"center": [0.5, 0.0, 0.7],  "half_sizes": [0.02, 0.3, 0.02]},
    ],
    start=np.zeros(7),
    goal=np.array([0.8, 0.5, 0.0, -1.0, 0.0, 1.5, 0.5]),
    description="7DOF Panda reaching into shelf",
),

"panda_multi_obstacle": BenchmarkScene(
    name="panda_multi_obstacle",
    robot_json="data/panda.json",
    obstacles=[
        {"center": [0.4, 0.0, 0.3],  "half_sizes": [0.1, 0.1, 0.1]},
        {"center": [0.6, 0.2, 0.5],  "half_sizes": [0.08, 0.08, 0.08]},
        {"center": [0.3, -0.3, 0.4], "half_sizes": [0.12, 0.06, 0.15]},
    ],
    start=np.zeros(7),
    goal=np.array([1.0, -0.5, 0.0, -2.0, 0.0, 1.8, 0.0]),
    description="7DOF Panda with 3 scattered obstacles",
),
```

### 场景覆盖度

| 场景 | DOF | 障碍数 | 难度 | 用途 |
|------|-----|--------|------|------|
| 2dof_simple | 2 | 1 | 简单 | 回归/冒烟测试 |
| 2dof_narrow | 2 | 2 | 窄通道 | 规划质量对比 |
| 2dof_cluttered | 2 | 4 | 多障碍 | 扩展性 |
| panda_tabletop | 7 | 2 | 中等 | 基础 7DOF |
| panda_shelf | 7 | 4 | 困难 | 受限空间 |
| panda_multi_obstacle | 7 | 3 | 中等 | 散布障碍 |

---

## Step Q2: 运行 SBF Benchmark

### 脚本
`python/scripts/run_benchmark.py`

```python
#!/usr/bin/env python3
"""Run SBF v5 benchmark suite."""
import sys
sys.path.insert(0, "python")

from sbf5_bench.runner import run_experiment, ExperimentConfig
from sbf5_bench.sbf_adapter import SBFPlannerAdapter
from sbf5_bench.report import summary_table, latex_table, plot_comparison

def main():
    planners = [
        SBFPlannerAdapter(use_gcs=False),   # SBF-Dijkstra
    ]

    config = ExperimentConfig(
        scenes=["2dof_simple", "2dof_narrow", "2dof_cluttered",
                "panda_tabletop"],
        planners=planners,
        n_trials=10,
        timeout=60.0,
        output_dir="results/benchmark_v5",
    )

    results = run_experiment(config)
    results.save("results/benchmark_v5/results.json")

    # 打印 Markdown 表格
    print(summary_table(results))

    # 导出 LaTeX
    with open("results/benchmark_v5/table.tex", "w") as f:
        f.write(latex_table(results))

    # 生成对比图
    fig = plot_comparison(results)
    fig.write_html("results/benchmark_v5/comparison.html")

if __name__ == "__main__":
    main()
```

### 运行命令
```powershell
cd v5
$env:PYTHONPATH = "build_x64/Release;python"
python python/scripts/run_benchmark.py
```

---

## Step Q3: C++ → JSON → HTML 全链路验证

### 脚本
`python/scripts/run_viz_demo.py`

```python
#!/usr/bin/env python3
"""End-to-end visualization demo: C++ plan → JSON export → Plotly HTML."""
import sys, os
sys.path.insert(0, "python")

import numpy as np
import sbf5
from sbf5_viz import load_snapshot, plot_combined

def main():
    # 1. C++ planning
    robot = sbf5.Robot.from_json("data/2dof_planar.json")
    obs = sbf5.Obstacle()
    obs.center = np.array([1.0, 0.0, 0.0])
    obs.half_sizes = np.array([0.2, 0.2, 0.2])

    planner = sbf5.SBFPlanner(robot)
    result = planner.plan(
        np.array([0.1, 0.2]),
        np.array([2.5, 1.5]),
        [obs], timeout_ms=30000.0
    )
    assert result.success, "Planning failed!"
    print(f"Plan: {result.n_boxes} boxes, {len(result.path)} waypoints")

    # 2. JSON export (via C++ viz exporter called from Python)
    #    → 或者直接构造 SnapshotData from Python
    from sbf5_viz.load_data import SnapshotData, RobotData, ForestData, SceneData

    # 构造 snapshot data from plan result
    forest_boxes = []
    for b in planner.boxes():
        forest_boxes.append({
            "id": b.id,
            "intervals": [[iv.lo, iv.hi] for iv in b.intervals],
            "volume": b.volume,
        })

    snapshot = SnapshotData(
        forest=ForestData(n_boxes=result.n_boxes, boxes=forest_boxes),
        scene=SceneData(obstacles=[
            {"center": [1.0, 0.0, 0.0], "half_sizes": [0.2, 0.2, 0.2]}
        ]),
    )

    # 3. Plotly visualization
    fig = plot_combined(snapshot, show_robot=False, show_envelope=False,
                        show_scene=True, show_forest=True)

    # 添加 path
    import plotly.graph_objects as go
    path_q0 = [w[0] for w in result.path]
    path_q1 = [w[1] for w in result.path]
    fig.add_trace(go.Scatter(
        x=path_q0, y=path_q1,
        mode="lines+markers", name="SBF Path",
        line=dict(color="red", width=3),
        marker=dict(size=8),
    ))

    os.makedirs("results/viz_demo", exist_ok=True)
    fig.write_html("results/viz_demo/2dof_plan.html")
    print("Saved: results/viz_demo/2dof_plan.html")

if __name__ == "__main__":
    main()
```

---

## Step Q4: LaTeX 表格格式确认

### 预期输出 (booktabs 格式)
```latex
\begin{table}[ht]
\centering
\caption{SBF v5 Benchmark Results}
\label{tab:benchmark}
\begin{tabular}{llcccccc}
\toprule
Scene & Planner & Success\% & Time (s) & PathLen & Smooth & Clearance & Boxes \\
\midrule
2dof\_simple  & SBF-Dijkstra & 100.0 & 0.12 & 3.45 & 0.89 & 0.15 & 150 \\
2dof\_narrow  & SBF-Dijkstra & 90.0  & 0.35 & 5.12 & 0.72 & 0.08 & 280 \\
2dof\_cluttered & SBF-Dijkstra & 100.0 & 0.18 & 4.20 & 0.81 & 0.12 & 200 \\
panda\_tabletop & SBF-Dijkstra & 80.0 & 2.50 & 8.30 & 0.65 & 0.05 & 1500 \\
\bottomrule
\end{tabular}
\end{table}
```

### 验证
- `report.latex_table()` 输出可直接粘贴到 `.tex` 文件
- 数值精度: 时间 2 位小数, 路径长度 2 位, 百分比 1 位

---

## Step Q5: 静态图片导出 (论文用)

### 依赖
```bash
pip install kaleido    # Plotly 静态导出后端
```

### 导出脚本片段
```python
# 在 run_benchmark.py 末尾追加:
fig = plot_comparison(results)
fig.write_image("results/benchmark_v5/comparison.pdf", width=800, height=400)
fig.write_image("results/benchmark_v5/comparison.png", width=1600, height=800, scale=2)

# 在 run_viz_demo.py 末尾追加:
fig.write_image("results/viz_demo/2dof_plan.pdf", width=600, height=600)
```

### 输出格式
| 文件 | 用途 |
|------|------|
| `comparison.pdf` | 论文 Figure (矢量) |
| `comparison.png` | Slides (高分辨率位图) |
| `table.tex` | 论文 Table |
| `2dof_plan.html` | 交互式 demo |
| `results.json` | 原始数据存档 |

---

## Step Q6: 多规划器对比 (可选, 需 WSL/Drake)

### 扩展 benchmark
```python
# 如果 OMPL 可用:
from sbf5_bench.ompl_adapter import OMPLPlanner
planners.append(OMPLPlanner("RRTConnect"))
planners.append(OMPLPlanner("RRTstar"))

# 如果 Drake 可用:
from sbf5_bench.iris_gcs_adapter import IRISGCSPlanner
planners.append(IRISGCSPlanner())
```

### 对比图
```python
fig = compare_planners(
    {"SBF-Dijkstra": sbf_result, "RRTConnect": ompl_result},
    forest_data=forest_data
)
```

---

## 测试

### 自动化验证
`python/tests/test_benchmark_run.py` (集成测试, 非日常运行)

| 用例 | 描述 |
|------|------|
| `test_single_scene_benchmark` | 1 scene × 1 planner × 1 trial → 产出 JSON |
| `test_latex_output_compiles` | latex_table() 输出包含 `\begin{table}` |
| `test_plot_returns_figure` | plot_comparison() → go.Figure 有 traces |
| `test_results_save_load` | save → load → trials 数量一致 |

---

## 验收标准

- [ ] 6 个 BenchmarkScene 全部可运行 (2DOF 100% success, 7DOF ≥ 50%)
- [ ] `results.json` 包含所有 trial 数据
- [ ] `table.tex` 可被 `pdflatex` 编译
- [ ] `comparison.html` 可交互 (旋转/缩放/hover)
- [ ] `comparison.pdf` 矢量图清晰
- [ ] `2dof_plan.html` 显示 forest + path + obstacles
- [ ] 全链路: C++ plan → Python → Plotly → HTML/PDF 无断裂
- [ ] Checkpoint/resume: 中断后可续跑
