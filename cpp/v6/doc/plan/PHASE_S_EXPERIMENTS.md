# Phase S: 核心实验设计与执行

> 依赖: Phase R (实验基础设施扩展)
> 状态: 🔲 未开始
> 产出: 5 组实验 → results JSON + 中间分析数据
> 预计: ~350 LOC (Python 实验脚本)

---

## 目标

为新 T-RO 论文设计并执行 5 组核心实验:

| 实验 | 变量 | 论文产出 |
|------|------|----------|
| S1 | Envelope 紧密度 | Table 1: 12 config 的 volume 对比 |
| S2 | Envelope 计算耗时 | Table 2 + Figure 1: 计算速度 |
| S3 | 端到端规划 | Table 3 + Figure 2: 主实验 |
| S4 | Baseline 对比 | Table 4 + Figure 3: SBF vs OMPL |
| S5 | 可扩展性 | Figure 4: DOF/obstacles/budget scaling |

机器人: 2DOF planar (可视化) + 7DOF Panda (性能)

---

## Step S1: Envelope 紧密度对比

### 目的
量化 4 × 3 = 12 pipeline 配置产生的 envelope 紧密程度差异。
紧密 = envelope volume 小 (更接近真实扫掠体积)。

### 脚本
`python/scripts/exp_s1_envelope_tightness.py`

### 实验设计

```
对每个 robot (2dof, panda):
    加载 GCPC cache
    采样 N=500 个随机 box (joint intervals, width ∈ [0.1, 0.5] rad)
    对每个 box:
        对每个 endpoint_source ∈ {IFK, CritSample, Analytical, GCPC}:
            对每个 envelope_type ∈ {LinkIAABB, LinkIAABB_Grid, Hull16_Grid}:
                计算 endpoint IAABB → link envelope
                记录: total_volume, is_safe, computation_time_us
```

### 需要暴露的 C++ 函数 (Phase R5 或此处追加)

```python
# 方案 A: 通过现有 SBFPlanner 间接测量 (build 1 box forest)
# 方案 B: 直接暴露 compute_endpoint_iaabb + compute_link_envelope 到 Python
# 推荐方案 B — 更精确, 避免 planner 开销
```

方案 B 需在 `sbf5_bindings.cpp` 中追加:
```cpp
m.def("compute_endpoint_iaabb", [](
    const sbf::Robot& robot,
    const std::vector<sbf::Interval>& intervals,
    const sbf::EndpointSourceConfig& config) {
    return sbf::compute_endpoint_iaabb(robot, intervals, config);
}, py::arg("robot"), py::arg("intervals"), py::arg("config"));
```

### 输出指标

| 指标 | 含义 |
|------|------|
| `volume_mean` | 平均 envelope 体积 (m³) |
| `volume_std` | volume 标准差 |
| `volume_ratio` | 相对 IFK-LinkIAABB 基准的压缩率 |
| `time_us` | 平均计算耗时 (μs) |
| `is_safe` | 是否保守偏安全 (bool) |

### 输出文件
```
results/paper/s1_envelope_tightness/
    ├── 2dof_volumes.json     # 500 boxes × 12 configs
    ├── panda_volumes.json    # 500 boxes × 12 configs
    └── summary.json          # aggregated mean ± std
```

### 预期结果验证
- IFK 产生最大 volume (最保守)
- GCPC ≤ Analytical ≤ IFK (在 volume 上)
- Hull16_Grid volume ≤ LinkIAABB_Grid volume ≤ LinkIAABB volume
- 所有安全方法 (IFK, Analytical, GCPC) 的 `is_safe = True`
- CritSample 的 `is_safe = False`

---

## Step S2: Envelope 计算耗时

### 目的
测量各 endpoint source + envelope type 的计算速度。
论文关键数据: "GCPC 比 Analytical 快 N×"。

### 脚本
`python/scripts/exp_s2_envelope_timing.py`

### 实验设计

```
对每个 robot (2dof, panda):
    加载 GCPC cache
    采样 N=1000 个随机 box
    对每个 config (12种):
        计时: 重复 100 次，取 mean ± std
        区分: endpoint_time_us + envelope_time_us
```

### 输出指标

| 指标 | 含义 |
|------|------|
| `endpoint_time_us` | compute_endpoint_iaabb 耗时 |
| `envelope_time_us` | compute_link_envelope 耗时 |
| `total_time_us` | endpoint + envelope |
| `speedup_vs_analytical` | Analytical 耗时 / 当前耗时 |

### 输出文件
```
results/paper/s2_envelope_timing/
    ├── 2dof_timing.json
    ├── panda_timing.json
    └── summary.json
```

### 预期结果
- IFK 最快 (endpoint 阶段)
- Analytical Phase-3 最慢 (endpoint)
- GCPC 介于两者之间 (cache lookup + Phase-2 boundary)
- LinkIAABB 最快 (envelope 阶段)
- Hull16_Grid 最慢 (envelope, voxel rasterization)
- 总体最快组合: IFK + LinkIAABB
- 总体最优性价比: GCPC + LinkIAABB_Grid

---

## Step S3: 端到端规划 Benchmark (主实验)

### 目的
在 12 pipeline 配置 × 6 场景 × 30 seeds 下测量规划性能。
这是论文的**核心实验**。

### 脚本
`python/scripts/exp_s3_e2e_benchmark.py`

### 实验设计

```python
from sbf5_bench.runner import ExperimentConfig, ALL_PIPELINE_CONFIGS, run_experiment

config = ExperimentConfig(
    scenes=["2dof_simple", "2dof_narrow", "2dof_cluttered",
            "panda_tabletop", "panda_shelf", "panda_multi_obstacle"],
    planners=[],
    pipeline_configs=ALL_PIPELINE_CONFIGS,  # 12
    gcpc_cache_path="data/panda_5000.gcpc",
    n_trials=30,
    timeout=60.0,
    output_dir="results/paper/s3_e2e/",
)

results = run_experiment(config)
# Total: 12 × 6 × 30 = 2160 trials
```

### 输出指标

| 指标 | 含义 | 表格用 |
|------|------|--------|
| `success_rate` | 成功规划百分比 | ✓ |
| `planning_time_s` | 总规划时间 (mean ± std) | ✓ |
| `build_time_ms` | Forest 构建时间 | ✓ |
| `path_length` | C-space 路径长度 | ✓ |
| `smoothness_mean` | 路径平滑度 | ✓ |
| `n_boxes` | 最终 box 数 | ✓ |
| `efficiency` | 直线距离 / 路径长度 | (附录) |
| `lect_time_ms` | LECT 构建时间 | (分析用) |
| `envelope_volume_total` | 总 envelope 体积 | (分析用) |

### 输出文件
```
results/paper/s3_e2e/
    ├── results_*.json          # 完整 2160 trial 数据
    ├── _checkpoint.json        # 中间 checkpoint (运行中)
    └── summary.json            # aggregated
```

### 预期运行时间
- 2DOF scenes: ~0.5s/trial × 12 configs × 30 seeds × 3 scenes = ~540s (~9min)
- 7DOF scenes: ~5s/trial × 12 configs × 30 seeds × 3 scenes = ~5400s (~90min)
- **总计: ~100 min** (有 checkpoint, 可中断续跑)

### 预期结果
- 2DOF: 所有 12 configs 100% success
- 7DOF: success rate 随 scene 难度降低
  - panda_tabletop: ≥ 80% (所有 configs)
  - panda_shelf: ≥ 50% (best configs), 可能 < 30% (worst)
  - panda_multi_obstacle: ≥ 60%
- GCPC + LinkIAABB_Grid 应为最优性价比 (fast + tight)
- Hull16_Grid 最紧但最慢

---

## Step S4: Baseline 对比 (SBF vs OMPL)

### 目的
将 SBF (最优配置) 与 OMPL sampling-based planners 对比,
展示 C-space box cover 方法的优势。

### 脚本
`python/scripts/exp_s4_baselines.py`

### 实验设计

```python
from sbf5_bench.sbf_adapter import SBFPlannerAdapter
from sbf5_bench.ompl_adapter import OMPLPlanner

planners = [
    # SBF configs (S3 中表现最好的 2-3 个)
    SBFPlannerAdapter("GCPC", "LinkIAABB_Grid",
                      gcpc_cache_path="data/panda_5000.gcpc"),
    SBFPlannerAdapter("Analytical", "LinkIAABB"),

    # OMPL baselines (需 WSL)
    OMPLPlanner("RRTConnect"),
    OMPLPlanner("RRTstar"),
    OMPLPlanner("BITstar"),
]

config = ExperimentConfig(
    scenes=["2dof_simple", "2dof_narrow",
            "panda_tabletop", "panda_shelf"],
    planners=planners,
    n_trials=30,
    timeout=60.0,
    output_dir="results/paper/s4_baselines/",
)
```

### 前置条件
- WSL 已安装 OMPL (若未安装, S4 可跳过或用 mock 数据)
- `ompl_adapter.py` 通过 WSL subprocess 调用 OMPL

### 输出指标

| 指标 | 含义 |
|------|------|
| `success_rate` | 成功率 |
| `planning_time_s` | 规划时间 |
| `path_length` | 路径长度 |
| `smoothness_mean` | 平滑度 |
| `n_waypoints` | waypoint 数 (SBF 通常更少) |

### 输出文件
```
results/paper/s4_baselines/
    ├── results_*.json
    └── summary.json
```

### 预期结果
- SBF 规划时间与 RRTConnect 接近或更快 (2DOF)
- SBF path quality (smoothness) 优于 RRT/RRTConnect (box-based smooth)
- RRT* / BITstar 路径质量更高但时间更长
- SBF 的核心优势: **deterministic forest reuse** (build once, query many)
  - S4 可额外测试: build + N 次 query 的摊销时间

---

## Step S5: 可扩展性分析

### 目的
展示 SBF 方法在不同维度、障碍数、box 预算下的 scaling 行为。

### 脚本
`python/scripts/exp_s5_scalability.py`

### 实验 S5a: DOF Scaling

```
Robot: 2DOF vs 7DOF (在相似难度场景下)
Config: GCPC + LinkIAABB_Grid (S3 最优)
场景: simple (1 obstacle), narrow (2 obstacles)
Seeds: 30
Metric: planning_time_s, n_boxes, success_rate
```

### 实验 S5b: Obstacle Count Scaling

```
Robot: 7DOF Panda
Obstacles: 1, 2, 3, 4, 6, 8 (递增)
Config: GCPC + LinkIAABB_Grid
Seeds: 30

构造方式: 在 panda_tabletop 基础上逐步添加随机散布障碍
```

需要新增 scene 或在脚本中动态构造:
```python
def make_obstacle_sweep_scenes(n_obs_list: list[int]) -> list[BenchmarkScene]:
    """Generate scenes with increasing obstacle count."""
    import sbf5
    base_obs = [
        {"center": [0.5, 0.0, 0.4], "half_sizes": [0.3, 0.3, 0.02]},
    ]
    extra_obs = [
        {"center": [0.4, 0.2, 0.3], "half_sizes": [0.08, 0.08, 0.08]},
        {"center": [0.6, -0.2, 0.5], "half_sizes": [0.06, 0.06, 0.06]},
        {"center": [0.3, 0.3, 0.6], "half_sizes": [0.1, 0.05, 0.05]},
        {"center": [0.7, -0.1, 0.3], "half_sizes": [0.05, 0.1, 0.1]},
        {"center": [0.2, -0.3, 0.4], "half_sizes": [0.07, 0.07, 0.07]},
        {"center": [0.55, 0.15, 0.55], "half_sizes": [0.04, 0.04, 0.04]},
        {"center": [0.45, -0.25, 0.35], "half_sizes": [0.09, 0.06, 0.08]},
    ]
    scenes = []
    for n in n_obs_list:
        obs = base_obs + extra_obs[:n-1]
        scenes.append(BenchmarkScene(
            name=f"panda_{n}obs",
            robot_json="panda.json",
            obstacles=obs[:n],
            start=np.zeros(7),
            goal=np.array([0.5, -0.3, 0.0, -1.5, 0.0, 1.2, 0.0]),
            description=f"7DOF Panda, {n} obstacles",
        ))
    return scenes
```

### 实验 S5c: Box Budget Scaling

```
Robot: 7DOF Panda
Scene: panda_tabletop (固定)
Config: GCPC + LinkIAABB_Grid
max_boxes: [50, 100, 200, 500, 1000, 2000]
Seeds: 30
Metric: success_rate, planning_time_s, path_length
```

### 输出文件
```
results/paper/s5_scalability/
    ├── s5a_dof.json
    ├── s5b_obstacles.json
    ├── s5c_budget.json
    └── summary.json
```

### 预期结果
- **S5a**: 7DOF 比 2DOF 慢 ~10-50×, n_boxes 增长 ~5-10×
- **S5b**: 障碍增加 → success_rate 降低, planning_time 增加 (约线性)
- **S5c**: 
  - max_boxes=50: 低 success rate
  - max_boxes=200-500: 甜蜜点 (success ≥ 80%, 速度适中)
  - max_boxes=2000: 高 success 但速度慢, path quality 边际递减

---

## 目录总览

```
results/paper/
├── s1_envelope_tightness/
│   ├── 2dof_volumes.json
│   ├── panda_volumes.json
│   └── summary.json
├── s2_envelope_timing/
│   ├── 2dof_timing.json
│   ├── panda_timing.json
│   └── summary.json
├── s3_e2e/
│   ├── results_*.json    (2160 trials, ~5MB)
│   └── summary.json
├── s4_baselines/
│   ├── results_*.json
│   └── summary.json
└── s5_scalability/
    ├── s5a_dof.json
    ├── s5b_obstacles.json
    ├── s5c_budget.json
    └── summary.json
```

---

## 测试 (单元测试, 非实验本身)

| 用例 | 文件 | 描述 |
|------|------|------|
| `test_s1_mini` | `python/tests/test_experiments.py` | S1 跑 2DOF × 10 boxes × 2 configs → JSON 输出 |
| `test_s3_mini` | `python/tests/test_experiments.py` | S3 跑 1 scene × 2 configs × 2 seeds → 4 trials |
| `test_s5c_mini` | `python/tests/test_experiments.py` | S5c 跑 2 budgets × 1 seed → 2 trials |

---

## 验收标准

- [ ] S1: 12 × 2 robot 的 volume 数据完整, JSON 可加载
- [ ] S2: timing 数据 mean ± std 合理 (IFK < 100μs, Analytical > 500μs on 7DOF)
- [ ] S3: 2160 trials 全部完成 (可中断续跑)
- [ ] S3: 所有 2DOF scene 的 success_rate = 100% (IFK/Analytical/GCPC)
- [ ] S4: SBF vs RRTConnect 对比数据完整 (或标注 OMPL 不可用)
- [ ] S5a: 2DOF/7DOF 对比数据产出
- [ ] S5b: 1-8 个障碍的 scaling 曲线数据
- [ ] S5c: 50-2000 budget 的 trade-off 数据
- [ ] 所有 JSON 可被 Phase T 的 report 函数正确加载

---

## 文件改动清单

| 文件 | 类型 | 改动量 |
|------|------|--------|
| `python/scripts/exp_s1_envelope_tightness.py` | 新建 | ~80 LOC |
| `python/scripts/exp_s2_envelope_timing.py` | 新建 | ~80 LOC |
| `python/scripts/exp_s3_e2e_benchmark.py` | 新建 | ~40 LOC |
| `python/scripts/exp_s4_baselines.py` | 新建 | ~50 LOC |
| `python/scripts/exp_s5_scalability.py` | 新建 | ~100 LOC |
| `python/tests/test_experiments.py` | 新建 | ~50 LOC |
| **总计** | | **~400 LOC** |
