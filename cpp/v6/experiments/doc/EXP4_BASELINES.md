# EXP4: 基线对比 (Baseline Comparison)

> 论文章节: §Baseline Comparison (sec:exp_baselines)
> 论文产出: **Table 4** (`tab4_baselines.tex`) + **Figure 3** (`fig3_baseline_pareto.pdf`)
> 代码入口: `run_all_experiments.py → run_s4()`
> 数据路径: `results/paper/s4_baselines/results.json`
> **Table 4 当前为 STUB (data not yet generated)**

---

## 1. 实验目标

将 SBF 推荐配置与 sampling-based baselines (OMPL) 进行对比。
目标是证明 SBF box-cover 方法在规划速度和路径质量上的优势。

论文声称对比:
- RRT, RRT-Connect, RRT*, Informed-RRT*, BiRRT*, PRM
- IRIS-NP-GCS, C-IRIS-GCS (Drake 可用时)

### 核心论点
> SBF achieves the highest success rate and fastest mean planning time
> across all scenes, with path lengths comparable to RRT*.
> The key advantage is deterministic coverage: once the forest is built,
> queries are guaranteed to find a path if one exists within the box connectivity graph.

---

## 2. 实验矩阵

### Planner 列表

| # | Planner | 类型 | 来源 | 状态 |
|---|---------|------|------|------|
| 1 | SBF-IFK-LinkIAABB | SBF | 本项目 | ✅ 可用 |
| 2 | SBF-Analytical-LinkIAABB | SBF | 本项目 | ✅ 可用 |
| 3 | SBF-GCPC-LinkIAABB_Grid | SBF | 本项目 | ⚠️ 需 GCPC cache |
| 4 | RRTConnect | OMPL | `ompl_adapter.py` | ⚠️ 需 OMPL 绑定 |
| 5 | RRT* | OMPL | `ompl_adapter.py` | ⚠️ 需 OMPL 绑定 |
| 6 | BIT* | OMPL | `ompl_adapter.py` | ⚠️ 需 OMPL 绑定 |
| 7 | IRIS-NP-GCS | Drake | 外部 | ❌ 未实现 |
| 8 | C-IRIS-GCS | Drake | 外部 | ❌ 未实现 |

### 场景

| 参数 | Quick | **Full** |
|------|-------|----------|
| 场景 | 2dof_simple, 2dof_narrow | **2dof_simple, 2dof_narrow, panda_tabletop, panda_shelf** |
| n_trials | 3 | **30** |
| timeout | 60 s | **60 s** |

### 总 trials (Full)
- 4 scenes × (3 SBF + 3 OMPL) × 30 seeds = **720** trials
- 若无 OMPL: 4 scenes × 3 SBF × 30 = **360** trials

---

## 3. 实验流程

```python
# SBF planners (S3 最优 2-3 个)
sbf_planners = [
    SBFPlannerAdapter("IFK", "LinkIAABB"),
    SBFPlannerAdapter("Analytical", "LinkIAABB"),
    SBFPlannerAdapter("GCPC", "LinkIAABB_Grid", gcpc_cache_path=...),
]

# OMPL baselines (需要 ompl_adapter.py + OMPL 安装)
ompl_planners = [
    OMPLPlanner("RRTConnect"),
    OMPLPlanner("RRTstar"),
    OMPLPlanner("BITstar"),
]

config = ExperimentConfig(
    scenes=["2dof_simple", "2dof_narrow", "panda_tabletop", "panda_shelf"],
    planners=sbf_planners + ompl_planners,
    n_trials=30,
    timeout=60.0,
)
```

---

## 4. 输出指标

| 指标 | 含义 | 表格用 |
|------|------|--------|
| `success_rate` | 成功规划百分比 | ✓ |
| `planning_time_s` | 总规划时间 (mean ± std) | ✓ |
| `path_length` | C-space 路径长度 | ✓ (cost) |
| `smoothness_mean` | 路径平滑度 (rad) | ✓ |
| `n_waypoints` | waypoint 数 | |

---

## 5. Table 4 格式

```
Planner        | 2D-S      | 2D-N      | P-Tab     | P-Shf
               | SR/T/Cost | SR/T/Cost | SR/T/Cost | SR/T/Cost
SBF-IFK-LIAABB | 100/8/2.1 | 100/8/3.5 | 100/23/-- | 100/23/--
RRTConnect     | 100/5/2.3 | 100/12/4.2| 95/45/--  | 80/55/--
RRT*           | 100/15/1.9| 100/30/3.0| 88/120/-- | 70/180/--
BIT*           | 100/20/1.8| 100/40/2.8| 90/90/--  | 75/120/--
```

### Figure 3: Pareto Front
- X 轴: planning_time_s (log)
- Y 轴: path_length (cost)
- 每个 planner 一个点 (mean) + error bars (±1 std)
- SBF 应位于 Pareto 前沿最快端

---

## 6. 预期结果

| 场景 | SBF 优势 | OMPL 优势 |
|------|----------|----------|
| 2DOF | SBF 更快 | RRT* 路径更优 |
| panda_tabletop | SBF 更快 + 更高 success | RRT* 路径更短 |
| panda_shelf | SBF deterministic success | RRTConnect 灵活适应 |

### 核心差异化
- **SBF**: build once → 多次 query, deterministic, 平滑路径
- **OMPL**: 无需预构建, 但每次 query 独立, 路径抖动大

---

## 7. 当前状态与差距

| 项目 | 当前 | 目标 | 差距 |
|------|------|------|------|
| SBF planners | 3 (✅) | 3 | 完成 |
| OMPL planners | 1 (RRTConnect only) | 3+ | 缺 RRT*, BIT* |
| IRIS-NP-GCS | ❌ | Table 4 中有 | 需 Drake 绑定 |
| 场景 | 2 (2dof only) | 4 | 缺 panda × 2 |
| n_trials | 3 (quick) | 30 | |
| Table 4 | STUB | 完整 | 需重跑 |

---

## 8. 关键前置条件

### OMPL 安装
```
# 方案 A: conda install ompl (Linux/WSL)
# 方案 B: pip install ompl (可能不支持 Windows)
# 方案 C: 通过 WSL subprocess 调用 OMPL
```

`ompl_adapter.py` 当前实现:
- 尝试 `from sbf5_bench.ompl_adapter import OMPLPlanner`
- ImportError 则跳过 OMPL → SBF-only comparison

### 若 OMPL 不可用
- S4 仍可运行, 仅包含 SBF 3 configs
- Table 4 退化为 SBF 内部对比
- 论文中 OMPL 数据可引用文献值 (标注 "reported by ...")

### Drake / IRIS-NP
- 论文提到 IRIS-NP-GCS 和 C-IRIS-GCS
- Table 4 现有 stub 未包含
- 需要 Drake 安装 + IRIS 绑定

---

## 9. 执行命令

```powershell
# 删除旧结果
Remove-Item "results/paper/s4_baselines/results.json" -Force
Remove-Item "results/paper/s4_baselines/results_*.json" -Force

# Full 模式 (若有 OMPL: ~60min, 无 OMPL: ~30min)
python -c "
import sys; sys.path.insert(0, 'python')
from scripts.run_all_experiments import run_s4
run_s4(quick=False, lite=False)
"
```

---

## 10. 风险与注意事项

1. **OMPL 不可用是最大风险**: Windows 原生无 OMPL, 需 WSL 或 conda
2. **Drake 不可用**: IRIS-NP-GCS 比较暂时无法实现
3. **SBF-only 退化**: 若只有 SBF, Table 4 变成 S3 子集, 价值降低
4. **公平性**: OMPL timeout 应与 SBF 一致 (60s), 且 OMPL 应使用 default params
5. **替代方案**: 若 OMPL 确实不可用, 可:
   - 引用 OMPL benchmark 文献数据
   - 在论文中注明 "SBF results only; OMPL baselines from [ref]"
   - 或在 WSL 中单独执行 OMPL 基线, 手动合并结果
