# EXP3: 端到端规划 Benchmark (End-to-End Planning)

> 论文章节: §End-to-End Planning (sec:exp_e2e)
> 论文产出: **Table 3** (`tab3_e2e_planning.tex`) + **Figure 2** (`fig2_e2e_heatmap.pdf`)
> 代码入口: `run_all_experiments.py → run_s3()`
> 数据路径: `results/paper/s3_e2e/results.json`
> **这是论文的核心实验**

---

## 1. 实验目标

在 12 pipeline 配置 × 6 场景 × 30 seeds 下执行完整的
build→plan→smooth 循环, 测量端到端规划性能。

论文声称: "comprehensive 4×3 benchmark (12 pipeline configurations × 6 scenes × 30 seeds)"

---

## 2. 实验矩阵

### Pipeline 配置 (4×3 = 12)

| # | Endpoint Source | Envelope Type | 名称 |
|---|----------------|---------------|------|
| 1 | IFK | LinkIAABB | SBF-IFK-LinkIAABB |
| 2 | IFK | LinkIAABB_Grid | SBF-IFK-LinkIAABB_Grid |
| 3 | IFK | Hull16_Grid | SBF-IFK-Hull16_Grid |
| 4 | CritSample | LinkIAABB | SBF-CritSample-LinkIAABB |
| 5 | CritSample | LinkIAABB_Grid | SBF-CritSample-LinkIAABB_Grid |
| 6 | CritSample | Hull16_Grid | SBF-CritSample-Hull16_Grid |
| 7 | Analytical | LinkIAABB | SBF-Analytical-LinkIAABB |
| 8 | Analytical | LinkIAABB_Grid | SBF-Analytical-LinkIAABB_Grid |
| 9 | Analytical | Hull16_Grid | SBF-Analytical-Hull16_Grid |
| 10 | GCPC | LinkIAABB | SBF-GCPC-LinkIAABB |
| 11 | GCPC | LinkIAABB_Grid | SBF-GCPC-LinkIAABB_Grid |
| 12 | GCPC | Hull16_Grid | SBF-GCPC-Hull16_Grid |

### 场景 (6 个)

| # | 场景名 | Robot | DOF | 障碍 | 难度 |
|---|--------|-------|-----|------|------|
| 1 | `2dof_simple` | 2DOF planar | 2 | 1 上方 | 简单 |
| 2 | `2dof_narrow` | 2DOF planar | 2 | 2 水平 (窄通道) | 中等 |
| 3 | `2dof_cluttered` | 2DOF planar | 2 | 2 静态 | 中等 |
| 4 | `panda_tabletop` | Panda 7DOF | 7 | 1 远处 | 中等 |
| 5 | `panda_shelf` | Panda 7DOF | 7 | 1 角落 | 中等 |
| 6 | `panda_multi_obstacle` | Panda 7DOF | 7 | 2 对角 | 困难 |

### 参数设置

| 参数 | Quick | Lite | **Full** |
|------|-------|------|----------|
| 场景数 | 2 | 6 | **6** |
| Pipeline configs | 3 | 12 | **12** |
| n_trials (seeds) | 3 | 3 | **30** |
| timeout (s) | 60 | 60 | **60** |
| **总 trials** | 18 | 216 | **2,160** |

### 依赖数据
- `data/panda_5000.gcpc` — GCPC 配置需要 (若缺失则过滤掉 GCPC 类 3 个 config → 9 configs)
- 所有场景定义在 `python/sbf5_bench/scenes.py`
- Checkpoint 机制: 每 5 trials 保存 `_checkpoint.json`, 支持中断续跑

---

## 3. 实验流程

```
scenes = ["2dof_simple", "2dof_narrow", "2dof_cluttered",
          "panda_tabletop", "panda_shelf", "panda_multi_obstacle"]
pipeline_configs = ALL_PIPELINE_CONFIGS  # 12 个
n_trials = 30
timeout = 60.0 s

config = ExperimentConfig(
    scenes=scenes,
    pipeline_configs=pipeline_configs,
    gcpc_cache_path="data/panda_5000.gcpc",
    n_trials=30,
    timeout=60.0,
    output_dir="results/paper/s3_e2e/",
)

results = run_experiment(config)
# 迭代: scenes × planners × seeds
# 每个 trial: planner.plan(start, goal, timeout=60s)
# 每 5 trials: checkpoint 保存
```

---

## 4. 输出指标

### 每个 Trial 的 PlanningResult

| 指标 | 类型 | 含义 | 表格用 |
|------|------|------|--------|
| `success` | bool | 是否成功找到路径 | ✓ (success_rate) |
| `planning_time_s` | float (s) | 总规划时间 | ✓ |
| `cost` | float | 路径长度 | ✓ |
| `n_waypoints` | int | 路径 waypoint 数 | |
| `n_boxes` | int | 最终 box 数 | ✓ |
| `build_time_ms` | float (ms) | Forest 构建时间 | ✓ |
| `lect_time_ms` | float (ms) | LECT 构建时间 | |
| `envelope_volume_total` | float (m³) | 总 envelope 体积 | |

### 每个 Trial 的 PathMetrics

| 指标 | 类型 | 含义 |
|------|------|------|
| `path_length` | float | C-space L2 路径长度 |
| `direct_distance` | float | 起点→终点直线距离 |
| `efficiency` | float [0,1] | direct / path_length |
| `smoothness_mean` | float (rad) | 平均转折角 |
| `smoothness_max` | float (rad) | 最大转折角 |

---

## 5. Table 3 格式

```
Source / Envelope | 2D-S | 2D-N | 2D-C | P-Tab | P-Shf | P-Multi
IFK / LinkIAABB   | 100/8 | 100/8 | 100/8 | 100/23 | 100/23 | ...
...
```

每格: `success_rate% / mean_time_ms`
`---` 表示超时或未运行

### Figure 2: E2E Heatmap
- X 轴: 6 场景
- Y 轴: 12 pipeline configs
- 颜色: planning_time (log scale)
- 标注: success_rate%

---

## 6. 预期结果验证

| 检查点 | 预期 |
|--------|------|
| 2DOF 全部 12 configs | 100% success |
| IFK+LinkIAABB 2DOF | ~8 ms |
| IFK+Hull16_Grid 2DOF | ~13 ms |
| IFK+LinkIAABB panda | ~23 ms |
| IFK+Hull16_Grid panda | ~455 ms |
| CritSample panda | ~2800 ms |
| Analytical panda | ~18 s |
| GCPC panda | ~16 s |
| panda_multi_obstacle | success rate 下降 |

---

## 7. 当前状态与差距

| 项目 | 当前 | 目标 | 差距 |
|------|------|------|------|
| 场景数 | 2 (quick) | **6** | 缺 4 个 (2dof_cluttered, 3× panda) |
| Pipeline configs | 3 (quick) | **12** | 缺 9 个 |
| n_trials | 3 (quick) | **30** | 10× 更多 |
| 总 trials | 18 | **2,160** | 120× 更多 |
| panda 性能 | ✓ (已修复 grower) | 验证 | 需确认 60s timeout 不浪费 |

---

## 8. 执行命令

```powershell
# 删除旧 quick 结果
Remove-Item "results/paper/s3_e2e/results.json" -Force
Remove-Item "results/paper/s3_e2e/results_*.json" -Force

# Full 模式
# 预估: 2DOF ~9min + 7DOF ~90min = ~100min total
# Analytical + GCPC panda trials 贡献最多时间
python -c "
import sys; sys.path.insert(0, 'python')
from scripts.run_all_experiments import run_s3
run_s3(quick=False, lite=False)
"
```

---

## 9. 运行时间估算

| Config 类型 | 2DOF (×3 scenes ×30) | 7DOF (×3 scenes ×30) | 小计 |
|-------------|----------------------|----------------------|------|
| IFK (3 configs) | ~90 × 0.01s = ~1s | ~90 × 0.5s = ~45s | ~46s |
| CritSample (3) | ~90 × 0.05s = ~5s | ~90 × 3s = ~270s | ~275s |
| Analytical (3) | ~90 × 0.03s = ~3s | ~90 × 18s = ~1620s | ~1623s |
| GCPC (3) | ~90 × 0.02s = ~2s | ~90 × 16s = ~1440s | ~1442s |
| **总计** | ~11s | ~3375s (~56min) | **~57min** |

注: 上述估算基于 Table 3 现有数据。若 Analytical/GCPC panda 趋近 timeout (60s), 最坏情况:
- 90 trials × 60s = 5400s = 90min per Source
- 总计最坏: ~3.5 小时

---

## 10. 风险与注意事项

1. **panda grower 已修复**: boundary-empty early exit 确保 panda 不再超时
   - 但 Analytical/GCPC 的 FFB 本身慢, 每个 box 仍需 ~15ms endpoint
   - 500 boxes × 15ms = 7.5s 仅 endpoint → 加上 envelope, build 可能 ~18s
2. **Checkpoint 续跑**: 若中断, 重启会从 `_checkpoint.json` 续跑, 无需重头
3. **GCPC cache**: 若无 panda_5000.gcpc, 3 个 GCPC config 自动跳过 → 9 configs × 2160/12 = 1620 trials
4. **Table 3 中 `---`**: panda_shelf 某些 config 可能超时, 用 `---` 标记
5. **统计显著性**: 30 seeds 足够进行 Wilcoxon signed-rank test (α=0.05)
