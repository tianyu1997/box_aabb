# EXP7: 可扩展性分析 (Scalability Analysis)

> 论文章节: §Scalability (sec:exp_scalability)
> 论文产出: **Table 7** (`tab7_subdivision.tex`) + **Figure 4** (`fig4_scalability.pdf`)
> 代码入口: `run_all_experiments.py → run_s5()`
> 数据路径: `results/paper/s5_scalability/results.json`
> Table 7 (subdivision) 已有数据 ✅; Figure 4 数据来自 S5a/b/c

---

## 1. 实验目标

从三个维度分析 SBF 方法的可扩展性:
- **S5a: DOF 缩放** — 2DOF vs 7DOF 性能对比
- **S5b: 障碍物数量缩放** — 1→8 个障碍的 success rate 和 time
- **S5c: Box 预算缩放** — max_boxes 从 50→2000 的影响

另外, 本节还包含:
- **Table 7: 连杆细分** — N_sub ∈ {1,2,4,8} 对 union volume 的影响
- **体素碰撞加速** — 预光栅化场景 2272× 加速
- **Wavefront vs RRT** — 生长策略对比 (文字描述)

---

## 2. Figure 4: 三面板可扩展性图

### Panel (a): Planning Time vs DOF
- X 轴: DOF (2, 7)
- Y 轴: Planning time (ms), log scale
- 数据源: S5a

### Panel (b): Success Rate vs Obstacles
- X 轴: 障碍物数量 (1, 2, 4, 6, 8)
- Y 轴: Success rate (%)
- 数据源: S5b

### Panel (c): Success Rate vs Box Budget
- X 轴: max_boxes (50, 100, 200, 500, 1000, 2000)
- Y 轴: Success rate (%)
- 数据源: S5c

---

## 3. 子实验 S5a: DOF 缩放

### 实验设计

| 参数 | 值 |
|------|------|
| DOF | 2 (2dof_simple), 7 (panda_tabletop) |
| Config 2DOF | IFK + LinkIAABB_Grid |
| Config 7DOF | GCPC + LinkIAABB_Grid (若有 cache); 否则 IFK |
| n_trials | Quick/Lite: 3; **Full: 10** |
| timeout | 60 s |

### 预期结果
- 7DOF 比 2DOF 慢 10-50×
- n_boxes 增长 5-10×

### 当前数据

```json
{
  "2": {"time_mean": 0.033, "success_rate": 100.0, "n_trials": 10},
  "7": {"time_mean": 0.062, "success_rate": 100.0, "n_trials": 10}
}
```

**问题**: panda 仅 62ms / 1 box (grower 立刻退出, 因为 panda_tabletop 障碍在 5m 外)。
这不是有意义的 DOF 缩放测试 — 需要更有挑战性的 7DOF 场景。

---

## 4. 子实验 S5b: 障碍物数量缩放

### 实验设计

| 参数 | 值 |
|------|------|
| Robot | 7DOF Panda |
| n_obs | Quick: 1,2,4; Lite: 1,2,4,8; **Full: 1,2,4,6,8** |
| Obstacles 位置 | 远处: (±5, ±5, 0.5) 和 (±4, ±4, 0.5) |
| Config | IFK + LinkIAABB_Grid |
| n_trials | Quick/Lite: 3; **Full: 10** |
| timeout | 60 s |

### 当前障碍物定义

```python
extra_obs = [
    {"center": [5.0, 5.0, 0.5], "half_sizes": [0.5, 0.5, 0.5]},    # 5m 远
    {"center": [-5.0, 5.0, 0.5], "half_sizes": [0.5, 0.5, 0.5]},
    {"center": [5.0, -5.0, 0.5], "half_sizes": [0.5, 0.5, 0.5]},
    {"center": [-5.0, -5.0, 0.5], "half_sizes": [0.5, 0.5, 0.5]},
    {"center": [4.0, 4.0, 0.5], "half_sizes": [0.3, 0.3, 0.3]},
    {"center": [-4.0, 4.0, 0.5], "half_sizes": [0.3, 0.3, 0.3]},
    {"center": [4.0, -4.0, 0.5], "half_sizes": [0.3, 0.3, 0.3]},
    {"center": [-4.0, -4.0, 0.5], "half_sizes": [0.3, 0.3, 0.3]},
]
```

### 当前数据

```json
{
  "1": {"time_mean": 0.025, "success_rate": 100.0},
  "2": {"time_mean": 0.025, "success_rate": 100.0},
  "4": {"time_mean": 0.024, "success_rate": 100.0},
  "6": {"time_mean": 0.025, "success_rate": 100.0},
  "8": {"time_mean": 0.024, "success_rate": 100.0}
}
```

**问题**: 所有障碍物在 4-5m 远处, 对 Panda (~1m 臂展) 完全无影响。
time 和 success_rate 恒定, 无法展示可扩展性趋势。

**修复方案**: 使用 PHASE_S_EXPERIMENTS.md 中描述的近距离障碍:
```python
base_obs = [{"center": [0.5, 0.0, 0.4], "half_sizes": [0.3, 0.3, 0.02]}]
extra_obs = [
    {"center": [0.4, 0.2, 0.3], "half_sizes": [0.08, 0.08, 0.08]},
    {"center": [0.6, -0.2, 0.5], "half_sizes": [0.06, 0.06, 0.06]},
    {"center": [0.3, 0.3, 0.6], "half_sizes": [0.1, 0.05, 0.05]},
    # ... 在机器人工作空间内
]
```

---

## 5. 子实验 S5c: Box 预算缩放

### 实验设计

| 参数 | 值 |
|------|------|
| Robot | 7DOF Panda |
| Scene | panda_tabletop |
| max_boxes | Quick: 50,200,500; Lite: +2000; **Full: 50,100,200,500,1000,2000** |
| Config | IFK + LinkIAABB |
| n_trials | Quick/Lite: 3; **Full: 10** |
| timeout | 60 s |

### 当前数据

```json
{
  "50": {"time_mean": 0.024, "success_rate": 100.0},
  "100": {"time_mean": 0.024, "success_rate": 100.0},
  "200": {"time_mean": 0.024, "success_rate": 100.0},
  "500": {"time_mean": 0.024, "success_rate": 100.0},
  "1000": {"time_mean": 0.024, "success_rate": 100.0},
  "2000": {"time_mean": 0.026, "success_rate": 100.0}
}
```

**问题**: 与 S5b 类似, panda_tabletop 障碍在远处, grower 只生成 1 box。
max_boxes 上限不影响只需 1 box 的场景。

**修复方案**: 用需要多 box 的场景 (如 panda_shelf 或近距离障碍场景):
- 预期: max_boxes=50 → success ~60%; max_boxes=500 → success ~90%

---

## 6. Table 7: 连杆细分 (Link Subdivision)

### 实验设计

| 参数 | 值 |
|------|------|
| Robot | IIWA14 / Panda |
| Source | CritSample |
| N_sub | 1, 2, 4, 8 |
| Width | 0.10, 0.20, 0.40, 0.60, 1.00, 1.60 rad |
| 指标 | Union volume (m³) |

### 当前数据 (已完整)

| N_sub | w=0.10 | w=0.20 | w=0.40 | w=0.60 | w=1.00 | w=1.60 |
|-------|--------|--------|--------|--------|--------|--------|
| 1 | 0.1090 | 0.1482 | 0.2740 | 0.5042 | 1.4079 | 3.2455 |
| 2 | 0.0831 | 0.1217 | 0.2469 | 0.4772 | 1.3921 | 3.2455 |
| 4 | 0.0723 | 0.1108 | 0.2352 | 0.4667 | 1.3849 | 3.2455 |
| 8 | 0.0670 | 0.1049 | 0.2301 | 0.4618 | 1.3818 | 3.2455 |

**状态**: ✅ 完整, 由 `bench_link_envelope` 独立 bench 生成。

### 关键观察
- Volume 随 N_sub 单调递减
- 窄 intervals (w=0.10): N_sub 1→8 减少 37%
- 宽 intervals (w≥1.0): N_sub 影响忽略 (sweep 主导)

---

## 7. 论文中其他 Scalability 声称

论文 §Scalability 还提到以下数据, 需确认来源:

| 声称 | 数据 | 来源 |
|------|------|------|
| 体素碰撞加速 | 0.47ms → 3.3ns (2272×) | 独立 bench 或代码内测量 |
| SBF 构建 200 boxes (10-obstacle) | < 1s cold | 需实际测量 |
| 增量更新 | < 200ms | 需实际测量 |
| Wavefront vs RRT | 50-500× speed gap | 独立 bench (54 configs × 3 scenes) |

---

## 8. 当前状态与差距总结

| 项目 | 状态 | 差距 |
|------|------|------|
| S5a DOF | ✅ 有数据 | ⚠️ panda 场景太简单, 无意义的 DOF scaling |
| S5b Obstacles | ✅ 有数据 | ⚠️ 障碍物太远, 全部 100% success, 无 scaling 趋势 |
| S5c Budget | ✅ 有数据 | ⚠️ 同上, grower 只生成 1 box |
| Table 7 | ✅ 完整 | 无 |
| Figure 4 | ❌ 待生成 | 需有意义的 S5a/b/c 数据 |
| 体素碰撞数据 | ⚠️ 文字引用 | 需确认来源 |
| Wavefront vs RRT | ⚠️ 文字引用 | 需确认来源 |

---

## 9. 修复计划

### 最高优先级: S5b 障碍物修正

将远处障碍物 (5m) 替换为机器人工作空间内障碍 (0.3-0.7m):

```python
# 新的近距离障碍
extra_obs = [
    {"center": [0.5, 0.0, 0.4], "half_sizes": [0.3, 0.3, 0.02]},  # 桌面
    {"center": [0.4, 0.2, 0.3], "half_sizes": [0.08, 0.08, 0.08]},
    {"center": [0.6, -0.2, 0.5], "half_sizes": [0.06, 0.06, 0.06]},
    {"center": [0.3, 0.3, 0.6], "half_sizes": [0.1, 0.05, 0.05]},
    {"center": [0.7, -0.1, 0.3], "half_sizes": [0.05, 0.1, 0.1]},
    {"center": [0.2, -0.3, 0.4], "half_sizes": [0.07, 0.07, 0.07]},
    {"center": [0.55, 0.15, 0.55], "half_sizes": [0.04, 0.04, 0.04]},
    {"center": [0.45, -0.25, 0.35], "half_sizes": [0.09, 0.06, 0.08]},
]
```

### S5a: 用更具挑战性的场景
- 替换 panda_tabletop → panda_shelf 或自定义近距离障碍场景

### S5c: 用需要多 box 的场景
- 替换 panda_tabletop → 需要 100+ box 才能成功的场景

---

## 10. 执行命令

```powershell
# 删除旧结果
Remove-Item "results/paper/s5_scalability" -Recurse -Force

# Full 模式 (当前: ~4s; 修正后可能 ~10-30min)
python -c "
import sys; sys.path.insert(0, 'python')
from scripts.run_all_experiments import run_s5
run_s5(quick=False, lite=False)
"
```

---

## 11. 风险与注意事项

1. **当前数据无意义**: S5a/b/c 全部 100% success + 1 box, 因为障碍太远
2. **替换障碍后可能失败**: 近距离障碍需要 grower 实际工作, 可能暴露新 bug
3. **panda grower 刚修复**: boundary-empty early exit 可能对近距离障碍也过早退出
4. **fallback_limit = 12**: 对简单场景可能太小, 导致 grower 过早放弃
5. **Table 7 独立**: subdivision bench 数据与 S5 pipeline 分离, 不受影响
6. **论文数据一致性**: 修改障碍后需重跑实验并更新 Figure 4
