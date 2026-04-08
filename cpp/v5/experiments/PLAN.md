# 后续实验计划

> 生成日期: 2026-04-08
> 基于当前实验数据审计结果

---

## 一、当前数据完成度

| 实验 | 数据文件 | 内容 | 完整性 |
|------|----------|------|--------|
| S1 Envelope Tightness | s1_envelope_tightness/results.json | 4 sources × 3 envelopes × 2 robots = 24 rows | **✅ 完整** (IFK/CritSample/Analytical/GCPC) |
| S2 Envelope Timing | s2_envelope_timing/results.json | 24 rows，同上 | **✅ 完整** |
| S3 E2E Planning | s3_e2e/results.json | 1080 trials, 6 planners × 6 scenes × 30 seeds | **⚠️ 缺 Analytical/GCPC** (仅 IFK+CritSample) |
| S4 Baselines | s4_baselines/results.json | 120 trials, 1 planner × 4 scenes × 30 seeds | **❌ 严重不足** (仅 SBF-IFK-LinkIAABB) |
| S5 Scalability | s5_scalability/results.json | DOF/obstacles/budget 三组 sweep | **✅ 完整** |
| bench_link_envelope | — | Tab7 subdivision 数据源 | **✅ Tab7 已用 S1 数据生成** |

---

## 二、已发现问题

### P1: Tab4/Fig3 数据严重不足 (优先级: **高**)

**现象**: Tab4 仅 1 行 (SBF-IFK-LinkIAABB)，Fig3 Pareto 图仅 1 个点

**原因**: `run_s4()` 的 `sbf_planners` 列表只包含:
- `SBFPlannerAdapter("IFK", "LinkIAABB")`
- `SBFPlannerAdapter("Analytical", "LinkIAABB")`

经 `ep_sources=["IFK", "CritSample"]` 过滤后，只剩 IFK-LinkIAABB。CritSample **从未被加入** S4 的 planner 列表。

### P2: Tab3/Fig2 缺 Analytical 和 GCPC 行 (优先级: **中**)

**现象**: Tab3 只有 IFK (3行) + CritSample (3行) = 6 行，缺少 Analytical 和 GCPC 各 3 行

**原因**: S3 运行时 `ep_sources=["IFK", "CritSample"]`，跳过了 Analytical 和 GCPC

### P3: panda_shelf 全场景 0% 成功率 (优先级: **中**)

**现象**: S3 所有 6 个 planner 在 panda_shelf 上 30/30 全部失败，reason = "SBF plan failed"

**可能原因**:
- 场景定义: 单个薄板障碍物 (half_z=0.02=4cm)，机械臂需从下方穿过
- start/goal 配置跨度大 (q0: 0→0.8, q1: 0→0.5)，box 覆盖无法连通窄通道
- box budget 或 timeout 不足以在窄通道中建立连通图

### P4: S5 Obstacles ≥4 时 0% 成功率 (优先级: **低**)

**现象**: Fig4(b) 中 1-2 个障碍物 100% 成功，4+ 个障碍物 0% 成功

**可能原因**: 随障碍物增多，自由空间碎片化，box 预算不足以覆盖所有通道

### P5: Fig3 Pareto 图无意义 (优先级: **高**，依赖 P1)

**现象**: 只有 1 个点，没有对比意义

### P6: OMPL baselines 缺失 (优先级: **低**)

**现象**: Tab4/Fig3 缺少 RRTConnect/RRT*/BIT* 等采样规划器对比

**原因**: WSL 环境损坏，OMPL 不可用

---

## 三、实验计划

### Phase 1: 修复 S4 代码并重跑 (预计 ~15 min)

**目标**: 让 Tab4/Fig3 有多 planner 对比

**步骤**:

1. **修改 `run_s4()` 函数** (`python/scripts/run_all_experiments.py:324`):
   在 `sbf_planners` 列表中增加 CritSample 配置:
   ```python
   sbf_planners = [
       SBFPlannerAdapter("IFK", "LinkIAABB"),
       SBFPlannerAdapter("CritSample", "LinkIAABB"),       # 新增
       SBFPlannerAdapter("CritSample", "Hull16_Grid"),      # 新增 (推荐配置)
       SBFPlannerAdapter("Analytical", "LinkIAABB"),
   ]
   ```

2. **删除旧 S4 结果**: `del experiments/results/s4_baselines/results.json`

3. **重跑 S4**:
   ```powershell
   python experiments/scripts/run_s4_only.py
   # 或在 run_all 中单独调 run_s4(skip_ompl=True, ep_sources=["IFK","CritSample"])
   ```

4. **估算时间**: 4 scenes × 3 planners × 30 seeds = 360 trials × ~2s ≈ 12 min

### Phase 2: 补跑 S3 的 Analytical/GCPC (预计 ~2 hr)

**目标**: Tab3/Fig2 显示完整 12 配置

**决策点**: Analytical/GCPC 每个 node 需 10-15ms，7DOF 场景 build 较慢

**步骤**:

1. **修改 S3 运行脚本**: 设 `ep_sources=["Analytical", "GCPC"]` 单独跑补充数据

2. **合并数据**: 将新 trials 追加到现有 s3_e2e/results.json

3. **估算时间**:
   - Analytical+GCPC × 3 envelopes = 6 新配置
   - 6 configs × 6 scenes × 30 seeds = 1080 trials
   - 7DOF 场景每 trial 可能需要 60-120s (build 慢)
   - 估计 1.5-2 小时

**备选**: 如果只跑 Analytical (不需要 GCPC cache)，可减半为 ~1 hr

**可跳过条件**: 如果论文只聚焦 IFK vs CritSample 对比，可在正文解释 "Analytical/GCPC results confirm CritSample accuracy (see Tab1/Tab2)" 并引用 S1/S2 数据

### Phase 3: 诊断并修复 panda_shelf (预计 30-60 min)

**目标**: 提高 panda_shelf 成功率或合理解释

**方案 A — 增加 box budget**:
```python
# 当前默认可能是 500 boxes
# 尝试 2000-5000 boxes，timeout 120s
```

**方案 B — 调整场景参数**:
- 检查 start/goal 是否确实在 C_free 中
- 降低障碍物厚度或调整位置使通道更宽

**方案 C — 在论文中合理解释**:
- panda_shelf 是一个极窄通道场景
- SBF 的 box 覆盖在极窄通道中连通性受限
- 这是已知局限性，可在 Discussion 中讨论

**建议**: 先单独测试 panda_shelf 的 start/goal 合法性，再决定方案

### Phase 4: 重新生成论文 (预计 ~5 min)

**步骤**:
```powershell
cd safeboxforest/v5
$env:PYTHONPATH = "build_x64/Release;python"
python experiments/scripts/gen_paper.py
```

自动执行: gen_tables → gen_figures → pdflatex×3 + bibtex

---

## 四、优先级排序

| 优先级 | 任务 | 预计时间 | 影响 |
|--------|------|----------|------|
| **🔴 P0** | Phase 1: 修复 S4 + 重跑 | 15 min | Tab4 从 1 行 → 3-4 行，Fig3 有多点 Pareto |
| **🟡 P1** | Phase 3C: panda_shelf 在论文中解释 | 10 min | 消除 reviewers 对 0% 成功率的疑问 |
| **🟡 P2** | Phase 4: 重新生成论文 | 5 min | 更新所有表格和图 |
| **🟢 P3** | Phase 2: 补跑 Analytical/GCPC (S3) | 1-2 hr | Tab3 从 6 行 → 12 行 (完整) |
| **🟢 P4** | Phase 3A: 调试 panda_shelf budget | 30-60 min | 可能改善成功率 |
| **⚪ P5** | OMPL baselines (需修 WSL) | 数小时 | 增加采样规划器对比 |

---

## 五、最小可发表方案 (Minimal Viable Paper)

如果时间紧迫，仅需完成:

1. **Phase 1** (15 min): 修复 S4，增加 CritSample planner
2. **Phase 3C** (10 min): 在 root.tex Discussion 中添加 panda_shelf 局限性说明
3. **Phase 4** (5 min): 重新生成论文

总需时间: **~30 min**

论文将有:
- Tab1/Tab2: 完整 4×3 = 12 配置 ✅
- Tab3/Fig2: IFK + CritSample (6 配置)，加脚注引用 Tab1/Tab2 说明 Analytical/GCPC 一致性
- Tab4/Fig3: IFK + CritSample + (可选 Analytical) 的 baseline 对比
- Tab5/Tab6: 完整 ✅
- Fig4: 完整 ✅

---

## 六、需要决定的事项

1. **S3 是否补跑 Analytical/GCPC?** — 增加 1-2 hr，Tab3 完整但 7DOF build 很慢
2. **panda_shelf 如何处理?** — 增加 budget 重跑 vs 在论文中解释局限性
3. **OMPL baselines 是否需要?** — 需要修复 WSL 或在其他机器上跑
4. **S5 obstacles≥4 失败是否需要处理?** — 可能是 budget 不足，可尝试增大
