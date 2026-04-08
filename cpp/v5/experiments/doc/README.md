# 实验总览与状态仪表板

> 更新日期: 2026-04-06
> 论文: SafeBoxForest v5 — T-RO 投稿
> 执行环境: Windows, MSVC 2022, Release /O2, Intel Core i7, Python 3.13

---

## 实验与论文章节映射

| # | 实验 | 论文章节 | 表格 | 图形 | 代码阶段 | 状态 |
|---|------|----------|------|------|----------|------|
| 1 | [Envelope Volume](EXP1_ENVELOPE_VOLUME.md) | §exp_pipeline | **Table 1** | — | S1 (run_s1) | ⚠️ Quick 模式, 需 Full |
| 2 | [Envelope Timing](EXP2_ENVELOPE_TIMING.md) | §exp_pipeline | **Table 2** | — | S2 (run_s2) | ⚠️ Quick 模式, 需 Full |
| 3 | [E2E Planning](EXP3_E2E_PLANNING.md) | §exp_e2e | **Table 3** | **Figure 2** | S3 (run_s3) | ⚠️ Quick 模式, 需 Full |
| 4 | [Baselines](EXP4_BASELINES.md) | §exp_baselines | **Table 4** | **Figure 3** | S4 (run_s4) | ❌ STUB, 需 OMPL |
| 5 | [Freeze Comparison](EXP5_FREEZE_COMPARISON.md) | §exp_freeze | **Table 5** | — | **无** (*) | ❌ STUB, 需实现 |
| 6 | [LECT Cache](EXP6_LECT_CACHE.md) | §exp_lect | **Table 6** | — | 独立 bench | ✅ 完整 |
| 7 | [Scalability](EXP7_SCALABILITY.md) | §exp_scalability | **Table 7** | **Figure 4** | S5 (run_s5) | ⚠️ 障碍物需修正 |

(*) EXP5 需新增代码阶段或独立脚本

---

## 全局参数

| 参数 | Quick | Lite | Full |
|------|-------|------|------|
| S1 n_boxes | 50 | 200 | **500** |
| S2 n_boxes × repeats | 100×10 | 300×20 | **1000×50** |
| S3 scenes × configs × seeds | 2×3×3=18 | 6×12×3=216 | **6×12×30=2160** |
| S4 scenes × planners × seeds | 2×4×3=24 | 4×6×3=72 | **4×6×30=720** |
| S5 trials/sub-experiment | 3 | 3 | **10** |
| **Total est. time** | ~3 min | ~65 min | **~3.5 h** |

---

## 执行优先级

### 第一优先级: 现有代码可直接重跑 (删旧 JSON + Full 模式)

| 顺序 | 实验 | 操作 | 预计时间 |
|------|------|------|----------|
| 1 | EXP1 (S1) | 删 results.json → run_s1() | ~10 min |
| 2 | EXP2 (S2) | 删 results.json → run_s2() | ~30-45 min |
| 3 | EXP3 (S3) | 删 results*.json → run_s3() | ~60-100 min |

### 第二优先级: 需代码修改

| 顺序 | 实验 | 需修改 | 预计工作量 |
|------|------|--------|------------|
| 4 | EXP7 (S5) | 修正 S5b 障碍物位置 → 近距离障碍 | ~30 min code + rerun |
| 5 | EXP5 (Freeze) | 新增 run_s6_freeze() + C++ binding check | ~2-4 h |

### 第三优先级: 外部依赖

| 顺序 | 实验 | 依赖 | 风险 |
|------|------|------|------|
| 6 | EXP4 (S4) | OMPL 安装 (WSL/conda) | 高: Windows 不原生支持 |
| — | EXP4 (S4) | Drake + IRIS-NP | 极高: 未有任何集成 |

### 无需操作

| 实验 | 原因 |
|------|------|
| EXP6 (LECT) | Table 6 已完整 |
| EXP7/Table 7 | tab7_subdivision.tex 已完整 |

---

## 执行一键命令

### 重跑 S1+S2+S3 Full 模式

```powershell
cd safeboxforest/v5/
$env:PYTHONPATH = "build_x64/Release;python"

# 删旧结果
Remove-Item "results/paper/s1_envelope_tightness/results.json" -Force
Remove-Item "results/paper/s2_envelope_timing/results.json" -Force
Remove-Item "results/paper/s3_e2e/results.json" -Force
Remove-Item "results/paper/s3_e2e/results_*.json" -Force
Remove-Item "results/paper/s5_scalability" -Recurse -Force

# 执行全部实验
python -u python/scripts/run_all_experiments.py --skip-figures 2>&1 | Tee-Object -FilePath full_run.log
```

### 仅重跑 S1 和 S2

```powershell
Remove-Item "results/paper/s1_envelope_tightness/results.json" -Force
Remove-Item "results/paper/s2_envelope_timing/results.json" -Force
python -c "
import sys, os; sys.path.insert(0, 'python'); os.chdir('.')
from scripts.run_all_experiments import run_s1, run_s2
run_s1(quick=False, lite=False)
run_s2(quick=False, lite=False)
"
```

---

## 数据完整性检查表

| 数据文件 | 预期行数 | 当前行数 | Full 目标 |
|----------|----------|----------|-----------|
| s1/results.json | 24 rows | 8 | **24** |
| s2/results.json | 24 rows | 8 | **24** |
| s3/results.json | 2160 trials | 18 | **2160** |
| s4/results.json | 720 trials | 24 | **720** |
| s5/results.json | 3 sub-dicts | 3 | **3** (修正数据) |
| tab5 | 完整表格 | STUB | **需实现** |
| tab6 | 完整表格 | ✅ | ✅ |
| tab7 | 完整表格 | ✅ | ✅ |

---

## C++ 调试输出清理

当前 C++ 代码中仍有 `fprintf(stderr, ...)` 调试输出:

| 文件 | 标签 | 建议 |
|------|------|------|
| `grower.cpp` | `[GRW] roots:`, `[GRW] timing:`, `[GRW] final:` | Full 实验前移除或 #ifdef |
| `sbf_planner.cpp` | `[PLN] lect=`, `[PLN] sg_connect:`, `[PLN] raw:`, 等 | Full 实验前移除 |

这些输出会污染 stderr 并影响实验性能 (fprintf I/O 开销)。建议:
1. Full 实验前注释掉所有 fprintf
2. 或用 `#ifdef SBF_DEBUG` 包裹
3. 或添加 verbosity level 控制
