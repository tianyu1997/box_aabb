# SafeBoxForest v5 — 进阶迁移路线图

> 创建: 2026-04-04
> 更新: 2026-04-04 — Phase I-Q 完成, 13/13 C++ 测试通过, 41/41 Python 测试通过
> 前置: Phase A-H (核心) 已完成, 9/9 测试通过

---

## 总览

| Phase | 名称 | 类型 | 预计 LOC | 依赖 | 优先级 |
|-------|------|------|---------|------|--------|
| **I** | VizExporter (C++ JSON 导出) | C++ | 368+55 | A, B-C, F | ✅ 已完成 |
| **J** | Python Bindings (pybind11) | C++/Python | 369+42 | A-H | ✅ 已完成 |
| **K** | Python 可视化 (sbf5_viz) | Python | ~900 | I 或 J | ✅ 已完成 |
| **L** | Drake GCS 完整实现 | C++ | 215+47 | H | ✅ 已完成 |
| **M** | Baselines + Metrics + Experiments | Python | ~1100 | J | ✅ 已完成 |
| **N** | Voxel Grid + Hull-16 | C++ | ~1000 | A, B-C | ✅ 已完成 |
| **O** | 端到端集成测试 + Python 验证 | C++/Python | ~300 | A-N | ✅ 已完成 |
| **P** | Envelope 对比导出 + Hull16 Pipeline | C++/Python | ~350 | I, N | ✅ 已完成 |
| **Q** | Benchmark 运行 + 论文就绪 | Python | ~400 | O, P | ✅ 已完成 |
| **R** | 实验基础设施扩展 (Pipeline Config) | C++/Python | ~280 | A-Q | 🔲 未开始 |
| **S** | 5 组核心实验 | Python | ~400 | R | 🔲 未开始 |
| **T** | 论文输出生成 (LaTeX + Figures) | Python | ~390 | S | 🔲 未开始 |

**合计: ~6670 LOC**

---

## 依赖图

```
Phase A-H (core, 已完成)
    │
    ├──▶ Phase I (VizExporter) ──▶ Phase K (Python Viz, 离线模式)
    │
    ├──▶ Phase J (pybind11) ──┬──▶ Phase K (Python Viz, 在线模式)
    │                         │
    │                         └──▶ Phase M (Baselines + Experiments)
    │
    ├──▶ Phase L (Drake GCS) ←── 可与 I/J/K 并行
    │
    └──▶ Phase N (Voxel) ←── 可与 I/J/K/L/M 并行

Phase I-N (进阶, 已完成)
    │
    ├──▶ Phase O (端到端测试 + Python 验证)
    │         ├── 填充 test_full_pipeline.cpp
    │         └── 构建 _sbf5_cpp + pytest
    │
    ├──▶ Phase P (功能补全) ←── 可与 O 并行
    │         ├── VizExporter 对比/体素导出
    │         └── Hull16_Grid → FFB pipeline
    │
    └──▶ Phase Q (Benchmark + 论文就绪) ←── 依赖 O + P
              ├── 新增场景 + 运行 benchmark
              ├── LaTeX 表格 + PDF 配图
              └── C++ → JSON → Plotly → HTML 全链路

Phase O-Q (功能补全 + benchmark 框架, 已完成)
    │
    └──▶ Phase R (Pipeline Config 穿透)
              ├── SBFPlannerConfig += EndpointSource + EnvelopeType
              ├── pybind11 绑定新枚举/config
              ├── Runner 配置矩阵 (12 combos)
              └── GCPC cache 生成工具
              │
              └──▶ Phase S (5 组核心实验)
                        ├── S1: Envelope 紧密度
                        ├── S2: Envelope 计算耗时
                        ├── S3: 端到端 12×6×30 = 2160 trials
                        ├── S4: SBF vs OMPL baselines
                        └── S5: DOF/obstacle/budget scaling
                        │
                        └──▶ Phase T (论文输出)
                                  ├── 4 LaTeX booktabs 表格
                                  ├── 4 PDF/PNG 配图
                                  ├── 统计显著性 (Wilcoxon)
                                  └── 一键复现脚本
```

---

## 推荐实施顺序

### 第一批: 调试与可视化能力 (I → J → K)

1. **Phase I**: C++ JSON 导出 → 无需 Python 即可 dump 调试数据
2. **Phase J**: pybind11 绑定 → Python 环境可访问 C++ planner
3. **Phase K**: 可视化包 → 交互式 3D 查看 + 论文配图

### 第二批: 算法完整性 (L, 可并行)

4. **Phase L**: 填充 GCS Drake 实现 → 从 fallback 升级为真正的 GCS

### 第三批: 实验框架 (M)

5. **Phase M**: 基线 + 指标 + 实验编排 → 论文实验数据

### 按需: 精度提升 (N)

6. **Phase N**: Voxel/Hull-16 体素化 → 更紧密 envelope, 论文精度分析

---

## 各阶段计划文档

| Phase | 文档 |
|-------|------|
| I | `doc/plan/PHASE_I_VIZ_EXPORTER.md` |
| J | `doc/plan/PHASE_J_PYTHON_BINDINGS.md` |
| K | `doc/plan/PHASE_K_PYTHON_VIZ.md` |
| L | `doc/plan/PHASE_L_DRAKE_GCS.md` |
| M | `doc/plan/PHASE_M_BASELINES_EXPERIMENTS.md` |
| N | `doc/plan/PHASE_N_VOXEL_HULL16.md` |
| O | `doc/plan/PHASE_O_E2E_PYTHON_VERIFY.md` |
| P | `doc/plan/PHASE_P_FEATURE_COMPLETION.md` |
| Q | `doc/plan/PHASE_Q_BENCHMARK_PAPER.md` |
| R | `doc/plan/PHASE_R_EXPERIMENT_INFRA.md` |
| S | `doc/plan/PHASE_S_EXPERIMENTS.md` |
| T | `doc/plan/PHASE_T_PAPER_OUTPUT.md` |

---

## v1-v4 迁移覆盖度

| 源功能 | 归属 Phase | 状态 |
|--------|-----------|------|
| v4 `viz_exporter.cpp` (800 LOC) | Phase I | ✅ |
| v4 `sbf4_bindings.cpp` (800 LOC) | Phase J | ✅ |
| v4 `sbf4_viz/` (8 模块) | Phase K | ✅ |
| v1 `drake_gcs.cpp` + v2-v3 GCS Python | Phase L | ✅ |
| v3 `baselines/` (800 LOC) | Phase M | ✅ |
| v3 `metrics.py` (400 LOC) | Phase M | ✅ |
| v3 `experiments/` (10 scripts) | Phase M | ✅ |
| v4 `voxel_grid.h` + `bit_brick.h` (1200 LOC) | Phase N | ✅ |
| v4 `hull_rasteriser.h` (60 LOC) | Phase N | ✅ |
| v1-v3 Python forest/collision (已被 C++ 替代) | — | ✅ 不再需要 |
| v4 Scene class (~50 LOC) | — | ✅ 已有 CollisionChecker |
| v4 URDF stubs | — | ⏳ 延后 (按需) |
