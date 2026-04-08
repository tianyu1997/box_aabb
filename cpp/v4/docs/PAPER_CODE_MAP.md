# Paper ↔ Code Traceability Map

> 论文中每个图表/实验必须有对应的代码和结果目录溯源条目。

---

## 论文表格

### Table 1 — 完整流水线基准测试（4×3 = 12 配置）

```
Table 1 → experiments/exp_v3v4_epiAABB.cpp → (stdout CSV)
```

- **源代码**: `experiments/exp_v3v4_epiAABB.cpp`
- **构建目标**: `exp_v3v4_epiAABB`
- **运行方式**: `exp_v3v4_epiAABB.exe [n_trials] [seed]`
- **输出**: stdout CSV（source, envelope, n_sub, volume, cold_ms, warm_ms, voxels）
- **论文对应**: root.tex `\label{tab:pipeline}` / root_cn.tex `\label{tab:pipeline}`
- **使用的默认配置**:
  - `AnalyticalCriticalConfig::all_enabled()` (dual_phase3=true)
  - GCPC cache: `build_gcpc_cache()` + `enrich_with_interior_search(500, 5)`
  - 4 sources × 3 envelopes, 50 random boxes, IIWA14 7-DOF

---

## 论文章节 ↔ 代码模块

| 论文章节 | 代码模块 | 关键头文件 |
|----------|---------|-----------|
| §III.1 Interval FK | `robot/interval_fk` | `include/sbf/robot/interval_fk.h` |
| §III.1 Active-link Filtering | `robot/iaabb_config` | `include/sbf/robot/iaabb_config.h` |
| §III.2 Subdivision | `envelope/envelope_derive` | `include/sbf/envelope/envelope_derive.h` |
| §III.3 Endpoint-iAABB | `envelope/endpoint_source` | `include/sbf/envelope/endpoint_source.h` |
| §III.4 Analytical Solver | `envelope/analytical_solve` | `include/sbf/envelope/analytical_solve.h` |
| §III.4 DH Coefficients | `envelope/analytical_coeff` | `include/sbf/envelope/analytical_coeff.h` |
| §III.5 CritSample | `envelope/crit_sample` | `include/sbf/envelope/crit_sample.h` |
| §III.6 GCPC | `envelope/gcpc` | `include/sbf/envelope/gcpc.h` |
| §III.7 Adaptive Freeze | `core/interval_trig`, `core/config` | `include/sbf/core/interval_trig.h`, `include/sbf/core/config.h` |
| §IV.1 Voxel BitBrick | `voxel/bit_brick` | `include/sbf/voxel/bit_brick.h` |
| §IV.2 Hull16 Rasterisation | `voxel/hull_rasteriser` | `include/sbf/voxel/hull_rasteriser.h` |
| §V.1 LECT | `forest/lect` | `include/sbf/forest/lect.h` |
| §V.2 Endpoint Dispatcher | `envelope/endpoint_source` | `include/sbf/envelope/endpoint_source.h` |
| §V.3 Pipeline Config | `envelope/pipeline` | `include/sbf/envelope/pipeline.h` |
| §VI Experiments | `experiments/` | (各实验文件) |

---

## 实验溯源

### Exp: 冻结深度扫描 (iFK-only)

```
exp_freeze_sweep → experiments/exp_freeze_sweep.cpp → (stdout CSV)
```

- 仅 iFK 源，width × freeze_depth → per-endpoint 体积膨胀比
- 宽度: {0.05, 0.1, 0.2, 0.3, 0.5, 0.7, 1.0, 1.5, 2.0, 3.0, 2π}
- 冻结深度: {0, 1, 2, 3, 4, 5, 6, 7}

### Exp: 冻结逐连杆分析 (iFK-only)

```
exp_freeze_per_link → experiments/exp_freeze_per_link.cpp → (stdout CSV)
```

- 仅 iFK 源，per-link iAABB 体积在固定宽度 {0.3, 0.5, 1.0, 1.5} 下的分析

### Exp: 非对称冻结

```
exp_freeze_asymmetric → experiments/exp_freeze_asymmetric.cpp → (stdout CSV)
```

### Exp: 冻结 × 源对比（4 源 × 自适应冻结）

```
exp_freeze_sources → experiments/exp_freeze_sources.cpp → exp_freeze_sources_output.csv
```

- **4 种源**: iFK, CritSample, Analytical, GCPC
- **对比模式**: 直接计算 (fd=0) vs 冻结+重建 (自适应 fd)
- **度量**: per-link body AABB 体积、时间 (ms)、体积膨胀比 (frozen/direct)
- **宽度**: {0.05, 0.10, 0.15, 0.20, 0.30, 0.50, 0.70, 1.00}
- **试次**: 50 随机区间，IIWA14 7-DOF
- **关键结论**:
  - iFK 冻结几乎无体积损失 (ratio ≈ 0.8–1.1)
  - CritSample/Analytical/GCPC 冻结引入 1.4–3.0× 体积膨胀
  - 窄区间时 Analytical 加速 6–14×, GCPC 加速 30–200×

### Exp: 4×3 全管线基准 (epiAABB)

```
exp_v3v4_epiAABB → experiments/exp_v3v4_epiAABB.cpp → (stdout CSV)
```

- Table 1 数据来源
- 4 源 × 3 包络, 50 随机盒子, IIWA14

### Exp: Analytical A/B 基准

```
exp_analytical_ab_benchmark → experiments/exp_analytical_ab_benchmark.cpp → (stdout CSV)
```

### Exp: 阶段对比

```
exp_phase_compare → experiments/exp_phase_compare.cpp → (stdout CSV)
```

### Exp: Link iAABB 对比 (v3 vs v4)

```
exp_v3v4_linkIAABB → experiments/exp_v3v4_linkIAABB.cpp → (stdout CSV)
```

### Exp: Grower 策略基准（Wavefront BFS vs RRT 参数扫描）

```
bench_grower_strategies → v4/experiments/bench_grower_strategies.py → v4/output/bench_grower_<ts>/
```

- **54 配置** × 3 场景种子 = 162 次运行（2DOF）
- Wavefront: `ffb_min_edge` × `guided_sample_ratio` × `n_edge_samples` = 27 配置
- RRT: `ffb_min_edge` × `rrt_goal_bias` × `rrt_step_ratio` = 27 配置
- **度量**: MC 覆盖率（30K samples）、box 数量、连通分量数/最大分量占比、生长耗时 (ms)、综合得分
- **输出**: `results.csv`, `summary.csv`, `charts/*.png` (8 张), `viz/*.png` (top-3×2)
- **论文对应**: root.tex §V.4 `\label{sec:forest_growth}` 策略对比段落 / root_cn.tex 同名章节
- **关键结论**: Wavefront BFS 与 RRT 覆盖率相当（~0.85@e=0.05），Wavefront 快 50–500×；`ffb_min_edge` 是最关键参数
