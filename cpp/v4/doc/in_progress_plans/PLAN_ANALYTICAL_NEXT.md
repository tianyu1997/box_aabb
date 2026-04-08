# v4 Analytical 方法：整体现状分析与后续改动计划

> 生成日期：2026-03-20
> 基线：v4 当前 172/172 测试通过，`analytical_solve.cpp` 3,077 行，10 项测试覆盖

---

## 一、当前实现现状总览

### 1.1 核心求解管线（6 阶段）— 全部已实现

| 阶段 | 功能 | 状态 | 代码位置 |
|------|------|------|----------|
| Phase 0 | kπ/2 顶点枚举 + baseline | **已完成** | `analytical_solve.cpp` L1750–1800 |
| Phase 1 | 1D edge atan2 闭式求解 | **已完成** | `solve_edges()` L299–470 |
| Phase 2 | 2D face degree-8 多项式 | **已完成** | `solve_faces()` L470–820 |
| Phase 2.5a | Pair-constrained 1D | **已完成** | `solve_pair_constrained_1d()` L979–1180 |
| Phase 2.5b | Pair-constrained 2D | **已完成** | `solve_pair_constrained_2d()` L1180–1463 |
| Phase 3+ | Multi-start interior descent | **已完成** | `solve_interior[_improved]()` L822–1700 |

### 1.2 v4 优化能力

| 能力 | 状态 | 默认 | 说明 |
|------|------|------|------|
| AA gap 剪枝（link 级 + 阶段间刷新） | **已完成** | ON | `refresh_skip_for_phase()` 每阶段前增量刷新 |
| P1/P2/P2.5/P3 候选去重（adaptive） | **已完成** | ON | `DedupAdaptiveRuntime` warmup + hit-rate 门控 |
| Parallel-by-link（`std::thread`） | **已完成** | OFF | 全阶段覆盖，自动检测核数 |
| Multi-qi 候选生成（P2） | **已完成** | OFF | 3 条件门控，O5.4d-O5.4h 结论 `NO-GO(default-off)` |
| Dual Phase 3 checkpoint | **已完成** | OFF | 从 (P0+P1) 与完整 checkpoint 分别跑 P3 |

### 1.3 O5 解析微分系列

| 组件 | 状态 | 说明 |
|------|------|------|
| P2 prototype 比较（compare-only） | **已完成** | QR-fitted vs symbolic 系数对照，3-way gate |
| Gate 阈值扫描与推荐 | **已完成** | O5.3c：推荐阈值 `fit=3e-17, sym_rms=1e-13, sym_max=1e-11` |
| Shadow run 覆盖度统计 | **已完成** | O5.4b：`avg_shadow_coverage_pct=100%`，`candidate_per_eval=0.0075` |
| All-branch shadow 统计 | **已完成** | O5.4c：`uplift_pct=21.05%` |
| Hybrid 配置与统计骨架 | **骨架** | 头文件字段已声明，benchmark 可读写，**但 `.cpp` 中无任何引用** |
| Hybrid 真实写回路径 | **未实现** | `solve_faces()` 始终使用 QR-fitted 系数，从未使用 symbolic 系数做 root-finding |

### 1.4 Endpoint Source 集成

| Source | 状态 | 说明 |
|--------|------|------|
| iFK | **已完成** | 完整实现 + 增量支持 |
| Analytical | **已完成** | 完整集成到 `compute_endpoint_iaabb()`，有 fast-path `memcpy` |
| CritSample | **Stub** | 头文件声明完整，`.cpp` 为空，dispatch 回退到 iFK |
| GCPC | **Stub** | 头文件最小，`.cpp` 占位，dispatch 回退到 iFK |

### 1.5 缓存集成

| 组件 | 状态 | 说明 |
|------|------|------|
| EnvelopeCache（3 层：tree/endpoint/hull） | **已完成** | HCACHE02 + FRM4 + HUL1 |
| Analytical 结果缓存 | **未实现** | 每次调用从头计算，无中间结果缓存 |
| 增量 Analytical | **未实现** | `compute_endpoint_iaabb_incremental` 对非 iFK 全部回退 |

### 1.6 测试覆盖

| 测试文件 | 覆盖范围 | 缺失 |
|----------|----------|------|
| `test_analytical.cpp`（10 项） | 基本求解、vs iFK、AA 剪枝、配置变体、端到端 dispatch | 无 hybrid 测试、无 parallel 测试、无 multi-qi 测试 |
| CritSample 测试 | **无** | — |
| GCPC 测试 | **无** | — |
| EnvelopeCache 测试 | **无** | — |

---

## 二、后续改动项（按优先级排序）

### P1（高优先）：Hybrid 写回真实实现

**现状**
- O5.5 骨架已经完成（config 字段 + stats 字段 + benchmark 环境变量 + CSV 回显列）。
- O5.6/O5.7 重复验证确认骨架本身无副作用。
- **但 `analytical_solve.cpp` 中从未引用过 `enable_p2_derivative_hybrid`**，hybrid 统计永远为 0。

**需要做的改动**
1. 在 `P2ProtoMetrics` 结构体中新增 hybrid 计数器字段。
2. 在 `merge_p2_proto_metrics()` 中合并 hybrid 计数器。
3. 在 `solve_faces()` 的系数提取循环（L678–690 区域）中插入 hybrid 路径选择逻辑：
   - 当 `enable_p2_derivative_hybrid && symbolic_ready && gate_accept_axis[d]` 时：
     - 计算 symbolic 系数（复用已有的 `compute_p2_symbolic_coeff_from_grid()`）。
     - 若 `enable_p2_hybrid_fallback`，比较 symbolic 与 QR 系数差异；若超过 `p2_hybrid_max_abs_error` 则回退。
     - 否则用 symbolic 系数替代 QR 系数。
   - 累加 `n_p2_hybrid_eval/applied/fallback_cases`。
4. 在 `derive_aabb_critical_analytical()` 末尾将 `P2ProtoMetrics` 的 hybrid 字段写回 `AnalyticalCriticalStats`。
5. 新增单元测试：验证 hybrid 开启时结果与 baseline 一致（`1e-6` 内），验证 fallback 触发时正确回退。

**预计工作量**：~50 行 C++ 改动 + ~40 行测试。

**验收标准**
- 默认 OFF 时行为与当前完全一致。
- 开启后，benchmark 中 hybrid 统计不再全 0。
- AABB 结果与 baseline 一致（`1e-6` 内）。

---

### P2（高优先）：Hybrid 路径 N=10 A/B 验证

**前置**：P1 完成。

**需要做的改动**
1. 用 O5.7 的参数与高分支 profile 跑 N=10 baseline/trial：
   - baseline：`SBF_P2_DERIVATIVE_HYBRID=0`
   - trial：`SBF_P2_DERIVATIVE_HYBRID=1`（`SBF_P2_HYBRID_FALLBACK=1`）
2. 汇总 mean/std：`mean_ms_delta_pct`、`mean_fk_p2_delta_pct`、`mean_fk_total_delta_pct`、`hybrid_eval/applied/fallback_cases`。
3. 产出 `o58_runs/` 下的 compare/summary/recommendation。

**验收标准**
- hybrid 统计非零且稳定。
- 给出最终 `GO_limited_rollout` 或 `NO_GO_default_off`。

---

### P3（中优先）：Segment/Phase 级动态 AA 剪枝

**现状**
- 当前 AA 剪枝在**阶段间**刷新 skip_link，粒度为 link 级。
- 阶段**内**的 segment/axis 级短路仅在 P2 有 `aa_axis.axis_can_improve[d]`，其他阶段缺失。

**需要做的改动**
1. 在 `solve_edges()`（P1）中增加 per-segment axis-level 短路：若当前 segment 的某 axis 已被 AA 证明不可改进，跳过该 axis 的候选评估。
2. 在 `solve_pair_constrained_1d/2d()`（P2.5）中接入同等的 axis-level 剪枝。
3. 在 `solve_interior[_improved]()`（P3）中增加同等判定。
4. 新增统计字段：
   - `n_aa_pruned_axis_checks_p1/p2/p25/p3`
5. 接入 benchmark CSV。

**预计工作量**：~100 行 C++ 改动 + CSV 扩展。

**验收标准**
- 不放宽包络安全性。
- 在至少 1 个场景下单阶段 FK 调用下降 `>10%`。

---

### P4（中优先）：`pair_2d` Root 批量评估

**现状**
- `solve_pair_constrained_2d()` 逐 root 调用 `compute_from`，局部 FK 前缀重建频繁。

**需要做的改动**
1. 将 roots 按 `from_joint` 排序分桶。
2. 同 `from_joint` 的 roots 共享 FK 前缀，batched 评估。
3. 新增 `pair2d_batch_size` 配置参数（默认 0=全批）。

**预计工作量**：~120 行 C++ 改动。

**验收标准**
- `compute_from` 调用次数下降 `>=15%`。
- 多次重复运行的耗时标准差下降。

---

### P5（中优先）：增量 Analytical 支持

**现状**
- `compute_endpoint_iaabb_incremental` 对 `EndpointSource::Analytical` 直接回退到 iFK。
- TODO 注释位于 `endpoint_source.cpp` L151。

**需要做的改动**
1. 设计增量策略：当仅少数关节的 interval 发生变化时，可复用上次求解中未受影响的 link 极值，仅对受影响 link 重新求解。
2. 在 `EndpointIAABBResult` 中增加"上次 analytical 状态快照"字段（per-link extremes + checksum）。
3. 实现 `compute_endpoint_iaabb_incremental` 的 Analytical 路径。

**预计工作量**：~200 行 C++ 改动 + 测试。

**验收标准**
- 增量结果与全量结果一致（`1e-6` 内）。
- 在"仅 1-2 关节变化"场景下耗时显著下降。

---

### P6（中优先）：Analytical 结果缓存（per-node）

**现状**
- `EnvelopeCache` 已有 3 层缓存架构（tree/endpoint/hull），但不缓存 analytical 中间结果。
- 对于 SBF forest 中大量 node 共享相似 interval 的场景，存在大量重复计算。

**需要做的改动**
1. 设计缓存键：`(robot_hash, intervals_hash, analytical_config_hash)` → per-link AABB 结果。
2. 在 `derive_aabb_critical_analytical()` 入口查询缓存，命中则直接返回。
3. 计算完成后写缓存。
4. 缓存淘汰策略（LRU / 容量上限）。

**预计工作量**：~150 行 C++ 改动 + 测试。

**验收标准**
- 缓存命中时耗时 <1% 全量计算。
- Forest 级重复场景端到端收益可观。

---

### P7（低优先）：CritSample 端源迁移

**现状**
- 头文件 `crit_sample.h` 已声明完整 `CriticalSamplingConfig`（含 coupled enumeration、manifold sampling、L-BFGS-B 等字段）。
- `crit_sample.cpp` 为空 stub。
- `endpoint_source.cpp` dispatch 回退到 iFK。

**需要做的改动**
1. 从 v3 `envelope_derive_critical.cpp` 拆分 CritSample 相关实现（~1,200 行），适配 v4 API。
2. 在 `endpoint_source.cpp` 接入真实 dispatch。
3. 新增 `test_crit_sample.cpp`。

**预计工作量**：~1,200 行 C++ 迁移 + 适配 + 测试。

---

### P8（低优先）：GCPC 端源迁移

**现状**
- `gcpc.h` 仅有最小 `GcpcCache` 骨架。
- `gcpc.cpp` 占位。

**需要做的改动**
1. 从 v3 `gcpc_cache.cpp` 迁移完整实现（~2,300 行）。
2. 接入 `endpoint_source.cpp` dispatch。
3. 新增 `test_gcpc.cpp`。

**预计工作量**：~2,300 行 C++ 迁移 + 适配 + 测试。

---

### P9（低优先）：Parallel-by-link 默认策略与基准测试

**现状**
- 实现已完成，但默认 OFF。
- 缺少系统的多线程 vs 单线程 A/B 基准数据。

**需要做的改动**
1. 设计 parallel benchmark（不同核数 × 不同 n_sub × 不同 width）。
2. 产出加速比曲线与最优线程数建议。
3. 决定是否在特定条件下默认开启。

**预计工作量**：~1 天实验 + 文档。

---

### P10（低优先）：测试覆盖补全

**现状**
- 10 项 analytical 测试覆盖核心路径，但缺少：
  - Hybrid 写回路径测试
  - Parallel-by-link 正确性测试（多线程 vs 单线程结果一致）
  - Multi-qi 候选测试
  - Edge case 测试（极窄/极宽 interval、高自由度机器人）
  - EnvelopeCache 测试
  - CritSample/GCPC 单元测试（待迁移后）

**需要做的改动**
1. 新增 `test_analytical_hybrid()`：hybrid on/off 结果一致。
2. 新增 `test_analytical_parallel()`：parallel on/off 结果一致。
3. 新增 `test_analytical_multi_qi()`：multi-qi on/off 结果一致。
4. 新增 `test_envelope_cache.cpp`。

**预计工作量**：~300 行测试代码。

---

### P11（未来大项）：解析微分全面替代采样 + QR

**现状**
- P2 prototype 已证明 symbolic 系数推导可行（gate_accept_ratio ≈ 31%）。
- 当前仅做 compare-only 验证，而 P1（Hybrid 写回）会做首次小范围替代。

**长期路线**
1. **P2/P2.5 全面覆盖**：在 hybrid 路径验证稳定后，逐步将 symbolic 系数作为主路径、QR 作为回退。
2. **去除 QR 冗余计算**：当 symbolic 路径成为默认时，可跳过 9-point sampling + QR，直接从 DH 参数推导系数。
3. **性能目标**：总 FK/拟合相关开销下降 `30%+`。

---

## 三、建议执行顺序

```
阶段 I  — 立即执行（analytical 核心提升）
  P1: Hybrid 写回真实实现
  P2: Hybrid N=10 A/B 验证
  P3: Segment/phase 级 AA 剪枝

阶段 II — 短期执行（性能与可用性）
  P4: pair_2d Root 批量评估
  P5: 增量 Analytical 支持
  P6: Analytical 结果缓存
  P10: 测试覆盖补全

阶段 III — 中期执行（完整端源覆盖）
  P7: CritSample 迁移
  P8: GCPC 迁移
  P9: Parallel-by-link 基准与默认策略

阶段 IV — 长期执行（最大潜在收益）
  P11: 解析微分全面替代
```

---

## 四、代码量与风险评估

| 改动项 | 预计新/改动行数 | 风险等级 | 依赖 |
|--------|----------------|----------|------|
| P1 Hybrid 写回 | ~90 | 低 | 无 |
| P2 Hybrid A/B | ~0（实验） | 低 | P1 |
| P3 Segment AA | ~100 | 低 | 无 |
| P4 pair_2d 批量 | ~120 | 中 | 无 |
| P5 增量 Analytical | ~200 | 中 | 无 |
| P6 结果缓存 | ~150 | 中 | 无 |
| P7 CritSample | ~1,200 | 中 | 无 |
| P8 GCPC | ~2,300 | 中 | 无 |
| P9 Parallel 基准 | ~50+实验 | 低 | 无 |
| P10 测试补全 | ~300 | 低 | P1, P7, P8 |
| P11 解析微分全面替代 | ~500+ | 高 | P1, P2 |

---

## 五、与现有 O5 系列实验的对应关系

| O5 实验 | 结论 | 对应新计划 |
|---------|------|-----------|
| O5.3/O5.3b/O5.3c | 门控阈值标定完成，推荐 `medium` 档 | P1 的前置已满足 |
| O5.4a–O5.4c | Shadow 覆盖度 100%，候选规模低 | P1 需解决"真实写回"问题 |
| O5.4d–O5.4h | Multi-qi `NO-GO(default-off)` | 结论已落地，P1 不依赖 multi-qi |
| O5.5 | 受控切换预案，骨架已完成 | **P1 的直接前身** |
| O5.6 | 骨架接入 + smoke 验证 | 已完成，P1 接续 |
| O5.7 | N=5 重复验证，`go_O5_5_2_N10` | 已完成，P2 接续 |

---

## 六、未解决的技术债务

1. **`endpoint_source.cpp` TODO 注释**（L82, L151）：CritSample/GCPC 回退到 iFK、增量非 iFK 回退。
2. **Python bindings**：CMakeLists 占位，未实现。
3. **benchmark 可否覆盖 CritSample/GCPC**：当前 `exp_analytical_ab_benchmark` 仅针对 Analytical source。
4. **序列化安全**：Analytical config 字段持续增长，需确保 cache metadata 交叉校验覆盖新字段。
