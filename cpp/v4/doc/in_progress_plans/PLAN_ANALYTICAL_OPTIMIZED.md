# Phase C+E 执行计划：Analytical Endpoint Source 迁移 + AA 剪枝 + 解析微分

> 基线：v4 当前 172/172 测试通过，0 错误，0 警告

---

## 概要

从 v3 `envelope_derive_critical.cpp`（3,513 行）迁移 6 阶段解析求解管线到 v4，
同时集成两项新优化：

| 优化 | 原理 | 预期加速 |
|------|------|----------|
| **AA gap 剪枝** | 用 AAFKWorkspace + `aa_position_bounds` 判断 "当前 AABB 是否已包含 AA 范围"，若是则跳过整条 link 的求解 | 2-5× |
| **解析微分 (future)** | DH 矩阵 T_k 在 (cos θ_k, sin θ_k) 线性 → 可直接解析推导 a1..a9 系数，替代 3/9-point FK 采样 + QR | 30-50% FK 调用 |

**注意：第一轮仅实现 AA 剪枝；解析微分标记为 future 占位。**

---

## 依赖前置

| 已完成 | 内容 |
|--------|------|
| Phase A | robot, interval_math, interval_fk, fk_scalar, affine_fk |
| Phase B | envelope_derive |
| Phase H | mmap_util, EndpointStore, EnvelopeCache |

| 需先完成 | 内容 |
|-----------|------|
| **S-1: fk_scalar.cpp 实装** | `dh_transform`, `fk_transforms`, `fk_transforms_inplace` — 当前为 stub |

---

## 步骤清单

### S-1: 实装 fk_scalar.cpp

从 v3 `src/robot/fk_scalar.cpp` 迁移：
- `dh_transform()` — 4×4 DH 变换矩阵
- `fk_transforms()` — 完整 FK 链
- `fk_transforms_inplace()` — 原地 FK（供 FKWorkspace 使用）

在 v4 `include/sbf/robot/fk.h` 中添加 `fk_transforms_inplace` 声明。

### S0: 创建 analytical_utils.h

提取跨阶段共享工具到 `include/sbf/envelope/analytical_utils.h`（~260 行）：

```
kShifts[3]
CRIT_MAX_COMBOS
FKWorkspace { tf[16], np, resize, compute, pos, set_identity, compute_joint, compute_from }
crit_angles(lo, hi) → vector<double>
build_csets(per_joint, intervals, n_joints) → vector<vector<double>>
crit_enum(..., callback)
crit_enum_fk(robot, csets, q, depth, n_needed, ws, callback)
LinkExtremes { vals[6], configs[6], init, update }
eval_and_update(robot, q, n_sub, inv_n, map, n_act, link_seg_ext, ws)
eval_and_update_from(robot, q, from_joint, n_sub, inv_n, map, n_act, link_seg_ext, ws)
FixedRoots { v[8], n, push, begin, end, size }
solve_poly_in_interval(c, deg, lo, hi, roots)
build_symbolic_poly8(a1..a8, poly_out)
half_angle_to_q(t, lo, hi, q_out) → bool
build_bg_values(lo, hi, add_kpi2) → vector<double>
```

### S1: 替换 analytical_solve.h

将 31 行 stub 替换为完整头文件（~120 行）：
- `AnalyticalCriticalConfig` — 完整 15 字段 + `enable_aa_pruning` 新字段
- `AnalyticalCriticalStats` — 完整 10 计数器 + AA 剪枝统计
- 函数声明：`derive_aabb_critical_analytical()`, `_with_configs()` 变体

### S2: 实现 analytical_solve.cpp

从 v3 迁移完整 6 阶段管线（~1,800 行），包括：

| 子步骤 | 内容 | 行数 |
|--------|------|------|
| S2a | AA 剪枝 — `aa_link_can_improve()` 函数 | ~50 |
| S2b | `solve_edges()` — Phase 1 (1D atan2) | ~130 |
| S2c | `solve_faces()` — Phase 2 (2D degree-8 poly) | ~280 |
| S2d | `solve_interior()` — Phase 3 (coordinate descent) | ~160 |
| S2e | `solve_pair_constrained_1d()` — Phase 2.5a | ~180 |
| S2f | `solve_pair_constrained_2d()` — Phase 2.5b | ~240 |
| S2g | `solve_interior_improved()` — Phase 3+ (multi-start) | ~200 |
| S2h | `derive_aabb_critical_analytical()` — 主入口 | ~300 |
| S2i | `_with_configs()` 变体 | ~300 |

### S2-int: endpoint_source.cpp 集成

修改 `compute_endpoint_iaabb()` 的 `case EndpointSource::Analytical`：
- 调用 `derive_aabb_critical_analytical()` 而非回退 iFK
- 将 `link_seg_ext` 结果转为 `endpoint_iaabbs` 格式

### S3: CMakeLists.txt 更新

- 无需添加新 .cpp（analytical_solve.cpp 已在列表中）
- 添加 `test_analytical` 测试可执行文件
- 添加 `exp_analytical_opt` 实验可执行文件（可选）

### S4: test_analytical.cpp

单元测试（~300 行）：
- T1: FKWorkspace 增量一致性
- T2: crit_angles / build_csets 正确性
- T3: solve_poly_in_interval 已知多项式
- T4: build_symbolic_poly8 系数验证
- T5: AA 剪枝逻辑验证
- T6: derive_aabb_critical_analytical 端到端（iiwa14, 1 interval）
- T7: freeze 兼容性（零宽度 interval 自动跳过）

### S5: 构建验证

```
cmake -B build -G Ninja -DSBF_BUILD_TESTS=ON -DSBF_BUILD_EXPERIMENTS=ON
cmake --build build --config Release
ctest --test-dir build --output-on-failure
```

目标：全部测试通过，0 错误，0 警告。

### S6: CHANGE_LOG_CN.md 更新

---

## 执行优先级

```
S-1 (fk_scalar) → S0 (analytical_utils.h) → S1 (analytical_solve.h)
    → S2a-S2i (analytical_solve.cpp) → S2-int (endpoint_source)
    → S3 (CMakeLists) → S5 (build) → S4 (test) → S6 (changelog)
```

## AA 剪枝集成点

在 `derive_aabb_critical_analytical()` 内：

```
Phase 0 (vertex enum) 之后：
  1. 计算 AAFKWorkspace ws_aa
  2. 从 link_seg_ext 提取当前 AABB
  3. 对每个 link ci：aa_link_can_improve(ws_aa.prefix, V, current_aabb)
     → false: 标记 skip_link[ci] = true, ++stats.n_aa_pruned_links

Phase 1 (solve_edges) 内循环：
  if (skip_link[ci]) continue;

Phase 2-3 同理
```

## Freeze 框架兼容性

- 冻结关节 → interval width = 0 → `crit_angles({x,x})` 返回 `{x}`
- `solve_edges` 中 `if (hi_j - lo_j < 1e-12) continue;` 自动跳过
- AA 剪枝：冻结关节 → AAFKWorkspace 使用标量 DH → 无噪声符号 → AA bounds 更紧
- 三系统完全正交，无需特殊处理

---

## 后续优化计划（按优先级）

> 更新时间：2026-03-20
> 目标：在不改变外部 API 语义与安全包络前提下，继续降低 FK 调用与总耗时。

### 高优先：P2（`solve_faces`）接入 root 候选去重

**背景与目标**
- 当前去重收益主要来自 P1；P2（2D degree-8 root 路径）候选量大、重复高，尚未充分去重。
- 目标是在 P2 root 候选生成与评估链路接入与 P1 一致的“命中率统计 + 自适应阈值”机制。

**实施要点**
- 在 `solve_faces()` 中将 root 候选组织为可去重容器（按 `(joint_i, joint_j, t_root_bucket, bg_state)` 归一化键）。
- 复用现有 `DedupAdaptiveRuntime` 决策逻辑：
  - warmup 阶段统计 raw/unique；
  - 达到 `candidate_dedup_min_candidates` 后按 `candidate_dedup_min_hit_rate` 决定 apply/skip。
- 扩展统计：新增或复用 P2 专属字段（raw/unique/applied/skipped），并接入 benchmark CSV。
- 保持数值稳定：root bucket 采用容差量化，避免浮点微扰导致“假不重复”。

**验收标准**
- P2 去重指标在 benchmark 中可见（至少包含 hit_rate 与 applied/skipped）。
- 在固定输入下，启用/禁用 P2 去重时 AABB 结果一致（允许浮点级微差，`1e-6` 内）。
- 中宽区间（如 width=0.35~0.70）下 P2 的 FK 调用下降可观（目标：`>=5%`）。

---

### 中优先：按 link（ci）维度并行 P1/P2/P2.5/P3

**背景与目标**
- 各 link（`ci`）天然独立，适合数据并行；当前串行执行限制多核利用率。
- 目标基于线程池并行各阶段 link 任务，末端安全归并 `LinkExtremes`。

**实施要点**
- 线程池粒度：以 `ci` 为主任务单元，阶段内并行，阶段间保持原有顺序屏障。
- 每线程使用局部 `LinkExtremes`/局部计数器，最终 reduce，避免共享写冲突。
- 统计合并策略：`AnalyticalCriticalStats` 采用线程本地累加后原子/串行归并。
- 可配置开关：`enable_parallel_by_link`、`parallel_num_threads`，默认保守开启（或自动检测核数）。

**验收标准**
- 单线程与多线程结果一致（误差阈值 `1e-6`），统计可重复。
- 8 核机器上端到端耗时有稳定收益（目标：`1.3x~2.0x`，视场景而定）。
- 无数据竞争（TSan/代码审查确认）。

---

### 中优先：AA 剪枝从 link 级增强到 segment/phase 级动态剪枝

**背景与目标**
- 现有 AA 剪枝以 link 粒度为主，仍存在部分 segment/phase 的无效搜索。
- 目标在阶段内动态判断“该 segment/phase 是否仍可能改进当前极值”，做到更细粒度跳过。

**实施要点**
- 建立 `can_improve(ci, seg, phase, axis)` 判定缓存，利用当前 best AABB 与 AA bounds 增量更新。
- 在 P1/P2/P2.5/P3 入口增加快速短路：若该阶段对某 segment 不可改进则直接 `continue`。
- 统计新增：`n_aa_pruned_segments`、`n_aa_pruned_phase_checks`（便于观察剪枝命中）。

**验收标准**
- 在不放宽包络安全性的前提下，阶段内 candidate 数与 FK 调用进一步下降。
- `AA` 剪枝收益在窄区间与中区间均可观测（至少一个场景提升 `>10%`）。

---

### 中优先：`pair_2d` root 批量评估，减少 `compute_from` 调用抖动

**背景与目标**
- 当前 `pair_2d` root 逐个处理，`compute_from` 调用频繁且局部抖动大。
- 目标将 root 处理改为批量分组评估（按相同前缀/相同 from_joint 聚类），提升缓存命中与流水效率。

**实施要点**
- root 排序与分桶：优先按 `from_joint` + 共享前缀配置聚类。
- 对每组 roots 执行 batched `eval_and_update_from`，减少重复 FK 前缀重建。
- 必要时引入小批大小参数：`pair2d_batch_size`，便于平台调优。

**验收标准**
- `pair_2d` 阶段 `compute_from` 次数明显下降（目标：`>=15%`）。
- 总耗时方差下降（多次重复运行的标准差更小）。

---

### 未来大项：解析微分替代部分采样 + QR（最大潜在收益）

**背景与目标**
- 文档已列为 future；理论上可显著减少采样与数值线性代数开销。
- 目标先在 P2/P2.5 的局部链路试点，用解析系数推导替代部分 3/9-point sampling + QR 拟合。

**实施路线（建议分三步）**
1. **原型阶段**：在单 link/单 phase 上实现解析系数与现有采样结果对照（误差与稳定性基线）。
2. **混合阶段**：解析与采样双轨，按条件切换（奇异位姿或数值病态时回退采样）。
3. **推广阶段**：逐步覆盖 P2/P2.5 主路径，并保留回退开关。

**验收标准**
- 精度：与现有方法在关键基准上保持同级别安全包络。
- 性能：在中大区间场景中实现显著降本（长期目标：总 FK/拟合相关开销下降 `30%+`）。

---

## 建议执行顺序（增量落地）

```
O1: P2 root 去重
 → O2: pair_2d 批量评估
 → O3: link 并行（先 P1/P2，再扩 P2.5/P3）
 → O4: segment/phase 级 AA 动态剪枝
 → O5: 解析微分（future, 分阶段试点）
```

## 统一验证与文档同步要求

- 每一项优化完成后，必须同步更新：
  - `doc/CHANGE_LOG_CN.md`（中文变更日志）
  - `docs/API_REFERENCE_CN.md`（若有配置/统计字段变化）
  - benchmark 输出说明（CSV 新列与定义）
- 验证口径统一：
  - 正确性：AABB 包络与基线一致（允许浮点微差）
  - 性能：给出 mean + 方差（至少 10~20 次重复）
  - 可回退：新优化均提供开关，支持 A/B 对照

---

## 下一阶段执行计划（O5.3 → O5.5）

### O5.3：门控决策统计（compare-only）

**目标**
- 在不改变当前求解语义的前提下，给出“若启用混合解析路径，当前 case 会否被门控接受”的统计画像。

**实施内容**
- 在 prototype 统计中新增门控计数：
  - `n_p2_proto_gate_eval`：参与门控评估的 case 数
  - `n_p2_proto_gate_accept`：门控接受（理论可走解析路径）case 数
  - `n_p2_proto_gate_reject_fit`：因拟合残差超阈值拒绝
  - `n_p2_proto_gate_reject_symbolic`：因 symbolic 系数偏差超阈值拒绝
  - `n_p2_proto_gate_reject_singular`：因 symbolic 奇异/不可逆拒绝
- 门控仅记录统计，不改变现有 QR 路径与 envelope 更新逻辑。
- benchmark CSV 增加上述字段与 `gate_accept_ratio`。

**验收标准**
- 默认配置下最终 AABB 与当前版本一致（误差阈值 `1e-6`）。
- 新统计在 A/B 模式均有稳定输出，且 `gate_accept_ratio` 可用于后续阈值调参。

### O5.4：混合路径干跑（shadow run）

**目标**
- 在保留主路径语义不变的前提下，执行“解析候选值 shadow 计算”，评估解析路径对候选生成和极值更新的覆盖度。

**实施内容**
- 新增 `enable_p2_derivative_shadow_run` 开关（默认关）。
- 在门控接受 case 中额外计算解析候选值并与现有候选对照：
  - 候选命中差异分布
  - 极值更新一致率
  - 额外耗时占比
- 保持最终 envelope 仍由现有路径写回。

**验收标准**
- shadow run 不影响结果一致性。
- 获得“可替代覆盖率”和“额外开销”两条曲线，为 O5.5 切换决策提供依据。

### O5.5：受控切换（hybrid on）

**目标**
- 在门控稳定且 shadow 指标达标后，允许解析路径在可控范围内替代部分采样+QR。

**实施内容**
- 新增 `enable_p2_derivative_hybrid`（默认关）与回退策略：
  - 先在 P2 单 phase/单 link 开启
  - 触发异常（奇异、偏差超阈、数值不稳定）即回退采样+QR
- 增加失败回退统计与路径占比统计。

**验收标准**
- 正确性：与基线 envelope 一致（误差阈值 `1e-6`）。
- 性能：在中大区间场景中有可复现收益（mean 提升且方差可控）。
- 可运维：开关与统计完整，支持快速 A/B 回滚。

### 执行顺序与里程碑

1. 完成 O5.3（门控统计 + CSV 扩展 + 文档同步）。
2. 基于 10~20 次重复统计确定初始门控阈值范围。
3. 完成 O5.4（shadow run）并输出覆盖率/开销对照表。
4. 在至少一个稳定场景试点 O5.5 受控切换。

---

## 下一轮进一步优化计划（O5.3b，立即执行）

> 触发背景：当前首轮 gate 结果出现 `gate_accept_ratio = 100%`，需要先做阈值敏感性标定，再进入 O5.4。

### O5.3b-1：阈值扫描能力接入 benchmark

**目标**
- 让 `exp_analytical_ab_benchmark` 支持通过环境变量注入 gate 阈值，避免每次调阈值都重新改代码。

**实施内容**
- 增加环境变量读取：
  - `SBF_P2_GATE_FIT_RMS`
  - `SBF_P2_GATE_SYMBOLIC_RMS`
  - `SBF_P2_GATE_SYMBOLIC_MAX_ABS`
- 将生效阈值写入 CSV 列，确保实验可追溯。

**验收标准**
- 不设置环境变量时与当前默认行为一致。
- 设置环境变量后，CSV 中阈值列与 gate 结果同步变化。

### O5.3b-2：批量阈值实验与推荐区间

**目标**
- 在 10~20 次重复口径下得到 reject 分布与 `gate_accept_ratio` 曲线，给出进入 O5.4 的初始阈值区间。

**实施内容**
- 至少测试 3 组阈值（宽松/中等/严格）。
- 输出汇总：按 width 统计 `accept_ratio_mean/std`、三类 reject 占比。

**验收标准**
- 形成可复现的阈值→门控结果映射表。
- 给出 1 组推荐阈值用于 O5.4 shadow run。

---

## 下一轮进一步优化计划（O5.3c，立即执行）

### O5.3c-1：三档阈值 × 10 次重复（稳定性统计）

**目标**
- 在一致口径下比较宽松/中等/严格三档 gate 阈值，输出可复现的 mean/std 统计。

**阈值矩阵**
- `loose`: `fit=1e-9`, `sym_rms=1e-8`, `sym_max=1e-6`
- `medium`: `fit=3e-17`, `sym_rms=1e-13`, `sym_max=1e-11`
- `strict`: `fit=1e-18`, `sym_rms=1e-14`, `sym_max=1e-12`

**实施内容**
- 每档阈值运行 10 次（每次 benchmark 内部 repeats=12 保持不变）。
- 汇总按 `mode × width × profile` 输出：
  - `gate_accept_ratio_mean/std`
  - `reject_fit_mean/std`
  - `reject_symbolic_mean/std`
  - `reject_singular_mean/std`

**验收标准**
- 产出 `gate_scan_repeat_summary.csv` 与 `gate_scan_recommendation.txt`。
- 汇总文件可直接作为 O5.4 门控初值输入依据。

### O5.3c-2：推荐阈值与 O5.4 入口参数

**目标**
- 依据统计结果选出 1 组“可用且稳定”的推荐阈值，进入 O5.4 shadow run。

**推荐准则**
- `gate_accept_ratio_mean` 不过低（避免 shadow 覆盖度不足）且不过高（避免门控失去区分度）。
- `reject_symbolic` 与 `reject_singular` 保持低位，优先使用 `reject_fit` 作为主门控来源。
- 同时考虑 A/B 模式一致性与标准差稳定性。

### O5.3c 执行结果（2026-03-20）

- 已完成三档阈值各 10 次重复（共 30 次），产物位于：
  - `results/exp_analytical_ab_repeat/o53c_runs/gate_scan_repeat_summary.csv`
  - `results/exp_analytical_ab_repeat/o53c_runs/gate_scan_recommendation.txt`
- 关键统计结论：
  - `loose`：`gate_accept_ratio_mean=100%`（区分度不足）
  - `strict`：`gate_accept_ratio_mean≈8%`（覆盖度偏低）
  - `medium`：`gate_accept_ratio_mean≈31.18%`，作为下一步 shadow run 的折中起点
- 推荐阈值（进入 O5.4 的初值）：
  - `p2_proto_gate_fit_rms_thresh = 3e-17`
  - `p2_proto_gate_symbolic_rms_thresh = 1e-13`
  - `p2_proto_gate_symbolic_max_abs_thresh = 1e-11`

---

## 下一轮进一步优化计划（O5.4a，立即执行）

### O5.4a-1：shadow run 统计骨架接入（compare-only）

**目标**
- 在不改变现有 envelope 写回语义前提下，接入 shadow run 计数口径，先观测覆盖度与候选规模。

**实施内容**
- 新增配置开关：`enable_p2_derivative_shadow_run`（默认关闭）。
- 在 P2 中对 gate-accept 样本记录 shadow 统计：
  - `n_p2_shadow_eval_cases`
  - `n_p2_shadow_root_count`
  - `n_p2_shadow_candidate_count`
- benchmark 增加：
  - 环境变量 `SBF_P2_SHADOW_RUN`
  - CSV 新列（shadow 计数与均值）

**验收标准**
- `SBF_P2_SHADOW_RUN=0` 时行为与当前一致。
- `SBF_P2_SHADOW_RUN=1` 时可观测到 shadow 计数且结果包络不变。

### O5.4a-2：首轮 shadow 基线采样

**目标**
- 用 O5.3c 推荐阈值做首轮 shadow 基线，确认不同 width 下的覆盖度量级。

**实施内容**
- 推荐阈值：`fit=3e-17, sym_rms=1e-13, sym_max=1e-11`。
- 运行 benchmark 并输出 shadow 列结果。

**验收标准**
- 产出首轮 `shadow_run` 日志与可追溯 CSV。

---

## 下一轮进一步优化计划（O5.4b，立即执行）

### O5.4b-1：shadow run 10 次重复统计

**目标**
- 在推荐阈值下评估 shadow 覆盖率稳定性，形成 mean/std 结论。

**实施内容**
- 固定参数：`SBF_P2_SHADOW_RUN=1`，`fit=3e-17`，`sym_rms=1e-13`，`sym_max=1e-11`。
- 执行 10 次 benchmark，输出到 `results/exp_analytical_ab_repeat/o54b_runs/`。
- 汇总维度：`mode × width`。

**验收标准**
- 生成 `shadow_repeat_summary.csv`（含 mean/std）。
- 每个 width 均有有效 shadow 统计行。

### O5.4b-2：覆盖率结论与 O5.5 入口建议

**目标**
- 给出是否进入 O5.5 受控切换试点的门控结论。

**实施内容**
- 汇总以下指标并给出结论：
  - `p2_shadow_eval_cases` 与 `p2_proto_gate_accept` 的比例（shadow 覆盖率）
  - `p2_shadow_root_count / p2_shadow_eval_cases`
  - `p2_shadow_candidate_count / p2_shadow_eval_cases`
- 形成 `shadow_recommendation.txt`，包含“继续 O5.4 深化”或“进入 O5.5 小范围试点”的建议。

**验收标准**
- 有明确推荐结论与阈值快照，可直接用于下一轮实现。

**执行结果（2026-03-20，已完成）**
- 已完成 10 次重复并产出：
  - `results/exp_analytical_ab_repeat/o54b_runs/shadow_repeat_summary.csv`
  - `results/exp_analytical_ab_repeat/o54b_runs/shadow_recommendation.txt`
- 关键结论（`B_dedup_adaptive_proto`，按 width 聚合后再求均值）：
  - `avg_shadow_coverage_pct = 100.00`
  - `avg_shadow_candidate_per_eval = 0.0075`
- 推荐结论：`stay_O5_4_deepen_low_candidate_scale`
- 下一步建议：保持 compare-only 语义，优先在 O5.4 深化“候选规模提升”（提高 gate-accept 样本中的有效解析候选产出），暂不进入 O5.5 写回切换。

---

## 下一轮进一步优化计划（O5.4c，立即执行）

### O5.4c-1：shadow 扩展候选统计（all-branch，compare-only）

**目标**
- 在不改变主求解写回与去重行为的前提下，评估“每个 root 的所有可行 `qi` 分支”带来的候选规模上限。

**实施内容**
- 新增配置开关：`enable_p2_shadow_all_qi_branches`（默认关闭）。
- 在 `solve_faces()` 的 shadow 统计中，增加 all-branch 口径：
  - 对每个 `tj_root`，枚举 `qi_cand / qi_cand±pi` 与 `kShifts` 的全部落区间解；
  - 记录扩展口径候选计数（仅统计，不参与 `eval_and_update_from` 与主路径去重）。
- 新增 shadow 扩展统计字段并接入 benchmark CSV：
  - `n_p2_shadow_candidate_count_all_branches`
  - `p2_shadow_candidate_per_eval_all_branches`

**验收标准**
- `enable_p2_shadow_all_qi_branches=0` 时行为与当前完全一致。
- `enable_p2_shadow_all_qi_branches=1` 时，扩展统计列可见，且主 AABB 结果与 baseline 一致（`1e-6` 内）。

### O5.4c-2：首轮验证与是否进入 O5.4d

**目标**
- 用 O5.3c 推荐阈值评估 all-branch 统计是否显著提升候选规模，并判断是否进入 O5.4d（真实候选生成策略优化）。

**实施内容**
- 固定参数：`SBF_P2_SHADOW_RUN=1`，`SBF_P2_SHADOW_ALL_QI=1`，`fit=3e-17`，`sym_rms=1e-13`，`sym_max=1e-11`。
- 先执行 1 次 smoke run，确认字段输出与结果一致性；再决定是否扩展到重复统计。

**验收标准**
- 产出 `shadow_all_branch_run_01.log/.err.log`。
- 输出 go/no-go 结论：进入 O5.4d（若 all-branch 指标明显抬升）或继续 O5.4c 深化。

**执行结果（2026-03-20，已完成）**
- 已完成 smoke run 并产出：
  - `results/exp_analytical_ab_repeat/o54c_runs/shadow_all_branch_run_01.log`
  - `results/exp_analytical_ab_repeat/o54c_runs/shadow_all_branch_run_01.err.log`
  - `results/exp_analytical_ab_repeat/o54c_runs/shadow_all_branch_recommendation.txt`
- 关键结论（`B_dedup_adaptive_proto`，5 个 width 汇总）：
  - `sum_eval = 2133`
  - `sum_candidate = 19`（`candidate_per_eval = 0.008908`）
  - `sum_candidate_all_branches = 23`（`candidate_per_eval_all_branches = 0.010783`）
  - `uplift_pct = 21.05%`
- 决策：`go_O5_4d_candidate_generation_strategy`
- 解释：all-branch 统计对候选规模有可观抬升，但绝对规模仍低，维持 compare-only，下一步进入 O5.4d 做真实候选生成策略优化。

---

## 下一轮进一步优化计划（O5.4d，立即执行）

### O5.4d-1：P2 多 `qi` 分支真实候选生成（可回退开关）

**目标**
- 将 O5.4c 的 all-branch 结论落地到真实候选生成路径，在不破坏默认语义的前提下提升 P2 候选规模。

**实施内容**
- 新增配置开关：`enable_p2_multi_qi_candidates`（默认关闭）。
- 在 `solve_faces()` 中，当开关开启时：
  - 对每个 root 不再“命中首个可行分支即 break”；
  - 采集全部落区间 `qi` 分支并交给现有 dedup 统一去重。
- 保持 `enable_p2_multi_qi_candidates=0` 时行为与当前一致。

**验收标准**
- 关闭开关时结果与 baseline 一致。
- 开启开关后 `n_dedup_raw_candidates_p2` 与/或 `n_dedup_unique_candidates_p2` 提升，且结果包络稳定（`1e-6` 内）。

### O5.4d-2：首轮 A/B 对照与 go/no-go

**目标**
- 验证“多 `qi` 分支真实候选生成”是否带来可接受收益并决定是否扩展重复实验。

**实施内容**
- 固定阈值：`fit=3e-17`, `sym_rms=1e-13`, `sym_max=1e-11`。
- 运行两组 smoke：
  - baseline：`SBF_P2_MULTI_QI_CAND=0`
  - trial：`SBF_P2_MULTI_QI_CAND=1`
- 比较 `p2 dedup raw/unique`、`fk_total`、`mean_ms`。

**验收标准**
- 产出 `o54d_runs/multi_qi_baseline_01.log` 与 `multi_qi_trial_01.log`。
- 输出结论：继续 O5.4d 扩展统计或回退保持 O5.4c 路径。

**执行结果（2026-03-20，已完成）**
- 已完成 baseline/trial 首轮 smoke，并产出：
  - `results/exp_analytical_ab_repeat/o54d_runs/multi_qi_baseline_01.log`
  - `results/exp_analytical_ab_repeat/o54d_runs/multi_qi_baseline_01.err.log`
  - `results/exp_analytical_ab_repeat/o54d_runs/multi_qi_trial_01.log`
  - `results/exp_analytical_ab_repeat/o54d_runs/multi_qi_trial_01.err.log`
  - `results/exp_analytical_ab_repeat/o54d_runs/multi_qi_compare_01.csv`
  - `results/exp_analytical_ab_repeat/o54d_runs/multi_qi_recommendation_01.txt`
- 关键对照结论（`B_dedup_adaptive_proto`，5 个 width 平均）：
  - `avg_mean_ms_delta_pct = -0.094%`（耗时基本持平）
  - `avg_mean_fk_p2_delta_pct = +0.345%`
  - `avg_mean_fk_total_delta_pct = +0.009%`
- 决策：`NO-GO(default-off)`。
- 解释：本轮 smoke 下“多 `qi` 分支真实候选生成”带来 P2/FK 计算量小幅上升，未观察到足以支撑默认开启的收益；保留开关能力，后续仅在高分支场景继续试点。

---

## 下一轮进一步优化计划（O5.4e，立即执行）

### O5.4e-1：benchmark 接入“高分支定向样本”入口（可配置 width 集）

**目标**
- 在不改默认 benchmark 行为的前提下，支持通过环境变量注入 width 列表，构造更高分支密度的定向样本集。

**实施内容**
- 在 `exp_analytical_ab_benchmark` 新增可选环境变量：
  - `SBF_BENCH_WIDTHS`，格式示例：`0.35,0.70,1.00,1.40`
- 若未设置该变量，保持当前默认 `0.10,0.20,0.35,0.50,0.70`。
- CSV 新增回显列：
  - `bench_width_profile`（`default` / `env_custom`）

**验收标准**
- 不设置环境变量时，输出与当前版本兼容。
- 设置 `SBF_BENCH_WIDTHS` 后，CSV 行的 width 集与回显列正确。

### O5.4e-2：高分支场景 compare-only smoke（multi-qi 开关 A/B）

**目标**
- 在高分支定向样本下重新评估 `enable_p2_multi_qi_candidates` 的有效性，判断是否值得进入重复实验。

**实施内容**
- 固定门控阈值：`fit=3e-17`, `sym_rms=1e-13`, `sym_max=1e-11`。
- 使用 `SBF_BENCH_WIDTHS` 构造更宽区间样本，并执行两组 smoke：
  - baseline：`SBF_P2_MULTI_QI_CAND=0`
  - trial：`SBF_P2_MULTI_QI_CAND=1`
- 比较 `mean_ms`、`mean_fk_p2`、`mean_fk_total` 与 `p2 dedup raw/unique`。

**验收标准**
- 产出 `o54e_runs/high_branch_baseline_01.log` 与 `high_branch_trial_01.log`。
- 输出首轮 go/no-go（继续扩展重复或保持默认关闭）。

**执行结果（2026-03-20，已完成）**
- 已完成 O5.4e 首轮实现与 smoke，产出：
  - `results/exp_analytical_ab_repeat/o54e_runs/high_branch_baseline_01.log`
  - `results/exp_analytical_ab_repeat/o54e_runs/high_branch_baseline_01.err.log`
  - `results/exp_analytical_ab_repeat/o54e_runs/high_branch_trial_01.log`
  - `results/exp_analytical_ab_repeat/o54e_runs/high_branch_trial_01.err.log`
  - `results/exp_analytical_ab_repeat/o54e_runs/high_branch_compare_01.csv`
  - `results/exp_analytical_ab_repeat/o54e_runs/high_branch_recommendation_01.txt`
- 自定义 width 配置生效确认：
  - `bench_width_profile = env_custom`
  - `widths = 0.35,0.70,1.00,1.40`
- 关键对照结论（`B_dedup_adaptive_proto`，4 个 width 平均）：
  - `avg_mean_ms_delta_pct = +0.489%`
  - `avg_mean_fk_p2_delta_pct = +1.220%`
  - `avg_mean_fk_total_delta_pct = +0.032%`
- 决策：`NO-GO(default-off)`。
- 解释：在高分支定向样本下，`multi-qi` 仍表现为额外开销上升且未观察到可支撑默认开启的收益；后续仅保留开关用于进一步定向试点。

---

## 下一轮进一步优化计划（O5.4f，立即执行）

### O5.4f-1：高分支场景 10 次重复 A/B（稳定性统计）

**目标**
- 在 O5.4e 的高分支 width 配置下，补齐 10 次重复统计，验证 `multi-qi` 结论是否稳定。

**实施内容**
- 固定参数：
  - `SBF_BENCH_WIDTHS=0.35,0.70,1.00,1.40`
  - `fit=3e-17`, `sym_rms=1e-13`, `sym_max=1e-11`
  - `SBF_P2_SHADOW_RUN=0`, `SBF_P2_SHADOW_ALL_QI=0`
- 每次重复运行两组：
  - baseline：`SBF_P2_MULTI_QI_CAND=0`
  - trial：`SBF_P2_MULTI_QI_CAND=1`
- 输出到 `results/exp_analytical_ab_repeat/o54f_runs/`。

**验收标准**
- 产出 `high_branch_baseline_run_01..10.log` 与 `high_branch_trial_run_01..10.log`。
- 每组均含 `bench_width_profile=env_custom` 且 width 集完整。

### O5.4f-2：汇总 mean/std 与最终推荐（高分支口径）

**目标**
- 输出可复现的稳定性统计并给出 `multi-qi` 在高分支场景下的 go/no-go 最终建议。

**实施内容**
- 按 `mode=B_dedup_adaptive_proto` 聚合 10 次重复，统计：
  - `avg_mean_ms_delta_pct_mean/std`
  - `avg_mean_fk_p2_delta_pct_mean/std`
  - `avg_mean_fk_total_delta_pct_mean/std`
- 产出：
  - `high_branch_repeat_summary.csv`
  - `high_branch_repeat_recommendation.txt`

**验收标准**
- 形成明确建议（`GO` 或 `NO-GO`），并给出是否继续保留“仅开关试点”策略。

**执行结果（2026-03-20，已完成）**
- 已完成 O5.4f 的高分支 10 次重复 A/B，并产出：
  - `results/exp_analytical_ab_repeat/o54f_runs/high_branch_baseline_run_01..10.log`
  - `results/exp_analytical_ab_repeat/o54f_runs/high_branch_trial_run_01..10.log`
  - `results/exp_analytical_ab_repeat/o54f_runs/high_branch_compare_run_01..10.csv`
  - `results/exp_analytical_ab_repeat/o54f_runs/high_branch_repeat_summary.csv`
  - `results/exp_analytical_ab_repeat/o54f_runs/high_branch_repeat_recommendation.txt`
- 配置生效确认：`bench_width_profile=env_custom`，`widths=0.35,0.70,1.00,1.40`。
- 关键汇总（10 runs，`B_dedup_adaptive_proto`，按每 run 的宽度平均再聚合）：
  - `avg_mean_ms_delta_pct_mean = -0.802%`，`std = 3.665%`
  - `avg_mean_fk_p2_delta_pct_mean = +1.131%`，`std = 0.211%`
  - `avg_mean_fk_total_delta_pct_mean = +0.029%`，`std = 0.006%`
- 决策：`NO-GO(default-off)`。
- 解释：高分支 profile 下 `multi-qi` 对 P2/FK 工作量增加趋势稳定，未形成可支撑默认开启的收益；继续仅保留开关能力用于定向试点。

---

## 下一轮进一步优化计划（O5.4g，立即执行）

### O5.4g-1：`multi-qi` 条件门控（root/width）最小实现

**目标**
- 在保持 `enable_p2_multi_qi_candidates` 默认关闭与现有语义不变前提下，增加更细粒度的“按 case 启用”门控，降低高分支场景额外 FK 开销。

**实施内容**
- 在 `AnalyticalCriticalConfig` 新增两项门控参数（默认保守，保持旧行为兼容）：
  - `p2_multi_qi_min_root_count`（默认 `1`）
  - `p2_multi_qi_min_joint_width`（默认 `0.0`）
- 在 `solve_faces()` 的 P2 候选生成中，将是否开启“全 `qi` 分支收集”改为 case 级判定：
  - `enable_p2_multi_qi_candidates == true`
  - 且 `tj_roots.size() >= p2_multi_qi_min_root_count`
  - 且 `max(hi_i-lo_i, hi_j-lo_j) >= p2_multi_qi_min_joint_width`
- 若上述任一条件不满足，维持单分支早停（与旧路径一致）。

**验收标准**
- 默认配置（不开 `multi-qi`）输出与当前版本一致。
- 当 `p2_multi_qi_min_root_count=1` 且 `p2_multi_qi_min_joint_width=0` 时，与 O5.4d 行为一致。

### O5.4g-2：benchmark 接入门控参数与 smoke A/B

**目标**
- 通过环境变量快速扫描门控参数，在高分支 profile 下做首轮 smoke，判断是否改善 O5.4f 的开销趋势。

**实施内容**
- `exp_analytical_ab_benchmark` 新增环境变量与 CSV 回显：
  - `SBF_P2_MULTI_QI_MIN_ROOTS`
  - `SBF_P2_MULTI_QI_MIN_WIDTH`
- 高分支 smoke 固定：
  - `SBF_BENCH_WIDTHS=0.35,0.70,1.00,1.40`
  - gate 阈值：`3e-17 / 1e-13 / 1e-11`
  - baseline：`SBF_P2_MULTI_QI_CAND=0`
  - trial：`SBF_P2_MULTI_QI_CAND=1`, `SBF_P2_MULTI_QI_MIN_ROOTS=2`, `SBF_P2_MULTI_QI_MIN_WIDTH=0.70`

**验收标准**
- 产出：
  - `results/exp_analytical_ab_repeat/o54g_runs/high_branch_gate_baseline_01.log`
  - `results/exp_analytical_ab_repeat/o54g_runs/high_branch_gate_trial_01.log`
  - `results/exp_analytical_ab_repeat/o54g_runs/high_branch_gate_compare_01.csv`
  - `results/exp_analytical_ab_repeat/o54g_runs/high_branch_gate_recommendation_01.txt`
- 输出 go/no-go（继续扩大重复统计或保持 default-off）。

**执行结果（2026-03-20，已完成）**
- 已完成 O5.4g 代码接入与高分支 smoke，产出：
  - `results/exp_analytical_ab_repeat/o54g_runs/high_branch_gate_baseline_01.log`
  - `results/exp_analytical_ab_repeat/o54g_runs/high_branch_gate_baseline_01.err.log`
  - `results/exp_analytical_ab_repeat/o54g_runs/high_branch_gate_trial_01.log`
  - `results/exp_analytical_ab_repeat/o54g_runs/high_branch_gate_trial_01.err.log`
  - `results/exp_analytical_ab_repeat/o54g_runs/high_branch_gate_compare_01.csv`
  - `results/exp_analytical_ab_repeat/o54g_runs/high_branch_gate_recommendation_01.txt`
- 生效 gate 参数：`multi_qi_min_roots=2`, `multi_qi_min_width=0.70`。
- 关键对照结论（`B_dedup_adaptive_proto`，4 个 width 平均）：
  - `avg_mean_ms_delta_pct = -2.030%`
  - `avg_mean_fk_p2_delta_pct = +0.346%`
  - `avg_mean_fk_total_delta_pct = +0.009%`
- 决策：`NO-GO(default-off)`。
- 解释：本轮门控可降低 wall-time，但仍伴随 P2/FK 轻微增量，尚不足以支持默认开启；继续保持开关试点策略。

---

## 下一轮进一步优化计划（O5.4h，立即执行）

### O5.4h-1：O5.4g 门控参数 10 次重复 A/B（稳定性统计）

**目标**
- 在 O5.4g 的门控参数（`min_roots=2`, `min_width=0.70`）下补齐 10 次重复统计，验证“耗时下降 + FK 轻微上升”是否稳定可复现。

**实施内容**
- 固定参数：
  - `SBF_BENCH_WIDTHS=0.35,0.70,1.00,1.40`
  - gate 阈值：`fit=3e-17`, `sym_rms=1e-13`, `sym_max=1e-11`
  - `SBF_P2_SHADOW_RUN=0`, `SBF_P2_SHADOW_ALL_QI=0`
  - 门控参数：`SBF_P2_MULTI_QI_MIN_ROOTS=2`, `SBF_P2_MULTI_QI_MIN_WIDTH=0.70`
- 每次重复运行两组：
  - baseline：`SBF_P2_MULTI_QI_CAND=0`
  - trial：`SBF_P2_MULTI_QI_CAND=1`
- 输出到 `results/exp_analytical_ab_repeat/o54h_runs/`。

**验收标准**
- 产出 `high_branch_gate_baseline_run_01..10.log` 与 `high_branch_gate_trial_run_01..10.log`。
- 每组均含 `bench_width_profile=env_custom`，且新列 `multi_qi_min_roots/multi_qi_min_width` 回显正确。

### O5.4h-2：汇总 mean/std 与是否继续深化门控

**目标**
- 形成 O5.4g 门控参数在高分支 profile 下的稳定性结论，并给出下一步 go/no-go。

**实施内容**
- 按 `mode=B_dedup_adaptive_proto` 聚合 10 次重复，统计：
  - `avg_mean_ms_delta_pct_mean/std`
  - `avg_mean_fk_p2_delta_pct_mean/std`
  - `avg_mean_fk_total_delta_pct_mean/std`
- 产出：
  - `high_branch_gate_repeat_summary.csv`
  - `high_branch_gate_repeat_recommendation.txt`

**验收标准**
- 输出明确建议：
  - 若 `ms` 下降且 `fk_total` 增量可忽略且稳定，则进入下一轮门控参数扫描；
  - 否则继续 `NO-GO(default-off)` 并保留开关试点。

**执行结果（2026-03-20，已完成）**
- 已完成 O5.4h 的高分支门控参数 10 次重复 A/B，并产出：
  - `results/exp_analytical_ab_repeat/o54h_runs/high_branch_gate_baseline_run_01..10.log`
  - `results/exp_analytical_ab_repeat/o54h_runs/high_branch_gate_baseline_run_01..10.err.log`
  - `results/exp_analytical_ab_repeat/o54h_runs/high_branch_gate_trial_run_01..10.log`
  - `results/exp_analytical_ab_repeat/o54h_runs/high_branch_gate_trial_run_01..10.err.log`
  - `results/exp_analytical_ab_repeat/o54h_runs/high_branch_gate_compare_run_01..10.csv`
  - `results/exp_analytical_ab_repeat/o54h_runs/high_branch_gate_repeat_summary.csv`
  - `results/exp_analytical_ab_repeat/o54h_runs/high_branch_gate_repeat_recommendation.txt`
- 配置生效确认：`bench_width_profile=env_custom`，`widths=0.35,0.70,1.00,1.40`，trial 侧 `multi_qi_min_roots=2`、`multi_qi_min_width=0.70`。
- 关键汇总（10 runs，`B_dedup_adaptive_proto`，按每 run 的宽度平均再聚合）：
  - `avg_mean_ms_delta_pct_mean = -0.532%`，`std = 2.489%`
  - `avg_mean_fk_p2_delta_pct_mean = +0.277%`，`std = 0.146%`
  - `avg_mean_fk_total_delta_pct_mean = +0.007%`，`std = 0.004%`
- 决策：`NO-GO(default-off)`。
- 解释：门控参数下 wall-time 均值仍为小幅下降，但 FK（尤其 P2）存在稳定轻微增量；在当前口径下仍不支持默认开启，继续保持开关试点策略。

---

## 下一轮进一步优化计划（O5.5，条件受控切换预案）

> 触发背景：O5.4h 结论为 `NO-GO(default-off)`，因此 O5.5 不做“默认开启”，仅做**可回退、受控、小范围**试点预案。

### O5.5-1：受控切换开关与回退护栏（默认关闭）

**目标**
- 在不改变默认语义前提下，为 O5.5 提供最小可控入口：仅当显式开启时才允许解析路径参与候选写回。

**实施内容**
- 在配置中保留/启用以下控制项（默认均为关闭或保守值）：
  - `enable_p2_derivative_hybrid`（主开关，默认 `false`）
  - `enable_p2_hybrid_fallback`（异常自动回退，默认 `true`）
  - `p2_hybrid_max_abs_error`（回退阈值，默认保守）
- 受控切换只允许在 `B_dedup_adaptive_proto` 模式下试点，其他模式维持旧路径。
- 任一异常触发立即回退到采样+QR：
  - symbolic 奇异/不可逆
  - 拟合残差超阈
  - 候选越界或数值不稳定

**验收标准**
- 默认配置下行为与 O5.4h 完全一致。
- 开启 hybrid 但触发异常时可稳定回退，且不影响最终包络安全性。

### O5.5-2：单场景小流量试点（1 profile × 1 width 集）

**目标**
- 在最小风险范围内验证“hybrid 写回”是否具备正收益趋势。

**实施内容**
- 固定高分支 profile：`SBF_BENCH_WIDTHS=0.35,0.70,1.00,1.40`。
- 仅在一组门控参数下试点：`min_roots=2`, `min_width=0.70`。
- 对照组：
  - baseline：`enable_p2_derivative_hybrid=0`
  - trial：`enable_p2_derivative_hybrid=1`（开启回退护栏）
- 重复次数：先 `N=5` smoke-repeat，若无异常再扩至 `N=10`。

**验收标准**
- 正确性：AABB 与 baseline 一致（`1e-6` 内）。
- 稳定性：无未回退异常；回退统计可追踪。
- 性能：`mean_ms` 至少不劣化，且 `fk_total` 增量可忽略。

### O5.5-3：试点决策门槛与路线分叉

**目标**
- 给出 O5.5 结束后的明确 go/no-go 判定，避免长期停留在灰区。

**实施内容**
- 产出：
  - `o55_runs/hybrid_repeat_summary.csv`
  - `o55_runs/hybrid_recommendation.txt`
- 判定规则：
  - **GO（扩大试点）**：`mean_ms` 改善稳定，且 `fk_total` 无显著上升，回退率处于低位。
  - **NO-GO（继续 default-off）**：任一关键指标恶化或回退率过高。

**验收标准**
- recommendation 必须给出单一结论（`GO_limited_rollout` 或 `NO_GO_default_off`）。
- 结论可直接驱动下一轮（扩大试点 or 回到 O5.4 参数继续优化）。

---

## 下一轮进一步优化计划（O5.6，立即执行）

> 触发背景：O5.5 当前为受控预案，需要先完成“最小可执行骨架 + 可观测性”再进入真正 hybrid 写回试点。

### O5.6 计划摘要（确认版）

- **目标**：先完成 O5.5-1 的最小工程闭环（配置、统计、实验入口），不改变默认语义；再做 1 轮 baseline/trial smoke 验证骨架有效性。
- **固定场景**：`SBF_BENCH_WIDTHS=0.35,0.70,1.00,1.40`，并沿用 O5.4h 的门控参数（`min_roots=2`, `min_width=0.70`）。
- **A/B 设计**：
  - baseline：`SBF_P2_DERIVATIVE_HYBRID=0`
  - trial：`SBF_P2_DERIVATIVE_HYBRID=1`（`SBF_P2_HYBRID_FALLBACK=1`）
- **必需产物**：`o56_hybrid_baseline_01.log`、`o56_hybrid_trial_01.log`、`o56_hybrid_compare_01.csv`、`o56_hybrid_recommendation_01.txt`。
- **验收门槛**：CSV 必须回显 hybrid 配置列与统计列；若骨架统计稳定且无异常，则进入 O5.5-2 的重复验证。

### O5.6-1：O5.5-1 骨架落地（配置/统计/实验入口）

**目标**
- 在不改变默认求解语义前提下，完成 O5.5-1 的最小工程闭环：开关可配置、状态可观测、实验可追踪。

**实施内容**
- 在 `AnalyticalCriticalConfig` 增加 O5.5 受控字段（默认保守）：
  - `enable_p2_derivative_hybrid`（默认 `false`）
  - `enable_p2_hybrid_fallback`（默认 `true`）
  - `p2_hybrid_max_abs_error`（默认 `1e-6`）
- 在 `AnalyticalCriticalStats` 增加 O5.5 观测字段（先打点，后续接真实写回路径）：
  - `n_p2_hybrid_eval_cases`
  - `n_p2_hybrid_applied_cases`
  - `n_p2_hybrid_fallback_cases`
- 在 benchmark 增加环境变量入口与 CSV 回显列：
  - `SBF_P2_DERIVATIVE_HYBRID`
  - `SBF_P2_HYBRID_FALLBACK`
  - `SBF_P2_HYBRID_MAX_ABS_ERR`

**验收标准**
- 默认配置下结果与 O5.4h 口径一致。
- benchmark 可通过环境变量切换并在 CSV 中回显生效参数/计数列。

### O5.6-2：首轮 smoke（baseline/trial）与执行结论

**目标**
- 用 1 轮最小 A/B 验证 O5.6-1 骨架完整性，为 O5.5-2 的重复试点做准备。

**实施内容**
- 运行两组 smoke：
  - baseline：`SBF_P2_DERIVATIVE_HYBRID=0`
  - trial：`SBF_P2_DERIVATIVE_HYBRID=1`, `SBF_P2_HYBRID_FALLBACK=1`
- 固定高分支 profile 与既有门控参数：
  - `SBF_BENCH_WIDTHS=0.35,0.70,1.00,1.40`
  - `SBF_P2_MULTI_QI_MIN_ROOTS=2`, `SBF_P2_MULTI_QI_MIN_WIDTH=0.70`
- 产出：
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_baseline_01.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_trial_01.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_compare_01.csv`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_recommendation_01.txt`

**验收标准**
- 工件齐套，CSV 含 O5.6 新列。
- 给出 go/no-go：进入 O5.5-2 的 N=5 重复，或继续完善骨架。

**执行结果（2026-03-20，已完成）**
- 已完成 O5.6-1 骨架接入并完成 O5.6-2 首轮 smoke，产出：
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_baseline_01.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_baseline_01.err.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_trial_01.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_trial_01.err.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_compare_01.csv`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_recommendation_01.txt`
- 新列生效确认：
  - 配置回显列：`enable_hybrid`、`enable_hybrid_fallback`、`hybrid_max_abs_err`
  - 统计列：`p2_hybrid_eval_cases`、`p2_hybrid_applied_cases`、`p2_hybrid_fallback_cases`
- 首轮对照结论（`B_dedup_adaptive_proto`，4 个 width 平均）：
  - `avg_mean_ms_delta_pct = +0.092%`
  - `avg_mean_fk_p2_delta_pct = +0.000%`
  - `avg_mean_fk_total_delta_pct = +0.000%`
  - `avg_trial_p2_hybrid_eval_cases = 0.000`
  - `avg_trial_p2_hybrid_applied_cases = 0.000`
  - `avg_trial_p2_hybrid_fallback_cases = 0.000`
- 决策：`go_O5_5_2_repeat_for_scaffold_validation`。
- 解释：当前为骨架阶段（尚未接入真实 hybrid 写回），统计行为符合预期；下一步进入 O5.5-2 的 `N=5` 重复验证可观测性与稳定性。

**补充执行结果（run_02，2026-03-20）**
- 已按相同 O5.6 参数完成一次重执行，新增产物：
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_baseline_02.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_baseline_02.err.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_trial_02.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_trial_02.err.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_compare_02.csv`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_recommendation_02.txt`
- run_02 关键结论（`B_dedup_adaptive_proto`，4 个 width 平均）：
  - `avg_mean_ms_delta_pct = -0.863%`
  - `avg_mean_fk_p2_delta_pct = +0.000%`
  - `avg_mean_fk_total_delta_pct = +0.000%`
  - `avg_trial_p2_hybrid_eval/applied/fallback_cases = 0.000 / 0.000 / 0.000`
- 决策保持：`go_O5_5_2_repeat_for_scaffold_validation`。

---

## 下一轮进一步优化计划（O5.7，立即执行）

> 触发背景：O5.6 首轮 smoke 已确认 O5.5 骨架开关/统计可观测，下一步执行 O5.5-2 的 `N=5` 重复稳定性验证。

### O5.7 计划摘要（确认版）

- **目标**：在骨架阶段完成 O5.5-2 的 `N=5` 重复 A/B，验证统计稳定性与可追溯性。
- **固定场景**：`SBF_BENCH_WIDTHS=0.35,0.70,1.00,1.40`，gate 维持 `3e-17 / 1e-13 / 1e-11`，并保持 `multi_qi_min_roots=2`、`multi_qi_min_width=0.70`。
- **A/B 设计**：
  - baseline：`SBF_P2_DERIVATIVE_HYBRID=0`
  - trial：`SBF_P2_DERIVATIVE_HYBRID=1`（`SBF_P2_HYBRID_FALLBACK=1`）
- **必需产物**：`o57_hybrid_baseline_run_01..05.log`、`o57_hybrid_trial_run_01..05.log`、`o57_hybrid_compare_run_01..05.csv`、`o57_hybrid_repeat_summary.csv`、`o57_hybrid_repeat_recommendation.txt`。
- **验收门槛**：输出单一决策（`go_O5_5_2_N10` 或 `hold_O5_5_2_refine_scaffold`），并以 repeat mean/std 作为下一轮是否扩展到 N=10 的依据。

### O5.7-1：O5.5-2 骨架阶段 N=5 重复 A/B

**目标**
- 在高分支固定 profile 下验证 O5.5 骨架统计在重复运行中的稳定性与一致性。

**实施内容**
- 固定参数：
  - `SBF_BENCH_WIDTHS=0.35,0.70,1.00,1.40`
  - gate：`fit=3e-17`, `sym_rms=1e-13`, `sym_max=1e-11`
  - `SBF_P2_MULTI_QI_CAND=1`, `SBF_P2_MULTI_QI_MIN_ROOTS=2`, `SBF_P2_MULTI_QI_MIN_WIDTH=0.70`
  - `SBF_P2_HYBRID_FALLBACK=1`, `SBF_P2_HYBRID_MAX_ABS_ERR=1e-6`
- 每轮两组：
  - baseline：`SBF_P2_DERIVATIVE_HYBRID=0`
  - trial：`SBF_P2_DERIVATIVE_HYBRID=1`
- 共执行 5 轮，输出到 `results/exp_analytical_ab_repeat/o57_runs/`。

**验收标准**
- 产出 `o57_hybrid_baseline_run_01..05.log` 与 `o57_hybrid_trial_run_01..05.log`。
- 每轮 CSV 均包含 hybrid 配置回显列与 hybrid 统计列。

### O5.7-2：repeat 汇总与是否进入 N=10

**目标**
- 形成 O5.5-2 在骨架阶段的 repeat 结论，并决定是否扩展到 `N=10`。

**实施内容**
- 按 `B_dedup_adaptive_proto` 对 5 轮聚合，统计：
  - `avg_mean_ms_delta_pct_mean/std`
  - `avg_mean_fk_p2_delta_pct_mean/std`
  - `avg_mean_fk_total_delta_pct_mean/std`
  - `avg_trial_p2_hybrid_eval/applied/fallback_cases_mean/std`
- 产出：
  - `o57_hybrid_repeat_summary.csv`
  - `o57_hybrid_repeat_recommendation.txt`

**验收标准**
- recommendation 给出单一建议：
  - `go_O5_5_2_N10`（若重复稳定且无异常）
  - 或 `hold_O5_5_2_refine_scaffold`（若统计不稳定或出现异常）

**执行结果（2026-03-20，已完成）**
- 已完成 O5.7 的 N=5 baseline/trial 重复，并产出：
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_baseline_run_01..05.log`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_baseline_run_01..05.err.log`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_trial_run_01..05.log`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_trial_run_01..05.err.log`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_compare_run_01..05.csv`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_repeat_summary.csv`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_repeat_recommendation.txt`
- 配置与字段回显确认：
  - `enable_hybrid` / `enable_hybrid_fallback` / `hybrid_max_abs_err`
  - `p2_hybrid_eval_cases` / `p2_hybrid_applied_cases` / `p2_hybrid_fallback_cases`
- 关键汇总（5 runs，`B_dedup_adaptive_proto`，按每 run 的宽度平均再聚合）：
  - `avg_mean_ms_delta_pct_mean = -0.606%`，`std = 3.232%`
  - `avg_mean_fk_p2_delta_pct_mean = +0.000%`，`std = 0.000%`
  - `avg_mean_fk_total_delta_pct_mean = +0.000%`，`std = 0.000%`
  - `avg_trial_p2_hybrid_eval_cases_mean = 0.000`，`std = 0.000`
  - `avg_trial_p2_hybrid_applied_cases_mean = 0.000`，`std = 0.000`
  - `avg_trial_p2_hybrid_fallback_cases_mean = 0.000`，`std = 0.000`
- 决策：`go_O5_5_2_N10`。
- 解释：骨架阶段统计在 N=5 下稳定，且无异常计数；建议按计划扩展至 N=10，完成 O5.5-2 稳定性验证收口。

**补充执行结果（run_06..10，2026-03-20）**
- 已按相同 O5.7 参数完成第二组 N=5 重复（run_06..10），新增产物：
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_baseline_run_06..10.log`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_baseline_run_06..10.err.log`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_trial_run_06..10.log`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_trial_run_06..10.err.log`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_compare_run_06..10.csv`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_repeat_summary_02.csv`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_repeat_recommendation_02.txt`
- 第二组关键汇总（5 runs，`B_dedup_adaptive_proto`，按每 run 的宽度平均再聚合）：
  - `avg_mean_ms_delta_pct_mean = -0.338%`，`std = 0.932%`
  - `avg_mean_fk_p2_delta_pct_mean = +0.000%`，`std = 0.000%`
  - `avg_mean_fk_total_delta_pct_mean = +0.000%`，`std = 0.000%`
  - `avg_trial_p2_hybrid_eval_cases_mean = 0.000`，`std = 0.000`
  - `avg_trial_p2_hybrid_applied_cases_mean = 0.000`，`std = 0.000`
  - `avg_trial_p2_hybrid_fallback_cases_mean = 0.000`，`std = 0.000`
- 决策保持：`go_O5_5_2_N10`。
