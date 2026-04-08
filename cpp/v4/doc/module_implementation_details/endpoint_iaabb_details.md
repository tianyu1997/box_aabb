# Endpoint iAABB 模块实现细节

## 1. 模块定位

Endpoint iAABB 模块是两阶段包络流水线中的 Stage 1。
其职责是把 C 空间关节区间转换为笛卡尔空间的端点区间包围盒（endpoint interval AABB），并以统一 paired 结构输出。

主要接口：
- compute_endpoint_iaabb(...)
- compute_endpoint_iaabb_incremental(...)

核心文件：
- include/sbf/envelope/endpoint_source.h
- src/envelope/endpoint_source.cpp

## 2. 核心类型与内存布局

### 2.1 EndpointSource

当前支持 4 条来源路径：
- IFK
- CritSample
- Analytical
- GCPC

### 2.2 安全类别与替代规则

- SAFE：IFK、Analytical、GCPC
- UNSAFE：CritSample

替代规则要点：
- SAFE 结果可用于 UNSAFE 请求（更保守但有效）
- UNSAFE 结果不能用于 SAFE 请求
- 同一安全类别内按质量顺序比较是否可复用

该规则直接用于 LECT 的缓存复用判定。

### 2.3 EndpointIAABBResult

关键字段：
- endpoint_iaabbs：定长缓冲区
- n_active_ep：有效端点数量（= n_active * 2）
- n_active：有效连杆数量
- fk_state：FK 状态（增量路径与回退派生会用到）
- n_evaluations：评估计数（诊断）

数据布局：
- endpoint_iaabbs 逻辑形状为 n_active * 2 * 6
- 每个 6 元组为 lo_x, lo_y, lo_z, hi_x, hi_y, hi_z
- 对第 ci 条 active link：
  - ci*2 为 proximal 端点区间
  - ci*2+1 为 distal 端点区间

辅助接口：
- has_endpoint_iaabbs()
- endpoint_iaabb_len()
- endpoint_data()

定长布局的关键收益是消除热路径上的小对象堆分配。

## 3. 全量计算流程

### 3.1 IFK 路径

1. compute_fk_full(robot, intervals)
2. fk_to_endpoints(...) 抽取端点区间
3. 写入 paired endpoint 输出

特点：
- 速度最快
- 保守但通常更松

### 3.2 CritSample 路径

1. 先计算 compute_fk_full（保留 FK 状态）
2. derive_crit_endpoints 基于关键采样更新端点区间
3. n_evaluations 记录采样评估数

特点：
- 常可得到更紧结果
- 但属于 UNSAFE（可能漏掉真实极值）

### 3.3 Analytical 路径

Analytical 路径通过精确枚举端点位置函数的所有临界点，得到比 IFK 更紧的 SAFE 区间包围盒。
核心入口为 `derive_aabb_critical_analytical()`，内部执行一条多阶段临界点搜索管线，最终将结果直接写入 paired 格式输出缓冲。

#### 步骤 1：compute_fk_full — 初始化 FK 状态

- 对整个关节区间盒执行完整正向运动学，建立基础包络 `fk_state`。
- FK 状态在后续增量路径（`compute_endpoint_iaabb_incremental`）中可复用。

#### 步骤 2：构建 AnalyticalCriticalConfig

- 若调用者未显式传入配置指针，则默认使用 `AnalyticalCriticalConfig::all_enabled()`，开启全部 6 个阶段。
- 每个阶段均可独立关闭，便于调试与性能对比（见 `analytical_solve.h` 的 factory methods）。

#### 步骤 3：Phase 0 — kπ/2 顶点枚举（基线 AABB）

**目的**：以低代价建立初始包络，覆盖所有关节角为 kπ/2 处的端点位置。

**原理**：
- 机器人端点位置是关节角的三角函数组合，其最值候选集包含所有「关键角度」（kπ/2）处的组合点。
- 对每个关节 j，收集区间 [lo_j, hi_j] 内的所有 kπ/2 角度，构成候选集 `per_joint[j]`。
- 通过 `build_csets` + `crit_enum_fk` 递归枚举全部关节角组合，依次对每条 active link 的 proximal/distal 端点更新 `link_seg_ext[ci]`（极值追踪结构）及 `out_endpoint_aabb`。
- 评估数记入 `stats.n_phase0_vertices`。

**输出**：`link_seg_ext` 已保存各端点 6 个极值面方向的最优关节角配置，`out_endpoint_aabb` 已填充初始包络。

#### 步骤 4：AA gap 剪枝（Phase 0 后）

- 调用 `aa_ws.compute(robot, intervals)` 构建区间算术（AA）前缀变换链。
- `refresh_phase_skip_links_from_aa` 对每条 link：比较 AA 推导出的可达边界是否能改进当前已知包络。若 AA 证明无法在任何轴上改进，则标记为 `skip_link[ci] = true`，后续各阶段直接跳过该 link。
- 每个新阶段开始前均重新调用此过程，随着包络不断收紧，被跳过的 link 数量递增。

#### 步骤 5：Phase 1 — 1D 边临界点求解（atan2 解析解）

**目的**：对关节盒的每条「边」（固定 n-1 个关节为边界，仅 1 个关节自由变化），精确求解使端点坐标取极值的关节角。

**原理**：
端点坐标沿第 j 个关节的分量可写成：

$$f(q_j) = \alpha \cdot \cos(q_j) + \beta \cdot \sin(q_j) + \gamma$$

极值条件 $f'(q_j) = 0$ 的解析解为：

$$q_j^* = \text{atan2}(\beta,\, \alpha), \quad q_j^* + \pi$$

**Phase B 直接系数提取**（`enable_p1_direct_coeff = true`）：
- 传统做法：在 3 个采样点计算 FK，再通过 3×3 QR 分解拟合 α、β、γ。
- Phase B 优化：直接从 DH 链的前缀变换矩阵（prefix）、关节 j 的 DH 参数，以及后缀位置（suffix_pos）解析提取 α、β，完全绕过采样 + QR，大幅减少 FK 调用。

**背景枚举（bg loop）**：
- 其余 n-1 个关节遍历 {lo, hi}² 组合，形成 $2^{n-1}$ 个背景配置（上限 128）。
- 对每种背景配置，分别为 proximal（V）和 distal（V+1）端点提取系数，各得 atan2 候选角，投影回区间并评估。

**后缀缓存（S1 优化）**：
- suffix_pos 仅与 j 之后的关节有关；当背景配置中后缀部分的 bit 不变时，复用上次计算结果，减少重复链式 FK。

评估数记入 `stats.n_phase1_edges` 和 `stats.n_phase1_fk_calls`。

#### 步骤 6：dual_phase3 检查点

- 若 `dual_phase3 = true` 且 `enable_interior_solve = true`，在 Phase 1 结束后快照当前 `link_seg_ext`，作为 Phase 3+ 的第一个起点集（覆盖仅 Phase 0+1 的包络质量）。
- 第二个起点集来自 Phase 2/2.5 完成后的包络，两套起点分别运行 Phase 3+，提升内部极值的覆盖完整性。

#### 步骤 7：Phase 2 — 2D 面临界点求解（8 次多项式）

**目的**：对关节盒每个「面」（固定 n-2 个关节，2 个关节自由），求解 2D 临界点。

**原理**：
端点坐标在两个自由关节 $(q_i, q_j)$ 的展开式中涉及 $\cos q_i, \sin q_i, \cos q_j, \sin q_j$ 的双线性项，共 8 个系数：

$$f(q_i, q_j) = a_1 c_i c_j + a_2 c_i s_j + a_3 s_i c_j + a_4 s_i s_j + a_5 c_i + a_6 s_i + a_7 c_j + a_8 s_j + \text{const}$$

临界条件 $\partial f/\partial q_i = 0,\; \partial f/\partial q_j = 0$ 联立，通过半角代换 $t_j = \tan(q_j/2)$ 消去 $q_j$，化为关于 $t_j$ 的 8 次多项式，再对每个根反解 $q_i$。

**Phase C 直接系数提取**（`enable_p2_direct_coeff = true`）：
- 直接从 DH 链提取 8 个系数矩阵，绕过 9 点采样 + QR 分解。

**适用范围**：
- 默认 `face_all_pairs = true`，枚举所有 C(n, 2) 关节对，确保完整性；关闭后仅枚举相邻对和机器人声明的耦合对。

评估数记入 `stats.n_phase2_faces`。

#### 步骤 8：Phase 2.5a — 对约束 1D 求解

**目的**：处理满足线性约束 $q_i + q_j = k\pi/2$ 的关节对，在约束超平面上精确求 1D 极值。

**原理**：
- 枚举区间内所有满足 $q_i + q_j \in \{k\pi/2\}$ 的整数 k，对每个 k 确定 $q_i$ 的有效范围 $[\text{eff\_lo}, \text{eff\_hi}]$。
- 代入约束后，位置函数退化为仅关于 $q_i$ 的三角函数，用 atan2 解析求解。
- **Phase D 优化**：直接从 2D 系数 + 约束值 $C = k\pi/2$ 的三角值（$\cos C, \sin C$）代数推导 α、β，无需采样。

评估数记入 `stats.n_phase25a_pair1d`。

#### 步骤 9：Phase 2.5b — 对约束 2D 求解

**目的**：在约束 $q_i + q_j = C$ 下，再引入第三个自由关节 $q_m$，求 (qi, qm) 二维临界点。

**原理**：
- 仍利用 8 次多项式对 $q_m$ 求根，再对每个根反解 $q_i$（已通过约束确定 $q_j$）。
- **Kronecker 分解（H6 优化）**：将 9×9 coefficient matrix 分解为两个 3×3 矩阵的 Kronecker 积，用两次廉价矩阵求逆取代高代价 QR，降低系数提取成本。

评估数记入 `stats.n_phase25b_pair2d`。

#### 步骤 10：Phase 3+ — 改进内部求解（多起点 + 对耦合坐标下降）

**目的**：处理高维关节空间的纯内部极值（边界搜索遗漏的内点），采用多起点坐标下降策略。

**流程**：
1. 对每条 link 的每个极值面方向（共 6 个），从多个初始配置（`n_restarts`）出发运行坐标下降。
2. 起点来自：Phase 0+1 快照（若 `dual_phase3`）、Phase 2/2.5 结束后的当前最优配置。
3. 每轮坐标下降对所有自由关节逐一执行 1D atan2 解析求解（Phase F 直接系数提取），迭代直到无改进或达到 `max_sweeps`。
4. **对耦合（pair coupling）**：在坐标下降中穿插对约束搜索（类 Phase 2.5a），提升在耦合关节方向上的逃脱能力。

适用条件：`nj >= 3` 且 `nj <= interior_max_free`（默认 7）。  
评估数记入 `stats.n_phase3_interior`。

#### 步骤 11：拷贝至输出缓冲

```cpp
std::memcpy(result.endpoint_data(), ep_aabbs.data(),
            result.n_active_ep * 6 * sizeof(float));
```

`derive_aabb_critical_analytical` 的输出 `ep_aabbs` 布局已经是 `[n_act × 2 × 6]`（proximal/distal paired），与 `EndpointIAABBResult::endpoint_iaabbs` 精确对齐，直接 memcpy 写入，无需格式转换。

`n_evaluations` 汇总所有阶段的评估计数之和（Phase 0–3）。

#### 特点总结

| 维度 | 说明 |
|---|---|
| 安全类别 | SAFE（解析保证覆盖所有临界点；含 interior 时不会漏极值） |
| 紧度 | 通常优于 IFK，GCPC 为其不含 interior solve 的变体 |
| 性能瓶颈 | Phase 2（C(n,2) 对数 × 背景 × 8 次多项式）；>6 关节时 Phase 3+ 主导 |
| 可调选项 | 各阶段均可独立关闭；`face_all_pairs`、`dual_phase3`、`interior_max_free` 是主要精度旋钮 |
| v4 优化 | AA gap 剪枝、直接系数提取（Phase B/C/D/F）、Kronecker 分解（H6）、按 link 并行 |

### 3.4 GCPC 路径

1. compute_fk_full
2. 运行 analytical 边界阶段（关闭 interior solve）
3. 边界结果写入 paired 输出
4. 使用 gcpc_cache_interior_expand 做缓存 interior 补充

特点：
- SAFE 路径
- 在精度与速度间取得折中

## 4. 增量计算流程

compute_endpoint_iaabb_incremental 先统一计算 incremental FK。
随后按 source 分流：
- IFK：直接从增量 FK 抽取端点区间
- 其他 source：在新子区间盒上执行对应 Stage 1 算法

其中 IFK 增量路径延迟最低，最适合树下降场景。

## 5. 关键内部辅助

### 5.1 fk_to_endpoints

基于 active_link_map，从 FK prefix 变换抽取 paired 端点区间。

### 5.2 update_endpoint_iaabb_from_ws_paired

把标量 FK workspace 点评估结果并入 paired endpoint 包络。
主要用于 GCPC interior cache 扩展。

### 5.3 GCPC 去重键

GCPC interior 扩展会离散化关节向量并去重，减少重复 FK 评估。

## 6. 与 Stage 2 的契约

Stage 1 仅输出几何端点区间，不做半径膨胀。
连杆半径膨胀与 link 级包络由 Stage 2 完成。

## 7. 性能要点

- 定长 endpoint 缓冲减少分配抖动
- IFK 路径结合增量 FK，数据搬运最小
- GCPC 通过 boundary + cache interior 避免 full interior solve

## 8. 边界行为

- 若 endpoint 输出不可用但 fk_state 有效，下游可回退到 FK 派生 link AABB
- CritSample 安全级别限制可防止其被误当作 SAFE 证书复用

## 9. 扩展建议

- 新增 source 时同步扩展：枚举、质量顺序、安全类别、全量入口、增量入口

### 9.1 显式替代矩阵（建议作为实现真值表）

行表示 cached source，列表示 requested source。

| cached \ requested | IFK | CritSample | Analytical | GCPC |
|---|---:|---:|---:|---:|
| IFK | ✅ | ✅ | ❌ | ❌ |
| CritSample | ❌ | ✅ | ❌ | ❌ |
| Analytical | ✅ | ✅ | ✅ | ❌ |
| GCPC | ✅ | ✅ | ✅ | ✅ |

解释：
- SAFE（IFK/Analytical/GCPC）可替代 UNSAFE（CritSample）请求。
- UNSAFE（CritSample）不能替代任何 SAFE 请求。
- SAFE 内部按质量链 IFK <= Analytical <= GCPC 单向替代。

建议：
- 代码层将该矩阵作为 `source_can_serve()` 的唯一判定来源；
- 单元测试按 4x4 全组合断言，防止后续枚举扩展或质量顺序调整时引入缓存语义漂移。
