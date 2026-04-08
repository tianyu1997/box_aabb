# 解析解 Endpoint iAABB 集成计划

> **目标**：将已有的 `derive_aabb_critical_analytical()` 六阶段解析求解器集成到
> `EndpointSource::Analytical` 路径中，使 Analytical 在 Stage 1 阶段直接产出分析
> 精确的 endpoint iAABBs，而非退化为 CritSample 的 kπ/2 边界枚举。

---

## 0. 现状分析

### 0.1 Analytical 解析求解器已有能力

`analytical_solve.h/.cpp` 实现了完整的六阶段梯度零点枚举管线：

| 阶段 | 方法 | 说明 |
|------|------|------|
| **P0** | kπ/2 顶点枚举 | `crit_angles()` + `build_csets()` + `crit_enum_fk()` |
| **P1 / Phase B** | 1D 边求解（atan2） | 固定其余关节，对自由关节 j 求 ∂f/∂q_j = 0 → `q* = atan2(β, α)` |
| **P2 / Phase C** | 2D 面求解（8次多项式） | 对耦合关节对 (i,j)，半角代换后建立 8 次多项式 `solve_poly_in_interval()` |
| **P2.5 / Phase D** | 对约束求解（1D + 2D） | 约束 `q_i + q_j = kπ/2` 下降至 1D 或 2D 问题 |
| **P3 / Phase F** | 内部求解（多起点坐标下降） | 改进的坐标扫描 + pair-coupling 扰动 |

每阶段均支持"**直接系数提取**"（Phase B/C/D/F），通过 DH 链矩阵乘法直接获得三角
多项式系数，无需 FK 采样 + QR 拟合，精确且高效。

### 0.2 当前 `EndpointSource::Analytical` 的问题

`endpoint_source.cpp` 中 Analytical case 仅调用 `derive_crit_endpoints(boundary_only())`，
等价于 CritSample（纯 kπ/2 枚举），**完全未使用**已有的六阶段解析求解器。

原因：`derive_aabb_critical_analytical()` 的输出格式为 **per-link sub-segment AABBs**
`[(n_active × n_sub) × 6]`，与 endpoint pipeline 要求的 **per-endpoint iAABBs**
`[n_endpoints × 6]` 不匹配。

### 0.3 数据流对比

```
已有 Analytical 管线（独立使用）:
  intervals → derive_aabb_critical_analytical()
    → Phase 0..3+ → LinkExtremes[ci][sub]
    → out_aabb[(n_active × n_sub) × 6]      # per-link, with radius, sub-segmented
    → out_endpoint_aabb[(n_active × (n_sub+1)) × 6]  # per-link-per-endpoint

Endpoint Pipeline（模块化，当前目标）:
  intervals → compute_endpoint_iaabb()
    → endpoint_iaabbs[n_endpoints × 6]     # per-endpoint, no radius, no sub-segment
    → extract_link_iaabbs() → per-link iAABBs（union + radius）
```

---

## 1. 集成方案

### 方案选择：**Endpoint-Level 适配（推荐）**

在 `endpoint_source.cpp` 的 `EndpointSource::Analytical` case 中调用
`derive_aabb_critical_analytical()`，然后将其 per-link per-endpoint 输出
重映射为标准 `endpoint_iaabbs[n_endpoints × 6]` 格式。

#### 为什么不绕过 endpoint pipeline？
保持模块化两阶段架构（Stage 1 endpoint → Stage 2 envelope）的一致性，
使 Analytical 可与任何 EnvelopeType（Hull-16、AABB 等）组合。

### 1.1 输出格式映射

```
Analytical 输出:
  out_endpoint_aabb[(n_active × (n_sub+1)) × 6]
    → 对 n_sub=1: 每 link ci 有 2 个 endpoint: proximal(ci) 和 distal(ci)
    → 布局: [ci=0 prox, ci=0 dist, ci=1 prox, ci=1 dist, ...]

Pipeline 需要:
  endpoint_iaabbs[n_endpoints × 6]
    → n_endpoints = n_joints + has_tool
    → endpoint k 对应 frame k+1（prefix[0] = base）
```

**关键映射**：
- endpoint k = frame k+1
- active link ci 的 proximal endpoint = alm[ci] 对应的 parent frame
- active link ci 的 distal endpoint   = alm[ci] 对应的 child frame
- 对于 n_sub=1：`out_endpoint_aabb[ci*2 + 0]` = proximal，`out_endpoint_aabb[ci*2 + 1]` = distal

需要一个重映射函数，将 per-link per-endpoint 的 AABB 聚合到 per-endpoint 格式。
由于同一个 endpoint 可能是多个 active link 的 proximal/distal，取所有引用的交集
（每个方向取 min-of-hi / max-of-lo？不对——应取 union = max-of-hi / min-of-lo，
因为每个 link 独立产出一个 AABB 估计，真值应被所有估计覆盖）。

### 1.2 实现步骤

#### Step 1: 新增转换函数

在 `endpoint_source.cpp` 中新增辅助函数：

```cpp
/// 将 derive_aabb_critical_analytical 的 out_endpoint_aabb 输出
/// 重映射为标准 endpoint_iaabbs 格式。
///
/// analytical_ep: [(n_active × 2) × 6]  (n_sub=1, proximal+distal per link)
/// out:           [n_endpoints × 6]
static void remap_analytical_to_endpoint_iaabbs(
    const float* analytical_ep,
    const Robot& robot,
    float* out)
{
    const int n_endpoints = robot.n_joints() + (robot.has_tool() ? 1 : 0);
    const int n_act = robot.n_active_links();
    const int* alm = robot.active_link_map();

    // Initialize to empty (lo = +inf, hi = -inf)
    for (int k = 0; k < n_endpoints; ++k) {
        out[k*6+0] = out[k*6+1] = out[k*6+2] =  1e30f;
        out[k*6+3] = out[k*6+4] = out[k*6+5] = -1e30f;
    }

    for (int ci = 0; ci < n_act; ++ci) {
        int li = alm[ci];   // link index
        // proximal endpoint = li, distal = li+1
        int ep_prox = li;
        int ep_dist = li + 1;

        const float* prox = &analytical_ep[(ci*2 + 0)*6];
        const float* dist = &analytical_ep[(ci*2 + 1)*6];

        // Union (expand): take min of lo, max of hi
        for (int d = 0; d < 3; ++d) {
            out[ep_prox*6+d]   = std::min(out[ep_prox*6+d],   prox[d]);
            out[ep_prox*6+3+d] = std::max(out[ep_prox*6+3+d], prox[3+d]);
            out[ep_dist*6+d]   = std::min(out[ep_dist*6+d],   dist[d]);
            out[ep_dist*6+3+d] = std::max(out[ep_dist*6+3+d], dist[3+d]);
        }
    }
}
```

#### Step 2: 修改 Analytical Case

```cpp
case EndpointSource::Analytical: {
    result.fk_state = compute_fk_full(robot, intervals);
    result.endpoint_iaabbs.resize(result.n_endpoints * 6);

    // 使用完整解析求解器
    const AnalyticalCriticalConfig acfg = config.analytical_config_ptr
        ? *config.analytical_config_ptr
        : AnalyticalCriticalConfig::all_enabled();

    const int n_sub = 1;  // endpoint-level: 不需要 sub-segment
    const int n_act = robot.n_active_links();

    // per-link AABBs (unused but required by API)
    std::vector<float> link_aabbs(n_act * n_sub * 6);
    // per-link per-endpoint AABBs (what we need)
    std::vector<float> ep_aabbs(n_act * (n_sub + 1) * 6);

    AnalyticalCriticalStats astats{};
    derive_aabb_critical_analytical(
        robot, intervals, n_sub, acfg,
        link_aabbs.data(), &astats,
        ep_aabbs.data());

    // 重映射到标准 endpoint_iaabbs 格式
    remap_analytical_to_endpoint_iaabbs(
        ep_aabbs.data(), robot,
        result.endpoint_iaabbs.data());

    result.n_evaluations = astats.n_phase0_vertices
                         + astats.n_phase1_edges
                         + astats.n_phase2_faces
                         + astats.n_phase25a_pair1d
                         + astats.n_phase25b_pair2d
                         + astats.n_phase3_interior;
    break;
}
```

#### Step 3: 处理增量路径

`compute_endpoint_iaabb_incremental()` 中 Analytical case 同样调用完整解析求解器
（无增量优化——留作后续 Phase 2 开发）。

#### Step 4: 增加必要的 #include

```cpp
#include "sbf/envelope/analytical_solve.h"
```

---

## 2. 验证计划

### 2.1 正确性验证

1. **Analytical ⊆ IFK**: 对随机区间，Analytical endpoint iAABBs 必须被 IFK 结果
   包含（Analytical 更紧，但不能超出 IFK 的保守边界）。

2. **Analytical ⊆ CritSample/GCPC**: 解析结果应 ≤ CritSample/GCPC（六阶段求解器
   包含 P0 的 kπ/2 枚举 + 更多临界点）。

3. **蒙特卡洛**: 在区间内密集采样 10K 配置，计算 FK，验证所有采样点均在 Analytical
   endpoint iAABBs 内。

### 2.2 性能基准

- 与 IFK / CritSample / GCPC 的耗时对比
- 期望：Analytical 耗时高于 GCPC（因为有 P1–P3 在线求解），但包络体积最小

### 2.3 端到端集成

- 确认 `extract_link_iaabbs()` + EnvelopeType 管线正常工作
- 验证 LECT 构建、碰撞检测结果一致

---

## 3. 质量排序更新

当前：`IFK(0) < CritSample(1) < Analytical(2) < GCPC(3)`

集成后 Analytical 使用完整六阶段解析器，理论上是最紧的方法：

**建议更新**：`IFK(0) < CritSample(1) < GCPC(2) < Analytical(3)`

这需要修改 `endpoint_source.h` 中的 `endpoint_source_quality()` 函数。

---

## 4. 后续优化方向

### 4.1 增量解析求解

当前增量路径直接调用完整解析器。优化方向：
- 仅重新求解受 `changed_dim` 影响的阶段
- P1 中仅需重算涉及 `changed_dim` 的 edge
- P2 中仅需重算包含 `changed_dim` 的面对

### 4.2 AA 剪枝与解析器协同

利用增量 iFK state 提供的 AA bounds 预过滤，跳过已被覆盖的 link/axis。

### 4.3 直接 Endpoint iAABB 输出

修改 `derive_aabb_critical_analytical()` 内部的 `LinkExtremes` 跟踪器，
直接输出 per-endpoint 格式，避免 post-hoc 重映射开销。

---

## 5. 时间估算

| 步骤 | 预估工时 |
|------|----------|
| Step 1-2: 转换函数 + Analytical case 重写 | 1h |
| Step 3: 增量路径 | 0.5h |
| Step 4: include 与构建修复 | 0.5h |
| 验证 + 测试 | 2h |
| 质量排序更新 + 文档 | 0.5h |
| **合计** | **~4.5h** |

---

## 6. 文件修改清单

| 文件 | 修改类型 | 说明 |
|------|----------|------|
| `src/envelope/endpoint_source.cpp` | 改 | Analytical case 调用 `derive_aabb_critical_analytical()` + 重映射 |
| `include/sbf/envelope/endpoint_source.h` | 改 | 质量排序：Analytical ↔ GCPC 互换 |
| `experiments/exp_ep_iaabb_bench.cpp` | 改 | Analytical 配置使用 `analytical_config_ptr` |
| `experiments/exp_small_ep_viz.cpp` | 改 | 同上 |
| `tests/test_envelope_pipeline.cpp` | 增 | 新增 Analytical 正确性验证用例 |
| `doc/CHANGE_LOG_CN.md` | 增 | 记录变更 |
