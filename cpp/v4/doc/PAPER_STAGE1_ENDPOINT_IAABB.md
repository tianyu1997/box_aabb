# SafeBoxForest v4 — Stage 1 端点 iAABB 流水线

> 本文档描述 SafeBoxForest 的 Stage 1（Endpoint iAABB Generation）算法架构。
> 对应代码模块：`envelope/endpoint_source`, `envelope/crit_sample`, `envelope/gcpc`, `envelope/analytical_solve`。
> 可直接嵌入论文 Section III 作为算法描述基础。

---

## 1. 流水线总体架构

SafeBoxForest 的包络计算采用两阶段模块化流水线：

```
Stage 1 (Endpoint Source):  C-space intervals → endpoint_iaabbs [n_endpoints × 6]
Stage 2 (Envelope Type):    endpoint_iaabbs → LinkEnvelope
```

Stage 1 提供 4 条端点源路径，按精度从低到高排列：

| 路径 | 质量 | 安全类 | 核心算法 | 典型 FK 调用数 | 典型耗时 |
|------|------|--------|---------|--------------|----------|
| iFK | 0 | SAFE | 区间前向运动学 | 1 次区间 FK | ~0.02 ms |
| CritSample | 1 | UNSAFE | GCPC 缓存 + 边界 kπ/2 枚举 | ~10²–10³ | ~0.04 ms |
| Analytical | 2 | SAFE | 解析梯度零点 P0–P3（边界+内部） | ~10³–10⁴ | ~9 ms |
| GCPC | 2 | SAFE | 解析边界 P0–P2.5 + GCPC 缓存内部 | ~10³–10⁴ | ~7 ms |

> **安全类**：SAFE 表示产出的 iAABB 保证 ⊇ 真实可达集（保守外界）；
> UNSAFE（CritSample）通过采样逼近，可能遗漏极值点导致 iAABB 略小于真实，
> 但计算速度最快。

**最新基线 Benchmark**（iiwa14, 50 trials, Release, seed 42，含 S4+S5+S1+S9 优化）：

| 管线 | 耗时 (ms) | 体积 | VolRatio vs IFK | FK evals |
|------|-----------|------|-----------------|----------|
| IFK | 0.022 | 2.512 | 1.0000 | 0 |
| CritSample | 0.040 | 0.394 | 0.157 | 511 |
| Analytical | 9.322 | 0.412 | 0.164 | 6948 |
| GCPC | 7.114 | 0.411 | 0.164 | 3343 |

**Per-width breakdown**（仅 Analytical / GCPC）：

| 管线 | narrow (ms) | medium (ms) | wide (ms) |
|------|------------|-------------|----------|
| Analytical | 3.889 | 9.405 | 20.021 |
| GCPC | 1.991 | 7.011 | 17.564 |

> **CritSample 体积更小的原因**：CritSample (UNSAFE) 因采样遗漏导致 iAABB 偏小 ~4%，
> 并非更精确，而是不完备（under-approximation）。Analytical 和 GCPC (SAFE) 的体积几乎
> 一致，说明 GCPC 用缓存替代 P3 坐标下降不损失覆盖质量。
>
> **已应用的速度优化**：S4（skip shallow links）、S5（自适应多项式子区间）、
> S1（P1 suffix 缓存）、S9（ConfigVec 堆分配消除）。
> 相比原始基线 Analytical 9.535→9.322ms（-2.2%），GCPC 7.407→7.114ms（-4.0%）。

所有路径产出统一的中间表示 `endpoint_iaabbs`，即每个端点（关节/工具尖端）在当前 C-space 区间内的位置区间 AABB。

---

## 2. iFK 路径（区间前向运动学）

**算法**：对 DH 参数链执行区间算术，每个变换矩阵元素以 `[lo, hi]` 区间传播。

**优势**：单次前向传播即得到所有端点的保守外界，速度最快。

**劣势**：区间算术的依赖性问题（dependency problem）导致结果过于保守，尤其对宽区间。

**复杂度**：$O(n)$，其中 $n$ 为关节数。

---

## 3. CritSample 路径（GCPC-based 临界采样）

### 3.1 设计理念

CritSample 在 v4 中采用全新设计。核心思想：实际 FK 采样配置的位置必定是 iFK 外界的内点，因此通过密集采样临界配置可以逼近真实包络。

v4 设计分为两个阶段：

### 3.2 Phase 1 — GCPC 内部极值点采样

利用预计算的 GCPC 缓存，查询当前区间内的所有已知内部极值点：

$$\text{Phase 1}: \quad \forall l \in \text{active\_links}, \quad \mathcal{P}_l = \text{GcpcCache.query\_link}(l, [\mathbf{q}^{lo}, \mathbf{q}^{hi}])$$

对每个匹配的临界点 $p \in \mathcal{P}_l$：
1. 重建 q₀（通过 atan2 解析重建）
2. 还原 q₁ 镜像（若 q₁ 经过 period-π 约减）
3. 构建完整关节配置 $\mathbf{q}_{\text{full}}$
4. 计算 FK 并扩展 endpoint iAABBs
5. 评估 q₀ 的对向和 ±π/2 变体

### 3.3 Phase 2 — 边界 + kπ/2 枚举

对每个关节 $j$，构建临界角集合：

$$C_j = \{q_j^{lo},\ q_j^{hi}\} \cup \{k\frac{\pi}{2} \mid k\frac{\pi}{2} \in (q_j^{lo}, q_j^{hi}),\ k \in \mathbb{Z}\}$$

构建笛卡尔积 $C_1 \times C_2 \times \cdots \times C_n$（受上限 `max_boundary_combos` 控制），对每个配置执行增量 FK 并更新 endpoint iAABBs。

**增量 FK 优化**：`crit_enum_fk()` 利用 DH 链的前缀乘积缓存，仅重新计算变化关节之后的帧，将每次 FK 复杂度从 $O(n)$ 降至 $O(n - j_{\text{changed}})$。

### 3.4 时间复杂度

$$T_{\text{CritSample}} = O(\underbrace{|\mathcal{P}| \cdot FK}_{\text{Phase 1}}) + O(\underbrace{\prod_{j=1}^{n} |C_j| \cdot FK_{\text{inc}}}_{\text{Phase 2}})$$

---

## 4. Analytical 路径（解析梯度零点枚举）

### 4.1 核心思想

对于 DH 参数化的串联机械臂，端点位置分量 $p_d(\mathbf{q})$ 可以表示为关节角的三角多项式。其对单个/成对自由关节的偏导数零点可以解析求解。

Analytical 路径是纯在线求解，不依赖预计算缓存。所有 endpoint iAABB 初始化为空集 $[+\infty, -\infty]$，由解析结果 union 展开，不使用 iFK 兜底。

### 4.2 六阶段求解

| 阶段 | 自由度 | 方法 | 输出 |
|------|--------|------|------|
| P0 — 顶点 | 0 | kπ/2 边界笛卡尔积枚举 + FK | 区间顶点的 FK 值（建立初始 baseline）|
| P1 — 边 | 1 | $\frac{\partial p_d}{\partial q_j} = 0$，DH 直接系数提取 + atan2 | 边上极值 |
| P2 — 面 | 2 | $\nabla_{q_i, q_j} p_d = 0$，DH 直接 2D 系数提取 + degree-8 多项式求根 | 面上极值（所有 $C(n,2)$ 关节对）|
| P2.5 — 成对约束 | 1-2 | $q_i + q_j = k\frac{\pi}{2}$ 降维 + DH 直接系数 + atan2/poly8 | 耦合极值 |
| P3 — 内部 | $n$ | 改进坐标下降（多起点+pair coupling）| 内部极值（启发式，不保证完备）|
| AA 剪枝 | — | 仿射算术（Affine Arithmetic）逐阶段刷新 | 跳过已收敛的 link/axis |

### 4.3 DH 系数直接提取（零 FK 优化）

v4 实现了 DH 链直接系数提取，绕过传统的多点 FK 采样 + QR 拟合：

**1D 系数**（P1 阶段）：
$$p_d(q_j) = \alpha_d \cos q_j + \beta_d \sin q_j + \gamma_d$$

$\alpha_d, \beta_d, \gamma_d$ 通过 prefix/suffix 矩阵的列向量直接计算（~30 次乘法，0 次 FK 调用）。

**P1 背景组合 suffix 缓存**（S1 优化）：
对给定活跃关节 $j$，背景组合 bg 的位编码中，低位 $0..j{-}1$ 对应 prefix 关节，高位 $j..n{-}2$ 对应 suffix 关节。`compute_suffix_pos()` 仅依赖 $q_{j+1} \ldots q_V$（suffix 关节角），当 bg 递增时 suffix bits (`bg >> j`) 每 $2^j$ 次才变化。因此缓存 suffix 结果，仅在 suffix bits 变化时重新计算，对 $V{=}6, j{=}3$ 可将 suffix 计算从 64 次降至 8 次。

**2D 系数**（P2 阶段）：
$$p_d(q_i, q_j) = \sum_{k=0}^{8} a_{k,d} \cdot \phi_k(c_i, s_i, c_j, s_j)$$

9 个系数通过 prefix × DH_i × middle × DH_j × suffix 矩阵链直接计算（~120 次乘法，0 次 FK 调用）。

### 4.4 AA 剪枝

轴向可改进性（AA, Axis-Achievability）剪枝：

对每个连杆 $l$ 和轴 $d$，若当前估计区间已经与某个更便宜的外界（iFK）一致，则跳过后续阶段对该连杆/轴的求解。剪枝在每个阶段开始时动态刷新。

---

## 5. GCPC 路径（解析边界 + 全局临界点缓存内部）

### 5.1 设计理念

GCPC 路径是 Analytical 路径的缓存加速变体：
- **边界求解**：与 Analytical 完全一致（P0 + P1 + P2 + P2.5 + AA 剪枝），`enable_interior_solve=false`
- **内部求解**：不使用 P3 坐标下降，改为 GCPC 缓存查找内部临界点

这样确保边界完备性与 Analytical 一致，同时利用缓存加速内部搜索（缓存是预计算的全局临界点集合，覆盖面通常优于启发式坐标下降）。

### 5.2 预计算阶段

通过坐标下降在机械臂全关节范围内搜索所有 FK 位置分量的驻点，存储到 KD-tree 中。

**对称性约减**：
- **q₀ 消除**：径向分量 $R = \sqrt{p_x^2 + p_y^2}$ 与 $p_z$ 不依赖 q₀，因此 q₀ 不存储；查询时通过 $q_0 = \text{atan2}(-B, A)$ 重建。
- **q₁ 周期-π**：由于 $R(q_1) = R(q_1 + \pi)$，仅存储 $q_1 \in [0, \pi]$；查询时镜像得到 $q_1 - \pi$ 的候选。

### 5.3 查询流水线

```
Step 1 — Analytical 边界求解（与 Analytical 路径一致）
  P0:   kπ/2 枚举
  P1:   1D atan2 边求解 + DH 直接系数 + AA 剪枝
  P2:   2D degree-8 面求解 + DH 直接系数 + AA 剪枝（所有 C(n,2) 对）
  P2.5: 成对约束 1D/2D + DH 直接系数 + AA 剪枝

Step 2 — GCPC 缓存内部查找
  KD-tree 区间查询 → q₀/q₁ 重建 → 去重 → FK → 展开 endpoint iAABBs
```

Analytical 边界阶段提供的 baseline 使 AA 剪枝可以跳过已收敛的 link/axis，
GCPC 缓存查找替代 P3 坐标下降，速度更快且覆盖面更广。

---

## 6. 统一输出格式

所有 4 条路径产出统一的 `EndpointIAABBResult`：

```
endpoint_iaabbs: float[n_endpoints × 6]
  其中 6 = {lo_x, lo_y, lo_z, hi_x, hi_y, hi_z}
  n_endpoints = n_joints + has_tool
```

Stage 2（`extract_link_iaabbs`）通过 `union(endpoint[parent], endpoint[child]) + radius` 将端点 iAABBs 转换为连杆 iAABBs。

---

## 7. 安全类、质量排序与缓存兼容性

```
安全类:
  SAFE:    iFK(q=0) ≤ Analytical(q=2) ≤ GCPC(q=3)     — 保守外界保证
  UNSAFE:  CritSample(q=1)                              — 采样逼近，可能遗漏极值

source_can_serve(cached, requested):
  同安全类内: quality(cached) ≥ quality(requested) → 允许替代
  SAFE → UNSAFE:  允许（保守外界是更宽但合法的替代）
  UNSAFE → SAFE:  禁止（CritSample 不可作为安全证书）
```

GCPC 质量严格高于 Analytical：边界求解一致，内部使用全局缓存覆盖面优于 P3 启发式坐标下降。
CritSample 因采样性质置于 UNSAFE 类，不可替代 SAFE 请求。

---

## 8. 测试覆盖

| 测试文件 | 测试数 | 覆盖模块 |
|---------|--------|---------|
| `test_ifk_pipeline` | 172 | 区间数学、iFK、endpoint_source (IFK)、LECT、refine |
| `test_analytical` | 38 | crit_angles、FKWorkspace、Analytical 各阶段、DH 系数提取、AA 剪枝 |
| `test_crit_gcpc` | 370 | CritSample 两阶段、Config 工厂、iFK 包含性、Dispatch、GCPC 构建/查询/充实/AABB |
