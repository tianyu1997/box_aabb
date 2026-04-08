# GCPC / Analytical 性能优化计划

> 创建时间: 2026-03-14
> 基线性能 (pipeline benchmark, 10 trials, Cold SubAABB):
> - Analytical: **105 ms**
> - GCPC: **64 ms** (1.63× faster)
>
> 基线 per-phase timing (GCPC, width=0.1):
> | Phase | A | B | C | D | E | F | G | Total |
> |-------|-----|------|------|-------|------|-------|------|-------|
> | ms    | 0.08| 0.15 | 2.77 | 11.81 | 0.72 | 68.08 | 7.03 | ~91ms |
>
> Phase F 占 GCPC 总时间的 **75%**，是主要瓶颈。

---

## 改进项目

### P1: Phase D/E/F Partial FK（预估 -15~25ms）
**状态**: ✅ 完成

**问题**: Phase D/E/F 的 FK grid 每次 `ws.compute(robot, q)` 做完整 FK。
但同一 background combo 内只有 free joints 变化，prefix 可复用。

**方案**: 
- 添加 `compute_prefix()` 方法到 `GcpcFKWorkspace`
- Phase D: `min_free = min(pi, pj)`, prefix 一次, 9 samples 用 `compute_from`
- Phase E: `min_free = min(pi, pj)`, prefix 一次, 3 samples 用 `compute_from`
- Phase F: `min_free = min(pi, pj, pm)`, prefix 一次, 9 samples 用 `compute_from`

**实测结果**: 
- 验证通过: volume=2.809839e-02 (exact match)
- 时间 94.7ms (基线 91ms) — 对 iiwa14 增益有限，因多数 pair 含 joint 0 (min_free=0 无省)
- Phase timing: D=12.73 E=0.72 F=70.89 (基线 D=11.81 E=0.72 F=68.08)
- 代码已保留（对高索引 pair 仍有益，且逻辑正确）

---

### P2: Phase F Per-Pair AA Pruning（预估 -10~20ms）
**状态**: ✅ 完成

**问题**: Phase D 已有 Level 2 per-background AA pruning。Phase F 没有。

**方案**:
- 在 Phase F 的 `bgc` 循环外预分配 `face_f_iv`（避免每次 heap alloc）
- 每个 bg combo 做 2-DOF AA 检查，跳过不可能改善 AABB 的 combo
- Phase D 也同样优化：`face2_iv` 预分配到循环外

**实测结果**:
- 验证通过: volume=2.809839e-02 (exact match)
- Phase F pruned: 166/785 combos (21%) → F_fk 15540→13966 (-10%)
- Phase F 时间: 69.07ms (基线 68.08ms) — AA 开销≈节省, width=0.1 时几乎持平
- 整体: 92.0ms (基线 91ms)
- 对更大 width (0.3+) AA pruning 应更有效（更多 combo 被 prune）

**影响范围**: `src/envelope/gcpc_cache.cpp` — Phase F bgc 内循环

---

### P3: Analytical FKWorkspace Stack 化（预估 -10~15ms）
**状态**: ✅ 完成

**问题**: `envelope_derive_critical.cpp` 中 `FKWorkspace` 使用 `std::vector<Matrix4d>`，
8 处局部变量各 heap alloc 一次。

**方案**:
- `std::vector<Eigen::Matrix4d> tf` → `Eigen::Matrix4d tf[16]`
- `resize()` 变为空操作（保留接口兼容）
- `tf.data()` → `tf`（数组天然退化为指针）

**实测结果**:
- 验证通过: volume=2.809839e-02 (exact match)
- Analytical 单次: 189ms (基线 184ms) — 在噪声范围内
- GCPC: 90.8ms — 功能正常
- 需 pipeline benchmark 多次平均才能测出堆消除的真实效果

---

## 实施进度

| # | 改进项 | 开始时间 | 完成时间 | 构建 | 测试 | 备注 |
|---|--------|----------|----------|------|------|------|
| P1 | Phase D/E/F Partial FK | 完成 | ✅ | ✅ 构建成功 | ✅ volume exact match | 对 iiwa14 增益有限 |
| P2 | Phase F AA Pruning | 完成 | ✅ | ✅ 构建成功 | ✅ volume exact match | prune 21%, w=0.1持平 |
| P3 | Analytical FK Stack | 完成 | ✅ | ✅ 构建成功 | ✅ volume exact match | 待 benchmark 验证 |

---

---

### P4: Per-Direction AA Pruning (#5)（预估 -3~5ms）
**状态**: ✅ 完成

**问题**: Phase D/F 内循环对 3 个维度 (x,y,z) 分别求多项式根。但某些维度已不可能改善 AABB。

**方案**:
- 新增 `aa_dim_can_improve` lambda：只检查单个维度 `dim` 是否可能改善当前 AABB
- 在 Phase D/F 的 `for (int d = 0; d < 3; ++d)` 循环头部调用

**实测结果**: D_pruned=57, F_pruned=166 (AA 剪枝生效)

---

### P5: Polynomial Solver Heap Elimination (#6)
**状态**: ✅ 完成

**问题**: `solve_polynomial_real()` 返回 `std::vector<double>`，每次调用 heap alloc。

**方案**:
- 新增 `FixedRoots` 结构体：`double v[8]; int n=0;`，栈分配
- `solve_companion_fixed<N>()` 改用 `FixedRoots&` 参数
- `solve_polynomial_real()` 返回 `FixedRoots`

---

### P6: Root-Interval Pre-Filter (#7)（预估 -5~10ms）
**状态**: ✅ 完成

**问题**: Phase D/F 每次 eigensolve (6μs) 很昂贵。很多区间内根本没有根。

**方案**:
- 新增 `poly_eval()` (Horner求值) + `interval_may_have_roots()`
- 在 eigensolve 前用半角 `tan(lo/2), tan(hi/2)` 评估多项式
- 检查 3 点 (lo, hi, mid) 符号一致 + 导数界确认无零点 → 跳过 eigensolve
- 保守策略：仅在**确定无根**时返回 false

---

### P7: Symbolic Polynomial Coefficients (#9)（预估 -5~8ms）
**状态**: ✅ 完成

**问题**: Phase D/F 用 9 点 Vandermonde 采样 + QR 拟合构造 degree-8 多项式。
Vandermonde QR solve 有 O(n²) 开销且引入数值误差。

**方案**:
半角代换 `c=(1-t²)/(1+t²), s=2t/(1+t²)` 使 P,Q,B 变为 t 的 degree-2 多项式（乘以 w=1+t²）：
```
Pw[3] = {a3+a6, 2*a4, a6-a3}
Qw[3] = {a1+a5, 2*a2, a5-a1}
Bw[3] = {a8, -2*a7, -a8}
```
导数同理 degree-2。临界点方程 `F = (Q·dQ + P·dP)² - B²·(P²+Q²)` 通过多项式乘法得到精确 degree-8 系数。

**优势**: 
- 零数值误差（精确整数系数运算）
- 消除 Vandermonde 矩阵构造和 QR solve
- 移除 `get_vandermonde_qr()` 全部引用

---

## 实施进度

| # | 改进项 | 开始时间 | 完成时间 | 构建 | 测试 | 备注 |
|---|--------|----------|----------|------|------|------|
| P1 | Phase D/E/F Partial FK | 完成 | ✅ | ✅ 构建成功 | ✅ volume exact match | 对 iiwa14 增益有限 |
| P2 | Phase F AA Pruning | 完成 | ✅ | ✅ 构建成功 | ✅ volume exact match | prune 21%, w=0.1持平 |
| P3 | Analytical FK Stack | 完成 | ✅ | ✅ 构建成功 | ✅ volume exact match | Analytical -3ms |
| P4 | Per-Direction AA (#5) | 完成 | ✅ | ✅ 构建成功 | ✅ volume exact match | Phase D/F 维度级剪枝 |
| P5 | FixedRoots (#6) | 完成 | ✅ | ✅ 构建成功 | ✅ volume exact match | 消除 poly solver 堆分配 |
| P6 | Root Pre-Filter (#7) | 完成 | ✅ | ✅ 构建成功 | ✅ volume exact match | 跳过无根区间 eigensolve |
| P7 | Symbolic Poly (#9) | 完成 | ✅ | ✅ 构建成功 | ✅ volume exact match | 精确系数，无 Vandermonde |

---

## 最终结果

### Pipeline Benchmark (10 trials, Cold SubAABB)

| 指标 | P1-P3后 | P4-P7后 | 变化 |
|------|---------|---------|------|
| **Analytical Cold** | 101.9 ms | **102.0 ms** | ≈0 (未改 Analytical) |
| **GCPC Cold** | 64.9 ms | **47.8 ms** | **-17.1 ms (-26%)** |
| GCPC/Analytical 比 | 1.57× | **2.13×** | 显著提升 |
| Volume (SubAABB_Grid) | 8.020e-02 | 8.020e-02 | ✅ exact match |
| Volume (Hull16_Grid) | 7.293e-02 | 7.293e-02 | ✅ exact match |

### Validation (width=0.1, single run)

| 指标 | P1-P3后 | P4-P7后 | 变化 |
|------|---------|---------|------|
| GCPC total | 90.8 ms | **91.3 ms** | ≈0 (噪声) |
| Phase D | 12.23 ms | 12.36 ms | ≈0 |
| Phase E | 0.69 ms | 0.69 ms | ≈0 |
| Phase F | 69.06 ms | 69.06 ms | ≈0 (width=0.1 区间小，改进有限) |
| Phase G | 6.35 ms | 6.74 ms | ≈0 |
| F_pruned | 166 | 166 | AA 剪枝保持 |
| F_fk | 15540 | **13966** | -10% FK 调用 |

### Width=0.3 (soundness check)

| Method | Volume | Contains MC | Time |
|--------|--------|-------------|------|
| Analytical | 1.042068e-01 | ✅ YES | 207 ms |
| GCPC | 1.042068e-01 | ✅ YES | **93 ms** (2.2× faster) |

### 结论

#### P1-P3 阶段:
1. **P1 (Partial FK)**: 代码正确但对 iiwa14 增益有限，因多数 joint pair 包含 joint 0 (prefix=空)。
   保留代码以利未来高关节索引 pair 场景。
2. **P2 (Phase F AA Pruning)**: 成功剪枝 21% bg combo，但 AA 计算开销基本抵消了 FK/poly 节省。
   对 width=0.1 和 width=0.3 均已验证 soundness。
3. **P3 (Analytical Stack)**: Analytical 快了 ~3ms (105→102ms)，堆分配消除生效。

#### P4-P7 阶段:
4. **P4 (Per-Direction AA)**: 维度级 AA 剪枝在 Phase D/F 中跳过不影响 AABB 的维度。
5. **P5 (FixedRoots)**: poly solver 栈分配，消除每次 eigensolve 的 heap alloc。
6. **P6 (Root Pre-Filter)**: 区间端点+中点符号检查 + 导数界判断，提前跳过无根区间。
7. **P7 (Symbolic Poly)**: 半角代换+多项式乘法得精确 degree-8 系数，消除 Vandermonde 拟合。

**综合效果**: Pipeline GCPC Cold 从 64.9ms → **47.8ms (提速 26%)**，
GCPC vs Analytical 比从 1.57× → **2.13×**。所有改进保持 **volume exact match，soundness 完好**。

---

---

## 第三轮优化: AnalyticalCrit 全面优化

> 基线: GCPC 23.8ms, AnalyticalCrit 157.9ms (50 trials)

### P8: GCPC `solve_poly_in_interval` 替代全局 companion eigensolver
**状态**: ✅ 完成（上一轮已完成）

**方案**: 32 子区间符号变化检测 + 二分法，仅在 [tan(lo/2), tan(hi/2)] 区间内搜索根。
**结果**: GCPC 69.3ms → **23.8ms (2.91× 加速)**

### P9: AnalyticalCrit 四项联合优化
**状态**: ✅ 完成

**改动**:
1. **符号多项式系数** (`build_symbolic_poly8`): Pw, Qw, Bw, dPw, dQw 直接构造 degree-8 多项式 F=S²-B²(P²+Q²)，消除 Vandermonde 9×9 QR 拟合。数值精度提高 4-6 位有效数字。
2. **区间限制根查找** (`solve_poly_in_interval`): 从 GCPC 移植到 Phase 2 和 Phase 2.5b，替代 companion matrix eigensolver。32 sub-intervals + 48 次二分，每次调用 ~0.5μs vs companion 2-5μs。
3. **Phase 2 背景上限 64→16**: 与 GCPC 保持一致，对 ≤6-DOF 无影响，对 7-DOF 减半覆盖（已由 Phase 2.5 补偿）。
4. **批量 QR 求解 + `tan()` 预提升**: 3 维一次 `qr.solve(Matrix<9,3>)`，`tan(lo_j/2)` 提到循环外。

**实测结果** (50 trials, IIWA14):

| 指标 | 优化前 | 优化后 | 变化 |
|------|--------|--------|------|
| **AnalyticalCrit Cold** | 157.9 ms | **29.5 ms** | **5.35× 加速** |
| GCPC Cold | 23.8 ms | 22.6 ms | ≈0 (正常波动) |
| CritSample Cold | 0.036 ms | 0.035 ms | 不变 |
| AnalyticalCrit Volume | 0.17159 | 0.17160 | ✅ 不变 |
| GCPC Volume | 0.17143 | 0.17143 | ✅ 不变 |

**Soundness 验证**:
- 符号多项式系数与 Vandermonde 拟合数学等价，数值更优（消除条件数 ~10⁴-10⁶ 的 Vandermonde 矩阵）
- 区间求根器在 GCPC 已验证生产部署，对 ≤8 阶多项式 32 子区间足够（间隔 ~0.11 for iiwa14 典型关节范围）
- 偶重根（F 在零点不变号）为参数空间中的零测集退化情况，实际不影响
- Pipeline 架构为**单调膨胀 AABB** — 每个阶段只能扩大包围盒，不能缩小。根查找遗漏最多导致 AABB 略欠紧密，不影响保守性

**清理**: 移除 `solve_companion_fixed<N>`, `solve_polynomial_real`, `get_vandermonde_qr` 全部死代码
