# Analytical / GCPC 解析管线速度优化分析

> 日期：2026-03-24
> 前提：GCPC 已升级为 Analytical 边界 + GCPC 缓存内部
> 目标：在不损失输出质量的前提下，降低 Analytical / GCPC 的 Stage 1 耗时

---

## 1. 当前基线

iiwa14, 50 trials, Release, seed 42:

| 管线 | 耗时 (ms) | 体积 | VolRatio | FK evals |
|------|-----------|------|----------|----------|
| IFK | 0.021 | 2.512 | 1.000 | 0 |
| CritSample | 0.037 | 0.393 | 0.157 | 511 |
| Analytical | 9.535 | 0.412 | 0.164 | 6948 |
| GCPC | 7.407 | 0.411 | 0.164 | 3343 |

**Per-width breakdown**:

| 管线 | narrow (ms) | medium (ms) | wide (ms) |
|------|------------|-------------|-----------|
| IFK | 0.021 | 0.021 | 0.022 |
| CritSample | 0.029 | 0.035 | 0.060 |
| Analytical | 3.96 | 9.70 | 20.3 |
| GCPC | 2.16 | 7.24 | 18.2 |

**关键观察**：
- Analytical wide (20ms) 比 narrow (4ms) 慢 5×，说明宽区间显著放大计算量
- GCPC 比 Analytical 快 ~22%，差距来自 P3 坐标下降 vs GCPC 缓存查询
- CritSample 体积更小 ~4% 是正常的 under-approximation（UNSAFE），不是质量优势

---

## 2. 每阶段理论迭代量分析（iiwa14, n=7, n_act~4-5）

### 2.1 P0: kπ/2 顶点枚举

**迭代量**: crit_angles per joint = {lo, hi} + kπ/2 interior ≈ 2-6 values
- narrow (5%): ~2-3 per joint → 2^7=128 to 3^7=2187 combos
- wide (100%): ~5-6 per joint → product → cap at CRIT_MAX_COMBOS, fallback to 3^7=2187

**单次成本**: incremental FK (1 matrix multiply per recursive level) + update 4-5 links
**总成本**: ~2K combos × ~0.7μs/combo ≈ 1.5ms

### 2.2 P1: 1D 边求解（atan2）

**外层循环**: per active link (ci) × per joint (j ≤ V)
- Link V=0: 1 joint, bg=1 → 1 iteration
- Link V=2: 3 joints, bg=min(2^2, 128)=4 → 12 iterations
- Link V=4: 5 joints, bg=min(2^4, 128)=16 → 80 iterations
- Link V=6: 7 joints, bg=min(2^6, 128)=64 → 448 iterations
- Total: ~541 iterations (pre-AA pruning)

**单次成本** (with enable_p1_direct_coeff=true):
1. prefix 已有 (ws.tf[j] incremental) — 已累积
2. `compute_suffix_pos()`: build suffix chain from j+1 to V+1 ≈ (V-j) × 4×4 multiply
3. `extract_1d_coefficients()`: ~30 multiplications
4. atan2 + range clamp: trivial
5. Per candidate: `eval_and_update_from(from=j)`: (n-j) matrix multiplies + update 4-5 links
   - 对 j=0, from=0, 全 FK (7 multiplies)
   - 对 j=6, from=6, 仅 1 multiply + tool

**关键瓶颈**: bg combo 的 prefix 重建
- 每次 bg 切换时，需要从 j=0 重新计算 prefix: `ws.set_identity(); for(k<j) ws.compute_joint()`
- 这是 O(j) 次 4×4 矩阵乘法
- 对深层 link (V=6, j=0), 每个 bg combo 需要 0 次 prefix (joint 0 无 prefix)
- 对深层 link (V=6, j=5), 每个 bg combo 需要 5 次 prefix multiply

**优化空间**: **P1 的成本主要在 bg combo × prefix rebuild + suffix build + eval FK**

### 2.3 P2: 2D 面求解（degree-8 polynomial）

**外层循环**: per active link × per face pair × per background combo
- face_all_pairs=true → C(7,2)=21 pairs
- max_bg = min(2^max(nj-2, 0), 16)
- Link V=6: nj=7, max_bg=min(2^5, 16)=16
- Total: ~21 × 4 links × 8 bg ≈ 672 iterations (with AA pruning)

**单次成本** (with enable_p2_direct_coeff=true):
1. prefix 到 j_lo: O(j_lo) multiply
2. `compute_middle_matrix()`: chain from j_lo+1 to j_hi-1 ≈ (j_hi-j_lo-1) multiply
3. `extract_2d_coefficients()`: ~120 multiplications
4. `build_symbolic_poly8()`: ~50 fixed-point ops
5. `solve_poly_in_interval()`: degree-8 polynomial root finding (bisection, ~32 sub-intervals)
   — **this is expensive**: ~200-500 evaluations of degree-8 polynomial
6. Per root: atan2 + range clamp + `eval_and_update_from()` partial FK

**关键瓶颈**: 多项式求根 `solve_poly_in_interval` 和 bg combo prefix rebuild

### 2.4 P2.5: 成对约束求解

**外层循环**: per link × per pair × per kk (sum constraint) × per bg combo
- pairs: up to all C(7,2)=21 (但默认 `pair_all_pairs=false`)
- pair_max_bg=64
- 子约束: qi+qj = kπ/2, 通常 2-4 个 kk

**P2.5a (1D)**: 对每个 kk, 1D atan2 求解 (类似 P1)
**P2.5b (2D)**: 对每个 kk × free joint, degree-8 polynomial 求根 (类似 P2)

**总迭代量**: 约 400-1500 次（受 AA 剪枝影响大）

### 2.5 P3: 内部坐标下降（仅 Analytical, GCPC 用缓存替代）

**外层循环**: per link × 6 face × interior_n_restarts(4) × interior_max_sweeps(3) × n_joints
- dual_phase3=true 时执行两轮（从 P0+P1 checkpoint 和 P0+P1+P2+P2.5 checkpoint）
- 总: ~4 links × 6 × 4 × 3 × 7 × 2 ≈ 4032 iterations

**单次成本**: prefix build + DH 1D coefficient extraction + atan2 + partial FK
这是 Analytical 比 GCPC 慢 22% 的原因。

---

## 3. 成本分解估算（按阶段）

| 阶段 | 迭代量 | 典型 FK evals | 估算耗时占比 |
|------|--------|--------------|-------------|
| P0 (kπ/2 vertices) | ~2000 | ~2000 | ~20% |
| P1 (1D edge) | ~500 | ~500-1000 | ~15% |
| P2 (2D face) | ~700 | ~200-700 | ~25% |
| P2.5 (pair constrained) | ~1000 | ~500-1500 | ~20% |
| P3 (interior) | ~4000 (Analytical only) | ~2000-4000 | ~20% (Analytical) / 0% (GCPC) |
| AA overhead | n_act × 5 refreshes | 0 | ~1% |

---

## 4. 优化方案（不损失质量）

### S1: P1/P2/P2.5 背景组合的 suffix 缓存 【预估 20-30% 加速】

**问题**:
P1 中 `compute_suffix_pos()` 对每个 (link, joint, bg) 组合都独立计算 suffix chain。
但 suffix chain 仅依赖 j+1..V+1 之间的关节角——当 bg combo 只改变了 j 之前的关节时，
suffix 不变。

**方案**:
- 对 P1: 当 bg combo 只改变 joint j 之前的关节时，suffix_pos 不变。
  但实际上 P1 的 bg 确实改变了 j 之前/之后的关节，所以 suffix 确实变化。
  → 不直接适用。

**更实际的方案**: suffix_pos 仅依赖 q[j+1..V]，与 q[0..j-1] 无关。因此对固定的 
(link V, eval_frame, joint j)，suffix_pos 仅在 bg 改变了 q[j+1..V-1] 时才变化。

- 对 P1 edge solver: bg combo 改变 jj ∈ {0..nj-1} \ {j}。若 jj < j, suffix 不变。
  → 可以按 suffix 分组：先枚举 j+1..nj-1 的 bg 值（改变 suffix），对每个 suffix 值
  枚举 0..j-1 的 bg 值（改变 prefix）。这样 suffix 只需计算 2^(nj-j-1) 次而非 2^(nj-1) 次。

- 对 V=6, j=0: suffix 依赖 q[1..6]，bg=64 种组合，suffix 每次都变。无优化空间。
- 对 V=6, j=3: suffix 依赖 q[4..6]，bg 共 64 种，其中 suffix 变化 2^3=8 种，
  prefix 变化 2^2=4 种。suffix 计算从 64 → 8，节省 7/8。

**实现**: 修改 P1/P2 的 bg 枚举为双层循环——外层 suffix bg, 内层 prefix bg。

### S2: P0 增量 FK 改为仅更新被影响的 link 【预估 5-10% 加速】

**问题**: P0 的 `crit_enum_fk` callback 遍历所有 n_act links，即使某些 link 的 
relevant joints 未被当前递归层改变。

**方案**: 在 callback 中只更新 V ≥ depth 的 links。已部分实现（`if (V+1 >= ws.np) continue`），
但 n_sub=1 时内层循环简化为 2 次 update 就够了。

### S3: P2 面求解的 pair 过滤 + AA 轴级剪枝 【预估 10-15% 加速】

**问题**: `face_all_pairs=true` 枚举所有 C(7,2)=21 面对，但许多面对对深层 link 无贡献
（joint pair (i,j) 只影响 V ≥ max(i,j) 的 links）。

**现有 AA 轴级剪枝** (`aa_axis.axis_can_improve[d]`) 已经跳过无需改善的轴，
但外层仍然遍历所有 21 对然后才发现许多 pair 不满足 `pi >= nj || pj >= nj`。

**方案**: 预先为每个 link 构建 valid_pairs 列表（`pi < nj && pj < nj`），避免无用循环。
对 V=0 的 link: 仅 1 个 joint，无面对。
对 V=2 的 link: 仅 C(3,2)=3 面对。
对 V=6 的 link: C(7,2)=21 面对。

### S4: eval_and_update_from 中跳过无关 link 【预估 5-10% 加速】

**问题**: 每次 `eval_and_update_from()` 遍历所有 n_act links 做 update，即使当前
candidate 来自一个特定 link ci 的求解，其他 links 的 bounds 不太可能改善。

**方案**: 对 P1/P2/P2.5，可以仅更新当前求解的 link ci 而非所有 links。
但这可能遗漏跨 link 的"意外"时候改善（同一配置 q 在多个 links 上都可能是极值点）。

**安全的折中**: 仅更新 V ≥ from_joint 的 links。因为 from_joint 之前的关节不变，
更浅层 link 的 FK 结果不变，不需要重新 update。

```cpp
// 当前: 遍历所有 n_act links
for (int ci = 0; ci < n_act; ++ci) { ... }

// 优化: 仅遍历 V >= from_joint 的 links
for (int ci = 0; ci < n_act; ++ci) {
    int V = map[ci];
    if (V < from_joint) continue;  // 这些 link 的 FK 位置未变
    ...
}
```

一次 `eval_and_update_from(from=5)` 原本遍历 4-5 links，优化后仅遍历 1-2 links。

### S5: 多项式求根优化 【预估 5-10% 加速】

**问题**: `solve_poly_in_interval()` 使用 32 子区间 bisection 找 degree-8 多项式的根。
每个子区间做 ~20 次 bisection 迭代，每次求值 degree-8 多项式 (~16 multiply)。
总: ~32 × 20 × 16 = 10240 次乘法/次调用。P2 中约 ~3000 次调用。

**方案**: 
1. **自适应子区间数**: 窄区间（narrow box）使用更少的子区间（如 16 或 8）
2. **Sturm chain 预过滤**: 快速判断区间内有无根，有则再 bisect
3. **区间 tan(θ/2) 范围收窄**: 当 box narrow 时, t_lo ≈ t_hi, 可大幅减少搜索范围

### S6: P2.5 pair_all_pairs=false 保持默认 【质量/速度权衡点】

**现状**: `pair_all_pairs=false`（默认），仅枚举 coupled + adjacent + 间隔-2 pairs。
这比 `face_all_pairs=true` 的 C(7,2) 小很多。

**分析**: P2.5 的约束 qi+qj = kπ/2 对非相邻关节对贡献有限。保持默认可节省 ~30% P2.5 时间。

### S7: dual_phase3 关闭（GCPC 路径不受影响） 【Analytical 专属 ~10% 加速】

**现状**: `dual_phase3=true` 在 P2 之前和之后都运行 P3，希望不同起始点找到不同极值。

**方案**: 关闭 dual_phase3（仅从最终 checkpoint 运行 P3），P3 迭代量减半。
GCPC 路径不使用 P3，不受影响。
Analytical 路径质量可能微降（但 P3 本身是启发式，无完备性保证）。

### S8: link-level 早期退出 + 分层 AA 刷新 【预估 5-15% 加速, 对 narrow 显著】

**问题**: AA 剪枝仅在每个 phase 开始时刷新一次。Phase 内部的迭代可能已经收敛了某些 link，
但仍继续计算。

**方案**: 
- 每 K 次 eval 后重新检查 AA pruning（如 K=100），动态标记已收敛的 link。
- 对 narrow box，AA bounds 很紧，P0 之后可能已经剪掉大部分 links → P1/P2 仅处理少量 link。

### S9: VectorXd → 固定数组 (堆分配消除) 【预估 3-5% 加速】

**问题**: `LinkExtremes::configs[6]` 是 `Eigen::VectorXd`（每个 7 维，堆分配）。
每次 `update()` 调用可能触发 `VectorXd::operator=` 深拷贝。
n_act × n_sub × 6 = 4 × 1 × 6 = 24 个 VectorXd × 多次拷贝。

**方案**: 改为 `double configs[6][8]`。已在 O7 中提出，延后执行。

---

## 5. 优化优先级排序

| 优先级 | 编号 | 方案 | 预估加速 | 实现难度 | 质量影响 |
|--------|------|------|----------|----------|----------|
| **P0** | S4 | eval_and_update_from 跳过浅层 link | 5-10% | **低** | 零 |
| **P0** | S3 | P2 面对预过滤 + 跳过无效 pair | 10-15% | **低** | 零 |
| **P1** | S1 | suffix 缓存 / 双层 bg 枚举 | 20-30% | **中** | 零 |
| **P1** | S8 | Phase 内动态 AA 刷新 | 5-15% | **中** | 零 |
| **P2** | S5 | 多项式求根自适应子区间 | 5-10% | **低** | 零 |
| **P2** | S9 | VectorXd → 固定数组 | 3-5% | **中** | 零 |
| **P3** | S7 | dual_phase3 关闭 | ~10% (Analytical) | **极低** | 微降（启发式部分）|

### 第一批实施目标（预估总加速 30-50%）

1. **S4**: `eval_and_update_from` 添加 `if (V < from_joint) continue` — 1 行改动
2. **S3**: P2 face pairs 预过滤 — ~10 行改动
3. **S5**: `solve_poly_in_interval` 自适应子区间数 — ~5 行改动
4. **S8**: 滚动 AA 刷新 — ~20 行改动

### 第二批实施目标（更高加速，需要更多重构）

5. **S1**: P1/P2 双层 bg 枚举（suffix cache） — ~100 行重构
6. **S9**: VectorXd → 固定数组 — ~50 行改动

---

## 6. CritSample 体积更小的解释

CritSample (VolRatio 0.157) 比 Analytical/GCPC (VolRatio 0.164) 更紧 ~4%。

**原因**: CritSample 属于 UNSAFE 安全类，其 kπ/2 笛卡尔积枚举是有限采样——
如果真实极值点不在 kπ/2 网格上，CritSample 会遗漏它，导致 iAABB 偏小。
这不是 CritSample "更精确"，而是 under-approximation（不完备）。

Analytical/GCPC (SAFE) 通过解析求根保证覆盖所有理论临界点（P1 边, P2 面, P2.5 约束），
因此产出的 iAABB 保证 ⊇ 真实可达集。体积略大是预期行为。

**wide 区间差距更大 (0.119 vs 0.127)** 是因为：宽区间内真实极值点更可能偏离 kπ/2 网格，
CritSample 遗漏概率增加。

---

## 7. Analytical vs GCPC 的架构权衡

| | Analytical | GCPC |
|--|-----------|------|
| 边界求解 | P0+P1+P2+P2.5+AA | 完全相同 |
| 内部求解 | P3 坐标下降（启发式）| GCPC 缓存查找 |
| 预计算 | 无 | GCPC 缓存（一次性 ~230ms）|
| 耗时 | 9.5ms | 7.4ms (-22%) |
| 体积 | 0.412 | 0.411 (≈) |
| 适用场景 | 无缓存、单次查询 | 长时间规划（缓存分摊）|

GCPC 的内部缓存覆盖面通常优于 P3 坐标下降（全局搜索 vs 局部启发式），
且查询速度更快，因此推荐作为生产环境的默认 Stage 1 源。

Analytical 适用于不需要预计算缓存的场景（如单次路径验证）。
