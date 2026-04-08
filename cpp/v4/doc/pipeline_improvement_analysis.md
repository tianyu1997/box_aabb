# Envelope Pipeline 改进空间分析

> 基于 v4 代码审计 + `phase_compare.csv` (200 trials, seed 42) 的定量分析

---

## 一、当前性能基线

### 1.1 各管线平均耗时

| 管线 | 平均耗时 (ms) | v4/v3 加速比 |
|------|-------------|-------------|
| IFK | 0.028 | ~1.0× |
| CritSample | 0.047 | 0.67× (33%↓) |
| Analytical | 15.60 | 0.60× (40%↓) |
| GCPC | 15.90 | 0.48× (52%↓) |

### 1.2 GCPC 八阶段耗时分布 (全宽度平均)

| Phase | 耗时 (ms) | 占比 (%) | 功能 |
|-------|----------|---------|------|
| A: Cache查询 | 0.122 | 0.8% | KD-tree range query |
| B: Boundary kπ/2 | 0.032 | 0.2% | 边界枚举 + 预计算DH栈DFS |
| C: 1D atan2 | 0.661 | 4.2% | 直接一维系数 + atan2 |
| D: 2D face | 0.471 | 3.0% | 直接二维系数 + 8次多项式 |
| E: Pair 1D | 0.699 | 4.4% | qi+qj=kπ/2 + atan2 |
| **F: Pair 2D** | **12.27** | **77.2%** | **qi+qj=kπ/2 + 自由关节m** |
| G: Interior CD | 1.63 | 10.3% | 坐标下降 |

### 1.3 Phase F 按宽度分布

| width | F (ms) | F占比 | n_f候选 | 单候选耗时 (µs) |
|-------|--------|-------|---------|----------------|
| 0.05 | 1.57 | 43.8% | 1 | 1700.7 |
| 0.10 | 3.63 | 57.5% | 27 | 136.4 |
| 0.20 | 8.01 | 71.3% | 115 | 69.7 |
| 0.50 | 20.39 | 82.3% | 1203 | 16.9 |
| 1.00 | 41.73 | 85.1% | 6265 | 6.7 |

**关键发现**: Phase F 占 GCPC 总耗时的 77-85%，是唯一有量级改进空间的阶段。

---

## 二、GCPC 管线改进机会

### 2.1 [H6] Phase F: 9点采样→直接系数提取 ⬆️ 最高优先级

**现状分析**:
Phase F 求解 qi+qj=kπ/2 约束下自由关节 m 的极值。当前方法：
1. 对 (qi, qm) 取 3×3=9 个采样点
2. 计算 9 个位置 → 构建 9×3 RHS 矩阵
3. H5-A: `coeff_3 = A_inv_f * rhs_3` (9×9 × 9×3 = 243 次乘法)
4. `build_symbolic_poly8` → `solve_poly_in_interval` → atan2 恢复 qi

**9参数双线性模型的近似性**:
由于约束 qj = C - qi，关节 j 的旋转矩阵也是 (cos qi, sin qi) 的线性函数。
DH链乘积 Ti(qi) · T_mid · Tj(C-qi) 实际是 (cos qi, sin qi) 的**二次函数**。
因此位置函数 f(qi, qm) 的真实基底有 5×3=15 项（而非 9 项）：

$$f_d(q_i, q_m) = \sum_{p+q \leq 2,\, r+s \leq 1} c_{p,q,r,s}\, \cos^p q_i \sin^q q_i \cos^r q_m \sin^s q_m$$

当前 9 系数模型是 15 参数真实函数的**线性近似**（忽略了 cos 2qi, sin 2qi 与 qm 的耦合项）。
现有测试全部通过表明该近似对 IIWA-14 是可接受的。

**改进方案A: 完整15参数模型**
- 需要 5×3=15 采样点 + 15×15 矩阵求逆
- 多项式次数从 8 升至 ~16 → 求根代价翻倍
- 无近似误差，理论上更精确
- 总代价: 增加约 50% → **不推荐**

**改进方案B: 保持9参数模型，优化计算路径**
- 目标: 将每个 bg 迭代的代价从 ~387 次乘法 + 多项式求根 → ~200 次乘法 + 多项式求根
- 具体措施:
  1. **统一 left_pre × tf_j2 × right 的矩阵链**: 当前对 9 个(si,sm)组合分别做 mat4×vec4 乘法。可以预先将 `left_pre[k1]` 和 `tf_j2_pre[k2]` 合并为 `combined[k1][k2]`（6 个 4×4 矩阵），每个只需一次 vec4 乘法
  2. **RHS矩阵结构利用**: `rhs_3` 的 9×3 结构有大量零模式，可展开为直接标量运算（消除 Eigen 矩阵开销）
  3. **A_inv_f 展开为编译期常数**: 9 个采样角度是确定性的（lo, mid, hi），A_inv_f 可以预计算为 81 个 double 常数
- 预估加速: ~30-40% Phase F → ~25-30% GCPC总加速

**改进方案C: 自适应 bg 枚举截断** ⬆️ 推荐
- 现状: Phase F 枚举所有 2^n_bg 个背景关节组合
- 问题: n_bg=4 时有 16 个组合，n_bg=5 时有 32 个 → 线性增长
- 改进: 维护一个 `improvement_streak` 计数器，当连续 K 个 bg 组合都不更新极值时，提前终止
- K=4 时预估可跳过 30-50% 的 bg 迭代（尤其在窄宽度区间）
- 代价: 零额外内存，判断开销 ~1 比较/迭代
- **预估加速: 20-40% Phase F**

**改进方案D: 窄区间快速路径**
- 当 eff_hi - eff_lo < threshold（如 0.1 rad），(qi, qm) 的变化范围小
- 可降级为 Phase E 式的离散评估（只算 lo/mid/hi 的笛卡尔积），跳过多项式求根
- 对 w=0.05 场景（Phase F 只有 1 个候选但耗时 1.7ms 的 setup）改进显著

### 2.2 [H7] Phase G: 种子改进 + 早期收敛 ⬅ 中优先级

**现状**: 1.63ms (10.3%)，0.3-1.0µs/候选，3 轮坐标扫描
**改进**:
1. **Phase F 极值回传**: Phase F 找到的极值配置作为 Phase G 的额外种子（目前 Phase G 只用 A-F 最优 + A+B checkpoint）
2. **收敛检测**: 如果第 2 轮扫描的改进量 < ε（如 1e-8），跳过第 3 轮
3. **AA 维度跳过**: 对已被 AA 证明紧致的维度，跳过该维度的坐标下降
- **预估加速: 15-30% Phase G → ~1.5-3% GCPC总加速**

### 2.3 [H8] Phase C+D 向量化 ⬅ 低优先级

**现状**: C+D 合计 1.13ms (7.1%)
- Phase C 的 extract_1d_coefficients (~30 mults) + atan2 已经很高效
- Phase D 的 extract_2d_coefficients (~120 mults) + poly8 solve 也较高效
**改进**: 批处理多个链路/维度的系数提取，利用 AVX2 SIMD
- **预估加速: ~20% C+D → ~1.4% GCPC总加速**
- 投入产出比低，**不推荐优先实施**

### 2.4 [H9] 自适应管线选择 ⬅ 中优先级

**数据观察**:
| width | GCPC (ms) | Analytical (ms) | GCPC/Anal |
|-------|----------|-----------------|-----------|
| 0.05 | 3.59 | 3.11 | **1.155** |
| 0.10 | 6.32 | 5.57 | **1.136** |
| 0.15 | 8.06 | 7.46 | **1.081** |
| 0.30 | 15.38 | 15.40 | 0.998 |
| 0.50 | 24.77 | 27.73 | 0.893 |
| 1.00 | 49.06 | 41.73 | **1.176** |

**关键发现**: 
- GCPC 在 w<0.15 时反而比 Analytical **慢** 15%
- GCPC 在 w=0.3-0.7 最优（10-20% 快于 Analytical）
- GCPC 在 w=1.0 时又慢于 Analytical

**改进**: 基于 interval 宽度的自适应选择：
- w < 0.15: 使用 Analytical（跳过 Phase E/F）
- 0.15 ≤ w ≤ 0.7: 使用 GCPC
- w > 0.7: 使用 Analytical 或混合策略
- **预估加速**: 窄/宽区间场景 10-15%

---

## 三、Analytical 管线改进机会

### 3.1 Phase 2.5b (pair-constrained 2D) — 同构优化

Analytical 的 `solve_pair_constrained_2d()` 与 GCPC Phase F 使用**完全相同**的算法结构：
- 相同的 9 系数双线性模型
- 相同的 `build_symbolic_poly8` + `solve_poly_in_interval`
- 但使用 `qr_2d.solve()` 代替 GCPC 的 `A_inv_f *`（QR分解 vs 预计算逆矩阵）
- 无 H5-B 增量链 → 每次 bg 迭代重算完整前缀链

**改进**:
1. **移植 H5-A**: QR → 预计算逆矩阵（已在 GCPC 验证，直接复制）
2. **移植 H5-B**: 4段增量链 + segment-dirty 追踪
3. **预估加速**: Phase 2.5b 约 40-50% → Analytical 总约 20-30%（取决于 2.5b 占 Analytical 的比重）

### 3.2 Phase 3 (Interior CD) — 协同优化

**现状**: P3 FK 调用数从 949 (w=0.05) 到 7899 (w=1.0)，占 Analytical 相当大比重
- 使用 `solve_interior_improved`: 多起点 + pair coupling + 直接1D系数 → 与 GCPC Phase G 一致
- 支持 `dual_phase3`: 从 Phase 0+1 checkpoint 和 Phase 0-2.5 状态分别启动

**改进同 GCPC Phase G**:
1. 收敛早期终止
2. AA 维度跳过
3. 更好的种子策略

### 3.3 Phase 2.5 FK 调用分析

| width | P25_FK | P3_FK | total (P25+P3) |
|-------|--------|-------|----------------|
| 0.05 | 11 | 949 | 960 |
| 0.20 | 468 | 3474 | 3942 |
| 0.50 | 3442 | 5336 | 8778 |
| 1.00 | 12754 | 7899 | 20653 |

在宽区间 (w≥0.5)，P25 FK 调用数**超过** P3，说明 Phase 2.5b 是 Analytical 的主要瓶颈。
这与 GCPC Phase F 的瓶颈位置一致 — 二者共享同一算法框架。

---

## 四、CritSample 管线改进

**当前**: 0.047ms — 已接近理论下限

### 4.1 唯一有意义的改进: 提升紧致度

CritSample = GCPC Cache 查询 + 边界 kπ/2 枚举。改进速度无意义（已经 <0.1ms）。

**紧致度提升**:
1. **增密 GCPC Cache**: 当前 cache 的 KD-tree 查询返回少量（0-2869 个）候选点。增大 cache 密度可返回更多候选 → 更紧AABB
2. **自适应候选去重阈值**: 当前 `DedupKey` 使用 1e-5 离散化。可根据 interval 宽度自适应调整

**代价**: 更密的 cache → 更大内存 + 更慢的构建（但查询时间影响不大）

---

## 五、IFK 管线改进

**当前**: 0.028ms — 区间算术理论下限

- `compute_fk_full`: 全链区间矩阵乘法
- `compute_fk_incremental`: 指定 `changed_dim` 只重算变化关节
- `FKState` 存储 `prefix_lo/hi[MAX_TF][16]` 避免动态分配

**无有意义的改进空间**。IFK 的计算量完全由区间 4×4 矩阵乘法次数决定，
已通过增量计算最小化。

---

## 六、跨管线共享改进

### 6.1 统一 Phase F / Phase 2.5b 代码

GCPC Phase F 和 Analytical Phase 2.5b 是**同一算法的两个实现**，
应提取为共享的 `solve_pair_2d_core()` 函数：
- 统一 H5-A/H5-B 优化
- 统一 AA pruning 接口
- 减少维护负担

### 6.2 AA Pruning 框架升级

当前 AA pruning 分三层:
1. **Link-level**: 跳过整个链路（Phase C-G, Analytical Phase 1-3）
2. **Per-face (Gray-code)**: Phase C/D 中按 bg 组合检查（有效）
3. **Per-dimension**: Phase F 中有 `aa_dim_can_improve` 但只在外层检查

**改进**: Phase F 内层 bg 循环中加入 **per-dimension early-exit**:
```cpp
for (int d = 0; d < 3; ++d) {
    if (do_aa_f && !aa_dim_can_improve(aa_face_f_ws.prefix, ci, d))
        continue;  // 跳过此维度的多项式求根
    // ... poly8 solve ...
}
```
目前此检查使用的是**外层** `aa_fk_ws`（全区间 AA），非 `aa_face_f_ws`（当前 bg 的 AA）。
若切换为当前 bg 的 AA bounds，pruning 率会显著提高。

### 6.3 预计算优化共享

| 预计算项 | 当前状态 | 共享机会 |
|---------|---------|---------|
| PairKpi2Config | GCPC Phase D½ 预计算 | 可共享给 Analytical Phase 2.5 |
| GcpcCache KD-tree | GCPC Phase A 使用 | 可用于 CritSample 种子 ✓ (已实现) |
| AA FK bounds | 每管线独立计算 | 可计算一次共享 |

---

## 七、改进优先级总结

| 编号 | 改进项 | 影响管线 | 预估加速 | 实施难度 | 优先级 |
|------|--------|---------|---------|---------|--------|
| **H6** | Phase F 自适应bg截断 | GCPC | **20-40% F → 15-30% 总** | 低 | ⬆⬆⬆ |
| **H7** | Phase F 窄区间快速路径 | GCPC | 消除setup开销 | 低 | ⬆⬆ |
| **H8** | Analytical移植H5-A/B | Analytical | **30-40% P2.5b** | 中 | ⬆⬆ |
| **H9** | Phase F per-dim AA pruning | GCPC+Analytical | 10-20% F | 低 | ⬆⬆ |
| H10 | Phase G 收敛早停 | GCPC+Analytical | 15-30% G → 1.5% 总 | 低 | ⬆ |
| H11 | 自适应管线选择 | 端到端 | 10-15% 窄/宽区间 | 中 | ⬆ |
| H12 | Phase F 矩阵链合并 | GCPC | 30-40% F | 中 | ⬆ |
| H13 | 统一Phase F/P2.5b代码 | 维护性 | 0 (架构改进) | 中 | ⬆ |
| H14 | Phase C+D SIMD | GCPC | ~1.4% 总 | 高 | ↓ |

### 理论极限分析

若将 Phase F 完全消除（理论不可能，但作为上界参考）：
- GCPC: 15.90ms → 3.63ms (ratio 0.233 vs Analytical)
- 若 Phase F 减半: 15.90ms → 9.76ms (ratio 0.626)
- **最现实目标**: H6+H7+H9 组合 → Phase F 减 40-50% → GCPC 约 9-10ms → 比 Analytical 快 35-40%

---

## 八、附录: Phase F 计算流程详解

```
对每个 (pair i,j), 约束 C = kπ/2, 自由关节 m:
  预计算 A_inv_f (9×9)                    // H5-A
  预计算 tf_j1_pre[3], tf_j2_pre[3], tf_j3_pre[3]
  初始化 4段增量链: prefix, mid12, mid23, suffix  // H5-B
  
  对每个 bg 组合 (Gray-code):
    增量更新 dirty segments                 // H5-B
    AA pruning (link-level)                // 跳过不可能改进的组合
    
    计算 9 个位置:                          // 主要代价
      for si=0..2, sm=0..2:
        pos[si*3+sm] = left_pre[k1] * tf_j2[k2] * right[k3]
    
    coeff_3 = A_inv_f * rhs_3              // 243 次乘法
    
    for dim d=0..2:                        // 多项式求根
      build_symbolic_poly8(coeffs) → pa[9]
      solve_poly_in_interval(pa, 8, tm_lo, tm_hi)
      for each root tm:
        qm = half_angle_to_q(tm)
        qi = atan2(P, Q)                   // 从 qm 推导 qi
        validate qi ∈ [eff_lo, eff_hi]
        eval_partial(q) → update extremes
```

每个 bg 迭代的代价分解:
- 9 位置计算: ~144 次乘法
- A_inv × rhs: ~243 次乘法
- 多项式构建+求解: ~200 次乘法 (每维度)
- 总计: ~387 + 600 = ~987 次乘法/bg (3维度)
