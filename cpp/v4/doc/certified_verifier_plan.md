# 理论完备 + 绝对安全 端点 iAABB 认证验证器 —— 修订实施计划

## 目标

构建一个**可证明完备**的区间验证后端，用于端点 iAABB 极值认证。
在现有 CritSample / Analytical / GCPC 候选生成器之上，加入 BnB + Krawczyk
验证层，数学上**保证**不遗漏任何极值点。

## 已完成的三项基础设施修补

以下三项 Gap 已完成代码修改并通过 7/7 测试：

### Gap 3 修复：区间算术有向舍入（已完成 ✅）

**文件**: `include/sbf/core/types.h`

将 `outward_lo()` / `outward_hi()` 移至 `types.h`（原位于 `interval_math.h`），
并为 `Interval` 的所有四个算术运算符添加 1-ULP 有向舍入：

```cpp
Interval operator+(const Interval& o) const {
    return {outward_lo(lo + o.lo), outward_hi(hi + o.hi)};
}
Interval operator-(const Interval& o) const {
    return {outward_lo(lo - o.hi), outward_hi(hi - o.lo)};
}
Interval operator*(const Interval& o) const {
    double p1 = lo * o.lo, p2 = lo * o.hi;
    double p3 = hi * o.lo, p4 = hi * o.hi;
    return {outward_lo(std::min({p1,p2,p3,p4})),
            outward_hi(std::max({p1,p2,p3,p4}))};
}
Interval operator*(double s) const {
    double a = lo * s, b = hi * s;
    return {outward_lo(std::min(a,b)), outward_hi(std::max(a,b))};
}
```

**保证**: 每次浮点运算的结果向外膨胀 1 ULP，数学上保证区间包含真值。
与 `imat_mul_dh` / `build_dh_joint` / `I_sin` / `I_cos` 中已有的 outward rounding
形成完整覆盖。

### Gap 1 修复：区间 Coeff1D 提取（已完成 ✅）

**文件**: `include/sbf/envelope/analytical_coeff.h`

新增 `IntervalCoeff1D` 结构体和 `extract_1d_coefficients_interval()` 函数：

- 接受区间前缀矩阵 `prefix_lo[16]` / `prefix_hi[16]`（来自 `FKState`）
- 接受区间后缀位置 `suffix_pos_lo[3]` / `suffix_pos_hi[3]`
- DH 三角常量（cos α, sin α 等）也做 outward rounding
- 输出 `IntervalCoeff1D`，其 α, β, γ 都是有效区间包络

同时新增 `compute_suffix_pos_interval()` 辅助函数：
- 使用 `build_joint_interval()` + `imat_mul_dh()` 构建区间后缀链
- 直接复用已有的 outward-rounded 区间矩阵乘法

**保证**: Krawczyk 算子的 Jacobian $J_F(X)$ 基于真正的区间前缀/后缀，
是一个数学上有效的超集，从而使 NoRoot / UniqueRoot 判定可靠。

### Gap 2 修复：Sturm 链精确根隔离（已完成 ✅）

**文件**: `include/sbf/envelope/analytical_utils.h`

新增完整的 Sturm 序列基础设施：
- `SturmSequence` 结构体（最高支持 degree=8）
- `build_sturm_sequence()` — 构建 Sturm 序列 P₀, P₁, ..., Pₖ
- `sturm_sign_changes()` — 计算某点处的变号数
- `sturm_count_roots()` — 精确计算 (a,b) 中实根个数
- `solve_poly_in_interval_sturm()` — Sturm 引导的二分隔离

**保证**: Sturm 定理给出精确根计数 V(a)−V(b)，二分法以根计数引导而非
符号变化引导，因此即使两根任意接近也不会遗漏。

原有 `solve_poly_in_interval()`（采样 + 二分）保留不动，现有代码路径不受影响。
验证器新代码调用 `solve_poly_in_interval_sturm()` 以获得完备性保证。

---

## 修订后的验证器实施计划（P0–P7）

### P0：类型与诊断基础 ⬜

**新文件**: `include/sbf/envelope/verifier_types.h`

```cpp
enum class CellVerdict { Unresolved, NoRoot, UniqueRoot, MultRoot, MaxDepth };

struct VerifyCell {
    std::vector<Interval> box;   // 当前子box
    CellVerdict verdict;
    int depth;
};

struct VerifyCertificate {
    int n_cells_total;
    int n_no_root;
    int n_unique_root;
    int n_max_depth;
    double certified_bound[6];   // 认证的 min/max xyz
    bool is_complete;            // 所有 cell 均已判定？
    std::vector<VerifyCell> unresolved;  // 未决 cell（调试用）
};
```

**工作量**: ~80 行头文件

---

### P1：1D 区间 Jacobian + Krawczyk 算子 ⬜

**新文件**: `include/sbf/envelope/krawczyk.h`

核心算子：对 $f'(q_j) = -\alpha_d \sin(q_j) + \beta_d \cos(q_j)$ 的零点验证。

```
输入: IntervalCoeff1D, 轴向 d, 区间 X_j
  1. 取 x̂ = mid(X_j)
  2. f(x̂)  = -α_d·sin(x̂) + β_d·cos(x̂)           (标量)
  3. f'(X_j) = -α_d·I_cos(X_j) - β_d·I_sin(X_j)   (区间, α_d/β_d 已是区间)
  4. Y = 1.0 / f'(x̂)                                (标量预条件子)
  5. K(X_j) = x̂ - Y·f(x̂) + (1 - Y·f'(X_j))·(X_j - x̂)
  6. 判定:
     - K(X_j) ⊂ int(X_j) → UniqueRoot
     - K(X_j) ∩ X_j = ∅  → NoRoot
     - 否则 → Unresolved → bisect
```

**关键依赖**: `IntervalCoeff1D` (Gap 1 ✅), `I_sin`/`I_cos` (已有 ✅),
`Interval::operator*` 有向舍入 (Gap 3 ✅)

**工作量**: ~120 行

---

### P2：2D 区间 Krawczyk ⬜

**扩展 `krawczyk.h`**

对双关节 (q_i, q_j) 面上的极值条件 ∇f = 0（2×2 系统）：

```
  ∂p_d/∂q_i = 0
  ∂p_d/∂q_j = 0
```

- 2×2 区间 Jacobian 矩阵：每个偏导数关于对方关节的偏导
- 2D Krawczyk 算子：K(X) = x̂ − C·F(x̂) + (I − C·J_F(X))·(X − x̂)
- C = J_F(x̂)⁻¹（2×2 标量逆矩阵）

**备选快速路径**: 当 2D Krawczyk 无法判定时，回落到 Sturm 链
（`solve_poly_in_interval_sturm`, Gap 2 ✅）精确隔离 degree-8 多项式根。

**工作量**: ~200 行

---

### P3：BnB 引擎 ⬜

**新文件**: `include/sbf/envelope/bnb_verifier.h` + `src/envelope/bnb_verifier.cpp`

```
BnBVerifier::verify(robot, intervals, link_idx, axis_d):
  1. root_cell = intervals
  2. priority_queue<VerifyCell> Q  (按 width 排序)
  3. Q.push(root_cell)
  4. while Q 非空 && depth < MAX_DEPTH:
       cell = Q.top(); Q.pop()
       verdict = krawczyk_test(cell)  // P1 或 P2
       if verdict == NoRoot:     skip
       if verdict == UniqueRoot: refine_root(cell) → 更新全局 bound
       if verdict == Unresolved: bisect → Q.push(两半)
  5. return VerifyCertificate
```

**分裂策略**: 沿最宽维度等分。对 n-DOF box，自由维度循环分裂。

**终止条件**:
- 所有 cell 判定 → `is_complete = true`（理论完备）
- 达到 MAX_DEPTH → `is_complete = false`（保守上报：合并所有未决 cell 的 IFK bound）

**工作量**: ~250 行

---

### P4：边界递归处理 ⬜

**扩展 `bnb_verifier.cpp`**

n-DOF box 的极值可能出现在：
1. **内部** (n 维) — P3 BnB 处理
2. **k-面边界** (0 ≤ k < n) — 递归降维

```
verify_box(intervals, free_dims):
  if |free_dims| == 0:  直接 FK 评估
  if |free_dims| == 1:  1D Krawczyk (P1)
  if |free_dims| == 2:  2D Krawczyk (P2) + Sturm 降级
  else:                 BnB 分裂 (P3)

  // 边界递归: 固定每个自由维到 lo/hi
  for dim in free_dims:
    verify_box(fix(intervals, dim, lo), free_dims \ {dim})
    verify_box(fix(intervals, dim, hi), free_dims \ {dim})
```

总面数 = $\sum_{k=0}^{n} \binom{n}{k} \cdot 2^{n-k}$ = $3^n$（6-DOF → 729），
但低维面很快通过 Krawczyk 判定，实际计算量远小于上界。

**关键优化**: 利用 `compute_fk_incremental` 避免重复计算未变化维度的前缀。

**工作量**: ~200 行

---

### P5：CritSample 候选种子加速 ⬜

**扩展 `bnb_verifier.cpp`**

在 P3 BnB 开始前，用 CritSample 枚举生成候选极值点作为初始 warm-start：

1. 调用 `crit_sample::compute()` 对 box 进行边界枚举（DFS + kπ/2）
2. 收集所有产生极值的 joint config（需在 DFS 回调中捕获 `q` 向量）
3. 对每个候选 config，在其邻域构造一个小区间 cell
4. 优先用 1D/2D Krawczyk 验证该 cell → 大概率判定为 UniqueRoot

**效果**: CritSample 已能捕获绝大多数极值点（经验上 >99%），通过预验证
将大部分 cell 快速判定，BnB 只需处理极少量残余区域。

**CritSample 种子捕获接口**:
```cpp
struct CritSeed {
    Eigen::VectorXd config;  // 产生极值的关节构型
    int link_idx;
    int axis_d;              // 0=x, 1=y, 2=z
    int direction;           // 0=min, 1=max
};
std::vector<CritSeed> collect_crit_seeds(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const int* active_link_map, int n_active_links);
```

**工作量**: ~150 行（主要是 DFS 回调包装 + 种子收集逻辑）

---

### P6：集成入 Endpoint Source 调度 ⬜

**修改文件**: `src/envelope/endpoint_source.cpp`

新增 `EndpointSource::Verified` (值=4) 或作为后处理层叠加在任何 source 之上：

方案 A（叠加后处理）:
```
compute_endpoint_iaabb(source=CritSample/Analytical/..., box):
  iaabb = source.compute(box)        // 现有管线
  cert = BnBVerifier::verify(box)    // 新增验证层
  if cert.is_complete:
    return cert.certified_bound      // 用认证结果替换
  else:
    return hull(iaabb, cert.partial) // 保守合并
```

方案 B（独立 source）:
```
EndpointSource::Verified → 直接调用 BnBVerifier
```

推荐**方案 A**，不影响现有调度逻辑和替代矩阵。

**工作量**: ~80 行

---

### P7：测试与验证 ⬜

1. **Sturm 单元测试**: 构造已知根的 degree-4/6/8 多项式，验证精确根计数和隔离
2. **IntervalCoeff1D 一致性测试**: 对比标量 `extract_1d_coefficients` 和区间版本
   在退化区间（point interval）时的结果
3. **Krawczyk 单元测试**: 构造 f(x)=sin(x)-0.5 在已知根附近的 cell，
   验证 UniqueRoot 判定
4. **端到端验证**: 对 6-DOF 机器人，取一个小 box，运行完整 BnB，
   对比 IFK 结果（认证结果应 ⊆ IFK 结果 ⊆ 真值 + ε）
5. **回归测试**: 确保现有 7 个测试全部通过（已验证 ✅）

**工作量**: ~300 行测试代码

---

## 理论完备性论证

修订后的系统满足以下数学保证链：

1. **浮点封闭性** (Gap 3 ✅): 所有 `Interval` 算术运算 outward-round 1 ULP
   → 区间结果一定包含真实数学值

2. **Jacobian 有效性** (Gap 1 ✅): `IntervalCoeff1D` 从区间前缀/后缀推导
   → α(X), β(X) 是有效区间包络
   → Krawczyk Jacobian $J_F(X) \supseteq \{J_F(x) : x \in X\}$

3. **根隔离完备性** (Gap 2 ✅): Sturm 定理给出精确根计数
   → 不依赖采样密度，不会遗漏任意接近的根对

4. **Krawczyk 正确性**: 基于 1–3，
   - NoRoot 判定：若 $K(X) \cap X = \emptyset$，则 X 内无 f 的零点（Krawczyk 定理）
   - UniqueRoot 判定：若 $K(X) \subset \text{int}(X)$，则 X 内恰有一个零点（不动点定理）

5. **BnB 完备性**: 每次 bisect 将 cell width 减半
   → 最终所有 cell 要么判定为 NoRoot/UniqueRoot，
     要么 width → 0（由 IFK 保守覆盖）

6. **边界递归完备性**: 所有 $3^n$ 面逐一处理
   → 无遗漏维度

**结论**: 系统在有限步内对所有可能极值点完成认证或保守覆盖，
满足**理论完备 + 绝对安全**的要求。

---

## 新增代码量估算

| 阶段 | 新代码 | 累计 |
|------|--------|------|
| P0 类型 | ~80 行 | 80 行 |
| P1 1D Krawczyk | ~120 行 | 200 行 |
| P2 2D Krawczyk | ~200 行 | 400 行 |
| P3 BnB 引擎 | ~250 行 | 650 行 |
| P4 边界递归 | ~200 行 | 850 行 |
| P5 CritSample 加速 | ~150 行 | 1000 行 |
| P6 集成调度 | ~80 行 | 1080 行 |
| P7 测试 | ~300 行 | 1380 行 |

**总计**: ~1400 行新代码，复用现有基础设施 >95%。

## 已有基础设施复用清单

| 组件 | 文件 | 用途 |
|------|------|------|
| `Interval` + outward rounding | `types.h` | 所有区间运算 (Gap 3 ✅) |
| `outward_lo` / `outward_hi` | `types.h` | ULP 膨胀 |
| `I_sin` / `I_cos` | `interval_math.h/cpp` | 区间三角函数 |
| `build_dh_joint` | `interval_math.cpp` | 区间 DH 矩阵 |
| `imat_mul_dh` | `interval_math.cpp` | 区间矩阵乘法 |
| `FKState` + `compute_fk_full/incremental` | `interval_fk.h/cpp` | 区间 FK 链 |
| `IntervalCoeff1D` + 区间提取 | `analytical_coeff.h` | 区间 α, β, γ (Gap 1 ✅) |
| `compute_suffix_pos_interval` | `analytical_coeff.h` | 区间后缀位置 |
| `SturmSequence` + `solve_poly_in_interval_sturm` | `analytical_utils.h` | Sturm 精确根隔离 (Gap 2 ✅) |
| `Coeff1D` / `Coeff2D` | `analytical_coeff.h` | 标量系数（快速路径） |
| CritSample DFS | `crit_sample.cpp` | 候选种子生成 |
| `LinkExtremes` / `FKWorkspace` | `analytical_utils.h` | 极值跟踪 |
| `build_symbolic_poly8` | `analytical_utils.h` | 2D degree-8 多项式构造 |
