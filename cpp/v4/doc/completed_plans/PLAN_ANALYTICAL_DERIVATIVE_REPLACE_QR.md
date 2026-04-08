# 解析微分全面替代采样 + QR：详细执行计划

> 生成日期：2026-03-20
> 前置文档：`PLAN_ANALYTICAL_NEXT.md`（P11 高阶条目）、`PLAN_ANALYTICAL_OPTIMIZED.md`（O5 系列实验记录）
> 目标：将 P1/P2/P2.5 所有阶段的 **"FK 采样 + QR 拟合"** 系数提取路径替换为 **DH 链解析推导**，消除冗余 FK 评估，达成总 FK 调用量下降 30%+ 的性能目标。

---

## 一、技术背景

### 1.1 当前系数提取方式：采样 + QR 拟合

在 P1/P2/P2.5 各阶段中，为获取各坐标轴上的三角函数系数，当前代码对自由关节进行**固定点采样**，利用 FK 正运动学计算采样点处的末端位置，再用 QR 分解拟合出三角函数系数：

| 阶段 | 1D 模型 ($p_d = \alpha c_j + \beta s_j + \gamma$) | 2D 模型 ($p_d = \sum a_k$ 双线性 9 项) |
|------|---|---|
| **P1 Edge** | ✅ 3 点采样，3×3 QR | — |
| **P2 Face** | — | ✅ 3×3=9 点采样，9×9 QR |
| **P2.5a Pair-1D** | ✅ 3 点采样，3×3 QR | — |
| **P2.5b Pair-2D** | — | ✅ 3×3=9 点采样，9×9 QR |

每组采样分别需要 3 次 (1D) 或 9 次 (2D) FK 前向运动学调用，这些 FK 调用合计**占全管线 FK 总量的绝大部分**。一旦系数已知，后续的 `atan2` / `build_symbolic_poly8` + `solve_poly_in_interval` 根求解过程本身不需要 FK。

### 1.2 DH 链的结构性质

DH 变换矩阵 $T_j(q_j) = Rz(q_j + \theta_j) \cdot Tz(d_j) \cdot Tx(a_j) \cdot Rx(\alpha_j)$ 的 (0,3)/(1,3)/(2,3) 列是关于 $(\cos q_j, \sin q_j)$ 的**严格线性函数**。因此：

- **1D 模型**：当仅 $q_j$ 自由、其余关节固定时，
  $$p_d = [\text{prefix row}_d] \cdot T_j(q_j) \cdot [\text{suffix col}] = \alpha_d \cos q_j + \beta_d \sin q_j + \gamma_d$$
  系数 $(\alpha_d, \beta_d, \gamma_d)$ 可从 prefix/suffix 矩阵元素**直接提取**，无需 FK 采样。

- **2D 模型**：当 $q_i, q_j$ 自由时，利用 Kronecker 积分解 $A_{9\times9} = U_j \otimes U_i$，其中 $U_i, U_j$ 为 3×3 插值矩阵，9 系数可分解为 $M = U_i^{-1} \cdot B_{grid} \cdot U_j^{-T}$。当从 DH 链直接推导时，更可**跳过 $B_{grid}$ 的 FK 采样**，直接从链式矩阵乘积中提取 Kronecker 系数。

### 1.3 O5 原型验证数据

| 实验 | 结论 |
|------|------|
| O5.3c 门控阈值 | medium 档 `gate_accept_ratio ≈ 31.18%`，`reject_fit` 为主过滤源 |
| O5.4b shadow | `coverage = 100%`，`candidate_per_eval = 0.0075` |
| O5.6/O5.7 骨架 | 骨架无副作用，hybrid 统计全 0（因实现为空） |

**关键发现**：当前 `compute_p2_symbolic_coeff_from_grid()` 仍需 9 个 FK 采样点作为输入 — 它只是用 **矩阵逆** 代替了 **QR 分解** 来提取系数，但 FK 采样未减少。若改为从 DH 参数直接推导系数，可完全消除 P2/P2.5b 的 9 次 FK 采样。

---

## 二、总体架构设计

### 2.1 核心思路：prefix/suffix 链式系数直接提取

```
DH Chain:  T_base → T_0(q_0) → T_1(q_1) → ... → T_{n-1}(q_{n-1}) → T_tool

对 link V+1 的位置：
  p = T_0...T_{j-1} · T_j(q_j) · T_{j+1}...T_n · [0,0,0,1]ᵀ
    = Prefix_j · T_j(q_j) · Suffix_j

其中 Prefix_j 和 Suffix_j 不含 q_j，可在切换 background 后一次性计算。
```

#### 1D 系数直接提取 (P1, P2.5a)

```
T_j(q_j) = [[c_j, -s_j·cα, s_j·sα,  a·c_j],
             [s_j,  c_j·cα, -c_j·sα, a·s_j],
             [0,    sα,      cα,      d    ],
             [0,    0,       0,       1    ]]

p_d = prefix_row_d · T_j · suffix_col
    = (prefix_row_d · col_1(T_j)) · c_j + (prefix_row_d · col_2(T_j)) · s_j + ...
```

系数 $(\alpha_d, \beta_d, \gamma_d)$ 可直接从 prefix/suffix 矩阵元素组合得出 — **零 FK 调用**。

#### 2D 系数直接提取 (P2, P2.5b)

```
p_d(q_i, q_j) = Prefix_i · T_i(q_i) · Middle_{ij} · T_j(q_j) · Suffix_j · e
```

展开后，$p_d$ 为关于 $(c_i, s_i, 1) \otimes (c_j, s_j, 1)$ 的双线性函数，9 个系数均可从 prefix / middle / suffix 矩阵元素的代数组合直接提取 — **零 FK 调用**。

### 2.2 新增模块结构

```
include/sbf/envelope/
  analytical_coeff.h        ← 新增：DH 链系数直接提取 API
    struct CoeffExtractor1D { α[3], β[3], γ[3] per axis }
    struct CoeffExtractor2D { a[9][3] per axis }
    void extract_1d_coefficients(prefix, suffix, DH_j, ...)
    void extract_2d_coefficients(prefix, middle, suffix, DH_i, DH_j, ...)
    (inline / header-only for零分配)

src/envelope/
  analytical_solve.cpp      ← 修改：在各阶段增加 symbolic-direct 路径
```

---

## 三、分阶段执行计划

### Phase A：P2 Hybrid 写回实现（PLAN_NEXT P1+P2 的落地）

**目标**：让当前已有的 `compute_p2_symbolic_coeff_from_grid()` 路径在门控通过时真正用于 root-finding。这是后续各阶段的**信心基础**。

#### A.1 实现 Hybrid 写回 (`~50 LOC`)

在 `solve_faces()` 的 `for (int d = 0; d < 3; ++d)` 循环中（L678 区域），插入选择逻辑：

```cpp
// 在 coeff_3d(k,d) 使用之前：
double a1, a2, ..., a8;  // 用于 root-finding 的系数

if (config.enable_p2_derivative_hybrid &&
    symbolic_ready && gate_accept_axis[d]) {
    // 使用 symbolic 系数
    const auto coeff_sym = compute_p2_symbolic_coeff_from_grid(...);
    a1 = coeff_sym(0,0); ... a8 = coeff_sym(7,0);

    // 可选 fallback 检查
    if (config.enable_p2_hybrid_fallback) {
        double max_diff = 0.0;
        for (int k = 0; k < 8; ++k)
            max_diff = std::max(max_diff, std::abs(coeff_sym(k,0) - coeff_3d(k,d)));
        if (max_diff > config.p2_hybrid_max_abs_error) {
            // 回退到 QR 系数
            a1 = coeff_3d(0,d); ... a8 = coeff_3d(7,d);
            ++p2_proto_metrics->n_hybrid_fallback_cases;
        } else {
            ++p2_proto_metrics->n_hybrid_applied_cases;
        }
    } else {
        ++p2_proto_metrics->n_hybrid_applied_cases;
    }
    ++p2_proto_metrics->n_hybrid_eval_cases;
} else {
    // 默认 QR 路径
    a1 = coeff_3d(0,d); ... a8 = coeff_3d(7,d);
}
```

#### A.2 扩展 P2ProtoMetrics + 合并函数

```cpp
struct P2ProtoMetrics {
    ...  // 现有字段
    // 新增:
    int64_t n_hybrid_eval_cases = 0;
    int64_t n_hybrid_applied_cases = 0;
    int64_t n_hybrid_fallback_cases = 0;
};

// merge_p2_proto_metrics() 中追加合并
```

#### A.3 统计写回到 AnalyticalCriticalStats

在 `derive_aabb_critical_analytical()` 末尾的统计写回区域增加：

```cpp
stats.n_p2_hybrid_eval_cases = p2_proto.n_hybrid_eval_cases;
stats.n_p2_hybrid_applied_cases = p2_proto.n_hybrid_applied_cases;
stats.n_p2_hybrid_fallback_cases = p2_proto.n_hybrid_fallback_cases;
```

#### A.4 单元测试

- `test_p2_hybrid_off()`：`enable_p2_derivative_hybrid=false`，结果与 baseline 完全一致。
- `test_p2_hybrid_on()`：`enable_p2_derivative_hybrid=true`，AABB 结果与 baseline 一致 (`1e-6`)。
- `test_p2_hybrid_fallback()`：用极端阈值 (`max_abs_error=0`) 验证 100% 回退。

#### A.5 N=10 A/B 实验 (O5.8)

- baseline: `SBF_P2_DERIVATIVE_HYBRID=0`
- trial: `SBF_P2_DERIVATIVE_HYBRID=1, SBF_P2_HYBRID_FALLBACK=1`
- 指标：`mean_ms_delta_pct`、`hybrid_eval/applied/fallback`、`fk_total_delta_pct`

**验收标准**：
- hybrid 统计非零且稳定
- AABB 结果一致 (`1e-6`)
- 决策：GO → 进入 Phase B；NO-GO → 调整阈值重试

**预计改动量**：~90 行 C++ + ~40 行测试
**预计工期**：0.5 天

---

### Phase B：P1 DH 链系数直接提取（消除 P1 采样 FK）

**目标**：P1 (1D edge) 当前每组 (joint × background) 需 3 次 FK 调用来拟合 $(\alpha, \beta, \gamma)$。改为从 prefix/suffix 矩阵直接提取系数后，这 3 次 FK 可完全消除。

#### B.1 数学推导

对第 $j$ 个关节，DH 变换矩阵（revolute）：

$$
T_j(q_j) = \begin{bmatrix}
c_j & -s_j c_\alpha & s_j s_\alpha & a_j c_j \\
s_j & c_j c_\alpha & -c_j s_\alpha & a_j s_j \\
0 & s_\alpha & c_\alpha & d_j \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

其中 $c_j = \cos(q_j + \theta_j)$，$s_j = \sin(q_j + \theta_j)$。

Link $V+1$ 的位置 $p = \text{Prefix}_j \cdot T_j \cdot \text{Suffix}_j \cdot [0,0,0,1]^T$。

令 $\mathbf{r} = \text{Suffix}_j \cdot [0,0,0,1]^T = [r_0, r_1, r_2, 1]$（suffix 产生的 4D 齐次列向量），$\mathbf{L} = \text{Prefix}_j$（4×4 矩阵），则：

$$
p_d = L_{d,0}(r_0 c_j - r_1 c_\alpha s_j + r_2 s_\alpha s_j + a_j c_j) + \\
      L_{d,1}(r_0 s_j + r_1 c_\alpha c_j - r_2 s_\alpha c_j + a_j s_j) + \\
      L_{d,2}(r_1 s_\alpha + r_2 c_\alpha + d_j) + L_{d,3}
$$

整理后：
$$
\alpha_d = L_{d,0}(r_0 + a_j) + L_{d,1} \cdot r_1 c_\alpha - L_{d,1} \cdot r_2 s_\alpha \\
\beta_d = -L_{d,0}(r_1 c_\alpha - r_2 s_\alpha) + L_{d,1}(r_0 + a_j) \\
\gamma_d = L_{d,2}(r_1 s_\alpha + r_2 c_\alpha + d_j) + L_{d,3}
$$

> **注意**：以上需处理 $\theta_j$ 偏移——实际 DH 中 $c_j = \cos(q_j + \theta_j)$，所以 `atan2(β, α)` 得到的角度需减去 $\theta_j$ 偏移。

#### B.2 实现 `extract_1d_coefficients()`

```cpp
// analytical_coeff.h (新文件)
struct Coeff1D {
    double alpha[3];  // per xyz
    double beta[3];
    double gamma[3];
};

inline Coeff1D extract_1d_coefficients(
    const Eigen::Matrix4d& prefix,  // T_0...T_{j-1}
    const Eigen::Matrix4d& suffix,  // T_{j+1}...T_n * [tool]
    const DHParam& dh_j)
{
    Coeff1D c;
    Eigen::Vector4d r = suffix.col(3);  // suffix position column

    double ca = std::cos(dh_j.alpha);
    double sa = std::sin(dh_j.alpha);

    // T_j 的列分解（以 c_j, s_j, 常数 为系数）
    // 列 [c_j 系数, s_j 系数, 常数] → 乘以 prefix 得到 xyz 对应行
    for (int d = 0; d < 3; ++d) {
        double L0 = prefix(d, 0), L1 = prefix(d, 1), L2 = prefix(d, 2), L3 = prefix(d, 3);

        // c_j 贡献 = L0*(r0 + a_j) + L1*(r1*ca - r2*sa)
        c.alpha[d] = L0 * (r[0] + dh_j.a) + L1 * (r[1] * ca - r[2] * sa);

        // s_j 贡献 = L1*(r0 + a_j) - L0*(r1*ca - r2*sa)
        c.beta[d]  = L1 * (r[0] + dh_j.a) - L0 * (r[1] * ca - r[2] * sa);

        // 常数 = L2*(r1*sa + r2*ca + d_j) + L3
        c.gamma[d] = L2 * (r[1] * sa + r[2] * ca + dh_j.d) + L3;
    }

    return c;
}
```

> 实际公式需严格按 DH 矩阵展开验证，上述为示意。实现时需写单元测试与 3-sample QR 交叉验证。

#### B.3 修改 `solve_edges()` 增加 direct 路径

```cpp
// solve_edges() 中，在当前 QR 拟合路径之前加入选择分支：
if (config.enable_p1_direct_coeff) {
    // 计算 prefix = T_0...T_{j-1}
    // 计算 suffix = T_{j+1}...T_n * T_tool
    // extract_1d_coefficients(prefix, suffix, dh_j)
    // → 直接获得 α, β, γ，无需 3 次 FK 采样
} else {
    // 当前 QR 路径（保留作为回退/验证）
}
```

**关键差异**：当前 P1 遍历 `max_bg` 个 background 组合，每个需 3 次 FK。direct 路径下，每个 background 仍需计算 prefix/suffix（1 次前向、1 次后向链乘），但省去了 3 次 FK → **净省 3 次 FK 变为 0 次 FK + 2 次部分链乘**。

#### B.4 FKWorkspace 扩展

当前 `FKWorkspace` 只有前向链。需新增 **suffix 链** 支持：

```cpp
struct FKWorkspace {
    Eigen::Matrix4d tf[16];        // prefix chain (existing)
    Eigen::Matrix4d suffix[16];    // NEW: suffix chain (从末端向前)
    int np = 0;

    // 计算 suffix[j] = T_{j+1}...T_n * T_tool （从末端向前递推）
    void compute_suffix(const Robot& robot, const Eigen::VectorXd& q);
};
```

suffix 链可在**每个 background 切换时**一次性计算（$O(n)$ 次 4×4 矩阵乘法），然后对所有 free joint $j$ 复用 — 实际上若 background 不含 $j$，prefix/suffix 在同一 background 内对不同 $j$ 有共享前缀。

#### B.5 Config 扩展

```cpp
struct AnalyticalCriticalConfig {
    ...
    // Phase B: P1 direct coefficient extraction (bypass 3-point sampling + QR)
    bool enable_p1_direct_coeff = false;  // default OFF, A/B 测试后开启
};
```

#### B.6 单元测试

- `test_extract_1d_coefficients()`：随机 DH 参数 / 随机 prefix/suffix，验证提取的 $(\alpha, \beta, \gamma)$ 与 3-point QR 一致 (`1e-12`)。
- `test_p1_direct_vs_qr()`：两条路径产出 AABB 一致 (`1e-6`)。
- `test_p1_direct_fk_savings()`：验证 `n_phase1_fk_calls` 下降到 0。

#### B.7 A/B 实验 (O6.1)

- baseline: `enable_p1_direct_coeff=false` (current QR path)
- trial: `enable_p1_direct_coeff=true`
- 指标：`n_phase1_fk_calls` (应 → 0)、`mean_ms_delta_pct`、`fk_total_delta_pct`

**验收**：FK 下降如预期、AABB 一致、性能中性或改善。

**预计改动量**：~150 行 C++ (coeff.h + solve_edges 修改) + ~80 行测试
**预计工期**：1 天

---

### Phase C：P2 DH 链系数直接提取（消除 P2 采样 FK）

**目标**：P2 (2D face) 目前每组 (face-pair × background) 需 9 次 FK 采样。改为从 DH 链直接推导 9 个双线性系数。

#### C.1 数学推导

$$
p_d(q_i, q_j) = \text{Prefix}_i \cdot T_i(q_i) \cdot \text{Middle}_{ij} \cdot T_j(q_j) \cdot \text{Suffix}_j \cdot e
$$

其中 $e = [0,0,0,1]^T$，$\text{Middle}_{ij} = T_{i+1}(q_{i+1}) \cdots T_{j-1}(q_{j-1})$（背景关节已固定），$\text{Prefix}_i = T_0 \cdots T_{i-1}$，$\text{Suffix}_j = T_{j+1} \cdots T_n$。

令：
- $\mathbf{R} = \text{Suffix}_j \cdot e = [r_0, r_1, r_2, 1]^T$
- $\mathbf{M} = \text{Middle}_{ij}$
- $\mathbf{P} = \text{Prefix}_i$

$T_j(q_j) \cdot \mathbf{R}$ 线性于 $(c_j, s_j, 1)$，记为 $\mathbf{v}_j = c_j \cdot \mathbf{u}_c + s_j \cdot \mathbf{u}_s + \mathbf{u}_0$（4D 向量）。

$\mathbf{M} \cdot \mathbf{v}_j$ 仍线性于 $(c_j, s_j, 1)$，记为 $c_j \mathbf{w}_c + s_j \mathbf{w}_s + \mathbf{w}_0$。

$T_i(q_i) \cdot (\cdot)$ 展开后线性于 $(c_i, s_i, 1)$，与 $(c_j, s_j, 1)$ 组合产生 $3 \times 3 = 9$ 个系数：

$$
p_d = a_1 c_i c_j + a_2 c_i s_j + a_3 s_i c_j + a_4 s_i s_j + a_5 c_i + a_6 s_i + a_7 c_j + a_8 s_j + a_9
$$

每个 $a_k$ 是 (prefix 行, DH_i 参数, middle, DH_j 参数, suffix) 的代数组合。展开公式虽然较长但结构清晰，可用 CAS 验证后实现。

#### C.2 实现 `extract_2d_coefficients()`

```cpp
// analytical_coeff.h
struct Coeff2D {
    double a[9][3];  // a[k][d] = 第 k 项系数在 d 轴上的值
};

inline Coeff2D extract_2d_coefficients(
    const Eigen::Matrix4d& prefix_i,
    const DHParam& dh_i,
    const Eigen::Matrix4d& middle_ij,
    const DHParam& dh_j,
    const Eigen::Matrix4d& suffix_j)
{
    Coeff2D c;
    // Step 1: suffix_j 作用 → R = suffix_j.col(3)
    // Step 2: T_j 分解为 (c_j, s_j, 1) 的线性组合
    // Step 3: middle_ij 乘入
    // Step 4: T_i 分解为 (c_i, s_i, 1)
    // Step 5: prefix_i 乘入
    // Step 6: 提取 9 个系数到 a[k][d]
    ...
    return c;
}
```

#### C.3 修改 `solve_faces()` 增加 direct 路径

```cpp
if (config.enable_p2_direct_coeff) {
    // 计算 prefix_i = T_0...T_{i-1}
    // 计算 middle_ij = T_{i+1}...T_{j-1} (背景关节已设好)
    // 计算 suffix_j = T_{j+1}...T_n
    // extract_2d_coefficients() → 直接获得 a[0..8][d]
    // 不需要 9 次 FK 采样，也不需要 QR 分解
    // 直接进入 build_symbolic_poly8 + solve_poly_in_interval
} else {
    // 当前 QR 路径
}
```

**FKWorkspace 需要的扩展**：需要能高效计算任意区间 $[i..j]$ 的链乘积。可用前缀积 + 后缀积实现。

#### C.4 与 Phase A (Hybrid) 的关系

Phase A 的 hybrid 路径仍需要 9 个 FK 采样（用于 `compute_p2_symbolic_coeff_from_grid`）。Phase C 则**完全消除 FK 采样**。两者可以共存：

- `enable_p2_derivative_hybrid`: 用 symbolic coeff 但仍然采样（Phase A）
- `enable_p2_direct_coeff`: 完全跳过采样，从 DH 链直接提取（Phase C）

Phase C 是 Phase A 的**上位替代**。验证通过后，Phase A 成为冗余路径。

#### C.5 Config 扩展

```cpp
struct AnalyticalCriticalConfig {
    ...
    // Phase C: P2 direct coefficient extraction
    bool enable_p2_direct_coeff = false;
    // 当 enable_p2_direct_coeff=true 时，也可以保持 QR 路径做 compare-only 验证
    bool enable_p2_direct_coeff_verify = false;
    double p2_direct_coeff_verify_max_abs = 1e-10;
};
```

#### C.6 单元测试

- `test_extract_2d_coefficients()`：随机 DH / prefix / middle / suffix，与 9-sample QR 交叉验证 (`1e-12`)。
- `test_p2_direct_vs_qr()`：AABB 一致 (`1e-6`)。
- `test_p2_direct_fk_savings()`：`n_phase2_fk_calls` 显著下降。
- `test_p2_direct_with_verify()`：verify 模式下检测一致性。

#### C.7 A/B 实验 (O6.2)

- baseline: QR path
- trial: `enable_p2_direct_coeff=true`
- 指标：`n_phase2_fk_calls` (应 → 0 或接近 0)、`mean_ms_delta_pct`

**预计改动量**：~250 行 C++ + ~100 行测试
**预计工期**：2 天

---

### Phase D：P2.5 DH 链系数直接提取

**目标**：将 P2.5a (pair-1D) 和 P2.5b (pair-2D) 的 QR 拟合路径替代为 DH 直接提取。

#### D.1 P2.5a (Pair-Constrained 1D)

P2.5a 的模型与 P1 完全相同（$\alpha c + \beta s + \gamma$ 3 系数），但自由变量受约束 $q_i + q_j = C$。

direct 路径与 Phase B 的 `extract_1d_coefficients()` 相同：
- prefix = $T_0 \cdots T_{i-1}$
- suffix = $T_{i+1} \cdots T_{j-1} \cdot T_j(C - q_i) \cdot T_{j+1} \cdots T_n$

**注意**：suffix 中包含了约束关节 $j$（其角度被 $C - q_i$ 耦合），但由于模型对 $q_i$ 线性，suffix 中 $T_j$ 的 $q_j$ 依赖需特殊处理——实际上约束使得 $q_j = C - q_i$，所以 $T_j$ 也是 $q_i$ 的函数。这意味着 P2.5a 的 suffix 不再是常数矩阵，而是包含了 $q_j(q_i)$ 的耦合。

正确方法：
- 模型实际上为 $p_d = \alpha' \cos q_i + \beta' \sin q_i + \gamma' \cos(C - q_i) + \delta' \sin(C - q_i) + \epsilon'$
- 利用和差化积：$\cos(C - q_i) = \cos C \cos q_i + \sin C \sin q_i$，化为 3 系数形式
- 因此仍可用 `extract_1d_coefficients()` 的框架，只需将约束关节对的贡献合并

**预计改动量**：~60 行 C++ 增量（复用 Phase B 函数）
**预计工期**：0.5 天

#### D.2 P2.5b (Pair-Constrained 2D)

P2.5b 在约束 $q_i + q_j = C$ 下，以 $q_i$ 和某自由 $q_m$ 为 2 个自由变量，模型结构与 P2 相同（9 项双线性）。

direct 路径与 Phase C 的 `extract_2d_coefficients()` 相同，但需处理约束耦合：
- free variables: $q_i$（受约束）和 $q_m$（自由）
- prefix: $T_0 \cdots T_{\min(i,m)-1}$
- middle: 链内段（含被约束的 $q_j = C - q_i$）
- suffix: 链尾段

约束使得 middle 包含 $q_i$ 的依赖 — 需将 $T_j(C - q_i)$ 展开为 $q_i$ 的三角函数，与 $T_i(q_i)$ 合并。这需要额外的代数推导，但仍然可以化为 9 系数双线性模型。

**预计改动量**：~120 行 C++ 增量
**预计工期**：1 天

#### D.3 A/B 实验 (O6.3)

- 指标：`n_phase25_fk_calls`、`mean_ms_delta_pct`

---

### Phase E：清理冗余 + 设置默认开启

**目标**：在 Phase A–D 全部验证通过后，将 direct 路径设置为默认，并清理/简化代码。

#### E.1 默认值切换

```cpp
bool enable_p1_direct_coeff = true;   // ← 改为默认 ON
bool enable_p2_direct_coeff = true;   // ← 改为默认 ON
```

保留 `enable_pX_direct_coeff_verify = false` 作为调试开关。

#### E.2 移除冗余采样点

当 direct 路径为默认且所有测试通过后：
- P1: 移除 `qvals[3]` 采样、`A_basis` 构建、`qr_A.solve()` 调用
- P2: 移除 `A_mat` 构建、9 次 FK 调用、`qr.solve()` 调用
- P2.5: 同上

保留 verify 路径（gated by compile-time flag 或 debug config），不进入 release 热路径。

#### E.3 Proto/Shadow 清理

Phase A 的 hybrid 路径、proto metrics、shadow run 在 direct 路径落地后成为冗余：
- `enable_p2_derivative_prototype` → 仅在 verify 模式下使用
- `enable_p2_derivative_shadow_run` → 可删除
- P2ProtoMetrics 大部分字段 → 简化或移入 debug-only 结构

#### E.4 Config 简化

```cpp
// 清理后的 Config（Phase E 完成后）
struct AnalyticalCriticalConfig {
    ...
    // 删除:
    // - enable_p2_derivative_prototype
    // - enable_p2_derivative_shadow_run
    // - enable_p2_shadow_all_qi_branches
    // - enable_p2_derivative_hybrid
    // - enable_p2_hybrid_fallback
    // - p2_hybrid_max_abs_error
    // - p2_proto_gate_*
    // 保留:
    bool enable_p1_direct_coeff = true;
    bool enable_p2_direct_coeff = true;
    bool enable_direct_coeff_verify = false;  // debug only
};
```

**预计改动量**：~200 行删除 + ~50 行重构
**预计工期**：1 天

---

### Phase F：P3 Interior 改进（bonus）

**非必须，但可以受益**。

P3 (coordinate-wise sweep) 不使用 QR 拟合 — 它基于 coordinate-descent，每步对一个关节做 1D atan2 求解。当前实现每步调用 `eval_and_update_from()`（完整 FK from free joint）。

Phase B 实现的 `extract_1d_coefficients()` 也可用于 P3 的 1D sweep：
- 每步固定其他关节后，对 free joint $j$ 直接提取 $(\alpha, \beta, \gamma)$
- 跳过 FK eval，直接 atan2 → 候选角度 → 仅对候选做 FK eval

收益有限（P3 FK 占比通常 <15%），但实现成本极低（复用 Phase B 基础设施）。

---

## 四、预期 FK 削减量分析

以 Panda 7-DOF 机械臂、`n_sub=4`、10 个 width 为例（基于 O5.7 benchmark 数据）：

| 阶段 | 当前 FK/次 | 采样占比 | Direct 后 FK | 削减 |
|------|-----------|----------|-------------|------|
| P0 | ~500-2000 | 0% (枚举) | 不变 | 0% |
| P1 | 3 × O(nj × 2^bg) | **100%** | ~0 | **~100%** |
| P2 | 9 × O(pairs × 2^bg) | **~90%** | ~0 (仅候选 eval) | **~90%** |
| P2.5a | 3 × O(pairs × bg) | **100%** | ~0 | **~100%** |
| P2.5b | 9 × O(pairs × bg) | **~90%** | ~0 | **~90%** |
| P3 | 变化 | ~0% | 不变 | 0% |

**总体预估**：FK 总调用量降低 **40–60%**（取决于 P2+P2.5 的 candidate eval 占比）。

> 注意：消除采样 FK 后的性能收益取决于 FK 计算在总时间中的比重。如果 polynomial root-finding 和 candidate evaluation 占主导，则 FK 削减的时间影响可能小于 FK 占比暗示的比例。A/B 实验将给出实际加速比。

---

## 五、依赖关系与执行顺序

```
Phase A (P2 Hybrid 写回)           ← 信心基础            [0.5 天]
    │
    ├── A/B O5.8 验证
    │       │
    ▼       ▼
Phase B (P1 Direct)               ← 独立于 A            [1 天]
    │
    ├── A/B O6.1 验证
    │
Phase C (P2 Direct)               ← 依赖 B (共享 FKWorkspace 扩展)  [2 天]
    │
    ├── A/B O6.2 验证
    │
Phase D (P2.5 Direct)             ← 依赖 B+C (复用 extract_1d/2d)   [1.5 天]
    │
    ├── A/B O6.3 验证
    │
Phase E (清理 + 默认开启)          ← 依赖 A+B+C+D 全部 GO            [1 天]
    │
Phase F (P3 bonus, optional)       ← 依赖 B                          [0.5 天]
```

**总预计工期**：~6.5 天（不含 Phase F）

---

## 六、风险评估与缓解

| 风险 | 等级 | 缓解方案 |
|------|------|----------|
| DH 链推导公式错误 | 中 | 每阶段必须有 vs-QR 交叉验证测试；verify 模式持续检查 |
| 数值精度问题（prefix/suffix 矩阵条件数差） | 中 | 保留 QR 回退路径（gated by config）；在 verify 模式下输出条件数统计 |
| P2.5 约束耦合推导复杂 | 中 | P2.5a 可用 Phase B 框架直接处理；P2.5b 的耦合展开需 CAS 辅助 |
| FK 削减后性能未改善（root-finding 成为新瓶颈） | 低 | A/B 实验将量化实际加速比；若不足预期，Phase E 仍可合并但不设默认 |
| parallel-by-link 与 direct 路径的交互 | 低 | FKWorkspace suffix 扩展需确保线程安全（每线程一份） |

---

## 七、Config 参数总览（全计划完成后）

```cpp
// 最终态 Config（Phase E 完成后的目标状态）
struct AnalyticalCriticalConfig {
    // Phase 0-3: 现有控制参数不变
    bool keep_kpi2_baseline = true;
    bool enable_edge_solve = true;
    bool enable_face_solve = true;
    bool face_all_pairs = false;
    bool enable_pair_1d = true;
    bool enable_pair_2d = true;
    int  pair_max_bg = 64;
    bool pair_kpi2_backgrounds = true;
    bool pair_all_pairs = false;
    bool enable_interior_solve = true;
    int  interior_max_free = 7;
    int  interior_max_sweeps = 3;
    bool improved_interior = true;
    int  interior_n_restarts = 4;
    bool interior_pair_coupling = true;
    bool dual_phase3 = false;

    // 优化控制
    bool enable_aa_pruning = true;
    bool enable_candidate_dedup = true;
    bool enable_adaptive_candidate_dedup = false;
    int  candidate_dedup_min_candidates = 4;
    int  candidate_dedup_warmup_sets = 32;
    double candidate_dedup_min_hit_rate = 0.02;
    bool enable_parallel_by_link = false;
    int  parallel_num_threads = 0;

    // DH 链直接系数提取（替代 FK 采样 + QR）
    bool enable_p1_direct_coeff = true;   // Phase B
    bool enable_p2_direct_coeff = true;   // Phase C
    // P2.5 自动跟随对应的 1D/2D 开关

    // Debug/验证
    bool enable_direct_coeff_verify = false;
    double direct_coeff_verify_max_abs = 1e-10;

    // Multi-qi (保留，独立功能)
    bool enable_p2_multi_qi_candidates = false;
    int  p2_multi_qi_min_root_count = 1;
    double p2_multi_qi_min_joint_width = 0.0;
};
```

---

## 八、Benchmark / 统计列扩展

Phase A–D 各增加的 stats 字段：

| 字段 | 说明 |
|------|------|
| `n_p1_direct_coeff_cases` | P1 direct 路径提取次数 |
| `n_p2_direct_coeff_cases` | P2 direct 路径提取次数 |
| `n_p25a_direct_coeff_cases` | P2.5a direct 路径提取次数 |
| `n_p25b_direct_coeff_cases` | P2.5b direct 路径提取次数 |
| `n_direct_verify_pass` | verify 交叉验证通过次数 |
| `n_direct_verify_fail` | verify 交叉验证失败次数（应为 0） |
| `max_direct_verify_error` | verify 最大系数偏差 |

---

## 九、关键里程碑检查点

| 检查点 | 条件 | 决策 | 状态 |
|--------|------|------|------|
| A 完成 | hybrid 写回工作、统计非零、AABB 一致 | GO → B+C+D | ✅ DONE |
| O5.8 通过 | N=10 A/B 验证稳定 | 确认 symbolic 系数可信 | ✅ DONE |
| B 完成 | P1 FK→0、AABB 一致 | GO → 继续 C | ✅ DONE (FK 75%↓) |
| O6.1 通过 | P1 direct 性能中性/改善 | 确认 prefix/suffix 架构可行 | ✅ DONE |
| C 完成 | P2 FK 大幅下降、AABB 一致 | GO → 继续 D | ✅ DONE (FK 98.4%↓) |
| O6.2 通过 | P2 direct 性能改善 | **核心里程碑：解析替代可行** | ✅ DONE |
| D 完成 | P2.5 FK 下降、AABB 一致 | GO → E | ✅ DONE (FK 92.9%↓) |
| E 完成 | 默认开启、冗余清理 | **计划完成** | ✅ DONE (E.1 默认ON, 49P/0F + 172P/0F) |

---

## 十、与现有 O5 系列的衔接

| 新阶段 | 对应 O 系列 | 前置 |
|--------|------------|------|
| Phase A | O5.8 (hybrid 写回) | O5.7 confirmed go |
| Phase B | O6.1 (P1 direct) | Phase A GO |
| Phase C | O6.2 (P2 direct) | Phase B GO |
| Phase D | O6.3 (P2.5 direct) | Phase C GO |
| Phase E | O6.4 (默认切换) | A+B+C+D all GO |
