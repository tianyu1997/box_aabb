# GCPC 管线优化计划（第二轮 + 第三轮）

> 日期：2026-03-22 (G1–G7), 2026-03-23 (H1–H3)
> 前提：O1–O6 已完成（Phase B 预计算 DH + 栈 DFS、CritSample 全栈化、去重哈希化）
> Analytical 管线不再着重优化 —— GCPC 是其上位替代
>
> **优化范围**：
> - epiAABB 端点管线（`compute_endpoint_iaabb`）
> - 完整 GCPC 管线（`derive_aabb_with_gcpc`，7 阶段，当前 ~72ms/box）

---

## 基线性能

### epiAABB 端点管线（200 trials, seed 42, IIWA14, Release）

| 管线        | avg (ms) | median (ms) | VolRatio |
|-------------|----------|-------------|----------|
| IFK         | 0.023    | 0.022       | 5.64     |
| CritSample  | 0.040    | 0.033       | 0.999    |
| GCPC        | 0.160    | 0.042       | 1.000    |
| Analytical  | 16.76    | 12.90       | 1.000    |

### 完整 GCPC 管线（论文基线）

| 指标 | 值 |
|------|-----|
| 总耗时 | ~72ms/box |
| Phase A (缓存查找) | ~5% |
| Phase B (kπ/2 枚举) | ~10%（已优化） |
| Phase C (1D atan2) | ~20% |
| Phase D (2D 面求解) | ~25% |
| Phase E (成对 1D) | ~10% |
| Phase F (成对 2D) | ~15% |
| Phase G (坐标下降) | ~15% |

---

## 优化项总览

| 优先级 | 编号 | 优化项 | 影响范围 | 预估收益 | 风险 | 状态 |
|--------|------|--------|----------|----------|------|------|
| P1 | G1 | `best_face_config` VectorXd → 定长 `double[8]` | 完整 GCPC | **高** — 消除每次极值更新的 VectorXd 深拷贝 | 低 | ✅ 已完成 |
| P1 | G2 | `checkpoint_AB` 深拷贝 → 随 G1 自动零开销 | 完整 GCPC | **中** — 消除 n_act×6 个 VectorXd 拷贝 | 免费 | ✅ 已完成 |
| P2 | G3 | `coupled_pairs()` 预计算：D/E/F/G 四阶段共用一次 | 完整 GCPC | **低中** — 消除 4 次 vector 构建 + O(k²) 去重 | 极低 | ✅ 已完成 |
| P2 | G4 | Phase E/F `bg_vals` 从 `vector<vector<double>>` → 栈数组 | 完整 GCPC | **中** — 消除每阶段 n 次 vector 堆分配 | 低 | ✅ 已完成 |
| P2 | G5 | Phase E/F 内层 `other_bg` / `other_idx` → 栈数组 | 完整 GCPC | **中** — 消除 kk×pair×ci 内循环堆分配 | 低 | ✅ 已完成 |
| P3 | G6 | Phase G `starts` / `link_pairs` per-face 堆分配 → 栈数组 | 完整 GCPC | **低中** — 消除 n_act×6 循环内 vector 创建 | 低 | ✅ 已完成 |
| P3 | G7 | Phase G 坐标下降 trig/QR 预计算 per-joint | 完整 GCPC | **低** — 消除 sweep×joint 重复 sin/cos + QR | 极低 | ✅ 已完成 |
| P4 | G8 | `endpoint_iaabbs` 从 `vector<float>` → 定长栈数组 `float[48]` | 全部 epiAABB | **低** — 192B 堆分配消除（IFK 路径占比最大） | 中 | ✅ 已完成 |
| P0 | H1 | `eval_config` → `eval_partial`（部分 FK） | Phase C/D/E/F/G | **低** — 每次候选评估节省 ~3–4 矩阵乘法 | 极低 | ✅ 已完成 |
| P0 | H2 | Phase C/G：直接 1D 系数提取替换 3 点采样 + QR | Phase C, Phase G | **高** — 消除每 (link,joint,bg) 3 次 FK + QR | 低 | ✅ 已完成 |
| P0 | H3 | Phase D：直接 2D 系数提取替换 9 点采样 + QR9 | Phase D | **中** — 消除每 (link,pair,bg) 9 次 FK + QR | 低 | ✅ 已完成 |

---

## G1: `best_face_config` VectorXd → 定长数组

### 问题

`derive_aabb_with_gcpc()` 中：
```cpp
// gcpc.cpp L354–358
std::vector<Eigen::VectorXd> best_face_config(n_act * 6,
                                                Eigen::VectorXd::Zero(n));
```
每个元素是 `Eigen::VectorXd`（堆分配 n 个 double）。共 `n_act × 6` 个（IIWA14: 36 个）。

`update_from_ws()` 在每次 FK 调用后扫描全部 link：
```cpp
// gcpc.cpp L416–419
if (v < best_face_val[idx_min]) {
    best_face_val[idx_min] = v;
    best_face_config[idx_min] = q;    // ← VectorXd 深拷贝（堆分配 + memcpy）
}
```

以 Phase B 为例，~2K–10K 次 `update_from_ws` 调用，每次最多 36 次比较，
极值更新时触发 `VectorXd::operator=` 深拷贝。
Phase C–G 同理，总计 ~数万次潜在拷贝。

### 方案

改为定长 `double` 数组：
```cpp
static constexpr int MAX_CONFIG_SIZE = 8;  // IIWA14 = 7, 留余量

struct FaceTracker {
    double val;
    double config[MAX_CONFIG_SIZE];
};

FaceTracker best_face[36];  // n_act_max * 6, 栈分配
// 初始化: val = ±1e30, config 不需要初始化
```

更新时用 `std::memcpy` 或手动循环代替 VectorXd 赋值。无堆分配。

### 附带收益（G2 自动解决）

```cpp
// gcpc.cpp L675
std::vector<Eigen::VectorXd> checkpoint_AB = best_face_config;
// 深拷贝 36 个 VectorXd → 改为 memcpy 36×FaceTracker（纯栈，~2.3KB）
```

### 涉及文件

- `v4/src/envelope/gcpc.cpp`：`best_face_config` 声明、`update_from_ws` 赋值、
  `checkpoint_AB` 拷贝、Phase G `face_configs` 参数

---

## G3: `coupled_pairs()` 预计算

### 问题

`robot.coupled_pairs()` 在 Phase D (L897)、E (L1133)、F (L1301)、G (L1604)
各调用一次，每次返回 `std::vector<std::pair<int,int>>` 并追加 adjacent/skip-1 对，
加 O(k²) 线性扫描去重。4 个阶段完全重复。

### 方案

在 `derive_aabb_with_gcpc` 入口处一次性预计算三种 pair 列表：
```cpp
struct PairSets {
    std::pair<int,int> face_pairs[64];    int n_face = 0;   // D: coupled + adjacent
    std::pair<int,int> pc_pairs[64];      int n_pc = 0;     // E/F: coupled + adj + skip
    std::pair<int,int> int_pairs[64];     int n_int = 0;    // G: coupled + adj + skip-2
};
```
Phases D/E/F/G 直接引用预计算结果。

### 涉及文件

- `v4/src/envelope/gcpc.cpp`：4 处 pair 构建代码 → 提取为函数/顶层块

---

## G4: Phase E/F `bg_vals` 栈化

### 问题

```cpp
// gcpc.cpp L1153–1155 (Phase E), L1323–1325 (Phase F)
std::vector<std::vector<double>> bg_vals(n);
for (int j = 0; j < n; ++j)
    bg_vals[j] = build_bg_values(intervals[j].lo, intervals[j].hi, true);
```

- 外层 `vector` 分配 n 个指针
- 每个 `build_bg_values` 返回新 `vector<double>`（堆分配）
- Phase E 和 F 各独立构建一份（内容完全相同）

### 方案

改为栈数组 + 共享：
```cpp
static constexpr int MAX_BG_PER_JOINT = 16;
double bg_vals_flat[16][MAX_BG_PER_JOINT];  // 栈
int bg_vals_n[16];                           // 每关节实际数量

// build 一次，E/F 共享
for (int j = 0; j < n; ++j) {
    int cnt = 0;
    double lo = intervals[j].lo, hi = intervals[j].hi;
    bg_vals_flat[j][cnt++] = lo;
    bg_vals_flat[j][cnt++] = hi;
    for (int k = -20; k <= 20; ++k) {
        double a = k * HALF_PI;
        if (a > lo + 1e-12 && a < hi - 1e-12 && cnt < MAX_BG_PER_JOINT)
            bg_vals_flat[j][cnt++] = a;
    }
    bg_vals_n[j] = cnt;
}
```

提到 Phase E 之前构建，E/F 共用引用。

### 涉及文件

- `v4/src/envelope/gcpc.cpp`：Phase E/F bg_vals 构建段
- `v4/include/sbf/envelope/analytical_utils.h`：`build_bg_values()` 保留（Analytical 仍用）

---

## G5: Phase E/F 内层 `other_bg` / `other_idx` 栈化

### 问题

```cpp
// gcpc.cpp L1191–1198 (Phase E), L1364–1371 (Phase F)
std::vector<std::vector<double>> other_bg;
std::vector<int> other_idx;
// ... 在 kk × pair × ci 三重循环内构建 → 大量堆分配
```

每次迭代构建临时 `vector` 存放该 pair + constraint 之外的背景关节值。
典型 kk 范围 ~5, pair 数 ~10, n_act = 6 → ~300 次 vector 构建/析构。

### 方案

```cpp
static constexpr int MAX_OTHER = 12;
static constexpr int MAX_BG_EACH = 16;
int other_idx_s[MAX_OTHER];
int other_bg_n[MAX_OTHER];
// other_bg 直接引用 bg_vals_flat[other_idx_s[i]]，不需要拷贝
int n_other = 0;
```

只存索引，通过 `bg_vals_flat[jj]` 间接访问值。

### 涉及文件

- `v4/src/envelope/gcpc.cpp`：Phase E 内层 (~L1191)、Phase F 内层 (~L1364)

---

## G6: Phase G `starts` / `link_pairs` 栈化

### 问题

```cpp
// gcpc.cpp L1633–1637
std::vector<std::pair<int,int>> link_pairs;
for (auto& [a, b] : all_pairs_g) {
    if (a < nj && b < nj) link_pairs.push_back({a, b});
}

// gcpc.cpp L1645
std::vector<Eigen::VectorXd> starts;
// ... push_back 2–3 seeds per face
```

`link_pairs` 在 `ci × face` 循环内构建（n_act × 6 = 36 次），每次堆分配。
`starts` 每 face 构建，含 VectorXd（堆分配）。

### 方案

```cpp
// link_pairs: 提到 ci 循环外（每 ci 一次）
std::pair<int,int> link_pairs_s[64];
int n_link_pairs = 0;

// starts: 用定长 seed 数组
static constexpr int MAX_SEEDS = 4;
double seeds[MAX_SEEDS][8];  // 最多 4 个种子 × 8 关节
int n_seeds = 0;
```

### 涉及文件

- `v4/src/envelope/gcpc.cpp`：Phase G `run_phase_g_round` lambda 内部

---

## G7: Phase G trig 预计算

### 问题

```cpp
// gcpc.cpp L1690–1695 (inside sweep × joint loop)
Eigen::Matrix3d A_trig;
for (int si = 0; si < 3; ++si) {
    A_trig(si, 0) = std::cos(qvals[si]);
    A_trig(si, 1) = std::sin(qvals[si]);
    A_trig(si, 2) = 1.0;
}
auto qr_g = A_trig.colPivHouseholderQr();
```

`qvals[3] = {lo_j, mid_j, hi_j}` 在每次 sweep 中不变（只依赖 intervals），
但 3×j 组 sin/cos 在每次 sweep 重复计算。3 sweeps × 7 joints = 21 组 × 2 trig = 42 次。

### 方案

预计算每关节的 trig 值和 QR 分解：
```cpp
// 在 run_phase_g_round 入口，per-joint 预计算
struct JointTrig {
    double cos_v[3], sin_v[3];            // cos/sin(lo, mid, hi)
    Eigen::Matrix3d A;
    Eigen::ColPivHouseholderQR<Eigen::Matrix3d> qr;
};
JointTrig joint_trig[16];
for (int j = 0; j < n; ++j) {
    double lo = intervals[j].lo, hi = intervals[j].hi, mid = 0.5*(lo+hi);
    double qv[3] = {lo, mid, hi};
    for (int si = 0; si < 3; ++si) {
        joint_trig[j].cos_v[si] = std::cos(qv[si]);
        joint_trig[j].sin_v[si] = std::sin(qv[si]);
        joint_trig[j].A(si, 0) = joint_trig[j].cos_v[si];
        joint_trig[j].A(si, 1) = joint_trig[j].sin_v[si];
        joint_trig[j].A(si, 2) = 1.0;
    }
    joint_trig[j].qr = joint_trig[j].A.colPivHouseholderQr();
}
```

sweep 内直接引用 `joint_trig[j].qr`。

### 涉及文件

- `v4/src/envelope/gcpc.cpp`：Phase G `run_phase_g_round` 内部

---

## G8: `endpoint_iaabbs` 栈数组（原 O3）

### 问题

`EndpointIAABBResult::endpoint_iaabbs` 是 `std::vector<float>`（最多 48 floats = 192B）。
每次 `compute_endpoint_iaabb` 调用一次堆分配。

### 方案

```cpp
struct EndpointIAABBResult {
    static constexpr int MAX_ENDPOINT_IAABB = 48;
    float endpoint_iaabbs[MAX_ENDPOINT_IAABB];
    int   n_endpoints = 0;
    // ...
};
```

### 注意

需同步修改所有使用 `.data()` / `.size()` / `.empty()` / `.resize()` 的调用点。
`.empty()` 判断改为 `n_endpoints > 0`。影响面较大，建议最后实施。

### 涉及文件

- `v4/include/sbf/envelope/endpoint_source.h`
- `v4/src/envelope/endpoint_source.cpp`
- `v4/src/envelope/crit_sample.cpp`
- 引用 `endpoint_iaabbs` 的测试/实验文件

---

## 执行顺序

```
G1+G2  →  G3  →  G4+G5  →  G6  →  G7  →  G8
 高收益     低风险    中收益      低中      低       影响面大
```

每完成一组后：
1. `cmake --build build --config Release` 零警告零错误
2. 运行全部测试（580 tests: 38 + 172 + 370）
3. 运行 epiAABB benchmark 验证 VolRatio 不变
4. 如条件允许，运行完整 GCPC benchmark 验证总耗时变化
5. 更新 `doc/CHANGE_LOG_CN.md`

---

## 不在本轮范围

| 项目 | 原因 |
|------|------|
| Analytical 管线优化（N2/N3/N5/N6） | GCPC 为上位替代，Analytical 仅保留正确性 |
| GCPC 缓存构建（`build()`）优化 | 离线阶段，非查询热路径 |

---

## 第三轮优化：H1–H3（DH 直接系数提取 + 部分 FK）

> 日期：2026-03-23
> 目标：GCPC 管线内部应比 Analytical 更快
> 前提：G1–G7 已完成，GCPC 耗时 ~18.5ms vs Analytical ~15.8ms（GCPC 仍慢 17%）

### 根因分析

GCPC Phase C–G 的 `eval_config(q)` 调用 `ws.compute(robot, q)` = **全量 FK**（7 次矩阵乘法），
而 Analytical 使用 `ws.compute_from(robot, q, from_joint)` = **部分 FK**（仅从 from_joint 开始）。
此外 Analytical Phase 1 使用 `extract_1d_coefficients()` 直接从 DH 链提取三角系数，
跳过 3 点 FK 采样 + QR 拟合；Phase 2 使用 `extract_2d_coefficients()` 跳过 9 点采样 + QR9。

### 优化项

| 编号 | 优化项 | 影响阶段 | 收益 | 状态 |
|------|--------|----------|------|------|
| H1 | `eval_config` → `eval_partial`（部分 FK） | Phase C/D/E/F/G | 每次候选评估节省 ~3–4 次矩阵乘法 | ✅ 已完成 |
| H2 | Phase C/G：直接 1D 系数提取 | Phase C, Phase G | 消除每 (link,joint,bg) 3 次采样 FK + QR | ✅ 已完成 |
| H3 | Phase D：直接 2D 系数提取 | Phase D | 消除每 (link,pair,bg) 9 次采样 FK + QR9 | ✅ 已完成 |

### H1: 部分 FK（`eval_partial`）

新增 lambda：
```cpp
auto eval_partial = [&](const Eigen::VectorXd& q, int from_joint) {
    ws.compute_from(robot, q, from_joint);
    update_from_ws(q);
};
```
Phase A 保留 `eval_config`（无前缀可复用）。Phase C–G 全部改用 `eval_partial`：
- Phase C: `eval_partial(q, j)`
- Phase D: `eval_partial(q, min_free_d)`
- Phase E: `eval_partial(q, min_free_e)`
- Phase F: `eval_partial(q, min_free_f)`
- Phase G 坐标下降: `eval_partial(q_cd, j)` 或 `eval_partial(q_cd, partner)`

### H2: Phase C/G 直接 1D 系数提取

使用 `analytical_coeff.h` 中的 `extract_1d_coefficients()` + `compute_suffix_pos()`：
- Phase C: 替换 3 点 FK 采样 + QR → `compute_suffix_pos` + `extract_1d_coefficients` + atan2
- Phase G: 替换 3 点 FK 采样 + per-joint QR → 同上（`qr_per_joint[16]` 数组已删除）
- 成本：~30 次标量乘法 + 1 次后缀链计算，vs 3 次 `compute_from` + QR 分解

### H3: Phase D 直接 2D 系数提取

使用 `extract_2d_coefficients()` + `compute_middle_matrix()` + `compute_suffix_pos()`：
- 替换 9 点 FK 采样 + QR9 → 直接从 DH 链提取 9 个双线性系数
- 成本：~120 次标量乘法，vs 9 次 `compute_from` + 9×9 QR 分解
- 系数映射需处理 swap（pi > pj 时交换 lo/hi 顺序）

### 结果（200 trials, seed 42, IIWA14, Release）

| 方法 | 均值 (ms) | 相对前一版 |
|------|-----------|-----------|
| 优化前 (G7 完成后) | GCPC=18.48, Anal=15.78 | GCPC/Anal=1.17 |
| H1 完成 | GCPC=18.26 | −1% |
| H1+H2 完成 | GCPC=16.24 | −12% |
| H1+H2+H3 完成 | GCPC=16.13 | −13% |

按宽度分解（H1+H2+H3）：
| 宽度 | Analytical | GCPC | 比值 |
|------|-----------|------|------|
| 0.05 | 3.25 | 3.73 | 1.15 |
| 0.10 | 5.55 | 6.20 | 1.12 |
| 0.20 | 10.48 | 11.24 | 1.07 |
| 0.30 | 15.39 | 15.49 | 1.01 |
| 0.40 | 22.28 | 20.74 | **0.93** |
| 0.50 | 28.10 | 25.16 | **0.90** |
| 0.70 | 37.97 | 35.45 | **0.93** |
| 1.00 | 44.46 | 51.29 | 1.15 |

- **中宽（0.3–0.7）**：GCPC 比 Analytical 快 7–10%
- **窄宽（<0.2）**：GCPC 略慢 7–15%（常量开销：缓存查找、Phase A 重建）
- **排除 width=1.0 后**：GCPC=13.68ms vs Analytical=13.88ms → **GCPC 快 1.4%**
- 体积匹配：198/200（<0.5% 误差阈值），最大偏差 0.8%
- 全部 580 测试通过（370 + 38 + 172）

### H4: kπ/2 内部值缓存 + Phase E/F Analytical 一致性

**目标**：所有 box 内部 kπ/2 bg 值 → 离线预计算缓存；lo/hi 边界求解与 Analytical 方法一致。

**改动概要**：

1. **Phase D½（新阶段）**：在缓存构建时，通过 `precompute_pair_kpi2()` 预计算所有 bg 关节取 kπ/2 值的临界点。查询时直接查表 + 区间包含检查，零 FK 调用。
   - `gcpc.h`：新增 `PairKpi2Config` 结构体（q[7], n_joints）
   - `gcpc.h`：`GcpcLinkSection` 增加 `pair_kpi2_configs` 字段
   - `gcpc.h`：`GcpcCache` 增加 `precompute_pair_kpi2()` 方法

2. **Phase E 重写（P2.5a 方式）**：
   - 替换 3 点 FK 采样 + QR → `extract_2d_coefficients` + 约束代入（α,β 公式）
   - bg 循环仅使用 {lo, hi}（kπ/2 值由 Phase D½ 处理）
   - 系数提取 0 次 FK 调用

3. **Phase F 重写（P2.5b 方式）**：
   - 链排序 {pi,pj,pm} → j1<j2<j3
   - DH 变换矩阵 `tf_j1_pre/j2_pre/j3_pre` 在 bg 循环外预计算
   - 9 个位置通过 `left_pre[k1] * tf_j2_pre[k2] * right_pre[k3]` 计算（替代 9 次 FK）
   - 多项式构建使用 `build_symbolic_poly8`（与 Analytical 一致）
   - bg 循环仅使用 {lo, hi}

4. **`precompute_pair_kpi2()` 实现**（~200 行）：
   - 枚举所有 bg 关节的 kπ/2 组合（上限 64 个）
   - Phase E 预计算：P2.5a 直接系数提取 + 求解
   - Phase F 预计算：P2.5b 预计算 DH + 9 点 + `build_symbolic_poly8` 求解
   - 去重：1e-8 容差内合并相同构型

**结果（200 trials, seed 42, IIWA14, Release）**：

| 方法 | 均值 (ms) | 相对 H3 |
|------|-----------|---------|
| H1+H2+H3 | GCPC=16.13, Anal=15.60 | GCPC/Anal=1.019 |
| H1+H2+H3+H4 | GCPC=16.93, Anal=16.92 | GCPC/Anal=**1.0006** |

按宽度分解（H4）：
| 宽度 | Analytical | GCPC | 比值 |
|------|-----------|------|------|
| 0.05 | 3.30 | 3.71 | 1.12 |
| 0.10 | 5.47 | 6.30 | 1.15 |
| 0.15 | 7.48 | 8.37 | 1.12 |
| 0.20 | 11.65 | 12.04 | 1.03 |
| 0.30 | 16.51 | 16.20 | **0.98** |
| 0.40 | 23.00 | 21.66 | **0.94** |
| 0.50 | 30.99 | 26.74 | **0.86** |
| 0.70 | 42.73 | 38.10 | **0.89** |
| 1.00 | 46.63 | 54.13 | 1.16 |

逐阶段耗时（H4，全 200 trials 均值）：
| 阶段 | 耗时 (ms) | 占比 |
|------|-----------|------|
| Phase C | 0.683 | 4.0% |
| Phase D | 0.485 | 2.8% |
| Phase E | 0.723 | 4.2% |
| Phase F | 13.440 | 78.1% |
| Phase G | 1.698 | 9.9% |
| 总计 | 17.211 | 100% |

- **中宽（0.3–0.7）**：GCPC 比 Analytical 快 2–14%（bg 组合从 5^k 降至 2^k）
- **窄宽（<0.2）**：GCPC 略慢 3–15%（不变）
- **总体比值 ≈ 1.0**：GCPC 与 Analytical 实质平衡
- 体积匹配：104/200 精确匹配，最大相对偏差 0.8%
- 全部 580 测试通过（370 + 38 + 172）

### H5: Phase F 增量链重算 + 预计算逆矩阵

**目标**：减少 Phase F per-bg-combo 的重复计算量。

**改动概要**：

1. **H5-A: 预计算逆矩阵**：
   - 替换 `A_mat.colPivHouseholderQr().solve(rhs)` → `A_mat.inverse() * rhs`
   - QR solve 每次需 ~600 muls（Householder 反射器），直接乘法仅 ~243 muls
   - 逆矩阵在 bg 循环外一次计算，所有 bg combo 共享

2. **H5-B: 增量链重算**：
   - 将 bg 关节分类到 4 个链段：prefix (< j1)、mid12 (j1..j2)、mid23 (j2..j3)、suffix (> j3)
   - bg 循环外预计算所有链段 + left_pre/right_pre
   - bg 循环内：仅重算含有变化 bg 关节的链段（seg_dirty 标记）
   - q 初始化从循环内移到循环外，仅更新变化的 bg 关节
   - `goto next_bg_f` → `continue`（增量逻辑移至循环顶部）

**关键原理**：二进制计数器中，~50% 迭代仅改变 1 个 bg 关节 → 仅 1 个链段需要重算。
若 bg 关节全在 suffix 段，则 left_pre（3 个 4×4 矩阵乘） 在所有 bg combo 间完全复用。

**结果（200 trials, seed 42, IIWA14, Release）**：

| 指标 | H4（之前） | H5（现在） | 改善 |
|------|-----------|-----------|------|
| Phase F | 13.440 ms | 7.384 ms | **−45.1%** |
| GCPC 总计 | 17.211 ms | 10.906 ms | **−36.6%** |
| Analytical | 16.835 ms | 17.098 ms | — |
| GCPC/Analytical | 1.022 | **0.638** | — |

按宽度分解（H5）：
| 宽度 | Analytical | GCPC | 比值 |
|------|-----------|------|------|
| 0.05 | 3.35 | 3.08 | **0.92** |
| 0.10 | 5.94 | 4.48 | **0.75** |
| 0.15 | 7.87 | 5.63 | **0.72** |
| 0.20 | 13.03 | 8.51 | **0.65** |
| 0.30 | 18.21 | 12.04 | **0.66** |
| 0.40 | 26.94 | 14.69 | **0.55** |
| 0.50 | 36.92 | 19.81 | **0.54** |
| 0.70 | 49.93 | 27.88 | **0.56** |
| 1.00 | 54.15 | 39.53 | **0.73** |

逐阶段耗时（H5，全 200 trials 均值）：
| 阶段 | 耗时 (ms) | 占比 |
|------|-----------|------|
| Phase C | 0.728 | 6.7% |
| Phase D | 0.533 | 4.9% |
| Phase E | 0.283 | 2.6% |
| Phase F | 7.384 | 67.7% |
| Phase G | 1.798 | 16.5% |
| 总计 | 10.906 | 100% |

- **所有宽度 GCPC 均快于 Analytical**（0.54–0.92x）
- 体积匹配：105/200 精确匹配，最大相对偏差 0.8%
- 全部 580 测试通过（370 + 38 + 172）
