# 计划：基于 URDF 网格的最优胶囊体拟合 — 替代 DH 参数几何

> 状态：计划阶段（未开始实现）
> 关联：`doc/AI_WORKING_SPEC.md` Rule 1, 6

---

## 1. 问题分析

### 1.1 当前模型：DH 导出的胶囊体

当前系统将每个活跃连杆建模为**胶囊体** = 线段 ⊕ B(r)：

```
胶囊体_ℓ = { x ∈ ℝ³ : dist(x, S_ℓ) ≤ r_ℓ }
```

其中：
- **线段 S_ℓ**：连接相邻 DH 帧原点（`prefix[V]` → `prefix[V+1]`）
- **半径 r_ℓ**：手工指定的 `link_radii[ci]`

线段长度由 DH 参数推导：

$$L_\ell = \sqrt{a_\ell^2 + d_\ell^2}$$

### 1.2 过估计来源

```
                    DH 帧 i               DH 帧 i+1
                       ●─────────────────────●           ← DH 线段（过长）
                       │   ╔═══════════╗     │
                       │   ║ 实际连杆  ║     │
                       │   ║  mesh     ║     │
                       │   ╚═══════════╝     │
                       │←─→│           │←───→│
                       过估计              过估计
                      (空区域)            (空区域)
```

**过估计两要素**：

| 来源 | 原因 | 量级 |
|------|------|------|
| **轴向过估计** | DH 帧原点位于关节轴交点处，而非连杆实体边界；d 参数包含关节间非实体段 | 可达 30-50% |
| **径向过估计** | 手工 radii 为保守值，未经实际 mesh 紧密拟合 | 可达 20-50% |

### 1.3 IIWA14 定量分析

| Link (ci) | DH 帧索引 | d (m) | a (m) | $L = \sqrt{a^2+d^2}$ | 手工 radius | 备注 |
|-----------|-----------|-------|-------|----------------------|-------------|------|
| 0 | 2→3 | 0.2025 | 0 | 0.2025 | 0.06 | 连杆 3：实际肘部粗约 50mm |
| 1 | 4→5 | 0.2155 | 0 | 0.2155 | 0.04 | 连杆 5：实际前臂粗约 35mm |
| 2 | 7→8(tool) | 0.081+0.185=0.266 | 0 | 0.266 | 0.03 | 含工具帧 |

**问题核心**：DH 的 d=0.2025 包含关节 2、3 之间的全部偏移，但连杆 3 的实际物理外壳可能只覆盖其中 ~60-70%。剩余空间是空气，但被计为碰撞体积。

### 1.4 影响链

```
过大胶囊 → 过大 endpoint iAABB → 过大 link iAABB / Hull16
         → 过大 envelope volume → forest box 间碰撞检测误报增多
         → GCS 图中可行边减少 → 规划质量下降
```

---

## 2. 算法设计

### 2.1 最小包围胶囊 (Minimum Enclosing Capsule, MEC) 算法

**输入**：连杆碰撞网格 $M = \{v_1, \dots, v_N\}$（顶点集）

**输出**：$(S^*, r^*)$ = 最小体积胶囊，满足 $M \subseteq S^* \oplus B(r^*)$

**两步法**：

#### Step 1：方向搜索（PCA + 优化）

1. 计算顶点集 $M$ 的协方差矩阵 → PCA 第一主成分 $\hat{d}_0$
2. 以 $\hat{d}_0$ 为初始方向，在球面 $S^2$ 上局部搜索最优方向 $\hat{d}^*$
3. 优化目标：$\min_{\hat{d}} \text{Vol}(\text{MEC}(\hat{d}, M))$

#### Step 2：给定方向的最优胶囊

给定轴方向 $\hat{d}$：

1. 投影所有顶点到轴上：$t_i = v_i \cdot \hat{d}$
2. 分解为轴向和径向分量：$v_i = t_i \hat{d} + \Delta_i$，其中 $\Delta_i \perp \hat{d}$
3. 胶囊段端点 $t_\min, t_\max$ 和径向半径 $r$ 的联合优化：

$$\text{对每个顶点 } v_i: \quad \text{dist}(v_i, S) \leq r$$

其中 $S = \{c + t \hat{d} : t \in [t_1, t_2]\}$, $c$ 为径向中心。

$\text{dist}(v_i, S) = \begin{cases} \|\Delta_i - c_\perp\| & \text{if } t_i \in [t_1, t_2] \\ \sqrt{\|\Delta_i - c_\perp\|^2 + (\text{clip}(t_i) - t_i)^2} & \text{otherwise} \end{cases}$

4. 最小化 $\text{Vol} = \pi r^2 |t_2 - t_1| + \tfrac{4}{3}\pi r^3$

**复杂度**：$O(N \log N)$ per direction × $O(K)$ directions ≈ 几毫秒/连杆。

### 2.2 腐蚀运算的等价解释

用户提到的"腐蚀运算"（Morphological Erosion）与 MEC 有如下对偶关系：

$$M \subseteq S \oplus B(r) \quad \Longleftrightarrow \quad M \ominus B(r) \supseteq S \text{ 的某个子集}$$

其中 $M \ominus B(r) = \{x : B(x, r) \subseteq M\}$ 为 Minkowski 差。

**实践意义**：
- 对网格 $M$ 做半径 $r$ 的腐蚀（向内收缩）
- 若腐蚀后剩余区域近似一维（像线段），则该线段就是胶囊的中轴
- 通过二分搜索 $r$：$r$ 太大 → 腐蚀后为空集；$r$ 刚好 → 腐蚀后为线段
- 这等价于寻找**最大可内切半径使得中轴为线段**

在实现中，我们推荐直接使用 **Step 1+2 的 MEC 算法**（更稳定），但在文档和论文中可以引用腐蚀运算作为直观解释。

### 2.3 从 MEC 胶囊到 DH 帧偏移量

MEC 输出的胶囊端点 $(p_1, p_2)$ 在 URDF 连杆坐标系中表示。需要转换为 DH 帧中的偏移量：

```
                DH frame_i     fitted proximal     fitted distal      DH frame_{i+1}
                   ●─ ─ ─ ─ ─ ─ ─ ●═══════════════════●─ ─ ─ ─ ─ ─ ─ ─●
                   │              │                    │                │
                   └── off_prox ──┘                    └── off_dist ────┘
                   (frame_i 中的常量偏               (frame_{i+1} 中的常量偏
                    移向量 [ox,oy,oz])                 移向量 [ox,oy,oz])
```

**转换步骤**：

1. 在 DH 参考位形（$q = 0$ 或零位）下计算所有 DH 帧的变换矩阵 $T_0^{(0)}, T_1^{(0)}, \dots$
2. 建立 URDF 帧 → DH 帧的映射 $R_\text{map}$（通过在零位下比较两种 FK 结果）
3. 对每个活跃连杆 $\ell$（DH 帧 $V$ → $V+1$）：
   - MEC 近端 $p_1^{\text{urdf}}$ → 转到 DH 帧 $V$ 局部坐标：$\text{off}_\text{prox} = (T_V^{(0)})^{-1} \cdot R_\text{map} \cdot p_1^{\text{urdf}}$
   - MEC 远端 $p_2^{\text{urdf}}$ → 转到 DH 帧 $V+1$ 局部坐标：$\text{off}_\text{dist} = (T_{V+1}^{(0)})^{-1} \cdot R_\text{map} \cdot p_2^{\text{urdf}}$
4. 偏移量 $\text{off}_\text{prox}$ 和 $\text{off}_\text{dist}$ 为**常量向量**，不随关节角变化

---

## 3. 数学基础

### 3.1 正确性证明：拟合胶囊 → 可证明保守的 iAABB

**命题**：若 MEC 胶囊 $(S^*, r^*)$ 满足 $M_\ell \subseteq S^* \oplus B(r^*)$，则以 $S^*$ 的端点偏移和 $r^*$ 计算的 endpoint iAABB，经 Minkowski 膨胀后，仍然保守地包围连杆 $\ell$ 在 box $B$ 中的可达体积。

**证明**：

1. 对任意 $q \in B$，连杆 $\ell$ 的物理体积 $\text{Vol}_\ell(q) \subseteq S_\ell(q) \oplus B(r^*)$
2. $S_\ell(q)$ 的端点为 $p_\text{prox}(q) = T_{0:V}(q) \cdot \text{off}_\text{prox}$ 和 $p_\text{dist}(q) = T_{0:V+1}(q) \cdot \text{off}_\text{dist}$
3. 由端点 iAABB 定义：$[\text{ep}]_B \supseteq \{p_\text{prox}(q) : q \in B\} \cup \{p_\text{dist}(q) : q \in B\}$
4. 胶囊内任意点 $x = \lambda p_\text{prox} + (1-\lambda) p_\text{dist} + \delta$（$\|\delta\| \leq r^*$, $\lambda \in [0,1]$）
5. 由凸性：$\lambda \cdot [\text{ep}_\text{prox}]_B + (1-\lambda) \cdot [\text{ep}_\text{dist}]_B \subseteq \text{conv}([\text{ep}]_B) \subseteq [\text{ep}]_B$
6. 加上 Minkowski 膨胀 $r^*$：完整包含。 ∎

### 3.2 iFK 中常量偏移的区间算术

当前（无偏移）：

```cpp
// extract_link_aabbs: 只取 prefix[V] 的平移列 — 列 3 (索引 [3], [7], [11])
out[0] = min(start_lo[3], end_lo[3]);   // 即 [T_V]_B 的 x 坐标区间下界
```

有偏移时：

```cpp
// 需要计算 [T_V]_B × off_prox = [R_V]_B × off + [t_V]_B
// 等价于 extract_ee_sphere_aabbs 中对 EESphere.center 的处理（已有实现！）
w_lo[i] = T_lo[i*4 + 3];   // 平移部分
w_hi[i] = T_hi[i*4 + 3];
for (int j = 0; j < 3; ++j) {
    double cj = off[j];          // 常量偏移分量
    if (abs(cj) < 1e-15) continue;
    double a = T_lo[i*4 + j];    // 旋转矩阵区间下界
    double b = T_hi[i*4 + j];    // 旋转矩阵区间上界
    if (cj >= 0) { w_lo[i] += a*cj; w_hi[i] += b*cj; }
    else         { w_lo[i] += b*cj; w_hi[i] += a*cj; }
}
```

**关键发现**：`extract_ee_sphere_aabbs`（interval_fk.cpp L150-180）**已经实现了完全相同的运算**！只需将此模式移植到 `extract_link_aabbs` 中。

### 3.3 区间包裹效应分析

偏移量 $\text{off} = [o_x, o_y, o_z]^T$ 是常量，与区间矩阵 $[R]$ 做乘法时：

$$[R \cdot \text{off}]_i = \sum_{j=1}^{3} [R_{ij}] \cdot o_j$$

这是**区间 × 标量**运算（每项精确），但求和会因区间相关性引入少量包裹过估。

**定量估计**：对于典型偏移 $\|\text{off}\| \leq 5$ cm，box 宽度 0.2 rad 时，附加包裹过估 < 0.5 mm（与连杆级 iAABB 尺度 50-200 mm 相比可忽略）。

**净效果**：`r_fit < r_manual` 和/或更短线段带来的体积缩减 >> 偏移包裹的边际增加。

### 3.4 体积缩减估算

以 IIWA14 连杆 3（ci=0）为例：
- 当前：轴长 = 0.2025m, r = 0.06m → 胶囊体积 = $\pi (0.06)^2 (0.2025) + \frac{4}{3}\pi (0.06)^3 = 2.29 \times 10^{-3} + 0.91 \times 10^{-3} = 3.2 \times 10^{-3}$ m³
- 估计拟合后：轴长 ≈ 0.14m, r ≈ 0.045m → 胶囊体积 = $\pi (0.045)^2 (0.14) + \frac{4}{3}\pi (0.045)^3 = 0.89 \times 10^{-3} + 0.38 \times 10^{-3} = 1.27 \times 10^{-3}$ m³
- **缩减约 60%**（单连杆）

综合所有活跃连杆：预期 envelope volume 缩减 **20-40%**。

---

## 4. 实现计划

### Phase 0：获取 URDF + 碰撞网格

| 任务 | 详情 |
|------|------|
| P0.1 | 下载 IIWA14 官方 URDF（`iiwa14.urdf` + STL/DAE mesh 文件），推荐来源：`kuka_experimental` ROS 包 或 Drake `manipulation_station` |
| P0.2 | 下载 Franka Panda 官方 URDF（`panda.urdf`），来源：`franka_ros` 或 Drake |
| P0.3 | 将 URDF + mesh 文件放入 `v4/robots/iiwa14/`, `v4/robots/panda/` |
| P0.4 | 验证 URDF 中的碰撞几何标签（`<collision>` → `<mesh filename="...">` 或 `<cylinder>/<capsule>`） |

**注意**：某些 URDF 的碰撞几何已经用简化凸包或原始体（cylinder / capsule），如果直接提供了碰撞 capsule，可以跳过 Phase 1 的 mesh 拟合，直接读取。

### Phase 1：URDF 解析 + MEC 拟合（Python 工具）

**新文件**：`scripts/fit_capsules.py`

```
用法: python scripts/fit_capsules.py robots/iiwa14/iiwa14.urdf -o robots/iiwa14/capsule_spec.json
```

功能模块：

| 步骤 | 实现 | 依赖 |
|------|------|------|
| 1.1 URDF 解析 | `urdfpy` 或 `yourdfpy` 库解析 URDF XML，提取连杆碰撞几何路径 + pose | `urdfpy` |
| 1.2 Mesh 加载 | `trimesh` 库加载 STL/DAE → 顶点数组 $V \in \mathbb{R}^{N \times 3}$ | `trimesh` |
| 1.3 坐标变换 | 将碰撞 mesh 顶点从 URDF 碰撞帧变换到连杆帧（应用 `<origin xyz rpy>` ） | numpy |
| 1.4 MEC 拟合 | 对每个连杆的顶点集执行最小包围胶囊算法（PCA + 方向搜索 + 1D 最优线段） | numpy, scipy |
| 1.5 DH 帧映射 | 在零位状态下计算 URDF FK 和 DH FK，建立帧对应关系，将 MEC 端点转换为 DH 帧内偏移 | numpy |
| 1.6 输出 | 生成 `capsule_spec.json`（JSON 格式，每个活跃连杆的偏移 + 半径） | json |
| 1.7 可视化 | 叠加绘图：原始 mesh / DH 胶囊 / 拟合胶囊（matplotlib + 3D scatter） | matplotlib |

MEC 核心算法伪代码：

```python
def fit_minimum_enclosing_capsule(vertices: np.ndarray) -> CapsuleSpec:
    """输入: Nx3 顶点数组. 输出: (p1, p2, radius)"""
    # Step 1: PCA 初始方向
    centroid = vertices.mean(axis=0)
    cov = np.cov((vertices - centroid).T)
    eigvals, eigvecs = np.linalg.eigh(cov)
    d0 = eigvecs[:, -1]  # 最大方差方向

    # Step 2: 方向优化（球面网格搜索 + 局部精化）
    best_dir, best_vol = d0, np.inf
    for d_candidate in sphere_grid_near(d0, n_samples=200):
        vol, p1, p2, r = compute_capsule_for_direction(vertices, d_candidate)
        if vol < best_vol:
            best_dir, best_vol = d_candidate, vol
            best_p1, best_p2, best_r = p1, p2, r

    # Step 3: Nelder-Mead 局部精化
    result = scipy.optimize.minimize(
        lambda angles: compute_capsule_for_direction(vertices, angles_to_dir(angles))[0],
        dir_to_angles(best_dir), method='Nelder-Mead')
    final_dir = angles_to_dir(result.x)
    _, p1, p2, r = compute_capsule_for_direction(vertices, final_dir)

    return CapsuleSpec(p1=p1, p2=p2, radius=r)

def compute_capsule_for_direction(vertices, direction):
    """给定方向,计算该方向上的最优胶囊"""
    d = direction / np.linalg.norm(direction)
    # 投影到轴
    projections = vertices @ d           # 轴向坐标
    perpendicular = vertices - np.outer(projections, d)  # 径向分量

    # 用 miniball 思想: 对每个线段端点候选, 计算最大距离
    t_min, t_max = projections.min(), projections.max()

    # 线段中心的最优径向偏移 (使最大径向距离最小)
    # → 2D 最小包围圆问题 (径向截面)
    perp_center = miniball_2d(perpendicular)  # 或简单用质心

    # 最大距离 → 半径
    dists = np.array([point_to_segment_dist(v, t_min*d + perp_center, t_max*d + perp_center)
                      for v in vertices])
    r = dists.max()

    p1 = t_min * d + perp_center
    p2 = t_max * d + perp_center
    vol = np.pi * r**2 * np.linalg.norm(p2 - p1) + (4/3) * np.pi * r**3
    return vol, p1, p2, r
```

### Phase 2：CapsuleSpec 数据结构（C++）

**新增/修改文件**：

| 文件 | 操作 | 内容 |
|------|------|------|
| `include/sbf/robot/capsule_spec.h` | 新建 | `CapsuleSpec` 结构体定义 |
| `src/robot/capsule_spec.cpp` | 新建 | JSON 序列化 + `from_json()` |

```cpp
// capsule_spec.h
#pragma once
#include <vector>
#include <string>

namespace sbf {

/// 单个活跃连杆的拟合胶囊参数。
/// 偏移量以连杆两端 DH 帧的局部坐标表示。
struct LinkCapsule {
    int    frame_index = -1;           // DH 帧索引 (= active_link_map[ci])
    double off_prox[3] = {0, 0, 0};   // 近端偏移 (在 DH frame_V 局部坐标中)
    double off_dist[3] = {0, 0, 0};   // 远端偏移 (在 DH frame_{V+1} 局部坐标中)
    double radius      = 0.0;         // 拟合半径 (m)
};

/// 完整机器人的胶囊规格。
/// 通过 Python 工具从 URDF 碰撞 mesh 拟合得到。
struct CapsuleSpec {
    std::string robot_name;
    int n_active = 0;
    std::vector<LinkCapsule> capsules;   // [n_active], 按 ci 索引

    // I/O
    void save_json(const std::string& path) const;
    static CapsuleSpec load_json(const std::string& path);

    // 查询：所有偏移是否为零（退化为 DH 原始模型）
    bool is_identity() const;
};

} // namespace sbf
```

**设计要点**：

- `off_prox = off_dist = [0,0,0]` 时退化为当前 DH 帧原点模型（完全向后兼容）
- `radius` 来自 MEC 拟合，替代手工 `link_radii`
- JSON 存储使其可在不重新编译的情况下更新

### Phase 3：IAABBConfig 集成

**修改文件**：`include/sbf/robot/iaabb_config.h`, `src/robot/iaabb_config.cpp`

```cpp
// 新增字段（IAABBConfig）:
struct IAABBConfig {
    // ... 已有字段 ...

    /// 可选：从 CapsuleSpec 拟合的端点偏移量。
    /// 长度 = n_active * 2 * 3 (近端3d + 远端3d per link)。
    /// 若为空则使用 DH 帧原点（off = [0,0,0]）。
    std::vector<double> endpoint_offsets;   // [n_active × 2 × 3], row-major

    /// 是否使用拟合偏移（非零偏移存在时为 true）。
    bool use_fitted_capsules = false;

    // 新增工厂方法
    static IAABBConfig from_robot(const Robot& robot,
                                  const CapsuleSpec* capsule_spec = nullptr,
                                  double threshold = 0.1);
};
```

### Phase 4：修改 iFK 端点提取（核心改动）

**修改文件**：`src/robot/interval_fk.cpp`

改前（`extract_link_aabbs`，L109-135）：

```cpp
// 只取 DH 帧原点的平移列
out[0] = min(start_lo[3], end_lo[3]);   // 无偏移
```

改后：

```cpp
void extract_link_aabbs_with_offset(
    const FKState& state,
    const int* active_link_map, int n_active_links,
    const double* endpoint_offsets,     // [n_active × 2 × 3] 或 nullptr
    const double* link_radii,
    float* out_aabb)
{
    for (int i = 0; i < n_active_links; ++i) {
        int li = active_link_map[i];
        float prox_lo[3], prox_hi[3], dist_lo[3], dist_hi[3];

        if (endpoint_offsets) {
            const double* off_p = endpoint_offsets + (i * 2) * 3;
            const double* off_d = endpoint_offsets + (i * 2 + 1) * 3;
            // 与 extract_ee_sphere_aabbs 相同的 [T]×offset 运算
            imat_apply_offset(state.prefix_lo[li], state.prefix_hi[li],
                              off_p, prox_lo, prox_hi);
            imat_apply_offset(state.prefix_lo[li+1], state.prefix_hi[li+1],
                              off_d, dist_lo, dist_hi);
        } else {
            // 退化：偏移 = 0, 直接取平移列
            for (int d = 0; d < 3; ++d) {
                int col = d * 4 + 3;
                prox_lo[d] = state.prefix_lo[li][col];
                prox_hi[d] = state.prefix_hi[li][col];
                dist_lo[d] = state.prefix_lo[li+1][col];
                dist_hi[d] = state.prefix_hi[li+1][col];
            }
        }

        float* out = out_aabb + i * 6;
        out[0] = min(prox_lo[0], dist_lo[0]);
        out[1] = min(prox_lo[1], dist_lo[1]);
        out[2] = min(prox_lo[2], dist_lo[2]);
        out[3] = max(prox_hi[0], dist_hi[0]);
        out[4] = max(prox_hi[1], dist_hi[1]);
        out[5] = max(prox_hi[2], dist_hi[2]);

        if (link_radii) {
            float r = link_radii[i];
            out[0] -= r; out[1] -= r; out[2] -= r;
            out[3] += r; out[4] += r; out[5] += r;
        }
    }
}

// 辅助函数：区间矩阵 × 常量偏移（复用 extract_ee_sphere_aabbs 的逻辑）
static inline void imat_apply_offset(
    const double T_lo[16], const double T_hi[16],
    const double off[3],
    float out_lo[3], float out_hi[3])
{
    for (int i = 0; i < 3; ++i) {
        double lo = T_lo[i * 4 + 3];
        double hi = T_hi[i * 4 + 3];
        for (int j = 0; j < 3; ++j) {
            double cj = off[j];
            if (std::abs(cj) < 1e-15) continue;
            double a = T_lo[i * 4 + j];
            double b = T_hi[i * 4 + j];
            if (cj >= 0.0) { lo += a * cj; hi += b * cj; }
            else           { lo += b * cj; hi += a * cj; }
        }
        out_lo[i] = static_cast<float>(lo);
        out_hi[i] = static_cast<float>(hi);
    }
}
```

**向后兼容**：`endpoint_offsets = nullptr` 时行为与原函数完全一致。

### Phase 5：修改所有 4 个 EndpointSource

| Source | 当前端点提取 | 需要的改动 |
|--------|-------------|-----------|
| **iFK** | `extract_link_aabbs` → 取 `prefix[V].col(3)` | → `extract_link_aabbs_with_offset` + 偏移参数 |
| **CritSample** | `update_endpoint_iaabb` → 取 `ws.tf[V].col(3)` | → 改为 `ws.tf[V] * off_prox` |
| **Analytical** | 类似 CritSample 的 FK 枚举 | → 同上模式 |
| **GCPC** | `fk_link_positions` → 取 FK 位置 | → FK 位置 + 偏移变换 |

**CritSample 改动**（`src/envelope/crit_sample.cpp`，`update_endpoint_iaabb` 函数）：

```cpp
// 改前:
const auto& col = ws.tf[V].col(3);  // DH 帧原点

// 改后:
Eigen::Vector4d p_local(off_prox[0], off_prox[1], off_prox[2], 1.0);
Eigen::Vector4d p_world = ws.tf[V] * p_local;  // TF × 偏移
const double x = p_world(0), y = p_world(1), z = p_world(2);
```

**GCPC 改动**（`src/envelope/gcpc.cpp`，所有 `fk_link_positions` 调用处）：

```cpp
// 改前:
auto pos = fk_link_positions(robot, q_full);
double px = pos[V].x(), py = pos[V].y(), pz = pos[V].z();

// 改后:
auto transforms = fk_transforms(robot, q_full);
Eigen::Vector4d p = transforms[V] * Eigen::Vector4d(off[0], off[1], off[2], 1.0);
double px = p(0), py = p(1), pz = p(2);
```

### Phase 6：Robot 构造集成

**修改文件**：`include/sbf/robot/robot.h`, `src/robot/robot.cpp`

```cpp
class Robot {
    // ... 已有 ...
    CapsuleSpec capsule_spec_;             // 可选
    std::vector<double> endpoint_offsets_; // 预计算的 [n_active × 2 × 3]

public:
    // 新增构造器重载 / setter
    void set_capsule_spec(const CapsuleSpec& spec);

    // 访问器
    const double* endpoint_offsets() const;  // nullptr if not fitted
    bool has_fitted_capsules() const;
};
```

### Phase 7：测试

| 测试文件 | 测试内容 |
|---------|---------|
| `tests/test_capsule_spec.cpp` | CapsuleSpec JSON 往返、load/save |
| `tests/test_capsule_fitting.cpp` | 零偏移退化一致性、已知 mesh 的 MEC 拟合正确性 |
| `tests/test_ifk_offset.cpp` | iFK + 偏移 vs 暴力 FK 采样：拟合端点 iAABB 仍保守包含所有采样位置 |
| `tests/test_all_sources_offset.cpp` | 4 个 EndpointSource 使用偏移后的一致性和保守性 |
| `tests/test_volume_reduction.cpp` | 拟合后 envelope volume < DH 原始 volume |

**关键回归测试**：所有 740 个现有测试应当通过（`endpoint_offsets = nullptr` 时行为不变）。

### Phase 8：实验 & 论文更新

| 任务 | 详情 |
|------|------|
| E1 | 运行 `fit_capsules.py`，生成 IIWA14 和 Panda 的 `capsule_spec.json` |
| E2 | 重跑 `exp_v3v4_epiAABB` 对比：DH 胶囊 vs 拟合胶囊的 volume |
| E3 | 重跑 `exp_link_iaabb_bench`：对比拟合后 5 种 envelope 方法的体积 |
| E4 | 论文新增一节 / 一个表格：拟合胶囊 vs DH 胶囊的体积对比 |
| E5 | 更新 Table 1 的 Volume 列（拟合后更紧） |

---

## 5. 文件变更矩阵

| 文件 | 操作 | 估计行数 |
|------|------|---------|
| `robots/iiwa14/iiwa14.urdf` + meshes | 新增（外部下载） | - |
| `robots/panda/panda.urdf` + meshes | 新增（外部下载） | - |
| `scripts/fit_capsules.py` | 新建 | ~300 |
| `include/sbf/robot/capsule_spec.h` | 新建 | ~50 |
| `src/robot/capsule_spec.cpp` | 新建 | ~80 |
| `include/sbf/robot/iaabb_config.h` | 修改 | +15 |
| `src/robot/iaabb_config.cpp` | 修改 | +30 |
| `include/sbf/robot/robot.h` | 修改 | +10 |
| `src/robot/robot.cpp` | 修改 | +20 |
| `src/robot/interval_fk.cpp` | 修改 | +40 (新函数) |
| `include/sbf/robot/interval_fk.h` | 修改 | +5 |
| `src/envelope/crit_sample.cpp` | 修改 | +15 |
| `src/envelope/analytical_solve.cpp` | 修改 | +15 |
| `src/envelope/gcpc.cpp` | 修改 | +20 |
| `src/envelope/endpoint_source.cpp` | 修改 | +10 |
| `tests/test_capsule_spec.cpp` | 新建 | ~80 |
| `tests/test_ifk_offset.cpp` | 新建 | ~120 |
| `CMakeLists.txt` | 修改 | +10 |
| `docs/API_REFERENCE_CN.md` | 更新 | +40 |
| `paper/root.tex` / `root_cn.tex` | 更新 | +30 |
| `doc/CHANGE_LOG_CN.md` | 追加 | +25 |

**共计**：~5 新文件，~11 修改文件，~900 行新增/修改。

---

## 6. 依赖与风险分析

### 6.1 外部依赖

| 依赖 | 用途 | 可选替代 |
|------|------|---------|
| URDF 文件 | 碰撞 mesh 来源 | 手工测量 + 直接编写 `capsule_spec.json` |
| `urdfpy` / `yourdfpy` | Python URDF 解析 | 手写 XML 解析 |
| `trimesh` | STL/DAE mesh 加载 | `numpy-stl` |
| `scipy` | 方向优化 | 纯 numpy 网格搜索 |

### 6.2 风险

| 风险 | 影响 | 缓解 |
|------|------|------|
| URDF 碰撞几何不可用（仅 visual） | 无法拟合, Phase 1 阻塞 | 使用 visual mesh（通常更精细）或手工测量 |
| URDF 帧 ≠ DH 帧（坐标系不对应） | 偏移转换错误 | Phase 1.5 中通过零位 FK 比对验证对应关系 |
| MEC 拟合不够紧（局部最优） | 体积改善有限 | 增大方向搜索密度、使用 miniball 库 |
| 偏移过大导致包裹过估 | 净效果为负（更差） | 设置偏移上限（如 L_ℓ/2），超过则回退 DH 帧 |
| GCPC 缓存失效 | 已有缓存中的极值点位置基于 DH 帧 | 需要判断缓存是否需重建（CapsuleSpec 作为缓存 key） |

### 6.3 回退策略

- `CapsuleSpec` 未提供（`has_fitted_capsules() == false`）时，所有代码路径退化为当前 DH 帧行为
- 可在 `PipelineConfig` 中添加开关 `use_fitted_capsules`
- 任何拟合后体积反而增大的连杆，自动回退到 DH 原始模型

---

## 7. 实施顺序

```
Phase 0: 获取 URDF + mesh              [前置，手工操作]
    ↓
Phase 1: Python MEC 拟合工具           [独立开发，不影响 C++]
    ↓
Phase 2: CapsuleSpec 数据结构          [C++, 独立编译测试]
    ↓
Phase 3: IAABBConfig 集成              [局部修改]
    ↓
Phase 4: iFK 偏移提取 (核心改动)        [interval_fk.cpp + 测试]
    ↓
Phase 5: 4 个 Source 适配              [逐个改动 + 测试]
    ↓
Phase 6: Robot 构造集成                [整合 + 全量回归测试]
    ↓
Phase 7: 测试 + 验证                   [保守性 + 体积改善]
    ↓
Phase 8: 实验 + 论文更新               [最终数据更新]
```

各 Phase 之间有明确的依赖关系，但 **Phase 0-1（Python）和 Phase 2-3（C++ 数据结构）可以并行开发**。

---

## 8. 替代方案对比

| 方案 | 优势 | 劣势 | 评估 |
|------|------|------|------|
| **A: URDF MEC 拟合（本计划）** | 自动化、精确、可复现 | 需 URDF 文件、帧对应复杂 | ★★★★★ 推荐 |
| **B: 手工测量 + JSON** | 简单、无外部依赖 | 不可扩展、容易出错 | ★★★ 备选 |
| **C: 仅缩短 DH 线段（保留 DH 轴方向）** | 最简单改动 | 轴方向仍为 DH 方向，可能非最优 | ★★ 最小方案 |
| **D: 多胶囊体近似** | 可拟合 L 形连杆 | 复杂度成倍增加 | ★ 过度工程 |

**推荐路径**：先实现 Phase 0-1（Python 工具），生成 JSON 后用方案 B 的方式验证效果。
确认体积改善显著后再实施 Phase 2-8 的 C++ 集成。

---

## 9. 成功标准

| 指标 | 目标 |
|------|------|
| IIWA14 envelope volume 缩减 | ≥ 20% |
| Panda envelope volume 缩减 | ≥ 15% |
| 保守性验证通过（无欠估） | 100% |
| 现有 740 测试全通过 | 100% |
| 无偏移时性能无回归 | < 1% |
| Python 拟合工具运行时间 | < 10s / 机器人 |
