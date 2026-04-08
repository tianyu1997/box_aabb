# PLAN: 利用关节对称性减少 LECT 区间缓存范围

> 目标：分析 6/7-DOF 串联机器人各关节的对称性质，设计通用方法用于
> 减少 LECT KD-tree 的 envelope 缓存计算量与存储量。
>
> 日期：2026-03-24
> 状态：计划阶段（Draft）

---

## 1. 问题背景

LECT (Link Envelope Collision Tree) 是一棵 KD-tree，将关节空间
`[q_0^lo, q_0^hi] × ... × [q_6^lo, q_6^hi]` 递归划分为超矩形节点。
每个节点需要计算并缓存所有活跃连杆的 iAABB（区间算术包围盒）。

如果某个关节 q_i 存在对称性，使得"关节区间平移/反射后的 iAABB"
能从"规范区间的 iAABB"通过简单坐标变换得到，
则可以只计算规范区间的 envelope，其余区间通过映射推导，
从而减少计算量与缓存存储。

**上一轮初步分析的结论（需修正）**：

之前错误地认为 q_0 具有 D_4（含反射）对称性（[0, π/4] 即可），
以及 q_1、q_2 具有 π 周期对称性。

**本文件纠正了这些错误**，给出严格的数学证明。

---

## 2. DH 变换矩阵约定

v4 使用 Modified DH (Craig) 约定。对关节 i，DH 变换矩阵为：

```
T_i(θ) = [ cos θ,        -sin θ,        0,      a_i       ]
          [ sin θ · cα_i,  cos θ · cα_i, -sα_i, -d_i · sα_i]
          [ sin θ · sα_i,  cos θ · sα_i,  cα_i,  d_i · cα_i]
          [ 0,             0,             0,      1          ]
```

其中 cα = cos(α_i)，sα = sin(α_i)。对旋转关节，θ = q_i + θ_offset。

正运动学链：
```
p_k = T_0(q_0) · T_1(q_1) · ... · T_k(q_k) · [0, 0, 0, 1]^T
```

---

## 3. 核心数学性质：关节平移的分解定理

**定理**：对于旋转关节 i，将 q_i 平移 Δ 等价于在 T_i 之后插入一个
局部坐标系下的 Rot_z(Δ)：

```
T_i(q_i + Δ) = T_i(q_i) · Rot_z(Δ)
```

**证明**：

```
T_i(θ) = M_{α,a} · Rot_z(θ) · Trans_z(d_i)

其中 M_{α,a} = Rot_x(α_i) · Trans_x(a_i)

T_i(θ + Δ) = M_{α,a} · Rot_z(θ + Δ) · Trans_z(d_i)
            = M_{α,a} · Rot_z(θ) · Rot_z(Δ) · Trans_z(d_i)
                                   ↑ Rot_z 可交换
            = M_{α,a} · Rot_z(θ) · Trans_z(d_i) · Rot_z(Δ)
                                   ↑ Rot_z(Δ) 与 Trans_z(d_i) 可交换
            = T_i(θ) · Rot_z(Δ)   ∎
```

**推论**：q_i 平移 Δ 后的 FK 链为：
```
p_k = T_{0..i-1} · T_i(q_i) · Rot_z(Δ) · T_{i+1..k} · origin
    = T_{prefix} · (Rot_z(Δ) · P_downstream)
```

这意味着 q_i 的平移等价于将所有 downstream 位置在关节 i 的
**局部坐标系**中绕 z 轴旋转 Δ。

---

## 4. 对称性能用于 LECT iAABB 的充要条件

为使 q_i → q_i + Δ 平移产生有效的 **世界坐标系 AABB 映射**，需要满足：

> **条件**：Rot_z(Δ) 作用于 P_downstream 后，经过 T_{prefix}（区间矩阵）
> 映射到世界坐标系后，ALL links 的 AABB 可由原 AABB 通过闭式变换得到。

**关键障碍**：当 i > 0 时，T_{prefix} 是一个区间矩阵（各元素有宽度），
它将局部 Rot_z(Δ) 在世界坐标系中的效果混合到 x, y, z 各分量中，
**无法**产生简单的 AABB 坐标映射（swap / negate）。

**唯一可行情形**：i = 0（第一个关节），此时 T_{prefix} = Identity，
且 T_0(q_0 + Δ) = Rot_z(q_0 + Δ) · Trans_z(d_0)，等价于在世界坐标系中
将所有连杆位置绕 z 轴旋转 Δ。

---

## 5. q_0（基座关节）对称性 — 严格证明

### 5.1 前提条件

q_0 的 DH 参数：α_0 = 0，a_0 = a（可 ≠ 0），d_0 = d，θ_offset = 0。

当 α_0 = 0 时：
```
T_0(q_0) = [ cos q_0,  -sin q_0,  0,  a  ]
            [ sin q_0,   cos q_0,  0,  0  ]
            [ 0,         0,        1,  d  ]
            [ 0,         0,        0,  1  ]
```

令下游位置 P_ds = [x', y', z', 1]^T，则世界位置为：
```
p_x = cos(q_0) · x' - sin(q_0) · y' + a
p_y = sin(q_0) · x' + cos(q_0) · y'
p_z = z' + d
```

### 5.2 π/2 旋转对称性 (Z4)

将 q_0 → q_0 + π/2：
```
cos(q_0 + π/2) = -sin(q_0),  sin(q_0 + π/2) = cos(q_0)

p_x' = -sin(q_0) · x' - cos(q_0) · y' + a  =  -p_y + a
p_y' =  cos(q_0) · x' - sin(q_0) · y'       =   p_x - a
p_z' =  z' + d                               =   p_z
```

**AABB 映射**（ALL links 同时成立，与下游构型无关）：
```
x'_min = -y_max + a,   x'_max = -y_min + a
y'_min =  x_min - a,   y'_max =  x_max - a
z'_min =  z_min,        z'_max =  z_max
```

当 a = 0（如 Panda、KUKA iiwa、UR 系列）时简化为：
```
x'_min = -y_max,   x'_max = -y_min
y'_min =  x_min,   y'_max =  x_max
z'_min =  z_min,   z'_max =  z_max
```

类似地：
- **q_0 + π**：(p_x, p_y, p_z) → (-p_x + 2a, -p_y, p_z)
- **q_0 + 3π/2**：(p_x, p_y, p_z) → (p_y + a, -p_x + a, p_z)

这四个映射构成 **Z_4 旋转群**（绕点 (a, 0, ⋅) 的 90° 旋转）。

### 5.3 q_0 → -q_0 反射对称性 — 不成立的证明

将 q_0 → -q_0：
```
p_x(-q_0) = cos(q_0) · x' + sin(q_0) · y' + a
p_y(-q_0) = -sin(q_0) · x' + cos(q_0) · y'
```

对比原始：
```
p_x(q_0) = cos(q_0) · x' - sin(q_0) · y' + a
```

差异在于 sin(q_0) · y' 的符号。由于 y' 取决于下游关节
（y' = z'' 对 Panda），它的取值域**不关于 0 对称**，因此：

```
range{cos(q_0) · x' + sin(q_0) · y'} ≠ range{cos(q_0) · x' - sin(q_0) · y'}
```

在一般情况下不等式两边是不同的集合。**反射对称不可用**。

> **结论**：q_0 只有 Z_4 旋转对称性，没有反射对称性。
> 最小规范区间为 [0, π/2]，而非 [0, π/4]。

---

## 6. q_1（第二关节）对称性 — 不成立的证明

### 6.1 Panda q_1 的平移分析

q_1 → q_1 + π 导致 T_1(q_1 + π) = T_1(q_1) · Rot_z(π)。

Rot_z(π) 作用于下游位置 P_ds = [x'', y'', z'', 1]^T：
```
Rot_z(π) · P_ds = [-x'', -y'', z'', 1]^T
```

通过 T_1（α = -π/2, a = 0, d = 0），在 T_0 输出坐标系中：
```
原始：    x' = cos(q_1)·x'' - sin(q_1)·y'',   y' = z'',   z' = -sin(q_1)·x'' - cos(q_1)·y''
平移后：  x'_s = -x'_orig,                     y'_s = y'_orig,  z'_s = -z'_orig
```

通过 T_0(q_0) 映射到世界坐标系：
```
p_x_shift = cos(q_0)·(-x') - sin(q_0)·y' + a = -cos(q_0)·x' - sin(q_0)·y' + a
p_y_shift = sin(q_0)·(-x') + cos(q_0)·y'     = -sin(q_0)·x' + cos(q_0)·y'
```

对比原始：
```
p_x_orig = cos(q_0)·x' - sin(q_0)·y' + a
p_y_orig = sin(q_0)·x' + cos(q_0)·y'
```

**关键**：p_x_shift ≠ f(p_x_orig, p_y_orig, p_z_orig) 的任何简单形式
（不是 negate、不是 swap、不是常数偏移），因为 x' 和 y' 是独立变量，
被 cos(q_0) 和 sin(q_0) 以不同方式组合。

当 q_0 是一个区间时，AABB 通过区间算术计算：
```
AABB_x = I_cos(q_0) · I(x') - I_sin(q_0) · I(y') + a   ... (1)
AABB_x_shifted = I_cos(q_0) · I(-x') - I_sin(q_0) · I(y') + a   ... (2)
```

由于区间乘法 `[α,β] · [-b,-a] ≠ -([α,β] · [a,b])` 在一般情况下，
(1) 和 (2) 给出不同的 AABB，且无闭式映射关系。

> **结论**：q_1 在 LECT 层面不具有可利用的 AABB 对称性。
>
> 注：GCPC 中 q_1 的 period-π 对称性之所以成立，是因为 GCPC 已经
> 将 q_0 **完全消除**（atan2 重建），在 q_0 被消除的 local frame 中
> 才有 A → -A, C → -C 的简单映射。这与 LECT 的 AABB 缓存是不同层面的。

---

## 7. q_2（第三关节）对称性 — 不成立的证明

与 q_1 类似。q_2 → q_2 + π 导致 Rot_z(π) 插入到关节 2 和关节 3 之间：
```
T_2(α=π/2, a=0, d=0.316) 下游位置变换：
原始：    x'' = cos(q_2)·x''' - sin(q_2)·y''',  y'' = -z''' - 0.316,  z'' = sin(q_2)·x''' + cos(q_2)·y'''
平移后：  x''_s = -x'',  y''_s = y'',  z''_s = -z''
```

经过 T_1 再进入 T_0 的 Rot_z(q_0) 后：
```
x'_shift = cos(q_1)·(-x'') - sin(q_1)·y'' = -cos(q_1)·x'' - sin(q_1)·y''
```

注意：x'_shift ≠ -x'_orig = -(cos(q_1)·x'' - sin(q_1)·y'')
因为 sin(q_1)·y'' 项的符号不同。

这导致经过整个前缀链后，世界坐标系 AABB 无法通过简单映射得到。

> **结论**：q_2 在 LECT 层面不具有可利用的 AABB 对称性。

---

## 8. q_3 至 q_6 分析

| 关节 | α | a | d | 关节范围 | 范围对称? | 对称性? |
|------|---|---|---|----------|-----------|---------|
| q_3 | π/2 | 0.0825 | 0 | [-3.07, -0.07] | ✗ 不对称 | ✗ 无 |
| q_4 | -π/2 | -0.0825 | 0.384 | [-2.90, 2.90] | ✓ 对称 | ✗ 无（前缀为区间矩阵）|
| q_5 | π/2 | 0 | 0 | [-0.02, 3.75] | ✗ 不对称 | ✗ 无 |
| q_6 | π/2 | 0.088 | 0 | [-2.90, 2.90] | ✓ 对称 | ✗ 无（前缀为区间矩阵）|

即使关节范围对称，由于前缀变换 T_{0..i-1} 是区间矩阵，
局部 Rot_z(Δ) 在世界坐标系中不产生简单 AABB 映射。

> **结论**：q_3 至 q_6 均不具有可利用的 LECT AABB 对称性。

---

## 9. 总结：可利用对称性一览

| 关节 | 可利用? | 对称群 | 条件 | 规范区间 | 缓存缩减倍数 |
|------|---------|--------|------|----------|--------------|
| q_0 | **✓** | Z_4（4-fold 旋转） | α_0 = 0 | [0, π/2] | **4×** |
| q_1 | ✗ | — | — | 全范围 | 1× |
| q_2 | ✗ | — | — | 全范围 | 1× |
| q_3 | ✗ | — | — | 全范围 | 1× |
| q_4 | ✗ | — | — | 全范围 | 1× |
| q_5 | ✗ | — | — | 全范围 | 1× |
| q_6 | ✗ | — | — | 全范围 | 1× |

> **只有 q_0 可利用，且仅有旋转对称性（无反射），规范区间为 [0, π/2]。**
> 之前声称的 [0, π/4]（含反射对称）是错误的。

---

## 10. 通用对称性检测方法设计

### 10.1 检测算法

对于一般 N-DOF 串联机器人，自动检测可利用的 LECT AABB 对称性：

```
算法: detect_lect_joint_symmetries(robot)

输入: Robot（含 DH 参数和关节限位）
输出: JointSymmetry[N]

for i = 0 to N-1:
    if i == 0:
        # 检查第一个关节是否为纯 z 旋转 (α_0 ≈ 0)
        if |α_0| < ε:
            symmetry[0] = Z4_ROTATION
            symmetry[0].canonical_range = [0, π/2]
            symmetry[0].period = π/2
            symmetry[0].offset = a_0    # AABB 变换的偏移量
        else:
            symmetry[0] = NONE
    else:
        # 对 i > 0 的关节：
        #   由于 prefix T_{0..i-1} 是区间矩阵，
        #   局部 Rot_z(Δ) 无法转化为世界 AABB 映射。
        symmetry[i] = NONE

return symmetry
```

### 10.2 数据结构

```cpp
// ─── 关节对称性类型 ─────────────────────────────────────────────────
enum class JointSymmetryType : uint8_t {
    NONE         = 0,   // 无可利用对称性
    Z4_ROTATION  = 1,   // 4-fold 旋转对称 (shift by kπ/2)
};

// ─── 单个关节的对称性描述 ───────────────────────────────────────────
struct JointSymmetry {
    int                joint_index = 0;
    JointSymmetryType  type = JointSymmetryType::NONE;
    double             canonical_lo = 0.0;  // 规范区间下界
    double             canonical_hi = 0.0;  // 规范区间上界
    double             period = 0.0;        // 对称周期 (π/2 for Z4)
    double             offset_x = 0.0;      // a_i 参数 (AABB 变换偏移)

    /// 将 q 映射到规范区间，返回扇区编号 (0-3 for Z4)
    /// sector 0: canonical,  1: +π/2,  2: +π,  3: +3π/2
    int canonicalize(double q, double& q_canonical) const;

    /// 将 canonical AABB 变换到目标扇区
    /// src: [x_min, y_min, z_min, x_max, y_max, z_max]
    /// dst: 变换后的 AABB
    void transform_aabb(const float* src, int sector, float* dst) const;

    /// 批量变换 ALL active links 的 AABB
    void transform_all_link_aabbs(
        const float* src_aabbs, int n_links,
        int sector,
        float* dst_aabbs) const;
};
```

### 10.3 AABB 变换实现

对于 Z4_ROTATION（offset = a）：

```cpp
void JointSymmetry::transform_aabb(
    const float* src, int sector, float* dst) const
{
    float x_lo = src[0], y_lo = src[1], z_lo = src[2];
    float x_hi = src[3], y_hi = src[4], z_hi = src[5];
    float a = static_cast<float>(offset_x);

    switch (sector % 4) {
    case 0:  // identity
        std::memcpy(dst, src, 6 * sizeof(float));
        break;
    case 1:  // +π/2 rotation
        dst[0] = -y_hi + a;   dst[3] = -y_lo + a;
        dst[1] =  x_lo - a;   dst[4] =  x_hi - a;
        dst[2] =  z_lo;       dst[5] =  z_hi;
        break;
    case 2:  // +π rotation
        dst[0] = -x_hi + 2*a; dst[3] = -x_lo + 2*a;
        dst[1] = -y_hi;       dst[4] = -y_lo;
        dst[2] =  z_lo;       dst[5] =  z_hi;
        break;
    case 3:  // +3π/2 rotation
        dst[0] =  y_lo - a;   dst[3] =  y_hi - a;
        dst[1] = -x_hi + a;   dst[4] = -x_lo + a;
        dst[2] =  z_lo;       dst[5] =  z_hi;
        break;
    }
}
```

### 10.4 区间映射（canonicalize）

```cpp
int JointSymmetry::canonicalize(double q, double& q_canonical) const {
    // 将 q 映射到 [canonical_lo, canonical_lo + 2π) 范围
    double q_norm = std::fmod(q - canonical_lo, 2 * M_PI);
    if (q_norm < 0) q_norm += 2 * M_PI;
    q_norm += canonical_lo;

    // 确定扇区
    int sector = static_cast<int>(std::floor(
        (q_norm - canonical_lo) / period));
    sector = std::clamp(sector, 0, 3);

    // 映射到规范区间
    q_canonical = q_norm - sector * period;
    return sector;
}
```

---

## 11. LECT 集成方案

### 11.1 Warm-Start Pre-Expansion

在 `LECT::pre_expand()` 中：

1. 首次沿 q_0 维度分裂时，强制在 q_0 = 0, π/2, -π/2, π
   （或在 joint limits 范围内的子集）处分裂。
2. 只对规范扇区（sector 0）的子树递归计算 envelope。
3. 对其余 3 个扇区的子树，用 `transform_all_link_aabbs()` 从
   规范扇区推导 iAABB，**跳过 FK 和 envelope 重新计算**。

**期望收益**：pre_expand 的 envelope 计算量减少至约 1/4。

### 11.2 Online FFB Symmetry Reuse

在 `find_free_box()` 下降过程中，当需要为新节点计算 envelope 时：

1. 检查该节点的 q_0 区间是否可映射到规范扇区。
2. 查找规范扇区中是否有"对称节点"已缓存 envelope。
3. 如果有，直接应用 AABB 变换而非重新计算。

**注意**：这要求 KD-tree 在 q_0 维度上的分裂结构具有扇区对齐的对称性。
实际上 FFB 是 lazy 的（按需分裂），不同查询路径可能导致不对称的分裂。
因此该优化仅在 warm-start 或预构建阶段最为有效。

### 11.3 Cache Persistence

保存 LECT 缓存时，可选择只存储规范扇区的节点 envelope，
加载时通过对称映射恢复其余扇区。可减少磁盘缓存大小至约 1/4。

### 11.4 Hull VoxelGrid 对称映射

对于 Hull-16 VoxelGrid，q_0 的 90° 旋转映射为：
- 体素坐标 (ix, iy, iz) → (-iy + offset, ix - offset, iz)
- 对应的 BitBrick 需要重新映射 brick 坐标并旋转内部位模式

此操作比 iAABB 映射复杂，但仍可实现。
初期建议：iAABB 使用对称映射，Hull 仍重新计算（Hull 计算较轻量）。

---

## 12. 实现步骤

### Phase 1: 基础设施（新增文件）

| 步骤 | 文件 | 内容 |
|------|------|------|
| 1.1 | `include/sbf/core/joint_symmetry.h` | `JointSymmetryType`, `JointSymmetry` struct |
| 1.2 | `src/core/joint_symmetry.cpp` | `detect_joint_symmetries()`, `canonicalize()`, `transform_aabb()` |
| 1.3 | `tests/test_joint_symmetry.cpp` | 单元测试：AABB 变换正确性、canonicalize 正确性 |

### Phase 2: LECT 集成

| 步骤 | 文件 | 内容 |
|------|------|------|
| 2.1 | `include/sbf/forest/lect.h` | 新增 `JointSymmetry symmetry_q0_` 成员 |
| 2.2 | `src/forest/lect.cpp` | 构造函数中调用 `detect_joint_symmetries()` |
| 2.3 | `src/forest/lect.cpp` | `pre_expand` 中加入对称跳过逻辑 |
| 2.4 | `tests/test_lect_symmetry.cpp` | 验证对称 pre-expand 与全量 pre-expand 结果一致 |

### Phase 3: Cache 持久化优化（可选）

| 步骤 | 文件 | 内容 |
|------|------|------|
| 3.1 | `src/envelope/envelope_cache.cpp` | save 时仅存规范扇区 |
| 3.2 | `src/forest/lect.cpp` | load 时通过对称映射恢复 |

---

## 13. 适用性分析

### 13.1 标准工业机器人

| 机器人 | α_0 | a_0 | Z4 对称? | 规范区间 |
|--------|-----|-----|----------|----------|
| Franka Panda (7-DOF) | 0 | 0 | ✓ | [0, π/2] |
| KUKA iiwa 14 (7-DOF) | 0 | 0 | ✓ | [0, π/2] |
| UR5/UR10 (6-DOF) | 0 | 0 | ✓ | [0, π/2] |
| ABB IRB 4600 | 0 | 0.175 | ✓ (offset=0.175) | [0, π/2] |
| Fanuc LR Mate 200 | 0 | 0.15 | ✓ (offset=0.15) | [0, π/2] |

几乎所有标准工业机器人的 q_0 都满足 α_0 = 0。

### 13.2 非标准情况

若 α_0 ≠ 0（如 Stewart 平台的特殊构型、某些并联机构），
则 q_0 无对称性可用，算法自动跳过。

---

## 14. 与 GCPC 对称性的关系

GCPC 已有的对称性优化:
- **q_0 完全消除** (atan2 重建)：比 Z4 更强，但仅用于 GCPC 内部
- **q_1 period-π** (q_1 ∈ [0,π] 存储)：仅在 q_0 被消除的 local frame 下有效

LECT 的 Z4 对称性与 GCPC 是**互补的、独立的**两层优化：
- GCPC 在 **临界点缓存** 层面利用对称性（减少预计算点数）
- LECT 在 **KD-tree 节点 envelope** 层面利用对称性（减少计算/存储量）

两者可以同时使用，不冲突。

---

## 15. 预期收益估算

对于 Panda 的 pre_expand（假设 q_0 范围 [-2.90, 2.90] ≈ 332°）：

- 规范区间 [0, π/2] ≈ [0°, 90°] 覆盖 90° / 332° ≈ 27% 的 q_0 范围
- 其余 73% 可由 3 个旋转映射覆盖
- **Envelope 计算量**：减少约 **3/4**（仅 q_0 维度分裂时生效）
- **缓存存储**：减少约 **3/4**（规范扇区存储 + 元数据）

整体影响取决于 tree 在 q_0 维度上的分裂比例。通常 7-DOF 树中约
1/7 的分裂在 q_0，因此全树 envelope 计算量减少约 3/7 × 75% ≈ **32%**。

---

## 16. 开放问题

1. **Hull-16 旋转映射**：是否值得实现 VoxelGrid 的 90° 旋转？
   还是说 Hull 的计算成本足够低，重计算即可？

2. **在线 FFB 对称复用**：lazy 分裂的 KD-tree 很难保证扇区对齐。
   是否应在首次 q_0 分裂时强制对齐到 kπ/2 边界？

3. **Endpoint iAABB 对称映射**：endpoint_iaabb 的局部坐标系
   是否也能利用 q_0 对称性？（需要分析 freeze-depth 的影响）

4. **多 Z4 关节**：如果一个机器人有多个连续的基座 z 旋转关节
   （如双臂），它们的 Z4 对称性能否叠加？

---

## 17. 下一步

1. **审阅本计划**：确认数学推导无误，确认 scope 合理
2. **Phase 1 实现**：`joint_symmetry.h/.cpp` + 单元测试
3. **Phase 2 集成**：LECT pre_expand 对称优化
4. **性能基准**：对比优化前后的 pre_expand 时间和缓存大小
