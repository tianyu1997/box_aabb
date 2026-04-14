# Phase C: Link IAABB 管线 (Module 2)

> 依赖: Phase B (Endpoint IAABB)
> 状态: **已完成** (2026-04-03)
> 产出: 3 种 Link Envelope 表示 + 统一接口

---

## 目标

将 endpoint iAABBs 转换为 per-link 工作空间包络，支持 AABB(sub=n) 细分和网格光栅化。

---

## 管线概述

```
endpoint_iaabbs [n_active × 2 × 6]
        ↓
EnvelopeType ∈ {LinkIAABB, LinkIAABB_Grid, Hull16_Grid}
        ↓
LinkEnvelope:
  - link_iaabbs [n_active × 6]    (所有类型都有)
  - grid (可选, Grid/Hull16 类型)
```

---

## Step C1: LinkIAABB — AABB(sub=n)

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/envelope/link_iaabb.h` | `src/envelope/link_iaabb.cpp` |

### 接口
```cpp
// 基本 paired derive: proximal+distal → link AABB
void derive_link_iaabb_paired(
    const float* endpoint_iaabbs,   // [n_active × 2 × 6]
    int n_active_links,
    const double* link_radii,       // [n_active], 每个 link 的碰撞半径
    float* out_link_iaabbs          // [n_active × 6]: [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
);

// 细分 derive: 将每个 link 等分为 n_sub 段, 每段独立计算 AABB
// 输出: [n_active × n_sub × 6]
void derive_link_iaabb_subdivided(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const FKState& fk,
    int n_subdivisions,
    float* out_sub_iaabbs           // [n_active × n_sub × 6]
);
```

### 算法
**Paired (sub=1)**:
```
for each link ci:
    prox = endpoint_iaabbs[ci*2]      // proximal endpoint iAABB
    dist = endpoint_iaabbs[ci*2 + 1]  // distal endpoint iAABB
    link_iaabb[ci].lo = min(prox.lo, dist.lo) - radius[ci]
    link_iaabb[ci].hi = max(prox.hi, dist.hi) + radius[ci]
```

**Subdivided (sub=n)**:
```
for each link ci, for each sub-segment s in [0, n_sub):
    t_lo = s / n_sub
    t_hi = (s+1) / n_sub
    // 在 link 参数空间 [t_lo, t_hi] 内插值 proximal↔distal
    sub_iaabb = lerp_interval(prox, dist, t_lo, t_hi) + radius
```

### 迁移来源
| 源版本 | 源文件 | 取什么 |
|--------|--------|--------|
| v4 | `safeboxforest/v4/include/sbf/envelope/envelope_derive.h` | `derive_aabb_paired()`, `derive_aabb_subdivided()` |

### 验收标准
- [x] sub=1 输出维度 `[n_active × 6]` 正确
- [x] sub=1 AABB ⊇ sub=n 各段 union（保守性）
- [x] sub=n 各段 union ≈ sub=1（覆盖性，差异 < radius）

### 实现备注
- `derive_link_iaabb_subdivided` 签名简化为直接接收 `endpoint_iaabbs` 指针 + `link_radii`，无需 `Robot`/`FKState`/`Interval`（纯几何插值）
- sub=1 通过 subdivided 函数的结果与 paired 完全一致（有测试覆盖）

---

## Step C2: LinkIAABB_Grid — 网格光栅化

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/envelope/link_grid.h` | `src/envelope/link_grid.cpp` |

### 接口
```cpp
struct GridConfig {
    double voxel_delta = 0.05;   // 体素边长 (米)
    // grid_origin 和 grid_dims 由 link iAABB 自动计算
};

struct VoxelGrid {
    Eigen::Vector3d origin;      // grid 原点 (世界坐标)
    Eigen::Vector3i dims;        // grid 尺寸 [nx, ny, nz]
    double delta;                // 体素边长
    std::vector<uint8_t> data;   // 占用标记 (0/1), 大小 = nx*ny*nz

    bool occupied(int ix, int iy, int iz) const;
    void set_occupied(int ix, int iy, int iz);
    int  n_occupied() const;
};

// 将 link iAABBs 光栅化到体素网格
VoxelGrid rasterize_link_iaabbs(
    const float* link_iaabbs,      // [n_active × 6]
    int n_active_links,
    const GridConfig& config
);
```

### 算法
```
1. 计算 bounding box = union of all link iAABBs
2. 设定 grid: origin = bounding_box.lo, dims = ceil((hi-lo)/delta)
3. for each link iAABB:
     for each voxel overlapping with link iAABB:
       grid.set_occupied(ix, iy, iz)
```

### 迁移来源
| 源版本 | 源文件 | 取什么 |
|--------|--------|--------|
| v4 | `safeboxforest/v4/include/sbf/envelope/envelope_derive.h` | `derive_grid()` |
| v4 | `safeboxforest/v4/include/sbf/voxel/voxel_grid.h` | 简化版 VoxelGrid（去掉 bit_brick） |

### 验收标准
- [x] 所有 link iAABB 在 grid 中对应体素被标记
- [x] grid 空间分辨率与 `delta` 一致
- [x] `n_occupied()` 单调递增随 n_active_links

### 实现备注
- 简化版 VoxelGrid：flat `std::vector<uint8_t>` 存储，无 bit_brick
- grid origin/dims 由 link iAABBs union 自动计算

---

## Step C3: Hull16_Grid — 延后实现

### 文件
仅预留枚举值，不实现。

```cpp
// 在 envelope_type.h 中:
enum class EnvelopeType : uint8_t {
    LinkIAABB      = 0,
    LinkIAABB_Grid = 1,
    Hull16_Grid    = 2   // 预留, 第一版不实现
};
```

后续按需从 v4 `include/sbf/voxel/hull_rasteriser.h` 迁移。

> **状态**: 仅预留枚举值 ✅ — 无需实现代码

---

## Step C4: 统一 Link Envelope 接口

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/envelope/envelope_type.h` | `src/envelope/envelope_type.cpp` |

### 接口
```cpp
enum class EnvelopeType : uint8_t {
    LinkIAABB      = 0,
    LinkIAABB_Grid = 1,
    Hull16_Grid    = 2   // 预留
};

struct EnvelopeTypeConfig {
    EnvelopeType type = EnvelopeType::LinkIAABB;
    int n_subdivisions = 1;          // LinkIAABB: sub=1 整体, sub=n 细分
    GridConfig grid_config;          // Grid types only
};

struct LinkEnvelope {
    EnvelopeType type;
    std::vector<float> link_iaabbs;  // [n_active × 6], 所有类型都有
    std::optional<VoxelGrid> grid;   // Grid/Hull16 类型才有
    int n_active_links;
};

LinkEnvelope compute_link_envelope(
    const float* endpoint_iaabbs,    // from Stage 1
    int n_active_links,
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const FKState& fk,
    const EnvelopeTypeConfig& config
);
```

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v4 | `safeboxforest/v4/include/sbf/envelope/envelope_type.h` |

### 实现备注
- `LinkEnvelope.grid` 使用 `std::unique_ptr<VoxelGrid>` 而非 `std::optional<VoxelGrid>`（避免 Eigen 对齐 + MSVC optional 的边界问题）
- `compute_link_envelope` 签名简化：去掉 `Robot`/`Interval`/`FKState` 参数，仅需 `endpoint_iaabbs` + `link_radii`
- Hull16_Grid 在 `compute_link_envelope` 中通过 `assert` 拦截

---

## 测试: test_link_iaabb.cpp

```
TEST_SUITE("LinkIAABB_Paired") {
    - 2DOF sub=1: 输出维度 [n_active × 6]
    - 2DOF sub=1: AABB 包含所有 endpoint iAABBs
    - radius 膨胀正确 (link_iaabb.lo = min(prox.lo, dist.lo) - r)
}

TEST_SUITE("LinkIAABB_Subdivided") {
    - 2DOF sub=4: 输出维度 [n_active × 4 × 6]
    - 包含关系: sub=1 ⊇ union(sub=4)
}

TEST_SUITE("VoxelGrid") {
    - rasterize 后所有 link 对应体素被标记
    - 空场景 → 0 occupied voxels
}

TEST_SUITE("Unified LinkEnvelope") {
    - EnvelopeType::LinkIAABB → grid == nullopt
    - EnvelopeType::LinkIAABB_Grid → grid != nullopt
    - EnvelopeType::Hull16_Grid → throw/assert (未实现)
}
```
