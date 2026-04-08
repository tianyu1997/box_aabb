# Phase N: Voxel Grid + BitBrick + Hull-16 光栅化

> 依赖: Phase A (Core) + Phase B-C (Envelope)
> 状态: ✅ 已完成
> 产出: `voxel/` 模块 — 高性能体素化 Envelope + 碰撞检测
> 完成日期: 2026-04-04

---

## 目标

从 v4 迁移成熟的体素化子系统，提供：
1. **BitBrick**: 8×8×8 = 512 voxels 压缩到 1 cache line (64 bytes)
2. **VoxelGrid**: 稀疏体素网格 (FlatBrickMap 哈希)
3. **Hull-16 光栅化**: Conv(B₁∪B₂)⊕Ball 的快速扫描线填充
4. **Envelope → Voxel**: 将 link iAABB / Hull-16 envelope 写入体素

### 为什么作为最后 Phase

- 核心流程 (iAABB → FFB → Forest → Plan) 已完整且高效
- Voxel 主要用于: (a) 更紧密的碰撞近似, (b) 可视化比较, (c) 论文精度分析
- v4 代码成熟 (~1200 LOC), 迁移风险低

---

## 架构概览

```
VoxelGrid
├── FlatBrickMap<BrickCoord, BitBrick>  — 稀疏哈希存储
├── fill_aabb(lo, hi)                   — AABB 光栅化
├── fill_hull16(B1, B2, radius)         — Hull-16 扫描线
├── merge(other)                        — 并集 (bitwise OR)
├── collides(other)                     — 碰撞 (bitwise AND)
└── count_occupied()                    — popcount 计数

BitBrick (64 bytes = 1 cache line)
├── words[8]: uint64_t                  — 8 slabs of 8×8
├── set(x,y,z), test(x,y,z)
├── popcount()
├── operator|(), operator&()
└── intersects(other)
```

---

## Step N1: BitBrick

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/voxel/bit_brick.h` | (header-only) |

### 接口
```cpp
namespace sbf::voxel {

/// 8×8×8 = 512 voxels packed into 64 bytes (1 cache line)
/// Layout: words[z], bit_index = y*8 + x
struct BitBrick {
    uint64_t words[8] = {};

    void set(int x, int y, int z) {
        words[z] |= (uint64_t(1) << (y * 8 + x));
    }

    bool test(int x, int y, int z) const {
        return (words[z] >> (y * 8 + x)) & 1;
    }

    void clear_voxel(int x, int y, int z) {
        words[z] &= ~(uint64_t(1) << (y * 8 + x));
    }

    int popcount() const {
        int c = 0;
        for (int i = 0; i < 8; ++i)
            c += popcount64(words[i]);
        return c;
    }

    BitBrick operator|(const BitBrick& o) const {
        BitBrick r;
        for (int i = 0; i < 8; ++i) r.words[i] = words[i] | o.words[i];
        return r;
    }

    BitBrick operator&(const BitBrick& o) const {
        BitBrick r;
        for (int i = 0; i < 8; ++i) r.words[i] = words[i] & o.words[i];
        return r;
    }

    bool intersects(const BitBrick& o) const {
        for (int i = 0; i < 8; ++i)
            if (words[i] & o.words[i]) return true;
        return false;
    }

    bool empty() const {
        for (int i = 0; i < 8; ++i)
            if (words[i]) return false;
        return true;
    }

private:
    static int popcount64(uint64_t v) {
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_ARM64))
        return (int)__popcnt64(v);
#else
        return __builtin_popcountll(v);
#endif
    }
};

}  // namespace sbf::voxel
```

### 迁移来源
| 源文件 |
|--------|
| `v4/include/sbf/voxel/bit_brick.h` (~150 LOC) |

---

## Step N2: BrickCoord + FlatBrickMap

### 文件
| 头文件 |
|--------|
| `include/sbf/voxel/voxel_grid.h` (与 VoxelGrid 同文件) |

### BrickCoord
```cpp
struct BrickCoord {
    int bx, by, bz;

    bool operator==(const BrickCoord& o) const {
        return bx == o.bx && by == o.by && bz == o.bz;
    }
};

struct BrickCoordHash {
    size_t operator()(const BrickCoord& c) const {
        // FNV-1a style hash
        size_t h = 2166136261u;
        h ^= (size_t)c.bx; h *= 16777619u;
        h ^= (size_t)c.by; h *= 16777619u;
        h ^= (size_t)c.bz; h *= 16777619u;
        return h;
    }
};
```

### FlatBrickMap
```cpp
/// Open-addressing hash map, 2-3× faster than unordered_map
/// Power-of-2 capacity, linear probing, 70% max load factor
class FlatBrickMap {
public:
    FlatBrickMap(int initial_capacity = 256);

    BitBrick& operator[](const BrickCoord& c);
    const BitBrick* find(const BrickCoord& c) const;
    bool contains(const BrickCoord& c) const;
    int size() const;

    // 迭代器
    struct Iterator { ... };
    Iterator begin() const;
    Iterator end() const;

private:
    struct Slot {
        BrickCoord key;
        BitBrick   brick;
        bool       occupied = false;
    };
    std::vector<Slot> slots_;
    int size_ = 0;
    int capacity_;
    float max_load_ = 0.7f;

    void rehash();
    int probe(const BrickCoord& c) const;
};
```

### 设计要点
- Power-of-2 capacity → `& (cap - 1)` 代替 `%`
- Linear probing → cache friendly
- 70% load factor → 少数 probe 即命中

### 迁移来源
| 源文件 |
|--------|
| `v4/include/sbf/voxel/voxel_grid.h` (FlatBrickMap 部分, ~300 LOC) |

---

## Step N3: VoxelGrid 核心

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/voxel/voxel_grid.h` | `src/voxel/voxel_grid.cpp` |

### 接口
```cpp
class VoxelGrid {
public:
    /// delta: 每个 voxel 的边长 (world units)
    /// origin: 网格原点 (world coordinates)
    /// safety_pad: 安全膨胀 (default: sqrt(3)*delta/2)
    VoxelGrid(double delta, const Eigen::Vector3d& origin,
              double safety_pad = -1.0);

    // --- 坐标转换 ---
    Eigen::Vector3i to_cell(const Eigen::Vector3d& world) const;
    Eigen::Vector3d cell_center(const Eigen::Vector3i& cell) const;
    BrickCoord to_brick(const Eigen::Vector3i& cell) const;

    // --- 填充 ---
    void fill_aabb(const Eigen::Vector3d& lo, const Eigen::Vector3d& hi);
    void fill_hull16(const Eigen::Vector3d& p1_lo, const Eigen::Vector3d& p1_hi,
                     const Eigen::Vector3d& p2_lo, const Eigen::Vector3d& p2_hi,
                     double radius);

    // --- 集合操作 ---
    void merge(const VoxelGrid& other);      // |= (union)
    bool collides(const VoxelGrid& other) const;  // & != 0

    // --- 查询 ---
    int count_occupied() const;
    int n_bricks() const;
    bool empty() const;

    // --- 访问 ---
    double delta() const;
    const Eigen::Vector3d& origin() const;
    const FlatBrickMap& bricks() const;

private:
    double delta_;
    Eigen::Vector3d origin_;
    double safety_pad_;
    FlatBrickMap bricks_;
};
```

### fill_aabb 算法
```cpp
void VoxelGrid::fill_aabb(lo, hi) {
    // 膨胀 safety_pad
    lo -= Eigen::Vector3d::Constant(safety_pad_);
    hi += Eigen::Vector3d::Constant(safety_pad_);

    // 转 cell 范围
    auto c_lo = to_cell(lo);
    auto c_hi = to_cell(hi);

    // 遍历 cell → set bit
    for (int z = c_lo.z(); z <= c_hi.z(); ++z)
      for (int y = c_lo.y(); y <= c_hi.y(); ++y)
        for (int x = c_lo.x(); x <= c_hi.x(); ++x) {
            auto bc = to_brick({x, y, z});
            bricks_[bc].set(x & 7, y & 7, z & 7);
        }
}
```

### 迁移来源
| 源文件 |
|--------|
| `v4/include/sbf/voxel/voxel_grid.h` (VoxelGrid 部分, ~400 LOC) |

---

## Step N4: Hull-16 扫描线光栅化

### 算法
```
fill_hull16(p1_lo, p1_hi, p2_lo, p2_hi, radius):
    // Conv(B₁∪B₂) ⊕ Ball(r) 的体素光栅化
    //
    // B₁ = [p1_lo, p1_hi] (link position at one config)
    // B₂ = [p2_lo, p2_hi] (link position at another config)
    //
    // 步骤:
    // 1. 计算 convex hull 的 AABB 边界 (保守)
    //    hull_lo = min(p1_lo, p2_lo) - radius - safety_pad
    //    hull_hi = max(p1_hi, p2_hi) + radius + safety_pad
    //
    // 2. 退化检测: 如果 B₁ ≈ B₂ → fallback fill_aabb
    //
    // 3. Turbo scanline (per-brick batch):
    //    - 以 brick (8×8×8) 为单位枚举 X 范围
    //    - 每个 brick 内, 对 8×8 YZ tile 解析 hull 边界
    //    - 用 find_t_range_yz() 求 Y-Z 平面上 t ∈ [0,1] 的有效范围
    //    - 合并 tile → set bits
```

### 性能
- Brick-batch 减少 hash 查找次数 (8×8×8 共享一次)
- 退化 hull fast-path: `||slope|| < ε → fill_aabb`
- 与 naive 逐 voxel 相比 ~3× 加速

### 迁移来源
| 源文件 |
|--------|
| `v4/include/sbf/voxel/voxel_grid.h` → `fill_hull16()` + `find_t_range_yz()` |

---

## Step N5: Hull Rasteriser (Robot → Voxel 桥接)

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/voxel/hull_rasteriser.h` | `src/voxel/hull_rasteriser.cpp` |

### 接口
```cpp
namespace sbf::voxel {

/// 从 FKState prefix transform 提取 (x,y,z) 位置区间
struct PosInterval {
    float lo[3], hi[3];
};

PosInterval frame_pos(const FKState& fk, int frame_idx);

/// Hull-16: per-link sub-segments (16 subdivision)
void rasterise_robot_hull16(VoxelGrid& grid,
                            const Robot& robot,
                            const BoxNode& box,
                            int n_subdiv = 16);

/// 更快但更粗的: per-link sub-AABB decomposition
void rasterise_robot_sub_aabbs(VoxelGrid& grid,
                               const Robot& robot,
                               const BoxNode& box,
                               int n_subdiv = 4);

/// 最粗: per-link full AABB (no subdivision)
void rasterise_robot_aabbs(VoxelGrid& grid,
                           const Robot& robot,
                           const BoxNode& box);

/// 障碍物 AABB → voxel
void rasterise_box_obstacle(VoxelGrid& grid,
                            const Obstacle& obs);

}
```

### 迁移来源
| 源文件 |
|--------|
| `v4/include/sbf/voxel/hull_rasteriser.h` (~60 LOC) |

---

## Step N6: EnvelopeType 扩展

### 现有 EnvelopeType (Phase C)
```cpp
enum class EnvelopeMethod {
    LinkIAABB,          // per-link AABB
    LinkIAABB_Grid,     // per-link + grid subdivision
    // 新增 ↓
    Hull16_Grid,        // Hull-16 voxel rasterization
};
```

### 新增: Hull16_Grid 评估
```cpp
// envelope_type.cpp 中新增分支:
case EnvelopeMethod::Hull16_Grid: {
    VoxelGrid robot_voxels(delta, origin);
    rasterise_robot_hull16(robot_voxels, robot, box, 16);

    VoxelGrid obs_voxels(delta, origin);
    for (int i = 0; i < n_obs; ++i)
        rasterise_box_obstacle(obs_voxels, obs[i]);

    return !robot_voxels.collides(obs_voxels);
}
```

---

## Step N7: CMake 集成

### CMakeLists.txt 变更
```cmake
# sbf5 library 新增源文件:
    src/voxel/voxel_grid.cpp
    src/voxel/hull_rasteriser.cpp
```

无额外依赖。

---

## Step N8: 测试

### 文件
`test/test_voxel.cpp`

| 用例 | 描述 |
|------|------|
| `bitbrick_set_test` | set(3,4,5) → test(3,4,5) == true |
| `bitbrick_popcount` | 填充 10 个 → popcount == 10 |
| `bitbrick_intersects` | 两个 brick 有/无交集 |
| `voxel_fill_aabb` | fill_aabb → count_occupied 在预期范围 |
| `voxel_merge` | 两 grid merge → count ≥ max(a, b) |
| `voxel_collides` | 重叠 AABB → collides == true |
| `voxel_no_collide` | 分离 AABB → collides == false |
| `hull16_vs_aabb` | hull16 occupied ≤ aabb occupied (更紧密) |
| `rasterise_robot_2dof` | 2DOF box → voxel 非空 |
| `flat_brick_map_rehash` | 插入 >70% capacity → rehash 正确 |

### CMake 新增
```cmake
# SBF5_TESTS list 中追加:
    test_voxel
```

---

## Step N9: VizExporter Voxel 支持 (与 Phase I 联动)

### Phase I 新增导出函数
```cpp
namespace sbf::viz {

/// 导出 voxel grid 数据 (brick-level, hex-encoded)
void export_voxel_json(
    const std::string& path,
    const VoxelGrid& grid
);

/// 导出 voxel centroids (Cartesian)
void export_voxel_centres_json(
    const std::string& path,
    const VoxelGrid& grid
);

}
```

### JSON Schema
```json
{
  "type": "voxel",
  "delta": 0.01,
  "origin": [0, 0, 0],
  "n_bricks": 42,
  "total_occupied": 1500,
  "bricks": [
    {"coord": [0, 0, 0], "popcount": 100, "words": ["ff00...hex"]}
  ]
}
```

---

## 验收标准

- [x] BitBrick 所有操作正确 (set/test/popcount/intersects)
- [x] FlatBrickMap 在高负载 (>70%) 下自动 rehash
- [x] fill_aabb 输出 voxel 数量在理论范围内 (±1 brick margin)
- [x] fill_hull16 输出 ≤ fill_aabb 输出 (更紧密)
- [x] 碰撞检测: 重叠=true, 分离=false
- [x] 22 个测试全部通过 (246 assertions)
- [x] 无平台特定编译问题 (MSVC popcount / GCC builtin)
