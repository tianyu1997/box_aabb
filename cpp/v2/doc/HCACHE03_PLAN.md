# HCACHE03 Multi-Envelope Universal Cache — Implementation Plan

> SafeBoxForest v2 C++ — 2026-03-08

## 1. 目标

将 LECT 的节点主存从 per-link AABB (`float[n_active_links × 6]`) 扩展为
**per-frame position intervals** (`float[n_stored_frames × 6]`)，作为所有
envelope 类型的通用基元。从此基元可 O(1) 导出 AABB、link-subdivided AABB、
IFK OBB、occupancy grid，无需重新运行 FK。

同时实现多种碰撞检测策略（collision policy），通过配置开关切换，最终
benchmark 对比选优。

## 2. 数据模型

### 2.1 Frame Position Intervals

每个 LECT 节点存储 `n_stored_frames` 个 frame 的 3D 位置区间：

```
frames[k] = [x_lo, y_lo, z_lo, x_hi, y_hi, z_hi]   (k = 0 .. n_stored_frames-1)
```

其中 frame k 对应 FK prefix transform `T^{0:k+1}` 的平移列
（跳过 base frame f_0 = [0,0,0]，恒为常量不存储）。

对 IIWA14 7-DOF + tool：`n_stored_frames = 8`（joint 0..6 + tool）。

### 2.2 从 frames 导出各类 Envelope

| Envelope | 导出方式 | 每节点开销 |
|----------|---------|-----------|
| AABB | `bbox(frames[link-1], frames[link]) ⊕ r` | ~20ns/link |
| AABB subdivided | interval lerp + bbox | ~50ns/link×n_sub |
| IFK OBB | interval dot product 投影到 3 轴 | ~60ns/link |
| Grid (R=32) | sub-AABB 体素化 | ~2µs total |

### 2.3 向后兼容

- 现有 AABB-only 路径保持不变（`StoreFormat::AABB_LEGACY`）
- `StoreFormat::FRAMES` 为新增模式
- 碰撞检测通过 `CollisionPolicy` 枚举切换

## 3. 文件变更清单

### 新增文件（7 个）

| # | 文件 | 内容 |
|---|------|------|
| H1 | `include/sbf/envelope/frame_store.h` | `FrameStore` — per-node frame position interval 存储 |
| H2 | `include/sbf/envelope/envelope_derive.h` | 导出函数：frames → AABB / OBB / subdivided AABB |
| H3 | `include/sbf/envelope/collision_policy.h` | `CollisionPolicy` 枚举 + 统一碰撞检测调度 |
| H4 | `include/sbf/envelope/grid_envelope.h` | `GridEnvelope`, `GridComputer` |
| S1 | `src/envelope/frame_store.cpp` | FrameStore 实现 |
| S2 | `src/envelope/envelope_derive.cpp` | 导出函数实现 |
| S3 | `src/envelope/grid_envelope.cpp` | Grid 生成/碰撞/RLE 压缩实现 |

### 修改文件（4 个）

| # | 文件 | 变更 |
|---|------|------|
| M1 | `include/sbf/common/config.h` | `EnvelopeConfig` 新增 `CollisionPolicy`, store_format, grid, subdivision 字段 |
| M2 | `include/sbf/envelope/envelope.h` | `EnvelopeKind` 增加 `Grid=2`（已有枚举值预留） |
| M3 | `include/sbf/forest/lect.h` | LECT 持有 `FrameStore`，`get_envelope()` 增加 frames 导出路径 |
| M4 | `CMakeLists.txt` | `sbf_envelope` target 添加新 .cpp 文件 |

## 4. 详细设计

### Phase 0 — 配置 (`config.h`)

在 `envelope::EnvelopeConfig` 中新增：

```cpp
// 主存格式
enum class StoreFormat : uint8_t { AABB_LEGACY = 0, FRAMES = 1 };
StoreFormat store_format = StoreFormat::FRAMES;

// 碰撞检测策略
enum class CollisionPolicy : uint8_t {
    AABB         = 0,  // 仅 per-link AABB-SAT
    AABB_SUBDIV  = 1,  // AABB + 碰撞时 link subdivision fallback
    OBB_IFK      = 2,  // 从 frames 导出 IFK OBB → 15-axis SAT
    GRID         = 3,  // 从 frames 生成 occupancy grid → bitwise AND
    TWO_PASS     = 4,  // AABB_SUBDIV 初筛 + OBB 精炼
};
CollisionPolicy collision_policy = CollisionPolicy::AABB;

// Link subdivision
int link_subdivision_n = 0;           // 0=不细分, >0=固定段数, -1=自适应
int link_subdivision_max = 8;         // 自适应最大段数

// Grid
int grid_resolution = 32;            // R³ 体素
float grid_world_bounds[6] = {-0.8f, -1.2f, 0.0f, 1.8f, 1.2f, 1.4f};
```

### Phase 1 — FrameStore (`frame_store.h/cpp`)

```cpp
class FrameStore {
    // 存储: flat float array [n_nodes × n_frames × 6]
    // 内存布局: frames_[node_idx * n_frames_ * 6 + frame * 6 + {0..5}]
    
    int n_frames_;           // = n_joints + has_tool
    int n_active_links_;     // compact active link count
    int active_link_map_[MAX_LINKS];  // compact idx → frame idx
    float base_pos_[3];     // base frame position (constant, default [0,0,0])
    float link_radii_[MAX_LINKS];     // per active link capsule radius
    
    // 核心 API
    void store_from_fk(int node_idx, const FKState& fk);
    void store_frames(int node_idx, const float* frames_6xN);
    const float* get_frames(int node_idx) const;
    
    // Union / Refine (frame-level)
    void union_frames(int dst, int a, int b);
    void refine_frames(int dst, int union_src);
    
    // 容量管理
    void ensure_capacity(int n_nodes);
};
```

`store_from_fk` 从 `FKState` 提取所有 frame 的 `prefix[k+1, :3, 3]`
（translation 列的 lo/hi），写入 flat array。

### Phase 2 — Envelope Derive (`envelope_derive.h/cpp`)

纯函数，无状态：

```cpp
// AABB: frames → per-link AABB
void derive_aabb(
    const float* frames,  int n_frames,
    const int* active_link_map, int n_active,
    const float* link_radii, const float* base_pos,
    float* out_aabb);     // [n_active × 6]

// Subdivided AABB: 单条 link 的 n_sub 段
void derive_aabb_subdivided(
    const float* frames, int n_frames,
    int link_frame_idx,   // distal frame index (0-based in frames array)
    int parent_frame_idx, // proximal frame index (-1 = use base_pos)
    int n_sub, float radius, const float* base_pos,
    float* out_sub_aabbs);  // [n_sub × 6]

// IFK OBB: frames → per-link OBB (15 floats/link)
void derive_ifk_obb(
    const float* frames, int n_frames,
    const int* active_link_map, int n_active,
    const float* link_radii, const float* base_pos,
    float* out_obb);      // [n_active × 15]

// Grid: frames → R³ bitset
void derive_grid(
    const float* frames, int n_frames,
    const int* active_link_map, int n_active,
    const float* link_radii, const float* base_pos,
    const float* world_bounds, int R,
    int n_sub_per_link,
    uint8_t* out_grid);   // [R³] or ceil(R³/8) bytes for bitset
```

### Phase 3 — CollisionPolicy (`collision_policy.h`)

统一碰撞检测入口，内部按 policy 分派：

```cpp
// 返回 true = 碰撞 (box 不安全)
bool check_collision(
    CollisionPolicy policy,
    const float* frames, int n_frames,        // node frame data
    const int* active_link_map, int n_active,
    const float* link_radii, const float* base_pos,
    const float* obs_compact, int n_obs,      // obstacle AABBs
    // ── policy-specific params ──
    int subdiv_n, int subdiv_max,             // for AABB_SUBDIV
    int grid_R, const float* grid_world_bounds, // for GRID
    float* workspace_buf = nullptr);          // pre-allocated temp buffer

// Legacy: 直接用已有 AABB 数据检测（无 FrameStore 时）
bool check_collision_aabb(
    const float* link_aabbs, int n_links,
    const float* obs_compact, int n_obs);
```

各 policy 实现（均在 .cpp 中，编译期完全展开）：

| Policy | 逻辑 |
|--------|------|
| `AABB` | `derive_aabb` → 3-axis SAT, early exit per link |
| `AABB_SUBDIV` | AABB 初测 → 碰撞时 `derive_aabb_subdivided` 精测 |
| `OBB_IFK` | `derive_ifk_obb` → 15-axis SAT |
| `GRID` | `derive_grid` → `grid & obs_grid` bitwise AND |
| `TWO_PASS` | AABB_SUBDIV pass 1 → OBB_IFK pass 2 on colliding links |

### Phase 4 — Grid Envelope (`grid_envelope.h/cpp`)

```cpp
class GridEnvelope : public IEnvelope {
    std::vector<uint8_t> grid_;  // R³ bytes (1B per voxel)
    int R_;
    float world_bounds_[6];
    float cell_size_[3];
    
    // IEnvelope
    bool collides(const float* obs_compact, int n_obs) const override;
    double volume() const override;
    std::vector<uint8_t> serialise() const override;  // RLE 压缩
    static EnvelopePtr deserialise(const uint8_t* data, size_t len);
};

// Grid vs obstacle: 体素化障碍物 AABB → bitwise AND
bool grid_obs_collide(const uint8_t* grid, int R,
                      const float* world_bounds,
                      const float* obs_compact, int n_obs);

// RLE 压缩/解压 (Z-slice RLE)
std::vector<uint8_t> grid_compress_rle(const uint8_t* grid, int R);
void grid_decompress_rle(const uint8_t* data, size_t len, int R, uint8_t* out);
```

### Phase 5 — LECT 集成 (`lect.h/cpp`)

```cpp
class LECT {
    HierAABBTree       tree_;      // KD-tree 拓扑 + 旧式 AABB (legacy compat)
    FrameStore         frames_;    // 新: per-node frame position intervals
    MultiEnvelopeCache cache_;     // sidecar: OBB_Critical, Grid 等
    // ...
    
    // 新增: 从 frames 按需导出
    void derive_aabb_for(int node_idx, float* out) const;
    void derive_obb_for(int node_idx, float* out) const;
    
    // 碰撞检测: 使用配置的 CollisionPolicy
    bool check_node_collision(int node_idx,
                              const float* obs_compact, int n_obs) const;
};
```

### Phase 6 — CMakeLists.txt

`sbf_envelope` target 新增 3 个 .cpp：

```cmake
add_library(sbf_envelope STATIC
    src/envelope/envelope_computer.cpp
    src/envelope/aabb_envelope.cpp
    src/envelope/obb_envelope.cpp
    src/envelope/envelope_cache.cpp
    src/envelope/multi_env_cache.cpp
    src/envelope/frame_store.cpp          # NEW
    src/envelope/envelope_derive.cpp      # NEW
    src/envelope/grid_envelope.cpp        # NEW
)
```

## 5. 磁盘开销

| Robot | Active Links | n_frames | FrameStore/node | AABB-only/node | 增幅 |
|-------|-------------|----------|-----------------|----------------|------|
| IIWA14 7-DOF + tool | 5 | 8 | 192 B | 120 B | 1.6× |
| Panda 7-DOF + tool | 4 | 8 | 192 B | 96 B | 2.0× |

1M 节点：192 MB（mmap 懒加载时 RAM 仅为已访问页面）。

## 6. 实施顺序

```
Phase 0  config.h 新增枚举与字段               ← 5 min
Phase 1  frame_store.h/cpp                     ← 核心存储
Phase 2  envelope_derive.h/cpp                 ← 导出层
Phase 3  collision_policy.h (调度 + 各 policy)  ← 碰撞检测
Phase 4  grid_envelope.h/cpp                   ← Grid envelope
Phase 5  lect.h/cpp 集成 FrameStore            ← 接入
Phase 6  CMakeLists.txt                        ← 编译
```

## 7. 验证计划

1. **数值一致性**: `derive_aabb(stored_frames)` ≡ `extract_link_aabbs(FKState)` 误差 < 1e-6
2. **Link subdivision 包含性**: `union(sub_aabbs) ⊆ full_link_aabb`
3. **OBB 保守性**: OBB vs AABB 投影一致
4. **Grid union 无膨胀**: `grid(parent) ⊇ grid(child_L) | grid(child_R)`
5. **Policy 回归**: 各 policy 对同一场景的 FFB 结果与 AABB baseline 一致

## 8. 实施状态 (2026-03-08)

| Phase | 文件 | 状态 |
|-------|------|------|
| 0 | `include/sbf/common/config.h` — `StoreFormat`, `CollisionPolicy` 枚举 + `EnvelopeConfig` 新字段 | ✅ Done |
| 1 | `include/sbf/envelope/frame_store.h` + `src/envelope/frame_store.cpp` | ✅ Done |
| 2 | `include/sbf/envelope/envelope_derive.h` + `src/envelope/envelope_derive.cpp` | ✅ Done |
| 3 | `include/sbf/envelope/collision_policy.h` + `src/envelope/collision_policy.cpp` | ✅ Done |
| 4 | `include/sbf/envelope/grid_envelope.h` + `src/envelope/grid_envelope.cpp` | ✅ Done |
| 5 | `include/sbf/forest/lect.h` + `src/forest/lect.cpp` — FrameStore 集成 | ✅ Done |
| 6 | `CMakeLists.txt` — 新增 3 个 .cpp 到 sbf_envelope | ✅ Done |

### 新增文件清单

**Headers (4):**
- `include/sbf/envelope/frame_store.h` — FrameStore 类
- `include/sbf/envelope/envelope_derive.h` — derive_aabb / derive_ifk_obb / derive_grid 等纯函数
- `include/sbf/envelope/collision_policy.h` — 5 策略调度 + OBB-AABB SAT + legacy AABB
- `include/sbf/envelope/grid_envelope.h` — GridEnvelope + GridComputer + RLE 压缩

**Sources (4):**
- `src/envelope/frame_store.cpp`
- `src/envelope/envelope_derive.cpp`
- `src/envelope/collision_policy.cpp`
- `src/envelope/grid_envelope.cpp`

**Modified (3):**
- `include/sbf/common/config.h`
- `include/sbf/forest/lect.h`
- `src/forest/lect.cpp`
- `CMakeLists.txt`

## 9. 旧版缓存代码清理 (2026-03-09)

HCACHE03 实施完成后，旧版 envelope cache / wrapper 代码已从代码库中移除，
仅保留 Python 绑定所需的最小兼容层。

### 已删除文件 (8)

| 文件 | 描述 |
|------|------|
| `include/sbf/envelope/envelope_cache.h` | EnvelopeRegistry, InMemoryEnvelopeCache, FileEnvelopeCache |
| `src/envelope/envelope_cache.cpp` | 262 行 |
| `include/sbf/envelope/multi_env_cache.h` | MultiEnvelopeCache |
| `src/envelope/multi_env_cache.cpp` | 132 行 |
| `include/sbf/envelope/aabb_envelope.h` | AabbEnvelope, IntervalFKComputer, CriticalComputer, RandomComputer |
| `src/envelope/aabb_envelope.cpp` | 393 行 |
| `include/sbf/envelope/obb_envelope.h` | ObbEnvelope, ObbIntervalFKComputer, ObbCriticalComputer |
| `src/envelope/obb_envelope.cpp` | 626 行 |

### 已重写文件 (2)

| 文件 | 变更 |
|------|------|
| `include/sbf/forest/lect.h` | 移除 MultiEnvelopeCache / IEnvelopeComputer registry / get_envelope / cache() 等；仅保留 HierAABBTree + FrameStore + EnvelopeConfig + collision_policy |
| `src/forest/lect.cpp` | 构造函数简化为 `LECT(robot)`（无 cache_dir）；load/load_mmap 改为 2 参数 |

### 已修改文件 (6)

| 文件 | 变更 |
|------|------|
| `CMakeLists.txt` | 从 sbf_envelope 移除 4 个旧 .cpp |
| `python/bindings.cpp` | EnvelopeResult / IntervalFKEnvelopeComputer 标记为 LEGACY |
| `python/forest_viz.py` | 更新 docstring 中对 obb_envelope.cpp 的引用 |
| `docs/API_REFERENCE.md` / `API_REFERENCE_zh.md` | Envelope 章节添加弃用通知 |
| `docs/USER_GUIDE.md` / `USER_GUIDE_zh.md` | 模块表格标注旧版/新版 API |

### 保留的兼容文件 (3)

| 文件 | 保留原因 |
|------|----------|
| `include/sbf/envelope/envelope.h` | IEnvelope 接口 — GridEnvelope 仍然继承 |
| `include/sbf/envelope/envelope_computer.h` | 旧版 IntervalFKEnvelopeComputer 类 — Python 绑定依赖 |
| `src/envelope/envelope_computer.cpp` | 同上，仅供 Python 绑定使用 |

### 已移除的 LECT API

以下方法从 `lect.h` / `lect.cpp` 中移除，在整个代码库中已无引用：
`register_computer`, `has_computer`, `computer_for`, `registered_kinds`,
`get_envelope`, `get_cached_envelope`, `has_envelope`, `precompute_all`,
`invalidate_node`, `flush_caches`, `cache()`, `make_aabb_envelope()`

### 构造函数签名变更

```cpp
// 旧版
LECT(const Robot& robot, const std::string& cache_dir);
LECT load(const fs::path& path, const Robot& robot, const std::string& cache_dir);

// 新版 (HCACHE03)
LECT(const Robot& robot);
LECT load(const fs::path& path, const Robot& robot);
```

## 10. Experiment 14 Results & Analysis (2026-03-08)

### 10.1 Setup

- Robot: Panda (7 DOF, 3 active links)
- Obstacles: 6 (shelves, poles, walls, tabletop)
- Trials: 200 per width regime × 3 regimes = 600
- Repeats: 10 (median timing)
- Grid R: 32³

### 10.2 Bugs Found & Fixed

1. **`derive_ifk_obb` 世界坐标中心错误** — OBB center 存储为轴投影坐标
   而非世界坐标，导致 `obb_aabb_sat` 计算距离向量错误。
   修复后 OBB false negatives 从 10/150 降至 0。

2. **Benchmark `volume_obb` 读取错误偏移** — 读 `o[12..14]`（az 轴）
   而非 `o[3..5]`（half-extents），导致 OBB 体积显示为 ~0。

3. **一致性分析改进** — 更紧方法（SUBDIV/OBB/GRID/TWO_PASS）拒绝
   AABB 误报是正确行为，不应计为 violation。重新分类为 "tighten"。

### 10.3 Collision Rejection Rate (关键指标)

| Width | AABB | SUBDIV(2) | SUBDIV(4) | OBB | GRID | TWO_PASS(2) |
|---|---|---|---|---|---|---|
| small | 27.5% | 26.5% | 26.0% | **35.0%** | 32.0% | 26.0% |
| medium | 66.0% | 65.5% | 64.5% | **92.0%** | 71.0% | 64.5% |
| large | 100% | 100% | 100% | 100% | 100% | 100% |

> 越低越好（= 越少误报 = forest 中可构建更大的安全盒）

**关键发现**: OBB_IFK 反而比 AABB 更保守（92% vs 66%），因为区间投影的
cross-term widening 使 OBB half-extent 膨胀。对轴对齐障碍物场景，OBB 不如 AABB。

### 10.4 Performance (cached collision check, µs)

| Policy | Cached µs | Speedup vs cold |
|---|---|---|
| REF_V1 (legacy) | 0.0 | 4600x |
| AABB | 0.0–0.1 | 4600x |
| AABB_SUBDIV | 0.1 | 46x |
| OBB_IFK | 0.2 | 24x |
| TWO_PASS | 0.1–0.2 | 24–48x |
| GRID | 0.9–5.1 | 2–7x |

Cold path (~4.7 µs) 主要由 FK 计算支配，所有策略开销相近。

### 10.5 Consistency

| Policy | Agree | Tighten | ExtraReject |
|---|---|---|---|
| AABB | 600/600 | 0 | 0 |
| AABB_SUBDIV(4) | 594 | 6 | 0 |
| OBB_IFK | 527 | 3 | 70 |
| GRID | 575 | 3 | 22 |
| TWO_PASS(4) | 594 | 6 | 0 |

- AABB 与 REF_V1 完全匹配（max error = 0.0）
- 18 次 tightening = 更紧方法正确拒绝 AABB 误报
- 0 次 false negative

### 10.6 Storage

| Storage | Bytes/node | Notes |
|---|---|---|
| FrameStore (universal) | 192 | 存一次，按需导出一切 |
| AABB (derived) | 72 | 临时 |
| AABB_SUBDIV(n=4) | 288 | 临时 |
| OBB_IFK (derived) | 180 | 临时 |
| GRID (R=32) | 32768 | 不可行 |
| REF_V1 (legacy) | 72 | 每节点永久存储 |

### 10.7 Recommendations

1. **默认策略: `AABB_SUBDIV` (n=2–4)**
   - 碰撞率比 AABB 低 1–3%（small/medium 区间）
   - 缓存检测时间 0.1 µs，与 AABB 相当
   - 存储 192 B/node（仅 FrameStore）

2. **OBB_IFK: 不推荐用于轴对齐障碍物**
   - 区间投影 cross-term widening 使体积膨胀 2.4x
   - 碰撞率反而高于 AABB（92% vs 66% for medium）
   - 仅在障碍物非轴对齐时可能有益

3. **GRID: 不推荐**
   - R=32 时体积膨胀 13.5x，最慢（5 µs cached for large）
   - 存储 32KB/node 不可行

4. **TWO_PASS: 等价于 AABB_SUBDIV**
   - OBB refine 步骤未提供额外收益（TWO_PASS 碰撞率 = AABB_SUBDIV）
