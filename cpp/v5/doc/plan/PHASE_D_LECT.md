# Phase D: LECT — Link Envelope Collision Tree (Module 3)

> 依赖: Phase B (Endpoint IAABB) + Phase C (Link IAABB)
> 状态: ✅ 已完成 (2026-04-04, 补丁 2026-04-05)
> 产出: LECT KD-tree + envelope 缓存 + split 策略 + Z4 对称性

---

## 目标

实现 LECT: 一棵 KD-tree，每个节点缓存对应 C-space 子区间的 link envelope。
这是 FFB (Find Free Box) 和 Grower 的核心数据结构。

### 默认启用特性 (2026-04-05 补丁)

| 特性 | 默认值 | API 控制 |
|------|--------|----------|
| Z4 对称性缓存 | 自动检测并启用 | `SBFPlannerConfig::z4_enabled` / `LECT::disable_z4()` |
| BEST_TIGHTEN 分割 | 默认启用 (minimax volume) | `SBFPlannerConfig::split_order` / `LECT::set_split_order()` |
| Z4 cache source 兼容性 | `source_can_serve()` 校验 + 自动升级 | 内部自动 |
| BEST_TIGHTEN 相对容差 | `1e-9 * max(1, max(|m|, |best|))` | 内部自动 |

---

## 架构概览

```
LECT = KD-tree over C-space
  每个 node 代表一个 C-space hyperrectangle
  每个 node 缓存:
    - endpoint_iaabbs [ep_stride floats]
    - link_iaabbs     [liaabb_stride floats]
    - source_quality  (EndpointSource enum)
    - has_data        (bool)
    - occupied        (box_id or -1)

树结构 (flat buffer, cache-friendly):
  left[], right[], parent[], depth[]
  split_dim[], split_val[]
```

### v5 简化（对比 v4）
- **单通道**: 去掉 v4 的 dual-channel (SAFE=0, UNSAFE=1)
- **无 hull_grids**: 去掉 per-node VoxelGrid（Hull16_Grid 延后）
- **无 hull_valid**: 对应去掉

---

## Step D1: LECT 核心 KD-tree

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/lect/lect.h` | `src/lect/lect.cpp` |

### 核心类定义
```cpp
class LECT {
public:
    LECT(const Robot& robot, const std::vector<Interval>& root_intervals,
         const EndpointSourceConfig& ep_config,
         const EnvelopeTypeConfig& env_config);

    // --- 元信息 ---
    int n_nodes() const;
    int n_dims() const;
    int n_active_links() const;
    int capacity() const;

    // --- 树结构访问 ---
    int  left(int i)   const;
    int  right(int i)  const;
    int  parent(int i) const;
    int  depth(int i)  const;
    bool is_leaf(int i) const;
    int  get_split_dim(int i) const;
    double split_val(int i) const;
    std::vector<Interval> node_intervals(int i) const;  // 重建 C-space 区间

    // --- Envelope 数据访问 ---
    bool has_data(int i) const;
    const float* get_endpoint_iaabbs(int i) const;   // [ep_stride]
    const float* get_link_iaabbs(int i) const;        // [liaabb_stride]
    EndpointSource source_quality(int i) const;

    // --- 核心操作 ---
    int expand_leaf(int node_idx);  // 分裂叶节点, 返回 left child idx
    void compute_envelope(int node_idx, FKState& fk,
                          const std::vector<Interval>& intervals,
                          int changed_dim = -1,
                          int parent_idx = -1);

    // --- 碰撞查询 ---
    bool collides_scene(int node_idx, const Obstacle* obs, int n_obs) const;
    bool intervals_collide_scene(const std::vector<Interval>& intervals,
                                 const Obstacle* obs, int n_obs) const;

    // --- 占用管理 (forest boxing) ---
    void mark_occupied(int node_idx, int box_id);
    void unmark_occupied(int node_idx);
    bool is_occupied(int node_idx) const;
    int  forest_id(int node_idx) const;  // 返回占用的 box_id, -1 if free

    // --- Split 策略 ---
    void set_split_order(SplitOrder so);
    SplitOrder split_order() const;

    // --- Z4 对称性 ---
    const JointSymmetry& symmetry_q0() const;

private:
    // Flat buffer storage
    std::vector<float>   ep_data_;           // [capacity × ep_stride_]
    std::vector<float>   link_iaabb_cache_;  // [capacity × liaabb_stride_]
    std::vector<uint8_t> source_quality_;    // [capacity]
    std::vector<uint8_t> has_data_;          // [capacity]

    // Tree structure
    std::vector<int>    left_, right_, parent_, depth_;
    std::vector<int>    split_dim_;
    std::vector<double> split_val_;

    // Occupation
    std::vector<int>    forest_id_;          // [capacity], -1 = free

    // Config
    int ep_stride_;       // n_active_links * 2 * 6
    int liaabb_stride_;   // n_active_links * 6
    int n_nodes_;
    int capacity_;

    // Z4 cache
    struct Z4CacheEntry {
        std::vector<float> ep_iaabbs;
        EndpointSource source;
    };
    std::unordered_map<uint64_t, Z4CacheEntry> z4_cache_;
    JointSymmetry symmetry_q0_;
};
```

### 内存布局
```
Node i 的 endpoint iAABBs:  ep_data_[i * ep_stride_ ... (i+1) * ep_stride_)
Node i 的 link iAABBs:      link_iaabb_cache_[i * liaabb_stride_ ... (i+1) * liaabb_stride_)
```
- `ep_stride_ = n_active_links × 2 × 6` (每个 link 2 个 endpoint, 每个 6 float)
- `liaabb_stride_ = n_active_links × 6`

### 容量增长
- 初始 capacity = 1023 (10 层满二叉树)
- 不足时 2x 扩容, 重新分配所有 flat buffer

### 迁移来源
| 源版本 | 源文件 | 取什么 |
|--------|--------|--------|
| v4 | `safeboxforest/v4/include/sbf/forest/lect.h` | 类结构, flat buffer 布局 |
| v4 | `safeboxforest/v4/src/forest/lect.cpp` | 核心算法 |

### 与 v4 差异
- 去掉 `ChannelIdx`, `ChannelData` — 单通道
- 去掉 `hull_grids`, `hull_valid` — 无 Hull16
- 去掉 `derive_link_iaabbs(node, out, ch)` 的 channel 参数

---

## Step D2: Split 策略

### 枚举
```cpp
enum class SplitOrder : uint8_t {
    ROUND_ROBIN   = 0,  // 按 depth 循环 [0, 1, ..., n_dims-1]
    WIDEST_FIRST  = 1,  // 选当前区间最宽维度
    BEST_TIGHTEN  = 2   // iFK probe minimax (默认)
};
```

### ROUND_ROBIN
```
split_dim = depth % n_dims
```

### WIDEST_FIRST
```
split_dim = argmax_d(intervals[d].width())
```

### BEST_TIGHTEN ⭐ (推荐默认)
```
for each dim d with width > 0:
    intervals_left  = intervals; intervals_left[d].hi = midpoint(d)
    intervals_right = intervals; intervals_right[d].lo = midpoint(d)

    fk_left  = compute_fk_incremental(fk, robot, intervals_left, d)
    fk_right = compute_fk_incremental(fk, robot, intervals_right, d)

    vol_left  = sum_link_volumes(fk_left)
    vol_right = sum_link_volumes(fk_right)

    metric[d] = max(vol_left, vol_right)  // minimax

split_dim = argmin_d(metric[d])
```

**要点**: 选择分裂后最大子节点 envelope volume 最小的维度 → 最有效的碰撞排除.

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v4 | `safeboxforest/v4/src/forest/lect.cpp` 中 `pick_best_tighten_dim()` |

### 验收标准
- [x] BEST_TIGHTEN 子节点 max volume < ROUND_ROBIN 子节点 max volume（同深度）
- [x] WIDEST_FIRST 总是选最宽维度

---

## Step D3: Z4 对称性优化

### 原理
当 joint 0 的区间 span ≈ 2π (full rotation) 时：
- 将 q₀ 规范化到 [0, period/4) 的规范 sector
- 用规范 sector 的 endpoint iAABBs + rotation transform 推导其他 sector
- FNV-1a hash on canonical interval → cache key

### 数据结构
```cpp
struct Z4CacheEntry {
    std::vector<float> ep_iaabbs;  // canonical sector endpoint iAABBs
    EndpointSource source;
};

// LECT 成员
std::unordered_map<uint64_t, Z4CacheEntry> z4_cache_;
JointSymmetry symmetry_q0_;
```

### 使用流程（在 `compute_envelope` 中）
```
1. 检测 symmetry_q0_.type == Z4_ROTATION
2. 将 intervals[0] 规范化 → canonical + sector_id
3. 计算 cache_key = fnv1a_hash(canonical_interval + intervals[1..n])
4. if cache hit (sector != 0):
     → 从 canonical entry 通过 rotation transform 推导 ep_iaabbs
     → derive link_iaabbs
     → return (跳过 FK 计算)
5. if cache miss || sector == 0:
     → 正常 compute (IFK/Analytical/GCPC)
     → if sector == 0: 存入 cache
```

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v4 | `safeboxforest/v4/include/sbf/core/joint_symmetry.h` |
| v4 | `safeboxforest/v4/src/forest/lect.cpp` Z4 cache 逻辑 |

### 验收标准
- [x] full-rotation joint: Z4 cache hit 率 > 50%
- [x] cache hit 结果与直接计算结果一致（float 精度内）
- [x] 非 full-rotation joint: Z4 cache 不启用

---

## Step D4: Envelope 计算集成

### `compute_envelope()` 流程

```
compute_envelope(node_idx, fk, intervals, changed_dim, parent_idx):

  // 1. Z4 cache 快速路径
  if symmetry_q0_.type == Z4_ROTATION:
    if z4_cache_hit → apply transform → derive link iaabbs → return

  // 2. IFK fast path (最常见)
  if config.source == IFK:
    if changed_dim >= 0 && parent_idx >= 0 && fk.valid:
      // 增量 FK: 只更新 changed_dim 影响的 link
      compute_fk_incremental(fk, robot, intervals, changed_dim)
      // Partial inheritance: 未变 link 从 parent 复制 ep_data
      for each active_link ci:
        if link_chain_index(ci) + 1 <= changed_dim:
          copy ep_data from parent
        else:
          extract from fk
    else:
      compute_fk_full(fk, robot, intervals)
      extract all ep_data from fk
    derive_link_iaabb_paired(ep_data, ...)
    return

  // 3. Analytical / GCPC path
  result = compute_endpoint_iaabb(robot, intervals, config)
  store ep_data
  derive link iaabbs
```

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v4 | `safeboxforest/v4/src/forest/lect.cpp` → `compute_envelope()` |

---

## Step D5: 碰撞查询

### `collides_scene()`
```cpp
bool collides_scene(int node_idx, const Obstacle* obs, int n_obs) const {
    const float* liaabbs = get_link_iaabbs(node_idx);
    for (int ci = 0; ci < n_active_links_; ++ci) {
        const float* lb = liaabbs + ci * 6;
        // lb = [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
        for (int oi = 0; oi < n_obs; ++oi) {
            const float* ob = obs[oi].bounds;
            if (lb[0] <= ob[3] && lb[3] >= ob[0] &&   // x overlap
                lb[1] <= ob[4] && lb[4] >= ob[1] &&   // y overlap
                lb[2] <= ob[5] && lb[5] >= ob[2])     // z overlap
                return true;
        }
    }
    return false;
}
```

---

## 测试: test_lect.cpp

```
TEST_SUITE("LECT_Structure") {
    - 初始化后 n_nodes == 1, root 有 data
    - expand_leaf → n_nodes == 3 (root + 2 children)
    - node_intervals 重建正确
}

TEST_SUITE("LECT_Split") {
    - ROUND_ROBIN: split_dim = depth % n_dims
    - WIDEST_FIRST: split_dim = widest
    - BEST_TIGHTEN: max child vol < ROUND_ROBIN
}

TEST_SUITE("LECT_Envelope") {
    - compute_envelope → has_data == true
    - link_iaabbs 保守 (MC 验证)
    - 增量 compute 与 full 一致
}

TEST_SUITE("LECT_Z4") {
    - full-rotation joint: cache hit 率 > 50% (10 random intervals)
    - cache hit 与直接计算一致
    - non-full-rotation: cache 不启用
}

TEST_SUITE("LECT_Collision") {
    - 已知碰撞节点 → collides_scene == true
    - 已知安全节点 → collides_scene == false
}

TEST_SUITE("LECT_Occupation") {
    - mark_occupied → is_occupied == true, forest_id == box_id
    - unmark_occupied → is_occupied == false
}
```
