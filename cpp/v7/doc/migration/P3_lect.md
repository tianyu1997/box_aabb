# P3 — LECT：Link Envelope Collision Tree（最小可用版）

> **目标**: Port v6 `LECT` 至 `sbf::lect`，作为 v7 forest grower 的核心空间索引。
> 严格执行 D1（碰撞检测在线膨胀 link_radii，缓存零半径几何）。
>
> **预计工时**: 2-3 天  
> **前置条件**: P0 + P1 + P2 完成  
> **完成标准**: `ctest -R smoke_P3` 通过；与 v6 同构型小规模查询路径质量一致。

---

## 0. 重要范围说明（与初稿差异）

v6 LECT 实际是 **~6 K LOC** 的复杂模块（双通道、Z4 对称缓存、mmap COW、磁盘 V5/V6 IO、
Z4EpCache + Z4GridCache + LectCacheManager、L1 thread-local cache、grid lazy load、…）。
**P3 不在一个 phase 内复刻全部。** 选择最小可用子集：

| v6 特性 | v7 P3 决策 | 理由 |
|---------|-----------|-----|
| 双通道 (CH_SAFE/CH_UNSAFE) | **单通道** | v7 仅 IFK source（P1），CritSample 等已 cut |
| Z4 旋转对称缓存 | ❌ 删除 | 只对 IIWA/Panda 部分提速；正确性无关 |
| mmap COW + V5/V6 磁盘格式 | ❌ 删除 | 持久化是 P6 关注点；vector 即可 |
| LectCacheManager + Z4EpCache + Z4GridCache | ❌ 删除 | 性能优化，不影响 P3-P5 路径质量 |
| L1 thread-local cache | ❌ 删除 | 未启用并行前不需要 |
| Per-node grid (LinkIAABB_Grid) | ⏳ 延后 | P3 仅 LinkIAABB；Grid 在 P4 真用上时启用 |
| BEST_TIGHTEN 拆分 | ⚠️ 简化为 WIDEST_FIRST | 默认策略；BEST_TIGHTEN 留作 P3.5 优化 |
| TreeArray 模板 | ❌ 删除 | 直接 std::vector |
| Phase A collide cache（per-node verified） | ⚠️ 仅保留接口、暂不实现 | P4 grower 真用上时再激活 |
| snapshot() / transplant_subtree() | ✅ 保留（vector 深拷贝） | P4 并行 grower 必需 |
| **`collides_scene` 应用 link_radii** | ✅ **新增** | **D1 落地点** |

**v6 已知缺陷在 v7 P3 中的命运**:
- 假惰性加载：v7 P3 不做磁盘 IO，自然没有这个问题
- 无界内存：v7 P3 通过减小 capacity 增长策略和无 grid cache 自然受限
- CacheKey 含 link_radii：v6 实际并未在 cache key 含 radii（Explore 已证实）；P3 同样不含 → D1 一致
- 全局写锁 ms 级：v7 P3 提供 `Snapshot` + `transplant_subtree` 接口，P4 才用上

---

## 1. 模块职责

```
P3 管辖（cpp/v7/{include,src}/sbf/lect/）:
  lect.h, lect.cpp   - LECT 类（约 250 + 700 LOC）

P3 不管辖:
  IEndpointSource 多源切换    - 直接调 P1 ifk 函数（无虚函数）
  并行 grower 调度             - P4
  磁盘持久化                   - P6
```

---

## 2. 公共 API（命名空间 `sbf::lect`）

```cpp
namespace sbf::lect {

enum class SplitOrder : uint8_t {
    ROUND_ROBIN  = 0,
    WIDEST_FIRST = 1,   // v7 P3 默认
    // BEST_TIGHTEN  - 留作 P3.5
};

struct LECTConfig {
    SplitOrder split_order   = SplitOrder::WIDEST_FIRST;
    int        initial_cap   = 1023;
    double     min_norm_width = 1e-4;   ///< 跳过归一化宽度 < 此值的维度
};

class LECT {
public:
    LECT(const sbf::core::Robot& robot,
         const std::vector<sbf::core::Interval>& root_intervals,
         const sbf::envelope::EnvelopeTypeConfig& env_config,
         LECTConfig cfg = {});

    LECT(LECT&&) noexcept            = default;
    LECT& operator=(LECT&&) noexcept = default;
    LECT(const LECT&)                = default;   // deep copy snapshot
    LECT& operator=(const LECT&)     = default;

    /// Deep-copy snapshot（P4 worker 用）。设置 snapshot_base = n_nodes。
    LECT snapshot() const;

    // ── Tree introspection ────────────────────────────────────────────
    int n_nodes()        const { return n_nodes_; }
    int n_dims()         const { return n_dims_; }
    int n_active_links() const { return n_active_links_; }
    int snapshot_base()  const { return snapshot_base_; }
    int left(int i)      const { return left_[i]; }
    int right(int i)     const { return right_[i]; }
    int parent(int i)    const { return parent_[i]; }
    int depth(int i)     const { return depth_[i]; }
    bool is_leaf(int i)  const { return left_[i] < 0 && right_[i] < 0; }
    int  split_dim(int i)const { return split_dim_[i]; }
    double split_val(int i) const { return split_val_[i]; }

    std::vector<sbf::core::Interval> node_intervals(int i) const;

    /// Zero-radius link AABBs of node i: [n_active_links * 6].
    const float* get_link_iaabbs(int i) const;

    // ── Core operations ───────────────────────────────────────────────
    /// Split a leaf along chosen dim. Returns left child idx (right = left+1).
    int expand_leaf(int node_idx);

    /// Recompute envelope (zero-radius). May be called after expand_leaf.
    /// changed_dim allows incremental FK; pass -1 for full recompute.
    void compute_envelope(int node_idx,
                          const sbf::core::FKState& fk,
                          const std::vector<sbf::core::Interval>& intervals,
                          int changed_dim = -1,
                          int parent_idx = -1);

    // ── D1 collision check ────────────────────────────────────────────
    /// Test node i's zero-radius link AABBs (inflated by link_radii inline)
    /// against compact obstacles.
    bool collides_scene(int node_idx,
                        const float* obs_compact, int n_obs) const;

    // ── Geometric queries ─────────────────────────────────────────────
    int find_leaf_containing(const Eigen::VectorXd& q) const;
    bool is_descendant_of(int node, int ancestor) const;
    std::vector<int> partition_for_seeds(
        const std::vector<Eigen::VectorXd>& seeds,
        int max_extra_splits = 256);

    // ── Occupation (forest tracking) ──────────────────────────────────
    void mark_occupied(int node_idx, int box_id);
    void unmark_occupied(int node_idx);
    bool is_occupied(int node_idx) const;
    int  forest_id(int node_idx)   const;
    int  subtree_occ(int node_idx) const;
    void clear_all_occupation();
    bool is_point_occupied(const Eigen::VectorXd& q) const;
    void mark_point_occupied(const Eigen::VectorXd& q);

    // ── Free-volume tracking (Phase U) ────────────────────────────────
    double subtree_free_volume(int i) const;
    void   refresh_free_volumes();

    // ── Parallel merge ────────────────────────────────────────────────
    /// Append worker's nodes [snapshot_base..worker.n_nodes) into this tree,
    /// remapping local indices and box IDs.
    /// Returns number of nodes transplanted.
    int transplant_subtree(const LECT& worker, int snapshot_base,
                           const std::unordered_map<int, int>& id_map);

    // ── Accessors ─────────────────────────────────────────────────────
    const sbf::core::Robot& robot() const { return robot_; }
    const sbf::envelope::EnvelopeTypeConfig& env_config() const { return env_config_; }
    const std::vector<sbf::core::Interval>& root_intervals() const { return root_intervals_; }

private:
    // ... (implementation detail in lect.cpp)
};

}  // namespace sbf::lect
```

---

## 3. D1 落地：`collides_scene` 在线膨胀

```cpp
bool LECT::collides_scene(int i, const float* obs_compact, int n_obs) const {
    const float* zero_aabbs = get_link_iaabbs(i);   // 零半径！
    return sbf::scene::aabbs_collide_obs_inflated(
        zero_aabbs, n_active_links_,
        radii_per_slot_.data(),       // 预计算：robot.active_link_radii() → float
        obs_compact, n_obs);
}
```

`radii_per_slot_` 在构造时一次性从 `robot.active_link_radii()` 缓存为 float（n_active_links_ 项）。
LECT 自身**不存储** radii 副本之外的任何东西；改 radii 只需重建 LECT。

---

## 4. 数据成员（精简）

```cpp
private:
    // ── Tree structure（vector-only） ────────────────────────────
    std::vector<int>    left_, right_, parent_, depth_, split_dim_;
    std::vector<double> split_val_;

    // ── Envelope cache（zero-radius） ────────────────────────────
    std::vector<uint8_t> has_data_;             // 0/1
    std::vector<float>   ep_data_;              // [cap × ep_stride]
    std::vector<float>   link_iaabb_cache_;     // [cap × liaabb_stride]
    std::vector<uint8_t> link_iaabb_dirty_;     // 0/1，懒生成

    // ── Occupation ───────────────────────────────────────────────
    std::vector<int>    forest_id_;             // -1 = free
    std::vector<int>    subtree_occ_;           // 子树中被占用叶数
    std::vector<double> subtree_free_volume_;

    // ── Config ───────────────────────────────────────────────────
    sbf::core::Robot robot_;
    sbf::envelope::EnvelopeTypeConfig env_config_;
    std::vector<sbf::core::Interval> root_intervals_;
    sbf::core::FKState root_fk_;
    LECTConfig cfg_;

    int ep_stride_      = 0;      // n_active * 2 * 6
    int liaabb_stride_  = 0;      // n_active * 6
    int n_dims_         = 0;
    int n_active_links_ = 0;
    int n_nodes_        = 0;
    int capacity_       = 0;
    int snapshot_base_  = 0;

    std::vector<float> radii_per_slot_;         // [n_active_links_]，D1 膨胀用

    // Helpers ...
    void ensure_capacity(int min_cap);
    int  alloc_node();
    void materialise_link_iaabb(int i) const;   // mutable cache write
    int  pick_split_dim(int depth, const std::vector<sbf::core::Interval>& iv) const;
    void split_leaf_impl(int node_idx,
                         const sbf::core::FKState& parent_fk,
                         const std::vector<sbf::core::Interval>& parent_iv);
    double node_box_volume(int i) const;
```

---

## 5. Smoke Tests — `smoke_p3.cpp`

| TEST | 验证 |
|------|------|
| `P3.RootEnvelopeIsZeroRadius` | 构造后 root 的 link_iaabbs 与 P1+P2 直接管线（IFK→derive_paired_zero）byte-equal |
| `P3.ExpandSplitsLeafIntoTwo` | `expand_leaf(0)` 后 `is_leaf(0)==false`、`is_leaf(1)==true`、`is_leaf(2)==true` |
| `P3.ChildEnvelopesTighter` | 父盒 link AABB volume ≥ 子盒之和（保守性） |
| `P3.CollidesSceneInflatesByRadii` | IIWA14 root，远处障碍 false；近 ee 障碍设 link_radii 时碰，0 时不碰 |
| `P3.MarkOccupiedPropagatesUp` | 标记叶为 occupied，根 `subtree_occ() == 1`、`subtree_free_volume` 减少 |
| `P3.IsPointOccupiedFastPath` | 标记叶后，点查询返回 true |
| `P3.SnapshotIsDeepCopy` | snapshot 上 expand_leaf 不影响原 LECT |

目标 < 500 ms。

---

## 6. CMake 集成

```cmake
# src/lect/CMakeLists.txt
add_library(sbf_lect STATIC lect.cpp)
target_include_directories(sbf_lect PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(sbf_lect PUBLIC sbf_envelope sbf_scene sbf_core sbf_util)
```

顶层 `CMakeLists.txt` 取消注释 `add_subdirectory(src/lect)`。

---

## 7. 已知陷阱

1. **link_iaabb_cache 懒生成**：`get_link_iaabbs` 在 `link_iaabb_dirty_[i]==1` 时
   触发 `materialise_link_iaabb`（const_cast 写入 mutable 缓存）。线程安全约束：
   每个 LECT 实例只能在单线程内被读写（P4 并行通过 snapshot 实现）。
2. **subtree_free_volume 一致性**：`mark_occupied`/`unmark_occupied` 必须沿 parent
   链向上传播差量；`expand_leaf` 时按子盒体积重新分配父值。出错时回退到
   `refresh_free_volumes()` O(n) 重算。
3. **node_intervals 重算**：v6 `node_intervals(i)` 从根沿 split_dim/split_val 反推。
   v7 同样做：O(depth)，不存中间结果。
4. **transplant_subtree 的 ID 重映射**：worker 中的 left_/right_/parent_ 数值是
   worker 本地索引；transplant 时按 `worker_idx → master_idx` 表整体改写。
5. **WIDEST_FIRST 与 IIWA**：在带 link_radii=0 的 zero-cache 下，WIDEST_FIRST 只看
   关节区间宽度，不看 envelope 体积——可能比 BEST_TIGHTEN 慢若干百分点，但
   smoke 测试不敏感。BEST_TIGHTEN 留 P3.5 任务。

---

## 8. Definition of Done

- [ ] `sbf_lect` 编译成功，无 `-Wall -Wextra` 新增警告
- [ ] `smoke_P3` 7 个 TEST 全部通过（< 500 ms）
- [ ] `collides_scene` 内部唯一调用方是 `aabbs_collide_obs_inflated`（grep 验证）
- [ ] LECT 构造、expand、collide 全部不接受 `link_radii` 参数（仅 ctor 时从 robot 取一次）
- [ ] 顶层 `CMakeLists.txt` 中 `src/lect` 取消注释
- [ ] `/memories/session/plan.md` P3 打勾

---

*Phase: P3 | 依赖: P1+P2 | 解锁: P4*
