# Phase E: FFB — Find Free Box (Module 4)

> 依赖: Phase D (LECT)
> 状态: **全部完成** (E1/E2: 2026-04-04, E3: 2026-04-04)
> 产出: FFB 搜索算法 + 占用标记 + LECT 二进制缓存持久化

---

## 目标

在 LECT 上执行 tree descent，从给定 seed 找到包含该 seed 的最大无碰撞 box。
这是 Grower 的核心原语——每次 "种一个 box" 都调用 FFB。

---

## 算法概览

```
FFB(seed, obstacles) → FFBResult

1. 从 LECT root 开始
2. 循环:
   a. 若当前节点已有 cached envelope → 使用缓存
   b. 否则 compute_envelope()
   c. 碰撞检测 collides_scene()
   d. 若无碰撞 + 未占用 → SUCCESS，返回当前节点
   e. 若碰撞:
      - 检查 depth / min_edge 停止条件
      - expand_leaf() 分裂
      - 选包含 seed 的子节点，继续
   f. 若已占用 → FAIL(occupied)
3. 达到 max_depth 或 min_edge → FAIL
```

---

## Step E1: FFB 核心算法 ✅ 已完成

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/ffb/ffb.h` | `src/ffb/ffb.cpp` |

### 数据结构
```cpp
struct FFBResult {
    int node_idx = -1;          // 目标 LECT 叶节点 (-1 on failure)
    std::vector<int> path;      // root → leaf 的节点路径
    int fail_code = 0;          // 0=success, 1=occupied, 2=max_depth, 3=min_edge, 4=deadline
    int n_new_nodes = 0;        // 本次搜索新扩展的节点数
    int n_fk_calls = 0;         // FK 计算次数

    bool success() const { return fail_code == 0 && node_idx >= 0; }
};

struct FFBConfig {
    double min_edge = 1e-4;     // 最小区间宽度
    int max_depth = 30;         // 最大树深度
    double deadline_ms = 0.0;   // 绝对超时 (0=无限制), v4.3 特性
};
```

### 接口
```cpp
FFBResult find_free_box(
    LECT& lect,
    const Eigen::VectorXd& seed,
    const Obstacle* obs,
    int n_obs,
    const FFBConfig& config = {}
);
```

### 详细算法
```cpp
FFBResult find_free_box(LECT& lect, seed, obs, n_obs, config) {
    FFBResult result;
    FKState fk;
    int current = 0;  // root
    result.path.push_back(current);

    while (true) {
        // 1. 检查 deadline
        if (config.deadline_ms > 0 && elapsed_ms() > config.deadline_ms) {
            result.fail_code = 4;
            return result;
        }

        // 2. 检查占用
        if (lect.is_occupied(current)) {
            result.fail_code = 1;
            return result;
        }

        // 3. 计算/复用 envelope
        auto intervals = lect.node_intervals(current);
        int parent_idx = result.path.size() >= 2 ? result.path[result.path.size()-2] : -1;
        int changed_dim = (parent_idx >= 0) ? lect.get_split_dim(parent_idx) : -1;

        if (!lect.has_data(current)) {
            lect.compute_envelope(current, fk, intervals, changed_dim, parent_idx);
            result.n_fk_calls++;
        }

        // 4. 碰撞检测
        bool collides = lect.collides_scene(current, obs, n_obs);

        if (!collides) {
            // SUCCESS: 无碰撞 + 未占用
            result.node_idx = current;
            result.fail_code = 0;
            return result;
        }

        // 5. 碰撞 → 需要分裂
        int d = lect.depth(current);
        if (d >= config.max_depth) {
            result.fail_code = 2;
            return result;
        }

        // 检查 min_edge: 分裂后子区间宽度
        int split_dim = lect.get_split_dim(current);
        double child_width = intervals[split_dim].width() / 2.0;
        if (child_width < config.min_edge) {
            result.fail_code = 3;
            return result;
        }

        // 6. 分裂
        if (lect.is_leaf(current)) {
            lect.expand_leaf(current);
            result.n_new_nodes += 2;
        }

        // 7. 选子节点 (包含 seed 的一侧)
        double sv = lect.split_val(current);
        int sd = lect.get_split_dim(current);
        if (seed[sd] <= sv) {
            current = lect.left(current);
        } else {
            current = lect.right(current);
        }
        result.path.push_back(current);
    }
}
```

### 缓存优先策略
- 若 `lect.has_data(current) == true`，直接使用已缓存的 link iAABBs 做碰撞检测
- 跳过 `compute_envelope()` → 节省 FK 计算
- 只有首次访问未缓存节点时才计算

### 迁移来源
| 源版本 | 源文件 | 取什么 |
|--------|--------|--------|
| v4 | `safeboxforest/v4/src/forest/lect.cpp` | `find_free_box()` 方法 |
| v1 | `v1/include/sbf/forest/hier_aabb_tree.h` | `find_free_box()` 基础版 |

---

## Step E2: 占用管理 ✅ 已完成（Phase D 提供）

已在 LECT 中实现（Phase D），此处仅说明 FFB 的使用方式：

```
// Grower 调用 FFB 后:
FFBResult ffb = find_free_box(lect, seed, obs, n_obs, config);
if (ffb.success()) {
    lect.mark_occupied(ffb.node_idx, new_box_id);
    // ... 创建 BoxNode
}
```

---

## Step E3: 缓存持久化 ✅ 已完成 (2026-04-04)

### 文件
| 头文件 | 实现 | 测试 |
|--------|------|------|
| `include/sbf/lect/lect_io.h` | `src/lect/lect_io.cpp` | `test/test_lect_io.cpp` |

### 接口
```cpp
bool lect_save_binary(const LECT& lect, const std::string& path);
bool lect_load_binary(LECT& lect, const Robot& robot, const std::string& path);
```

### 格式
- **Header** (48 bytes): magic `SBF5LECT` (8B) + version=1 (uint32) + n_nodes, n_dims, n_active_links, ep_stride, liaabb_stride (各 uint32) + split_order, ep_source, env_type (各 uint8) + reserved (5B) + capacity (uint32)
- **Root intervals**: n_dims × 2 doubles
- **Tree arrays**: left_, right_, parent_, depth_, split_dim_ (int32); split_val_ (double); forest_id_ (int32); has_data_, source_quality_ (uint8)
- **Envelope data**: ep_data_, link_iaabb_cache_ (float arrays)
- Robot 不序列化，load 时由调用方提供；Z4 cache / radii / root_fk 从 robot 重建

### 测试: 12/12 通过 (88 assertions)
- roundtrip root-only / expanded tree
- envelope data / endpoint iAABBs / occupation state / root intervals / split_order 全部保留
- bad magic / nonexistent file / invalid path → 返回 false
- loaded LECT 可继续 collides_scene / expand_leaf

---

## Fail Code 语义

| Code | 含义 | 后续处理 |
|------|------|---------|
| 0 | **成功** | 创建 BoxNode, 标记占用 |
| 1 | 被已有 box 占用 | 换 seed 重试 |
| 2 | 达到 max_depth | 提高 max_depth 或放弃 |
| 3 | 区间宽度 ≤ min_edge | 该区域太小，放弃 |
| 4 | 超时 (deadline) | 终止当前批次 |

---

## 测试: test_ffb.cpp ✅ 12/12 通过 (26 assertions)

```
TEST_SUITE("FFB_Basic") {                                          ✅
    - 2DOF 无障碍: seed 在区间中心 → success, node 为 root (无需分裂)
    - 2DOF 有障碍: seed 远离障碍 → success, depth > 0
    - 2DOF 有障碍: seed 在障碍内 → fail_code = 3 (min_edge)
}

TEST_SUITE("FFB_FailCodes") {                                      ✅
    - occupied: 先 mark_occupied root → fail_code = 1
    - max_depth: config.max_depth = 2, 障碍场景 → fail_code = 2
    - min_edge: config.min_edge = 很大值 → fail_code = 3
    - deadline: config.deadline_ms = 0.001 (极短) → fail_code = 4
}

TEST_SUITE("FFB_Cache") {                                          ✅
    - 同一 seed 两次 FFB: 第二次 n_fk_calls == 0 (缓存命中)
    - 不同 seed 同一子树: 共享祖先节点缓存
}

TEST_SUITE("FFB_Path") {                                           ✅
    - path 从 0 (root) 开始
    - path 末尾 == node_idx (success 时)
    - path 长度 ≤ max_depth + 1
}
```

---

## 实施备注 (2026-04-04)

### 与计划的差异
1. **min_edge 检查**: 计划中使用 `lect.get_split_dim(current)` 获取分裂维度后检查该维度宽度。
   实际实现中，leaf 节点的 `split_dim` 在 `expand_leaf` 之前为 -1（未确定），
   因此改用所有维度最小宽度的保守估计: `min(intervals[d].width()) / 2 < min_edge`。
2. **deadline 计时**: 使用 `std::chrono::steady_clock`，在循环入口检查。
3. **FKState 传递**: FFB 中创建的默认 `FKState` (`valid=false`) 不会触发 IFK 快速路径，
   `compute_envelope` 内部会 fallback 到 `compute_endpoint_iaabb` 全量计算。
   但首次访问后节点有缓存，不影响正确性。
