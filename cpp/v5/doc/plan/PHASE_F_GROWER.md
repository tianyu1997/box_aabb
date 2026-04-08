# Phase F: Forest Grower (Module 5)

> 依赖: Phase E (FFB)
> 状态: ✅ 已完成
> 产出: RRT 生长 + Wavefront 扩展 + Root 选择 + Promotion + 邻接图 + 连通性

---

## 目标

通过反复调用 FFB 在 C-space 中种植 collision-free box，构建覆盖足够大区域的 box forest。
提供两种生长策略（RRT / Wavefront）和后处理（Promotion + 邻接 + 连通性）。

---

## 架构概览

```
ForestGrower
  ├── Root 选择 (start/goal 优先 + diversity sampling)
  ├── 生长策略
  │   ├── grow_rrt()      — 随机采样 + nearest box + snap to face
  │   └── grow_wavefront() — volume priority queue + boundary expansion
  ├── try_create_box()     — 调用 FFB, 创建 BoxNode
  ├── Promotion            — 子节点合并到父节点
  ├── 邻接图               — 计算 box 间邻接关系
  └── 连通性               — UnionFind + bridge
```

---

## Step F1: Grower 主框架

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/forest/grower.h` | `src/forest/grower.cpp` |

### 数据结构
```cpp
struct GrowerConfig {
    // 生长模式
    enum class Mode { RRT, WAVEFRONT };
    Mode mode = Mode::WAVEFRONT;

    // FFB 参数
    FFBConfig ffb_config;

    // 通用参数
    int max_boxes = 500;             // 最大 box 数
    double timeout_ms = 30000.0;     // 总超时 (ms)
    int max_consecutive_miss = 50;   // 连续失败上限

    // RRT 参数
    double rrt_goal_bias = 0.3;     // goal 偏向采样概率
    double rrt_step_ratio = 0.1;    // 步长 = ratio × max_range_width

    // Wavefront 参数
    struct WavefrontStage {
        double min_edge;
        int box_limit;
    };
    std::vector<WavefrontStage> wavefront_stages = {
        {0.2, 50}, {0.1, 150}, {0.05, 300}, {0.02, 500}
    };

    // Promotion
    bool enable_promotion = true;
};

struct GrowerResult {
    std::vector<BoxNode> boxes;
    int n_roots = 0;
    int n_ffb_success = 0;
    int n_ffb_fail = 0;
    int n_promotions = 0;
    bool start_goal_connected = false;
    double total_volume = 0.0;
    double build_time_ms = 0.0;
};
```

### 接口
```cpp
class ForestGrower {
public:
    ForestGrower(const Robot& robot, LECT& lect, const GrowerConfig& config);

    void set_endpoints(const Eigen::VectorXd& start, const Eigen::VectorXd& goal);

    GrowerResult grow(const Obstacle* obs, int n_obs);

    // 访问
    const std::vector<BoxNode>& boxes() const;
    const LECT& lect() const;

private:
    int try_create_box(const Eigen::VectorXd& seed, int parent_box_id,
                       int face_dim, int face_side, int root_id);
    void grow_rrt(const Obstacle* obs, int n_obs);
    void grow_wavefront(const Obstacle* obs, int n_obs);
    void select_roots(const Obstacle* obs, int n_obs);
    void promote_all();

    const Robot& robot_;
    LECT& lect_;
    GrowerConfig config_;
    std::vector<BoxNode> boxes_;
    Eigen::VectorXd start_, goal_;
    bool has_endpoints_ = false;
    int next_box_id_ = 0;
};
```

### try_create_box()
```cpp
int try_create_box(seed, parent_box_id, face_dim, face_side, root_id) {
    FFBResult ffb = find_free_box(lect_, seed, obs_, n_obs_, config_.ffb_config);

    if (!ffb.success() || lect_.is_occupied(ffb.node_idx))
        return -1;

    BoxNode box;
    box.id = next_box_id_++;
    box.joint_intervals = lect_.node_intervals(ffb.node_idx);
    box.seed_config = seed;
    box.tree_id = ffb.node_idx;
    box.parent_box_id = parent_box_id;
    box.root_id = root_id;
    box.compute_volume();

    lect_.mark_occupied(ffb.node_idx, box.id);
    boxes_.push_back(std::move(box));
    return box.id;
}
```

### 迁移来源
| 源版本 | 源文件 | 取什么 |
|--------|--------|--------|
| v4 | `safeboxforest/v4/include/sbf/forest/forest_grower.h` | 类结构 |
| v4 | `safeboxforest/v4/src/forest/forest_grower.cpp` | 生长算法 |
| v1 | `v1/include/sbf/forest/forest_grower.h` | C++ 基线 |

---

## Step F2: RRT 生长

### 算法
```
grow_rrt():
  while boxes < max_boxes && time < timeout && miss < max_miss:
    // 1. 采样
    if random() < rrt_goal_bias && has_endpoints_:
      q_rand = goal_  (或 start_, 交替)
    else:
      q_rand = uniform_random_in(joint_limits)

    // 2. 找最近 box
    nearest_box = argmin_{b in boxes} ||b.center() - q_rand||

    // 3. 方向 + snap to face
    direction = (q_rand - nearest_box.center()).normalized()
    (face_dim, face_side, face_seed) = snap_to_face(nearest_box, direction)

    // 4. 创建 box
    box_id = try_create_box(face_seed, nearest_box.id, face_dim, face_side, ...)
    if box_id >= 0:
      miss_count = 0
    else:
      miss_count++
```

### snap_to_face()
```
for each face (dim, side) of nearest_box:
    normal_sign = (side == HI) ? +1 : -1
    score = normal_sign * direction[dim]

选 score 最大的 face
在 face 上采样: 70% target + 30% random within face bounds
返回 seed 在 face 外侧 epsilon 处
```

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v4 | `safeboxforest/v4/src/forest/forest_grower.cpp` → `grow_rrt()` |

---

## Step F3: Wavefront 扩展

### 算法
```
grow_wavefront():
  priority_queue<(volume, box_id)> pq  // 最大优先
  current_stage = 0

  // 初始化: root boxes 入队
  for root in roots: pq.push(root.volume, root.id)

  while boxes < max_boxes && time < timeout:
    // Adaptive staging
    if boxes >= stages[current_stage].box_limit && current_stage + 1 < stages.size():
      current_stage++
      // 重置 queue: 所有当前 box 重新入队
      pq.clear()
      for b in boxes: pq.push(b.volume, b.id)

    min_edge = stages[current_stage].min_edge

    if pq.empty():
      // fallback: random boundary sampling
      random_box = random_choice(boxes)
      (dim, side, seed) = random_face_sample(random_box)
      try_create_box(seed, ...)
      continue

    (_, box_id) = pq.pop()
    box = boxes[box_id]

    // 对 box 的每个面采样
    for (dim, side) in faces(box):
      seed = sample_on_face(box, dim, side, goal_bias)
      new_id = try_create_box(seed, box_id, dim, side, box.root_id)
      if new_id >= 0:
        pq.push(boxes[new_id].volume, new_id)
```

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v4 | `safeboxforest/v4/src/forest/forest_grower.cpp` → `grow_wavefront()` |

---

## Step F4: Root 选择

### 文件
逻辑在 `grower.cpp` 的 `select_roots()` 中。

### 策略
```
if has_endpoints_:
    root[0] = try_create_box(start_, ..., root_id=0)
    root[1] = try_create_box(goal_,  ..., root_id=1)
    // 剩余: diversity sampling (均匀网格 / 随机)
else:
    // 全部 random FFB
    for i in range(n_initial_roots):
        seed = uniform_random_in(joint_limits)
        try_create_box(seed, ..., root_id=i)
```

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v4 | `safeboxforest/v4/src/forest/grower_roots.cpp` |

---

## Step F5: Promotion

### 算法
```
promote_all():
  changed = true
  while changed:
    changed = false
    for each non-leaf node n in lect:
      left = lect.left(n), right = lect.right(n)
      if !lect.is_leaf(left) || !lect.is_leaf(right): continue
      if !lect.is_occupied(left) || !lect.is_occupied(right): continue
      if lect.is_occupied(n): continue

      // 验证父节点 envelope 无碰撞
      parent_ivs = lect.node_intervals(n)
      if intervals_collide_scene(parent_ivs, obs, n_obs): continue

      // 合并
      left_box = find_box(lect.forest_id(left))
      right_box = find_box(lect.forest_id(right))

      lect.unmark_occupied(left)
      lect.unmark_occupied(right)
      remove_box(left_box)
      remove_box(right_box)

      new_box = create_box(parent_intervals=parent_ivs, tree_id=n)
      lect.mark_occupied(n, new_box.id)
      boxes_.push_back(new_box)

      changed = true
```

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v4 | `safeboxforest/v4/src/forest/forest_grower.cpp` → `promote_all()` |

---

## Step F6: 邻接图

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/forest/adjacency.h` | `src/forest/adjacency.cpp` |

### 接口
```cpp
using AdjacencyGraph = std::unordered_map<int, std::vector<int>>;  // box_id → neighbors

AdjacencyGraph compute_adjacency(
    const std::vector<BoxNode>& boxes,
    double tol = 1e-10
);

// 共享面信息
struct SharedFace {
    int dim;                            // touching 维度
    double value;                       // 共享面的坐标值
    std::vector<Interval> face_ivs;     // 共享面的其余维度区间
};

std::optional<SharedFace> shared_face(const BoxNode& a, const BoxNode& b, double tol = 1e-10);
```

### 邻接判定
```
两个 box A, B 邻接 iff:
  1. 没有任何维度分离 (∀d: A.hi[d] + tol ≥ B.lo[d] && B.hi[d] + tol ≥ A.lo[d])
  2. 至少 1 个维度 touching (∃d: |A.hi[d] - B.lo[d]| ≤ tol 或 |B.hi[d] - A.lo[d]| ≤ tol)
  3. 至少 D-1 个维度 overlapping (overlap_width > tol)
```

### 复杂度
- O(N² × D)，N = box 数，D = 维度
- 可选优化: 对 N ≤ 300 用全矩阵, N > 300 分 chunk

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v1 | `v1/include/sbf/forest/adjacency.h` + `v1/src/forest/deoverlap.cpp` |

---

## Step F7: 连通性

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/forest/connectivity.h` | `src/forest/connectivity.cpp` |

### 接口
```cpp
class UnionFind {
public:
    explicit UnionFind(int n);
    int find(int x);
    void unite(int x, int y);
    bool connected(int x, int y);
};

// 从邻接图检测连通分量
std::vector<std::vector<int>> find_islands(const AdjacencyGraph& adj);

// 桥接: 找跨 island 最近 box pair → 尝试在间隙中插入 bridge box
int bridge_islands(
    std::vector<BoxNode>& boxes,
    LECT& lect,
    const Obstacle* obs, int n_obs,
    const AdjacencyGraph& adj,
    const FFBConfig& ffb_config
);
```

### bridge 算法
```
islands = find_islands(adj)
if islands.size() <= 1: return 0

for each pair (island_a, island_b):
    // 找最近 box 对
    (box_a, box_b) = closest_pair(island_a, island_b)
    midpoint = (box_a.center() + box_b.center()) / 2
    // 尝试在 midpoint 种 box
    try_create_box(midpoint, ...)
```

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v1 | `v1/include/sbf/forest/connectivity.h` + `v1/src/forest/connectivity.cpp` |

---

## 测试: test_grower.cpp

```
TEST_SUITE("Grower_RRT") {
    - 2DOF 无障碍: boxes > 10, 覆盖率 > 80%
    - 2DOF 有障碍: start-goal 连通
}

TEST_SUITE("Grower_Wavefront") {
    - 2DOF 无障碍: boxes > 10, 覆盖率 > 90%
    - 2DOF 有障碍: 多 stage 递进 (box 数递增)
}

TEST_SUITE("Promotion") {
    - 促使 promotion: 两个相邻子 box → 合并为父 box
    - promotion 后 box 数减少
}

TEST_SUITE("Adjacency") {
    - 2 touching boxes → 邻接
    - 2 separated boxes → 不邻接
    - 2 overlapping boxes → 邻接
    - shared_face 正确返回 touching 维度
}

TEST_SUITE("Connectivity") {
    - 连通图 → 1 island
    - 2 isolated groups → 2 islands
    - bridge 后 → 1 island (如果 bridge 成功)
}
```
