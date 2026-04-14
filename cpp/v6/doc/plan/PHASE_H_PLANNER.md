# Phase H: Planner (Module 7)

> 依赖: Phase F (Grower) + Phase G (Coarsening)
> 状态: ✅ 已完成
> 产出: Dijkstra + GCS Planner (可选) + Path Smoother + SBF 顶层 + IPlanner 接口

---

## 目标

在 box forest 的邻接图上进行路径搜索，提取 C-space waypoints，并进行路径平滑。
提供 Dijkstra (默认) 和 GCS (可选 Drake 依赖) 两种搜索方式。
定义 `IPlanner` 接口为后续扩展预留。

---

## 管线概览

```
BoxForest (from Grower + Coarsening)
    ↓
AdjacencyGraph
    ↓
Search: Dijkstra / GCS
    ↓
Box sequence [box_0, box_1, ..., box_n]
    ↓
Waypoint extraction (shared-face centers)
    ↓
[q_start, wp_1, wp_2, ..., q_goal]
    ↓
Path Smoothing (shortcut + moving average)
    ↓
Final path
```

---

## Step H1: Dijkstra 搜索

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/planner/dijkstra.h` | `src/planner/dijkstra.cpp` |

### 接口
```cpp
struct DijkstraResult {
    bool found = false;
    std::vector<int> box_sequence;    // [box_id_0, ..., box_id_n]
    double total_cost = 0.0;
};

DijkstraResult dijkstra_search(
    const AdjacencyGraph& adj,
    const std::vector<BoxNode>& boxes,
    int start_box_id,
    int goal_box_id
);
```

### 算法
```cpp
DijkstraResult dijkstra_search(adj, boxes, start_id, goal_id) {
    // 标准堆 Dijkstra
    // cost(a, b) = ||center(a) - center(b)||₂
    // priority_queue<(cost, box_id)>

    unordered_map<int, double> dist;    // box_id → min distance
    unordered_map<int, int>    prev;    // box_id → predecessor
    priority_queue<pair<double,int>, ..., greater> pq;

    dist[start_id] = 0.0;
    pq.push({0.0, start_id});

    while (!pq.empty()) {
        auto [d, u] = pq.top(); pq.pop();
        if (d > dist[u]) continue;       // stale entry

        if (u == goal_id) {
            // reconstruct path
            return {true, reconstruct(prev, start_id, goal_id), d};
        }

        for (int v : adj.at(u)) {
            double edge_cost = (boxes[u].center() - boxes[v].center()).norm();
            double new_dist = d + edge_cost;
            if (new_dist < dist[v] (or v not in dist)) {
                dist[v] = new_dist;
                prev[v] = u;
                pq.push({new_dist, v});
            }
        }
    }
    return {false, {}, 0.0};
}
```

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v1 | `v1/include/sbf/planner/graph_search.h` → `dijkstra_center_distance()` |

---

## Step H2: 路径提取

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/planner/path_extract.h` | `src/planner/path_extract.cpp` |

### 接口
```cpp
std::vector<Eigen::VectorXd> extract_waypoints(
    const std::vector<int>& box_sequence,
    const std::vector<BoxNode>& boxes,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal
);
```

### 算法
```
path = [start]

for i in [0, len(box_sequence) - 1):
    box_a = boxes[box_sequence[i]]
    box_b = boxes[box_sequence[i+1]]

    face = shared_face(box_a, box_b)
    if face:
        // waypoint = face center (touching dim 固定, 其余取中点)
        wp = face_center(face)
    else:
        // fallback: 两 box center 中点
        wp = (box_a.center() + box_b.center()) / 2

    path.push_back(wp)

path.push_back(goal)
return path
```

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v1 | `v1/include/sbf/planner/graph_search.h` → `extract_waypoints()` |

---

## Step H3: GCS-Based Planner (可选 Drake 依赖)

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/planner/gcs_planner.h` | `src/planner/gcs_planner.cpp` |

### 编译条件
```cpp
// CMakeLists.txt:
find_package(drake QUIET)
if(drake_FOUND)
    target_compile_definitions(sbf5 PRIVATE SBF_HAS_DRAKE)
    target_link_libraries(sbf5 PRIVATE drake::drake)
endif()
```

### 接口
```cpp
struct GCSConfig {
    int bezier_degree = 3;           // Bezier 曲线阶数
    double time_limit_sec = 30.0;    // 求解超时
};

struct GCSResult {
    bool found = false;
    std::vector<Eigen::VectorXd> path;
    double cost = 0.0;
};

#ifdef SBF_HAS_DRAKE
GCSResult gcs_plan(
    const AdjacencyGraph& adj,
    const std::vector<BoxNode>& boxes,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    const GCSConfig& config = {}
);
#endif

// Fallback: 无 Drake 时使用 Dijkstra + waypoint 优化
GCSResult gcs_plan_fallback(
    const AdjacencyGraph& adj,
    const std::vector<BoxNode>& boxes,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal
);
```

### Drake 模式算法
```
1. 构建 GraphOfConvexSets:
   - 每个 box → HPolyhedron vertex (lb ≤ x ≤ ub)
   - start/goal → point vertices
   - 每条邻接边 → graph edge
   - Edge cost: ||x_u - x_v||² (L2)
   - Edge constraint: x_u[d] == x_v[d] (continuity)

2. Solve: gcs.SolveShortestPath(source, target, convex_relaxation=true)

3. Round: Extract activated edges → collect waypoints
```

### Fallback 模式算法
```
1. Dijkstra: 获取 box_sequence
2. extract_waypoints: shared_face center
3. (可选) L-BFGS-B 局部优化 waypoint 位置, 约束在 box 内
```

### 迁移来源
| 源版本 | 源文件 | 取什么 |
|--------|--------|--------|
| v1 | `v1/src/adapters/drake_gcs.cpp` | Drake C++ 接口 |
| v2 | `v2/src/planner/gcs_optimizer.py` | GCS 逻辑 (graph build + solve) |
| v3 | `v3/src/planner/gcs_optimizer.py` | 同上 |

---

## Step H4: Path Smoother

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/planner/path_smoother.h` | `src/planner/path_smoother.cpp` |

### 接口
```cpp
struct SmootherConfig {
    int shortcut_max_iters = 100;
    int smooth_window = 3;
    int smooth_iters = 5;
    int segment_resolution = 10;     // check_segment 分辨率
};

// 随机 shortcut: 随机选两点, 若直连无碰撞则移除中间点
std::vector<Eigen::VectorXd> shortcut(
    const std::vector<Eigen::VectorXd>& path,
    const CollisionChecker& checker,
    const SmootherConfig& config,
    uint64_t seed = 42
);

// 移动平均平滑: 窗口内取平均, 约束在 box 边界内
std::vector<Eigen::VectorXd> smooth_moving_average(
    const std::vector<Eigen::VectorXd>& path,
    const std::vector<BoxNode>& box_sequence,   // 对应的 box (用于约束)
    const SmootherConfig& config
);

// 组合: shortcut → smooth
std::vector<Eigen::VectorXd> smooth_path(
    const std::vector<Eigen::VectorXd>& path,
    const std::vector<BoxNode>& box_sequence,
    const CollisionChecker& checker,
    const SmootherConfig& config
);
```

### Shortcut 算法
```
for iter in [0, max_iters):
    i = random_int(0, path.size() - 2)
    j = random_int(i + 2, path.size())
    if !checker.check_segment(path[i], path[j], resolution):
        // 直连无碰撞 → 移除中间点
        path.erase(path.begin() + i + 1, path.begin() + j)
```

### Moving Average 算法
```
for iter in [0, smooth_iters):
    for i in [1, path.size() - 1):  // 不动 start/goal
        // 窗口平均
        sum = 0
        count = 0
        for j in [max(0, i-window), min(path.size(), i+window+1)):
            sum += path[j]
            count++
        candidate = sum / count

        // 约束在对应 box 内
        candidate = clamp_to_box(candidate, box_sequence[i])
        path[i] = candidate
```

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v1 | `v1/include/sbf/planner/path_smoother.h` |
| v2 | `v2/src/planner/path_smoother.py` |

---

## Step H5: SBF Planner 顶层

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/planner/sbf_planner.h` | `src/planner/sbf_planner.cpp` |

### 接口
```cpp
struct PlanResult {
    bool success = false;
    std::vector<Eigen::VectorXd> path;       // final smoothed path
    std::vector<int> box_sequence;             // box ids along path
    double path_length = 0.0;
    double planning_time_ms = 0.0;
    int n_boxes = 0;
    int n_coarsen_merges = 0;
};

struct SBFPlannerConfig {
    GrowerConfig grower;
    GreedyCoarsenConfig coarsen;
    SmootherConfig smoother;
    bool use_gcs = false;            // true: 优先 GCS (需 Drake), false: Dijkstra
    GCSConfig gcs;
};

class SBFPlanner {
public:
    SBFPlanner(const Robot& robot, const SBFPlannerConfig& config = {});

    // 一次性: grow + coarsen + search + smooth
    PlanResult plan(const Eigen::VectorXd& start,
                    const Eigen::VectorXd& goal,
                    const Obstacle* obs, int n_obs,
                    double timeout_ms = 30000.0);

    // 预构建: grow + coarsen (不搜索)
    void build(const Eigen::VectorXd& start,
               const Eigen::VectorXd& goal,
               const Obstacle* obs, int n_obs,
               double timeout_ms = 30000.0);

    // 查询已有 forest
    PlanResult query(const Eigen::VectorXd& start,
                     const Eigen::VectorXd& goal);

    // 清除
    void clear_forest();

    // 访问
    const std::vector<BoxNode>& boxes() const;
    int n_boxes() const;

private:
    const Robot& robot_;
    SBFPlannerConfig config_;
    std::unique_ptr<LECT> lect_;
    std::vector<BoxNode> boxes_;
    AdjacencyGraph adj_;
    bool built_ = false;
};
```

### plan() 管线
```
plan(start, goal, obs, n_obs, timeout):
  // 1. 初始化 LECT
  lect_ = make_unique<LECT>(robot_, joint_limits, ep_config, env_config)

  // 2. Grow forest
  ForestGrower grower(robot_, *lect_, config_.grower)
  grower.set_endpoints(start, goal)
  GrowerResult gr = grower.grow(obs, n_obs)
  boxes_ = gr.boxes

  // 3. Coarsen
  coarsen_forest(boxes_, checker, 10)
  coarsen_greedy(boxes_, checker, config_.coarsen, lect_.get())

  // 4. Build adjacency
  adj_ = compute_adjacency(boxes_)

  // 5. Find start/goal boxes
  start_box_id = find_containing_box(boxes_, start)
  goal_box_id  = find_containing_box(boxes_, goal)

  // 6. Search
  if config_.use_gcs:
    gcs_result = gcs_plan(adj_, boxes_, start, goal, config_.gcs)  // or fallback
    path = gcs_result.path
    box_seq = ...
  else:
    dij = dijkstra_search(adj_, boxes_, start_box_id, goal_box_id)
    path = extract_waypoints(dij.box_sequence, boxes_, start, goal)
    box_seq = dij.box_sequence

  // 7. Smooth
  path = smooth_path(path, ..., checker, config_.smoother)

  // 8. Return
  return {true, path, box_seq, path_length(path), elapsed, ...}
```

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v1 | `v1/include/sbf/planner/sbf_planner.h` + `v1/src/planner/sbf_planner.cpp` |

---

## Step H6: IPlanner 扩展接口

### 文件
| 头文件 |
|--------|
| `include/sbf/planner/i_planner.h` |

### 接口
```cpp
class IPlanner {
public:
    virtual ~IPlanner() = default;

    virtual PlanResult plan(
        const Eigen::VectorXd& start,
        const Eigen::VectorXd& goal,
        const Obstacle* obs, int n_obs,
        double timeout_ms = 30000.0
    ) = 0;
};
```

- `SBFPlanner` 继承 `IPlanner`
- 后续可实现: `PureDijkstraPlanner`, `GCSOnlyPlanner`, `HybridPlanner` 等

---

## 测试

### test_planner.cpp
```
TEST_SUITE("Dijkstra") {
    - 2DOF 3 boxes 线性: 路径 = [start_box, mid_box, goal_box]
    - 2DOF 不连通: found == false
    - cost 单调递增
}

TEST_SUITE("PathExtract") {
    - waypoints 在 shared face 上
    - path[0] == start, path.back() == goal
}

TEST_SUITE("PathSmoother") {
    - shortcut: path_length 递减
    - smooth_moving_average: path_length 递减或不变
    - 平滑后路径仍无碰撞
}

TEST_SUITE("SBFPlanner") {
    - 2DOF 无障碍: plan → success
    - 2DOF 有障碍: plan → success, 路径无碰撞
    - build → query: 两次 query 结果一致
    - clear_forest: boxes 清空
}
```

### test_full_pipeline.cpp (集成测试)
```
TEST_SUITE("FullPipeline_2DOF") {
    - 完整管线: IFK → LinkIAABB → LECT → FFB → Wavefront → Coarsen → Dijkstra → Smooth
    - 验证: 路径无碰撞, path.size() >= 2
}

TEST_SUITE("FullPipeline_7DOF") {
    - (需 Panda config)
    - GCPC → LinkIAABB → LECT → FFB → RRT → Coarsen → Dijkstra
    - 验证: 路径存在 (如果 timeout 允许)
}
```
