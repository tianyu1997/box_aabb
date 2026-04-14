# Phase L: Drake GCS 完整实现

> 依赖: Phase H (Planner, 已有 fallback) + Drake C++ 安装
> 状态: 🔲 未开始
> 产出: 填充 `gcs_planner.cpp` 中 `#ifdef SBF_HAS_DRAKE` 的 TODO → 真正的 GCS 求解
> 预计: ~250 LOC (C++)

---

## 目标

将 Phase H 中的 `gcs_plan()` 从 TODO stub 升级为完整的 Drake GraphOfConvexSets 实现。
当前 `gcs_plan()` 只是调用 `gcs_plan_fallback()` (Dijkstra)。

### 算法概述

```
BoxForest Adjacency Graph
    ↓
1. Dijkstra 获取初始路径 (baseline)
    ↓
2. 扩展走廊: 路径上的 box ± corridor_hops 层邻居
    ↓
3. 构建 Drake GraphOfConvexSets:
   - 每个 box → HPolyhedron (lb ≤ x ≤ ub)
   - Start/Goal → Point vertex
   - 邻接 box → Edge (with continuity constraint)
    ↓
4. Solve GCS shortest path (convex relaxation)
    ↓
5. Round → Extract waypoints
    ↓
6. Fallback to Dijkstra if solve fails
```

---

## Step L1: GCSConfig 扩展

### 文件
| 头文件 |
|--------|
| `include/sbf/planner/gcs_planner.h` |

### 现有接口 (Phase H)
```cpp
struct GCSConfig {
    int bezier_degree = 3;
    double time_limit_sec = 30.0;
};
```

### 新增字段
```cpp
struct GCSConfig {
    int bezier_degree = 3;
    double time_limit_sec = 30.0;
    int corridor_hops = 2;           // 走廊扩展层数
    bool convex_relaxation = true;   // 是否使用凸松弛
    double cost_weight_length = 1.0; // 路径长度权重
};
```

---

## Step L2: 走廊扩展

### 文件
`src/planner/gcs_planner.cpp`

### 算法
```cpp
/// 从 Dijkstra 路径扩展 ±hops 层邻居, 构建小规模子图
std::unordered_set<int> expand_corridor(
    const AdjacencyGraph& adj,
    const std::vector<int>& path_boxes,
    int hops
) {
    std::unordered_set<int> corridor(path_boxes.begin(), path_boxes.end());
    std::unordered_set<int> frontier = corridor;

    for (int h = 0; h < hops; ++h) {
        std::unordered_set<int> next_frontier;
        for (int box_id : frontier) {
            for (int nbr : adj.at(box_id)) {
                if (corridor.insert(nbr).second) {
                    next_frontier.insert(nbr);
                }
            }
        }
        frontier = std::move(next_frontier);
    }
    return corridor;
}
```

### 设计意图
- 全局 GCS 在 1000+ box 上太慢 → 只在走廊子图 (~50-200 box) 上求解
- `corridor_hops = 2` → 路径好的情况下已足够; 可增大以探索更多空间

---

## Step L3: Drake GCS 构建与求解

### 依赖
```cpp
#ifdef SBF_HAS_DRAKE
#include <drake/geometry/optimization/graph_of_convex_sets.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/point.h>
#include <drake/solvers/solve.h>

using drake::geometry::optimization::GraphOfConvexSets;
using drake::geometry::optimization::HPolyhedron;
using drake::geometry::optimization::Point;
#endif
```

### 核心实现
```cpp
#ifdef SBF_HAS_DRAKE
GCSResult gcs_plan(
    const AdjacencyGraph& adj,
    const std::vector<BoxNode>& boxes,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    const GCSConfig& config)
{
    const int n = start.size();

    // 1. Dijkstra baseline
    int start_box = find_containing_box(boxes, start);
    int goal_box  = find_containing_box(boxes, goal);
    if (start_box < 0 || goal_box < 0)
        return gcs_plan_fallback(adj, boxes, start, goal);

    auto dij = dijkstra_search(adj, boxes, start_box, goal_box);
    if (!dij.found)
        return {false, {}, 0.0};

    // 2. 扩展走廊
    auto corridor = expand_corridor(adj, dij.box_sequence, config.corridor_hops);

    // 3. 构建 GCS
    GraphOfConvexSets gcs;

    // 3a. Box → HPolyhedron vertices
    std::unordered_map<int, GraphOfConvexSets::Vertex*> verts;
    for (int box_id : corridor) {
        const auto& box = boxes[box_id];
        Eigen::VectorXd lb(n), ub(n);
        for (int d = 0; d < n; ++d) {
            lb[d] = box.intervals[d].lo;
            ub[d] = box.intervals[d].hi;
        }
        auto hpoly = HPolyhedron::MakeBox(lb, ub);
        verts[box_id] = gcs.AddVertex(hpoly, "box_" + std::to_string(box_id));
    }

    // 3b. Start/Goal point vertices
    auto* v_start = gcs.AddVertex(Point(start), "start");
    auto* v_goal  = gcs.AddVertex(Point(goal),  "goal");

    // 3c. Start/Goal → containing box edges
    gcs.AddEdge(v_start, verts[start_box]);
    gcs.AddEdge(verts[goal_box], v_goal);

    // 3d. Adjacency edges (bidirectional)
    for (int u : corridor) {
        for (int v : adj.at(u)) {
            if (corridor.count(v) && u < v) {
                gcs.AddEdge(verts[u], verts[v]);
                gcs.AddEdge(verts[v], verts[u]);
            }
        }
    }

    // 3e. Edge cost: ||x_u - x_v||₂²
    for (auto* edge : gcs.Edges()) {
        auto xu = edge->xu();
        auto xv = edge->xv();
        edge->AddCost((xu - xv).squaredNorm() * config.cost_weight_length);
    }

    // 4. Solve
    auto result = gcs.SolveShortestPath(
        *v_start, *v_goal,
        config.convex_relaxation
    );

    if (!result.is_success()) {
        // GCS 失败 → fallback Dijkstra
        return gcs_plan_fallback(adj, boxes, start, goal);
    }

    // 5. 提取路径 (activated edges → waypoints)
    std::vector<Eigen::VectorXd> path;
    path.push_back(start);

    // 遍历 activated edges, 提取顶点处的最优解
    for (auto* edge : gcs.Edges()) {
        double flow = result.GetSolution(edge->phi());
        if (flow > 0.5) {
            Eigen::VectorXd wp = result.GetSolution(edge->xv());
            path.push_back(wp);
        }
    }
    // 排序/去重 (flow rounding 可能产生乱序)
    // ... (具体实现取决于 Drake API 版本)

    path.push_back(goal);

    double cost = result.get_optimal_cost();
    return {true, path, cost};
}
#endif  // SBF_HAS_DRAKE
```

---

## Step L4: 更新 CMake Drake 集成

### 当前 CMakeLists.txt (已有)
```cmake
find_package(drake QUIET)
if(drake_FOUND)
    target_compile_definitions(sbf5 PUBLIC SBF_HAS_DRAKE)
    target_link_libraries(sbf5 PUBLIC drake::drake)
endif()
```

### 额外: Drake GCS 测试 (仅 Drake 环境可用)
```cmake
if(drake_FOUND)
    add_executable(test_gcs_drake test/test_gcs_drake.cpp)
    target_link_libraries(test_gcs_drake PRIVATE sbf5 doctest::doctest)
    add_test(NAME test_gcs_drake COMMAND test_gcs_drake)
    set_tests_properties(test_gcs_drake PROPERTIES
        WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
        LABELS "drake")
endif()
```

---

## Step L5: 测试

### 无 Drake 测试 (已有)
`test/test_planner.cpp` 中的 GCS fallback 测试已通过。

### Drake 环境测试 (新增)
`test/test_gcs_drake.cpp`

| 用例 | 描述 |
|------|------|
| `expand_corridor_basic` | 3-box 链 → corridor_hops=1 → 扩展到全部 |
| `gcs_plan_2dof_simple` | 2DOF + 单障碍 → GCS 返回 success, path 连续 |
| `gcs_plan_cost_vs_dijkstra` | GCS path_length ≤ Dijkstra path_length |
| `gcs_plan_fallback_on_failure` | 不可达目标 → 返回 fallback result |

---

## 迁移来源

| 源版本 | 源文件 | 取什么 |
|--------|--------|--------|
| v1 | `v1/src/adapters/drake_gcs.cpp` | 走廊扩展 + GCS 构建模式 |
| v2 | `v2/src/planner/gcs_optimizer.py` | HPolyhedron 构建 + edge cost |
| v3 | `v3/src/planner/gcs_optimizer.py` | SolveShortestPath 调用方式 |

---

## 验收标准

- [ ] `gcs_plan()` 在 Drake 环境下编译通过
- [ ] 走廊扩展正确 (corridor_hops=0 → 仅路径 box)
- [ ] 2DOF 简单场景 GCS 求解成功
- [ ] GCS 路径质量 ≥ Dijkstra (cost 更低或相等)
- [ ] 无 Drake 时 `gcs_plan_fallback()` 不受影响
- [ ] `SBF_HAS_DRAKE` 条件编译无泄漏 (不 break 无 Drake 构建)
