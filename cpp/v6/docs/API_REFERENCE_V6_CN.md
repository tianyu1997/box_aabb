# SafeBoxForest v6 — API 参考手册（中文）

> **版本**: v6（42 头文件，10 模块）
> **语言**: C++17, Eigen 3.x
> **构建**: CMake 3.16+, `-O3 -march=native`
> **命名空间**: `sbf`

---

## 架构概述

SBF v6 是面向串联机械臂的两阶段运动规划系统：

1. **离线阶段** (`build_coverage`)：构建覆盖 C-space 的认证安全盒森林
2. **在线阶段** (`query`)：通过盒森林图搜索 + 多层优化规划路径

```
                    ┌─────────────────────────────────────┐
                    │         SBFPlanner (planner/)        │
                    │  build_coverage() → query(s, g)      │
                    └────┬──────────┬──────────┬──────────┘
                         │          │          │
              ┌──────────▼──┐  ┌───▼───┐  ┌──▼──────────┐
              │ ForestGrower │  │ LECT  │  │ PathSmoother│
              │ (forest/)    │  │(lect/)│  │ (planner/)  │
              └──────┬───────┘  └───┬───┘  └─────────────┘
                     │              │
           ┌─────────▼───┐   ┌─────▼─────────┐
           │ FFB (ffb/)   │   │ Envelope       │
           │ find_free_box│   │ (envelope/)    │
           └──────────────┘   └───────────────┘
```

---

## 目录

- [core/types](#coretypes) — 基础类型
- [core/robot](#corerobot) — DH 机器人
- [core/fk_state](#corefk_state) — FK 状态
- [core/log](#corelog) — 日志宏 *(v6 新增)*
- [core/constants](#coreconstants) — 命名常量 *(v6 新增)*
- [core/interval_math](#coreinterval_math) — 区间算术
- [core/union_find](#coreunion_find) — 并查集 *(v6: 移至 core/)*
- [core/joint_symmetry](#corejoint_symmetry) — Z4 对称
- [core/ray_aabb](#coreray_aabb) — 射线相交
- [scene/collision_checker](#scenecollision_checker) — 碰撞检测
- [envelope/endpoint_source](#envelopeendpoint_source) — 端点源
- [lect/lect](#lectlect) — 包络缓存树
- [lect/lect_io](#lectlect_io) — 持久化
- [ffb/ffb](#ffbffb) — 安全盒搜索
- [forest/grower](#forestgrower) — 森林生长
- [forest/adjacency](#forestadjacency) — 邻接
- [forest/coarsen](#forestcoarsen) — 粗化
- [forest/connectivity](#forestconnectivity) — 连通性
- [forest/thread_pool](#forestthread_pool) — 线程池
- [planner/sbf_planner](#plannersbf_planner) — 顶层规划器
- [planner/dijkstra](#plannerdijkstra) — 图搜索
- [planner/path_smoother](#plannerpath_smoother) — 路径优化
- [planner/gcs_planner](#plannergcs_planner) — GCS 规划
- [voxel/](#voxel) — 稀疏体素
- [viz/](#viz) — 可视化
- [adapters/](#adapters) — OMPL 适配器
- [Python 绑定](#python-绑定-sbf6)
- [v6 与 v5 对比](#v6-与-v5-对比)

---

## core/types

> 头文件：`include/sbf/core/types.h`

### `Interval` — 闭区间 [lo, hi]

```cpp
struct Interval {
    double lo, hi;
    double width() const;
    double mid() const;
    bool contains(double v) const;
    Interval hull(const Interval& o) const;
    Interval intersect(const Interval& o) const;
    bool overlaps(const Interval& o) const;
    // 区间算术: +, -, *
};
```

### `Obstacle` — 工作空间 AABB

```cpp
struct Obstacle {
    float bounds[6]; // [lx, ly, lz, hx, hy, hz]
};
```

### `BoxNode` — C-space 安全盒

```cpp
struct BoxNode {
    int id;
    std::vector<Interval> joint_intervals;
    Eigen::VectorXd seed_config;
    double volume;
    int tree_id, parent_box_id, root_id;
    int n_dims() const;
    Eigen::VectorXd center() const;
    void compute_volume();
    bool contains(const Eigen::VectorXd& q, double tol) const;
};
```

---

## core/robot

> 头文件：`include/sbf/core/robot.h`

```cpp
class Robot {
public:
    static Robot from_json(const std::string& path);
    const std::string& name() const;
    int n_joints() const;
    int n_active_links() const;
    uint64_t fingerprint() const;  // FNV-1a 64位（标识LECT缓存）
};
```

---

## core/log

> 头文件：`include/sbf/core/log.h`（**v6 新增**）

集中日志宏，从分散源文件提取：

```cpp
#define SBF_LOG_INFO(fmt, ...)
#define SBF_LOG_WARN(fmt, ...)
#define SBF_LOG_ERR(fmt, ...)
#define SBF_LOG_DBG(fmt, ...)   // 编译时门控
```

---

## core/constants

> 头文件：`include/sbf/core/constants.h`（**v6 新增**）

命名常量替代魔术数字：

```cpp
namespace sbf::constants {
    constexpr double kDefaultMinEdge = 1e-4;
    constexpr int    kDefaultMaxDepth = 30;
    constexpr double kAdjacencyTol = 1e-10;
    // ...
}
```

---

## core/union_find

> 头文件：`include/sbf/core/union_find.h`（**v6：从 forest/ 移至 core/**）

```cpp
class UnionFind {
public:
    explicit UnionFind(int n = 0);
    int find(int x);               // 路径压缩
    void unite(int x, int y);      // 按秩合并
    bool connected(int x, int y);  // O(α(n))
    int size() const;
    void resize(int new_size);
};
```

---

## scene/collision_checker

> 头文件：`include/sbf/scene/collision_checker.h`

```cpp
class CollisionChecker {
public:
    CollisionChecker(const Robot& robot, const std::vector<Obstacle>& obstacles);
    bool check_config(const Eigen::VectorXd& q) const;
    bool check_box(const std::vector<Interval>& ivs) const;
    bool check_segment(const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                       int resolution = 10) const;
};
```

> **线程安全**：所有 const 方法，多线程共享安全。

---

## envelope/endpoint_source

> 头文件：`include/sbf/envelope/endpoint_source.h`

4 种端点源：

| 枚举值 | 质量 | 安全类 | 说明 |
|--------|------|------|------|
| `IFK` | 0 | SAFE | 区间 FK（最快，⊇ 真实可达集） |
| `CritSample` | 1 | UNSAFE | 边界 k·π/2 采样 |
| `Analytical` | 2 | SAFE | 解析梯度零点枚举 |
| `GCPC` | 3 | SAFE | 全局缓存 + 解析（推荐） |

源替代矩阵：IFK 可服务 CritSample（更保守），反之不行。

```cpp
EndpointIAABBResult compute_endpoint_iaabb(
    const Robot& robot, const std::vector<Interval>& intervals,
    const EndpointSourceConfig& config, FKState* fk = nullptr);
```

---

## lect/lect

> 头文件：`include/sbf/lect/lect.h`

惰性 KD 树 + 双通道缓存。

| 通道 | 源 | 语义 | 用途 |
|------|---|------|------|
| CH\_SAFE (0) | IFK | 保守 ⊇ 真实 | 碰撞否证 |
| CH\_UNSAFE (1) | CritSample 等 | 紧致逼近 | 覆盖估计 |

```cpp
class LECT {
public:
    LECT(const Robot& robot, const std::vector<Interval>& root_intervals,
         const EndpointSourceConfig& ep_config, const EnvelopeTypeConfig& env_config);

    int expand_leaf(int node_idx);
    void compute_envelope(int node_idx, const FKState& fk,
                          const std::vector<Interval>& intervals);
    bool collides_scene(int node_idx, const Obstacle* obs, int n_obs) const;

    void mark_occupied(int node_idx, int box_id);
    bool is_occupied(int node_idx) const;
    int subtree_occ(int node_idx) const;

    void set_split_order(SplitOrder so);  // BEST_TIGHTEN 默认
    LECT snapshot() const;                // 深拷贝
    void set_ep_config(const EndpointSourceConfig& cfg);
};
```

---

## lect/lect_io

> 头文件：`include/sbf/lect/lect_io.h`

```cpp
bool lect_save_binary(const LECT& lect, const std::string& path);
bool lect_save_incremental(const LECT& lect, const std::string& path, int old_n_nodes);
bool lect_load_binary(LECT& lect, const Robot& robot, const std::string& path);
```

---

## ffb/ffb

> 头文件：`include/sbf/ffb/ffb.h`

```cpp
FFBResult find_free_box(
    LECT& lect, const Eigen::VectorXd& seed,
    const Obstacle* obs, int n_obs, const FFBConfig& config = {});
```

| fail\_code | 含义 |
|-----------|------|
| 0 | 成功 |
| 1 | 已占用 |
| 2 | 超深度 |
| 3 | 低于最小边 |
| 4 | 超时 |

---

## forest/grower

> 头文件：`include/sbf/forest/grower.h`

```cpp
class ForestGrower {
public:
    ForestGrower(const Robot& robot, LECT& lect, const GrowerConfig& config);
    GrowerResult grow(const Obstacle* obs, int n_obs);
};
```

**`connect_mode = true`** 推荐：`rrt_goal_bias=0.8`, `rrt_step_ratio=0.05`, `max_consecutive_miss=2000`

---

## forest/adjacency

> 头文件：`include/sbf/forest/adjacency.h`

```cpp
using AdjacencyGraph = std::unordered_map<int, std::vector<int>>;
AdjacencyGraph compute_adjacency(const std::vector<BoxNode>& boxes);
std::unordered_set<int> find_articulation_points(const AdjacencyGraph& adj);
```

---

## forest/coarsen

> 头文件：`include/sbf/forest/coarsen.h`

五级流水线：维度扫描 → 松弛扫描 → 贪心邻接 → 聚类 → 重叠过滤。
桥接点（Tarjan 算法）自动保护。

---

## forest/connectivity

> 头文件：`include/sbf/forest/connectivity.h`

```cpp
std::vector<VectorXd> rrt_connect(const VectorXd& q_a, const VectorXd& q_b,
                                   const CollisionChecker& checker, const Robot& robot);
int bridge_all_islands(std::vector<BoxNode>& boxes, LECT& lect, ...);
```

---

## planner/sbf_planner

> 头文件：`include/sbf/planner/sbf_planner.h`

```cpp
class SBFPlanner : public IPlanner {
public:
    SBFPlanner(const Robot& robot, const SBFPlannerConfig& config = {});

    PlanResult plan(const VectorXd& start, const VectorXd& goal,
                    const Obstacle* obs, int n_obs, double timeout_ms = 30000.0) override;

    void build_coverage(const Obstacle* obs, int n_obs, double timeout_ms = 30000.0,
                        const std::vector<VectorXd>& seed_points = {});

    PlanResult query(const VectorXd& start, const VectorXd& goal,
                     const Obstacle* obs = nullptr, int n_obs = 0);
};
```

### `PlanResult`

| 字段 | 类型 | 说明 |
|------|------|------|
| `success` | `bool` | 找到路径 |
| `path` | `vector<VectorXd>` | 路径点 |
| `path_length` | `double` | 路径长度 (rad) |
| `planning_time_ms` | `double` | 总时间 |
| `n_boxes` | `int` | 盒数 |
| `build_time_ms` | `double` | 构建时间 |

---

## planner/dijkstra

> 头文件：`include/sbf/planner/dijkstra.h`

A* 搜索，h(n) = ||c_n - q_g||₂。

---

## planner/path_smoother

> 头文件：`include/sbf/planner/path_smoother.h`

5 步优化流水线：简化 → Shortcut → Densify → 弹性带 → Pass 2。
三层安全网：逐段回退 → 完全恢复 → Emergency RRT。

---

## planner/gcs_planner

> 头文件：`include/sbf/planner/gcs_planner.h`（需 `SBF_HAS_DRAKE`）

走廊粗化：超过 `max_gcs_verts` 时分组 + `hull_region_safe(depth=6)` 验证。

---

## voxel/

| 头文件 | 说明 |
|--------|------|
| `bit_brick.h` | 8³ BitBrick（64 字节缓存行） |
| `voxel_grid.h` | FNV-1a 稀疏体素哈希 |
| `hull_rasteriser.h` | 16 点凸包 turbo 扫描线 |

---

## viz/

| 头文件 | 说明 |
|--------|------|
| `viz_exporter.h` | JSON/HTML 可视化导出 |

---

## adapters/

| 头文件 | 说明 |
|--------|------|
| `ompl_adapter.h` | OMPL StateValidityChecker 适配器 |

---

## Python 绑定 (sbf6)

```python
from sbf6 import Robot, SBFPlanner, SBFPlannerConfig, EndpointSource, Obstacle

config = SBFPlannerConfig()
config.endpoint_source.source = EndpointSource.IFK

robot = Robot.from_json("iiwa14.json")
planner = SBFPlanner(robot, config)

obs = [Obstacle(0.3, -0.3, 0.0, 0.5, 0.3, 0.8)]
planner.build_coverage(obs, len(obs), timeout_ms=60000.0)

result = planner.query(start, goal)
# result.success, result.path, result.path_length, result.planning_time_ms
```

---

## v6 与 v5 对比

### 新增头文件

| 头文件 | 说明 |
|--------|------|
| `core/log.h` | 集中日志宏 |
| `core/constants.h` | 命名常量 |
| `core/union_find.h` | 并查集（从 forest/ 移入） |

### 源文件拆分

| v5 | v6 |
|----|-----|
| `grower.cpp` | `grower.cpp` + `grower_coordinated.cpp` + `grower_parallel.cpp` + `rrt_bridge.cpp` |
| `lect.cpp` | `lect.cpp` + `lect_snapshot.cpp` |
| `sbf_planner.cpp` | `sbf_planner.cpp` + `sbf_planner_build.cpp` + `sbf_planner_query.cpp` |

### 性能

| 指标 | v5 → v6 |
|------|---------|
| 包络体积 | 比特级等价 |
| 路径长度 | 比特级等价 |
| 构建时间 | 2–5% 更快 |
| 查询时间 | 2–4% 更快 |
