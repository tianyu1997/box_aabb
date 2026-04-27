# P4 — Forest Grower：FFB + 邻接 + 连通性 + 端点桥接

> **目标**: 在 P3 LECT 之上，构建覆盖 C-space 自由域的 BoxNode 森林，
> 维护 **box 级邻接图** 与 **连通分量（DSU）**，并在 start↔goal 不连通时
> 通过端点自动桥接修复，把可用结果交给 P5 GCS planner。
>
> **预计工时**: 4-6 天  
> **前置条件**: P3 完成（smoke_P3 全部通过）  
> **完成标准**: `ctest -R smoke_P4` 通过；2-DoF + IIWA14 单线程 SR=100%

---

## 0. 与初稿的偏差（基于 v6 实地探查）

初稿基于若干不实假设；下面用真相替换：

| 初稿声称 | v6 实地真相 | v7 P4 决策 |
|---------|-------------|-----------|
| v6 是 1200 行单函数 | 实际是 `grower.cpp` ≈1400 + `grower_coordinated.cpp` ≈800 + 5 个辅助文件，多模式 | 接受现实 |
| v6 桥接仅 AABB overlap，无面重叠 | v6 `boxes_adjacent` 已检 face-contact（tol=1e-11）；`enforce_parent_adjacency` 强制新 box 与父共享面 | **D2 已部分落地**；v7 直接 port |
| v6 有 GCS 跨层边 bug | **不存在**。adjacency 只来自 `boxes_adjacent` 面接触；不区分代际 | 取消"修跨层边"的伪需求 |
| v7 P4 必须多线程，移植 SubtreeLease | v6 走 LECT snapshot + transplant_domain 路线（无 SubtreeLease），且并行只在多 goal 时启用 | **P4 单线程版优先**，并行延后到 P4.5 |
| 必须实现"K 次 REJECT 后细分父" | v6 没有此特性；只有 `max_consecutive_miss` 退出 | **删除**；v7 P4 不引入未验证的新机制 |
| GoalBiasPolicy 类 | v6 是 3 行 inline `if (u01 < bias) q=goal` | 不抽象成类 |
| AdjacentLeafFinder 深度差 ≤1 | v6 没有这个限制；adjacency 纯几何 | 不引入 |

**核心简化原则**: P4 = "v6 grower 单线程精华版 + 边界清洁化"。
不做并行、不做反应式细分、不引入未在 v6 验证的新算法。

---

## 1. 模块结构

```
include/sbf/forest/
  ffb.h              - Find-Free-Box 接口（FFBConfig/FFBResult）
  adjacency.h        - boxes_adjacent / compute_adjacency / find_islands
  union_find.h       - 简易 DSU
  bridge.h           - enforce_parent_adjacency / endpoint_auto_bridge
  grower.h           - ForestGrower 主类（单线程）

src/forest/
  ffb.cpp            - LECT 下降 + expand + 碰撞 → 自由叶节点
  adjacency.cpp      - boxes_adjacent + 邻接图构建 + 连通分量
  bridge.cpp         - 父邻接强制 + RRT 桥接孤立岛
  grower.cpp         - RRT 主循环 + 连通跟踪
  CMakeLists.txt
```

每个 `.cpp` ≤ 500 行（C1 红线）。

---

## 2. 公共 API

### 2.1 `sbf::forest::FFBConfig` / `FFBResult` / `find_free_box`

```cpp
namespace sbf::forest {

struct FFBConfig {
    int    max_depth      = 30;
    double deadline_ms    = 0.0;          // 0 = unlimited
    bool   seed_known_free = false;       // skip seed collision check if pre-verified
};

struct FFBResult {
    int                node_idx = -1;     // LECT leaf containing seed (free)
    std::vector<int>   path;              // root → leaf
    int                fail_code = 0;     // 0=ok, 1=occupied, 2=max_depth, 3=collision_at_root, 4=deadline
    int                n_new_nodes = 0;   // expand_leaf calls
    int                n_collide_calls = 0;
    int                n_steps = 0;
    bool success() const { return fail_code == 0 && node_idx >= 0; }
};

FFBResult find_free_box(
    sbf::lect::LECT& lect,
    const Eigen::VectorXd& seed,
    const float* obs_compact, int n_obs,
    const FFBConfig& cfg = {});

}
```

**算法**（直接复刻 v6 简化版）：
```
cur = 0 (root)
while depth(cur) < max_depth:
    if collides_scene(cur, obs): fail_code=3 (root) / 退到上一层
    if is_leaf(cur):
        if not contains seed: fail
        if is_occupied(cur): fail_code=1; return
        return cur
    // descend by split plane
    cur = (seed[split_dim] <= split_val) ? left : right
// reached max_depth: try expand_leaf if leaf, else return current cur as best-effort
```

### 2.2 `sbf::forest::union_find` / `adjacency`

```cpp
class UnionFind {
public:
    explicit UnionFind(int n);
    int  find(int x);
    bool unite(int a, int b);          // returns true if a merge happened
    bool connected(int a, int b);
    int  num_components() const;
    void reset(int n);
};

/// Two BoxNodes are adjacent if every dim either:
///   • has positive overlap (≥ tol), or
///   • has face-contact (overlap_hi - overlap_lo < tol but ≥ 0).
/// AND at least one dim has face-contact (otherwise they'd be interior overlap).
/// Returns true on full volumetric overlap too (interpenetrating boxes).
bool boxes_adjacent(const sbf::scene::BoxNode& a,
                    const sbf::scene::BoxNode& b,
                    double tol = 1e-11);

struct AdjacencyGraph {
    std::vector<std::vector<int>> nbrs;   // box_idx -> neighbor box indices
    int n_edges = 0;
};

AdjacencyGraph compute_adjacency_graph(
    const std::vector<sbf::scene::BoxNode>& boxes, double tol = 1e-11);

/// Connected components (DSU). Returns vector of component IDs per box.
std::vector<int> find_islands(const AdjacencyGraph& g);
```

### 2.3 `sbf::forest::bridge`

```cpp
/// After a new box is created, snap one face to touch its parent if the gap
/// is small (< 0.05 rad). If gap requires extending across collision, abort
/// (returns false). Mirrors v6 `enforce_parent_adjacency`.
bool enforce_parent_adjacency(
    sbf::scene::BoxNode& new_box,
    const sbf::scene::BoxNode& parent_box,
    sbf::lect::LECT& lect,
    const float* obs_compact, int n_obs);

/// If start_box and goal_box are in different islands of the adjacency graph,
/// run a serial RRT pass with high goal-bias to add bridge boxes. Caps at
/// `extra_budget` boxes. Returns # bridges added.
int endpoint_auto_bridge(
    sbf::lect::LECT& lect,
    std::vector<sbf::scene::BoxNode>& boxes,
    int start_box, int goal_box,
    const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_goal,
    const float* obs_compact, int n_obs,
    int extra_budget,
    uint64_t rng_seed);
```

### 2.4 `sbf::forest::ForestGrower`（单线程）

```cpp
struct GrowerConfig {
    int      max_boxes            = 500;
    double   timeout_ms           = 30000.0;
    int      max_consecutive_miss = 2000;
    double   rrt_goal_bias        = 0.1;
    double   rrt_step_ratio       = 0.05;     // step as fraction of joint range
    bool     connect_mode         = true;     // stop when start↔goal connected
    bool     stop_after_connect   = false;
    bool     endpoint_auto_bridge = true;
    int      endpoint_bridge_max_boxes = 0;   // 0 → 5% of max_boxes, clamped [50,500]
    uint64_t rng_seed             = 42;
    sbf::forest::FFBConfig ffb;
};

struct GrowerResult {
    std::vector<sbf::scene::BoxNode> boxes;
    int    n_ffb_success    = 0;
    int    n_ffb_fail       = 0;
    int    n_bridge_boxes   = 0;
    bool   start_goal_connected = false;
    bool   adjacency_all_connected = false;
    int    adjacency_islands = 0;
    int    adjacency_largest_island = 0;
    double build_time_ms    = 0.0;
};

class ForestGrower {
public:
    ForestGrower(const sbf::core::Robot& robot,
                 sbf::lect::LECT& lect,
                 GrowerConfig cfg = {});

    void set_endpoints(const Eigen::VectorXd& q_start,
                       const Eigen::VectorXd& q_goal);

    GrowerResult grow(const float* obs_compact, int n_obs);
};
```

---

## 3. 多线程问题（关键议题 1）

### 3.1 v6 实地真相
- v6 的并行（`grow_coordinated` / `grow_parallel`）依赖 LECT 的 **snapshot + transplant_domain + partition_for_seeds**。
- v7 LECT 当前只实现了 `snapshot()`，**没有** `transplant_subtree()` 也没有 `partition_for_seeds()`。
- v6 的并行实现通过"几何域隔离"消除全局锁——但这需要 LECT 支持区域所有权。

### 3.2 v7 P4 决策：**单线程优先 + 显式并行接缝**
- **P4 主体只实现单线程 grower**。这是正确性基线，覆盖 ≥95% 的实际用例（IIWA + 简单场景 SR=100%）。
- **不引入** `SubtreeLease` 抽象。v6 没有它；v7 也不需要。
- **预留并行接缝**（不实现）：
  - `ForestGrower` 构造时接受 `int n_threads` 但 P4 仅实现 `n_threads==1` 分支。`n_threads>1` 抛 `std::runtime_error("P4.5 not yet")`。
  - LECT 端待后续阶段补充：`partition_for_seeds(seeds)`、`transplant_subtree(worker, snapshot_base, id_map)`。
- **何时启用并行**: 当 IIWA14 单线程 build_time > 2× v6 时启动 P4.5（独立分支）。否则不做。

### 3.3 线程安全断言（即使单线程也明确）
```cpp
// 文档化断言：grow() 期间，外部不得访问 lect_、boxes_。
// boxes_/lect_ 的变更只发生在 grow() 内部主循环。
// goal_reached_ 是单线程内的普通 bool，不需 atomic。
```

---

## 4. 连通性问题（关键议题 2）

### 4.1 三个层次的连通性

v6 同时维护三套口径，造成混乱。v7 P4 **明确三层**：

| 层级 | 数据结构 | 何时维护 | 用途 |
|-----|---------|---------|------|
| **(L1) Tree-UF** | `UnionFind(n_trees)` | 多 goal 模式增量维护 | 提前停止条件（`stop_after_connect`） |
| **(L2) Box-DSU** | `UnionFind(n_boxes)` | 每次新 box 增量与所有现有 box 试 `boxes_adjacent` | 端点连通判定（`start_goal_connected`） |
| **(L3) Final adjacency graph** | `AdjacencyGraph` + `find_islands` | grow() 结束后一次性构建 | 交给 P5 GCS planner 当作图 |

**v7 P4 仅实现 L2 + L3**（不做多 goal）。L1 留给 P5 多 goal 实验。

### 4.2 L2 增量维护（O(N²) 总开销，但 box 数 ≤ max_boxes ≤ 500）

```cpp
// 新 box 加入：
int new_idx = boxes_.size() - 1;
for (int i = 0; i < new_idx; ++i) {
    if (boxes_adjacent(boxes_[new_idx], boxes_[i])) {
        box_uf_.unite(new_idx, i);
    }
}
// 检查端点连通
if (start_box_ >= 0 && goal_box_ >= 0 &&
    box_uf_.connected(start_box_, goal_box_)) {
    start_goal_connected_ = true;
    if (cfg_.stop_after_connect) break;
}
```

每次 O(N) 检查；500 个 box 总开销 ~125k 次 boxes_adjacent，每次 O(n_dims)，总 < 50ms。

### 4.3 父邻接强制（D2 落地点）

每次 FFB 成功后，**立即** 调用 `enforce_parent_adjacency(new_box, parent_box, lect, obs)`：
- 若已 `shared_face`：直接返回 true。
- 若沿某轴 gap ∈ (0, 0.05)：扩展 new_box 的对应面与 parent 面贴合，**碰撞重检**。
- 若 gap ≥ 0.05：放弃扩展，返回 true（仍认为 box 有效，但本对不直接邻接——它会通过其它 box 链路连通或最终走 `endpoint_auto_bridge`）。

**这是 D2 的实际落地形式**：在 v6 的成熟语义上保持，不发明新阈值。

### 4.4 Endpoint Auto-Bridge（C-1）

如果 `grow()` 结束后 `start_box_` 与 `goal_box_` 不在同一连通分量：

```
budget = cfg.endpoint_bridge_max_boxes (default = clamp(0.05*max_boxes, 50, 500))
loop while budget > 0 and not connected:
    q_rand = (u01 < 0.9) ? (toward goal_island) : random
    nearest_box = find_nearest_box_in_start_island(q_rand)
    seed = nearest_box.center + step_ratio * (q_rand - nearest_box.center)
    res = find_free_box(lect, seed, obs)
    if res.success(): create_box_from_leaf(res); update DSU; budget--
```

简化版：偏置直接朝 goal_box.center；不需要"沿 island 边界"复杂搜索。

---

## 5. 主循环（单线程 RRT）

```cpp
GrowerResult ForestGrower::grow(const float* obs, int n_obs) {
    // 1. 创建 root box from start (and goal)
    if (q_start.size()) ensure_box_at(q_start, /*is_start=*/true);
    if (q_goal.size())  ensure_box_at(q_goal,  /*is_start=*/false);

    int miss = 0;
    auto t0 = Clock::now();
    while ((int)boxes_.size() < cfg_.max_boxes && miss < cfg_.max_consecutive_miss) {
        if (elapsed_ms(t0) > cfg_.timeout_ms) break;
        if (cfg_.stop_after_connect && start_goal_connected_) break;

        Eigen::VectorXd q = sample_q();              // RRT bias
        int parent = find_nearest_box_index(q);
        Eigen::VectorXd seed = step_toward(boxes_[parent].center(), q);

        FFBResult r = find_free_box(lect_, seed, obs, n_obs, cfg_.ffb);
        if (!r.success()) { miss++; continue; }

        BoxNode nb = build_box_from_leaf(r.node_idx, parent);
        enforce_parent_adjacency(nb, boxes_[parent], lect_, obs, n_obs);
        boxes_.push_back(nb);
        lect_.mark_occupied(r.node_idx, (int)boxes_.size() - 1);
        update_box_uf(/*new_idx=*/boxes_.size() - 1);
        miss = 0;
    }

    // Phase C-1
    if (cfg_.endpoint_auto_bridge && !start_goal_connected_)
        endpoint_auto_bridge(lect_, boxes_, start_box_, goal_box_,
                             q_start_, q_goal_, obs, n_obs,
                             effective_bridge_budget(), cfg_.rng_seed + 1);

    // Final adjacency graph
    auto g = compute_adjacency_graph(boxes_);
    auto comp = find_islands(g);
    fill_result_metrics(g, comp);
    return std::move(result_);
}
```

---

## 6. Smoke Tests — `smoke_p4.cpp`

| TEST | 验证 |
|------|------|
| `P4.UnionFindBasic` | unite/find/connected/num_components 在 6 节点上正确 |
| `P4.BoxesAdjacentFaceContact` | 两 box 沿 dim 0 面接触（tol 内） → adjacent=true |
| `P4.BoxesAdjacentSeparated` | 同 box 错开 0.5 → adjacent=false |
| `P4.BoxesAdjacentInterior` | 完全重叠 → adjacent=true |
| `P4.AdjacencyGraphIslands` | 4 boxes（两两相邻+独立），find_islands 给出 2 个分量 |
| `P4.FFBSuccessFromRoot` | 2-DoF 平面机器人，无障碍，FFB(seed) 成功 |
| `P4.FFBOccupiedReject` | mark_occupied 后再 FFB 同 leaf → fail_code=1 |
| `P4.FFBCollisionFails` | 障碍盖住整个 root，FFB 失败 |
| `P4.GrowerStartGoalConnected2DoF` | 2-DoF 无障碍，start/goal connect=true，box 数 ≥ 2 |
| `P4.GrowerEndpointAutoBridge` | 2-DoF 障碍把 C-space 切成两半但留缝隙，bridge 让 start/goal 连通 |
| `P4.GrowerIIWA14SmallScene` | IIWA14 + 1 远障碍，max_boxes=50，SR=100%，build_time<2s |

目标 < 5 s 总用时。

---

## 7. CMake

```cmake
# src/forest/CMakeLists.txt
add_library(sbf_forest STATIC
    ffb.cpp adjacency.cpp bridge.cpp grower.cpp)
target_link_libraries(sbf_forest PUBLIC
    sbf_lect sbf_envelope sbf_scene sbf_core sbf_util Eigen3::Eigen)
target_compile_options(sbf_forest PRIVATE -Wall -Wextra)
```

顶层 `CMakeLists.txt` 取消注释 `add_subdirectory(src/forest)`（注意 v7 现状是 `src/grower`，统一改为 `src/forest`）。

---

## 8. 已知陷阱

1. **boxes_adjacent 的 tol**: v6 用 1e-11，与 D2 的 ε_face 概念不冲突——D2 关心"面接触"是布尔，v6 已有；不引入面积权重（避免 v6 未验证的新算法）。

2. **FFB 与 LECT 占用语义**:
   `lect_.mark_occupied(leaf, box_id)` 必须在 `boxes_.push_back` 之后（box_id = boxes_.size()-1 已稳定）。否则 box_id 错位。

3. **enforce_parent_adjacency 的回退**:
   若 gap ≥ 0.05 不扩展，box 仍保留，但与 parent 不直接邻接。**不要** 因此放弃 box——它可能通过别的链路连通，最终由 endpoint_auto_bridge 修复孤立。

4. **RRT step_toward 的归一化**:
   各 dim 的 range 不同（IIWA q1 范围 5.93 vs q4 范围 4.19），step 必须按每 dim 归一化后再计算欧氏，否则 step_ratio 失真。

5. **find_nearest_box_index O(N)**:
   500 个 box 没问题（~50us）。若 N 增大到 5000+ 再考虑 kd-tree。**P4 不预优化**。

6. **ensure_box_at 失败处理**:
   如果 q_start 本身碰撞（FFB 失败），返回 `INFEASIBLE` 给上层（F2 原则）。不要在主循环里反复尝试。

---

## 9. Definition of Done

- [ ] `sbf_forest` 编译成功，无 `-Wall -Wextra` 新增警告
- [ ] `smoke_P4` 11 个 TEST 全部通过（< 5 s 总用时）
- [ ] 单线程 IIWA14 SR=100%（小场景 ≥ 5 trials 确认）
- [ ] `enforce_parent_adjacency` 是 v7 中**唯一**修改 box.joint_intervals 边界的位置
- [ ] L2 box-DSU 和 L3 adjacency_graph 二者结果一致（在 grow() 末尾断言 `box_uf.connected(start,goal) == islands[start]==islands[goal]`）
- [ ] 顶层 `CMakeLists.txt` 中 `src/forest` 取消注释
- [ ] `/memories/session/plan.md` P4 打勾，记录单线程现状 + P4.5 并行延后

---

## 10. P4.5 并行扩展（不在 P4 范围）

文档化为后续阶段：

1. LECT 新增 `transplant_subtree(worker, snapshot_base, id_map)`（v6 实现可参考）。
2. LECT 新增 `partition_for_seeds(seeds, max_extra_splits)`，返回 leaf-per-seed 列表。
3. `ForestGrower` 新增 `grow_parallel()` 内部调用：
   - master 调 `partition_for_seeds(roots)` 得 N 个域
   - N 个 worker 各持 `lect.snapshot()`，在自己域内 RRT
   - join 时 `transplant_subtree`，box id 重映射到全局
4. 启动条件：`n_threads > 1 && multi_goals.size() ≥ 2`。

P4.5 决策：**仅当 P5 实验测得 v7 单线程 build_time > 1.5× v6 时才执行**。否则永久搁置。

---

*Phase: P4 | 依赖: P3 | 解锁: P5 | 多线程: 延至 P4.5*
