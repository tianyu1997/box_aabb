# P5 — Planner：状态机重构与 GCS 接口清洁化

> **目标**: 重构 v6 的 `sbf_planner.cpp`（3 个方法共 2027 行）。
> 引入显式状态机替代嵌套 if/else，清洁化 GCS Drake 接口调用，
> 修复 goal_bias 逻辑散落问题，使路径质量指标与 v6 持平或更优。
>
> **预计工时**: 4-5 天  
> **前置条件**: P4 完成（smoke_P4 + regression_P4 全部通过）  
> **完成标准**: `ctest -R smoke_P5` 通过；端到端 IIWA14 SR=100%；path_quality ≥ v6

---

## 1. v6 的设计问题总结

| 问题 | v6 现状 | v7 修复 |
|------|---------|---------|
| 单方法 2027 行 | `sbf_planner.cpp` 3个方法，最大的一个超 1000 行 | GCS 状态机，每个状态 ≤ 100 行 |
| GCS 边构建散落 | 边构建逻辑嵌入多处，跨层边混入 | `GcsGraphBuilder` 集中处理，严格父邻接 |
| PathOpt 5步固定 | 5步优化顺序硬编码，无法跳过 | `PathOptPipeline` 可配置步骤 |
| 状态无法中断 | 长运行无法提前终止 | 每个状态检查 `cancellation_token` |
| 内部状态不可观测 | 调试困难 | `PlannerStatus` 结构体，外部可轮询 |

---

## 2. 状态机设计

```
PlannerState:
  IDLE             → START_CALLED → GROWING
  GROWING          → GOAL_REACHED → BUILDING_GCS
  GROWING          → TIMEOUT      → FAILED
  BUILDING_GCS     → GCS_BUILT    → OPTIMIZING_PATH
  OPTIMIZING_PATH  → PATH_DONE    → SUCCESS
  *                → CANCELLED    → FINISHED (with status=CANCELLED)
```

---

## 3. 文件结构

```
src/planner/
  sbf_planner.cpp          - SbfPlanner 主类（状态机调度，≤ 200 行）
  gcs_graph_builder.cpp    - GCS 图构建（严格父邻接 + 叶邻接，≤ 300 行）
  path_opt_pipeline.cpp    - 5步路径优化管道（可配置，≤ 250 行）
  planner_status.cpp       - 状态查询（≤ 50 行）

include/sbf/planner/
  sbf_planner.h
  gcs_graph_builder.h
  path_opt_pipeline.h
  planner_status.h
```

---

## 4. SbfPlanner — 主类

```cpp
// include/sbf/planner/sbf_planner.h
#pragma once
#include <sbf/grower/grower_thread.h>
#include <sbf/lect/lect_tree.h>
#include "gcs_graph_builder.h"
#include "path_opt_pipeline.h"
#include "planner_status.h"
#include <atomic>
#include <functional>

namespace sbf::planner {

struct PlannerConfig {
    grower::GrowerConfig  grower;
    GcsGraphConfig        gcs;
    PathOptConfig         path_opt;
    double                timeout_s   = 60.0;
    bool                  quick_mode  = false;  // D3: --quick flag
};

struct PlanResult {
    bool              success    = false;
    std::vector<Eigen::VectorXd> path;       // 关节角序列
    double            build_time = 0.0;      // 树展开时间(s)
    double            opt_time   = 0.0;      // 路径优化时间(s)
    std::string       fail_reason;
};

class SbfPlanner {
public:
    SbfPlanner(
        lect::LectTree&             tree,
        envelope::CollisionChecker& checker,
        const envelope::CollisionConfig& coll_cfg,
        core::IEndpointSource&      ep_src,
        PlannerConfig               cfg);

    /// 阻塞式规划（适合 Python 调用）
    PlanResult plan(
        const Eigen::VectorXd& q_start,
        const Eigen::VectorXd& q_goal);

    /// 异步规划（可中途 cancel）
    void plan_async(
        const Eigen::VectorXd& q_start,
        const Eigen::VectorXd& q_goal,
        std::function<void(PlanResult)> callback);

    void cancel();

    PlannerStatus status() const;

private:
    PlannerState  state_{PlannerState::IDLE};
    std::atomic<bool> cancelled_{false};

    void transition_to(PlannerState next);
    PlanResult run_state_machine(
        const Eigen::VectorXd& q_start,
        const Eigen::VectorXd& q_goal);

    // 各阶段实现
    bool run_growing(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_goal);
    bool run_build_gcs();
    PlanResult run_optimize_path();

    lect::LectTree&             tree_;
    envelope::CollisionChecker& checker_;
    envelope::CollisionConfig   coll_cfg_;
    core::IEndpointSource&      ep_src_;
    PlannerConfig               cfg_;
};

}  // namespace sbf::planner
```

---

## 5. GcsGraphBuilder — 严格父邻接

v6 的 GCS 图构建逻辑散落在 planner 中，且存在跨层边。
v7 集中到 `GcsGraphBuilder`，强制严格父邻接。

```cpp
// include/sbf/planner/gcs_graph_builder.h
#pragma once
#include <sbf/lect/lect_tree.h>
#include <drake/planning/graph_of_convex_sets.h>

namespace sbf::planner {

struct GcsGraphConfig {
    double edge_cost_weight  = 1.0;
    bool   use_euclidean_cost = true;
    // 严格父邻接：不构建深度差 > 1 的边
    // 此参数不允许设为 false（固化 F1 原则）
};

class GcsGraphBuilder {
public:
    /// 从 LECT 树构建 GCS 图（严格父邻接）。
    /// 只创建两种边：
    ///   (1) parent_id → node_id（树展开时的边）
    ///   (2) leaf ↔ adjacent_leaf（叶邻接，深度差≤1，face_overlap 验证）
    drake::planning::GraphOfConvexSets build(
        const lect::LectTree&   tree,
        const GcsGraphConfig&   cfg,
        int32_t                 start_node_id,
        int32_t                 goal_node_id);

private:
    /// 验证边合法性：严格父邻接规则
    /// 返回 false 则拒绝建边（记录 warning）
    bool validate_edge(
        const lect::LectTree& tree,
        int32_t from_id,
        int32_t to_id) const;
};

}  // namespace sbf::planner
```

### 5.1 F1 原则的代码强制

```cpp
bool GcsGraphBuilder::validate_edge(
    const lect::LectTree& tree,
    int32_t from_id, int32_t to_id) const
{
    auto* from_node = tree.get_node(from_id);
    auto* to_node   = tree.get_node(to_id);
    if (!from_node || !to_node) return false;

    // 严格父邻接：禁止深度差 > 1 的边
    int depth_diff = std::abs(from_node->depth - to_node->depth);
    if (depth_diff > 1) {
        // 这是一个 bug，记录到日志
        spdlog::warn("Rejected cross-layer edge: {} (depth={}) → {} (depth={})",
                     from_id, from_node->depth, to_id, to_node->depth);
        return false;
    }
    return true;
}
```

---

## 6. PathOptPipeline — 可配置 5 步管道

```cpp
// include/sbf/planner/path_opt_pipeline.h
#pragma once
#include <vector>
#include <Eigen/Core>
#include <functional>

namespace sbf::planner {

enum class PathOptStep {
    GREEDY_SHORTCUT,   // step 1: 贪心路径点删减
    SHORTCUT,          // step 2: 随机捷径
    DENSIFY,           // step 3: 路径点增密（为 elastic 准备）
    ELASTIC_BAND,      // step 4: 弹性带平滑（必须在 densify 之后）
    SHORTCUT_FINAL,    // step 5: 最终捷径清理
};

struct PathOptConfig {
    std::vector<PathOptStep> steps = {
        PathOptStep::GREEDY_SHORTCUT,
        PathOptStep::SHORTCUT,
        PathOptStep::DENSIFY,
        PathOptStep::ELASTIC_BAND,
        PathOptStep::SHORTCUT_FINAL,
    };
    int    shortcut_iters   = 100;
    double densify_dt       = 0.01;  // rad
    double elastic_alpha    = 0.1;   // 弹性系数
    bool   quick_mode       = false; // D3: 精简迭代次数
};

class PathOptPipeline {
public:
    using CollisionFn = std::function<bool(const Eigen::VectorXd&)>;

    PathOptPipeline(PathOptConfig cfg, CollisionFn is_free);

    /// 运行配置的步骤序列，返回优化后的路径
    std::vector<Eigen::VectorXd> optimize(
        const std::vector<Eigen::VectorXd>& raw_path);

    /// 获取每步的路径长度（调试/论文数据用）
    std::vector<double> step_lengths() const { return step_lengths_; }

private:
    PathOptConfig cfg_;
    CollisionFn   is_free_;
    std::vector<double> step_lengths_;

    std::vector<Eigen::VectorXd> run_step(
        PathOptStep step,
        const std::vector<Eigen::VectorXd>& path);
};

}  // namespace sbf::planner
```

---

## 7. quick_mode 实现（D3 决策）

`PlannerConfig::quick_mode = true` 时，行为变化：

| 参数 | 正常 | quick 模式 |
|------|------|-----------|
| `grower.max_nodes` | 50000 | 5000 |
| `path_opt.shortcut_iters` | 100 | 10 |
| `timeout_s` | 60 | 10 |
| GCS solver timeout | 30s | 5s |

**实现位置**: `SbfPlanner::plan` 开头，若 `cfg_.quick_mode` 则覆盖上述参数。
不允许在其他地方检查 `quick_mode`；所有 quick 行为集中在此处。

---

## 8. Python binding 更新

P5 需更新 `python/sbf/` 中的 pybind11 绑定，公开 `SbfPlanner` 接口：

```python
# python/sbf/__init__.py
from ._sbf7_cpp import (
    SbfPlanner,
    PlannerConfig,
    PlanResult,
    CollisionConfig,
    UrdfEndpointSource,
    Scene,
)

# 使用示例
planner = SbfPlanner(tree, checker, collision_cfg, ep_src, cfg)
result = planner.plan(q_start, q_goal)
if result.success:
    path = result.path  # list of np.ndarray
```

**注意**: Python 模块名从 v6 的 `_sbf5_cpp` 改为 `_sbf7_cpp`。
在 `python/sbf/_bootstrap.py` 中更新 import。

---

## 9. 迁移任务列表

### Task 5.1 — 读取 v6 sbf_planner.cpp 全文结构

```bash
wc -l cpp/v6/src/sbf_planner.cpp
grep -n "^[A-Za-z].*(" cpp/v6/src/sbf_planner.cpp | head -50
```

标注出 3 个方法（`plan_path`, `build_gcs_graph`, `optimize_path`）的边界，
分别对应 P5 的哪个类。

### Task 5.2 — 实现 `GcsGraphBuilder`

- 严格父邻接（`validate_edge` 过滤跨层边）
- Drake GCS API 调用方式参考 v6 实现，不修改数学逻辑

### Task 5.3 — 实现 `PathOptPipeline`

- 5 步可配置
- `quick_mode` 下精简为 2 步（greedy_shortcut + shortcut_final）

### Task 5.4 — 实现 `SbfPlanner` 状态机

- 状态转换日志（spdlog debug 级别）
- `cancel()` → `cancelled_.store(true)` → 每个状态检查后提前退出

### Task 5.5 — Python binding

- `python/CMakeLists.txt` 中模块名 `_sbf7_cpp`
- 更新 `_bootstrap.py` 和 `__init__.py`

### Task 5.6 — 端到端回归测试

- IIWA14 5 个场景 × 3 seeds = 15 trials
- SR=100%（**若有任何 FAIL，必须在此 Phase 修复，不允许带病进 P6**）
- `path_quality`（L2 路径长度）≥ v6 平均值（不允许退化）

---

## 10. 测试矩阵

| 测试名 | 内容 | 超时 |
|--------|------|------|
| `smoke_P5_state_machine` | IDLE→GROWING→SUCCESS 状态转换正确 | 5s |
| `smoke_P5_cancel` | plan_async 期间 cancel，状态变为 CANCELLED | 5s |
| `smoke_P5_strict_adjacency` | 跨层边被拒绝，warning 日志出现 | 2s |
| `smoke_P5_path_opt_5step` | 5步管道各步骤长度严格非增 | 10s |
| `smoke_P5_quick_mode` | quick_mode 下 max_nodes=5000，plan≤10s | 15s |
| `regression_P5_e2e` | IIWA14 15 trials, SR=100%, quality≥v6 | 300s |

---

## 11. `src/planner/CMakeLists.txt`

```cmake
add_library(sbf_planner STATIC
    sbf_planner.cpp
    gcs_graph_builder.cpp
    path_opt_pipeline.cpp
    planner_status.cpp
)
target_link_libraries(sbf_planner PUBLIC
    sbf_grower
    sbf_lect
    sbf_envelope
    sbf_util
    drake::drake
)

if(BUILD_PYTHON)
    pybind11_add_module(_sbf7_cpp python/sbf7_bindings.cpp)
    target_link_libraries(_sbf7_cpp PRIVATE sbf_planner)
endif()
```

---

## 12. 已知陷阱

1. **Drake GCS API 版本**: Drake 的 GCS API 在不同版本间有变化。
   确认 v7 构建环境中的 Drake 版本，使用对应 API（参见 
   `/memories/repo/drake_version_iriszo.md`）。

2. **GCS 中的 RegionOfInterest**: v6 用 Polytope，v7 继续用 `Hyperrectangle`
   （即 AABB 的 GCS 表示）。不要引入新的 ConvexSet 类型。

3. **路径后处理的线程安全**: `PathOptPipeline` 是单线程调用，
   但其 `CollisionFn` 不是（来自多线程 planner）。
   确认 `is_free` 函数内使用正确的 mutex 或 thread_local context。

4. **Python binding 名称变更**: 所有依赖 `_sbf5_cpp` 的 Python 脚本
   在 P6 会被更新。P5 暂时保留 `_sbf5_cpp` 的 alias backward-compat import。

---

## 13. Definition of Done

- [ ] `sbf_planner` 库编译成功（4 个源文件，无单文件超 300 行）
- [ ] `smoke_P5_*` 全部通过（无 skip）
- [ ] `regression_P5_e2e` 通过（SR=100%, quality ≥ v6）
- [ ] `GcsGraphBuilder::validate_edge` 拒绝跨层边（有 warning 日志）
- [ ] `quick_mode` 逻辑集中在 `SbfPlanner::plan` 一处
- [ ] Python binding `_sbf7_cpp` 可以 `import` 并调用 `plan()`
- [ ] 顶层 `CMakeLists.txt` 中 `src/planner` 已取消注释
- [ ] 已更新 `/memories/session/plan.md` P5 打勾

---

*Phase: P5 | 依赖: P4 | 解锁: P6, P7*
