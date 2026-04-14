# SafeBoxForest v6 — 各 Phase 详细执行计划

> 基于 v5 完整代码分析生成。每个 Phase 列出精确的文件、行号、操作步骤。

---

## Phase 0: 项目骨架搭建

### 目标
复制 v5 到 v6, 编译通过, 全部测试通过, 记录 v5 性能基线。

### 步骤

| # | 操作 | 详情 |
|---|------|------|
| 0.1 | 复制 v5 → v6 | `cp -r cpp/v5/* cpp/v6/` (排除 build/, _sbf5_deps/, result/) |
| 0.2 | 更新 CMakeLists.txt | 版本号 5.0.0 → 6.0.0; `_sbf5_deps` → `_sbf6_deps` |
| 0.3 | 更新 FetchDeps.cmake | `FETCHCONTENT_BASE_DIR` 改为 `_sbf6_deps` |
| 0.4 | 更新顶层 build cmake | 添加 v6 到 cpp/CMakeLists.txt (如有) |
| 0.5 | 编译验证 | `cmake -B build -S . && cmake --build build -j$(nproc)` |
| 0.6 | 测试验证 | `cd build && ctest --output-on-failure` |
| 0.7 | 记录基线 | 运行 exp2 --seeds 3, 记录到 `result/v5_baseline.txt` |

### 验收
- 编译零错误
- ctest 全部通过
- 基线数据已记录

---

## Phase 1: 日志系统与配置常量

### 目标
引入 `SBF_LOG` 宏替换所有 `fprintf(stderr, ...)`, 集中魔数到 `constants.h`。

### 1.1 新建 `include/sbf/core/log.h`

```cpp
#pragma once
#include <cstdio>

namespace sbf {
enum class LogLevel : int { SILENT=0, ERROR=1, WARN=2, INFO=3, DEBUG=4, TRACE=5 };

inline LogLevel& global_log_level() {
    static LogLevel lvl = LogLevel::INFO;
    return lvl;
}
inline void set_log_level(LogLevel lvl) { global_log_level() = lvl; }
inline void set_log_level_from_env() {
    if (const char* e = std::getenv("SBF_LOG_LEVEL")) {
        int v = std::atoi(e);
        if (v >= 0 && v <= 5) global_log_level() = static_cast<LogLevel>(v);
    }
}
} // namespace sbf

#define SBF_LOG(level, fmt, ...) \
    do { if (static_cast<int>(sbf::global_log_level()) >= static_cast<int>(level)) \
        std::fprintf(stderr, fmt "\n", ##__VA_ARGS__); } while(0)

#define SBF_INFO(fmt, ...)  SBF_LOG(sbf::LogLevel::INFO,  fmt, ##__VA_ARGS__)
#define SBF_DEBUG(fmt, ...) SBF_LOG(sbf::LogLevel::DEBUG, fmt, ##__VA_ARGS__)
#define SBF_WARN(fmt, ...)  SBF_LOG(sbf::LogLevel::WARN,  fmt, ##__VA_ARGS__)
#define SBF_TRACE(fmt, ...) SBF_LOG(sbf::LogLevel::TRACE, fmt, ##__VA_ARGS__)
```

### 1.2 fprintf → SBF_LOG 替换清单

| 文件 | fprintf 数量 | 操作 |
|------|-------------|------|
| `src/planner/sbf_planner.cpp` | 91 | `fprintf(stderr, "[PLN] ...")` → `SBF_INFO("[PLN] ...")` |
| `src/forest/grower.cpp` | 47 | `fprintf(stderr, "[GRW] ...")` → `SBF_INFO("[GRW] ...")` |
| `src/forest/connectivity.cpp` | 18 | `fprintf(stderr, "[BRG] ...")` → `SBF_INFO("[BRG] ...")` |
| `src/lect/z4_ep_cache.cpp` | 11 | `fprintf(stderr, "[Z4E] ...")` → `SBF_INFO("[Z4E] ...")` |
| `src/lect/z4_grid_cache.cpp` | 10 | `fprintf(stderr, "[Z4G] ...")` → `SBF_INFO("[Z4G] ...")` |
| `src/lect/lect_cache_manager.cpp` | 10 | `fprintf(stderr, "[LCM] ...")` → `SBF_INFO("[LCM] ...")` |
| `src/planner/gcs_planner.cpp` | 6 | `fprintf(stderr, "[GCS] ...")` → `SBF_INFO("[GCS] ...")` |
| `src/lect/lect_io.cpp` | 5 | → `SBF_INFO(...)` |
| `src/lect/lect.cpp` | 2 | → `SBF_WARN(...)` |
| `src/viz/viz_exporter.cpp` | 1 | → `SBF_WARN(...)` |

**替换规则**: 保持输出格式完全一致（实验脚本解析兼容）。只修改调用宏，不改格式字符串。
- `fprintf(stderr, "xxx\n", args...)` → `SBF_INFO("xxx", args...)`
- 注意: SBF_LOG 已自动附加 `\n`, 原 fprintf 的 `\n` 需移除

### 1.3 新建 `include/sbf/core/constants.h`

从代码中提取的常量 (值不变):

```cpp
#pragma once
namespace sbf::constants {
    // Adjacency
    constexpr double kAdjacencyTol          = 1e-6;
    // Bridge / Connectivity  
    constexpr double kBridgeGapMax          = 1e-4;
    constexpr double kOverlapMargin         = 1e-8;
    // FFB
    constexpr int    kFFBDeadlineSample     = 64;
    // Dijkstra
    constexpr double kDijkstraHopPenalty    = 0.02;
    // Collision
    constexpr double kSlabEpsilon           = 1e-15;
    // Analytical envelope
    constexpr double kPhase123Threshold     = 0.15;
} // namespace sbf::constants
```

### 验收
- 编译通过, ctest 全部通过
- `grep -rc 'fprintf' src/ | grep -v ':0$'` 返回 0 (除了 snprintf)

---

## Phase 2: 核心工具提取

### 目标
消除 3 处 DH 矩阵重复, slab test 重复, UnionFind 重复。

### 2.1 新建 `include/sbf/core/dh_matrix.h` (header-only inline)

提取自:
- `src/scene/collision_checker.cpp` 匿名 ns 中的 DH 构造 (~30 行)
- `src/envelope/analytical_source.cpp` 内的相同代码
- `src/envelope/dh_enumerate.cpp` 内的相同代码

```cpp
#pragma once
#include <cmath>
namespace sbf {
/// Build 4×4 DH transform (row-major) in-place.
inline void build_dh_matrix(double a, double alpha, double d, double theta,
                            double out[16]) {
    double ct = std::cos(theta), st = std::sin(theta);
    double ca = std::cos(alpha), sa = std::sin(alpha);
    out[0]=ct; out[1]=-st*ca; out[2]=st*sa;   out[3]=a*ct;
    out[4]=st; out[5]=ct*ca;  out[6]=-ct*sa;  out[7]=a*st;
    out[8]=0;  out[9]=sa;     out[10]=ca;      out[11]=d;
    out[12]=0; out[13]=0;     out[14]=0;       out[15]=1;
}
} // namespace sbf
```

原文件中的内联版本删除, 改为 `#include <sbf/core/dh_matrix.h>` + 调用。

### 2.2 collision_checker.cpp slab test 循环化

当前: XYZ 各 ~12 行重复 = ~36 行
改为: 一个 3 次循环 = ~15 行

```cpp
// 替换前: 独立的 X, Y, Z slab 测试
// 替换后:
for (int axis = 0; axis < 3; ++axis) {
    double dir_a = dir[axis], orig_a = origin[axis];
    double lo_a = obs_lo[axis], hi_a = obs_hi[axis];
    if (std::abs(dir_a) < kSlabEpsilon) {
        if (orig_a < lo_a || orig_a > hi_a) { hit = false; break; }
        continue;
    }
    double inv = 1.0 / dir_a;
    double t1 = (lo_a - orig_a) * inv, t2 = (hi_a - orig_a) * inv;
    if (t1 > t2) std::swap(t1, t2);
    t_enter = std::max(t_enter, t1);
    t_exit  = std::min(t_exit,  t2);
    if (t_enter > t_exit) { hit = false; break; }
}
```

### 2.3 新建 `include/sbf/core/union_find.h`

将 `connectivity.cpp` 的 `UnionFind` 类声明从 `include/sbf/forest/connectivity.h` 移到 `core/union_find.h`。

`grower.cpp` 中匿名 ns 的 `InlineUF` 删除, 改用 `#include <sbf/core/union_find.h>`。

### 验收
- 编译通过, ctest 全部通过
- `grep -r 'build_dh_4x4\|dh_matrix_inline' src/` → 只引用 dh_matrix.h  
- bench_pipeline 性能 ≥ v5

---

## Phase 3: 拆分 sbf_planner.cpp (2,027行 → ~4文件)

### 目标
最大单函数 ≤ 200 行, 最大单文件 ≤ 800 行。

### 3.1 文件拆分方案

| 新文件 | 方法 | 行范围 (v5) | 预计行数 |
|--------|------|-------------|----------|
| `sbf_planner.cpp` | 构造函数, lect_auto_cache_path, clear_forest, plan() | L1-22, L1898-2027 | ~200 |
| `sbf_planner_build.cpp` | init_lect_(), build(), warmup_lect(), build_coverage() | L23-758 | ~500 |
| `sbf_planner_query.cpp` | query() 拆分为子步骤 | L759-1500(约) | ~500 |
| `sbf_planner_optimize.cpp` | 路径优化 (query 后半) | L1500-1897(约) | ~350 |

### 3.2 LECT 初始化去重

当前重复 3 次 (build L50-96, warmup_lect L270-314, build_coverage L345-383):
```
1. 创建 LECT from root_ivs
2. set_split_order
3. disable_z4 (if needed)
4. load cache
5. init V6 cache manager
6. log timing
```

提取为私有方法:
```cpp
// sbf_planner.h 中新增:
private:
    void init_lect_();  // 创建 + 加载 + 配置
```

### 3.3 query() 子步骤拆分

当前 query() L759-1897 = 1,138 行。拆分为:

```cpp
// sbf_planner.h 中新增:
private:
    struct QueryContext {
        Eigen::VectorXd start, goal;
        const Obstacle* obs; int n_obs;
        int start_box_id = -1, goal_box_id = -1;
        bool start_isolated = false, goal_isolated = false;
        std::vector<Eigen::VectorXd> proxy_start_path, proxy_goal_path;
        std::vector<int> box_seq;
        std::vector<Eigen::VectorXd> waypoints;
        // timing
        double find_box_ms=0, bridge_ms=0, search_ms=0, rrt_ms=0;
        double optimize_ms=0, validate_ms=0;
    };

    bool locate_endpoints_(QueryContext& ctx);      // ~100行
    bool bridge_endpoints_(QueryContext& ctx);       // ~150行
    bool search_path_(QueryContext& ctx);            // ~100行
    bool rrt_compete_(QueryContext& ctx);            // ~150行
    void optimize_path_(QueryContext& ctx);          // ~300行 → 进一步拆到 optimize 文件
    PlanResult finalize_result_(QueryContext& ctx);  // ~50行
```

### 3.4 路径优化模板化

当前 greedy+shortcut+elastic+densify 在 pass1 和 pass2 各出现一次。

```cpp
struct OptimizePassConfig {
    bool do_greedy = false;
    int  shortcut_iters = 0;
    bool do_elastic = false;
    double elastic_alpha = 0.0;
    int  elastic_iters = 0;
    bool do_densify = false;
    double densify_resolution = 0.0;
};

// 在 sbf_planner_optimize.cpp 中:
void SBFPlanner::run_optimize_pass_(
    QueryContext& ctx, const OptimizePassConfig& pass_cfg);
```

### 3.5 CMakeLists.txt 更新

```cmake
add_library(sbf_planner STATIC
    src/planner/sbf_planner.cpp
    src/planner/sbf_planner_build.cpp
    src/planner/sbf_planner_query.cpp
    src/planner/sbf_planner_optimize.cpp
    src/planner/dijkstra.cpp
    src/planner/gcs_planner.cpp
    src/planner/path_extract.cpp
    src/planner/path_smoother.cpp
)
```

### 验收
- 编译通过, ctest 全部通过
- `wc -l src/planner/sbf_planner*.cpp` 每个 < 600
- exp2 --seeds 3 性能 ±5%

---

## Phase 4: 拆分 forest 模块

### 目标
拆分 grower.cpp (1,951行) 和 connectivity.cpp (1,231行)。

### 4.1 grower.cpp → 3 文件

| 新文件 | 方法 | 行范围 | 预计行数 |
|--------|------|--------|----------|
| `grower.cpp` | 构造, set_*, helpers, try_create_box, enforce_parent_adjacency, snap_to_face, sample_boundary, select_roots, grow_rrt, grow_wavefront, promote_all, grow | L1-853 | ~850 |
| `grower_parallel.cpp` | grow_subtree, grow_parallel | L1027-1279 | ~250 |
| `grower_coordinated.cpp` | grow_coordinated | L1280-1951 | ~670 |

**也可选方案**: 只拆 grow_coordinated 出去 (它是最独立的)。

### 4.2 connectivity.cpp → 2 文件

| 新文件 | 函数 | 行范围 | 预计行数 |
|--------|------|--------|----------|
| `connectivity.cpp` | UnionFind(→引用core/), find_islands, bridge_s_t, bridge_all_islands | L31-65, L798-end | ~500 |
| `rrt_bridge.cpp` | rrt_nearest/steer/extend/connect_greedy/extract_path(匿名ns), rrt_connect, bitstar_bridge, pave_snap_seed, find_containing_box, add_adj_edge, commit_box, chain_pave_along_path, repair_bridge_adjacency | L112-797 | ~680 |

### 4.3 coarsen.cpp 清理 (原地)

- 删除未使用的 `geometric_mean_edge()` (若确认未使用)
- dead-box 标记模式: 保持现状 (低优先级, 算法相关风险)
- 魔数 → `constants.h` 引用

### 4.4 adjacency.cpp 子函数化 (原地)

拆分 `compute_adjacency()` (258行) 为:
- `sweep_find_pairs()` — 维度扫描找相邻对
- `build_adj_graph()` — 构建邻接图
- `prune_degree()` — 度数裁剪

### 4.5 CMakeLists.txt 更新

```cmake
add_library(sbf_forest STATIC
    src/forest/grower.cpp
    src/forest/grower_parallel.cpp
    src/forest/grower_coordinated.cpp
    src/forest/adjacency.cpp
    src/forest/coarsen.cpp
    src/forest/connectivity.cpp
    src/forest/rrt_bridge.cpp
)
```

### 验收
- 编译通过, ctest 全部通过
- `wc -l src/forest/*.cpp` 每个 < 900
- test_grower, test_coarsen 通过
- exp2 --seeds 3 性能 ±5%

---

## Phase 5: LECT + Envelope 清理

### 目标
拆分 lect.cpp (1,533行), 清理 analytical_source.cpp 重复。

### 5.1 lect.cpp → 2 文件

| 新文件 | 方法 | 行范围 | 预计行数 |
|--------|------|--------|----------|
| `lect.cpp` | 构造, materialize_mmap, ensure_capacity, alloc_node, node_intervals, z4_*, pick_split_dim*, split_leaf_impl, expand_leaf, warmup, collides_scene*, intervals_collide_scene, mark/unmark/clear | L22-1277 | ~800 |
| `lect_snapshot.cpp` | snapshot(), transplant_subtree() | L1279-end | ~250 |

或备选: 将 compute_envelope (L311-619, ~310行) 抽出:
| `lect_envelope.cpp` | compute_envelope, derive_merged_link_iaabb, materialise_link_iaabb, compute_node_grids | L311-1123 | ~500 |

### 5.2 analytical_source.cpp QR 求解器提取

Phase1/2/3 各 ~130 行，内部模式相同:
1. 采样候选点 → 建 QR
2. 求解 atan2
3. clamp 到区间
4. 更新端点

提取:
```cpp
// 在 analytical_source.cpp 内 (匿名ns):
struct CritSolver {
    Eigen::ColPivHouseholderQR<Eigen::Matrix3d> qr;
    void setup(double q0, double q1, double q2);
    void solve_and_update(double lo, double hi,
                          const Eigen::Vector3d& target,
                          double& ep_lo, double& ep_hi);
};
```

Phase1/2/3 循环体从 ~130行 → ~50行 (调用 solver)。

### 5.3 FFBResult 字段清理

- 删除 `FFBResult::path` (从未赋值)
- 删除未填充的 timing 字段: `envelope_ms`, `collide_ms`, `expand_ms`
  (需确认无使用者: grep 验证)

### 5.4 CMakeLists.txt 更新

```cmake
add_library(sbf_lect STATIC
    src/lect/lect.cpp
    src/lect/lect_envelope.cpp    # 新增 (或 lect_snapshot.cpp)
    src/lect/lect_io.cpp
    src/lect/lect_mmap.cpp
    src/lect/z4_ep_cache.cpp
    src/lect/z4_grid_cache.cpp
    src/lect/lect_cache_manager.cpp
)
```

### 验收
- 编译通过, ctest 全部通过
- test_lect, test_lect_io, bench_ffb 通过
- exp1 envelope volume 数值不变
- bench_endpoint_iaabb 性能 ±3%

---

## Phase 6: GCS + 路径模块清理

### 目标
拆分 gcs_plan() (455行), 共享 face_center, 修复 path_smoother 边界。

### 6.1 gcs_planner.cpp 子函数化 (原地)

将 `gcs_plan()` (L220-553) 拆为:

```cpp
// 匿名 ns 或 static:
static std::vector<int> gcs_search_corridor_(
    const std::vector<BoxNode>& boxes, const AdjacencyGraph& adj,
    int start_id, int goal_id);                    // ~80行

static std::vector<CorridorGroup> gcs_build_groups_(
    const std::vector<int>& corridor,
    const std::vector<BoxNode>& boxes,
    const AdjacencyGraph& adj, int max_verts);     // ~100行

static GCSResult gcs_solve_drake_(
    const std::vector<CorridorGroup>& groups,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal,
    const GCSConfig& cfg);                          // ~120行

// gcs_plan() 变为 ~60行调度
GCSResult gcs_plan(...) {
    auto corridor = gcs_search_corridor_(...);
    auto groups = gcs_build_groups_(...);
    return gcs_solve_drake_(...);
}
```

### 6.2 共享 face_center

当前 `dijkstra.cpp` 和 `path_extract.cpp` 各有 `face_center` 实现。

方案: 在 `include/sbf/planner/path_extract.h` 中声明:
```cpp
Eigen::VectorXd face_center(const BoxNode& a, const BoxNode& b);
```

`dijkstra.cpp` 中删除本地版本, 改为调用 `path_extract.h` 的版本。

### 6.3 path_smoother.cpp 边界修复

`shortcut()` 中 `dist_j(i+2, n-1)`: 当 `n < 3` 时 UB。加保护:
```cpp
if (n < 3) return path; // 不够短路
```

### 验收
- 编译通过, ctest 全部通过
- test_planner 通过
- exp3 --seeds 3 --gcs 性能 ±5%

---

## Phase 7: Viz / Adapters / 死代码清理

### 目标
JSON 序列化去重, OMPL config 结构化, 删除死代码。

### 7.1 viz_exporter.cpp JSON 序列化去重

提取:
```cpp
// 匿名 ns helper:
static nlohmann::json serialize_box(const BoxNode& box);        // ~15行
static nlohmann::json serialize_link_aabbs(const Robot& robot,
    const double* tf, int n_links);                              // ~20行
```

原本 3 处重复的 interval/AABB 序列化 → 调用这些 helper。

### 7.2 ompl_adapter.cpp

- 魔数 `0.05`, `0.7` → 放入 OMPLAdapterConfig (在 header 中)
- switch 加 default case

### 7.3 diag_split_compare.cpp

移出常规编译: CMakeLists 中从 sbf_planner 移除, 改为独立可选 target:
```cmake
if(SBF_BUILD_DIAG)
    add_executable(diag_split_compare src/diag/diag_split_compare.cpp)
    target_link_libraries(diag_split_compare PRIVATE sbf)
endif()
```

### 7.4 确认删除的死代码

| 项目 | 文件 | 操作 |
|------|------|------|
| `FFBResult::path` | `include/sbf/ffb/ffb.h` | 删除字段 |
| `FFBResult::envelope_ms` 等 | `include/sbf/ffb/ffb.h` | 删除未赋值字段 |
| `geometric_mean_edge()` | `src/forest/coarsen.cpp` | 如未使用则删除 |

### 验收
- 编译通过, ctest 全部通过
- test_viz_exporter 通过

---

## Phase 8: 实验代码对齐

### 目标
清理实验文件, 删除开发期调试实验, 对齐论文参数。

### 8.1 删除非论文实验

| 文件 | 行数 | 理由 |
|------|------|------|
| `exp_box_connect.cpp` | 525 | 开发期调试, 非论文 |
| `exp_crit_timing.cpp` | 453 | 已被 exp1 替代 |
| `exp_rrt_path.cpp` | 536 | 开发期调试, 非论文 |

### 8.2 移动非实验文件

| 文件 | 从 | 到 |
|------|------|------|
| `test_ffb_marcucci.cpp` | experiments/ | test/ |
| `verify_collision.cpp` | experiments/ | tools/ (新目录) |

### 8.3 experiments/CMakeLists.txt 更新

```cmake
sbf_add_experiment(exp1_coverage       exp1_coverage.cpp)
sbf_add_experiment(exp2_e2e_planning   exp2_e2e_planning.cpp)
sbf_add_experiment(exp3_incremental    exp3_incremental.cpp)

# Drake-dependent
if(SBF_WITH_DRAKE)
    # ...test_drake_gcs targets...
endif()
```

### 验收
- 编译通过
- exp1, exp2, exp3 均正常运行

---

## Phase 9: 全量回归验证

### 目标
确认 v6 所有指标 ≥ v5。

### 9.1 运行全部测试

```bash
cd build && ctest --output-on-failure -j$(nproc)
```

### 9.2 运行性能基准

```bash
./test/bench_ffb
./test/bench_endpoint_iaabb
./test/bench_pipeline
```

### 9.3 运行实验

```bash
./experiments/exp1_coverage --quick
./experiments/exp2_e2e_planning --seeds 5
./experiments/exp3_incremental --seeds 3
```

### 9.4 对比 v5 基线

| 指标 | v5 基线 | v6 结果 | 容忍 |
|------|---------|---------|------|
| exp1 envelope volume | (基线值) | (v6值) | ±1% |
| exp2 planning time | (基线值) | (v6值) | ±5% |
| exp2 success rate | (基线值) | (v6值) | ≥ v5 |
| exp3 cache speedup | (基线值) | (v6值) | ±5% |
| bench_ffb throughput | (基线值) | (v6值) | ±3% |
| bench_pipeline | (基线值) | (v6值) | ±5% |

### 9.5 代码质量检查

```bash
# 最大文件行数
wc -l src/**/*.cpp | sort -n | tail -5  # 目标: 每个 < 800

# fprintf 残留
grep -rc 'fprintf' src/ | grep -v ':0$'  # 目标: 0 (除 snprintf)

# 编译警告
cmake --build . 2>&1 | grep -i 'warning'  # 目标: 0
```

### 验收
- 所有指标通过
- 代码质量目标达成
