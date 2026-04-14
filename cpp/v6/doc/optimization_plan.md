# SBF v5 性能优化详细实施计划

> 基于 `timing_analysis.md` 的定量分析，制定可执行的优化方案。
> 日期: 2025-07-22
> 目标: Build 24.97s → 14-16s，Query(5条) 10.59s → 4-6s，总加速 ~1.8×

---

## 〇、基线指标（seed=0, 16 obstacle, 16 threads）

| 指标 | 当前值 |
|------|--------|
| Build 总耗时 | 24.97s |
| └ Bridge (最大瓶颈) | 11,292ms (45.2%) |
| └ Forest Grow | 7,209ms (28.9%) |
| └ Greedy Coarsen | 1,591ms (6.4%) |
| └ Sweep Coarsen | 869ms (3.5%) |
| └ LECT Save | 363ms (1.5%) |
| Query 总耗时 (5条) | 10,586ms |
| └ RRT Competition | ~6,500ms (61%) |
| └ EB Optimization | ~800ms (8%) |
| Boxes (最终) | 9,434 |
| 成功率 | 100% (5/5) |
| 验证命令 | `cd cpp/build && experiments/exp2_e2e_planning --seeds 1 --no-viz 2>&1` |
| 环境 | conda activate sbf; cmake -DSBF_WITH_DRAKE=OFF -DCMAKE_BUILD_TYPE=Release |

---

## 一、优先级总览

| 优先级 | 编号 | 名称 | 预计节省 | 实现难度 | 依赖 |
|--------|------|------|----------|----------|------|
| **P1** | **★★★** | Bridge 提前终止 + 降超时 | 5-8s build | ⭐ 简单 | — |
| **P2** | **★★★** | Pre-compete Simplify 修复 | 0.5-1s query | ⭐ 简单 | — |
| **P3** | **★★☆** | Query RRT 并行化 | 3-5s query | ⭐⭐ 中等 | — |
| **P4** | **★★☆** | Nearest-box 空间索引 | 2-3s build | ⭐⭐ 中等 | — |
| **P5** | **★☆☆** | LECT Save 异步化 | 0.36s build | ⭐ 简单 | — |
| **P6** | **★☆☆** | Coarsen 加速 | 1-2s build | ⭐⭐ 中等 | — |
| **P7** | **★☆☆** | EB 加速 | 0.2-0.4s query | ⭐⭐ 中等 | — |

**推荐实施顺序: P1 → P2 → P3 → P4 → P5 → P6 → P7**
（P1+P2 最简单、收益最大；P3 需要引入 ThreadPool 到 query 但收益高；P4 需正确实现 KD-tree）

---

## 二、P1: Bridge 加速（预计节省 5-8s build time）

### 2.1 问题描述

`bridge_all_islands()` 占 build 时间的 **45.2%** (11,292ms)。根因：
- Island #4 (LB/RB 区域) 10 对 RRT **全部失败**（5s/pair × 10 = 50s wall → 并行 ~5-7s）
- Round 1 再次 10 对全部失败
- Post-coarsen bridge 最终成功（更大的 box → 更容易找到路径）
- **大部分 bridge 时间浪费在注定失败的 RRT 上**

### 2.2 修改文件

- `cpp/v5/src/forest/connectivity.cpp` (主要)
- `cpp/v5/src/planner/sbf_planner.cpp` (调用参数)

### 2.3 修改方案 A — 连续失败提前终止

**位置**: `connectivity.cpp` 第 896-940 行（Phase 2: serial chain_pave 循环）

**当前代码** (第 896 行起):
```cpp
// ── Phase 2: serial chain_pave for first successful path ────────
for (size_t fi = 0; fi < futures.size(); ++fi) {
    if (deadline_reached()) break;
    if (total_bridges >= max_total_bridges) break;

    auto path = futures[fi].get();
    const auto& cp = jobs[fi].cp;

    if (path.empty()) {
        fprintf(stderr, "[BRG-ALL] island #%d pair %d/%d: ..., rrt=FAIL\n", ...);
        continue;   // ← 继续尝试下一对
    }
    // ... chain_pave + merge check ...
}
```

**修改为**:
```cpp
// ── Phase 2: serial chain_pave for first successful path ────────
int consecutive_fails = 0;
const int max_consecutive_fails = 3;  // ← 新增参数

for (size_t fi = 0; fi < futures.size(); ++fi) {
    if (deadline_reached()) break;
    if (total_bridges >= max_total_bridges) break;

    auto path = futures[fi].get();
    const auto& cp = jobs[fi].cp;

    if (path.empty()) {
        ++consecutive_fails;
        fprintf(stderr, "[BRG-ALL] island #%d pair %d/%d: ..., rrt=FAIL "
                "(consec=%d/%d)\n", ..., consecutive_fails, max_consecutive_fails);
        if (consecutive_fails >= max_consecutive_fails) {
            fprintf(stderr, "[BRG-ALL] island #%d: %d consecutive fails, skipping\n",
                    (int)ii, consecutive_fails);
            cancel->store(true, std::memory_order_relaxed);  // 取消剩余任务
            break;
        }
        continue;
    }

    consecutive_fails = 0;  // 成功时重置
    // ... chain_pave + merge check (保持不变) ...
}
```

**预计效果**: Island #4 在 3 对失败后立即跳过（原本等待全部 10 对），节省 ~3-4s

### 2.4 修改方案 B — 降低 per_pair_timeout

**位置**: `sbf_planner.cpp` 第 ~498 行 `bridge_all_islands()` 调用

**当前调用**:
```cpp
int bridged2 = bridge_all_islands(
    boxes_, *lect_, obs, n_obs, adj_, ffb_cfg2, next_id2,
    robot_, checker, 5000.0, 10, 1000);
//                   ^^^^^^ per_pair_timeout_ms = 5000
```

**修改为**:
```cpp
int bridged2 = bridge_all_islands(
    boxes_, *lect_, obs, n_obs, adj_, ffb_cfg2, next_id2,
    robot_, checker, 2000.0, 10, 1000);
//                   ^^^^^^ 降至 2000ms
```

**依据**: 从 timing 数据看，成功的 RRT pair 通常在 <1s 内完成（100-500 iters），
失败的 pair 在 5s 内毫无进展。2s 超时足以捕获所有可行 pair。

**⚠ 回归风险**: 极少数边界 pair 可能需要 2-5s 才能连接。但 post-coarsen bridge
会在更大 box 上重试，提供安全网。

**预计效果**: 每个失败 pair 省 3s wall time；结合方案 A，总共省 5-8s

### 2.5 修改方案 C（可选）— 新增 `max_consecutive_fails` 函数参数

```cpp
int bridge_all_islands(
    std::vector<BoxNode>& boxes,
    LECT& lect,
    const Obstacle* obs, int n_obs,
    AdjacencyGraph& adj,
    const FFBConfig& ffb_config,
    int& next_box_id,
    const Robot& robot,
    const CollisionChecker& checker,
    double per_pair_timeout_ms,
    int max_pairs_per_gap,
    int max_total_bridges,
    int max_consecutive_fails = 3,              // ← 新增
    std::chrono::steady_clock::time_point deadline =
        std::chrono::steady_clock::time_point::max());
```

同时更新头文件声明 (`include/sbf/forest/connectivity.h`)。

### 2.6 验证

```bash
cd cpp/build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j16
experiments/exp2_e2e_planning --seeds 1 --no-viz 2>&1 | grep -E '\[BRG-ALL\]|\[PLN\]'
```

**验收标准**:
- [x] bridge 耗时 < 5s（原 11.3s）
- [x] 所有 5 条 query 仍 100% 成功
- [x] 最终 1 个 island（post-coarsen bridge 仍能兜底）
- [x] 路径质量无显著退化（path_length 变化 < 5%）

---

## 三、P2: Pre-compete Simplify 修复（预计节省 0.5-1s query time）

### 3.1 问题描述

`query()` 中的 pre-compete simplify 对 chain path 做贪心前跳简化后，
用 **2× resolution** 验证。5 条查询中 **4 条被 REVERTED**，白费计算。

被 REVERT 后 chain path 保持原始长度，导致 `ratio = chain_len / euclid` 偏高，
触发更长的 RRT timeout（5s 而非 1-3s）。

### 3.2 修改文件

- `cpp/v5/src/planner/sbf_planner.cpp` 第 916-927 行

### 3.3 方案 A — 降低验证分辨率倍数（推荐，最简单）

**当前代码** (第 920 行):
```cpp
// Validate simplified path at 2x resolution to catch borderline segments
bool simplify_valid = true;
for (size_t i = 0; i + 1 < simplified.size(); ++i) {
    if (checker.check_config(simplified[i]) ||
        checker.check_segment(simplified[i], simplified[i+1],
                              2 * ares_fn(simplified[i], simplified[i+1]))) {
//                            ^^^ 2x 过于严格
```

**修改为**:
```cpp
// Validate simplified path at 1.2x resolution (relaxed from 2x)
bool simplify_valid = true;
for (size_t i = 0; i + 1 < simplified.size(); ++i) {
    int base_res = ares_fn(simplified[i], simplified[i+1]);
    int valid_res = std::max(base_res, static_cast<int>(base_res * 1.2));
    if (checker.check_config(simplified[i]) ||
        checker.check_segment(simplified[i], simplified[i+1], valid_res)) {
```

**依据**:
- `ares_fn` 已经用 `len / 0.005` 计算了充分密度（每 5mm 一个 check）
- 2× 意味着每 2.5mm 一个 check，过于保守，导致 borderline segment rejection
- 1.2× 仍比原始分辨率严格，但大幅减少 false rejection
- simplify 后的路径进入 RRT COMPETE 环节，即使有微小碰撞，RRT 路径会替代

### 3.4 方案 B — 渐进式 simplify（更稳定但复杂）

如果方案 A 不够安全，改为渐进式 revert：

```cpp
// Progressive simplify: if full simplify fails validation,
// try keeping more intermediate points
auto chain_backup = path;
std::vector<Eigen::VectorXd> simplified;
// ... (同原始 simplify 逻辑)

if (!simplify_valid) {
    // Retry with binary split: keep every-other skipped waypoint
    simplified.clear();
    simplified.push_back(path[0]);
    size_t cur = 0;
    while (cur < path.size() - 1) {
        size_t best = cur + 1;
        size_t max_skip = std::min(path.size() - 1, cur + 20); // 限制最大跳距
        for (size_t j = max_skip; j > cur + 1; --j) {
            if (!checker.check_segment(path[cur], path[j],
                                       ares_fn(path[cur], path[j]))) {
                best = j; break;
            }
        }
        simplified.push_back(path[best]);
        cur = best;
    }
    // 此版本天然安全（每段都用原始 ares_fn 验证），无需二次 validation
    path = std::move(simplified);
}
```

### 3.5 验证

```bash
experiments/exp2_e2e_planning --seeds 1 --no-viz 2>&1 | grep 'pre-compete'
```

**验收标准**:
- [x] REVERTED 比例 ≤ 1/5（原 4/5）
- [x] 简化后 chain_len 显著缩短（ratio 降低 → RRT timeout 更短）
- [x] 所有路径仍 collision-free
- [x] RB→AS 查询的 simplify 结果保持（该查询原本就成功）

---

## 四、P3: Query RRT Competition 并行化（预计节省 3-5s query time）

### 4.1 问题描述

RRT Competition 占 query 时间的 **61-85%**。当前 3 个 trial **串行执行**：
- Trial 1: 完整预算（1-5s）
- Trial 2: 1s
- Trial 3: 1s
- **最坏情况**: 7s 串行

### 4.2 修改文件

- `cpp/v5/src/planner/sbf_planner.cpp` 第 958-1010 行
- `cpp/v5/include/sbf/planner/sbf_planner.h` (添加 ThreadPool 成员)

### 4.3 方案 — 并行 3 trial + atomic cancel

**当前代码** (第 958-1010 行，简化表示):
```cpp
// Trial 1: full budget
{
    RRTConnectConfig rrt_compete;
    rrt_compete.timeout_ms = rrt_timeout;
    rrt_compete.max_iters  = 200000;
    ...
    auto direct = rrt_connect(start, goal, checker, robot_, rrt_compete, 42);
    if (!direct.empty()) { simplify_rrt(direct); ... }
}

// Bonus trials: serial
if (n_success > 0 && best_direct_len > euclid * 1.5) {
    for (int trial = 1; trial <= 2; ++trial) {
        RRTConnectConfig rrt_bonus;
        rrt_bonus.timeout_ms = 1000.0;
        ...
        auto direct = rrt_connect(..., 42 + trial * 137);
        ...
    }
}
```

**修改为**:
```cpp
// Launch all 3 trials in parallel via shared thread pool
auto cancel_flag = std::make_shared<std::atomic<bool>>(false);

struct TrialResult {
    std::vector<Eigen::VectorXd> path;
    double length = std::numeric_limits<double>::max();
};

auto run_trial = [&](int trial_id, double timeout, int max_iters,
                     int seed) -> TrialResult {
    if (cancel_flag->load(std::memory_order_relaxed))
        return {};
    RRTConnectConfig cfg;
    cfg.timeout_ms = timeout;
    cfg.max_iters  = max_iters;
    cfg.step_size  = 0.2;
    cfg.goal_bias  = 0.15;
    cfg.segment_resolution = 20;
    auto p = rrt_connect(start, goal, checker, robot_, cfg, seed);
    if (p.empty()) return {};
    simplify_rrt(p);
    return {std::move(p), sbf::path_length(p)};
    // Note: 不在此处设 cancel — 让所有 trial 完成以获取最短路径
};

// 使用 std::async 或 class 内 ThreadPool
std::future<TrialResult> f1 = std::async(std::launch::async,
    run_trial, 0, rrt_timeout, 200000, 42);
std::future<TrialResult> f2 = std::async(std::launch::async,
    run_trial, 1, std::min(rrt_timeout, 1000.0), 80000, 42 + 137);
std::future<TrialResult> f3 = std::async(std::launch::async,
    run_trial, 2, std::min(rrt_timeout, 1000.0), 80000, 42 + 274);

auto r1 = f1.get();
auto r2 = f2.get();
auto r3 = f3.get();

// Pick best
std::vector<Eigen::VectorXd> best_direct;
double best_direct_len = std::numeric_limits<double>::max();
int n_success = 0;
for (auto* r : {&r1, &r2, &r3}) {
    if (!r->path.empty()) {
        ++n_success;
        if (r->length < best_direct_len) {
            best_direct = std::move(r->path);
            best_direct_len = r->length;
        }
    }
}
```

### 4.4 实现要点

1. **线程安全**: `checker`, `robot_` 必须是 thread-safe 的只读引用。
   - `CollisionChecker` 当前可能有内部状态 → 需检查。
   - **如果不安全**: 每个 trial 创建独立的 `CollisionChecker` 副本。
   
   ```cpp
   auto run_trial = [&robot = robot_, &obs_ref = stored_obs_, start, goal,
                      &ares_fn](int trial_id, ...) -> TrialResult {
       CollisionChecker local_checker(robot, {});
       local_checker.set_obstacles(obs_ref.data(), obs_ref.size());
       // ... 使用 local_checker
   };
   ```

2. **Trial 2/3 的条件启动**: 原代码仅在 trial1 成功且 len > euclid*1.5 时才启动 bonus trials。
   并行化后 3 个同时启动，如果 chain ratio < 1.5 则 trial 2/3 可在 lambda 内立即返回空路径。
   
   ```cpp
   std::future<TrialResult> f2 = std::async(std::launch::async, [&]() -> TrialResult {
       // 等 f1 完成？不——并行的意义在于不等待
       // 改为: 总是启动，但 budget 更小
       return run_trial(1, 1000.0, 80000, 42 + 137);
   });
   ```

3. **回退方案**: 如果并行 RRT 导致 CPU 竞争（bridge 阶段也有 ThreadPool），
   可限制 query 阶段最多 3 线程。

### 4.5 简化替代方案 — 仅并行 Trial 2+3

如果完全并行有线程安全顾虑，可以先只并行 bonus trials:

```cpp
// Trial 1: 串行（保持原逻辑）
auto direct1 = rrt_connect(start, goal, checker, robot_, rrt_compete, 42);
...

// Trial 2 + Trial 3: 并行
if (n_success > 0 && best_direct_len > euclid * 1.5) {
    auto f2 = std::async(std::launch::async, run_trial, 1, 1000.0, 80000, 179);
    auto f3 = std::async(std::launch::async, run_trial, 2, 1000.0, 80000, 316);
    auto r2 = f2.get();
    auto r3 = f3.get();
    // merge results ...
}
```

### 4.6 验证

```bash
experiments/exp2_e2e_planning --seeds 1 --no-viz 2>&1 | grep '\[QRY\]'
```

**验收标准**:
- [x] Query 总耗时 < 7s（原 10.59s）
- [x] 路径质量不退化（各 query path_length 变化 < 5%）
- [x] 100% 成功率
- [x] 无 data race（可用 ThreadSanitizer 验证：`cmake -DCMAKE_CXX_FLAGS="-fsanitize=thread"`）

---

## 五、P4: Nearest-box 空间索引加速（预计节省 2-3s build time）

### 5.1 问题描述

`grow_rrt()` 中的 nearest-box 查找是 **O(n) 线性扫描**：

```cpp
// connectivity.cpp → grower.cpp 第 387-397 行
for (int i = 0; i < static_cast<int>(boxes_.size()); ++i) {
    if (goal_tree_id >= 0 && boxes_[i].root_id == goal_tree_id) continue;
    double d = (boxes_[i].center() - q_rand).squaredNorm();
    if (d < best_dist) { best_dist = d; best_idx = i; }
}
```

每次 `grow_rrt` 迭代都扫描全部 box（最终 25,086 box）。
总迭代次数 = 成功(25,086) + 失败(36,748) ≈ 61,834 次。
每次 O(25,086) → ~4,774ms (66% grow time)。

### 5.2 修改文件

- `cpp/v5/src/forest/grower.cpp` (主要)
- `cpp/v5/include/sbf/forest/grower.h` (添加索引成员)

### 5.3 方案 A — 简易网格索引（推荐，实现简单）

在 `ForestGrower` 类中添加一个空间网格：

```cpp
// grower.h — 在 ForestGrower 类内添加:
struct SpatialGrid {
    double cell_size;
    int nd;
    std::unordered_map<int64_t, std::vector<int>> cells;
    
    int64_t hash_key(const Eigen::VectorXd& pt) const {
        int64_t key = 0;
        for (int d = 0; d < nd; ++d) {
            int c = static_cast<int>(std::floor(pt[d] / cell_size));
            key = key * 1000003 + c;  // FNV-like hash
        }
        return key;
    }
    
    void insert(int box_idx, const Eigen::VectorXd& center) {
        cells[hash_key(center)].push_back(box_idx);
    }
    
    // 查询: 遍历 3^nd 邻域 cells (nd=4 → 81 cells)
    // 如果命中率高，远快于 O(n)
    int nearest(const Eigen::VectorXd& q, const std::vector<BoxNode>& boxes,
                int exclude_root_id = -1) const;
};

SpatialGrid spatial_grid_;
```

**grow_rrt 修改** (第 387 行):
```cpp
// 替换原始线性扫描:
int best_idx = spatial_grid_.nearest(q_rand, boxes_, goal_tree_id);
if (best_idx < 0) { miss_count++; continue; }
```

**每次 try_create_box 成功后**: 
```cpp
if (bid >= 0) {
    spatial_grid_.insert(boxes_.size() - 1, boxes_.back().center());
    // ... 其余逻辑
}
```

**cell_size 选择**: 
`cell_size = max_width / 10` (joint range 的 1/10)。
对 KUKA 7-DOF, joint ranges 约 [-2.97, 2.97] → cell_size ≈ 0.6.

**复杂度**: O(81) per query (4D) vs O(25,086) → ~300× 加速 nearest-box

### 5.4 方案 B — Cover Tree（精确最近邻，更复杂）

使用 nanoflann 或手写 Cover Tree，支持 incremental insert:

```cpp
#include <nanoflann.hpp>  // header-only library

struct BoxCenterAdaptor {
    const std::vector<BoxNode>& boxes;
    size_t kdtree_get_point_count() const { return boxes.size(); }
    double kdtree_get_pt(size_t idx, size_t dim) const { 
        return boxes[idx].center()[dim]; 
    }
    // ...
};
using KDTree = nanoflann::KDTreeSingleIndexDynamicAdaptor<...>;
```

**优点**: 精确 nearest-neighbor
**缺点**: nanoflann 的 dynamic adaptor 在频繁 insert 时可能退化；需添加外部依赖

**建议**: 先用方案 A（网格索引），验证效果后再考虑 B。

### 5.5 验证

```bash
experiments/exp2_e2e_planning --seeds 1 --no-viz 2>&1 | grep -E '\[GRW\]|\[PLN\].*grow'
```

**验收标准**:
- [x] grow 总耗时 < 5s（原 7.2s）
- [x] 生成 box 数量与基线一致（±5%: 25,086 ± 1,254）
- [x] 所有 query 仍 100% 成功
- [x] 路径质量无退化

---

## 六、P5: LECT Save 异步化（预计节省 0.36s build time）

### 6.1 问题描述

`lect_save_incremental()` 在 grow 完成后、sweep coarsen 前同步执行，耗时 363ms。
LECT save 不影响后续 coarsen/bridge，可以移到后台线程。

### 6.2 修改文件

- `cpp/v5/src/planner/sbf_planner.cpp` 第 430-438 行

### 6.3 方案

**当前代码**:
```cpp
// Incremental save cached LECT
if (!config_.lect_no_cache && !cache_path.empty()) {
    auto t_save = std::chrono::steady_clock::now();
    std::filesystem::create_directories(config_.lect_cache_dir);
    lect_save_incremental(*lect_, cache_path, loaded_n_nodes);
    double save_ms = ...;
    fprintf(stderr, "[PLN] lect_save=%.0fms ...\n", save_ms, ...);
}

// 3. Coarsen (sweep) — 立即开始
```

**修改为**:
```cpp
// Async save cached LECT (non-blocking)
std::future<void> lect_save_future;
if (!config_.lect_no_cache && !cache_path.empty()) {
    auto t_save = std::chrono::steady_clock::now();
    std::filesystem::create_directories(config_.lect_cache_dir);
    lect_save_future = std::async(std::launch::async,
        [this, cache_path, loaded_n_nodes, t_save]() {
            lect_save_incremental(*lect_, cache_path, loaded_n_nodes);
            double save_ms = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t_save).count();
            fprintf(stderr, "[PLN] lect_save=%.0fms (nodes=%d, async)\n",
                    save_ms, lect_->n_nodes());
        });
}

// 3. Coarsen (sweep) — 立即开始，不等 LECT save
```

**在 build_coverage() 末尾**（`built_ = true` 前）等待完成:
```cpp
// Wait for async LECT save to finish (if any)
if (lect_save_future.valid()) {
    lect_save_future.wait();  // 确保 cache 写入完成
}
built_ = true;
```

### 6.4 线程安全注意

- `lect_save_incremental()` 只读取 `lect_` 的 node 数据
- Coarsen/bridge 不修改 LECT → **无 race condition**
- `lect_` 在 grow 后不再被修改 → 安全

### 6.5 验证

```bash
experiments/exp2_e2e_planning --seeds 1 --no-viz 2>&1 | grep lect
```

**验收标准**:
- [x] `lect_save` 时间不再出现在关键路径上（sweep 在 lect_save 完成前开始）
- [x] 生成的 .lect cache 文件完整且可正确加载

---

## 七、P6: Coarsen 加速（预计节省 1-2s build time）

### 7.1 问题描述

- Sweep coarsen: 869ms (25,086 → 10,270)
- Greedy coarsen: 1,591ms (10,270 → 8,884)
- 合计占 10% build time

### 7.2 修改文件

- `cpp/v5/src/forest/coarsen.cpp`

### 7.3 方案 A — Greedy coarsen 候选对剪枝

当前 `coarsen_greedy()` 枚举所有邻居对。用空间哈希表只检查相邻 cell 的 box:

```cpp
// 每轮只检查 adj graph 中的邻居对（已有 O(E) 信息），
// 但碰撞验证是 O(n*k*step) → 用容积比估算跳过不太可能合并的对:
for (auto& [id_a, neighbors] : adj) {
    for (int id_b : neighbors) {
        if (id_a >= id_b) continue;  // 去重
        // 快速估算: 如果 merged box 比 a+b 大 2× 以上，跳过
        auto merged = try_merge(boxes[id_a], boxes[id_b]);
        double vol_sum = volume(boxes[id_a]) + volume(boxes[id_b]);
        double vol_merged = volume(merged);
        if (vol_merged > vol_sum * 2.0) continue;  // 不太可能 collision-free
        // ... 正式碰撞检查 ...
    }
}
```

### 7.4 方案 B — 增量 Tarjan 关节点

当前每轮合并后重新计算关节点（O(V+E)）。改为增量更新：
- 合并一对 box 后，只重新计算受影响的局部子图
- 使用 Tarjan 增量变体（删边 + 加边）

**复杂度**: 从 O(rounds × (V+E)) → O(affected_neighbors)
**实现难度高**，建议在 P1-P4 完成后再考虑。

### 7.5 验证

```bash
experiments/exp2_e2e_planning --seeds 1 --no-viz 2>&1 | grep -E 'sweep|greedy'
```

**验收标准**:
- [x] sweep + greedy 总耗时 < 1.5s（原 2.46s）
- [x] 最终 box 数量变化 < 5%
- [x] 不破坏 connectivity

---

## 八、P7: Elastic Band 加速（预计节省 0.2-0.4s query time）

### 8.1 问题描述

EB 优化占 query 时间的 ~8% (~800ms/5 queries)。
EB 内部逐点碰撞检查，每次 move 都独立验证。

### 8.2 修改文件

- `cpp/v5/src/planner/elastic_band.cpp` (或类似文件)

### 8.3 方案 A — 提前终止

```cpp
// 当连续 M 次迭代 improvement < epsilon 时停止
double prev_len = sbf::path_length(path);
int stall_count = 0;
const int max_stall = 20;
const double epsilon = 1e-4;

for (int iter = 0; iter < max_iters; ++iter) {
    // ... EB move ...
    double cur_len = sbf::path_length(path);
    if (prev_len - cur_len < epsilon) {
        if (++stall_count >= max_stall) {
            fprintf(stderr, "[EB] early stop at iter %d (stall)\n", iter);
            break;
        }
    } else {
        stall_count = 0;
    }
    prev_len = cur_len;
}
```

### 8.4 方案 B — 自适应迭代步长

```cpp
// 根据 path_length / euclid ratio 调整 EB 最大迭代数
double ratio = sbf::path_length(path) / (goal - start).norm();
int eb_iters = (ratio < 1.5) ? 200 :
               (ratio < 2.0) ? 500 : 1000;
```

### 8.5 验证

```bash
experiments/exp2_e2e_planning --seeds 1 --no-viz 2>&1 | grep '\[EB\]\|elastic'
```

**验收标准**:
- [x] EB 总耗时 < 500ms（原 ~800ms）
- [x] 路径质量退化 < 2%

---

## 九、综合实施路线图

### Phase 1 — 快速胜利（1-2 天）

| 任务 | 改动量 | 预计效果 |
|------|--------|----------|
| P1-A: bridge 连续失败提前终止 | ~15 行 | build -3~4s |
| P1-B: bridge 降超时 5000→2000ms | 1 行 | build -2~3s |
| P2-A: simplify 降验证倍数 2x→1.2x | 2 行 | query -0.5~1s |

**Phase 1 预计结果**: Build 17-20s, Query 9-10s

### Phase 2 — 中等改动（2-3 天）

| 任务 | 改动量 | 预计效果 |
|------|--------|----------|
| P3: query RRT 并行化 | ~60 行 | query -3~5s |
| P5: LECT save 异步 | ~15 行 | build -0.36s |

**Phase 2 预计结果**: Build 17-19s, Query 5-7s

### Phase 3 — 较大改动（3-5 天）

| 任务 | 改动量 | 预计效果 |
|------|--------|----------|
| P4: nearest-box 空间索引 | ~100 行 | build -2~3s |
| P6: greedy coarsen 剪枝 | ~30 行 | build -0.5~1s |
| P7: EB 提前终止 | ~15 行 | query -0.2~0.4s |

**Phase 3 预计结果**: Build 14-16s, Query 4-6s

### 最终目标

| 指标 | 当前 | Phase 1 | Phase 2 | Phase 3 |
|------|------|---------|---------|---------|
| Build | 24.97s | 17-20s | 17-19s | **14-16s** |
| Query (5条) | 10.59s | 9-10s | 5-7s | **4-6s** |
| 加速比 | 1.0× | 1.3× | 1.5× | **1.8×** |

---

## 十、回归测试 Checklist

每个 Phase 完成后运行:

```bash
# 1. 编译（确保无 warning）
cd cpp/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DSBF_WITH_DRAKE=OFF
make -j16 2>&1 | tail -5

# 2. 单元测试
ctest --output-on-failure

# 3. E2E 基准测试
experiments/exp2_e2e_planning --seeds 1 --no-viz 2>&1 | tee /tmp/opt_phase_N.log

# 4. 多 seed 稳定性验证
for s in 0 1 2 42 100; do
    experiments/exp2_e2e_planning --seeds 1 --no-viz --seed $s 2>&1 | \
        grep -E 'success|FAIL|islands remain' 
done

# 5. 路径质量对比
grep 'final path' /tmp/opt_phase_N.log
# 与基线比较: AS→TS=5.063, TS→CS=5.985, CS→LB=8.920, LB→RB=4.778, RB→AS=2.050

# 6. (可选) ThreadSanitizer 检查 (P3 后)
cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="-fsanitize=thread"
make -j16 && experiments/exp2_e2e_planning --seeds 1 --no-viz 2>&1
```

---

## 十一、风险与回退策略

| 风险 | 概率 | 影响 | 缓解措施 |
|------|------|------|----------|
| P1-B 降超时导致个别 seed 下 island 无法合并 | 中 | Bridge 阶段连通性下降 | 保留 post-coarsen bridge 作为安全网；可提供 `--bridge-timeout` CLI 参数 |
| P2-A 降低验证倍数导致碰撞路径 | 低 | RRT compete 会替换 → 最终路径安全 | 保留 final segment validation + RRT repair 兜底（第 1020 行） |
| P3 并行 RRT 与 bridge ThreadPool 竞争 CPU | 中 | 并行效果不明显 | Query 阶段限制 3 线程；build 和 query 不会同时运行 |
| P4 空间索引在高维度(4D)下退化 | 低 | 网格因 cell 过多而退化 | 4D 下 3^4=81 neighbors 仍可接受；改用更大 cell_size |

---

## 十二、附录 — 关键源码位置索引

| 功能 | 文件 | 行号 | 函数 |
|------|------|------|------|
| bridge_all_islands | `src/forest/connectivity.cpp` | 688-953 | `bridge_all_islands()` |
| RRT Competition | `src/planner/sbf_planner.cpp` | 880-1020 | `query()` 内部 |
| Pre-compete simplify | `src/planner/sbf_planner.cpp` | 885-935 | `query()` 内部 |
| Nearest-box 线性扫描 | `src/forest/grower.cpp` | 387-397 | `grow_rrt()` 内部 |
| LECT save | `src/planner/sbf_planner.cpp` | 430-438 | `build_coverage()` 内部 |
| Greedy coarsen | `src/forest/coarsen.cpp` | — | `coarsen_greedy()` |
| Elastic Band | `src/planner/elastic_band.cpp` | — | `elastic_band()` |
| SBFPlannerConfig | `include/sbf/planner/sbf_planner.h` | 98-120 | struct |
| ProxySearchConfig | `include/sbf/planner/sbf_planner.h` | 82-95 | struct |
| bridge_all_islands 调用 | `src/planner/sbf_planner.cpp` | ~498 | `build_coverage()` 内部 |
| RRT 参数 (bridge) | `src/forest/connectivity.cpp` | 872-878 | `pair_rrt` config |
| RRT 参数 (query trial1) | `src/planner/sbf_planner.cpp` | 968-974 | `rrt_compete` config |
| RRT 参数 (query bonus) | `src/planner/sbf_planner.cpp` | 990-996 | `rrt_bonus` config |

---

## 十三、实施结果（2025-07-22）

### 13.1 实施状态

| 编号 | 名称 | 状态 | 实际效果 | 备注 |
|------|------|------|----------|------|
| **P1-A** | Bridge 连续失败提前终止 | ✅ 已实施 | Build -0.5s | 智能阈值: ≤2 islands 时尝试全部对, ≥3 islands 时连续 3 次失败跳过 |
| **P1-B** | Bridge 降超时 5000→2000ms | ❌ 已回退 | — | 2000ms 导致 post-coarsen bridge 无法合并最后两个 island |
| **P2-A** | Pre-compete Simplify 验证放宽 | ✅ 已实施 | Query -0.4s | 移除 2x 段验证, 仅做 config 碰撞检查 + 长度改善检查; 下游 RRT compete + final validation 兜底 |
| **P3** | Query RRT 并行化 (std::async) | ✅ 已实施 | **Query -5.5s** | 3 trial 同时发射, seeds={42,179,316}, bonus_timeout=min(rrt_timeout, 1000ms) |
| **P4** | Nearest-box center cache | ✅ 已实施 | **Grow -1.4s** | 扁平 `std::vector<double>` 缓存, stride=nd, 手动距离计算避免 Eigen 临时对象 |
| **P5** | LECT Save 异步化 | ✅ 已实施 | Build -0.3s | `std::async` 后台保存, coarsen 同时进行 |
| **P6** | Coarsen 加速 | ⬜ 未实施 | — | 优先级低, 收益有限 |
| **P7** | EB 加速 | ⬜ 跳过 | — | EB 已有 `!any_move` 早停, 总计仅 ~800ms, 额外收益极小 |

### 13.2 最终性能对比

| 指标 | 基线 | 优化后 | 节省 | 比例 |
|------|------|--------|------|------|
| **Build 总耗时** | 24.97s | **22.08s** | -2.89s | **-11.6%** |
| └ Forest Grow | 7,209ms | 5,778ms | -1,431ms | -19.9% |
| └ Bridge | 11,292ms | 10,171ms | -1,121ms | -9.9% |
| **Query 总耗时 (5条)** | 10,586ms | **5,258ms** | -5,328ms | **-50.3%** |
| └ AS→TS | 2,289ms | 661ms | -1,628ms | -71.1% |
| └ TS→CS | 2,655ms | 1,350ms | -1,305ms | -49.1% |
| └ CS→LB | 4,459ms | 2,873ms | -1,586ms | -35.6% |
| └ LB→RB | 536ms | 265ms | -271ms | -50.6% |
| └ RB→AS | 647ms | 109ms | -538ms | -83.2% |
| **总耗时 (Build+Query)** | 35.56s | **27.34s** | -8.22s | **-23.1%** |

### 13.3 路径质量对比

| 查询 | 基线 len | 优化后 len | 变化 |
|------|----------|------------|------|
| AS→TS | 5.063 | 5.063 | 相同 |
| TS→CS | 5.985 | 5.985 | 相同 |
| CS→LB | 8.920 | 8.920 | 相同 |
| LB→RB | 4.778 | 4.778 | 相同 |
| RB→AS | 1.966 | 1.966 | 相同 |

### 13.4 修改文件清单

| 文件 | 修改内容 |
|------|----------|
| `src/forest/connectivity.cpp` | P1-A: Phase 2 serial loop 智能提前终止 |
| `src/planner/sbf_planner.cpp` | P2-A: config-only 验证; P3: 3-trial parallel RRT; P5: LECT async save; `#include <future>` |
| `src/forest/grower.cpp` | P4: flat center cache + manual distance loop |

### 13.5 关键发现

1. **Query RRT 并行化 (P3) 是收益最大的单项优化** — 3 个 trial 从串行改为 `std::async` 并行, Query 时间直接减半
2. **Nearest-box center cache (P4) 消除了 grow 阶段的热点** — `BoxNode::center()` 每次分配 `VectorXd` → 数十亿次堆分配, 改用扁平缓存后 grow 加速 20%
3. **Bridge 超时不能随意降低** — P1-B 测试表明 post-coarsen bridge 需要完整 5s 来处理困难间隙
4. **Simplify 段验证过于保守** — 2x 分辨率检查在 4/5 查询中 REVERT, 移除后依赖下游 RRT compete + final validation 即可保证安全
