# Step 1: 树均衡生长（Issue 3）— 多线程独立生长方案

## 问题描述

Multi-goal RRT 生长后，5 棵树的 box 分布极度不均衡：

| 树 | 种子 | box 数 | 占比 |
|----|------|--------|------|
| root0 (AS) | seed[0] | 4059 | 81.2% |
| root3 (LB) | seed[3] | 471 | 9.4% |
| root4 (RB) | seed[4] | 324 | 6.5% |
| root2 (CS) | seed[2] | 96 | 1.9% |
| root1 (TS) | seed[1] | 50 | 1.0% |

TS 和 CS 各自被隔离为小岛（17 和 46 boxes after coarsening），导致 query 时需要昂贵的 proxy RRT 跨越 C-space 连接最大岛，TS→CS 和 CS→LB 各耗 ~10s。

## 根因分析

文件：`src/forest/grower.cpp`，`grow_rrt()` 方法（约 L340-L445）

### 马太效应

当前采用单线程 monolithic 循环生长所有树：

```cpp
while (boxes_.size() < max_boxes && ...) {
    // 50% goal_bias → 均匀随机选目标树
    // 最近邻搜索 → AS 树有 4059 box，几乎总赢最近邻 → AS 获得更多扩展
    // 正反馈循环：大树越来越大，小树永远无法增长
}
```

**根本原因**：所有树共享同一个全局循环和全局 box 池，大树的面积优势让它在最近邻竞争中垄断扩展机会。

### 现有并行基础设施

代码已有完整的并行生长框架（`grow_parallel()` + `ThreadPool` + `grow_subtree()`），但 **multi-goal RRT 被显式禁用**：

```cpp
// grow() L698:
if (config_.n_threads > 1 && static_cast<int>(boxes_.size()) >= 2
    && !has_multi_goals_) {   // ← 禁用了 multi-goal
```

原因注释说"needs cross-tree goal bias"——但这恰恰是马太效应的根源。

## 修改方案：多线程独立生长 + 每树额度

### 核心思路

**彻底消除马太效应**：不再让所有树在同一循环中竞争，而是让每棵树独立生长、独立线程、独立额度。

1. 移除 `!has_multi_goals_` 限制，让 multi-goal RRT 使用 `grow_parallel()`
2. 每棵树分配独立额度：`per_tree_budget = max_boxes / n_trees`
3. 每个 worker 独立运行纯 RRT（无跨树 goal bias）
4. 增大 `max_consecutive_miss` 容忍度（紧凑空间的树需要更多尝试）

### 为什么不需要跨树 goal bias

在独立生长模式下：
- 每棵树从自己的 goal 种子点出发，按预算向 C-space 各方向扩展
- 预算均等 → 每棵树获得大致相同数量的 box
- 树之间的连通性由下游 coarsening + GCS 处理（Step 2/3）
- 跨树 goal bias 反而是马太效应的根源

### 具体修改

#### 文件：`src/forest/grower.cpp`

**修改 1**：`grow()` — 移除 multi-goal 并行限制（~L698）

```cpp
// 原代码：
if (config_.n_threads > 1 && static_cast<int>(boxes_.size()) >= 2
    && !has_multi_goals_) {

// 新代码：
if (config_.n_threads > 1 && static_cast<int>(boxes_.size()) >= 2) {
```

**修改 2**：`grow_parallel()` — multi-goal 独立额度（worker 创建循环）

```cpp
for (int i = 0; i < n_subtrees; ++i) {
    GrowerConfig worker_cfg = config_;

    // ── 新增：multi-goal 每树独立额度 ──
    if (has_multi_goals_) {
        worker_cfg.max_boxes = std::max(config_.max_boxes / n_subtrees, 50);
        // 紧凑空间的树可能 FFB 频繁失败，放宽容忍度
        worker_cfg.max_consecutive_miss =
            std::max(config_.max_consecutive_miss, 200);
    } else {
        worker_cfg.max_boxes = config_.max_boxes;
    }

    // ...（其余不变）
}
```

**修改 3**：`grow_parallel()` lambda — multi-goal 不传 shared_counter

```cpp
// multi-goal: 各树独立预算，不共享计数器
auto worker_counter = has_multi_goals_
    ? std::shared_ptr<std::atomic<int>>(nullptr)
    : shared_counter;

// lambda 中：
pwr.result = worker.grow_subtree(seed, rid, obs, n_obs, worker_counter);
```

**修改 4**：`grow_parallel()` — 合并后打印每树 box 数

```cpp
// 在合并完成后添加树大小统计
if (has_multi_goals_) {
    std::unordered_map<int, int> tree_sizes;
    for (const auto& b : boxes_) tree_sizes[b.root_id]++;
    fprintf(stderr, "[GRW] tree sizes:");
    for (auto& [rid, cnt] : tree_sizes)
        fprintf(stderr, " root%d=%d", rid, cnt);
    fprintf(stderr, "\n");
}
```

### 不修改的部分

- `select_roots()` 不变（仍按 multi_goals_ 创建 root boxes）
- `grow_rrt()` 内部逻辑不变（但 worker 内不设 multi_goals，走纯随机采样分支）
- `grow_subtree()` 不变
- `promote_all()` 不变
- `rrt_goal_bias = 0.5` 不变（worker 内不生效，因为 `has_multi_goals_=false`）
- LECT snapshot + transplant 逻辑不变

### 线程安全分析

| 资源 | 隔离方式 | 说明 |
|------|----------|------|
| `boxes_` | 每 worker 独立 | `grow_subtree()` 清空后独立操作 |
| LECT | 每 worker 深拷贝 `snapshot()` | 主 LECT 在 worker 完成后通过 `transplant_subtree()` 合并 |
| `shared_box_count_` | multi-goal 传 nullptr | 各树只看本地 `boxes_.size() < max_boxes` |
| `rng_` | 每 worker 独立种子 | `rng_seed + i * 12345 + 1` |
| obstacles | const 只读 | 指针传入，不修改 |

## 预期效果

| 指标 | 修改前 | 修改后预期 |
|------|--------|-----------|
| 树大小分布 | 4059/50/96/471/324 | ~1000/1000/1000/1000/1000 |
| 最小树 box 数 | 50 (TS) | >500 |
| 总 box 数 | 5000 | 5000（分布均匀） |
| 构建时间 | ~30s（单线程） | ~10s（5 线程并行） |
| 后续 query | TS/CS 隔离需 proxy | 各树覆盖充分 |

## 验证步骤

```bash
cd /home/tian/桌面/box_aabb/cpp/build
cmake --build . --target exp2_e2e_planning -j$(nproc)
./experiments/exp2_e2e_planning --seeds 1 2>&1 | grep -E "tree sizes|parallel done|timing"
```

检查：
1. `[GRW] tree sizes: root0=N root1=N ...` 各树差异 < 3x
2. `[GRW] parallel done:` 总 box 数接近 max_boxes
3. 构建时间应下降（并行加速）
4. 各树均有有效覆盖

## 风险与回退

- **风险 1**：某树所在 C-space 极度拥挤 → FFB 全部失败
  - **缓解**：`max_consecutive_miss=200` 给更多机会
  - **兜底**：该树少于 50 box，但不再拖累其他树

- **风险 2**：独立生长的树之间缺少重叠 → connectivity 差
  - **缓解**：预算足够大时（~1000/树），随机 RRT 自然向外扩展
  - **兜底**：Step 2 的 coarsening + GCS 处理跨树连通

- **回退**：如果需要恢复原行为，只需还原 `grow()` 中的 `!has_multi_goals_` 条件
