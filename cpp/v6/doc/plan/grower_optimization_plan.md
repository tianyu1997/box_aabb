# Grower 架构优化详细计划（v6）

> 制定日期：2026-04-23
> 目标：在不重写 `ForestGrower` 的前提下，通过四个 Phase 把 FFB collide_ms / envelope_ms / 单端点并行吞吐提升一档，并让采样自动倾向未探索区域、并行 worker 严格不重叠。

---

## 0. TL;DR

| Phase | 目标 | 主要改动 | 预期收益 |
|---|---|---|---|
| **A** | FFB collide-verified 缓存（free **+** collide） | LECT 加 `obs_generation_` + `collide_verified_gen_[i]` + `collide_state_[i]∈{unknown,free,collide}`；envelope 变化（compute/expand/promote_union/transplant）→ 失效 | ffb_collide_ms ↓ 30–60% |
| **B** | 分层 max_depth schedule | `GrowerConfig.ffb_depth_stages` 默认空（保持 baseline），上层显式开启 | envelope_ms ↓ 15–40%（开启时） |
| **C-1** | 无 root-seed 时自动几何分区 + 自动桥接 | `partition_for_seeds` 合成 N 子域，每 worker 独占非重叠 LECT 子树；post-merge 自动跑 endpoint bridge | 单端点对场景 ≈ 0.4–0.8 × N_threads |
| **C-2** | n_threads > n_subtrees 时二次切分 | 对最大 subtree `expand_leaf` 强制再切，分给空闲线程；保证子域两两 disjoint | 5 root + 8 thread 比 5 root + 5 thread 快 ≥1.3× |
| **U** | 采样偏向未探索区域 | LECT subtree free-volume 跟踪 + 加权随机 leaf 采样（Wavefront/RRT 共用） | 同 box budget vcov ↑ 5–15% |
| **D** | Quick wins | 去重 seed-collision、复用 `CollisionChecker`、`grow_rrt` K-subsample、`promote_all` worklist | 各 5–15% 局部优化 |

落地顺序：**A → U → B → C-1 → C-2 → D → 回归**。

**用户决定记录**
1. Phase A：同时缓存 collide=true，但 envelope 变化时失效。
2. Phase B：默认 schedule 留空。
3. Phase C-1：grower 内自动桥接；并行 worker 严格非重叠子树。
4. 新需求 U：采样偏向未探索区域。

---

## 1. 现状速记（research 结论）

**三条 grow 路径**
- `grow()` 入口分发：
  - `connect_mode + n_threads>1 + multi_goals` → `grow_coordinated`（master 单线程，worker 池只跑 FFB）
  - `n_threads>1 + boxes>=2` → `grow_parallel`（`partition_for_seeds` 切独立 leaf，每 worker `set_domain_root`，`transplant_domain` 合并）
  - 其他 → serial `grow_rrt`/`grow_wavefront` + `promote_all`

**FFB 主循环**（[cpp/v6/src/ffb/ffb.cpp](../../src/ffb/ffb.cpp)）
- step 1 进入时 `CollisionChecker.check_config(seed)`（重复 3 次：`try_create_box` + 此处 + coordinated master pre-filter）
- step 3 envelope：`!has_data` → `compute_envelope`
- step 4 collision：**无条件** `collides_scene(current, obs, n_obs[, grid])`
- 没有 obstacle-aware 的 "已验证 free / collide" 缓存

**LECT 已有能力**
- `partition_for_seeds(seeds, max_extra_splits=256)` → 切到每 seed 独立 leaf
- `transplant_domain(worker, R_i, id_map, node_remap)` → 把 worker 子树搬回 master
- `try_promote_envelope_union(p, L, R, obs)` → children union 写回 master envelope cache
- `is_descendant_of`、`is_point_occupied`(O(depth))、`subtree_occ(i)`(子树已 occupy 的 leaf 数)

**已知热点**
- envelope_ms（首次 FK + LinkIAABB compose）+ collide_ms 占 FFB 总时绝大头
- 浅层节点反复被多次 FFB 走过，每次都 collide 一遍
- coordinated 路径 master 还做 O(n_boxes) `box.contains(seed)` pre-filter
- 采样目前 `sample_random` + boundary bias，**没有"远离已 occupy 区域"的偏置**

---

## 2. Phase A — FFB collide-verified 缓存（free + collide 双向）

### 2.1 数据结构（LECT）
```cpp
// include/sbf/lect/lect.h  (private)
uint32_t obs_generation_ = 1;                 // bump_obs_generation() 后 +1
std::vector<uint8_t>  collide_state_;         // [capacity_]: 0=unknown, 1=free, 2=collide
std::vector<uint32_t> collide_verified_gen_;  // [capacity_]: 与 obs_generation_ 比对

// public API
void     bump_obs_generation()             { ++obs_generation_; }
uint32_t obs_generation() const            { return obs_generation_; }
inline bool collide_cache_hit_free(int i) const {
    return collide_state_[i] == 1 && collide_verified_gen_[i] == obs_generation_;
}
inline bool collide_cache_hit_collide(int i) const {
    return collide_state_[i] == 2 && collide_verified_gen_[i] == obs_generation_;
}
inline void mark_collide_free(int i) {
    collide_state_[i] = 1; collide_verified_gen_[i] = obs_generation_;
}
inline void mark_collide_hit(int i) {
    collide_state_[i] = 2; collide_verified_gen_[i] = obs_generation_;
}
inline void invalidate_collide(int i) { collide_state_[i] = 0; }
```

### 2.2 失效点（必须！）
| 触发 | 操作 |
|---|---|
| `compute_envelope(i, ...)` 末尾 | `invalidate_collide(i)`（envelope 变了） |
| `expand_leaf(i, ...)` 末尾 | `invalidate_collide(i)`；新分配 `L,R` `collide_state_=0`，`collide_verified_gen_=0` |
| `try_promote_envelope_union(p,L,R,obs)` 成功 | `invalidate_collide(p)`（envelope 用 union 替换了，旧验证失效）。**注意**：union 内部已经做了 collide 验证，直接 `mark_collide_free(p)`（promote 成功必意味着 obstacle-free） |
| `try_promote_envelope_union` 失败回滚 | `invalidate_collide(p)` 防止脏写 |
| `transplant_domain` | 复制 worker 的 `collide_state_` + `collide_verified_gen_`（worker 与 master 同 generation，沿用） |
| `snapshot()` | 同上复制 |
| `clear_all_occupation()` | 不清 collide cache（occupation 不影响 envelope/collision） |
| `set_obstacles(...)` 后 `grow()` 入口 | `lect_.bump_obs_generation()` |
| `resize`（capacity 增长） | 新元素 `collide_state_=0`，`collide_verified_gen_=0` |

### 2.3 FFB 改造（[cpp/v6/src/ffb/ffb.cpp](../../src/ffb/ffb.cpp)）

```cpp
// step 4 改成：
{
    bool collides;
    if (lect.collide_cache_hit_free(current)) {
        collides = false;
        // 不计入 n_collide_calls
    } else if (lect.collide_cache_hit_collide(current)) {
        collides = true;
    } else {
        collides = (config.obs_grid && config.grid_margin_threshold > 0.0f)
            ? lect.collides_scene(current, obs, n_obs, config.obs_grid, config.grid_margin_threshold)
            : lect.collides_scene(current, obs, n_obs);
        result.n_collide_calls++;
        if (collides) lect.mark_collide_hit(current);
        else          lect.mark_collide_free(current);
    }
    if (!collides) { /* 返回 success */ }
}
```

**注意 step 3**：当 envelope 命中 cache (`has_data=true`) 但 collide cache miss 时，仍调 `collides_scene`；不是只在 envelope 重算时跑。

### 2.4 Grower 入口改造
`grow_rrt`/`grow_wavefront` 调用前：
```cpp
GrowerResult ForestGrower::grow(const Obstacle* obs, int n_obs) {
    auto t0 = Clock::now();
    lect_.bump_obs_generation();   // ← 新增
    ...
}
```
parallel/coordinated 入口同样 bump（master 一次即可，worker snapshot 自动继承）。

### 2.5 单测
- 静态场景，连续 2 次 FFB 同 seed → 第 2 次 `n_collide_calls == 0`
- `bump_obs_generation` 后第 3 次重新跑满
- 一次 promote_envelope_union 成功后，对父节点的 FFB 访问应直接命中 free
- expand_leaf 后访问新 child → cache miss

### 2.6 验收
- `exp5_ablation` IIWA14 三场景 `ffb_collide_ms` ↓ ≥30%
- `wall_time` ↓ ≥10%
- box 总数 / valid 率不退化

---

## 3. Phase U — 采样偏向未探索区域（新增需求）

### 3.1 思路
LECT 自带 `subtree_occ_[i]`（子树内已 occupy 的 leaf 数）。引入 `subtree_free_volume_[i]`（子树未 occupy 体积），递归地按 `free_volume / total_volume` 加权选择子节点 → 等价于在"未探索体积"上做加权均匀采样。

### 3.2 实现
**(a) LECT 维护 free volume**
```cpp
// include/sbf/lect/lect.h  (private)
std::vector<double> subtree_free_volume_;  // [capacity_]

// 接口
double subtree_free_volume(int i) const { return subtree_free_volume_[i]; }
void   refresh_free_volume();            // 一次性自底向上重算
```
- `expand_leaf(p)` 时把 `parent.free_volume` 拆给 `L,R`（按各自 interval 体积）
- `mark_occupied(i)` 时把 `i` 上溯到 root，每个 ancestor `free_volume -= node_volume(i)`
- `unmark_occupied(i)` 反向加回
- `try_promote_envelope_union(p, L, R)` 成功后：父节点变 occupy，`free_volume[p] = 0`，沿父链上溯做差额修正（实际效果：`L.free + R.free` 都已经在 occupy 时减过了，所以 union 不再额外修正——只是合并的地方记下"父节点变 leaf-style occupy"）

**(b) 采样函数**
```cpp
// grower 内新增
Eigen::VectorXd sample_unexplored() {
    // 自顶向下加权随机 walk
    int node = (domain_root_ >= 0) ? domain_root_ : 0;
    while (!lect_.is_leaf(node)) {
        double fl = lect_.subtree_free_volume(lect_.left(node));
        double fr = lect_.subtree_free_volume(lect_.right(node));
        if (fl + fr <= 0) break;
        double r = u01_(rng_) * (fl + fr);
        node = (r < fl) ? lect_.left(node) : lect_.right(node);
    }
    // 在 node 的 interval 内均匀采样
    auto ivs = lect_.node_intervals(node);
    Eigen::VectorXd q(static_cast<int>(ivs.size()));
    for (int d = 0; d < (int)ivs.size(); ++d)
        q[d] = ivs[d].lo + u01_(rng_) * ivs[d].width();
    return q;
}
```

**(c) 接入采样路径**（用 `unexplored_sample_prob` 配置）
```cpp
struct GrowerConfig {
    ...
    double unexplored_sample_prob = 0.7;  // 采样时偏向未探索区域的概率
};
```
- `grow_rrt`：原 `q_rand = sample_random()` 改为 `q_rand = (u01<prob) ? sample_unexplored() : sample_random()`
- `grow_wavefront::sample_boundary` 不动（已经是 boundary 偏置）
- `grow_coordinated` 同 `grow_rrt`

### 3.3 性能注意
- `mark_occupied`/`unmark_occupied` 上溯路径长度 = depth；每次 RRT step 增加 O(depth) ≈ O(20) 加减法，可忽略
- `expand_leaf` 时一次拆分，O(1)
- 采样 walk-down O(depth)，比 `sample_random + clamp` 仅多一倍

### 3.4 验收
- 同 box budget 下 vcov ↑ ≥5%（`exp5_ablation`）
- connect_time 不退化
- 关闭 prob=0 时与 baseline 完全一致（回归保护）

---

## 4. Phase B — 分层 max_depth schedule

### 4.1 配置
```cpp
struct GrowerConfig {
    ...
    /// Hierarchical FFB depth schedule. Empty (default) → single pass with ffb_config.max_depth.
    /// Each entry: (max_depth_for_stage, box_quota_ratio_of_total).
    std::vector<std::pair<int, double>> ffb_depth_stages;
    // 推荐值（默认空，由实验脚本显式开启）：
    // {{30, 0.4}, {80, 0.4}, {300, 0.2}}
};
```

### 4.2 实现
`grow()` 内主循环改造（serial 路径示例，parallel/coordinated 同样）：
```cpp
auto run_stages = [&](auto grow_fn) {
    if (config_.ffb_depth_stages.empty()) {
        grow_fn();
        return;
    }
    int total_budget = config_.max_boxes;
    int saved_max_depth = config_.ffb_config.max_depth;
    int cum_quota = 0;
    for (auto [d, ratio] : config_.ffb_depth_stages) {
        if (deadline_reached()) break;
        if (config_.connect_mode && config_.stop_after_connect && wf_all_connected_) break;
        cum_quota += static_cast<int>(total_budget * ratio);
        config_.ffb_config.max_depth = d;
        config_.max_boxes = std::min(total_budget, cum_quota);
        grow_fn();
    }
    config_.ffb_config.max_depth = saved_max_depth;
    config_.max_boxes = total_budget;
};
```
应用：`run_stages([&]{ grow_rrt(obs, n_obs); });`

### 4.3 验收
- 默认（空）下 baseline 数字完全不变
- 启用 `{{30,0.4},{80,0.4},{300,0.2}}` 后 envelope_ms ↓ ≥15%、connect_time 不退化

---

## 5. Phase C-1 — 无 root-seed 自动几何并行 + 自动桥接

### 5.1 触发条件
```cpp
bool should_partition_endpoint =
    config_.n_threads > 1 &&
    !has_multi_goals_ &&
    has_endpoints_ &&
    boxes_.size() < 2;  // 走不进 grow_parallel 的情况
```

### 5.2 流程
1. **合成种子**：
   - `seeds = {start_, goal_}`
   - 补 `n_threads - 2` 个 `sample_unexplored()` 种子（拒绝 `is_point_occupied` / 与已有 seed 同 leaf）
   - 用 `partition_for_seeds(seeds)` 把 master LECT 切到每个 seed 各占一个 leaf
2. **派发**：每 worker `set_domain_root(leaf_i)`，`grow_subtree(seed_i, root_id=i)`，复用现有 `transplant_domain` 合并
3. **Endpoint 桥接（自动）**：
   ```cpp
   // 在 grow_parallel 末尾、master promote_all 之后
   if (has_endpoints_ && !has_multi_goals_) {
       int start_box = find_box_containing(start_);
       int goal_box  = find_box_containing(goal_);
       if (start_box >= 0 && goal_box >= 0) {
           UnionFind uf((int)boxes_.size());
           for (int i = 0; i < (int)boxes_.size(); ++i)
               for (int j = i+1; j < (int)boxes_.size(); ++j)
                   if (boxes_adjacent(boxes_[i], boxes_[j]))
                       uf.unite(i, j);
           if (!uf.connected(start_box, goal_box)) {
               // 触发短窗 RRT bridge：临时把 start/goal box 当 multi_goals
               run_endpoint_bridge(obs, n_obs, start_box, goal_box);
           }
       }
   }
   ```
4. **Bridge 实现**（`run_endpoint_bridge`）：
   - 临时设 `multi_goals_ = {start_, goal_}`
   - 临时设 `connect_mode = true; stop_after_connect = true`
   - 给定一个小预算（`max_boxes += budget_bridge` ≈ 200 或 deadline 剩余 30%）
   - 调 `grow_rrt`（serial）；写完后恢复原配置
   - 失败则保留现状（grower 不报错；planner 层判断是否找到路径）

### 5.3 严格非重叠保证（用户强调）
- `partition_for_seeds` 已经保证每个 seed 落在不同 leaf
- 每 worker 用 `set_domain_root(leaf_i)`，`try_create_box` 内 `is_descendant_of(node, domain_root_)` 检查会拒绝任何越界 box
- `sample_random` / `sample_unexplored`（注意改造！）也要 clamp 到 `node_intervals(domain_root_)` 内
- **新增 sanity check**：`grow_subtree` 返回时遍历 `result.boxes`，断言每个 box 的 `joint_intervals` 完全被 `node_intervals(domain_root_)` 包住；不通过 → SBF_ERROR + 丢弃

### 5.4 sample_unexplored 的 domain 限制
```cpp
Eigen::VectorXd sample_unexplored() {
    int root = (domain_root_ >= 0) ? domain_root_ : 0;
    int node = root;
    while (!lect_.is_leaf(node)) { ... }
    // 在 node interval 内均匀采样（node 必在 root 子树内）
}
```
天然不会越界。

### 5.5 验收
- 单端点对场景 4 线程 wall-time speedup ≥ 2.4×
- 桥接成功率（5 seeds median）≥ 90%
- 跨 worker box 对的 `boxes_adjacent` 检查（合并后）必出现至少 1 对

---

## 6. Phase C-2 — n_threads > n_subtrees 二次切分

### 6.1 实现位置
`grow_parallel.cpp` worker 派发前：
```cpp
int n_workers = std::min(config_.n_threads, kMaxWorkers);
std::vector<int> domain_for_worker = lect_.partition_for_seeds(seed_vec);
std::vector<Eigen::VectorXd> worker_seeds = seed_vec;
std::vector<int> worker_root_id(n_subtrees);
std::iota(worker_root_id.begin(), worker_root_id.end(), 0);
// 都填入 worker_root_id

while ((int)domain_for_worker.size() < n_workers) {
    // 找 free volume 最大的 domain，强制 expand_leaf
    int largest = 0;
    double best_v = -1;
    for (int i = 0; i < (int)domain_for_worker.size(); ++i) {
        double v = lect_.subtree_free_volume(domain_for_worker[i]);
        if (v > best_v) { best_v = v; largest = i; }
    }
    int parent = domain_for_worker[largest];
    if (lect_.is_leaf(parent)) lect_.expand_leaf(parent);
    int L = lect_.left(parent), R = lect_.right(parent);
    // 替换：原来的 worker 改 domain=L，新增 worker domain=R，root_id 共享，seed = R 的 chebyshev center
    domain_for_worker[largest] = L;
    domain_for_worker.push_back(R);
    worker_seeds[largest] = chebyshev_of(L);
    worker_seeds.push_back(chebyshev_of(R));
    worker_root_id.push_back(worker_root_id[largest]);  // 共享同 RRT 树
}
```

### 6.2 严格非重叠
- `expand_leaf` 自动产生两个 disjoint child interval，仍是 LECT 结构
- 同 root_id 的两个 worker 共享 RRT 树概念但写入不同子树
- 合并阶段无需特别处理（box.root_id 一致就视为同树）

### 6.3 验收
- 5 multi-goal + 8 threads vs 5 + 5 threads → wall_time ↓ ≥1.3×
- box 总数变化 ≤10%

---

## 7. Phase D — Quick Wins

| ID | 改动 | 实现位置 |
|---|---|---|
| D1 | `seed_known_free` 透传去重 | `FFBConfig` 加 bool；FFB step 1 与 `try_create_box` 检查后置 true 后续 skip |
| D2 | `ForestGrower` 持 `CollisionChecker` 成员 | `grow()` 入口 set_obstacles 一次；`enforce_parent_adjacency` 读用 |
| D3 | `grow_rrt` K-subsample | port coordinated 的 `K_TREE=128` 到 serial |
| D4 | `promote_all` worklist | 全量分支换优先级队列；初始 push 所有"双 child 已 occupy"内部节点 |

---

## 8. 落地顺序

```
1. Phase A  ── LECT collide cache + FFB fast-path + grower bump generation
2. Phase U  ── LECT free_volume + sample_unexplored + grow_rrt 接入
3. Phase B  ── ffb_depth_stages 字段 + run_stages 包装
4. Phase C-1 ── partition_endpoint 分支 + run_endpoint_bridge
5. Phase C-2 ── secondary split for n_threads > n_subtrees
6. Phase D  ── 4 个 quick wins
7. 集成回归 ── exp5/exp6/exp9/exp10 跑 5 seeds
```

每完成一个 Phase 立即 build + 单测 + 一个轻量 benchmark；不一次堆完。

---

## 9. 验证清单

| 项 | 命令 | 通过条件 |
|---|---|---|
| 编译 | `cd cpp/v6/build && cmake --build . -j` | exit 0 |
| Phase A 单测 | 新增 `tests/test_ffb_collide_cache.cpp` | 同 seed 第 2 次 `n_collide_calls==0`；bump 后重置 |
| Phase A bench | `./build/experiments/exp5_ablation --seeds 5 --scene combined` | `ffb_collide_ms` ↓ ≥30% |
| Phase U bench | 同上 | vcov ↑ ≥5%、connect_time 不退化 |
| Phase B 默认 | 同上（不开 stages） | 数字与 baseline 完全一致 |
| Phase B 启用 | 加 `--ffb-stages 30:0.4,80:0.4,300:0.2` | envelope_ms ↓ ≥15% |
| Phase C-1 | endpoint 场景 1/2/4/8 thread | 4t speedup ≥2.4×、桥接成功 ≥90% |
| Phase C-2 | 5 multi-goal + 8t vs +5t | wall_time ↓ ≥1.3× |
| 回归 | exp5/6/9/10 5 seeds | path valid 不退化、box ±10% |
| 论文 | `latexmk` 重编译 | EN 20p、ZH 22p 不变 |

---

## 10. 风险与缓解

1. **Phase A collide=true 缓存可能阻断深层 split**：FFB step 4 命中 collide → step 5 走 `expand_leaf` + descend，等同于原行为只是少了一次 collide 调用，**安全**。但要确认 `expand_leaf` 末尾把 child 的 `collide_state_` 置 0（child envelope 是新计算的，必须重新验证）。
2. **Phase U 首次进入空 LECT**：`subtree_free_volume_` 初值 = root 的体积；`is_leaf(root) == true` → walk-down 0 步 → 直接全空间均匀采样，等价 `sample_random`，安全。
3. **Phase C-1 `partition_for_seeds` 失败**（合成种子落点都在同一 leaf 且 `max_extra_splits` 不够）：fallback 到 serial `grow_rrt`，记 SBF_WARN。
4. **Phase C-1 桥接失败**：grower 仍返回（产生的 box 集合可能多连通分量），planner 层 fall-back（已有 `wf_all_connected_` 标记）。
5. **Phase B 与 Phase C 同时启用**：每个 worker 独立跑 `run_stages`，无相互影响。
6. **Phase U + 并行**：每 worker 自己的 LECT snapshot 持有自己的 `subtree_free_volume_`，互不干扰；transplant 回 master 时**不**合并 free_volume，master 在 replay 阶段 `mark_occupied` 时自然重建。

---

## 11. Out of Scope
- LECT KD-split 策略（`BEST_TIGHTEN_V2` 已在）
- `try_promote_envelope_union` 算法本身
- planner / GCS / IRIS 改动
- 工作窃取（C-1 + C-2 稳定后再评估）
