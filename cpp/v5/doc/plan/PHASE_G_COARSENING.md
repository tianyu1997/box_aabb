# Phase G: Coarsening (Module 6)

> 依赖: Phase F (Forest Grower)
> 状态: ✅ 已完成 (2026-04-04)
> 产出: Dimension-Sweep Merge + Tree-Cached Greedy Coarsening

---

## 目标

减少 box 数量以加速图搜索。提供两种合并策略：
1. **Dimension-Sweep**: 快速、无碰撞检查的精确面合并
2. **Tree-Cached Greedy**: 利用 LECT FK 缓存加速的贪心 hull 合并（**核心算法**）

---

## 核心参考

> ⭐ **v1 是唯一完整的 C++ tree-cached greedy coarsening 实现**
> 源文件: `v1/include/sbf/forest/coarsen.h` + `v1/src/forest/coarsen.cpp`

---

## Step G1: Dimension-Sweep Merge

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/forest/coarsen.h` | `src/forest/coarsen.cpp` |

### 数据结构
```cpp
struct CoarsenResult {
    int merges_performed = 0;
    int rounds = 0;
    int boxes_before = 0;
    int boxes_after = 0;
};
```

### 接口
```cpp
CoarsenResult coarsen_forest(
    std::vector<BoxNode>& boxes,
    const CollisionChecker& checker,
    int max_rounds = 20
);
```

### 算法
```
for round in [0, max_rounds):
    merged_this_round = 0
    for dim in [0, n_dims):
        // 沿 dim 排序
        sort boxes by intervals[dim].lo

        // 找 touching runs: box_a.hi[dim] == box_b.lo[dim] (within tol)
        // 且其他维度完全匹配
        for each touching pair (a, b):
            if all other dims: a.intervals[d] == b.intervals[d]:
                // 合并: new_box = hull(a, b)
                merge(a, b) → new_box
                merged_this_round++

    if merged_this_round == 0: break

return {total_merges, rounds, before, after}
```

### 特点
- 要求**精确面接触** + 其他维度完全匹配
- **无需碰撞检查**: A safe + B safe + exact-touching → hull safe
- 简单快速，但合并率有限（仅限完美对齐的 box）

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v1 | `v1/include/sbf/forest/coarsen.h` → `coarsen_forest()` |
| v1 | `v1/src/forest/coarsen.cpp` |

---

## Step G2: Tree-Cached Greedy Coarsening ⭐

### 数据结构
```cpp
struct GreedyCoarsenResult {
    int merges_performed = 0;
    int rounds = 0;
    int boxes_before = 0;
    int boxes_after = 0;
    double elapsed_sec = 0.0;
};

struct GreedyCoarsenConfig {
    int target_boxes = 0;             // 目标 box 数 (0=收敛)
    int max_rounds = 100;             // 最大轮数
    double adjacency_tol = 1e-10;     // 邻接检测容差
    double score_threshold = 50.0;    // 分数阈值 (跳过 score > threshold)
    int max_lect_fk_per_round = 2000; // 每轮 LECT FK 调用预算
};
```

### 接口
```cpp
GreedyCoarsenResult coarsen_greedy(
    std::vector<BoxNode>& boxes,
    const CollisionChecker& checker,
    const GreedyCoarsenConfig& config,
    LECT* lect = nullptr              // 可选: LECT FK 加速
);
```

### 算法详解

```
coarsen_greedy(boxes, checker, config, lect):
  result.boxes_before = boxes.size()

  for round in [0, max_rounds):
    // ---- Stage 1: 候选收集 ----
    adj = compute_adjacency(boxes, config.adjacency_tol)
    candidates = []

    for (id_a, neighbors) in adj:
      for id_b in neighbors:
        if id_a >= id_b: continue  // 去重

        box_a = boxes[id_a], box_b = boxes[id_b]

        // 计算 hull AABB
        hull_ivs = []
        for d in [0, n_dims):
            hull_ivs[d] = {min(a.lo[d], b.lo[d]), max(a.hi[d], b.hi[d])}

        hull_vol = product(hull_ivs[d].width() for d)
        sum_vol  = box_a.volume + box_b.volume

        score = hull_vol / sum_vol  // 越低越好 (1.0 = 完美贴合)

        if score > config.score_threshold: continue  // 启发式剪枝

        candidates.push_back({score, id_a, id_b, hull_ivs})

    if candidates.empty(): break

    // ---- Stage 2: 排序 + 贪心执行 ----
    sort(candidates, by score ascending)  // 最优先合并最紧凑的

    merged_this_round = set<int>()
    fk_budget_used = 0

    for cand in candidates:
      if cand.id_a in merged_this_round: continue  // 每 box 每轮最多合并一次
      if cand.id_b in merged_this_round: continue

      // ---- 碰撞检查: 两阶段 ----
      hull_safe = false

      // Fast path: 直接 checker
      if !checker.check_box(cand.hull_ivs):
        hull_safe = true

      // Tree path: LECT FK 加速 (if lect available && budget remaining)
      if !hull_safe && lect != nullptr && fk_budget_used < config.max_lect_fk_per_round:
        fk_before = lect->total_fk_calls()
        hull_safe = !lect->intervals_collide_scene(cand.hull_ivs, obs, n_obs)
        fk_budget_used += (lect->total_fk_calls() - fk_before)

      if !hull_safe: continue

      // ---- 执行合并 ----
      new_box.joint_intervals = cand.hull_ivs
      new_box.volume = hull_vol
      new_box.tree_id = -1  // 不再绑定 LECT 节点
      new_box.seed_config = (box_a.center() + box_b.center()) / 2

      remove box_a, box_b from boxes
      add new_box to boxes

      merged_this_round.insert(cand.id_a)
      merged_this_round.insert(cand.id_b)

    result.merges_performed += merged_this_round.size() / 2

    if merged_this_round.empty(): break
    if boxes.size() <= config.target_boxes && config.target_boxes > 0: break

  result.boxes_after = boxes.size()
  return result
```

### 打分机制

$$\text{score} = \frac{V_{\text{hull}}}{V_A + V_B}$$

| Score | 含义 | 合并质量 |
|-------|------|---------|
| ≈ 1.0 | 完美贴合，无浪费空间 | 最优 |
| 1.0 ~ 2.0 | 较好，少量间隙 | 好 |
| 2.0 ~ 10.0 | 中等，有明显间隙 | 可接受 |
| > 50.0 | 差，hull 远大于原始 box | 跳过 |

### Tree-Cached 加速原理
- LECT 中已缓存大量节点的 FK 结果
- 碰撞检查时，LECT 可以复用已有的 link iAABBs，避免重新计算 FK
- `max_lect_fk_per_round` 防止过多 FK 调用拖慢单轮速度

### 迁移来源
| 源版本 | 源文件 | 备注 |
|--------|--------|------|
| **v1** | `v1/include/sbf/forest/coarsen.h` → `coarsen_greedy()` | **核心参考** |
| **v1** | `v1/src/forest/coarsen.cpp` | 完整实现 |

### 与 v1 差异
- `HierAABBTree*` → `LECT*`（类型变化，接口类似）
- `tree_split_depth` 参数 → 改为使用 LECT 的统一碰撞查询
- 打分阈值从硬编码 50.0 改为可配置

---

## 推荐使用方式

```cpp
// 1. 先 dimension-sweep (快速, 无碰撞检查)
auto sweep_result = coarsen_forest(boxes, checker, 10);

// 2. 再 greedy coarsening (精细, 利用 LECT 缓存)
GreedyCoarsenConfig gc_config;
gc_config.target_boxes = 100;
gc_config.max_rounds = 50;
auto greedy_result = coarsen_greedy(boxes, checker, gc_config, &lect);
```

---

## 测试: test_coarsen.cpp

```
TEST_SUITE("DimensionSweep") {
    - 2DOF 两个 exact-touching boxes → 合并为 1
    - 2DOF 两个 non-touching boxes → 不合并
    - 2DOF 4×4 网格 → 合并后 box 数显著减少
    - 合并后 box 覆盖与合并前一致 (volume 守恒)
}

TEST_SUITE("GreedyCoarsen_Basic") {
    - 2DOF: boxes 减少
    - 合并后所有 box 无碰撞 (MC 验证)
    - score 排序正确: 首先合并 score 最低的
    - target_boxes 生效: 达到目标后停止
}

TEST_SUITE("GreedyCoarsen_TreeCached") {
    - 有 LECT vs 无 LECT: 有 LECT 时 FK 调用更少
    - FK budget: max_lect_fk_per_round 生效
    - 合并结果与无 LECT 时一致 (正确性不变)
}

TEST_SUITE("GreedyCoarsen_Edge") {
    - 空 box 列表 → 不报错, merges = 0
    - 1 个 box → 不报错, merges = 0
    - 全部 box 互不邻接 → merges = 0
    - score_threshold = 0.0 → 不合并 (所有 score > 0)
}
```
