# Exp3 优化计划：Cold vs Warm Rebuild 性能改进

## 基线数据 (2026-04-13, 修正同 seed)

| 模式 | 中位时间 | 平均时间 | 中位 box 数 | 查询成功率 | 路径长度中位 |
|------|---------|---------|-----------|-----------|-----------|
| Cold (无缓存) | 9.04s | 8.87s | 6124 | 100% | 3.99 |
| Warm (有缓存) | 10.71s | 10.69s | 8314 | 100% | 3.78 |
| Perturb ±2cm | 12.27s | 13.64s | 7026 | 100% | 3.81 |

**核心矛盾**: Cold/Warm speedup = 0.84x, warm 反而比 cold 慢 16%

### 阶段耗时分析

| 阶段 | Cold 平均 | Warm+Pert 平均 | 方差来源 |
|------|----------|---------------|---------|
| grow | 5002ms | 5002ms | 固定超时，无差异 |
| sweep1 | ~640ms | ~640ms | 一致 |
| greedy1 | ~830ms | ~950ms | candidate 数量差异 |
| bridge_all | ~860ms | ~1185ms | island 数量/大小差异 |
| greedy2 | **175–5881ms** | **同** | **35x 方差，关键瓶颈** |
| seed-point bridge | 0 | **+208–4964 boxes** | **RRT 随机性** |

### GCS 查询质量

- post-hoc 碰撞率 22.1% (58/60 查询受影响)
- 所有查询均退回 raw waypoints
- hull_region_safe 递归深度仅 6 层

---

## 方案 P0: Greedy Coarsen Timeout + Stagnation Early-Exit

**优先级**: 最高  
**预期收益**: 消除 greedy2 的 5881ms 极端值 → worst-case 降至 ~1s  
**实现难度**: 低

### 问题根因

`coarsen_greedy()` 循环最多 `max_rounds=100` 轮，每轮需要:
1. 重建 adjacency (O(n²) worst-case)
2. 遍历所有邻居对构建 candidate 列表
3. 排序 + greedy 执行

当 box 数量较多(>6000) 且 `score_threshold=50` 较宽松时，candidate 列表可达数万项，
但 collision check 失败率很高(大多数 hull 不安全)，导致每轮仅合并 1-2 对。

### 具体改动

**文件**: `cpp/v5/src/forest/coarsen.cpp`, `coarsen_greedy()` 函数

1. **添加 `timeout_ms` 参数** (默认 2000ms):
   - 在 round 循环开头检查 elapsed > timeout_ms → break
   
2. **添加 stagnation 检测**:
   - 连续 `max_stagnant_rounds=3` 轮 merge 数为 0 → break
   - (当前只有 `merges_this_round == 0` 时立即 break — 已有此逻辑)
   - 改为: 当连续 2 轮 merge < 上一轮的 10% → break

**文件**: `cpp/v5/include/sbf/forest/coarsen.h`, `GreedyCoarsenConfig`

```cpp
struct GreedyCoarsenConfig {
    // ... 现有字段 ...
    double timeout_ms = 2000.0;       // 单次 greedy 最大耗时
};
```

---

## 方案 P1: Seed-Point Bridge 自适应 & 提前跳过

**优先级**: 高  
**预期收益**: 减少 warm/pert 的 seed-point bridge 开销 30-50%  
**实现难度**: 低

### 问题根因

`build_coverage()` 第 660-710 行的 OP-1 seed-point bridge:
- 对**每个** seed point 检查是否在 main island 中
- 若不在，用 `bridge_s_t(per_pair_timeout=3000ms, max_pairs=10)` 暴力桥接
- 每次桥接后重建 find_islands()

高方差原因:
- 有些 seed point 在 coarsen/filter 后被剥离出 main island
- bridge_s_t 的 RRT 随机性巨大 (200-4964 boxes)

### 具体改动

**文件**: `cpp/v5/src/planner/sbf_planner.cpp`, OP-1 区块

1. **distance-adaptive timeout**: `per_pair_timeout = min(3000, 500 + 50 * sqrt(dist))`
   - 近距离 seed point (~0.3 rad) → 500ms 即可
   - 远距离 seed point (~2.0 rad) → 需要 2-3s

2. **先检查 containment**: 如果 seed point 被 main island 的某个 box 包含，直接跳过
   (当前代码用 `b.contains(sp)` 找最近 box → 取 nearest，但即使该 box 已在 main island，
   仍会走到 `main_set.count(sp_id)` 判断 — 这部分逻辑正确，但可以提前短路)

3. **限制 max_pairs**: `max_pairs = min(10, 3 + int(dist))` — 近距离只尝试 3 对

4. **总预算限制**: seed-point bridge 总时间限制 = 10s，超时跳过剩余 seed points

---

## 方案 P2: GCS Corridor 碰撞分割深度提升

**优先级**: 中  
**预期收益**: 降低 post-hoc 碰撞率 22% → ~10%, 减少 RRT fallback  
**实现难度**: 中

### 问题根因

`gcs_planner.cpp:374` 调用 `hull_region_safe(..., max_depth=6)`:
- 7-DOF 空间的 hull 区域在 6 层递归细分后 (2^6=64 个子域) 仍可能遗漏安全子域
- 导致保守地判断为"不安全"→ 碰撞分割
- 但 post-hoc 验证又发现碰撞 → 说明分组本身就不合理

分析: `group_size=2`（最小值），但即使 2:1 grouping，corridor 中相邻 box 的 hull
可能跨越障碍物边界 → 应该在 grouping 策略上更保守。

### 具体改动

**文件**: `cpp/v5/src/planner/gcs_planner.cpp`

1. **提升 hull_region_safe 递归深度**: `max_depth = 6 → 8`
   - 代价: 最坏情况下 collision check 增加 4x (2^2 extra subdivisions)
   - 好处: 更准确判断 hull 安全性，减少不必要的碰撞分割

2. **Post-hoc 碰撞检查修复**: 在 GCS 求解后，对碰撞段用 corridor 内 box 做局部
   shortcut 而非直接退回 raw waypoints:
   - 碰撞段 → 找到该段两端 box → 在 corridor 内做 Dijkstra reroute
   - 这避免了退回到低质量的 chain path

---

## 方案 P3: 修正实验基准 — LECT 缓存的真实加速点

**优先级**: 中  
**预期收益**: 重新定义 cold/warm 含义，量化 FFB 加速  
**实现难度**: 低

### 问题根因

当前 `lect_no_cache=true` 只影响:
- LECT 树加载 (0ms vs 0ms — 无区别)
- V6 Z4-keyed cache (envelope 查找)

真正的缓存加速发生在 **FFB 阶段**: warm 的 423 个 pre-expanded 节点
使 FFB 少做 expand_leaf() 调用。但 grow 阶段受 timeout 限制 (5000ms)，
总 FFB 调用量不变 — 每次 FFB 更快但次数不减。

warm 反而更慢的原因: warm FFB 更快 → 同样 5s 内生成更多 box (22000 vs 23000) 
→ 后续 coarsen 和 bridge 处理更多 box。

### 具体改动

**文件**: `cpp/v5/experiments/exp3_incremental.cpp`

1. **分离 FFB 加速计量**: 在 grow 阶段输出 `n_cache_hits / n_cache_misses / n_fk_calls`
   - 这是 LECT 缓存的真实作用点
   - cold 应该 `cache_miss >> cache_hit`, warm 相反

2. **增加 `--match-boxes` 模式**: warm build 设置 `max_boxes = cold 的 box 数`
   - 这样 box 数量一致，仅比较单个 FFB 调用的速度差异
   - 可以正确量化 LECT cache hit 的加速比

3. **输出 per-phase timing**: 在 summary 中打印每个阶段的 cold/warm 对比

---

## 实施顺序

1. **P0** (greedy2 timeout) — 最高 ROI, 5881ms → ~1s
2. **P1** (seed-point bridge adaptive) — 直接减少 warm 开销
3. **P2** (GCS depth) — 改善查询质量
4. **P3** (实验基准修正) — 正确量化缓存效果

每一步实施后跑 exp3 验证。
