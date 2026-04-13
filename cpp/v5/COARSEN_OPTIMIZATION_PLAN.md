# Coarsen 优化详细计划

## 目标

当前 25000 原始盒子经管线后只剩 ~8500（压缩率 66%）。目标：降至 3000-5000 boxes，以缩短 Dijkstra box_seq 长度、大幅提升 GCS 规划效率和路径质量。

## 当前管线与瓶颈

```
Raw(25000) → sweep1(10168,60%) → greedy1(8758,14%) → bridge(+544→9302)
           → sweep2(9189) → greedy2(8715) → filter(8589)
```

### 瓶颈 1: Sweep 条件极严（60% 压缩率上限）

- **条件**: 两盒在 sweep 维 d 接触（gap ≈ 0），且**所有其余 D-1 维的 lo/hi 精确匹配**（< 1e-10）
- **无碰撞检查**: 精确匹配保证 hull = A ∪ B（无多余空间）
- **问题**: 7D 空间中，6 维全匹配极少；来自不同 RRT 分支的盒子几乎不可能对齐
- **max_rounds = 10**（planner 中硬编码），header 默认 20

### 瓶颈 2: Greedy 参数过于保守（14% 额外压缩）

| 参数 | 当前值 | 瓶颈说明 |
|------|--------|---------|
| `score_threshold` | **50.0** | hull_vol/sum_vol > 50 直接跳过。7D 中 hull 体积膨胀指数级，大量可行候选被过滤 |
| `max_lect_fk_per_round` | **2000** | `check_box`(区间FK)有 false-positive，LECT 精确检查可挽救但预算不够 |
| `consumed_ids` | 每轮同一盒子只参与1次合并 | 限制了每轮产出 |
| 无 multi-box 合并 | 只做 pairwise | 小盒子簇需要多轮才能逐步合并 |

### 瓶颈 3: 无松弛 Sweep

Sweep 阶段完全跳过非精确对齐但接触的盒子对。这些候选可以用碰撞检查验证后合并。

## 修改计划

### Step A: 提高 Greedy 参数（最快见效）

**文件**: `include/sbf/forest/coarsen.h`

| 参数 | 旧值 | 新值 | 理由 |
|------|------|------|------|
| `score_threshold` | 50.0 | **200.0** | 7D hull 膨胀快，50 太保守；200 覆盖大部分实际可行的高维合并 |
| `max_lect_fk_per_round` | 2000 | **20000** | 当前大量 check_box false-positive 被拒，LECT 精确检查能挽救但预算不够 |

**文件**: `src/planner/sbf_planner.cpp`

- sweep `max_rounds`: `10` → `20`（两处调用）

**预期**: 8758 → ~6000-7000（greedy 多合并 20-30%）

### Step B: Multi-box Cluster 合并（新功能）

**文件**: `src/forest/coarsen.cpp`（新增函数）, `include/sbf/forest/coarsen.h`（声明）

**算法: `coarsen_cluster()`**

```
输入: boxes, checker, lect, protected_ids
每轮:
  1. compute_adjacency(boxes)
  2. 对每个盒子 b，收集其 1-hop 邻居形成候选 cluster = {b} ∪ neighbors(b)
  3. 计算 cluster hull AABB, score = hull_vol / sum_vol
  4. 按 score 升序排列所有候选 cluster
  5. 贪心选取（consumed_ids 互斥）:
     - cluster 内无 protected_id 被删除
     - hull 通过碰撞检查 (check_box || LECT)
     - 合并: 保留 b, 删除其余 cluster 成员
  6. 重复直到收敛
```

**关键设计选择**:
- **1-hop 邻居**(不是 2-hop): 限制 cluster 大小（~2-8 盒子），控制 hull 体积膨胀
- **按 score 排序**: 优先合并"紧凑"的 cluster（hull 浪费最少）
- **cluster_score_threshold = 100.0**: 比 pairwise(200) 低，因为 multi-box hull 膨胀更快
- **max_cluster_size = 8**: 限制最大 cluster 大小

**Config 新增**（`GreedyCoarsenConfig` 扩展）:

```cpp
double cluster_score_threshold = 100.0;  // hull_vol/sum_vol 上限
int max_cluster_size = 8;                // 每个 cluster 最大盒子数
int cluster_max_rounds = 20;             // cluster 合并最大轮数
```

**预期**: 在 greedy 基础上额外减少 20-30%（6000 → ~4000-5000）

### Step C: 松弛 Sweep（with 碰撞检查）

**文件**: `src/forest/coarsen.cpp`（新增函数）, `include/sbf/forest/coarsen.h`（声明）

**算法: `coarsen_sweep_relaxed()`**

与 `coarsen_forest` 结构相同（逐维排序扫描），但放宽合并条件:

```
原来: dim d 接触 + 所有其余维 lo/hi 精确匹配
新增: dim d 接触 + 所有其余维有显著重叠 (overlap > 50% of min_width)
```

具体变更:
1. 在精确匹配失败后，检查"松弛匹配":
   - 对每个非 sweep 维 d'，计算 overlap = min(A.hi, B.hi) - max(A.lo, B.lo)
   - 要求 overlap > 0.5 × min(A.width, B.width)（至少 50% 重叠）
2. 计算 hull AABB
3. **碰撞检查**: `check_box(hull)` + LECT fallback
4. 若安全，合并

**与 greedy 的区别**: 利用 sweep 的排序剪枝（`lo_j > hi_i` 时 break），只检查 dim d 上接触的对 — 比 greedy 的全邻接图遍历更高效。且不受 `score_threshold` 限制。

**参数**:
```cpp
struct RelaxedSweepConfig {
    double min_overlap_ratio = 0.5;     // 非 sweep 维最小重叠比例
    int max_rounds = 10;                // 最大轮数
    int max_lect_fk_per_round = 5000;   // LECT FK 预算
};
```

**预期**: sweep 阶段额外合并 10-20%（10168 → ~8000-9000，然后后续 greedy+cluster 进一步压缩）

### Step D: Pipeline 集成

**文件**: `src/planner/sbf_planner.cpp`

两个 coarsen pass 更新为:

```
Coarsen1:
  sweep_exact(20轮)
  → sweep_relaxed(with碰撞检查)
  → greedy(score=200, fk=20000)
  → cluster(score=100, max_size=8)

Coarsen2:
  sweep_exact(20轮)
  → sweep_relaxed
  → greedy
  → cluster
```

每步之间打印诊断:
```
[PLN] sweep1=Xms (25000->N1)
[PLN] sweep1_relaxed=Xms (N1->N2)
[PLN] greedy1=Xms (N2->N3)
[PLN] cluster1=Xms (N3->N4)
```

## 实施顺序

1. **Step A** (参数调优): 改 2 个常量 + 2 处 max_rounds → 编译运行 → 对比基线
2. **Step C** (松弛 sweep): 新增函数 → 集成 pipeline → 编译运行
3. **Step B** (cluster 合并): 新增函数 → 集成 pipeline → 编译运行
4. **验证**: 5 seeds × 5 queries，对比改进前后的:
   - box count (目标 3000-5000)
   - Dijkstra box_seq 长度
   - GCS 规划时间和路径质量
   - 路径碰撞安全性 (25/25)

## 预期效果

| 指标 | 改进前 | 改进后(预期) |
|------|--------|------------|
| 最终 boxes | ~8500 | ~3000-5000 |
| Dijkstra box_seq 长度 | 50-384 | 20-100 |
| GCS backbone boxes | 60+(触发 subsample) | 20-60(直接使用) |
| GCS 求解时间 | 0.5-3s | 0.1-1s |
| GCS 路径改善率 | 仅 TS→CS 有效 | 全部 5 查询有效 |
| coarsen 总耗时 | ~400ms | ~1000-2000ms(可接受) |

## 风险与缓解

| 风险 | 缓解措施 |
|------|---------|
| 大 hull 引入碰撞 | check_box + LECT 双重验证 |
| 过度合并破坏连通性 | 保护关节点(articulation points) |
| coarsen 耗时大增 | LECT FK 预算控制 + 收敛检测(无合并则停止) |
| 大盒子降低路径质量 | GCS 在大盒子内优化能力更强 |
