# SBF v5 全流程详细耗时分析与加速建议

> **运行环境**: KUKA IIWA14 (7-DOF, 4 active links), 16 obstacles, 16 threads, seed=0
> **日期**: 2026-04-10 (v4 — T+B+C series 优化后)
> **基准缓存**: `~/.sbf_cache/kuka_iiwa14_r820_bc1d61198ab31e09.lect.baseline`
>
> **已实施优化**:
> - T-series: T1 (sweep-line adjacency), T7 (sweep mark-compact)
> - B-series: B1 (碰撞检测栈分配), B2 (point FK 快速路径), B3 (bridge seg_res 10), B6 (渐进超时+cancel)
> - C-series: C4 (bridge timeout 缩短 R0=2500ms/50K, R1=80K), C2 (ares 0.005→0.01)
> - Earlier: P1-A (bridge 智能早停), P2-A (simplify 松弛), P3 (query RRT 并行), P4 (center cache), P5 (LECT 异步保存), S4-A (短路径跳过 random shortcut), S5-A (inline config check), S7-A/B (短路径跳过 shortcut/pass2)

---

## 一、总体时间分布 (当前)

| 阶段 | 耗时 | 占比 | vs 原始 35.56s |
|------|------|------|---------------|
| **Offline Build** | **7.29s** | **68.3%** | **-70.8%** (was 24.97s) |
| └ LECT 加载 | 5ms | 0.1% | mmap cached 204,859 nodes |
| └ **并行森林生长 (grow)** | **3,661ms** | **34.3%** | 5 trees × 5,000 boxes |
| └ LECT 保存 (async) | 261ms | — | 与后续阶段重叠 |
| └ Sweep coarsen | 433ms | 4.1% | 25,086 → 10,269 |
| └ Adj+Artic (pre-greedy) | 27ms | 0.3% | 6,860 articulation points |
| └ Greedy coarsen | 126ms | 1.2% | 10,269 → 8,883 |
| └ Adj+Artic+Filter (post-greedy) | 336ms | 3.1% | 8,883 → 8,720 |
| └ Pre-bridge adj | 17ms | 0.2% | |
| └ **Parallel RRT Bridge** | **2,679ms** | **25.1%** | C4 优化: was 7,097ms |
| **Online Query (5 queries)** | **3.25s** | **30.4%** | **-69.3%** (was 10.59s) |
| └ AS→TS | 328ms | 3.1% | len=5.060, 12 pts |
| └ TS→CS | 768ms | 7.2% | len=5.978, 7 pts |
| └ **CS→LB** | **2,088ms** | **19.6%** | len=8.920, 11 pts ← **最慢** |
| └ LB→RB | 39ms | 0.4% | len=4.788, 6 pts |
| └ RB→AS | 25ms | 0.2% | len=1.961, 6 pts |
| **E2E Total** | **10.54s** | **100%** | **-70.4%** |

---

## 二、Offline Build 各子阶段细分 (7.29s)

### 2.1 LECT 加载 (5ms = 0.1%)

```
header=0.0ms  tree=3.9ms  mmap+lazy=0.7ms  derived=0.0ms  total=4.6ms
loaded 204,859 cached nodes
```

完全 I/O bound，已接近极限。

### 2.2 并行森林生长 (3,661ms = 50.2%)

| 子环节 | 耗时 | 备注 |
|--------|------|------|
| Root selection (FPS) | <1ms | 5 seeds via multi-goal |
| **Parallel RRT grow** | **3,659ms** | 5 trees × 5,000 boxes each → 25,086 总 |
| Cross-seed | 含在上面 | +86 boxes, 5 islands detected |
| **FFB 总计** | **2,240ms** | 100% cache hit (BEST_TIGHTEN) |
| └ expand calls | 316,479 | avg 0.002ms/call |
| └ pick_dim | 9.0ms | 含 cache 查找 |
| └ FK (incremental) | 316.0ms | 0.001ms/call |
| └ envelope compute | 310.3ms | 0.001ms/call |
| └ refine | 18.6ms | |
| **RRT overhead** | **~1,420ms** | sampling + nearest + snap + collision |

**FFB 详情**: 316,479 expand calls 产生 632,958 new nodes。expand_dim cache 100% 命中。
每次 expand 平均: pick_dim=0.000ms, fk=0.001ms, env=0.001ms, refine=0.000ms → total=0.002ms。

**关键发现**: Grow 已充分优化。FFB 2.24s (61%) + RRT overhead 1.42s (39%)。
- FFB 受限于 FK + envelope 计算 (~626ms = 44% of FFB)，这是物理计算下界
- RRT overhead (sampling/nearest/snap) ~1.42s 主要是碰撞检查 (check_config/check_segment)
- 100% LECT cache hit → 无需远程重算
- 进一步优化空间 <10%

### 2.3 Sweep Coarsen (433ms = 5.9%)

维度扫描合并: 25,086 → 10,269 boxes (减少 59%)。
T1 (sweep-line adjacency) + T7 (mark-compact) 已将此阶段从 ~840ms 优化到 433ms (-48%)。
主要成本是 7 维度 × N 个 box 的排序和合并扫描。

### 2.4 Adj + Greedy + Filter (489ms = 6.7%)

| 子环节 | 耗时 | 说明 |
|--------|------|------|
| Pre-greedy adj+artic | 27ms | Tarjan 算法找 6,860 articulation points |
| Greedy coarsen | 126ms | 10,269 → 8,883 (-13.5%) |
| Post-greedy adj+artic+filter | 336ms | 8,883 → 8,720, 含碰撞过滤 |
| **合计** | **489ms** | |

**观察**: adj+artic 合计 363ms (74%)，greedy 仅 126ms (26%)。adj 阶段需对所有 box pair 做区间重叠检测，是 O(n × avg_neighbors)。

### 2.5 Parallel RRT Bridge (2,679ms = 36.7%) ← **Build 最大瓶颈**

**配置** (C4 优化后):
- 15 worker threads
- Round 0: timeout=2500ms, max_iters=50K, step_size=0.1, goal_bias=0.20, seg_res=10
- Round 1+: timeout=per_pair_timeout, max_iters=80K
- 连续失败 3 次 → 跳过该 island

#### Round 0: 5 islands → 2 islands

| Island | 大小 | Pairs 尝试 | 结果 | RRT 详情 |
|--------|------|-----------|------|----------|
| #1 (2080 boxes) | 10 pairs | **1** | ✅ merged | 42wp, 103 iters (极快) |
| #2 (1673 boxes) | 10 pairs | **2** | ✅ pair 2 | pair 1: 5× FAIL@50K; pair 2: 201wp, 21978 iters |
| #3 (1559 boxes) | 10 pairs | **1** | ✅ merged | 34wp, 232 iters (快) |
| **#4 (1295 boxes)** | 10 pairs | **3** | **❌ ALL FAIL** | 3× FAIL@50K, tb=19K+ 已饱和 |

**Island #2 详情**: Pair 1 启动 5 个并行 RRT (15 threads pool)，全部 FAIL@50K iters。
Pair 2 的多个并行 RRT 中至少 3 个成功 (21978, 32344, 29953 iters)。
Wall time 受限于 FAIL 组的 50K iters。

**Island #4 分析**: 3 个 pair 各 4~5 并行 RRT，全部 FAIL@50K iters (ta=469-641, tb=19464-19704)。
tree_b ≈ 19.5K 节点说明 goal 侧空间已充分探索但 start/goal 确实不连通。
最终靠 Round 1 post-bridge adj 重算连通。

#### Round 1: 2 islands → 1 island

| 详情 | |
|------|--|
| 剩余: main=7800, #1=1295 | |
| Pair 1: **✅ 成功** | 287wp, 69912 iters (ta=1128, tb=1681) |
| 378 bridge boxes 创建 | |
| 最终: **9098 boxes, 1 island, 10022 edges** | |

#### Bridge 时间分解 (Wall time)

| 阶段 | 估算 | 说明 |
|------|------|------|
| R0 island #1+#3 (极快合并) | ~0.1s | 并行完成, 几乎瞬时 |
| R0 island #2 pair 1 FAIL | ~1.0s | 50K iters@2500ms timeout |
| R0 island #2 pair 2 SUCCESS | ~0.5s | 21978 iters |
| R0 island #4 (3 FAIL) | ~1.0s | 与 island #2 并行, 50K@2500ms |
| R1 pair 1 SUCCESS | ~1.0s | 69912 iters |
| **Total** | **~2.7s** | 匹配实测 2,679ms |

**Bridge 浪费分析**:
- Island #4 R0: 12 次 FAIL RRT (3 pair × ~4 parallel), 每次 50K iters → CPU 总量大但由于并行仅浪费 ~1s wall time
- R1 成功极快 (1 pair 即连通), 但 69K iters 表示这是一个较难的连通

### 2.6 其他 (微小)

- LECT 异步保存: 261ms (360,489 nodes), 与 sweep/adj 并行重叠不占主流水线
- Pre-bridge adj: 17ms

---

## 三、Online Query 各查询详细分析 (3.25s)

### 3.0 Query 流水线结构

```
Dijkstra shortest path → box-chain extract → pre-compete simplify (greedy)
  → RRT compete (3 parallel trials: 1 main + 2 bonus)
  → segment validation + RRT repair (for collision segments)
  → greedy forward simplify → [random shortcut if >8 pts]
  → densify (max_seg=0.3 rad) → elastic band (60 iters, 4 alpha)
  → final shortcut → [pass2 EB (30 iters, if EB improve ≥5%)]
  → final safety-net validation → [emergency RRT if still colliding]
```

**关键参数**:
| 参数 | 值 | 用途 |
|------|-----|------|
| segment_resolution | 20 | 所有 query-time RRT, repair, validation |
| ares | max(20, ceil(len/0.01)) | EB, simplify, shortcut 的碰撞精度 (C2) |
| RRT step_size | 0.2 | query compete |
| RRT goal_bias | 0.15 | query compete |
| RRT compete timeout | adaptive by ratio | main: 1/3/5s; bonus: min(main,1s) |
| EB iterations | 60 outer | 4 alpha = {0.5, 0.3, 0.15, 0.05} |
| Pass2 EB | 30 outer | 3 alpha = {0.4, 0.2, 0.08}, gated by ≥5% improvement |

**RRT compete timeout 逻辑**:
```
ratio = chain_len / max(euclid_dist, 0.01)
rrt_timeout = (ratio < 2.0) ? 1000 : (ratio < 3.0) ? 3000 : 5000  // main trial
bonus_timeout = min(rrt_timeout, 1000)                              // bonus trials
```

### 3.1 AS→TS (328ms)

| 步骤 | 详情 | 估算耗时 |
|------|------|---------|
| Dijkstra + extract | 483 pts, len=32.796, 0 link paths | ~2ms |
| Pre-compete simplify | 483→4 pts → **REVERTED** (config invalid) | ~5ms (S5-A early bail) |
| **RRT compete** | **3/3 OK**, ratio≈6.6→timeout=5s/1s | **~250ms** |
| └ Trial 1 (main, seed=42) | 84wp, 24229 iters (ta=9980, tb=459) | |
| └ Trial 2 (bonus, seed=179) | 99wp, 26063 iters (ta=10614, tb=504) | |
| └ Trial 3 (bonus, seed=316) | 61wp, 28729 iters (ta=11819, tb=503) | |
| └ simplify_rrt (per trial) | 含 check_segment | 含在上面 |
| → best direct | len=6.116 (vs chain 32.796, **-81%**) | |
| Segment validation + repair | 2/11 segs collide → RRT repair | ~15ms |
| Greedy simplify | 6→6 pts (无效) | ~2ms |
| Densify | 6→23 pts (max_seg=0.3) | <1ms |
| **EB** | **409 moves**, 6.116→5.310 (-13.2%) | **~30ms** |
| Final shortcut | 5.310→5.060 (-4.7%) | ~10ms |
| Final validation | 2/11 segs IN COLLISION (已 repair) | ~5ms |
| **Total** | **328ms** | len=5.060, 12 pts |

**RRT tree 不平衡**: ta≈10K vs tb≈500 → **22:1** → AS 端有 narrow passage。

### 3.2 TS→CS (768ms)

| 步骤 | 详情 | 估算耗时 |
|------|------|---------|
| Dijkstra + extract | 96 pts, len=7.406 | ~1ms |
| Pre-compete simplify | 96→3 pts → **REVERTED** | ~3ms (S5-A early bail) |
| **RRT compete** | **3/3 OK**, ratio≈1.2→timeout=**1s/1s** | **~700ms** |
| └ Trial 1 (main) | 177wp, 63605 iters (ta=7389, tb=496) | |
| └ Trial 2 (bonus) | 80wp, 63642 iters (ta=10180, tb=339) | |
| └ Trial 3 (bonus) | 65wp, **71447 iters** (ta=19262, tb=398) | |
| └ simplify_rrt | 177→~5 pts etc. | 含在上面 |
| → best direct | len=6.187 (vs chain 7.406, **-16%**) | |
| Segment validation | 1/6 segs collide | ~3ms |
| Greedy simplify | 5→5 pts | ~1ms |
| Densify | 5→23 pts | <1ms |
| **EB** | **914 moves**, 6.187→5.978 (-3.4%) | **~50ms** |
| Final shortcut | 5.978→5.978 (无效 → SKIPPED by S7-A, ≤6pts?) | ~5ms |
| Pass2 EB | **SKIPPED** (EB improve 3.4% < 5%) | — |
| **Total** | **768ms** | len=5.978, 7 pts |

**关键问题**: chain=7.406 vs direct=6.187 仅差 **16%**，但 RRT 做了 **71K iters, ~700ms** 。
这是 **"高成本低回报"** 的典型案例 — 花 700ms 仅获得 1.2 rad 路径缩短。
由于 ratio=1.2 < 2.0, timeout=1000ms, 3 个 trial 全部接近 1s 极限。

**RRT tree 不平衡**: ta≈12K vs tb≈400 → **30:1** → TS 端高维探索困难。

### 3.3 CS→LB (2,088ms) ← **最慢查询, 占 Query 64%**

| 步骤 | 详情 | 估算耗时 |
|------|------|---------|
| Dijkstra + extract | 617 pts, len=37.684 | ~2ms |
| Pre-compete simplify | 617→3 pts → **REVERTED** | ~5ms |
| **RRT compete** | **1/3 OK**, ratio≈7.2→timeout=5s/1s | **~1,500ms** |
| └ Trial 1 (bonus, seed=179) | **TIMEOUT** 1000ms, 53640 iters (ta=326, tb=21474) | 1,000ms |
| └ Trial 2 (bonus, seed=316) | **TIMEOUT** 1000ms, 51512 iters (ta=533, tb=20723) | 1,000ms |
| └ Trial 3 (main, seed=42) | **SUCCESS** 116wp, 78442 iters (ta=648, tb=31320) | ~1,500ms |
| └ simplify_rrt on trial 3 | 116→~5 pts | 含在上面 |
| → best direct | len=10.492 (vs chain 37.684, **-72%**) | |
| Segment validation | 2/10 segs collide → repair | ~5ms |
| Greedy simplify | 5→5 pts | ~1ms |
| Densify | 5→38 pts | <1ms |
| **EB** | **1764 moves**, 10.492→9.604 (-8.5%) | **~150ms** |
| Final shortcut | 9.604→9.590 (-0.1%) | ~60ms (max_iters=1140) |
| **Pass2 EB** | **246 moves**, 9.590→8.920 (**-7.0%**) | **~30ms** |
| └ Pass2 greedy simplify | 含在内 | |
| Final validation | 2/10 segs IN COLLISION (已 repair) | ~5ms |
| **Total** | **2,088ms** | len=8.920, 11 pts |

**RRT 分析**:
- Main trial (5s budget) 用了 78442 iters, tree_b 膨胀至 31320 → **极高难度 query**
- 2 bonus trials (1s budget) 各做 ~52K iters 全部 timeout
- Wall time: main trial ~1.5s 是瓶颈 (bonus 在 1s 已结束)
- **CS→LB 是唯一需要 >1s RRT 的查询**

**RRT tree 不平衡**: ta≈500 vs tb≈24K → **1:49** → LB 端高维探索困难。
方向相反: 此次是 goal (LB) 端膨胀, start (CS) 端受阻。

**EB 分析**:
- 1764 moves on 38 densified pts, 60 outer iters → 每 iter ~29 moves 平均
- Step 3 EB: 10.492→9.604 (-8.5%)
- Step 5 pass2 EB: 9.590→8.920 (-7.0%)
- 两轮 EB 合计削减 **15%**，因初始 RRT 路径弯绕严重

### 3.4 LB→RB (39ms) — 快速查询

| 步骤 | 详情 | 估算耗时 |
|------|------|---------|
| Dijkstra + extract | 326 pts, len=27.474 | ~2ms |
| **Pre-compete simplify** | 326→8 pts, **成功!** 27.474→14.793 (**-46%**) | ~10ms |
| RRT compete | 3/3 OK, direct=5.010 | ~5ms |
| └ Trial 1/2/3 | 38/41/52 wp, 87/97/360 iters (极快) | |
| Greedy simplify | 6→6 pts | ~1ms |
| Densify | 6→21 pts | <1ms |
| EB | 292 moves, 5.010→4.863 (-2.9%) | ~10ms |
| Final shortcut | 4.863→4.788 (-1.5%) | ~5ms |
| **Total** | **39ms** | len=4.788, 6 pts |

**关键**: Pre-compete simplify 成功 (chain 可简化 46%) 使得后续 RRT 极快 (<400 iters)。

### 3.5 RB→AS (25ms) — 最快查询

| 步骤 | 详情 | 估算耗时 |
|------|------|---------|
| Dijkstra + extract | 278 pts, len=25.664 | ~2ms |
| **Pre-compete simplify** | 278→4 pts, **成功!** 25.664→7.472 (**-71%**) | ~8ms |
| RRT compete | 3/3 OK, direct=2.024 | ~2ms |
| └ Trials | 极快 (无单独日志) | |
| Greedy simplify | 4→4 pts | <1ms |
| Densify | 4→10 pts | <1ms |
| EB | 91 moves, 2.024→1.961 (-3.1%) | ~5ms |
| Final shortcut | SKIPPED (≤6 pts, S7-A) | — |
| **Total** | **25ms** | len=1.961, 6 pts |

### 3.6 Query 汇总

#### 各步骤耗时占比 (估算)

| 步骤 | AS→TS | TS→CS | CS→LB | LB→RB | RB→AS | 合计 | 占比 |
|------|-------|-------|-------|-------|-------|------|------|
| Dijkstra+extract | 2ms | 1ms | 2ms | 2ms | 2ms | 9ms | 0.3% |
| Pre-compete simplify | 5ms | 3ms | 5ms | 10ms | 8ms | 31ms | 1.0% |
| **RRT compete** | **250ms** | **700ms** | **1500ms** | 5ms | 2ms | **2,457ms** | **75.6%** |
| Validation+repair | 15ms | 3ms | 5ms | 2ms | 2ms | 27ms | 0.8% |
| Greedy simplify | 2ms | 1ms | 1ms | 1ms | <1ms | 5ms | 0.2% |
| Densify | <1ms | <1ms | <1ms | <1ms | <1ms | 2ms | <0.1% |
| **EB (+pass2)** | **30ms** | **50ms** | **180ms** | 10ms | 5ms | **275ms** | **8.5%** |
| Shortcut | 10ms | 5ms | 60ms | 5ms | — | 80ms | 2.5% |
| Other (overhead) | 14ms | 5ms | 335ms | 4ms | 6ms | 364ms | 11.2% |
| **Total** | **328ms** | **768ms** | **2,088ms** | **39ms** | **25ms** | **3,248ms** | **100%** |

> "Other" 包括 simplify_rrt (含在 RRT trial 内)、validation overhead、get() 同步等。

#### RRT compete 迭代数统计

| 查询 | Trial 1 iters | Trial 2 iters | Trial 3 iters | 成功数 | 估算 wall time |
|------|---------------|---------------|---------------|--------|---------------|
| AS→TS | 24,229 | 26,063 | 28,729 | 3/3 | ~250ms |
| TS→CS | 63,605 | 63,642 | **71,447** | 3/3 | ~700ms |
| **CS→LB** | timeout@53,640 | timeout@51,512 | **78,442** | **1/3** | **~1,500ms** |
| LB→RB | 87 | 97 | 360 | 3/3 | ~5ms |
| RB→AS | <100 | <100 | <100 | 3/3 | ~2ms |

**关键洞察**:
- **RRT compete 占 Query 75.6%** 是绝对主导瓶颈
- TS→CS 虽然 3/3 成功但 71K iters 消耗 700ms, 仅获得 16% 路径缩短
- CS→LB 的 main trial (78K iters at 5s budget) 耗时 ~1.5s 是 wall time 瓶颈
- LB→RB 和 RB→AS 因为 pre-compete simplify 成功，RRT 几乎瞬时

#### RRT tree 不平衡分析

| 查询 | tree_a avg | tree_b avg | 比例 | 含义 |
|------|-----------|-----------|------|------|
| AS→TS | 10,804 | 489 | **22:1** | AS 端 narrow passage |
| TS→CS | 12,277 | 411 | **30:1** | TS 端高维探索困难 |
| CS→LB | 502 | 24,506 | **1:49** | LB 端高维探索困难 |
| LB→RB | 33 | 35 | 1:1 | 两端容易到达 |
| RB→AS | — | — | ~1:1 | 极快 |

**发现**: tree 极度不平衡是高迭代数的根因。一侧快速膨胀说明该端 free space 宽阔但另一端被 narrow passage 阻挡。RRT-connect 算法在严重不平衡时退化为单侧搜索。

---

## 四、性能瓶颈排序 (当前)

| 排名 | 瓶颈 | 耗时 | 占 E2E 10.54s | 类型 | 优化潜力 |
|------|------|------|--------------|------|---------|
| 🥇 | **Grow (并行森林)** | 3,661ms | **34.7%** | Build | 低 (100% cache hit) |
| 🥈 | **Bridge (并行 RRT)** | 2,679ms | **25.4%** | Build | 中 (island #4 仍浪费) |
| 🥉 | **Query RRT compete** | ~2,457ms | **23.3%** | Query | **高** (CS→LB 1.5s) |
| 4 | Sweep coarsen | 433ms | 4.1% | Build | 低 |
| 5 | Adj+filter (post-greedy) | 336ms | 3.2% | Build | 低 |
| 6 | **Query EB + shortcut** | ~355ms | **3.4%** | Query | 中 |
| 7 | Greedy coarsen | 126ms | 1.2% | Build | 低 |
| 8 | Pre-compete simplify | 31ms | 0.3% | Query | 低 |

---

## 五、加速建议 (D-series)

### 🔴 D1: CS→LB 专项加速 — RRT warm-start with chain waypoints [预计 -500~800ms query]

**问题**: CS→LB 的 RRT compete 耗时 ~1.5s，是全部 query 的 46%。
根因: 78K iters 的 tree_a 仅 648 节点 (start 端极难扩展), tree_b 膨胀至 31320。
RRT-connect 在 tree 严重不平衡时退化为单侧搜索。

**方案**: 将 box-chain 路径的**关键中间 waypoints** 注入 RRT tree_a:
1. 从 617 pts chain 中等距采样 10-20 个 waypoints
2. 对每个 waypoint 做 `check_config` — 跳过碰撞的
3. 将 collision-free waypoints 作为 tree_a 的初始节点 (multi-root)
4. 为 RRT 跨越 narrow passage 提供 "stepping stones"

**原理**: chain 经过 box corridor, 虽然多数 waypoints 碰撞 (pre-compete simplify REVERTED),
但仍可能找到 ~5-10 个 collision-free 点, 分布在路径中段, 大幅缩小搜索空间。

**预计收益**: 主 trial 从 78K iters 降到 ~10-20K → RRT 时间 ~300ms → **query 节省 ~1.2s**。
保守估计 (chain waypoints 可用数有限) → **节省 500-800ms**。

**风险**: chain waypoints 可能误导 RRT 进入死胡同; 需修改 rrt_connect API。
**实施难度**: ⭐⭐⭐ (需修改 rrt_connect 支持多初始节点, 验证路径连通性)

### 🔴 D2: TS→CS RRT budget 降低 — chain/direct 差距小时减少搜索 [预计 -300~400ms query]

**问题**: TS→CS chain=7.406, direct=6.187, 仅差 **16%**。
但 RRT 做了 **71K iters, ~700ms** 来找到这条仅稍短的路径。

当 chain 已经很短且 ratio < 2 时, RRT 的边际收益极低:
- chain_len=7.406 已可直接使用
- direct=6.187 仅节省 1.2 rad
- **700ms 换取 1.2 rad 缩短，成本/收益比极差**

**方案A — 缩短 timeout** (推荐):
```cpp
// 当前: ratio < 2.0 → timeout=1000ms
// 改为: ratio < 1.5 → timeout=300ms, max_iters=20K (chain 已够好)
//        ratio < 2.0 → timeout=500ms (原 1000ms 的一半)
rrt_timeout = (ratio < 1.5) ? 300.0 : (ratio < 2.0) ? 500.0 
            : (ratio < 3.0) ? 3000.0 : 5000.0;
```

**方案B — 跳过 RRT compete** (激进):
当 `chain_len < 10.0 && ratio < 1.5` 时直接使用 chain 路径。
TS→CS 路径从 5.978 变为 7.406 (长 24%)。

**预计收益**: 方案A: TS→CS 768ms→~350ms (**-400ms**); 方案B: 768ms→~100ms
**风险**: 方案A 低风险 (RRT 仍会尝试, 只是 budget 更小); 方案B 路径质量下降
**实施难度**: ⭐

### 🟡 D3: EB 早期终止 — per-iteration improvement tracking [预计 -50~100ms query]

**问题**: CS→LB 的 EB 做了 1764 moves (60 outer iters × ~30 pts active)。
后期 iteration 改善极小但仍消耗碰撞检查 (每 move: check_config + 2× check_segment)。

**方案**: 追踪每轮 outer iteration 的路径长度改善, 当连续 N 轮改善 < 0.1% 时 break:
```cpp
double len_prev = sbf::path_length(path);
for (int iter = 0; iter < 60; ++iter) {
    // ... existing EB inner loop ...
    if (!any_move) break;
    if (iter >= 10) {  // 至少 10 轮保证初始收敛
        double len_cur = sbf::path_length(path);
        if ((len_prev - len_cur) / len_prev < 0.001) break;
        len_prev = len_cur;
    }
}
```

**预计收益**: CS→LB EB 从 60 轮可能减到 30-40 轮 → 节省 ~40-60ms, 其他 query 类似小幅节省。
**风险**: 低。EB 后期改善确实极小。已有 `if (!any_move) break` 但不够: 单个 move+即使改善 0.0001 仍算 any_move=true。
**实施难度**: ⭐

### 🟡 D4: Bridge island #4 探测降低 — tree_b 饱和快速判断 [预计 -200~400ms build]

**问题**: Island #4 Round 0 做了 3 pair × 4-5 并行 RRT = ~14 次 FAIL@50K iters。
tree_b 全部膨胀至 ~19.5K 节点, 说明 goal 空间已完全探索但路径确实不存在。

**方案A — 减少 consecutive_fails 阈值**: 3→2
```cpp
// 当前: const int max_consecutive_fails = (islands.size() <= 2) ? futures.size() : 3;
// 改为: ... : 2;
```
Island #4 的 pair 3 不会被尝试 → 省 ~4 次 50K FAIL = 少 200K CPU iters。

**方案B — 首次 FAIL 后降低后续 budget**:
如果 pair 1 的所有并行 RRT 的 tree_b avg > 15K 且全部 FAIL, 
将 pair 2/3 的 max_iters 从 50K 降至 **20K**。

**预计收益**: 方案A: ~200ms build; 方案B: ~400ms build
**风险**: 低。Island #4 最终靠 Round 1 post-adj 连通, 不靠 Round 0 RRT。
**实施难度**: ⭐

### 🟡 D5: RRT compete 减少到 2 trials [预计 -50~150ms query]

**问题**: 3 trials (1 main + 2 bonus, 全部并行) 中:
- CS→LB: 2 bonus 全 timeout, 无贡献
- TS→CS: 3 trials 结果接近, best 仅比 worst 好 ~5%
- LB→RB/RB→AS: 极快, 差异可忽略

**方案**: 减到 2 trials (1 main + 1 bonus):
```cpp
auto f1 = std::async(std::launch::async, run_trial, rrt_timeout, 200000, 42);
auto f2 = std::async(std::launch::async, run_trial, bonus_timeout, 80000, 179);
// 删除 f3
```

**预计收益**: ~50-150ms (减少 1 个 simplify_rrt 和线程调度)
**风险**: 路径质量可能微降 (从 "best of 3" 变 "best of 2")。
AS→TS 当前 best=Trial 3 (61wp, len=6.116), 删掉 Trial 3 后 best 变 Trial 1 (84wp, 可能稍长)。
**实施难度**: ⭐

### 🟢 D6: Grow 精简 — 5 trees → 4 trees 或减少 box target [预计 -700ms build]

**问题**: 5 trees × 5000 boxes = 25000 raw boxes → sweep 后仅 10269 → greedy 后 8883。
**58%** 的 boxes 被 sweep 淘汰, 说明存在大量冗余。

**方案**: 减少到 4 trees × 5000 或 5 trees × 4000, 测试覆盖质量是否保持。

**预计收益**: ~700ms grow (proportional to boxes)
**风险**: **高**。5 trees 的覆盖能力是 island 连通的基础, 减少 trees 可能导致 islands 增加、bridge 时间增加, 净效果可能为负。
**实施难度**: ⭐ (但需仔细验证)

### 🟢 D7: 加 query 子阶段 chrono 计时 [工具项, 无性能收益]

**问题**: query 子阶段耗时全靠日志推断 (RRT iters, EB moves), 不够精确。

**方案**: 在 `query()` 函数中用 `chrono::steady_clock` 在每个子阶段打点:
```
[QRY-T] dijkstra=0.8ms extract=1.2ms pre_simplify=4.5ms rrt_compete=702.3ms
        validate=3.1ms greedy=1.0ms densify=0.2ms eb=48.7ms shortcut=5.3ms
        pass2=12.1ms final_val=2.8ms total=780.1ms
```

**预计收益**: **0ms** (纯诊断工具)。但为后续所有 D-series 优化提供精确数据基础。
**实施难度**: ⭐

---

## 六、D-series 优先级排序

| 优先级 | 编号 | 操作 | 预计收益 | 难度 | 风险 |
|--------|------|------|---------|------|------|
| **P0** | **D7** | **加 chrono 计时** | 诊断 | ⭐ | 无 |
| **P0** | **D2** | **TS→CS RRT budget 降低** | **-300~400ms query** | ⭐ | 低 |
| **P1** | **D1** | **CS→LB warm-start RRT** | **-500~800ms query** | ⭐⭐⭐ | 中 |
| P1 | D3 | EB 早期终止 | -50~100ms query | ⭐ | 低 |
| P1 | D4 | Bridge island 快速放弃 | -200~400ms build | ⭐ | 低 |
| P2 | D5 | RRT 3→2 trials | -50~150ms query | ⭐ | 中 |
| P3 | D6 | Grow tree 精简 | -700ms build | ⭐ | **高** |

**建议执行路径**: D7 → D2 → D3 → D4 → D1 → D5

**D-series 实际结果**:
- Build: 7.29s → ~7.27s (D4 -196ms, 被 variance 掩盖)
- Query: 3.25s → ~3.33s (D3 -58ms, D7 诊断开销 ~5ms)
- **Total: 10.54s → ~10.60s (variance 范围内, 净收益 ~0%)**
- **根因**: RRT compete = 93.8%, 是算法性能墙, 非参数可调

---

## 七、全部优化历史

| 阶段 | Build | Query(5) | Total | vs 原始 |
|------|-------|----------|-------|---------|
| 原始基线 | 24.97s | 10.59s | 35.56s | — |
| +T-series | 13.3s | ~5.1s | ~18.4s | -48% |
| +B-series | 12.47s | 3.62s | 16.09s | -55% |
| +C-series | 7.29s | 3.25s | 10.54s | -70% |
| **+D-series** | **7.27s** | **3.33s** | **10.60s** | **-70%** |

### 各优化详情

| 编号 | 优化项 | 类型 | 效果 | 状态 |
|------|--------|------|------|------|
| T1 | Sweep-line adjacency | Build | adj 3.5s→0.3s | ✅ |
| T7 | Sweep mark-compact | Build | sweep 0.8s→0.4s | ✅ |
| B1 | 碰撞检测栈分配 | Both | 消除 63 heap alloc/check | ✅ |
| B2 | Point FK 快速路径 | Both | check_config 7× FLOP 减少 | ✅ |
| B3 | Bridge seg_res 20→10 | Build | bridge 碰撞检查减半 | ✅ |
| B6 | 渐进超时+cancel | Build | R0 timeout 分级 + cancel flag | ✅ |
| C1 | Query seg_res 20→10 | Query | ❌ RRT compete 路径质量退化 | ❌ |
| C2 | ares 0.005→0.01 | Query | 路径微提升, 无显著时间改善 | ✅ |
| C3 | RRT compete shared cancel | Query | ❌ cancel 杀好 trial, 路径变差 | ❌ |
| C4 | Bridge timeout 缩短 | Build | bridge 7.1s→2.7s (-62%) | ✅ |
| C5 | Pre-compete quick check | Query | ❌ 未触发, S5-A 已足够 | ❌ |
| D7 | Query chrono 计时 | Query | 诊断工具, RRT=93.8% | ✅ |
| D2 | TS→CS RRT budget 缩减 | Query | ❌ 所有 ratio>4, tier 无法触发 | ❌ |
| D3 | EB 早期终止 | Query | EB 81→23ms (-58ms) | ✅ |
| D4 | Bridge 快速放弃 | Build | consec_fails 3→2, bridge -196ms | ✅ |
| D1 | CS→LB warm-start RRT | Query | ❌ 破坏 RRT 探索拓扑 | ❌ |

### 路径质量 (始终不变)

```
AS→TS: 5.065    TS→CS: 5.978    CS→LB: 8.929
LB→RB: 4.790    RB→AS: 1.961
5/5 queries collision-free ✅
boxes=9098  islands=1
```

---

## 八、关键指标总结 (当前 — D-series 完成后)

```
Build:    7.27s   (grow=50.2%, bridge=34.1%, coarsen=8.7%, other=7.0%)
Query:    3.33s   (RRT compete≈93.8%, EB≈0.7%, shortcut≈1.1%, other≈4.4%)
Boxes:    9,098   (25,086 raw → sweep 10,269 → greedy 8,883 → filter 8,720 + bridge 378)
Islands:  1       (5→2 via bridge → 1 via R1)
Success:  100%    (5/5 queries, all collision-free)
Total:    10.60s  (-70% vs 原始 35.56s)
```

### D7 精确 Query 计时 (chrono, 毫秒)

RRT compete 是绝对瓶颈 (3079ms / 3281ms = 93.8%):
- CS→LB: 1946ms (59% of total query) — tree 不平衡 (tree_a=648 vs tree_b=31320)
- TS→CS: 822ms (25%) — 3 trials 均 ~70K iters
- AS→TS: 307ms (9%) — 3 trials 均 ~26K iters
- LB→RB + RB→AS: 4ms (<0.2%) — pre-simplify 成功, RRT 瞬时

**性能墙**: RRT compete 无法通过 timeout/budget/warm-start 优化。
需要算法级变更 (Informed RRT*, BIT*, 或 optimization-based) 才能突破 3s query 墙。
