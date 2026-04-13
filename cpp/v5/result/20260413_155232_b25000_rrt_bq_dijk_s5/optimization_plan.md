# Exp2 优化方案

## 当前基准 (2026-04-13, 5 seeds, Dijkstra, build+query)

| 指标 | 值 |
|------|-----|
| 成功率 | **100%** (25/25, 0 碰撞) |
| Build 中位数 | 9.91s (mean=9.99s) |
| Growth | 4.8–5.2s (25000 boxes → 1 component) |
| LECT cache | 100% hit, 0 miss |
| 岛屿数 (med) | **87** (范围 32–130) |
| Final boxes (med) | 6549 (范围 6096–7624) |
| Query 中位数 | 0.026–0.159s (正常 seed), **Seed3 异常 0.9–2.6s** |

### 瓶颈分析

| 阶段 | 耗时 | 问题 |
|------|------|------|
| Growth | ~5s | 正常，LECT 全缓存 |
| Coarsen1 (sweep+relaxed+artic+greedy+cluster) | ~2.5s | 合理 |
| Bridge | **1.1–2.4s** | 连接 top-10 岛，但仍留 32–130 岛 |
| Coarsen2 | ~1s | 合理 |
| **Query-time bridge (seed3)** | **0.5–2.6s/query** | S/G 不在主岛 → 运行时建桥 |

---

## 优化方案

### OP-1: Bridge 后增加针对 query endpoints 的保障桥接 [P0]

**问题**: build_coverage 的 bridge_all_islands 只连接 top-10 大岛到主岛，
不保证 query endpoints 所在的小岛被连接。Seed3 主岛碎片化(3272)，
导致 query 时花 0.5–2.6s 做 bridge_s_t。

**方案**: 在 build_coverage 的 bridge_all 之后，检查每个 seed_point 所在岛
是否属于主岛。若不在，为该 seed_point 做一次 bridge_s_t(该点→主岛)。

**改动位置**: `sbf_planner.cpp` build_coverage(), bridge_all 之后、coarsen2 之前

**预期效果**: 消除 query-time bridge，seed3 的 query 时间从 5.25s → <0.5s

---

### OP-2: 增加 bridge 总预算到 3000 [P0]

**问题**: 当前 max_total_bridges=2000，但很多岛仍未连接(81–159→71–130)。

**方案**: 将 max_total_bridges 从 2000 提升到 3000。

**改动位置**: `sbf_planner.cpp` L549 bridge_all_islands 调用

**预期效果**: 更多岛被连接，最终岛数从 87 降到 ~40

---

### OP-3: Coarsen1 sweep 保守化 — 保护小体积 box [P1]

**问题**: sweep1 从 25000→13000 删除了 48% boxes，过于激进。
在狭窄通道处可能删掉关键的小 box，导致碎片化。

**方案**: 在 relaxed_sweep 中，对体积低于中位数 1/4 的 box 提高保护权重。

**改动位置**: `coarsen.cpp` coarsen_sweep_relaxed()

**预期效果**: 减少碎片化产生的根源，岛数降低 20-30%

---

### OP-4: TS→CS 路径优化 — 增加 elastic band 迭代 [P2]

**问题**: TS→CS 路径长度方差最大(5.1 vs 6.5)，elastic band 只做 11 iter。

**方案**: 当 RRT compete 保留 chain 路径时(说明直连路径更长),
增加 EB 的 max_iters 到 30。

**改动位置**: `sbf_planner.cpp` query() 中 optimize_path 调用

**预期效果**: TS→CS len_med 从 5.1 降到 ~4.5

---

## 执行顺序

1. OP-1 + OP-2 (一起做，两者相关)
2. 重新跑 exp2 --seeds 5 验证
3. 如果岛数仍 >50，做 OP-3
4. OP-4 路径质量
5. 最终 10-seed 验证
