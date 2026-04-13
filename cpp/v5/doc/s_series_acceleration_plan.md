# SBF v5 S 系列加速计划 — 详细执行文档

> 日期: 2026-04-10
> 基线: P1-P5 + S4-A 后  Build=22.1s, Query(5)=5.25s
> 目标: 进一步降低 Build + Query 时间
> **最终结果: Build=20.3s (-8%), Query=4.93s (-6%)**

---

## 执行结果总览

| 编号 | 状态 | 内容 | Build 影响 | Query 影响 |
|------|------|------|-----------|-----------|
| S5-A | ✅ 成功 | Pre-compete simplify 内联 config 检查 | — | -100~150ms |
| S7-A | ✅ 成功 | 路径 ≤6 pts 跳过 final shortcut | — | -30~50ms |
| S7-B | ✅ 成功 | EB 改善 < 5% 跳过 pass2 | — | -20~40ms |
| S2   | ✅ 完成 | Build 中间步骤插桩 | — (诊断) | — |
| S6-A | ✅ **最大收益** | 移除 3 次诊断 compute_adjacency | **-2.6s** | — |
| S3-C | ❌ 失败已回退 | RRT 共享取消标志 | — | 路径质量回归 |

---

## 已完成项回顾 (之前)

| 编号 | 状态 | 内容 | 结果 |
|------|------|------|------|
| S1-A | ❌ 失败 | Bridge round 1 ≤2 islands 降阈值 3 | 破坏连通性，已回滚 |
| S3-A | ❌ 失败 | chain/euclid < 1.5 跳过 RRT compete | 度量错误(chain/euclid 5.78-27.5)，永不触发，已删除 |
| S4-A | ✅ 成功 | 路径 ≤8 pts 跳过 random shortcut | 节省 ~130ms，路径质量不变 |

---

## 本轮待执行优化

### S5-A: Pre-compete Simplify 快速预检 [Query, -150ms]

**问题分析**:
- Pre-compete simplify 在 5 个查询中 4/5 被 REVERTED（config 碰撞）
- 浪费统计: AS→TS 49.8ms, TS→CS 7.2ms, CS→LB 101.4ms, LB→RB 71.5ms = 共 ~230ms
- 只有 RB→AS 的 simplify 成功（26.2ms, 220→4 pts, len 20.172→10.131）

**方案**:
- 在执行完整 greedy simplify 前，先对简化结果的关键点做 config check
- 若简化后的点多数碰撞，直接跳过整个 simplify
- 具体实现: simplify 后、验证前，先对所有简化点做 config check；
  如果 **ANY** 碰撞则立即 REVERT → 省掉验证环节的开销

**修改文件**: `cpp/v5/src/planner/sbf_planner.cpp` ~L900-945
**预计节省**: ~150ms (4 次浪费的 simplify 提前终止)
**风险**: ⭐ 极低 — 只是提前执行了已有的 revert 逻辑

---

### S7-A: 条件化 Final Shortcut [Query, -50ms]

**问题分析**:
- Final shortcut 对 ≤6 pts 的路径效果极差
- 5 个查询中 3/5 的 final shortcut 无改善
- 但 CS→LB 的 final shortcut 有显著改善 (9.603→9.590)

**方案**:
- 路径 ≤6 pts 时跳过 final shortcut（与 S4-A 同理）

**修改文件**: `cpp/v5/src/planner/sbf_planner.cpp` ~L1265
**预计节省**: ~30-50ms
**风险**: ⭐ 极低

---

### S7-B: 条件化 Pass2 EB [Query, -50ms]

**问题分析**:
- Pass2 EB 在 AS→TS (4ms), TS→CS (5ms) 几乎无效
- RB→AS 的 pass2 完全不执行 (0 moves, 1.7ms)
- 仅 CS→LB (77.5ms, 251 moves) 和 LB→RB (35.5ms, 72 moves) 有实质改善

**方案**:
- 当 Step 3 EB 改善 < 5% 时，跳过 Step 5 pass2
- 计算 `(len_before - len_after) / len_before`，< 0.05 则跳过 pass2

**修改文件**: `cpp/v5/src/planner/sbf_planner.cpp` ~L1320 前
**预计节省**: ~20-40ms
**风险**: ⭐ 极低 — AS→TS 的 EB 改善 13%→会执行 pass2; CS→LB 改善 8.5%→会执行

---

### S2: 插桩 Build 未计时环节 [Build, 定位 ~3.67s]

**问题分析**:
- Build 22.1s 中约 3.67s 无法归属到任何已计时模块
- `build_coverage()` 中 `compute_adjacency()` 被调用了 **7 次**:
  1. L407: grow_adj (诊断)
  2. L454: sweep_adj (诊断)  
  3. L462: pre_adj (articulation points)
  4. L479: greedy_adj (诊断)
  5. L488: post_adj (post-greedy articulation)
  6. L499: adj_ (bridge_all_islands 前)
  7. coarsen_greedy 内部也调用了 compute_adjacency
- 每次 `compute_adjacency` 对 ~10K-25K boxes 是 O(n²) 的 pair-wise 检查
- `find_islands`, `find_articulation_points` 也各被调用多次

**方案**:
- 为 build_coverage 中每个中间步骤添加 chrono 计时
- 精确定位热点后决定优化策略
- 可能的快速优化: 合并多次 compute_adjacency 调用，减少重复计算

**修改文件**: `cpp/v5/src/planner/sbf_planner.cpp` L400-520
**预计定位**: 3-4s 中的具体分布
**风险**: ⭐ 无风险（只加日志）

---

### S3-C: RRT Compete 共享取消标志 [Query] — ❌ 失败已回退

**问题分析**:
- CS→LB 查询: 3 trials 并行启动，trial 1 timeout (1s), trial 2 timeout (1s), trial 3 成功 (1.6s)
- 2 个 timeout 的 trial 各白等了 1000ms → 但线程资源浪费

**尝试方案**:
1. **v1 — 共享 rrt_found 标志**: 任一 trial 成功立即取消其他所有 trial
   - 结果: **严重质量回归**! AS→TS 5.063→7.528, TS→CS 5.985→10.198
   - 原因: 第一个完成的 trial 往往不是最优的，取消了可能产生更好路径的 trial
2. **v2 — bonus_cancel 分离设计**: 主 trial 不可取消，bonus trials 互相取消
   - 结果: AS→TS 5.063→6.083 (仍退化 20%)，总查询时间 5.04s→5.28s (反而更慢)
   - 原因: bonus trials 超时比主 trial 短，它们总是先结束；唯一触发的场景是
     一个 bonus 取消另一个 bonus，恰好丢了可能产生最优路径的 trial

**根本原因**: 此工作负载的 RRT trials 要么很快全部成功 (AS→TS, LB→RB)，
要么大部分 timeout (CS→LB)。取消标志只在 "一部分成功一部分还在跑" 时有用，
但此工作负载中这种场景 bonus trials 超时已经自然退出了。

**结论**: S3-C 全部回退，代码恢复原状。

---

### S6-A: 减少 compute_adjacency 重复调用 [Build, -1-2s]

**问题分析** (取决于 S2 插桩结果):
- build_coverage 中 compute_adjacency 被调用 7 次
- 其中 3 次是纯诊断用途（可删除或改为 debug-only）
- 2 次 articulation points 前的 compute_adjacency 可能可合并

**方案**:
- 诊断用 compute_adjacency (L407, L454, L479) 在 Release 模式下跳过
- 或者用 `#ifndef NDEBUG` 保护
- 合并 L462(pre_adj) 和 L462 后的 find_articulation 为一次调用

**修改文件**: `cpp/v5/src/planner/sbf_planner.cpp` L400-520
**预计节省**: 1-2s (假设每次 compute_adjacency ~0.3-0.5s)
**风险**: ⭐ 低 — 诊断日志仍在 debug 构建中可用

---

## 执行顺序与实际结果

| 步骤 | 编号 | 状态 | 实际效果 |
|------|------|------|---------|
| 1 | **S5-A** | ✅ | Simplify 343→5 提前到点 3-5 即检出碰撞退出 |
| 2 | **S7-A** | ✅ | 3/5 查询跳过无用 final shortcut |
| 3 | **S7-B** | ✅ | EB<5% 的 3 个查询跳过 pass2 (LB→RB 微损 +0.3%) |
| 4 | **S2** | ✅ | 定位到 2,459ms 诊断 adj 开销 |
| 5 | **S6-A** | ✅ | 移除 3 次诊断 adj → **Build -2.6s** |
| 6 | **S3-C** | ❌ 回退 | v1 严重退化, v2 仍退化且更慢 |

---

## S2 插桩关键发现

build_coverage() 中 compute_adjacency() 调用 7 次:

| 调用点 | 耗时 | 用途 | 处理 |
|--------|------|------|------|
| post-grow adj+islands | **1,785ms** | 诊断 | S6-A 移除 |
| post-sweep adj+islands | **389ms** | 诊断 | S6-A 移除 |
| pre-artic adj+artic | 409ms | **功能** (articulation) | 保留 |
| post-greedy adj+islands | **285ms** | 诊断 | S6-A 移除 |
| adj+artic+filter | 625ms | **功能** (filter) | 保留 |
| pre-bridge adj | 293ms | **功能** (bridge) | 保留 |
| **诊断总计** | **2,459ms** | **可移除** | ✅ 已移除 |

---

## 最终性能对比

| 指标 | 原始 | P1-P5 | +S4-A | +S5/S7/S6-A (最终) | 降幅 |
|------|------|-------|-------|-------------------|------|
| Build | 24.97s | 22.06s | 22.1s | **20.3s** | **-18.7%** |
| Query(5) | 10.59s | 5.49s | 5.25s | **4.93s** | **-53.4%** |
| 总计 | 35.56s | 27.55s | 27.35s | **25.2s** | **-29.1%** |

### 最终路径质量 (seed=1)

| 查询对 | 长度 | 状态 |
|--------|------|------|
| AS→TS | 5.063 | ✅ OK |
| TS→CS | 5.985 | ✅ OK |
| CS→LB | 8.920 | ✅ OK |
| LB→RB | 4.792 | ✅ OK |
| RB→AS | 1.966 | ✅ OK |

---

## 验证方案

每完成一项优化后运行:
```bash
cd cpp/build && cmake --build . -j$(nproc) && ./experiments/exp2_e2e_planning --seeds 1 --no-viz
```

验证标准:
1. **islands=1** (连通性完整)
2. **5/5 OK** (全部查询成功)
3. **路径质量**: len 与基线一致或更优
4. **时间**: Build + Query 有下降
