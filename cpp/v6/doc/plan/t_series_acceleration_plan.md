# SBF v5 T 系列加速计划 — 详细执行文档

> 日期: 2026-04-10
> 基线: P1-P5 + S 系列全部完成后  Build=20.3s, Query(5)=4.93s
> 目标: 进一步降低 Build 时间 (尤其是 adjacency + coarsen 环节)
> **最终结果: Build ≈13.3s (-35%), Query ≈5.1s (~持平)**

---

## 执行结果总览

| 编号 | 状态 | 内容 | Build 影响 | Query 影响 |
|------|------|------|-----------|-----------|
| T1   | ✅ **最大收益** | `compute_adjacency` sweep-line O(N²)→O(N log N) | **adj -93%, greedy -93%** | — |
| T7   | ✅ 成功 | `coarsen_forest` mark-and-compact 替代 vector::erase | **sweep -56%** | — |
| T2   | ❌ 已回退 | Bridge timeout 逐轮递减 | 破坏连通性(islands=2) | 严重回归 |
| T4   | ❌ 已回退 | EB 内部使用粗分辨率碰撞检查 | — | 路径质量回归 |
| T5   | ⏭ 跳过 | 高 ratio 跳过 pre-compete simplify | — | 收益 <50ms |
| T3   | ⏭ 跳过 | Greedy incremental adjacency | — | greedy 已 130ms |
| T6   | ⏭ 跳过 | Sweep parallel sort | — | sweep 已 400ms |

---

## T1: compute_adjacency Sweep-Line 算法 [Build, -93% adj]

### 问题分析
- `compute_adjacency()` 使用 O(N²·D) 双循环遍历所有 box 对
- 约 8,720 个 box: 8720² × 4dim ≈ 3 亿次比较
- 每次 `shared_face()` 调用分配 `SharedFace` 结构 + 检查 D 维
- Build 中被调用 4 次: adj+artic, greedy (内部每轮), adj+filter, pre-bridge

### 实现方案: 1D Sweep-Line

**算法**:
1. **选择最紧维度**: 计算每个维度的 `sum(hi - lo)`, 选 span 最小的维度作为 sweep 轴
2. **排序**: 按 `lo[sweep_dim]` 排序 (O(N log N))
3. **扫描**: 对每个 box i, 只与后续 box j 比较, 当 `lo_j > hi_i + tol` 时 break
4. **内联邻接判断**: 替代独立 `shared_face()` + `boxes_overlap()`, 在单次 D 维循环中同时判断:
   - `separated` (两维完全分开) → 跳过
   - `all_strict_overlap` (所有维度严格重叠) → overlap 邻接
   - `n_touching` + `n_overlapping` → shared-face 邻接条件

**复杂度**: O(N log N + K·D), 其中 K 为 sweep 窗口内的候选对数 (远小于 N²)

**修改文件**: `cpp/v5/src/forest/adjacency.cpp`
- 添加 `#include <numeric>` (用于 `std::iota`)
- 完全重写 `compute_adjacency()` 函数 (~80 行 → ~90 行)
- `shared_face()` 和 `find_articulation_points()` 不变

### 验证结果

| 步骤 | T1 前 | T1 后 | 加速比 |
|------|-------|-------|--------|
| adj+artic | 404ms | **28ms** | **14.4×** |
| greedy coarsen | 1,752ms | **130ms** | **13.5×** |
| adj+artic+filter | 604ms | **336ms** | **1.8×** |
| pre-bridge adj | 287ms | **17ms** | **16.9×** |
| **总 Build** | **20.3s** | **~16.5s** | **-19%** |

路径质量: 5.063 / 5.985 / 8.920 / 4.792 / 1.966 ✅ 完全一致

---

## T7: coarsen_forest Mark-and-Compact [Build, sweep -56%]

### 问题分析
- `coarsen_forest()` (sweep 阶段) 在内层循环中调用 `boxes.erase()`
- `vector::erase()` 需要移动后续所有元素, 每次 O(N)
- 约 15,000 次合并 × 平均 O(10,000) 移动 = ~1.5 亿次元素移动

### 实现方案: Mark-and-Compact

替代方案:
1. 合并 box j 到 i 时, 将 j 的 `volume` 设为 -1.0 (标记死亡), O(1)
2. 扫描时跳过 `volume < 0` 的死亡 box
3. 每个维度处理完后, 用 `std::remove_if` + `erase` 一次性压缩

**复杂度**: 合并操作从 O(K·N) 降为 O(K + D·N), 其中 K=合并次数, D=维度数

**修改文件**: `cpp/v5/src/forest/coarsen.cpp`
- `coarsen_forest()` 函数内:
  - `boxes.erase()` → `boxes[j].volume = -1.0;`
  - 添加维度间 compact: `boxes.erase(std::remove_if(...), boxes.end())`
  - 添加轮次末 compact
  - 扫描时跳过 `volume < 0` 的 box

### 验证结果

| 指标 | T7 前 | T7 后 | 改善 |
|------|-------|-------|------|
| sweep 耗时 | ~900ms | **~400ms** | **-56%** |
| sweep 结果 | 25086→10270 | 25086→10270 | 完全一致 |

路径质量: 完全一致 ✅

---

## T2: Bridge Timeout 递减 [已回退]

### 方案
Round 0 保持原始 5000ms/100k iters, Round 1 降为 3000ms/60k, Round 2+ 降为 2000ms/40k

### 失败原因
- Round 1 有一个关键桥接 pair 在 ~98k iters 才成功连接
- 60k iters 上限导致该 pair 失败,最终 islands=2
- 保守版本 (仅 Round 2+ 递减) 则无任何效果——Round 2+ 从未被触发

### 结论: 当前场景的 bridge 预算恰好在临界点, 不可压缩

---

## T4: EB 粗分辨率碰撞检查 [已回退]

### 方案
EB 内部 `ares` 从 `max(20, len/0.005)` 改为 `max(10, len/0.02)` (4× 粗)
EB 结束后仍有全精度验证 (per-segment revert)

### 失败原因
- 4D KUKA 场景碰撞间隙极小, 粗检查遗漏真实碰撞
- Per-segment revert 频繁触发 (4/5 查询有 revert)
- 路径质量严重回归: CS→LB 从 8.920 升到 9.732 (+9%)
- 即使改用 2× 粗 (0.01 步长) 也同样回归

### 结论: 该机器人场景的碰撞精度不可降低

---

## 最终性能对比

| 指标 | 原始 | P1-P5 | +S 系列 | **+T 系列 (最终)** | 总降幅 |
|------|------|-------|---------|-------------------|--------|
| Build | 24.97s | 22.06s | 20.3s | **~13.3s** | **-47%** |
| Query(5) | 10.59s | 5.49s | 4.93s | **~5.1s** | **-52%** |
| 总计 | 35.56s | 27.55s | 25.2s | **~18.4s** | **-48%** |

### Build 时间分解 (T 系列后, 典型值)

| 步骤 | 耗时 | 占比 | 备注 |
|------|------|------|------|
| grow | ~3.9s | 29% | 5000×5=25000 boxes, 5 线程并行 |
| sweep | ~0.4s | 3% | T7 mark-and-compact |
| adj+artic | ~25ms | <1% | T1 sweep-line |
| greedy | ~120ms | 1% | T1 间接加速 |
| adj+filter | ~330ms | 2% | T1 加速 |
| pre-bridge adj | ~17ms | <1% | T1 加速 |
| **bridge** | **~8.5-11s** | **65%** | **瓶颈: RRT 随机性, C-space 拓扑限制** |

### 最终路径质量 (seed=1)

| 查询对 | 长度 | 状态 |
|--------|------|------|
| AS→TS | 5.063 | ✅ OK |
| TS→CS | 5.985 | ✅ OK |
| CS→LB | 8.920 | ✅ OK |
| LB→RB | 4.792 | ✅ OK |
| RB→AS | 1.966 | ✅ OK |

---

## 后续优化建议

当前 Bridge RRT 占 Build 的 65%, 是绝对瓶颈。可能的方向:
1. **并行化 bridge 策略**: 同时尝试多组 seed → 选最快连通的
2. **Grow 阶段增加 cross-seed 预算**: 让 grow 阶段生长更多跨种子 box 减少 bridge 需求
3. **更智能的 bridge 候选选择**: 使用 kd-tree 或 R-tree 选择更好的 box 对
4. **Query RRT 加速**: 利用 box 图的几何信息引导 RRT (e.g., 在 box 链中心初始化)
