# Phase 5: ForestGrower 性能优化计划 ✅ 已完成

> 基于 Phase 1-4 完成后的管线瓶颈分析（Section 7），选取 **低难度 + 高收益** 的优化项优先实施。
> **完成日期: 2026-03-17**

---

## 0. 性能基线与最终结果

| 指标 | Phase 4 基线 (串行) | Phase 4 基线 (并行) | Phase 5 最终 (并行) |
|---|---|---|---|
| 总耗时 | 165 ms | 60 ms | **48 ms** |
| 加速比 | 1.0× | 2.83× | **3.41×** |
| box 利用率 | — | ~70% | **93%** (work-stealing) |
| box 查找 | O(N) 线性 | O(N) 线性 | **O(1)** hashmap |

---

## 1. 优化项清单

### 1.1 ✅ Box ID → 索引 O(1) 查找

**问题**：`grow_wavefront()` 和 `grow_rrt()` 中通过 `for (const auto& b : boxes_) { if (b.id == entry.box_id) }` 线性查找 box，O(N) 复杂度。

**方案**：
- 在 `ForestGrower` 中维护 `std::unordered_map<int, int> box_id_to_idx_`（box_id → boxes_ 中的下标）
- `try_create_box` 成功时自动更新此映射
- 所有 `for + if (b.id == ...)` 替换为 O(1) 查表

**改动文件**：
- `include/sbf/forest/forest_grower.h` — 新增 `box_id_to_idx_` 成员
- `src/forest/forest_grower.cpp` — 修改 `try_create_box` / `grow_wavefront` / `grow_rrt` / `promote_all`

**预期收益**：消除扩展循环中的 O(N²) 瓶颈 → 大 box 数量时提速 2-5×  
**难度**：★☆☆ (低)

---

### 1.2 ✅ Work-Stealing 动态负载均衡

**问题**：`grow_parallel()` 中 budget 静态均分，靠近障碍的子树 FFB 失败多、提前停止，导致 worker 利用率不均（~70%）。

**方案**：
- 引入 `std::shared_ptr<std::atomic<int>>` 全局已用 box 计数器
- 每个 worker 通过 `fetch_add(1)` 递增，直到 `total >= max_boxes`
- 移除 `budget_per_root` 静态分配

**改动文件**：
- `include/sbf/forest/forest_grower.h` — `grow_subtree` 新增 `shared_counter` 参数
- `src/forest/forest_grower.cpp` — 修改 `grow_parallel` / `grow_subtree` / `grow_wavefront` / `grow_rrt`

**预期收益**：worker 利用率 70% → 90%，并行加速比 2.83× → 3.2×  
**难度**：★☆☆ (低)

---

### 1.3 ✅ 跨子树边界桥接（可配置，默认关闭）

**问题**：并行 worker 各管各的子树区域，合并后在子树边界处可能缺少邻接连接，增加 connected components 数量。

**方案**：
- 在 `grow_parallel` 合并阶段完成后，增加 **boundary bridging** 步骤
- 对每对相邻子树区域的共享面，在面上采样 seed
- 在共享面的 epsilon 邻域内尝试 `try_create_box`，生成 bridge box
- 新函数 `bridge_subtree_boundaries()`

**改动文件**：
- `include/sbf/forest/forest_grower.h` — 新增 `bridge_subtree_boundaries()` 声明
- `src/forest/forest_grower.cpp` — 实现桥接逻辑

**预期收益**：连通分量减少 30-50%，start-goal 连通率提升  
**难度**：★☆☆ (低)

---

### 1.4 ✅ 自适应 min_edge 两阶段策略

**问题**：固定 `min_edge = 0.01` 在密集障碍区浪费时间生成微小 box，在空旷区域不够积极。

**方案**：
- 新增 `GrowerConfig` 字段：`adaptive_min_edge = true`, `coarse_min_edge = 0.1`, `coarse_fraction = 0.6`
- Phase 1 (前 60% budget)：使用 `coarse_min_edge` 快速铺满
- Phase 2 (后 40% budget)：切换到 `min_edge` 精细填充
- `grow_wavefront` / `grow_rrt` 中根据当前 box 数 / max_boxes 比例切换

**改动文件**：
- `include/sbf/forest/grower_config.h` — 3 个新字段
- `src/forest/forest_grower.cpp` — 修改扩展循环中的 min_edge 选择逻辑

**预期收益**：相同时间内覆盖率 +15-25%，box 分布更均匀  
**难度**：★☆☆ (低)

---

## 2. 实施顺序

```
Step 1: Box ID O(1) 查找    ← 最简单，零风险，所有模式受益
Step 2: Work-stealing        ← 仅改并行路径
Step 3: 边界桥接             ← 仅改 grow_parallel 后处理
Step 4: 自适应 min_edge      ← 改扩展循环，影响 serial + parallel
Step 5: 新增/更新测试         ← 验证所有优化
Step 6: Benchmark 验证        ← 前后对比
```

---

## 3. 成功标准

| 指标 | Phase 4 基线 | Phase 5 目标 | **Phase 5 实测** |
|---|---|---|---|
| 并行加速比 (4 threads) | 2.83× | ≥ 3.2× | **3.41×** ✅ |
| Worker 利用率 | ~70% | ≥ 85% | **93%** ✅ |
| 连通分量数 (200 boxes) | ~5-8 | ≤ 3 | 4 (串行同等) |
| 大规模 box 查找 | O(N²) 线性扫描 | O(N) hashmap | **O(1)** ✅ |
| 测试通过 | 10/10 | 全部通过 + 新增 Phase 5 测试 | **15/15** ✅ |

> **注**: 边界桥接因 LECT 树不一致导致 bridge box 无法与 worker box 对齐面邻接，
> 当前以可配置功能保留（`bridge_samples=0` 默认关闭），后续需合并 LECT 后重新启用。

---

## 4. 不在本阶段实施

| 优化项 | 原因 |
|---|---|
| KD-tree `find_nearest_box` | 需引入 nanoflann 依赖，留到 Phase 6 |
| SIMD 邻接内核 | 中等难度，需要数据布局重排 |
| LECT warm-start | 高难度，需 COW/snapshot 机制 |
| GPU 碰撞检测 | 超范围 |
