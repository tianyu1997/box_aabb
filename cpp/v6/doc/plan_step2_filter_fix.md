# Step 2: 修复 Filter 导致的岛屿碎片化（Issue 2）

## 问题描述

`build_coverage()` 中的 box 处理流程为：

```
grow_rrt → 5 islands → coarsen_greedy → filter_coarsen_overlaps → compute_adjacency
```

grow_rrt 后有 5 个 islands（每棵树 1 个），但 `filter_coarsen_overlaps` 执行后变成 **10 个 islands**。

最终岛屿大小分布：

| 排名 | box 数 |
|------|--------|
| 1 | 1370 |
| 2 | 220 |
| 3 | 160 |
| 4 | 46 |
| 5 | 27 |
| 6 | 17 |
| 7 | 2 |
| 8-10 | 1 each |

本来 5 棵树就有 5 个 islands，但 filter 后反而多出了 5 个碎片小岛。

## 根因分析

文件：`src/forest/coarsen.cpp`，`filter_coarsen_overlaps()` 方法（约 L302-L400）

### Pass 1（无问题）

```cpp
// Pass 1: mark fully contained boxes for removal
for (size_t i = 0; i < boxes.size(); ++i) {
    if (removed[i]) continue;
    for (size_t j = i + 1; j < boxes.size(); ++j) {
        if (removed[j]) continue;
        if (protected_ids && protected_ids->count(boxes[j].id) > 0)
            continue;  // never remove bridge boxes
        if (box_contains_box(boxes[i], boxes[j])) {
            removed[j] = true;
        }
    }
}
```

删除被完全包含的小 box。这不影响连通性——被包含的 box 的邻居也一定和大 box 相邻。

### Pass 2（问题根源）

```cpp
// Pass 2: trim partial overlaps — shrink the smaller box
for (size_t i = 0; i < boxes.size(); ++i) {
    if (removed[i]) continue;
    for (size_t j = i + 1; j < boxes.size(); ++j) {
        if (removed[j]) continue;
        if (!boxes_overlap(boxes[i], boxes[j])) continue;

        // 选择最优裁剪维度和方向
        // ... 裁剪 box j 的某维度边界 ...

        if (best_side == 0)
            small.joint_intervals[best_dim].lo = overlap_hi;
        else
            small.joint_intervals[best_dim].hi = overlap_lo;
        small.compute_volume();

        // 如果裁剪后太小就删除
        if (geometric_mean_edge(small) < min_gmean_edge) {
            if (!protected_ids || protected_ids->count(small.id) == 0)
                removed[j] = true;
        }
    }
}
```

**问题机制**：

1. **裁剪破坏邻接关系**：两个 box A 和 B 本来通过**面接触**（shared_face）构成邻接边。B 同时和 C 重叠，Pass 2 裁剪 B 的某维 `lo` 或 `hi`。裁剪后，B 可能不再和 A 面接触（B 的该维度区间被缩短了），导致 A-B 邻接断裂。

2. **多轮裁剪叠加**：B 可能被多个更大的 box 依次裁剪多个维度，最终变成非常薄的 box，和原本的邻居全部失去面接触。

3. **传递性断裂**：一条 island 内 box 的链式邻接 ...- X - B - Y - ... 如果 B 被裁剪后与 X 和 Y 都失去接触，整个链条断裂，X 侧和 Y 侧各成独立 island。

4. **protected_ids 只保护不被删除**：`protected_ids` 防止 bridge box 被 `removed[j] = true`，但 **不防止被裁剪**。Bridge box 被裁剪后同样可能失去与原邻居的面接触。

### 图示

```
修改前：
  [box A]──shared_face──[box B]──shared_face──[box C]
                          ↑
                       和 box D 重叠

Pass 2 裁剪 B 的 dim3.lo:
  [box A]    gap    [box B']──shared_face──[box C]
                    (缩小后)
  → A 和 B' 不再面接触 → island 断裂
```

## 修改方案：禁用 Pass 2

### 核心思路

**完全移除 Pass 2**（trim partial overlaps），只保留 Pass 1（remove fully contained boxes）。

重叠 box 的存在不影响规划正确性——adjacency 计算 (`shared_face`) 只看面接触，重叠区域不会导致错误邻接。重叠只会浪费少量存储空间，但保持连通性远比空间效率重要。

### 具体修改

#### 文件：`src/forest/coarsen.cpp`

**修改**：注释掉 Pass 2 整个代码块（约 L340-L395）

```cpp
// 在 Pass 1 的闭合花括号之后，直接跳到 Compact 段
// 将 Pass 2 代码块注释掉或删除：

// [DISABLED: Pass 2 trimming breaks adjacency, causing island fragmentation]
// // Pass 2: trim partial overlaps — shrink the smaller box
// for (size_t i = 0; i < boxes.size(); ++i) {
//     ...
// }
```

### 不修改的部分

- Pass 1（delete fully contained）保持不变
- Compact 段保持不变
- `coarsen_greedy()` 保持不变
- `min_gmean_edge` 参数保留，但仅被 Pass 1 使用（如果需要）

## 预期效果

| 指标 | 修改前 | 修改后预期 |
|------|--------|-----------|
| post-filter islands | 10 | 5（与 post-grow 一致） |
| box 总数 | ~1845 | 略多（保留被 trim 的 box） |
| 碎片 island (≤5 boxes) | 5 | 0 |
| adjacency 边数 | ~2800 | 可能增多（更多 shared_face） |
| query需proxy | TS→CS, CS→LB | 极少（tree间bridge直连） |

## 验证步骤

```bash
cd /home/tian/桌面/box_aabb/cpp/build
cmake --build . --target exp2_e2e_planning -j$(nproc)
./experiments/exp2_e2e_planning --seeds 1 2>&1 | grep -E "post-|coverage|islands"
```

检查：
1. `coverage: islands=` 数量 ≤ 5
2. `post-filter: boxes=` 保持合理（可能比之前略多）
3. SR=100%
4. query 时间应显著下降（不再需要 proxy RRT 跨越大间隙）
5. 所有 5 条路径 Drake 验证 0 collisions

## 风险与回退

- **风险**：保留重叠 box → Dijkstra 搜索图变大 → 搜索时间可能略增（但 Dijkstra 在当前规模极快，<1ms）
- **风险**：重叠 box 可能导致 `extract_waypoints` 生成冗余中间点 → 后续 simplify 已能处理
- **回退**：如果 box 总数膨胀严重（>3000），可改为"裁剪后检查邻接关系、若断裂则回滚裁剪"的策略
- **监控**：比对 filter 前后 box 数差异，确认 Pass 1 仍在有效减少冗余
