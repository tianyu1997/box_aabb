# Plan Step 5: Box-Chain 路径质量优化 (P0)

## 问题

当前所有 5 个查询的最终路径均来自 RRT compete（直连），box-chain 路径被全部淘汰。
系统本质退化为"RRT planner + 后处理优化"，box forest 核心价值未被利用。

**根因**：
1. Dijkstra 使用 face-center 距离做 edge cost，但 box-chain 路径经过大量中间 box，
   绕行严重（如 LB→RB chain=27.2 vs RRT direct=9.9）
2. `extract_waypoints` 在每个 box 的面中心放一个路点，产生大量无用折线
3. Dijkstra 无方向引导，在连通图中可能选择绕远的路径

## 方案

### 5a. Dijkstra → A* (Euclidean heuristic)

**文件**: `src/planner/dijkstra.cpp`

- 在 priority queue item 中加入 `f = g + h`，其中 `h = ||当前box中心 - goal box中心||`
- 保持 admissible（Euclidean 距离是下界），保证最优性
- 效果：A* 优先搜索朝目标方向的 box，避免绕远

```cpp
// 当前：pq.push({d, node_id})        // Dijkstra: f = g
// 改为：pq.push({d + h(node), node_id})  // A*: f = g + h
```

### 5b. extract_waypoints 贪心简化

**文件**: `src/planner/sbf_planner.cpp` (extract_waypoints 附近)

- Box-chain 提取路点后，立即做 greedy forward simplification
- 从 start 开始，连接能直达的最远路点（collision-free 检查）
- 大幅减少冗余路点，缩短路径

### 5c. Box-chain 路径中心线优化

**文件**: `src/planner/dijkstra.cpp` (face_center)

- 当前 face_center 取面的几何中心
- 改为取 "biased face center"：面中心偏移向 start→goal 方向
- 让 box-chain 路径更贴近直线

## 预期效果

| 指标 | 当前 | 目标 |
|------|------|------|
| Box-chain 被 RRT 淘汰次数 | 5/5 | ≤2/5 |
| LB→RB chain 长度 | 27.2 | <15 |
| RB→AS chain 长度 | 18.4 | <8 |

## 实现顺序

1. **5a** Dijkstra → A*（15 min，改 dijkstra.cpp）
2. **5b** extract_waypoints 贪心简化（10 min，改 sbf_planner.cpp）
3. **5c** biased face center（10 min，改 dijkstra.cpp）
4. 测试验证
