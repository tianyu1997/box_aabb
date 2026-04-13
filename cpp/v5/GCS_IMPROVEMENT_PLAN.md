# GCS 路径质量改进计划

## 现状

5-seed 测试 (25/25 成功, 0 碰撞):

| Query  | GCS len | Dijkstra len | 差距   |
|--------|---------|-------------|--------|
| AS→TS  | 3.099   | 3.099       | =      |
| TS→CS  | 8.223   | 5.116       | +60%   |
| CS→LB  | 5.801   | 2.983       | +94%   |
| LB→RB  | 3.851   | 4.223       | -9%    |
| RB→AS  | 1.908   | 1.908       | =      |

**核心问题**: TS→CS 和 CS→LB 两个 query 的 GCS 路径显著更长。

## 瓶颈分析

### B1: GCS 路径缺少 corridor 信息 → 后处理失效
- `sbf_planner.cpp` 的 GCS 分支中 `box_seq` 始终为空
- 导致 `corridor_boxes` 为空, `corridor_ok()` 恒返回 false
- 后处理阶段 (greedy simplify / shortcut / elastic band) 全部使用昂贵的 FK碰撞检测，无法利用 box union 快速通道
- **影响**: 后处理速度慢且效果差

### B2: GCS corridor 太窄 (corridor_hops=0)
- 当前 GCS 只在 shortcut_seq + 2-hop gap filler 箱子内优化
- MOSEK 的优化空间受限于这些箱子的凸包
- 对于需要绕行的 query (TS→CS, CS→LB), 缺少平行替代路径
- **影响**: 最优解可能在 corridor 之外

### B3: Dijkstra 管线有 RRT compete 优势
- Dijkstra 路径经过 RRT compete: 3个并行 RRT trial 与 box-chain 竞争
- RRT compete 可能找到直接跨越障碍空间的短路径
- GCS 路径也经过相同的 RRT compete，但如果 GCS 原始路径已经较短，RRT budget 会很小
- **影响**: Dijkstra+RRT 在某些 query 上天然有优势

### B4: 共线简化阈值过紧
- 当前阈值 `deviation < 0.05 rad` 保留了过多近似共线点
- 对于短路径 (8-18 waypoints) 影响不大，但增加了后处理负担

## 改进策略

### P1: GCS 返回 box_seq 供 corridor 构建 [高优先级]
**文件**: `gcs_planner.h`, `gcs_planner.cpp`, `sbf_planner.cpp`

**改动**:
1. `GCSResult` 增加 `std::vector<int> box_sequence` 字段
2. `gcs_plan()` 在返回时填充 `corridor_seq` 作为 `box_sequence`
3. `sbf_planner.cpp` GCS 分支: `box_seq = gcs_res.box_sequence`

**预期效果**: post-processing 可利用 corridor_ok 快速通道, 大幅提升后处理效率和效果。

### P2: corridor_hops=1 扩展 [中优先级]
**文件**: `gcs_planner.h`

**改动**: `corridor_hops` 默认值 0 → 1

**预期效果**: MOSEK 有更多候选箱子, 对复杂 query 可找到更短路径。shortcut_seq 已压缩了基底规模, 1-hop 扩展不会导致 SOCP 爆炸。

### P3: GCS+Dijkstra 双路径竞争 [高优先级]
**文件**: `sbf_planner.cpp`

**改动**: GCS 分支中同时运行 Dijkstra path extraction, 比较长度:
1. GCS → gcs_plan() 得到 gcs_path
2. Dijkstra → dijkstra_search() + shortcut + extract_waypoints 得到 dij_path
3. 选择更短的路径作为 `path`

**预期效果**: 对 TS→CS 和 CS→LB 这类 Dijkstra 天然更优的 query, 不会因使用 GCS 而退化。保留 GCS 在 LB→RB 等 query 上的优势。

### P4: 放宽共线简化阈值 [低优先级]
**文件**: `gcs_planner.cpp`

**改动**: collinear deviation 阈值 0.05 → 0.10

**预期效果**: 减少冗余 waypoints, 让后续 elastic band 等优化有更多提升空间。

## 执行顺序

1. P1 → P3 → P2 → P4
2. 每步编译验证, 最终 5-seed 对比测试
