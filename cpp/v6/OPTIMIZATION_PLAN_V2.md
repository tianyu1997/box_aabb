# SBF v5 优化计划 V2：Grower连通 + Coarsen压缩 + GCS + 射线检测

> 日期：2026-04-11  
> 基线：5-seed median build=15.4s, boxes~5500, 25/25 OK, pre-bridge 5 islands, 4/5 seeds final 1 island

## 当前数据基线（5-seed Dijkstra）

| Seed | Build  | Boxes | Islands(final) | TS→CS   | CS→LB   |
|------|--------|-------|----------------|---------|---------|
| 0    | 21.50s | 5633  | 1              | 1.81s   | 4.20s   |
| 1    | 15.38s | 5628  | 2              | 5.70s   | 3.53s   |
| 2    | 15.04s | 5478  | 1              | 2.16s   | 4.42s   |
| 3    | 20.60s | 5555  | 1              | 2.93s   | 4.84s   |
| 4    | 13.83s | 5503  | 1              | 1.90s   | 1.82s   |
| Med  | 15.38s | 5555  | —              | 2.16s   | 4.20s   |

Pipeline 瓶颈：
- pre-bridge 始终 5 islands（cross-seed +124 boxes 但不减少 islands）
- cluster merge 修复后有效（88/56 clusters），greedy score=500 工作正常
- island #3 (~950 boxes) 需 >25 RRT 尝试才能桥接，bridge 占 build 50-70%
- query 时跨 island 的 TS→CS 需 runtime bridge（5.7s 最差 case）

---

## Phase D: 射线-AABB 基础工具

> **依赖**：无  
> **被依赖**：Phase B, Phase C  
> **预期效果**：提供精确的段-box 包含检测，替代离散采样

### D1: `ray_aabb.h` — 核心射线工具

**文件**: `include/sbf/core/ray_aabb.h`

实现三个函数：

```cpp
// 1. 线段完全在单个 AABB 内（利用凸性：两端点在内即可）
bool segment_in_box(const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                    const BoxNode& box, double tol = 1e-10);

// 2. Slab-method 射线-AABB 交叉：返回 [t_enter, t_exit]
//    线段 p(t) = origin + t * dir, t ∈ [0, 1]
//    若不相交返回 {1, 0}（t_enter > t_exit）
std::pair<double, double> ray_intersect_box(
    const Eigen::VectorXd& origin,
    const Eigen::VectorXd& dir,
    const BoxNode& box);

// 3. 线段是否完全在 box 集合的并集内
//    算法：每个 box 算 [t_enter, t_exit] ∩ [0,1]，排序合并，检查是否覆盖 [0,1]
bool segment_in_box_union(
    const Eigen::VectorXd& a, const Eigen::VectorXd& b,
    const std::vector<BoxNode>& boxes,
    const std::vector<int>& box_ids,
    double tol = 1e-10);
```

### D2: Shortcut 快速路径预检

在 `sbf_planner.cpp` 的 shortcut 阶段增加 box-union 预检：
- 维护 corridor box set（Dijkstra box_sequence ± 1 hop）
- `segment_in_box_union()` 通过 → 跳过 FK 碰撞检查
- 不通过 → fallback 到 `checker.check_segment()`

### D3: 单元测试

`test/test_ray_aabb.cpp`：端点在内/外、部分穿越、边界、dir≈0、union 覆盖/不覆盖

### 验证标准
- 单元测试全通过
- 1-seed query 时间不增长
- shortcut 阶段产生的路径长度 ≤ 原始（不劣化）

---

## Phase A: Grower Pre-bridge 连通性

> **目标**：pre-bridge islands 从 5 降到 ≤2  
> **预期 build 影响**：+1-2s（Phase 2 额外 20% 生长）

### A3: 两阶段生长

修改 `grower.cpp::grow_parallel()`：

1. **Phase 1** (80%)：每棵树 `max_boxes/n_subtrees * 0.8 = 4000` boxes
2. 合并 Phase 1 结果 → `compute_adjacency()` + `find_islands()`
3. **Cross-seed**（现有逻辑，budget 5000）
4. **Phase 2** (20%)：仅为**小岛**（非主组件）的树启动新 worker
   - 每棵 `max_boxes/n_subtrees * 0.2 = 1000` boxes
   - `rrt_goal_bias = 0.7`（强偏向主组件最近 box center）
   - 作为定向生长，直奔主组件

### A5: 中点播种

Phase 1 后、cross-seed 前：对每对 (小岛, 主组件) 找最近 box 对，在中点 `try_create_box()`

### 验证标准
- 1-seed: pre-bridge islands ≤ 3（当前=5）
- 5-seed: final islands=1 比例 5/5（当前=4/5）
- build median ≤ 18s
- 失败则回退（恢复 100% 单阶段生长）

---

## Phase B: Coarsen 路径走廊压缩

> **目标**：路径 box 序列从 ~15-30 减少到 ≤10  
> **依赖**：Phase D（射线验证合并后 hull）

### B7: 路径感知走廊合并

在 coarsen2 cluster2 之后插入新阶段：

1. 对每个 query pair 预跑 `dijkstra_search()` 获取 box_sequence
2. 扩展 ±1 hop 邻居 → corridor set
3. 对 corridor 做 `coarsen_greedy()` with 极宽松参数：
   - `score_threshold = 5000, max_lect_fk = 50000`
4. 用 `segment_in_box()` 验证 hull 仍覆盖原路径

### B8: Cluster2 增大

- `max_cluster_size`: 12 → 20
- `score_threshold`: 500 → 1000

### B9: Query-time box-chain 压缩

Dijkstra `box_sequence` → `shortcut_box_sequence()` 之后：
- 对连续 box 尝试 hull AABB 碰撞检查
- 通过 → 合并为临时大 box
- 减少 `extract_waypoints()` 的 waypoint 数

### 验证标准
- 1-seed: box_sequence 长度 ≤ 10（检查 log）
- final boxes < 5000
- 路径长度不劣化
- 25/25 OK 不退化

---

## Phase C: 启用 Drake GCS 规划

> **目标**：GCS 凸优化产生更平滑、更短路径  
> **依赖**：Drake 已安装

### C1: 启用 Drake 编译

```bash
cd cpp/build
cmake .. -DSBF_WITH_DRAKE=ON
make -j
```

### C2: 默认启用 GCS

修改 `exp2_e2e_planning.cpp`：`cfg.use_gcs = true`

### C3: GCS 走廊扩展

`GCSConfig::corridor_hops` = 3（从 2）

### C4: GCS 射线 shortcut 后处理

GCS 输出 path 后，用 `segment_in_box_union()` 做贪心前向 shortcut：
- 从 wp[0] 找最远 wp[j] 使得段在 corridor 内 → 跳过中间 waypoints
- O(d × k × n) 精确 shortcut

### C5: Smoother 兜底

GCS+shortcut 后的 path 仍接入现有 smoother 管线（EB + safety net）

### 验证标准
- 5-seed `--gcs`: 25/25 OK
- path length ≤ Dijkstra baseline
- TS→CS query time < 2s median
- build time 无明显增长

---

## 执行顺序与回退策略

```
Phase D (射线工具) ─── 无 build 影响，纯新增
    │ OK → 保留
    │ FAIL → 回退 ray_aabb.h
    ▼
Phase A (两阶段生长) ─── build +1-2s，减少 islands
    │ OK → 保留
    │ build >20s 或 success rate 下降 → 回退到单阶段
    ▼
Phase B (路径走廊压缩) ─── coarsen 阶段 +0.5s
    │ OK → 保留
    │ box count 不减 或 路径劣化 → 回退
    ▼
Phase C (Drake GCS) ─── query 侧优化
    │ OK → 保留
    │ Drake 编译失败 → 保留 gcs_plan_fallback
    │ GCS 路径劣化 → 回退到 Dijkstra
    ▼
5-seed 最终验证
```

---

## 文件修改清单

| Phase | 文件 | 修改类型 |
|-------|------|----------|
| D1 | `include/sbf/core/ray_aabb.h` | 新建 |
| D2 | `src/planner/sbf_planner.cpp` | 修改 shortcut 阶段 |
| D3 | `test/test_ray_aabb.cpp` | 新建 |
| A3 | `src/forest/grower.cpp` | 修改 grow_parallel |
| A5 | `src/forest/grower.cpp` | 修改 cross-seed 前 |
| B7 | `src/planner/sbf_planner.cpp` | 新增走廊合并阶段 |
| B8 | `src/planner/sbf_planner.cpp` | 修改 cluster2 参数 |
| B9 | `src/planner/dijkstra.cpp` | 新增 hull 合并逻辑 |
| C1 | `CMakeLists.txt` (build) | cmake 参数 |
| C2 | `experiments/exp2_e2e_planning.cpp` | 默认 gcs=true |
| C3 | `src/planner/gcs_planner.cpp` | corridor_hops |
| C4 | `src/planner/sbf_planner.cpp` | GCS 后处理 |
