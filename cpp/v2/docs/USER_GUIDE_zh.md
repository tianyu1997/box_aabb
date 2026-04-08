# SafeBoxForest v2 — 用户指南

## 架构概览

SafeBoxForest（SBF）是一个运动规划库，它将构型空间（C-space）分解为
**轴对齐安全盒（axis-aligned safe boxes）**——覆盖自由构型空间的无碰撞超矩形。
规划过程简化为在相邻盒子组成的图上进行搜索。

```
┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐    ┌──────────┐
│   机器人  │───▶│   场景   │───▶│  包络体   │───▶│   森林   │───▶│  规划器   │
│   模型    │    │  碰撞检测 │    │  计算器   │    │  生长器   │    │   搜索    │
└──────────┘    └──────────┘    └──────────┘    └──────────┘    └──────────┘
     DH            AABB          区间            FFB           Dijkstra
    参数          碰撞检测        正运动学        二分法          快捷路径
```

### 模块一览

| 模块 | 功能 | 关键类 |
|--------|---------|-------------|
| **common** | 类型与配置 | `Interval`、`BoxNode`、`SBFConfig` |
| **robot** | 运动学 | `Robot`、`DHParam`、FK 函数 |
| **scene** | 碰撞检测 | `AabbCollisionChecker`、`Scene` |
| **envelope** | AABB 包络体 | `IntervalFKEnvelopeComputer`（旧版）、`FrameStore` + `collision_policy.h`（新版） |
| **forest** | 盒子生长 | `ForestGrower`、`SafeBoxForest`、`HierAABBTree` |
| **bridge** | 后处理 | `coarsen_forest()`、`find_islands()` |
| **planner** | 路径搜索 | `SBFPlanner`、`PathSmoother` |
| **viz** | 导出可视化 | `export_forest_json()` |
| **io** | 持久化存储 | `load_obstacles_json()`、`HCacheFile` |

---

## 核心概念

### 1. 安全盒（Safe Box）

**安全盒**（`BoxNode`）是构型空间（关节空间）中的轴对齐超矩形，保证其内部无碰撞。
每个盒子由一组 `Interval` 值定义——每个关节对应一个区间。

```python
# 一个 7 自由度的盒子：每个关节有一个区间
intervals = [
    pysbf2.Interval(-0.5, 0.3),   # 关节 0
    pysbf2.Interval(-1.0, -0.2),  # 关节 1
    # ... 还有 5 个
]
box = pysbf2.BoxNode(id=0, intervals=intervals, seed_config=q)
```

### 2. 自由盒搜索（Find-Free-Box, FFB）

FFB 算法接收一个种子构型，在 `HierAABBTree` 中进行二分搜索，
寻找包含该种子的最大无碰撞盒子。在每一层，它通过区间正运动学
计算 AABB 包络体，并检查与障碍物的重叠情况。

### 3. 森林生长

`ForestGrower` 负责协调盒子的创建过程：
1. **锚定阶段**：在起点和终点周围创建盒子
2. **BFS 扩展**：从现有盒子的边界生长相邻盒子
3. **随机填充**：随机采样种子并运行 FFB
4. **阶段松弛**：逐步放松 `min_edge` 以允许更小的盒子

### 4. 图搜索

森林生长完成后，形成一个邻接图。`SBFPlanner`：
1. 通过 `TreeConnector` 将起点/终点连接到图中
2. 运行 Dijkstra 算法寻找盒子序列
3. 在共享面中心提取路径点
4. 通过 `PathSmoother` 平滑路径

### 5. 增量更新

当障碍物发生变化时，SBF 支持增量更新：
- `add_obstacle()`：使碰撞盒子失效
- `remove_obstacle()`：（可重新生长盒子）
- `regrow()`：填充缺失区域

---

## 工作流程

### 工作流 A：一次性规划

最简单的用法——从起点到终点规划一次：

```python
import numpy as np
import pysbf2

robot = pysbf2.Robot.from_json("configs/panda.json")
obstacles = pysbf2.io.load_obstacles_json("scene.json")

start = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])
goal  = np.array([2.0, 0.5, -1.0, -1.5, 0.5, 1.0, -0.5])

result = pysbf2.plan_once(robot, obstacles, start, goal)
print(f"成功: {result.success}, 代价: {result.cost:.3f}")
```

### 工作流 B：一次构建，多次查询

在同一环境中执行多次查询：

```python
config = pysbf2.make_panda_config(seed=42)
planner = pysbf2.SBFPlanner(robot, obstacles, config)

# 为代表性的起点/终点构建森林
planner.build(start, goal, timeout=60.0)
print(f"森林: {planner.forest().n_boxes()} 个盒子")

# 多次查询（快速——无需重新构建）
for goal_i in goals:
    result = planner.query(start, goal_i, timeout=5.0)
    if result.success:
        print(f"  到达目标, 代价={result.cost:.3f}")
```

### 工作流 C：增量更新

障碍物动态变化的环境：

```python
planner = pysbf2.SBFPlanner(robot, initial_obstacles, config)
planner.build(start, goal, timeout=30.0)
result = planner.query(start, goal)

# 新障碍物出现
new_obs = pysbf2.Obstacle(
    np.array([0.4, 0.0, 0.6]),
    np.array([0.1, 0.1, 0.1]),
    "moving_box"
)
planner.add_obstacle(new_obs)

# 重新生长缺失区域
planner.regrow(500, timeout=10.0)

# 重新规划
result2 = planner.query(start, goal)
```

### 工作流 D：多对起终点森林

构建覆盖多对起终点的森林：

```python
pairs = [(start1, goal1), (start2, goal2), (start3, goal3)]
planner.build_multi(pairs, n_random_boxes=5000, timeout=120.0)

# 所有起终点对均可查询
for s, g in pairs:
    result = planner.query(s, g)
```

### 工作流 E：底层控制

完全控制规划流水线：

```python
# 1. 初始化
robot = pysbf2.Robot.from_json("configs/panda.json")
checker = pysbf2.AabbCollisionChecker(robot, obstacles)
config = pysbf2.SBFConfig()
config.max_boxes = 500

# 2. 树与森林
tree = pysbf2.HierAABBTree(robot, initial_cap=128)
forest = pysbf2.SafeBoxForest(robot.n_joints(), robot.joint_limits())

# 3. 生长
grower = pysbf2.ForestGrower(robot, checker, tree, forest, config)
grower.grow(start, goal)

# 4. 粗化（合并盒子）
cr = pysbf2.coarsen_greedy(forest, checker, target_boxes=200)
print(f"粗化: {cr.boxes_before} → {cr.boxes_after}")

# 5. 图搜索
forest.rebuild_adjacency()
adj = forest.adjacency()
boxes = forest.boxes()
start_ids = {b.id for b in boxes.values() if b.contains(start)}
goal_ids  = {b.id for b in boxes.values() if b.contains(goal)}

dr = pysbf2.dijkstra_center_distance(adj, boxes, start_ids, goal_ids)
if dr.found:
    waypoints = pysbf2.extract_waypoints(dr.path, boxes, start, goal)

    # 6. 平滑
    smoother = pysbf2.PathSmoother(checker, 0.05)
    smoothed = smoother.shortcut(waypoints, max_iters=200)
    print(f"路径长度: {pysbf2.PathSmoother.path_length(smoothed):.3f}")
```

---

## 调参指南

### 速度与覆盖率

| 目标 | 设置 |
|---|---|
| 更多盒子 | `max_boxes` ↑ |
| 更快生长 | `max_consecutive_miss` ↓ |
| 更好覆盖 | `guided_sample_ratio` ↓（更多随机采样） |
| 更小盒子 | `ffb_min_edge` ↓ |
| 更少 FK 调用 | `ffb_max_depth` ↓ |

### 路径质量

| 目标 | 设置 |
|---|---|
| 更平滑的路径 | `shortcut_max_iters` ↑ |
| 更细的分段 | `segment_resolution` ↓ |
| 更紧致的盒子 | `ffb_min_edge` ↓ |

### 内存与缓存

| 用途 | 方法 |
|---|---|
| 持久化树 | `tree.save("cache.hcache")` |
| 快速重载 | `HierAABBTree.load_mmap(path, robot)` |
| 自动缓存 | `config.use_cache = True; config.cache_path = "..."` |

---

## 机器人 JSON 格式

```json
{
  "name": "panda",
  "dh_params": [
    {"alpha": 0, "a": 0, "d": 0.333, "theta": 0, "joint_type": 0},
    {"alpha": -1.5708, "a": 0, "d": 0, "theta": 0, "joint_type": 0}
  ],
  "joint_limits": [
    {"lo": -2.8973, "hi": 2.8973},
    {"lo": -1.7628, "hi": 1.7628}
  ],
  "tool_frame": {"alpha": 0, "a": 0, "d": 0.107, "theta": -0.7854},
  "link_radii": [0.08, 0.08, 0.07, 0.07, 0.06, 0.06, 0.05, 0.04],
  "ee_spheres": [
    {"center": [0, 0, 0], "radius": 0.04}
  ],
  "ee_spheres_frame": 8
}
```

---

## 错误处理

所有 C++ 异常都会以 `RuntimeError` 的形式传递到 Python 端。
常见问题：

| 错误信息 | 原因 | 解决方法 |
|---|---|---|
| "No containing box" | 起点/终点超出关节限位 | 使用 `joint_limits.clamp(q)` 进行截断 |
| "Forest empty" | 森林生长失败 | 增大 `max_boxes`，放松 `min_edge` |
| "Timeout" | 规划超时 | 增加 `timeout` 或减少 `max_boxes` |
| "File not found" | JSON/缓存路径错误 | 检查路径参数 |
