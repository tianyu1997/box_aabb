# SafeBoxForest v2 — Python API 参考手册

> **模块名：** `pysbf2`
> **后端：** C++17，通过 pybind11 绑定
> **依赖：** NumPy、Eigen 3.3+

---

## 目录

1. [快速入门](#快速入门)
2. [通用类型](#1-通用类型)
3. [配置](#2-配置)
4. [机器人](#3-机器人)
5. [场景与碰撞](#4-场景与碰撞)
6. [包络体](#5-包络体)
   - [5.1 区间 FK 包络体（C++ 后端）](#51-区间-fk-包络体c-后端)
   - [5.2 采样包络计算器（Python aabb 模块）](#52-采样包络计算器python-aabb-模块)
7. [森林](#6-森林)
8. [桥接 / 粗化](#7-桥接--粗化)
9. [规划器](#8-规划器)
10. [可视化](#9-可视化)
11. [输入/输出](#10-输入输出)
12. [功能标志](#11-功能标志)

---

## 快速入门

```python
import numpy as np
import pysbf2

# 加载机器人并创建障碍物
robot = pysbf2.Robot.from_json("configs/panda.json")
obstacles = [
    pysbf2.Obstacle(np.array([0.5, 0.0, 0.5]),
                     np.array([0.05, 0.3, 0.05]), "shelf"),
]

# 规划
config = pysbf2.make_panda_config(seed=42)
result = pysbf2.plan_once(robot, obstacles, start, goal, config)

if result.success:
    print(f"路径: {result.n_waypoints()} 个路径点, 代价={result.cost:.3f}")
```

---

## 1. 通用类型

### `Interval(lo=0.0, hi=0.0)`

实数轴上的闭区间 \[lo, hi\]。

| 属性 / 方法 | 返回值 | 说明 |
|---|---|---|
| `lo` | `float` | 下界（可读写） |
| `hi` | `float` | 上界（可读写） |
| `width()` | `float` | `hi - lo` |
| `mid()` | `float` | `(lo + hi) / 2` |
| `center()` | `float` | `mid()` 的别名 |
| `contains(v, tol=1e-10)` | `bool` | `lo - tol <= v <= hi + tol` |
| `overlaps(other, tol=0.0)` | `bool` | 两个区间是否有公共点 |
| `hull(other)` | `Interval` | 包含两个区间的最小区间 |
| `intersect(other)` | `Interval` | 交集 |
| `+`、`-`、`*` | `Interval` | 区间算术运算符 |

```python
iv = pysbf2.Interval(-1.0, 1.0)
print(iv.width())        # 2.0
print(iv.contains(0.5))  # True
print(iv * pysbf2.Interval(2.0, 3.0))  # [-3.0, 3.0]
```

---

### `Obstacle(center, half_sizes, name="")`

三维工作空间中的轴对齐盒状障碍物。

| 属性 | 类型 | 说明 |
|---|---|---|
| `center` | `np.ndarray (3,)` | 中心位置 |
| `half_sizes` | `np.ndarray (3,)` | 半尺寸 |
| `name` | `str` | 显示名称 |
| `lo()` | `np.ndarray (3,)` | 下角点 |
| `hi()` | `np.ndarray (3,)` | 上角点 |

---

### `JointLimits()`

每个关节的 `Interval` 范围集合。

| 属性 / 方法 | 返回值 | 说明 |
|---|---|---|
| `limits` | `list[Interval]` | 每个关节的范围 |
| `n_dims()` | `int` | 关节数量 |
| `contains(q, tol=1e-10)` | `bool` | 所有关节是否在限位内 |
| `clamp(q)` | `np.ndarray` | 将 q 截断到限位内 |

---

### `BoxNode(id, intervals, seed_config)`

构型空间中的轴对齐盒子。

| 属性 / 方法 | 返回值 | 说明 |
|---|---|---|
| `id` | `int` | 唯一标识符 |
| `joint_intervals` | `list[Interval]` | 每个关节的范围 |
| `seed_config` | `np.ndarray` | 生成此盒子的种子构型 |
| `volume` | `float` | 超体积 |
| `parent_id` | `int` | 父盒子 ID（-1 表示根节点） |
| `tree_id` | `int` | 树索引（0=起点，1=终点） |
| `children_ids` | `list[int]` | 子盒子 ID 列表 |
| `n_dims()` | `int` | 关节维数 |
| `center()` | `np.ndarray` | 中心构型 |
| `contains(q, tol)` | `bool` | 盒子是否包含 q？ |
| `distance_to_config(q)` | `float` | 到最近点的 L2 距离 |
| `nearest_point_to(q)` | `np.ndarray` | 将 q 投影到盒子上 |
| `overlaps_with(other, tol)` | `bool` | 轴对齐重叠检查 |
| `is_adjacent_to(other, tol)` | `bool` | 面接触检查 |
| `shared_face_center(other)` | `np.ndarray` | 共享面的中心 |

---

### `Edge(from_id, to_id, weight=1.0)`

两个盒子 ID 之间的加权边。

| 字段 | 类型 |
|---|---|
| `from_id` | `int` |
| `to_id` | `int` |
| `weight` | `float` |

---

### `PlanningResult()`

| 字段 | 类型 | 说明 |
|---|---|---|
| `success` | `bool` | 是否找到路径？ |
| `path` | `np.ndarray (N, J)` | 路径点矩阵 |
| `cost` | `float` | 构型空间路径总长度 |
| `planning_time` | `float` | 实际用时（秒） |
| `first_solution_time` | `float` | 首次找到解的时间 |
| `collision_checks` | `int` | 碰撞检测总次数 |
| `nodes_explored` | `int` | 图搜索展开的节点数 |
| `phase_times` | `dict[str, float]` | 各阶段耗时 |
| `metadata` | `dict[str, float]` | 任意元数据 |
| `n_waypoints()` | `int` | 路径点数量 |
| `PlanningResult.failure(time)` | 静态方法 | 创建失败结果 |

---

### `FFBResult()`

自由盒搜索（Find-Free-Box）算法的结果。

| 字段 | 类型 | 说明 |
|---|---|---|
| `node_idx` | `int` | 树节点索引（失败时为 -1） |
| `path` | `list[int]` | 二分搜索路径 |
| `fail_code` | `int` | 0 = 成功 |
| `n_new_nodes` | `int` | 创建的新节点数 |
| `n_fk_calls` | `int` | 正运动学计算次数 |
| `success()` | `bool` | `fail_code == 0` |

---

## 2. 配置

### `EnvelopeConfig()`

| 字段 | 默认值 | 说明 |
|---|---|---|
| `max_depth` | `1000` | 最大二分深度 |
| `min_edge` | `0.01` | 区间最小宽度 |
| `min_edge_anchor` | `0.001` | 锚点最小边长 |
| `min_edge_relaxed` | `0.05` | 松弛后的最小边长 |
| `promotion_depth` | `2` | 提升遍历深度 |

### `ForestConfig()`

| 字段 | 默认值 | 说明 |
|---|---|---|
| `max_boxes` | `500` | 最大盒子数 |
| `max_consecutive_miss` | `20` | 连续 N 次失败后停止 |
| `bfs_phase_k` | `[5, 2, 1]` | 阶段倍率 |
| `bfs_phase_budget` | `[100, 200, 200]` | 每阶段预算 |
| `guided_sample_ratio` | `0.6` | 目标偏向概率 |
| `boundary_expand_epsilon` | `0.01` | 边界扩展量 |
| `n_edge_samples` | `3` | 每个盒子边的采样数 |
| `adjacency_tol` | `1e-10` | 邻接容差 |
| `min_boxes_per_pair` | `500` | 多对起终点最少盒子数 |
| `max_boxes_per_pair` | `5000` | 多对起终点最多盒子数 |

### `BridgeConfig()`

| 字段 | 默认值 | 说明 |
|---|---|---|
| `coarsen_max_rounds` | `20` | 最大粗化轮数 |
| `coarsen_target_boxes` | `0` | 目标盒子数（0=自动） |
| `coarsen_greedy_rounds` | `200` | 贪婪粗化轮数 |
| `coarsen_grid_check` | `False` | 基于网格的检查 |
| `coarsen_split_depth` | `3` | 凸包拆分深度 |
| `coarsen_max_tree_fk` | `2000` | 每轮 FK 预算 |
| `corridor_hops` | `2` | GCS 走廊跳数 |
| `use_gcs` | `False` | 是否使用 Drake GCS |

### `PlannerConfig()`

| 字段 | 默认值 | 说明 |
|---|---|---|
| `shortcut_max_iters` | `100` | 快捷路径迭代次数 |
| `segment_resolution` | `0.05` | 碰撞检测步长 |
| `parallel_grow` | `False` | 是否并行生长 |
| `n_partitions_depth` | `3` | 分区深度 |
| `parallel_workers` | `4` | 工作线程数 |
| `seed` | `0` | 随机种子 |

### `SBFConfig()`

所有子配置的聚合。主要属性：

```python
cfg = pysbf2.SBFConfig()
cfg.max_boxes = 1000
cfg.shortcut_max_iters = 200
cfg.seed = 42

# 分解为子配置
env_cfg = cfg.envelope_config()
forest_cfg = cfg.forest_config()
bridge_cfg = cfg.bridge_config()
planner_cfg = cfg.planner_config()
```

| 字段 | 说明 |
|---|---|
| `ffb_max_depth`、`ffb_min_edge` 等 | 包络体参数 |
| `max_boxes`、`guided_sample_ratio` 等 | 森林参数 |
| `coarsen_max_rounds` 等 | 桥接参数 |
| `shortcut_max_iters`、`segment_resolution` | 规划器参数 |
| `use_cache`、`cache_path` | 缓存设置 |
| `seed` | 全局随机种子 |

---

## 3. 机器人

### `Robot()`

使用 DH 参数的运动学模型。

**工厂方法：**
```python
robot = pysbf2.Robot.from_json("configs/panda.json")
```

| 方法 | 返回值 | 说明 |
|---|---|---|
| `from_json(path)` | `Robot` | 静态方法：从 JSON 加载 |
| `name()` | `str` | 机器人名称 |
| `n_joints()` | `int` | 驱动关节数 |
| `n_links()` | `int` | 连杆数 |
| `n_transforms()` | `int` | 变换矩阵数 |
| `joint_limits()` | `JointLimits&` | 关节限位引用 |
| `dh_params()` | `list[DHParam]` | DH 参数列表 |
| `has_tool()` | `bool` | 是否有工具坐标系？ |
| `has_ee_spheres()` | `bool` | 是否有末端执行器球体？ |
| `n_ee_spheres()` | `int` | 末端执行器球体数 |
| `fingerprint()` | `str` | 几何指纹（哈希值） |
| `fk_link_positions(q)` | `np.ndarray (N,3)` | 连杆位置 |
| `fk_transforms(q)` | `list[np.ndarray(4,4)]` | 完整变换矩阵 |

### `DHParam()`

| 字段 | 类型 | 说明 |
|---|---|---|
| `alpha` | `float` | 连杆扭转角（弧度） |
| `a` | `float` | 连杆长度 |
| `d` | `float` | 连杆偏移 |
| `theta` | `float` | 关节角偏移 |
| `joint_type` | `int` | 0=旋转关节，1=移动关节 |

### 正运动学自由函数

```python
# 标量正运动学
positions = pysbf2.fk_link_positions(robot, q)
transforms = pysbf2.fk_transforms(robot, q)
T = pysbf2.dh_transform(alpha, a, d, theta)

# 区间算术
sin_iv = pysbf2.I_sin(-0.5, 0.5)  # 保证包络
cos_iv = pysbf2.I_cos(-0.5, 0.5)

# 区间正运动学（用于包络体计算）
fk_state = pysbf2.compute_fk_full(robot, intervals)
fk_inc   = pysbf2.compute_fk_incremental(parent, robot, intervals, changed_dim)
```

---

## 4. 场景与碰撞

### `ICollisionChecker`（抽象基类）

| 方法 | 返回值 | 说明 |
|---|---|---|
| `check_config(q)` | `bool` | q 是否无碰撞？ |
| `check_box(intervals)` | `bool` | 盒子是否无碰撞？ |
| `check_segment(q1, q2, step=0.05)` | `bool` | 线段是否无碰撞？ |
| `n_checks()` | `int` | 碰撞检测总次数 |
| `reset_counter()` | | 重置计数器 |

### `AabbCollisionChecker(robot, obstacles)`

基于 AABB 的碰撞检测器。继承自 `ICollisionChecker`。

```python
checker = pysbf2.AabbCollisionChecker(robot, obstacles)
assert checker.check_config(q_free) == True
assert checker.check_config(q_collision) == False
print(f"碰撞检测次数: {checker.n_checks()}")
```

额外方法：`n_obs()`、`n_aabb_slots()`。

### `Scene(obstacles=[])`

动态障碍物集合管理器。

```python
scene = pysbf2.Scene()
scene.add_obstacle(pysbf2.Obstacle(center, half_sizes, "box1"))
scene.add_obstacle(pysbf2.Obstacle(center2, half_sizes2, "box2"))
scene.remove_obstacle("box1")
print(scene.n_obstacles())  # 1
```

| 方法 | 说明 |
|---|---|
| `add_obstacle(obs)` | 添加障碍物 |
| `remove_obstacle(name)` | 按名称移除 |
| `clear()` | 移除全部 |
| `obstacles()` | 获取列表 |
| `n_obstacles()` | 障碍物数量 |
| `repack()` | 重建紧凑数组 |

---

## 5. 包络体

> **已弃用：** Python 层的 `EnvelopeResult` / `IntervalFKEnvelopeComputer`
> API 仅为向后兼容而保留。新 C++ 代码应使用
> `FrameStore` + `collision_policy.h`（参见 HCACHE03）。

### 5.1 区间 FK 包络体（C++ 后端）

### `EnvelopeResult()`

| 字段 | 说明 |
|---|---|
| `n_link_slots` | 连杆 AABB 数量 |
| `n_ee_slots` | 末端执行器 AABB 数量 |
| `valid` | 计算是否成功？ |
| `fk_state` | 用于增量更新的 `FKState` |

### `IntervalFKEnvelopeComputer(robot)`

通过区间正运动学从关节区间计算 AABB 包络体。

```python
env_comp = pysbf2.IntervalFKEnvelopeComputer(robot)
intervals = [pysbf2.Interval(q[i]-0.1, q[i]+0.1) for i in range(7)]
result = env_comp.compute_envelope(intervals)

# 增量计算（在维度 3 被拆分后）
result2 = env_comp.compute_envelope_incremental(
    result.fk_state, new_intervals, changed_dim=3)
```

| 方法 | 说明 |
|---|---|
| `compute_envelope(intervals)` | 完整计算 |
| `compute_envelope_incremental(fk, ivs, dim)` | 增量更新 |
| `n_total_aabb_slots()` | AABB 槽总数 |

---

### 5.2 采样包络计算器（Python aabb 模块）

> **模块路径：** `aabb`（位于 `v4/src/aabb`，独立于 `pysbf2` C++ 绑定）
> **依赖：** NumPy、SciPy（L-BFGS-B 优化）；Cython 加速模块可选

该模块通过**关键点枚举**或**随机采样** + **局部优化**，精确计算每根连杆在给定关节区间内的 AABB（轴对齐包围盒），并提供区间算术保守估计作为对照。结果可用于给 `pysbf2` 的包络体参数提供更紧的先验约束。

#### 计算方法对比

| `method` | `sampling` | 描述 | 精度 | 速度 |
|---|---|---|---|---|
| `'numerical'` | `'critical'` | 关键点枚举（梯度为零点）+ 约束流形采样 + L-BFGS-B | ⭐⭐⭐⭐⭐ | 快 |
| `'numerical'` | `'random'` | 大规模均匀随机采样 + L-BFGS-B | ⭐⭐⭐⭐ | 中 |
| `'interval'` | *(N/A)* | 区间算术，保守上界，不需要采样 | 保守 | 极快 |

#### `AABBCalculator(robot, robot_name=None, skip_first_link=True)`

```python
from aabb.robot import load_robot
from aabb.calculator import AABBCalculator

robot = load_robot('panda')
intervals = robot.joint_limits   # list[tuple[float, float]]

calc = AABBCalculator(robot, robot_name='panda')
```

| 参数 | 类型 | 说明 |
|---|---|---|
| `robot` | `Robot` | 机器人运动学模型 |
| `robot_name` | `str \| None` | 显示名称（默认取 `robot.name`） |
| `skip_first_link` | `bool` | 是否跳过第一个（通常为零长度）连杆，默认 `True` |

---

#### `AABBCalculator.compute_envelope(...) → AABBEnvelopeResult`

```python
# 关键点枚举（默认，推荐）
result = calc.compute_envelope(
    joint_intervals=intervals,
    method='numerical',
    sampling='critical',
)

# 随机采样
result = calc.compute_envelope(
    joint_intervals=intervals,
    method='numerical',
    sampling='random',
    n_random_samples=10000,
)

# 区间算术（保守上界）
result = calc.compute_envelope(
    joint_intervals=intervals,
    method='interval',
)
```

**完整参数列表：**

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `joint_intervals` | `list[tuple[float,float]]` | — | 每个关节的 `(lo, hi)` 区间 |
| `method` | `str` | `'numerical'` | `'numerical'` 或 `'interval'` |
| `sampling` | `str` | `'critical'` | `'critical'` 或 `'random'`（仅 `numerical` 时有效） |
| `n_random_samples` | `int` | `10000` | `random` 策略每连杆采样数 |
| `critical_proximity_threshold` | `float` | `0.05` | `random` 策略避开关键点邻域的半径（rad） |
| `n_subdivisions` | `int` | `1` | 每根连杆等分段数（≥1），>1 时返回更精细的 AABB 序列 |
| `skip_zero_length` | `bool` | `True` | 跳过零长度连杆 |
| `save_report` | `str \| None` | `None` | 若指定路径则保存文本报告 |
| `n_workers` | `int` | `1` | 连杆级并行线程数（各连杆独立，FK 只读线程安全） |
| `opt_workers` | `int` | `1` | 优化器内 6 轴并行线程数 |
| `maxiter` | `int` | `30` | L-BFGS-B 最大迭代次数（减小可加速，精度略降） |
| `n_seeds` | `int` | `2` | 每个轴送入优化器的最大种子数 |
| `max_opt_seeds` | `int` | `300` | 传递给优化器前随机裁剪的种子上限（高自由度连杆加速） |

---

#### `AABBEnvelopeResult`

| 字段 / 属性 | 类型 | 说明 |
|---|---|---|
| `robot_name` | `str` | 机器人名称 |
| `n_joints` | `int` | 关节数 |
| `joint_intervals` | `list[tuple]` | 输入的关节区间 |
| `method` | `str` | 方法标识，如 `'numerical_critical'`、`'interval'` |
| `sampling_mode` | `str` | 采样模式（`'critical'`/`'random'`/`'N/A'`） |
| `link_aabbs` | `list[LinkAABBInfo]` | 各连杆（或子段）的 AABB 列表 |
| `n_samples_evaluated` | `int` | 评估的总采样点数 |
| `n_subdivisions` | `int` | 连杆等分段数 |
| `computation_time` | `float` | 计算耗时（秒） |

```python
for aabb in result.link_aabbs:
    if aabb.is_zero_length:
        continue
    print(f"Link {aabb.link_index}: "
          f"min={aabb.min_point}  max={aabb.max_point}  "
          f"vol={aabb.volume:.4f} m³")
```

---

#### `LinkAABBInfo`

| 字段 / 属性 | 类型 | 说明 |
|---|---|---|
| `link_index` | `int` | 连杆索引（1-based） |
| `link_name` | `str` | 显示名称 |
| `min_point` | `list[float]` | AABB 最小角点 `[x, y, z]`（米） |
| `max_point` | `list[float]` | AABB 最大角点 `[x, y, z]`（米） |
| `is_zero_length` | `bool` | 是否为零长度连杆 |
| `boundary_configs` | `dict[str, BoundaryConfig]` | 六个面各自的极值配置（键：`'x_min'`…`'z_max'`） |
| `segment_index` | `int` | 子段索引（`n_subdivisions>1` 时有效） |
| `n_segments` | `int` | 该连杆的总段数 |
| `t_start` / `t_end` | `float` | 子段在连杆全长上的参数范围 `[0, 1]` |
| `volume` | `float` | AABB 体积 $\mathrm{m}^3$（属性） |
| `size` | `list[float]` | `[dx, dy, dz]`（属性） |

---

#### `BoundaryConfig`

记录使 AABB 某个面达到极值的关节配置。

| 字段 | 类型 | 说明 |
|---|---|---|
| `joint_values` | `np.ndarray` | 完整关节角数组（弧度） |
| `boundary_value` | `float` | 该面的极值坐标（米） |
| `boundary_type` | `str` | `'x_min'`/`'x_max'`/`'y_min'`/`'y_max'`/`'z_min'`/`'z_max'` |
| `link_index` | `int` | 对应连杆索引 |
| `relevant_joints` | `set[int]` | 影响该连杆的关节索引集合（0-based） |
| `boundary_joints` | `set[int]` | 处于区间边界的关节索引集合 |
| `is_aabb_vertex` | `bool` | 是否同时是多个面的极值点 |
| `angle_constraints` | `list[str]` | 检测到的角度耦合约束描述 |

---

#### 完整使用示例

```python
from aabb.robot import load_robot
from aabb.calculator import AABBCalculator
import numpy as np

# 加载机器人
robot = load_robot('panda')
intervals = robot.joint_limits  # 全关节限位范围

calc = AABBCalculator(robot, 'panda')

# 1. 关键点采样（推荐，精度最高）
r_crit = calc.compute_envelope(intervals, method='numerical', sampling='critical')

# 2. 区间算术（保守上界，极快）
r_ifk = calc.compute_envelope(intervals, method='interval')

# 3. 验证包含关系：critical ⊆ IFK
for ca, ia in zip(r_crit.link_aabbs, r_ifk.link_aabbs):
    if ca.is_zero_length:
        continue
    assert np.all(np.array(ia.min_point) <= np.array(ca.min_point) + 1e-6)
    assert np.all(np.array(ca.max_point) <= np.array(ia.max_point) + 1e-6)

print(f"critical 方法耗时: {r_crit.computation_time*1000:.0f} ms")
print(f"interval 方法耗时: {r_ifk.computation_time*1000:.0f} ms")

# 4. 局部关节区间（小盒子）
q0 = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])
delta = 0.3  # rad
local_ivs = [(q0[i] - delta, q0[i] + delta) for i in range(7)]
r_local = calc.compute_envelope(local_ivs, method='numerical', sampling='critical')

# 5. 查看边界臂形
for aabb in r_local.link_aabbs:
    if aabb.is_zero_length:
        continue
    bc = aabb.boundary_configs.get('z_max')
    if bc:
        print(f"Link {aabb.link_index} z_max={bc.boundary_value:.4f}m "
              f"at q={np.round(bc.joint_values, 3)}")
```

#### `Robot`（aabb 模块）

> 注意：`aabb.robot.Robot` 为 **纯 Python** 实现，与 `pysbf2.Robot` 独立，仅用于采样包络计算。

```python
from aabb.robot import load_robot, Robot

# 按名称加载内置配置（'panda'、'2dof_planar'、'3dof_planar' 等）
robot = load_robot('panda')

# 或从 JSON 手动构建
import json
with open('configs/panda.json', encoding='utf-8') as f:
    cfg = json.load(f)
robot = Robot.from_config(cfg)
```

| 属性 / 方法 | 返回值 | 说明 |
|---|---|---|
| `name` | `str` | 机器人名称 |
| `n_joints` | `int` | 关节数 |
| `joint_limits` | `list[tuple[float,float]]` | 各关节 `(lo, hi)` 范围（弧度） |
| `zero_length_links` | `set[int]` | 零长度连杆索引集合 |
| `tool_frame` | `DHParam \| None` | 工具坐标系（若有） |
| `coupled_pairs` | `list[tuple]` | 耦合关节对（关键点搜索用） |
| `coupled_triples` | `list[tuple]` | 耦合关节三元组 |
| `get_link_position(q, link_idx)` | `np.ndarray(3,)` | 标量正运动学 |
| `get_link_positions_batch(Q, link_idx)` | `np.ndarray(N,3)` | 批量正运动学 |
| `compute_relevant_joints(link_idx)` | `set[int]` | 影响该连杆的关节集合 |

---

## 6. 森林

### `SafeBoxForest(n_dims, limits)`

核心数据结构：带邻接图的安全盒集合。

```python
forest = pysbf2.SafeBoxForest(7, robot.joint_limits())

# 批量添加
for box in boxes:
    forest.add_box_no_adjacency(box)
forest.rebuild_adjacency(tol=1e-8)

# 查询
box = forest.find_containing(q)
print(f"构型 q 在盒子 {box.id} 中" if box else "未被覆盖")

# 邻接关系
adj = forest.adjacency()  # dict[int, list[int]]
```

| 方法 | 说明 |
|---|---|
| `add_box_direct(box)` | 添加盒子并更新邻接 |
| `add_box_no_adjacency(box)` | 添加盒子但不更新邻接 |
| `rebuild_adjacency(tol)` | 重建邻接图 |
| `find_containing(q)` | 查找包含 q 的盒子（或 None） |
| `find_nearest(q)` | 查找离 q 最近的盒子 |
| `validate_boxes(checker)` | 移除无效盒子 → ID 集合 |
| `invalidate_against_obstacle(obs, robot, margin)` | 移除碰撞盒子 |
| `n_boxes()` | 盒子数量 |
| `total_volume()` | 盒子体积之和 |
| `boxes()` | `dict[int, BoxNode]` |
| `adjacency()` | `dict[int, list[int]]` |
| `allocate_id()` | 获取新的唯一 ID |

### `HierAABBTree(robot, initial_cap=64)`

用于 FFB 的层次 AABB 树。

```python
tree = pysbf2.HierAABBTree(robot, initial_cap=128)

# 自由盒搜索
result = tree.find_free_box(seed_q, obs_compact, n_obs,
                             max_depth=200, min_edge=0.01)
if result.success():
    intervals = tree.get_node_intervals(result.node_idx)

# 持久化
tree.save("cache.hcache")
tree2 = pysbf2.HierAABBTree.load("cache.hcache", robot)
tree3 = pysbf2.HierAABBTree.load_mmap("cache.hcache", robot)  # 延迟加载
```

### `RootSampler(limits, seed=0)`

```python
sampler = pysbf2.RootSampler(robot.joint_limits(), seed=42)
q_random = sampler.sample_uniform()
q_biased = sampler.sample_guided(goal, guided_ratio=0.6)
```

### `FFBEngine(tree, forest_config)`

带阶段管理的 FFB 引擎。

```python
from pysbf2 import FFBEngine, ForestConfig
engine = FFBEngine(tree, ForestConfig())
result = engine.find_free_box(seed, obs_compact, n_obs)
engine.advance_phase()  # 放松 min_edge 进入下一阶段
```

### `ForestGrower(robot, checker, tree, forest, config)`

协调完整的森林生长算法。

```python
grower = pysbf2.ForestGrower(robot, checker, tree, forest, config)
success = grower.grow(start, goal)
print(f"生长了 {forest.n_boxes()} 个盒子")
```

| 方法 | 说明 |
|---|---|
| `grow(start, goal)` | 单对起终点生长 |
| `grow_multi(pairs, n_random, timeout)` | 多对起终点生长 |
| `regrow(n_target, timeout)` | 重新填充缺失区域 |

---

## 7. 桥接 / 粗化

### 自由函数

```python
# 合并相邻盒子
result = pysbf2.coarsen_forest(forest, checker, max_rounds=20)
print(f"合并次数: {result.merges_performed}")

# 贪婪粗化
result = pysbf2.coarsen_greedy(forest, checker, target_boxes=100)

# 孤岛检测
adj = forest.adjacency()
all_ids = list(forest.boxes().keys())
islands = pysbf2.find_islands(adj, all_ids)
print(f"发现 {len(islands)} 个连通分量")

# 桥接不连通的孤岛
bridge_result = pysbf2.bridge_islands(islands, forest.boxes())
```

### `CoarsenResult`

| 字段 | 说明 |
|---|---|
| `merges_performed` | 合并次数 |
| `rounds` | 执行轮数 |
| `boxes_before` / `boxes_after` | 粗化前后的盒子数 |

### `UnionFind(n)`

| 方法 | 说明 |
|---|---|
| `find(x)` | 查找代表元素 |
| `unite(x, y)` | 合并集合 |
| `connected(x, y)` | 是否在同一集合？ |
| `n_components()` | 连通分量数 |
| `components()` | `dict[int, list[int]]` |

---

## 8. 规划器

### `SBFPlanner(robot, obstacles, config=SBFConfig())`

端到端运动规划器。

```python
planner = pysbf2.SBFPlanner(robot, obstacles, config)

# 一次性规划
result = planner.plan(start, goal, timeout=30.0)

# 或者：构建一次，多次查询
planner.build(start, goal, timeout=30.0)
result1 = planner.query(start, goal)
result2 = planner.query(start2, goal2)

# 增量更新
planner.add_obstacle(new_obs)
planner.regrow(500, timeout=10.0)
result3 = planner.query(start, goal)
```

| 方法 | 返回值 | 说明 |
|---|---|---|
| `plan(start, goal, timeout)` | `PlanningResult` | 构建 + 查询 |
| `build(start, goal, timeout)` | | 仅构建森林 |
| `build_multi(pairs, n_random, timeout)` | | 多对起终点构建 |
| `query(start, goal, timeout)` | `PlanningResult` | 查询已构建的森林 |
| `add_obstacle(obs)` | | 添加障碍物并使碰撞盒失效 |
| `remove_obstacle(name)` | | 按名称移除障碍物 |
| `regrow(n_target, timeout)` | `int` | 重新填充盒子 |
| `clear_forest()` | | 清除森林，保留缓存 |
| `forest` | `SafeBoxForest` | 访问森林 |
| `forest_built` | `bool` | 森林是否已构建？ |
| `config` | `SBFConfig` | 配置 |
| `tree` | `HierAABBTree` | 访问层次树 |

### `PathSmoother(checker, segment_resolution=0.05)`

```python
smoother = pysbf2.PathSmoother(checker, 0.05)
smoothed = smoother.shortcut(raw_path, max_iters=200)
smoothed2 = smoother.box_aware_shortcut(smoothed, forest, max_iters=100)
smoothed3 = smoother.smooth_moving_average(smoothed2, forest, window=5)
resampled = smoother.resample(smoothed3, resolution=0.02)
length = pysbf2.PathSmoother.path_length(resampled)
```

### `TreeConnector(forest, checker, radius=2.0, resolution=0.05, max_attempts=50)`

将起点/终点连接到森林图中。

### 图搜索

```python
# 在盒子邻接图上运行 Dijkstra
result = pysbf2.dijkstra_center_distance(adj, boxes, start_ids, goal_ids)
if result.found:
    waypoints = pysbf2.extract_waypoints(result.path, boxes, start, goal)
```

### 流水线辅助函数

```python
# Panda 默认配置
config = pysbf2.make_panda_config(seed=42)

# 一次性规划便捷函数
result = pysbf2.plan_once(robot, obstacles, start, goal, config)

# 随机场景生成
scene_cfg = pysbf2.SceneConfig()
scene_cfg.n_obstacles = 8
# obstacles = pysbf2.build_random_scene(robot, scene_cfg, start, goal, rng)
```

---

## 9. 可视化

通过子模块 `pysbf2.viz` 访问：

```python
import pysbf2

# 导出森林 + 障碍物 + 路径
pysbf2.viz.export_forest_json(forest, obstacles, path_matrix, "forest.json")

# 仅导出森林
pysbf2.viz.export_forest_json(forest, "forest.json")

# 导出路径上的 FK 坐标系
pysbf2.viz.export_path_frames(robot, path_matrix, "frames.json")

# 导出盒子区间为 CSV
pysbf2.viz.export_boxes_csv(forest, "boxes.csv")
```

然后使用 `python forest_viz.py forest.json` 进行可视化。

---

## 10. 输入/输出

通过子模块 `pysbf2.io` 访问：

```python
import pysbf2

# 障碍物
obstacles = pysbf2.io.load_obstacles_json("scene.json")
pysbf2.io.save_obstacles_json("scene.json", obstacles)

# 规划结果
pysbf2.io.save_result_json("result.json", planning_result)
loaded_result = pysbf2.io.load_result_json("result.json")

# HCache 持久化
cache = pysbf2.io.HCacheFile.create("tree.hcache", n_dims=7, n_links=8,
                                      limits=robot.joint_limits())
cache.flush()
cache.close()

cache2 = pysbf2.io.HCacheFile.open("tree.hcache")
print(f"节点数: {cache2.n_nodes()}, FK 调用次数: {cache2.n_fk_calls()}")
```

---

## 11. 功能标志

```python
print(pysbf2.drake_available())  # 如果编译时包含 Drake 则为 True
print(pysbf2.ompl_available())   # 如果编译时包含 OMPL 则为 True
```

### `GCSOptimizer()`（需要 Drake）

```python
if pysbf2.drake_available():
    gcs = pysbf2.GCSOptimizer()
    result = gcs.optimize(forest, start, goal, corridor_hops=2)
    if result.success:
        print(f"GCS 代价: {result.cost:.3f}")
```

---

## 构建说明

```bash
# 在 v2/ 目录下
mkdir build && cd build

# 不包含 Python 绑定
cmake .. -DCMAKE_BUILD_TYPE=Release

# 包含 Python 绑定
cmake .. -DCMAKE_BUILD_TYPE=Release -DSBF_WITH_PYTHON=ON

# 包含 Drake / OMPL
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DSBF_WITH_PYTHON=ON \
         -DSBF_WITH_DRAKE=ON \
         -DSBF_WITH_OMPL=ON

cmake --build . --config Release
```

构建完成后，将构建目录添加到 `PYTHONPATH` 或安装：

```bash
# 直接导入
export PYTHONPATH=/path/to/v2/build:$PYTHONPATH
python -c "import pysbf2; print(pysbf2.__doc__)"
```
