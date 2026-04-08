# SafeBoxForest v3 API 参考文档（中文版）

> 从 v2 迁移。涵盖当前已迁移模块：common、robot、envelope、scene。

---

## 1. 通用类型 (`include/sbf/common/types.h`)

### `Interval`（区间）
| 成员 | 类型 | 说明 |
|---|---|---|
| `lo`, `hi` | `double` | 下界/上界 |
| `width()` | `double` | `hi - lo` |
| `center()` / `mid()` | `double` | `(lo + hi) / 2` |
| `contains(v, tol)` | `bool` | 点包含判断 |
| `overlaps(other, tol)` | `bool` | 区间重叠判断 |
| `+`, `-`, `*` | `Interval` | 区间算术 |
| `hull(other)` | `Interval` | 两个区间的凸包 |

### `AABB3D`（轴对齐包围盒）
| 成员 | 类型 | 说明 |
|---|---|---|
| `lo[3]`, `hi[3]` | `float` | 轴对齐包围盒 |
| `overlaps(o, eps)` | `bool` | 三维重叠检测 |

### `Obstacle`（障碍物）
| 成员 | 类型 | 说明 |
|---|---|---|
| `center`, `half_sizes` | `Vector3d` | 盒子中心和半尺寸 |
| `lo()`, `hi()` | `Vector3d` | 角点访问器 |

### `BoxNode`（位形空间盒子节点）
包含 `id`、`joint_intervals`、`seed_config`、`volume`、邻接信息的位形空间盒子。

| 字段 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `id` | `int` | `-1` | 唯一盒子标识符 |
| `joint_intervals` | `vector<Interval>` | — | 各维度的位形空间盒子边界 |
| `seed_config` | `VectorXd` | — | 生成此盒子的种子位形 |
| `volume` | `double` | `0.0` | 位形空间体积（各区间宽度之积） |
| `parent_id` | `int` | `-1` | 父盒子 ID（树结构） |
| `tree_id` | `int` | `-1` | LECT 树节点索引 |
| `children_ids` | `vector<int>` | — | 子盒子 ID |
| `parent_box_id` | `int` | `-1` | 扩展来源盒子（`-1` = 根节点） |
| `expand_face_dim` | `int` | `-1` | 扩展所用面的维度 |
| `expand_face_side` | `int` | `-1` | `0` = 下面, `1` = 上面 |
| `root_id` | `int` | `-1` | 所属根树 |

主要方法：`contains()`、`is_adjacent_to()`、`shared_face_center()`、`overlaps_with()`、`distance_to_config()`、`nearest_point_to()`。

### `JointLimits`（关节限位）
包含 `contains()` 和 `clamp()` 的 `Interval` 向量。

### 其他：`PlanningResult`、`FFBResult`、`Edge`

### `EnvelopeConfig` (`include/sbf/common/config.h`)
模块级包络配置。

| 字段 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `max_depth` | `int` | `1000` | 最大树深度 |
| `min_edge` | `double` | `0.01` | 分裂最小盒子边长 |
| `min_edge_anchor` | `double` | `0.001` | 锚点节点最小边长 |
| `min_edge_relaxed` | `double` | `0.05` | 放松最小边长 |
| `store_format` | `StoreFormat` | `FRAMES` | `AABB_LEGACY` 或 `FRAMES` |
| `collision_policy` | `CollisionPolicy` | `AABB` | `AABB`、`AABB_SUBDIV` 或 `GRID` |
| `aa_crossover_width` | `double` | `0.0` | **混合 IA/AA**：当最大区间宽度 ≤ 阈值时使用 AA 计算 AABB。0 = 禁用。推荐：0.05–0.10 |

### `PipelineConfig` (`include/sbf/envelope/envelope_type.h`)
组合源+包络配置，含工厂方法。

| 字段 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `source` | `EndpointSourceConfig` | — | 端点源方法配置 |
| `envelope` | `EnvelopeTypeConfig` | — | 包络类型配置 |
| `aa_crossover_width` | `double` | `0.0` | **混合 IA/AA** 切换宽度（语义同 `EnvelopeConfig`） |

| 工厂方法 | 源 | 包络 | 说明 |
|---|---|---|---|
| `recommended()` | CritSample | Hull16_Grid | 最佳性价比 |
| `fast()` | IFK | SubAABB(n_sub=1) | 最快，最保守 |
| `tightest()` | AnalyticalCrit | Hull16_Grid_Analytical | 最慢，最紧 |
| `production()` | IFK | Hull16_Grid | 默认（快速+紧凑） |

---

## 2. 机器人 (`include/sbf/robot/`)

### `Robot` 类
| 方法 | 返回值 | 说明 |
|---|---|---|
| `from_json(path)` | `Robot` | 从 JSON 配置加载 |
| `n_joints()` | `int` | 自由度数 |
| `n_active_links()` | `int` | 碰撞相关连杆数 |
| `dh_params()` | `vector<DHParam>` | DH 参数链 |
| `joint_limits()` | `JointLimits` | 关节范围限制 |
| `link_radii()` | `vector<double>` | 各连杆胶囊半径 |
| `active_link_map()` | `const int*` | 活动连杆索引到帧的映射 |
| `fk_link_positions(q)` | `MatrixXd` | 标量正运动学计算所有连杆 |
| `fk_transforms(q)` | `vector<Matrix4d>` | 完整正运动学变换 |

### 区间正运动学 (`interval_fk.h`)
| 函数 | 说明 |
|---|---|
| `compute_fk_full(robot, intervals)` | 从头计算完整区间正运动学 |
| `compute_fk_incremental(parent, robot, intervals, dim)` | 利用父节点前缀增量计算 |
| `extract_link_aabbs(state, ...)` | 从正运动学状态导出连杆 AABB |
| `compute_link_aabbs(robot, intervals, out)` | 一步完成：正运动学 + 提取 |
| `compute_all_aabbs(robot, intervals, out_link, out_ee)` | 连杆 + 末端球体 |

### 区间数学 (`interval_math.h`)
| 函数 | 说明 |
|---|---|
| `I_sin(lo, hi)` / `I_cos(lo, hi)` | 区间正弦/余弦 |
| `build_dh_joint(...)` | 构建区间 DH 矩阵 |
| `imat_mul_dh(...)` | 区间 4×4 矩阵乘法 |

### 仿射算术正运动学 (`affine_fk.h`)

DH 链的简化仿射算术（AA）正运动学。每个标量表示为 $\hat{x} = x_0 + \sum_i x_i \varepsilon_i + \delta \varepsilon^*$，其中 $\varepsilon_i \in [-1,1]$ 为噪声符号（每个关节一个），$\delta \geq 0$ 为累积非线性误差。相比区间算术的关键优势：$\cos(q_j)$ 和 $\sin(q_j)$ 共享噪声符号 $\varepsilon_j$，在链式乘法中保持 $\cos^2+\sin^2=1$ 的相关性。

#### 常量

| 常量 | 值 | 说明 |
|---|---|---|
| `AA_NSYM` | `8` | 噪声符号数（每关节一个，8 足以应对 7 自由度手臂） |

#### `AffineForm`（仿射形式）
| 成员/方法 | 类型 | 说明 |
|---|---|---|
| `x0` | `double` | 中心值 |
| `xi[AA_NSYM]` | `double[8]` | 偏差分量（噪声系数） |
| `delta` | `double` | 累积非线性误差界（≥ 0） |
| `radius()` | `double` | 总偏差：$\delta + \sum |x_i|$ |
| `lo()`, `hi()` | `double` | 区间界：$x_0 \mp \text{radius}$ |
| `constant(v)` | `AffineForm` | （静态）从标量常数创建 |
| `from_interval(lo, hi, sym_id)` | `AffineForm` | （静态）从区间和噪声符号 `sym_id` 创建 |

#### 算术运算

| 函数 | 说明 |
|---|---|
| `aa_add(a, b)` | 仿射加法 |
| `aa_sub(a, b)` | 仿射减法 |
| `aa_neg(a)` | 仿射取反 |
| `aa_scale(a, s)` | 常数缩放 |
| `aa_mul(a, b)` | 简化 AA 乘法，非线性余项 $R_x \cdot R_y$ |

#### 三角函数（Taylor 线性化）

| 函数 | 返回值 | 说明 |
|---|---|---|
| `aa_cos_sin(theta)` | `AACosSin` | 最小范围一阶 Taylor：$\cos(\hat\theta) \approx \cos\theta_0 - \sin\theta_0 \cdot \Delta$，余项 $\leq R^2/2$。cos 和 sin 共享相同噪声符号。 |
| `aa_cos_sin_interval(lo, hi, sym_id)` | `AACosSin` | 便捷版：直接从区间界创建 |

#### `AAMatrix4`
由 `AffineForm` 组成的 3×4 齐次矩阵（第 4 行隐含为 `[0,0,0,1]`）。

| 方法 | 说明 |
|---|---|
| `identity()` | （静态）AA 单位矩阵 |

| 函数 | 说明 |
|---|---|
| `aa_mat_mul(A, B)` | 4×4 AA 矩阵乘法 |

#### `AAFKResult`
| 成员 | 类型 | 说明 |
|---|---|---|
| `prefix` | `vector<AAMatrix4>` | 前缀变换（基座, T₀, T₀T₁, ...） |
| `n_tf` | `int` | 变换数量 |
| `valid` | `bool` | 计算是否成功 |

#### 核心 API

| 函数 | 说明 |
|---|---|
| `aa_compute_fk(robot, intervals)` | 完整 AA 正运动学链 → `AAFKResult` |
| `aa_position_bounds(T, lo, hi)` | 从 `AAMatrix4` 提取 xyz 位置区间 |
| `aa_extract_link_aabbs(state, link_map, n, out, radii)` | 从 AA 前缀变换导出各连杆 AABB（类比 IA 的 `extract_link_aabbs`） |
| `aa_compute_link_aabbs(robot, intervals, out)` | 便捷版：AA 正运动学 + 提取一步完成 |

**性能特征**（与 IA 对比，7-DOF Panda/IIWA14）：
- **约 6.5 倍更快**（约 3.1 µs 对 约 20.5 µs 每次调用）
- **窄区间更紧凑**（宽度 ≤ 0.05）：AA/MC ≈ 1.37 对 IA/MC ≈ 1.87
- **宽区间发散**（宽度 > 0.2）：Taylor 余项 $R^2/2$ 在链中累积
- 两种方法均 100% 保守（始终包含蒙特卡洛真值）

### 混合 IA/AA AABB (`hybrid_aabb.h`)

根据最大区间宽度自动选择 IA 或 AA 的调度器。

| 函数 | 返回值 | 说明 |
|---|---|---|
| `max_interval_width(intervals)` | `double` | 所有关节中最大的 `hi - lo` |
| `extract_link_aabbs_hybrid(fk, intervals, robot, out, aa_crossover)` | `bool` | 若 `aa_crossover > 0` 且 `max_width ≤ aa_crossover`：使用 AA（返回 `true`）。否则：使用预计算的 FKState 中的 IA（返回 `false`）。 |

**集成**：已接入 `LECT::compute_envelope()` 和 `AabbCollisionChecker::check_box()`——两者均通过 config 接受 `aa_crossover_width`。

---

## 3. 包络 (`include/sbf/envelope/`)

### 两阶段流水线架构

包络流水线采用简洁的**两阶段架构**：

```
阶段 1（EndpointSource）：位形空间区间 → endpoint_aabbs [n_endpoints × 6]
阶段 2（EnvelopeType）：  endpoint_aabbs → LinkEnvelope（SubAABB / Grid / VoxelGrid）
```

**4 × 3 = 12 种流水线组合：**

| | SubAABB | SubAABB_Grid | Hull16_Grid |
|---|---|---|---|
| **IFK** | ✓ | ✓ | ✓ |
| **CritSample** | ✓ | ✓ | ✓ |
| **Analytical** | ✓ | ✓ | ✓ |
| **GCPC** | ✓ | ✓ | ✓ |

### 阶段 1：端点源 (`frame_source.h`)

#### `EndpointSource`（枚举）
| 值 | 名称 | 说明 |
|---|---|---|
| `0` | `IFK` | 区间正运动学（快速，保守） |
| `1` | `CritSample` | 临界点采样（中等，更紧） |
| `2` | `Analytical` | 解析雅可比求解器（慢，最紧） |
| `3` | `GCPC` | 全局临界点缓存 + AA 剪枝 |

向后兼容别名：`FrameSourceMethod = EndpointSource`

#### `EndpointSourceConfig`
| 工厂方法 | 源 | 备注 |
|---|---|---|
| `ifk()` | IFK | 可选设置 `ifk_config.aa_crossover` |
| `crit_sampling()` | CritSample | 使用 `CriticalSamplingConfig::full_fast()` |
| `analytical()` | Analytical | 使用默认 `AnalyticalCriticalConfig` |
| `gcpc(cache)` | GCPC | 需要 `GcpcCache*` |

向后兼容别名：`FrameSourceConfig = EndpointSourceConfig`，`analytical_critical() = analytical()`

#### `EndpointAABBResult`
| 成员 | 类型 | 说明 |
|---|---|---|
| `endpoint_aabbs` | `vector<float>` | `[n_endpoints × 6]`——各端点位置区间 AABB（仅几何信息，无半径） |
| `n_endpoints` | `int` | 端点数（= n_joints + has_tool） |
| `n_active` | `int` | 活动连杆数 |
| `fk_state` | `FKState` | 区间正运动学状态（始终填充） |
| `n_evaluations` | `int` | 总正运动学评估次数 |
| `total_boxes()` | `int` | `n_endpoints` |
| `has_fk_state()` | `bool` | `fk_state.valid` |

向后兼容别名：`FrameSourceResult = EndpointAABBResult`

#### 核心 API
| 函数 | 说明 |
|---|---|
| `compute_endpoint_aabb(config, robot, intervals)` | 阶段 1：计算端点 AABB `[n_endpoints × 6]` |
| `compute_endpoint_aabb_incremental(config, parent_fk, robot, intervals, dim)` | 增量计算（前缀复用） |
| `extract_link_aabbs_from_endpoint(result, robot, out)` | 从端点导出各连杆 AABB `[n_active × 6]` |
| `fk_to_endpoints(fk, robot, out)` | 从 FKState 提取位置区间 |

向后兼容别名：`compute_frame_source()`、`compute_frame_source_incremental()`、`extract_link_aabbs_from_result()`

### 阶段 2：包络类型 (`envelope_type.h`)

#### `EnvelopeType`（枚举）
| 值 | 名称 | 说明 |
|---|---|---|
| `0` | `SubAABB` | 按连杆细分 AABB（n_sub=1 → 每连杆单个 AABB） |
| `1` | `SubAABB_Grid` | 子 AABB 栅格化为字节网格 |
| `2` | `Hull16_Grid` | 通过 `fill_hull16()` 按连杆稀疏 VoxelGrid（Conv(B₁∪B₂)⊕Ball） |

#### `EnvelopeTypeConfig`
| 工厂方法 | 类型 | 关键参数 |
|---|---|---|
| `sub_aabb()` | SubAABB | `n_sub`（默认 1） |
| `aabb()` | SubAABB | 向后兼容别名，等价于 `sub_aabb()` |
| `sub_aabb_grid(R)` | SubAABB_Grid | `n_sub`、`grid_R` |
| `hull16_grid(delta)` | Hull16_Grid | `delta`（`n_sub` 已忽略） |

#### `LinkEnvelopeResult`
| 成员 | 类型 | 说明 |
|---|---|---|
| `valid` | `bool` | 计算是否成功 |
| `volume` | `double` | 占用体积 (m³) |
| `n_voxels` | `int` | 占用体素/子 AABB 数量 |
| `n_bricks` | `int` | 砖块数量（仅 Hull16_Grid） |
| `sub_aabbs` | `vector<float>` | 各连杆 AABB（SubAABB） |
| `grid` | `vector<uint8_t>` | 字节网格（SubAABB_Grid） |
| `voxel_grid` | `VoxelGrid` | 稀疏体素网格（Hull16_Grid） |

#### 核心 API
| 函数 | 说明 |
|---|---|
| `compute_link_envelope(config, robot, ep_result)` | 阶段 2：endpoint_aabbs → 连杆包络 |
| `default_envelope_config(source, env_type)` | 获取源×包络组合的默认配置 |

向后兼容别名：`compute_envelope_repr()`

### SBFConfig 工厂方法 (`forest/sbf.h`)

12 个静态工厂方法对应所有源×包络组合
（另有 4 个向后兼容 `*_aabb()` 别名 → SubAABB(n_sub=1)）：

| IFK | CritSample | Analytical | GCPC |
|---|---|---|---|
| `ifk_sub_aabb()` | `crit_sub_aabb()` | `analytical_sub_aabb()` | `gcpc_sub_aabb(cache)` |
| `ifk_sub_aabb_grid()` | `crit_sub_aabb_grid()` | `analytical_sub_aabb_grid()` | `gcpc_sub_aabb_grid(cache)` |
| `ifk_hull16_grid()` | `crit_hull16_grid()` | `analytical_hull16_grid()` | `gcpc_hull16_grid(cache)` |

---

### AABB 导出 (`envelope_derive.h`)
| 函数 | 说明 |
|---|---|
| `derive_aabb(frames, ..., out)` | 从缓存的关节原点帧导出 AABB |
| `derive_aabb_subdivided(frames, ..., n_sub, ..., out)` | 连杆细分 AABB |
| `derive_grid(frames, ..., R, ..., out)` | 从帧数据导出体素网格包络 |

### 临界点包络 (`envelope_derive_critical.h`)
| 函数 | 说明 |
|---|---|
| `derive_crit_endpoints(robot, intervals, out)` | 临界点端点采样 |
| `derive_aabb_critical(robot, intervals, n_sub, out)` | 通过临界点计算 AABB |
| `derive_aabb_critical_enhanced(robot, intervals, n_sub, config, out)` | 增强版含流形+LBFGS |
| `derive_aabb_critical_analytical(robot, intervals, n_sub, config, out)` | 解析雅可比方法 |

#### `AnalyticalCriticalConfig`（解析临界点求解器配置）
五阶段解析临界点求解器的配置。

| 字段 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `keep_kpi2_baseline` | `bool` | `true` | 阶段 0：枚举 kπ/2 顶点位形 |
| `enable_edge_solve` | `bool` | `true` | 阶段 1：单自由度边临界点 |
| `enable_face_solve` | `bool` | `true` | 阶段 2：双自由度面临界点（符号多项式+区间求解器） |
| `face_all_pairs` | `bool` | `false` | 阶段 2：使用所有关节对（而非仅耦合+相邻） |
| `enable_pair_1d` | `bool` | `true` | 阶段 2.5a：配对约束单自由度求解器 |
| `enable_pair_2d` | `bool` | `true` | 阶段 2.5b：配对约束双自由度求解器 |
| `pair_max_bg` | `int` | `25` | 阶段 2.5：每对最大背景位形数 |
| `pair_kpi2_backgrounds` | `bool` | `true` | 阶段 2.5：使用 kπ/2 晶格背景 |
| `pair_all_pairs` | `bool` | `false` | 阶段 2.5：使用所有关节对 |
| `enable_interior_solve` | `bool` | `true` | 阶段 3：坐标下降内部扫描 |
| `interior_max_free` | `int` | `2` | 阶段 3：最大同时自由关节数 |
| `interior_max_sweeps` | `int` | `3` | 阶段 3：最大完整扫描迭代数 |
| `improved_interior` | `bool` | `true` | 阶段 3：使用增强多起点扫描 |
| `interior_n_restarts` | `int` | `4` | 阶段 3：额外起始位形数 |
| `interior_pair_coupling` | `bool` | `true` | 阶段 3：扫描时耦合配对关节 |
| `dual_phase3` | `bool` | `false` | 融合双阶段 3 模式（见下方） |
| `certified` | `bool` | `true` | 区间正运动学认证模式（见下方） |

**`dual_phase3`**：启用后，阶段 3 在单次解析调用中运行两次：
1. 从完整的阶段 0+1+2+2.5 累积状态出发（标准路径）。
2. 从阶段 0+1 检查点出发（绕过阶段 2 的"盆地偏移"）。

结果通过逐维度 min(lo)/max(hi) 合并。这消除了单独运行无阶段 2 通道的需要，避免了 2× 冗余的阶段 0/1 计算。在约 43 ms 平均时间下实现零遗漏 AABB（相比多通道方法 15× 加速）。参见 Exp-28。

**`certified`**：启用后，在解析求解器写入 AABB 后，计算区间正运动学外界并将每个面钳位到至少与区间正运动学结果一样极端。这无条件保证输出 AABB ⊇ 真实扫略体积（定理 3，认证保守性）。开销为一次 `compute_fk_full()` 调用（约 5 μs），叠加在约 43 ms 解析求解之上——相对成本不到 1%。

#### `AnalyticalCriticalStats`

| 字段 | 类型 | 说明 |
|---|---|---|
| `n_phase0_vertices` | `int` | 阶段 0 边界顶点评估数 |
| `n_phase1_edges` | `int` | 阶段 1：单自由度边临界点数 |
| `n_phase2_faces` | `int` | 阶段 2：双自由度面临界点数 |
| `n_phase25a_pair1d` | `int` | 阶段 2.5a 配对约束单自由度点数 |
| `n_phase25b_pair2d` | `int` | 阶段 2.5b 配对约束双自由度点数 |
| `n_phase3_interior` | `int` | 阶段 3+：内部临界点数 |
| `n_phase1_fk_calls` | `int` | 阶段 1 正运动学调用数 |
| `n_phase2_fk_calls` | `int` | 阶段 2 正运动学调用数 |
| `n_phase25_fk_calls` | `int` | 阶段 2.5 正运动学调用数 |
| `n_phase3_fk_calls` | `int` | 阶段 3+ 正运动学调用数 |
| `certified_max_gap` | `float` | 最大逐面间隙 \|ifk − analytical\|（仅认证模式） |
| `certified_gap_faces` | `int` | ifk ≠ analytical 的面数（仅认证模式） |

### GCPC 缓存 (`envelope/gcpc_cache.h` / `src/envelope/gcpc_cache.cpp`)

**全局临界点缓存（GCPC）**——预计算的 FK 位置内部临界点，存储在按连杆划分的 KD 树中。查询时，七阶段"缓存优先 + AA 剪枝"流水线生成紧凑、保守的 AABB，匹配或超过解析求解器的质量，同时通过仿射算术边界实现激进的早期退出。

#### 对称性简化

| 简化方式 | 效果 |
|---|---|
| q₀ 消除 | q₀ 在查询时通过 `atan2(B, A)` 重建；不存储 |
| q₁ 周期 π | 仅存储 q₁ ∈ [0, π]；查询时测试镜像副本 |
| q₆ 跳过 | 当 d₆ = a₆ = 0 且无工具时省略 q₆ |

#### `GcpcPoint`
一个缓存的内部临界点。

| 成员 | 类型 | 说明 |
|---|---|---|
| `q_eff[7]` | `double[7]` | 关节位形（q₁ … qₖ，q₁ ∈ [0,π]）。q₀ 不存储。 |
| `n_eff` | `int` | 使用的有效关节数 |
| `link_id` | `int` | 基于 0 的帧索引 |
| `direction` | `int` | 0 = xy（径向 R²），1 = z |
| `A`, `B` | `double` | 局部 FK 位置 x, y（T₀ 之后的帧） |
| `C` | `double` | 局部 FK 位置 z |
| `R` | `double` | 预计算径向距离 √(A² + B²) |

#### `GcpcLinkSection`
按连杆划分的缓存段，含用于范围查询的 KD 树。

| 成员 | 类型 | 说明 |
|---|---|---|
| `link_id` | `int` | 基于 0 的帧索引 |
| `n_eff_joints` | `int` | 有效自由度（q₀ 消除后） |
| `n_points` | `int` | 该连杆缓存的临界点数 |
| `q6_skipped` | `bool` | 是否跳过了 q₆ |
| `points` | `vector<GcpcPoint>` | 该连杆的所有缓存点 |
| `kd_tree` | `vector<KdNode>` | KD 树节点 |
| `kd_root` | `int` | 根节点索引 |

#### `GcpcQueryResult`
范围查询匹配的单个临界点。

| 成员 | 类型 | 说明 |
|---|---|---|
| `point` | `const GcpcPoint*` | 指向缓存点的指针 |
| `q0_optimal` | `double` | 重建的 q₀ |
| `q1_reflected` | `bool` | q₁ 是否被镜像（q₁ − π） |
| `q1_actual` | `double` | 实际使用的 q₁（镜像后） |

#### `GcpcQueryStats`
完整的逐阶段遥测数据，包括 AA 剪枝计数。

| 字段 | 类型 | 说明 |
|---|---|---|
| `n_cache_matches` | `int` | 阶段 A：缓存匹配点数 |
| `n_q1_reflected` | `int` | 来自 q₁ 镜像的点数 |
| `n_q0_valid` | `int` | q₀ 在范围内有效的点数 |
| `n_boundary_kpi2` | `int` | 阶段 B：kπ/2 顶点评估数 |
| `n_boundary_atan2` | `int` | 阶段 C：单自由度 atan2 解数 |
| `n_fk_calls` | `int` | 总正运动学评估数 |
| `n_aa_prune_checks` | `int` | AA 检查面数（二级剪枝） |
| `n_aa_pruned` | `int` | 被 AA 边界剪枝的面数 |
| `n_aa_not_pruned` | `int` | 通过 AA 剪枝的面数 |
| `n_phase_d_faces` | `int` | 阶段 D：双自由度面解数 |
| `n_phase_d_fk` | `int` | 阶段 D：正运动学调用数 |
| `n_phase_d_pruned` | `int` | 阶段 D：被 AA 剪枝的组合数 |
| `n_phase_e_pair1d` | `int` | 阶段 E：配对单自由度解数 |
| `n_phase_e_fk` | `int` | 阶段 E：正运动学调用数 |
| `n_phase_e_pruned` | `int` | 阶段 E：被 AA 剪枝数 |
| `n_phase_f_pair2d` | `int` | 阶段 F：配对双自由度解数 |
| `n_phase_f_fk` | `int` | 阶段 F：正运动学调用数 |
| `n_phase_f_pruned` | `int` | 阶段 F：被 AA 剪枝数 |
| `n_phase_g_interior` | `int` | 阶段 G：内部候选评估数 |
| `n_phase_g_fk` | `int` | 阶段 G：正运动学调用数 |
| `n_phase_g_pruned` | `int` | 阶段 G：被 AA 剪枝的连杆数 |
| `n_interior_added` | `int` | 富化：新增点数 |

#### `GcpcCache` 类

| 方法 | 返回值 | 说明 |
|---|---|---|
| `load_json(path, robot)` | `bool` | 从 JSON 加载（nlohmann/json），构建 KD 树 |
| `load(path)` | `bool` | 从二进制 GCPC 缓存文件加载 |
| `save(path)` | `bool` | 保存为二进制 GCPC 缓存文件 |
| `build(robot, points)` | `void` | 从一组临界点构建缓存 |
| `enrich_with_interior_search(robot, n_random=500, max_sweeps=5)` | `int` | 多起点坐标下降富化（见下方）。返回新增唯一点数。 |
| `query_link(link_id, intervals, results)` | `void` | KD 树范围查询 → 匹配的 `GcpcQueryResult` 向量 |
| `derive_aabb_with_gcpc(robot, intervals, n_sub, out, stats)` | `void` | 完整阶段 A–G 流水线 → 紧凑 AABB（见下方） |
| `is_loaded()` | `bool` | 缓存非空 |
| `n_links()` | `int` | 连杆段数 |
| `n_total_points()` | `int` | 总缓存临界点数 |
| `find_section(link_id)` | `const GcpcLinkSection*` | 给定连杆的段（或 `nullptr`） |
| `d0()` | `double` | 机器人基座 d₀（用于 z 重建） |

#### 阶段 A–G 流水线 (`derive_aabb_with_gcpc`)

流水线按成本递增顺序评估临界点候选，使用 AA 边界剪枝昂贵的后续阶段：

| 阶段 | 名称 | 说明 |
|---|---|---|
| **A** | 缓存查找 | KD 树范围查询 + q₀ 重建 + q₁ 镜像。从缓存的内部临界点建立强预界。 |
| **B** | kπ/2 顶点 | 枚举查询区间内每个关节在 kπ/2 处的所有边界顶点。 |
| **C** | 单自由度边 atan2 | 通过三点三角拟合 + atan2 求解 ∂R/∂qⱼ = 0。**两级 AA 剪枝**：一级（全盒）跳过整个连杆，二级（逐面）跳过单个 AABB 面。 |
| **D** | 双自由度面多项式 | 通过符号 8 阶多项式 + 区间限制二分法求解 ∂R/∂qⱼ = ∂R/∂qₖ = 0。连杆级 AA 剪枝。 |
| **E** | 配对约束单自由度 | 固定 qᵢ + qⱼ = kπ/2，求解单自由度 atan2。AA 剪枝。 |
| **F** | 配对约束双自由度 | 固定 qᵢ + qⱼ = kπ/2，扫描额外自由关节。AA 剪枝。 |
| **G** | 双内部下降 | 从 `best_face_config`（每 AABB 面的最佳位形）出发的坐标下降。运行两次：(1) 从阶段 A–F 最终状态出发，(2) 从阶段 A+B 检查点出发（避免阶段 C–F 的盆地偏移）。结果按逐维度 min/max 合并。 |

**双阶段 G** 镜像了解析求解器的 `dual_phase3` 机制：从两个不同起点运行消除了阶段 C–F 累积导致的"盆地偏移"，确保 GCPC ≥ Analytical 在所有区间宽度下成立。

#### 缓存富化 (`enrich_with_interior_search`)

预计算 R(q₁,…,qₖ) 和 pz(q₁,…,qₖ) 在全关节范围内的内部临界点，添加到 KD 树中以便阶段 A 给出紧凑的预界。

**种子：**
- 全关节范围上的 kπ/2 网格顶点
- `n_random_seeds`（默认 500）个均匀随机位形
- 配对约束种子（qᵢ + qⱼ = kπ/2）

**算法：** 多起点坐标下降（`max_sweeps` 次迭代），去重（容差 1e-6）。

**典型结果：** 每连杆 80–150 个唯一内部临界点（7-DOF IIWA14），使阶段 A 在任何昂贵的边界求解之前即可恢复 ≥99.9% 的最终 AABB 体积。

### 碰撞策略 (`collision_policy.h`)
| 函数 | 说明 |
|---|---|
| `check_collision(policy, frames, ...)` | 按 `CollisionPolicy` 枚举统一调度 |
| `collide_aabb(frames, ..., obs, n_obs)` | 早期退出 AABB 碰撞 |
| `collide_aabb_subdiv(frames, ..., n_sub, ...)` | 细分 AABB 碰撞 |
| `collide_grid(frames, ..., R, ...)` | 网格体素碰撞 |

### FrameStore (`frame_store.h`)
关节原点区间向量的持久存储（LECT 数据）。
| 方法 | 说明 |
|---|---|
| `store_from_fk(idx, fk)` | 存储节点的正运动学状态 |
| `get_frames(idx)` | 获取缓存帧数据 |
| `union_frames(dst, a, b)` | 合并两个节点的帧数据 |
| `save(path)` / `load(path)` | 文件持久化 |
| `create_mmap(path, cap)` / `load_mmap(path)` | 内存映射持久化 |

### GridStore (`grid_store.h`)
位域网格存储（32×32×32 = 32768 体素，每节点 512 个 uint64 字）。
| 方法 | 说明 |
|---|---|
| `derive_from_frames(idx, frames, n_sub)` | 从缓存帧构建网格 |
| `check_collision(idx, obs, n_obs)` | 基于网格的碰撞检测 |
| `union_grids(dst, a, b)` | 按位 OR 合并 |
| `save()` / `load()` / mmap 变体 | 持久化 |

### GridEnvelope / GridComputer (`grid_envelope.h`)
使用体素网格的 `IEnvelope` 实现。`GridComputer` 是对应的 `IEnvelopeComputer`。

### EnvelopeComputer (`envelope_computer.h`) *(旧版)*
`IntervalFKEnvelopeComputer` 包装 Robot + 区间正运动学进行 AABB 包络计算。

---

## 4. 场景 (`include/sbf/scene/`)

### `ICollisionChecker`（接口）
| 方法 | 说明 |
|---|---|
| `check_config(q)` | 点碰撞检测 |
| `check_box(intervals)` | 盒子碰撞检测 |
| `check_segment(q1, q2, step)` | 线段碰撞检测 |

### `AabbCollisionChecker`
使用 AABB 包络 + 障碍物重叠实现 `ICollisionChecker`。
别名：`CollisionChecker = AabbCollisionChecker`。

| 方法 | 返回值 | 说明 |
|---|---|---|
| `set_aa_crossover(w)` | `void` | 设置 `check_box()` 的混合 IA/AA 切换宽度（0 = 禁用） |
| `aa_crossover()` | `double` | 当前切换宽度 |

---

## 5. 体素 (`include/sbf/voxel/`)

### BitBrick (`bit_brick.h`)
8×8×8 = 512 体素瓦片，打包为 8 × `uint64_t`（64 字节 = 1 缓存行）。
布局：`word[z]`，位 = `y * 8 + x`。

| 方法 | 返回值 | 说明 |
|---|---|---|
| `set(x, y, z)` | `void` | 设置局部坐标处的体素 |
| `test(x, y, z)` | `bool` | 测试局部坐标处的体素 |
| `clear_voxel(x, y, z)` | `void` | 清除单个体素 |
| `clear()` | `void` | 清零所有字 |
| `is_empty()` | `bool` | 所有字是否为零 |
| `popcount()` | `int` | 计数已设置体素（MSVC 上使用 `__popcnt64`） |
| `operator\|` / `operator\|=` | `BitBrick` | 按位 OR（合并） |
| `operator&` | `BitBrick` | 按位 AND |
| `intersects(other)` | `bool` | 是否有重叠体素？（早期退出 AND） |

### BrickCoord / BrickCoordHash (`bit_brick.h`)
| 成员 | 类型 | 说明 |
|---|---|---|
| `bx`, `by`, `bz` | `int` | 整数瓦片地址 |
| `BrickCoordHash` | 函子 | FNV-1a 风格哈希，用于 `unordered_map` |

### VoxelGrid (`voxel_grid.h`)
由 `unordered_map<BrickCoord, BitBrick>` 支持的稀疏体素网格。

**构造函数：**

| 参数 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `delta` | `double` | — | 体素边长（米）（例如 0.02） |
| `ox`, `oy`, `oz` | `double` | `0.0` | 世界空间原点 |

安全填充：`√3 · Δ / 2`（体素立方体半对角线），自动计算。

**坐标辅助：**

| 方法 | 返回值 | 说明 |
|---|---|---|
| `to_cell(w, axis)` | `int` | 世界坐标 → 整数单元索引 |
| `cell_center(c, axis)` | `double` | 单元索引 → 单元中心世界坐标 |

**栅格化：**

| 方法 | 说明 |
|---|---|
| `fill_aabb(float[6])` | 填充所有与轴对齐盒子重叠的体素 |
| `fill_hull16(prox_iv, dist_iv, radius)` | Conv(B₁∪B₂)⊕Ball(r) 的高速扫描线栅格化。O(ny·nz)，无逐体素距离评估。 |
| `find_t_range_yz(qy, qz, r², ...)` | （静态）求解分段二次方程以获取 dist²_yz ≤ r² 的 t 范围 |

**体积查询：**

| 方法 | 返回值 | 说明 |
|---|---|---|
| `count_occupied()` | `int` | 总已设置体素数（所有砖块的 popcount） |
| `occupied_volume()` | `double` | 体积（m³）（`count_occupied * Δ³`） |
| `num_bricks()` | `int` | 已分配砖块数 |

**合并/碰撞：**

| 方法 | 返回值 | 说明 |
|---|---|---|
| `merge(other)` | `void` | 所有砖块按位 OR |
| `collides(other)` | `bool` | 在较小映射上早期退出 AND |
| `count_colliding(other)` | `int` | 重叠体素数 |
| `clear()` | `void` | 移除所有砖块 |

**访问器：** `delta()`、`safety_pad()`、`bricks()`。

### Hull 栅格化器 (`hull_rasteriser.h` / `src/voxel/hull_rasteriser.cpp`)
连接 Robot/FKState 与 VoxelGrid 的高级 API。

| 类型/函数 | 说明 |
|---|---|
| `PosInterval` | `float lo[3], hi[3]`——从前缀变换提取的位置区间 |
| `frame_pos(fk, frame_idx)` | 从 FKState 列 [3],[7],[11] 提取 (x,y,z) 位置区间 |
| `rasterise_robot_hull16(robot, fk, grid, n_sub=8)` | 通过 Hull-16 高速扫描线栅格化所有活动连杆包络 |
| `rasterise_robot_sub_aabbs(robot, fk, grid, n_sub=8)` | 通过逐连杆子 AABB 分解栅格化 |
| `rasterise_robot_aabbs(robot, fk, grid)` | 栅格化完整逐连杆 AABB（无细分） |
| `rasterise_box_obstacle(cx, cy, cz, hx, hy, hz, grid)` | 栅格化盒子障碍物 |

---

## 6. 森林 (`include/sbf/forest/`)

### 概览

森林模块构建无碰撞的位形空间盒子森林。两个构建器类：
- **`SafeBoxForest`**：随机采样 → FFB → 晋升（既有）
- **`ForestGrower`**：带波前/RRT 模式的多根定向扩展（新增）

### `GrowerConfig` (`grower_config.h`)

| 字段 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `n_roots` | `int` | `2` | 根盒子数量（FPS 选择） |
| `mode` | `Mode` | `Wavefront` | `Wavefront`（BFS 边界）或 `RRT`（随机延伸） |
| `max_boxes` | `int` | `500` | 最大盒子增长数 |
| `min_edge` | `double` | `0.01` | FFB 最小边长 |
| `max_depth` | `int` | `30` | FFB 最大 KD 树深度 |
| `max_consecutive_miss` | `int` | `200` | 连续 FFB 失败 N 次后停止 |
| `timeout` | `double` | `60.0` | 挂钟超时（秒） |
| `adjacency_tol` | `double` | `1e-10` | 面邻接测试容差 |
| `rng_seed` | `uint64_t` | `42` | 随机数生成器种子 |
| `n_boundary_samples` | `int` | `6` | 每盒子面的边界种子数（波前） |
| `boundary_epsilon` | `double` | `0.01` | 超出盒子面的偏移距离（波前） |
| `goal_face_bias` | `double` | `0.6` | 选择目标方向面的概率（波前） |
| `rrt_step_ratio` | `double` | `0.3` | 步长 = 比率 × 最大关节范围宽度（RRT） |
| `rrt_goal_bias` | `double` | `0.1` | 采样偏向目标的概率（按树：起点→目标，目标→起点，随机→50/50） |
| `n_threads` | `int` | `1` | 工作线程数（`1` = 串行，`>1` = 线程池并行） |
| `adaptive_min_edge` | `bool` | `false` | 启用粗→细两阶段最小边长策略 |
| `coarse_min_edge` | `double` | `0.1` | 粗阶段最小边长（启用 `adaptive_min_edge` 时） |
| `coarse_fraction` | `double` | `0.6` | 使用粗最小边长的预算比例（0.0–1.0） |
| `bridge_samples` | `int` | `0` | 子树边界共享面处的种子数（`0` = 禁用） |
| `coarsen_enabled` | `bool` | `false` | 扩展后启用贪心粗化（凸包合并相邻对） |
| `coarsen_target_boxes` | `int` | `0` | 盒子数 ≤ 目标时停止粗化（`0` = 尽量减少） |
| `max_coarsen_rounds` | `int` | `50` | 每次 grow() 最大贪心粗化轮数 |
| `coarsen_score_threshold` | `double` | `50.0` | hull_vol/sum_vol > 阈值时跳过合并候选 |
| `warm_start_depth` | `int` | `0` | 启动工作线程前预扩展 LECT 骨架深度。`0` = 自动（深度 3），`-1` = 禁用 |
| `pipeline` | `PipelineConfig` | `recommended()` | LECT 包络流水线 |

### `AdjacencyGraph` (`adjacency_graph.h` / `adjacency_graph.cpp`)

扫描线排序加速的盒子邻接图。从 v2 的 `deoverlap.cpp` 移植。

**加速技术：**
1. 按填充率维度排序 → 最佳扫描/过滤维度
2. 扫描线排序，在扫描维度排序中断
3. 双维度预过滤后再进行全维度检查
4. 行主序平坦布局以优化缓存访问
5. 窄→宽维度排序以实现早期退出

| 方法 | 返回值 | 说明 |
|---|---|---|
| `AdjacencyGraph(limits, tol)` | — | 带关节限位的构造函数 |
| `add_box(box)` | `void` | 增量插入：O(log N + K) |
| `remove_box(id)` | `void` | 移除盒子及其所有边 |
| `clear()` | `void` | 移除所有盒子 |
| `rebuild(boxes)` | `void` | 完整批量重建：O(N·K) |
| `neighbors(id)` | `const vector<int>&` | 相邻盒子 ID |
| `connected(a, b)` | `bool` | BFS 可达性测试 |
| `n_components()` | `int` | 连通分量数 |
| `components()` | `vector<vector<int>>` | 所有连通分量 |
| `find_nearest_box(q)` | `int` | 中心最近 q 的盒子。使用内部 KD 树+子树 AABB 剪枝——O(log N) 平均，变异时惰性重建 |
| `n_boxes()` | `int` | 图中盒子数 |
| `n_edges()` | `int` | 邻接边数 |
| `sweep_dim()` | `int` | 当前扫描维度 |
| `filter_dim()` | `int` | 当前过滤维度 |

### `ThreadPool` (`thread_pool.h`，仅头文件)

轻量级 C++17 线程池，基于 `std::future` 的任务提交。

| 方法 | 返回值 | 说明 |
|---|---|---|
| `ThreadPool(n_threads)` | — | 创建具有 `n_threads` 工作线程的池 |
| `submit(f, args...)` | `std::future<R>` | 提交可调用对象进行异步执行；返回 future |
| `size()` | `int` | 工作线程数 |
| `~ThreadPool()` | — | 加入所有工作线程；清空剩余任务 |

不可复制、不可移动。由 `ForestGrower::grow_parallel()` 内部使用。

### `ForestGrower` (`forest_grower.h` / `forest_grower.cpp`)

多根森林增长引擎，含两种扩展模式和可选线程池并行。

**串行流水线：** 根选择 → 子树分区 → 扩展（波前/RRT）→ 晋升 → 最终邻接重建。

**并行流水线**（`n_threads > 1`）：根选择 → 子树分区 → **启动各子树工作线程**（各有独立 LECT + RNG）→ **合并盒子** → 可选边界桥接 → 最终邻接重建。

| 方法 | 返回值 | 说明 |
|---|---|---|
| `ForestGrower(robot, config)` | — | 构造函数 |
| `ForestGrower(robot, config, warm_lect)` | — | 热启动构造函数：移入预构建的 LECT 骨架（供并行工作线程使用） |
| `set_endpoints(start, goal)` | `void` | 强制设定起点/目标为根 0 和根 1 |
| `get_bias_target(root_id)` | `const VectorXd*` | 逐树偏向目标：根 0→目标，根 1→起点，根≥2→随机 50/50。无端点时返回 `nullptr`。 |
| `grow(obstacles, n_obs)` | `GrowerResult` | 主入口：根据 `n_threads` 调度串行或并行 |
| `grow_subtree(seed, root_id, limits, obs, n_obs, shared_box_count)` | `GrowerResult` | 在给定位形空间子区域内增长单棵子树。可选 `shared_box_count` 启用工作窃取预算。 |
| `lect()` | `const LECT&` | 访问底层 LECT 树 |
| `lect_mut()` | `LECT&` | LECT 的可变访问 |
| `graph()` | `const AdjacencyGraph&` | 访问邻接图 |
| `boxes()` | `const vector<BoxNode>&` | 森林中所有盒子 |
| `config()` | `const GrowerConfig&` | 配置 |
| `n_boxes()` | `int` | 盒子数量 |
| `is_grid_envelope()` | `bool` | 包络类型是否使用体素网格 |

**根选择：**
- 无端点：最远点采样（FPS），K=50 候选
- 有端点：起点/目标强制为根 0/1，其余根沿起点→目标线段 FPS

**子树分区：**
根选择后，位形空间沿循环维度递归二分，直到每个单元最多包含一个根。每根的子区域被存储并用于约束波前回退和 RRT 目标选择的随机采样。

**波前扩展：**
基于 BFS 队列的边界扩展。对每个盒子，在有效面上生成边界种子（排除入口面）。面选择通过 `get_bias_target(root_id)` 使用**逐树偏向**。队列为空时，使用 `sample_near_existing_boundary()` 的**边界感知回退**保持新盒子与树相邻（避免孤立碎片）。在自适应粗阶段，尝试最多 30 次边界重新填充采样后强制过渡到细阶段。

**RRT 扩展：**
采样随机目标（受子树约束），找到最近盒子，然后使用 `rrt_snap_to_face()` 的**边界吸附**：将种子放置在最近盒子最对齐方向的面上（70% 定向 + 30% 随机抖动）。这确保新盒子始终与树相邻，消除孤立小盒子。以概率 `rrt_goal_bias`，随机盒子的树确定逐树偏向目标。步长 = `rrt_step_ratio × max_joint_range_width`。

**晋升：**
迭代自下而上叶子合并（与 `SafeBoxForest` 相同）。根据包络类型调度 AABB 或凸包网格碰撞检测。

**并行扩展**（`n_threads > 1`）：
当 `config.n_threads > 1` 且子树 ≥ 2 时，`grow()` 调度到 `grow_parallel()`。每棵子树分配一个工作线程：
- **独立 ForestGrower 实例**（自有 LECT、RNG、盒子向量）——零共享可变状态
- **工作窃取预算**：共享 `std::atomic<int>` 计数器跟踪全局盒子数。工作线程调用 `fetch_add(1)` 并在 `total >= max_boxes` 时停止——动态负载均衡而非静态逐工作线程预算
- **O(1) 盒子查找**：`box_id_to_idx_` 哈希映射替代扩展循环中的线性搜索
- **确定性 RNG**：工作线程 `i` 获得种子 `rng_seed + i * 12345 + 1`

所有工作线程完成后，主线程：
1. 合并所有工作线程盒子，全局重分配 ID
2. 重映射 `parent_box_id` 引用
3. 可选运行**边界桥接**（`bridge_samples > 0`）：在子树区域共享面附近采样种子并创建桥接盒子
4. 通过包含检查识别起点/目标盒子
5. 在所有合并盒子上重建最终邻接图

**LECT 热启动**（`warm_start_depth >= 0`）：
启动并行工作线程前，协调器预扩展 LECT 树到指定深度（默认自动 = 3）。每个工作线程接收快照（深拷贝，清除占用标记）。避免前几层树级别的冗余正运动学 + 包络计算。实测深度 3 下 7→15 个预构建节点。

**边界桥接**（`bridge_samples > 0`）：
并行合并后，在子树分区区域的共享面附近采样种子。创建交替 `root_id` 的桥接盒子以建立跨子树邻接。包括 `sync_lect_occupation()` 以防止重叠盒子。

**自适应最小边长**（`adaptive_min_edge = true`）：
两阶段扩展策略。第一阶段（预算的前 `coarse_fraction`），使用 `coarse_min_edge`（默认 0.1）以大盒子快速覆盖。过渡点：
- `miss_count` 重置为零（细最小边长开启许多新 FFB 站点）
- 波前队列用所有现有盒子重新填充（以细最小边长重新探索边界）
- RRT 以较小步长容差无缝继续

实测 2.3× 总体积改进（80 个盒子，相同种子）。串行和并行模式均支持；并行工作线程聚合粗/细计数。

**基准测试**（7-DOF iiwa14，4 根，200 盒子，4 线程）：
- 第 4 阶段：串行 169 ms → 并行 60 ms = 2.83× 加速
- **第 5 阶段：串行 165 ms → 并行 48 ms = 3.41× 加速**（工作窃取 + O(1) 查找 + 自适应最小边长）

### `GrowerResult`

| 字段 | 类型 | 说明 |
|---|---|---|
| `boxes` | `vector<BoxNode>` | 森林中所有盒子 |
| `n_roots` | `int` | 根盒子数 |
| `n_boxes_total` | `int` | 总盒子数 |
| `n_ffb_success` | `int` | 成功 FFB 调用数 |
| `n_ffb_fail` | `int` | 失败 FFB 调用数 |
| `n_promotions` | `int` | 叶子晋升数 |
| `n_bridge_boxes` | `int` | 子树边界处创建的桥接盒子数 |
| `n_coarse_boxes` | `int` | 粗阶段创建的盒子数（自适应最小边长） |
| `n_fine_boxes` | `int` | 细阶段创建的盒子数（自适应最小边长） |
| `n_coarsen_merges` | `int` | 贪心粗化合并执行数 |
| `n_components` | `int` | 邻接图中的连通分量数 |
| `start_goal_connected` | `bool` | 起点和目标盒子是否在同一分量中 |
| `total_volume` | `double` | 所有盒子体积之和 |
| `build_time_ms` | `double` | 挂钟构建时间 |
| `phase_times` | `map<string,double>` | 逐阶段计时（root_select_ms、expand_ms、adj_rebuild_ms、warm_start_ms、coarsen_ms） |

---

## 7. 可视化 (`include/sbf/viz/` + `python/sbf_viz/`)

架构：**C++ JSON 导出 → Python Plotly 交互 HTML**。

### C++ JSON 导出器 (`viz_exporter.h` / `src/viz/viz_exporter.cpp`)

| 函数 | 说明 |
|---|---|
| `export_robot_json(robot, configs, path)` | 导出多个位形的机器人正运动学连杆位置 |
| `export_robot_json(robot, config, path)` | 单位形便捷重载 |
| `export_envelope_json(robot, store, node_indices, n_sub, path)` | 从 FrameStore 导出各节点 AABB 连杆包络 |
| `export_envelope_from_boxes_json(robot, boxes, n_sub, path)` | 导出位形空间盒子的包络（内部计算正运动学） |
| `export_voxel_json(grid, path)` | 导出 VoxelGrid 为砖块级十六进制数据 |
| `export_voxel_centres_json(grid, path)` | 导出 VoxelGrid 为显式占用单元中心 |
| `export_scene_json(scene, path)` | 导出场景障碍物 |
| `export_snapshot_json(robot, config, box, robot_grid, obs_grid, scene, n_sub, path)` | 含所有层的组合快照 |

### Python 可视化包 (`python/sbf_viz/`)

| 模块 | 主要函数 | 说明 |
|---|---|---|
| `load_data` | `load_robot_json()` 等 | JSON 加载器，返回类型化数据类 |
| `robot_viz` | `plot_robot_3d(data)` | 多位形叠加的 3D 手臂轨迹 + 关节标记 |
| `envelope_viz` | `plot_envelope_3d(data)` | 逐连杆着色的 AABB 网格 + 线框 |
| `voxel_viz` | `plot_voxel_3d(data, mode)` | 散点或立方体网格体素渲染 |
| `scene_viz` | `plot_scene_3d(data)` | 障碍物网格 + 线框 |
| `combined_viz` | `plot_snapshot_3d(data)` | 带切换按钮的统一多层查看器 |
| `run_demo` | CLI | 读取 6 个 JSON 文件 → 生成 6 个 HTML 文件 |

**使用方法（CLI）：**
```bash
# 1. C++ 导出 JSON
viz_demo <robot.json> <output_dir>

# 2. Python 渲染 HTML
cd python && python -m sbf_viz.run_demo <output_dir>
```

---

## 8. 2D ForestGrower 可视化 (`src/viz/`)

纯 Python 2D 可视化包，在带随机盒子障碍物的 2D 位形空间上模拟和动画化 ForestGrower 算法。用于算法理解、调试和论文配图。

**架构：** `GrowVizConfig` → `ForestGrower2D` → 快照 → `render` → PNG 帧 → GIF

**主要特性（2026-03-17）：**
- RRT 边界吸附：新盒子吸附到最近盒子的边界面，避免散布小盒子
- 全空间 FPS 根选择：根在整个位形空间均匀分布
- 自适应两阶段（粗→细）：先用大盒子填充空间，再用细盒子填补间隙
- `PatchCollection` + `LineCollection` 加速渲染（约 2× 加速）
- 默认 `snapshot_every=1`（每个盒子记录），CLI `--snap-every 2`

### `GrowVizConfig` (`src/viz/core.py`)

与 C++ `GrowerConfig` 参数对齐的配置数据类。

| 字段 | 类型 | 默认值 | 说明 |
|---|---|---|---|
| `seed` | `int` | `42` | 随机种子 |
| `q_lo`, `q_hi` | `(float, float)` | `(-π, -π)`, `(π, π)` | 位形空间边界 |
| `q_start`, `q_goal` | `list[float]` 或 `None` | 已设定 | 起点/目标位形 |
| `n_obstacles` | `int` | `10` | 随机障碍物数 |
| `mode` | `str` | `"wavefront"` | `"wavefront"` 或 `"rrt"` |
| `n_roots` | `int` | `3` | 根树数量 |
| `max_boxes` | `int` | `300` | 总盒子预算 |
| `max_consecutive_miss` | `int` | `150` | 停止前的失败计数 |
| `min_edge` | `float` | `0.03` | FFB 细最小边长 |
| `max_depth` | `int` | `20` | FFB 最大 KD 树深度 |
| `n_boundary_samples` | `int` | `6` | 每盒子面的种子数 |
| `boundary_epsilon` | `float` | `0.02` | 面偏移距离 |
| `goal_face_bias` | `float` | `0.6` | 目标方向面优先级 |
| `rrt_step_ratio` | `float` | `0.15` | RRT 步长 = 比率 × 范围 |
| `rrt_goal_bias` | `float` | `0.1` | RRT 目标采样概率 |
| `adaptive_min_edge` | `bool` | `False` | 启用粗→细两阶段 |
| `coarse_min_edge` | `float` | `0.25` | 第一阶段最小边长（8× 更粗） |
| `coarse_fraction` | `float` | `0.5` | 粗阶段预算比例 |
| `n_threads` | `int` | `1` | 模拟轮转线程数 |
| `snapshot_every` | `int` | `1` | 每盒子帧数（1 = 每个盒子） |
| `collision_resolution` | `float` | `0.025` | 碰撞地图像素大小 |
| `dpi` | `int` | `150` | PNG DPI |
| `gif_frame_ms` | `int` | `200` | GIF 帧持续时间（毫秒） |
| `fig_size` | `(float, float)` | `(10, 8)` | 图形尺寸（英寸） |

### `BoxInfo` (`src/viz/core.py`)

| 字段 | 类型 | 说明 |
|---|---|---|
| `box_id` | `int` | 唯一 ID |
| `lo`, `hi` | `ndarray` | AABB 边界 |
| `seed` | `ndarray` | 种子位形 |
| `parent_box_id` | `int` | 扩展父节点（-1 = 根） |
| `expand_face_dim` | `int` | 使用的面维度 |
| `expand_face_side` | `int` | 0 = 下, 1 = 上 |
| `root_id` | `int` | 根树成员关系 |
| `is_coarse` | `bool` | 在粗阶段创建 |
| `volume` | `float` | （属性）盒子体积 |
| `center()` | `ndarray` | 盒子中心 |
| `is_adjacent(other)` | `bool` | 面邻接测试 |

### `FFBEngine` (`src/viz/ffb_engine.py`)

使用 KD 树和自适应 `_effective_min_edge` 的 2D 查找自由盒子引擎。

| 方法 | 返回值 | 说明 |
|---|---|---|
| `FFBEngine(cspace, min_edge, max_depth)` | — | 构造函数 |
| `find_free_box(seed)` | `(lo, hi)` 或 `None` | 下降并分裂以找到无碰撞叶子 |
| `mark_box_id(lo, hi, bid)` | `None` | 将叶子标记为已占用 |
| `set_effective_min_edge(me)` | `None` | 更改最小边长阈值（自适应两阶段） |

### `ForestGrower2D` (`src/viz/forest_grower_2d.py`)

模拟 C++ `ForestGrower` 算法的 2D 多根森林增长器。

| 方法 | 返回值 | 说明 |
|---|---|---|
| `ForestGrower2D(cspace, cfg)` | — | 构造函数 |
| `grow(snapshot_every=1)` | `list[dict]` | 运行增长，返回快照 |

**扩展模式：**

| 模式 | 配置 | 说明 |
|---|---|---|
| 波前 | `mode="wavefront"` | 带目标面偏向的 BFS 边界扩展 |
| RRT | `mode="rrt"` | 随机目标 + 最近盒子边界吸附延伸 |
| 多线程 | `n_threads>1` | 逐子树轮转模拟 |
| 自适应 | `adaptive_min_edge=True` | 粗阶段 → 细阶段两级 |

### 渲染 (`src/viz/render.py`)

| 函数 | 说明 |
|---|---|
| `_draw_boxes_on_ax(ax, snap, ...)` | 在 matplotlib 轴上绘制盒子 + 邻接边。使用 `PatchCollection` + `LineCollection` 提升性能。 |
| `_build_title(snap, cfg, frame_idx)` | 构建包含模式/线程/自适应阶段信息的标题字符串。 |
| `plot_snapshot(snap, cmap, extent, cfg, idx, subtrees)` | 渲染单帧快照 → `Figure`。 |
| `compose_gif(frames_dir, gif_path, duration_ms)` | 组装 PNG 帧 → 动画 GIF。最后一帧保持 2 秒。 |

**配色方案：**
- 逐根固定颜色，取自 `ROOT_COLORS`（10 种颜色）
- 粗盒子：虚线边框，20% 透明度
- 细盒子：实线边框，35% 透明度
- 最新盒子：红色 (#ff0000)，55% 透明度

### `plot_4panel(...)` (`src/viz/compare_expansion.py`)

| 参数 | 类型 | 说明 |
|---|---|---|
| `snaps` | `list[dict]` | 4 个快照字典（每模式一个） |
| `cfgs` | `list[GrowVizConfig]` | 4 个配置 |
| `subtrees_list` | `list[list]` | 逐模式子树分区 |
| `labels` | `list[str]` | 面板标签 |
| `cmap_data` | `ndarray` | 碰撞地图 |
| `extent` | `list` | 绘图范围 |
| `frame_idx` | `int` | 帧索引 |
| `seed` | `int` | 随机种子（用于标题） |

返回包含 2×2 子图的 `Figure`，比较所有 4 种扩展模式。

### CLI 入口

```bash
# 单模式增长动画
python -m src.viz.forest_grower_2d \
    --mode wavefront --seed 42 --max-boxes 300 --snap-every 2 \
    [--adaptive] [--n-threads 3] [--no-endpoints]

# 4 模式对比动画（波前 / RRT / 多线程 / 自适应）
python -m src.viz.compare_expansion \
    --seed 42 --max-boxes 200 --snap-every 2 --dpi 80

# 多进程可视化
python -m src.viz.multiprocess \
    --seed 42 --n-roots 3 --max-boxes 300

# 单波前步骤详情
python -m src.viz.wavefront_step \
    --seed 42 --pre-grow 20
```

**输出结构**（`results/viz_compare_<timestamp>/`）：
```
├── compare_4panel.gif     — 2×2 动画对比
├── final_compare.png      — 最终帧静态图
├── scene.json             — 障碍物几何
├── summary.json           — 逐模式计时和盒子数
├── README.md              — 自动生成的摘要
└── frames/                — 单帧 PNG
    ├── frame_0000.png
    ├── frame_0001.png
    └── ...
```
