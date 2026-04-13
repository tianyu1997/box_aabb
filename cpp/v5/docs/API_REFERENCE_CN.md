# SafeBoxForest v5 API 参考（中文）

> 最后更新：2026-04-10

---

## 目录

- [总体架构](#总体架构)
- [core/types — 核心类型](#coretypes)
- [core/robot — 机器人模型](#corerobot)
- [scene/collision\_checker — 碰撞检测](#scenecollision_checker)
- [envelope/endpoint\_source — 端点源统一调度](#envelopeendpoint_source)
- [lect/lect — 连杆包络碰撞树 (LECT)](#lectlect)
- [ffb/ffb — Find Free Box (FFB)](#ffbffb)
- [forest/grower — 森林生长器](#forestgrower)
- [forest/adjacency — 邻接图](#forestadjacency)
- [forest/coarsen — 粗化](#forestcoarsen)
- [forest/connectivity — 连通性与桥接](#forestconnectivity)
- [forest/thread\_pool — 线程池](#forestthread_pool)
- [planner/dijkstra — A\* 图搜索](#plannerdijkstra)
- [planner/path\_extract — 路径提取](#plannerpath_extract)
- [planner/path\_smoother — 路径优化](#plannerpath_smoother)
- [planner/gcs\_planner — GCS 规划器](#plannergcs_planner)
- [planner/sbf\_planner — 顶层规划器](#plannersbf_planner)
- [Python 绑定 (sbf5)](#python-绑定-sbf5)

---

## 总体架构

SBF v5 是两阶段运动规划系统：

```
离线 build_coverage():
  模式 A (connect_mode=true, 推荐):
    LECT 加载/构建 → grow_coordinated(协调多树生长 + 内联桥接)
    → promote → coarsen (可选) → compute_adjacency

  模式 B (connect_mode=false, 传统):
    LECT 加载/构建 → grow_parallel(N棵种子树) → promote
    → coarsen_forest (维度扫描) → coarsen_greedy (邻接贪心)
    → filter_coarsen_overlaps → bridge_all_islands (并行RRT)
    → compute_adjacency → lect_save_incremental

在线 query(start, goal):
  locate_box → bridge_s_t → proxy_search → dijkstra(A*)
  → extract_waypoints → pre-compete simplify → multi-trial RRT竞争
  → segment_validate → greedy_simplify → shortcut → densify
  → elastic_band (60 iter + per-segment回退)
  → final_shortcut → pass2 (mini EB + simplify)
  → emergency_rrt (安全网)
```

---

## core/types

> 头文件：`include/sbf/core/types.h`

### 常量

| 名称 | 值 | 说明 |
|------|-----|------|
| `MAX_JOINTS` | 32 | 支持的最大关节数 |
| `SAT_EPS` | 1e-10 | SAT 检测容差 |
| `CONTAIN_TOL` | 1e-10 | 包含性检测容差 |

### `Interval` — 闭区间

```cpp
struct Interval {
    double lo, hi;
    double width() const;
    double center() const;
    bool empty() const;
    bool contains(double v, double tol = CONTAIN_TOL) const;
    bool overlaps(const Interval& other, double tol = 0.0) const;
    Interval hull(const Interval& o) const;      // 凸包
    Interval intersect(const Interval& o) const;  // 交集
    // 区间算术: operator+, -, *, *(double)
};
```

### `Obstacle` — 3D 工作空间 AABB

```cpp
struct Obstacle {
    float bounds[6]; // [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
    Obstacle(float lx, float ly, float lz, float hx, float hy, float hz);
};
```

### `BoxNode` — C-space 安全盒

```cpp
struct BoxNode {
    int id;
    std::vector<Interval> joint_intervals;  // 每关节 [lo, hi]
    Eigen::VectorXd seed_config;            // 创建此盒的种子配置
    double volume;                          // 各维宽度之积
    int tree_id;                            // LECT 叶节点索引
    int parent_box_id;                      // 父盒 ID (-1 = root)
    int root_id;                            // 子树根 ID

    int n_dims() const;
    Eigen::VectorXd center() const;
    void compute_volume();
    bool contains(const Eigen::VectorXd& q, double tol) const;
};
```

---

## core/robot

> 头文件：`include/sbf/core/robot.h`

### `Robot` — DH 参数化串联机器人

```cpp
class Robot {
public:
    static Robot from_json(const std::string& path);

    const std::string& name() const;
    int n_joints() const;
    const std::vector<DHParam>& dh_params() const;
    const JointLimits& joint_limits() const;
    int n_active_links() const;
    int n_active_endpoints() const;
    const int* active_link_map() const;
    bool has_link_radii() const;
    uint64_t fingerprint() const;  // FNV-1a 64位指纹（标识LECT缓存）
};
```

---

## scene/collision\_checker

> 头文件：`include/sbf/scene/collision_checker.h`

### `CollisionChecker` — 碰撞检测器

```cpp
class CollisionChecker {
public:
    CollisionChecker();
    CollisionChecker(const Robot& robot, const std::vector<Obstacle>& obstacles);
    void set_obstacles(const Obstacle* obs, int n_obs);

    bool check_config(const Eigen::VectorXd& q) const;        // 单构型碰撞
    bool check_box(const std::vector<Interval>& ivs) const;   // 区间盒碰撞
    bool check_segment(const Eigen::VectorXd& a,               // 线段碰撞
                       const Eigen::VectorXd& b,
                       int resolution = 10) const;

    const Robot& robot() const;
    int n_obs() const;
};
```

> **线程安全**：`CollisionChecker` 无可变状态，多线程共享安全（const 方法）。

---

## envelope/endpoint\_source

> 头文件：`include/sbf/envelope/endpoint_source.h`
> 实现：`src/envelope/endpoint_source.cpp`

### `EndpointSource` 枚举

Stage 1 端点源类别，4 种 iAABB 计算路径：

| 枚举值 | 编号 | 质量等级 | 安全类 | 说明 |
|--------|------|---------|------|------|
| `IFK` | 0 | 0 | SAFE | 区间 FK（最快，$\supseteq$ 真实可达集） |
| `CritSample` | 1 | 1 | UNSAFE | 边界 $k\pi/2$ 采样（采样可能遗漏极值点） |
| `Analytical` | 2 | 2 | SAFE | 解析梯度零点枚举 + AA 剪枝 |
| `GCPC` | 3 | 3 | SAFE | 解析边界 + 全局缓存内部（推荐） |

> **质量排序**：IFK(0) < CritSample(1) < Analytical(2) < GCPC(3)。
> **安全类**：SAFE 产出保守外界（iAABB ⊇ 真实可达集）；UNSAFE (CritSample) 因采样可能遗漏极值。

### `EndpointSourceConfig`

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `source` | `EndpointSource` | `IFK` | 端点源选择 |
| `n_samples_crit` | `int` | `1000` | CritSample 采样数 |
| `max_phase_analytical` | `int` | `3` | Analytical 最大阶段 (0..3) |
| `gcpc_cache` | `const GcpcCache*` | `nullptr` | GCPC 缓存指针（不拥有生命周期） |

### `EndpointIAABBResult`

| 字段 | 类型 | 说明 |
|------|------|------|
| `endpoint_iaabbs` | `vector<float>` | `[n_active × 2 × 6]`，配对端点 iAABBs |
| `source` | `EndpointSource` | 实际使用的端点源 |
| `is_safe` | `bool` | 结果是否保守安全 |
| `n_active_links` | `int` | 活跃连杆数 |
| `fk_state` | `FKState` | FK 状态（IFK 时有效） |
| `n_pruned_links` | `int` | AA Gap Pruning 完全剪枝的连杆数 |

### 源替代矩阵 (`kSourceSubstitutionMatrix`)

缓存数据可否服务不同源的请求：

| 缓存 \ 请求 | IFK | CritSample | Analytical | GCPC |
|-------------|-----|-----------|-----------|------|
| **IFK** | ✓ | ✓ | ✗ | ✗ |
| **CritSample** | ✗ | ✓ | ✗ | ✗ |
| **Analytical** | ✓ | ✓ | ✓ | ✗ |
| **GCPC** | ✓ | ✓ | ✓ | ✓ |

> IFK 可服务 CritSample 请求（IFK 比 CritSample 更保守）。
> CritSample 不可服务 IFK 请求（CritSample 可能遗漏极值）。

### 通道映射

```cpp
int source_channel(EndpointSource s);
// IFK → CH_SAFE (0),  其余 → CH_UNSAFE (1)
```

### 主要函数

```cpp
EndpointIAABBResult compute_endpoint_iaabb(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const EndpointSourceConfig& config,
    FKState* fk = nullptr,
    int changed_dim = -1);

bool source_can_serve(EndpointSource cached, EndpointSource requested);
void hull_endpoint_iaabbs(float* dst, const float* src, int n_endpoints);
```

---

## lect/lect

> 头文件：`include/sbf/lect/lect.h`
> 实现：`src/lect/lect.cpp`

### 设计概述

LECT 是位形空间上的惰性 KD 树，缓存每节点双通道连杆包络数据。

**双通道架构（v5 新增）：**

| 通道 | 常量 | 源 | 语义 |
|------|------|----|------|
| CH\_SAFE | 0 | IFK | 保守过近似，$\supseteq$ 真实可达集 |
| CH\_UNSAFE | 1 | CritSample / Analytical / GCPC | 采样/解析，更紧密 |

每节点可同时持有两个通道的端点 iAABB 数据。碰撞查询使用
`derive_merged_link_iaabb()` 从两通道取逐元素紧界（hull）。

**运行时源覆盖**：LECT 缓存文件记录了生成时的端点源。运行时可通过
`set_ep_config()` 切换端点源，后续 `expand_leaf()` 对新建节点使用覆盖后的源，
已缓存节点保留原有通道数据。典型用法：加载 CritSample 缓存获取广覆盖，
运行时使用 IFK 的增量快速路径扩展新节点。

### `ChannelData`

每通道的平铺缓冲存储：

| 字段 | 类型 | 说明 |
|------|------|------|
| `ep_data` | `vector<float>` | `[capacity × ep_stride]` 端点 iAABB |
| `source_quality` | `vector<uint8_t>` | `[capacity]` EndpointSource 枚举 |
| `has_data` | `vector<uint8_t>` | `[capacity]` 0/1 标志 |

### `GridSlot`

每节点网格槽元数据：

| 字段 | 类型 | 说明 |
|------|------|------|
| `type` | `EnvelopeType` | 网格类型 |
| `delta` | `float` | 体素边长 |
| `channel` | `uint8_t` | CH\_SAFE 或 CH\_UNSAFE |

### `LECT` 类

#### 构造

```cpp
LECT(const Robot& robot,
     const std::vector<Interval>& root_intervals,
     const EndpointSourceConfig& ep_config,
     const EnvelopeTypeConfig& env_config,
     int initial_cap = 1023);
```

#### 元数据

```cpp
int n_nodes() const;
int n_dims() const;
int n_active_links() const;
int capacity() const;
const Robot& robot() const;
const EndpointSourceConfig& ep_config() const;
const EnvelopeTypeConfig& env_config() const;
```

#### 运行时源覆盖

```cpp
void set_ep_config(const EndpointSourceConfig& cfg);
```

覆盖端点源配置。用于加载缓存后恢复用户指定的端点源。
**调用时机**：在 `lect_load_binary()` 之后，因为 `set_lect_state()`
会从缓存文件头恢复生成时的端点源，覆盖用户配置。
规划器内部在以下 3 处自动调用：`build()`、`warmup()`、`build_coverage()`。

#### 双通道数据访问

```cpp
bool has_data(int i) const;       // CH_SAFE 或 CH_UNSAFE 任一有数据
bool has_safe_data(int i) const;  // CH_SAFE 有数据
bool has_unsafe_data(int i) const;// CH_UNSAFE 有数据

const float* get_endpoint_iaabbs(int i, int channel = CH_SAFE) const;
const float* get_link_iaabbs(int i) const;  // 惰性合并两通道紧界

EndpointSource source_quality(int i, int channel = CH_SAFE) const;
```

#### 树结构访问

```cpp
int left(int i) const;
int right(int i) const;
int parent(int i) const;
int depth(int i) const;
bool is_leaf(int i) const;
int get_split_dim(int i) const;
double split_val(int i) const;
std::vector<Interval> node_intervals(int node_idx) const;
```

#### 核心操作

```cpp
// 展开叶节点（无 FK 状态——从头计算）
int expand_leaf(int node_idx);

// 展开叶节点（带 FK 状态——增量计算）
int expand_leaf(int node_idx, const FKState& fk,
                const std::vector<Interval>& intervals);

// 计算节点包络
void compute_envelope(int node_idx, const FKState& fk,
                      const std::vector<Interval>& intervals,
                      int changed_dim = -1,
                      int parent_idx = -1);
```

`compute_envelope` 三条执行路径：
1. **IFK 快速路径**（IFK 源 + `fk.valid`）：直接从 FK 前缀矩阵提取端点，
   部分继承未受分裂影响的连杆数据，40–45% FK 节省。
2. **Z4 缓存查找**（非规范扇区）：从缓存变换得到端点。
3. **通用回退**：调用 `compute_endpoint_iaabb()`。

#### 碰撞查询

```cpp
bool collides_scene(int node_idx,
                    const Obstacle* obs, int n_obs) const;

// 双层碰撞：iAABB 粗筛 → Grid 精炼
bool collides_scene(int node_idx,
                    const Obstacle* obs, int n_obs,
                    const std::unordered_map<float, voxel::SparseVoxelGrid>& obs_grids) const;

bool intervals_collide_scene(const std::vector<Interval>& intervals,
                             const Obstacle* obs, int n_obs) const;
```

#### 占用管理

```cpp
void mark_occupied(int node_idx, int box_id);
void unmark_occupied(int node_idx);
bool is_occupied(int node_idx) const;
int  forest_id(int node_idx) const;
int  subtree_occ(int node_idx) const;
void clear_all_occupation();
void clear_forest_state();

/// 点查询：O(depth) 从根到叶遍历，判断 q 是否落入已占用区域
bool is_point_occupied(const Eigen::VectorXd& q) const;

/// 点标记：O(depth) 从根到叶遍历，将包含 q 的叶标记为已占用
/// 注意：仅适用于 worker LECT（master LECT 节点太少，标记会封锁大片区域）
void mark_point_occupied(const Eigen::VectorXd& q);
```

#### 切分策略

```cpp
void set_split_order(SplitOrder so);
SplitOrder split_order() const;
```

| 策略 | 说明 |
|------|------|
| `ROUND_ROBIN` | 按 `depth % n_dims` 轮转 |
| `WIDEST_FIRST` | 选最宽维度 |
| `BEST_TIGHTEN` | 选使 $\max(\sum V_i^L, \sum V_i^R)$ 最小的维度（默认） |

#### 暖启动 & 并行

```cpp
LECT snapshot() const;   // 深拷贝快照
int warmup(int max_depth, int n_paths, int seed = 42);
int transplant_subtree(const LECT& worker, int snapshot_base,
                       const std::unordered_map<int, int>& id_map);
```

#### 扩展性能分析计数器

```cpp
struct ExpandProfile {
    double pick_dim_ms, fk_inc_ms, envelope_ms, refine_ms;
    int pick_dim_calls, pick_dim_cache_hits, expand_calls;
    void reset();
    void merge(const ExpandProfile& o);
};
ExpandProfile expand_profile_;  // 公开可读
```

使用 `[GRW] expand_profile:` 输出至 stderr，含 `new_nodes` 计数。

#### 持久化 (lect\_io)

```cpp
bool lect_save_binary(const LECT& lect, const std::string& path);
bool lect_save_incremental(const LECT& lect, const std::string& path,
                           int old_n_nodes);
bool lect_load_binary(LECT& lect, const Robot& robot, const std::string& path);
```

V4 二进制格式支持双通道和网格节。V3/V2/V1 向后兼容（加载入 CH\_SAFE）。
机器人指纹哈希验证缓存一致性。

> **注意**：`lect_load_binary()` 中的 `set_lect_state()` 会用缓存文件头中
> 记录的端点源覆盖当前 `ep_config_`。调用方应在加载后调用
> `lect_->set_ep_config(config.endpoint_source)` 恢复用户配置。

---

## ffb/ffb

> 头文件：`include/sbf/ffb/ffb.h`
> 实现：`src/ffb/ffb.cpp`

### `FFBResult`

| 字段 | 类型 | 说明 |
|------|------|------|
| `node_idx` | `int` | 目标 LECT 叶节点（-1 表示失败） |
| `path` | `vector<int>` | 根 → 叶节点路径 |
| `fail_code` | `int` | 0=成功, 1=已占用, 2=超深度, 3=低于最小边, 4=超时 |
| `n_new_nodes` | `int` | 本次搜索展开的新节点数 |
| `n_fk_calls` | `int` | FK 计算次数 |
| `n_cache_hits` | `int` | 缓存命中（每个访问的节点若已有数据） |
| `n_cache_misses` | `int` | 缓存未命中 |
| `total_ms` | `double` | 总 FFB 调用时间 |
| `envelope_ms` | `double` | `compute_envelope` 耗时 |
| `collide_ms` | `double` | 碰撞检查耗时 |
| `expand_ms` | `double` | `expand_leaf` 耗时 |
| `intervals_ms` | `double` | `node_intervals` 耗时 |
| `n_collide_calls` | `int` | `collides_scene` 调用次数 |
| `n_expand_calls` | `int` | `expand_leaf` 调用次数 |
| `n_steps` | `int` | 循环迭代次数 |

> **缓存命中统计语义（v5 修改）**：`n_cache_hits` / `n_cache_misses`
> 在循环头部对**每个访问的节点**计数，而非仅在包络计算步骤。
> 因此 `n_cache_hits + n_cache_misses == n_steps`。

### `FFBConfig`

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `min_edge` | `double` | `1e-4` | 最小区间宽度 |
| `max_depth` | `int` | `30` | 最大树深度 |
| `deadline_ms` | `double` | `0.0` | 超时 (0 = 无限) |

### 主要函数

```cpp
FFBResult find_free_box(
    LECT& lect,
    const Eigen::VectorXd& seed,
    const Obstacle* obs,
    int n_obs,
    const FFBConfig& config = FFBConfig());
```

FFB 沿 LECT 向种子配置下降：
1. 占用检查 → fail\_code=1
2. iAABB 碰撞粗筛
3. Hull/Grid 精炼
4. `subtree_occ` 早停 → 成功
5. 展开叶节点并向种子子节点下降

---

## forest/grower

> 头文件：`include/sbf/forest/grower.h`
> 实现：`src/forest/grower.cpp`

### `GrowerConfig` — 生长器配置

```cpp
struct GrowerConfig {
    enum Mode { RRT, WAVEFRONT };

    Mode mode = WAVEFRONT;
    FFBConfig ffb_config;
    int max_boxes = 500;
    double timeout_ms = 30000.0;
    int max_consecutive_miss = 50;    // 连续FFB失败停止阈值
    double rrt_goal_bias = 0.5;       // RRT目标偏置概率 (connect_mode推荐0.8)
    double rrt_step_ratio = 0.1;      // RRT步长比 (connect_mode推荐0.05)
    double boundary_epsilon = 1e-6;   // snap-to-face距离
    int n_boundary_samples = 4;       // 面采样数
    double goal_face_bias = 0.5;      // 面选择目标偏向
    uint64_t rng_seed = 42;
    int n_threads = hardware_concurrency();
    int bridge_n_threads = 0;         // bridge_all_islands线程数 (0=用n_threads)

    /// 协调多树生长模式：生长与连通性检测一体化。
    /// 当所有 multi-goal 树通过 box 邻接连通时不停止（继续覆盖），
    /// 并在停滞时自动触发中点桥接采样策略。
    /// 若为 false 则使用传统独立并行生长 + bridge_all_islands。
    bool connect_mode = false;

    struct WavefrontStage {
        int box_limit;      // 此阶段累计盒上限
    };
    std::vector<WavefrontStage> wavefront_stages =
        {{50}, {150}, {300}, {500}};
    bool enable_promotion = true;     // 启用 promotion 合并
};
```

**`connect_mode` 协调生长关键参数（推荐值）**：

| 参数 | 推荐值 | 说明 |
|------|--------|------|
| `connect_mode` | `true` | 启用协调多树生长 |
| `rrt_goal_bias` | 0.8 | 高目标偏置加速树间连通 |
| `rrt_step_ratio` | 0.05 | 较小步长精确探索间隙（停滞时自动减半） |
| `max_consecutive_miss` | 2000 | 允许更多尝试 |
| `n_threads` | 5 | 每棵树一个worker |
| `timeout_ms` | 10000 | 10s足以实现连通+覆盖 |

**协调生长内部参数**（硬编码于 `grow_coordinated`）：

| 参数 | 值 | 说明 |
|------|-----|------|
| `lect_refresh_interval` | 10 | 每10批次刷新worker LECT快照 |
| `stall_threshold` | 500 | 500个box无分量变化视为停滞 |
| `K_SUBSAMPLE` | 64 | 最近box搜索子采样数 |
| 桥接概率 | 0.7 | 停滞时70%的种子使用中点桥接 |
| 桥接α范围 | [0.3, 1.0] | 插值点偏向目标端 |
| 桥接扰动σ | 0.02 | 高斯扰动标准差 |

### `GrowerResult` — 生长统计

| 字段 | 类型 | 说明 |
|------|------|------|
| `boxes` | `vector<BoxNode>` | 生成的所有盒 |
| `n_roots` | `int` | 根节点数 |
| `n_ffb_success` / `n_ffb_fail` | `int` | FFB 成功/失败计数 |
| `n_promotions` | `int` | Promotion 合并次数 |
| `total_volume` | `double` | 总包络体积 |
| `build_time_ms` | `double` | 构建耗时 |
| `ffb_cache_hits` / `ffb_cache_misses` | `int` | 缓存命中/未命中 |
| `lect_nodes_final` | `int` | LECT 最终节点数 |
| `all_connected` | `bool` | 所有 multi-goal 树是否全连通（`connect_mode`） |
| `connect_time_ms` | `double` | 首次实现全连通的时刻（ms），未连通则为总耗时 |
| `connect_n_boxes` | `int` | 首次实现全连通时的box数 |

### `ForestGrower` — 森林生长器

```cpp
class ForestGrower {
public:
    // 串行模式
    ForestGrower(const Robot& robot, LECT& lect, const GrowerConfig& config);
    // 并行模式（owns LECT）
    ForestGrower(const Robot& robot, LECT&& lect_owned, const GrowerConfig& config);

    void set_endpoints(const VectorXd& start, const VectorXd& goal);
    void set_multi_goals(const std::vector<VectorXd>& goals);
    void set_deadline(Clock::time_point deadline);

    GrowerResult grow(const Obstacle* obs, int n_obs);

    // 并行子树生长（worker内部调用）
    GrowerResult grow_subtree(const VectorXd& root_seed, int root_id,
                              const Obstacle* obs, int n_obs,
                              std::shared_ptr<std::atomic<int>> shared_counter);

    const std::vector<BoxNode>& boxes() const;
    const LECT& lect() const;
    LECT&& take_lect();  // 转移LECT所有权
};
```

**生长流程**：

**模式 A — 协调多树生长（`connect_mode = true`，推荐）**：

调用 `grow_coordinated()`，所有树共享一个 master 线程 + worker 线程池：

1. **Root 创建**：每个 seed 点一个根box（FFB）
2. **协调循环**：master 生成任务批次 → worker 并行 FFB → master 接受/拒绝
3. **连接驱动调度**：优先给最小连通分量的树分配资源
4. **中点桥接**：停滞时沿断开树间连线插值采样（70%概率）
5. **实时连通跟踪**：新box接受后检查跨树邻接，增量 UnionFind
6. **Promotion**：合并子叶节点为更大安全盒

典型结果（7-DOF iiwa14，5树）：3.8s连通，10s得7.6K boxes，25% vcov。

**模式 B — 传统独立并行生长（`connect_mode = false`）**：

1. **Root 选择**：Farthest Point Sampling（30 候选选最远），每树独立预算
2. **RRT 生长**：goal-biased 采样 → 最近盒 → snap-to-face → FFB → 强制邻接
3. **并行执行**：`ThreadPool`，每棵树独立 LECT `snapshot()`
4. **ID 重映射**：完成后 `transplant_subtree` 合并回 master LECT
5. **Promotion**：合并子叶节点为更大安全盒
6. **后续需调用 `bridge_all_islands()` 实现连通**

---

## forest/adjacency

> 头文件：`include/sbf/forest/adjacency.h`

### 类型别名

```cpp
using AdjacencyGraph = std::unordered_map<int, std::vector<int>>;
```

### `SharedFace` — 共面描述

```cpp
struct SharedFace {
    int dim;                        // 共享面所在维度
    double value;                   // 面的坐标值
    std::vector<Interval> face_ivs; // 其余维度的重叠区间
};
```

### 自由函数

```cpp
// 计算所有盒的邻接图
AdjacencyGraph compute_adjacency(const std::vector<BoxNode>& boxes,
                                  double tol = 1e-10);

// 获取两盒共面
std::optional<SharedFace> shared_face(const BoxNode& a, const BoxNode& b,
                                       double tol = 1e-10);

// Tarjan 算法查找桥接点（Articulation Points）
std::unordered_set<int> find_articulation_points(const AdjacencyGraph& adj);
```

**邻接条件**：恰好 1 个维度为"接触"（$|w| \leq \text{tol}$），其余维度为"重叠"（$w > \text{tol}$）。

---

## forest/coarsen

> 头文件：`include/sbf/forest/coarsen.h`
> 实现：`src/forest/coarsen.cpp`

### `GreedyCoarsenConfig`

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `target_boxes` | `int` | 0 | 目标盒数 (0=收敛即停) |
| `max_rounds` | `int` | 100 | 最大轮数 |
| `adjacency_tol` | `double` | 1e-10 | 邻接容差 |
| `score_threshold` | `double` | 50.0 | 合并评分阈值 |
| `max_lect_fk_per_round` | `int` | 2000 | 每轮最大FK调用 |

### 三级粗化函数

```cpp
// 1. 维度扫描合并（精确匹配面）
CoarsenResult coarsen_forest(std::vector<BoxNode>& boxes,
                              const CollisionChecker& checker,
                              int max_rounds = 20);

// 2. 贪心邻接合并（hull评分排序 + 碰撞验证）
GreedyCoarsenResult coarsen_greedy(std::vector<BoxNode>& boxes,
                                     const CollisionChecker& checker,
                                     const GreedyCoarsenConfig& config,
                                     LECT* lect = nullptr,
                                     const std::unordered_set<int>* protected_ids = nullptr);

// 3. 重叠过滤（删除被包含的冗余盒）
int filter_coarsen_overlaps(std::vector<BoxNode>& boxes,
                             double min_gmean_edge = 1e-4,
                             const std::unordered_set<int>* protected_ids = nullptr);
```

**粗化流水线**：`coarsen_forest` → `coarsen_greedy` → `filter_coarsen_overlaps`。
桥接点（`find_articulation_points`）自动受保护，不会被合并或删除。

---

## forest/connectivity

> 头文件：`include/sbf/forest/connectivity.h`
> 实现：`src/forest/connectivity.cpp`

### `UnionFind` — 加权并查集

```cpp
class UnionFind {
public:
    explicit UnionFind(int n = 0);
    int find(int x);                  // 路径压缩
    void unite(int x, int y);         // 按秩合并
    bool connected(int x, int y);     // O(α(n)) 连通查询
    int size() const;
    void resize(int new_size);        // 增量扩展
};
```

### `RRTConnectConfig`

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_iters` | `int` | 8000 | 最大迭代 |
| `step_size` | `double` | 0.2 | 步长 (rad) |
| `goal_bias` | `double` | 0.15 | 目标偏置 |
| `timeout_ms` | `double` | 500.0 | 超时 (ms) |
| `segment_resolution` | `int` | 10 | 段碰撞分辨率 |

### 核心函数

```cpp
// 检测连通分量
std::vector<std::vector<int>> find_islands(const AdjacencyGraph& adj);

// RRT-Connect：双树交替扩展 + 贪心连接
std::vector<VectorXd> rrt_connect(
    const VectorXd& q_a, const VectorXd& q_b,
    const CollisionChecker& checker, const Robot& robot,
    const RRTConnectConfig& cfg = {}, int seed = 42);

// 沿RRT路径铺设box走廊
int chain_pave_along_path(
    const std::vector<VectorXd>& rrt_path,
    int anchor_box_id,
    std::vector<BoxNode>& boxes, LECT& lect,
    const Obstacle* obs, int n_obs,
    const FFBConfig& ffb_config, AdjacencyGraph& adj,
    int& next_box_id, const Robot& robot,
    int max_chain = 200, int max_steps_per_wp = 15);

// S-T桥接：best-first候选对 RRT
int bridge_s_t(int start_box_id, int goal_box_id,
               std::vector<BoxNode>& boxes, LECT& lect, /*...*/);

// 全岛并行桥接（P2优化）
// ThreadPool并行RRT + 增量UnionFind + cancel flag
int bridge_all_islands(
    std::vector<BoxNode>& boxes, LECT& lect,
    const Obstacle* obs, int n_obs,
    AdjacencyGraph& adj, const FFBConfig& ffb_config,
    int& next_box_id, const Robot& robot,
    const CollisionChecker& checker,
    double per_pair_timeout_ms = 300.0,
    int max_pairs_per_gap = 15,
    int max_total_bridges = 200,
    Clock::time_point deadline = Clock::time_point::max());
```

**`bridge_all_islands` 关键设计**：

- **Phase 1（并行）**：ThreadPool (N_hw - 1 线程) 提交所有候选pair的RRT任务
- **Phase 2（串行）**：对第一个成功路径执行 `chain_pave_along_path`
- **Cancel flag**：`std::shared_ptr<std::atomic<bool>>`，岛合并时设置，剩余RRT立即返回
- **增量 UnionFind**：O(α(n)) 合并检查，避免每轮重建连通分量

---

## forest/thread\_pool

> 头文件：`include/sbf/forest/thread_pool.h`

### `ThreadPool` — C++17 线程池

```cpp
class ThreadPool {
public:
    explicit ThreadPool(int n_threads);
    ~ThreadPool();  // join所有线程

    // 提交任务，返回 future
    template<typename F, typename... Args>
    auto submit(F&& f, Args&&... args) -> std::future<result_of_t<F(Args...)>>;

    int size() const;
    // 不可拷贝/移动
};
```

---

## planner/dijkstra

> 头文件：`include/sbf/planner/dijkstra.h`
> 实现：`src/planner/dijkstra.cpp`

### `DijkstraResult`

| 字段 | 类型 | 说明 |
|------|------|------|
| `found` | `bool` | 是否找到路径 |
| `box_sequence` | `vector<int>` | 盒ID序列 |
| `total_cost` | `double` | 总代价 |

```cpp
// A*搜索（带欧氏距离启发式）
DijkstraResult dijkstra_search(
    const AdjacencyGraph& adj,
    const std::vector<BoxNode>& boxes,
    int start_box_id, int goal_box_id,
    const Eigen::VectorXd& goal_point = Eigen::VectorXd());
```

启发式：$h(n) = \|c_n - q_g\|_2$，边权 = 当前点到共面中心的欧氏距离。

---

## planner/path\_extract

> 头文件：`include/sbf/planner/path_extract.h`
> 实现：`src/planner/path_extract.cpp`

```cpp
// 从盒序列提取waypoints（共面中心插入）
std::vector<Eigen::VectorXd> extract_waypoints(
    const std::vector<int>& box_sequence,
    const std::vector<BoxNode>& boxes,
    const Eigen::VectorXd& start,
    const Eigen::VectorXd& goal);
```

---

## planner/path\_smoother

> 头文件：`include/sbf/planner/path_smoother.h`
> 实现：`src/planner/path_smoother.cpp`

### `SmootherConfig`

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `shortcut_max_iters` | `int` | 100 | Shortcut 最大迭代 |
| `smooth_window` | `int` | 3 | 滑动窗口大小 |
| `smooth_iters` | `int` | 5 | 平滑迭代次数 |
| `segment_resolution` | `int` | 10 | 段碰撞分辨率 |

### 自由函数

```cpp
// 随机 shortcut
std::vector<VectorXd> shortcut(const std::vector<VectorXd>& path,
                                const CollisionChecker& checker,
                                const SmootherConfig& config,
                                uint64_t seed = 42);

// 滑动窗口均值平滑（投影回box）
std::vector<VectorXd> smooth_moving_average(
    const std::vector<VectorXd>& path,
    const std::vector<BoxNode>& box_sequence,
    const SmootherConfig& config);

// 路径长度计算
double path_length(const std::vector<VectorXd>& path);
```

### 查询内路径优化流水线（sbf_planner.cpp 中实现）

| 步骤 | 操作 | 参数 | 安全保证 |
|------|------|------|---------|
| 1 | 贪心前向简化 | 跳到最远无碰撞可达点 | 逐段碰撞验证 |
| 2 | 随机 Shortcut | max(300, 50×n_wp) 轮 | 直连验证 |
| 2.5 | Densify | > 0.3 rad 段插入中间点 | 跳过碰撞点 |
| 3 | **Elastic Band** | 60 iter, α∈{0.5, 0.3, 0.15, 0.05} | per-segment 回退 |
| 4 | 最终 Shortcut | max(300, 30×n_wp) 轮 | 碰撞则回退 |
| 5 | **Pass2** | mini EB (30 iter, α∈{0.4, 0.2, 0.08}) | 整体回退 |

**自适应碰撞分辨率**：`ares(a,b) = max(20, ceil(||b-a|| / 0.005))`

**三层安全网**：

1. Per-segment 回退：3遍扫描，仅回退碰撞 waypoint
2. Full revert：EB 后仍碰撞 → 恢复到 EB 前
3. Emergency RRT：所有优化后碰撞 → 5s RRT + 完整子流水线

---

## planner/gcs\_planner

> 头文件：`include/sbf/planner/gcs_planner.h`
> 条件编译：`SBF_HAS_DRAKE`

### `GCSConfig`

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `bezier_degree` | `int` | 3 | Bézier 曲线阶数 |
| `time_limit_sec` | `double` | 30.0 | 求解时间限制 |
| `corridor_hops` | `int` | 2 | 走廊扩展跳数 |
| `convex_relaxation` | `bool` | `true` | 凸松弛 |
| `cost_weight_length` | `double` | 1.0 | 路径长度权重 |

```cpp
// GCS 规划（需 Drake）
GCSResult gcs_plan(const AdjacencyGraph& adj, const std::vector<BoxNode>& boxes,
                    const VectorXd& start, const VectorXd& goal,
                    const GCSConfig& config = {});

// Corridor 扩展
std::unordered_set<int> expand_corridor(const AdjacencyGraph& adj,
                                          const std::vector<int>& path_boxes,
                                          int hops);
```

---

## planner/sbf\_planner

> 头文件：`include/sbf/planner/sbf_planner.h`
> 实现：`src/planner/sbf_planner.cpp`

### `PlanResult` — 规划结果

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `success` | `bool` | `false` | 是否找到路径 |
| `path` | `vector<VectorXd>` | — | 平滑后的 C-space 路点 |
| `box_sequence` | `vector<int>` | — | 盒 ID 序列 |
| `path_length` | `double` | `0.0` | 欧几里得路径总长 |
| `planning_time_ms` | `double` | `0.0` | 总壁钟时间 |
| `n_boxes` | `int` | `0` | 粗化后盒数 |
| `build_time_ms` | `double` | `0.0` | 森林构建时间 |
| `lect_time_ms` | `double` | `0.0` | LECT 构建/加载时间 |

### `ProxySearchConfig` — 代理 RRT 搜索配置

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `max_candidates` | `int` | 25 | 最大候选数 |
| `tier1_count` / `tier2_count` | `int` | 5 / 10 | 各层候选数 |
| `tier1_timeout_ms` | `double` | 200.0 | Tier 1 超时 |
| `tier2_timeout_ms` | `double` | 500.0 | Tier 2 超时 |
| `tier3_timeout_ms` | `double` | 2000.0 | Tier 3 超时 |
| `total_budget_ms` | `double` | 8000.0 | 总预算 |

### `SBFPlannerConfig` — 顶层配置

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `grower` | `GrowerConfig` | — | 森林生长配置 |
| `coarsen` | `GreedyCoarsenConfig` | — | 贪心粗化配置 |
| `smoother` | `SmootherConfig` | — | 路径平滑配置 |
| `proxy` | `ProxySearchConfig` | — | Proxy搜索配置 |
| `use_gcs` | `bool` | `false` | 是否启用 GCS |
| `gcs` | `GCSConfig` | — | GCS 配置 |
| `endpoint_source` | `EndpointSourceConfig` | IFK | 端点源 |
| `envelope_type` | `EnvelopeTypeConfig` | — | 包络类型 |
| `split_order` | `SplitOrder` | `BEST_TIGHTEN` | 切分策略 |
| `z4_enabled` | `bool` | `true` | Z4 旋转对称 |
| `lect_no_cache` | `bool` | `false` | 禁用持久化 |
| `lect_cache_dir` | `string` | `~/.sbf_cache` | 缓存目录 |

### `SBFPlanner` — 顶层规划器

```cpp
class SBFPlanner : public IPlanner {
public:
    SBFPlanner(const Robot& robot, const SBFPlannerConfig& config = {});

    // 一体化接口：build + query
    PlanResult plan(const VectorXd& start, const VectorXd& goal,
                    const Obstacle* obs, int n_obs,
                    double timeout_ms = 30000.0) override;

    // 分离接口
    void build(const VectorXd& start, const VectorXd& goal,
               const Obstacle* obs, int n_obs,
               double timeout_ms = 30000.0);

    void build_coverage(const Obstacle* obs, int n_obs,
                        double timeout_ms = 30000.0,
                        const std::vector<VectorXd>& seed_points = {});

    PlanResult query(const VectorXd& start, const VectorXd& goal,
                     const Obstacle* obs = nullptr, int n_obs = 0);

    // 状态查询
    const std::vector<BoxNode>& boxes() const;
    const AdjacencyGraph& adjacency() const;
    int n_boxes() const;
    void clear_forest();

    // 预热
    int warmup_lect(int max_depth, int n_paths, int seed = 42);
};
```

**`build_coverage` 流程**：

**模式 A — 协调多树生长（`connect_mode = true`）**：
1. LECT 加载/构建
2. `grow_coordinated`（协调多树生长 + 内联中点桥接 + 实时连通跟踪）
3. `promote_all`（子叶合并）
4. `coarsen`（可选：维度扫描 + 邻接贪心 + 重叠过滤）
5. `compute_adjacency`（最终邻接图）

**模式 B — 传统两阶段（`connect_mode = false`）**：
1. LECT 加载/构建
2. `grow_parallel`（N棵种子树并行生长 + cross-seed）
3. `promote_all`（子叶合并）
4. `coarsen_forest`（维度扫描合并）
5. `coarsen_greedy`（邻接贪心合并 + 桥接点保护）
6. `filter_coarsen_overlaps`（重叠过滤）
7. `bridge_all_islands`（并行RRT桥接）
8. `compute_adjacency`（最终邻接图）
9. `lect_save_incremental`（缓存保存）

**`query` 流程**：
1. Box 定位 → 连通性检查 → `bridge_s_t`
2. 孤立检测 → Proxy Search（分层超时RRT）
3. `dijkstra_search`（A* 启发式）→ `extract_waypoints`
4. Link path 拼接
5. **P0**：预竞赛简化 + 多试次 RRT 竞争（最多3次，自适应超时）
6. 段验证 + RRT 修复
7. 5步路径优化流水线
8. Emergency 安全网

---

## Python 绑定 (sbf5)

> 绑定文件：`python/sbf5_bindings.cpp`

### `EndpointSource` 枚举

```python
from sbf5 import EndpointSource
EndpointSource.IFK         # 区间 FK
EndpointSource.CritSample  # 边界采样
EndpointSource.Analytical  # 解析求解
EndpointSource.GCPC        # 全局缓存
```

### `EndpointSourceConfig`

```python
from sbf5 import EndpointSourceConfig

cfg = EndpointSourceConfig()
cfg.source = EndpointSource.IFK
cfg.n_samples_crit = 1000
cfg.max_phase_analytical = 3
cfg.set_gcpc_cache(cache)  # 设置 GCPC 缓存
```

### `SBFPlannerConfig`

```python
from sbf5 import SBFPlannerConfig, EndpointSource

config = SBFPlannerConfig()
config.endpoint_source.source = EndpointSource.IFK
config.split_order = SplitOrder.BEST_TIGHTEN
config.lect_no_cache = False
config.lect_cache_dir = "path/to/cache"
```

### `SBFPlanner`

```python
from sbf5 import SBFPlanner, Robot

planner = SBFPlanner(robot, config)
result = planner.plan(start, goal, obstacles)
```

### `PlanResult`

| 属性 | 类型 | 说明 |
|------|------|------|
| `success` | `bool` | 规划是否成功 |
| `path` | `ndarray` | 路径点序列 (N×D) |
| `box_sequence` | `list[int]` | 经过的盒子 ID |
| `path_length` | `float` | 路径长度 (rad) |
| `planning_time_ms` | `float` | 总规划时间 |
| `n_boxes` | `int` | 盒子数量 |
| `build_time_ms` | `float` | 森林构建时间 |
| `lect_time_ms` | `float` | LECT 加载/保存时间 |

### 典型使用示例

```python
import numpy as np
from sbf5 import (Robot, SBFPlanner, SBFPlannerConfig,
                   EndpointSource, SplitOrder, Obstacle)

# 配置
config = SBFPlannerConfig()
config.endpoint_source.source = EndpointSource.IFK
config.lect_cache_dir = "./lect_cache"

# 构建
robot = Robot.from_json("iiwa14.json")
planner = SBFPlanner(robot, config)

# 离线构建
obs = [Obstacle(0.3, -0.3, 0.0, 0.5, 0.3, 0.8)]
planner.build_coverage(obs, len(obs), timeout_ms=60000.0)

# 在线查询
result = planner.query(
    start=np.array([0, 0.5, 0, -1.5, 0, 0.5, 1.57]),
    goal=np.array([1.57, 0.3, 0, -0.8, 0, 1.0, 0]))

print(f"Success={result.success}, len={result.path_length:.3f}, "
      f"boxes={result.n_boxes}, time={result.planning_time_ms:.0f}ms")
```
