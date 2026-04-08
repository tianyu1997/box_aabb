# SafeBoxForest v5 API 参考（中文）

---

## 目录

- [envelope/endpoint\_source — 端点源统一调度](#envelopeendpoint_source)
- [lect/lect — 连杆包络碰撞树 (LECT)](#lectlect)
- [ffb/ffb — Find Free Box (FFB)](#ffbffb)
- [planner/sbf\_planner — 规划器配置](#plannersbf_planner)
- [Python 绑定 (sbf5)](#python-绑定-sbf5)

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

## planner/sbf\_planner

> 头文件：`include/sbf/planner/sbf_planner.h`

### `SBFPlannerConfig`

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `grower` | `GrowerConfig` | — | 森林生长配置 |
| `coarsen` | `GreedyCoarsenConfig` | — | 贪心粗化配置 |
| `smoother` | `SmootherConfig` | — | 路径平滑配置 |
| `use_gcs` | `bool` | `false` | 是否启用 GCS |
| `gcs` | `GCSConfig` | — | GCS 配置 |
| `endpoint_source` | `EndpointSourceConfig` | IFK | 端点源配置 |
| `envelope_type` | `EnvelopeTypeConfig` | — | 包络类型配置 |
| `split_order` | `SplitOrder` | `BEST_TIGHTEN` | 切分策略 |
| `z4_enabled` | `bool` | `true` | Z4 旋转对称缓存 |
| `lect_no_cache` | `bool` | `false` | 禁用 LECT 持久化 |
| `lect_cache_dir` | `string` | `default_lect_cache_dir()` | 缓存目录 |

> **端点源运行时覆盖**：`endpoint_source.source` 指定运行时使用的端点源。
> 即使缓存文件以不同源生成（如 CritSample），规划器加载后会自动调用
> `lect_->set_ep_config(config.endpoint_source)` 恢复运行时源。新扩展的节点
> 使用指定源，已缓存节点保留原有通道数据。

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
config.endpoint_source.source = EndpointSource.IFK   # 运行时端点源
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
| `path` | `ndarray` | 路径点序列 |
| `box_sequence` | `list[int]` | 经过的盒子 ID |
| `path_length` | `float` | 路径长度 |
| `planning_time_ms` | `float` | 总规划时间 |
| `n_boxes` | `int` | 盒子数量 |
| `n_coarsen_merges` | `int` | 粗化合并次数 |
| `envelope_volume_total` | `float` | 总包络体积 |
| `build_time_ms` | `float` | 森林构建时间 |
| `lect_time_ms` | `float` | LECT 加载/保存时间 |

### 典型使用示例

```python
import numpy as np
from sbf5 import (Robot, SBFPlanner, SBFPlannerConfig,
                   EndpointSource, SplitOrder, Obstacle)

# 配置
config = SBFPlannerConfig()
config.endpoint_source.source = EndpointSource.IFK  # 快速增量路径
config.lect_cache_dir = "./lect_cache"

# 构建
robot = Robot.from_urdf("iiwa14.urdf")
planner = SBFPlanner(robot, config)

# 规划
obs = [Obstacle(center=[0.5, 0, 0.5], half_extents=[0.1, 0.1, 0.1])]
result = planner.plan(
    start=np.array([0, 0, 0, -1.57, 0, 1.57, 0]),
    goal=np.array([1.57, 0.5, 0, -0.8, 0, 1.0, 0]),
    obstacles=obs
)
print(f"Success={result.success}, boxes={result.n_boxes}, "
      f"time={result.planning_time_ms:.0f}ms")
```
