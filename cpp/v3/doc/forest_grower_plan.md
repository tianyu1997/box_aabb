# ForestGrower 实现计划

> SafeBoxForest v3 — 多根森林生长引擎
> 创建日期: 2026-03-16

---

## 1. 设计目标

实现一个新的 `ForestGrower` 类，从多个 root 出发，以两种扩展策略（前波扩展 / 类 RRT 扩展）在 C-space 中生成碰撞安全的 box 森林。

**核心特性：**
- 多 root 并行逻辑（按 LECT KD-tree 子树划分，单线程优先，接口预留多线程）
- 两种扩展模式：Wavefront（BFS 边界扩展）和 RRT-like（随机采样+最近 box 扩展）
- 每个 box 记录拓展来源（parent_box_id + expand_face），方便 wavefront 避免回头探索
- 增量邻接图（AdjacencyGraph）管理 box 连通关系

---

## 2. 关键设计决策

| 决策项 | 选择 | 理由 |
|---|---|---|
| 代码组织 | 独立 `ForestGrower` 类 | 与现有 `SafeBoxForest::build()` 分离，职责清晰 |
| LECT 共享方式 | 按 KD-tree 子树划分 | 每个 root 对应一个 LECT 子树，子树内部操作无需加锁 |
| Box 元数据 | 记录 parent_box_id + expand_face | 成本低（3 个 int），wavefront 可避免回头采样 |
| 线程模型 | 单线程 + 线程池并行（`n_threads`） | 单线程保证正确性，多线程每个 worker 独立 LECT/RNG，零共享可变状态 |
| 邻接维护 | Sweep-and-prune 加速 | 批量 O(N·K) sweep 代替 O(N²)；增量 O(log N + K) 二分查找 + 局部扫描 |

---

## 3. 文件清单

### 3.1 新增文件

| 文件路径 | 内容 | 状态 |
|---|---|---|
| `include/sbf/forest/grower_config.h` | `GrowerConfig` 配置结构体（含 `n_threads`） | ✅ 已实现 |
| `include/sbf/forest/forest_grower.h` | `ForestGrower` 类声明 + `GrowerResult` | ✅ 已实现 |
| `include/sbf/forest/adjacency_graph.h` | `AdjacencyGraph` 邻接图类声明 | ✅ 已实现 |
| `include/sbf/forest/thread_pool.h` | `ThreadPool` 轻量级 C++17 线程池（header-only） | ✅ 已实现 |
| `src/forest/forest_grower.cpp` | ForestGrower 全部实现（含串行 + 并行路径） | ✅ 已实现 |
| `src/forest/adjacency_graph.cpp` | AdjacencyGraph 实现 | ✅ 已实现 |
| `tests/test_forest_grower.cpp` | 单元测试（5 个测试） | ✅ 已实现 |
| `tests/test_parallel_grower.cpp` | 并行测试 + benchmark（5 个测试） | ✅ 已实现 |
| `tests/test_phase5_optimizations.cpp` | Phase 5 优化测试（4 个测试） | ✅ 已实现 |
| `doc/phase5_optimization_plan.md` | Phase 5 优化计划文档 | ✅ 已完成 |

### 3.2 修改文件

| 文件路径 | 修改内容 |
|---|---|
| `include/sbf/common/types.h` | `BoxNode` 新增 `parent_box_id`, `expand_face_dim`, `expand_face_side` |
| `CMakeLists.txt` | 测试目标注册 + `Threads::Threads` 链接 | ✅ 已实现 |
| `docs/API_REFERENCE.md` | 新增 ForestGrower / AdjacencyGraph / ThreadPool / GrowerConfig 文档 | ✅ 已实现 |

---

## 4. 数据结构设计

### 4.1 BoxNode 扩展

```cpp
// 在 types.h 的 BoxNode 中新增：
int parent_box_id   = -1;   // 拓展来源 box (-1 表示 root)
int expand_face_dim = -1;   // 拓展经过的面所在维度
int expand_face_side = -1;  // 0 = lo 面, 1 = hi 面
```

### 4.2 GrowerConfig

```cpp
struct GrowerConfig {
    // ── Root 选取 ──────────────────────────────────────────
    int n_roots = 2;                   // root 数量

    // ── 扩展模式 ──────────────────────────────────────────
    enum class Mode { Wavefront, RRT };
    Mode mode = Mode::Wavefront;

    // ── 通用参数 ──────────────────────────────────────────
    int    max_boxes             = 500;    // 最大 box 数量
    double min_edge              = 0.01;   // FFB 最小边长
    int    max_depth             = 30;     // FFB 最大 KD-tree 深度
    int    max_consecutive_miss  = 200;    // 连续失败上限
    double timeout               = 60.0;   // 超时 (秒)
    double adjacency_tol         = 1e-10;  // 邻接容差
    uint64_t rng_seed            = 42;

    // ── Wavefront 特有 ────────────────────────────────────
    int    n_boundary_samples    = 6;      // 每个 box 采边界种子数
    double boundary_epsilon      = 0.01;   // 面外偏移距离
    double goal_face_bias        = 0.6;    // 目标方向面选择概率

    // ── RRT 特有 ──────────────────────────────────────────
    double rrt_step_ratio        = 0.3;    // 步长 = ratio × 关节范围宽度
    double rrt_goal_bias         = 0.1;    // 采样目标点的概率
    // ── 并行化 ─────────────────────────────────────────
    int    n_threads             = 1;        // 工作线程数 (1=串行)
    // ── Pipeline (转发给 LECT) ────────────────────────────
    envelope::PipelineConfig pipeline =
        envelope::PipelineConfig::recommended();
};
```

### 4.3 AdjacencyGraph

**加速策略**（移植自 v2 `deoverlap.cpp` 的 sweep-and-prune）：

| 技术 | 描述 | 效果 |
|---|---|---|
| **维度排序** | 按 fill_ratio = avg_width / range 排序维度。最窄维度 → sweep 维度，次窄 → filter 维度 | 选择最佳剪枝维度 |
| **Sweep-and-prune** | 按 sweep 维度的 lo 值排序所有 box，顺序扫描时 lo_j > hi_i + tol 立即 break | 剪枝 >95% 无关对 |
| **双维度预过滤** | sweep 维度通过后，先检查 filter 维度再做全维度检查 | 进一步剪枝 >80% |
| **行优先平坦布局** | 将 intervals 打包为 `double lo[N*D]` 行优先数组，每个 box 占 ~1 cache line | CPU 缓存友好 |
| **窄→宽维度检查** | 全维度检查时按从窄到宽排序，最可能拒绝的维度先检查 | 早期退出 |
| **增量二分扫描** | 新增 box 时，在已排序的 sweep 数组中二分查找插入位置，只扫描局部邻域 | O(log N + K) vs O(N) |

**v2 实测数据**：N=5000 时 sweep 剪枝 99.6%，双维度剪枝 99.9%，总邻接对从 12,497,500 降至 891。

```cpp
class AdjacencyGraph {
public:
    // ── 增量操作 (sweep-accelerated) ──────────────────────
    // O(log N + K): 二分查找 sweep 维度排序位置 + 局部扫描
    void add_box(const BoxNode& box, double tol = 1e-10);
    void remove_box(int box_id);

    // ── 批量重建 (sweep-and-prune) ────────────────────────
    // O(N·K) where K = avg sweep overlap count, K ≪ N
    void rebuild(const std::vector<BoxNode>& boxes, double tol = 1e-10);

    // ── 查询 ──────────────────────────────────────────────
    const std::vector<int>& neighbors(int box_id) const;
    bool connected(int a, int b) const;          // BFS 可达性
    int  n_components() const;                   // 连通分量数
    std::vector<std::vector<int>> components() const;

    // ── RRT 用：最近 box 查询 ─────────────────────────────
    // O(log N) via KD-tree (subtree-bbox pruning)
    int find_nearest_box(const Eigen::VectorXd& q) const;

    // ── 统计 ──────────────────────────────────────────────
    int n_boxes() const;
    int n_edges() const;

private:
    // ── Box 存储 ──────────────────────────────────────────
    std::unordered_map<int, BoxNode>          box_map_;
    std::unordered_map<int, std::vector<int>> adj_;

    // ── Sweep-and-prune 加速结构 ──────────────────────────
    int n_dims_ = 0;
    int sweep_dim_   = 0;   // 最佳 sweep 维度 (fill ratio 最小)
    int filter_dim_  = 0;   // 第二 sweep 维度
    std::vector<int> dim_order_;  // 剩余维度按窄→宽排列

    // 行优先平坦数组 (cache-friendly)
    std::vector<double> flat_lo_;   // [N × D] row-major
    std::vector<double> flat_hi_;   // [N × D] row-major
    std::vector<int>    flat_ids_;  // [N] box IDs

    // 已按 sweep_dim 排序的索引
    std::vector<int>    sweep_order_;  // 排列：flat index sorted by lo[sweep_dim]

    // ── 内部方法 ──────────────────────────────────────────
    void rank_dimensions();           // 计算 fill ratio 选择 sweep/filter dim
    void rebuild_flat_arrays();       // 将 box_map_ 打包到 flat 数组
    void sort_sweep_order();          // 按 sweep_dim 排序

    // 单对邻接检查 (flat 数组版本，维度按剪枝顺序排列)
    bool check_adjacent_flat(int flat_i, int flat_j, double tol) const;

    // Sweep-and-prune 核心：返回所有邻接对
    std::vector<std::pair<int,int>>
    sweep_and_prune(double tol) const;
};
```

### 4.4 ForestGrower

```cpp
class ForestGrower {
public:
    ForestGrower(const Robot& robot, const GrowerConfig& config);

    // ── 端点设置 (可选) ───────────────────────────────────
    void set_endpoints(const Eigen::VectorXd& start,
                       const Eigen::VectorXd& goal);

    // ── 主入口 ────────────────────────────────────────────
    GrowerResult grow(const Obstacle* obstacles, int n_obs);

    // ── 子树独立扩展（并行 worker 使用） ───────────────────
    GrowerResult grow_subtree(const Eigen::VectorXd& root_seed,
                              int root_id,
                              const std::vector<Interval>& subtree_limits,
                              const Obstacle* obs, int n_obs);

    // ── 访问器 ──────────────────────────────────────────
    const LECT&           lect()  const;
    LECT&                 lect_mut();     // 可变访问
    const AdjacencyGraph& graph() const;
    const std::vector<BoxNode>& boxes() const;
    const GrowerConfig&   config() const;
    int n_boxes() const;

private:
    // ── Root 选取 ─────────────────────────────────────────
    void select_roots(const Obstacle* obstacles, int n_obs);
    void select_roots_no_endpoints(const Obstacle* obs, int n_obs);
    void select_roots_with_endpoints(const Obstacle* obs, int n_obs);

    // ── 子树划分 ──────────────────────────────────────────
    void partition_subtrees();

    // ── 两种扩展模式 ─────────────────────────────────────
    void grow_wavefront(const Obstacle* obs, int n_obs);
    void grow_rrt(const Obstacle* obs, int n_obs);
    // ── 并行调度 ───────────────────────────────────────
    void grow_parallel(const Obstacle* obs, int n_obs,
                       GrowerResult& result);
    // ── 边界采样 ──────────────────────────────────────────
    struct BoundarySeed {
        int dim;
        int side;                      // 0=lo, 1=hi
        Eigen::VectorXd config;
    };
    std::vector<BoundarySeed> sample_boundary(
        const BoxNode& box, const Eigen::VectorXd* goal_pt);

    // ── FFB 封装 ──────────────────────────────────────────
    int try_create_box(const Eigen::VectorXd& seed,
                       const Obstacle* obs, int n_obs,
                       int parent_box_id = -1,
                       int face_dim = -1, int face_side = -1);

    // ── Promotion (复用) ─────────────────────────────────
    int promote_all(const Obstacle* obs, int n_obs);
};
```

### 4.5 GrowerResult

```cpp
struct GrowerResult {
    std::vector<BoxNode> boxes;
    int  n_roots             = 0;
    int  n_boxes_total       = 0;
    int  n_ffb_success       = 0;
    int  n_ffb_fail          = 0;
    int  n_promotions        = 0;
    int  n_components        = 0;     // 连通分量数
    bool start_goal_connected = false;
    double total_volume      = 0.0;
    double build_time_ms     = 0.0;
    std::unordered_map<std::string, double> phase_times;
};
```

---

## 5. 算法流程

### 5.1 Root 选取

#### 无端点模式（覆盖最大化）

```
算法: 最远点采样 (Farthest Point Sampling)
1. 随机采样第一个 seed，FFB 生成第一个 root box
2. 对 i = 2..n_roots:
   a. 随机采样 K 个候选点
   b. 对每个候选点，计算它到所有已有 root 的最小距离
   c. 选择最小距离最大的候选点作为新 root seed
   d. FFB 生成 root box
```

#### 有端点模式（start/goal 连接）

```
1. start 和 goal 强制作为 root 0 和 root 1
2. 对剩余 root i = 2..n_roots:
   a. 在 start-goal 连线的中间区域采样候选点
      - 沿 s→g 方向均匀分布
      - 垂直方向高斯扰动 (σ = 0.3 × ‖s-g‖)
   b. 从候选点中做 FPS 选取
   c. FFB 生成 root box
```

### 5.2 LECT 子树划分

```
算法: 按 root 位置确定子树归属
1. 在 LECT root 节点处展开，根据各 root seed 的坐标，
   确定每个 root 落在哪个 LECT 子节点中
2. 记录 subtree_root_node_[i] = LECT 节点索引
3. 记录 subtree_limits_[i] = 该子树对应的 C-space 子区域

注意: 单线程版本中，子树划分主要用于引导 wavefront
      采样方向和 RRT 随机采样范围。
      多线程版本中，每个线程只对自己的子树调用 FFB。
```

### 5.3 Wavefront (前波扩展)

```
输入: roots[], obstacles, config
输出: boxes[], adjacency_graph

1. 初始化:
   expand_queue ← 所有 root boxes
   miss_count ← 0

2. 主循环 (while n_boxes < max_boxes AND miss < max_miss AND !timeout):

   2a. 如果 expand_queue 非空:
       entry ← expand_queue.pop_front()
       box ← boxes[entry.box_id]

       // 选择要探索的面
       faces ← 所有 2×n_dims 个面
       移除 entry.expand_face (来时的面，避免回头)
       移除超出 joint limits 的面

       // 目标偏置面选择
       如果有 goal:
           以 goal_face_bias 概率选择最朝向 goal 的面
           以 (1 - goal_face_bias) 概率随机选面
       否则:
           随机选面

       // 生成边界种子
       对选中的 n_boundary_samples 个面:
           seed ← box 中心，在 dim 方向偏移 ±(edge/2 + epsilon)
           其他维度在 box 范围内随机抖动

           如果 seed 在已占用区域 → skip
           box_id ← try_create_box(seed, obstacles, parent=box.id, face)

           如果成功:
               adjacency_graph.add_box(new_box)    // O(log N + K) 增量
               expand_queue.push_back(new_box)
               miss_count ← 0
           否则:
               miss_count++

   2b. 如果 expand_queue 为空 (随机回退):
       seed ← uniform_random(joint_limits)
       box_id ← try_create_box(seed, obstacles)
       如果成功:
           adjacency_graph.add_box(new_box)
           expand_queue.push_back(new_box)
           miss_count ← 0
       否则:
           miss_count++

3. Promotion 阶段:
   promote_all(obstacles)  // 合并相邻叶节点

4. 最终 adjacency sweep-and-prune rebuild
   （全量重建 O(N·K)，修正增量期间可能的遗漏）
```

### 5.4 RRT-like (类 RRT 扩展)

```
输入: roots[], obstacles, config
输出: boxes[], adjacency_graph

1. 初始化:
   所有 root boxes 加入 adjacency_graph

2. 主循环 (while n_boxes < max_boxes AND miss < max_miss AND !timeout):

   2a. 采样随机目标点 q_rand:
       以 rrt_goal_bias 概率:
           如果有 goal: q_rand ← goal
           否则: q_rand ← uniform_random
       以 (1 - rrt_goal_bias) 概率:
           q_rand ← uniform_random(joint_limits)

   2b. 找最近 box:
       nearest ← adjacency_graph.find_nearest_box(q_rand)

   2c. 计算扩展种子:
       direction ← q_rand - nearest.center()
       step ← rrt_step_ratio × max_dim_width
       如果 ‖direction‖ > step:
           direction ← direction.normalized() × step
       extend_seed ← nearest.nearest_point_to(nearest.center() + direction)
       clamp(extend_seed, joint_limits)

   2d. FFB 创建 box:
       box_id ← try_create_box(extend_seed, obstacles, parent=nearest.id)
       如果成功:
           adjacency_graph.add_box(new_box)
           miss ← 0
       否则:
           miss++

3. Promotion 阶段 + 最终 adjacency sweep-and-prune rebuild O(N·K)
```

---

## 5.5 邻接检测加速：Sweep-and-Prune 算法

移植自 v2 (`deoverlap.cpp`)，核心思想：**利用排序避免枚举所有 N² 对**。

```
算法: Sweep-and-Prune 邻接检测

═══ 预处理 ═══

1. 维度排名:
   对每个维度 d = 0..D-1:
     fill_ratio[d] = avg(box_width[d]) / (limit_hi[d] - limit_lo[d])
   按 fill_ratio 升序排列维度:
     sweep_dim  ← fill_ratio 最小的维度 (最分散 → 最强剪枝)
     filter_dim ← fill_ratio 次小的维度
     dim_order  ← 剩余维度按 fill_ratio 升序

2. 打包为行优先平坦数组:
   flat_lo[i * D + d] = box_i.lo[d]     // 每个 box 占 D×8 bytes ≈ 1 cache line
   flat_hi[i * D + d] = box_i.hi[d]

3. 按 sweep_dim 排序:
   sweep_order ← argsort(flat_lo[:, sweep_dim])

═══ 批量检测 (rebuild) ═══

4. 双指针 sweep:
   对 i = 0..N-1 (按 sweep_order):
     对 j = i+1..N-1 (按 sweep_order):
       // ── sweep 维度剪枝 ──
       如果 flat_lo[j][sweep_dim] > flat_hi[i][sweep_dim] + tol:
           BREAK  ← 排序保证后续的 j 也不会重叠

       // ── filter 维度剪枝 ──
       如果 flat_lo[j][filter_dim] > flat_hi[i][filter_dim] + tol
       或者 flat_lo[i][filter_dim] > flat_hi[j][filter_dim] + tol:
           CONTINUE

       // ── 全维度检查 (窄→宽) ──
       adjacent = true
       has_touching = false
       对 dim_order 中的每个 d:
           overlap = min(hi_i[d], hi_j[d]) - max(lo_i[d], lo_j[d])
           如果 overlap < -tol: adjacent = false; BREAK
           如果 overlap <= tol: has_touching = true

       如果 adjacent && has_touching:
           记录邻接对 (i, j)

   复杂度: O(N · K), K = 平均 sweep 方向重叠数, 通常 K ≪ N

═══ 增量添加 (add_box) ═══

5. 新 box 插入已排序的 sweep_order:
   pos ← binary_search(sweep_order, new_box.lo[sweep_dim])
   
   // 向两侧扫描 (只看 sweep 方向重叠的 box)
   向左扫描: j = pos-1, pos-2, ...
     直到 flat_hi[j][sweep_dim] < new_box.lo[sweep_dim] - tol → BREAK
     执行 filter + 全维度检查
   向右扫描: j = pos+1, pos+2, ...
     直到 flat_lo[j][sweep_dim] > new_box.hi[sweep_dim] + tol → BREAK
     执行 filter + 全维度检查

   插入 new_box 到 sweep_order 中
   复杂度: O(log N + K)

═══ 诊断输出 ═══

6. 打印:
   [adjacency] N=5000 sweep=3 filter=1
               sweep_cand=48231 filter_cand=12019 pairs=891
               brute=12497500 sweep_prune=99.6% dual_prune=99.9%
```

**两种邻接条件**:
- **overlaps**: 所有维度上 overlap > 0 (box 体积相交)
- **adjacent**: 所有维度 overlap ≥ -tol，且至少一个维度 overlap ≤ tol (面接触但不深入)

SBF 使用 **adjacent** 检查——box 之间通过共享面连接，允许零厚度接触。

---

## 6. 实现步骤

### Phase 1: 基础设施 ✅ 已完成

| 步骤 | 内容 | 状态 |
|---|---|---|
| 1.1 | 修改 `types.h`：BoxNode 新增 3 个拓展元数据字段 + `root_id` | ✅ |
| 1.2 | 创建 `grower_config.h`：GrowerConfig 配置结构体 | ✅ |
| 1.3 | 创建 `adjacency_graph.h` + `.cpp`：AdjacencyGraph 完整实现 | ✅ |

### Phase 2: ForestGrower 核心 ✅ 已完成

| 步骤 | 内容 | 状态 |
|---|---|---|
| 2.1 | 创建 `forest_grower.h`：类声明 + GrowerResult | ✅ |
| 2.2 | 实现 root 选取逻辑 (FPS + start/goal 模式) | ✅ |
| 2.3 | 实现 LECT 子树划分 | ✅ |
| 2.4 | 实现 `try_create_box()` — FFB 封装 | ✅ |
| 2.5 | 实现 `grow_wavefront()` | ✅ |
| 2.6 | 实现 `grow_rrt()` | ✅ |
| 2.7 | 实现 `grow()` 主入口（dispatch + promotion + 统计） | ✅ |

### Phase 3: 测试与集成 ✅ 已完成

| 步骤 | 内容 | 状态 |
|---|---|---|
| 3.1 | 更新 `CMakeLists.txt` 注册测试目标 | ✅ |
| 3.2 | 创建 `test_forest_grower.cpp`（5 个测试全部通过） | ✅ |
| 3.3 | 编译 + 运行测试 | ✅ |
| 3.4 | 更新 `docs/API_REFERENCE.md` | ✅ |

### Phase 4: 线程池并行 ✅ 已完成

| 步骤 | 内容 | 状态 |
|---|---|---|
| 4.1 | 创建 `thread_pool.h`：C++17 header-only 线程池 | ✅ |
| 4.2 | `GrowerConfig` 新增 `n_threads` 参数 | ✅ |
| 4.3 | 实现 `grow_subtree()`：单子树独立扩展 | ✅ |
| 4.4 | 实现 `grow_parallel()`：线程池调度 + 结果合并 + ID 重映射 | ✅ |
| 4.5 | `CMakeLists.txt` 添加 `Threads::Threads` | ✅ |
| 4.6 | 创建 `test_parallel_grower.cpp`（5 个测试全部通过） | ✅ |
| 4.7 | Benchmark: serial 169ms → parallel 60ms = **2.83× speedup** | ✅ |

---

## 7. 管线优化空间分析

> 截至 2026-03-16，Phase 1-4 全部完成。以下按 **收益/难度** 排序列出剩余优化方向。

### 7.1 已完成的优化

| 优化项 | 实现方式 | 实测效果 |
|---|---|---|
| 多根并行扩展 | `ThreadPool` + 每 worker 独立 `ForestGrower`（独立 LECT/RNG），零共享可变状态 | 4 threads: **2.83× speedup**（200 boxes, 7-DOF） |
| Sweep-and-prune 邻接图 | 维度排序 + 双维度预过滤 + 行优先平坦数组 | 5000 boxes 剪枝 99.6%，邻接对从 12.5M 降至 891 |
| 增量邻接检测 | 二分查找 + 局部扫描 | O(log N + K) vs O(N) |
| 目标引导面选择 | Goal-face bias (0.6 概率选最朝向目标的面) | start-goal 连通率显著提升 |
| 子树分区采样 | C-space KD-tree 分割，每个 root 在自己的区域采样 | 覆盖均匀，减少 FFB 冲突 |

### 7.2 高收益优化（推荐优先实现）

#### ① LECT Warm-Start / 共享只读树层 ✅ (Phase 7)

**问题**：当前每个 parallel worker 从零构建自己的 LECT，重复计算根节点附近的 FK + envelope。
**方案**：主线程先构建到 depth=2~4 的 LECT 骨架（只读层），worker 线程在其之上继续 split + mark_occupied（写时复制或引用上层）。
**实现**：
- `NodeStore::snapshot()` — 深拷贝树结构 + per-link AABB，清空 occupation/forest_id/subtree_occ
- `LECT::snapshot()` — 手动拷贝全部成员（robot, store, ep_store, hull_grids, scene_grid, root_fk, split_dims），跳过不可拷贝的 `frame_store_`（worker 不需要持久化）
- `LECT::pre_expand(target_depth)` — 递归 `split_leaf()`，配合 `compute_fk_incremental()` 传递增量 FK
- `ForestGrower(robot, config, LECT warm_lect)` — warm-start 构造器，移入预建 LECT
- `grow_parallel()` — 自动深度 heuristic（默认 depth=3），snapshot per worker 放入 `shared_ptr`（兼容 `std::function` copyable 要求）
- `GrowerConfig::warm_start_depth` — 0=自动, -1=禁用
**实测结果**：pre_expand(2) → 7 nodes / pre_expand(3) → 15 nodes, snapshot + occupation 独立性验证通过。25/25 全量回归通过。
**难度**：中

#### ② 跨子树边界桥接 ✅ (Phase 8)

**问题**：当前并行 worker 各管各的子树，合并后的森林可能在子树边界处缺少邻接连接。
**方案**：合并阶段增加一步 boundary seeding——在两个子树共享面处注入采样种子，尝试在边界生成 bridge boxes。
**实现**：
- `sync_lect_occupation()` — 合并后遍历 coordinator LECT，为每个 merged box 标记 occupation
- `bridge_subtree_boundaries()` 重写 — Phase 0: occupation sync; Phase 1: 共享面检测; Phase 2: 近边界 box 收集; Phase 3: 跨根交替采样（even→i→j 用 root_j, odd→j→i 用 root_i）; Phase 4: proximity-based seeding
- `GrowerResult::n_bridge_boxes` — 统计 bridge box 数量
- **额外修复**：发现并修复 `grow_wavefront()` 中的 dangling reference 缺陷——`const BoxNode& box` 在 `try_create_box()` 的 `push_back` 后失效，导致 `root_id` 读取垃圾值。改为循环前拷贝 `expand_from_id` 和 `expand_root_id`。
**实测结果**：obstacles + endpoints 场景下 2 roots / 2 subtrees，bridge 成功生成 1-3 boxes/run。30/30 全量回归通过。
**难度**：低

#### ③ 自适应 min_edge 阶段策略 ✅ (Phase 9)

**问题**：固定 `min_edge = 0.01` 在密集障碍区域浪费时间，在空旷区域不够积极。
**方案**：借鉴 v2 的 phase 机制——先以大 min_edge (0.1) 快速铺满，再以小 min_edge (0.01) 在稀疏区域填充。
**实测结果**：80 boxes 场景下 adaptive 总体积 0.033 vs fixed 0.014 (2.3× 提升)；Phase 转换后 miss_count 重置 + wavefront 重新入队确保 fine 阶段继续生长。
**难度**：低

#### ④ KD-tree 加速 `find_nearest_box` ✅ (Phase 6)

**问题**：RRT 模式的 `find_nearest_box()` 当前为 O(N) 线性扫描。
**方案**：自研 header-only KD-tree (`kd_tree.h`)，基于 box interval 子树包围盒剪枝，查询 O(log N)。
**实测结果**：2000 boxes / 5000 queries: brute-force 242ms → KD-tree 35ms = **6.9× 加速**。
**关键修复**：中心距离剪枝对 box-distance 度量无效，改为子树 AABB 下界剪枝；Entry 存储 interval 拷贝而非指针（避免悬垂引用）。
**难度**：低→中（box-distance 剪枝需要子树包围盒）

### 7.3 中等收益优化

#### ⑤ SIMD 加速邻接内核 ✅ (Phase 6)

**问题**：全维度邻接检查是 sweep-and-prune 的内层循环，当前是标量逐维比较。
**方案**：AVX2 `__m256d` intrinsics，`padded_dims_` (n_dims 向上取整到 4 的倍数)，padding 维填 lo=0/hi=1 (中性值不影响判结果)。
**实现**：`check_adjacent_flat()` AVX2 路径 + 标量 fallback，`rebuild_flat_arrays()` padded 布局，CMake 添加 `/arch:AVX2` (MSVC)。
**实测结果**：500 boxes rebuild 0.25ms；当前测试机走 scalar fallback（MSVC 未定义 `__AVX2__` 宏，需手动 `#define`）。
**难度**：中

#### ⑥ Work-stealing 动态负载均衡 ✅ (Phase 5)

**问题**：当前每个 worker 的 box budget 是静态均分，但子树大小不均匀（靠近障碍的子树 FFB 失败多，早早停止）。
**方案**：改为共享原子计数器 `total_boxes`，每个 worker 用 `fetch_add` 自增，直到全局预算耗尽。
**实测结果**：worker 利用率从 ~70% → 93%，多线程加速比 2.83× → 3.41×。
**难度**：低

#### ⑦ Envelope Cache 跨 worker 共享

**问题**：不同 worker 可能对 LECT 同一位置的子树做相同的 FK + envelope 计算（overlap 区域）。
**方案**：将已计算的 envelope 存入 concurrent hashmap（key = node_idx），worker 先查再算。
**预期收益**：在 hull-grid 管线中减少 20-30% 的 FK 调用。
**难度**：高（需要线程安全 data structure + 粒度控制）

### 7.4 长期优化方向

| 优化项 | 描述 | 预期收益 | 难度 |
|---|---|---|---|
| GPU 并行碰撞检测 | Hull-grid 碰撞判定迁移到 CUDA/Vulkan compute | Box 生成 5-10× 加速 | 很高 |
| ✅ Coarsening（贪心合并） | 相邻同 root 的 box 合并为更大 box（减少总数） | 减少邻接图规模 30-50% | 中 |
| 路径规划集成 | Dijkstra/A* 在邻接图上搜索 + 内点平滑 | 完整 S→G 规划管线 | 中 |
| 自适应 sweep dim | 增量模式下动态跟踪 fill ratio，分布显著变化时重选 sweep/filter 维度 | 邻接增量步更稳定 | 低 |
| 渲染反馈引导 | 可视化中实时显示覆盖热力图，人工指定稀疏区域优先扩展 | 交互式调参 | 中 |

### 7.5 管线各阶段耗时分析

基于 Benchmark（7-DOF, 4 roots, 200 boxes, `sub_aabb` pipeline）的典型耗时占比：

```
Phase 4 串行管线 (169.4 ms 总计):
  ┌──────────────────────┬──────┬───────┐
  │ 阶段                 │ 耗时 │  占比 │
  ├──────────────────────┼──────┼───────┤
  │ root_select          │ ~5ms │   3%  │
  │ partition_subtrees   │ <1ms │  <1%  │
  │ expand (wavefront)   │~140ms│  83%  │  ← 主要瓶颈 (FFB = FK + envelope)
  │ promote_all          │ ~5ms │   3%  │
  │ adj_rebuild          │~18ms │  11%  │
  └──────────────────────┴──────┴───────┘

Phase 5 并行管线 (47.8 ms 总计, 4 threads):
  ┌──────────────────────┬──────┬───────┐
  │ 阶段                 │ 耗时 │  占比 │
  ├──────────────────────┼──────┼───────┤
  │ root_select          │ ~5ms │  10%  │
  │ partition_subtrees   │ <1ms │  <1%  │
  │ expand (parallel)    │~24ms │  50%  │  ← work-stealing + O(1) lookup
  │ merge + remap        │ ~1ms │   2%  │
  │ adj_rebuild          │~17ms │  36%  │  ← 成为新的比例瓶颈
  └──────────────────────┴──────┴───────┘

演进:
  Phase 4: serial 169ms → parallel 60ms = 2.83× speedup
  Phase 5: serial 165ms → parallel 48ms = 3.41× speedup (↑ 20%)
  
  主要改进:
  ① O(1) box_id 查表 (unordered_map) — 消除扩展循环 O(N) 查找
  ② Work-stealing 原子计数器 — worker 利用率 70% → 93%
  ③ 自适应 min_edge — 粗→细两阶段，空旷区覆盖更快
  ④ 边界桥接 (可选) — bridge_samples 控制，默认关闭
  
  下一个瓶颈是 adj_rebuild (17ms, 36%)，可用 SIMD 优化 (⑤)

  Phase 6: KD-tree + SIMD 优化
  ④ KD-tree find_nearest_box: O(N)→O(log N), 6.9× 加速 (2000 boxes)
  ⑤ SIMD 邻接内核: AVX2 padded layout, scalar fallback 已验证
  新增文件: kd_tree.h, test_phase6_kdtree_simd.cpp (5 tests ALL PASSED)
  全部 20 个测试通过 (Phase 1-6 无回归)

  Phase 7: LECT Warm-Start / 共享只读树层
  ① NodeStore::snapshot() — 深拷贝 + 清空 occupation
  ② LECT::snapshot() — 手动拷贝全成员, 跳过 frame_store_
  ③ LECT::pre_expand(depth) — 递归 split_leaf + incremental FK
  ④ ForestGrower warm-start 构造器 + grow_parallel 两路径 (warm/legacy)
  ⑤ GrowerConfig::warm_start_depth (0=auto, -1=disabled)
  修改文件: node_store.h/.cpp, lect.h/.cpp, forest_grower.h/.cpp, grower_config.h
  新增文件: test_phase7_warmstart.cpp (5 tests ALL PASSED)
  全部 25 个测试通过 (Phase 1-7 无回归)

  Phase 8: 跨子树边界桥接 + dangling reference 修复
  ① sync_lect_occupation() — 合并后恢复 coordinator LECT occupation 状态
  ② bridge_subtree_boundaries() 重写 — 共享面检测 + 近边界 box 收集 + 跨根交替采样
  ③ GrowerResult::n_bridge_boxes — 统计 bridge 产生的 box 数量
  ④ grow_wavefront() dangling ref 修复 — const BoxNode& 在 push_back 后失效
     → 改为循环前拷贝 expand_from_id/expand_root_id (根治随机 root_id 垃圾值)
  修改文件: forest_grower.h, forest_grower.cpp
  新增文件: test_phase8_bridging.cpp (5 tests ALL PASSED)
  全部 30 个测试通过 (Phase 1-8 无回归)

  Phase 9: 自适应 min_edge 两阶段策略
  ① grow_wavefront() / grow_rrt() — in_coarse_phase 状态变量替代每次重算
  ② 转换时 miss_count 重置 — fine min_edge 开启大量新 FFB 站点
  ③ wavefront 转换时重新入队 — coarse 阶段的 box 用 fine min_edge 重新探索边界
  ④ grow_rrt() dangling ref 预防 — 拷贝 extend_from_id/extend_root_id
  ⑤ GrowerResult::n_coarse_boxes / n_fine_boxes — 逐 phase 统计
  ⑥ grow_parallel() 聚合 worker 的 coarse/fine 计数
  ⑦ grow() 日志输出 adaptive 统计
  修改文件: forest_grower.h, forest_grower.cpp, grower_config.h(已有), CMakeLists.txt
  新增文件: test_phase9_adaptive_min_edge.cpp (5 tests ALL PASSED)
  全部 35 个测试通过 (Phase 1-9 无回归)

  Phase 10: 贪心合并 (Greedy Coarsening)
  ① LECT::intervals_collide_scene() — 对任意 C-space intervals 做两阶段碰撞检测
     (endpoint AABB → link AABBs → obstacle overlap; grid 模式 SubAABB → hull grid)
  ② ForestGrower::remove_box_by_id() — O(1) 查找 + LECT unmark + swap-with-last
  ③ ForestGrower::coarsen_greedy() — 每轮重建邻接 → 收集相邻对 → hull merge
     → score = hull_vol / sum_vol → 降序排序 → 贪心执行 → collision check
     → 移除旧对 → 创建合并 box → 更新 start/goal IDs
  ④ grow() Phase 3b: 在 expansion/promotion 之后、最终 adj_rebuild 之前调用
  ⑤ GrowerConfig: coarsen_enabled, coarsen_target_boxes, max_coarsen_rounds, coarsen_score_threshold
  ⑥ GrowerResult::n_coarsen_merges — 统计合并次数
  修改文件: grower_config.h, forest_grower.h, forest_grower.cpp, lect.h, lect.cpp, CMakeLists.txt
  新增文件: test_phase10_coarsening.cpp (6 tests ALL PASSED)
  全部 41 个测试通过 (Phase 1-10 无回归)

  Phase 11: Python 2D 可视化工具包 (src/viz/)
  ① core.py — GrowVizConfig, Obstacle2D, CSpace2D, BoxInfo 数据类
  ② ffb_engine.py — 2D Find-Free-Box KD-tree 引擎, 支持 set_effective_min_edge()
  ③ forest_grower_2d.py — ForestGrower2D 仿真 (Wavefront/RRT/Multi-thread/Adaptive)
     - RRT boundary-snap: 新 box 紧邻最近 box 的边界面 (消除零散小 box)
     - 全空间 FPS root 选取 (K=80 均匀候选, 覆盖整个 C-space)
     - Adaptive 两阶段: 先 coarse (大 box) 后 fine (小 box), 效果显著
     - 多线程 round-robin 模拟: 各 subtree 独立 queue, 全局 miss 计数
     - Per-tree goal bias: _get_bias_target(root_id) — start→goal, goal→start, rand→50/50
  ④ render.py — PatchCollection + LineCollection 加速渲染 (~2× 提速)
  ⑤ compare_expansion.py — 4-模式 2×2 并排 GIF 对比
  ⑥ multiprocess.py — 多进程 ForestGrower 可视化
  ⑦ wavefront_step.py — 单步波前扩展详细动画
  ⑧ __init__.py — 完整 API 导出 (GrowVizConfig, ForestGrower2D, plot_4panel 等)
  ⑨ 默认 snapshot_every=1 (逐框记录), CLI --snap-every=2
  文档更新: docs/API_REFERENCE.md §8, docs/PAPER_CODE_MAP.md (新增 viz 映射)

  Phase 12: Per-tree Goal Bias (C++ 整合)
  ① get_bias_target(root_id) — start tree(0)→goal, goal tree(1)→start, random tree(≥2)→50/50
  ② grow_wavefront() — 每个 box 按其 root_id 获取偏置目标，传给 sample_boundary()
  ③ grow_rrt() — goal_bias 采样时随机选一个 box 的 root_id 决定偏置方向
  ④ sample_boundary() — 参数从 goal_pt 重命名为 bias_target，to_goal → to_target
  ⑤ 删除 grow_wavefront() 中未使用的 goal_ptr 偶变量
  文档更新: API_REFERENCE.md (新增 get_bias_target, 更新 wavefront/RRT 描述)
             PAPER_CODE_MAP.md (新增 per-tree goal bias 映射)
  全部 41 个测试通过 (Phase 1-12 无回归)

  Phase 13: Boundary-snap & 减少独立小 box (C++ + Python 重构)
  ① rrt_snap_to_face(nearest, direction, step) — RRT 边界吸附:
     找到 nearest box 上与 direction 最对齐的面, 在该面上放 seed (70% 方向 + 30% 随机)
     消除 RRT 模式下的零散独立小 box, 保证新 box 紧邻已有树
  ② sample_near_existing_boundary(root_id) — 从已有 box 的随机面上采样 seed:
     用于 wavefront queue 耗尽时的 fallback, 替代纯随机采样
     保证 fallback 产生的 box 也紧邻已有树, 减少孤立碎片
  ③ grow_wavefront() boundary-aware fallback:
     - Queue 空 + coarse phase: 尝试 30 次 boundary refill → 失败则 force 切 fine phase
     - Queue 空 + fine phase: 先 sample_near_existing_boundary(), 失败再用 subtree random
     - 继承 nearest box 的 root_id (保持树归属一致)
  ④ grow_rrt() boundary-snap:
     - 用 rrt_snap_to_face() 替代原来的 nearest_center + direction 纯延伸
     - 传递 face_dim / face_side 给 try_create_box (正确记录展开面信息)
  ⑤ Python _rrt_snap_to_face() 提取:
     - 从 _grow_rrt() 中提取 ~40 行边界吸附逻辑为独立方法
     - _grow_rrt() 现在只调用 _rrt_snap_to_face(), 不含内联算法
     - 对齐 C++ rrt_snap_to_face() 的接口和行为
  修改文件: forest_grower.h, forest_grower.cpp, src/viz/forest_grower_2d.py
  文档更新: API_REFERENCE.md (新增 rrt_snap_to_face, sample_near_existing_boundary)
             PAPER_CODE_MAP.md (新增 boundary-snap 和 boundary-aware fallback 映射)
  全部 41 个测试通过 (Phase 1-13 无回归)
```

---

## 8. 遵循规范

本实现遵循 [AI_WORKING_SPEC.md](../AI_WORKING_SPEC.md) 的以下规则：

- **Triple-Sync**: 新增 C++ 函数同步更新 `docs/API_REFERENCE.md`
- **Coding Standards**: C++17, snake_case 函数/变量, PascalCase 类型, `#pragma once`
- **Tests**: 每个公共 API 在 `tests/` 中有对应测试
- **Module mapping**: ForestGrower 属于 `forest/` 模块 (Paper Sec IV)
