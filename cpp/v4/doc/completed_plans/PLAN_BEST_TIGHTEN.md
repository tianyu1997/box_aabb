# LECT 启发式维度选择优化计划：BEST_TIGHTEN

> 状态：待实施
> 日期：2026-03-24
> 影响模块：`core/config`, `forest/lect`
> 关联论文章节：§IV.1 (LECT)

---

## 1. 背景与动机

### 1.1 现状

LECT 的 KD-tree 在分裂节点时需选择沿哪个 C-space 维度（关节轴）进行二分。
当前 v4 支持两种维度选择策略 (`SplitOrder`)：

| 策略 | 行为 | 特点 |
|------|------|------|
| `ROUND_ROBIN` (=0) | 按 `[0,1,...,n-1,0,1,...]` 循环 | 确定性，深度 d 固定选维度 d%n |
| `WIDEST_FIRST` (=2) | 每次选当前区间最宽的维度 | 同深度不同节点可能选不同维度 |

两者都未直接考虑 **二分后 iAABB 实际能收紧多少**。理想情况下，应优先切分那些切分后能最大幅度收紧 iAABB 的维度。

### 1.2 核心观察

iAABB 过估计的根源是 **区间矩阵乘法链** 中的累积膨胀。每个关节 $d$ 的 DH 矩阵包含
`I_cos(θ_d)` 和 `I_sin(θ_d)` 区间条目，这些条目通过 `imat_mul_dh` 向下游传播：

$$T^{0:k} = T^{0:0} \cdot A_0(\theta_0) \cdot A_1(\theta_1) \cdot \ldots \cdot A_{k-1}(\theta_{k-1})$$

端点 $p_k$ 的位置区间取自 $T^{0:k}$ 的第 3 列（平移列），是所有上游 DH 矩阵
旋转区间条目与位移条目的 **区间内积累积**。

**因此：不同维度二分后对最终 iAABB 的收紧效果取决于完整的 FK 链传播，
而非单个关节的局部三角区间宽度变化。**

### 1.3 早期方案（三角宽度启发式）的局限性分析

最初考虑的方案是基于单关节三角宽度收紧量 $\times$ 下游权重的启发式：
$$\text{score}(d) = \bigl[w_{\cos+\sin}^{\text{full}}(d) - \max(w_{\cos+\sin}^L, w_{\cos+\sin}^R)\bigr] \times (n_j - d)$$

**该方案存在三个根本缺陷**：

#### 缺陷 1：忽略 FK 链的乘法传播效应

三角宽度启发式只看 `I_cos(θ_d) + I_sin(θ_d)` 的**局部**宽度变化，
忽略了这些条目如何通过 `imat_mul_dh` 的 12 个有效矩阵乘积项传播到下游。

在 DH 矩阵 $A_d$ 中，`cos θ_d` 和 `sin θ_d` 分布在特定位置：
```
A[0] = cos θ     A[1] = -sin θ        A[3] = a
A[4] = sin θ·cα  A[5] = cos θ·cα      A[7] = -d·sα
A[8] = sin θ·sα  A[9] = cos θ·sα      A[11] = d·cα
```

当 `imat_mul_dh(prefix[d], A[d], prefix[d+1])` 执行时，
`prefix[d+1]` 的每个元素是 `prefix[d]` 的一行与 `A[d]` 的一列的区间内积。
trig 区间宽度变化通过 **与 prefix 中旋转区间的乘积** 放大或衰减。
纯局部的 `I_cos + I_sin` 宽度无法反映这种交互。

#### 缺陷 2：线性权重 $(n_j - d)$ 是弱代理

真实影响是**乘法级联**：关节 $d$ 的 DH 矩阵条目与关节 $d+1, d+2, \ldots$
的累积旋转矩阵做区间乘法。影响量取决于整条链上所有关节区间的当前宽度，
是高度非线性的。简单的线性计数 $(n_j - d)$ 无法捕捉：
- 下游关节区间已经很窄时，上游的收紧有更大相对效果
- 下游关节区间很宽时，乘法膨胀使上游的收紧被淹没

#### 缺陷 3：忽略 DH 参数的影响

不同机器人的 $\alpha, a, d$ 参数不同。例如：
- 当 $\alpha_{d+1} = \pm\pi/2, a_{d+1} = 0$ (典型腕部关节) 时，
  DH 矩阵相当于纯旋转，trig 膨胀直接传递
- 当 $a_{d+1} \neq 0$ 时，平移项 $a$ 在乘法中作为常数偏移，
  不放大 trig 区间但贡献固定平移

这些结构信息只有完整 FK 链计算才能正确反映。

### 1.4 改进方案：iFK 探针法 (FK Probe)

**核心思想**：既然每个深度只评估一次，可以对每个候选维度直接执行
`compute_fk_incremental` + `extract_link_aabbs`，
度量真实的 per-link iAABB 体积来选择最优维度。

**优势**：
- **直接度量目标量**：不使用代理指标，直接比较分裂后的 link iAABB 体积
- **自动包含 FK 链传播**：`compute_fk_incremental` 从 `changed_dim` 开始
  重新链式乘法，完整反映区间膨胀传播
- **自动适应 DH 参数**：DH 的 $\alpha, a, d$ 值全部参与 `build_dh_joint` + `imat_mul_dh`
- **Source 无关**：无论实际管线用 iFK / CritSample / GCPC，iFK 探针作为
  维度排名的代理始终有效（维度排名在不同 source 之间高度相关）

**代价分析**（以 7-DOF iiwa14 为例，每个深度仅执行一次）：

| 操作 | 每候选维度 | 7 个候选总计 |
|------|-----------|-------------|
| `compute_fk_incremental` | 2 次 (左/右) | 14 次 |
| `imat_mul_dh` 调用 | $2 \times (n_j - d)$ | $2 \times \sum_{d=0}^{6}(7-d) = 56$ 次 |
| `extract_link_aabbs` | 2 次 | 14 次 |
| FKState memcpy (~8KB) | 2 次 | 14 次 |

对比：单次 `compute_fk_full` = 7 次 `imat_mul_dh`。
探针总计相当于 **8 次完整 FK**，对比整棵树数千次 FK 调用可完全忽略。

### 1.3 设计约束

**不应每次 `split_leaf` 都独立触发维度选择启发式。**

理由：
- 同深度的所有节点共享相同的"已分裂维度序列"历史
- `ROUND_ROBIN` 和 `WIDEST_FIRST` 以外的策略（如本特性）需要保证同深度一致性，
  否则 `node_intervals()` 的维度回溯在 `node_split_dim_` 缺失时无法正确推断
- 缓存一致性：预热 (`pre_expand`) 和运行时延迟分裂应选同样的维度
- 开销最小化：对 d 维度的启发式评估只需做一次，后续同深度节点直接复用

**策略：某个深度第一次 `split_leaf` 时触发启发式、确定该深度的切分维度，
之后同一深度的所有节点复用此结果。**

---

## 2. 算法设计

### 2.1 按深度懒惰选维 (Per-Depth Lazy Dimension Selection)

```
split_dims_[depth] — 按深度索引的维度数组，初始全部为 -1（未决定）

split_leaf(node, depth, parent_fk, intervals):
    if split_dims_[depth] == -1:
        // 首次在此深度分裂 → iFK 探针选维
        split_dims_[depth] = pick_best_tighten_dim(parent_fk, intervals)
    dim = split_dims_[depth]
    ... 正常二分 ...
```

`split_dims_` 的语义变化：

| 策略 | `split_dims_` 大小 | 索引方式 | 填充时机 |
|------|-------------------|----------|----------|
| `ROUND_ROBIN` | `n_joints` | `depth % n_joints` | 构造时一次性 |
| `WIDEST_FIRST` | 空 | 不使用 (每次动态算) | — |
| `BEST_TIGHTEN` (新) | `max_depth` (128) | `depth` (直接索引) | 懒惰填充（首次访问） |

### 2.2 iFK 探针评分函数 `pick_best_tighten_dim()`

对每个候选维度 $d$，分别构造左右子区间，执行增量 FK，
提取 per-link iAABB，计算总体积，选择使**较大子树总体积最小**的维度。

**度量**：$\text{metric}(d) = \max\bigl(\text{vol}_L(d),\; \text{vol}_R(d)\bigr)$

选择使 $\text{metric}(d)$ **最小**的维度。使用 max 而非 sum 的原因：
min-max 策略确保**较差子节点**尽量紧凑——这直接影响 FFB 能否在低深度
通过碰撞检测提前返回。若用 sum，可能选择"一个子节点极紧 + 另一个极松"的维度，
而松的那个恰好是 FFB 需要继续分裂的那个。

```
pick_best_tighten_dim(parent_fk, intervals[0..n-1]):
    nj = n_joints
    nact = n_active_links
    radii = active_link_radii

    best_dim = argmax(intervals[d].width())   // fallback
    best_metric = +INF

    for d in [0, nj):
        if intervals[d].width() <= 0: continue

        mid = intervals[d].center()

        // ── Probe left child ────────────────────────────────────
        left_ivs = intervals;  left_ivs[d].hi = mid
        left_fk = compute_fk_incremental(parent_fk, robot, left_ivs, d)
        float left_aabbs[nact × 6]
        extract_link_aabbs(left_fk, active_link_map, nact, left_aabbs, radii)

        // ── Probe right child ───────────────────────────────────
        right_ivs = intervals;  right_ivs[d].lo = mid
        right_fk = compute_fk_incremental(parent_fk, robot, right_ivs, d)
        float right_aabbs[nact × 6]
        extract_link_aabbs(right_fk, active_link_map, nact, right_aabbs, radii)

        // ── Compute per-link volumes and aggregate ──────────────
        vol_left = 0.0, vol_right = 0.0
        for i in [0, nact):
            vol_left  += (left_aabbs[i*6+3] - left_aabbs[i*6+0]) *
                         (left_aabbs[i*6+4] - left_aabbs[i*6+1]) *
                         (left_aabbs[i*6+5] - left_aabbs[i*6+2])
            vol_right += (right_aabbs[i*6+3] - right_aabbs[i*6+0]) *
                         (right_aabbs[i*6+4] - right_aabbs[i*6+1]) *
                         (right_aabbs[i*6+5] - right_aabbs[i*6+2])

        metric = max(vol_left, vol_right)

        if metric < best_metric:
            best_metric = metric
            best_dim = d

    return best_dim
```

**为什么用 iFK 探针而非三角宽度启发式**（详见 §1.3 分析）：
- 三角宽度启发式 $\Delta_{\text{trig}}(d) \times (n_j - d)$
  忽略了 FK 链的乘法传播、DH 参数差异和关节间区间交互
- iFK 探针直接度量最终 link iAABB 体积，自动包含所有因素
- 代价仅增加约 8 倍完整 FK（7-DOF），每个深度仅执行一次，完全可接受

**与实际管线的关系**：
若实际管线使用 CritSample / Analytical / GCPC（比 iFK 更紧），
探针仍以 iFK 级别执行探测。这是合理的：
- iFK 探针的维度**排名**与更紧源的排名高度相关（底层 DH 链结构相同）
- 用更紧源做探针代价过高（CritSample = 边界枚举，GCPC = 全局规划），
  不适合做每个候选维度的评估
- 实际分裂后的 envelope 仍由完整管线计算，探针仅影响维度选择

### 2.3 首次触发节点的代表性

同深度不同节点的区间不同。我们用**第一个触发分裂的节点**的区间作为代表。

合理性：
- `find_free_box` 中，节点沿 seed 路径从根到叶延伸分裂，深度 $d$ 的第一个节点
  就是 seed 路径上的节点，其区间反映了当前查询的工作区域
- `pre_expand` 中，深度优先遍历左子树，第一个触发的是"全左"路径上的节点
- 实际测试中 ROUND_ROBIN 也对所有同深度节点使用相同维度，本策略同理

### 2.4 `node_intervals()` 兼容性

`node_intervals()` 重建区间时的维度回溯使用以下优先级：
1. `node_split_dim_[p]` — 若已记录（`split_leaf` 内始终设置）
2. `split_dims_[depth % len]` — ROUND_ROBIN 回退
3. `depth % n_dims` — 最终回退

BEST_TIGHTEN 在 `split_leaf` 中正常设置 `node_split_dim_[node]`，因此优先级 1
始终生效。`split_dims_` 的索引方式变化不影响 `node_intervals()`。

---

## 3. 代码修改清单

### 3.1 `include/sbf/core/config.h` — 新增枚举值

```diff
 enum class SplitOrder : uint8_t {
     ROUND_ROBIN     = 0,
-    WIDEST_FIRST    = 2
+    WIDEST_FIRST    = 2,
+    BEST_TIGHTEN    = 3   // 启发式：按深度选择三角收紧收益最大的维度
 };
```

**Enum Safety (spec §4)**：
- `BEST_TIGHTEN = 3`，不改动已有枚举值（0、2），无序列化兼容性问题
- 磁盘缓存中不存储 `SplitOrder`（仅 `EndpointSource` 和 `EnvelopeType`），无需 name-based cross-check

### 3.2 `include/sbf/forest/lect.h` — 新增方法声明

公共方法：
```cpp
/// Set split order strategy (ROUND_ROBIN / WIDEST_FIRST / BEST_TIGHTEN).
void set_split_order(SplitOrder so);

/// Get current split order.
SplitOrder split_order() const { return split_order_; }
```

私有方法：
```cpp
/// iFK 探针维度选择：对每个候选维度执行增量FK+提取link iAABB，
/// 选择使 max(vol_left, vol_right) 最小的维度 (minimax 策略)。
/// 当所有维度宽度≤0时，回退到 widest-first。
int pick_best_tighten_dim(const FKState& parent_fk,
                          const std::vector<Interval>& intervals) const;
```

### 3.3 `src/forest/lect.cpp` — 实现

#### 3.3.1 `init_split_dims()` (L42–55)

```diff
 void LECT::init_split_dims(int n_joints) {
     split_dims_.clear();
     switch (split_order_) {
     case SplitOrder::ROUND_ROBIN:
         split_dims_.resize(n_joints);
         std::iota(split_dims_.begin(), split_dims_.end(), 0);
         break;
     case SplitOrder::WIDEST_FIRST:
         break;
+    case SplitOrder::BEST_TIGHTEN:
+        // 预分配，按深度索引，懒惰填充(-1 = 未决定)
+        split_dims_.assign(128, -1);
+        break;
     }
 }
```

#### 3.3.2 新增 `set_split_order()` + `pick_best_tighten_dim()`

```cpp
void LECT::set_split_order(SplitOrder so) {
    split_order_ = so;
    init_split_dims(robot_->n_joints());
}

int LECT::pick_best_tighten_dim(
    const FKState& parent_fk,
    const std::vector<Interval>& intervals) const
{
    const int nj = robot_->n_joints();
    const int nact = robot_->n_active_links();
    const int* alm = robot_->active_link_map();
    const double* radii = robot_->active_link_radii();

    int best_dim = 0;
    double best_metric = std::numeric_limits<double>::infinity();

    // Fallback: widest-first (used if all dims have width ≤ 0)
    {
        double max_w = intervals[0].width();
        for (int d = 1; d < nj; ++d) {
            double w = intervals[d].width();
            if (w > max_w) { max_w = w; best_dim = d; }
        }
    }

    // Stack-allocated AABB buffers (max 16 active links × 6 floats)
    float left_aabbs[16 * 6];
    float right_aabbs[16 * 6];

    for (int d = 0; d < nj; ++d) {
        if (intervals[d].width() <= 0.0) continue;

        double mid_d = intervals[d].center();

        // ── Probe left child ────────────────────────────────────
        auto left_ivs = intervals;
        left_ivs[d].hi = mid_d;
        FKState left_fk = compute_fk_incremental(
            parent_fk, *robot_, left_ivs, d);
        extract_link_aabbs(left_fk, alm, nact, left_aabbs, radii);

        // ── Probe right child ───────────────────────────────────
        auto right_ivs = intervals;
        right_ivs[d].lo = mid_d;
        FKState right_fk = compute_fk_incremental(
            parent_fk, *robot_, right_ivs, d);
        extract_link_aabbs(right_fk, alm, nact, right_aabbs, radii);

        // ── Compute per-link volumes and aggregate ──────────────
        double vol_left = 0.0, vol_right = 0.0;
        for (int i = 0; i < nact; ++i) {
            const float* L = left_aabbs + i * 6;
            vol_left += static_cast<double>(L[3] - L[0]) *
                        static_cast<double>(L[4] - L[1]) *
                        static_cast<double>(L[5] - L[2]);
            const float* R = right_aabbs + i * 6;
            vol_right += static_cast<double>(R[3] - R[0]) *
                         static_cast<double>(R[4] - R[1]) *
                         static_cast<double>(R[5] - R[2]);
        }

        double metric = std::max(vol_left, vol_right);
        if (metric < best_metric) {
            best_metric = metric;
            best_dim = d;
        }
    }

    return best_dim;
}
```

#### 3.3.3 `split_leaf()` (L142–155) — 维度选择逻辑

当前代码 (L144-154):
```cpp
int dim;
if (split_order_ == SplitOrder::WIDEST_FIRST) {
    dim = 0;
    double max_w = parent_intervals[0].width();
    for (int d = 1; d < nj; ++d) { ... }
} else {
    dim = split_dims_[parent_depth % static_cast<int>(split_dims_.size())];
}
```

修改为三分支：
```cpp
int dim;
if (split_order_ == SplitOrder::WIDEST_FIRST) {
    dim = 0;
    double max_w = parent_intervals[0].width();
    for (int d = 1; d < nj; ++d) {
        double w = parent_intervals[d].width();
        if (w > max_w) { max_w = w; dim = d; }
    }
} else if (split_order_ == SplitOrder::BEST_TIGHTEN) {
    // 按深度懒惰选维：同深度复用同一维度
    if (parent_depth >= static_cast<int>(split_dims_.size()))
        split_dims_.resize(static_cast<size_t>(parent_depth) + 1, -1);
    if (split_dims_[parent_depth] < 0)
        split_dims_[parent_depth] = pick_best_tighten_dim(parent_fk, parent_intervals);
    dim = split_dims_[parent_depth];
} else {
    dim = split_dims_[parent_depth % static_cast<int>(split_dims_.size())];
}
```

注意：`split_leaf` 签名已有 `parent_fk`，无需额外改动。

#### 3.3.4 `find_free_box()` 叶节点分裂前维度选择 (L435–444)

同 `split_leaf` 相同的三分支逻辑。此处的 `dim` 仅用于 `min_edge` 预检查，
`split_leaf` 被调用时内部会再次确定维度。
但两处必须一致，否则 `min_edge` 检查的维度可能与实际分裂维度不符。

`find_free_box` 中 `fk` 变量在下降循环中通过 `compute_fk_incremental` 维护，
始终对应当前 `intervals` 的 FK 状态，可直接传入 `pick_best_tighten_dim`。

```cpp
if (split_order_ == SplitOrder::WIDEST_FIRST) {
    // ... widest-first logic ...
} else if (split_order_ == SplitOrder::BEST_TIGHTEN) {
    int depth = store_.depth(cur);
    if (depth >= static_cast<int>(split_dims_.size()))
        split_dims_.resize(static_cast<size_t>(depth) + 1, -1);
    if (split_dims_[depth] < 0)
        split_dims_[depth] = pick_best_tighten_dim(fk, intervals);
    dim = split_dims_[depth];
} else {
    dim = split_dims_[d % static_cast<int>(split_dims_.size())];
}
```

#### 3.3.5 `pre_expand_recursive()` 叶节点分裂前维度选择 (L737–748)

同 `split_leaf` 相同的三分支逻辑。`pre_expand_recursive` 签名已有
`const FKState& fk`，可直接传入：

```cpp
if (split_order_ == SplitOrder::WIDEST_FIRST) {
    // ... widest-first logic ...
} else if (split_order_ == SplitOrder::BEST_TIGHTEN) {
    if (d >= static_cast<int>(split_dims_.size()))
        split_dims_.resize(static_cast<size_t>(d) + 1, -1);
    if (split_dims_[d] < 0)
        split_dims_[d] = pick_best_tighten_dim(fk, intervals);
    dim = split_dims_[d];
} else {
    dim = split_dims_[d % static_cast<int>(split_dims_.size())];
}
```

#### 3.3.6 `snapshot()` (L520–530)

无需修改。`copy.split_dims_ = split_dims_` 已正确复制包括懒惰填充的维度。

#### 3.3.7 `load()` (L596)

`load()` 中硬编码 `lect.split_order_ = SplitOrder::WIDEST_FIRST`。
可选改为从 metadata 恢复，但不影响正确性（加载后仅用 `node_split_dim_` 回溯）。
本次不修改。

### 3.4 `node_intervals()` 兼容性分析 (L234–270)

**无需修改**。

`node_intervals()` 使用优先级：
1. `node_split_dim_[p]` ← `split_leaf` 中始终设置
2. `split_dims_[depth % len]` ← 回退路径（BEST_TIGHTEN 不走此路径）
3. `depth % n_dims` ← 最终回退

BEST_TIGHTEN 下所有节点通过 `split_leaf` 分裂，`node_split_dim_` 一定有值，
因此优先级 1 始终命中。

### 3.5 `find_free_box()` 下降阶段维度推断 (L460–469)

**无需修改**。

下降阶段（已经分裂过的节点）使用 `node_split_dim_[cur]`（优先级 1），
不依赖 `split_dims_` 的索引方式。

---

## 4. 测试计划

### 4.1 新增测试 `test_lect_best_tighten()`

文件：`tests/test_ifk_pipeline.cpp`

```cpp
static void test_lect_best_tighten() {
    SECTION("LECT BEST_TIGHTEN");

    Robot robot = make_iiwa14();
    auto pipeline = envelope::PipelineConfig::fast();
    forest::LECT lect(robot, pipeline, 64);
    lect.set_split_order(SplitOrder::BEST_TIGHTEN);

    // 1. 基本结构正确性 —— 分裂后子节点有效
    lect.expand_leaf(0);
    CHECK(lect.n_nodes() == 3, "3 nodes after root split");
    CHECK(lect.has_iaabb(lect.left(0)), "left has iAABB");
    CHECK(lect.has_iaabb(lect.right(0)), "right has iAABB");

    // 2. 恰好 1 个维度被二分
    auto root_ivs = lect.node_intervals(0);
    auto left_ivs = lect.node_intervals(lect.left(0));
    int split_count = 0;
    for (int d = 0; d < 7; ++d)
        if (left_ivs[d].width() < root_ivs[d].width() - 1e-10)
            ++split_count;
    CHECK(split_count == 1, "exactly 1 dimension halved");

    // 3. 同深度一致性 —— 两侧子节点再分裂应选相同维度
    int left_idx = lect.left(0);
    int right_idx = lect.right(0);
    lect.expand_leaf(left_idx);
    lect.expand_leaf(right_idx);
    //  depth=1 的 4 个子节点应选相同维度
    auto ll_ivs = lect.node_intervals(lect.left(left_idx));
    auto rl_ivs = lect.node_intervals(lect.left(right_idx));
    int dim_ll = -1, dim_rl = -1;
    auto left1_ivs = lect.node_intervals(left_idx);
    auto right1_ivs = lect.node_intervals(right_idx);
    for (int d = 0; d < 7; ++d) {
        if (ll_ivs[d].width() < left1_ivs[d].width() - 1e-10) dim_ll = d;
        if (rl_ivs[d].width() < right1_ivs[d].width() - 1e-10) dim_rl = d;
    }
    CHECK(dim_ll == dim_rl, "same depth → same split dimension");

    // 4. 更深层分裂不崩溃
    for (int i = 0; i < 10; ++i) {
        int n = lect.n_nodes();
        for (int j = 0; j < n; ++j) {
            if (lect.is_leaf(j))
                lect.expand_leaf(j);
        }
    }
    CHECK(lect.n_nodes() > 3, "deep expansion succeeds");

    // 5. iAABB 总体积对比 —— BEST_TIGHTEN 不应严格劣于 WIDEST_FIRST
    //    (此处只验证不崩溃，定量对比由实验完成)

    std::printf("  LECT BEST_TIGHTEN: OK\n");
}
```

### 4.2 现有测试不受影响

- `test_lect()` 使用默认 `WIDEST_FIRST`，行为不变
- `test_refine_aabb()` 使用默认策略，行为不变
- `test_lect_v3_compat()` 若存在，使用固定策略，行为不变

---

## 5. 文档同步 (Spec §1)

### 5.1 `doc/CHANGE_LOG_CN.md`

追加：

```markdown
## 2026-03-24 — LECT 新增 BEST_TIGHTEN 维度选择策略

- 修改文件 / 模块：
  - `include/sbf/core/config.h` — `SplitOrder` 枚举新增 `BEST_TIGHTEN = 3`
  - `include/sbf/forest/lect.h` — 新增 `set_split_order()`, `split_order()`,
    `pick_best_tighten_dim()` 方法
  - `src/forest/lect.cpp` — `init_split_dims()`, `split_leaf()`, `find_free_box()`,
    `pre_expand_recursive()` 中新增 BEST_TIGHTEN 分支；实现 `pick_best_tighten_dim()`
  - `tests/test_ifk_pipeline.cpp` — 新增 `test_lect_best_tighten()` 测试
- 实现内容：
  - iFK 探针维度选择策略：对每个候选维度执行增量 FK + 提取 per-link iAABB，
    选择使 $\max(\text{vol}_L, \text{vol}_R)$ 最小的维度 (minimax 策略)
  - **按深度懒惰选维**：某个深度首次 `split_leaf` 时触发 iFK 探针，同深度所有节点复用同一维度
  - 当所有维度宽度≤0时 fallback 到 widest-first
- 影响：
  - API：新增 `SplitOrder::BEST_TIGHTEN` 枚举值和 `set_split_order()` 公共方法
  - 缓存：磁盘缓存不存储 `SplitOrder`，无兼容性影响
  - 行为语义：默认 `WIDEST_FIRST` 不变，仅显式设置 `BEST_TIGHTEN` 时生效
```

### 5.2 `docs/API_REFERENCE_CN.md`

在 `SplitOrder` 枚举文档表格中追加行：

```
| `BEST_TIGHTEN` | 3 | 按深度懒惰选择使 link iAABB 体积最小的维度（iFK 探针 minimax）；同深度所有节点复用相同维度 |
```

在 `LECT` 类文档中添加：

```
| `set_split_order(so)` | void | 设置维度选择策略 |
| `split_order()` | SplitOrder | 获取当前维度选择策略 |
```

### 5.3 论文同步 (Spec §1)

当前论文 §IV.1 "Lazy splitting" 段仅描述 "splitting along dimension $d$"，
未指定维度选择策略。因此：

- **`paper/root.tex`** §IV.1 (L955–958)：在 "Lazy splitting" 段落末追加一句：
  > A per-depth heuristic selects the dimension whose bisection
  > minimises the minimax link-envelope volume:
  > for each candidate dimension $d$, incremental FK is computed
  > for both child intervals, and
  > $\text{metric}(d) = \max\bigl(\sum_i V_i^L,\; \sum_i V_i^R\bigr)$
  > is evaluated; the dimension with the smallest metric is chosen.

- **`paper/root_cn.tex`** §IV.1 (L768 附近)：追加对应中文：
  > 按深度采用 iFK 探针选择分裂维度：对每个候选维度 $d$，分别
  > 构造左右子区间并执行增量 FK，提取各连杆 iAABB 体积，选择使
  > $\text{metric}(d) = \max\bigl(\sum_i V_i^L,\; \sum_i V_i^R\bigr)$
  > 最小的维度。

---

## 6. 验证步骤

1. **编译**：`cmake --build . --config Release` — 零错零警告
2. **测试**：运行 `test_ifk_pipeline`，全部测试通过（含新 `test_lect_best_tighten`）
3. **可选实验**：在 `exp_split_order.cpp` 中新增 BEST_TIGHTEN 配置行，
   与 WIDEST_FIRST / ROUND_ROBIN 对比 FFB 性能（节点数、深度、iAABB 体积）

---

## 7. 设计决策记录

| 决策 | 选项 | 选择 | 理由 |
|------|------|------|------|
| 触发时机 | 每次 split_leaf / **每个深度首次** | 每个深度首次 | 同深度一致性、开销最小化、缓存友好 |
| 评分指标 | 三角宽度差 / **iFK probe 体积度量** | iFK probe | 直接度量最终 link iAABB 体积，自动包含 DH 参数和链传播效应 |
| 度量策略 | sum / **minimax** | minimax | 确保较差子节点尽量紧凑，直接影响 FFB 提前返回 |
| Fallback | 报错 / **widest-first** | widest-first | 所有维度宽度≤0时保证不退化 |
| `split_dims_` 索引 | `depth % len` / **`depth` 直接索引** | 直接索引 | BEST_TIGHTEN 不循环，每个深度独立决定 |
| API 暴露 | 构造函数参数 / **setter** | setter | 后向兼容，不改动现有构造函数签名 |

---

## 8. 风险与限制

1. **代表性偏差**：首次触发节点的区间不一定代表同深度所有节点的最优选择。
   极端情况下（某些节点的区间分布与首个节点差异大），个别节点的选维可能非最优。
   但与 ROUND_ROBIN 完全忽略区间信息相比仍是严格改进。

2. **iFK 探针与实际管线的精度差**：探针使用 iFK 级别（最宽松的 Cartesian 包围），
   而实际管线可能使用更紧的源（CritSample、GCPC）。维度排名在不同精度下
   可能略有差异，但底层 DH 链结构决定排名高度相关。

3. **与 warm-start 缓存的交互**：`pre_expand` 会按深度优先填充 `split_dims_`，
   后续 `find_free_box` 使用相同的已填充值。两者一致。

4. **序列化**：`split_dims_` 不持久化到磁盘。`load()` 后重置为默认策略，
   依赖 `node_split_dim_` 重建。这与现有行为一致。

5. **栈分配限制**：`pick_best_tighten_dim` 中使用 `float[16*6]` 栈分配，
   限制最多 16 条 active link。当前 v4 中 iiwa14 = 7 条、Panda = 7 条，
   充足。若未来支持更多连杆，需改为 `std::vector` 或 `alloca`。

---

## 附录 A：代码修改位置汇总

| 文件 | 函数/区域 | 行号 (当前) | 修改类型 |
|------|-----------|-------------|----------|
| `include/sbf/core/config.h` | `SplitOrder` 枚举 | L37–40 | 新增值 |
| `include/sbf/forest/lect.h` | public 方法声明 | ~L67 | 新增 2 方法 |
| `include/sbf/forest/lect.h` | private 方法声明 | ~L180 | 新增 1 方法 |
| `src/forest/lect.cpp` | `init_split_dims()` | L42–55 | 新增 case |
| `src/forest/lect.cpp` | 新函数 | (新增) | `set_split_order()` + `pick_best_tighten_dim()` |
| `src/forest/lect.cpp` | `split_leaf()` | L144–154 | 三分支改写 |
| `src/forest/lect.cpp` | `find_free_box()` 叶节点 | L435–444 | 三分支改写 |
| `src/forest/lect.cpp` | `pre_expand_recursive()` | L737–748 | 三分支改写 |
| `tests/test_ifk_pipeline.cpp` | main + 新测试 | (新增) | `test_lect_best_tighten()` |
| `doc/CHANGE_LOG_CN.md` | (追加) | 文件末尾 | 变更记录 |
| `docs/API_REFERENCE_CN.md` | SplitOrder 表 + LECT 表 | ~L788 | 新增行 |
| `paper/root.tex` | §IV.1 | L955–958 | 追加描述 |
| `paper/root_cn.tex` | §IV.1 | L768 | 追加描述 |
