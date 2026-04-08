# iFK Pipeline 迁移与自适应冻结适配计划

> 创建时间: 2026-03-19
> 模块: `robot/` + `envelope/endpoint_source` + `forest/lect`
> 前置依赖: PLAN_ADAPTIVE_FREEZE_DEPTH.md（设计阶段→实现阶段）
> 参考: MIGRATION_ENDPOINT_IAABB.md Phase A-B, G（本文档为其细化执行版本）
> 状态: **可执行**

---

## 0. 目标

将 v3 的 iFK（Interval Forward Kinematics）流水线完整迁移至 v4，并适配：

1. **EndpointIAABB 命名系统**：`endpoint_aabb` → `endpoint_iaabb`
2. **自适应冻结深度**：per-node `effective_freeze_depth` 替代全局 `freeze_depth_`
3. **冻结优先切分**：`SplitOrder::FREEZE_PRIORITY` 的 `init_split_dims()` 实现
4. **iFK→LECT 集成**：`compute_envelope()` + `split_leaf()` + `reconstruct_and_derive_link_iaabbs()` 完整实现

完成后，v4 将具备 **iFK 路径的端到端工作能力**：
```
Robot(DH) → compute_fk_full/incremental → fk_to_endpoints → endpoint_iaabbs
         → reconstruct (freeze) → extract_link_iaabbs → LECT compute_envelope
         → split_leaf (Case A/B) → find_free_box → FFBResult
```

其余三条 EndpointSource 路径（CritSample / Analytical / GCPC）由后续 Phase D-F 迁移。

---

## 1. 依赖链与执行顺序

```
┌─────────────────────────────────────────────┐
│  S1: Robot 类迁移 (robot.h / robot.cpp)     │  ← 基础层，所有模块依赖
└──────┬──────────────────────────────────────┘
       │
  ┌────▼─────────────────────────────────┐
  │  S2: interval_math 完整迁移          │  ← I_sin/I_cos 已有临时实现
  │  (interval_math.h / interval_math.cpp)│     需要: build_dh_joint, imat_mul_dh,
  └──────┬───────────────────────────────┘     outward_lo/hi
         │
    ┌────▼───────────────────────────────────┐
    │  S3: interval_fk 迁移                  │  ← FKState 完整字段 +
    │  (interval_fk.h / interval_fk.cpp)     │     compute_fk_full/incremental +
    └──────┬─────────────────────────────────┘     extract_link_aabbs
           │
      ┌────▼────────────────────────────────────┐
      │  S4: envelope_derive 迁移               │  ← derive_aabb_subdivided
      │  (envelope_derive.h / envelope_derive.cpp)│     (extract_link_iaabbs 依赖)
      └──────┬──────────────────────────────────┘
             │
        ┌────▼──────────────────────────────────────────┐
        │  S5: endpoint_source iFK 路径实现              │
        │  (endpoint_source.cpp)                        │
        │  fk_to_endpoints, compute_endpoint_iaabb(IFK) │
        │  compute_endpoint_iaabb_incremental(IFK)      │
        │  extract_link_iaabbs                          │
        └──────┬────────────────────────────────────────┘
               │
          ┌────▼──────────────────────────────────────────────────────┐
          │  S6: LECT iFK 集成 + 自适应冻结                           │
          │  init_root, init_split_dims, compute_envelope             │
          │  split_leaf (Case A 继承 / Case B 增量 FK)                │
          │  reconstruct_and_derive_link_iaabbs（per-node fd）        │
          │  NodeStore 核心 SoA 数组                                   │
          └──────┬────────────────────────────────────────────────────┘
                 │
            ┌────▼──────────────────────────────┐
            │  S7: 编译验证 + 单元测试            │
            └───────────────────────────────────┘
```

---

## 2. S1: Robot 类迁移

### 2.1 当前 v4 状态

- `include/sbf/robot/robot.h`：53 行 stub，用 `struct Impl;` pimpl，仅声明少量接口
- `src/robot/robot.cpp`：33 行 stub，所有方法返回 0/empty

### 2.2 迁移目标

从 v3 `robot.h`（146 行）/ `robot.cpp`（254 行）完整替换。

### 2.3 改动要点

| 项目 | v3 | v4 |
|------|----|----|
| include 路径 | `sbf/common/types.h` | `sbf/core/types.h` |
| pimpl | 无（直接成员） | **放弃 pimpl，改用 v3 直接成员** |
| namespace | `sbf::` | 保持 `sbf::`（v4 `using robot::Robot` 别名已存在） |
| 新增接口 | — | `n_endpoints()` 便利方法 = `n_joints() + has_tool()` |
| 版本标注 | "SafeBoxForest v2" | "SafeBoxForest v4" |
| JSON 加载 | `from_json(path)` | 保留同签名 |

### 2.4 需要迁移的 v3 类型

v3 `robot.h` 定义了以下在 v4 中缺失或不完整的类型：

| 类型 | v3 存在 | v4 当前状态 | 操作 |
|------|---------|-----------|------|
| `DHParam` | ✓ (L15-21) | ✓ (已在 v4 robot.h 声明) | 保留 v4 版本 |
| `EESphere` | ✓ (L24-28) | ✗ | 迁移 |
| `EEGroupKey` | ✓ (L31-34) | ✗ | 迁移 |
| `EEGroup` | ✓ (L37-41) | ✗ | 迁移 |
| `Robot` class | ✓ (L43-146) | stub | 完整替换 |

### 2.5 Robot 私有成员完整列表（从 v3 迁移）

```cpp
// 核心数据
std::string name_;
std::vector<DHParam> dh_params_;
JointLimits limits_;
std::optional<DHParam> tool_frame_;
std::vector<double> link_radii_;

// 耦合约束（CritSample/Analytical 使用，iFK 不需要但保留）
std::vector<std::pair<int,int>> coupled_pairs_;
std::vector<std::tuple<int,int,int>> coupled_triples_;

// EE 球体（碰撞检测扩展）
std::vector<EESphere> ee_spheres_;
int ee_spheres_frame_ = 0;
std::vector<EEGroup> ee_groups_;

// 预计算缓存
int n_joints_ = 0, n_links_ = 0, n_tf_ = 0, n_active_links_ = 0;
std::vector<double> dh_alpha_, dh_a_, dh_d_, dh_theta_;
std::vector<int> dh_joint_type_;
std::vector<int> active_link_map_;
std::vector<double> active_link_radii_;

void pack_arrays();  // 预打包 DH 数组 + 活跃连杆映射
```

### 2.6 关键方法清单

| 方法 | 行数 | iFK 必需 | 说明 |
|------|------|---------|------|
| `from_json(path)` | ~100 | ✓ | JSON 加载 → 构造 Robot |
| `Robot(...)` 构造函数 | ~20 | ✓ | 调用 `pack_arrays()` |
| `pack_arrays()` | ~50 | ✓ | 构建 active_link_map, 打包 DH 连续数组 |
| `fk_link_positions(q)` | ~20 | ✗ | 标量 FK（Eigen），可延迟 |
| `fk_transforms(q)` | ~20 | ✗ | 标量 FK（Eigen），可延迟 |
| `fingerprint()` | ~10 | ✗ | 缓存兼容指纹 |

**最小迁移**：构造函数 + `pack_arrays()` + `from_json()` + 全部访问器。
标量 FK 方法可用 stub 占位（`fk.h` / `fk_scalar.cpp` 在 Phase A 一起迁移更好，
但不是 iFK 路径的阻塞项）。

### 2.7 文件操作

```
1. 替换 include/sbf/robot/robot.h    ← v3 robot.h 适配 v4 include 路径
2. 替换 src/robot/robot.cpp          ← v3 robot.cpp 适配 v4 include 路径
3. 编译验证
```

**预估：~400 行，0.5h**

---

## 3. S2: interval_math 完整迁移

### 3.1 当前 v4 状态

- `interval_math.h`：33 行，仅声明 `I_sin` / `I_cos`
- `interval_math.cpp`：37 行，`I_sin`/`I_cos` 有简化实现（缺少 outward rounding）

**缺失项**：

| 函数 | v3 行号 | 用途 |
|------|---------|------|
| `outward_lo(v)` / `outward_hi(v)` | L17-22 | ULP 外向取整（保守区间） |
| `build_dh_joint(...)` | L74-116 | 构造 4×4 DH 区间矩阵 |
| `imat_mul_dh(...)` | L118-153 | 4×4 区间矩阵乘法 |
| `imat_identity(lo, hi)` | L43-48 | 初始化单位矩阵 |
| `imat_copy(src, dst)` | L50-55 | 复制区间矩阵 |

### 3.2 迁移操作

**完整替换**两个文件为 v3 版本。关键改动：

| 项目 | v3 | v4 |
|------|----|----|
| header include | `sbf/common/types.h` | `sbf/core/types.h` |
| namespace | `sbf::` | `sbf::`（不变） |
| `I_sin` / `I_cos` | outward rounded by 1 ULP | 替换当前简化版 |
| inline 函数 | `outward_lo/hi`, `imat_identity`, `imat_copy` 在 `.h` | 保持 inline |

### 3.3 outward rounding 的重要性

v4 当前的 `I_sin` 缺少 outward rounding — 这意味着返回的区间**不是 provably conservative**：

```cpp
// v3（正确）：I_sin 返回值经 outward_lo / outward_hi 处理
slo = outward_lo(slo);  // 向 −∞ 偏移 1 ULP
shi = outward_hi(shi);  // 向 +∞ 偏移 1 ULP

// v4（缺失）：没有 ULP 偏移 → 浮点舍入可能导致非保守结果
```

这在碰撞检测中会导致 **false negative**（漏检碰撞），必须修复。

### 3.4 文件操作

```
1. 替换 include/sbf/robot/interval_math.h  ← v3 完整版（~60 行）
2. 替换 src/robot/interval_math.cpp        ← v3 完整版（~153 行）
3. 编译验证（interval_trig.h 的 inline 函数调用 I_sin/I_cos，
   签名不变所以 interval_trig.h 无需修改）
```

**预估：~213 行，0.3h**

---

## 4. S3: interval_fk 迁移

### 4.1 当前 v4 状态

- `interval_fk.h`：35 行 stub，`FKState` 仅有 `bool valid`
- `interval_fk.cpp`：11 行，空 namespace

### 4.2 迁移目标

完整迁移 v3 `interval_fk.h`（67 行）和 `interval_fk.cpp`（255 行）。

### 4.3 FKState 完整字段（v3 → v4）

```cpp
struct FKState {
    // MAX_TF = 34, MAX_JOINTS = 32 (from types.h)
    double prefix_lo[MAX_TF][16];    // 累积变换链 lower bound
    double prefix_hi[MAX_TF][16];    // 累积变换链 upper bound
    double joints_lo[MAX_JOINTS][16]; // 单关节 DH 矩阵 lower bound
    double joints_hi[MAX_JOINTS][16]; // 单关节 DH 矩阵 upper bound
    int n_tf  = 0;    // n_joints + 1 + has_tool
    int n_jm  = 0;    // n_joints + has_tool
    bool valid = false;
};
```

**注意**：`MAX_TF` 和 `MAX_JOINTS` 需要在 `types.h` 中定义。

### 4.4 需要检查 v4 types.h 是否有 MAX_TF / MAX_JOINTS

v3 `types.h` 定义：
```cpp
static constexpr int MAX_JOINTS = 32;
static constexpr int MAX_TF     = 34;  // MAX_JOINTS + 2 (tool)
```

若 v4 `types.h` 缺少这些常量，需要添加。

### 4.5 迁移函数清单

| 函数 | v3 行号 | 签名 | iFK 必需 |
|------|---------|------|---------|
| `build_joint_interval` | L8-34 | `(Robot, joint_idx, Interval, A_lo, A_hi)` | ✓ |
| `compute_fk_full` | L38-64 | `(Robot, intervals) → FKState` | ✓ |
| `compute_fk_incremental` | L66-100 | `(FKState, Robot, intervals, dim) → FKState` | ✓ |
| `extract_link_aabbs` | L102-126 | `(FKState, alm, n_act, out, radii)` | ✓ |
| `extract_ee_sphere_aabbs` | L128-175 | `(FKState, spheres, n, fi, out)` | ✗ (EE 扩展) |
| `extract_ee_group_aabbs` | L176-229 | `(FKState, groups, n, fi, out)` | ✗ (EE 扩展) |
| `compute_link_aabbs` | L130-135 | `(Robot, intervals, out)` | ✗ (便捷) |
| `compute_all_aabbs` | L137-229 | `(Robot, intervals, link_out, ee_out)` | ✗ (便捷) |

**最小迁移**：前 4 个函数。EE 相关和便捷函数一并迁移（代码量不大）。

### 4.6 v4 命名适配

| v3 名称 | v4 名称 | 说明 |
|---------|---------|------|
| `compute_fk_full` | `compute_fk_full` | 不变（iFK 内部名称） |
| `compute_fk_incremental` | `compute_fk_incremental` | 不变 |
| `extract_link_aabbs` | `extract_link_aabbs` | 不变（直接从 FKState 提取） |
| `FKState` | `FKState` | 不变 |
| `build_joint_interval` | `build_joint_interval` | 不变 |

**注意**：`extract_link_aabbs` 扩展了 iAABB 以 radius，但数据直接来自 FKState
而非 endpoint_iaabbs，所以不需要 i-前缀。i-前缀只用于通过 endpoint 中间格式派生
的 per-link iAABB（见 `extract_link_iaabbs`）。

### 4.7 文件操作

```
1. 替换 include/sbf/robot/interval_fk.h   ← v3 完整版（~67 行）
   - include 路径: sbf/common/types.h → sbf/core/types.h
   - 添加 'using robot::FKState;' 在 sbf:: 命名空间（v3 兼容）
2. 替换 src/robot/interval_fk.cpp         ← v3 完整版（~255 行）
   - include 路径适配
3. 确认 types.h 有 MAX_TF / MAX_JOINTS
4. 编译验证
```

**预估：~322 行，0.5h**

---

## 5. S4: envelope_derive 迁移

### 5.1 当前 v4 状态

文件不存在。`extract_link_iaabbs()` 在 `endpoint_source.cpp` 中是空 stub。

### 5.2 迁移目标

从 v3 `envelope_derive.h`（108 行）和 `envelope_derive.cpp`（216 行）新建文件。

### 5.3 需要迁移的函数

| 函数 | 用途 | iFK 路径必需 |
|------|------|-------------|
| `derive_aabb` | per-link AABB from frames | ✗ (直接路径用 extract_link_aabbs) |
| `derive_aabb_subdivided` | 子段 AABB (插值 + radius) | ✓ (extract_link_iaabbs 使用) |
| `adaptive_subdivision_count` | 自适应子段数 | ✗ (Hull Grid 使用) |
| `derive_grid` | 栅格化 | ✗ (Hull Grid 使用) |

**最小迁移**：`derive_aabb_subdivided` 是 `extract_link_iaabbs` 的唯一依赖。
但全部迁移代码量不大（~324 行），且 Hull Grid 后续需要，建议**全部迁移**。

### 5.4 v4 改动

| 项目 | v3 | v4 |
|------|----|----|
| include 路径 | `sbf/common/types.h` | `sbf/core/types.h` |
| 版本标注 | "SafeBoxForest v3" | "SafeBoxForest v4" |
| 函数签名 | 不变 | 不变 |

### 5.5 文件操作

```
1. 新建 include/sbf/envelope/envelope_derive.h  ← v3 适配 v4
2. 新建 src/envelope/envelope_derive.cpp         ← v3 适配 v4
3. CMakeLists.txt: 添加 src/envelope/envelope_derive.cpp 到源文件列表
4. 编译验证
```

**预估：~324 行，0.3h**

---

## 6. S5: endpoint_source iFK 路径实现

### 6.1 当前 v4 状态

- `endpoint_source.h`：177 行，**声明完整**（所有函数签名已定义）
- `endpoint_source.cpp`：59 行，全部函数体为空 stub

### 6.2 实现目标

仅实现 iFK 路径（其余三条路径保持 stub + 注释）。

### 6.3 需要实现的函数

#### 6.3.1 `fk_to_endpoints`

从 v3 `frame_source.cpp` L27-37 直接迁移：

```cpp
void fk_to_endpoints(const FKState& fk, const Robot& robot,
                     std::vector<float>& out) {
    const int n_endpoints = robot.n_joints() + (robot.has_tool() ? 1 : 0);
    out.resize(n_endpoints * 6);
    for (int k = 0; k < n_endpoints; ++k) {
        int fi = k + 1;
        out[k*6+0] = float(fk.prefix_lo[fi][3]);   // x_lo
        out[k*6+1] = float(fk.prefix_lo[fi][7]);   // y_lo
        out[k*6+2] = float(fk.prefix_lo[fi][11]);   // z_lo
        out[k*6+3] = float(fk.prefix_hi[fi][3]);    // x_hi
        out[k*6+4] = float(fk.prefix_hi[fi][7]);    // y_hi
        out[k*6+5] = float(fk.prefix_hi[fi][11]);   // z_hi
    }
}
```

#### 6.3.2 `compute_endpoint_iaabb` (IFK 路径)

```cpp
EndpointIAABBResult compute_endpoint_iaabb(
    const EndpointSourceConfig& config,
    const Robot& robot,
    const std::vector<Interval>& intervals)
{
    EndpointIAABBResult result;
    result.n_active = robot.n_active_links();
    result.n_endpoints = robot.n_joints() + (robot.has_tool() ? 1 : 0);

    switch (config.method) {
    case EndpointSource::IFK: {
        result.fk_state = compute_fk_full(robot, intervals);
        fk_to_endpoints(result.fk_state, robot, result.endpoint_iaabbs);
        break;
    }
    case EndpointSource::CritSample:
    case EndpointSource::Analytical:
    case EndpointSource::GCPC:
        // TODO: Phase D-F 迁移后实现
        result.fk_state = compute_fk_full(robot, intervals);
        fk_to_endpoints(result.fk_state, robot, result.endpoint_iaabbs);
        break;
    }
    return result;
}
```

#### 6.3.3 `compute_endpoint_iaabb_incremental` (IFK 路径)

```cpp
EndpointIAABBResult compute_endpoint_iaabb_incremental(
    const EndpointSourceConfig& config,
    const FKState& parent_fk,
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int changed_dim)
{
    EndpointIAABBResult result;
    result.n_active = robot.n_active_links();
    result.n_endpoints = robot.n_joints() + (robot.has_tool() ? 1 : 0);

    // 增量 FK 总是可用（不论 method）
    result.fk_state = compute_fk_incremental(parent_fk, robot, intervals, changed_dim);

    switch (config.method) {
    case EndpointSource::IFK:
        fk_to_endpoints(result.fk_state, robot, result.endpoint_iaabbs);
        break;
    default:
        // TODO: 其余路径迁移后实现（暂回退到 IFK）
        fk_to_endpoints(result.fk_state, robot, result.endpoint_iaabbs);
        break;
    }
    return result;
}
```

#### 6.3.4 `extract_link_iaabbs`

从 v3 `extract_link_aabbs_from_endpoint` 迁移，引用 `derive_aabb_subdivided`：

```cpp
void extract_link_iaabbs(
    const EndpointIAABBResult& result,
    const Robot& robot,
    float* out_iaabb)
{
    if (result.endpoint_iaabbs.empty()) {
        if (result.has_fk_state()) {
            extract_link_aabbs(result.fk_state,
                               robot.active_link_map(),
                               robot.n_active_links(),
                               out_iaabb,
                               robot.active_link_radii());
        }
        return;
    }
    const int n_act = robot.n_active_links();
    const int* alm = robot.active_link_map();
    const double* lr = robot.active_link_radii();
    float base_pos[3] = {0.f, 0.f, 0.f};
    for (int ci = 0; ci < n_act; ++ci) {
        int parent_fi = (alm[ci] == 0) ? -1 : alm[ci] - 1;
        int link_fi = alm[ci];
        float r = lr ? static_cast<float>(lr[ci]) : 0.f;
        derive_aabb_subdivided(
            result.endpoint_iaabbs.data(), result.n_endpoints,
            parent_fi, link_fi,
            1, r, base_pos,
            out_iaabb + ci * 6);
    }
}
```

### 6.4 v4 头文件适配

`endpoint_source.h` 的声明已经完整正确（使用 `EndpointIAABBResult` 而非 v3 的
`EndpointAABBResult`，字段名 `endpoint_iaabbs` 而非 `endpoint_aabbs`）。

需要添加的 include：
```cpp
#include "sbf/envelope/envelope_derive.h"  // derive_aabb_subdivided
```

### 6.5 文件操作

```
1. 替换 src/envelope/endpoint_source.cpp  ← iFK 路径实现 + 其余路径 stub
2. endpoint_source.h 可能需要微调（添加 include envelope_derive）
3. 编译验证
```

**预估：~220 行，0.5h**

---

## 7. S6: LECT iFK 集成 + 自适应冻结

这是最核心的步骤——将 iFK pipeline 与 LECT 的 KD-tree 操作完整对接。

### 7.1 NodeStore SoA 核心数组

v4 `node_store.h` 当前只有 `freeze_depth_` 和 `source_quality_` 向量，
缺少 KD-tree 运作所需的核心数组。

**需要添加的 SoA 字段**：

```cpp
// ── KD-tree 结构 ──
std::vector<int> left_;         // 左子节点索引 (-1 = 叶节点)
std::vector<int> right_;        // 右子节点索引
std::vector<int> parent_;       // 父节点索引 (-1 = 根节点)
std::vector<int> depth_;        // 节点深度
std::vector<double> split_;     // 分割值

// ── per-node AABB 数据 ──
std::vector<float> link_iaabbs_; // [capacity × n_active × 6] flat
std::vector<uint8_t> has_aabb_;  // 是否有有效 AABB

// ── 占领标记 ──
std::vector<int>  forest_id_;   // 所属 forest (-1 = 未占领)
std::vector<bool> occupied_;    // 是否被占领

// ── 元数据 ──
int n_nodes_     = 0;
int n_dims_      = 0;
int n_active_    = 0;
int capacity_    = 0;
```

### 7.2 init_split_dims — 冻结优先切分

```cpp
void LECT::init_split_dims(int n_joints) {
    split_dims_.clear();
    
    switch (freeze_policy_.split_order) {
    case SplitOrder::FREEZE_PRIORITY: {
        // Phase I: 冻结维度缩窄
        // 每个冻结维度需要 ceil(log2(W0 / threshold)) 次二分
        const double W0 = /* 从 root_limits 计算最大关节宽度 */;
        const double thr = freeze_policy_.width_threshold;
        int n_splits_per = static_cast<int>(
            std::ceil(std::log2(W0 / thr))) + 1;  // +1 确保 < threshold
        
        for (int d = 0; d < max_freeze_depth_; ++d) {
            for (int s = 0; s < n_splits_per; ++s) {
                split_dims_.push_back(d);
            }
        }
        // Phase II: 非冻结维度 round-robin
        // 追加足够多轮使树深度覆盖 max_depth
        const int MAX_DEPTH = 40;
        int phase2_start = static_cast<int>(split_dims_.size());
        int remaining_dims = n_joints - max_freeze_depth_;
        if (remaining_dims > 0) {
            for (int rep = 0; rep < MAX_DEPTH; ++rep) {
                // 非冻结维度 round-robin
                for (int d = max_freeze_depth_; d < n_joints; ++d)
                    split_dims_.push_back(d);
                // 冻结维度也加入（它们会触发 Case A 继承）
                for (int d = 0; d < max_freeze_depth_; ++d)
                    split_dims_.push_back(d);
            }
        }
        break;
    }
    case SplitOrder::WIDEST_FIRST:
        // 动态策略：不需要预生成 split_dims_
        // split_leaf 中根据实际区间宽度选择最宽维度
        // 保留空的 split_dims_ 作为标志
        break;
            
    case SplitOrder::ROUND_ROBIN:
    default: {
        // v3 兼容：identity permutation round-robin
        for (int d = 0; d < n_joints; ++d)
            split_dims_.push_back(d);
        break;
    }
    }
}
```

### 7.3 init_root

```cpp
void LECT::init_root() {
    root_limits_ = robot_->joint_limits();
    
    // 初始化 NodeStore
    store_.init(robot_->n_joints(), robot_->n_active_links(), initial_cap);
    int root_idx = store_.alloc_node();  // = 0
    store_.set_depth(root_idx, 0);
    
    // 初始化 split_dims
    init_split_dims(robot_->n_joints());
    
    // 计算根节点的 iFK + endpoint iAABBs
    FKState root_fk = compute_fk_full(*robot_, root_limits_.limits);
    compute_envelope(root_idx, root_fk, root_limits_.limits);
}
```

### 7.4 compute_envelope — 自适应冻结深度版本

```cpp
void LECT::compute_envelope(int node_idx, const FKState& /*fk*/,
                            const std::vector<Interval>& intervals)
{
    // ── 缓存检查 ──
    if (store_.has_aabb(node_idx)) {
        uint8_t cached_q = store_.source_quality(node_idx);
        uint8_t needed_q = envelope::endpoint_source_quality(
            pipeline_config_.source.method);
        if (cached_q >= needed_q)
            return;
    }

    // ── 自适应冻结深度 ──
    int fd = effective_freeze_depth(intervals);  // ← 关键改动
    store_.set_freeze_depth(node_idx, static_cast<uint8_t>(fd));

    // ── Stage 1: 在 LOCAL FRAME 计算 endpoint iAABBs ──
    std::vector<Interval> local_intervals = intervals;
    for (int j = 0; j < fd; ++j)
        local_intervals[j] = {0.0, 0.0};

    auto ep = envelope::compute_endpoint_iaabb(
        pipeline_config_.source, *robot_, local_intervals);

    ensure_ep_capacity(node_idx + 1);
    ep_store_[node_idx] = ep.endpoint_iaabbs;

    // ── Stage 2: 重建笛卡尔 + 派生 link iAABBs ──
    reconstruct_and_derive_link_iaabbs(node_idx, intervals, fd);

    // ── 记录 source quality ──
    store_.set_source_quality(node_idx,
        envelope::endpoint_source_quality(pipeline_config_.source.method));
}
```

**vs. v3 的关键差异**：
1. `fd` 由 `effective_freeze_depth(intervals)` 动态计算，非全局 `freeze_depth_`
2. `fd` 存入 per-node `store_.freeze_depth_`
3. 分离了 `frame_store_` 持久化（当前 iFK 阶段不需要磁盘缓存）

### 7.5 reconstruct_and_derive_link_iaabbs — per-node fd

```cpp
void LECT::reconstruct_and_derive_link_iaabbs(
    int node_idx, const std::vector<Interval>& intervals, int fd)
{
    const auto& local_ep = ep_store_[node_idx];
    int n_ep = robot_->n_joints() + (robot_->has_tool() ? 1 : 0);

    if (fd == 0) {
        // 无冻结：ep_store 就是笛卡尔坐标，直接派生 link iAABBs
        envelope::EndpointIAABBResult cart_result;
        cart_result.endpoint_iaabbs = local_ep;  // 已是世界坐标
        cart_result.n_active = robot_->n_active_links();
        cart_result.n_endpoints = n_ep;

        store_.ensure_capacity(node_idx + 1);
        float* aabb = store_.link_iaabbs_mut(node_idx);
        envelope::extract_link_iaabbs(cart_result, *robot_, aabb);
        store_.set_has_aabb(node_idx, true);
        return;
    }

    // 构造冻结关节描述符（使用实际区间）
    std::vector<FrozenJointDesc> descs(
        frozen_descs_.begin(), frozen_descs_.begin() + fd);
    for (int j = 0; j < fd; ++j)
        descs[j].q = intervals[j];

    // 多层重建：local frame → Cartesian
    std::vector<float> cart_ep = reconstruct_cartesian_endpoints_multilevel(
        local_ep, n_ep, descs.data(), fd);

    // 派生 per-link iAABBs
    envelope::EndpointIAABBResult cart_result;
    cart_result.endpoint_iaabbs = std::move(cart_ep);
    cart_result.n_active = robot_->n_active_links();
    cart_result.n_endpoints = n_ep;

    store_.ensure_capacity(node_idx + 1);
    float* aabb = store_.link_iaabbs_mut(node_idx);
    envelope::extract_link_iaabbs(cart_result, *robot_, aabb);
    store_.set_has_aabb(node_idx, true);
}
```

### 7.6 split_leaf — Case A/B + 自适应冻结

```cpp
void LECT::split_leaf(int node_idx, const FKState& parent_fk,
                      const std::vector<Interval>& parent_intervals)
{
    int d = store_.depth(node_idx);
    
    // ── 确定切分维度 ──
    int dim;
    if (freeze_policy_.split_order == SplitOrder::WIDEST_FIRST) {
        // 动态最宽维度
        dim = 0;
        double max_w = parent_intervals[0].width();
        for (int j = 1; j < static_cast<int>(parent_intervals.size()); ++j) {
            if (parent_intervals[j].width() > max_w) {
                max_w = parent_intervals[j].width();
                dim = j;
            }
        }
    } else {
        // round-robin 或 freeze-priority：从预生成序列取
        dim = split_dims_[d % static_cast<int>(split_dims_.size())];
    }
    
    double mid = parent_intervals[dim].center();

    // 分配子节点
    int left_idx  = store_.alloc_node();
    int right_idx = store_.alloc_node();

    store_.set_left(node_idx, left_idx);
    store_.set_right(node_idx, right_idx);
    store_.set_parent(left_idx, node_idx);
    store_.set_parent(right_idx, node_idx);
    store_.set_depth(left_idx, d + 1);
    store_.set_depth(right_idx, d + 1);
    store_.set_split(node_idx, mid);

    // 计算子节点区间
    std::vector<Interval> left_ivs = parent_intervals;
    left_ivs[dim].hi = mid;
    std::vector<Interval> right_ivs = parent_intervals;
    right_ivs[dim].lo = mid;

    // ── 判断 Case A or Case B ──
    //
    //  Case A（冻结维度继承）的条件：
    //    1. dim < parent 的 freeze_depth（切分维度在冻结范围内）
    //    2. ep_store_ 有有效数据
    //
    //  自适应冻结版本：使用 parent 的 per-node fd，而非全局值
    int parent_fd = static_cast<int>(store_.freeze_depth(node_idx));
    bool has_ep = (node_idx < static_cast<int>(ep_store_.size()) &&
                   !ep_store_[node_idx].empty());

    if (dim < parent_fd && has_ep) {
        // ══ Case A: 冻结维度切分 → 继承 ep_store ══
        ensure_ep_capacity(std::max(left_idx, right_idx) + 1);
        ep_store_[left_idx]  = ep_store_[node_idx];    // 继承
        ep_store_[right_idx] = ep_store_[node_idx];

        // 子节点沿用 parent_fd（ep_store 在 parent_fd 坐标系下计算）
        store_.set_freeze_depth(left_idx, static_cast<uint8_t>(parent_fd));
        store_.set_freeze_depth(right_idx, static_cast<uint8_t>(parent_fd));

        // 仅重做 Stage 2 重建（用子节点的更窄区间）
        reconstruct_and_derive_link_iaabbs(left_idx, left_ivs, parent_fd);
        reconstruct_and_derive_link_iaabbs(right_idx, right_ivs, parent_fd);

        // 继承 source quality
        uint8_t parent_q = store_.source_quality(node_idx);
        store_.set_source_quality(left_idx, parent_q);
        store_.set_source_quality(right_idx, parent_q);
    } else {
        // ══ Case B: 非冻结维度 → 增量 FK + 完整 compute_envelope ══
        FKState left_fk  = compute_fk_incremental(parent_fk, *robot_, left_ivs, dim);
        FKState right_fk = compute_fk_incremental(parent_fk, *robot_, right_ivs, dim);

        // compute_envelope 会为每个子节点计算新的 effective_freeze_depth
        compute_envelope(left_idx, left_fk, left_ivs);
        compute_envelope(right_idx, right_fk, right_ivs);
    }

    // 上提 AABB 精度
    refine_aabb(node_idx);
}
```

### 7.7 refine_aabb

```cpp
void LECT::refine_aabb(int node_idx) {
    int li = store_.left(node_idx);
    int ri = store_.right(node_idx);
    if (li < 0 || ri < 0) return;
    if (!store_.has_aabb(li) || !store_.has_aabb(ri)) return;

    const int n_act = robot_->n_active_links();
    float* p = store_.link_iaabbs_mut(node_idx);
    const float* l = store_.link_iaabbs(li);
    const float* r = store_.link_iaabbs(ri);

    for (int c = 0; c < n_act; ++c) {
        for (int d = 0; d < 3; ++d) {
            // 父节点 = intersect(parent, union(left, right))
            float ulo = std::min(l[c*6+d], r[c*6+d]);
            float uhi = std::max(l[c*6+d+3], r[c*6+d+3]);
            p[c*6+d]   = std::max(p[c*6+d],   ulo);    // tighten lo
            p[c*6+d+3] = std::min(p[c*6+d+3], uhi);    // tighten hi
        }
    }
}
```

### 7.8 自适应冻结与冻结优先切分的交互

| 场景 | split_order | 行为 |
|------|-------------|------|
| 根节点 (W=5.94) | FREEZE_PRIORITY | fd=0 → Stage 1 全 iFK |
| depth=2 (q₀=0.74) | FREEZE_PRIORITY | fd=1 → ep_store 在 fd=1 坐标系 |
| depth=3 切 q₀ | FREEZE_PRIORITY | dim=0 < fd=1 → **Case A** 继承 |
| depth=5 (q₁=0.74) | FREEZE_PRIORITY | fd=2 → **注意：子节点的 fd 可能更大** |

**重要设计决策**：

当 Case A 继承时，子节点沿用 `parent_fd` 而不是重新计算 `effective_fd`。
原因：`ep_store` 是在 `parent_fd` 坐标系下计算的，如果使用更大的 `fd`
则需要完全重算 Stage 1（违背继承优化的初衷）。

当 Case B（非冻结维度切分）时，`compute_envelope` 会为子节点重新计算 `effective_fd`。
如果子节点的区间更窄，`effective_fd` 可能**增加**（冻结更多关节），
这是自适应策略的核心优势。

### 7.9 文件操作

```
1. 完善 include/sbf/forest/node_store.h ← 添加核心 SoA 数组 + 方法
2. 新建/替换 src/forest/node_store.cpp  ← SoA 操作实现
3. 替换 src/forest/lect.cpp             ← 完整 LECT 实现
4. 微调 include/sbf/forest/lect.h       ← 添加 init_split_dims 声明
5. CMakeLists.txt: 确认 node_store.cpp 在源文件列表中
6. 编译验证
```

**预估：~800 行，1.5h**

---

## 8. S7: 编译验证 + 单元测试

### 8.1 编译验证

```bash
cmake --build . --config Release  # 零错误零警告 → sbf4.lib
```

### 8.2 单元测试

| 测试文件 | 内容 | 预估行数 |
|---------|------|---------|
| `tests/test_interval_math.cpp` | I_sin/I_cos 精度; outward rounding; build_dh_joint 对称性; imat_mul_dh 单位元 | ~100 |
| `tests/test_interval_fk.cpp` | compute_fk_full 返回有效 FKState; incremental == full(重新计算); extract_link_aabbs 输出合理; n_endpoints 正确 | ~150 |
| `tests/test_endpoint_source.cpp` | IFK 路径产生 n_endpoints×6 输出; 增量与全量一致; extract_link_iaabbs 输出 n_active×6 | ~100 |
| `tests/test_lect_ifk.cpp` | 构造 LECT; init_root 产生根节点; compute_envelope 产生有效 AABB; split_leaf Case A 继承正确; split_leaf Case B 增量正确; effective_freeze_depth 随深度变化 | ~200 |

### 8.3 文件操作

```
1. 新建 tests/test_interval_math.cpp
2. 新建 tests/test_interval_fk.cpp
3. 新建 tests/test_endpoint_source.cpp
4. 新建 tests/test_lect_ifk.cpp
5. CMakeLists.txt: 添加测试目标
6. 运行测试
```

**预估：~550 行，1.0h**

---

## 9. 与自适应冻结深度方案的整合点清单

| 整合点 | 位置 | 说明 |
|--------|------|------|
| `FreezePolicy` | config.h (已完成) | 双重上限参数化 |
| `SplitOrder` | config.h (已完成) | 三种切分策略枚举 |
| `init_freeze_depth` | lect.cpp (已完成) | DH 自动推导 + cap |
| `effective_freeze_depth` | lect.cpp (已完成) | 双重 cap 逻辑 |
| `init_split_dims` | lect.cpp (**S6 新增**) | FREEZE_PRIORITY 序列生成 |
| `compute_envelope` | lect.cpp (**S6 实现**) | per-node fd + Stage 1/2 |
| `split_leaf` | lect.cpp (**S6 实现**) | Case A/B + parent_fd 继承 |
| `store_.freeze_depth` | node_store (**S6 完善**) | per-node 冻结深度存储 |
| `reconstructMLvl` | interval_trig.h (已完成) | per-node fd 多层重建 |

### 9.1 与冻结优先切分的交互矩阵

```
                           SplitOrder
                  ┌───────────────────────────────────┐
                  │ ROUND_ROBIN  │ FREEZE_PRIORITY    │
  adaptive=false  │ v3 兼容      │ 固定 fd + 优先缩窄  │
  adaptive=true   │ 自适应+均匀切│ 自适应+优先缩窄     │
                  └───────────────────────────────────┘
```

**推荐组合**：`adaptive=true` + `FREEZE_PRIORITY`
- Phase I 快速缩窄冻结维度 → `effective_fd` 尽早提升
- Phase II 后冻结维度 split 全部 Case A 继承
- 实测前使用 `ROUND_ROBIN` 作为默认值（v3 兼容），实验后切换

---

## 10. 涉及文件清单

| 文件 | 操作 | 预估行数 | 步骤 |
|------|------|---------|------|
| `include/sbf/robot/robot.h` | 替换 stub | ~146 | S1 |
| `src/robot/robot.cpp` | 替换 stub | ~254 | S1 |
| `include/sbf/robot/interval_math.h` | 替换 stub | ~60 | S2 |
| `src/robot/interval_math.cpp` | 替换 stub | ~153 | S2 |
| `include/sbf/robot/interval_fk.h` | 替换 stub | ~67 | S3 |
| `src/robot/interval_fk.cpp` | 替换 stub | ~255 | S3 |
| `include/sbf/envelope/envelope_derive.h` | **新建** | ~108 | S4 |
| `src/envelope/envelope_derive.cpp` | **新建** | ~216 | S4 |
| `src/envelope/endpoint_source.cpp` | 实现 iFK 路径 | ~220 | S5 |
| `include/sbf/forest/node_store.h` | 完善 SoA | ~120 | S6 |
| `src/forest/node_store.cpp` | 实现 SoA | ~200 | S6 |
| `src/forest/lect.cpp` | 完整实现 | ~500 | S6 |
| `include/sbf/forest/lect.h` | 微调 | ~10 | S6 |
| `include/sbf/core/types.h` | 添加 MAX_TF/MAX_JOINTS（如缺） | ~5 | S3 |
| `CMakeLists.txt` | 添加新源文件 | ~5 | S4 |
| `tests/test_interval_math.cpp` | **新建** | ~100 | S7 |
| `tests/test_interval_fk.cpp` | **新建** | ~150 | S7 |
| `tests/test_endpoint_source.cpp` | **新建** | ~100 | S7 |
| `tests/test_lect_ifk.cpp` | **新建** | ~200 | S7 |
| **合计** | | **~2,869 行** | |

---

## 11. 时间线

| 步骤 | 内容 | 依赖 | 预估 |
|------|------|------|------|
| S1 | Robot 类迁移 | — | 0.5h |
| S2 | interval_math 完整迁移 | S1 | 0.3h |
| S3 | interval_fk 迁移 | S2 | 0.5h |
| S4 | envelope_derive 迁移 | — | 0.3h |
| S5 | endpoint_source iFK 路径 | S3, S4 | 0.5h |
| S6 | LECT iFK 集成 + 自适应冻结 | S5 | 1.5h |
| S7 | 编译验证 + 单元测试 | S6 | 1.0h |
| **合计** | | | **~4.6h** |

> S4 与 S1-S3 无依赖，可并行开发。

---

## 12. 验收标准

1. **编译**: `cmake --build . --config Release` 零错误零警告
2. **iFK 全链路**: Robot → compute_fk_full → fk_to_endpoints → endpoint_iaabbs →
   reconstruct → extract_link_iaabbs → per-link iAABBs 全部产生有效结果
3. **自适应冻结**: 根节点 fd=0，深层节点 fd>0，per-node fd 存储正确
4. **Case A 继承**: 冻结维度切分时 ep_store 被正确继承，仅 Stage 2 重建
5. **Case B 增量**: 非冻结维度切分时 incremental FK 正确，compute_envelope 重算
6. **冻结优先切分**: `FREEZE_PRIORITY` 模式下 split_dims 按设计序列生成
7. **v3 兼容**: `FreezePolicy::v3_compatible()` + `ROUND_ROBIN` 时行为与 v3 一致
8. **单元测试**: 全部通过
9. **术语**: 使用 `endpoint_iaabb` / `link_iaabb` / `EndpointIAABBResult`

---

## 13. 后续工作（本计划范围外）

| 任务 | 优先级 | 依赖 |
|------|--------|------|
| CritSample 路径迁移 (Phase D) | 高 | S5 完成 |
| Analytical 路径迁移 (Phase E) | 中 | Phase D |
| GCPC 路径迁移 (Phase F) | 中 | Phase E |
| find_free_box 完整实现 | 高 | S6 完成 |
| Hull Grid 计算 | 中 | S6 + envelope_type |
| 磁盘缓存 (save/load) | 低 | 全部 pipeline |
| 冻结深度实验 1-6 | 高 | S6 完成 |
