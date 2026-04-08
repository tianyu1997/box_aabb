# Endpoint iAABB 迁移计划（v3 → v4）

> 创建时间: 2026-03-19
> 迁移范围: Stage 1 完整流水线（endpoint_iaabb 计算 + 4 条路径 + robot 模块 + 全部依赖）
> 基准: v3 最新稳定状态 → v4 骨架（已搭建、已编译通过）

---

## 0. 迁移概述

将 v3 的 `compute_endpoint_aabb()` 完整 Stage 1 流水线迁移至 v4 的 `compute_endpoint_iaabb()`。
包含 **4 条端点源路径**（iFK / CritSample / Analytical / GCPC）及其全部依赖模块。

### 迁移规模

| 类别 | v3 文件数 | 总行数 | v4 操作 |
|------|---------|--------|---------|
| robot 模块 (头文件) | 5 | 475 行 | 替换 v4 stub |
| robot 模块 (源文件) | 5 | 906 行 | 替换 v4 stub |
| envelope 核心 (头文件) | 6 | 1,164 行 | 替换/创建 |
| envelope 核心 (源文件) | 6 | 6,409 行 | 替换/创建 |
| 辅助模块 (mmap_util 等) | 1 | 245 行 | 新增 |
| **合计** | **23 个文件** | **~9,199 行** | — |

### v4 额外改进（相对 v3）

1. **i-前缀重命名**：`endpoint_aabb` → `endpoint_iaabb`，`SubAABB` → `LinkIAABB`
2. **文件重命名**：`frame_source` → `endpoint_source`，`gcpc_cache` → `gcpc`，`frame_store` → `endpoint_store`
3. **大文件拆分**：`envelope_derive_critical.cpp`（3,097 行）→ `crit_sample.cpp` + `analytical_solve.cpp`
4. **代码去重**：4 个重复工具函數提取为共享头文件 `analytical_utils.h`
5. **删除全部 backward-compat 别名**

---

## 1. 依赖关系图

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          core/types.h (已完成)                              │
│          Interval, AABB3D, BoxNode, JointLimits, 常量                      │
└────────────┬────────────────────────────┬───────────────────────────────────┘
             │                            │
     ┌───────▼────────┐          ┌────────▼─────────┐
     │  robot/robot.h  │          │ core/mmap_util.h │    ← Phase A 新增
     │  DHParam, Robot  │          │ (mmap 工具)      │
     └───┬───┬───┬────┘          └────────┬─────────┘
         │   │   │                        │
    ┌────▼┐ ┌▼────┐ ┌──▼─────────┐  ┌────▼────────────┐
    │fk.h │ │i_fk │ │affine_fk.h │  │endpoint_store.h │
    │标量FK│ │区间FK│ │仿射FK      │  │(原 frame_store) │
    └──┬──┘ └──┬──┘ └──┬─────────┘  └─────────────────┘
       │       │       │
    ┌──▼───────▼───────▼──────────────────────────────────────────┐
    │              envelope/envelope_derive.h                      │
    │  derive_aabb, derive_aabb_subdivided, derive_grid            │
    └──────────────────────┬──────────────────────────────────────┘
                           │
    ┌──────────────────────▼──────────────────────────────────────┐
    │              envelope/analytical_utils.h (v4 新增)           │
    │  FixedRoots, solve_poly_in_interval, half_angle_to_q,       │
    │  build_bg_values, FKWorkspace, crit_angles, build_csets,     │
    │  crit_enum, crit_enum_fk                                     │
    └──────────┬───────────────────────┬──────────────────────────┘
               │                       │
    ┌──────────▼──────────┐  ┌────────▼───────────┐  ┌───────────────┐
    │ crit_sample.h/.cpp  │  │analytical_solve.h/ │  │ gcpc.h/.cpp   │
    │ CritSample 路径     │  │  .cpp               │  │ GCPC 路径     │
    │ ~1,500 行           │  │ Analytical 路径     │  │ ~2,329 行     │
    │                     │  │ ~1,600 行           │  │               │
    └──────────┬──────────┘  └────────┬───────────┘  └───────┬───────┘
               │                      │                      │
    ┌──────────▼──────────────────────▼──────────────────────▼───────┐
    │                  endpoint_source.h/.cpp                         │
    │  compute_endpoint_iaabb() — 4 路径 dispatcher                  │
    │  fk_to_endpoints(), merge_analytical_to_endpoints()            │
    │  extract_link_iaabbs()                                          │
    │  ~300 行                                                        │
    └────────────────────────────────────────────────────────────────┘
```

---

## 2. 分步实施计划

### Phase A: Robot 模块迁移（基础层）

**目标**：迁移 Robot 类、标量 FK、区间 FK、仿射 FK、区间数学，使 v4 robot 模块完整可用。

| 步骤 | 操作 | v3 源 | v4 目标 | 行数 |
|------|------|-------|---------|------|
| A-1 | 替换头文件 | `robot/robot.h` (117 行) | `include/sbf/robot/robot.h` | 117 |
| A-2 | 替换实现 | `robot/robot.cpp` (226 行) | `src/robot/robot.cpp` | 226 |
| A-3 | 替换头文件 | `robot/interval_math.h` (46 行) | `include/sbf/robot/interval_math.h` | 46 |
| A-4 | 替换实现 | `robot/interval_math.cpp` (128 行) | `src/robot/interval_math.cpp` | 128 |
| A-5 | 替换头文件 | `robot/fk.h` (35 行) | `include/sbf/robot/fk.h` | 35 |
| A-6 | 替换实现 | `robot/fk_scalar.cpp` (135 行) | `src/robot/fk_scalar.cpp` | 135 |
| A-7 | 替换头文件 | `robot/interval_fk.h` (54 行) | `include/sbf/robot/interval_fk.h` | 54 |
| A-8 | 替换实现 | `robot/interval_fk.cpp` (220 行) | `src/robot/interval_fk.cpp` | 220 |
| A-9 | 替换头文件 | `robot/affine_fk.h` (223 行) | `include/sbf/robot/affine_fk.h` | 223 |
| A-10 | 替换实现 | `robot/affine_fk.cpp` (197 行) | `src/robot/affine_fk.cpp` | 197 |
| A-11 | 编译验证 | — | `cmake --build . --config Release` | — |

**v4 改动要点**：

- **放弃 pimpl 模式**：v4 骨架的 `Robot` 用了 `struct Impl;` pimpl，迁移时改为 v3 的直接成员存储（已验证、性能好）。
- **include 路径更新**：`sbf/common/types.h` → `sbf/core/types.h`。
- **保留 v4 新增接口**：`n_endpoints()` 方法（v4 新增便利方法，= `n_joints() + has_tool()`）。
- **v4 interval_math.h stub 错误**：当前 v4 stub 声明了不存在的函数（`interval_sqr`, `interval_sqrt`, `interval_atan2`），需完全替换为 v3 的 `I_sin`, `I_cos`, `build_dh_joint`, `imat_mul_dh` 等。
- **v4 fk.h stub 错误**：当前 v4 stub 声明了 `compute_fk_full()`（应属于 interval_fk），需替换为 v3 的 `dh_transform`, `fk_link_positions`, `fk_transforms` 等标量 FK 签名。
- **v4 interval_fk.h FKState 不完整**：当前仅有 `bool valid`，需迁移 v3 完整定义（`prefix_lo/hi[MAX_TF][16]`, `joints_lo/hi[MAX_JOINTS][16]`, `n_tf`, `n_jm`）。
- **namespace 策略**：`Robot`, `FKState` 等保持在 `sbf::` 命名空间（v3 一致），不急于移入 `sbf::robot::`（那是 Phase 4 的工作）。

**小计：~1,381 行**

---

### Phase B: Envelope Derive 基础函数迁移

**目标**：迁移 `derive_aabb_subdivided()` 等基础派生函数，被 `extract_link_iaabbs()` 和 Stage 2 共同依赖。

| 步骤 | 操作 | v3 源 | v4 目标 | 行数 |
|------|------|-------|---------|------|
| B-1 | 新建头文件 | `envelope/envelope_derive.h` (100 行) | `include/sbf/envelope/envelope_derive.h` | 100 |
| B-2 | 新建实现 | `envelope/envelope_derive.cpp` (181 行) | `src/envelope/envelope_derive.cpp` | 181 |
| B-3 | 更新 CMakeLists.txt | — | 添加 `src/envelope/envelope_derive.cpp` | — |
| B-4 | 编译验证 | — | — | — |

**v4 改动要点**：

- include 路径更新：`sbf/common/types.h` → `sbf/core/types.h`。
- 版本标注改为 "SafeBoxForest v4"。
- 函数签名不变（纯浮点指针操作，无命名冲突）。

**小计：~281 行**

---

### Phase C: 共享工具函数抽取

**目标**：将 v3 中 `envelope_derive_critical.cpp` 和 `gcpc_cache.cpp` 之间重复的工具函数统一抽出为 `analytical_utils.h`，消除代码重复。

| 步骤 | 操作 | 来源 | v4 目标 |
|------|------|------|---------|
| C-1 | 新建内部头文件 | 从两个大文件提取 | `include/sbf/envelope/analytical_utils.h` |

**提取的共享工具**：

| 工具 | v3 中的位置 | 说明 |
|------|------------|------|
| `struct FixedRoots` | critical L1757, gcpc L50 | 固定容量根（最多 8 个） |
| `solve_poly_in_interval()` | critical L1769, gcpc L64 | 区间限制多项式求根（bisection） |
| `half_angle_to_q()` | critical L1860, gcpc L112 | 半角参数化 → 关节角 |
| `build_bg_values()` | critical L2250, gcpc L190 | pair-constrained 背景角度网格 |
| `struct FKWorkspace` | critical L32 | 栈分配标量 FK 工作空间 |
| `crit_angles()` | critical L83 | 区间内 kπ/2 临界角枚举 |
| `build_csets()` | critical L97 | 笛卡尔积构建（含降级逻辑） |
| `crit_enum()` | critical L119 | 递归临界配置枚举 |
| `crit_enum_fk()` | critical L135 | 增量 FK 版枚举 |

**注意**：`GcpcFKWorkspace`（gcpc L148）与 `FKWorkspace` 不同（前者用标量 FK，后者用 Eigen Matrix4d），保持独立在 `gcpc.cpp` 中。

**小计：~200 行**（新文件）

---

### Phase D: CritSample 路径迁移

**目标**：迁移 CritSample（临界采样）端点源。

| 步骤 | 操作 | v3 源行范围 | v4 目标 | 行数 |
|------|------|-----------|---------|------|
| D-1 | 替换头文件 | `envelope_derive_critical.h` L45–L201 | `include/sbf/envelope/crit_sample.h` | ~160 |
| D-2 | 替换实现 | `envelope_derive_critical.cpp` 多段 | `src/envelope/crit_sample.cpp` | ~1,500 |
| D-3 | 编译验证 | — | — | — |

**v3 → v4 crit_sample.cpp 代码映射**：

```
v3 envelope_derive_critical.cpp          → v4 crit_sample.cpp
─────────────────────────────────────    ───────────────────────
L159-215:  derive_crit_endpoints()       → derive_crit_endpoints()
L215-369:  derive_aabb_critical()        → derive_iaabb_critical()  (i-前缀)
L369-527:  derive_obb_critical()         → 保留（OBB 功能）
           collide_obb_obs()             → 保留
           volume_obb_slots()            → 保留
L550-580:  struct LinkExtremes           → 移至此文件
L581-698:  generate_coupled_*()          → 移至此文件
L699-768:  generate_manifold_*()         → 移至此文件
L769-1043: lbfgs_minimize (slow+fast)    → 移至此文件
L1044-1279: optimize_aabb_extremes       → 移至此文件
L1284-1483: derive_aabb_critical_enhanced → derive_iaabb_critical_enhanced
```

**v4 改动要点**：

- **`CriticalSamplingConfig` 完整迁移**：当前 v4 stub 缺少 `max_combos`, `manifold_n_per`, `lbfgs_ftol`, `lbfgs_n_seeds`, `use_analytical_jacobian`, `restrict_joint_scope`, `smart_seed_selection`, `skip_gap_threshold`，全部从 v3 迁移。
- **工厂方法完整迁移**：`all_enabled()`, `baseline()`, `full_slow()`, `full_fast()`。
- 共享工具（`crit_angles`, `build_csets` 等）改为 include `analytical_utils.h`。
- 函数名 i-前缀化（如 `derive_aabb_critical` → `derive_iaabb_critical`）。
- `LinkExtremes` 结构：CritSample 和 Analytical 共用。由于 Analytical 不依赖 CritSample 头文件，将其放在 `analytical_utils.h` 或 `crit_sample.h`（哪个更合理取决于 analytical_solve 是否也 include crit_sample.h）。**决策：放在 `analytical_utils.h`**，因为 Analytical 也使用。

**小计：~1,660 行**

---

### Phase E: Analytical 路径迁移

**目标**：迁移 Analytical（解析求解）端点源。

| 步骤 | 操作 | v3 源行范围 | v4 目标 | 行数 |
|------|------|-----------|---------|------|
| E-1 | 替换头文件 | `envelope_derive_critical.h` L225–305 | `include/sbf/envelope/analytical_solve.h` | ~90 |
| E-2 | 替换实现 | `envelope_derive_critical.cpp` 多段 | `src/envelope/analytical_solve.cpp` | ~1,580 |
| E-3 | 编译验证 | — | — | — |

**v3 → v4 analytical_solve.cpp 代码映射**：

```
v3 envelope_derive_critical.cpp          → v4 analytical_solve.cpp
─────────────────────────────────────    ───────────────────────────
L1516-1591: eval_and_update() 两个重载  → 移至此文件
L1592-1756: solve_edges()               → 移至此文件
L1814-1871: build_symbolic_poly8()      → 移至此文件
L1872-2112: solve_faces()               → 移至此文件
L2113-2249: solve_interior()            → 移至此文件
L2273-2461: solve_pair_constrained_1d()  → 移至此文件
L2462-2697: solve_pair_constrained_2d()  → 移至此文件
L2698-2909: solve_interior_improved()    → 移至此文件
L2910-3264: derive_aabb_critical_analytical()           → 移至此文件
L3265-3097: derive_aabb_critical_analytical_with_configs → 移至此文件
```

**v4 改动要点**：

- **`AnalyticalCriticalConfig` 完整迁移**：当前 v4 stub 只有 4 个字段，v3 有 ~15 个细粒度控制字段。全部迁移。
- **`AnalyticalCriticalStats` 迁移**：完整的 per-phase 统计。
- 共享工具（`FixedRoots`, `solve_poly_in_interval`, `half_angle_to_q`, `build_bg_values`）改为 include `analytical_utils.h`。
- 使用 `LinkExtremes`（从 `analytical_utils.h` 或 `crit_sample.h` include）。
- 工厂方法完整迁移：`all_enabled()`, `edges_only()`, `edges_and_faces()`, `v1_analytical()`。
- 函数返回的 `out_endpoint_aabb` 参数名保留（中间格式，非最终 endpoint_iaabb）。

**小计：~1,670 行**

---

### Phase F: GCPC 路径迁移

**目标**：迁移 GCPC（全局临界点缓存）端点源。

| 步骤 | 操作 | v3 源 | v4 目标 | 行数 |
|------|------|-------|---------|------|
| F-1 | 替换头文件 | `gcpc_cache.h` (218 行) | `include/sbf/envelope/gcpc.h` | 218 |
| F-2 | 替换实现 | `gcpc_cache.cpp` (2,329 行) | `src/envelope/gcpc.cpp` | ~2,300 |
| F-3 | 编译验证 | — | — | — |

**v4 改动要点**：

- 文件重命名：`gcpc_cache.h/cpp` → `gcpc.h/cpp`。
- include 路径更新。
- 代码重复的 4 个工具函数（`FixedRoots`, `solve_poly_in_interval`, `half_angle_to_q`, `build_bg_values`）删除本地定义，改为 include `analytical_utils.h`。
- `Vandermonde QR`（L125）保留在 `gcpc.cpp` 中（仅 GCPC 使用）。
- `GcpcFKWorkspace`（L148）保留在 `gcpc.cpp` 中（仅 GCPC 使用，与 `FKWorkspace` 不同）。
- `GcpcCache` 接口变更：`is_loaded()` 改为使用 `!sections_.empty()`（保持 v3 一致）。
- `derive_aabb_with_gcpc()` 内部逻辑不变（纯迁移，算法不改）。

**小计：~2,518 行**

---

### Phase G: Endpoint Source Dispatcher 迁移

**目标**：迁移 `compute_endpoint_iaabb()` 的完整 dispatcher 逻辑。

| 步骤 | 操作 | v3 源 | v4 目标 | 行数 |
|------|------|-------|---------|------|
| G-1 | 更新头文件 | `frame_source.h` (253 行) | `include/sbf/envelope/endpoint_source.h` | ~170 |
| G-2 | 替换实现 | `frame_source.cpp` (265 行) | `src/envelope/endpoint_source.cpp` | ~220 |
| G-3 | 编译验证 | — | — | — |

**v4 改动要点**：

- **EndpointSourceConfig 改回 by-value 存储**：v4 骨架用了 `const CriticalSamplingConfig* crit_config_ptr`（指针），但完整配置类型现已可用，改回 v3 的 by-value 存储（`CriticalSamplingConfig crit_config`）。理由：消除生命周期管理假设，配置结构体体积小（几十字节），值语义更安全。
- **i-前缀重命名**：
  - `EndpointAABBResult` → `EndpointIAABBResult`
  - `endpoint_aabbs` → `endpoint_iaabbs`
  - `compute_endpoint_aabb()` → `compute_endpoint_iaabb()`
  - `extract_link_aabbs_from_endpoint()` → `extract_link_iaabbs()`
- **删除全部 backward-compat 别名**：
  - `FrameSourceMethod`, `FrameSourceConfig`, `FrameSourceResult` — 删除
  - `compute_frame_source()`, `compute_frame_source_incremental()` — 删除
  - `extract_link_aabbs_from_result()` — 删除
  - `frame_source_name()` — 删除
- **include 路径更新**：
  - `"sbf/envelope/frame_source.h"` → `"sbf/envelope/endpoint_source.h"`
  - `"sbf/envelope/envelope_derive_critical.h"` → `"sbf/envelope/crit_sample.h"` + `"sbf/envelope/analytical_solve.h"`
  - `"sbf/envelope/gcpc_cache.h"` → `"sbf/envelope/gcpc.h"`
- endpoint_source.h 不再全量 include crit_sample.h / analytical_solve.h / gcpc.h。改为前向声明，仅在 endpoint_source.cpp 中 include（依赖切断）。

**小计：~390 行**

---

### Phase H: Endpoint Store + Envelope Cache 迁移

**目标**：迁移 per-node endpoint iAABB 存储和缓存验证。

| 步骤 | 操作 | v3 源 | v4 目标 | 行数 |
|------|------|-------|---------|------|
| H-1 | 新建 mmap_util.h | `common/mmap_util.h` (245 行) | `include/sbf/core/mmap_util.h` | 245 |
| H-2 | 替换头文件 | `frame_store.h` (193 行) | `include/sbf/envelope/endpoint_store.h` | ~190 |
| H-3 | 替换实现 | `frame_store.cpp` (439 行) | `src/envelope/endpoint_store.cpp` | ~430 |
| H-4 | 替换头文件 | `envelope_cache.h` (95 行) | `include/sbf/envelope/envelope_cache.h` | ~90 |
| H-5 | 替换实现 | `envelope_cache.cpp` (138 行) | `src/envelope/envelope_cache.cpp` | ~130 |
| H-6 | 编译验证 | — | — | — |

**v4 改动要点**：

- `FrameStore` → `EndpointStore`（类名 + 文件名重命名）。
- `store_frames` / `get_frames` / `union_frames` → `store_endpoints` / `get_endpoints` / `union_endpoints`。
- `mmap_util.h` 路径从 `sbf/common/` → `sbf/core/`。
- `envelope_cache.h/.cpp` 中引用的 `FrameSourceMethod` → `EndpointSource`，`frame_source_name` → `endpoint_source_name` 等。

**小计：~1,085 行**

---

### Phase I: CMakeLists.txt 更新 + 全量编译

| 步骤 | 操作 |
|------|------|
| I-1 | 更新 CMakeLists.txt 源文件列表：添加 `src/envelope/envelope_derive.cpp` |
| I-2 | 确认所有 include 路径正确 |
| I-3 | 全量编译：`cmake --build . --config Release` → 零错误零警告 |

---

### Phase J: 术语检查 + 日志更新

| 步骤 | 操作 |
|------|------|
| J-1 | 全局 grep 验证 v4 代码中无 v3 残留术语 |
| J-2 | 更新 `doc/CHANGE_LOG_CN.md` |

**grep 验证清单**（v4 `include/` + `src/` 范围内，排除注释）：

| grep 模式 | 期望匹配数 |
|-----------|----------|
| `endpoint_aabb` (非 `endpoint_iaabb`) | 0 |
| `frame_source` | 0（仅注释可提及） |
| `FrameSourceMethod\|FrameSourceConfig\|FrameSourceResult` | 0 |
| `compute_frame_source\|compute_envelope_repr` | 0 |
| `extract_link_aabbs_from_result\|extract_link_aabbs_from_endpoint` | 0 |
| `gcpc_cache` (include 路径中) | 0 |
| `sbf/common/` (include 路径中) | 0 |
| `SubAABB` (枚举值，非注释) | 0 |

---

## 3. v3 → v4 完整文件映射

### 头文件

| v3 路径 | v4 路径 | 操作 |
|---------|---------|------|
| `include/sbf/robot/robot.h` | `include/sbf/robot/robot.h` | 替换 stub |
| `include/sbf/robot/interval_math.h` | `include/sbf/robot/interval_math.h` | 替换 stub |
| `include/sbf/robot/fk.h` | `include/sbf/robot/fk.h` | 替换 stub |
| `include/sbf/robot/interval_fk.h` | `include/sbf/robot/interval_fk.h` | 替换 stub |
| `include/sbf/robot/affine_fk.h` | `include/sbf/robot/affine_fk.h` | 替换 stub |
| `include/sbf/envelope/envelope_derive.h` | `include/sbf/envelope/envelope_derive.h` | **新增** |
| `include/sbf/envelope/envelope_derive_critical.h` | `include/sbf/envelope/crit_sample.h` + `analytical_solve.h` | 拆分替换 |
| `include/sbf/envelope/gcpc_cache.h` | `include/sbf/envelope/gcpc.h` | 重命名替换 |
| `include/sbf/envelope/frame_source.h` | `include/sbf/envelope/endpoint_source.h` | 重命名替换 |
| `include/sbf/envelope/frame_store.h` | `include/sbf/envelope/endpoint_store.h` | 重命名替换 |
| `include/sbf/envelope/envelope_cache.h` | `include/sbf/envelope/envelope_cache.h` | 替换 stub |
| `include/sbf/common/mmap_util.h` | `include/sbf/core/mmap_util.h` | **新增** |
| （无） | `include/sbf/envelope/analytical_utils.h` | **v4 新增（去重）** |

### 源文件

| v3 路径 | v4 路径 | 操作 |
|---------|---------|------|
| `src/robot/robot.cpp` | `src/robot/robot.cpp` | 替换 stub |
| `src/robot/interval_math.cpp` | `src/robot/interval_math.cpp` | 替换 stub |
| `src/robot/fk_scalar.cpp` | `src/robot/fk_scalar.cpp` | 替换 stub |
| `src/robot/interval_fk.cpp` | `src/robot/interval_fk.cpp` | 替换 stub |
| `src/robot/affine_fk.cpp` | `src/robot/affine_fk.cpp` | 替换 stub |
| `src/envelope/envelope_derive.cpp` | `src/envelope/envelope_derive.cpp` | **新增** |
| `src/envelope/envelope_derive_critical.cpp` (L159–1483) | `src/envelope/crit_sample.cpp` | 拆分替换 |
| `src/envelope/envelope_derive_critical.cpp` (L1484–3097) | `src/envelope/analytical_solve.cpp` | 拆分替换 |
| `src/envelope/gcpc_cache.cpp` | `src/envelope/gcpc.cpp` | 重命名替换 |
| `src/envelope/frame_source.cpp` | `src/envelope/endpoint_source.cpp` | 重命名替换 |
| `src/envelope/frame_store.cpp` | `src/envelope/endpoint_store.cpp` | 重命名替换 |
| `src/envelope/envelope_cache.cpp` | `src/envelope/envelope_cache.cpp` | 替换 stub |

---

## 4. 关键设计决策

### 4.1 Robot 类：放弃 pimpl，改用直接成员

v4 骨架用了 `struct Impl;` pimpl 模式，但 v3 Robot 使用直接成员存储。

**决策**：采用 v3 的直接成员存储。
**理由**：
- 已经过完整测试验证
- 无 ABI 稳定性需求（v4 是内部库）
- 避免额外 heap alloc 和指针间接
- pimpl 可在后续需要 ABI 兼容时再引入

### 4.2 EndpointSourceConfig：by-value 而非指针

v4 骨架用了 `const CriticalSamplingConfig* crit_config_ptr`（指针存储），试图减少头文件依赖。

**决策**：改回 v3 的 by-value 存储（`CriticalSamplingConfig crit_config`）。
**理由**：
- 配置结构体体积小（~100 字节），拷贝成本忽略
- 消除生命周期管理风险（指针可能悬空）
- 值语义更简单安全
- endpoint_source.h 已经需要前向声明这些类型，include 完整定义不会显著增加编译时间

### 4.3 大文件拆分边界

`envelope_derive_critical.cpp`（3,097 行）的拆分方案：

```
CritSample 归属 (crit_sample.cpp):
├── L32–158:   FKWorkspace + crit 工具 → analytical_utils.h
├── L159–527:  derive_crit_endpoints + derive_aabb_critical + OBB 相关
├── L550–580:  LinkExtremes → analytical_utils.h
├── L581–768:  generate_coupled + generate_manifold
├── L769–1043: L-BFGS-B minimize (slow+fast)
├── L1044–1279: optimize_aabb_extremes (slow+fast)
└── L1284–1483: derive_aabb_critical_enhanced

Analytical 归属 (analytical_solve.cpp):
├── L1516–1591: eval_and_update() 重载
├── L1592–1756: solve_edges()
├── L1757–1871: FixedRoots + solve_poly + build_symbolic_poly8 + half_angle → 引用 analytical_utils.h
├── L1872–2112: solve_faces()
├── L2113–2249: solve_interior()
├── L2250–2272: build_bg_values → 引用 analytical_utils.h
├── L2273–2461: solve_pair_constrained_1d()
├── L2462–2697: solve_pair_constrained_2d()
├── L2698–2909: solve_interior_improved()
├── L2910–3264: derive_aabb_critical_analytical()
└── L3265–3097: derive_aabb_critical_analytical_with_configs()
```

### 4.4 共享工具函数去重策略

4 个在两大文件中完全重复的工具函数抽取到 `analytical_utils.h`：

| 函数 | 原位置 1 | 原位置 2 | 拆分后 |
|------|---------|---------|--------|
| `FixedRoots` | critical L1757 | gcpc L50 | `analytical_utils.h` |
| `solve_poly_in_interval()` | critical L1769 | gcpc L64 | `analytical_utils.h` |
| `half_angle_to_q()` | critical L1860 | gcpc L112 | `analytical_utils.h` |
| `build_bg_values()` | critical L2250 | gcpc L190 | `analytical_utils.h` |

另外，以下 CritSample 工具也放入 `analytical_utils.h`（因为 GCPC 的 Phase B 也需要临界角枚举）：

| 函数 | 来源 | 说明 |
|------|------|------|
| `FKWorkspace` | critical L32 | 栈分配 FK 工作空间（Eigen Matrix4d） |
| `crit_angles()` | critical L83 | kπ/2 临界角生成 |
| `build_csets()` | critical L97 | 笛卡尔积构建 |
| `crit_enum()` | critical L119 | 临界配置递归枚举 |
| `crit_enum_fk()` | critical L135 | 增量 FK 版枚举 |
| `LinkExtremes` | critical L550 | per-link min/max 跟踪 |

### 4.5 namespace 策略

本次迁移保持 v3 的命名空间不变：
- `Robot`, `FKState`, `compute_fk_full` 等在 `sbf::` 命名空间
- `EndpointSource`, `compute_endpoint_iaabb` 等在 `sbf::envelope::`
- Namespace 整理（v3 Phase 4: `sbf::robot::` 等）后续单独进行

---

## 5. 风险与对策

| 风险 | 概率 | 影响 | 对策 |
|------|------|------|------|
| v4 stub 的 include 路径与 v3 不一致导致编译错误 | 高 | 低 | 逐 Phase 编译验证，每步修复 |
| 拆分 critical 大文件时遗漏共享 static 函数 | 中 | 中 | 拆分前完整记录所有 static 函数的行号和调用关系 |
| `endpoint_store.h/cpp` 依赖 `mmap_util.h` | 低 | 中 | Phase H 先迁移 mmap_util |
| by-value 改造后，`EndpointSourceConfig` 拷贝增大 | 低 | 忽略 | Config 结构体 ~100 字节，远小于 FKState ~35KB |
| GCPC 的 `GcpcFKWorkspace` 与 `FKWorkspace` 名称冲突 | 低 | 低 | 保持 GCPC 使用独立的 `GcpcFKWorkspace` |

---

## 6. 执行顺序总结

```
Phase A → Phase B → Phase C → Phase D → Phase E → Phase F → Phase G → Phase H → Phase I → Phase J
robot    derive基础  共享工具    CritSample  Analytical   GCPC     dispatcher  store+cache  编译    日志
~1381行    ~281行     ~200行     ~1660行      ~1670行    ~2518行     ~390行     ~1085行
                                                                                     总计: ~9,185行
```

每个 Phase 结束后都进行编译验证（零错误零警告），确保渐进迁移不引入 regression。

---

## 7. 验收标准

1. **编译**: `cmake --build . --config Release` 零错误零警告
2. **术语**: v4 代码中无 `endpoint_aabb`、`SubAABB`、`frame_source`、`FrameSourceMethod` 等 v3 旧名残留
3. **别名**: 无任何 backward-compat alias（已全部清除）
4. **去重**: `FixedRoots` / `solve_poly_in_interval` / `half_angle_to_q` / `build_bg_values` 仅在 `analytical_utils.h` 中定义一次
5. **拆分**: `envelope_derive_critical.cpp` 不存在于 v4 中；功能拆分到 `crit_sample.cpp`（≤1,800 行）和 `analytical_solve.cpp`（≤1,800 行）
6. **日志**: `doc/CHANGE_LOG_CN.md` 包含本次迁移的完整记录
