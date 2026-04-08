# SafeBoxForest v4 C++ 重构计划

> 创建时间: 2026-03-19
> 基准版本: safeboxforest/v3（T-RO journal extension，28 个构建目标，0 errors / 0 warnings）
> 目标: 在 v3 功能完整的基础上，彻底清理架构债务，为后续扩展（planner、动态场景、多机器人）奠定坚实基础

---

## 0. v3 架构审计摘要

### 0.1 v3 核心成就（必须完整保留）
- 两阶段模块化 pipeline：4 EndpointSource × 3 EnvelopeType = 12 组合
- LECT KD-tree + FFB descent + Hull-16 碰撞
- ForestGrower：多根生长 + Wavefront/RRT + 并行 + 桥接 + coarsen
- 稀疏体素引擎：BitBrick + FlatBrickMap + ScanLine 光栅化
- 场景预光栅化 + 两阶段碰撞检测（2272× 加速）
- 无损合并（bitwise OR）
- pybind11 绑定（pysbf3）
- T-RO 论文（中英双语）

### 0.2 v3 架构问题清单（20 项）

| # | 严重度 | 类别 | 问题 |
|---|---|---|---|
| 1 | 🔴 高 | 死代码 | `envelope.h` + `envelope_computer.h/.cpp` 整套 v2 抽象接口，完全未被 v3 pipeline 使用 |
| 2 | 🔴 高 | 命名冲突 | 两个 `SBFConfig`：`sbf::SBFConfig`（config.h）vs `sbf::forest::SBFConfig`（sbf.h） |
| 3 | 🔴 高 | 类型冲突 | 两个 `EnvelopeResult`：`sbf::EnvelopeResult` vs `sbf::envelope::EnvelopeResult` |
| 4 | 🔴 高 | 死代码 | `config.h` 中 80% 旧配置（ForestConfig, BridgeConfig, PlannerConfig, 旧 SBFConfig）已被替代 |
| 5 | 🟡 中 | 别名堆积 | 10+ 个 backward-compat aliases，无 `[[deprecated]]` 标记 |
| 6 | 🟡 中 | 文件命名 | `frame_source.h` 应改名为 `endpoint_source.h`，匹配核心概念 |
| 7 | 🟡 中 | 职责过重 | `envelope_derive_critical.cpp`（3,097 行）包含 CritSample + Analytical 两个独立算法 |
| 8 | 🟡 中 | 职责过重 | `forest_grower.cpp`（1,803 行）混合 5+ 个独立关注点 |
| 9 | 🟡 中 | 残留 | 8 个 `.bak` 文件 |
| 10 | 🟡 中 | 根目录污染 | 46 个临时 `.txt/.log/.csv` 输出 |
| 11 | 🟡 中 | 空模块 | `planner/` 头文件和源文件目录均为空 |
| 12 | 🟡 中 | 版本标注 | 12+ 个头文件仍标注 "SafeBoxForest v2" |
| 13 | 🟡 中 | 网格重叠 | `GridStore`（R=32 bitfield）与 `VoxelGrid`（BitBrick sparse）两套网格并存 |
| 14 | 🟡 中 | 头文件传递 | `envelope_type.h` 传递包含整个 critical/GCPC 系统 |
| 15 | 🟡 中 | 测试缺失 | Robot, IFK, AAFK, CritSample, GCPC, FrameStore, GridStore, envelope_cache 无独立单元测试 |
| 16 | 🟢 低 | 构建 | `GLOB_RECURSE` 且 `src/` 含 Python 碎片（`__init__.py`, `__pycache__/`） |
| 17 | 🟢 低 | 文档 | `doc/` vs `docs/` 两个文档目录并存 |
| 18 | 🟢 低 | 命名空间 | robot 模块函数在 `sbf::` 而非 `sbf::robot::` |
| 19 | 🟢 低 | viz 混合 | `src/viz/` 混合 C++（.cpp）和 Python（.py）文件 |
| 20 | 🟡 中 | 命名不一致 | `endpoint_aabb` / `SubAABB` 命名未体现区间算术（interval）前缀，与 `iFK` 命名惯例不统一；`SubAABB` 称呼语义不清 |

---

## 1. 重构原则

1. **功能零回归**：v4 每个 phase 完成后必须通过 v3 全部 16 个测试的等价测试
2. **渐进迁移**：v3 保持冻结可用，v4 逐 phase 复制 + 重构，不做 big-bang
3. **一次清理到位**：所有 backward-compat aliases、.bak、死代码在迁移时一次性删除，不带入 v4
4. **新增 planner 骨架**：v4 中 planner 模块至少有接口定义，为后续实现铺路
5. **i-前缀命名惯例**：凡通过区间算术（Interval Arithmetic）计算得到的几何量，统一使用 `i` 前缀——`iFK`（区间 FK）、`endpoint iAABB`（区间端点 AABB）、`link iAABB`（区间连杆 AABB）

---

## 2. v4 命名体系（核心变更）

### 2.1 i-前缀命名惯例

由于 SafeBoxForest 的 AABB 全部基于区间算术推导而非点采样，v4 统一引入 `i` 前缀，与 `iFK`（interval Forward Kinematics）保持一致：

| 概念 | v3 名称 | v4 名称 | 说明 |
|---|---|---|---|
| Stage 1 输出 | `endpoint_aabb` | **`endpoint_iaabb`** | 区间 FK 或临界采样产生的端点区间 AABB |
| Stage 1 结果类型 | `EndpointAABBResult` | **`EndpointIAABBResult`** | 结果结构体 |
| Stage 1 计算函数 | `compute_endpoint_aabb()` | **`compute_endpoint_iaabb()`** | 主入口 |
| Stage 2 枚举值 | `SubAABB` | **`LinkIAABB`** | 连杆区间 AABB，用 `sub=n` 参数控制细分数 |
| Stage 2 枚举值 | `SubAABB_Grid` | **`LinkIAABB_Grid`** | link iAABB 光栅化到 VoxelGrid |
| Stage 2 枚举值 | `Hull16_Grid` | `Hull16_Grid` | 不变（基于凸包而非 AABB） |
| 工厂函数 | `sub_aabb()` | **`link_iaabb()`** | `EnvelopeTypeConfig` 工厂 |
| 工厂函数 | `ifk_sub_aabb()` | **`ifk_link_iaabb()`** | `SBFConfig` 工厂（其余 3 个 source 同理） |
| derive 函数 | `derive_aabb_ifk()` | **`derive_iaabb_ifk()`** | IFK→iAABB 推导 |
| 中间数据 | `endpoint_aabbs [n×6]` | **`endpoint_iaabbs [n×6]`** | 变量/字段名 |
| 提取函数 | `extract_link_aabbs_from_endpoint()` | **`extract_link_iaabbs()`** | 简化 + i 前缀 |

### 2.2 link iAABB(sub=n) 参数化

v3 的 `SubAABB` 名称暗示"子 AABB"是核心概念，但实际上 `sub` 仅是一个参数：
- `link iAABB(sub=1)` = 单个连杆区间 AABB（原来的 "AABB"）
- `link iAABB(sub=4)` = 连杆被细分为 4 段，每段一个 iAABB
- `link iAABB(sub=16)` = 连杆被细分为 16 段

v4 中 `EnvelopeType` 枚举变为：

```cpp
enum class EnvelopeType : uint8_t {
    LinkIAABB      = 0,  // link iAABB(sub=n), n≥1
    LinkIAABB_Grid = 1,  // link iAABB rasterised to VoxelGrid
    Hull16_Grid    = 2,  // 16-point convex hull rasterised to sparse BitBrick VoxelGrid
};
```

### 2.3 Pipeline 描述更新

```
Stage 1 (EndpointSource): C-space intervals → endpoint_iaabbs [n_endpoints × 6]
  {iFK, CritSample, Analytical, GCPC}  — 4 sources

Stage 2 (EnvelopeType):   endpoint_iaabbs → LinkEnvelope
  {LinkIAABB, LinkIAABB_Grid, Hull16_Grid} — 3 representations

4 × 3 = 12 pipeline combinations.
```

> **历史沿革**：
> - v3 早期有 `AABB`(=0) 和 `SubAABB`(=1) 两个不同枚举值，后合并为 `SubAABB`（n_sub=1 时退化为单 AABB）
> - v4 将 `SubAABB` 正式更名为 `LinkIAABB`，用 `sub=n` 参数化细分，废弃 "SubAABB" 称呼
> - v4 将 `endpoint_aabb` 更名为 `endpoint_iaabb`，与 `iFK` 命名惯例统一

---

## 3. 目标目录结构

```
safeboxforest/v4/
├── CMakeLists.txt                  # 显式文件列表（不用 GLOB_RECURSE）
├── AI_WORKING_SPEC.md              # v4 版 spec（含本计划中的新规则）
│
├── include/sbf/
│   ├── core/                       # ← 原 common/（重命名）
│   │   ├── types.h                 #   Interval, BoxNode, Obstacle, FFBResult, Edge
│   │   ├── config.h                #   全局配置（仅保留 v4 活跃结构）
│   │   └── interval_trig.h         #   区间三角函数
│   │
│   ├── robot/                      # ← 原 robot/（命名空间 sbf::robot::）
│   │   ├── robot.h                 #   DH 参数模型
│   │   ├── fk.h                    #   标量 FK（thin convenience header）
│   │   ├── interval_fk.h           #   区间 FK (iFK)
│   │   ├── affine_fk.h             #   仿射 FK
│   │   └── interval_math.h         #   区间算术
│   │                               #   ✂ 删除 hybrid_aabb.h（已被 envelope pipeline 取代）
│   │
│   ├── envelope/                   # ← 原 envelope/（大幅精简 + i-前缀重命名）
│   │   ├── endpoint_source.h       #   ← 原 frame_source.h（EndpointIAABBResult, compute_endpoint_iaabb()）
│   │   ├── envelope_type.h         #   EnvelopeType enum {LinkIAABB, LinkIAABB_Grid, Hull16_Grid}
│   │   ├── pipeline.h              #   ← 新：PipelineConfig（从 envelope_type.h 拆出）
│   │   ├── crit_sample.h           #   ← 从 envelope_derive_critical.h 拆出
│   │   ├── analytical_solve.h      #   ← 从 envelope_derive_critical.h 拆出
│   │   ├── gcpc.h                  #   ← 原 gcpc_cache.h（重命名）
│   │   ├── envelope_cache.h        #   缓存持久化
│   │   ├── endpoint_store.h        #   ← 原 frame_store.h（重命名）
│   │   └── collision_policy.h      #   碰撞策略分发
│   │                               #   ✂ 删除: envelope.h, envelope_computer.h（v2 死代码）
│   │                               #   ✂ 删除: envelope_derive.h（合并到 endpoint_source）
│   │                               #   ✂ 删除: grid_envelope.h, grid_store.h（旧 R=32 网格，被 VoxelGrid 取代）
│   │
│   ├── voxel/                      # ← 不变
│   │   ├── bit_brick.h
│   │   ├── voxel_grid.h
│   │   └── hull_rasteriser.h
│   │
│   ├── scene/                      # ← 不变
│   │   ├── scene.h
│   │   ├── i_collision_checker.h
│   │   └── aabb_collision_checker.h
│   │
│   ├── forest/                     # ← 原 forest/（拆分大文件）
│   │   ├── sbf.h                   #   SBFConfig 工厂（ifk_link_iaabb() 等）
│   │   ├── lect.h                  #   LECT KD-tree
│   │   ├── forest_grower.h         #   ForestGrower 接口（声明）
│   │   ├── grower_strategy.h       #   ← 新：根选取、子树分区策略
│   │   ├── grower_config.h         #   GrowerConfig
│   │   ├── adjacency_graph.h       #   Sweep-and-Prune
│   │   ├── node_store.h            #   SoA 节点存储
│   │   ├── kd_tree.h               #   最近邻
│   │   └── thread_pool.h           #   线程池
│   │
│   └── planner/                    # ← 新：planner 骨架
│       ├── i_planner.h             #   抽象 planner 接口
│       ├── sbf_planner.h           #   SBF-based planner
│       └── path_result.h           #   规划结果类型
│
├── src/
│   ├── robot/
│   │   ├── robot.cpp
│   │   ├── fk_scalar.cpp
│   │   ├── interval_fk.cpp
│   │   ├── interval_math.cpp
│   │   └── affine_fk.cpp
│   │
│   ├── envelope/
│   │   ├── endpoint_source.cpp     #   ← 原 frame_source.cpp（compute_endpoint_iaabb 实现）
│   │   ├── envelope_type.cpp       #   compute_link_envelope (LinkIAABB / LinkIAABB_Grid / Hull16_Grid)
│   │   ├── crit_sample.cpp         #   ← 从 envelope_derive_critical.cpp 拆出
│   │   ├── analytical_solve.cpp    #   ← 从 envelope_derive_critical.cpp 拆出
│   │   ├── gcpc.cpp                #   ← 原 gcpc_cache.cpp
│   │   ├── envelope_cache.cpp
│   │   ├── endpoint_store.cpp      #   ← 原 frame_store.cpp
│   │   └── collision_policy.cpp
│   │                               #   ✂ 删除: envelope_computer.cpp, grid_envelope.cpp, grid_store.cpp
│   │                               #   ✂ 删除: envelope_derive.cpp（合并到 endpoint_source.cpp 和 crit_sample.cpp）
│   │
│   ├── voxel/
│   │   └── hull_rasteriser.cpp
│   │
│   ├── scene/
│   │   ├── scene.cpp
│   │   └── collision.cpp
│   │
│   ├── forest/
│   │   ├── sbf.cpp
│   │   ├── lect.cpp
│   │   ├── forest_grower.cpp       #   核心扩展循环
│   │   ├── grower_roots.cpp        #   ← 新：根选取 + FPS + 子树分区（从 forest_grower.cpp 拆出）
│   │   ├── grower_bridge.cpp       #   ← 新：跨子树桥接（从 forest_grower.cpp 拆出）
│   │   ├── grower_coarsen.cpp      #   ← 新：贪心合并（从 forest_grower.cpp 拆出）
│   │   ├── adjacency_graph.cpp
│   │   └── node_store.cpp
│   │
│   └── planner/                    # ← 新：planner 骨架实现
│       └── sbf_planner.cpp         #   （初始版本可为空壳 + TODO）
│
├── tests/
│   ├── test_robot.cpp              #   ← 新：Robot 模型单元测试
│   ├── test_interval_fk.cpp        #   ← 新：区间 FK 单元测试
│   ├── test_affine_fk.cpp          #   ← 新：仿射 FK 单元测试
│   ├── test_endpoint_source.cpp    #   ← 新：EndpointSource 各 source 的单元测试
│   ├── test_crit_sample.cpp        #   ← 新：CritSample 单元测试
│   ├── test_envelope_cache.cpp     #   ← 新：缓存序列化 / name cross-check 测试
│   ├── test_voxel_grid.cpp         #   ← 新：BitBrick / VoxelGrid 单元测试
│   ├── test_modular_pipeline.cpp   #   ← 从 v3 迁移（更新 include paths + 术语）
│   ├── test_sbf.cpp                #   ← 从 v3 迁移
│   ├── test_lect.cpp               #   ← 从 v3 test_lect_phase3.cpp 迁移
│   ├── test_forest_grower.cpp      #   ← 从 v3 迁移
│   ├── test_parallel_grower.cpp    #   ← 从 v3 迁移
│   ├── test_adjacency_graph.cpp    #   ← 新：独立邻接图测试
│   ├── test_bridging.cpp           #   ← 从 v3 test_phase8_bridging.cpp 迁移
│   ├── test_coarsening.cpp         #   ← 从 v3 test_phase10_coarsening.cpp 迁移
│   ├── bench_pipeline_compare.cpp  #   ← 从 v3 迁移
│   └── bench_warm_pipeline.cpp     #   ← 从 v3 迁移
│
├── experiments/                    #   ← 从 v3 迁移（只调用 public API）
│   ├── exp_pipeline_benchmark.cpp
│   ├── exp_coverage.cpp
│   ├── exp_cache_bench.cpp
│   ├── exp_certified_envelope.cpp
│   └── ...
│
├── examples/                       #   ← 从 v3 迁移（只调用 public API）
│   ├── lect_demo.cpp
│   ├── viz_demo.cpp
│   └── output/                     #   示例输出目录
│
├── python/
│   └── bindings.cpp                #   pybind11 绑定（更新 include paths + 删除旧 alias）
│
├── results/                        #   实验结果输出目录
│
├── doc/                            #   设计文档
│   ├── v4_refactoring_plan.md      #   本文件
│   └── ...
│
├── docs/                           #   API 文档（统一为 docs/，删除 doc/ vs docs/ 双目录问题）
│   ├── API_REFERENCE.md
│   ├── API_REFERENCE_CN.md
│   ├── PAPER_CODE_MAP.md
│   └── MIGRATION_FROM_V3.md        #   ← 新：v3→v4 迁移指南
│
├── paper/
│   ├── root.tex
│   └── root_cn.tex
│
└── scripts/                        #   构建/部署脚本
    └── (按需)
```

### 文件数量对比

| | v3 | v4（目标） | 变化 |
|---|---|---|---|
| 头文件 (include/) | 38 | 31 | −7（删死代码 + 合并） |
| 源文件 (src/) | 24 | 22 | −2（删死代码，+3 拆分，−5 删除） |
| 测试 (tests/) | 16 | 17 | +1（新增单元测试 7 个，合并/删除旧 phase 测试） |
| .bak | 8 | 0 | −8 |
| 根目录临时文件 | 46 | 0 | −46 |

---

## 4. 分 Phase 实施计划

### Phase 1: 骨架搭建 + 死代码清除（1-2 天）

**目标**：创建 v4 目录结构，复制 v3 的活跃代码，删除所有死代码

**具体操作**：

| 步骤 | 动作 | 详情 |
|---|---|---|
| 1.1 | 创建目录树 | 按上方目标结构创建全部目录 |
| 1.2 | 复制活跃代码 | 从 v3 复制所有还在使用的 .h/.cpp 文件（38+24=62 个活跃文件） |
| 1.3 | 删除 v2 死代码 | 不复制以下文件：`envelope.h`, `envelope_computer.h/.cpp`, `grid_envelope.h/.cpp`, `grid_store.h/.cpp`, `hybrid_aabb.h`（共 7 个文件，~1,800 行死代码） |
| 1.4 | 清理 config.h | 从 config.h 中删除旧的 `sbf::SBFConfig`（保留 `sbf::forest::SBFConfig`）、删除 `ForestConfig`、`BridgeConfig`、`PlannerConfig` 旧结构体。仅保留 `StoreFormat`、`CollisionPolicy`、`EnvelopeConfig`（仍在 LECT 中使用） |
| 1.5 | 不复制 .bak | 8 个 .bak 文件不带入 v4 |
| 1.6 | 不复制根目录垃圾 | _test_*.py, _fix_*.py, _check_*.py, *.txt, *.log, *.csv, test_gcpc_cache.bin 等全部不带入 |
| 1.7 | 不复制 Python 碎片 | `src/__init__.py`, `src/__pycache__/` 不带入 |
| 1.8 | 编写 CMakeLists.txt | 显式列举源文件（不用 GLOB_RECURSE），配置与 v3 一致 |
| 1.9 | 编译验证 | `cmake --build . --config Release` 零 errors 零 warnings |

**Phase 1 验收标准**：
- v4 编译通过，零 errors 零 warnings
- 功能与 v3 完全一致（所有代码原封不动复制，仅删除死代码）
- 无 .bak、无根目录临时文件、无 Python 碎片

---

### Phase 2: 命名统一 + 别名清除 + i-前缀重命名（1-2 天）

**目标**：将所有名称统一为 v4 最终形式，引入 i-前缀命名惯例，一次性删除所有 backward-compat aliases

**具体操作**：

| 步骤 | 动作 | 影响范围 |
|---|---|---|
| **文件重命名** | | |
| 2.1 | `frame_source.h/.cpp` → `endpoint_source.h/.cpp` | 匹配 EndpointSource 概念 |
| 2.2 | `gcpc_cache.h/.cpp` → `gcpc.h/.cpp` | 去掉实现细节后缀 |
| 2.3 | `envelope_derive_critical.h/.cpp` → `crit_sample.h` + `analytical_solve.h` | 暂不拆实现，仅重命名 |
| 2.4 | `frame_store.h/.cpp` → `endpoint_store.h/.cpp` | 匹配 endpoint 术语 |
| **i-前缀重命名** | | |
| 2.5 | `EndpointAABBResult` → `EndpointIAABBResult` | 结构体 |
| 2.6 | `compute_endpoint_aabb()` → `compute_endpoint_iaabb()` | Stage 1 主入口 |
| 2.7 | `extract_link_aabbs_from_endpoint()` → `extract_link_iaabbs()` | 提取函数 |
| 2.8 | `derive_aabb_ifk()` → `derive_iaabb_ifk()` | IFK derive |
| 2.9 | 所有变量/字段 `endpoint_aabbs` → `endpoint_iaabbs` | 全局搜索替换 |
| **枚举重命名** | | |
| 2.10 | `EnvelopeType::SubAABB` → `EnvelopeType::LinkIAABB` | 枚举值 |
| 2.11 | `EnvelopeType::SubAABB_Grid` → `EnvelopeType::LinkIAABB_Grid` | 枚举值 |
| 2.12 | `EnvelopeTypeConfig::sub_aabb()` → `EnvelopeTypeConfig::link_iaabb()` | 工厂方法 |
| 2.13 | `SBFConfig::ifk_sub_aabb()` → `SBFConfig::ifk_link_iaabb()` | 及其余 3 个 source 同理 |
| **删除旧别名** | | |
| 2.14 | 删除 `FrameSourceMethod`, `FrameSourceConfig`, `FrameSourceResult` 等 typedef | v2→v3 遗留 |
| 2.15 | 删除 `compute_frame_source()`, `compute_envelope_repr()` 别名 | v2→v3 遗留 |
| 2.16 | 删除 `EnvelopeTypeConfig::aabb()`, `SBFConfig::ifk_aabb()` 等 4 个 AABB 别名 | v3 AABB→SubAABB 遗留 |
| **收尾** | | |
| 2.17 | 更新全部 #include 路径、函数调用、类型引用 | 全量 grep 确认零残留 |
| 2.18 | 更新版本标注：所有文件头注释改为 "SafeBoxForest v4" | 消除 v2/v3 标注 |
| 2.19 | 编译验证 | 零 errors 零 warnings |

**Phase 2 验收标准**：
- 代码中无 "SubAABB" 字样（仅历史注释可提及）
- 代码中无 "endpoint_aabb"（统一为 "endpoint_iaabb"）
- 代码中无任何 backward-compat alias
- 所有文件使用最终名称
- 编译通过

---

### Phase 3: 大文件拆分（1-2 天）

**目标**：将职责过重的文件拆分为合理粒度

#### 3.1 拆分 `envelope_derive_critical.cpp`（3,097 行 → 2 个文件）

| 新文件 | 内容 | 估算行数 |
|---|---|---|
| `crit_sample.cpp` | CritSample 三阶段：组合枚举 + manifold 采样 + L-BFGS-B 优化 | ~1,800 |
| `analytical_solve.cpp` | Analytical 解析：1D atan2 + 2D companion matrix + pair-constrained | ~1,300 |

#### 3.2 拆分 `forest_grower.cpp`（1,803 行 → 4 个文件）

| 新文件 | 内容 | 估算行数 |
|---|---|---|
| `forest_grower.cpp` | 构造函数 + Wavefront/RRT 核心扩展循环 + 并行 worker 分派 | ~700 |
| `grower_roots.cpp` | 根选取（3 策略 + FPS）+ 子树分区（uniform / lect-aligned / partition_in_lect） | ~400 |
| `grower_bridge.cpp` | 跨子树桥接 + 边界采样 + RRT face-snap | ~350 |
| `grower_coarsen.cpp` | Greedy coarsening + promotion (leaf merging) | ~350 |

#### 3.3 拆分 `envelope_type.h`（230 行 → 2 个文件）

| 新文件 | 内容 |
|---|---|
| `envelope_type.h` | EnvelopeType enum {LinkIAABB, LinkIAABB_Grid, Hull16_Grid} + EnvelopeTypeConfig + compute_link_envelope() 声明 |
| `pipeline.h` | PipelineConfig（依赖 EndpointSourceConfig + EnvelopeTypeConfig） |

**减少头文件传递**：`envelope_type.h` 不再 `#include "endpoint_source.h"`，改为前向声明。`pipeline.h` 才 include 两者。

**Phase 3 验收标准**：
- 无文件超过 1,000 行
- 头文件传递依赖被切断（`envelope_type.h` 不再拉入 critical/GCPC）
- 编译通过

---

### Phase 4: 命名空间整理 + 配置统一（0.5 天）

**目标**：所有符号归入正确的命名空间，消除配置体系冲突

| 步骤 | 动作 |
|---|---|
| 4.1 | robot 模块函数从 `sbf::` 移入 `sbf::robot::`（`compute_fk_full`, `extract_link_iaabbs` 等） |
| 4.2 | 统一命名空间层次：`sbf::core::`, `sbf::robot::`, `sbf::envelope::`, `sbf::voxel::`, `sbf::scene::`, `sbf::forest::`, `sbf::planner::` |
| 4.3 | 只保留一个 `SBFConfig`（即 `sbf::forest::SBFConfig`，在顶层提供 `using sbf::SBFConfig = sbf::forest::SBFConfig;` 便利别名） |
| 4.4 | 只保留一个 `EnvelopeResult`（即 `sbf::envelope::EnvelopeResult`） |
| 4.5 | config.h 精简为仅存放跨模块共享的配置类型 |
| 4.6 | common/ → core/ 重命名 |
| 4.7 | 删除 `mmap_util.h`（如已停用）或归入 `core/` |

**Phase 4 验收标准**：
- `namespace sbf { namespace XXX { }}` 层次清晰一致
- 无跨命名空间的同名类型
- 编译通过

---

### Phase 5: 补充单元测试（1-2 天）

**目标**：为 v3 中缺少独立测试的模块补充单元测试

| 测试文件 | 覆盖模块 | 关键测试用例 |
|---|---|---|
| `test_robot.cpp` | Robot | DH 参数加载、FK 一致性（标量 vs 区间 at width=0） |
| `test_interval_fk.cpp` | IntervalFK | 边界包含性（随机点在区间 iAABB 内）、退化区间（width=0） |
| `test_affine_fk.cpp` | AffineFK | AA vs IA 结果一致性、紧致度对比 |
| `test_endpoint_source.cpp` | EndpointSource | 4 种 source 基本正确性、output shape、iAABB 包含性 |
| `test_crit_sample.cpp` | CritSample | 临界点正确性（比 iFK 更紧？）、边界包含性 |
| `test_envelope_cache.cpp` | EnvelopeCache | 写入→读取一致性、name cross-check 拒绝不匹配缓存 |
| `test_voxel_grid.cpp` | VoxelGrid | fill_aabb / fill_hull16 正确性、merge bitwise OR、collides |
| `test_adjacency_graph.cpp` | AdjacencyGraph | Sweep-and-Prune 正确性（暴力对照）、边界情况 |

**Phase 5 验收标准**：
- 所有新测试通过
- 核心模块都有独立单元测试

---

### Phase 6: Planner 骨架 + 文档更新（1 天）

**目标**：为 planner 模块定义接口，更新全部文档

| 步骤 | 动作 |
|---|---|
| 6.1 | 定义 `IPlanner` 抽象接口：`plan(start, goal) → PathResult` |
| 6.2 | 定义 `SBFPlanner`：基于 SBF + AdjacencyGraph 的路径搜索（可初始为空壳 + TODO） |
| 6.3 | 定义 `PathResult` 类型：路径点序列、路径成本、规划时间、成功标志 |
| 6.4 | 更新 `docs/API_REFERENCE.md` + `docs/API_REFERENCE_CN.md`（反映 v4 术语：LinkIAABB, endpoint_iaabb 等） |
| 6.5 | 更新 `docs/PAPER_CODE_MAP.md` |
| 6.6 | 创建 `docs/MIGRATION_FROM_V3.md`（列出所有 v3→v4 的 breaking changes，含完整命名映射表） |
| 6.7 | 合并 v3 遗留的 `doc/` + `docs/` 为单一 `docs/`（v4 设计文档放 `doc/`） |
| 6.8 | 更新 pybind11 绑定（使用 v4 术语） |
| 6.9 | 更新 AI_WORKING_SPEC.md |

**Phase 6 验收标准**：
- planner 目录不再为空
- 文档与代码同步，使用 v4 术语
- 编译通过

---

### Phase 7: 最终验证 + 性能回归（0.5 天）

| 步骤 | 动作 |
|---|---|
| 7.1 | 全量编译：`cmake --build . --config Release`（零 errors 零 warnings） |
| 7.2 | 全量测试：所有 test_* 通过 |
| 7.3 | Pipeline benchmark：运行 `exp_pipeline_benchmark`，对比 v3 数据确认无性能回归 |
| 7.4 | pysbf 绑定测试：Python import + 基本功能调用 |
| 7.5 | 论文检查：确认 paper 中的代码引用路径正确（PAPER_CODE_MAP） |
| 7.6 | 术语检查：全局 grep 确认无 "SubAABB"、"endpoint_aabb"、"frame_source" 残留（仅历史注释除外） |

---

## 5. 关键设计决策

### 5.1 关于 config.h 的处理

**决策**：将 `config.h` 瘦身，只留真正跨模块共享的类型。

- ✅ 保留：`StoreFormat`, `CollisionPolicy`, `EnvelopeConfig`（LECT FFB 仍在使用）
- ✂ 删除：旧 `sbf::SBFConfig`（被 `sbf::forest::SBFConfig` 取代）
- ✂ 删除：`ForestConfig`, `BridgeConfig`, `PlannerConfig`（被 `GrowerConfig` 取代）

### 5.2 关于 `envelope_derive.h/.cpp` 的处理

**现状**：`envelope_derive.h` 声明了 IFK 路径的 `derive_aabb_ifk()` 等函数。

**决策**：将 IFK 路径的 derive 函数合并到 `endpoint_source.cpp` 中（它们本质就是 EndpointSource::iFK 的 Stage 1 实现），CritSample/Analytical 路径拆到独立文件。函数名同步改为 `derive_iaabb_ifk()`。

### 5.3 关于 GridStore 与 VoxelGrid 的处理

**现状**：`GridStore`（定分辨率 R=32 字节网格）和 `VoxelGrid`（BitBrick 稀疏体素）两套方案并存。

**决策**：v4 只保留 `VoxelGrid`（BitBrick），删除 `GridStore` 和 `GridEnvelope`。原因：
- `VoxelGrid` 在所有维度上优于 `GridStore`（更紧凑、更快碰撞检测、无损合并）
- `GridStore` 仅在 v2 遗留的 `LinkIAABB_Grid`（原 `SubAABB_Grid`）路径中使用，但 v3 pipeline 中已通过 rasterise → VoxelGrid 实现

### 5.4 关于文件命名约定

| v3 文件名 | v4 文件名 | 原因 |
|---|---|---|
| `frame_source.h` | `endpoint_source.h` | 匹配核心概念 `EndpointSource` |
| `frame_store.h` | `endpoint_store.h` | 匹配 endpoint 术语 |
| `gcpc_cache.h` | `gcpc.h` | "cache" 是实现细节，不应出现在模块名中 |
| `envelope_derive_critical.h` | `crit_sample.h` + `analytical_solve.h` | 职责拆分 |
| `common/` | `core/` | 更准确（不是"公共"而是"核心"类型） |

### 5.5 关于 i-前缀命名惯例

**动机**：SafeBoxForest 的核心方法论是区间算术（Interval Arithmetic）。`iFK`（interval FK）已经是公认的缩写，但 v3 中 Stage 1 的输出仍叫 `endpoint_aabb`、Stage 2 的枚举仍叫 `SubAABB`，未体现"这些 AABB 是通过区间运算得到的"这一核心特征。

**决策**：
- 所有通过区间算术产出的 AABB 统一加 `i` 前缀：`iAABB`
- Stage 1 输出：`endpoint_iaabb`（取代 `endpoint_aabb`）
- Stage 2 连杆包络：`LinkIAABB`（取代 `SubAABB`），用 `sub=n` 参数化
- `Hull16_Grid` 不变（它基于凸包，不直接是 AABB）
- 这使得 pipeline 描述更自然：*iFK → endpoint iAABBs → link iAABBs → forest*

### 5.6 关于 Python 绑定

- v4 pybind11 绑定保持 `pysbf` 模块名（考虑兼容性，改为 `pysbf4` 或保持 `pysbf3`）
- 旧 enum/config binding（`StoreFormat`, `CollisionPolicy`, `ForestConfig` 等）如果还在 Python 端使用则保留，否则删除
- 枚举绑定使用 v4 名称：`LinkIAABB`, `LinkIAABB_Grid`, `Hull16_Grid`
- 新增 planner 类的绑定

---

## 6. 风险与对策

| 风险 | 概率 | 影响 | 对策 |
|---|---|---|---|
| 拆分 forest_grower 时破坏内部状态共享 | 中 | 高 | 拆分后用 `ForestGrower::Impl` pimpl 保持私有状态共享 |
| config.h 瘦身后遗漏某个旧字段的使用 | 低 | 中 | Phase 1 结束后 grep 全部旧结构名，确认无残留引用 |
| endpoint_source.h 重命名后脱离 git history | 低 | 低 | 使用 `git mv`，确保 blame 可追踪 |
| pybind11 绑定更新后破坏 Python 侧调用 | 中 | 中 | 保留旧模块名 + 编写迁移指南 |
| i-前缀大规模重命名导致遗漏 | 中 | 中 | Phase 2 结束后执行自动化检查：`grep -rn "SubAABB\|endpoint_aabb" --include="*.h" --include="*.cpp"`，确认零匹配 |

---

## 7. 时间线

| Phase | 内容 | 预计耗时 | 累计 |
|---|---|---|---|
| Phase 1 | 骨架搭建 + 死代码清除 | 1-2 天 | 1-2 天 |
| Phase 2 | 命名统一 + 别名清除 + i-前缀重命名 | 1-2 天 | 2-4 天 |
| Phase 3 | 大文件拆分 | 1-2 天 | 3-6 天 |
| Phase 4 | 命名空间整理 + 配置统一 | 0.5 天 | 3.5-6.5 天 |
| Phase 5 | 补充单元测试 | 1-2 天 | 4.5-8.5 天 |
| Phase 6 | Planner 骨架 + 文档更新 | 1 天 | 5.5-9.5 天 |
| Phase 7 | 最终验证 + 性能回归 | 0.5 天 | **6-10 天** |

---

## 8. 完整命名映射表（v3 → v4）

快速查阅用。涵盖所有需要在迁移时搜索替换的标识符。

### 8.1 枚举值

| v3 | v4 |
|---|---|
| `EnvelopeType::SubAABB` | `EnvelopeType::LinkIAABB` |
| `EnvelopeType::SubAABB_Grid` | `EnvelopeType::LinkIAABB_Grid` |
| `EnvelopeType::Hull16_Grid` | `EnvelopeType::Hull16_Grid`（不变） |

### 8.2 类型 / 结构体

| v3 | v4 |
|---|---|
| `EndpointAABBResult` | `EndpointIAABBResult` |
| `FrameSourceMethod` | ✂ 删除（已在 v3 中别名为 `EndpointSource`） |
| `FrameSourceConfig` | ✂ 删除（已在 v3 中别名为 `EndpointSourceConfig`） |
| `FrameSourceResult` | ✂ 删除（已在 v3 中别名为 `EndpointAABBResult`） |
| `sbf::EnvelopeResult` | ✂ 删除（仅保留 `sbf::envelope::EnvelopeResult`） |
| `sbf::IEnvelopeComputer` | ✂ 删除（v2 死代码） |
| `sbf::SBFConfig`（config.h） | ✂ 删除（仅保留 `sbf::forest::SBFConfig`） |

### 8.3 函数

| v3 | v4 |
|---|---|
| `compute_endpoint_aabb()` | `compute_endpoint_iaabb()` |
| `compute_frame_source()` | ✂ 删除（v3 别名） |
| `compute_envelope_repr()` | ✂ 删除（v3 别名） |
| `extract_link_aabbs_from_endpoint()` | `extract_link_iaabbs()` |
| `derive_aabb_ifk()` | `derive_iaabb_ifk()` |

### 8.4 工厂方法

| v3 | v4 |
|---|---|
| `EnvelopeTypeConfig::sub_aabb()` | `EnvelopeTypeConfig::link_iaabb()` |
| `EnvelopeTypeConfig::aabb()` | ✂ 删除（v3 别名） |
| `SBFConfig::ifk_sub_aabb()` | `SBFConfig::ifk_link_iaabb()` |
| `SBFConfig::crit_sub_aabb()` | `SBFConfig::crit_link_iaabb()` |
| `SBFConfig::analytical_sub_aabb()` | `SBFConfig::analytical_link_iaabb()` |
| `SBFConfig::gcpc_sub_aabb()` | `SBFConfig::gcpc_link_iaabb()` |
| `SBFConfig::ifk_aabb()` | ✂ 删除（v3 别名） |
| `SBFConfig::crit_aabb()` | ✂ 删除（v3 别名） |
| `SBFConfig::analytical_aabb()` | ✂ 删除（v3 别名） |
| `SBFConfig::gcpc_aabb()` | ✂ 删除（v3 别名） |

### 8.5 文件名

| v3 | v4 |
|---|---|
| `frame_source.h/.cpp` | `endpoint_source.h/.cpp` |
| `frame_store.h/.cpp` | `endpoint_store.h/.cpp` |
| `gcpc_cache.h/.cpp` | `gcpc.h/.cpp` |
| `envelope_derive_critical.h/.cpp` | `crit_sample.h/.cpp` + `analytical_solve.h/.cpp` |
| `envelope.h` | ✂ 删除 |
| `envelope_computer.h/.cpp` | ✂ 删除 |
| `grid_envelope.h/.cpp` | ✂ 删除 |
| `grid_store.h/.cpp` | ✂ 删除 |
| `hybrid_aabb.h` | ✂ 删除 |

### 8.6 变量 / 字段名（全局搜索替换）

| v3 模式 | v4 替换 |
|---|---|
| `endpoint_aabb` | `endpoint_iaabb` |
| `endpoint_aabbs` | `endpoint_iaabbs` |
| `sub_aabb` | `link_iaabb` |
| `n_sub` | `n_sub`（保留，作为 link iAABB 的细分参数） |

---

## 9. 补充说明

### 9.1 v3 将保持冻结
v3 代码保持原样不动，作为回退参考。v4 独立开发，不破坏 v3 的可编译性。

### 9.2 迁移时不改算法
本次重构**纯粹是架构清理 + 命名统一**，不涉及任何算法变更。所有 FFB descent、hull 碰撞、forest growing、pipeline 组合逻辑保持原样。

### 9.3 v3 未完成项在 v4 中的状态
以下 v3 未实施的优化项将在 v4 架构清理完成后视情况推进：

| 项目 | v4 状态 |
|---|---|
| P1-6: fill_hull16 scanline 缓存优化 | 暂缓（高风险） |
| P3-2: AVX2/512 加速 BitBrick 操作 | 待评估 |
| P3-3: Per-link 稀疏 grid 存储 | 待评估 |
| P3-4: 关联端点对（correlated endpoint pairs） | 待评估 |
| P3-5: 二级分辨率碰撞 grid | 待评估 |
| Planner 实现 | Phase 6 建骨架，后续迭代实现 |
