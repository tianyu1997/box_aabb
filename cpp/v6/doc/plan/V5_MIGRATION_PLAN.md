# SafeBoxForest v5 迁移总计划

> 创建日期: 2026-04-03
> 状态: 进行中 (Phase A–G 已完成, Phase H 待实施)

---

## 1. 背景

v4 的迁移因架构混乱（CMake 体系臃肿、dual-channel LECT 过度工程、4-tier GCPC 缓存等）而失败。
v5 从零搭建纯 C++17 简洁架构，从 v1-v4 历史版本中提取各模块**最优实现**，统一到清晰的 7 模块管线。

**核心原则**: **简洁 > 全面，可用 > 完美**

---

## 2. 架构总览

```
safeboxforest/v5/
├── CMakeLists.txt
├── include/sbf/
│   ├── core/           # 基础类型、区间、Robot、DH、FKState、JointSymmetry
│   ├── scene/          # CollisionChecker (SAT)
│   ├── envelope/       # Module 1+2: EndpointIAABB (4 source) + LinkIAABB (3 type)
│   ├── lect/           # Module 3: LECT KD-tree + envelope cache
│   ├── ffb/            # Module 4: Find Free Box
│   ├── forest/         # Module 5+6: Grower(RRT/Wavefront) + Coarsening
│   └── planner/        # Module 7: Dijkstra + GCS + Smoother + interface
├── src/                # 对应 .cpp 实现
│   ├── core/
│   ├── scene/
│   ├── envelope/
│   ├── lect/
│   ├── ffb/
│   ├── forest/
│   └── planner/
├── tests/              # doctest 单元测试 + 集成测试
├── configs/            # Robot JSON configs (panda.json, 2dof_planar.json 等)
├── doc/                # 计划文档、变更日志
│   ├── plan/           # 各 Phase 独立计划
│   ├── CHANGE_LOG_CN.md
│   └── AI_WORKING_SPEC.md
└── third_party/        # Eigen, nlohmann/json, doctest (FetchContent 或本地)
```

---

## 3. 两阶段管线

```
Stage 1 (EndpointSource): C-space intervals → endpoint_iaabbs [n_active × 2 × 6]
  {IFK, CritSample, Analytical, GCPC}  — 4 sources

Stage 2 (EnvelopeType):   endpoint_iaabbs → LinkEnvelope
  {LinkIAABB, LinkIAABB_Grid, Hull16_Grid} — 3 representations

4 × 3 = 12 pipeline combinations (Hull16_Grid 延后实现)
```

---

## 4. Phase 分解

| Phase | 名称 | 依赖 | 状态 | 计划文档 |
|-------|------|------|------|----------|
| **A** | 基础设施 (CMake + core + scene) | 无 | ✅ 已完成 | `doc/plan/PHASE_A_INFRASTRUCTURE.md` |
| **B** | Endpoint IAABB 管线 (Module 1) | A | ✅ 已完成 | `doc/plan/PHASE_B_ENDPOINT_IAABB.md` |
| **C** | Link IAABB 管线 (Module 2) | B | ✅ 已完成 | `doc/plan/PHASE_C_LINK_IAABB.md` |
| **D** | LECT (Module 3) | B+C | ✅ 已完成 | `doc/plan/PHASE_D_LECT.md` |
| **E** | FFB (Module 4) | D | ✅ 已完成 | `doc/plan/PHASE_E_FFB.md` |
| **F** | Forest Grower (Module 5) | E | ✅ 已完成 | `doc/plan/PHASE_F_GROWER.md` |
| **G** | Coarsening (Module 6) | F | ✅ 已完成 | `doc/plan/PHASE_G_COARSENING.md` |
| **H** | Planner (Module 7) | F+G | 待实施 | `doc/plan/PHASE_H_PLANNER.md` |

---

## 5. 关键决策

| 决策 | 理由 |
|------|------|
| 纯 C++17, 无 Python 绑定 | 保持清爽，后续按需添加 |
| 单通道 LECT | v4 dual-channel 几乎只用 SAFE 通道，复杂度收益不成比例 |
| GCPC 单层缓存 | v4 的 4-tier 过度工程 |
| 保留 Z4 对称性 | full-rotation joints 有显著性能收益 |
| Coarsening 以 v1 C++ 为基准 | v1 是唯一完整的 C++ tree-cached greedy 实现 |
| Hull16_Grid 延后 | 第一版聚焦核心管线 |
| doctest 测试框架 | header-only, 轻量 |
| Drake GCS 通过 find_package 可选 | fallback Dijkstra |
| CMake 全新搭建 | v4 体系混乱，抛弃 |

---

## 6. 排除范围（第一版不实现）

- Python 绑定 (pybind11)
- Hull16_Grid 完整实现（仅预留枚举）
- Voxel bit_brick 压缩
- 可视化模块 (viz/)
- Experiment/benchmark 套件
- HCACHE 持久化（可选延后）
- OMPL adapter

---

## 7. 验证策略

### 单元测试（每 Phase 完成时运行）
1. `test_core` — Interval 运算、IFK、Robot 加载
2. `test_endpoint_iaabb` — 4 种 source 正确性 + 保守性
3. `test_link_iaabb` — derive + grid 覆盖
4. `test_lect` — 树结构 + split + Z4 + 碰撞
5. `test_ffb` — search + fail codes + 缓存命中
6. `test_grower` — RRT + Wavefront + promotion
7. `test_coarsen` — sweep + greedy + tree-cached
8. `test_planner` — Dijkstra + smoother + end-to-end

### 集成测试
9. `test_full_pipeline` — 2DOF + 7DOF 完整管线

### 构建验证
- `cmake --build . --config Release` 零错误零警告 (MSVC + GCC)

---

## 8. 主要参考文件索引

| 模块 | 主要参考源 |
|------|-----------|
| Core/FK | v1 `v1/include/sbf/robot/interval_fk.h` + v4 `safeboxforest/v4/include/sbf/robot/interval_fk.h` |
| Collision | v1 `v1/include/sbf/scene/aabb_collision_checker.h` |
| Endpoint IAABB | v4 `safeboxforest/v4/include/sbf/envelope/{endpoint_source,analytical_solve,gcpc}.h` |
| Link IAABB | v4 `safeboxforest/v4/include/sbf/envelope/envelope_derive.h` |
| LECT | v4 `safeboxforest/v4/include/sbf/forest/lect.h` + `src/forest/lect.cpp` |
| Grower | v4 `safeboxforest/v4/src/forest/forest_grower.cpp` + v1 `v1/include/sbf/forest/forest_grower.h` |
| **Coarsening** | **v1 `v1/include/sbf/forest/coarsen.h` + `v1/src/forest/coarsen.cpp`** |
| Planner | v1 `v1/include/sbf/planner/{sbf_planner,graph_search,path_smoother}.h` + `v1/src/adapters/drake_gcs.cpp` |
