# v7 迁移总览

> **目标**: 将 v6 重构为 v7，作为 TRO 最终投稿版本和代码开源版本。
> 本文档是所有迁移文档的入口，记录三个架构决策和 8 个阶段的总体状态。
>
> **创建时间**: 2026-05  
> **最后更新**: 2026-05

---

## 已解决的三个架构决策

### D1 — Radii=0 Envelope Cache ✅

**背景**: v6 的 LECT cache 以含 `link_radii` 的 AABB 作为 key，
导致更换安全余量配置时整个缓存失效。

**决策**: 缓存以 `link_radii=0` 计算的原始 AABB（纯扫掠几何）；
碰撞检测时在线 Minkowski 膨胀。

**影响模块**:
| 模块 | 变化 |
|------|------|
| P1 EndpointSource | `compute()` 输出零半径链，不接受 link_radii 参数 |
| P2 CollisionChecker | `check()` 接受 `CollisionConfig{link_radii}` 参数 |
| P2 BoxNode | 删除 `link_radii` 字段 |
| P3 CacheKey | 删除 `link_radii` 字段（同构型共享缓存） |
| P4 GrowerThread | 调用 `check()` 时传入 `coll_cfg` |

**收益**: 同一机器人不同安全余量配置共享 LECT 缓存，缓存命中率显著提升。

---

### D2 — 强制 Face-Overlap 桥接 ✅

**背景**: v6 的桥接 box 可以与父节点仅点切/边切，
导致 GCS 路径在桥接处出现细 "缝"，影响路径平滑性。

**决策**: 桥接 box 必须与父 leaf-box 有正面积面重叠（`face_overlap_area ≥ ε_face`）。
若连续 8 次 REJECT，细分父节点（沿最短轴），最多递归 3 次。

**影响模块**:
| 模块 | 变化 |
|------|------|
| P0 sbf_util | 新增 `has_face_overlap()`, `face_overlap_area()` |
| P4 BridgeBuilder | `try_extend()` 含面重叠检查 |
| P4 SubdivisionPolicy | 新类，处理细分逻辑 |

**收益**: 消除 GCS 路径中的桥接缝隙，路径质量改善（预期 &lt;5% path_length 改善）。

---

### D3 — CI `--quick` + 夜间全量 ✅

**背景**: v6 的 CI 运行完整实验（~2 小时），导致 PR 反馈慢。

**决策**: CI gate 仅跑 `--quick`（≤5 分钟，3 trials per experiment）；
夜间机器（self-hosted）每天运行全量实验（20 trials per experiment）。

**影响模块**:
| 模块 | 变化 |
|------|------|
| P5 SbfPlanner | 支持 `quick_mode` flag（精简参数） |
| P6 实验脚本 | 每个脚本均有 `--quick` CLI flag |
| P6 CI yaml | quick job（≤8min) + nightly job（≤8h） |

**收益**: CI 反馈时间从 ~2 小时降至 ~8 分钟。

---

## 8 阶段总览

| Phase | 名称 | 核心产出 | 状态 | 依赖 |
|-------|------|---------|------|------|
| P0 | 骨架建立 | `cpp/v7/` 目录结构 + CMake + sbf_util + smoke_P0 | ⏳ 待开始 | 无 |
| P1 | EPI-AABB | AffineChain + UrdfEndpointSource（零半径） + smoke_P1 | ⏳ 待开始 | P0 |
| P2 | Link Envelope | CollisionChecker（在线膨胀）+ Z4Voxel + BoxNode + smoke_P2 | ⏳ 待开始 | P1 |
| P3 | LECT | 真惰性加载 + OOM守卫 + SubtreeLease + CacheKey无radii + smoke_P3 | ⏳ 待开始 | P1, P2 |
| P4 | Grower | face-overlap桥接 + 子树隔离 + 严格父邻接 + regression_P4 | ⏳ 待开始 | P3 |
| P5 | Planner | 状态机 + GcsGraphBuilder + PathOptPipeline + regression_P5 | ⏳ 待开始 | P4 |
| P6 | 实验系统 | 11 脚本 + --quick + 夜间CI + build_tables.py | ⏳ 待开始 | P5 |
| P7 | 论文发布 | 最终论文 + README + LICENSE + clean build | ⏳ 待开始 | P6 |

---

## 文档索引

| 文档 | 内容 |
|------|------|
| [00_ai_principles.md](00_ai_principles.md) | AI 工作原则手册（强制约束） |
| [P0_skeleton.md](P0_skeleton.md) | P0 详细计划 |
| [P1_epi_aabb.md](P1_epi_aabb.md) | P1 详细计划 |
| [P2_link_envelope.md](P2_link_envelope.md) | P2 详细计划 |
| [P3_lect.md](P3_lect.md) | P3 详细计划 |
| [P4_grower.md](P4_grower.md) | P4 详细计划 |
| [P5_planner.md](P5_planner.md) | P5 详细计划 |
| [P6_experiments.md](P6_experiments.md) | P6 详细计划 |
| [P7_paper_release.md](P7_paper_release.md) | P7 详细计划 |

---

## v6 → v7 模块对应关系

| v6 模块/文件 | v7 替代 | 处理方式 |
|-------------|---------|---------|
| `sbf_common.h` | 拆分到各子模块头文件 | P1 重写 |
| `sbf_robot.cpp` | `src/core/affine_chain.cpp` | P1 重写 |
| `sbf_envelope.cpp` | `src/envelope/collision_checker.cpp` | P2 重写 |
| `sbf_voxel.cpp` | `src/envelope/z4_voxel.cpp` | P2 迁移 |
| `sbf_lect.cpp`（~2800 LOC）| `src/lect/*.cpp`（6 文件） | P3 全新实现 |
| `grower_coordinated.cpp`（1200 LOC 单函数）| `src/grower/*.cpp`（6 文件）| P4 重写拆分 |
| `sbf_planner.cpp`（2027 LOC 3 方法）| `src/planner/*.cpp`（4 文件）| P5 重写状态机 |
| `sbf_ffb.cpp` | 直接迁移（无重大改动） | P4 使用 |
| `sbf_forest.cpp` | 直接迁移 | P4 使用 |
| 实验脚本（~30 个）| `scripts/run_exp_*.py`（11 个）| P6 整合 |
| `build_b1_b3_b5_tables.py` | `scripts/build_tables.py` | P6 统一 |

---

## 关键指标目标

| 指标 | v6 基准 | v7 目标 |
|------|---------|---------|
| IIWA14 SR | 100% | 100% |
| Panda SR | 100% | 100% |
| IIWA14 build_time | 1.38s | ≤ 1.52s（+10%） |
| Memory peak (100K nodes) | ~8GB | ≤ 2GB |
| CI 反馈时间 | ~120min | ≤ 8min |
| 最大单函数 LOC | 1200 | 500 |
| 最大单文件 LOC | 2800 | 600 |

---

## 技术债清零验证

以下 v6 已知 debt 在 v7 中全部消除：

- [ ] ❌ 假 Lazy Load → ✅ 真惰性（P3）
- [ ] ❌ CacheKey 含 link_radii → ✅ 移除（P3, D1）
- [ ] ❌ OOM 存入后检查 → ✅ 存入前检查（P3）
- [ ] ❌ 单函数 1200 行 → ✅ 每函数 ≤ 500 行（P4）
- [ ] ❌ 桥接仅点切/边切 → ✅ 强制面重叠（P4, D2）
- [ ] ❌ GCS 跨层边 → ✅ 严格父邻接（P4, P5）
- [ ] ❌ 全局写锁 ms 级 → ✅ SubtreeLease 短锁 &lt;1ms（P3, P4）
- [ ] ❌ CI 运行 2 小时 → ✅ CI ≤ 8 分钟（P6, D3）

---

*本文件自动维护，每个 Phase 完成后更新对应行的状态。*
