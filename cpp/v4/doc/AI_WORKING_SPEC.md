# SafeBoxForest v4 AI Working Specification

> This file defines rules for both human and AI contributors.
> Every commit MUST comply with these rules.
> 本文件可随项目推进而修改完善（见第 13 条）。

---

## 1. Sync Rule (Code ↔ Paper_EN/CN ↔ API_CN ↔ Log_CN)

| Change type | Must also update |
|---|---|
| New/modified C++ function | `docs/API_REFERENCE_CN.md` + `doc/CHANGE_LOG_CN.md` + paper algorithm if applicable |
| Paper algorithm change | Corresponding C++ implementation + `paper/root.tex`（或英文主论文文件） + `paper/root_cn.tex`（或中文主论文文件） + `doc/CHANGE_LOG_CN.md` |
| New experiment result | `docs/PAPER_CODE_MAP.md` traceback entry + `doc/CHANGE_LOG_CN.md` |
| Any enum/struct/factory change | Chinese doc + `doc/CHANGE_LOG_CN.md` + `paper/root.tex` / `paper/root_cn.tex` 对应内容（若论文提及） |

> v4 阶段默认维护**中文版 API / 说明文档**，不再强制要求同步更新英文版 API 文档。
> 但**英文论文与中文论文都需要维护**；凡是论文中提及的算法、术语、实验、结构或结论发生变化，必须同步更新中英文论文内容。
> 论文主文件默认记为 `paper/root.tex` 与 `paper/root_cn.tex`；若项目实际使用其他主文件名，则应更新对应的英文/中文主论文文件。
> 所有代码改动都必须同步记录到 `doc/CHANGE_LOG_CN.md`，说明本次实现了什么、改了什么、影响了哪些模块。

## 2. Experiment Traceability

Every figure / table in the paper MUST have a traceback entry in
`docs/PAPER_CODE_MAP.md` of the form:

```
Figure X → experiments/NNN_name.cpp → results/NNN_name/
```

## 3. Coding Standards

- **Language**: C++17, compiled with MSVC 2022 (Windows) and GCC 12+ (Linux).
- **Naming**: `snake_case` for functions/variables, `PascalCase` for types.
- **Headers**: `#pragma once`; include what you use.
- **Tests**: every public API function has a corresponding test in `tests/`.
- **Build**: every change MUST pass `cmake --build . --config Release` with **zero errors and zero warnings** before commit.

## 4. Enum & Serialisation Safety

- When **renumbering** enum values (e.g. removing an entry), old disk caches will have stale integer mappings.
  - 必须在 cache validation 中增加 name-based cross-check（参考 `envelope_cache.cpp: is_valid` 中 `envelope_name` 的做法）。
  - 在 `CacheMetadata` 中同时存储 numeric value 和 string name，load 时交叉比对。
- Enum 值变更后，所有 `for (int e = 0; e < N; ++e)` 循环的上界 `N` 必须同步更新。
- 所有 `static_cast<EnumType>(int)` 的调用处必须检查有效范围。

## 5. Backward-Compatible Aliases

- 合并或重命名 API 时，旧名称保留为 `inline` 别名并标注 `// Backward-compatible alias`。
- 别名在代码中保留不少于 **两个大版本**，之后可移除。
- v4 已清除的别名（v3 遗留，不再保留）：
  - `FrameSourceMethod` / `FrameSourceConfig` / `FrameSourceResult`（已删除，统一为 `EndpointSource` / `EndpointSourceConfig` / `EndpointIAABBResult`）
  - `compute_frame_source()`（已删除，统一为 `compute_endpoint_iaabb()`）
  - `compute_envelope_repr()`（已删除，统一为 `compute_link_envelope()`）
  - `EnvelopeTypeConfig::aabb()` / `sub_aabb()`（已删除，统一为 `link_iaabb()`）
  - `SBFConfig::ifk_aabb()` / `crit_aabb()` 等（已删除，统一为 `ifk_link_iaabb()` 等）
  - `extract_link_aabbs_from_result()` / `extract_link_aabbs_from_endpoint()`（已删除，统一为 `extract_link_iaabbs()`）
- 当前活跃别名:
  - （v4 初始阶段无活跃别名，所有旧别名已在 v3→v4 迁移时一次性清除）

## 6. Pipeline Architecture

Two-stage modular pipeline:

```
Stage 1 (EndpointSource): C-space intervals → endpoint_iaabbs [n_endpoints × 6]
  {iFK, CritSample, Analytical, GCPC}  — 4 sources

Stage 2 (EnvelopeType):   endpoint_iaabbs → LinkEnvelope
  {LinkIAABB, LinkIAABB_Grid, Hull16_Grid} — 3 representations

4 × 3 = 12 pipeline combinations.
```

### 命名惯例

凡通过区间算术（Interval Arithmetic）计算得到的 AABB，统一使用 `i` 前缀，与 `iFK` 保持一致：

| 语境 | 命名规则 | 示例 |
|---|---|---|
| 枚举值 / 类型（PascalCase） | `IAABB` | `EnvelopeType::LinkIAABB`, `EndpointIAABBResult` |
| 函数 / 变量（snake_case） | `iaabb` | `compute_endpoint_iaabb()`, `endpoint_iaabbs`, `link_iaabb()` |
| 参数化表达 | `link iAABB(sub=n)` | link iAABB(sub=1) = 单连杆 iAABB; link iAABB(sub=16) = 16 段细分 |

> 历史:
> - v3 早期有 AABB(=0) 和 SubAABB(=1) 两个不同枚举值，后合并为 SubAABB
> - v4 将 SubAABB 更名为 LinkIAABB，用 sub=n 参数化细分，废弃 "SubAABB" 称呼
> - v4 将 endpoint_aabb 更名为 endpoint_iaabb，与 iFK 命名惯例统一
> - v4 将 frame_source 更名为 endpoint_source，匹配 EndpointSource 概念

## 7. Paper Sections ↔ Code Modules

| Paper Section | Code Module | Header |
|---|---|---|
| Sec III: Interval Envelope & Caching | `envelope/` | `include/sbf/envelope/` |
| Sec IV: SBF Generation & Maintenance | `forest/` | `include/sbf/forest/` |
| Sec V: SBF Planner | `planner/` | `include/sbf/planner/` |

## 8. Version History

| Version | Target | Status |
|---|---|---|
| v1 | Prototype (Python + Cython) | Archived |
| v2 | IROS 2026 C++（已在 IROS 提交基础上持续迭代） | Evolving |
| v3 | T-RO journal extension | Archived baseline |
| v4 | v3 最新状态基础上的持续迁移与重构版本 | **Active** |

> **Note**: v2 代码库在 IROS 投稿后已有大量改动（新 experiment、API 调整等），
> v3 应以 v2 **当前最新状态**为基准迁移，而非 IROS 提交时的快照。
>
> **v4 Migration Rule**: v4 的开发与迁移应以 v3 **当前最新稳定状态**为直接基线，
> 所有迁移、重构、替换和新增能力都应在 `doc/CHANGE_LOG_CN.md` 中留下中文记录。

## 9. Example / Benchmark 编写规范

- 生成 example 或 benchmark 时，**只调用已有 public API**，不得在 example/benchmark 文件中重新实现算法逻辑。
- Example/benchmark 的职责是展示和度量，而非引入新实现。如果缺少所需功能，应先在核心库中实现并测试，再在 example 中调用。

## 10. 输出文件路径规范

- 所有实验/benchmark/example 的输出文件（结果、日志、缓存、图片等）必须保存到项目约定的目录中：
  - 实验结果 → `results/<exp_name>/`
  - Example 输出 → `examples/output/`
  - Benchmark 输出 → `benchmarks/output/` 或对应实验目录
- **禁止**将输出文件写到项目根目录、用户桌面、或其他非约定位置。
- 输出路径应使用相对路径或通过配置/参数指定，确保跨环境可移植。

## 11. 沟通语言

- 与用户的所有沟通一律使用**中文**。

## 12. 中文变更日志维护

- 项目必须维护中文变更日志文件：`doc/CHANGE_LOG_CN.md`。
- 每次修改代码时，必须在该日志中追加记录，至少包括：
  - 日期
  - 修改文件 / 模块
  - 实现或修复了什么内容
  - 是否影响 API、实验、缓存或行为语义
- 日志记录必须使用中文，要求能让后续开发者快速追踪一次改动的目的和影响。
- 若一次提交包含多项独立改动，应拆分为多条日志记录，避免写成笼统描述。

## 13. Self-Amendment

本 spec 文件本身可以随项目推进而修改完善。
当发现规则不适用、缺失或需要细化时，应直接更新本文件并在 commit message 中注明。