# SafeBoxForest v3  AI Working Specification

> This file defines rules for both human and AI contributors.
> Every commit MUST comply with these rules.
> 本文件可随项目推进而修改完善（见第 9 条）。

---

## 1. Quad-Sync Rule (Code ↔ Paper ↔ API ↔ CN)

| Change type | Must also update |
|---|---|
| New/modified C++ function | `docs/API_REFERENCE.md` + `docs/API_REFERENCE_CN.md` + paper algorithm if applicable |
| Paper algorithm change | Corresponding C++ implementation + `paper/root_cn.tex` |
| New experiment result | `docs/PAPER_CODE_MAP.md` traceback entry |
| Any enum/struct/factory change | English doc + Chinese doc + paper (EN & CN) if mentioned |

> 中英文文档和论文必须同步更新。有 `_CN.md` 或 `_cn.tex` 后缀的文件是其英文版的翻译对应物。

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
- 当前活跃别名:
  - `EnvelopeTypeConfig::aabb()` → `sub_aabb()` (since v3, AABB→SubAABB merge)
  - `SBFConfig::ifk_aabb()` / `crit_aabb()` / `analytical_aabb()` / `gcpc_aabb()` → 对应 `*_sub_aabb()`
  - `FrameSourceMethod` → `EndpointSource`
  - `FrameSourceConfig` → `EndpointSourceConfig`
  - `compute_frame_source()` → `compute_endpoint_aabb()`
  - `compute_envelope_repr()` → `compute_link_envelope()`

## 6. Pipeline Architecture

Two-stage modular pipeline:

```
Stage 1 (EndpointSource): C-space intervals → endpoint_aabbs [n_endpoints × 6]
  {IFK, CritSample, Analytical, GCPC}  — 4 sources

Stage 2 (EnvelopeType):   endpoint_aabbs → LinkEnvelope
  {SubAABB, SubAABB_Grid, Hull16_Grid} — 3 representations

4 × 3 = 12 pipeline combinations.
```

> 历史: v3 早期有 AABB (=0) 和 SubAABB (=1) 两个不同枚举值，
> 已合并为统一 SubAABB（n_sub=1 时退化为单 AABB）。

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
| v3 | T-RO journal extension | **Active** |

> **Note**: v2 代码库在 IROS 投稿后已有大量改动（新 experiment、API 调整等），
> v3 应以 v2 **当前最新状态**为基准迁移，而非 IROS 提交时的快照。

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

## 12. Self-Amendment

本 spec 文件本身可以随项目推进而修改完善。
当发现规则不适用、缺失或需要细化时，应直接更新本文件并在 commit message 中注明。
