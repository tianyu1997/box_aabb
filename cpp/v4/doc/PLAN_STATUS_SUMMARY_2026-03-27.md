# v4 doc 计划实施状态汇总（2026-03-27）

## 一、已确认完成并已归档

已移动到 `doc/completed_plans/`：

1. `PLAN_BEST_TIGHTEN.md`
- 代码已落地 `SplitOrder::BEST_TIGHTEN`、`LECT::set_split_order()`、`LECT::pick_best_tighten_dim()`。
- 相关测试已在 `tests/test_ifk_pipeline.cpp` 中覆盖（`test_lect_best_tighten()`）。

2. `PLAN_ANALYTICAL_DERIVATIVE_REPLACE_QR.md`
- 文档里程碑 A-E 均标记 `✅ DONE`，并写明计划完成。
- 当前 `analytical_solve.cpp` 已为大体量完整实现，不再是 QR-only 的初始状态。

3. `PLAN_ANALYTICAL_ENDPOINT.md`
- `EndpointSource::Analytical` 在 `src/envelope/endpoint_source.cpp` 中已直接调用 `derive_aabb_critical_analytical()`。
- 非回退路径，输出已按 endpoint paired layout 写回。

---

## 二、进行中/部分完成计划（暂不归档）

1. `OPTIMIZATION_PLAN_EPIAABB.md`
- 已完成：O1/O2/O3/O4/O5/O6。
- 暂缓：O7（LinkExtremes 定长数组）。

2. `OPTIMIZATION_PLAN_GCPC.md`
- 已完成：G1-G8、H1-H3。

3. `PLAN_ANALYTICAL_NEXT.md`
- 文档里有大量“已完成”条目，但仍列出后续 P1-P11 改动项与技术债；属于下一阶段执行单。

4. `PLAN_ANALYTICAL_OPTIMIZED.md`
- 含大量已执行结果记录，但仍有后续实施段落与阶段目标，未达到“全文收口”。

5. `MIGRATION_ENDPOINT_IAABB.md`
- 迁移主链路大部分已在代码体现，但文档本身为大迁移总计划，未明确写“全阶段收口完成”。

6. `PLAN_IFK_PIPELINE.md`
- iFK 主链路、LECT 与 BEST_TIGHTEN 已在代码中落地。
- 文档仍保留后续工作项（如更高阶段扩展/实验等），按“未完全收口”处理。

7. `PLAN_JOINT_SYMMETRY_CACHE.md`
- 文档状态为 Draft；当前 `lect.cpp` 已有 Z4 对称缓存相关实现。
- 但按文档定义仍属于“部分落地，未正式收口”。

---

## 三、待执行计划（当前代码下未完整实施）

1. `OPTIMIZATION_PLAN_ANALYTICAL_SPEED.md`
- 主要是分析与建议清单（S1-S9、H6-H14 等），并非已完成实施记录。

2. `v4_refactoring_plan.md`
- 顶层重构蓝图，跨度大，仍包含大量中长期目标，未收口。

---

## 四、建议的下一步执行顺序

1. 推进 `OPTIMIZATION_PLAN_EPIAABB.md` 的 O7（`LinkExtremes` 定长数组）作为下一项堆分配清理重点。
2. 对 `PLAN_ANALYTICAL_NEXT.md` 做一次“已完成项清洗”，把过期条目迁移到结项记录，保留真正未完成任务。
