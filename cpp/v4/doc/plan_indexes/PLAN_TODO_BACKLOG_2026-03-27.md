# v4 待执行计划清单（2026-03-27）

> 目的：将当前所有“尚未闭环”的计划集中到一个单独文档，作为后续执行入口。

## 一、未启动/未完整实施计划

1. `OPTIMIZATION_PLAN_ANALYTICAL_SPEED.md`
- 状态：分析与建议为主，尚未形成完整实施记录。
- 关键缺口：S1-S9、H6-H14 为候选优化项，缺少逐项落地状态与验收结果。
- 建议动作：
  - 先按低风险高收益项试点（例如 S3/S4/S5）；
  - 每项落地后补 benchmark 对照与回归结论。

2. `v4_refactoring_plan.md`
- 状态：顶层重构蓝图，仍包含大量中长期目标。
- 关键缺口：尚未收口，存在跨模块未完成条目。
- 建议动作：
  - 按模块拆分为“短期可交付批次”；
  - 对已完成条目做回填，减少文档与代码状态偏差。

---

## 二、部分完成计划中的剩余待办

1. `OPTIMIZATION_PLAN_EPIAABB.md`
- 已完成：O1/O2/O3/O4/O5/O6。
- 剩余待办：
  - O7：`LinkExtremes` 定长数组化（暂缓）

2. `OPTIMIZATION_PLAN_GCPC.md`
- 已完成：G1-G8、H1-H3。
- 剩余待办：无（当前文档项已收口）

3. `PLAN_ANALYTICAL_NEXT.md`
- 剩余待办：P1-P11 后续改造与技术债清理。
- 说明：文档含大量已完成条目，建议先做一次“已完成项清洗”，再执行剩余项。

4. `PLAN_ANALYTICAL_OPTIMIZED.md`
- 剩余待办：后续实施段落尚未全部收口。

5. `MIGRATION_ENDPOINT_IAABB.md`
- 剩余待办：迁移总计划尚未声明全阶段收口，需补闭环确认。

6. `PLAN_IFK_PIPELINE.md`
- 剩余待办：文档末尾后续项仍在（高阶段扩展、实验等）。

7. `PLAN_JOINT_SYMMETRY_CACHE.md`
- 状态：Draft。
- 剩余待办：需要将已部分落地的代码行为与文档结论对齐，形成收口版本。

---

## 三、建议执行顺序（下一轮）

1. 推进 O7（`LinkExtremes` 定长数组化），继续清理热路径堆分配。
2. 清洗 `PLAN_ANALYTICAL_NEXT.md`：拆出“已完成归档”和“真实待办”两部分。
3. 对 `v4_refactoring_plan.md` 做阶段化拆分，形成 2-3 个可交付批次。
