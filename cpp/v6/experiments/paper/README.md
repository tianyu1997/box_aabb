# v6 Paper Experiment Entrypoints

All paper-facing experiments are run from `cpp/v6`. Do not route paper runs
through `cpp/v7`.

## Active scripts

- `01_epiaabb_pipeline.py`: Exp.1 endpoint iAABB source pipeline.
- `02_link_envelope_pipeline.py`: Exp.2 link envelope pipeline.
- `03_marcucci_envelope_build.py`: Exp.3 same-route EP/Grid cache replay.
- `04_e2e_baselines_combined.py`: Exp.4 SBF build/query row.
- `04_baselines_marcucci.py`: Exp.4 v6-native IRIS/GCS and OMPL baselines.
- `05_random_robot_scenes.py`: Exp.5 random UR5/Panda scenes.
- `07_update_paper_results.py`: regenerate paper tables/macros.
- `09_generate_paper_figures.py`: regenerate paper figures.
- `run_all.py`: section-order orchestrator for active experiment wrappers.
  Its defaults encode the current paper rerun profile: Exp.1 uses 1000 boxes
  per width bin, and Exp.5 measures one UR5 and one Panda blocked random scene
  with five baseline seeds in full mode. Exp.4 baselines are launched through
  the v6-native wrapper with IRIS-NP at 800 s, IRIS-ZO at 120 s, PRM with OMPL
  simplification counted in query time, and BIT* at a fixed 2 s path-quality
  budget.

## Legacy or diagnostic scripts

These remain in place for archaeology and diagnostics, but are not paper-facing
unless explicitly requested: `02_5_marcucci_envelope_build.py`,
`02_6_link_envelope_growth_calibration.py`, `03_1_marcucci_envelope_budget_scan.py`,
`03_e2e_baselines_combined.py`, `06_sbf_parameter_scan.py`, and
`08_build_ablation_sweep.py`.

`05_v7_baselines_migrated.py` is a compatibility alias that forwards to
`04_baselines_marcucci.py`; it no longer calls v7.

## Resource policy

- Paper runs use 16 logical threads and CPU affinity `0-15`.
- Seeds are executed serially to avoid resource contention.
- The Exp.4 baseline wrapper launches IRIS-NP, IRIS-ZO, PRM, and BIT* as
  sequential child commands so heavyweight baselines do not share CPU cores.
- LECT caches are preserved; do not delete them during paper reruns.

## Metrics fixed for Exp.3-5

- Exp.3 reports complete build time, including cache fill/hit bookkeeping, and
  same-route cache hit replay. The box-volume statistic is the deduplicated box
  volume sum. This is an envelope/cache replay measurement for one route setup,
  not a full SBF forest build metric and should not be compared directly with
  Exp.4/5 SBF build times.
- Exp.4 SBF build time includes build-time prebridging for the five paper query
  pairs; query time is cached query after build. PRM query time includes the
  second solve and OMPL `simplifySolution`. BIT* uses a fixed 2 s wall-clock
  path-quality budget. IRIS/GCS paths are validated with Drake edge collision
  checks before being counted as successes; invalid GCS path segments are
  locally repaired with collision-checked RRT-Connect and the repair time is
  counted in query time. The IRIS-ZO runner seeds route-interpolated and
  jitter-repaired collision-free configurations so the 120 s budget is spent on
  useful regions rather than only anchor/midpoint regions.
- Exp.5 records SBF build and query separately after an untimed per-scene LECT
  prewarm. Its non-SBF method rows are local reproducible proxies for random
  URDF scenes, not the Marcucci OMPL/IRIS baselines. PRM build is sampling plus
  roadmap construction and PRM query is Dijkstra plus shortcut; IRIS/GCS proxy
  build is skeleton generation plus local region inflation and query is
  shortest-chain extraction plus shortcut.

## Package note

这里是当前 v6 论文实验的 Python 包装层。它负责把底层实验实现组织成论文口径的输入输出，而不是承载所有底层逻辑本身。

## 当前入口

- `run_all.py`: 当前论文实验总入口。
- `02_link_envelope_pipeline.py`: Exp.2 link-envelope pipeline 入口。
- `03_marcucci_envelope_build.py`: Exp.3 Marcucci same-route 包络/cache build replay 入口。
- `04_e2e_baselines_combined.py`: Exp.4 Marcucci SBF cached-query workload 与 baseline 表源入口。
- `05_random_robot_scenes.py`: Exp.5 随机 UR5/Panda 场景生成入口；保存可复用场景 JSON，并可显式开启 SBF build/query 测量。
- `06_sbf_parameter_scan.py`: 参数扫描入口。
- `07_update_paper_results.py`: 用结果 JSON 更新论文表格与宏。

## 共享层

- `common.py`: 路径约定、build 目录解析、Python 扩展检查、结果写出等共享工具。
- `exp5_scene_utils.py`: Exp.5 的 DH FK、AABB 碰撞校验、工作空间采样与场景约束工具。
- `results_frozen/`: 冻结快照或对照输出，用于防止后续脚本改写时丢失历史口径。

## 命名说明

- 当前论文实验编号严格跟随脚本名前缀：`02_*` 是 Exp.2，`03_*` 是 Exp.3，`04_*` 是 Exp.4，`05_*` 是 Exp.5。
- `02_5_marcucci_envelope_build.py` 是历史兼容入口；当前 Exp.3 请使用 `03_marcucci_envelope_build.py`。
- `03_e2e_baselines_combined.py` 是历史兼容入口；当前 SBF cached-query 与 baseline 表源请使用 `04_e2e_baselines_combined.py`。
- 当 paper 脚本只是包装层时，权威实现通常位于上一级 `../scripts/` 或同目录共享模块。Exp.3 的 same-route replay 实现在 `marcucci_envelope_build_replay.py`；Exp.4 的权威 SBF build/query 实现在 `../../scripts/run_online_query_comparison.py`。

## 使用原则

- 需要重跑论文实验时，优先从这里进入，而不是直接调用历史 phase 脚本。
- 需要追底层实现时，再跳到 `../../scripts/` 或 C++ 实验二进制。
- 需要判断某个文件是否仍活跃时，先查 `run_all.py` 是否还在调用它。