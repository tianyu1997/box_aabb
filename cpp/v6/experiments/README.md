# Experiments

当前维护的论文实验全部留在 `cpp/v6`。

## 当前入口

- `experiments/paper/run_all.py`: 当前 v6 paper 实验总入口。
- `experiments/paper/03_marcucci_envelope_build.py`: Marcucci 包络构建对比。
- `experiments/paper/04_e2e_baselines_combined.py`: Exp.3 的 SBF cached-query 入口。
- `experiments/paper/07_update_paper_results.py`: 用结果 JSON 更新论文表格与宏。

## 结果目录

- `experiments/results_paper/`: 当前论文结果与中间 raw JSON。
- `experiments/paper/common.py`: 共享路径、构建目录与 Python 扩展检查。

## 当前约束

- 后续论文实验默认只在 `cpp/v6` rerun；`cpp/v7` 仅用于明确要求的诊断工作。
- 当前 paper 管线已经移除 LinkIAABB-Grid；活跃比较只保留 `LinkIAABB` 与 `Hull16_Grid`。
- Exp.3 的权威 SBF build/query 实现位于 `scripts/run_online_query_comparison.py`，paper-facing 包装层位于 `experiments/paper/04_e2e_baselines_combined.py`。
