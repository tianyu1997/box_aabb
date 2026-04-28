# v6 Experiments

`cpp/v6/experiments` 是 v6 的实验工作区：C++ 实验可执行文件、论文重跑包装层、历史结果目录都集中在这里。

## 目录分层

- `paper/`: 当前论文口径的 Python 包装层与结果汇总入口。需要重跑论文实验时，优先从这里进入。
- `scripts/`: 较早期的 phase/stage 风格批处理脚本与基线复现辅助层。仍可用于诊断，但不是当前论文的第一入口。
- `doc/`: 历史实验计划与分项说明，保留作背景材料；其中部分命令和环境描述已经过时。
- `configs/`: 场景、查询对、实验配置。
- `exp*.cpp`: 当前 C++ 实验二进制的源码入口。
- `results_paper/`: 当前论文使用的结果目录。
- `results_*`: 各轮 rerun、quickcheck、对照实验、临时验证输出；默认视为历史结果，不自动当作论文权威结果。

## 当前活跃入口

- `paper/run_all.py`: 当前 v6 论文实验总入口。
- `paper/03_marcucci_envelope_build.py`: Marcucci 包络构建对比入口。
- `paper/04_e2e_baselines_combined.py`: Exp.3 的 paper-facing SBF cached-query 入口。
- `paper/06_sbf_parameter_scan.py`: 参数扫描入口。
- `paper/07_update_paper_results.py`: 用结果 JSON 更新论文表格与宏。

## 当前约定

- 论文 rerun 默认只在 `cpp/v6` 内进行；`cpp/v7` 仅用于明确要求的诊断或历史对比。
- 当前 paper 管线已停用 LinkIAABB-Grid；活跃比较只保留 `LinkIAABB` 与 `Hull16_Grid`。
- Exp.3 的权威 SBF build/query 实现在 `../scripts/run_online_query_comparison.py`，`paper/04_e2e_baselines_combined.py` 只是论文口径包装层。
- 若只是查看目录状态，优先看本 README；若要看论文脚本职责，再看 `paper/README.md`。
