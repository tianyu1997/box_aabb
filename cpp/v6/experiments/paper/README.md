# experiments/paper

这里是当前 v6 论文实验的 Python 包装层。它负责把底层实验实现组织成论文口径的输入输出，而不是承载所有底层逻辑本身。

## 当前入口

- `run_all.py`: 当前论文实验总入口。
- `03_marcucci_envelope_build.py`: Marcucci 包络构建对比入口。
- `04_e2e_baselines_combined.py`: Exp.3 的 paper-facing SBF cached-query 入口。
- `06_sbf_parameter_scan.py`: 参数扫描入口。
- `07_update_paper_results.py`: 用结果 JSON 更新论文表格与宏。

## 共享层

- `common.py`: 路径约定、build 目录解析、Python 扩展检查、结果写出等共享工具。
- `results_frozen/`: 冻结快照或对照输出，用于防止后续脚本改写时丢失历史口径。

## 命名说明

- 本目录中保留了若干重复编号或近似命名的脚本，这是因为论文口径在多轮迭代中发生过调整。
- 若看到两个相近名字的脚本，不要仅凭编号判断新旧，优先以上面的“当前入口”列表为准。
- 当 paper 脚本只是包装层时，权威实现通常位于上一级 `../scripts/`。例如 Exp.3 的权威 SBF build/query 实现在 `../../scripts/run_online_query_comparison.py`。

## 使用原则

- 需要重跑论文实验时，优先从这里进入，而不是直接调用历史 phase 脚本。
- 需要追底层实现时，再跳到 `../../scripts/` 或 C++ 实验二进制。
- 需要判断某个文件是否仍活跃时，先查 `run_all.py` 是否还在调用它。