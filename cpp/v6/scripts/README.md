# v6 Support Scripts

`cpp/v6/scripts` 放的是支撑层脚本：校验、绘图、基线复现、GCS 后处理、论文表格生成都在这里。它们很重要，但默认不作为论文实验的第一入口。

## 脚本分组

- `run_*.py`: 运行或补跑某个实验块、基线或诊断任务。
- `plot_*.py`: 从已有 JSON 结果生成图或辅助可视化。
- `gcs_*.py`: GCS 查询、后处理、路径优化与诊断。
- `check_*.py` / `verify_*.py`: 一致性、碰撞、盒内性等校验脚本。
- `regen_paper_tables.py`, `gen_tab4_baselines.py`, `build_b1_b3_b5_tables.py`: 论文表格生成与汇总。
- `old/`: 历史保留脚本，不作为默认入口。

## 当前活跃入口

- `run_online_query_comparison.py`: 当前论文 Exp.3 的权威 SBF build/query 实现。
- `run_baselines.py`: 论文基线复现入口之一。
- `check_paper_consistency.py`: 论文数字一致性检查。
- `regen_paper_tables.py`: 基于结果 JSON 重建论文表格。

## 使用边界

- 需要重跑论文实验时，优先从 `../experiments/paper/` 的包装层进入；那里负责论文口径、输出文件名和路径约定。
- 需要直接调底层逻辑、做局部诊断或单独出图时，再从这里调用具体脚本。
- 若目录里出现功能接近的脚本，优先看文件头 docstring 和这里列出的活跃入口，不要默认最新修改时间就代表权威口径。