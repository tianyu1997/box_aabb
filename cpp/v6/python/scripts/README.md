# python/scripts

这里保留的是较早期的 Python 批处理和演示脚本层。它们仍然可用，但已经不是当前 v6 论文实验的主入口。

## 目录角色

- `run_all_experiments.py`, `run_benchmark.py`: 旧式整批实验驱动。
- `gen_tables.py`, `gen_figures.py`, `gen_paper_tables.py`: 旧式论文表格和图生成入口。
- `run_*`, `run_viz_demo.py`: 局部实验或演示脚本。

## 当前建议

- 当前论文 rerun 优先使用 `../../experiments/paper/`。
- 如果只是验证 Python 绑定、跑旧批处理或复用某个历史脚本片段，可以回到这里。
- 若脚本文案与当前 v6 目录结构不一致，以上层 README 和当前 paper 包装层为准。