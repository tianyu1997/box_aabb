# python/tests

这里是 v6 Python 绑定层的测试目录，属于活跃层。

## 目录角色

- `test_sbf6.py`: 主绑定与基础接口测试。
- `test_bench.py`, `test_benchmark_run.py`: Python 基线/实验适配层相关测试。
- `test_viz.py`: 可视化层测试。

## 当前约定

- 这里验证的是 `cpp/v6/python` 这层的 import、包装与辅助逻辑，不直接替代 C++ 单元测试。
- 包名继续保持 `sbf6` 兼容命名；测试名里出现 `sbf6` 是当前有效口径，不是待清理残留。
- 若论文实验脚本失败，需要先区分是绑定问题还是上层实验逻辑问题；这层测试主要用于排除绑定/包导入故障。