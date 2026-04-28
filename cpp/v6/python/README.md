# v6 Python Layer

`cpp/v6/python` 保存 v6 的 Python 绑定与配套包。虽然工作区已经是 v6，但包名仍保持 `sbf5` 兼容命名；这是一项有意保留的兼容层，而不是未清理完的残留。

## 目录分层

- `sbf5/`: 主 Python 包，负责暴露 `_sbf5_cpp` 扩展模块和少量辅助函数。
- `sbf5_bench/`: Python 侧基线、统计与实验适配层。
- `sbf5_viz/`: Python 可视化入口与工具。
- `scripts/`: 旧式 Python 批处理/演示脚本层，现已退居次要位置。
- `tests/`: Python 侧测试。
- `sbf5_bindings.cpp`: pybind11 模块入口，生成 `_sbf5_cpp`。
- `CMakeLists.txt`: Python 扩展的 CMake 构建规则。

## 当前约定

- 构建出的扩展模块名是 `_sbf5_cpp`。
- Python import 名保持为 `sbf5`，不要在说明文档里把它误写成 `sbf6`。
- 当前论文实验入口不在这里，而在 `../experiments/paper/`；本目录提供的是绑定、基线适配和可视化支撑层。

## 使用建议

- 需要直接写 Python 调用、做 notebook 验证或运行 Python 测试时，从这里进入。
- 需要跑论文实验时，不要把 `python/scripts/` 当成默认入口，先看 `../experiments/paper/`。