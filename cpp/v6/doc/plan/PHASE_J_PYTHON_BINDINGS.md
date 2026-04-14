# Phase J: Python Bindings (pybind11)

> 依赖: Phase A-H (全部核心模块)
> 状态: ✅ 已完成
> 产出: `_sbf5_cpp` Python 扩展模块 — 暴露 Robot, ForestGrower, SBFPlanner 等核心类到 Python
> 预计: ~600 LOC (bindings) + CMake 配置

---

## 目标

通过 pybind11 将 v5 C++ 核心暴露给 Python，使：
- Python 可以直接调用 `SBFPlanner.plan()` 进行规划
- 可视化层 (Phase K) 可以在线交互而非依赖 JSON 文件
- 与 Drake Python API / OMPL 基线 (Phase M) 在同一 Python 环境中比较
- 支持 Jupyter Notebook 开发和实验

### 与 v4 的差异
| 方面 | v4 | v5 |
|------|----|----|
| 模块名 | `_sbf4_cpp` | `_sbf5_cpp` |
| Robot 来源 | DH 参数手动传入 | `Robot::from_json()` 直接加载 |
| Pipeline 配置 | 多层嵌套 Config | 简化为 `SBFPlannerConfig` |
| GIL 管理 | 仅 `grow()` 释放 | `plan()` 和 `build()` 均释放 |

---

## 管线概览

```
Python
    ↓
import sbf5                    # Python wrapper package
    ↓
sbf5._sbf5_cpp                 # pybind11 extension module
    ↓
sbf5 C++ library (libsbf5.a)
```

---

## Step J1: CMake pybind11 集成

### CMakeLists.txt 变更
```cmake
option(SBF_BUILD_PYTHON "Build Python bindings (pybind11)" OFF)

if(SBF_BUILD_PYTHON)
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
    include(FetchContent)
    FetchContent_Declare(pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG        v2.12.0
        SOURCE_DIR     "${CMAKE_SOURCE_DIR}/_sbf5_deps/pybind11"
    )
    FetchContent_MakeAvailable(pybind11)

    pybind11_add_module(_sbf5_cpp python/sbf5_bindings.cpp)
    target_link_libraries(_sbf5_cpp PRIVATE sbf5)
    target_include_directories(_sbf5_cpp PRIVATE include)

    # 安装到 python/sbf5/ 目录
    install(TARGETS _sbf5_cpp DESTINATION python/sbf5)
endif()
```

### 构建命令
```bash
cmake -B build_x64 -DSBF_BUILD_PYTHON=ON
cmake --build build_x64 --config Release --target _sbf5_cpp
```

---

## Step J2: 核心类型绑定

### 文件
`python/sbf5_bindings.cpp`

### 绑定: Interval
```cpp
py::class_<Interval>(m, "Interval")
    .def(py::init<double, double>())
    .def_readwrite("lo", &Interval::lo)
    .def_readwrite("hi", &Interval::hi)
    .def("width", &Interval::width)
    .def("center", &Interval::center)
    .def("__repr__", [](const Interval& iv) {
        return "[" + std::to_string(iv.lo) + ", " + std::to_string(iv.hi) + "]";
    });
```

### 绑定: Robot
```cpp
py::class_<Robot>(m, "Robot")
    .def_static("from_json", &Robot::from_json, py::arg("path"))
    .def("n_joints", &Robot::n_joints)
    .def("n_links", &Robot::n_links)
    .def("name", &Robot::name)
    .def("joint_lo", &Robot::joint_lo)
    .def("joint_hi", &Robot::joint_hi)
    .def("fk_positions", &Robot::fk_positions, py::arg("q"));
```

### 绑定: BoxNode
```cpp
py::class_<BoxNode>(m, "BoxNode")
    .def_readonly("id", &BoxNode::id)
    .def_readonly("intervals", &BoxNode::intervals)
    .def("center", &BoxNode::center)
    .def("volume", &BoxNode::volume);
```

### 绑定: Obstacle
```cpp
py::class_<Obstacle>(m, "Obstacle")
    .def(py::init<>())
    .def_readwrite("center", &Obstacle::center)
    .def_readwrite("half_sizes", &Obstacle::half_sizes)
    .def("min_point", [](const Obstacle& o) { return o.center - o.half_sizes; })
    .def("max_point", [](const Obstacle& o) { return o.center + o.half_sizes; });
```

---

## Step J3: SBFPlanner 绑定

### 绑定: SBFPlannerConfig
```cpp
py::class_<SBFPlannerConfig>(m, "SBFPlannerConfig")
    .def(py::init<>())
    .def_readwrite("grower", &SBFPlannerConfig::grower)
    .def_readwrite("smoother", &SBFPlannerConfig::smoother)
    .def_readwrite("use_gcs", &SBFPlannerConfig::use_gcs);
```

### 绑定: GrowerConfig
```cpp
py::class_<GrowerConfig>(m, "GrowerConfig")
    .def(py::init<>())
    .def_readwrite("n_roots", &GrowerConfig::n_roots)
    .def_readwrite("max_boxes", &GrowerConfig::max_boxes)
    .def_readwrite("min_edge", &GrowerConfig::min_edge)
    .def_readwrite("mode", &GrowerConfig::mode);

py::enum_<GrowerConfig::Mode>(m, "GrowerMode")
    .value("Wavefront", GrowerConfig::Mode::Wavefront)
    .value("RRT", GrowerConfig::Mode::RRT);
```

### 绑定: PlanResult
```cpp
py::class_<PlanResult>(m, "PlanResult")
    .def_readonly("success", &PlanResult::success)
    .def_readonly("path", &PlanResult::path)
    .def_readonly("box_sequence", &PlanResult::box_sequence)
    .def_readonly("path_length", &PlanResult::path_length)
    .def_readonly("planning_time_ms", &PlanResult::planning_time_ms)
    .def_readonly("n_boxes", &PlanResult::n_boxes);
```

### 绑定: SBFPlanner
```cpp
py::class_<SBFPlanner>(m, "SBFPlanner")
    .def(py::init<const Robot&, const SBFPlannerConfig&>(),
         py::arg("robot"), py::arg("config") = SBFPlannerConfig{})

    .def("plan", [](SBFPlanner& self,
                    const Eigen::VectorXd& start,
                    const Eigen::VectorXd& goal,
                    const std::vector<Obstacle>& obstacles,
                    double timeout_ms) {
        py::gil_scoped_release release;  // 释放 GIL 允许 C++ 多线程
        return self.plan(start, goal,
                        obstacles.data(), (int)obstacles.size(),
                        timeout_ms);
    }, py::arg("start"), py::arg("goal"),
       py::arg("obstacles"), py::arg("timeout_ms") = 30000.0)

    .def("build", [](SBFPlanner& self,
                     const Eigen::VectorXd& start,
                     const Eigen::VectorXd& goal,
                     const std::vector<Obstacle>& obstacles,
                     double timeout_ms) {
        py::gil_scoped_release release;
        self.build(start, goal,
                  obstacles.data(), (int)obstacles.size(),
                  timeout_ms);
    }, py::arg("start"), py::arg("goal"),
       py::arg("obstacles"), py::arg("timeout_ms") = 30000.0)

    .def("query", &SBFPlanner::query)
    .def("clear_forest", &SBFPlanner::clear_forest)
    .def("boxes", &SBFPlanner::boxes, py::return_value_policy::reference_internal)
    .def("n_boxes", &SBFPlanner::n_boxes);
```

---

## Step J4: Eigen ↔ NumPy 转换

### 依赖
pybind11 内置 `pybind11/eigen.h` 自动处理 `Eigen::VectorXd` ↔ `np.ndarray` 转换。

```cpp
#include <pybind11/eigen.h>
#include <pybind11/stl.h>       // std::vector ↔ list
#include <pybind11/stl_bind.h>  // opaque bindings if needed
```

### 注意事项
- `Eigen::VectorXd` 默认按值拷贝到 NumPy (安全但非零拷贝)
- `std::vector<Obstacle>` 通过 `pybind11/stl.h` 自动转换
- `PlanResult.path` (vector<VectorXd>) → list of np.ndarray

---

## Step J5: Python 包装层

### 文件
`python/sbf5/__init__.py`

```python
"""SafeBoxForest v5 — Python interface."""
from sbf5._sbf5_cpp import (
    Interval, Robot, BoxNode, Obstacle,
    SBFPlanner, SBFPlannerConfig, GrowerConfig, GrowerMode,
    PlanResult,
)

__version__ = "5.0.0"

__all__ = [
    "Interval", "Robot", "BoxNode", "Obstacle",
    "SBFPlanner", "SBFPlannerConfig", "GrowerConfig", "GrowerMode",
    "PlanResult",
]
```

### 文件
`python/sbf5/helpers.py`

```python
"""便捷函数."""
import numpy as np
from sbf5 import Robot, SBFPlanner, Obstacle

def quick_plan(robot_json: str, start, goal, obstacles_list, **kwargs):
    """一行快速规划.

    obstacles_list: list of dict {"center": [x,y,z], "half_sizes": [hx,hy,hz]}
    """
    robot = Robot.from_json(robot_json)
    planner = SBFPlanner(robot)
    obs = [Obstacle() for _ in obstacles_list]
    for o, d in zip(obs, obstacles_list):
        o.center = np.array(d["center"])
        o.half_sizes = np.array(d["half_sizes"])
    return planner.plan(np.array(start), np.array(goal), obs, **kwargs)
```

---

## Step J6: 测试

### C++ 编译测试
`test/test_bindings_compile.cpp` — 确保绑定代码无链接错误 (可选)

### Python 测试
`python/tests/test_sbf5.py`

| 用例 | 描述 |
|------|------|
| `test_robot_load` | `Robot.from_json("data/2dof_planar.json")` → 检查 n_joints == 2 |
| `test_interval` | `Interval(0, 1)` → width == 1, center == 0.5 |
| `test_plan_2dof` | 完整 plan() → 检查 success, path 非空, path_length > 0 |
| `test_obstacle_api` | 创建 Obstacle → 检查 min_point/max_point |
| `test_boxes_access` | plan() 后 boxes() 非空 |

### 运行
```bash
cd v5
python -m pytest python/tests/ -v
```

---

## 验收标准

- [x] `import sbf5` 成功 (需先 build `_sbf5_cpp`)
- [x] `SBFPlanner.plan()` 返回 `PlanResult` 且 `success == True`
- [x] GIL 正确释放 — `plan()` 和 `build()` 均使用 `py::gil_scoped_release`
- [x] Eigen ↔ NumPy 自动转换无报错
- [x] 9 个 Python 测试通过 (`pytest python/tests/ -v`)
- [x] `SBF_BUILD_PYTHON=OFF` (默认) 时不影响现有构建
