# P0 — 骨架建立（Skeleton Setup）

> **目标**: 创建 `cpp/v7/` 目录结构，迁移 CMake 构建系统，建立 CI 流水线，
> 保证"空模块可编译"通过 smoke test。
>
> **预计工时**: 1-2 天  
> **前置条件**: 无（本 Phase 是一切的起点）  
> **完成标准**: `cmake .. && make -j8 && ctest -R smoke_P0` 全部绿灯

---

## 0. 设计原则（本 Phase 特有）

v7 不是 v6 的增量补丁，是**全新代码库**，v6 作为只读参考。
本 Phase 只做脚手架，**不迁移任何业务逻辑**。

---

## 1. 目录结构

创建以下目录树（`create_directory` 或 `mkdir -p`）：

```
cpp/v7/
├── CMakeLists.txt
├── include/
│   └── sbf/            # 所有公共头文件放这里
├── src/
│   ├── core/           # P1: epi_aabb, affine_chain
│   ├── envelope/       # P2: link_envelope, voxel, z4
│   ├── lect/           # P3: lect, cache, subtree_lease
│   ├── grower/         # P4: grower_thread, bridge_builder
│   ├── planner/        # P5: sbf_planner, gcs_state_machine
│   └── util/           # 工具函数（无业务依赖）
├── test/
│   ├── smoke/          # smoke tests (≤60s each)
│   └── unit/           # unit tests
├── scripts/
│   ├── run_exp_*.py    # 实验脚本 (P6)
│   └── build_tables.py # 论文表格生成 (P7)
├── experiments/
│   ├── configs/        # YAML 配置文件
│   └── results_nightly/
├── doc/
│   ├── migration/      # 本目录（迁移文档）
│   └── generated/      # 论文表格 tex 文件（机器生成，勿手改）
└── python/
    └── sbf/            # pybind11 Python bindings
```

---

## 2. CMakeLists.txt 骨架

`cpp/v7/CMakeLists.txt` 最低需求：

```cmake
cmake_minimum_required(VERSION 3.18)
project(box_aabb_v7 VERSION 7.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ---- 依赖 --------------------------------------------------------
find_package(Eigen3 3.4 REQUIRED)
find_package(Drake REQUIRED)
find_package(GTest REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(pybind11 REQUIRED)

# ---- 选项 --------------------------------------------------------
option(BUILD_TESTS     "Build test suite"        ON)
option(BUILD_PYTHON    "Build Python bindings"   ON)
option(ENABLE_NIGHTLY  "Enable nightly experiments" OFF)

# ---- 公共 include ------------------------------------------------
include_directories(include)

# ---- 子模块（各 Phase 逐步添加）----------------------------------
add_subdirectory(src/util)
# add_subdirectory(src/core)     # P1
# add_subdirectory(src/envelope) # P2
# add_subdirectory(src/lect)     # P3
# add_subdirectory(src/grower)   # P4
# add_subdirectory(src/planner)  # P5

# ---- 测试 --------------------------------------------------------
if(BUILD_TESTS)
  enable_testing()
  add_subdirectory(test/smoke)
  add_subdirectory(test/unit)
endif()
```

**注**: 其他 `add_subdirectory` 行在对应 Phase 开始时取消注释，不允许提前添加。

---

## 3. src/util 骨架

这是唯一在 P0 实现的模块。包含零依赖工具函数，为后续模块提供基础。

### 3.1 头文件

**`include/sbf/util/math_util.h`**
```cpp
#pragma once
#include <Eigen/Core>

namespace sbf::util {

// 在 [lo, hi] 区间夹紧值
template<typename T>
T clamp(T val, T lo, T hi);

// 两个 AABB 的面重叠面积（沿 axis 方向）
// 返回 0 若不重叠
double face_overlap_area(
    const Eigen::AlignedBox3d& a,
    const Eigen::AlignedBox3d& b,
    int axis);  // 0=x, 1=y, 2=z

// 判断两个 AABB 是否有正面积面重叠（任一轴）
// 这是 B2 face-overlap 门槛检查的基础
bool has_face_overlap(
    const Eigen::AlignedBox3d& a,
    const Eigen::AlignedBox3d& b,
    double epsilon = 1e-6);

}  // namespace sbf::util
```

**`include/sbf/util/timer.h`**
```cpp
#pragma once
#include <chrono>

namespace sbf::util {

class Timer {
public:
    void start();
    double elapsed_ms() const;  // 毫秒
    double elapsed_s() const;   // 秒
private:
    std::chrono::steady_clock::time_point t0_;
};

}  // namespace sbf::util
```

**`include/sbf/util/result.h`**
```cpp
#pragma once
#include <variant>
#include <string>

namespace sbf::util {

// 轻量 Result<T, E> 类型，替代异常
template<typename T, typename E = std::string>
class Result {
public:
    static Result ok(T val);
    static Result err(E msg);
    bool is_ok() const;
    T& value();
    const E& error() const;
private:
    std::variant<T, E> data_;
};

}  // namespace sbf::util
```

### 3.2 实现文件

`src/util/CMakeLists.txt`:
```cmake
add_library(sbf_util STATIC
    math_util.cpp
    timer.cpp
)
target_link_libraries(sbf_util PUBLIC Eigen3::Eigen)
```

---

## 4. Smoke Test — P0

**`test/smoke/CMakeLists.txt`**:
```cmake
add_executable(smoke_p0 smoke_p0.cpp)
target_link_libraries(smoke_p0 sbf_util GTest::gtest_main)
add_test(NAME smoke_P0 COMMAND smoke_p0)
set_tests_properties(smoke_P0 PROPERTIES TIMEOUT 60)
```

**`test/smoke/smoke_p0.cpp`**:
```cpp
#include <gtest/gtest.h>
#include <sbf/util/math_util.h>
#include <sbf/util/timer.h>
#include <Eigen/Geometry>

TEST(P0Smoke, ClampWorks) {
    EXPECT_EQ(sbf::util::clamp(5.0, 0.0, 3.0), 3.0);
    EXPECT_EQ(sbf::util::clamp(-1.0, 0.0, 3.0), 0.0);
    EXPECT_EQ(sbf::util::clamp(2.0, 0.0, 3.0), 2.0);
}

TEST(P0Smoke, FaceOverlapDetected) {
    Eigen::AlignedBox3d a(Eigen::Vector3d(0,0,0), Eigen::Vector3d(1,1,1));
    Eigen::AlignedBox3d b(Eigen::Vector3d(1,0,0), Eigen::Vector3d(2,1,1));
    // 共享 x=1 面，应有 face overlap
    EXPECT_TRUE(sbf::util::has_face_overlap(a, b));
}

TEST(P0Smoke, NoFaceOverlapWhenDiagonal) {
    Eigen::AlignedBox3d a(Eigen::Vector3d(0,0,0), Eigen::Vector3d(1,1,1));
    Eigen::AlignedBox3d b(Eigen::Vector3d(1,1,1), Eigen::Vector3d(2,2,2));
    // 仅点接触，没有面重叠
    EXPECT_FALSE(sbf::util::has_face_overlap(a, b));
}

TEST(P0Smoke, TimerWorks) {
    sbf::util::Timer t;
    t.start();
    // 忙等 1ms
    volatile int x = 0;
    for (int i = 0; i < 1000000; ++i) x += i;
    EXPECT_GT(t.elapsed_ms(), 0.0);
}
```

---

## 5. CI 配置

创建 `cpp/v7/.github/workflows/ci.yml`（若使用 GitHub Actions）：

```yaml
name: v7-CI

on: [push, pull_request]

jobs:
  quick-build-test:
    runs-on: ubuntu-22.04
    timeout-minutes: 10
    steps:
      - uses: actions/checkout@v4
      - name: Configure
        run: |
          cd cpp/v7/build
          cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON
      - name: Build
        run: make -C cpp/v7/build -j4
      - name: Smoke Tests (--quick)
        run: |
          cd cpp/v7/build
          ctest --output-on-failure -R smoke_P -j4 --timeout 60
```

**注意 D3 决策**: CI 仅跑 `smoke_P*` 测试（`-R smoke_P`），不跑实验。

---

## 6. 从 v6 迁移的零件（只复制，不修改）

以下文件可以从 v6 原样复制，作为 P0 的基础素材，在后续 Phase 按模块改写：

| v6 来源 | v7 目标 | 后续处理 |
|---------|---------|---------|
| `v6/include/sbf_common.h` | 阅读后提取到各子模块头 | P1 改写 |
| `v6/src/sbf_robot.cpp` | 读取后在 P2 改写 | P2 改写 |
| `v6/experiments/configs/*.yaml` | `v7/experiments/configs/` | P6 清理 |

**不复制**:
- `v6/src/sbf_lect.cpp`（P3 全新实现）
- `v6/src/grower_coordinated.cpp`（P4 全新拆分）
- `v6/src/sbf_planner.cpp`（P5 全新拆分）

---

## 7. 构建验证步骤

P0 完成时，运行以下命令验证，**所有命令必须成功**：

```bash
cd cpp/v7
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTS=ON
make -j8 sbf_util smoke_p0
ctest -R smoke_P0 --output-on-failure
# 期望：4/4 tests passed
```

---

## 8. Definition of Done

- [ ] `cpp/v7/` 目录结构创建完毕
- [ ] `CMakeLists.txt` 可配置，所有业务子模块处于注释状态
- [ ] `sbf_util` 编译成功（`libsbf_util.a`）
- [ ] `smoke_P0` 4 个测试全部通过
- [ ] CI yaml 文件就位，可本地 dry-run
- [ ] `compile_commands.json` 生成（供 clangd）
- [ ] `/memories/session/plan.md` 中 P0 打勾

---

*Phase: P0 | 依赖: 无 | 解锁: P1*
