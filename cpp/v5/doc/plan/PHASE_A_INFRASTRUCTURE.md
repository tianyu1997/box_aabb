# Phase A: 基础设施

> 依赖: 无
> 状态: ✅ 已完成 (2026-04-03)
> 产出: CMake 构建系统 + core/ 类型库 + scene/ 碰撞检测

---

## 目标

搭建 v5 的构建骨架和基础类型系统，使后续所有模块可以在此基础上编译和测试。

---

## Step A1: CMake 构建系统

### 文件
- `CMakeLists.txt` — 顶层构建文件

### 设计要求

```cmake
cmake_minimum_required(VERSION 3.16)
project(SafeBoxForest_v5 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
```

**依赖管理** (FetchContent, 缓存到 `_sbf5_deps/`):
| 依赖 | 版本 | 方式 | 备注 |
|------|------|------|------|
| Eigen3 | 3.4.0 | FetchContent (always) | 缓存到 `_sbf5_deps/`; 跳过 find_package 以避免旧缓存问题 |
| nlohmann/json | 3.11+ | FetchContent | header-only |
| doctest | 2.4+ | FetchContent | 测试框架, header-only |
| Drake | any | `find_package(drake QUIET)` | 可选, 仅 planner 使用 |

**编译选项**:
- MSVC: `/W4 /WX /wd4819 /wd4127`（警告作为错误; wd4819=Eigen codepage 936 警告, wd4127=Eigen 常量条件表达式）
- GCC/Clang: `-Wall -Wextra -Wpedantic -Werror`
- 统一: `-DNOMINMAX`（Windows 防止 min/max 宏冲突）
- 注意: 编译选项使用 `target_compile_options` 而非全局 `add_compile_options`，避免影响 FetchContent 目标

**构建目标**:
| Target | 类型 | 内容 |
|--------|------|------|
| `sbf5` | STATIC library | 全部 src/ 下的 .cpp (当前: interval_math, robot, fk_state, joint_symmetry, collision_checker) |
| `test_core` | 可执行文件 | Phase A 测试, link sbf5 + doctest + Eigen |
| `sbf5_experiments` | 多个可执行文件 | 可选, `SBF_BUILD_EXPERIMENTS=OFF` (尚未创建) |

**CMake Options**:
```cmake
option(SBF_BUILD_TESTS       "Build unit tests"       ON)
option(SBF_BUILD_EXPERIMENTS "Build experiment suite"  OFF)
```

### 迁移来源
- v4 `safeboxforest/v4/CMakeLists.txt` — 参考依赖下载逻辑，但大幅简化
- v4 的 `_sbf4_fc/` 缓存方式 → v5 改为 `_sbf5_deps/`

### 验收标准
- [x] `cmake -B build` 成功配置 (VS2022 x64)
- [x] `cmake --build build --config Release` 零错误 (sbf5.lib)
- [x] tests target 可编译运行 (test_core.exe, 16 tests, 400933 assertions, 全部通过)

---

## Step A2: core/ — 基础类型

### 文件
| 头文件 | 实现 | 职责 |
|--------|------|------|
| `include/sbf/core/types.h` | (header-only) | `Interval`, `Obstacle`, `BoxNode` |
| `include/sbf/core/interval_math.h` | (header-only) | 区间三角函数 `I_sin`, `I_cos`, `imat_mul_dh` |
| `include/sbf/core/robot.h` | `src/core/robot.cpp` | DH Robot 模型 |
| `include/sbf/core/fk_state.h` | `src/core/fk_state.cpp` | FKState + IFK 计算 |
| `include/sbf/core/joint_symmetry.h` | (header-only 或 .cpp) | Z4 对称性检测 |

### 关键结构定义

```cpp
// types.h
struct Interval {
    double lo, hi;
    double width()  const { return hi - lo; }
    double center() const { return 0.5 * (lo + hi); }
    bool   contains(double v) const { return v >= lo && v <= hi; }
    bool   empty()  const { return lo > hi; }
};

struct Obstacle {
    float bounds[6];  // [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
};

struct BoxNode {
    int id;
    std::vector<Interval> joint_intervals;
    Eigen::VectorXd seed_config;
    double volume;
    int tree_id = -1;       // LECT node index
    int parent_box_id = -1;
    int root_id = -1;

    int    n_dims() const;
    Eigen::VectorXd center() const;
    void   compute_volume();
    bool   contains(const Eigen::VectorXd& q, double tol = 1e-10) const;
};
```

```cpp
// interval_math.h
namespace sbf {
    Interval I_sin(double lo, double hi);
    Interval I_cos(double lo, double hi);

    // 4x4 interval matrix (flat float[16] or Eigen-based)
    void imat_mul_dh(const float* A, const float* B, float* C);
    void build_dh_joint(float a, float alpha, float d, Interval theta, float* out);
}
```

```cpp
// robot.h
class Robot {
public:
    static Robot from_json(const std::string& path);

    int n_joints() const;
    int n_active_links() const;
    const std::vector<Interval>& joint_limits() const;
    const std::vector<std::array<double,4>>& dh_params() const;  // [a, alpha, d, theta_offset]
    const std::vector<double>& link_radii() const;

    // Coupled joints (for constrained robots)
    const std::vector<std::pair<int,int>>& coupled_pairs() const;
};
```

```cpp
// fk_state.h
struct FKState {
    int n_joints;
    std::vector<float> prefix_chain;  // [n_joints × 16] interval matrices
    bool valid = false;
};

void compute_fk_full(FKState& fk, const Robot& robot, const std::vector<Interval>& intervals);
void compute_fk_incremental(FKState& fk, const Robot& robot, const std::vector<Interval>& intervals, int changed_dim);
void extract_link_iaabbs(const FKState& fk, const Robot& robot, float* out_endpoint_iaabbs);
```

```cpp
// joint_symmetry.h
enum class JointSymmetryType { NONE, Z4_ROTATION };

struct JointSymmetry {
    JointSymmetryType type = JointSymmetryType::NONE;
    double period = 0.0;  // e.g. 2π for full rotation
    double alpha  = 0.0;

    static JointSymmetry detect(const Robot& robot, int joint_idx);
};
```

### 迁移来源
| 组件 | 源版本 | 源文件 | 备注 |
|------|--------|--------|------|
| `Interval` | v1 | `v1/include/sbf/robot/interval_math.h` | C++ 原生, 最成熟 |
| `interval_math` | v1 | `v1/include/sbf/robot/interval_math.h` | `I_sin`, `I_cos` |
| `Robot` | v4 | `safeboxforest/v4/include/sbf/robot/robot.h` | 支持 coupled joints |
| `FKState` | v1+v4 | v1 `interval_fk.h` + v4 `interval_fk.h` | v4 增量 FK 更优 |
| `JointSymmetry` | v4 | `safeboxforest/v4/include/sbf/core/joint_symmetry.h` | Z4 检测 |

### 验收标准
- [x] `Interval` 基本运算通过单元测试
- [x] `I_sin`, `I_cos` 保守性验证（MC 100 trials × 1000 点全包含）
- [x] `Robot::from_json("data/2dof_planar.json")` 成功加载
- [x] `compute_fk_full` + `extract_link_aabbs` 输出合理 AABB
- [x] `compute_fk_incremental` 与 `compute_fk_full` 结果一致

---

## Step A3: scene/ — 碰撞检测

### 文件
| 头文件 | 实现 | 职责 |
|--------|------|------|
| `include/sbf/scene/collision_checker.h` | `src/scene/collision_checker.cpp` | SAT 碰撞检测 |

### 接口

```cpp
class CollisionChecker {
public:
    CollisionChecker(const Robot& robot);

    void set_obstacles(const Obstacle* obs, int n_obs);

    // 单配置碰撞检测: 计算 FK → 检查 link AABB vs obstacles
    bool check_config(const Eigen::VectorXd& q) const;

    // 区间 box 碰撞检测: 计算 link iAABBs → SAT
    bool check_box(const std::vector<Interval>& intervals) const;

    // 线段碰撞检测: 线性插值 N 步, 每步 check_config
    bool check_segment(const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                       int resolution = 10) const;

    const Obstacle* obstacles() const;
    int n_obstacles() const;

private:
    const Robot& robot_;
    const Obstacle* obs_ = nullptr;
    int n_obs_ = 0;
};
```

### 碰撞算法
- **SAT (Separating Axis Theorem)**: 对每个 link iAABB × 每个 obstacle, 检查 6 轴分离
- Link iAABB 与 obstacle 均为 axis-aligned → SAT 退化为 6 维区间重叠检测:
  ```
  collides = ∀d∈{x,y,z}: link.lo[d] ≤ obs.hi[d] AND link.hi[d] ≥ obs.lo[d]
  ```
- 碰撞=true 当任何一个 link 与任何一个 obstacle 重叠

### 迁移来源
| 组件 | 源版本 | 源文件 |
|------|--------|--------|
| `CollisionChecker` | v1 | `v1/include/sbf/scene/aabb_collision_checker.h` + `v1/src/scene/scene.cpp` |

### 验收标准
- [x] 已知碰撞/无碰撞配置测试通过
- [x] `check_box` 保守性: 若 box 内任意 config 碰撞 → check_box 为 true（MC 验证）
- [x] `check_segment` 在直线碰撞路径上返回 true

---

## 测试: test_core.cpp

```
TEST_SUITE("Interval") {
    - width, center, contains
    - I_sin / I_cos 保守性 (MC 1000 random points within interval → result in output)
    - I_sin / I_cos 紧致性 (width 不超过真实范围 2x)
}

TEST_SUITE("Robot") {
    - from_json 加载 2dof_planar.json
    - n_joints, joint_limits 正确
}

TEST_SUITE("FKState") {
    - compute_fk_full 输出 link iAABB 维度正确
    - compute_fk_incremental 与 full 一致 (修改 dim 0)
    - extract_link_iaabbs 输出格式 [n_active × 2 × 6]
}

TEST_SUITE("CollisionChecker") {
    - 2DOF 已知无碰撞 config
    - 2DOF 已知碰撞 config
    - check_box 保守性
}
```
