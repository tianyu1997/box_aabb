# P1 — Interval FK 与零半径端点 AABB（core 模块迁移）

> **目标**: 将 v6 的 `Robot` (DH 模型) + `FKState` (区间 FK) + 端点 AABB 提取直接迁移到 v7
> `src/core/`，**全部输出零半径** AABB（执行决策 D1）。
>
> **预计工时**: 1-2 天（直接 port，无算法变更）  
> **前置条件**: P0 完成（smoke_P0 通过）  
> **完成标准**: `ctest -R smoke_P1` 通过；与 v6 在相同 DH 配置 + 相同关节区间下输出
> **逐 byte 一致**（FK 数值差 < 1e-12）。

---

## 0. 重要更正（与初稿差异）

P1 初稿假设 v6 使用 URDF + Drake FK，**实际不是**。v6 通过 DH JSON 参数 + 手写
区间矩阵乘法（IFK）实现 FK。本 Phase 因此调整为：

| 维度 | 初稿 | 修订 |
|------|------|------|
| 运动学源 | URDF (Drake) | **DH JSON**（v6 现状） |
| FK 计算 | `MultibodyPlant::CalcRelativeTransform` | **手写 4×4 区间矩阵乘**（v6 `interval_math.cpp`） |
| 端点输出 | 8-vertex AABB 变换 | **`prefix[li]` / `prefix[li+1]` 平移列**（v6 `extract_endpoint_iaabbs`） |
| 模块名 | `AffineChain` / `UrdfEndpointSource` | **保留 v6 命名** `Robot` / `FKState` / `compute_endpoint_iaabb_ifk` |
| 数值对齐 | 与 Drake FK 对比 | **与 v6 byte-equal**（同一份代码 port） |

不引入新依赖（D1 之外），但需 `nlohmann/json`（v6 已用，FetchContent 引入）。

---

## 1. 模块职责边界

```
P1 管辖 (cpp/v7/src/core/):
  include/sbf/core/types.h          - Interval, JointLimits, MAX_JOINTS, MAX_TF
  include/sbf/core/interval_math.h  - I_sin/I_cos, build_dh_joint, imat_mul_dh
  include/sbf/core/robot.h          - Robot (DH params + active_link_map)
  include/sbf/core/fk_state.h       - FKState + compute_fk_full/incremental
  include/sbf/core/endpoint_iaabb.h - compute_endpoint_iaabb_ifk(零半径)
  src/core/{interval_math,robot,fk_state,endpoint_iaabb}.cpp

P1 不管辖（其他 Phase）:
  link_radii 应用                    → P2 (derive_link_iaabb_paired)
  CritSample / Analytical / GCPC / MC → 暂不迁移（仅 IFK；其他变体在 P2 或留作 v8）
  joint_symmetry / coupled_pairs     → 仅传递结构，不实现策略（P3 用）
  环境 / 碰撞 / 缓存                  → P2 / P3
```

**架构决策 D1 在 P1 的具体落实**:
- 头文件提供 **唯一公开 API** `compute_endpoint_iaabb_ifk(robot, intervals)`，返回
  `EndpointIAABBResult{ endpoint_iaabbs(零半径), fk_state }`。
- v6 `extract_link_aabbs(... link_radii=nullptr)` 中带 radii 参数的重载在 v7 P1 **不再
  暴露**；调用方只能拿到零半径输出。
- 所有"含半径"的工具（`derive_link_iaabb_paired` with radii）放到 P2 `src/envelope/`。

---

## 2. 文件清单（按 v6→v7 一一对应）

| v7 路径 | v6 来源 | 改动 |
|---------|---------|------|
| `include/sbf/core/types.h` | `cpp/v6/include/sbf/core/types.h` | port `Interval`/`JointLimits` 部分；**删除** `Obstacle`/`BoxNode`（属 P2/P3） |
| `include/sbf/core/interval_math.h` | `cpp/v6/include/sbf/core/interval_math.h` | 原样 port |
| `src/core/interval_math.cpp` | `cpp/v6/src/core/interval_math.cpp` | 原样 port |
| `include/sbf/core/robot.h` | `cpp/v6/include/sbf/core/robot.h` | port；**保留** `link_radii` 字段（P2 读），但 P1 内部不使用 |
| `src/core/robot.cpp` | `cpp/v6/src/core/robot.cpp` | port |
| `include/sbf/core/fk_state.h` | `cpp/v6/include/sbf/core/fk_state.h` | port；**`extract_link_aabbs` 删除 link_radii 形参**（D1） |
| `src/core/fk_state.cpp` | `cpp/v6/src/core/fk_state.cpp` | port + 上面那处签名变化 |
| `include/sbf/core/endpoint_iaabb.h` | `cpp/v6/include/sbf/envelope/endpoint_source.h`（部分） | 仅保留 `EndpointIAABBResult` + IFK 变体接口 |
| `src/core/endpoint_iaabb.cpp` | `cpp/v6/src/envelope/endpoint_source_ifk.cpp` | port，**输入路径不接受 link_radii** |
| `data/iiwa14.json`, `data/panda.json`, `data/2dof_planar.json` | `cpp/v6/data/*.json` | 拷贝 |

**共计**: ~600 LOC（含注释），单文件均 < 250 行。

---

## 3. 关键 API（v7 形式）

```cpp
// include/sbf/core/endpoint_iaabb.h
namespace sbf::core {

/// Result of one IFK evaluation. All AABBs are zero-radius (D1).
struct EndpointIAABBResult {
    /// [n_active_links × 2 × 6] flat float buffer.
    /// Layout per link i: proximal[lo3, hi3], distal[lo3, hi3].
    std::vector<float> endpoint_iaabbs;
    int                n_active_links = 0;
    FKState            fk_state;        // useful for incremental updates
    bool               is_safe = true;  // IFK always returns conservative bound
};

/// Interval-FK endpoint AABB computation (zero-radius).
/// `fk_cache` may be nullptr (full FK) or a parent state (incremental, pass `changed_dim`).
EndpointIAABBResult compute_endpoint_iaabb_ifk(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    FKState* fk_cache = nullptr,
    int changed_dim = -1);

}  // namespace sbf::core
```

**注意**：v6 的 `EndpointSource` 枚举（IFK/CritSample/...）暂**不**进入 v7。v7 P1
只暴露 IFK 一种来源。其他变体若 P5/P6 实验需要，再以独立函数加入。

---

## 4. Smoke Test — P1

`test/smoke/smoke_p1.cpp` 包含以下用例（均 ≤ 5s）：

1. **`P1.RobotLoadsIIWA`** — 从 `data/iiwa14.json` 加载，断言 `n_joints()==7`，
   `n_active_links() == 5`（active map: links 0,2,4,6 + tool 7 — 与 v6 输出一致）。
2. **`P1.IntervalSinCos`** — `I_sin(0, π) == [0, 1]`、`I_cos(-π, π) == [-1, 1]`。
3. **`P1.FkAtZeroConfigIIWA`** — IIWA 全零关节区间 `[0,0]^7`，
   FK 后 tool 帧 z 平移 = `0.36+0.42+0.4+0.081+0.2 = 1.461`，误差 < 1e-12。
4. **`P1.IncrementalFkMatchesFull`** — 任取一个非零构型，先 full FK，再改第 3 维度
   重算（一次 incremental，一次 full from scratch），两者所有 prefix 矩阵 byte-equal。
5. **`P1.EndpointIaabbZeroRadius`** — IIWA 在 `[-0.1, 0.1]^7` 区间下计算端点 iAABB，
   断言每个 link 的 AABB **不**包含 `link_radii` 膨胀（与同一构型用 v6 的 zero-radius
   输出 byte-equal —— 因为代码即来自 v6，写为自洽测试：手算第一个 link 端点 = 原点）。

**总耗时目标**: < 1 秒（CI `--quick` 远低于 60s 上限）。

---

## 5. CMake 集成

### 5.1 顶层增量

在 `cpp/v7/CMakeLists.txt` 取消注释：
```cmake
add_subdirectory(src/core)   # P1
```

### 5.2 nlohmann/json 引入

P1 需 JSON 解析。在顶层 CMake 添加：
```cmake
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        nlohmann_json
        URL https://github.com/nlohmann/json/archive/refs/tags/v3.11.3.tar.gz
    )
    FetchContent_MakeAvailable(nlohmann_json)
endif()
```

### 5.3 `src/core/CMakeLists.txt`

```cmake
add_library(sbf_core STATIC
    interval_math.cpp
    robot.cpp
    fk_state.cpp
    endpoint_iaabb.cpp
)
target_link_libraries(sbf_core
    PUBLIC sbf_util Eigen3::Eigen
    PRIVATE nlohmann_json::nlohmann_json
)
```

### 5.4 数据文件

将 `cpp/v6/data/{iiwa14,panda,2dof_planar}.json` 拷贝到 `cpp/v7/data/`。
smoke_p1 通过 `${CMAKE_SOURCE_DIR}/data/iiwa14.json` 路径读取（CMake `configure_file` 或
`target_compile_definitions(... -DSBF_DATA_DIR=...)`）。

---

## 6. 已知陷阱

1. **`MAX_JOINTS=32`、`MAX_TF=34`**：v6 用栈分配 `double[16][34]` 等数组，PFK state
   单实例约 17 KB。v7 保持，避免堆分配开销。
2. **`fingerprint()` 哈希**：v6 包含 `link_radii`，v7 P3 缓存依赖此 fingerprint。但
   决策 D1 要求缓存 key **不含** link_radii。**P1 暂不修改**（保持 v6 行为），P3 在
   构建 cache key 时显式跳过 radii。这一点写入 `/memories/repo/`。
3. **`coupled_pairs`**：v6 的关节耦合策略在 P3 LECT 中使用，P1 仅 port 字段，不实现
   逻辑。
4. **JSON 路径**：v6 中很多代码假设 `data/iiwa14.json` 相对工作目录。v7 用 CMake
   `target_compile_definitions` 注入 `SBF_DATA_DIR` 宏，测试中显式拼接绝对路径。

---

## 7. Definition of Done

- [ ] `sbf_core` 库编译成功（无警告，`-Wall -Wextra`）
- [ ] `smoke_P1` 5 个 TEST 全部通过（ctest < 1s）
- [ ] `data/{iiwa14,panda,2dof_planar}.json` 已拷贝
- [ ] 头文件不再暴露 `link_radii` 形参（D1 落实）
- [ ] 顶层 `CMakeLists.txt` 中 `src/core` 已取消注释
- [ ] `/memories/session/plan.md` P1 打勾
- [ ] `/memories/repo/v7_d1_radii_note.md` 记录 P3 fingerprint 调整 TODO

---

*Phase: P1 | 依赖: P0 | 解锁: P2, P3*
