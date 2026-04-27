# P2 — Link AABB、Z4 体素与场景碰撞检测（envelope / voxel / scene）

> **目标**: 从 P1 的零半径端点 iAABB 派生**零半径** link AABB（保留 D1 几何分离）；
> port v6 SparseVoxelGrid（BitBrick + FlatBrickMap）；提供"传入 link_radii 即在线
> 膨胀"的批量 SAT 碰撞检测函数；定义 v7 `BoxNode`（无 radii）。
>
> **预计工时**: 2-3 天（大量文件可原样 port，仅 API 表面层调整）  
> **前置条件**: P1 完成（smoke_P1 通过）  
> **完成标准**: `ctest -R smoke_P2` 通过；voxel_grid 单元测试 byte-equal v6。

---

## 0. 重要更正（与初稿差异）

P2 初稿基于"假设 v6 有 Scene/CollisionChecker 类"，实际上 v6 是**函数式接口**：

| 维度 | 初稿 | 实际 v6 现状 / v7 修订 |
|------|------|----------------------|
| 障碍物类型 | `Obstacle{Eigen::AlignedBox3d}` | **`Obstacle{float bounds[6]}`**（紧凑布局） |
| 场景容器 | `Scene::load_yaml/json` | **不存在**；调用方传 `Obstacle*` + `n_obs` |
| 碰撞接口 | `CollisionChecker::check(chain, cfg)` | **`aabbs_collide_obs(aabb, n_slots, obs_compact, n_obs)`**（无状态函数） |
| 体素后端 | "Z4VoxelGrid（黑盒）" | **`SparseVoxelGrid` (FlatBrickMap + 8³ BitBrick)**（v6 完整实现，566+123 LOC，原样 port） |
| YAML | yaml-cpp | **JSON**（沿用 nlohmann_json） |
| BoxNode | "v6 含 radii，v7 删除" | **v6 已无 radii**（D1 天然满足）；只需 port |

**架构决策 D1 在 P2 的实际落点**:
- `derive_link_iaabb_paired_zero` 输入零半径端点 iAABB → 输出**零半径** link AABB（**不再**像 v6 在此处应用 radii）。
- `BoxNode::link_iaabbs` 存储零半径 link AABB。
- 新增 `aabbs_collide_obs_inflated(link_iaabbs, n, link_radii, obs_compact, n_obs)`：在 SAT 检测前对每个 link 在线 ±radius 膨胀。这是 v7 中**唯一**应用 link_radii 的地方。
- 旧 `aabbs_collide_obs(aabb, n, obs)` 保留作"已膨胀"快速路径供测试和回归对比。

> v6 `derive_link_iaabb_paired(... link_radii != nullptr ...)` 在 v7 P2 **不**直接对外暴露含 radii 的版本；只暴露 zero 版本。但内部实现可保留 radii overload 作为单元测试 oracle（验证"先膨胀后检测"与"先 derive 后膨胀检测"等价）。

---

## 1. 模块职责边界

```
P2 管辖 (cpp/v7/{include,src}/sbf/):
  envelope/link_iaabb.h, .cpp        - 端点 iAABB → 零半径 link AABB
  envelope/envelope_type.h, .cpp     - LinkEnvelope 容器 + 派生入口
  voxel/bit_brick.h                  - 8³ BitBrick 单元（header-only）
  voxel/voxel_grid.h                 - FlatBrickMap + SparseVoxelGrid
  voxel/hull_rasteriser.h, .cpp      - rasterise_robot_hull16 / sub_aabbs / aabbs
  scene/box_node.h                   - v7 BoxNode 定义（已无 radii）
  scene/obstacle.h                   - Obstacle{float[6]}（从 v6 types.h 拆出）
  scene/collision.h, .cpp            - aabbs_collide_obs + aabbs_collide_obs_inflated

P2 不管辖（其他 Phase）:
  LECT 树、node_grids_、cache key       → P3
  Hull16 turbo 扫描线优化（已 port，但 hull-rasteriser 接口由 P3/P4 决策）
  多线程 grower、obs_grid 生命周期管理   → P4
  YAML 场景加载                          → 不做（v6 无此抽象，v7 沿用调用方传指针）
```

---

## 2. 文件清单（v6 → v7 一一对应）

| v7 路径 | v6 来源 | LOC（v6） | 改动级别 |
|---------|---------|-----------|---------|
| `include/sbf/scene/obstacle.h` | `core/types.h` 中的 `Obstacle` 结构 | ~12 | 拆分到独立头 |
| `include/sbf/scene/box_node.h` | `core/types.h` 中的 `BoxNode` 结构 | ~30 | 拆分到独立头；P2 仅引用，不赋值 |
| `include/sbf/envelope/link_iaabb.h` | `envelope/link_iaabb.h` | 42 | 改 `derive_*` 签名为零半径版 |
| `src/envelope/link_iaabb.cpp` | `envelope/link_iaabb.cpp` | 80 | 删除 link_radii 形参；保留 paired/subdivided |
| `include/sbf/envelope/envelope_type.h` | `envelope/envelope_type.h` | 84 | port；`compute_link_envelope` 不接受 radii |
| `src/envelope/envelope_type.cpp` | `envelope/envelope_type.cpp` | 86 | port；体素填充时 radii=0（盘子 P3 在 obs 端膨胀） |
| `include/sbf/voxel/bit_brick.h` | `voxel/bit_brick.h` | 123 | 原样 port |
| `include/sbf/voxel/voxel_grid.h` | `voxel/voxel_grid.h` | 566 | **header-only**，原样 port（含 fill_hull16） |
| `include/sbf/voxel/hull_rasteriser.h` | `voxel/hull_rasteriser.h` | ~80 | port |
| `src/voxel/hull_rasteriser.cpp` | `src/voxel/hull_rasteriser.cpp` | ?? | port |
| `include/sbf/scene/collision.h` | `scene/collision_checker.h` | 70 | 改名；保留函数式接口 |
| `src/scene/collision.cpp` | `src/scene/collision_checker.cpp` | 222 | port + **新增** `aabbs_collide_obs_inflated` |

总计估计 ~1.3 K LOC。voxel_grid.h 单文件 566 LOC 但由 ~15 个独立函数组成，每个 < 100 行（C1 满足）。

---

## 3. 关键 API（v7 形式）

### 3.1 零半径派生

```cpp
// include/sbf/envelope/link_iaabb.h
namespace sbf::envelope {

/// Derive zero-radius link AABBs from zero-radius endpoint iAABBs.
/// D1: this function does NOT inflate by link_radii. Inflation happens
/// at scene-collision time via aabbs_collide_obs_inflated().
///
/// in:  endpoint_iaabbs[n_active * 2 * 6]
/// out: link_iaabbs[n_active * 6]    layout [lo_xyz, hi_xyz]
void derive_link_iaabb_paired_zero(
    const float* endpoint_iaabbs,
    int n_active_links,
    float* out_link_iaabbs);

void derive_link_iaabb_subdivided_zero(
    const float* endpoint_iaabbs,
    int n_active_links,
    int n_subdivisions,
    float* out_sub_iaabbs);

}  // namespace sbf::envelope
```

### 3.2 在线膨胀的 SAT 检测

```cpp
// include/sbf/scene/collision.h
namespace sbf::scene {

/// AABB-vs-Obstacle SAT batch test (already-inflated AABBs).
/// Returns true if any of the n_slots AABBs overlaps any of the n_obs obstacles.
/// Direct port of v6 aabbs_collide_obs.
bool aabbs_collide_obs(
    const float* aabb,        // [n_slots * 6]
    int n_slots,
    const float* obs_compact, // [n_obs * 6]
    int n_obs);

/// D1 enforcement: inflate each link's zero-radius AABB by link_radii[i]
/// inline, then run SAT against obstacles.
/// link_radii_per_slot:
///   - if subdivisions > 1, n_slots = n_active * n_sub and the same radii[ci]
///     applies to all sub-slots of link ci. Caller must pass a per-slot vector
///     (size n_slots) — typically built once via expand_radii_to_slots().
bool aabbs_collide_obs_inflated(
    const float* zero_link_aabb,    // [n_slots * 6]
    int n_slots,
    const float* link_radii_per_slot, // [n_slots]
    const float* obs_compact,
    int n_obs);

/// Helper: expand per-active-link radii to per-slot radii.
void expand_radii_to_slots(
    const double* link_radii, int n_active_links, int n_subdivisions,
    float* out_per_slot);  // [n_active * n_subdivisions]

}  // namespace sbf::scene
```

### 3.3 BoxNode（已无 radii）

```cpp
// include/sbf/scene/box_node.h
namespace sbf::scene {

/// LECT leaf node. Per D1, stores zero-radius link AABBs only.
struct BoxNode {
    int                       id = -1;
    std::vector<core::Interval> joint_intervals;
    Eigen::VectorXd           seed_config;
    double                    volume = 0.0;
    int                       tree_id = -1;
    int                       parent_box_id = -1;
    int                       root_id = -1;

    /// Zero-radius link AABBs ([n_active * 6]). Inflation happens at check time.
    std::vector<float>        link_iaabbs;
};

}  // namespace sbf::scene
```

---

## 4. Smoke Test — P2

`test/smoke/smoke_p2.cpp` 包含 ≤ 5s 的用例：

1. **`P2.DerivePairedZeroNoInflation`** — 给一组手造端点 iAABB（proximal=[0,0,0]→[1,1,1]，distal=[2,2,2]→[3,3,3]），调用 `derive_link_iaabb_paired_zero`，断言输出 = `[0,0,0,3,3,3]`，**不**含任何 radii 痕迹（与 v6 `derive(... nullptr)` 同等）。
2. **`P2.SatNoCollisionInFreeSpace`** — link AABB `[0,0,0,1,1,1]`，障碍物 `[5,5,5,6,6,6]`，`aabbs_collide_obs` 返回 false。
3. **`P2.SatDetectsCollision`** — link AABB `[0,0,0,1,1,1]`，障碍物 `[0.5,0.5,0.5,2,2,2]`，返回 true。
4. **`P2.InflatedCollisionDetected`** — link `[0,0,0,1,1,1]`（零半径）+ radii=0.6 → 膨胀后 `[-0.6,..,1.6]`；障碍物 `[1.3,1.3,1.3,2,2,2]` 被命中，`aabbs_collide_obs` 否（无膨胀）但 `aabbs_collide_obs_inflated` 是。
5. **`P2.VoxelGridFillAndCollide`** — `SparseVoxelGrid g1(0.1)` 填充 `[0,0,0,1,1,1]`；`g2(0.1)` 填充 `[0.5,0.5,0.5,1.5,1.5,1.5]`；`g1.collides(g2) == true`。
6. **`P2.VoxelGridDisjoint`** — 两个不重叠 grid 的 `collides` 返回 false。
7. **`P2.PipelineEndToEnd`** — IIWA14 在零构型，调用 P1 计算端点 iAABB → P2 派生 link AABB → `aabbs_collide_obs_inflated` 与一个远处障碍物 `[10,10,10,11,11,11]` 不碰撞；近处障碍物 `[0,0,1.4, 1,1,1.5]` 在 link_radii ≥ 0.05 时碰撞。

**目标耗时**: < 200 ms。

---

## 5. CMake 集成

### 5.1 顶层

取消注释：
```cmake
add_subdirectory(src/envelope)   # P2
add_subdirectory(src/voxel)      # P2
add_subdirectory(src/scene)      # P2
```

### 5.2 子模块

```cmake
# src/envelope/CMakeLists.txt
add_library(sbf_envelope STATIC link_iaabb.cpp envelope_type.cpp)
target_link_libraries(sbf_envelope PUBLIC sbf_core sbf_voxel)

# src/voxel/CMakeLists.txt
add_library(sbf_voxel STATIC hull_rasteriser.cpp)
target_include_directories(sbf_voxel PUBLIC ${CMAKE_SOURCE_DIR}/include)
target_link_libraries(sbf_voxel PUBLIC Eigen3::Eigen)

# src/scene/CMakeLists.txt
add_library(sbf_scene STATIC collision.cpp)
target_link_libraries(sbf_scene PUBLIC sbf_core sbf_voxel)
```

`smoke_p2` 链接 `sbf_envelope sbf_voxel sbf_scene sbf_core sbf_util GTest::gtest_main`。

---

## 6. 已知陷阱

1. **`fill_hull16` 复杂度**：v6 实现含数值二次方程求解，P2 **原样 port，不重写**（C1 不超 500 行的"函数"，不是文件）。仅保证编译 + 跑现有 v6 单元测试用例通过即可。
2. **`SparseVoxelGrid::safety_pad` 默认值**：v6 默认 `sqrt(3)*delta/2`（半体素对角线）。v7 必须保持完全相同，否则 byte-equal 失败。
3. **AABB 字节布局**：v6 用 `[lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]`（紧凑 6 个 float）；v7 不引入 `Eigen::AlignedBox3d` 作为存储格式，仅在测试断言里临时构造。
4. **`Obstacle` 既是 v6 types.h 一部分**：v7 P1 已**未** port 此结构，需在 P2 新建 `scene/obstacle.h`。
5. **`BoxNode`**：v6 已无 radii；v7 P2 仅声明结构，**不**在 P2 内做任何分配/管理。LECT 在 P3 才使用。
6. **`aabbs_collide_obs_inflated` 的 ε**：v6 SAT 的 epsilon 是 `1e-6f`（[collision_checker.cpp](cpp/v6/src/scene/collision_checker.cpp)）。v7 保持。膨胀后该 ε 不应改变。
7. **subdivisions 与 radii 对齐**：subdivided 模式下 `n_slots = n_active * n_sub`，但 `link_radii` 只有 `n_active` 个。`expand_radii_to_slots` 必须正确广播。

---

## 7. Definition of Done

- [ ] `sbf_envelope` / `sbf_voxel` / `sbf_scene` 三库编译成功（`-Wall -Wextra` 无新增警告）
- [ ] `smoke_P2` 7 个 TEST 全部通过（< 1 s）
- [ ] `BoxNode` 中无 `link_radii` 字段（D1 落实，与 v6 一致）
- [ ] `derive_link_iaabb_paired_zero` 不含 link_radii 形参
- [ ] `aabbs_collide_obs_inflated` 是 v7 中**唯一**直接消费 link_radii 的函数（grep 验证）
- [ ] 顶层 `CMakeLists.txt` 中 `src/envelope`、`src/voxel`、`src/scene` 已取消注释
- [ ] `/memories/session/plan.md` P2 打勾

---

*Phase: P2 | 依赖: P1 | 解锁: P3, P4*
