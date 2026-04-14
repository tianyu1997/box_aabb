# Phase O: 端到端集成测试 + Python Bindings 验证

> 依赖: Phase A-N (全部)
> 状态: ✅ 已完成
> 产出: 完整端到端 C++ 测试 + Python bindings 构建验证 + pytest 全通过
> 预计: ~300 LOC (C++ 测试) + 构建验证

---

## 目标

Phase A-N 各模块已通过单元测试，但缺少将所有模块串联的端到端集成测试。
此阶段补齐两块关键缺失：

1. `test_full_pipeline.cpp` 当前为空壳 (2 个 TODO)，需要填充为真实测试
2. Python bindings (`_sbf5_cpp`) 尚未实际构建和运行 pytest

---

## Step O1: 2DOF 端到端测试

### 文件
`test/test_full_pipeline.cpp`

### 测试内容
```cpp
TEST_SUITE("FullPipeline_2DOF") {

TEST_CASE("plan_2dof_simple") {
    // 1. 加载 Robot
    Robot robot = Robot::from_json(std::string(SBF_DATA_DIR) + "/2dof_planar.json");
    CHECK(robot.n_joints() == 2);

    // 2. 定义 start/goal + obstacle
    Eigen::VectorXd start(2), goal(2);
    start << 0.1, 0.2;
    goal  << 2.5, 1.5;
    Obstacle obs;
    obs.center = Eigen::Vector3d(1.0, 0.0, 0.0);
    obs.half_sizes = Eigen::Vector3d(0.2, 0.2, 0.2);

    // 3. SBFPlanner 一次性 plan
    SBFPlannerConfig config;
    config.grower.max_boxes = 500;
    config.grower.mode = GrowerConfig::Mode::Wavefront;
    SBFPlanner planner(robot, config);

    PlanResult result = planner.plan(start, goal, &obs, 1, 30000.0);

    // 4. 验证结果
    CHECK(result.success);
    CHECK(result.path.size() >= 2);           // 至少 start + goal
    CHECK(result.path.front().isApprox(start, 1e-6));
    CHECK(result.path.back().isApprox(goal, 1e-6));
    CHECK(result.path_length > 0.0);
    CHECK(result.n_boxes > 0);
    CHECK(result.planning_time_ms > 0.0);
}

TEST_CASE("plan_2dof_build_then_query") {
    Robot robot = Robot::from_json(std::string(SBF_DATA_DIR) + "/2dof_planar.json");
    Eigen::VectorXd start(2), goal(2);
    start << 0.5, 0.5;
    goal  << 2.0, 2.0;

    // 预构建
    SBFPlanner planner(robot);
    planner.build(start, goal, nullptr, 0, 30000.0);
    CHECK(planner.n_boxes() > 0);

    // 查询
    PlanResult result = planner.query(start, goal);
    CHECK(result.success);
    CHECK(result.path.size() >= 2);
}

TEST_CASE("viz_export_after_plan") {
    Robot robot = Robot::from_json(std::string(SBF_DATA_DIR) + "/2dof_planar.json");
    Eigen::VectorXd start(2), goal(2);
    start << 0.1, 0.2;
    goal  << 2.5, 1.5;
    Obstacle obs;
    obs.center = Eigen::Vector3d(1.0, 0.0, 0.0);
    obs.half_sizes = Eigen::Vector3d(0.2, 0.2, 0.2);

    SBFPlanner planner(robot);
    PlanResult result = planner.plan(start, goal, &obs, 1);
    REQUIRE(result.success);

    // VizExporter: snapshot
    std::string out = "test_output_pipeline_snapshot.json";
    sbf::viz::export_snapshot_json(out, robot, planner.boxes(),
        /* lect 需要从 planner 暴露, 如不可用则跳过 */
        &obs, 1, {start, goal});
    // 检查文件存在
    std::ifstream f(out);
    CHECK(f.good());
    // 清理
    std::remove(out.c_str());
}

}  // TEST_SUITE FullPipeline_2DOF
```

---

## Step O2: 7DOF Panda 端到端测试

### 测试内容
```cpp
TEST_SUITE("FullPipeline_7DOF") {

TEST_CASE("plan_panda_tabletop") {
    Robot robot = Robot::from_json(std::string(SBF_DATA_DIR) + "/panda.json");
    CHECK(robot.n_joints() == 7);

    Eigen::VectorXd start = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd goal(7);
    goal << 0.5, -0.3, 0.0, -1.5, 0.0, 1.2, 0.0;

    // 桌面 + 柱子
    std::vector<Obstacle> obstacles(2);
    obstacles[0].center = Eigen::Vector3d(0.5, 0.0, 0.4);
    obstacles[0].half_sizes = Eigen::Vector3d(0.3, 0.3, 0.02);
    obstacles[1].center = Eigen::Vector3d(0.5, 0.0, 0.2);
    obstacles[1].half_sizes = Eigen::Vector3d(0.05, 0.05, 0.2);

    SBFPlannerConfig config;
    config.grower.max_boxes = 2000;
    config.grower.n_roots = 50;
    config.grower.mode = GrowerConfig::Mode::Wavefront;

    SBFPlanner planner(robot, config);
    PlanResult result = planner.plan(start, goal,
                                     obstacles.data(), (int)obstacles.size(),
                                     60000.0);

    // 7DOF 可能因复杂度较高而失败, 此处允许失败但记录
    if (result.success) {
        CHECK(result.path.size() >= 2);
        CHECK(result.path_length > 0.0);
        CHECK(result.n_boxes > 10);
        MESSAGE("Panda plan: " << result.n_boxes << " boxes, "
                << result.path.size() << " waypoints, "
                << result.path_length << " length, "
                << result.planning_time_ms << " ms");
    } else {
        MESSAGE("Panda plan failed (expected in limited box budget)");
    }
}

}  // TEST_SUITE FullPipeline_7DOF
```

---

## Step O3: Voxel 端到端测试

### 测试内容 (追加到 test_full_pipeline.cpp)
```cpp
TEST_SUITE("FullPipeline_Voxel") {

TEST_CASE("voxel_collision_check_2dof") {
    Robot robot = Robot::from_json(std::string(SBF_DATA_DIR) + "/2dof_planar.json");
    Eigen::VectorXd q(2);
    q << 1.0, 0.5;

    FKState fk = robot.fk(q);

    // Robot voxels
    voxel::SparseVoxelGrid robot_grid(0.05);
    voxel::rasterise_robot_hull16(robot, fk, robot_grid, 8);
    CHECK(robot_grid.count_occupied() > 0);

    // Obstacle voxels (不与 robot 碰撞的位置)
    Obstacle far_obs;
    far_obs.center = Eigen::Vector3d(10.0, 10.0, 10.0);
    far_obs.half_sizes = Eigen::Vector3d(0.1, 0.1, 0.1);
    voxel::SparseVoxelGrid obs_grid(0.05);
    voxel::rasterise_box_obstacle(far_obs, obs_grid);
    CHECK(!robot_grid.collides(obs_grid));

    // Obstacle voxels (碰撞位置)
    Obstacle near_obs;
    near_obs.center = Eigen::Vector3d(0.5, 0.0, 0.0);
    near_obs.half_sizes = Eigen::Vector3d(1.0, 1.0, 1.0);
    voxel::SparseVoxelGrid obs_grid2(0.05);
    voxel::rasterise_box_obstacle(near_obs, obs_grid2);
    CHECK(robot_grid.collides(obs_grid2));
}

}  // TEST_SUITE FullPipeline_Voxel
```

---

## Step O4: CMake 集成

### CMakeLists.txt 变更
```cmake
# SBF5_TESTS list 追加:
    test_full_pipeline
```

`test_full_pipeline` 链接 `sbf5` + `doctest`，与其他测试共享 `SBF_DATA_DIR` 和 `WORKING_DIRECTORY` 设置。

---

## Step O5: Python Bindings 构建

### 构建命令
```powershell
cmake -B build_x64 -S . -G "Visual Studio 17 2022" -A x64 -DSBF_BUILD_PYTHON=ON
cmake --build build_x64 --config Release --target _sbf5_cpp
```

### 验证步骤
1. 确认 `build_x64/Release/_sbf5_cpp.pyd` 存在
2. 将 `build_x64/Release/` 加入 `PYTHONPATH`
3. 运行: `python -c "import sbf5; print(sbf5.__version__)"`

---

## Step O6: Python 测试运行

### 运行 pytest
```powershell
cd v5
$env:PYTHONPATH = "build_x64/Release;python"
python -m pytest python/tests/ -v --tb=short
```

### 预期通过的测试文件
| 文件 | 测试数 | 描述 |
|------|--------|------|
| `python/tests/test_sbf5.py` | ~8 | Interval, Obstacle, Robot, Plan, Boxes |
| `python/tests/test_viz.py` | ~10 | JSON load, Plotly figure generation, CLI |
| `python/tests/test_bench.py` | ~8 | PlanningResult, metrics, scenes, runner |

### 已知前提
- `test_viz.py`: 需要 `plotly >= 5.0` (`pip install plotly`)
- `test_bench.py` 中 OMPL 测试: 需要 WSL (Windows 上可能 skip)
- `test_bench.py` 中 IRIS-GCS 测试: 需要 pydrake (可 skip)

---

## 验收标准

- [x] `test_full_pipeline` 中 2DOF 端到端: plan → success, path 有效
- [x] `test_full_pipeline` 中 Voxel 端到端: 碰撞/无碰撞判定正确
- [x] 7DOF Panda: 运行不崩溃 (允许规划失败)
- [x] C++ 测试: 13/13 通过 (原 12 + `test_full_pipeline`)
- [x] `_sbf5_cpp.pyd` 成功构建
- [x] `python -c "import sbf5"` 无报错
- [x] `pytest python/tests/test_sbf5.py` 全部通过 (9/9)
- [x] `pytest python/tests/test_viz.py` 全部通过 (14/14)
- [x] `pytest python/tests/test_bench.py` 核心测试通过 (18/18)
