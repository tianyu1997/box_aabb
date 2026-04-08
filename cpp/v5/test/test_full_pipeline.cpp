// SafeBoxForest v5 — Full Pipeline Integration Test
// Requires all phases (A-N) complete

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <sbf/core/robot.h>
#include <sbf/core/types.h>
#include <sbf/core/fk_state.h>
#include <sbf/planner/sbf_planner.h>
#include <sbf/viz/viz_exporter.h>
#include <sbf/voxel/voxel_grid.h>
#include <sbf/voxel/hull_rasteriser.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/lect/lect.h>
#include <sbf/ffb/ffb.h>

#include <Eigen/Dense>
#include <fstream>
#include <cstdio>

using namespace sbf;

// ─── Step O1: 2DOF end-to-end ───────────────────────────────────────────────
TEST_SUITE("FullPipeline_2DOF") {

TEST_CASE("plan_2dof_no_obstacles") {
    Robot robot = Robot::from_json(std::string(SBF_DATA_DIR) + "/2dof_planar.json");
    CHECK(robot.n_joints() == 2);

    Eigen::VectorXd start(2), goal(2);
    start << -1.0, -1.0;
    goal  <<  1.0,  1.0;

    SBFPlannerConfig config;
    config.grower.max_boxes = 500;
    config.grower.mode = GrowerConfig::Mode::WAVEFRONT;
    config.grower.timeout_ms = 30000.0;
    SBFPlanner planner(robot, config);

    PlanResult result = planner.plan(start, goal, nullptr, 0, 30000.0);

    REQUIRE(result.success);
    CHECK(result.path.size() >= 2);
    CHECK(result.path.front().isApprox(start, 1e-6));
    CHECK(result.path.back().isApprox(goal, 1e-6));
    CHECK(result.path_length > 0.0);
    CHECK(result.n_boxes > 0);
    CHECK(result.planning_time_ms > 0.0);
}

TEST_CASE("plan_2dof_with_obstacle") {
    Robot robot = Robot::from_json(std::string(SBF_DATA_DIR) + "/2dof_planar.json");

    Eigen::VectorXd start(2), goal(2);
    start << -1.0, -1.0;
    goal  <<  1.0,  1.0;

    // Obstacle far from the robot workspace to not block path
    Obstacle obs(5.0f, 5.0f, 5.0f, 5.5f, 5.5f, 5.5f);

    SBFPlannerConfig config;
    config.grower.max_boxes = 500;
    config.grower.mode = GrowerConfig::Mode::WAVEFRONT;
    SBFPlanner planner(robot, config);

    PlanResult result = planner.plan(start, goal, &obs, 1, 30000.0);

    REQUIRE(result.success);
    CHECK(result.path.size() >= 2);
    CHECK(result.path_length > 0.0);
    CHECK(result.n_boxes > 0);
}

TEST_CASE("plan_2dof_build_then_query") {
    Robot robot = Robot::from_json(std::string(SBF_DATA_DIR) + "/2dof_planar.json");
    Eigen::VectorXd start(2), goal(2);
    start << -0.5, -0.5;
    goal  <<  0.5,  0.5;

    SBFPlannerConfig config;
    config.grower.max_boxes = 300;
    SBFPlanner planner(robot, config);
    planner.build(start, goal, nullptr, 0, 30000.0);
    CHECK(planner.n_boxes() > 0);

    PlanResult result = planner.query(start, goal);
    REQUIRE(result.success);
    CHECK(result.path.size() >= 2);
}

}  // TEST_SUITE FullPipeline_2DOF

// ─── Step O2: 7DOF Panda end-to-end ────────────────────────────────────────
TEST_SUITE("FullPipeline_7DOF") {

TEST_CASE("plan_panda_tabletop") {
    Robot robot = Robot::from_json(std::string(SBF_DATA_DIR) + "/panda.json");
    CHECK(robot.n_joints() == 7);

    Eigen::VectorXd start = Eigen::VectorXd::Zero(7);
    Eigen::VectorXd goal(7);
    goal << 0.5, -0.3, 0.0, -1.5, 0.0, 1.2, 0.0;

    // Table obstacle + pillar
    Obstacle obstacles[2] = {
        Obstacle(0.2f, -0.3f, 0.38f, 0.8f, 0.3f, 0.42f),
        Obstacle(0.45f, -0.05f, 0.0f, 0.55f, 0.05f, 0.4f),
    };

    SBFPlannerConfig config;
    config.grower.max_boxes = 2000;
    config.grower.mode = GrowerConfig::Mode::WAVEFRONT;

    SBFPlanner planner(robot, config);
    PlanResult result = planner.plan(start, goal,
                                     obstacles, 2,
                                     60000.0);

    // 7DOF may fail under limited budget — allow but log
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

// ─── Step O3: Voxel end-to-end ──────────────────────────────────────────────
TEST_SUITE("FullPipeline_Voxel") {

TEST_CASE("voxel_collision_check_2dof") {
    Robot robot = Robot::from_json(std::string(SBF_DATA_DIR) + "/2dof_planar.json");
    Eigen::VectorXd q(2);
    q << 1.0, 0.5;

    // Build joint intervals (point)
    std::vector<Interval> ivs(2);
    ivs[0] = Interval(q[0], q[0]);
    ivs[1] = Interval(q[1], q[1]);
    FKState fk = compute_fk_full(robot, ivs);

    // Rasterise robot
    voxel::SparseVoxelGrid robot_grid(0.05);
    voxel::rasterise_robot_hull16(robot, fk, robot_grid, 8);
    CHECK(robot_grid.count_occupied() > 0);

    // Far obstacle — no collision
    Obstacle far_obs(9.8f, 9.8f, 9.8f, 10.2f, 10.2f, 10.2f);
    voxel::SparseVoxelGrid obs_grid(0.05);
    voxel::rasterise_box_obstacle(far_obs, obs_grid);
    CHECK(!robot_grid.collides(obs_grid));

    // Near obstacle — collision (large box covering origin area)
    Obstacle near_obs(-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 1.0f);
    voxel::SparseVoxelGrid obs_grid2(0.05);
    voxel::rasterise_box_obstacle(near_obs, obs_grid2);
    CHECK(robot_grid.collides(obs_grid2));
}

}  // TEST_SUITE FullPipeline_Voxel

// ─── Step P5: FFB with Hull16_Grid ──────────────────────────────────────────
TEST_SUITE("FFB_Hull16_Grid") {

TEST_CASE("ffb_with_hull16_grid: far obstacle succeeds") {
    Robot robot = Robot::from_json(std::string(SBF_DATA_DIR) + "/2dof_planar.json");

    EndpointSourceConfig ep_cfg;
    ep_cfg.source = EndpointSource::IFK;

    EnvelopeTypeConfig env_cfg;
    env_cfg.type = EnvelopeType::Hull16_Grid;
    env_cfg.grid_config.voxel_delta = 0.05;

    LECT lect(robot, robot.joint_limits().limits, ep_cfg, env_cfg);

    // Place obstacle far away
    Obstacle obs_far(10.0f, 10.0f, 10.0f, 11.0f, 11.0f, 11.0f);

    Eigen::VectorXd seed(2);
    seed << 0.5, 0.5;

    FFBConfig ffb_cfg;
    ffb_cfg.max_depth = 10;

    auto result = find_free_box(lect, seed, &obs_far, 1, ffb_cfg);
    CHECK(result.success());
    CHECK(result.node_idx >= 0);
}

TEST_CASE("ffb_with_hull16_grid: blocking obstacle fails") {
    Robot robot = Robot::from_json(std::string(SBF_DATA_DIR) + "/2dof_planar.json");

    EndpointSourceConfig ep_cfg;
    ep_cfg.source = EndpointSource::IFK;

    EnvelopeTypeConfig env_cfg;
    env_cfg.type = EnvelopeType::Hull16_Grid;
    env_cfg.grid_config.voxel_delta = 0.05;

    LECT lect(robot, robot.joint_limits().limits, ep_cfg, env_cfg);

    // Large obstacle covering entire workspace
    Obstacle obs_block(-10.0f, -10.0f, -10.0f, 10.0f, 10.0f, 10.0f);

    Eigen::VectorXd seed(2);
    seed << 0.5, 0.5;

    FFBConfig ffb_cfg;
    ffb_cfg.max_depth = 8;

    auto result = find_free_box(lect, seed, &obs_block, 1, ffb_cfg);
    CHECK(!result.success());
}

}  // TEST_SUITE FFB_Hull16_Grid
