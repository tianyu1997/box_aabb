// SafeBoxForest v5 — test_planner (Phase H)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <sbf/planner/dijkstra.h>
#include <sbf/planner/path_extract.h>
#include <sbf/planner/path_smoother.h>
#include <sbf/planner/gcs_planner.h>
#include <sbf/planner/sbf_planner.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/forest/adjacency.h>
#include <sbf/core/types.h>
#include <sbf/core/robot.h>

#include <Eigen/Dense>
#include <vector>

using namespace sbf;

// ─── Helpers ────────────────────────────────────────────────────────────────

static BoxNode make_box(int id, std::vector<std::pair<double,double>> ivs) {
    BoxNode b;
    b.id = id;
    for (auto& [lo, hi] : ivs)
        b.joint_intervals.push_back({lo, hi});
    b.seed_config = b.center();
    b.compute_volume();
    return b;
}

static Robot make_2dof_robot() {
    // 2DOF planar robot with 1m links
    std::vector<DHParam> dh = {
        {0.0, 1.0, 0.0, 0.0, 0},
        {0.0, 1.0, 0.0, 0.0, 0},
    };
    JointLimits limits;
    limits.limits = {{-PI, PI}, {-PI, PI}};
    std::vector<double> radii = {0.05, 0.05};
    return Robot("test_2dof", dh, limits, std::nullopt, radii);
}

// ─── Dijkstra tests ─────────────────────────────────────────────────────────

TEST_SUITE("Dijkstra") {

    TEST_CASE("linear 3-box path: found") {
        // [0,1]x[0,1] -> [1,2]x[0,1] -> [2,3]x[0,1]
        std::vector<BoxNode> boxes = {
            make_box(0, {{0,1},{0,1}}),
            make_box(1, {{1,2},{0,1}}),
            make_box(2, {{2,3},{0,1}}),
        };
        auto adj = compute_adjacency(boxes);

        auto res = dijkstra_search(adj, boxes, 0, 2);
        CHECK(res.found);
        REQUIRE(res.box_sequence.size() == 3);
        CHECK(res.box_sequence[0] == 0);
        CHECK(res.box_sequence[1] == 1);
        CHECK(res.box_sequence[2] == 2);
        CHECK(res.total_cost > 0.0);
    }

    TEST_CASE("disconnected boxes: not found") {
        std::vector<BoxNode> boxes = {
            make_box(0, {{0,1},{0,1}}),
            make_box(1, {{5,6},{5,6}}),
        };
        auto adj = compute_adjacency(boxes);

        auto res = dijkstra_search(adj, boxes, 0, 1);
        CHECK_FALSE(res.found);
    }

    TEST_CASE("same start and goal box") {
        std::vector<BoxNode> boxes = { make_box(0, {{0,1},{0,1}}) };
        auto adj = compute_adjacency(boxes);

        auto res = dijkstra_search(adj, boxes, 0, 0);
        CHECK(res.found);
        CHECK(res.box_sequence.size() == 1);
        CHECK(res.total_cost == doctest::Approx(0.0));
    }

    TEST_CASE("cost is monotonically non-decreasing along path") {
        std::vector<BoxNode> boxes = {
            make_box(0, {{0,1},{0,1}}),
            make_box(1, {{1,2},{0,1}}),
            make_box(2, {{2,3},{0,1}}),
            make_box(3, {{3,4},{0,1}}),
        };
        auto adj = compute_adjacency(boxes);
        auto res = dijkstra_search(adj, boxes, 0, 3);
        CHECK(res.found);
        CHECK(res.total_cost > 0.0);
        // Total cost should equal sum of center-to-center distances
        double expected = 3.0;  // centers at 0.5, 1.5, 2.5, 3.5 → 3 hops of 1.0
        CHECK(res.total_cost == doctest::Approx(expected).epsilon(1e-6));
    }
}

// ─── PathExtract tests ──────────────────────────────────────────────────────

TEST_SUITE("PathExtract") {

    TEST_CASE("waypoints on shared face") {
        std::vector<BoxNode> boxes = {
            make_box(0, {{0,1},{0,2}}),
            make_box(1, {{1,3},{0,2}}),
        };
        Eigen::VectorXd start(2); start << 0.5, 1.0;
        Eigen::VectorXd goal(2);  goal  << 2.0, 1.0;

        auto path = extract_waypoints({0, 1}, boxes, start, goal);
        REQUIRE(path.size() == 3);  // start + 1 waypoint + goal

        // First is start
        CHECK(path[0][0] == doctest::Approx(0.5));
        CHECK(path[0][1] == doctest::Approx(1.0));

        // Waypoint should be on shared face: x=1, y=center of overlap=[0,2]=1.0
        CHECK(path[1][0] == doctest::Approx(1.0).epsilon(1e-10));
        CHECK(path[1][1] == doctest::Approx(1.0).epsilon(1e-10));

        // Last is goal
        CHECK(path[2][0] == doctest::Approx(2.0));
        CHECK(path[2][1] == doctest::Approx(1.0));
    }

    TEST_CASE("path endpoints match start and goal") {
        std::vector<BoxNode> boxes = {
            make_box(0, {{0,1},{0,1}}),
            make_box(1, {{1,2},{0,1}}),
            make_box(2, {{2,3},{0,1}}),
        };
        Eigen::VectorXd start(2); start << 0.2, 0.3;
        Eigen::VectorXd goal(2);  goal  << 2.8, 0.7;

        auto path = extract_waypoints({0, 1, 2}, boxes, start, goal);
        CHECK(path.front() == start);
        CHECK(path.back() == goal);
    }
}

// ─── PathSmoother tests ─────────────────────────────────────────────────────

TEST_SUITE("PathSmoother") {

    TEST_CASE("shortcut: path length non-increasing") {
        // Create a zig-zag path in free space (no obstacles)
        std::vector<Eigen::VectorXd> path;
        for (int i = 0; i <= 10; ++i) {
            Eigen::VectorXd q(2);
            q << i * 0.1, (i % 2 == 0) ? 0.0 : 0.3;
            path.push_back(q);
        }

        double len_before = path_length(path);

        Robot robot = make_2dof_robot();
        CollisionChecker checker(robot, {});
        SmootherConfig cfg;
        cfg.shortcut_max_iters = 50;
        cfg.segment_resolution = 5;

        auto shortened = shortcut(path, checker, cfg);
        double len_after = path_length(shortened);

        CHECK(len_after <= len_before + 1e-10);
        CHECK(shortened.front() == path.front());
        CHECK(shortened.back() == path.back());
    }

    TEST_CASE("smooth_moving_average: endpoints preserved") {
        std::vector<Eigen::VectorXd> path;
        for (int i = 0; i <= 5; ++i) {
            Eigen::VectorXd q(2);
            q << i * 0.5, (i % 2 == 0) ? 0.0 : 0.2;
            path.push_back(q);
        }

        std::vector<BoxNode> seq_boxes;
        for (int i = 0; i <= 5; ++i) {
            seq_boxes.push_back(make_box(
                i, {{i * 0.5 - 0.1, i * 0.5 + 0.6}, {-0.5, 0.5}}));
        }

        SmootherConfig cfg;
        cfg.smooth_window = 3;
        cfg.smooth_iters = 3;

        auto smoothed = smooth_moving_average(path, seq_boxes, cfg);
        CHECK(smoothed.front() == path.front());
        CHECK(smoothed.back() == path.back());
    }

    TEST_CASE("path_length: basic computation") {
        std::vector<Eigen::VectorXd> path;
        Eigen::VectorXd a(2); a << 0.0, 0.0;
        Eigen::VectorXd b(2); b << 3.0, 4.0;
        path.push_back(a);
        path.push_back(b);

        CHECK(path_length(path) == doctest::Approx(5.0));
    }
}

// ─── GCS fallback tests ─────────────────────────────────────────────────────

TEST_SUITE("GCSFallback") {

    TEST_CASE("gcs_plan_fallback: uses Dijkstra internally") {
        std::vector<BoxNode> boxes = {
            make_box(0, {{0,1},{0,1}}),
            make_box(1, {{1,2},{0,1}}),
        };
        auto adj = compute_adjacency(boxes);

        Eigen::VectorXd start(2); start << 0.5, 0.5;
        Eigen::VectorXd goal(2);  goal  << 1.5, 0.5;

        auto res = gcs_plan_fallback(adj, boxes, start, goal);
        CHECK(res.found);
        CHECK(res.path.size() >= 3);  // start + wp + goal
        CHECK(res.path.front()[0] == doctest::Approx(0.5));
        CHECK(res.path.back()[0] == doctest::Approx(1.5));
    }
}

// ─── SBFPlanner pipeline config tests (Phase R) ─────────────────────────────

TEST_SUITE("PlannerConfig") {

    TEST_CASE("planner_with_analytical: plan succeeds") {
        auto robot = make_2dof_robot();
        SBFPlannerConfig cfg;
        cfg.grower.max_boxes = 200;
        cfg.grower.mode = GrowerConfig::Mode::WAVEFRONT;
        cfg.endpoint_source.source = EndpointSource::Analytical;

        SBFPlanner planner(robot, cfg);
        Eigen::VectorXd start(2); start << 0.5, 0.5;
        Eigen::VectorXd goal(2);  goal  << 2.0, 1.0;
        // obstacle far from workspace
        Obstacle obs{5.0f, 5.0f, 0.0f, 6.0f, 6.0f, 1.0f};
        auto result = planner.plan(start, goal, &obs, 1, 30000.0);
        CHECK(result.success);
        CHECK(result.path.size() > 0);
        CHECK(result.build_time_ms > 0.0);
        CHECK(result.lect_time_ms > 0.0);
    }

    TEST_CASE("planner_with_linkiaabb_grid: plan succeeds") {
        auto robot = make_2dof_robot();
        SBFPlannerConfig cfg;
        cfg.grower.max_boxes = 200;
        cfg.grower.mode = GrowerConfig::Mode::WAVEFRONT;
        cfg.envelope_type.type = EnvelopeType::LinkIAABB_Grid;

        SBFPlanner planner(robot, cfg);
        Eigen::VectorXd start(2); start << 0.5, 0.5;
        Eigen::VectorXd goal(2);  goal  << 2.0, 1.0;
        Obstacle obs{5.0f, 5.0f, 0.0f, 6.0f, 6.0f, 1.0f};
        auto result = planner.plan(start, goal, &obs, 1, 30000.0);
        CHECK(result.success);
        CHECK(result.path.size() > 0);
    }

    TEST_CASE("default config: build_time_ms and lect_time_ms populated") {
        auto robot = make_2dof_robot();
        SBFPlannerConfig cfg;
        cfg.grower.max_boxes = 100;
        cfg.grower.mode = GrowerConfig::Mode::WAVEFRONT;

        SBFPlanner planner(robot, cfg);
        Eigen::VectorXd start(2); start << 0.5, 0.5;
        Eigen::VectorXd goal(2);  goal  << 2.0, 1.0;
        auto result = planner.plan(start, goal, nullptr, 0, 30000.0);
        CHECK(result.success);
        CHECK(result.build_time_ms >= result.lect_time_ms);
        CHECK(result.lect_time_ms >= 0.0);
    }
}
