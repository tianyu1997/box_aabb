// SafeBoxForest v5 — test_gcs_drake (Phase L)
// Tests expand_corridor (always) + Drake GCS plan (only when SBF_HAS_DRAKE).

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <sbf/planner/gcs_planner.h>
#include <sbf/planner/dijkstra.h>
#include <sbf/forest/adjacency.h>
#include <sbf/core/types.h>

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

// ─── expand_corridor tests ──────────────────────────────────────────────────

TEST_SUITE("ExpandCorridor") {

    TEST_CASE("expand_corridor_basic: 3-box chain, hops=1 expands to all") {
        // Chain: 0 -- 1 -- 2
        std::vector<BoxNode> boxes = {
            make_box(0, {{0,1},{0,1}}),
            make_box(1, {{1,2},{0,1}}),
            make_box(2, {{2,3},{0,1}}),
        };
        auto adj = compute_adjacency(boxes);

        // Path is just the middle box
        auto corridor = expand_corridor(adj, {1}, 1);
        // hops=1 should reach 0 and 2
        CHECK(corridor.count(0));
        CHECK(corridor.count(1));
        CHECK(corridor.count(2));
        CHECK(corridor.size() == 3);
    }

    TEST_CASE("expand_corridor: hops=0 returns only path boxes") {
        std::vector<BoxNode> boxes = {
            make_box(0, {{0,1},{0,1}}),
            make_box(1, {{1,2},{0,1}}),
            make_box(2, {{2,3},{0,1}}),
        };
        auto adj = compute_adjacency(boxes);

        auto corridor = expand_corridor(adj, {0, 1}, 0);
        CHECK(corridor.size() == 2);
        CHECK(corridor.count(0));
        CHECK(corridor.count(1));
        CHECK_FALSE(corridor.count(2));
    }

    TEST_CASE("expand_corridor: isolated box stays isolated") {
        // Boxes 0-1 adjacent, box 2 isolated
        std::vector<BoxNode> boxes = {
            make_box(0, {{0,1},{0,1}}),
            make_box(1, {{1,2},{0,1}}),
            make_box(2, {{5,6},{5,6}}),
        };
        auto adj = compute_adjacency(boxes);

        auto corridor = expand_corridor(adj, {0}, 3);
        CHECK(corridor.count(0));
        CHECK(corridor.count(1));
        CHECK_FALSE(corridor.count(2));
    }

    TEST_CASE("expand_corridor: 5-box grid, hops=2 reaches all") {
        // Cross pattern: center(2) adjacent to 0,1,3,4
        //   0:[0,1]x[1,2]  1:[1,2]x[1,2]
        //   2:[0,1]x[0,1]  3:[1,2]x[0,1]
        //   4:[-1,0]x[0,1]
        std::vector<BoxNode> boxes = {
            make_box(0, {{0,1},{1,2}}),
            make_box(1, {{1,2},{1,2}}),
            make_box(2, {{0,1},{0,1}}),
            make_box(3, {{1,2},{0,1}}),
            make_box(4, {{-1,0},{0,1}}),
        };
        auto adj = compute_adjacency(boxes);

        // Start from box 4 only
        auto corridor = expand_corridor(adj, {4}, 2);
        // 4->2 (hop 1), 2->0,3 (hop 2), plus 0->1 is hop 3
        CHECK(corridor.count(4));
        CHECK(corridor.count(2));
        CHECK(corridor.count(0));
        CHECK(corridor.count(3));
    }
}

// ─── Drake GCS tests (only compiled with Drake) ─────────────────────────────

#ifdef SBF_HAS_DRAKE

TEST_SUITE("GCSDrake") {

    TEST_CASE("gcs_plan_2dof_simple: 2-box path returns success") {
        std::vector<BoxNode> boxes = {
            make_box(0, {{0,1},{0,1}}),
            make_box(1, {{1,2},{0,1}}),
        };
        auto adj = compute_adjacency(boxes);

        Eigen::VectorXd start(2); start << 0.5, 0.5;
        Eigen::VectorXd goal(2);  goal  << 1.5, 0.5;

        GCSConfig cfg;
        cfg.corridor_hops = 0;

        auto res = gcs_plan(adj, boxes, start, goal, cfg);
        CHECK(res.found);
        CHECK(res.path.size() >= 3);
        CHECK(res.path.front()[0] == doctest::Approx(0.5));
        CHECK(res.path.back()[0] == doctest::Approx(1.5));
    }

    TEST_CASE("gcs_plan_cost_vs_dijkstra: GCS cost <= Dijkstra cost") {
        std::vector<BoxNode> boxes = {
            make_box(0, {{0,1},{0,1}}),
            make_box(1, {{1,2},{0,1}}),
            make_box(2, {{2,3},{0,1}}),
        };
        auto adj = compute_adjacency(boxes);

        Eigen::VectorXd start(2); start << 0.5, 0.5;
        Eigen::VectorXd goal(2);  goal  << 2.5, 0.5;

        auto dij = dijkstra_search(adj, boxes, 0, 2);
        REQUIRE(dij.found);

        GCSConfig cfg;
        auto gcs_res = gcs_plan(adj, boxes, start, goal, cfg);
        CHECK(gcs_res.found);
        // GCS should find equal or better path
        CHECK(gcs_res.cost <= dij.total_cost + 1e-6);
    }

    TEST_CASE("gcs_plan_fallback_on_failure: unreachable goal uses fallback") {
        std::vector<BoxNode> boxes = {
            make_box(0, {{0,1},{0,1}}),
            make_box(1, {{5,6},{5,6}}),
        };
        auto adj = compute_adjacency(boxes);

        Eigen::VectorXd start(2); start << 0.5, 0.5;
        Eigen::VectorXd goal(2);  goal  << 5.5, 5.5;

        GCSConfig cfg;
        auto res = gcs_plan(adj, boxes, start, goal, cfg);
        // Dijkstra also cannot connect, so result should be not found
        CHECK_FALSE(res.found);
    }
}

#endif  // SBF_HAS_DRAKE
