// SafeBoxForest v5 鈥?test_coarsen (Phase G verification)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <sbf/forest/coarsen.h>
#include <sbf/forest/adjacency.h>
#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/scene/collision_checker.h>

#include <algorithm>
#include <cmath>
#include <vector>

using namespace sbf;

// 鈹€鈹€鈹€ Helpers 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

// Collision checker with no obstacles 鈫?check_box always returns false (safe)
static CollisionChecker make_safe_checker() {
    Robot robot = Robot::from_json("data/2dof_planar.json");
    std::vector<Obstacle> empty_obs;
    return CollisionChecker(robot, empty_obs);
}

static BoxNode make_box(int id, std::vector<Interval> ivs) {
    BoxNode b;
    b.id = id;
    b.joint_intervals = ivs;
    b.seed_config = Eigen::VectorXd::Zero(static_cast<int>(ivs.size()));
    for (int d = 0; d < static_cast<int>(ivs.size()); ++d)
        b.seed_config[d] = ivs[d].center();
    b.compute_volume();
    return b;
}

static double total_volume(const std::vector<BoxNode>& boxes) {
    double v = 0.0;
    for (auto& b : boxes) v += b.volume;
    return v;
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
TEST_SUITE("DimensionSweep") {

TEST_CASE("coarsen_forest: two exact-touching boxes merge to 1") {
    auto checker = make_safe_checker();
    std::vector<BoxNode> boxes;
    boxes.push_back(make_box(0, {{0.0, 1.0}, {0.0, 1.0}}));
    boxes.push_back(make_box(1, {{1.0, 2.0}, {0.0, 1.0}}));

    auto r = coarsen_forest(boxes, checker);
    CHECK(r.merges_performed == 1);
    CHECK(r.boxes_before == 2);
    CHECK(r.boxes_after == 1);
    REQUIRE(boxes.size() == 1);

    // Merged box spans [0,2] x [0,1]
    CHECK(boxes[0].joint_intervals[0].lo == doctest::Approx(0.0));
    CHECK(boxes[0].joint_intervals[0].hi == doctest::Approx(2.0));
    CHECK(boxes[0].joint_intervals[1].lo == doctest::Approx(0.0));
    CHECK(boxes[0].joint_intervals[1].hi == doctest::Approx(1.0));
}

TEST_CASE("coarsen_forest: two non-touching boxes 鈫?no merge") {
    auto checker = make_safe_checker();
    std::vector<BoxNode> boxes;
    boxes.push_back(make_box(0, {{0.0, 1.0}, {0.0, 1.0}}));
    boxes.push_back(make_box(1, {{2.0, 3.0}, {0.0, 1.0}}));

    auto r = coarsen_forest(boxes, checker);
    CHECK(r.merges_performed == 0);
    CHECK(boxes.size() == 2);
}

TEST_CASE("coarsen_forest: touching but misaligned 鈫?no merge") {
    auto checker = make_safe_checker();
    std::vector<BoxNode> boxes;
    // Same dim-0 boundary but different dim-1 ranges
    boxes.push_back(make_box(0, {{0.0, 1.0}, {0.0, 1.0}}));
    boxes.push_back(make_box(1, {{1.0, 2.0}, {0.5, 1.5}}));

    auto r = coarsen_forest(boxes, checker);
    CHECK(r.merges_performed == 0);
    CHECK(boxes.size() == 2);
}

TEST_CASE("coarsen_forest: 4 boxes in a row 鈫?merge to 1") {
    auto checker = make_safe_checker();
    std::vector<BoxNode> boxes;
    for (int i = 0; i < 4; ++i)
        boxes.push_back(make_box(i, {{i * 1.0, (i + 1) * 1.0}, {0.0, 1.0}}));

    double vol_before = total_volume(boxes);
    auto r = coarsen_forest(boxes, checker);
    CHECK(r.merges_performed == 3);
    CHECK(r.boxes_after == 1);

    // Volume conserved
    CHECK(total_volume(boxes) == doctest::Approx(vol_before));
}

TEST_CASE("coarsen_forest: 2x2 grid merges") {
    auto checker = make_safe_checker();
    std::vector<BoxNode> boxes;
    // [0,1]x[0,1], [1,2]x[0,1], [0,1]x[1,2], [1,2]x[1,2]
    boxes.push_back(make_box(0, {{0.0, 1.0}, {0.0, 1.0}}));
    boxes.push_back(make_box(1, {{1.0, 2.0}, {0.0, 1.0}}));
    boxes.push_back(make_box(2, {{0.0, 1.0}, {1.0, 2.0}}));
    boxes.push_back(make_box(3, {{1.0, 2.0}, {1.0, 2.0}}));

    double vol_before = total_volume(boxes);
    auto r = coarsen_forest(boxes, checker);
    CHECK(r.boxes_after == 1);
    CHECK(total_volume(boxes) == doctest::Approx(vol_before));
}

TEST_CASE("coarsen_forest: empty box list 鈫?no error") {
    auto checker = make_safe_checker();
    std::vector<BoxNode> boxes;
    auto r = coarsen_forest(boxes, checker);
    CHECK(r.merges_performed == 0);
    CHECK(r.boxes_after == 0);
}

TEST_CASE("coarsen_forest: single box 鈫?no merge") {
    auto checker = make_safe_checker();
    std::vector<BoxNode> boxes;
    boxes.push_back(make_box(0, {{0.0, 1.0}, {0.0, 1.0}}));
    auto r = coarsen_forest(boxes, checker);
    CHECK(r.merges_performed == 0);
    CHECK(r.boxes_after == 1);
}

}  // TEST_SUITE("DimensionSweep")

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
TEST_SUITE("GreedyCoarsen_Basic") {

TEST_CASE("coarsen_greedy: two adjacent boxes 鈫?merge") {
    auto checker = make_safe_checker();
    std::vector<BoxNode> boxes;
    boxes.push_back(make_box(0, {{0.0, 1.0}, {0.0, 1.0}}));
    boxes.push_back(make_box(1, {{1.0, 2.0}, {0.0, 1.0}}));

    GreedyCoarsenConfig cfg;
    auto r = coarsen_greedy(boxes, checker, cfg);
    CHECK(r.merges_performed == 1);
    CHECK(r.boxes_after == 1);
}

TEST_CASE("coarsen_greedy: score ordering 鈥?tighter pairs first") {
    auto checker = make_safe_checker();
    // Tight pair: [0,1]x[0,1] + [1,2]x[0,1] 鈫?score = 2/(1+1) = 1.0
    // Loose pair: [0,1]x[0,1] + [1,3]x[0,1] 鈫?score = 3/(1+2) = 1.0
    // Another: [0,1]x[0,1] + [2,3]x[0,1] 鈫?not adjacent
    std::vector<BoxNode> boxes;
    boxes.push_back(make_box(0, {{0.0, 1.0}, {0.0, 1.0}}));
    boxes.push_back(make_box(1, {{1.0, 2.0}, {0.0, 1.0}}));
    boxes.push_back(make_box(2, {{2.0, 3.0}, {0.0, 1.0}}));

    GreedyCoarsenConfig cfg;
    auto r = coarsen_greedy(boxes, checker, cfg);
    // Should merge all 3 in 2 rounds (or 1 round merges first pair)
    CHECK(r.merges_performed >= 1);
    CHECK(r.boxes_after <= 2);
}

TEST_CASE("coarsen_greedy: target_boxes stops early") {
    auto checker = make_safe_checker();
    std::vector<BoxNode> boxes;
    for (int i = 0; i < 4; ++i)
        boxes.push_back(make_box(i, {{i * 1.0, (i + 1) * 1.0}, {0.0, 1.0}}));

    GreedyCoarsenConfig cfg;
    cfg.target_boxes = 3;
    auto r = coarsen_greedy(boxes, checker, cfg);
    CHECK(static_cast<int>(boxes.size()) <= 3);
    CHECK(r.boxes_after <= 3);
}

TEST_CASE("coarsen_greedy: already at target 鈫?no merges") {
    auto checker = make_safe_checker();
    std::vector<BoxNode> boxes;
    boxes.push_back(make_box(0, {{0.0, 1.0}, {0.0, 1.0}}));

    GreedyCoarsenConfig cfg;
    cfg.target_boxes = 5;
    auto r = coarsen_greedy(boxes, checker, cfg);
    CHECK(r.merges_performed == 0);
}

}  // TEST_SUITE("GreedyCoarsen_Basic")

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
TEST_SUITE("GreedyCoarsen_Edge") {

TEST_CASE("coarsen_greedy: empty box list 鈫?no error") {
    auto checker = make_safe_checker();
    std::vector<BoxNode> boxes;
    GreedyCoarsenConfig cfg;
    auto r = coarsen_greedy(boxes, checker, cfg);
    CHECK(r.merges_performed == 0);
    CHECK(r.boxes_after == 0);
}

TEST_CASE("coarsen_greedy: single box 鈫?no error") {
    auto checker = make_safe_checker();
    std::vector<BoxNode> boxes;
    boxes.push_back(make_box(0, {{0.0, 1.0}, {0.0, 1.0}}));

    GreedyCoarsenConfig cfg;
    auto r = coarsen_greedy(boxes, checker, cfg);
    CHECK(r.merges_performed == 0);
    CHECK(r.boxes_after == 1);
}

TEST_CASE("coarsen_greedy: non-adjacent boxes 鈫?no merge") {
    auto checker = make_safe_checker();
    std::vector<BoxNode> boxes;
    boxes.push_back(make_box(0, {{0.0, 1.0}, {0.0, 1.0}}));
    boxes.push_back(make_box(1, {{5.0, 6.0}, {5.0, 6.0}}));

    GreedyCoarsenConfig cfg;
    auto r = coarsen_greedy(boxes, checker, cfg);
    CHECK(r.merges_performed == 0);
    CHECK(r.boxes_after == 2);
}

TEST_CASE("coarsen_greedy: score_threshold = 0 鈫?no merges") {
    auto checker = make_safe_checker();
    std::vector<BoxNode> boxes;
    boxes.push_back(make_box(0, {{0.0, 1.0}, {0.0, 1.0}}));
    boxes.push_back(make_box(1, {{1.0, 2.0}, {0.0, 1.0}}));

    GreedyCoarsenConfig cfg;
    cfg.score_threshold = 0.0;
    auto r = coarsen_greedy(boxes, checker, cfg);
    CHECK(r.merges_performed == 0);
}

TEST_CASE("coarsen_greedy: max_rounds = 0 鈫?no merges") {
    auto checker = make_safe_checker();
    std::vector<BoxNode> boxes;
    boxes.push_back(make_box(0, {{0.0, 1.0}, {0.0, 1.0}}));
    boxes.push_back(make_box(1, {{1.0, 2.0}, {0.0, 1.0}}));

    GreedyCoarsenConfig cfg;
    cfg.max_rounds = 0;
    auto r = coarsen_greedy(boxes, checker, cfg);
    CHECK(r.merges_performed == 0);
}

}  // TEST_SUITE("GreedyCoarsen_Edge")
