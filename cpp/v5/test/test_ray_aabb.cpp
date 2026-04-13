// Unit tests for ray_aabb.h: segment_in_box, ray_intersect_box, segment_in_box_union
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>
#include <sbf/core/ray_aabb.h>
#include <sbf/core/types.h>
#include <cmath>

using namespace sbf;

// Helper to create a BoxNode from lo/hi vectors
static BoxNode make_box(int id, const std::vector<std::pair<double,double>>& ivs) {
    BoxNode b;
    b.id = id;
    for (auto& [lo, hi] : ivs)
        b.joint_intervals.push_back(Interval(lo, hi));
    b.compute_volume();
    b.seed_config = b.center();
    return b;
}

// ─── segment_in_box ─────────────────────────────────────────────────────────

TEST_CASE("segment_in_box: both endpoints inside") {
    auto box = make_box(0, {{0, 1}, {0, 1}});
    Eigen::VectorXd a(2), b(2);
    a << 0.2, 0.3;
    b << 0.8, 0.7;
    CHECK(segment_in_box(a, b, box));
}

TEST_CASE("segment_in_box: one endpoint outside") {
    auto box = make_box(0, {{0, 1}, {0, 1}});
    Eigen::VectorXd a(2), b(2);
    a << 0.5, 0.5;
    b << 1.5, 0.5;
    CHECK_FALSE(segment_in_box(a, b, box));
}

TEST_CASE("segment_in_box: both endpoints outside") {
    auto box = make_box(0, {{0, 1}, {0, 1}});
    Eigen::VectorXd a(2), b(2);
    a << -0.5, 0.5;
    b << 1.5, 0.5;
    CHECK_FALSE(segment_in_box(a, b, box));
}

TEST_CASE("segment_in_box: endpoint on boundary") {
    auto box = make_box(0, {{0, 1}, {0, 1}});
    Eigen::VectorXd a(2), b(2);
    a << 0.0, 0.0;
    b << 1.0, 1.0;
    CHECK(segment_in_box(a, b, box));
}

// ─── ray_intersect_box ──────────────────────────────────────────────────────

TEST_CASE("ray_intersect_box: ray through center") {
    auto box = make_box(0, {{0, 2}, {0, 2}});
    Eigen::VectorXd origin(2), dir(2);
    origin << -1.0, 1.0;
    dir << 4.0, 0.0;  // horizontal ray through center
    auto [te, tx] = ray_intersect_box(origin, dir, box);
    CHECK(te == doctest::Approx(0.25));   // t where x=0: (-1+4t)=0 → t=0.25
    CHECK(tx == doctest::Approx(0.75));   // t where x=2: (-1+4t)=2 → t=0.75
}

TEST_CASE("ray_intersect_box: ray misses box") {
    auto box = make_box(0, {{0, 1}, {0, 1}});
    Eigen::VectorXd origin(2), dir(2);
    origin << 2.0, 2.0;
    dir << 1.0, 0.0;  // going further away
    auto [te, tx] = ray_intersect_box(origin, dir, box);
    CHECK(te > tx);  // miss: t_enter > t_exit
}

TEST_CASE("ray_intersect_box: ray parallel to slab, inside") {
    auto box = make_box(0, {{0, 2}, {0, 2}});
    Eigen::VectorXd origin(2), dir(2);
    origin << 1.0, 0.5;
    dir << 0.0, 1.0;  // vertical ray, x=1 is inside [0,2]
    auto [te, tx] = ray_intersect_box(origin, dir, box);
    // y: (0 - 0.5)/1 = -0.5, (2 - 0.5)/1 = 1.5
    CHECK(te == doctest::Approx(-0.5));
    CHECK(tx == doctest::Approx(1.5));
}

TEST_CASE("ray_intersect_box: ray parallel to slab, outside") {
    auto box = make_box(0, {{0, 1}, {0, 1}});
    Eigen::VectorXd origin(2), dir(2);
    origin << 2.0, 0.5;
    dir << 0.0, 1.0;  // x=2.0 is outside [0,1]
    auto [te, tx] = ray_intersect_box(origin, dir, box);
    CHECK(te > tx);  // miss
}

TEST_CASE("ray_intersect_box: 7D box") {
    // Simulate a typical 7-DOF robot C-space box
    auto box = make_box(0, {{-0.5, 0.5}, {0.3, 0.7}, {-0.2, 0.3},
                             {-1.0, -0.5}, {-0.1, 0.1}, {0.2, 0.6}, {1.0, 2.0}});
    Eigen::VectorXd a(7), b(7);
    a << 0.0, 0.5, 0.0, -0.7, 0.0, 0.4, 1.5;
    b << 0.1, 0.5, 0.1, -0.8, 0.0, 0.3, 1.6;
    // Both inside — segment_in_box should pass
    CHECK(segment_in_box(a, b, box));
    // Ray intersection for t∈[0,1] should overlap
    auto [te, tx] = ray_intersect_box(a, b - a, box);
    CHECK(te <= 0.0 + 1e-9);
    CHECK(tx >= 1.0 - 1e-9);
}

// ─── segment_in_box_union ───────────────────────────────────────────────────

TEST_CASE("segment_in_box_union: segment in single box") {
    auto box = make_box(1, {{0, 2}, {0, 2}});
    std::vector<BoxNode> boxes = {box};
    Eigen::VectorXd a(2), b(2);
    a << 0.5, 0.5;
    b << 1.5, 1.5;
    CHECK(segment_in_box_union(a, b, boxes, {1}));
}

TEST_CASE("segment_in_box_union: segment spans two touching boxes") {
    auto b1 = make_box(1, {{0, 1}, {0, 1}});
    auto b2 = make_box(2, {{1, 2}, {0, 1}});
    std::vector<BoxNode> boxes = {b1, b2};
    Eigen::VectorXd a(2), b(2);
    a << 0.2, 0.5;
    b << 1.8, 0.5;
    CHECK(segment_in_box_union(a, b, boxes, {1, 2}));
}

TEST_CASE("segment_in_box_union: gap between boxes") {
    auto b1 = make_box(1, {{0, 1}, {0, 1}});
    auto b2 = make_box(2, {{1.5, 2.5}, {0, 1}});
    std::vector<BoxNode> boxes = {b1, b2};
    Eigen::VectorXd a(2), b(2);
    a << 0.5, 0.5;
    b << 2.0, 0.5;
    CHECK_FALSE(segment_in_box_union(a, b, boxes, {1, 2}));
}

TEST_CASE("segment_in_box_union: overlapping boxes cover gap") {
    auto b1 = make_box(1, {{0, 1.5}, {0, 1}});
    auto b2 = make_box(2, {{1, 2.5}, {0, 1}});
    std::vector<BoxNode> boxes = {b1, b2};
    Eigen::VectorXd a(2), b(2);
    a << 0.2, 0.5;
    b << 2.3, 0.5;
    CHECK(segment_in_box_union(a, b, boxes, {1, 2}));
}

TEST_CASE("segment_in_box_union: three boxes chain") {
    auto b1 = make_box(1, {{0, 1}, {0, 1}});
    auto b2 = make_box(2, {{1, 2}, {0, 1}});
    auto b3 = make_box(3, {{2, 3}, {0, 1}});
    std::vector<BoxNode> boxes = {b1, b2, b3};
    Eigen::VectorXd a(2), b(2);
    a << 0.1, 0.5;
    b << 2.9, 0.5;
    CHECK(segment_in_box_union(a, b, boxes, {1, 2, 3}));
}

TEST_CASE("segment_in_box_union: endpoint outside all boxes") {
    auto b1 = make_box(1, {{0, 1}, {0, 1}});
    std::vector<BoxNode> boxes = {b1};
    Eigen::VectorXd a(2), b(2);
    a << -0.5, 0.5;
    b << 0.5, 0.5;
    CHECK_FALSE(segment_in_box_union(a, b, boxes, {1}));
}

TEST_CASE("segment_in_box_union: with prebuilt id_to_idx") {
    auto b1 = make_box(1, {{0, 1}, {0, 1}});
    auto b2 = make_box(2, {{1, 2}, {0, 1}});
    std::vector<BoxNode> boxes = {b1, b2};
    std::unordered_map<int, int> id_to_idx;
    for (int i = 0; i < (int)boxes.size(); ++i)
        id_to_idx[boxes[i].id] = i;

    Eigen::VectorXd a(2), b(2);
    a << 0.2, 0.5;
    b << 1.8, 0.5;
    CHECK(segment_in_box_union(a, b, boxes, {1, 2}, id_to_idx));
}

TEST_CASE("segment_in_box_union: diagonal through overlapping 7D boxes") {
    auto b1 = make_box(1, {{0, 0.6}, {0, 0.6}, {0, 0.6}, {0, 0.6},
                            {0, 0.6}, {0, 0.6}, {0, 0.6}});
    auto b2 = make_box(2, {{0.4, 1}, {0.4, 1}, {0.4, 1}, {0.4, 1},
                            {0.4, 1}, {0.4, 1}, {0.4, 1}});
    std::vector<BoxNode> boxes = {b1, b2};
    Eigen::VectorXd a(7), b(7);
    a << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    b << 0.9, 0.9, 0.9, 0.9, 0.9, 0.9, 0.9;
    CHECK(segment_in_box_union(a, b, boxes, {1, 2}));
}
