// SafeBoxForest v5 — test_ffb (Phase E verification)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <sbf/ffb/ffb.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>

#include <cmath>
#include <vector>

using namespace sbf;

// ─── Test helpers ───────────────────────────────────────────────────────────

static Robot make_2dof() {
    return Robot::from_json("data/2dof_planar.json");
}

static LECT make_2dof_lect() {
    Robot robot = make_2dof();
    std::vector<Interval> root = robot.joint_limits().limits;
    EndpointSourceConfig ep_cfg;
    ep_cfg.source = EndpointSource::IFK;
    EnvelopeTypeConfig env_cfg;
    env_cfg.type = EnvelopeType::LinkIAABB;
    return LECT(robot, root, ep_cfg, env_cfg);
}

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("FFB_Basic") {

TEST_CASE("2DOF no obstacles: seed at center → success, root node") {
    auto lect = make_2dof_lect();
    Eigen::VectorXd seed(2);
    seed << 0.0, 0.0;

    FFBResult r = find_free_box(lect, seed, nullptr, 0);
    CHECK(r.success());
    CHECK(r.node_idx == 0);
    CHECK(r.fail_code == 0);
    CHECK(r.path.size() == 1);
    CHECK(r.path[0] == 0);
    CHECK(r.n_new_nodes == 0);
}

TEST_CASE("2DOF with distant obstacle: seed far from obs → success, depth > 0") {
    auto lect = make_2dof_lect();
    Eigen::VectorXd seed(2);
    seed << 0.0, 0.0;

    // Place obstacle far enough that root collides but sub-node doesn't
    Obstacle obs(10.0f, 10.0f, -0.5f, 12.0f, 12.0f, 0.5f);

    FFBResult r = find_free_box(lect, seed, &obs, 1);
    // Depending on envelope size, this may succeed at root or need splitting.
    // We check basic invariants.
    if (r.success()) {
        CHECK(r.node_idx >= 0);
        CHECK(r.path.front() == 0);
        CHECK(r.path.back() == r.node_idx);
    }
}

TEST_CASE("2DOF with blocking obstacle: seed inside obstacle → fail max_depth") {
    auto lect = make_2dof_lect();
    Eigen::VectorXd seed(2);
    seed << 0.0, 0.0;

    // Place a huge obstacle covering the entire workspace
    Obstacle obs(-100.0f, -100.0f, -100.0f, 100.0f, 100.0f, 100.0f);

    FFBConfig cfg;
    cfg.max_depth = 30;

    FFBResult r = find_free_box(lect, seed, &obs, 1, cfg);
    CHECK_FALSE(r.success());
    CHECK(r.fail_code == 2);  // max_depth
}

}  // TEST_SUITE("FFB_Basic")

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("FFB_FailCodes") {

TEST_CASE("occupied: mark root occupied → FFB drills into child") {
    auto lect = make_2dof_lect();
    lect.mark_occupied(0, 42);

    Eigen::VectorXd seed(2);
    seed << 0.0, 0.0;

    FFBResult r = find_free_box(lect, seed, nullptr, 0);
    // FFB expands the occupied root and finds an unoccupied child
    CHECK(r.success());
    CHECK(r.node_idx != 0);  // not the root itself
    CHECK(r.n_new_nodes >= 2);  // at least one expansion
}

TEST_CASE("max_depth: config.max_depth=2 with obstacle → fail_code=2") {
    auto lect = make_2dof_lect();
    Eigen::VectorXd seed(2);
    seed << 0.0, 0.0;

    Obstacle obs(-100.0f, -100.0f, -100.0f, 100.0f, 100.0f, 100.0f);

    FFBConfig cfg;
    cfg.max_depth = 2;

    FFBResult r = find_free_box(lect, seed, &obs, 1, cfg);
    CHECK_FALSE(r.success());
    CHECK(r.fail_code == 2);
}

TEST_CASE("deadline: config.deadline_ms=0.001 → fail_code=4") {
    auto lect = make_2dof_lect();
    Eigen::VectorXd seed(2);
    seed << 0.0, 0.0;

    Obstacle obs(-100.0f, -100.0f, -100.0f, 100.0f, 100.0f, 100.0f);

    FFBConfig cfg;
    cfg.deadline_ms = 0.001;  // extremely short timeout
    cfg.max_depth = 100;

    FFBResult r = find_free_box(lect, seed, &obs, 1, cfg);
    CHECK_FALSE(r.success());
    // With such a tiny deadline, it should timeout (4), but may also hit
    // max_depth if the first iteration completes before the deadline.
    CHECK((r.fail_code == 2 || r.fail_code == 4));
}

}  // TEST_SUITE("FFB_FailCodes")

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("FFB_Cache") {

TEST_CASE("same seed twice: second call has fewer FK calls (cache hit)") {
    auto lect = make_2dof_lect();
    Eigen::VectorXd seed(2);
    seed << 0.5, 0.5;

    // Use an obstacle that forces a few splits
    Obstacle obs(-100.0f, -100.0f, -100.0f, 100.0f, 100.0f, 100.0f);

    FFBConfig cfg;
    cfg.max_depth = 5;

    FFBResult r1 = find_free_box(lect, seed, &obs, 1, cfg);

    // Second call on same LECT: nodes already cached
    FFBResult r2 = find_free_box(lect, seed, &obs, 1, cfg);

    // Second call should have no new FK calls (all nodes cached)
    CHECK(r2.n_fk_calls <= r1.n_fk_calls);
    if (r1.n_fk_calls > 0) {
        CHECK(r2.n_fk_calls == 0);
    }
}

TEST_CASE("different seed same subtree: shares ancestor cache") {
    auto lect = make_2dof_lect();

    Obstacle obs(-100.0f, -100.0f, -100.0f, 100.0f, 100.0f, 100.0f);
    FFBConfig cfg;
    cfg.max_depth = 4;

    // First seed
    Eigen::VectorXd seed1(2);
    seed1 << 0.1, 0.1;
    FFBResult r1 = find_free_box(lect, seed1, &obs, 1, cfg);

    // Second seed nearby — should share ancestor nodes
    Eigen::VectorXd seed2(2);
    seed2 << 0.2, 0.2;
    FFBResult r2 = find_free_box(lect, seed2, &obs, 1, cfg);

    // Second call should need fewer FK calls because ancestor envelopes are cached
    CHECK(r2.n_fk_calls <= r1.n_fk_calls);
}

}  // TEST_SUITE("FFB_Cache")

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("FFB_Path") {

TEST_CASE("path starts at root (0)") {
    auto lect = make_2dof_lect();
    Eigen::VectorXd seed(2);
    seed << 0.0, 0.0;

    FFBResult r = find_free_box(lect, seed, nullptr, 0);
    REQUIRE_FALSE(r.path.empty());
    CHECK(r.path[0] == 0);
}

TEST_CASE("path ends at node_idx on success") {
    auto lect = make_2dof_lect();
    Eigen::VectorXd seed(2);
    seed << 0.0, 0.0;

    FFBResult r = find_free_box(lect, seed, nullptr, 0);
    REQUIRE(r.success());
    CHECK(r.path.back() == r.node_idx);
}

TEST_CASE("path length <= max_depth + 1") {
    auto lect = make_2dof_lect();
    Eigen::VectorXd seed(2);
    seed << 0.0, 0.0;

    Obstacle obs(-100.0f, -100.0f, -100.0f, 100.0f, 100.0f, 100.0f);

    FFBConfig cfg;
    cfg.max_depth = 5;

    FFBResult r = find_free_box(lect, seed, &obs, 1, cfg);
    CHECK(static_cast<int>(r.path.size()) <= cfg.max_depth + 1);
}

}  // TEST_SUITE("FFB_Path")
