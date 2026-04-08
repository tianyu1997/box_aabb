// SafeBoxForest v5 — test_lect_io (Phase E, Step E3 verification)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <sbf/lect/lect_io.h>
#include <sbf/lect/lect.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>

#include <cstdio>
#include <string>
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

static const char* TEST_FILE = "test_lect_io_tmp.bin";

struct CleanupGuard {
    ~CleanupGuard() { std::remove(TEST_FILE); }
};

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("LECT_IO") {

TEST_CASE("save and load: root-only tree roundtrips") {
    CleanupGuard guard;
    auto lect = make_2dof_lect();
    REQUIRE(lect.n_nodes() == 1);
    REQUIRE(lect.has_data(0));

    CHECK(lect_save_binary(lect, TEST_FILE));

    LECT loaded;
    Robot robot = make_2dof();
    CHECK(lect_load_binary(loaded, robot, TEST_FILE));

    CHECK(loaded.n_nodes() == 1);
    CHECK(loaded.n_dims() == lect.n_dims());
    CHECK(loaded.n_active_links() == lect.n_active_links());
    CHECK(loaded.has_data(0));
    CHECK(loaded.is_leaf(0));
    CHECK(loaded.depth(0) == 0);
}

TEST_CASE("save and load: expanded tree preserves structure") {
    CleanupGuard guard;
    auto lect = make_2dof_lect();
    lect.expand_leaf(0);
    int li = lect.left(0);
    lect.expand_leaf(li);
    REQUIRE(lect.n_nodes() == 5);

    CHECK(lect_save_binary(lect, TEST_FILE));

    LECT loaded;
    Robot robot = make_2dof();
    CHECK(lect_load_binary(loaded, robot, TEST_FILE));

    CHECK(loaded.n_nodes() == 5);
    CHECK_FALSE(loaded.is_leaf(0));
    CHECK(loaded.left(0) == lect.left(0));
    CHECK(loaded.right(0) == lect.right(0));
    CHECK(loaded.get_split_dim(0) == lect.get_split_dim(0));
    CHECK(loaded.split_val(0) == doctest::Approx(lect.split_val(0)));

    int loaded_li = loaded.left(0);
    CHECK_FALSE(loaded.is_leaf(loaded_li));
    CHECK(loaded.depth(loaded_li) == 1);
    CHECK(loaded.depth(loaded.left(loaded_li)) == 2);
}

TEST_CASE("save and load: envelope data matches") {
    CleanupGuard guard;
    auto lect = make_2dof_lect();
    lect.expand_leaf(0);
    CHECK(lect_save_binary(lect, TEST_FILE));

    LECT loaded;
    Robot robot = make_2dof();
    CHECK(lect_load_binary(loaded, robot, TEST_FILE));

    // Compare link iAABBs for all nodes
    for (int i = 0; i < lect.n_nodes(); ++i) {
        CHECK(loaded.has_data(i) == lect.has_data(i));
        if (!lect.has_data(i)) continue;

        const float* orig = lect.get_link_iaabbs(i);
        const float* load = loaded.get_link_iaabbs(i);
        for (int j = 0; j < lect.n_active_links() * 6; ++j) {
            CHECK(load[j] == doctest::Approx(orig[j]));
        }
    }
}

TEST_CASE("save and load: endpoint iAABBs match") {
    CleanupGuard guard;
    auto lect = make_2dof_lect();
    CHECK(lect_save_binary(lect, TEST_FILE));

    LECT loaded;
    Robot robot = make_2dof();
    CHECK(lect_load_binary(loaded, robot, TEST_FILE));

    const float* orig = lect.get_endpoint_iaabbs(0);
    const float* load = loaded.get_endpoint_iaabbs(0);
    int ep_size = lect.n_active_links() * 2 * 6;
    for (int j = 0; j < ep_size; ++j) {
        CHECK(load[j] == doctest::Approx(orig[j]));
    }
}

TEST_CASE("save and load: occupation state preserved") {
    CleanupGuard guard;
    auto lect = make_2dof_lect();
    lect.expand_leaf(0);
    lect.mark_occupied(lect.left(0), 42);

    CHECK(lect_save_binary(lect, TEST_FILE));

    LECT loaded;
    Robot robot = make_2dof();
    CHECK(lect_load_binary(loaded, robot, TEST_FILE));

    CHECK_FALSE(loaded.is_occupied(0));
    CHECK(loaded.is_occupied(loaded.left(0)));
    CHECK(loaded.forest_id(loaded.left(0)) == 42);
    CHECK_FALSE(loaded.is_occupied(loaded.right(0)));
}

TEST_CASE("save and load: root intervals preserved") {
    CleanupGuard guard;
    auto lect = make_2dof_lect();
    auto orig_ivs = lect.node_intervals(0);

    CHECK(lect_save_binary(lect, TEST_FILE));

    LECT loaded;
    Robot robot = make_2dof();
    CHECK(lect_load_binary(loaded, robot, TEST_FILE));

    auto loaded_ivs = loaded.node_intervals(0);
    REQUIRE(loaded_ivs.size() == orig_ivs.size());
    for (size_t d = 0; d < orig_ivs.size(); ++d) {
        CHECK(loaded_ivs[d].lo == doctest::Approx(orig_ivs[d].lo));
        CHECK(loaded_ivs[d].hi == doctest::Approx(orig_ivs[d].hi));
    }
}

TEST_CASE("save and load: split_order preserved") {
    CleanupGuard guard;
    auto lect = make_2dof_lect();
    lect.set_split_order(SplitOrder::ROUND_ROBIN);

    CHECK(lect_save_binary(lect, TEST_FILE));

    LECT loaded;
    Robot robot = make_2dof();
    CHECK(lect_load_binary(loaded, robot, TEST_FILE));

    CHECK(loaded.split_order() == SplitOrder::ROUND_ROBIN);
}

TEST_CASE("load: bad magic returns false") {
    CleanupGuard guard;
    // Write garbage
    {
        std::ofstream f(TEST_FILE, std::ios::binary);
        f.write("BADMAGIC", 8);
    }

    LECT loaded;
    Robot robot = make_2dof();
    CHECK_FALSE(lect_load_binary(loaded, robot, TEST_FILE));
}

TEST_CASE("load: nonexistent file returns false") {
    LECT loaded;
    Robot robot = make_2dof();
    CHECK_FALSE(lect_load_binary(loaded, robot, "nonexistent_file.bin"));
}

TEST_CASE("save: invalid path returns false") {
    auto lect = make_2dof_lect();
    CHECK_FALSE(lect_save_binary(lect, ""));
}

TEST_CASE("loaded LECT: collides_scene works") {
    CleanupGuard guard;
    auto lect = make_2dof_lect();
    lect.expand_leaf(0);
    CHECK(lect_save_binary(lect, TEST_FILE));

    LECT loaded;
    Robot robot = make_2dof();
    CHECK(lect_load_binary(loaded, robot, TEST_FILE));

    // Known collision obstacle overlapping link envelope (from test_lect)
    Obstacle obs(1.0f, -0.5f, -0.5f, 2.5f, 0.5f, 0.5f);
    CHECK(loaded.collides_scene(0, &obs, 1));

    // Far away obstacle
    Obstacle obs_far(100.f, 100.f, 100.f, 101.f, 101.f, 101.f);
    CHECK_FALSE(loaded.collides_scene(0, &obs_far, 1));
}

TEST_CASE("loaded LECT: can expand further") {
    CleanupGuard guard;
    auto lect = make_2dof_lect();
    CHECK(lect_save_binary(lect, TEST_FILE));

    LECT loaded;
    Robot robot = make_2dof();
    CHECK(lect_load_binary(loaded, robot, TEST_FILE));

    // Expand root in loaded tree
    int added = loaded.expand_leaf(0);
    CHECK(added == 2);
    CHECK(loaded.n_nodes() == 3);
    CHECK(loaded.has_data(loaded.left(0)));
    CHECK(loaded.has_data(loaded.right(0)));
}

}  // TEST_SUITE("LECT_IO")
