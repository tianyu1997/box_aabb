// SafeBoxForest v5 — test_lect (Phase D verification)

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <sbf/lect/lect.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>
#include <sbf/core/joint_symmetry.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>

#include <cmath>
#include <random>
#include <vector>

using namespace sbf;

// ─── Test helpers ───────────────────────────────────────────────────────────

static Robot make_2dof() {
    return Robot::from_json("data/2dof_planar.json");
}

static LECT make_2dof_lect(SplitOrder so = SplitOrder::BEST_TIGHTEN) {
    Robot robot = make_2dof();
    std::vector<Interval> root = robot.joint_limits().limits;
    EndpointSourceConfig ep_cfg;
    ep_cfg.source = EndpointSource::IFK;
    EnvelopeTypeConfig env_cfg;
    env_cfg.type = EnvelopeType::LinkIAABB;
    LECT lect(robot, root, ep_cfg, env_cfg);
    lect.set_split_order(so);
    return lect;
}

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("LECT_Structure") {

TEST_CASE("init: n_nodes == 1, root has data") {
    auto lect = make_2dof_lect();
    CHECK(lect.n_nodes() == 1);
    CHECK(lect.n_dims() == 2);
    CHECK(lect.has_data(0));
    CHECK(lect.is_leaf(0));
    CHECK(lect.n_active_links() > 0);
}

TEST_CASE("expand_leaf: n_nodes == 3") {
    auto lect = make_2dof_lect();
    int added = lect.expand_leaf(0);
    CHECK(added == 2);
    CHECK(lect.n_nodes() == 3);
    CHECK_FALSE(lect.is_leaf(0));
    CHECK(lect.is_leaf(lect.left(0)));
    CHECK(lect.is_leaf(lect.right(0)));
    CHECK(lect.has_data(lect.left(0)));
    CHECK(lect.has_data(lect.right(0)));
}

TEST_CASE("expand non-leaf returns 0") {
    auto lect = make_2dof_lect();
    lect.expand_leaf(0);
    CHECK(lect.expand_leaf(0) == 0);
}

TEST_CASE("tree structure: parent/left/right consistency") {
    auto lect = make_2dof_lect();
    lect.expand_leaf(0);
    int li = lect.left(0);
    int ri = lect.right(0);
    CHECK(lect.parent(li) == 0);
    CHECK(lect.parent(ri) == 0);
    CHECK(lect.depth(li) == 1);
    CHECK(lect.depth(ri) == 1);
}

TEST_CASE("multi-level expand") {
    auto lect = make_2dof_lect();
    lect.expand_leaf(0);
    int li = lect.left(0);
    int added2 = lect.expand_leaf(li);
    CHECK(added2 == 2);
    CHECK(lect.n_nodes() == 5);
    CHECK(lect.depth(lect.left(li)) == 2);
}

TEST_CASE("node_intervals: root intervals match") {
    Robot robot = make_2dof();
    auto root_ivs = robot.joint_limits().limits;
    EndpointSourceConfig ep_cfg;
    ep_cfg.source = EndpointSource::IFK;
    EnvelopeTypeConfig env_cfg;
    LECT lect(robot, root_ivs, ep_cfg, env_cfg);

    auto reconstructed = lect.node_intervals(0);
    REQUIRE(reconstructed.size() == root_ivs.size());
    for (size_t d = 0; d < root_ivs.size(); ++d) {
        CHECK(reconstructed[d].lo == doctest::Approx(root_ivs[d].lo));
        CHECK(reconstructed[d].hi == doctest::Approx(root_ivs[d].hi));
    }
}

TEST_CASE("node_intervals: child intervals correct after split") {
    auto lect = make_2dof_lect();
    auto root_ivs = lect.node_intervals(0);
    lect.expand_leaf(0);

    int li = lect.left(0);
    int ri = lect.right(0);
    int dim = lect.get_split_dim(0);
    double sv = lect.split_val(0);

    auto left_ivs = lect.node_intervals(li);
    auto right_ivs = lect.node_intervals(ri);

    CHECK(left_ivs[dim].hi == doctest::Approx(sv));
    CHECK(right_ivs[dim].lo == doctest::Approx(sv));
    CHECK(left_ivs[dim].lo == doctest::Approx(root_ivs[dim].lo));
    CHECK(right_ivs[dim].hi == doctest::Approx(root_ivs[dim].hi));
}

}  // TEST_SUITE("LECT_Structure")

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("LECT_Split") {

TEST_CASE("ROUND_ROBIN: split_dim = depth % n_dims") {
    auto lect = make_2dof_lect(SplitOrder::ROUND_ROBIN);
    lect.expand_leaf(0);
    CHECK(lect.get_split_dim(0) == 0);  // depth 0 % 2 = 0

    int li = lect.left(0);
    lect.expand_leaf(li);
    CHECK(lect.get_split_dim(li) == 1);  // depth 1 % 2 = 1
}

TEST_CASE("WIDEST_FIRST: selects widest dimension") {
    Robot robot = make_2dof();
    // Asymmetric intervals: dim 0 is narrower than dim 1
    std::vector<Interval> root = {{-0.5, 0.5}, {-2.0, 2.0}};
    EndpointSourceConfig ep_cfg;
    ep_cfg.source = EndpointSource::IFK;
    EnvelopeTypeConfig env_cfg;
    LECT lect(robot, root, ep_cfg, env_cfg);
    lect.set_split_order(SplitOrder::WIDEST_FIRST);

    lect.expand_leaf(0);
    CHECK(lect.get_split_dim(0) == 1);  // dim 1 is wider
}

TEST_CASE("BEST_TIGHTEN: max child vol <= ROUND_ROBIN max child vol") {
    Robot robot = make_2dof();
    std::vector<Interval> root = {{-1.0, 1.0}, {-1.0, 1.0}};
    EndpointSourceConfig ep_cfg;
    ep_cfg.source = EndpointSource::IFK;
    EnvelopeTypeConfig env_cfg;

    // Build ROUND_ROBIN tree
    LECT lect_rr(robot, root, ep_cfg, env_cfg);
    lect_rr.set_split_order(SplitOrder::ROUND_ROBIN);
    lect_rr.expand_leaf(0);

    // Build BEST_TIGHTEN tree
    LECT lect_bt(robot, root, ep_cfg, env_cfg);
    lect_bt.set_split_order(SplitOrder::BEST_TIGHTEN);
    lect_bt.expand_leaf(0);

    // Compare max child link iAABB volume
    auto child_max_vol = [](const LECT& l, int parent) {
        int li = l.left(parent);
        int ri = l.right(parent);
        double max_vol = 0.0;
        for (int child : {li, ri}) {
            const float* la = l.get_link_iaabbs(child);
            double vol = 0.0;
            for (int ci = 0; ci < l.n_active_links(); ++ci) {
                const float* b = la + ci * 6;
                vol += static_cast<double>(b[3] - b[0]) *
                       static_cast<double>(b[4] - b[1]) *
                       static_cast<double>(b[5] - b[2]);
            }
            max_vol = std::max(max_vol, vol);
        }
        return max_vol;
    };

    double rr_vol = child_max_vol(lect_rr, 0);
    double bt_vol = child_max_vol(lect_bt, 0);
    CHECK(bt_vol <= rr_vol + 1e-6);
}

}  // TEST_SUITE("LECT_Split")

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("LECT_Envelope") {

TEST_CASE("compute_envelope: root has_data and link_iaabbs valid") {
    auto lect = make_2dof_lect();
    CHECK(lect.has_data(0));

    const float* la = lect.get_link_iaabbs(0);
    for (int ci = 0; ci < lect.n_active_links(); ++ci) {
        const float* b = la + ci * 6;
        CHECK(b[0] < b[3]);  // lo_x < hi_x
        CHECK(b[1] < b[4]);  // lo_y < hi_y
        // z might be zero-width for 2D planar
    }
}

TEST_CASE("link_iaabbs are conservative: MC verification") {
    Robot robot = make_2dof();
    std::vector<Interval> root = {{-1.0, 1.0}, {-0.5, 0.5}};
    EndpointSourceConfig ep_cfg;
    ep_cfg.source = EndpointSource::IFK;
    EnvelopeTypeConfig env_cfg;
    LECT lect(robot, root, ep_cfg, env_cfg);

    const float* la = lect.get_link_iaabbs(0);
    int n_act = lect.n_active_links();

    // MC : random configs should lie within link iAABBs
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> d0(root[0].lo, root[0].hi);
    std::uniform_real_distribution<double> d1(root[1].lo, root[1].hi);

    for (int s = 0; s < 500; ++s) {
        std::vector<Interval> pt = {{d0(gen), 0.0}, {d1(gen), 0.0}};
        pt[0].hi = pt[0].lo;
        pt[1].hi = pt[1].lo;

        FKState fk = compute_fk_full(robot, pt);
        std::vector<float> aabb(n_act * 6);
        extract_link_aabbs(fk, robot.active_link_map(), n_act,
                           aabb.data(), robot.active_link_radii());

        for (int ci = 0; ci < n_act; ++ci) {
            const float* mc = aabb.data() + ci * 6;
            const float* iv = la + ci * 6;
            for (int d = 0; d < 3; ++d) {
                CHECK_MESSAGE(mc[d] >= iv[d] - 1e-4f,
                    "MC lo_" << d << " outside link iAABB");
                CHECK_MESSAGE(mc[d + 3] <= iv[d + 3] + 1e-4f,
                    "MC hi_" << d << " outside link iAABB");
            }
        }
    }
}

TEST_CASE("incremental compute matches full") {
    auto lect = make_2dof_lect();
    lect.expand_leaf(0);
    int li = lect.left(0);

    auto ivs = lect.node_intervals(li);
    Robot robot = make_2dof();
    FKState full_fk = compute_fk_full(robot, ivs);
    std::vector<float> full_ep(robot.n_active_links() * 2 * 6);
    extract_endpoint_iaabbs(full_fk, robot.active_link_map(),
                            robot.n_active_links(), full_ep.data());

    const float* lect_ep = lect.get_endpoint_iaabbs(li);
    for (int k = 0; k < robot.n_active_links() * 2 * 6; ++k) {
        CHECK(lect_ep[k] == doctest::Approx(full_ep[k]).epsilon(1e-6));
    }
}

}  // TEST_SUITE("LECT_Envelope")

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("LECT_Z4") {

TEST_CASE("2DOF full-rotation: Z4 cache active") {
    // 2DOF planar: alpha_0 == 0 → Z4 symmetry detected
    auto lect = make_2dof_lect();
    CHECK(lect.symmetry_q0().type == JointSymmetryType::Z4_ROTATION);
}

TEST_CASE("Z4 cache hit yields consistent results") {
    Robot robot = make_2dof();
    auto root = robot.joint_limits().limits;
    EndpointSourceConfig ep_cfg;
    ep_cfg.source = EndpointSource::IFK;
    EnvelopeTypeConfig env_cfg;
    LECT lect(robot, root, ep_cfg, env_cfg);

    // Expand several levels to exercise the Z4 cache
    lect.expand_leaf(0);
    int li = lect.left(0);
    int ri = lect.right(0);
    lect.expand_leaf(li);
    lect.expand_leaf(ri);

    // For each leaf, verify link iAABBs are conservative via MC
    std::mt19937 gen(123);
    int n_act = robot.n_active_links();
    for (int ni = 0; ni < lect.n_nodes(); ++ni) {
        if (!lect.is_leaf(ni) || !lect.has_data(ni)) continue;
        auto ivs = lect.node_intervals(ni);
        const float* la = lect.get_link_iaabbs(ni);

        for (int s = 0; s < 100; ++s) {
            std::vector<Interval> pt(2);
            for (int d = 0; d < 2; ++d) {
                double lo = ivs[d].lo, hi = ivs[d].hi;
                std::uniform_real_distribution<double> dist(lo, hi);
                double v = dist(gen);
                pt[d] = {v, v};
            }
            FKState fk = compute_fk_full(robot, pt);
            std::vector<float> aabb(n_act * 6);
            extract_link_aabbs(fk, robot.active_link_map(), n_act,
                               aabb.data(), robot.active_link_radii());
            for (int ci = 0; ci < n_act; ++ci) {
                for (int d = 0; d < 3; ++d) {
                    CHECK(aabb[ci * 6 + d] >= la[ci * 6 + d] - 1e-3f);
                    CHECK(aabb[ci * 6 + d + 3] <= la[ci * 6 + d + 3] + 1e-3f);
                }
            }
        }
    }
}

}  // TEST_SUITE("LECT_Z4")

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("LECT_Collision") {

TEST_CASE("known collision: obstacle overlapping link envelope") {
    Robot robot = make_2dof();
    std::vector<Interval> root = {{-0.2, 0.2}, {-0.2, 0.2}};
    EndpointSourceConfig ep_cfg;
    ep_cfg.source = EndpointSource::IFK;
    EnvelopeTypeConfig env_cfg;
    LECT lect(robot, root, ep_cfg, env_cfg);

    // Place obstacle at (1.5, 0, 0) — overlaps link between frame 1 and 2
    Obstacle obs(1.0f, -0.5f, -0.5f, 2.5f, 0.5f, 0.5f);
    CHECK(lect.collides_scene(0, &obs, 1));
}

TEST_CASE("known safe: obstacle far from workspace") {
    Robot robot = make_2dof();
    std::vector<Interval> root = {{-0.1, 0.1}, {-0.1, 0.1}};
    EndpointSourceConfig ep_cfg;
    ep_cfg.source = EndpointSource::IFK;
    EnvelopeTypeConfig env_cfg;
    LECT lect(robot, root, ep_cfg, env_cfg);

    // Place obstacle far away
    Obstacle obs(100.f, 100.f, 100.f, 101.f, 101.f, 101.f);
    CHECK_FALSE(lect.collides_scene(0, &obs, 1));
}

TEST_CASE("intervals_collide_scene consistency") {
    Robot robot = make_2dof();
    std::vector<Interval> ivs = {{0.0, 0.5}, {0.0, 0.5}};
    EndpointSourceConfig ep_cfg;
    ep_cfg.source = EndpointSource::IFK;
    EnvelopeTypeConfig env_cfg;
    LECT lect(robot, ivs, ep_cfg, env_cfg);

    Obstacle obs_far(100.f, 100.f, 100.f, 101.f, 101.f, 101.f);
    CHECK_FALSE(lect.intervals_collide_scene(ivs, &obs_far, 1));
}

}  // TEST_SUITE("LECT_Collision")

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("LECT_Occupation") {

TEST_CASE("mark/unmark occupied") {
    auto lect = make_2dof_lect();
    CHECK_FALSE(lect.is_occupied(0));
    CHECK(lect.forest_id(0) == -1);

    lect.mark_occupied(0, 42);
    CHECK(lect.is_occupied(0));
    CHECK(lect.forest_id(0) == 42);

    lect.unmark_occupied(0);
    CHECK_FALSE(lect.is_occupied(0));
    CHECK(lect.forest_id(0) == -1);
}

}
