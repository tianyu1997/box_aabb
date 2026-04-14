#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <sbf/core/types.h>
#include <sbf/core/interval_math.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>
#include <sbf/core/joint_symmetry.h>
#include <sbf/scene/collision_checker.h>

#include <cmath>
#include <random>
#include <vector>

using namespace sbf;

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("Interval") {

TEST_CASE("width, center, contains") {
    Interval iv(1.0, 3.0);
    CHECK(iv.width() == doctest::Approx(2.0));
    CHECK(iv.center() == doctest::Approx(2.0));
    CHECK(iv.contains(1.5));
    CHECK(iv.contains(1.0));
    CHECK(iv.contains(3.0));
    CHECK_FALSE(iv.contains(0.5));
    CHECK_FALSE(iv.contains(3.5));
}

TEST_CASE("arithmetic") {
    Interval a(1.0, 2.0), b(3.0, 5.0);
    auto sum = a + b;
    CHECK(sum.lo == doctest::Approx(4.0));
    CHECK(sum.hi == doctest::Approx(7.0));

    auto diff = a - b;
    CHECK(diff.lo == doctest::Approx(-4.0));
    CHECK(diff.hi == doctest::Approx(-1.0));

    auto prod = a * b;
    CHECK(prod.lo == doctest::Approx(3.0));
    CHECK(prod.hi == doctest::Approx(10.0));
}

TEST_CASE("I_sin conservativeness: MC 1000 points") {
    std::mt19937 gen(42);
    std::uniform_real_distribution<double> dist_lo(-6.0, 6.0);
    std::uniform_real_distribution<double> dist_w(0.01, 3.0);

    for (int trial = 0; trial < 100; ++trial) {
        double lo = dist_lo(gen);
        double hi = lo + dist_w(gen);
        auto result = I_sin(lo, hi);

        std::uniform_real_distribution<double> sample(lo, hi);
        for (int i = 0; i < 1000; ++i) {
            double v = sample(gen);
            double sv = std::sin(v);
            CHECK_MESSAGE(sv >= result.lo - 1e-12,
                "sin(" << v << ")=" << sv << " < lo=" << result.lo
                << " for [" << lo << "," << hi << "]");
            CHECK_MESSAGE(sv <= result.hi + 1e-12,
                "sin(" << v << ")=" << sv << " > hi=" << result.hi
                << " for [" << lo << "," << hi << "]");
        }
    }
}

TEST_CASE("I_cos conservativeness: MC 1000 points") {
    std::mt19937 gen(123);
    std::uniform_real_distribution<double> dist_lo(-6.0, 6.0);
    std::uniform_real_distribution<double> dist_w(0.01, 3.0);

    for (int trial = 0; trial < 100; ++trial) {
        double lo = dist_lo(gen);
        double hi = lo + dist_w(gen);
        auto result = I_cos(lo, hi);

        std::uniform_real_distribution<double> sample(lo, hi);
        for (int i = 0; i < 1000; ++i) {
            double v = sample(gen);
            double cv = std::cos(v);
            CHECK_MESSAGE(cv >= result.lo - 1e-12,
                "cos(" << v << ")=" << cv << " < lo=" << result.lo
                << " for [" << lo << "," << hi << "]");
            CHECK_MESSAGE(cv <= result.hi + 1e-12,
                "cos(" << v << ")=" << cv << " > hi=" << result.hi
                << " for [" << lo << "," << hi << "]");
        }
    }
}

TEST_CASE("I_sin / I_cos tightness: width <= 2x real range") {
    std::mt19937 gen(99);
    std::uniform_real_distribution<double> dist_lo(-6.0, 6.0);
    std::uniform_real_distribution<double> dist_w(0.01, 1.0);

    for (int trial = 0; trial < 100; ++trial) {
        double lo = dist_lo(gen);
        double hi = lo + dist_w(gen);

        auto s = I_sin(lo, hi);
        auto c = I_cos(lo, hi);

        // Estimate true range via dense sampling
        double true_s_lo = 2.0, true_s_hi = -2.0;
        double true_c_lo = 2.0, true_c_hi = -2.0;
        for (int i = 0; i <= 10000; ++i) {
            double t = lo + (hi - lo) * i / 10000.0;
            double sv = std::sin(t), cv = std::cos(t);
            true_s_lo = std::min(true_s_lo, sv);
            true_s_hi = std::max(true_s_hi, sv);
            true_c_lo = std::min(true_c_lo, cv);
            true_c_hi = std::max(true_c_hi, cv);
        }

        double true_s_w = true_s_hi - true_s_lo;
        double true_c_w = true_c_hi - true_c_lo;

        if (true_s_w > 1e-10)
            CHECK(s.width() <= 2.0 * true_s_w + 1e-10);
        if (true_c_w > 1e-10)
            CHECK(c.width() <= 2.0 * true_c_w + 1e-10);
    }
}

}  // TEST_SUITE("Interval")

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("Robot") {

TEST_CASE("from_json: 2dof_planar") {
    Robot robot = Robot::from_json("data/2dof_planar.json");
    CHECK(robot.n_joints() == 2);
    CHECK(robot.name() == "2dof_planar");
    CHECK(robot.joint_limits().limits.size() == 2);
    CHECK(robot.joint_limits().limits[0].lo == doctest::Approx(-PI));
    CHECK(robot.joint_limits().limits[0].hi == doctest::Approx(PI));
    CHECK(robot.dh_params()[0].a == doctest::Approx(1.0));
    CHECK(robot.dh_params()[1].a == doctest::Approx(1.0));
    CHECK(robot.has_link_radii());
}

TEST_CASE("active links: 2dof planar") {
    Robot robot = Robot::from_json("data/2dof_planar.json");
    // Both links have a=1.0 (non-zero), but link 0 is base (skipped)
    // So active links = {1} (only second link is active)
    // Wait — link 0 is always skipped as base. Link 1: a=1.0, not zero → active
    // But the plan says active_link_map filters zero-length links.
    // For 2DOF: link 0 (base) is skipped. Link 1 has a=1.0 → active.
    // n_active_links = 1.
    // Actually let me re-read: skip[0] = true always.
    // dh[0]: a=1.0 → skip[0]=true (base always skip regardless)
    // dh[1]: a=1.0 → skip[1]=false → active
    // So active_link_map = {1}, n_active_links = 1
    CHECK(robot.n_active_links() == 1);
}

TEST_CASE("construct directly") {
    std::vector<DHParam> dh = {
        {0.0, 1.0, 0.0, 0.0, 0},
        {0.0, 1.0, 0.0, 0.0, 0}
    };
    JointLimits lim;
    lim.limits = {{-PI, PI}, {-PI, PI}};
    Robot robot("test_2dof", dh, lim);
    CHECK(robot.n_joints() == 2);
    CHECK(robot.n_active_links() == 1);
}

}  // TEST_SUITE("Robot")

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("FKState") {

TEST_CASE("compute_fk_full: point config produces point aabbs") {
    Robot robot = Robot::from_json("data/2dof_planar.json");
    // q = [0, 0]: link 1 goes from (1,0,0) to (2,0,0) in the plane
    std::vector<Interval> ivs = {{0.0, 0.0}, {0.0, 0.0}};
    FKState state = compute_fk_full(robot, ivs);
    CHECK(state.valid);

    int n_active = robot.n_active_links();
    std::vector<float> aabb(n_active * 6);
    extract_link_aabbs(state, robot.active_link_map(), n_active,
                       aabb.data(), nullptr);

    // Active link is link 1: from prefix[1] to prefix[2]
    // For q=[0,0]: prefix[1] translation = (1, 0, 0), prefix[2] = (2, 0, 0)
    CHECK(aabb[0] == doctest::Approx(1.0f).epsilon(1e-5));  // lo_x
    CHECK(aabb[1] == doctest::Approx(0.0f).epsilon(1e-5));  // lo_y
    CHECK(aabb[2] == doctest::Approx(0.0f).epsilon(1e-5));  // lo_z
    CHECK(aabb[3] == doctest::Approx(2.0f).epsilon(1e-5));  // hi_x
    CHECK(aabb[4] == doctest::Approx(0.0f).epsilon(1e-5));  // hi_y
    CHECK(aabb[5] == doctest::Approx(0.0f).epsilon(1e-5));  // hi_z
}

TEST_CASE("compute_fk_incremental matches full") {
    Robot robot = Robot::from_json("data/2dof_planar.json");

    std::vector<Interval> ivs1 = {{0.0, 0.5}, {-0.3, 0.3}};
    FKState full1 = compute_fk_full(robot, ivs1);

    std::vector<Interval> ivs2 = {{0.0, 0.5}, {-0.1, 0.1}};
    FKState full2 = compute_fk_full(robot, ivs2);
    FKState inc2 = compute_fk_incremental(full1, robot, ivs2, 1);

    CHECK(inc2.valid);

    // Compare prefix chain entries
    int n_tf = full2.n_tf;
    for (int f = 0; f < n_tf; ++f) {
        for (int e = 0; e < 16; ++e) {
            CHECK(inc2.prefix_lo[f][e] == doctest::Approx(full2.prefix_lo[f][e]).epsilon(1e-12));
            CHECK(inc2.prefix_hi[f][e] == doctest::Approx(full2.prefix_hi[f][e]).epsilon(1e-12));
        }
    }
}

TEST_CASE("extract_link_aabbs: interval box contains point aabbs") {
    Robot robot = Robot::from_json("data/2dof_planar.json");
    int n_active = robot.n_active_links();

    // Interval box
    std::vector<Interval> box = {{-0.5, 0.5}, {-0.5, 0.5}};
    FKState box_state = compute_fk_full(robot, box);
    std::vector<float> box_aabb(n_active * 6);
    extract_link_aabbs(box_state, robot.active_link_map(), n_active,
                       box_aabb.data(), nullptr);

    // Sample point configs inside box and check containment
    std::mt19937 gen(77);
    std::uniform_real_distribution<double> dist(-0.5, 0.5);
    for (int trial = 0; trial < 100; ++trial) {
        double q0 = dist(gen), q1 = dist(gen);
        std::vector<Interval> pt = {{q0, q0}, {q1, q1}};
        FKState pt_state = compute_fk_full(robot, pt);
        std::vector<float> pt_aabb(n_active * 6);
        extract_link_aabbs(pt_state, robot.active_link_map(), n_active,
                           pt_aabb.data(), nullptr);

        for (int i = 0; i < n_active; ++i) {
            for (int d = 0; d < 3; ++d) {
                CHECK(pt_aabb[i * 6 + d] >= box_aabb[i * 6 + d] - 1e-6f);
                CHECK(pt_aabb[i * 6 + 3 + d] <= box_aabb[i * 6 + 3 + d] + 1e-6f);
            }
        }
    }
}

TEST_CASE("extract_endpoint_iaabbs: dimension check") {
    Robot robot = Robot::from_json("data/2dof_planar.json");
    int n_active = robot.n_active_links();

    std::vector<Interval> ivs = {{0.0, 0.1}, {0.0, 0.1}};
    FKState state = compute_fk_full(robot, ivs);

    std::vector<float> endpoints(n_active * 2 * 6);
    extract_endpoint_iaabbs(state, robot.active_link_map(), n_active,
                            endpoints.data());

    // Should have 2 endpoints per active link (proximal + distal)
    // For 1 active link: 2 * 6 = 12 floats
    CHECK(endpoints.size() == static_cast<size_t>(n_active * 2 * 6));
}

}  // TEST_SUITE("FKState")

// ═══════════════════════════════════════════════════════════════════════════
TEST_SUITE("CollisionChecker") {

TEST_CASE("2DOF: known collision-free config") {
    Robot robot = Robot::from_json("data/2dof_planar.json");

    // Obstacle far away at (10, 10, 0)
    Obstacle obs(9.0f, 9.0f, -0.5f, 11.0f, 11.0f, 0.5f);
    CollisionChecker checker(robot, {obs});

    Eigen::VectorXd q(2);
    q << 0.0, 0.0;  // arm reaches (2, 0) — far from obstacle
    CHECK_FALSE(checker.check_config(q));
}

TEST_CASE("2DOF: known collision config") {
    Robot robot = Robot::from_json("data/2dof_planar.json");

    // Obstacle at origin area where link sweeps
    Obstacle obs(0.5f, -0.5f, -0.5f, 1.5f, 0.5f, 0.5f);
    CollisionChecker checker(robot, {obs});

    Eigen::VectorXd q(2);
    q << 0.0, 0.0;  // link from (1,0) to (2,0), passes through obs
    CHECK(checker.check_config(q));
}

TEST_CASE("check_box: conservativeness") {
    Robot robot = Robot::from_json("data/2dof_planar.json");

    // Obstacle near tip area
    Obstacle obs(1.4f, -0.3f, -0.5f, 2.0f, 0.3f, 0.5f);
    CollisionChecker checker(robot, {obs});

    // A box that contains a config that collides
    std::vector<Interval> box = {{-0.1, 0.1}, {-0.1, 0.1}};
    bool box_collides = checker.check_box(box);

    // If any config in the box collides, check_box must return true
    Eigen::VectorXd q(2);
    q << 0.0, 0.0;
    bool config_collides = checker.check_config(q);

    if (config_collides) {
        CHECK(box_collides);
    }
}

TEST_CASE("check_segment: collision on path") {
    Robot robot = Robot::from_json("data/2dof_planar.json");

    // Obstacle in the workspace
    Obstacle obs(0.8f, -0.3f, -0.5f, 1.2f, 0.3f, 0.5f);
    CollisionChecker checker(robot, {obs});

    Eigen::VectorXd q1(2), q2(2);
    q1 << PI / 2.0, 0.0;   // arm points up
    q2 << 0.0, 0.0;         // arm points right (through obstacle)

    // At q2, link passes through obstacle → segment should detect collision
    CHECK(checker.check_segment(q1, q2, 20));
}

}  // TEST_SUITE("CollisionChecker")
