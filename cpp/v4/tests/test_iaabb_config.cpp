// ?
//  SafeBoxForest v4 ?IAABBConfig Tests
//
//  Tests:
//    1. Panda (no tool) ?threshold 0.1
//    2. IIWA14 (with tool) ?threshold 0.1
//    3. Threshold comparison ?0.1 vs 1e-10
//    4. JSON round-trip ?save then load
//    5. Per-link affecting joints correctness
//    6. summary() output
//    7. Custom 3-DOF robot ?generality verification
//
//  Build: cmake --build . --config Release --target test_iaabb_config
//  Run:   Release\test_iaabb_config.exe
// ?
#include "sbf/robot/robot.h"
#include "sbf/robot/iaabb_config.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>

using namespace sbf;

//  Test macros 
static int g_pass = 0, g_fail = 0;

#define CHECK(cond, msg) do {                                         \
    if (!(cond)) {                                                    \
        std::printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, msg); \
        ++g_fail;                                                     \
    } else { ++g_pass; }                                              \
} while(0)

#define CHECK_CLOSE(a, b, tol, msg) \
    CHECK(std::abs(static_cast<double>(a) - static_cast<double>(b)) < (tol), msg)

#define SECTION(name) std::printf("\n %s \n", (name))

// Use a short alias for pi/2 to avoid name clashes with sbf::HALF_PI
static constexpr double kHPI = 1.5707963267948966;

// ?
//  Robot builders
// ?

static Robot make_panda() {
    // Panda 7-DOF, no tool frame
    // DH: , a, d, , type
    //  Link 0: a=0,      d=0.333  ?len=0.333  [SKIP: base]
    //  Link 1: a=0,      d=0      ?len=0      [SKIP: zero]
    //  Link 2: a=0,      d=0.316  ?len=0.316  [ACTIVE]
    //  Link 3: a=0.0825, d=0      ?len=0.0825 [SKIP: <0.1]
    //  Link 4: a=-0.0825,d=0.384  ?len?.393  [ACTIVE]
    //  Link 5: a=0,      d=0      ?len=0      [SKIP: zero]
    //  Link 6: a=0.088,  d=0      ?len=0.088  [SKIP: <0.1]
    std::vector<DHParam> dh = {
        {  0.0,     0.0,     0.333,  0.0, 0},
        { -kHPI, 0.0,     0.0,    0.0, 0},
        {  kHPI, 0.0,     0.316,  0.0, 0},
        {  kHPI, 0.0825,  0.0,    0.0, 0},
        { -kHPI,-0.0825,  0.384,  0.0, 0},
        {  kHPI, 0.0,     0.0,    0.0, 0},
        {  kHPI, 0.088,   0.0,    0.0, 0},
    };
    JointLimits limits;
    limits.limits = {
        {-2.8973, 2.8973}, {-1.7628, 1.7628}, {-2.8973, 2.8973},
        {-3.0718,-0.0698}, {-2.8973, 2.8973}, {-0.0175, 3.7525},
        {-2.8973, 2.8973},
    };
    std::vector<double> radii = {0.08, 0.08, 0.06, 0.06, 0.04, 0.04, 0.04};
    return Robot("panda_test", dh, limits, std::nullopt, radii);
}

static Robot make_iiwa14() {
    // IIWA14 7-DOF with tool frame (DH from URDF, R820)
    // DH: α, a, d, θ, type
    //  Link 0: a=0, d=0.36   ‖len=0.36  [SKIP: base]
    //  Link 1: a=0, d=0      ‖len=0     [SKIP: zero]
    //  Link 2: a=0, d=0.42   ‖len=0.42  [ACTIVE]
    //  Link 3: a=0, d=0      ‖len=0     [SKIP: zero]
    //  Link 4: a=0, d=0.4    ‖len=0.4   [ACTIVE]
    //  Link 5: a=0, d=0      ‖len=0     [SKIP: zero]
    //  Link 6: a=0, d=0      ‖len=0     [SKIP: zero]
    //  Tool 7: a=0, d=0.126  ‖len=0.126 [ACTIVE]
    std::vector<DHParam> dh = {
        {  0.0,    0.0, 0.36,   0.0, 0},
        { -kHPI,0.0, 0.0,    0.0, 0},
        {  kHPI,0.0, 0.42,   0.0, 0},
        {  kHPI,0.0, 0.0,    0.0, 0},
        { -kHPI,0.0, 0.4,    0.0, 0},
        { -kHPI,0.0, 0.0,    0.0, 0},
        {  kHPI,0.0, 0.0,    0.0, 0},
    };
    JointLimits limits;
    limits.limits = {
        {-2.9668, 2.9668}, {-2.0942, 2.0942}, {-2.9668, 2.9668},
        {-2.0942, 2.0942}, {-2.9668, 2.9668}, {-2.0942, 2.0942},
        {-3.0541, 3.0541},
    };
    DHParam tool{0.0, 0.0, 0.126, 0.0, 0};
    std::vector<double> radii = {0.08, 0.08, 0.06, 0.06, 0.04, 0.04, 0.04, 0.03};
    return Robot("iiwa14_test", dh, limits, tool, radii);
}

static Robot make_custom_3dof() {
    // Custom 3-DOF robot for generality test
    //  Link 0: a=0.5, d=0    ?len=0.5  [SKIP: base]
    //  Link 1: a=0,   d=0.3  ?len=0.3  [ACTIVE]
    //  Link 2: a=0.05,d=0.02 ?len?.054 [SKIP: <0.1]
    std::vector<DHParam> dh = {
        {  0.0,     0.5, 0.0,  0.0, 0},
        { -kHPI, 0.0, 0.3,  0.0, 0},
        {  kHPI, 0.05,0.02, 0.0, 0},
    };
    JointLimits limits;
    limits.limits = {{-3.14, 3.14}, {-3.14, 3.14}, {-3.14, 3.14}};
    std::vector<double> radii = {0.1, 0.05, 0.03};
    return Robot("custom_3dof", dh, limits, std::nullopt, radii);
}

// ?
//  Test 1: Panda IAABBConfig with threshold=0.1
// ?
static void test_panda_config() {
    SECTION("Panda IAABBConfig (threshold=0.1)");

    Robot robot = make_panda();
    auto cfg = IAABBConfig::from_robot(robot, 0.1);

    CHECK(cfg.n_joints == 7, "panda: 7 joints");
    CHECK(!cfg.has_tool, "panda: no tool");
    CHECK(cfg.n_total_links == 7, "panda: 7 total links");
    CHECK(cfg.zero_length_threshold == 0.1, "panda: threshold=0.1");

    // Active links: [2, 4]
    CHECK(cfg.n_active == 2, "panda: 2 active links");
    CHECK(cfg.n_active_endpoints == 4, "panda: 4 active endpoints (paired)");
    CHECK(cfg.active_link_map.size() == 2, "panda: map size=2");
    CHECK(cfg.active_link_map[0] == 2, "panda: active[0]=2");
    CHECK(cfg.active_link_map[1] == 4, "panda: active[1]=4");

    // Last active frame: prefix[4+1] = prefix[5]
    CHECK(cfg.last_active_frame == 5, "panda: last_active_frame=5");

    // Link lengths for all links
    CHECK_CLOSE(cfg.all_link_lengths[0], 0.333, 1e-6, "link 0 len=0.333");
    CHECK_CLOSE(cfg.all_link_lengths[1], 0.0,   1e-6, "link 1 len=0");
    CHECK_CLOSE(cfg.all_link_lengths[2], 0.316, 1e-6, "link 2 len=0.316");
    CHECK_CLOSE(cfg.all_link_lengths[3], 0.0825,1e-6, "link 3 len=0.0825");
    double link4_len = std::sqrt(0.0825*0.0825 + 0.384*0.384);
    CHECK_CLOSE(cfg.all_link_lengths[4], link4_len, 1e-6, "link 4 len~0.393");
    CHECK_CLOSE(cfg.all_link_lengths[5], 0.0,   1e-6, "link 5 len=0");
    CHECK_CLOSE(cfg.all_link_lengths[6], 0.088, 1e-6, "link 6 len=0.088");

    // Active link lengths
    CHECK_CLOSE(cfg.active_link_lengths[0], 0.316, 1e-6, "active[0] len=0.316");
    CHECK_CLOSE(cfg.active_link_lengths[1], link4_len, 1e-6, "active[1] len~0.393");

    // Active link radii (from original indices 2, 4)
    CHECK_CLOSE(cfg.active_link_radii[0], 0.06, 1e-6, "active[0] radius=0.06");
    CHECK_CLOSE(cfg.active_link_radii[1], 0.04, 1e-6, "active[1] radius=0.04");

    // Skip status
    CHECK(cfg.link_is_active[0] == false, "link 0 skipped (base)");
    CHECK(cfg.link_is_active[1] == false, "link 1 skipped (zero)");
    CHECK(cfg.link_is_active[2] == true,  "link 2 active");
    CHECK(cfg.link_is_active[3] == false, "link 3 skipped (<0.1)");
    CHECK(cfg.link_is_active[4] == true,  "link 4 active");
    CHECK(cfg.link_is_active[5] == false, "link 5 skipped (zero)");
    CHECK(cfg.link_is_active[6] == false, "link 6 skipped (<0.1)");

    // Per-link affecting joints
    CHECK(cfg.n_affecting_joints[0] == 3, "active[0] needs joints 0..2 (3 joints)");
    CHECK(cfg.n_affecting_joints[1] == 5, "active[1] needs joints 0..4 (5 joints)");

    std::printf("  OK  Panda: n_active=%d, map=[%d,%d], last_frame=%d\n",
                cfg.n_active,
                cfg.active_link_map[0], cfg.active_link_map[1],
                cfg.last_active_frame);
}

// ?
//  Test 2: IIWA14 IAABBConfig with threshold=0.1
// ?
static void test_iiwa_config() {
    SECTION("IIWA14 IAABBConfig (threshold=0.1)");

    Robot robot = make_iiwa14();
    auto cfg = IAABBConfig::from_robot(robot, 0.1);

    CHECK(cfg.n_joints == 7, "iiwa: 7 joints");
    CHECK(cfg.has_tool, "iiwa: has tool");
    CHECK(cfg.n_total_links == 8, "iiwa: 8 total links");

    // Active links: [2, 4, 7]
    CHECK(cfg.n_active == 3, "iiwa: 3 active links");
    CHECK(cfg.n_active_endpoints == 6, "iiwa: 6 active endpoints (paired)");
    CHECK(cfg.active_link_map[0] == 2, "iiwa: active[0]=2");
    CHECK(cfg.active_link_map[1] == 4, "iiwa: active[1]=4");
    CHECK(cfg.active_link_map[2] == 7, "iiwa: active[2]=7 (tool)");

    // Last active frame: prefix[7+1] = prefix[8]
    CHECK(cfg.last_active_frame == 8, "iiwa: last_active_frame=8");

    // Tool link length
    CHECK_CLOSE(cfg.all_link_lengths[7], 0.126, 1e-6, "tool len=0.126");

    // Link 6: d=0 → skipped
    CHECK(cfg.link_is_active[6] == false, "link 6 skipped (<0.1)");

    // Affecting joints
    CHECK(cfg.n_affecting_joints[0] == 3, "active[0] frame=2: 3 joints");
    CHECK(cfg.n_affecting_joints[1] == 5, "active[1] frame=4: 5 joints");
    CHECK(cfg.n_affecting_joints[2] == 7, "active[2] frame=7 (tool): 7 joints");

    // Active link radii (from original indices 2, 4, 7)
    CHECK_CLOSE(cfg.active_link_radii[0], 0.06, 1e-6, "active[0] radius=0.06");
    CHECK_CLOSE(cfg.active_link_radii[1], 0.04, 1e-6, "active[1] radius=0.04");
    CHECK_CLOSE(cfg.active_link_radii[2], 0.03, 1e-6, "active[2] radius=0.03");

    std::printf("  OK  IIWA14: n_active=%d, map=[%d,%d,%d], last_frame=%d\n",
                cfg.n_active,
                cfg.active_link_map[0], cfg.active_link_map[1],
                cfg.active_link_map[2],
                cfg.last_active_frame);
}

// ?
//  Test 3: Threshold comparison ?0.1 vs 1e-10 (backward compat)
// ?
static void test_threshold_comparison() {
    SECTION("Threshold comparison (0.1 vs 1e-10)");

    Robot robot = make_panda();

    // Strict threshold (1e-10): same as old pack_arrays behavior
    auto cfg_strict = IAABBConfig::from_robot(robot, 1e-10);
    CHECK(cfg_strict.n_active == 4, "strict: n_active=4 [2,3,4,6]");
    CHECK(cfg_strict.active_link_map[0] == 2, "strict: map[0]=2");
    CHECK(cfg_strict.active_link_map[1] == 3, "strict: map[1]=3");
    CHECK(cfg_strict.active_link_map[2] == 4, "strict: map[2]=4");
    CHECK(cfg_strict.active_link_map[3] == 6, "strict: map[3]=6");
    CHECK(cfg_strict.n_active_endpoints == 8, "strict: 8 active endpoints");

    // 0.1 threshold: aggressive, skips short links
    auto cfg_loose = IAABBConfig::from_robot(robot, 0.1);
    CHECK(cfg_loose.n_active == 2, "loose: n_active=2 [2,4]");
    CHECK(cfg_loose.active_link_map[0] == 2, "loose: map[0]=2");
    CHECK(cfg_loose.active_link_map[1] == 4, "loose: map[1]=4");
    CHECK(cfg_loose.n_active_endpoints == 4, "loose: 4 active endpoints");

    // Verify robot.iaabb_config() is built automatically with 0.1 threshold
    const auto& auto_cfg = robot.iaabb_config();
    CHECK(auto_cfg.n_active == 2, "auto config: n_active=2");
    CHECK(auto_cfg.zero_length_threshold == 0.1, "auto config: threshold=0.1");

    std::printf("  OK  strict=%d active, 0.1=%d active\n",
                cfg_strict.n_active, cfg_loose.n_active);
}

// ?
//  Test 4: JSON round-trip
// ?
static void test_json_roundtrip() {
    SECTION("JSON round-trip");

    Robot robot = make_panda();
    auto cfg = IAABBConfig::from_robot(robot, 0.1);

    const char* path = "test_iaabb_config_tmp.json";
    cfg.save_json(path);

    auto cfg2 = IAABBConfig::load_json(path);

    CHECK(cfg2.n_joints == cfg.n_joints, "rt: n_joints");
    CHECK(cfg2.has_tool == cfg.has_tool, "rt: has_tool");
    CHECK(cfg2.n_total_links == cfg.n_total_links, "rt: n_total_links");
    CHECK(cfg2.n_active == cfg.n_active, "rt: n_active");
    CHECK(cfg2.n_active_endpoints == cfg.n_active_endpoints, "rt: n_active_endpoints");
    CHECK(cfg2.last_active_frame == cfg.last_active_frame, "rt: last_active_frame");
    CHECK_CLOSE(cfg2.zero_length_threshold, cfg.zero_length_threshold, 1e-12, "rt: threshold");

    CHECK(cfg2.active_link_map == cfg.active_link_map, "rt: active_link_map");
    CHECK(cfg2.n_affecting_joints == cfg.n_affecting_joints, "rt: n_affecting_joints");

    for (int ci = 0; ci < cfg.n_active; ++ci) {
        CHECK_CLOSE(cfg2.active_link_radii[ci], cfg.active_link_radii[ci],
                     1e-12, "rt: radii[ci]");
        CHECK_CLOSE(cfg2.active_link_lengths[ci], cfg.active_link_lengths[ci],
                     1e-12, "rt: lengths[ci]");
    }

    for (int i = 0; i < cfg.n_total_links; ++i) {
        CHECK_CLOSE(cfg2.all_link_lengths[i], cfg.all_link_lengths[i],
                     1e-12, "rt: all_link_lengths[i]");
        CHECK(cfg2.link_is_active[i] == cfg.link_is_active[i],
              "rt: link_is_active[i]");
    }

    // Clean up
    std::remove(path);

    std::printf("  OK  JSON round-trip\n");
}

// ?
//  Test 5: Per-link affecting joints for IIWA14 (with tool)
// ?
static void test_affecting_joints_detail() {
    SECTION("Per-link affecting joints (detailed)");

    Robot robot = make_iiwa14();
    auto cfg = IAABBConfig::from_robot(robot, 0.1);

    // active[0] = frame 2 ?joints 0, 1, 2 affect it ?n_affecting = 3
    // active[1] = frame 4 ?joints 0..4 affect it ?n_affecting = 5
    // active[2] = frame 7 (tool) ?all 7 joints ?n_affecting = min(7+1, 7) = 7
    CHECK(cfg.n_affecting_joints[0] == 3, "frame 2: 3 joints");
    CHECK(cfg.n_affecting_joints[1] == 5, "frame 4: 5 joints");
    CHECK(cfg.n_affecting_joints[2] == 7, "frame 7 (tool): capped at n_joints=7");

    // For Panda (no tool, 7 joints)
    Robot panda = make_panda();
    auto pcfg = IAABBConfig::from_robot(panda, 0.1);
    // active[0] = frame 2 ?n_affecting = min(3, 7) = 3
    // active[1] = frame 4 ?n_affecting = min(5, 7) = 5
    CHECK(pcfg.n_affecting_joints[0] == 3, "panda active[0]: 3 joints");
    CHECK(pcfg.n_affecting_joints[1] == 5, "panda active[1]: 5 joints");

    std::printf("  OK  affecting joints correct\n");
}

// ?
//  Test 6: summary() output (smoke test)
// ?
static void test_summary() {
    SECTION("summary() output");

    Robot robot = make_panda();
    auto cfg = IAABBConfig::from_robot(robot, 0.1);
    std::string s = cfg.summary();

    CHECK(!s.empty(), "summary non-empty");
    CHECK(s.find("n_active=2") != std::string::npos, "summary contains n_active=2");
    CHECK(s.find("[SKIP: base]") != std::string::npos, "summary mentions base skip");
    CHECK(s.find("[ACTIVE]") != std::string::npos, "summary has active links");

    std::printf("  summary() output:\n%s", s.c_str());
}

// ?
//  Test 7: Custom 3-DOF robot ?generality verification
// ?
static void test_custom_3dof() {
    SECTION("Custom 3-DOF robot (generality)");

    Robot robot = make_custom_3dof();
    auto cfg = IAABBConfig::from_robot(robot, 0.1);

    CHECK(cfg.n_joints == 3, "3dof: 3 joints");
    CHECK(!cfg.has_tool, "3dof: no tool");
    CHECK(cfg.n_total_links == 3, "3dof: 3 total links");

    // Link 0: a=0.5, d=0 ?len=0.5 ?skip (base)
    // Link 1: a=0, d=0.3 ?len=0.3 ?active
    // Link 2: a=0.05, d=0.02 ?len=sqrt(0.0025+0.0004)=sqrt(0.0029)?.0539 ?skip
    CHECK(cfg.n_active == 1, "3dof: 1 active link");
    CHECK(cfg.active_link_map[0] == 1, "3dof: active[0]=1");
    CHECK(cfg.n_active_endpoints == 2, "3dof: 2 active endpoints");
    CHECK(cfg.last_active_frame == 2, "3dof: last_active_frame=2");

    CHECK(cfg.n_affecting_joints[0] == 2, "3dof: link 1 needs 2 joints (0,1)");
    CHECK_CLOSE(cfg.active_link_radii[0], 0.05, 1e-6, "3dof: radius=0.05");

    std::printf("  OK  Custom 3-DOF: n_active=%d, map=[%d]\n",
                cfg.n_active, cfg.active_link_map[0]);
}

// ?
//  Test 8: Robot.iaabb_config() is auto-built during construction
// ?
static void test_robot_auto_config() {
    SECTION("Robot.iaabb_config() auto-build");

    Robot panda = make_panda();
    const auto& cfg = panda.iaabb_config();

    CHECK(cfg.n_active == 2, "auto: panda n_active=2");
    CHECK(cfg.zero_length_threshold == 0.1, "auto: threshold=0.1");
    CHECK(cfg.active_link_map[0] == 2, "auto: map[0]=2");
    CHECK(cfg.active_link_map[1] == 4, "auto: map[1]=4");

    Robot iiwa = make_iiwa14();
    const auto& cfg2 = iiwa.iaabb_config();
    CHECK(cfg2.n_active == 3, "auto: iiwa n_active=3");
    CHECK(cfg2.active_link_map[2] == 7, "auto: iiwa map[2]=7");

    std::printf("  OK  auto-build correct\n");
}

// ?
//  Test 9: Edge case ?all links zero length
// ?
static void test_all_zero_links() {
    SECTION("All-zero-length links (edge case)");

    std::vector<DHParam> dh = {
        {0.0, 0.0, 0.0, 0.0, 0},
        {0.0, 0.0, 0.0, 0.0, 0},
    };
    JointLimits limits;
    limits.limits = {{-3.14, 3.14}, {-3.14, 3.14}};
    Robot robot("zero_robot", dh, limits);
    auto cfg = IAABBConfig::from_robot(robot, 0.1);

    CHECK(cfg.n_active == 0, "zero: no active links");
    CHECK(cfg.n_active_endpoints == 0, "zero: 0 endpoints");
    CHECK(cfg.last_active_frame == 0, "zero: last_active_frame=0");

    std::printf("  OK  all-zero: n_active=%d\n", cfg.n_active);
}

// ?
//  main
// ?
int main() {
    std::printf("?IAABBConfig Tests \n");

    test_panda_config();
    test_iiwa_config();
    test_threshold_comparison();
    test_json_roundtrip();
    test_affecting_joints_detail();
    test_summary();
    test_custom_3dof();
    test_robot_auto_config();
    test_all_zero_links();

    std::printf("\n\n");
    std::printf("  PASSED: %d   FAILED: %d   TOTAL: %d\n",
                g_pass, g_fail, g_pass + g_fail);
    std::printf("\n");

    return g_fail > 0 ? 1 : 0;
}

