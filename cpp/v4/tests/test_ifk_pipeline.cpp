// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — iFK Pipeline Integration Tests
//
//  Tests the complete iFK pipeline: interval_math → interval_fk →
//  endpoint_source → envelope_derive → LECT.
//
//  Build: cmake --build . --config Release --target test_ifk_pipeline
//  Run:   Release\test_ifk_pipeline.exe
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_math.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/envelope/endpoint_source.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/core/interval_trig.h"
#include "sbf/core/config.h"
#include "sbf/forest/lect.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cassert>
#include <vector>

using namespace sbf;

// ─── Helpers ────────────────────────────────────────────────────────────────
static int g_pass = 0, g_fail = 0;

#define CHECK(cond, msg) do {                                         \
    if (!(cond)) {                                                    \
        std::printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, msg); \
        ++g_fail;                                                     \
    } else { ++g_pass; }                                              \
} while(0)

#define CHECK_CLOSE(a, b, tol, msg) \
    CHECK(std::abs((a) - (b)) < (tol), msg)

#define SECTION(name) std::printf("\n── %s ──\n", (name))

// ─── Build IIWA14-like robot (programmatic, no JSON dependency) ─────────────
static Robot make_iiwa14() {
    // 7-DOF serial chain (DH from URDF, R820), all joints revolute, a≈0
    std::vector<DHParam> dh = {
        {  0.0,             0.0, 0.36,   0.0, 0},   // α=0
        { -HALF_PI,         0.0, 0.0,    0.0, 0},   // α=−π/2
        {  HALF_PI,         0.0, 0.42,   0.0, 0},   // α=+π/2
        {  HALF_PI,         0.0, 0.0,    0.0, 0},   // α=+π/2
        { -HALF_PI,         0.0, 0.4,    0.0, 0},   // α=−π/2
        { -HALF_PI,         0.0, 0.0,    0.0, 0},   // α=−π/2
        {  HALF_PI,         0.0, 0.0,    0.0, 0},   // α=+π/2
    };

    JointLimits limits;
    limits.limits = {
        {-2.9668, 2.9668},
        {-2.0942, 2.0942},
        {-2.9668, 2.9668},
        {-2.0942, 2.0942},
        {-2.9668, 2.9668},
        {-2.0942, 2.0942},
        {-3.0541, 3.0541},
    };

    DHParam tool{0.0, 0.0, 0.126, 0.0, 0};

    std::vector<double> radii = {0.08, 0.08, 0.06, 0.06, 0.04, 0.04, 0.04, 0.03};

    return Robot("iiwa14_test", dh, limits, tool, radii);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 1: interval_math — I_sin / I_cos with outward rounding
// ═══════════════════════════════════════════════════════════════════════════
static void test_interval_math() {
    SECTION("interval_math");

    // I_sin([0, π/2]) should contain [0, 1]
    auto s = I_sin(0.0, HALF_PI);
    CHECK(s.lo <= 0.0,  "I_sin([0,π/2]).lo ≤ 0");
    CHECK(s.hi >= 1.0,  "I_sin([0,π/2]).hi ≥ 1");

    // I_cos([0, π/2]) should contain [≈0, 1]
    // Note: cos(HALF_PI) ≈ 6.12e-17 (not exactly 0 in IEEE 754)
    auto c = I_cos(0.0, HALF_PI);
    CHECK(c.lo < 1e-10,  "I_cos([0,π/2]).lo ≈ 0 (< 1e-10)");
    CHECK(c.hi >= 1.0,  "I_cos([0,π/2]).hi ≥ 1");

    // I_sin([−π, π]) should contain [−1, 1]
    auto s2 = I_sin(-PI, PI);
    CHECK(s2.lo <= -1.0, "I_sin([−π,π]).lo ≤ −1");
    CHECK(s2.hi >=  1.0, "I_sin([−π,π]).hi ≥ 1");

    // I_cos([−π, π]) should contain [−1, 1]
    auto c2 = I_cos(-PI, PI);
    CHECK(c2.lo <= -1.0, "I_cos([−π,π]).lo ≤ −1");
    CHECK(c2.hi >=  1.0, "I_cos([−π,π]).hi ≥ 1");

    // Outward rounding: I_sin of a point should be slightly wider than exact
    auto sp = I_sin(0.5, 0.5);
    double exact_s = std::sin(0.5);
    CHECK(sp.lo <= exact_s, "I_sin(0.5).lo ≤ sin(0.5) (outward)");
    CHECK(sp.hi >= exact_s, "I_sin(0.5).hi ≥ sin(0.5) (outward)");
    // Should be very tight (within 1 ULP)
    CHECK(sp.hi - sp.lo < 1e-14, "I_sin(0.5) width < 1e-14 (tight)");

    std::printf("  interval_math: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 2: interval_fk — compute_fk_full
// ═══════════════════════════════════════════════════════════════════════════
static void test_interval_fk() {
    SECTION("interval_fk");

    Robot robot = make_iiwa14();

    // Point interval (single configuration) → AABB should be very tight
    std::vector<Interval> pt_ivs = {
        {0.0, 0.0}, {0.5, 0.5}, {-0.3, -0.3}, {-1.0, -1.0},
        {0.1, 0.1}, {0.2, 0.2}, {1.5, 1.5}
    };

    FKState fk = compute_fk_full(robot, pt_ivs);
    CHECK(fk.valid, "FKState.valid after compute_fk_full");
    CHECK(fk.n_tf == 9, "n_tf = 7 joints + base + tool = 9");
    CHECK(fk.n_jm == 8, "n_jm = 7 joints + tool = 8");

    // Check that position intervals are tight for point query
    // prefix[8] = tool frame (last endpoint)
    double x_lo = fk.prefix_lo[8][3];
    double x_hi = fk.prefix_hi[8][3];
    CHECK(x_hi - x_lo < 1e-10, "Point FK: x interval width < 1e-10");

    double y_lo = fk.prefix_lo[8][7];
    double y_hi = fk.prefix_hi[8][7];
    CHECK(y_hi - y_lo < 1e-10, "Point FK: y interval width < 1e-10");

    // Wider interval → wider AABB
    std::vector<Interval> wide_ivs = {
        {-0.5, 0.5}, {-0.1, 0.5}, {-0.3, 0.3}, {-1.5, -0.5},
        {-0.3, 0.3}, {-0.5, 0.5}, {1.0, 2.0}
    };

    FKState fk2 = compute_fk_full(robot, wide_ivs);
    CHECK(fk2.valid, "FKState.valid for wide intervals");

    double wx = fk2.prefix_hi[8][3] - fk2.prefix_lo[8][3];
    CHECK(wx > 0.01, "Wide FK: x interval > 0.01");

    // Extract link AABBs
    const int n_act = robot.n_active_links();
    std::vector<float> aabbs(n_act * 6);
    extract_link_aabbs(fk2, robot.active_link_map(), n_act,
                       aabbs.data(), robot.active_link_radii());

    // All AABBs should have lo ≤ hi
    for (int i = 0; i < n_act; ++i) {
        float* a = aabbs.data() + i * 6;
        CHECK(a[0] <= a[3], "link AABB: x_lo ≤ x_hi");
        CHECK(a[1] <= a[4], "link AABB: y_lo ≤ y_hi");
        CHECK(a[2] <= a[5], "link AABB: z_lo ≤ z_hi");
    }

    // Incremental FK: changing one dim should produce valid result
    std::vector<Interval> inc_ivs = wide_ivs;
    inc_ivs[3] = {-1.2, -0.7};
    FKState fk3 = compute_fk_incremental(fk2, robot, inc_ivs, 3);
    CHECK(fk3.valid, "FKState.valid after incremental FK");

    std::printf("  interval_fk: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 3: endpoint_source — compute_endpoint_iaabb
// ═══════════════════════════════════════════════════════════════════════════
static void test_endpoint_source() {
    SECTION("endpoint_source");

    Robot robot = make_iiwa14();
    auto cfg = envelope::EndpointSourceConfig::ifk();

    std::vector<Interval> ivs = {
        {-0.5, 0.5}, {-0.1, 0.5}, {-0.3, 0.3}, {-1.5, -0.5},
        {-0.3, 0.3}, {-0.5, 0.5}, {1.0, 2.0}
    };

    auto result = envelope::compute_endpoint_iaabb(cfg, robot, ivs);
    CHECK(result.n_active_ep == 6, "n_active_ep = n_active(3)*2 = 6");
    CHECK(result.n_active > 0, "n_active > 0");
    CHECK(result.has_fk_state(), "result has FK state");
    CHECK(result.endpoint_iaabb_len() == 6 * 6,
          "endpoint_iaabbs size = 36");

    // All endpoint AABBs: lo ≤ hi
    for (int k = 0; k < result.n_active_ep; ++k) {
        const float* ep = result.endpoint_iaabbs.data() + k * 6;
        CHECK(ep[0] <= ep[3], "endpoint AABB: x_lo ≤ x_hi");
        CHECK(ep[1] <= ep[4], "endpoint AABB: y_lo ≤ y_hi");
        CHECK(ep[2] <= ep[5], "endpoint AABB: z_lo ≤ z_hi");
    }

    // Extract link iAABBs
    int n_act = robot.n_active_links();
    std::vector<float> link_aabbs(n_act * 6);
    envelope::extract_link_iaabbs(result, robot, link_aabbs.data());

    for (int i = 0; i < n_act; ++i) {
        float* a = link_aabbs.data() + i * 6;
        CHECK(a[0] <= a[3], "link iAABB: x_lo ≤ x_hi");
        CHECK(a[1] <= a[4], "link iAABB: y_lo ≤ y_hi");
        CHECK(a[2] <= a[5], "link iAABB: z_lo ≤ z_hi");
    }

    // Incremental: change dim 2
    std::vector<Interval> ivs2 = ivs;
    ivs2[2] = {-0.1, 0.1};
    auto result2 = envelope::compute_endpoint_iaabb_incremental(
        cfg, result.fk_state, robot, ivs2, 2);
    CHECK(result2.has_fk_state(), "incremental result has FK state");

    std::printf("  endpoint_source: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 4: interval_trig — classify_alpha (DH preprocessing)
// ═══════════════════════════════════════════════════════════════════════════
static void test_interval_trig() {
    SECTION("interval_trig");

    // classify_alpha
    CHECK(classify_alpha(0.0) == 0,     "α=0 → type 0");
    CHECK(classify_alpha(-HALF_PI) == 1, "α=−π/2 → type 1");
    CHECK(classify_alpha(HALF_PI) == 2,  "α=+π/2 → type 2");
    CHECK(classify_alpha(1.0) == -1,     "α=1 → unsupported");

    std::printf("  interval_trig: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 5: LECT — construction and tree operations
// ═══════════════════════════════════════════════════════════════════════════
static void test_lect() {
    SECTION("LECT");

    Robot robot = make_iiwa14();

    // ── Construction with default pipeline ──────────────────────────────
    auto pipeline = envelope::PipelineConfig::fast();
    forest::LECT lect(robot, pipeline, 64);

    CHECK(lect.n_nodes() == 1, "LECT has 1 node (root) after construction");
    CHECK(lect.n_dims() == 7, "LECT n_dims = 7");
    CHECK(lect.n_active_links() > 0, "LECT n_active > 0");
    CHECK(lect.has_iaabb(0), "root node has iAABB");

    // ── Root link iAABBs should be valid ────────────────────────────────
    const float* root_aabbs = lect.get_link_iaabbs(0);
    CHECK(root_aabbs != nullptr, "root has link iAABBs");
    for (int i = 0; i < lect.n_active_links(); ++i) {
        const float* a = root_aabbs + i * 6;
        CHECK(a[0] <= a[3], "root link iAABB: x_lo ≤ x_hi");
        CHECK(a[1] <= a[4], "root link iAABB: y_lo ≤ y_hi");
        CHECK(a[2] <= a[5], "root link iAABB: z_lo ≤ z_hi");
    }

    // ── Root endpoint iAABBs should exist ───────────────────────────────
    CHECK(lect.has_endpoint_iaabbs(0), "root has endpoint iAABBs");

    // ── Tree structure: root is leaf, no children ───────────────────────
    CHECK(lect.is_leaf(0),  "root is leaf before split");
    CHECK(lect.left(0) < 0, "root has no left child");
    CHECK(lect.parent(0) < 0, "root has no parent");
    CHECK(lect.depth(0) == 0, "root depth = 0");

    // ── Occupation ──────────────────────────────────────────────────────
    CHECK(!lect.is_occupied(0), "root not occupied initially");
    lect.mark_occupied(0, 42);
    CHECK(lect.is_occupied(0),  "root occupied after mark");
    CHECK(lect.forest_id(0) == 42, "forest_id = 42");
    lect.unmark_occupied(0);
    CHECK(!lect.is_occupied(0), "root unoccupied after unmark");

    // ── Split root ──────────────────────────────────────────────────────
    int new_nodes = lect.expand_leaf(0);
    CHECK(new_nodes == 2, "expand_leaf returns 2 new nodes");

    CHECK(!lect.is_leaf(0),  "root no longer leaf after split");
    CHECK(lect.left(0) >= 0, "root has left child");
    CHECK(lect.right(0) >= 0, "root has right child");
    CHECK(lect.n_nodes() == 3, "3 nodes after root split");

    int left_idx = lect.left(0);
    int right_idx = lect.right(0);
    CHECK(lect.parent(left_idx) == 0, "left's parent = 0");
    CHECK(lect.parent(right_idx) == 0, "right's parent = 0");
    CHECK(lect.depth(left_idx) == 1, "left depth = 1");
    CHECK(lect.is_leaf(left_idx), "left is leaf");
    CHECK(lect.has_iaabb(left_idx), "left has iAABB");
    CHECK(lect.has_iaabb(right_idx), "right has iAABB");

    // ── Child link iAABBs should be valid ───────────────────────────────
    const float* left_aabbs = lect.get_link_iaabbs(left_idx);
    const float* right_aabbs = lect.get_link_iaabbs(right_idx);
    for (int i = 0; i < lect.n_active_links(); ++i) {
        const float* la = left_aabbs + i * 6;
        CHECK(la[0] <= la[3], "left link iAABB: x_lo ≤ x_hi");
        const float* ra = right_aabbs + i * 6;
        CHECK(ra[0] <= ra[3], "right link iAABB: x_lo ≤ x_hi");
    }

    // ── node_intervals for children ─────────────────────────────────────
    auto root_ivs = lect.node_intervals(0);
    auto left_ivs = lect.node_intervals(left_idx);
    auto right_ivs = lect.node_intervals(right_idx);
    CHECK(static_cast<int>(left_ivs.size()) == 7, "left ivs size = 7");

    // Children's intervals should be subsets of root
    for (int d = 0; d < 7; ++d) {
        CHECK(left_ivs[d].lo >= root_ivs[d].lo - 1e-10,
              "left interval ⊂ root (lo)");
        CHECK(left_ivs[d].hi <= root_ivs[d].hi + 1e-10,
              "left interval ⊂ root (hi)");
    }

    // Exactly one dimension was halved
    int split_count = 0;
    for (int d = 0; d < 7; ++d) {
        if (left_ivs[d].width() < root_ivs[d].width() - 1e-10)
            ++split_count;
    }
    CHECK(split_count == 1, "exactly 1 dimension halved");

    // ── Split left child too (depth 2) ──────────────────────────────────
    lect.expand_leaf(left_idx);
    CHECK(lect.n_nodes() == 5, "5 nodes after second split");

    std::printf("  LECT: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 6: LECT — refinement tightens parent AABBs
// ═══════════════════════════════════════════════════════════════════════════
static void test_refine_aabb() {
    SECTION("refine_aabb");

    Robot robot = make_iiwa14();
    auto pipeline = envelope::PipelineConfig::fast();
    forest::LECT lect(robot, pipeline, 64);

    // Get root AABBs before split
    const int n_act = lect.n_active_links();
    std::vector<float> root_before(n_act * 6);
    const float* r0 = lect.get_link_iaabbs(0);
    for (int i = 0; i < n_act * 6; ++i) root_before[i] = r0[i];

    // Split root
    lect.expand_leaf(0);

    // Root AABBs should be at least as tight after refinement
    const float* r1 = lect.get_link_iaabbs(0);
    for (int i = 0; i < n_act; ++i) {
        // lo should have been tightened (≥)
        CHECK(r1[i*6+0] >= root_before[i*6+0] - 1e-6f,
              "refine: lo_x tightened or unchanged");
        CHECK(r1[i*6+1] >= root_before[i*6+1] - 1e-6f,
              "refine: lo_y tightened or unchanged");
        CHECK(r1[i*6+2] >= root_before[i*6+2] - 1e-6f,
              "refine: lo_z tightened or unchanged");
        // hi should have been tightened (≤)
        CHECK(r1[i*6+3] <= root_before[i*6+3] + 1e-6f,
              "refine: hi_x tightened or unchanged");
        CHECK(r1[i*6+4] <= root_before[i*6+4] + 1e-6f,
              "refine: hi_y tightened or unchanged");
        CHECK(r1[i*6+5] <= root_before[i*6+5] + 1e-6f,
              "refine: hi_z tightened or unchanged");
    }

    std::printf("  refine_aabb: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Test 7: LECT — BEST_TIGHTEN split dimension selection
// ═══════════════════════════════════════════════════════════════════════════
static void test_lect_best_tighten() {
    SECTION("LECT BEST_TIGHTEN");

    Robot robot = make_iiwa14();
    auto pipeline = envelope::PipelineConfig::fast();
    forest::LECT lect(robot, pipeline, 64);
    lect.set_split_order(SplitOrder::BEST_TIGHTEN);

    // 1. Basic structure correctness — split produces valid children
    lect.expand_leaf(0);
    CHECK(lect.n_nodes() == 3, "3 nodes after root split");
    CHECK(lect.has_iaabb(lect.left(0)), "left has iAABB");
    CHECK(lect.has_iaabb(lect.right(0)), "right has iAABB");

    // 2. Exactly 1 dimension was halved
    auto root_ivs = lect.node_intervals(0);
    auto left_ivs = lect.node_intervals(lect.left(0));
    int split_count = 0;
    for (int d = 0; d < 7; ++d)
        if (left_ivs[d].width() < root_ivs[d].width() - 1e-10)
            ++split_count;
    CHECK(split_count == 1, "exactly 1 dimension halved");

    // 3. Same-depth consistency — both children split along the same dim
    int left_idx = lect.left(0);
    int right_idx = lect.right(0);
    lect.expand_leaf(left_idx);
    lect.expand_leaf(right_idx);
    // depth=1: 4 grandchildren should use the same split dimension
    auto left1_ivs = lect.node_intervals(left_idx);
    auto right1_ivs = lect.node_intervals(right_idx);
    auto ll_ivs = lect.node_intervals(lect.left(left_idx));
    auto rl_ivs = lect.node_intervals(lect.left(right_idx));
    int dim_ll = -1, dim_rl = -1;
    for (int d = 0; d < 7; ++d) {
        if (ll_ivs[d].width() < left1_ivs[d].width() - 1e-10) dim_ll = d;
        if (rl_ivs[d].width() < right1_ivs[d].width() - 1e-10) dim_rl = d;
    }
    CHECK(dim_ll == dim_rl, "same depth -> same split dimension");

    // 4. Deep expansion does not crash
    for (int iter = 0; iter < 5; ++iter) {
        int n = lect.n_nodes();
        for (int j = 0; j < n; ++j) {
            if (lect.is_leaf(j))
                lect.expand_leaf(j);
        }
    }
    CHECK(lect.n_nodes() > 3, "deep expansion succeeds");

    // 5. Verify split_order() getter
    CHECK(lect.split_order() == SplitOrder::BEST_TIGHTEN,
          "split_order() returns BEST_TIGHTEN");

    std::printf("  LECT BEST_TIGHTEN: OK\n");
}

// ═══════════════════════════════════════════════════════════════════════════
//  Main
// ═══════════════════════════════════════════════════════════════════════════
int main() {
    std::printf("═══ SafeBoxForest v4 — iFK Pipeline Tests ═══\n");

    test_interval_math();
    test_interval_fk();
    test_endpoint_source();
    test_interval_trig();
    test_lect();
    test_refine_aabb();
    test_lect_best_tighten();

    std::printf("\n═══ Results: %d passed, %d failed ═══\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
