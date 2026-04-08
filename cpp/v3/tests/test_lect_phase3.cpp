// ═══════════════════════════════════════════════════════════════════════════
//  Phase 3 Test — Scene Grid, Coarsening, collides_scene
//
//  Tests the new LECT functionality:
//    1. set_scene() pre-rasterizes obstacles → O(1) per-query hull collision
//    2. merge_children_hulls() performs zero-loss bitwise-OR merge
//    3. coarsen_volume_ratio() evaluates coarsening quality
//    4. collides_scene() two-stage AABB→Hull collision
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/forest/lect.h"
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"

#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <vector>

using namespace sbf;
using namespace sbf::forest;

static const std::string ROBOT_PATH =
    "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong"
    "/Desktop/code/box_aabb/safeboxforest/v1/configs/iiwa14.json";

// ─── Helper: build obstacles ────────────────────────────────────────────────
static std::vector<Obstacle> make_test_obstacles() {
    std::vector<Obstacle> obs;
    obs.emplace_back(Eigen::Vector3d(0.5, 0.0, 0.3),
                     Eigen::Vector3d(0.1, 0.1, 0.1), "box1");
    obs.emplace_back(Eigen::Vector3d(-0.3, 0.4, 0.5),
                     Eigen::Vector3d(0.05, 0.05, 0.05), "box2");
    obs.emplace_back(Eigen::Vector3d(0.0, -0.5, 0.1),
                     Eigen::Vector3d(0.15, 0.08, 0.2), "box3");
    return obs;
}

// ─── Test 1: Scene grid pre-rasterization ───────────────────────────────────
static void test_scene_grid() {
    std::cout << "=== Test 1: Scene Grid Pre-Rasterization ===\n";

    Robot robot = Robot::from_json(ROBOT_PATH);
    LECT lect(robot, 0.02);

    auto obs_vec = make_test_obstacles();
    assert(!lect.has_scene_grid());

    // Set scene
    lect.set_scene(obs_vec.data(), static_cast<int>(obs_vec.size()));
    assert(lect.has_scene_grid());
    int sv = lect.scene_grid_voxels();
    std::cout << "  Scene grid: " << sv << " voxels, "
              << lect.scene_grid().num_bricks() << " bricks\n";
    assert(sv > 0);

    // Clear scene
    lect.clear_scene();
    assert(!lect.has_scene_grid());
    assert(lect.scene_grid_voxels() == 0);

    std::cout << "  PASS\n\n";
}

// ─── Test 2: hull_collides with/without scene grid (performance) ────────────
static void test_hull_collides_perf() {
    std::cout << "=== Test 2: Hull Collision — Scene Grid vs On-The-Fly ===\n";

    Robot robot = Robot::from_json(ROBOT_PATH);
    LECT lect(robot, 0.02);

    auto obs_vec = make_test_obstacles();
    const int n_obs = static_cast<int>(obs_vec.size());

    // Build some tree nodes by calling FFB
    Eigen::VectorXd seed = Eigen::VectorXd::Zero(robot.n_joints());
    auto res = lect.find_free_box(seed, obs_vec.data(), n_obs, 1e-3, 12);
    std::cout << "  FFB result: " << (res.success() ? "OK" : "FAIL")
              << ", nodes=" << lect.n_nodes()
              << ", with_hull=" << lect.count_nodes_with_hull() << "\n";

    // --- Without scene grid (on-the-fly, baseline) ---
    const int N_ITER = 100;
    int total_collisions_baseline = 0;
    auto t1 = std::chrono::high_resolution_clock::now();
    for (int iter = 0; iter < N_ITER; ++iter) {
        for (int i = 0; i < lect.n_nodes(); ++i) {
            if (lect.has_hull_grid(i)) {
                if (lect.hull_collides_grid(i,
                        [&]() {
                            voxel::VoxelGrid g(0.02);
                            for (auto& ob : obs_vec) {
                                auto lo = ob.lo(); auto hi = ob.hi();
                                float a[6] = {(float)lo[0],(float)lo[1],(float)lo[2],
                                              (float)hi[0],(float)hi[1],(float)hi[2]};
                                g.fill_aabb(a);
                            }
                            return g;
                        }()))
                    ++total_collisions_baseline;
            }
        }
    }
    auto t2 = std::chrono::high_resolution_clock::now();
    double ms_baseline = std::chrono::duration<double, std::milli>(t2 - t1).count();

    // --- With scene grid (pre-rasterized, optimized) ---
    lect.set_scene(obs_vec.data(), n_obs);
    int total_collisions_scene = 0;
    auto t3 = std::chrono::high_resolution_clock::now();
    for (int iter = 0; iter < N_ITER; ++iter) {
        for (int i = 0; i < lect.n_nodes(); ++i) {
            if (lect.has_hull_grid(i)) {
                if (lect.hull_collides_grid(i, lect.scene_grid()))
                    ++total_collisions_scene;
            }
        }
    }
    auto t4 = std::chrono::high_resolution_clock::now();
    double ms_scene = std::chrono::duration<double, std::milli>(t4 - t3).count();

    std::cout << "  Baseline (on-the-fly): " << ms_baseline << " ms (" << N_ITER << " iters)\n"
              << "  Scene grid:            " << ms_scene    << " ms (" << N_ITER << " iters)\n"
              << "  Speedup:               " << ms_baseline / std::max(ms_scene, 0.001)
              << "x\n";

    // Results must match
    assert(total_collisions_baseline == total_collisions_scene);
    std::cout << "  Collision results match: " << total_collisions_scene << " hits\n";
    std::cout << "  PASS\n\n";
}

// ─── Test 3: Coarsening — merge children hulls ─────────────────────────────
static void test_coarsening() {
    std::cout << "=== Test 3: Hull-Based Coarsening ===\n";

    Robot robot = Robot::from_json(ROBOT_PATH);
    LECT lect(robot, 0.02);

    // Use obstacles to force FFB to split repeatedly
    auto obs_vec = make_test_obstacles();
    const int n_obs = static_cast<int>(obs_vec.size());
    lect.set_scene(obs_vec.data(), n_obs);

    // Build tree nodes with multiple FFB calls at different seeds
    std::vector<Eigen::VectorXd> seeds;
    for (double v = -1.0; v <= 1.0; v += 0.5) {
        Eigen::VectorXd s = Eigen::VectorXd::Constant(robot.n_joints(), v);
        seeds.push_back(s);
    }
    // Also add some off-axis seeds
    for (int d = 0; d < robot.n_joints(); ++d) {
        Eigen::VectorXd s = Eigen::VectorXd::Zero(robot.n_joints());
        s[d] = 1.5;
        seeds.push_back(s);
        s[d] = -1.5;
        seeds.push_back(s);
    }

    for (auto& seed : seeds) {
        lect.find_free_box(seed, obs_vec.data(), n_obs, 1e-2, 12);
    }
    std::cout << "  After " << seeds.size() << " FFB calls: n_nodes=" << lect.n_nodes()
              << ", with_hull=" << lect.count_nodes_with_hull() << "\n";

    // Find a non-leaf node with both children having hull grids
    int test_parent = -1;
    for (int i = 0; i < lect.n_nodes(); ++i) {
        if (!lect.is_leaf(i)) {
            int li = lect.left(i);
            int ri = lect.right(i);
            if (li >= 0 && ri >= 0 &&
                lect.has_hull_grid(li) && lect.has_hull_grid(ri))
            {
                test_parent = i;
                break;
            }
        }
    }

    if (test_parent < 0) {
        std::cout << "  SKIP: no non-leaf node with both children having hulls\n\n";
        return;
    }

    int li = lect.left(test_parent);
    int ri = lect.right(test_parent);

    double lv = lect.get_hull_grid(li).occupied_volume();
    double rv = lect.get_hull_grid(ri).occupied_volume();
    double pv = lect.get_hull_grid(test_parent).occupied_volume();

    std::cout << "  Node " << test_parent << ": left=" << li << " right=" << ri << "\n"
              << "    Left  hull vol:  " << lv << " m^3\n"
              << "    Right hull vol:  " << rv << " m^3\n"
              << "    Parent hull vol: " << pv << " m^3\n";

    // Test merged volume
    double merged_vol = lect.merged_children_hull_volume(test_parent);
    assert(merged_vol >= 0);
    std::cout << "    Merged vol:      " << merged_vol << " m^3\n";

    // merged >= max(left, right) (union can only grow)
    assert(merged_vol >= std::max(lv, rv) - 1e-9);
    // merged <= left + right (union ≤ sum)
    assert(merged_vol <= lv + rv + 1e-9);

    // Volume ratio
    double ratio = lect.coarsen_volume_ratio(test_parent);
    std::cout << "    Volume ratio:    " << ratio << "\n";
    assert(ratio >= 1.0 - 1e-6);  // ratio ≥ 1 always

    // Actually merge
    bool ok = lect.merge_children_hulls(test_parent);
    assert(ok);
    double new_pv = lect.get_hull_grid(test_parent).occupied_volume();
    std::cout << "    After merge:     " << new_pv << " m^3\n";

    // Merged parent == computed merged_vol
    assert(std::abs(new_pv - merged_vol) < 1e-9);
    std::cout << "  PASS\n\n";
}

// ─── Test 4: collides_scene (two-stage) ─────────────────────────────────────
static void test_collides_scene() {
    std::cout << "=== Test 4: collides_scene (AABB → Hull) ===\n";

    Robot robot = Robot::from_json(ROBOT_PATH);
    LECT lect(robot, 0.02);

    auto obs_vec = make_test_obstacles();
    const int n_obs = static_cast<int>(obs_vec.size());

    // Pre-rasterize scene for fast hull queries
    lect.set_scene(obs_vec.data(), n_obs);

    // Build tree
    Eigen::VectorXd seed = Eigen::VectorXd::Zero(robot.n_joints());
    auto res = lect.find_free_box(seed, obs_vec.data(), n_obs, 1e-3, 12);
    std::cout << "  FFB: " << (res.success() ? "OK" : "FAIL")
              << ", node=" << res.node_idx
              << ", n_nodes=" << lect.n_nodes() << "\n";

    // Test collides_scene on root (should collide — root covers full C-space)
    bool root_collides = lect.collides_scene(0, obs_vec.data(), n_obs);
    std::cout << "  Root collides_scene: " << root_collides << "\n";
    assert(root_collides);  // root hull should overlap obstacles

    // If FFB found a free box, its node should NOT collide
    if (res.success()) {
        bool leaf_collides = lect.collides_scene(res.node_idx, obs_vec.data(), n_obs);
        std::cout << "  FFB leaf collides_scene: " << leaf_collides << "\n";
        assert(!leaf_collides);  // FFB guarantees collision-free
    }

    std::cout << "  PASS\n\n";
}

// ═══════════════════════════════════════════════════════════════════════════
int main() {
    std::cout << "╔══════════════════════════════════════════════════════╗\n"
              << "║  LECT Phase 3 Tests — Scene Grid + Coarsening      ║\n"
              << "╚══════════════════════════════════════════════════════╝\n\n";

    test_scene_grid();
    test_hull_collides_perf();
    test_coarsening();
    test_collides_scene();

    std::cout << "═══════════════════════════════════════════════════════\n"
              << "  ALL TESTS PASSED\n"
              << "═══════════════════════════════════════════════════════\n";
    return 0;
}
