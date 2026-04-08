// ═══════════════════════════════════════════════════════════════════════════
//  LECT Demo — build a tree, verify data caching, test save/load
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/forest/lect.h"
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"

#include <filesystem>
#include <iostream>
#include <string>

namespace fs = std::filesystem;

int main() {
    // ── Load robot ──────────────────────────────────────────────────────
    std::string robot_path =
        "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong"
        "/Desktop/code/box_aabb/safeboxforest/v1/configs/iiwa14.json";

    std::cout << "Loading robot from: " << robot_path << "\n";
    sbf::Robot robot = sbf::Robot::from_json(robot_path);
    std::cout << "  name:          " << robot.name()
              << "\n  n_joints:      " << robot.n_joints()
              << "\n  n_active_links:" << robot.n_active_links()
              << "\n  has_link_radii:" << robot.has_link_radii()
              << "\n\n";

    // ── Create LECT ─────────────────────────────────────────────────────
    double voxel_delta = 0.02;
    sbf::forest::LECT lect(robot, voxel_delta);

    std::cout << "=== LECT created ===\n"
              << "  n_nodes:    " << lect.n_nodes()     << "\n"
              << "  n_dims:     " << lect.n_dims()      << "\n"
              << "  n_active:   " << lect.n_active_links() << "\n"
              << "  voxel_delta:" << lect.voxel_delta()  << "\n\n";

    // ── Verify root node (index 0) has cached data ──────────────────────
    std::cout << "--- Root node (0) ---\n";
    std::cout << "  has_aabb:    " << lect.has_aabb(0) << "\n";
    std::cout << "  has_frames:  " << lect.has_frames(0) << "\n";
    std::cout << "  has_hull:    " << lect.has_hull_grid(0) << "\n";

    if (lect.has_aabb(0)) {
        const float* aabb = lect.get_link_aabbs(0);
        std::cout << "  Link AABBs:\n";
        for (int k = 0; k < lect.n_active_links(); ++k) {
            std::cout << "    link " << k << ": ["
                      << aabb[k*6+0] << "," << aabb[k*6+1] << "," << aabb[k*6+2]
                      << "] -> ["
                      << aabb[k*6+3] << "," << aabb[k*6+4] << "," << aabb[k*6+5]
                      << "]\n";
        }
    }

    if (lect.has_hull_grid(0)) {
        const auto& hull = lect.get_hull_grid(0);
        std::cout << "  Hull-16: " << hull.count_occupied() << " voxels, "
                  << hull.num_bricks() << " bricks\n";
    }

    // ── Test FFB (no obstacles) ─────────────────────────────────────────
    std::cout << "\n--- FFB test (no obstacles) ---\n";
    Eigen::VectorXd seed = Eigen::VectorXd::Zero(robot.n_joints());
    sbf::FFBResult res = lect.find_free_box(seed, nullptr, 0, 1e-4, 20);

    std::cout << "  success:    " << res.success() << "\n"
              << "  node_idx:   " << res.node_idx << "\n"
              << "  fail_code:  " << res.fail_code << "\n"
              << "  path_len:   " << res.path.size() << "\n"
              << "  new_nodes:  " << res.n_new_nodes << "\n"
              << "  fk_calls:   " << res.n_fk_calls << "\n";

    std::cout << "\n  After FFB: n_nodes = " << lect.n_nodes() << "\n";
    std::cout << "  nodes_with_aabb: " << lect.count_nodes_with_aabb() << "\n";
    std::cout << "  nodes_with_hull: " << lect.count_nodes_with_hull() << "\n";
    std::cout << "  total_voxels:    " << lect.total_hull_voxels() << "\n";

    // ── Test FFB (with obstacle) ────────────────────────────────────────
    std::cout << "\n--- FFB test (with obstacle) ---\n";
    sbf::Obstacle obs(Eigen::Vector3d(0.5, 0.0, 0.3),
                      Eigen::Vector3d(0.1, 0.1, 0.1),
                      "test_box");

    seed = Eigen::VectorXd::Constant(robot.n_joints(), 0.5);
    res = lect.find_free_box(seed, &obs, 1, 1e-3, 15);

    std::cout << "  success:    " << res.success() << "\n"
              << "  node_idx:   " << res.node_idx << "\n"
              << "  fail_code:  " << res.fail_code << "\n"
              << "  path_len:   " << res.path.size() << "\n"
              << "  new_nodes:  " << res.n_new_nodes << "\n"
              << "  fk_calls:   " << res.n_fk_calls << "\n";
    std::cout << "  After FFB: n_nodes = " << lect.n_nodes() << "\n";

    if (res.success()) {
        std::cout << "\n  Found free box at node " << res.node_idx << ":\n";
        auto ivs = lect.node_intervals(res.node_idx);
        for (int d = 0; d < static_cast<int>(ivs.size()); ++d)
            std::cout << "    dim " << d << ": [" << ivs[d].lo << ", " << ivs[d].hi << "]\n";

        // Mark occupied
        lect.mark_occupied(res.node_idx, 1);
        std::cout << "  Marked occupied with box_id=1\n";
    }

    // ── Test save/load ──────────────────────────────────────────────────
    std::cout << "\n=== Save/Load test ===\n";
    std::string save_dir = "lect_test_output";
    fs::create_directories(save_dir);

    std::cout << "  Saving to: " << save_dir << " ...\n";
    lect.save(save_dir);
    std::cout << "  Saved: lect.hcache + lect.frames + lect.hulls\n";

    // Load into a new LECT
    sbf::forest::LECT lect2;
    lect2.load(save_dir, robot);

    std::cout << "\n  Loaded LECT2:\n"
              << "    n_nodes:    " << lect2.n_nodes() << "\n"
              << "    with_aabb:  " << lect2.count_nodes_with_aabb() << "\n"
              << "    with_hull:  " << lect2.count_nodes_with_hull() << "\n"
              << "    total_vox:  " << lect2.total_hull_voxels() << "\n";

    // Verify root node data matches
    if (lect2.has_hull_grid(0)) {
        int orig_vox  = lect.get_hull_grid(0).count_occupied();
        int load_vox  = lect2.get_hull_grid(0).count_occupied();
        bool match = (orig_vox == load_vox);
        std::cout << "\n  Root hull voxels — orig: " << orig_vox
                  << ", loaded: " << load_vox
                  << " → " << (match ? "MATCH ✓" : "MISMATCH ✗") << "\n";
    }

    // Verify link AABBs match
    if (lect2.has_aabb(0)) {
        const float* a1 = lect.get_link_aabbs(0);
        const float* a2 = lect2.get_link_aabbs(0);
        bool match = true;
        for (int i = 0; i < lect.n_active_links() * 6; ++i) {
            if (std::abs(a1[i] - a2[i]) > 1e-6f) {
                match = false;
                break;
            }
        }
        std::cout << "  Root AABBs — " << (match ? "MATCH ✓" : "MISMATCH ✗") << "\n";
    }

    std::cout << "\n=== LECT demo complete ===\n";
    return 0;
}
