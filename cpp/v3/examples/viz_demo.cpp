// SafeBoxForest v3 — Visualization Demo
//
// Loads a robot model, computes interval-FK envelope + voxel grid,
// and exports JSON files for the Python sbf_viz viewer.
//
// Build:  cmake --build build --target viz_demo --config Release
// Run:    build/Release/viz_demo <robot_json> <output_dir>
//
// Example (use absolute paths or run from the v3/ directory):
//   viz_demo C:/path/to/v1/configs/iiwa14.json examples/viz_output
//   cd python && python -m sbf_viz.run_demo ../examples/viz_output

#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/envelope/frame_store.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/scene/scene.h"
#include "sbf/voxel/voxel_grid.h"
#include "sbf/voxel/hull_rasteriser.h"
#include "sbf/viz/viz_exporter.h"

#include <Eigen/Core>
#include <iostream>
#include <filesystem>
#include <chrono>
#include <cmath>

namespace fs = std::filesystem;

int main(int argc, char** argv)
{
  try {
    // ── Parse args ──
    std::string robot_path = (argc > 1) ? argv[1] : "../../v1/configs/iiwa14.json";
    std::string out_dir    = (argc > 2) ? argv[2] : "viz_output";
    fs::create_directories(out_dir);

    std::cout << "[viz_demo] robot: " << robot_path << "\n"
              << "[viz_demo] output: " << out_dir << "\n";
    std::cout.flush();

    // ── Load robot ──
    std::cout << "[viz_demo] loading robot...\n"; std::cout.flush();
    auto robot = sbf::Robot::from_json(robot_path);
    std::cout << "[viz_demo] " << robot.name()
              << "  joints=" << robot.n_joints()
              << "  active_links=" << robot.n_active_links() << "\n";
    std::cout.flush();

    // ── Define configurations ──
    const int D = robot.n_joints();
    Eigen::VectorXd q_mid = Eigen::VectorXd::Zero(D);
    Eigen::VectorXd q_up  = Eigen::VectorXd::Zero(D);
    q_up[1] = -0.5;  q_up[3] = -1.2;

    std::vector<Eigen::VectorXd> configs = {q_mid, q_up};

    // ── Export robot FK ──
    sbf::viz::export_robot_json(robot, configs, out_dir + "/robot.json");
    std::cout << "[viz_demo] wrote robot.json (" << configs.size() << " configs)\n";

    // ── Define a C-space box (10% of joint range around q_mid) ──
    std::vector<sbf::Interval> box(D);
    for (int d = 0; d < D; ++d) {
        double lo = robot.joint_limits().limits[d].lo;
        double hi = robot.joint_limits().limits[d].hi;
        double range = hi - lo;
        double center = q_mid[d];
        box[d] = {center - 0.05 * range, center + 0.05 * range};
    }

    // ── Compute FK + envelope ──
    sbf::FKState fk = sbf::compute_fk_full(robot, box);
    sbf::envelope::FrameStore store(robot, 4);
    store.store_from_fk(0, fk);

    int n_sub = 8;
    std::vector<std::vector<sbf::Interval>> boxes_vec = {box};
    sbf::viz::export_envelope_from_boxes_json(robot, boxes_vec, n_sub,
                                               out_dir + "/envelope.json");
    std::cout << "[viz_demo] wrote envelope.json (n_sub=" << n_sub << ")\n";

    // ── Compute voxel grid (robot envelope, hull-16) ──
    double delta = 0.01;   // finer grid → ≤6% discretisation error (was 0.02 → ~30%)
    sbf::voxel::VoxelGrid robot_grid(delta, 0, 0, 0);
    sbf::voxel::rasterise_robot_hull16(robot, fk, robot_grid, n_sub);

    sbf::viz::export_voxel_centres_json(robot_grid,
                                         out_dir + "/voxel_robot.json");
    std::cout << "[viz_demo] wrote voxel_robot.json ("
              << robot_grid.count_occupied() << " voxels, "
              << robot_grid.num_bricks() << " bricks)\n";

    // ── Define scene (obstacles) ──
    sbf::Scene scene;
    scene.add_obstacle(sbf::Obstacle(
        Eigen::Vector3d(0.5, 0.0, 0.3),
        Eigen::Vector3d(0.3, 0.4, 0.02),
        "table"));
    scene.add_obstacle(sbf::Obstacle(
        Eigen::Vector3d(-0.3, 0.3, 0.5),
        Eigen::Vector3d(0.05, 0.05, 0.5),
        "pillar"));

    sbf::viz::export_scene_json(scene, out_dir + "/scene.json");
    std::cout << "[viz_demo] wrote scene.json ("
              << scene.n_obstacles() << " obstacles)\n";

    // ── Compute voxel grid (obstacles) ──
    sbf::voxel::VoxelGrid obs_grid(delta, 0, 0, 0);
    for (const auto& obs : scene.obstacles()) {
        sbf::voxel::rasterise_box_obstacle(
            obs.center.x(), obs.center.y(), obs.center.z(),
            obs.half_sizes.x(), obs.half_sizes.y(), obs.half_sizes.z(),
            obs_grid);
    }

    sbf::viz::export_voxel_centres_json(obs_grid,
                                         out_dir + "/voxel_obstacle.json");
    std::cout << "[viz_demo] wrote voxel_obstacle.json ("
              << obs_grid.count_occupied() << " voxels)\n";

    // ── Export combined snapshot ──
    sbf::viz::export_snapshot_json(robot, q_mid, box,
                                    robot_grid, obs_grid, scene,
                                    n_sub, out_dir + "/snapshot.json");
    std::cout << "[viz_demo] wrote snapshot.json\n";

    // ── Export multi-method envelope comparison ──
    sbf::viz::export_envelope_comparison_json(robot, box, n_sub, delta,
                                               out_dir + "/envelope_comparison.json");
    std::cout << "[viz_demo] wrote envelope_comparison.json (7 methods)\n";

    // ── Check collision ──
    bool col = robot_grid.collides(obs_grid);
    std::cout << "[viz_demo] voxel collision: " << (col ? "YES" : "NO") << "\n";

    std::cout << "\n[viz_demo] Done. Run Python viewer:\n"
              << "  cd python && python -m sbf_viz.run_demo ../"
              << out_dir << "\n";

    return 0;
  } catch (const std::exception& e) {
    std::cerr << "[viz_demo] EXCEPTION: " << e.what() << "\n";
    return 1;
  } catch (...) {
    std::cerr << "[viz_demo] UNKNOWN EXCEPTION\n";
    return 2;
  }
}
