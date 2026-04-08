// Example: Plan a Panda arm motion with random obstacles
#include "sbf/planner/sbf_planner.h"
#include "sbf/planner/pipeline.h"
#include "sbf/io/json_io.h"
#include <iostream>
#include <iomanip>
#include <random>

int main(int argc, char** argv) {
    using namespace sbf;

    std::string robot_path = "configs/panda.json";
    if (argc > 1) robot_path = argv[1];

    std::cout << "Loading robot from: " << robot_path << "\n";
    Robot robot = Robot::from_json(robot_path);
    std::cout << "  DOF: " << robot.n_joints()
              << ", links: " << robot.n_links() << "\n";

    // Start / goal
    Eigen::VectorXd start(7), goal(7);
    start << 0, -0.785, 0, -2.356, 0, 1.571, 0.785;
    goal  << 1.5, -0.5, 0.3, -1.8, -0.2, 1.2, 0.5;

    // Random scene
    SceneConfig scene_cfg;
    scene_cfg.n_obstacles = 8;
    std::mt19937 rng(42);
    auto obstacles = build_random_scene(robot, scene_cfg, start, goal, rng);

    std::cout << "Generated " << obstacles.size() << " obstacles\n";

    SBFConfig cfg = make_panda_config(42);
    cfg.max_boxes = 500;

    std::cout << "Planning...\n";
    SBFPlanner planner(robot, obstacles, cfg);
    auto result = planner.plan(start, goal, 30.0);

    std::cout << "─────────────────────────────────────\n";
    std::cout << "  Success:    " << (result.success ? "YES" : "NO") << "\n";
    std::cout << "  Cost:       " << std::fixed << std::setprecision(4)
              << result.cost << "\n";
    std::cout << "  Time:       " << result.planning_time << " s\n";
    std::cout << "  Path pts:   " << result.path.rows() << "\n";
    std::cout << "  Boxes:      " << result.nodes_explored << "\n";
    std::cout << "  Coll.checks:" << result.collision_checks << "\n";

    if (result.success) {
        save_result_json("plan_result.json", result);
        std::cout << "  Result saved to plan_result.json\n";
    }

    return result.success ? 0 : 1;
}
