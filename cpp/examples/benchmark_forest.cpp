// Benchmark: forest building speed
#include "sbf/planner/sbf_planner.h"
#include "sbf/planner/pipeline.h"
#include <iostream>
#include <chrono>
#include <iomanip>
#include <random>

int main(int argc, char** argv) {
    using namespace sbf;

    std::string robot_path = "configs/panda.json";
    if (argc > 1) robot_path = argv[1];
    int n_trials = 5;
    if (argc > 2) n_trials = std::atoi(argv[2]);

    Robot robot = Robot::from_json(robot_path);

    Eigen::VectorXd start(7), goal(7);
    start << 0, -0.785, 0, -2.356, 0, 1.571, 0.785;
    goal  << 1.5, -0.5, 0.3, -1.8, -0.2, 1.2, 0.5;

    std::cout << "Benchmark: " << n_trials << " trials\n";
    std::cout << std::fixed << std::setprecision(4);

    SceneConfig scene_cfg;
    scene_cfg.n_obstacles = 10;

    for (int t = 0; t < n_trials; ++t) {
        std::mt19937 rng(100 + t);
        auto obstacles = build_random_scene(robot, scene_cfg, start, goal, rng);

        SBFConfig cfg = make_panda_config(100 + t);

        auto t0 = std::chrono::steady_clock::now();
        SBFPlanner planner(robot, obstacles, cfg);
        auto result = planner.plan(start, goal, 30.0);
        double dt = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();

        std::cout << "  Trial " << t << ": "
                  << (result.success ? "OK" : "FAIL")
                  << "  time=" << dt << "s"
                  << "  cost=" << result.cost
                  << "  boxes=" << result.nodes_explored
                  << "\n";
    }

    return 0;
}
