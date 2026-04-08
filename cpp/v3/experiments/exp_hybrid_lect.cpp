// ═══════════════════════════════════════════════════════════════════════════
//  Experiment: Hybrid IA/AA LECT test
//
//  Builds LECT tree with AA crossover enabled and compares timings & node
//  counts against pure-IA mode.
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/robot/robot.h"
#include "sbf/forest/lect.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/common/types.h"

#include <chrono>
#include <cstdio>
#include <random>
#include <string>
#include <vector>
#include <Eigen/Core>

using namespace sbf;

static const std::string ROBOT_DIR =
    "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong"
    "/Desktop/code/box_aabb/safeboxforest/v1/configs/";

// Time find_free_box descent
struct FFBBench {
    int n_nodes;
    int n_fk_calls;
    double time_ms;
    bool found;
};

static FFBBench bench_ffb(forest::LECT& lect,
                          const Eigen::VectorXd& seed,
                          const Obstacle* obs, int n_obs,
                          double min_edge, int max_depth)
{
    auto t0 = std::chrono::high_resolution_clock::now();
    auto result = lect.find_free_box(seed, obs, n_obs, min_edge, max_depth);
    auto t1 = std::chrono::high_resolution_clock::now();

    FFBBench b;
    b.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    b.n_nodes = lect.n_nodes();
    b.n_fk_calls = result.n_fk_calls;
    b.found = result.success();
    return b;
}

int main() {
    const char* robots[] = {"panda", "iiwa14"};

    // Simple obstacle set
    std::vector<Obstacle> obstacles;
    {
        Obstacle o;
        o.center[0] = 0.5; o.center[1] = 0.0; o.center[2] = 0.5;
        o.half_sizes[0] = 0.1; o.half_sizes[1] = 0.1; o.half_sizes[2] = 0.1;
        obstacles.push_back(o);
    }

    std::mt19937 rng(42);

    fprintf(stderr, "%-8s %8s | %8s %8s %10s | %8s %8s %10s | crossover_w\n",
            "robot", "mode", "nodes", "fk_calls", "time_ms",
            "nodes", "fk_calls", "time_ms");
    fprintf(stderr, "%-8s %8s | %8s %8s %10s | %8s %8s %10s |\n",
            "", "", "--- IA ---", "", "",
            "--- AA ---", "", "");

    for (const char* rname : robots) {
        Robot robot = Robot::from_json(ROBOT_DIR + rname + ".json");

        int n = robot.n_joints();

        // Generate a random seed inside joint limits
        Eigen::VectorXd seed(n);
        const auto& lim = robot.joint_limits();
        for (int i = 0; i < n; ++i) {
            double lo = lim.limits[i].lo, hi = lim.limits[i].hi;
            std::uniform_real_distribution<double> dist(lo * 0.3 + hi * 0.7,
                                                        lo * 0.2 + hi * 0.8);
            seed[i] = dist(rng);
        }

        double crossover_values[] = {0.0, 0.05, 0.10, 0.20};

        for (double cw : crossover_values) {
            // Build pipeline config with AA crossover
            auto pipeline = envelope::PipelineConfig::production();
            pipeline.aa_crossover_width = cw;

            forest::LECT lect(robot, pipeline);

            auto b = bench_ffb(lect, seed, obstacles.data(),
                               (int)obstacles.size(), 0.01, 30);

            fprintf(stderr, "%-8s cw=%.2f | %6d %8d %10.2f | %s\n",
                    rname, cw,
                    b.n_nodes, b.n_fk_calls, b.time_ms,
                    b.found ? "OK" : "FAIL");
        }
        fprintf(stderr, "\n");
    }

    printf("Hybrid IA/AA LECT test completed.\n");
    return 0;
}
