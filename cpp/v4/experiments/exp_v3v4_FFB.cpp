// ═══════════════════════════════════════════════════════════════════════════
//  Experiment: v3 vs v4 find_free_box (FFB) latency comparison  — v4 side
//
//  Generates random AABB obstacle scenes and measures wall-clock FFB time.
//
//  Config:
//    robots      = {iiwa14, panda}
//    n_obstacles  = {5, 10, 20}
//    scene_seeds  = 0..9   (10 random scenes per obstacle count)
//    queries/scene= 200    (tree accumulates across queries)
//    pipeline     = fast() = IFK + LinkIAABB(n_sub=1)
//    split_order  = WIDEST_FIRST (v4 default)
//
//  CSV (stdout):
//    version,robot,n_obstacles,scene_seed,query_idx,time_us,success,
//    n_new_nodes,n_fk_calls,tree_size
//
//  Build (Release):
//    cmake --build build --config Release --target exp_v3v4_FFB
//
//  Run:
//    build\Release\exp_v3v4_FFB.exe > v4_ffb_results.csv 2>v4_ffb_log.txt
// ═══════════════════════════════════════════════════════════════════════════

#include "sbf/robot/robot.h"
#include "sbf/forest/lect.h"
#include "sbf/envelope/pipeline.h"
#include "sbf/core/types.h"
#include "sbf/core/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>
#include <Eigen/Core>

using namespace sbf;
using namespace sbf::envelope;

// ─── Robot factories ─────────────────────────────────────────────────────────

static Robot make_iiwa14() {
    std::vector<DHParam> dh = {
        {  0.0,     0.0, 0.1575, 0.0, 0},
        { -HALF_PI, 0.0, 0.0,    0.0, 0},
        {  HALF_PI, 0.0, 0.2025, 0.0, 0},
        {  HALF_PI, 0.0, 0.0,    0.0, 0},
        { -HALF_PI, 0.0, 0.2155, 0.0, 0},
        { -HALF_PI, 0.0, 0.0,    0.0, 0},
        {  HALF_PI, 0.0, 0.081,  0.0, 0},
    };
    JointLimits limits;
    limits.limits = {
        {-1.865, 1.866}, {-0.100, 1.087}, {-0.663, 0.662},
        {-2.094,-0.372}, {-0.619, 0.620}, {-1.095, 1.258},
        { 1.050, 2.091},
    };
    DHParam tool{0.0, 0.0, 0.185, 0.0, 0};
    std::vector<double> radii = {0.08, 0.08, 0.06, 0.06, 0.04, 0.04, 0.04, 0.03};
    return Robot("iiwa14", dh, limits, tool, radii);
}

static Robot make_panda() {
    std::vector<DHParam> dh = {
        {  0.0,     0.0,     0.333,  0.0, 0},
        { -HALF_PI, 0.0,     0.0,    0.0, 0},
        {  HALF_PI, 0.0,     0.316,  0.0, 0},
        {  HALF_PI, 0.0825,  0.0,    0.0, 0},
        { -HALF_PI,-0.0825,  0.384,  0.0, 0},
        {  HALF_PI, 0.0,     0.0,    0.0, 0},
        {  HALF_PI, 0.088,   0.0,    0.0, 0},
    };
    JointLimits limits;
    limits.limits = {
        {-2.8973, 2.8973}, {-1.7628, 1.7628}, {-2.8973, 2.8973},
        {-3.0718,-0.0698}, {-2.8973, 2.8973}, {-0.0175, 3.7525},
        {-2.8973, 2.8973},
    };
    std::vector<double> radii = {0.08, 0.08, 0.06, 0.06, 0.04, 0.04, 0.04};
    return Robot("panda", dh, limits, std::nullopt, radii);
}

// ─── Random obstacle generator (deterministic, identical to v3 side) ────────

struct ObstacleGenerator {
    std::mt19937& rng;

    // Workspace bounds: [-0.5, -1.0, 0.0] to [1.5, 1.0, 1.2]
    static constexpr double WS_XMIN = -0.5, WS_XMAX = 1.5;
    static constexpr double WS_YMIN = -1.0, WS_YMAX = 1.0;
    static constexpr double WS_ZMIN =  0.0, WS_ZMAX = 1.2;
    static constexpr double MIN_SIZE = 0.04, MAX_SIZE = 0.12;
    static constexpr double BASE_CLEARANCE = 0.15;

    std::vector<Obstacle> generate(int n_obstacles) {
        std::vector<Obstacle> obs;
        obs.reserve(n_obstacles);

        std::uniform_real_distribution<double> dx(WS_XMIN, WS_XMAX);
        std::uniform_real_distribution<double> dy(WS_YMIN, WS_YMAX);
        std::uniform_real_distribution<double> dz(WS_ZMIN, WS_ZMAX);
        std::uniform_real_distribution<double> ds(MIN_SIZE, MAX_SIZE);

        for (int attempt = 0; attempt < n_obstacles * 3 && (int)obs.size() < n_obstacles; ++attempt) {
            double cx = dx(rng), cy = dy(rng), cz = dz(rng);

            // Skip obstacles too close to robot base (origin)
            if (std::sqrt(cx * cx + cy * cy) < BASE_CLEARANCE)
                continue;

            double hx = ds(rng) / 2.0;
            double hy = ds(rng) / 2.0;
            double hz = ds(rng) / 2.0;

            Obstacle o;
            o.center    = Eigen::Vector3d(cx, cy, cz);
            o.half_sizes = Eigen::Vector3d(hx, hy, hz);
            o.name       = "rand_" + std::to_string(attempt);
            obs.push_back(o);
        }
        return obs;
    }
};

// ─── Random seed (config-space point) generator ─────────────────────────────

static Eigen::VectorXd random_seed(const Robot& robot, std::mt19937& rng) {
    int n = robot.n_joints();
    Eigen::VectorXd q(n);
    const auto& lim = robot.joint_limits().limits;
    for (int j = 0; j < n; ++j) {
        std::uniform_real_distribution<double> d(lim[j].lo, lim[j].hi);
        q[j] = d(rng);
    }
    return q;
}

// ─── Main ───────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    // Optional overrides: n_scenes_per_count  n_queries_per_scene  base_seed
    const int n_scenes  = (argc > 1) ? std::atoi(argv[1]) : 10;
    const int n_queries = (argc > 2) ? std::atoi(argv[2]) : 200;
    const int base_seed = (argc > 3) ? std::atoi(argv[3]) : 42;

    const int obstacle_counts[] = {5, 10, 20};
    const int n_counts = 3;

    struct RobotEntry {
        const char* name;
        Robot robot;
    };

    RobotEntry robots[] = {
        {"iiwa14", make_iiwa14()},
        {"panda",  make_panda()},
    };
    const int n_robots = 2;

    // Pipeline: fast() = IFK + LinkIAABB(n_sub=1)
    auto pipeline = PipelineConfig::fast();

    // CSV header
    std::printf("version,robot,n_obstacles,scene_seed,query_idx,"
                "time_us,success,n_new_nodes,n_fk_calls,tree_size\n");

    int total_scenes = n_robots * n_counts * n_scenes;
    int scene_done = 0;

    for (int ri = 0; ri < n_robots; ++ri) {
        auto& re = robots[ri];

        for (int ci = 0; ci < n_counts; ++ci) {
            int n_obs = obstacle_counts[ci];

            for (int si = 0; si < n_scenes; ++si) {
                // Deterministic scene seed: combine base_seed, obstacle count, scene index
                int scene_seed = base_seed + n_obs * 1000 + si;
                std::mt19937 obs_rng(scene_seed);

                ObstacleGenerator gen{obs_rng};
                auto obstacles = gen.generate(n_obs);
                int actual_n_obs = (int)obstacles.size();

                // Fresh LECT per scene, tree accumulates across queries
                forest::LECT lect(re.robot, pipeline);
                lect.set_scene(obstacles.data(), actual_n_obs);

                // Query-level RNG: deterministic per (scene_seed, robot)
                std::mt19937 q_rng(scene_seed * 31 + ri);

                for (int qi = 0; qi < n_queries; ++qi) {
                    Eigen::VectorXd seed = random_seed(re.robot, q_rng);

                    auto t0 = std::chrono::high_resolution_clock::now();
                    auto result = lect.find_free_box(seed, obstacles.data(),
                                                     actual_n_obs);
                    auto t1 = std::chrono::high_resolution_clock::now();

                    double time_us = std::chrono::duration<double, std::micro>(
                                         t1 - t0).count();

                    std::printf("v4,%s,%d,%d,%d,%.1f,%d,%d,%d,%d\n",
                                re.name, n_obs, si, qi,
                                time_us,
                                result.success() ? 1 : 0,
                                result.n_new_nodes,
                                result.n_fk_calls,
                                lect.n_nodes());

                    if ((qi + 1) % 50 == 0) {
                        std::fprintf(stderr,
                            "[v4] %s n_obs=%d scene=%d/%d query=%d/%d  "
                            "tree=%d nodes\n",
                            re.name, n_obs, si + 1, n_scenes,
                            qi + 1, n_queries, lect.n_nodes());
                    }
                }

                ++scene_done;
                std::fprintf(stderr,
                    "[v4] Scene %d/%d done (%s, %d obs, seed=%d, "
                    "final tree=%d nodes)\n",
                    scene_done, total_scenes, re.name, n_obs,
                    si, lect.n_nodes());
            }
        }
    }

    std::fprintf(stderr, "[v4] All %d scenes completed.\n", total_scenes);
    return 0;
}
