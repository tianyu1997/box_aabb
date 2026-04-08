// ═══════════════════════════════════════════════════════════════════════════
//  Experiment: v3 vs v4 ForestGrower benchmark  — v3 side
//
//  Grows forests in random AABB scenes, measuring timing and quality.
//  Identical config/seed logic to the v4 side for fair comparison.
//
//  Build:
//    cmake --build build --config Release --target exp_v3v4_grower
//
//  Run:
//    build\Release\exp_v3v4_grower.exe > v3_grower_results.csv 2>v3_grower_log.txt
// ═══════════════════════════════════════════════════════════════════════════

#include "sbf/robot/robot.h"
#include "sbf/forest/forest_grower.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/common/types.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>
#include <string>
#include <vector>
#include <Eigen/Core>

using namespace sbf;
using namespace sbf::forest;
using namespace sbf::envelope;

// ─── Robot factories (identical to v4 side) ──────────────────────────────────

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

// ─── Random obstacle generator (identical to v4 side) ────────────────────────

struct ObstacleGenerator {
    std::mt19937& rng;

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
            if (std::sqrt(cx * cx + cy * cy) < BASE_CLEARANCE) continue;
            double hx = ds(rng) / 2.0, hy = ds(rng) / 2.0, hz = ds(rng) / 2.0;
            Obstacle o;
            o.center     = Eigen::Vector3d(cx, cy, cz);
            o.half_sizes = Eigen::Vector3d(hx, hy, hz);
            o.name       = "rand_" + std::to_string(attempt);
            obs.push_back(o);
        }
        return obs;
    }
};

// ─── Random config-space point ──────────────────────────────────────────────

static Eigen::VectorXd random_config(const Robot& robot, std::mt19937& rng) {
    int n = robot.n_joints();
    Eigen::VectorXd q(n);
    const auto& lim = robot.joint_limits().limits;
    for (int j = 0; j < n; ++j) {
        std::uniform_real_distribution<double> d(lim[j].lo, lim[j].hi);
        q[j] = d(rng);
    }
    return q;
}

// ─── Helper: extract phase time with fallback ───────────────────────────────

static double phase_ms(const GrowerResult& r, const std::string& key) {
    auto it = r.phase_times.find(key);
    return (it != r.phase_times.end()) ? it->second : 0.0;
}

// ─── Main ───────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    const int n_scenes     = (argc > 1) ? std::atoi(argv[1]) : 5;
    const int max_boxes    = (argc > 2) ? std::atoi(argv[2]) : 300;
    const double timeout_s = (argc > 3) ? std::atof(argv[3]) : 30.0;
    const int base_seed    = (argc > 4) ? std::atoi(argv[4]) : 42;

    const int obstacle_counts[] = {5, 10, 20};
    const int n_counts = 3;

    const char* mode_names[] = {"Wavefront", "RRT"};
    const GrowerConfig::Mode modes[] = {GrowerConfig::Mode::Wavefront,
                                         GrowerConfig::Mode::RRT};
    const int n_modes = 2;

    struct RobotEntry { const char* name; Robot robot; };
    RobotEntry robots[] = {
        {"iiwa14", make_iiwa14()},
        {"panda",  make_panda()},
    };
    const int n_robots = 2;

    auto pipeline = PipelineConfig::fast();

    // CSV header
    std::printf("version,robot,n_obstacles,mode,scene_seed,"
                "grow_time_ms,n_boxes,n_ffb_success,n_ffb_fail,"
                "n_components,start_goal_connected,total_volume,"
                "tree_size,phase_root_ms,phase_expand_ms,phase_adj_ms\n");

    int total = n_robots * n_counts * n_modes * n_scenes;
    int done = 0;

    for (int ri = 0; ri < n_robots; ++ri) {
        auto& re = robots[ri];

        for (int ci = 0; ci < n_counts; ++ci) {
            int n_obs = obstacle_counts[ci];

            for (int mi = 0; mi < n_modes; ++mi) {
                auto mode = modes[mi];

                for (int si = 0; si < n_scenes; ++si) {
                    int scene_seed = base_seed + n_obs * 1000 + si;
                    std::mt19937 obs_rng(scene_seed);

                    ObstacleGenerator gen{obs_rng};
                    auto obstacles = gen.generate(n_obs);
                    int actual_n_obs = (int)obstacles.size();

                    std::mt19937 ep_rng(scene_seed * 31 + ri + mi * 7);
                    Eigen::VectorXd start = random_config(re.robot, ep_rng);
                    Eigen::VectorXd goal  = random_config(re.robot, ep_rng);

                    GrowerConfig cfg;
                    cfg.mode              = mode;
                    cfg.max_boxes         = max_boxes;
                    cfg.min_edge          = 0.01;
                    cfg.max_depth         = 30;
                    cfg.max_consecutive_miss = 200;
                    cfg.timeout           = timeout_s;
                    cfg.rng_seed          = scene_seed;
                    cfg.pipeline          = pipeline;
                    cfg.n_roots           = 2;

                    ForestGrower grower(re.robot, cfg);
                    grower.set_endpoints(start, goal);

                    auto result = grower.grow(obstacles.data(), actual_n_obs);

                    int tree_size = grower.lect().n_nodes();

                    double p_root   = phase_ms(result, "root_select");
                    double p_expand = phase_ms(result, "expand");
                    if (p_expand == 0.0) p_expand = phase_ms(result, "wavefront");
                    if (p_expand == 0.0) p_expand = phase_ms(result, "rrt");
                    double p_adj    = phase_ms(result, "adjacency");

                    std::printf("v3,%s,%d,%s,%d,"
                                "%.1f,%d,%d,%d,"
                                "%d,%d,%.6f,"
                                "%d,%.1f,%.1f,%.1f\n",
                                re.name, n_obs, mode_names[mi], si,
                                result.build_time_ms,
                                result.n_boxes_total,
                                result.n_ffb_success,
                                result.n_ffb_fail,
                                result.n_components,
                                result.start_goal_connected ? 1 : 0,
                                result.total_volume,
                                tree_size,
                                p_root, p_expand, p_adj);

                    ++done;
                    std::fprintf(stderr,
                        "[v3] %d/%d %s %s n_obs=%d seed=%d: "
                        "%.0f ms, %d boxes, %d comp, sg=%d\n",
                        done, total, re.name, mode_names[mi],
                        n_obs, si,
                        result.build_time_ms,
                        result.n_boxes_total,
                        result.n_components,
                        result.start_goal_connected ? 1 : 0);
                }
            }
        }
    }

    std::fprintf(stderr, "[v3] All %d runs completed.\n", total);
    return 0;
}
