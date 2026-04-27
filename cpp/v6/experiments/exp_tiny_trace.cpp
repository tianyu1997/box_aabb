/**
 * exp_tiny_trace.cpp — Minimal SBF build with full TRACE logging.
 *
 * Forces:
 *   - SBF_LOG_LEVEL=5 (TRACE) regardless of env
 *   - Default log file under <SBF_LOG_DIR>/tiny_trace_<ts>.log unless
 *     SBF_LOG_FILE is set
 *   - Tiny budget (max_boxes ~= 30, single tree, single thread) so the
 *     trace is human-readable.
 *
 * Logs cover:
 *   - Each seed (multi-goal / endpoint / diversity FPS candidate scoring)
 *   - Every FFB internal step (occupied descend / leaf expand / collide
 *     descend / free return) with current node, depth, split_dim/val
 *   - Every successful RRT sample with nearest box / snap face / snap_seed
 *   - Boundary sample generation per box
 *   - Bridge candidate listing and chain-pave step decisions
 *
 * Usage:
 *   build/experiments/exp_tiny_trace [--scene combined|shelves|bins|table]
 *                                    [--max-boxes N]
 *                                    [--no-bridge]
 *                                    [--single-query]
 */

#include <sbf/planner/sbf_planner.h>
#include <sbf/forest/connectivity.h>
#include <sbf/core/robot.h>
#include <sbf/core/log.h>
#include "marcucci_scenes.h"

#include <chrono>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sys/stat.h>
#include <vector>

using namespace sbf;

static std::string default_log_path() {
    std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    char ts[32];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &tm);
    ::mkdir(SBF_LOG_DIR, 0755);
    char path[1024];
    std::snprintf(path, sizeof(path), "%s/tiny_trace_%s.log",
                  SBF_LOG_DIR, ts);
    return std::string(path);
}

int main(int argc, char** argv) {
    // Force TRACE level; respect SBF_LOG_FILE if user set it.
    sbf::set_log_level(sbf::LogLevel::TRACE);
    if (const char* f = std::getenv("SBF_LOG_FILE"); f && *f) {
        sbf::set_log_file(f);
        std::fprintf(stderr, "[exp_tiny_trace] log -> %s\n", f);
    } else {
        std::string p = default_log_path();
        sbf::set_log_file(p.c_str());
        std::fprintf(stderr, "[exp_tiny_trace] log -> %s\n", p.c_str());
    }

    std::string robot_path = std::string(SBF_DATA_DIR) + "/iiwa14.json";
    std::string scene_name = "combined";
    int max_boxes = 30;
    bool no_bridge = false;
    bool single_query = true;
    uint64_t rng_seed = 0;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--scene" && i + 1 < argc) scene_name = argv[++i];
        else if (a == "--max-boxes" && i + 1 < argc) max_boxes = std::atoi(argv[++i]);
        else if (a == "--no-bridge") no_bridge = true;
        else if (a == "--all-queries") single_query = false;
        else if (a == "--seed" && i + 1 < argc) rng_seed = std::strtoull(argv[++i], nullptr, 10);
        else if (a[0] != '-') robot_path = a;
    }

    Robot robot = Robot::from_json(robot_path);

    std::vector<Obstacle> obstacles;
    std::vector<QueryPair> queries;
    if (scene_name == "shelves")      { obstacles = make_shelves_obstacles(); queries = make_shelves_queries(); }
    else if (scene_name == "bins")    { obstacles = make_bins_obstacles();    queries = make_bins_queries(); }
    else if (scene_name == "table")   { obstacles = make_table_obstacles();   queries = make_table_queries(); }
    else                              { obstacles = make_combined_obstacles();queries = make_combined_queries(); }
    int n_obs = static_cast<int>(obstacles.size());

    // Seed points: just the first query (unless --all-queries) so the
    // trace is small and focused.
    std::vector<Eigen::VectorXd> seed_points;
    if (!queries.empty()) {
        seed_points.push_back(queries[0].start);
        seed_points.push_back(queries[0].goal);
        if (!single_query) {
            for (size_t i = 1; i < queries.size(); ++i) {
                seed_points.push_back(queries[i].start);
                seed_points.push_back(queries[i].goal);
            }
        }
    }

    SBF_INFO("[TINY] robot=%s dof=%d scene=%s n_obs=%d max_boxes=%d "
             "n_seed_points=%zu rng_seed=%llu",
             robot.name().c_str(), robot.n_joints(),
             scene_name.c_str(), n_obs, max_boxes,
             seed_points.size(), (unsigned long long)rng_seed);

    SBFPlannerConfig cfg;
    cfg.z4_enabled = true;
    cfg.split_order = SplitOrder::BEST_TIGHTEN;
    cfg.lect_no_cache = false;  // keep cache so we see hit/miss patterns

    cfg.grower.mode = GrowerConfig::Mode::RRT;
    cfg.grower.max_boxes = max_boxes;
    cfg.grower.timeout_ms = 30000.0;
    cfg.grower.n_threads = 1;             // serial, deterministic trace
    cfg.grower.bridge_n_threads = 1;
    cfg.grower.rng_seed = rng_seed;
    cfg.grower.max_consecutive_miss = 200;
    cfg.grower.rrt_goal_bias = 0.1;
    cfg.grower.rrt_step_ratio = 0.05;
    cfg.grower.connect_mode = !no_bridge;
    cfg.grower.enable_promotion = false;
    cfg.grower.post_connect_extra_boxes = 0;
    cfg.grower.ffb_config.max_depth = 100;

    cfg.coarsen.target_boxes = max_boxes;  // skip coarsen
    cfg.coarsen.max_rounds = 0;

    SBFPlanner planner(robot, cfg);

    auto t0 = std::chrono::steady_clock::now();
    planner.build_coverage(obstacles.data(), n_obs, cfg.grower.timeout_ms,
                           seed_points);
    double total_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - t0).count();

    int n_boxes = planner.n_boxes();
    const auto& adj = planner.adjacency();
    int n_edges = 0;
    for (auto& kv : adj) n_edges += static_cast<int>(kv.second.size());
    n_edges /= 2;
    auto islands = find_islands(adj);

    SBF_INFO("[TINY] DONE total=%.2fs boxes=%d edges=%d islands=%zu",
             total_s, n_boxes, n_edges, islands.size());

    std::cout << "tiny_trace done. boxes=" << n_boxes
              << " edges=" << n_edges
              << " islands=" << islands.size()
              << " total=" << total_s << "s\n";
    std::cout << "  see log under " << SBF_LOG_DIR << "/\n";
    return 0;
}
