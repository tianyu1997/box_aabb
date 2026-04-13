/**
 * exp_rrt_path.cpp — RRT path feasibility check + path-guided box expansion
 *
 * 1. Point-RRT from TS→CS to check if a free path exists
 * 2. If found, expand boxes along the path using FFB
 * 3. Report path length, collision-check stats, and box coverage along path
 */

#include <sbf/core/robot.h>
#include <sbf/core/types.h>
#include <sbf/ffb/ffb.h>
#include <sbf/forest/grower.h>
#include <sbf/forest/adjacency.h>
#include <sbf/forest/connectivity.h>
#include <sbf/lect/lect.h>
#include <sbf/lect/lect_io.h>
#include <sbf/lect/lect_cache_manager.h>
#include <sbf/scene/collision_checker.h>
#include "marcucci_scenes.h"

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

using namespace sbf;
using Clock = std::chrono::steady_clock;

// ═════════════════════════════════════════════════════════════════════════════
// Simple point-RRT in C-space with AABB collision checking
// ═════════════════════════════════════════════════════════════════════════════

struct RRTNode {
    Eigen::VectorXd q;
    int parent = -1;
};

struct RRTResult {
    bool found = false;
    std::vector<Eigen::VectorXd> path;   // start → goal waypoints
    int n_nodes = 0;
    int n_collision_checks = 0;
    double time_ms = 0.0;
};

/// Collision-check a straight-line segment with specified resolution.
static bool segment_free(const CollisionChecker& cc,
                         const Eigen::VectorXd& a,
                         const Eigen::VectorXd& b,
                         int n_checks, int& total_checks) {
    for (int i = 0; i <= n_checks; ++i) {
        double t = (n_checks > 0) ? (double)i / n_checks : 0.0;
        Eigen::VectorXd q = (1.0 - t) * a + t * b;
        total_checks++;
        if (cc.check_config(q)) return false;  // in collision
    }
    return true;
}

/// Standard bi-directional RRT-Connect between start and goal.
static RRTResult rrt_connect(const CollisionChecker& cc,
                             const Eigen::VectorXd& start,
                             const Eigen::VectorXd& goal,
                             const JointLimits& limits,
                             int max_iters = 500000,
                             double step_size = 0.2,
                             double goal_bias = 0.05,
                             int seg_resolution = 20,
                             double timeout_s = 60.0) {
    auto t0 = Clock::now();
    const int nd = start.size();
    RRTResult result;

    // Forward tree (from start) and backward tree (from goal)
    std::vector<RRTNode> tree_fwd, tree_bwd;
    tree_fwd.push_back({start, -1});
    tree_bwd.push_back({goal, -1});

    std::mt19937_64 rng(42);
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    auto sample_random = [&]() -> Eigen::VectorXd {
        Eigen::VectorXd q(nd);
        for (int d = 0; d < nd; ++d) {
            double lo = limits.limits[d].lo;
            double hi = limits.limits[d].hi;
            q[d] = lo + u01(rng) * (hi - lo);
        }
        return q;
    };

    auto nearest = [&](const std::vector<RRTNode>& tree,
                       const Eigen::VectorXd& q) -> int {
        double best = std::numeric_limits<double>::max();
        int idx = 0;
        for (int i = 0; i < (int)tree.size(); ++i) {
            double d = (tree[i].q - q).squaredNorm();
            if (d < best) { best = d; idx = i; }
        }
        return idx;
    };

    // Try to extend tree toward q_target. Returns index of new/nearest node,
    // or -1 if blocked.
    auto extend = [&](std::vector<RRTNode>& tree,
                      const Eigen::VectorXd& q_target) -> int {
        int ni = nearest(tree, q_target);
        Eigen::VectorXd dir = q_target - tree[ni].q;
        double dist = dir.norm();
        if (dist < 1e-12) return ni;

        // Step toward target
        Eigen::VectorXd q_new;
        if (dist <= step_size) {
            q_new = q_target;
        } else {
            q_new = tree[ni].q + dir * (step_size / dist);
        }

        // Clamp to limits
        for (int d = 0; d < nd; ++d)
            q_new[d] = std::clamp(q_new[d], limits.limits[d].lo, limits.limits[d].hi);

        // Check segment
        if (!segment_free(cc, tree[ni].q, q_new, seg_resolution,
                          result.n_collision_checks))
            return -1;

        tree.push_back({q_new, ni});
        return (int)tree.size() - 1;
    };

    // Try to connect tree to q_target (greedy extend until blocked or reached).
    auto connect = [&](std::vector<RRTNode>& tree,
                       const Eigen::VectorXd& q_target) -> int {
        int last = -1;
        for (int step = 0; step < 200; ++step) {
            int ni = extend(tree, q_target);
            if (ni < 0) return last;
            last = ni;
            if ((tree[ni].q - q_target).norm() < 1e-6) return ni;  // reached
        }
        return last;
    };

    // Extract path from tree root to node idx
    auto extract_path = [](const std::vector<RRTNode>& tree, int idx) {
        std::vector<Eigen::VectorXd> path;
        while (idx >= 0) {
            path.push_back(tree[idx].q);
            idx = tree[idx].parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    };

    bool swap = false;  // alternate which tree extends vs connects
    for (int iter = 0; iter < max_iters; ++iter) {
        double elapsed = std::chrono::duration<double>(Clock::now() - t0).count();
        if (elapsed > timeout_s) break;

        // Sample
        Eigen::VectorXd q_rand;
        if (u01(rng) < goal_bias) {
            q_rand = swap ? start : goal;
        } else {
            q_rand = sample_random();
        }

        auto& tree_a = swap ? tree_bwd : tree_fwd;
        auto& tree_b = swap ? tree_fwd : tree_bwd;

        // Extend tree_a toward random sample
        int new_a = extend(tree_a, q_rand);
        if (new_a >= 0) {
            // Try to connect tree_b to the new node in tree_a
            int new_b = connect(tree_b, tree_a[new_a].q);
            if (new_b >= 0 &&
                (tree_b[new_b].q - tree_a[new_a].q).norm() < 1e-6) {
                // Connected! Extract path
                auto path_a = extract_path(tree_a, new_a);
                auto path_b = extract_path(tree_b, new_b);
                // path_a: start/goal → new node
                // path_b: other start/goal → same node (reversed)
                std::reverse(path_b.begin(), path_b.end());

                if (swap) {
                    // tree_a=bwd, tree_b=fwd
                    // path_a: goal→node (reverse it), path_b: start→node (already fwd)
                    // Full: path_b (fwd, start→node) + path_a reversed (node→goal)
                    result.path = path_b;
                    // path_a goes goal→new_a, reversed = new_a→goal
                    // But we already reversed path_b... let me re-think:
                    // path_a = extract(tree_bwd, new_a) = goal ... new_a
                    // path_b = extract(tree_fwd, new_b) = start ... new_b
                    // reversed path_b = new_b ... start
                    // We want: start → ... → new → ... → goal
                    // = path_b_original + path_a_reversed
                    auto fwd_path = extract_path(tree_fwd, new_b);
                    auto bwd_path = extract_path(tree_bwd, new_a);
                    std::reverse(bwd_path.begin(), bwd_path.end());
                    result.path = fwd_path;
                    result.path.insert(result.path.end(),
                                       bwd_path.begin() + 1, bwd_path.end());
                } else {
                    // tree_a=fwd, tree_b=bwd
                    auto fwd_path = extract_path(tree_fwd, new_a);
                    auto bwd_path = extract_path(tree_bwd, new_b);
                    std::reverse(bwd_path.begin(), bwd_path.end());
                    result.path = fwd_path;
                    result.path.insert(result.path.end(),
                                       bwd_path.begin() + 1, bwd_path.end());
                }

                result.found = true;
                result.n_nodes = (int)tree_fwd.size() + (int)tree_bwd.size();
                result.time_ms = std::chrono::duration<double, std::milli>(
                    Clock::now() - t0).count();
                return result;
            }
        }
        swap = !swap;
    }

    result.n_nodes = (int)tree_fwd.size() + (int)tree_bwd.size();
    result.time_ms = std::chrono::duration<double, std::milli>(
        Clock::now() - t0).count();
    return result;
}

/// Shorten path using greedy shortcutting.
static void shortcut_path(std::vector<Eigen::VectorXd>& path,
                          const CollisionChecker& cc,
                          int max_iters = 500,
                          int seg_resolution = 30) {
    if (path.size() < 3) return;
    std::mt19937_64 rng(123);
    int checks = 0;
    for (int iter = 0; iter < max_iters && path.size() > 2; ++iter) {
        int i = std::uniform_int_distribution<int>(0, (int)path.size() - 3)(rng);
        int j = std::uniform_int_distribution<int>(i + 2, (int)path.size() - 1)(rng);
        if (segment_free(cc, path[i], path[j], seg_resolution, checks)) {
            path.erase(path.begin() + i + 1, path.begin() + j);
        }
    }
}

/// Compute C-space path length (sum of Euclidean segment lengths).
static double path_length(const std::vector<Eigen::VectorXd>& path) {
    double len = 0.0;
    for (int i = 1; i < (int)path.size(); ++i)
        len += (path[i] - path[i - 1]).norm();
    return len;
}

// ═════════════════════════════════════════════════════════════════════════════
// Path-guided box expansion: place FFB boxes along the RRT path
// ═════════════════════════════════════════════════════════════════════════════

struct PathBoxResult {
    int n_boxes = 0;
    int n_ffb_calls = 0;
    int n_ffb_success = 0;
    double time_ms = 0.0;
    bool path_covered = false;  // all waypoints inside some box
    int n_connected_components = 0;
};

static PathBoxResult expand_boxes_along_path(
        const Robot& robot,
        LECT& lect,
        const std::vector<Eigen::VectorXd>& path,
        const Obstacle* obs, int n_obs,
        int ffb_depth = 80,
        double densify_step = 0.05) {
    auto t0 = Clock::now();
    PathBoxResult result;

    // 1. Densify the path: resample so consecutive waypoints are at most
    //    densify_step apart in C-space.
    std::vector<Eigen::VectorXd> dense_path;
    for (int i = 0; i < (int)path.size(); ++i) {
        if (i == 0) { dense_path.push_back(path[0]); continue; }
        double seg_len = (path[i] - path[i - 1]).norm();
        int n_sub = std::max(1, (int)std::ceil(seg_len / densify_step));
        for (int s = 1; s <= n_sub; ++s) {
            double t = (double)s / n_sub;
            dense_path.push_back((1.0 - t) * path[i - 1] + t * path[i]);
        }
    }

    fprintf(stderr, "[PATH] densified: %d → %d waypoints (step=%.3f)\n",
            (int)path.size(), (int)dense_path.size(), densify_step);

    // 2. Expand FFB box at each waypoint
    FFBConfig fcfg;
    fcfg.max_depth = ffb_depth;

    std::vector<BoxNode> boxes;
    int n_covered = 0;

    for (int wi = 0; wi < (int)dense_path.size(); ++wi) {
        const auto& seed = dense_path[wi];

        // Check if already inside an existing box
        bool inside = false;
        for (const auto& b : boxes) {
            if (b.contains(seed)) { inside = true; break; }
        }
        if (inside) { n_covered++; continue; }

        // FFB
        result.n_ffb_calls++;
        auto ffb = find_free_box(lect, seed, obs, n_obs, fcfg);
        if (!ffb.success()) continue;
        if (lect.is_occupied(ffb.node_idx)) continue;

        BoxNode box;
        box.joint_intervals = lect.node_intervals(ffb.node_idx);
        box.seed_config = seed;
        box.tree_id = ffb.node_idx;
        box.id = (int)boxes.size();
        box.root_id = 0;
        box.compute_volume();
        boxes.push_back(std::move(box));
        result.n_ffb_success++;

        lect.mark_occupied(ffb.node_idx, 0);

        // Re-check coverage
        if (boxes.back().contains(seed)) n_covered++;
    }

    // 3. Check connectivity
    auto adj = compute_adjacency(boxes);
    auto islands = find_islands(adj);
    result.n_connected_components = (int)islands.size();

    // 4. Check how many waypoints are covered
    int total_covered = 0;
    for (const auto& wp : dense_path) {
        for (const auto& b : boxes) {
            if (b.contains(wp)) { total_covered++; break; }
        }
    }
    result.path_covered = (total_covered == (int)dense_path.size());

    result.n_boxes = (int)boxes.size();
    result.time_ms = std::chrono::duration<double, std::milli>(
        Clock::now() - t0).count();

    fprintf(stderr, "[PATH] boxes=%d  ffb_calls=%d  ffb_success=%d  "
            "covered=%d/%d  islands=%d  time=%.1fms\n",
            result.n_boxes, result.n_ffb_calls, result.n_ffb_success,
            total_covered, (int)dense_path.size(),
            result.n_connected_components, result.time_ms);

    // Print box volume stats
    if (!boxes.empty()) {
        double sum_vol = 0, min_vol = 1e30, max_vol = 0;
        for (const auto& b : boxes) {
            sum_vol += b.volume;
            min_vol = std::min(min_vol, b.volume);
            max_vol = std::max(max_vol, b.volume);
        }
        fprintf(stderr, "[PATH] box_vol: min=%.2e  max=%.2e  avg=%.2e  total=%.2e\n",
                min_vol, max_vol, sum_vol / boxes.size(), sum_vol);
    }

    return result;
}

// ═════════════════════════════════════════════════════════════════════════════
// Main
// ═════════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    // ── Parse arguments ──
    double rrt_timeout_s = 60.0;
    int    rrt_max_iters = 2000000;
    double rrt_step_size = 0.2;
    int    ffb_depth     = 80;
    double densify_step  = 0.03;
    int    pair_idx      = -1;  // -1 = all 10 pairs

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--rrt-timeout" && i+1 < argc) rrt_timeout_s = std::atof(argv[++i]);
        else if (a == "--rrt-step" && i+1 < argc) rrt_step_size = std::atof(argv[++i]);
        else if (a == "--ffb-depth" && i+1 < argc) ffb_depth = std::atoi(argv[++i]);
        else if (a == "--densify-step" && i+1 < argc) densify_step = std::atof(argv[++i]);
        else if (a == "--pair" && i+1 < argc) pair_idx = std::atoi(argv[++i]);
    }

    // ── Setup ──
    std::string robot_path = std::string(SBF_DATA_DIR) + "/iiwa14.json";
    Robot robot = Robot::from_json(robot_path);
    {
        auto& lim = const_cast<JointLimits&>(robot.joint_limits());
        const double planning_limits[7][2] = {
            {-1.865488,  1.865691},
            {-0.100000,  1.086648},
            {-0.662656,  0.662338},
            {-2.094400, -0.371673},
            {-0.619251,  0.619534},
            {-1.095222,  1.257951},
            { 1.050209,  2.091190},
        };
        for (int j = 0; j < 7; ++j) {
            lim.limits[j].lo = planning_limits[j][0];
            lim.limits[j].hi = planning_limits[j][1];
        }
    }

    auto obstacles = make_combined_obstacles();
    const int n_obs = (int)obstacles.size();
    const Obstacle* obs = obstacles.data();
    CollisionChecker cc(robot, obstacles);

    // 5 seeds
    std::vector<Eigen::VectorXd> configs = {
        config_AS(), config_TS(), config_CS(), config_LB(), config_RB()
    };
    const char* names[] = {"AS", "TS", "CS", "LB", "RB"};

    // All 10 unique pairs
    struct Pair { int a, b; };
    std::vector<Pair> pairs;
    for (int i = 0; i < 5; ++i)
        for (int j = i + 1; j < 5; ++j)
            pairs.push_back({i, j});

    // ── Check each seed for collision ──
    fprintf(stderr, "╔══════════════════════════════════════════════╗\n");
    fprintf(stderr, "║  RRT Path Feasibility + Path-Guided FFB     ║\n");
    fprintf(stderr, "╚══════════════════════════════════════════════╝\n");
    fprintf(stderr, "  Robot: %s  DOF=%d\n", robot.name().c_str(), robot.n_joints());
    fprintf(stderr, "  rrt_step=%.3f  rrt_timeout=%.0fs  ffb_depth=%d  densify=%.3f\n\n",
            rrt_step_size, rrt_timeout_s, ffb_depth, densify_step);

    for (int i = 0; i < 5; ++i) {
        bool col = cc.check_config(configs[i]);
        fprintf(stderr, "  %s: %s\n", names[i], col ? "COLLISION" : "free");
    }
    fprintf(stderr, "\n");

    // ── RRT for each pair ──
    auto run_pair = [&](int pi) {
        int ai = pairs[pi].a, bi = pairs[pi].b;
        fprintf(stderr, "═══════════ %s → %s ═══════════\n", names[ai], names[bi]);

        // 1. Straight-line check
        {
            double dist = (configs[bi] - configs[ai]).norm();
            int checks = 0;
            bool straight = segment_free(cc, configs[ai], configs[bi], 100, checks);
            fprintf(stderr, "  straight-line: %s (dist=%.4f, checks=%d)\n",
                    straight ? "FREE" : "blocked", dist, checks);
            if (straight) {
                fprintf(stderr, "  → trivially connected!\n\n");
                return;
            }
        }

        // 2. Bi-RRT-Connect
        auto rrt = rrt_connect(cc, configs[ai], configs[bi],
                               robot.joint_limits(),
                               rrt_max_iters, rrt_step_size, 0.05,
                               20, rrt_timeout_s);

        if (!rrt.found) {
            fprintf(stderr, "  RRT: FAILED (%d nodes, %d checks, %.1fms)\n\n",
                    rrt.n_nodes, rrt.n_collision_checks, rrt.time_ms);
            return;
        }

        double raw_len = path_length(rrt.path);
        fprintf(stderr, "  RRT: FOUND path (%d waypoints, len=%.4f, %d nodes, "
                "%d checks, %.1fms)\n",
                (int)rrt.path.size(), raw_len, rrt.n_nodes,
                rrt.n_collision_checks, rrt.time_ms);

        // 3. Shortcut
        auto path = rrt.path;
        shortcut_path(path, cc, 1000, 30);
        double short_len = path_length(path);
        fprintf(stderr, "  Shortcut: %d waypoints, len=%.4f (%.0f%% of original)\n",
                (int)path.size(), short_len, 100.0 * short_len / raw_len);

        // 4. Path-guided box expansion
        // Build LECT for this expansion
        auto root_ivs = robot.joint_limits().limits;
        EndpointSourceConfig ep_cfg;
        ep_cfg.source = EndpointSource::CritSample;
        ep_cfg.n_samples_crit = 64;
        EnvelopeTypeConfig env_cfg;
        env_cfg.type = EnvelopeType::Hull16_Grid;
        env_cfg.n_subdivisions = 1;
        env_cfg.grid_config.voxel_delta = 0.04;
        LECT lect(robot, root_ivs, ep_cfg, env_cfg);
        lect.set_split_order(SplitOrder::BEST_TIGHTEN);

        // V6 cache
        std::string cache_dir = std::string(getenv("HOME") ? getenv("HOME") : "/tmp") + "/.sbf_cache";
        uint64_t fp = robot.fingerprint();
        LectCacheManager cache_mgr;
        if (cache_mgr.init(fp, robot.name(),
                           lect.n_active_links() * 2 * 6, cache_dir)) {
            lect.set_cache_manager(&cache_mgr);
        }

        auto pbr = expand_boxes_along_path(robot, lect, path, obs, n_obs,
                                           ffb_depth, densify_step);

        fprintf(stderr, "  Path boxes: %d boxes, %d islands, covered=%s\n\n",
                pbr.n_boxes, pbr.n_connected_components,
                pbr.path_covered ? "YES" : "NO");
    };

    if (pair_idx >= 0 && pair_idx < (int)pairs.size()) {
        run_pair(pair_idx);
    } else {
        for (int pi = 0; pi < (int)pairs.size(); ++pi) {
            run_pair(pi);
        }
    }

    fprintf(stderr, "Done.\n");
    return 0;
}
