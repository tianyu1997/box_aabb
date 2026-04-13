/**
 * exp2_e2e_planning.cpp — 实验 2: 端到端规划性能
 *
 * v5 pipeline: LECT → Wavefront → Coarsen → Dijkstra/GCS → Smooth
 *
 * 设计:
 *   合并场景 (16 obstacles), 5 query pairs
 *   两种模式:
 *     A) build_coverage → query: 先构建全覆盖 forest, 再逐个查询
 *     B) plan (one-shot): 对每个 pair 单独 build + query
 *   测量: build_time, query_time, path_length, path_smoothness, success_rate
 *
 * v5 优化:
 *   - BEST_TIGHTEN, Z4, LECT cache, wavefront, parallel grow
 *   - Dijkstra path search (default) 或 GCS-SOCP (--gcs)
 *   - Shortcut + moving-average path smoothing
 *
 * 用法:
 *   ./exp2_e2e_planning [--seeds N] [--threads N] [--gcs] [--one-shot] [--quick]
 */

#include <sbf/planner/sbf_planner.h>
#include <sbf/forest/adjacency.h>
#include <sbf/core/robot.h>
#include "marcucci_scenes.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace sbf;

// ═══════════════════════════════════════════════════════════════════════════
// Path quality metrics
// ═══════════════════════════════════════════════════════════════════════════

double path_length(const std::vector<Eigen::VectorXd>& path) {
    double len = 0;
    for (size_t i = 1; i < path.size(); ++i)
        len += (path[i] - path[i - 1]).norm();
    return len;
}

double path_smoothness(const std::vector<Eigen::VectorXd>& path) {
    if (path.size() < 3) return 0;
    double cost = 0;
    for (size_t i = 1; i + 1 < path.size(); ++i) {
        auto a = path[i + 1] - 2.0 * path[i] + path[i - 1];
        cost += a.norm();
    }
    return cost;
}

// ═══════════════════════════════════════════════════════════════════════════
// Statistics
// ═══════════════════════════════════════════════════════════════════════════

struct Stats { double median, q25, q75, mean; };

Stats compute_stats(std::vector<double>& d) {
    Stats s{};
    if (d.empty()) return s;
    std::sort(d.begin(), d.end());
    int n = static_cast<int>(d.size());
    s.median = d[n / 2]; s.q25 = d[n / 4]; s.q75 = d[3 * n / 4];
    s.mean = std::accumulate(d.begin(), d.end(), 0.0) / n;
    return s;
}

// ═══════════════════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    std::string robot_path = std::string(SBF_DATA_DIR) + "/iiwa14.json";
    int n_seeds = 10;
    int n_threads = static_cast<int>(std::thread::hardware_concurrency());
    double timeout_ms = 60000.0;
    int max_boxes = 25000;
    bool quick = false;
    bool use_gcs = false;
    bool one_shot = false;
    bool no_viz = false;
    std::string save_paths_file;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seeds" && i+1 < argc) n_seeds = std::atoi(argv[++i]);
        else if (a == "--threads" && i+1 < argc) n_threads = std::atoi(argv[++i]);
        else if (a == "--timeout" && i+1 < argc) timeout_ms = std::atof(argv[++i]);
        else if (a == "--max-boxes" && i+1 < argc) max_boxes = std::atoi(argv[++i]);
        else if (a == "--gcs") use_gcs = true;
        else if (a == "--one-shot") one_shot = true;
        else if (a == "--quick") quick = true;
        else if (a == "--no-viz") no_viz = true;
        else if (a == "--save-paths" && i+1 < argc) save_paths_file = argv[++i];
        else if (a[0] != '-') robot_path = a;
    }

    if (quick) { n_seeds = 3; timeout_ms = 30000.0; }
    if (n_threads < 1) n_threads = 1;

    Robot robot = Robot::from_json(robot_path);
    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints()
              << "  active_links=" << robot.n_active_links() << "\n\n";

    auto obstacles = make_combined_obstacles();
    auto queries = make_combined_queries();
    int n_obs = static_cast<int>(obstacles.size());

    std::cout << "Scene: combined  (" << n_obs << " obstacles, "
              << queries.size() << " pairs)\n"
              << "  mode=" << (one_shot ? "one-shot" : "build+query")
              << "  solver=" << (use_gcs ? "GCS" : "Dijkstra")
              << "  seeds=" << n_seeds
              << "  threads=" << n_threads << "\n"
              << std::string(72, '=') << "\n";
    std::cout << std::fixed;

    // Per-pair accumulators
    int n_pairs = static_cast<int>(queries.size());
    std::vector<double> all_build;
    std::vector<std::vector<double>> per_pair_query(n_pairs);
    std::vector<std::vector<double>> per_pair_len(n_pairs);
    std::vector<std::vector<double>> per_pair_smooth(n_pairs);
    std::vector<int> per_pair_success(n_pairs, 0);

    // Collected paths for JSON export
    struct PathRecord {
        int seed, pair_idx;
        std::string label;
        bool success;
        std::vector<Eigen::VectorXd> path;
        std::vector<int> box_sequence;
        double path_length;
    };
    std::vector<PathRecord> all_paths;

    // Boxes captured from last seed (for JSON export)
    std::vector<BoxNode> exported_boxes;
    AdjacencyGraph exported_adj;

    for (int seed = 0; seed < n_seeds; ++seed) {
        SBFPlannerConfig cfg;
        cfg.z4_enabled = true;
        cfg.split_order = SplitOrder::BEST_TIGHTEN;
        cfg.grower.mode = GrowerConfig::Mode::RRT;
        cfg.grower.max_boxes = max_boxes;
        cfg.grower.timeout_ms = timeout_ms;
        cfg.grower.n_threads = n_threads;
        cfg.grower.rng_seed = static_cast<uint64_t>(seed);
        cfg.grower.max_consecutive_miss = 2000;
        cfg.grower.rrt_goal_bias = 0.8;
        cfg.grower.rrt_step_ratio = 0.05;
        cfg.grower.connect_mode = true;
        cfg.grower.enable_promotion = true;

        cfg.coarsen.target_boxes = 300;
        cfg.coarsen.max_rounds = 100;
        cfg.coarsen.max_lect_fk_per_round = 10000;
        cfg.coarsen.score_threshold = 500.0;

        cfg.smoother.shortcut_max_iters = 100;
        cfg.smoother.smooth_window = 3;
        cfg.smoother.smooth_iters = 5;

        cfg.use_gcs = use_gcs;

        if (one_shot) {
            // ── One-shot mode: plan() for each pair independently ──
            std::cout << "\n  seed=" << seed << "  (one-shot mode)\n";

            for (int pi = 0; pi < n_pairs; ++pi) {
                auto& qp = queries[pi];
                SBFPlanner planner(robot, cfg);

                auto t0 = std::chrono::steady_clock::now();
                auto res = planner.plan(qp.start, qp.goal,
                                        obstacles.data(), n_obs, timeout_ms);
                double total_t = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t0).count();

                if (res.success) {
                    per_pair_query[pi].push_back(total_t);
                    per_pair_len[pi].push_back(res.path_length);
                    per_pair_smooth[pi].push_back(path_smoothness(res.path));
                    per_pair_success[pi]++;
                }

                std::cout << "    " << std::left << std::setw(8) << qp.label
                          << std::right
                          << "  t=" << std::setprecision(3) << total_t << "s"
                          << "  " << (res.success ? "OK  " : "FAIL")
                          << "  boxes=" << std::setw(5) << res.n_boxes
                          << "  len=" << std::setprecision(3) << res.path_length
                          << "  pts=" << res.path.size()
                          << "\n";

                all_paths.push_back({seed, pi, qp.label, res.success,
                                     res.path, res.box_sequence, res.path_length});
            }
        } else {
            // ── Build+query mode: build_coverage once → query each pair ──
            SBFPlanner planner(robot, cfg);

            // Collect unique query endpoints as seed points for multi-goal RRT
            std::vector<Eigen::VectorXd> seed_points;
            {
                auto approx_eq = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
                    return (a - b).squaredNorm() < 1e-8;
                };
                auto maybe_add = [&](const Eigen::VectorXd& q) {
                    for (const auto& s : seed_points)
                        if (approx_eq(s, q)) return;
                    seed_points.push_back(q);
                };
                for (const auto& qp : queries) {
                    maybe_add(qp.start);
                    maybe_add(qp.goal);
                }
            }

            auto t_build0 = std::chrono::steady_clock::now();
            planner.build_coverage(obstacles.data(), n_obs, timeout_ms, seed_points);
            double build_t = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_build0).count();
            all_build.push_back(build_t);

            std::cout << "\n  seed=" << seed
                      << "  build=" << std::setprecision(2) << build_t << "s"
                      << "  boxes=" << planner.n_boxes() << "\n";

            for (int pi = 0; pi < n_pairs; ++pi) {
                auto& qp = queries[pi];

                auto t0 = std::chrono::steady_clock::now();
                auto res = planner.query(qp.start, qp.goal,
                                         obstacles.data(), n_obs);
                double qt = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t0).count();

                if (res.success) {
                    per_pair_query[pi].push_back(qt);
                    per_pair_len[pi].push_back(res.path_length);
                    per_pair_smooth[pi].push_back(path_smoothness(res.path));
                    per_pair_success[pi]++;

                    // ── Path collision validation ──
                    // check_config returns true = IN COLLISION
                    CollisionChecker val_checker(robot, {});
                    val_checker.set_obstacles(obstacles.data(), n_obs);
                    int n_col_pts = 0, n_col_segs = 0;
                    for (size_t wi = 0; wi < res.path.size(); ++wi) {
                        if (val_checker.check_config(res.path[wi])) {
                            if (n_col_pts < 3)
                                fprintf(stderr, "[VAL] %s: waypoint %d IN COLLISION\n",
                                        qp.label.c_str(), (int)wi);
                            n_col_pts++;
                        }
                    }
                    for (size_t wi = 0; wi + 1 < res.path.size(); ++wi) {
                        double slen = (res.path[wi+1] - res.path[wi]).norm();
                        int vres = std::max(20, static_cast<int>(std::ceil(slen / 0.005)));
                        if (val_checker.check_segment(res.path[wi], res.path[wi+1], vres)) {
                            if (n_col_segs < 3)
                                fprintf(stderr, "[VAL] %s: segment %d->%d IN COLLISION\n",
                                        qp.label.c_str(), (int)wi, (int)(wi+1));
                            n_col_segs++;
                        }
                    }
                    if (n_col_pts > 0 || n_col_segs > 0)
                        fprintf(stderr, "[VAL] %s: %d/%d pts collide, %d/%d segs collide\n",
                                qp.label.c_str(), n_col_pts, (int)res.path.size(),
                                n_col_segs, (int)(res.path.size()-1));
                    else
                        fprintf(stderr, "[VAL] %s: path OK (0 collisions)\n",
                                qp.label.c_str());
                }

                std::cout << "    " << std::left << std::setw(8) << qp.label
                          << std::right
                          << "  q=" << std::setprecision(4) << qt << "s"
                          << "  " << (res.success ? "OK  " : "FAIL")
                          << "  len=" << std::setprecision(3) << res.path_length
                          << "  pts=" << res.path.size()
                          << "\n";

                all_paths.push_back({seed, pi, qp.label, res.success,
                                     res.path, res.box_sequence, res.path_length});
            }

            // Capture boxes AFTER all queries (includes bridge boxes)
            exported_boxes = planner.boxes();

            // Capture internal adjacency (includes bridge edges)
            exported_adj = planner.adjacency();
        }
    }

    // ═══════════════════════════════════════════════════════════════════════
    // Summary
    // ═══════════════════════════════════════════════════════════════════════
    std::cout << "\n" << std::string(72, '=')
              << "\n  Summary (" << (use_gcs ? "GCS" : "Dijkstra")
              << ", " << (one_shot ? "one-shot" : "build+query") << ")\n"
              << std::string(72, '=') << "\n";

    if (!all_build.empty()) {
        auto sb = compute_stats(all_build);
        std::cout << "  Build:   med=" << std::setprecision(3) << sb.median << "s"
                  << "  mean=" << sb.mean << "s\n\n";
    }

    for (int pi = 0; pi < n_pairs; ++pi) {
        double sr = n_seeds > 0 ? 100.0 * per_pair_success[pi] / n_seeds : 0;
        std::cout << "  " << std::left << std::setw(8) << queries[pi].label
                  << std::right
                  << "  SR=" << std::setprecision(0) << std::setw(4) << sr << "%";
        if (!per_pair_query[pi].empty()) {
            auto sq = compute_stats(per_pair_query[pi]);
            auto sl = compute_stats(per_pair_len[pi]);
            std::cout << "  t_med=" << std::setprecision(4) << sq.median << "s"
                      << "  len_med=" << std::setprecision(3) << sl.median;
        }
        std::cout << "\n";
    }

    std::cout << "\n  Exp 2 complete.\n";

    // ═══════════════════════════════════════════════════════════════════════
    // Auto output: create result/<date_params>/ subfolder, save JSON + summary
    // ═══════════════════════════════════════════════════════════════════════
    if (!all_paths.empty()) {
        namespace fs = std::filesystem;

        // Build timestamp + param tag
        auto t_now = std::time(nullptr);
        auto* lt = std::localtime(&t_now);
        char ts[32];
        std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", lt);

        std::string mode_str = one_shot ? "oneshot" : "bq";
        std::string graph_str = use_gcs ? "gcs" : "dijk";

        std::ostringstream dir_name;
        dir_name << ts << "_b" << max_boxes
                 << "_rrt_" << mode_str
                 << "_" << graph_str
                 << "_s" << n_seeds;

        // result/ lives alongside data/ under v5 source root
        std::string v5_root = std::string(SBF_DATA_DIR) + "/..";
        fs::path result_dir = fs::canonical(v5_root) / "result" / dir_name.str();
        fs::create_directories(result_dir);

        // ── Save paths JSON (with boxes, adjacency, obstacles, queries) ──
        fs::path json_path = result_dir / "paths.json";
        {
            using json = nlohmann::json;
            json root;
            root["robot"] = robot.name();
            root["dof"] = robot.n_joints();

            // Obstacles
            json j_obs = json::array();
            for (int oi = 0; oi < n_obs; ++oi) {
                const auto& o = obstacles[oi];
                j_obs.push_back({
                    {"lo", {o.bounds[0], o.bounds[1], o.bounds[2]}},
                    {"hi", {o.bounds[3], o.bounds[4], o.bounds[5]}}
                });
            }
            root["obstacles"] = j_obs;

            // Queries
            json j_queries = json::array();
            for (const auto& qp : queries) {
                json jq;
                jq["label"] = qp.label;
                jq["start"] = std::vector<double>(qp.start.data(), qp.start.data() + qp.start.size());
                jq["goal"]  = std::vector<double>(qp.goal.data(), qp.goal.data() + qp.goal.size());
                j_queries.push_back(jq);
            }
            root["queries"] = j_queries;

            // Boxes (from last seed's build)
            json j_boxes = json::array();
            for (const auto& b : exported_boxes) {
                json jb;
                jb["id"] = b.id;
                std::vector<double> lo(b.n_dims()), hi(b.n_dims());
                for (int d = 0; d < b.n_dims(); ++d) {
                    lo[d] = b.joint_intervals[d].lo;
                    hi[d] = b.joint_intervals[d].hi;
                }
                jb["lo"] = lo;
                jb["hi"] = hi;
                j_boxes.push_back(jb);
            }
            root["boxes"] = j_boxes;

            // Adjacency (recomputed from boxes)
            if (!exported_adj.empty()) {
                json j_adj = json::object();
                for (const auto& [k, nbrs] : exported_adj) {
                    j_adj[std::to_string(k)] = nbrs;
                }
                root["adjacency"] = j_adj;
            }

            // Paths
            json j_paths = json::array();
            for (const auto& pr : all_paths) {
                json jp;
                jp["seed"] = pr.seed;
                jp["pair_idx"] = pr.pair_idx;
                jp["label"] = pr.label;
                jp["success"] = pr.success;
                jp["path_length"] = pr.path_length;
                jp["box_sequence"] = pr.box_sequence;
                json wps = json::array();
                for (const auto& wp : pr.path) {
                    wps.push_back(std::vector<double>(wp.data(), wp.data() + wp.size()));
                }
                jp["waypoints"] = wps;
                j_paths.push_back(jp);
            }
            root["paths"] = j_paths;

            std::ofstream ofs(json_path);
            ofs << root.dump(2) << "\n";
            ofs.close();
        }

        // ── Also honour legacy --save-paths for backward compat ──
        if (!save_paths_file.empty()) {
            fs::copy_file(json_path, save_paths_file,
                          fs::copy_options::overwrite_existing);
        }

        std::cerr << "[EXP2] Result dir: " << result_dir.string() << "\n";
        std::cerr << "[EXP2] Saved " << all_paths.size() << " paths → "
                  << json_path.string() << "\n";

        // ── Auto-launch visualization ──
        if (!no_viz) {
            fs::path viz_script = fs::canonical(v5_root) / "viz" / "viz_drake_paths.py";
            fs::path html_out  = result_dir / "viz.html";

            std::ostringstream cmd;
            cmd << "python3 \"" << viz_script.string() << "\""
                << " \"" << json_path.string() << "\""
                << " --save \"" << html_out.string() << "\""
                << " --seed 0"
                << " &";

            std::cerr << "[EXP2] Launching viz: " << cmd.str() << "\n";
            int rc = std::system(cmd.str().c_str());
            if (rc != 0)
                std::cerr << "[EXP2] viz launch returned " << rc << "\n";
        }
    }

    return 0;
}
