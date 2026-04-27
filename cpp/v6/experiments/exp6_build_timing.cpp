/**
 * exp6_build_timing.cpp — 实验 6: 离线构建阶段计时分解
 *
 * 对应论文 Tab 2 (Offline Build Performance).
 *
 * 详细分解 build_coverage 各阶段耗时:
 *   - LECT load/init
 *   - Multi-tree grow (FFB + RRT expansion)
 *   - Coarsen (sweep + greedy + cluster)
 *   - Bridge (parallel RRT bridging)
 *   - Seed-point bridge
 *   - Coverage adjacency
 *
 * 同时记录各阶段的 box 数量变化, 形成完整的 pipeline profile.
 *
 * Connectivity metric contract:
 *   - Canonical connected metric: Grower UF (`all_connected`).
 *   - Adjacency islands are diagnostic only.
 *
 * 用法:
 *   ./exp6_build_timing [--seeds N] [--threads N] [--quick]
 */

#include <sbf/planner/sbf_planner.h>
#include <sbf/forest/connectivity.h>
#include <sbf/core/robot.h>
#include <sbf/core/log.h>
#include "marcucci_scenes.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <regex>
#include <string>
#include <sstream>
#include <thread>
#include <vector>

using namespace sbf;

// ═══════════════════════════════════════════════════════════════════════════

struct Stats { double median, mean, q25, q75; };

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
    sbf::init_log_from_env();
    std::string robot_path = std::string(SBF_DATA_DIR) + "/iiwa14.json";
    int n_seeds = 5;
    int n_threads = static_cast<int>(std::thread::hardware_concurrency());
    bool quick = false;
    std::string scene_name = "combined";
    std::string json_out;
    std::string endpoint_str = "ifk";   // ifk | critsample
    std::string envelope_str = "linkiaabb"; // linkiaabb | hull16_grid
    bool use_lect_cache = false;            // --lect-cache enables persistent mmap cache
    bool force_bridge = false;              // --force-bridge: exhaustive RRT-then-FFB to merge all islands
    bool z4_enabled = true;                 // --z4-off disables Z4 symmetry cache (R3-D5 ablation)
    bool use_unexplored = true;
    bool use_coordinated_grower = true;
    bool use_seed_bridge = true;
    bool use_rescue_bridge = true;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seeds" && i + 1 < argc) n_seeds = std::atoi(argv[++i]);
        else if (a == "--threads" && i + 1 < argc) n_threads = std::atoi(argv[++i]);
        else if (a == "--scene" && i + 1 < argc) scene_name = argv[++i];
        else if (a == "--json" && i + 1 < argc) json_out = argv[++i];
        else if (a == "--endpoint" && i + 1 < argc) endpoint_str = argv[++i];
        else if (a == "--envelope" && i + 1 < argc) envelope_str = argv[++i];
        else if (a == "--lect-cache") use_lect_cache = true;
        else if (a == "--force-bridge") force_bridge = true;
        else if (a == "--z4-off") z4_enabled = false;
        else if (a == "--no-unexplored") use_unexplored = false;
        else if (a == "--no-coordinated-grower") use_coordinated_grower = false;
        else if (a == "--no-seed-bridge") use_seed_bridge = false;
        else if (a == "--no-rescue-bridge") use_rescue_bridge = false;
        else if (a == "--quick") quick = true;
        else if (a[0] != '-') robot_path = a;
    }

    if (quick) { n_seeds = 2; }
    if (n_threads < 1) n_threads = 1;

    EndpointSource ep_src = EndpointSource::IFK;
    if (endpoint_str == "critsample" || endpoint_str == "crit")
        ep_src = EndpointSource::CritSample;
    else if (endpoint_str == "analytical")
        ep_src = EndpointSource::Analytical;

    EnvelopeType env_type = EnvelopeType::LinkIAABB;
    if (envelope_str == "hull16_grid" || envelope_str == "hull16")
        env_type = EnvelopeType::Hull16_Grid;
    else if (envelope_str == "linkiaabb_grid" || envelope_str == "grid")
        env_type = EnvelopeType::LinkIAABB_Grid;

    Robot robot = Robot::from_json(robot_path);

    // Scene selection
    std::vector<Obstacle> obstacles;
    std::vector<QueryPair> queries;
    if (scene_name == "shelves") {
        obstacles = make_shelves_obstacles();
        queries   = make_shelves_queries();
    } else if (scene_name == "bins") {
        obstacles = make_bins_obstacles();
        queries   = make_bins_queries();
    } else if (scene_name == "table") {
        obstacles = make_table_obstacles();
        queries   = make_table_queries();
    } else {
        obstacles = make_combined_obstacles();
        queries   = make_combined_queries();
    }
    int n_obs = static_cast<int>(obstacles.size());

    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints() << "\n"
              << "Scene: " << scene_name << " (" << n_obs << " obs)\n"
              << "Endpoint: " << endpoint_source_name(ep_src)
              << "  Envelope: " << envelope_type_name(env_type) << "\n"
              << "Seeds=" << n_seeds << "  Threads=" << n_threads << "\n"
              << "Ablations: unexplored=" << (use_unexplored ? "on" : "off")
              << " coordinated=" << (use_coordinated_grower ? "on" : "off")
              << " seed_bridge=" << (use_seed_bridge ? "on" : "off")
              << " rescue_bridge=" << (use_rescue_bridge ? "on" : "off")
              << "\n\n";

    // Seed points
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

    std::cout << std::string(90, '=') << "\n"
              << "  Build Timing Breakdown (Paper Tab 2)\n"
              << std::string(90, '=') << "\n\n";

    // Per-seed detailed results
    struct SeedResult {
        double total_s;
        BuildTimingProfile timing;
        int n_boxes_final;
        int n_islands;
        int n_edges;
        bool connected;
        int query_success;
        double query_mean_s;
        double query_mean_length;
    };
    std::vector<SeedResult> results;

    for (int seed = 0; seed < n_seeds; ++seed) {
        SBFPlannerConfig cfg;
        cfg.z4_enabled = z4_enabled;
        cfg.split_order = SplitOrder::BEST_TIGHTEN;
        cfg.lect_no_cache = !use_lect_cache;

        cfg.grower.mode = GrowerConfig::Mode::RRT;
        cfg.grower.max_boxes = 200000;
        cfg.grower.timeout_ms = 60000.0;
        cfg.grower.n_threads = 5;
        cfg.grower.rng_seed = static_cast<uint64_t>(seed);
        cfg.grower.max_consecutive_miss = 2000;
        cfg.grower.rrt_goal_bias = 0.1;
        cfg.grower.rrt_step_ratio = 0.05;
        cfg.grower.connect_mode = true;
        cfg.grower.enable_coordinated_multi_goal = use_coordinated_grower;
        cfg.grower.unexplored_sample_prob = use_unexplored ? 0.7 : 0.0;
        cfg.grower.enable_promotion = true;
        cfg.grower.post_connect_extra_boxes = 4000;
        cfg.grower.ffb_config.max_depth = 300;

        cfg.coarsen.target_boxes = 300;
        cfg.coarsen.max_rounds = 100;
        cfg.coarsen.max_lect_fk_per_round = 10000;
        cfg.coarsen.score_threshold = 500.0;
        cfg.grower.bridge_n_threads = n_threads;
        cfg.enable_seed_bridge = use_seed_bridge;
        cfg.enable_rescue_bridge = use_rescue_bridge;
        cfg.force_full_bridge = force_bridge;

        // Apply endpoint / envelope choice from CLI.
        cfg.endpoint_source.source = ep_src;
        cfg.envelope_type.type     = env_type;

        // coarsen + adjacency: defaults

        SBFPlanner planner(robot, cfg);

        std::cout << "  seed=" << seed << "\n";

        auto t0 = std::chrono::steady_clock::now();
        planner.build_coverage(obstacles.data(), n_obs, cfg.grower.timeout_ms,
                               seed_points);
        double total_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();

        int n_boxes = planner.n_boxes();
        const auto& adj = planner.adjacency();
        auto islands = find_islands(adj);
        int n_islands = static_cast<int>(islands.size());
        int n_edges = 0;
        for (auto& kv : adj) n_edges += static_cast<int>(kv.second.size());
        n_edges /= 2;

        // Diagnostic: tolerance sweep to test whether island split is caused
        // by an overly strict adjacency tolerance.
        auto adj_t1 = compute_adjacency(planner.boxes(), 1e-6);
        auto adj_t2 = compute_adjacency(planner.boxes(), 1e-3);
        auto adj_t3 = compute_adjacency(planner.boxes(), 1e-3, 0, 0.05);
        auto adj_t4 = compute_adjacency(planner.boxes(), 1e-3, 0, 0.10);
        int islands_t1 = static_cast<int>(find_islands(adj_t1).size());
        int islands_t2 = static_cast<int>(find_islands(adj_t2).size());
        int islands_t3 = static_cast<int>(find_islands(adj_t3).size());
        int islands_t4 = static_cast<int>(find_islands(adj_t4).size());

        auto& bt = planner.build_timing();
        std::cout << "    total=" << std::fixed << std::setprecision(2) << total_s << "s"
                  << "  lect=" << std::setprecision(0) << bt.lect_ms << "ms"
                  << "  grow=" << bt.grow_ms << "ms"
                  << "  coarsen1=" << bt.coarsen1_ms << "ms"
                  << "  bridge=" << bt.bridge_ms << "ms"
                  << "  coarsen2=" << bt.coarsen2_ms << "ms"
                  << "  adj=" << bt.adjacency_ms << "ms"
                  << " (pre=" << bt.adjacency_pre_seed_ms
                  << ", seed_bridge=" << bt.seed_bridge_ms
                  << ", final=" << bt.adjacency_final_ms << ")"
                  << "  boxes=" << n_boxes
                  << "  islands=" << n_islands
                  << "  edges=" << n_edges << "\n";
            std::cout << "    islands: strict=" << islands_t1
                  << " tol=1e-3=" << islands_t2
                  << " gap=0.05=" << islands_t3
                  << " gap=0.10=" << islands_t4
                  << "\n";

        // Diagnostic: find minimum inter-island gap
        {
            auto strict_adj = compute_adjacency(planner.boxes(), 1e-6);
            auto islands_vec = find_islands(strict_adj);
            if (islands_vec.size() > 1) {
                const auto& bx = planner.boxes();
                // Build box_id -> island_id map
                std::unordered_map<int, int> box_island;
                for (int ii = 0; ii < (int)islands_vec.size(); ++ii)
                    for (int bid : islands_vec[ii])
                        box_island[bid] = ii;
                // Build box_id -> box index map
                std::unordered_map<int, int> id_to_idx;
                for (int ii = 0; ii < (int)bx.size(); ++ii)
                    id_to_idx[bx[ii].id] = ii;
                // For each island pair, find min maximum-dim-gap
                int n_isl = (int)islands_vec.size();
                int nd_loc = bx[0].n_dims();
                for (int ia = 0; ia < n_isl; ++ia) {
                    for (int ib = ia + 1; ib < n_isl; ++ib) {
                        double min_max_gap = 1e9;
                        int best_n_gap_dims = 0;
                        // Sample: check up to 500 pairs per island pair
                        int checked = 0;
                        for (int ai : islands_vec[ia]) {
                            auto ita = id_to_idx.find(ai);
                            if (ita == id_to_idx.end()) continue;
                            const auto& ba = bx[ita->second];
                            for (int bi : islands_vec[ib]) {
                                auto itb = id_to_idx.find(bi);
                                if (itb == id_to_idx.end()) continue;
                                const auto& bb = bx[itb->second];
                                double max_gap = 0;
                                int n_gap_dims = 0;
                                for (int d = 0; d < nd_loc; ++d) {
                                    double gap = std::max(ba.joint_intervals[d].lo, bb.joint_intervals[d].lo)
                                               - std::min(ba.joint_intervals[d].hi, bb.joint_intervals[d].hi);
                                    if (gap > 1e-6) { max_gap = std::max(max_gap, gap); n_gap_dims++; }
                                }
                                if (max_gap < min_max_gap) {
                                    min_max_gap = max_gap;
                                    best_n_gap_dims = n_gap_dims;
                                }
                                if (++checked > 2000) break; // limit
                            }
                            if (checked > 2000) break;
                        }
                        if (min_max_gap < 1e8) {
                            std::cout << "    gap[" << ia << "-" << ib << "]: max_dim_gap="
                                  << std::scientific << std::setprecision(3) << min_max_gap
                                  << " (" << best_n_gap_dims << " dims)"
                                  << " isl_sizes=" << islands_vec[ia].size() << "+" << islands_vec[ib].size()
                                  << "\n";
                        }
                    }
                }
            }
        }

        // ── Seed-point coverage diagnostic ─────────────────────────────────
        // For every seed_point, report which island it lands in.  If the
        // point is not contained in any box, that's a hard build failure —
        // the query side will have no anchor to attach RRT/proxy to.
        {
            const auto& bx = planner.boxes();
            std::unordered_map<int, int> box_island;
            {
                int ii = 0;
                for (const auto& isl : islands) {
                    for (int bid : isl) box_island[bid] = ii;
                    ++ii;
                }
            }
            std::vector<int> island_of_seed(seed_points.size(), -1);
            for (size_t si = 0; si < seed_points.size(); ++si) {
                const auto& q = seed_points[si];
                for (const auto& b : bx) {
                    if (b.contains(q)) {
                        auto it = box_island.find(b.id);
                        island_of_seed[si] = (it != box_island.end()) ? it->second : -2;
                        break;
                    }
                }
            }
            std::cout << "    seed_pts in islands: ";
            for (size_t si = 0; si < seed_points.size(); ++si) {
                if (si > 0) std::cout << ",";
                if (island_of_seed[si] == -1) std::cout << "OUT";
                else std::cout << island_of_seed[si];
            }
            std::cout << "\n";
        }

        int query_success = 0;
        double query_total_s = 0.0;
        double query_total_len = 0.0;
        for (const auto& qp : queries) {
            auto tq0 = std::chrono::steady_clock::now();
            auto qres = planner.query(qp.start, qp.goal, obstacles.data(), n_obs);
            query_total_s += std::chrono::duration<double>(
                std::chrono::steady_clock::now() - tq0).count();
            if (qres.success) {
                ++query_success;
                query_total_len += qres.path_length;
            }
        }
        const double query_mean_s = query_total_s / queries.size();
        const double query_mean_length = query_success > 0
            ? query_total_len / query_success : 0.0;
        std::cout << "    query_success=" << query_success << "/" << queries.size()
                  << "  mean_query_s=" << std::fixed << std::setprecision(4)
                  << query_mean_s
                  << "  mean_path_len=" << std::setprecision(3)
                  << query_mean_length << "\n";

        results.push_back({total_s, bt, n_boxes, n_islands, n_edges,
                          n_islands <= 2, query_success,
                          query_mean_s, query_mean_length});  // islands only diagnostic
    }

    // Also run full query cycle on last seed to get per-query timing
    std::cout << "\n" << std::string(90, '-') << "\n"
              << "  Per-Query Timing (last seed, Paper Tab 3)\n"
              << std::string(90, '-') << "\n\n";

    {
        SBFPlannerConfig cfg;
        cfg.z4_enabled = z4_enabled;
        cfg.split_order = SplitOrder::BEST_TIGHTEN;
        cfg.lect_no_cache = !use_lect_cache;
        cfg.grower.mode = GrowerConfig::Mode::RRT;
        cfg.grower.max_boxes = 200000;
        cfg.grower.timeout_ms = 60000.0;
        cfg.grower.n_threads = 5;
        cfg.grower.rng_seed = 0;
        cfg.grower.max_consecutive_miss = 2000;
        cfg.grower.rrt_goal_bias = 0.1;
        cfg.grower.rrt_step_ratio = 0.05;
        cfg.grower.connect_mode = true;
        cfg.grower.enable_promotion = true;
        cfg.grower.post_connect_extra_boxes = 4000;
        cfg.grower.ffb_config.max_depth = 300;
        cfg.coarsen.target_boxes = 300;
        cfg.coarsen.max_rounds = 100;
        cfg.coarsen.max_lect_fk_per_round = 10000;
        cfg.coarsen.score_threshold = 500.0;
        cfg.grower.bridge_n_threads = n_threads;
        cfg.endpoint_source.source = ep_src;
        cfg.envelope_type.type     = env_type;
        cfg.smoother.shortcut_max_iters = 100;
        cfg.smoother.smooth_window = 3;
        cfg.smoother.smooth_iters = 5;

        SBFPlanner planner(robot, cfg);
        planner.build_coverage(obstacles.data(), n_obs, cfg.grower.timeout_ms,
                               seed_points);

        std::cout << std::left << std::setw(10) << "Query"
                  << std::right
                  << std::setw(14) << "Length(rad)"
                  << std::setw(12) << "Time(s)"
                  << std::setw(10) << "Points"
                  << std::setw(10) << "Status"
                  << "\n" << std::string(60, '-') << "\n";

        double total_len = 0, total_time = 0;
        int n_ok = 0;

        for (auto& qp : queries) {
            auto t0 = std::chrono::steady_clock::now();
            auto res = planner.query(qp.start, qp.goal, obstacles.data(), n_obs);
            double qt = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();

            // Validate path
            std::string status = "FAIL";
            if (res.success) {
                CollisionChecker val_checker(robot, {});
                val_checker.set_obstacles(obstacles.data(), n_obs);
                bool clean = true;
                for (size_t wi = 0; wi + 1 < res.path.size(); ++wi) {
                    double slen = (res.path[wi + 1] - res.path[wi]).norm();
                    int vres = std::max(20, (int)std::ceil(slen / 0.005));
                    if (val_checker.check_segment(res.path[wi], res.path[wi + 1], vres)) {
                        clean = false;
                        break;
                    }
                }
                status = clean ? "OK" : "COLL";
                n_ok++;
                total_len += res.path_length;
            }
            total_time += qt;

            std::cout << std::left << std::setw(10) << qp.label
                      << std::right << std::fixed
                      << std::setw(14) << std::setprecision(3) << res.path_length
                      << std::setw(12) << std::setprecision(4) << qt
                      << std::setw(10) << res.path.size()
                      << std::setw(10) << status
                      << "\n";
        }

        std::cout << std::string(60, '-') << "\n"
                  << std::left << std::setw(10) << "Mean"
                  << std::right << std::fixed
                  << std::setw(14) << std::setprecision(3)
                  << (n_ok > 0 ? total_len / n_ok : 0.0)
                  << std::setw(12) << std::setprecision(4)
                  << total_time / queries.size()
                  << std::setw(10) << ""
                  << std::setw(10) << (std::to_string(n_ok) + "/" +
                                       std::to_string((int)queries.size()))
                  << "\n";
    }

    // Summary table
    std::cout << "\n" << std::string(90, '-') << "\n"
              << "  Build Summary\n"
              << std::string(90, '-') << "\n\n";

    std::vector<double> totals;
    for (auto& r : results) totals.push_back(r.total_s);
    auto st = compute_stats(totals);

    std::cout << "    Build time: med=" << std::setprecision(2) << st.median << "s"
              << "  mean=" << st.mean << "s\n";
    std::cout << "    Boxes (final): ";
    for (auto& r : results) std::cout << r.n_boxes_final << " ";
    std::cout << "\n";
    std::cout << "    Islands: ";
    for (auto& r : results) std::cout << r.n_islands << " ";
    std::cout << "\n";
    std::cout << "    Edges: ";
    for (auto& r : results) std::cout << r.n_edges << " ";
    std::cout << "\n";

    std::cout << "    Query success: ";
    for (auto& r : results) std::cout << r.query_success << "/" << queries.size() << " ";
    std::cout << "\n";

    int n_connected = 0;
    for (auto& r : results) if (r.connected) n_connected++;
    std::cout << "    Connected: " << n_connected << "/" << results.size() << "\n";

    // Per-phase breakdown summary
    std::cout << "\n" << std::string(90, '-') << "\n"
              << "  Per-Phase Breakdown (median over " << n_seeds << " seeds)\n"
              << std::string(90, '-') << "\n\n";

    auto median_of = [](std::vector<double>& v) -> double {
        if (v.empty()) return 0;
        std::sort(v.begin(), v.end());
        return v[v.size()/2];
    };

    std::vector<double> v_lect, v_grow, v_c1, v_brg, v_c2, v_adj, v_adj_pre,
        v_seed_bridge, v_adj_final, v_query_mean_s, v_query_mean_len;
    for (auto& r : results) {
        v_lect.push_back(r.timing.lect_ms);
        v_grow.push_back(r.timing.grow_ms);
        v_c1.push_back(r.timing.coarsen1_ms);
        v_brg.push_back(r.timing.bridge_ms);
        v_c2.push_back(r.timing.coarsen2_ms);
        v_adj.push_back(r.timing.adjacency_ms);
        v_adj_pre.push_back(r.timing.adjacency_pre_seed_ms);
        v_seed_bridge.push_back(r.timing.seed_bridge_ms);
        v_adj_final.push_back(r.timing.adjacency_final_ms);
        v_query_mean_s.push_back(r.query_mean_s * 1000.0);
        v_query_mean_len.push_back(r.query_mean_length);
    }

    std::cout << std::fixed << std::setprecision(0)
              << "    LECT:      " << std::setw(8) << median_of(v_lect) << " ms\n"
              << "    Grow:      " << std::setw(8) << median_of(v_grow) << " ms\n"
              << "    Coarsen1:  " << std::setw(8) << median_of(v_c1) << " ms\n"
              << "    Bridge:    " << std::setw(8) << median_of(v_brg) << " ms\n"
              << "    Coarsen2:  " << std::setw(8) << median_of(v_c2) << " ms\n"
              << "    Adjacency: " << std::setw(8) << median_of(v_adj) << " ms\n"
              << "      pre-seed:" << std::setw(8) << median_of(v_adj_pre) << " ms\n"
              << "      seed-brg:" << std::setw(8) << median_of(v_seed_bridge) << " ms\n"
              << "      final:   " << std::setw(8) << median_of(v_adj_final) << " ms\n"
              << "    Query mean:" << std::setw(8) << median_of(v_query_mean_s) << " ms\n"
              << "    Path mean: " << std::setw(8) << std::setprecision(3)
              << median_of(v_query_mean_len) << " rad\n";

    // Write JSON if requested
    if (!json_out.empty()) {
        std::ofstream ofs(json_out);
        if (ofs.is_open()) {
            ofs << "{\"scene\":\"" << scene_name << "\","
                << "\"robot\":\"" << robot.name() << "\","
                << "\"n_seeds\":" << n_seeds << ","
                << "\"n_threads\":" << n_threads << ","
                << "\"use_unexplored\":" << (use_unexplored ? "true" : "false") << ","
                << "\"use_coordinated_grower\":" << (use_coordinated_grower ? "true" : "false") << ","
                << "\"use_seed_bridge\":" << (use_seed_bridge ? "true" : "false") << ","
                << "\"use_rescue_bridge\":" << (use_rescue_bridge ? "true" : "false") << ","
                << "\"build_results\":[\n";
            for (size_t i = 0; i < results.size(); ++i) {
                auto& r = results[i];
                ofs << (i > 0 ? ",\n" : "")
                    << "  {\"seed\":" << i
                    << ",\"total_ms\":" << std::fixed << std::setprecision(1) << r.total_s * 1000
                    << ",\"lect_ms\":" << r.timing.lect_ms
                    << ",\"grow_ms\":" << r.timing.grow_ms
                    << ",\"coarsen1_ms\":" << r.timing.coarsen1_ms
                    << ",\"bridge_ms\":" << r.timing.bridge_ms
                    << ",\"coarsen2_ms\":" << r.timing.coarsen2_ms
                    << ",\"adjacency_ms\":" << r.timing.adjacency_ms
                    << ",\"adjacency_pre_seed_ms\":" << r.timing.adjacency_pre_seed_ms
                    << ",\"seed_bridge_ms\":" << r.timing.seed_bridge_ms
                    << ",\"adjacency_final_ms\":" << r.timing.adjacency_final_ms
                    << ",\"boxes_final\":" << r.n_boxes_final
                    << ",\"boxes_after_grow\":" << r.timing.boxes_after_grow
                    << ",\"boxes_after_coarsen1\":" << r.timing.boxes_after_coarsen1
                    << ",\"boxes_after_bridge\":" << r.timing.boxes_after_bridge
                    << ",\"islands\":" << r.n_islands
                    << ",\"query_success\":" << r.query_success
                    << ",\"query_mean_s\":" << r.query_mean_s
                    << ",\"query_mean_length\":" << r.query_mean_length
                    << ",\"edges\":" << r.n_edges << "}";
            }
            ofs << "\n]}\n";
            ofs.close();
            std::cout << "\n  JSON written to: " << json_out << "\n";
        }
    }

    std::cout << "\n" << std::string(90, '=') << "\n"
              << "  Exp 6 complete.\n";
    return 0;
}
