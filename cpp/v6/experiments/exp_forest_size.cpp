/**
 * exp_forest_size.cpp — |F| (seed-tree count) completeness sweep.
 *
 * For each |F| in --F-values (default 1,2,3,4,5), grow coverage with only
 * the first |F| canonical seed points (subset of {AS,TS,CS,LB,RB}), then
 * run all 5 benchmark queries and report success rate, build time, box
 * count and per-query mean path length. Outputs JSON for the paper's
 * |F|>=3 completeness table.
 *
 * Usage:
 *   ./exp_forest_size --F-values 1,2,3,4,5 --seeds 5 --threads 16
 *                     --json results_new/exp_forest_size.json
 */
#include <sbf/planner/sbf_planner.h>
#include <sbf/forest/connectivity.h>
#include <sbf/core/robot.h>
#include <sbf/core/log.h>
#include "marcucci_scenes.h"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace sbf;

static std::vector<int> parse_int_list(const std::string& s) {
    std::vector<int> out; std::stringstream ss(s); std::string tok;
    while (std::getline(ss, tok, ',')) if (!tok.empty()) out.push_back(std::atoi(tok.c_str()));
    return out;
}

int main(int argc, char** argv) {
    sbf::init_log_from_env();
    std::string robot_path = std::string(SBF_DATA_DIR) + "/iiwa14.json";
    int n_seeds = 5;
    int n_threads = static_cast<int>(std::thread::hardware_concurrency());
    std::vector<int> F_values = {1, 2, 3, 4, 5};
    std::string json_out;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seeds" && i + 1 < argc) n_seeds = std::atoi(argv[++i]);
        else if (a == "--threads" && i + 1 < argc) n_threads = std::atoi(argv[++i]);
        else if (a == "--F-values" && i + 1 < argc) F_values = parse_int_list(argv[++i]);
        else if (a == "--json" && i + 1 < argc) json_out = argv[++i];
        else if (a[0] != '-') robot_path = a;
    }
    if (n_threads < 1) n_threads = 1;

    Robot robot = Robot::from_json(robot_path);
    auto obstacles = make_combined_obstacles();
    auto queries   = make_combined_queries();
    int n_obs = static_cast<int>(obstacles.size());

    // Canonical 5 unique seed points in deterministic order.
    std::vector<Eigen::VectorXd> all_seeds;
    {
        auto eq = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
            return (a - b).squaredNorm() < 1e-8;
        };
        for (auto& qp : queries) {
            bool dup = false;
            for (auto& s : all_seeds) if (eq(s, qp.start)) { dup = true; break; }
            if (!dup) all_seeds.push_back(qp.start);
            dup = false;
            for (auto& s : all_seeds) if (eq(s, qp.goal)) { dup = true; break; }
            if (!dup) all_seeds.push_back(qp.goal);
        }
    }
    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints() << "\n"
              << "Scene: combined (" << n_obs << " obs)\n"
              << "Unique anchors: " << all_seeds.size() << "  (queries: "
              << queries.size() << ")\n"
              << "F-values: ";
    for (int F : F_values) std::cout << F << ' ';
    std::cout << "\nSeeds=" << n_seeds << "  Threads=" << n_threads << "\n\n";

    struct Cell { int F; int seed; double build_s; int n_boxes; int n_islands;
                  int n_ok; double mean_len; double mean_qt; bool connected; };
    std::vector<Cell> cells;

    for (int F : F_values) {
        if (F < 1 || F > (int)all_seeds.size()) {
            std::cerr << "[skip] F=" << F << " out of range [1,"
                      << all_seeds.size() << "]\n"; continue;
        }
        std::vector<Eigen::VectorXd> seed_subset(all_seeds.begin(),
                                                 all_seeds.begin() + F);
        std::cout << "─── F=" << F << " ────────────────────\n";
        for (int seed = 0; seed < n_seeds; ++seed) {
            SBFPlannerConfig cfg;
            cfg.z4_enabled = true;
            cfg.split_order = SplitOrder::BEST_TIGHTEN;
            cfg.lect_no_cache = true; // memory-safe: no cross-cell mmap accumulation
            cfg.grower.mode = GrowerConfig::Mode::RRT;
            cfg.grower.max_boxes = 5000;
            cfg.grower.timeout_ms = 30000.0;
            cfg.grower.n_threads = std::min(F, 2);
            cfg.grower.rng_seed = static_cast<uint64_t>(seed);
            cfg.grower.max_consecutive_miss = 2000;
            cfg.grower.rrt_goal_bias = 0.1;
            cfg.grower.rrt_step_ratio = 0.05;
            cfg.grower.connect_mode = true;
            cfg.grower.enable_promotion = true;
            cfg.grower.post_connect_extra_boxes = 1000;
            cfg.grower.ffb_config.max_depth = 300;
            cfg.coarsen.target_boxes = 300;
            cfg.coarsen.max_rounds = 100;
            cfg.coarsen.max_lect_fk_per_round = 10000;
            cfg.coarsen.score_threshold = 500.0;
            cfg.grower.bridge_n_threads = n_threads;
            cfg.endpoint_source.source = EndpointSource::IFK;
            cfg.envelope_type.type     = EnvelopeType::Hull16_Grid;

            SBFPlanner planner(robot, cfg);
            auto t0 = std::chrono::steady_clock::now();
            planner.build_coverage(obstacles.data(), n_obs,
                                   cfg.grower.timeout_ms, seed_subset);
            double build_s = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();

            int n_boxes = planner.n_boxes();
            const auto& adj = planner.adjacency();
            int n_islands = static_cast<int>(find_islands(adj).size());

            int n_ok = 0;
            double tot_len = 0, tot_qt = 0;
            for (auto& qp : queries) {
                auto qt0 = std::chrono::steady_clock::now();
                auto res = planner.query(qp.start, qp.goal,
                                         obstacles.data(), n_obs);
                double qt = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - qt0).count();
                if (res.success) {
                    CollisionChecker val(robot, {});
                    val.set_obstacles(obstacles.data(), n_obs);
                    bool clean = true;
                    for (size_t wi = 0; wi + 1 < res.path.size(); ++wi) {
                        double slen = (res.path[wi+1]-res.path[wi]).norm();
                        int vres = std::max(20,(int)std::ceil(slen/0.005));
                        if (val.check_segment(res.path[wi], res.path[wi+1], vres)) {
                            clean = false; break;
                        }
                    }
                    if (clean) { n_ok++; tot_len += res.path_length; }
                }
                tot_qt += qt;
            }
            Cell c{F, seed, build_s, n_boxes, n_islands, n_ok,
                   n_ok > 0 ? tot_len / n_ok : 0.0,
                   tot_qt / queries.size(),
                   n_islands == 1};
            cells.push_back(c);
            std::cout << "  seed=" << seed
                      << " build=" << std::fixed << std::setprecision(2) << build_s << "s"
                      << " boxes=" << n_boxes
                      << " islands=" << n_islands
                      << " SR=" << n_ok << "/" << queries.size()
                      << " mean_len=" << std::setprecision(3) << c.mean_len
                      << "\n";
        }
    }

    // Summary
    std::cout << "\n─── Summary ───────────────────\n"
              << std::left << std::setw(6) << "|F|"
              << std::setw(14) << "build_med(s)"
              << std::setw(12) << "boxes_med"
              << std::setw(10) << "SR%"
              << std::setw(14) << "mean_len(rad)\n";
    for (int F : F_values) {
        std::vector<double> bs, lens; std::vector<int> sr, bx;
        for (auto& c : cells) if (c.F == F) {
            bs.push_back(c.build_s); bx.push_back(c.n_boxes);
            sr.push_back(c.n_ok); lens.push_back(c.mean_len);
        }
        if (bs.empty()) continue;
        std::sort(bs.begin(), bs.end()); std::sort(bx.begin(), bx.end());
        double sr_pct = 100.0 * std::accumulate(sr.begin(), sr.end(), 0)
                        / (sr.size() * (int)queries.size());
        double mean_len = 0; int nl = 0;
        for (double l : lens) if (l > 0) { mean_len += l; nl++; }
        if (nl) mean_len /= nl;
        std::cout << std::left << std::setw(6) << F
                  << std::setw(14) << std::fixed << std::setprecision(2) << bs[bs.size()/2]
                  << std::setw(12) << bx[bx.size()/2]
                  << std::setw(10) << std::setprecision(1) << sr_pct
                  << std::setw(14) << std::setprecision(3) << mean_len << "\n";
    }

    if (!json_out.empty()) {
        std::ofstream of(json_out);
        of << "{\n  \"scene\": \"combined\",\n  \"n_queries\": "
           << queries.size() << ",\n  \"n_seeds\": " << n_seeds
           << ",\n  \"cells\": [";
        for (size_t i = 0; i < cells.size(); ++i) {
            auto& c = cells[i];
            of << (i ? ",\n    " : "\n    ")
               << "{\"F\":" << c.F << ",\"seed\":" << c.seed
               << ",\"build_s\":" << c.build_s
               << ",\"n_boxes\":" << c.n_boxes
               << ",\"n_islands\":" << c.n_islands
               << ",\"n_ok\":" << c.n_ok
               << ",\"mean_len_rad\":" << c.mean_len
               << ",\"mean_query_s\":" << c.mean_qt
               << ",\"connected\":" << (c.connected ? "true" : "false") << "}";
        }
        of << "\n  ]\n}\n";
        std::cout << "\nWrote " << json_out << "\n";
    }
    return 0;
}
