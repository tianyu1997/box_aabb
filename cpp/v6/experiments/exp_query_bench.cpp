/**
 * exp_query_bench.cpp — ≥20-query benchmark on the canonical 16-obstacle scene.
 *
 * Builds one SBF forest with the canonical 5-anchor seed set, then runs N
 * (default 24) extended queries enumerated from 8 named anchors. Reports
 * per-query latency + length and aggregate SR.
 *
 * Usage:
 *   ./exp_query_bench --n-queries 24 --seeds 3 --threads 2
 *                     --json results_new/D_query_bench.json
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
#include <string>
#include <sys/resource.h>
#include <thread>
#include <vector>

using namespace sbf;

static long peak_rss_kb() {
    struct rusage ru{};
    getrusage(RUSAGE_SELF, &ru);
    return ru.ru_maxrss;
}

int main(int argc, char** argv) {
    sbf::init_log_from_env();
    std::string robot_path = std::string(SBF_DATA_DIR) + "/iiwa14.json";
    int n_queries = 24;
    int n_seeds = 3;
    int n_threads = 2;
    std::string json_out;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--n-queries" && i + 1 < argc) n_queries = std::atoi(argv[++i]);
        else if (a == "--seeds" && i + 1 < argc) n_seeds = std::atoi(argv[++i]);
        else if (a == "--threads" && i + 1 < argc) n_threads = std::atoi(argv[++i]);
        else if (a == "--json" && i + 1 < argc) json_out = argv[++i];
        else if (a[0] != '-') robot_path = a;
    }

    Robot robot = Robot::from_json(robot_path);
    auto obstacles = make_combined_obstacles();
    int n_obs = static_cast<int>(obstacles.size());
    auto seed_anchors = make_combined_queries();
    std::vector<Eigen::VectorXd> seeds;
    {
        auto eq = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
            return (a - b).squaredNorm() < 1e-8;
        };
        for (auto& qp : seed_anchors) {
            bool dup = false;
            for (auto& s : seeds) if (eq(s, qp.start)) { dup = true; break; }
            if (!dup) seeds.push_back(qp.start);
            dup = false;
            for (auto& s : seeds) if (eq(s, qp.goal)) { dup = true; break; }
            if (!dup) seeds.push_back(qp.goal);
        }
    }
    auto queries = make_extended_queries(n_queries);
    n_queries = static_cast<int>(queries.size());

    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints()
              << "\nScene: marcucci_combined (" << n_obs << " obs)"
              << "\nQueries: " << n_queries << "  Seeds: " << n_seeds
              << "  Threads: " << n_threads << "\n\n";

    struct Cell { int seed; double build_s; int n_boxes;
                  std::vector<int> ok; std::vector<double> qt;
                  std::vector<double> qlen; long peak_rss_kb; };
    std::vector<Cell> cells;

    for (int seed = 0; seed < n_seeds; ++seed) {
        SBFPlannerConfig cfg;
        cfg.z4_enabled = true;
        cfg.split_order = SplitOrder::BEST_TIGHTEN;
        cfg.lect_no_cache = true;
        cfg.grower.mode = GrowerConfig::Mode::RRT;
        cfg.grower.max_boxes = 5000;
        cfg.grower.timeout_ms = 30000.0;
        cfg.grower.n_threads = 2;
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
                               cfg.grower.timeout_ms, seeds);
        double build_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();

        Cell c{seed, build_s, planner.n_boxes(), {}, {}, {}, 0};

        std::cout << "─── seed=" << seed << " build="
                  << std::fixed << std::setprecision(2) << build_s
                  << "s boxes=" << c.n_boxes << " ───\n";
        for (int qi = 0; qi < n_queries; ++qi) {
            auto& qp = queries[qi];
            auto qt0 = std::chrono::steady_clock::now();
            auto res = planner.query(qp.start, qp.goal,
                                     obstacles.data(), n_obs);
            double qt = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - qt0).count();
            int ok = 0; double L = 0;
            if (res.success) {
                CollisionChecker val(robot, {});
                val.set_obstacles(obstacles.data(), n_obs);
                bool clean = true;
                for (size_t wi = 0; wi + 1 < res.path.size(); ++wi) {
                    double slen = (res.path[wi+1] - res.path[wi]).norm();
                    int vres = std::max(20, (int)std::ceil(slen / 0.005));
                    if (val.check_segment(res.path[wi], res.path[wi+1], vres)) {
                        clean = false; break;
                    }
                }
                if (clean) { ok = 1; L = res.path_length; }
            }
            c.ok.push_back(ok);
            c.qt.push_back(qt);
            c.qlen.push_back(L);
        }
        c.peak_rss_kb = peak_rss_kb();
        int sum_ok = std::accumulate(c.ok.begin(), c.ok.end(), 0);
        std::cout << "  → SR=" << sum_ok << "/" << n_queries
                  << " RSS=" << (c.peak_rss_kb / 1024) << "MB\n";
        cells.push_back(std::move(c));
    }

    // Aggregate
    std::vector<double> all_qt, all_len; int total_ok = 0, total_q = 0;
    std::vector<double> builds;
    for (auto& c : cells) {
        builds.push_back(c.build_s);
        for (int i = 0; i < n_queries; ++i) {
            total_q++;
            if (c.ok[i]) {
                total_ok++;
                all_qt.push_back(c.qt[i]);
                all_len.push_back(c.qlen[i]);
            }
        }
    }
    auto med = [](std::vector<double> v) {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        return v[v.size()/2];
    };
    auto qtl = [](std::vector<double> v, double q) {
        if (v.empty()) return 0.0;
        std::sort(v.begin(), v.end());
        size_t k = std::min(v.size()-1, (size_t)(q*(v.size()-1)));
        return v[k];
    };
    std::cout << "\n─── Summary ───────────────────────────────\n"
              << "  build_s  median: " << std::fixed << std::setprecision(2) << med(builds) << "\n"
              << "  query_s  median: " << std::setprecision(3) << med(all_qt)
              << "  p25/p75: " << qtl(all_qt,0.25) << "/" << qtl(all_qt,0.75) << "\n"
              << "  path_rad median: " << med(all_len) << "\n"
              << "  SR overall:  " << total_ok << "/" << total_q
              << " = " << (100.0 * total_ok / total_q) << "%\n";

    if (!json_out.empty()) {
        std::ofstream of(json_out);
        of << "{\n  \"scene\": \"marcucci_combined_16obs\",\n"
           << "  \"n_queries\": " << n_queries << ",\n"
           << "  \"n_seeds\": " << n_seeds << ",\n"
           << "  \"queries\": [";
        for (int i = 0; i < n_queries; ++i)
            of << (i ? ", " : "") << "\"" << queries[i].label << "\"";
        of << "],\n  \"seeds\": [";
        for (size_t s = 0; s < cells.size(); ++s) {
            auto& c = cells[s];
            of << (s ? ",\n    " : "\n    ")
               << "{\"seed\":" << c.seed
               << ",\"build_s\":" << c.build_s
               << ",\"n_boxes\":" << c.n_boxes
               << ",\"peak_rss_kb\":" << c.peak_rss_kb
               << ",\"ok\":[";
            for (int i = 0; i < n_queries; ++i) of << (i ? "," : "") << c.ok[i];
            of << "],\"qt_s\":[";
            for (int i = 0; i < n_queries; ++i)
                of << (i ? "," : "") << std::setprecision(6) << c.qt[i];
            of << "],\"qlen_rad\":[";
            for (int i = 0; i < n_queries; ++i)
                of << (i ? "," : "") << std::setprecision(6) << c.qlen[i];
            of << "]}";
        }
        of << "\n  ],\n"
           << "  \"summary\": {\n"
           << "    \"build_s_median\": " << std::setprecision(4) << med(builds) << ",\n"
           << "    \"query_s_median\": " << med(all_qt) << ",\n"
           << "    \"query_s_p25\": "    << qtl(all_qt,0.25) << ",\n"
           << "    \"query_s_p75\": "    << qtl(all_qt,0.75) << ",\n"
           << "    \"path_rad_median\": " << med(all_len) << ",\n"
           << "    \"sr_total\": "        << total_ok << ",\n"
           << "    \"queries_total\": "   << total_q << "\n"
           << "  }\n}\n";
        std::cout << "\nWrote " << json_out << "\n";
    }
    return 0;
}
