/**
 * exp_obstacle_scale.cpp — Obstacle-count scaling on the Marcucci-style scene.
 *
 * For each n in --N-values (default 8,16,32), build the SBF coverage on
 * make_scaled_obstacles(n) using the canonical 5 anchors as seed points,
 * then run all 5 benchmark queries. Reports build time, box count,
 * SR, mean path length, peak RSS. Outputs JSON for the new tab:scale
 * obstacle row of the paper.
 *
 * Usage:
 *   ./exp_obstacle_scale --N-values 8,16,32 --seeds 5 --threads 16
 *                        --json results_new/exp_obstacle_scale.json
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
#include <sys/resource.h>
#include <thread>
#include <vector>

using namespace sbf;

static std::vector<int> parse_int_list(const std::string& s) {
    std::vector<int> out; std::stringstream ss(s); std::string tok;
    while (std::getline(ss, tok, ',')) if (!tok.empty()) out.push_back(std::atoi(tok.c_str()));
    return out;
}

static long peak_rss_kb() {
    struct rusage ru{};
    getrusage(RUSAGE_SELF, &ru);
    return ru.ru_maxrss;
}

int main(int argc, char** argv) {
    sbf::init_log_from_env();
    std::string robot_path = std::string(SBF_DATA_DIR) + "/iiwa14.json";
    int n_seeds = 5;
    int n_threads = static_cast<int>(std::thread::hardware_concurrency());
    std::vector<int> N_values = {8, 16, 32};
    std::string json_out;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seeds" && i + 1 < argc) n_seeds = std::atoi(argv[++i]);
        else if (a == "--threads" && i + 1 < argc) n_threads = std::atoi(argv[++i]);
        else if (a == "--N-values" && i + 1 < argc) N_values = parse_int_list(argv[++i]);
        else if (a == "--json" && i + 1 < argc) json_out = argv[++i];
        else if (a[0] != '-') robot_path = a;
    }
    if (n_threads < 1) n_threads = 1;

    Robot robot = Robot::from_json(robot_path);
    auto queries = make_combined_queries();

    // Canonical 5 anchors.
    std::vector<Eigen::VectorXd> seeds;
    {
        auto eq = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
            return (a - b).squaredNorm() < 1e-8;
        };
        for (auto& qp : queries) {
            bool dup = false;
            for (auto& s : seeds) if (eq(s, qp.start)) { dup = true; break; }
            if (!dup) seeds.push_back(qp.start);
            dup = false;
            for (auto& s : seeds) if (eq(s, qp.goal)) { dup = true; break; }
            if (!dup) seeds.push_back(qp.goal);
        }
    }

    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints()
              << "\nN-values: ";
    for (int n : N_values) std::cout << n << ' ';
    std::cout << "  Seeds=" << n_seeds << "  Threads=" << n_threads << "\n\n";

    struct Cell { int N; int seed; double build_s; int n_boxes; int n_islands;
                  int n_ok; double mean_len; double mean_qt; long peak_rss_kb; };
    std::vector<Cell> cells;

    for (int N : N_values) {
        auto obstacles = make_scaled_obstacles(N);
        int n_obs = static_cast<int>(obstacles.size());
        std::cout << "─── N=" << N << " (actual=" << n_obs << " obs) ───\n";
        for (int seed = 0; seed < n_seeds; ++seed) {
            SBFPlannerConfig cfg;
            cfg.z4_enabled = true;
            cfg.split_order = SplitOrder::BEST_TIGHTEN;
            cfg.lect_no_cache = true; // memory-safe per-cell isolation
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

            int n_boxes = planner.n_boxes();
            const auto& adj = planner.adjacency();
            int n_islands = static_cast<int>(find_islands(adj).size());

            int n_ok = 0; double tot_len = 0, tot_qt = 0;
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
                        double slen = (res.path[wi+1] - res.path[wi]).norm();
                        int vres = std::max(20, (int)std::ceil(slen / 0.005));
                        if (val.check_segment(res.path[wi], res.path[wi+1], vres)) {
                            clean = false; break;
                        }
                    }
                    if (clean) { n_ok++; tot_len += res.path_length; }
                }
                tot_qt += qt;
            }
            Cell c{N, seed, build_s, n_boxes, n_islands, n_ok,
                   n_ok > 0 ? tot_len / n_ok : 0.0,
                   tot_qt / queries.size(), peak_rss_kb()};
            cells.push_back(c);
            std::cout << "  seed=" << seed
                      << " build=" << std::fixed << std::setprecision(2) << build_s << "s"
                      << " boxes=" << n_boxes
                      << " islands=" << n_islands
                      << " SR=" << n_ok << "/" << queries.size()
                      << " mean_len=" << std::setprecision(3) << c.mean_len
                      << " RSS=" << (c.peak_rss_kb / 1024) << "MB\n";
        }
    }

    std::cout << "\n─── Summary ───\n"
              << std::left << std::setw(6) << "N"
              << std::setw(14) << "build_med(s)"
              << std::setw(12) << "boxes_med"
              << std::setw(10) << "SR%"
              << std::setw(14) << "mean_len(rad)"
              << std::setw(12) << "peak_RSS(MB)\n";
    for (int N : N_values) {
        std::vector<double> bs, lens; std::vector<int> sr, bx; std::vector<long> rss;
        for (auto& c : cells) if (c.N == N) {
            bs.push_back(c.build_s); bx.push_back(c.n_boxes);
            sr.push_back(c.n_ok); lens.push_back(c.mean_len);
            rss.push_back(c.peak_rss_kb);
        }
        if (bs.empty()) continue;
        std::sort(bs.begin(), bs.end()); std::sort(bx.begin(), bx.end()); std::sort(rss.begin(), rss.end());
        double sr_pct = 100.0 * std::accumulate(sr.begin(), sr.end(), 0)
                        / (sr.size() * (int)queries.size());
        double mean_len = 0; int nl = 0;
        for (double l : lens) if (l > 0) { mean_len += l; nl++; }
        if (nl) mean_len /= nl;
        std::cout << std::left << std::setw(6) << N
                  << std::setw(14) << std::fixed << std::setprecision(2) << bs[bs.size()/2]
                  << std::setw(12) << bx[bx.size()/2]
                  << std::setw(10) << std::setprecision(1) << sr_pct
                  << std::setw(14) << std::setprecision(3) << mean_len
                  << std::setw(12) << (rss[rss.size()/2] / 1024) << "\n";
    }

    if (!json_out.empty()) {
        std::ofstream of(json_out);
        of << "{\n  \"scene\": \"marcucci_scaled\",\n  \"n_queries\": "
           << queries.size() << ",\n  \"n_seeds\": " << n_seeds
           << ",\n  \"cells\": [";
        for (size_t i = 0; i < cells.size(); ++i) {
            auto& c = cells[i];
            of << (i ? ",\n    " : "\n    ")
               << "{\"N\":" << c.N << ",\"seed\":" << c.seed
               << ",\"build_s\":" << c.build_s
               << ",\"n_boxes\":" << c.n_boxes
               << ",\"n_islands\":" << c.n_islands
               << ",\"n_ok\":" << c.n_ok
               << ",\"mean_len_rad\":" << c.mean_len
               << ",\"mean_query_s\":" << c.mean_qt
               << ",\"peak_rss_kb\":" << c.peak_rss_kb << "}";
        }
        of << "\n  ]\n}\n";
        std::cout << "\nWrote " << json_out << "\n";
    }
    return 0;
}
