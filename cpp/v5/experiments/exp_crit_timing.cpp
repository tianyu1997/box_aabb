/**
 * exp_crit_timing.cpp — CritSample 全流水线计时实验
 *
 * 在 Marcucci 合并场景 (16 obstacles) 上运行 5 个 query pair (AS→TS→CS→LB→RB→AS),
 * 使用 CritSample endpoint source + BEST_TIGHTEN_V2 split order (auto wp=0),
 * 以 one-shot plan() 模式运行并收集各阶段耗时:
 *   1. LECT 构建/加载
 *   2. Forest 生长 (wavefront)
 *   3. S→G 连接器
 *   4. Sweep coarsen
 *   5. Greedy coarsen
 *   6. Bridge 连接
 *   7. Dijkstra 搜索
 *   8. Path 平滑
 *   9. 总耗时
 *
 * 对比模式: --compare 同时运行 IFK 和 CritSample, 输出对比表.
 *
 * 用法:
 *   ./exp_crit_timing [--ep-source crit|ifk] [--split-order btv2|bt|rr]
 *                     [--seeds N] [--threads N] [--timeout MS]
 *                     [--compare] [--cold] [--quick]
 */

#include <sbf/planner/sbf_planner.h>
#include <sbf/core/robot.h>
#include <sbf/envelope/endpoint_source.h>
#include "marcucci_scenes.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <thread>
#include <vector>
#include <regex>
#include <sstream>
#include <unistd.h>

using namespace sbf;

// ═══════════════════════════════════════════════════════════════════════════
// Timing capture: parse SBFPlanner stderr output
// ═══════════════════════════════════════════════════════════════════════════

struct StageTiming {
    double lect_ms    = 0;
    double grow_ms    = 0;
    double sg_ms      = 0;
    double sweep_ms   = 0;
    double greedy_ms  = 0;
    double bridge_ms  = 0;
    double search_ms  = 0;
    double smooth_ms  = 0;
    double total_ms   = 0;

    int n_boxes_grow   = 0;
    int n_boxes_sweep  = 0;
    int n_boxes_greedy = 0;
    int n_boxes_final  = 0;
    int n_islands      = 0;
    int lect_nodes     = 0;
};

// ═══════════════════════════════════════════════════════════════════════════
// Path quality
// ═══════════════════════════════════════════════════════════════════════════

double path_length_metric(const std::vector<Eigen::VectorXd>& path) {
    double len = 0;
    for (size_t i = 1; i < path.size(); ++i)
        len += (path[i] - path[i - 1]).norm();
    return len;
}

// ═══════════════════════════════════════════════════════════════════════════
// Run one query pair with full timing
// ═══════════════════════════════════════════════════════════════════════════

struct RunResult {
    bool success = false;
    StageTiming timing;
    double path_length = 0;
    int n_waypoints = 0;
};

RunResult run_one_query(
    const Robot& robot,
    const std::vector<Obstacle>& obstacles,
    const QueryPair& qp,
    EndpointSourceConfig ep_cfg,
    SplitOrder split_order,
    int n_threads,
    double timeout_ms,
    bool lect_cold,
    uint64_t seed)
{
    RunResult rr;

    SBFPlannerConfig cfg;
    cfg.z4_enabled = true;
    cfg.split_order = split_order;
    cfg.lect_no_cache = lect_cold;
    cfg.endpoint_source = ep_cfg;

    cfg.grower.mode = GrowerConfig::Mode::RRT;
    cfg.grower.max_boxes = 2000;
    cfg.grower.n_threads = n_threads;
    cfg.grower.rng_seed = seed;
    cfg.grower.wavefront_stages = {
        {100}, {300}, {800}, {1500}, {2000}
    };
    cfg.grower.max_consecutive_miss = 100;
    cfg.grower.enable_promotion = true;
    cfg.grower.ffb_config.max_depth = 60;

    cfg.coarsen.target_boxes = 500;
    cfg.coarsen.max_rounds = 100;
    cfg.coarsen.max_lect_fk_per_round = 2000;

    cfg.smoother.shortcut_max_iters = 100;
    cfg.smoother.smooth_window = 3;
    cfg.smoother.smooth_iters = 5;

    cfg.use_gcs = false;

    // ── redirect stderr to capture per-stage timing ──
    // We run plan() which prints timing to stderr; we capture it via pipe
    int pipefd[2];
    if (pipe(pipefd) != 0) {
        perror("pipe");
        return rr;
    }

    int saved_stderr = dup(2);
    dup2(pipefd[1], 2);

    auto t_total = std::chrono::steady_clock::now();
    SBFPlanner planner(robot, cfg);
    auto result = planner.plan(qp.start, qp.goal,
                               obstacles.data(),
                               static_cast<int>(obstacles.size()),
                               timeout_ms);
    rr.timing.total_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t_total).count();

    // Restore stderr & read captured output
    fflush(stderr);
    dup2(saved_stderr, 2);
    close(saved_stderr);
    close(pipefd[1]);

    // Read pipe
    std::string captured;
    char buf[4096];
    ssize_t n;
    while ((n = read(pipefd[0], buf, sizeof(buf) - 1)) > 0) {
        buf[n] = '\0';
        captured += buf;
    }
    close(pipefd[0]);

    // Parse captured PLN timing lines
    auto extract_ms = [&](const std::string& key) -> double {
        // Match "[PLN] key=XXXms"
        std::string pattern = "\\[PLN\\] " + key + "=([0-9.]+)ms";
        std::regex re(pattern);
        std::smatch m;
        if (std::regex_search(captured, m, re))
            return std::stod(m[1].str());
        return 0;
    };

    auto extract_int = [&](const std::string& pattern_str) -> int {
        std::regex re(pattern_str);
        std::smatch m;
        if (std::regex_search(captured, m, re))
            return std::stoi(m[1].str());
        return 0;
    };

    rr.timing.lect_ms   = extract_ms("lect");
    rr.timing.grow_ms   = extract_ms("grow");
    rr.timing.sg_ms     = extract_ms("sg_connect");
    rr.timing.sweep_ms  = extract_ms("sweep");
    rr.timing.greedy_ms = extract_ms("greedy");
    rr.timing.bridge_ms = extract_ms("bridge");

    rr.timing.search_ms = rr.timing.total_ms -
        (rr.timing.lect_ms + rr.timing.grow_ms + rr.timing.sg_ms +
         rr.timing.sweep_ms + rr.timing.greedy_ms + rr.timing.bridge_ms);
    if (rr.timing.search_ms < 0) rr.timing.search_ms = 0;

    // Box counts from "[PLN] grow=XXms (boxes=NNN)"
    rr.timing.n_boxes_grow = extract_int("grow=[0-9.]+ms \\(boxes=([0-9]+)\\)");
    rr.timing.lect_nodes = extract_int("lect=[0-9.]+ms \\(nodes=([0-9]+)\\)");
    // "[PLN] sweep=XXms (NNN->MMM)"
    rr.timing.n_boxes_sweep = extract_int("sweep=[0-9.]+ms \\([0-9]+->([0-9]+)\\)");
    // "[PLN] greedy=XXms (NNN->MMM)"
    rr.timing.n_boxes_greedy = extract_int("greedy=[0-9.]+ms \\([0-9]+->([0-9]+)\\)");

    rr.timing.n_boxes_final = result.n_boxes;

    rr.success = result.success;
    rr.path_length = result.path_length;
    rr.n_waypoints = static_cast<int>(result.path.size());

    // Also print captured output to stderr for debugging
    fprintf(stderr, "%s", captured.c_str());

    return rr;
}

// ═══════════════════════════════════════════════════════════════════════════
// Pretty print
// ═══════════════════════════════════════════════════════════════════════════

void print_header() {
    std::cout << std::string(120, '-') << "\n"
              << std::left
              << std::setw(8) << "Query"
              << std::right
              << std::setw(7)  << "OK"
              << std::setw(9)  << "Total"
              << std::setw(8)  << "LECT"
              << std::setw(9)  << "Grow"
              << std::setw(8)  << "SG"
              << std::setw(9)  << "Sweep"
              << std::setw(9)  << "Greedy"
              << std::setw(9)  << "Bridge"
              << std::setw(9)  << "Search"
              << std::setw(7)  << "Boxes"
              << std::setw(8)  << "Nodes"
              << std::setw(8)  << "Len"
              << "\n"
              << std::string(120, '-') << "\n";
}

void print_row(const std::string& label, const RunResult& r) {
    auto& t = r.timing;
    std::cout << std::left << std::setw(8) << label
              << std::right
              << std::setw(7)  << (r.success ? "OK" : "FAIL")
              << std::setw(8) << std::fixed << std::setprecision(0) << t.total_ms << "ms"
              << std::setw(7) << std::setprecision(0) << t.lect_ms << "ms"
              << std::setw(8) << std::setprecision(0) << t.grow_ms << "ms"
              << std::setw(7) << std::setprecision(0) << t.sg_ms << "ms"
              << std::setw(8) << std::setprecision(0) << t.sweep_ms << "ms"
              << std::setw(8) << std::setprecision(0) << t.greedy_ms << "ms"
              << std::setw(8) << std::setprecision(0) << t.bridge_ms << "ms"
              << std::setw(8) << std::setprecision(0) << t.search_ms << "ms"
              << std::setw(7) << t.n_boxes_final
              << std::setw(8) << t.lect_nodes
              << std::setw(8) << std::setprecision(2) << r.path_length
              << "\n";
}

void print_summary_row(const std::string& label,
                       const std::vector<RunResult>& results) {
    double sum_total = 0, sum_lect = 0, sum_grow = 0, sum_sg = 0;
    double sum_sweep = 0, sum_greedy = 0, sum_bridge = 0, sum_search = 0;
    int n_ok = 0;
    for (auto& r : results) {
        sum_total  += r.timing.total_ms;
        sum_lect   += r.timing.lect_ms;
        sum_grow   += r.timing.grow_ms;
        sum_sg     += r.timing.sg_ms;
        sum_sweep  += r.timing.sweep_ms;
        sum_greedy += r.timing.greedy_ms;
        sum_bridge += r.timing.bridge_ms;
        sum_search += r.timing.search_ms;
        if (r.success) n_ok++;
    }
    int n = static_cast<int>(results.size());
    if (n == 0) return;

    std::cout << std::string(120, '-') << "\n"
              << std::left << std::setw(8) << label
              << std::right
              << std::setw(5) << n_ok << "/" << n
              << std::setw(8) << std::fixed << std::setprecision(0) << sum_total / n << "ms"
              << std::setw(7) << std::setprecision(0) << sum_lect / n << "ms"
              << std::setw(8) << std::setprecision(0) << sum_grow / n << "ms"
              << std::setw(7) << std::setprecision(0) << sum_sg / n << "ms"
              << std::setw(8) << std::setprecision(0) << sum_sweep / n << "ms"
              << std::setw(8) << std::setprecision(0) << sum_greedy / n << "ms"
              << std::setw(8) << std::setprecision(0) << sum_bridge / n << "ms"
              << std::setw(8) << std::setprecision(0) << sum_search / n << "ms"
              << "\n";

    // Also print breakdown percentages
    double pct_lect   = sum_total > 0 ? 100.0 * sum_lect / sum_total : 0;
    double pct_grow   = sum_total > 0 ? 100.0 * sum_grow / sum_total : 0;
    double pct_sg     = sum_total > 0 ? 100.0 * sum_sg / sum_total : 0;
    double pct_sweep  = sum_total > 0 ? 100.0 * sum_sweep / sum_total : 0;
    double pct_greedy = sum_total > 0 ? 100.0 * sum_greedy / sum_total : 0;
    double pct_bridge = sum_total > 0 ? 100.0 * sum_bridge / sum_total : 0;
    double pct_search = sum_total > 0 ? 100.0 * sum_search / sum_total : 0;

    std::cout << std::left << std::setw(8) << "  %"
              << std::right
              << std::setw(7) << ""
              << std::setw(9) << "100%"
              << std::setw(7) << std::setprecision(1) << pct_lect << "%"
              << std::setw(7) << std::setprecision(1) << pct_grow << "%"
              << std::setw(7) << std::setprecision(1) << pct_sg << "%"
              << std::setw(7) << std::setprecision(1) << pct_sweep << "%"
              << std::setw(7) << std::setprecision(1) << pct_greedy << "%"
              << std::setw(7) << std::setprecision(1) << pct_bridge << "%"
              << std::setw(7) << std::setprecision(1) << pct_search << "%"
              << "\n";
}

// ═══════════════════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════════════════

int main(int argc, char** argv) {
    std::string robot_path = std::string(SBF_DATA_DIR) + "/iiwa14.json";
    int n_seeds = 1;
    int n_threads = static_cast<int>(std::thread::hardware_concurrency());
    double timeout_ms = 60000.0;
    bool lect_cold = false;
    bool compare_mode = false;
    std::string ep_source_str = "crit";
    std::string split_order_str = "btv2";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seeds" && i+1 < argc) n_seeds = std::atoi(argv[++i]);
        else if (a == "--threads" && i+1 < argc) n_threads = std::atoi(argv[++i]);
        else if (a == "--timeout" && i+1 < argc) timeout_ms = std::atof(argv[++i]);
        else if (a == "--ep-source" && i+1 < argc) ep_source_str = argv[++i];
        else if (a == "--split-order" && i+1 < argc) split_order_str = argv[++i];
        else if (a == "--cold") lect_cold = true;
        else if (a == "--compare") compare_mode = true;
        else if (a[0] != '-') robot_path = a;
    }

    if (n_threads < 1) n_threads = 1;

    // Parse endpoint source
    auto parse_ep = [](const std::string& s) -> EndpointSourceConfig {
        EndpointSourceConfig ep;
        if (s == "crit" || s == "critsample") {
            ep.source = EndpointSource::CritSample;
        } else if (s == "analytical") {
            ep.source = EndpointSource::Analytical;
        } else {
            ep.source = EndpointSource::IFK;
        }
        return ep;
    };

    // Parse split order
    auto parse_so = [](const std::string& s) -> SplitOrder {
        if (s == "btv2" || s == "BT_V2")
            return SplitOrder::BEST_TIGHTEN_V2;
        else if (s == "bt" || s == "BT")
            return SplitOrder::BEST_TIGHTEN;
        else if (s == "rr" || s == "RR")
            return SplitOrder::ROUND_ROBIN;
        else if (s == "wf" || s == "WF")
            return SplitOrder::WIDEST_FIRST;
        return SplitOrder::BEST_TIGHTEN_V2;
    };

    Robot robot = Robot::from_json(robot_path);
    auto obstacles = make_combined_obstacles();
    auto queries = make_combined_queries();
    int n_obs = static_cast<int>(obstacles.size());

    std::cout << "╔════════════════════════════════════════════════════════════╗\n"
              << "║  CritSample Pipeline Timing — Marcucci Scene             ║\n"
              << "╚════════════════════════════════════════════════════════════╝\n"
              << "  Robot: " << robot.name() << "  DOF=" << robot.n_joints()
              << "  active=" << robot.n_active_links() << "\n"
              << "  Scene: combined (" << n_obs << " obstacles, "
              << queries.size() << " pairs)\n"
              << "  threads=" << n_threads
              << "  timeout=" << timeout_ms << "ms"
              << "  seeds=" << n_seeds
              << "  cache=" << (lect_cold ? "OFF" : "ON") << "\n";

    if (compare_mode) {
        // ── Compare mode: run both IFK and CritSample ──
        struct ConfigSet {
            std::string label;
            EndpointSourceConfig ep;
            SplitOrder so;
        };
        std::vector<ConfigSet> configs = {
            {"IFK+BT",    parse_ep("ifk"),  SplitOrder::BEST_TIGHTEN},
            {"IFK+V2",    parse_ep("ifk"),  SplitOrder::BEST_TIGHTEN_V2},
            {"Crit+BT",   parse_ep("crit"), SplitOrder::BEST_TIGHTEN},
            {"Crit+V2",   parse_ep("crit"), SplitOrder::BEST_TIGHTEN_V2},
        };

        for (auto& cs : configs) {
            std::cout << "\n\n═══ " << cs.label
                      << " (ep=" << endpoint_source_name(cs.ep.source)
                      << " split=" << (cs.so == SplitOrder::BEST_TIGHTEN_V2 ? "BT_V2" :
                                       cs.so == SplitOrder::BEST_TIGHTEN ? "BT" : "RR")
                      << ") ═══\n";

            for (int s = 0; s < n_seeds; ++s) {
                std::cout << "\n  seed=" << s << "\n";
                print_header();

                std::vector<RunResult> all_results;
                for (auto& qp : queries) {
                    auto rr = run_one_query(robot, obstacles, qp,
                                            cs.ep, cs.so,
                                            n_threads, timeout_ms, lect_cold,
                                            static_cast<uint64_t>(s));
                    print_row(qp.label, rr);
                    all_results.push_back(rr);
                }
                print_summary_row("AVG", all_results);
            }
        }
    } else {
        // ── Single configuration mode ──
        auto ep_cfg = parse_ep(ep_source_str);
        auto split_order = parse_so(split_order_str);

        std::cout << "  ep_source=" << endpoint_source_name(ep_cfg.source)
                  << "  split_order=" << split_order_str << "\n\n";

        for (int s = 0; s < n_seeds; ++s) {
            std::cout << "  seed=" << s << "\n";
            print_header();

            std::vector<RunResult> all_results;
            for (auto& qp : queries) {
                auto rr = run_one_query(robot, obstacles, qp,
                                        ep_cfg, split_order,
                                        n_threads, timeout_ms, lect_cold,
                                        static_cast<uint64_t>(s));
                print_row(qp.label, rr);
                all_results.push_back(rr);
            }
            print_summary_row("AVG", all_results);
        }
    }

    std::cout << "\n  Experiment complete.\n";
    return 0;
}
