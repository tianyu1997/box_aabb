/**
 * test_ffb_marcucci.cpp — 对 Marcucci 场景 5 个 s-t 点分别做 FFB
 *
 * 10 个 milestone config: AS, TS, CS, LB, RB  (既作 start 又作 goal)
 * 合并场景 (16 obstacles)
 * 打印: 是否成功 / fail_code / box volume / depth / timing
 *
 * --compare  同时对比 ROUND_ROBIN 和 BEST_TIGHTEN (cold start)
 */

#include <sbf/ffb/ffb.h>
#include <sbf/lect/lect.h>
#include <sbf/lect/lect_io.h>
#include <sbf/core/robot.h>
#include <sbf/core/types.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>
#include "marcucci_scenes.h"

#include <cinttypes>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace sbf;

static const char* fail_code_str(int code) {
    switch (code) {
        case 0: return "SUCCESS";
        case 1: return "OCCUPIED";
        case 2: return "MAX_DEPTH";
        case 3: return "MIN_EDGE(removed)";
        case 4: return "DEADLINE";
        default: return "UNKNOWN";
    }
}

static const char* split_order_str(SplitOrder so) {
    switch (so) {
        case SplitOrder::ROUND_ROBIN:     return "ROUND_ROBIN";
        case SplitOrder::WIDEST_FIRST:    return "WIDEST_FIRST";
        case SplitOrder::BEST_TIGHTEN:    return "BEST_TIGHTEN";
        case SplitOrder::BEST_TIGHTEN_V2: return "BEST_TIGHTEN_V2";
        default: return "UNKNOWN";
    }
}

struct FFBRunResult {
    std::string name;
    bool ok;
    int fail_code;
    int node_idx;
    int depth;
    int n_new_nodes;
    int n_steps;
    double total_ms;
    double envelope_ms;
    double collide_ms;
    double volume;
    std::vector<Interval> intervals;
};

// Run FFB on all configs with a given split order, returns per-config results
static std::vector<FFBRunResult> run_ffb_suite(
    const Robot& robot,
    const std::vector<Obstacle>& obstacles,
    SplitOrder split_order,
    const FFBConfig& ffb_cfg,
    const EndpointSourceConfig& ep_cfg,
    bool load_cache,
    int n_probes = 8,
    double width_power = 1.0)
{
    auto root_intervals = robot.joint_limits().limits;
    EnvelopeTypeConfig env_cfg;

    LECT lect(robot, root_intervals, ep_cfg, env_cfg);
    lect.set_split_order(split_order);
    lect.set_n_bt_probes(n_probes);
    lect.set_bt_width_power(width_power);

    if (load_cache) {
        std::string home = std::getenv("HOME") ? std::getenv("HOME") : ".";
        uint64_t fp = robot.fingerprint();
        char hex[17];
        snprintf(hex, sizeof(hex), "%016" PRIx64, fp);
        std::string path = home + "/.sbf_cache/" + robot.name() + "_" + hex + ".lect";
        if (lect_load_binary(lect, robot, path))
            std::cout << "  Loaded LECT cache (" << lect.n_nodes() << " nodes)\n";
    }

    struct NamedConfig {
        std::string name;
        Eigen::VectorXd q;
    };
    std::vector<NamedConfig> configs = {
        {"AS", config_AS()},
        {"TS", config_TS()},
        {"CS", config_CS()},
        {"LB", config_LB()},
        {"RB", config_RB()},
    };

    std::vector<FFBRunResult> results;
    for (auto& nc : configs) {
        FFBResult res = find_free_box(lect, nc.q,
                                      obstacles.data(),
                                      static_cast<int>(obstacles.size()),
                                      ffb_cfg);
        FFBRunResult r;
        r.name = nc.name;
        r.ok = res.success();
        r.fail_code = res.fail_code;
        r.node_idx = res.node_idx;
        r.n_new_nodes = res.n_new_nodes;
        r.n_steps = res.n_steps;
        r.total_ms = res.total_ms;
        r.envelope_ms = res.envelope_ms;
        r.collide_ms = res.collide_ms;
        r.depth = -1;
        r.volume = 0.0;

        if (res.success()) {
            r.intervals = lect.node_intervals(res.node_idx);
            r.depth = lect.depth(res.node_idx);
            r.volume = 1.0;
            for (auto& iv : r.intervals)
                r.volume *= iv.width();
        }
        results.push_back(std::move(r));
    }

    std::cout << "  LECT nodes after FFB: " << lect.n_nodes() << "\n";
    return results;
}

static void print_results(const std::string& label,
                           const std::vector<FFBRunResult>& results) {
    std::cout << "\n═══ " << label << " ═══\n";
    std::cout << std::left
              << std::setw(6) << "Name"
              << std::setw(10) << "Result"
              << std::setw(12) << "Code"
              << std::setw(8)  << "Depth"
              << std::setw(8)  << "Steps"
              << std::setw(10) << "NewNode"
              << std::setw(12) << "TotalMs"
              << std::setw(12) << "EnvMs"
              << std::setw(12) << "ColMs"
              << std::setw(14) << "Volume"
              << "\n";
    std::cout << std::string(104, '-') << "\n";

    std::cout << std::fixed << std::setprecision(3);
    int n_ok = 0;
    for (auto& r : results) {
        if (r.ok) n_ok++;
        std::cout << std::left
                  << std::setw(6)  << r.name
                  << std::setw(10) << (r.ok ? "OK" : "FAIL")
                  << std::setw(12) << fail_code_str(r.fail_code)
                  << std::setw(8)  << r.depth
                  << std::setw(8)  << r.n_steps
                  << std::setw(10) << r.n_new_nodes
                  << std::setw(12) << r.total_ms
                  << std::setw(12) << r.envelope_ms
                  << std::setw(12) << r.collide_ms
                  << std::scientific << std::setprecision(4)
                  << std::setw(14) << r.volume
                  << std::fixed << std::setprecision(3)
                  << "\n";
        if (r.ok) {
            std::cout << "       Intervals:";
            for (int d = 0; d < static_cast<int>(r.intervals.size()); ++d) {
                std::cout << " j" << d << "=[" << std::setprecision(4)
                          << r.intervals[d].lo << "," << r.intervals[d].hi
                          << "](w=" << std::setprecision(4) << r.intervals[d].width() << ")";
            }
            std::cout << "\n";
        }
    }
    std::cout << "  Success: " << n_ok << " / " << results.size() << "\n";
}

int main(int argc, char** argv) {
    std::string robot_path = std::string(SBF_DATA_DIR) + "/iiwa14.json";
    double min_edge_unused = 0.0;  // removed, kept for CLI compat
    int max_depth = 60;
    double deadline_ms = 0.0;
    bool compare_mode = false;
    bool no_cache = false;
    int n_probes = 8;
    double width_power = 1.0;
    std::string split_str = "best_tighten";
    std::string ep_source_str = "ifk";

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--robot" && i + 1 < argc) robot_path = argv[++i];
        else if (a == "--min-edge" && i + 1 < argc) { min_edge_unused = std::atof(argv[++i]); (void)min_edge_unused; }
        else if (a == "--max-depth" && i + 1 < argc) max_depth = std::atoi(argv[++i]);
        else if (a == "--deadline" && i + 1 < argc) deadline_ms = std::atof(argv[++i]);
        else if (a == "--split-order" && i + 1 < argc) split_str = argv[++i];
        else if (a == "--ep-source" && i + 1 < argc) ep_source_str = argv[++i];
        else if (a == "--n-probes" && i + 1 < argc) n_probes = std::atoi(argv[++i]);
        else if (a == "--width-power" && i + 1 < argc) width_power = std::atof(argv[++i]);
        else if (a == "--compare") compare_mode = true;
        else if (a == "--no-cache") no_cache = true;
    }

    Robot robot = Robot::from_json(robot_path);
    std::cout << "Robot: " << robot.name()
              << "  DOF=" << robot.n_joints()
              << "  active_links=" << robot.n_active_links() << "\n";

    auto obstacles = make_combined_obstacles();
    std::cout << "Scene: combined  (" << obstacles.size() << " obstacles)\n";
    std::cout << "FFB config: max_depth=" << max_depth
              << "  deadline_ms=" << deadline_ms
              << "  ep_source=" << ep_source_str << "\n";

    // Parse endpoint source
    EndpointSourceConfig ep_cfg;
    if (ep_source_str == "crit" || ep_source_str == "critsample") {
        ep_cfg.source = EndpointSource::CritSample;
    } else if (ep_source_str == "analytical") {
        ep_cfg.source = EndpointSource::Analytical;
    } else {
        ep_cfg.source = EndpointSource::IFK;
    }

    FFBConfig ffb_cfg;
    ffb_cfg.max_depth = max_depth;
    ffb_cfg.deadline_ms = deadline_ms;

    if (compare_mode) {
        // ── Compare ROUND_ROBIN vs BEST_TIGHTEN vs BEST_TIGHTEN_V2 (cold start) ──
        std::cout << "\n====== COMPARE MODE: RR vs BT vs BT_V2 (cold, n_probes=" << n_probes << ", wp=" << width_power << ") ======\n";

        std::cout << "\n── Running ROUND_ROBIN ──\n";
        auto rr_results = run_ffb_suite(robot, obstacles, SplitOrder::ROUND_ROBIN, ffb_cfg, ep_cfg, false, n_probes, width_power);
        print_results("ROUND_ROBIN", rr_results);

        std::cout << "\n── Running BEST_TIGHTEN ──\n";
        auto bt_results = run_ffb_suite(robot, obstacles, SplitOrder::BEST_TIGHTEN, ffb_cfg, ep_cfg, false, n_probes, width_power);
        print_results("BEST_TIGHTEN", bt_results);

        std::cout << "\n── Running BEST_TIGHTEN_V2 (K=" << n_probes << ", wp=" << width_power << ") ──\n";
        auto v2_results = run_ffb_suite(robot, obstacles, SplitOrder::BEST_TIGHTEN_V2, ffb_cfg, ep_cfg, false, n_probes, width_power);
        print_results("BEST_TIGHTEN_V2", v2_results);

        // ── 3-way comparison ──
        std::cout << "\n═══ 3-WAY COMPARISON TABLE ═══\n";
        std::cout << std::left
                  << std::setw(6) << "Name"
                  << "│ "
                  << std::setw(6) << "RR_D"
                  << std::setw(10) << "RR_Ms"
                  << std::setw(14) << "RR_Vol"
                  << "│ "
                  << std::setw(6) << "BT_D"
                  << std::setw(10) << "BT_Ms"
                  << std::setw(14) << "BT_Vol"
                  << "│ "
                  << std::setw(6) << "V2_D"
                  << std::setw(10) << "V2_Ms"
                  << std::setw(14) << "V2_Vol"
                  << "│ Δ(RR-V2)"
                  << "\n";
        std::cout << std::string(120, '-') << "\n";

        for (size_t i = 0; i < rr_results.size(); ++i) {
            auto& rr = rr_results[i];
            auto& bt = bt_results[i];
            auto& v2 = v2_results[i];
            std::cout << std::left << std::fixed << std::setprecision(3)
                      << std::setw(6) << rr.name
                      << "│ "
                      << std::setw(6) << rr.depth
                      << std::setw(10) << rr.total_ms
                      << std::scientific << std::setprecision(2)
                      << std::setw(14) << rr.volume
                      << std::fixed << std::setprecision(3)
                      << "│ "
                      << std::setw(6) << bt.depth
                      << std::setw(10) << bt.total_ms
                      << std::scientific << std::setprecision(2)
                      << std::setw(14) << bt.volume
                      << std::fixed << std::setprecision(3)
                      << "│ "
                      << std::setw(6) << v2.depth
                      << std::setw(10) << v2.total_ms
                      << std::scientific << std::setprecision(2)
                      << std::setw(14) << v2.volume
                      << std::fixed
                      << "│ " << (rr.depth - v2.depth)
                      << "\n";
        }
    } else {
        // ── Single mode ──
        SplitOrder so = SplitOrder::BEST_TIGHTEN;
        if (split_str == "round_robin" || split_str == "rr")
            so = SplitOrder::ROUND_ROBIN;
        else if (split_str == "widest_first" || split_str == "wf")
            so = SplitOrder::WIDEST_FIRST;
        else if (split_str == "btv2" || split_str == "best_tighten_v2")
            so = SplitOrder::BEST_TIGHTEN_V2;

        std::cout << "Split order: " << split_order_str(so)
                  << "  cache: " << (no_cache ? "OFF" : "ON")
                  << "  n_probes: " << n_probes
                  << "  width_power: " << width_power << "\n";

        auto results = run_ffb_suite(robot, obstacles, so, ffb_cfg, ep_cfg, !no_cache, n_probes, width_power);
        print_results(split_order_str(so), results);
    }

    return 0;
}
