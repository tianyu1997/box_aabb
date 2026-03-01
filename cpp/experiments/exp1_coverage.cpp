/**
 * exp1_coverage.cpp — 实验 1: Region 生成效率与自由空间覆盖 (build_multi)
 *
 * 设计:
 *   1 个合并场景 (shelves + bins + table = 16 obstacles)
 *   5 个 s-t pairs: AS→TS, TS→CS, CS→LB, LB→RB, RB→AS
 *   SBF: build_multi (2000 boxes/pair + 5000 random)
 *   N_seeds 次重复, 测量: n_boxes, total_volume, build_time, coverage_rate (MC)
 *
 * 用法:
 *   ./exp1_coverage [robot.json] [--seeds N] [--max-boxes N] [--mc N] [--quick]
 */

#include "sbf/planner/sbf_planner.h"
#include "sbf/planner/pipeline.h"
#include "sbf/io/json_io.h"
#include "marcucci_scenes.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <vector>

using namespace sbf;

// ═══════════════════════════════════════════════════════════════════════════
// Monte Carlo coverage estimation
// ═══════════════════════════════════════════════════════════════════════════

struct CoverageResult {
    double coverage_rate;
    int n_free, n_covered, n_total;
};

CoverageResult estimate_coverage_mc(
    const Robot& robot,
    const CollisionChecker& checker,
    const SafeBoxForest& forest,
    int n_samples, int mc_seed)
{
    std::mt19937 rng(mc_seed);
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    int ndim = robot.n_joints();
    const auto& jl = robot.joint_limits();

    int n_free = 0, n_covered = 0;
    Eigen::VectorXd q(ndim);
    for (int s = 0; s < n_samples; ++s) {
        for (int d = 0; d < ndim; ++d)
            q[d] = jl.limits[d].lo + unif(rng) * jl.limits[d].width();
        if (!checker.check_config(q)) {
            n_free++;
            if (forest.find_containing(q))
                n_covered++;
        }
    }
    return {n_free > 0 ? double(n_covered) / n_free : 0.0,
            n_free, n_covered, n_samples};
}

// ═══════════════════════════════════════════════════════════════════════════
// Statistics
// ═══════════════════════════════════════════════════════════════════════════

struct Stats { double median, q25, q75, mean; };

Stats compute_stats(std::vector<double>& data) {
    Stats s{};
    if (data.empty()) return s;
    std::sort(data.begin(), data.end());
    int n = (int)data.size();
    s.median = data[n / 2];
    s.q25 = data[n / 4];
    s.q75 = data[3 * n / 4];
    s.mean = std::accumulate(data.begin(), data.end(), 0.0) / n;
    return s;
}

// ═══════════════════════════════════════════════════════════════════════════
// Main
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// Preset configurations
// ═══════════════════════════════════════════════════════════════════════════

struct Preset {
    std::string name;
    int max_boxes;
    int n_random;
    int max_consecutive_miss;
    double ffb_min_edge;
    double ffb_min_edge_relaxed;
    std::vector<double> phase_k;
    std::vector<int> phase_budget;
    int max_boxes_per_pair;
    int coarsen_target_boxes;     // 0 = no greedy coarsen
    bool grid_check;              // grid-based hull safety check
};

// A: 最快 — 保持连通性, 粗粒度 box, 少量生成
Preset preset_A() {
    return {"A-fast", 10000, 0, 500,
            0.02, 0.08,
            {5.0, 2.0, 1.0, 0.5},
            {300, 500, 800, 1500},
            3000, 0, false};
}

// B: 基线 — 当前默认配置
Preset preset_B() {
    return {"B-baseline", 10000, 0, 500,
            0.01, 0.05,
            {5.0, 2.0, 1.0, 0.5, 0.1},
            {500, 1000, 1000, 2000, 2000},
            5000, 0, false};
}

// C: 高覆盖 — 更多 box, 更多粗阶段预算, random 补填
Preset preset_C() {
    return {"C-coverage", 30000, 5000, 800,
            0.01, 0.04,
            {5.0, 3.0, 2.0, 1.0, 0.5, 0.1},
            {1000, 1500, 2000, 2000, 2000, 2000},
            5000, 0, false};
}

// D: 极致覆盖 — 大量 box + 大量 random
Preset preset_D() {
    return {"D-maxcov", 50000, 10000, 1000,
            0.01, 0.03,
            {5.0, 3.0, 2.0, 1.0, 0.5, 0.1},
            {1500, 2000, 3000, 3000, 3000, 3000},
            8000, 0, false};
}

// E: 快速+少量random辅助
Preset preset_E() {
    return {"E-fast+rand", 10000, 2000, 500,
            0.02, 0.06,
            {5.0, 2.0, 1.0, 0.5},
            {400, 600, 1000, 2000},
            3000, 0, false};
}

// F: 中等 — 粗阶段加大预算
Preset preset_F() {
    return {"F-balanced", 15000, 2000, 600,
            0.015, 0.05,
            {5.0, 3.0, 2.0, 1.0, 0.5},
            {800, 1200, 1500, 2000, 2000},
            5000, 0, false};
}

// G: E + greedy coarsen → 500 boxes
Preset preset_G() {
    return {"G-E+coarsen500", 10000, 2000, 500,
            0.02, 0.06,
            {5.0, 2.0, 1.0, 0.5},
            {400, 600, 1000, 2000},
            3000, 500, false};
}

// H: C + greedy coarsen → 500 boxes
Preset preset_H() {
    return {"H-C+coarsen500", 30000, 5000, 800,
            0.01, 0.04,
            {5.0, 3.0, 2.0, 1.0, 0.5, 0.1},
            {1000, 1500, 2000, 2000, 2000, 2000},
            5000, 500, false};
}

// I: E + greedy coarsen → 1000 boxes
Preset preset_I() {
    return {"I-E+coarsen1k", 10000, 2000, 500,
            0.02, 0.06,
            {5.0, 2.0, 1.0, 0.5},
            {400, 600, 1000, 2000},
            3000, 1000, false};
}

// J: 少box从源头控制 — 100/pair, min_edge=0.05, + coarsen→200
Preset preset_J() {
    return {"J-sparse200", 1000, 200, 300,
            0.05, 0.15,
            {5.0, 2.0, 1.0},
            {50, 100, 200},
            200, 200, false};
}

// K: 中等稀疏 — 200/pair, + coarsen→500
Preset preset_K() {
    return {"K-sparse500", 2000, 500, 400,
            0.04, 0.10,
            {5.0, 2.0, 1.0, 0.5},
            {100, 200, 300, 500},
            400, 500, false};
}

// L: 保留较多 — 500/pair, + coarsen→1000
Preset preset_L() {
    return {"L-sparse1k", 5000, 1000, 500,
            0.03, 0.08,
            {5.0, 2.0, 1.0, 0.5},
            {200, 400, 600, 1000},
            1000, 1000, false};
}

// M: 极少 — 50/pair, 大box, + coarsen→100
Preset preset_M() {
    return {"M-ultra100", 500, 100, 200,
            0.08, 0.20,
            {5.0, 2.0},
            {30, 60},
            100, 100, false};
}

// N: 200 boxes 无random — 纯s-t pair 连通
Preset preset_N() {
    return {"N-pure200", 500, 0, 300,
            0.06, 0.15,
            {5.0, 2.0, 1.0},
            {30, 60, 100},
            100, 100, false};
}

// J2: J-sparse + grid_check (tighter hull validation)
Preset preset_J2() {
    return {"J2-grid200", 1000, 200, 300,
            0.05, 0.15,
            {5.0, 2.0, 1.0},
            {50, 100, 200},
            200, 200, true};
}

// J3: E + grid coarsen → 500
Preset preset_J3() {
    return {"J3-E+grid500", 10000, 2000, 500,
            0.02, 0.06,
            {5.0, 2.0, 1.0, 0.5},
            {400, 600, 1000, 2000},
            3000, 500, true};
}

// J4: C + grid coarsen → 500
Preset preset_J4() {
    return {"J4-C+grid500", 30000, 5000, 800,
            0.01, 0.04,
            {5.0, 3.0, 2.0, 1.0, 0.5, 0.1},
            {1000, 1500, 2000, 2000, 2000, 2000},
            5000, 500, true};
}

void run_preset(const Preset& p, const Robot& robot,
                const std::vector<Obstacle>& obstacles,
                const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& pairs,
                int n_seeds, int n_mc,
                const std::string& cache_dir = "") {
    std::cout << "\n╔════════════════════════════════════════════════╗\n"
              << "║  Preset: " << std::left << std::setw(38) << p.name << "║\n"
              << "╚════════════════════════════════════════════════╝\n"
              << "  max_boxes=" << p.max_boxes
              << "  random=" << p.n_random
              << "  min_edge=" << p.ffb_min_edge
              << "  relaxed=" << p.ffb_min_edge_relaxed
              << "  max_per_pair=" << p.max_boxes_per_pair
              << "  coarsen_target=" << p.coarsen_target_boxes
              << "  grid_check=" << (p.grid_check ? "ON" : "off") << "\n"
              << "  phases: k=[";
    for (size_t i = 0; i < p.phase_k.size(); ++i)
        std::cout << (i ? "," : "") << p.phase_k[i];
    std::cout << "]  budget=[";
    for (size_t i = 0; i < p.phase_budget.size(); ++i)
        std::cout << (i ? "," : "") << p.phase_budget[i];
    std::cout << "]\n\n";

    std::vector<double> v_reg, v_cov, v_time, v_vol;

    for (int seed = 0; seed < n_seeds; ++seed) {
        SBFConfig cfg = make_panda_config(seed);
        cfg.max_boxes = p.max_boxes;
        cfg.max_consecutive_miss = p.max_consecutive_miss;
        cfg.ffb_min_edge = p.ffb_min_edge;
        cfg.ffb_min_edge_relaxed = p.ffb_min_edge_relaxed;
        cfg.bfs_phase_k = p.phase_k;
        cfg.bfs_phase_budget = p.phase_budget;
        cfg.max_boxes_per_pair = p.max_boxes_per_pair;
        cfg.coarsen_target_boxes = p.coarsen_target_boxes;
        cfg.coarsen_grid_check = p.grid_check;
        cfg.coarsen_split_depth = 1;  // max extra splits for tree-cached check
        cfg.coarsen_max_tree_fk = 2000;  // FK budget per coarsen round

        // Disk cache
        if (!cache_dir.empty()) {
            cfg.use_cache = true;
            cfg.cache_path = cache_dir + "/" + p.name + "_seed" + std::to_string(seed) + ".hcache";
        }

        auto t0 = std::chrono::steady_clock::now();
        SBFPlanner planner(robot, obstacles, cfg);
        planner.build_multi(pairs, p.n_random, 300.0);
        double wall = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();

        int nreg = planner.forest().n_boxes();
        double vol = planner.forest().total_volume();

        int mc_seed = seed + 1000000;
        auto cov = estimate_coverage_mc(
            robot, planner.collision_checker(),
            planner.forest(), n_mc, mc_seed);

        v_reg.push_back(nreg);
        v_cov.push_back(cov.coverage_rate * 100.0);
        v_time.push_back(wall);
        v_vol.push_back(vol);

        std::cout << "    s" << std::setw(2) << seed
                  << "  boxes=" << std::setw(6) << nreg
                  << "  vol=" << std::setw(8) << std::setprecision(3) << vol
                  << "  cov=" << std::setw(5) << std::setprecision(1) << cov.coverage_rate * 100 << "%"
                  << "  t=" << std::setprecision(2) << wall << "s"
                  << "  free=" << std::setprecision(1) << 100.0 * cov.n_free / cov.n_total << "%"
                  << "\n";
    }

    auto sr = compute_stats(v_reg);
    auto sc = compute_stats(v_cov);
    auto st = compute_stats(v_time);
    auto sv = compute_stats(v_vol);
    std::cout << "\n  ── " << p.name << " ──\n"
              << "    Boxes:    med=" << std::setprecision(0) << sr.median
              << "  mean=" << std::setprecision(1) << sr.mean
              << "  [" << sr.q25 << "," << sr.q75 << "]\n"
              << "    Volume:   med=" << std::setprecision(3) << sv.median
              << "  mean=" << sv.mean << "\n"
              << "    Coverage: med=" << std::setprecision(1) << sc.median << "%"
              << "  mean=" << sc.mean << "%"
              << "  [" << sc.q25 << "," << sc.q75 << "]\n"
              << "    Time:     med=" << std::setprecision(3) << st.median << "s"
              << "  mean=" << st.mean << "s\n";
}

int main(int argc, char** argv) {
    std::string robot_path = "configs/iiwa14.json";
    int n_seeds = 10;
    int n_mc = 100000;
    bool quick = false;
    std::string preset_name = "all";  // "all" or "A"/"B"/.../"F"
    std::string cache_dir = "";  // if non-empty, enable disk cache
    // Legacy single-run params
    int max_boxes = 10000;
    int n_random = 0;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seeds" && i+1 < argc) n_seeds = std::atoi(argv[++i]);
        else if (a == "--mc" && i+1 < argc) n_mc = std::atoi(argv[++i]);
        else if (a == "--max-boxes" && i+1 < argc) max_boxes = std::atoi(argv[++i]);
        else if (a == "--random" && i+1 < argc) n_random = std::atoi(argv[++i]);
        else if (a == "--preset" && i+1 < argc) preset_name = argv[++i];
        else if (a == "--cache-dir" && i+1 < argc) cache_dir = argv[++i];
        else if (a == "--quick") quick = true;
        else if (a[0] != '-') robot_path = a;
    }

    if (quick) { n_seeds = 3; n_mc = 50000; }

    Robot robot = Robot::from_json(robot_path);
    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints()
              << "  active_links=" << robot.n_active_links() << "\n";

    auto obstacles = make_combined_obstacles();
    auto queries = make_combined_queries();
    std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> pairs;
    for (auto& qp : queries)
        pairs.push_back({qp.start, qp.goal});

    std::cout << "\n  Scene: combined  (" << obstacles.size() << " obstacles, "
              << pairs.size() << " pairs)\n"
              << "  seeds=" << n_seeds << "  MC=" << n_mc
              << "  preset=" << preset_name
              << "  cache=" << (cache_dir.empty() ? "OFF" : cache_dir) << "\n";

    std::cout << std::fixed;

    // Collect presets to run
    std::vector<Preset> presets;
    if (preset_name == "all" || preset_name == "sweep") {
        presets = {preset_A(), preset_B(), preset_C(), preset_D(), preset_E(), preset_F()};
    } else if (preset_name == "A") { presets = {preset_A()}; }
    else if (preset_name == "B") { presets = {preset_B()}; }
    else if (preset_name == "C") { presets = {preset_C()}; }
    else if (preset_name == "D") { presets = {preset_D()}; }
    else if (preset_name == "E") { presets = {preset_E()}; }
    else if (preset_name == "F") { presets = {preset_F()}; }
    else if (preset_name == "G") { presets = {preset_G()}; }
    else if (preset_name == "H") { presets = {preset_H()}; }
    else if (preset_name == "I") { presets = {preset_I()}; }
    else if (preset_name == "J") { presets = {preset_J()}; }
    else if (preset_name == "K") { presets = {preset_K()}; }
    else if (preset_name == "L") { presets = {preset_L()}; }
    else if (preset_name == "M") { presets = {preset_M()}; }
    else if (preset_name == "N") { presets = {preset_N()}; }
    else if (preset_name == "J2") { presets = {preset_J2()}; }
    else if (preset_name == "J3") { presets = {preset_J3()}; }
    else if (preset_name == "J4") { presets = {preset_J4()}; }
    else if (preset_name == "coarsen") {
        presets = {preset_G(), preset_H(), preset_I()};
    } else if (preset_name == "sparse") {
        presets = {preset_J(), preset_K(), preset_L(), preset_M(), preset_N()};
    } else if (preset_name == "grid") {
        presets = {preset_J(), preset_J2(), preset_J3(), preset_J4()};
    } else {
        // Legacy mode: use CLI params with default config
        Preset legacy{"legacy", max_boxes, n_random, 500,
                       0.01, 0.05,
                       {5.0, 2.0, 1.0, 0.5, 0.1},
                       {500, 1000, 1000, 2000, 2000},
                       5000, 0, false};
        presets = {legacy};
    }

    // Create cache directory if needed
    if (!cache_dir.empty()) {
        std::string mkdir_cmd = "mkdir -p " + cache_dir;
        std::system(mkdir_cmd.c_str());
    }

    for (auto& p : presets) {
        run_preset(p, robot, obstacles, pairs, n_seeds, n_mc, cache_dir);
    }

    if (presets.size() > 1) {
        std::cout << "\n\n╔════════════════════════════════════════════════════════════════════╗\n"
                  << "║  SWEEP COMPLETE — all " << presets.size() << " presets done"
                  << std::string(39 - std::to_string(presets.size()).size(), ' ') << "║\n"
                  << "╚════════════════════════════════════════════════════════════════════╝\n";
    }

    std::cout << "\n  Exp 1 complete.\n";
    return 0;
}
