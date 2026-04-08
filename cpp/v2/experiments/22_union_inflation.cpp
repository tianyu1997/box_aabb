// ═══════════════════════════════════════════════════════════════════════════
// Experiment 22 — Grid vs AABB 并集膨胀对比 (CRIT 模式)
// ═══════════════════════════════════════════════════════════════════════════
//
// 目标: 量化 AABB 与 Grid 在 promotion/union 操作中的体积膨胀
//
// 方法:
//   1. 生成随机 C-space 区间作为「父节点」
//   2. 沿某个活动关节切分 → 两个子节点
//   3. 分别对子/父计算 CRIT 帧 → 推导 AABB(n_sub=8) 和 Grid
//   4. Union = AABB: per-slot min(lo)/max(hi);  Grid: bitwise OR
//   5. 膨胀率 = volume(union) / volume(parent_direct)
//        parent_direct = 直接对父区间计算的 envelope (理想紧致值)
//
// 多级 union:
//   depth=1: 2^1=2 children → 1 union
//   depth=2: 2^2=4 children → 3 unions (模拟树 2 层)
//   depth=3: 2^3=8 children → 7 unions (模拟树 3 层)
//
// Build:
//   cmake --build . --target exp_22_union_inflation --config Release
//
// ═══════════════════════════════════════════════════════════════════════════

#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/envelope/envelope_derive_critical.h"
#include "sbf/envelope/grid_store.h"
#include "sbf/envelope/frame_store.h"
#include "sbf/common/config.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace sbf;
using namespace sbf::envelope;
using Clock = std::chrono::high_resolution_clock;

// ─── World bounds ────────────────────────────────────────────────────────
static const float WORLD_BOUNDS[6] = {
    -0.8f, -1.2f, 0.0f,
     1.8f,  1.2f, 1.4f
};

// ─── Robot config finder ─────────────────────────────────────────────────
static std::string find_robot_config(const std::string& name) {
    std::vector<std::string> bases = {
        "../../../safeboxforest/v1/configs",
        "../../safeboxforest/v1/configs",
        "../safeboxforest/v1/configs",
        "../../../../safeboxforest/v1/configs",
    };
    for (auto& b : bases) {
        std::string p = b + "/" + name + ".json";
        if (fs::exists(p)) return p;
    }
    return "";
}

// ─── Width regimes ───────────────────────────────────────────────────────
struct WidthRegime {
    const char* name;
    double lo, hi;
};
static const WidthRegime WIDTHS[] = {
    {"small",  0.05, 0.15},
    {"medium", 0.15, 0.50},
    {"large",  0.50, 1.50},
};
static constexpr int N_WIDTHS = 3;

// ─── Volume helpers ──────────────────────────────────────────────────────
static double aabb_volume(const float* aabbs, int n_slots) {
    double total = 0.0;
    for (int i = 0; i < n_slots; ++i) {
        const float* a = aabbs + i * 6;
        double dx = std::max(0.0, double(a[3] - a[0]));
        double dy = std::max(0.0, double(a[4] - a[1]));
        double dz = std::max(0.0, double(a[5] - a[2]));
        total += dx * dy * dz;
    }
    return total;
}

static double grid_volume_from_count(int occ) {
    double cell_vol = 1.0;
    for (int c = 0; c < 3; ++c) {
        double span = double(WORLD_BOUNDS[3 + c]) - double(WORLD_BOUNDS[c]);
        cell_vol *= span / GRID_R;
    }
    return occ * cell_vol;
}

// ─── AABB union (per-slot min/max) ───────────────────────────────────────
static void aabb_union(const float* a, const float* b, float* out, int n_slots) {
    for (int i = 0; i < n_slots; ++i) {
        int off = i * 6;
        out[off + 0] = std::min(a[off + 0], b[off + 0]);
        out[off + 1] = std::min(a[off + 1], b[off + 1]);
        out[off + 2] = std::min(a[off + 2], b[off + 2]);
        out[off + 3] = std::max(a[off + 3], b[off + 3]);
        out[off + 4] = std::max(a[off + 4], b[off + 4]);
        out[off + 5] = std::max(a[off + 5], b[off + 5]);
    }
}

// ─── Grid occupied count via popcount ────────────────────────────────────
static int count_bits(const uint64_t* grid) {
    int cnt = 0;
    for (int i = 0; i < WORDS_PER_NODE; ++i) {
#if defined(_MSC_VER)
        cnt += static_cast<int>(__popcnt64(grid[i]));
#else
        cnt += __builtin_popcountll(grid[i]);
#endif
    }
    return cnt;
}

static void grid_union_raw(const uint64_t* a, const uint64_t* b, uint64_t* out) {
    for (int i = 0; i < WORDS_PER_NODE; ++i)
        out[i] = a[i] | b[i];
}

// ═══════════════════════════════════════════════════════════════════════════
// Helper: compute CRIT frames for an interval set
// ═══════════════════════════════════════════════════════════════════════════
static void compute_crit_frames_for_interval(
    const Robot& robot,
    const std::vector<Interval>& ivls,
    std::vector<float>& out_frames,
    int n_frames)
{
    out_frames.resize(n_frames * 6);
    derive_crit_frames(robot, ivls, out_frames.data(), nullptr);
}

// ═══════════════════════════════════════════════════════════════════════════
// Helper: derive AABB envelope (n_sub=8) from frames
// ═══════════════════════════════════════════════════════════════════════════
static void derive_full_aabb(
    const Robot& robot,
    const float* frames,
    int n_frames,
    int n_sub,
    std::vector<float>& out_aabbs)
{
    int n_act = robot.n_active_links();
    out_aabbs.resize(n_act * n_sub * 6);

    std::vector<float> radii(n_act, 0.f);
    if (robot.has_link_radii()) {
        const double* rd = robot.active_link_radii();
        for (int i = 0; i < n_act; ++i)
            radii[i] = static_cast<float>(rd[i]);
    }
    float base_pos[3] = {0.f, 0.f, 0.f};

    for (int i = 0; i < n_act; ++i) {
        int frame_idx  = robot.active_link_map()[i];
        int parent_idx = frame_idx - 1;
        derive_aabb_subdivided(
            frames, n_frames,
            parent_idx, frame_idx,
            n_sub, radii[i], base_pos,
            out_aabbs.data() + i * n_sub * 6);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Helper: derive Grid from frames
// ═══════════════════════════════════════════════════════════════════════════
static void derive_full_grid(
    const Robot& robot,
    const float* frames,
    int n_frames,
    int n_sub,
    GridStore& gs,
    int node_idx)
{
    gs.derive_from_frames(node_idx, frames, n_sub);
}

// ═══════════════════════════════════════════════════════════════════════════
// Split interval in half along a joint
// ═══════════════════════════════════════════════════════════════════════════
static void split_interval(
    const std::vector<Interval>& parent,
    int split_joint,
    std::vector<Interval>& left,
    std::vector<Interval>& right)
{
    int n = static_cast<int>(parent.size());
    left = parent;
    right = parent;
    double mid = (parent[split_joint].lo + parent[split_joint].hi) / 2.0;
    left[split_joint].hi = mid;
    right[split_joint].lo = mid;
}

// ─── Result record ───────────────────────────────────────────────────────
struct TrialResult {
    std::string robot;
    std::string width_regime;
    int depth;          // number of binary Split levels
    int trial_idx;

    // Parent direct (ideal)
    double aabb_vol_direct;
    double grid_vol_direct;
    int    grid_occ_direct;

    // Union from children
    double aabb_vol_union;
    double grid_vol_union;
    int    grid_occ_union;

    // Inflation ratios
    double aabb_inflation;  // aabb_vol_union / aabb_vol_direct
    double grid_inflation;  // grid_vol_union / grid_vol_direct

    // Timing (microseconds)
    double crit_derive_us;      // total CRIT frames derive for all leaves
    double aabb_derive_us;      // total AABB derive for all leaves
    double grid_derive_us;      // total Grid derive for all leaves
    double aabb_union_us;       // total AABB union (bottom-up)
    double grid_union_us;       // total Grid union (bottom-up)
};

// ═════════════════════════════════════════════════════════════════════════════
//  Main
// ═════════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv)
{
    // Parse args
    int n_trials = 30;
    std::string output_dir;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--trials" && i + 1 < argc) n_trials = std::stoi(argv[++i]);
        if (a == "--output" && i + 1 < argc) output_dir = argv[++i];
    }

    std::string config_path = find_robot_config("panda");
    if (config_path.empty()) {
        std::cerr << "Robot config not found.\n";
        return 1;
    }
    Robot robot = Robot::from_json(config_path);
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int n_frames = n + (robot.has_tool() ? 1 : 0);
    constexpr int N_SUB = 8;

    // Robot metadata for GridStore
    std::vector<float> radii(n_act, 0.f);
    if (robot.has_link_radii()) {
        const double* rd = robot.active_link_radii();
        for (int i = 0; i < n_act; ++i)
            radii[i] = static_cast<float>(rd[i]);
    }
    float base_pos[3] = {0.f, 0.f, 0.f};

    const std::vector<int> DEPTHS = {1, 2, 3};

    std::vector<TrialResult> all_results;
    std::mt19937 rng(42);

    std::cout << "================================================================\n";
    std::cout << "  Experiment 22: Grid vs AABB Union Inflation (CRIT)\n";
    std::cout << "================================================================\n";
    std::cout << "Robot: panda  Joints: " << n << "  Active links: " << n_act << "\n";
    std::cout << "N_SUB: " << N_SUB << "  Grid R: " << GRID_R << "\n";
    std::cout << "Trials per config: " << n_trials << "\n";
    std::cout << "Depths: 1, 2, 3\n";
    std::cout << "Width regimes: small, medium, large\n\n";

    int total_configs = N_WIDTHS * static_cast<int>(DEPTHS.size());
    int config_idx = 0;

    for (int wi = 0; wi < N_WIDTHS; ++wi) {
        for (int depth : DEPTHS) {
            config_idx++;
            std::cout << "[" << config_idx << "/" << total_configs << "] "
                      << WIDTHS[wi].name << " depth=" << depth << " ...\n";

            int n_leaves = 1 << depth;  // 2^depth leaves

            for (int trial = 0; trial < n_trials; ++trial) {

                // Generate random parent interval
                std::vector<Interval> parent_ivls(n);
                for (int j = 0; j < n; ++j) {
                    double lo_j = robot.joint_limits().limits[j].lo;
                    double hi_j = robot.joint_limits().limits[j].hi;
                    double range = hi_j - lo_j;
                    double width = std::uniform_real_distribution<double>(
                        WIDTHS[wi].lo, WIDTHS[wi].hi)(rng) * range;
                    double center = std::uniform_real_distribution<double>(
                        lo_j + width / 2, hi_j - width / 2)(rng);
                    parent_ivls[j].lo = center - width / 2;
                    parent_ivls[j].hi = center + width / 2;
                }

                // ─── Parent direct envelope (ground truth) ───────────
                std::vector<float> parent_frames;
                compute_crit_frames_for_interval(robot, parent_ivls, parent_frames, n_frames);

                std::vector<float> parent_aabbs;
                derive_full_aabb(robot, parent_frames.data(), n_frames, N_SUB, parent_aabbs);
                double parent_aabb_vol = aabb_volume(parent_aabbs.data(), n_act * N_SUB);

                // GridStore: allocate node 0 for parent
                int total_grid_nodes = 1 + n_leaves + (n_leaves - 1);  // parent + leaves + intermediates
                GridStore gs(n_frames, n_act,
                             robot.active_link_map(), radii.data(),
                             base_pos, WORLD_BOUNDS, total_grid_nodes + 4);
                derive_full_grid(robot, parent_frames.data(), n_frames, N_SUB, gs, 0);
                int parent_grid_occ = gs.occupied_count(0);
                double parent_grid_vol = grid_volume_from_count(parent_grid_occ);

                // ─── Build leaf intervals by recursive binary split ──
                // Split along cycling joints 0,1,2,... using active links
                std::vector<std::vector<Interval>> leaves = {parent_ivls};
                for (int d = 0; d < depth; ++d) {
                    int split_joint = d % n;
                    std::vector<std::vector<Interval>> new_leaves;
                    for (auto& leaf : leaves) {
                        std::vector<Interval> left, right;
                        split_interval(leaf, split_joint, left, right);
                        new_leaves.push_back(std::move(left));
                        new_leaves.push_back(std::move(right));
                    }
                    leaves = std::move(new_leaves);
                }
                // Now leaves.size() == n_leaves

                // ─── Compute envelope for each leaf (timed) ─────────
                int aabb_slots = n_act * N_SUB;
                std::vector<std::vector<float>> leaf_aabbs(n_leaves);

                double crit_derive_us = 0, aabb_derive_us = 0, grid_derive_us = 0;

                for (int li = 0; li < n_leaves; ++li) {
                    std::vector<float> frames;

                    auto t0 = Clock::now();
                    compute_crit_frames_for_interval(robot, leaves[li], frames, n_frames);
                    auto t1 = Clock::now();
                    crit_derive_us += std::chrono::duration<double, std::micro>(t1 - t0).count();

                    auto t2 = Clock::now();
                    derive_full_aabb(robot, frames.data(), n_frames, N_SUB, leaf_aabbs[li]);
                    auto t3 = Clock::now();
                    aabb_derive_us += std::chrono::duration<double, std::micro>(t3 - t2).count();

                    auto t4 = Clock::now();
                    derive_full_grid(robot, frames.data(), n_frames, N_SUB, gs, 1 + li);
                    auto t5 = Clock::now();
                    grid_derive_us += std::chrono::duration<double, std::micro>(t5 - t4).count();
                }

                // ─── Bottom-up union (timed) ─────────────────────────
                // AABB: merge pairs bottom-up
                auto tu_aabb_0 = Clock::now();
                std::vector<std::vector<float>> level = leaf_aabbs;
                while (level.size() > 1) {
                    std::vector<std::vector<float>> next_level;
                    for (size_t i = 0; i < level.size(); i += 2) {
                        std::vector<float> merged(aabb_slots * 6);
                        aabb_union(level[i].data(), level[i + 1].data(),
                                   merged.data(), aabb_slots);
                        next_level.push_back(std::move(merged));
                    }
                    level = std::move(next_level);
                }
                auto tu_aabb_1 = Clock::now();
                double aabb_union_us = std::chrono::duration<double, std::micro>(tu_aabb_1 - tu_aabb_0).count();
                double union_aabb_vol = aabb_volume(level[0].data(), aabb_slots);

                // Grid: merge pairs bottom-up
                auto tu_grid_0 = Clock::now();
                int next_grid_idx = 1 + n_leaves;
                std::vector<int> grid_level;
                for (int li = 0; li < n_leaves; ++li) grid_level.push_back(1 + li);

                while (grid_level.size() > 1) {
                    std::vector<int> next_grid_level;
                    for (size_t i = 0; i < grid_level.size(); i += 2) {
                        int dst = next_grid_idx++;
                        gs.union_grids(dst, grid_level[i], grid_level[i + 1]);
                        next_grid_level.push_back(dst);
                    }
                    grid_level = std::move(next_grid_level);
                }
                auto tu_grid_1 = Clock::now();
                double grid_union_us = std::chrono::duration<double, std::micro>(tu_grid_1 - tu_grid_0).count();
                int union_grid_occ = gs.occupied_count(grid_level[0]);
                double union_grid_vol = grid_volume_from_count(union_grid_occ);

                // ─── Record ──────────────────────────────────────────
                TrialResult r;
                r.robot = "panda";
                r.width_regime = WIDTHS[wi].name;
                r.depth = depth;
                r.trial_idx = trial;
                r.aabb_vol_direct = parent_aabb_vol;
                r.grid_vol_direct = parent_grid_vol;
                r.grid_occ_direct = parent_grid_occ;
                r.aabb_vol_union = union_aabb_vol;
                r.grid_vol_union = union_grid_vol;
                r.grid_occ_union = union_grid_occ;
                r.aabb_inflation = (parent_aabb_vol > 0) ? union_aabb_vol / parent_aabb_vol : 1.0;
                r.grid_inflation = (parent_grid_vol > 0) ? union_grid_vol / parent_grid_vol : 1.0;
                r.crit_derive_us = crit_derive_us;
                r.aabb_derive_us = aabb_derive_us;
                r.grid_derive_us = grid_derive_us;
                r.aabb_union_us = aabb_union_us;
                r.grid_union_us = grid_union_us;
                all_results.push_back(r);
            }
        }
    }

    // ─── Output ──────────────────────────────────────────────────────────
    if (output_dir.empty()) {
        // Timestamp
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        struct tm lt = {};
#if defined(_MSC_VER)
        localtime_s(&lt, &t);
#else
        localtime_r(&t, &lt);
#endif
        char ts[64];
        std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", &lt);
        output_dir = std::string("../results/exp22_union_inflation_") + ts;
    }
    fs::create_directories(output_dir);

    // CSV
    std::string csv_path = output_dir + "/results.csv";
    std::ofstream csv(csv_path);
    csv << "robot,width,depth,trial,"
        << "aabb_vol_direct,grid_vol_direct,grid_occ_direct,"
        << "aabb_vol_union,grid_vol_union,grid_occ_union,"
        << "aabb_inflation,grid_inflation,"
        << "crit_derive_us,aabb_derive_us,grid_derive_us,"
        << "aabb_union_us,grid_union_us\n";
    for (auto& r : all_results) {
        csv << r.robot << "," << r.width_regime << "," << r.depth << "," << r.trial_idx << ","
            << std::scientific << std::setprecision(8)
            << r.aabb_vol_direct << "," << r.grid_vol_direct << "," << r.grid_occ_direct << ","
            << r.aabb_vol_union << "," << r.grid_vol_union << "," << r.grid_occ_union << ","
            << std::fixed << std::setprecision(6)
            << r.aabb_inflation << "," << r.grid_inflation << ","
            << std::fixed << std::setprecision(2)
            << r.crit_derive_us << "," << r.aabb_derive_us << "," << r.grid_derive_us << ","
            << r.aabb_union_us << "," << r.grid_union_us << "\n";
    }
    csv.close();
    std::cout << "\nCSV written to: " << csv_path << "\n";

    // ─── Summary report ──────────────────────────────────────────────────
    std::string report_path = output_dir + "/report.md";
    std::ofstream rpt(report_path);
    rpt << "# Experiment 22: Grid vs AABB Union Inflation (CRIT)\n\n";
    rpt << "Robot: panda | N_SUB: " << N_SUB << " | Grid R: " << GRID_R
        << " | Trials: " << n_trials << "\n\n";

    // Aggregate by (width, depth)
    rpt << "## Average Inflation Ratio\n\n";
    rpt << "| Width | Depth | AABB Inflation | Grid Inflation | AABB/Grid Ratio |\n";
    rpt << "|-------|-------|----------------|----------------|------------------|\n";

    for (int wi = 0; wi < N_WIDTHS; ++wi) {
        for (int depth : DEPTHS) {
            double aabb_sum = 0, grid_sum = 0;
            int cnt = 0;
            for (auto& r : all_results) {
                if (r.width_regime == WIDTHS[wi].name && r.depth == depth) {
                    aabb_sum += r.aabb_inflation;
                    grid_sum += r.grid_inflation;
                    cnt++;
                }
            }
            double aabb_avg = aabb_sum / cnt;
            double grid_avg = grid_sum / cnt;
            rpt << "| " << WIDTHS[wi].name
                << " | " << depth
                << " | " << std::fixed << std::setprecision(4) << aabb_avg << "x"
                << " | " << std::fixed << std::setprecision(4) << grid_avg << "x"
                << " | " << std::fixed << std::setprecision(2) << aabb_avg / grid_avg << "x"
                << " |\n";
        }
    }

    // Detail: percentile inflation
    rpt << "\n## Inflation Distribution (Percentiles)\n\n";
    rpt << "| Width | Depth | Type | P25 | P50 (Median) | P75 | P95 | Max |\n";
    rpt << "|-------|-------|------|-----|--------------|-----|-----|-----|\n";

    for (int wi = 0; wi < N_WIDTHS; ++wi) {
        for (int depth : DEPTHS) {
            std::vector<double> aabb_infl, grid_infl;
            for (auto& r : all_results) {
                if (r.width_regime == WIDTHS[wi].name && r.depth == depth) {
                    aabb_infl.push_back(r.aabb_inflation);
                    grid_infl.push_back(r.grid_inflation);
                }
            }
            auto percentile = [](std::vector<double>& v, double p) {
                std::sort(v.begin(), v.end());
                int idx = std::min(int(p * v.size()), int(v.size()) - 1);
                return v[idx];
            };
            rpt << "| " << WIDTHS[wi].name << " | " << depth << " | AABB"
                << " | " << std::fixed << std::setprecision(3) << percentile(aabb_infl, 0.25) << "x"
                << " | " << percentile(aabb_infl, 0.50) << "x"
                << " | " << percentile(aabb_infl, 0.75) << "x"
                << " | " << percentile(aabb_infl, 0.95) << "x"
                << " | " << percentile(aabb_infl, 1.0) << "x"
                << " |\n";
            rpt << "| " << WIDTHS[wi].name << " | " << depth << " | Grid"
                << " | " << std::fixed << std::setprecision(3) << percentile(grid_infl, 0.25) << "x"
                << " | " << percentile(grid_infl, 0.50) << "x"
                << " | " << percentile(grid_infl, 0.75) << "x"
                << " | " << percentile(grid_infl, 0.95) << "x"
                << " | " << percentile(grid_infl, 1.0) << "x"
                << " |\n";
        }
    }

    // Volume comparison section
    rpt << "\n## Volume Comparison (Parent Direct)\n\n";
    rpt << "| Width | Depth | AABB Vol (avg) | Grid Vol (avg) | Grid/AABB |\n";
    rpt << "|-------|-------|----------------|----------------|───────────|\n";

    for (int wi = 0; wi < N_WIDTHS; ++wi) {
        for (int depth : DEPTHS) {
            double aabb_sum = 0, grid_sum = 0;
            int cnt = 0;
            for (auto& r : all_results) {
                if (r.width_regime == WIDTHS[wi].name && r.depth == depth) {
                    aabb_sum += r.aabb_vol_direct;
                    grid_sum += r.grid_vol_direct;
                    cnt++;
                }
            }
            rpt << "| " << WIDTHS[wi].name << " | " << depth
                << " | " << std::scientific << std::setprecision(4) << aabb_sum / cnt
                << " | " << grid_sum / cnt
                << " | " << std::fixed << std::setprecision(2)
                << (grid_sum / aabb_sum) * 100 << "%"
                << " |\n";
        }
    }

    // Timing section — component breakdown
    rpt << "\n## Timing Breakdown (microseconds, average per trial)\n\n";
    rpt << "| Width | Depth | CRIT Frames | AABB Derive | Grid Derive | AABB Union | Grid Union |\n";
    rpt << "|-------|-------|-------------|-------------|-------------|------------|------------|\n";

    for (int wi = 0; wi < N_WIDTHS; ++wi) {
        for (int depth : DEPTHS) {
            double crit_sum=0, aabb_d_sum=0, grid_d_sum=0, aabb_u_sum=0, grid_u_sum=0;
            int cnt = 0;
            for (auto& r : all_results) {
                if (r.width_regime == WIDTHS[wi].name && r.depth == depth) {
                    crit_sum  += r.crit_derive_us;
                    aabb_d_sum += r.aabb_derive_us;
                    grid_d_sum += r.grid_derive_us;
                    aabb_u_sum += r.aabb_union_us;
                    grid_u_sum += r.grid_union_us;
                    cnt++;
                }
            }
            rpt << "| " << WIDTHS[wi].name << " | " << depth
                << " | " << std::fixed << std::setprecision(1) << crit_sum / cnt
                << " | " << aabb_d_sum / cnt
                << " | " << grid_d_sum / cnt
                << " | " << aabb_u_sum / cnt
                << " | " << grid_u_sum / cnt
                << " |\n";
        }
    }

    // Cold derive = CRIT frames + envelope derive (full cost from scratch per leaf set)
    rpt << "\n## Cold Derive (CRIT + Envelope, avg us per trial)\n\n";
    rpt << "AABB cold = CRIT frames + AABB derive; Grid cold = CRIT frames + Grid derive\n\n";
    rpt << "| Width | Depth | AABB Cold | Grid Cold | Grid/AABB | Grid Overhead |\n";
    rpt << "|-------|-------|-----------|-----------|-----------|---------------|\n";

    for (int wi = 0; wi < N_WIDTHS; ++wi) {
        for (int depth : DEPTHS) {
            double crit_sum=0, aabb_d_sum=0, grid_d_sum=0;
            int cnt = 0;
            for (auto& r : all_results) {
                if (r.width_regime == WIDTHS[wi].name && r.depth == depth) {
                    crit_sum  += r.crit_derive_us;
                    aabb_d_sum += r.aabb_derive_us;
                    grid_d_sum += r.grid_derive_us;
                    cnt++;
                }
            }
            double aabb_cold = (crit_sum + aabb_d_sum) / cnt;
            double grid_cold = (crit_sum + grid_d_sum) / cnt;
            double overhead_pct = (grid_d_sum - aabb_d_sum) / (crit_sum + aabb_d_sum) * 100;
            rpt << "| " << WIDTHS[wi].name << " | " << depth
                << " | " << std::fixed << std::setprecision(1) << aabb_cold
                << " | " << grid_cold
                << " | " << std::setprecision(2) << grid_cold / aabb_cold << "x"
                << " | +" << std::setprecision(1) << overhead_pct << "%"
                << " |\n";
        }
    }

    // Full pipeline cost = cold derive + union
    rpt << "\n## Full Pipeline (cold derive + union, avg us per trial)\n\n";
    rpt << "AABB full = CRIT + AABB derive + AABB union; Grid full = CRIT + Grid derive + Grid union\n\n";
    rpt << "| Width | Depth | AABB Full | Grid Full | Grid/AABB | AABB Inflation | Grid Inflation | Grid Vol/AABB Vol |\n";
    rpt << "|-------|-------|-----------|-----------|-----------|----------------|----------------|-------------------|\n";

    for (int wi = 0; wi < N_WIDTHS; ++wi) {
        for (int depth : DEPTHS) {
            double crit_s=0, aabb_d=0, grid_d=0, aabb_u=0, grid_u=0;
            double aabb_infl=0, grid_infl=0, aabb_vol_s=0, grid_vol_s=0;
            int cnt = 0;
            for (auto& r : all_results) {
                if (r.width_regime == WIDTHS[wi].name && r.depth == depth) {
                    crit_s    += r.crit_derive_us;
                    aabb_d    += r.aabb_derive_us;
                    grid_d    += r.grid_derive_us;
                    aabb_u    += r.aabb_union_us;
                    grid_u    += r.grid_union_us;
                    aabb_infl += r.aabb_inflation;
                    grid_infl += r.grid_inflation;
                    aabb_vol_s += r.aabb_vol_direct;
                    grid_vol_s += r.grid_vol_direct;
                    cnt++;
                }
            }
            double aabb_full = (crit_s + aabb_d + aabb_u) / cnt;
            double grid_full = (crit_s + grid_d + grid_u) / cnt;
            rpt << "| " << WIDTHS[wi].name << " | " << depth
                << " | " << std::fixed << std::setprecision(1) << aabb_full
                << " | " << grid_full
                << " | " << std::setprecision(2) << grid_full / aabb_full << "x"
                << " | " << std::setprecision(4) << aabb_infl / cnt << "x"
                << " | " << grid_infl / cnt << "x"
                << " | " << std::setprecision(1) << (grid_vol_s / aabb_vol_s) * 100 << "%"
                << " |\n";
        }
    }

    rpt.close();
    std::cout << "Report written to: " << report_path << "\n";

    // Console summary
    std::cout << "\n─── Summary (Cold Derive + Inflation) ───\n";
    std::cout << std::fixed;
    for (int wi = 0; wi < N_WIDTHS; ++wi) {
        for (int depth : DEPTHS) {
            double aabb_inf_s=0, grid_inf_s=0;
            double crit_s=0, aabb_d=0, grid_d=0, aabb_u=0, grid_u=0;
            int cnt = 0;
            for (auto& r : all_results) {
                if (r.width_regime == WIDTHS[wi].name && r.depth == depth) {
                    aabb_inf_s += r.aabb_inflation;
                    grid_inf_s += r.grid_inflation;
                    crit_s += r.crit_derive_us;
                    aabb_d += r.aabb_derive_us;
                    grid_d += r.grid_derive_us;
                    aabb_u += r.aabb_union_us;
                    grid_u += r.grid_union_us;
                    cnt++;
                }
            }
            double aabb_cold = (crit_s + aabb_d) / cnt;
            double grid_cold = (crit_s + grid_d) / cnt;
            double aabb_full = (crit_s + aabb_d + aabb_u) / cnt;
            double grid_full = (crit_s + grid_d + grid_u) / cnt;
            std::cout << WIDTHS[wi].name << " d=" << depth
                      << std::setprecision(3)
                      << "  infl: AABB=" << aabb_inf_s / cnt << "x"
                      << " Grid=" << grid_inf_s / cnt << "x"
                      << std::setprecision(1)
                      << "  cold(us): AABB=" << aabb_cold
                      << " Grid=" << grid_cold
                      << " (" << std::setprecision(2) << grid_cold / aabb_cold << "x)"
                      << std::setprecision(1)
                      << "  full(us): AABB=" << aabb_full
                      << " Grid=" << grid_full
                      << "\n";
        }
    }
    std::cout << "\nTotal trials: " << all_results.size() << "\n";

    return 0;
}
