// ═══════════════════════════════════════════════════════════════════════════
// Experiment 24 — 极值点关节构型特征分析
// ═══════════════════════════════════════════════════════════════════════════
//
// 以 Full Analytical (Phase 0+1+2+3) 运行后，导出每个极值面方向对应的
// 最优关节构型 q*，分析其结构特征：
//
//   1. 各关节是否在边界 (lo/hi) 上
//   2. 各关节是否在 kπ/2 处
//   3. 关节子集之和是否为 kπ/2（特别是奇数下标/偶数下标）
//   4. 相邻关节对之和、三元组之和是否为 kπ/2
//   5. 连续前缀和 (q0+q1...+qk) 是否为 kπ/2
//   6. 有多少关节"自由"（既不在边界也不在 kπ/2）
//
// 输出：详细 CSV + 控制台统计
//
// Build:
//   cmake --build . --target exp_24_extrema_configs --config Release
//
// ═══════════════════════════════════════════════════════════════════════════

#define _USE_MATH_DEFINES

#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"
#include "sbf/envelope/envelope_derive_critical.h"
#include "sbf/common/config.h"

#include <Eigen/Core>
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
#include <set>
#include <sstream>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
static constexpr double kHalfPi = M_PI / 2.0;

namespace fs = std::filesystem;
using namespace sbf;
using namespace sbf::envelope;

// ─── CLI ─────────────────────────────────────────────────────────────────

struct ExpConfig {
    int n_trials = 50;
    std::string output_dir;
};

static ExpConfig parse_args(int argc, char** argv) {
    ExpConfig cfg;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--trials" && i + 1 < argc) cfg.n_trials = std::stoi(argv[++i]);
        if (a == "--output" && i + 1 < argc) cfg.output_dir = argv[++i];
    }
    return cfg;
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

// ─── Robot finder ────────────────────────────────────────────────────────

struct RobotSetup {
    std::string name;
    std::string json_path;
};

static std::vector<RobotSetup> find_robots() {
    std::vector<RobotSetup> robots;
    std::vector<std::string> search_paths = {
        "../../../safeboxforest/v1/configs",
        "../../safeboxforest/v1/configs",
        "../safeboxforest/v1/configs",
        "../../../../safeboxforest/v1/configs",
    };
    for (auto& base : search_paths) {
        std::string panda_path = base + "/panda.json";
        std::string iiwa_path  = base + "/iiwa14.json";
        if (fs::exists(panda_path)) {
            robots.push_back({"panda", panda_path});
            if (fs::exists(iiwa_path))
                robots.push_back({"iiwa14", iiwa_path});
            break;
        }
    }
    return robots;
}

// ─── Face names ──────────────────────────────────────────────────────────

static const char* FACE_NAMES[6] = {
    "x_min", "x_max", "y_min", "y_max", "z_min", "z_max"
};

// ─── Analysis helpers ────────────────────────────────────────────────────

// Check if a value is close to k*π/2 for some integer k
static bool is_kpi2(double v, double tol = 1e-6) {
    double k = v / kHalfPi;
    return std::abs(k - std::round(k)) < tol;
}

// Return the nearest k such that v ≈ k*π/2, or INT_MIN if not close
static int nearest_kpi2(double v, double tol = 1e-6) {
    double k = v / kHalfPi;
    int ki = (int)std::round(k);
    if (std::abs(k - ki) < tol) return ki;
    return INT_MIN;
}

// Check if a value is on one of the boundaries
static bool is_on_boundary(double v, double lo, double hi, double tol = 1e-6) {
    return std::abs(v - lo) < tol || std::abs(v - hi) < tol;
}

// Per-extreme record — goes into the CSV
struct ExtremeRecord {
    std::string robot;
    std::string width_regime;
    int trial;
    int link_ci;
    int face;               // 0..5
    int n_joints_total;     // total joints in robot
    int n_joints_affecting; // joints [0..V] that affect this link

    // Per-joint analysis (stored as comma-separated in CSV)
    std::vector<double> q_vals;        // joint values
    std::vector<bool>   on_boundary;   // is this joint on lo or hi?
    std::vector<bool>   at_kpi2;       // is this joint at k*pi/2?
    std::vector<bool>   free_interior; // not on boundary AND not at kpi2 boundary AND not degenerate

    // Aggregate counts
    int n_on_boundary;
    int n_at_kpi2;
    int n_free;         // joints not on boundary
    int n_strictly_free; // joints not on boundary AND not at kpi2

    // Sum patterns:
    // Check all subsets of consecutive joints from index 0
    // q0, q0+q1, q0+q1+q2, ...
    std::vector<bool> prefix_sum_kpi2;  // is sum(q[0..k]) = m*pi/2?

    // Odd-indexed joints sum: q1+q3+q5+...
    bool odd_sum_kpi2;
    // Even-indexed joints sum: q0+q2+q4+q6+...
    bool even_sum_kpi2;

    // All consecutive pairs qi+q(i+1)
    std::vector<bool> pair_sum_kpi2;

    // All consecutive triples qi+q(i+1)+q(i+2)
    std::vector<bool> triple_sum_kpi2;

    // Which phase found this extreme (from incremental comparison)
    int found_by_phase;
};

// ─── Main ────────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    auto exp_cfg = parse_args(argc, argv);
    auto robots = find_robots();
    if (robots.empty()) {
        std::cerr << "No robot configs found\n";
        return 1;
    }

    constexpr int N_SUB = 1;
    constexpr double GAP_THRESHOLD = 1e-7;

    std::vector<ExtremeRecord> all_records;
    std::mt19937 rng(12345);

    std::cout << std::string(78, '=') << "\n"
              << " Experiment 24: Extrema Configuration Analysis\n"
              << std::string(78, '=') << "\n"
              << " Trials: " << exp_cfg.n_trials
              << "  Robots: " << robots.size() << "\n\n";

    for (auto& rs : robots) {
        Robot robot = Robot::from_json(rs.json_path);
        const int n = robot.n_joints();
        const int n_act = robot.n_active_links();
        const int* map = robot.active_link_map();
        const int total_slots = n_act * N_SUB;

        std::cout << "== Robot: " << rs.name
                  << " (" << n << "J, " << n_act << " active links) ==\n";
        // Print DH info
        auto& dh = robot.dh_params();
        std::cout << "   DH alpha: [";
        for (int j = 0; j < n; ++j) {
            if (j > 0) std::cout << ", ";
            std::cout << std::fixed << std::setprecision(4) << dh[j].alpha;
        }
        std::cout << "]\n";
        std::cout << "   active_link_map: [";
        for (int ci = 0; ci < n_act; ++ci) {
            if (ci > 0) std::cout << ", ";
            std::cout << map[ci];
        }
        std::cout << "]\n\n";

        for (auto& wr : WIDTHS) {
            std::cout << "  [" << wr.name << "] ";
            std::cout.flush();

            for (int trial = 0; trial < exp_cfg.n_trials; ++trial) {
                if (trial % 10 == 0) { std::cout << "."; std::cout.flush(); }

                // Generate random intervals
                std::vector<Interval> ivls(n);
                for (int j = 0; j < n; ++j) {
                    double range = robot.joint_limits().limits[j].hi
                                 - robot.joint_limits().limits[j].lo;
                    double width = std::uniform_real_distribution<double>(
                        wr.lo, wr.hi)(rng) * range;
                    double center = std::uniform_real_distribution<double>(
                        robot.joint_limits().limits[j].lo + width / 2,
                        robot.joint_limits().limits[j].hi - width / 2)(rng);
                    ivls[j].lo = center - width / 2;
                    ivls[j].hi = center + width / 2;
                }

                // ──── Full Analytical with configs ──────────────────
                std::vector<float> aabb_full(total_slots * 6);
                std::vector<Eigen::VectorXd> configs_full(total_slots * 6);
                {
                    auto cfg = AnalyticalCriticalConfig::all_enabled();
                    derive_aabb_critical_analytical_with_configs(
                        robot, ivls, N_SUB, cfg,
                        aabb_full.data(), configs_full.data(), nullptr);
                }

                // ──── Incremental phases for classification ─────────
                std::vector<float> aabb_p0(total_slots * 6);
                {
                    AnalyticalCriticalConfig cfg;
                    cfg.keep_kpi2_baseline = true;
                    cfg.enable_edge_solve = false;
                    cfg.enable_face_solve = false;
                    cfg.enable_interior_solve = false;
                    derive_aabb_critical_analytical(robot, ivls, N_SUB, cfg,
                                                   aabb_p0.data(), nullptr);
                }

                std::vector<float> aabb_p01(total_slots * 6);
                {
                    auto cfg = AnalyticalCriticalConfig::edges_only();
                    derive_aabb_critical_analytical(robot, ivls, N_SUB, cfg,
                                                   aabb_p01.data(), nullptr);
                }

                std::vector<float> aabb_p012(total_slots * 6);
                {
                    auto cfg = AnalyticalCriticalConfig::edges_and_faces();
                    derive_aabb_critical_analytical(robot, ivls, N_SUB, cfg,
                                                   aabb_p012.data(), nullptr);
                }

                // ──── Analyze each extreme config ───────────────────
                for (int slot = 0; slot < total_slots; ++slot) {
                    int ci = slot / N_SUB;
                    int V  = map[ci];
                    int nj = std::min(V + 1, n); // joints affecting this link

                    for (int face = 0; face < 6; ++face) {
                        float ref_val = aabb_full[slot * 6 + face];
                        const Eigen::VectorXd& q = configs_full[slot * 6 + face];
                        if (q.size() != n) continue;

                        // Determine phase
                        auto gap_fn = [&](float v) -> double {
                            if (face % 2 == 0) return double(v) - double(ref_val);
                            else return double(ref_val) - double(v);
                        };
                        float p0_val   = aabb_p0[slot * 6 + face];
                        float p01_val  = aabb_p01[slot * 6 + face];
                        float p012_val = aabb_p012[slot * 6 + face];
                        int found_by_phase;
                        if (gap_fn(p0_val) < GAP_THRESHOLD)        found_by_phase = 0;
                        else if (gap_fn(p01_val) < GAP_THRESHOLD)  found_by_phase = 1;
                        else if (gap_fn(p012_val) < GAP_THRESHOLD) found_by_phase = 2;
                        else                                       found_by_phase = 3;

                        // Build record
                        ExtremeRecord rec;
                        rec.robot = rs.name;
                        rec.width_regime = wr.name;
                        rec.trial = trial;
                        rec.link_ci = ci;
                        rec.face = face;
                        rec.n_joints_total = n;
                        rec.n_joints_affecting = nj;
                        rec.found_by_phase = found_by_phase;

                        rec.q_vals.resize(n);
                        rec.on_boundary.resize(n);
                        rec.at_kpi2.resize(n);
                        rec.free_interior.resize(n);

                        rec.n_on_boundary = 0;
                        rec.n_at_kpi2 = 0;
                        rec.n_free = 0;
                        rec.n_strictly_free = 0;

                        for (int j = 0; j < n; ++j) {
                            rec.q_vals[j] = q[j];
                            bool on_bnd = is_on_boundary(q[j], ivls[j].lo, ivls[j].hi);
                            bool at_kp  = is_kpi2(q[j]);
                            rec.on_boundary[j] = on_bnd;
                            rec.at_kpi2[j] = at_kp;
                            if (on_bnd) rec.n_on_boundary++;
                            if (at_kp) rec.n_at_kpi2++;

                            bool is_free = !on_bnd;
                            if (j >= nj) is_free = false; // past affecting joints, always boundary
                            if (is_free) rec.n_free++;

                            rec.free_interior[j] = is_free && !at_kp;
                            if (is_free && !at_kp) rec.n_strictly_free++;
                        }

                        // Prefix sums: sum(q[0..k])
                        rec.prefix_sum_kpi2.resize(nj);
                        double prefix = 0;
                        for (int k = 0; k < nj; ++k) {
                            prefix += q[k];
                            rec.prefix_sum_kpi2[k] = is_kpi2(prefix);
                        }

                        // Odd/even sums (over affecting joints only)
                        double odd_sum = 0, even_sum = 0;
                        for (int j = 0; j < nj; ++j) {
                            if (j % 2 == 0) even_sum += q[j];
                            else             odd_sum  += q[j];
                        }
                        rec.odd_sum_kpi2  = is_kpi2(odd_sum);
                        rec.even_sum_kpi2 = is_kpi2(even_sum);

                        // Consecutive pair sums
                        rec.pair_sum_kpi2.resize(nj > 0 ? nj - 1 : 0);
                        for (int j = 0; j + 1 < nj; ++j)
                            rec.pair_sum_kpi2[j] = is_kpi2(q[j] + q[j+1]);

                        // Consecutive triple sums
                        rec.triple_sum_kpi2.resize(nj > 1 ? nj - 2 : 0);
                        for (int j = 0; j + 2 < nj; ++j)
                            rec.triple_sum_kpi2[j] = is_kpi2(q[j] + q[j+1] + q[j+2]);

                        all_records.push_back(std::move(rec));
                    }
                }
            }
            std::cout << " done\n";
        }
        std::cout << "\n";
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  Console analysis
    // ═══════════════════════════════════════════════════════════════════════

    int total_rec = (int)all_records.size();
    std::cout << std::string(78, '=') << "\n"
              << " Analysis (" << total_rec << " total extreme records)\n"
              << std::string(78, '=') << "\n\n";

    // ──── 1. Boundary/free/kpi2 distribution by phase ────────────────
    std::cout << "1. Joint status distribution by discovery phase\n";
    std::cout << std::string(80, '-') << "\n";
    {
        struct PhaseStat {
            int count = 0;
            double sum_n_boundary = 0;
            double sum_n_free = 0;
            double sum_n_kpi2 = 0;
            double sum_n_strictly_free = 0;
            double sum_frac_boundary = 0;
            double sum_frac_free = 0;
        };
        PhaseStat ps[4];
        for (auto& r : all_records) {
            int p = r.found_by_phase;
            ps[p].count++;
            ps[p].sum_n_boundary += r.n_on_boundary;
            ps[p].sum_n_free += r.n_free;
            ps[p].sum_n_kpi2 += r.n_at_kpi2;
            ps[p].sum_n_strictly_free += r.n_strictly_free;
            double nj = r.n_joints_affecting;
            if (nj > 0) {
                ps[p].sum_frac_boundary += r.n_on_boundary / nj;
                ps[p].sum_frac_free += r.n_free / nj;
            }
        }

        std::cout << std::left << std::setw(10) << "Phase"
                  << std::right
                  << std::setw(8) << "Count"
                  << std::setw(12) << "AvgBndry"
                  << std::setw(12) << "AvgFree"
                  << std::setw(12) << "AvgKpi2"
                  << std::setw(12) << "AvgStrFree"
                  << std::setw(14) << "FracBndry"
                  << std::setw(14) << "FracFree"
                  << "\n";
        for (int p = 0; p < 4; ++p) {
            auto& s = ps[p];
            if (s.count == 0) continue;
            std::cout << std::left << std::setw(10) << ("Phase " + std::to_string(p))
                      << std::right
                      << std::setw(8) << s.count
                      << std::fixed << std::setprecision(2)
                      << std::setw(12) << s.sum_n_boundary / s.count
                      << std::setw(12) << s.sum_n_free / s.count
                      << std::setw(12) << s.sum_n_kpi2 / s.count
                      << std::setw(12) << s.sum_n_strictly_free / s.count
                      << std::setw(14) << s.sum_frac_boundary / s.count
                      << std::setw(14) << s.sum_frac_free / s.count
                      << "\n";
        }
    }

    // ──── 2. Free-joint count distribution by phase ──────────────────
    std::cout << "\n2. Free-joint count (non-boundary) distribution within affecting joints\n";
    std::cout << std::string(80, '-') << "\n";
    {
        // For each phase: histogram of n_free (among affecting joints)
        std::map<int, std::map<int, int>> hist; // phase -> n_free -> count
        for (auto& r : all_records) {
            // Count free joints among the affecting subset
            int n_free_aff = 0;
            for (int j = 0; j < r.n_joints_affecting; ++j)
                if (!r.on_boundary[j]) n_free_aff++;
            hist[r.found_by_phase][n_free_aff]++;
        }
        for (int p = 0; p < 4; ++p) {
            std::cout << "  Phase " << p << ": ";
            for (auto& [nf, cnt] : hist[p])
                std::cout << nf << "free=" << cnt << "  ";
            std::cout << "\n";
        }
    }

    // ──── 3. Odd/even sum patterns ───────────────────────────────────
    std::cout << "\n3. Odd/even joint-index sum = k*pi/2 patterns by phase\n";
    std::cout << std::string(80, '-') << "\n";
    {
        struct SumStat {
            int count = 0;
            int odd_kpi2 = 0;
            int even_kpi2 = 0;
            int both_kpi2 = 0;
            int either_kpi2 = 0;
        };
        SumStat ss[4];
        for (auto& r : all_records) {
            int p = r.found_by_phase;
            ss[p].count++;
            if (r.odd_sum_kpi2) ss[p].odd_kpi2++;
            if (r.even_sum_kpi2) ss[p].even_kpi2++;
            if (r.odd_sum_kpi2 && r.even_sum_kpi2) ss[p].both_kpi2++;
            if (r.odd_sum_kpi2 || r.even_sum_kpi2) ss[p].either_kpi2++;
        }
        std::cout << std::left << std::setw(10) << "Phase"
                  << std::right
                  << std::setw(8) << "Count"
                  << std::setw(12) << "OddKpi2"
                  << std::setw(12) << "EvenKpi2"
                  << std::setw(12) << "Both"
                  << std::setw(12) << "Either"
                  << std::setw(14) << "OddRate"
                  << std::setw(14) << "EvenRate"
                  << "\n";
        for (int p = 0; p < 4; ++p) {
            auto& s = ss[p];
            if (s.count == 0) continue;
            std::cout << std::left << std::setw(10) << ("Phase " + std::to_string(p))
                      << std::right
                      << std::setw(8) << s.count
                      << std::setw(12) << s.odd_kpi2
                      << std::setw(12) << s.even_kpi2
                      << std::setw(12) << s.both_kpi2
                      << std::setw(12) << s.either_kpi2
                      << std::fixed << std::setprecision(1)
                      << std::setw(13) << (100.0 * s.odd_kpi2 / s.count) << "%"
                      << std::setw(13) << (100.0 * s.even_kpi2 / s.count) << "%"
                      << "\n";
        }
    }

    // ──── 4. Consecutive pair sum patterns ───────────────────────────
    std::cout << "\n4. Consecutive pair sum (q_j + q_{j+1}) = k*pi/2 analysis\n";
    std::cout << std::string(80, '-') << "\n";
    {
        // For each phase, count how many records have at least one pair sum = kpi2
        // and the average number of such pairs
        struct PairStat {
            int count = 0;
            int has_any_pair = 0;
            int total_pairs_checked = 0;
            int total_pairs_kpi2 = 0;
        };
        PairStat pst[4];
        for (auto& r : all_records) {
            int p = r.found_by_phase;
            pst[p].count++;
            bool any = false;
            for (int j = 0; j < (int)r.pair_sum_kpi2.size(); ++j) {
                pst[p].total_pairs_checked++;
                if (r.pair_sum_kpi2[j]) {
                    pst[p].total_pairs_kpi2++;
                    any = true;
                }
            }
            if (any) pst[p].has_any_pair++;
        }
        std::cout << std::left << std::setw(10) << "Phase"
                  << std::right
                  << std::setw(8) << "Count"
                  << std::setw(14) << "HasAnyPair"
                  << std::setw(14) << "HasAnyRate"
                  << std::setw(14) << "TotalPairs"
                  << std::setw(14) << "PairsKpi2"
                  << std::setw(14) << "PairRate"
                  << "\n";
        for (int p = 0; p < 4; ++p) {
            auto& s = pst[p];
            if (s.count == 0) continue;
            std::cout << std::left << std::setw(10) << ("Phase " + std::to_string(p))
                      << std::right
                      << std::setw(8) << s.count
                      << std::setw(14) << s.has_any_pair
                      << std::fixed << std::setprecision(1)
                      << std::setw(13) << (100.0 * s.has_any_pair / s.count) << "%"
                      << std::setw(14) << s.total_pairs_checked
                      << std::setw(14) << s.total_pairs_kpi2
                      << std::setw(13) << (s.total_pairs_checked > 0
                          ? 100.0 * s.total_pairs_kpi2 / s.total_pairs_checked : 0) << "%"
                      << "\n";
        }
    }

    // ──── 5. Consecutive triple sum patterns ─────────────────────────
    std::cout << "\n5. Consecutive triple sum (q_j + q_{j+1} + q_{j+2}) = k*pi/2 analysis\n";
    std::cout << std::string(80, '-') << "\n";
    {
        struct TriStat {
            int count = 0;
            int has_any_triple = 0;
            int total_triples = 0;
            int triples_kpi2 = 0;
        };
        TriStat tst[4];
        for (auto& r : all_records) {
            int p = r.found_by_phase;
            tst[p].count++;
            bool any = false;
            for (int j = 0; j < (int)r.triple_sum_kpi2.size(); ++j) {
                tst[p].total_triples++;
                if (r.triple_sum_kpi2[j]) {
                    tst[p].triples_kpi2++;
                    any = true;
                }
            }
            if (any) tst[p].has_any_triple++;
        }
        std::cout << std::left << std::setw(10) << "Phase"
                  << std::right
                  << std::setw(8) << "Count"
                  << std::setw(16) << "HasAnyTriple"
                  << std::setw(14) << "HasAnyRate"
                  << std::setw(14) << "TotalTrips"
                  << std::setw(14) << "TripsKpi2"
                  << std::setw(14) << "TripleRate"
                  << "\n";
        for (int p = 0; p < 4; ++p) {
            auto& s = tst[p];
            if (s.count == 0) continue;
            std::cout << std::left << std::setw(10) << ("Phase " + std::to_string(p))
                      << std::right
                      << std::setw(8) << s.count
                      << std::setw(16) << s.has_any_triple
                      << std::fixed << std::setprecision(1)
                      << std::setw(13) << (100.0 * s.has_any_triple / s.count) << "%"
                      << std::setw(14) << s.total_triples
                      << std::setw(14) << s.triples_kpi2
                      << std::setw(13) << (s.total_triples > 0
                          ? 100.0 * s.triples_kpi2 / s.total_triples : 0) << "%"
                      << "\n";
        }
    }

    // ──── 6. Prefix sum patterns ─────────────────────────────────────
    std::cout << "\n6. Prefix sum (q_0+...+q_k) = k*pi/2 analysis by joint depth\n";
    std::cout << std::string(80, '-') << "\n";
    {
        // For each joint index k, what fraction of extremes have
        // sum(q[0..k]) = m*pi/2?
        // Split by phase.
        const int MAX_J = 8;
        struct PrefixStat {
            int count = 0;
            int hit[MAX_J] = {};       // how many have prefix_sum[k] = kpi2
            int checked[MAX_J] = {};
        };
        PrefixStat pfst[4];
        for (auto& r : all_records) {
            int p = r.found_by_phase;
            pfst[p].count++;
            for (int k = 0; k < (int)r.prefix_sum_kpi2.size() && k < MAX_J; ++k) {
                pfst[p].checked[k]++;
                if (r.prefix_sum_kpi2[k]) pfst[p].hit[k]++;
            }
        }
        std::cout << std::left << std::setw(10) << "Phase";
        for (int k = 0; k < MAX_J; ++k)
            std::cout << std::right << std::setw(10) << ("q0..q" + std::to_string(k));
        std::cout << "\n";
        for (int p = 0; p < 4; ++p) {
            auto& s = pfst[p];
            if (s.count == 0) continue;
            std::cout << std::left << std::setw(10) << ("Phase " + std::to_string(p));
            for (int k = 0; k < MAX_J; ++k) {
                if (s.checked[k] > 0)
                    std::cout << std::right << std::fixed << std::setprecision(1)
                              << std::setw(9) << (100.0 * s.hit[k] / s.checked[k]) << "%";
                else
                    std::cout << std::right << std::setw(10) << "-";
            }
            std::cout << "\n";
        }
    }

    // ──── 7. Arbitrary subset sums (all C(nj,2) and C(nj,3)) ────────
    std::cout << "\n7. Arbitrary (non-consecutive) pair & triple sums = k*pi/2\n";
    std::cout << std::string(80, '-') << "\n";
    {
        struct ArbitStat {
            int count = 0;
            int has_any_pair_kpi2 = 0;
            int has_any_triple_kpi2 = 0;
            int total_arb_pairs = 0;
            int arb_pairs_kpi2 = 0;
            int total_arb_triples = 0;
            int arb_triples_kpi2 = 0;
        };
        ArbitStat ast[4];
        for (auto& r : all_records) {
            int p = r.found_by_phase;
            ast[p].count++;
            int nj = r.n_joints_affecting;
            bool any_pair = false, any_triple = false;
            // All C(nj, 2) pairs
            for (int i = 0; i < nj; ++i) {
                for (int j = i + 1; j < nj; ++j) {
                    ast[p].total_arb_pairs++;
                    if (is_kpi2(r.q_vals[i] + r.q_vals[j])) {
                        ast[p].arb_pairs_kpi2++;
                        any_pair = true;
                    }
                }
            }
            // All C(nj, 3) triples
            for (int i = 0; i < nj; ++i)
                for (int j = i + 1; j < nj; ++j)
                    for (int k = j + 1; k < nj; ++k) {
                        ast[p].total_arb_triples++;
                        if (is_kpi2(r.q_vals[i] + r.q_vals[j] + r.q_vals[k])) {
                            ast[p].arb_triples_kpi2++;
                            any_triple = true;
                        }
                    }
            if (any_pair) ast[p].has_any_pair_kpi2++;
            if (any_triple) ast[p].has_any_triple_kpi2++;
        }
        std::cout << std::left << std::setw(10) << "Phase"
                  << std::right
                  << std::setw(8) << "Count"
                  << std::setw(14) << "AnyPairKpi2"
                  << std::setw(14) << "PairRate"
                  << std::setw(16) << "AnyTripleKpi2"
                  << std::setw(14) << "TripleRate"
                  << std::setw(16) << "PairHit/Total"
                  << std::setw(18) << "TripleHit/Total"
                  << "\n";
        for (int p = 0; p < 4; ++p) {
            auto& s = ast[p];
            if (s.count == 0) continue;
            std::cout << std::left << std::setw(10) << ("Phase " + std::to_string(p))
                      << std::right
                      << std::setw(8) << s.count
                      << std::setw(14) << s.has_any_pair_kpi2
                      << std::fixed << std::setprecision(1)
                      << std::setw(13) << (100.0 * s.has_any_pair_kpi2 / s.count) << "%"
                      << std::setw(16) << s.has_any_triple_kpi2
                      << std::setw(13) << (100.0 * s.has_any_triple_kpi2 / s.count) << "%"
                      << "  " << s.arb_pairs_kpi2 << "/" << s.total_arb_pairs
                      << "  " << s.arb_triples_kpi2 << "/" << s.total_arb_triples
                      << "\n";
        }
    }

    // ──── 8. Phase 3 deep dive: what makes them special ──────────────
    std::cout << "\n8. Phase 3+ deep dive: config characteristics\n";
    std::cout << std::string(80, '-') << "\n";
    {
        // For Phase 3 records only, print detailed stats
        int p3_count = 0;
        int p3_has_pair_sum = 0;
        int p3_has_triple_sum = 0;
        int p3_has_odd = 0, p3_has_even = 0;
        // Histogram: how many free joints (not on boundary)?
        std::map<int, int> p3_free_hist;
        // Histogram: how many joints at kpi2?
        std::map<int, int> p3_kpi2_hist;
        // How many have ALL affecting joints free?
        int p3_all_free = 0;
        // How many have >= N joints free, for N in {1,2,3,4,5}
        int p3_ge_n_free[6] = {};

        for (auto& r : all_records) {
            if (r.found_by_phase != 3) continue;
            p3_count++;

            int n_free_aff = 0;
            int n_kpi2_aff = 0;
            for (int j = 0; j < r.n_joints_affecting; ++j) {
                if (!r.on_boundary[j]) n_free_aff++;
                if (r.at_kpi2[j]) n_kpi2_aff++;
            }
            p3_free_hist[n_free_aff]++;
            p3_kpi2_hist[n_kpi2_aff]++;
            if (n_free_aff == r.n_joints_affecting) p3_all_free++;
            for (int nn = 1; nn <= 5; ++nn)
                if (n_free_aff >= nn) p3_ge_n_free[nn]++;

            bool any_pair = false;
            for (auto b : r.pair_sum_kpi2) if (b) any_pair = true;
            bool any_triple = false;
            for (auto b : r.triple_sum_kpi2) if (b) any_triple = true;
            if (any_pair) p3_has_pair_sum++;
            if (any_triple) p3_has_triple_sum++;
            if (r.odd_sum_kpi2) p3_has_odd++;
            if (r.even_sum_kpi2) p3_has_even++;
        }

        std::cout << "  Total Phase 3 records: " << p3_count << "\n";
        std::cout << "  Free-joints histogram (affecting only):\n";
        for (auto& [nf, cnt] : p3_free_hist)
            std::cout << "    " << nf << " free: " << cnt
                      << " (" << std::fixed << std::setprecision(1)
                      << (100.0 * cnt / p3_count) << "%)\n";
        std::cout << "  At-kpi2 histogram:\n";
        for (auto& [nk, cnt] : p3_kpi2_hist)
            std::cout << "    " << nk << " at kpi2: " << cnt
                      << " (" << std::fixed << std::setprecision(1)
                      << (100.0 * cnt / p3_count) << "%)\n";
        std::cout << "  All affecting joints free: " << p3_all_free
                  << " (" << std::fixed << std::setprecision(1)
                  << (100.0 * p3_all_free / std::max(1, p3_count)) << "%)\n";
        for (int nn = 1; nn <= 5; ++nn)
            std::cout << "  >= " << nn << " free: " << p3_ge_n_free[nn]
                      << " (" << (100.0 * p3_ge_n_free[nn] / std::max(1, p3_count)) << "%)\n";
        std::cout << "  Has consec pair sum=kpi2:   " << p3_has_pair_sum
                  << " (" << (100.0 * p3_has_pair_sum / std::max(1, p3_count)) << "%)\n";
        std::cout << "  Has consec triple sum=kpi2: " << p3_has_triple_sum
                  << " (" << (100.0 * p3_has_triple_sum / std::max(1, p3_count)) << "%)\n";
        std::cout << "  Odd-index sum=kpi2:  " << p3_has_odd
                  << " (" << (100.0 * p3_has_odd / std::max(1, p3_count)) << "%)\n";
        std::cout << "  Even-index sum=kpi2: " << p3_has_even
                  << " (" << (100.0 * p3_has_even / std::max(1, p3_count)) << "%)\n";
    }

    // ═══════════════════════════════════════════════════════════════════════
    //  Write CSV
    // ═══════════════════════════════════════════════════════════════════════

    std::string dir_path;
    if (!exp_cfg.output_dir.empty()) {
        dir_path = exp_cfg.output_dir;
    } else {
        auto now = std::chrono::system_clock::now();
        auto t = std::chrono::system_clock::to_time_t(now);
        std::tm tm_buf{};
#ifdef _WIN32
        localtime_s(&tm_buf, &t);
#else
        localtime_r(&t, &tm_buf);
#endif
        std::ostringstream oss;
        oss << "../results/exp24_configs_"
            << std::put_time(&tm_buf, "%Y%m%d_%H%M%S");
        dir_path = oss.str();
    }
    fs::create_directories(dir_path);

    // ──── Detailed CSV ───────────────────────────────────────────────
    {
        std::string csv_path = dir_path + "/exp24_extremes.csv";
        std::ofstream csv(csv_path);
        if (csv.is_open()) {
            // Header
            csv << "robot,width,trial,link_ci,face,face_name,found_by_phase,"
                << "n_joints,n_affecting,"
                << "n_on_boundary,n_free,n_at_kpi2,n_strictly_free,"
                << "odd_sum_kpi2,even_sum_kpi2";
            // q_vals and joint flags (up to 7 joints)
            for (int j = 0; j < 7; ++j)
                csv << ",q" << j << ",bnd" << j << ",kpi2_" << j;
            // Pair sums
            for (int j = 0; j < 6; ++j)
                csv << ",pair" << j << "_" << (j+1);
            // Triple sums
            for (int j = 0; j < 5; ++j)
                csv << ",triple" << j << "_" << (j+1) << "_" << (j+2);
            // Prefix sums
            for (int k = 0; k < 7; ++k)
                csv << ",prefix_0_" << k;
            csv << "\n";

            for (auto& r : all_records) {
                csv << r.robot << "," << r.width_regime << ","
                    << r.trial << "," << r.link_ci << ","
                    << r.face << "," << FACE_NAMES[r.face] << ","
                    << r.found_by_phase << ","
                    << r.n_joints_total << "," << r.n_joints_affecting << ","
                    << r.n_on_boundary << "," << r.n_free << ","
                    << r.n_at_kpi2 << "," << r.n_strictly_free << ","
                    << (r.odd_sum_kpi2 ? 1 : 0) << ","
                    << (r.even_sum_kpi2 ? 1 : 0);

                for (int j = 0; j < 7; ++j) {
                    if (j < r.n_joints_total) {
                        csv << "," << std::fixed << std::setprecision(8) << r.q_vals[j]
                            << "," << (r.on_boundary[j] ? 1 : 0)
                            << "," << (r.at_kpi2[j] ? 1 : 0);
                    } else {
                        csv << ",,,";
                    }
                }
                for (int j = 0; j < 6; ++j) {
                    if (j < (int)r.pair_sum_kpi2.size())
                        csv << "," << (r.pair_sum_kpi2[j] ? 1 : 0);
                    else
                        csv << ",";
                }
                for (int j = 0; j < 5; ++j) {
                    if (j < (int)r.triple_sum_kpi2.size())
                        csv << "," << (r.triple_sum_kpi2[j] ? 1 : 0);
                    else
                        csv << ",";
                }
                for (int k = 0; k < 7; ++k) {
                    if (k < (int)r.prefix_sum_kpi2.size())
                        csv << "," << (r.prefix_sum_kpi2[k] ? 1 : 0);
                    else
                        csv << ",";
                }
                csv << "\n";
            }
            csv.close();
            std::cout << "\nDetailed CSV: " << csv_path
                      << " (" << all_records.size() << " records)\n";
        }
    }

    std::cout << "\nDone.\n";
    return 0;
}
