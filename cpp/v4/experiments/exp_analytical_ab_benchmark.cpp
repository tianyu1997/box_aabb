// ═══════════════════════════════════════════════════════════════════════════
//  Experiment: Analytical A/B Benchmark (candidate dedup off vs on)
//
//  Compares analytical solver performance under identical intervals:
//    A: enable_candidate_dedup = false
//    B: enable_candidate_dedup = true
//
//  Metrics:
//    - mean runtime (ms)
//    - mean total FK calls and per-phase FK calls
//    - speedup ratio B/A
//
//  Build: cmake --build . --config Release --target exp_analytical_ab_benchmark
//  Run:   Release\exp_analytical_ab_benchmark.exe
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_math.h"
#include "sbf/envelope/analytical_solve.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cstdio>
#include <numeric>
#include <string>
#include <vector>

using namespace sbf;
using namespace sbf::envelope;

static std::vector<double> env_widths_or_default(const char* name, bool* used_env) {
    if (used_env) *used_env = false;
    const char* val = std::getenv(name);
    if (!val || !(*val)) return {0.10, 0.20, 0.35, 0.50, 0.70};

    std::vector<double> widths;
    const char* p = val;
    while (*p) {
        while (*p == ' ' || *p == '\t' || *p == ',') ++p;
        if (!(*p)) break;
        char* end_ptr = nullptr;
        const double w = std::strtod(p, &end_ptr);
        if (end_ptr == p) {
            while (*p && *p != ',') ++p;
            continue;
        }
        if (w > 0.0) widths.push_back(w);
        p = end_ptr;
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == ',') ++p;
    }

    if (widths.empty()) return {0.10, 0.20, 0.35, 0.50, 0.70};
    if (used_env) *used_env = true;
    return widths;
}

static Robot make_iiwa14() {
    std::vector<DHParam> dh = {
        {  0.0,     0.0, 0.1575, 0.0, 0},
        { -HALF_PI, 0.0, 0.0,    0.0, 0},
        {  HALF_PI, 0.0, 0.2025, 0.0, 0},
        {  HALF_PI, 0.0, 0.0,    0.0, 0},
        { -HALF_PI, 0.0, 0.2155, 0.0, 0},
        { -HALF_PI, 0.0, 0.0,    0.0, 0},
        {  HALF_PI, 0.0, 0.081,  0.0, 0},
    };

    JointLimits limits;
    limits.limits = {
        {-1.865, 1.866}, {-0.100, 1.087}, {-0.663, 0.662},
        {-2.094,-0.372}, {-0.619, 0.620}, {-1.095, 1.258},
        { 1.050, 2.091},
    };

    DHParam tool{0.0, 0.0, 0.185, 0.0, 0};
    std::vector<double> radii = {0.08, 0.08, 0.06, 0.06, 0.04, 0.04, 0.04, 0.03};
    return Robot("iiwa14", dh, limits, tool, radii);
}

static std::vector<Interval> make_centered_intervals(const Robot& robot, double width) {
    const int n = robot.n_joints();
    std::vector<Interval> ivs(n);
    for (int j = 0; j < n; ++j) {
        const auto& lim = robot.joint_limits().limits[j];
        const double center = 0.5 * (lim.lo + lim.hi);
        double lo = std::max(lim.lo, center - 0.5 * width);
        double hi = std::min(lim.hi, center + 0.5 * width);
        if (hi - lo < 1e-9) {
            const double eps = std::min(1e-3, 0.5 * (lim.hi - lim.lo));
            lo = std::max(lim.lo, center - eps);
            hi = std::min(lim.hi, center + eps);
        }
        ivs[j] = Interval(lo, hi);
    }
    return ivs;
}

struct RunAgg {
    double total_ms = 0.0;
    long long total_fk = 0;
    long long p1_fk = 0;
    long long p2_fk = 0;
    long long p25_fk = 0;
    long long p3_fk = 0;

    long long d_raw_p1 = 0;
    long long d_uni_p1 = 0;
    long long d_app_p1 = 0;
    long long d_skip_p1 = 0;

    long long d_raw_p2 = 0;
    long long d_uni_p2 = 0;
    long long d_app_p2 = 0;
    long long d_skip_p2 = 0;

    long long d_raw_p25 = 0;
    long long d_uni_p25 = 0;
    long long d_app_p25 = 0;
    long long d_skip_p25 = 0;

    long long d_raw_p3 = 0;
    long long d_uni_p3 = 0;
    long long d_app_p3 = 0;
    long long d_skip_p3 = 0;

    // (O5 proto/shadow/hybrid fields removed in Phase E cleanup)

    double checksum = 0.0;
};

static RunAgg run_benchmark(
    const Robot& robot,
    const std::vector<Interval>& ivs,
    const AnalyticalCriticalConfig& cfg,
    int n_sub,
    int repeats)
{
    RunAgg agg;
    const int n_act = robot.n_active_links();
    std::vector<float> out_aabb(n_act * n_sub * 6);

    for (int r = 0; r < repeats; ++r) {
        AnalyticalCriticalStats stats{};
        auto t0 = std::chrono::high_resolution_clock::now();
        derive_aabb_critical_analytical(robot, ivs, n_sub, cfg, out_aabb.data(), &stats);
        auto t1 = std::chrono::high_resolution_clock::now();

        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        agg.total_ms += ms;

        const int fk_total =
            stats.n_phase1_fk_calls + stats.n_phase2_fk_calls +
            stats.n_phase25_fk_calls + stats.n_phase3_fk_calls;
        agg.total_fk += fk_total;
        agg.p1_fk += stats.n_phase1_fk_calls;
        agg.p2_fk += stats.n_phase2_fk_calls;
        agg.p25_fk += stats.n_phase25_fk_calls;
        agg.p3_fk += stats.n_phase3_fk_calls;

        agg.d_raw_p1 += static_cast<long long>(stats.n_dedup_raw_candidates_p1);
        agg.d_uni_p1 += static_cast<long long>(stats.n_dedup_unique_candidates_p1);
        agg.d_app_p1 += static_cast<long long>(stats.n_dedup_applied_sets_p1);
        agg.d_skip_p1 += static_cast<long long>(stats.n_dedup_skipped_sets_p1);

        agg.d_raw_p2 += static_cast<long long>(stats.n_dedup_raw_candidates_p2);
        agg.d_uni_p2 += static_cast<long long>(stats.n_dedup_unique_candidates_p2);
        agg.d_app_p2 += static_cast<long long>(stats.n_dedup_applied_sets_p2);
        agg.d_skip_p2 += static_cast<long long>(stats.n_dedup_skipped_sets_p2);

        agg.d_raw_p25 += static_cast<long long>(stats.n_dedup_raw_candidates_p25);
        agg.d_uni_p25 += static_cast<long long>(stats.n_dedup_unique_candidates_p25);
        agg.d_app_p25 += static_cast<long long>(stats.n_dedup_applied_sets_p25);
        agg.d_skip_p25 += static_cast<long long>(stats.n_dedup_skipped_sets_p25);

        agg.d_raw_p3 += static_cast<long long>(stats.n_dedup_raw_candidates_p3);
        agg.d_uni_p3 += static_cast<long long>(stats.n_dedup_unique_candidates_p3);
        agg.d_app_p3 += static_cast<long long>(stats.n_dedup_applied_sets_p3);
        agg.d_skip_p3 += static_cast<long long>(stats.n_dedup_skipped_sets_p3);

        for (float v : out_aabb) agg.checksum += static_cast<double>(v);
    }

    return agg;
}

int main() {
    Robot robot = make_iiwa14();

    bool used_custom_widths = false;
    const std::vector<double> widths = env_widths_or_default("SBF_BENCH_WIDTHS", &used_custom_widths);
    const char* bench_width_profile = used_custom_widths ? "env_custom" : "default";

    AnalyticalCriticalConfig cfg_a = AnalyticalCriticalConfig::all_enabled();
    cfg_a.enable_candidate_dedup = false;
    cfg_a.enable_aa_pruning = true;

    AnalyticalCriticalConfig cfg_b = AnalyticalCriticalConfig::all_enabled();
    cfg_b.enable_candidate_dedup = true;
    cfg_b.enable_aa_pruning = true;
    cfg_b.enable_adaptive_candidate_dedup = true;
    cfg_b.candidate_dedup_min_candidates = 4;
    cfg_b.candidate_dedup_warmup_sets = 32;
    cfg_b.candidate_dedup_min_hit_rate = 0.02;

    const int warmup = 2;
    const int repeats = 12;
    const int n_sub = 1;

    std::printf("mode,width,bench_width_profile,"
                "mean_ms,mean_fk_total,mean_fk_p1,mean_fk_p2,mean_fk_p25,mean_fk_p3,"
                "dedup_hit_rate_p1,dedup_hit_rate_p2,dedup_hit_rate_p25,dedup_hit_rate_p3,"
                "dedup_applied_sets_p1,dedup_applied_sets_p2,dedup_applied_sets_p25,dedup_applied_sets_p3,"
                "dedup_skipped_sets_p1,dedup_skipped_sets_p2,dedup_skipped_sets_p25,dedup_skipped_sets_p3,"
                "checksum\n");

    for (size_t wi = 0; wi < widths.size(); ++wi) {
        const double width = widths[wi];
        const auto ivs = make_centered_intervals(robot, width);

        // warmup
        (void)run_benchmark(robot, ivs, cfg_a, n_sub, warmup);
        (void)run_benchmark(robot, ivs, cfg_b, n_sub, warmup);

        const RunAgg agg_a = run_benchmark(robot, ivs, cfg_a, n_sub, repeats);
        const RunAgg agg_b = run_benchmark(robot, ivs, cfg_b, n_sub, repeats);

        const double inv_rep = 1.0 / static_cast<double>(repeats);
        const double a_ms = agg_a.total_ms * inv_rep;
        const double b_ms = agg_b.total_ms * inv_rep;
        const double a_fk = static_cast<double>(agg_a.total_fk) * inv_rep;
        const double b_fk = static_cast<double>(agg_b.total_fk) * inv_rep;

        const auto hit_rate = [](long long raw, long long uni) {
            return (raw > 0) ? (100.0 * static_cast<double>(raw - uni) / static_cast<double>(raw)) : 0.0;
        };

        auto print_row = [&](const char* mode, const RunAgg& agg, double ms, double fk) {
            std::printf("%s,%.2f,%s,"
                "%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,"
                "%.4f,%.4f,%.4f,%.4f,"
                "%.2f,%.2f,%.2f,%.2f,"
                "%.2f,%.2f,%.2f,%.2f,"
                "%.10e\n",
                mode, width, bench_width_profile,
                ms, fk,
                static_cast<double>(agg.p1_fk) * inv_rep,
                static_cast<double>(agg.p2_fk) * inv_rep,
                static_cast<double>(agg.p25_fk) * inv_rep,
                static_cast<double>(agg.p3_fk) * inv_rep,
                hit_rate(agg.d_raw_p1, agg.d_uni_p1),
                hit_rate(agg.d_raw_p2, agg.d_uni_p2),
                hit_rate(agg.d_raw_p25, agg.d_uni_p25),
                hit_rate(agg.d_raw_p3, agg.d_uni_p3),
                static_cast<double>(agg.d_app_p1) * inv_rep,
                static_cast<double>(agg.d_app_p2) * inv_rep,
                static_cast<double>(agg.d_app_p25) * inv_rep,
                static_cast<double>(agg.d_app_p3) * inv_rep,
                static_cast<double>(agg.d_skip_p1) * inv_rep,
                static_cast<double>(agg.d_skip_p2) * inv_rep,
                static_cast<double>(agg.d_skip_p25) * inv_rep,
                static_cast<double>(agg.d_skip_p3) * inv_rep,
                agg.checksum);
        };

        print_row("A_no_dedup", agg_a, a_ms, a_fk);
        print_row("B_dedup_adaptive", agg_b, b_ms, b_fk);

        const double speedup = (b_ms > 1e-12) ? (a_ms / b_ms) : 0.0;
        const double fk_reduction = (a_fk > 1e-12) ? (100.0 * (a_fk - b_fk) / a_fk) : 0.0;
        std::fprintf(stderr,
                     "[width=%.2f] speedup=%.3fx, fk_reduction=%.2f%%\n",
                     width, speedup, fk_reduction);
    }

    return 0;
}
