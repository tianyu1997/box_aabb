// ═══════════════════════════════════════════════════════════════════════════
//  exp_ep_source_gap — Endpoint source signed-gap analysis (Analytical baseline)
//
//  Baseline:
//    Analytical — exact analytical critical-point solver (conservative)
//
//  Compared sources:
//    1. CritSample — critical-grid boundary sampling
//    2. IFK        — interval FK
//    3. MC         — Monte Carlo reference sampling
//
//  Robots:
//    - iiwa14
//    - panda
//
//  Signed gap definition relative to Analytical:
//    hi direction: gap_signed = method_hi - analytical_hi
//    lo direction: gap_signed = analytical_lo - method_lo
//
//  Therefore:
//    gap_signed > 0  => method is looser than Analytical on that side
//    gap_signed < 0  => method is tighter than Analytical on that side
//
//  Scheduling:
//    - 9 width fractions are evaluated separately
//    - each width fraction is repeated 100 times by default
//    - width-band-aware summary/extreme statistics are emitted

//  Output:
//    <out_dir>/results.csv   — full per-trial per-link per-axis signed gaps
//    <out_dir>/summary.csv   — aggregated sign/extreme statistics per robot/source/width_band
//    <out_dir>/extremes.csv  — max positive / max negative / max abs cases per robot/source/width_band
//
//  CSV columns (results.csv):
//    robot, source, baseline, width_band, width_frac, trial, repeat_idx,
//    link, axis, dir, gap_signed_m, gap_abs_m, baseline_bound, method_bound
//
//  Build:
//    cmake --build build --config Release --target exp_ep_source_gap
//  Run:
//    build\Release\exp_ep_source_gap.exe [repeats_per_width] [seed] [n_mc] [out_dir]
// ═══════════════════════════════════════════════════════════════════════════
#define _USE_MATH_DEFINES
#include <cmath>

#include "sbf/robot/robot.h"
#include "sbf/envelope/endpoint_source.h"
#include "sbf/envelope/mc_envelope.h"
#include "sbf/core/types.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <random>
#include <string>
#include <vector>

using namespace sbf;
using namespace sbf::envelope;
namespace fs = std::filesystem;

// ─── IIWA14 ──────────────────────────────────────────────────────────────────

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

static Robot make_panda() {
    std::vector<DHParam> dh = {
        {  0.0,     0.0,     0.333,  0.0, 0},
        { -HALF_PI, 0.0,     0.0,    0.0, 0},
        {  HALF_PI, 0.0,     0.316,  0.0, 0},
        {  HALF_PI, 0.0825,  0.0,    0.0, 0},
        { -HALF_PI,-0.0825,  0.384,  0.0, 0},
        {  HALF_PI, 0.0,     0.0,    0.0, 0},
        {  HALF_PI, 0.088,   0.0,    0.0, 0},
    };
    JointLimits limits;
    limits.limits = {
        {-2.8973, 2.8973}, {-1.7628, 1.7628}, {-2.8973, 2.8973},
        {-3.0718,-0.0698}, {-2.8973, 2.8973}, {-0.0175, 3.7525},
        {-2.8973, 2.8973},
    };
    std::vector<double> radii = {0.08, 0.08, 0.06, 0.06, 0.04, 0.04, 0.04};
    return Robot("panda", dh, limits, std::nullopt, radii);
}

// ─── Box generator ───────────────────────────────────────────────────────────

struct BoxGenerator {
    const Robot& robot;
    std::mt19937& rng;

    std::vector<Interval> generate(double frac) const {
        const auto& lim = robot.joint_limits().limits;
        int n = robot.n_joints();
        std::vector<Interval> ivs(n);
        for (int j = 0; j < n; ++j) {
            double lo = lim[j].lo, hi = lim[j].hi, range = hi - lo;
            std::uniform_real_distribution<double> d(lo, hi);
            double c = d(rng);
            double hw = range * frac * 0.5;
            ivs[j].lo = std::max(lo, c - hw);
            ivs[j].hi = std::min(hi, c + hw);
        }
        return ivs;
    }
};

struct WidthEntry {
    double frac;
    const char* band;
};

static const WidthEntry kWidths[] = {
    {0.05, "narrow"}, {0.10, "narrow"}, {0.15, "narrow"},
    {0.20, "medium"}, {0.30, "medium"}, {0.40, "medium"},
    {0.50, "wide"},   {0.70, "wide"},   {1.00, "wide"},
};

// ─── Timestamp helper ────────────────────────────────────────────────────────

static std::string make_timestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm_val{};
#ifdef _MSC_VER
    localtime_s(&tm_val, &t);
#else
    localtime_r(&t, &tm_val);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y%m%d_%H%M%S", &tm_val);
    return buf;
}

// ─── Signed-gap row writer & aggregate stats ────────────────────────────────

static const char* kAxisNames[] = {"x", "y", "z"};

struct ExtremeRow {
    bool        valid = false;
    const char* kind = "";
    int         trial = -1;
    double      width_frac = 0.0;
    int         link = -1;
    const char* axis = "";
    const char* dir = "";
    double      gap_signed = 0.0;
    double      gap_abs = 0.0;
    double      baseline_bound = 0.0;
    double      method_bound = 0.0;
};

struct GapStats {
    std::string robot;
    std::string source;
    std::string baseline = "Analytical";
    std::string width_band = "all";
    long long   n_rows = 0;
    long long   n_pos = 0;
    long long   n_neg = 0;
    long long   n_zero = 0;
    double      sum_signed = 0.0;
    double      sum_abs = 0.0;
    ExtremeRow  max_pos;
    ExtremeRow  max_neg;
    ExtremeRow  max_abs;
};

static void update_extreme(ExtremeRow& slot,
                           const char* kind,
                           int trial,
                           double width_frac,
                           int link,
                           const char* axis,
                           const char* dir,
                           double gap_signed,
                           double baseline_bound,
                           double method_bound)
{
    slot.valid = true;
    slot.kind = kind;
    slot.trial = trial;
    slot.width_frac = width_frac;
    slot.link = link;
    slot.axis = axis;
    slot.dir = dir;
    slot.gap_signed = gap_signed;
    slot.gap_abs = std::abs(gap_signed);
    slot.baseline_bound = baseline_bound;
    slot.method_bound = method_bound;
}

static void consume_gap(GapStats& stats,
                        int trial,
                        double width_frac,
                        int link,
                        const char* axis,
                        const char* dir,
                        double gap_signed,
                        double baseline_bound,
                        double method_bound)
{
    const double gap_abs = std::abs(gap_signed);
    ++stats.n_rows;
    stats.sum_signed += gap_signed;
    stats.sum_abs += gap_abs;

    const double eps = 1e-12;
    if (gap_signed > eps) ++stats.n_pos;
    else if (gap_signed < -eps) ++stats.n_neg;
    else ++stats.n_zero;

    if (!stats.max_pos.valid || gap_signed > stats.max_pos.gap_signed) {
        update_extreme(stats.max_pos, "max_positive", trial, width_frac,
                       link, axis, dir, gap_signed, baseline_bound, method_bound);
    }
    if (!stats.max_neg.valid || gap_signed < stats.max_neg.gap_signed) {
        update_extreme(stats.max_neg, "max_negative", trial, width_frac,
                       link, axis, dir, gap_signed, baseline_bound, method_bound);
    }
    if (!stats.max_abs.valid || gap_abs > stats.max_abs.gap_abs) {
        update_extreme(stats.max_abs, "max_abs", trial, width_frac,
                       link, axis, dir, gap_signed, baseline_bound, method_bound);
    }
}

static GapStats& get_or_create_stats(std::vector<GapStats>& stats_all,
                                     const std::string& robot,
                                     const std::string& source,
                                     const std::string& width_band)
{
    for (auto& s : stats_all) {
        if (s.robot == robot && s.source == source && s.width_band == width_band)
            return s;
    }
    stats_all.push_back(GapStats{});
    auto& s = stats_all.back();
    s.robot = robot;
    s.source = source;
    s.width_band = width_band;
    return s;
}

static void write_gap_rows(
    std::FILE* csv,
    const char* robot_name,
    const char* source_name,
    const char* width_band,
    int         trial,
    int         repeat_idx,
    double      width_frac,
    int         n_active,
    const int*  alm,
    const float* analytical,
    const float* method,
    GapStats&   stats_all,
    GapStats&   stats_band)
{
    for (int a = 0; a < n_active; ++a) {
        const int abs_link = alm[a];
        for (int d = 0; d < 3; ++d) {
            const float base_lo = analytical[a*6 + d];
            const float meth_lo = method[a*6 + d];
            const float gap_lo = base_lo - meth_lo;

            const float base_hi = analytical[a*6 + 3 + d];
            const float meth_hi = method[a*6 + 3 + d];
            const float gap_hi = meth_hi - base_hi;

            std::fprintf(csv,
                "%s,%s,Analytical,%s,%.3f,%d,%d,%d,%s,lo,%+.6f,%.6f,%.6f,%.6f\n",
                robot_name, source_name, width_band, width_frac, trial, repeat_idx, abs_link,
                kAxisNames[d], (double)gap_lo, std::abs((double)gap_lo),
                (double)base_lo, (double)meth_lo);
            consume_gap(stats_all, trial, width_frac, abs_link, kAxisNames[d], "lo",
                        gap_lo, base_lo, meth_lo);
            consume_gap(stats_band, trial, width_frac, abs_link, kAxisNames[d], "lo",
                        gap_lo, base_lo, meth_lo);

            std::fprintf(csv,
                "%s,%s,Analytical,%s,%.3f,%d,%d,%d,%s,hi,%+.6f,%.6f,%.6f,%.6f\n",
                robot_name, source_name, width_band, width_frac, trial, repeat_idx, abs_link,
                kAxisNames[d], (double)gap_hi, std::abs((double)gap_hi),
                (double)base_hi, (double)meth_hi);
            consume_gap(stats_all, trial, width_frac, abs_link, kAxisNames[d], "hi",
                        gap_hi, base_hi, meth_hi);
            consume_gap(stats_band, trial, width_frac, abs_link, kAxisNames[d], "hi",
                        gap_hi, base_hi, meth_hi);
        }
    }
}

static void write_summary_csv(const std::string& path,
                              const std::vector<GapStats>& stats_all)
{
    std::FILE* fp = std::fopen(path.c_str(), "w");
    if (!fp) return;
    std::fprintf(fp,
        "robot,source,baseline,width_band,n_rows,n_positive,n_negative,n_zero,"
        "mean_gap_signed_m,mean_gap_abs_m,pos_frac,neg_frac,zero_frac,"
        "max_positive_gap_m,max_negative_gap_m,max_abs_gap_m\n");

    for (const auto& s : stats_all) {
        const double n = (s.n_rows > 0) ? static_cast<double>(s.n_rows) : 1.0;
        std::fprintf(fp,
            "%s,%s,%s,%s,%lld,%lld,%lld,%lld,%+.6f,%.6f,%.6f,%.6f,%.6f,%+.6f,%+.6f,%+.6f\n",
            s.robot.c_str(), s.source.c_str(), s.baseline.c_str(), s.width_band.c_str(),
            s.n_rows, s.n_pos, s.n_neg, s.n_zero,
            s.sum_signed / n, s.sum_abs / n,
            s.n_pos / n, s.n_neg / n, s.n_zero / n,
            s.max_pos.valid ? s.max_pos.gap_signed : 0.0,
            s.max_neg.valid ? s.max_neg.gap_signed : 0.0,
            s.max_abs.valid ? s.max_abs.gap_signed : 0.0);
    }
    std::fclose(fp);
}

static void write_extremes_csv(const std::string& path,
                               const std::vector<GapStats>& stats_all)
{
    std::FILE* fp = std::fopen(path.c_str(), "w");
    if (!fp) return;
    std::fprintf(fp,
        "robot,source,baseline,width_band,kind,trial,width_frac,link,axis,dir,"
        "gap_signed_m,gap_abs_m,baseline_bound,method_bound\n");

    for (const auto& s : stats_all) {
        const ExtremeRow rows[] = {s.max_pos, s.max_neg, s.max_abs};
        for (const auto& row : rows) {
            if (!row.valid) continue;
            std::fprintf(fp,
                "%s,%s,%s,%s,%s,%d,%.3f,%d,%s,%s,%+.6f,%.6f,%.6f,%.6f\n",
                s.robot.c_str(), s.source.c_str(), s.baseline.c_str(), s.width_band.c_str(),
                row.kind, row.trial, row.width_frac, row.link, row.axis, row.dir,
                row.gap_signed, row.gap_abs, row.baseline_bound, row.method_bound);
        }
    }
    std::fclose(fp);
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    const int repeats_per_width = (argc > 1) ? std::atoi(argv[1]) : 100;
    const int seed              = (argc > 2) ? std::atoi(argv[2]) : 42;
    const int n_mc              = (argc > 3) ? std::atoi(argv[3]) : 100000;
    const std::string out_dir = (argc > 4)
        ? std::string(argv[4])
        : ("results/exp_ep_source_gap_" + make_timestamp());

    fs::create_directories(out_dir);

    // Endpoint source configs
    EndpointSourceConfig cfg_ifk  = EndpointSourceConfig::ifk();
    EndpointSourceConfig cfg_crit = EndpointSourceConfig::crit_sampling();
    EndpointSourceConfig cfg_anal = EndpointSourceConfig::analytical();

    // Open CSV
    const std::string csv_path = out_dir + "/results.csv";
    std::FILE* csv = std::fopen(csv_path.c_str(), "w");
    if (!csv) {
        std::fprintf(stderr, "[exp] ERROR: cannot open %s\n", csv_path.c_str());
        return 1;
    }
    std::fprintf(csv,
        "robot,source,baseline,width_band,width_frac,trial,repeat_idx,link,axis,dir,"
        "gap_signed_m,gap_abs_m,baseline_bound,method_bound\n");

    std::fprintf(stderr,
        "[exp] repeats_per_width=%d  seed=%d  n_mc=%d  out_dir=%s\n",
        repeats_per_width, seed, n_mc, out_dir.c_str());

    std::vector<Robot> robots;
    robots.push_back(make_iiwa14());
    robots.push_back(make_panda());

    std::vector<GapStats> stats_all;
    stats_all.reserve(32);

    for (size_t ri = 0; ri < robots.size(); ++ri) {
        Robot& robot = robots[ri];
        const int n_act = robot.n_active_links();
        const int* alm = robot.active_link_map();

        std::mt19937 rng(static_cast<uint32_t>(seed + 1000 * static_cast<int>(ri)));
        BoxGenerator gen{robot, rng};

        std::vector<float> mc_iaabb(n_act * 6);
        std::vector<float> ifk_iaabb(n_act * 6);
        std::vector<float> crit_iaabb(n_act * 6);
        std::vector<float> anal_iaabb(n_act * 6);

        std::fprintf(stderr, "[exp] robot=%s  n_active=%d\n",
                     robot.name().c_str(), n_act);

        int trial_id = 0;
        for (const auto& w : kWidths)
        {
            auto& ifk_all = get_or_create_stats(stats_all, robot.name(), "IFK", "all");
            auto& crit_all = get_or_create_stats(stats_all, robot.name(), "CritSample", "all");
            auto& mc_all = get_or_create_stats(stats_all, robot.name(), "MC", "all");
            auto& ifk_band = get_or_create_stats(stats_all, robot.name(), "IFK", w.band);
            auto& crit_band = get_or_create_stats(stats_all, robot.name(), "CritSample", w.band);
            auto& mc_band = get_or_create_stats(stats_all, robot.name(), "MC", w.band);

            for (int repeat_idx = 0; repeat_idx < repeats_per_width; ++repeat_idx, ++trial_id)
            {
                auto intervals = gen.generate(w.frac);

                {
                    auto ep_result = compute_endpoint_iaabb(cfg_anal, robot, intervals);
                    extract_link_iaabbs(ep_result, robot, anal_iaabb.data());
                }

                {
                    auto ep_result = compute_endpoint_iaabb(cfg_ifk, robot, intervals);
                    extract_link_iaabbs(ep_result, robot, ifk_iaabb.data());
                }

                {
                    auto ep_result = compute_endpoint_iaabb(cfg_crit, robot, intervals);
                    extract_link_iaabbs(ep_result, robot, crit_iaabb.data());
                }

                compute_mc_link_iaabb(robot, intervals, n_mc,
                                      static_cast<uint32_t>(seed + 100000 * static_cast<int>(ri) + trial_id),
                                      mc_iaabb.data());

                write_gap_rows(csv, robot.name().c_str(), "IFK", w.band, trial_id, repeat_idx, w.frac,
                               n_act, alm, anal_iaabb.data(), ifk_iaabb.data(), ifk_all, ifk_band);
                write_gap_rows(csv, robot.name().c_str(), "CritSample", w.band, trial_id, repeat_idx, w.frac,
                               n_act, alm, anal_iaabb.data(), crit_iaabb.data(), crit_all, crit_band);
                write_gap_rows(csv, robot.name().c_str(), "MC", w.band, trial_id, repeat_idx, w.frac,
                               n_act, alm, anal_iaabb.data(), mc_iaabb.data(), mc_all, mc_band);

                if ((repeat_idx + 1) % 25 == 0)
                    std::fprintf(stderr, "[exp] robot=%s  width=%s(%.2f)  %d/%d repeats done.\n",
                                 robot.name().c_str(), w.band, w.frac, repeat_idx + 1, repeats_per_width);
            }
        }
    }

    std::fclose(csv);
    std::fprintf(stderr, "[exp] CSV written: %s\n", csv_path.c_str());

    const std::string summary_path = out_dir + "/summary.csv";
    const std::string extremes_path = out_dir + "/extremes.csv";
    write_summary_csv(summary_path, stats_all);
    write_extremes_csv(extremes_path, stats_all);
    std::fprintf(stderr, "[exp] Summary CSV:  %s\n", summary_path.c_str());
    std::fprintf(stderr, "[exp] Extremes CSV: %s\n", extremes_path.c_str());

    for (const auto& s : stats_all) {
        if (s.width_band != "all") continue;
        const double n = (s.n_rows > 0) ? static_cast<double>(s.n_rows) : 1.0;
        std::fprintf(stderr,
            "[summary] robot=%s source=%s band=%s mean=%+.4f abs_mean=%.4f pos=%.2f%% neg=%.2f%% max_pos=%+.4f max_neg=%+.4f max_abs=%+.4f\n",
            s.robot.c_str(), s.source.c_str(), s.width_band.c_str(),
            s.sum_signed / n, s.sum_abs / n,
            100.0 * s.n_pos / n, 100.0 * s.n_neg / n,
            s.max_pos.valid ? s.max_pos.gap_signed : 0.0,
            s.max_neg.valid ? s.max_neg.gap_signed : 0.0,
            s.max_abs.valid ? s.max_abs.gap_signed : 0.0);
    }

    std::fprintf(stderr, "[exp] Done. Results in: %s\n", out_dir.c_str());
    return 0;
}
