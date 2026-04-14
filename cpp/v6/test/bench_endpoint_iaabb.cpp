// SafeBoxForest v5 — Benchmark: LECT tree expansion per source
// Cold = root-level construction (no cache, no parent).
// Hot  = deeper expansions benefiting from Z4 cache, partial inheritance,
//        and incremental FK.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>
#include <sbf/lect/lect.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/envelope/gcpc_source.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <random>
#include <string>
#include <vector>

using namespace sbf;
using Clock = std::chrono::high_resolution_clock;

// ═══════════════════════════════════════════════════════════════════════════
//  Helpers
// ═══════════════════════════════════════════════════════════════════════════

static double sum_volume_ep(const float* ep, int n_act) {
    double vol = 0.0;
    int n_ep = n_act * 2;
    for (int i = 0; i < n_ep; ++i) {
        const float* a = ep + i * 6;
        double dx = std::max(0.0, double(a[3] - a[0]));
        double dy = std::max(0.0, double(a[4] - a[1]));
        double dz = std::max(0.0, double(a[5] - a[2]));
        vol += dx * dy * dz;
    }
    return vol;
}

static double avg_extent_ep(const float* ep, int n_act) {
    double sum = 0.0;
    int n_ep = n_act * 2;
    for (int i = 0; i < n_ep; ++i) {
        const float* a = ep + i * 6;
        for (int d = 0; d < 3; ++d)
            sum += double(a[d + 3] - a[d]);
    }
    return sum / (n_ep * 3);
}

static std::vector<Interval> random_intervals(
    const Robot& robot, std::mt19937& rng, double half_width)
{
    int n = robot.n_joints();
    const auto& lim = robot.joint_limits().limits;
    std::vector<Interval> ivs(n);
    for (int j = 0; j < n; ++j) {
        double lo = lim[j].lo + half_width;
        double hi = lim[j].hi - half_width;
        if (lo > hi) lo = hi = (lim[j].lo + lim[j].hi) * 0.5;
        std::uniform_real_distribution<double> dist(lo, hi);
        double c = dist(rng);
        ivs[j] = Interval(c - half_width, c + half_width);
    }
    return ivs;
}

static GcpcCache make_gcpc_cache(const Robot& robot, int n_points, uint64_t seed) {
    GcpcCache cache;
    int n = robot.n_joints();
    cache.set_n_dims(n);
    std::mt19937 gen(seed);
    const auto& lim = robot.joint_limits().limits;
    for (int i = 0; i < n_points; ++i) {
        Eigen::VectorXd pt(n);
        for (int j = 0; j < n; ++j) {
            std::uniform_real_distribution<double> d(lim[j].lo, lim[j].hi);
            pt[j] = d(gen);
        }
        cache.add_point(pt);
    }
    return cache;
}

// ═══════════════════════════════════════════════════════════════════════════
//  LECT expansion benchmark  —  expand level by level, time each level
// ═══════════════════════════════════════════════════════════════════════════

struct LevelStats {
    int    depth      = 0;
    int    n_nodes    = 0;     // nodes expanded on this level
    double total_us   = 0.0;  // wall time to expand all leaves at this depth
    double avg_us     = 0.0;  // per-expand_leaf average
    double avg_vol    = 0.0;  // average leaf endpoint volume after expansion
    double avg_extent = 0.0;
};

static std::vector<LevelStats> bench_lect_expansion(
    const Robot& robot,
    const std::vector<Interval>& root_ivs,
    const EndpointSourceConfig& ep_cfg,
    int max_depth)
{
    EnvelopeTypeConfig env_cfg;
    env_cfg.type = EnvelopeType::LinkIAABB;
    env_cfg.n_subdivisions = 1;

    std::vector<LevelStats> stats;

    // Level 0 = constructor (creates root node)
    auto t0 = Clock::now();
    LECT lect(robot, root_ivs, ep_cfg, env_cfg);
    auto t1 = Clock::now();
    double root_us = std::chrono::duration<double, std::micro>(t1 - t0).count();

    int n_act = lect.n_active_links();
    {
        LevelStats ls;
        ls.depth    = 0;
        ls.n_nodes  = 1;
        ls.total_us = root_us;
        ls.avg_us   = root_us;
        ls.avg_vol    = sum_volume_ep(lect.get_endpoint_iaabbs(0), n_act);
        ls.avg_extent = avg_extent_ep(lect.get_endpoint_iaabbs(0), n_act);
        stats.push_back(ls);
    }

    // Expand level by level
    for (int d = 0; d < max_depth; ++d) {
        // Collect current leaves at this depth
        std::vector<int> leaves;
        for (int i = 0; i < lect.n_nodes(); ++i) {
            if (lect.is_leaf(i) && lect.depth(i) == d)
                leaves.push_back(i);
        }
        if (leaves.empty()) break;

        t0 = Clock::now();
        int created = 0;
        for (int leaf : leaves) {
            created += lect.expand_leaf(leaf);
        }
        t1 = Clock::now();
        double level_us = std::chrono::duration<double, std::micro>(t1 - t0).count();

        // Gather volume/extent over new leaves
        double vol_sum = 0.0, ext_sum = 0.0;
        int n_new_leaves = 0;
        for (int i = 0; i < lect.n_nodes(); ++i) {
            if (lect.is_leaf(i) && lect.depth(i) == d + 1 && lect.has_data(i)) {
                vol_sum += sum_volume_ep(lect.get_endpoint_iaabbs(i), n_act);
                ext_sum += avg_extent_ep(lect.get_endpoint_iaabbs(i), n_act);
                ++n_new_leaves;
            }
        }

        LevelStats ls;
        ls.depth    = d + 1;
        ls.n_nodes  = created;
        ls.total_us = level_us;
        ls.avg_us   = (created > 0) ? level_us / created : 0.0;
        ls.avg_vol    = (n_new_leaves > 0) ? vol_sum / n_new_leaves : 0.0;
        ls.avg_extent = (n_new_leaves > 0) ? ext_sum / n_new_leaves : 0.0;
        stats.push_back(ls);
    }

    return stats;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Printing
// ═══════════════════════════════════════════════════════════════════════════

static void print_level_header() {
    std::printf("  %5s %6s %11s %11s %12s %10s\n",
                "Depth", "Nodes", "TotalUs", "AvgUs/node", "AvgVolume", "AvgExtent");
    std::printf("  %5s %6s %11s %11s %12s %10s\n",
                "-----", "-----", "---------", "----------", "----------", "--------");
}

static void print_level(const LevelStats& ls) {
    std::printf("  %5d %6d %11.1f %11.1f %12.6f %10.6f\n",
                ls.depth, ls.n_nodes, ls.total_us, ls.avg_us,
                ls.avg_vol, ls.avg_extent);
}

// ── Per-source cold/hot summary (from one LECT expansion) ──────────────

struct SourceSummary {
    double cold_us    = 0.0;   // depth-0 avg_us
    double hot_us     = 0.0;   // depth>=2 weighted avg_us
    double root_vol   = 0.0;   // root volume
    double root_ext   = 0.0;   // root avg extent
    double leaf_vol   = 0.0;   // deepest-level avg volume
    double leaf_ext   = 0.0;   // deepest-level avg extent
};

static SourceSummary summarise(const std::vector<LevelStats>& levels) {
    SourceSummary s;
    if (levels.empty()) return s;
    s.cold_us  = levels[0].avg_us;
    s.root_vol = levels[0].avg_vol;
    s.root_ext = levels[0].avg_extent;
    s.leaf_vol = levels.back().avg_vol;
    s.leaf_ext = levels.back().avg_extent;
    if (levels.size() >= 3) {
        double hot_sum = 0.0;
        int hot_cnt = 0;
        for (size_t i = 2; i < levels.size(); ++i) {
            hot_sum += levels[i].avg_us * levels[i].n_nodes;
            hot_cnt += levels[i].n_nodes;
        }
        s.hot_us = (hot_cnt > 0) ? hot_sum / hot_cnt : 0.0;
    } else if (levels.size() == 2) {
        s.hot_us = levels[1].avg_us;
    }
    return s;
}

// ── Run one LECT expansion, print detail, return summary ───────────────

static SourceSummary run_one(const Robot& robot,
                             const std::vector<Interval>& root_ivs,
                             const EndpointSourceConfig& cfg,
                             const char* title,
                             int max_depth,
                             bool verbose)
{
    auto levels = bench_lect_expansion(robot, root_ivs, cfg, max_depth);
    if (verbose) {
        std::printf("\n--- %s ---\n", title);
        print_level_header();
        for (auto& ls : levels)
            print_level(ls);
    }
    return summarise(levels);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Width × Source summary table
// ═══════════════════════════════════════════════════════════════════════════

struct WidthSourceEntry {
    double width;
    const char* source_name;
    // accumulated over N_RANDOM runs
    double cold_us_sum  = 0.0;
    double hot_us_sum   = 0.0;
    double root_vol_sum = 0.0;
    double leaf_vol_sum = 0.0;
    double root_ext_sum = 0.0;
    double leaf_ext_sum = 0.0;
    int    count        = 0;
};

static void print_summary_table(const char* robot_name,
                                std::vector<WidthSourceEntry>& table) {
    std::printf("\n+------+------------+---------+---------+-------+-----------+-----------+----------+----------+\n");
    std::printf("| Summary: %-80s |\n", robot_name);
    std::printf("+------+------------+---------+---------+-------+-----------+-----------+----------+----------+\n");
    std::printf("| %5s %-12s %9s %9s %7s %11s %11s %10s %10s|\n",
                "Width", "Source", "Cold(us)", "Hot(us)", "Speedup",
                "RootVol", "LeafVol", "RootExt", "LeafExt");
    std::printf("+------+------------+---------+---------+-------+-----------+-----------+----------+----------+\n");

    for (auto& e : table) {
        if (e.count == 0) continue;
        double n = e.count;
        double cold = e.cold_us_sum / n;
        double hot  = e.hot_us_sum / n;
        double spd  = (hot > 0) ? cold / hot : 0.0;
        double rv   = e.root_vol_sum / n;
        double lv   = e.leaf_vol_sum / n;
        double re   = e.root_ext_sum / n;
        double le   = e.leaf_ext_sum / n;
        std::printf("| %5.2f %-12s %9.1f %9.1f %6.2fx %11.4f %11.4f %10.6f %10.6f|\n",
                    e.width, e.source_name, cold, hot, spd, rv, lv, re, le);
    }
    std::printf("+------+------------+---------+---------+-------+-----------+-----------+----------+----------+\n");
}

// ── Full benchmark for one robot ───────────────────────────────────────

static const struct { const char* name; EndpointSource source; } SOURCES[] = {
    {"IFK",        EndpointSource::IFK},
    {"CritSample", EndpointSource::CritSample},
    {"Analytical", EndpointSource::Analytical},
    {"GCPC",       EndpointSource::GCPC},
};
static constexpr int N_SOURCES = 4;

static void bench_robot(const Robot& robot,
                        const GcpcCache& cache,
                        const char* robot_name,
                        const double* half_widths, int n_hw,
                        int n_random, int max_depth,
                        uint64_t seed, bool verbose)
{
    // Prepare summary table: n_hw × N_SOURCES entries
    std::vector<WidthSourceEntry> table;
    for (int hw = 0; hw < n_hw; ++hw) {
        for (int si = 0; si < N_SOURCES; ++si) {
            WidthSourceEntry e;
            e.width = half_widths[hw] * 2;
            e.source_name = SOURCES[si].name;
            table.push_back(e);
        }
    }

    for (int hw = 0; hw < n_hw; ++hw) {
        double half = half_widths[hw];
        std::mt19937 rng(seed);

        for (int s = 0; s < n_random; ++s) {
            auto ivs = random_intervals(robot, rng, half);

            for (int si = 0; si < N_SOURCES; ++si) {
                EndpointSourceConfig cfg;
                cfg.source = SOURCES[si].source;
                cfg.n_samples_crit = 1000;
                cfg.max_phase_analytical = 3;
                cfg.gcpc_cache = &cache;

                char title[256];
                std::snprintf(title, sizeof(title), "%s w=%.2f #%d / %s",
                              robot_name, half * 2, s + 1, SOURCES[si].name);

                auto sm = run_one(robot, ivs, cfg, title, max_depth, verbose);

                auto& te = table[hw * N_SOURCES + si];
                te.cold_us_sum  += sm.cold_us;
                te.hot_us_sum   += sm.hot_us;
                te.root_vol_sum += sm.root_vol;
                te.leaf_vol_sum += sm.leaf_vol;
                te.root_ext_sum += sm.root_ext;
                te.leaf_ext_sum += sm.leaf_ext;
                te.count++;
            }
        }
    }

    print_summary_table(robot_name, table);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Main
// ═══════════════════════════════════════════════════════════════════════════

int main() {
    std::printf("SafeBoxForest v5 — LECT Expansion Benchmark (cold/hot)\n");
    std::printf("======================================================\n");
    std::printf("Cold = root construction (no cache/parent).\n");
    std::printf("Hot  = deeper expansions (Z4 cache, partial inheritance, incr FK).\n");

    constexpr uint64_t SEED = 42;
    constexpr int N_RANDOM = 3;
    constexpr bool VERBOSE = false;  // set true for per-level detail

    // ── 2DOF ────────────────────────────────────────────────────────────
    {
        auto robot = Robot::from_json("data/2dof_planar.json");
        auto cache = make_gcpc_cache(robot, 200, SEED);

        std::printf("\n\n>>> Robot: 2dof_planar  (n_joints=%d, n_active=%d)\n",
                    robot.n_joints(), robot.n_active_links());

        const double HW[] = {0.05, 0.15, 0.25, 0.5, 1.0};
        bench_robot(robot, cache, "2DOF", HW, 5, N_RANDOM,
                    /*max_depth=*/6, SEED, VERBOSE);
    }

    // ── 7DOF Panda ──────────────────────────────────────────────────────
    {
        auto robot = Robot::from_json("data/panda.json");
        auto cache = make_gcpc_cache(robot, 500, SEED);

        std::printf("\n\n>>> Robot: panda_7dof  (n_joints=%d, n_active=%d, tool=%d)\n",
                    robot.n_joints(), robot.n_active_links(),
                    robot.has_tool() ? 1 : 0);

        const double HW[] = {0.05, 0.15, 0.25, 0.5};
        bench_robot(robot, cache, "Panda7DOF", HW, 4, N_RANDOM,
                    /*max_depth=*/4, SEED, VERBOSE);
    }

    std::printf("\n\nDone.\n");
    return 0;
}
