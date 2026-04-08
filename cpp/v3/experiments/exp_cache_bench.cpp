// ═══════════════════════════════════════════════════════════════════════════
//  exp_cache_bench.cpp — Envelope pipeline × cache benchmark
//
//  Compares each pipeline configuration with and without offline LECT cache.
//  Pipelines tested:
//    1. fast()        = IFK + AABB
//    2. production()  = IFK + Hull16_Grid
//    3. recommended() = CritSample + Hull16_Grid
//
//  For each pipeline, two runs are performed:
//    (a) No cache   — build LECT from scratch during grow
//    (b) With cache — pre-built LECT loaded at startup
//
//  Output: summary table + CSV
//
//  Scene: Marcucci combined (16 obstacles)
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"
#include "sbf/envelope/frame_source.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/forest/forest_grower.h"
#include "sbf/forest/lect.h"
#include "marcucci_scenes.h"

#include <Eigen/Core>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace sbf;
using namespace sbf::envelope;
using namespace sbf::forest;
namespace fs = std::filesystem;

static const char* ROBOT_PATH =
    "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong"
    "/Desktop/code/box_aabb/safeboxforest/v1/configs/iiwa14.json";

// ─── Result struct ──────────────────────────────────────────────────────────
struct BenchResult {
    std::string label;
    std::string pipeline_name;
    bool   use_cache     = false;
    int    n_boxes       = 0;
    double total_volume  = 0.0;
    double coverage_pct  = 0.0;
    int    n_components  = 0;
    double total_ms      = 0.0;
    double expand_ms     = 0.0;
    double root_select_ms = 0.0;
    int    lect_nodes    = 0;
};

// ─── Free-volume estimation (reuse from exp_coverage) ───────────────────────
static double estimate_V_free(const Robot& robot,
                              const std::vector<Obstacle>& obstacles,
                              int n_samples, uint64_t seed)
{
    const auto& lim = robot.joint_limits();
    int ndim = robot.n_joints();
    int n_active = robot.n_active_links();

    double V_total = 1.0;
    for (int d = 0; d < ndim; ++d)
        V_total *= lim.limits[d].width();

    auto ifk_cfg = EndpointSourceConfig::ifk();
    std::mt19937_64 rng(seed);
    std::vector<std::uniform_real_distribution<double>> dists;
    for (int d = 0; d < ndim; ++d)
        dists.emplace_back(lim.limits[d].lo, lim.limits[d].hi);

    int n_free = 0;
    for (int i = 0; i < n_samples; ++i) {
        std::vector<Interval> pt_ivs(ndim);
        for (int d = 0; d < ndim; ++d) {
            double q = dists[d](rng);
            pt_ivs[d] = {q, q};
        }
        auto ep = compute_endpoint_aabb(ifk_cfg, robot, pt_ivs);
        std::vector<float> link_aabbs(n_active * 6);
        extract_link_aabbs_from_endpoint(ep, robot, link_aabbs.data());

        bool collides = false;
        for (int k = 0; k < n_active && !collides; ++k) {
            const float* a = link_aabbs.data() + k * 6;
            for (auto& obs : obstacles) {
                float lo[3], hi[3];
                for (int d = 0; d < 3; ++d) {
                    lo[d] = static_cast<float>(obs.center[d] - obs.half_sizes[d]);
                    hi[d] = static_cast<float>(obs.center[d] + obs.half_sizes[d]);
                }
                if (a[3] >= lo[0] && a[0] <= hi[0] &&
                    a[4] >= lo[1] && a[1] <= hi[1] &&
                    a[5] >= lo[2] && a[2] <= hi[2]) {
                    collides = true; break;
                }
            }
        }
        if (!collides) ++n_free;
    }
    return (static_cast<double>(n_free) / n_samples) * V_total;
}

// ─── Build or load per-pipeline LECT cache ──────────────────────────────────
static void ensure_cache(const Robot& robot,
                         const PipelineConfig& pipe,
                         const std::string& cache_dir,
                         int depth)
{
    if (fs::exists(cache_dir + "/lect.hcache")) {
        std::printf("  [cache] reusing: %s\n", cache_dir.c_str());
        return;
    }
    std::printf("  [cache] building depth-%d cache → %s ...\n",
                depth, cache_dir.c_str());
    auto t0 = std::chrono::high_resolution_clock::now();

    LECT lect(robot, pipe);
    int n_new = lect.pre_expand(depth);
    lect.compute_all_hull_grids();
    fs::create_directories(cache_dir);
    lect.save(cache_dir);

    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    std::printf("  [cache] built: %d nodes (%d new), %.1f ms\n",
                lect.n_nodes(), n_new, ms);
}

// ─── Run one configuration ──────────────────────────────────────────────────
static BenchResult run_one(const Robot& robot,
                           const std::vector<Obstacle>& obstacles,
                           double V_free,
                           const std::string& label,
                           const PipelineConfig& pipe,
                           const std::string& pipe_name,
                           bool use_cache,
                           const std::string& cache_dir)
{
    GrowerConfig cfg;
    cfg.mode                 = GrowerConfig::Mode::Wavefront;
    cfg.n_threads            = 1;
    cfg.pipeline             = pipe;
    cfg.max_boxes            = 500;
    cfg.min_edge             = 0.01;
    cfg.max_depth            = 30;
    cfg.timeout              = 120.0;
    cfg.rng_seed             = 42;
    cfg.n_roots              = 2;
    cfg.n_boundary_samples   = 6;
    cfg.goal_face_bias       = 0.6;
    cfg.max_consecutive_miss = 200;
    cfg.adaptive_min_edge    = true;
    cfg.coarse_min_edge      = 0.1;
    cfg.coarse_fraction      = 0.6;
    cfg.root_min_edge        = 0.2;
    cfg.hull_skip_vol        = 1e-6;

    if (use_cache)
        cfg.lect_cache_dir = cache_dir;

    ForestGrower grower(robot, cfg);
    auto result = grower.grow(obstacles.data(),
                              static_cast<int>(obstacles.size()));

    BenchResult br;
    br.label         = label;
    br.pipeline_name = pipe_name;
    br.use_cache     = use_cache;
    br.n_boxes       = result.n_boxes_total;
    br.total_volume  = result.total_volume;
    br.coverage_pct  = (V_free > 0) ? 100.0 * result.total_volume / V_free : 0.0;
    br.n_components  = result.n_components;
    br.total_ms      = result.build_time_ms;
    br.lect_nodes    = grower.lect().n_nodes();

    auto it_e = result.phase_times.find("expand_ms");
    br.expand_ms = (it_e != result.phase_times.end()) ? it_e->second : 0.0;
    auto it_r = result.phase_times.find("root_select_ms");
    br.root_select_ms = (it_r != result.phase_times.end()) ? it_r->second : 0.0;

    return br;
}

// ═════════════════════════════════════════════════════════════════════════════
int main()
{
    // ── Load robot ──────────────────────────────────────────────────────
    std::printf("Loading robot from: %s\n", ROBOT_PATH);
    Robot robot = Robot::from_json(ROBOT_PATH);
    std::printf("  n_joints=%d  n_active_links=%d\n",
                robot.n_joints(), robot.n_active_links());

    // ── Load scene ──────────────────────────────────────────────────────
    auto obstacles = make_combined_obstacles();
    std::printf("Scene: combined (%d obstacles)\n\n",
                static_cast<int>(obstacles.size()));

    // ── MC free-volume ──────────────────────────────────────────────────
    std::printf("═══ MC free-volume estimation (100k samples) ═══\n");
    auto t_mc0 = std::chrono::high_resolution_clock::now();
    double V_free = estimate_V_free(robot, obstacles, 100000, 12345);
    auto t_mc1 = std::chrono::high_resolution_clock::now();
    double mc_ms = std::chrono::duration<double, std::milli>(t_mc1 - t_mc0).count();
    std::printf("  V_free = %.6e  (%.1f ms)\n\n", V_free, mc_ms);

    // ── Define pipelines ────────────────────────────────────────────────
    struct PipeDef {
        std::string name;
        PipelineConfig config;
        std::string cache_dir;
        int cache_depth;
    };

    std::vector<PipeDef> pipelines = {
        { "IFK+SubAABB",     PipelineConfig::fast(),        "cache_ifk_sub_aabb", 8 },
        { "IFK+Hull16Grid",  PipelineConfig::production(),  "cache_ifk_hull16",  8 },
        { "Crit+Hull16Grid", PipelineConfig::recommended(), "cache_crit_hull16", 8 },
    };

    // ── Pre-build all caches ────────────────────────────────────────────
    std::printf("═══ Building per-pipeline caches ═══\n");
    for (auto& pd : pipelines)
        ensure_cache(robot, pd.config, pd.cache_dir, pd.cache_depth);
    std::printf("\n");

    // ── Run benchmarks ──────────────────────────────────────────────────
    std::printf("═══ Running pipeline × cache benchmark (500 boxes each) ═══\n");
    std::vector<BenchResult> results;

    for (auto& pd : pipelines) {
        // (a) No cache
        std::string label_a = pd.name + " (no cache)";
        std::printf("\n─── %s ───\n", label_a.c_str());
        auto r_a = run_one(robot, obstacles, V_free,
                           label_a, pd.config, pd.name, false, "");
        std::printf("  boxes=%d  vol=%.4e  cover=%.2f%%  time=%.1f ms  "
                    "expand=%.1f ms  root_sel=%.1f ms\n",
                    r_a.n_boxes, r_a.total_volume, r_a.coverage_pct,
                    r_a.total_ms, r_a.expand_ms, r_a.root_select_ms);
        results.push_back(r_a);

        // (b) With cache
        std::string label_b = pd.name + " (cached)";
        std::printf("\n─── %s ───\n", label_b.c_str());
        auto r_b = run_one(robot, obstacles, V_free,
                           label_b, pd.config, pd.name, true, pd.cache_dir);
        std::printf("  boxes=%d  vol=%.4e  cover=%.2f%%  time=%.1f ms  "
                    "expand=%.1f ms  root_sel=%.1f ms\n",
                    r_b.n_boxes, r_b.total_volume, r_b.coverage_pct,
                    r_b.total_ms, r_b.expand_ms, r_b.root_select_ms);
        results.push_back(r_b);
    }

    // ── Summary table ───────────────────────────────────────────────────
    std::printf("\n\n═══ Summary ═══\n\n");
    std::printf("%-28s %6s %10s %9s %9s %10s %10s %7s\n",
                "Config", "Boxes", "Volume", "Cover%%",
                "Total ms", "Expand ms", "RSel ms", "Comps");
    std::printf("──────────────────────────────────────────────────"
                "──────────────────────────────────────────────\n");
    for (auto& r : results) {
        std::printf("%-28s %6d %10.4e %8.2f%% %9.1f %10.1f %10.1f %7d\n",
                    r.label.c_str(), r.n_boxes, r.total_volume,
                    r.coverage_pct, r.total_ms, r.expand_ms,
                    r.root_select_ms, r.n_components);
    }
    std::printf("──────────────────────────────────────────────────"
                "──────────────────────────────────────────────\n");

    // ── Speedup summary ─────────────────────────────────────────────────
    std::printf("\n═══ Cache speedup per pipeline ═══\n\n");
    std::printf("%-20s %10s %10s %10s %10s %10s %10s\n",
                "Pipeline", "T_no ms", "T_yes ms", "Speedup",
                "RS_no ms", "RS_yes ms", "RS Speed");
    std::printf("────────────────────────────────────────────────────"
                "────────────────────────────────\n");
    for (size_t i = 0; i + 1 < results.size(); i += 2) {
        auto& no   = results[i];
        auto& yes  = results[i + 1];
        double sp_total = (yes.total_ms > 0) ? no.total_ms / yes.total_ms : 0;
        double sp_rs    = (yes.root_select_ms > 0)
                          ? no.root_select_ms / yes.root_select_ms : 0;
        std::printf("%-20s %10.1f %10.1f %9.2fx %10.1f %10.1f %9.2fx\n",
                    no.pipeline_name.c_str(),
                    no.total_ms, yes.total_ms, sp_total,
                    no.root_select_ms, yes.root_select_ms, sp_rs);
    }
    std::printf("────────────────────────────────────────────────────"
                "────────────────────────────────\n");

    // ── CSV output ──────────────────────────────────────────────────────
    const char* csv_path = "exp_cache_bench_results.csv";
    std::ofstream csv(csv_path);
    csv << "label,pipeline,use_cache,n_boxes,total_volume,coverage_pct,"
           "n_components,total_ms,expand_ms,root_select_ms,lect_nodes\n";
    for (auto& r : results) {
        csv << r.label << ","
            << r.pipeline_name << ","
            << (r.use_cache ? "yes" : "no") << ","
            << r.n_boxes << ","
            << r.total_volume << ","
            << r.coverage_pct << ","
            << r.n_components << ","
            << r.total_ms << ","
            << r.expand_ms << ","
            << r.root_select_ms << ","
            << r.lect_nodes << "\n";
    }
    csv.close();
    std::printf("\nResults written to %s\n", csv_path);

    return 0;
}
