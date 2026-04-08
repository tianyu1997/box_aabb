// ═══════════════════════════════════════════════════════════════════════════
//  exp_coverage.cpp — C-space free-volume coverage experiment
//
//  Phase A: MC estimation of free C-space volume
//    - Sample N random configs uniformly from joint limits
//    - Point-FK → link AABBs → collision check against obstacles
//    - V_free = (n_free / N) * V_total
//
//  Phase B: Wavefront expansion coverage
//    - Serial (n_threads=1) and parallel (n_threads=min(4,hw))
//    - CritSample + Hull16_Grid pipeline (freeze_depth=2 auto)
//    - coverage_pct = 100 * total_box_volume / V_free
//
//  Output: stdout summary + exp_coverage_results.csv
//
//  Scene: Marcucci combined (shelves+bins+table, 16 obstacles)
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"
#include "sbf/envelope/frame_source.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/forest/forest_grower.h"
#include "sbf/forest/lect.h"
#include "marcucci_scenes.h"

#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace sbf;
using namespace sbf::envelope;
using namespace sbf::forest;

static const char* ROBOT_PATH =
    "C:/Users/TIAN/OneDrive - The Chinese University of Hong Kong"
    "/Desktop/code/box_aabb/safeboxforest/v1/configs/iiwa14.json";

// ─── Helper: check if a link AABB overlaps an obstacle ──────────────────────
static bool link_aabb_hits_obs(const float* aabb, const Obstacle& obs) {
    float lo[3], hi[3];
    for (int a = 0; a < 3; ++a) {
        lo[a] = static_cast<float>(obs.center[a] - obs.half_sizes[a]);
        hi[a] = static_cast<float>(obs.center[a] + obs.half_sizes[a]);
    }
    if (aabb[3] < lo[0] || hi[0] < aabb[0]) return false;
    if (aabb[4] < lo[1] || hi[1] < aabb[1]) return false;
    if (aabb[5] < lo[2] || hi[2] < aabb[2]) return false;
    return true;
}

// ─── Phase A: Monte Carlo free-volume estimation ────────────────────────────
struct MCResult {
    int    n_samples   = 0;
    int    n_free      = 0;
    double free_ratio  = 0.0;
    double V_total     = 0.0;
    double V_free      = 0.0;
    double mc_time_ms  = 0.0;
};

static MCResult estimate_free_volume(
    const Robot& robot,
    const std::vector<Obstacle>& obstacles,
    int n_samples,
    uint64_t seed)
{
    MCResult mc;
    mc.n_samples = n_samples;

    const auto& lim = robot.joint_limits();
    int ndim = robot.n_joints();
    int n_active = robot.n_active_links();

    // Total C-space volume = product of joint ranges
    mc.V_total = 1.0;
    for (int d = 0; d < ndim; ++d)
        mc.V_total *= lim.limits[d].width();

    // IFK config for point queries (cheapest, exact for point intervals)
    auto ifk_cfg = EndpointSourceConfig::ifk();

    std::mt19937_64 rng(seed);
    std::vector<std::uniform_real_distribution<double>> dists;
    for (int d = 0; d < ndim; ++d)
        dists.emplace_back(lim.limits[d].lo, lim.limits[d].hi);

    auto t0 = std::chrono::high_resolution_clock::now();

    int n_free = 0;
    for (int i = 0; i < n_samples; ++i) {
        // Sample random config
        std::vector<Interval> pt_ivs(ndim);
        for (int d = 0; d < ndim; ++d) {
            double q = dists[d](rng);
            pt_ivs[d] = {q, q};
        }

        // Point FK → endpoint AABBs → link AABBs
        auto ep = compute_endpoint_aabb(ifk_cfg, robot, pt_ivs);

        std::vector<float> link_aabbs(n_active * 6);
        extract_link_aabbs_from_endpoint(ep, robot, link_aabbs.data());

        // Collision check: any link vs any obstacle
        bool collides = false;
        for (int k = 0; k < n_active && !collides; ++k) {
            for (size_t j = 0; j < obstacles.size(); ++j) {
                if (link_aabb_hits_obs(link_aabbs.data() + k * 6, obstacles[j])) {
                    collides = true;
                    break;
                }
            }
        }

        if (!collides) ++n_free;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    mc.mc_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();

    mc.n_free     = n_free;
    mc.free_ratio = static_cast<double>(n_free) / n_samples;
    mc.V_free     = mc.free_ratio * mc.V_total;

    return mc;
}

// ─── Phase B: Wavefront coverage ────────────────────────────────────────────
struct CoverageResult {
    std::string mode;
    int    n_threads        = 0;
    int    n_boxes          = 0;
    int    n_ffb_success    = 0;
    int    n_ffb_fail       = 0;
    double total_volume     = 0.0;
    double coverage_pct     = 0.0;
    int    n_components     = 0;
    double build_time_ms    = 0.0;
    double expand_ms        = 0.0;
    int    lect_nodes       = 0;
    int    lect_hull_voxels = 0;
    int    freeze_depth     = 0;

    // Phase timing breakdown
    std::unordered_map<std::string, double> phase_times;

    // Box volume distribution: boxes stored for post-hoc analysis
    std::vector<BoxNode> boxes;
};

static CoverageResult run_wavefront(
    const Robot& robot,
    const std::vector<Obstacle>& obstacles,
    double V_free,
    const GrowerConfig& cfg,
    const std::string& label)
{
    CoverageResult cr;
    cr.mode      = label;
    cr.n_threads = cfg.n_threads;

    ForestGrower grower(robot, cfg);
    auto result = grower.grow(obstacles.data(),
                              static_cast<int>(obstacles.size()));

    cr.n_boxes          = result.n_boxes_total;
    cr.n_ffb_success    = result.n_ffb_success;
    cr.n_ffb_fail       = result.n_ffb_fail;
    cr.total_volume     = result.total_volume;
    cr.coverage_pct     = (V_free > 0) ? 100.0 * result.total_volume / V_free : 0.0;
    cr.n_components     = result.n_components;
    cr.build_time_ms    = result.build_time_ms;

    auto it = result.phase_times.find("expand_ms");
    cr.expand_ms = (it != result.phase_times.end()) ? it->second : 0.0;

    cr.lect_nodes       = grower.lect().n_nodes();
    cr.lect_hull_voxels = grower.lect().total_hull_voxels();
    cr.freeze_depth     = 2;  // auto from DH params

    // Copy phase times and boxes for analysis
    cr.phase_times = result.phase_times;
    cr.boxes       = result.boxes;

    return cr;
}

// ─── Phase timing breakdown printer ─────────────────────────────────────────
static void print_phase_timing(const CoverageResult& cr) {
    std::printf("\n  Phase timing breakdown:\n");
    // Collect and sort by key
    std::vector<std::pair<std::string, double>> phases(
        cr.phase_times.begin(), cr.phase_times.end());
    std::sort(phases.begin(), phases.end());

    double accounted = 0.0;
    for (auto& [name, ms] : phases) {
        double pct = (cr.build_time_ms > 0) ? 100.0 * ms / cr.build_time_ms : 0.0;
        std::printf("    %-22s %8.1f ms  (%5.1f%%)\n", name.c_str(), ms, pct);
        accounted += ms;
    }
    double other = cr.build_time_ms - accounted;
    if (other > 0.1) {
        double pct = 100.0 * other / cr.build_time_ms;
        std::printf("    %-22s %8.1f ms  (%5.1f%%)\n", "(other/overhead)", other, pct);
    }
    std::printf("    %-22s %8.1f ms\n", "TOTAL", cr.build_time_ms);
}

// ─── Box volume distribution analysis ───────────────────────────────────────
static void print_volume_distribution(const CoverageResult& cr) {
    if (cr.boxes.empty()) return;

    const int ndim = cr.boxes[0].n_dims();
    const int n = static_cast<int>(cr.boxes.size());

    // Collect volumes
    std::vector<double> vols;
    vols.reserve(n);
    for (auto& b : cr.boxes)
        vols.push_back(b.volume);

    std::sort(vols.begin(), vols.end());

    double v_min = vols.front();
    double v_max = vols.back();
    double v_median = vols[n / 2];
    double v_sum = 0.0;
    for (double v : vols) v_sum += v;
    double v_mean = v_sum / n;

    std::printf("\n  Box volume statistics (n=%d):\n", n);
    std::printf("    min=%.3e  median=%.3e  mean=%.3e  max=%.3e\n",
                v_min, v_median, v_mean, v_max);

    // Log10-scale histogram buckets
    double log_min = std::floor(std::log10(v_min + 1e-30));
    double log_max = std::ceil(std::log10(v_max + 1e-30));
    int n_buckets = static_cast<int>(log_max - log_min);
    if (n_buckets < 1) n_buckets = 1;
    if (n_buckets > 20) n_buckets = 20;  // cap

    std::vector<int> counts(n_buckets, 0);
    std::vector<double> bucket_vol(n_buckets, 0.0);  // total volume per bucket
    for (double v : vols) {
        double lv = std::log10(v + 1e-30);
        int idx = static_cast<int>((lv - log_min) / (log_max - log_min) * n_buckets);
        if (idx < 0) idx = 0;
        if (idx >= n_buckets) idx = n_buckets - 1;
        counts[idx]++;
        bucket_vol[idx] += v;
    }

    std::printf("\n  Box volume histogram (log10 scale):\n");
    std::printf("    %-24s %6s  %10s  %6s\n", "Volume range", "Count", "Sum vol", "Vol%%");
    int max_count = *std::max_element(counts.begin(), counts.end());
    for (int i = 0; i < n_buckets; ++i) {
        double lo_exp = log_min + i * (log_max - log_min) / n_buckets;
        double hi_exp = log_min + (i + 1) * (log_max - log_min) / n_buckets;
        double vol_pct = (v_sum > 0) ? 100.0 * bucket_vol[i] / v_sum : 0.0;
        int bar_len = (max_count > 0) ? (counts[i] * 30 / max_count) : 0;
        std::string bar(bar_len, '#');

        std::printf("    [1e%+.0f, 1e%+.0f) %6d  %10.3e  %5.1f%%  %s\n",
                    lo_exp, hi_exp, counts[i], bucket_vol[i], vol_pct, bar.c_str());
    }

    // Per-dimension edge width statistics
    std::printf("\n  Per-dimension edge width (min / median / max):\n");
    for (int d = 0; d < ndim; ++d) {
        std::vector<double> widths;
        widths.reserve(n);
        for (auto& b : cr.boxes)
            widths.push_back(b.joint_intervals[d].width());
        std::sort(widths.begin(), widths.end());
        std::printf("    q%d: %.4f / %.4f / %.4f\n",
                    d, widths.front(), widths[n / 2], widths.back());
    }
}

// ═════════════════════════════════════════════════════════════════════════════
// Helper: make baseline GrowerConfig
// ═════════════════════════════════════════════════════════════════════════════
static GrowerConfig make_baseline_config() {
    GrowerConfig cfg;
    cfg.mode                = GrowerConfig::Mode::Wavefront;
    cfg.n_threads           = 1;
    cfg.pipeline            = PipelineConfig::recommended();  // CritSample + Hull16_Grid
    cfg.max_boxes           = 500;
    cfg.min_edge            = 0.01;
    cfg.max_depth           = 30;
    cfg.timeout             = 120.0;
    cfg.rng_seed            = 42;
    cfg.n_roots             = 2;
    cfg.n_boundary_samples  = 6;
    cfg.goal_face_bias      = 0.6;
    cfg.max_consecutive_miss = 200;
    return cfg;
}

// Helper: print result + phase timing + volume distribution
static void print_result(const CoverageResult& r) {
    std::printf("  boxes=%d  ffb_ok=%d  ffb_fail=%d\n",
                r.n_boxes, r.n_ffb_success, r.n_ffb_fail);
    std::printf("  volume=%.6e  coverage=%.4f%%\n",
                r.total_volume, r.coverage_pct);
    std::printf("  components=%d  time=%.1f ms  expand=%.1f ms\n",
                r.n_components, r.build_time_ms, r.expand_ms);
    std::printf("  lect_nodes=%d  hull_voxels=%d  freeze_depth=%d\n",
                r.lect_nodes, r.lect_hull_voxels, r.freeze_depth);
    print_phase_timing(r);
    print_volume_distribution(r);
}

// ═════════════════════════════════════════════════════════════════════════════
int main()
{
    // ── Load robot ──────────────────────────────────────────────────────
    std::printf("Loading robot from: %s\n", ROBOT_PATH);
    Robot robot = Robot::from_json(ROBOT_PATH);
    std::printf("  n_joints=%d  n_active_links=%d  has_tool=%d\n",
                robot.n_joints(), robot.n_active_links(),
                robot.has_tool() ? 1 : 0);

    // ── Load scene ──────────────────────────────────────────────────────
    auto obstacles = make_combined_obstacles();
    std::printf("Scene: combined (%d obstacles)\n",
                static_cast<int>(obstacles.size()));

    // ── Phase A: MC free-volume estimation ──────────────────────────────
    const int MC_SAMPLES = 100000;
    std::printf("\n═══ Phase A: MC free-volume estimation (%d samples) ═══\n",
                MC_SAMPLES);

    MCResult mc = estimate_free_volume(robot, obstacles, MC_SAMPLES, 12345);

    std::printf("  n_free=%d / %d  free_ratio=%.4f\n",
                mc.n_free, mc.n_samples, mc.free_ratio);
    std::printf("  V_total=%.6e  V_free=%.6e\n", mc.V_total, mc.V_free);
    std::printf("  MC time: %.1f ms\n", mc.mc_time_ms);

    // ═══════════════════════════════════════════════════════════════════
    //  Phase B: Optimization Experiments A-F (serial, n_threads=1)
    //
    //  A: Baseline (current defaults — P1 priority queue is always on)
    //  B: P0 (adaptive_min_edge=true)
    //  C: P0 + P4 (+ coarsen)
    //  D: P0 + P3 (+ root_min_edge=0.2)
    //  E: P0 + P5 (+ hull_skip_vol=1e-6)
    //  F: P0 + P3 + P4 + P5 (all opts serial)
    //  G: F  + P6 (all opts parallel, warm_start_depth=5)
    // ═══════════════════════════════════════════════════════════════════
    std::printf("\n═══ Phase B: Optimization experiments (max_boxes=500) ═══\n");

    std::vector<std::pair<std::string, GrowerConfig>> experiments;

    // A: Baseline
    {
        auto cfg = make_baseline_config();
        experiments.push_back({"A-Baseline", cfg});
    }

    // B: P0 — adaptive_min_edge
    {
        auto cfg = make_baseline_config();
        cfg.adaptive_min_edge = true;
        cfg.coarse_min_edge   = 0.1;
        cfg.coarse_fraction   = 0.6;
        experiments.push_back({"B-Adaptive", cfg});
    }

    // C: P0 + P4 — adaptive + coarsen (DISABLED: coarsen UB TBD)
    {
        auto cfg = make_baseline_config();
        cfg.adaptive_min_edge  = true;
        cfg.coarse_min_edge    = 0.1;
        cfg.coarse_fraction    = 0.6;
        // cfg.coarsen_enabled    = true;   // disabled: coarsen UB TBD
        // cfg.coarsen_target_boxes = 200;
        experiments.push_back({"C-Adapt-NoCoarsen", cfg});
    }

    // D: P0 + P3 — adaptive + fast root
    {
        auto cfg = make_baseline_config();
        cfg.adaptive_min_edge = true;
        cfg.coarse_min_edge   = 0.1;
        cfg.coarse_fraction   = 0.6;
        cfg.root_min_edge     = 0.2;
        experiments.push_back({"D-Adapt+FastRoot", cfg});
    }

    // E: P0 + P5 — adaptive + hull skip
    {
        auto cfg = make_baseline_config();
        cfg.adaptive_min_edge = true;
        cfg.coarse_min_edge   = 0.1;
        cfg.coarse_fraction   = 0.6;
        cfg.hull_skip_vol     = 1e-6;
        experiments.push_back({"E-Adapt+HullSkip", cfg});
    }

    // F: P0 + P3 + P5 — serial opts (without coarsen)
    {
        auto cfg = make_baseline_config();
        cfg.adaptive_min_edge  = true;
        cfg.coarse_min_edge    = 0.1;
        cfg.coarse_fraction    = 0.6;
        cfg.root_min_edge      = 0.2;
        cfg.hull_skip_vol      = 1e-6;
        // cfg.coarsen_enabled    = true;   // disabled: coarsen UB TBD
        // cfg.coarsen_target_boxes = 200;
        experiments.push_back({"F-NoCoarsen", cfg});
    }

    // G: All opts + parallel — SKIPPED (parallel coarsen crash, bug TBD)
#if 0
    {
        int hw_threads = static_cast<int>(std::thread::hardware_concurrency());
        int par_threads = std::min(4, std::max(2, hw_threads));

        auto cfg = make_baseline_config();
        cfg.adaptive_min_edge  = true;
        cfg.coarse_min_edge    = 0.1;
        cfg.coarse_fraction    = 0.6;
        cfg.root_min_edge      = 0.2;
        cfg.hull_skip_vol      = 1e-6;
        cfg.coarsen_enabled    = true;
        cfg.coarsen_target_boxes = 200;
        cfg.n_threads           = par_threads;
        cfg.warm_start_depth    = 5;  // P6: deeper pre-expand
        experiments.push_back({"G-AllOpts-Par" + std::to_string(par_threads), cfg});
    }
#endif

    // H: Multi-stage adaptive — SKIPPED (crashes during coarsen, bug TBD)
#if 0
    {
        int hw_threads = static_cast<int>(std::thread::hardware_concurrency());
        int par_threads = std::min(4, std::max(2, hw_threads));

        auto cfg = make_baseline_config();
        cfg.adaptive_min_edge    = true;
        cfg.coarse_min_edge      = 0.16;
        cfg.coarse_fraction      = 0.6;
        cfg.adaptive_n_stages    = 5;
        cfg.root_min_edge        = 0.2;
        cfg.hull_skip_vol        = 1e-6;
        cfg.coarsen_enabled      = true;
        cfg.coarsen_target_boxes = 200;
        cfg.n_threads            = par_threads;
        cfg.n_roots              = par_threads;
        cfg.warm_start_depth     = 5;
        experiments.push_back({"H-MultiStage-Par" + std::to_string(par_threads), cfg});
    }
#endif

    // I: Same as F (all opts serial) but with offline LECT cache
    //    Compares with F to isolate the cache speedup on root_select.
    {
        // Build cache if not already present
        const std::string cache_dir = "lect_cache";
        namespace fs = std::filesystem;
        if (!fs::exists(cache_dir + "/lect.hcache")) {
            std::printf("\n[cache] Building offline LECT cache (depth 8)...\n");
            auto t_cb0 = std::chrono::high_resolution_clock::now();
            auto pipe = PipelineConfig::recommended();
            LECT cache_lect(robot, pipe);
            int n_new = cache_lect.pre_expand(8);
            fs::create_directories(cache_dir);
            cache_lect.save(cache_dir);
            auto t_cb1 = std::chrono::high_resolution_clock::now();
            double cb_ms = std::chrono::duration<double, std::milli>(t_cb1 - t_cb0).count();
            std::printf("[cache] Built: %d nodes (%d new), saved to %s (%.1f ms)\n",
                        cache_lect.n_nodes(), n_new, cache_dir.c_str(), cb_ms);
        } else {
            std::printf("\n[cache] Found existing LECT cache: %s\n", cache_dir.c_str());
        }

        auto cfg = make_baseline_config();
        cfg.adaptive_min_edge    = true;
        cfg.coarse_min_edge      = 0.1;
        cfg.coarse_fraction      = 0.6;
        cfg.root_min_edge        = 0.2;
        cfg.hull_skip_vol        = 1e-6;
        // cfg.coarsen_enabled      = true;   // disabled: coarsen UB TBD
        // cfg.coarsen_target_boxes = 200;
        cfg.lect_cache_dir       = cache_dir;   // ← use offline cache
        experiments.push_back({"I-Cached-NoCoarsen", cfg});
    }

    std::vector<CoverageResult> results;

    for (auto& [name, cfg] : experiments) {
        std::printf("\n─── %s (threads=%d) ───\n", name.c_str(), cfg.n_threads);
        auto cr = run_wavefront(robot, obstacles, mc.V_free, cfg, name);
        print_result(cr);
        results.push_back(std::move(cr));
    }

    // ── Write CSV ───────────────────────────────────────────────────────
    const char* csv_path = "exp_coverage_results.csv";
    std::ofstream csv(csv_path);
    csv << "# mc_samples=" << MC_SAMPLES
        << ", free_ratio=" << mc.free_ratio
        << ", V_total=" << mc.V_total
        << ", V_free=" << mc.V_free
        << ", mc_time_ms=" << mc.mc_time_ms
        << "\n";
    csv << "mode,n_threads,n_boxes,n_ffb_success,n_ffb_fail,"
           "total_volume,coverage_pct,n_components,"
           "build_time_ms,expand_ms,lect_nodes,lect_hull_voxels,freeze_depth\n";
    for (auto& r : results) {
        csv << r.mode << ","
            << r.n_threads << ","
            << r.n_boxes << ","
            << r.n_ffb_success << ","
            << r.n_ffb_fail << ","
            << r.total_volume << ","
            << r.coverage_pct << ","
            << r.n_components << ","
            << r.build_time_ms << ","
            << r.expand_ms << ","
            << r.lect_nodes << ","
            << r.lect_hull_voxels << ","
            << r.freeze_depth << "\n";
    }
    csv.close();

    std::printf("\n═══ Results written to %s ═══\n", csv_path);

    // ── Summary table ───────────────────────────────────────────────────
    std::printf("\n%-22s %6s %7s %10s %10s %8s %8s\n",
                "Mode", "Boxes", "FFBok", "Volume", "Cover%%", "Time ms", "Comps");
    std::printf("─────────────────────────────────────────────────────────────────────────\n");
    for (auto& r : results) {
        std::printf("%-22s %6d %7d %10.4e %9.4f%% %8.1f %8d\n",
                    r.mode.c_str(), r.n_boxes, r.n_ffb_success,
                    r.total_volume, r.coverage_pct,
                    r.build_time_ms, r.n_components);
    }
    std::printf("─────────────────────────────────────────────────────────────────────────\n");

    return 0;
}
