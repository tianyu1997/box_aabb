// SafeBoxForest v5 — Diagnostic: SplitOrder comparison
//
// Compares BEST_TIGHTEN vs ROUND_ROBIN (cyclic bisection) on identical
// seeds and scenes.  For each (scene, seed) pair, two independent LECT
// trees are constructed — one per split order — and FFB is called with
// the same seed.  Volume, depth, timing, and FK calls are recorded.
//
// Cold mode only (fresh LECT per FFB call) to eliminate cache effects.
// Outputs CSV to stdout.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>
#include <sbf/lect/lect.h>
#include <sbf/ffb/ffb.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/envelope/gcpc_source.h>

#include <Eigen/Core>
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
//  Configuration
// ═══════════════════════════════════════════════════════════════════════════

static constexpr uint64_t MASTER_SEED = 42;
static constexpr int N_SEEDS   = 50;     // seeds per (hw, scene)
static constexpr int N_SCENES  = 5;      // independent obstacle scenes per hw
static constexpr int N_OBS     = 5;
static constexpr int FFB_MAX_DEPTH = 14;
static constexpr double FFB_MIN_EDGE = 0.005;

static const char* split_order_name(SplitOrder so) {
    switch (so) {
        case SplitOrder::ROUND_ROBIN:  return "ROUND_ROBIN";
        case SplitOrder::WIDEST_FIRST: return "WIDEST_FIRST";
        case SplitOrder::BEST_TIGHTEN: return "BEST_TIGHTEN";
    }
    return "?";
}

// ═══════════════════════════════════════════════════════════════════════════
//  Helpers  (adapted from bench_ffb.cpp)
// ═══════════════════════════════════════════════════════════════════════════

static std::vector<Interval> make_root_intervals(
    const Robot& robot, double half_width, std::mt19937& rng)
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

static Eigen::VectorXd random_seed_in(
    const std::vector<Interval>& ivs, std::mt19937& rng)
{
    int n = static_cast<int>(ivs.size());
    Eigen::VectorXd s(n);
    for (int j = 0; j < n; ++j) {
        std::uniform_real_distribution<double> d(ivs[j].lo, ivs[j].hi);
        s[j] = d(rng);
    }
    return s;
}

static std::vector<Obstacle> make_obstacles(
    const Robot& robot, const std::vector<Interval>& ivs,
    int n_obs, std::mt19937& rng)
{
    int n = robot.n_joints();
    int n_act = robot.n_active_links();
    std::vector<Interval> mid_ivs(n);
    for (int j = 0; j < n; ++j) {
        double c = (ivs[j].lo + ivs[j].hi) * 0.5;
        mid_ivs[j] = Interval(c, c);
    }
    FKState fk = compute_fk_full(robot, mid_ivs);
    std::vector<float> link_boxes(n_act * 6);
    extract_link_aabbs(fk, robot.active_link_map(), n_act,
                       link_boxes.data(), robot.active_link_radii());

    std::uniform_int_distribution<int> link_dist(0, n_act - 1);
    std::uniform_real_distribution<float> shift(-0.5f, 0.5f);
    std::uniform_real_distribution<float> sz(0.05f, 0.15f);

    std::vector<Obstacle> obs(n_obs);
    for (int i = 0; i < n_obs; ++i) {
        int li = link_dist(rng);
        const float* lb = link_boxes.data() + li * 6;
        float cx = (lb[0] + lb[3]) * 0.5f + shift(rng);
        float cy = (lb[1] + lb[4]) * 0.5f + shift(rng);
        float cz = (lb[2] + lb[5]) * 0.5f + shift(rng);
        float hw = sz(rng);
        obs[i] = Obstacle(cx - hw, cy - hw, cz - hw,
                          cx + hw, cy + hw, cz + hw);
    }
    return obs;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Per-call result row
// ═══════════════════════════════════════════════════════════════════════════

struct Row {
    const char* robot_name;
    const char* split_str;
    double      half_width;
    int         scene_idx;
    int         seed_idx;
    int         success;
    int         fail_code;
    int         depth;
    int         n_fk_calls;
    int         n_new_nodes;
    double      time_us;
    double      box_volume;
    int         lect_n_nodes;
};

// ═══════════════════════════════════════════════════════════════════════════
//  Runner
// ═══════════════════════════════════════════════════════════════════════════

static void run_split_compare(
    const Robot& robot,
    const GcpcCache& gcpc_cache,
    const char* robot_name,
    const double* half_widths, int n_hw,
    std::vector<Row>& results)
{
    FFBConfig ffb_cfg;
    ffb_cfg.max_depth = FFB_MAX_DEPTH;
    ffb_cfg.min_edge  = FFB_MIN_EDGE;

    // Fixed envelope pipeline (Analytical + LinkIAABB for speed)
    EndpointSourceConfig ep_cfg;
    ep_cfg.source = EndpointSource::Analytical;
    ep_cfg.max_phase_analytical = 3;
    ep_cfg.gcpc_cache = &gcpc_cache;

    EnvelopeTypeConfig env_cfg;
    env_cfg.type = EnvelopeType::LinkIAABB;

    const SplitOrder orders[] = {SplitOrder::ROUND_ROBIN, SplitOrder::BEST_TIGHTEN};
    const int N_ORDER = 2;

    for (int hw_i = 0; hw_i < n_hw; ++hw_i) {
        double hw = half_widths[hw_i];

        for (int sc = 0; sc < N_SCENES; ++sc) {
            // Generate root intervals and obstacles
            std::mt19937 rng_scene(MASTER_SEED + hw_i * 10000 + sc * 100);
            auto root_ivs = make_root_intervals(robot, hw, rng_scene);
            auto obs = make_obstacles(robot, root_ivs, N_OBS, rng_scene);

            // Pre-generate seeds (shared across split orders)
            std::mt19937 rng_seeds(MASTER_SEED + hw_i * 20000 + sc * 200);
            std::vector<Eigen::VectorXd> seeds(N_SEEDS);
            for (int s = 0; s < N_SEEDS; ++s)
                seeds[s] = random_seed_in(root_ivs, rng_seeds);

            for (int oi = 0; oi < N_ORDER; ++oi) {
                SplitOrder so = orders[oi];
                const char* so_str = split_order_name(so);

                for (int s = 0; s < N_SEEDS; ++s) {
                    // Fresh LECT per call — cold mode
                    LECT lect(robot, root_ivs, ep_cfg, env_cfg);
                    lect.set_split_order(so);

                    auto t0 = Clock::now();
                    FFBResult ffb = find_free_box(
                        lect, seeds[s], obs.data(), N_OBS, ffb_cfg);
                    auto t1 = Clock::now();

                    Row row;
                    row.robot_name  = robot_name;
                    row.split_str   = so_str;
                    row.half_width  = hw;
                    row.scene_idx   = sc;
                    row.seed_idx    = s;
                    row.success     = ffb.success() ? 1 : 0;
                    row.fail_code   = ffb.fail_code;
                    row.depth       = static_cast<int>(ffb.path.size()) - 1;
                    row.n_fk_calls  = ffb.n_fk_calls;
                    row.n_new_nodes = ffb.n_new_nodes;
                    row.time_us     = std::chrono::duration<double, std::micro>(
                                          t1 - t0).count();
                    if (ffb.success()) {
                        auto biv = lect.node_intervals(ffb.node_idx);
                        double vol = 1.0;
                        for (auto& iv : biv) vol *= iv.width();
                        row.box_volume = vol;
                    } else {
                        row.box_volume = 0.0;
                    }
                    row.lect_n_nodes = lect.n_nodes();
                    results.push_back(row);
                }
            }

            std::fprintf(stderr, "  [%s] hw=%.3f scene=%d done\n",
                         robot_name, hw, sc);
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Main
// ═══════════════════════════════════════════════════════════════════════════

int main() {
    std::fprintf(stderr, "SafeBoxForest v5 — SplitOrder Comparison (BT vs RR)\n");
    std::fprintf(stderr, "N_SEEDS=%d  N_SCENES=%d  max_depth=%d  min_edge=%.0e\n\n",
                 N_SEEDS, N_SCENES, FFB_MAX_DEPTH, FFB_MIN_EDGE);

    // CSV header
    std::printf("robot,split_order,half_width,scene,seed,"
                "success,fail_code,depth,n_fk,n_new,time_us,box_volume,lect_nodes\n");

    std::vector<Row> results;
    results.reserve(20000);

    // ── 2DOF planar ─────────────────────────────────────────────────────
    {
        std::fprintf(stderr, "Loading 2dof_planar ...\n");
        auto robot = Robot::from_json("data/2dof_planar.json");
        GcpcCache gcpc;  // empty — Analytical doesn't need it
        std::fprintf(stderr, "  n_joints=%d  n_active=%d\n",
                     robot.n_joints(), robot.n_active_links());

        const double hws[] = {0.1, 0.2, 0.4, 0.8, 1.2, 1.57};
        run_split_compare(robot, gcpc, "2dof", hws, 6, results);
    }

    // ── 7DOF Panda ──────────────────────────────────────────────────────
    {
        std::fprintf(stderr, "Loading panda ...\n");
        auto robot = Robot::from_json("data/panda.json");
        GcpcCache gcpc;
        std::fprintf(stderr, "  n_joints=%d  n_active=%d\n",
                     robot.n_joints(), robot.n_active_links());

        const double hws[] = {0.05, 0.1, 0.2, 0.4};
        run_split_compare(robot, gcpc, "panda", hws, 4, results);
    }

    // ── Flush CSV ───────────────────────────────────────────────────────
    for (auto& r : results) {
        std::printf("%s,%s,%.4f,%d,%d,%d,%d,%d,%d,%d,%.1f,%.10g,%d\n",
                    r.robot_name, r.split_str, r.half_width,
                    r.scene_idx, r.seed_idx,
                    r.success, r.fail_code, r.depth,
                    r.n_fk_calls, r.n_new_nodes, r.time_us,
                    r.box_volume, r.lect_n_nodes);
    }

    // ── Quick summary to stderr ─────────────────────────────────────────
    for (const char* rname : {"2dof", "panda"}) {
        for (const char* so : {"ROUND_ROBIN", "BEST_TIGHTEN"}) {
            int cnt = 0, succ = 0;
            double vol_sum = 0, time_sum = 0;
            int fk_sum = 0;
            for (auto& r : results) {
                if (std::strcmp(r.robot_name, rname) != 0) continue;
                if (std::strcmp(r.split_str, so) != 0) continue;
                cnt++;
                succ += r.success;
                time_sum += r.time_us;
                fk_sum += r.n_fk_calls;
                if (r.success) vol_sum += r.box_volume;
            }
            if (cnt == 0) continue;
            std::fprintf(stderr, "[%s][%s] n=%d succ=%d avg_time=%.1fus avg_fk=%.1f avg_vol=%.6g\n",
                         rname, so, cnt, succ,
                         time_sum / cnt, double(fk_sum) / cnt,
                         succ > 0 ? vol_sum / succ : 0.0);
        }
    }

    return 0;
}
