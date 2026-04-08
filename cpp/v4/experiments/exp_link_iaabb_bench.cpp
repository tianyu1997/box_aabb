// ═══════════════════════════════════════════════════════════════════════════
//  exp_link_iaabb_bench — Link iAABB 方法耗时 & 体积对比实验
//
//  端点源固定 CritSample (Stage 1)，对以下 5 种 Stage 2 方法进行系统性对比：
//    1. LinkIAABB(sub=1)         — 每连杆 1 个 iAABB
//    2. LinkIAABB(sub=4)         — 每连杆 4 分段 iAABB
//    3. LinkIAABB(sub=16)        — 每连杆 16 分段 iAABB
//    4. LinkIAABB_Grid(n_sub=16) — 字节栅格
//    5. Hull16_Grid(δ=0.01)      — 稀疏体素格（最紧密）
//
//  测量指标：
//    - Stage 1 耗时 (ms, 中位数，5次重复)
//    - Stage 2 耗时 (ms, 中位数，5次重复)
//    - 总耗时 = Stage1 + Stage2 (ms)
//    - 体积 (m³)    [EnvelopeResult.volume]
//
//  输出：
//    <out_dir>/results.csv           — 全量数据表
//    <out_dir>/comparison.json       — 代表性配置的 5 方法对比 viz JSON（3D 体素/iAABB 叠加）
//    <out_dir>/method_iaabbs.json    — 各方法 per-link iAABB 盒子数组（Python 渲染用）
//
//  CSV 列：
//    envelope,trial,width_frac,n_active,volume,
//    ep_time_ms,env_time_ms,total_ms
//
//  Build:
//    cmake --build build --config Release --target exp_link_iaabb_bench
//  Run:
//    build\Release\exp_link_iaabb_bench.exe [n_trials] [seed] [out_dir]
//    (out_dir 默认: results/exp_link_iaabb_bench_<timestamp>/)
// ═══════════════════════════════════════════════════════════════════════════
#define _USE_MATH_DEFINES
#include <cmath>

#include "sbf/robot/robot.h"
#include "sbf/envelope/endpoint_source.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/envelope/crit_sample.h"
#include "sbf/core/types.h"
#include "sbf/robot/fk.h"
#include "sbf/viz/viz_exporter.h"

#include <nlohmann/json.hpp>
#include <fstream>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <random>
#include <string>
#include <vector>

using namespace sbf;
using namespace sbf::envelope;
namespace fs = std::filesystem;

// ─── IIWA14 factory ──────────────────────────────────────────────────────────

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

// ─── Random box generator (deterministic with seed) ──────────────────────────

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

// ─── Build envelope configs ───────────────────────────────────────────────────

struct EnvEntry {
    const char* name;
    EnvelopeTypeConfig config;
};

static std::vector<EnvEntry> make_env_configs()
{
    float wb[] = {-2.f, -2.f, -0.5f, 2.f, 2.f, 2.5f};

    auto make_link = [&](int n_sub) {
        EnvelopeTypeConfig c;
        c.type  = EnvelopeType::LinkIAABB;
        c.n_sub = n_sub;
        std::memcpy(c.world_bounds, wb, sizeof(wb));
        return c;
    };

    EnvelopeTypeConfig cfg_grid = EnvelopeTypeConfig::link_iaabb_grid();
    std::memcpy(cfg_grid.world_bounds, wb, sizeof(wb));

    EnvelopeTypeConfig cfg_hull = EnvelopeTypeConfig::hull16_grid();  // δ=0.01, n_sub=1

    return {
        {"LinkIAABB_s1",   make_link(1)},
        {"LinkIAABB_s4",   make_link(4)},
        {"LinkIAABB_s16",  make_link(16)},
        {"LinkIAABB_Grid", cfg_grid},
        {"Hull16_Grid",    cfg_hull},
    };
}

// ─── Export per-method link iAABBs for the representative box ───────────────
//
// Runs all 5 Stage-2 methods on a single C-space box using CritSample endpoints
// and serialises each method's link_iaabbs to JSON, so the Python viz can render
// per-link box geometry per method.
//
// Hull16_Grid has no link_iaabbs; its voxel centres are exported instead.
//
// JSON schema:
//   { robot_name, n_active, n_joints, active_link_map[],
//     box_intervals[[lo,hi],...], robot_arm: { link_positions[[x,y,z],...] },
//     methods: {
//       "<name>": { type, n_sub?, n_active,
//                   link_iaabbs: [{link,seg,lo[3],hi[3]},...] }  -- for aabb
//         OR       { type:"voxel", delta, n_occupied,
//                   centres:[[x,y,z],...] }                      -- for Hull16
//     }
//   }

static void export_method_iaabbs_json(const Robot& robot,
                                      const std::vector<Interval>& box_intervals,
                                      const std::vector<EnvEntry>& envs,
                                      const EndpointSourceConfig& src_cfg,
                                      const std::string& filepath)
{
    using json = nlohmann::json;

    // Stage 1: compute endpoint iAABBs
    auto ep_result = compute_endpoint_iaabb(src_cfg, robot, box_intervals);
    const int n_act = robot.n_active_links();
    const int* alm  = robot.active_link_map();

    json j;
    j["robot_name"] = robot.name();
    j["n_joints"]   = robot.n_joints();
    j["n_active"]   = n_act;

    json alm_j = json::array();
    for (int i = 0; i < n_act; ++i) alm_j.push_back(alm[i]);
    j["active_link_map"] = alm_j;

    json radii_j = json::array();
    for (const auto& r : robot.link_radii()) radii_j.push_back(r);
    j["link_radii"] = radii_j;

    json bivs = json::array();
    for (const auto& iv : box_intervals) bivs.push_back({iv.lo, iv.hi});
    j["box_intervals"] = bivs;

    // Robot arm FK at box centre
    {
        Eigen::VectorXd q_c(robot.n_joints());
        for (int d = 0; d < robot.n_joints(); ++d)
            q_c[d] = box_intervals[d].center();
        auto pos = fk_link_positions(robot, q_c);
        json pa = json::array();
        for (const auto& p : pos) pa.push_back({p.x(), p.y(), p.z()});
        j["robot_arm"] = {{"link_positions", pa}};
    }

    json methods = json::object();

    for (const auto& e : envs) {
        EnvelopeResult res = compute_link_envelope(e.config, robot, ep_result);

        if (e.config.type == EnvelopeType::Hull16_Grid) {
            // Voxel: export centres from hull_grid
            double delta = res.hull_grid.delta();
            json centres = json::array();
            for (const auto& [coord, brick] : res.hull_grid.bricks()) {
                for (int z = 0; z < 8; ++z) {
                    uint64_t w = brick.words[z];
                    for (int bit = 0; bit < 64; ++bit) {
                        if (!(w & (1ULL << bit))) continue;
                        int lx = bit % 8, ly = bit / 8;
                        double cx = (coord.bx*8 + lx + 0.5) * delta;
                        double cy = (coord.by*8 + ly + 0.5) * delta;
                        double cz = (coord.bz*8 + z  + 0.5) * delta;
                        centres.push_back({cx, cy, cz});
                    }
                }
            }
            methods[e.name] = {
                {"type",       "voxel"},
                {"label",      e.name},
                {"delta",      delta},
                {"n_occupied", res.hull_grid.count_occupied()},
                {"centres",    centres},
            };
        } else {
            // AABB: read link_iaabbs flat array [n_act * n_sub * 6]
            int n_sub_actual = (n_act > 0)
                ? static_cast<int>(res.link_iaabbs.size()) / (n_act * 6)
                : 1;

            json aabb_list = json::array();
            for (int a = 0; a < n_act; ++a) {
                for (int s = 0; s < n_sub_actual; ++s) {
                    const float* ab =
                        res.link_iaabbs.data() + (a * n_sub_actual + s) * 6;
                    json ab_j;
                    ab_j["link"] = alm[a];
                    ab_j["seg"]  = s;
                    ab_j["lo"]   = {ab[0], ab[1], ab[2]};
                    ab_j["hi"]   = {ab[3], ab[4], ab[5]};
                    aabb_list.push_back(ab_j);
                }
            }
            methods[e.name] = {
                {"type",     "aabb"},
                {"label",    e.name},
                {"n_sub",    n_sub_actual},
                {"n_active", n_act},
                {"n_aabbs",  static_cast<int>(aabb_list.size())},
                {"aabbs",    aabb_list},
            };
        }
    }

    j["methods"] = methods;

    std::ofstream ofs(filepath);
    if (ofs.is_open())
        ofs << j.dump(2);
    else
        std::fprintf(stderr, "[exp] ERROR: cannot write %s\n", filepath.c_str());
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    const int    n_trials = (argc > 1) ? std::atoi(argv[1]) : 100;
    const int    seed     = (argc > 2) ? std::atoi(argv[2]) : 42;
    const std::string out_dir = (argc > 3)
        ? std::string(argv[3])
        : ("results/exp_link_iaabb_bench_" + make_timestamp());

    // Create output directory
    fs::create_directories(out_dir);

    Robot robot   = make_iiwa14();
    const int n_act = robot.n_active_links();

    // CritSample endpoint source (Stage 1) — fixed
    EndpointSourceConfig src_cfg;
    src_cfg.method = EndpointSource::CritSample;

    // Stage 2 variants
    auto envs = make_env_configs();
    const int n_envs = static_cast<int>(envs.size());

    // Width distribution: narrow / medium / wide (identical to v3v4 bench)
    const double widths_narrow[] = {0.05, 0.10, 0.15};
    const double widths_medium[] = {0.20, 0.30, 0.40};
    const double widths_wide[]   = {0.50, 0.70, 1.00};

    constexpr int N_REPEATS = 5;

    // Open CSV
    std::string csv_path = out_dir + "/results.csv";
    std::FILE* csv = std::fopen(csv_path.c_str(), "w");
    if (!csv) {
        std::fprintf(stderr, "[exp] ERROR: cannot open %s\n", csv_path.c_str());
        return 1;
    }
    std::fprintf(csv,
        "envelope,trial,width_frac,n_active,volume,"
        "ep_time_ms,env_time_ms,total_ms\n");

    std::fprintf(stderr, "[exp] n_trials=%d  seed=%d  out_dir=%s\n",
                 n_trials, seed, out_dir.c_str());

    // Generate a dedicated narrow viz box (frac=0.08, fixed seed=7)
    // so that comparison.json and method_iaabbs.json always show a tight interval
    // where all 5 methods produce meaningfully different geometries.
    std::mt19937 viz_rng(7);
    BoxGenerator viz_gen{robot, viz_rng};
    std::vector<Interval> viz_box = viz_gen.generate(0.08);

    // Separate rng for the benchmark trial loop
    std::mt19937 rng(seed);
    BoxGenerator gen{robot, rng};

    for (int trial = 0; trial < n_trials; ++trial)
    {
        // Assign width fraction by category
        double frac;
        if (trial < n_trials * 4 / 10)
            frac = widths_narrow[trial % 3];
        else if (trial < n_trials * 8 / 10)
            frac = widths_medium[(trial - n_trials * 4 / 10) % 3];
        else
            frac = widths_wide[(trial - n_trials * 8 / 10) % 3];

        auto intervals = gen.generate(frac);

        // ── Stage 1: CritSample endpoint iAABBs ─────────────────────────
        // Warmup
        { auto _r = compute_endpoint_iaabb(src_cfg, robot, intervals); (void)_r; }

        double ep_times[N_REPEATS];
        EndpointIAABBResult ep_result;
        for (int r = 0; r < N_REPEATS; ++r) {
            auto t0 = std::chrono::high_resolution_clock::now();
            ep_result = compute_endpoint_iaabb(src_cfg, robot, intervals);
            auto t1 = std::chrono::high_resolution_clock::now();
            ep_times[r] = std::chrono::duration<double, std::milli>(t1 - t0).count();
        }
        std::sort(ep_times, ep_times + N_REPEATS);
        double median_ep_ms = ep_times[N_REPEATS / 2];

        // ── Stage 2: each link iAABB method ─────────────────────────────
        for (int ei = 0; ei < n_envs; ++ei) {
            // Warmup
            { auto _r = compute_link_envelope(envs[ei].config, robot, ep_result); (void)_r; }

            double env_times[N_REPEATS];
            EnvelopeResult env_result;
            for (int r = 0; r < N_REPEATS; ++r) {
                auto t0 = std::chrono::high_resolution_clock::now();
                env_result = compute_link_envelope(envs[ei].config, robot, ep_result);
                auto t1 = std::chrono::high_resolution_clock::now();
                env_times[r] = std::chrono::duration<double, std::milli>(t1 - t0).count();
            }
            std::sort(env_times, env_times + N_REPEATS);
            double median_env_ms = env_times[N_REPEATS / 2];

            std::fprintf(csv,
                "%s,%d,%.3f,%d,%.10e,%.6f,%.6f,%.6f\n",
                envs[ei].name, trial, frac, n_act,
                env_result.volume,
                median_ep_ms, median_env_ms,
                median_ep_ms + median_env_ms);
        }

        if ((trial + 1) % 20 == 0)
            std::fprintf(stderr, "[exp] %d/%d trials done.\n", trial + 1, n_trials);
    }

    std::fclose(csv);
    std::fprintf(stderr, "[exp] CSV written: %s\n", csv_path.c_str());

    // ── Viz exports for representative box ──────────────────────────────────
    if (!viz_box.empty()) {
        // 1. 5-method 3D comparison JSON (voxels + iAABB overlay)
        std::string cmp_path = out_dir + "/comparison.json";
        sbf::viz::export_envelope_comparison_json(robot, viz_box,
                                                  /*n_sub=*/16,
                                                  /*delta=*/0.04,   // coarser voxels for viz (~64x fewer)
                                                  cmp_path);
        std::fprintf(stderr, "[exp] Comparison JSON:    %s\n", cmp_path.c_str());

        // 2. Per-method link iAABB geometry JSON (for Python box viz)
        std::string iaabb_path = out_dir + "/method_iaabbs.json";
        export_method_iaabbs_json(robot, viz_box, envs, src_cfg, iaabb_path);
        std::fprintf(stderr, "[exp] Method iAABBs JSON: %s\n", iaabb_path.c_str());
    }

    std::fprintf(stderr, "[exp] Done. Results in: %s\n", out_dir.c_str());
    return 0;
}
