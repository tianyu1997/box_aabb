// ═══════════════════════════════════════════════════════════════════════════
//  exp_ep_iaabb_bench — Endpoint iAABB Stage 1 方法耗时 & 体积对比实验
//
//  对以下 4 种 Stage 1 端点源方法进行系统性对比（固定 Stage 2 = extract_link_iaabbs）：
//    1. IFK          — Interval FK，最快但最保守 (SAFE)
//    2. CritSample   — 边界 kπ/2 枚举，更紧密但可能遗漏极值 (UNSAFE)
//    3. Analytical   — 解析 P0–P3 梯度零点枚举 + AA 剪枝，纯在线 (SAFE)
//    4. GCPC         — 解析边界 P0–P2.5 + GCPC 缓存内部 (SAFE，推荐)
//
//  Stage 2 固定为 extract_link_iaabbs()（derive_aabb，n_sub=1），确保公平对比。
//  Volume = Σ per-link body AABB volume（endpoint_iaabbs → derive_aabb + radius）。
//
//  GCPC 缓存：实验开始时在线构建（kπ/2 枚举 + enrich_with_interior_search）；
//            也可通过 --gcpc-cache <file> 加载已有缓存文件。
//
//  测量指标：
//    - Stage 1 耗时 (ms, 中位数，5 次重复)
//    - 总 per-link 体积 (m³)  [sum of link body AABB volumes]
//    - FK 评估次数 n_eval      [来自 EndpointIAABBResult.n_evaluations]
//
//  CSV 列：
//    source,trial,width_frac,n_active,volume,time_ms,n_eval
//
//  Build:
//    cmake --build build --config Release --target exp_ep_iaabb_bench
//  Run:
//    build\Release\exp_ep_iaabb_bench.exe [n_trials] [seed] [out_dir] [gcpc_cache]
//    (out_dir 默认: results/exp_ep_iaabb_bench_<timestamp>/)
// ═══════════════════════════════════════════════════════════════════════════
#define _USE_MATH_DEFINES
#include <cmath>

#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"
#include "sbf/envelope/endpoint_source.h"
#include "sbf/envelope/crit_sample.h"
#include "sbf/envelope/gcpc.h"
#include "sbf/core/types.h"
#include "sbf/viz/viz_exporter.h"

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

// ─── Volume from EndpointIAABBResult via extract_link_iaabbs ─────────────────

static double compute_volume(const EndpointIAABBResult& result,
                             const Robot& robot) {
    const int n_act = robot.n_active_links();
    std::vector<float> link_aabb(n_act * 6);
    extract_link_iaabbs(result, robot, link_aabb.data());

    double vol = 0.0;
    for (int i = 0; i < n_act; ++i) {
        const float* a = link_aabb.data() + i * 6;
        double dx = std::max(0.0, (double)(a[3] - a[0]));
        double dy = std::max(0.0, (double)(a[4] - a[1]));
        double dz = std::max(0.0, (double)(a[5] - a[2]));
        vol += dx * dy * dz;
    }
    return vol;
}

// ─── GCPC cache builder (kπ/2 enum + interior enrichment) ────────────────────

static GcpcCache build_gcpc_cache(const Robot& robot) {
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* link_map = robot.active_link_map();
    auto limits = robot.joint_limits();

    // kπ/2 values per joint (including endpoints and midpoint)
    std::vector<std::vector<double>> per_joint(n);
    for (int j = 0; j < n; ++j) {
        double lo = limits.limits[j].lo, hi = limits.limits[j].hi;
        per_joint[j].push_back(lo);
        per_joint[j].push_back(hi);
        per_joint[j].push_back(0.5 * (lo + hi));
        for (int k = -20; k <= 20; ++k) {
            double a = k * HALF_PI;
            if (a > lo + 1e-10 && a < hi - 1e-10)
                per_joint[j].push_back(a);
        }
        std::sort(per_joint[j].begin(), per_joint[j].end());
        per_joint[j].erase(
            std::unique(per_joint[j].begin(), per_joint[j].end()),
            per_joint[j].end());
    }

    // q₁ restricted to [0, π]
    std::vector<double> q1_half;
    for (double v : per_joint[1])
        if (v >= -1e-10 && v <= M_PI + 1e-10)
            q1_half.push_back(std::max(0.0, std::min((double)M_PI, v)));
    if (q1_half.empty()) q1_half.push_back(0.5);

    std::vector<GcpcPoint> cache_points;

    for (int ci = 0; ci < n_act; ++ci) {
        int link_id = link_map[ci];
        int nj      = std::min(link_id + 1, n);
        int n_eff   = nj - 1;
        if (n_eff <= 0) continue;

        std::vector<std::vector<double>> jv(n_eff);
        jv[0] = q1_half;
        for (int d = 1; d < n_eff; ++d)
            jv[d] = per_joint[d + 1];

        long long product = 1;
        for (int d = 0; d < n_eff; ++d) {
            product *= (long long)jv[d].size();
            if (product > 50000) { product = 50001; break; }
        }

        auto eval_config = [&](const std::vector<double>& cfg) {
            GcpcPoint pt{};
            pt.link_id = link_id;
            pt.n_eff   = n_eff;
            for (int i = 0; i < n_eff; ++i)
                pt.q_eff[i] = cfg[i];

            Eigen::VectorXd q_full(n);
            q_full.setZero();
            for (int i = 0; i < n_eff && i + 1 < n; ++i)
                q_full[i + 1] = pt.q_eff[i];

            auto pos = fk_link_positions(robot, q_full);
            if (link_id + 1 < (int)pos.size()) {
                const auto& p = pos[link_id + 1];
                pt.direction = 0; pt.A = p[0]; pt.B = p[1]; pt.C = p[2];
                pt.R = std::sqrt(pt.A * pt.A + pt.B * pt.B);
                cache_points.push_back(pt);
                GcpcPoint ptz = pt; ptz.direction = 1;
                cache_points.push_back(ptz);
            }
        };

        if (product > 50000) {
            std::mt19937 srng(12345 + ci);
            for (int s = 0; s < 5000; ++s) {
                std::vector<double> cfg(n_eff);
                for (int d = 0; d < n_eff; ++d)
                    cfg[d] = jv[d][std::uniform_int_distribution<int>(
                        0, (int)jv[d].size() - 1)(srng)];
                eval_config(cfg);
            }
        } else {
            std::vector<double> cfg(n_eff);
            std::function<void(int)> enumerate;
            enumerate = [&](int d) {
                if (d >= n_eff) { eval_config(cfg); return; }
                for (double v : jv[d]) { cfg[d] = v; enumerate(d + 1); }
            };
            enumerate(0);
        }
    }

    GcpcCache cache;
    cache.build(robot, cache_points);
    cache.enrich_with_interior_search(robot, 500, 5);
    return cache;
}

// ─── Main ─────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    const int n_trials = (argc > 1) ? std::atoi(argv[1]) : 100;
    const int seed     = (argc > 2) ? std::atoi(argv[2]) : 42;
    const std::string out_dir = (argc > 3)
        ? std::string(argv[3])
        : ("results/exp_ep_iaabb_bench_" + make_timestamp());
    const std::string gcpc_cache_file = (argc > 4) ? std::string(argv[4]) : "";

    fs::create_directories(out_dir);

    Robot robot   = make_iiwa14();
    const int n_act = robot.n_active_links();

    // ── Build or load GCPC cache ─────────────────────────────────────────
    GcpcCache gcpc_cache;
    if (!gcpc_cache_file.empty()) {
        std::fprintf(stderr, "[exp] Loading GCPC cache from: %s\n",
                     gcpc_cache_file.c_str());
        if (!gcpc_cache.load(gcpc_cache_file)) {
            std::fprintf(stderr, "[exp] WARN: failed to load cache, building inline.\n");
            gcpc_cache = build_gcpc_cache(robot);
        } else {
            std::fprintf(stderr, "[exp] GCPC cache loaded: %d links, %d points.\n",
                         gcpc_cache.n_links(), gcpc_cache.n_total_points());
        }
    } else {
        std::fprintf(stderr, "[exp] Building GCPC cache inline...\n");
        auto t0 = std::chrono::high_resolution_clock::now();
        gcpc_cache = build_gcpc_cache(robot);
        auto t1 = std::chrono::high_resolution_clock::now();
        double build_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        std::fprintf(stderr, "[exp] GCPC cache built in %.1f ms: %d links, %d points.\n",
                     build_ms, gcpc_cache.n_links(), gcpc_cache.n_total_points());
    }

    // ── Endpoint source configurations ───────────────────────────────────
    //
    //  IFK:        纯区间 FK，单次前向传播
    //  CritSample: 边界 kπ/2 枚举（UNSAFE，可能遗漏极值，最快采样）
    //  Analytical: P0–P3 解析梯度零点 + AA 剪枝（SAFE，纯在线）
    //  GCPC:       P0–P2.5 解析边界 + GCPC 缓存内部（SAFE，推荐）
    //
    EndpointSourceConfig cfg_ifk  = EndpointSourceConfig::ifk();

    EndpointSourceConfig cfg_crit = EndpointSourceConfig::crit_sampling();

    EndpointSourceConfig cfg_anal = EndpointSourceConfig::analytical();

    EndpointSourceConfig cfg_gcpc = EndpointSourceConfig::gcpc(&gcpc_cache);

    struct SourceEntry {
        const char*         name;
        EndpointSourceConfig config;
    };
    const SourceEntry sources[] = {
        {"IFK",        cfg_ifk},
        {"CritSample", cfg_crit},
        {"Analytical", cfg_anal},
        {"GCPC",       cfg_gcpc},
    };
    const int n_sources = static_cast<int>(sizeof(sources) / sizeof(sources[0]));

    // ── Width brackets (40 % narrow, 40 % medium, 20 % wide) ─────────────
    const double widths_narrow[] = {0.05, 0.10, 0.15};
    const double widths_medium[] = {0.20, 0.30, 0.40};
    const double widths_wide[]   = {0.50, 0.70, 1.00};

    constexpr int N_REPEATS = 5;

    // ── Open CSV ─────────────────────────────────────────────────────────
    std::string csv_path = out_dir + "/results.csv";
    std::FILE* csv = std::fopen(csv_path.c_str(), "w");
    if (!csv) {
        std::fprintf(stderr, "[exp] ERROR: cannot open %s\n", csv_path.c_str());
        return 1;
    }
    std::fprintf(csv,
        "source,trial,width_frac,n_active,volume,time_ms,n_eval\n");

    std::fprintf(stderr, "[exp] n_trials=%d  seed=%d  out_dir=%s\n",
                 n_trials, seed, out_dir.c_str());

    std::mt19937 rng(seed);
    BoxGenerator gen{robot, rng};

    // Representative box for 3D viz (middle trial)
    const int viz_trial = n_trials / 2;
    std::vector<Interval> viz_box;

    for (int trial = 0; trial < n_trials; ++trial)
    {
        double frac;
        if (trial < n_trials * 4 / 10)
            frac = widths_narrow[trial % 3];
        else if (trial < n_trials * 8 / 10)
            frac = widths_medium[(trial - n_trials * 4 / 10) % 3];
        else
            frac = widths_wide[(trial - n_trials * 8 / 10) % 3];

        auto intervals = gen.generate(frac);
        if (trial == viz_trial) viz_box = intervals;

        for (int si = 0; si < n_sources; ++si)
        {
            const auto& src = sources[si];

            // Warmup
            { auto r = compute_endpoint_iaabb(src.config, robot, intervals); (void)r; }

            double times[N_REPEATS];
            EndpointIAABBResult ep_result;
            for (int rep = 0; rep < N_REPEATS; ++rep) {
                auto t0 = std::chrono::high_resolution_clock::now();
                ep_result = compute_endpoint_iaabb(src.config, robot, intervals);
                auto t1 = std::chrono::high_resolution_clock::now();
                times[rep] = std::chrono::duration<double, std::milli>(t1 - t0).count();
            }
            std::sort(times, times + N_REPEATS);
            double median_ms = times[N_REPEATS / 2];
            double vol = compute_volume(ep_result, robot);

            std::fprintf(csv,
                "%s,%d,%.3f,%d,%.10e,%.6f,%d\n",
                src.name, trial, frac, n_act,
                vol, median_ms, ep_result.n_evaluations);
        }

        if ((trial + 1) % 20 == 0)
            std::fprintf(stderr, "[exp] %d/%d trials done.\n", trial + 1, n_trials);
    }

    std::fclose(csv);
    std::fprintf(stderr, "[exp] CSV written: %s\n", csv_path.c_str());

    // ── Viz comparison JSON for representative box ────────────────────────
    if (!viz_box.empty()) {
        // Re-compute each source on the viz box and collect endpoint_iaabbs
        std::vector<std::pair<std::string, std::vector<float>>> viz_sources;
        for (const auto& src : sources) {
            auto res = compute_endpoint_iaabb(src.config, robot, viz_box);
            viz_sources.push_back({std::string(src.name), res.endpoint_iaabbs});
        }
        std::string viz_path = out_dir + "/ep_iaabb_comparison.json";
        sbf::viz::export_ep_iaabb_comparison_json(robot, viz_box, viz_sources, viz_path);
        std::fprintf(stderr, "[exp] Endpoint iAABB comparison JSON: %s\n", viz_path.c_str());
    }

    std::fprintf(stderr, "[exp] Done. Results in: %s\n", out_dir.c_str());
    return 0;
}
