// exp_small_ep_viz.cpp — 固定小区间下 endpoint iAABB 单次可视化导出
//
// 使用固定的小 C-space 区间（每个关节以关节范围中心为中心，frac 控制宽度比例）
// 对 4 种 Stage-1 源生成 endpoint iAABB，仅导出 ep_iaabb_comparison.json。
//
// Build:
//   cmake --build build --config Release --target exp_small_ep_viz
// Run:
//   build\Release\exp_small_ep_viz.exe [frac=0.05] [out_dir=results/small_ep_viz] [seed=42]
// ─────────────────────────────────────────────────────────────────────────────

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
#include <filesystem>
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

// ─── Inline GCPC cache builder (same as exp_ep_iaabb_bench) ──────────────────
static GcpcCache build_gcpc_cache(const Robot& robot)
{
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* link_map = robot.active_link_map();
    auto limits = robot.joint_limits();

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
    double frac      = (argc > 1) ? std::atof(argv[1]) : 0.05;
    std::string out  = (argc > 2) ? std::string(argv[2]) : "results/small_ep_viz";
    int seed         = (argc > 3) ? std::atoi(argv[3]) : 42;

    fs::create_directories(out);

    Robot robot = make_iiwa14();
    const int n = robot.n_joints();

    // ── Build GCPC cache ──────────────────────────────────────────────────
    std::fprintf(stderr, "[small_ep_viz] Building GCPC cache...\n");
    auto t0c = std::chrono::high_resolution_clock::now();
    GcpcCache gcpc_cache = build_gcpc_cache(robot);
    auto t1c = std::chrono::high_resolution_clock::now();
    std::fprintf(stderr, "[small_ep_viz] Cache built in %.1f ms: %d links, %d pts.\n",
        std::chrono::duration<double,std::milli>(t1c - t0c).count(),
        gcpc_cache.n_links(), gcpc_cache.n_total_points());

    // ── Generate small interval at random center ──────────────────────────
    std::mt19937 rng(seed);
    const auto& lims = robot.joint_limits().limits;
    std::vector<Interval> ivs(n);
    for (int j = 0; j < n; ++j) {
        double lo = lims[j].lo, hi = lims[j].hi, range = hi - lo;
        std::uniform_real_distribution<double> d(lo, hi);
        double c  = d(rng);
        double hw = range * frac * 0.5;
        ivs[j].lo = std::max(lo, c - hw);
        ivs[j].hi = std::min(hi, c + hw);
    }

    std::fprintf(stderr, "[small_ep_viz] frac=%.3f  seed=%d\n", frac, seed);
    for (int j = 0; j < n; ++j) {
        std::fprintf(stderr, "  j%d: [%.4f, %.4f]  width=%.4f rad\n",
                     j, ivs[j].lo, ivs[j].hi, ivs[j].hi - ivs[j].lo);
    }

    // ── Source configs ─────────────────────────────────────────────────────
    EndpointSourceConfig cfg_ifk  = EndpointSourceConfig::ifk();
    EndpointSourceConfig cfg_crit = EndpointSourceConfig::crit_sampling();
    EndpointSourceConfig cfg_anal = EndpointSourceConfig::analytical();
    EndpointSourceConfig cfg_gcpc = EndpointSourceConfig::gcpc(&gcpc_cache);

    struct Src { const char* name; EndpointSourceConfig cfg; };
    const Src sources[] = {
        {"IFK",        cfg_ifk},
        {"CritSample", cfg_crit},
        {"Analytical", cfg_anal},
        {"GCPC",       cfg_gcpc},
    };

    // ── Compute endpoint_iaabbs ────────────────────────────────────────────
    std::vector<std::pair<std::string, std::vector<float>>> viz_sources;
    for (const auto& s : sources) {
        auto res = compute_endpoint_iaabb(s.cfg, robot, ivs);
        std::vector<float> ep(
            res.endpoint_data(),
            res.endpoint_data() + res.endpoint_iaabb_len());
        viz_sources.push_back({std::string(s.name), std::move(ep)});
        std::fprintf(stderr, "[small_ep_viz] %s: %d endpoints, n_eval=%d\n",
                     s.name, res.endpoint_iaabb_len() / 6, res.n_evaluations);
    }

    // ── Export JSON ────────────────────────────────────────────────────────
    std::string json_out = out + "/ep_iaabb_comparison.json";
    sbf::viz::export_ep_iaabb_comparison_json(robot, ivs, viz_sources, json_out);
    std::fprintf(stderr, "[small_ep_viz] JSON: %s\n", json_out.c_str());
    return 0;
}
