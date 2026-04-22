/**
 * exp10_theory_validation.cpp — 实验 10: 理论章节数值支撑
 *
 * 为论文新增的理论章节补齐数字证据，包含 4 个子任务:
 *   --task m1  Erosion duality verification (Thm erosion / certify)
 *              随机采样 N 个 box，对比 iaabb_certify vs 暴力采样胶囊-AABB 扫掠。
 *              统计 false-accept (应为 0) 与 false-reject（保守度）。
 *   --task m2  epiAABB width-stratified Pareto
 *              5 宽度箱 × 3 endpoint sources × 3 envelope types
 *              输出 (volume, time) Pareto 表。
 *   --task m3  LECT lazy materialisation / mmap residency
 *              measure file size / cold-load latency / RSS delta
 *              before and after a query burst.
 *   --task s4  Lifelong incremental cache growth
 *              连续 N 次 build（同机器人，不同障碍扰动），
 *              记录每次 cache 文件大小与 build 时间。
 *
 * 用法:
 *   ./exp10_theory_validation --task {m1|m2|m3|s4} [options]
 */

#include <sbf/planner/sbf_planner.h>
#include <sbf/scene/collision_checker.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/lect/lect.h>
#include <sbf/lect/lect_io.h>
#include "marcucci_scenes.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace sbf;

// ─── Small stats helper ─────────────────────────────────────────────────────
struct Stats { double mean, median, std_dev; };
static Stats compute_stats(std::vector<double> d) {
    Stats s{};
    if (d.empty()) return s;
    std::sort(d.begin(), d.end());
    int n = (int)d.size();
    s.median = d[n / 2];
    s.mean = std::accumulate(d.begin(), d.end(), 0.0) / n;
    double var = 0;
    for (double v : d) var += (v - s.mean) * (v - s.mean);
    s.std_dev = n > 1 ? std::sqrt(var / (n - 1)) : 0.0;
    return s;
}

// ─── RSS reader for Linux ───────────────────────────────────────────────────
static long rss_kb() {
    std::ifstream f("/proc/self/statm");
    long size, resident;
    if (!(f >> size >> resident)) return -1;
    return resident * (sysconf(_SC_PAGESIZE) / 1024);  // KB
}

static long file_size_bytes(const std::string& path) {
    struct stat st;
    if (::stat(path.c_str(), &st) != 0) return -1;
    return static_cast<long>(st.st_size);
}

// ─── Robot limits helper (match exp4) ──────────────────────────────────────
static void apply_planning_limits(Robot& robot) {
    auto& lim = const_cast<JointLimits&>(robot.joint_limits());
    const double planning_limits[7][2] = {
        {-1.865488,  1.865691}, {-0.100000,  1.086648},
        {-0.662656,  0.662338}, {-2.094400, -0.371673},
        {-0.619251,  0.619534}, {-1.095222,  1.257951},
        { 1.050209,  2.091190},
    };
    for (int j = 0; j < 7; ++j) {
        lim.limits[j].lo = planning_limits[j][0];
        lim.limits[j].hi = planning_limits[j][1];
    }
}

static std::vector<Interval> sample_box(const Robot& robot, double w,
                                        std::mt19937_64& rng) {
    std::uniform_real_distribution<double> u(0.0, 1.0);
    int D = robot.n_joints();
    const auto& jl = robot.joint_limits();
    std::vector<Interval> iv(D);
    for (int d = 0; d < D; ++d) {
        double range = jl.limits[d].hi - jl.limits[d].lo;
        double ww = std::min(w, range);
        double lo = jl.limits[d].lo + u(rng) * (range - ww);
        iv[d] = Interval{lo, lo + ww};
    }
    return iv;
}

// ═══════════════════════════════════════════════════════════════════════════
//  M1 — Erosion duality verification
// ═══════════════════════════════════════════════════════════════════════════
static int run_m1(Robot& robot, int n_boxes_per_bin, int n_samples_per_box,
                  uint64_t seed) {
    apply_planning_limits(robot);
    auto obs = make_combined_obstacles();
    CollisionChecker cc(robot, obs);

    const double bins[][2] = {
        {0.01, 0.05}, {0.05, 0.10}, {0.10, 0.20},
        {0.20, 0.35}, {0.35, 0.50},
    };
    int n_bin = sizeof(bins) / sizeof(bins[0]);

    std::cout << "\n" << std::string(88, '=') << "\n"
              << "  M1: Erosion duality verification\n"
              << "      boxes/bin=" << n_boxes_per_bin
              << "  samples/box=" << n_samples_per_box
              << "  seed=" << seed << "\n"
              << std::string(88, '=') << "\n\n";
    std::cout << std::left << std::setw(18) << "Width (rad)"
              << std::right << std::setw(10) << "N"
              << std::setw(12) << "IAABB_coll"
              << std::setw(12) << "Sample_coll"
              << std::setw(12) << "FalseAcc"
              << std::setw(12) << "FalseRej"
              << std::setw(12) << "FR_rate%" << "\n"
              << std::string(88, '-') << "\n";

    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> unif(0.0, 1.0);

    for (int b = 0; b < n_bin; ++b) {
        int iaabb_coll = 0, sample_coll = 0;
        int false_acc = 0, false_rej = 0;

        for (int i = 0; i < n_boxes_per_bin; ++i) {
            double w = bins[b][0] + unif(rng) * (bins[b][1] - bins[b][0]);
            auto iv = sample_box(robot, w, rng);

            // iaabb-based conservative check
            bool iaabb_hit = cc.check_box(iv);

            // Brute-force sample inside the box
            bool sample_hit = false;
            int D = robot.n_joints();
            Eigen::VectorXd q(D);
            for (int s = 0; s < n_samples_per_box; ++s) {
                for (int d = 0; d < D; ++d) {
                    q[d] = iv[d].lo + unif(rng) * (iv[d].hi - iv[d].lo);
                }
                if (cc.check_config(q)) { sample_hit = true; break; }
            }

            if (iaabb_hit) iaabb_coll++;
            if (sample_hit) sample_coll++;

            // False-accept: iaabb says free but a sample collides → unsound
            if (!iaabb_hit && sample_hit) false_acc++;
            // False-reject: iaabb claims collision but M samples all free
            if (iaabb_hit && !sample_hit) false_rej++;
        }

        double fr_pct = 100.0 * false_rej / n_boxes_per_bin;
        std::cout << std::left << std::setw(18)
                  << (std::to_string(bins[b][0]) + "-" + std::to_string(bins[b][1])).substr(0, 17)
                  << std::right << std::setw(10) << n_boxes_per_bin
                  << std::setw(12) << iaabb_coll
                  << std::setw(12) << sample_coll
                  << std::setw(12) << false_acc
                  << std::setw(12) << false_rej
                  << std::setw(12) << std::fixed << std::setprecision(2) << fr_pct
                  << "\n";
    }
    std::cout << std::string(88, '-') << "\n"
              << "  Soundness (Thm erosion): FalseAcc should be 0.\n"
              << "  Conservativeness (Remark tightness): FR_rate rises with box width.\n"
              << std::string(88, '=') << "\n";
    return 0;
}

// ═══════════════════════════════════════════════════════════════════════════
//  M2 — epiAABB width-stratified Pareto
// ═══════════════════════════════════════════════════════════════════════════
static double envelope_vol(const float* lia, int n_act, int n_sub) {
    double v = 0;
    int n = n_act * n_sub;
    for (int i = 0; i < n; ++i) {
        double dx = lia[i*6+3] - lia[i*6+0];
        double dy = lia[i*6+4] - lia[i*6+1];
        double dz = lia[i*6+5] - lia[i*6+2];
        v += std::max(0.0, dx) * std::max(0.0, dy) * std::max(0.0, dz);
    }
    return v;
}

static int run_m2(Robot& robot, int n_boxes_per_bin, uint64_t seed) {
    apply_planning_limits(robot);

    const double bins[][2] = {
        {0.01, 0.05}, {0.05, 0.10}, {0.10, 0.20},
        {0.20, 0.35}, {0.35, 0.50},
    };
    int n_bin = sizeof(bins) / sizeof(bins[0]);
    const char* bin_name[] = {"0.01-0.05", "0.05-0.10", "0.10-0.20",
                              "0.20-0.35", "0.35-0.50"};

    struct EP { const char* name; EndpointSource src; };
    EP eps[] = {
        {"IFK",        EndpointSource::IFK},
        {"CritSample", EndpointSource::CritSample},
        {"Analytical", EndpointSource::Analytical},
    };

    struct ENV { const char* name; EnvelopeType t; };
    ENV envs[] = {
        {"LinkIAABB",       EnvelopeType::LinkIAABB},
        {"LinkIAABB_Grid",  EnvelopeType::LinkIAABB_Grid},
        {"Hull16_Grid",     EnvelopeType::Hull16_Grid},
    };

    std::cout << "\n" << std::string(96, '=') << "\n"
              << "  M2: epiAABB width-stratified Pareto (boxes/bin=" << n_boxes_per_bin << ")\n"
              << std::string(96, '=') << "\n\n";
    std::cout << std::left << std::setw(12) << "Width"
              << std::setw(14) << "EP"
              << std::setw(18) << "Envelope"
              << std::right << std::setw(14) << "Vol_med(m³)"
              << std::setw(12) << "EP_us"
              << std::setw(12) << "Env_us"
              << std::setw(12) << "Tot_us"
              << "\n" << std::string(96, '-') << "\n";

    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> unif(0.0, 1.0);

    for (int b = 0; b < n_bin; ++b) {
        std::vector<std::vector<Interval>> boxes;
        for (int i = 0; i < n_boxes_per_bin; ++i) {
            double w = bins[b][0] + unif(rng) * (bins[b][1] - bins[b][0]);
            boxes.push_back(sample_box(robot, w, rng));
        }

        for (auto& ep : eps) {
            EndpointSourceConfig ep_cfg;
            ep_cfg.source = ep.src;
            if (ep.src == EndpointSource::CritSample) ep_cfg.n_samples_crit = 64;

            for (auto& env : envs) {
                EnvelopeTypeConfig env_cfg;
                env_cfg.type = env.t;
                env_cfg.n_subdivisions = 1;
                if (env.t != EnvelopeType::LinkIAABB)
                    env_cfg.grid_config.voxel_delta = 0.04f;

                std::vector<double> vols, ept, envt;
                for (auto& iv : boxes) {
                    auto t0 = std::chrono::steady_clock::now();
                    auto ep_res = compute_endpoint_iaabb(robot, iv, ep_cfg);
                    auto t1 = std::chrono::steady_clock::now();
                    auto env_res = compute_link_envelope(
                        ep_res.endpoint_iaabbs.data(),
                        ep_res.n_active_links,
                        robot.active_link_radii(), env_cfg);
                    auto t2 = std::chrono::steady_clock::now();
                    double e_us = std::chrono::duration<double, std::micro>(t1 - t0).count();
                    double v_us = std::chrono::duration<double, std::micro>(t2 - t1).count();
                    ept.push_back(e_us);
                    envt.push_back(v_us);
                    vols.push_back(envelope_vol(env_res.link_iaabbs.data(),
                                                env_res.n_active_links,
                                                env_res.n_subdivisions));
                }
                auto sv = compute_stats(vols);
                auto se = compute_stats(ept);
                auto sen = compute_stats(envt);
                std::cout << std::left << std::setw(12) << bin_name[b]
                          << std::setw(14) << ep.name
                          << std::setw(18) << env.name
                          << std::right << std::fixed
                          << std::setw(14) << std::setprecision(6) << sv.median
                          << std::setw(12) << std::setprecision(1) << se.median
                          << std::setw(12) << std::setprecision(1) << sen.median
                          << std::setw(12) << std::setprecision(1) << (se.median + sen.median)
                          << "\n";
            }
        }
        std::cout << "\n";
    }
    std::cout << std::string(96, '=') << "\n";
    return 0;
}

// ═══════════════════════════════════════════════════════════════════════════
//  M3 — LECT lazy materialisation (RSS + file size observation)
// ═══════════════════════════════════════════════════════════════════════════
static int run_m3(Robot& robot, int seed) {
    apply_planning_limits(robot);
    auto obs = make_combined_obstacles();
    auto queries = make_combined_queries();

    std::cout << "\n" << std::string(80, '=') << "\n"
              << "  M3: LECT lazy materialisation (seed=" << seed << ")\n"
              << std::string(80, '=') << "\n\n";

    // ── Phase 0: prime the cache with a warm build ──
    {
        SBFPlannerConfig cfg;
        cfg.z4_enabled = true;
        cfg.lect_no_cache = false;
        cfg.grower.rng_seed = seed;
        cfg.grower.timeout_ms = 5000.0;
        cfg.grower.max_boxes = 4000;
        cfg.grower.n_threads = 5;
        cfg.grower.connect_mode = true;
        cfg.grower.enable_promotion = true;
        cfg.grower.ffb_config.max_depth = 300;
        cfg.coarsen.target_boxes = 300;
        std::cout << "  [prime]     priming LECT cache ...\n";
        SBFPlanner prime(robot, cfg);
        prime.build_coverage(obs.data(), (int)obs.size(), 5000.0);
    }

    // Find the cache file path — use the first .bin* under ~/.sbf_cache
    std::string home = std::getenv("HOME") ? std::getenv("HOME") : ".";
    std::string cache_root = home + "/.sbf_cache";
    std::string cache_file;
    {
        FILE* p = popen(("find " + cache_root +
                         " -name '*.lect' -o -name '*.bin' 2>/dev/null "
                         "| head -1").c_str(), "r");
        char buf[1024]; if (p) {
            if (fgets(buf, sizeof(buf), p)) {
                std::string s(buf);
                while (!s.empty() && (s.back() == '\n' || s.back() == '\r')) s.pop_back();
                cache_file = s;
            }
            pclose(p);
        }
    }
    long fsz_kb = cache_file.empty() ? -1 : file_size_bytes(cache_file) / 1024;
    std::cout << "  [cache]     " << (cache_file.empty() ? "(not found)" : cache_file)
              << "   size=" << fsz_kb << " KB\n\n";

    // ── Phase 1: Direct LECT load — isolate lazy-mmap effect ──
    long rss0 = rss_kb();
    auto t_load0 = std::chrono::steady_clock::now();
    LECT lect;
    bool loaded = !cache_file.empty() && lect_load_binary(lect, robot, cache_file);
    auto t_load1 = std::chrono::steady_clock::now();
    long rss_after_load = rss_kb();
    double load_ms = std::chrono::duration<double, std::milli>(t_load1 - t_load0).count();

    std::cout << "  Phase 1 (direct LECT mmap load):\n";
    std::cout << "    loaded=" << (loaded ? "OK" : "FAIL")
              << "  n_nodes=" << lect.n_nodes()
              << "  load_time=" << std::fixed << std::setprecision(2) << load_ms << " ms\n";
    std::cout << "    RSS before: " << rss0 << " KB"
              << "   after: " << rss_after_load << " KB"
              << "   Δ=" << (rss_after_load - rss0) << " KB\n";
    if (fsz_kb > 0) {
        double ratio = 100.0 * double(rss_after_load - rss0) / double(fsz_kb);
        std::cout << "    Resident Δ / file size = " << std::setprecision(1) << ratio << "%"
                  << "  (lazy mmap: only pages touched so far)\n";
    }

    // ── Phase 2: touch every node → full materialisation ──
    long rss_pre_touch = rss_kb();
    auto t_touch0 = std::chrono::steady_clock::now();
    double accum = 0;
    for (int i = 0; i < lect.n_nodes(); ++i) {
        const float* ep = lect.get_endpoint_iaabbs(i, CH_SAFE);
        accum += ep[0] + ep[5];   // force read of first + last float of record
    }
    auto t_touch1 = std::chrono::steady_clock::now();
    long rss_post_touch = rss_kb();
    double touch_ms = std::chrono::duration<double, std::milli>(t_touch1 - t_touch0).count();

    std::cout << "\n  Phase 2 (touch all " << lect.n_nodes() << " nodes):\n";
    std::cout << "    touch_time=" << std::setprecision(2) << touch_ms << " ms"
              << "   RSS Δ=" << (rss_post_touch - rss_pre_touch) << " KB"
              << "   (checksum=" << accum << ")\n";

    std::cout << "\n  Interpretation:\n";
    std::cout << "    cold mmap load resident ratio ≈ "
              << std::setprecision(1)
              << (fsz_kb > 0 ? 100.0 * double(rss_after_load - rss0) / double(fsz_kb) : 0)
              << "%  (should be small; mmap is zero-copy)\n";
    std::cout << "    full touch resident ratio    ≈ "
              << (fsz_kb > 0 ? 100.0 * double(rss_post_touch - rss0) / double(fsz_kb) : 0)
              << "%  (rises as more pages fault in)\n";
    std::cout << std::string(80, '=') << "\n";
    return 0;
}

// ═══════════════════════════════════════════════════════════════════════════
//  S4 — Lifelong cache: sequential builds, same robot, perturbed scenes
// ═══════════════════════════════════════════════════════════════════════════
static std::vector<Obstacle> perturb_obs(const std::vector<Obstacle>& base,
                                         double delta, std::mt19937& rng) {
    std::uniform_real_distribution<float> ud(-delta, delta);
    auto out = base;
    for (auto& o : out) {
        float dx = ud(rng), dy = ud(rng), dz = ud(rng);
        o.bounds[0] += dx; o.bounds[3] += dx;
        o.bounds[1] += dy; o.bounds[4] += dy;
        o.bounds[2] += dz; o.bounds[5] += dz;
    }
    return out;
}

static int run_s4(Robot& robot, int n_runs, double delta, uint64_t seed) {
    apply_planning_limits(robot);
    auto obs_base = make_combined_obstacles();
    auto queries = make_combined_queries();

    // Build seed points from query endpoints so build_coverage has anchors
    // even when obstacle perturbation fragments the free space.
    std::vector<Eigen::VectorXd> seed_points;
    {
        auto eq = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
            return (a - b).squaredNorm() < 1e-8;
        };
        for (auto& qp : queries) {
            bool dup_s = false, dup_g = false;
            for (auto& s : seed_points) {
                if (eq(s, qp.start)) dup_s = true;
                if (eq(s, qp.goal)) dup_g = true;
            }
            if (!dup_s) seed_points.push_back(qp.start);
            if (!dup_g) seed_points.push_back(qp.goal);
        }
    }

    std::cout << "\n" << std::string(88, '=') << "\n"
              << "  S4: Lifelong incremental cache (runs=" << n_runs
              << ", perturb=±" << delta*100 << "cm, seed=" << seed << ")\n"
              << std::string(88, '=') << "\n\n";

    // Locate cache dir for file-size tracking
    std::string home = std::getenv("HOME") ? std::getenv("HOME") : ".";
    std::string cache_root = home + "/.sbf_cache";
    auto total_cache_size = [&]() -> long {
        long t = 0;
        FILE* p = popen(("du -sb " + cache_root + " 2>/dev/null | awk '{print $1}'").c_str(), "r");
        if (!p) return -1;
        char b[64]; if (fgets(b, sizeof(b), p)) t = atol(b);
        pclose(p);
        return t;
    };

    std::cout << std::left << std::setw(6) << "Run"
              << std::right << std::setw(12) << "Build_s"
              << std::setw(10) << "Boxes"
              << std::setw(14) << "CacheSize_KB"
              << std::setw(14) << "Query1st_ms"
              << std::setw(14) << "Query_all_s"
              << std::setw(8) << "SR"
              << "\n" << std::string(88, '-') << "\n";

    std::mt19937 rng(static_cast<uint32_t>(seed));
    for (int r = 0; r < n_runs; ++r) {
        auto pert = (r == 0) ? obs_base : perturb_obs(obs_base, delta, rng);

        SBFPlannerConfig cfg;
        cfg.z4_enabled = true;
        cfg.lect_no_cache = false;   // persistent cache ON
        cfg.grower.rng_seed = seed + r;
        cfg.grower.timeout_ms = 5000.0;
        cfg.grower.max_boxes = 4000;
        cfg.grower.n_threads = 5;
        cfg.grower.connect_mode = true;
        cfg.grower.enable_promotion = true;
        cfg.grower.ffb_config.max_depth = 300;
        cfg.coarsen.target_boxes = 300;

        SBFPlanner planner(robot, cfg);
        auto t0 = std::chrono::steady_clock::now();
        planner.build_coverage(pert.data(), (int)pert.size(), 5000.0, seed_points);
        auto t1 = std::chrono::steady_clock::now();
        double build_s = std::chrono::duration<double>(t1 - t0).count();

        // Query burst
        int ok = 0;
        double first_ms = -1;
        auto tq0 = std::chrono::steady_clock::now();
        for (size_t qi = 0; qi < queries.size(); ++qi) {
            auto& qp = queries[qi];
            auto ta = std::chrono::steady_clock::now();
            auto res = planner.query(qp.start, qp.goal, pert.data(), (int)pert.size());
            auto tb = std::chrono::steady_clock::now();
            if (qi == 0)
                first_ms = std::chrono::duration<double, std::milli>(tb - ta).count();
            if (res.success) ok++;
        }
        auto tq1 = std::chrono::steady_clock::now();
        double q_s = std::chrono::duration<double>(tq1 - tq0).count();

        long sz = total_cache_size();
        std::cout << std::left << std::setw(6) << r
                  << std::right << std::fixed << std::setprecision(3)
                  << std::setw(12) << build_s
                  << std::setw(10) << planner.n_boxes()
                  << std::setw(14) << (sz > 0 ? sz / 1024 : 0)
                  << std::setw(14) << std::setprecision(2) << first_ms
                  << std::setw(14) << std::setprecision(3) << q_s
                  << std::setw(8) << (std::to_string(ok) + "/" + std::to_string(queries.size()))
                  << "\n";
    }
    std::cout << std::string(88, '-') << "\n"
              << "  Note: CacheSize grows monotonically across runs (append-only);\n"
              << "        build_s should decrease after run 0 (warm cache).\n"
              << std::string(88, '=') << "\n";
    return 0;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Main
// ═══════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv) {
    std::string robot_path = std::string(SBF_DATA_DIR) + "/iiwa14.json";
    std::string task = "m1";
    int n_boxes = 500;
    int n_samples = 200;
    int n_runs = 5;
    uint64_t seed = 42;
    double delta = 0.02;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--task" && i+1 < argc) task = argv[++i];
        else if (a == "--n-boxes" && i+1 < argc) n_boxes = std::atoi(argv[++i]);
        else if (a == "--n-samples" && i+1 < argc) n_samples = std::atoi(argv[++i]);
        else if (a == "--runs" && i+1 < argc) n_runs = std::atoi(argv[++i]);
        else if (a == "--seed" && i+1 < argc) seed = std::strtoull(argv[++i], nullptr, 10);
        else if (a == "--delta" && i+1 < argc) delta = std::atof(argv[++i]);
        else if (a[0] != '-') robot_path = a;
    }

    Robot robot = Robot::from_json(robot_path);
    std::cout << "Robot: " << robot.name() << "  DOF=" << robot.n_joints()
              << "  active=" << robot.n_active_links() << "\n";

    if (task == "m1") return run_m1(robot, n_boxes, n_samples, seed);
    if (task == "m2") return run_m2(robot, n_boxes, seed);
    if (task == "m3") return run_m3(robot, (int)seed);
    if (task == "s4") return run_s4(robot, n_runs, delta, seed);

    std::cerr << "Unknown task: " << task
              << "\nValid: m1 | m2 | m3 | s4\n";
    return 1;
}
