// ═══════════════════════════════════════════════════════════════════════════
//  FK Cost Analysis — Why 10× FK reduction ≠ 10× speedup?
//
//  Measures per-call cost of different operation types:
//    (1) sampling FK: ws.compute_from(robot, q, j) for data extraction only
//    (2) eval FK:     eval_and_update_from — compute_from + update ALL links
//    (3) direct coeff: compute_suffix_pos + extract_1d_coefficients
//    (4) QR decomp:   3×3 colPivHouseholderQr
//    (5) suffix_pos:  compute_suffix_pos alone
//
//  Also breaks down Phase 1–3 attribution:
//    For each phase, how much time is sampling vs eval vs other?
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_math.h"
#include "sbf/robot/fk.h"
#include "sbf/envelope/analytical_solve.h"
#include "sbf/envelope/analytical_utils.h"
#include "sbf/envelope/analytical_coeff.h"

#include <Eigen/QR>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <vector>

using namespace sbf;
using namespace sbf::envelope;

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

int main() {
    Robot robot = make_iiwa14();
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();

    // Joint configuration at center
    Eigen::VectorXd q(n);
    for (int j = 0; j < n; ++j) {
        auto& lim = robot.joint_limits().limits[j];
        q[j] = 0.5 * (lim.lo + lim.hi);
    }

    const double width = 0.20;
    std::vector<Interval> ivs(n);
    for (int j = 0; j < n; ++j) {
        auto& lim = robot.joint_limits().limits[j];
        double c = 0.5 * (lim.lo + lim.hi);
        ivs[j] = Interval(std::max(lim.lo, c - 0.5 * width),
                           std::min(lim.hi, c + 0.5 * width));
    }

    // Prepare workspace
    FKWorkspace ws;
    ws.resize(n + 2);

    // Setup link_seg_ext for eval_and_update_from
    const int* map = robot.active_link_map();
    int n_sub = 1;
    float inv_n = 1.0f;
    std::vector<std::vector<LinkExtremes>> link_seg_ext(n_act,
        std::vector<LinkExtremes>(n_sub));
    // Initialize with a full FK
    ws.compute(robot, q);
    for (int ci = 0; ci < n_act; ++ci) {
        int V = map[ci];
        if (V + 1 >= ws.np) continue;
        for (int s = 0; s < n_sub; ++s) {
            link_seg_ext[ci][s].update(ws.pos(V), q);
            link_seg_ext[ci][s].update(ws.pos(V + 1), q);
        }
    }

    const int ITERS = 100000;
    auto now = []{ return std::chrono::high_resolution_clock::now(); };
    auto elapsed_ns = [](auto t0, auto t1) {
        return std::chrono::duration<double, std::nano>(t1 - t0).count();
    };

    std::fprintf(stderr, "═══ FK Cost Micro-Benchmark (iiwa14, %d iterations) ═══\n\n", ITERS);

    // ── (1) Sampling FK: ws.compute_from(robot, q, j) — extract position only ──
    {
        double total_ns = 0;
        volatile double sink = 0;
        for (int iter = 0; iter < ITERS; ++iter) {
            int j = iter % n;
            ws.set_identity();
            for (int k = 0; k < j; ++k) ws.compute_joint(robot, q, k);
            auto t0 = now();
            ws.compute_from(robot, q, j);
            auto t1 = now();
            total_ns += elapsed_ns(t0, t1);
            int V = map[std::min(n_act - 1, iter % n_act)];
            if (V + 1 < ws.np) sink += ws.pos(V + 1)[0];
        }
        std::fprintf(stderr, "  (1) sampling FK  (compute_from)          : %7.0f ns/call\n",
                     total_ns / ITERS);
        (void)sink;
    }

    // ── (2) eval_and_update_from — compute_from + update ALL n_act links ──
    {
        double total_ns = 0;
        for (int iter = 0; iter < ITERS; ++iter) {
            int j = iter % n;
            ws.set_identity();
            for (int k = 0; k < j; ++k) ws.compute_joint(robot, q, k);
            auto t0 = now();
            eval_and_update_from(robot, q, j, n_sub, inv_n,
                                 map, n_act, link_seg_ext, ws);
            auto t1 = now();
            total_ns += elapsed_ns(t0, t1);
        }
        std::fprintf(stderr, "  (2) eval_and_update_from (FK+update)     : %7.0f ns/call\n",
                     total_ns / ITERS);
    }

    // ── (3) compute_suffix_pos ──
    {
        double total_ns = 0;
        volatile double sink = 0;
        for (int iter = 0; iter < ITERS; ++iter) {
            int j = iter % n;
            int V = map[std::min(n_act - 1, j % n_act)];
            auto t0 = now();
            auto sp = compute_suffix_pos(robot, q, j, V + 1);
            auto t1 = now();
            total_ns += elapsed_ns(t0, t1);
            sink += sp[0];
        }
        std::fprintf(stderr, "  (3) compute_suffix_pos                   : %7.0f ns/call\n",
                     total_ns / ITERS);
        (void)sink;
    }

    // ── (4) extract_1d_coefficients ──
    {
        double total_ns = 0;
        volatile double sink = 0;
        ws.set_identity();
        for (int k = 0; k < n; ++k) ws.compute_joint(robot, q, k);
        auto sp = compute_suffix_pos(robot, q, 3, 5);
        for (int iter = 0; iter < ITERS; ++iter) {
            int j = iter % n;
            const auto& dh_j = robot.dh_params()[j];
            auto t0 = now();
            auto c = extract_1d_coefficients(ws.tf[j], sp, dh_j);
            auto t1 = now();
            total_ns += elapsed_ns(t0, t1);
            sink += c.alpha[0];
        }
        std::fprintf(stderr, "  (4) extract_1d_coefficients              : %7.0f ns/call\n",
                     total_ns / ITERS);
        (void)sink;
    }

    // ── (5) 3×3 QR decomposition ──
    {
        double lo_j = ivs[3].lo, hi_j = ivs[3].hi;
        double mid_j = 0.5 * (lo_j + hi_j);
        double qvals[3] = {lo_j, mid_j, hi_j};
        Eigen::Matrix3d A;
        for (int si = 0; si < 3; ++si) {
            A(si, 0) = std::cos(qvals[si]);
            A(si, 1) = std::sin(qvals[si]);
            A(si, 2) = 1.0;
        }
        double total_ns = 0;
        volatile double sink = 0;
        for (int iter = 0; iter < ITERS; ++iter) {
            auto t0 = now();
            auto qr = A.colPivHouseholderQr();
            auto t1 = now();
            total_ns += elapsed_ns(t0, t1);
            Eigen::Vector3d b(0.1, 0.2, 0.3);
            sink += qr.solve(b)[0];
        }
        std::fprintf(stderr, "  (5) 3x3 QR decomp + solve                : %7.0f ns/call\n",
                     total_ns / ITERS);
        (void)sink;
    }

    // ── (6) prefix computation (set_identity + compute_joint × j) ──
    {
        double total_ns = 0;
        for (int iter = 0; iter < ITERS; ++iter) {
            int j = iter % n;
            auto t0 = now();
            ws.set_identity();
            for (int k = 0; k < j; ++k) ws.compute_joint(robot, q, k);
            auto t1 = now();
            total_ns += elapsed_ns(t0, t1);
        }
        std::fprintf(stderr, "  (6) prefix FK (identity + joints 0..j-1) : %7.0f ns/call\n",
                     total_ns / ITERS);
    }

    // ═══════════════════════════════════════════════════════════════
    //  Cost attribution: eliminated vs remaining per phase
    // ═══════════════════════════════════════════════════════════════
    std::fprintf(stderr, "\n═══ Per-Phase Cost Attribution (w=%.2f) ═══\n\n", width);

    auto cfg_qr = AnalyticalCriticalConfig::all_enabled();
    cfg_qr.enable_p1_direct_coeff = false;
    cfg_qr.enable_p2_direct_coeff = false;
    cfg_qr.enable_p25_direct_coeff = false;
    cfg_qr.enable_p3_direct_coeff = false;

    auto cfg_direct = AnalyticalCriticalConfig::all_enabled();

    // Run QR mode and time each phase independently by disabling others
    auto time_phase = [&](const char* label,
                          std::function<AnalyticalCriticalConfig()> make_full,
                          std::function<AnalyticalCriticalConfig()> make_without) {
        const int n_act2 = robot.n_active_links();
        std::vector<float> aabb(n_act2 * n_sub * 6);
        AnalyticalCriticalStats st;

        // warmup
        auto cfg_f = make_full();
        for (int r = 0; r < 3; ++r)
            derive_aabb_critical_analytical(robot, ivs, n_sub, cfg_f, aabb.data(), nullptr);

        // timed full
        double t_full = 0;
        for (int r = 0; r < 20; ++r) {
            auto t0 = now();
            derive_aabb_critical_analytical(robot, ivs, n_sub, cfg_f, aabb.data(), &st);
            auto t1 = now();
            t_full += elapsed_ns(t0, t1);
        }
        t_full /= 20;

        auto cfg_w = make_without();
        for (int r = 0; r < 3; ++r)
            derive_aabb_critical_analytical(robot, ivs, n_sub, cfg_w, aabb.data(), nullptr);
        double t_without = 0;
        for (int r = 0; r < 20; ++r) {
            auto t0 = now();
            derive_aabb_critical_analytical(robot, ivs, n_sub, cfg_w, aabb.data(), nullptr);
            auto t1 = now();
            t_without += elapsed_ns(t0, t1);
        }
        t_without /= 20;

        double phase_ms = (t_full - t_without) / 1e6;
        std::fprintf(stderr, "  %s: %.3f ms\n", label, phase_ms);
    };

    // QR mode — phase timing by subtraction
    std::fprintf(stderr, "── QR Mode (v3-equivalent) ──\n");
    time_phase("P1 (QR)  ", [&]{ return cfg_qr; },
               [&]{ auto c = cfg_qr; c.enable_edge_solve = false; return c; });
    time_phase("P2 (QR)  ", [&]{ return cfg_qr; },
               [&]{ auto c = cfg_qr; c.enable_face_solve = false; return c; });
    time_phase("P2.5 (QR)", [&]{ return cfg_qr; },
               [&]{ auto c = cfg_qr; c.enable_pair_1d = false; c.enable_pair_2d = false; return c; });
    time_phase("P3 (QR)  ", [&]{ return cfg_qr; },
               [&]{ auto c = cfg_qr; c.enable_interior_solve = false; return c; });

    std::fprintf(stderr, "\n── Direct Mode (Phase B+C+D+F) ──\n");
    time_phase("P1 (dir) ", [&]{ return cfg_direct; },
               [&]{ auto c = cfg_direct; c.enable_edge_solve = false; return c; });
    time_phase("P2 (dir) ", [&]{ return cfg_direct; },
               [&]{ auto c = cfg_direct; c.enable_face_solve = false; return c; });
    time_phase("P2.5(dir)", [&]{ return cfg_direct; },
               [&]{ auto c = cfg_direct; c.enable_pair_1d = false; c.enable_pair_2d = false; return c; });
    time_phase("P2.5a(d) ", [&]{ return cfg_direct; },
               [&]{ auto c = cfg_direct; c.enable_pair_1d = false; return c; });
    time_phase("P2.5b(d) ", [&]{ return cfg_direct; },
               [&]{ auto c = cfg_direct; c.enable_pair_2d = false; return c; });
    time_phase("P3 (dir) ", [&]{ return cfg_direct; },
               [&]{ auto c = cfg_direct; c.enable_interior_solve = false; return c; });

    // Total
    std::fprintf(stderr, "\n── Total Pipeline ──\n");
    {
        const int n_act2 = robot.n_active_links();
        std::vector<float> aabb(n_act2 * n_sub * 6);
        AnalyticalCriticalStats st_qr, st_dir;

        for (int r = 0; r < 5; ++r) {
            derive_aabb_critical_analytical(robot, ivs, n_sub, cfg_qr, aabb.data(), nullptr);
            derive_aabb_critical_analytical(robot, ivs, n_sub, cfg_direct, aabb.data(), nullptr);
        }

        double t_qr = 0, t_dir = 0;
        for (int r = 0; r < 30; ++r) {
            auto t0 = now();
            derive_aabb_critical_analytical(robot, ivs, n_sub, cfg_qr, aabb.data(), &st_qr);
            auto t1 = now();
            t_qr += elapsed_ns(t0, t1);

            t0 = now();
            derive_aabb_critical_analytical(robot, ivs, n_sub, cfg_direct, aabb.data(), &st_dir);
            t1 = now();
            t_dir += elapsed_ns(t0, t1);
        }
        t_qr /= 30; t_dir /= 30;

        std::fprintf(stderr, "  QR total     : %.3f ms  (FK=%d)\n",
                     t_qr / 1e6, st_qr.n_phase1_fk_calls + st_qr.n_phase2_fk_calls +
                     st_qr.n_phase25_fk_calls + st_qr.n_phase3_fk_calls);
        std::fprintf(stderr, "  Direct total : %.3f ms  (FK=%d)\n",
                     t_dir / 1e6, st_dir.n_phase1_fk_calls + st_dir.n_phase2_fk_calls +
                     st_dir.n_phase25_fk_calls + st_dir.n_phase3_fk_calls);
        std::fprintf(stderr, "  FK reduction : %.1f×  Time reduction: %.1f%%\n",
                     double(st_qr.n_phase1_fk_calls + st_qr.n_phase2_fk_calls +
                            st_qr.n_phase25_fk_calls + st_qr.n_phase3_fk_calls) /
                     double(st_dir.n_phase1_fk_calls + st_dir.n_phase2_fk_calls +
                            st_dir.n_phase25_fk_calls + st_dir.n_phase3_fk_calls),
                     100.0 * (1.0 - t_dir / t_qr));

        std::fprintf(stderr, "\n  FK breakdown:      P1       P2       P2.5     P3\n");
        std::fprintf(stderr, "    QR     FK:    %5d    %5d    %5d    %5d\n",
                     st_qr.n_phase1_fk_calls, st_qr.n_phase2_fk_calls,
                     st_qr.n_phase25_fk_calls, st_qr.n_phase3_fk_calls);
        std::fprintf(stderr, "    Direct FK:    %5d    %5d    %5d    %5d\n",
                     st_dir.n_phase1_fk_calls, st_dir.n_phase2_fk_calls,
                     st_dir.n_phase25_fk_calls, st_dir.n_phase3_fk_calls);
    }

    return 0;
}
