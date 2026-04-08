// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Analytical Utils (shared tools for Analytical solver)
//  Module: sbf::envelope
//
//  Cross-phase utility functions and structs extracted from the v3 monolithic
//  envelope_derive_critical.cpp.  Used by analytical_solve.cpp, crit_sample,
//  and gcpc for:
//    - FK workspace (stack-allocated, incremental)
//    - Critical-angle enumeration
//    - LinkExtremes tracking
//    - Polynomial root-finding in intervals
//    - Bilinear trig polynomial construction
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/core/types.h"
#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"

#include <Eigen/Core>
#include <algorithm>
#include <cassert>
#include <cmath>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sbf {
namespace envelope {

// ─── Constants ──────────────────────────────────────────────────────────────
static const double kShifts[3] = {0.0, 2 * M_PI, -2 * M_PI};
static constexpr long long CRIT_MAX_COMBOS = 60'000;

// ─── FKWorkspace (stack-allocated, incremental) ─────────────────────────────
//
// Pre-allocated 4×4 transform buffer for fast FK evaluation during
// analytical critical-point enumeration.  Avoids heap allocation per call.
//
struct FKWorkspace {
    Eigen::Matrix4d tf[16];   // stack-allocated (max 7 joints + tool + base)
    int np = 0;

    void resize(int /*max_frames*/) { /* no-op: fixed-size stack array */ }

    void compute(const Robot& robot, const Eigen::VectorXd& q) {
        np = fk_transforms_inplace(robot, q, tf);
    }

    Eigen::Vector3d pos(int k) const {
        return tf[k].block<3, 1>(0, 3);
    }

    // ── Incremental FK helpers ──────────────────────────────────────
    void set_identity() { tf[0] = Eigen::Matrix4d::Identity(); }

    // Compute T[j+1] = T[j] * dh_transform(q[j])  — single matrix mult
    void compute_joint(const Robot& robot, const Eigen::VectorXd& q, int j) {
        const auto& dh = robot.dh_params()[j];
        double theta, d;
        if (dh.joint_type == 0) {
            theta = q[j] + dh.theta;
            d = dh.d;
        } else {
            theta = dh.theta;
            d = q[j] + dh.d;
        }
        tf[j + 1] = tf[j] * dh_transform(dh.alpha, dh.a, d, theta);
    }

    // Compute remaining joints from `from` to end, including tool
    void compute_from(const Robot& robot, const Eigen::VectorXd& q, int from) {
        int n = robot.n_joints();
        for (int j = from; j < n; ++j)
            compute_joint(robot, q, j);
        if (robot.has_tool()) {
            const auto& tool = *robot.tool_frame();
            tf[n + 1] = tf[n] * dh_transform(tool.alpha, tool.a, tool.d, tool.theta);
            np = n + 2;
        } else {
            np = n + 1;
        }
    }

    // Compute FK prefix: T[0]=I, T[1]..T[up_to_joint].
    // After this call, T[0..up_to_joint] are valid and can be extended by
    // compute_from(robot, q, up_to_joint).
    void compute_prefix(const Robot& robot, const Eigen::VectorXd& q, int up_to_joint) {
        tf[0] = Eigen::Matrix4d::Identity();
        for (int j = 0; j < up_to_joint; ++j)
            compute_joint(robot, q, j);
    }
};

// ─── Critical-angle enumeration helpers ─────────────────────────────────────

// Build candidate angles for one joint interval: {lo, hi} ∪ {kπ/2 ∈ (lo,hi)}
inline std::vector<double> crit_angles(double lo, double hi) {
    std::vector<double> v = { lo, hi };
    for (int k = -20; k <= 20; ++k) {
        double a = k * HALF_PI;
        if (a > lo && a < hi)
            v.push_back(a);
    }
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
    return v;
}

// Build Cartesian-product sets for joints [0..n_joints-1].
// If product exceeds CRIT_MAX_COMBOS, fall back to {lo, mid, hi}.
inline std::vector<std::vector<double>> build_csets(
    const std::vector<std::vector<double>>& per_joint,
    const std::vector<Interval>& intervals,
    int n_joints)
{
    std::vector<std::vector<double>> csets(n_joints);
    long long total = 1;
    for (int j = 0; j < n_joints; ++j) {
        csets[j] = per_joint[j];
        total *= static_cast<long long>(csets[j].size());
        if (total > CRIT_MAX_COMBOS) {
            for (int j2 = 0; j2 < n_joints; ++j2) {
                double lo = intervals[j2].lo, hi = intervals[j2].hi;
                csets[j2] = { lo, 0.5 * (lo + hi), hi };
            }
            break;
        }
    }
    return csets;
}

// Recursive Cartesian-product enumeration (templated callback for zero-overhead)
template<typename Fn>
inline void crit_enum(
    const std::vector<std::vector<double>>& csets,
    Eigen::VectorXd& q, int depth,
    const Fn& cb)
{
    if (depth == static_cast<int>(csets.size())) { cb(); return; }
    for (double v : csets[depth]) {
        q[depth] = v;
        crit_enum(csets, q, depth + 1, cb);
    }
}

// Incremental FK variant: maintains transform chain as we recurse,
// only 1 matrix mult per recursion step instead of full FK at leaf.
// Templated callback avoids std::function virtual-dispatch overhead.
template<typename Fn>
inline void crit_enum_fk(
    const Robot& robot,
    const std::vector<std::vector<double>>& csets,
    Eigen::VectorXd& q, int depth, int n_needed,
    FKWorkspace& ws,
    const Fn& cb)
{
    if (depth == n_needed) {
        ws.compute_from(robot, q, n_needed);
        cb();
        return;
    }
    for (double v : csets[depth]) {
        q[depth] = v;
        ws.compute_joint(robot, q, depth);
        crit_enum_fk(robot, csets, q, depth + 1, n_needed, ws, cb);
    }
}

// ─── LinkExtremes — per-link AABB extremes tracker ──────────────────────────
//
// Tracks 6-direction extremes (min/max × 3 axes) and the joint configuration
// that produced each extreme.
//
// S9: ConfigVec uses inline storage (max 16 joints) to avoid heap allocation.
using ConfigVec = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 16, 1>;

struct LinkExtremes {
    double vals[6];           // [x_min, x_max, y_min, y_max, z_min, z_max]
    ConfigVec configs[6];

    void init(int n_joints) {
        for (int i = 0; i < 6; ++i) {
            vals[i] = (i % 2 == 0) ? 1e30 : -1e30;  // min=+inf, max=-inf
            configs[i] = ConfigVec::Zero(n_joints);
        }
    }

    void update(const Eigen::Vector3d& pos, const Eigen::VectorXd& q) {
        for (int d = 0; d < 3; ++d) {
            double v = pos[d];
            if (v < vals[d * 2]) {       // d_min
                vals[d * 2] = v;
                configs[d * 2] = q;
            }
            if (v > vals[d * 2 + 1]) {   // d_max
                vals[d * 2 + 1] = v;
                configs[d * 2 + 1] = q;
            }
        }
    }
};

// ─── eval_and_update — FK + update link extremes ────────────────────────────

inline void eval_and_update(
    const Robot& robot,
    const Eigen::VectorXd& q,
    int n_sub, float inv_n,
    const int* map, int n_act,
    std::vector<std::vector<LinkExtremes>>& link_seg_ext,
    FKWorkspace& ws)
{
    ws.compute(robot, q);
    for (int ci = 0; ci < n_act; ++ci) {
        int V = map[ci];
        if (V + 1 >= ws.np) continue;
        const Eigen::Vector3d p_prox = ws.pos(V);
        const Eigen::Vector3d p_dist = ws.pos(V + 1);
        for (int s = 0; s < n_sub; ++s) {
            float t0 = s * inv_n;
            float t1 = (s + 1) * inv_n;
            Eigen::Vector3d s0 = p_prox + double(t0) * (p_dist - p_prox);
            Eigen::Vector3d s1 = p_prox + double(t1) * (p_dist - p_prox);
            link_seg_ext[ci][s].update(s0, q);
            link_seg_ext[ci][s].update(s1, q);
        }
    }
}

// Partial-FK variant: only recompute FK from `from_joint` onward.
inline void eval_and_update_from(
    const Robot& robot,
    const Eigen::VectorXd& q,
    int from_joint,
    int n_sub, float inv_n,
    const int* map, int n_act,
    std::vector<std::vector<LinkExtremes>>& link_seg_ext,
    FKWorkspace& ws)
{
    ws.compute_from(robot, q, from_joint);
    for (int ci = 0; ci < n_act; ++ci) {
        int V = map[ci];
        if (V < from_joint) continue;   // S4: link unaffected by this joint
        if (V + 1 >= ws.np) continue;
        const Eigen::Vector3d p_prox = ws.pos(V);
        const Eigen::Vector3d p_dist = ws.pos(V + 1);
        for (int s = 0; s < n_sub; ++s) {
            float t0 = s * inv_n;
            float t1 = (s + 1) * inv_n;
            Eigen::Vector3d s0 = p_prox + double(t0) * (p_dist - p_prox);
            Eigen::Vector3d s1 = p_prox + double(t1) * (p_dist - p_prox);
            link_seg_ext[ci][s].update(s0, q);
            link_seg_ext[ci][s].update(s1, q);
        }
    }
}

// ─── FixedRoots — stack-allocated root storage ──────────────────────────────

struct FixedRoots {
    double v[8];
    int n = 0;
    void push(double x) { if (n < 8) v[n++] = x; }
    const double* begin() const { return v; }
    const double* end()   const { return v + n; }
    int size() const { return n; }
};

// ─── Interval-restricted polynomial root finder ─────────────────────────────
//
// Finds real roots of p(t) = c[0] + c[1]*t + ... + c[deg]*t^deg
// ONLY within [lo, hi].  Uses sign-change sampling plus bisection.
//
// nsub_override: if > 0, use this many sub-intervals instead of the default 32.
//                Useful for narrower intervals where 16 suffices.
//
inline void solve_poly_in_interval(const double* c, int deg,
                                   double lo, double hi,
                                   FixedRoots& roots,
                                   int nsub_override = 0) {
    while (deg > 0 && std::abs(c[deg]) < 1e-15) --deg;
    if (deg <= 0) return;
    if (hi <= lo) return;

    const int NSUB = (nsub_override > 0) ? nsub_override
                   : ((hi - lo) < 0.5 ? 12 : ((hi - lo) < 2.0 ? 20 : 32));
    constexpr int BISECT_ITER = 48;
    constexpr double REL_TOL = 1e-12;

    auto eval = [&](double t) -> double {
        double val = c[deg];
        for (int i = deg - 1; i >= 0; --i)
            val = val * t + c[i];
        return val;
    };

    double dt = (hi - lo) / NSUB;
    double prev_t = lo;
    double prev_f = eval(lo);

    for (int i = 1; i <= NSUB; ++i) {
        double t = (i < NSUB) ? lo + i * dt : hi;
        double f = eval(t);
        if (prev_f * f < 0) {
            double a = prev_t, b = t;
            double fa = prev_f, fb = f;
            for (int iter = 0; iter < BISECT_ITER; ++iter) {
                double mid = 0.5 * (a + b);
                double fm = eval(mid);
                if (fa * fm <= 0) { b = mid; fb = fm; }
                else              { a = mid; fa = fm; }
                if (b - a < REL_TOL * (1.0 + std::abs(a))) break;
            }
            roots.push(0.5 * (a + b));
        }
        prev_t = t;
        prev_f = f;
    }
}

// ─── Sturm-chain certified polynomial root finder ───────────────────────────
//
// Provably finds ALL real roots of p(t) = c[0] + c[1]*t + ... + c[deg]*t^deg
// within [lo, hi].  Uses Sturm's theorem for exact root counting plus
// Sturm-guided bisection for isolation.  Closes Gap 2 (sampling can miss
// close root pairs in the same sub-interval).
//
// Maximum supported degree: 8 (matching degree-8 bilinear poly from 2D case).

static constexpr int STURM_MAX_DEG   = 8;
static constexpr int STURM_MAX_TERMS = STURM_MAX_DEG + 1;  // coefficients per poly
static constexpr int STURM_MAX_SEQ   = STURM_MAX_DEG + 1;  // max chain length

struct SturmSequence {
    double coeffs[STURM_MAX_SEQ][STURM_MAX_TERMS];
    int    degrees[STURM_MAX_SEQ];
    int    length;
};

/// Horner evaluation of a polynomial at t
inline double sturm_eval(const double* c, int deg, double t) {
    double val = c[deg];
    for (int i = deg - 1; i >= 0; --i)
        val = val * t + c[i];
    return val;
}

/// Build the Sturm sequence for polynomial c of degree deg.
///
/// P₀ = P,  P₁ = P',  Pₖ₊₁ = −rem(Pₖ₋₁, Pₖ)
inline SturmSequence build_sturm_sequence(const double* c, int deg) {
    SturmSequence seq{};
    // P₀
    for (int i = 0; i <= deg; ++i) seq.coeffs[0][i] = c[i];
    seq.degrees[0] = deg;
    // P₁ = P'
    if (deg <= 0) { seq.length = 1; return seq; }
    for (int i = 1; i <= deg; ++i)
        seq.coeffs[1][i - 1] = c[i] * i;
    seq.degrees[1] = deg - 1;
    seq.length = 2;

    for (int step = 2; step < STURM_MAX_SEQ; ++step) {
        int da = seq.degrees[step - 2];
        int db = seq.degrees[step - 1];
        if (db < 0 || std::abs(seq.coeffs[step - 1][db]) < 1e-30) break;
        if (da < db) break;

        // Polynomial long division: A / B → remainder R; store −R
        double work[STURM_MAX_TERMS + 2]{};
        for (int i = 0; i <= da; ++i) work[i] = seq.coeffs[step - 2][i];
        double lead = seq.coeffs[step - 1][db];
        for (int i = da - db; i >= 0; --i) {
            double q = work[i + db] / lead;
            for (int j = 0; j <= db; ++j)
                work[i + j] -= q * seq.coeffs[step - 1][j];
        }
        int rem_deg = db - 1;
        while (rem_deg > 0 && std::abs(work[rem_deg]) < 1e-15) --rem_deg;
        for (int i = 0; i <= rem_deg; ++i)
            seq.coeffs[step][i] = -work[i];
        seq.degrees[step] = rem_deg;
        seq.length = step + 1;
        if (rem_deg <= 0) break;
    }
    return seq;
}

/// Count sign changes in the Sturm sequence evaluated at t
inline int sturm_sign_changes(const SturmSequence& seq, double t) {
    int changes = 0, prev_sign = 0;
    for (int i = 0; i < seq.length; ++i) {
        double val = sturm_eval(seq.coeffs[i], seq.degrees[i], t);
        int sign = (val > 0.0) ? 1 : ((val < 0.0) ? -1 : 0);
        if (sign == 0) continue;
        if (prev_sign != 0 && sign != prev_sign)
            ++changes;
        prev_sign = sign;
    }
    return changes;
}

/// Number of distinct real roots in the open interval (a, b)
inline int sturm_count_roots(const SturmSequence& seq, double a, double b) {
    return sturm_sign_changes(seq, a) - sturm_sign_changes(seq, b);
}

/// Certified root finder: guarantees ALL real roots of c[0..deg] in [lo,hi].
///
/// Uses Sturm-count-guided bisection: never misses a root, even when two
/// roots are arbitrarily close (the Sturm count tells us exactly how many
/// roots remain in each sub-interval).
inline void solve_poly_in_interval_sturm(const double* c, int deg,
                                         double lo, double hi,
                                         FixedRoots& roots) {
    while (deg > 0 && std::abs(c[deg]) < 1e-15) --deg;
    if (deg <= 0 || hi <= lo) return;

    SturmSequence seq = build_sturm_sequence(c, deg);
    int total = sturm_count_roots(seq, lo, hi);

    // Also check endpoints (Sturm gives open-interval count)
    if (std::abs(sturm_eval(c, deg, lo)) < 1e-14) { roots.push(lo); }
    if (std::abs(sturm_eval(c, deg, hi)) < 1e-14) { roots.push(hi); }
    if (total <= 0) return;

    constexpr int    BISECT_MAX = 52;
    constexpr double ABS_TOL    = 1e-14;

    // Stack-based iterative subdivision
    struct Sub { double a, b; int cnt; };
    Sub stack[64];
    int sp = 0;
    stack[sp++] = {lo, hi, total};

    while (sp > 0 && roots.n < 8) {
        Sub cur = stack[--sp];
        if (cur.cnt <= 0) continue;

        if (cur.cnt == 1) {
            // Isolate single root by Sturm-guided bisection
            double a = cur.a, b = cur.b;
            for (int iter = 0; iter < BISECT_MAX; ++iter) {
                if (b - a < ABS_TOL) break;
                double mid = 0.5 * (a + b);
                int left = sturm_count_roots(seq, a, mid);
                if (left >= 1) b = mid; else a = mid;
            }
            roots.push(0.5 * (a + b));
        } else {
            // Multiple roots: split and recurse
            double mid = 0.5 * (cur.a + cur.b);
            int left_cnt  = sturm_count_roots(seq, cur.a, mid);
            int right_cnt = cur.cnt - left_cnt;
            if (right_cnt > 0 && sp < 63)
                stack[sp++] = {mid, cur.b, right_cnt};
            if (left_cnt > 0 && sp < 63)
                stack[sp++] = {cur.a, mid, left_cnt};
        }
    }
}

// ─── Build degree-8 polynomial from bilinear trig coefficients ──────────────
//
// Given bilinear trig model: p_d = a1·ci·cj + a2·ci·sj + a3·si·cj + a4·si·sj
//                               + a5·ci + a6·si + a7·cj + a8·sj + a9
//
// Gradient-zero conditions → after Weierstrass substitution t_j = tan(q_j/2),
// yields F(t_j) = S² − B²(P²+Q²) of degree ≤ 8.
//
inline void build_symbolic_poly8(double a1, double a2, double a3, double a4,
                                 double a5, double a6, double a7, double a8,
                                 double* poly_out) {
    // Half-angle: c=(1-t²)/(1+t²), s=2t/(1+t²), w=1+t²
    // Pw = P*w  (degree 2)
    double Pw[3] = {a3+a6, 2*a4, a6-a3};
    double Qw[3] = {a1+a5, 2*a2, a5-a1};
    double Bw[3] = {a8, -2*a7, -a8};
    // dPw_half = (-a3*s+a4*c)*w  (degree 2)
    double dPw[3] = {a4, -2*a3, -a4};
    double dQw[3] = {a2, -2*a1, -a2};

    // QdQ = Qw * dQw [degree 4]
    double QdQ[5] = {0,0,0,0,0};
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) QdQ[i+j] += Qw[i]*dQw[j];
    // PdP = Pw * dPw [degree 4]
    double PdP[5] = {0,0,0,0,0};
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) PdP[i+j] += Pw[i]*dPw[j];
    // S = QdQ + PdP [degree 4]
    double S[5];
    for (int i=0;i<5;++i) S[i] = QdQ[i] + PdP[i];
    // S² [degree 8]
    double S2[9] = {0,0,0,0,0,0,0,0,0};
    for (int i=0;i<5;++i) for (int j=0;j<5;++j) S2[i+j] += S[i]*S[j];

    // B² [degree 4]
    double B2[5] = {0,0,0,0,0};
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) B2[i+j] += Bw[i]*Bw[j];
    // P² [degree 4]
    double P2[5] = {0,0,0,0,0};
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) P2[i+j] += Pw[i]*Pw[j];
    // Q² [degree 4]
    double Q2[5] = {0,0,0,0,0};
    for (int i=0;i<3;++i) for (int j=0;j<3;++j) Q2[i+j] += Qw[i]*Qw[j];
    // PQ2 = P² + Q² [degree 4]
    double PQ2[5];
    for (int i=0;i<5;++i) PQ2[i] = P2[i] + Q2[i];
    // B²(P²+Q²) [degree 8]
    double B2PQ2[9] = {0,0,0,0,0,0,0,0,0};
    for (int i=0;i<5;++i) for (int j=0;j<5;++j) B2PQ2[i+j] += B2[i]*PQ2[j];

    // F = S² - B²(P²+Q²)
    for (int i=0;i<9;++i) poly_out[i] = S2[i] - B2PQ2[i];
}

// ─── half_angle_to_q — recover q from half-angle root t ─────────────────────
inline bool half_angle_to_q(double t, double lo, double hi, double& q_out) {
    double q = 2.0 * std::atan(t);
    for (double shift : kShifts) {
        double qtest = q + shift;
        if (qtest >= lo - 1e-10 && qtest <= hi + 1e-10) {
            q_out = std::max(lo, std::min(hi, qtest));
            return true;
        }
    }
    return false;
}

// ─── build_bg_values — background set for one joint ─────────────────────────
// Returns {lo, hi} and optionally kπ/2 values within (lo, hi).
inline std::vector<double> build_bg_values(double lo, double hi, bool add_kpi2) {
    std::vector<double> bg = { lo, hi };
    if (add_kpi2) {
        for (int k = -20; k <= 20; ++k) {
            double a = k * HALF_PI;
            if (a > lo + 1e-12 && a < hi - 1e-12)
                bg.push_back(a);
        }
    }
    return bg;
}

} // namespace envelope
} // namespace sbf
