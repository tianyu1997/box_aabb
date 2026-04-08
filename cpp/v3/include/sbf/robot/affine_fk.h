// SafeBoxForest v3 — Affine-Arithmetic Forward Kinematics
// Module: sbf::robot
//
// Reduced Affine Arithmetic (AA) for DH-chain FK.
// Each quantity is represented as:
//    x̂ = x₀ + Σᵢ xᵢ·εᵢ + δ·ε*
// where εᵢ ∈ [-1,1] are noise symbols (one per joint), and δ ≥ 0
// is the accumulated nonlinear error bound.
//
// Key advantage over standard Interval Arithmetic (IA):
//   cos(qⱼ) and sin(qⱼ) share noise symbol εⱼ, preserving their
//   cos²+sin²=1 correlation through the FK chain.  This eliminates
//   the "dependency problem" that makes IA overestimate by √2× or more.
//
#pragma once

#include "sbf/common/types.h"
#include "sbf/robot/robot.h"
#include <cmath>
#include <cstring>
#include <vector>

namespace sbf {

// Number of primary noise symbols.  One per joint; 8 suffices for Panda/iiwa.
static constexpr int AA_NSYM = 8;

// ─── Reduced Affine Form ────────────────────────────────────────────────────
struct AffineForm {
    double x0;              // central value
    double xi[AA_NSYM];     // partial deviations (noise coefficients)
    double delta;           // accumulated nonlinear error bound (≥ 0)

    // ——— Bound computation ———
    double radius() const {
        double r = delta;
        for (int i = 0; i < AA_NSYM; ++i)
            r += std::abs(xi[i]);
        return r;
    }
    double lo() const { return x0 - radius(); }
    double hi() const { return x0 + radius(); }

    // ——— Factories ———
    static AffineForm constant(double v) {
        AffineForm a;
        a.x0 = v;
        std::memset(a.xi, 0, sizeof(a.xi));
        a.delta = 0.0;
        return a;
    }

    static AffineForm from_interval(double lo, double hi, int sym_id) {
        AffineForm a;
        a.x0 = 0.5 * (lo + hi);
        std::memset(a.xi, 0, sizeof(a.xi));
        a.xi[sym_id] = 0.5 * (hi - lo);
        a.delta = 0.0;
        return a;
    }
};

// ─── Arithmetic ─────────────────────────────────────────────────────────────

inline AffineForm aa_add(const AffineForm& a, const AffineForm& b) {
    AffineForm r;
    r.x0 = a.x0 + b.x0;
    for (int i = 0; i < AA_NSYM; ++i) r.xi[i] = a.xi[i] + b.xi[i];
    r.delta = a.delta + b.delta;
    return r;
}

inline AffineForm aa_sub(const AffineForm& a, const AffineForm& b) {
    AffineForm r;
    r.x0 = a.x0 - b.x0;
    for (int i = 0; i < AA_NSYM; ++i) r.xi[i] = a.xi[i] - b.xi[i];
    r.delta = a.delta + b.delta;
    return r;
}

inline AffineForm aa_neg(const AffineForm& a) {
    AffineForm r;
    r.x0 = -a.x0;
    for (int i = 0; i < AA_NSYM; ++i) r.xi[i] = -a.xi[i];
    r.delta = a.delta;
    return r;
}

inline AffineForm aa_scale(const AffineForm& a, double s) {
    AffineForm r;
    r.x0 = a.x0 * s;
    for (int i = 0; i < AA_NSYM; ++i) r.xi[i] = a.xi[i] * s;
    r.delta = a.delta * std::abs(s);
    return r;
}

// Reduced AA multiplication:
//   z₀ = x₀·y₀
//   zᵢ = x₀·yᵢ + xᵢ·y₀
//   δ_z = |x₀|·δ_y + δ_x·|y₀| + R_x·R_y
// where R_x = Σ|xᵢ| + δ_x, R_y = Σ|yᵢ| + δ_y.
inline AffineForm aa_mul(const AffineForm& a, const AffineForm& b) {
    AffineForm r;
    r.x0 = a.x0 * b.x0;
    for (int i = 0; i < AA_NSYM; ++i)
        r.xi[i] = a.x0 * b.xi[i] + a.xi[i] * b.x0;

    double ra = a.delta, rb = b.delta;
    for (int i = 0; i < AA_NSYM; ++i) {
        ra += std::abs(a.xi[i]);
        rb += std::abs(b.xi[i]);
    }
    r.delta = std::abs(a.x0) * b.delta + a.delta * std::abs(b.x0) + ra * rb;
    return r;
}

// ─── Trigonometric: min-range Taylor linearisation ──────────────────────────
//
// cos(x̂) ≈ cos(x₀) − sin(x₀)·(x̂−x₀) + R ,  |R| ≤ R_x²/2
// sin(x̂) ≈ sin(x₀) + cos(x₀)·(x̂−x₀) + R ,  |R| ≤ R_x²/2
//
// Both share the SAME noise symbols → cos²+sin² = 1 up to O(R²).
//
struct AACosSin {
    AffineForm cos_val;
    AffineForm sin_val;
};

inline AACosSin aa_cos_sin(const AffineForm& theta) {
    double c0 = std::cos(theta.x0);
    double s0 = std::sin(theta.x0);

    double R = theta.radius();             // max |θ − θ₀|
    double remainder = 0.5 * R * R;        // Taylor 2nd-order bound

    AACosSin cs;
    // cos(θ) ≈ cos(θ₀) − sin(θ₀)·Σθᵢεᵢ   + remainder
    cs.cos_val.x0 = c0;
    for (int i = 0; i < AA_NSYM; ++i)
        cs.cos_val.xi[i] = -s0 * theta.xi[i];
    cs.cos_val.delta = std::abs(s0) * theta.delta + remainder;

    // sin(θ) ≈ sin(θ₀) + cos(θ₀)·Σθᵢεᵢ   + remainder
    cs.sin_val.x0 = s0;
    for (int i = 0; i < AA_NSYM; ++i)
        cs.sin_val.xi[i] = c0 * theta.xi[i];
    cs.sin_val.delta = std::abs(c0) * theta.delta + remainder;

    return cs;
}

// Overload for direct interval input (convenience for DH setup)
inline AACosSin aa_cos_sin_interval(double theta_lo, double theta_hi, int sym_id) {
    AffineForm theta = AffineForm::from_interval(theta_lo, theta_hi, sym_id);
    return aa_cos_sin(theta);
}

// ─── 4×4 homogeneous matrix of AffineForm (first 3 rows only) ──────────────
//   Row-major: m[row*4+col], row=0..2, col=0..3
//   4th row is implicitly [0, 0, 0, 1].
struct AAMatrix4 {
    AffineForm m[12];  // 3 rows × 4 columns

    static AAMatrix4 identity() {
        AAMatrix4 M;
        for (int i = 0; i < 12; ++i) M.m[i] = AffineForm::constant(0.0);
        M.m[0]  = AffineForm::constant(1.0);  // [0][0]
        M.m[5]  = AffineForm::constant(1.0);  // [1][1]
        M.m[10] = AffineForm::constant(1.0);  // [2][2]
        return M;
    }
};

// 4×4 AA matrix multiplication  (A × B, both 3-row representation)
inline AAMatrix4 aa_mat_mul(const AAMatrix4& A, const AAMatrix4& B) {
    AAMatrix4 R;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 4; ++c) {
            AffineForm sum = aa_mul(A.m[r*4+0], B.m[0*4+c]);
            sum = aa_add(sum, aa_mul(A.m[r*4+1], B.m[1*4+c]));
            sum = aa_add(sum, aa_mul(A.m[r*4+2], B.m[2*4+c]));
            if (c == 3)
                sum = aa_add(sum, A.m[r*4+3]);
            R.m[r*4+c] = sum;
        }
    }
    return R;
}

// ─── AA FK Result ───────────────────────────────────────────────────────────
struct AAFKResult {
    std::vector<AAMatrix4> prefix;
    int n_tf = 0;
    bool valid = false;
};

// ─── AA FK Workspace (stack-allocated, incremental recomputation) ───────────
//
// Pre-allocated workspace that avoids heap allocation per aa_compute_fk call.
// Supports incremental recomputation: when only one joint's interval changes,
// call recompute_from() to rebuild only the affected portion of the chain.
//
// Usage:
//   AAFKWorkspace ws;
//   ws.compute(robot, ivs);                        // full computation
//   ws.compute(robot, ivs, max_frame);             // early termination
//   // ... modify ivs[j] ...
//   ws.recompute_from(robot, ivs, j, max_frame);   // incremental
//
struct AAFKWorkspace {
    static constexpr int MAX_TF = 16;

    AAMatrix4 prefix[MAX_TF];   // prefix[0]=I, prefix[i+1]=prefix[i]*dh_i
    int n_computed = 0;          // frames 0..n_computed-1 are valid

    // Full computation.  Computes prefix[0..max_frame].
    // max_frame = -1 → compute full chain (joints + tool if present).
    void compute(const Robot& robot, const Interval* intervals,
                 int max_frame = -1);

    // Incremental: recomputes prefix[from_joint+1..max_frame].
    // Requires prefix[0..from_joint] valid (joints before from_joint unchanged).
    void recompute_from(const Robot& robot, const Interval* intervals,
                        int from_joint, int max_frame = -1);
};

// ─── Core API ───────────────────────────────────────────────────────────────

// Build full AA FK chain.
AAFKResult aa_compute_fk(const Robot& robot,
                         const std::vector<Interval>& intervals);

// Extract position bounds from a cumulative transform.
inline void aa_position_bounds(const AAMatrix4& T,
                               double lo[3], double hi[3]) {
    static const int off[3] = {3, 7, 11};
    for (int d = 0; d < 3; ++d) {
        lo[d] = T.m[off[d]].lo();
        hi[d] = T.m[off[d]].hi();
    }
}

// ─── Extract Link AABBs (AA version, analogous to IA extract_link_aabbs) ───
//
// Computes per-link AABB from AA prefix transforms.
// Each link AABB encloses the origin of the proximal/distal frames,
// same convention as the IA version.
void aa_extract_link_aabbs(const AAFKResult& state,
                           const int* active_link_map, int n_active_links,
                           float* out_aabb,
                           const double* link_radii = nullptr);

// Convenience: compute AA link AABBs directly from robot + intervals.
void aa_compute_link_aabbs(const Robot& robot,
                           const std::vector<Interval>& intervals,
                           float* out_aabb);

} // namespace sbf
