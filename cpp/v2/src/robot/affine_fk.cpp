// SafeBoxForest v2 — Affine-Arithmetic FK implementation
#include "sbf/robot/affine_fk.h"
#include <cassert>

namespace sbf {

// ─── Build AA DH transform for one joint ────────────────────────────────────
//
// DH matrix (same convention as fk_scalar.cpp):
//   [ ct,     -st,     0,   a    ]
//   [ st*ca,   ct*ca, -sa, -d*sa ]
//   [ st*sa,   ct*sa,  ca,  d*ca ]
//   [ 0,       0,      0,   1    ]
//
// For revolute joints: theta = q_i + dh.theta (interval), d = dh.d (scalar).
// For prismatic joints: theta = dh.theta (scalar), d = q_i + dh.d (interval).
//
static AAMatrix4 aa_build_dh(const DHParam& dh, const Interval& iv, int sym_id) {
    double ca = std::cos(dh.alpha);
    double sa = std::sin(dh.alpha);
    double a  = dh.a;

    AffineForm ct, st, d_aa;

    if (dh.joint_type == 0) {
        // Revolute: theta is interval, d is scalar
        double theta_lo = iv.lo + dh.theta;
        double theta_hi = iv.hi + dh.theta;
        AACosSin cs = aa_cos_sin_interval(theta_lo, theta_hi, sym_id);
        ct = cs.cos_val;
        st = cs.sin_val;
        d_aa = AffineForm::constant(dh.d);
    } else {
        // Prismatic: theta is scalar, d is interval
        ct = AffineForm::constant(std::cos(dh.theta));
        st = AffineForm::constant(std::sin(dh.theta));
        d_aa = AffineForm::from_interval(iv.lo + dh.d, iv.hi + dh.d, sym_id);
    }

    AAMatrix4 M;
    // Row 0: [ct, -st, 0, a]
    M.m[0] = ct;
    M.m[1] = aa_neg(st);
    M.m[2] = AffineForm::constant(0.0);
    M.m[3] = AffineForm::constant(a);

    // Row 1: [st*ca, ct*ca, -sa, -d*sa]
    M.m[4] = aa_scale(st, ca);
    M.m[5] = aa_scale(ct, ca);
    M.m[6] = AffineForm::constant(-sa);
    M.m[7] = aa_scale(d_aa, -sa);

    // Row 2: [st*sa, ct*sa, ca, d*ca]
    M.m[8]  = aa_scale(st, sa);
    M.m[9]  = aa_scale(ct, sa);
    M.m[10] = AffineForm::constant(ca);
    M.m[11] = aa_scale(d_aa, ca);

    return M;
}

// Scalar DH matrix (for tool frame or scalar joints)
static AAMatrix4 aa_build_dh_scalar(double alpha, double a, double d, double theta) {
    double ct = std::cos(theta);
    double st = std::sin(theta);
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);

    AAMatrix4 M;
    M.m[0]  = AffineForm::constant(ct);
    M.m[1]  = AffineForm::constant(-st);
    M.m[2]  = AffineForm::constant(0.0);
    M.m[3]  = AffineForm::constant(a);
    M.m[4]  = AffineForm::constant(st * ca);
    M.m[5]  = AffineForm::constant(ct * ca);
    M.m[6]  = AffineForm::constant(-sa);
    M.m[7]  = AffineForm::constant(-d * sa);
    M.m[8]  = AffineForm::constant(st * sa);
    M.m[9]  = AffineForm::constant(ct * sa);
    M.m[10] = AffineForm::constant(ca);
    M.m[11] = AffineForm::constant(d * ca);

    return M;
}

// ─── Full AA FK chain ───────────────────────────────────────────────────────
AAFKResult aa_compute_fk(const Robot& robot,
                         const std::vector<Interval>& intervals) {
    int n = robot.n_joints();
    assert(n <= AA_NSYM && "Robot has more joints than AA_NSYM noise symbols");

    bool has_tool = robot.has_tool();
    int n_tf = n + 1 + (has_tool ? 1 : 0);

    AAFKResult result;
    result.prefix.resize(n_tf);
    result.n_tf = n_tf;

    // prefix[0] = Identity
    result.prefix[0] = AAMatrix4::identity();

    // Chain multiply
    for (int i = 0; i < n; ++i) {
        const auto& dh = robot.dh_params()[i];
        bool is_scalar = (intervals[i].hi - intervals[i].lo < 1e-15);

        AAMatrix4 M;
        if (is_scalar) {
            // Joint pinned to a single value → scalar DH transform (no noise)
            double theta, d;
            if (dh.joint_type == 0) {
                theta = intervals[i].lo + dh.theta;
                d = dh.d;
            } else {
                theta = dh.theta;
                d = intervals[i].lo + dh.d;
            }
            M = aa_build_dh_scalar(dh.alpha, dh.a, d, theta);
        } else {
            M = aa_build_dh(dh, intervals[i], i);
        }

        result.prefix[i + 1] = aa_mat_mul(result.prefix[i], M);
    }

    // Tool frame (scalar)
    if (has_tool) {
        const auto& tool = *robot.tool_frame();
        AAMatrix4 Mt = aa_build_dh_scalar(tool.alpha, tool.a, tool.d, tool.theta);
        result.prefix[n + 1] = aa_mat_mul(result.prefix[n], Mt);
    }

    result.valid = true;
    return result;
}

} // namespace sbf
