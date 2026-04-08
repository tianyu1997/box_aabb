// SafeBoxForest v4 — Affine-Arithmetic FK implementation
// 迁移自 v3 affine_fk.cpp
#include "sbf/robot/affine_fk.h"
#include <cassert>
#include <algorithm>

namespace sbf {

// ─── Build AA DH transform for one joint ────────────────────────────────────
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

// Scalar DH matrix (for tool frame or pinned joints)
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

// ─── AAFKWorkspace ──────────────────────────────────────────────────────────

void AAFKWorkspace::compute(const Robot& robot, const Interval* intervals,
                            int max_frame) {
    int n = robot.n_joints();
    bool has_tool = robot.has_tool();
    int full_n_tf = n + 1 + (has_tool ? 1 : 0);
    int limit = (max_frame >= 0) ? std::min(max_frame + 1, full_n_tf) : full_n_tf;

    prefix[0] = AAMatrix4::identity();

    for (int i = 0; i < n && i + 1 < limit; ++i) {
        const auto& dh = robot.dh_params()[i];
        bool is_scalar = (intervals[i].hi - intervals[i].lo < 1e-15);

        AAMatrix4 M;
        if (is_scalar) {
            double theta, d;
            if (dh.joint_type == 0) { theta = intervals[i].lo + dh.theta; d = dh.d; }
            else { theta = dh.theta; d = intervals[i].lo + dh.d; }
            M = aa_build_dh_scalar(dh.alpha, dh.a, d, theta);
        } else {
            M = aa_build_dh(dh, intervals[i], i);
        }
        prefix[i + 1] = aa_mat_mul(prefix[i], M);
    }

    if (has_tool && n + 1 < limit) {
        const auto& tool = *robot.tool_frame();
        AAMatrix4 Mt = aa_build_dh_scalar(tool.alpha, tool.a, tool.d, tool.theta);
        prefix[n + 1] = aa_mat_mul(prefix[n], Mt);
    }

    n_computed = limit;
}

void AAFKWorkspace::recompute_from(const Robot& robot, const Interval* intervals,
                                    int from_joint, int max_frame) {
    int n = robot.n_joints();
    bool has_tool = robot.has_tool();
    int full_n_tf = n + 1 + (has_tool ? 1 : 0);
    int limit = (max_frame >= 0) ? std::min(max_frame + 1, full_n_tf) : full_n_tf;

    // prefix[from_joint] is valid (depends on joints 0..from_joint-1)
    for (int i = from_joint; i < n && i + 1 < limit; ++i) {
        const auto& dh = robot.dh_params()[i];
        bool is_scalar = (intervals[i].hi - intervals[i].lo < 1e-15);

        AAMatrix4 M;
        if (is_scalar) {
            double theta, d;
            if (dh.joint_type == 0) { theta = intervals[i].lo + dh.theta; d = dh.d; }
            else { theta = dh.theta; d = intervals[i].lo + dh.d; }
            M = aa_build_dh_scalar(dh.alpha, dh.a, d, theta);
        } else {
            M = aa_build_dh(dh, intervals[i], i);
        }
        prefix[i + 1] = aa_mat_mul(prefix[i], M);
    }

    if (has_tool && n + 1 < limit && from_joint <= n) {
        const auto& tool = *robot.tool_frame();
        AAMatrix4 Mt = aa_build_dh_scalar(tool.alpha, tool.a, tool.d, tool.theta);
        prefix[n + 1] = aa_mat_mul(prefix[n], Mt);
    }

    n_computed = limit;
}

// ─── Extract Link AABBs from AA FK ──────────────────────────────────────────
void aa_extract_link_aabbs(const AAFKResult& state,
                           const int* active_link_map, int n_active_links,
                           float* out_aabb,
                           const double* link_radii) {
    for (int i = 0; i < n_active_links; ++i) {
        int li = active_link_map[i];

        // Proximal frame: prefix[li], column 3
        double start_lo[3], start_hi[3];
        aa_position_bounds(state.prefix[li], start_lo, start_hi);

        // Distal frame: prefix[li+1], column 3
        double end_lo[3], end_hi[3];
        aa_position_bounds(state.prefix[li + 1], end_lo, end_hi);

        float* out = out_aabb + i * 6;
        out[0] = static_cast<float>(std::min(start_lo[0], end_lo[0]));
        out[1] = static_cast<float>(std::min(start_lo[1], end_lo[1]));
        out[2] = static_cast<float>(std::min(start_lo[2], end_lo[2]));
        out[3] = static_cast<float>(std::max(start_hi[0], end_hi[0]));
        out[4] = static_cast<float>(std::max(start_hi[1], end_hi[1]));
        out[5] = static_cast<float>(std::max(start_hi[2], end_hi[2]));

        if (link_radii) {
            float r = static_cast<float>(link_radii[i]);
            out[0] -= r; out[1] -= r; out[2] -= r;
            out[3] += r; out[4] += r; out[5] += r;
        }
    }
}

void aa_compute_link_aabbs(const Robot& robot,
                           const std::vector<Interval>& intervals,
                           float* out_aabb) {
    AAFKResult state = aa_compute_fk(robot, intervals);
    aa_extract_link_aabbs(state, robot.active_link_map(), robot.n_active_links(),
                          out_aabb, nullptr);
}

} // namespace sbf
