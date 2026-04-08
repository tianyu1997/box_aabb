// ═══════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — DH Chain Analytical Coefficient Extraction
//
//  Directly extracts trigonometric coefficients from the DH chain
//  structure, bypassing FK sampling + QR fitting.
//
//    1D model: p_d = alpha*cos(q_j) + beta*sin(q_j) + gamma
//    2D model: p_d = sum of 9 bilinear terms in (cos/sin q_i, cos/sin q_j)
//
//  Phase B: extract_1d_coefficients  (P1 Edge, P2.5a)
//  Phase C: extract_2d_coefficients  (P2 Face, P2.5b)  — TBD
// ═══════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"
#include "sbf/robot/interval_math.h"
#include "sbf/robot/interval_fk.h"
#include <Eigen/Core>
#include <cmath>

namespace sbf {
namespace envelope {

// ───────────────────────────────────────────────────────────────────
//  1D Coefficient Extraction
// ───────────────────────────────────────────────────────────────────

/// Coefficients for p_d = alpha[d]*cos(q_j) + beta[d]*sin(q_j) + gamma[d]
struct Coeff1D {
    double alpha[3];   // coefficient of cos(q_j), per xyz axis
    double beta[3];    // coefficient of sin(q_j), per xyz axis
    double gamma[3];   // constant term, per xyz axis
};

/// Extract 1D trigonometric coefficients for position of a point
/// along the kinematic chain, as a function of joint q_j.
///
/// Given the factorisation:
///   pos = prefix * T_j(q_j) * suffix_col
///
/// where:
///   prefix   = T_0(q_0) * ... * T_{j-1}(q_{j-1})   (4×4 matrix)
///   suffix_col = the position column [x,y,z,1]^T from the suffix chain
///              = (T_{j+1} * ... * T_n * T_tool) * [0,0,0,1]^T
///   T_j(q_j) = dh_transform(alpha_j, a_j, d_j, q_j + theta_j)
///
/// The position is linear in (cos(q_j+θ_j), sin(q_j+θ_j)), which can
/// be re-expressed as linear in (cos(q_j), sin(q_j)) via angle addition.
///
/// @param prefix   4×4 accumulated transform up to (but not including) joint j
/// @param suffix_pos  3D position = (suffix_chain * [0,0,0,1]^T).head<3>()
/// @param dh_j     DH parameters for joint j
/// @return Coeff1D with alpha, beta, gamma per xyz axis
///
/// Cost: ~30 multiplications, 0 FK calls
inline Coeff1D extract_1d_coefficients(
    const Eigen::Matrix4d& prefix,
    const Eigen::Vector3d& suffix_pos,
    const DHParam& dh_j)
{
    Coeff1D c;

    const double s0 = suffix_pos[0];
    const double s1 = suffix_pos[1];
    const double s2 = suffix_pos[2];

    const double ca = std::cos(dh_j.alpha);
    const double sa = std::sin(dh_j.alpha);
    const double ct_off = std::cos(dh_j.theta);
    const double st_off = std::sin(dh_j.theta);

    // T_j(q_j) * [s0, s1, s2, 1]^T, grouped by cos(q+θ) and sin(q+θ):
    //
    //   dh_transform = [[ct, -st,  0,    a ],
    //                   [st*ca, ct*ca, -sa, -d*sa],
    //                   [st*sa, ct*sa,  ca,  d*ca],
    //                   [0,     0,     0,   1    ]]
    //
    //   v = T_j * S  where S = [s0, s1, s2, 1]^T :
    //     v[0] = s0*ct - s1*st + a
    //     v[1] = ca*(s0*st + s1*ct) - sa*(s2 + d)
    //     v[2] = sa*(s0*st + s1*ct) + ca*(s2 + d)
    //     v[3] = 1
    //
    //   Coefficient of ct = cos(q+θ):  v_ct = [s0,   ca*s1, sa*s1, 0]
    //   Coefficient of st = sin(q+θ):  v_st = [-s1,  ca*s0, sa*s0, 0]
    //   Constant:                      v_c  = [a,   -sa*(s2+d), ca*(s2+d), 1]

    const double v_ct[4] = { s0,        ca * s1,  sa * s1,  0.0 };
    const double v_st[4] = { -s1,       ca * s0,  sa * s0,  0.0 };
    const double s2_d = s2 + dh_j.d;
    const double v_c[4]  = { dh_j.a,  -sa * s2_d, ca * s2_d, 1.0 };

    // p_d = prefix_row_d · (ct * v_ct + st * v_st + v_c)
    //     = ct * (prefix_row_d · v_ct) + st * (prefix_row_d · v_st) + (prefix_row_d · v_c)
    //
    // Let A_d = prefix_row_d · v_ct   (coefficient of cos(q+θ))
    //     B_d = prefix_row_d · v_st   (coefficient of sin(q+θ))
    //     C_d = prefix_row_d · v_c    (constant)
    //
    // Then p_d = A_d * cos(q+θ) + B_d * sin(q+θ) + C_d
    //          = [A_d*cos(θ) + B_d*sin(θ)] * cos(q)
    //          + [-A_d*sin(θ) + B_d*cos(θ)] * sin(q)
    //          + C_d

    for (int d = 0; d < 3; ++d) {
        double A_d = 0.0, B_d = 0.0, C_d = 0.0;
        for (int k = 0; k < 4; ++k) {
            A_d += prefix(d, k) * v_ct[k];
            B_d += prefix(d, k) * v_st[k];
            C_d += prefix(d, k) * v_c[k];
        }

        // Convert from cos(q+θ)/sin(q+θ) basis to cos(q)/sin(q) basis
        c.alpha[d] =  A_d * ct_off + B_d * st_off;
        c.beta[d]  = -A_d * st_off + B_d * ct_off;
        c.gamma[d] = C_d;
    }

    return c;
}

// ───────────────────────────────────────────────────────────────────
//  1D Interval Coefficient Extraction  (Fix 1: Gap 1 closure)
// ───────────────────────────────────────────────────────────────────

/// Interval version of Coeff1D: each coefficient is an Interval enclosing
/// all possible values as the prefix chain varies over its interval.
struct IntervalCoeff1D {
    Interval alpha[3];   // interval enclosure of cos(q_j) coefficient
    Interval beta[3];    // interval enclosure of sin(q_j) coefficient
    Interval gamma[3];   // interval enclosure of constant term
};

/// Extract 1D trigonometric coefficients with interval prefix and suffix.
///
/// Same mathematical derivation as extract_1d_coefficients, but the prefix
/// is an interval 4×4 matrix (from FKState::prefix_lo/hi) and suffix_pos
/// is an interval 3-vector, so the resulting α, β, γ are valid interval
/// enclosures.  This makes the Krawczyk Jacobian J_F(X) a provably valid
/// superset, closing Gap 1 (center-point prefix incompleteness).
///
/// @param prefix_lo  Row-major 4×4 lower bounds of prefix matrix
/// @param prefix_hi  Row-major 4×4 upper bounds of prefix matrix
/// @param suffix_pos_lo  Lower bounds of suffix position (3D)
/// @param suffix_pos_hi  Upper bounds of suffix position (3D)
/// @param dh_j      DH parameters for the free joint j
/// @return IntervalCoeff1D with interval enclosures of alpha, beta, gamma
inline IntervalCoeff1D extract_1d_coefficients_interval(
    const double prefix_lo[16], const double prefix_hi[16],
    const double suffix_pos_lo[3], const double suffix_pos_hi[3],
    const DHParam& dh_j)
{
    IntervalCoeff1D c;

    // Suffix position as intervals
    Interval s0{suffix_pos_lo[0], suffix_pos_hi[0]};
    Interval s1{suffix_pos_lo[1], suffix_pos_hi[1]};
    Interval s2{suffix_pos_lo[2], suffix_pos_hi[2]};

    // DH constants (exact from specification, but outward-round trig)
    double ca_raw = std::cos(dh_j.alpha);
    double sa_raw = std::sin(dh_j.alpha);
    Interval ca{outward_lo(ca_raw), outward_hi(ca_raw)};
    Interval sa{outward_lo(sa_raw), outward_hi(sa_raw)};

    double ct_raw = std::cos(dh_j.theta);
    double st_raw = std::sin(dh_j.theta);
    Interval ct_off{outward_lo(ct_raw), outward_hi(ct_raw)};
    Interval st_off{outward_lo(st_raw), outward_hi(st_raw)};

    // v_ct = [s0, ca*s1, sa*s1, 0]   (coefficient of cos(q+θ))
    // v_st = [-s1, ca*s0, sa*s0, 0]  (coefficient of sin(q+θ))
    // v_c  = [a, -sa*(s2+d), ca*(s2+d), 1]  (constant)
    Interval v_ct[4] = { s0,       ca * s1,  sa * s1,  {0.0, 0.0} };
    Interval v_st[4] = { s1 * (-1.0), ca * s0,  sa * s0,  {0.0, 0.0} };
    Interval s2_d = s2 + Interval{dh_j.d, dh_j.d};
    Interval v_c[4]  = { {dh_j.a, dh_j.a}, sa * (-1.0) * s2_d, ca * s2_d, {1.0, 1.0} };

    for (int d = 0; d < 3; ++d) {
        // A_d = prefix_row_d · v_ct  (interval dot product)
        // B_d = prefix_row_d · v_st
        // C_d = prefix_row_d · v_c
        Interval A_d{0.0, 0.0}, B_d{0.0, 0.0}, C_d{0.0, 0.0};
        for (int k = 0; k < 4; ++k) {
            Interval pref{prefix_lo[d * 4 + k], prefix_hi[d * 4 + k]};
            A_d = A_d + pref * v_ct[k];
            B_d = B_d + pref * v_st[k];
            C_d = C_d + pref * v_c[k];
        }

        // Convert from cos(q+θ)/sin(q+θ) to cos(q)/sin(q) basis
        c.alpha[d] = A_d * ct_off + B_d * st_off;
        c.beta[d]  = A_d * Interval{-1.0, -1.0} * st_off + B_d * ct_off;
        c.gamma[d] = C_d;
    }

    return c;
}

/// Compute the suffix position vector for a given eval_frame.
///
/// suffix_pos = (T_{from+1} * T_{from+2} * ... * T_{eval_frame-1} * T_tool_if_needed)
///              * [0,0,0,1]^T
///
/// This is the position column of the suffix chain starting after joint `from`.
///
/// For the typical P1 case:
///   - distal:  eval_frame = V+1, from = j
///      suffix = T_{j+1}...T_V * (T_tool if V==n) applied to [0,0,0,1]
///   - proximal: eval_frame = V, from = j
///      suffix = T_{j+1}...T_{V-1} applied to [0,0,0,1]
///
/// @param robot   Robot definition
/// @param q       Joint angles (only joints between from+1 and eval_frame-1 matter)
/// @param from    Joint index (exclusive start); suffix starts at from+1
/// @param eval_frame  Frame index (0-based: frame k = after joint k-1)
///                    For link V, frame = V; for link V+1, frame = V+1
/// @return suffix_pos  3D position vector
inline Eigen::Vector3d compute_suffix_pos(
    const Robot& robot,
    const Eigen::VectorXd& q,
    int from,
    int eval_frame)
{
    const int n = robot.n_joints();

    // Build suffix chain from right to left:
    //   suffix = T_{from+1} * T_{from+2} * ... * T_{last}
    // where last joint depends on eval_frame:
    //   frame k corresponds to position after joint k-1
    //   so we need joints from+1 through eval_frame-1

    Eigen::Matrix4d suffix = Eigen::Matrix4d::Identity();

    // Determine the last joint in the suffix chain
    // Frame V = after joint V-1, so eval joints are from+1 .. V-1
    // But we might also need tool frame
    int last_joint = eval_frame - 1;  // last joint that contributes to this frame

    // If eval_frame == n+1 (tool frame), include tool
    bool need_tool = robot.has_tool() && (eval_frame == n + 1);

    for (int k = from + 1; k <= std::min(last_joint, n - 1); ++k) {
        const auto& dh = robot.dh_params()[k];
        double theta, d_val;
        if (dh.joint_type == 0) {
            theta = q[k] + dh.theta;
            d_val = dh.d;
        } else {
            theta = dh.theta;
            d_val = q[k] + dh.d;
        }
        suffix = suffix * dh_transform(dh.alpha, dh.a, d_val, theta);
    }

    if (need_tool) {
        const auto& tool = *robot.tool_frame();
        suffix = suffix * dh_transform(tool.alpha, tool.a, tool.d, tool.theta);
    }

    return suffix.block<3,1>(0, 3);
}

/// Interval version of compute_suffix_pos.
///
/// Uses the interval FK prefix chain (FKState) to extract the suffix
/// position as an interval 3-vector.  For eval_frame = V+1 (distal endpoint
/// of link V), suffix_pos = T_{from+1} * ... * T_V * [0,0,0,1]^T.
///
/// The suffix chain result is obtained from:
///   suffix_pos = (prefix[eval_frame])^{-1} * prefix[from] ... but this
///   requires interval matrix inversion.
///
/// A simpler approach: the suffix chain transforms [0,0,0,1]^T to produce
/// a position vector.  We can compute this using the interval joint matrices
/// stored in FKState.  We build the suffix product right-to-left using
/// imat_mul_dh and extract the position column.
///
/// @param robot       Robot definition
/// @param intervals   Joint intervals for each DOF
/// @param from        Joint index (exclusive start)
/// @param eval_frame  Frame index
/// @param suffix_lo   Output: lower bounds of suffix position [3]
/// @param suffix_hi   Output: upper bounds of suffix position [3]
inline void compute_suffix_pos_interval(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int from,
    int eval_frame,
    double suffix_lo[3], double suffix_hi[3])
{
    const int n = robot.n_joints();
    int last_joint = eval_frame - 1;
    bool need_tool = robot.has_tool() && (eval_frame == n + 1);

    // Build interval suffix chain: T_{from+1} * ... * T_{last} (* T_tool)
    double chain_lo[16], chain_hi[16];
    imat_identity(chain_lo, chain_hi);

    for (int k = from + 1; k <= std::min(last_joint, n - 1); ++k) {
        double jt_lo[16], jt_hi[16];
        build_joint_interval(robot, k, intervals[k], jt_lo, jt_hi);
        double tmp_lo[16], tmp_hi[16];
        imat_mul_dh(chain_lo, chain_hi, jt_lo, jt_hi, tmp_lo, tmp_hi);
        imat_copy(tmp_lo, tmp_hi, chain_lo, chain_hi);
    }

    if (need_tool) {
        const auto& tool = *robot.tool_frame();
        double ct = std::cos(tool.theta), st = std::sin(tool.theta);
        double tl[16], th[16];
        build_dh_joint(tool.alpha, tool.a, ct, ct, st, st,
                       tool.d, tool.d, tl, th);
        double tmp_lo[16], tmp_hi[16];
        imat_mul_dh(chain_lo, chain_hi, tl, th, tmp_lo, tmp_hi);
        imat_copy(tmp_lo, tmp_hi, chain_lo, chain_hi);
    }

    // Position column: elements [3], [7], [11] of the 4×4 matrix
    suffix_lo[0] = chain_lo[3];  suffix_hi[0] = chain_hi[3];
    suffix_lo[1] = chain_lo[7];  suffix_hi[1] = chain_hi[7];
    suffix_lo[2] = chain_lo[11]; suffix_hi[2] = chain_hi[11];
}

// ───────────────────────────────────────────────────────────────────
//  2D Coefficient Extraction
// ───────────────────────────────────────────────────────────────────

/// Coefficients for the bilinear model:
///   p_d = a[0]*c_lo*c_hi + a[1]*c_lo*s_hi + a[2]*s_lo*c_hi + a[3]*s_lo*s_hi
///       + a[4]*c_lo + a[5]*s_lo + a[6]*c_hi + a[7]*s_hi + a[8]
///
/// where c_lo = cos(q_{j_lo}), s_lo = sin(q_{j_lo}),
///       c_hi = cos(q_{j_hi}), s_hi = sin(q_{j_hi}),
///       and j_lo < j_hi are in chain order.
///
/// a[k][d] = coefficient k for axis d (x=0, y=1, z=2)
struct Coeff2D {
    double a[9][3];
};

/// Compute the middle chain matrix between two joints.
///
///   middle = T_{lo_joint+1} * T_{lo_joint+2} * ... * T_{hi_joint-1}
///
/// All joints in this range are "background" (fixed angles).
///
/// @param robot      Robot definition
/// @param q          Joint angles (background joints use their current values)
/// @param lo_joint   Exclusive start joint (middle starts at lo_joint+1)
/// @param hi_joint   Exclusive end joint (middle ends at hi_joint-1)
/// @return 4×4 accumulated transform
inline Eigen::Matrix4d compute_middle_matrix(
    const Robot& robot,
    const Eigen::VectorXd& q,
    int lo_joint,
    int hi_joint)
{
    Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
    for (int k = lo_joint + 1; k < hi_joint; ++k) {
        const auto& dh = robot.dh_params()[k];
        double theta, d_val;
        if (dh.joint_type == 0) {
            theta = q[k] + dh.theta;
            d_val = dh.d;
        } else {
            theta = dh.theta;
            d_val = q[k] + dh.d;
        }
        result = result * dh_transform(dh.alpha, dh.a, d_val, theta);
    }
    return result;
}

/// Extract 2D bilinear trigonometric coefficients from the DH chain.
///
/// Given the full kinematic chain factorisation for two free joints:
///   pos = Prefix * T_lo(q_lo) * Middle * T_hi(q_hi) * [suffix_pos; 1]
///
/// This function decomposes the expression into:
///   p_d = sum of 9 bilinear terms in (cos(q_lo), sin(q_lo), 1) × (cos(q_hi), sin(q_hi), 1)
///
/// The factorisation:
///   Step 1: T_hi * [suffix_pos; 1] → v = c_hi * uc + s_hi * us + u0
///   Step 2: Middle * v → w = c_hi * wc + s_hi * ws + w0
///   Step 3: T_lo * w → decompose each of {wc, ws, w0} into (c_lo, s_lo, 1) terms
///   Step 4: Prefix · each basis vector → 9 scalar coefficients per axis
///
/// @param prefix     4×4 accumulated transform T_0 * ... * T_{lo-1}
/// @param dh_lo      DH parameters for the lower joint (j_lo)
/// @param middle     4×4 accumulated transform T_{lo+1} * ... * T_{hi-1}
/// @param dh_hi      DH parameters for the higher joint (j_hi)
/// @param suffix_pos 3D position = (suffix_chain * [0,0,0,1]^T).head<3>()
/// @return Coeff2D with a[k][d], k=0..8, d=0..2 (in chain order: lo/hi)
///
/// Cost: ~120 multiplications, 0 FK calls
inline Coeff2D extract_2d_coefficients(
    const Eigen::Matrix4d& prefix,
    const DHParam& dh_lo,
    const Eigen::Matrix4d& middle,
    const DHParam& dh_hi,
    const Eigen::Vector3d& suffix_pos)
{
    Coeff2D coeff;

    const double r0 = suffix_pos[0];
    const double r1 = suffix_pos[1];
    const double r2 = suffix_pos[2];

    // ── Step 1: T_hi * [r0,r1,r2,1]^T → decompose into cos(q_hi)/sin(q_hi)/1 ──
    //
    //   DH transform T_hi(q_hi) grouped by ct=cos(q+θ), st=sin(q+θ):
    //     vct = [r0,       ca*r1,  sa*r1,  0]
    //     vst = [-r1,      ca*r0,  sa*r0,  0]
    //     vc  = [a,   -sa*(r2+d), ca*(r2+d), 1]
    //
    //   Convert to cos(q)/sin(q) basis:
    //     uc = cθ * vct + sθ * vst
    //     us = -sθ * vct + cθ * vst

    const double ca_hi = std::cos(dh_hi.alpha);
    const double sa_hi = std::sin(dh_hi.alpha);
    const double ct_hi = std::cos(dh_hi.theta);
    const double st_hi = std::sin(dh_hi.theta);
    const double r2_d_hi = r2 + dh_hi.d;

    const double vct_hi_0 = r0,         vct_hi_1 = ca_hi * r1, vct_hi_2 = sa_hi * r1;
    const double vst_hi_0 = -r1,        vst_hi_1 = ca_hi * r0, vst_hi_2 = sa_hi * r0;

    double uc_hi[4], us_hi[4], u0_hi[4];
    uc_hi[0] = ct_hi * vct_hi_0 + st_hi * vst_hi_0;
    uc_hi[1] = ct_hi * vct_hi_1 + st_hi * vst_hi_1;
    uc_hi[2] = ct_hi * vct_hi_2 + st_hi * vst_hi_2;
    uc_hi[3] = 0.0;

    us_hi[0] = -st_hi * vct_hi_0 + ct_hi * vst_hi_0;
    us_hi[1] = -st_hi * vct_hi_1 + ct_hi * vst_hi_1;
    us_hi[2] = -st_hi * vct_hi_2 + ct_hi * vst_hi_2;
    us_hi[3] = 0.0;

    u0_hi[0] = dh_hi.a;
    u0_hi[1] = -sa_hi * r2_d_hi;
    u0_hi[2] = ca_hi * r2_d_hi;
    u0_hi[3] = 1.0;

    // ── Step 2: Middle * {uc_hi, us_hi, u0_hi} → {wc, ws, w0} ──
    double wc[4], ws[4], w0[4];
    for (int k = 0; k < 4; ++k) {
        wc[k] = 0.0; ws[k] = 0.0; w0[k] = 0.0;
        for (int m = 0; m < 4; ++m) {
            wc[k] += middle(k, m) * uc_hi[m];
            ws[k] += middle(k, m) * us_hi[m];
            w0[k] += middle(k, m) * u0_hi[m];
        }
    }

    // ── Step 3: T_lo decomposition for each w-vector ──
    //
    // For a generic 4D vector w, T_lo * w gives:
    //   ct: [w[0],        ca*w[1],      sa*w[1],      0]
    //   st: [-w[1],       ca*w[0],      sa*w[0],      0]
    //   const: [a*w[3], -sa*(w[2]+d*w[3]), ca*(w[2]+d*w[3]), w[3]]
    //
    // Convert ct/st → c_lo/s_lo via angle-addition.

    const double ca_lo = std::cos(dh_lo.alpha);
    const double sa_lo = std::sin(dh_lo.alpha);
    const double ct_lo = std::cos(dh_lo.theta);
    const double st_lo = std::sin(dh_lo.theta);

    // 9 basis vectors (4D each):
    //   basis[0] = uc_lo(wc) → c_lo * c_hi
    //   basis[1] = uc_lo(ws) → c_lo * s_hi
    //   basis[2] = us_lo(wc) → s_lo * c_hi
    //   basis[3] = us_lo(ws) → s_lo * s_hi
    //   basis[4] = uc_lo(w0) → c_lo
    //   basis[5] = us_lo(w0) → s_lo
    //   basis[6] = u0_lo(wc) → c_hi
    //   basis[7] = u0_lo(ws) → s_hi
    //   basis[8] = u0_lo(w0) → 1

    double basis[9][4];

    auto decompose_T_lo = [&](const double* w,
                              double* uc_out, double* us_out, double* u0_out) {
        const double vct0 = w[0],  vct1 = ca_lo * w[1], vct2 = sa_lo * w[1];
        const double vst0 = -w[1], vst1 = ca_lo * w[0], vst2 = sa_lo * w[0];
        const double w2_d = w[2] + dh_lo.d * w[3];

        uc_out[0] =  ct_lo * vct0 + st_lo * vst0;
        uc_out[1] =  ct_lo * vct1 + st_lo * vst1;
        uc_out[2] =  ct_lo * vct2 + st_lo * vst2;
        uc_out[3] = 0.0;

        us_out[0] = -st_lo * vct0 + ct_lo * vst0;
        us_out[1] = -st_lo * vct1 + ct_lo * vst1;
        us_out[2] = -st_lo * vct2 + ct_lo * vst2;
        us_out[3] = 0.0;

        u0_out[0] = dh_lo.a * w[3];
        u0_out[1] = -sa_lo * w2_d;
        u0_out[2] =  ca_lo * w2_d;
        u0_out[3] = w[3];
    };

    // wc → basis[0](uc_lo), basis[2](us_lo), basis[6](u0_lo)
    decompose_T_lo(wc, basis[0], basis[2], basis[6]);
    // ws → basis[1](uc_lo), basis[3](us_lo), basis[7](u0_lo)
    decompose_T_lo(ws, basis[1], basis[3], basis[7]);
    // w0 → basis[4](uc_lo), basis[5](us_lo), basis[8](u0_lo)
    decompose_T_lo(w0, basis[4], basis[5], basis[8]);

    // ── Step 4: Prefix · each basis vector → 9 coefficients per axis ──
    for (int d = 0; d < 3; ++d) {
        for (int idx = 0; idx < 9; ++idx) {
            double val = 0.0;
            for (int k = 0; k < 4; ++k)
                val += prefix(d, k) * basis[idx][k];
            coeff.a[idx][d] = val;
        }
    }

    return coeff;
}

}  // namespace envelope
}  // namespace sbf
