// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
// SafeBoxForest v3 鈥?GCPC Cache Implementation
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//
// KD-tree construction, range query, q鈧€ reconstruction, q鈧?reflection,
// and the combined GCPC + boundary (k蟺/2 + atan2) pipeline with
// two-level AA-based pruning.
//
// Ported from v2.  JSON loading now uses nlohmann/json (via FetchContent).
//
#define _USE_MATH_DEFINES
#include "sbf/envelope/gcpc_cache.h"
#include "sbf/robot/fk.h"
#include "sbf/robot/affine_fk.h"   // Affine-Arithmetic FK for tight pruning

#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>       // companion-matrix polynomial solver
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sbf {
namespace envelope {

static constexpr double HALF_PI = 1.5707963267948966;
static constexpr double kShifts[3] = {0.0, 2*M_PI, -2*M_PI};

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  Polynomial solver utilities (for Phase D / F companion-matrix solves)
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

// Stack-allocated root storage (max 8 real roots for degree-8 polynomial)
struct FixedRoots {
    double v[8];
    int n = 0;
    void push(double x) { if (n < 8) v[n++] = x; }
    const double* begin() const { return v; }
    const double* end()   const { return v + n; }
    int size() const { return n; }
};

// ─── Interval-restricted root finder: sign-change sampling + bisection ───────
// Finds real roots of p(t) = c[0] + c[1]*t + ... + c[deg]*t^deg
// ONLY within [lo, hi]. Much faster than finding all roots then filtering.
// Uses NSUB sub-intervals for root isolation + bisection for refinement.
//
static void solve_poly_in_interval(const double* c, int deg,
                                   double lo, double hi,
                                   FixedRoots& roots) {
    // Reduce degree for near-zero leading coefficients
    while (deg > 0 && std::abs(c[deg]) < 1e-15) --deg;
    if (deg <= 0) return;
    if (hi <= lo) return;

    constexpr int NSUB = 32;   // 32 sub-intervals (enough for degree 8)
    constexpr int BISECT_ITER = 48;
    constexpr double REL_TOL = 1e-12;

    // Horner evaluation
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
            // Sign change → one root in (prev_t, t)
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

// From half-angle t = tan(q/2), recover q in [lo, hi]
static bool half_angle_to_q(double t, double lo, double hi, double& q_out) {
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

// Pre-computed 9脳9 Vandermonde QR for degree-8 polynomial fitting
static Eigen::ColPivHouseholderQR<Eigen::Matrix<double,9,9>>& get_vandermonde_qr() {
    static Eigen::ColPivHouseholderQR<Eigen::Matrix<double,9,9>> qr = []() {
        Eigen::Matrix<double, 9, 9> V_mat;
        for (int k = 0; k < 9; ++k) {
            double t = -2.0 + k * 0.5;
            double pw = 1.0;
            for (int p = 0; p < 9; ++p) {
                V_mat(k, p) = pw;
                pw *= t;
            }
        }
        return V_mat.colPivHouseholderQr();
    }();
    return qr;
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  GcpcFKWorkspace 鈥?pre-allocated FK buffer (avoids heap alloc per call)
//  Same idea as envelope_derive_critical.cpp's FKWorkspace, but uses a
//  fixed-size stack array instead of std::vector.  Placed in anonymous
//  namespace to avoid ODR collision with the other FKWorkspace.
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
namespace {
struct GcpcFKWorkspace {
    Eigen::Matrix4d tf[16];   // stack-allocated (max 7 joints + tool + base = 10)
    int np = 0;

    void compute(const Robot& robot, const Eigen::VectorXd& q) {
        np = fk_transforms_inplace(robot, q, tf);
    }
    Eigen::Vector3d pos(int k) const {
        return tf[k].block<3, 1>(0, 3);
    }
    // Compute T[j+1] = T[j] * dh_transform(q[j])  鈥?single joint step
    void compute_joint(const Robot& robot, const Eigen::VectorXd& q, int j) {
        const auto& dh = robot.dh_params()[j];
        double theta = (dh.joint_type == 0) ? (q[j] + dh.theta) : dh.theta;
        double d     = (dh.joint_type == 0) ? dh.d : (q[j] + dh.d);
        tf[j + 1] = tf[j] * dh_transform(dh.alpha, dh.a, d, theta);
    }
    // Recompute from joint `from` to end (prefix T[0..from] must be valid)
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
} // anonymous namespace

// Build background values: {lo, hi} + optionally k蟺/2 in (lo, hi)
static std::vector<double> build_bg_values(double lo, double hi, bool add_kpi2) {
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

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  q鈧€ reconstruction helpers
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

double GcpcCache::reconstruct_q0_xy(double A, double B) {
    return std::atan2(-B, A);
}

bool GcpcCache::q0_in_range(double q0, double lo0, double hi0) {
    for (double shift : kShifts) {
        double q = q0 + shift;
        if (q >= lo0 - 1e-10 && q <= hi0 + 1e-10)
            return true;
    }
    return false;
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  KD-tree construction
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

void GcpcCache::build_kdtree(GcpcLinkSection& section) {
    int n = section.n_points;
    if (n == 0) {
        section.kd_root = -1;
        return;
    }

    section.kd_tree.clear();
    section.kd_tree.reserve(2 * n);

    std::vector<int> indices(n);
    std::iota(indices.begin(), indices.end(), 0);

    section.kd_root = build_kdtree_recursive(section, indices, 0, n, 0);
}

int GcpcCache::build_kdtree_recursive(
    GcpcLinkSection& section,
    std::vector<int>& indices,
    int lo, int hi, int depth)
{
    if (lo >= hi) return -1;

    int node_idx = static_cast<int>(section.kd_tree.size());
    section.kd_tree.push_back(KdNode{});
    auto& node = section.kd_tree[node_idx];

    if (hi - lo == 1) {
        node.split_dim = -1;
        node.split_val = 0.0;
        node.left = -1;
        node.right = -1;
        node.point_idx = indices[lo];
        return node_idx;
    }

    int n_eff = section.n_eff_joints;
    int dim = depth % n_eff;

    int mid = (lo + hi) / 2;
    std::nth_element(
        indices.begin() + lo,
        indices.begin() + mid,
        indices.begin() + hi,
        [&](int a, int b) {
            return section.points[a].q_eff[dim] < section.points[b].q_eff[dim];
        });

    double median_val = section.points[indices[mid]].q_eff[dim];

    node.split_dim = dim;
    node.split_val = median_val;
    node.point_idx = -1;

    node.left = build_kdtree_recursive(section, indices, lo, mid, depth + 1);
    // Re-fetch node reference since vector may have grown
    section.kd_tree[node_idx].right =
        build_kdtree_recursive(section, indices, mid, hi, depth + 1);

    return node_idx;
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  KD-tree range query
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

void GcpcCache::kdtree_range_query(
    const GcpcLinkSection& section,
    int node_idx,
    const double* lo, const double* hi,
    int depth,
    std::vector<int>& matched_indices) const
{
    if (node_idx < 0) return;
    const auto& node = section.kd_tree[node_idx];

    if (node.point_idx >= 0) {
        const auto& pt = section.points[node.point_idx];
        bool in_range = true;
        for (int d = 0; d < section.n_eff_joints; ++d) {
            if (pt.q_eff[d] < lo[d] - 1e-10 || pt.q_eff[d] > hi[d] + 1e-10) {
                in_range = false;
                break;
            }
        }
        if (in_range) {
            matched_indices.push_back(node.point_idx);
        }
        return;
    }

    int dim = node.split_dim;
    if (lo[dim] <= node.split_val + 1e-10) {
        kdtree_range_query(section, node.left, lo, hi, depth + 1, matched_indices);
    }
    if (hi[dim] > node.split_val - 1e-10) {
        kdtree_range_query(section, node.right, lo, hi, depth + 1, matched_indices);
    }
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  Query: find critical points for a link within intervals
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

void GcpcCache::query_link(
    int link_id,
    const Interval* intervals,
    std::vector<GcpcQueryResult>& results) const
{
    results.clear();

    const GcpcLinkSection* sec = find_section(link_id);
    if (!sec || sec->kd_root < 0) return;

    int n_eff = sec->n_eff_joints;

    double lo_q[7], hi_q[7];
    for (int d = 0; d < n_eff; ++d) {
        lo_q[d] = intervals[d + 1].lo;
        hi_q[d] = intervals[d + 1].hi;
    }

    // 鈹€鈹€ Step 1: Query the half-range [0,蟺] cache 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    double orig_lo_q1 = lo_q[0];
    double orig_hi_q1 = hi_q[0];

    double q1_lo_half = std::max(orig_lo_q1, 0.0);
    double q1_hi_half = std::min(orig_hi_q1, M_PI);

    std::vector<int> matched;

    if (q1_lo_half <= q1_hi_half + 1e-10) {
        lo_q[0] = q1_lo_half;
        hi_q[0] = q1_hi_half;
        kdtree_range_query(*sec, sec->kd_root, lo_q, hi_q, 0, matched);
    }

    for (int idx : matched) {
        const auto& pt = sec->points[idx];
        GcpcQueryResult res;
        res.point = &pt;
        res.q1_reflected = false;
        res.q1_actual = pt.q_eff[0];

        if (pt.direction == 0) {
            // Check all 4 critical q鈧€ candidates:
            //   q0_base      鈫?max p_x
            //   q0_base + 蟺  鈫?min p_x
            //   q0_base + 蟺/2 鈫?max p_y
            //   q0_base - 蟺/2 鈫?min p_y
            double q0_base = reconstruct_q0_xy(pt.A, pt.B);
            double q0_cands[4] = {
                q0_base,
                q0_base + M_PI,
                q0_base + HALF_PI,
                q0_base - HALF_PI
            };
            for (auto& qc : q0_cands) {
                if (qc > M_PI)  qc -= 2 * M_PI;
                if (qc < -M_PI) qc += 2 * M_PI;
            }
            bool found = false;
            for (int c = 0; c < 4; ++c) {
                if (q0_in_range(q0_cands[c], intervals[0].lo, intervals[0].hi)) {
                    if (!found) { res.q0_optimal = q0_cands[c]; found = true; }
                }
            }
            if (!found) continue;
        } else {
            res.q0_optimal = 0.5 * (intervals[0].lo + intervals[0].hi);
        }

        results.push_back(res);
    }

    // 鈹€鈹€ Step 2: q鈧?reflection 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    double refl_lo = std::max(orig_lo_q1, -M_PI);
    double refl_hi = std::min(orig_hi_q1, 0.0);

    if (refl_lo <= refl_hi + 1e-10) {
        double cache_q1_lo = refl_lo + M_PI;
        double cache_q1_hi = refl_hi + M_PI;
        cache_q1_lo = std::max(cache_q1_lo, 0.0);
        cache_q1_hi = std::min(cache_q1_hi, M_PI);

        if (cache_q1_lo <= cache_q1_hi + 1e-10) {
            double lo_q2[7], hi_q2[7];
            for (int d = 0; d < n_eff; ++d) {
                lo_q2[d] = intervals[d + 1].lo;
                hi_q2[d] = intervals[d + 1].hi;
            }
            lo_q2[0] = cache_q1_lo;
            hi_q2[0] = cache_q1_hi;

            std::vector<int> matched_refl;
            kdtree_range_query(*sec, sec->kd_root, lo_q2, hi_q2, 0, matched_refl);

            for (int idx : matched_refl) {
                const auto& pt = sec->points[idx];
                GcpcQueryResult res;
                res.point = &pt;
                res.q1_reflected = true;
                res.q1_actual = pt.q_eff[0] - M_PI;

                if (pt.direction == 0) {
                    double A_refl = -pt.A;
                    double B_refl = pt.B;
                    double q0_base = reconstruct_q0_xy(A_refl, B_refl);
                    double q0_cands[4] = {
                        q0_base,
                        q0_base + M_PI,
                        q0_base + HALF_PI,
                        q0_base - HALF_PI
                    };
                    for (auto& qc : q0_cands) {
                        if (qc > M_PI)  qc -= 2 * M_PI;
                        if (qc < -M_PI) qc += 2 * M_PI;
                    }
                    bool found = false;
                    for (int c = 0; c < 4; ++c) {
                        if (q0_in_range(q0_cands[c], intervals[0].lo, intervals[0].hi)) {
                            if (!found) { res.q0_optimal = q0_cands[c]; found = true; }
                        }
                    }
                    if (!found) continue;
                } else {
                    res.q0_optimal = 0.5 * (intervals[0].lo + intervals[0].hi);
                }

                results.push_back(res);
            }
        }
    }
}

// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  Main GCPC query pipeline: derive_aabb_with_gcpc
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

void GcpcCache::derive_aabb_with_gcpc(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    float* out_aabb,
    GcpcQueryStats* out_stats,
    float* out_endpoint_aabb) const
{
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* map = robot.active_link_map();
    const double* rad = robot.active_link_radii();
    const float inv_n = 1.0f / static_cast<float>(n_sub);

    GcpcQueryStats stats{};

    // Initialize output AABBs to inverted bounds
    for (int ci = 0; ci < n_act; ++ci) {
        for (int s = 0; s < n_sub; ++s) {
            float* a = out_aabb + (ci * n_sub + s) * 6;
            a[0] = a[1] = a[2] =  1e30f;
            a[3] = a[4] = a[5] = -1e30f;
        }
    }

    // Initialize endpoint interval AABBs (n_sub+1 per link)
    if (out_endpoint_aabb) {
        for (int ci = 0; ci < n_act; ++ci) {
            for (int s = 0; s <= n_sub; ++s) {
                float* fa = out_endpoint_aabb + (ci * (n_sub + 1) + s) * 6;
                fa[0] = fa[1] = fa[2] =  1e30f;
                fa[3] = fa[4] = fa[5] = -1e30f;
            }
        }
    }

    // 鈹€鈹€ Config tracking for Phase G seeds 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    //  Track per (link, face) the config that produces the most extreme
    //  distal-point value.  face = dim*2 + (is_max ? 1 : 0)
    //    face 0: x_min, 1: x_max, 2: y_min, 3: y_max, 4: z_min, 5: z_max
    //
    std::vector<Eigen::VectorXd> best_face_config(n_act * 6,
                                                    Eigen::VectorXd::Zero(n));
    std::vector<double> best_face_val(n_act * 6);
    for (int ci = 0; ci < n_act; ++ci) {
        for (int d = 0; d < 3; ++d) {
            best_face_val[ci * 6 + d * 2]     =  1e30;  // min: init high
            best_face_val[ci * 6 + d * 2 + 1] = -1e30;  // max: init low
        }
    }

    // Pre-allocated FK workspace (avoids heap allocation per call)
    GcpcFKWorkspace ws;

    // Helper: evaluate a full-DOF config and update AABBs + track configs
    auto eval_config = [&](const Eigen::VectorXd& q) {
        ws.compute(robot, q);
        ++stats.n_fk_calls;

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

                float* a = out_aabb + (ci * n_sub + s) * 6;
                for (int d = 0; d < 3; ++d) {
                    float vmin = static_cast<float>(std::min(s0[d], s1[d]));
                    float vmax = static_cast<float>(std::max(s0[d], s1[d]));
                    if (vmin < a[d])     a[d]     = vmin;
                    if (vmax > a[d + 3]) a[d + 3] = vmax;
                }

                // Track per-endpoint interval AABBs for hull16
                if (out_endpoint_aabb) {
                    float* fa0 = out_endpoint_aabb + (ci * (n_sub + 1) + s) * 6;
                    float* fa1 = out_endpoint_aabb + (ci * (n_sub + 1) + s + 1) * 6;
                    for (int d = 0; d < 3; ++d) {
                        float v0 = static_cast<float>(s0[d]);
                        float v1 = static_cast<float>(s1[d]);
                        if (v0 < fa0[d])     fa0[d]     = v0;
                        if (v0 > fa0[d + 3]) fa0[d + 3] = v0;
                        if (v1 < fa1[d])     fa1[d]     = v1;
                        if (v1 > fa1[d + 3]) fa1[d + 3] = v1;
                    }
                }
            }

            // Track best config per link per face (distal point)
            for (int d = 0; d < 3; ++d) {
                double v = p_dist[d];
                int idx_min = ci * 6 + d * 2;
                int idx_max = ci * 6 + d * 2 + 1;
                if (v < best_face_val[idx_min]) {
                    best_face_val[idx_min] = v;
                    best_face_config[idx_min] = q;
                }
                if (v > best_face_val[idx_max]) {
                    best_face_val[idx_max] = v;
                    best_face_config[idx_max] = q;
                }
            }
        }
    };

    auto _tA = std::chrono::high_resolution_clock::now();

    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
    //  Phase A: Cache lookup
    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲

    for (int ci = 0; ci < n_act; ++ci) {
        int link_id = map[ci];
        std::vector<GcpcQueryResult> results;
        query_link(link_id, intervals.data(), results);
        stats.n_cache_matches += static_cast<int>(results.size());

        for (const auto& res : results) {
            const auto& pt = *res.point;
            if (res.q1_reflected) ++stats.n_q1_reflected;
            ++stats.n_q0_valid;

            Eigen::VectorXd q(n);
            for (int j = 0; j < n; ++j) q[j] = intervals[j].mid();

            q[0] = res.q0_optimal;
            for (int d = 0; d < pt.n_eff; ++d) {
                if (res.q1_reflected && d == 0) {
                    q[d + 1] = res.q1_actual;
                } else {
                    q[d + 1] = pt.q_eff[d];
                }
            }

            eval_config(q);

            if (pt.direction == 0) {
                // q鈧€+蟺 鈫?opposite extremum for p_x (min p_x if base = max p_x)
                double q0_opp = res.q0_optimal + M_PI;
                if (q0_opp > M_PI) q0_opp -= 2 * M_PI;
                if (q0_in_range(q0_opp, intervals[0].lo, intervals[0].hi)) {
                    q[0] = q0_opp;
                    eval_config(q);
                }
                double q0_y_max = res.q0_optimal + HALF_PI;
                if (q0_y_max > M_PI) q0_y_max -= 2 * M_PI;
                if (q0_in_range(q0_y_max, intervals[0].lo, intervals[0].hi)) {
                    q[0] = q0_y_max;
                    eval_config(q);
                }
                double q0_y_min = res.q0_optimal - HALF_PI;
                if (q0_y_min < -M_PI) q0_y_min += 2 * M_PI;
                if (q0_in_range(q0_y_min, intervals[0].lo, intervals[0].hi)) {
                    q[0] = q0_y_min;
                    eval_config(q);
                }
            }
        }
    }

    stats.phase_a_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - _tA).count();

    auto _tB = std::chrono::high_resolution_clock::now();

    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
    //  Phase B: Boundary k蟺/2 enumeration
    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲

    {
        auto crit_angles_fn = [](double lo, double hi) -> std::vector<double> {
            std::vector<double> v = { lo, hi };
            for (int k = -20; k <= 20; ++k) {
                double a = k * HALF_PI;
                if (a > lo && a < hi)
                    v.push_back(a);
            }
            std::sort(v.begin(), v.end());
            v.erase(std::unique(v.begin(), v.end()), v.end());
            return v;
        };

        std::vector<std::vector<double>> per_joint(n);
        for (int j = 0; j < n; ++j)
            per_joint[j] = crit_angles_fn(intervals[j].lo, intervals[j].hi);

        int max_V = 0;
        for (int ci = 0; ci < n_act; ++ci)
            max_V = std::max(max_V, map[ci]);
        int n_joints_needed = std::min(max_V + 1, n);

        long long product = 1;
        for (int j = 0; j < n_joints_needed; ++j) {
            product *= static_cast<long long>(per_joint[j].size());
            if (product > 60000) break;
        }

        if (product <= 60000) {
            Eigen::VectorXd q(n);
            for (int j = 0; j < n; ++j) q[j] = intervals[j].mid();

            std::function<void(int)> enumerate;
            enumerate = [&](int j) {
                if (j >= n_joints_needed) {
                    eval_config(q);
                    ++stats.n_boundary_kpi2;
                    return;
                }
                for (double v : per_joint[j]) {
                    q[j] = v;
                    enumerate(j + 1);
                }
            };
            enumerate(0);
        } else {
            Eigen::VectorXd q(n);
            for (int j = 0; j < n; ++j) q[j] = intervals[j].mid();

            // Phase B-1: all 2^n corners (lo/hi per joint)
            int n_corners = std::min(1 << n_joints_needed, 1024);
            for (int mask = 0; mask < n_corners; ++mask) {
                for (int j = 0; j < n_joints_needed && j < 10; ++j)
                    q[j] = (mask & (1 << j)) ? intervals[j].hi : intervals[j].lo;
                eval_config(q);
                ++stats.n_boundary_kpi2;
            }

            // Phase B-2: random k蟺/2 sampling from full grid
            int budget = std::max(0, 60000 - n_corners);
            std::mt19937 rng_b(12345);
            for (int trial = 0; trial < budget; ++trial) {
                for (int j = 0; j < n_joints_needed; ++j) {
                    int sz = static_cast<int>(per_joint[j].size());
                    q[j] = per_joint[j][
                        std::uniform_int_distribution<int>(0, sz - 1)(rng_b)];
                }
                for (int j = n_joints_needed; j < n; ++j)
                    q[j] = intervals[j].mid();
                eval_config(q);
                ++stats.n_boundary_kpi2;
            }
        }
    }

    stats.phase_b_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - _tB).count();

    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
    //  Shared: Compute full-box AA FK once (used by Phases C鈥揊)
    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲

    // 鈹€鈹€ dual_phase_g checkpoint: save Phase A+B state 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
    //  When dual_phase3 is enabled in Analytical, it runs Phase 3 twice:
    //  once from Phase 0+1+2+2.5 state, once from Phase 0+1 checkpoint.
    //  We mirror this: save best_face_config after Phase A+B.
    std::vector<Eigen::VectorXd> checkpoint_AB = best_face_config;

    AAFKWorkspace aa_fk_ws;
    aa_fk_ws.compute(robot, intervals.data());

    // Check if AA bounds for frame pair (prox, dist) can improve out_aabb
    auto aa_link_can_improve = [&](const AAMatrix4* prefix,
                                   int ci) -> bool {
        int li = map[ci];
        double plo[3], phi[3], dlo[3], dhi[3];
        aa_position_bounds(prefix[li],     plo, phi);
        aa_position_bounds(prefix[li + 1], dlo, dhi);
        for (int s = 0; s < n_sub; ++s) {
            float t0 = static_cast<float>(s) * inv_n;
            float t1 = static_cast<float>(s + 1) * inv_n;
            const float* oa = out_aabb + (ci * n_sub + s) * 6;
            for (int d = 0; d < 3; ++d) {
                float pA_lo = static_cast<float>(plo[d]);
                float pA_hi = static_cast<float>(phi[d]);
                float pB_lo = static_cast<float>(dlo[d]);
                float pB_hi = static_cast<float>(dhi[d]);
                float s0_lo = pA_lo * (1.f - t0) + pB_lo * t0;
                float s0_hi = pA_hi * (1.f - t0) + pB_hi * t0;
                float s1_lo = pA_lo * (1.f - t1) + pB_lo * t1;
                float s1_hi = pA_hi * (1.f - t1) + pB_hi * t1;
                float aa_lo = std::min(s0_lo, s1_lo);
                float aa_hi = std::max(s0_hi, s1_hi);
                if (aa_lo < oa[d] || aa_hi > oa[d + 3])
                    return true;
            }
        }
        return false;
    };

    // #5: Per-direction AA pruning 鈥?check if dimension d specifically can improve
    auto aa_dim_can_improve = [&](const AAMatrix4* prefix,
                                   int ci, int dim) -> bool {
        int li = map[ci];
        double plo[3], phi[3], dlo[3], dhi[3];
        aa_position_bounds(prefix[li],     plo, phi);
        aa_position_bounds(prefix[li + 1], dlo, dhi);
        for (int s = 0; s < n_sub; ++s) {
            float t0 = static_cast<float>(s) * inv_n;
            float t1 = static_cast<float>(s + 1) * inv_n;
            const float* oa = out_aabb + (ci * n_sub + s) * 6;
            float pA_lo = static_cast<float>(plo[dim]);
            float pA_hi = static_cast<float>(phi[dim]);
            float pB_lo = static_cast<float>(dlo[dim]);
            float pB_hi = static_cast<float>(dhi[dim]);
            float s0_lo = pA_lo * (1.f - t0) + pB_lo * t0;
            float s0_hi = pA_hi * (1.f - t0) + pB_hi * t0;
            float s1_lo = pA_lo * (1.f - t1) + pB_lo * t1;
            float s1_hi = pA_hi * (1.f - t1) + pB_hi * t1;
            float aa_lo = std::min(s0_lo, s1_lo);
            float aa_hi = std::max(s0_hi, s1_hi);
            if (aa_lo < oa[dim] || aa_hi > oa[dim + 3])
                return true;
        }
        return false;
    };

    Eigen::VectorXd q(n);

    auto _tC = std::chrono::high_resolution_clock::now();

    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
    //  Phase C: Boundary 1D atan2  (with two-level AA pruning)
    //  FIXED: now also processes proximal link (V) alongside distal (V+1)
    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲

    for (int ci = 0; ci < n_act; ++ci) {
        int V = map[ci];
        int nj = std::min(V + 1, n);
        bool has_prox = (V >= 1);

        int n_bg_bits = nj - 1;
        int max_bg = std::min(1 << n_bg_bits, 32);  // reduced from 128

        for (int j = 0; j < nj; ++j) {
            double lo_j = intervals[j].lo;
            double hi_j = intervals[j].hi;
            if (hi_j - lo_j < 1e-12) continue;

            double mid_j = 0.5 * (lo_j + hi_j);
            double qvals[3] = { lo_j, mid_j, hi_j };

            // Level 1: full-box AA check for this link
            {
                ++stats.n_aa_prune_checks;
                if (!aa_link_can_improve(aa_fk_ws.prefix, ci)) {
                    ++stats.n_aa_pruned;
                    continue;
                }
            }

            // Pre-compute trig basis + QR (same for all backgrounds)
            Eigen::Matrix3d A_trig;
            for (int si = 0; si < 3; ++si) {
                A_trig(si, 0) = std::cos(qvals[si]);
                A_trig(si, 1) = std::sin(qvals[si]);
                A_trig(si, 2) = 1.0;
            }
            auto qr_A = A_trig.colPivHouseholderQr();

            bool do_level2 = (max_bg >= 4);

            // Build reversed bg joint map: highest-index first for Gray code
            // (most frequently changing bit → highest joint → least recomputation)
            int bg_joints_c[12];
            int n_bg_c = 0;
            for (int jj = nj - 1; jj >= 0; --jj) {
                if (jj == j) continue;
                bg_joints_c[n_bg_c++] = jj;
            }

            // Pre-allocate face intervals on stack (avoid per-bg heap alloc)
            Interval face_iv_c[AAFKWorkspace::MAX_TF];
            for (int jj = 0; jj < n; ++jj) face_iv_c[jj] = intervals[jj];
            face_iv_c[j] = intervals[j]; // free joint

            AAFKWorkspace aa_face_ws;
            int max_aa_frame_c = V + 1;   // early termination

            for (int bg = 0; bg < max_bg; ++bg) {
                int gray = bg ^ (bg >> 1);

                for (int jj = 0; jj < n; ++jj) q[jj] = intervals[jj].mid();
                for (int b = 0; b < n_bg_c; ++b) {
                    int jj = bg_joints_c[b];
                    double bv = (gray & (1 << b)) ? intervals[jj].hi : intervals[jj].lo;
                    q[jj] = bv;
                }

                // Level 2: per-face 1-DOF AA check (incremental + early termination)
                if (do_level2) {
                    if (bg == 0) {
                        for (int b = 0; b < n_bg_c; ++b) {
                            int jj = bg_joints_c[b];
                            face_iv_c[jj] = Interval{q[jj], q[jj]};
                        }
                        aa_face_ws.compute(robot, face_iv_c, max_aa_frame_c);
                    } else {
                        // Gray code: exactly one bit changed — find it
                        int cb = 0;
                        for (int i = 0; ; ++i) if (bg & (1 << i)) { cb = i; break; }
                        int changed_jj = bg_joints_c[cb];
                        face_iv_c[changed_jj] = Interval{q[changed_jj], q[changed_jj]};
                        aa_face_ws.recompute_from(robot, face_iv_c, changed_jj,
                                                  max_aa_frame_c);
                    }
                    ++stats.n_aa_prune_checks;
                    if (!aa_link_can_improve(aa_face_ws.prefix, ci)) {
                        ++stats.n_aa_pruned;
                        continue;
                    }
                    ++stats.n_aa_not_pruned;
                }

                // Evaluate FK at 3 sample points 鈥?partial FK: prefix once
                Eigen::Vector3d pts_dist[3], pts_prox[3];
                bool ok = true;
                {
                    // Build prefix T[0..j-1] (background joints are pinned)
                    ws.tf[0] = Eigen::Matrix4d::Identity();
                    for (int jj = 0; jj < j; ++jj)
                        ws.compute_joint(robot, q, jj);
                    for (int si = 0; si < 3; ++si) {
                        q[j] = qvals[si];
                        ws.compute_from(robot, q, j);
                        ++stats.n_fk_calls;
                        if (V + 1 >= ws.np) { ok = false; break; }
                        pts_dist[si] = ws.pos(V + 1);
                        if (has_prox) pts_prox[si] = ws.pos(V);
                    }
                }
                if (!ok) continue;

                // 鈹€鈹€ 1D atan2 solve helper (processes one eval_link) 鈹€鈹€
                auto solve_atan2_1d = [&](const Eigen::Vector3d pts3[3]) {
                    Eigen::Vector3d b_vec, coeff;
                    for (int d = 0; d < 3; ++d) {
                        b_vec << pts3[0][d], pts3[1][d], pts3[2][d];
                        coeff = qr_A.solve(b_vec);
                        double ac = coeff[0], bc = coeff[1];
                        if (std::abs(ac) < 1e-15 && std::abs(bc) < 1e-15) continue;

                        double q_star = std::atan2(bc, ac);
                        double cands[2] = { q_star, q_star + M_PI };
                        if (cands[1] > M_PI + 0.1) cands[1] -= 2 * M_PI;

                        for (double qc : cands) {
                            for (double shift : kShifts) {
                                double qt = qc + shift;
                                if (qt >= lo_j - 1e-12 && qt <= hi_j + 1e-12) {
                                    qt = std::max(lo_j, std::min(hi_j, qt));
                                    q[j] = qt;
                                    eval_config(q);
                                    ++stats.n_boundary_atan2;
                                    break;
                                }
                            }
                        }
                    }
                };

                // Process distal (V+1)
                solve_atan2_1d(pts_dist);

                // Process proximal (V) 鈥?same atan2 solve, no extra FK calls
                if (has_prox)
                    solve_atan2_1d(pts_prox);
            }
        }
    }

    stats.phase_c_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - _tC).count();

    auto _tD = std::chrono::high_resolution_clock::now();

    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
    //  Phase D: 2D Face Solve  (companion-matrix deg-8 polynomial)
    //  with two-level AA pruning
    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
    //
    //  For two free joints (qi, qj) with all others on boundaries:
    //    p_d(qi,qj) = a1路ci路cj + a2路ci路sj + a3路si路cj + a4路si路sj
    //               + a5路ci + a6路si + a7路cj + a8路sj + a9
    //  Solve 鈭俻/鈭俼i = 0 鈭?鈭俻/鈭俼j = 0 鈫?degree-8 polynomial in t_j.
    //
    //  Face pairs: coupled_pairs + adjacent + skip-1 (matching AnalyticalCrit)
    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲

    {
        // Build face pairs: coupled + adjacent + skip-1 (same as AnalyticalCrit)
        std::vector<std::pair<int,int>> face_pairs;
        {
            auto cp = robot.coupled_pairs();
            face_pairs.insert(face_pairs.end(), cp.begin(), cp.end());
            for (int i = 0; i < n - 1; ++i) {
                std::pair<int,int> p = {i, i+1};
                bool dup = false;
                for (auto& fp : face_pairs) if (fp == p) { dup = true; break; }
                if (!dup) face_pairs.push_back(p);
            }
        }

        for (int ci = 0; ci < n_act; ++ci) {
            int V = map[ci];
            int nj = std::min(V + 1, n);

            // Level 1: full-box AA check for this link
            ++stats.n_aa_prune_checks;
            if (!aa_link_can_improve(aa_fk_ws.prefix, ci)) {
                ++stats.n_aa_pruned;
                continue;
            }

            for (auto& [pi, pj] : face_pairs) {
                if (pi >= nj || pj >= nj) continue;
                double lo_i = intervals[pi].lo, hi_i = intervals[pi].hi;
                double lo_j = intervals[pj].lo, hi_j = intervals[pj].hi;
                if (hi_i - lo_i < 1e-12 || hi_j - lo_j < 1e-12) continue;

                // Pre-compute tan half-angle bounds (constant for this pair)
                double tj_lo_pre = std::tan(lo_j * 0.5);
                double tj_hi_pre = std::tan(hi_j * 0.5);
                if (tj_lo_pre > tj_hi_pre) std::swap(tj_lo_pre, tj_hi_pre);

                // Background combos: all joints except pi,pj 鈫?{lo, hi}
                int n_bg_joints = nj - 2;
                int max_bg_face = std::min(1 << std::max(n_bg_joints, 0), 16);

                // 3脳3 sample grid
                double qi_vals[3] = { lo_i, 0.5*(lo_i+hi_i), hi_i };
                double qj_vals[3] = { lo_j, 0.5*(lo_j+hi_j), hi_j };

                // Pre-compute 9脳9 trig basis matrix
                Eigen::Matrix<double, 9, 9> A_mat;
                {
                    int row = 0;
                    for (int si = 0; si < 3; ++si) {
                        double ci_v = std::cos(qi_vals[si]);
                        double si_v = std::sin(qi_vals[si]);
                        for (int sj = 0; sj < 3; ++sj) {
                            double cj_v = std::cos(qj_vals[sj]);
                            double sj_v = std::sin(qj_vals[sj]);
                            A_mat(row,0) = ci_v*cj_v; A_mat(row,1) = ci_v*sj_v;
                            A_mat(row,2) = si_v*cj_v; A_mat(row,3) = si_v*sj_v;
                            A_mat(row,4) = ci_v;       A_mat(row,5) = si_v;
                            A_mat(row,6) = cj_v;       A_mat(row,7) = sj_v;
                            A_mat(row,8) = 1.0;
                            ++row;
                        }
                    }
                }
                auto qr_9 = A_mat.colPivHouseholderQr();

                // Pre-allocate interval array for AA pruning (stack, no heap)
                bool do_aa_d = (max_bg_face >= 4);
                Interval face2_iv_d[AAFKWorkspace::MAX_TF];
                if (do_aa_d) {
                    for (int jj = 0; jj < n; ++jj) face2_iv_d[jj] = intervals[jj];
                    face2_iv_d[pi] = intervals[pi];
                    face2_iv_d[pj] = intervals[pj];
                }

                // Build reversed bg joint map for Gray code
                int bg_joints_d[12];
                int n_bg_d = 0;
                for (int jj = nj - 1; jj >= 0; --jj) {
                    if (jj == pi || jj == pj) continue;
                    bg_joints_d[n_bg_d++] = jj;
                }

                AAFKWorkspace aa_face2_ws;
                int max_aa_frame_d = V + 1;   // early termination

                for (int bg = 0; bg < max_bg_face; ++bg) {
                    int gray = bg ^ (bg >> 1);

                    for (int jj = 0; jj < n; ++jj) q[jj] = intervals[jj].mid();
                    for (int b = 0; b < n_bg_d; ++b) {
                        int jj = bg_joints_d[b];
                        double bv = (gray & (1 << b)) ? intervals[jj].hi : intervals[jj].lo;
                        q[jj] = bv;
                    }

                    // Level 2: per-face 2-DOF AA check (incremental + early termination)
                    if (do_aa_d) {
                        if (bg == 0) {
                            for (int b = 0; b < n_bg_d; ++b) {
                                int jj = bg_joints_d[b];
                                face2_iv_d[jj] = Interval{q[jj], q[jj]};
                            }
                            aa_face2_ws.compute(robot, face2_iv_d, max_aa_frame_d);
                        } else {
                            int cb = 0;
                            for (int i = 0; ; ++i) if (bg & (1 << i)) { cb = i; break; }
                            int changed_jj = bg_joints_d[cb];
                            face2_iv_d[changed_jj] = Interval{q[changed_jj], q[changed_jj]};
                            aa_face2_ws.recompute_from(robot, face2_iv_d, changed_jj,
                                                       max_aa_frame_d);
                        }
                        ++stats.n_aa_prune_checks;
                        if (!aa_link_can_improve(aa_face2_ws.prefix, ci)) {
                            ++stats.n_phase_d_pruned;
                            ++stats.n_aa_pruned;
                            continue;
                        }
                        ++stats.n_aa_not_pruned;
                    }

                    // Partial FK: compute prefix once, then reuse for 9 samples
                    int min_free_d = std::min(pi, pj);
                    ws.compute_prefix(robot, q, min_free_d);

                    // Evaluate 9 FK samples (3脳3 grid)
                    Eigen::Vector3d pos_V_cache[9], pos_V1_cache[9];
                    bool valid = true;
                    {
                        int row = 0;
                        for (int si = 0; si < 3 && valid; ++si) {
                            for (int sj = 0; sj < 3 && valid; ++sj) {
                                q[pi] = qi_vals[si];
                                q[pj] = qj_vals[sj];
                                ws.compute_from(robot, q, min_free_d);
                                ++stats.n_fk_calls;
                                ++stats.n_phase_d_fk;
                                if (V + 1 >= ws.np) { valid = false; break; }
                                pos_V_cache[row] = ws.pos(V);
                                pos_V1_cache[row] = ws.pos(V + 1);
                                ++row;
                            }
                        }
                    }
                    if (!valid) continue;

                    // Process both distal (V+1) and proximal (V)
                    for (int eval_idx = 0; eval_idx < 2; ++eval_idx) {
                        int eval_link = (eval_idx == 0) ? V + 1 : V;
                        if (eval_link < 1) continue;
                        const Eigen::Vector3d* pc = (eval_idx == 0) ? pos_V1_cache : pos_V_cache;

                        // Batch QR solve for all 3 dimensions at once
                        Eigen::Matrix<double, 9, 3> rhs_3d;
                        for (int r = 0; r < 9; ++r)
                            for (int dd = 0; dd < 3; ++dd)
                                rhs_3d(r, dd) = pc[r][dd];
                        Eigen::Matrix<double, 9, 3> coeff_3d = qr_9.solve(rhs_3d);

                        for (int d = 0; d < 3; ++d) {
                            // #5: Per-direction AA pruning
                            if (do_aa_d && !aa_dim_can_improve(aa_fk_ws.prefix, ci, d))
                                continue;

                            double a1=coeff_3d(0,d), a2=coeff_3d(1,d), a3=coeff_3d(2,d), a4=coeff_3d(3,d);
                            double a5=coeff_3d(4,d), a6=coeff_3d(5,d), a7=coeff_3d(6,d), a8=coeff_3d(7,d);

                            // #9: Symbolic polynomial coefficients (no Vandermonde fitting)
                            // Half-angle substitution: c=(1-t虏)/(1+t虏), s=2t/(1+t虏), w=1+t虏
                            // Pw = P*w = (a6-a3)t虏 + 2a4*t + (a3+a6)  [degree 2]
                            // Qw = Q*w = (a5-a1)t虏 + 2a2*t + (a1+a5)  [degree 2]
                            // dPw = dP*(w虏/2) = -a4*t虏 + 2*(a6-a3)*t + a4  (derivative via chain rule)
                            // dQw = dQ*(w虏/2) = -a2*t虏 + 2*(a5-a1)*t + a2
                            // Bw = B*w = -a8*t虏 + 2*(-a7)*t + a8  鈫?actually: 
                            //   c_j = (1-t虏)/(1+t虏), s_j = 2t/(1+t虏)
                            //   B = -a7*s + a8*c 鈫?B*w = -2a7*t + a8*(1-t虏) = -a8*t虏 - 2a7*t + a8
                            // F_scaled = (Qw*dQw + Pw*dPw)虏 - Bw虏*(Pw虏 + Qw虏)
                            // All terms are polynomials in t. F_scaled has degree 8.
                            double Pw[3] = {a3+a6, 2*a4, a6-a3};
                            double Qw[3] = {a1+a5, 2*a2, a5-a1};
                            double Bw[3] = {a8, -2*a7, -a8};
                            // dP/dq_j in half-angle: P'(q)*dq/dt*w虏
                            // d/dq(a3*cj+a4*sj+a6) = -a3*sj+a4*cj
                            // In half-angle: (-a3*s+a4*c)*w = a4(1-t虏)-a3*2t = a4 - 2a3*t - a4*t虏
                            // 鈫?dPw_half = {a4, -2*a3, -a4}  (this is (-a3*s+a4*c)*w)
                            // Similarly: dQw_half = {a2, -2*a1, -a2}
                            double dPw[3] = {a4, -2*a3, -a4};
                            double dQw[3] = {a2, -2*a1, -a2};

                            // QdQ = Qw * dQw [degree 4]
                            double QdQ[5] = {0,0,0,0,0};
                            for (int i2=0;i2<3;++i2) for (int j2=0;j2<3;++j2) QdQ[i2+j2] += Qw[i2]*dQw[j2];
                            // PdP = Pw * dPw [degree 4]
                            double PdP[5] = {0,0,0,0,0};
                            for (int i2=0;i2<3;++i2) for (int j2=0;j2<3;++j2) PdP[i2+j2] += Pw[i2]*dPw[j2];
                            // sum = QdQ + PdP [degree 4]
                            double S[5];
                            for (int i2=0;i2<5;++i2) S[i2] = QdQ[i2] + PdP[i2];
                            // S虏 [degree 8]
                            double S2[9] = {0,0,0,0,0,0,0,0,0};
                            for (int i2=0;i2<5;++i2) for (int j2=0;j2<5;++j2) S2[i2+j2] += S[i2]*S[j2];

                            // Bw虏 [degree 4]
                            double B2[5] = {0,0,0,0,0};
                            for (int i2=0;i2<3;++i2) for (int j2=0;j2<3;++j2) B2[i2+j2] += Bw[i2]*Bw[j2];
                            // Pw虏 [degree 4]
                            double P2[5] = {0,0,0,0,0};
                            for (int i2=0;i2<3;++i2) for (int j2=0;j2<3;++j2) P2[i2+j2] += Pw[i2]*Pw[j2];
                            // Qw虏 [degree 4]
                            double Q2[5] = {0,0,0,0,0};
                            for (int i2=0;i2<3;++i2) for (int j2=0;j2<3;++j2) Q2[i2+j2] += Qw[i2]*Qw[j2];
                            // PQ2 = P虏 + Q虏 [degree 4]
                            double PQ2[5];
                            for (int i2=0;i2<5;++i2) PQ2[i2] = P2[i2] + Q2[i2];
                            // B2_PQ2 = B虏 * (P虏+Q虏) [degree 8]
                            double B2PQ2[9] = {0,0,0,0,0,0,0,0,0};
                            for (int i2=0;i2<5;++i2) for (int j2=0;j2<5;++j2) B2PQ2[i2+j2] += B2[i2]*PQ2[j2];

                            // F = S^2 - B^2*(P^2+Q^2) [degree 8]
                            double poly_arr[9];
                            for (int i2=0;i2<9;++i2) poly_arr[i2] = S2[i2] - B2PQ2[i2];

                            // #7: Interval-restricted root finding
                            FixedRoots tj_roots;
                            solve_poly_in_interval(poly_arr, 8, tj_lo_pre, tj_hi_pre, tj_roots);

                            for (double tj_root : tj_roots) {
                                double qj_cand;
                                if (!half_angle_to_q(tj_root, lo_j, hi_j, qj_cand))
                                    continue;

                                double tj2 = tj_root*tj_root;
                                double denom_j = 1.0 + tj2;
                                double cj_v = (1.0 - tj2) / denom_j;
                                double sj_v = 2.0 * tj_root / denom_j;

                                double P = a3*cj_v + a4*sj_v + a6;
                                double Q_v = a1*cj_v + a2*sj_v + a5;
                                double qi_cand = std::atan2(P, Q_v);

                                double qi_tries[3] = {qi_cand, qi_cand+M_PI, qi_cand-M_PI};
                                for (double qi_try : qi_tries) {
                                    for (double shift : kShifts) {
                                        double qi_test = qi_try + shift;
                                        if (qi_test >= lo_i - 1e-10 && qi_test <= hi_i + 1e-10) {
                                            qi_test = std::max(lo_i, std::min(hi_i, qi_test));
                                            q[pi] = qi_test;
                                            q[pj] = qj_cand;
                                            eval_config(q);
                                            ++stats.n_phase_d_faces;
                                            ++stats.n_fk_calls;
                                            ++stats.n_phase_d_fk;
                                            goto next_d_root;
                                        }
                                    }
                                }
                                next_d_root:;
                            }
                        }
                    } // for eval_idx
                } // for bg
            } // for face_pairs
        } // for ci
    }

    stats.phase_d_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - _tD).count();

    auto _tE = std::chrono::high_resolution_clock::now();

    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
    //  Phase E: Pair-Constrained 1D  (qi+qj = k蟺/2, atan2 solve)
    //  with AA pruning
    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
    //
    //  On the manifold qi+qj = C = k蟺/2, substitute qj = C 鈭?qi.
    //  All other joints on boundary 鈫?1-DOF atan2 solve in qi.
    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲

    {
        // Build pair list: coupled + adjacent + skip-1 (same as AnalyticalCrit)
        std::vector<std::pair<int,int>> pc_pairs;
        {
            auto cp = robot.coupled_pairs();
            pc_pairs.insert(pc_pairs.end(), cp.begin(), cp.end());
            for (int i = 0; i < n - 1; ++i) {
                std::pair<int,int> p = {i, i+1};
                bool dup = false;
                for (auto& fp : pc_pairs) if (fp == p) { dup = true; break; }
                if (!dup) pc_pairs.push_back(p);
            }
            for (int i = 0; i < n; ++i) {
                for (int j2 = i + 2; j2 < n; ++j2) {
                    std::pair<int,int> p = {i, j2};
                    bool dup = false;
                    for (auto& fp : pc_pairs) if (fp == p) { dup = true; break; }
                    if (!dup) pc_pairs.push_back(p);
                }
            }
        }

        // Pre-compute per-joint background values
        std::vector<std::vector<double>> bg_vals(n);
        for (int j = 0; j < n; ++j)
            bg_vals[j] = build_bg_values(intervals[j].lo, intervals[j].hi, true);

        constexpr int PAIR_MAX_BG = 16;   // reduced: cache provides coverage

        for (int ci = 0; ci < n_act; ++ci) {
            int V = map[ci];
            int nj = std::min(V + 1, n);
            bool has_prox = (V >= 1);

            // Level 1: full-box AA check
            ++stats.n_aa_prune_checks;
            if (!aa_link_can_improve(aa_fk_ws.prefix, ci)) {
                ++stats.n_aa_pruned;
                continue;
            }

            for (auto& [pi, pj] : pc_pairs) {
                if (pi >= nj || pj >= nj) continue;
                double lo_i = intervals[pi].lo, hi_i = intervals[pi].hi;
                double lo_j2 = intervals[pj].lo, hi_j2 = intervals[pj].hi;
                if (hi_i - lo_i < 1e-12 || hi_j2 - lo_j2 < 1e-12) continue;

                // Enumerate constraint values C = k蟺/2 where qi+qj = C
                double sum_lo = lo_i + lo_j2, sum_hi = hi_i + hi_j2;
                int k_lo = static_cast<int>(std::ceil(sum_lo / HALF_PI - 1e-9));
                int k_hi = static_cast<int>(std::floor(sum_hi / HALF_PI + 1e-9));

                for (int kk = k_lo; kk <= k_hi; ++kk) {
                    double C = kk * HALF_PI;
                    double eff_lo = std::max(lo_i, C - hi_j2);
                    double eff_hi = std::min(hi_i, C - lo_j2);
                    if (eff_hi - eff_lo < 1e-12) continue;

                    // Build backgrounds for other joints
                    std::vector<std::vector<double>> other_bg;
                    std::vector<int> other_idx;
                    long long total_bg = 1;
                    for (int jj = 0; jj < nj; ++jj) {
                        if (jj == pi || jj == pj) continue;
                        other_idx.push_back(jj);
                        other_bg.push_back(bg_vals[jj]);
                        total_bg *= static_cast<long long>(bg_vals[jj].size());
                        if (total_bg > PAIR_MAX_BG) break;
                    }
                    if (total_bg > PAIR_MAX_BG) {
                        other_bg.clear(); other_idx.clear(); total_bg = 1;
                        for (int jj = 0; jj < nj; ++jj) {
                            if (jj == pi || jj == pj) continue;
                            other_idx.push_back(jj);
                            other_bg.push_back({intervals[jj].lo, intervals[jj].hi});
                            total_bg *= 2;
                            if (total_bg > PAIR_MAX_BG) { total_bg = PAIR_MAX_BG; break; }
                        }
                    }

                    int n_other = static_cast<int>(other_idx.size());
                    double eff_mid = 0.5 * (eff_lo + eff_hi);
                    double qvals[3] = { eff_lo, eff_mid, eff_hi };

                    Eigen::Matrix3d A_trig;
                    for (int si = 0; si < 3; ++si) {
                        A_trig(si, 0) = std::cos(qvals[si]);
                        A_trig(si, 1) = std::sin(qvals[si]);
                        A_trig(si, 2) = 1.0;
                    }
                    auto qr_e = A_trig.colPivHouseholderQr();

                    // Mixed-radix enumeration of backgrounds
                    std::vector<int> bg_counter(n_other, 0);
                    int min_free_e = std::min(pi, pj);
                    for (long long bgc = 0; bgc < total_bg; ++bgc) {
                        for (int jj = 0; jj < n; ++jj) q[jj] = intervals[jj].mid();
                        for (int oi = 0; oi < n_other; ++oi)
                            q[other_idx[oi]] = other_bg[oi][bg_counter[oi]];

                        // Partial FK: compute prefix once per background
                        ws.compute_prefix(robot, q, min_free_e);

                        // Evaluate FK at 3 sample points on constraint manifold
                        Eigen::Vector3d pts_dist[3], pts_prox[3];
                        bool ok = true;
                        for (int si = 0; si < 3; ++si) {
                            q[pi] = qvals[si];
                            q[pj] = C - qvals[si];
                            ws.compute_from(robot, q, min_free_e);
                            ++stats.n_fk_calls;
                            ++stats.n_phase_e_fk;
                            if (V + 1 >= ws.np) { ok = false; break; }
                            pts_dist[si] = ws.pos(V + 1);
                            if (has_prox) pts_prox[si] = ws.pos(V);
                        }
                        if (!ok) goto next_bg_e;

                        // Process distal + proximal
                        for (int eval_idx = 0; eval_idx < 2; ++eval_idx) {
                            if (eval_idx == 1 && !has_prox) break;
                            const Eigen::Vector3d* pts = (eval_idx == 0) ? pts_dist : pts_prox;

                            Eigen::Vector3d b_vec, coeff;
                            for (int d = 0; d < 3; ++d) {
                                b_vec << pts[0][d], pts[1][d], pts[2][d];
                                coeff = qr_e.solve(b_vec);
                                double ac = coeff[0], bc = coeff[1];
                                if (std::abs(ac) < 1e-15 && std::abs(bc) < 1e-15) continue;

                                double q_star = std::atan2(bc, ac);
                                double cands[2] = { q_star, q_star + M_PI };
                                if (cands[1] > M_PI + 0.1) cands[1] -= 2 * M_PI;

                                for (double qc : cands) {
                                    for (double shift : kShifts) {
                                        double qi_test = qc + shift;
                                        if (qi_test >= eff_lo - 1e-10 && qi_test <= eff_hi + 1e-10) {
                                            qi_test = std::max(eff_lo, std::min(eff_hi, qi_test));
                                            double qj_test = C - qi_test;
                                            if (qj_test < lo_j2 - 1e-10 || qj_test > hi_j2 + 1e-10)
                                                continue;
                                            qj_test = std::max(lo_j2, std::min(hi_j2, qj_test));
                                            q[pi] = qi_test;
                                            q[pj] = qj_test;
                                            eval_config(q);
                                            ++stats.n_phase_e_pair1d;
                                            ++stats.n_fk_calls;
                                            ++stats.n_phase_e_fk;
                                            break;
                                        }
                                    }
                                }
                            }
                        }
                        next_bg_e:;

                        // Increment mixed-radix counter
                        for (int oi = n_other - 1; oi >= 0; --oi) {
                            if (++bg_counter[oi] < static_cast<int>(other_bg[oi].size())) break;
                            bg_counter[oi] = 0;
                        }
                    }
                }
            }
        }
    }

    stats.phase_e_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - _tE).count();

    auto _tF = std::chrono::high_resolution_clock::now();

    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
    //  Phase F: Pair-Constrained 2D  (qi+qj = k蟺/2 + free joint m)
    //  Companion-matrix solve with AA pruning
    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
    //
    //  On manifold qi+qj = C = k蟺/2, substitute qj = C 鈭?qi.
    //  With one additional free joint qm, fit 9-coeff bilinear-trig
    //  model in (qi, qm) and solve degree-8 polynomial.
    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲

    {
        // Reuse the same pair list from Phase E
        std::vector<std::pair<int,int>> pc_pairs_f;
        {
            auto cp = robot.coupled_pairs();
            pc_pairs_f.insert(pc_pairs_f.end(), cp.begin(), cp.end());
            for (int i = 0; i < n - 1; ++i) {
                std::pair<int,int> p = {i, i+1};
                bool dup = false;
                for (auto& fp : pc_pairs_f) if (fp == p) { dup = true; break; }
                if (!dup) pc_pairs_f.push_back(p);
            }
            for (int i = 0; i < n; ++i) {
                for (int j2 = i + 2; j2 < n; ++j2) {
                    std::pair<int,int> p = {i, j2};
                    bool dup = false;
                    for (auto& fp : pc_pairs_f) if (fp == p) { dup = true; break; }
                    if (!dup) pc_pairs_f.push_back(p);
                }
            }
        }

        std::vector<std::vector<double>> bg_vals_f(n);
        for (int j = 0; j < n; ++j)
            bg_vals_f[j] = build_bg_values(intervals[j].lo, intervals[j].hi, true);

        constexpr int PAIR2D_MAX_BG = 16;   // reduced: cache provides coverage

        for (int ci = 0; ci < n_act; ++ci) {
            int V = map[ci];
            int nj = std::min(V + 1, n);

            // Level 1: full-box AA check
            ++stats.n_aa_prune_checks;
            if (!aa_link_can_improve(aa_fk_ws.prefix, ci)) {
                ++stats.n_aa_pruned;
                continue;
            }

            for (auto& [pi, pj] : pc_pairs_f) {
                if (pi >= nj || pj >= nj) continue;
                double lo_i = intervals[pi].lo, hi_i = intervals[pi].hi;
                double lo_j2 = intervals[pj].lo, hi_j2 = intervals[pj].hi;
                if (hi_i - lo_i < 1e-12 || hi_j2 - lo_j2 < 1e-12) continue;

                double sum_lo = lo_i + lo_j2, sum_hi = hi_i + hi_j2;
                int k_lo = static_cast<int>(std::ceil(sum_lo / HALF_PI - 1e-9));
                int k_hi = static_cast<int>(std::floor(sum_hi / HALF_PI + 1e-9));

                for (int kk = k_lo; kk <= k_hi; ++kk) {
                    double C = kk * HALF_PI;
                    double eff_lo = std::max(lo_i, C - hi_j2);
                    double eff_hi = std::min(hi_i, C - lo_j2);
                    if (eff_hi - eff_lo < 1e-12) continue;

                    // For each additional free joint m
                    for (int pm = 0; pm < nj; ++pm) {
                        if (pm == pi || pm == pj) continue;
                        double lo_m = intervals[pm].lo, hi_m = intervals[pm].hi;
                        if (hi_m - lo_m < 1e-12) continue;

                        // Build backgrounds for remaining joints
                        std::vector<int> bg_idx;
                        std::vector<std::vector<double>> other_bg;
                        long long total_bg = 1;
                        for (int jj = 0; jj < nj; ++jj) {
                            if (jj == pi || jj == pj || jj == pm) continue;
                            bg_idx.push_back(jj);
                            other_bg.push_back(bg_vals_f[jj]);
                            total_bg *= static_cast<long long>(bg_vals_f[jj].size());
                            if (total_bg > PAIR2D_MAX_BG) break;
                        }
                        if (total_bg > PAIR2D_MAX_BG) {
                            other_bg.clear(); bg_idx.clear(); total_bg = 1;
                            for (int jj = 0; jj < nj; ++jj) {
                                if (jj == pi || jj == pj || jj == pm) continue;
                                bg_idx.push_back(jj);
                                other_bg.push_back({intervals[jj].lo, intervals[jj].hi});
                                total_bg *= 2;
                                if (total_bg > PAIR2D_MAX_BG) { total_bg = PAIR2D_MAX_BG; break; }
                            }
                        }

                        int n_bg_f = static_cast<int>(bg_idx.size());

                        // 3脳3 sample grid: (qi, qm)
                        double qi_vals[3] = { eff_lo, 0.5*(eff_lo+eff_hi), eff_hi };
                        double qm_vals[3] = { lo_m, 0.5*(lo_m+hi_m), hi_m };

                        // Pre-compute 9脳9 trig basis
                        Eigen::Matrix<double, 9, 9> A_mat_f;
                        {
                            int row = 0;
                            for (int si = 0; si < 3; ++si) {
                                double ci_v = std::cos(qi_vals[si]);
                                double si_v = std::sin(qi_vals[si]);
                                for (int sm = 0; sm < 3; ++sm) {
                                    double cm_v = std::cos(qm_vals[sm]);
                                    double sm_v = std::sin(qm_vals[sm]);
                                    A_mat_f(row,0) = ci_v*cm_v; A_mat_f(row,1) = ci_v*sm_v;
                                    A_mat_f(row,2) = si_v*cm_v; A_mat_f(row,3) = si_v*sm_v;
                                    A_mat_f(row,4) = ci_v;       A_mat_f(row,5) = si_v;
                                    A_mat_f(row,6) = cm_v;       A_mat_f(row,7) = sm_v;
                                    A_mat_f(row,8) = 1.0;
                                    ++row;
                                }
                            }
                        }
                        auto qr_f = A_mat_f.colPivHouseholderQr();

                        std::vector<int> bg_cnt(n_bg_f, 0);
                        int min_free_f = std::min({pi, pj, pm});

                        // Pre-allocate interval array for AA pruning (stack, no heap)
                        bool do_aa_f = (total_bg >= 4);
                        Interval face_f_iv_s[AAFKWorkspace::MAX_TF];
                        if (do_aa_f) {
                            for (int jj = 0; jj < n; ++jj) face_f_iv_s[jj] = intervals[jj];
                            face_f_iv_s[pi] = intervals[pi];
                            face_f_iv_s[pj] = intervals[pj];
                            face_f_iv_s[pm] = intervals[pm];
                        }

                        AAFKWorkspace aa_face_f_ws;
                        int max_aa_frame_f = V + 1;   // early termination
                        int earliest_changed_f = -1;   // -1 = full recompute

                        for (long long bgc = 0; bgc < total_bg; ++bgc) {
                            for (int jj = 0; jj < n; ++jj) q[jj] = intervals[jj].mid();
                            for (int oi = 0; oi < n_bg_f; ++oi)
                                q[bg_idx[oi]] = other_bg[oi][bg_cnt[oi]];

                            // Level 2: per-background AA pruning (incremental + early termination)
                            if (do_aa_f) {
                                for (int oi = 0; oi < n_bg_f; ++oi) {
                                    double bv = other_bg[oi][bg_cnt[oi]];
                                    face_f_iv_s[bg_idx[oi]] = Interval{bv, bv};
                                }
                                if (earliest_changed_f < 0) {
                                    aa_face_f_ws.compute(robot, face_f_iv_s, max_aa_frame_f);
                                } else {
                                    aa_face_f_ws.recompute_from(robot, face_f_iv_s,
                                                                earliest_changed_f,
                                                                max_aa_frame_f);
                                }
                                ++stats.n_aa_prune_checks;
                                if (!aa_link_can_improve(aa_face_f_ws.prefix, ci)) {
                                    ++stats.n_phase_f_pruned;
                                    ++stats.n_aa_pruned;
                                    goto next_bg_f;
                                }
                                ++stats.n_aa_not_pruned;
                            }

                            // Partial FK: compute prefix once per background
                            ws.compute_prefix(robot, q, min_free_f);

                            // Evaluate 9 FK samples
                            Eigen::Vector3d pos_V_c[9], pos_V1_c[9];
                            bool valid = true;
                            {
                                int row = 0;
                                for (int si = 0; si < 3 && valid; ++si) {
                                    q[pi] = qi_vals[si];
                                    q[pj] = C - qi_vals[si];
                                    for (int sm = 0; sm < 3 && valid; ++sm) {
                                        q[pm] = qm_vals[sm];
                                        ws.compute_from(robot, q, min_free_f);
                                        ++stats.n_fk_calls;
                                        ++stats.n_phase_f_fk;
                                        if (V + 1 >= ws.np) { valid = false; break; }
                                        pos_V_c[row] = ws.pos(V);
                                        pos_V1_c[row] = ws.pos(V + 1);
                                        ++row;
                                    }
                                }
                            }
                            if (!valid) goto next_bg_f;

                            // Pre-compute tan half-angle bounds (constant for this pm)
                            // Hoisted from inner dim loop
                            double tm_lo_pre = std::tan(lo_m * 0.5);
                            double tm_hi_pre = std::tan(hi_m * 0.5);
                            if (tm_lo_pre > tm_hi_pre) std::swap(tm_lo_pre, tm_hi_pre);

                            // Process both distal and proximal
                            for (int eval_idx = 0; eval_idx < 2; ++eval_idx) {
                                int eval_link = (eval_idx == 0) ? V + 1 : V;
                                if (eval_link < 1) continue;
                                const Eigen::Vector3d* pc =
                                    (eval_idx == 0) ? pos_V1_c : pos_V_c;

                                // Batch QR solve for all 3 dimensions at once
                                Eigen::Matrix<double, 9, 3> rhs_3;
                                for (int r = 0; r < 9; ++r)
                                    for (int dd = 0; dd < 3; ++dd)
                                        rhs_3(r, dd) = pc[r][dd];
                                Eigen::Matrix<double, 9, 3> coeff_3 = qr_f.solve(rhs_3);

                                for (int d = 0; d < 3; ++d) {
                                    // #5: Per-direction AA pruning
                                    if (do_aa_f && !aa_dim_can_improve(aa_fk_ws.prefix, ci, d))
                                        continue;

                                    double a1=coeff_3(0,d), a2=coeff_3(1,d), a3=coeff_3(2,d), a4=coeff_3(3,d);
                                    double a5=coeff_3(4,d), a6=coeff_3(5,d), a7=coeff_3(6,d), a8=coeff_3(7,d);

                                    // #9: Symbolic polynomial coefficients
                                    double Pw[3] = {a3+a6, 2*a4, a6-a3};
                                    double Qw[3] = {a1+a5, 2*a2, a5-a1};
                                    double Bw[3] = {a8, -2*a7, -a8};
                                    double dPw[3] = {a4, -2*a3, -a4};
                                    double dQw[3] = {a2, -2*a1, -a2};

                                    double QdQ[5]={0,0,0,0,0}, PdP[5]={0,0,0,0,0};
                                    for (int i2=0;i2<3;++i2) for (int j2=0;j2<3;++j2) {
                                        QdQ[i2+j2] += Qw[i2]*dQw[j2];
                                        PdP[i2+j2] += Pw[i2]*dPw[j2];
                                    }
                                    double S[5];
                                    for (int i2=0;i2<5;++i2) S[i2] = QdQ[i2] + PdP[i2];
                                    double S2[9]={0,0,0,0,0,0,0,0,0};
                                    for (int i2=0;i2<5;++i2) for (int j2=0;j2<5;++j2) S2[i2+j2] += S[i2]*S[j2];

                                    double B2[5]={0,0,0,0,0}, P2[5]={0,0,0,0,0}, Q2[5]={0,0,0,0,0};
                                    for (int i2=0;i2<3;++i2) for (int j2=0;j2<3;++j2) {
                                        B2[i2+j2] += Bw[i2]*Bw[j2];
                                        P2[i2+j2] += Pw[i2]*Pw[j2];
                                        Q2[i2+j2] += Qw[i2]*Qw[j2];
                                    }
                                    double PQ2[5];
                                    for (int i2=0;i2<5;++i2) PQ2[i2] = P2[i2] + Q2[i2];
                                    double B2PQ2[9]={0,0,0,0,0,0,0,0,0};
                                    for (int i2=0;i2<5;++i2) for (int j2=0;j2<5;++j2) B2PQ2[i2+j2] += B2[i2]*PQ2[j2];

                                    double pa[9];
                                    for (int i2=0;i2<9;++i2) pa[i2] = S2[i2] - B2PQ2[i2];

                                    // #7: Interval-restricted root finding
                                    FixedRoots tm_roots;
                                    solve_poly_in_interval(pa, 8, tm_lo_pre, tm_hi_pre, tm_roots);

                                    for (double tm_r : tm_roots) {
                                        double qm_cand;
                                        if (!half_angle_to_q(tm_r, lo_m, hi_m, qm_cand))
                                            continue;

                                        double tm2 = tm_r*tm_r;
                                        double den_m = 1.0 + tm2;
                                        double cm_v = (1.0 - tm2)/den_m;
                                        double sm_v = 2.0*tm_r/den_m;

                                        double P = a3*cm_v + a4*sm_v + a6;
                                        double Q_v = a1*cm_v + a2*sm_v + a5;
                                        double qi_cand = std::atan2(P, Q_v);

                                        double qi_tries[3] = {qi_cand, qi_cand+M_PI, qi_cand-M_PI};
                                        for (double qi_try : qi_tries) {
                                            for (double shift : kShifts) {
                                                double qi_test = qi_try + shift;
                                                if (qi_test >= eff_lo - 1e-10 &&
                                                    qi_test <= eff_hi + 1e-10) {
                                                    qi_test = std::max(eff_lo, std::min(eff_hi, qi_test));
                                                    double qj_test = C - qi_test;
                                                    if (qj_test < lo_j2 - 1e-10 || qj_test > hi_j2 + 1e-10)
                                                        continue;
                                                    qj_test = std::max(lo_j2, std::min(hi_j2, qj_test));
                                                    q[pi] = qi_test;
                                                    q[pj] = qj_test;
                                                    q[pm] = qm_cand;
                                                    eval_config(q);
                                                    ++stats.n_phase_f_pair2d;
                                                    ++stats.n_fk_calls;
                                                    ++stats.n_phase_f_fk;
                                                    goto next_f_root;
                                                }
                                            }
                                        }
                                        next_f_root:;
                                    }
                                }
                            } // for eval_idx
                            next_bg_f:;

                            // Increment mixed-radix counter and track earliest change
                            earliest_changed_f = (n_bg_f > 0) ? bg_idx[0] : 0;
                            for (int oi = n_bg_f - 1; oi >= 0; --oi) {
                                earliest_changed_f = bg_idx[oi];
                                if (++bg_cnt[oi] < static_cast<int>(other_bg[oi].size())) break;
                                bg_cnt[oi] = 0;
                            }
                        }
                    } // for pm
                } // for kk
            } // for pc_pairs_f
        } // for ci
    }

    stats.phase_f_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - _tF).count();

    auto _tG = std::chrono::high_resolution_clock::now();

    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
    //  Phase G: Interior coordinate descent (query-local, per-face)
    //
    //  Mirrors AnalyticalCritical Phase 3 (improved + dual):
    //   - Per (link, face): multi-start coordinate-wise atan2 sweep
    //   - Seed 0: current best config from Phases A-F (or checkpoint_AB)
    //   - Seeds 1+: pair-constrained perturbations + k蟺/2 midpoint
    //   - Dual mode: run twice 鈥?once from Phase A-F state, once from
    //     Phase A+B checkpoint (avoids basin shift from Phase C-F roots)
    //   - Pair coupling: after each joint update, adjust partner joint
    //     to maintain qi+qj 鈮?nearest k蟺/2
    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲

    {
        const int max_g_sweeps = 3;   // match Analytical interior_max_sweeps

        // Build priority pairs (coupled + adjacent + skip-1)
        auto coupled_g = robot.coupled_pairs();
        std::vector<std::pair<int,int>> all_pairs_g = coupled_g;
        for (int i = 0; i < n - 1; ++i) {
            std::pair<int,int> p = {i, i + 1};
            bool dup = false;
            for (auto& fp : all_pairs_g) if (fp == p) { dup = true; break; }
            if (!dup) all_pairs_g.push_back(p);
        }
        for (int i = 0; i < n - 2; i += 2) {
            std::pair<int,int> p = {i, i + 2};
            bool dup = false;
            for (auto& fp : all_pairs_g) if (fp == p) { dup = true; break; }
            if (!dup) all_pairs_g.push_back(p);
        }

        // Lambda: run one round of Phase G from a given set of face configs
        auto run_phase_g_round = [&](
            const std::vector<Eigen::VectorXd>& face_configs)
        {
            for (int ci = 0; ci < n_act; ++ci) {
                int V = map[ci];
                int nj = std::min(V + 1, n);
                if (nj < 3) continue;

                bool has_prox = (V >= 1);

                // Level 1 AA check
                if (!aa_link_can_improve(aa_fk_ws.prefix, ci)) {
                    ++stats.n_phase_g_pruned;
                    continue;
                }

                // Link-specific pairs
                std::vector<std::pair<int,int>> link_pairs;
                for (auto& [a, b] : all_pairs_g) {
                    if (a < nj && b < nj) link_pairs.push_back({a, b});
                }

                for (int face = 0; face < 6; ++face) {
                    int dim = face / 2;
                    bool is_min = (face % 2 == 0);

                    // 鈹€鈹€ Collect starting configs 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
                    std::vector<Eigen::VectorXd> starts;

                    // Seed 0: face-config from the given checkpoint
                    {
                        const auto& q_fc = face_configs[ci * 6 + face];
                        if (q_fc.size() == n) starts.push_back(q_fc);
                    }

                    // Seed 1: k蟺/2 midpoint
                    {
                        Eigen::VectorXd q_kpi2(n);
                        for (int j = 0; j < n; ++j) {
                            double mid = 0.5 * (intervals[j].lo + intervals[j].hi);
                            double bestv = mid;
                            double best_dist = 1e30;
                            for (int kk = -20; kk <= 20; ++kk) {
                                double a = kk * HALF_PI;
                                if (a >= intervals[j].lo && a <= intervals[j].hi) {
                                    double dist = std::abs(a - mid);
                                    if (dist < best_dist) {
                                        best_dist = dist;
                                        bestv = a;
                                    }
                                }
                            }
                            q_kpi2[j] = bestv;
                        }
                        starts.push_back(q_kpi2);
                    }

                    // Seeds 2+: pair-constrained perturbations (max 4, match Analytical)
                    for (int ri = 0; ri < std::min((int)link_pairs.size(), 4); ++ri) {
                        auto [pa, pb] = link_pairs[ri % link_pairs.size()];
                        Eigen::VectorXd q_seed = starts[0];
                        double sum_ab = q_seed[pa] + q_seed[pb];
                        double nearest_k =
                            std::round(sum_ab / HALF_PI) * HALF_PI;
                        double delta = (nearest_k - sum_ab) * 0.5;
                        q_seed[pa] = std::max(intervals[pa].lo,
                            std::min(intervals[pa].hi, q_seed[pa] + delta));
                        q_seed[pb] = std::max(intervals[pb].lo,
                            std::min(intervals[pb].hi, q_seed[pb] + delta));
                        starts.push_back(q_seed);
                    }

                    // 鈹€鈹€ Run coordinate descent from each seed 鈹€鈹€鈹€鈹€
                    for (auto& q_start : starts) {
                        Eigen::VectorXd q_cd = q_start;

                        for (int sweep = 0; sweep < max_g_sweeps; ++sweep) {
                            bool improved = false;

                            for (int j = 0; j < nj; ++j) {
                                double lo_j = intervals[j].lo;
                                double hi_j = intervals[j].hi;
                                if (hi_j - lo_j < 1e-12) continue;

                                double mid_j = 0.5 * (lo_j + hi_j);
                                double qvals[3] = { lo_j, mid_j, hi_j };

                                Eigen::Matrix3d A_trig;
                                for (int si = 0; si < 3; ++si) {
                                    A_trig(si, 0) = std::cos(qvals[si]);
                                    A_trig(si, 1) = std::sin(qvals[si]);
                                    A_trig(si, 2) = 1.0;
                                }
                                auto qr_g = A_trig.colPivHouseholderQr();

                                double q_save = q_cd[j];

                                Eigen::Vector3d pts_dist[3];
                                Eigen::Vector3d pts_prox[3];
                                bool pts_ok = true;

                                // Partial FK: prefix T[0..j] is valid for
                                // the current q_cd background, only recompute
                                // from joint j onward for each sample.
                                // Establish prefix once:
                                ws.tf[0] = Eigen::Matrix4d::Identity();
                                for (int jj = 0; jj < j; ++jj)
                                    ws.compute_joint(robot, q_cd, jj);

                                for (int si = 0; si < 3; ++si) {
                                    q_cd[j] = qvals[si];
                                    ws.compute_from(robot, q_cd, j);
                                    ++stats.n_fk_calls;
                                    ++stats.n_phase_g_fk;
                                    if (V + 1 >= ws.np) {
                                        pts_ok = false;
                                        break;
                                    }
                                    pts_dist[si] = ws.pos(V + 1);
                                    if (has_prox) pts_prox[si] = ws.pos(V);
                                }
                                q_cd[j] = q_save;
                                if (!pts_ok) continue;

                                // Distal and proximal endpoints
                                for (int eval_idx = 0; eval_idx < 2; ++eval_idx) {
                                    if (eval_idx == 1 && !has_prox) continue;
                                    const Eigen::Vector3d* pts =
                                        (eval_idx == 0) ? pts_dist : pts_prox;

                                    Eigen::Vector3d b_vec;
                                    b_vec << pts[0][dim], pts[1][dim],
                                             pts[2][dim];
                                    Eigen::Vector3d coeff = qr_g.solve(b_vec);
                                    double ac = coeff[0], bc = coeff[1];
                                    if (std::abs(ac) < 1e-15 &&
                                        std::abs(bc) < 1e-15) continue;

                                    double q_star = std::atan2(bc, ac);
                                    if (is_min) q_star += M_PI;

                                    // Primary candidate
                                    for (double shift : kShifts) {
                                        double qt = q_star + shift;
                                        if (qt >= lo_j - 1e-10 &&
                                            qt <= hi_j + 1e-10) {
                                            qt = std::max(lo_j,
                                                 std::min(hi_j, qt));
                                            q_cd[j] = qt;
                                            eval_config(q_cd);
                                            ++stats.n_phase_g_interior;
                                            improved = true;
                                            break;
                                        }
                                    }
                                    // Opposite candidate
                                    double q_star2 = q_star + M_PI;
                                    for (double shift : kShifts) {
                                        double qt = q_star2 + shift;
                                        if (qt >= lo_j - 1e-10 &&
                                            qt <= hi_j + 1e-10) {
                                            qt = std::max(lo_j,
                                                 std::min(hi_j, qt));
                                            q_cd[j] = qt;
                                            eval_config(q_cd);
                                            ++stats.n_phase_g_interior;
                                            break;
                                        }
                                    }
                                }

                                // Pair coupling
                                for (auto& [pa, pb] : link_pairs) {
                                    int partner = -1;
                                    if (pa == j) partner = pb;
                                    else if (pb == j) partner = pa;
                                    if (partner < 0 || partner >= nj) continue;

                                    double sum_cur = q_cd[j] + q_cd[partner];
                                    double nearest_c =
                                        std::round(sum_cur / HALF_PI) * HALF_PI;
                                    if (std::abs(sum_cur - nearest_c) > 0.3)
                                        continue;

                                    double new_partner = nearest_c - q_cd[j];
                                    if (new_partner >= intervals[partner].lo - 1e-10 &&
                                        new_partner <= intervals[partner].hi + 1e-10)
                                    {
                                        new_partner = std::max(
                                            intervals[partner].lo,
                                            std::min(intervals[partner].hi,
                                                     new_partner));
                                        q_cd[partner] = new_partner;
                                        eval_config(q_cd);
                                        ++stats.n_phase_g_interior;
                                        improved = true;
                                    }
                                }
                            }

                            if (!improved) break;
                        }
                    }  // for starts
                }  // for face
            }  // for ci
        };  // end run_phase_g_round lambda

        // 鈹€鈹€ Run 1: from Phase A-F final state 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
        run_phase_g_round(best_face_config);

        // 鈹€鈹€ Run 2: from Phase A+B checkpoint (dual, avoid basin shift)
        run_phase_g_round(checkpoint_AB);
    }

    stats.phase_g_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - _tG).count();

    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲
    //  Finalize: inflate by link radii
    // 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲

    for (int ci = 0; ci < n_act; ++ci) {
        float r = (rad) ? static_cast<float>(rad[ci]) : 0.f;
        for (int s = 0; s < n_sub; ++s) {
            float* a = out_aabb + (ci * n_sub + s) * 6;
            a[0] -= r;  a[1] -= r;  a[2] -= r;
            a[3] += r;  a[4] += r;  a[5] += r;
        }
        if (out_endpoint_aabb) {
            // NOTE: endpoint_aabb is NOT radius-inflated 鈥?fill_hull16 handles radius
            // separately for correct Minkowski expansion.
        }
    }

    if (out_stats) *out_stats = stats;
}


// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  Accessors
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

int GcpcCache::n_total_points() const {
    int total = 0;
    for (const auto& sec : sections_)
        total += sec.n_points;
    return total;
}

const GcpcLinkSection* GcpcCache::find_section(int link_id) const {
    for (const auto& sec : sections_) {
        if (sec.link_id == link_id) return &sec;
    }
    return nullptr;
}


// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  Build from points
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

void GcpcCache::build(const Robot& robot, const std::vector<GcpcPoint>& points) {
    robot_name_ = robot.name();
    d0_ = robot.dh_params()[0].d;
    has_tool_ = robot.has_tool();

    sections_.clear();

    std::map<int, std::vector<const GcpcPoint*>> by_link;
    for (const auto& pt : points)
        by_link[pt.link_id].push_back(&pt);

    int n_act = robot.n_active_links();
    const int* map = robot.active_link_map();

    for (int ci = 0; ci < n_act; ++ci) {
        int link_id = map[ci];
        GcpcLinkSection section;
        section.link_id = link_id;

        int nj = std::min(link_id + 1, robot.n_joints());
        section.n_eff_joints = nj - 1;

        section.q6_skipped = false;
        if (section.n_eff_joints >= 6 && link_id < robot.n_joints()) {
            const auto& dh6 = robot.dh_params()[6];
            if (std::abs(dh6.d) < 1e-10 && std::abs(dh6.a) < 1e-10) {
                section.n_eff_joints = 5;
                section.q6_skipped = true;
            }
        }

        auto it = by_link.find(link_id);
        if (it != by_link.end()) {
            for (const auto* pp : it->second) {
                section.points.push_back(*pp);
            }
        }
        section.n_points = static_cast<int>(section.points.size());

        build_kdtree(section);

        sections_.push_back(std::move(section));
    }
}


// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  Enrich cache with interior critical points (coordinate descent)
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//
//  For each active link, we seek the stationary points of:
//    R(q鈧?...,q鈧? = 鈭?A虏 + B虏)   (xy direction, after q鈧€ elimination)
//    pz(q鈧?...,q鈧? = C + d鈧€       (z direction, q鈧€ independent)
//
//  After q鈧€ elimination, the extrema of p_x (or p_y) over q鈧€ reduce to
//  extrema of R over q鈧?...,q鈧?(since max/min p_x = 卤R + 蔚). Similarly
//  pz is independent of q鈧€.
//
//  We run multi-start coordinate descent over the FULL joint range to find
//  all stationary points. At each step, we fix all joints except one and
//  analytically solve for the optimal in that dimension via atan2.
//
//  Points are stored in the q鈧佲垐[0,蟺] half-range using q鈧?symmetry:
//    R(q鈧? = R(q鈧?鈭?蟺)  (period-蟺 in q鈧?for radial distance)
//    pz(q鈧? = pz(q鈧?     (may need both halves for z)

int GcpcCache::enrich_with_interior_search(
    const Robot& robot,
    int n_random_seeds,
    int max_sweeps)
{
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* link_map = robot.active_link_map();
    auto limits = robot.joint_limits();

    int total_added = 0;

    // For deduplication: collect existing points' q_eff hashes per section
    auto q_eff_hash = [](const GcpcPoint& pt) -> uint64_t {
        uint64_t h = 0;
        for (int d = 0; d < pt.n_eff; ++d) {
            // Quantize to ~1e-6 resolution
            int64_t q = static_cast<int64_t>(pt.q_eff[d] * 1e6);
            h ^= static_cast<uint64_t>(q * 2654435761ULL + d * 40503ULL);
        }
        return h ^ (static_cast<uint64_t>(pt.direction) * 7919ULL);
    };

    // Gather k蟺/2 values per joint (within full joint limits)
    std::vector<std::vector<double>> kpi2_vals(n);
    for (int j = 0; j < n; ++j) {
        double lo = limits.limits[j].lo, hi = limits.limits[j].hi;
        for (int k = -20; k <= 20; ++k) {
            double a = k * HALF_PI;
            if (a >= lo - 1e-10 && a <= hi + 1e-10) {
                a = std::max(lo, std::min(hi, a));
                kpi2_vals[j].push_back(a);
            }
        }
        if (kpi2_vals[j].empty()) {
            kpi2_vals[j].push_back(0.5 * (lo + hi));
        }
        std::sort(kpi2_vals[j].begin(), kpi2_vals[j].end());
        kpi2_vals[j].erase(
            std::unique(kpi2_vals[j].begin(), kpi2_vals[j].end()),
            kpi2_vals[j].end());
    }

    // Build coupled pairs for pair-initialised seeds
    auto coupled = robot.coupled_pairs();
    std::vector<std::pair<int,int>> all_pairs = coupled;
    for (int i = 0; i < n - 1; ++i) {
        std::pair<int,int> p = {i, i+1};
        bool dup = false;
        for (auto& fp : all_pairs) if (fp == p) { dup = true; break; }
        if (!dup) all_pairs.push_back(p);
    }
    for (int i = 0; i < n - 2; i += 2) {
        std::pair<int,int> p = {i, i+2};
        bool dup = false;
        for (auto& fp : all_pairs) if (fp == p) { dup = true; break; }
        if (!dup) all_pairs.push_back(p);
    }

    std::mt19937 rng(42);

    for (int sec_idx = 0; sec_idx < static_cast<int>(sections_.size()); ++sec_idx) {
        auto& section = sections_[sec_idx];
        int link_id = section.link_id;
        int n_eff = section.n_eff_joints;
        if (n_eff < 2) continue;  // 1-DOF already fully covered by boundary solvers

        int nj = n_eff + 1;  // joints 0..n_eff, but q鈧€ is eliminated
        // Effective joints for coordinate descent: q鈧?..q_{n_eff}
        // (joint indices 1..n_eff in the robot)

        // Build existing point hash set for dedup
        std::unordered_set<uint64_t> existing_hashes;
        for (const auto& pt : section.points)
            existing_hashes.insert(q_eff_hash(pt));

        std::vector<GcpcPoint> new_points;

        // Helper: run coordinate descent from a seed, converge to stationary point
        auto coordinate_descent = [&](Eigen::VectorXd& q_full, int direction) {
            // direction: 0 = optimize R (xy), 1 = optimize pz (z)
            int target_dim = -1;  // -1 means optimize R for xy, or dim=2 for z
            
            for (int sweep = 0; sweep < max_sweeps; ++sweep) {
                bool changed = false;

                for (int eff_j = 0; eff_j < n_eff; ++eff_j) {
                    int j = eff_j + 1;  // actual joint index
                    double lo_j = limits.limits[j].lo;
                    double hi_j = limits.limits[j].hi;
                    if (hi_j - lo_j < 1e-12) continue;

                    // Sample 3 values to fit p_d(q_j) = 伪 cos(q_j) + 尾 sin(q_j) + 纬
                    double mid_j = 0.5 * (lo_j + hi_j);
                    double qvals[3] = { lo_j, mid_j, hi_j };

                    Eigen::Matrix3d A_trig;
                    for (int si = 0; si < 3; ++si) {
                        A_trig(si, 0) = std::cos(qvals[si]);
                        A_trig(si, 1) = std::sin(qvals[si]);
                        A_trig(si, 2) = 1.0;
                    }
                    auto qr_A = A_trig.colPivHouseholderQr();

                    if (direction == 0) {
                        // Optimize R = sqrt(px虏 + py虏) at q鈧€=0
                        // Actually optimize px and py separately; the maximum of R
                        // is where 鈭俁/鈭俼j = 0, which for fixed q鈧€=0 means
                        // 鈭俻x/鈭俼j 路 px + 鈭俻y/鈭俼j 路 py = 0.
                        // But since we iterate, we optimise each coordinate direction:
                        // max of px 鈫?q_j = atan2(尾_x, 伪_x)
                        // max of py 鈫?q_j = atan2(尾_y, 伪_y)
                        // We try both and keep which gives larger R.
                        
                        double q_save = q_full[j];
                        double best_R = -1;
                        double best_q = q_save;

                        for (int d = 0; d < 2; ++d) {  // x=0, y=1
                            Eigen::Vector3d vals;
                            for (int si = 0; si < 3; ++si) {
                                q_full[j] = qvals[si];
                                auto pos = fk_link_positions(robot, q_full);
                                if (link_id + 1 < static_cast<int>(pos.size()))
                                    vals[si] = pos[link_id + 1][d];
                                else
                                    vals[si] = 0;
                            }
                            q_full[j] = q_save;

                            Eigen::Vector3d coeff = qr_A.solve(vals);
                            double ac = coeff[0], bc = coeff[1];
                            if (std::abs(ac) < 1e-15 && std::abs(bc) < 1e-15) continue;

                            double q_star = std::atan2(bc, ac);
                            double cands[] = {q_star, q_star + M_PI, q_star - M_PI};
                            for (double qc : cands) {
                                for (double shift : kShifts) {
                                    double qt = qc + shift;
                                    if (qt >= lo_j - 1e-10 && qt <= hi_j + 1e-10) {
                                        qt = std::max(lo_j, std::min(hi_j, qt));
                                        q_full[j] = qt;
                                        auto pos = fk_link_positions(robot, q_full);
                                        if (link_id + 1 < static_cast<int>(pos.size())) {
                                            double px = pos[link_id + 1][0];
                                            double py = pos[link_id + 1][1];
                                            double R = std::sqrt(px*px + py*py);
                                            if (R > best_R) { best_R = R; best_q = qt; }
                                        }
                                        q_full[j] = q_save;
                                        break;
                                    }
                                }
                            }
                        }
                        
                        // Also try min R (for R_min critical point)
                        for (int d = 0; d < 2; ++d) {
                            Eigen::Vector3d vals;
                            for (int si = 0; si < 3; ++si) {
                                q_full[j] = qvals[si];
                                auto pos = fk_link_positions(robot, q_full);
                                if (link_id + 1 < static_cast<int>(pos.size()))
                                    vals[si] = pos[link_id + 1][d];
                                else
                                    vals[si] = 0;
                            }
                            q_full[j] = q_save;

                            Eigen::Vector3d coeff = qr_A.solve(vals);
                            double ac = coeff[0], bc = coeff[1];
                            if (std::abs(ac) < 1e-15 && std::abs(bc) < 1e-15) continue;

                            double q_star = std::atan2(bc, ac) + M_PI;
                            for (double shift : kShifts) {
                                double qt = q_star + shift;
                                if (qt >= lo_j - 1e-10 && qt <= hi_j + 1e-10) {
                                    qt = std::max(lo_j, std::min(hi_j, qt));
                                    if (std::abs(qt - best_q) > 1e-6) {
                                        changed = true;
                                    }
                                    break;
                                }
                            }
                        }

                        if (std::abs(best_q - q_save) > 1e-8) {
                            q_full[j] = best_q;
                            changed = true;
                        }

                    } else {
                        // direction == 1: optimize pz (max AND min)
                        Eigen::Vector3d vals;
                        double q_save = q_full[j];
                        for (int si = 0; si < 3; ++si) {
                            q_full[j] = qvals[si];
                            auto pos = fk_link_positions(robot, q_full);
                            if (link_id + 1 < static_cast<int>(pos.size()))
                                vals[si] = pos[link_id + 1][2];
                            else
                                vals[si] = 0;
                        }
                        q_full[j] = q_save;

                        Eigen::Vector3d coeff = qr_A.solve(vals);
                        double ac = coeff[0], bc = coeff[1];
                        if (std::abs(ac) < 1e-15 && std::abs(bc) < 1e-15) continue;

                        double q_star = std::atan2(bc, ac);
                        double cands[] = {q_star, q_star + M_PI};
                        bool found = false;
                        for (double qc : cands) {
                            for (double shift : kShifts) {
                                double qt = qc + shift;
                                if (qt >= lo_j - 1e-10 && qt <= hi_j + 1e-10) {
                                    qt = std::max(lo_j, std::min(hi_j, qt));
                                    if (std::abs(qt - q_save) > 1e-8) {
                                        q_full[j] = qt;
                                        changed = true;
                                    }
                                    found = true;
                                    break;
                                }
                            }
                            if (found) break;
                        }
                    }
                }

                if (!changed) break;
            }

            // Extract converged point 鈫?GcpcPoint
            // First apply q鈧?symmetry: if q鈧?< 0, reflect to q鈧?蟺
            double q1_val = q_full[1];
            bool reflected = false;
            if (q1_val < -1e-10) {
                q1_val += M_PI;
                reflected = true;
            } else if (q1_val > M_PI + 1e-10) {
                q1_val -= M_PI;
                reflected = true;
            }
            q1_val = std::max(0.0, std::min(M_PI, q1_val));

            GcpcPoint pt{};
            pt.link_id = link_id;
            pt.n_eff = n_eff;
            pt.q_eff[0] = q1_val;
            for (int d = 1; d < n_eff; ++d)
                pt.q_eff[d] = q_full[d + 1];
            pt.direction = direction;

            // Evaluate FK at q鈧€=0 (possibly with reflected q鈧?
            Eigen::VectorXd q_eval(n);
            q_eval.setZero();
            q_eval[1] = q1_val;
            for (int d = 1; d < n_eff && d + 1 < n; ++d)
                q_eval[d + 1] = q_full[d + 1];

            auto pos = fk_link_positions(robot, q_eval);
            if (link_id + 1 < static_cast<int>(pos.size())) {
                const auto& p = pos[link_id + 1];
                pt.A = p[0]; pt.B = p[1]; pt.C = p[2];
                pt.R = std::sqrt(pt.A*pt.A + pt.B*pt.B);

                // Dedup check
                uint64_t h = q_eff_hash(pt);
                if (existing_hashes.find(h) == existing_hashes.end()) {
                    existing_hashes.insert(h);
                    new_points.push_back(pt);
                }
            }
        };

        // 鈹€鈹€ Generate seeds 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€

        // Seed type 1: k蟺/2 grid seeds (most important critical configs)
        {
            // Build k蟺/2 combos for q鈧?..q_{n_eff}
            std::vector<std::vector<double>> eff_kpi2(n_eff);
            // q鈧? only [0, 蟺]
            for (double v : kpi2_vals[1]) {
                if (v >= -1e-10 && v <= M_PI + 1e-10)
                    eff_kpi2[0].push_back(std::max(0.0, std::min(M_PI, v)));
            }
            if (eff_kpi2[0].empty()) eff_kpi2[0].push_back(HALF_PI);
            for (int d = 1; d < n_eff; ++d)
                eff_kpi2[d] = kpi2_vals[d + 1];

            long long total = 1;
            for (int d = 0; d < n_eff; ++d) {
                total *= static_cast<long long>(eff_kpi2[d].size());
                if (total > 5000) break;
            }

            if (total <= 5000) {
                // Full enumeration of k蟺/2 seeds
                std::vector<double> config(n_eff);
                std::function<void(int)> gen_seeds;
                gen_seeds = [&](int d) {
                    if (d >= n_eff) {
                        Eigen::VectorXd q_seed(n);
                        q_seed.setZero();
                        for (int i = 0; i < n_eff && i + 1 < n; ++i)
                            q_seed[i + 1] = config[i];

                        // Run coordinate descent for both xy and z
                        for (int dir = 0; dir < 2; ++dir) {
                            Eigen::VectorXd q_cd = q_seed;
                            coordinate_descent(q_cd, dir);
                        }
                        return;
                    }
                    for (double v : eff_kpi2[d]) {
                        config[d] = v;
                        gen_seeds(d + 1);
                    }
                };
                gen_seeds(0);
            }
        }

        // Seed type 2: Random seeds
        for (int s = 0; s < n_random_seeds; ++s) {
            Eigen::VectorXd q_seed(n);
            q_seed.setZero();
            // q鈧?in [0, 蟺], others in full range
            q_seed[1] = std::uniform_real_distribution<double>(0, M_PI)(rng);
            for (int d = 1; d < n_eff; ++d) {
                int j = d + 1;
                q_seed[j] = std::uniform_real_distribution<double>(
                    limits.limits[j].lo, limits.limits[j].hi)(rng);
            }

            for (int dir = 0; dir < 2; ++dir) {
                Eigen::VectorXd q_cd = q_seed;
                coordinate_descent(q_cd, dir);
            }
        }

        // Seed type 3: Pair-constrained seeds (qi+qj 鈮?k蟺/2)
        for (auto& [pa, pb] : all_pairs) {
            if (pa == 0 || pb == 0) continue;  // q鈧€ is eliminated
            if (pa > n_eff || pb > n_eff) continue;

            for (int k = -10; k <= 10; ++k) {
                double C = k * HALF_PI;
                // Try splitting C evenly between pa and pb
                double qa_try = std::max(limits.limits[pa].lo,
                                 std::min(limits.limits[pa].hi, C * 0.5));
                double qb_try = std::max(limits.limits[pb].lo,
                                 std::min(limits.limits[pb].hi, C - qa_try));

                Eigen::VectorXd q_seed(n);
                q_seed.setZero();
                q_seed[1] = HALF_PI;  // start q鈧?at 蟺/2
                for (int d = 1; d < n_eff; ++d) {
                    int j = d + 1;
                    q_seed[j] = 0.5 * (limits.limits[j].lo + limits.limits[j].hi);
                }
                q_seed[pa] = qa_try;
                q_seed[pb] = qb_try;

                for (int dir = 0; dir < 2; ++dir) {
                    Eigen::VectorXd q_cd = q_seed;
                    coordinate_descent(q_cd, dir);
                }
            }
        }

        // 鈹€鈹€ Add new points to section 鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€鈹€
        if (!new_points.empty()) {
            for (auto& pt : new_points)
                section.points.push_back(pt);
            section.n_points = static_cast<int>(section.points.size());
            
            // Rebuild KD-tree with all points
            build_kdtree(section);
            
            total_added += static_cast<int>(new_points.size());
        }
    }

    return total_added;
}


// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  JSON loading 鈥?uses nlohmann/json
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

bool GcpcCache::load_json(const std::string& path, const Robot& robot) {
    // Read JSON file
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "GCPC: Cannot open " << path << std::endl;
        return false;
    }

    nlohmann::json doc;
    try {
        ifs >> doc;
    } catch (const std::exception& e) {
        std::cerr << "GCPC: JSON parse error: " << e.what() << std::endl;
        return false;
    }

    robot_name_ = robot.name();
    d0_ = robot.dh_params()[0].d;
    has_tool_ = robot.has_tool();

    std::vector<GcpcPoint> points;

    // Expect a "points" array at root level
    nlohmann::json arr;
    if (doc.contains("points") && doc["points"].is_array()) {
        arr = doc["points"];
    } else if (doc.is_array()) {
        arr = doc;
    } else {
        std::cerr << "GCPC: Expected 'points' array in JSON" << std::endl;
        return false;
    }

    for (const auto& obj : arr) {
        GcpcPoint pt{};

        // Parse q_eff
        if (obj.contains("q_eff") && obj["q_eff"].is_array()) {
            const auto& qa = obj["q_eff"];
            pt.n_eff = static_cast<int>(qa.size());
            for (int i = 0; i < pt.n_eff && i < 7; ++i)
                pt.q_eff[i] = qa[i].get<double>();
        }

        // Parse link_id
        if (obj.contains("link_id"))
            pt.link_id = obj["link_id"].get<int>();

        // Parse direction
        if (obj.contains("direction")) {
            if (obj["direction"].is_string()) {
                pt.direction = (obj["direction"].get<std::string>() == "xy") ? 0 : 1;
            } else {
                pt.direction = obj["direction"].get<int>();
            }
        }

        // Compute A, B, C from FK evaluation
        // (Julia script may store p_critical, but we need A, B, C decomposition)
        {
            Eigen::VectorXd q_full(robot.n_joints());
            for (int j = 0; j < robot.n_joints(); ++j)
                q_full[j] = 0.0;  // q鈧€ = 0

            for (int d = 0; d < pt.n_eff && d + 1 < robot.n_joints(); ++d)
                q_full[d + 1] = pt.q_eff[d];

            auto positions = fk_link_positions(robot, q_full);
            int np = static_cast<int>(positions.size());
            if (pt.link_id + 1 < np) {
                const auto& p = positions[pt.link_id + 1];
                pt.A = p[0];
                pt.B = p[1];
                pt.C = p[2] - d0_;
                pt.R = std::sqrt(pt.A * pt.A + pt.B * pt.B);
            }
        }

        points.push_back(pt);
    }

    std::cout << "GCPC: Loaded " << points.size()
              << " critical points from " << path << std::endl;

    build(robot, points);
    return true;
}


// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?
//  Binary save/load
// 鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺愨晲鈺?

bool GcpcCache::save(const std::string& path) const {
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs.is_open()) return false;

    char header[512] = {};
    std::memcpy(header, "GCPC", 4);
    uint32_t version = 2;  // v2: added n_kd_nodes per section
    std::memcpy(header + 4, &version, 4);

    uint32_t name_len = static_cast<uint32_t>(robot_name_.size());
    std::memcpy(header + 8, &name_len, 4);
    std::memcpy(header + 12, robot_name_.c_str(), std::min<size_t>(name_len, 63));

    std::memcpy(header + 76, &d0_, 8);
    uint8_t ht = has_tool_ ? 1 : 0;
    std::memcpy(header + 84, &ht, 1);

    uint32_t n_sec = static_cast<uint32_t>(sections_.size());
    std::memcpy(header + 85, &n_sec, 4);

    // Compute section offsets
    uint64_t offset = 512;
    size_t table_pos = 89;
    for (const auto& sec : sections_) {
        int32_t lid = sec.link_id;
        int32_t nej = sec.n_eff_joints;
        int32_t np = sec.n_points;
        int32_t nkd = static_cast<int32_t>(sec.kd_tree.size());
        uint8_t q6s = sec.q6_skipped ? 1 : 0;

        std::memcpy(header + table_pos, &lid, 4); table_pos += 4;
        std::memcpy(header + table_pos, &nej, 4); table_pos += 4;
        std::memcpy(header + table_pos, &np, 4);  table_pos += 4;
        std::memcpy(header + table_pos, &nkd, 4); table_pos += 4;
        std::memcpy(header + table_pos, &q6s, 1); table_pos += 1;
        std::memcpy(header + table_pos, &offset, 8); table_pos += 8;

        offset += sec.kd_tree.size() * sizeof(KdNode);
        offset += sec.n_points * sizeof(GcpcPoint);
    }

    ofs.write(header, 512);

    for (const auto& sec : sections_) {
        if (!sec.kd_tree.empty())
            ofs.write(reinterpret_cast<const char*>(sec.kd_tree.data()),
                     sec.kd_tree.size() * sizeof(KdNode));
        if (!sec.points.empty())
            ofs.write(reinterpret_cast<const char*>(sec.points.data()),
                     sec.n_points * sizeof(GcpcPoint));
    }

    return ofs.good();
}

bool GcpcCache::load(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) return false;

    char header[512];
    ifs.read(header, 512);
    if (!ifs.good()) return false;

    if (std::memcmp(header, "GCPC", 4) != 0) return false;

    uint32_t version;
    std::memcpy(&version, header + 4, 4);
    if (version != 2) return false;  // v2: with n_kd_nodes per section

    uint32_t name_len;
    std::memcpy(&name_len, header + 8, 4);
    robot_name_ = std::string(header + 12, std::min<uint32_t>(name_len, 63));

    std::memcpy(&d0_, header + 76, 8);
    uint8_t ht;
    std::memcpy(&ht, header + 84, 1);
    has_tool_ = (ht != 0);

    uint32_t n_sec;
    std::memcpy(&n_sec, header + 85, 4);

    sections_.resize(n_sec);

    size_t table_pos = 89;
    for (uint32_t i = 0; i < n_sec; ++i) {
        auto& sec = sections_[i];
        int32_t lid, nej, np, nkd;
        uint8_t q6s;
        uint64_t sec_offset;

        std::memcpy(&lid, header + table_pos, 4); table_pos += 4;
        std::memcpy(&nej, header + table_pos, 4); table_pos += 4;
        std::memcpy(&np,  header + table_pos, 4); table_pos += 4;
        std::memcpy(&nkd, header + table_pos, 4); table_pos += 4;
        std::memcpy(&q6s, header + table_pos, 1); table_pos += 1;
        std::memcpy(&sec_offset, header + table_pos, 8); table_pos += 8;

        sec.link_id = lid;
        sec.n_eff_joints = nej;
        sec.n_points = np;
        sec.q6_skipped = (q6s != 0);

        ifs.seekg(sec_offset);
        sec.kd_tree.resize(nkd);
        ifs.read(reinterpret_cast<char*>(sec.kd_tree.data()),
                nkd * sizeof(KdNode));

        sec.points.resize(np);
        ifs.read(reinterpret_cast<char*>(sec.points.data()),
                np * sizeof(GcpcPoint));

        sec.kd_root = (nkd > 0) ? 0 : -1;
    }

    return ifs.good();
}

} // namespace envelope
} // namespace sbf
