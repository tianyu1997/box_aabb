#pragma once
/// @file dh_enumerate.h
/// @brief Shared DH enumeration utilities for CritSample and Analytical sources.
///
/// Provides precomputed DH matrix infrastructure and prefix-cached iterative
/// critical-point enumeration.  Key components:
///   - `PreDH` — precomputed 3×4 DH matrix for a (joint, candidate) pair.
///   - `build_dh_matrix()` — build DH matrix for joint at a given angle.
///   - `collect_kpi2()` — gather kπ/2 candidate values within an interval.
///   - `enumerate_critical_iterative()` — odometer-style combo traversal
///     with prefix transform reuse (eliminates sin/cos from hot loop).

#include <sbf/core/types.h>
#include <sbf/core/robot.h>

#include <cmath>
#include <vector>

namespace sbf {

// Joints narrower than this (rad) collapse to a single midpoint candidate.
static constexpr double kNarrowThreshold = 0.01;  // ~0.57°

// Precomputed DH matrix for one (joint, candidate) pair.
// Stores rows 0-2 of the 4×4 DH matrix (row 3 is always [0,0,0,1]).
struct PreDH {
    double A[12];
};

// Build the 3×4 DH matrix for joint described by `dh` at value `q_val`.
inline void build_dh_matrix(const DHParam& dh, double q_val, double A[12]) {
    double d_val = (dh.joint_type == 1) ? (q_val + dh.d) : dh.d;
    double angle = (dh.joint_type == 0) ? (q_val + dh.theta) : dh.theta;
    double ct = std::cos(angle), st = std::sin(angle);
    double ca = std::cos(dh.alpha), sa = std::sin(dh.alpha);
    A[0]  = ct;      A[1]  = -st;     A[2]  = 0.0;  A[3]  = dh.a;
    A[4]  = st*ca;   A[5]  = ct*ca;   A[6]  = -sa;  A[7]  = -d_val*sa;
    A[8]  = st*sa;   A[9]  = ct*sa;   A[10] = ca;   A[11] = d_val*ca;
}

// Multiply prefix (3×4) × DH (3×4 + implicit row3=[0,0,0,1]) → result (3×4).
inline void mul_prefix_dh(const double Tprev[12], const double A[12],
                          double Tnext[12]) {
    for (int r = 0; r < 3; ++r) {
        const double* tp = Tprev + r * 4;
        double* tn = Tnext + r * 4;
        for (int c = 0; c < 4; ++c) {
            tn[c] = tp[0]*A[c] + tp[1]*A[4+c] + tp[2]*A[8+c];
        }
        tn[3] += tp[3];  // + Tprev[r,3] × 1 (implicit A[3,3]=1)
    }
}

// Collect kπ/2 candidate values within [lo, hi].
// Narrow intervals (width < kNarrowThreshold) collapse to a single midpoint.
inline void collect_kpi2(double lo, double hi, std::vector<double>& out) {
    if (hi - lo < kNarrowThreshold) {
        out.push_back(0.5 * (lo + hi));
        return;
    }
    out.push_back(lo);
    out.push_back(hi);
    double k_lo = std::ceil(lo / HALF_PI);
    double k_hi = std::floor(hi / HALF_PI);
    for (double k = k_lo; k <= k_hi; k += 1.0) {
        double v = k * HALF_PI;
        if (v > lo + 1e-12 && v < hi - 1e-12)
            out.push_back(v);
    }
}

// Expand endpoint iAABBs from FK positions.
// Layout: out[ci*2+ep][d] where ep=0 proximal, ep=1 distal.
inline void update_endpoints_from_positions(
    const double positions[][3],
    const int* active_link_map, int n_active,
    float* out)
{
    for (int ci = 0; ci < n_active; ++ci) {
        int V = active_link_map[ci];
        float* p = out + (ci * 2) * 6;
        float px = static_cast<float>(positions[V][0]);
        float py = static_cast<float>(positions[V][1]);
        float pz = static_cast<float>(positions[V][2]);
        if (px < p[0]) p[0] = px;  if (px > p[3]) p[3] = px;
        if (py < p[1]) p[1] = py;  if (py > p[4]) p[4] = py;
        if (pz < p[2]) p[2] = pz;  if (pz > p[5]) p[5] = pz;

        float* d = out + (ci * 2 + 1) * 6;
        float dx = static_cast<float>(positions[V + 1][0]);
        float dy = static_cast<float>(positions[V + 1][1]);
        float dz = static_cast<float>(positions[V + 1][2]);
        if (dx < d[0]) d[0] = dx;  if (dx > d[3]) d[3] = dx;
        if (dy < d[1]) d[1] = dy;  if (dy > d[4]) d[4] = dy;
        if (dz < d[2]) d[2] = dz;  if (dz > d[5]) d[5] = dz;
    }
}

// Initialize endpoint iAABBs to inverted extremes (±1e30).
inline void init_endpoints_inf(float* out, int n_active) {
    for (int i = 0; i < n_active * 2; ++i) {
        float* a = out + i * 6;
        a[0] = a[1] = a[2] =  1e30f;
        a[3] = a[4] = a[5] = -1e30f;
    }
}

// Iterative prefix-cached critical point enumeration.
// Precomputed DH matrices eliminate sin/cos from the hot loop.
// Odometer-style combination traversal with prefix transform reuse.
void enumerate_critical_iterative(
    const Robot& robot,
    const std::vector<std::vector<PreDH>>& pre_dh,
    const std::vector<int>& n_cands,
    const int* active_link_map, int n_active,
    float* out);

}  // namespace sbf
