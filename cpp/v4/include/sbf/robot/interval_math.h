// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Interval arithmetic functions
//  Module: sbf::robot
//  Conservative interval enclosures for sin/cos, matrix operations
//  迁移自 v3 interval_math.h，包含 outward rounding + build_dh_joint + imat_mul_dh
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/core/types.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace sbf {

// outward_lo / outward_hi are now defined in sbf/core/types.h
// and available here via the #include above.

// ─── Interval trigonometric functions ───────────────────────────────────────
Interval I_sin(double lo, double hi);
Interval I_cos(double lo, double hi);

// ─── 4x4 interval matrix operations (flat row-major double[16]) ────────────
void build_dh_joint(double alpha, double a,
                    double ct_lo, double ct_hi,
                    double st_lo, double st_hi,
                    double d_lo,  double d_hi,
                    double A_lo[16], double A_hi[16]);

void imat_mul_dh(const double T_lo[16], const double T_hi[16],
                 const double A_lo[16], const double A_hi[16],
                 double R_lo[16], double R_hi[16]);

inline void imat_identity(double lo[16], double hi[16]) {
    for (int i = 0; i < 16; ++i) lo[i] = hi[i] = 0.0;
    lo[0] = hi[0] = 1.0;
    lo[5] = hi[5] = 1.0;
    lo[10] = hi[10] = 1.0;
    lo[15] = hi[15] = 1.0;
}

inline void imat_copy(const double src_lo[16], const double src_hi[16],
                      double dst_lo[16], double dst_hi[16]) {
    for (int i = 0; i < 16; ++i) {
        dst_lo[i] = src_lo[i];
        dst_hi[i] = src_hi[i];
    }
}

} // namespace sbf
