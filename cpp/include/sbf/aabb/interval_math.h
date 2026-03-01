// SafeBoxForest — Interval arithmetic functions
// Conservative interval enclosures for sin/cos, matrix operations
#pragma once

#include "sbf/core/types.h"
#include <cmath>
#include <algorithm>

namespace sbf {

// ─── Interval trigonometric functions ───────────────────────────────────────
// Conservative interval enclosure for sin([lo, hi])
// Reference: v4/_interval_fk_core.pyx _isin()
Interval I_sin(double lo, double hi);

// Conservative interval enclosure for cos([lo, hi])
Interval I_cos(double lo, double hi);

// ─── 4×4 interval matrix operations (flat row-major double[16]) ────────────
// These operate on pairs of lo/hi arrays representing interval matrices.
// Only rows 0-2 are computed; row 3 is hardcoded [0,0,0,1].

// Build DH joint interval matrix from DH parameters and interval θ/d
// ct_lo/hi = cos(theta interval), st_lo/hi = sin(theta interval)
// d_lo/hi  = d interval (for prismatic) or scalar d
void build_dh_joint(double alpha, double a,
                    double ct_lo, double ct_hi,
                    double st_lo, double st_hi,
                    double d_lo,  double d_hi,
                    double A_lo[16], double A_hi[16]);

// Interval matrix multiply: R = T × A (only rows 0-2, row 3 hardcoded)
void imat_mul_dh(const double T_lo[16], const double T_hi[16],
                 const double A_lo[16], const double A_hi[16],
                 double R_lo[16], double R_hi[16]);

// Set 4×4 interval matrix to identity
inline void imat_identity(double lo[16], double hi[16]) {
    for (int i = 0; i < 16; ++i) lo[i] = hi[i] = 0.0;
    lo[0] = hi[0] = 1.0;
    lo[5] = hi[5] = 1.0;
    lo[10] = hi[10] = 1.0;
    lo[15] = hi[15] = 1.0;
}

// Copy 4×4 interval matrix
inline void imat_copy(const double src_lo[16], const double src_hi[16],
                      double dst_lo[16], double dst_hi[16]) {
    for (int i = 0; i < 16; ++i) {
        dst_lo[i] = src_lo[i];
        dst_hi[i] = src_hi[i];
    }
}

} // namespace sbf
