#pragma once
/// @file interval_math.h
/// @brief Interval trigonometry and 4×4 interval matrix operations.
///
/// Provides guaranteed-enclosing interval extensions of sin/cos,
/// and row-major 4×4 interval DH matrix construction / multiplication
/// for interval forward kinematics (IFK).

#include <sbf/core/types.h>

#include <cmath>
#include <algorithm>

namespace sbf {

// ─── Interval trigonometric functions ───────────────────────────────────────

/// Interval extension of sin over [lo, hi].  Conservative enclosure.
Interval I_sin(double lo, double hi);
/// Interval extension of cos over [lo, hi].  Conservative enclosure.
Interval I_cos(double lo, double hi);

// ─── 4×4 interval matrix operations (flat row-major double[16]) ────────────
// Only rows 0-2 are computed; row 3 is hardcoded [0,0,0,1].

/// Build a single-joint DH interval matrix from pre-computed
/// interval sin/cos and offset ranges.
void build_dh_joint(double alpha, double a,
                    double ct_lo, double ct_hi,
                    double st_lo, double st_hi,
                    double d_lo,  double d_hi,
                    double A_lo[16], double A_hi[16]);

/// Multiply two interval 4×4 matrices: R = T × A.
void imat_mul_dh(const double T_lo[16], const double T_hi[16],
                 const double A_lo[16], const double A_hi[16],
                 double R_lo[16], double R_hi[16]);

/// Set a 4×4 interval matrix to the identity.
inline void imat_identity(double lo[16], double hi[16]) {
    for (int i = 0; i < 16; ++i) lo[i] = hi[i] = 0.0;
    lo[0] = hi[0] = 1.0;
    lo[5] = hi[5] = 1.0;
    lo[10] = hi[10] = 1.0;
    lo[15] = hi[15] = 1.0;
}

/// Copy a 4×4 interval matrix.
inline void imat_copy(const double src_lo[16], const double src_hi[16],
                      double dst_lo[16], double dst_hi[16]) {
    for (int i = 0; i < 16; ++i) {
        dst_lo[i] = src_lo[i];
        dst_hi[i] = src_hi[i];
    }
}

}  // namespace sbf
