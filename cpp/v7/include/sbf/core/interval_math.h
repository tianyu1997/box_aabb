#pragma once
/// @file interval_math.h
/// @brief Interval trigonometry and 4×4 row-major interval matrix ops.
///
/// Direct port of v6's interval_math.h. Provides guaranteed enclosures
/// of sin/cos and DH-style 4×4 interval matrix multiplication used by
/// interval forward kinematics (IFK).

#include "sbf/core/types.h"

namespace sbf::core {

// Conservative interval extensions of trig functions.
Interval I_sin(double lo, double hi);
Interval I_cos(double lo, double hi);

// ─── 4×4 row-major interval matrix operations ────────────────────────────
// Only rows 0..2 are computed; row 3 is hardcoded [0, 0, 0, 1].

/// Build a single-joint DH interval matrix from precomputed sin/cos and offset
/// ranges.
void build_dh_joint(double alpha, double a,
                    double ct_lo, double ct_hi,
                    double st_lo, double st_hi,
                    double d_lo,  double d_hi,
                    double A_lo[16], double A_hi[16]);

/// Multiply two interval matrices: R = T * A.
void imat_mul_dh(const double T_lo[16], const double T_hi[16],
                 const double A_lo[16], const double A_hi[16],
                 double R_lo[16], double R_hi[16]);

inline void imat_identity(double lo[16], double hi[16]) {
    for (int i = 0; i < 16; ++i) lo[i] = hi[i] = 0.0;
    lo[0]  = hi[0]  = 1.0;
    lo[5]  = hi[5]  = 1.0;
    lo[10] = hi[10] = 1.0;
    lo[15] = hi[15] = 1.0;
}

}  // namespace sbf::core
