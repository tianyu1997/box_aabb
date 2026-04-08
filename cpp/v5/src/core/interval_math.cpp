#include <sbf/core/interval_math.h>

#include <algorithm>
#include <cmath>

namespace sbf {

Interval I_sin(double lo, double hi) {
    double w = hi - lo;
    if (w >= TWO_PI)
        return {-1.0, 1.0};

    double s_lo = std::sin(lo);
    double s_hi = std::sin(hi);
    double r_lo = std::min(s_lo, s_hi);
    double r_hi = std::max(s_lo, s_hi);

    double lo_n = std::fmod(lo, TWO_PI);
    if (lo_n < 0.0) lo_n += TWO_PI;
    double hi_n = lo_n + w;

    // Check if π/2 (max) is in the interval
    {
        double k = std::ceil((lo_n - HALF_PI) / TWO_PI);
        double check = HALF_PI + k * TWO_PI;
        if (check <= hi_n + 1e-15) r_hi = 1.0;
    }
    // Check if 3π/2 (min) is in the interval
    {
        double k = std::ceil((lo_n - THREE_HALF_PI) / TWO_PI);
        double check = THREE_HALF_PI + k * TWO_PI;
        if (check <= hi_n + 1e-15) r_lo = -1.0;
    }

    return {r_lo, r_hi};
}

Interval I_cos(double lo, double hi) {
    double w = hi - lo;
    if (w >= TWO_PI)
        return {-1.0, 1.0};

    double c_lo = std::cos(lo);
    double c_hi = std::cos(hi);
    double r_lo = std::min(c_lo, c_hi);
    double r_hi = std::max(c_lo, c_hi);

    double lo_n = std::fmod(lo, TWO_PI);
    if (lo_n < 0.0) lo_n += TWO_PI;
    double hi_n = lo_n + w;

    // Check if 0 (max) is in the interval
    {
        double k = std::ceil(lo_n / TWO_PI);
        double check = k * TWO_PI;
        if (check <= hi_n + 1e-15) r_hi = 1.0;
    }
    // Check if π (min) is in the interval
    {
        double k = std::ceil((lo_n - PI) / TWO_PI);
        double check = PI + k * TWO_PI;
        if (check <= hi_n + 1e-15) r_lo = -1.0;
    }

    return {r_lo, r_hi};
}

void build_dh_joint(double alpha, double a,
                    double ct_lo, double ct_hi,
                    double st_lo, double st_hi,
                    double d_lo,  double d_hi,
                    double A_lo[16], double A_hi[16]) {
    double ca = std::cos(alpha);
    double sa = std::sin(alpha);

    auto imul_s = [](double a_lo, double a_hi, double s,
                     double& r_lo, double& r_hi) {
        double v1 = a_lo * s, v2 = a_hi * s;
        r_lo = std::min(v1, v2);
        r_hi = std::max(v1, v2);
    };

    for (int i = 0; i < 16; ++i) A_lo[i] = A_hi[i] = 0.0;
    A_lo[15] = A_hi[15] = 1.0;

    // Row 0: [ct, -st, 0, a]
    A_lo[0] = ct_lo; A_hi[0] = ct_hi;
    A_lo[1] = -st_hi; A_hi[1] = -st_lo;
    A_lo[3] = A_hi[3] = a;

    // Row 1: [st*ca, ct*ca, -sa, -d*sa]
    imul_s(st_lo, st_hi, ca, A_lo[4], A_hi[4]);
    imul_s(ct_lo, ct_hi, ca, A_lo[5], A_hi[5]);
    A_lo[6] = A_hi[6] = -sa;
    imul_s(d_lo, d_hi, -sa, A_lo[7], A_hi[7]);

    // Row 2: [st*sa, ct*sa, ca, d*ca]
    imul_s(st_lo, st_hi, sa, A_lo[8], A_hi[8]);
    imul_s(ct_lo, ct_hi, sa, A_lo[9], A_hi[9]);
    A_lo[10] = A_hi[10] = ca;
    imul_s(d_lo, d_hi, ca, A_lo[11], A_hi[11]);
}

void imat_mul_dh(const double T_lo[16], const double T_hi[16],
                 const double A_lo[16], const double A_hi[16],
                 double R_lo[16], double R_hi[16]) {
    R_lo[12] = R_hi[12] = 0.0;
    R_lo[13] = R_hi[13] = 0.0;
    R_lo[14] = R_hi[14] = 0.0;
    R_lo[15] = R_hi[15] = 1.0;

    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 4; ++j) {
            double lo_sum = 0.0, hi_sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                int t_idx = i * 4 + k;
                int a_idx = k * 4 + j;
                double p1 = T_lo[t_idx] * A_lo[a_idx];
                double p2 = T_lo[t_idx] * A_hi[a_idx];
                double p3 = T_hi[t_idx] * A_lo[a_idx];
                double p4 = T_hi[t_idx] * A_hi[a_idx];
                lo_sum += std::min({p1, p2, p3, p4});
                hi_sum += std::max({p1, p2, p3, p4});
            }
            if (j == 3) {
                lo_sum += T_lo[i * 4 + 3];
                hi_sum += T_hi[i * 4 + 3];
            }
            R_lo[i * 4 + j] = lo_sum;
            R_hi[i * 4 + j] = hi_sum;
        }
    }
}

}  // namespace sbf
