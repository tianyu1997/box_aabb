#include "sbf/util/math_util.h"

#include <algorithm>

namespace sbf::util {

namespace {

// Length of overlap between [a_lo, a_hi] and [b_lo, b_hi] (>=0).
inline double interval_overlap(double a_lo, double a_hi,
                               double b_lo, double b_hi) {
    return std::max(0.0, std::min(a_hi, b_hi) - std::max(a_lo, b_lo));
}

}  // namespace

double face_overlap_area(const Eigen::AlignedBox3d& a,
                         const Eigen::AlignedBox3d& b,
                         int axis) {
    if (axis < 0 || axis > 2) return 0.0;

    // Faces share an axis-perpendicular plane only if the projections meet.
    const double a_lo = a.min()(axis);
    const double a_hi = a.max()(axis);
    const double b_lo = b.min()(axis);
    const double b_hi = b.max()(axis);

    // Need touching faces: a_hi == b_lo (or vice versa).
    constexpr double kPlaneEps = 1e-9;
    const bool touch_ab = std::abs(a_hi - b_lo) <= kPlaneEps;
    const bool touch_ba = std::abs(b_hi - a_lo) <= kPlaneEps;
    if (!touch_ab && !touch_ba) return 0.0;

    // Cross-section overlap on the other two axes.
    const int u = (axis + 1) % 3;
    const int v = (axis + 2) % 3;
    const double ou = interval_overlap(a.min()(u), a.max()(u),
                                       b.min()(u), b.max()(u));
    const double ov = interval_overlap(a.min()(v), a.max()(v),
                                       b.min()(v), b.max()(v));
    return ou * ov;
}

bool has_face_overlap(const Eigen::AlignedBox3d& a,
                      const Eigen::AlignedBox3d& b,
                      double epsilon) {
    for (int axis = 0; axis < 3; ++axis) {
        if (face_overlap_area(a, b, axis) > epsilon) return true;
    }
    return false;
}

}  // namespace sbf::util
