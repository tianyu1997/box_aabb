#pragma once
/// @file types.h
/// @brief Core numeric types for v7 IFK pipeline.
///
/// Subset of v6's types.h: only `Interval`, `JointLimits`, and dimensional
/// constants required by the FK / endpoint-AABB pipeline. Workspace-side
/// types (Obstacle, BoxNode) belong to later phases.

#include <algorithm>
#include <cstdint>
#include <vector>

namespace sbf::core {

// ─── Constants ────────────────────────────────────────────────────────────
constexpr int MAX_JOINTS = 32;   ///< Maximum number of joints supported.
constexpr int MAX_TF     = 34;   ///< MAX_JOINTS + base + optional tool frame.

constexpr double PI            = 3.141592653589793;
constexpr double TWO_PI        = 6.283185307179586;
constexpr double HALF_PI       = 1.5707963267948966;
constexpr double THREE_HALF_PI = 4.71238898038469;

constexpr double CONTAIN_TOL = 1e-12;

// ─── Interval ─────────────────────────────────────────────────────────────

/// Closed 1-D interval [lo, hi] with basic interval arithmetic.
struct Interval {
    double lo = 0.0;
    double hi = 0.0;

    Interval() = default;
    Interval(double l, double h) : lo(l), hi(h) {}

    double width()  const { return hi - lo; }
    double center() const { return 0.5 * (lo + hi); }
    bool   empty()  const { return lo > hi; }

    bool contains(double v, double tol = CONTAIN_TOL) const {
        return v >= lo - tol && v <= hi + tol;
    }
    bool overlaps(const Interval& o, double tol = 0.0) const {
        return lo <= o.hi + tol && o.lo <= hi + tol;
    }
};

// ─── JointLimits ──────────────────────────────────────────────────────────

struct JointLimits {
    std::vector<Interval> limits;
    int n_dims() const { return static_cast<int>(limits.size()); }
};

}  // namespace sbf::core
