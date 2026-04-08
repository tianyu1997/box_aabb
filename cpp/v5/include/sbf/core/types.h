#pragma once
/// @file types.h
/// @brief Core data types and constants for SafeBoxForest v5.
///
/// Defines the fundamental building blocks used throughout the library:
///   - Numeric constants (PI, tolerances, dimension limits)
///   - `Interval` — closed 1-D interval with arithmetic operators
///   - `Obstacle` — axis-aligned bounding box in workspace (3-D)
///   - `JointLimits` — per-joint interval bounds
///   - `BoxNode` — a collision-free axis-aligned box in configuration space

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <string>

namespace sbf {

// ─── Constants ──────────────────────────────────────────────────────────────
constexpr int MAX_JOINTS = 32;            ///< Maximum number of joints supported.
constexpr int MAX_TF     = 34;            ///< MAX_JOINTS + 2 (base frame + tool frame).

constexpr double PI             = 3.141592653589793;   ///< π
constexpr double TWO_PI         = 6.283185307179586;   ///< 2π
constexpr double HALF_PI        = 1.5707963267948966;  ///< π/2
constexpr double THREE_HALF_PI  = 4.71238898038469;    ///< 3π/2

constexpr double SAT_EPS     = 1e-10;  ///< Tolerance for SAT (Separating Axis Theorem) tests.
constexpr double CONTAIN_TOL = 1e-10;  ///< Tolerance for point-in-interval containment checks.

// ─── Interval ───────────────────────────────────────────────────────────────

/// @brief Closed 1-D interval [lo, hi] with basic interval arithmetic.
///
/// Used to represent joint ranges, configuration-space extents, and
/// intermediate results of interval forward-kinematics (IFK).
struct Interval {
    double lo = 0.0;  ///< Lower bound.
    double hi = 0.0;  ///< Upper bound.

    Interval() = default;
    /// @param l Lower bound.  @param h Upper bound.
    Interval(double l, double h) : lo(l), hi(h) {}

    double width()  const { return hi - lo; }        ///< Interval width (hi − lo).
    double center() const { return 0.5 * (lo + hi); } ///< Midpoint.
    bool   empty()  const { return lo > hi; }          ///< True if degenerate (lo > hi).

    /// Check whether scalar @p v lies within [lo−tol, hi+tol].
    bool contains(double v, double tol = CONTAIN_TOL) const {
        return v >= lo - tol && v <= hi + tol;
    }

    /// Check whether this interval overlaps @p other (with tolerance).
    bool overlaps(const Interval& other, double tol = 0.0) const {
        return lo <= other.hi + tol && other.lo <= hi + tol;
    }

    Interval operator+(const Interval& o) const { return {lo + o.lo, hi + o.hi}; }  ///< Interval addition.
    Interval operator-(const Interval& o) const { return {lo - o.hi, hi - o.lo}; }  ///< Interval subtraction.

    /// Interval multiplication (all four endpoint products).
    Interval operator*(const Interval& o) const {
        double p1 = lo * o.lo, p2 = lo * o.hi;
        double p3 = hi * o.lo, p4 = hi * o.hi;
        return {std::min({p1, p2, p3, p4}), std::max({p1, p2, p3, p4})};
    }

    /// Scalar multiplication.
    Interval operator*(double s) const {
        double a = lo * s, b = hi * s;
        return {std::min(a, b), std::max(a, b)};
    }

    /// Convex hull (tightest enclosing interval of both).
    Interval hull(const Interval& o) const {
        return {std::min(lo, o.lo), std::max(hi, o.hi)};
    }

    /// Intersection (may return empty interval if disjoint).
    Interval intersect(const Interval& o) const {
        return {std::max(lo, o.lo), std::min(hi, o.hi)};
    }
};

// ─── Obstacle ───────────────────────────────────────────────────────────────

/// @brief Axis-aligned bounding box (AABB) obstacle in 3-D workspace.
///
/// Stored as 6 floats: [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z].
/// Used for both workspace obstacles and link envelope collision checks.
struct Obstacle {
    float bounds[6] = {};  ///< [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]

    Obstacle() = default;
    /// Construct from lower and upper corner coordinates.
    Obstacle(float lx, float ly, float lz, float hx, float hy, float hz)
        : bounds{lx, ly, lz, hx, hy, hz} {}
};

// ─── JointLimits ────────────────────────────────────────────────────────────

/// @brief Per-joint interval limits defining the robot's configuration space.
struct JointLimits {
    std::vector<Interval> limits;  ///< One Interval per joint.
    int n_dims() const { return static_cast<int>(limits.size()); }  ///< Number of joints.
};

// ─── BoxNode ────────────────────────────────────────────────────────────────

/// @brief A collision-free axis-aligned box in configuration space.
///
/// Each BoxNode represents a hyper-rectangular region of C-space that is
/// certified collision-free by the envelope pipeline.  Boxes are the
/// fundamental output of the forest grower and the input to the planner.
///
/// The box is defined by per-joint intervals (`joint_intervals`).
/// Its `seed_config` is the C-space point that triggered its creation.
struct BoxNode {
    int id = -1;                                ///< Unique box identifier.
    std::vector<Interval> joint_intervals;      ///< Per-joint [lo, hi] defining the box.
    Eigen::VectorXd seed_config;                ///< C-space seed that created this box.
    double volume = 0.0;                        ///< Product of interval widths.
    int tree_id = -1;                           ///< LECT leaf node backing this box.
    int parent_box_id = -1;                     ///< Parent box in the RRT/wavefront tree (-1 for roots).
    int root_id = -1;                           ///< Root box of this subtree.

    int n_dims() const { return static_cast<int>(joint_intervals.size()); }  ///< Dimensionality.

    /// Compute the geometric center of the box.
    Eigen::VectorXd center() const {
        Eigen::VectorXd c(n_dims());
        for (int d = 0; d < n_dims(); ++d)
            c[d] = joint_intervals[d].center();
        return c;
    }

    /// Recompute `volume` from current `joint_intervals`.
    void compute_volume() {
        volume = 1.0;
        for (auto& iv : joint_intervals)
            volume *= iv.width();
    }

    /// Test whether configuration @p q lies inside this box (with tolerance).
    bool contains(const Eigen::VectorXd& q, double tol = CONTAIN_TOL) const {
        for (int d = 0; d < n_dims(); ++d)
            if (!joint_intervals[d].contains(q[d], tol))
                return false;
        return true;
    }
};

}  // namespace sbf
