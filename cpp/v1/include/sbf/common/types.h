// SafeBoxForest — Core type definitions (shared across all modules)
// Ported from v4 Python/Cython implementation
#pragma once

#include <Eigen/Core>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace sbf {

// ─── Constants ──────────────────────────────────────────────────────────────
constexpr int MAX_JOINTS = 32;
constexpr int MAX_TF     = 34;   // MAX_JOINTS + 2 (base + tool)
constexpr int MAX_LINKS  = 32;
constexpr int MAX_DIMS   = 16;

constexpr double PI           = 3.141592653589793;
constexpr double TWO_PI       = 6.283185307179586;
constexpr double HALF_PI      = 1.5707963267948966;
constexpr double THREE_HALF_PI = 4.71238898038469;

constexpr double SAT_EPS      = 1e-10;
constexpr double CONTAIN_TOL  = 1e-10;
constexpr double INVARIANT_TOL = 1e-8;

// ─── Interval ───────────────────────────────────────────────────────────────
struct Interval {
    double lo = 0.0;
    double hi = 0.0;

    Interval() = default;
    Interval(double l, double h) : lo(l), hi(h) {}

    double width()  const { return hi - lo; }
    double center() const { return 0.5 * (lo + hi); }
    double mid()    const { return 0.5 * (lo + hi); }

    bool contains(double v, double tol = CONTAIN_TOL) const {
        return v >= lo - tol && v <= hi + tol;
    }

    bool overlaps(const Interval& other, double tol = 0.0) const {
        return lo <= other.hi + tol && other.lo <= hi + tol;
    }

    // Interval arithmetic
    Interval operator+(const Interval& o) const { return {lo + o.lo, hi + o.hi}; }
    Interval operator-(const Interval& o) const { return {lo - o.hi, hi - o.lo}; }

    Interval operator*(const Interval& o) const {
        double p1 = lo * o.lo, p2 = lo * o.hi;
        double p3 = hi * o.lo, p4 = hi * o.hi;
        return {std::min({p1, p2, p3, p4}), std::max({p1, p2, p3, p4})};
    }

    // Multiply by scalar
    Interval operator*(double s) const {
        double a = lo * s, b = hi * s;
        return {std::min(a, b), std::max(a, b)};
    }

    // Union (hull)
    Interval hull(const Interval& o) const {
        return {std::min(lo, o.lo), std::max(hi, o.hi)};
    }

    // Intersection
    Interval intersect(const Interval& o) const {
        return {std::max(lo, o.lo), std::min(hi, o.hi)};
    }
};

// ─── AABB3D ─────────────────────────────────────────────────────────────────
struct AABB3D {
    float lo[3] = {0, 0, 0};
    float hi[3] = {0, 0, 0};

    bool overlaps(const AABB3D& o, float eps = 1e-10f) const {
        if (hi[0] < o.lo[0] - eps || o.hi[0] < lo[0] - eps) return false;
        if (hi[1] < o.lo[1] - eps || o.hi[1] < lo[1] - eps) return false;
        if (hi[2] < o.lo[2] - eps || o.hi[2] < lo[2] - eps) return false;
        return true;
    }
};

// ─── Obstacle ───────────────────────────────────────────────────────────────
struct Obstacle {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    Eigen::Vector3d half_sizes = Eigen::Vector3d::Zero();
    std::string name;

    Obstacle() = default;
    Obstacle(Eigen::Vector3d c, Eigen::Vector3d hs, std::string n = "")
        : center(std::move(c)), half_sizes(std::move(hs)), name(std::move(n)) {}

    Eigen::Vector3d lo() const { return center - half_sizes; }
    Eigen::Vector3d hi() const { return center + half_sizes; }
};

// ─── BoxNode ────────────────────────────────────────────────────────────────
struct BoxNode {
    int id = -1;
    std::vector<Interval> joint_intervals;
    Eigen::VectorXd seed_config;
    double volume = 0.0;
    int parent_id = -1;
    int tree_id = -1;
    std::vector<int> children_ids;

    BoxNode() = default;
    BoxNode(int id_, std::vector<Interval> ivs, Eigen::VectorXd seed)
        : id(id_), joint_intervals(std::move(ivs)), seed_config(std::move(seed))
    {
        compute_volume();
    }

    int n_dims() const { return static_cast<int>(joint_intervals.size()); }

    Eigen::VectorXd center() const {
        Eigen::VectorXd c(n_dims());
        for (int d = 0; d < n_dims(); ++d)
            c[d] = joint_intervals[d].center();
        return c;
    }

    void compute_volume() {
        volume = 1.0;
        for (auto& iv : joint_intervals)
            volume *= iv.width();
    }

    bool contains(const Eigen::VectorXd& q, double tol = CONTAIN_TOL) const {
        for (int d = 0; d < n_dims(); ++d)
            if (!joint_intervals[d].contains(q[d], tol))
                return false;
        return true;
    }

    double distance_to_config(const Eigen::VectorXd& q) const {
        double dist_sq = 0.0;
        for (int d = 0; d < n_dims(); ++d) {
            double v = q[d];
            double lo = joint_intervals[d].lo;
            double hi = joint_intervals[d].hi;
            if (v < lo) dist_sq += (lo - v) * (lo - v);
            else if (v > hi) dist_sq += (v - hi) * (v - hi);
        }
        return std::sqrt(dist_sq);
    }

    Eigen::VectorXd nearest_point_to(const Eigen::VectorXd& q) const {
        Eigen::VectorXd p(n_dims());
        for (int d = 0; d < n_dims(); ++d)
            p[d] = std::clamp(q[d], joint_intervals[d].lo, joint_intervals[d].hi);
        return p;
    }

    // Check if this box overlaps with another in all dimensions
    bool overlaps_with(const BoxNode& other, double tol = 0.0) const {
        for (int d = 0; d < n_dims(); ++d)
            if (!joint_intervals[d].overlaps(other.joint_intervals[d], tol))
                return false;
        return true;
    }

    // Check if two boxes are adjacent (touching but not deeply overlapping)
    // touching = overlap width >= 0 in all dims, exactly 0 in at least one dim
    bool is_adjacent_to(const BoxNode& other, double tol = 1e-10) const {
        bool has_touching_dim = false;
        for (int d = 0; d < n_dims(); ++d) {
            double overlap = std::min(joint_intervals[d].hi, other.joint_intervals[d].hi)
                           - std::max(joint_intervals[d].lo, other.joint_intervals[d].lo);
            if (overlap < -tol) return false;  // separated in this dim
            if (overlap <= tol) has_touching_dim = true;
        }
        return has_touching_dim;  // adjacent = touching in at least one dim
    }

    // Compute shared face center with another adjacent box
    Eigen::VectorXd shared_face_center(const BoxNode& other) const {
        Eigen::VectorXd c(n_dims());
        for (int d = 0; d < n_dims(); ++d) {
            double overlap_lo = std::max(joint_intervals[d].lo, other.joint_intervals[d].lo);
            double overlap_hi = std::min(joint_intervals[d].hi, other.joint_intervals[d].hi);
            c[d] = 0.5 * (overlap_lo + overlap_hi);
        }
        return c;
    }
};

// ─── JointLimits ────────────────────────────────────────────────────────────
struct JointLimits {
    std::vector<Interval> limits;

    int n_dims() const { return static_cast<int>(limits.size()); }

    bool contains(const Eigen::VectorXd& q, double tol = CONTAIN_TOL) const {
        for (int d = 0; d < n_dims(); ++d)
            if (!limits[d].contains(q[d], tol))
                return false;
        return true;
    }

    Eigen::VectorXd clamp(const Eigen::VectorXd& q) const {
        Eigen::VectorXd c(n_dims());
        for (int d = 0; d < n_dims(); ++d)
            c[d] = std::clamp(q[d], limits[d].lo, limits[d].hi);
        return c;
    }
};

// ─── Planning Result ────────────────────────────────────────────────────────
struct PlanningResult {
    bool success = false;
    Eigen::MatrixXd path;           // (N, DOF) waypoints
    double cost = 0.0;              // path length (L2 joint space)
    double planning_time = 0.0;     // wall-clock seconds (total)
    double first_solution_time = 0.0;
    int collision_checks = 0;
    int nodes_explored = 0;
    std::unordered_map<std::string, double> phase_times;
    std::unordered_map<std::string, double> metadata;

    int n_waypoints() const { return static_cast<int>(path.rows()); }

    static PlanningResult failure(double time) {
        PlanningResult r;
        r.planning_time = time;
        return r;
    }
};

// ─── FFBResult (find_free_box result) ───────────────────────────────────────
struct FFBResult {
    int node_idx = -1;
    std::vector<int> path;    // descent path (node indices)
    int fail_code = 0;        // 0=success, 1=occupied, 2=depth, 3=edge
    int n_new_nodes = 0;
    int n_fk_calls = 0;

    bool success() const { return fail_code == 0 && node_idx >= 0; }
};

// ─── Edge (for graph) ───────────────────────────────────────────────────────
struct Edge {
    int from_id = -1;
    int to_id = -1;
    double weight = 0.0;

    Edge() = default;
    Edge(int f, int t, double w = 1.0) : from_id(f), to_id(t), weight(w) {}
};

} // namespace sbf
