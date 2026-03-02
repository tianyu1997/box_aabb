// SafeBoxForest — Collision detection (SAT-based AABB)
#pragma once

#include "sbf/core/robot.h"
#include "sbf/core/types.h"
#include <Eigen/Core>
#include <vector>

namespace sbf {

class Scene;  // forward decl

// ─── Primitive SAT functions ────────────────────────────────────────────────

// 3D AABB overlap test with tolerance
inline bool aabb_overlap_3d(const float* min1, const float* max1,
                            const float* min2, const float* max2,
                            float eps = 1e-10f) {
    if (max1[0] < min2[0] - eps || max2[0] < min1[0] - eps) return false;
    if (max1[1] < min2[1] - eps || max2[1] < min1[1] - eps) return false;
    if (max1[2] < min2[2] - eps || max2[2] < min1[2] - eps) return false;
    return true;
}

// Check if inflated link/EE AABBs collide with compact obstacle array.
// aabb:        [n_slots * 6] float32 — inflated AABBs in [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
// obs_compact: [n_obs * 6] float32 — raw obstacle bounds [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]
// Outer loop over AABB slots, inner loop over obstacles — AABB values cached in registers.
bool aabbs_collide_obs(const float* aabb, int n_slots,
                       const float* obs_compact, int n_obs);

// ─── CollisionChecker ───────────────────────────────────────────────────────
class CollisionChecker {
public:
    CollisionChecker() = default;
    CollisionChecker(const Robot& robot, const std::vector<Obstacle>& obstacles);

    // Check single configuration for collision
    bool check_config(const Eigen::VectorXd& q) const;

    // Check if a box (C-space hyperrectangle) is collision-free
    // Uses interval FK to compute link AABB envelopes
    bool check_box(const std::vector<Interval>& intervals) const;

    // Check if a line segment between two configs is collision-free
    // Discretizes the segment at resolution `step`
    bool check_segment(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
                       double step = 0.05) const;

    // Number of collision checks performed
    int n_checks() const { return n_checks_; }
    void reset_counter() { n_checks_ = 0; }

    // Access compact obstacle array [n_obs * 6] and count
    const float* obs_compact() const { return obs_compact_.data(); }
    int n_obs() const { return static_cast<int>(obs_compact_.size() / 6); }
    int n_aabb_slots() const;  // n_active_links + n_ee_aabb_slots

    const Robot& robot() const { return *robot_; }

private:
    const Robot* robot_ = nullptr;
    std::vector<Obstacle> obstacles_;
    std::vector<float> obs_compact_;  // compact obstacle array [n_obs * 6]
    mutable int n_checks_ = 0;

    void pack_obstacles();
};

} // namespace sbf
