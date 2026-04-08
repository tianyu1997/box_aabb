// SafeBoxForest — AABB-SAT Collision Checker
// Module: sbf::scene
// Concrete implementation of ICollisionChecker using interval FK + SAT
#pragma once

#include "sbf/scene/i_collision_checker.h"
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"
#include <Eigen/Core>
#include <vector>

namespace sbf {

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
bool aabbs_collide_obs(const float* aabb, int n_slots,
                       const float* obs_compact, int n_obs);

// ─── AabbCollisionChecker ───────────────────────────────────────────────────
// Implements ICollisionChecker using interval FK for boxes and
// scalar FK + SAT for single configurations.
class AabbCollisionChecker : public ICollisionChecker {
public:
    AabbCollisionChecker() = default;
    AabbCollisionChecker(const Robot& robot, const std::vector<Obstacle>& obstacles);

    // ICollisionChecker interface
    bool check_config(const Eigen::VectorXd& q) const override;
    bool check_box(const std::vector<Interval>& intervals) const override;
    bool check_segment(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
                       double step = 0.05) const override;
    int n_checks() const override { return n_checks_; }
    void reset_counter() override { n_checks_ = 0; }

    // Access compact obstacle array [n_obs * 6] and count
    const float* obs_compact() const { return obs_compact_.data(); }
    int n_obs() const { return static_cast<int>(obs_compact_.size() / 6); }
    int n_aabb_slots() const;  // n_active_links + n_ee_aabb_slots

    const Robot& robot() const { return *robot_; }

private:
    const Robot* robot_ = nullptr;
    std::vector<Obstacle> obstacles_;
    std::vector<float> obs_compact_;
    mutable int n_checks_ = 0;

    void pack_obstacles();
};

// ─── Backward-compatible alias ──────────────────────────────────────────────
using CollisionChecker = AabbCollisionChecker;

} // namespace sbf
