// SafeBoxForest — Collision detection interface
// Module: sbf::scene
#pragma once

#include "sbf/common/types.h"
#include <Eigen/Core>
#include <vector>

namespace sbf {

// ─── Abstract collision checking interface ──────────────────────────────────
// Allows different collision backends (AABB-SAT, FCL, etc.)
// and mock implementations for testing.
class ICollisionChecker {
public:
    virtual ~ICollisionChecker() = default;

    // Check single configuration for collision (returns true if colliding)
    virtual bool check_config(const Eigen::VectorXd& q) const = 0;

    // Check if a box (C-space hyperrectangle) is collision-free
    // Returns true if colliding
    virtual bool check_box(const std::vector<Interval>& intervals) const = 0;

    // Check if a line segment between two configs is collision-free
    // Returns true if colliding
    virtual bool check_segment(const Eigen::VectorXd& q1,
                               const Eigen::VectorXd& q2,
                               double step = 0.05) const = 0;

    // Number of collision checks performed
    virtual int n_checks() const = 0;
    virtual void reset_counter() = 0;
};

} // namespace sbf
