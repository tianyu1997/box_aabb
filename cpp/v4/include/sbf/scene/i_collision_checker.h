// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Abstract collision checker interface
//  Module: sbf::scene
//
//  v4 simplified interface vs v3:
//    v3 had check_config / check_box / check_segment — those are
//    planner-level concerns.  v4 keeps only the primitive iAABB check
//    so collision checkers are stateless w.r.t. FK / robot model.
//
//  迁移自 v3 i_collision_checker.h
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/core/types.h"

namespace sbf {
namespace scene {

class ICollisionChecker {
public:
    virtual ~ICollisionChecker() = default;

    /// Returns true if ANY link iAABB collides with ANY obstacle.
    /// link_iaabbs: [n_links × 6]  layout [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
    virtual bool check_collision(const float* link_iaabbs, int n_links) const = 0;
};

} // namespace scene
} // namespace sbf
