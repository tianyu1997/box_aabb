// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — AABB-based collision checker
//  Module: sbf::scene
//
//  Implements ICollisionChecker by delegating to envelope::collide_aabb().
//  Holds a const pointer to a Scene for obstacle access — no separate
//  obs_compact_ copy (v4 optimisation over v3).
//
//  迁移自 v3 aabb_collision_checker.h
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/scene/i_collision_checker.h"
#include "sbf/scene/scene.h"

namespace sbf {
namespace scene {

class AABBCollisionChecker : public ICollisionChecker {
public:
    AABBCollisionChecker() = default;

    /// Construct with a scene whose lifetime must outlast this checker.
    explicit AABBCollisionChecker(const Scene& scene);

    /// Check if any link iAABB collides with any obstacle in the scene.
    /// link_iaabbs  : [n_links × 6]  layout [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
    bool check_collision(const float* link_iaabbs, int n_links) const override;

    const Scene* scene() const { return scene_; }

private:
    const Scene* scene_ = nullptr;
};

} // namespace scene
} // namespace sbf
