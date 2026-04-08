// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — AABBCollisionChecker implementation
//
//  v4 optimisation vs v3:
//    • No pack_obstacles() / obs_compact_ — delegates directly to
//      envelope::collide_aabb() which takes Obstacle* (centre+half_sizes).
//    • No Robot* dependency — the checker operates purely on pre-computed
//      link iAABBs, decoupled from FK / robot model.
//    • Single-responsibility: check_config / check_box / check_segment are
//      planner-level concerns, not collision-checker concerns.
//
//  迁移自 v3 collision.cpp
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/scene/aabb_collision_checker.h"
#include "sbf/envelope/collision_policy.h"

namespace sbf {
namespace scene {

AABBCollisionChecker::AABBCollisionChecker(const Scene& scene)
    : scene_(&scene) {}

bool AABBCollisionChecker::check_collision(
    const float* link_iaabbs, int n_links) const
{
    if (!scene_ || scene_->n_obstacles() == 0 || n_links == 0)
        return false;

    return envelope::collide_aabb(
        link_iaabbs, n_links,
        scene_->obstacles(), scene_->n_obstacles());
}

} // namespace scene
} // namespace sbf
