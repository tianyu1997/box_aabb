// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Scene implementation
//
//  v4 optimisation: no pack_obstacles() — v4 Obstacle has .lo()/.hi()
//  directly, collision code iterates Obstacle* without interleaved copy.
//
//  迁移自 v3 scene.cpp
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/scene/scene.h"

#include <algorithm>

namespace sbf {
namespace scene {

Scene::Scene(std::vector<Obstacle> obstacles)
    : obstacles_(std::move(obstacles)) {}

void Scene::add_obstacle(const Obstacle& obs) {
    obstacles_.push_back(obs);
}

void Scene::remove_obstacle(const std::string& name) {
    obstacles_.erase(
        std::remove_if(obstacles_.begin(), obstacles_.end(),
                       [&name](const Obstacle& o) { return o.name == name; }),
        obstacles_.end());
}

void Scene::clear() {
    obstacles_.clear();
}

} // namespace scene
} // namespace sbf
