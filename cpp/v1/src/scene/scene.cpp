// SafeBoxForest — Scene implementation
#include "sbf/scene/scene.h"
#include <algorithm>
#include <stdexcept>

namespace sbf {

Scene::Scene(std::vector<Obstacle> obstacles)
    : obstacles_(std::move(obstacles))
{
    pack_obstacles();
}

void Scene::add_obstacle(const Obstacle& obs) {
    obstacles_.push_back(obs);
    pack_obstacles();
}

void Scene::remove_obstacle(const std::string& name) {
    obstacles_.erase(
        std::remove_if(obstacles_.begin(), obstacles_.end(),
                       [&name](const Obstacle& o) { return o.name == name; }),
        obstacles_.end());
    pack_obstacles();
}

void Scene::clear() {
    obstacles_.clear();
    obs_compact_.clear();
}

void Scene::repack() {
    pack_obstacles();
}

void Scene::pack_obstacles() {
    obs_compact_.resize(obstacles_.size() * 6);
    for (size_t i = 0; i < obstacles_.size(); ++i) {
        auto lo = obstacles_[i].lo();
        auto hi = obstacles_[i].hi();
        obs_compact_[i * 6 + 0] = static_cast<float>(lo[0]);
        obs_compact_[i * 6 + 1] = static_cast<float>(hi[0]);
        obs_compact_[i * 6 + 2] = static_cast<float>(lo[1]);
        obs_compact_[i * 6 + 3] = static_cast<float>(hi[1]);
        obs_compact_[i * 6 + 4] = static_cast<float>(lo[2]);
        obs_compact_[i * 6 + 5] = static_cast<float>(hi[2]);
    }
}

} // namespace sbf
