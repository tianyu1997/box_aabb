// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Scene: obstacle collection management
//  Module: sbf::scene
//
//  v4 optimisation vs v3:
//    • Eliminated obs_compact_ interleaved float array.
//      v4 Obstacle has .lo()/.hi() directly — collision code uses Obstacle*,
//      no redundant shadow copy, no pack_obstacles() overhead on mutate.
//
//  迁移自 v3 scene.h
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/core/types.h"

#include <string>
#include <vector>

namespace sbf {
namespace scene {

class Scene {
public:
    Scene() = default;
    explicit Scene(std::vector<Obstacle> obstacles);

    // ── Obstacle management ──────────────────────────────────────────────
    void add_obstacle(const Obstacle& obs);
    void remove_obstacle(const std::string& name);
    void clear();

    // ── Access ───────────────────────────────────────────────────────────
    int              n_obstacles() const { return static_cast<int>(obstacles_.size()); }
    const Obstacle*  obstacles()   const { return obstacles_.data(); }
    const Obstacle&  operator[](int i) const { return obstacles_[i]; }

private:
    std::vector<Obstacle> obstacles_;
};

} // namespace scene
} // namespace sbf
