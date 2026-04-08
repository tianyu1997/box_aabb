// SafeBoxForest — Scene: obstacle collection management
// Module: sbf::scene
#pragma once

#include "sbf/common/types.h"
#include <Eigen/Core>
#include <string>
#include <vector>

namespace sbf {

class Scene {
public:
    Scene() = default;
    explicit Scene(std::vector<Obstacle> obstacles);

    // ── Obstacle management ──────────────────────────────────────────────
    void add_obstacle(const Obstacle& obs);
    void remove_obstacle(const std::string& name);
    void clear();

    // ── Access ───────────────────────────────────────────────────────────
    const std::vector<Obstacle>& obstacles() const { return obstacles_; }
    int n_obstacles() const { return static_cast<int>(obstacles_.size()); }

    // Compact obstacle array [n_obs * 6] in format [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]
    const float* obs_compact() const { return obs_compact_.data(); }
    int obs_compact_size() const { return static_cast<int>(obs_compact_.size()); }

    // Force repack of compact array (called automatically on add/remove)
    void repack();

private:
    std::vector<Obstacle> obstacles_;
    std::vector<float> obs_compact_;

    void pack_obstacles();
};

} // namespace sbf
