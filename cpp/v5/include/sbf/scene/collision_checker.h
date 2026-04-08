#pragma once
/// @file collision_checker.h
/// @brief Fast AABB-based collision checking for robot configurations and boxes.
///
/// Provides two layers:
///   - `aabbs_collide_obs()` — low-level SAT test between inflated link AABBs
///     and a compact obstacle array.
///   - `CollisionChecker` — high-level class that combines Robot FK with
///     obstacle packing for config, box, and segment collision queries.
///
/// Obstacle data is packed into a compact `[lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]`
/// layout for cache-friendly SIMD-amenable comparison.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>

#include <Eigen/Core>
#include <vector>

namespace sbf {

/// @brief Check inflated link AABBs against compact obstacle array.
/// @param aabb         Link AABBs: [n_slots * 6] = [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z].
/// @param n_slots      Number of link AABB slots.
/// @param obs_compact  Packed obstacles: [n_obs * 6] = [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z].
/// @param n_obs        Number of obstacles.
/// @return true if any link AABB overlaps any obstacle.
bool aabbs_collide_obs(const float* aabb, int n_slots,
                       const float* obs_compact, int n_obs);

/// @brief Robot collision checker with cached obstacle packing.
///
/// Combines Robot FK with packed obstacle data for efficient collision
/// queries at the config, box, and segment levels.
class CollisionChecker {
public:
    CollisionChecker() = default;
    CollisionChecker(const Robot& robot, const std::vector<Obstacle>& obstacles);

    /// Replace the obstacle set (re-packs compact representation).
    void set_obstacles(const Obstacle* obs, int n_obs);

    /// Check if a single configuration is in collision.
    bool check_config(const Eigen::VectorXd& q) const;

    /// Check if a box (interval product) is in collision (conservative).
    bool check_box(const std::vector<Interval>& intervals) const;

    /// Check a linear segment via discrete sampling.
    /// @param resolution  Number of intermediate samples.
    bool check_segment(const Eigen::VectorXd& a, const Eigen::VectorXd& b,
                       int resolution = 10) const;

    const float* obs_compact() const { return obs_compact_.data(); }
    int n_obs() const { return static_cast<int>(obs_compact_.size() / 6); }
    const Obstacle* obstacles() const { return obstacles_.data(); }

    const Robot& robot() const { return *robot_; }

private:
    const Robot* robot_ = nullptr;
    std::vector<Obstacle> obstacles_;
    std::vector<float> obs_compact_;  // [n_obs * 6]: [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]

    void pack_obstacles();
};

}  // namespace sbf
