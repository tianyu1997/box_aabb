#pragma once
/// @file obstacle.h
/// @brief Axis-aligned bounding box obstacle in 3-D workspace.
///
/// Compact 6-float layout: [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z].

namespace sbf::scene {

struct Obstacle {
    float bounds[6] = {};

    Obstacle() = default;
    Obstacle(float lx, float ly, float lz, float hx, float hy, float hz)
        : bounds{lx, ly, lz, hx, hy, hz} {}
};

}  // namespace sbf::scene
