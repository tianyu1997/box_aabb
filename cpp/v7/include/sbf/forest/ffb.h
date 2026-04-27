#pragma once
/// @file ffb.h
/// @brief Find-Free-Box: descend LECT to a collision-free leaf containing seed.

#include "sbf/lect/lect.h"

#include <Eigen/Core>
#include <vector>

namespace sbf::forest {

struct FFBConfig {
    int    max_depth        = 30;
    double deadline_ms      = 0.0;       ///< 0 = unlimited
    bool   seed_known_free  = false;     ///< caller verified seed has no collision
};

struct FFBResult {
    int               node_idx        = -1;   ///< target leaf (-1 on failure)
    std::vector<int>  path;                   ///< root → leaf
    int               fail_code       = 0;    ///< 0=ok 1=occupied 2=max_depth 3=collision_root 4=deadline 5=outside
    int               n_new_nodes     = 0;
    int               n_collide_calls = 0;
    int               n_steps         = 0;

    bool success() const { return fail_code == 0 && node_idx >= 0; }
};

FFBResult find_free_box(
    sbf::lect::LECT& lect,
    const Eigen::VectorXd& seed,
    const float* obs_compact, int n_obs,
    const FFBConfig& cfg = {});

}  // namespace sbf::forest
