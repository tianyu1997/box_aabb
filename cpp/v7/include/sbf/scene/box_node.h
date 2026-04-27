#pragma once
/// @file box_node.h
/// @brief Configuration-space collision-free box (LECT leaf).
///
/// Per architectural decision D1 (Radii=0 cache), `link_iaabbs` stores the
/// **zero-radius** link AABB envelope. Inflation by per-link radii is
/// applied lazily at scene-collision time via
/// `sbf::scene::aabbs_collide_obs_inflated()`.

#include "sbf/core/types.h"

#include <Eigen/Dense>
#include <vector>

namespace sbf::scene {

struct BoxNode {
    int                              id = -1;
    std::vector<sbf::core::Interval> joint_intervals;
    Eigen::VectorXd                  seed_config;
    double                           volume = 0.0;
    int                              tree_id = -1;
    int                              parent_box_id = -1;
    int                              root_id = -1;

    /// Zero-radius link AABBs ([n_active * 6]).
    std::vector<float>               link_iaabbs;

    int n_dims() const { return static_cast<int>(joint_intervals.size()); }

    Eigen::VectorXd center() const {
        Eigen::VectorXd c(n_dims());
        for (int d = 0; d < n_dims(); ++d) c[d] = joint_intervals[d].center();
        return c;
    }

    void compute_volume() {
        volume = 1.0;
        for (auto& iv : joint_intervals) volume *= iv.width();
    }

    bool contains(const Eigen::VectorXd& q,
                  double tol = sbf::core::CONTAIN_TOL) const {
        for (int d = 0; d < n_dims(); ++d)
            if (!joint_intervals[d].contains(q[d], tol)) return false;
        return true;
    }
};

}  // namespace sbf::scene
