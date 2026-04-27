/// @file experiments/baseline_collision.h
/// @brief Per-configuration collision check shared by baseline binaries.
///
/// Builds zero-radius link AABBs from interval FK at q (degenerate
/// interval [q, q]) and tests them against obstacles via
/// `sbf::scene::aabbs_collide_obs_inflated`. Apples-to-apples with the
/// SBF collision model (LinkIAABB envelope), so baselines and SBF use
/// the same definition of "free configuration".
#pragma once

#include "sbf/core/fk_state.h"
#include "sbf/core/robot.h"
#include "sbf/scene/collision.h"

#include <Eigen/Core>
#include <vector>

namespace sbf::exp {

/// State carried per worker — pre-allocates scratch buffers so the
/// validity-check inner loop allocates nothing.
class QFreeChecker {
public:
    QFreeChecker(const sbf::core::Robot& robot,
                 const float* obs_compact,
                 int          n_obs)
        : robot_(robot), obs_(obs_compact), n_obs_(n_obs) {
        n_active_  = robot.n_active_links();
        intervals_.reserve(robot.n_joints());
        link_aabb_.assign(n_active_ * 6, 0.0f);
        if (robot.has_link_radii()) {
            link_radii_.assign(robot.active_link_radii(),
                               robot.active_link_radii() + n_active_);
        } else {
            link_radii_.assign(n_active_, 0.0f);
        }
    }

    bool is_free(const Eigen::VectorXd& q) {
        intervals_.clear();
        for (int i = 0; i < q.size(); ++i)
            intervals_.emplace_back(q[i], q[i]);
        auto fk = sbf::core::compute_fk_full(robot_, intervals_);
        sbf::core::extract_link_aabbs(
            fk, robot_.active_link_map(), n_active_, link_aabb_.data());
        return !sbf::scene::aabbs_collide_obs_inflated(
            link_aabb_.data(), n_active_,
            link_radii_.data(),
            obs_, n_obs_);
    }

    /// Collision-checked motion validation by interpolation step `dt`.
    bool segment_free(const Eigen::VectorXd& a,
                      const Eigen::VectorXd& b,
                      double                 dt = 0.05) {
        double L = (b - a).norm();
        int    n = std::max(1, static_cast<int>(std::ceil(L / dt)));
        for (int i = 1; i < n; ++i) {
            Eigen::VectorXd q = a + (b - a) * (static_cast<double>(i) / n);
            if (!is_free(q)) return false;
        }
        return true;
    }

private:
    const sbf::core::Robot&         robot_;
    const float*                    obs_;
    int                             n_obs_;
    int                             n_active_;
    std::vector<sbf::core::Interval> intervals_;
    std::vector<float>              link_aabb_;
    std::vector<float>              link_radii_;
};

}  // namespace sbf::exp
