// SafeBoxForest — Collision detection implementation (SAT)
#include "sbf/forest/collision.h"
#include "sbf/aabb/interval_fk.h"
#include "sbf/aabb/fk_scalar.h"
#include <cmath>

namespace sbf {

// ─── Link AABB collision (flat obs format) ──────────────────────────────────
bool link_aabbs_collide_flat(const float* aabb,
                             const float* obs_flat, int n_obs) {
    for (int i = 0; i < n_obs; ++i) {
        const float* obs = obs_flat + i * 7;
        int link_idx = static_cast<int>(obs[0]);
        int off = link_idx * 6;

        float lo_x = obs[1], hi_x = obs[2];
        float lo_y = obs[3], hi_y = obs[4];
        float lo_z = obs[5], hi_z = obs[6];

        // SAT: check 3 axes — if separated on any axis, no collision
        // Use 1e-10 tolerance to match Python v4's _aabb_overlap (eps=1e-10)
        constexpr float eps = 1e-10f;
        if (aabb[off + 3] < lo_x - eps || aabb[off]     > hi_x + eps) continue;
        if (aabb[off + 4] < lo_y - eps || aabb[off + 1] > hi_y + eps) continue;
        if (aabb[off + 5] < lo_z - eps || aabb[off + 2] > hi_z + eps) continue;

        return true;  // overlap on all 3 axes → collision
    }
    return false;
}

// ─── Batch collision ────────────────────────────────────────────────────────
void batch_link_collision(const float* all_aabbs, int n_configs,
                          const float* obs_flat, int n_obs,
                          int n_links, bool* result) {
    int aabb_stride = n_links * 6;
    for (int ci = 0; ci < n_configs; ++ci) {
        if (result[ci]) continue;  // already known to collide
        result[ci] = link_aabbs_collide_flat(all_aabbs + ci * aabb_stride,
                                              obs_flat, n_obs);
    }
}

// ─── CollisionChecker ───────────────────────────────────────────────────────
CollisionChecker::CollisionChecker(const Robot& robot,
                                   const std::vector<Obstacle>& obstacles)
    : robot_(&robot), obstacles_(obstacles)
{
    pack_obstacles();
}

void CollisionChecker::pack_obstacles() {
    packed_obs_.clear();
    obs_flat_.clear();

    // For each obstacle, create entries for each active link using COMPACT index.
    // obs_flat stores compact_index so link_aabbs_collide_flat can directly
    // index into the compact AABB array.
    for (const auto& obs : obstacles_) {
        auto obs_lo = obs.lo();
        auto obs_hi = obs.hi();
        if (obs.link_idx >= 0) {
            // Specific link — find its compact index
            int compact_idx = -1;
            for (int ci = 0; ci < robot_->n_active_links(); ++ci) {
                if (robot_->active_link_map()[ci] == obs.link_idx) {
                    compact_idx = ci;
                    break;
                }
            }
            if (compact_idx < 0) continue;  // link not active, skip
            PackedObstacle po;
            po.link_idx = compact_idx;
            po.lo_x = static_cast<float>(obs_lo.x()); po.hi_x = static_cast<float>(obs_hi.x());
            po.lo_y = static_cast<float>(obs_lo.y()); po.hi_y = static_cast<float>(obs_hi.y());
            po.lo_z = static_cast<float>(obs_lo.z()); po.hi_z = static_cast<float>(obs_hi.z());
            packed_obs_.push_back(po);
        } else {
            // Check against all active links — use compact index directly
            for (int ci = 0; ci < robot_->n_active_links(); ++ci) {
                PackedObstacle po;
                po.link_idx = ci;  // compact index
                po.lo_x = static_cast<float>(obs_lo.x()); po.hi_x = static_cast<float>(obs_hi.x());
                po.lo_y = static_cast<float>(obs_lo.y()); po.hi_y = static_cast<float>(obs_hi.y());
                po.lo_z = static_cast<float>(obs_lo.z()); po.hi_z = static_cast<float>(obs_hi.z());
                packed_obs_.push_back(po);
            }
        }
    }

    // Build flat array — obstacles are pre-inflated by per-link radius so that
    // the tree nodes can store raw (uninflated) AABBs and be reused across
    // different radius settings.
    const double* radii = robot_->active_link_radii();
    obs_flat_.resize(packed_obs_.size() * 7);
    for (size_t i = 0; i < packed_obs_.size(); ++i) {
        auto& po = packed_obs_[i];
        float r = (radii && po.link_idx >= 0)
                      ? static_cast<float>(radii[po.link_idx]) : 0.0f;
        float* p = obs_flat_.data() + i * 7;
        p[0] = static_cast<float>(po.link_idx);
        p[1] = po.lo_x - r; p[2] = po.hi_x + r;
        p[3] = po.lo_y - r; p[4] = po.hi_y + r;
        p[5] = po.lo_z - r; p[6] = po.hi_z + r;
    }
}

bool CollisionChecker::check_config(const Eigen::VectorXd& q) const {
    ++n_checks_;

    // Compute scalar FK → link positions → check each active link segment vs obstacles
    auto positions = fk_link_positions(*robot_, q);
    int n_links = robot_->n_active_links();
    const double* radii = robot_->active_link_radii();

    for (size_t oi = 0; oi < obstacles_.size(); ++oi) {
        const auto& obs = obstacles_[oi];
        for (int ci = 0; ci < n_links; ++ci) {
            int link_start = robot_->active_link_map()[ci];
            int link_end = link_start + 1;
            if (link_end >= static_cast<int>(positions.size())) continue;

            // Link AABB: bounding box of two endpoint positions
            Eigen::Vector3d lo = positions[link_start].cwiseMin(positions[link_end]);
            Eigen::Vector3d hi = positions[link_start].cwiseMax(positions[link_end]);

            // Inflate by link radius (capsule approximation)
            if (radii) {
                double r = radii[ci];
                lo.array() -= r;
                hi.array() += r;
            }

            // SAT test
            auto obs_lo = obs.lo();
            auto obs_hi = obs.hi();
            if (hi.x() < obs_lo.x() || lo.x() > obs_hi.x()) continue;
            if (hi.y() < obs_lo.y() || lo.y() > obs_hi.y()) continue;
            if (hi.z() < obs_lo.z() || lo.z() > obs_hi.z()) continue;
            return true;
        }
    }
    return false;
}

bool CollisionChecker::check_box(const std::vector<Interval>& intervals) const {
    ++n_checks_;

    // Use interval FK to compute link AABB envelopes
    std::vector<float> aabb(robot_->n_active_links() * 6);
    compute_link_aabbs(*robot_, intervals, aabb.data());

    return link_aabbs_collide_flat(aabb.data(), obs_flat_.data(), n_obs());
}

bool CollisionChecker::check_segment(const Eigen::VectorXd& q1,
                                      const Eigen::VectorXd& q2,
                                      double step) const {
    double dist = (q2 - q1).norm();
    if (dist < 1e-12) return check_config(q1);

    int n_steps = std::max(1, static_cast<int>(std::ceil(dist / step)));
    for (int i = 0; i <= n_steps; ++i) {
        double t = static_cast<double>(i) / n_steps;
        Eigen::VectorXd q = q1 + t * (q2 - q1);
        if (check_config(q)) return true;
    }
    return false;
}

} // namespace sbf
