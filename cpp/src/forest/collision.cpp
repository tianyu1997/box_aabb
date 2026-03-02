// SafeBoxForest — Collision detection implementation (SAT)
#include "sbf/forest/collision.h"
#include "sbf/aabb/interval_fk.h"
#include "sbf/aabb/fk_scalar.h"
#include <cmath>

namespace sbf {

// ─── AABB-vs-compact-obstacles collision ─────────────────────────────────────
// Outer loop: AABB slots (values cached in registers)
// Inner loop: obstacles (sequential scan, cache-friendly)
bool aabbs_collide_obs(const float* aabb, int n_slots,
                       const float* obs_compact, int n_obs) {
    for (int si = 0; si < n_slots; ++si) {
        int off = si * 6;
        float a_lo_x = aabb[off],     a_lo_y = aabb[off + 1], a_lo_z = aabb[off + 2];
        float a_hi_x = aabb[off + 3], a_hi_y = aabb[off + 4], a_hi_z = aabb[off + 5];

        for (int oi = 0; oi < n_obs; ++oi) {
            const float* o = obs_compact + oi * 6;
            // obs_compact format: [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]
            constexpr float eps = 1e-10f;
            if (a_hi_x < o[0] - eps || a_lo_x > o[1] + eps) continue;
            if (a_hi_y < o[2] - eps || a_lo_y > o[3] + eps) continue;
            if (a_hi_z < o[4] - eps || a_lo_z > o[5] + eps) continue;
            return true;
        }
    }
    return false;
}

// ─── CollisionChecker ───────────────────────────────────────────────────────
CollisionChecker::CollisionChecker(const Robot& robot,
                                   const std::vector<Obstacle>& obstacles)
    : robot_(&robot), obstacles_(obstacles)
{
    pack_obstacles();
}

void CollisionChecker::pack_obstacles() {
    // Compact obstacle array: 6 floats per obstacle [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]
    int n = static_cast<int>(obstacles_.size());
    obs_compact_.resize(n * 6);
    for (int i = 0; i < n; ++i) {
        auto lo = obstacles_[i].lo();
        auto hi = obstacles_[i].hi();
        float* p = obs_compact_.data() + i * 6;
        p[0] = static_cast<float>(lo.x()); p[1] = static_cast<float>(hi.x());
        p[2] = static_cast<float>(lo.y()); p[3] = static_cast<float>(hi.y());
        p[4] = static_cast<float>(lo.z()); p[5] = static_cast<float>(hi.z());
    }
}

int CollisionChecker::n_aabb_slots() const {
    return robot_->n_active_links() + robot_->n_ee_aabb_slots();
}

bool CollisionChecker::check_config(const Eigen::VectorXd& q) const {
    ++n_checks_;

    // Compute scalar FK → full transforms (needed for EE spheres)
    auto transforms = fk_transforms(*robot_, q);
    int n_links = robot_->n_active_links();
    const double* radii = robot_->active_link_radii();

    // Extract positions from transforms
    std::vector<Eigen::Vector3d> positions;
    positions.reserve(transforms.size());
    for (auto& T : transforms)
        positions.push_back(T.block<3, 1>(0, 3));

    for (size_t oi = 0; oi < obstacles_.size(); ++oi) {
        const auto& obs = obstacles_[oi];
        // Check link segments
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

        // Check EE collision spheres
        if (robot_->has_ee_spheres()) {
            int fi = robot_->ee_spheres_frame();
            if (fi < static_cast<int>(transforms.size())) {
                const auto& T_frame = transforms[fi];
                Eigen::Matrix3d R = T_frame.block<3, 3>(0, 0);
                Eigen::Vector3d t = T_frame.block<3, 1>(0, 3);

                for (int sk = 0; sk < robot_->n_ee_spheres(); ++sk) {
                    const auto& sp = robot_->ee_spheres()[sk];
                    Eigen::Vector3d c_local(sp.center[0], sp.center[1], sp.center[2]);
                    Eigen::Vector3d c_world = R * c_local + t;
                    double rk = sp.radius;

                    Eigen::Vector3d s_lo = c_world.array() - rk;
                    Eigen::Vector3d s_hi = c_world.array() + rk;

                    auto obs_lo = obs.lo();
                    auto obs_hi = obs.hi();
                    if (s_hi.x() < obs_lo.x() || s_lo.x() > obs_hi.x()) continue;
                    if (s_hi.y() < obs_lo.y() || s_lo.y() > obs_hi.y()) continue;
                    if (s_hi.z() < obs_lo.z() || s_lo.z() > obs_hi.z()) continue;
                    return true;
                }
            }
        }
    }
    return false;
}

bool CollisionChecker::check_box(const std::vector<Interval>& intervals) const {
    ++n_checks_;

    // Use interval FK to compute link + EE AABB envelopes
    FKState state = compute_fk_full(*robot_, intervals);

    int n_active = robot_->n_active_links();
    int n_ee_slots = robot_->n_ee_aabb_slots();
    int n_slots = n_active + n_ee_slots;

    std::vector<float> aabb(n_slots * 6);
    // Extract link AABBs inflated by link_radii
    extract_link_aabbs(state, robot_->active_link_map(), n_active,
                       aabb.data(), robot_->active_link_radii());
    if (n_ee_slots > 0) {
        extract_ee_group_aabbs(state, robot_->ee_groups().data(),
                               robot_->n_ee_groups(), robot_->ee_spheres_frame(),
                               aabb.data() + n_active * 6);
    }

    return aabbs_collide_obs(aabb.data(), n_slots, obs_compact_.data(), n_obs());
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
