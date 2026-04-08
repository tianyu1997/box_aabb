#include <sbf/scene/collision_checker.h>
#include <sbf/core/fk_state.h>

#include <algorithm>
#include <cmath>

namespace sbf {

bool aabbs_collide_obs(const float* aabb, int n_slots,
                       const float* obs_compact, int n_obs) {
    for (int si = 0; si < n_slots; ++si) {
        int off = si * 6;
        float a_lo_x = aabb[off],     a_lo_y = aabb[off + 1], a_lo_z = aabb[off + 2];
        float a_hi_x = aabb[off + 3], a_hi_y = aabb[off + 4], a_hi_z = aabb[off + 5];

        for (int oi = 0; oi < n_obs; ++oi) {
            const float* o = obs_compact + oi * 6;
            constexpr float eps = 1e-10f;
            if (a_hi_x < o[0] - eps || a_lo_x > o[1] + eps) continue;
            if (a_hi_y < o[2] - eps || a_lo_y > o[3] + eps) continue;
            if (a_hi_z < o[4] - eps || a_lo_z > o[5] + eps) continue;
            return true;
        }
    }
    return false;
}

CollisionChecker::CollisionChecker(const Robot& robot,
                                   const std::vector<Obstacle>& obstacles)
    : robot_(&robot), obstacles_(obstacles)
{
    pack_obstacles();
}

void CollisionChecker::set_obstacles(const Obstacle* obs, int n_obs) {
    obstacles_.assign(obs, obs + n_obs);
    pack_obstacles();
}

void CollisionChecker::pack_obstacles() {
    int n = static_cast<int>(obstacles_.size());
    obs_compact_.resize(n * 6);
    for (int i = 0; i < n; ++i) {
        const auto& b = obstacles_[i].bounds;
        float* p = obs_compact_.data() + i * 6;
        // Input bounds: [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
        // Compact: [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]
        p[0] = b[0]; p[1] = b[3];  // x
        p[2] = b[1]; p[3] = b[4];  // y
        p[4] = b[2]; p[5] = b[5];  // z
    }
}

bool CollisionChecker::check_config(const Eigen::VectorXd& q) const {
    int n = robot_->n_joints();
    std::vector<Interval> intervals(n);
    for (int i = 0; i < n; ++i)
        intervals[i] = {q[i], q[i]};

    return check_box(intervals);
}

bool CollisionChecker::check_box(const std::vector<Interval>& intervals) const {
    FKState state = compute_fk_full(*robot_, intervals);

    int n_active = robot_->n_active_links();
    std::vector<float> aabb(n_active * 6);
    extract_link_aabbs(state, robot_->active_link_map(), n_active,
                       aabb.data(), robot_->active_link_radii());

    return aabbs_collide_obs(aabb.data(), n_active,
                             obs_compact_.data(), n_obs());
}

bool CollisionChecker::check_segment(const Eigen::VectorXd& a,
                                     const Eigen::VectorXd& b,
                                     int resolution) const {
    for (int i = 0; i <= resolution; ++i) {
        double t = static_cast<double>(i) / resolution;
        Eigen::VectorXd q = a + t * (b - a);
        if (check_config(q)) return true;
    }
    return false;
}

}  // namespace sbf
