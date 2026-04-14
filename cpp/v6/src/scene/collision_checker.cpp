#include <sbf/scene/collision_checker.h>
#include <sbf/core/fk_state.h>

#include <algorithm>
#include <cmath>
#include <cstring>

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

// ── Point FK helpers (standard 4×4 matrix multiply, no interval overhead) ───
namespace {

/// Standard 4×4 matrix multiply: R = T * A  (both row-major [16]).
/// Only computes the upper 3×4 block; row 3 is fixed to [0,0,0,1].
inline void mat4_mul_point(const double T[16], const double A[16],
                           double R[16]) {
    for (int i = 0; i < 3; ++i) {
        const double* Ti = T + i * 4;
        for (int j = 0; j < 3; ++j)
            R[i * 4 + j] = Ti[0] * A[j] + Ti[1] * A[4 + j] + Ti[2] * A[8 + j];
        R[i * 4 + 3] = Ti[0] * A[3] + Ti[1] * A[7] + Ti[2] * A[11] + Ti[3];
    }
    R[12] = 0.0; R[13] = 0.0; R[14] = 0.0; R[15] = 1.0;
}

/// Build point DH matrix A (no intervals, no min/max).
inline void build_dh_point(double alpha, double a,
                           double ct, double st, double d,
                           double A[16]) {
    double ca = std::cos(alpha), sa = std::sin(alpha);
    A[0]  = ct;      A[1]  = -st;      A[2]  = 0.0;  A[3]  = a;
    A[4]  = st * ca;  A[5]  = ct * ca;  A[6]  = -sa;  A[7]  = -d * sa;
    A[8]  = st * sa;  A[9]  = ct * sa;  A[10] = ca;   A[11] = d * ca;
    A[12] = 0.0;      A[13] = 0.0;      A[14] = 0.0;  A[15] = 1.0;
}

}  // anonymous namespace

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
    const int n = robot_->n_joints();
    const auto& dh = robot_->dh_params();

    // ── Point FK: standard 4×4 matrix multiply (no interval overhead) ──
    double prefix[MAX_TF][16];

    // Identity prefix[0]
    std::memset(prefix[0], 0, 16 * sizeof(double));
    prefix[0][0] = prefix[0][5] = prefix[0][10] = prefix[0][15] = 1.0;

    double A[16];
    for (int i = 0; i < n; ++i) {
        double theta = (dh[i].joint_type == 0) ? q[i] + dh[i].theta : dh[i].theta;
        double d_val = (dh[i].joint_type == 0) ? dh[i].d : q[i] + dh[i].d;
        build_dh_point(dh[i].alpha, dh[i].a,
                       std::cos(theta), std::sin(theta), d_val, A);
        mat4_mul_point(prefix[i], A, prefix[i + 1]);
    }

    if (robot_->has_tool()) {
        const auto& tool = *robot_->tool_frame();
        build_dh_point(tool.alpha, tool.a,
                       std::cos(tool.theta), std::sin(tool.theta), tool.d, A);
        mat4_mul_point(prefix[n], A, prefix[n + 1]);
    }

    // ── Check each active link (capsule) vs obstacles ──
    // Use segment-vs-inflated-box test instead of AABB-of-capsule:
    //   Expand obstacle box by link radius, then test line segment intersection.
    //   This is MUCH tighter than bounding the capsule with an AABB.
    const int n_active = robot_->n_active_links();
    const int* active_map = robot_->active_link_map();
    const double* radii = robot_->active_link_radii();
    const int nobs = n_obs();

    for (int i = 0; i < n_active; ++i) {
        int li = active_map[i];
        double ax = prefix[li][3],     ay = prefix[li][7],     az = prefix[li][11];
        double bx = prefix[li + 1][3], by = prefix[li + 1][7], bz = prefix[li + 1][11];
        double r = (radii) ? radii[i] : 0.0;

        double dx = bx - ax, dy = by - ay, dz = bz - az;

        for (int oi = 0; oi < nobs; ++oi) {
            const float* o = obs_compact_.data() + oi * 6;
            // obs_compact: [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]
            // Expand obstacle by capsule radius
            const double lo[3] = {(double)o[0] - r, (double)o[2] - r, (double)o[4] - r};
            const double hi[3] = {(double)o[1] + r, (double)o[3] + r, (double)o[5] + r};
            const double dir[3]  = {dx, dy, dz};
            const double orig[3] = {ax, ay, az};

            double t_enter = 0.0, t_exit = 1.0;
            bool hit = true;
            for (int axis = 0; axis < 3; ++axis) {
                if (std::abs(dir[axis]) < 1e-15) {
                    if (orig[axis] < lo[axis] || orig[axis] > hi[axis])
                        { hit = false; break; }
                } else {
                    double inv = 1.0 / dir[axis];
                    double t1 = (lo[axis] - orig[axis]) * inv;
                    double t2 = (hi[axis] - orig[axis]) * inv;
                    if (t1 > t2) std::swap(t1, t2);
                    t_enter = std::max(t_enter, t1);
                    t_exit  = std::min(t_exit,  t2);
                    if (t_enter > t_exit) { hit = false; break; }
                }
            }
            if (hit) return true;
        }
    }
    return false;
}

bool CollisionChecker::check_box(const std::vector<Interval>& intervals) const {
    FKState state = compute_fk_full(*robot_, intervals);

    int n_active = robot_->n_active_links();
    float aabb[MAX_TF * 6];  // stack-allocated
    extract_link_aabbs(state, robot_->active_link_map(), n_active,
                       aabb, robot_->active_link_radii());

    return aabbs_collide_obs(aabb, n_active,
                             obs_compact_.data(), n_obs());
}

bool CollisionChecker::check_segment(const Eigen::VectorXd& a,
                                     const Eigen::VectorXd& b,
                                     int resolution) const {
    const int n = static_cast<int>(a.size());
    const Eigen::VectorXd diff = b - a;  // compute once
    Eigen::VectorXd q(n);                // allocate once
    for (int i = 0; i <= resolution; ++i) {
        double t = static_cast<double>(i) / resolution;
        q.noalias() = a + t * diff;
        if (check_config(q)) return true;
    }
    return false;
}

}  // namespace sbf
