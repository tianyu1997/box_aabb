// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Interval FK implementation
//  迁移自 v3 interval_fk.cpp
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/robot/interval_fk.h"
#include <cstring>
#include <algorithm>

namespace sbf {

void build_joint_interval(const Robot& robot, int joint_idx,
                          const Interval& iv,
                          double A_lo[16], double A_hi[16]) {
    const auto& dh = robot.dh_params()[joint_idx];

    double ct_lo, ct_hi, st_lo, st_hi;
    double d_lo, d_hi;

    if (dh.joint_type == 0) {
        double theta_lo = iv.lo + dh.theta;
        double theta_hi = iv.hi + dh.theta;
        auto ct = I_cos(theta_lo, theta_hi);
        auto st = I_sin(theta_lo, theta_hi);
        ct_lo = ct.lo; ct_hi = ct.hi;
        st_lo = st.lo; st_hi = st.hi;
        d_lo = d_hi = dh.d;
    } else {
        double theta = dh.theta;
        ct_lo = ct_hi = std::cos(theta);
        st_lo = st_hi = std::sin(theta);
        d_lo = iv.lo + dh.d;
        d_hi = iv.hi + dh.d;
    }

    build_dh_joint(dh.alpha, dh.a, ct_lo, ct_hi, st_lo, st_hi, d_lo, d_hi,
                   A_lo, A_hi);
}

FKState compute_fk_full(const Robot& robot,
                        const std::vector<Interval>& intervals) {
    FKState state;
    int n = robot.n_joints();
    state.n_jm = n + (robot.has_tool() ? 1 : 0);
    state.n_tf = n + 1 + (robot.has_tool() ? 1 : 0);

    imat_identity(state.prefix_lo[0], state.prefix_hi[0]);

    for (int i = 0; i < n; ++i) {
        build_joint_interval(robot, i, intervals[i],
                             state.joints_lo[i], state.joints_hi[i]);
        imat_mul_dh(state.prefix_lo[i], state.prefix_hi[i],
                    state.joints_lo[i], state.joints_hi[i],
                    state.prefix_lo[i + 1], state.prefix_hi[i + 1]);
    }

    if (robot.has_tool()) {
        const auto& tool = *robot.tool_frame();
        double ct = std::cos(tool.theta);
        double st = std::sin(tool.theta);
        build_dh_joint(tool.alpha, tool.a, ct, ct, st, st, tool.d, tool.d,
                       state.joints_lo[n], state.joints_hi[n]);
        imat_mul_dh(state.prefix_lo[n], state.prefix_hi[n],
                    state.joints_lo[n], state.joints_hi[n],
                    state.prefix_lo[n + 1], state.prefix_hi[n + 1]);
    }

    state.valid = true;
    return state;
}

FKState compute_fk_incremental(const FKState& parent,
                               const Robot& robot,
                               const std::vector<Interval>& intervals,
                               int changed_dim) {
    FKState state;
    int n = robot.n_joints();
    state.n_jm = parent.n_jm;
    state.n_tf = parent.n_tf;

    std::memcpy(state.prefix_lo, parent.prefix_lo, sizeof(double) * MAX_TF * 16);
    std::memcpy(state.prefix_hi, parent.prefix_hi, sizeof(double) * MAX_TF * 16);
    std::memcpy(state.joints_lo, parent.joints_lo, sizeof(double) * MAX_JOINTS * 16);
    std::memcpy(state.joints_hi, parent.joints_hi, sizeof(double) * MAX_JOINTS * 16);

    build_joint_interval(robot, changed_dim, intervals[changed_dim],
                         state.joints_lo[changed_dim],
                         state.joints_hi[changed_dim]);

    imat_mul_dh(state.prefix_lo[changed_dim], state.prefix_hi[changed_dim],
                state.joints_lo[changed_dim], state.joints_hi[changed_dim],
                state.prefix_lo[changed_dim + 1], state.prefix_hi[changed_dim + 1]);

    for (int k = changed_dim + 1; k < n; ++k) {
        imat_mul_dh(state.prefix_lo[k], state.prefix_hi[k],
                    state.joints_lo[k], state.joints_hi[k],
                    state.prefix_lo[k + 1], state.prefix_hi[k + 1]);
    }

    if (robot.has_tool()) {
        imat_mul_dh(state.prefix_lo[n], state.prefix_hi[n],
                    state.joints_lo[n], state.joints_hi[n],
                    state.prefix_lo[n + 1], state.prefix_hi[n + 1]);
    }

    state.valid = true;
    return state;
}

void extract_link_aabbs(const FKState& state,
                        const int* active_link_map, int n_active_links,
                        float* out_aabb,
                        const double* link_radii) {
    for (int i = 0; i < n_active_links; ++i) {
        int li = active_link_map[i];
        const double* start_lo = state.prefix_lo[li];
        const double* start_hi = state.prefix_hi[li];
        const double* end_lo   = state.prefix_lo[li + 1];
        const double* end_hi   = state.prefix_hi[li + 1];

        float* out = out_aabb + i * 6;
        out[0] = static_cast<float>(std::min(start_lo[3],  end_lo[3]));
        out[1] = static_cast<float>(std::min(start_lo[7],  end_lo[7]));
        out[2] = static_cast<float>(std::min(start_lo[11], end_lo[11]));
        out[3] = static_cast<float>(std::max(start_hi[3],  end_hi[3]));
        out[4] = static_cast<float>(std::max(start_hi[7],  end_hi[7]));
        out[5] = static_cast<float>(std::max(start_hi[11], end_hi[11]));

        if (link_radii) {
            float r = static_cast<float>(link_radii[i]);
            out[0] -= r; out[1] -= r; out[2] -= r;
            out[3] += r; out[4] += r; out[5] += r;
        }
    }
}

void compute_link_aabbs(const Robot& robot,
                        const std::vector<Interval>& intervals,
                        float* out_aabb) {
    FKState state = compute_fk_full(robot, intervals);
    extract_link_aabbs(state, robot.active_link_map(), robot.n_active_links(),
                       out_aabb, nullptr);
}

void extract_ee_sphere_aabbs(const FKState& state,
                             const EESphere* spheres, int n_spheres,
                             int frame_index,
                             float* out_aabb) {
    const double* T_lo = state.prefix_lo[frame_index];
    const double* T_hi = state.prefix_hi[frame_index];

    for (int k = 0; k < n_spheres; ++k) {
        const double* c = spheres[k].center;
        double r = spheres[k].radius;

        double w_lo[3], w_hi[3];
        for (int i = 0; i < 3; ++i) {
            w_lo[i] = T_lo[i * 4 + 3];
            w_hi[i] = T_hi[i * 4 + 3];
        }

        for (int j = 0; j < 3; ++j) {
            double cj = c[j];
            if (std::abs(cj) < 1e-15) continue;
            for (int i = 0; i < 3; ++i) {
                double a = T_lo[i * 4 + j];
                double b = T_hi[i * 4 + j];
                if (cj >= 0.0) {
                    w_lo[i] += a * cj;
                    w_hi[i] += b * cj;
                } else {
                    w_lo[i] += b * cj;
                    w_hi[i] += a * cj;
                }
            }
        }

        float* out = out_aabb + k * 6;
        out[0] = static_cast<float>(w_lo[0] - r);
        out[1] = static_cast<float>(w_lo[1] - r);
        out[2] = static_cast<float>(w_lo[2] - r);
        out[3] = static_cast<float>(w_hi[0] + r);
        out[4] = static_cast<float>(w_hi[1] + r);
        out[5] = static_cast<float>(w_hi[2] + r);
    }
}

void extract_ee_group_aabbs(const FKState& state,
                            const EEGroup* groups, int n_groups,
                            int frame_index,
                            float* out_aabb) {
    const double* T_lo = state.prefix_lo[frame_index];
    const double* T_hi = state.prefix_hi[frame_index];

    for (int gi = 0; gi < n_groups; ++gi) {
        float* gout = out_aabb + gi * 6;
        gout[0] = gout[1] = gout[2] =  1e30f;
        gout[3] = gout[4] = gout[5] = -1e30f;

        const auto& grp = groups[gi];
        for (int ki = 0; ki < grp.n_keys; ++ki) {
            const double* c = grp.keys[ki].center;
            double r = grp.keys[ki].coverage_radius;

            double w_lo[3], w_hi[3];
            for (int i = 0; i < 3; ++i) {
                w_lo[i] = T_lo[i * 4 + 3];
                w_hi[i] = T_hi[i * 4 + 3];
            }
            for (int j = 0; j < 3; ++j) {
                double cj = c[j];
                if (std::abs(cj) < 1e-15) continue;
                for (int i = 0; i < 3; ++i) {
                    double a = T_lo[i * 4 + j];
                    double b = T_hi[i * 4 + j];
                    if (cj >= 0.0) {
                        w_lo[i] += a * cj;
                        w_hi[i] += b * cj;
                    } else {
                        w_lo[i] += b * cj;
                        w_hi[i] += a * cj;
                    }
                }
            }

            float klo0 = static_cast<float>(w_lo[0] - r);
            float klo1 = static_cast<float>(w_lo[1] - r);
            float klo2 = static_cast<float>(w_lo[2] - r);
            float khi0 = static_cast<float>(w_hi[0] + r);
            float khi1 = static_cast<float>(w_hi[1] + r);
            float khi2 = static_cast<float>(w_hi[2] + r);

            if (klo0 < gout[0]) gout[0] = klo0;
            if (klo1 < gout[1]) gout[1] = klo1;
            if (klo2 < gout[2]) gout[2] = klo2;
            if (khi0 > gout[3]) gout[3] = khi0;
            if (khi1 > gout[4]) gout[4] = khi1;
            if (khi2 > gout[5]) gout[5] = khi2;
        }
    }
}

void compute_all_aabbs(const Robot& robot,
                       const std::vector<Interval>& intervals,
                       float* out_link_aabb,
                       float* out_ee_aabb) {
    FKState state = compute_fk_full(robot, intervals);
    extract_link_aabbs(state, robot.active_link_map(), robot.n_active_links(),
                       out_link_aabb, nullptr);
    if (robot.has_ee_spheres()) {
        extract_ee_sphere_aabbs(state, robot.ee_spheres().data(),
                                robot.n_ee_spheres(),
                                robot.ee_spheres_frame(),
                                out_ee_aabb);
    }
}

} // namespace sbf
