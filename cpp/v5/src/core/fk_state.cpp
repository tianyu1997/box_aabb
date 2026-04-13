#include <sbf/core/fk_state.h>

#include <algorithm>
#include <cmath>
#include <cstring>

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
                        const Interval* intervals, int /*n_intervals*/) {
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

FKState compute_fk_full(const Robot& robot,
                        const std::vector<Interval>& intervals) {
    return compute_fk_full(robot, intervals.data(),
                           static_cast<int>(intervals.size()));
}

FKState compute_fk_incremental(const FKState& parent,
                               const Robot& robot,
                               const std::vector<Interval>& intervals,
                               int changed_dim) {
    FKState state;
    int n = robot.n_joints();
    state.n_jm = parent.n_jm;
    state.n_tf = parent.n_tf;

    std::memcpy(state.prefix_lo, parent.prefix_lo, sizeof(state.prefix_lo));
    std::memcpy(state.prefix_hi, parent.prefix_hi, sizeof(state.prefix_hi));
    std::memcpy(state.joints_lo, parent.joints_lo, sizeof(state.joints_lo));
    std::memcpy(state.joints_hi, parent.joints_hi, sizeof(state.joints_hi));

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

void update_fk_inplace(FKState& state,
                       const Robot& robot,
                       const std::vector<Interval>& intervals,
                       int changed_dim) {
    int n = robot.n_joints();

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

void extract_endpoint_iaabbs(const FKState& state,
                             const int* active_link_map, int n_active_links,
                             float* out) {
    for (int i = 0; i < n_active_links; ++i) {
        int li = active_link_map[i];

        // Proximal endpoint: prefix[li] translation column
        float* p = out + (i * 2) * 6;
        p[0] = static_cast<float>(state.prefix_lo[li][3]);
        p[1] = static_cast<float>(state.prefix_lo[li][7]);
        p[2] = static_cast<float>(state.prefix_lo[li][11]);
        p[3] = static_cast<float>(state.prefix_hi[li][3]);
        p[4] = static_cast<float>(state.prefix_hi[li][7]);
        p[5] = static_cast<float>(state.prefix_hi[li][11]);

        // Distal endpoint: prefix[li+1] translation column
        float* d = out + (i * 2 + 1) * 6;
        d[0] = static_cast<float>(state.prefix_lo[li + 1][3]);
        d[1] = static_cast<float>(state.prefix_lo[li + 1][7]);
        d[2] = static_cast<float>(state.prefix_lo[li + 1][11]);
        d[3] = static_cast<float>(state.prefix_hi[li + 1][3]);
        d[4] = static_cast<float>(state.prefix_hi[li + 1][7]);
        d[5] = static_cast<float>(state.prefix_hi[li + 1][11]);
    }
}

}  // namespace sbf
