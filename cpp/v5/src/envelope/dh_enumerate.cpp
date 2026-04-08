// SafeBoxForest v5 — Shared DH enumeration implementation
#include <sbf/envelope/dh_enumerate.h>

namespace sbf {

void enumerate_critical_iterative(
    const Robot& robot,
    const std::vector<std::vector<PreDH>>& pre_dh,
    const std::vector<int>& n_cands,
    const int* active_link_map, int n_active,
    float* out)
{
    const int n = robot.n_joints();

    // Stack of prefix transforms (3×4 each, row 3 implicit [0,0,0,1])
    double stack_T[(MAX_JOINTS + 2) * 12];
    double* T0 = stack_T;
    for (int i = 0; i < 12; ++i) T0[i] = 0.0;
    T0[0] = T0[5] = T0[10] = 1.0;

    double positions[MAX_TF][3];
    positions[0][0] = 0.0; positions[0][1] = 0.0; positions[0][2] = 0.0;

    // Precompute tool DH matrix (constant)
    double tool_A[12] = {};
    bool has_tool = robot.has_tool();
    if (has_tool) {
        const auto& tf = *robot.tool_frame();
        double ct = std::cos(tf.theta), st = std::sin(tf.theta);
        double ca = std::cos(tf.alpha), sa = std::sin(tf.alpha);
        tool_A[0]  = ct;      tool_A[1]  = -st;     tool_A[2]  = 0.0;  tool_A[3]  = tf.a;
        tool_A[4]  = st*ca;   tool_A[5]  = ct*ca;   tool_A[6]  = -sa;  tool_A[7]  = -tf.d*sa;
        tool_A[8]  = st*sa;   tool_A[9]  = ct*sa;   tool_A[10] = ca;   tool_A[11] = tf.d*ca;
    }

    // Counter array
    int idx[MAX_JOINTS];
    for (int j = 0; j < n; ++j) idx[j] = 0;

    // Initialize prefix transforms for idx=[0,0,...,0]
    for (int j = 0; j < n; ++j) {
        const double* Tprev = stack_T + j * 12;
        double* Tnext = stack_T + (j + 1) * 12;
        mul_prefix_dh(Tprev, pre_dh[j][0].A, Tnext);
        positions[j + 1][0] = Tnext[3];
        positions[j + 1][1] = Tnext[7];
        positions[j + 1][2] = Tnext[11];
    }

    // Process first combination
    if (has_tool) {
        double tool_R[12];
        mul_prefix_dh(stack_T + n * 12, tool_A, tool_R);
        positions[n + 1][0] = tool_R[3];
        positions[n + 1][1] = tool_R[7];
        positions[n + 1][2] = tool_R[11];
    }
    update_endpoints_from_positions(positions, active_link_map, n_active, out);

    // Iterate remaining combinations (odometer-style)
    while (true) {
        int carry = n - 1;
        while (carry >= 0) {
            idx[carry]++;
            if (idx[carry] < n_cands[carry]) break;
            idx[carry] = 0;
            carry--;
        }
        if (carry < 0) break;

        for (int j = carry; j < n; ++j) {
            const double* Tprev = stack_T + j * 12;
            double* Tnext = stack_T + (j + 1) * 12;
            mul_prefix_dh(Tprev, pre_dh[j][idx[j]].A, Tnext);
            positions[j + 1][0] = Tnext[3];
            positions[j + 1][1] = Tnext[7];
            positions[j + 1][2] = Tnext[11];
        }

        if (has_tool) {
            double tool_R[12];
            mul_prefix_dh(stack_T + n * 12, tool_A, tool_R);
            positions[n + 1][0] = tool_R[3];
            positions[n + 1][1] = tool_R[7];
            positions[n + 1][2] = tool_R[11];
        }
        update_endpoints_from_positions(positions, active_link_map, n_active, out);
    }
}

}  // namespace sbf
