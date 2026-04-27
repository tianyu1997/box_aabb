#pragma once
/// @file endpoint_sources_profile.h
/// @brief Endpoint-source helpers used by the v7 profiling experiments.

#include "sbf/core/endpoint_iaabb.h"
#include "sbf/core/fk_state.h"
#include "sbf/core/robot.h"

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <string>
#include <vector>

namespace sbf::exp::endpoint_profile {

using sbf::core::Interval;
using sbf::core::Robot;

struct EndpointResult {
    std::vector<float> endpoint_iaabbs;
    std::string        source;
    int                n_active_links = 0;
    bool               certified = false;
    int                gcpc_hits = 0;
};

inline void init_endpoints_inf(float* out, int n_active) {
    for (int i = 0; i < n_active * 2; ++i) {
        float* a = out + i * 6;
        a[0] = a[1] = a[2] =  std::numeric_limits<float>::infinity();
        a[3] = a[4] = a[5] = -std::numeric_limits<float>::infinity();
    }
}

inline void hull_endpoint_iaabbs(float* dst, const float* src, int n_endpoints) {
    for (int i = 0; i < n_endpoints; ++i) {
        const float* s = src + i * 6;
        float*       d = dst + i * 6;
        for (int k = 0; k < 3; ++k) d[k] = std::min(d[k], s[k]);
        for (int k = 3; k < 6; ++k) d[k] = std::max(d[k], s[k]);
    }
}

inline double endpoint_volume(const std::vector<float>& ep) {
    double v = 0.0;
    for (std::size_t i = 0; i < ep.size(); i += 6) {
        const double dx = std::max(0.0f, ep[i + 3] - ep[i + 0]);
        const double dy = std::max(0.0f, ep[i + 4] - ep[i + 1]);
        const double dz = std::max(0.0f, ep[i + 5] - ep[i + 2]);
        v += dx * dy * dz;
    }
    return v;
}

inline double max_negative_gap_to(const std::vector<float>& src,
                                  const std::vector<float>& baseline) {
    double worst = 0.0;
    const std::size_t n = std::min(src.size(), baseline.size());
    for (std::size_t i = 0; i + 5 < n; i += 6) {
        for (int d = 0; d < 3; ++d) {
            if (src[i + d] > baseline[i + d]) {
                worst = std::min(worst,
                                 static_cast<double>(baseline[i + d] - src[i + d]));
            }
            if (src[i + d + 3] < baseline[i + d + 3]) {
                worst = std::min(worst,
                                 static_cast<double>(src[i + d + 3] - baseline[i + d + 3]));
            }
        }
    }
    return worst;
}

inline void update_endpoints_from_positions(const double positions[][3],
                                            const int* active_link_map,
                                            int n_active,
                                            float* out) {
    for (int ci = 0; ci < n_active; ++ci) {
        const int v = active_link_map[ci];
        float* prox = out + (ci * 2) * 6;
        float* dist = out + (ci * 2 + 1) * 6;
        for (int d = 0; d < 3; ++d) {
            const float pv = static_cast<float>(positions[v][d]);
            prox[d] = std::min(prox[d], pv);
            prox[d + 3] = std::max(prox[d + 3], pv);
            const float dv = static_cast<float>(positions[v + 1][d]);
            dist[d] = std::min(dist[d], dv);
            dist[d + 3] = std::max(dist[d + 3], dv);
        }
    }
}

inline void scalar_fk_positions(const Robot& robot, const double* q,
                                double positions[][3]) {
    const auto& dh = robot.dh_params();
    const int n = robot.n_joints();

    double T[16]{};
    T[0] = T[5] = T[10] = T[15] = 1.0;
    positions[0][0] = 0.0;
    positions[0][1] = 0.0;
    positions[0][2] = 0.0;

    for (int j = 0; j < n; ++j) {
        const double alpha = dh[j].alpha;
        const double a = dh[j].a;
        const double d_val = (dh[j].joint_type == 1) ? (q[j] + dh[j].d) : dh[j].d;
        const double angle = (dh[j].joint_type == 0) ? (q[j] + dh[j].theta) : dh[j].theta;
        const double ct = std::cos(angle), st = std::sin(angle);
        const double ca = std::cos(alpha), sa = std::sin(alpha);
        const double A[16] = {
            ct,      -st,      0.0,  a,
            st * ca,  ct * ca, -sa,  -d_val * sa,
            st * sa,  ct * sa,  ca,   d_val * ca,
            0.0,      0.0,     0.0,  1.0
        };
        double R[16]{};
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 4; ++c) {
                R[r * 4 + c] = T[r * 4 + 0] * A[c]
                             + T[r * 4 + 1] * A[4 + c]
                             + T[r * 4 + 2] * A[8 + c]
                             + T[r * 4 + 3] * A[12 + c];
            }
        }
        R[12] = 0.0; R[13] = 0.0; R[14] = 0.0; R[15] = 1.0;
        std::copy(std::begin(R), std::end(R), std::begin(T));
        positions[j + 1][0] = T[3];
        positions[j + 1][1] = T[7];
        positions[j + 1][2] = T[11];
    }

    if (robot.has_tool()) {
        const auto& tf = *robot.tool_frame();
        const double ct = std::cos(tf.theta), st = std::sin(tf.theta);
        const double ca = std::cos(tf.alpha), sa = std::sin(tf.alpha);
        const double A[16] = {
            ct,      -st,      0.0,  tf.a,
            st * ca,  ct * ca, -sa,  -tf.d * sa,
            st * sa,  ct * sa,  ca,   tf.d * ca,
            0.0,      0.0,     0.0,  1.0
        };
        double R[16]{};
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 4; ++c) {
                R[r * 4 + c] = T[r * 4 + 0] * A[c]
                             + T[r * 4 + 1] * A[4 + c]
                             + T[r * 4 + 2] * A[8 + c]
                             + T[r * 4 + 3] * A[12 + c];
            }
        }
        positions[n + 1][0] = R[3];
        positions[n + 1][1] = R[7];
        positions[n + 1][2] = R[11];
    }
}

inline EndpointResult endpoint_ifk(const Robot& robot,
                                   const std::vector<Interval>& intervals) {
    auto res = sbf::core::compute_endpoint_iaabb_ifk(robot, intervals);
    return EndpointResult{res.endpoint_iaabbs, "IFK", res.n_active_links, true, 0};
}

inline EndpointResult endpoint_mc(const Robot& robot,
                                  const std::vector<Interval>& intervals,
                                  int n_samples,
                                  std::uint64_t seed) {
    EndpointResult result;
    result.source = "MC";
    result.certified = false;
    result.n_active_links = robot.n_active_links();
    result.endpoint_iaabbs.resize(static_cast<std::size_t>(result.n_active_links) * 2 * 6);
    init_endpoints_inf(result.endpoint_iaabbs.data(), result.n_active_links);

    n_samples = std::max(1, n_samples);
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::vector<double> q(robot.n_joints());
    double positions[sbf::core::MAX_TF][3];

    for (int s = 0; s < n_samples; ++s) {
        for (int j = 0; j < robot.n_joints(); ++j) {
            q[j] = intervals[j].lo + u01(rng) * (intervals[j].hi - intervals[j].lo);
        }
        scalar_fk_positions(robot, q.data(), positions);
        update_endpoints_from_positions(positions, robot.active_link_map(),
                                        result.n_active_links,
                                        result.endpoint_iaabbs.data());
    }
    return result;
}

static constexpr double kNarrowThreshold = 0.01;

struct PreDH {
    double A[12];
};

inline void build_dh_matrix(const sbf::core::DHParam& dh,
                            double q_val,
                            double A[12]) {
    const double d_val = (dh.joint_type == 1) ? (q_val + dh.d) : dh.d;
    const double angle = (dh.joint_type == 0) ? (q_val + dh.theta) : dh.theta;
    const double ct = std::cos(angle), st = std::sin(angle);
    const double ca = std::cos(dh.alpha), sa = std::sin(dh.alpha);
    A[0]  = ct;      A[1]  = -st;     A[2]  = 0.0;  A[3]  = dh.a;
    A[4]  = st * ca; A[5]  = ct * ca; A[6]  = -sa;  A[7]  = -d_val * sa;
    A[8]  = st * sa; A[9]  = ct * sa; A[10] = ca;   A[11] = d_val * ca;
}

inline void mul_prefix_dh(const double Tprev[12], const double A[12],
                          double Tnext[12]) {
    for (int r = 0; r < 3; ++r) {
        const double* tp = Tprev + r * 4;
        double* tn = Tnext + r * 4;
        for (int c = 0; c < 4; ++c) {
            tn[c] = tp[0] * A[c] + tp[1] * A[4 + c] + tp[2] * A[8 + c];
        }
        tn[3] += tp[3];
    }
}

inline void collect_kpi2(double lo, double hi, std::vector<double>& out) {
    if (hi - lo < kNarrowThreshold) {
        out.push_back(0.5 * (lo + hi));
        return;
    }
    out.push_back(lo);
    out.push_back(hi);
    const double k_lo = std::ceil(lo / sbf::core::HALF_PI);
    const double k_hi = std::floor(hi / sbf::core::HALF_PI);
    for (double k = k_lo; k <= k_hi; k += 1.0) {
        const double v = k * sbf::core::HALF_PI;
        if (v > lo + 1e-12 && v < hi - 1e-12) out.push_back(v);
    }
}

inline void enumerate_critical_iterative(
    const Robot& robot,
    const std::vector<std::vector<PreDH>>& pre_dh,
    const std::vector<int>& n_cands,
    const int* active_link_map,
    int n_active,
    float* out) {
    const int n = robot.n_joints();
    double stack_T[(sbf::core::MAX_JOINTS + 2) * 12];
    double* T0 = stack_T;
    for (int i = 0; i < 12; ++i) T0[i] = 0.0;
    T0[0] = T0[5] = T0[10] = 1.0;

    double positions[sbf::core::MAX_TF][3];
    positions[0][0] = positions[0][1] = positions[0][2] = 0.0;

    double tool_A[12]{};
    const bool has_tool = robot.has_tool();
    if (has_tool) {
        const auto& tf = *robot.tool_frame();
        build_dh_matrix(tf, 0.0, tool_A);
    }

    int idx[sbf::core::MAX_JOINTS]{};
    for (int j = 0; j < n; ++j) {
        const double* Tprev = stack_T + j * 12;
        double* Tnext = stack_T + (j + 1) * 12;
        mul_prefix_dh(Tprev, pre_dh[j][0].A, Tnext);
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

inline void phase0_vertices(const Robot& robot,
                            const std::vector<Interval>& intervals,
                            float* out,
                            int n_active,
                            const int* active_link_map) {
    const int n = robot.n_joints();
    const auto& dh = robot.dh_params();
    std::vector<std::vector<double>> candidates(n);
    for (int j = 0; j < n; ++j) collect_kpi2(intervals[j].lo, intervals[j].hi, candidates[j]);

    std::vector<std::vector<PreDH>> pre_dh(n);
    std::vector<int> n_cands(n);
    for (int j = 0; j < n; ++j) {
        n_cands[j] = static_cast<int>(candidates[j].size());
        pre_dh[j].resize(n_cands[j]);
        for (int k = 0; k < n_cands[j]; ++k) {
            build_dh_matrix(dh[j], candidates[j][k], pre_dh[j][k].A);
        }
    }
    enumerate_critical_iterative(robot, pre_dh, n_cands,
                                 active_link_map, n_active, out);
}

inline EndpointResult endpoint_crit(const Robot& robot,
                                    const std::vector<Interval>& intervals) {
    EndpointResult result;
    result.source = "CritSample";
    result.certified = false;
    result.n_active_links = robot.n_active_links();
    result.endpoint_iaabbs.resize(static_cast<std::size_t>(result.n_active_links) * 2 * 6);
    init_endpoints_inf(result.endpoint_iaabbs.data(), result.n_active_links);

    const int n = robot.n_joints();
    const auto& dh = robot.dh_params();
    std::vector<std::vector<double>> candidates(n);
    for (int j = 0; j < n; ++j) collect_kpi2(intervals[j].lo, intervals[j].hi, candidates[j]);

    static constexpr std::int64_t kMaxCombos = 8192;
    std::int64_t total = 1;
    for (int j = 0; j < n; ++j) {
        total *= static_cast<std::int64_t>(candidates[j].size());
        if (total > kMaxCombos * 16) break;
    }
    while (total > kMaxCombos) {
        int worst = 0;
        for (int j = 1; j < n; ++j) {
            if (candidates[j].size() > candidates[worst].size()) worst = j;
        }
        if (candidates[worst].size() <= 3) break;
        const double lo = intervals[worst].lo;
        const double hi = intervals[worst].hi;
        candidates[worst] = {lo, 0.5 * (lo + hi), hi};
        total = 1;
        for (int j = 0; j < n; ++j) total *= static_cast<std::int64_t>(candidates[j].size());
    }

    std::vector<std::vector<PreDH>> pre_dh(n);
    std::vector<int> n_cands(n);
    for (int j = 0; j < n; ++j) {
        n_cands[j] = static_cast<int>(candidates[j].size());
        pre_dh[j].resize(n_cands[j]);
        for (int k = 0; k < n_cands[j]; ++k) {
            build_dh_matrix(dh[j], candidates[j][k], pre_dh[j][k].A);
        }
    }
    enumerate_critical_iterative(robot, pre_dh, n_cands,
                                 robot.active_link_map(), result.n_active_links,
                                 result.endpoint_iaabbs.data());
    return result;
}

inline void phase1_edges(const Robot& robot,
                         const std::vector<Interval>& intervals,
                         float* out,
                         int n_active,
                         const int* active_link_map,
                         const bool* skip_link) {
    const int n = robot.n_joints();
    double positions[sbf::core::MAX_TF][3];
    std::vector<double> q(n);

    for (int j = 0; j < n; ++j) {
        const double lo_j = intervals[j].lo;
        const double hi_j = intervals[j].hi;
        if (hi_j - lo_j < kNarrowThreshold) continue;

        const double qvals[3] = {lo_j, 0.5 * (lo_j + hi_j), hi_j};
        Eigen::Matrix3d A;
        for (int si = 0; si < 3; ++si) {
            A(si, 0) = std::cos(qvals[si]);
            A(si, 1) = std::sin(qvals[si]);
            A(si, 2) = 1.0;
        }
        auto qr = A.colPivHouseholderQr();

        int max_bg = 1 << std::max(0, n - 1);
        if (max_bg > 128) max_bg = 128;
        for (int bg = 0; bg < max_bg; ++bg) {
            for (int jj = 0; jj < n; ++jj) q[jj] = intervals[jj].center();
            int bit = 0;
            for (int jj = 0; jj < n; ++jj) {
                if (jj == j) continue;
                q[jj] = (bg & (1 << bit)) ? intervals[jj].hi : intervals[jj].lo;
                ++bit;
            }

            double p3[3][sbf::core::MAX_TF][3];
            for (int si = 0; si < 3; ++si) {
                q[j] = qvals[si];
                scalar_fk_positions(robot, q.data(), p3[si]);
            }

            for (int ci = 0; ci < n_active; ++ci) {
                if (skip_link && skip_link[ci]) continue;
                const int v = active_link_map[ci];
                for (int ep = 0; ep < 2; ++ep) {
                    const int frame = ep == 0 ? v : v + 1;
                    for (int d = 0; d < 3; ++d) {
                        Eigen::Vector3d b;
                        for (int si = 0; si < 3; ++si) b[si] = p3[si][frame][d];
                        Eigen::Vector3d coeff = qr.solve(b);
                        if (std::abs(coeff[0]) < 1e-15 && std::abs(coeff[1]) < 1e-15) continue;
                        double cands[2] = {std::atan2(coeff[1], coeff[0]),
                                           std::atan2(coeff[1], coeff[0]) + sbf::core::PI};
                        for (double qc : cands) {
                            while (qc > sbf::core::PI + 1e-6) qc -= sbf::core::TWO_PI;
                            while (qc < -sbf::core::PI - 1e-6) qc += sbf::core::TWO_PI;
                            if (qc >= lo_j - 1e-12 && qc <= hi_j + 1e-12) {
                                q[j] = std::max(lo_j, std::min(hi_j, qc));
                                scalar_fk_positions(robot, q.data(), positions);
                                update_endpoints_from_positions(positions, active_link_map, n_active, out);
                            }
                        }
                    }
                }
            }
        }
    }
}

inline void phase2_faces(const Robot& robot,
                         const std::vector<Interval>& intervals,
                         float* out,
                         int n_active,
                         const int* active_link_map,
                         const bool* skip_link) {
    const int n = robot.n_joints();
    if (n < 2) return;

    double positions[sbf::core::MAX_TF][3];
    std::vector<double> q(n);
    const double shifts[3] = {0.0, sbf::core::PI, -sbf::core::PI};

    for (int ji = 0; ji < n; ++ji) {
        for (int jj = ji + 1; jj < n; ++jj) {
            const double lo_i = intervals[ji].lo, hi_i = intervals[ji].hi;
            const double lo_j = intervals[jj].lo, hi_j = intervals[jj].hi;
            if (hi_i - lo_i < kNarrowThreshold || hi_j - lo_j < kNarrowThreshold) continue;

            int max_bg = 1 << std::max(0, n - 2);
            if (max_bg > 16) max_bg = 16;
            for (int bg = 0; bg < max_bg; ++bg) {
                for (int k = 0; k < n; ++k) q[k] = intervals[k].center();
                int bit = 0;
                for (int k = 0; k < n; ++k) {
                    if (k == ji || k == jj) continue;
                    q[k] = (bg & (1 << bit)) ? intervals[k].hi : intervals[k].lo;
                    ++bit;
                }
                q[ji] = intervals[ji].center();
                q[jj] = intervals[jj].center();

                for (int iter = 0; iter < 3; ++iter) {
                    const int solve_joint[2] = {ji, jj};
                    for (int which = 0; which < 2; ++which) {
                        const int sj = solve_joint[which];
                        const double lo_s = intervals[sj].lo;
                        const double hi_s = intervals[sj].hi;
                        const double qvals[3] = {lo_s, 0.5 * (lo_s + hi_s), hi_s};
                        Eigen::Matrix3d A;
                        for (int si = 0; si < 3; ++si) {
                            A(si, 0) = std::cos(qvals[si]);
                            A(si, 1) = std::sin(qvals[si]);
                            A(si, 2) = 1.0;
                        }
                        auto qr = A.colPivHouseholderQr();
                        double p3[3][sbf::core::MAX_TF][3];
                        for (int si = 0; si < 3; ++si) {
                            q[sj] = qvals[si];
                            scalar_fk_positions(robot, q.data(), p3[si]);
                        }
                        for (int ci = 0; ci < n_active; ++ci) {
                            if (skip_link && skip_link[ci]) continue;
                            const int v = active_link_map[ci];
                            for (int ep = 0; ep < 2; ++ep) {
                                const int frame = ep == 0 ? v : v + 1;
                                for (int d = 0; d < 3; ++d) {
                                    Eigen::Vector3d b;
                                    for (int si = 0; si < 3; ++si) b[si] = p3[si][frame][d];
                                    Eigen::Vector3d coeff = qr.solve(b);
                                    if (std::abs(coeff[0]) < 1e-15 && std::abs(coeff[1]) < 1e-15) continue;
                                    const double qs = std::atan2(coeff[1], coeff[0]);
                                    for (double shift : shifts) {
                                        double qc = qs + shift;
                                        while (qc > sbf::core::PI + 1e-6) qc -= sbf::core::TWO_PI;
                                        while (qc < -sbf::core::PI - 1e-6) qc += sbf::core::TWO_PI;
                                        if (qc >= lo_s - 1e-12 && qc <= hi_s + 1e-12) {
                                            q[sj] = std::max(lo_s, std::min(hi_s, qc));
                                            scalar_fk_positions(robot, q.data(), positions);
                                            update_endpoints_from_positions(positions, active_link_map, n_active, out);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

inline void phase3_interior(const Robot& robot,
                            const std::vector<Interval>& intervals,
                            float* out,
                            int n_active,
                            const int* active_link_map,
                            const bool* skip_link) {
    const int n = robot.n_joints();
    if (n < 2) return;

    double positions[sbf::core::MAX_TF][3];
    std::vector<double> q(n);
    std::vector<std::vector<double>> starts;
    std::vector<double> mid(n);
    for (int j = 0; j < n; ++j) mid[j] = intervals[j].center();
    starts.push_back(mid);
    int n_corners = 1 << n;
    if (n_corners > 16) n_corners = 16;
    for (int c = 0; c < n_corners; ++c) {
        std::vector<double> corner(n);
        for (int j = 0; j < n; ++j) corner[j] = (c & (1 << j)) ? intervals[j].hi : intervals[j].lo;
        starts.push_back(corner);
    }

    const double shifts[3] = {0.0, sbf::core::PI, -sbf::core::PI};
    for (const auto& start : starts) {
        q = start;
        for (int sweep = 0; sweep < 3; ++sweep) {
            for (int j = 0; j < n; ++j) {
                const double lo_j = intervals[j].lo;
                const double hi_j = intervals[j].hi;
                if (hi_j - lo_j < kNarrowThreshold) continue;
                const double qvals[3] = {lo_j, 0.5 * (lo_j + hi_j), hi_j};
                Eigen::Matrix3d A;
                for (int si = 0; si < 3; ++si) {
                    A(si, 0) = std::cos(qvals[si]);
                    A(si, 1) = std::sin(qvals[si]);
                    A(si, 2) = 1.0;
                }
                auto qr = A.colPivHouseholderQr();
                double p3[3][sbf::core::MAX_TF][3];
                for (int si = 0; si < 3; ++si) {
                    q[j] = qvals[si];
                    scalar_fk_positions(robot, q.data(), p3[si]);
                }
                for (int ci = 0; ci < n_active; ++ci) {
                    if (skip_link && skip_link[ci]) continue;
                    const int v = active_link_map[ci];
                    for (int ep = 0; ep < 2; ++ep) {
                        const int frame = ep == 0 ? v : v + 1;
                        for (int d = 0; d < 3; ++d) {
                            Eigen::Vector3d b;
                            for (int si = 0; si < 3; ++si) b[si] = p3[si][frame][d];
                            Eigen::Vector3d coeff = qr.solve(b);
                            if (std::abs(coeff[0]) < 1e-15 && std::abs(coeff[1]) < 1e-15) continue;
                            const double qs = std::atan2(coeff[1], coeff[0]);
                            for (double shift : shifts) {
                                double qc = qs + shift;
                                while (qc > sbf::core::PI + 1e-6) qc -= sbf::core::TWO_PI;
                                while (qc < -sbf::core::PI - 1e-6) qc += sbf::core::TWO_PI;
                                if (qc >= lo_j - 1e-12 && qc <= hi_j + 1e-12) {
                                    q[j] = std::max(lo_j, std::min(hi_j, qc));
                                    scalar_fk_positions(robot, q.data(), positions);
                                    update_endpoints_from_positions(positions, active_link_map, n_active, out);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

inline EndpointResult endpoint_analytical(const Robot& robot,
                                          const std::vector<Interval>& intervals,
                                          int max_phase,
                                          bool bypass_narrow_skip) {
    EndpointResult result;
    result.source = "Analytical";
    result.certified = true;
    result.n_active_links = robot.n_active_links();
    result.endpoint_iaabbs.resize(static_cast<std::size_t>(result.n_active_links) * 2 * 6);
    init_endpoints_inf(result.endpoint_iaabbs.data(), result.n_active_links);

    const int n_active = result.n_active_links;
    const int* active_map = robot.active_link_map();
    phase0_vertices(robot, intervals, result.endpoint_iaabbs.data(), n_active, active_map);

    std::vector<char> skip(n_active, 0);
    int n_pruned = 0;
    if (max_phase >= 1) {
        auto ifk = sbf::core::compute_endpoint_iaabb_ifk(robot, intervals);
        constexpr float prune_tol = 1e-4f;
        for (int ci = 0; ci < n_active; ++ci) {
            bool can_skip = true;
            for (int ep = 0; ep < 2 && can_skip; ++ep) {
                const float* ifk_b = ifk.endpoint_iaabbs.data() + (ci * 2 + ep) * 6;
                const float* cur_b = result.endpoint_iaabbs.data() + (ci * 2 + ep) * 6;
                for (int d = 0; d < 3; ++d) {
                    if (ifk_b[d] < cur_b[d] - prune_tol ||
                        ifk_b[d + 3] > cur_b[d + 3] + prune_tol) {
                        can_skip = false;
                        break;
                    }
                }
            }
            if (can_skip) {
                skip[ci] = 1;
                ++n_pruned;
            }
        }
    }

    double max_width = 0.0;
    for (const auto& iv : intervals) max_width = std::max(max_width, iv.width());
    static constexpr double kPhase123Threshold = 0.15;
    if ((max_width <= kPhase123Threshold && !bypass_narrow_skip) || max_phase < 1) return result;

    std::unique_ptr<bool[]> skip_bool;
    const bool* skip_ptr = nullptr;
    if (n_pruned > 0) {
        skip_bool = std::make_unique<bool[]>(n_active);
        for (int i = 0; i < n_active; ++i) skip_bool[i] = skip[i] != 0;
        skip_ptr = skip_bool.get();
    }

    if (max_phase >= 1) phase1_edges(robot, intervals, result.endpoint_iaabbs.data(), n_active, active_map, skip_ptr);
    if (max_phase >= 2) phase2_faces(robot, intervals, result.endpoint_iaabbs.data(), n_active, active_map, skip_ptr);
    if (max_phase >= 3) phase3_interior(robot, intervals, result.endpoint_iaabbs.data(), n_active, active_map, skip_ptr);
    return result;
}

class GcpcCache {
public:
    static GcpcCache load(const std::string& path) {
        GcpcCache cache;
        std::ifstream ifs(path, std::ios::binary);
        if (!ifs.is_open()) return cache;
        std::uint32_t magic = 0, nd = 0, np = 0;
        ifs.read(reinterpret_cast<char*>(&magic), 4);
        ifs.read(reinterpret_cast<char*>(&nd), 4);
        ifs.read(reinterpret_cast<char*>(&np), 4);
        if (!ifs || magic != 0x47435043U || nd == 0) return GcpcCache{};
        cache.n_dims_ = static_cast<int>(nd);
        cache.points_.resize(np, std::vector<double>(nd));
        for (std::uint32_t i = 0; i < np; ++i) {
            ifs.read(reinterpret_cast<char*>(cache.points_[i].data()),
                     static_cast<std::streamsize>(nd * sizeof(double)));
            if (!ifs) {
                cache.points_.resize(i);
                break;
            }
        }
        return cache;
    }

    bool empty() const { return points_.empty(); }

    std::vector<const std::vector<double>*> lookup(const std::vector<Interval>& intervals) const {
        std::vector<const std::vector<double>*> hits;
        const int n = static_cast<int>(intervals.size());
        for (const auto& pt : points_) {
            if (static_cast<int>(pt.size()) != n) continue;
            bool inside = true;
            for (int j = 0; j < n; ++j) {
                if (pt[j] < intervals[j].lo - 1e-12 || pt[j] > intervals[j].hi + 1e-12) {
                    inside = false;
                    break;
                }
            }
            if (inside) hits.push_back(&pt);
        }
        return hits;
    }

private:
    int n_dims_ = 0;
    std::vector<std::vector<double>> points_;
};

inline EndpointResult endpoint_gcpc(const Robot& robot,
                                    const std::vector<Interval>& intervals,
                                    const GcpcCache& cache,
                                    int max_phase,
                                    bool bypass_narrow_skip) {
    if (cache.empty()) {
        auto parity = endpoint_analytical(robot, intervals, max_phase, bypass_narrow_skip);
        parity.source = "GCPC";
        parity.gcpc_hits = 0;
        return parity;
    }

    auto result = endpoint_analytical(robot, intervals, std::min(max_phase, 2), bypass_narrow_skip);
    result.source = "GCPC";
    result.certified = true;
    if (max_phase < 3) return result;

    auto hits = cache.lookup(intervals);
    result.gcpc_hits = static_cast<int>(hits.size());
    double positions[sbf::core::MAX_TF][3];
    for (const auto* pt : hits) {
        scalar_fk_positions(robot, pt->data(), positions);
        update_endpoints_from_positions(positions, robot.active_link_map(),
                                        result.n_active_links,
                                        result.endpoint_iaabbs.data());
    }
    return result;
}

}  // namespace sbf::exp::endpoint_profile
