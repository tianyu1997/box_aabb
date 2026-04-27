#include "sbf/core/endpoint_iaabb.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace sbf::core {

namespace {

constexpr double kCritNarrowThreshold = 0.01;
constexpr std::int64_t kCritMaxCombos = 8192;

struct PreDH {
    double a[12];
};

void init_endpoint_inf(float* out, int n_active) {
    for (int i = 0; i < n_active * 2; ++i) {
        float* box = out + i * 6;
        box[0] = box[1] = box[2] =  std::numeric_limits<float>::infinity();
        box[3] = box[4] = box[5] = -std::numeric_limits<float>::infinity();
    }
}

void build_dh_matrix(const DHParam& dh, double q_val, double out[12]) {
    const double d_val = (dh.joint_type == 1) ? (q_val + dh.d) : dh.d;
    const double angle = (dh.joint_type == 0) ? (q_val + dh.theta) : dh.theta;
    const double ct = std::cos(angle);
    const double st = std::sin(angle);
    const double ca = std::cos(dh.alpha);
    const double sa = std::sin(dh.alpha);
    out[0]  = ct;      out[1]  = -st;     out[2]  = 0.0;  out[3]  = dh.a;
    out[4]  = st * ca; out[5]  = ct * ca; out[6]  = -sa;  out[7]  = -d_val * sa;
    out[8]  = st * sa; out[9]  = ct * sa; out[10] = ca;   out[11] = d_val * ca;
}

void mul_prefix_dh(const double prev[12], const double a[12], double next[12]) {
    for (int r = 0; r < 3; ++r) {
        const double* tp = prev + r * 4;
        double* tn = next + r * 4;
        for (int c = 0; c < 4; ++c) {
            tn[c] = tp[0] * a[c] + tp[1] * a[4 + c] + tp[2] * a[8 + c];
        }
        tn[3] += tp[3];
    }
}

void collect_kpi2(double lo, double hi, std::vector<double>& out) {
    if (hi - lo < kCritNarrowThreshold) {
        out.push_back(0.5 * (lo + hi));
        return;
    }
    out.push_back(lo);
    out.push_back(hi);
    const double k_lo = std::ceil(lo / HALF_PI);
    const double k_hi = std::floor(hi / HALF_PI);
    for (double k = k_lo; k <= k_hi; k += 1.0) {
        const double v = k * HALF_PI;
        if (v > lo + 1e-12 && v < hi - 1e-12) out.push_back(v);
    }
}

void update_endpoints_from_positions(const double positions[][3],
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

void enumerate_critical_iterative(
    const Robot& robot,
    const std::vector<std::vector<PreDH>>& pre_dh,
    const std::vector<int>& n_cands,
    float* out) {
    const int n = robot.n_joints();
    double stack_t[(MAX_JOINTS + 2) * 12];
    double* t0 = stack_t;
    for (int i = 0; i < 12; ++i) t0[i] = 0.0;
    t0[0] = t0[5] = t0[10] = 1.0;

    double positions[MAX_TF][3];
    positions[0][0] = positions[0][1] = positions[0][2] = 0.0;

    double tool_a[12]{};
    const bool has_tool = robot.has_tool();
    if (has_tool) {
        build_dh_matrix(*robot.tool_frame(), 0.0, tool_a);
    }

    int idx[MAX_JOINTS]{};
    auto update_from = [&](int start_joint) {
        for (int j = start_joint; j < n; ++j) {
            const double* prev = stack_t + j * 12;
            double* next = stack_t + (j + 1) * 12;
            mul_prefix_dh(prev, pre_dh[j][idx[j]].a, next);
            positions[j + 1][0] = next[3];
            positions[j + 1][1] = next[7];
            positions[j + 1][2] = next[11];
        }
        if (has_tool) {
            double tool_r[12];
            mul_prefix_dh(stack_t + n * 12, tool_a, tool_r);
            positions[n + 1][0] = tool_r[3];
            positions[n + 1][1] = tool_r[7];
            positions[n + 1][2] = tool_r[11];
        }
    };

    update_from(0);
    update_endpoints_from_positions(positions, robot.active_link_map(),
                                    robot.n_active_links(), out);

    while (true) {
        int carry = n - 1;
        while (carry >= 0) {
            idx[carry]++;
            if (idx[carry] < n_cands[carry]) break;
            idx[carry] = 0;
            carry--;
        }
        if (carry < 0) break;
        update_from(carry);
        update_endpoints_from_positions(positions, robot.active_link_map(),
                                        robot.n_active_links(), out);
    }
}

EndpointIAABBResult compute_endpoint_iaabb_critsample(
    const Robot& robot,
    const std::vector<Interval>& intervals) {
    EndpointIAABBResult result;
    result.n_active_links = robot.n_active_links();
    result.is_safe = false;
    result.endpoint_iaabbs.resize(
        static_cast<std::size_t>(result.n_active_links) * 2u * 6u);
    init_endpoint_inf(result.endpoint_iaabbs.data(), result.n_active_links);

    const int n = robot.n_joints();
    const auto& dh = robot.dh_params();
    std::vector<std::vector<double>> candidates(n);
    for (int j = 0; j < n; ++j) collect_kpi2(intervals[j].lo, intervals[j].hi, candidates[j]);

    std::int64_t total = 1;
    for (int j = 0; j < n; ++j) {
        total *= static_cast<std::int64_t>(candidates[j].size());
        if (total > kCritMaxCombos * 16) break;
    }
    while (total > kCritMaxCombos) {
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
        pre_dh[j].resize(static_cast<std::size_t>(n_cands[j]));
        for (int k = 0; k < n_cands[j]; ++k) {
            build_dh_matrix(dh[j], candidates[j][k], pre_dh[j][k].a);
        }
    }
    enumerate_critical_iterative(robot, pre_dh, n_cands,
                                 result.endpoint_iaabbs.data());
    return result;
}

}  // namespace

const char* endpoint_source_name(EndpointSourceKind kind) {
    switch (kind) {
    case EndpointSourceKind::IFK:        return "ifk";
    case EndpointSourceKind::CritSample: return "critsample";
    }
    return "unknown";
}

EndpointSourceKind parse_endpoint_source_kind(const std::string& name) {
    if (name == "ifk" || name == "IFK") return EndpointSourceKind::IFK;
    if (name == "critsample" || name == "crit_sample" ||
        name == "crit-sample" || name == "CritSample") {
        return EndpointSourceKind::CritSample;
    }
    throw std::invalid_argument("unknown endpoint source: " + name);
}

EndpointIAABBResult compute_endpoint_iaabb_ifk(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    FKState* fk_cache,
    int changed_dim) {
    EndpointIAABBResult res;
    res.n_active_links = robot.n_active_links();
    res.is_safe = true;
    res.endpoint_iaabbs.assign(res.n_active_links * 2 * 6, 0.0f);

    if (fk_cache && fk_cache->valid && changed_dim >= 0) {
        update_fk_inplace(*fk_cache, robot, intervals, changed_dim);
        res.fk_state = *fk_cache;
    } else {
        res.fk_state = compute_fk_full(robot, intervals);
        if (fk_cache) *fk_cache = res.fk_state;
    }

    extract_endpoint_iaabbs(res.fk_state,
                            robot.active_link_map(),
                            res.n_active_links,
                            res.endpoint_iaabbs.data());
    return res;
}

EndpointIAABBResult compute_endpoint_iaabb(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    EndpointSourceConfig source,
    FKState* fk_cache,
    int changed_dim) {
    switch (source.kind) {
    case EndpointSourceKind::IFK:
        return compute_endpoint_iaabb_ifk(robot, intervals, fk_cache, changed_dim);
    case EndpointSourceKind::CritSample:
        return compute_endpoint_iaabb_critsample(robot, intervals);
    }
    throw std::invalid_argument("unsupported endpoint source");
}

}  // namespace sbf::core
