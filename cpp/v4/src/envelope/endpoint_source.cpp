// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Endpoint source implementation
//  Stage 1: compute_endpoint_iaabb() dispatches to iFK / CritSample /
//           Analytical / GCPC and produces unified endpoint_iaabbs.
//  迁移自 v3 frame_source.cpp，实现 iFK 路径
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/envelope/endpoint_source.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/envelope/crit_sample.h"
#include "sbf/envelope/analytical_solve.h"
#include "sbf/envelope/gcpc.h"

#include "sbf/envelope/analytical_utils.h"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cstring>
#include <stdexcept>
#include <unordered_set>

namespace sbf {
namespace envelope {

// ── Factory methods that depend on config types ─────────────────────────────
EndpointSourceConfig EndpointSourceConfig::crit_sampling() {
    EndpointSourceConfig c;
    c.method = EndpointSource::CritSample;
    return c;
}

EndpointSourceConfig EndpointSourceConfig::analytical() {
    EndpointSourceConfig c;
    c.method = EndpointSource::Analytical;
    return c;
}

// ═════════════════════════════════════════════════════════════════════════════
//  update_endpoint_iaabb_from_ws_paired
//  Expand paired endpoint iAABBs from a scalar FK workspace.
//  Only accesses frames for active links (proximal + distal).
// ═════════════════════════════════════════════════════════════════════════════

static inline void update_endpoint_iaabb_from_ws_paired(
    const FKWorkspace& ws,
    const int* active_link_map,
    int n_active,
    float* __restrict out)
{
    for (int ci = 0; ci < n_active; ++ci) {
        int V = active_link_map[ci];

        // Proximal: prefix[V]
        {
            int fi = V;
            if (fi >= ws.np) break;
            const auto& col = ws.tf[fi].col(3);
            float* __restrict a = out + (ci * 2) * 6;
            const float x = static_cast<float>(col(0));
            const float y = static_cast<float>(col(1));
            const float z = static_cast<float>(col(2));
            if (x < a[0]) a[0] = x;  if (x > a[3]) a[3] = x;
            if (y < a[1]) a[1] = y;  if (y > a[4]) a[4] = y;
            if (z < a[2]) a[2] = z;  if (z > a[5]) a[5] = z;
        }

        // Distal: prefix[V+1]
        {
            int fi = V + 1;
            if (fi >= ws.np) break;
            const auto& col = ws.tf[fi].col(3);
            float* __restrict a = out + (ci * 2 + 1) * 6;
            const float x = static_cast<float>(col(0));
            const float y = static_cast<float>(col(1));
            const float z = static_cast<float>(col(2));
            if (x < a[0]) a[0] = x;  if (x > a[3]) a[3] = x;
            if (y < a[1]) a[1] = y;  if (y > a[4]) a[4] = y;
            if (z < a[2]) a[2] = z;  if (z > a[5]) a[5] = z;
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  GCPC cache interior lookup — expand endpoint iAABBs with cached interior
//  critical points.  Replaces Phase 3 (coordinate descent) when using GCPC.
// ═════════════════════════════════════════════════════════════════════════════

static constexpr int MAX_JOINTS_DEDUP = 16;
using DedupKey = std::array<int, MAX_JOINTS_DEDUP>;

static DedupKey discretize_config_ep(const Eigen::VectorXd& q, int n) {
    DedupKey k{};
    for (int i = 0; i < n; ++i)
        k[i] = static_cast<int>(std::round(q[i] * 1e5));
    return k;
}

struct DedupKeyHash_EP {
    size_t operator()(const DedupKey& k) const noexcept {
        size_t h = 0;
        for (int v : k)
            h = h * 2654435761u + static_cast<size_t>(v);
        return h;
    }
};

static int gcpc_cache_interior_expand(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const GcpcCache& cache,
    float* out_endpoint_iaabb)
{
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* map = robot.active_link_map();

    std::unordered_set<DedupKey, DedupKeyHash_EP> seen_configs;
    Eigen::VectorXd q(n);
    FKWorkspace ws;
    int fk_count = 0;

    for (int ci = 0; ci < n_act; ++ci) {
        int link_id = map[ci];
        std::vector<GcpcQueryResult> results;
        cache.query_link(link_id, intervals.data(), results);

        for (const auto& res : results) {
            const auto& pt = *res.point;
            for (int j = 0; j < n; ++j)
                q[j] = intervals[j].mid();
            q[0] = res.q0_optimal;
            for (int d = 0; d < pt.n_eff; ++d) {
                if (res.q1_reflected && d == 0)
                    q[d + 1] = res.q1_actual;
                else
                    q[d + 1] = pt.q_eff[d];
            }
            auto dkey = discretize_config_ep(q, n);
            if (!seen_configs.insert(dkey).second)
                continue;
            ws.compute(robot, q);
            update_endpoint_iaabb_from_ws_paired(
                ws, map, n_act, out_endpoint_iaabb);
            ++fk_count;
        }
    }
    return fk_count;
}

// ═════════════════════════════════════════════════════════════════════════════
//  fk_to_endpoints — extract paired active endpoint iAABBs from FKState
//
//  Layout: [n_active × 2 × 6]
//    out[(ci*2)*6]   = proximal = prefix[V]       (V = active_link_map[ci])
//    out[(ci*2+1)*6] = distal   = prefix[V+1]
// ═════════════════════════════════════════════════════════════════════════════

void fk_to_endpoints(const FKState& fk, const Robot& robot,
                     float* out, int out_len) {
    const int n_act = robot.n_active_links();
    const int* alm  = robot.active_link_map();
    const int n_active_ep = n_act * 2;
    if (out == nullptr || out_len < n_active_ep * 6) return;

    for (int ci = 0; ci < n_act; ++ci) {
        int V = alm[ci];

        // Proximal: prefix[V]
        out[(ci*2)*6 + 0] = static_cast<float>(fk.prefix_lo[V][3]);
        out[(ci*2)*6 + 1] = static_cast<float>(fk.prefix_lo[V][7]);
        out[(ci*2)*6 + 2] = static_cast<float>(fk.prefix_lo[V][11]);
        out[(ci*2)*6 + 3] = static_cast<float>(fk.prefix_hi[V][3]);
        out[(ci*2)*6 + 4] = static_cast<float>(fk.prefix_hi[V][7]);
        out[(ci*2)*6 + 5] = static_cast<float>(fk.prefix_hi[V][11]);

        // Distal: prefix[V+1]
        out[(ci*2+1)*6 + 0] = static_cast<float>(fk.prefix_lo[V+1][3]);
        out[(ci*2+1)*6 + 1] = static_cast<float>(fk.prefix_lo[V+1][7]);
        out[(ci*2+1)*6 + 2] = static_cast<float>(fk.prefix_lo[V+1][11]);
        out[(ci*2+1)*6 + 3] = static_cast<float>(fk.prefix_hi[V+1][3]);
        out[(ci*2+1)*6 + 4] = static_cast<float>(fk.prefix_hi[V+1][7]);
        out[(ci*2+1)*6 + 5] = static_cast<float>(fk.prefix_hi[V+1][11]);
    }
}

void fk_to_endpoints(const FKState& fk, const Robot& robot,
                     std::vector<float>& out) {
    const int n_active_ep = robot.n_active_links() * 2;
    out.resize(n_active_ep * 6);
    fk_to_endpoints(fk, robot, out.data(), static_cast<int>(out.size()));
}

// ═════════════════════════════════════════════════════════════════════════════
//  compute_endpoint_iaabb — Stage 1 entry point (full computation)
// ═════════════════════════════════════════════════════════════════════════════

EndpointIAABBResult compute_endpoint_iaabb(
    const EndpointSourceConfig& config,
    const Robot& robot,
    const std::vector<Interval>& intervals)
{
    static_assert(kEndpointSourceCount == 4,
                  "New EndpointSource added: update full compute_endpoint_iaabb dispatch");

    EndpointIAABBResult result;
    result.n_active    = robot.n_active_links();
    result.n_active_ep = result.n_active * 2;

    if (config.method == EndpointSource::Count) {
        throw std::invalid_argument("compute_endpoint_iaabb: invalid EndpointSource::Count");
    }

    switch (config.method) {
    case EndpointSource::IFK: {
        result.fk_state = compute_fk_full(robot, intervals);
        fk_to_endpoints(result.fk_state, robot,
                        result.endpoint_data(), result.endpoint_iaabb_len());
        break;
    }
    case EndpointSource::CritSample: {
        // Compute iFK for FK state (incremental support + IFK clamp in extract_link_iaabbs)
        result.fk_state = compute_fk_full(robot, intervals);
        // Skip fk_to_endpoints — derive_crit_endpoints initializes endpoint_iaabbs to ±1e30f
        CriticalSamplingConfig cscfg = config.crit_config_ptr
            ? *config.crit_config_ptr
            : CriticalSamplingConfig::boundary_only();

        CritSampleStats cstats{};
        result.n_evaluations = derive_crit_endpoints(
            robot, intervals, cscfg,
            result.endpoint_data(), &cstats);
        break;
    }
    case EndpointSource::GCPC: {
        result.fk_state = compute_fk_full(robot, intervals);

        AnalyticalCriticalConfig acfg = config.analytical_config_ptr
            ? *config.analytical_config_ptr
            : AnalyticalCriticalConfig::all_enabled();
        acfg.enable_interior_solve = false;

        const int n_sub = 1;
        const int n_act = result.n_active;
        std::vector<float> link_aabbs(n_act * n_sub * 6);
        // analytical ep_aabbs is already in paired layout: [n_act × 2 × 6]
        std::vector<float> ep_aabbs(n_act * (n_sub + 1) * 6);

        AnalyticalCriticalStats astats{};
        derive_aabb_critical_analytical(
            robot, intervals, n_sub, acfg,
            link_aabbs.data(), &astats,
            ep_aabbs.data());

        // Direct copy: analytical ep_aabbs IS already paired layout
        std::memcpy(result.endpoint_data(), ep_aabbs.data(),
                    result.n_active_ep * 6 * sizeof(float));

        // GCPC cache interior expand
        int gcpc_fk = 0;
        if (config.gcpc_cache && config.gcpc_cache->is_loaded()) {
            gcpc_fk = gcpc_cache_interior_expand(
                robot, intervals, *config.gcpc_cache,
                result.endpoint_data());
        }

        result.n_evaluations = astats.n_phase0_vertices
                             + astats.n_phase1_edges
                             + astats.n_phase2_faces
                             + astats.n_phase25a_pair1d
                             + astats.n_phase25b_pair2d
                             + gcpc_fk;
        break;
    }

    case EndpointSource::Analytical: {
        result.fk_state = compute_fk_full(robot, intervals);

        const AnalyticalCriticalConfig acfg = config.analytical_config_ptr
            ? *config.analytical_config_ptr
            : AnalyticalCriticalConfig::all_enabled();

        const int n_sub = 1;
        const int n_act = result.n_active;
        std::vector<float> link_aabbs(n_act * n_sub * 6);
        std::vector<float> ep_aabbs(n_act * (n_sub + 1) * 6);

        AnalyticalCriticalStats astats{};
        derive_aabb_critical_analytical(
            robot, intervals, n_sub, acfg,
            link_aabbs.data(), &astats,
            ep_aabbs.data());

        // Direct copy: analytical ep_aabbs IS already paired layout
        std::memcpy(result.endpoint_data(), ep_aabbs.data(),
                    result.n_active_ep * 6 * sizeof(float));

        result.n_evaluations = astats.n_phase0_vertices
                             + astats.n_phase1_edges
                             + astats.n_phase2_faces
                             + astats.n_phase25a_pair1d
                             + astats.n_phase25b_pair2d
                             + astats.n_phase3_interior;
        break;
    }
    case EndpointSource::Count:
        throw std::invalid_argument("compute_endpoint_iaabb: invalid EndpointSource::Count");
    }

    return result;
}

// ═════════════════════════════════════════════════════════════════════════════
//  compute_endpoint_iaabb_incremental — iFK fast path
// ═════════════════════════════════════════════════════════════════════════════

EndpointIAABBResult compute_endpoint_iaabb_incremental(
    const EndpointSourceConfig& config,
    const FKState& parent_fk,
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int changed_dim)
{
    static_assert(kEndpointSourceCount == 4,
                  "New EndpointSource added: update incremental compute_endpoint_iaabb dispatch");

    EndpointIAABBResult result;
    result.n_active    = robot.n_active_links();
    result.n_active_ep = result.n_active * 2;

    if (config.method == EndpointSource::Count) {
        throw std::invalid_argument("compute_endpoint_iaabb_incremental: invalid EndpointSource::Count");
    }

    // Incremental FK is always available regardless of method
    result.fk_state = compute_fk_incremental(parent_fk, robot, intervals, changed_dim);

    switch (config.method) {
    case EndpointSource::IFK:
        fk_to_endpoints(result.fk_state, robot,
                        result.endpoint_data(), result.endpoint_iaabb_len());
        break;

    case EndpointSource::CritSample: {
        CriticalSamplingConfig cscfg = config.crit_config_ptr
            ? *config.crit_config_ptr
            : CriticalSamplingConfig::boundary_only();
        result.n_evaluations = derive_crit_endpoints(
            robot, intervals, cscfg,
            result.endpoint_data(), nullptr);
        break;
    }

    case EndpointSource::GCPC: {
        AnalyticalCriticalConfig acfg = config.analytical_config_ptr
            ? *config.analytical_config_ptr
            : AnalyticalCriticalConfig::all_enabled();
        acfg.enable_interior_solve = false;

        const int n_sub = 1;
        const int n_act = result.n_active;
        std::vector<float> link_aabbs(n_act * n_sub * 6);
        std::vector<float> ep_aabbs(n_act * (n_sub + 1) * 6);

        AnalyticalCriticalStats astats{};
        derive_aabb_critical_analytical(
            robot, intervals, n_sub, acfg,
            link_aabbs.data(), &astats,
            ep_aabbs.data());

        // Direct copy: analytical ep_aabbs IS already paired layout
        std::memcpy(result.endpoint_data(), ep_aabbs.data(),
                    result.n_active_ep * 6 * sizeof(float));

        int gcpc_fk = 0;
        if (config.gcpc_cache && config.gcpc_cache->is_loaded()) {
            gcpc_fk = gcpc_cache_interior_expand(
                robot, intervals, *config.gcpc_cache,
                result.endpoint_data());
        }

        result.n_evaluations = astats.n_phase0_vertices
                             + astats.n_phase1_edges
                             + astats.n_phase2_faces
                             + astats.n_phase25a_pair1d
                             + astats.n_phase25b_pair2d
                             + gcpc_fk;
        break;
    }

    case EndpointSource::Analytical: {
        const AnalyticalCriticalConfig acfg = config.analytical_config_ptr
            ? *config.analytical_config_ptr
            : AnalyticalCriticalConfig::all_enabled();

        const int n_sub = 1;
        const int n_act = result.n_active;
        std::vector<float> link_aabbs(n_act * n_sub * 6);
        std::vector<float> ep_aabbs(n_act * (n_sub + 1) * 6);

        AnalyticalCriticalStats astats{};
        derive_aabb_critical_analytical(
            robot, intervals, n_sub, acfg,
            link_aabbs.data(), &astats,
            ep_aabbs.data());

        std::memcpy(result.endpoint_data(), ep_aabbs.data(),
                    result.n_active_ep * 6 * sizeof(float));

        result.n_evaluations = astats.n_phase0_vertices
                             + astats.n_phase1_edges
                             + astats.n_phase2_faces
                             + astats.n_phase25a_pair1d
                             + astats.n_phase25b_pair2d
                             + astats.n_phase3_interior;
        break;
    }
    case EndpointSource::Count:
        throw std::invalid_argument("compute_endpoint_iaabb_incremental: invalid EndpointSource::Count");
    }

    return result;
}

// ═════════════════════════════════════════════════════════════════════════════
//  extract_link_iaabbs — derive per-link iAABBs from endpoint iAABBs
// ═════════════════════════════════════════════════════════════════════════════

void extract_link_iaabbs(
    const EndpointIAABBResult& result,
    const Robot& robot,
    float* out_iaabb)
{
    const int n_act = robot.n_active_links();

    // ── Derive per-link iAABBs from endpoint iAABBs ────────────
    // union(parent_endpoint, child_endpoint) + link_radius for each active link.
    // No sublink subdivision — purely endpoint-to-link conversion.
    if (result.has_endpoint_iaabbs()) {
        const int* alm = robot.active_link_map();
        const double* lr = robot.active_link_radii();

        // Convert double radii → float for derive_aabb_paired
        std::vector<float> float_radii(n_act);
        for (int ci = 0; ci < n_act; ++ci)
            float_radii[ci] = lr ? static_cast<float>(lr[ci]) : 0.f;

        derive_aabb_paired(
            result.endpoint_data(), n_act,
            float_radii.data(),
            out_iaabb);
        return;
    }

    // Fallback: if we have FK state, extract directly
    if (result.has_fk_state()) {
        extract_link_aabbs(result.fk_state,
                          robot.active_link_map(), robot.n_active_links(),
                          out_iaabb, robot.active_link_radii());
    }
}

} // namespace envelope
} // namespace sbf