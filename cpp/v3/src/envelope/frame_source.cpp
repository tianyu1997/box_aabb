// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — Endpoint Source Module implementation
//
//  All four endpoint sources produce the same unified output format:
//    endpoint_aabbs [n_endpoints × 6]  (per-endpoint position interval AABBs,
//    geometry only — no link radius)
//
//  IFK and CritSample produce endpoint intervals directly.
//  Analytical and GCPC extract analytical-tight endpoint intervals via
//  out_endpoint_aabb, with IFK fallback for uncovered endpoints.
//
//  Per-link AABBs and sub-AABBs are derived from these endpoint intervals
//  by interpolation + radius in Stage 2 / extract_link_aabbs_from_endpoint.
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/envelope/frame_source.h"
#include "sbf/envelope/envelope_derive.h"

#include <algorithm>
#include <cstring>

namespace sbf {
namespace envelope {

// ─── Helper: extract endpoints from FKState into flat array ────────────────
void fk_to_endpoints(const FKState& fk, const Robot& robot,
                          std::vector<float>& out)
{
    const int n_endpoints = robot.n_joints() + (robot.has_tool() ? 1 : 0);
    out.resize(n_endpoints * 6);

    for (int k = 0; k < n_endpoints; ++k) {
        int fi = k + 1;  // prefix transform index (0 = base, skip)
        out[k * 6 + 0] = static_cast<float>(fk.prefix_lo[fi][3]);   // x_lo
        out[k * 6 + 1] = static_cast<float>(fk.prefix_lo[fi][7]);   // y_lo
        out[k * 6 + 2] = static_cast<float>(fk.prefix_lo[fi][11]);  // z_lo
        out[k * 6 + 3] = static_cast<float>(fk.prefix_hi[fi][3]);   // x_hi
        out[k * 6 + 4] = static_cast<float>(fk.prefix_hi[fi][7]);   // y_hi
        out[k * 6 + 5] = static_cast<float>(fk.prefix_hi[fi][11]);  // z_hi
    }
}

// ─── Helper: convert Analytical/GCPC out_endpoint_aabb → [n_endpoints × 6] ────
//
// Analytical/GCPC produce per-active-link endpoint AABBs:
//   out_endpoint_aabb[(ci * 2 + 0) * 6 ..] = proximal position AABB (transform V)
//   out_endpoint_aabb[(ci * 2 + 1) * 6 ..] = distal position AABB (transform V+1)
// where V = active_link_map[ci].
//
// We merge these into a [n_endpoints × 6] array (endpoints[V-1] and endpoints[V]).
// endpoints[] is pre-initialised with IFK values so uncovered endpoints have valid data.
//
static void merge_analytical_to_endpoints(
    const float* ep_aabb,     // [n_active × 2 × 6] from Analytical/GCPC
    const Robot& robot,
    std::vector<float>& endpoints)  // [n_endpoints × 6], pre-initialised with IFK
{
    const int n_act = robot.n_active_links();
    const int* alm = robot.active_link_map();

    for (int ci = 0; ci < n_act; ++ci) {
        int V = alm[ci];  // link_map: frame index of distal endpoint

        // Proximal: transform V → endpoint V-1
        if (V > 0) {
            const float* src = ep_aabb + (ci * 2 + 0) * 6;
            float* dst = endpoints.data() + (V - 1) * 6;
            // Analytical values are tighter — overwrite (take tighter of
            // multiple active links referencing the same frame via min/max union)
            for (int d = 0; d < 3; ++d) {
                dst[d]     = std::min(dst[d],     src[d]);       // lo
                dst[d + 3] = std::max(dst[d + 3], src[d + 3]);  // hi
            }
        }

        // Distal: transform V+1 → endpoint V
        {
            const float* src = ep_aabb + (ci * 2 + 1) * 6;
            float* dst = endpoints.data() + V * 6;
            for (int d = 0; d < 3; ++d) {
                dst[d]     = std::min(dst[d],     src[d]);       // lo
                dst[d + 3] = std::max(dst[d + 3], src[d + 3]);  // hi
            }
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  compute_endpoint_aabb — full computation
// ═════════════════════════════════════════════════════════════════════════════
EndpointAABBResult compute_endpoint_aabb(
    const EndpointSourceConfig& config,
    const Robot& robot,
    const std::vector<Interval>& intervals)
{
    EndpointAABBResult result;
    result.n_active = robot.n_active_links();
    result.n_endpoints = robot.n_joints() + (robot.has_tool() ? 1 : 0);
    switch (config.method) {

    // ── IFK: Interval FK → endpoint position intervals directly ─────────
    case EndpointSource::IFK: {
        result.fk_state = compute_fk_full(robot, intervals);
        fk_to_endpoints(result.fk_state, robot, result.endpoint_aabbs);
        break;
    }

    // ── CritSample: Critical enumeration → endpoint intervals ────────────
    case EndpointSource::CritSample: {
        result.endpoint_aabbs.resize(result.n_endpoints * 6);
        derive_crit_endpoints(robot, intervals, result.endpoint_aabbs.data(),
                           &result.n_evaluations);

        // Also compute IFK for incremental support
        result.fk_state = compute_fk_full(robot, intervals);
        break;
    }

    // ── Analytical: analytical-tight endpoint intervals via out_endpoint_aabb
    case EndpointSource::Analytical: {
        const int n_act = robot.n_active_links();

        // Temporary: per-link AABBs (not used, but required by the API)
        std::vector<float> link_aabbs(n_act * 6);
        // Analytical endpoint AABBs: [n_active × 2 × 6] (no radius)
        std::vector<float> ep_aabb(n_act * 2 * 6);

        AnalyticalCriticalStats stats{};
        derive_aabb_critical_analytical(
            robot, intervals, 1, config.analytical_config,
            link_aabbs.data(), &stats, ep_aabb.data());

        result.n_evaluations = stats.n_phase0_vertices +
                               stats.n_phase1_edges +
                               stats.n_phase2_faces +
                               stats.n_phase25a_pair1d +
                               stats.n_phase25b_pair2d +
                               stats.n_phase3_interior;

        // Compute IFK endpoints as fallback (for uncovered endpoints)
        result.fk_state = compute_fk_full(robot, intervals);
        fk_to_endpoints(result.fk_state, robot, result.endpoint_aabbs);

        // Overwrite with analytical-tight values for active-link endpoints
        merge_analytical_to_endpoints(ep_aabb.data(), robot,
                                             result.endpoint_aabbs);
        break;
    }

    // ── GCPC: cached critical points → analytical-tight endpoint intervals ─
    case EndpointSource::GCPC: {
        const int n_act = robot.n_active_links();

        // Compute IFK endpoints as fallback
        result.fk_state = compute_fk_full(robot, intervals);
        fk_to_endpoints(result.fk_state, robot, result.endpoint_aabbs);

        if (config.gcpc_cache && config.gcpc_cache->is_loaded()) {
            std::vector<float> link_aabbs(n_act * 6);
            std::vector<float> ep_aabb(n_act * 2 * 6);

            GcpcQueryStats gstats{};
            config.gcpc_cache->derive_aabb_with_gcpc(
                robot, intervals, 1,
                link_aabbs.data(), &gstats, ep_aabb.data());

            result.n_evaluations = gstats.n_cache_matches +
                                   gstats.n_boundary_kpi2 +
                                   gstats.n_boundary_atan2;

            // Overwrite with GCPC-tight values
            merge_analytical_to_endpoints(ep_aabb.data(), robot,
                                                 result.endpoint_aabbs);
        } else {
            // Fallback to Analytical
            std::vector<float> link_aabbs(n_act * 6);
            std::vector<float> ep_aabb(n_act * 2 * 6);

            AnalyticalCriticalStats stats{};
            derive_aabb_critical_analytical(
                robot, intervals, 1,
                AnalyticalCriticalConfig::all_enabled(),
                link_aabbs.data(), &stats, ep_aabb.data());

            merge_analytical_to_endpoints(ep_aabb.data(), robot,
                                                 result.endpoint_aabbs);
        }
        break;
    }
    }

    return result;
}

// ═════════════════════════════════════════════════════════════════════════════
//  compute_endpoint_aabb_incremental — after one dimension change
// ═════════════════════════════════════════════════════════════════════════════
EndpointAABBResult compute_endpoint_aabb_incremental(
    const EndpointSourceConfig& config,
    const FKState& parent_fk,
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int changed_dim)
{
    EndpointAABBResult result;
    result.n_active = robot.n_active_links();
    result.n_endpoints = robot.n_joints() + (robot.has_tool() ? 1 : 0);

    // IFK incremental: efficient partial FK recomputation
    result.fk_state = compute_fk_incremental(parent_fk, robot,
                                              intervals, changed_dim);

    if (config.method == EndpointSource::IFK) {
        // IFK: extract endpoint intervals directly
        fk_to_endpoints(result.fk_state, robot, result.endpoint_aabbs);

    } else if (config.method == EndpointSource::CritSample) {
        // CritSample: full re-derive
        result.endpoint_aabbs.resize(result.n_endpoints * 6);
        derive_crit_endpoints(robot, intervals, result.endpoint_aabbs.data(),
                           &result.n_evaluations);

    } else if (config.method == EndpointSource::Analytical) {
        // Analytical: use out_endpoint_aabb for analytical-tight frames
        const int n_act = robot.n_active_links();
        std::vector<float> link_aabbs(n_act * 6);
        std::vector<float> ep_aabb(n_act * 2 * 6);

        derive_aabb_critical_analytical(
            robot, intervals, 1, config.analytical_config,
            link_aabbs.data(), nullptr, ep_aabb.data());

        // IFK endpoints as fallback for uncovered endpoints
        fk_to_endpoints(result.fk_state, robot, result.endpoint_aabbs);
        merge_analytical_to_endpoints(ep_aabb.data(), robot,
                                             result.endpoint_aabbs);

    } else if (config.method == EndpointSource::GCPC) {
        // GCPC: cached crits → analytical-tight endpoints
        const int n_act = robot.n_active_links();

        fk_to_endpoints(result.fk_state, robot, result.endpoint_aabbs);

        if (config.gcpc_cache && config.gcpc_cache->is_loaded()) {
            std::vector<float> link_aabbs(n_act * 6);
            std::vector<float> ep_aabb(n_act * 2 * 6);

            config.gcpc_cache->derive_aabb_with_gcpc(
                robot, intervals, 1,
                link_aabbs.data(), nullptr, ep_aabb.data());

            merge_analytical_to_endpoints(ep_aabb.data(), robot,
                                                 result.endpoint_aabbs);
        } else {
            std::vector<float> link_aabbs(n_act * 6);
            std::vector<float> ep_aabb(n_act * 2 * 6);

            derive_aabb_critical_analytical(
                robot, intervals, 1,
                AnalyticalCriticalConfig::all_enabled(),
                link_aabbs.data(), nullptr, ep_aabb.data());

            merge_analytical_to_endpoints(ep_aabb.data(), robot,
                                                 result.endpoint_aabbs);
        }
    }

    return result;
}

// ═════════════════════════════════════════════════════════════════════════════
//  extract_link_aabbs_from_endpoint — derive per-link AABBs from endpoint intervals
// ═════════════════════════════════════════════════════════════════════════════
void extract_link_aabbs_from_endpoint(
    const EndpointAABBResult& result,
    const Robot& robot,
    float* out_aabb)
{
    const int n_act = result.n_active;
    const int n_endpoints = result.n_endpoints;

    if (result.endpoint_aabbs.empty()) {
        // Fallback: use FKState directly if endpoint_aabbs not available
        if (result.has_fk_state()) {
            extract_link_aabbs(result.fk_state,
                               robot.active_link_map(),
                               robot.n_active_links(),
                               out_aabb,
                               robot.active_link_radii());
        }
        return;
    }

    // Derive per-link AABB from endpoint intervals:
    //   per-link AABB = union(endpoint[parent], endpoint[child]) + radius
    const int* alm = robot.active_link_map();
    const double* lr = robot.active_link_radii();
    float base_pos[3] = {0.f, 0.f, 0.f};

    for (int ci = 0; ci < n_act; ++ci) {
        int parent_fi = (alm[ci] == 0) ? -1 : alm[ci] - 1;
        int link_fi = alm[ci];
        float r = lr ? static_cast<float>(lr[ci]) : 0.f;
        derive_aabb_subdivided(
            result.endpoint_aabbs.data(), n_endpoints,
            parent_fi, link_fi,
            1, r, base_pos,
            out_aabb + ci * 6);
    }
}

} // namespace envelope
} // namespace sbf
