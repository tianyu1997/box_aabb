// ═══════════════════════════════════════════════════════════════════════════
// SafeBoxForest v4 — CritSample Implementation (GCPC-based, optimized)
// ═══════════════════════════════════════════════════════════════════════════
//
// Two-phase critical sampling:
//   Phase 1: GCPC cache lookup → deduplicated FK for matching interior
//            critical points within the query interval.
//   Phase 2: Boundary + kπ/2 enumeration → Cartesian product of
//            {lo, hi, kπ/2 ∩ (lo,hi)} per joint, incremental FK.
//
// Optimizations over naive implementation:
//   - Config dedup in Phase 1 (hash set) avoids redundant FK calls
//   - Eigen allocation hoisted outside inner loop
//   - q₀ variant exploration removed (query_link already selects optimal q₀)
//
// Output: per-endpoint iAABBs [n_endpoints × 6] (geometry-only).
//
#define _USE_MATH_DEFINES
#include "sbf/envelope/crit_sample.h"
#include "sbf/envelope/analytical_utils.h"
#include "sbf/envelope/gcpc.h"
#include "sbf/robot/fk.h"

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <unordered_set>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sbf {
namespace envelope {

// ─── Helper: update paired endpoint iAABBs from FK workspace ────────────────
// Hot path — called once per FK evaluation (~500 times per derive_crit_endpoints).
// Only accesses frames for active links (proximal + distal).
static inline void update_endpoint_iaabb(
    const FKWorkspace& ws,
    const int* active_link_map,
    int n_active,
    float* __restrict out)
{
    const int np = ws.np;
    for (int ci = 0; ci < n_active; ++ci) {
        int V = active_link_map[ci];

        // Proximal: prefix[V]
        if (V < np) {
            const auto& col = ws.tf[V].col(3);
            float* __restrict a = out + (ci * 2) * 6;
            const float x = static_cast<float>(col(0));
            const float y = static_cast<float>(col(1));
            const float z = static_cast<float>(col(2));
            if (x < a[0]) a[0] = x;  if (x > a[3]) a[3] = x;
            if (y < a[1]) a[1] = y;  if (y > a[4]) a[4] = y;
            if (z < a[2]) a[2] = z;  if (z > a[5]) a[5] = z;
        }

        // Distal: prefix[V+1]
        if (V + 1 < np) {
            const auto& col = ws.tf[V + 1].col(3);
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

// ─── Helper: fixed-size discretize + hash for dedup (no heap alloc) ─────────
static constexpr int MAX_JOINTS_DEDUP = 16;
using DedupKey = std::array<int, MAX_JOINTS_DEDUP>;

static DedupKey discretize_config(const Eigen::VectorXd& q, int n) {
    DedupKey k{};
    for (int i = 0; i < n; ++i)
        k[i] = static_cast<int>(std::round(q[i] * 1e5));
    return k;
}

struct DedupKeyHash {
    size_t operator()(const DedupKey& k) const noexcept {
        size_t h = 0;
        for (int v : k)
            h = h * 2654435761u + static_cast<size_t>(v);
        return h;
    }
};

// ═══════════════════════════════════════════════════════════════════════════
//  derive_crit_endpoints — main entry point
// ═══════════════════════════════════════════════════════════════════════════

int derive_crit_endpoints(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const CriticalSamplingConfig& config,
    float* out_endpoint_iaabb,
    CritSampleStats* out_stats)
{
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* map = robot.active_link_map();
    const int n_active_ep = n_act * 2;

    CritSampleStats stats{};

    // Initialize output paired endpoint iAABBs to inverted bounds
    for (int k = 0; k < n_active_ep; ++k) {
        float* a = out_endpoint_iaabb + k * 6;
        a[0] = a[1] = a[2] =  1e30f;
        a[3] = a[4] = a[5] = -1e30f;
    }

    FKWorkspace ws;

    // ═══════════════════════════════════════════════════════════════════
    //  Phase 1: GCPC cache lookup (deduplicated)
    // ═══════════════════════════════════════════════════════════════════

    auto t1_start = std::chrono::high_resolution_clock::now();

    if (config.enable_gcpc && config.gcpc_cache && config.gcpc_cache->is_loaded()) {
        const GcpcCache& cache = *config.gcpc_cache;

        // Collect all GCPC matches across all active links, dedup before FK
        std::unordered_set<DedupKey, DedupKeyHash> seen_configs;
        Eigen::VectorXd q(n);

        for (int ci = 0; ci < n_act; ++ci) {
            int link_id = map[ci];

            std::vector<GcpcQueryResult> results;
            cache.query_link(link_id, intervals.data(), results);
            stats.n_gcpc_matches += static_cast<int>(results.size());

            for (const auto& res : results) {
                const auto& pt = *res.point;

                // Build full config: use interval midpoints for non-GCPC joints
                for (int j = 0; j < n; ++j)
                    q[j] = intervals[j].mid();

                q[0] = res.q0_optimal;
                for (int d = 0; d < pt.n_eff; ++d) {
                    if (res.q1_reflected && d == 0)
                        q[d + 1] = res.q1_actual;
                    else
                        q[d + 1] = pt.q_eff[d];
                }

                // Dedup: skip if we've already evaluated this config
                auto dkey = discretize_config(q, n);
                if (!seen_configs.insert(dkey).second)
                    continue;

                // FK and update all paired endpoint iAABBs
                ws.compute(robot, q);
                update_endpoint_iaabb(ws, map, n_act, out_endpoint_iaabb);
                ++stats.n_gcpc_fk;
            }
        }
    }

    stats.phase1_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t1_start).count();

    // ═══════════════════════════════════════════════════════════════════
    //  Phase 2: Boundary + kπ/2 enumeration (precomputed transforms)
    // ═══════════════════════════════════════════════════════════════════

    auto t2_start = std::chrono::high_resolution_clock::now();

    if (config.enable_boundary) {
        // ── Stack-allocated critical angle sets (O4: zero heap alloc) ────
        // Max angles per joint: 2 (boundary) + ~13 (kπ/2 in [-2π, 2π]) = 15
        static constexpr int MAX_ANGLES = 16;
        static constexpr int MAX_JOINTS = 16;
        int cset_n[MAX_JOINTS];            // number of angles per joint
        double cset_v[MAX_JOINTS][MAX_ANGLES];  // angle values

        // Build critical angle sets per joint (inline, no heap)
        for (int j = 0; j < n; ++j) {
            double lo = intervals[j].lo, hi = intervals[j].hi;
            int cnt = 0;
            cset_v[j][cnt++] = lo;
            cset_v[j][cnt++] = hi;
            for (int k = -20; k <= 20; ++k) {
                double a = k * HALF_PI;
                if (a > lo && a < hi && cnt < MAX_ANGLES)
                    cset_v[j][cnt++] = a;
            }
            // Sort
            std::sort(cset_v[j], cset_v[j] + cnt);
            cnt = static_cast<int>(
                std::unique(cset_v[j], cset_v[j] + cnt) - cset_v[j]);
            cset_n[j] = cnt;
        }

        // Check total product; if > CRIT_MAX_COMBOS, fallback to {lo, mid, hi}
        {
            long long total = 1;
            bool fallback = false;
            for (int j = 0; j < n; ++j) {
                total *= cset_n[j];
                if (total > CRIT_MAX_COMBOS) { fallback = true; break; }
            }
            if (fallback) {
                for (int j = 0; j < n; ++j) {
                    double lo = intervals[j].lo, hi = intervals[j].hi;
                    cset_v[j][0] = lo;
                    cset_v[j][1] = 0.5 * (lo + hi);
                    cset_v[j][2] = hi;
                    cset_n[j] = 3;
                }
            }
        }

        // ── Precompute DH transforms (O6: flat stack array) ─────────────
        // pre_tf_flat[offset[j] + i] = DH transform for joint j, angle i.
        // Eliminates all sin/cos from the ~10K hot-loop iterations.
        static constexpr int MAX_TOTAL_TF = MAX_JOINTS * MAX_ANGLES;  // 256
        Eigen::Matrix4d pre_tf_flat[MAX_TOTAL_TF];
        int pre_tf_off[MAX_JOINTS];
        {
            const auto& dh_params = robot.dh_params();
            int offset = 0;
            for (int j = 0; j < n; ++j) {
                pre_tf_off[j] = offset;
                const auto& dh = dh_params[j];
                for (int i = 0; i < cset_n[j]; ++i) {
                    double theta, d;
                    if (dh.joint_type == 0) {
                        theta = cset_v[j][i] + dh.theta;
                        d = dh.d;
                    } else {
                        theta = dh.theta;
                        d = cset_v[j][i] + dh.d;
                    }
                    pre_tf_flat[offset + i] = dh_transform(dh.alpha, dh.a, d, theta);
                }
                offset += cset_n[j];
            }
        }

        // Precompute tool transform (constant across all leaves)
        const bool has_tool = robot.has_tool();
        Eigen::Matrix4d tool_tf;
        if (has_tool) {
            const auto& t = *robot.tool_frame();
            tool_tf = dh_transform(t.alpha, t.a, t.d, t.theta);
        }

        // ── Stack-based iterative DFS with precomputed transforms ───────
        // Only 4×4 matrix multiplies in the hot path — no trig, no heap.
        ws.set_identity();

        struct Frame { int depth; int idx; };
        Frame stack[MAX_JOINTS];
        int sp = 0;
        stack[0] = {0, 0};

        while (sp >= 0) {
            Frame& fr = stack[sp];
            if (fr.idx >= cset_n[fr.depth]) {
                --sp;
                if (sp >= 0) ++stack[sp].idx;
                continue;
            }

            // Apply precomputed transform: tf[depth+1] = tf[depth] * pre_tf
            ws.tf[fr.depth + 1] = ws.tf[fr.depth]
                * pre_tf_flat[pre_tf_off[fr.depth] + fr.idx];

            if (fr.depth + 1 == n) {
                // Leaf: finalize (tool frame + update iAABBs)
                if (has_tool) {
                    ws.tf[n + 1] = ws.tf[n] * tool_tf;
                    ws.np = n + 2;
                } else {
                    ws.np = n + 1;
                }
                update_endpoint_iaabb(ws, map, n_act, out_endpoint_iaabb);
                ++stats.n_boundary_combos;
                ++stats.n_boundary_fk;
                ++fr.idx;
            } else {
                ++sp;
                stack[sp] = {fr.depth + 1, 0};
            }
        }
    }

    stats.phase2_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - t2_start).count();

    int total_fk = stats.n_gcpc_fk + stats.n_boundary_fk;
    if (out_stats) *out_stats = stats;
    return total_fk;
}

} // namespace envelope
} // namespace sbf
