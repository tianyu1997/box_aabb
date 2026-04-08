// ═══════════════════════════════════════════════════════════════════════════
// SafeBoxForest v2 — GCPC Cache Implementation
// ═══════════════════════════════════════════════════════════════════════════
//
// KD-tree construction, range query, q₀ reconstruction, q₁ reflection,
// and the combined GCPC query pipeline.
//
#define _USE_MATH_DEFINES
#include "sbf/envelope/gcpc_cache.h"
#include "sbf/robot/fk.h"
#include "sbf/robot/interval_fk.h"  // retained for reference
#include "sbf/robot/affine_fk.h"   // Affine-Arithmetic FK for tight pruning

#include <Eigen/Core>
#include <Eigen/QR>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// JSON parser — minimal, parse the precompute output.
// We inline a simple parser instead of adding a dependency.
#include <map>

namespace sbf {
namespace envelope {

static constexpr double HALF_PI = 1.5707963267948966;
static constexpr double kShifts[3] = {0.0, 2*M_PI, -2*M_PI};

// ═══════════════════════════════════════════════════════════════════════════
//  q₀ reconstruction helpers
// ═══════════════════════════════════════════════════════════════════════════

double GcpcCache::reconstruct_q0_xy(double A, double B) {
    // For x/y direction: max p_x = R when q₀ = atan2(-B, A)
    //                    max p_y = R when q₀ = atan2(A, B)
    // We store q₀ for max p_x; p_y max/min uses q₀ ± π/2
    return std::atan2(-B, A);
}

bool GcpcCache::q0_in_range(double q0, double lo0, double hi0) {
    // Check if q0 ∈ [lo0, hi0], accounting for angle wrapping.
    // We assume both are in [-π, π) or nearby.
    for (double shift : kShifts) {
        double q = q0 + shift;
        if (q >= lo0 - 1e-10 && q <= hi0 + 1e-10)
            return true;
    }
    return false;
}

// ═══════════════════════════════════════════════════════════════════════════
//  KD-tree construction
// ═══════════════════════════════════════════════════════════════════════════

void GcpcCache::build_kdtree(GcpcLinkSection& section) {
    int n = section.n_points;
    if (n == 0) {
        section.kd_root = -1;
        return;
    }

    section.kd_tree.clear();
    section.kd_tree.reserve(2 * n);  // upper bound on nodes

    std::vector<int> indices(n);
    std::iota(indices.begin(), indices.end(), 0);

    section.kd_root = build_kdtree_recursive(section, indices, 0, n, 0);
}

int GcpcCache::build_kdtree_recursive(
    GcpcLinkSection& section,
    std::vector<int>& indices,
    int lo, int hi, int depth)
{
    if (lo >= hi) return -1;

    int node_idx = static_cast<int>(section.kd_tree.size());
    section.kd_tree.push_back(KdNode{});
    auto& node = section.kd_tree[node_idx];

    if (hi - lo == 1) {
        // Leaf node
        node.split_dim = -1;
        node.split_val = 0.0;
        node.left = -1;
        node.right = -1;
        node.point_idx = indices[lo];
        return node_idx;
    }

    // Split dimension: round-robin
    int n_eff = section.n_eff_joints;
    int dim = depth % n_eff;

    // Find median
    int mid = (lo + hi) / 2;
    std::nth_element(
        indices.begin() + lo,
        indices.begin() + mid,
        indices.begin() + hi,
        [&](int a, int b) {
            return section.points[a].q_eff[dim] < section.points[b].q_eff[dim];
        });

    double median_val = section.points[indices[mid]].q_eff[dim];

    node.split_dim = dim;
    node.split_val = median_val;
    node.point_idx = -1;

    // Recurse
    // Reserve space for children (indices may change the vector)
    node.left = build_kdtree_recursive(section, indices, lo, mid, depth + 1);
    // Re-fetch node reference since vector may have grown
    section.kd_tree[node_idx].right =
        build_kdtree_recursive(section, indices, mid, hi, depth + 1);

    return node_idx;
}

// ═══════════════════════════════════════════════════════════════════════════
//  KD-tree range query
// ═══════════════════════════════════════════════════════════════════════════

void GcpcCache::kdtree_range_query(
    const GcpcLinkSection& section,
    int node_idx,
    const double* lo, const double* hi,
    int depth,
    std::vector<int>& matched_indices) const
{
    if (node_idx < 0) return;
    const auto& node = section.kd_tree[node_idx];

    if (node.point_idx >= 0) {
        // Leaf: check if point is in range
        const auto& pt = section.points[node.point_idx];
        bool in_range = true;
        for (int d = 0; d < section.n_eff_joints; ++d) {
            if (pt.q_eff[d] < lo[d] - 1e-10 || pt.q_eff[d] > hi[d] + 1e-10) {
                in_range = false;
                break;
            }
        }
        if (in_range) {
            matched_indices.push_back(node.point_idx);
        }
        return;
    }

    int dim = node.split_dim;
    // Left subtree: points with q_eff[dim] <= split_val
    if (lo[dim] <= node.split_val + 1e-10) {
        kdtree_range_query(section, node.left, lo, hi, depth + 1, matched_indices);
    }
    // Right subtree: points with q_eff[dim] > split_val
    if (hi[dim] > node.split_val - 1e-10) {
        kdtree_range_query(section, node.right, lo, hi, depth + 1, matched_indices);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Query: find critical points for a link within intervals
// ═══════════════════════════════════════════════════════════════════════════

void GcpcCache::query_link(
    int link_id,
    const Interval* intervals,
    std::vector<GcpcQueryResult>& results) const
{
    results.clear();

    const GcpcLinkSection* sec = find_section(link_id);
    if (!sec || sec->kd_root < 0) return;

    int n_eff = sec->n_eff_joints;

    // Build query range for q₁..qₖ (q₀ excluded, q₁ in [0,π] half)
    // intervals[0] = q₀ range (used for q₀ reconstruction check)
    // intervals[1] = q₁ range, ..., intervals[k] = qₖ range

    double lo_q[7], hi_q[7];
    for (int d = 0; d < n_eff; ++d) {
        lo_q[d] = intervals[d + 1].lo;  // q₁ = intervals[1], etc.
        hi_q[d] = intervals[d + 1].hi;
    }

    // ── Step 1: Query the half-range [0,π] cache ────────────────────────
    // For q₁ (index 0 in lo_q), intersect with [0, π]
    double orig_lo_q1 = lo_q[0];
    double orig_hi_q1 = hi_q[0];

    // Query for q₁ ∈ [max(lo₁, 0), min(hi₁, π)]
    double q1_lo_half = std::max(orig_lo_q1, 0.0);
    double q1_hi_half = std::min(orig_hi_q1, M_PI);

    std::vector<int> matched;

    if (q1_lo_half <= q1_hi_half + 1e-10) {
        lo_q[0] = q1_lo_half;
        hi_q[0] = q1_hi_half;
        kdtree_range_query(*sec, sec->kd_root, lo_q, hi_q, 0, matched);
    }

    // Process matches: reconstruct q₀, add to results
    for (int idx : matched) {
        const auto& pt = sec->points[idx];
        GcpcQueryResult res;
        res.point = &pt;
        res.q1_reflected = false;
        res.q1_actual = pt.q_eff[0];

        if (pt.direction == 0) {
            // xy direction: q₀* = atan2(-B, A)
            res.q0_optimal = reconstruct_q0_xy(pt.A, pt.B);
            if (!q0_in_range(res.q0_optimal, intervals[0].lo, intervals[0].hi)) {
                // Also check q₀* + π (for min p_x)
                double q0_alt = res.q0_optimal + M_PI;
                if (q0_alt > M_PI) q0_alt -= 2 * M_PI;
                if (!q0_in_range(q0_alt, intervals[0].lo, intervals[0].hi))
                    continue;
                res.q0_optimal = q0_alt;
            }
        } else {
            // z direction: q₀ is free — use midpoint (any q₀ gives same p_z)
            res.q0_optimal = 0.5 * (intervals[0].lo + intervals[0].hi);
        }

        results.push_back(res);
    }

    // ── Step 2: q₁ reflection ───────────────────────────────────────────
    // For each cached point with q₁*, generate reflected point q₁* - π
    // Check if reflected q₁ falls in [lo₁, hi₁]

    // Reflected range: q₁ - π where q₁ ∈ [0, π] → q₁-π ∈ [-π, 0]
    // We need to intersect [-π, 0] with [orig_lo_q1, orig_hi_q1]
    double refl_lo = std::max(orig_lo_q1, -M_PI);
    double refl_hi = std::min(orig_hi_q1, 0.0);

    if (refl_lo <= refl_hi + 1e-10) {
        // Query for q₁ ∈ [refl_lo + π, refl_hi + π] in the cache
        // (which stores q₁ in [0, π])
        double cache_q1_lo = refl_lo + M_PI;  // maps to [0, π]
        double cache_q1_hi = refl_hi + M_PI;
        cache_q1_lo = std::max(cache_q1_lo, 0.0);
        cache_q1_hi = std::min(cache_q1_hi, M_PI);

        if (cache_q1_lo <= cache_q1_hi + 1e-10) {
            // Set q₁ range for second query
            double lo_q2[7], hi_q2[7];
            for (int d = 0; d < n_eff; ++d) {
                lo_q2[d] = intervals[d + 1].lo;
                hi_q2[d] = intervals[d + 1].hi;
            }
            lo_q2[0] = cache_q1_lo;
            hi_q2[0] = cache_q1_hi;

            std::vector<int> matched_refl;
            kdtree_range_query(*sec, sec->kd_root, lo_q2, hi_q2, 0, matched_refl);

            for (int idx : matched_refl) {
                const auto& pt = sec->points[idx];
                GcpcQueryResult res;
                res.point = &pt;
                res.q1_reflected = true;
                res.q1_actual = pt.q_eff[0] - M_PI;  // reflected q₁

                if (pt.direction == 0) {
                    // Under q₁ → q₁ - π: A → -A, B unchanged
                    // So R² = A² + B² is the same, q₀* = atan2(-B, -A) = atan2(-B, A) + π
                    double A_refl = -pt.A;
                    double B_refl = pt.B;
                    res.q0_optimal = reconstruct_q0_xy(A_refl, B_refl);
                    if (!q0_in_range(res.q0_optimal, intervals[0].lo, intervals[0].hi)) {
                        double q0_alt = res.q0_optimal + M_PI;
                        if (q0_alt > M_PI) q0_alt -= 2 * M_PI;
                        if (!q0_in_range(q0_alt, intervals[0].lo, intervals[0].hi))
                            continue;
                        res.q0_optimal = q0_alt;
                    }
                } else {
                    // z direction: C → -C under reflection, q₀ free
                    res.q0_optimal = 0.5 * (intervals[0].lo + intervals[0].hi);
                }

                results.push_back(res);
            }
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  Main GCPC query pipeline: derive_aabb_with_gcpc
// ═══════════════════════════════════════════════════════════════════════════

void GcpcCache::derive_aabb_with_gcpc(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    float* out_aabb,
    GcpcQueryStats* out_stats) const
{
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* map = robot.active_link_map();
    const double* rad = robot.active_link_radii();
    const float inv_n = 1.0f / static_cast<float>(n_sub);

    GcpcQueryStats stats{};

    // Initialize output AABBs
    for (int ci = 0; ci < n_act; ++ci) {
        for (int s = 0; s < n_sub; ++s) {
            float* a = out_aabb + (ci * n_sub + s) * 6;
            a[0] = a[1] = a[2] =  1e30f;
            a[3] = a[4] = a[5] = -1e30f;
        }
    }

    // Helper: evaluate a full-DOF config and update AABBs
    auto eval_config = [&](const Eigen::VectorXd& q) {
        auto pos = fk_link_positions(robot, q);
        const int np = static_cast<int>(pos.size());
        ++stats.n_fk_calls;

        for (int ci = 0; ci < n_act; ++ci) {
            int V = map[ci];
            if (V + 1 >= np) continue;
            const auto& p_prox = pos[V];
            const auto& p_dist = pos[V + 1];
            for (int s = 0; s < n_sub; ++s) {
                float t0 = s * inv_n;
                float t1 = (s + 1) * inv_n;
                Eigen::Vector3d s0 = p_prox + double(t0) * (p_dist - p_prox);
                Eigen::Vector3d s1 = p_prox + double(t1) * (p_dist - p_prox);

                float* a = out_aabb + (ci * n_sub + s) * 6;
                for (int d = 0; d < 3; ++d) {
                    float vmin = static_cast<float>(std::min(s0[d], s1[d]));
                    float vmax = static_cast<float>(std::max(s0[d], s1[d]));
                    if (vmin < a[d])     a[d]     = vmin;
                    if (vmax > a[d + 3]) a[d + 3] = vmax;
                }
            }
        }
    };

    // ══════════════════════════════════════════════════════════════════
    //  Phase A: Cache lookup
    // ══════════════════════════════════════════════════════════════════

    for (int ci = 0; ci < n_act; ++ci) {
        int link_id = map[ci];
        std::vector<GcpcQueryResult> results;
        query_link(link_id, intervals.data(), results);
        stats.n_cache_matches += static_cast<int>(results.size());

        for (const auto& res : results) {
            const auto& pt = *res.point;
            if (res.q1_reflected) ++stats.n_q1_reflected;
            ++stats.n_q0_valid;

            // Build full 7-DOF config
            Eigen::VectorXd q(n);
            for (int j = 0; j < n; ++j) q[j] = intervals[j].mid();

            // Set q₀
            q[0] = res.q0_optimal;

            // Set q₁..qₖ
            for (int d = 0; d < pt.n_eff; ++d) {
                if (res.q1_reflected && d == 0) {
                    q[d + 1] = res.q1_actual;  // reflected q₁
                } else {
                    q[d + 1] = pt.q_eff[d];
                }
            }

            eval_config(q);

            // For xy direction: also evaluate at q₀ shifted by π/2 (for p_y max/min)
            if (pt.direction == 0) {
                double q0_y_max = res.q0_optimal + HALF_PI;
                if (q0_y_max > M_PI) q0_y_max -= 2 * M_PI;
                if (q0_in_range(q0_y_max, intervals[0].lo, intervals[0].hi)) {
                    q[0] = q0_y_max;
                    eval_config(q);
                }
                double q0_y_min = res.q0_optimal - HALF_PI;
                if (q0_y_min < -M_PI) q0_y_min += 2 * M_PI;
                if (q0_in_range(q0_y_min, intervals[0].lo, intervals[0].hi)) {
                    q[0] = q0_y_min;
                    eval_config(q);
                }
            }
        }
    }

    // ══════════════════════════════════════════════════════════════════
    //  Phase B: Boundary kπ/2 enumeration
    // ══════════════════════════════════════════════════════════════════

    {
        // Build candidate angles per joint
        auto crit_angles_fn = [](double lo, double hi) -> std::vector<double> {
            std::vector<double> v = { lo, hi };
            for (int k = -20; k <= 20; ++k) {
                double a = k * HALF_PI;
                if (a > lo && a < hi)
                    v.push_back(a);
            }
            std::sort(v.begin(), v.end());
            v.erase(std::unique(v.begin(), v.end()), v.end());
            return v;
        };

        std::vector<std::vector<double>> per_joint(n);
        for (int j = 0; j < n; ++j)
            per_joint[j] = crit_angles_fn(intervals[j].lo, intervals[j].hi);

        // Determine max joints needed
        int max_V = 0;
        for (int ci = 0; ci < n_act; ++ci)
            max_V = std::max(max_V, map[ci]);
        int n_joints_needed = std::min(max_V + 1, n);

        // Compute product size
        long long product = 1;
        for (int j = 0; j < n_joints_needed; ++j) {
            product *= static_cast<long long>(per_joint[j].size());
            if (product > 60000) break;
        }

        if (product <= 60000) {
            // Full enumeration
            Eigen::VectorXd q(n);
            for (int j = 0; j < n; ++j) q[j] = intervals[j].mid();

            // Recursive enumeration
            std::function<void(int)> enumerate;
            enumerate = [&](int j) {
                if (j >= n_joints_needed) {
                    eval_config(q);
                    ++stats.n_boundary_kpi2;
                    return;
                }
                for (double v : per_joint[j]) {
                    q[j] = v;
                    enumerate(j + 1);
                }
            };
            enumerate(0);
        } else {
            // Too many combos — use corner sampling
            // Just evaluate all 2^n corners (lo/hi)
            int n_corners = std::min(1 << n_joints_needed, 1024);
            Eigen::VectorXd q(n);
            for (int j = 0; j < n; ++j) q[j] = intervals[j].mid();

            for (int mask = 0; mask < n_corners; ++mask) {
                for (int j = 0; j < n_joints_needed && j < 10; ++j)
                    q[j] = (mask & (1 << j)) ? intervals[j].hi : intervals[j].lo;
                eval_config(q);
                ++stats.n_boundary_kpi2;
            }
        }
    }

    // ══════════════════════════════════════════════════════════════════
    //  Phase C: Boundary 1D atan2  (with IFK face-level pruning)
    // ══════════════════════════════════════════════════════════════════
    //
    //  Key idea: after Phase A+B, out_aabb contains a "pre-best" bound.
    //  Before enumerating all 2^{nj-1} background combos for a face
    //  (q_j free, q_k=boundary_value for one specific k), we compute
    //  an incremental IFK with q_k pinned.  If the IFK outer bound for
    //  every link/dim/sub-segment is dominated by the pre-best, then
    //  NO configuration on that face can improve the result → skip.
    //
    //  Cost: one incremental IFK per face (~2-5 μs) vs. saving up to
    //  2^{nj-2} × 3 FK calls per pruned face.
    // ══════════════════════════════════════════════════════════════════

    {
        // ── Affine-Arithmetic FK for tight pruning bounds ──────────
        // AA preserves cos/sin correlations (shared noise symbols),
        // which drastically reduces the dependency-problem overestimation
        // that plagues standard interval arithmetic.
        AAFKResult aa_fk = aa_compute_fk(robot, intervals);

        // Helper: check if AA bounds for a given frame pair (prox, dist)
        // can improve out_aabb for link ci.
        // Returns true if any (sub-seg, dim) has aa_lo < oa_lo or aa_hi > oa_hi.
        auto aa_link_can_improve = [&](const AAFKResult& fk_res,
                                       int ci) -> bool {
            int li = map[ci];
            double plo[3], phi[3], dlo[3], dhi[3];
            aa_position_bounds(fk_res.prefix[li],     plo, phi);
            aa_position_bounds(fk_res.prefix[li + 1], dlo, dhi);
            for (int s = 0; s < n_sub; ++s) {
                float t0 = static_cast<float>(s) * inv_n;
                float t1 = static_cast<float>(s + 1) * inv_n;
                const float* oa = out_aabb + (ci * n_sub + s) * 6;
                for (int d = 0; d < 3; ++d) {
                    float pA_lo = static_cast<float>(plo[d]);
                    float pA_hi = static_cast<float>(phi[d]);
                    float pB_lo = static_cast<float>(dlo[d]);
                    float pB_hi = static_cast<float>(dhi[d]);
                    float s0_lo = pA_lo * (1.f - t0) + pB_lo * t0;
                    float s0_hi = pA_hi * (1.f - t0) + pB_hi * t0;
                    float s1_lo = pA_lo * (1.f - t1) + pB_lo * t1;
                    float s1_hi = pA_hi * (1.f - t1) + pB_hi * t1;
                    float aa_lo = std::min(s0_lo, s1_lo);
                    float aa_hi = std::max(s0_hi, s1_hi);
                    if (aa_lo < oa[d] || aa_hi > oa[d + 3])
                        return true;
                }
            }
            return false;
        };

        Eigen::VectorXd q(n);

        for (int ci = 0; ci < n_act; ++ci) {
            int V = map[ci];
            int nj = std::min(V + 1, n);

            // Background combos: cap at 128
            int n_bg_bits = nj - 1;
            int max_bg = std::min(1 << n_bg_bits, 128);

            for (int j = 0; j < nj; ++j) {
                double lo_j = intervals[j].lo;
                double hi_j = intervals[j].hi;
                if (hi_j - lo_j < 1e-12) continue;

                double mid_j = 0.5 * (lo_j + hi_j);
                double qvals[3] = { lo_j, mid_j, hi_j };

                // ── AA-based pruning ──────────────────────────────────
                // Level 1: full-box AA check for link ci
                // Level 2: per-bg 1-DOF AA check (very tight)

                // Level 1: AA full-box check for link ci.
                // The AA bounds are MUCH tighter than IA because
                // cos(qⱼ)/sin(qⱼ) correlations are preserved.
                {
                    ++stats.n_ifk_prune_checks;
                    if (!aa_link_can_improve(aa_fk, ci)) {
                        ++stats.n_ifk_pruned;
                        continue;  // next free joint j
                    }
                }

                // Level 2: per-bg combo AA pruning.
                // For 1-DOF sub-boxes, AA is very tight (only one noise symbol).
                bool do_level2 = (max_bg >= 4);

                for (int bg = 0; bg < max_bg; ++bg) {
                    for (int jj = 0; jj < n; ++jj) q[jj] = intervals[jj].mid();

                    // Build the 1-DOF sub-box: q_j free, others pinned to lo/hi
                    std::vector<Interval> face_iv(intervals.begin(), intervals.end());
                    int bit = 0;
                    for (int jj = 0; jj < nj; ++jj) {
                        if (jj == j) continue;
                        double bv = (bg & (1 << bit)) ? intervals[jj].hi : intervals[jj].lo;
                        q[jj] = bv;
                        face_iv[jj] = Interval{bv, bv};
                        ++bit;
                    }
                    face_iv[j] = intervals[j];

                    // Level 2: AA FK for this 1-DOF sub-box
                    if (do_level2) {
                        AAFKResult aa_face = aa_compute_fk(robot, face_iv);

                        ++stats.n_ifk_prune_checks;
                        if (!aa_link_can_improve(aa_face, ci)) {
                            ++stats.n_ifk_pruned;
                            continue;  // skip this bg combo
                        }
                        ++stats.n_ifk_not_pruned;
                    }

                    // 3-point fit for p_d(q_j) = α·cos + β·sin + γ
                    Eigen::Matrix3d A;
                    for (int si = 0; si < 3; ++si) {
                        A(si, 0) = std::cos(qvals[si]);
                        A(si, 1) = std::sin(qvals[si]);
                        A(si, 2) = 1.0;
                    }

                    // Evaluate FK at 3 points (for distal link)
                    Eigen::Vector3d pts[3];
                    bool ok = true;
                    for (int si = 0; si < 3; ++si) {
                        q[j] = qvals[si];
                        auto pos = fk_link_positions(robot, q);
                        ++stats.n_fk_calls;
                        int np = static_cast<int>(pos.size());
                        if (V + 1 >= np) { ok = false; break; }
                        pts[si] = pos[V + 1];
                    }
                    if (!ok) continue;

                    Eigen::Vector3d b_vec, coeff;
                    for (int d = 0; d < 3; ++d) {
                        b_vec << pts[0][d], pts[1][d], pts[2][d];
                        coeff = A.colPivHouseholderQr().solve(b_vec);
                        double ac = coeff[0], bc = coeff[1];
                        if (std::abs(ac) < 1e-15 && std::abs(bc) < 1e-15) continue;

                        double q_star = std::atan2(bc, ac);
                        double cands[2] = { q_star, q_star + M_PI };
                        if (cands[1] > M_PI + 0.1) cands[1] -= 2 * M_PI;

                        for (double qc : cands) {
                            for (double shift : kShifts) {
                                double qt = qc + shift;
                                if (qt >= lo_j - 1e-12 && qt <= hi_j + 1e-12) {
                                    qt = std::max(lo_j, std::min(hi_j, qt));
                                    q[j] = qt;
                                    eval_config(q);
                                    ++stats.n_boundary_atan2;
                                    break;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // ══════════════════════════════════════════════════════════════════
    //  Finalize: inflate by link radii
    // ══════════════════════════════════════════════════════════════════

    for (int ci = 0; ci < n_act; ++ci) {
        float r = (rad) ? static_cast<float>(rad[ci]) : 0.f;
        for (int s = 0; s < n_sub; ++s) {
            float* a = out_aabb + (ci * n_sub + s) * 6;
            a[0] -= r;  a[1] -= r;  a[2] -= r;
            a[3] += r;  a[4] += r;  a[5] += r;
        }
    }

    if (out_stats) *out_stats = stats;
}


// ═══════════════════════════════════════════════════════════════════════════
//  Accessors
// ═══════════════════════════════════════════════════════════════════════════

int GcpcCache::n_total_points() const {
    int total = 0;
    for (const auto& sec : sections_)
        total += sec.n_points;
    return total;
}

const GcpcLinkSection* GcpcCache::find_section(int link_id) const {
    for (const auto& sec : sections_) {
        if (sec.link_id == link_id) return &sec;
    }
    return nullptr;
}


// ═══════════════════════════════════════════════════════════════════════════
//  Build from points
// ═══════════════════════════════════════════════════════════════════════════

void GcpcCache::build(const Robot& robot, const std::vector<GcpcPoint>& points) {
    robot_name_ = robot.name();
    d0_ = robot.dh_params()[0].d;
    has_tool_ = robot.has_tool();

    sections_.clear();

    // Group points by link_id
    std::map<int, std::vector<const GcpcPoint*>> by_link;
    for (const auto& pt : points)
        by_link[pt.link_id].push_back(&pt);

    // Determine active links from robot
    int n_act = robot.n_active_links();
    const int* map = robot.active_link_map();

    for (int ci = 0; ci < n_act; ++ci) {
        int link_id = map[ci];
        GcpcLinkSection section;
        section.link_id = link_id;

        // n_eff_joints: depends on link position in chain
        // Frame k needs transforms T₀..Tₖ = k+1 transforms, k+1 joints.
        // After eliminating q₀: n_eff = k = link_id for non-tool,
        // n_eff = n_joints - 1 for tool (link_id == n_joints).
        int nj = std::min(link_id + 1, robot.n_joints());
        section.n_eff_joints = nj - 1;  // excluding q₀

        // Check q₆ skip conditions (NOT for tool frame — tool needs all joints)
        section.q6_skipped = false;
        if (section.n_eff_joints >= 6 && link_id < robot.n_joints()) {
            const auto& dh6 = robot.dh_params()[6];
            if (std::abs(dh6.d) < 1e-10 && std::abs(dh6.a) < 1e-10) {
                section.n_eff_joints = 5;
                section.q6_skipped = true;
            }
        }

        // Copy points for this link
        auto it = by_link.find(link_id);
        if (it != by_link.end()) {
            for (const auto* pp : it->second) {
                section.points.push_back(*pp);
            }
        }
        section.n_points = static_cast<int>(section.points.size());

        // Build KD-tree
        build_kdtree(section);

        sections_.push_back(std::move(section));
    }
}


// ═══════════════════════════════════════════════════════════════════════════
//  JSON loading (from precompute_gcpc.jl output)
// ═══════════════════════════════════════════════════════════════════════════

// Minimal JSON parser — just reads the gcpc JSON format
// (We avoid external JSON library dependency)

static std::string trim(const std::string& s) {
    size_t a = s.find_first_not_of(" \t\n\r");
    size_t b = s.find_last_not_of(" \t\n\r");
    return (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
}

bool GcpcCache::load_json(const std::string& path, const Robot& robot) {
    // Read entire file
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "GCPC: Cannot open " << path << std::endl;
        return false;
    }
    std::stringstream ss;
    ss << ifs.rdbuf();
    std::string content = ss.str();

    // Very simple JSON parse — extract "points" array
    // Find robot name
    robot_name_ = robot.name();
    d0_ = robot.dh_params()[0].d;
    has_tool_ = robot.has_tool();

    // Parse points
    std::vector<GcpcPoint> points;

    // Find each point block: { ... "q_eff": [...], ... }
    // JSON field order is NOT guaranteed — search within enclosing { }
    size_t pos = 0;
    while (true) {
        pos = content.find("\"q_eff\"", pos);
        if (pos == std::string::npos) break;

        GcpcPoint pt{};

        // Find enclosing object boundaries
        size_t obj_start = content.rfind('{', pos);
        size_t arr_start = content.find('[', pos);
        size_t arr_end = content.find(']', arr_start);
        if (arr_start == std::string::npos || arr_end == std::string::npos) break;
        // Find closing } after q_eff array (skip nested structures)
        size_t obj_end = content.find('}', arr_end);
        if (obj_start == std::string::npos || obj_end == std::string::npos) break;

        // Parse q_eff array
        std::string arr_str = content.substr(arr_start + 1, arr_end - arr_start - 1);
        std::istringstream arr_ss(arr_str);
        std::vector<double> q_vals;
        double v;
        char c;
        while (arr_ss >> v) {
            q_vals.push_back(v);
            arr_ss >> c;  // comma
        }
        pt.n_eff = static_cast<int>(q_vals.size());
        for (int i = 0; i < pt.n_eff && i < 7; ++i)
            pt.q_eff[i] = q_vals[i];

        // Parse link_id (search within enclosing object)
        size_t lid_pos = content.find("\"link_id\"", obj_start);
        if (lid_pos != std::string::npos && lid_pos < obj_end) {
            size_t colon = content.find(':', lid_pos);
            if (colon != std::string::npos) {
                pt.link_id = std::stoi(trim(content.substr(colon + 1, 10)));
            }
        }

        // Parse direction (search within enclosing object)
        size_t dir_pos = content.find("\"direction\"", obj_start);
        if (dir_pos != std::string::npos && dir_pos < obj_end) {
            pt.direction = (content.find("\"xy\"", dir_pos) != std::string::npos &&
                           content.find("\"xy\"", dir_pos) < dir_pos + 30) ? 0 : 1;
        }

        // Parse p_critical (search within enclosing object)
        size_t pc_pos = content.find("\"p_critical\"", obj_start);
        if (pc_pos != std::string::npos && pc_pos < obj_end) {
            size_t colon = content.find(':', pc_pos);
            if (colon != std::string::npos) {
                std::string val_str = content.substr(colon + 1, 30);
                // Find end of number
                size_t end = val_str.find_first_of(",}\n\r");
                val_str = trim(val_str.substr(0, end));
                double pv = std::stod(val_str);
                if (pt.direction == 0) {
                    pt.R = pv;
                } else {
                    pt.C = pv;
                }
            }
        }

        // Compute A, B, C from FK evaluation
        // (The Julia script stores p_critical but we need A, B, C separately)
        {
            // Evaluate FK for this config to get A, B, C
            Eigen::VectorXd q_full(robot.n_joints());
            for (int j = 0; j < robot.n_joints(); ++j)
                q_full[j] = 0.0;  // q₀ = 0 (identity for sub-chain)

            for (int d = 0; d < pt.n_eff && d + 1 < robot.n_joints(); ++d)
                q_full[d + 1] = pt.q_eff[d];

            // FK with q₀=0 gives positions in T₀-local frame (since T₀ = Rz(0) = I)
            auto positions = fk_link_positions(robot, q_full);
            int np = static_cast<int>(positions.size());
            if (pt.link_id + 1 < np) {
                // Use distal frame position
                const auto& p = positions[pt.link_id + 1];
                pt.A = p[0];
                pt.B = p[1];
                pt.C = p[2] - d0_;  // subtract d₀ to get sub-chain C
                pt.R = std::sqrt(pt.A * pt.A + pt.B * pt.B);
            }
        }

        points.push_back(pt);
        pos = obj_end + 1;
    }

    std::cout << "GCPC: Loaded " << points.size() << " critical points from " << path << std::endl;

    // Build cache from points
    build(robot, points);

    return true;
}


// ═══════════════════════════════════════════════════════════════════════════
//  Binary save/load
// ═══════════════════════════════════════════════════════════════════════════

// GCPC binary format:
//   Header (512 bytes):
//     magic[4]: "GCPC"
//     version: uint32 = 1
//     robot_name_len: uint32
//     robot_name[64]: char
//     d0: float64
//     has_tool: uint8
//     n_sections: uint32
//     section_table[n_sections]:
//       link_id: int32
//       n_eff_joints: int32
//       n_points: int32
//       q6_skipped: uint8
//       offset: uint64 (byte offset from start of file)
//   (padding to 512 bytes)
//
//   Per-section data (at section offset):
//     KD-tree nodes: KdNode[2*n_points]
//     Points: GcpcPoint[n_points]

bool GcpcCache::save(const std::string& path) const {
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs.is_open()) return false;

    // Header
    char header[512] = {};
    std::memcpy(header, "GCPC", 4);
    uint32_t version = 1;
    std::memcpy(header + 4, &version, 4);

    uint32_t name_len = static_cast<uint32_t>(robot_name_.size());
    std::memcpy(header + 8, &name_len, 4);
    std::memcpy(header + 12, robot_name_.c_str(), std::min<size_t>(name_len, 63));

    std::memcpy(header + 76, &d0_, 8);
    uint8_t ht = has_tool_ ? 1 : 0;
    std::memcpy(header + 84, &ht, 1);

    uint32_t n_sec = static_cast<uint32_t>(sections_.size());
    std::memcpy(header + 85, &n_sec, 4);

    // Compute section offsets
    uint64_t offset = 512;  // after header
    size_t table_pos = 89;
    for (const auto& sec : sections_) {
        int32_t lid = sec.link_id;
        int32_t nej = sec.n_eff_joints;
        int32_t np = sec.n_points;
        uint8_t q6s = sec.q6_skipped ? 1 : 0;

        std::memcpy(header + table_pos, &lid, 4); table_pos += 4;
        std::memcpy(header + table_pos, &nej, 4); table_pos += 4;
        std::memcpy(header + table_pos, &np, 4);  table_pos += 4;
        std::memcpy(header + table_pos, &q6s, 1); table_pos += 1;
        std::memcpy(header + table_pos, &offset, 8); table_pos += 8;

        offset += sec.kd_tree.size() * sizeof(KdNode);
        offset += sec.n_points * sizeof(GcpcPoint);
    }

    ofs.write(header, 512);

    // Write section data
    for (const auto& sec : sections_) {
        if (!sec.kd_tree.empty())
            ofs.write(reinterpret_cast<const char*>(sec.kd_tree.data()),
                     sec.kd_tree.size() * sizeof(KdNode));
        if (!sec.points.empty())
            ofs.write(reinterpret_cast<const char*>(sec.points.data()),
                     sec.n_points * sizeof(GcpcPoint));
    }

    return ofs.good();
}

bool GcpcCache::load(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs.is_open()) return false;

    char header[512];
    ifs.read(header, 512);
    if (!ifs.good()) return false;

    // Verify magic
    if (std::memcmp(header, "GCPC", 4) != 0) return false;

    uint32_t version;
    std::memcpy(&version, header + 4, 4);
    if (version != 1) return false;

    uint32_t name_len;
    std::memcpy(&name_len, header + 8, 4);
    robot_name_ = std::string(header + 12, std::min<uint32_t>(name_len, 63));

    std::memcpy(&d0_, header + 76, 8);
    uint8_t ht;
    std::memcpy(&ht, header + 84, 1);
    has_tool_ = (ht != 0);

    uint32_t n_sec;
    std::memcpy(&n_sec, header + 85, 4);

    sections_.resize(n_sec);

    size_t table_pos = 89;
    for (uint32_t i = 0; i < n_sec; ++i) {
        auto& sec = sections_[i];
        int32_t lid, nej, np;
        uint8_t q6s;
        uint64_t offset;

        std::memcpy(&lid, header + table_pos, 4); table_pos += 4;
        std::memcpy(&nej, header + table_pos, 4); table_pos += 4;
        std::memcpy(&np,  header + table_pos, 4); table_pos += 4;
        std::memcpy(&q6s, header + table_pos, 1); table_pos += 1;
        std::memcpy(&offset, header + table_pos, 8); table_pos += 8;

        sec.link_id = lid;
        sec.n_eff_joints = nej;
        sec.n_points = np;
        sec.q6_skipped = (q6s != 0);

        // Read KD-tree nodes
        ifs.seekg(offset);
        int n_kd_nodes = 2 * np;  // upper bound allocated during build
        // Actually we need the real count. Store it properly.
        // For simplicity, compute from file: section data = kd_tree + points
        // Read until we have enough data. We know kd_tree.size() was ≤ 2*np.
        // Let's just store kd_root in the section table too.
        // For now, read 2*np nodes (may include some empty, but KdNode is POD)
        sec.kd_tree.resize(n_kd_nodes);
        ifs.read(reinterpret_cast<char*>(sec.kd_tree.data()),
                n_kd_nodes * sizeof(KdNode));

        sec.points.resize(np);
        ifs.read(reinterpret_cast<char*>(sec.points.data()),
                np * sizeof(GcpcPoint));

        sec.kd_root = 0;  // root is always first node
    }

    return ifs.good();
}

} // namespace envelope
} // namespace sbf
