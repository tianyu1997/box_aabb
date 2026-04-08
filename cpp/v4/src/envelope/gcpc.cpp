// ═══════════════════════════════════════════════════════════════════════════
// SafeBoxForest v4 — GCPC Cache Implementation
// ═══════════════════════════════════════════════════════════════════════════
// ═══════════════════════════════════════════════════════════════════════════
// KD-tree construction, range query, q₀ reconstruction, q₁ reflection,
// and the combined GCPC + boundary (kπ/2 + atan2) pipeline with
// two-level AA-based pruning.
// ═══════════════════════════════════════════════════════════════════════════
// Migrated from v3 gcpc_cache.cpp.
// Shared utilities (FixedRoots, solve_poly_in_interval, half_angle_to_q,
// build_bg_values, FKWorkspace) are now in analytical_utils.h.
// ═══════════════════════════════════════════════════════════════════════════
#define _USE_MATH_DEFINES
#include "sbf/envelope/gcpc.h"
#include "sbf/envelope/analytical_solve.h"  // boundary solver (Phases 0–2.5b)
#include "sbf/envelope/analytical_utils.h"
#include "sbf/envelope/analytical_coeff.h"
#include "sbf/robot/fk.h"
#include "sbf/robot/affine_fk.h"   // Affine-Arithmetic FK for tight pruning

#include <Eigen/Core>
#include <Eigen/QR>
#include <Eigen/Eigenvalues>       // companion-matrix polynomial solver
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <unordered_set>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sbf {
namespace envelope {

// ═══════════════════════════════════════════════════════════════════════════
//  GCPC-local utilities
// ═══════════════════════════════════════════════════════════════════════════

// Pre-computed 9×9 Vandermonde QR for degree-8 polynomial fitting
static Eigen::ColPivHouseholderQR<Eigen::Matrix<double,9,9>>& get_vandermonde_qr() {
    static Eigen::ColPivHouseholderQR<Eigen::Matrix<double,9,9>> qr = []() {
        Eigen::Matrix<double, 9, 9> V_mat;
        for (int k = 0; k < 9; ++k) {
            double t = -2.0 + k * 0.5;
            double pw = 1.0;
            for (int p = 0; p < 9; ++p) {
                V_mat(k, p) = pw;
                pw *= t;
            }
        }
        return V_mat.colPivHouseholderQr();
    }();
    return qr;
}

// ═══════════════════════════════════════════════════════════════════════════
//  q₀ reconstruction helpers
// ═══════════════════════════════════════════════════════════════════════════

double GcpcCache::reconstruct_q0_xy(double A, double B) {
    return std::atan2(-B, A);
}

bool GcpcCache::q0_in_range(double q0, double lo0, double hi0) {
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
    section.kd_tree.reserve(2 * n);

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
        node.split_dim = -1;
        node.split_val = 0.0;
        node.left = -1;
        node.right = -1;
        node.point_idx = indices[lo];
        return node_idx;
    }

    int n_eff = section.n_eff_joints;
    int dim = depth % n_eff;

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
    if (lo[dim] <= node.split_val + 1e-10) {
        kdtree_range_query(section, node.left, lo, hi, depth + 1, matched_indices);
    }
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

    double lo_q[7], hi_q[7];
    for (int d = 0; d < n_eff; ++d) {
        lo_q[d] = intervals[d + 1].lo;
        hi_q[d] = intervals[d + 1].hi;
    }

    // ── Step 1: Query the half-range [0,π] cache ────────────────────────
    double orig_lo_q1 = lo_q[0];
    double orig_hi_q1 = hi_q[0];

    double q1_lo_half = std::max(orig_lo_q1, 0.0);
    double q1_hi_half = std::min(orig_hi_q1, M_PI);

    std::vector<int> matched;

    if (q1_lo_half <= q1_hi_half + 1e-10) {
        lo_q[0] = q1_lo_half;
        hi_q[0] = q1_hi_half;
        kdtree_range_query(*sec, sec->kd_root, lo_q, hi_q, 0, matched);
    }

    for (int idx : matched) {
        const auto& pt = sec->points[idx];
        GcpcQueryResult res;
        res.point = &pt;
        res.q1_reflected = false;
        res.q1_actual = pt.q_eff[0];

        if (pt.direction == 0) {
            double q0_base = reconstruct_q0_xy(pt.A, pt.B);
            double q0_cands[4] = {
                q0_base,
                q0_base + M_PI,
                q0_base + HALF_PI,
                q0_base - HALF_PI
            };
            for (auto& qc : q0_cands) {
                if (qc > M_PI)  qc -= 2 * M_PI;
                if (qc < -M_PI) qc += 2 * M_PI;
            }
            bool found = false;
            for (int c = 0; c < 4; ++c) {
                if (q0_in_range(q0_cands[c], intervals[0].lo, intervals[0].hi)) {
                    if (!found) { res.q0_optimal = q0_cands[c]; found = true; }
                }
            }
            if (!found) continue;
        } else {
            res.q0_optimal = 0.5 * (intervals[0].lo + intervals[0].hi);
        }

        results.push_back(res);
    }

    // ── Step 2: q₁reflection ───────────────────────────────────────────
    double refl_lo = std::max(orig_lo_q1, -M_PI);
    double refl_hi = std::min(orig_hi_q1, 0.0);

    if (refl_lo <= refl_hi + 1e-10) {
        double cache_q1_lo = refl_lo + M_PI;
        double cache_q1_hi = refl_hi + M_PI;
        cache_q1_lo = std::max(cache_q1_lo, 0.0);
        cache_q1_hi = std::min(cache_q1_hi, M_PI);

        if (cache_q1_lo <= cache_q1_hi + 1e-10) {
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
                res.q1_actual = pt.q_eff[0] - M_PI;

                if (pt.direction == 0) {
                    double A_refl = -pt.A;
                    double B_refl = pt.B;
                    double q0_base = reconstruct_q0_xy(A_refl, B_refl);
                    double q0_cands[4] = {
                        q0_base,
                        q0_base + M_PI,
                        q0_base + HALF_PI,
                        q0_base - HALF_PI
                    };
                    for (auto& qc : q0_cands) {
                        if (qc > M_PI)  qc -= 2 * M_PI;
                        if (qc < -M_PI) qc += 2 * M_PI;
                    }
                    bool found = false;
                    for (int c = 0; c < 4; ++c) {
                        if (q0_in_range(q0_cands[c], intervals[0].lo, intervals[0].hi)) {
                            if (!found) { res.q0_optimal = q0_cands[c]; found = true; }
                        }
                    }
                    if (!found) continue;
                } else {
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
//  Architecture (refactored):
//    Step 1 — Analytical boundary solver (Phases 0–2.5b, no interior)
//             Delegates all boundary-related solving to derive_aabb_critical_analytical()
//             with enable_interior_solve=false. This ensures full consistency with
//             the Analytical pipeline for boundary enumeration.
//    Step 2 — Phase A: Cache lookup (interior critical points from KD-tree)
//             Merges cached interior positions into the boundary AABB.
//             Interior values use cache reuse: no runtime enumeration.
// ═══════════════════════════════════════════════════════════════════════════
//  Both steps produce radius-inflated out_aabb, and raw out_endpoint_iaabb.
//  Merging is via min/max — both sources are sound overapproximations,
//  so the union preserves soundness while being tighter than either alone.
// ═══════════════════════════════════════════════════════════════════════════

void GcpcCache::derive_aabb_with_gcpc(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    float* out_aabb,
    GcpcQueryStats* out_stats,
    float* out_endpoint_iaabb) const
{
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* map = robot.active_link_map();
    const double* rad = robot.active_link_radii();
    const float inv_n = 1.0f / static_cast<float>(n_sub);

    GcpcQueryStats stats{};

// ═══════════════════════════════════════════════════════════════════════════
    //  Step 1: Analytical boundary solver (Phases 0–2.5b, no interior)
    //  Produces radius-inflated out_aabb and raw out_endpoint_iaabb.
    //  Analytical handles initialization of both output arrays internally.
// ═══════════════════════════════════════════════════════════════════════════

    auto _tBoundary = std::chrono::high_resolution_clock::now();

    AnalyticalCriticalConfig acfg = AnalyticalCriticalConfig::all_enabled();
    acfg.enable_interior_solve = false;
    acfg.dual_phase3 = false;

    AnalyticalCriticalStats anal_stats{};
    derive_aabb_critical_analytical(robot, intervals, n_sub, acfg,
                                    out_aabb, &anal_stats, out_endpoint_iaabb);

    stats.analytical_boundary_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - _tBoundary).count();

    // Map Analytical stats to GCPC stats for telemetry
    stats.n_boundary_kpi2  = anal_stats.n_phase0_vertices;
    stats.n_boundary_atan2 = anal_stats.n_phase1_edges;
    stats.n_phase_d_faces  = anal_stats.n_phase2_faces;
    stats.n_phase_e_pair1d = anal_stats.n_phase25a_pair1d;
    stats.n_phase_f_pair2d = anal_stats.n_phase25b_pair2d;

// ═══════════════════════════════════════════════════════════════════════════
    //  Step 2: Phase A — Cache lookup (interior critical points)
    //  Merges cached interior positions into already-initialized out_aabb.
    //  out_aabb is radius-inflated from Analytical, so Phase A writes
    //  (pos - r) for min and (pos + r) for max for correct comparison.
    //  out_endpoint_iaabb remains raw (no radius), consistent with
    //  Analytical's treatment.
// ═══════════════════════════════════════════════════════════════════════════

    auto _tA = std::chrono::high_resolution_clock::now();

    FKWorkspace ws;

    // Helper: update AABBs from CURRENT ws state.
    // Writes radius-inflated positions to out_aabb (consistent with Analytical output).
    // Writes raw positions to out_endpoint_iaabb (no radius).
    auto update_from_ws = [&](const Eigen::VectorXd& q) {
        ++stats.n_fk_calls;

        for (int ci = 0; ci < n_act; ++ci) {
            int V = map[ci];
            if (V + 1 >= ws.np) continue;
            float r = (rad) ? static_cast<float>(rad[ci]) : 0.f;
            const Eigen::Vector3d p_prox = ws.pos(V);
            const Eigen::Vector3d p_dist = ws.pos(V + 1);
            for (int s = 0; s < n_sub; ++s) {
                float t0 = s * inv_n;
                float t1 = (s + 1) * inv_n;
                Eigen::Vector3d s0 = p_prox + double(t0) * (p_dist - p_prox);
                Eigen::Vector3d s1 = p_prox + double(t1) * (p_dist - p_prox);

                float* a = out_aabb + (ci * n_sub + s) * 6;
                for (int d = 0; d < 3; ++d) {
                    float vmin = static_cast<float>(std::min(s0[d], s1[d])) - r;
                    float vmax = static_cast<float>(std::max(s0[d], s1[d])) + r;
                    if (vmin < a[d])     a[d]     = vmin;
                    if (vmax > a[d + 3]) a[d + 3] = vmax;
                }

                // Track per-endpoint interval iAABBs for hull16 (raw, no radius)
                if (out_endpoint_iaabb) {
                    float* fa0 = out_endpoint_iaabb + (ci * (n_sub + 1) + s) * 6;
                    float* fa1 = out_endpoint_iaabb + (ci * (n_sub + 1) + s + 1) * 6;
                    for (int d = 0; d < 3; ++d) {
                        float v0 = static_cast<float>(s0[d]);
                        float v1 = static_cast<float>(s1[d]);
                        if (v0 < fa0[d])     fa0[d]     = v0;
                        if (v0 > fa0[d + 3]) fa0[d + 3] = v0;
                        if (v1 < fa1[d])     fa1[d]     = v1;
                        if (v1 > fa1[d + 3]) fa1[d + 3] = v1;
                    }
                }
            }
        }
    };

    // Helper: full FK compute + update. Used by Phase A.
    auto eval_config = [&](const Eigen::VectorXd& q) {
        ws.compute(robot, q);
        update_from_ws(q);
    };

    // H6-E: Cap Phase A reconstructions per link to limit overhead at wide widths.
    static constexpr int MAX_PHASE_A_EVALS_PER_LINK = 200;

    for (int ci = 0; ci < n_act; ++ci) {
        int link_id = map[ci];
        std::vector<GcpcQueryResult> results;
        query_link(link_id, intervals.data(), results);
        stats.n_cache_matches += static_cast<int>(results.size());

        const int n_eval_a = std::min(static_cast<int>(results.size()),
                                       MAX_PHASE_A_EVALS_PER_LINK);
        for (int ri = 0; ri < n_eval_a; ++ri) {
            const auto& res = results[ri];
            const auto& pt = *res.point;
            if (res.q1_reflected) ++stats.n_q1_reflected;
            ++stats.n_q0_valid;

            Eigen::VectorXd q(n);
            for (int j = 0; j < n; ++j) q[j] = intervals[j].mid();

            q[0] = res.q0_optimal;
            for (int d = 0; d < pt.n_eff; ++d) {
                if (res.q1_reflected && d == 0) {
                    q[d + 1] = res.q1_actual;
                } else {
                    q[d + 1] = pt.q_eff[d];
                }
            }

            eval_config(q);

            if (pt.direction == 0) {
                double q0_opp = res.q0_optimal + M_PI;
                if (q0_opp > M_PI) q0_opp -= 2 * M_PI;
                if (q0_in_range(q0_opp, intervals[0].lo, intervals[0].hi)) {
                    q[0] = q0_opp;
                    eval_config(q);
                }
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

    stats.phase_a_ms = std::chrono::duration<double, std::milli>(
        std::chrono::high_resolution_clock::now() - _tA).count();

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

    std::map<int, std::vector<const GcpcPoint*>> by_link;
    for (const auto& pt : points)
        by_link[pt.link_id].push_back(&pt);

    int n_act = robot.n_active_links();
    const int* map = robot.active_link_map();

    for (int ci = 0; ci < n_act; ++ci) {
        int link_id = map[ci];
        GcpcLinkSection section;
        section.link_id = link_id;

        int nj = std::min(link_id + 1, robot.n_joints());
        section.n_eff_joints = nj - 1;

        section.q6_skipped = false;
        if (section.n_eff_joints >= 6 && link_id < robot.n_joints()) {
            const auto& dh6 = robot.dh_params()[6];
            if (std::abs(dh6.d) < 1e-10 && std::abs(dh6.a) < 1e-10) {
                section.n_eff_joints = 5;
                section.q6_skipped = true;
            }
        }

        auto it = by_link.find(link_id);
        if (it != by_link.end()) {
            for (const auto* pp : it->second) {
                section.points.push_back(*pp);
            }
        }
        section.n_points = static_cast<int>(section.points.size());

        build_kdtree(section);

        sections_.push_back(std::move(section));
    }
}


// ═══════════════════════════════════════════════════════════════════════════
//  Precompute Phase E/F critical configs with kπ/2-only backgrounds
// ═══════════════════════════════════════════════════════════════════════════

void GcpcCache::precompute_pair_kpi2(const Robot& robot) {
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* link_map = robot.active_link_map();
    auto limits = robot.joint_limits();

    // Build kπ/2 values for each joint (full range)
    std::vector<std::vector<double>> kpi2_vals(n);
    for (int j = 0; j < n; ++j) {
        double lo = limits.limits[j].lo, hi = limits.limits[j].hi;
        for (int k = -20; k <= 20; ++k) {
            double a = k * HALF_PI;
            if (a > lo + 1e-12 && a < hi - 1e-12) {
                kpi2_vals[j].push_back(a);
            }
        }
    }

    // Build pair lists (same as derive_aabb_with_gcpc Phase E/F)
    auto coupled = robot.coupled_pairs();
    static constexpr int MAX_PAIRS = 64;
    std::pair<int,int> pc_pairs[MAX_PAIRS];
    int n_pc_pairs = 0;
    for (auto& fp : coupled) {
        if (n_pc_pairs < MAX_PAIRS) pc_pairs[n_pc_pairs++] = fp;
    }
    for (int i = 0; i < n - 1; ++i) {
        std::pair<int,int> p = {i, i+1};
        bool dup = false;
        for (int k = 0; k < n_pc_pairs; ++k)
            if (pc_pairs[k] == p) { dup = true; break; }
        if (!dup && n_pc_pairs < MAX_PAIRS) pc_pairs[n_pc_pairs++] = p;
    }
    for (int i = 0; i < n - 2; i += 2) {
        std::pair<int,int> p = {i, i+2};
        bool dup = false;
        for (int k = 0; k < n_pc_pairs; ++k)
            if (pc_pairs[k] == p) { dup = true; break; }
        if (!dup && n_pc_pairs < MAX_PAIRS) pc_pairs[n_pc_pairs++] = p;
    }

    FKWorkspace ws;
    ws.resize(n + 2);
    Eigen::VectorXd q(n);

    for (int sec_idx = 0; sec_idx < static_cast<int>(sections_.size()); ++sec_idx) {
        auto& section = sections_[sec_idx];
        int V = section.link_id;
        int nj = std::min(V + 1, n);
        if (nj < 2) continue;

        section.pair_kpi2_configs.clear();

        auto add_config = [&](const Eigen::VectorXd& q_cfg) {
            // Dedup: check if this config is already stored
            for (const auto& existing : section.pair_kpi2_configs) {
                bool same = true;
                for (int j = 0; j < nj && same; ++j) {
                    if (std::abs(existing.q[j] - q_cfg[j]) > 1e-8) same = false;
                }
                if (same) return;
            }
            PairKpi2Config cfg;
            cfg.n_joints = nj;
            for (int j = 0; j < nj; ++j) cfg.q[j] = q_cfg[j];
            for (int j = nj; j < 7; ++j) cfg.q[j] = 0.0;
            section.pair_kpi2_configs.push_back(cfg);
        };

        // ── Phase E precompute: Pair-Constrained 1D with kπ/2 bg ────
        for (int fp_i = 0; fp_i < n_pc_pairs; ++fp_i) {
            int pi = pc_pairs[fp_i].first, pj = pc_pairs[fp_i].second;
            if (pi >= nj || pj >= nj) continue;
            double lo_i = limits.limits[pi].lo, hi_i = limits.limits[pi].hi;
            double lo_j = limits.limits[pj].lo, hi_j = limits.limits[pj].hi;
            if (hi_i - lo_i < 1e-12 || hi_j - lo_j < 1e-12) continue;

            double sum_lo = lo_i + lo_j, sum_hi = hi_i + hi_j;
            int k_lo = static_cast<int>(std::ceil(sum_lo / HALF_PI - 1e-9));
            int k_hi = static_cast<int>(std::floor(sum_hi / HALF_PI + 1e-9));

            for (int kk = k_lo; kk <= k_hi; ++kk) {
                double C = kk * HALF_PI;
                double eff_lo = std::max(lo_i, C - hi_j);
                double eff_hi = std::min(hi_i, C - lo_j);
                if (eff_hi - eff_lo < 1e-12) continue;

                // Enumerate kπ/2-only bg combinations
                constexpr int MAX_OTHER = 12;
                int other_idx[MAX_OTHER];
                int n_other = 0;
                long long total_bg = 1;
                bool overflow = false;
                for (int jj = 0; jj < nj; ++jj) {
                    if (jj == pi || jj == pj) continue;
                    if (n_other >= MAX_OTHER) break;
                    other_idx[n_other] = jj;
                    int sz = static_cast<int>(kpi2_vals[jj].size());
                    if (sz == 0) { overflow = true; break; }
                    total_bg *= sz;
                    ++n_other;
                    if (total_bg > 64) { overflow = true; break; }
                }
                if (overflow || total_bg == 0) continue;

                int bg_counter[MAX_OTHER] = {};
                int j_lo_e = std::min(pi, pj);
                int j_hi_e = std::max(pi, pj);
                const auto& dh_lo_e = robot.dh_params()[j_lo_e];
                const auto& dh_hi_e = robot.dh_params()[j_hi_e];
                double cosC = std::cos(C), sinC = std::sin(C);

                for (long long bgc = 0; bgc < total_bg; ++bgc) {
                    for (int jj = 0; jj < n; ++jj) q[jj] = 0.0;
                    for (int oi = 0; oi < n_other; ++oi)
                        q[other_idx[oi]] = kpi2_vals[other_idx[oi]][bg_counter[oi]];

                    // P2.5a direct coefficient extraction
                    ws.set_identity();
                    for (int k = 0; k < j_lo_e; ++k) ws.compute_joint(robot, q, k);
                    const Eigen::Matrix4d& prefix_e = ws.tf[j_lo_e];
                    Eigen::Matrix4d mid_e = compute_middle_matrix(robot, q, j_lo_e, j_hi_e);

                    for (int eval_idx = 0; eval_idx < 2; ++eval_idx) {
                        int eval_link = (eval_idx == 0) ? V + 1 : V;
                        if (eval_link < 1) continue;

                        Eigen::Vector3d suf_e = compute_suffix_pos(robot, q, j_hi_e, eval_link);
                        Coeff2D c2d = extract_2d_coefficients(prefix_e, dh_lo_e, mid_e, dh_hi_e, suf_e);

                        for (int d = 0; d < 3; ++d) {
                            double alpha_c, beta_c;
                            if (pi <= pj) {
                                alpha_c = c2d.a[4][d] + c2d.a[6][d]*cosC + c2d.a[7][d]*sinC;
                                beta_c  = c2d.a[5][d] + c2d.a[6][d]*sinC - c2d.a[7][d]*cosC;
                            } else {
                                alpha_c = c2d.a[4][d]*cosC + c2d.a[5][d]*sinC + c2d.a[6][d];
                                beta_c  = c2d.a[4][d]*sinC - c2d.a[5][d]*cosC + c2d.a[7][d];
                            }
                            if (std::abs(alpha_c) < 1e-15 && std::abs(beta_c) < 1e-15) continue;

                            double q_star = std::atan2(beta_c, alpha_c);
                            double cands[2] = { q_star, q_star + M_PI };
                            if (cands[1] > M_PI + 0.1) cands[1] -= 2 * M_PI;

                            for (double qc : cands) {
                                for (double shift : kShifts) {
                                    double qi_test = qc + shift;
                                    if (qi_test >= eff_lo - 1e-10 && qi_test <= eff_hi + 1e-10) {
                                        qi_test = std::max(eff_lo, std::min(eff_hi, qi_test));
                                        double qj_test = C - qi_test;
                                        if (qj_test < lo_j - 1e-10 || qj_test > hi_j + 1e-10) continue;
                                        qj_test = std::max(lo_j, std::min(hi_j, qj_test));
                                        q[pi] = qi_test; q[pj] = qj_test;
                                        add_config(q);
                                        break;
                                    }
                                }
                            }
                        }
                    }

                    for (int oi = n_other - 1; oi >= 0; --oi) {
                        if (++bg_counter[oi] < static_cast<int>(kpi2_vals[other_idx[oi]].size())) break;
                        bg_counter[oi] = 0;
                    }
                }
            }
        }

        // ── Phase F precompute: Pair-Constrained 2D with kπ/2 bg ────
        for (int fp_i = 0; fp_i < n_pc_pairs; ++fp_i) {
            int pi = pc_pairs[fp_i].first, pj = pc_pairs[fp_i].second;
            if (pi >= nj || pj >= nj) continue;
            double lo_i = limits.limits[pi].lo, hi_i = limits.limits[pi].hi;
            double lo_j = limits.limits[pj].lo, hi_j = limits.limits[pj].hi;
            if (hi_i - lo_i < 1e-12 || hi_j - lo_j < 1e-12) continue;

            double sum_lo = lo_i + lo_j, sum_hi = hi_i + hi_j;
            int k_lo = static_cast<int>(std::ceil(sum_lo / HALF_PI - 1e-9));
            int k_hi = static_cast<int>(std::floor(sum_hi / HALF_PI + 1e-9));

            for (int kk = k_lo; kk <= k_hi; ++kk) {
                double C = kk * HALF_PI;
                double eff_lo = std::max(lo_i, C - hi_j);
                double eff_hi = std::min(hi_i, C - lo_j);
                if (eff_hi - eff_lo < 1e-12) continue;

                for (int pm = 0; pm < nj; ++pm) {
                    if (pm == pi || pm == pj) continue;
                    double lo_m = limits.limits[pm].lo, hi_m = limits.limits[pm].hi;
                    if (hi_m - lo_m < 1e-12) continue;

                    double tm_lo = std::tan(lo_m * 0.5), tm_hi = std::tan(hi_m * 0.5);
                    if (tm_lo > tm_hi) std::swap(tm_lo, tm_hi);

                    constexpr int MAX_OTHER = 12;
                    int other_idx_f[MAX_OTHER];
                    int n_other_f = 0;
                    long long total_bg_f = 1;
                    bool overflow_f = false;
                    for (int jj = 0; jj < nj; ++jj) {
                        if (jj == pi || jj == pj || jj == pm) continue;
                        if (n_other_f >= MAX_OTHER) break;
                        other_idx_f[n_other_f] = jj;
                        int sz = static_cast<int>(kpi2_vals[jj].size());
                        if (sz == 0) { overflow_f = true; break; }
                        total_bg_f *= sz;
                        ++n_other_f;
                        if (total_bg_f > 64) { overflow_f = true; break; }
                    }
                    if (overflow_f || total_bg_f == 0) continue;
                    if (n_other_f == 0) total_bg_f = 1; // no bg joints

                    double qi_vals[3] = { eff_lo, 0.5*(eff_lo+eff_hi), eff_hi };
                    double qm_vals[3] = { lo_m, 0.5*(lo_m+hi_m), hi_m };

                    Eigen::Matrix<double, 9, 9> A_mat;
                    { int row = 0;
                      for (int si = 0; si < 3; ++si) {
                        double ci_v = std::cos(qi_vals[si]), si_v = std::sin(qi_vals[si]);
                        for (int sm = 0; sm < 3; ++sm) {
                            double cm_v = std::cos(qm_vals[sm]), sm_v = std::sin(qm_vals[sm]);
                            A_mat(row,0)=ci_v*cm_v; A_mat(row,1)=ci_v*sm_v;
                            A_mat(row,2)=si_v*cm_v; A_mat(row,3)=si_v*sm_v;
                            A_mat(row,4)=ci_v; A_mat(row,5)=si_v;
                            A_mat(row,6)=cm_v; A_mat(row,7)=sm_v;
                            A_mat(row,8)=1.0; ++row;
                    }}}
                    auto qr_f = A_mat.colPivHouseholderQr();

                    int sj[3] = {pi, pj, pm};
                    std::sort(sj, sj + 3);
                    const int j1 = sj[0], j2 = sj[1], j3 = sj[2];
                    const bool j1_is_pm = (j1 == pm);
                    const bool j2_is_pm = (j2 == pm);
                    const bool j3_is_pm = (j3 == pm);

                    auto build_tf = [&](int joint, double qv) -> Eigen::Matrix4d {
                        const auto& dh = robot.dh_params()[joint];
                        double th = (dh.joint_type == 0) ? qv + dh.theta : dh.theta;
                        double dv = (dh.joint_type == 0) ? dh.d : qv + dh.d;
                        return dh_transform(dh.alpha, dh.a, dv, th);
                    };
                    Eigen::Matrix4d tf1[3], tf2[3], tf3[3];
                    for (int k = 0; k < 3; ++k) {
                        double q1k = j1_is_pm ? qm_vals[k] : (j1==pi ? qi_vals[k] : C - qi_vals[k]);
                        tf1[k] = build_tf(j1, q1k);
                        double q2k = j2_is_pm ? qm_vals[k] : (j2==pi ? qi_vals[k] : C - qi_vals[k]);
                        tf2[k] = build_tf(j2, q2k);
                        double q3k = j3_is_pm ? qm_vals[k] : (j3==pi ? qi_vals[k] : C - qi_vals[k]);
                        tf3[k] = build_tf(j3, q3k);
                    }

                    int bg_cnt_f[MAX_OTHER] = {};
                    int min_free_f = std::min({pi, pj, pm});

                    for (long long bgc = 0; bgc < total_bg_f; ++bgc) {
                        for (int jj = 0; jj < n; ++jj) q[jj] = 0.0;
                        for (int oi = 0; oi < n_other_f; ++oi)
                            q[other_idx_f[oi]] = kpi2_vals[other_idx_f[oi]][bg_cnt_f[oi]];

                        ws.set_identity();
                        for (int k = 0; k < min_free_f; ++k) ws.compute_joint(robot, q, k);

                        Eigen::Matrix4d mid12 = compute_middle_matrix(robot, q, j1, j2);
                        Eigen::Matrix4d mid23 = compute_middle_matrix(robot, q, j2, j3);
                        const Eigen::Matrix4d& prefix_f = ws.tf[j1];

                        for (int eval_idx = 0; eval_idx < 2; ++eval_idx) {
                            int eval_link = (eval_idx == 0) ? V + 1 : V;
                            if (eval_link < 1) continue;

                            Eigen::Vector3d suf = compute_suffix_pos(robot, q, j3, eval_link);
                            Eigen::Matrix4d left[3];
                            for (int k = 0; k < 3; ++k) left[k] = prefix_f * tf1[k] * mid12;
                            Eigen::Vector4d sv; sv << suf, 1.0;
                            Eigen::Vector4d right[3];
                            for (int k = 0; k < 3; ++k) right[k] = mid23 * (tf3[k] * sv);

                            Eigen::Vector3d pos9[9];
                            int row = 0;
                            for (int si = 0; si < 3; ++si)
                                for (int sm = 0; sm < 3; ++sm) {
                                    int k1 = j1_is_pm ? sm : si;
                                    int k2 = j2_is_pm ? sm : si;
                                    int k3 = j3_is_pm ? sm : si;
                                    pos9[row++] = (left[k1] * (tf2[k2] * right[k3])).head<3>();
                                }

                            Eigen::Matrix<double, 9, 3> B;
                            for (int r = 0; r < 9; ++r) for (int dd = 0; dd < 3; ++dd) B(r,dd) = pos9[r][dd];
                            Eigen::Matrix<double, 9, 3> coeff = qr_f.solve(B);

                            for (int d = 0; d < 3; ++d) {
                                double a1=coeff(0,d),a2=coeff(1,d),a3=coeff(2,d),a4=coeff(3,d);
                                double a5=coeff(4,d),a6=coeff(5,d),a7=coeff(6,d),a8=coeff(7,d);

                                double pa[9];
                                build_symbolic_poly8(a1,a2,a3,a4,a5,a6,a7,a8,pa);
                                FixedRoots roots;
                                solve_poly_in_interval(pa, 8, tm_lo, tm_hi, roots);

                                for (double tm_r : roots) {
                                    double qm_cand;
                                    if (!half_angle_to_q(tm_r, lo_m, hi_m, qm_cand)) continue;
                                    double tm2=tm_r*tm_r, den=1.0+tm2;
                                    double cm=(1.0-tm2)/den, sm=2.0*tm_r/den;
                                    double P=a3*cm+a4*sm+a6, Q=a1*cm+a2*sm+a5;
                                    double qi_cand=std::atan2(P,Q);

                                    double qi_tries[3]={qi_cand,qi_cand+M_PI,qi_cand-M_PI};
                                    for (double qi_try : qi_tries) {
                                        for (double shift : kShifts) {
                                            double qi_test = qi_try + shift;
                                            if (qi_test >= eff_lo-1e-10 && qi_test <= eff_hi+1e-10) {
                                                qi_test = std::max(eff_lo, std::min(eff_hi, qi_test));
                                                double qj_test = C - qi_test;
                                                if (qj_test < lo_j-1e-10 || qj_test > hi_j+1e-10) continue;
                                                qj_test = std::max(lo_j, std::min(hi_j, qj_test));
                                                q[pi]=qi_test; q[pj]=qj_test; q[pm]=qm_cand;
                                                add_config(q);
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                        }

                        for (int oi = n_other_f - 1; oi >= 0; --oi) {
                            if (++bg_cnt_f[oi] < static_cast<int>(kpi2_vals[other_idx_f[oi]].size())) break;
                            bg_cnt_f[oi] = 0;
                        }
                    }
                }
            }
        }
    } // for sec_idx
}


// ═══════════════════════════════════════════════════════════════════════════
//  Enrich cache with interior critical points (coordinate descent)
// ═══════════════════════════════════════════════════════════════════════════

int GcpcCache::enrich_with_interior_search(
    const Robot& robot,
    int n_random_seeds,
    int max_sweeps)
{
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* link_map = robot.active_link_map();
    auto limits = robot.joint_limits();

    int total_added = 0;

    auto q_eff_hash = [](const GcpcPoint& pt) -> uint64_t {
        uint64_t h = 0;
        for (int d = 0; d < pt.n_eff; ++d) {
            int64_t q = static_cast<int64_t>(pt.q_eff[d] * 1e6);
            h ^= static_cast<uint64_t>(q * 2654435761ULL + d * 40503ULL);
        }
        return h ^ (static_cast<uint64_t>(pt.direction) * 7919ULL);
    };

    std::vector<std::vector<double>> kpi2_vals(n);
    for (int j = 0; j < n; ++j) {
        double lo = limits.limits[j].lo, hi = limits.limits[j].hi;
        for (int k = -20; k <= 20; ++k) {
            double a = k * HALF_PI;
            if (a >= lo - 1e-10 && a <= hi + 1e-10) {
                a = std::max(lo, std::min(hi, a));
                kpi2_vals[j].push_back(a);
            }
        }
        if (kpi2_vals[j].empty()) {
            kpi2_vals[j].push_back(0.5 * (lo + hi));
        }
        std::sort(kpi2_vals[j].begin(), kpi2_vals[j].end());
        kpi2_vals[j].erase(
            std::unique(kpi2_vals[j].begin(), kpi2_vals[j].end()),
            kpi2_vals[j].end());
    }

    auto coupled = robot.coupled_pairs();
    std::vector<std::pair<int,int>> all_pairs = coupled;
    for (int i = 0; i < n - 1; ++i) {
        std::pair<int,int> p = {i, i+1};
        bool dup = false;
        for (auto& fp : all_pairs) if (fp == p) { dup = true; break; }
        if (!dup) all_pairs.push_back(p);
    }
    for (int i = 0; i < n - 2; i += 2) {
        std::pair<int,int> p = {i, i+2};
        bool dup = false;
        for (auto& fp : all_pairs) if (fp == p) { dup = true; break; }
        if (!dup) all_pairs.push_back(p);
    }

    std::mt19937 rng(42);

    for (int sec_idx = 0; sec_idx < static_cast<int>(sections_.size()); ++sec_idx) {
        auto& section = sections_[sec_idx];
        int link_id = section.link_id;
        int n_eff = section.n_eff_joints;
        if (n_eff < 2) continue;

        int nj = n_eff + 1;

        std::unordered_set<uint64_t> existing_hashes;
        for (const auto& pt : section.points)
            existing_hashes.insert(q_eff_hash(pt));

        std::vector<GcpcPoint> new_points;

        auto coordinate_descent = [&](Eigen::VectorXd& q_full, int direction) {
            for (int sweep = 0; sweep < max_sweeps; ++sweep) {
                bool changed = false;

                for (int eff_j = 0; eff_j < n_eff; ++eff_j) {
                    int j = eff_j + 1;
                    double lo_j = limits.limits[j].lo;
                    double hi_j = limits.limits[j].hi;
                    if (hi_j - lo_j < 1e-12) continue;

                    double mid_j = 0.5 * (lo_j + hi_j);
                    double qvals[3] = { lo_j, mid_j, hi_j };

                    Eigen::Matrix3d A_trig;
                    for (int si = 0; si < 3; ++si) {
                        A_trig(si, 0) = std::cos(qvals[si]);
                        A_trig(si, 1) = std::sin(qvals[si]);
                        A_trig(si, 2) = 1.0;
                    }
                    auto qr_A = A_trig.colPivHouseholderQr();

                    if (direction == 0) {
                        double q_save = q_full[j];
                        double best_R = -1;
                        double best_q = q_save;

                        for (int d = 0; d < 2; ++d) {
                            Eigen::Vector3d vals;
                            for (int si = 0; si < 3; ++si) {
                                q_full[j] = qvals[si];
                                auto pos = fk_link_positions(robot, q_full);
                                if (link_id + 1 < static_cast<int>(pos.size()))
                                    vals[si] = pos[link_id + 1][d];
                                else
                                    vals[si] = 0;
                            }
                            q_full[j] = q_save;

                            Eigen::Vector3d coeff = qr_A.solve(vals);
                            double ac = coeff[0], bc = coeff[1];
                            if (std::abs(ac) < 1e-15 && std::abs(bc) < 1e-15) continue;

                            double q_star = std::atan2(bc, ac);
                            double cands[] = {q_star, q_star + M_PI, q_star - M_PI};
                            for (double qc : cands) {
                                for (double shift : kShifts) {
                                    double qt = qc + shift;
                                    if (qt >= lo_j - 1e-10 && qt <= hi_j + 1e-10) {
                                        qt = std::max(lo_j, std::min(hi_j, qt));
                                        q_full[j] = qt;
                                        auto pos = fk_link_positions(robot, q_full);
                                        if (link_id + 1 < static_cast<int>(pos.size())) {
                                            double px = pos[link_id + 1][0];
                                            double py = pos[link_id + 1][1];
                                            double R = std::sqrt(px*px + py*py);
                                            if (R > best_R) { best_R = R; best_q = qt; }
                                        }
                                        q_full[j] = q_save;
                                        break;
                                    }
                                }
                            }
                        }

                        // Also try min R
                        for (int d = 0; d < 2; ++d) {
                            Eigen::Vector3d vals;
                            for (int si = 0; si < 3; ++si) {
                                q_full[j] = qvals[si];
                                auto pos = fk_link_positions(robot, q_full);
                                if (link_id + 1 < static_cast<int>(pos.size()))
                                    vals[si] = pos[link_id + 1][d];
                                else
                                    vals[si] = 0;
                            }
                            q_full[j] = q_save;

                            Eigen::Vector3d coeff = qr_A.solve(vals);
                            double ac = coeff[0], bc = coeff[1];
                            if (std::abs(ac) < 1e-15 && std::abs(bc) < 1e-15) continue;

                            double q_star = std::atan2(bc, ac) + M_PI;
                            for (double shift : kShifts) {
                                double qt = q_star + shift;
                                if (qt >= lo_j - 1e-10 && qt <= hi_j + 1e-10) {
                                    qt = std::max(lo_j, std::min(hi_j, qt));
                                    if (std::abs(qt - best_q) > 1e-6) {
                                        changed = true;
                                    }
                                    break;
                                }
                            }
                        }

                        if (std::abs(best_q - q_save) > 1e-8) {
                            q_full[j] = best_q;
                            changed = true;
                        }

                    } else {
                        // direction == 1: optimize pz
                        Eigen::Vector3d vals;
                        double q_save = q_full[j];
                        for (int si = 0; si < 3; ++si) {
                            q_full[j] = qvals[si];
                            auto pos = fk_link_positions(robot, q_full);
                            if (link_id + 1 < static_cast<int>(pos.size()))
                                vals[si] = pos[link_id + 1][2];
                            else
                                vals[si] = 0;
                        }
                        q_full[j] = q_save;

                        Eigen::Vector3d coeff = qr_A.solve(vals);
                        double ac = coeff[0], bc = coeff[1];
                        if (std::abs(ac) < 1e-15 && std::abs(bc) < 1e-15) continue;

                        double q_star = std::atan2(bc, ac);
                        double cands[] = {q_star, q_star + M_PI};
                        bool found = false;
                        for (double qc : cands) {
                            for (double shift : kShifts) {
                                double qt = qc + shift;
                                if (qt >= lo_j - 1e-10 && qt <= hi_j + 1e-10) {
                                    qt = std::max(lo_j, std::min(hi_j, qt));
                                    if (std::abs(qt - q_save) > 1e-8) {
                                        q_full[j] = qt;
                                        changed = true;
                                    }
                                    found = true;
                                    break;
                                }
                            }
                            if (found) break;
                        }
                    }
                }

                if (!changed) break;
            }

            // Extract converged point 鈫?GcpcPoint
            double q1_val = q_full[1];
            bool reflected = false;
            if (q1_val < -1e-10) {
                q1_val += M_PI;
                reflected = true;
            } else if (q1_val > M_PI + 1e-10) {
                q1_val -= M_PI;
                reflected = true;
            }
            q1_val = std::max(0.0, std::min(M_PI, q1_val));

            GcpcPoint pt{};
            pt.link_id = link_id;
            pt.n_eff = n_eff;
            pt.q_eff[0] = q1_val;
            for (int d = 1; d < n_eff; ++d)
                pt.q_eff[d] = q_full[d + 1];
            pt.direction = direction;

            Eigen::VectorXd q_eval(n);
            q_eval.setZero();
            q_eval[1] = q1_val;
            for (int d = 1; d < n_eff && d + 1 < n; ++d)
                q_eval[d + 1] = q_full[d + 1];

            auto pos = fk_link_positions(robot, q_eval);
            if (link_id + 1 < static_cast<int>(pos.size())) {
                const auto& p = pos[link_id + 1];
                pt.A = p[0]; pt.B = p[1]; pt.C = p[2];
                pt.R = std::sqrt(pt.A*pt.A + pt.B*pt.B);

                uint64_t h = q_eff_hash(pt);
                if (existing_hashes.find(h) == existing_hashes.end()) {
                    existing_hashes.insert(h);
                    new_points.push_back(pt);
                }
            }
        };

        // ── Generate seeds ──────────────────────────────────────────────

        // Seed type 1: kπ/2 grid seeds
        {
            std::vector<std::vector<double>> eff_kpi2(n_eff);
            for (double v : kpi2_vals[1]) {
                if (v >= -1e-10 && v <= M_PI + 1e-10)
                    eff_kpi2[0].push_back(std::max(0.0, std::min(M_PI, v)));
            }
            if (eff_kpi2[0].empty()) eff_kpi2[0].push_back(HALF_PI);
            for (int d = 1; d < n_eff; ++d)
                eff_kpi2[d] = kpi2_vals[d + 1];

            long long total = 1;
            for (int d = 0; d < n_eff; ++d) {
                total *= static_cast<long long>(eff_kpi2[d].size());
                if (total > 5000) break;
            }

            if (total <= 5000) {
                std::vector<double> config(n_eff);
                std::function<void(int)> gen_seeds;
                gen_seeds = [&](int d) {
                    if (d >= n_eff) {
                        Eigen::VectorXd q_seed(n);
                        q_seed.setZero();
                        for (int i = 0; i < n_eff && i + 1 < n; ++i)
                            q_seed[i + 1] = config[i];

                        for (int dir = 0; dir < 2; ++dir) {
                            Eigen::VectorXd q_cd = q_seed;
                            coordinate_descent(q_cd, dir);
                        }
                        return;
                    }
                    for (double v : eff_kpi2[d]) {
                        config[d] = v;
                        gen_seeds(d + 1);
                    }
                };
                gen_seeds(0);
            }
        }

        // Seed type 2: Random seeds
        for (int s = 0; s < n_random_seeds; ++s) {
            Eigen::VectorXd q_seed(n);
            q_seed.setZero();
            q_seed[1] = std::uniform_real_distribution<double>(0, M_PI)(rng);
            for (int d = 1; d < n_eff; ++d) {
                int j = d + 1;
                q_seed[j] = std::uniform_real_distribution<double>(
                    limits.limits[j].lo, limits.limits[j].hi)(rng);
            }

            for (int dir = 0; dir < 2; ++dir) {
                Eigen::VectorXd q_cd = q_seed;
                coordinate_descent(q_cd, dir);
            }
        }

        // Seed type 3: Pair-constrained seeds (qi+qj 鈮?kπ/2)
        for (auto& [pa, pb] : all_pairs) {
            if (pa == 0 || pb == 0) continue;
            if (pa > n_eff || pb > n_eff) continue;

            for (int k = -10; k <= 10; ++k) {
                double C = k * HALF_PI;
                double qa_try = std::max(limits.limits[pa].lo,
                                 std::min(limits.limits[pa].hi, C * 0.5));
                double qb_try = std::max(limits.limits[pb].lo,
                                 std::min(limits.limits[pb].hi, C - qa_try));

                Eigen::VectorXd q_seed(n);
                q_seed.setZero();
                q_seed[1] = HALF_PI;
                for (int d = 1; d < n_eff; ++d) {
                    int j = d + 1;
                    q_seed[j] = 0.5 * (limits.limits[j].lo + limits.limits[j].hi);
                }
                q_seed[pa] = qa_try;
                q_seed[pb] = qb_try;

                for (int dir = 0; dir < 2; ++dir) {
                    Eigen::VectorXd q_cd = q_seed;
                    coordinate_descent(q_cd, dir);
                }
            }
        }

        // ── Add new points to section ───────────────────────────────────
        if (!new_points.empty()) {
            for (auto& pt : new_points)
                section.points.push_back(pt);
            section.n_points = static_cast<int>(section.points.size());
            build_kdtree(section);
            total_added += static_cast<int>(new_points.size());
        }
    }

    // H4: precompute Phase E/F kπ/2-bg critical configs
    precompute_pair_kpi2(robot);

    return total_added;
}


// ═══════════════════════════════════════════════════════════════════════════
//  JSON loading –uses nlohmann/json
// ═══════════════════════════════════════════════════════════════════════════

bool GcpcCache::load_json(const std::string& path, const Robot& robot) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        std::cerr << "GCPC: Cannot open " << path << std::endl;
        return false;
    }

    nlohmann::json doc;
    try {
        ifs >> doc;
    } catch (const std::exception& e) {
        std::cerr << "GCPC: JSON parse error: " << e.what() << std::endl;
        return false;
    }

    robot_name_ = robot.name();
    d0_ = robot.dh_params()[0].d;
    has_tool_ = robot.has_tool();

    std::vector<GcpcPoint> points;

    nlohmann::json arr;
    if (doc.contains("points") && doc["points"].is_array()) {
        arr = doc["points"];
    } else if (doc.is_array()) {
        arr = doc;
    } else {
        std::cerr << "GCPC: Expected 'points' array in JSON" << std::endl;
        return false;
    }

    for (const auto& obj : arr) {
        GcpcPoint pt{};

        if (obj.contains("q_eff") && obj["q_eff"].is_array()) {
            const auto& qa = obj["q_eff"];
            pt.n_eff = static_cast<int>(qa.size());
            for (int i = 0; i < pt.n_eff && i < 7; ++i)
                pt.q_eff[i] = qa[i].get<double>();
        }

        if (obj.contains("link_id"))
            pt.link_id = obj["link_id"].get<int>();

        if (obj.contains("direction")) {
            if (obj["direction"].is_string()) {
                pt.direction = (obj["direction"].get<std::string>() == "xy") ? 0 : 1;
            } else {
                pt.direction = obj["direction"].get<int>();
            }
        }

        {
            Eigen::VectorXd q_full(robot.n_joints());
            for (int j = 0; j < robot.n_joints(); ++j)
                q_full[j] = 0.0;

            for (int d = 0; d < pt.n_eff && d + 1 < robot.n_joints(); ++d)
                q_full[d + 1] = pt.q_eff[d];

            auto positions = fk_link_positions(robot, q_full);
            int np = static_cast<int>(positions.size());
            if (pt.link_id + 1 < np) {
                const auto& p = positions[pt.link_id + 1];
                pt.A = p[0];
                pt.B = p[1];
                pt.C = p[2] - d0_;
                pt.R = std::sqrt(pt.A * pt.A + pt.B * pt.B);
            }
        }

        points.push_back(pt);
    }

    std::cout << "GCPC: Loaded " << points.size()
              << " critical points from " << path << std::endl;

    build(robot, points);
    return true;
}


// ═══════════════════════════════════════════════════════════════════════════
//  Binary save/load
// ═══════════════════════════════════════════════════════════════════════════

bool GcpcCache::save(const std::string& path) const {
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs.is_open()) return false;

    char header[512] = {};
    std::memcpy(header, "GCPC", 4);
    uint32_t version = 2;
    std::memcpy(header + 4, &version, 4);

    uint32_t name_len = static_cast<uint32_t>(robot_name_.size());
    std::memcpy(header + 8, &name_len, 4);
    std::memcpy(header + 12, robot_name_.c_str(), std::min<size_t>(name_len, 63));

    std::memcpy(header + 76, &d0_, 8);
    uint8_t ht = has_tool_ ? 1 : 0;
    std::memcpy(header + 84, &ht, 1);

    uint32_t n_sec = static_cast<uint32_t>(sections_.size());
    std::memcpy(header + 85, &n_sec, 4);

    uint64_t offset = 512;
    size_t table_pos = 89;
    for (const auto& sec : sections_) {
        int32_t lid = sec.link_id;
        int32_t nej = sec.n_eff_joints;
        int32_t np = sec.n_points;
        int32_t nkd = static_cast<int32_t>(sec.kd_tree.size());
        uint8_t q6s = sec.q6_skipped ? 1 : 0;

        std::memcpy(header + table_pos, &lid, 4); table_pos += 4;
        std::memcpy(header + table_pos, &nej, 4); table_pos += 4;
        std::memcpy(header + table_pos, &np, 4);  table_pos += 4;
        std::memcpy(header + table_pos, &nkd, 4); table_pos += 4;
        std::memcpy(header + table_pos, &q6s, 1); table_pos += 1;
        std::memcpy(header + table_pos, &offset, 8); table_pos += 8;

        offset += sec.kd_tree.size() * sizeof(KdNode);
        offset += sec.n_points * sizeof(GcpcPoint);
    }

    ofs.write(header, 512);

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

    if (std::memcmp(header, "GCPC", 4) != 0) return false;

    uint32_t version;
    std::memcpy(&version, header + 4, 4);
    if (version != 2) return false;

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
        int32_t lid, nej, np, nkd;
        uint8_t q6s;
        uint64_t sec_offset;

        std::memcpy(&lid, header + table_pos, 4); table_pos += 4;
        std::memcpy(&nej, header + table_pos, 4); table_pos += 4;
        std::memcpy(&np,  header + table_pos, 4); table_pos += 4;
        std::memcpy(&nkd, header + table_pos, 4); table_pos += 4;
        std::memcpy(&q6s, header + table_pos, 1); table_pos += 1;
        std::memcpy(&sec_offset, header + table_pos, 8); table_pos += 8;

        sec.link_id = lid;
        sec.n_eff_joints = nej;
        sec.n_points = np;
        sec.q6_skipped = (q6s != 0);

        ifs.seekg(sec_offset);
        sec.kd_tree.resize(nkd);
        ifs.read(reinterpret_cast<char*>(sec.kd_tree.data()),
                nkd * sizeof(KdNode));

        sec.points.resize(np);
        ifs.read(reinterpret_cast<char*>(sec.points.data()),
                np * sizeof(GcpcPoint));

        sec.kd_root = (nkd > 0) ? 0 : -1;
    }

    return ifs.good();
}

} // namespace envelope
} // namespace sbf
