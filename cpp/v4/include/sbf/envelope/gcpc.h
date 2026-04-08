// ═══════════════════════════════════════════════════════════════════════════
// SafeBoxForest v4 — GCPC (Global Critical Point Cache)
// ═══════════════════════════════════════════════════════════════════════════
//
// Precomputed critical points of FK position for each active link, stored
// in a KD-tree for fast orthogonal range queries.
//
// Architecture (v4 unified pipeline):
//   GCPC is one of four endpoint-iAABB sources (IFK, CritSample, Analytical, GCPC).
//   derive_aabb_with_gcpc() produces out_endpoint_iaabb [n_active × (n_sub+1) × 6]
//   which feeds into the unified compute_endpoint_iaabb() function.
//   See endpoint_source.h for the pipeline overview.
//
// Symmetry reductions applied:
//   - q₀ fully eliminated (atan2 reconstruction at query time)
//   - q₁ period-π: only q₁ ∈ [0,π] stored; reflected at query time
//   - q₆ conditionally skipped (when d₆=0, a₆=0, no tool)
//
// derive_aabb_with_gcpc() pipeline ("cache + analytical boundary"):
//   Step 1 — Analytical boundary solver (Phases 0–2.5b, no interior)
//            Delegates all boundary-related solving to
//            derive_aabb_critical_analytical() with enable_interior_solve=false.
//   Step 2 — Phase A: Cache lookup (interior critical points from KD-tree)
//            Merges cached interior positions into the boundary AABB.
//            Interior values use cache reuse: no runtime enumeration.
//
// Cache enrichment:
//   enrich_with_interior_search() runs coordinate descent from many seeds
//   across the full joint range to find all interior critical points of R
//   and pz. These are added to the KD-tree so Phase A gives tight pre-bounds,
//   allowing aggressive AA pruning of boundary phases.
//
// Binary cache format: GCPC (see GcpcCache::save / load)
//
// Migrated from v3 gcpc_cache.h.  Uses nlohmann/json for JSON loading.
//
#pragma once

#include "sbf/robot/robot.h"
#include "sbf/core/types.h"
#include <Eigen/Core>
#include <vector>
#include <string>
#include <cstdint>
#include <cstring>
#include <memory>

namespace sbf {
namespace envelope {

// ═══════════════════════════════════════════════════════════════════════════
//  GcpcPoint — one cached interior critical point
// ═══════════════════════════════════════════════════════════════════════════
struct GcpcPoint {
    // Joint configuration: q₁..qₖ (q₁ ∈ [0,π], others ∈ [-π,π])
    // q₀ is NOT stored — analytically reconstructed at query time.
    double q_eff[7];       // max 7 joints; only first n_eff used
    int    n_eff;          // number of effective joints

    int    link_id;        // 0-based frame index (e.g., 2, 4, 6, 7)
    int    direction;      // 0 = xy (radial R²), 1 = z

    // Precomputed FK sub-chain values (in post-T₀ frame)
    double A;              // p_local_x (for xy: used in R = √(A²+B²))
    double B;              // p_local_y (for xy: used in R)
    double C;              // p_local_z (for z direction)
    double R;              // √(A²+B²) — radial distance (precomputed)
};

// ═══════════════════════════════════════════════════════════════════════════
//  KD-Tree node — for range queries over joint-space coordinates
// ═══════════════════════════════════════════════════════════════════════════
struct KdNode {
    int      split_dim;    // dimension to split on (round-robin)
    double   split_val;    // median value in split dimension
    int      left;         // left child index (-1 = leaf)
    int      right;        // right child index (-1 = leaf)
    int      point_idx;    // if leaf: index into points array; else -1
};

// ═══════════════════════════════════════════════════════════════════════════
//  Precomputed kπ/2-background critical config (Phase E/F)
// ═══════════════════════════════════════════════════════════════════════════
struct PairKpi2Config {
    double q[7];           // full joint configuration (up to 7 joints)
    int    n_joints;       // number of joints stored
};

// ═══════════════════════════════════════════════════════════════════════════
//  Per-link cache section
// ═══════════════════════════════════════════════════════════════════════════
struct GcpcLinkSection {
    int link_id;           // 0-based frame index
    int n_eff_joints;      // effective DOF for this link (after q₀ elimination)
    int n_points;          // number of critical points for this link
    bool q6_skipped;       // whether q₆ was skipped for this link

    std::vector<GcpcPoint> points;
    std::vector<KdNode>    kd_tree;
    int                    kd_root;  // root node index in kd_tree

    /// Precomputed Phase E/F critical configs where ALL bg joints are kπ/2.
    /// These are interval-independent → precomputed once, reused per query.
    std::vector<PairKpi2Config> pair_kpi2_configs;
};

// ═══════════════════════════════════════════════════════════════════════════
//  Query result — a critical point matched in an interval query
// ═══════════════════════════════════════════════════════════════════════════
struct GcpcQueryResult {
    const GcpcPoint* point;    // pointer to cached point
    double q0_optimal;         // reconstructed q₀
    bool   q1_reflected;       // whether q₁ was reflected (q₁ - π)
    double q1_actual;          // actual q₁ used (after reflection if any)
};

// ═══════════════════════════════════════════════════════════════════════════
//  Query statistics (incl. AA pruning telemetry)
// ═══════════════════════════════════════════════════════════════════════════
struct GcpcQueryStats {
    int n_cache_matches = 0;     // Phase A: points found in cache
    int n_q1_reflected = 0;      // points from q₁ reflection
    int n_q0_valid = 0;          // points with q₀ in range
    int n_boundary_kpi2 = 0;     // Phase B: kπ/2 boundary points
    int n_boundary_atan2 = 0;    // Phase C: 1D atan2 points
    int n_fk_calls = 0;          // total FK evaluations

    // AA pruning stats (Phase C face-level bound pruning)
    int n_aa_prune_checks = 0;   // faces checked via AA bound
    int n_aa_pruned = 0;         // faces pruned (skipped bg enumeration)
    int n_aa_not_pruned = 0;     // faces that survived pruning

    // Phase D: 2D face solve (companion-matrix polynomial roots)
    int n_phase_d_faces = 0;     // face critical points evaluated
    int n_phase_d_fk = 0;        // FK calls in Phase D
    int n_phase_d_pruned = 0;    // link×pair combos pruned by AA

    // Phase E: Pair-constrained 1D (qi+qj = kπ/2, atan2 solve)
    int n_phase_e_pair1d = 0;    // pair-1D critical points evaluated
    int n_phase_e_fk = 0;        // FK calls in Phase E
    int n_phase_e_pruned = 0;    // pruned by AA

    // Phase F: Pair-constrained 2D (qi+qj = kπ/2 + extra free joint)
    int n_phase_f_pair2d = 0;    // pair-2D critical points evaluated
    int n_phase_f_fk = 0;        // FK calls in Phase F
    int n_phase_f_pruned = 0;    // pruned by AA

    // Phase G: Interior coordinate descent (query-local, per-face)
    int n_phase_g_interior = 0;  // interior candidate evaluations
    int n_phase_g_fk = 0;        // FK calls in Phase G
    int n_phase_g_pruned = 0;    // links pruned by AA (skipped entirely)

    // Interior cache enrichment (precomputation stats)
    int n_interior_added = 0;    // points added by enrich_with_interior_search

    // Per-phase timing (ms) — set when derive_aabb_with_gcpc returns
    double phase_a_ms = 0;       // Cache lookup
    double analytical_boundary_ms = 0;  // Analytical boundary solver (Phases 0–2.5b)
    double phase_b_ms = 0;       // (legacy) Boundary kπ/2 — now in analytical
    double phase_c_ms = 0;       // (legacy) Boundary 1D atan2 — now in analytical
    double phase_d_ms = 0;       // (legacy) 2D face solve — now in analytical
    double phase_e_ms = 0;       // (legacy) Pair 1D — now in analytical
    double phase_f_ms = 0;       // (legacy) Pair 2D — now in analytical
    double phase_g_ms = 0;       // (legacy) Interior CD — removed (cache only)
};

// ═══════════════════════════════════════════════════════════════════════════
//  GcpcCache — Main cache class
// ═══════════════════════════════════════════════════════════════════════════
class GcpcCache {
public:
    GcpcCache() = default;

    // ── Construction ─────────────────────────────────────────────────────

    /// Load from JSON file (generated by precompute_gcpc.jl).
    /// Uses nlohmann/json. Builds KD-trees for each link section.
    bool load_json(const std::string& path, const Robot& robot);

    /// Load from binary GCPC cache file.
    bool load(const std::string& path);

    /// Save to binary GCPC cache file.
    bool save(const std::string& path) const;

    /// Build cache from a Robot + set of critical points.
    /// Points should be in the q₁∈[0,π] half-range.
    void build(const Robot& robot,
               const std::vector<GcpcPoint>& points);

    /// Enrich cache with interior critical points found by coordinate descent.
    /// Runs multi-start optimization over the robot's full joint range to find
    /// all stationary points of R(q₁,...,qₖ) and pz(q₁,...,qₖ).
    /// Adds discovered points to the KD-tree so Phase A gives tight pre-bounds.
    /// @param n_random_seeds number of random seeds per link (default: 500)
    /// @param max_sweeps     coordinate descent iterations (default: 5)
    /// @return number of new unique interior critical points added
    int enrich_with_interior_search(
        const Robot& robot,
        int n_random_seeds = 500,
        int max_sweeps = 5);

    /// Precompute Phase E/F critical configs with kπ/2-only backgrounds.
    /// Stores results in each link section's pair_kpi2_configs vector.
    /// Called from enrich_with_interior_search or explicitly after build.
    void precompute_pair_kpi2(const Robot& robot);

    // ── Query ────────────────────────────────────────────────────────────

    /// Find all critical points for the given link within the joint interval.
    void query_link(int link_id,
                    const Interval* intervals,
                    std::vector<GcpcQueryResult>& results) const;

    /// Evaluate cached interior + analytical boundary critical points and compute tight AABB.
    ///   Step 1:  Analytical boundary solver (Phases 0–2.5b) — calls derive_aabb_critical_analytical()
    ///   Step 2:  Phase A: Cache lookup (KD-tree range query + q₀/q₁ reconstruction)
    /// out_aabb: [n_active * n_sub * 6] floats (radius-inflated)
    /// out_endpoint_iaabb: optional [n_active * (n_sub+1) * 6] floats (raw, no radius)
    void derive_aabb_with_gcpc(
        const Robot& robot,
        const std::vector<Interval>& intervals,
        int n_sub,
        float* out_aabb,
        GcpcQueryStats* out_stats = nullptr,
        float* out_endpoint_iaabb = nullptr) const;

    // ── Accessors ────────────────────────────────────────────────────────

    bool is_loaded() const { return !sections_.empty(); }
    int  n_links()   const { return static_cast<int>(sections_.size()); }
    int  n_total_points() const;

    const GcpcLinkSection* find_section(int link_id) const;

    /// Robot's base d₀ (needed for z reconstruction)
    double d0() const { return d0_; }

private:
    // ── KD-tree construction ─────────────────────────────────────────────
    void build_kdtree(GcpcLinkSection& section);

    int  build_kdtree_recursive(
        GcpcLinkSection& section,
        std::vector<int>& indices,
        int lo, int hi, int depth);

    // ── KD-tree range query ──────────────────────────────────────────────
    void kdtree_range_query(
        const GcpcLinkSection& section,
        int node_idx,
        const double* lo, const double* hi,
        int depth,
        std::vector<int>& matched_indices) const;

    // ── q₀ reconstruction ────────────────────────────────────────────────
    static double reconstruct_q0_xy(double A, double B);
    static bool   q0_in_range(double q0, double lo0, double hi0);

    // ── Data ─────────────────────────────────────────────────────────────
    std::string robot_name_;
    double d0_ = 0.0;
    bool   has_tool_ = false;
    std::vector<GcpcLinkSection> sections_;
};

} // namespace envelope
} // namespace sbf
