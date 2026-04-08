// SafeBoxForest v3 — Critical-Point Envelope Derive
// Module: sbf::envelope
//
// Pure functions that derive AABB / OBB envelopes by enumerating critical
// joint configurations (interval endpoints + sin/cos zeros) and evaluating
// scalar FK.  These produce tighter envelopes than interval-FK at the cost
// of many FK evaluations, and are intended as pre-computed sidecar data
// stored alongside the FrameStore.
//
// Architecture (v3 unified pipeline):
//   derive_crit_endpoints()              → endpoints → endpoint_aabb (in pipeline)
//   derive_aabb_critical_analytical()   → out_endpoint_aabb (optional output)
//   Both feed into the unified compute_envelope() function.
//   See envelope_derive.h for the pipeline overview.
//
// The critical-point set for a link depending on joints [0..V] is the
// Cartesian product of per-joint candidate angles:
//   { q_lo, q_hi } ∪ { kπ/2 | k ∈ Z, q_lo < kπ/2 < q_hi }
// capped at CRIT_MAX_COMBOS (falls back to {lo, mid, hi} per joint).
//
// Subdivision (n_sub > 1) splits each link's parametric segment [0,1]
// into equal parts and computes independent envelopes per sub-segment.
//
#pragma once

#include "sbf/robot/robot.h"
#include "sbf/common/types.h"
#include <Eigen/Core>
#include <vector>
#include <utility>
#include <tuple>

namespace sbf {
namespace envelope {

// OBB slot layout: 15 floats = [cx cy cz | hx hy hz | Ax(3) | Ay(3) | Az(3)]
// Centre is stored in WORLD coordinates (not projected) for correct SAT.
static constexpr int CRIT_OBB_FLOATS = 15;

// ═════════════════════════════════════════════════════════════════════════════
//  Enhanced critical sampling configuration
// ═════════════════════════════════════════════════════════════════════════════
//
// Three-stage pipeline (ported from Python v4):
//   Stage 1: Critical-point enumeration (base + coupled constraints)
//   Stage 2: Constraint manifold random sampling
//   Stage 3: L-BFGS-B local optimization
//
struct CriticalSamplingConfig {
    long long max_combos = 60'000;

    // Stage 1 enhancement: coupled joint constraint enumeration
    //   Adds strategies 3-6 from v4:
    //     3: Two-joint sum constraint  q_i + q_j = k*pi/2
    //     4: Specific coupled pairs    (from robot config)
    //     5-6: Coupled triples         (from robot config)
    bool enable_coupled_constraints = true;

    // Stage 2: Constraint manifold random sampling
    //   Random sampling on q_i + q_j = k*pi/2 manifolds
    //   and q_a + q_b + q_c = k*pi/2 manifolds
    bool enable_manifold_sampling = true;
    int  manifold_n_per = 7;   // samples per valid constraint value

    // Stage 3: L-BFGS-B local optimization
    //   For each AABB face direction (x_min, x_max, ...),
    //   run box-constrained optimization from best seeds
    bool enable_lbfgs_optimization = true;
    int  lbfgs_max_iter = 30;
    double lbfgs_ftol = 1e-8;
    int  lbfgs_n_seeds = 2;    // exploit top-1 + explore furthest

    // ── Acceleration options ─────────────────────────────────────────
    // Analytical FK Jacobian replaces finite-difference gradient.
    // Reduces gradient cost from 2*n_joints FK calls to n cross-products.
    bool use_analytical_jacobian = true;
    // Restrict optimisation to joints [0..V] per link V (implied by analytical).
    // Also reduces L-BFGS memory and per-iteration cost.
    bool restrict_joint_scope = true;
    // Use tracked configs from Stage 1/2 extremes as seeds directly,
    // instead of pre-screening all constraint seeds per face (saves ~84k FK).
    bool smart_seed_selection = true;
    // Skip L-BFGS for directions where gap < threshold (relative to interval width)
    double skip_gap_threshold = 0.0;  // 0 = never skip

    // Default: all enhancements enabled
    static CriticalSamplingConfig all_enabled() {
        return CriticalSamplingConfig{};
    }
    // Baseline: no enhancements (equivalent to v2 original)
    static CriticalSamplingConfig baseline() {
        CriticalSamplingConfig c;
        c.enable_coupled_constraints = false;
        c.enable_manifold_sampling   = false;
        c.enable_lbfgs_optimization  = false;
        return c;
    }
    // All stages enabled, but no acceleration (for timing comparison)
    static CriticalSamplingConfig full_slow() {
        CriticalSamplingConfig c;
        c.use_analytical_jacobian = false;
        c.restrict_joint_scope    = false;
        c.smart_seed_selection    = false;
        return c;
    }
    // All stages enabled, all accelerations (recommended)
    static CriticalSamplingConfig full_fast() {
        CriticalSamplingConfig c;
        c.lbfgs_max_iter = 15;
        c.lbfgs_n_seeds  = 1;
        return c;
    }
};

// ═════════════════════════════════════════════════════════════════════════════
//  Critical-point frames → FrameStore format (subdivision-agnostic)
// ═════════════════════════════════════════════════════════════════════════════
//
// Enumerates critical configurations and evaluates scalar FK to compute
// per-endpoint position intervals [x_lo, y_lo, z_lo, x_hi, y_hi, z_hi].
//
// Output is directly compatible with FrameStore::store_frames(), allowing
// all existing derive functions (derive_aabb, derive_aabb_subdivided, etc.)
// to be reused at any subdivision level without re-running FK enumeration.
//
// NOTE: link_radii are NOT baked in — inflate at derive time, same as IFK.
//
// out_endpoints: [n_endpoints × 6] floats   (n_endpoints = n_joints + has_tool)
// out_n_combos: (optional) receives the number of critical configs evaluated
//
void derive_crit_endpoints(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    float* out_endpoints,
    int* out_n_combos = nullptr);

// ═════════════════════════════════════════════════════════════════════════════
//  AABB from critical-point enumeration
// ═════════════════════════════════════════════════════════════════════════════
//
// For each active link (compact index ci, link V = active_link_map[ci]):
//   - Enumerate critical configs for joints [0..V]
//   - Evaluate scalar FK → positions[V] (proximal), positions[V+1] (distal)
//   - With n_sub > 1, interpolate sub-segment endpoints
//   - Take axis-aligned bounding box, inflated by link_radii
//
// out_aabb: [n_active * n_sub * 6] floats [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
//
void derive_aabb_critical(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    float* out_aabb);

// ═════════════════════════════════════════════════════════════════════════════
//  OBB from critical-point enumeration + PCA
// ═════════════════════════════════════════════════════════════════════════════
//
// Same critical sampling as AABB, but fits PCA-based OBB to each
// sub-segment's point cloud.  Centre in world frame.
//
// out_obbs: [n_active * n_sub * CRIT_OBB_FLOATS] floats
//
void derive_obb_critical(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    float* out_obbs);

// ═════════════════════════════════════════════════════════════════════════════
//  Collision check: OBB slots vs interleaved obstacles
// ═════════════════════════════════════════════════════════════════════════════
//
// 15-axis SAT per (OBB slot, obstacle).
// obs_compact: [n_obs × 6] in interleaved [lo_x, hi_x, lo_y, hi_y, lo_z, hi_z]
//
bool collide_obb_obs(
    const float* obb_slots, int n_slots,
    const float* obs_compact, int n_obs);

// Volume of OBB slots: sum of 8 * hx * hy * hz
double volume_obb_slots(const float* obb_slots, int n_slots);

// ═════════════════════════════════════════════════════════════════════════════
//  Enhanced AABB from critical-point enumeration (3-stage pipeline)
// ═════════════════════════════════════════════════════════════════════════════
//
// Extends derive_aabb_critical with:
//   Stage 1+: Coupled constraint enumeration (strategies 3-6)
//   Stage 2:  Manifold random sampling
//   Stage 3:  L-BFGS-B local optimization per AABB face
//
// out_aabb: [n_active * n_sub * 6] floats
// out_stats: (optional) receives { n_stage1, n_stage2, n_stage3 } sample counts
//
struct CriticalStats {
    int n_stage1_base = 0;       // basic critical enumeration
    int n_stage1_coupled = 0;    // coupled constraint points
    int n_stage2_manifold = 0;   // manifold random samples
    int n_stage3_lbfgs = 0;      // L-BFGS-B optimization calls
};

void derive_aabb_critical_enhanced(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    const CriticalSamplingConfig& config,
    float* out_aabb,
    CriticalStats* out_stats = nullptr);

// ═════════════════════════════════════════════════════════════════════════════
//  Analytical critical-point enumeration (zero-optimisation)
// ═════════════════════════════════════════════════════════════════════════════
//
// Replaces L-BFGS-B optimisation with exact analytical gradient-zero point
// enumeration.  For each box face dimension k (0D → 1D → 2D → 3D+):
//
//   Phase 0 (Vertices):  All joints on boundaries {lo, hi}^n
//   Phase 1 (Edges):     1 free joint j, others on boundary.
//                         Solve ∂p_d/∂q_j = 0 via atan2  (2 candidates/edge)
//   Phase 2 (Faces):     2 free joints (i,j), others on boundary.
//                         Solve 2×2 trig system via half-angle → degree-8 poly
//                         Symbolic polynomial coefficients + interval-restricted
//                         sign-change bisection solver (all roots in [lo, hi])
//   Phase 3+ (Interior): k free joints (k ≥ 3), others on boundary.
//                         Iterative reduction: fix k−1 joints at partial
//                         analytical critical values, solve 1D for remaining.
//                         Repeated sweeps until convergence.
//
// No iterative optimisation, deterministic, and provably complete up to 2D.

struct AnalyticalCriticalConfig {
    bool keep_kpi2_baseline  = true;   // Phase 0: keep standard kπ/2 enumeration
    bool enable_edge_solve   = true;   // Phase 1: 1D atan2 on each edge
    bool enable_face_solve   = true;   // Phase 2: 2D polynomial on coupled pairs
    bool enable_interior_solve = true; // Phase 3+: iterative coordinate-wise sweep
    int  interior_max_free   = 7;      // max free-joint dimension for Phase 3+
    int  interior_max_sweeps = 3;      // convergence sweeps for Phase 3+
    bool face_all_pairs      = false;  // false: only coupled_pairs;  true: all C(n,2)

    // Phase 2.5: Pair-constrained solvers (NEW from Exp-24 analysis)
    //   On the manifold qi + qj = k*π/2, substitute qj = C - qi:
    //   - Phase 2.5a: all other joints on boundary → 1-DOF atan2 solve
    //   - Phase 2.5b: one additional free joint m  → 2-DOF polynomial solve
    //                 (symbolic degree-8 + interval-restricted bisection)
    bool enable_pair_1d       = true;   // Phase 2.5a
    bool enable_pair_2d       = true;   // Phase 2.5b
    int  pair_max_bg          = 64;     // max background combos per pair constraint
    bool pair_kpi2_backgrounds = true;  // also use kπ/2 values (not just lo/hi) in backgrounds
    bool pair_all_pairs       = false;  // false: only adjacent+coupled; true: all C(n,2)

    // Improved Phase 3: multi-start + pair-aware coordinate sweep
    bool improved_interior     = true;   // use enhanced Phase 3 (vs original sweep)
    int  interior_n_restarts   = 4;      // extra starting configs from pair-constrained results
    bool interior_pair_coupling = true;  // during sweep, couple paired joints

    // Fused dual-Phase-3 mode: run Phase 3 twice in a single analytical call.
    //   Run 1: from Phase 0+1+2+2.5 accumulated state (standard).
    //   Run 2: from Phase 0+1 checkpoint (no Phase 2 basin shift).
    // Eliminates redundant Phase 0/1 when merging v1_full + no_P2 passes.
    // See Exp-28 (GCPC validation): achieves zero-miss AABB at 43 ms avg.
    bool dual_phase3 = false;

    static AnalyticalCriticalConfig all_enabled() {
        return AnalyticalCriticalConfig{};
    }
    static AnalyticalCriticalConfig edges_only() {
        AnalyticalCriticalConfig c;
        c.enable_face_solve = false;
        c.enable_interior_solve = false;
        return c;
    }
    static AnalyticalCriticalConfig edges_and_faces() {
        AnalyticalCriticalConfig c;
        c.enable_interior_solve = false;
        c.enable_pair_1d = false;
        c.enable_pair_2d = false;
        return c;
    }
    // Original v1 analytical (no pair-constrained phases)
    static AnalyticalCriticalConfig v1_analytical() {
        AnalyticalCriticalConfig c;
        c.enable_pair_1d = false;
        c.enable_pair_2d = false;
        c.improved_interior = false;
        return c;
    }
};

struct AnalyticalCriticalStats {
    int n_phase0_vertices = 0;    // boundary vertex evaluations
    int n_phase1_edges = 0;       // 1D edge critical points found
    int n_phase2_faces = 0;       // 2D face critical points found
    int n_phase25a_pair1d = 0;    // Phase 2.5a pair-constrained 1D points
    int n_phase25b_pair2d = 0;    // Phase 2.5b pair-constrained 2D points
    int n_phase3_interior = 0;    // 3D+ interior critical points found
    int n_phase1_fk_calls = 0;    // FK calls during Phase 1
    int n_phase2_fk_calls = 0;    // FK calls during Phase 2
    int n_phase25_fk_calls = 0;   // FK calls during Phase 2.5
    int n_phase3_fk_calls = 0;    // FK calls during Phase 3+
};

/// out_endpoint_aabb: optional [n_active * (n_sub+1) * 6] floats
///   Per-endpoint interval AABB: the AABB of each link point at
///   t=s/n_sub, swept over all configs in the box (geometry-only, no radius).
///   Used for hull16 envelope: fill_hull16(ep[0], ep[n_sub], r).
void derive_aabb_critical_analytical(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    const AnalyticalCriticalConfig& config,
    float* out_aabb,
    AnalyticalCriticalStats* out_stats = nullptr,
    float* out_endpoint_aabb = nullptr);

// Same as above but also outputs the joint configurations achieving each extreme.
// out_configs: array of size [n_active_links * n_sub * 6] Eigen::VectorXd
//   Layout: out_configs[(ci * n_sub + s) * 6 + face]
//   face: 0=x_min, 1=x_max, 2=y_min, 3=y_max, 4=z_min, 5=z_max
// If out_configs is nullptr, behaves identically to the version above.
void derive_aabb_critical_analytical_with_configs(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_sub,
    const AnalyticalCriticalConfig& config,
    float* out_aabb,
    Eigen::VectorXd* out_configs,
    AnalyticalCriticalStats* out_stats = nullptr);

} // namespace envelope
} // namespace sbf
