#pragma once
/// @file ffb.h
/// @brief Find-Free-Box (FFB): the core box-construction algorithm.
///
/// Given a seed configuration `q` in C-space, FFB descends through the
/// LECT tree to find a leaf node whose interval box contains `q` and
/// is collision-free against the obstacle set.  Along the way it may
/// expand (split) existing leaves and compute link envelopes.
///
/// The returned `FFBResult` carries the LECT leaf index, the root→leaf
/// node path, detailed timing instrumentation, and a `fail_code`
/// indicating why the search stopped (success, occupied, depth/edge
/// limits, or deadline).
///
/// @see LECT, ForestGrower

#include <sbf/core/types.h>
#include <sbf/lect/lect.h>

#include <Eigen/Dense>

#include <vector>

namespace sbf {

// ─── FFB result ─────────────────────────────────────────────────────────────

/// @brief Outcome of a single FFB invocation.
///
/// Contains the target LECT leaf, root→leaf path, timing breakdown,
/// and cache-hit statistics for performance analysis.
struct FFBResult {
    int node_idx = -1;          ///< Target LECT leaf index (−1 on failure).
    std::vector<int> path;      ///< Root → leaf node path through the LECT.
    int fail_code = 0;          ///< 0=success, 1=occupied, 2=max_depth, 3=unused, 4=deadline.
    int n_new_nodes = 0;        ///< Nodes expanded (split) during this search.
    int n_fk_calls = 0;         ///< Forward-kinematics computations performed.

    // ── Timing instrumentation ──
    int n_cache_hits = 0;       ///< Envelope data already cached in LECT node.
    int n_cache_misses = 0;     ///< Envelope computed fresh.
    double total_ms = 0.0;      ///< Total FFB call wall-clock time.
    double envelope_ms = 0.0;   ///< Time spent in compute_envelope only.
    double collide_ms = 0.0;    ///< Time spent in collision checks.
    double expand_ms = 0.0;     ///< Time spent in expand_leaf.
    double intervals_ms = 0.0;  ///< Time spent in node_intervals.
    int n_collide_calls = 0;    ///< Number of collides_scene invocations.
    int n_expand_calls = 0;     ///< Number of expand_leaf invocations.
    int n_steps = 0;            ///< Total descent-loop iterations.

    bool success() const { return fail_code == 0 && node_idx >= 0; }
};

// ─── FFB configuration ─────────────────────────────────────────────────────

/// @brief Configuration for one FFB invocation.
struct FFBConfig {
    int max_depth = 30;         ///< Maximum LECT tree depth.
    double deadline_ms = 0.0;   ///< Absolute timeout (0 = unlimited).
};

// ─── FFB main entry point ───────────────────────────────────────────────────
/// @brief Find or create a collision-free LECT leaf containing `seed`.
///
/// Descends the LECT from root to leaf, splitting nodes as needed,
/// computing link envelopes, and testing against the obstacle scene.
///
/// @param lect    LECT tree (will be expanded in place).
/// @param seed    Target configuration in C-space.
/// @param obs     Obstacle array.
/// @param n_obs   Number of obstacles.
/// @param config  FFB parameters (max_depth, deadline).
/// @return  FFBResult with leaf index, path, and timing.
FFBResult find_free_box(
    LECT& lect,
    const Eigen::VectorXd& seed,
    const Obstacle* obs,
    int n_obs,
    const FFBConfig& config = FFBConfig()
);

}  // namespace sbf
