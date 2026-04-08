// SafeBoxForest v3 — Hybrid IA/AA AABB extraction
//
// Dispatcher that selects Interval Arithmetic (IA) or Affine Arithmetic (AA)
// for per-link AABB computation based on the maximum interval width.
//
// At narrow intervals (width ≤ crossover), AA is both tighter and ~6× faster.
// At wide intervals, IA is more robust (AA's Taylor remainder explodes).
//
#pragma once

#include "sbf/robot/interval_fk.h"
#include "sbf/robot/affine_fk.h"
#include "sbf/robot/robot.h"
#include "sbf/common/types.h"
#include <vector>

namespace sbf {

/// Compute the maximum interval width across all joints.
inline double max_interval_width(const std::vector<Interval>& intervals) {
    double w = 0.0;
    for (const auto& iv : intervals)
        w = std::max(w, iv.hi - iv.lo);
    return w;
}

/// Hybrid per-link AABB extraction.
///
/// If aa_crossover > 0 and max interval width ≤ aa_crossover, uses Affine
/// Arithmetic (aa_compute_link_aabbs) which is tighter and faster for narrow
/// boxes.  Otherwise falls back to standard Interval Arithmetic via the
/// pre-computed FKState (extract_link_aabbs).
///
/// @param fk               Pre-computed IA FKState (always available)
/// @param intervals        C-space intervals for this node
/// @param robot            Robot model
/// @param out_aabb         Output: n_active_links × 6 floats
/// @param aa_crossover     Crossover width threshold (0 = always IA)
/// @return true if AA was used, false if IA was used
inline bool extract_link_aabbs_hybrid(
    const FKState& fk,
    const std::vector<Interval>& intervals,
    const Robot& robot,
    float* out_aabb,
    double aa_crossover)
{
    if (aa_crossover > 0.0 && max_interval_width(intervals) <= aa_crossover) {
        // AA path: compute from scratch (fast at narrow intervals)
        aa_compute_link_aabbs(robot, intervals, out_aabb);
        return true;
    }

    // IA path: use pre-computed FKState
    extract_link_aabbs(fk,
                       robot.active_link_map(),
                       robot.n_active_links(),
                       out_aabb,
                       robot.active_link_radii());
    return false;
}

} // namespace sbf
