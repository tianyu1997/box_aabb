// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Monte Carlo link iAABB reference baseline
//  Module: sbf::envelope
//
//  Provides a brute-force MC sampling reference for evaluating how tight
//  (or under-estimated) other endpoint sources are.
//
//  NOT intended for production use — only for experiments / gap analysis.
//
//  API:
//    compute_mc_link_iaabb(robot, intervals, n_mc, seed, out_iaabb)
//      Draws n_mc uniform samples from intervals, evaluates FK for each,
//      and unions per-link endpoint AABBs (+ link radius) into out_iaabb.
//      out_iaabb layout: [n_active × 6] = {lo_x,lo_y,lo_z,hi_x,hi_y,hi_z}
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/robot/robot.h"
#include "sbf/core/types.h"

#include <cstdint>
#include <vector>

namespace sbf {
namespace envelope {

/// Monte Carlo link iAABB reference baseline.
///
/// Draws @p n_mc uniform random configurations from @p intervals and
/// accumulates per-link sphere-swept AABBs (link radius included) into
/// @p out_iaabb.
///
/// @param robot       Robot definition (DH params, radii, active links).
/// @param intervals   Joint intervals to sample from (size = n_joints).
/// @param n_mc        Number of Monte Carlo samples (recommended: 50000+).
/// @param seed        RNG seed for reproducibility.
/// @param out_iaabb   Output buffer [n_active × 6], caller-allocated.
///                    Layout: {lo_x, lo_y, lo_z, hi_x, hi_y, hi_z} per link.
void compute_mc_link_iaabb(
    const Robot&                 robot,
    const std::vector<Interval>& intervals,
    int                          n_mc,
    uint32_t                     seed,
    float*                       out_iaabb);

} // namespace envelope
} // namespace sbf
