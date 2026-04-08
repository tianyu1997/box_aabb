#pragma once
/// @file analytical_source.h
/// @brief Multi-phase analytical critical-point endpoint source (safe).
///
/// Four phases (controlled by `max_phase`):
///   - Phase 0: kπ/2 vertex enumeration (boundary extrema).
///   - Phase 1: 1-D edge solve via atan2 closed-form per joint.
///   - Phase 2: 2-D face solve via atan2 for joint pairs.
///   - Phase 3: Interior coordinate descent (multi-start).
///
/// Result `is_safe = true` — conservative bounds guaranteed.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/envelope/endpoint_source.h>

#include <vector>

namespace sbf {

// Compute endpoint iAABBs via multi-phase analytical critical-point solve.
//
// Phases:
//   0  kπ/2 vertex enumeration (boundary extrema from grid combos)
//   1  1D edge solve: atan2 closed-form per joint (with background combos)
//   2  2D face solve: atan2 for joint-pairs (with background combos)
//   3  Interior coordinate descent (multi-start)
//
// max_phase controls how many phases to run (0..3).
// Result is_safe = true (SAFE — conservative bounds).
EndpointIAABBResult compute_endpoint_iaabb_analytical(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int max_phase = 3);

}  // namespace sbf
