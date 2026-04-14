#pragma once
/// @file crit_source.h
/// @brief Critical-point + random sampling endpoint source (unsafe).
///
/// Three stages:
///   1. Critical point enumeration: {lo, hi, kπ/2} boundary combos → FK → AABB.
///   2. Random sampling: adaptive sample count scaled by C-space volume fraction.
///   3. (Future) Local optimization at discovered extrema.
///
/// Result `is_safe = false` — may miss true extrema beyond sampled points.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/envelope/endpoint_source.h>

#include <cstdint>
#include <vector>

namespace sbf {

// Compute endpoint iAABBs via critical-point + random sampling.
// Three stages:
//   1. Critical point enumeration: {lo, hi, k*pi/2 within range} combos → FK → AABB
//      Narrow intervals (< 0.01 rad) collapse to midpoint only.
//   2. Random sampling: adaptive n_samples (scaled by C-space volume fraction).
//      When changed_dim >= 0, prefix FK for joints 0..changed_dim-1 is
//      computed once and reused across all samples.
//   3. (Optional future) Local optimization
// Result is_safe = false (UNSAFE — may miss extrema).
EndpointIAABBResult compute_endpoint_iaabb_crit(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_samples = 1000,
    uint64_t seed = 42,
    int changed_dim = -1);

}  // namespace sbf
