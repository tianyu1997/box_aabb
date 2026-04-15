#pragma once
/// @file crit_source.h
/// @brief Critical-point boundary enumeration endpoint source (unsafe).
///
/// Two stages:
///   1. Critical point enumeration: {lo, hi, kπ/2} boundary combos → FK → AABB.
///   2. (Future) Local optimization at discovered extrema.
///
/// Result `is_safe = false` — may miss true extrema beyond sampled points.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/envelope/endpoint_source.h>

#include <cstdint>
#include <vector>

namespace sbf {

// Compute endpoint iAABBs via critical-point boundary enumeration.
// Two stages:
//   1. Critical point enumeration: {lo, hi, k*pi/2 within range} combos → FK → AABB
//      Narrow intervals (< 0.01 rad) collapse to midpoint only.
//   2. (Optional future) Local optimization
// Result is_safe = false (UNSAFE — may miss extrema).
EndpointIAABBResult compute_endpoint_iaabb_crit(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_samples = 1000,
    uint64_t seed = 42,
    int changed_dim = -1);

}  // namespace sbf
