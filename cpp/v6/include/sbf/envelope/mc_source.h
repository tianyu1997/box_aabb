#pragma once
/// @file mc_source.h
/// @brief Pure Monte Carlo endpoint source (unsafe).
///
/// Samples joint vectors uniformly inside the input interval box and
/// computes endpoint iAABBs as the hull of sampled FK positions.
///
/// Result `is_safe = false` — random sampling may miss true extrema.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/envelope/endpoint_source.h>

#include <cstdint>
#include <vector>

namespace sbf {

EndpointIAABBResult compute_endpoint_iaabb_mc(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_samples = 1000,
    uint64_t seed = 42,
    int changed_dim = -1);

}  // namespace sbf
