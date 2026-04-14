#pragma once
/// @file ifk_source.h
/// @brief Interval FK (IFK) endpoint iAABB source — provably conservative.
///
/// Computes endpoint iAABBs by propagating interval DH matrices through
/// the kinematic chain.  Supports incremental mode when a parent FK
/// state and the changed dimension are provided.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>
#include <sbf/envelope/endpoint_source.h>

#include <vector>

namespace sbf {

// Compute endpoint iAABBs via Interval FK.
// If fk != nullptr and fk->valid and changed_dim >= 0, uses incremental FK.
EndpointIAABBResult compute_endpoint_iaabb_ifk(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    FKState* fk = nullptr,
    int changed_dim = -1);

}  // namespace sbf
