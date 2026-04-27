#pragma once
/// @file endpoint_iaabb.h
/// @brief Public endpoint-AABB API (zero-radius, D1).

#include "sbf/core/fk_state.h"

#include <cstdint>
#include <string>
#include <vector>

namespace sbf::core {

enum class EndpointSourceKind : uint8_t {
    IFK        = 0,
    CritSample = 1,
};

struct EndpointSourceConfig {
    EndpointSourceKind kind = EndpointSourceKind::IFK;
};

const char* endpoint_source_name(EndpointSourceKind kind);
EndpointSourceKind parse_endpoint_source_kind(const std::string& name);

/// Result of one endpoint-AABB evaluation. All AABBs are zero-radius.
struct EndpointIAABBResult {
    /// Flat float buffer of shape [n_active_links × 2 × 6].
    /// Per active link i: proximal[lo3, hi3] then distal[lo3, hi3].
    std::vector<float> endpoint_iaabbs;
    int                n_active_links = 0;
    FKState            fk_state;        // for downstream incremental updates
    bool               is_safe = true;
};

/// Compute zero-radius endpoint iAABBs via interval FK.
/// `fk_cache` may be nullptr (full recompute) or a parent state for
/// incremental update (then `changed_dim >= 0`).
EndpointIAABBResult compute_endpoint_iaabb_ifk(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    FKState* fk_cache = nullptr,
    int changed_dim = -1);

/// Compute zero-radius endpoint AABBs from the selected source. IFK is the
/// certified default. CritSample is an advisory sampling source used by
/// profiling/build experiments and does not produce a certificate.
EndpointIAABBResult compute_endpoint_iaabb(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    EndpointSourceConfig source = {},
    FKState* fk_cache = nullptr,
    int changed_dim = -1);

}  // namespace sbf::core
