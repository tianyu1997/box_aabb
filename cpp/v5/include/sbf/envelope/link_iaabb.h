#pragma once
/// @file link_iaabb.h
/// @brief Link IAABB derivation from endpoint iAABBs (Phase C1).
///
/// Converts proximal+distal endpoint iAABBs into per-link bounding boxes.
/// Two modes:
///   - **Paired**: hull of proximal and distal endpoints + radius inflation.
///   - **Subdivided**: each link split into n_sub segments via linear
///     interpolation between proximal and distal intervals.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>

#include <vector>

namespace sbf {

// ─── Paired derive: proximal+distal → link AABB ────────────────────────────
// endpoint_iaabbs layout: [n_active × 2 × 6]
//   (ci*2+0)*6 = proximal iAABB, (ci*2+1)*6 = distal iAABB
// out_link_iaabbs layout: [n_active × 6]: [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
void derive_link_iaabb_paired(
    const float* endpoint_iaabbs,
    int n_active_links,
    const double* link_radii,
    float* out_link_iaabbs);

// ─── Subdivided derive: each link split into n_sub segments ────────────────
// Linear interpolation between proximal and distal endpoint intervals.
// out_sub_iaabbs layout: [n_active × n_sub × 6]
void derive_link_iaabb_subdivided(
    const float* endpoint_iaabbs,
    int n_active_links,
    const double* link_radii,
    int n_subdivisions,
    float* out_sub_iaabbs);

}  // namespace sbf
