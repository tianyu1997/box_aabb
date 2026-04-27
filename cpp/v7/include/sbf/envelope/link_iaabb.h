#pragma once
/// @file link_iaabb.h
/// @brief Zero-radius link AABB derivation from endpoint iAABBs.
///
/// Per architectural decision D1 (Radii=0 cache), these functions derive
/// **zero-radius** link AABBs only. Per-link radius inflation is applied
/// at scene-collision time via
/// `sbf::scene::aabbs_collide_obs_inflated()`.

namespace sbf::envelope {

/// Paired derive: hull(proximal, distal) per active link.
///   in:  endpoint_iaabbs[n_active * 2 * 6]
///   out: out_link_iaabbs[n_active * 6]   layout [lo_x,lo_y,lo_z,hi_x,hi_y,hi_z]
void derive_link_iaabb_paired_zero(
    const float* endpoint_iaabbs,
    int n_active_links,
    float* out_link_iaabbs);

/// Subdivided derive: each link split into n_sub segments via t-lerp.
///   out: out_sub_iaabbs[n_active * n_sub * 6]
void derive_link_iaabb_subdivided_zero(
    const float* endpoint_iaabbs,
    int n_active_links,
    int n_subdivisions,
    float* out_sub_iaabbs);

}  // namespace sbf::envelope
