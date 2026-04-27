// SafeBoxForest v7 — Unified link envelope (D1: zero-radius AABB cache).
#include "sbf/envelope/envelope_type.h"
#include "sbf/envelope/link_iaabb.h"

#include <algorithm>
#include <memory>

namespace sbf::envelope {

LinkEnvelope compute_link_envelope(
    const float* endpoint_iaabbs,
    int n_active_links,
    const double* link_radii,
    const EnvelopeTypeConfig& config)
{
    LinkEnvelope result;
    result.type           = config.type;
    result.n_active_links = n_active_links;

    const int n_sub = std::max(1, config.n_subdivisions);
    result.n_subdivisions = n_sub;

    if (n_active_links <= 0 || endpoint_iaabbs == nullptr) {
        return result;
    }

    // Always derive zero-radius link AABBs (D1).
    if (n_sub <= 1) {
        result.link_iaabbs.resize(n_active_links * 6);
        derive_link_iaabb_paired_zero(endpoint_iaabbs, n_active_links,
                                      result.link_iaabbs.data());
    } else {
        result.link_iaabbs.resize(n_active_links * n_sub * 6);
        derive_link_iaabb_subdivided_zero(endpoint_iaabbs, n_active_links,
                                          n_sub, result.link_iaabbs.data());
    }

    // LinkIAABB_Grid: rasterise sub-AABBs inflated by (link_radius + safety_pad).
    if (config.type == EnvelopeType::LinkIAABB_Grid) {
        const double delta = config.grid_config.voxel_delta;
        auto sg = std::make_unique<sbf::voxel::SparseVoxelGrid>(delta);

        const int   n_boxes = n_active_links * n_sub;
        const float pad     = static_cast<float>(sg->safety_pad());
        for (int i = 0; i < n_boxes; ++i) {
            const float* src = result.link_iaabbs.data() + i * 6;
            const int    ci  = i / n_sub;
            const float  r   = (link_radii != nullptr)
                                   ? static_cast<float>(link_radii[ci]) + pad
                                   : pad;
            const float inflated[6] = {
                src[0] - r, src[1] - r, src[2] - r,
                src[3] + r, src[4] + r, src[5] + r,
            };
            sg->fill_aabb(inflated);
        }
        result.sparse_grid = std::move(sg);
    }

    // Hull16_Grid: rasterise per-link convex hull segments.
    if (config.type == EnvelopeType::Hull16_Grid) {
        const double delta = config.grid_config.voxel_delta;
        auto sg = std::make_unique<sbf::voxel::SparseVoxelGrid>(delta);

        for (int i = 0; i < n_active_links; ++i) {
            const float* prox = endpoint_iaabbs + i * 12;
            const float* dist = endpoint_iaabbs + i * 12 + 6;
            const double r    = link_radii ? link_radii[i] : 0.0;
            sg->fill_hull16(prox, dist, r);
        }
        result.sparse_grid = std::move(sg);
    }

    return result;
}

}  // namespace sbf::envelope
