// SafeBoxForest v6 — Link IAABB (Phase C1)
#include <sbf/envelope/link_iaabb.h>

#include <algorithm>
#include <cassert>
#include <cmath>

namespace sbf {

void derive_link_iaabb_paired(
    const float* endpoint_iaabbs,
    int n_active_links,
    const double* link_radii,
    float* out_link_iaabbs)
{
    assert(endpoint_iaabbs && out_link_iaabbs);

    for (int ci = 0; ci < n_active_links; ++ci) {
        const float* prox = endpoint_iaabbs + (ci * 2) * 6;
        const float* dist = endpoint_iaabbs + (ci * 2 + 1) * 6;
        float* out = out_link_iaabbs + ci * 6;

        float r = (link_radii != nullptr)
                      ? static_cast<float>(link_radii[ci])
                      : 0.0f;

        // lo = min(prox.lo, dist.lo) - radius
        out[0] = std::min(prox[0], dist[0]) - r;
        out[1] = std::min(prox[1], dist[1]) - r;
        out[2] = std::min(prox[2], dist[2]) - r;

        // hi = max(prox.hi, dist.hi) + radius
        out[3] = std::max(prox[3], dist[3]) + r;
        out[4] = std::max(prox[4], dist[4]) + r;
        out[5] = std::max(prox[5], dist[5]) + r;
    }
}

void derive_link_iaabb_subdivided(
    const float* endpoint_iaabbs,
    int n_active_links,
    const double* link_radii,
    int n_sub,
    float* out_sub_iaabbs)
{
    assert(endpoint_iaabbs && out_sub_iaabbs);
    assert(n_sub >= 1);

    const float inv_n = 1.0f / static_cast<float>(n_sub);

    for (int ci = 0; ci < n_active_links; ++ci) {
        const float* prox = endpoint_iaabbs + (ci * 2) * 6;
        const float* dist = endpoint_iaabbs + (ci * 2 + 1) * 6;

        float r = (link_radii != nullptr)
                      ? static_cast<float>(link_radii[ci])
                      : 0.0f;

        for (int s = 0; s < n_sub; ++s) {
            float t_lo = static_cast<float>(s) * inv_n;
            float t_hi = static_cast<float>(s + 1) * inv_n;

            float* out = out_sub_iaabbs + (ci * n_sub + s) * 6;

            for (int d = 0; d < 3; ++d) {
                // Interval lerp: lo bound
                float lo_at_t0 = prox[d] * (1.0f - t_lo) + dist[d] * t_lo;
                float lo_at_t1 = prox[d] * (1.0f - t_hi) + dist[d] * t_hi;
                out[d] = std::min(lo_at_t0, lo_at_t1) - r;

                // Interval lerp: hi bound
                float hi_at_t0 = prox[d + 3] * (1.0f - t_lo) + dist[d + 3] * t_lo;
                float hi_at_t1 = prox[d + 3] * (1.0f - t_hi) + dist[d + 3] * t_hi;
                out[d + 3] = std::max(hi_at_t0, hi_at_t1) + r;
            }
        }
    }
}

}  // namespace sbf
