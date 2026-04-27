// SafeBoxForest v7 — AABB-vs-Obstacle SAT collision tests (P2).
#include "sbf/scene/collision.h"

namespace sbf::scene {

namespace {
constexpr float kSatEps = 1e-10f;
}

bool aabbs_collide_obs(const float* aabb, int n_slots,
                       const float* obs_compact, int n_obs) {
    for (int si = 0; si < n_slots; ++si) {
        const int   off    = si * 6;
        const float a_lo_x = aabb[off];
        const float a_lo_y = aabb[off + 1];
        const float a_lo_z = aabb[off + 2];
        const float a_hi_x = aabb[off + 3];
        const float a_hi_y = aabb[off + 4];
        const float a_hi_z = aabb[off + 5];

        for (int oi = 0; oi < n_obs; ++oi) {
            const float* o = obs_compact + oi * 6;
            if (a_hi_x < o[0] - kSatEps || a_lo_x > o[1] + kSatEps) continue;
            if (a_hi_y < o[2] - kSatEps || a_lo_y > o[3] + kSatEps) continue;
            if (a_hi_z < o[4] - kSatEps || a_lo_z > o[5] + kSatEps) continue;
            return true;
        }
    }
    return false;
}

bool aabbs_collide_obs_inflated(const float* zero_aabb, int n_slots,
                                const float* link_radii_per_slot,
                                const float* obs_compact, int n_obs) {
    for (int si = 0; si < n_slots; ++si) {
        const int   off = si * 6;
        const float r   = link_radii_per_slot ? link_radii_per_slot[si] : 0.0f;

        const float a_lo_x = zero_aabb[off]     - r;
        const float a_lo_y = zero_aabb[off + 1] - r;
        const float a_lo_z = zero_aabb[off + 2] - r;
        const float a_hi_x = zero_aabb[off + 3] + r;
        const float a_hi_y = zero_aabb[off + 4] + r;
        const float a_hi_z = zero_aabb[off + 5] + r;

        for (int oi = 0; oi < n_obs; ++oi) {
            const float* o = obs_compact + oi * 6;
            if (a_hi_x < o[0] - kSatEps || a_lo_x > o[1] + kSatEps) continue;
            if (a_hi_y < o[2] - kSatEps || a_lo_y > o[3] + kSatEps) continue;
            if (a_hi_z < o[4] - kSatEps || a_lo_z > o[5] + kSatEps) continue;
            return true;
        }
    }
    return false;
}

void pack_obstacles(const Obstacle* obs, int n_obs, float* out_compact) {
    for (int i = 0; i < n_obs; ++i) {
        const float* b = obs[i].bounds;          // xyzxyz
        float*       p = out_compact + i * 6;    // xhxyhyzhz
        p[0] = b[0]; p[1] = b[3];
        p[2] = b[1]; p[3] = b[4];
        p[4] = b[2]; p[5] = b[5];
    }
}

void expand_radii_to_slots(const double* link_radii,
                           int n_active_links,
                           int n_subdivisions,
                           float* out_per_slot) {
    if (link_radii == nullptr) {
        for (int i = 0; i < n_active_links * n_subdivisions; ++i)
            out_per_slot[i] = 0.0f;
        return;
    }
    for (int ci = 0; ci < n_active_links; ++ci) {
        const float r = static_cast<float>(link_radii[ci]);
        for (int s = 0; s < n_subdivisions; ++s)
            out_per_slot[ci * n_subdivisions + s] = r;
    }
}

}  // namespace sbf::scene
