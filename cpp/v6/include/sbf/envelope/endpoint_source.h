#pragma once
/// @file endpoint_source.h
/// @brief Endpoint iAABB computation: multiple source methods + source substitution.
///
/// Four methods to compute endpoint (proximal + distal) interval AABBs:
///   - **IFK** (safe) — Interval Forward Kinematics, conservative enclosure.
///   - **CritSample** (unsafe) — Critical-point + random sampling.
///   - **Analytical** (safe) — Multi-phase closed-form critical-point solve.
///   - **GCPC** (safe) — Pre-computed interior critical points cache.
///
/// A substitution matrix (`kSourceSubstitutionMatrix`) determines when
/// cached data from one source can serve a request for another source.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>

#include <cstdint>
#include <vector>

namespace sbf {

// Forward declaration
class GcpcCache;

// ─── Endpoint source method ─────────────────────────────────────────────────
enum class EndpointSource : uint8_t {
    IFK         = 0,
    CritSample  = 1,
    Analytical  = 2,
    GCPC        = 3
};

inline const char* endpoint_source_name(EndpointSource s) {
    switch (s) {
        case EndpointSource::IFK:        return "IFK";
        case EndpointSource::CritSample: return "CritSample";
        case EndpointSource::Analytical: return "Analytical";
        case EndpointSource::GCPC:       return "GCPC";
        default:                         return "Unknown";
    }
}

// ─── Configuration ──────────────────────────────────────────────────────────
struct EndpointSourceConfig {
    EndpointSource source = EndpointSource::IFK;
    int n_samples_crit = 1000;          // CritSample
    int max_phase_analytical = 3;       // Analytical (0..3)
    const GcpcCache* gcpc_cache = nullptr;  // GCPC (not owned)
};

// ─── Result ─────────────────────────────────────────────────────────────────
struct EndpointIAABBResult {
    std::vector<float> endpoint_iaabbs;  // [n_active × 2 × 6]
    EndpointSource source = EndpointSource::IFK;
    bool is_safe = false;
    int n_active_links = 0;

    // FK state (valid for IFK; may be empty for other sources)
    FKState fk_state;

    // Analytical source: number of links fully pruned by AA Gap Pruning
    int n_pruned_links = 0;

    int endpoint_iaabb_len() const { return n_active_links * 2 * 6; }
};

// ─── Unified entry point ────────────────────────────────────────────────────
EndpointIAABBResult compute_endpoint_iaabb(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const EndpointSourceConfig& config,
    FKState* fk = nullptr,
    int changed_dim = -1);
// ─── Source substitution matrix ─────────────────────────────────────────────────
// Row = cached source, Column = requested source.
// Order: IFK(0), CritSample(1), Analytical(2), GCPC(3).
inline constexpr int kEndpointSourceCount = 4;

inline constexpr bool kSourceSubstitutionMatrix[kEndpointSourceCount][kEndpointSourceCount] = {
    // requested:          IFK    Crit   Analyt GCPC
    /* cached IFK */      {true,  false, false, false},
    /* cached Crit */     {false, true,  false, false},
    /* cached Analytical*/{true,  true,  true,  false},
    /* cached GCPC */     {true,  true,  true,  true },
};

static_assert(static_cast<int>(EndpointSource::IFK) == 0,
              "EndpointSource enum order changed: update substitution matrix");
static_assert(static_cast<int>(EndpointSource::CritSample) == 1,
              "EndpointSource enum order changed: update substitution matrix");
static_assert(static_cast<int>(EndpointSource::Analytical) == 2,
              "EndpointSource enum order changed: update substitution matrix");
static_assert(static_cast<int>(EndpointSource::GCPC) == 3,
              "EndpointSource enum order changed: update substitution matrix");

/// Can a cached result (from `cached`) serve a request for `requested`?
inline bool source_can_serve(EndpointSource cached, EndpointSource requested) {
    const int ci = static_cast<int>(cached);
    const int ri = static_cast<int>(requested);
    if (ci < 0 || ci >= kEndpointSourceCount ||
        ri < 0 || ri >= kEndpointSourceCount)
        return false;
    return kSourceSubstitutionMatrix[ci][ri];
}

/// Map endpoint source to channel index: IFK → CH_SAFE (0), else → CH_UNSAFE (1).
inline int source_channel(EndpointSource s) {
    return (s == EndpointSource::IFK) ? 0 : 1;
}

// ─── Hull (union) of endpoint iAABBs ───────────────────────────────────────────────
// Element-wise hull: dst[i] = min(dst[i], src[i]) for lo, max for hi.
// Layout: n_endpoints × 6 floats  [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
void hull_endpoint_iaabbs(float* dst, const float* src, int n_endpoints);
}  // namespace sbf
