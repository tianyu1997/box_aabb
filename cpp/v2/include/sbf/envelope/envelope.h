// SafeBoxForest v2 — Abstract Envelope Interface
// Module: sbf::envelope
//
// Design:
//   IEnvelope         — abstract result returned by a computer; knows how
//                       to collision-test itself against obstacles.
//   IEnvelopeComputer — abstract factory: intervals → IEnvelope.
//
// Concrete implementations:
//   - GridEnvelope / GridComputer  (grid_envelope.h)
//
// NOTE: IEnvelopeCache was removed in HCACHE03 cleanup.  Caching is now
//       handled by FrameStore + collision_policy.h.
//
#pragma once

#include "sbf/common/types.h"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace sbf {
namespace envelope {

// ─── Envelope kind tag ──────────────────────────────────────────────────────
// Stored in caches so the loader knows which concrete class to instantiate.
enum class EnvelopeKind : uint8_t {
    AABB           = 0,   // per-link axis-aligned bounding boxes
    OBB            = 1,   // per-link oriented bounding boxes (future)
    Grid           = 2,   // voxel grid occupancy (future)
    ConvexHull     = 3,   // per-link convex hulls (future)
    // Reserve 4-15 for user extensions
};

// Human-readable name (for logs / JSON)
inline const char* envelope_kind_name(EnvelopeKind k) {
    switch (k) {
        case EnvelopeKind::AABB:       return "AABB";
        case EnvelopeKind::OBB:        return "OBB";
        case EnvelopeKind::Grid:       return "Grid";
        case EnvelopeKind::ConvexHull: return "ConvexHull";
        default:                       return "Unknown";
    }
}

// ─── IEnvelope — abstract envelope result ──────────────────────────────────
//
// An envelope encapsulates the workspace geometry of a robot over a C-space
// box.  The only operations the rest of the system needs are:
//   - collision test against an obstacle set
//   - serialisation / deserialisation (for caching)
//   - geometric queries (volume, containment)
//
class IEnvelope {
public:
    virtual ~IEnvelope() = default;

    // ── Identity ────────────────────────────────────────────────────────
    virtual EnvelopeKind kind() const = 0;

    // ── Collision ───────────────────────────────────────────────────────
    // Test if any link envelope overlaps any obstacle.
    // `obs_compact` is a flat array of obstacles in [x_lo, y_lo, z_lo,
    // x_hi, y_hi, z_hi] layout, n_obs entries.
    virtual bool collides(const float* obs_compact, int n_obs) const = 0;

    // ── Geometric queries ───────────────────────────────────────────────
    // Total workspace volume covered by the envelope.
    virtual double volume() const = 0;

    // Number of geometric primitives (AABBs, OBBs, voxels, …)
    virtual int n_primitives() const = 0;

    // ── Validity ────────────────────────────────────────────────────────
    virtual bool valid() const = 0;

    // ── Serialisation ───────────────────────────────────────────────────
    // Serialise to a byte blob (for caching).
    // Returns the blob; on failure returns empty vector.
    virtual std::vector<uint8_t> serialise() const = 0;

    // Factory: deserialise from a byte blob.
    // Implemented by each concrete class; dispatched via EnvelopeKind tag
    // in the cache reader.
    // Returns nullptr on failure.
    // (Static factory lives in envelope_registry.h)
};

// Shared ownership (envelopes may be cached and referenced by multiple boxes)
using EnvelopePtr = std::shared_ptr<const IEnvelope>;

// ─── IEnvelopeComputer — abstract envelope factory ─────────────────────────
//
// Given a set of C-space intervals, produce an envelope.
// Different strategies (interval FK, critical sampling, random sampling,
// OBB fitting, …) implement this interface.
//
class IEnvelopeComputer {
public:
    virtual ~IEnvelopeComputer() = default;

    // ── Identity ────────────────────────────────────────────────────────
    virtual EnvelopeKind kind() const = 0;
    virtual std::string  name() const = 0;

    // ── Compute ─────────────────────────────────────────────────────────
    // Full computation from scratch.
    virtual EnvelopePtr compute(
        const std::vector<Interval>& intervals) const = 0;

    // Incremental computation after changing one dimension.
    // `parent` is the envelope of the parent box (before the split).
    // Default implementation falls back to full computation.
    virtual EnvelopePtr compute_incremental(
        const EnvelopePtr& parent,
        const std::vector<Interval>& intervals,
        int changed_dim) const
    {
        (void)parent; (void)changed_dim;
        return compute(intervals);
    }

    // ── Capacity ────────────────────────────────────────────────────────
    // Number of AABB / OBB / primitive slots this computer produces.
    virtual int n_total_slots() const = 0;
};

} // namespace envelope
} // namespace sbf
