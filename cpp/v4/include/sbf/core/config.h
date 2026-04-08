// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Global configuration (cross-module shared types)
//  Module: sbf::core
//
//  Only cross-module shared config types live here.
//  Module-specific configs live in their own headers.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include <cstdint>
#include <string>

namespace sbf {

// ─── Collision policy ───────────────────────────────────────────────────────
enum class CollisionPolicy : uint8_t {
    AABB        = 0,  // Per-link iAABB check only
    AABB_SUBDIV = 1,  // Per-link subdivided iAABB check
    GRID        = 2   // Sparse voxel grid (BitBrick) collision
};

// ─── Store format (for cache persistence) ───────────────────────────────────
enum class StoreFormat : uint8_t {
    HCACHE02 = 0,  // KD-tree + per-link AABBs
    FRM4     = 1,  // Endpoint iAABBs (per-endpoint position intervals)
    HUL1     = 2   // Hull VoxelGrids (sparse BitBrick)
};

// ─── Envelope config (used by LECT FFB) ─────────────────────────────────────
struct EnvelopeConfig {
    int n_sub = 1;
    double delta = 0.01;
    int grid_R = 64;
    float world_bounds[6] = {-0.8f, -1.2f, 0.0f, 1.8f, 1.2f, 1.4f};
};
// ─── Split order for KD-tree dimension selection ───────────────────────────────────
enum class SplitOrder : uint8_t {
    ROUND_ROBIN     = 0,  // v3 default: cycle [0,1,...,n-1,0,1,...]
    WIDEST_FIRST    = 2,  // Always split widest dim (dynamic, per-node)
    BEST_TIGHTEN    = 3   // iFK probe: per-depth lazy selection of
                          // dimension that minimises minimax link volume
};

} // namespace sbf
