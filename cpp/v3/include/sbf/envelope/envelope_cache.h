// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — Unified Cache Module
//  Module: sbf::envelope
//
//  Unified persistence layer that manages three cache formats:
//
//    HCACHE02 (NodeStore): Tree structure + per-link AABBs
//      - Binary format: tree topology, per-node metadata, flat AABB arrays
//      - File: <dir>/lect.hcache
//
//    FRM4 (FrameStore): Per-node frame position intervals
//      - Mmap-friendly flat indexed format: [valid:1B][pad:7B][frames]
//      - File: <dir>/lect.frames
//      - Supports lazy load via mmap, incremental saves
//
//    HUL1 (Hull grids): Per-node Hull-16 VoxelGrids (sparse BitBrick)
//      - Header + per-node { valid, n_bricks, [BrickCoord + BitBrick]* }
//      - File: <dir>/lect.hulls
//
//  The cache module provides:
//    - Unified save/load API across all three formats
//    - Pipeline metadata persistence (FrameSourceMethod + EnvelopeType)
//    - Cache validity checking (robot config fingerprint)
//
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/envelope/frame_source.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/envelope/frame_store.h"
#include "sbf/forest/node_store.h"
#include "sbf/voxel/voxel_grid.h"

#include <string>
#include <vector>

namespace sbf {
namespace envelope {

// ─── Cache metadata (persisted alongside data) ──────────────────────────────
struct CacheMetadata {
    FrameSourceMethod source_method = FrameSourceMethod::IFK;
    EnvelopeType      envelope_type = EnvelopeType::Hull16_Grid;
    std::string       envelope_name;   // cached name for cross-version validation
    int    n_sub     = 1;
    double delta     = 0.01;
    int    grid_R    = 64;
    int    n_nodes   = 0;
    int    n_frames  = 0;
    int    n_hulls   = 0;

    // Robot fingerprint (DH hash for invalidation)
    uint64_t robot_hash = 0;
};

// ─── Cache layer flags ──────────────────────────────────────────────────────
enum CacheLayer : uint8_t {
    CACHE_NONE   = 0,
    CACHE_TREE   = 1 << 0,  // HCACHE02: tree structure + AABBs
    CACHE_FRAMES = 1 << 1,  // FRM4: frame position intervals
    CACHE_HULLS  = 1 << 2,  // HUL1: hull-16 VoxelGrids
    CACHE_ALL    = CACHE_TREE | CACHE_FRAMES | CACHE_HULLS
};

// ─── Unified cache interface ────────────────────────────────────────────────
class EnvelopeCache {
public:
    EnvelopeCache() = default;

    // ── Query ───────────────────────────────────────────────────────────

    /// Check which cache layers exist in the given directory.
    static uint8_t available_layers(const std::string& dir);

    /// Check if cache directory has valid data for the given pipeline config.
    static bool is_valid(const std::string& dir,
                         const PipelineConfig& config,
                         const Robot& robot);

    // ── Save / Load metadata ────────────────────────────────────────────

    /// Save pipeline metadata to <dir>/lect.meta.json
    static void save_metadata(const std::string& dir,
                              const CacheMetadata& meta);

    /// Load pipeline metadata from <dir>/lect.meta.json
    /// Returns false if file doesn't exist or is invalid.
    static bool load_metadata(const std::string& dir,
                              CacheMetadata& meta);

    // ── Robot fingerprint ───────────────────────────────────────────────

    /// Compute a hash of robot configuration for cache invalidation.
    static uint64_t compute_robot_hash(const Robot& robot);

    // ── File path helpers ───────────────────────────────────────────────

    static std::string hcache_path(const std::string& dir) {
        return dir + "/lect.hcache";
    }
    static std::string frames_path(const std::string& dir) {
        return dir + "/lect.frames";
    }
    static std::string hulls_path(const std::string& dir) {
        return dir + "/lect.hulls";
    }
    static std::string meta_path(const std::string& dir) {
        return dir + "/lect.meta.json";
    }
};

} // namespace envelope
} // namespace sbf
