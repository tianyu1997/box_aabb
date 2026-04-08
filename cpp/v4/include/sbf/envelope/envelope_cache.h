// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Unified Cache Module
//  Module: sbf::envelope
//
//  Unified persistence layer that manages three cache formats:
//
//    HCACHE02 (NodeStore): Tree structure + per-link AABBs
//    FRM4 (EndpointStore): Per-node endpoint position intervals
//    HUL1 (Hull grids): Per-node Hull-16 VoxelGrids (sparse BitBrick)
//
//  The cache module provides:
//    - Unified save/load API across all three formats
//    - Pipeline metadata persistence (EndpointSource + EnvelopeType)
//    - Cache validity checking (robot config fingerprint)
//
//  迁移自 v3 envelope_cache.h，适配 v4 命名
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/envelope/endpoint_source.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/envelope/pipeline.h"
#include "sbf/robot/robot.h"

#include <cstdint>
#include <string>

namespace sbf {
namespace envelope {

struct CacheMetadata {
    EndpointSource    source_method = EndpointSource::IFK;
    EnvelopeType      envelope_type = EnvelopeType::Hull16_Grid;
    std::string       envelope_name;
    int    n_sub     = 1;
    double delta     = 0.01;
    int    grid_R    = 64;
    int    n_nodes   = 0;
    int    n_endpoints = 0;
    int    n_hulls   = 0;
    uint64_t robot_hash = 0;
};

enum CacheLayer : uint8_t {
    CACHE_NONE      = 0,
    CACHE_TREE      = 1 << 0,
    CACHE_ENDPOINTS = 1 << 1,
    CACHE_HULLS     = 1 << 2,
    CACHE_ALL       = CACHE_TREE | CACHE_ENDPOINTS | CACHE_HULLS
};

class EnvelopeCache {
public:
    EnvelopeCache() = default;

    static uint8_t available_layers(const std::string& dir);
    static bool is_valid(const std::string& dir,
                         const PipelineConfig& config,
                         const Robot& robot);
    static void save_metadata(const std::string& dir,
                              const CacheMetadata& meta);
    static bool load_metadata(const std::string& dir,
                              CacheMetadata& meta);
    static uint64_t compute_robot_hash(const Robot& robot);

    static std::string hcache_path(const std::string& dir) {
        return dir + "/lect.hcache";
    }
    static std::string endpoints_path(const std::string& dir) {
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
