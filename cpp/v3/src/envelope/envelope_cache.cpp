// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v3 — Unified Cache Module implementation
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/envelope/envelope_cache.h"

#include <nlohmann/json.hpp>

#include <filesystem>
#include <fstream>
#include <functional>

namespace fs = std::filesystem;
using json = nlohmann::json;

namespace sbf {
namespace envelope {

// ═════════════════════════════════════════════════════════════════════════════
//  available_layers — check which cache files exist
// ═════════════════════════════════════════════════════════════════════════════
uint8_t EnvelopeCache::available_layers(const std::string& dir)
{
    uint8_t layers = CACHE_NONE;
    if (fs::exists(hcache_path(dir))) layers |= CACHE_TREE;
    if (fs::exists(frames_path(dir))) layers |= CACHE_FRAMES;
    if (fs::exists(hulls_path(dir)))  layers |= CACHE_HULLS;
    return layers;
}

// ═════════════════════════════════════════════════════════════════════════════
//  is_valid — check cache validity for a pipeline config
// ═════════════════════════════════════════════════════════════════════════════
bool EnvelopeCache::is_valid(const std::string& dir,
                              const PipelineConfig& config,
                              const Robot& robot)
{
    CacheMetadata meta;
    if (!load_metadata(dir, meta))
        return false;

    // Check robot fingerprint
    uint64_t rh = compute_robot_hash(robot);
    if (meta.robot_hash != 0 && meta.robot_hash != rh)
        return false;

    // Check pipeline config compatibility
    // Source compatibility: cached source can serve if quality >= requested
    if (!source_can_serve(meta.source_method, config.source.method))
        return false;
    if (meta.envelope_type != config.envelope.type)
        return false;

    // Safety: after AABB→SubAABB merge, enum values were renumbered.
    // Cross-check the persisted envelope_name to reject stale caches.
    if (!meta.envelope_name.empty()) {
        std::string expected = envelope_type_name(config.envelope.type);
        if (meta.envelope_name != expected)
            return false;
    }

    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
//  save_metadata / load_metadata — JSON pipeline metadata
// ═════════════════════════════════════════════════════════════════════════════
void EnvelopeCache::save_metadata(const std::string& dir,
                                   const CacheMetadata& meta)
{
    fs::create_directories(dir);

    json j;
    j["source_method"]  = static_cast<int>(meta.source_method);
    j["envelope_type"]  = static_cast<int>(meta.envelope_type);
    j["source_name"]    = frame_source_name(meta.source_method);
    j["envelope_name"]  = envelope_type_name(meta.envelope_type);
    j["n_sub"]          = meta.n_sub;
    j["delta"]          = meta.delta;
    j["grid_R"]         = meta.grid_R;
    j["n_nodes"]        = meta.n_nodes;
    j["n_frames"]       = meta.n_frames;
    j["n_hulls"]        = meta.n_hulls;
    j["robot_hash"]     = meta.robot_hash;

    std::ofstream f(meta_path(dir));
    f << j.dump(2) << "\n";
}

bool EnvelopeCache::load_metadata(const std::string& dir,
                                   CacheMetadata& meta)
{
    std::string path = meta_path(dir);
    if (!fs::exists(path))
        return false;

    try {
        std::ifstream f(path);
        json j;
        f >> j;

        meta.source_method = static_cast<FrameSourceMethod>(
            j.value("source_method", 0));
        meta.envelope_type = static_cast<EnvelopeType>(
            j.value("envelope_type", 2));
        meta.envelope_name = j.value("envelope_name", std::string(""));
        meta.n_sub     = j.value("n_sub", 1);
        meta.delta     = j.value("delta", 0.01);
        meta.grid_R    = j.value("grid_R", 64);
        meta.n_nodes   = j.value("n_nodes", 0);
        meta.n_frames  = j.value("n_frames", 0);
        meta.n_hulls   = j.value("n_hulls", 0);
        meta.robot_hash = j.value("robot_hash", static_cast<uint64_t>(0));
        return true;
    } catch (...) {
        return false;
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  compute_robot_hash — FNV-1a hash of robot DH configuration
// ═════════════════════════════════════════════════════════════════════════════
uint64_t EnvelopeCache::compute_robot_hash(const Robot& robot)
{
    // FNV-1a 64-bit
    constexpr uint64_t FNV_OFFSET = 14695981039346656037ULL;
    constexpr uint64_t FNV_PRIME  = 1099511628211ULL;

    uint64_t h = FNV_OFFSET;

    auto hash_bytes = [&](const void* data, size_t n) {
        auto* p = static_cast<const uint8_t*>(data);
        for (size_t i = 0; i < n; ++i) {
            h ^= p[i];
            h *= FNV_PRIME;
        }
    };

    // Hash n_joints, n_active_links
    int nj = robot.n_joints();
    int na = robot.n_active_links();
    hash_bytes(&nj, sizeof(nj));
    hash_bytes(&na, sizeof(na));

    // Hash joint limits
    const auto& lim = robot.joint_limits().limits;
    for (int j = 0; j < nj; ++j) {
        hash_bytes(&lim[j].lo, sizeof(double));
        hash_bytes(&lim[j].hi, sizeof(double));
    }

    // Hash active link radii
    const double* lr = robot.active_link_radii();
    if (lr) hash_bytes(lr, na * sizeof(double));

    // Hash active link map
    const int* alm = robot.active_link_map();
    if (alm) hash_bytes(alm, na * sizeof(int));

    return h;
}

} // namespace envelope
} // namespace sbf
