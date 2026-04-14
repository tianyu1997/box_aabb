#pragma once
/// @file lect_cache_manager.h
/// @brief LectCacheManager — manages per-channel Z4 EP and Grid caches.
///
/// Cache directory structure:
///   ~/.sbf_cache/<robot_hash>/
///     ep_safe.cache        Z4EpCache (safe-channel EP iAABBs)
///     ep_unsafe.cache      Z4EpCache (unsafe-channel EP iAABBs)
///     grid_safe.cache      Z4GridCache (safe-channel grids, quality-aware)
///     grid_unsafe.cache    Z4GridCache (unsafe-channel grids, quality-aware)

#include <sbf/lect/z4_ep_cache.h>
#include <sbf/lect/z4_grid_cache.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/envelope/endpoint_source.h>

#include <cstdint>
#include <string>

namespace sbf {

class LectCacheManager {
public:
    LectCacheManager() = default;

    /// Initialize the cache manager for a given robot.
    /// Opens 4 cache files: ep_safe, ep_unsafe, grid_safe, grid_unsafe.
    /// @param robot_hash    Unique hash of the robot's DH parameters
    /// @param robot_name    Human-readable robot name (for logging)
    /// @param ep_stride     Number of floats per channel per node
    /// @param cache_dir     Base cache directory (default: ~/.sbf_cache)
    /// @param ep_max_cap    Max EP hash slots per channel (0=unlimited)
    /// @param grid_max_cap  Max grid index slots per channel (0=unlimited)
    /// @return true on success
    bool init(uint64_t robot_hash, const std::string& robot_name,
              int ep_stride, const std::string& cache_dir = "",
              int ep_max_cap = 0, int grid_max_cap = 0);

    // ─── EP cache access (per channel) ──────────────────────────────────

    /// Get the EP cache for a specific channel (CH_SAFE=0, CH_UNSAFE=1).
    Z4EpCache& ep_cache(int channel) {
        return channel == 0 ? ep_safe_ : ep_unsafe_;
    }
    const Z4EpCache& ep_cache(int channel) const {
        return channel == 0 ? ep_safe_ : ep_unsafe_;
    }

    // ─── Grid cache access (per channel) ────────────────────────────────

    /// Get the grid cache for a specific channel.
    Z4GridCache& grid_cache(int channel) {
        return channel == 0 ? grid_safe_ : grid_unsafe_;
    }
    const Z4GridCache& grid_cache(int channel) const {
        return channel == 0 ? grid_safe_ : grid_unsafe_;
    }

    /// Cache directory path.
    const std::string& cache_dir() const { return cache_dir_; }

    /// Print cache stats to stderr.
    void print_stats() const;

private:
    std::string cache_dir_;
    int ep_stride_ = 0;

    Z4EpCache    ep_safe_;
    Z4EpCache    ep_unsafe_;
    Z4GridCache  grid_safe_;
    Z4GridCache  grid_unsafe_;
};

}  // namespace sbf
