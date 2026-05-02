#pragma once
/// @file lect_cache_manager.h
/// @brief LectCacheManager — manages per-channel Z4 EP and Grid caches.
///
/// Cache directory structure:
///   ~/.sbf_cache/<robot_hash>/
///     ep_safe.cache        Z4EpCache (safe-channel EP iAABBs)
///     ep_unsafe.cache      Z4EpCache (unsafe-channel EP iAABBs)
///     grid_safe.cache      Z4GridCache (safe-channel grids, quality-aware; grid envelopes only)
///     grid_unsafe.cache    Z4GridCache (unsafe-channel grids, quality-aware; grid envelopes only)

#include <sbf/lect/z4_ep_cache.h>
#include <sbf/lect/z4_grid_cache.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/envelope/endpoint_source.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace sbf {

class LectCacheManager {
public:
    LectCacheManager() = default;
    ~LectCacheManager();

    /// Initialize the cache manager for a given robot.
    /// Opens EP cache files and opens grid cache files only for grid envelope
    /// variants. LinkIAABB payloads do not need grid files.
    /// @param robot_hash    Unique hash of the robot's DH parameters
    /// @param robot_name    Human-readable robot name (for logging)
    /// @param ep_stride     Number of floats per channel per node
    /// @param ep_src        Endpoint-source variant (cache dir is isolated
    ///                      per source so caches built with different EP
    ///                      strategies do not collide).
    /// @param env_type      Envelope type variant (similar isolation).
    /// @param cache_dir     Base cache directory (default: ~/.sbf_cache)
    /// @param ep_max_cap    Max EP hash slots per channel (0=unlimited)
    /// @param grid_max_cap  Max grid index slots per channel (0=unlimited)
    /// @return true on success
    bool init(uint64_t robot_hash, const std::string& robot_name,
              int ep_stride,
              EndpointSource ep_src, EnvelopeType env_type,
              const std::string& cache_dir = "",
              int ep_max_cap = 0, int grid_max_cap = 0,
              int ep_initial_cap = 4096, int grid_initial_cap = 4096);

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

    /// Queue cache writes on a background worker. The caller pays only the
    /// in-memory copy needed to decouple the cache from LECT node storage.
    void enqueue_ep_insert(int channel, uint64_t key, EndpointSource source,
                           const float* ep_data, const float* liaabb_data);
    void enqueue_grid_insert(int channel, uint64_t key,
                             std::shared_ptr<const voxel::SparseVoxelGrid> grid,
                             const GridQuality& quality);

    /// Wait until all queued writes are durable in the underlying cache files.
    void flush_async();

    /// Print cache stats to stderr.
    void print_stats() const;

private:
    enum class AsyncTaskType : uint8_t { EpInsert, GridInsert };

    struct AsyncTask {
        AsyncTaskType type = AsyncTaskType::EpInsert;
        int channel = 0;
        uint64_t key = 0;
        EndpointSource ep_source = EndpointSource::IFK;
        GridQuality grid_quality{};
        std::vector<float> ep_data;
        std::vector<float> liaabb_data;
        std::shared_ptr<const voxel::SparseVoxelGrid> grid;
    };

    void start_async_worker();
    void stop_async_worker();
    void async_worker_loop();

    std::string cache_dir_;
    int ep_stride_ = 0;
    int liaabb_stride_ = 0;
    bool grid_enabled_ = false;

    Z4EpCache    ep_safe_;
    Z4EpCache    ep_unsafe_;
    Z4GridCache  grid_safe_;
    Z4GridCache  grid_unsafe_;

    std::mutex async_mu_;
    std::condition_variable async_cv_;
    std::condition_variable async_idle_cv_;
    std::deque<AsyncTask> async_queue_;
    std::thread async_worker_;
    bool async_stop_ = false;
    size_t async_inflight_ = 0;
};

}  // namespace sbf
