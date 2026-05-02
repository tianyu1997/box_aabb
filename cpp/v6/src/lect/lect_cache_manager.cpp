/// @file lect_cache_manager.cpp
/// @brief LectCacheManager implementation — per-channel EP + Grid caches.

#include <sbf/lect/lect_cache_manager.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>
#include <sbf/core/log.h>

namespace sbf {

// ─── Helper: ensure directory exists ────────────────────────────────────────
static bool ensure_dir(const std::string& path) {
    struct stat st;
    if (::stat(path.c_str(), &st) == 0 && S_ISDIR(st.st_mode))
        return true;
    if (::mkdir(path.c_str(), 0755) == 0) return true;
    auto pos = path.rfind('/');
    if (pos != std::string::npos && pos > 0) {
        ensure_dir(path.substr(0, pos));
        return ::mkdir(path.c_str(), 0755) == 0;
    }
    return false;
}

// ─── Init ───────────────────────────────────────────────────────────────────
bool LectCacheManager::init(uint64_t robot_hash, const std::string& robot_name,
                            int ep_stride,
                            EndpointSource ep_src, EnvelopeType env_type,
                            const std::string& cache_dir,
                            int ep_max_cap, int grid_max_cap,
                            int ep_initial_cap, int grid_initial_cap) {
    stop_async_worker();
    ep_stride_ = ep_stride;
    liaabb_stride_ = ep_stride / 2;
    grid_enabled_ = (env_type != EnvelopeType::LinkIAABB);
    grid_safe_.close();
    grid_unsafe_.close();

    // Determine cache directory
    if (cache_dir.empty()) {
        const char* home = std::getenv("HOME");
        if (!home) home = "/tmp";
        cache_dir_ = std::string(home) + "/.sbf_cache";
    } else {
        cache_dir_ = cache_dir;
    }

    // Robot-specific subdirectory, further isolated by (ep_src, env_type)
    // so caches built with different EP/envelope variants never alias.
    char hash_str[32];
    std::snprintf(hash_str, sizeof(hash_str), "%016llx",
                  static_cast<unsigned long long>(robot_hash));
    char variant_tag[16];
    std::snprintf(variant_tag, sizeof(variant_tag), "ep%d_env%d",
                  static_cast<int>(ep_src), static_cast<int>(env_type));
    cache_dir_ = cache_dir_ + "/" + hash_str + "/" + variant_tag;

    if (!ensure_dir(cache_dir_)) {
        SBF_WARN("[LectCacheManager] mkdir(%s) failed: %s", cache_dir_.c_str(), strerror(errno));
        return false;
    }

    // Open 2 EP caches (safe / unsafe)
    std::string ep_safe_path   = cache_dir_ + "/ep_safe.cache";
    std::string ep_unsafe_path = cache_dir_ + "/ep_unsafe.cache";

    // ep_stride = n_active_links * 2 * 6, liaabb_stride = n_active_links * 6
    const int liaabb_stride = ep_stride / 2;

    if (!ep_safe_.open(ep_safe_path, ep_stride, liaabb_stride, ep_initial_cap, ep_max_cap)) {
        SBF_WARN("[LectCacheManager] EP safe cache open failed: %s", ep_safe_path.c_str());
        return false;
    }
    if (!ep_unsafe_.open(ep_unsafe_path, ep_stride, liaabb_stride, ep_initial_cap, ep_max_cap)) {
        SBF_WARN("[LectCacheManager] EP unsafe cache open failed: %s", ep_unsafe_path.c_str());
        return false;
    }

    if (grid_enabled_) {
        // Open 2 Grid caches (safe / unsafe) only when grid payloads exist.
        std::string grid_safe_path   = cache_dir_ + "/grid_safe.cache";
        std::string grid_unsafe_path = cache_dir_ + "/grid_unsafe.cache";

        if (!grid_safe_.open(grid_safe_path, grid_initial_cap, grid_max_cap)) {
            SBF_WARN("[LectCacheManager] Grid safe cache open failed: %s", grid_safe_path.c_str());
            // Non-fatal: grid cache is optional
        }
        if (!grid_unsafe_.open(grid_unsafe_path, grid_initial_cap, grid_max_cap)) {
            SBF_WARN("[LectCacheManager] Grid unsafe cache open failed: %s", grid_unsafe_path.c_str());
        }
    }

    SBF_INFO("[LectCacheManager] init: robot=%s hash=%s dir=%s" " EP safe=%d/%d unsafe=%d/%d stride=%d\n" " Grid %s safe=%d/%d unsafe=%d/%d\n", robot_name.c_str(), hash_str, cache_dir_.c_str(), ep_safe_.size(), ep_safe_.capacity(), ep_unsafe_.size(), ep_unsafe_.capacity(), ep_stride, grid_enabled_ ? "enabled" : "disabled", grid_safe_.size(), grid_safe_.capacity(), grid_unsafe_.size(), grid_unsafe_.capacity());

    start_async_worker();
    return true;
}

LectCacheManager::~LectCacheManager() {
    stop_async_worker();
}

void LectCacheManager::start_async_worker() {
    std::lock_guard<std::mutex> lock(async_mu_);
    async_stop_ = false;
    if (!async_worker_.joinable()) {
        async_worker_ = std::thread(&LectCacheManager::async_worker_loop, this);
    }
}

void LectCacheManager::stop_async_worker() {
    flush_async();
    {
        std::lock_guard<std::mutex> lock(async_mu_);
        async_stop_ = true;
    }
    async_cv_.notify_all();
    if (async_worker_.joinable()) {
        async_worker_.join();
    }
    {
        std::lock_guard<std::mutex> lock(async_mu_);
        async_stop_ = false;
    }
}

void LectCacheManager::flush_async() {
    std::unique_lock<std::mutex> lock(async_mu_);
    async_idle_cv_.wait(lock, [this]() {
        return async_queue_.empty() && async_inflight_ == 0;
    });
}

void LectCacheManager::enqueue_ep_insert(int channel, uint64_t key,
                                         EndpointSource source,
                                         const float* ep_data,
                                         const float* liaabb_data) {
    if (!ep_data || key == 0 || !ep_cache(channel).is_open()) return;
    AsyncTask task;
    task.type = AsyncTaskType::EpInsert;
    task.channel = channel;
    task.key = key;
    task.ep_source = source;
    task.ep_data.assign(ep_data, ep_data + ep_stride_);
    if (liaabb_data && liaabb_stride_ > 0) {
        task.liaabb_data.assign(liaabb_data, liaabb_data + liaabb_stride_);
    }
    {
        std::lock_guard<std::mutex> lock(async_mu_);
        async_queue_.push_back(std::move(task));
    }
    async_cv_.notify_one();
}

void LectCacheManager::enqueue_grid_insert(
    int channel, uint64_t key,
    std::shared_ptr<const voxel::SparseVoxelGrid> grid,
    const GridQuality& quality) {
    if (!grid || key == 0 || !grid_cache(channel).is_open()) return;
    AsyncTask task;
    task.type = AsyncTaskType::GridInsert;
    task.channel = channel;
    task.key = key;
    task.grid = std::move(grid);
    task.grid_quality = quality;
    {
        std::lock_guard<std::mutex> lock(async_mu_);
        async_queue_.push_back(std::move(task));
    }
    async_cv_.notify_one();
}

void LectCacheManager::async_worker_loop() {
    while (true) {
        AsyncTask task;
        {
            std::unique_lock<std::mutex> lock(async_mu_);
            async_cv_.wait(lock, [this]() {
                return async_stop_ || !async_queue_.empty();
            });
            if (async_stop_ && async_queue_.empty()) break;
            task = std::move(async_queue_.front());
            async_queue_.pop_front();
            ++async_inflight_;
        }

        if (task.type == AsyncTaskType::EpInsert) {
            const float* liaabb = task.liaabb_data.empty()
                ? nullptr
                : task.liaabb_data.data();
            ep_cache(task.channel).insert(
                task.key, task.ep_source, task.ep_data.data(), liaabb);
        } else {
            grid_cache(task.channel).insert(
                task.key, *task.grid, task.grid_quality);
        }

        {
            std::lock_guard<std::mutex> lock(async_mu_);
            --async_inflight_;
            if (async_queue_.empty() && async_inflight_ == 0) {
                async_idle_cv_.notify_all();
            }
        }
    }

    std::lock_guard<std::mutex> lock(async_mu_);
    if (async_queue_.empty() && async_inflight_ == 0) {
        async_idle_cv_.notify_all();
    }
}

// ─── Stats ──────────────────────────────────────────────────────────────────
void LectCacheManager::print_stats() const {
    SBF_INFO("[LectCacheManager] EP safe: %d/%d (%.1f%%) unsafe: %d/%d (%.1f%%)", ep_safe_.size(), ep_safe_.capacity(), ep_safe_.capacity() > 0 ? 100.0 * ep_safe_.size() / ep_safe_.capacity() : 0.0, ep_unsafe_.size(), ep_unsafe_.capacity(), ep_unsafe_.capacity() > 0 ? 100.0 * ep_unsafe_.size() / ep_unsafe_.capacity() : 0.0);
    if (!grid_enabled_) {
        SBF_INFO("[LectCacheManager] Grid cache disabled for non-grid envelope");
        return;
    }
    SBF_INFO("[LectCacheManager] Grid safe: %d/%d unsafe: %d/%d", grid_safe_.size(), grid_safe_.capacity(), grid_unsafe_.size(), grid_unsafe_.capacity());

    // LRU stats
    auto print_lru = [](const char* label, const Z4GridCache& gc) {
        int64_t hits = gc.mem_hits(), misses = gc.mem_misses();
        int64_t total = hits + misses;
        double rate = total > 0 ? 100.0 * hits / total : 0.0;
        SBF_INFO("[LectCacheManager] Grid %s LRU: %d entries %.1fMB " "hit=%ld miss=%ld (%.1f%%)", label, gc.mem_entries(), gc.mem_bytes() / (1024.0 * 1024.0), static_cast<long>(hits), static_cast<long>(misses), rate);
    };
    print_lru("safe",   grid_safe_);
    print_lru("unsafe", grid_unsafe_);
    auto print_grid_io = [](const char* label, const Z4GridCache& gc) {
        SBF_INFO("[LectCacheManager] Grid %s IO: pread=%ld/%.1fMB pwrite=%ld/%.1fMB grow=%ld dead=%.1fMB",
                 label,
                 static_cast<long>(gc.pread_calls()),
                 gc.pread_bytes() / (1024.0 * 1024.0),
                 static_cast<long>(gc.pwrite_calls()),
                 gc.pwrite_bytes() / (1024.0 * 1024.0),
                 static_cast<long>(gc.grow_calls()),
                 gc.dead_bytes() / (1024.0 * 1024.0));
    };
    print_grid_io("safe", grid_safe_);
    print_grid_io("unsafe", grid_unsafe_);
}

}  // namespace sbf
