/// @file lect_cache_manager.cpp
/// @brief LectCacheManager implementation — per-channel EP + Grid caches.

#include <sbf/lect/lect_cache_manager.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sys/stat.h>
#include <sys/types.h>

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
                            int ep_stride, const std::string& cache_dir,
                            int ep_max_cap, int grid_max_cap) {
    ep_stride_ = ep_stride;

    // Determine cache directory
    if (cache_dir.empty()) {
        const char* home = std::getenv("HOME");
        if (!home) home = "/tmp";
        cache_dir_ = std::string(home) + "/.sbf_cache";
    } else {
        cache_dir_ = cache_dir;
    }

    // Robot-specific subdirectory
    char hash_str[32];
    std::snprintf(hash_str, sizeof(hash_str), "%016llx",
                  static_cast<unsigned long long>(robot_hash));
    cache_dir_ = cache_dir_ + "/" + hash_str;

    if (!ensure_dir(cache_dir_)) {
        fprintf(stderr, "[LectCacheManager] mkdir(%s) failed: %s\n",
                cache_dir_.c_str(), strerror(errno));
        return false;
    }

    // Open 2 EP caches (safe / unsafe)
    std::string ep_safe_path   = cache_dir_ + "/ep_safe.cache";
    std::string ep_unsafe_path = cache_dir_ + "/ep_unsafe.cache";

    if (!ep_safe_.open(ep_safe_path, ep_stride, 4096, ep_max_cap)) {
        fprintf(stderr, "[LectCacheManager] EP safe cache open failed: %s\n",
                ep_safe_path.c_str());
        return false;
    }
    if (!ep_unsafe_.open(ep_unsafe_path, ep_stride, 4096, ep_max_cap)) {
        fprintf(stderr, "[LectCacheManager] EP unsafe cache open failed: %s\n",
                ep_unsafe_path.c_str());
        return false;
    }

    // Open 2 Grid caches (safe / unsafe)
    std::string grid_safe_path   = cache_dir_ + "/grid_safe.cache";
    std::string grid_unsafe_path = cache_dir_ + "/grid_unsafe.cache";

    if (!grid_safe_.open(grid_safe_path, 4096, grid_max_cap)) {
        fprintf(stderr, "[LectCacheManager] Grid safe cache open failed: %s\n",
                grid_safe_path.c_str());
        // Non-fatal: grid cache is optional
    }
    if (!grid_unsafe_.open(grid_unsafe_path, 4096, grid_max_cap)) {
        fprintf(stderr, "[LectCacheManager] Grid unsafe cache open failed: %s\n",
                grid_unsafe_path.c_str());
    }

    fprintf(stderr, "[LectCacheManager] init: robot=%s hash=%s dir=%s\n"
                    "  EP  safe=%d/%d  unsafe=%d/%d  stride=%d\n"
                    "  Grid safe=%d/%d  unsafe=%d/%d\n",
            robot_name.c_str(), hash_str, cache_dir_.c_str(),
            ep_safe_.size(), ep_safe_.capacity(),
            ep_unsafe_.size(), ep_unsafe_.capacity(), ep_stride,
            grid_safe_.size(), grid_safe_.capacity(),
            grid_unsafe_.size(), grid_unsafe_.capacity());

    return true;
}

// ─── Stats ──────────────────────────────────────────────────────────────────
void LectCacheManager::print_stats() const {
    fprintf(stderr, "[LectCacheManager] EP  safe: %d/%d (%.1f%%)  unsafe: %d/%d (%.1f%%)\n",
            ep_safe_.size(), ep_safe_.capacity(),
            ep_safe_.capacity() > 0 ? 100.0 * ep_safe_.size() / ep_safe_.capacity() : 0.0,
            ep_unsafe_.size(), ep_unsafe_.capacity(),
            ep_unsafe_.capacity() > 0 ? 100.0 * ep_unsafe_.size() / ep_unsafe_.capacity() : 0.0);
    fprintf(stderr, "[LectCacheManager] Grid safe: %d/%d  unsafe: %d/%d\n",
            grid_safe_.size(), grid_safe_.capacity(),
            grid_unsafe_.size(), grid_unsafe_.capacity());

    // LRU stats
    auto print_lru = [](const char* label, const Z4GridCache& gc) {
        int64_t hits = gc.mem_hits(), misses = gc.mem_misses();
        int64_t total = hits + misses;
        double rate = total > 0 ? 100.0 * hits / total : 0.0;
        fprintf(stderr, "[LectCacheManager] Grid %s LRU: %d entries %.1fMB  "
                        "hit=%ld miss=%ld (%.1f%%)\n",
                label, gc.mem_entries(),
                gc.mem_bytes() / (1024.0 * 1024.0),
                static_cast<long>(hits), static_cast<long>(misses), rate);
    };
    print_lru("safe",   grid_safe_);
    print_lru("unsafe", grid_unsafe_);
}

}  // namespace sbf
