/// @file z4_grid_cache.cpp
/// @brief Z4GridCache implementation — single-channel mmap-backed grid cache
///        with quality metadata per entry.

#include <sbf/lect/z4_grid_cache.h>
#include <sbf/voxel/bit_brick.h>

#include <cassert>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <shared_mutex>
#include <sbf/core/log.h>

namespace sbf {

using namespace voxel;

// Magic / version for quality-aware single-channel format
static constexpr char kMagic[8] = {'S','B','F','7','G','R','D','\0'};
static constexpr uint32_t kVersion = 2;

// ─── Destructor ─────────────────────────────────────────────────────────────
Z4GridCache::~Z4GridCache() { close(); }

// ─── Open / Create ──────────────────────────────────────────────────────────
bool Z4GridCache::open(const std::string& path,
                       int initial_capacity, int max_capacity,
                       size_t mem_cache_bytes) {
    close();
    path_ = path;
    max_capacity_ = max_capacity;
    lru_max_bytes_ = mem_cache_bytes;

    struct stat st;
    bool exists = (::stat(path.c_str(), &st) == 0 &&
                   st.st_size >= static_cast<off_t>(sizeof(Header)));

    if (exists) {
        fd_ = ::open(path.c_str(), O_RDWR);
        if (fd_ < 0) {
            SBF_WARN("[Z4GridCache] open(%s) failed: %s", path.c_str(), strerror(errno));
            return false;
        }

        Header hdr{};
        if (::pread(fd_, &hdr, sizeof(hdr), 0) != sizeof(hdr)) {
            ::close(fd_); fd_ = -1;
            exists = false;
        } else if (std::memcmp(hdr.magic, kMagic, 8) != 0 ||
                   hdr.version != kVersion) {
            SBF_INFO("[Z4GridCache] incompatible cache, recreating");
            ::close(fd_); fd_ = -1;
            exists = false;
        } else {
            file_size_ = static_cast<size_t>(st.st_size);
        }
    }

    if (!exists) {
        int cap = initial_capacity;
        if (cap < 64) cap = 64;
        int v = 64;
        while (v < cap) v <<= 1;
        cap = v;

        size_t index_bytes = static_cast<size_t>(cap) * sizeof(IndexSlot);
        size_t data_start = sizeof(Header) + index_bytes;
        size_t initial_data = 1024 * 1024;  // 1 MB headroom
        file_size_ = data_start + initial_data;

        fd_ = ::open(path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
        if (fd_ < 0) {
            SBF_WARN("[Z4GridCache] create(%s) failed: %s", path.c_str(), strerror(errno));
            return false;
        }
        if (::ftruncate(fd_, static_cast<off_t>(file_size_)) != 0) {
            ::close(fd_); fd_ = -1;
            return false;
        }

        mmap_size_ = sizeof(Header) + index_bytes;
        data_ = static_cast<uint8_t*>(
            ::mmap(nullptr, mmap_size_, PROT_READ | PROT_WRITE,
                   MAP_SHARED, fd_, 0));
        if (data_ == MAP_FAILED) {
            data_ = nullptr; ::close(fd_); fd_ = -1;
            return false;
        }

        Header* hdr = reinterpret_cast<Header*>(data_);
        std::memcpy(hdr->magic, kMagic, 8);
        hdr->version          = kVersion;
        hdr->index_capacity   = cap;
        hdr->index_size       = 0;
        hdr->reserved         = 0;
        hdr->best_resolution  = kBestResolution;
        hdr->data_section_off = data_start;
        hdr->data_used        = 0;
        std::memset(hdr->pad, 0, sizeof(hdr->pad));

        // Zero-init index slots
        std::memset(data_ + sizeof(Header), 0, index_bytes);

        ::msync(data_, mmap_size_, MS_ASYNC);
        return true;
    }

    // Only mmap header + index section (data section accessed via pread/pwrite)
    {
        Header hdr_tmp{};
        if (::pread(fd_, &hdr_tmp, sizeof(hdr_tmp), 0) != sizeof(hdr_tmp)) {
            ::close(fd_); fd_ = -1;
            return false;
        }
        mmap_size_ = sizeof(Header) +
                     static_cast<size_t>(hdr_tmp.index_capacity) * sizeof(IndexSlot);
    }
    data_ = static_cast<uint8_t*>(
        ::mmap(nullptr, mmap_size_, PROT_READ | PROT_WRITE,
               MAP_SHARED, fd_, 0));
    if (data_ == MAP_FAILED) {
        data_ = nullptr; ::close(fd_); fd_ = -1;
        return false;
    }

    return true;
}

// ─── Close ──────────────────────────────────────────────────────────────────
void Z4GridCache::close() {
    if (data_) {
        ::msync(data_, mmap_size_, MS_SYNC);
        ::munmap(data_, mmap_size_);
        data_ = nullptr;
    }
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    file_size_ = 0;
    mmap_size_ = 0;
    // Clear LRU
    lru_list_.clear();
    lru_map_.clear();
    lru_bytes_ = 0;
}

// ─── Stats ──────────────────────────────────────────────────────────────────
int Z4GridCache::capacity() const {
    if (!data_) return 0;
    return reinterpret_cast<const Header*>(data_)->index_capacity;
}
int Z4GridCache::size() const {
    if (!data_) return 0;
    return reinterpret_cast<const Header*>(data_)->index_size;
}

// ─── find_index_slot (linear probing) ───────────────────────────────────────
int Z4GridCache::find_index_slot(uint64_t key) const {
    const int cap = capacity();
    const int mask = cap - 1;
    int idx = static_cast<int>(key & static_cast<uint64_t>(mask));
    for (int i = 0; i < cap; ++i) {
        IndexSlot* s = index_slot(idx);
        if (s->key == kEmptyKey || s->key == key) return idx;
        idx = (idx + 1) & mask;
    }
    return -1;
}

// ─── Lookup (with quality check) ────────────────────────────────────────────
std::unique_ptr<SparseVoxelGrid> Z4GridCache::lookup(
        uint64_t z4_key, const GridQuality& req) const {
    if (!data_ || z4_key == kEmptyKey) return nullptr;

    // ── Phase 1: LRU hit (shared lock) ──────────────────────────────────
    if (lru_max_bytes_ > 0) {
        // Try read-only check first with shared lock
        std::shared_lock<std::shared_mutex> rlock(mu_);
        auto it = lru_map_.find(z4_key);
        if (it != lru_map_.end()) {
            // Copy grid while holding shared lock
            auto grid = std::make_unique<SparseVoxelGrid>(it->second->grid);
            mem_hits_.fetch_add(1, std::memory_order_relaxed);
            // Promote to front requires unique lock — defer (amortized cost ok)
            return grid;
        }
    }

    // ── Phase 2: Disk lookup (shared lock for index, then upgrade) ──────
    mem_misses_.fetch_add(1, std::memory_order_relaxed);

    // Read from disk under shared lock
    std::unique_ptr<SparseVoxelGrid> grid;
    {
        std::shared_lock<std::shared_mutex> rlock(mu_);

        int idx = find_index_slot(z4_key);
        if (idx < 0) return nullptr;

        IndexSlot* s = index_slot(idx);
        if (s->key != z4_key) return nullptr;

        GridQuality cached_q = slot_quality(s);
        if (!cached_q.satisfies(req)) return nullptr;

        const Header* hdr = reinterpret_cast<const Header*>(data_);
        int n_bricks = static_cast<int>(s->n_bricks);

        size_t brick_bytes = static_cast<size_t>(n_bricks) * kBrickRecordBytes;
        std::vector<uint8_t> brick_buf(brick_bytes);
        if (::pread(fd_, brick_buf.data(), brick_bytes,
                    hdr->data_section_off + s->data_offset)
                != static_cast<ssize_t>(brick_bytes)) {
            return nullptr;
        }
        const uint8_t* brick_data = brick_buf.data();

        grid = std::make_unique<SparseVoxelGrid>(
            static_cast<double>(cached_q.resolution));

        for (int i = 0; i < n_bricks; ++i) {
            const uint8_t* rec = brick_data +
                                 static_cast<size_t>(i) * kBrickRecordBytes;
            BrickCoord bc;
            std::memcpy(&bc.bx, rec + 0, 4);
            std::memcpy(&bc.by, rec + 4, 4);
            std::memcpy(&bc.bz, rec + 8, 4);

            BitBrick bb;
            std::memcpy(bb.words, rec + 12, 64);
            grid->set_brick(bc, bb);
        }
    }  // release shared lock

    // ── Phase 3: Populate LRU (unique lock) ─────────────────────────────
    if (grid && lru_max_bytes_ > 0) {
        std::unique_lock<std::shared_mutex> wlock(mu_);
        // Re-check: another thread may have inserted it
        if (lru_map_.find(z4_key) == lru_map_.end()) {
            lru_put(z4_key, *grid);
        }
    }

    return grid;
}

bool Z4GridCache::contains(uint64_t z4_key, const GridQuality& req) const {
    if (!data_ || z4_key == kEmptyKey) return false;
    std::shared_lock<std::shared_mutex> lock(mu_);
    if (lru_max_bytes_ > 0 && lru_map_.count(z4_key)) return true;
    int idx = find_index_slot(z4_key);
    if (idx < 0) return false;
    IndexSlot* s = index_slot(idx);
    if (s->key != z4_key) return false;
    return slot_quality(s).satisfies(req);
}

// ─── remap_file ─────────────────────────────────────────────────────────────
void Z4GridCache::remap_file(size_t new_file_size, size_t new_mmap_size) {
    if (data_) {
        ::msync(data_, mmap_size_, MS_SYNC);
        ::munmap(data_, mmap_size_);
        data_ = nullptr;
    }
    if (::ftruncate(fd_, static_cast<off_t>(new_file_size)) != 0) {
        SBF_WARN("[Z4GridCache] ftruncate(%zu) failed: %s", new_file_size, strerror(errno));
        return;
    }
    data_ = static_cast<uint8_t*>(
        ::mmap(nullptr, new_mmap_size, PROT_READ | PROT_WRITE,
               MAP_SHARED, fd_, 0));
    if (data_ == MAP_FAILED) {
        data_ = nullptr;
        SBF_WARN("[Z4GridCache] remap mmap failed");
        return;
    }
    file_size_ = new_file_size;
    mmap_size_ = new_mmap_size;
}

// ─── Insert / Update (with quality metadata) ────────────────────────────────
void Z4GridCache::insert(uint64_t z4_key,
                         const SparseVoxelGrid& grid,
                         const GridQuality& quality) {
    if (!data_ || z4_key == kEmptyKey) return;

    std::unique_lock<std::shared_mutex> lock(mu_);

    Header* hdr = reinterpret_cast<Header*>(data_);

    // Check load factor (applies only for new keys)
    bool need_grow = false;
    {
        int idx = find_index_slot(z4_key);
        if (idx >= 0 && index_slot(idx)->key == z4_key) {
            // Key already exists — check if upgrade needed
            GridQuality cached_q = slot_quality(index_slot(idx));
            if (cached_q.satisfies(quality)) return;  // already good enough
            // Fall through to overwrite (update in place)
        } else {
            // New key — may need grow
            if (static_cast<double>(hdr->index_size + 1) >
                hdr->index_capacity * kMaxLoadFactor) {
                if (max_capacity_ > 0 &&
                    hdr->index_capacity >= max_capacity_) return;
                need_grow = true;
            }
        }
    }

    if (need_grow) {
        grow_index();
        hdr = reinterpret_cast<Header*>(data_);
    }

    // Count bricks and compute bytes
    int n_bricks = grid.num_bricks();
    size_t brick_bytes = static_cast<size_t>(n_bricks) * kBrickRecordBytes;

    // Ensure data section has enough space (grow file without remapping mmap)
    size_t needed = hdr->data_section_off + hdr->data_used + brick_bytes;
    if (needed > file_size_) {
        size_t new_size = std::max(needed + 1024 * 1024,
                                   file_size_ + file_size_ / 2);
        if (::ftruncate(fd_, static_cast<off_t>(new_size)) != 0) {
            SBF_WARN("[Z4GridCache] ftruncate(%zu) failed: %s", new_size, strerror(errno));
            return;
        }
        file_size_ = new_size;
    }

    // Append brick data via pwrite (data section not mmap'd)
    uint64_t data_offset = hdr->data_used;
    std::vector<uint8_t> brick_buf(brick_bytes);
    uint8_t* brick_dst = brick_buf.data();

    int bi = 0;
    for (auto it = grid.bricks().begin(); it != grid.bricks().end(); ++it) {
        uint8_t* rec = brick_dst +
                       static_cast<size_t>(bi) * kBrickRecordBytes;
        auto entry = *it;
        const BrickCoord& bc = entry.key;
        const BitBrick& bb = entry.value;
        std::memcpy(rec + 0,  &bc.bx, 4);
        std::memcpy(rec + 4,  &bc.by, 4);
        std::memcpy(rec + 8,  &bc.bz, 4);
        std::memcpy(rec + 12, bb.words, 64);
        bi++;
    }

    ::pwrite(fd_, brick_buf.data(), brick_bytes,
             hdr->data_section_off + data_offset);
    hdr->data_used += brick_bytes;

    // Write / update index entry
    int idx = find_index_slot(z4_key);
    if (idx < 0) return;

    IndexSlot* s = index_slot(idx);
    bool is_new = (s->key == kEmptyKey);

    s->key           = z4_key;
    s->data_offset   = data_offset;
    s->n_bricks      = static_cast<uint32_t>(n_bricks);
    s->envelope_type = static_cast<uint8_t>(quality.type);
    s->n_sub         = static_cast<uint8_t>(quality.n_sub);
    s->reserved      = 0;
    s->resolution    = quality.resolution;
    s->pad           = 0;

    if (is_new) {
        hdr->index_size++;
    }
    // Note: when updating an existing entry, old brick data becomes dead space.
    // This is acceptable for mmap-backed append-only layout.

    // Populate LRU cache
    if (lru_max_bytes_ > 0) {
        lru_put(z4_key, grid);
    }
}

// ─── Grow index (rehash) ────────────────────────────────────────────────────
void Z4GridCache::grow_index() {
    Header* hdr = reinterpret_cast<Header*>(data_);
    int old_cap = hdr->index_capacity;
    int new_cap = old_cap * 2;

    // Respect max_capacity
    if (max_capacity_ > 0 && new_cap > max_capacity_)
        new_cap = max_capacity_;
    if (new_cap <= old_cap) return;

    size_t new_index_bytes = static_cast<size_t>(new_cap) * sizeof(IndexSlot);

    size_t old_data_off = hdr->data_section_off;
    size_t old_data_used = hdr->data_used;
    size_t new_data_off = sizeof(Header) + new_index_bytes;

    size_t new_file_size = new_data_off + old_data_used + 1024 * 1024;

    // Snapshot old index entries (small: ~32B per entry)
    std::vector<IndexSlot> old_entries;
    old_entries.reserve(hdr->index_size);
    for (int i = 0; i < old_cap; ++i) {
        IndexSlot* s = index_slot(i);
        if (s->key != kEmptyKey) {
            old_entries.push_back(*s);
        }
    }

    // Extend file BEFORE moving data (so dest range exists on disk)
    if (::ftruncate(fd_, static_cast<off_t>(new_file_size)) != 0) {
        SBF_WARN("[Z4GridCache] grow_index ftruncate(%zu) failed: %s", new_file_size, strerror(errno));
        return;
    }
    file_size_ = new_file_size;

    // Move data section with chunked reverse copy (64MB chunks).
    // new_data_off > old_data_off, so regions may overlap — reverse copy
    // ensures we never overwrite unread source data.
    if (old_data_used > 0) {
        constexpr size_t kChunkSize = 64 * 1024 * 1024;  // 64MB
        std::vector<uint8_t> chunk_buf(std::min(kChunkSize, old_data_used));

        size_t remaining = old_data_used;
        while (remaining > 0) {
            size_t chunk = std::min(remaining, kChunkSize);
            size_t off_in_data = remaining - chunk;

            ssize_t rd = ::pread(fd_, chunk_buf.data(), chunk,
                                 old_data_off + off_in_data);
            if (rd != static_cast<ssize_t>(chunk)) {
                SBF_WARN("[Z4GridCache] grow_index pread failed at offset %zu", off_in_data);
                return;
            }
            ::pwrite(fd_, chunk_buf.data(), chunk,
                     new_data_off + off_in_data);
            remaining -= chunk;
        }
    }

    // Remap header + new (larger) index section
    if (data_) {
        ::msync(data_, mmap_size_, MS_SYNC);
        ::munmap(data_, mmap_size_);
        data_ = nullptr;
    }
    size_t new_mmap_size = sizeof(Header) + new_index_bytes;
    data_ = static_cast<uint8_t*>(
        ::mmap(nullptr, new_mmap_size, PROT_READ | PROT_WRITE,
               MAP_SHARED, fd_, 0));
    if (data_ == MAP_FAILED) {
        data_ = nullptr;
        SBF_WARN("[Z4GridCache] grow_index mmap failed");
        return;
    }
    mmap_size_ = new_mmap_size;
    hdr = reinterpret_cast<Header*>(data_);

    // Clear index section
    std::memset(data_ + sizeof(Header), 0, new_index_bytes);

    // Update header
    hdr->index_capacity = new_cap;
    hdr->data_section_off = new_data_off;

    // Rehash
    hdr->index_size = 0;
    int new_mask = new_cap - 1;
    for (const auto& entry : old_entries) {
        int idx = static_cast<int>(entry.key & static_cast<uint64_t>(new_mask));
        for (;;) {
            IndexSlot* s = index_slot(idx);
            if (s->key == kEmptyKey) {
                *s = entry;
                hdr->index_size++;
                break;
            }
            idx = (idx + 1) & new_mask;
        }
    }

    SBF_INFO("[Z4GridCache] grow_index: %d → %d slots (%d entries, data=%zuMB chunked)", old_cap, new_cap, hdr->index_size, old_data_used / (1024*1024));
}

// ─── LRU helpers ────────────────────────────────────────────────────────────

size_t Z4GridCache::estimate_grid_bytes(const SparseVoxelGrid& g) {
    // FlatBrickMap stores vector<Slot> where Slot ≈ 80 bytes.
    // Approximate: num_bricks / 0.7 (load factor) * sizeof(Slot) + overhead.
    int nb = g.num_bricks();
    int cap = 16;
    while (cap * 7 < nb * 10) cap *= 2;  // match FlatBrickMap growth
    return static_cast<size_t>(cap) * 80 + 128;  // 80B/slot + object overhead
}

void Z4GridCache::lru_evict() const {
    while (lru_bytes_ > lru_max_bytes_ && !lru_list_.empty()) {
        auto& victim = lru_list_.back();
        lru_bytes_ -= victim.byte_size;
        lru_map_.erase(victim.key);
        lru_list_.pop_back();
    }
}

void Z4GridCache::lru_put(uint64_t key, const SparseVoxelGrid& grid) const {
    // If already present, move to front and update
    auto it = lru_map_.find(key);
    if (it != lru_map_.end()) {
        lru_bytes_ -= it->second->byte_size;
        lru_list_.erase(it->second);
        lru_map_.erase(it);
    }

    size_t cost = estimate_grid_bytes(grid);
    lru_list_.emplace_front(LRUEntry{key, grid, cost});
    lru_map_[key] = lru_list_.begin();
    lru_bytes_ += cost;

    lru_evict();
}

int Z4GridCache::mem_entries() const {
    std::shared_lock<std::shared_mutex> lock(mu_);
    return static_cast<int>(lru_map_.size());
}

size_t Z4GridCache::mem_bytes() const {
    std::shared_lock<std::shared_mutex> lock(mu_);
    return lru_bytes_;
}

}  // namespace sbf
