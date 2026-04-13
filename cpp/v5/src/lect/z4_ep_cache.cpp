/// @file z4_ep_cache.cpp
/// @brief Z4EpCache implementation — single-channel mmap-backed EP hash table.

#include <sbf/lect/z4_ep_cache.h>

#include <cassert>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace sbf {

// Magic / version for single-channel format
static constexpr char kMagic[8] = {'S','B','F','7','E','P','\0','\0'};
static constexpr uint32_t kVersion = 2;

// ─── Destructor ─────────────────────────────────────────────────────────────
Z4EpCache::~Z4EpCache() { close(); }

// ─── Open / Create ──────────────────────────────────────────────────────────
bool Z4EpCache::open(const std::string& path, int ep_stride,
                     int initial_capacity, int max_capacity) {
    close();
    path_ = path;
    ep_stride_ = ep_stride;
    max_capacity_ = max_capacity;

    struct stat st;
    bool exists = (::stat(path.c_str(), &st) == 0 &&
                   st.st_size >= static_cast<off_t>(sizeof(Header)));

    if (exists) {
        fd_ = ::open(path.c_str(), O_RDWR);
        if (fd_ < 0) {
            fprintf(stderr, "[Z4EpCache] open(%s) failed: %s\n",
                    path.c_str(), strerror(errno));
            return false;
        }

        Header hdr{};
        if (::pread(fd_, &hdr, sizeof(hdr), 0) != sizeof(hdr)) {
            fprintf(stderr, "[Z4EpCache] read header failed\n");
            ::close(fd_); fd_ = -1;
            return false;
        }

        if (std::memcmp(hdr.magic, kMagic, 8) != 0 ||
            hdr.version != kVersion || hdr.ep_stride != ep_stride) {
            fprintf(stderr, "[Z4EpCache] incompatible cache "
                    "(magic/version/stride mismatch), recreating\n");
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

        file_size_ = sizeof(Header) +
                     static_cast<size_t>(cap) * slot_bytes();

        fd_ = ::open(path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
        if (fd_ < 0) {
            fprintf(stderr, "[Z4EpCache] create(%s) failed: %s\n",
                    path.c_str(), strerror(errno));
            return false;
        }
        if (::ftruncate(fd_, static_cast<off_t>(file_size_)) != 0) {
            fprintf(stderr, "[Z4EpCache] ftruncate failed: %s\n",
                    strerror(errno));
            ::close(fd_); fd_ = -1;
            return false;
        }

        data_ = static_cast<uint8_t*>(
            ::mmap(nullptr, file_size_, PROT_READ | PROT_WRITE,
                   MAP_SHARED, fd_, 0));
        if (data_ == MAP_FAILED) {
            fprintf(stderr, "[Z4EpCache] mmap failed: %s\n",
                    strerror(errno));
            data_ = nullptr; ::close(fd_); fd_ = -1;
            return false;
        }

        Header* hdr = reinterpret_cast<Header*>(data_);
        std::memcpy(hdr->magic, kMagic, 8);
        hdr->version    = kVersion;
        hdr->ep_stride  = ep_stride;
        hdr->capacity   = cap;
        hdr->size       = 0;
        std::memset(hdr->pad, 0, sizeof(hdr->pad));

        std::memset(data_ + sizeof(Header), 0,
                    static_cast<size_t>(cap) * slot_bytes());

        ::msync(data_, file_size_, MS_ASYNC);
        return true;
    }

    // mmap existing file
    data_ = static_cast<uint8_t*>(
        ::mmap(nullptr, file_size_, PROT_READ | PROT_WRITE,
               MAP_SHARED, fd_, 0));
    if (data_ == MAP_FAILED) {
        fprintf(stderr, "[Z4EpCache] mmap failed: %s\n", strerror(errno));
        data_ = nullptr; ::close(fd_); fd_ = -1;
        return false;
    }

    return true;
}

// ─── Close ──────────────────────────────────────────────────────────────────
void Z4EpCache::close() {
    if (data_) {
        ::msync(data_, file_size_, MS_SYNC);
        ::munmap(data_, file_size_);
        data_ = nullptr;
    }
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    file_size_ = 0;
}

// ─── Stats ──────────────────────────────────────────────────────────────────
int Z4EpCache::capacity() const {
    if (!data_) return 0;
    return reinterpret_cast<const Header*>(data_)->capacity;
}
int Z4EpCache::size() const {
    if (!data_) return 0;
    return reinterpret_cast<const Header*>(data_)->size;
}

// ─── find_slot (linear probing) ─────────────────────────────────────────────
int Z4EpCache::find_slot(uint64_t key) const {
    const int cap = capacity();
    const int mask = cap - 1;
    int idx = static_cast<int>(key & static_cast<uint64_t>(mask));
    for (int i = 0; i < cap; ++i) {
        uint8_t* s = slot_ptr(idx);
        uint64_t sk = slot_key(s);
        if (sk == kEmptyKey || sk == key) return idx;
        idx = (idx + 1) & mask;
    }
    return -1;
}

// ─── Lookup (single-channel) ────────────────────────────────────────────────
const float* Z4EpCache::lookup(uint64_t z4_key,
                               EndpointSource requested_source) const {
    if (!data_ || z4_key == kEmptyKey) return nullptr;

    std::shared_lock<std::shared_mutex> lock(mu_);

    int idx = find_slot(z4_key);
    if (idx < 0) return nullptr;

    uint8_t* s = slot_ptr(idx);
    if (slot_key(s) != z4_key) return nullptr;

    EndpointSource cached = static_cast<EndpointSource>(slot_source(s));
    if (cached != requested_source &&
        !source_can_serve(cached, requested_source))
        return nullptr;

    return slot_ep(s);
}

bool Z4EpCache::contains(uint64_t z4_key,
                         EndpointSource requested_source) const {
    return lookup(z4_key, requested_source) != nullptr;
}

// ─── Lookup + Copy ──────────────────────────────────────────────────────────
bool Z4EpCache::lookup_copy(uint64_t z4_key,
                            EndpointSource requested_source,
                            float* out) const {
    if (!data_ || z4_key == kEmptyKey || !out) return false;

    std::shared_lock<std::shared_mutex> lock(mu_);

    int idx = find_slot(z4_key);
    if (idx < 0) return false;

    uint8_t* s = slot_ptr(idx);
    if (slot_key(s) != z4_key) return false;

    EndpointSource cached = static_cast<EndpointSource>(slot_source(s));
    if (cached != requested_source &&
        !source_can_serve(cached, requested_source))
        return false;

    std::memcpy(out, slot_ep(s),
                static_cast<size_t>(ep_stride_) * sizeof(float));
    return true;
}

// ─── Insert (single-channel) ────────────────────────────────────────────────
void Z4EpCache::insert(uint64_t z4_key,
                       EndpointSource source, const float* ep) {
    if (!data_ || z4_key == kEmptyKey || !ep) return;

    std::unique_lock<std::shared_mutex> lock(mu_);

    Header* hdr = reinterpret_cast<Header*>(data_);

    // Check load factor
    if (static_cast<double>(hdr->size + 1) >
        hdr->capacity * kMaxLoadFactor) {
        // Respect max_capacity: silently drop if limit reached
        if (max_capacity_ > 0 && hdr->capacity >= max_capacity_) return;
        grow();
        hdr = reinterpret_cast<Header*>(data_);
    }

    int idx = find_slot(z4_key);
    if (idx < 0) return;

    uint8_t* s = slot_ptr(idx);
    bool is_new = (slot_key(s) == kEmptyKey);

    slot_key(s) = z4_key;
    slot_source(s) = static_cast<uint8_t>(source);
    std::memcpy(slot_ep(s), ep,
                static_cast<size_t>(ep_stride_) * sizeof(float));

    if (is_new) {
        hdr->size++;
    }
}

// ─── Grow (rehash) ──────────────────────────────────────────────────────────
void Z4EpCache::grow() {
    Header* old_hdr = reinterpret_cast<Header*>(data_);
    int old_cap = old_hdr->capacity;
    int new_cap = old_cap * 2;

    // Respect max_capacity
    if (max_capacity_ > 0 && new_cap > max_capacity_)
        new_cap = max_capacity_;
    if (new_cap <= old_cap) return;  // cannot grow further

    size_t new_file_size = sizeof(Header) +
                           static_cast<size_t>(new_cap) * slot_bytes();

    std::string tmp_path = path_ + ".tmp";
    int new_fd = ::open(tmp_path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (new_fd < 0) {
        fprintf(stderr, "[Z4EpCache] grow: create tmp failed: %s\n",
                strerror(errno));
        return;
    }
    if (::ftruncate(new_fd, static_cast<off_t>(new_file_size)) != 0) {
        fprintf(stderr, "[Z4EpCache] grow: ftruncate failed\n");
        ::close(new_fd);
        return;
    }

    uint8_t* new_data = static_cast<uint8_t*>(
        ::mmap(nullptr, new_file_size, PROT_READ | PROT_WRITE,
               MAP_SHARED, new_fd, 0));
    if (new_data == MAP_FAILED) {
        fprintf(stderr, "[Z4EpCache] grow: mmap failed\n");
        ::close(new_fd);
        return;
    }

    // Initialize new header
    Header* new_hdr = reinterpret_cast<Header*>(new_data);
    std::memcpy(new_hdr->magic, kMagic, 8);
    new_hdr->version    = kVersion;
    new_hdr->ep_stride  = ep_stride_;
    new_hdr->capacity   = new_cap;
    new_hdr->size       = 0;
    std::memset(new_hdr->pad, 0, sizeof(new_hdr->pad));

    std::memset(new_data + sizeof(Header), 0,
                static_cast<size_t>(new_cap) * slot_bytes());

    // Rehash all occupied slots
    const int sb = slot_bytes();
    const int new_mask = new_cap - 1;
    int rehashed = 0;
    for (int i = 0; i < old_cap; ++i) {
        uint8_t* old_slot = data_ + sizeof(Header) +
                            static_cast<size_t>(i) * sb;
        uint64_t k = *reinterpret_cast<uint64_t*>(old_slot);
        if (k == kEmptyKey) continue;

        int idx = static_cast<int>(k & static_cast<uint64_t>(new_mask));
        for (;;) {
            uint8_t* ns = new_data + sizeof(Header) +
                          static_cast<size_t>(idx) * sb;
            if (*reinterpret_cast<uint64_t*>(ns) == kEmptyKey) {
                std::memcpy(ns, old_slot, sb);
                break;
            }
            idx = (idx + 1) & new_mask;
        }
        rehashed++;
    }
    new_hdr->size = rehashed;

    ::munmap(data_, file_size_);
    ::close(fd_);
    ::msync(new_data, new_file_size, MS_SYNC);
    ::rename(tmp_path.c_str(), path_.c_str());

    data_ = new_data;
    file_size_ = new_file_size;
    fd_ = new_fd;

    fprintf(stderr, "[Z4EpCache] grow: %d → %d slots (%d entries)\n",
            old_cap, new_cap, rehashed);
}

}  // namespace sbf
