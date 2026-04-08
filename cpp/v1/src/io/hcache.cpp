// SafeBoxForest — HCACHE02 binary format I/O (mmap-based)
#include "sbf/io/hcache.h"
#include <cstring>
#include <stdexcept>
#include <algorithm>

#if defined(__linux__) || defined(__APPLE__)
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

namespace sbf {

// ─── Destructor ──────────────────────────────────────────────────────────────
HCacheFile::~HCacheFile() { close(); }

// ─── Read header fields from mmap ────────────────────────────────────────────
void HCacheFile::read_header() {
    if (!mapped_) return;
    // Verify magic
    if (std::memcmp(mapped_, hcache::MAGIC, 8) != 0)
        throw std::runtime_error("HCACHE: invalid magic");

    int32_t ver;
    std::memcpy(&ver, mapped_ + 8, 4);
    if (ver != hcache::VERSION)
        throw std::runtime_error("HCACHE: unsupported version");

    std::memcpy(&n_nodes_,    mapped_ + 12, 8);
    std::memcpy(&n_alloc_,    mapped_ + 20, 8);
    std::memcpy(&n_dims_,     mapped_ + 28, 4);
    std::memcpy(&n_links_,    mapped_ + 32, 4);
    std::memcpy(&n_fk_calls_, mapped_ + 36, 8);
    std::memcpy(&stride_,     mapped_ + 44, 4);
}

// ─── Write header fields to mmap ─────────────────────────────────────────────
void HCacheFile::write_header() {
    if (!mapped_) return;
    std::memcpy(mapped_, hcache::MAGIC, 8);
    int32_t ver = hcache::VERSION;
    std::memcpy(mapped_ + 8,  &ver, 4);
    std::memcpy(mapped_ + 12, &n_nodes_, 8);
    std::memcpy(mapped_ + 20, &n_alloc_, 8);
    std::memcpy(mapped_ + 28, &n_dims_, 4);
    std::memcpy(mapped_ + 32, &n_links_, 4);
    std::memcpy(mapped_ + 36, &n_fk_calls_, 8);
    std::memcpy(mapped_ + 44, &stride_, 4);
}

// ─── Grow underlying file and remap ──────────────────────────────────────────
void HCacheFile::grow_file(int new_cap) {
#if defined(__linux__) || defined(__APPLE__)
    if (!mapped_ || fd_ < 0) return;

    // Compute new file size
    size_t data_size = static_cast<size_t>(new_cap) * stride_;
    size_t new_size = hcache::HEADER_SIZE + data_size;

    // Unmap, resize, remap
    munmap(mapped_, file_size_);
    if (ftruncate(fd_, static_cast<off_t>(new_size)) < 0)
        throw std::runtime_error("HCACHE: ftruncate failed");

    mapped_ = static_cast<char*>(
        mmap(nullptr, new_size, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0));
    if (mapped_ == MAP_FAILED) {
        mapped_ = nullptr;
        throw std::runtime_error("HCACHE: mmap failed on grow");
    }
    file_size_ = new_size;
    n_alloc_ = new_cap;
    write_header();

    // Re-attach NodeStore to the new mapping (preserves auxiliary data)
    char* data_base = mapped_ + hcache::HEADER_SIZE;
    store_.attach_buffer(data_base, new_cap);
#else
    (void)new_cap;
    throw std::runtime_error("HCACHE: grow not supported on this platform");
#endif
}

// ─── Open existing HCACHE file ───────────────────────────────────────────────
HCacheFile HCacheFile::open(const std::string& path) {
    HCacheFile hf;
#if defined(__linux__) || defined(__APPLE__)
    hf.fd_ = ::open(path.c_str(), O_RDWR);
    if (hf.fd_ < 0)
        throw std::runtime_error("HCACHE: cannot open " + path);

    struct stat st;
    if (fstat(hf.fd_, &st) < 0) {
        ::close(hf.fd_);
        throw std::runtime_error("HCACHE: fstat failed");
    }
    hf.file_size_ = static_cast<size_t>(st.st_size);

    if (hf.file_size_ < static_cast<size_t>(hcache::HEADER_SIZE)) {
        ::close(hf.fd_);
        throw std::runtime_error("HCACHE: file too small");
    }

    hf.mapped_ = static_cast<char*>(
        mmap(nullptr, hf.file_size_, PROT_READ | PROT_WRITE, MAP_SHARED, hf.fd_, 0));
    if (hf.mapped_ == MAP_FAILED) {
        hf.mapped_ = nullptr;
        ::close(hf.fd_);
        throw std::runtime_error("HCACHE: mmap failed");
    }

    hf.read_header();

    // Set up NodeStore backed by mmap data region
    char* data_base = hf.mapped_ + hcache::HEADER_SIZE;
    hf.store_ = NodeStore(data_base, hf.n_links_, hf.n_dims_,
                           static_cast<int>(hf.n_alloc_),
                           static_cast<int>(hf.n_nodes_));

    // Hook resize callback so NodeStore::ensure_capacity triggers grow_file
    hf.setup_resize_callback();

    return hf;
#else
    (void)path;
    throw std::runtime_error("HCACHE: mmap not supported on this platform");
#endif
}

// ─── Create new HCACHE file ──────────────────────────────────────────────────
HCacheFile HCacheFile::create(const std::string& path,
                               int n_dims, int n_links,
                               const JointLimits& limits,
                               const std::string& fingerprint,
                               int initial_cap) {
    HCacheFile hf;
#if defined(__linux__) || defined(__APPLE__)
    hf.n_dims_ = n_dims;
    hf.n_links_ = n_links;
    hf.n_nodes_ = 0;
    hf.n_alloc_ = initial_cap;
    hf.n_fk_calls_ = 0;
    hf.stride_ = node_layout::compute_stride(n_links);

    size_t data_size = static_cast<size_t>(initial_cap) * hf.stride_;
    hf.file_size_ = hcache::HEADER_SIZE + data_size;

    hf.fd_ = ::open(path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
    if (hf.fd_ < 0)
        throw std::runtime_error("HCACHE: cannot create " + path);

    if (ftruncate(hf.fd_, static_cast<off_t>(hf.file_size_)) < 0) {
        ::close(hf.fd_);
        throw std::runtime_error("HCACHE: ftruncate failed");
    }

    hf.mapped_ = static_cast<char*>(
        mmap(nullptr, hf.file_size_, PROT_READ | PROT_WRITE, MAP_SHARED, hf.fd_, 0));
    if (hf.mapped_ == MAP_FAILED) {
        hf.mapped_ = nullptr;
        ::close(hf.fd_);
        throw std::runtime_error("HCACHE: mmap failed");
    }

    // Zero header and write
    std::memset(hf.mapped_, 0, hcache::HEADER_SIZE);
    hf.write_header();

    // Write joint limits at offset 80
    for (int d = 0; d < n_dims && d < MAX_JOINTS; ++d) {
        double lo = limits.limits[d].lo;
        double hi = limits.limits[d].hi;
        std::memcpy(hf.mapped_ + 80 + d * 16,     &lo, 8);
        std::memcpy(hf.mapped_ + 80 + d * 16 + 8, &hi, 8);
    }

    // Write fingerprint SHA at offset 48
    size_t fp_len = std::min(fingerprint.size(), size_t(32));
    if (fp_len > 0)
        std::memcpy(hf.mapped_ + 48, fingerprint.data(), fp_len);

    // Set up NodeStore
    char* data_base = hf.mapped_ + hcache::HEADER_SIZE;
    hf.store_ = NodeStore(data_base, n_links, n_dims, initial_cap, 0);

    // Hook resize callback so NodeStore::ensure_capacity triggers grow_file
    hf.setup_resize_callback();

    return hf;
#else
    (void)path; (void)n_dims; (void)n_links; (void)limits;
    (void)fingerprint; (void)initial_cap;
    throw std::runtime_error("HCACHE: mmap not supported on this platform");
#endif
}

// ─── Flush to disk ───────────────────────────────────────────────────────────
void HCacheFile::setup_resize_callback() {
    store_.set_resize_callback([this](int /*old_cap*/, int new_cap) {
        grow_file(new_cap);
    });
}

void HCacheFile::flush() {
#if defined(__linux__) || defined(__APPLE__)
    if (mapped_) {
        // Update node count from store before flushing
        n_nodes_ = store_.next_idx();
        write_header();
        msync(mapped_, file_size_, MS_SYNC);
    }
#endif
}

// ─── Close and unmap ─────────────────────────────────────────────────────────
void HCacheFile::close() {
#if defined(__linux__) || defined(__APPLE__)
    if (mapped_) {
        flush();
        munmap(mapped_, file_size_);
        mapped_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    file_size_ = 0;
#endif
}

} // namespace sbf
