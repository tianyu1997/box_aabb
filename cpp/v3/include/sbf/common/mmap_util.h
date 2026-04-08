// SafeBoxForest v2 — Cross-platform memory-mapped file utility (header-only)
// Module: sbf::mmap_util
//
// Provides MmapHandle: a RAII handle for read-write shared memory mapping
// backed by a disk file.  OS loads pages lazily on demand (page faults).
//
// Byte order: LITTLE-ENDIAN only (x86, x86_64, ARM64-LE).
// Files produced on Windows x64 are binary-compatible with Linux x64
// and vice versa — both use the same LE byte order.
//
// Supported platforms:
//   Linux / macOS:  mmap(2) + ftruncate(2)
//   Windows:        CreateFileMapping + MapViewOfFile
//
#pragma once

#ifdef _WIN32
  #ifndef WIN32_LEAN_AND_MEAN
  #define WIN32_LEAN_AND_MEAN
  #endif
  #ifndef NOMINMAX
  #define NOMINMAX
  #endif
  #include <windows.h>
#else
  #include <sys/mman.h>
  #include <sys/stat.h>
  #include <fcntl.h>
  #include <unistd.h>
#endif

#include <cstddef>
#include <cstdint>
#include <string>
#include <stdexcept>

namespace sbf {
namespace mmap_util {

// ─────────────────────────────────────────────────────────────────────────────
//  MmapHandle — cross-platform memory-mapped file handle
// ─────────────────────────────────────────────────────────────────────────────
struct MmapHandle {
    char*  ptr  = nullptr;   // pointer to mapped memory
    size_t size = 0;         // current mapped size in bytes

#ifdef _WIN32
    HANDLE hFile    = INVALID_HANDLE_VALUE;
    HANDLE hMapping = nullptr;
#else
    int fd = -1;
#endif

    bool is_open() const { return ptr != nullptr; }

    // Non-copyable
    MmapHandle() = default;
    MmapHandle(const MmapHandle&) = delete;
    MmapHandle& operator=(const MmapHandle&) = delete;

    // Move semantics — transfer ownership, null-out source
    MmapHandle(MmapHandle&& o) noexcept
        : ptr(o.ptr), size(o.size)
#ifdef _WIN32
        , hFile(o.hFile), hMapping(o.hMapping)
#else
        , fd(o.fd)
#endif
    {
        o.ptr  = nullptr;
        o.size = 0;
#ifdef _WIN32
        o.hFile    = INVALID_HANDLE_VALUE;
        o.hMapping = nullptr;
#else
        o.fd = -1;
#endif
    }

    MmapHandle& operator=(MmapHandle&& o) noexcept {
        if (this != &o) {
            // Note: caller should close() before reassigning.
            ptr  = o.ptr;  size = o.size;
#ifdef _WIN32
            hFile = o.hFile; hMapping = o.hMapping;
            o.hFile = INVALID_HANDLE_VALUE; o.hMapping = nullptr;
#else
            fd = o.fd; o.fd = -1;
#endif
            o.ptr = nullptr; o.size = 0;
        }
        return *this;
    }
};

// ─── Open (or create) a file and map it read-write ──────────────────────────
//
// If the file is smaller than min_size, it is extended (zero-filled).
// Returns an MmapHandle with ptr pointing to the beginning of the file.
//
inline MmapHandle open_rw(const std::string& path, size_t min_size) {
    MmapHandle h;

#ifdef _WIN32
    // Open or create file for read-write
    h.hFile = CreateFileA(
        path.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        FILE_SHARE_READ,
        nullptr,
        OPEN_ALWAYS,
        FILE_ATTRIBUTE_NORMAL,
        nullptr);
    if (h.hFile == INVALID_HANDLE_VALUE)
        throw std::runtime_error("mmap_util::open_rw: cannot open " + path);

    // Get current file size
    LARGE_INTEGER fsize;
    if (!GetFileSizeEx(h.hFile, &fsize)) {
        CloseHandle(h.hFile); h.hFile = INVALID_HANDLE_VALUE;
        throw std::runtime_error("mmap_util::open_rw: GetFileSizeEx failed");
    }
    size_t cur_size = static_cast<size_t>(fsize.QuadPart);
    size_t map_size = (cur_size < min_size) ? min_size : cur_size;

    // Create file mapping (also extends the file if map_size > cur_size)
    LARGE_INTEGER li;
    li.QuadPart = static_cast<LONGLONG>(map_size);
    h.hMapping = CreateFileMappingA(
        h.hFile, nullptr, PAGE_READWRITE,
        li.HighPart, li.LowPart, nullptr);
    if (!h.hMapping) {
        CloseHandle(h.hFile); h.hFile = INVALID_HANDLE_VALUE;
        throw std::runtime_error("mmap_util::open_rw: CreateFileMapping failed for " + path);
    }

    h.ptr = static_cast<char*>(
        MapViewOfFile(h.hMapping, FILE_MAP_ALL_ACCESS, 0, 0, map_size));
    if (!h.ptr) {
        CloseHandle(h.hMapping); h.hMapping = nullptr;
        CloseHandle(h.hFile);    h.hFile = INVALID_HANDLE_VALUE;
        throw std::runtime_error("mmap_util::open_rw: MapViewOfFile failed for " + path);
    }
    h.size = map_size;

#else  // Linux / macOS
    h.fd = ::open(path.c_str(), O_RDWR | O_CREAT, 0644);
    if (h.fd < 0)
        throw std::runtime_error("mmap_util::open_rw: cannot open " + path);

    struct stat st;
    if (fstat(h.fd, &st) < 0) {
        ::close(h.fd); h.fd = -1;
        throw std::runtime_error("mmap_util::open_rw: fstat failed");
    }
    size_t cur_size = static_cast<size_t>(st.st_size);
    size_t map_size = (cur_size < min_size) ? min_size : cur_size;

    if (map_size > cur_size) {
        if (ftruncate(h.fd, static_cast<off_t>(map_size)) < 0) {
            ::close(h.fd); h.fd = -1;
            throw std::runtime_error("mmap_util::open_rw: ftruncate failed");
        }
    }

    h.ptr = static_cast<char*>(
        mmap(nullptr, map_size, PROT_READ | PROT_WRITE, MAP_SHARED, h.fd, 0));
    if (h.ptr == MAP_FAILED) {
        ::close(h.fd); h.fd = -1; h.ptr = nullptr;
        throw std::runtime_error("mmap_util::open_rw: mmap failed");
    }
    h.size = map_size;
#endif

    return h;
}

// ─── Grow the mapping to new_size ───────────────────────────────────────────
//
// Extends the file and remaps.  The 'ptr' field may change.
// new_size must be >= current size.
// New bytes are zero-filled by the OS.
//
inline void grow(MmapHandle& h, size_t new_size) {
    if (new_size <= h.size) return;

#ifdef _WIN32
    // Unmap current view
    if (h.ptr)      { UnmapViewOfFile(h.ptr);      h.ptr = nullptr; }
    if (h.hMapping) { CloseHandle(h.hMapping);      h.hMapping = nullptr; }

    // Extend file
    LARGE_INTEGER li;
    li.QuadPart = static_cast<LONGLONG>(new_size);
    if (!SetFilePointerEx(h.hFile, li, nullptr, FILE_BEGIN) ||
        !SetEndOfFile(h.hFile)) {
        throw std::runtime_error("mmap_util::grow: SetEndOfFile failed");
    }

    // Re-map
    h.hMapping = CreateFileMappingA(
        h.hFile, nullptr, PAGE_READWRITE, li.HighPart, li.LowPart, nullptr);
    if (!h.hMapping)
        throw std::runtime_error("mmap_util::grow: CreateFileMapping failed");

    h.ptr = static_cast<char*>(
        MapViewOfFile(h.hMapping, FILE_MAP_ALL_ACCESS, 0, 0, new_size));
    if (!h.ptr)
        throw std::runtime_error("mmap_util::grow: MapViewOfFile failed");
    h.size = new_size;

#else  // Linux / macOS
    munmap(h.ptr, h.size);

    if (ftruncate(h.fd, static_cast<off_t>(new_size)) < 0)
        throw std::runtime_error("mmap_util::grow: ftruncate failed");

    h.ptr = static_cast<char*>(
        mmap(nullptr, new_size, PROT_READ | PROT_WRITE, MAP_SHARED, h.fd, 0));
    if (h.ptr == MAP_FAILED) {
        h.ptr = nullptr;
        throw std::runtime_error("mmap_util::grow: mmap failed");
    }
    h.size = new_size;
#endif
}

// ─── Sync dirty pages to disk ───────────────────────────────────────────────
inline void flush(MmapHandle& h) {
    if (!h.ptr) return;
#ifdef _WIN32
    FlushViewOfFile(h.ptr, h.size);
    FlushFileBuffers(h.hFile);
#else
    msync(h.ptr, h.size, MS_SYNC);
#endif
}

// ─── Unmap and close file handle ────────────────────────────────────────────
inline void close(MmapHandle& h) {
    if (!h.is_open()) return;
    flush(h);
#ifdef _WIN32
    if (h.ptr)      { UnmapViewOfFile(h.ptr);   h.ptr = nullptr; }
    if (h.hMapping) { CloseHandle(h.hMapping);   h.hMapping = nullptr; }
    if (h.hFile != INVALID_HANDLE_VALUE) {
        CloseHandle(h.hFile);
        h.hFile = INVALID_HANDLE_VALUE;
    }
#else
    if (h.ptr)    { munmap(h.ptr, h.size); h.ptr = nullptr; }
    if (h.fd >= 0) { ::close(h.fd);         h.fd = -1; }
#endif
    h.size = 0;
}

// ─── Portable 64-bit fseek / ftell ──────────────────────────────────────────
// On Windows (MSVC), `long` is 32-bit even on x64 → fseek cannot seek >2 GB.
// These helpers use _fseeki64/_ftelli64 on Windows and fseeko/ftello on POSIX.

inline int portable_fseek(FILE* fp, int64_t offset, int whence) {
#ifdef _WIN32
    return _fseeki64(fp, offset, whence);
#else
    return fseeko(fp, static_cast<off_t>(offset), whence);
#endif
}

inline int64_t portable_ftell(FILE* fp) {
#ifdef _WIN32
    return _ftelli64(fp);
#else
    return static_cast<int64_t>(ftello(fp));
#endif
}

} // namespace mmap_util
} // namespace sbf
