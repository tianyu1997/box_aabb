// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Cross-platform memory-mapped file utility (header-only)
//  Module: sbf::mmap_util
//
//  Provides MmapHandle: a RAII handle for read-write shared memory mapping
//  backed by a disk file.  OS loads pages lazily on demand (page faults).
//
//  Byte order: LITTLE-ENDIAN only (x86, x86_64, ARM64-LE).
//  Supported platforms: Linux/macOS (mmap), Windows (CreateFileMapping)
//
//  迁移自 v3 mmap_util.h，路径 sbf/common/ → sbf/core/
// ═══════════════════════════════════════════════════════════════════════════
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

struct MmapHandle {
    char*  ptr  = nullptr;
    size_t size = 0;

#ifdef _WIN32
    HANDLE hFile    = INVALID_HANDLE_VALUE;
    HANDLE hMapping = nullptr;
#else
    int fd = -1;
#endif

    bool is_open() const { return ptr != nullptr; }

    MmapHandle() = default;
    MmapHandle(const MmapHandle&) = delete;
    MmapHandle& operator=(const MmapHandle&) = delete;

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

inline MmapHandle open_rw(const std::string& path, size_t min_size) {
    MmapHandle h;

#ifdef _WIN32
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

    LARGE_INTEGER fsize;
    if (!GetFileSizeEx(h.hFile, &fsize)) {
        CloseHandle(h.hFile); h.hFile = INVALID_HANDLE_VALUE;
        throw std::runtime_error("mmap_util::open_rw: GetFileSizeEx failed");
    }
    size_t cur_size = static_cast<size_t>(fsize.QuadPart);
    size_t map_size = (cur_size < min_size) ? min_size : cur_size;

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

#else
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

inline void grow(MmapHandle& h, size_t new_size) {
    if (new_size <= h.size) return;

#ifdef _WIN32
    if (h.ptr)      { UnmapViewOfFile(h.ptr);      h.ptr = nullptr; }
    if (h.hMapping) { CloseHandle(h.hMapping);      h.hMapping = nullptr; }

    LARGE_INTEGER li;
    li.QuadPart = static_cast<LONGLONG>(new_size);
    if (!SetFilePointerEx(h.hFile, li, nullptr, FILE_BEGIN) ||
        !SetEndOfFile(h.hFile)) {
        throw std::runtime_error("mmap_util::grow: SetEndOfFile failed");
    }

    h.hMapping = CreateFileMappingA(
        h.hFile, nullptr, PAGE_READWRITE, li.HighPart, li.LowPart, nullptr);
    if (!h.hMapping)
        throw std::runtime_error("mmap_util::grow: CreateFileMapping failed");

    h.ptr = static_cast<char*>(
        MapViewOfFile(h.hMapping, FILE_MAP_ALL_ACCESS, 0, 0, new_size));
    if (!h.ptr)
        throw std::runtime_error("mmap_util::grow: MapViewOfFile failed");
    h.size = new_size;

#else
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

inline void flush(MmapHandle& h) {
    if (!h.ptr) return;
#ifdef _WIN32
    FlushViewOfFile(h.ptr, h.size);
    FlushFileBuffers(h.hFile);
#else
    msync(h.ptr, h.size, MS_SYNC);
#endif
}

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
