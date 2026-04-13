#pragma once
/// @file lect_mmap.h
/// @brief RAII wrapper for COW memory-mapped LECT binary files (Linux/POSIX).

#include <cstddef>
#include <cstdint>
#include <string>

namespace sbf {

/// RAII wrapper for a COW (copy-on-write) memory-mapped file.
/// Opened with PROT_READ|PROT_WRITE + MAP_PRIVATE so reads demand-page
/// from the file and writes trigger per-page COW (file is never modified).
/// Used by LECT to lazily access ep_data from the cached binary file.
class LectMmap {
public:
    LectMmap() = default;
    ~LectMmap();

    LectMmap(const LectMmap&) = delete;
    LectMmap& operator=(const LectMmap&) = delete;

    LectMmap(LectMmap&& o) noexcept;
    LectMmap& operator=(LectMmap&& o) noexcept;

    /// Open a file for COW memory mapping (PROT_READ|PROT_WRITE, MAP_PRIVATE).
    bool open(const std::string& path);

    /// Create an independent COW mapping of the same file (dup fd + new mmap).
    /// Returns empty LectMmap on failure.
    LectMmap clone() const;

    /// Release the mapping and close the file descriptor.
    void close();

    bool is_open() const { return data_ != nullptr; }

    const uint8_t* data() const { return data_; }
    /// Writable pointer — writes trigger per-page COW (MAP_PRIVATE).
    uint8_t* writable_data() { return data_; }
    size_t size() const { return size_; }

private:
    uint8_t* data_ = nullptr;
    size_t   size_ = 0;
    int      fd_   = -1;
};

}  // namespace sbf
