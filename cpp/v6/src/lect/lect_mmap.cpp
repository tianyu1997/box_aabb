// SafeBoxForest v6 — LECT memory-mapped file wrapper (Linux/POSIX)

#include <sbf/lect/lect_mmap.h>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

namespace sbf {

LectMmap::~LectMmap() { close(); }

LectMmap::LectMmap(LectMmap&& o) noexcept
    : data_(o.data_), size_(o.size_), fd_(o.fd_) {
    o.data_ = nullptr;
    o.size_ = 0;
    o.fd_   = -1;
}

LectMmap& LectMmap::operator=(LectMmap&& o) noexcept {
    if (this != &o) {
        close();
        data_ = o.data_;
        size_ = o.size_;
        fd_   = o.fd_;
        o.data_ = nullptr;
        o.size_ = 0;
        o.fd_   = -1;
    }
    return *this;
}

bool LectMmap::open(const std::string& path) {
    close();
    fd_ = ::open(path.c_str(), O_RDONLY);
    if (fd_ < 0) return false;

    struct stat st;
    if (fstat(fd_, &st) < 0) {
        ::close(fd_);
        fd_ = -1;
        return false;
    }
    size_ = static_cast<size_t>(st.st_size);
    if (size_ == 0) {
        ::close(fd_);
        fd_ = -1;
        size_ = 0;
        return false;
    }

    void* ptr = ::mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_PRIVATE, fd_, 0);
    if (ptr == MAP_FAILED) {
        ::close(fd_);
        fd_ = -1;
        size_ = 0;
        return false;
    }

    data_ = static_cast<uint8_t*>(ptr);
    // Random access pattern — let kernel decide prefetch policy
    ::madvise(data_, size_, MADV_RANDOM);
    return true;
}

void LectMmap::close() {
    if (data_) {
        ::munmap(data_, size_);
        data_ = nullptr;
    }
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
    size_ = 0;
}

LectMmap LectMmap::clone() const {
    if (fd_ < 0 || !data_) return {};
    int new_fd = ::dup(fd_);
    if (new_fd < 0) return {};
    void* ptr = ::mmap(nullptr, size_, PROT_READ | PROT_WRITE,
                       MAP_PRIVATE, new_fd, 0);
    if (ptr == MAP_FAILED) {
        ::close(new_fd);
        return {};
    }
    ::madvise(ptr, size_, MADV_RANDOM);
    LectMmap result;
    result.data_ = static_cast<uint8_t*>(ptr);
    result.size_ = size_;
    result.fd_ = new_fd;
    return result;
}

}  // namespace sbf
