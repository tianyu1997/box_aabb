// SafeBoxForest v2 — FrameStore implementation
// Includes binary persistence (.frames, HCACHE03 format) with
// dirty tracking and incremental append-only save.
#include "sbf/envelope/frame_store.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <stdexcept>

namespace sbf {
namespace envelope {

// ─── Construction ───────────────────────────────────────────────────────────

FrameStore::FrameStore(const Robot& robot, int initial_capacity)
    : n_frames_(robot.n_joints() + (robot.has_tool() ? 1 : 0))
    , n_active_links_(robot.n_active_links())
    , capacity_(0)
{
    // Copy active link map
    assert(n_active_links_ <= MAX_LINKS);
    std::memcpy(active_link_map_, robot.active_link_map(),
                n_active_links_ * sizeof(int));

    // Copy link radii (as float)
    std::memset(link_radii_, 0, sizeof(link_radii_));
    if (robot.has_link_radii()) {
        const double* rd = robot.active_link_radii();
        for (int i = 0; i < n_active_links_; ++i)
            link_radii_[i] = static_cast<float>(rd[i]);
    }

    // Base position (default origin)
    base_pos_[0] = base_pos_[1] = base_pos_[2] = 0.f;

    ensure_capacity(initial_capacity);
}
FrameStore::~FrameStore() {
    if (mmap_.is_open()) {
        try { close_mmap(); } catch (...) {}
    }
}

FrameStore::FrameStore(FrameStore&& o) noexcept
    : n_frames_(o.n_frames_)
    , n_active_links_(o.n_active_links_)
    , capacity_(o.capacity_)
    , data_(std::move(o.data_))
    , valid_(std::move(o.valid_))
    , dirty_(std::move(o.dirty_))
    , n_dirty_(o.n_dirty_)
    , mmap_(std::move(o.mmap_))
    , save_path_(std::move(o.save_path_))
{
    std::memcpy(active_link_map_, o.active_link_map_, sizeof(active_link_map_));
    std::memcpy(link_radii_, o.link_radii_, sizeof(link_radii_));
    std::memcpy(base_pos_, o.base_pos_, sizeof(base_pos_));
    o.capacity_ = 0;
    o.n_dirty_ = 0;
}

FrameStore& FrameStore::operator=(FrameStore&& o) noexcept {
    if (this == &o) return *this;
    if (mmap_.is_open()) {
        try { close_mmap(); } catch (...) {}
    }
    n_frames_ = o.n_frames_;
    n_active_links_ = o.n_active_links_;
    capacity_ = o.capacity_;
    data_ = std::move(o.data_);
    valid_ = std::move(o.valid_);
    dirty_ = std::move(o.dirty_);
    n_dirty_ = o.n_dirty_;
    mmap_ = std::move(o.mmap_);
    save_path_ = std::move(o.save_path_);
    std::memcpy(active_link_map_, o.active_link_map_, sizeof(active_link_map_));
    std::memcpy(link_radii_, o.link_radii_, sizeof(link_radii_));
    std::memcpy(base_pos_, o.base_pos_, sizeof(base_pos_));
    o.capacity_ = 0;
    o.n_dirty_ = 0;
    return *this;
}
// ─── Dirty tracking helper ──────────────────────────────────────────────────

void FrameStore::mark_dirty(int node_idx) {
    if (mmap_.is_open()) {
        ++n_dirty_;
        return;
    }
    if (node_idx >= 0 && node_idx < capacity_) {
        if (!dirty_[node_idx]) {
            dirty_[node_idx] = 1;
            ++n_dirty_;
        }
    }
}

// ─── Store / retrieve ───────────────────────────────────────────────────────

void FrameStore::store_from_fk(int node_idx, const FKState& fk) {
    ensure_capacity(node_idx + 1);
    float* dst = get_frames_mut(node_idx);

    for (int k = 0; k < n_frames_; ++k) {
        const double* plo = fk.prefix_lo[k + 1];
        const double* phi = fk.prefix_hi[k + 1];
        float* f = dst + k * 6;
        f[0] = static_cast<float>(plo[3]);
        f[1] = static_cast<float>(plo[7]);
        f[2] = static_cast<float>(plo[11]);
        f[3] = static_cast<float>(phi[3]);
        f[4] = static_cast<float>(phi[7]);
        f[5] = static_cast<float>(phi[11]);
    }
    set_valid(node_idx, true);
    mark_dirty(node_idx);
}

void FrameStore::store_frames(int node_idx, const float* frames) {
    ensure_capacity(node_idx + 1);
    std::memcpy(get_frames_mut(node_idx), frames,
                n_frames_ * 6 * sizeof(float));
    set_valid(node_idx, true);
    mark_dirty(node_idx);
}

void FrameStore::set_has_frames(int node_idx, bool val) {
    if (node_idx >= 0 && node_idx < capacity_)
        set_valid(node_idx, val);
}

// ─── Frame-level union / refine ─────────────────────────────────────────────

void FrameStore::union_frames(int dst_idx, int a_idx, int b_idx) {
    const float* a = get_frames(a_idx);
    const float* b = get_frames(b_idx);
    if (!a || !b) return;

    ensure_capacity(dst_idx + 1);
    float* dst = get_frames_mut(dst_idx);

    for (int f = 0; f < n_frames_; ++f) {
        int off = f * 6;
        dst[off + 0] = std::min(a[off + 0], b[off + 0]);
        dst[off + 1] = std::min(a[off + 1], b[off + 1]);
        dst[off + 2] = std::min(a[off + 2], b[off + 2]);
        dst[off + 3] = std::max(a[off + 3], b[off + 3]);
        dst[off + 4] = std::max(a[off + 4], b[off + 4]);
        dst[off + 5] = std::max(a[off + 5], b[off + 5]);
    }
    set_valid(dst_idx, true);
    mark_dirty(dst_idx);
}

bool FrameStore::refine_frames(int dst_idx, int union_src_idx) {
    float* dst = get_frames_mut(dst_idx);
    const float* src = get_frames(union_src_idx);
    if (!dst || !src) return false;

    bool changed = false;
    for (int f = 0; f < n_frames_; ++f) {
        int off = f * 6;
        // Tighten lower bounds: take max
        for (int c = 0; c < 3; ++c) {
            float v = std::max(dst[off + c], src[off + c]);
            if (v != dst[off + c]) { dst[off + c] = v; changed = true; }
        }
        // Tighten upper bounds: take min
        for (int c = 3; c < 6; ++c) {
            float v = std::min(dst[off + c], src[off + c]);
            if (v != dst[off + c]) { dst[off + c] = v; changed = true; }
        }
    }
    if (changed) mark_dirty(dst_idx);
    return changed;
}

// ─── Persistence (.frames binary file) ──────────────────────────────────────
//
//  FRM3 (legacy) — sequential [node_idx, frames] records
//  FRM4 (new)    — fixed-stride flat indexed, mmap-friendly
//
//  Header layout (512 bytes, same offsets for both):
//    [0..3]     magic  (FRM3=0x334D5246, FRM4=0x344D5246)
//    [4..7]     version
//    [8..11]    n_frames
//    [12..15]   n_active
//    [16..19]   n_stored (FRM3) / capacity (FRM4)
//    [20..147]  active_link_map (32 × int32)
//    [148..275] link_radii (32 × float32)
//    [276..287] base_pos (3 × float32)
//    [288..511] reserved

void FrameStore::save(const std::string& path) {
    if (mmap_.is_open()) {
        flush_mmap();
        return;
    }

    FILE* fp = std::fopen(path.c_str(), "wb");
    if (!fp) throw std::runtime_error("FrameStore::save: cannot open " + path);

    int cap = capacity_;

    // Write FRM4 header
    uint8_t hdr[MMAP_HEADER_SIZE];
    std::memset(hdr, 0, MMAP_HEADER_SIZE);
    auto* u = reinterpret_cast<uint32_t*>(hdr);
    u[0] = MAGIC_V4;
    u[1] = VERSION_V4;
    u[2] = static_cast<uint32_t>(n_frames_);
    u[3] = static_cast<uint32_t>(n_active_links_);
    u[4] = static_cast<uint32_t>(cap);
    std::memcpy(hdr + 20, active_link_map_, MAX_LINKS * sizeof(int));
    std::memcpy(hdr + 148, link_radii_, MAX_LINKS * sizeof(float));
    std::memcpy(hdr + 276, base_pos_, 3 * sizeof(float));
    std::fwrite(hdr, 1, MMAP_HEADER_SIZE, fp);

    // Write all records: [valid:1B][pad:7B][frames: n_frames*24 bytes]
    const size_t data_per_rec = static_cast<size_t>(n_frames_) * 6 * sizeof(float);
    const size_t rec_sz = mmap_record_size();
    std::vector<uint8_t> rec(rec_sz, 0);

    for (int i = 0; i < cap; ++i) {
        std::memset(rec.data(), 0, 8);
        rec[0] = valid_[i];
        std::memcpy(rec.data() + 8, data_.data() + offset(i), data_per_rec);
        std::fwrite(rec.data(), 1, rec_sz, fp);
    }

    std::fclose(fp);
    std::fill(dirty_.begin(), dirty_.end(), 0);
    n_dirty_ = 0;
    save_path_ = path;
}

void FrameStore::save_incremental() {
    if (save_path_.empty()) return;
    save_incremental(save_path_);
}

void FrameStore::save_incremental(const std::string& path) {
    if (mmap_.is_open()) {
        flush_mmap();
        return;
    }

    if (save_path_.empty() || save_path_ != path) {
        save(path);
        return;
    }
    if (n_dirty_ == 0) return;

    FILE* fp = std::fopen(path.c_str(), "r+b");
    if (!fp) {
        save(path);
        return;
    }

    const size_t data_per_rec = static_cast<size_t>(n_frames_) * 6 * sizeof(float);
    const size_t rec_sz = mmap_record_size();
    std::vector<uint8_t> rec(rec_sz, 0);

    for (int i = 0; i < capacity_; ++i) {
        if (!dirty_[i]) continue;
        int64_t off = MMAP_HEADER_SIZE + static_cast<int64_t>(i) * static_cast<int64_t>(rec_sz);
        mmap_util::portable_fseek(fp, off, SEEK_SET);
        std::memset(rec.data(), 0, 8);
        rec[0] = valid_[i];
        std::memcpy(rec.data() + 8, data_.data() + offset(i), data_per_rec);
        std::fwrite(rec.data(), 1, rec_sz, fp);
    }

    // Update capacity in header
    mmap_util::portable_fseek(fp, 16, SEEK_SET);
    uint32_t cap32 = static_cast<uint32_t>(capacity_);
    std::fwrite(&cap32, sizeof(uint32_t), 1, fp);

    std::fclose(fp);
    std::fill(dirty_.begin(), dirty_.end(), 0);
    n_dirty_ = 0;
    save_path_ = path;
}

void FrameStore::load(const std::string& path) {
    FILE* fp = std::fopen(path.c_str(), "rb");
    if (!fp) throw std::runtime_error("FrameStore::load: cannot open " + path);

    uint8_t hdr[MMAP_HEADER_SIZE];
    if (std::fread(hdr, 1, MMAP_HEADER_SIZE, fp) !=
        static_cast<size_t>(MMAP_HEADER_SIZE)) {
        std::fclose(fp);
        throw std::runtime_error("FrameStore::load: truncated header in " + path);
    }

    auto* u = reinterpret_cast<const uint32_t*>(hdr);
    uint32_t magic = u[0];
    uint32_t ver   = u[1];

    if (magic == MAGIC_V4 && ver == VERSION_V4) {
        // ---- FRM4 flat format ----
        n_frames_       = static_cast<int>(u[2]);
        n_active_links_ = static_cast<int>(u[3]);
        int cap         = static_cast<int>(u[4]);

        std::memcpy(active_link_map_, hdr + 20,  MAX_LINKS * sizeof(int));
        std::memcpy(link_radii_,      hdr + 148, MAX_LINKS * sizeof(float));
        std::memcpy(base_pos_,        hdr + 276, 3 * sizeof(float));

        capacity_ = 0;
        data_.clear(); valid_.clear(); dirty_.clear();
        ensure_capacity(cap);

        const size_t rec_sz = mmap_record_size();
        std::vector<uint8_t> rec(rec_sz);
        const size_t data_per_rec = static_cast<size_t>(n_frames_) * 6 * sizeof(float);

        for (int i = 0; i < cap; ++i) {
            if (std::fread(rec.data(), 1, rec_sz, fp) != rec_sz) break;
            valid_[i] = rec[0];
            std::memcpy(data_.data() + offset(i), rec.data() + 8, data_per_rec);
        }

        std::fclose(fp);
        std::fill(dirty_.begin(), dirty_.end(), 0);
        n_dirty_ = 0;
        save_path_ = path;

    } else if (magic == MAGIC_V3 && ver == VERSION_V3) {
        // ---- FRM3 legacy format ----
        n_frames_       = static_cast<int>(u[2]);
        n_active_links_ = static_cast<int>(u[3]);
        int ns          = static_cast<int>(u[4]);

        std::memcpy(active_link_map_, hdr + 20,  MAX_LINKS * sizeof(int));
        std::memcpy(link_radii_,      hdr + 148, MAX_LINKS * sizeof(float));
        std::memcpy(base_pos_,        hdr + 276, 3 * sizeof(float));

        capacity_ = 0;
        data_.clear(); valid_.clear(); dirty_.clear();

        const int fpn = n_frames_ * 6;
        int max_idx = 0;

        struct Record {
            uint32_t idx;
            std::vector<float> data;
        };
        std::vector<Record> records;
        records.reserve(ns);

        for (int r = 0; r < ns; ++r) {
            uint32_t idx;
            if (std::fread(&idx, sizeof(uint32_t), 1, fp) != 1) break;
            records.push_back({idx, std::vector<float>(fpn)});
            if (std::fread(records.back().data.data(), sizeof(float), fpn, fp)
                != static_cast<size_t>(fpn)) {
                records.pop_back();
                break;
            }
            if (static_cast<int>(idx) + 1 > max_idx)
                max_idx = static_cast<int>(idx) + 1;
        }
        std::fclose(fp);

        ensure_capacity(max_idx);
        for (auto& rec : records) {
            int ni = static_cast<int>(rec.idx);
            std::memcpy(data_.data() + offset(ni), rec.data.data(),
                        fpn * sizeof(float));
            valid_[ni] = 1;
        }

        std::fill(dirty_.begin(), dirty_.end(), 0);
        n_dirty_ = 0;
        save_path_ = path;

    } else {
        std::fclose(fp);
        throw std::runtime_error(
            "FrameStore::load: unknown magic/version in " + path);
    }
}

int FrameStore::n_valid() const {
    int count = 0;
    for (int i = 0; i < capacity_; ++i)
        if (get_valid(i)) ++count;
    return count;
}

// ─── Capacity management ────────────────────────────────────────────────────

void FrameStore::ensure_capacity(int n_nodes) {
    if (n_nodes <= capacity_) return;
    int new_cap = std::max(n_nodes, capacity_ * 2);
    if (new_cap < 1024) new_cap = 1024;

    if (mmap_.is_open()) {
        grow_mmap(new_cap);
    } else {
        data_.resize(static_cast<size_t>(new_cap) * n_frames_ * 6, 0.f);
        valid_.resize(static_cast<size_t>(new_cap), 0);
        dirty_.resize(static_cast<size_t>(new_cap), 0);
    }
    capacity_ = new_cap;
}

// ─── Mmap persistence (lazy load + incremental store) ────────────────────────
//
//  FRM4 header layout (512 bytes), same offsets as FRM3.
//  After header: n_alloc × record_size bytes.
//  record_size = 8 + n_frames * 6 * sizeof(float)
//  Each record: [valid:1B][pad:7B][frames data]

void FrameStore::write_mmap_header() {
    assert(mmap_.is_open());
    char* p = mmap_.ptr;
    std::memset(p, 0, MMAP_HEADER_SIZE);
    auto* u = reinterpret_cast<uint32_t*>(p);
    u[0] = MAGIC_V4;
    u[1] = VERSION_V4;
    u[2] = static_cast<uint32_t>(n_frames_);
    u[3] = static_cast<uint32_t>(n_active_links_);
    u[4] = static_cast<uint32_t>(capacity_);
    std::memcpy(p + 20, active_link_map_, MAX_LINKS * sizeof(int));
    std::memcpy(p + 148, link_radii_, MAX_LINKS * sizeof(float));
    std::memcpy(p + 276, base_pos_, 3 * sizeof(float));
}

void FrameStore::read_mmap_header() {
    assert(mmap_.is_open());
    const char* p = mmap_.ptr;
    auto* u = reinterpret_cast<const uint32_t*>(p);
    if (u[0] != MAGIC_V4 || u[1] != VERSION_V4)
        throw std::runtime_error("FrameStore::read_mmap_header: bad magic/version");
    n_frames_       = static_cast<int>(u[2]);
    n_active_links_ = static_cast<int>(u[3]);
    capacity_       = static_cast<int>(u[4]);
    std::memcpy(active_link_map_, p + 20,  MAX_LINKS * sizeof(int));
    std::memcpy(link_radii_,      p + 148, MAX_LINKS * sizeof(float));
    std::memcpy(base_pos_,        p + 276, 3 * sizeof(float));
}

void FrameStore::grow_mmap(int new_cap) {
    assert(mmap_.is_open());
    size_t new_sz = MMAP_HEADER_SIZE +
                    static_cast<size_t>(new_cap) * mmap_record_size();
    mmap_util::grow(mmap_, new_sz);

    auto* u = reinterpret_cast<uint32_t*>(mmap_.ptr);
    u[4] = static_cast<uint32_t>(new_cap);
    capacity_ = new_cap;
}

void FrameStore::create_mmap(const std::string& path, int initial_capacity) {
    if (mmap_.is_open()) close_mmap();

    int cap = std::max(initial_capacity, 1024);
    size_t file_sz = MMAP_HEADER_SIZE +
                     static_cast<size_t>(cap) * mmap_record_size();

    mmap_ = mmap_util::open_rw(path, file_sz);
    capacity_ = cap;

    data_.clear(); data_.shrink_to_fit();
    valid_.clear(); valid_.shrink_to_fit();
    dirty_.clear(); dirty_.shrink_to_fit();
    n_dirty_ = 0;

    write_mmap_header();
    std::memset(mmap_.ptr + MMAP_HEADER_SIZE, 0,
                static_cast<size_t>(cap) * mmap_record_size());

    save_path_ = path;
}

void FrameStore::load_mmap(const std::string& path) {
    if (mmap_.is_open()) close_mmap();

    mmap_ = mmap_util::open_rw(path, 0);
    if (mmap_.size < static_cast<size_t>(MMAP_HEADER_SIZE))
        throw std::runtime_error("FrameStore::load_mmap: file too small " + path);

    read_mmap_header();

    size_t expected = MMAP_HEADER_SIZE +
                      static_cast<size_t>(capacity_) * mmap_record_size();
    if (mmap_.size < expected)
        mmap_util::grow(mmap_, expected);

    data_.clear(); data_.shrink_to_fit();
    valid_.clear(); valid_.shrink_to_fit();
    dirty_.clear(); dirty_.shrink_to_fit();
    n_dirty_ = 0;

    save_path_ = path;
}

void FrameStore::flush_mmap() {
    if (!mmap_.is_open()) return;
    write_mmap_header();
    mmap_util::flush(mmap_);
    n_dirty_ = 0;
}

void FrameStore::close_mmap() {
    if (!mmap_.is_open()) return;
    flush_mmap();
    mmap_util::close(mmap_);
    capacity_ = 0;
}

} // namespace envelope
} // namespace sbf
