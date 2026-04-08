// SafeBoxForest v2 — FrameStore: per-node frame position interval storage
// Module: sbf::envelope
//
// Stores per-frame position intervals (HCACHE03) as the universal base
// from which all envelope types (AABB, OBB, Grid, subdivided AABB) can
// be derived without re-running interval FK.
//
// Each frame k stores:
//   [x_lo, y_lo, z_lo, x_hi, y_hi, z_hi]  (6 floats)
//
// Frame indexing:
//   frame 0  =  FK prefix transform T^{0:1} (joint 0 distal)
//   frame k  =  FK prefix transform T^{0:k+1}
//   frame n_joints-1 = last joint distal
//   frame n_joints    = tool tip (if has_tool)
//
// Base frame (f_0, i.e. T^{0:0}) is constant [0,0,0] and NOT stored;
// it is kept in base_pos_[3] for derive functions.
//
// Memory layout: flat contiguous array
//   data_[node_idx * n_frames_ * 6 + frame * 6 + {0..5}]
//
// Persistence (.frames binary file):
//   Two on-disk formats:
//
//   FRM3 (legacy) — variable-length sequential records:
//     Header (512B) + [node_idx, payload] per valid node
//     Supported by load() only (full read, not mmap-friendly).
//
//   FRM4 (mmap-friendly) — fixed-stride flat indexed:
//     Header (512B) + n_alloc × RECORD_SIZE bytes
//     Each record: [valid:1B][pad:7B][n_frames×6 floats]
//     record_size = 8 + n_frames * 24
//     load_mmap() maps the file; OS loads pages lazily on demand.
//     save() / save_incremental() always write FRM4 format.
//
#pragma once

#include "sbf/common/types.h"
#include "sbf/common/mmap_util.h"
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"

#include <cstring>
#include <string>
#include <vector>

namespace sbf {
namespace envelope {

// ═════════════════════════════════════════════════════════════════════════════
//  FrameStore — per-node frame position interval storage
// ═════════════════════════════════════════════════════════════════════════════
class FrameStore {
public:
    FrameStore() = default;

    // Construct from robot metadata.
    // n_frames = n_joints + (has_tool ? 1 : 0).
    explicit FrameStore(const Robot& robot, int initial_capacity = 1024);

    // Destructor (closes mmap if open)
    ~FrameStore();

    // Non-copyable
    FrameStore(const FrameStore&)            = delete;
    FrameStore& operator=(const FrameStore&) = delete;

    // Move semantics (transfers mmap ownership)
    FrameStore(FrameStore&& o) noexcept;
    FrameStore& operator=(FrameStore&& o) noexcept;

    // ── Metadata ─────────────────────────────────────────────────────────
    int n_frames()       const { return n_frames_; }
    int n_active_links() const { return n_active_links_; }
    int capacity()       const { return capacity_; }

    // Active link index → frame index mapping (compact_idx → frame_idx)
    const int* active_link_map() const { return active_link_map_; }

    // Link radii in active-link compact order
    const float* link_radii() const { return link_radii_; }

    // Base frame position (constant, typically [0,0,0])
    const float* base_pos() const { return base_pos_; }

    // Floats per node: n_frames * 6
    int floats_per_node() const { return n_frames_ * 6; }

    // ── Store / retrieve ─────────────────────────────────────────────────

    // Extract and store frames from a completed FKState.
    // Reads prefix_lo/hi[k+1][:3,3] for k = 0 .. n_frames-1.
    void store_from_fk(int node_idx, const FKState& fk);

    // Store raw frame data (n_frames × 6 floats).
    void store_frames(int node_idx, const float* frames);

    // Get pointer to frame data for node_idx (n_frames × 6 floats).
    // Returns nullptr if node has no data (caller must check).
    const float* get_frames(int node_idx) const {
        if (mmap_.is_open())
            return reinterpret_cast<const float*>(
                mmap_.ptr + MMAP_HEADER_SIZE +
                static_cast<size_t>(node_idx) * mmap_record_size() + 8);
        return data_.data() + offset(node_idx);
    }

    // Mutable access (for union/refine in-place operations).
    float* get_frames_mut(int node_idx) {
        if (mmap_.is_open())
            return reinterpret_cast<float*>(
                mmap_.ptr + MMAP_HEADER_SIZE +
                static_cast<size_t>(node_idx) * mmap_record_size() + 8);
        return data_.data() + offset(node_idx);
    }

    // Check if a node has frame data stored.
    bool has_frames(int node_idx) const {
        if (node_idx < 0 || node_idx >= capacity_) return false;
        return get_valid(node_idx);
    }

    // Mark a node as having valid frame data.
    void set_has_frames(int node_idx, bool val);

    // ── Frame-level union / refine ───────────────────────────────────────

    // Union: dst = hull(a, b) — per-float min/max.
    //   dst_lo = min(a_lo, b_lo), dst_hi = max(a_hi, b_hi)
    void union_frames(int dst_idx, int a_idx, int b_idx);

    // Refine: dst = intersect(dst, union_src) — tightens bounds.
    //   dst_lo = max(dst_lo, union_lo), dst_hi = min(dst_hi, union_hi)
    // Returns true if any value changed.
    bool refine_frames(int dst_idx, int union_src_idx);

    // ── Persistence ──────────────────────────────────────────────────────

    // Save all valid nodes (FRM4 flat format). In mmap mode: flushes.
    void save(const std::string& path);

    // Incremental: mmap → flush; non-mmap → write dirty in-place (FRM4).
    void save_incremental();
    void save_incremental(const std::string& path);

    // Load from .frames file. Auto-detects FRM3 (legacy) or FRM4 (flat).
    void load(const std::string& path);

    // ── Mmap persistence (lazy load) ─────────────────────────────────────

    // Create a new mmap-backed FRM4 file.
    void create_mmap(const std::string& path, int initial_capacity);

    // Open existing FRM4 file via mmap (lazy load on demand).
    void load_mmap(const std::string& path);

    // Sync dirty pages to disk.
    void flush_mmap();

    // Close mmap (flushes first).
    void close_mmap();

    bool is_mmap() const { return mmap_.is_open(); }

    int n_dirty() const { return n_dirty_; }
    int n_valid() const;
    const std::string& save_path() const { return save_path_; }

    // ── Capacity management ──────────────────────────────────────────────

    // Ensure space for at least n_nodes entries.
    void ensure_capacity(int n_nodes);

private:
    int n_frames_       = 0;
    int n_active_links_ = 0;
    int capacity_       = 0;

    int   active_link_map_[MAX_LINKS] = {};
    float link_radii_[MAX_LINKS]      = {};
    float base_pos_[3]                = {0.f, 0.f, 0.f};

    // ── Vector-backed storage (non-mmap mode) ────────────────────────────
    std::vector<float>   data_;    // [capacity * n_frames * 6]
    std::vector<uint8_t> valid_;   // [capacity]
    std::vector<uint8_t> dirty_;   // [capacity]
    int n_dirty_ = 0;

    // ── Mmap state ───────────────────────────────────────────────────────
    mmap_util::MmapHandle mmap_;

    // ── Common persistence state ─────────────────────────────────────────
    std::string save_path_;

    void mark_dirty(int node_idx);

    // ── Valid flag access (dual-mode) ────────────────────────────────────
    bool get_valid(int node_idx) const {
        if (mmap_.is_open())
            return *(mmap_.ptr + MMAP_HEADER_SIZE +
                     static_cast<size_t>(node_idx) * mmap_record_size()) != 0;
        return valid_[node_idx] != 0;
    }
    void set_valid(int node_idx, bool v) {
        if (mmap_.is_open())
            *(mmap_.ptr + MMAP_HEADER_SIZE +
              static_cast<size_t>(node_idx) * mmap_record_size()) =
                static_cast<char>(v ? 1 : 0);
        else
            valid_[node_idx] = v ? 1 : 0;
    }

    // Offset helper (non-mmap)
    int offset(int node_idx) const { return node_idx * n_frames_ * 6; }

    // ── Mmap helpers ─────────────────────────────────────────────────────
    size_t mmap_record_size() const {
        return 8 + static_cast<size_t>(n_frames_) * 6 * sizeof(float);
    }
    void grow_mmap(int new_cap);
    void write_mmap_header();
    void read_mmap_header();

    // ── Binary format constants ──────────────────────────────────────────

    // Legacy FRM3
    static constexpr uint32_t MAGIC_V3   = 0x334D5246;  // "FRM3" LE
    static constexpr uint32_t VERSION_V3 = 1;

    // FRM4 (mmap-friendly fixed stride)
    static constexpr uint32_t MAGIC_V4   = 0x344D5246;  // "FRM4" LE
    static constexpr uint32_t VERSION_V4 = 2;

    static constexpr int MMAP_HEADER_SIZE = 512;

    // Legacy aliases for load() v3 path
    static constexpr uint32_t MAGIC   = MAGIC_V3;
    static constexpr uint32_t VERSION = VERSION_V3;
    static constexpr int HEADER_SIZE  = 512;
};

} // namespace envelope
} // namespace sbf
