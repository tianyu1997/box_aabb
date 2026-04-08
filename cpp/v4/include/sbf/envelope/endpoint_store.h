// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — EndpointStore: per-node endpoint position interval storage
//  Module: sbf::envelope
//
//  Stores per-endpoint position intervals (FRM4 format) as the universal base
//  from which all envelope types (iAABB, OBB, Grid, subdivided iAABB) can
//  be derived without re-running interval FK.
//
//  Each endpoint k stores:
//    [x_lo, y_lo, z_lo, x_hi, y_hi, z_hi]  (6 floats)
//
//  Endpoint indexing:
//    endpoint 0  =  FK prefix transform T^{0:1} (joint 0 distal)
//    endpoint k  =  FK prefix transform T^{0:k+1}
//    endpoint n_joints-1 = last joint distal
//    endpoint n_joints    = tool tip (if has_tool)
//
//  Base frame (f_0, i.e. T^{0:0}) is constant [0,0,0] and NOT stored;
//  it is kept in base_pos_[3] for derive functions.
//
//  Memory layout: flat contiguous array
//    data_[node_idx * n_endpoints_ * 6 + endpoint * 6 + {0..5}]
//
//  Persistence (.frames binary file):
//    Two on-disk formats:
//
//    FRM3 (legacy) — variable-length sequential records:
//      Header (512B) + [node_idx, payload] per valid node
//      Supported by load() only (full read, not mmap-friendly).
//
//    FRM4 (mmap-friendly) — fixed-stride flat indexed:
//      Header (512B) + n_alloc × RECORD_SIZE bytes
//      Each record: [valid:1B][pad:7B][n_endpoints×6 floats]
//      record_size = 8 + n_endpoints * 24
//      load_mmap() maps the file; OS loads pages lazily on demand.
//      save() / save_incremental() always write FRM4 format.
//
//  迁移自 v3 frame_store.h (FrameStore → EndpointStore)
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/core/types.h"
#include "sbf/core/mmap_util.h"
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"

#include <cstring>
#include <string>
#include <vector>

namespace sbf {
namespace envelope {

class EndpointStore {
public:
    EndpointStore() = default;
    explicit EndpointStore(const Robot& robot, int initial_capacity = 1024);
    ~EndpointStore();

    EndpointStore(const EndpointStore&)            = delete;
    EndpointStore& operator=(const EndpointStore&) = delete;
    EndpointStore(EndpointStore&& o) noexcept;
    EndpointStore& operator=(EndpointStore&& o) noexcept;

    int n_endpoints()    const { return n_endpoints_; }
    int n_active_links() const { return n_active_links_; }
    int capacity()       const { return capacity_; }
    const int* active_link_map() const { return active_link_map_; }
    const float* link_radii() const { return link_radii_; }
    const float* base_pos() const { return base_pos_; }
    int floats_per_node() const { return n_endpoints_ * 6; }

    void store_from_fk(int node_idx, const FKState& fk);
    void store_endpoints(int node_idx, const float* endpoints);

    const float* get_endpoints(int node_idx) const {
        if (mmap_.is_open())
            return reinterpret_cast<const float*>(
                mmap_.ptr + MMAP_HEADER_SIZE +
                static_cast<size_t>(node_idx) * mmap_record_size() + 8);
        return data_.data() + offset(node_idx);
    }

    float* get_endpoints_mut(int node_idx) {
        if (mmap_.is_open())
            return reinterpret_cast<float*>(
                mmap_.ptr + MMAP_HEADER_SIZE +
                static_cast<size_t>(node_idx) * mmap_record_size() + 8);
        return data_.data() + offset(node_idx);
    }

    bool has_endpoints(int node_idx) const {
        if (node_idx < 0 || node_idx >= capacity_) return false;
        return get_valid(node_idx);
    }

    void set_has_endpoints(int node_idx, bool val);

    void union_endpoints(int dst_idx, int a_idx, int b_idx);
    bool refine_endpoints(int dst_idx, int union_src_idx);

    void save(const std::string& path);
    void save_incremental();
    void save_incremental(const std::string& path);
    void load(const std::string& path);

    void create_mmap(const std::string& path, int initial_capacity);
    void load_mmap(const std::string& path);
    void flush_mmap();
    void close_mmap();

    bool is_mmap() const { return mmap_.is_open(); }
    int n_dirty() const { return n_dirty_; }
    int n_valid() const;
    const std::string& save_path() const { return save_path_; }

    void ensure_capacity(int n_nodes);

private:
    int n_endpoints_    = 0;
    int n_active_links_ = 0;
    int capacity_       = 0;

    int   active_link_map_[MAX_LINKS] = {};
    float link_radii_[MAX_LINKS]      = {};
    float base_pos_[3]                = {0.f, 0.f, 0.f};

    std::vector<float>   data_;
    std::vector<uint8_t> valid_;
    std::vector<uint8_t> dirty_;
    int n_dirty_ = 0;

    mmap_util::MmapHandle mmap_;
    std::string save_path_;

    void mark_dirty(int node_idx);

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

    int offset(int node_idx) const { return node_idx * n_endpoints_ * 6; }

    size_t mmap_record_size() const {
        return 8 + static_cast<size_t>(n_endpoints_) * 6 * sizeof(float);
    }
    void grow_mmap(int new_cap);
    void write_mmap_header();
    void read_mmap_header();

    static constexpr uint32_t MAGIC_V3   = 0x334D5246;  // "FRM3"
    static constexpr uint32_t VERSION_V3 = 1;
    static constexpr uint32_t MAGIC_V4   = 0x344D5246;  // "FRM4"
    static constexpr uint32_t VERSION_V4 = 2;
    static constexpr int MMAP_HEADER_SIZE = 512;

    static constexpr uint32_t MAGIC   = MAGIC_V3;
    static constexpr uint32_t VERSION = VERSION_V3;
    static constexpr int HEADER_SIZE  = 512;
};

} // namespace envelope
} // namespace sbf
