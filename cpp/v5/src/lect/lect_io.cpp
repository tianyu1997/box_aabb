// SafeBoxForest v5 — LECT binary cache IO (V5 SoA mmap format)
//
// V5 layout: SoA (Structure-of-Arrays) with separated tree + EP sections.
//
//   [Header 128B]
//   [Root intervals: n_dims × 2 × double]
//   [Tree section: cap × 32 bytes (SoA)]
//     ├─ left[cap]             int32
//     ├─ right[cap]            int32
//     ├─ parent[cap]           int32
//     ├─ depth[cap]            int32
//     ├─ split_dim[cap]        int32
//     ├─ split_val[cap]        float64
//     ├─ has_data[cap × 2]     uint8  (safe, unsafe per node)
//     └─ source_quality[cap×2] uint8
//   [EP section: cap × ep_node_stride bytes]
//     per-node: ep_safe[ep_stride] then ep_unsafe[ep_stride]
//   [Derived cache: root_fk + symmetry + radii]
//   [Cache trailer: z4_cache + depth_split_dim_cache]
//   [Grid section: sparse voxel grids]
//
// The tree section is bulk-memcpy'd into vectors on load (~17MB → <2ms).
// EP data is served from a COW (MAP_PRIVATE) mmap of the EP section.

#include <sbf/lect/lect_io.h>
#include <sbf/lect/lect_mmap.h>
#include <sbf/core/fk_state.h>
#include <sbf/core/joint_symmetry.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/link_iaabb.h>
#include <sbf/voxel/voxel_grid.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <streambuf>
#include <vector>

#include <sys/mman.h>

namespace sbf {

// ═══════════════════════════════════════════════════════════════════════════
//  Binary format constants
// ═══════════════════════════════════════════════════════════════════════════

static constexpr char MAGIC[8] = {'S','B','F','5','L','E','C','T'};
static constexpr uint32_t FORMAT_V5 = 5;

// ═════════════════════════════════════════════════════════════════════════════
//  Zero-copy streambuf over a memory region (for reading trailer from mmap)
// ═════════════════════════════════════════════════════════════════════════════

class MemBuf : public std::streambuf {
public:
    MemBuf(const char* begin, size_t size) {
        char* b = const_cast<char*>(begin);
        setg(b, b, b + size);
    }
};

// ═════════════════════════════════════════════════════════════════════════════
//  V5 Header (128 bytes, fixed)
// ═════════════════════════════════════════════════════════════════════════════

struct LectFileHeaderV5 {
    char     magic[8];          //   0
    uint32_t version;           //   8
    int32_t  n_nodes;           //  12
    int32_t  n_dims;            //  16
    int32_t  n_active_links;    //  20
    int32_t  ep_stride;         //  24
    int32_t  liaabb_stride;     //  28
    uint8_t  split_order;       //  32
    uint8_t  ep_source;         //  33
    uint8_t  env_type;          //  34
    uint8_t  n_channels;        //  35
    int32_t  capacity;          //  36: SoA array size (≥ n_nodes)
    uint64_t robot_hash;        //  40
    uint8_t  grid_section;      //  48
    uint8_t  has_derived;       //  49
    uint8_t  reserved2[2];      //  50
    float    grid_delta;        //  52
    uint32_t tree_record_bytes; //  56
    uint32_t reserved3;         //  60
    uint64_t tree_section_off;  //  64
    uint64_t ep_section_off;    //  72
    uint64_t derived_off;       //  80
    uint64_t trailer_off;       //  88
    uint64_t grid_off;          //  96
    uint8_t  reserved5[24];     // 104-127
};

static_assert(sizeof(LectFileHeaderV5) == 128, "V5 header must be 128 bytes");

// SoA per-node tree record = 5×int32 + double + 2×uint8 + 2×uint8 = 32
static constexpr int TREE_RECORD_BYTES = 32;

/// EP stride per node in bytes: 2 × ep_stride × sizeof(float).
static size_t ep_node_bytes(int ep_stride) {
    return static_cast<size_t>(2) * ep_stride * sizeof(float);
}

// ═══════════════════════════════════════════════════════════════════════════
//  LectIOHelper — friend struct for accessing LECT internals
// ═══════════════════════════════════════════════════════════════════════════

struct LectIOHelper {

// ── make_header ─────────────────────────────────────────────────────────

static LectFileHeaderV5 make_header_v5(const LECT& lect, int nn, int cap) {
    LectFileHeaderV5 hdr{};
    std::memcpy(hdr.magic, MAGIC, 8);
    hdr.version            = FORMAT_V5;
    hdr.n_nodes            = nn;
    hdr.n_dims             = lect.n_dims_;
    hdr.n_active_links     = lect.n_active_links_;
    hdr.ep_stride          = lect.ep_stride_;
    hdr.liaabb_stride      = lect.liaabb_stride_;
    hdr.split_order        = static_cast<uint8_t>(lect.split_order_);
    hdr.ep_source          = static_cast<uint8_t>(lect.ep_config_.source);
    hdr.env_type           = static_cast<uint8_t>(lect.env_config_.type);
    hdr.n_channels         = N_CHANNELS;
    hdr.capacity           = cap;
    hdr.robot_hash         = lect.robot_.fingerprint();
    hdr.tree_record_bytes  = TREE_RECORD_BYTES;
    hdr.grid_delta         = static_cast<float>(lect.env_config_.grid_config.voxel_delta);

    bool has_grids = false;
    for (int i = 0; i < nn && !has_grids; ++i) {
        if (i < static_cast<int>(lect.node_grids_.size()) &&
            !lect.node_grids_[i].empty())
            has_grids = true;
    }
    hdr.grid_section = has_grids ? 1 : 0;
    hdr.has_derived  = 1;

    // Section offsets
    size_t root_iv_size = static_cast<size_t>(lect.n_dims_) * 2 * sizeof(double);
    hdr.tree_section_off = sizeof(LectFileHeaderV5) + root_iv_size;
    hdr.ep_section_off   = hdr.tree_section_off
                         + static_cast<size_t>(cap) * TREE_RECORD_BYTES;
    // derived_off, trailer_off, grid_off: set after writing EP section
    return hdr;
}

// ── set_lect_state ──────────────────────────────────────────────────────

static void set_lect_state(LECT& lect, const Robot& robot,
                           int n_dims, int n_active_links,
                           int ep_stride, int liaabb_stride,
                           int n_nodes,
                           uint8_t split_order, uint8_t ep_source,
                           uint8_t env_type,
                           std::vector<Interval>& root_intervals) {
    lect.robot_           = robot;
    lect.n_dims_          = n_dims;
    lect.n_active_links_  = n_active_links;
    lect.ep_stride_       = ep_stride;
    lect.liaabb_stride_   = liaabb_stride;
    lect.root_intervals_  = root_intervals;
    lect.split_order_     = static_cast<SplitOrder>(split_order);
    lect.n_nodes_         = n_nodes;
    lect.capacity_        = n_nodes;
    lect.ep_config_.source = static_cast<EndpointSource>(ep_source);
    lect.env_config_.type  = static_cast<EnvelopeType>(env_type);
}

// ── rebuild_derived (fallback when no derived cache in file) ────────────

static void rebuild_derived(LECT& lect, const Robot& robot) {
    auto syms = detect_joint_symmetries(robot);
    if (!syms.empty()) lect.symmetry_q0_ = syms[0];
    lect.z4_active_ = (lect.symmetry_q0_.type == JointSymmetryType::Z4_ROTATION);

    const double* lr = robot.active_link_radii();
    lect.radii_f_.clear();
    if (lr) {
        lect.radii_f_.resize(lect.n_active_links_);
        for (int i = 0; i < lect.n_active_links_; ++i)
            lect.radii_f_[i] = static_cast<float>(lr[i]);
    }

    lect.root_fk_ = compute_fk_full(robot, lect.root_intervals_);
}

// ── Root intervals write ────────────────────────────────────────────────

static void write_root_intervals(std::ostream& out, const LECT& lect) {
    for (int d = 0; d < lect.n_dims_; ++d) {
        double lo = lect.root_intervals_[d].lo;
        double hi = lect.root_intervals_[d].hi;
        out.write(reinterpret_cast<const char*>(&lo), sizeof(double));
        out.write(reinterpret_cast<const char*>(&hi), sizeof(double));
    }
}

// ── Derived cache: root_fk_ + symmetry_q0_ + radii_f_ ──────────────────

static void write_derived_cache(std::ostream& out, const LECT& lect) {
    static constexpr char DRIV_MAGIC[4] = {'D','R','I','V'};
    out.write(DRIV_MAGIC, 4);

    // symmetry
    uint8_t sym_type = static_cast<uint8_t>(lect.symmetry_q0_.type);
    out.write(reinterpret_cast<const char*>(&sym_type), 1);
    int32_t sym_joint = lect.symmetry_q0_.joint_index;
    out.write(reinterpret_cast<const char*>(&sym_joint), 4);
    double sym_period = lect.symmetry_q0_.period;
    out.write(reinterpret_cast<const char*>(&sym_period), 8);
    uint8_t pad3[3] = {0,0,0};
    out.write(reinterpret_cast<const char*>(pad3), 3);  // align to 16B boundary

    // radii_f_
    int32_t n_radii = static_cast<int32_t>(lect.radii_f_.size());
    out.write(reinterpret_cast<const char*>(&n_radii), 4);
    if (n_radii > 0) {
        out.write(reinterpret_cast<const char*>(lect.radii_f_.data()),
                  n_radii * sizeof(float));
    }

    // root_fk_: FKState with fixed-size arrays
    const auto& fk = lect.root_fk_;
    int32_t n_tf = fk.n_tf;
    int32_t n_jm = fk.n_jm;
    uint8_t fk_valid = fk.valid ? 1 : 0;
    out.write(reinterpret_cast<const char*>(&n_tf), 4);
    out.write(reinterpret_cast<const char*>(&n_jm), 4);
    out.write(reinterpret_cast<const char*>(&fk_valid), 1);
    out.write("\0\0\0", 3);  // pad

    for (int i = 0; i < n_tf; ++i) {
        out.write(reinterpret_cast<const char*>(fk.prefix_lo[i]), 16 * sizeof(double));
        out.write(reinterpret_cast<const char*>(fk.prefix_hi[i]), 16 * sizeof(double));
    }
    for (int i = 0; i < n_jm; ++i) {
        out.write(reinterpret_cast<const char*>(fk.joints_lo[i]), 16 * sizeof(double));
        out.write(reinterpret_cast<const char*>(fk.joints_hi[i]), 16 * sizeof(double));
    }
}

static bool read_derived_cache(std::istream& in, LECT& lect) {
    char magic[4];
    in.read(magic, 4);
    if (!in.good() || std::memcmp(magic, "DRIV", 4) != 0) return false;

    uint8_t sym_type;
    in.read(reinterpret_cast<char*>(&sym_type), 1);
    int32_t sym_joint;
    in.read(reinterpret_cast<char*>(&sym_joint), 4);
    double sym_period;
    in.read(reinterpret_cast<char*>(&sym_period), 8);
    uint8_t pad3[3];
    in.read(reinterpret_cast<char*>(pad3), 3);

    lect.symmetry_q0_.type = static_cast<JointSymmetryType>(sym_type);
    lect.symmetry_q0_.joint_index = sym_joint;
    lect.symmetry_q0_.period = sym_period;
    lect.z4_active_ = (lect.symmetry_q0_.type == JointSymmetryType::Z4_ROTATION);

    int32_t n_radii;
    in.read(reinterpret_cast<char*>(&n_radii), 4);
    if (!in.good() || n_radii < 0) return false;
    lect.radii_f_.resize(n_radii);
    if (n_radii > 0) {
        in.read(reinterpret_cast<char*>(lect.radii_f_.data()),
                n_radii * sizeof(float));
    }

    // root_fk_: FKState
    int32_t n_tf, n_jm;
    uint8_t fk_valid;
    in.read(reinterpret_cast<char*>(&n_tf), 4);
    in.read(reinterpret_cast<char*>(&n_jm), 4);
    in.read(reinterpret_cast<char*>(&fk_valid), 1);
    char pad4[3];
    in.read(pad4, 3);

    if (!in.good() || n_tf < 0 || n_tf > MAX_TF || n_jm < 0 || n_jm > MAX_JOINTS)
        return false;

    lect.root_fk_.n_tf  = n_tf;
    lect.root_fk_.n_jm  = n_jm;
    lect.root_fk_.valid  = (fk_valid != 0);

    for (int i = 0; i < n_tf; ++i) {
        in.read(reinterpret_cast<char*>(lect.root_fk_.prefix_lo[i]), 16 * sizeof(double));
        in.read(reinterpret_cast<char*>(lect.root_fk_.prefix_hi[i]), 16 * sizeof(double));
    }
    for (int i = 0; i < n_jm; ++i) {
        in.read(reinterpret_cast<char*>(lect.root_fk_.joints_lo[i]), 16 * sizeof(double));
        in.read(reinterpret_cast<char*>(lect.root_fk_.joints_hi[i]), 16 * sizeof(double));
    }

    return in.good();
}

// ── Cache trailer: z4_cache_ + depth_split_dim_cache_ ───────────────────

static void write_cache_trailer(std::ostream& out, const LECT& lect) {
    // z4 cache
    int32_t n_z4 = static_cast<int32_t>(lect.z4_cache_.size());
    out.write(reinterpret_cast<const char*>(&n_z4), sizeof(n_z4));
    for (auto& [key, entry] : lect.z4_cache_) {
        uint64_t k = key;
        out.write(reinterpret_cast<const char*>(&k), sizeof(k));
        uint8_t src = static_cast<uint8_t>(entry.source);
        uint8_t ch_byte = static_cast<uint8_t>(entry.channel);
        out.write(reinterpret_cast<const char*>(&src), 1);
        out.write(reinterpret_cast<const char*>(&ch_byte), 1);
        uint8_t pad[2] = {0, 0};
        out.write(reinterpret_cast<const char*>(pad), 2);
        out.write(reinterpret_cast<const char*>(entry.ep_iaabbs.data()),
                  static_cast<std::streamsize>(lect.ep_stride_) * sizeof(float));
    }
    // depth → split-dim cache
    int32_t n_dd = static_cast<int32_t>(lect.depth_split_dim_cache_.size());
    out.write(reinterpret_cast<const char*>(&n_dd), sizeof(n_dd));
    for (auto& [depth, dim] : lect.depth_split_dim_cache_) {
        int32_t d = static_cast<int32_t>(depth);
        int32_t m = static_cast<int32_t>(dim);
        out.write(reinterpret_cast<const char*>(&d), sizeof(d));
        out.write(reinterpret_cast<const char*>(&m), sizeof(m));
    }
}

static void read_cache_trailer(std::istream& in, LECT& lect) {
    // Legacy istream path (unused — see read_cache_trailer_fast)
    (void)in; (void)lect;
}

// Fast zero-copy read directly from mmap pointer (no istream overhead)
// z4_cache_ is loaded lazily — only depth_split_dim_cache_ is read here.
static void read_cache_trailer_fast(const uint8_t* data, size_t len, LECT& lect) {
    const uint8_t* ptr = data;
    const uint8_t* end = data + len;

    // ── Skip z4 cache section (lazy: rebuilt during grow) ──
    if (ptr + 4 > end) return;
    int32_t n_z4;
    std::memcpy(&n_z4, ptr, 4); ptr += 4;
    if (n_z4 < 0 || n_z4 > 10'000'000) return;

    const size_t ep_bytes = static_cast<size_t>(lect.ep_stride_) * sizeof(float);
    const size_t entry_stride = 8 + 4 + ep_bytes;  // key(8) + src+ch+pad(4) + ep
    size_t skip = static_cast<size_t>(n_z4) * entry_stride;
    if (ptr + skip > end) return;
    ptr += skip;  // skip all z4 entries

    // ── depth → split-dim cache (small, always loaded) ──
    if (ptr + 4 > end) return;
    int32_t n_dd;
    std::memcpy(&n_dd, ptr, 4); ptr += 4;
    if (n_dd < 0 || n_dd > 100'000) return;
    if (ptr + static_cast<size_t>(n_dd) * 8 > end) return;

    for (int32_t i = 0; i < n_dd; ++i) {
        int32_t d, m;
        std::memcpy(&d, ptr, 4); ptr += 4;
        std::memcpy(&m, ptr, 4); ptr += 4;
        lect.depth_split_dim_cache_[d] = m;
    }
}

// ── Grid section write/read/index ────────────────────────────────────────

/// Write grid section.  Handles both loaded entries (in node_grids_) and
/// lazy entries (raw bytes in grid_lazy_index_) for efficient persistence
/// without requiring full grid deserialization before save.
static void write_grid_section(std::ostream& out, const LECT& lect) {
    static constexpr char GRID_MAGIC[4] = {'G','R','I','D'};
    out.write(GRID_MAGIC, 4);

    int nn = lect.n_nodes_;

    // Count loaded entries
    int32_t n_entries = 0;
    for (int i = 0; i < nn; ++i) {
        if (i < static_cast<int>(lect.node_grids_.size()) &&
            !lect.node_grids_[i].empty())
            n_entries++;
    }
    // (Lazy grid entries removed — new LECT handles grids eagerly.)
    out.write(reinterpret_cast<const char*>(&n_entries), sizeof(n_entries));

    // ── Write loaded entries ────────────────────────────────────────────
    for (int i = 0; i < nn; ++i) {
        if (i >= static_cast<int>(lect.node_grids_.size()) ||
            lect.node_grids_[i].empty())
            continue;

        const auto& grids = lect.node_grids_[i];
        const auto& metas = lect.node_grid_meta_[i];

        int32_t node_idx = static_cast<int32_t>(i);
        int32_t n_slots  = static_cast<int32_t>(grids.size());
        out.write(reinterpret_cast<const char*>(&node_idx), sizeof(node_idx));
        out.write(reinterpret_cast<const char*>(&n_slots), sizeof(n_slots));

        for (int s = 0; s < n_slots; ++s) {
            uint8_t type_byte = static_cast<uint8_t>(metas[s].type);
            uint8_t ch_byte   = metas[s].channel;
            uint8_t pad[2]    = {0, 0};
            float   delta     = metas[s].delta;
            out.write(reinterpret_cast<const char*>(&type_byte), 1);
            out.write(reinterpret_cast<const char*>(&ch_byte), 1);
            out.write(reinterpret_cast<const char*>(pad), 2);
            out.write(reinterpret_cast<const char*>(&delta), sizeof(delta));

            const auto& grid = grids[s];
            double gdelta = grid.delta();
            double gpad   = grid.safety_pad();
            out.write(reinterpret_cast<const char*>(&gdelta), sizeof(gdelta));
            out.write(reinterpret_cast<const char*>(&gpad), sizeof(gpad));
            const double* org = grid.origin();
            out.write(reinterpret_cast<const char*>(org), 3 * sizeof(double));

            int32_t n_bricks = grid.num_bricks();
            out.write(reinterpret_cast<const char*>(&n_bricks), sizeof(n_bricks));

            for (auto e : grid.bricks()) {
                int32_t bc[3] = {e.key.bx, e.key.by, e.key.bz};
                out.write(reinterpret_cast<const char*>(bc), 12);
                out.write(reinterpret_cast<const char*>(e.value.words), 64);
            }
        }
    }

    // (Lazy grid write removed — new LECT handles grids eagerly.)
}

/// Full (eager) grid section reader — still used as fallback if needed.
static bool read_grid_section(std::istream& in, LECT& lect) {
    char magic[4];
    in.read(magic, 4);
    if (!in.good() || std::memcmp(magic, "GRID", 4) != 0) return false;

    int32_t n_entries = 0;
    in.read(reinterpret_cast<char*>(&n_entries), sizeof(n_entries));
    if (!in.good() || n_entries < 0) return false;

    for (int32_t e = 0; e < n_entries; ++e) {
        int32_t node_idx, n_slots;
        in.read(reinterpret_cast<char*>(&node_idx), sizeof(node_idx));
        in.read(reinterpret_cast<char*>(&n_slots), sizeof(n_slots));
        if (!in.good() || node_idx < 0 || node_idx >= lect.n_nodes_)
            return false;

        if (node_idx >= static_cast<int>(lect.node_grids_.size())) {
            lect.node_grids_.resize(node_idx + 1);
            lect.node_grid_meta_.resize(node_idx + 1);
        }

        auto& grids = lect.node_grids_[node_idx];
        auto& metas = lect.node_grid_meta_[node_idx];
        grids.reserve(n_slots);
        metas.reserve(n_slots);

        for (int32_t s = 0; s < n_slots; ++s) {
            uint8_t type_byte, ch_byte, pad[2];
            float slot_delta;
            in.read(reinterpret_cast<char*>(&type_byte), 1);
            in.read(reinterpret_cast<char*>(&ch_byte), 1);
            in.read(reinterpret_cast<char*>(pad), 2);
            in.read(reinterpret_cast<char*>(&slot_delta), sizeof(slot_delta));

            GridSlot meta;
            meta.type    = static_cast<EnvelopeType>(type_byte);
            meta.delta   = slot_delta;
            meta.channel = ch_byte;

            double gdelta, gpad;
            double origin[3];
            in.read(reinterpret_cast<char*>(&gdelta), sizeof(gdelta));
            in.read(reinterpret_cast<char*>(&gpad), sizeof(gpad));
            in.read(reinterpret_cast<char*>(origin), sizeof(origin));

            int32_t n_bricks;
            in.read(reinterpret_cast<char*>(&n_bricks), sizeof(n_bricks));
            if (!in.good()) return false;

            voxel::SparseVoxelGrid grid(gdelta, origin[0], origin[1], origin[2], gpad);
            for (int32_t b = 0; b < n_bricks; ++b) {
                int32_t bc[3];
                in.read(reinterpret_cast<char*>(bc), 12);
                voxel::BitBrick brick;
                in.read(reinterpret_cast<char*>(brick.words), 64);
                if (!in.good()) return false;
                voxel::BrickCoord coord;
                coord.bx = bc[0]; coord.by = bc[1]; coord.bz = bc[2];
                grid.set_brick(coord, brick);
            }

            grids.push_back(std::move(grid));
            metas.push_back(meta);
        }
    }
    return true;
}

/// Build a lightweight index of the grid section without deserializing
/// bricks.  For each node with grid data, record the byte offset and
/// total length of its slot data in the lazy index.  Actual brick data
/// is deserialized on first access via ensure_node_grid_loaded().
static bool index_grid_section(const uint8_t* data, size_t len, LECT& lect) {
    const uint8_t* ptr = data;
    const uint8_t* end = data + len;

    if (ptr + 4 > end) return false;
    if (std::memcmp(ptr, "GRID", 4) != 0) return false;
    ptr += 4;

    if (ptr + 4 > end) return false;
    int32_t n_entries;
    std::memcpy(&n_entries, ptr, 4); ptr += 4;
    if (n_entries < 0) return false;
    if (n_entries == 0) return true;

    // (Lazy grid indexing removed — new LECT handles grids eagerly.)
    // Skip all entries by just reading through them.
    for (int32_t e = 0; e < n_entries; ++e) {
        if (ptr + 8 > end) return false;

        int32_t node_idx, n_slots;
        std::memcpy(&node_idx, ptr, 4); ptr += 4;
        std::memcpy(&n_slots, ptr, 4);  ptr += 4;
        if (node_idx < 0 || node_idx >= lect.n_nodes_ || n_slots < 0)
            return false;

        // Skip all slot data to compute total byte length
        for (int32_t s = 0; s < n_slots; ++s) {
            if (ptr + 52 > end) return false;
            ptr += 48;   // skip meta(8) + grid header(40)

            int32_t n_bricks;
            std::memcpy(&n_bricks, ptr, 4); ptr += 4;
            if (n_bricks < 0) return false;

            size_t brick_bytes = static_cast<size_t>(n_bricks) * 76;
            if (ptr + brick_bytes > end) return false;
            ptr += brick_bytes;
        }
    }

    fprintf(stderr, "[PLN] index_grid_section: %d entries skipped (no lazy)\n",
            n_entries);
    return true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  V5 SoA tree + EP section writers (sequential for full save)
// ═══════════════════════════════════════════════════════════════════════════

/// Write SoA tree section for @p cap slots. Only [0, nn) carry valid data.
static void write_tree_section(std::ostream& out, const LECT& lect,
                               int nn, int cap) {
    const size_t n = static_cast<size_t>(nn);
    const size_t sc = static_cast<size_t>(cap);
    const size_t pad_bytes = (sc - n) * 4;
    const size_t pad_bytes8 = (sc - n) * 8;

    // Helper: write then pad to capacity
    auto write_vec_i32 = [&](const int* data) {
        out.write(reinterpret_cast<const char*>(data), n * 4);
        if (pad_bytes > 0) {
            std::vector<char> zeros(pad_bytes, 0);
            out.write(zeros.data(), static_cast<std::streamsize>(pad_bytes));
        }
    };

    write_vec_i32(lect.left_.data());
    write_vec_i32(lect.right_.data());
    write_vec_i32(lect.parent_.data());
    write_vec_i32(lect.depth_.data());
    write_vec_i32(lect.split_dim_.data());

    // split_val: double
    out.write(reinterpret_cast<const char*>(lect.split_val_.data()), n * 8);
    if (pad_bytes8 > 0) {
        std::vector<char> zeros(pad_bytes8, 0);
        out.write(zeros.data(), static_cast<std::streamsize>(pad_bytes8));
    }

    // has_data: interleaved [safe, unsafe] per node
    {
        std::vector<uint8_t> buf(sc * 2, 0);
        for (size_t i = 0; i < n; ++i) {
            buf[i * 2]     = lect.channels_[CH_SAFE].has_data[i];
            buf[i * 2 + 1] = lect.channels_[CH_UNSAFE].has_data[i];
        }
        out.write(reinterpret_cast<const char*>(buf.data()),
                  static_cast<std::streamsize>(sc * 2));
    }

    // source_quality: same interleave
    {
        std::vector<uint8_t> buf(sc * 2, 0);
        for (size_t i = 0; i < n; ++i) {
            buf[i * 2]     = lect.channels_[CH_SAFE].source_quality[i];
            buf[i * 2 + 1] = lect.channels_[CH_UNSAFE].source_quality[i];
        }
        out.write(reinterpret_cast<const char*>(buf.data()),
                  static_cast<std::streamsize>(sc * 2));
    }
}

/// Write EP section for @p cap slots. Only [0, nn) carry valid data.
static void write_ep_section(std::ostream& out, const LECT& lect,
                             int nn, int cap) {
    const int ep = lect.ep_stride_;
    const size_t ep_bytes = static_cast<size_t>(ep) * sizeof(float);
    for (int i = 0; i < nn; ++i) {
        out.write(reinterpret_cast<const char*>(lect.ep_data_read(i, CH_SAFE)),
                  static_cast<std::streamsize>(ep_bytes));
        out.write(reinterpret_cast<const char*>(lect.ep_data_read(i, CH_UNSAFE)),
                  static_cast<std::streamsize>(ep_bytes));
    }
    // Pad remaining [nn, cap) with zeros
    if (cap > nn) {
        size_t pad_total = static_cast<size_t>(cap - nn) * ep_node_bytes(ep);
        // Write in 64KB chunks to avoid huge alloc
        constexpr size_t chunk = 65536;
        std::vector<char> zeros(std::min(pad_total, chunk), 0);
        for (size_t remaining = pad_total; remaining > 0; ) {
            size_t write_sz = std::min(remaining, chunk);
            out.write(zeros.data(), static_cast<std::streamsize>(write_sz));
            remaining -= write_sz;
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  V5 SoA mmap-backed loader
// ═══════════════════════════════════════════════════════════════════════════

static bool load_v5_mmap(LECT& lect, const Robot& robot,
                         LectMmap& mmap, const LectFileHeaderV5& hdr) {
    using Clock = std::chrono::steady_clock;
    auto t0 = Clock::now();

    const int nn  = hdr.n_nodes;
    const int cap = hdr.capacity;
    const int ep  = hdr.ep_stride;
    const size_t ep_ns = ep_node_bytes(ep);
    const size_t tree_off = hdr.tree_section_off;
    const size_t ep_off   = hdr.ep_section_off;

    // Validate: file must cover header + root + tree + EP(nn nodes at minimum)
    size_t expected_min = ep_off + static_cast<size_t>(nn) * ep_ns;
    if (mmap.size() < expected_min) return false;

    // ── Read root intervals ─────────────────────────────────────────────
    const uint8_t* root_ptr = mmap.data() + sizeof(LectFileHeaderV5);
    std::vector<Interval> root_iv(hdr.n_dims);
    for (int d = 0; d < hdr.n_dims; ++d) {
        double lo, hi;
        std::memcpy(&lo, root_ptr + d * 16, 8);
        std::memcpy(&hi, root_ptr + d * 16 + 8, 8);
        root_iv[d] = {lo, hi};
    }

    set_lect_state(lect, robot, hdr.n_dims, hdr.n_active_links,
                   ep, hdr.liaabb_stride, nn,
                   hdr.split_order, hdr.ep_source, hdr.env_type, root_iv);
    lect.env_config_.grid_config.voxel_delta = static_cast<double>(hdr.grid_delta);

    // Clear stale link_iaabb_cache_ from fresh construction (if the LECT was
    // constructed before load).  The lazy alloc in get_link_iaabbs() will
    // re-allocate at the correct capacity_ (= nn) on first use.
    lect.link_iaabb_cache_.clear();
    lect.link_iaabb_cache_.shrink_to_fit();

    auto t1 = Clock::now();

    // ── Zero-copy tree section via set_mmap (COW) ─────────────────────────
    // Instead of bulk-memcpy'ing ~17 MB of tree data into vectors, point
    // TreeArray at the mmap region.  MAP_PRIVATE gives copy-on-write
    // semantics: reads are free; writes (e.g. repair or tree expansion)
    // trigger per-page COW.  On first growth that exceeds mmap capacity,
    // ensure_capacity() materializes to vectors automatically.
    uint8_t* tb_w = mmap.writable_data() + tree_off;
    const uint8_t* tb = mmap.data() + tree_off;
    madvise(const_cast<uint8_t*>(tb),
            static_cast<size_t>(cap) * TREE_RECORD_BYTES,
            MADV_WILLNEED);

    const size_t n = static_cast<size_t>(nn);
    const size_t sc = static_cast<size_t>(cap);  // SoA array stride for file

    // Check alignment: split_val_ is double*, needs 8-byte alignment.
    const size_t sv_byte_off = 5 * sc * 4;
    const bool sv_aligned = ((sv_byte_off & 7) == 0);

    lect.left_.set_mmap(reinterpret_cast<int*>(tb_w + 0 * sc * 4));
    lect.right_.set_mmap(reinterpret_cast<int*>(tb_w + 1 * sc * 4));
    lect.parent_.set_mmap(reinterpret_cast<int*>(tb_w + 2 * sc * 4));
    lect.depth_.set_mmap(reinterpret_cast<int*>(tb_w + 3 * sc * 4));
    lect.split_dim_.set_mmap(reinterpret_cast<int*>(tb_w + 4 * sc * 4));
    if (sv_aligned) {
        lect.split_val_.set_mmap(reinterpret_cast<double*>(tb_w + sv_byte_off));
    } else {
        // Odd cap → misaligned double*. Fall back to memcpy.
        lect.split_val_.resize(nn);
        std::memcpy(lect.split_val_.data(), tb + sv_byte_off, n * 8);
    }

    for (int ch = 0; ch < N_CHANNELS; ++ch) {
        lect.channels_[ch].has_data.resize(nn, 0);
        lect.channels_[ch].source_quality.resize(nn, 0);
    }

    // ── Validate tree: repair dangling child pointers ───────────────────
    // A previous crash during async incremental save may leave parent nodes
    // with child indices >= n_nodes (tree data was written but the header's
    // n_nodes was not updated before the process died).  Detect and reset
    // such dangling pointers to make the node a leaf again.
    {
        int n_repaired = 0;
        for (int i = 0; i < nn; ++i) {
            int li = lect.left_[i];
            int ri = lect.right_[i];
            if ((li >= 0 && li >= nn) || (ri >= 0 && ri >= nn)) {
                lect.left_[i]  = -1;
                lect.right_[i] = -1;
                lect.split_dim_[i] = -1;
                lect.split_val_[i] = 0.0;
                ++n_repaired;
            }
        }
        if (n_repaired > 0) {
            fprintf(stderr, "[PLN] lect_load: repaired %d nodes with dangling "
                    "child pointers (>= n_nodes=%d)\n", n_repaired, nn);
        }
    }

    // Record mmap tree section metadata for snapshot() zero-copy path
    lect.mmap_tree_off_ = tree_off;
    lect.mmap_tree_cap_ = cap;

    // has_data: interleaved [safe, unsafe] per node at offset 5*sc*4 + sc*8
    const uint8_t* hd_ptr = tb + 5 * sc * 4 + sc * 8;
    for (size_t i = 0; i < n; ++i) {
        lect.channels_[CH_SAFE].has_data[i]   = hd_ptr[i * 2];
        lect.channels_[CH_UNSAFE].has_data[i] = hd_ptr[i * 2 + 1];
    }

    // source_quality: same layout at offset 5*sc*4 + sc*8 + sc*2
    const uint8_t* sq_ptr = hd_ptr + sc * 2;
    for (size_t i = 0; i < n; ++i) {
        lect.channels_[CH_SAFE].source_quality[i]   = sq_ptr[i * 2];
        lect.channels_[CH_UNSAFE].source_quality[i] = sq_ptr[i * 2 + 1];
    }

    auto t2 = Clock::now();

    // ── mmap-backed EP section (zero-copy COW) ──────────────────────────
    lect.mmap_         = std::move(mmap);
    lect.mmap_ep_base_ = lect.mmap_.writable_data() + ep_off;
    lect.mmap_node_stride_ = static_cast<int>(ep_ns);
    lect.nn_loaded_    = nn;
    lect.use_mmap_     = true;

    // ── Lazy arrays: NOT allocated ──────────────────────────────────────
    // link_iaabb_cache_: empty → allocated on first get_link_iaabbs()
    // node_grids_/node_grid_meta_: empty → populated from grid section
    lect.link_iaabb_dirty_.assign(nn, 1);
    lect.forest_id_.resize(nn);
    lect.subtree_occ_.resize(nn);
    // capacity_ stays = nn (from set_lect_state); ensure_capacity handles growth.
    std::fill(lect.forest_id_.begin(), lect.forest_id_.end(), -1);
    std::fill(lect.subtree_occ_.begin(), lect.subtree_occ_.end(), 0);

    auto t3 = Clock::now();

    // ── Read derived cache (or fallback to recomputation) ───────────────
    if (hdr.has_derived && hdr.derived_off > 0 && hdr.derived_off < lect.mmap_.size()) {
        size_t remaining = lect.mmap_.size() - hdr.derived_off;
        MemBuf mbuf(reinterpret_cast<const char*>(lect.mmap_.data() + hdr.derived_off),
                    remaining);
        std::istream din(&mbuf);
        if (!read_derived_cache(din, lect)) {
            rebuild_derived(lect, robot);
        }
    } else {
        rebuild_derived(lect, robot);
    }

    auto t4 = Clock::now();

    // ── Read cache trailer + grid section (lazy) ──────────────────────────
    if (hdr.trailer_off > 0 && hdr.trailer_off < lect.mmap_.size()) {
        size_t remaining = lect.mmap_.size() - hdr.trailer_off;
        read_cache_trailer_fast(lect.mmap_.data() + hdr.trailer_off,
                                remaining, lect);

        if (hdr.grid_section && hdr.grid_off > 0 && hdr.grid_off < lect.mmap_.size()) {
            size_t grem = lect.mmap_.size() - hdr.grid_off;
            // Lazy: build an index of grid entries without deserializing
            // bricks.  Actual deserialization happens on first access.
            index_grid_section(lect.mmap_.data() + hdr.grid_off, grem, lect);
        }
    }

    auto t5 = Clock::now();

    // ── Timing ──────────────────────────────────────────────────────────
    auto ms = [](auto a, auto b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    fprintf(stderr, "[PLN] lect_v5_load: header=%.1fms tree_mmap=%.1fms "
            "mmap+lazy=%.1fms derived=%.1fms trailer+grid_idx=%.1fms total=%.1fms\n",
            ms(t0, t1), ms(t1, t2), ms(t2, t3), ms(t3, t4), ms(t4, t5),
            ms(t0, t5));

    return true;
}

};  // struct LectIOHelper

// ═══════════════════════════════════════════════════════════════════════════
//  lect_save_binary — full V5 SoA save
// ═══════════════════════════════════════════════════════════════════════════

bool lect_save_binary(const LECT& lect, const std::string& path) {
    const int nn = lect.n_nodes_;
    if (nn <= 0) return false;

    // Materialize mmap data before potentially truncating the file
    lect.materialize_mmap();

    std::ofstream out(path, std::ios::binary);
    if (!out.is_open()) return false;

    // Capacity: nn + 25% headroom for future incremental growth.
    // Ensure cap is even so split_val (double*) is always 8-byte aligned
    // in mmap SoA layout (split_val starts at offset 5*cap*4 bytes).
    int cap = nn + std::max(nn / 4, 4096);
    cap += (cap & 1);  // round up to even

    auto hdr = LectIOHelper::make_header_v5(lect, nn, cap);

    // Write header (placeholder — rewritten at end with final offsets)
    out.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
    if (!out.good()) return false;

    // Root intervals
    LectIOHelper::write_root_intervals(out, lect);

    // Tree section (SoA, padded to cap)
    LectIOHelper::write_tree_section(out, lect, nn, cap);

    // EP section (padded to cap)
    LectIOHelper::write_ep_section(out, lect, nn, cap);

    // Derived cache
    hdr.derived_off = static_cast<uint64_t>(out.tellp());
    LectIOHelper::write_derived_cache(out, lect);

    // Cache trailer
    hdr.trailer_off = static_cast<uint64_t>(out.tellp());
    LectIOHelper::write_cache_trailer(out, lect);

    // Grid section
    if (hdr.grid_section) {
        hdr.grid_off = static_cast<uint64_t>(out.tellp());
        LectIOHelper::write_grid_section(out, lect);
    }

    // Rewrite header with final offsets
    out.seekp(0);
    out.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));

    return out.good();
}

// ═══════════════════════════════════════════════════════════════════════════
//  lect_save_incremental — O(Δ) update within capacity gap
// ═══════════════════════════════════════════════════════════════════════════

bool lect_save_incremental(const LECT& lect, const std::string& path,
                           int old_n_nodes) {
    const int nn = lect.n_nodes_;
    if (nn <= 0) return false;

    // Materialize mmap data before file modification
    lect.materialize_mmap();

    if (old_n_nodes <= 0)
        return lect_save_binary(lect, path);

    // If no new nodes, just rewrite trailing sections
    if (old_n_nodes >= nn) {
        // Probe for V5 format
        std::ifstream probe(path, std::ios::binary);
        if (probe.is_open()) {
            LectFileHeaderV5 old_hdr{};
            probe.read(reinterpret_cast<char*>(&old_hdr), sizeof(old_hdr));
            if (probe.good() && old_hdr.version == FORMAT_V5)
                return true;  // nothing changed
        }
        return lect_save_binary(lect, path);
    }

    // Read old header — must be V5 with enough capacity
    LectFileHeaderV5 old_hdr{};
    {
        std::ifstream probe(path, std::ios::binary);
        if (!probe.is_open()) return lect_save_binary(lect, path);
        probe.read(reinterpret_cast<char*>(&old_hdr), sizeof(old_hdr));
        if (!probe.good() || old_hdr.version != FORMAT_V5)
            return lect_save_binary(lect, path);
    }

    // Need more capacity? Full rewrite.
    if (nn > old_hdr.capacity)
        return lect_save_binary(lect, path);

    // ── Incremental update within existing capacity ─────────────────────
    std::fstream fs(path, std::ios::in | std::ios::out | std::ios::binary);
    if (!fs.is_open())
        return lect_save_binary(lect, path);

    const int cap = old_hdr.capacity;
    const size_t sc = static_cast<size_t>(cap);
    const size_t toff = old_hdr.tree_section_off;
    const int ep = lect.ep_stride_;
    const size_t ep_ns = ep_node_bytes(ep);

    // Helper: write one node's tree fields at SoA positions
    auto write_node_tree = [&](int i) {
        int32_t v;
        v = lect.left_[i];
        fs.seekp(static_cast<std::streamoff>(toff + 0 * sc * 4 + i * 4));
        fs.write(reinterpret_cast<const char*>(&v), 4);
        v = lect.right_[i];
        fs.seekp(static_cast<std::streamoff>(toff + 1 * sc * 4 + i * 4));
        fs.write(reinterpret_cast<const char*>(&v), 4);
        v = lect.parent_[i];
        fs.seekp(static_cast<std::streamoff>(toff + 2 * sc * 4 + i * 4));
        fs.write(reinterpret_cast<const char*>(&v), 4);
        v = lect.depth_[i];
        fs.seekp(static_cast<std::streamoff>(toff + 3 * sc * 4 + i * 4));
        fs.write(reinterpret_cast<const char*>(&v), 4);
        v = lect.split_dim_[i];
        fs.seekp(static_cast<std::streamoff>(toff + 4 * sc * 4 + i * 4));
        fs.write(reinterpret_cast<const char*>(&v), 4);

        double sv = lect.split_val_[i];
        fs.seekp(static_cast<std::streamoff>(toff + 5 * sc * 4 + i * 8));
        fs.write(reinterpret_cast<const char*>(&sv), 8);

        uint8_t hd[2] = { lect.channels_[CH_SAFE].has_data[i],
                           lect.channels_[CH_UNSAFE].has_data[i] };
        fs.seekp(static_cast<std::streamoff>(toff + 5 * sc * 4 + sc * 8 + i * 2));
        fs.write(reinterpret_cast<const char*>(hd), 2);

        uint8_t sq[2] = { lect.channels_[CH_SAFE].source_quality[i],
                           lect.channels_[CH_UNSAFE].source_quality[i] };
        fs.seekp(static_cast<std::streamoff>(toff + 5 * sc * 4 + sc * 8 + sc * 2 + i * 2));
        fs.write(reinterpret_cast<const char*>(sq), 2);
    };

    auto write_node_ep = [&](int i) {
        fs.seekp(static_cast<std::streamoff>(old_hdr.ep_section_off + static_cast<size_t>(i) * ep_ns));
        fs.write(reinterpret_cast<const char*>(lect.ep_data_read(i, CH_SAFE)),
                 static_cast<std::streamsize>(ep * sizeof(float)));
        fs.write(reinterpret_cast<const char*>(lect.ep_data_read(i, CH_UNSAFE)),
                 static_cast<std::streamsize>(ep * sizeof(float)));
    };

    // ── Crash-safe: update n_nodes in header FIRST ────────────────────────
    // If the process crashes during data writes, the header already has the
    // correct n_nodes.  On next load, tree validation will repair any
    // partially-written nodes (dangling child pointers).
    {
        int32_t nn32 = static_cast<int32_t>(nn);
        fs.seekp(12);  // offset of n_nodes in LectFileHeaderV5
        fs.write(reinterpret_cast<const char*>(&nn32), 4);
        fs.flush();
    }

    // Patch modified old nodes (split nodes whose children are new)
    for (int i = 0; i < old_n_nodes; ++i) {
        if (lect.left_[i] >= old_n_nodes) {
            write_node_tree(i);
            write_node_ep(i);
        }
    }

    // Write new nodes
    for (int i = old_n_nodes; i < nn; ++i) {
        write_node_tree(i);
        write_node_ep(i);
    }

    // ── Rewrite trailing sections ───────────────────────────────────────
    auto hdr = LectIOHelper::make_header_v5(lect, nn, cap);
    hdr.tree_section_off = old_hdr.tree_section_off;
    hdr.ep_section_off   = old_hdr.ep_section_off;

    size_t after_ep = old_hdr.ep_section_off + static_cast<size_t>(cap) * ep_ns;
    fs.seekp(static_cast<std::streamoff>(after_ep));

    hdr.derived_off = static_cast<uint64_t>(fs.tellp());
    LectIOHelper::write_derived_cache(fs, lect);

    hdr.trailer_off = static_cast<uint64_t>(fs.tellp());
    LectIOHelper::write_cache_trailer(fs, lect);

    if (hdr.grid_section) {
        hdr.grid_off = static_cast<uint64_t>(fs.tellp());
        LectIOHelper::write_grid_section(fs, lect);
    }

    auto end_pos = fs.tellp();

    // Update header
    fs.seekp(0);
    fs.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
    fs.close();

    // Truncate file if it shrank
    std::error_code ec;
    std::filesystem::resize_file(path, static_cast<std::uintmax_t>(end_pos), ec);
    return !ec;
}

// ═══════════════════════════════════════════════════════════════════════════
//  lect_load_binary — V5 SoA mmap-only entry point
// ═══════════════════════════════════════════════════════════════════════════

bool lect_load_binary(LECT& lect, const Robot& robot, const std::string& path) {
    LectMmap mmap;
    if (!mmap.open(path)) return false;
    if (mmap.size() < sizeof(LectFileHeaderV5)) return false;

    LectFileHeaderV5 hdr{};
    std::memcpy(&hdr, mmap.data(), sizeof(hdr));

    if (std::memcmp(hdr.magic, MAGIC, 8) != 0) return false;
    if (hdr.version != FORMAT_V5)              return false;
    if (hdr.n_dims != robot.n_joints())        return false;
    if (hdr.n_active_links != robot.n_active_links()) return false;
    if (hdr.robot_hash != 0 && hdr.robot_hash != robot.fingerprint())
        return false;
    if (hdr.ep_stride != hdr.n_active_links * 2 * 6) return false;
    if (hdr.liaabb_stride != hdr.n_active_links * 6)  return false;
    if (hdr.n_nodes <= 0) return false;

    return LectIOHelper::load_v5_mmap(lect, robot, mmap, hdr);
}

// ═══════════════════════════════════════════════════════════════════════════
//  V6 tree-only format
// ═══════════════════════════════════════════════════════════════════════════
//
// Layout:
//   [Header 128B]               (magic "SBF6LECT", version=6)
//   [Root intervals: n_dims×16]
//   [Tree SoA section]          (left, right, parent, depth, split_dim, split_val)
//   [Derived cache]             (root_fk_, symmetry, radii)
//   [Depth split dim cache]     (entries count + (depth, dim) pairs)
//
// No EP data, no has_data/source_quality, no grids, no z4_cache.
// EP and grid data lives in .ep_cache and .grid.*.cache managed by
// LectCacheManager.

static constexpr char MAGIC_V6[8] = {'S','B','F','6','L','E','C','T'};
static constexpr uint32_t FORMAT_V6 = 6;

struct LectFileHeaderV6 {
    char     magic[8];          //   0
    uint32_t version;           //   8
    int32_t  n_nodes;           //  12
    int32_t  n_dims;            //  16
    int32_t  n_active_links;    //  20
    int32_t  ep_stride;         //  24
    int32_t  liaabb_stride;     //  28
    uint8_t  split_order;       //  32
    uint8_t  ep_source;         //  33
    uint8_t  env_type;          //  34
    uint8_t  reserved1;         //  35
    int32_t  capacity;          //  36
    uint64_t robot_hash;        //  40
    float    grid_delta;        //  48
    uint32_t reserved2;         //  52
    uint64_t tree_section_off;  //  56
    uint64_t derived_off;       //  64
    uint64_t depth_cache_off;   //  72
    uint8_t  reserved3[48];     //  80-127
};
static_assert(sizeof(LectFileHeaderV6) == 128, "V6 header must be 128 bytes");

// ── V6 Save ─────────────────────────────────────────────────────────────────

bool lect_save_v6(const LECT& lect, const std::string& path) {
    const int nn = lect.n_nodes_;
    if (nn <= 0) return false;

    // Materialize any mmap-backed data into writable vectors
    lect.materialize_mmap();

    std::ofstream out(path, std::ios::binary);
    if (!out.is_open()) return false;

    int cap = nn + std::max(nn / 4, 4096);
    cap += (cap & 1);  // even cap for double alignment

    LectFileHeaderV6 hdr{};
    std::memcpy(hdr.magic, MAGIC_V6, 8);
    hdr.version         = FORMAT_V6;
    hdr.n_nodes         = nn;
    hdr.n_dims          = lect.n_dims_;
    hdr.n_active_links  = lect.n_active_links_;
    hdr.ep_stride       = lect.ep_stride_;
    hdr.liaabb_stride   = lect.liaabb_stride_;
    hdr.split_order     = static_cast<uint8_t>(lect.split_order_);
    hdr.ep_source       = static_cast<uint8_t>(lect.ep_config_.source);
    hdr.env_type        = static_cast<uint8_t>(lect.env_config_.type);
    hdr.capacity        = cap;
    hdr.robot_hash      = lect.robot_.fingerprint();
    hdr.grid_delta      = static_cast<float>(lect.env_config_.grid_config.voxel_delta);

    size_t root_iv_bytes = static_cast<size_t>(lect.n_dims_) * 2 * sizeof(double);
    hdr.tree_section_off = sizeof(LectFileHeaderV6) + root_iv_bytes;

    // Write header placeholder
    out.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));

    // Root intervals
    LectIOHelper::write_root_intervals(out, lect);

    // Tree SoA (left, right, parent, depth, split_dim, split_val)
    // Same format as V5 tree section, just without has_data/source_quality
    const size_t n = static_cast<size_t>(nn);
    const size_t sc = static_cast<size_t>(cap);
    const size_t pad4 = (sc - n) * 4;
    const size_t pad8 = (sc - n) * 8;

    auto write_i32 = [&](const int* data) {
        out.write(reinterpret_cast<const char*>(data), n * 4);
        if (pad4 > 0) {
            std::vector<char> z(pad4, 0);
            out.write(z.data(), static_cast<std::streamsize>(pad4));
        }
    };

    write_i32(lect.left_.data());
    write_i32(lect.right_.data());
    write_i32(lect.parent_.data());
    write_i32(lect.depth_.data());
    write_i32(lect.split_dim_.data());

    out.write(reinterpret_cast<const char*>(lect.split_val_.data()), n * 8);
    if (pad8 > 0) {
        std::vector<char> z(pad8, 0);
        out.write(z.data(), static_cast<std::streamsize>(pad8));
    }

    // Derived cache
    hdr.derived_off = static_cast<uint64_t>(out.tellp());
    LectIOHelper::write_derived_cache(out, lect);

    // Depth → split-dim cache
    hdr.depth_cache_off = static_cast<uint64_t>(out.tellp());
    {
        int32_t n_dd = static_cast<int32_t>(lect.depth_split_dim_cache_.size());
        out.write(reinterpret_cast<const char*>(&n_dd), 4);
        for (auto& [depth, dim] : lect.depth_split_dim_cache_) {
            int32_t d = static_cast<int32_t>(depth);
            int32_t m = static_cast<int32_t>(dim);
            out.write(reinterpret_cast<const char*>(&d), 4);
            out.write(reinterpret_cast<const char*>(&m), 4);
        }
    }

    // Rewrite header with final offsets
    out.seekp(0);
    out.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));

    fprintf(stderr, "[PLN] lect_save_v6: %d nodes, cap=%d, file≈%.1f KB\n",
            nn, cap, static_cast<double>(out.tellp()) / 1024.0);

    return out.good();
}

// ── V6 Load ─────────────────────────────────────────────────────────────────

bool lect_load_v6(LECT& lect, const Robot& robot, const std::string& path) {
    using Clock = std::chrono::steady_clock;
    auto t0 = Clock::now();

    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in.is_open()) return false;
    size_t file_size = static_cast<size_t>(in.tellg());
    if (file_size < sizeof(LectFileHeaderV6)) return false;
    in.seekg(0);

    LectFileHeaderV6 hdr{};
    in.read(reinterpret_cast<char*>(&hdr), sizeof(hdr));
    if (!in.good()) return false;

    if (std::memcmp(hdr.magic, MAGIC_V6, 8) != 0) return false;
    if (hdr.version != FORMAT_V6) return false;
    if (hdr.n_dims != robot.n_joints()) return false;
    if (hdr.n_active_links != robot.n_active_links()) return false;
    if (hdr.robot_hash != 0 && hdr.robot_hash != robot.fingerprint()) return false;
    if (hdr.n_nodes <= 0) return false;

    const int nn  = hdr.n_nodes;
    const int cap = hdr.capacity;
    const size_t n = static_cast<size_t>(nn);
    const size_t sc = static_cast<size_t>(cap);

    // Read root intervals
    std::vector<Interval> root_iv(hdr.n_dims);
    for (int d = 0; d < hdr.n_dims; ++d) {
        double lo, hi;
        in.read(reinterpret_cast<char*>(&lo), 8);
        in.read(reinterpret_cast<char*>(&hi), 8);
        root_iv[d] = {lo, hi};
    }
    if (!in.good()) return false;

    LectIOHelper::set_lect_state(lect, robot, hdr.n_dims, hdr.n_active_links,
                                  hdr.ep_stride, hdr.liaabb_stride, nn,
                                  hdr.split_order, hdr.ep_source, hdr.env_type,
                                  root_iv);
    lect.env_config_.grid_config.voxel_delta = static_cast<double>(hdr.grid_delta);

    // Allocate tree vectors
    lect.left_.resize(nn);
    lect.right_.resize(nn);
    lect.parent_.resize(nn);
    lect.depth_.resize(nn);
    lect.split_dim_.resize(nn);
    lect.split_val_.resize(nn);

    // Read tree SoA
    auto read_i32 = [&](int* dst) {
        in.read(reinterpret_cast<char*>(dst), n * 4);
        if (sc > n) in.seekg(static_cast<std::streamoff>((sc - n) * 4), std::ios::cur);
    };
    read_i32(lect.left_.data());
    read_i32(lect.right_.data());
    read_i32(lect.parent_.data());
    read_i32(lect.depth_.data());
    read_i32(lect.split_dim_.data());

    in.read(reinterpret_cast<char*>(lect.split_val_.data()), n * 8);
    if (sc > n) in.seekg(static_cast<std::streamoff>((sc - n) * 8), std::ios::cur);

    if (!in.good()) return false;

    // No EP data — channels are empty (has_data = 0 for all)
    for (int ch = 0; ch < N_CHANNELS; ++ch) {
        lect.channels_[ch].has_data.assign(nn, 0);
        lect.channels_[ch].source_quality.assign(nn, 0);
        lect.channels_[ch].ep_data.resize(static_cast<size_t>(nn) * hdr.ep_stride, 0.0f);
    }
    lect.use_mmap_ = false;
    lect.nn_loaded_ = 0;
    lect.mmap_ep_base_ = nullptr;
    lect.mmap_node_stride_ = 0;
    lect.mmap_tree_off_ = 0;
    lect.mmap_tree_cap_ = 0;

    // Link iAABB cache — lazy
    lect.link_iaabb_cache_.clear();
    lect.link_iaabb_cache_.shrink_to_fit();
    lect.link_iaabb_dirty_.assign(nn, 1);

    // Occupation — fresh
    lect.forest_id_.assign(nn, -1);
    lect.subtree_occ_.assign(nn, 0);

    // Read derived cache
    if (hdr.derived_off > 0 && hdr.derived_off < file_size) {
        in.seekg(static_cast<std::streamoff>(hdr.derived_off));
        if (!LectIOHelper::read_derived_cache(in, lect)) {
            LectIOHelper::rebuild_derived(lect, robot);
        }
    } else {
        LectIOHelper::rebuild_derived(lect, robot);
    }

    // Read depth → split-dim cache
    if (hdr.depth_cache_off > 0 && hdr.depth_cache_off < file_size) {
        in.seekg(static_cast<std::streamoff>(hdr.depth_cache_off));
        int32_t n_dd;
        in.read(reinterpret_cast<char*>(&n_dd), 4);
        if (in.good() && n_dd >= 0 && n_dd < 100000) {
            for (int32_t i = 0; i < n_dd; ++i) {
                int32_t d, m;
                in.read(reinterpret_cast<char*>(&d), 4);
                in.read(reinterpret_cast<char*>(&m), 4);
                lect.depth_split_dim_cache_[d] = m;
            }
        }
    }

    auto t1 = Clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    fprintf(stderr, "[PLN] lect_load_v6: %d nodes, cap=%d, %.1fms (tree-only, no EP)\n",
            nn, cap, ms);

    return true;
}

}  // namespace sbf
