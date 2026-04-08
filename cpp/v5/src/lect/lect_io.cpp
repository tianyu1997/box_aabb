// SafeBoxForest v5 — LECT binary cache IO (V4 dual-channel + grid format)
//
// V4 layout: Dual-channel AoS with optional grid section.
//   - Header (64 bytes): extended from V3 48-byte header
//   - Root intervals (n_dims × 2 × double)
//   - Per-node AoS records: tree struct + 2 channels of ep_data
//   - Cache trailer: z4_cache + depth_split_dim_cache
//   - Grid section: per-node sparse brick lists
//   - Backward compatible: reads V1, V2, V3 files (into CH_SAFE)
//
// V4 per-node record:
//   [0..19]   int32 left, right, parent, depth, split_dim
//   [20..21]  uint8 has_data[2]   (CH_SAFE, CH_UNSAFE)
//   [22..23]  uint8 source_quality[2]
//   [24..31]  double split_val
//   [32..]    float ep_data_safe[ep_stride]
//   [32+ep*4] float ep_data_unsafe[ep_stride]
//
// V3 per-node record (legacy):
//   [0..19]  int32 left, right, parent, depth, split_dim
//   [20]     uint8 has_data
//   [21]     uint8 source_quality
//   [22..23] reserved (pad)
//   [24..31] double split_val
//   [32..]   float ep_data[ep_stride]

#include <sbf/lect/lect_io.h>
#include <sbf/core/fk_state.h>
#include <sbf/core/joint_symmetry.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/link_iaabb.h>
#include <sbf/voxel/voxel_grid.h>

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <vector>

namespace sbf {

// ═══════════════════════════════════════════════════════════════════════════
//  Binary format constants
// ═══════════════════════════════════════════════════════════════════════════

static constexpr char MAGIC[8] = {'S','B','F','5','L','E','C','T'};
static constexpr uint32_t FORMAT_V3 = 3;  // AoS single-channel (legacy)
static constexpr uint32_t FORMAT_V4 = 4;  // AoS dual-channel + grids

// ═════════════════════════════════════════════════════════════════════════════
//  V3 Header layout (48 bytes — V1/V2/V3)
// ═════════════════════════════════════════════════════════════════════════════

struct LectFileHeader {
    char     magic[8];          //  0: "SBF5LECT"
    uint32_t version;           //  8: 1 | 2 | 3 | 4
    int32_t  n_nodes;           // 12: active node count
    int32_t  n_dims;            // 16
    int32_t  n_active_links;    // 20
    int32_t  ep_stride;         // 24: floats per node (n_active_links * 2 * 6)
    int32_t  liaabb_stride;     // 28: n_active_links * 6
    uint8_t  split_order;       // 32
    uint8_t  ep_source;         // 33
    uint8_t  env_type;          // 34
    uint8_t  reserved1;         // 35
    int32_t  capacity;          // 36: pre-allocated node slots in file
    uint64_t robot_hash;        // 40: FNV-1a Robot::fingerprint()
};

static_assert(sizeof(LectFileHeader) == 48, "Header must be 48 bytes");

// ═════════════════════════════════════════════════════════════════════════════
//  V4 Header layout (64 bytes — extends V3)
// ═════════════════════════════════════════════════════════════════════════════

struct LectFileHeaderV4 {
    char     magic[8];          //  0: "SBF5LECT"
    uint32_t version;           //  8: 4
    int32_t  n_nodes;           // 12
    int32_t  n_dims;            // 16
    int32_t  n_active_links;    // 20
    int32_t  ep_stride;         // 24
    int32_t  liaabb_stride;     // 28
    uint8_t  split_order;       // 32
    uint8_t  ep_source;         // 33
    uint8_t  env_type;          // 34
    uint8_t  n_channels;        // 35: always 2
    int32_t  capacity;          // 36
    uint64_t robot_hash;        // 40
    uint8_t  grid_section;      // 48: 0=no grids, 1=has grid section
    uint8_t  reserved2[3];      // 49-51
    float    grid_delta;        // 52: default grid voxel delta
    uint8_t  reserved3[8];      // 56-63
};

static_assert(sizeof(LectFileHeaderV4) == 64, "V4 Header must be 64 bytes");

// ═══════════════════════════════════════════════════════════════════════════
//  AoS node record helpers (via LectIOHelper — friend of LECT)
// ═══════════════════════════════════════════════════════════════════════════

struct LectIOHelper {

// ── V4 per-node record stride (dual-channel) ───────────────────────────
static int stride_v4(int ep_stride) {
    // 32 bytes tree struct + 2 × ep_stride×4 bytes ep_data
    return 32 + 2 * ep_stride * static_cast<int>(sizeof(float));
}

// ── V3 per-node record stride (single-channel, legacy) ─────────────────
static int stride_v3(int ep_stride) {
    return 32 + ep_stride * static_cast<int>(sizeof(float));
}

static size_t doff_v4(int n_dims) {
    return sizeof(LectFileHeaderV4) + n_dims * 2 * sizeof(double);
}

static size_t doff_v3(int n_dims) {
    return sizeof(LectFileHeader) + n_dims * 2 * sizeof(double);
}

// ── V4 pack/unpack ──────────────────────────────────────────────────────
static void pack_node_v4(uint8_t* buf, const LECT& lect, int i) {
    int32_t fields[5] = {
        static_cast<int32_t>(lect.left_[i]),
        static_cast<int32_t>(lect.right_[i]),
        static_cast<int32_t>(lect.parent_[i]),
        static_cast<int32_t>(lect.depth_[i]),
        static_cast<int32_t>(lect.split_dim_[i])
    };
    std::memcpy(buf, fields, 20);
    buf[20] = lect.channels_[CH_SAFE].has_data[i];
    buf[21] = lect.channels_[CH_UNSAFE].has_data[i];
    buf[22] = lect.channels_[CH_SAFE].source_quality[i];
    buf[23] = lect.channels_[CH_UNSAFE].source_quality[i];
    double sv = lect.split_val_[i];
    std::memcpy(buf + 24, &sv, 8);
    int ep = lect.ep_stride_;
    if (ep > 0) {
        std::memcpy(buf + 32,
                    lect.channels_[CH_SAFE].ep_data.data() + i * ep,
                    ep * sizeof(float));
        std::memcpy(buf + 32 + ep * sizeof(float),
                    lect.channels_[CH_UNSAFE].ep_data.data() + i * ep,
                    ep * sizeof(float));
    }
}

static void unpack_node_v4(const uint8_t* buf, LECT& lect, int i) {
    int32_t fields[5];
    std::memcpy(fields, buf, 20);
    lect.left_[i]      = fields[0];
    lect.right_[i]     = fields[1];
    lect.parent_[i]    = fields[2];
    lect.depth_[i]     = fields[3];
    lect.split_dim_[i] = fields[4];
    lect.channels_[CH_SAFE].has_data[i]        = buf[20];
    lect.channels_[CH_UNSAFE].has_data[i]      = buf[21];
    lect.channels_[CH_SAFE].source_quality[i]  = buf[22];
    lect.channels_[CH_UNSAFE].source_quality[i]= buf[23];
    double sv;
    std::memcpy(&sv, buf + 24, 8);
    lect.split_val_[i] = sv;
    int ep = lect.ep_stride_;
    if (ep > 0) {
        std::memcpy(lect.channels_[CH_SAFE].ep_data.data() + i * ep,
                    buf + 32, ep * sizeof(float));
        std::memcpy(lect.channels_[CH_UNSAFE].ep_data.data() + i * ep,
                    buf + 32 + ep * sizeof(float), ep * sizeof(float));
    }
}

// ── V3 pack/unpack (legacy, single-channel → CH_SAFE) ──────────────────
static void pack_node_v3(uint8_t* buf, const LECT& lect, int i) {
    int32_t fields[5] = {
        static_cast<int32_t>(lect.left_[i]),
        static_cast<int32_t>(lect.right_[i]),
        static_cast<int32_t>(lect.parent_[i]),
        static_cast<int32_t>(lect.depth_[i]),
        static_cast<int32_t>(lect.split_dim_[i])
    };
    std::memcpy(buf, fields, 20);
    buf[20] = lect.channels_[CH_SAFE].has_data[i];
    buf[21] = lect.channels_[CH_SAFE].source_quality[i];
    buf[22] = 0;
    buf[23] = 0;
    double sv = lect.split_val_[i];
    std::memcpy(buf + 24, &sv, 8);
    int ep = lect.ep_stride_;
    if (ep > 0) {
        std::memcpy(buf + 32,
                    lect.channels_[CH_SAFE].ep_data.data() + i * ep,
                    ep * sizeof(float));
    }
}

static void unpack_node_v3(const uint8_t* buf, LECT& lect, int i) {
    int32_t fields[5];
    std::memcpy(fields, buf, 20);
    lect.left_[i]      = fields[0];
    lect.right_[i]     = fields[1];
    lect.parent_[i]    = fields[2];
    lect.depth_[i]     = fields[3];
    lect.split_dim_[i] = fields[4];
    lect.channels_[CH_SAFE].has_data[i]       = buf[20];
    lect.channels_[CH_SAFE].source_quality[i] = buf[21];
    double sv;
    std::memcpy(&sv, buf + 24, 8);
    lect.split_val_[i] = sv;
    int ep = lect.ep_stride_;
    if (ep > 0) {
        std::memcpy(lect.channels_[CH_SAFE].ep_data.data() + i * ep,
                    buf + 32, ep * sizeof(float));
    }
}

template<typename T>
static bool read_vec(std::ifstream& in, std::vector<T>& v, int count) {
    if (count <= 0) return true;
    v.resize(count);
    in.read(reinterpret_cast<char*>(v.data()),
            static_cast<std::streamsize>(count) * sizeof(T));
    return in.good();
}

static LectFileHeaderV4 make_header_v4(const LECT& lect, int nn, int cap) {
    LectFileHeaderV4 hdr{};
    std::memcpy(hdr.magic, MAGIC, 8);
    hdr.version        = FORMAT_V4;
    hdr.n_nodes        = nn;
    hdr.n_dims         = lect.n_dims_;
    hdr.n_active_links = lect.n_active_links_;
    hdr.ep_stride      = lect.ep_stride_;
    hdr.liaabb_stride   = lect.liaabb_stride_;
    hdr.split_order    = static_cast<uint8_t>(lect.split_order_);
    hdr.ep_source      = static_cast<uint8_t>(lect.ep_config_.source);
    hdr.env_type       = static_cast<uint8_t>(lect.env_config_.type);
    hdr.n_channels     = N_CHANNELS;
    hdr.capacity       = cap;
    hdr.robot_hash     = lect.robot_.fingerprint();

    // Grid section presence
    bool has_grids = false;
    for (int i = 0; i < nn && !has_grids; ++i) {
        if (i < static_cast<int>(lect.node_grids_.size()) &&
            !lect.node_grids_[i].empty())
            has_grids = true;
    }
    hdr.grid_section = has_grids ? 1 : 0;
    hdr.grid_delta   = static_cast<float>(lect.env_config_.grid_config.voxel_delta);
    std::memset(hdr.reserved2, 0, sizeof(hdr.reserved2));
    std::memset(hdr.reserved3, 0, sizeof(hdr.reserved3));
    return hdr;
}

static void rebuild_link_iaabbs(LECT& lect, const Robot& robot, int nn) {
    int lia_total = nn * lect.liaabb_stride_;
    lect.link_iaabb_cache_.resize(lia_total, 0.0f);
    const double* radii = robot.active_link_radii();
    for (int i = 0; i < nn; ++i) {
        if (!lect.has_data(i)) continue;
        // Prefer SAFE channel if available
        const float* ep = lect.channels_[CH_SAFE].has_data[i]
            ? lect.channels_[CH_SAFE].ep_data.data() + i * lect.ep_stride_
            : lect.channels_[CH_UNSAFE].ep_data.data() + i * lect.ep_stride_;
        derive_link_iaabb_paired(ep, lect.n_active_links_, radii,
                                 lect.link_iaabb_cache_.data() + i * lect.liaabb_stride_);
    }
}

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

static void write_root_intervals(std::ostream& out, const LECT& lect) {
    for (int d = 0; d < lect.n_dims_; ++d) {
        double lo = lect.root_intervals_[d].lo;
        double hi = lect.root_intervals_[d].hi;
        out.write(reinterpret_cast<const char*>(&lo), sizeof(double));
        out.write(reinterpret_cast<const char*>(&hi), sizeof(double));
    }
}

static bool read_root_intervals(std::ifstream& in, int n_dims,
                                std::vector<Interval>& out) {
    out.resize(n_dims);
    for (int d = 0; d < n_dims; ++d) {
        double lo, hi;
        in.read(reinterpret_cast<char*>(&lo), sizeof(double));
        in.read(reinterpret_cast<char*>(&hi), sizeof(double));
        out[d] = {lo, hi};
    }
    return in.good();
}

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

static void read_cache_trailer(std::istream& in, LECT& lect, bool v4_format) {
    // z4 cache
    int32_t n_z4 = 0;
    in.read(reinterpret_cast<char*>(&n_z4), sizeof(n_z4));
    if (!in.good() || n_z4 < 0) return;
    for (int32_t i = 0; i < n_z4; ++i) {
        uint64_t key;
        in.read(reinterpret_cast<char*>(&key), sizeof(key));
        uint8_t src;
        in.read(reinterpret_cast<char*>(&src), 1);
        LECT::Z4CacheEntry entry;
        entry.source = static_cast<EndpointSource>(src);
        if (v4_format) {
            uint8_t ch_byte;
            in.read(reinterpret_cast<char*>(&ch_byte), 1);
            entry.channel = static_cast<int>(ch_byte);
            uint8_t pad[2];
            in.read(reinterpret_cast<char*>(pad), 2);
        } else {
            entry.channel = source_channel(entry.source);
            uint8_t pad[3];
            in.read(reinterpret_cast<char*>(pad), 3);
        }
        entry.ep_iaabbs.resize(lect.ep_stride_);
        in.read(reinterpret_cast<char*>(entry.ep_iaabbs.data()),
                static_cast<std::streamsize>(lect.ep_stride_) * sizeof(float));
        if (!in.good()) return;
        lect.z4_cache_[key] = std::move(entry);
    }
    // depth → split-dim cache
    int32_t n_dd = 0;
    in.read(reinterpret_cast<char*>(&n_dd), sizeof(n_dd));
    if (!in.good() || n_dd < 0) return;
    for (int32_t i = 0; i < n_dd; ++i) {
        int32_t d, m;
        in.read(reinterpret_cast<char*>(&d), sizeof(d));
        in.read(reinterpret_cast<char*>(&m), sizeof(m));
        if (!in.good()) return;
        lect.depth_split_dim_cache_[d] = m;
    }
}

// ── Grid section write/read ─────────────────────────────────────────────
static void write_grid_section(std::ostream& out, const LECT& lect) {
    static constexpr char GRID_MAGIC[4] = {'G','R','I','D'};
    out.write(GRID_MAGIC, 4);

    // Count nodes that have grids
    int32_t n_entries = 0;
    int nn = lect.n_nodes_;
    for (int i = 0; i < nn; ++i) {
        if (i < static_cast<int>(lect.node_grids_.size()) &&
            !lect.node_grids_[i].empty())
            n_entries++;
    }
    out.write(reinterpret_cast<const char*>(&n_entries), sizeof(n_entries));

    for (int i = 0; i < nn; ++i) {
        if (i >= static_cast<int>(lect.node_grids_.size()) ||
            lect.node_grids_[i].empty())
            continue;

        const auto& grids = lect.node_grids_[i];
        const auto& metas = lect.node_grid_meta_[i];

        int32_t node_idx = static_cast<int32_t>(i);
        int32_t n_slots = static_cast<int32_t>(grids.size());
        out.write(reinterpret_cast<const char*>(&node_idx), sizeof(node_idx));
        out.write(reinterpret_cast<const char*>(&n_slots), sizeof(n_slots));

        for (int s = 0; s < n_slots; ++s) {
            // GridSlot metadata: type(1) + channel(1) + pad(2) + delta(4) = 8 bytes
            uint8_t type_byte = static_cast<uint8_t>(metas[s].type);
            uint8_t ch_byte   = metas[s].channel;
            uint8_t pad[2]    = {0, 0};
            float   delta     = metas[s].delta;
            out.write(reinterpret_cast<const char*>(&type_byte), 1);
            out.write(reinterpret_cast<const char*>(&ch_byte), 1);
            out.write(reinterpret_cast<const char*>(pad), 2);
            out.write(reinterpret_cast<const char*>(&delta), sizeof(delta));

            // Grid data: delta(8) + safety_pad(8) + origin(24) + n_bricks(4) + bricks
            const auto& grid = grids[s];
            double gdelta = grid.delta();
            double gpad   = grid.safety_pad();
            out.write(reinterpret_cast<const char*>(&gdelta), sizeof(gdelta));
            out.write(reinterpret_cast<const char*>(&gpad), sizeof(gpad));
            const double* org = grid.origin();
            out.write(reinterpret_cast<const char*>(org), 3 * sizeof(double));

            int32_t n_bricks = grid.num_bricks();
            out.write(reinterpret_cast<const char*>(&n_bricks), sizeof(n_bricks));

            // Per brick: BrickCoord(3×int32=12) + BitBrick(64 bytes) = 76 bytes
            for (auto e : grid.bricks()) {
                int32_t bc[3] = {e.key.bx, e.key.by, e.key.bz};
                out.write(reinterpret_cast<const char*>(bc), 12);
                out.write(reinterpret_cast<const char*>(e.value.words), 64);
            }
        }
    }
}

static bool read_grid_section(std::istream& in, LECT& lect) {
    char magic[4];
    in.read(magic, 4);
    if (!in.good()) return false;
    if (std::memcmp(magic, "GRID", 4) != 0) return false;

    int32_t n_entries = 0;
    in.read(reinterpret_cast<char*>(&n_entries), sizeof(n_entries));
    if (!in.good() || n_entries < 0) return false;

    for (int32_t e = 0; e < n_entries; ++e) {
        int32_t node_idx, n_slots;
        in.read(reinterpret_cast<char*>(&node_idx), sizeof(node_idx));
        in.read(reinterpret_cast<char*>(&n_slots), sizeof(n_slots));
        if (!in.good() || node_idx < 0 || node_idx >= lect.n_nodes_)
            return false;

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
                voxel::BrickCoord coord; coord.bx = bc[0]; coord.by = bc[1]; coord.bz = bc[2];
                grid.set_brick(coord, brick);
            }

            grids.push_back(std::move(grid));
            metas.push_back(meta);
        }
    }
    return true;
}

// ── Allocate dual-channel buffers for loading ───────────────────────────
static void alloc_buffers(LECT& lect, int nn) {
    for (int ch = 0; ch < N_CHANNELS; ++ch)
        lect.channels_[ch].resize(nn, lect.ep_stride_);
    lect.left_.resize(nn);
    lect.right_.resize(nn);
    lect.parent_.resize(nn);
    lect.depth_.resize(nn);
    lect.split_dim_.resize(nn);
    lect.split_val_.resize(nn);
    lect.node_grids_.resize(nn);
    lect.node_grid_meta_.resize(nn);
}

// ── V4 loader ───────────────────────────────────────────────────────────
static bool load_v4(LECT& lect, const Robot& robot,
                    std::ifstream& in, const LectFileHeaderV4& hdr) {
    const int nn = hdr.n_nodes;

    std::vector<Interval> root_iv;
    if (!read_root_intervals(in, hdr.n_dims, root_iv)) return false;
    set_lect_state(lect, robot, hdr.n_dims, hdr.n_active_links,
                   hdr.ep_stride, hdr.liaabb_stride, nn,
                   hdr.split_order, hdr.ep_source, hdr.env_type, root_iv);

    lect.env_config_.grid_config.voxel_delta = static_cast<double>(hdr.grid_delta);
    alloc_buffers(lect, nn);

    // Bulk-read all node records in a single IO call
    int st = stride_v4(hdr.ep_stride);
    std::vector<uint8_t> bulk(static_cast<size_t>(st) * nn);
    in.read(reinterpret_cast<char*>(bulk.data()),
            static_cast<std::streamsize>(st) * nn);
    if (!in.good()) return false;
    for (int i = 0; i < nn; ++i)
        unpack_node_v4(bulk.data() + static_cast<size_t>(i) * st, lect, i);

    // Lazy link IAABB: mark all dirty, skip rebuild_link_iaabbs
    lect.link_iaabb_cache_.resize(static_cast<size_t>(nn) * lect.liaabb_stride_, 0.0f);
    lect.link_iaabb_dirty_.assign(nn, 1);
    lect.forest_id_.assign(nn, -1);
    lect.subtree_occ_.assign(nn, 0);
    rebuild_derived(lect, robot);

    read_cache_trailer(in, lect, true);

    // Read grid section if present
    if (hdr.grid_section) {
        read_grid_section(in, lect);
    }

    return true;
}

// ── V3 loader ───────────────────────────────────────────────────────────
static bool load_v3(LECT& lect, const Robot& robot,
                    std::ifstream& in, const LectFileHeader& hdr) {
    const int nn = hdr.n_nodes;

    std::vector<Interval> root_iv;
    if (!read_root_intervals(in, hdr.n_dims, root_iv)) return false;
    set_lect_state(lect, robot, hdr.n_dims, hdr.n_active_links,
                   hdr.ep_stride, hdr.liaabb_stride, nn,
                   hdr.split_order, hdr.ep_source, hdr.env_type, root_iv);

    alloc_buffers(lect, nn);

    // Bulk-read all node records
    int st = stride_v3(hdr.ep_stride);
    std::vector<uint8_t> bulk(static_cast<size_t>(st) * nn);
    in.read(reinterpret_cast<char*>(bulk.data()),
            static_cast<std::streamsize>(st) * nn);
    if (!in.good()) return false;
    for (int i = 0; i < nn; ++i)
        unpack_node_v3(bulk.data() + static_cast<size_t>(i) * st, lect, i);

    // Lazy link IAABB
    lect.link_iaabb_cache_.resize(static_cast<size_t>(nn) * lect.liaabb_stride_, 0.0f);
    lect.link_iaabb_dirty_.assign(nn, 1);
    lect.forest_id_.assign(nn, -1);
    lect.subtree_occ_.assign(nn, 0);
    rebuild_derived(lect, robot);

    // Read z4/depth-dim cache trailer (gracefully handles old files)
    read_cache_trailer(in, lect, false);
    return true;
}

// ── V1/V2 loader ────────────────────────────────────────────────────────
static bool load_v1v2(LECT& lect, const Robot& robot,
                      std::ifstream& in, const LectFileHeader& hdr) {
    const bool is_v1 = (hdr.version == 1);
    const int nn = hdr.n_nodes;

    std::vector<Interval> root_iv;
    if (!read_root_intervals(in, hdr.n_dims, root_iv)) return false;
    set_lect_state(lect, robot, hdr.n_dims, hdr.n_active_links,
                   hdr.ep_stride, hdr.liaabb_stride, nn,
                   hdr.split_order, hdr.ep_source, hdr.env_type, root_iv);

    // V1/V2: SoA layout → load into CH_SAFE
    alloc_buffers(lect, nn);

    if (!read_vec(in, lect.left_, nn))       return false;
    if (!read_vec(in, lect.right_, nn))      return false;
    if (!read_vec(in, lect.parent_, nn))     return false;
    if (!read_vec(in, lect.depth_, nn))      return false;
    if (!read_vec(in, lect.split_dim_, nn))  return false;

    lect.split_val_.resize(nn);
    in.read(reinterpret_cast<char*>(lect.split_val_.data()),
            static_cast<std::streamsize>(nn) * sizeof(double));
    if (!in.good()) return false;

    if (is_v1) {
        std::vector<int> dummy;
        if (!read_vec(in, dummy, nn)) return false;
    }

    // has_data and source_quality → CH_SAFE
    if (!read_vec(in, lect.channels_[CH_SAFE].has_data, nn))         return false;
    if (!read_vec(in, lect.channels_[CH_SAFE].source_quality, nn))   return false;

    int ep_total = nn * hdr.ep_stride;
    lect.channels_[CH_SAFE].ep_data.resize(ep_total);
    if (ep_total > 0) {
        in.read(reinterpret_cast<char*>(lect.channels_[CH_SAFE].ep_data.data()),
                static_cast<std::streamsize>(ep_total) * sizeof(float));
        if (!in.good()) return false;
    }

    if (is_v1) {
        int lia_total = nn * hdr.liaabb_stride;
        std::vector<float> dummy(lia_total);
        if (lia_total > 0) {
            in.read(reinterpret_cast<char*>(dummy.data()),
                    static_cast<std::streamsize>(lia_total) * sizeof(float));
            if (!in.good()) return false;
        }
    }

    rebuild_link_iaabbs(lect, robot, nn);
    lect.link_iaabb_dirty_.assign(nn, 0);
    lect.forest_id_.assign(nn, -1);
    lect.subtree_occ_.assign(nn, 0);
    rebuild_derived(lect, robot);
    return true;
}

};  // struct LectIOHelper

// ═══════════════════════════════════════════════════════════════════════════
//  Save — full rewrite in V4 AoS dual-channel format
// ═══════════════════════════════════════════════════════════════════════════

bool lect_save_binary(const LECT& lect, const std::string& path) {
    const int nn = lect.n_nodes_;
    if (nn <= 0) return false;

    std::ofstream out(path, std::ios::binary);
    if (!out.is_open()) return false;

    auto hdr = LectIOHelper::make_header_v4(lect, nn, nn);
    out.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
    if (!out.good()) return false;

    LectIOHelper::write_root_intervals(out, lect);
    if (!out.good()) return false;

    int st = LectIOHelper::stride_v4(lect.ep_stride_);
    std::vector<uint8_t> buf(st, 0);
    for (int i = 0; i < nn; ++i) {
        LectIOHelper::pack_node_v4(buf.data(), lect, i);
        out.write(reinterpret_cast<const char*>(buf.data()), st);
    }

    LectIOHelper::write_cache_trailer(out, lect);

    // Write grid section if any grids exist
    if (hdr.grid_section) {
        LectIOHelper::write_grid_section(out, lect);
    }

    return out.good();
}

// ═══════════════════════════════════════════════════════════════════════════
//  Incremental save — V4 dual-channel O(delta) with AoS layout
//
//  Rewrites header + modified old nodes + appends new nodes.
//  Modified old nodes are detected by: left_[i] >= old_n_nodes
//  (they were leaf nodes that got split during this session).
//
//  Note: grid section is NOT incrementally updated — always full rewrite.
//  Incremental save only patches the ep_data AoS section.
// ═══════════════════════════════════════════════════════════════════════════

bool lect_save_incremental(const LECT& lect, const std::string& path,
                           int old_n_nodes) {
    const int nn = lect.n_nodes_;
    if (nn <= 0) return false;

    if (old_n_nodes <= 0)
        return lect_save_binary(lect, path);

    // No new nodes — check if format migration is needed
    if (old_n_nodes >= nn) {
        std::ifstream probe(path, std::ios::binary);
        if (probe.is_open()) {
            LectFileHeaderV4 old_hdr{};
            probe.read(reinterpret_cast<char*>(&old_hdr), sizeof(old_hdr));
            if (probe.good() && old_hdr.version == FORMAT_V4)
                return true;  // already V4, nothing changed
        }
        return lect_save_binary(lect, path);  // migration needed
    }

    // Check if old file was V4 (same header size) — if not, full rewrite
    {
        std::ifstream probe(path, std::ios::binary);
        if (!probe.is_open()) return lect_save_binary(lect, path);
        LectFileHeaderV4 old_hdr{};
        probe.read(reinterpret_cast<char*>(&old_hdr), sizeof(old_hdr));
        if (!probe.good() || old_hdr.version != FORMAT_V4)
            return lect_save_binary(lect, path);
    }

    std::fstream fs(path, std::ios::in | std::ios::out | std::ios::binary);
    if (!fs.is_open())
        return lect_save_binary(lect, path);

    auto hdr = LectIOHelper::make_header_v4(lect, nn, nn);
    fs.seekp(0);
    fs.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));
    if (!fs.good()) { fs.close(); return lect_save_binary(lect, path); }

    int st = LectIOHelper::stride_v4(lect.ep_stride_);
    size_t off = LectIOHelper::doff_v4(lect.n_dims_);
    std::vector<uint8_t> buf(st, 0);

    // Rewrite modified old nodes (parents that were split this session)
    for (int i = 0; i < old_n_nodes; ++i) {
        if (lect.left_[i] >= old_n_nodes) {
            LectIOHelper::pack_node_v4(buf.data(), lect, i);
            fs.seekp(static_cast<std::streamoff>(off + i * st));
            fs.write(reinterpret_cast<const char*>(buf.data()), st);
        }
    }

    // Append new nodes
    fs.seekp(static_cast<std::streamoff>(off + old_n_nodes * st));
    for (int i = old_n_nodes; i < nn; ++i) {
        LectIOHelper::pack_node_v4(buf.data(), lect, i);
        fs.write(reinterpret_cast<const char*>(buf.data()), st);
    }

    // Rewrite cache trailer at new end (after all nodes)
    LectIOHelper::write_cache_trailer(fs, lect);

    // Rewrite grid section (full, not incremental)
    if (hdr.grid_section) {
        LectIOHelper::write_grid_section(fs, lect);
    }

    // Truncate: the file may have old trailer/grid bytes beyond this point
    auto end_pos = static_cast<std::uintmax_t>(fs.tellp());
    fs.close();
    std::error_code ec;
    std::filesystem::resize_file(path, end_pos, ec);
    return !ec;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Load — dispatch to V1/V2, V3, or V4 based on version field
// ═══════════════════════════════════════════════════════════════════════════

bool lect_load_binary(LECT& lect, const Robot& robot, const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) return false;

    // Read the V3-size header first (48 bytes) — works for all versions
    LectFileHeader hdr{};
    in.read(reinterpret_cast<char*>(&hdr), sizeof(hdr));
    if (!in.good()) return false;

    if (std::memcmp(hdr.magic, MAGIC, 8) != 0) return false;
    if (hdr.version < 1 || hdr.version > 4) return false;
    if (hdr.n_dims != robot.n_joints()) return false;
    if (hdr.n_active_links != robot.n_active_links()) return false;
    if (hdr.robot_hash != 0 && hdr.robot_hash != robot.fingerprint())
        return false;
    if (hdr.ep_stride != hdr.n_active_links * 2 * 6) return false;
    if (hdr.liaabb_stride != hdr.n_active_links * 6) return false;
    if (hdr.n_nodes <= 0) return false;

    if (hdr.version == 4) {
        // Re-read as V4 header (64 bytes)
        in.seekg(0);
        LectFileHeaderV4 hdr4{};
        in.read(reinterpret_cast<char*>(&hdr4), sizeof(hdr4));
        if (!in.good()) return false;
        return LectIOHelper::load_v4(lect, robot, in, hdr4);
    } else if (hdr.version == 3) {
        return LectIOHelper::load_v3(lect, robot, in, hdr);
    } else {
        return LectIOHelper::load_v1v2(lect, robot, in, hdr);
    }
}

}  // namespace sbf
