#pragma once
/// @file voxel_grid.h
/// @brief Sparse voxel grid with FlatBrickMap and Hull-16 turbo scanline.
///
/// `SparseVoxelGrid` stores occupied voxels as a hash map of 8×8×8
/// BitBrick tiles (`FlatBrickMap`).  Key operations:
///   - `fill_aabb()` — rasterise an axis-aligned box.
///   - `fill_hull16()` — rasterise the convex hull of two link iAABBs
///     inflated by link radius ("Hull-16 turbo scanline").
///   - `collides()` — O(min(n,m)) brick-level overlap test.
///   - `merge()` — union of two grids.
///
/// The grid is resolution-parameterised by `delta` (voxel edge length),
/// and an optional `safety_pad` adds Minkowski inflation.

#include <sbf/voxel/bit_brick.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstring>
#include <utility>
#include <vector>

namespace sbf::voxel {

// ═════════════════════════════════════════════════════════════════════════════
//  FlatBrickMap — open-addressing hash map for BrickCoord → BitBrick
// ═════════════════════════════════════════════════════════════════════════════
class FlatBrickMap {
    struct Slot {
        BrickCoord key;
        BitBrick   value;
        bool       occupied = false;
    };

    std::vector<Slot> table_;
    int    size_     = 0;
    int    capacity_ = 0;
    int    mask_     = 0;

    static int next_pow2(int n) {
        int v = 16;
        while (v < n) v <<= 1;
        return v;
    }

    int probe(const BrickCoord& k) const noexcept {
        BrickCoordHash h;
        int idx = static_cast<int>(h(k) & static_cast<std::size_t>(mask_));
        while (table_[idx].occupied && table_[idx].key != k)
            idx = (idx + 1) & mask_;
        return idx;
    }

    void grow() {
        int new_cap = (capacity_ == 0) ? 64 : capacity_ * 2;
        std::vector<Slot> old_table = std::move(table_);
        capacity_ = new_cap;
        mask_     = new_cap - 1;
        table_.assign(new_cap, Slot{});
        size_ = 0;
        for (auto& s : old_table) {
            if (s.occupied) {
                int idx = probe(s.key);
                table_[idx].key      = s.key;
                table_[idx].value    = s.value;
                table_[idx].occupied = true;
                ++size_;
            }
        }
    }

public:
    FlatBrickMap() = default;

    BitBrick& operator[](const BrickCoord& k) {
        if (capacity_ == 0 || size_ * 10 >= capacity_ * 7) grow();
        int idx = probe(k);
        if (!table_[idx].occupied) {
            table_[idx].key      = k;
            table_[idx].occupied = true;
            table_[idx].value.clear();
            ++size_;
        }
        return table_[idx].value;
    }

    // ── Iterators ───────────────────────────────────────────────────────
    struct Entry {
        const BrickCoord& key;
        BitBrick& value;
    };
    struct ConstEntry {
        const BrickCoord& key;
        const BitBrick& value;
    };

    struct iterator {
        Slot* ptr;
        Slot* end_ptr;
        void advance() { while (ptr != end_ptr && !ptr->occupied) ++ptr; }
        iterator(Slot* p, Slot* e) : ptr(p), end_ptr(e) { advance(); }
        bool operator!=(const iterator& o) const { return ptr != o.ptr; }
        iterator& operator++() { ++ptr; advance(); return *this; }
        Entry operator*() { return {ptr->key, ptr->value}; }
    };

    struct const_iterator {
        const Slot* ptr;
        const Slot* end_ptr;
        void advance() { while (ptr != end_ptr && !ptr->occupied) ++ptr; }
        const_iterator(const Slot* p, const Slot* e) : ptr(p), end_ptr(e) { advance(); }
        bool operator!=(const const_iterator& o) const { return ptr != o.ptr; }
        const_iterator& operator++() { ++ptr; advance(); return *this; }
        ConstEntry operator*() const { return {ptr->key, ptr->value}; }
        const BitBrick& second() const { return ptr->value; }
    };

    const_iterator find(const BrickCoord& k) const {
        if (capacity_ == 0) return {nullptr, nullptr};
        BrickCoordHash h;
        int idx = static_cast<int>(h(k) & static_cast<std::size_t>(mask_));
        while (table_[idx].occupied && table_[idx].key != k)
            idx = (idx + 1) & mask_;
        if (table_[idx].occupied)
            return {table_.data() + idx, table_.data() + capacity_};
        return {table_.data() + capacity_, table_.data() + capacity_};
    }

    iterator       begin()       { return {table_.data(), table_.data() + capacity_}; }
    iterator       end()         { return {table_.data() + capacity_, table_.data() + capacity_}; }
    const_iterator begin() const { return {table_.data(), table_.data() + capacity_}; }
    const_iterator end()   const { return {table_.data() + capacity_, table_.data() + capacity_}; }

    int  size()  const noexcept { return size_; }
    bool empty() const noexcept { return size_ == 0; }

    bool contains(const BrickCoord& k) const {
        if (capacity_ == 0) return false;
        BrickCoordHash h;
        int idx = static_cast<int>(h(k) & static_cast<std::size_t>(mask_));
        while (table_[idx].occupied && table_[idx].key != k)
            idx = (idx + 1) & mask_;
        return table_[idx].occupied;
    }

    void reserve(int n_hint) {
        int target = next_pow2(n_hint * 10 / 7 + 1);
        if (target <= capacity_) return;
        std::vector<Slot> old_table = std::move(table_);
        capacity_ = target;
        mask_     = target - 1;
        table_.assign(target, Slot{});
        size_ = 0;
        for (auto& s : old_table) {
            if (s.occupied) {
                int idx = probe(s.key);
                table_[idx].key      = s.key;
                table_[idx].value    = s.value;
                table_[idx].occupied = true;
                ++size_;
            }
        }
    }

    void clear() {
        for (auto& s : table_) s.occupied = false;
        size_ = 0;
    }
};

// ═════════════════════════════════════════════════════════════════════════════
//  SparseVoxelGrid — brick-based sparse voxel grid with turbo scanline
// ═════════════════════════════════════════════════════════════════════════════
class SparseVoxelGrid {
public:
    using BrickMap = FlatBrickMap;

    SparseVoxelGrid() = default;

    /// @param delta  voxel edge length (metres)
    /// @param ox,oy,oz  world-space origin
    /// @param safety_pad  extra padding; default = sqrt(3)*delta/2
    explicit SparseVoxelGrid(double delta,
                             double ox = 0.0, double oy = 0.0, double oz = 0.0,
                             double safety_pad = -1.0)
        : delta_(delta)
        , inv_delta_(1.0 / delta)
        , origin_{ox, oy, oz}
        , safety_pad_(safety_pad < 0 ? std::sqrt(3.0) * delta * 0.5 : safety_pad)
    {}

    // ── Coordinate helpers ──────────────────────────────────────────────
    int to_cell(double w, int axis) const noexcept {
        return static_cast<int>(std::floor((w - origin_[axis]) * inv_delta_));
    }

    double cell_center(int c, int axis) const noexcept {
        return origin_[axis] + (c + 0.5) * delta_;
    }

    // ── Rasterisation ───────────────────────────────────────────────────

    void reserve_from_bounds(const double lo[3], const double hi[3]) {
        int bx = (to_cell(hi[0], 0) / 8) - (to_cell(lo[0], 0) / 8) + 1;
        int by = (to_cell(hi[1], 1) / 8) - (to_cell(lo[1], 1) / 8) + 1;
        int bz = (to_cell(hi[2], 2) / 8) - (to_cell(lo[2], 2) / 8) + 1;
        if (bx < 1) bx = 1;
        if (by < 1) by = 1;
        if (bz < 1) bz = 1;
        bricks_.reserve(bx * by * bz);
    }

    /// Fill all voxels overlapping an axis-aligned box.
    /// @param aabb  float[6]: [x_lo, y_lo, z_lo, x_hi, y_hi, z_hi]
    void fill_aabb(const float* aabb) {
        int cx0 = to_cell(static_cast<double>(aabb[0]), 0);
        int cy0 = to_cell(static_cast<double>(aabb[1]), 1);
        int cz0 = to_cell(static_cast<double>(aabb[2]), 2);
        int cx1 = to_cell(static_cast<double>(aabb[3]), 0);
        int cy1 = to_cell(static_cast<double>(aabb[4]), 1);
        int cz1 = to_cell(static_cast<double>(aabb[5]), 2);

        for (int cz = cz0; cz <= cz1; ++cz)
            for (int cy = cy0; cy <= cy1; ++cy)
                set_cell_range_x(cx0, cx1, cy, cz);
    }

    // ── Hull-16 t-range solver ──────────────────────────────────────────

    static std::pair<double,double> find_t_range_yz(
        double qy, double qz, double r_eff_sq,
        const double b1_lo[3], const double b1_hi[3],
        const double slo[3],   const double shi[3]) noexcept
    {
        double bp[6];
        int nb = 0;
        bp[nb++] = 0.0;
        bp[nb++] = 1.0;
        for (int a = 1; a <= 2; ++a) {
            double qa = (a == 1) ? qy : qz;
            if (std::abs(slo[a]) > 1e-15) {
                double t = (qa - b1_lo[a]) / slo[a];
                if (t > 1e-12 && t < 1.0 - 1e-12) bp[nb++] = t;
            }
            if (std::abs(shi[a]) > 1e-15) {
                double t = (qa - b1_hi[a]) / shi[a];
                if (t > 1e-12 && t < 1.0 - 1e-12) bp[nb++] = t;
            }
        }
        for (int i = 1; i < nb; ++i)
            for (int j = i; j > 0 && bp[j] < bp[j - 1]; --j)
                std::swap(bp[j], bp[j - 1]);

        double t_entry = 2.0, t_exit = -1.0;

        for (int i = 0; i + 1 < nb; ++i) {
            double ta = bp[i], tb = bp[i + 1];
            if (tb - ta < 1e-12) continue;

            double tmid = 0.5 * (ta + tb);
            double A = 0.0, B2 = 0.0, C = 0.0;
            for (int a = 1; a <= 2; ++a) {
                double qa = (a == 1) ? qy : qz;
                double lo_m = b1_lo[a] + tmid * slo[a];
                double hi_m = b1_hi[a] + tmid * shi[a];
                if (qa < lo_m) {
                    double c = b1_lo[a] - qa, d = slo[a];
                    A += d * d;  B2 += 2 * c * d;  C += c * c;
                } else if (qa > hi_m) {
                    double c = qa - b1_hi[a], d = -shi[a];
                    A += d * d;  B2 += 2 * c * d;  C += c * c;
                }
            }

            double Cadj = C - r_eff_sq;

            if (A < 1e-15) {
                if (std::abs(B2) < 1e-15) {
                    if (C <= r_eff_sq + 1e-12) {
                        t_entry = std::min(t_entry, ta);
                        t_exit  = std::max(t_exit,  tb);
                    }
                } else {
                    double tc = -Cadj / B2;
                    double slo_t, shi_t;
                    if (B2 > 0) { slo_t = ta; shi_t = std::min(tb, tc); }
                    else        { slo_t = std::max(ta, tc); shi_t = tb; }
                    if (slo_t <= shi_t + 1e-12) {
                        t_entry = std::min(t_entry, slo_t);
                        t_exit  = std::max(t_exit,  shi_t);
                    }
                }
            } else {
                double disc = B2 * B2 - 4.0 * A * Cadj;
                if (disc < 0) continue;
                double sqd = std::sqrt(disc);
                double r1 = (-B2 - sqd) / (2.0 * A);
                double r2 = (-B2 + sqd) / (2.0 * A);
                double slo_t = std::max(ta, std::min(r1, r2));
                double shi_t = std::min(tb, std::max(r1, r2));
                if (slo_t <= shi_t + 1e-12) {
                    t_entry = std::min(t_entry, slo_t);
                    t_exit  = std::max(t_exit,  shi_t);
                }
            }
        }

        if (t_entry > t_exit) return {2.0, -1.0};
        return {std::max(0.0, t_entry), std::min(1.0, t_exit)};
    }

    /// Rasterise Conv(B1 ∪ B2) ⊕ Ball(r_link + safety_pad) via turbo scanline.
    void fill_hull16(const float* prox_iv, const float* dist_iv,
                     double link_radius)
    {
        const double r_eff    = link_radius + safety_pad_;
        const double r_eff_sq = r_eff * r_eff;

        double b1_lo[3], b1_hi[3], slo[3], shi[3];
        double lo[3], hi[3];
        for (int a = 0; a < 3; ++a) {
            b1_lo[a] = prox_iv[a];
            b1_hi[a] = prox_iv[a + 3];
            slo[a]   = dist_iv[a]     - prox_iv[a];
            shi[a]   = dist_iv[a + 3] - prox_iv[a + 3];
            lo[a] = std::min(static_cast<double>(prox_iv[a]),
                             static_cast<double>(dist_iv[a]))    - r_eff;
            hi[a] = std::max(static_cast<double>(prox_iv[a + 3]),
                             static_cast<double>(dist_iv[a + 3])) + r_eff;
        }

        // Fast-path: degenerate hull (slopes ~= 0) → fill_aabb
        const double slope_eps = delta_ * 0.01;
        bool degenerate = true;
        for (int a = 0; a < 3; ++a) {
            if (std::abs(slo[a]) > slope_eps || std::abs(shi[a]) > slope_eps) {
                degenerate = false;
                break;
            }
        }
        if (degenerate) {
            float padded[6];
            for (int a = 0; a < 3; ++a) {
                padded[a]     = static_cast<float>(lo[a]);
                padded[a + 3] = static_cast<float>(hi[a]);
            }
            fill_aabb(padded);
            return;
        }

        // Brick-batch turbo scanline
        const int cx0 = to_cell(lo[0], 0), cx1 = to_cell(hi[0], 0);
        const int cy0 = to_cell(lo[1], 1), cy1 = to_cell(hi[1], 1);
        const int cz0 = to_cell(lo[2], 2), cz1 = to_cell(hi[2], 2);

        const int bx0 = brick_of(cx0), bx1 = brick_of(cx1);
        const int by0 = brick_of(cy0), by1 = brick_of(cy1);
        const int bz0 = brick_of(cz0), bz1 = brick_of(cz1);

        const int n_xb = bx1 - bx0 + 1;

        const double brick_half_diag = 3.5 * 1.4142135624 * delta_;
        const double r_test = r_eff + brick_half_diag;
        const double r_test_sq = r_test * r_test;

        static constexpr int MAX_XB = 128;
        BitBrick local_bricks[MAX_XB];

        // Fallback for extremely wide hulls
        if (n_xb > MAX_XB) {
            for (int cz = cz0; cz <= cz1; ++cz) {
                const double wz = cell_center(cz, 2);
                for (int cy = cy0; cy <= cy1; ++cy) {
                    const double wy = cell_center(cy, 1);
                    auto [tlo2, thi2] = find_t_range_yz(
                        wy, wz, r_eff_sq, b1_lo, b1_hi, slo, shi);
                    if (tlo2 > thi2) continue;
                    double xl = std::min(b1_lo[0] + tlo2 * slo[0],
                                         b1_lo[0] + thi2 * slo[0]) - r_eff;
                    double xh = std::max(b1_hi[0] + tlo2 * shi[0],
                                         b1_hi[0] + thi2 * shi[0]) + r_eff;
                    int cxs = std::max(cx0, to_cell(xl, 0));
                    int cxe = std::min(cx1, to_cell(xh, 0));
                    if (cxs <= cxe) set_cell_range_x(cxs, cxe, cy, cz);
                }
            }
            return;
        }

        for (int bz = bz0; bz <= bz1; ++bz) {
            const int cz_s = std::max(cz0, bz * 8);
            const int cz_e = std::min(cz1, bz * 8 + 7);

            for (int by = by0; by <= by1; ++by) {
                const int cy_s = std::max(cy0, by * 8);
                const int cy_e = std::min(cy1, by * 8 + 7);

                // Brick-level early-out
                {
                    double wy_c = 0.5 * (cell_center(cy_s, 1)
                                       + cell_center(cy_e, 1));
                    double wz_c = 0.5 * (cell_center(cz_s, 2)
                                       + cell_center(cz_e, 2));
                    auto [t0, t1] = find_t_range_yz(
                        wy_c, wz_c, r_test_sq,
                        b1_lo, b1_hi, slo, shi);
                    if (t0 > t1) continue;
                }

                std::memset(local_bricks, 0,
                            static_cast<std::size_t>(n_xb) * sizeof(BitBrick));

                for (int cz = cz_s; cz <= cz_e; ++cz) {
                    const int lz = local_of(cz);
                    const double wz = cell_center(cz, 2);

                    for (int cy = cy_s; cy <= cy_e; ++cy) {
                        const int ly = local_of(cy);
                        const double wy = cell_center(cy, 1);

                        auto [tlo, thi] = find_t_range_yz(
                            wy, wz, r_eff_sq,
                            b1_lo, b1_hi, slo, shi);
                        if (tlo > thi) continue;

                        double x_lo = std::min(
                            b1_lo[0] + tlo * slo[0],
                            b1_lo[0] + thi * slo[0]) - r_eff;
                        double x_hi = std::max(
                            b1_hi[0] + tlo * shi[0],
                            b1_hi[0] + thi * shi[0]) + r_eff;

                        int cxs = std::max(cx0, to_cell(x_lo, 0));
                        int cxe = std::min(cx1, to_cell(x_hi, 0));
                        if (cxs > cxe) continue;

                        int cx = cxs;
                        while (cx <= cxe) {
                            int bx    = brick_of(cx);
                            int lx0   = local_of(cx);
                            int lx1   = std::min(7, lx0 + (cxe - cx));
                            int count = lx1 - lx0 + 1;
                            uint64_t mask =
                                ((uint64_t(1) << count) - 1)
                                << (ly * 8 + lx0);
                            local_bricks[bx - bx0].words[lz] |= mask;
                            cx += count;
                        }
                    }
                }

                // Flush to global brick map
                for (int i = 0; i < n_xb; ++i) {
                    if (!local_bricks[i].is_empty()) {
                        BrickCoord bc{bx0 + i, by, bz};
                        bricks_[bc] |= local_bricks[i];
                    }
                }
            }
        }
    }

    // ── Volume queries ──────────────────────────────────────────────────
    int count_occupied() const noexcept {
        int n = 0;
        for (auto e : bricks_) n += e.value.popcount();
        return n;
    }

    double occupied_volume() const noexcept {
        return count_occupied() * delta_ * delta_ * delta_;
    }

    int num_bricks() const noexcept { return bricks_.size(); }

    // ── Merge / Collision ───────────────────────────────────────────────
    void merge(const SparseVoxelGrid& other) {
        for (auto e : other.bricks_)
            bricks_[e.key] |= e.value;
    }

    bool collides(const SparseVoxelGrid& other) const {
        const auto& sm = (bricks_.size() <= other.bricks_.size())
                         ? bricks_ : other.bricks_;
        const auto& lg = (bricks_.size() <= other.bricks_.size())
                         ? other.bricks_ : bricks_;
        for (auto e : sm) {
            auto it = lg.find(e.key);
            if (it != lg.end() && e.value.intersects(it.second()))
                return true;
        }
        return false;
    }

    int count_colliding(const SparseVoxelGrid& other) const {
        int total = 0;
        const auto& sm = (bricks_.size() <= other.bricks_.size())
                         ? bricks_ : other.bricks_;
        const auto& lg = (bricks_.size() <= other.bricks_.size())
                         ? other.bricks_ : bricks_;
        for (auto e : sm) {
            auto it = lg.find(e.key);
            if (it != lg.end())
                total += (e.value & it.second()).popcount();
        }
        return total;
    }

    void clear() { bricks_.clear(); }

    // ── Accessors ───────────────────────────────────────────────────────
    double         delta()      const noexcept { return delta_; }
    double         safety_pad() const noexcept { return safety_pad_; }
    const BrickMap& bricks()    const noexcept { return bricks_; }
    const double*   origin()    const noexcept { return origin_; }

    void set_brick(const BrickCoord& coord, const BitBrick& brick) {
        bricks_[coord] = brick;
    }

private:
    double delta_      = 0.02;
    double inv_delta_  = 50.0;
    double origin_[3]  = {};
    double safety_pad_ = 0.0;
    BrickMap bricks_;

    static int brick_of(int cell) noexcept {
        return (cell >= 0) ? cell / 8 : (cell - 7) / 8;
    }
    static int local_of(int cell) noexcept {
        int m = cell % 8;
        return (m >= 0) ? m : m + 8;
    }

    void set_cell_range_x(int cx_start, int cx_end, int cy, int cz) noexcept {
        if (cx_start > cx_end) return;
        const int by = brick_of(cy), bz = brick_of(cz);
        const int ly = local_of(cy), lz = local_of(cz);

        int cx = cx_start;
        while (cx <= cx_end) {
            int bx  = brick_of(cx);
            int lx0 = local_of(cx);
            int lx1 = std::min(7, lx0 + (cx_end - cx));
            int count = lx1 - lx0 + 1;

            int bit_start = ly * 8 + lx0;
            uint64_t mask = ((uint64_t(1) << count) - 1) << bit_start;

            BrickCoord bc{bx, by, bz};
            bricks_[bc].words[lz] |= mask;

            cx += count;
        }
    }
};

}  // namespace sbf::voxel
