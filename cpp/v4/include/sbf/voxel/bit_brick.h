// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Sparse Voxel BitMask
//
//  bit_brick.h
//
//  BitBrick:    8×8×8 = 512 voxels packed into 8 × uint64_t  (64 bytes)
//  BrickCoord:  integer tile address in the sparse spatial hash
//
//  迁移自 v3 bit_brick.h
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include <cstdint>
#include <cstring>
#include <functional>

#ifdef _MSC_VER
#include <intrin.h>          // __popcnt64
#pragma intrinsic(__popcnt)
#if defined(_M_X64) || defined(_M_ARM64)
#pragma intrinsic(__popcnt64)
#endif
#endif

namespace sbf {
namespace voxel {

// ─────────────────────────────────────────────────────────────────────────
//  BitBrick — 512-bit occupancy mask for an 8³ tile
// ─────────────────────────────────────────────────────────────────────────
//
//  Voxel local coordinate (x, y, z), each in [0, 7].
//  Layout:  word_index = z,  bit_index = y * 8 + x
//  → each word encodes one horizontal (x-y) slab.
//
struct BitBrick {
    uint64_t words[8] = {};

    // ── Single-voxel access ─────────────────────────────────────────────
    void set(int x, int y, int z) noexcept {
        words[z] |= uint64_t(1) << (y * 8 + x);
    }
    bool test(int x, int y, int z) const noexcept {
        return (words[z] >> (y * 8 + x)) & 1;
    }
    void clear_voxel(int x, int y, int z) noexcept {
        words[z] &= ~(uint64_t(1) << (y * 8 + x));
    }

    // ── Bulk ────────────────────────────────────────────────────────────
    void clear() noexcept { std::memset(words, 0, sizeof(words)); }

    bool is_empty() const noexcept {
        for (int i = 0; i < 8; ++i)
            if (words[i]) return false;
        return true;
    }

    int popcount() const noexcept {
        int n = 0;
        for (int i = 0; i < 8; ++i) {
#ifdef _MSC_VER
        #if defined(_M_X64) || defined(_M_ARM64)
            n += static_cast<int>(__popcnt64(words[i]));
        #else
            const uint64_t w = words[i];
            n += static_cast<int>(__popcnt(static_cast<unsigned int>(w)));
            n += static_cast<int>(__popcnt(static_cast<unsigned int>(w >> 32)));
        #endif
#else
            n += __builtin_popcountll(words[i]);
#endif
        }
        return n;
    }

    // ── Bitwise merge (OR) — zero-loss union ────────────────────────────
    BitBrick operator|(const BitBrick& o) const noexcept {
        BitBrick r;
        for (int i = 0; i < 8; ++i) r.words[i] = words[i] | o.words[i];
        return r;
    }
    BitBrick& operator|=(const BitBrick& o) noexcept {
        for (int i = 0; i < 8; ++i) words[i] |= o.words[i];
        return *this;
    }

    // ── Bitwise intersection (AND) ──────────────────────────────────────
    BitBrick operator&(const BitBrick& o) const noexcept {
        BitBrick r;
        for (int i = 0; i < 8; ++i) r.words[i] = words[i] & o.words[i];
        return r;
    }

    // ── Fast collision test — any overlapping voxel? ────────────────────
    bool intersects(const BitBrick& o) const noexcept {
        for (int i = 0; i < 8; ++i)
            if (words[i] & o.words[i]) return true;
        return false;
    }
};

// Sanity: each BitBrick is exactly 64 bytes (1 cache line on most CPUs).
static_assert(sizeof(BitBrick) == 64, "BitBrick must be 64 bytes");

// ─────────────────────────────────────────────────────────────────────────
//  BrickCoord — integer tile address (brick_x, brick_y, brick_z)
// ─────────────────────────────────────────────────────────────────────────
struct BrickCoord {
    int bx = 0, by = 0, bz = 0;

    bool operator==(const BrickCoord& o) const noexcept {
        return bx == o.bx && by == o.by && bz == o.bz;
    }
    bool operator!=(const BrickCoord& o) const noexcept {
        return !(*this == o);
    }
};

struct BrickCoordHash {
    std::size_t operator()(const BrickCoord& c) const noexcept {
        // FNV-1a style hash
        std::size_t h = 14695981039346656037ULL;
        h ^= static_cast<std::size_t>(c.bx); h *= 1099511628211ULL;
        h ^= static_cast<std::size_t>(c.by); h *= 1099511628211ULL;
        h ^= static_cast<std::size_t>(c.bz); h *= 1099511628211ULL;
        return h;
    }
};

} // namespace voxel
} // namespace sbf
