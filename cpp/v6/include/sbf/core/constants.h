#pragma once
/// @file constants.h
/// @brief Collected numerical constants used across SafeBoxForest modules.
///
/// These values are extracted from various source files to eliminate magic
/// numbers.  The values themselves are unchanged from v5.

namespace sbf::constants {

// ── Adjacency / geometry ────────────────────────────────────────────────────
constexpr double kAdjacencyTol       = 1e-6;   // adjacency.cpp: face-sharing tolerance
constexpr double kBridgeGapMax       = 1e-4;   // connectivity.cpp: max gap for bridge repair
constexpr double kOverlapMargin      = 1e-8;   // connectivity.cpp: overlap margin in repair

// ── FFB ────────────────────────────────────────────────────────────────────
constexpr int    kFFBDeadlineSample  = 64;     // ffb.cpp: check deadline every N iterations

// ── Dijkstra ───────────────────────────────────────────────────────────────
constexpr double kDijkstraHopPenalty = 0.02;   // dijkstra.cpp: per-hop cost penalty

// ── Collision ──────────────────────────────────────────────────────────────
constexpr double kSlabEpsilon        = 1e-15;  // collision_checker.cpp: ray-AABB slab tolerance

// ── Analytical envelope ────────────────────────────────────────────────────
constexpr double kPhase123Threshold  = 0.15;   // analytical_source.cpp: ~8.6° narrow-joint skip

}  // namespace sbf::constants
