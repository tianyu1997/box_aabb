// SafeBoxForest — Bridge module: coarsen, connectivity, GCS
// Module: sbf::bridge
// Re-exports coarsen, connectivity, and GCS optimizer under bridge namespace.
#pragma once

// Coarsen algorithms
#include "sbf/forest/coarsen.h"

// Connectivity (UnionFind, islands, bridging)
#include "sbf/forest/connectivity.h"

// GCS optimizer
#include "sbf/adapters/drake_gcs.h"

namespace sbf {
namespace bridge {

// ─── Re-export core bridge types into sbf::bridge namespace ─────────────
using sbf::CoarsenResult;
using sbf::GreedyCoarsenResult;
using sbf::coarsen_forest;
using sbf::coarsen_greedy;
using sbf::UnionFind;
using sbf::find_islands;
using sbf::BridgeResult;
using sbf::bridge_islands;
using sbf::GCSResult;
using sbf::GCSOptimizer;

} // namespace bridge
} // namespace sbf
