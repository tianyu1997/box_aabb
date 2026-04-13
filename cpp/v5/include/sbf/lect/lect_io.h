#pragma once
/// @file lect_io.h
/// @brief LECT binary cache persistence (V5 SoA mmap format + V6 tree-only format).
///
/// V5: Full save with tree + EP + grids in a single file (legacy).
/// V6: Tree-only .lect6 file: tree structure, derived cache, depth_split_dim.
///     EP data and grids are stored in separate Z4-keyed mmap caches
///     managed by LectCacheManager.

#include <sbf/lect/lect.h>

#include <string>

namespace sbf {

// ═══════════════════════════════════════════════════════════════════════════
//  V5 format (legacy)
// ═══════════════════════════════════════════════════════════════════════════

/// Save LECT to binary file (V5 SoA format, full rewrite).
bool lect_save_binary(const LECT& lect, const std::string& path);

/// Incremental save: patch modified/new nodes within capacity gap.
/// Falls back to full save if the file is incompatible or capacity exceeded.
bool lect_save_incremental(const LECT& lect, const std::string& path,
                           int old_n_nodes);

/// Load LECT from binary file via mmap (V5 SoA format only).
/// Tree metadata is bulk-memcpy'd; ep_data is served from COW mmap.
/// link_iaabb_cache_ is lazy; forest_id_ is initialised to −1.
bool lect_load_binary(LECT& lect, const Robot& robot, const std::string& path);

// ═══════════════════════════════════════════════════════════════════════════
//  V6 format (tree-only, caches separate)
// ═══════════════════════════════════════════════════════════════════════════

/// Save LECT tree structure to a V6 .lect6 file.
/// Only saves: tree SoA, config, derived cache, depth_split_dim.
/// EP data and grids are NOT saved (managed by LectCacheManager).
bool lect_save_v6(const LECT& lect, const std::string& path);

/// Load LECT tree structure from a V6 .lect6 file.
/// The loaded LECT has NO EP data (has_data=0 for all nodes).
/// The caller must set_cache_manager() and re-compute envelopes or
/// rely on V6 persistent cache hits during tree traversal.
bool lect_load_v6(LECT& lect, const Robot& robot, const std::string& path);

}  // namespace sbf
