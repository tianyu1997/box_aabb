#pragma once
/// @file lect_io.h
/// @brief LECT binary cache persistence (V4 dual-channel format).
///
/// Supports full save, incremental save (O(Δ) append), and load.
/// File format versions:
///   - V1–V3: legacy formats (auto-detected on load, upconverted).
///   - V4: dual-channel AoS layout with optional voxel grid section.
///         `link_iaabb` and `forest_id` are derived/reset on load.

#include <sbf/lect/lect.h>

#include <string>

namespace sbf {

/// Save LECT to binary file (V4 format, full rewrite).
bool lect_save_binary(const LECT& lect, const std::string& path);

/// Incremental save: update header + rewrite modified parents + append new nodes.
/// O(Δ) with AoS layout.  Falls back to full save if the file is incompatible.
bool lect_save_incremental(const LECT& lect, const std::string& path,
                           int old_n_nodes);

/// Load LECT from binary file.  Accepts V1–V4 formats.
/// `link_iaabb_cache_` is re-derived; `forest_id_` is initialised to −1.
bool lect_load_binary(LECT& lect, const Robot& robot, const std::string& path);

}  // namespace sbf
