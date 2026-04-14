#pragma once
/// @file viz_exporter.h
/// @brief JSON export utilities for visualisation.
///
/// Exports robot FK, link envelopes, obstacle scenes, box forests, and
/// voxel grids to JSON files consumable by the Leaflet/Three.js viewer.
///
/// @see leaflet/ directory for the browser-based visualiser.

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/core/fk_state.h>
#include <sbf/lect/lect.h>
#include <sbf/voxel/voxel_grid.h>

#include <Eigen/Core>
#include <string>
#include <vector>

namespace sbf::viz {

// ─── Robot FK Export (Step I1) ──────────────────────────────────────────────
// Export robot model + FK link positions for one or more configurations.
void export_robot_json(
    const std::string& path,
    const Robot& robot,
    const std::vector<Eigen::VectorXd>& configs);

// ─── Envelope iAABB Export (Step I2) ────────────────────────────────────────
// Export per-node link iAABB envelope from a LECT.
void export_envelope_json(
    const std::string& path,
    const Robot& robot,
    const std::vector<BoxNode>& boxes,
    const LECT& lect);

// Export FK-based envelope (snapshot) — computes FK internally per box.
void export_envelope_from_boxes_json(
    const std::string& path,
    const Robot& robot,
    const std::vector<BoxNode>& boxes,
    int samples_per_box = 8);

// ─── Scene / Obstacle Export (Step I3) ──────────────────────────────────────
void export_scene_json(
    const std::string& path,
    const Obstacle* obs, int n_obs);

// ─── Snapshot Unified Export (Step I4) ──────────────────────────────────────
// One-shot export of all layers: robot + envelope + scene + forest.
void export_snapshot_json(
    const std::string& path,
    const Robot& robot,
    const std::vector<BoxNode>& boxes,
    const LECT& lect,
    const Obstacle* obs, int n_obs,
    const std::vector<Eigen::VectorXd>& sample_configs = {});

// ─── Internal Helpers (Step I5) ─────────────────────────────────────────────
namespace detail {

Eigen::VectorXd box_center(const BoxNode& box);

}  // namespace detail

// ─── Envelope Comparison Export (Step P1) ───────────────────────────────────
// 3 envelope methods side-by-side for a single box.
void export_envelope_comparison_json(
    const std::string& path,
    const Robot& robot,
    const BoxNode& box,
    const LECT& lect,
    double voxel_delta = 0.02);

// ─── Voxel Export (Step P2) ─────────────────────────────────────────────────
// Brick-level voxel grid export (hex-encoded words).
void export_voxel_json(
    const std::string& path,
    const voxel::SparseVoxelGrid& grid);

// Voxel centre points export (Cartesian coordinates).
void export_voxel_centres_json(
    const std::string& path,
    const voxel::SparseVoxelGrid& grid);

}  // namespace sbf::viz
