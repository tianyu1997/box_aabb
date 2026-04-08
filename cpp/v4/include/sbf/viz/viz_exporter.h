// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Visualization JSON Exporter
//  Module: sbf::viz
//
//  迁移自 v3 viz_exporter.h
//
//  Pattern: C++ exports structured JSON → Python reads & renders via Plotly.
//  All export functions produce standalone JSON files for the Python viz
//  package (sbf4_viz).
//
//  v3→v4 变更:
//    • FrameStore → EndpointStore
//    • JSON key: link_aabbs → link_iaabbs, sub_aabbs → link_iaabbs_sub
//    • Scene API: obstacles() 返回 const Obstacle*, 配合 n_obstacles()
//    • 移除 Analytical 系列方法, 保留 5 种对比 (Full/Link iAABB + 3 Voxel)
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/envelope/endpoint_store.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/scene/scene.h"
#include "sbf/voxel/voxel_grid.h"
#include "sbf/voxel/hull_rasteriser.h"

#include <Eigen/Core>
#include <string>
#include <vector>

namespace sbf {
namespace viz {

// ─── Robot FK Export ────────────────────────────────────────────────────────
// Export robot model + FK link positions for one or more configurations.
// JSON schema: { name, n_joints, link_radii[], active_link_map[],
//                configs[{ q[], link_positions[[x,y,z],...] }] }
void export_robot_json(const Robot& robot,
                       const std::vector<Eigen::VectorXd>& configs,
                       const std::string& filepath);

// Single-config convenience
void export_robot_json(const Robot& robot,
                       const Eigen::VectorXd& config,
                       const std::string& filepath);

// ─── Envelope (iAABB) Export ────────────────────────────────────────────────
// Export per-node iAABB link envelopes derived from EndpointStore.
// JSON schema: { n_active_links, link_radii[],
//                nodes[{ node_idx, box_intervals[[lo,hi],...],
//                        link_iaabbs[{link, lo[3], hi[3]}],
//                        link_iaabbs_sub[{link, seg, lo[3], hi[3]}] }] }
void export_envelope_json(const Robot& robot,
                          const envelope::EndpointStore& store,
                          const std::vector<int>& node_indices,
                          int n_sub,
                          const std::string& filepath);

// Export envelopes for a set of C-space interval boxes (no EndpointStore needed).
// Computes FK internally. Useful for one-off visualisation.
void export_envelope_from_boxes_json(
    const Robot& robot,
    const std::vector<std::vector<Interval>>& boxes,
    int n_sub,
    const std::string& filepath);

// ─── Voxel Grid Export ──────────────────────────────────────────────────────
// Export VoxelGrid as brick-level data.
// JSON schema: { delta, origin[3], safety_pad, n_bricks, total_occupied,
//                bricks[{ coord[3], popcount, words[8] (hex-encoded) }] }
void export_voxel_json(const voxel::VoxelGrid& grid,
                       const std::string& filepath);

// Export VoxelGrid with explicit occupied cell centres (larger but simpler).
// JSON schema: { delta, origin[3], n_occupied, centres[[x,y,z],...] }
void export_voxel_centres_json(const voxel::VoxelGrid& grid,
                               const std::string& filepath);

// ─── Scene (Obstacles) Export ───────────────────────────────────────────────
// JSON schema: { obstacles[{ name, center[3], half_sizes[3] }] }
void export_scene_json(const scene::Scene& scene,
                       const std::string& filepath);

// ─── Combined Snapshot Export ───────────────────────────────────────────────
// Export robot FK + envelope iAABBs + voxel grid + scene in a single JSON.
// Produces a self-contained file for the combined Python viewer.
void export_snapshot_json(const Robot& robot,
                          const Eigen::VectorXd& config,
                          const std::vector<Interval>& box_intervals,
                          const voxel::VoxelGrid& robot_grid,
                          const voxel::VoxelGrid& obstacle_grid,
                          const scene::Scene& scene,
                          int n_sub,
                          const std::string& filepath);

// ─── Multi-Method Envelope Comparison Export ────────────────────────────────
// Computes envelope using available methods and exports to one JSON
// for side-by-side comparison with toggleable layers:
//   1. Full Link iAABB (sub=1)   (derive_aabb)
//   2. Link iAABB (sub=n_sub)    (derive_aabb_subdivided)
//   3. Voxel Hull-16             (rasterise_robot_hull16)
//   4. Voxel Link iAABB          (rasterise_robot_sub_aabbs)
//   5. Voxel Full Link iAABB     (rasterise_robot_aabbs)
//   + robot arm FK at box center
//
// JSON schema:
//   { robot_name, n_joints, n_active_links, link_radii[],
//     box_intervals[[lo,hi],...], delta,
//     robot_arm: { link_positions[[x,y,z],...] },
//     methods: {
//       full_link_iaabb:       { type:"aabb",  aabbs:[{link,lo[3],hi[3]}] },
//       link_iaabb:            { type:"aabb",  n_sub, aabbs:[{link,seg,lo[3],hi[3]}] },
//       voxel_hull16:          { type:"voxel", n_occupied, centres[[x,y,z],...] },
//       voxel_link_iaabb:      { type:"voxel", n_occupied, centres[[x,y,z],...] },
//       voxel_full_link_iaabb: { type:"voxel", n_occupied, centres[[x,y,z],...] },
//     }
//   }
void export_envelope_comparison_json(
    const Robot& robot,
    const std::vector<Interval>& box_intervals,
    int n_sub,
    double delta,
    const std::string& filepath);

// ─── Endpoint iAABB Comparison Export ───────────────────────────────────────
// Export per-endpoint position interval iAABBs from multiple Stage-1 sources
// for side-by-side 3D comparison.
//
// sources: list of (source_name, endpoint_iaabbs [n_endpoints × 6]).
//   n_endpoints = robot.n_joints() + (robot.has_tool() ? 1 : 0)
//   endpoint_iaabbs layout: {lo_x, lo_y, lo_z, hi_x, hi_y, hi_z} per endpoint
//
// JSON schema:
//   { robot_name, n_joints, n_active_links, link_radii[], active_link_map[],
//     n_endpoints, box_intervals[[lo,hi],...],
//     robot_arm: { link_positions[[x,y,z],...] },
//     methods: {
//       <source_name>: {
//         endpoint_iaabbs: [{endpoint, lo[3], hi[3]}, ...]
//       }, ...
//     }
//   }
void export_ep_iaabb_comparison_json(
    const Robot& robot,
    const std::vector<Interval>& box_intervals,
    const std::vector<std::pair<std::string, std::vector<float>>>& sources,
    const std::string& filepath);

} // namespace viz
} // namespace sbf
