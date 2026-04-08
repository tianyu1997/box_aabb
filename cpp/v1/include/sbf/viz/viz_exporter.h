// SafeBoxForest — Visualization Exporter
// Module: sbf::viz
// Exports forest data to JSON for Python visualization scripts.
#pragma once

#include "sbf/common/types.h"
#include "sbf/forest/safe_box_forest.h"
#include "sbf/robot/robot.h"
#include <Eigen/Core>
#include <string>
#include <vector>

namespace sbf {
namespace viz {

// ─── Export forest data to JSON for visualization ───────────────────────────
// Output format:
//   {
//     "boxes": [{id, intervals:[[lo,hi],...], seed:[...]},...],
//     "adjacency": [[a,b],...]
//     "path": [[q0,q1,...], ...],
//     "obstacles": [{name, center:[x,y,z], half:[hx,hy,hz]}, ...]
//   }
void export_forest_json(const SafeBoxForest& forest,
                         const std::string& filepath);

void export_forest_json(const SafeBoxForest& forest,
                         const std::vector<Obstacle>& obstacles,
                         const Eigen::MatrixXd& path,
                         const std::string& filepath);

// ─── Export robot FK poses along a path ─────────────────────────────────────
// Each row in path is a configuration. Output:
//   { "frames": [{"q":[...], "link_positions":[[x,y,z],...]}, ...] }
void export_path_frames(const Robot& robot,
                         const Eigen::MatrixXd& path,
                         const std::string& filepath);

// ─── Export box intervals as CSV ────────────────────────────────────────────
void export_boxes_csv(const SafeBoxForest& forest,
                      const std::string& filepath);

} // namespace viz
} // namespace sbf
