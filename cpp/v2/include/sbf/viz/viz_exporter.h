// SafeBoxForest v2 — Visualization Exporter
// Module: sbf::viz
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
void export_forest_json(const SafeBoxForest& forest,
                         const std::string& filepath);

void export_forest_json(const SafeBoxForest& forest,
                         const std::vector<Obstacle>& obstacles,
                         const Eigen::MatrixXd& path,
                         const std::string& filepath);

// ─── Export robot FK poses along a path ─────────────────────────────────────
void export_path_frames(const Robot& robot,
                         const Eigen::MatrixXd& path,
                         const std::string& filepath);

// ─── Export box intervals as CSV ────────────────────────────────────────────
void export_boxes_csv(const SafeBoxForest& forest,
                      const std::string& filepath);

} // namespace viz
} // namespace sbf
