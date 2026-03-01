// SafeBoxForest — TreeConnector: endpoint attachment and inter-tree connections
#pragma once

#include "sbf/core/types.h"
#include "sbf/forest/collision.h"
#include "sbf/forest/safe_box_forest.h"
#include <Eigen/Core>
#include <vector>

namespace sbf {

class TreeConnector {
public:
    TreeConnector() = default;
    TreeConnector(const SafeBoxForest& forest,
                  const CollisionChecker& checker,
                  double connection_radius = 2.0,
                  double segment_resolution = 0.05,
                  int max_attempts = 50);

    // ── Connect start/goal to forest ─────────────────────────────────────
    // Returns the box IDs that contain or are nearest to the config
    struct AttachResult {
        int box_id = -1;
        bool inside_box = false;
        Eigen::VectorXd attach_point;
    };

    AttachResult attach_config(const Eigen::VectorXd& q) const;

    // ── Connect within trees ─────────────────────────────────────────────
    // Find edges between pairs of boxes in different trees
    std::vector<Edge> connect_between_trees() const;

private:
    const SafeBoxForest* forest_ = nullptr;
    const CollisionChecker* checker_ = nullptr;
    double connection_radius_ = 2.0;
    double segment_resolution_ = 0.05;
    int max_attempts_ = 50;
};

} // namespace sbf
