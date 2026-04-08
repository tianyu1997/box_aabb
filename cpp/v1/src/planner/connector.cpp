// SafeBoxForest — TreeConnector implementation
#include "sbf/planner/connector.h"
#include <limits>

namespace sbf {

TreeConnector::TreeConnector(const SafeBoxForest& forest,
                              const CollisionChecker& checker,
                              double connection_radius,
                              double segment_resolution,
                              int max_attempts)
    : forest_(&forest), checker_(&checker),
      connection_radius_(connection_radius),
      segment_resolution_(segment_resolution),
      max_attempts_(max_attempts) {}

TreeConnector::AttachResult TreeConnector::attach_config(const Eigen::VectorXd& q) const {
    AttachResult result;

    // First check if inside any box
    const BoxNode* containing = forest_->find_containing(q);
    if (containing) {
        result.box_id = containing->id;
        result.inside_box = true;
        result.attach_point = q;
        return result;
    }

    // Find nearest box
    const BoxNode* nearest = forest_->find_nearest(q);
    if (nearest) {
        result.box_id = nearest->id;
        result.inside_box = false;
        result.attach_point = nearest->nearest_point_to(q);

        // Verify the connection segment is collision-free
        if (checker_->check_segment(q, result.attach_point, segment_resolution_)) {
            // Connection is blocked — try other nearby boxes
            double best_dist = std::numeric_limits<double>::max();
            for (auto& [id, box] : forest_->boxes()) {
                double dist = box.distance_to_config(q);
                if (dist >= best_dist || dist > connection_radius_) continue;

                Eigen::VectorXd pt = box.nearest_point_to(q);
                if (!checker_->check_segment(q, pt, segment_resolution_)) {
                    result.box_id = id;
                    result.attach_point = pt;
                    best_dist = dist;
                }
            }
        }
    }

    return result;
}

std::vector<Edge> TreeConnector::connect_between_trees() const {
    // TODO: implement inter-tree connection for multi-tree mode
    return {};
}

} // namespace sbf
