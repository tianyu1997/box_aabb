// SafeBoxForest — FFB Engine implementation
#include "sbf/forest/ffb.h"

namespace sbf {
namespace forest {

FFBEngine::FFBEngine(HierAABBTree& tree, const ForestConfig& config)
    : tree_(&tree), config_(config), current_phase_(0), boxes_in_phase_(0) {}

FFBResult FFBEngine::find_free_box(const Eigen::VectorXd& seed,
                                    const float* obs_compact, int n_obs,
                                    double min_edge_override)
{
    double min_edge = min_edge_override > 0 ? min_edge_override
                                             : current_min_edge(0.01); // default
    auto result = tree_->find_free_box(seed, obs_compact, n_obs,
                                       1000, min_edge);
    if (result.success()) {
        boxes_in_phase_++;
        // Check if we should advance phase
        if (current_phase_ < static_cast<int>(config_.bfs_phase_budget.size()) &&
            boxes_in_phase_ >= config_.bfs_phase_budget[current_phase_]) {
            advance_phase();
        }
    }
    return result;
}

double FFBEngine::current_min_edge(double base_min_edge) const {
    if (current_phase_ < static_cast<int>(config_.bfs_phase_k.size()))
        return base_min_edge * config_.bfs_phase_k[current_phase_];
    return base_min_edge;
}

void FFBEngine::advance_phase() {
    current_phase_++;
    boxes_in_phase_ = 0;
}

void FFBEngine::reset_phases() {
    current_phase_ = 0;
    boxes_in_phase_ = 0;
}

} // namespace forest
} // namespace sbf
