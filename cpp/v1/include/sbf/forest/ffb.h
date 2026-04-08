// SafeBoxForest — FFB Engine: Find Free Box algorithm wrapper
// Module: sbf::forest
#pragma once

#include "sbf/common/types.h"
#include "sbf/common/config.h"
#include "sbf/cache/hier_aabb_tree.h"

namespace sbf {
namespace forest {

// ─── FFB Engine ─────────────────────────────────────────────────────────────
// Wraps HierAABBTree::find_free_box() with multi-phase min_edge management
// and promotion logic.
class FFBEngine {
public:
    FFBEngine() = default;
    FFBEngine(HierAABBTree& tree, const ForestConfig& config);

    // Attempt to create a free box at the given seed
    // Returns FFBResult with success/failure info
    FFBResult find_free_box(const Eigen::VectorXd& seed,
                            const float* obs_compact, int n_obs,
                            double min_edge_override = -1.0);

    // Get current effective min_edge based on phase
    double current_min_edge(double base_min_edge) const;

    // Advance BFS phase after budget exhausted
    void advance_phase();

    // Reset phase tracking
    void reset_phases();

    // Phase state
    int current_phase() const { return current_phase_; }
    int boxes_in_phase() const { return boxes_in_phase_; }

private:
    HierAABBTree* tree_ = nullptr;
    ForestConfig config_;
    int current_phase_ = 0;
    int boxes_in_phase_ = 0;
};

} // namespace forest
} // namespace sbf
