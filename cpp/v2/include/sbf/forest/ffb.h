// SafeBoxForest v2 — FFB Engine: Find Free Box algorithm wrapper
// Module: sbf::forest
#pragma once

#include "sbf/common/types.h"
#include "sbf/common/config.h"
#include "sbf/forest/hier_aabb_tree.h"

namespace sbf {
namespace forest {

// ─── FFB Engine ─────────────────────────────────────────────────────────────
class FFBEngine {
public:
    FFBEngine() = default;
    FFBEngine(HierAABBTree& tree, const ForestConfig& config);

    // Attempt to create a free box at the given seed
    FFBResult find_free_box(const Eigen::VectorXd& seed,
                            const float* obs_compact, int n_obs,
                            double min_edge_override = -1.0);

    double current_min_edge(double base_min_edge) const;
    void advance_phase();
    void reset_phases();

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
