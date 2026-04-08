// SafeBoxForest v2 — Boundary seed sampler for directed expansion
// Module: sbf::forest
#pragma once

#include "sbf/common/types.h"
#include <Eigen/Core>
#include <random>
#include <set>
#include <vector>

namespace sbf {
namespace forest {

// ─── Boundary Seed ──────────────────────────────────────────────────────────
struct BoundarySeed {
    int dim;
    int side;  // 0=lo, 1=hi
    Eigen::VectorXd config;
};

// ─── Boundary Sampler ───────────────────────────────────────────────────────
class BoundarySampler {
public:
    BoundarySampler() = default;
    BoundarySampler(const JointLimits& limits, double epsilon = 0.01);

    // Generate boundary seeds from a given box
    std::vector<BoundarySeed> generate_boundary_seeds(
        const BoxNode& box,
        const std::set<std::pair<int,int>>& excluded_faces,
        const Eigen::VectorXd* goal_point,
        int n_samples,
        std::mt19937& rng,
        double eps_override = -1.0) const;

private:
    JointLimits limits_;
    double epsilon_ = 0.01;
};

} // namespace forest
} // namespace sbf
