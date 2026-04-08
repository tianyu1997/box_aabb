// SafeBoxForest — Root seed sampler for forest growth
// Module: sbf::forest
#pragma once

#include "sbf/common/types.h"
#include "sbf/common/config.h"
#include <Eigen/Core>
#include <random>

namespace sbf {
namespace forest {

// ─── Root Sampler ───────────────────────────────────────────────────────────
// Generates seed configurations for new box creation.
// Supports uniform random sampling and goal-biased sampling.
class RootSampler {
public:
    RootSampler() = default;
    RootSampler(const JointLimits& limits, unsigned seed = 0);

    // Sample uniformly from joint limits
    Eigen::VectorXd sample_uniform();

    // Sample with goal bias: probability `guided_ratio` toward goal
    Eigen::VectorXd sample_guided(const Eigen::VectorXd& goal,
                                   double guided_ratio = 0.6);

    // Access RNG for external use
    std::mt19937& rng() { return rng_; }

    // Reset RNG seed
    void set_seed(unsigned seed) { rng_.seed(seed); }

private:
    JointLimits limits_;
    std::mt19937 rng_;
    int n_dims_ = 0;
};

} // namespace forest
} // namespace sbf
