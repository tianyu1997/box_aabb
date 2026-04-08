// SafeBoxForest — Root Sampler implementation
#include "sbf/forest/root_sampler.h"

namespace sbf {
namespace forest {

RootSampler::RootSampler(const JointLimits& limits, unsigned seed)
    : limits_(limits), rng_(seed), n_dims_(limits.n_dims()) {}

Eigen::VectorXd RootSampler::sample_uniform() {
    Eigen::VectorXd q(n_dims_);
    for (int d = 0; d < n_dims_; ++d) {
        std::uniform_real_distribution<double> dist(limits_.limits[d].lo,
                                                     limits_.limits[d].hi);
        q[d] = dist(rng_);
    }
    return q;
}

Eigen::VectorXd RootSampler::sample_guided(const Eigen::VectorXd& goal,
                                             double guided_ratio) {
    std::uniform_real_distribution<double> coin(0.0, 1.0);
    if (coin(rng_) < guided_ratio) {
        // Goal-biased: sample near the goal
        Eigen::VectorXd q(n_dims_);
        for (int d = 0; d < n_dims_; ++d) {
            double range = limits_.limits[d].width() * 0.3;
            std::uniform_real_distribution<double> dist(
                std::max(limits_.limits[d].lo, goal[d] - range),
                std::min(limits_.limits[d].hi, goal[d] + range));
            q[d] = dist(rng_);
        }
        return q;
    }
    return sample_uniform();
}

} // namespace forest
} // namespace sbf
