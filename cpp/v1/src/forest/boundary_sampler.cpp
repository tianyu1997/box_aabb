// SafeBoxForest — Boundary Sampler implementation
#include "sbf/forest/boundary_sampler.h"
#include <algorithm>

namespace sbf {
namespace forest {

BoundarySampler::BoundarySampler(const JointLimits& limits, double epsilon)
    : limits_(limits), epsilon_(epsilon) {}

std::vector<BoundarySeed> BoundarySampler::generate_boundary_seeds(
    const BoxNode& box,
    const std::set<std::pair<int,int>>& excluded_faces,
    const Eigen::VectorXd* goal_point,
    int n_samples,
    std::mt19937& rng,
    double eps_override) const
{
    double eps = eps_override > 0 ? eps_override : epsilon_;
    int n_dims = box.n_dims();
    std::vector<BoundarySeed> seeds;

    // Collect available faces
    std::vector<std::pair<int,int>> faces;
    for (int d = 0; d < n_dims; ++d) {
        for (int side = 0; side < 2; ++side) {
            if (excluded_faces.count({d, side})) continue;
            // Check joint limit: don't expand beyond limits
            double boundary = (side == 0) ? box.joint_intervals[d].lo
                                          : box.joint_intervals[d].hi;
            double limit = (side == 0) ? limits_.limits[d].lo
                                       : limits_.limits[d].hi;
            if ((side == 0 && boundary - eps < limit) ||
                (side == 1 && boundary + eps > limit))
                continue;
            faces.push_back({d, side});
        }
    }

    if (faces.empty()) return seeds;

    // Goal-biased face selection (60% goal-facing, 40% random)
    for (int i = 0; i < n_samples && !faces.empty(); ++i) {
        int fi;
        if (goal_point) {
            std::uniform_real_distribution<double> coin(0.0, 1.0);
            if (coin(rng) < 0.6) {
                // Pick face closest to goal
                double best_score = -1e30;
                fi = 0;
                for (int j = 0; j < static_cast<int>(faces.size()); ++j) {
                    auto [d, side] = faces[j];
                    double dir = (side == 0) ? -1.0 : 1.0;
                    double goal_dir = (*goal_point)[d] - box.joint_intervals[d].center();
                    double score = dir * goal_dir;
                    if (score > best_score) {
                        best_score = score;
                        fi = j;
                    }
                }
            } else {
                std::uniform_int_distribution<int> idx(0, static_cast<int>(faces.size()) - 1);
                fi = idx(rng);
            }
        } else {
            std::uniform_int_distribution<int> idx(0, static_cast<int>(faces.size()) - 1);
            fi = idx(rng);
        }

        auto [dim, side] = faces[fi];

        // Create seed at face center + epsilon offset
        BoundarySeed seed;
        seed.dim = dim;
        seed.side = side;
        seed.config = box.center();

        if (side == 0)
            seed.config[dim] = box.joint_intervals[dim].lo - eps;
        else
            seed.config[dim] = box.joint_intervals[dim].hi + eps;

        // Clamp to joint limits
        for (int d = 0; d < n_dims; ++d)
            seed.config[d] = std::clamp(seed.config[d],
                                         limits_.limits[d].lo,
                                         limits_.limits[d].hi);

        seeds.push_back(std::move(seed));
    }

    return seeds;
}

} // namespace forest
} // namespace sbf
