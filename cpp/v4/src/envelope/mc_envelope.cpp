// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — Monte Carlo link iAABB implementation
// ═══════════════════════════════════════════════════════════════════════════
#include "sbf/envelope/mc_envelope.h"
#include "sbf/robot/fk.h"

#include <Eigen/Core>
#include <limits>
#include <random>

namespace sbf {
namespace envelope {

void compute_mc_link_iaabb(
    const Robot&                 robot,
    const std::vector<Interval>& intervals,
    int                          n_mc,
    uint32_t                     seed,
    float*                       out_iaabb)
{
    const int n_joints     = robot.n_joints();
    const int n_active     = robot.n_active_links();
    const int* alm         = robot.active_link_map();   // int[n_active]: abs link idx
    const double* aradii   = robot.active_link_radii(); // double[n_active]

    // Initialise output to inverted bounds
    for (int a = 0; a < n_active; ++a) {
        out_iaabb[a*6+0] = out_iaabb[a*6+1] = out_iaabb[a*6+2] =
             std::numeric_limits<float>::max();
        out_iaabb[a*6+3] = out_iaabb[a*6+4] = out_iaabb[a*6+5] =
            -std::numeric_limits<float>::max();
    }

    // Per-joint uniform distributions
    std::mt19937 rng(seed);
    std::vector<std::uniform_real_distribution<double>> dists;
    dists.reserve(n_joints);
    for (int j = 0; j < n_joints; ++j)
        dists.emplace_back(intervals[j].lo, intervals[j].hi);

    Eigen::VectorXd q(n_joints);

    for (int s = 0; s < n_mc; ++s) {
        for (int j = 0; j < n_joints; ++j)
            q[j] = dists[j](rng);

        // FK: returns link frame positions, index [0..n_joints] = base..tool
        auto positions = fk_link_positions(robot, q);

        for (int a = 0; a < n_active; ++a) {
            const int link_idx = alm[a];
            const float r = (aradii) ? static_cast<float>(aradii[a]) : 0.0f;

            // Proximal & distal endpoints of link link_idx
            for (int ep = 0; ep < 2; ++ep) {
                const auto& p = positions[link_idx + ep];
                for (int d = 0; d < 3; ++d) {
                    float v = static_cast<float>(p[d]);
                    if (v - r < out_iaabb[a*6 + d])
                        out_iaabb[a*6 + d] = v - r;
                    if (v + r > out_iaabb[a*6 + 3 + d])
                        out_iaabb[a*6 + 3 + d] = v + r;
                }
            }
        }
    }
}

} // namespace envelope
} // namespace sbf
