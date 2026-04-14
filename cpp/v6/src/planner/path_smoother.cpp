// SafeBoxForest v6 — Path Smoother (Phase H4)
#include <sbf/planner/path_smoother.h>

#include <algorithm>
#include <random>

namespace sbf {

double path_length(const std::vector<Eigen::VectorXd>& path) {
    double len = 0.0;
    for (size_t i = 1; i < path.size(); ++i)
        len += (path[i] - path[i - 1]).norm();
    return len;
}

static Eigen::VectorXd clamp_to_box(const Eigen::VectorXd& q,
                                     const BoxNode& box) {
    Eigen::VectorXd c = q;
    for (int d = 0; d < box.n_dims(); ++d) {
        c[d] = std::max(c[d], box.joint_intervals[d].lo);
        c[d] = std::min(c[d], box.joint_intervals[d].hi);
    }
    return c;
}

std::vector<Eigen::VectorXd> shortcut(
    const std::vector<Eigen::VectorXd>& path,
    const CollisionChecker& checker,
    const SmootherConfig& config,
    uint64_t seed)
{
    if (path.size() <= 2) return path;

    std::mt19937_64 rng(seed);
    std::vector<Eigen::VectorXd> current = path;

    for (int iter = 0; iter < config.shortcut_max_iters; ++iter) {
        if (current.size() <= 2) break;

        int n = static_cast<int>(current.size());
        std::uniform_int_distribution<int> dist_i(0, n - 3);
        int i = dist_i(rng);
        std::uniform_int_distribution<int> dist_j(i + 2, n - 1);
        int j = dist_j(rng);

        if (!checker.check_segment(current[i], current[j],
                                   config.segment_resolution)) {
            // Direct connection is collision-free → remove intermediate
            std::vector<Eigen::VectorXd> shortened;
            for (int k = 0; k <= i; ++k) shortened.push_back(current[k]);
            for (int k = j; k < n; ++k) shortened.push_back(current[k]);
            current = std::move(shortened);
        }
    }
    return current;
}

std::vector<Eigen::VectorXd> smooth_moving_average(
    const std::vector<Eigen::VectorXd>& path,
    const std::vector<BoxNode>& box_sequence,
    const SmootherConfig& config)
{
    if (path.size() <= 2) return path;

    std::vector<Eigen::VectorXd> current = path;
    int half = config.smooth_window / 2;

    for (int iter = 0; iter < config.smooth_iters; ++iter) {
        int n = static_cast<int>(current.size());
        std::vector<Eigen::VectorXd> smoothed(n);
        smoothed[0] = current[0];
        smoothed[n - 1] = current[n - 1];

        for (int i = 1; i < n - 1; ++i) {
            int lo = std::max(0, i - half);
            int hi = std::min(n - 1, i + half);
            Eigen::VectorXd avg = Eigen::VectorXd::Zero(current[0].size());
            int count = 0;
            for (int k = lo; k <= hi; ++k) {
                avg += current[k];
                ++count;
            }
            avg /= count;

            // Clamp to corresponding box if available
            if (!box_sequence.empty()) {
                // Map waypoint index to box: waypoint i corresponds to
                // box_sequence[min(i, box_sequence.size()-1)]
                int box_idx = std::min(i, static_cast<int>(box_sequence.size()) - 1);
                avg = clamp_to_box(avg, box_sequence[box_idx]);
            }
            smoothed[i] = avg;
        }
        current = std::move(smoothed);
    }
    return current;
}

std::vector<Eigen::VectorXd> smooth_path(
    const std::vector<Eigen::VectorXd>& path,
    const std::vector<BoxNode>& box_sequence,
    const CollisionChecker& checker,
    const SmootherConfig& config)
{
    auto result = shortcut(path, checker, config);
    result = smooth_moving_average(result, box_sequence, config);
    return result;
}

}  // namespace sbf
