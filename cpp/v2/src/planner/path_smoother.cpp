// SafeBoxForest v2 — PathSmoother implementation
// Module: sbf (planner)
#include "sbf/planner/path_smoother.h"
#include <algorithm>
#include <cmath>
#include <random>

namespace sbf {

PathSmoother::PathSmoother(const CollisionChecker& checker,
                            double segment_resolution)
    : checker_(&checker), segment_resolution_(segment_resolution) {}

double PathSmoother::path_length(const std::vector<Eigen::VectorXd>& path) {
    double len = 0.0;
    for (size_t i = 1; i < path.size(); ++i)
        len += (path[i] - path[i - 1]).norm();
    return len;
}

std::vector<Eigen::VectorXd>
PathSmoother::shortcut(const std::vector<Eigen::VectorXd>& path,
                        int max_iters, std::mt19937* rng) const {
    if (path.size() <= 2) return path;

    std::mt19937 local_rng(42);
    if (!rng) rng = &local_rng;

    std::vector<Eigen::VectorXd> current = path;

    for (int iter = 0; iter < max_iters; ++iter) {
        if (current.size() <= 2) break;

        int n = static_cast<int>(current.size());
        std::uniform_int_distribution<int> dist(0, n - 1);
        int i = dist(*rng);
        int j = dist(*rng);
        if (i > j) std::swap(i, j);
        if (j - i < 2) continue;

        if (!checker_->check_segment(current[i], current[j], segment_resolution_)) {
            std::vector<Eigen::VectorXd> shortened;
            for (int k = 0; k <= i; ++k) shortened.push_back(current[k]);
            for (int k = j; k < n; ++k) shortened.push_back(current[k]);
            current = std::move(shortened);
        }
    }
    return current;
}

std::vector<Eigen::VectorXd>
PathSmoother::box_aware_shortcut(const std::vector<Eigen::VectorXd>& path,
                                  const SafeBoxForest& forest,
                                  int max_iters, std::mt19937* rng) const {
    if (path.size() <= 2) return path;

    std::mt19937 local_rng(42);
    if (!rng) rng = &local_rng;

    std::vector<Eigen::VectorXd> current = path;

    for (int iter = 0; iter < max_iters; ++iter) {
        if (current.size() <= 2) break;

        int n = static_cast<int>(current.size());
        std::uniform_int_distribution<int> dist(0, n - 1);
        int i = dist(*rng);
        int j = dist(*rng);
        if (i > j) std::swap(i, j);
        if (j - i < 2) continue;

        const BoxNode* box_i = forest.find_containing(current[i]);
        const BoxNode* box_j = forest.find_containing(current[j]);

        bool safe_shortcut = false;
        if (box_i && box_j) {
            if (box_i->id == box_j->id) {
                safe_shortcut = true;
            } else {
                auto adj = forest.adjacency();
                auto it = adj.find(box_i->id);
                if (it != adj.end() &&
                    std::find(it->second.begin(), it->second.end(), box_j->id) != it->second.end()) {
                    safe_shortcut = true;
                }
            }
        }

        if (safe_shortcut) {
            std::vector<Eigen::VectorXd> shortened;
            for (int k = 0; k <= i; ++k) shortened.push_back(current[k]);
            for (int k = j; k < n; ++k) shortened.push_back(current[k]);
            current = std::move(shortened);
        } else if (!checker_->check_segment(current[i], current[j], segment_resolution_)) {
            std::vector<Eigen::VectorXd> shortened;
            for (int k = 0; k <= i; ++k) shortened.push_back(current[k]);
            for (int k = j; k < n; ++k) shortened.push_back(current[k]);
            current = std::move(shortened);
        }
    }
    return current;
}

std::vector<Eigen::VectorXd>
PathSmoother::smooth_moving_average(const std::vector<Eigen::VectorXd>& path,
                                     const SafeBoxForest& forest,
                                     int window, int iterations) const {
    if (path.size() <= 2) return path;

    std::vector<Eigen::VectorXd> current = path;
    int half = window / 2;

    for (int iter = 0; iter < iterations; ++iter) {
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
                count++;
            }
            avg /= count;

            const BoxNode* box = forest.find_containing(current[i]);
            if (box) {
                avg = box->nearest_point_to(avg);
            }
            smoothed[i] = avg;
        }
        current = std::move(smoothed);
    }
    return current;
}

std::vector<Eigen::VectorXd>
PathSmoother::resample(const std::vector<Eigen::VectorXd>& path,
                        double resolution) const {
    if (path.size() <= 1) return path;

    double total = path_length(path);
    if (total < 1e-12) return path;

    int n_samples = std::max(2, static_cast<int>(std::ceil(total / resolution)) + 1);
    double step = total / (n_samples - 1);

    std::vector<Eigen::VectorXd> resampled;
    resampled.push_back(path[0]);

    double target = step;
    int seg = 0;
    double seg_start = 0.0;

    while (static_cast<int>(resampled.size()) < n_samples - 1 &&
           seg < static_cast<int>(path.size()) - 1) {
        double seg_len = (path[seg + 1] - path[seg]).norm();

        while (target <= seg_start + seg_len + 1e-12 &&
               static_cast<int>(resampled.size()) < n_samples - 1) {
            double t = (target - seg_start) / seg_len;
            t = std::clamp(t, 0.0, 1.0);
            resampled.push_back(path[seg] + t * (path[seg + 1] - path[seg]));
            target += step;
        }
        seg_start += seg_len;
        seg++;
    }

    resampled.push_back(path.back());
    return resampled;
}

} // namespace sbf
