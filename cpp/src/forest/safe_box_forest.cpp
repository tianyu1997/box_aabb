// SafeBoxForest — SafeBoxForest implementation
#include "sbf/forest/safe_box_forest.h"
#include "sbf/forest/deoverlap.h"
#include <algorithm>
#include <chrono>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>

namespace sbf {

SafeBoxForest::SafeBoxForest(int n_dims, const JointLimits& limits)
    : n_dims_(n_dims), limits_(limits) {}

int SafeBoxForest::allocate_id() {
    return next_id_++;
}

void SafeBoxForest::add_box_direct(const BoxNode& box) {
    boxes_[box.id] = box;

    // Incremental adjacency: check against all existing boxes
    for (auto& [other_id, other_box] : boxes_) {
        if (other_id == box.id) continue;
        if (check_adjacency(box, other_box, 1e-10)) {
            add_adjacency_edge(box.id, other_id);
        }
    }
}

void SafeBoxForest::add_box_no_adjacency(const BoxNode& box) {
    boxes_[box.id] = box;
}

void SafeBoxForest::remove_boxes(const std::unordered_set<int>& box_ids) {
    for (int id : box_ids) {
        remove_adjacency_node(id);
        boxes_.erase(id);
    }
}

void SafeBoxForest::remove_boxes_no_adjacency(const std::unordered_set<int>& box_ids) {
    for (int id : box_ids)
        boxes_.erase(id);
}

void SafeBoxForest::clear() {
    boxes_.clear();
    adjacency_.clear();
    // Note: next_id_ is NOT reset — IDs are never reused to prevent aliasing
}

void SafeBoxForest::rebuild_adjacency(double tol) {
    auto ta = std::chrono::high_resolution_clock::now();
    adjacency_.clear();

    // Initialize empty adjacency sets for all boxes
    for (auto& [id, _] : boxes_)
        adjacency_[id] = {};

    // Build vectorized interval cache
    rebuild_interval_cache();
    auto tb = std::chrono::high_resolution_clock::now();

    // Use vectorized adjacency computation
    auto pairs = compute_adjacency(ivs_lo_, ivs_hi_, tol);
    auto tc = std::chrono::high_resolution_clock::now();

    for (auto& [i, j] : pairs) {
        int id_a = ivs_ids_[i];
        int id_b = ivs_ids_[j];
        adjacency_[id_a].push_back(id_b);
        adjacency_[id_b].push_back(id_a);
    }
    auto td = std::chrono::high_resolution_clock::now();
    std::cout << "  [rebuild_adj] cache="
              << std::chrono::duration<double>(tb - ta).count() << "s"
              << " compute=" << std::chrono::duration<double>(tc - tb).count() << "s"
              << " map_insert=" << std::chrono::duration<double>(td - tc).count() << "s"
              << " total=" << std::chrono::duration<double>(td - ta).count() << "s"
              << " pairs=" << pairs.size() << std::endl;
}

void SafeBoxForest::rebuild_interval_cache() {
    int n = n_boxes();
    ivs_lo_.resize(n, n_dims_);
    ivs_hi_.resize(n, n_dims_);
    ivs_ids_.resize(n);

    int row = 0;
    for (auto& [id, box] : boxes_) {
        ivs_ids_[row] = id;
        for (int d = 0; d < n_dims_; ++d) {
            ivs_lo_(row, d) = box.joint_intervals[d].lo;
            ivs_hi_(row, d) = box.joint_intervals[d].hi;
        }
        ++row;
    }
}

const BoxNode* SafeBoxForest::find_containing(const Eigen::VectorXd& q) const {
    for (auto& [id, box] : boxes_) {
        if (box.contains(q))
            return &box;
    }
    return nullptr;
}

const BoxNode* SafeBoxForest::find_nearest(const Eigen::VectorXd& q) const {
    const BoxNode* best = nullptr;
    double best_dist = std::numeric_limits<double>::max();

    for (auto& [id, box] : boxes_) {
        double dist = box.distance_to_config(q);
        if (dist < best_dist) {
            best_dist = dist;
            best = &box;
        }
    }
    return best;
}

std::vector<Eigen::VectorXd> SafeBoxForest::get_uncovered_seeds(int n, std::mt19937& rng) const {
    std::vector<Eigen::VectorXd> seeds;
    seeds.reserve(n);

    std::uniform_real_distribution<double> dist(0.0, 1.0);

    int max_attempts = n * 10;
    for (int attempt = 0; attempt < max_attempts && static_cast<int>(seeds.size()) < n; ++attempt) {
        Eigen::VectorXd q(n_dims_);
        for (int d = 0; d < n_dims_; ++d) {
            q[d] = limits_.limits[d].lo + dist(rng) * limits_.limits[d].width();
        }
        if (!find_containing(q))
            seeds.push_back(q);
    }
    return seeds;
}

std::unordered_set<int> SafeBoxForest::validate_boxes(const CollisionChecker& checker) {
    std::unordered_set<int> invalid;
    for (auto& [id, box] : boxes_) {
        if (checker.check_box(box.joint_intervals))
            invalid.insert(id);
    }
    return invalid;
}

std::unordered_set<int> SafeBoxForest::invalidate_against_obstacle(
    const Obstacle& obs, const Robot& robot, double safety_margin) {
    (void)obs; (void)robot; (void)safety_margin;
    // TODO: implement incremental obstacle collision check
    std::unordered_set<int> invalid;
    return invalid;
}

void SafeBoxForest::validate_invariants(double tol) const {
    // Check adjacency symmetry
    for (auto& [id, neighbors] : adjacency_) {
        for (int nb : neighbors) {
            auto it = adjacency_.find(nb);
            if (it == adjacency_.end() ||
                std::find(it->second.begin(), it->second.end(), id) == it->second.end()) {
                throw std::runtime_error(
                    "Adjacency asymmetry: " + std::to_string(id) +
                    " → " + std::to_string(nb) + " but not reverse");
            }
        }
    }

    // Check no deep overlaps (boxes should only touch, not deeply overlap)
    (void)tol;
}

double SafeBoxForest::total_volume() const {
    double vol = 0.0;
    for (auto& [id, box] : boxes_)
        vol += box.volume;
    return vol;
}

bool SafeBoxForest::check_adjacency(const BoxNode& a, const BoxNode& b, double tol) const {
    return a.is_adjacent_to(b, tol);
}

void SafeBoxForest::add_adjacency_edge(int a, int b) {
    adjacency_[a].push_back(b);
    adjacency_[b].push_back(a);
}

void SafeBoxForest::remove_adjacency_node(int id) {
    auto it = adjacency_.find(id);
    if (it != adjacency_.end()) {
        for (int nb : it->second) {
            auto& vec = adjacency_[nb];
            vec.erase(std::remove(vec.begin(), vec.end(), id), vec.end());
        }
        adjacency_.erase(it);
    }
}

} // namespace sbf
