// SafeBoxForest — Adjacency computation (vectorized O(N²D))
#pragma once

#include "sbf/core/types.h"
#include <Eigen/Core>
#include <utility>
#include <vector>

namespace sbf {

// Compute adjacency pairs from vectorized interval arrays
// ivs_lo, ivs_hi: (N, D) matrices
// Returns list of (i, j) index pairs that are adjacent
std::vector<std::pair<int, int>>
compute_adjacency(const Eigen::MatrixXd& ivs_lo,
                  const Eigen::MatrixXd& ivs_hi,
                  double tol = 1e-10,
                  int chunk_size = 64);

} // namespace sbf
