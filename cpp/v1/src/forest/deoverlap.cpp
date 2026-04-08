// SafeBoxForest — Adjacency computation
// Optimised: (a) row-major flat arrays, (b) dual-dim sweep, (c) dim-order early exit
#include "sbf/forest/deoverlap.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>

namespace sbf {

std::vector<std::pair<int, int>>
compute_adjacency(const Eigen::MatrixXd& ivs_lo,
                  const Eigen::MatrixXd& ivs_hi,
                  double tol,
                  int /*chunk_size*/) {
    const int N = static_cast<int>(ivs_lo.rows());
    const int D = static_cast<int>(ivs_lo.cols());
    std::vector<std::pair<int, int>> pairs;
    if (N <= 1) return pairs;

    // ── (a) Copy to row-major flat arrays for cache-friendliness ──────
    // Eigen default is column-major; ivs_lo(i,d) stride=N → cache miss per dim.
    // Flat row-major: lo[i*D+d] → 7 dims in 56 bytes = 1 cache line.
    std::vector<double> lo(N * D), hi(N * D);
    for (int i = 0; i < N; ++i)
        for (int d = 0; d < D; ++d) {
            lo[i * D + d] = ivs_lo(i, d);
            hi[i * D + d] = ivs_hi(i, d);
        }

    // ── Rank dimensions by "fill ratio" (avg_width / range) ───────────
    // Lower ratio → narrower boxes relative to space → better pruning
    std::vector<std::pair<double, int>> dim_ranks(D);
    for (int d = 0; d < D; ++d) {
        double total_w = 0, rng_lo = lo[d], rng_hi = hi[d];
        for (int i = 0; i < N; ++i) {
            total_w += hi[i * D + d] - lo[i * D + d];
            rng_lo = std::min(rng_lo, lo[i * D + d]);
            rng_hi = std::max(rng_hi, hi[i * D + d]);
        }
        dim_ranks[d] = {total_w / std::max(rng_hi - rng_lo, 1e-15), d};
    }
    std::sort(dim_ranks.begin(), dim_ranks.end());

    // sweep_dim = narrowest fill ratio (primary sort + break)
    // filter_dim = 2nd narrowest (continue-based pre-filter)
    const int sweep_dim = dim_ranks[0].second;
    const int filter_dim = (D >= 2) ? dim_ranks[1].second : -1;

    // Remaining dims ordered narrowest-first for early-exit in full check
    std::vector<int> rest_dims;
    rest_dims.reserve(D);
    for (int k = 2; k < D; ++k)
        rest_dims.push_back(dim_ranks[k].second);

    // ── Sort box indices by lo[sweep_dim] ─────────────────────────────
    std::vector<int> order(N);
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return lo[a * D + sweep_dim] < lo[b * D + sweep_dim];
    });

    // ── (b,c) Dual-dim sweep ──────────────────────────────────────────
    long long n_sweep_cand = 0, n_filter_cand = 0;
    auto t0 = std::chrono::high_resolution_clock::now();

    pairs.reserve(N * 4);  // heuristic pre-alloc

    for (int ii = 0; ii < N; ++ii) {
        const int i = order[ii];
        const double* lo_i = &lo[i * D];
        const double* hi_i = &hi[i * D];
        const double hi_i_sweep = hi_i[sweep_dim] + tol;

        // Pre-fetch filter_dim bounds for box i
        const double lo_i_f = (filter_dim >= 0) ? lo_i[filter_dim] - tol : 0;
        const double hi_i_f = (filter_dim >= 0) ? hi_i[filter_dim] + tol : 0;

        for (int jj = ii + 1; jj < N; ++jj) {
            const int j = order[jj];
            const double* lo_j = &lo[j * D];
            const double* hi_j = &hi[j * D];

            // Primary sweep dim: break (sorted → no more candidates)
            if (lo_j[sweep_dim] > hi_i_sweep)
                break;

            ++n_sweep_cand;

            // (c) Secondary filter dim: continue (not sorted, just skip)
            if (filter_dim >= 0) {
                if (lo_j[filter_dim] > hi_i_f || hi_j[filter_dim] < lo_i_f)
                    continue;
            }

            ++n_filter_cand;

            // Full D-dim adjacency check on remaining dims
            bool adjacent = true;
            bool has_touching = false;

            // Check sweep dim
            {
                double ov = std::min(hi_i[sweep_dim], hi_j[sweep_dim])
                          - std::max(lo_i[sweep_dim], lo_j[sweep_dim]);
                // ov >= -tol guaranteed by sweep break
                if (ov <= tol) has_touching = true;
            }

            // Check filter dim
            if (filter_dim >= 0) {
                double ov = std::min(hi_i[filter_dim], hi_j[filter_dim])
                          - std::max(lo_i[filter_dim], lo_j[filter_dim]);
                // ov >= -tol guaranteed by filter continue
                if (ov <= tol) has_touching = true;
            }

            // Check remaining dims (ordered narrowest → widest for early exit)
            for (int rd : rest_dims) {
                double ov = std::min(hi_i[rd], hi_j[rd])
                          - std::max(lo_i[rd], lo_j[rd]);
                if (ov < -tol) {
                    adjacent = false;
                    break;
                }
                if (ov <= tol) has_touching = true;
            }

            if (adjacent && has_touching) {
                pairs.emplace_back(i, j);
            }
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    double dt = std::chrono::duration<double>(t1 - t0).count();
    long long brute = (long long)N * (N - 1) / 2;
    std::cout << "  [adjacency-diag] N=" << N
              << " sweep=" << sweep_dim << " filter=" << filter_dim
              << " sweep_cand=" << n_sweep_cand
              << " filter_cand=" << n_filter_cand
              << " pairs=" << pairs.size()
              << " brute=" << brute
              << " sweep_prune=" << (1.0 - (double)n_sweep_cand / brute) * 100.0 << "%"
              << " dual_prune=" << (1.0 - (double)n_filter_cand / brute) * 100.0 << "%"
              << " time=" << dt << "s" << std::endl;

    return pairs;
}

} // namespace sbf
