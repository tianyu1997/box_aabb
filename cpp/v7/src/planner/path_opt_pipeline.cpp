/// @file path_opt_pipeline.cpp
#include "sbf/planner/path_opt_pipeline.h"

#include <algorithm>
#include <cmath>
#include <random>

namespace sbf::planner {

double PathOptPipeline::path_length(
    const std::vector<Eigen::VectorXd>& path) {
    double L = 0.0;
    for (size_t i = 1; i < path.size(); ++i)
        L += (path[i] - path[i - 1]).norm();
    return L;
}

bool PathOptPipeline::is_segment_free(
    const Eigen::VectorXd& a, const Eigen::VectorXd& b) const {
    double L = (b - a).norm();
    int n_steps = std::max(1, static_cast<int>(std::ceil(L / cfg_.seg_check_dt)));
    // Endpoints assumed free; sample interior.
    for (int k = 0; k <= n_steps; ++k) {
        double t = static_cast<double>(k) / n_steps;
        Eigen::VectorXd q = (1.0 - t) * a + t * b;
        if (!is_free_(q)) return false;
    }
    return true;
}

bool PathOptPipeline::is_path_free(
    const std::vector<Eigen::VectorXd>& path) const {
    for (size_t i = 1; i < path.size(); ++i)
        if (!is_segment_free(path[i - 1], path[i])) return false;
    return true;
}

// ── Step 1: greedy shortcut ─────────────────────────────────────────
std::vector<Eigen::VectorXd> PathOptPipeline::step_greedy_shortcut(
    const std::vector<Eigen::VectorXd>& p) {
    if (p.size() < 3) return p;
    std::vector<Eigen::VectorXd> out;
    out.reserve(p.size());
    out.push_back(p.front());
    size_t i = 0;
    while (i < p.size() - 1) {
        size_t j = p.size() - 1;
        while (j > i + 1) {
            if (is_segment_free(p[i], p[j])) break;
            --j;
        }
        out.push_back(p[j]);
        i = j;
    }
    return out;
}

// ── Step 2/5: randomised shortcut ──────────────────────────────────
std::vector<Eigen::VectorXd> PathOptPipeline::step_shortcut(
    const std::vector<Eigen::VectorXd>& p, int iters) {
    if (p.size() < 3 || iters <= 0) return p;
    std::vector<Eigen::VectorXd> out = p;
    std::mt19937_64 rng(cfg_.rng_seed);
    for (int it = 0; it < iters; ++it) {
        if (out.size() < 4) break;
        // Pick two random waypoints i < j with j - i >= 2.
        std::uniform_int_distribution<int> di(0, (int)out.size() - 3);
        int i = di(rng);
        std::uniform_int_distribution<int> dj(i + 2, (int)out.size() - 1);
        int j = dj(rng);
        if (is_segment_free(out[i], out[j])) {
            out.erase(out.begin() + i + 1, out.begin() + j);
        }
    }
    return out;
}

// ── Step 3: densify ─────────────────────────────────────────────────
std::vector<Eigen::VectorXd> PathOptPipeline::step_densify(
    const std::vector<Eigen::VectorXd>& p) {
    if (p.size() < 2) return p;
    std::vector<Eigen::VectorXd> out;
    out.push_back(p.front());
    for (size_t i = 1; i < p.size(); ++i) {
        const auto& a = p[i - 1];
        const auto& b = p[i];
        double L = (b - a).norm();
        int n = std::max(1, static_cast<int>(std::ceil(L / cfg_.densify_dt)));
        for (int k = 1; k <= n; ++k) {
            double t = static_cast<double>(k) / n;
            out.push_back((1.0 - t) * a + t * b);
        }
    }
    return out;
}

// ── Step 4: elastic-band Laplacian smoothing ───────────────────────
std::vector<Eigen::VectorXd> PathOptPipeline::step_elastic_band(
    const std::vector<Eigen::VectorXd>& p) {
    if (p.size() < 3) return p;
    std::vector<Eigen::VectorXd> cur = p;
    const double a = cfg_.elastic_alpha;
    for (int it = 0; it < cfg_.elastic_iters; ++it) {
        std::vector<Eigen::VectorXd> next = cur;
        for (size_t i = 1; i + 1 < cur.size(); ++i) {
            Eigen::VectorXd cand =
                (1.0 - a) * cur[i] + 0.5 * a * (cur[i - 1] + cur[i + 1]);
            // Only accept if both adjacent segments remain free.
            if (is_segment_free(cur[i - 1], cand) &&
                is_segment_free(cand, cur[i + 1])) {
                next[i] = cand;
            }
        }
        cur.swap(next);
    }
    return cur;
}

// ── Pipeline driver ─────────────────────────────────────────────────
std::vector<Eigen::VectorXd> PathOptPipeline::optimize(
    const std::vector<Eigen::VectorXd>& raw) {
    step_lengths_.clear();
    step_lengths_.push_back(path_length(raw));
    if (raw.size() < 2) return raw;

    std::vector<Eigen::VectorXd> cur = raw;
    for (PathOptStep s : cfg_.steps) {
        switch (s) {
            case PathOptStep::GREEDY_SHORTCUT:
                cur = step_greedy_shortcut(cur); break;
            case PathOptStep::SHORTCUT:
                cur = step_shortcut(cur, cfg_.shortcut_iters); break;
            case PathOptStep::DENSIFY:
                cur = step_densify(cur); break;
            case PathOptStep::ELASTIC_BAND:
                cur = step_elastic_band(cur); break;
            case PathOptStep::SHORTCUT_FINAL:
                cur = step_shortcut(cur, cfg_.shortcut_iters); break;
        }
        step_lengths_.push_back(path_length(cur));
    }
    return cur;
}

}  // namespace sbf::planner
