/// @file bridge.cpp
#include <cstdio>
#include "sbf/forest/bridge.h"

#include "sbf/core/endpoint_iaabb.h"
#include "sbf/envelope/link_iaabb.h"
#include "sbf/forest/adjacency.h"
#include "sbf/forest/ffb.h"
#include "sbf/scene/collision.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <random>

namespace sbf::forest {

namespace {

bool intervals_share_face(const sbf::scene::BoxNode& a,
                          const sbf::scene::BoxNode& b,
                          double tol) {
    return boxes_adjacent(a, b, tol);
}

bool interval_box_collides_scene(
    const sbf::lect::LECT& lect,
    const std::vector<sbf::core::Interval>& intervals,
    const float* obs_compact,
    int n_obs) {
    if (n_obs <= 0 || obs_compact == nullptr) return false;

    const auto& robot = lect.robot();
    auto ep = sbf::core::compute_endpoint_iaabb_ifk(robot, intervals);
    const int n_active = ep.n_active_links;
    const int n_sub = std::max(1, lect.n_subdivisions());
    const int n_slots = n_active * n_sub;

    std::vector<float> link_aabbs(static_cast<std::size_t>(n_slots) * 6, 0.0f);
    if (n_sub <= 1) {
        sbf::envelope::derive_link_iaabb_paired_zero(
            ep.endpoint_iaabbs.data(), n_active, link_aabbs.data());
    } else {
        sbf::envelope::derive_link_iaabb_subdivided_zero(
            ep.endpoint_iaabbs.data(), n_active, n_sub, link_aabbs.data());
    }

    std::vector<float> radii(static_cast<std::size_t>(n_slots), 0.0f);
    if (const double* rr = robot.active_link_radii()) {
        for (int ci = 0; ci < n_active; ++ci) {
            const float r = static_cast<float>(rr[ci]);
            for (int s = 0; s < n_sub; ++s) radii[ci * n_sub + s] = r;
        }
    }
    return sbf::scene::aabbs_collide_obs_inflated(
        link_aabbs.data(), n_slots, radii.data(), obs_compact, n_obs);
}

}  // namespace

bool enforce_parent_adjacency(
    sbf::scene::BoxNode& new_box,
    const sbf::scene::BoxNode& parent_box,
    sbf::lect::LECT& lect,
    const float* obs_compact, int n_obs,
    double small_gap) {

    constexpr double kTol = 1e-11;
    if (intervals_share_face(new_box, parent_box, kTol)) return true;

    const int nd = new_box.n_dims();
    if (nd != parent_box.n_dims()) return false;

    // v6-style safe gap repair: snap/extend every separated dimension whose
    // gap is within the allowed LECT-cell scale.  Multi-dimensional gaps occur
    // when an FFB leaf lies diagonally across a kd-cell corner from the parent;
    // closing only the single smallest gap leaves the bridge fragmented.
    auto candidate = new_box.joint_intervals;
    constexpr double kOverlap = 1e-8;
    bool modified = false;
    for (int d = 0; d < nd; ++d) {
        double gap_hi = candidate[d].lo - parent_box.joint_intervals[d].hi;
        double gap_lo = parent_box.joint_intervals[d].lo - candidate[d].hi;
        double gap = std::max(gap_hi, gap_lo);
        if (gap <= 0.0) continue;
        if (gap > small_gap) return false;
        if (gap_hi > 0.0) {
            candidate[d].lo = parent_box.joint_intervals[d].hi - kOverlap;
        } else {
            candidate[d].hi = parent_box.joint_intervals[d].lo + kOverlap;
        }
        modified = true;
    }
    if (!modified) return false;

    auto old = new_box.joint_intervals;
    new_box.joint_intervals = std::move(candidate);
    new_box.compute_volume();

    if (!intervals_share_face(new_box, parent_box, kTol)) {
        new_box.joint_intervals = std::move(old);
        new_box.compute_volume();
        return false;
    }
    if (interval_box_collides_scene(lect, new_box.joint_intervals,
                                    obs_compact, n_obs)) {
        new_box.joint_intervals = std::move(old);
        new_box.compute_volume();
        return false;
    }
    return true;
}

// ────────────────────────────────────────────────────────────────────
//  snap_to_face  (P4.5 — mirror of v6 ForestGrower::snap_to_face)
// ────────────────────────────────────────────────────────────────────

SnapResult snap_to_face(
    const sbf::scene::BoxNode& parent,
    const Eigen::VectorXd& direction,
    const std::vector<sbf::core::Interval>& joint_limits,
    double rrt_step_ratio,
    std::mt19937_64& rng,
    double boundary_eps) {

    const int nd = parent.n_dims();
    SnapResult r;
    r.seed = Eigen::VectorXd::Zero(nd);

    // Pick face whose outward normal has the largest positive dot with `direction`,
    // skipping faces flush with the global joint limits.
    int best_dim = -1, best_side = -1;
    double best_score = 0.0;
    for (int d = 0; d < nd; ++d) {
        for (int side = 0; side < 2; ++side) {
            double normal_sign = (side == 1) ? 1.0 : -1.0;
            double score = direction[d] * normal_sign;
            if (score <= 0.0) continue;
            if (side == 0 && parent.joint_intervals[d].lo - boundary_eps < joint_limits[d].lo) continue;
            if (side == 1 && parent.joint_intervals[d].hi + boundary_eps > joint_limits[d].hi) continue;
            if (score > best_score) {
                best_score = score; best_dim = d; best_side = side;
            }
        }
    }

    Eigen::VectorXd c = parent.center();
    if (best_dim < 0) {
        // Fallback: small step in `direction` from center.
        double max_w = 0.0;
        for (int d = 0; d < nd; ++d) max_w = std::max(max_w, joint_limits[d].width());
        double step = rrt_step_ratio * max_w;
        for (int d = 0; d < nd; ++d) {
            r.seed[d] = std::clamp(c[d] + step * direction[d],
                                   joint_limits[d].lo, joint_limits[d].hi);
        }
        return r;
    }

    std::uniform_real_distribution<double> u01(0.0, 1.0);
    for (int d = 0; d < nd; ++d) {
        if (d == best_dim) {
            r.seed[d] = (best_side == 0)
                ? parent.joint_intervals[d].lo - boundary_eps
                : parent.joint_intervals[d].hi + boundary_eps;
        } else {
            double lo = parent.joint_intervals[d].lo;
            double hi = parent.joint_intervals[d].hi;
            double target = c[d] + direction[d] * rrt_step_ratio * joint_limits[d].width();
            target = std::clamp(target, lo, hi);
            double rand_on_face = lo + u01(rng) * std::max(0.0, hi - lo);
            r.seed[d] = 0.7 * target + 0.3 * rand_on_face;
        }
        r.seed[d] = std::clamp(r.seed[d], joint_limits[d].lo, joint_limits[d].hi);
    }
    r.face_dim = best_dim;
    r.face_side = best_side;
    return r;
}

// ────────────────────────────────────────────────────────────────────
//  endpoint_auto_bridge
// ────────────────────────────────────────────────────────────────────

namespace {

double center_dist2(const sbf::scene::BoxNode& a,
                    const sbf::scene::BoxNode& b) {
    Eigen::VectorXd ca = a.center();
    Eigen::VectorXd cb = b.center();
    double d2 = 0.0;
    for (int d = 0; d < ca.size(); ++d) {
        double dx = ca[d] - cb[d];
        d2 += dx * dx;
    }
    return d2;
}

double point_center_dist2(const sbf::scene::BoxNode& b,
                          const Eigen::VectorXd& q) {
    Eigen::VectorXd c = b.center();
    double d2 = 0.0;
    for (int d = 0; d < q.size(); ++d) {
        double dx = q[d] - c[d];
        d2 += dx * dx;
    }
    return d2;
}

int find_nearest_box_in_set(const std::vector<sbf::scene::BoxNode>& boxes,
                            const std::vector<int>& set_indices,
                            const Eigen::VectorXd& q,
                            std::mt19937_64* rng = nullptr,
                            int max_scan = 0) {
    int best = -1;
    double best_d2 = 1e300;
    auto visit = [&](int idx) {
        double d2 = point_center_dist2(boxes[idx], q);
        if (d2 < best_d2) { best_d2 = d2; best = idx; }
    };

    if (max_scan <= 0 || !rng || static_cast<int>(set_indices.size()) <= max_scan) {
        for (int idx : set_indices) visit(idx);
    } else {
        std::uniform_int_distribution<int> pick(0, static_cast<int>(set_indices.size()) - 1);
        for (int k = 0; k < max_scan; ++k) visit(set_indices[pick(*rng)]);
    }
    return best;
}

std::pair<int, int> closest_cross_pair(
    const std::vector<sbf::scene::BoxNode>& boxes,
    const std::vector<int>& a,
    const std::vector<int>& b,
    std::mt19937_64& rng) {
    if (a.empty() || b.empty()) return {-1, -1};

    int best_a = -1, best_b = -1;
    double best_d2 = std::numeric_limits<double>::infinity();
    auto test_pair = [&](int ia, int ib) {
        double d2 = center_dist2(boxes[ia], boxes[ib]);
        if (d2 < best_d2) { best_d2 = d2; best_a = ia; best_b = ib; }
    };

    const std::size_t n_pairs = a.size() * b.size();
    if (n_pairs <= 8192) {
        for (int ia : a)
            for (int ib : b) test_pair(ia, ib);
    } else {
        std::uniform_int_distribution<int> pick_a(0, static_cast<int>(a.size()) - 1);
        std::uniform_int_distribution<int> pick_b(0, static_cast<int>(b.size()) - 1);
        constexpr int kSamples = 8192;
        for (int k = 0; k < kSamples; ++k) test_pair(a[pick_a(rng)], b[pick_b(rng)]);
    }
    return {best_a, best_b};
}

sbf::scene::BoxNode build_box_from_leaf(
    sbf::lect::LECT& lect, int leaf_idx, int parent_box_id) {
    sbf::scene::BoxNode nb;
    nb.id = parent_box_id + 1000000;     // placeholder, caller sets real id
    nb.joint_intervals = lect.node_intervals(leaf_idx);
    nb.parent_box_id = parent_box_id;
    nb.compute_volume();
    int n_slots = lect.n_slots();
    const float* env = lect.get_link_iaabbs(leaf_idx);
    nb.link_iaabbs.assign(env, env + n_slots * 6);
    nb.seed_config = nb.center();
    return nb;
}

}  // namespace

AutoBridgeResult endpoint_auto_bridge(
    sbf::lect::LECT& lect,
    std::vector<sbf::scene::BoxNode>& boxes,
    int start_box, int goal_box,
    const Eigen::VectorXd& q_start,
    const Eigen::VectorXd& q_goal,
    const float* obs_compact, int n_obs,
    int extra_budget,
    uint64_t rng_seed,
    const FFBConfig& ffb_cfg) {

    AutoBridgeResult res;
    if (start_box < 0 || goal_box < 0 || extra_budget <= 0) return res;

    auto cur_g = compute_adjacency_graph(boxes);
    auto cur_isl = find_islands(cur_g);
    if (cur_isl.component_id[start_box] == cur_isl.component_id[goal_box]) {
        res.start_goal_connected = true;
        return res;
    }

    std::mt19937_64 rng(rng_seed);
    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::normal_distribution<double> n01(0.0, 1.0);

    const int nd = lect.n_dims();
    const auto& root_iv = lect.root_intervals();

    int budget = extra_budget;
    bool from_start = true;
    std::vector<int> start_set, goal_set;
    for (int i = 0; i < (int)boxes.size(); ++i) {
        if (cur_isl.component_id[i] == cur_isl.component_id[start_box]) start_set.push_back(i);
        else if (cur_isl.component_id[i] == cur_isl.component_id[goal_box]) goal_set.push_back(i);
    }

    auto frontier = closest_cross_pair(boxes, start_set, goal_set, rng);
    int refresh_countdown = 0;

    while (budget > 0) {
        ++res.n_ffb_attempts;

        if (--refresh_countdown <= 0 || frontier.first < 0 || frontier.second < 0) {
            frontier = closest_cross_pair(boxes, start_set, goal_set, rng);
            refresh_countdown = 32;
        }

        // Alternate which side we extend from.
        const std::vector<int>& src_set = from_start ? start_set : goal_set;
        const std::vector<int>& dst_set = from_start ? goal_set  : start_set;
        if (src_set.empty() || dst_set.empty()) break;

        // Sample mixture:
        //   75% midpoint/frontier bridge between closest disconnected components;
        //   20% toward a random box-center on the other side plus true opposite endpoint;
        //    5% uniform fallback.
        Eigen::VectorXd q(nd);
        double mode = u01(rng);
        if (mode < 0.75 && frontier.first >= 0 && frontier.second >= 0) {
            int src_frontier = from_start ? frontier.first : frontier.second;
            int dst_frontier = from_start ? frontier.second : frontier.first;
            Eigen::VectorXd cs = boxes[src_frontier].center();
            Eigen::VectorXd cd = boxes[dst_frontier].center();
            double alpha = 0.30 + 0.70 * u01(rng);
            for (int d = 0; d < nd; ++d) {
                double sigma = 0.02 * std::max(1.0, root_iv[d].width());
                q[d] = (1.0 - alpha) * cs[d] + alpha * cd[d] + sigma * n01(rng);
                q[d] = std::clamp(q[d], root_iv[d].lo, root_iv[d].hi);
            }
        } else if (mode < 0.95) {
            int gi = dst_set[rng() % dst_set.size()];
            const auto& gc = boxes[gi].center();
            for (int d = 0; d < nd; ++d) {
                double r = u01(rng) * 0.4;
                double pull_target = from_start ? q_goal[d] : q_start[d];
                q[d] = gc[d] * (1.0 - r) + pull_target * r;
                q[d] = std::clamp(q[d], root_iv[d].lo, root_iv[d].hi);
            }
        } else {
            for (int d = 0; d < nd; ++d) {
                double lo = root_iv[d].lo, hi = root_iv[d].hi;
                q[d] = lo + u01(rng) * (hi - lo);
            }
        }

        // Pick nearest from src side and snap-to-face toward q.
        int parent = find_nearest_box_in_set(boxes, src_set, q, &rng, 64);
        if (parent < 0) { from_start = !from_start; --budget; continue; }
        Eigen::VectorXd c = boxes[parent].center();
        Eigen::VectorXd dir = q - c;
        if (dir.norm() < 1e-15) {
            for (int d = 0; d < nd; ++d) dir[d] = u01(rng) - 0.5;
        }
        SnapResult sn = snap_to_face(boxes[parent], dir, root_iv,
                                     /*rrt_step_ratio=*/0.10, rng);
        Eigen::VectorXd seed = sn.seed;

        FFBResult fr = find_free_box(lect, seed, obs_compact, n_obs, ffb_cfg);
        if (!fr.success()) { from_start = !from_start; --budget; continue; }
        if (lect.is_occupied(fr.node_idx)) { from_start = !from_start; --budget; continue; }

        sbf::scene::BoxNode nb = build_box_from_leaf(lect, fr.node_idx, parent);
        nb.id = static_cast<int>(boxes.size());
        nb.tree_id = boxes[parent].tree_id;
        nb.root_id = boxes[parent].root_id;
        if (!enforce_parent_adjacency(nb, boxes[parent], lect, obs_compact, n_obs,
                                      /*small_gap=*/0.20)) {
            from_start = !from_start; --budget; continue;
        }
        boxes.push_back(nb);
        lect.mark_occupied(fr.node_idx, nb.id);
        ++res.n_bridges_added;
        --budget;

        bool connected_now = false;
        for (int other : dst_set) {
            if (boxes_adjacent(boxes.back(), boxes[other])) {
                connected_now = true;
                break;
            }
        }
        if (from_start) start_set.push_back(nb.id);
        else goal_set.push_back(nb.id);

        if (connected_now) {
            res.start_goal_connected = true;
            break;
        }

        // Cheap frontier update against the opposite side; full refresh is
        // still performed periodically to recover from stochastic misses.
        if (frontier.first >= 0 && frontier.second >= 0) {
            const int new_id = nb.id;
            double best_d2 = center_dist2(boxes[frontier.first], boxes[frontier.second]);
            for (int other : dst_set) {
                double d2 = center_dist2(boxes[new_id], boxes[other]);
                if (d2 < best_d2) {
                    if (from_start) frontier = {new_id, other};
                    else frontier = {other, new_id};
                    best_d2 = d2;
                }
            }
        } else {
            frontier = closest_cross_pair(boxes, start_set, goal_set, rng);
        }

        from_start = !from_start;
    }
    return res;
}

}  // namespace sbf::forest
