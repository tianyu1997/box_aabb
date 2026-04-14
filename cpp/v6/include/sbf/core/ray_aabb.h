/// @file ray_aabb.h
/// @brief Analytical ray-AABB intersection and segment-in-box-union tests.
///
/// Provides exact geometric containment checks for line segments against
/// axis-aligned bounding boxes (AABBs), replacing discrete sampling methods.
///
/// Key functions:
///   - segment_in_box(): segment fully inside one AABB (O(d), convexity)
///   - ray_intersect_box(): slab-method ray-AABB intersection
///   - segment_in_box_union(): segment inside union of AABBs (exact)
#pragma once

#include <sbf/core/types.h>
#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>
#include <unordered_map>

namespace sbf {

/// Check whether a line segment [a, b] is entirely inside a single AABB.
/// Since an AABB is convex, it suffices to check that both endpoints are inside.
/// @param a  Segment start (n-dimensional).
/// @param b  Segment end (n-dimensional).
/// @param box The AABB to test against.
/// @param tol Containment tolerance (added to box boundaries).
/// @return true if the entire segment lies within the box.
inline bool segment_in_box(
    const Eigen::VectorXd& a,
    const Eigen::VectorXd& b,
    const BoxNode& box,
    double tol = 1e-10)
{
    return box.contains(a, tol) && box.contains(b, tol);
}

/// Slab-method ray-AABB intersection.
///
/// Given a ray p(t) = origin + t * dir, computes the entry and exit parameter
/// values [t_enter, t_exit] where the ray intersects the AABB.
///
/// For a line segment from `a` to `b`:
///   origin = a, dir = b - a, and the segment corresponds to t ∈ [0, 1].
///
/// @param origin Ray origin.
/// @param dir    Ray direction (need not be normalized; segment = origin + t*dir for t∈[0,1]).
/// @param box    The AABB to intersect.
/// @return {t_enter, t_exit}. If t_enter > t_exit, the ray misses the box entirely.
inline std::pair<double, double> ray_intersect_box(
    const Eigen::VectorXd& origin,
    const Eigen::VectorXd& dir,
    const BoxNode& box)
{
    const int nd = box.n_dims();
    double t_enter = -std::numeric_limits<double>::infinity();
    double t_exit  =  std::numeric_limits<double>::infinity();

    for (int d = 0; d < nd; ++d) {
        double o = origin[d];
        double dd = dir[d];
        double lo = box.joint_intervals[d].lo;
        double hi = box.joint_intervals[d].hi;

        if (std::abs(dd) < 1e-15) {
            // Ray is parallel to slab in this dimension.
            // If origin is outside slab → no intersection.
            if (o < lo - 1e-12 || o > hi + 1e-12) {
                return {1.0, 0.0};  // miss
            }
            // Otherwise this dimension doesn't constrain t.
            continue;
        }

        double inv_d = 1.0 / dd;
        double t0 = (lo - o) * inv_d;
        double t1 = (hi - o) * inv_d;

        // Ensure t0 ≤ t1
        if (t0 > t1) std::swap(t0, t1);

        t_enter = std::max(t_enter, t0);
        t_exit  = std::min(t_exit, t1);

        if (t_enter > t_exit + 1e-12) {
            return {1.0, 0.0};  // miss — early exit
        }
    }

    return {t_enter, t_exit};
}

/// Check whether a line segment [a, b] lies entirely within the union of a set
/// of AABBs.
///
/// Algorithm:
///   1. For each box in `box_ids`, compute the ray-AABB intersection
///      [t_enter, t_exit] for the ray from a to b (t ∈ [0, 1]).
///   2. Clip each interval to [0, 1].
///   3. Sort intervals by t_enter and greedily merge.
///   4. Check whether the merged intervals cover [0, 1].
///
/// Complexity: O(k·d + k·log(k)) where k = |box_ids|, d = dimensionality.
///
/// @param a       Segment start.
/// @param b       Segment end.
/// @param boxes   All boxes (for id→box lookup).
/// @param box_ids IDs of boxes to consider.
/// @param tol     Tolerance for containment (small overlap between intervals).
/// @return true if the segment is fully covered by the union of boxes.
inline bool segment_in_box_union(
    const Eigen::VectorXd& a,
    const Eigen::VectorXd& b,
    const std::vector<BoxNode>& boxes,
    const std::vector<int>& box_ids,
    double tol = 1e-8)
{
    if (box_ids.empty()) return false;

    // Build id→index map for O(1) lookup
    // (For performance, caller should cache this for repeated calls)
    std::unordered_map<int, int> id_to_idx;
    id_to_idx.reserve(boxes.size());
    for (int i = 0; i < static_cast<int>(boxes.size()); ++i)
        id_to_idx[boxes[i].id] = i;

    Eigen::VectorXd dir = b - a;

    // Collect [t_enter, t_exit] ∩ [0, 1] for each box
    std::vector<std::pair<double, double>> intervals;
    intervals.reserve(box_ids.size());

    for (int bid : box_ids) {
        auto it = id_to_idx.find(bid);
        if (it == id_to_idx.end()) continue;
        const BoxNode& box = boxes[it->second];

        auto [te, tx] = ray_intersect_box(a, dir, box);

        // Clip to [0, 1]
        te = std::max(te, 0.0);
        tx = std::min(tx, 1.0);

        if (te <= tx + tol) {
            intervals.push_back({te, tx});
        }
    }

    if (intervals.empty()) return false;

    // Sort by t_enter
    std::sort(intervals.begin(), intervals.end());

    // Greedy merge and check coverage of [0, 1]
    double covered_up_to = -tol;  // allow small tolerance at boundaries

    for (const auto& [te, tx] : intervals) {
        if (te > covered_up_to + tol) {
            // Gap detected — not fully covered
            return false;
        }
        covered_up_to = std::max(covered_up_to, tx);
    }

    return covered_up_to >= 1.0 - tol;
}

/// Overload that takes a prebuilt id→index map for performance in hot loops.
/// @param id_to_idx Prebuilt mapping from box ID to index in `boxes`.
inline bool segment_in_box_union(
    const Eigen::VectorXd& a,
    const Eigen::VectorXd& b,
    const std::vector<BoxNode>& boxes,
    const std::vector<int>& box_ids,
    const std::unordered_map<int, int>& id_to_idx,
    double tol = 1e-8)
{
    if (box_ids.empty()) return false;

    Eigen::VectorXd dir = b - a;

    std::vector<std::pair<double, double>> intervals;
    intervals.reserve(box_ids.size());

    for (int bid : box_ids) {
        auto it = id_to_idx.find(bid);
        if (it == id_to_idx.end()) continue;
        const BoxNode& box = boxes[it->second];

        auto [te, tx] = ray_intersect_box(a, dir, box);
        te = std::max(te, 0.0);
        tx = std::min(tx, 1.0);

        if (te <= tx + tol) {
            intervals.push_back({te, tx});
        }
    }

    if (intervals.empty()) return false;

    std::sort(intervals.begin(), intervals.end());

    double covered_up_to = -tol;
    for (const auto& [te, tx] : intervals) {
        if (te > covered_up_to + tol)
            return false;
        covered_up_to = std::max(covered_up_to, tx);
    }

    return covered_up_to >= 1.0 - tol;
}

}  // namespace sbf
