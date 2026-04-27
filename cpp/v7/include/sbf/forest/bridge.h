#pragma once
/// @file bridge.h
/// @brief Parent-face adjacency enforcement and endpoint auto-bridge.

#include "sbf/forest/ffb.h"
#include "sbf/lect/lect.h"
#include "sbf/scene/box_node.h"

#include <Eigen/Core>
#include <cstdint>
#include <random>
#include <vector>

namespace sbf::forest {

/// Output of snap_to_face: a seed configuration positioned on the chosen
/// face of `parent`, plus the dim/side of that face. If no valid face
/// exists (parent fills the limits in every dim), face_dim = -1 and the
/// seed is just a small step in `direction`.
struct SnapResult {
    Eigen::VectorXd seed;
    int face_dim  = -1;
    int face_side = -1;     // 0 = parent.lo side, 1 = parent.hi side
};

/// Mirror of v6 ForestGrower::snap_to_face. Picks the parent face whose
/// outward normal best aligns with `direction` (and which is not at the
/// global joint limit); places the seed just past that face plane, with
/// non-exit dims biased between (target along direction) and a uniform
/// face sample. RNG is owned by caller via std::mt19937_64& ref.
SnapResult snap_to_face(
    const sbf::scene::BoxNode& parent,
    const Eigen::VectorXd& direction,
    const std::vector<sbf::core::Interval>& joint_limits,
    double rrt_step_ratio,
    std::mt19937_64& rng,
    double boundary_eps = 1e-4);

/// Snap one face of `new_box` to coincide with `parent_box` if a small
/// gap exists along a single dim. Mirrors v6 enforce_parent_adjacency.
/// Returns true if `new_box` ends up face-adjacent to `parent_box`;
/// false otherwise (caller should reject the box).
bool enforce_parent_adjacency(
    sbf::scene::BoxNode& new_box,
    const sbf::scene::BoxNode& parent_box,
    sbf::lect::LECT& lect,
    const float* obs_compact, int n_obs,
    double small_gap = 0.05);

/// If start_box and goal_box are in disjoint islands, run a serial RRT
/// pass biased toward `q_goal` (when growing from start side) to add
/// bridge boxes. Updates `boxes` in-place; returns # bridges added.
struct AutoBridgeResult {
    int  n_bridges_added   = 0;
    int  n_ffb_attempts    = 0;
    bool start_goal_connected = false;
};

AutoBridgeResult endpoint_auto_bridge(
    sbf::lect::LECT& lect,
    std::vector<sbf::scene::BoxNode>& boxes,
    int start_box, int goal_box,
    const Eigen::VectorXd& q_start,
    const Eigen::VectorXd& q_goal,
    const float* obs_compact, int n_obs,
    int extra_budget,
    uint64_t rng_seed,
    const FFBConfig& ffb_cfg = FFBConfig{});

}  // namespace sbf::forest
