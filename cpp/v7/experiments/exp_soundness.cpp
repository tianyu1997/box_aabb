/// @file exp_soundness.cpp
/// @brief P1.5 Empirical soundness check.
///
/// For each scene: build a forest, then for M random q's sampled
/// uniformly from the union of accepted safe boxes, compute a
/// per-link ground-truth collision check (degenerate-interval FK
/// + radius-inflated obstacle SAT) and count any violations.
///
/// Expected outcome (per Theorem 1): zero violations for all M.
#include "experiments/common.h"

#include "sbf/core/fk_state.h"
#include "sbf/core/endpoint_iaabb.h"
#include "sbf/core/robot.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/lect/lect.h"
#include "sbf/scene/collision.h"
#include "sbf/planner/sbf_planner.h"

#include <algorithm>
#include <cstdint>
#include <iostream>
#include <random>
#include <vector>

namespace sbfp = sbf::planner;
namespace sbfl = sbf::lect;
namespace sbfc = sbf::core;
namespace sbfs = sbf::scene;

int main(int argc, char** argv) {
    // Strip --samples=N before generic CliArgs parsing.
    int n_samples = 1'000'000;
    std::vector<char*> kept_argv;
    kept_argv.reserve(static_cast<size_t>(argc));
    for (int i = 0; i < argc; ++i) {
        std::string s = argv[i];
        if (s.rfind("--samples=", 0) == 0) {
            n_samples = std::stoi(s.substr(10));
        } else {
            kept_argv.push_back(argv[i]);
        }
    }
    auto a  = sbf::exp::CliArgs::parse(static_cast<int>(kept_argv.size()),
                                       kept_argv.data());
    auto sc = sbf::scene::load_scene_json(a.scene_path);

    auto robot = sbfc::Robot::from_json(sbf::exp::robot_json_path(sc.robot));
    auto lect  = sbf::exp::load_or_make_lect(robot,
                                             a.env_type_enum(),
                                             a.n_subdivisions,
                                             a.voxel_delta,
                                             a.lect_cache_path);

    sbfp::PlannerConfig cfg;
    cfg.grower.rng_seed   = 42;
    cfg.grower.n_threads  = a.n_threads;
    cfg.grower.max_boxes  = a.max_boxes > 0 ? a.max_boxes : 800;
    cfg.grower.timeout_ms = static_cast<double>(a.timeout_s) * 1000.0;
    if (robot.n_joints() <= 3) cfg.grower.ffb.max_depth = 8;
    cfg.timeout_s = static_cast<double>(a.timeout_s);

    sbfp::SbfPlanner planner(robot, lect, cfg);
    auto packed = sc.packed_obstacles();
    auto pr = planner.plan(sc.q_start, sc.q_goal,
                           packed.empty() ? nullptr : packed.data(),
                           static_cast<int>(sc.obstacles.size()));
    sbf::exp::save_lect_geometry_cache(lect, a.lect_cache_path);

    // Enumerate accepted safe boxes from the LECT.
    std::vector<int> safe_nodes;
    safe_nodes.reserve(lect.n_nodes());
    for (int i = 0; i < lect.n_nodes(); ++i) {
        if (lect.is_occupied(i)) safe_nodes.push_back(i);
    }
    std::cout << "[soundness] scene=" << sc.name
              << " forest_boxes=" << safe_nodes.size()
              << " plan_success=" << pr.success
              << " sampling=" << n_samples << " configs\n";
    if (safe_nodes.empty()) {
        std::cerr << "[soundness] no safe boxes — aborting.\n";
        return 2;
    }

    // Pre-fetch obstacle compact buffer (xhxyhyzhz layout) once.
    std::vector<float> obs_compact(sc.obstacles.size() * 6);
    if (!sc.obstacles.empty()) {
        sbfs::pack_obstacles(sc.obstacles.data(),
                             static_cast<int>(sc.obstacles.size()),
                             obs_compact.data());
    }
    const int n_active   = robot.n_active_links();
    const int n_sub      = std::max(1, lect.n_subdivisions());
    const int n_slots    = n_active * n_sub;
    std::vector<float> per_slot_radii(n_slots, 0.0f);
    if (robot.active_link_radii() != nullptr) {
        for (int k = 0; k < n_active; ++k) {
            const float rf = static_cast<float>(robot.active_link_radii()[k]);
            for (int s = 0; s < n_sub; ++s)
                per_slot_radii[k * n_sub + s] = rf;
        }
    }
    const auto& env_config = lect.env_config();
    const double* link_radii_ptr =
        robot.active_link_radii();  // for grid-side inflation; AABBs are zero-radius

    std::mt19937_64 rng(123);
    std::uniform_int_distribution<int> pick_box(
        0, static_cast<int>(safe_nodes.size()) - 1);
    std::uniform_real_distribution<double> u01(0.0, 1.0);

    std::vector<sbfc::Interval> intervals(robot.n_joints());
    int n_violations = 0;
    int report_every = std::max(1, n_samples / 10);
    for (int k = 0; k < n_samples; ++k) {
        const int   nidx     = safe_nodes[pick_box(rng)];
        auto        box_iv   = lect.node_intervals(nidx);
        // Sample a uniform configuration q ∈ box.
        for (int d = 0; d < robot.n_joints(); ++d) {
            const double lo = box_iv[d].lo;
            const double hi = box_iv[d].hi;
            const double q  = lo + u01(rng) * (hi - lo);
            intervals[d]    = sbfc::Interval{q, q};
        }
        // Reproduce the planner's per-slot envelope at the degenerate
        // interval [q,q] using the same envelope config the LECT used.
        auto ep = sbfc::compute_endpoint_iaabb_ifk(robot, intervals);
        auto le = sbf::envelope::compute_link_envelope(
            ep.endpoint_iaabbs.data(), n_active, link_radii_ptr, env_config);
        bool collides = sbfs::aabbs_collide_obs_inflated(
            le.link_iaabbs.data(), n_slots,
            per_slot_radii.data(),
            obs_compact.empty() ? nullptr : obs_compact.data(),
            static_cast<int>(sc.obstacles.size()));
        if (collides) ++n_violations;
        if ((k + 1) % report_every == 0) {
            std::cout << "  progress " << (k + 1) << "/" << n_samples
                      << " violations=" << n_violations << "\n";
        }
    }

    nlohmann::json out = {
        {"experiment",   "soundness"},
        {"scene",        sc.name},
        {"robot",        sc.robot},
        {"lect_cache",   a.lect_cache_path.empty()
                      ? nlohmann::json(nullptr)
                      : nlohmann::json(a.lect_cache_path)},
        {"forest_boxes", static_cast<int>(safe_nodes.size())},
        {"samples",      n_samples},
        {"violations",   n_violations},
        {"violation_rate", static_cast<double>(n_violations) / n_samples},
    };
    if (!a.out_path.empty()) sbf::exp::write_json(a.out_path, out);
    std::cout << "[soundness] DONE violations=" << n_violations
              << " / " << n_samples << "\n";
    return n_violations == 0 ? 0 : 1;
}
