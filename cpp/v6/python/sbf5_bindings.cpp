/// @file sbf5_bindings.cpp
/// @brief pybind11 Python bindings for SafeBoxForest v5 (`_sbf5_cpp` module).
///
/// Exposes the following to Python:
///   - Core types: Interval, Obstacle, JointLimits, BoxNode
///   - Robot: from_json(), kinematic queries
///   - Configuration structs: FFBConfig, GrowerConfig, GreedyCoarsenConfig,
///     SmootherConfig, GCSConfig, SBFPlannerConfig
///   - Enums: GrowerMode, EndpointSource, EnvelopeType, SplitOrder
///   - SBFPlanner: plan(), build(), build_coverage(), query(), warmup_lect()
///   - GcpcCache: load/save GCPC critical-point cache
///   - compute_envelope_info(): one-shot envelope volume + timing measurement
///
/// The GIL is released during long-running C++ operations (plan, build,
/// warmup_lect) so Python threads remain responsive.
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>

#include <sbf/core/types.h>
#include <sbf/core/robot.h>
#include <sbf/scene/collision_checker.h>
#include <sbf/envelope/endpoint_source.h>
#include <sbf/envelope/envelope_type.h>
#include <sbf/envelope/gcpc_source.h>
#include <sbf/forest/grower.h>
#include <sbf/forest/coarsen.h>
#include <sbf/planner/sbf_planner.h>

#include <chrono>
#include <array>
#include <string>
#include <unordered_set>
#include <vector>

namespace py = pybind11;

PYBIND11_MODULE(_sbf5_cpp, m) {
    m.doc() = "SafeBoxForest v5 C++ extension";

    // ─── Interval ───────────────────────────────────────────────────────
    py::class_<sbf::Interval>(m, "Interval")
        .def(py::init<>())
        .def(py::init<double, double>(), py::arg("lo"), py::arg("hi"))
        .def_readwrite("lo", &sbf::Interval::lo)
        .def_readwrite("hi", &sbf::Interval::hi)
        .def("width",  &sbf::Interval::width)
        .def("center", &sbf::Interval::center)
        .def("empty",  &sbf::Interval::empty)
        .def("__repr__", [](const sbf::Interval& iv) {
            return "[" + std::to_string(iv.lo) + ", " + std::to_string(iv.hi) + "]";
        });

    // ─── Obstacle ───────────────────────────────────────────────────────
    py::class_<sbf::Obstacle>(m, "Obstacle")
        .def(py::init<>())
        .def(py::init<float, float, float, float, float, float>(),
             py::arg("lx"), py::arg("ly"), py::arg("lz"),
             py::arg("hx"), py::arg("hy"), py::arg("hz"))
        .def("min_point", [](const sbf::Obstacle& o) {
            return Eigen::Vector3d(o.bounds[0], o.bounds[1], o.bounds[2]);
        })
        .def("max_point", [](const sbf::Obstacle& o) {
            return Eigen::Vector3d(o.bounds[3], o.bounds[4], o.bounds[5]);
        })
        .def_property("bounds",
            [](const sbf::Obstacle& o) {
                std::vector<float> b(o.bounds, o.bounds + 6);
                return b;
            },
            [](sbf::Obstacle& o, const std::vector<float>& b) {
                for (int i = 0; i < 6 && i < (int)b.size(); ++i)
                    o.bounds[i] = b[i];
            })
        .def("__repr__", [](const sbf::Obstacle& o) {
            return "Obstacle([" +
                std::to_string(o.bounds[0]) + "," + std::to_string(o.bounds[1]) + "," +
                std::to_string(o.bounds[2]) + "] -> [" +
                std::to_string(o.bounds[3]) + "," + std::to_string(o.bounds[4]) + "," +
                std::to_string(o.bounds[5]) + "])";
        });

    // ─── JointLimits ────────────────────────────────────────────────────
    py::class_<sbf::JointLimits>(m, "JointLimits")
        .def(py::init<>())
        .def_readwrite("limits", &sbf::JointLimits::limits)
        .def("n_dims", &sbf::JointLimits::n_dims);

    // ─── BoxNode ────────────────────────────────────────────────────────
    py::class_<sbf::BoxNode>(m, "BoxNode")
        .def_readonly("id", &sbf::BoxNode::id)
        .def_readonly("joint_intervals", &sbf::BoxNode::joint_intervals)
        .def_readonly("seed_config", &sbf::BoxNode::seed_config)
        .def_readonly("volume", &sbf::BoxNode::volume)
        .def_readonly("tree_id", &sbf::BoxNode::tree_id)
        .def_readonly("parent_box_id", &sbf::BoxNode::parent_box_id)
        .def_readonly("root_id", &sbf::BoxNode::root_id)
        .def("center", &sbf::BoxNode::center)
        .def("n_dims", &sbf::BoxNode::n_dims);

    // ─── Robot ──────────────────────────────────────────────────────────
    py::class_<sbf::Robot>(m, "Robot")
        .def_static("from_json", &sbf::Robot::from_json, py::arg("path"))
        .def("name",     &sbf::Robot::name)
        .def("n_joints", &sbf::Robot::n_joints)
        .def("n_active_links", &sbf::Robot::n_active_links)
        .def("has_tool", &sbf::Robot::has_tool)
        .def("fingerprint", &sbf::Robot::fingerprint)
        .def("joint_limits", &sbf::Robot::joint_limits,
             py::return_value_policy::reference_internal)
        .def("link_radii", &sbf::Robot::link_radii,
             py::return_value_policy::reference_internal)
        .def("active_link_radii", [](const sbf::Robot& r) {
            const double* p = r.active_link_radii();
            if (!p) return std::vector<double>();
            return std::vector<double>(p, p + r.n_active_links());
        });

    // ─── GrowerConfig ───────────────────────────────────────────────────
    py::enum_<sbf::GrowerConfig::Mode>(m, "GrowerMode")
        .value("RRT",       sbf::GrowerConfig::Mode::RRT)
        .value("WAVEFRONT", sbf::GrowerConfig::Mode::WAVEFRONT);

    py::class_<sbf::FFBConfig>(m, "FFBConfig")
        .def(py::init<>())
        .def_readwrite("max_depth", &sbf::FFBConfig::max_depth)
        .def_readwrite("deadline_ms", &sbf::FFBConfig::deadline_ms)
        .def_readwrite("grid_margin_threshold", &sbf::FFBConfig::grid_margin_threshold)
        .def_readwrite("seed_known_free",       &sbf::FFBConfig::seed_known_free);

    py::class_<sbf::GrowerConfig>(m, "GrowerConfig")
        .def(py::init<>())
        .def_readwrite("mode",             &sbf::GrowerConfig::mode)
        .def_readwrite("ffb_config",       &sbf::GrowerConfig::ffb_config)
        .def_readwrite("max_boxes",        &sbf::GrowerConfig::max_boxes)
        .def_readwrite("timeout_ms",       &sbf::GrowerConfig::timeout_ms)
        .def_readwrite("max_consecutive_miss", &sbf::GrowerConfig::max_consecutive_miss)
        .def_readwrite("rrt_goal_bias",    &sbf::GrowerConfig::rrt_goal_bias)
        .def_readwrite("rrt_step_ratio",   &sbf::GrowerConfig::rrt_step_ratio)
        .def_readwrite("vol_bonus_alpha",  &sbf::GrowerConfig::vol_bonus_alpha)
        .def_readwrite("enable_promotion", &sbf::GrowerConfig::enable_promotion)
        .def_readwrite("rng_seed",         &sbf::GrowerConfig::rng_seed)
        .def_readwrite("n_threads",        &sbf::GrowerConfig::n_threads)
        .def_readwrite("bridge_n_threads", &sbf::GrowerConfig::bridge_n_threads)
        .def_readwrite("connect_mode",     &sbf::GrowerConfig::connect_mode)
        .def_readwrite("enable_partitioned_lect_parallel", &sbf::GrowerConfig::enable_partitioned_lect_parallel)
        .def_readwrite("partitioned_box_budget_per_tree", &sbf::GrowerConfig::partitioned_box_budget_per_tree)
        .def_readwrite("enable_coordinated_multi_goal", &sbf::GrowerConfig::enable_coordinated_multi_goal)
        .def_readwrite("stop_after_connect",       &sbf::GrowerConfig::stop_after_connect)
        .def_readwrite("post_connect_extra_boxes", &sbf::GrowerConfig::post_connect_extra_boxes)
        .def_readwrite("batch_size",               &sbf::GrowerConfig::batch_size)
        // Phase U / B / C-1
        .def_readwrite("unexplored_sample_prob",   &sbf::GrowerConfig::unexplored_sample_prob)
        .def_readwrite("ffb_depth_stages",         &sbf::GrowerConfig::ffb_depth_stages)
        .def_readwrite("endpoint_auto_bridge",     &sbf::GrowerConfig::endpoint_auto_bridge)
        .def_readwrite("endpoint_bridge_max_boxes",&sbf::GrowerConfig::endpoint_bridge_max_boxes);

    // ─── GreedyCoarsenConfig ────────────────────────────────────────────
    py::class_<sbf::GreedyCoarsenConfig>(m, "GreedyCoarsenConfig")
        .def(py::init<>())
        .def_readwrite("target_boxes",     &sbf::GreedyCoarsenConfig::target_boxes)
        .def_readwrite("max_rounds",       &sbf::GreedyCoarsenConfig::max_rounds)
        .def_readwrite("score_threshold",  &sbf::GreedyCoarsenConfig::score_threshold);

    // ─── SmootherConfig ─────────────────────────────────────────────────
    py::class_<sbf::SmootherConfig>(m, "SmootherConfig")
        .def(py::init<>())
        .def_readwrite("shortcut_max_iters", &sbf::SmootherConfig::shortcut_max_iters)
        .def_readwrite("smooth_window",      &sbf::SmootherConfig::smooth_window)
        .def_readwrite("smooth_iters",       &sbf::SmootherConfig::smooth_iters)
        .def_readwrite("segment_resolution", &sbf::SmootherConfig::segment_resolution);

    // ─── GCSConfig ──────────────────────────────────────────────────────
    py::class_<sbf::GCSConfig>(m, "GCSConfig")
        .def(py::init<>())
        .def_readwrite("bezier_degree",      &sbf::GCSConfig::bezier_degree)
        .def_readwrite("time_limit_sec",     &sbf::GCSConfig::time_limit_sec)
        .def_readwrite("corridor_hops",      &sbf::GCSConfig::corridor_hops)
        .def_readwrite("max_corridor_size",  &sbf::GCSConfig::max_corridor_size)
        .def_readwrite("max_gcs_verts",      &sbf::GCSConfig::max_gcs_verts)
        .def_readwrite("convex_relaxation",  &sbf::GCSConfig::convex_relaxation)
        .def_readwrite("cost_weight_length", &sbf::GCSConfig::cost_weight_length);

    // ─── EndpointSource enum (Phase R2) ─────────────────────────────────
    py::enum_<sbf::EndpointSource>(m, "EndpointSource")
        .value("IFK",        sbf::EndpointSource::IFK)
        .value("CritSample", sbf::EndpointSource::CritSample)
        .value("Analytical", sbf::EndpointSource::Analytical)
        .value("GCPC",       sbf::EndpointSource::GCPC)
        .value("MC",         sbf::EndpointSource::MC);

    // ─── EnvelopeType enum (Phase R2) ───────────────────────────────────
    py::enum_<sbf::EnvelopeType>(m, "EnvelopeType")
        .value("LinkIAABB",      sbf::EnvelopeType::LinkIAABB)
        .value("LinkIAABB_Grid", sbf::EnvelopeType::LinkIAABB_Grid)
        .value("Hull16_Grid",    sbf::EnvelopeType::Hull16_Grid);

    // ─── GridConfig (for grid-based envelopes) ─────────────────────────
    py::class_<sbf::GridConfig>(m, "GridConfig")
        .def(py::init<>())
        .def_readwrite("voxel_delta", &sbf::GridConfig::voxel_delta);

    // ─── EndpointSourceConfig (Phase R2) ────────────────────────────────
    py::class_<sbf::EndpointSourceConfig>(m, "EndpointSourceConfig")
        .def(py::init<>())
        .def_readwrite("source",              &sbf::EndpointSourceConfig::source)
        .def_readwrite("n_samples_crit",      &sbf::EndpointSourceConfig::n_samples_crit)
        .def_readwrite("max_phase_analytical", &sbf::EndpointSourceConfig::max_phase_analytical)
        .def_readwrite("bypass_narrow_skip",  &sbf::EndpointSourceConfig::bypass_narrow_skip)
        .def_readwrite("gcpc_match_analytical", &sbf::EndpointSourceConfig::gcpc_match_analytical)
        .def("set_gcpc_cache", [](sbf::EndpointSourceConfig& self,
                                   const sbf::GcpcCache& cache) {
            self.gcpc_cache = &cache;
        }, py::arg("cache"),
           "Set GCPC cache (caller must keep the cache alive)");

    // ─── EnvelopeTypeConfig (Phase R2) ──────────────────────────────────
    py::class_<sbf::EnvelopeTypeConfig>(m, "EnvelopeTypeConfig")
        .def(py::init<>())
        .def_readwrite("type",           &sbf::EnvelopeTypeConfig::type)
        .def_readwrite("n_subdivisions", &sbf::EnvelopeTypeConfig::n_subdivisions)
        .def_readwrite("grid_config",    &sbf::EnvelopeTypeConfig::grid_config);

    // ─── GcpcCache (Phase R2) ───────────────────────────────────────────
    py::class_<sbf::GcpcCache>(m, "GcpcCache")
        .def(py::init<>())
        .def_static("load", &sbf::GcpcCache::load, py::arg("path"))
        .def("save",     &sbf::GcpcCache::save, py::arg("path"))
        .def("n_points", &sbf::GcpcCache::n_points)
        .def("n_dims",   &sbf::GcpcCache::n_dims)
        .def("empty",    &sbf::GcpcCache::empty);

    // ─── SplitOrder enum ─────────────────────────────────────────────────
    py::enum_<sbf::SplitOrder>(m, "SplitOrder")
        .value("ROUND_ROBIN",  sbf::SplitOrder::ROUND_ROBIN)
        .value("WIDEST_FIRST", sbf::SplitOrder::WIDEST_FIRST)
        .value("BEST_TIGHTEN", sbf::SplitOrder::BEST_TIGHTEN);

    // ─── SBFPlannerConfig ───────────────────────────────────────────────
    py::class_<sbf::SBFPlannerConfig>(m, "SBFPlannerConfig")
        .def(py::init<>())
        .def_readwrite("grower",          &sbf::SBFPlannerConfig::grower)
        .def_readwrite("coarsen",         &sbf::SBFPlannerConfig::coarsen)
        .def_readwrite("smoother",        &sbf::SBFPlannerConfig::smoother)
        .def_readwrite("use_gcs",         &sbf::SBFPlannerConfig::use_gcs)
        .def_readwrite("gcs",             &sbf::SBFPlannerConfig::gcs)
        .def_readwrite("endpoint_source", &sbf::SBFPlannerConfig::endpoint_source)
        .def_readwrite("envelope_type",   &sbf::SBFPlannerConfig::envelope_type)
        .def_readwrite("split_order",     &sbf::SBFPlannerConfig::split_order)
        .def_readwrite("z4_enabled",      &sbf::SBFPlannerConfig::z4_enabled)
        .def_readwrite("enable_coarsen",  &sbf::SBFPlannerConfig::enable_coarsen)
        .def_readwrite("enable_path_opt", &sbf::SBFPlannerConfig::enable_path_opt)
        .def_readwrite("adjacency_tol",   &sbf::SBFPlannerConfig::adjacency_tol)
        .def_readwrite("adjacency_gap_tol", &sbf::SBFPlannerConfig::adjacency_gap_tol)
        .def_readwrite("lect_no_cache",   &sbf::SBFPlannerConfig::lect_no_cache)
        .def_readwrite("lect_file_cache_load", &sbf::SBFPlannerConfig::lect_file_cache_load)
        .def_readwrite("lect_file_cache_save", &sbf::SBFPlannerConfig::lect_file_cache_save)
        .def_readwrite("use_v6_cache",    &sbf::SBFPlannerConfig::use_v6_cache)
        .def_readwrite("v6_cache_strict", &sbf::SBFPlannerConfig::v6_cache_strict)
        .def_readwrite("lect_cache_dir",  &sbf::SBFPlannerConfig::lect_cache_dir)
        .def_readwrite("enable_seed_bridge", &sbf::SBFPlannerConfig::enable_seed_bridge)
        .def_readwrite("enable_rescue_bridge", &sbf::SBFPlannerConfig::enable_rescue_bridge)
        .def_readwrite("force_full_bridge", &sbf::SBFPlannerConfig::force_full_bridge)
        .def_readwrite("force_full_bridge_timeout_ms", &sbf::SBFPlannerConfig::force_full_bridge_timeout_ms)
        .def_readwrite("force_full_bridge_max_pairs_per_gap", &sbf::SBFPlannerConfig::force_full_bridge_max_pairs_per_gap)
        .def_readwrite("force_full_bridge_max_total_bridges", &sbf::SBFPlannerConfig::force_full_bridge_max_total_bridges);

    py::class_<sbf::BuildTimingProfile>(m, "BuildTimingProfile")
        .def_readonly("lect_ms", &sbf::BuildTimingProfile::lect_ms)
        .def_readonly("grow_ms", &sbf::BuildTimingProfile::grow_ms)
        .def_readonly("grow_roots_ms", &sbf::BuildTimingProfile::grow_roots_ms)
        .def_readonly("grow_expand_ms", &sbf::BuildTimingProfile::grow_expand_ms)
        .def_readonly("grow_promotion_ms", &sbf::BuildTimingProfile::grow_promotion_ms)
        .def_readonly("grow_ffb_total_ms", &sbf::BuildTimingProfile::grow_ffb_total_ms)
        .def_readonly("grow_ffb_envelope_ms", &sbf::BuildTimingProfile::grow_ffb_envelope_ms)
        .def_readonly("grow_ffb_collide_ms", &sbf::BuildTimingProfile::grow_ffb_collide_ms)
        .def_readonly("grow_ffb_expand_ms", &sbf::BuildTimingProfile::grow_ffb_expand_ms)
        .def_readonly("grow_ffb_intervals_ms", &sbf::BuildTimingProfile::grow_ffb_intervals_ms)
        .def_readonly("grow_expand_calls", &sbf::BuildTimingProfile::grow_expand_calls)
        .def_readonly("grow_expand_new_nodes", &sbf::BuildTimingProfile::grow_expand_new_nodes)
        .def_readonly("grow_expand_profile_total_ms", &sbf::BuildTimingProfile::grow_expand_profile_total_ms)
        .def_readonly("grow_expand_pick_dim_ms", &sbf::BuildTimingProfile::grow_expand_pick_dim_ms)
        .def_readonly("grow_expand_fk_ms", &sbf::BuildTimingProfile::grow_expand_fk_ms)
        .def_readonly("grow_expand_env_ms", &sbf::BuildTimingProfile::grow_expand_env_ms)
        .def_readonly("grow_expand_refine_ms", &sbf::BuildTimingProfile::grow_expand_refine_ms)
        .def_readonly("v6_cache_ep_hits", &sbf::BuildTimingProfile::v6_cache_ep_hits)
        .def_readonly("v6_cache_ep_misses", &sbf::BuildTimingProfile::v6_cache_ep_misses)
        .def_readonly("v6_cache_grid_hits", &sbf::BuildTimingProfile::v6_cache_grid_hits)
        .def_readonly("v6_cache_grid_misses", &sbf::BuildTimingProfile::v6_cache_grid_misses)
        .def_readonly("v6_cache_grid_compute_fallbacks", &sbf::BuildTimingProfile::v6_cache_grid_compute_fallbacks)
        .def_readonly("boxes_after_grow", &sbf::BuildTimingProfile::boxes_after_grow)
        .def_readonly("boxes_final", &sbf::BuildTimingProfile::boxes_final)
        .def_readonly("total_ms", &sbf::BuildTimingProfile::total_ms);

    // ─── PlanResult ─────────────────────────────────────────────────────
    py::class_<sbf::PlanResult>(m, "PlanResult")
        .def_readonly("success",               &sbf::PlanResult::success)
        .def_readonly("path",                  &sbf::PlanResult::path)
        .def_readonly("box_sequence",          &sbf::PlanResult::box_sequence)
        .def_readonly("path_length",           &sbf::PlanResult::path_length)
        .def_readonly("planning_time_ms",      &sbf::PlanResult::planning_time_ms)
        .def_readonly("n_boxes",               &sbf::PlanResult::n_boxes)
        .def_readonly("n_coarsen_merges",      &sbf::PlanResult::n_coarsen_merges)
        .def_readonly("envelope_volume_total", &sbf::PlanResult::envelope_volume_total)
        .def_readonly("build_time_ms",         &sbf::PlanResult::build_time_ms)
        .def_readonly("lect_time_ms",          &sbf::PlanResult::lect_time_ms);

    // ─── SBFPlanner ─────────────────────────────────────────────────────
#if 1
    py::class_<sbf::SBFPlanner>(m, "SBFPlanner")
        .def(py::init<const sbf::Robot&, const sbf::SBFPlannerConfig&>(),
             py::arg("robot"), py::arg("config") = sbf::SBFPlannerConfig{},
             py::keep_alive<1, 2>())  // planner keeps robot alive

        .def("plan", [](sbf::SBFPlanner& self,
                        const Eigen::VectorXd& start,
                        const Eigen::VectorXd& goal,
                        const std::vector<sbf::Obstacle>& obstacles,
                        double timeout_ms) {
            py::gil_scoped_release release;
            return self.plan(start, goal,
                             obstacles.data(),
                             static_cast<int>(obstacles.size()),
                             timeout_ms);
        }, py::arg("start"), py::arg("goal"),
           py::arg("obstacles"), py::arg("timeout_ms") = 30000.0)

        .def("build", [](sbf::SBFPlanner& self,
                         const Eigen::VectorXd& start,
                         const Eigen::VectorXd& goal,
                         const std::vector<sbf::Obstacle>& obstacles,
                         double timeout_ms) {
            py::gil_scoped_release release;
            self.build(start, goal,
                       obstacles.data(),
                       static_cast<int>(obstacles.size()),
                       timeout_ms);
        }, py::arg("start"), py::arg("goal"),
           py::arg("obstacles"), py::arg("timeout_ms") = 30000.0)

        .def("build_coverage", [](sbf::SBFPlanner& self,
                                   const std::vector<sbf::Obstacle>& obstacles,
                                   double timeout_ms,
                                   const std::vector<Eigen::VectorXd>& seed_points) {
            py::gil_scoped_release release;
            self.build_coverage(obstacles.data(),
                                static_cast<int>(obstacles.size()),
                                timeout_ms, seed_points);
        }, py::arg("obstacles"), py::arg("timeout_ms") = 30000.0,
           py::arg("seed_points") = std::vector<Eigen::VectorXd>{})

        .def("warmup_lect", [](sbf::SBFPlanner& self,
                                int max_depth, int n_paths, int seed) {
            py::gil_scoped_release release;
            return self.warmup_lect(max_depth, n_paths, seed);
        }, py::arg("max_depth"), py::arg("n_paths"), py::arg("seed") = 42)

        .def("pre_bridge_pairs", [](sbf::SBFPlanner& self,
                                     const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& pairs,
                                     const std::vector<sbf::Obstacle>& obstacles,
                                     double per_pair_timeout_ms,
                                     int max_pairs_per_call) {
            py::gil_scoped_release release;
            return self.pre_bridge_pairs(
                pairs,
                obstacles.data(),
                static_cast<int>(obstacles.size()),
                per_pair_timeout_ms,
                max_pairs_per_call);
        }, py::arg("pairs"), py::arg("obstacles"),
           py::arg("per_pair_timeout_ms") = 800.0,
           py::arg("max_pairs_per_call") = 4)

        .def("query", [](sbf::SBFPlanner& self,
                        const Eigen::VectorXd& start,
                        const Eigen::VectorXd& goal) {
            return self.query(start, goal);
        }, py::arg("start"), py::arg("goal"))

        .def("clear_forest", &sbf::SBFPlanner::clear_forest)
        .def("boxes", &sbf::SBFPlanner::boxes,
             py::return_value_policy::reference_internal)
        .def("raw_boxes", &sbf::SBFPlanner::raw_boxes,
             py::return_value_policy::reference_internal)
           .def("build_timing", &sbf::SBFPlanner::build_timing,
               py::return_value_policy::reference_internal)
        .def("n_boxes", &sbf::SBFPlanner::n_boxes);
#endif

    // ─── compute_envelope_info (Phase U1) ───────────────────────────────
    // High-level function for S1/S2 experiments: computes endpoint IAABBs
    // and link envelope in one call, returning volume + timing.
    m.def("compute_envelope_info", [](
            const sbf::Robot& robot,
            const std::vector<sbf::Interval>& intervals,
            sbf::EndpointSourceConfig ep_config,
            const sbf::EnvelopeTypeConfig& env_config,
            const sbf::GcpcCache* gcpc_cache) -> py::dict {

        if (gcpc_cache) {
            ep_config.gcpc_cache = gcpc_cache;
        }

        using Clock = std::chrono::high_resolution_clock;

        auto t0 = Clock::now();
        auto ep_result = sbf::compute_endpoint_iaabb(
            robot, intervals, ep_config);
        auto t1 = Clock::now();

        auto env_result = sbf::compute_link_envelope(
            ep_result.endpoint_iaabbs.data(),
            ep_result.n_active_links,
            robot.active_link_radii(),
            env_config);
        auto t2 = Clock::now();

        // Volume: grid occupied_volume for Grid types;
        //         for AABB-only, use exact continuous union of inflated AABBs.
        double volume = 0.0;
        if (env_result.has_grid()) {
            volume = env_result.sparse_grid->occupied_volume();
        } else {
            auto exact_union_volume_3d = [](const std::vector<std::array<double, 6>>& boxes) {
                if (boxes.empty()) return 0.0;

                std::vector<double> xs;
                xs.reserve(boxes.size() * 2);
                for (const auto& b : boxes) {
                    xs.push_back(b[0]);
                    xs.push_back(b[3]);
                }
                std::sort(xs.begin(), xs.end());
                xs.erase(std::unique(xs.begin(), xs.end()), xs.end());
                if (xs.size() < 2) return 0.0;

                double total = 0.0;
                for (size_t xi = 0; xi + 1 < xs.size(); ++xi) {
                    const double x0 = xs[xi];
                    const double x1 = xs[xi + 1];
                    if (!(x1 > x0)) continue;

                    std::vector<std::array<double, 4>> rects;
                    rects.reserve(boxes.size());
                    for (const auto& b : boxes) {
                        if (b[0] < x1 && b[3] > x0) {
                            rects.push_back({b[1], b[4], b[2], b[5]});
                        }
                    }
                    if (rects.empty()) continue;

                    std::vector<double> ys;
                    std::vector<double> zs;
                    ys.reserve(rects.size() * 2);
                    zs.reserve(rects.size() * 2);
                    for (const auto& r : rects) {
                        ys.push_back(r[0]);
                        ys.push_back(r[1]);
                        zs.push_back(r[2]);
                        zs.push_back(r[3]);
                    }
                    std::sort(ys.begin(), ys.end());
                    ys.erase(std::unique(ys.begin(), ys.end()), ys.end());
                    std::sort(zs.begin(), zs.end());
                    zs.erase(std::unique(zs.begin(), zs.end()), zs.end());
                    if (ys.size() < 2 || zs.size() < 2) continue;

                    const int ny = static_cast<int>(ys.size() - 1);
                    const int nz = static_cast<int>(zs.size() - 1);
                    std::vector<unsigned char> covered(static_cast<size_t>(ny * nz), 0);

                    for (const auto& r : rects) {
                        const int y0 = static_cast<int>(std::lower_bound(ys.begin(), ys.end(), r[0]) - ys.begin());
                        const int y1 = static_cast<int>(std::lower_bound(ys.begin(), ys.end(), r[1]) - ys.begin());
                        const int z0 = static_cast<int>(std::lower_bound(zs.begin(), zs.end(), r[2]) - zs.begin());
                        const int z1 = static_cast<int>(std::lower_bound(zs.begin(), zs.end(), r[3]) - zs.begin());
                        for (int yi = y0; yi < y1; ++yi) {
                            for (int zi = z0; zi < z1; ++zi) {
                                covered[static_cast<size_t>(yi * nz + zi)] = 1;
                            }
                        }
                    }

                    double area = 0.0;
                    for (int yi = 0; yi < ny; ++yi) {
                        const double dy = ys[yi + 1] - ys[yi];
                        if (!(dy > 0.0)) continue;
                        for (int zi = 0; zi < nz; ++zi) {
                            if (!covered[static_cast<size_t>(yi * nz + zi)]) continue;
                            const double dz = zs[zi + 1] - zs[zi];
                            if (dz > 0.0) area += dy * dz;
                        }
                    }

                    total += area * (x1 - x0);
                }
                return total;
            };

            const double delta = env_config.grid_config.voxel_delta;
            const double pad = std::sqrt(3.0) * delta * 0.5;
            const int n_sub = env_result.n_subdivisions;
            const int n_boxes = env_result.n_active_links * n_sub;
            const double* radii = robot.active_link_radii();

            std::vector<std::array<double, 6>> inflated_boxes;
            inflated_boxes.reserve(static_cast<size_t>(n_boxes));
            for (int i = 0; i < n_boxes; ++i) {
                const float* s = &env_result.link_iaabbs[i * 6];
                const double r = radii ? radii[i / n_sub] + pad : pad;
                inflated_boxes.push_back({
                    static_cast<double>(s[0]) - r,
                    static_cast<double>(s[1]) - r,
                    static_cast<double>(s[2]) - r,
                    static_cast<double>(s[3]) + r,
                    static_cast<double>(s[4]) + r,
                    static_cast<double>(s[5]) + r,
                });
            }
            volume = exact_union_volume_3d(inflated_boxes);
        }

        auto ep_us  = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        auto env_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

        int64_t grid_num_bricks = 0;
        int64_t grid_num_voxels = 0;
        double grid_occupied_volume = 0.0;
        double grid_cache_payload_bytes = 0.0;
        double grid_delta = env_config.grid_config.voxel_delta;
        std::vector<double> grid_xy_points;  // flattened [x0,y0,x1,y1,...]
        if (env_result.sparse_grid) {
            grid_num_bricks = static_cast<int64_t>(env_result.sparse_grid->num_bricks());
            grid_num_voxels = static_cast<int64_t>(env_result.sparse_grid->count_occupied());
            grid_occupied_volume = env_result.sparse_grid->occupied_volume();
            // BitBrick payload only (8 * uint64_t = 64 bytes per brick).
            // Hash-map/node overhead is allocator/container dependent.
            grid_cache_payload_bytes = static_cast<double>(grid_num_bricks) * 64.0;

            // Export XY projection of occupied voxels for diagnostics/visualization.
            // Deduplicate by (cell_x, cell_y) so point count is bounded by columns,
            // not by all occupied z slices.
            const auto* g = env_result.sparse_grid.get();
            const double delta = g->delta();
            const double* org = g->origin();

            struct CellXY {
                int x = 0;
                int y = 0;
                bool operator==(const CellXY& o) const noexcept {
                    return x == o.x && y == o.y;
                }
            };
            struct CellXYHash {
                std::size_t operator()(const CellXY& c) const noexcept {
                    std::size_t h = 14695981039346656037ULL;
                    h ^= static_cast<std::size_t>(c.x); h *= 1099511628211ULL;
                    h ^= static_cast<std::size_t>(c.y); h *= 1099511628211ULL;
                    return h;
                }
            };

            std::unordered_set<CellXY, CellXYHash> seen;
            seen.reserve(static_cast<std::size_t>(std::max<int64_t>(1024, grid_num_voxels / 4)));

            constexpr std::size_t kMaxXY = 50000;
            for (auto e : g->bricks()) {
                const auto& bc = e.key;
                const auto& brick = e.value;
                for (int lz = 0; lz < 8; ++lz) {
                    uint64_t word = brick.words[lz];
                    while (word) {
                        const int bit = __builtin_ctzll(word);
                        word &= (word - 1);
                        const int lx = bit % 8;
                        const int ly = bit / 8;
                        const int cx = bc.bx * 8 + lx;
                        const int cy = bc.by * 8 + ly;
                        CellXY cell{cx, cy};
                        if (seen.insert(cell).second) {
                            const double wx = org[0] + (static_cast<double>(cx) + 0.5) * delta;
                            const double wy = org[1] + (static_cast<double>(cy) + 0.5) * delta;
                            grid_xy_points.push_back(wx);
                            grid_xy_points.push_back(wy);
                            if (grid_xy_points.size() / 2 >= kMaxXY) {
                                break;
                            }
                        }
                    }
                    if (grid_xy_points.size() / 2 >= kMaxXY) {
                        break;
                    }
                }
                if (grid_xy_points.size() / 2 >= kMaxXY) {
                    break;
                }
            }
        }

        py::dict result;
        result["volume"]         = volume;
        result["is_safe"]        = ep_result.is_safe;
        result["n_active_links"] = ep_result.n_active_links;
        result["n_pruned_links"] = ep_result.n_pruned_links;
        result["ep_time_us"]     = static_cast<int64_t>(ep_us);
        result["env_time_us"]    = static_cast<int64_t>(env_us);
        result["total_time_us"]  = static_cast<int64_t>(ep_us + env_us);
        result["grid_delta"] = grid_delta;
        result["grid_num_bricks"] = grid_num_bricks;
        result["grid_num_voxels"] = grid_num_voxels;
        result["grid_occupied_volume"] = grid_occupied_volume;
        result["grid_cache_payload_bytes"] = grid_cache_payload_bytes;
        result["link_iaabbs"] = env_result.link_iaabbs;
        result["n_subdivisions"] = env_result.n_subdivisions;
        result["grid_xy_points"] = grid_xy_points;
        return result;

    }, py::arg("robot"), py::arg("intervals"),
       py::arg("ep_config"), py::arg("env_config"),
       py::arg("gcpc_cache") = nullptr,
       "Compute endpoint IAABBs + link envelope, return volume & timing.");

    // ─── compute_link_iaabb_info (for per-axis gap diagnostics) ─────────
    m.def("compute_link_iaabb_info", [](
            const sbf::Robot& robot,
            const std::vector<sbf::Interval>& intervals,
            sbf::EndpointSourceConfig ep_config,
            const sbf::GcpcCache* gcpc_cache) -> py::dict {

        if (gcpc_cache) {
            ep_config.gcpc_cache = gcpc_cache;
        }

        auto ep_result = sbf::compute_endpoint_iaabb(robot, intervals, ep_config);

        sbf::EnvelopeTypeConfig env_cfg;
        env_cfg.type = sbf::EnvelopeType::LinkIAABB;
        auto env_result = sbf::compute_link_envelope(
            ep_result.endpoint_iaabbs.data(),
            ep_result.n_active_links,
            robot.active_link_radii(),
            env_cfg);

        py::dict result;
        result["link_iaabbs"] = env_result.link_iaabbs;
        result["n_active_links"] = env_result.n_active_links;
        result["n_subdivisions"] = env_result.n_subdivisions;
        result["is_safe"] = ep_result.is_safe;
        return result;

    }, py::arg("robot"), py::arg("intervals"),
       py::arg("ep_config"), py::arg("gcpc_cache") = nullptr,
       "Compute LinkIAABB and return raw [n_links*n_sub*6] array for diagnostics.");

    // ─── compute_endpoint_iaabb_info (endpoint-source diagnostics) ─────
    m.def("compute_endpoint_iaabb_info", [](
            const sbf::Robot& robot,
            const std::vector<sbf::Interval>& intervals,
            sbf::EndpointSourceConfig ep_config,
            const sbf::GcpcCache* gcpc_cache) -> py::dict {

        if (gcpc_cache) {
            ep_config.gcpc_cache = gcpc_cache;
        }

        auto t0 = std::chrono::steady_clock::now();
        auto ep_result = sbf::compute_endpoint_iaabb(robot, intervals, ep_config);
        auto t1 = std::chrono::steady_clock::now();
        auto ep_us = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();

        // Total volume of endpoint IAABBs (sum over n_active * 2 boxes).
        double volume_sum = 0.0;
        const int n_boxes = ep_result.n_active_links * 2;
        for (int i = 0; i < n_boxes; ++i) {
            const float* b = &ep_result.endpoint_iaabbs[i * 6];
            const double dx = std::max(0.0, static_cast<double>(b[3] - b[0]));
            const double dy = std::max(0.0, static_cast<double>(b[4] - b[1]));
            const double dz = std::max(0.0, static_cast<double>(b[5] - b[2]));
            volume_sum += dx * dy * dz;
        }

        py::dict result;
        result["endpoint_iaabbs"] = ep_result.endpoint_iaabbs;
        result["source"] = std::string(sbf::endpoint_source_name(ep_result.source));
        result["is_safe"] = ep_result.is_safe;
        result["n_active_links"] = ep_result.n_active_links;
        result["n_pruned_links"] = ep_result.n_pruned_links;
        result["ep_time_us"] = static_cast<int64_t>(ep_us);
        result["volume_sum"] = volume_sum;
        return result;

    }, py::arg("robot"), py::arg("intervals"),
       py::arg("ep_config"), py::arg("gcpc_cache") = nullptr,
       "Compute endpoint IAABBs and return raw boxes, volume sum and endpoint timing.");
}
