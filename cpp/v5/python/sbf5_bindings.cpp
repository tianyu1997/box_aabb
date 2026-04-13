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
#include <string>
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
        .def_readwrite("deadline_ms", &sbf::FFBConfig::deadline_ms);

    py::class_<sbf::GrowerConfig>(m, "GrowerConfig")
        .def(py::init<>())
        .def_readwrite("mode",             &sbf::GrowerConfig::mode)
        .def_readwrite("ffb_config",       &sbf::GrowerConfig::ffb_config)
        .def_readwrite("max_boxes",        &sbf::GrowerConfig::max_boxes)
        .def_readwrite("timeout_ms",       &sbf::GrowerConfig::timeout_ms)
        .def_readwrite("max_consecutive_miss", &sbf::GrowerConfig::max_consecutive_miss)
        .def_readwrite("rrt_goal_bias",    &sbf::GrowerConfig::rrt_goal_bias)
        .def_readwrite("rrt_step_ratio",   &sbf::GrowerConfig::rrt_step_ratio)
        .def_readwrite("enable_promotion", &sbf::GrowerConfig::enable_promotion)
        .def_readwrite("rng_seed",         &sbf::GrowerConfig::rng_seed)
        .def_readwrite("n_threads",        &sbf::GrowerConfig::n_threads)
        .def_readwrite("bridge_n_threads", &sbf::GrowerConfig::bridge_n_threads)
        .def_readwrite("connect_mode",     &sbf::GrowerConfig::connect_mode);

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
        .def_readwrite("bezier_degree",  &sbf::GCSConfig::bezier_degree)
        .def_readwrite("time_limit_sec", &sbf::GCSConfig::time_limit_sec);

    // ─── EndpointSource enum (Phase R2) ─────────────────────────────────
    py::enum_<sbf::EndpointSource>(m, "EndpointSource")
        .value("IFK",        sbf::EndpointSource::IFK)
        .value("CritSample", sbf::EndpointSource::CritSample)
        .value("Analytical", sbf::EndpointSource::Analytical)
        .value("GCPC",       sbf::EndpointSource::GCPC);

    // ─── EnvelopeType enum (Phase R2) ───────────────────────────────────
    py::enum_<sbf::EnvelopeType>(m, "EnvelopeType")
        .value("LinkIAABB",      sbf::EnvelopeType::LinkIAABB)
        .value("LinkIAABB_Grid", sbf::EnvelopeType::LinkIAABB_Grid)
        .value("Hull16_Grid",    sbf::EnvelopeType::Hull16_Grid);

    // ─── EndpointSourceConfig (Phase R2) ────────────────────────────────
    py::class_<sbf::EndpointSourceConfig>(m, "EndpointSourceConfig")
        .def(py::init<>())
        .def_readwrite("source",              &sbf::EndpointSourceConfig::source)
        .def_readwrite("n_samples_crit",      &sbf::EndpointSourceConfig::n_samples_crit)
        .def_readwrite("max_phase_analytical", &sbf::EndpointSourceConfig::max_phase_analytical)
        .def("set_gcpc_cache", [](sbf::EndpointSourceConfig& self,
                                   const sbf::GcpcCache& cache) {
            self.gcpc_cache = &cache;
        }, py::arg("cache"),
           "Set GCPC cache (caller must keep the cache alive)");

    // ─── EnvelopeTypeConfig (Phase R2) ──────────────────────────────────
    py::class_<sbf::EnvelopeTypeConfig>(m, "EnvelopeTypeConfig")
        .def(py::init<>())
        .def_readwrite("type",           &sbf::EnvelopeTypeConfig::type)
        .def_readwrite("n_subdivisions", &sbf::EnvelopeTypeConfig::n_subdivisions);

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
        .def_readwrite("lect_no_cache",   &sbf::SBFPlannerConfig::lect_no_cache)
        .def_readwrite("lect_cache_dir",  &sbf::SBFPlannerConfig::lect_cache_dir);

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
                                   double timeout_ms) {
            py::gil_scoped_release release;
            self.build_coverage(obstacles.data(),
                                static_cast<int>(obstacles.size()),
                                timeout_ms);
        }, py::arg("obstacles"), py::arg("timeout_ms") = 30000.0)

        .def("warmup_lect", [](sbf::SBFPlanner& self,
                                int max_depth, int n_paths, int seed) {
            py::gil_scoped_release release;
            return self.warmup_lect(max_depth, n_paths, seed);
        }, py::arg("max_depth"), py::arg("n_paths"), py::arg("seed") = 42)

        .def("query", &sbf::SBFPlanner::query,
             py::arg("start"), py::arg("goal"))

        .def("clear_forest", &sbf::SBFPlanner::clear_forest)
        .def("boxes", &sbf::SBFPlanner::boxes,
             py::return_value_policy::reference_internal)
        .def("raw_boxes", &sbf::SBFPlanner::raw_boxes,
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
        //         for AABB-only, build temp grid with inflated AABBs for fair comparison
        double volume = 0.0;
        if (env_result.has_grid()) {
            volume = env_result.sparse_grid->occupied_volume();
        } else {
            // Sum individual inflated AABB volumes (conservative upper bound)
            const double delta = env_config.grid_config.voxel_delta;
            const double pad = std::sqrt(3.0) * delta * 0.5;
            const int n_sub = env_result.n_subdivisions;
            const int n_boxes = env_result.n_active_links * n_sub;
            const double* radii = robot.active_link_radii();
            for (int i = 0; i < n_boxes; ++i) {
                const float* s = &env_result.link_iaabbs[i * 6];
                double r = radii ? radii[i / n_sub] + pad : pad;
                double dx = static_cast<double>(s[3] - s[0]) + 2.0 * r;
                double dy = static_cast<double>(s[4] - s[1]) + 2.0 * r;
                double dz = static_cast<double>(s[5] - s[2]) + 2.0 * r;
                volume += dx * dy * dz;
            }
        }

        auto ep_us  = std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count();
        auto env_us = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();

        py::dict result;
        result["volume"]         = volume;
        result["is_safe"]        = ep_result.is_safe;
        result["n_active_links"] = ep_result.n_active_links;
        result["n_pruned_links"] = ep_result.n_pruned_links;
        result["ep_time_us"]     = static_cast<int64_t>(ep_us);
        result["env_time_us"]    = static_cast<int64_t>(env_us);
        result["total_time_us"]  = static_cast<int64_t>(ep_us + env_us);
        return result;

    }, py::arg("robot"), py::arg("intervals"),
       py::arg("ep_config"), py::arg("env_config"),
       py::arg("gcpc_cache") = nullptr,
       "Compute endpoint IAABBs + link envelope, return volume & timing.");
}
