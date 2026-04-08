// ═══════════════════════════════════════════════════════════════════════════
//  SafeBoxForest v4 — pybind11 Python bindings
//  Exposes ForestGrower (serial + parallel) to Python.
// ═══════════════════════════════════════════════════════════════════════════
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/functional.h>

#include "sbf/robot/robot.h"
#include "sbf/core/types.h"
#include "sbf/forest/forest_grower.h"
#include "sbf/forest/grower_config.h"
#include "sbf/envelope/endpoint_source.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/envelope/gcpc.h"
#include "sbf/envelope/pipeline.h"
#include "sbf/robot/fk.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <Eigen/Core>
#include <optional>
#include <random>
#include <string>
#include <vector>

namespace py = pybind11;
using namespace sbf;
using namespace sbf::forest;
using namespace sbf::envelope;

// ─── helpers ─────────────────────────────────────────────────────────────────

// Convert Python obstacle list to C++ vector.
// Python Obstacle has .min_point, .max_point (3-element arrays/ndarrays).
static std::vector<Obstacle> obstacles_from_python(const py::list& py_obs) {
    std::vector<Obstacle> obs;
    obs.reserve(py_obs.size());
    for (const auto& item : py_obs) {
        auto min_pt = item.attr("min_point").cast<Eigen::Vector3d>();
        auto max_pt = item.attr("max_point").cast<Eigen::Vector3d>();
        Eigen::Vector3d center = 0.5 * (min_pt + max_pt);
        Eigen::Vector3d half   = 0.5 * (max_pt - min_pt);
        std::string name;
        try { name = item.attr("name").cast<std::string>(); }
        catch (...) { name = ""; }
        obs.emplace_back(center, half, name);
    }
    return obs;
}

// Convert Python DH params list to C++ vector.
// Python: list of dicts with alpha, a, d, theta, type.
static std::vector<DHParam> dh_from_python(const py::list& py_dh) {
    std::vector<DHParam> dh;
    dh.reserve(py_dh.size());
    for (const auto& item : py_dh) {
        DHParam p;
        auto d = item.cast<py::dict>();
        p.alpha = d["alpha"].cast<double>();
        p.a     = d["a"].cast<double>();
        p.d     = d["d"].cast<double>();
        p.theta = d["theta"].cast<double>();
        auto type_str = d["type"].cast<std::string>();
        p.joint_type = (type_str == "prismatic") ? 1 : 0;
        dh.push_back(p);
    }
    return dh;
}

static GcpcCache build_gcpc_cache_for_robot(
    const Robot& robot,
    int n_random_seeds = 500,
    int max_sweeps = 5,
    int max_exact_product = 50000,
    int n_random_configs = 5000) {
    const int n = robot.n_joints();
    const int n_act = robot.n_active_links();
    const int* link_map = robot.active_link_map();
    auto limits = robot.joint_limits();

    std::vector<std::vector<double>> per_joint(n);
    for (int j = 0; j < n; ++j) {
        double lo = limits.limits[j].lo;
        double hi = limits.limits[j].hi;
        per_joint[j].push_back(lo);
        per_joint[j].push_back(hi);
        per_joint[j].push_back(0.5 * (lo + hi));
        for (int k = -20; k <= 20; ++k) {
            double angle = k * HALF_PI;
            if (angle > lo + 1e-10 && angle < hi - 1e-10)
                per_joint[j].push_back(angle);
        }
        std::sort(per_joint[j].begin(), per_joint[j].end());
        per_joint[j].erase(
            std::unique(per_joint[j].begin(), per_joint[j].end()),
            per_joint[j].end());
    }

    constexpr double kPi = 3.14159265358979323846;

    std::vector<double> q1_half;
    for (double value : per_joint[1]) {
        if (value >= -1e-10 && value <= kPi + 1e-10)
            q1_half.push_back(std::max(0.0, std::min(kPi, value)));
    }
    if (q1_half.empty())
        q1_half.push_back(0.5);

    std::vector<GcpcPoint> cache_points;
    for (int ci = 0; ci < n_act; ++ci) {
        int link_id = link_map[ci];
        int nj = std::min(link_id + 1, n);
        int n_eff = nj - 1;
        if (n_eff <= 0)
            continue;

        std::vector<std::vector<double>> joint_values(n_eff);
        joint_values[0] = q1_half;
        for (int d = 1; d < n_eff; ++d)
            joint_values[d] = per_joint[d + 1];

        long long product = 1;
        for (int d = 0; d < n_eff; ++d) {
            product *= static_cast<long long>(joint_values[d].size());
            if (product > max_exact_product) {
                product = static_cast<long long>(max_exact_product) + 1;
                break;
            }
        }

        auto eval_config = [&](const std::vector<double>& config) {
            GcpcPoint point{};
            point.link_id = link_id;
            point.n_eff = n_eff;
            for (int i = 0; i < n_eff; ++i)
                point.q_eff[i] = config[i];

            Eigen::VectorXd q_full(n);
            q_full.setZero();
            for (int i = 0; i < n_eff && i + 1 < n; ++i)
                q_full[i + 1] = point.q_eff[i];

            auto positions = fk_link_positions(robot, q_full);
            if (link_id + 1 < static_cast<int>(positions.size())) {
                const auto& p = positions[link_id + 1];
                point.direction = 0;
                point.A = p[0];
                point.B = p[1];
                point.C = p[2];
                point.R = std::sqrt(point.A * point.A + point.B * point.B);
                cache_points.push_back(point);

                GcpcPoint point_z = point;
                point_z.direction = 1;
                cache_points.push_back(point_z);
            }
        };

        if (product > max_exact_product) {
            std::mt19937 rng(12345 + ci);
            for (int sample = 0; sample < n_random_configs; ++sample) {
                std::vector<double> config(n_eff);
                for (int d = 0; d < n_eff; ++d) {
                    auto dist = std::uniform_int_distribution<int>(
                        0, static_cast<int>(joint_values[d].size()) - 1);
                    config[d] = joint_values[d][dist(rng)];
                }
                eval_config(config);
            }
        } else {
            std::vector<double> config(n_eff);
            std::function<void(int)> enumerate;
            enumerate = [&](int d) {
                if (d >= n_eff) {
                    eval_config(config);
                    return;
                }
                for (double value : joint_values[d]) {
                    config[d] = value;
                    enumerate(d + 1);
                }
            };
            enumerate(0);
        }
    }

    GcpcCache cache;
    cache.build(robot, cache_points);
    cache.enrich_with_interior_search(robot, n_random_seeds, max_sweeps);
    return cache;
}

// ─── Module ──────────────────────────────────────────────────────────────────

PYBIND11_MODULE(_sbf4_cpp, m) {
    m.doc() = "SafeBoxForest v4 C++ bindings — ForestGrower with native threading";

    // ── Interval ─────────────────────────────────────────────────────────
    py::class_<Interval>(m, "Interval")
        .def(py::init<>())
        .def(py::init<double, double>())
        .def_readwrite("lo", &Interval::lo)
        .def_readwrite("hi", &Interval::hi)
        .def("width",  &Interval::width)
        .def("center", &Interval::center)
        .def("__repr__", [](const Interval& iv) {
            return "Interval(" + std::to_string(iv.lo) + ", "
                   + std::to_string(iv.hi) + ")";
        });

    // ── Obstacle (C++ side) ──────────────────────────────────────────────
    py::class_<Obstacle>(m, "Obstacle")
        .def(py::init<>())
        .def(py::init<Eigen::Vector3d, Eigen::Vector3d, std::string>(),
             py::arg("center"), py::arg("half_sizes"), py::arg("name") = "")
        .def_readwrite("center",     &Obstacle::center)
        .def_readwrite("half_sizes", &Obstacle::half_sizes)
        .def_readwrite("name",       &Obstacle::name);

    // ── BoxNode ──────────────────────────────────────────────────────────
    py::class_<BoxNode>(m, "BoxNode")
        .def_readwrite("id",               &BoxNode::id)
        .def_readwrite("joint_intervals",  &BoxNode::joint_intervals)
        .def_readwrite("seed_config",      &BoxNode::seed_config)
        .def_readwrite("volume",           &BoxNode::volume)
        .def_readwrite("parent_box_id",    &BoxNode::parent_box_id)
        .def_readwrite("root_id",          &BoxNode::root_id)
        .def_readwrite("tree_id",          &BoxNode::tree_id)
        .def("n_dims",  &BoxNode::n_dims)
        .def("center",  &BoxNode::center)
        .def("contains", &BoxNode::contains, py::arg("q"), py::arg("tol") = 0.0);

    // ── GrowerConfig ─────────────────────────────────────────────────────
    py::class_<GrowerConfig> gc(m, "GrowerConfig");
    gc.def(py::init<>())
        .def_readwrite("n_roots",                &GrowerConfig::n_roots)
        .def_readwrite("max_boxes",              &GrowerConfig::max_boxes)
        .def_readwrite("min_edge",               &GrowerConfig::min_edge)
        .def_readwrite("max_depth",              &GrowerConfig::max_depth)
        .def_readwrite("max_consecutive_miss",   &GrowerConfig::max_consecutive_miss)
        .def_readwrite("timeout",                &GrowerConfig::timeout)
        .def_readwrite("adjacency_tol",          &GrowerConfig::adjacency_tol)
        .def_readwrite("rng_seed",               &GrowerConfig::rng_seed)
        .def_readwrite("n_boundary_samples",     &GrowerConfig::n_boundary_samples)
        .def_readwrite("boundary_epsilon",       &GrowerConfig::boundary_epsilon)
        .def_readwrite("goal_face_bias",         &GrowerConfig::goal_face_bias)
        .def_readwrite("rrt_step_ratio",         &GrowerConfig::rrt_step_ratio)
        .def_readwrite("rrt_goal_bias",          &GrowerConfig::rrt_goal_bias)
        .def_readwrite("n_threads",              &GrowerConfig::n_threads)
        .def_readwrite("adaptive_min_edge",      &GrowerConfig::adaptive_min_edge)
        .def_readwrite("coarse_min_edge",        &GrowerConfig::coarse_min_edge)
        .def_readwrite("coarse_fraction",        &GrowerConfig::coarse_fraction)
        .def_readwrite("adaptive_n_stages",      &GrowerConfig::adaptive_n_stages)
        .def_readwrite("bridge_samples",         &GrowerConfig::bridge_samples)
        .def_readwrite("root_min_edge",          &GrowerConfig::root_min_edge)
        .def_readwrite("root_n_candidates",      &GrowerConfig::root_n_candidates)
        .def_readwrite("hull_skip_vol",          &GrowerConfig::hull_skip_vol)
        .def_readwrite("coarsen_enabled",        &GrowerConfig::coarsen_enabled)
        .def_readwrite("coarsen_target_boxes",   &GrowerConfig::coarsen_target_boxes)
        .def_readwrite("max_coarsen_rounds",     &GrowerConfig::max_coarsen_rounds)
        .def_readwrite("coarsen_score_threshold",&GrowerConfig::coarsen_score_threshold)
        .def_readwrite("warm_start_depth",       &GrowerConfig::warm_start_depth)
        .def_readwrite("lect_cache_dir",         &GrowerConfig::lect_cache_dir);

    py::enum_<GrowerConfig::Mode>(gc, "Mode")
        .value("Wavefront", GrowerConfig::Mode::Wavefront)
        .value("RRT",       GrowerConfig::Mode::RRT)
        .export_values();

    gc.def_readwrite("mode", &GrowerConfig::mode);

    // ── PartitionMode enum ──────────────────────────────────────────────
    py::enum_<PartitionMode>(m, "PartitionMode")
        .value("KDSplit",     PartitionMode::KDSplit)
        .value("Uniform",    PartitionMode::Uniform)
        .value("LectAligned", PartitionMode::LectAligned)
        .export_values();

    gc.def_readwrite("partition_mode", &GrowerConfig::partition_mode);

    // ── SplitOrder enum ─────────────────────────────────────────────────
    py::enum_<SplitOrder>(m, "SplitOrder")
        .value("ROUND_ROBIN",  SplitOrder::ROUND_ROBIN)
        .value("WIDEST_FIRST", SplitOrder::WIDEST_FIRST)
        .value("BEST_TIGHTEN", SplitOrder::BEST_TIGHTEN)
        .export_values();

    // ── EndpointSource enum ─────────────────────────────────────────────
    py::enum_<EndpointSource>(m, "EndpointSource")
        .value("IFK",        EndpointSource::IFK)
        .value("CritSample", EndpointSource::CritSample)
        .value("Analytical", EndpointSource::Analytical)
        .value("GCPC",       EndpointSource::GCPC)
        .export_values();

        py::class_<GcpcCache>(m, "GcpcCache")
           .def(py::init<>())
           .def("load_json", &GcpcCache::load_json,
               py::arg("path"), py::arg("robot"),
               "Load a GCPC cache from JSON and build per-link KD-trees")
           .def("load", &GcpcCache::load,
               py::arg("path"),
               "Load a GCPC cache from the binary GCPC format")
           .def("save", &GcpcCache::save,
               py::arg("path"),
               "Save a GCPC cache to the binary GCPC format")
           .def("enrich_with_interior_search", &GcpcCache::enrich_with_interior_search,
               py::arg("robot"), py::arg("n_random_seeds") = 500, py::arg("max_sweeps") = 5)
           .def("precompute_pair_kpi2", &GcpcCache::precompute_pair_kpi2, py::arg("robot"))
           .def("is_loaded", &GcpcCache::is_loaded)
           .def("n_links", &GcpcCache::n_links)
           .def("n_total_points", &GcpcCache::n_total_points);

        m.def("build_gcpc_cache",
            [](const Robot& robot,
               int n_random_seeds,
               int max_sweeps,
               int max_exact_product,
               int n_random_configs) {
                return build_gcpc_cache_for_robot(
                   robot,
                   n_random_seeds,
                   max_sweeps,
                   max_exact_product,
                   n_random_configs);
            },
            py::arg("robot"),
            py::arg("n_random_seeds") = 500,
            py::arg("max_sweeps") = 5,
            py::arg("max_exact_product") = 50000,
            py::arg("n_random_configs") = 5000,
            "Build a GCPC cache inline from a robot model using k*pi/2 enumeration and interior enrichment");

    // ── EndpointSourceConfig ─────────────────────────────────────────────
    py::class_<EndpointSourceConfig>(m, "EndpointSourceConfig")
        .def(py::init<>())
        .def_readwrite("method", &EndpointSourceConfig::method)
        .def_static("ifk",           &EndpointSourceConfig::ifk)
        .def_static("crit_sampling", &EndpointSourceConfig::crit_sampling)
           .def_static("analytical",    &EndpointSourceConfig::analytical)
           .def_static("gcpc",          &EndpointSourceConfig::gcpc,
                    py::arg("cache"));

    // ── EnvelopeType enum ────────────────────────────────────────────────
    py::enum_<EnvelopeType>(m, "EnvelopeType")
        .value("LinkIAABB",      EnvelopeType::LinkIAABB)
        .value("LinkIAABB_Grid", EnvelopeType::LinkIAABB_Grid)
        .value("Hull16_Grid",    EnvelopeType::Hull16_Grid)
        .export_values();

    // ── EnvelopeTypeConfig ───────────────────────────────────────────────
    py::class_<EnvelopeTypeConfig>(m, "EnvelopeTypeConfig")
        .def(py::init<>())
        .def_readwrite("type",   &EnvelopeTypeConfig::type)
        .def_readwrite("n_sub",  &EnvelopeTypeConfig::n_sub)
        .def_readwrite("delta",  &EnvelopeTypeConfig::delta)
        .def_readwrite("grid_R", &EnvelopeTypeConfig::grid_R)
        .def_static("link_iaabb",      &EnvelopeTypeConfig::link_iaabb)
        .def_static("link_iaabb_grid", &EnvelopeTypeConfig::link_iaabb_grid)
        .def_static("hull16_grid",     &EnvelopeTypeConfig::hull16_grid);

    // ── PipelineConfig ───────────────────────────────────────────────────
    py::class_<PipelineConfig>(m, "PipelineConfig")
        .def(py::init<>())
        .def_readwrite("source",   &PipelineConfig::source)
        .def_readwrite("envelope", &PipelineConfig::envelope)
        .def_readwrite("aa_crossover_width", &PipelineConfig::aa_crossover_width)
        .def_static("recommended", py::overload_cast<>(&PipelineConfig::recommended),
                     "Default recommended pipeline (CritSample + Hull16_Grid)")
        .def_static("recommended", py::overload_cast<const GcpcCache*>(&PipelineConfig::recommended),
                     py::arg("cache"),
                     "Recommended cached pipeline (GCPC + Hull16_Grid)")
        .def_static("tightest", py::overload_cast<>(&PipelineConfig::tightest),
                     "Tightest cache-less pipeline (Analytical + Hull16_Grid)")
        .def_static("tightest", py::overload_cast<const GcpcCache*>(&PipelineConfig::tightest),
                     py::arg("cache"),
                     "Tightest cached pipeline (GCPC + Hull16_Grid)")
        .def_static("production", py::overload_cast<>(&PipelineConfig::production),
                     "Production cache-less pipeline (IFK + Hull16_Grid)")
        .def_static("production", py::overload_cast<const GcpcCache*>(&PipelineConfig::production),
                     py::arg("cache"),
                     "Production cached pipeline (GCPC + Hull16_Grid)")
        .def_static("fast", &PipelineConfig::fast,
                     "Fast pipeline (iFK + LinkIAABB, no cache)");

    gc.def_readwrite("pipeline", &GrowerConfig::pipeline);

    // ── GrowerResult ─────────────────────────────────────────────────────
    py::class_<GrowerResult>(m, "GrowerResult")
        .def(py::init<>())
        .def_readwrite("boxes",                &GrowerResult::boxes)
        .def_readwrite("n_roots",              &GrowerResult::n_roots)
        .def_readwrite("n_boxes_total",        &GrowerResult::n_boxes_total)
        .def_readwrite("n_ffb_success",        &GrowerResult::n_ffb_success)
        .def_readwrite("n_ffb_fail",           &GrowerResult::n_ffb_fail)
        .def_readwrite("n_promotions",         &GrowerResult::n_promotions)
        .def_readwrite("n_bridge_boxes",       &GrowerResult::n_bridge_boxes)
        .def_readwrite("n_coarse_boxes",       &GrowerResult::n_coarse_boxes)
        .def_readwrite("n_fine_boxes",         &GrowerResult::n_fine_boxes)
        .def_readwrite("n_coarsen_merges",     &GrowerResult::n_coarsen_merges)
        .def_readwrite("n_components",         &GrowerResult::n_components)
        .def_readwrite("start_goal_connected", &GrowerResult::start_goal_connected)
        .def_readwrite("total_volume",         &GrowerResult::total_volume)
        .def_readwrite("build_time_ms",        &GrowerResult::build_time_ms)
        .def_readwrite("phase_times",          &GrowerResult::phase_times);

    // ── DHParam ──────────────────────────────────────────────────────────
    py::class_<DHParam>(m, "DHParam")
        .def(py::init<>())
        .def_readwrite("alpha",      &DHParam::alpha)
        .def_readwrite("a",          &DHParam::a)
        .def_readwrite("d",          &DHParam::d)
        .def_readwrite("theta",      &DHParam::theta)
        .def_readwrite("joint_type", &DHParam::joint_type);

    // ── JointLimits ──────────────────────────────────────────────────────
    py::class_<JointLimits>(m, "JointLimits")
        .def(py::init<>())
        .def_readwrite("limits", &JointLimits::limits)
        .def("n_dims", &JointLimits::n_dims);

    // ── Robot ────────────────────────────────────────────────────────────
    py::class_<Robot>(m, "Robot")
        .def(py::init<>())
        .def_static("from_json", &Robot::from_json, py::arg("path"),
                     "Load robot from JSON config file")
        .def(py::init([](const std::string& name,
                         const py::list& py_dh,
                         const py::list& py_limits,
                         const py::object& py_tool,
                         const std::vector<double>& link_radii) {
            auto dh = dh_from_python(py_dh);
            JointLimits jl;
            for (const auto& item : py_limits) {
                auto p = item.cast<py::tuple>();
                jl.limits.push_back({p[0].cast<double>(), p[1].cast<double>()});
            }
            std::optional<DHParam> tool;
            if (!py_tool.is_none()) {
                auto td = py_tool.cast<py::dict>();
                DHParam tp;
                tp.alpha = td.contains("alpha") ? td["alpha"].cast<double>() : 0.0;
                tp.a     = td.contains("a") ? td["a"].cast<double>() : 0.0;
                tp.d     = td.contains("d") ? td["d"].cast<double>() : 0.0;
                tp.theta = td.contains("theta") ? td["theta"].cast<double>() : 0.0;
                tp.joint_type = 0;
                tool = tp;
            }
            return Robot(name, dh, jl, tool, link_radii);
        }), py::arg("name"), py::arg("dh_params"), py::arg("joint_limits"),
            py::arg("tool_frame") = py::none(),
            py::arg("link_radii") = std::vector<double>{},
            "Construct Robot from DH parameters")
        .def("name",        &Robot::name)
        .def("n_joints",    &Robot::n_joints)
        .def("n_links",     &Robot::n_links)
        .def("n_endpoints", &Robot::n_endpoints)
        .def("joint_limits", &Robot::joint_limits, py::return_value_policy::reference_internal)
        .def("has_tool",     &Robot::has_tool)
        .def("link_radii",   &Robot::link_radii)
        .def("fingerprint",  &Robot::fingerprint);

    // ── ForestGrower ─────────────────────────────────────────────────────
    py::class_<ForestGrower>(m, "ForestGrower")
        .def(py::init<const Robot&, const GrowerConfig&>(),
             py::arg("robot"), py::arg("config"))
        .def("set_endpoints", &ForestGrower::set_endpoints,
             py::arg("start"), py::arg("goal"))
        .def("set_split_order", [](ForestGrower& grower, SplitOrder so) {
            grower.lect_mut().set_split_order(so);
        }, py::arg("split_order"),
           "Set the internal LECT split-dimension policy")
        .def("split_order", [](const ForestGrower& grower) {
            return grower.lect().split_order();
        }, "Get the current internal LECT split-dimension policy")
        // Main grow method — accepts a Python obstacle list (from Scene.get_obstacles())
        .def("grow", [](ForestGrower& grower, const py::list& py_obs) -> GrowerResult {
            auto obs = obstacles_from_python(py_obs);
            py::gil_scoped_release release;  // release GIL for C++ threading
            return grower.grow(obs.data(), static_cast<int>(obs.size()));
        }, py::arg("obstacles"),
           "Grow the forest (releases GIL for parallel execution).\n"
           "obstacles: list of Python Obstacle objects with .min_point/.max_point")
        // Also provide a typed version for C++ Obstacle vector
        .def("grow_typed", [](ForestGrower& grower,
                              const std::vector<Obstacle>& obs) -> GrowerResult {
            py::gil_scoped_release release;
            return grower.grow(obs.data(), static_cast<int>(obs.size()));
        }, py::arg("obstacles"),
           "Grow the forest with C++ Obstacle vector")
        .def("n_boxes",  &ForestGrower::n_boxes)
        .def("boxes",    &ForestGrower::boxes, py::return_value_policy::reference_internal)
        .def("config",   &ForestGrower::config, py::return_value_policy::reference_internal)
        // Return adjacency as Python dict {box_id: [neighbor_ids]}
        .def("get_adjacency_dict", [](const ForestGrower& grower) -> py::dict {
            py::dict adj;
            const auto& g = grower.graph();
            for (const auto& box : grower.boxes()) {
                py::list nbrs;
                for (int nb : g.neighbors(box.id))
                    nbrs.append(nb);
                adj[py::int_(box.id)] = nbrs;
            }
            return adj;
        }, "Return adjacency graph as dict {box_id: [neighbor_ids]}");

    // ── Helper: convert Python robot to C++ Robot ────────────────────────
    m.def("robot_from_python", [](const py::object& py_robot) -> Robot {
        std::string name = py_robot.attr("name").cast<std::string>();
        auto py_dh = py_robot.attr("dh_params").cast<py::list>();
        auto dh = dh_from_python(py_dh);

        JointLimits jl;
        auto py_jl = py_robot.attr("joint_limits");
        if (!py_jl.is_none()) {
            for (const auto& item : py_jl.cast<py::list>()) {
                auto p = item.cast<py::tuple>();
                jl.limits.push_back({p[0].cast<double>(), p[1].cast<double>()});
            }
        }

        std::optional<DHParam> tool;
        auto py_tool = py_robot.attr("tool_frame");
        if (!py_tool.is_none()) {
            auto td = py_tool.cast<py::dict>();
            DHParam tp;
            tp.alpha = td.contains("alpha") ? td["alpha"].cast<double>() : 0.0;
            tp.a     = td.contains("a") ? td["a"].cast<double>() : 0.0;
            tp.d     = td.contains("d") ? td["d"].cast<double>() : 0.0;
            tp.theta = td.contains("theta") ? td["theta"].cast<double>() : 0.0;
            tp.joint_type = 0;
            tool = tp;
        }

        std::vector<double> radii;
        auto py_radii = py_robot.attr("link_radii");
        if (!py_radii.is_none()) {
            try {
                auto arr = py_radii.cast<py::array_t<double>>();
                auto buf = arr.request();
                double* ptr = static_cast<double*>(buf.ptr);
                radii.assign(ptr, ptr + buf.shape[0]);
            } catch (...) {
                // try list fallback
                try {
                    radii = py_radii.cast<std::vector<double>>();
                } catch (...) {}
            }
        }

        return Robot(name, dh, jl, tool, radii);
    }, py::arg("py_robot"),
       "Convert a Python Robot object to C++ Robot");
}
