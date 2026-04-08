// pybind11 bindings for SafeBoxForest C++ library
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "sbf/core/types.h"
#include "sbf/core/config.h"
#include "sbf/core/robot.h"
#include "sbf/forest/collision.h"
#include "sbf/forest/hier_aabb_tree.h"
#include "sbf/forest/safe_box_forest.h"
#include "sbf/planner/sbf_planner.h"
#include "sbf/planner/pipeline.h"
#include "sbf/adapters/drake_gcs.h"
#include "sbf/adapters/ompl_adapter.h"
#include "sbf/io/json_io.h"
#include "sbf/io/hcache.h"

namespace py = pybind11;
using namespace sbf;

PYBIND11_MODULE(pysbf, m) {
    m.doc() = "SafeBoxForest (SBF) motion planning — C++ backend";

    // ─── Interval ─────────────────────────────────────────────────────
    py::class_<Interval>(m, "Interval")
        .def(py::init<double, double>())
        .def_readwrite("lo", &Interval::lo)
        .def_readwrite("hi", &Interval::hi)
        .def("width", &Interval::width)
        .def("mid", &Interval::mid)
        .def("__repr__", [](const Interval& iv) {
            return "[" + std::to_string(iv.lo) + ", " + std::to_string(iv.hi) + "]";
        });

    // ─── JointLimits ─────────────────────────────────────────────────
    py::class_<JointLimits>(m, "JointLimits")
        .def(py::init<>())
        .def_readwrite("limits", &JointLimits::limits)
        .def("n_dims", &JointLimits::n_dims)
        .def("contains", &JointLimits::contains, py::arg("q"), py::arg("tol") = 1e-10);

    // ─── Obstacle ─────────────────────────────────────────────────────
    py::class_<Obstacle>(m, "Obstacle")
        .def(py::init<>())
        .def_readwrite("name", &Obstacle::name)
        .def_readwrite("center", &Obstacle::center)
        .def_readwrite("half_sizes", &Obstacle::half_sizes);

    // ─── BoxNode ──────────────────────────────────────────────────────
    py::class_<BoxNode>(m, "BoxNode")
        .def(py::init<int, const std::vector<Interval>&, const Eigen::VectorXd&>())
        .def_readonly("id", &BoxNode::id)
        .def_readonly("joint_intervals", &BoxNode::joint_intervals)
        .def_readonly("seed_config", &BoxNode::seed_config)
        .def_readonly("volume", &BoxNode::volume)
        .def("contains", &BoxNode::contains);

    // ─── PlanningResult ──────────────────────────────────────────────
    py::class_<PlanningResult>(m, "PlanningResult")
        .def(py::init<>())
        .def_readwrite("success", &PlanningResult::success)
        .def_readwrite("path", &PlanningResult::path)
        .def_readwrite("cost", &PlanningResult::cost)
        .def_readwrite("planning_time", &PlanningResult::planning_time)
        .def_readwrite("first_solution_time", &PlanningResult::first_solution_time)
        .def_readwrite("collision_checks", &PlanningResult::collision_checks)
        .def_readwrite("nodes_explored", &PlanningResult::nodes_explored)
        .def_readwrite("phase_times", &PlanningResult::phase_times);

    // ─── SBFConfig ───────────────────────────────────────────────────
    py::class_<SBFConfig>(m, "SBFConfig")
        .def(py::init<>())
        .def_readwrite("max_boxes", &SBFConfig::max_boxes)
        .def_readwrite("ffb_max_depth", &SBFConfig::ffb_max_depth)
        .def_readwrite("ffb_min_edge", &SBFConfig::ffb_min_edge)
        .def_readwrite("ffb_min_edge_anchor", &SBFConfig::ffb_min_edge_anchor)
        .def_readwrite("ffb_min_edge_relaxed", &SBFConfig::ffb_min_edge_relaxed)
        .def_readwrite("guided_sample_ratio", &SBFConfig::guided_sample_ratio)
        .def_readwrite("max_consecutive_miss", &SBFConfig::max_consecutive_miss)
        .def_readwrite("coarsen_max_rounds", &SBFConfig::coarsen_max_rounds)
        .def_readwrite("coarsen_target_boxes", &SBFConfig::coarsen_target_boxes)
        .def_readwrite("coarsen_grid_check", &SBFConfig::coarsen_grid_check)
        .def_readwrite("coarsen_split_depth", &SBFConfig::coarsen_split_depth)
        .def_readwrite("coarsen_max_tree_fk", &SBFConfig::coarsen_max_tree_fk)
        .def_readwrite("shortcut_max_iters", &SBFConfig::shortcut_max_iters)
        .def_readwrite("adjacency_tol", &SBFConfig::adjacency_tol)
        .def_readwrite("segment_resolution", &SBFConfig::segment_resolution)
        .def_readwrite("boundary_expand_epsilon", &SBFConfig::boundary_expand_epsilon)
        .def_readwrite("bfs_phase_k", &SBFConfig::bfs_phase_k)
        .def_readwrite("bfs_phase_budget", &SBFConfig::bfs_phase_budget)
        .def_readwrite("n_edge_samples", &SBFConfig::n_edge_samples)
        .def_readwrite("use_cache", &SBFConfig::use_cache)
        .def_readwrite("cache_path", &SBFConfig::cache_path)
        .def_readwrite("min_boxes_per_pair", &SBFConfig::min_boxes_per_pair)
        .def_readwrite("max_boxes_per_pair", &SBFConfig::max_boxes_per_pair)
        .def_readwrite("proxy_anchor_max_samples", &SBFConfig::proxy_anchor_max_samples)
        .def_readwrite("proxy_anchor_radius", &SBFConfig::proxy_anchor_radius)
        .def_readwrite("seed", &SBFConfig::seed);

    // ─── Robot ───────────────────────────────────────────────────────
    py::class_<Robot>(m, "Robot")
        .def_static("from_json", &Robot::from_json)
        .def("n_joints", &Robot::n_joints)
        .def("n_links", &Robot::n_links)
        .def("joint_limits", &Robot::joint_limits, py::return_value_policy::reference_internal)
        .def("has_ee_spheres", &Robot::has_ee_spheres)
        .def("n_ee_spheres", &Robot::n_ee_spheres)
        .def("ee_spheres_frame", &Robot::ee_spheres_frame);

    // ─── CollisionChecker ────────────────────────────────────────────
    py::class_<CollisionChecker>(m, "CollisionChecker")
        .def(py::init<const Robot&, const std::vector<Obstacle>&>())
        .def("check_config", &CollisionChecker::check_config)
        .def("check_segment", &CollisionChecker::check_segment)
        .def("n_checks", &CollisionChecker::n_checks);

    // ─── SafeBoxForest ───────────────────────────────────────────────
    py::class_<SafeBoxForest>(m, "SafeBoxForest")
        .def(py::init<int, const JointLimits&>())
        .def("n_boxes", &SafeBoxForest::n_boxes)
        .def("n_dims", &SafeBoxForest::n_dims)
        .def("total_volume", &SafeBoxForest::total_volume)
        .def("add_box_no_adjacency", &SafeBoxForest::add_box_no_adjacency)
        .def("rebuild_adjacency", &SafeBoxForest::rebuild_adjacency,
             py::arg("tol") = 1e-8)
        .def("find_containing", &SafeBoxForest::find_containing,
             py::return_value_policy::reference_internal)
        // Interval cache for GCS export
        .def("rebuild_interval_cache", &SafeBoxForest::rebuild_interval_cache)
        .def("intervals_lo", &SafeBoxForest::intervals_lo,
             py::return_value_policy::reference_internal)
        .def("intervals_hi", &SafeBoxForest::intervals_hi,
             py::return_value_policy::reference_internal)
        .def("interval_ids", &SafeBoxForest::interval_ids,
             py::return_value_policy::reference_internal)
        // Adjacency graph
        .def("adjacency", [](const SafeBoxForest& f) {
            // Convert to Python-friendly dict[int, list[int]]
            py::dict result;
            for (auto& [k, v] : f.adjacency()) {
                py::list nbrs;
                for (int n : v) nbrs.append(n);
                result[py::int_(k)] = nbrs;
            }
            return result;
        })
        // Boxes access
        .def("boxes", [](const SafeBoxForest& f) {
            // Convert to Python-friendly dict[int, BoxNode]
            py::dict result;
            for (auto& [k, v] : f.boxes())
                result[py::int_(k)] = v;
            return result;
        })
        // Validate/invalidate
        .def("validate_boxes", &SafeBoxForest::validate_boxes)
        .def("invalidate_against_obstacle",
             &SafeBoxForest::invalidate_against_obstacle,
             py::arg("obs"), py::arg("robot"), py::arg("safety_margin") = 0.0);

    // ─── SBFPlanner ──────────────────────────────────────────────────
    py::class_<SBFPlanner>(m, "SBFPlanner")
        .def(py::init<const Robot&, const std::vector<Obstacle>&, const SBFConfig&>())
        // Build forest only (no planning)
        .def("build", &SBFPlanner::build,
             py::arg("start"), py::arg("goal"), py::arg("timeout") = 30.0)
        // Build multi-pair forest
        .def("build_multi", &SBFPlanner::build_multi,
             py::arg("pairs"), py::arg("n_random_boxes") = 5000,
             py::arg("timeout") = 120.0)
        // Plan (build + query in one call)
        .def("plan", &SBFPlanner::plan,
             py::arg("start"), py::arg("goal"), py::arg("timeout") = 30.0)
        // Query on pre-built forest
        .def("query", &SBFPlanner::query,
             py::arg("start"), py::arg("goal"), py::arg("timeout") = 10.0)
        // Incremental update
        .def("add_obstacle", &SBFPlanner::add_obstacle)
        .def("remove_obstacle", &SBFPlanner::remove_obstacle)
        // Targeted regrowth: fill depleted regions with incremental adjacency
        .def("regrow", &SBFPlanner::regrow,
             py::arg("n_target"), py::arg("timeout") = 60.0)
        // Warm rebuild: clear forest & tree occupation, keep FK cache
        .def("clear_forest", &SBFPlanner::clear_forest)
        // Accessors
        .def("forest", &SBFPlanner::forest,
             py::return_value_policy::reference_internal)
        .def("forest_built", &SBFPlanner::forest_built)
        .def("config", py::overload_cast<>(&SBFPlanner::config, py::const_),
             py::return_value_policy::reference_internal)
        .def("tree", [](SBFPlanner& p) -> HierAABBTree& {
            return const_cast<HierAABBTree&>(p.tree());
        }, py::return_value_policy::reference_internal,
            "Access the HierAABBTree (mutable reference)");

    // ─── Pipeline helpers ────────────────────────────────────────────
    m.def("make_panda_config", &make_panda_config, py::arg("seed") = 0);
    m.def("plan_once", &plan_once,
          py::arg("robot"), py::arg("obstacles"),
          py::arg("start"), py::arg("goal"),
          py::arg("config") = SBFConfig());

    // ─── JSON I/O ────────────────────────────────────────────────────
    m.def("load_obstacles_json", &load_obstacles_json);
    m.def("save_obstacles_json", &save_obstacles_json);
    m.def("save_result_json", &save_result_json);
    m.def("load_result_json", &load_result_json);

    // ─── HierAABBTree ────────────────────────────────────────────────
    py::class_<HierAABBTree>(m, "HierAABBTree")
        .def(py::init<const Robot&, int>(),
             py::arg("robot"), py::arg("initial_cap") = 64)
        .def("find_free_box", &HierAABBTree::find_free_box,
             py::arg("seed"), py::arg("obs_compact"), py::arg("n_obs"),
             py::arg("max_depth") = 200, py::arg("min_edge") = 0.01)
        // Full save (write entire tree to disk)
        .def("save", &HierAABBTree::save, py::arg("path"),
             "Save tree to HCACHE02 binary file (full write)")
        // Incremental save (only dirty + new nodes)
        .def("save_incremental", &HierAABBTree::save_incremental,
             py::arg("path"),
             "Incremental save: only write dirty + new nodes. Requires prior save().")
        // Full load (read all into memory)
        .def_static("load", &HierAABBTree::load,
             py::arg("path"), py::arg("robot"),
             "Load tree from HCACHE02 binary file (full read)")
        // Lazy load via mmap
        .def_static("load_mmap", &HierAABBTree::load_mmap,
             py::arg("path"), py::arg("robot"),
             "Load tree via mmap (lazy: pages loaded on demand). Auto-persists writes.")
        // mmap management
        .def("flush_mmap", &HierAABBTree::flush_mmap,
             "Flush mmap changes to disk (no-op if not mmap-backed)")
        .def("close_mmap", &HierAABBTree::close_mmap,
             "Close mmap mapping (no-op if not mmap-backed)")
        .def("total_fk_calls", &HierAABBTree::total_fk_calls)
        .def("n_nodes", [](const HierAABBTree& t) { return t.store().next_idx(); });

    // ─── HCacheFile (mmap-backed persistence) ────────────────────────
    py::class_<HCacheFile>(m, "HCacheFile")
        .def_static("create", &HCacheFile::create,
             py::arg("path"), py::arg("n_dims"), py::arg("n_links"),
             py::arg("limits"), py::arg("fingerprint") = "",
             py::arg("initial_cap") = 64,
             "Create a new HCACHE02 file with mmap backing")
        .def_static("open", &HCacheFile::open,
             py::arg("path"),
             "Open existing HCACHE02 file with mmap read-write")
        .def("flush", &HCacheFile::flush, "Sync changes to disk")
        .def("close", &HCacheFile::close, "Close and unmap file")
        .def("is_open", &HCacheFile::is_open)
        .def("n_nodes", &HCacheFile::n_nodes)
        .def("n_fk_calls", &HCacheFile::n_fk_calls)
        .def("set_n_fk_calls", &HCacheFile::set_n_fk_calls);

    // ─── Feature flags ──────────────────────────────────────────────
    m.def("drake_available", &drake_available);
#ifdef SBF_WITH_OMPL
    m.def("ompl_available", &ompl_available);
#else
    m.def("ompl_available", []() { return false; });
#endif
}
