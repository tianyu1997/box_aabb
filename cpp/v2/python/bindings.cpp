// ═══════════════════════════════════════════════════════════════════════════
// SafeBoxForest v2 — Comprehensive pybind11 bindings
//
// Exposes all public interfaces from the 10 modules:
//   common, robot, scene, envelope, forest, bridge, planner, viz, io, cache
//
// Module name: pysbf2
// ═══════════════════════════════════════════════════════════════════════════
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

// ── v2 headers (new modules) ────────────────────────────────────────────────
#include "sbf/common/types.h"
#include "sbf/common/config.h"
#include "sbf/robot/robot.h"
#include "sbf/robot/fk.h"
#include "sbf/robot/interval_math.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/scene/i_collision_checker.h"
#include "sbf/scene/aabb_collision_checker.h"
#include "sbf/scene/scene.h"
#include "sbf/envelope/envelope_computer.h"
#include "sbf/forest/root_sampler.h"
#include "sbf/forest/boundary_sampler.h"
#include "sbf/forest/ffb.h"
#include "sbf/forest/forest_grower.h"
#include "sbf/viz/viz_exporter.h"

// ── Parent headers (unchanged modules) ──────────────────────────────────────
#include "sbf/forest/safe_box_forest.h"
#include "sbf/forest/hier_aabb_tree.h"
#include "sbf/forest/coarsen.h"
#include "sbf/forest/connectivity.h"
#include "sbf/planner/sbf_planner.h"
#include "sbf/planner/pipeline.h"
#include "sbf/planner/connector.h"
#include "sbf/planner/graph_search.h"
#include "sbf/planner/path_smoother.h"
#include "sbf/io/json_io.h"
#include "sbf/io/hcache.h"
#include "sbf/adapters/drake_gcs.h"

namespace py = pybind11;
using namespace pybind11::literals;
using namespace sbf;

// ─────────────────────────────────────────────────────────────────────────────
// Helper: convert unordered_map<int, vector<int>> → Python dict
// ─────────────────────────────────────────────────────────────────────────────
static py::dict adj_to_dict(const std::unordered_map<int, std::vector<int>>& adj) {
    py::dict d;
    for (auto& [k, v] : adj) {
        py::list nbrs;
        for (int n : v) nbrs.append(n);
        d[py::int_(k)] = nbrs;
    }
    return d;
}

// ═══════════════════════════════════════════════════════════════════════════
//  MODULE DEFINITION
// ═══════════════════════════════════════════════════════════════════════════

PYBIND11_MODULE(pysbf2, m) {
    m.doc() = R"doc(
SafeBoxForest v2 (pysbf2) — Motion planning via safe boxes in C-space.

Modules
-------
- **common**: Core types (Interval, BoxNode, Obstacle, …) and configs
- **robot**: Robot kinematic model, FK, interval FK
- **scene**: Collision checking and obstacle scene management
- **envelope**: AABB envelope computation from interval FK
- **forest**: Forest growth (RootSampler, FFBEngine, ForestGrower, SafeBoxForest)
- **bridge**: Coarsening and island connectivity
- **planner**: SBFPlanner end-to-end, graph search, path smoothing
- **viz**: JSON / CSV export for visualization
- **io**: Robot / obstacle / result JSON I/O, HCache persistence
)doc";

    // =====================================================================
    //  1. COMMON — types
    // =====================================================================

    // ── Interval ─────────────────────────────────────────────────────────
    py::class_<Interval>(m, "Interval", R"doc(
A closed interval [lo, hi] on the real line.
Used to represent joint ranges and box extents per dimension.
)doc")
        .def(py::init<>(), "Default interval [0, 0].")
        .def(py::init<double, double>(), "lo"_a, "hi"_a,
             "Construct interval with bounds.")
        .def_readwrite("lo", &Interval::lo, "Lower bound.")
        .def_readwrite("hi", &Interval::hi, "Upper bound.")
        .def("width", &Interval::width, "hi - lo")
        .def("mid", &Interval::mid, "(lo + hi) / 2")
        .def("center", &Interval::center, "Alias for mid().")
        .def("contains", &Interval::contains, "v"_a, "tol"_a = CONTAIN_TOL,
             "Check if v is within [lo-tol, hi+tol].")
        .def("overlaps", &Interval::overlaps, "other"_a, "tol"_a = 0.0,
             "Check if two intervals overlap.")
        .def("hull", &Interval::hull, "other"_a,
             "Smallest interval containing both.")
        .def("intersect", &Interval::intersect, "other"_a,
             "Intersection of two intervals.")
        .def("__repr__", [](const Interval& iv) {
            return "[" + std::to_string(iv.lo) + ", " + std::to_string(iv.hi) + "]";
        })
        .def(py::self + py::self)
        .def(py::self - py::self)
        .def(py::self * py::self)
        .def(py::self * double());

    // ── Obstacle ─────────────────────────────────────────────────────────
    py::class_<Obstacle>(m, "Obstacle", R"doc(
Axis-aligned box obstacle in workspace (3D).

Attributes
----------
center : numpy.ndarray (3,)
    Center position [x, y, z].
half_sizes : numpy.ndarray (3,)
    Half extents along each axis.
name : str
    Optional display name.
)doc")
        .def(py::init<>())
        .def(py::init<Eigen::Vector3d, Eigen::Vector3d, std::string>(),
             "center"_a, "half_sizes"_a, "name"_a = "",
             "Create obstacle from center, half-sizes, and optional name.")
        .def_readwrite("center", &Obstacle::center)
        .def_readwrite("half_sizes", &Obstacle::half_sizes)
        .def_readwrite("name", &Obstacle::name)
        .def("lo", &Obstacle::lo, "Lower corner (center - half_sizes).")
        .def("hi", &Obstacle::hi, "Upper corner (center + half_sizes).")
        .def("__repr__", [](const Obstacle& o) {
            return "Obstacle('" + o.name + "', center=[" +
                   std::to_string(o.center.x()) + "," +
                   std::to_string(o.center.y()) + "," +
                   std::to_string(o.center.z()) + "])";
        });

    // ── JointLimits ──────────────────────────────────────────────────────
    py::class_<JointLimits>(m, "JointLimits", R"doc(
Collection of per-joint Interval bounds defining the valid C-space.
)doc")
        .def(py::init<>())
        .def_readwrite("limits", &JointLimits::limits,
                       "List of Interval bounds per joint.")
        .def("n_dims", &JointLimits::n_dims, "Number of joints.")
        .def("contains", &JointLimits::contains, "q"_a, "tol"_a = 1e-10,
             "Check if configuration q is within all limits.")
        .def("clamp", &JointLimits::clamp, "q"_a,
             "Clamp q to joint limits.");

    // ── BoxNode ──────────────────────────────────────────────────────────
    py::class_<BoxNode>(m, "BoxNode", R"doc(
An axis-aligned box in C-space, identified by integer id.

Each box stores its joint intervals, a seed configuration,
and connectivity information (parent_id, children_ids, tree_id).
)doc")
        .def(py::init<>())
        .def(py::init<int, const std::vector<Interval>&, const Eigen::VectorXd&>(),
             "id"_a, "intervals"_a, "seed_config"_a)
        .def_readonly("id", &BoxNode::id)
        .def_readonly("joint_intervals", &BoxNode::joint_intervals)
        .def_readonly("seed_config", &BoxNode::seed_config)
        .def_readonly("volume", &BoxNode::volume)
        .def_readwrite("parent_id", &BoxNode::parent_id)
        .def_readwrite("tree_id", &BoxNode::tree_id)
        .def_readwrite("children_ids", &BoxNode::children_ids)
        .def("n_dims", &BoxNode::n_dims)
        .def("center", &BoxNode::center,
             "Center configuration of the box.")
        .def("compute_volume", &BoxNode::compute_volume)
        .def("contains", &BoxNode::contains, "q"_a, "tol"_a = CONTAIN_TOL,
             "Check if configuration q lies inside this box.")
        .def("distance_to_config", &BoxNode::distance_to_config, "q"_a,
             "L2 distance from q to nearest point in box.")
        .def("nearest_point_to", &BoxNode::nearest_point_to, "q"_a,
             "Project q onto box boundary (or return q if inside).")
        .def("overlaps_with", &BoxNode::overlaps_with, "other"_a, "tol"_a = 0.0,
             "Check if two boxes overlap (coordinate-wise).")
        .def("is_adjacent_to", &BoxNode::is_adjacent_to, "other"_a, "tol"_a = 1e-10,
             "Check face-adjacency (touch on at least one dimension).")
        .def("shared_face_center", &BoxNode::shared_face_center, "other"_a,
             "Compute the center of the shared face with another box.")
        .def("__repr__", [](const BoxNode& b) {
            return "BoxNode(id=" + std::to_string(b.id) +
                   ", dims=" + std::to_string(b.n_dims()) +
                   ", vol=" + std::to_string(b.volume) + ")";
        });

    // ── Edge ─────────────────────────────────────────────────────────────
    py::class_<Edge>(m, "Edge", "Weighted edge between two box IDs.")
        .def(py::init<>())
        .def(py::init<int, int, double>(), "from_id"_a, "to_id"_a, "weight"_a = 1.0)
        .def_readwrite("from_id", &Edge::from_id)
        .def_readwrite("to_id", &Edge::to_id)
        .def_readwrite("weight", &Edge::weight);

    // ── PlanningResult ───────────────────────────────────────────────────
    py::class_<PlanningResult>(m, "PlanningResult", R"doc(
Result of a motion planning query.

Attributes
----------
success : bool
    Whether a valid path was found.
path : numpy.ndarray (N, n_joints)
    Waypoint matrix. Empty if planning failed.
cost : float
    Total path length (L2 in C-space).
planning_time : float
    Wall-clock time for the entire plan() call (seconds).
first_solution_time : float
    Time to find the first feasible solution.
collision_checks : int
    Number of collision checks performed.
nodes_explored : int
    Number of graph nodes expanded.
phase_times : dict
    Per-phase timing breakdown.
)doc")
        .def(py::init<>())
        .def_readwrite("success", &PlanningResult::success)
        .def_readwrite("path", &PlanningResult::path)
        .def_readwrite("cost", &PlanningResult::cost)
        .def_readwrite("planning_time", &PlanningResult::planning_time)
        .def_readwrite("first_solution_time", &PlanningResult::first_solution_time)
        .def_readwrite("collision_checks", &PlanningResult::collision_checks)
        .def_readwrite("nodes_explored", &PlanningResult::nodes_explored)
        .def_readwrite("phase_times", &PlanningResult::phase_times)
        .def_readwrite("metadata", &PlanningResult::metadata)
        .def("n_waypoints", &PlanningResult::n_waypoints)
        .def_static("failure", &PlanningResult::failure, "time"_a,
                     "Create a failure result with given timing.")
        .def("__repr__", [](const PlanningResult& r) {
            return std::string("PlanningResult(success=") +
                   (r.success ? "True" : "False") +
                   ", cost=" + std::to_string(r.cost) +
                   ", waypoints=" + std::to_string(r.n_waypoints()) + ")";
        });

    // ── FFBResult ────────────────────────────────────────────────────────
    py::class_<FFBResult>(m, "FFBResult", R"doc(
Result of the Find-Free-Box (FFB) algorithm.

Attributes
----------
node_idx : int
    Index of the resulting node in the HierAABBTree (-1 on failure).
path : list of int
    Bisection path from root to the result node.
fail_code : int
    0 = success, nonzero = failure reason.
n_new_nodes : int
    Number of new nodes created during search.
n_fk_calls : int
    FK evaluations used.
)doc")
        .def(py::init<>())
        .def_readwrite("node_idx", &FFBResult::node_idx)
        .def_readwrite("path", &FFBResult::path)
        .def_readwrite("fail_code", &FFBResult::fail_code)
        .def_readwrite("n_new_nodes", &FFBResult::n_new_nodes)
        .def_readwrite("n_fk_calls", &FFBResult::n_fk_calls)
        .def("success", &FFBResult::success);

    // =====================================================================
    //  2. COMMON — config
    // =====================================================================

    // ── EnvelopeConfig ───────────────────────────────────────────────────
    using EC = sbf::envelope::EnvelopeConfig;
    py::class_<EC>(m, "EnvelopeConfig", R"doc(
Parameters controlling the AABB envelope computation.

Attributes
----------
max_depth : int
    Maximum bisection depth (default 1000).
min_edge : float
    Minimum interval width for bisection (default 0.01).
min_edge_anchor : float
    Minimum edge for anchor boxes (default 0.001).
min_edge_relaxed : float
    Relaxed minimum for later phases (default 0.05).
promotion_depth : int
    Parent walk depth for promotion (default 2).
)doc")
        .def(py::init<>())
        .def_readwrite("max_depth", &EC::max_depth)
        .def_readwrite("min_edge", &EC::min_edge)
        .def_readwrite("min_edge_anchor", &EC::min_edge_anchor)
        .def_readwrite("min_edge_relaxed", &EC::min_edge_relaxed)
        .def_readwrite("promotion_depth", &EC::promotion_depth);

    // ── ForestConfig ─────────────────────────────────────────────────────
    using FC = sbf::forest::ForestConfig;
    py::class_<FC>(m, "ForestConfig", R"doc(
Parameters for forest growth (number of boxes, sampling, etc.).
)doc")
        .def(py::init<>())
        .def_readwrite("max_boxes", &FC::max_boxes)
        .def_readwrite("max_consecutive_miss", &FC::max_consecutive_miss)
        .def_readwrite("bfs_phase_k", &FC::bfs_phase_k)
        .def_readwrite("bfs_phase_budget", &FC::bfs_phase_budget)
        .def_readwrite("min_boxes_per_pair", &FC::min_boxes_per_pair)
        .def_readwrite("max_boxes_per_pair", &FC::max_boxes_per_pair)
        .def_readwrite("guided_sample_ratio", &FC::guided_sample_ratio)
        .def_readwrite("boundary_expand_epsilon", &FC::boundary_expand_epsilon)
        .def_readwrite("n_edge_samples", &FC::n_edge_samples)
        .def_readwrite("adjacency_tol", &FC::adjacency_tol)
        .def_readwrite("proxy_anchor_max_samples", &FC::proxy_anchor_max_samples)
        .def_readwrite("proxy_anchor_radius", &FC::proxy_anchor_radius);

    // ── BridgeConfig ─────────────────────────────────────────────────────
    using BC = sbf::bridge::BridgeConfig;
    py::class_<BC>(m, "BridgeConfig", R"doc(
Parameters for coarsening and island bridging.
)doc")
        .def(py::init<>())
        .def_readwrite("coarsen_max_rounds", &BC::coarsen_max_rounds)
        .def_readwrite("coarsen_target_boxes", &BC::coarsen_target_boxes)
        .def_readwrite("coarsen_greedy_rounds", &BC::coarsen_greedy_rounds)
        .def_readwrite("coarsen_grid_check", &BC::coarsen_grid_check)
        .def_readwrite("coarsen_split_depth", &BC::coarsen_split_depth)
        .def_readwrite("coarsen_max_tree_fk", &BC::coarsen_max_tree_fk)
        .def_readwrite("corridor_hops", &BC::corridor_hops)
        .def_readwrite("use_gcs", &BC::use_gcs);

    // ── PlannerConfig ────────────────────────────────────────────────────
    using PC = sbf::planner::PlannerConfig;
    py::class_<PC>(m, "PlannerConfig", R"doc(
Parameters for the path search and smoothing phase.
)doc")
        .def(py::init<>())
        .def_readwrite("shortcut_max_iters", &PC::shortcut_max_iters)
        .def_readwrite("segment_resolution", &PC::segment_resolution)
        .def_readwrite("parallel_grow", &PC::parallel_grow)
        .def_readwrite("n_partitions_depth", &PC::n_partitions_depth)
        .def_readwrite("parallel_workers", &PC::parallel_workers)
        .def_readwrite("seed", &PC::seed);

    // ── SBFConfig (aggregate) ────────────────────────────────────────────
    py::class_<SBFConfig>(m, "SBFConfig", R"doc(
Aggregate configuration for the full SBF pipeline.

Contains all tunable parameters merged from EnvelopeConfig,
ForestConfig, BridgeConfig, and PlannerConfig. Use the decompose
methods (envelope_config(), forest_config(), etc.) to extract
sub-configs for individual modules.

Example
-------
>>> cfg = pysbf2.SBFConfig()
>>> cfg.max_boxes = 1000
>>> cfg.shortcut_max_iters = 200
>>> cfg.seed = 42
)doc")
        .def(py::init<>())
        // FFB / envelope
        .def_readwrite("ffb_max_depth", &SBFConfig::ffb_max_depth)
        .def_readwrite("ffb_min_edge", &SBFConfig::ffb_min_edge)
        .def_readwrite("ffb_min_edge_anchor", &SBFConfig::ffb_min_edge_anchor)
        .def_readwrite("ffb_min_edge_relaxed", &SBFConfig::ffb_min_edge_relaxed)
        // Forest
        .def_readwrite("max_boxes", &SBFConfig::max_boxes)
        .def_readwrite("guided_sample_ratio", &SBFConfig::guided_sample_ratio)
        .def_readwrite("max_consecutive_miss", &SBFConfig::max_consecutive_miss)
        .def_readwrite("bfs_phase_k", &SBFConfig::bfs_phase_k)
        .def_readwrite("bfs_phase_budget", &SBFConfig::bfs_phase_budget)
        .def_readwrite("min_boxes_per_pair", &SBFConfig::min_boxes_per_pair)
        .def_readwrite("max_boxes_per_pair", &SBFConfig::max_boxes_per_pair)
        .def_readwrite("n_edge_samples", &SBFConfig::n_edge_samples)
        .def_readwrite("adjacency_tol", &SBFConfig::adjacency_tol)
        .def_readwrite("boundary_expand_epsilon", &SBFConfig::boundary_expand_epsilon)
        .def_readwrite("proxy_anchor_max_samples", &SBFConfig::proxy_anchor_max_samples)
        .def_readwrite("proxy_anchor_radius", &SBFConfig::proxy_anchor_radius)
        // Bridge / coarsen
        .def_readwrite("coarsen_max_rounds", &SBFConfig::coarsen_max_rounds)
        .def_readwrite("coarsen_target_boxes", &SBFConfig::coarsen_target_boxes)
        .def_readwrite("coarsen_grid_check", &SBFConfig::coarsen_grid_check)
        .def_readwrite("coarsen_split_depth", &SBFConfig::coarsen_split_depth)
        .def_readwrite("coarsen_max_tree_fk", &SBFConfig::coarsen_max_tree_fk)
        // Planner
        .def_readwrite("shortcut_max_iters", &SBFConfig::shortcut_max_iters)
        .def_readwrite("segment_resolution", &SBFConfig::segment_resolution)
        // Cache
        .def_readwrite("use_cache", &SBFConfig::use_cache)
        .def_readwrite("cache_path", &SBFConfig::cache_path)
        // Seed
        .def_readwrite("seed", &SBFConfig::seed)
        // Decompose
        .def("envelope_config", &SBFConfig::envelope_config,
             "Extract EnvelopeConfig sub-config.")
        .def("forest_config", &SBFConfig::forest_config,
             "Extract ForestConfig sub-config.")
        .def("bridge_config", &SBFConfig::bridge_config,
             "Extract BridgeConfig sub-config.")
        .def("planner_config", &SBFConfig::planner_config,
             "Extract PlannerConfig sub-config.");

    // =====================================================================
    //  3. ROBOT
    // =====================================================================

    // ── DHParam ──────────────────────────────────────────────────────────
    py::class_<DHParam>(m, "DHParam", R"doc(
Denavit-Hartenberg parameter set for one link.

Attributes
----------
alpha : float
    Link twist (radians).
a : float
    Link length.
d : float
    Link offset.
theta : float
    Joint angle offset.
joint_type : int
    0 = revolute, 1 = prismatic.
)doc")
        .def(py::init<>())
        .def_readwrite("alpha", &DHParam::alpha)
        .def_readwrite("a", &DHParam::a)
        .def_readwrite("d", &DHParam::d)
        .def_readwrite("theta", &DHParam::theta)
        .def_readwrite("joint_type", &DHParam::joint_type);

    // ── EESphere ─────────────────────────────────────────────────────────
    py::class_<EESphere>(m, "EESphere",
                         "End-effector bounding sphere (center + radius).")
        .def(py::init<>())
        .def_property("center",
            [](const EESphere& s) {
                return Eigen::Vector3d(s.center[0], s.center[1], s.center[2]);
            },
            [](EESphere& s, const Eigen::Vector3d& c) {
                s.center[0] = c.x(); s.center[1] = c.y(); s.center[2] = c.z();
            })
        .def_readwrite("radius", &EESphere::radius);

    // ── Robot ────────────────────────────────────────────────────────────
    py::class_<Robot>(m, "Robot", R"doc(
Robot kinematic model based on DH convention.

Construct from DH parameters + joint limits, or load from a JSON
config file via ``Robot.from_json(path)``.

Example
-------
>>> robot = pysbf2.Robot.from_json("configs/panda.json")
>>> print(robot.n_joints())   # 7
>>> q = np.zeros(7)
>>> positions = robot.fk_link_positions(q)
)doc")
        .def(py::init<>())
        .def_static("from_json", &Robot::from_json, "path"_a,
                     "Load robot from a JSON config file.")
        .def("name", &Robot::name, "Robot name string.")
        .def("n_joints", &Robot::n_joints, "Number of actuated joints.")
        .def("n_links", &Robot::n_links, "Number of links (including base).")
        .def("n_transforms", &Robot::n_transforms,
             "Number of homogeneous transforms.")
        .def("joint_limits", &Robot::joint_limits,
             py::return_value_policy::reference_internal,
             "Joint limits as JointLimits reference.")
        .def("dh_params", &Robot::dh_params,
             py::return_value_policy::reference_internal,
             "DH parameter list.")
        .def("has_tool", &Robot::has_tool)
        .def("link_radii", &Robot::link_radii,
             py::return_value_policy::reference_internal)
        .def("has_ee_spheres", &Robot::has_ee_spheres)
        .def("n_ee_spheres", &Robot::n_ee_spheres)
        .def("ee_spheres", &Robot::ee_spheres,
             py::return_value_policy::reference_internal)
        .def("ee_spheres_frame", &Robot::ee_spheres_frame)
        .def("has_ee_groups", &Robot::has_ee_groups)
        .def("n_ee_groups", &Robot::n_ee_groups)
        .def("fingerprint", &Robot::fingerprint,
             "Hash string encoding robot geometry.")
        .def("fk_link_positions", &Robot::fk_link_positions, "q"_a,
             "Compute link positions via FK. Returns (n_links, 3) matrix.")
        .def("fk_transforms", &Robot::fk_transforms, "q"_a,
             "Compute full 4×4 transforms for all links.")
        .def("__repr__", [](const Robot& r) {
            return "Robot('" + r.name() + "', joints=" +
                   std::to_string(r.n_joints()) + ")";
        });

    // ── FK free functions ────────────────────────────────────────────────
    m.def("fk_link_positions", &fk_link_positions, "robot"_a, "q"_a,
          "Forward kinematics: returns list of link positions.");
    m.def("fk_transforms", &fk_transforms, "robot"_a, "q"_a,
          "Forward kinematics: returns list of 4×4 transforms.");
    m.def("dh_transform", &dh_transform,
          "alpha"_a, "a"_a, "d"_a, "theta"_a,
          "Single DH homogeneous transform matrix.");

    // ── Interval arithmetic ──────────────────────────────────────────────
    m.def("I_sin", &I_sin, "lo"_a, "hi"_a,
          "Interval sine: guaranteed enclosure of sin over [lo, hi].");
    m.def("I_cos", &I_cos, "lo"_a, "hi"_a,
          "Interval cosine: guaranteed enclosure of cos over [lo, hi].");

    // ── Interval FK ──────────────────────────────────────────────────────
    py::class_<FKState>(m, "FKState", R"doc(
Internal state of interval FK computation.
Stores prefix transform intervals for incremental updates.
)doc")
        .def(py::init<>())
        .def_readonly("n_tf", &FKState::n_tf)
        .def_readonly("n_jm", &FKState::n_jm)
        .def_readonly("valid", &FKState::valid);

    m.def("compute_fk_full", &compute_fk_full, "robot"_a, "intervals"_a,
          "Full interval FK: returns FKState with all transform intervals.");
    m.def("compute_fk_incremental", &compute_fk_incremental,
          "parent"_a, "robot"_a, "intervals"_a, "changed_dim"_a, R"doc(
Incremental interval FK after splitting one dimension.
Reuses parent transforms for unchanged dimensions.
)doc");

    // =====================================================================
    //  4. SCENE
    // =====================================================================

    // ── ICollisionChecker (abstract interface) ───────────────────────────
    py::class_<ICollisionChecker>(m, "ICollisionChecker", R"doc(
Abstract collision checking interface.
Use AabbCollisionChecker for the concrete implementation.
)doc")
        .def("check_config", &ICollisionChecker::check_config, "q"_a,
             "Returns True if config q is collision-free.")
        .def("check_box", &ICollisionChecker::check_box, "intervals"_a,
             "Returns True if the entire box is collision-free.")
        .def("check_segment", &ICollisionChecker::check_segment,
             "q1"_a, "q2"_a, "step"_a = 0.05,
             "Returns True if the linear segment is collision-free.")
        .def("n_checks", &ICollisionChecker::n_checks,
             "Total collision checks performed.")
        .def("reset_counter", &ICollisionChecker::reset_counter);

    // ── AabbCollisionChecker ─────────────────────────────────────────────
    py::class_<AabbCollisionChecker, ICollisionChecker>(m,
        "AabbCollisionChecker", R"doc(
AABB-based collision checker using interval FK.

Constructs bounding boxes from the robot's link geometry and
checks overlap with workspace obstacles using the SAT algorithm.

Example
-------
>>> checker = pysbf2.AabbCollisionChecker(robot, obstacles)
>>> is_free = checker.check_config(q)
)doc")
        .def(py::init<>())
        .def(py::init<const Robot&, const std::vector<Obstacle>&>(),
             "robot"_a, "obstacles"_a)
        .def("check_config", &AabbCollisionChecker::check_config, "q"_a)
        .def("check_box", &AabbCollisionChecker::check_box, "intervals"_a)
        .def("check_segment", &AabbCollisionChecker::check_segment,
             "q1"_a, "q2"_a, "step"_a = 0.05)
        .def("n_checks", &AabbCollisionChecker::n_checks)
        .def("reset_counter", &AabbCollisionChecker::reset_counter)
        .def("n_obs", &AabbCollisionChecker::n_obs,
             "Number of obstacles.")
        .def("n_aabb_slots", &AabbCollisionChecker::n_aabb_slots);

    // ── CollisionChecker alias ───────────────────────────────────────────
    // CollisionChecker is just AabbCollisionChecker; no separate binding needed

    // ── Scene ────────────────────────────────────────────────────────────
    py::class_<Scene>(m, "Scene", R"doc(
Obstacle collection with efficient packed representation.

Use Scene to manage obstacles dynamically (add/remove/clear)
and get a compact float array for collision checking.

Example
-------
>>> scene = pysbf2.Scene()
>>> scene.add_obstacle(pysbf2.Obstacle(
...     np.array([0.5, 0.0, 0.5]),
...     np.array([0.1, 0.1, 0.1]),
...     "table"))
>>> print(scene.n_obstacles())
)doc")
        .def(py::init<>())
        .def(py::init<std::vector<Obstacle>>(), "obstacles"_a)
        .def("add_obstacle", &Scene::add_obstacle, "obs"_a)
        .def("remove_obstacle", &Scene::remove_obstacle, "name"_a)
        .def("clear", &Scene::clear)
        .def("obstacles", &Scene::obstacles,
             py::return_value_policy::reference_internal)
        .def("n_obstacles", &Scene::n_obstacles)
        .def("repack", &Scene::repack,
             "Rebuild the compact obstacle array after modifications.");

    // =====================================================================
    //  5. ENVELOPE
    // =====================================================================
    //
    // NOTE: EnvelopeResult and IntervalFKEnvelopeComputer are LEGACY APIs
    // kept for backward compatibility.  New code should use:
    //   - FrameStore + derive_aabb/derive_ifk_obb/derive_grid in C++
    //   - check_collision() with CollisionPolicy dispatch
    //

    // ── EnvelopeResult (LEGACY) ──────────────────────────────────────────
    py::class_<EnvelopeResult>(m, "EnvelopeResult", R"doc(
[DEPRECATED] Result of a legacy AABB envelope computation.

New code should use FrameStore + collision_policy.h in C++.
Contains link AABBs, end-effector AABBs, and the FK state
for incremental updates.
)doc")
        .def(py::init<>())
        .def_readonly("n_link_slots", &EnvelopeResult::n_link_slots)
        .def_readonly("n_ee_slots", &EnvelopeResult::n_ee_slots)
        .def_readonly("valid", &EnvelopeResult::valid)
        .def_readonly("fk_state", &EnvelopeResult::fk_state);

    // ── IntervalFKEnvelopeComputer (LEGACY) ─────────────────────────────
    py::class_<IntervalFKEnvelopeComputer>(m, "IntervalFKEnvelopeComputer",
        R"doc(
[DEPRECATED] Envelope computer using interval forward kinematics.

New code should use FrameStore + check_collision() in C++.
Computes tight AABB enclosures for each link given joint intervals,
supporting both full computation and incremental updates.

Example
-------
>>> env_comp = pysbf2.IntervalFKEnvelopeComputer(robot)
>>> intervals = [pysbf2.Interval(-0.5, 0.5) for _ in range(7)]
>>> result = env_comp.compute_envelope(intervals)
>>> print(result.valid)
)doc")
        .def(py::init<>())
        .def(py::init<const Robot&>(), "robot"_a)
        .def("compute_envelope", &IntervalFKEnvelopeComputer::compute_envelope,
             "intervals"_a,
             "Full envelope from joint intervals → EnvelopeResult.")
        .def("compute_envelope_incremental",
             &IntervalFKEnvelopeComputer::compute_envelope_incremental,
             "parent_fk"_a, "intervals"_a, "changed_dim"_a,
             "Incremental update after splitting one dimension.")
        .def("n_total_aabb_slots",
             &IntervalFKEnvelopeComputer::n_total_aabb_slots)
        .def("robot", &IntervalFKEnvelopeComputer::robot,
             py::return_value_policy::reference_internal);

    // =====================================================================
    //  6. FOREST
    // =====================================================================

    // ── SafeBoxForest ────────────────────────────────────────────────────
    py::class_<SafeBoxForest>(m, "SafeBoxForest", R"doc(
Core data structure: a collection of axis-aligned safe boxes
in C-space with an adjacency graph.

The forest stores collision-free boxes, tracks adjacency between
touching boxes, and supports queries like find_containing() and
find_nearest().

Example
-------
>>> forest = pysbf2.SafeBoxForest(7, robot.joint_limits())
>>> forest.add_box_no_adjacency(box)
>>> forest.rebuild_adjacency(tol=1e-8)
>>> containing_box = forest.find_containing(q)
)doc")
        .def(py::init<>())
        .def(py::init<int, const JointLimits&>(), "n_dims"_a, "limits"_a)
        // Box management
        .def("add_box_direct", &SafeBoxForest::add_box_direct, "box"_a,
             "Add box and update adjacency immediately.")
        .def("add_box_no_adjacency", &SafeBoxForest::add_box_no_adjacency,
             "box"_a, "Add box without adjacency update (batch mode).")
        .def("clear", &SafeBoxForest::clear)
        .def("remove_boxes", &SafeBoxForest::remove_boxes, "box_ids"_a)
        .def("remove_boxes_no_adjacency",
             &SafeBoxForest::remove_boxes_no_adjacency, "box_ids"_a)
        // Adjacency
        .def("rebuild_adjacency", &SafeBoxForest::rebuild_adjacency,
             "tol"_a = 1e-10)
        .def("adjacency", [](const SafeBoxForest& f) { return adj_to_dict(f.adjacency()); },
             "Adjacency graph as dict[int, list[int]].")
        // Queries
        .def("find_containing", &SafeBoxForest::find_containing, "q"_a,
             py::return_value_policy::reference_internal,
             "Find box containing config q, or None.")
        .def("find_nearest", &SafeBoxForest::find_nearest, "q"_a,
             py::return_value_policy::reference_internal,
             "Find nearest box to config q.")
        // Validation
        .def("validate_boxes", &SafeBoxForest::validate_boxes, "checker"_a,
             "Remove invalid boxes. Returns set of removed IDs.")
        .def("invalidate_against_obstacle",
             &SafeBoxForest::invalidate_against_obstacle,
             "obs"_a, "robot"_a, "safety_margin"_a = 0.0,
             "Remove boxes colliding with new obstacle.")
        .def("validate_invariants", &SafeBoxForest::validate_invariants,
             "tol"_a = 1e-8)
        // Properties
        .def("n_boxes", &SafeBoxForest::n_boxes)
        .def("n_dims", &SafeBoxForest::n_dims)
        .def("total_volume", &SafeBoxForest::total_volume)
        .def("joint_limits", &SafeBoxForest::joint_limits,
             py::return_value_policy::reference_internal)
        .def("boxes", [](const SafeBoxForest& f) {
            py::dict result;
            for (auto& [k, v] : f.boxes())
                result[py::int_(k)] = v;
            return result;
        }, "All boxes as dict[int, BoxNode].")
        // Interval cache for GCS
        .def("rebuild_interval_cache", &SafeBoxForest::rebuild_interval_cache)
        .def("intervals_lo", &SafeBoxForest::intervals_lo,
             py::return_value_policy::reference_internal)
        .def("intervals_hi", &SafeBoxForest::intervals_hi,
             py::return_value_policy::reference_internal)
        .def("interval_ids", &SafeBoxForest::interval_ids,
             py::return_value_policy::reference_internal)
        .def("allocate_id", &SafeBoxForest::allocate_id,
             "Allocate a new unique box ID.");

    // ── HierAABBTree ─────────────────────────────────────────────────────
    py::class_<HierAABBTree>(m, "HierAABBTree", R"doc(
Hierarchical AABB tree for the Find-Free-Box algorithm.

The tree subdivides C-space into a binary tree where each node
has pre-computed AABB envelopes. FFB walks the tree to find a
leaf node whose envelope does not collide with obstacles.

Supports persistence via save/load (binary) and mmap.

Example
-------
>>> tree = pysbf2.HierAABBTree(robot, initial_cap=128)
>>> result = tree.find_free_box(seed_q, obs_compact, n_obs)
>>> tree.save("cache.hcache")
)doc")
        .def(py::init<>())
        .def(py::init<const Robot&, int>(), "robot"_a, "initial_cap"_a = 64)
        .def("find_free_box", &HierAABBTree::find_free_box,
             "seed"_a, "obs_compact"_a, "n_obs"_a,
             "max_depth"_a = 200, "min_edge"_a = 0.01,
             "Run FFB from seed config → FFBResult.")
        .def("mark_occupied", &HierAABBTree::mark_occupied,
             "node_idx"_a, "forest_box_id"_a)
        .def("unmark_occupied", &HierAABBTree::unmark_occupied,
             "node_idx"_a)
        .def("clear_all_occupation", &HierAABBTree::clear_all_occupation)
        .def("get_node_intervals", &HierAABBTree::get_node_intervals,
             "node_idx"_a, "Get joint intervals at node.")
        // Persistence
        .def("save", &HierAABBTree::save, "path"_a,
             "Save tree to HCACHE02 binary file.")
        .def("save_incremental", &HierAABBTree::save_incremental, "path"_a,
             "Incremental save (only dirty nodes).")
        .def_static("load", &HierAABBTree::load, "path"_a, "robot"_a,
                     "Load tree from HCACHE02 binary file.")
        .def_static("load_mmap", &HierAABBTree::load_mmap, "path"_a, "robot"_a,
                     "Load tree via memory-mapped file.")
        .def("flush_mmap", &HierAABBTree::flush_mmap)
        .def("close_mmap", &HierAABBTree::close_mmap)
        // Stats
        .def("total_fk_calls", &HierAABBTree::total_fk_calls)
        .def("n_nodes", [](const HierAABBTree& t) { return t.store().next_idx(); },
             "Total number of tree nodes.");

    // ── RootSampler ──────────────────────────────────────────────────────
    py::class_<sbf::forest::RootSampler>(m, "RootSampler", R"doc(
Uniform and goal-biased sampler for root seeds.

Used to generate starting configurations for FFB queries.
Supports reproducible sampling with settable seed.

Example
-------
>>> sampler = pysbf2.RootSampler(robot.joint_limits(), seed=42)
>>> q = sampler.sample_uniform()
>>> q_biased = sampler.sample_guided(goal, guided_ratio=0.6)
)doc")
        .def(py::init<>())
        .def(py::init<const JointLimits&, unsigned>(),
             "limits"_a, "seed"_a = 0)
        .def("sample_uniform", &sbf::forest::RootSampler::sample_uniform,
             "Sample uniformly from joint limits.")
        .def("sample_guided", &sbf::forest::RootSampler::sample_guided,
             "goal"_a, "guided_ratio"_a = 0.6,
             "Sample with probability guided_ratio towards goal.")
        .def("set_seed", &sbf::forest::RootSampler::set_seed, "seed"_a);

    // ── BoundarySeed ─────────────────────────────────────────────────────
    py::class_<sbf::forest::BoundarySeed>(m, "BoundarySeed", R"doc(
A seed on a box's boundary face.

Attributes
----------
dim : int
    Joint dimension index.
side : int
    0 = lower face, 1 = upper face.
config : numpy.ndarray
    Configuration vector on the boundary.
)doc")
        .def(py::init<>())
        .def_readwrite("dim", &sbf::forest::BoundarySeed::dim)
        .def_readwrite("side", &sbf::forest::BoundarySeed::side)
        .def_readwrite("config", &sbf::forest::BoundarySeed::config);

    // ── BoundarySampler ──────────────────────────────────────────────────
    py::class_<sbf::forest::BoundarySampler>(m, "BoundarySampler", R"doc(
Generates boundary seeds for directed box expansion.

Given a box, produces seed configurations on its faces
(optionally biased towards a goal), for growing adjacent boxes.
)doc")
        .def(py::init<>())
        .def(py::init<const JointLimits&, double>(),
             "limits"_a, "epsilon"_a = 0.01);

    // ── FFBEngine ────────────────────────────────────────────────────────
    py::class_<sbf::forest::FFBEngine>(m, "FFBEngine", R"doc(
Wrapper around HierAABBTree.find_free_box with phase management.

Manages multi-phase box growth where the minimum edge constraint
is progressively relaxed for later phases.

Example
-------
>>> engine = pysbf2.FFBEngine(tree, forest_config)
>>> result = engine.find_free_box(seed, obs_compact, n_obs)
>>> engine.advance_phase()
)doc")
        .def(py::init<>())
        .def(py::init<HierAABBTree&, const sbf::forest::ForestConfig&>(),
             "tree"_a, "config"_a)
        .def("find_free_box", &sbf::forest::FFBEngine::find_free_box,
             "seed"_a, "obs_compact"_a, "n_obs"_a,
             "min_edge_override"_a = -1.0)
        .def("advance_phase", &sbf::forest::FFBEngine::advance_phase)
        .def("reset_phases", &sbf::forest::FFBEngine::reset_phases)
        .def("current_phase", &sbf::forest::FFBEngine::current_phase)
        .def("boxes_in_phase", &sbf::forest::FFBEngine::boxes_in_phase);

    // ── ForestGrower ─────────────────────────────────────────────────────
    py::class_<sbf::forest::ForestGrower>(m, "ForestGrower", R"doc(
Orchestrates the forest growth algorithm.

Given a robot, collision checker, tree, and forest, grows boxes
from start/goal anchors and fills C-space using BFS expansion.

Example
-------
>>> grower = pysbf2.ForestGrower(robot, checker, tree, forest, config)
>>> success = grower.grow(start, goal)
>>> print(f"Grew {forest.n_boxes()} boxes")
)doc")
        .def(py::init<>())
        .def(py::init<const Robot&, const CollisionChecker&,
                       HierAABBTree&, SafeBoxForest&, const SBFConfig&>(),
             "robot"_a, "checker"_a, "tree"_a, "forest"_a, "config"_a)
        .def("grow", &sbf::forest::ForestGrower::grow, "start"_a, "goal"_a,
             "Grow forest for a single start-goal pair.")
        .def("grow_multi", &sbf::forest::ForestGrower::grow_multi,
             "pairs"_a, "n_random_boxes"_a = 5000, "timeout"_a = 120.0,
             "Grow forest for multiple start-goal pairs.")
        .def("regrow", &sbf::forest::ForestGrower::regrow,
             "n_target"_a, "timeout"_a = 60.0,
             "Regrow to fill depleted regions.");

    // =====================================================================
    //  7. BRIDGE — coarsening & connectivity
    // =====================================================================

    // ── CoarsenResult ────────────────────────────────────────────────────
    py::class_<CoarsenResult>(m, "CoarsenResult",
                              "Statistics from coarsen_forest().")
        .def(py::init<>())
        .def_readwrite("merges_performed", &CoarsenResult::merges_performed)
        .def_readwrite("rounds", &CoarsenResult::rounds)
        .def_readwrite("boxes_before", &CoarsenResult::boxes_before)
        .def_readwrite("boxes_after", &CoarsenResult::boxes_after);

    py::class_<GreedyCoarsenResult>(m, "GreedyCoarsenResult",
                                    "Statistics from coarsen_greedy().")
        .def(py::init<>())
        .def_readwrite("merges_performed", &GreedyCoarsenResult::merges_performed)
        .def_readwrite("rounds", &GreedyCoarsenResult::rounds)
        .def_readwrite("boxes_before", &GreedyCoarsenResult::boxes_before)
        .def_readwrite("boxes_after", &GreedyCoarsenResult::boxes_after)
        .def_readwrite("elapsed_sec", &GreedyCoarsenResult::elapsed_sec);

    m.def("coarsen_forest", &coarsen_forest,
          "forest"_a, "checker"_a, "max_rounds"_a = 20,
          "Merge adjacent collision-free boxes to reduce total count.");
    m.def("coarsen_greedy", &coarsen_greedy,
          "forest"_a, "checker"_a, "target_boxes"_a = 0,
          "max_rounds"_a = 100, "adjacency_tol"_a = 1e-10,
          "tree"_a = nullptr, "tree_split_depth"_a = 3,
          "max_tree_fk_per_round"_a = 2000,
          "Greedy coarsening with optional tree-based hull checks.");

    // ── UnionFind ────────────────────────────────────────────────────────
    py::class_<UnionFind>(m, "UnionFind", R"doc(
Union-Find (Disjoint Set Union) data structure.
Supports path compression and union by rank.
)doc")
        .def(py::init<>())
        .def(py::init<int>(), "n"_a)
        .def("find", &UnionFind::find, "x"_a)
        .def("unite", &UnionFind::unite, "x"_a, "y"_a)
        .def("connected", &UnionFind::connected, "x"_a, "y"_a)
        .def("n_components", &UnionFind::n_components)
        .def("components", &UnionFind::components);

    // ── BridgeResult ─────────────────────────────────────────────────────
    py::class_<BridgeResult>(m, "BridgeResult")
        .def(py::init<>())
        .def_readwrite("bridges", &BridgeResult::bridges)
        .def_readwrite("n_bridges", &BridgeResult::n_bridges)
        .def_readwrite("fully_connected", &BridgeResult::fully_connected);

    m.def("find_islands", &find_islands, "adjacency"_a, "all_ids"_a,
          "Detect connected components in the adjacency graph.");
    m.def("bridge_islands", &bridge_islands,
          "islands"_a, "boxes"_a, "max_attempts"_a = 20,
          "Connect disconnected islands via closest-pair bridges.");

    // =====================================================================
    //  8. PLANNER
    // =====================================================================

    // ── SBFPlanner (end-to-end) ──────────────────────────────────────────
    py::class_<SBFPlanner>(m, "SBFPlanner", R"doc(
End-to-end SafeBoxForest motion planner.

Integrates forest growth, coarsening, graph search, and path
smoothing into a single ``plan()`` call. Also supports incremental
obstacle addition/removal.

Example
-------
>>> planner = pysbf2.SBFPlanner(robot, obstacles, config)
>>> result = planner.plan(start, goal, timeout=30.0)
>>> if result.success:
...     print(f"Path found: {result.n_waypoints()} waypoints, "
...           f"cost={result.cost:.3f}")
>>> # Incremental update
>>> planner.add_obstacle(new_obs)
>>> planner.regrow(500, timeout=10.0)
>>> result2 = planner.query(start, goal)
)doc")
        .def(py::init<>())
        .def(py::init<const Robot&, const std::vector<Obstacle>&,
                       const SBFConfig&>(),
             "robot"_a, "obstacles"_a, "config"_a = SBFConfig())
        // Core planning
        .def("plan", &SBFPlanner::plan,
             "start"_a, "goal"_a, "timeout"_a = 30.0, R"doc(
Build forest + find path in one call.

Parameters
----------
start : numpy.ndarray (n_joints,)
    Start configuration.
goal : numpy.ndarray (n_joints,)
    Goal configuration.
timeout : float
    Maximum wall-clock time in seconds.

Returns
-------
PlanningResult
    Contains path, cost, timing, and diagnostic info.
)doc")
        // Build only
        .def("build", &SBFPlanner::build,
             "start"_a, "goal"_a, "timeout"_a = 30.0,
             "Build forest without planning a path.")
        .def("build_multi", &SBFPlanner::build_multi,
             "pairs"_a, "n_random_boxes"_a = 5000, "timeout"_a = 120.0,
             "Build forest covering multiple start-goal pairs.")
        // Query
        .def("query", &SBFPlanner::query,
             "start"_a, "goal"_a, "timeout"_a = 10.0,
             "Query path on pre-built forest.")
        // Incremental
        .def("add_obstacle", &SBFPlanner::add_obstacle, "obs"_a,
             "Add obstacle and invalidate affected boxes.")
        .def("remove_obstacle", &SBFPlanner::remove_obstacle, "name"_a,
             "Remove obstacle by name.")
        .def("regrow", &SBFPlanner::regrow,
             "n_target"_a, "timeout"_a = 60.0,
             "Grow new boxes to fill depleted regions.")
        .def("clear_forest", &SBFPlanner::clear_forest,
             "Clear forest but keep FK cache.")
        // Accessors
        .def("forest", &SBFPlanner::forest,
             py::return_value_policy::reference_internal)
        .def("forest_built", &SBFPlanner::forest_built)
        .def("config",
             py::overload_cast<>(&SBFPlanner::config, py::const_),
             py::return_value_policy::reference_internal)
        .def("tree", [](SBFPlanner& p) -> HierAABBTree& {
            return const_cast<HierAABBTree&>(p.tree());
        }, py::return_value_policy::reference_internal);

    // ── DijkstraResult ───────────────────────────────────────────────────
    py::class_<DijkstraResult>(m, "DijkstraResult",
                               "Result of Dijkstra graph search on box adjacency.")
        .def(py::init<>())
        .def_readwrite("path", &DijkstraResult::path)
        .def_readwrite("total_cost", &DijkstraResult::total_cost)
        .def_readwrite("found", &DijkstraResult::found);

    m.def("dijkstra", &dijkstra,
          "adjacency"_a, "start_ids"_a, "goal_ids"_a, "weight_fn"_a,
          "Dijkstra shortest path with custom weight function.");
    m.def("dijkstra_center_distance", &dijkstra_center_distance,
          "adjacency"_a, "boxes"_a, "start_ids"_a, "goal_ids"_a,
          "Dijkstra with L2 distance between box centers as weight.");
    m.def("extract_waypoints", &extract_waypoints,
          "box_sequence"_a, "boxes"_a, "start"_a, "goal"_a,
          "Convert box sequence to waypoint list via shared face centers.");

    // ── TreeConnector ────────────────────────────────────────────────────
    py::class_<TreeConnector>(m, "TreeConnector", R"doc(
Connects start/goal configurations to the forest.
)doc")
        .def(py::init<>())
        .def(py::init<const SafeBoxForest&, const CollisionChecker&,
                       double, double, int>(),
             "forest"_a, "checker"_a, "connection_radius"_a = 2.0,
             "segment_resolution"_a = 0.05, "max_attempts"_a = 50);

    py::class_<TreeConnector::AttachResult>(m, "AttachResult")
        .def(py::init<>())
        .def_readwrite("box_id", &TreeConnector::AttachResult::box_id)
        .def_readwrite("inside_box", &TreeConnector::AttachResult::inside_box)
        .def_readwrite("attach_point", &TreeConnector::AttachResult::attach_point);

    // ── PathSmoother ─────────────────────────────────────────────────────
    py::class_<PathSmoother>(m, "PathSmoother", R"doc(
Path optimization via shortcutting and smoothing.

Example
-------
>>> smoother = pysbf2.PathSmoother(checker, segment_resolution=0.05)
>>> smoothed = smoother.shortcut(raw_path, max_iters=200)
>>> final = smoother.box_aware_shortcut(smoothed, forest)
)doc")
        .def(py::init<>())
        .def(py::init<const CollisionChecker&, double>(),
             "checker"_a, "segment_resolution"_a = 0.05)
        .def("shortcut", &PathSmoother::shortcut,
             "path"_a, "max_iters"_a = 100, "rng"_a = nullptr,
             "Random shortcutting: pick two points, try direct connection.")
        .def("box_aware_shortcut", &PathSmoother::box_aware_shortcut,
             "path"_a, "forest"_a, "max_iters"_a = 100, "rng"_a = nullptr,
             "Shortcut within/between adjacent boxes (guaranteed safe).")
        .def("smooth_moving_average", &PathSmoother::smooth_moving_average,
             "path"_a, "forest"_a, "window"_a = 5, "iterations"_a = 3,
             "Box-aware moving average smoothing.")
        .def("resample", &PathSmoother::resample,
             "path"_a, "resolution"_a = 0.05,
             "Resample path at uniform spacing.")
        .def_static("path_length", &PathSmoother::path_length, "path"_a,
                     "Compute total L2 path length.");

    // ── Pipeline helpers ─────────────────────────────────────────────────
    py::class_<SceneConfig>(m, "SceneConfig", R"doc(
Configuration for random scene generation.
)doc")
        .def(py::init<>())
        .def_readwrite("n_obstacles", &SceneConfig::n_obstacles)
        .def_readwrite("workspace_radius", &SceneConfig::workspace_radius)
        .def_readwrite("workspace_z_min", &SceneConfig::workspace_z_min)
        .def_readwrite("workspace_z_max", &SceneConfig::workspace_z_max)
        .def_readwrite("obs_size_min", &SceneConfig::obs_size_min)
        .def_readwrite("obs_size_max", &SceneConfig::obs_size_max);

    m.def("make_panda_config", &make_panda_config, "seed"_a = 0,
          "Create default SBFConfig tuned for Franka Panda.");
    m.def("plan_once", &plan_once,
          "robot"_a, "obstacles"_a, "start"_a, "goal"_a,
          "config"_a = SBFConfig(),
          "One-shot planning: build + plan + return result.");

    // =====================================================================
    //  9. VIZ
    // =====================================================================
    auto viz_mod = m.def_submodule("viz", "Visualization export utilities.");

    viz_mod.def("export_forest_json",
        py::overload_cast<const SafeBoxForest&, const std::string&>(
            &sbf::viz::export_forest_json),
        "forest"_a, "filepath"_a,
        "Export forest boxes + adjacency to JSON.");

    viz_mod.def("export_forest_json",
        py::overload_cast<const SafeBoxForest&, const std::vector<Obstacle>&,
                          const Eigen::MatrixXd&, const std::string&>(
            &sbf::viz::export_forest_json),
        "forest"_a, "obstacles"_a, "path"_a, "filepath"_a,
        "Export forest + obstacles + path to JSON.");

    viz_mod.def("export_path_frames", &sbf::viz::export_path_frames,
        "robot"_a, "path"_a, "filepath"_a,
        "Export FK frames along path waypoints to JSON.");

    viz_mod.def("export_boxes_csv", &sbf::viz::export_boxes_csv,
        "forest"_a, "filepath"_a,
        "Export box intervals as CSV.");

    // =====================================================================
    //  10. IO
    // =====================================================================
    auto io_mod = m.def_submodule("io", "JSON I/O and HCache persistence.");

    io_mod.def("load_obstacles_json", &load_obstacles_json, "path"_a,
               "Load obstacles from JSON file.");
    io_mod.def("save_obstacles_json", &save_obstacles_json,
               "path"_a, "obstacles"_a,
               "Save obstacles to JSON file.");
    io_mod.def("save_result_json", &save_result_json,
               "path"_a, "result"_a,
               "Save PlanningResult to JSON file.");
    io_mod.def("load_result_json", &load_result_json, "path"_a,
               "Load PlanningResult from JSON file.");

    // ── HCacheFile ───────────────────────────────────────────────────────
    py::class_<HCacheFile>(io_mod, "HCacheFile", R"doc(
Memory-mapped persistence file for HierAABBTree nodes.

Supports creating new cache files and opening existing ones.
All nodes are stored in a flat binary format for fast I/O.
)doc")
        .def_static("create", &HCacheFile::create,
             "path"_a, "n_dims"_a, "n_links"_a, "limits"_a,
             "fingerprint"_a = "", "initial_cap"_a = 64)
        .def_static("open", &HCacheFile::open, "path"_a)
        .def("flush", &HCacheFile::flush)
        .def("close", &HCacheFile::close)
        .def("is_open", &HCacheFile::is_open)
        .def("n_nodes", &HCacheFile::n_nodes)
        .def("n_fk_calls", &HCacheFile::n_fk_calls)
        .def("set_n_fk_calls", &HCacheFile::set_n_fk_calls, "v"_a);

    // =====================================================================
    //  Feature flags
    // =====================================================================
    m.def("drake_available", &drake_available,
          "Whether Drake GCS support is compiled in.");
#ifdef SBF_WITH_OMPL
    m.def("ompl_available", []() { return true; });
#else
    m.def("ompl_available", []() { return false; },
          "Whether OMPL adapter support is compiled in.");
#endif

    // ── GCSOptimizer (if Drake available) ────────────────────────────────
    py::class_<GCSResult>(m, "GCSResult", "Result from GCS trajectory optimization.")
        .def(py::init<>())
        .def_readwrite("success", &GCSResult::success)
        .def_readwrite("waypoints", &GCSResult::waypoints)
        .def_readwrite("active_box_ids", &GCSResult::active_box_ids)
        .def_readwrite("cost", &GCSResult::cost)
        .def_readwrite("solve_time", &GCSResult::solve_time);

    py::class_<GCSOptimizer>(m, "GCSOptimizer", R"doc(
Graph of Convex Sets optimizer (requires Drake).
Optimizes a trajectory through boxes using convex programming.
)doc")
        .def(py::init<>())
        .def("optimize", &GCSOptimizer::optimize,
             "forest"_a, "start"_a, "goal"_a, "corridor_hops"_a = 2)
        .def_static("drake_available", &GCSOptimizer::drake_available);
}
