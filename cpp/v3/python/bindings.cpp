// ═══════════════════════════════════════════════════════════════════════════
// SafeBoxForest v3 — Comprehensive pybind11 bindings
//
// Module name: pysbf3
//
// Exposes all public interfaces:
//   common (types, config), robot, scene, voxel, forest (LECT)
// ═══════════════════════════════════════════════════════════════════════════
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/numpy.h>

#include <optional>
#include <string>
#include <vector>

// ── v3 headers ──────────────────────────────────────────────────────────────
#include "sbf/common/types.h"
#include "sbf/common/config.h"
#include "sbf/robot/robot.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/scene/scene.h"
#include "sbf/voxel/voxel_grid.h"
#include "sbf/voxel/hull_rasteriser.h"
#include "sbf/forest/lect.h"
#include "sbf/forest/forest_grower.h"
#include "sbf/forest/grower_config.h"

namespace py = pybind11;
using namespace pybind11::literals;
using namespace sbf;

// ═══════════════════════════════════════════════════════════════════════════
//  MODULE DEFINITION
// ═══════════════════════════════════════════════════════════════════════════

PYBIND11_MODULE(pysbf3, m) {
    m.doc() = R"doc(
SafeBoxForest v3 (pysbf3) — Motion planning via certified safe boxes in C-space.

Key components
--------------
- **Interval / Obstacle / BoxNode**: Core geometric types
- **Robot**: DH kinematic model with interval FK
- **Scene**: Obstacle collection management
- **VoxelGrid**: Sparse BitBrick voxel grid (fill, collide, merge)
- **LECT**: Lifelong Envelope Cache Tree (FFB, coarsening, persistence)
)doc";

    // ─────────────────────────────────────────────────────────────────────
    //  common/types.h
    // ─────────────────────────────────────────────────────────────────────

    py::class_<Interval>(m, "Interval", "Closed interval [lo, hi]")
        .def(py::init<>())
        .def(py::init<double, double>(), "lo"_a, "hi"_a)
        .def_readwrite("lo", &Interval::lo)
        .def_readwrite("hi", &Interval::hi)
        .def("width",    &Interval::width)
        .def("center",   &Interval::center)
        .def("mid",      &Interval::mid)
        .def("contains", &Interval::contains, "v"_a, "tol"_a = CONTAIN_TOL)
        .def("overlaps", &Interval::overlaps, "other"_a, "tol"_a = 0.0)
        .def("hull",     &Interval::hull, "other"_a)
        .def("intersect",&Interval::intersect, "other"_a)
        .def("__repr__", [](const Interval& iv) {
            return "[" + std::to_string(iv.lo) + ", " + std::to_string(iv.hi) + "]";
        });

    py::class_<Obstacle>(m, "Obstacle", "Axis-aligned box obstacle (center + half_sizes)")
        .def(py::init<>())
        .def(py::init<Eigen::Vector3d, Eigen::Vector3d, std::string>(),
             "center"_a, "half_sizes"_a, "name"_a = "")
        .def_readwrite("center",     &Obstacle::center)
        .def_readwrite("half_sizes", &Obstacle::half_sizes)
        .def_readwrite("name",       &Obstacle::name)
        .def("lo", &Obstacle::lo)
        .def("hi", &Obstacle::hi)
        .def("__repr__", [](const Obstacle& o) {
            return "Obstacle('" + o.name + "', center=[" +
                   std::to_string(o.center[0]) + "," +
                   std::to_string(o.center[1]) + "," +
                   std::to_string(o.center[2]) + "])";
        });

    py::class_<BoxNode>(m, "BoxNode", "Collision-free C-space box")
        .def(py::init<>())
        .def_readwrite("id",              &BoxNode::id)
        .def_readwrite("joint_intervals", &BoxNode::joint_intervals)
        .def_readwrite("seed_config",     &BoxNode::seed_config)
        .def_readwrite("volume",          &BoxNode::volume)
        .def_readwrite("parent_id",       &BoxNode::parent_id)
        .def_readwrite("tree_id",         &BoxNode::tree_id)
        .def_readwrite("children_ids",    &BoxNode::children_ids)
        .def("n_dims",  &BoxNode::n_dims)
        .def("center",  &BoxNode::center)
        .def("compute_volume", &BoxNode::compute_volume)
        .def("contains",       &BoxNode::contains, "q"_a, "tol"_a = CONTAIN_TOL)
        .def("distance_to_config", &BoxNode::distance_to_config, "q"_a)
        .def("nearest_point_to",   &BoxNode::nearest_point_to, "q"_a)
        .def("overlaps_with", &BoxNode::overlaps_with, "other"_a, "tol"_a = 0.0)
        .def("__repr__", [](const BoxNode& b) {
            return "BoxNode(id=" + std::to_string(b.id) +
                   ", vol=" + std::to_string(b.volume) + ")";
        });

    py::class_<JointLimits>(m, "JointLimits", "Per-joint interval limits")
        .def(py::init<>())
        .def_readwrite("limits", &JointLimits::limits)
        .def("n_dims",   &JointLimits::n_dims)
        .def("contains", &JointLimits::contains, "q"_a, "tol"_a = CONTAIN_TOL)
        .def("clamp",    &JointLimits::clamp, "q"_a);

    py::class_<FFBResult>(m, "FFBResult", "Result of find_free_box query")
        .def(py::init<>())
        .def_readwrite("node_idx",    &FFBResult::node_idx)
        .def_readwrite("path",        &FFBResult::path)
        .def_readwrite("fail_code",   &FFBResult::fail_code)
        .def_readwrite("n_new_nodes", &FFBResult::n_new_nodes)
        .def_readwrite("n_fk_calls",  &FFBResult::n_fk_calls)
        .def("success", &FFBResult::success)
        .def("__repr__", [](const FFBResult& r) {
            return std::string("FFBResult(") +
                   (r.success() ? "OK" : "FAIL") +
                   ", node=" + std::to_string(r.node_idx) + ")";
        });

    py::class_<PlanningResult>(m, "PlanningResult", "Motion planning result")
        .def(py::init<>())
        .def_readwrite("success",       &PlanningResult::success)
        .def_readwrite("path",          &PlanningResult::path)
        .def_readwrite("cost",          &PlanningResult::cost)
        .def_readwrite("planning_time", &PlanningResult::planning_time)
        .def_readwrite("collision_checks", &PlanningResult::collision_checks)
        .def_readwrite("nodes_explored",   &PlanningResult::nodes_explored)
        .def_readwrite("phase_times",   &PlanningResult::phase_times)
        .def_readwrite("metadata",      &PlanningResult::metadata)
        .def("n_waypoints", &PlanningResult::n_waypoints);

    py::class_<Edge>(m, "Edge", "Graph edge (from_id, to_id, weight)")
        .def(py::init<>())
        .def(py::init<int, int, double>(), "from_id"_a, "to_id"_a, "weight"_a = 1.0)
        .def_readwrite("from_id", &Edge::from_id)
        .def_readwrite("to_id",   &Edge::to_id)
        .def_readwrite("weight",  &Edge::weight);

    // ─────────────────────────────────────────────────────────────────────
    //  common/config.h  — Enums and config structs
    // ─────────────────────────────────────────────────────────────────────

    py::enum_<envelope::StoreFormat>(m, "StoreFormat")
        .value("AABB_LEGACY", envelope::StoreFormat::AABB_LEGACY)
        .value("FRAMES",      envelope::StoreFormat::FRAMES);

    py::enum_<envelope::CollisionPolicy>(m, "CollisionPolicy")
        .value("AABB",       envelope::CollisionPolicy::AABB)
        .value("AABB_SUBDIV",envelope::CollisionPolicy::AABB_SUBDIV)
        .value("GRID",       envelope::CollisionPolicy::GRID);

    py::class_<envelope::EnvelopeConfig>(m, "EnvelopeConfig")
        .def(py::init<>())
        .def_readwrite("max_depth",        &envelope::EnvelopeConfig::max_depth)
        .def_readwrite("min_edge",         &envelope::EnvelopeConfig::min_edge)
        .def_readwrite("min_edge_anchor",  &envelope::EnvelopeConfig::min_edge_anchor)
        .def_readwrite("min_edge_relaxed", &envelope::EnvelopeConfig::min_edge_relaxed)
        .def_readwrite("promotion_depth",  &envelope::EnvelopeConfig::promotion_depth)
        .def_readwrite("safety_check_depth",    &envelope::EnvelopeConfig::safety_check_depth)
        .def_readwrite("store_format",          &envelope::EnvelopeConfig::store_format)
        .def_readwrite("collision_policy",      &envelope::EnvelopeConfig::collision_policy)
        .def_readwrite("link_subdivision_n",    &envelope::EnvelopeConfig::link_subdivision_n)
        .def_readwrite("link_subdivision_max",  &envelope::EnvelopeConfig::link_subdivision_max)
        .def_readwrite("use_grid_union",        &envelope::EnvelopeConfig::use_grid_union)
        .def_readwrite("grid_n_sub_per_link",   &envelope::EnvelopeConfig::grid_n_sub_per_link);

    py::class_<forest::ForestConfig>(m, "ForestConfig")
        .def(py::init<>())
        .def_readwrite("max_boxes",              &forest::ForestConfig::max_boxes)
        .def_readwrite("max_consecutive_miss",   &forest::ForestConfig::max_consecutive_miss)
        .def_readwrite("bfs_phase_k",            &forest::ForestConfig::bfs_phase_k)
        .def_readwrite("bfs_phase_budget",       &forest::ForestConfig::bfs_phase_budget)
        .def_readwrite("min_boxes_per_pair",     &forest::ForestConfig::min_boxes_per_pair)
        .def_readwrite("max_boxes_per_pair",     &forest::ForestConfig::max_boxes_per_pair)
        .def_readwrite("guided_sample_ratio",    &forest::ForestConfig::guided_sample_ratio)
        .def_readwrite("boundary_expand_epsilon",&forest::ForestConfig::boundary_expand_epsilon)
        .def_readwrite("n_edge_samples",         &forest::ForestConfig::n_edge_samples)
        .def_readwrite("adjacency_tol",          &forest::ForestConfig::adjacency_tol)
        .def_readwrite("proxy_anchor_max_samples", &forest::ForestConfig::proxy_anchor_max_samples)
        .def_readwrite("proxy_anchor_radius",      &forest::ForestConfig::proxy_anchor_radius);

    py::class_<bridge::BridgeConfig>(m, "BridgeConfig")
        .def(py::init<>())
        .def_readwrite("coarsen_max_rounds",      &bridge::BridgeConfig::coarsen_max_rounds)
        .def_readwrite("coarsen_target_boxes",    &bridge::BridgeConfig::coarsen_target_boxes)
        .def_readwrite("coarsen_greedy_rounds",   &bridge::BridgeConfig::coarsen_greedy_rounds)
        .def_readwrite("coarsen_grid_check",      &bridge::BridgeConfig::coarsen_grid_check)
        .def_readwrite("coarsen_split_depth",     &bridge::BridgeConfig::coarsen_split_depth)
        .def_readwrite("coarsen_max_tree_fk",     &bridge::BridgeConfig::coarsen_max_tree_fk)
        .def_readwrite("corridor_hops",           &bridge::BridgeConfig::corridor_hops)
        .def_readwrite("use_gcs",                 &bridge::BridgeConfig::use_gcs);

    py::class_<planner::PlannerConfig>(m, "PlannerConfig")
        .def(py::init<>())
        .def_readwrite("shortcut_max_iters",   &planner::PlannerConfig::shortcut_max_iters)
        .def_readwrite("segment_resolution",   &planner::PlannerConfig::segment_resolution)
        .def_readwrite("parallel_grow",        &planner::PlannerConfig::parallel_grow)
        .def_readwrite("n_partitions_depth",   &planner::PlannerConfig::n_partitions_depth)
        .def_readwrite("parallel_workers",     &planner::PlannerConfig::parallel_workers)
        .def_readwrite("seed",                 &planner::PlannerConfig::seed);

    py::class_<SBFConfig>(m, "SBFConfig", "Aggregate config (decomposable to module configs)")
        .def(py::init<>())
        .def_readwrite("max_boxes",              &SBFConfig::max_boxes)
        .def_readwrite("max_consecutive_miss",   &SBFConfig::max_consecutive_miss)
        .def_readwrite("ffb_min_edge",           &SBFConfig::ffb_min_edge)
        .def_readwrite("ffb_min_edge_anchor",    &SBFConfig::ffb_min_edge_anchor)
        .def_readwrite("ffb_min_edge_relaxed",   &SBFConfig::ffb_min_edge_relaxed)
        .def_readwrite("ffb_max_depth",          &SBFConfig::ffb_max_depth)
        .def_readwrite("guided_sample_ratio",    &SBFConfig::guided_sample_ratio)
        .def_readwrite("seed",                   &SBFConfig::seed)
        .def_readwrite("use_cache",              &SBFConfig::use_cache)
        .def_readwrite("cache_path",             &SBFConfig::cache_path)
        .def_readwrite("store_format",           &SBFConfig::store_format)
        .def_readwrite("collision_policy",       &SBFConfig::collision_policy)
        .def_readwrite("link_subdivision_n",     &SBFConfig::link_subdivision_n)
        .def("envelope_config", &SBFConfig::envelope_config)
        .def("forest_config",   &SBFConfig::forest_config)
        .def("bridge_config",   &SBFConfig::bridge_config)
        .def("planner_config",  &SBFConfig::planner_config);

    // ─────────────────────────────────────────────────────────────────────
    //  robot/robot.h
    // ─────────────────────────────────────────────────────────────────────

    py::class_<DHParam>(m, "DHParam", "DH parameter for one joint")
        .def(py::init<>())
        .def_readwrite("alpha", &DHParam::alpha)
        .def_readwrite("a",     &DHParam::a)
        .def_readwrite("d",     &DHParam::d)
        .def_readwrite("theta", &DHParam::theta)
        .def_readwrite("joint_type", &DHParam::joint_type);

    py::class_<Robot>(m, "Robot", "Robot kinematic model (DH parameters)")
        .def(py::init<>())
        .def_static("from_json", &Robot::from_json, "path"_a,
             R"doc(Load robot from JSON config file (panda.json / iiwa14.json))doc")
        .def("name",         &Robot::name)
        .def("n_joints",     &Robot::n_joints)
        .def("n_links",      &Robot::n_links)
        .def("n_transforms", &Robot::n_transforms)
        .def("n_active_links", &Robot::n_active_links)
        .def("joint_limits", &Robot::joint_limits, py::return_value_policy::reference_internal)
        .def("dh_params",    &Robot::dh_params, py::return_value_policy::reference_internal)
        .def("link_radii",   &Robot::link_radii, py::return_value_policy::reference_internal)
        .def("has_link_radii", &Robot::has_link_radii)
        .def("has_tool",     &Robot::has_tool)
        .def("__repr__", [](const Robot& r) {
            return "Robot('" + r.name() + "', " +
                   std::to_string(r.n_joints()) + " joints, " +
                   std::to_string(r.n_active_links()) + " active links)";
        });

    // ─────────────────────────────────────────────────────────────────────
    //  scene/scene.h
    // ─────────────────────────────────────────────────────────────────────

    py::class_<Scene>(m, "Scene", "Obstacle collection")
        .def(py::init<>())
        .def(py::init<std::vector<Obstacle>>(), "obstacles"_a)
        .def("add_obstacle",    &Scene::add_obstacle, "obs"_a)
        .def("remove_obstacle", &Scene::remove_obstacle, "name"_a)
        .def("clear",           &Scene::clear)
        .def("obstacles",       &Scene::obstacles, py::return_value_policy::reference_internal)
        .def("n_obstacles",     &Scene::n_obstacles)
        .def("__repr__", [](const Scene& s) {
            return "Scene(" + std::to_string(s.n_obstacles()) + " obstacles)";
        });

    // ─────────────────────────────────────────────────────────────────────
    //  voxel/voxel_grid.h  — Sparse BitBrick VoxelGrid
    // ─────────────────────────────────────────────────────────────────────

    py::class_<voxel::VoxelGrid>(m, "VoxelGrid",
        R"doc(Sparse voxel grid backed by FlatBrickMap with 8³ BitBrick tiles.

Supports fill_aabb (axis-aligned box), fill_hull16 (turbo scanline),
merge (bitwise OR), collides (bitwise AND), and popcount queries.
)doc")
        .def(py::init<>())
        .def(py::init<double, double, double, double, double>(),
             "delta"_a, "ox"_a = 0.0, "oy"_a = 0.0, "oz"_a = 0.0,
             "safety_pad"_a = -1.0,
             R"doc(Create VoxelGrid.

Parameters
----------
delta : float
    Voxel edge length in metres (e.g. 0.02).
ox, oy, oz : float
    World-space origin (lower corner of cell 0,0,0).
safety_pad : float
    Extra radius padding for fill operations.
    Default (-1) = sqrt(3)*delta/2 for certified conservative coverage.
)doc")
        .def("fill_aabb", [](voxel::VoxelGrid& g, py::array_t<float> aabb) {
            auto buf = aabb.request();
            if (buf.size != 6) throw std::runtime_error("aabb must have 6 elements");
            g.fill_aabb(static_cast<const float*>(buf.ptr));
        }, "aabb"_a,
           R"doc(Fill voxels overlapping axis-aligned box [x_lo, y_lo, z_lo, x_hi, y_hi, z_hi])doc")

        .def("fill_hull16", [](voxel::VoxelGrid& g,
                                py::array_t<float> prox_iv,
                                py::array_t<float> dist_iv,
                                double link_radius) {
            auto p = prox_iv.request();
            auto d = dist_iv.request();
            if (p.size != 6 || d.size != 6)
                throw std::runtime_error("prox_iv and dist_iv must have 6 elements each");
            g.fill_hull16(static_cast<const float*>(p.ptr),
                          static_cast<const float*>(d.ptr),
                          link_radius);
        }, "prox_iv"_a, "dist_iv"_a, "link_radius"_a,
           R"doc(Turbo scanline rasterisation of Conv(B1 ∪ B2) ⊕ Ball(r).

Parameters
----------
prox_iv : array[6]
    Proximal endpoint AABB [xlo, ylo, zlo, xhi, yhi, zhi].
dist_iv : array[6]
    Distal endpoint AABB [xlo, ylo, zlo, xhi, yhi, zhi].
link_radius : float
    Cylindrical radius inflation.
)doc")
        .def("count_occupied",  &voxel::VoxelGrid::count_occupied,
             "Number of voxels with at least one bit set")
        .def("occupied_volume", &voxel::VoxelGrid::occupied_volume,
             "Total occupied volume in m³")
        .def("num_bricks",      &voxel::VoxelGrid::num_bricks,
             "Number of allocated 8³ BitBrick tiles")
        .def("merge", &voxel::VoxelGrid::merge, "other"_a,
             "Bitwise OR merge (zero-loss union)")
        .def("collides", &voxel::VoxelGrid::collides, "other"_a,
             "True if any voxel overlaps (bitwise AND != 0)")
        .def("count_colliding", &voxel::VoxelGrid::count_colliding, "other"_a,
             "Number of overlapping voxels")
        .def("clear",  &voxel::VoxelGrid::clear)
        .def("delta",  &voxel::VoxelGrid::delta)
        .def("safety_pad", &voxel::VoxelGrid::safety_pad)
        .def("__repr__", [](const voxel::VoxelGrid& g) {
            return "VoxelGrid(delta=" + std::to_string(g.delta()) +
                   ", bricks=" + std::to_string(g.num_bricks()) +
                   ", voxels=" + std::to_string(g.count_occupied()) + ")";
        });

    // ─────────────────────────────────────────────────────────────────────
    //  forest/lect.h  — Lifelong Envelope Cache Tree
    // ─────────────────────────────────────────────────────────────────────

    using LECT = forest::LECT;

    py::class_<LECT>(m, "LECT",
        R"doc(Lifelong Envelope Cache Tree (LECT).

Unified KD-tree caching per-node:
  1. Per-link AABBs (from Interval FK)
  2. Frame position intervals (universal base repr)
  3. Hull-16 VoxelGrids (sparse BitBrick)

Supports scene pre-rasterisation, find_free_box, and
hull-based greedy coarsening.
)doc")
        .def(py::init<>())
        .def(py::init<const Robot&, double, int>(),
             "robot"_a, "voxel_delta"_a = 0.02, "initial_cap"_a = 1024,
             R"doc(Construct LECT from robot model.

Parameters
----------
robot : Robot
    Robot kinematic model.
voxel_delta : float
    Hull-16 voxel resolution in metres.
initial_cap : int
    Initial node capacity.
)doc")
        // ── Metadata ────────────────────────────────────────────────────
        .def("n_nodes",        &LECT::n_nodes)
        .def("n_dims",         &LECT::n_dims)
        .def("n_active_links", &LECT::n_active_links)
        .def("capacity",       &LECT::capacity)
        .def("voxel_delta",    &LECT::voxel_delta)
        .def("robot",          &LECT::robot, py::return_value_policy::reference_internal)
        .def("root_limits",    &LECT::root_limits, py::return_value_policy::reference_internal)

        // ── Per-node data access ────────────────────────────────────────
        .def("has_aabb",       &LECT::has_aabb, "node_idx"_a)
        .def("has_frames",     &LECT::has_frames, "node_idx"_a)
        .def("has_hull_grid",  &LECT::has_hull_grid, "node_idx"_a)
        .def("get_hull_grid",  &LECT::get_hull_grid, "node_idx"_a,
             py::return_value_policy::reference_internal,
             "Get hull-16 VoxelGrid for a node (const ref)")

        .def("get_link_aabbs", [](const LECT& lect, int node_idx) -> py::array_t<float> {
            const float* data = lect.get_link_aabbs(node_idx);
            if (!data) return py::array_t<float>();
            int n = lect.n_active_links() * 6;
            py::array_t<float> arr({n});
            std::memcpy(arr.mutable_data(), data, n * sizeof(float));
            return arr;
        }, "node_idx"_a,
           "Per-link AABBs as flat array [n_active_links * 6]")

        // ── Tree structure ──────────────────────────────────────────────
        .def("left",      &LECT::left,  "i"_a)
        .def("right",     &LECT::right, "i"_a)
        .def("parent",    &LECT::parent, "i"_a)
        .def("depth",     &LECT::depth, "i"_a)
        .def("split_val", &LECT::split_val, "i"_a)
        .def("is_leaf",   &LECT::is_leaf, "i"_a)
        .def("node_intervals", &LECT::node_intervals, "node_idx"_a,
             "Reconstruct C-space intervals for a node")

        // ── Scene management ────────────────────────────────────────────
        .def("set_scene", [](LECT& lect, const std::vector<Obstacle>& obstacles) {
            lect.set_scene(obstacles.data(), static_cast<int>(obstacles.size()));
        }, "obstacles"_a,
           R"doc(Pre-rasterize obstacles into shared VoxelGrid for fast hull collision.

Call once at scene initialisation. Reuse across all FFB queries.
If obstacles change, call set_scene again to rebuild.
)doc")
        .def("clear_scene",    &LECT::clear_scene,
             "Clear the pre-rasterized scene grid")
        .def("has_scene_grid", &LECT::has_scene_grid,
             "Whether a pre-rasterized scene grid is available")
        .def("scene_grid",     &LECT::scene_grid,
             py::return_value_policy::reference_internal,
             "Access the pre-rasterized scene grid (const)")

        // ── Core tree operations ────────────────────────────────────────
        .def("find_free_box", [](LECT& lect,
                                  const Eigen::VectorXd& seed,
                                  const std::vector<Obstacle>& obstacles,
                                  double min_edge, int max_depth) {
            return lect.find_free_box(seed, obstacles.data(),
                                      static_cast<int>(obstacles.size()),
                                      min_edge, max_depth);
        }, "seed"_a, "obstacles"_a,
           "min_edge"_a = 1e-4, "max_depth"_a = 30,
           R"doc(Find collision-free box containing the seed configuration.

Parameters
----------
seed : ndarray
    Target configuration (n_dims).
obstacles : list[Obstacle]
    Obstacle array.
min_edge : float
    Minimum box edge length before stopping.
max_depth : int
    Maximum tree depth.

Returns
-------
FFBResult
    Result with node_idx and descent path.
)doc")

        .def("mark_occupied",   &LECT::mark_occupied, "node_idx"_a, "box_id"_a)
        .def("unmark_occupied", &LECT::unmark_occupied, "node_idx"_a)
        .def("is_occupied",     &LECT::is_occupied, "node_idx"_a)
        .def("forest_id",      &LECT::forest_id, "node_idx"_a)

        // ── Persistence ─────────────────────────────────────────────────
        .def("save", &LECT::save, "dir"_a,
             R"doc(Save all data to directory:
  dir/lect.hcache — tree structure + per-link AABBs
  dir/lect.frames — frame position intervals
  dir/lect.hulls  — hull-16 VoxelGrids
)doc")
        .def("load", &LECT::load, "dir"_a, "robot"_a,
             "Load from directory. Requires the same Robot model.")

        // ── Public collision queries ────────────────────────────────────
        .def("collides_scene", [](const LECT& lect, int node_idx,
                                   const std::vector<Obstacle>& obstacles) {
            return lect.collides_scene(node_idx, obstacles.data(),
                                       static_cast<int>(obstacles.size()));
        }, "node_idx"_a, "obstacles"_a,
           "Two-stage collision: AABB early-out → Hull-16 refinement")

        .def("hull_collides_grid", &LECT::hull_collides_grid,
             "node_idx"_a, "obs_grid"_a,
             "Hull-16 collision against a pre-built VoxelGrid")

        // ── Hull-based Greedy Coarsening ────────────────────────────────
        .def("merged_children_hull_volume", &LECT::merged_children_hull_volume,
             "node_idx"_a,
             "Bitwise-OR merged hull volume of children (m³). Returns -1 if not computable.")
        .def("coarsen_volume_ratio", &LECT::coarsen_volume_ratio,
             "node_idx"_a,
             "Coarsening quality ratio: merged_vol / max(left, right). Closer to 1.0 is better.")
        .def("merge_children_hulls", &LECT::merge_children_hulls,
             "node_idx"_a,
             "Merge child hull grids into parent (bitwise-OR). Returns True on success.")

        // ── Statistics ──────────────────────────────────────────────────
        .def("count_nodes_with_aabb", &LECT::count_nodes_with_aabb)
        .def("count_nodes_with_hull", &LECT::count_nodes_with_hull)
        .def("total_hull_voxels",     &LECT::total_hull_voxels)
        .def("scene_grid_voxels",     &LECT::scene_grid_voxels)

        .def("__repr__", [](const LECT& l) {
            return "LECT(nodes=" + std::to_string(l.n_nodes()) +
                   ", dims=" + std::to_string(l.n_dims()) +
                   ", delta=" + std::to_string(l.voxel_delta()) + ")";
        });

    // ─────────────────────────────────────────────────────────────────────
    //  voxel/hull_rasteriser.h — free functions
    // ─────────────────────────────────────────────────────────────────────

    m.def("rasterise_box_obstacle",
          [](double cx, double cy, double cz,
             double hx, double hy, double hz,
             voxel::VoxelGrid& grid) {
              voxel::rasterise_box_obstacle(
                  static_cast<float>(cx), static_cast<float>(cy), static_cast<float>(cz),
                  static_cast<float>(hx), static_cast<float>(hy), static_cast<float>(hz),
                  grid);
          },
          "cx"_a, "cy"_a, "cz"_a, "hx"_a, "hy"_a, "hz"_a, "grid"_a,
          "Rasterize a box obstacle (center + half-sizes) into a VoxelGrid");

    // ─────────────────────────────────────────────────────────────────────
    //  forest/grower_config.h  — GrowerConfig
    // ─────────────────────────────────────────────────────────────────────

    using GrowerConfig = forest::GrowerConfig;
    using GrowerResult = forest::GrowerResult;
    using ForestGrower = forest::ForestGrower;

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
        .def_readwrite("bridge_samples",         &GrowerConfig::bridge_samples)
        .def_readwrite("root_min_edge",          &GrowerConfig::root_min_edge)
        .def_readwrite("warm_start_depth",       &GrowerConfig::warm_start_depth)
        .def_readwrite("lect_cache_dir",         &GrowerConfig::lect_cache_dir)
        .def_readwrite("adaptive_min_edge",      &GrowerConfig::adaptive_min_edge)
        .def_readwrite("coarse_min_edge",        &GrowerConfig::coarse_min_edge)
        .def_readwrite("coarse_fraction",        &GrowerConfig::coarse_fraction)
        .def_readwrite("adaptive_n_stages",      &GrowerConfig::adaptive_n_stages)
        .def_readwrite("hull_skip_vol",          &GrowerConfig::hull_skip_vol)
        .def_readwrite("coarsen_enabled",        &GrowerConfig::coarsen_enabled)
        .def_readwrite("coarsen_target_boxes",   &GrowerConfig::coarsen_target_boxes)
        .def_readwrite("max_coarsen_rounds",     &GrowerConfig::max_coarsen_rounds)
        .def_readwrite("coarsen_score_threshold",&GrowerConfig::coarsen_score_threshold);

    py::enum_<GrowerConfig::Mode>(gc, "Mode")
        .value("Wavefront", GrowerConfig::Mode::Wavefront)
        .value("RRT",       GrowerConfig::Mode::RRT)
        .export_values();

    gc.def_readwrite("mode", &GrowerConfig::mode);

    // ── GrowerResult ─────────────────────────────────────────────────────
    py::class_<GrowerResult>(m, "GrowerResult")
        .def(py::init<>())
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

    // ── ForestGrower ─────────────────────────────────────────────────────
    py::class_<ForestGrower>(m, "ForestGrower")
        .def(py::init<const Robot&, const GrowerConfig&>(),
             "robot"_a, "config"_a)
        .def("set_endpoints", &ForestGrower::set_endpoints,
             "start"_a, "goal"_a)
        // grow with Python obstacle list (.min_point, .max_point)
        .def("grow", [](ForestGrower& grower, const py::list& py_obs) -> GrowerResult {
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
            py::gil_scoped_release release;
            return grower.grow(obs.data(), static_cast<int>(obs.size()));
        }, "obstacles"_a,
           "Grow the forest. obstacles: list with .min_point/.max_point attrs")
        // grow with native Obstacle vector
        .def("grow_typed", [](ForestGrower& grower,
                              const std::vector<Obstacle>& obs) -> GrowerResult {
            py::gil_scoped_release release;
            return grower.grow(obs.data(), static_cast<int>(obs.size()));
        }, "obstacles"_a,
           "Grow the forest with C++ Obstacle vector")
        .def("n_boxes", &ForestGrower::n_boxes)
        .def("boxes",   &ForestGrower::boxes, py::return_value_policy::reference_internal)
        .def("config",  &ForestGrower::config, py::return_value_policy::reference_internal)
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

    // ── robot_from_python helper ─────────────────────────────────────────
    m.def("robot_from_python", [](const py::object& py_robot) -> Robot {
        std::string name = py_robot.attr("name").cast<std::string>();
        auto py_dh = py_robot.attr("dh_params").cast<py::list>();
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
                try { radii = py_radii.cast<std::vector<double>>(); }
                catch (...) {}
            }
        }
        return Robot(name, dh, jl, tool, radii);
    }, "py_robot"_a,
       "Convert a Python Robot object to C++ Robot");
}
