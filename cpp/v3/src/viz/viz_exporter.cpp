// SafeBoxForest v3 — Visualization JSON Exporter implementation
#include "sbf/viz/viz_exporter.h"
#include "sbf/robot/fk.h"
#include "sbf/robot/interval_fk.h"
#include "sbf/envelope/envelope_derive.h"
#include "sbf/envelope/envelope_derive_critical.h"
#include "sbf/envelope/frame_store.h"

#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

#ifdef _MSC_VER
#include <intrin.h>
static inline int ctz64(uint64_t v) {
    unsigned long idx;
    _BitScanForward64(&idx, v);
    return static_cast<int>(idx);
}
#else
static inline int ctz64(uint64_t v) { return __builtin_ctzll(v); }
#endif

namespace sbf {
namespace viz {

using json = nlohmann::json;
using envelope::FrameStore;
using envelope::derive_aabb;
using envelope::derive_aabb_subdivided;

// ── helpers ─────────────────────────────────────────────────────────────────
static bool write_json(const json& j, const std::string& filepath)
{
    std::ofstream ofs(filepath);
    if (!ofs.is_open()) {
        std::cerr << "[viz] cannot open " << filepath << "\n";
        return false;
    }
    ofs << j.dump(2);
    return true;
}

static std::string hex64(uint64_t v)
{
    std::ostringstream oss;
    oss << std::hex << std::setfill('0') << std::setw(16) << v;
    return oss.str();
}

// ─── Robot FK ───────────────────────────────────────────────────────────────
void export_robot_json(const Robot& robot,
                       const std::vector<Eigen::VectorXd>& configs,
                       const std::string& filepath)
{
    json j;
    j["name"]    = robot.name();
    j["n_joints"] = robot.n_joints();

    // link radii
    json radii = json::array();
    const auto& lr = robot.link_radii();
    for (size_t i = 0; i < lr.size(); ++i) radii.push_back(lr[i]);
    j["link_radii"] = radii;

    // active link map
    json alm = json::array();
    for (int i = 0; i < robot.n_active_links(); ++i)
        alm.push_back(robot.active_link_map()[i]);
    j["active_link_map"] = alm;
    j["n_active_links"]  = robot.n_active_links();

    // joint limits
    json lim = json::array();
    for (int d = 0; d < robot.n_joints(); ++d) {
        const auto& jl = robot.joint_limits().limits;
        lim.push_back({jl[d].lo, jl[d].hi});
    }
    j["joint_limits"] = lim;

    // configs + FK positions
    json cfgs = json::array();
    for (const auto& q : configs) {
        json c;
        json qa = json::array();
        for (int d = 0; d < q.size(); ++d) qa.push_back(q[d]);
        c["q"] = qa;

        auto positions = fk_link_positions(robot, q);
        json pos = json::array();
        for (const auto& p : positions)
            pos.push_back({p.x(), p.y(), p.z()});
        c["link_positions"] = pos;

        cfgs.push_back(c);
    }
    j["configs"] = cfgs;

    write_json(j, filepath);
}

void export_robot_json(const Robot& robot,
                       const Eigen::VectorXd& config,
                       const std::string& filepath)
{
    export_robot_json(robot, std::vector<Eigen::VectorXd>{config}, filepath);
}

// ─── Envelope ───────────────────────────────────────────────────────────────
void export_envelope_json(const Robot& robot,
                          const FrameStore& store,
                          const std::vector<int>& node_indices,
                          int n_sub,
                          const std::string& filepath)
{
    json j;
    j["n_active_links"] = store.n_active_links();
    json radii = json::array();
    for (int i = 0; i < store.n_active_links(); ++i)
        radii.push_back(store.link_radii()[i]);
    j["link_radii"] = radii;
    j["n_sub"]      = n_sub;

    json nodes = json::array();
    for (int ni : node_indices) {
        if (!store.has_frames(ni)) continue;

        json nd;
        nd["node_idx"] = ni;

        const float* frames = store.get_frames(ni);
        int n_frames        = store.n_frames();
        int n_active        = store.n_active_links();
        const int* alm      = store.active_link_map();
        const float* lr     = store.link_radii();
        const float* bp     = store.base_pos();

        // Full link AABBs
        std::vector<float> aabbs(n_active * 6);
        derive_aabb(frames, n_frames, alm, n_active, lr, bp, aabbs.data());

        json laabbs = json::array();
        for (int a = 0; a < n_active; ++a) {
            json ab;
            ab["link"] = alm[a];
            ab["lo"]   = {aabbs[a*6+0], aabbs[a*6+1], aabbs[a*6+2]};
            ab["hi"]   = {aabbs[a*6+3], aabbs[a*6+4], aabbs[a*6+5]};
            laabbs.push_back(ab);
        }
        nd["link_aabbs"] = laabbs;

        // Sub-AABBs per link
        json saabbs = json::array();
        for (int a = 0; a < n_active; ++a) {
            int link_idx = alm[a];
            int parent_frame_idx = (link_idx == 0) ? -1 : link_idx - 1;
            int link_frame_idx   = link_idx;

            std::vector<float> sub(n_sub * 6);
            derive_aabb_subdivided(frames, n_frames,
                                   parent_frame_idx, link_frame_idx,
                                   n_sub, lr[a], bp, sub.data());

            for (int s = 0; s < n_sub; ++s) {
                json sb;
                sb["link"] = link_idx;
                sb["seg"]  = s;
                sb["lo"]   = {sub[s*6+0], sub[s*6+1], sub[s*6+2]};
                sb["hi"]   = {sub[s*6+3], sub[s*6+4], sub[s*6+5]};
                saabbs.push_back(sb);
            }
        }
        nd["sub_aabbs"] = saabbs;

        nodes.push_back(nd);
    }
    j["nodes"] = nodes;

    write_json(j, filepath);
}

void export_envelope_from_boxes_json(
    const Robot& robot,
    const std::vector<std::vector<Interval>>& boxes,
    int n_sub,
    const std::string& filepath)
{
    FrameStore store(robot, static_cast<int>(boxes.size()) + 1);

    json j;
    j["n_active_links"] = store.n_active_links();
    json radii = json::array();
    for (int i = 0; i < store.n_active_links(); ++i)
        radii.push_back(store.link_radii()[i]);
    j["link_radii"] = radii;
    j["n_sub"]      = n_sub;

    json nodes = json::array();
    for (size_t bi = 0; bi < boxes.size(); ++bi) {
        const auto& ivs = boxes[bi];

        // Compute FK
        FKState fk = compute_fk_full(robot, ivs);
        int node_idx = static_cast<int>(bi);
        store.store_from_fk(node_idx, fk);

        json nd;
        nd["node_idx"] = node_idx;

        // Box intervals
        json bivs = json::array();
        for (const auto& iv : ivs)
            bivs.push_back({iv.lo, iv.hi});
        nd["box_intervals"] = bivs;

        const float* frames = store.get_frames(node_idx);
        int n_frames        = store.n_frames();
        int n_active        = store.n_active_links();
        const int* alm      = store.active_link_map();
        const float* lr     = store.link_radii();
        const float* bp     = store.base_pos();

        // Full link AABBs
        std::vector<float> aabbs(n_active * 6);
        derive_aabb(frames, n_frames, alm, n_active, lr, bp, aabbs.data());

        json laabbs = json::array();
        for (int a = 0; a < n_active; ++a) {
            json ab;
            ab["link"] = alm[a];
            ab["lo"]   = {aabbs[a*6+0], aabbs[a*6+1], aabbs[a*6+2]};
            ab["hi"]   = {aabbs[a*6+3], aabbs[a*6+4], aabbs[a*6+5]};
            laabbs.push_back(ab);
        }
        nd["link_aabbs"] = laabbs;

        // Sub-AABBs
        json saabbs = json::array();
        for (int a = 0; a < n_active; ++a) {
            int link_idx = alm[a];
            int parent_frame_idx = (link_idx == 0) ? -1 : link_idx - 1;
            int link_frame_idx   = link_idx;

            std::vector<float> sub(n_sub * 6);
            derive_aabb_subdivided(frames, n_frames,
                                   parent_frame_idx, link_frame_idx,
                                   n_sub, lr[a], bp, sub.data());
            for (int s = 0; s < n_sub; ++s) {
                json sb;
                sb["link"] = link_idx;
                sb["seg"]  = s;
                sb["lo"]   = {sub[s*6+0], sub[s*6+1], sub[s*6+2]};
                sb["hi"]   = {sub[s*6+3], sub[s*6+4], sub[s*6+5]};
                saabbs.push_back(sb);
            }
        }
        nd["sub_aabbs"] = saabbs;

        nodes.push_back(nd);
    }
    j["nodes"] = nodes;

    write_json(j, filepath);
}

// ─── Voxel Grid ─────────────────────────────────────────────────────────────
void export_voxel_json(const voxel::VoxelGrid& grid,
                       const std::string& filepath)
{
    json j;
    j["delta"]      = grid.delta();
    j["safety_pad"] = grid.safety_pad();
    j["n_bricks"]   = grid.num_bricks();
    j["total_occupied"] = grid.count_occupied();

    json bricks = json::array();
    for (const auto& [coord, brick] : grid.bricks()) {
        json bj;
        bj["coord"]    = {coord.bx, coord.by, coord.bz};
        bj["popcount"] = brick.popcount();
        json words = json::array();
        for (int z = 0; z < 8; ++z)
            words.push_back(hex64(brick.words[z]));
        bj["words"] = words;
        bricks.push_back(bj);
    }
    j["bricks"] = bricks;

    write_json(j, filepath);
}

void export_voxel_centres_json(const voxel::VoxelGrid& grid,
                               const std::string& filepath)
{
    json j;
    double delta = grid.delta();
    j["delta"]      = delta;
    j["n_occupied"] = grid.count_occupied();

    json centres = json::array();
    for (const auto& [coord, brick] : grid.bricks()) {
        for (int z = 0; z < 8; ++z) {
            uint64_t w = brick.words[z];
            while (w) {
                int bit = ctz64(w);
                w &= w - 1;
                int ly = bit / 8;
                int lx = bit % 8;
                int lz = z;

                int gx = coord.bx * 8 + lx;
                int gy = coord.by * 8 + ly;
                int gz = coord.bz * 8 + lz;

                double cx = (gx + 0.5) * delta;
                double cy = (gy + 0.5) * delta;
                double cz = (gz + 0.5) * delta;

                centres.push_back({cx, cy, cz});
            }
        }
    }
    j["centres"] = centres;

    write_json(j, filepath);
}

// ─── Scene ──────────────────────────────────────────────────────────────────
void export_scene_json(const Scene& scene,
                       const std::string& filepath)
{
    json j;
    json obs = json::array();
    for (const auto& o : scene.obstacles()) {
        json oj;
        oj["name"]       = o.name;
        oj["center"]     = {o.center.x(), o.center.y(), o.center.z()};
        oj["half_sizes"] = {o.half_sizes.x(), o.half_sizes.y(), o.half_sizes.z()};
        obs.push_back(oj);
    }
    j["obstacles"] = obs;

    write_json(j, filepath);
}

// ─── Combined Snapshot ──────────────────────────────────────────────────────
void export_snapshot_json(const Robot& robot,
                          const Eigen::VectorXd& config,
                          const std::vector<Interval>& box_intervals,
                          const voxel::VoxelGrid& robot_grid,
                          const voxel::VoxelGrid& obstacle_grid,
                          const Scene& scene,
                          int n_sub,
                          const std::string& filepath)
{
    json j;

    // ── Robot section ──
    {
        json rj;
        rj["name"]    = robot.name();
        rj["n_joints"] = robot.n_joints();
        json radii = json::array();
        for (const auto& r : robot.link_radii()) radii.push_back(r);
        rj["link_radii"] = radii;

        json alm = json::array();
        for (int i = 0; i < robot.n_active_links(); ++i)
            alm.push_back(robot.active_link_map()[i]);
        rj["active_link_map"] = alm;

        json qa = json::array();
        for (int d = 0; d < config.size(); ++d) qa.push_back(config[d]);
        rj["q"] = qa;

        auto positions = fk_link_positions(robot, config);
        json pos = json::array();
        for (const auto& p : positions)
            pos.push_back({p.x(), p.y(), p.z()});
        rj["link_positions"] = pos;

        j["robot"] = rj;
    }

    // ── Envelope section ──
    {
        json ej;
        json bivs = json::array();
        for (const auto& iv : box_intervals)
            bivs.push_back({iv.lo, iv.hi});
        ej["box_intervals"] = bivs;

        // Compute FK → envelopes
        FKState fk = compute_fk_full(robot, box_intervals);
        FrameStore store(robot, 2);
        store.store_from_fk(0, fk);

        const float* frames = store.get_frames(0);
        int n_frames  = store.n_frames();
        int n_active  = store.n_active_links();
        const int* am = store.active_link_map();
        const float* lr = store.link_radii();
        const float* bp = store.base_pos();

        std::vector<float> aabbs(n_active * 6);
        derive_aabb(frames, n_frames, am, n_active, lr, bp, aabbs.data());

        json laabbs = json::array();
        for (int a = 0; a < n_active; ++a) {
            json ab;
            ab["link"] = am[a];
            ab["lo"]   = {aabbs[a*6+0], aabbs[a*6+1], aabbs[a*6+2]};
            ab["hi"]   = {aabbs[a*6+3], aabbs[a*6+4], aabbs[a*6+5]};
            laabbs.push_back(ab);
        }
        ej["link_aabbs"] = laabbs;

        json saabbs = json::array();
        for (int a = 0; a < n_active; ++a) {
            int link_idx = am[a];
            int pfi = (link_idx == 0) ? -1 : link_idx - 1;
            int lfi = link_idx;
            std::vector<float> sub(n_sub * 6);
            derive_aabb_subdivided(frames, n_frames, pfi, lfi,
                                   n_sub, lr[a], bp, sub.data());
            for (int s = 0; s < n_sub; ++s) {
                json sb;
                sb["link"] = link_idx;
                sb["seg"]  = s;
                sb["lo"]   = {sub[s*6+0], sub[s*6+1], sub[s*6+2]};
                sb["hi"]   = {sub[s*6+3], sub[s*6+4], sub[s*6+5]};
                saabbs.push_back(sb);
            }
        }
        ej["sub_aabbs"] = saabbs;

        j["envelope"] = ej;
    }

    // ── Voxel section (robot) ──
    {
        json vj;
        vj["delta"]     = robot_grid.delta();
        vj["n_bricks"]  = robot_grid.num_bricks();
        vj["n_occupied"] = robot_grid.count_occupied();
        json centres = json::array();
        double delta = robot_grid.delta();
        for (const auto& [coord, brick] : robot_grid.bricks()) {
            for (int z = 0; z < 8; ++z) {
                uint64_t w = brick.words[z];
                while (w) {
                    int bit = ctz64(w);
                    w &= w - 1;
                    int lx = bit % 8, ly = bit / 8;
                    double cx = (coord.bx*8 + lx + 0.5) * delta;
                    double cy = (coord.by*8 + ly + 0.5) * delta;
                    double cz = (coord.bz*8 + z  + 0.5) * delta;
                    centres.push_back({cx, cy, cz});
                }
            }
        }
        vj["centres"] = centres;
        j["robot_voxel"] = vj;
    }

    // ── Voxel section (obstacle) ──
    {
        json vj;
        vj["delta"]     = obstacle_grid.delta();
        vj["n_bricks"]  = obstacle_grid.num_bricks();
        vj["n_occupied"] = obstacle_grid.count_occupied();
        json centres = json::array();
        double delta = obstacle_grid.delta();
        for (const auto& [coord, brick] : obstacle_grid.bricks()) {
            for (int z = 0; z < 8; ++z) {
                uint64_t w = brick.words[z];
                while (w) {
                    int bit = ctz64(w);
                    w &= w - 1;
                    int lx = bit % 8, ly = bit / 8;
                    double cx = (coord.bx*8 + lx + 0.5) * delta;
                    double cy = (coord.by*8 + ly + 0.5) * delta;
                    double cz = (coord.bz*8 + z  + 0.5) * delta;
                    centres.push_back({cx, cy, cz});
                }
            }
        }
        vj["centres"] = centres;
        j["obstacle_voxel"] = vj;
    }

    // ── Scene section ──
    {
        json sj;
        json obs = json::array();
        for (const auto& o : scene.obstacles()) {
            json oj;
            oj["name"]       = o.name;
            oj["center"]     = {o.center.x(), o.center.y(), o.center.z()};
            oj["half_sizes"] = {o.half_sizes.x(), o.half_sizes.y(), o.half_sizes.z()};
            obs.push_back(oj);
        }
        sj["obstacles"] = obs;
        j["scene"] = sj;
    }

    write_json(j, filepath);
}

// ─── Multi-Method Envelope Comparison ───────────────────────────────────────
static json grid_to_centres_json(const voxel::VoxelGrid& grid)
{
    json centres = json::array();
    double delta = grid.delta();
    for (const auto& [coord, brick] : grid.bricks()) {
        for (int z = 0; z < 8; ++z) {
            uint64_t w = brick.words[z];
            while (w) {
                int bit = ctz64(w);
                w &= w - 1;
                int lx = bit % 8, ly = bit / 8;
                double cx = (coord.bx * 8 + lx + 0.5) * delta;
                double cy = (coord.by * 8 + ly + 0.5) * delta;
                double cz = (coord.bz * 8 + z  + 0.5) * delta;
                centres.push_back({cx, cy, cz});
            }
        }
    }
    return centres;
}

void export_envelope_comparison_json(
    const Robot& robot,
    const std::vector<Interval>& box_intervals,
    int n_sub,
    double delta,
    const std::string& filepath)
{
    json j;
    j["robot_name"]     = robot.name();
    j["n_joints"]       = robot.n_joints();
    j["n_active_links"] = robot.n_active_links();
    j["delta"]          = delta;

    // Link radii
    json radii = json::array();
    for (const auto& r : robot.link_radii()) radii.push_back(r);
    j["link_radii"] = radii;

    // Box intervals
    json bivs = json::array();
    for (const auto& iv : box_intervals)
        bivs.push_back({iv.lo, iv.hi});
    j["box_intervals"] = bivs;

    // Compute FK
    FKState fk = compute_fk_full(robot, box_intervals);

    // Robot arm at box center
    {
        Eigen::VectorXd q_center(robot.n_joints());
        for (int d = 0; d < robot.n_joints(); ++d)
            q_center[d] = box_intervals[d].center();
        auto positions = fk_link_positions(robot, q_center);
        json pos = json::array();
        for (const auto& p : positions)
            pos.push_back({p.x(), p.y(), p.z()});
        j["robot_arm"] = {{"link_positions", pos}};
    }

    // FrameStore for AABB derivation
    FrameStore store(robot, 2);
    store.store_from_fk(0, fk);

    const float* frames = store.get_frames(0);
    int n_frames  = store.n_frames();
    int n_active  = store.n_active_links();
    const int* am = store.active_link_map();
    const float* lr = store.link_radii();
    const float* bp = store.base_pos();

    json methods = json::object();

    // ── Method 1: Full AABB per link ──
    {
        std::vector<float> aabbs(n_active * 6);
        derive_aabb(frames, n_frames, am, n_active, lr, bp, aabbs.data());

        json aabb_list = json::array();
        for (int a = 0; a < n_active; ++a) {
            json ab;
            ab["link"] = am[a];
            ab["lo"]   = {aabbs[a*6+0], aabbs[a*6+1], aabbs[a*6+2]};
            ab["hi"]   = {aabbs[a*6+3], aabbs[a*6+4], aabbs[a*6+5]};
            aabb_list.push_back(ab);
        }
        methods["full_aabb"] = {
            {"type", "aabb"}, {"label", "Full AABB"},
            {"n_aabbs", n_active}, {"aabbs", aabb_list}
        };
    }

    // ── Method 2: Subdivided Sub-AABB ──
    {
        json aabb_list = json::array();
        int total = 0;
        for (int a = 0; a < n_active; ++a) {
            int link_idx = am[a];
            int pfi = (link_idx == 0) ? -1 : link_idx - 1;
            int lfi = link_idx;
            std::vector<float> sub(n_sub * 6);
            derive_aabb_subdivided(frames, n_frames, pfi, lfi,
                                   n_sub, lr[a], bp, sub.data());
            for (int s = 0; s < n_sub; ++s) {
                json sb;
                sb["link"] = link_idx;
                sb["seg"]  = s;
                sb["lo"]   = {sub[s*6+0], sub[s*6+1], sub[s*6+2]};
                sb["hi"]   = {sub[s*6+3], sub[s*6+4], sub[s*6+5]};
                aabb_list.push_back(sb);
            }
            total += n_sub;
        }
        methods["sub_aabb"] = {
            {"type", "aabb"}, {"label", "Sub-AABB (n_sub=" + std::to_string(n_sub) + ")"},
            {"n_sub", n_sub}, {"n_aabbs", total}, {"aabbs", aabb_list}
        };
    }

    // Note: use safety_pad=0 for pure-geometry comparison (no collision margin).
    // The default √3·Δ/2 safety pad would inflate Hull-16 unfairly since
    // fill_aabb (Sub-AABB / Full-AABB) does not apply safety_pad.
    const double pad = 0.0;

    // ── Method 3: Voxel Hull-16 ──
    {
        voxel::VoxelGrid grid(delta, 0, 0, 0, pad);
        voxel::rasterise_robot_hull16(robot, fk, grid, n_sub);
        methods["voxel_hull16"] = {
            {"type", "voxel"}, {"label", "Voxel Hull-16"},
            {"delta", delta},
            {"n_occupied", grid.count_occupied()},
            {"n_bricks", grid.num_bricks()},
            {"centres", grid_to_centres_json(grid)}
        };
    }

    // ── Method 4: Voxel Sub-AABB ──
    {
        voxel::VoxelGrid grid(delta, 0, 0, 0, pad);
        voxel::rasterise_robot_sub_aabbs(robot, fk, grid, n_sub);
        methods["voxel_sub_aabb"] = {
            {"type", "voxel"}, {"label", "Voxel Sub-AABB"},
            {"delta", delta},
            {"n_occupied", grid.count_occupied()},
            {"n_bricks", grid.num_bricks()},
            {"centres", grid_to_centres_json(grid)}
        };
    }

    // ── Method 5: Voxel Full-AABB ──
    {
        voxel::VoxelGrid grid(delta, 0, 0, 0, pad);
        voxel::rasterise_robot_aabbs(robot, fk, grid);
        methods["voxel_full_aabb"] = {
            {"type", "voxel"}, {"label", "Voxel Full-AABB"},
            {"delta", delta},
            {"n_occupied", grid.count_occupied()},
            {"n_bricks", grid.num_bricks()},
            {"centres", grid_to_centres_json(grid)}
        };
    }

    // ── Method 6: Analytical Critical Sub-AABB ──
    {
        const int n_act = robot.n_active_links();
        const int total = n_act * n_sub;
        std::vector<float> aabbs_a(total * 6);
        envelope::AnalyticalCriticalConfig cfg =
            envelope::AnalyticalCriticalConfig::all_enabled();
        envelope::AnalyticalCriticalStats stats;
        envelope::derive_aabb_critical_analytical(
            robot, box_intervals, n_sub, cfg, aabbs_a.data(), &stats);

        const int* alm = robot.active_link_map();
        json aabb_list = json::array();
        int idx = 0;
        for (int a = 0; a < n_act; ++a) {
            for (int s = 0; s < n_sub; ++s) {
                json ab;
                ab["link"] = alm[a];
                ab["seg"]  = s;
                ab["lo"] = {aabbs_a[idx*6+0], aabbs_a[idx*6+1], aabbs_a[idx*6+2]};
                ab["hi"] = {aabbs_a[idx*6+3], aabbs_a[idx*6+4], aabbs_a[idx*6+5]};
                aabb_list.push_back(ab);
                ++idx;
            }
        }
        methods["analytical_sub_aabb"] = {
            {"type", "aabb"},
            {"label", "Analytical Sub-AABB (n_sub=" + std::to_string(n_sub) + ")"},
            {"n_sub", n_sub}, {"n_aabbs", total}, {"aabbs", aabb_list}
        };
    }

    // ── Method 7: Voxel Analytical Sub-AABB ──
    {
        const int n_act = robot.n_active_links();
        const int total = n_act * n_sub;
        std::vector<float> aabbs_a(total * 6);
        envelope::AnalyticalCriticalConfig cfg =
            envelope::AnalyticalCriticalConfig::all_enabled();
        envelope::derive_aabb_critical_analytical(
            robot, box_intervals, n_sub, cfg, aabbs_a.data());

        voxel::VoxelGrid grid(delta, 0, 0, 0, pad);
        for (int k = 0; k < total; ++k)
            grid.fill_aabb(aabbs_a.data() + k * 6);

        methods["voxel_analytical"] = {
            {"type", "voxel"}, {"label", "Voxel Analytical"},
            {"delta", delta},
            {"n_occupied", grid.count_occupied()},
            {"n_bricks", grid.num_bricks()},
            {"centres", grid_to_centres_json(grid)}
        };
    }

    // ── Method 8: Hull-16 ∩ Analytical (tightest possible) ──
    //
    // Rationale: Hull-16 captures coordinate correlation (tight convex hull),
    // Analytical captures coordinate-independent optimal bounds.
    // Their voxel-level intersection is tighter than either alone.
    {
        // Build Hull-16 grid
        voxel::VoxelGrid hull_grid(delta, 0, 0, 0, pad);
        voxel::rasterise_robot_hull16(robot, fk, hull_grid, n_sub);

        // Build Analytical AABB grid
        const int n_act = robot.n_active_links();
        const int total = n_act * n_sub;
        std::vector<float> aabbs_a(total * 6);
        envelope::AnalyticalCriticalConfig cfg =
            envelope::AnalyticalCriticalConfig::all_enabled();
        envelope::derive_aabb_critical_analytical(
            robot, box_intervals, n_sub, cfg, aabbs_a.data());

        voxel::VoxelGrid anal_grid(delta, 0, 0, 0, pad);
        for (int k = 0; k < total; ++k)
            anal_grid.fill_aabb(aabbs_a.data() + k * 6);

        // Intersect: keep only voxels present in BOTH
        hull_grid.intersect_inplace(anal_grid);

        methods["voxel_hull16_analytical"] = {
            {"type", "voxel"},
            {"label", "Hull-16 AND Analytical"},
            {"delta", delta},
            {"n_occupied", hull_grid.count_occupied()},
            {"n_bricks", hull_grid.num_bricks()},
            {"centres", grid_to_centres_json(hull_grid)}
        };
    }

    j["methods"] = methods;

    write_json(j, filepath);
}

} // namespace viz
} // namespace sbf
