/**
 * marcucci_scenes.h — Marcucci et al. (Science Robotics, 2024) 基准场景 (v5)
 *
 * 三个标准场景: shelves (hard), bins (medium), table (easy)
 * 以及预定义的 iiwa14 query pairs.
 *
 * v5 Obstacle: float bounds[6] = {lo_x, lo_y, lo_z, hi_x, hi_y, hi_z}
 * 从 v1 (center + half_sizes) 转换而来.
 *
 * 所有数据来自 gcs-science-robotics prm_comparison notebook.
 */
#pragma once

#include <sbf/core/types.h>
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <random>
#include <string>
#include <vector>

namespace sbf {

// ─── Predefined iiwa14 Configurations ────────────────────────────────────────

inline Eigen::VectorXd config_C() {    // Center
    Eigen::VectorXd q(7);
    q << 0.0, 0.2, 0.0, -2.09, 0.0, -0.3, M_PI / 2;
    return q;
}
inline Eigen::VectorXd config_L() {    // Left
    Eigen::VectorXd q(7);
    q << 0.8, 0.7, 0.0, -1.6, 0.0, 0.0, M_PI / 2;
    return q;
}
inline Eigen::VectorXd config_R() {    // Right
    Eigen::VectorXd q(7);
    q << -0.8, 0.7, 0.0, -1.6, 0.0, 0.0, M_PI / 2;
    return q;
}

// IK milestones — from iiwa14_spheres_collision_welded_gripper.yaml
inline Eigen::VectorXd config_AS() {   // Above Shelf
    Eigen::VectorXd q(7);
    q << 6.42e-05, 0.4719533, -0.0001493, -0.6716735, 0.0001854, 0.4261696, 1.5706922;
    return q;
}
inline Eigen::VectorXd config_TS() {   // Top Shelf
    Eigen::VectorXd q(7);
    q << -1.55e-04, 0.3972726, 0.0002196, -1.3674756, 0.0002472, -0.1929518, 1.5704688;
    return q;
}
inline Eigen::VectorXd config_CS() {   // Center Shelf
    Eigen::VectorXd q(7);
    q << -1.76e-04, 0.6830279, 0.0002450, -1.6478229, 2.09e-05, -0.7590545, 1.5706263;
    return q;
}
inline Eigen::VectorXd config_LB() {   // Left Bin
    Eigen::VectorXd q(7);
    q << 1.3326656, 0.7865932, 0.3623384, -1.4916529, -0.3192509, 0.9217325, 1.7911904;
    return q;
}
inline Eigen::VectorXd config_RB() {   // Right Bin
    Eigen::VectorXd q(7);
    q << -1.3324624, 0.7866478, -0.3626562, -1.4916528, 0.3195340, 0.9217833, 1.3502090;
    return q;
}

// ─── Query Pair ──────────────────────────────────────────────────────────────

struct QueryPair {
    std::string label;
    Eigen::VectorXd start;
    Eigen::VectorXd goal;
};

// ─── Helper: make AABB from center + half-extents ────────────────────────────

inline Obstacle make_aabb(double cx, double cy, double cz,
                          double hx, double hy, double hz) {
    return Obstacle(
        static_cast<float>(cx - hx), static_cast<float>(cy - hy), static_cast<float>(cz - hz),
        static_cast<float>(cx + hx), static_cast<float>(cy + hy), static_cast<float>(cz + hz));
}

// ─── Shelves Scene (5 obstacles) ─────────────────────────────────────────────
// Narrow shelf structure at origin [0.85, 0, 0.4]

inline std::vector<Obstacle> make_shelves_obstacles() {
    const double ox = 0.85, oy = 0.0, oz = 0.4;
    std::vector<Obstacle> obs;

    // add(local_pos, full_size) → center = origin + local_pos, half = full_size/2
    auto add = [&](double lx, double ly, double lz,
                   double fx, double fy, double fz) {
        obs.push_back(make_aabb(ox + lx, oy + ly, oz + lz,
                                fx / 2, fy / 2, fz / 2));
    };

    add( 0,  0.292,  0,       0.3,  0.016, 0.783); // right_wall
    add( 0, -0.292,  0,       0.3,  0.016, 0.783); // left_wall
    add( 0,  0,      0.3995,  0.3,  0.6,   0.016); // top
    add( 0,  0,     -0.13115, 0.3,  0.6,   0.016); // shelf_lower
    add( 0,  0,      0.13115, 0.3,  0.6,   0.016); // shelf_upper

    return obs;
}

// ─── Bins Scene (10 obstacles) ───────────────────────────────────────────────
// Two bins (left & right), each with 5 faces. 90° yaw rotation.
// yaw=90°: body (lx,ly,lz) → world (−ly, lx, lz)
//          size  (fx,fy,fz) → world (fy, fx, fz)

inline std::vector<Obstacle> make_bins_obstacles() {
    std::vector<Obstacle> obs;

    auto add_bin = [&](double bx, double by, double bz) {
        // face(local_x, local_y, local_z, full_size_x, full_size_y, full_size_z)
        // world center = (bx - ly, by + lx, bz + lz)
        // world half   = (fy/2, fx/2, fz/2)
        auto add = [&](double lx, double ly, double lz,
                       double fx, double fy, double fz) {
            obs.push_back(make_aabb(bx - ly, by + lx, bz + lz,
                                    fy / 2, fx / 2, fz / 2));
        };

        add( 0.22,  0,     0.105,  0.05, 0.63, 0.21);  // front
        add(-0.22,  0,     0.105,  0.05, 0.63, 0.21);  // back
        add( 0,     0.29,  0.105,  0.49, 0.05, 0.21);  // left
        add( 0,    -0.29,  0.105,  0.49, 0.05, 0.21);  // right
        add( 0,     0,     0.0075, 0.49, 0.63, 0.015); // bottom
    };

    add_bin(0, -0.6, 0);  // binR
    add_bin(0,  0.6, 0);  // binL

    return obs;
}

// ─── Table Scene (1 obstacle) ────────────────────────────────────────────────

inline std::vector<Obstacle> make_table_obstacles() {
    // origin (0.4, 0, 0), table_top local (0, 0, -0.25), size 2.5×2.5×0.2
    return { make_aabb(0.4, 0.0, -0.25,  2.5 / 2, 2.5 / 2, 0.2 / 2) };
}

// ─── Combined Scene (16 obstacles) ───────────────────────────────────────────

inline std::vector<Obstacle> make_combined_obstacles() {
    std::vector<Obstacle> obs;
    for (auto& o : make_shelves_obstacles()) obs.push_back(o);
    for (auto& o : make_bins_obstacles())    obs.push_back(o);
    for (auto& o : make_table_obstacles())   obs.push_back(o);
    return obs;
}

// ─── Scaled-obstacle Scene (8 / 16 / 32) ─────────────────────────────────────
// Deterministic scaling of the Marcucci combined scene for the obstacle
// scaling row of tab:scale. n=16 reproduces make_combined_obstacles().
// n<16: take the first n obstacles in canonical order (shelves→bins→table).
// n>16: extend with procedurally placed 4cm cubes inside the workspace box
//       [0.40,1.20]×[-0.60,0.60]×[0.20,1.00], using a fixed RNG (seed=0xC0DE)
//       and rejecting cubes that intersect any existing obstacle.
inline std::vector<Obstacle> make_scaled_obstacles(int n) {
    auto base = make_combined_obstacles();
    if (n <= (int)base.size()) {
        return std::vector<Obstacle>(base.begin(), base.begin() + std::max(1, n));
    }
    // Procedural extension up to n.
    std::vector<Obstacle> obs = base;
    auto intersects = [](const Obstacle& a, const Obstacle& b) {
        // bounds = [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
        return !(a.bounds[0] > b.bounds[3] || a.bounds[3] < b.bounds[0] ||
                 a.bounds[1] > b.bounds[4] || a.bounds[4] < b.bounds[1] ||
                 a.bounds[2] > b.bounds[5] || a.bounds[5] < b.bounds[2]);
    };
    std::mt19937 rng(0xC0DE);
    std::uniform_real_distribution<double> Ux(0.40, 1.20);
    std::uniform_real_distribution<double> Uy(-0.60, 0.60);
    std::uniform_real_distribution<double> Uz(0.20, 1.00);
    int needed = n - (int)obs.size();
    int attempts = 0, kMaxAttempts = 50000;
    while (needed > 0 && attempts++ < kMaxAttempts) {
        double cx = Ux(rng), cy = Uy(rng), cz = Uz(rng);
        Obstacle cand = make_aabb(cx, cy, cz, 0.04, 0.04, 0.04);
        bool clash = false;
        for (auto& o : obs) if (intersects(cand, o)) { clash = true; break; }
        if (!clash) { obs.push_back(cand); --needed; }
    }
    return obs;
}

// ─── Query Pairs ─────────────────────────────────────────────────────────────

inline std::vector<QueryPair> make_shelves_queries() {
    return {
        {"LB->TS", config_LB(), config_TS()},
        {"RB->CS", config_RB(), config_CS()},
        {"LB->RB", config_LB(), config_RB()},
        {"C->CS",  config_C(),  config_CS()},
        {"AS->LB", config_AS(), config_LB()},
        {"TS->RB", config_TS(), config_RB()},
    };
}

inline std::vector<QueryPair> make_bins_queries() {
    return {
        {"LB->RB", config_LB(), config_RB()},
        {"C->LB",  config_C(),  config_LB()},
        {"C->RB",  config_C(),  config_RB()},
        {"L->RB",  config_L(),  config_RB()},
        {"R->LB",  config_R(),  config_LB()},
        {"L->R",   config_L(),  config_R()},
    };
}

inline std::vector<QueryPair> make_table_queries() {
    return {
        {"L->R",   config_L(),  config_R()},
        {"C->L",   config_C(),  config_L()},
        {"C->R",   config_C(),  config_R()},
        {"AS->C",  config_AS(), config_C()},
        {"L->AS",  config_L(),  config_AS()},
        {"R->AS",  config_R(),  config_AS()},
    };
}

// Combined: 5 canonical pairs (跨区域循环)
// AS→TS→CS→LB→RB→AS  (与 prm_comparison notebook tasks_for_paper 一致)
inline std::vector<QueryPair> make_combined_queries() {
    return {
        {"AS->TS", config_AS(), config_TS()},
        {"TS->CS", config_TS(), config_CS()},
        {"CS->LB", config_CS(), config_LB()},
        {"LB->RB", config_LB(), config_RB()},
        {"RB->AS", config_RB(), config_AS()},
    };
}

// Extended: deterministic enumeration of ordered (start,goal) pairs over the
// 8 named anchors {AS,TS,CS,C,L,R,LB,RB}. 56 pairs total. Used by the ≥20
// query benchmark (D-task). Returns the first N pairs.
inline std::vector<QueryPair> make_extended_queries(int N) {
    using F = Eigen::VectorXd(*)();
    struct A { const char* name; F fn; };
    static const A anchors[] = {
        {"AS", &config_AS}, {"TS", &config_TS}, {"CS", &config_CS},
        {"C",  &config_C},  {"L",  &config_L},  {"R",  &config_R},
        {"LB", &config_LB}, {"RB", &config_RB},
    };
    constexpr int K = sizeof(anchors)/sizeof(anchors[0]);
    std::vector<QueryPair> out;
    for (int i = 0; i < K && (int)out.size() < N; ++i) {
        for (int j = 0; j < K && (int)out.size() < N; ++j) {
            if (i == j) continue;
            std::string nm = std::string(anchors[i].name) + "->" + anchors[j].name;
            out.push_back({nm, anchors[i].fn(), anchors[j].fn()});
        }
    }
    return out;
}

// ─── Scene Registry ──────────────────────────────────────────────────────────

struct SceneData {
    std::string name;
    std::vector<Obstacle> obstacles;
    std::vector<QueryPair> queries;
};

inline std::vector<SceneData> all_marcucci_scenes() {
    return {
        {"combined", make_combined_obstacles(), make_combined_queries()},
    };
}

}  // namespace sbf
