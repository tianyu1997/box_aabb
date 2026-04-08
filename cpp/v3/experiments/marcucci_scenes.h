/**
 * marcucci_scenes.h — Marcucci et al. (Science Robotics, 2024) 基准场景
 *
 * 三个标准场景: shelves (hard), bins (medium), table (easy)
 * 以及预定义的 iiwa14 query pairs.
 *
 * 所有数据来自 v4/experiments/marcucci_scenes.py
 */
#pragma once

#include "sbf/common/types.h"
#include <Eigen/Core>
#include <cmath>
#include <string>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace sbf {

// ─── Predefined iiwa14 Configurations ────────────────────────────────────────
// Seed points
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

// IK milestones — corrected values computed with iiwa14_spheres_collision_welded_gripper.yaml
// q0=[0, 0.3, 0, -1.8, 0, 1, 1.57], verified PointInSet=True in IRIS regions
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

// ─── Shelves Scene ───────────────────────────────────────────────────────────
// Narrow shelf structure at origin [0.85, 0, 0.4]
inline std::vector<Obstacle> make_shelves_obstacles() {
    Eigen::Vector3d origin(0.85, 0.0, 0.4);
    std::vector<Obstacle> obs;

    auto add = [&](const std::string& name, Eigen::Vector3d local_pos,
                   double fx, double fy, double fz) {
        Obstacle o;
        o.name = name;
        o.center = origin + local_pos;
        o.half_sizes = Eigen::Vector3d(fx / 2, fy / 2, fz / 2);
        obs.push_back(o);
    };

    add("shelf_right_wall", {0, 0.292, 0},       0.3, 0.016, 0.783);
    add("shelf_left_wall",  {0, -0.292, 0},      0.3, 0.016, 0.783);
    add("shelf_top",        {0, 0, 0.3995},       0.3, 0.6, 0.016);
    add("shelf_lower",      {0, 0, -0.13115},     0.3, 0.6, 0.016);
    add("shelf_upper",      {0, 0, 0.13115},      0.3, 0.6, 0.016);

    return obs;
}

// Shelves query pairs: (LB→TS), (RB→CS), (LB→RB), (C→CS), (AS→LB), (TS→RB)
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

// ─── Bins Scene ──────────────────────────────────────────────────────────────
// Two bins (left & right), each with 5 faces. 90° yaw rotation.
// yaw=90° maps (x,y) → (-y, x): for body-frame offset (lx,ly,lz),
// world offset = (-ly, lx, lz) relative to bin origin.
inline std::vector<Obstacle> make_bins_obstacles() {
    std::vector<Obstacle> obs;

    auto add_bin = [&](const std::string& prefix, const Eigen::Vector3d& bin_origin) {
        // yaw=90°: body (lx,ly,lz) → world (−ly, lx, lz)
        // size (fx,fy,fz) → world (fy, fx, fz)
        auto add = [&](const std::string& face, double lx, double ly, double lz,
                       double fx, double fy, double fz) {
            Obstacle o;
            o.name = prefix + "_" + face;
            o.center = bin_origin + Eigen::Vector3d(-ly, lx, lz);
            o.half_sizes = Eigen::Vector3d(fy / 2, fx / 2, fz / 2);
            obs.push_back(o);
        };

        add("front",  0.22,  0,     0.105, 0.05,  0.63,  0.21);
        add("back",  -0.22,  0,     0.105, 0.05,  0.63,  0.21);
        add("left",   0,     0.29,  0.105, 0.49,  0.05,  0.21);
        add("right",  0,    -0.29,  0.105, 0.49,  0.05,  0.21);
        add("bottom", 0,     0,     0.0075,0.49,  0.63,  0.015);
    };

    add_bin("binR", {0, -0.6, 0});
    add_bin("binL", {0,  0.6, 0});

    return obs;
}

// Bins query pairs: (LB→RB), (C→LB), (C→RB), (L→RB), (R→LB), (L→R)
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

// ─── Table Scene ─────────────────────────────────────────────────────────────
// Big table below the robot
inline std::vector<Obstacle> make_table_obstacles() {
    Eigen::Vector3d origin(0.4, 0.0, 0.0);
    std::vector<Obstacle> obs;

    Obstacle o;
    o.name = "table_top";
    o.center = origin + Eigen::Vector3d(0, 0, -0.25);
    o.half_sizes = Eigen::Vector3d(2.5 / 2, 2.5 / 2, 0.2 / 2);
    obs.push_back(o);

    return obs;
}

// Table query pairs: (L→R), (C→L), (C→R), (AS→C), (L→AS), (R→AS)
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

// ─── Combined Scene ──────────────────────────────────────────────────────────
// shelves (5) + binR (5) + binL (5) + table (1) = 16 obstacles, 同 notebook
inline std::vector<Obstacle> make_combined_obstacles() {
    std::vector<Obstacle> obs;
    for (auto& o : make_shelves_obstacles()) obs.push_back(o);
    for (auto& o : make_bins_obstacles())   obs.push_back(o);
    for (auto& o : make_table_obstacles())  obs.push_back(o);
    return obs;
}

// Combined 场景 query pairs: 5 canonical pairs (跨区域循环)
// AS→TS→CS→LB→RB→AS, 与 prm_comparison notebook tasks_for_paper 一致
inline std::vector<QueryPair> make_combined_queries() {
    return {
        {"AS->TS", config_AS(), config_TS()},
        {"TS->CS", config_TS(), config_CS()},
        {"CS->LB", config_CS(), config_LB()},
        {"LB->RB", config_LB(), config_RB()},
        {"RB->AS", config_RB(), config_AS()},
    };
}

// ─── Scene Registry ──────────────────────────────────────────────────────────
struct SceneData {
    std::string name;
    std::vector<Obstacle> obstacles;
    std::vector<QueryPair> queries;
};

inline std::vector<SceneData> all_marcucci_scenes() {
    // 使用合并场景 (与 prm_comparison notebook 完全一致)
    return {
        {"combined", make_combined_obstacles(), make_combined_queries()},
    };
}

} // namespace sbf
