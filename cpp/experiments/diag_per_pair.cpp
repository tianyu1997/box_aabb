/**
 * diag_per_pair.cpp — 诊断: 对 combined 场景中每个 s-t pair 单独使用 old build()
 *
 * Phase 0: 检查每个里程碑 config 在 combined 场景中的碰撞状态
 * Phase 1: 每个 pair 使用全新的 SBFPlanner, 从 max_boxes=1000 开始逐步增加
 * 直到 connected, 或达到上限 50000. 报告连通所需的 box 数量和用时.
 *
 * 用法:
 *   ./diag_per_pair [robot.json] [--seed N]
 */

#include "sbf/planner/sbf_planner.h"
#include "sbf/forest/collision.h"
#include "sbf/aabb/fk_scalar.h"
#include "sbf/planner/pipeline.h"
#include "marcucci_scenes.h"
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_set>
#include <vector>

using namespace sbf;

// Check connectivity between start and goal via adjacency BFS
bool is_connected(const SBFPlanner& planner,
                  const Eigen::VectorXd& start,
                  const Eigen::VectorXd& goal)
{
    const auto& forest = planner.forest();
    const BoxNode* s_box = forest.find_containing(start);
    const BoxNode* g_box = forest.find_containing(goal);
    if (!s_box || !g_box) return false;
    if (s_box->id == g_box->id) return true;

    const auto& adj = forest.adjacency();
    std::unordered_set<int> visited;
    std::vector<int> queue = {s_box->id};
    visited.insert(s_box->id);

    while (!queue.empty()) {
        int cur = queue.back(); queue.pop_back();
        auto it = adj.find(cur);
        if (it == adj.end()) continue;
        for (int nb : it->second) {
            if (nb == g_box->id) return true;
            if (!visited.count(nb)) {
                visited.insert(nb);
                queue.push_back(nb);
            }
        }
    }
    return false;
}

int main(int argc, char** argv)
{
    std::string robot_path = "configs/iiwa14.json";
    int seed = 0;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--seed" && i + 1 < argc) seed = std::atoi(argv[++i]);
        else if (a[0] != '-') robot_path = a;
    }

    Robot robot = Robot::from_json(robot_path);
    std::cout << "Robot: " << robot.name()
              << "  DOF=" << robot.n_joints()
              << "  seed=" << seed << "\n\n";

    auto obstacles = make_combined_obstacles();
    auto queries   = make_combined_queries();

    // ══════════════════════════════════════════════════════
    // Phase 0: 碰撞状态检查 + per-link 详细分析
    // ══════════════════════════════════════════════════════
    {
        CollisionChecker checker(robot, obstacles);
        struct Cfg { std::string name; Eigen::VectorXd q; };
        std::vector<Cfg> cfgs = {
            {"AS", config_AS()}, {"TS", config_TS()}, {"CS", config_CS()},
            {"LB", config_LB()}, {"RB", config_RB()},
        };
        const double* radii = robot.active_link_radii();
        int n_links = robot.n_active_links();

        std::cout << "=== Phase 0: milestone collision check (combined, "
                  << obstacles.size() << " obs) ===\n";
        for (auto& c : cfgs) {
            bool col = checker.check_config(c.q);
            std::cout << "  " << std::setw(4) << c.name
                      << ": " << (col ? "COLLISION" : "free") << "\n";

            if (col) {
                // Per-link AABB analysis
                auto positions = fk_link_positions(robot, c.q);
                std::cout << "    link radii: [";
                for (int i = 0; i < n_links; ++i)
                    std::cout << (radii ? radii[i] : 0.0) << (i+1<n_links?",":"");
                std::cout << "]\n";

                for (int ci = 0; ci < n_links; ++ci) {
                    int ls = robot.active_link_map()[ci];
                    int le = ls + 1;
                    if (le >= static_cast<int>(positions.size())) continue;

                    Eigen::Vector3d lo = positions[ls].cwiseMin(positions[le]);
                    Eigen::Vector3d hi = positions[ls].cwiseMax(positions[le]);
                    double r = radii ? radii[ci] : 0.0;
                    lo.array() -= r;
                    hi.array() += r;

                    // Check vs each obstacle
                    for (auto& obs : obstacles) {
                        auto obs_lo = obs.lo();
                        auto obs_hi = obs.hi();
                        // Overlap in each axis
                        double ox = std::min(hi.x(), obs_hi.x()) - std::max(lo.x(), obs_lo.x());
                        double oy = std::min(hi.y(), obs_hi.y()) - std::max(lo.y(), obs_lo.y());
                        double oz = std::min(hi.z(), obs_hi.z()) - std::max(lo.z(), obs_lo.z());
                        if (ox > 0 && oy > 0 && oz > 0) {
                            double pen = std::min({ox, oy, oz});
                            std::cout << "    !! link" << ci
                                      << " (r=" << r << ")"
                                      << " vs " << obs.name
                                      << "  pen=" << std::fixed << std::setprecision(4) << pen
                                      << "  overlap=(" << ox << "," << oy << "," << oz << ")\n";
                        }
                    }
                }
            }
        }
        std::cout << "\n";

        // Phase 0b: try with reduced radii to see threshold
        std::cout << "  --- testing reduced ffb_min_edge=0.005 anchor creation ---\n";
        for (auto& c : {"LB", "RB"}) {
            Eigen::VectorXd q = (std::string(c) == "LB") ? config_LB() : config_RB();
            SBFConfig cfg0 = make_panda_config(seed);
            cfg0.max_boxes = 50;
            cfg0.ffb_min_edge = 0.005;
            SBFPlanner pl0(robot, obstacles, cfg0);
            // Try building just 50 boxes from this seed as start
            pl0.build(q, q, 5.0);
            bool anchor_ok = pl0.forest().find_containing(q) != nullptr;
            std::cout << "  " << c << " anchor with ffb_min_edge=0.005: "
                      << (anchor_ok ? "OK" : "FAILED") << "\n";
        }
        std::cout << "\n";
    }

    // Box budgets to try: escalating
    std::vector<int> budgets = {500, 1000, 2000, 5000, 10000, 20000};

    std::cout << "=== Phase 1: per-pair build() connectivity ===\n";
    std::cout << std::setw(12) << "pair"
              << std::setw(10) << "budget"
              << std::setw(12) << "n_boxes"
              << std::setw(12) << "connected"
              << std::setw(10) << "time(s)"
              << "\n"
              << std::string(56, '-') << "\n";

    std::cout << std::fixed;

    for (int pi = 0; pi < static_cast<int>(queries.size()); ++pi) {
        const auto& qp = queries[pi];
        bool found = false;
        for (int bud : budgets) {
            SBFConfig cfg = make_panda_config(seed);
            cfg.max_boxes = bud;
            cfg.max_consecutive_miss = 500;

            auto t0 = std::chrono::steady_clock::now();
            SBFPlanner planner(robot, obstacles, cfg);
            planner.build(qp.start, qp.goal, 120.0);
            double dt = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t0).count();

            bool conn = is_connected(planner, qp.start, qp.goal);
            int nb    = planner.forest().n_boxes();

            std::cout << std::setw(12) << qp.label
                      << std::setw(10) << bud
                      << std::setw(12) << nb
                      << std::setw(12) << (conn ? "YES" : "no")
                      << std::setw(10) << std::setprecision(2) << dt
                      << "\n";

            if (conn) { found = true; break; }
        }
        if (!found)
            std::cout << "  !! " << qp.label << " NEVER connected within budget\n";
        std::cout << "\n";
    }

    return 0;
}
