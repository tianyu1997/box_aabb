/**
 * diagnose_table.cpp — 诊断 table 场景 50% 成功率的原因
 */
#include "sbf/planner/sbf_planner.h"
#include "sbf/planner/pipeline.h"
#include "sbf/planner/graph_search.h"
#include "sbf/forest/connectivity.h"
#include "marcucci_scenes.h"
#include <chrono>
#include <iomanip>
#include <iostream>

using namespace sbf;

int main() {
    Robot robot = Robot::from_json("configs/iiwa14.json");
    auto table_obs = make_table_obstacles();
    auto table_qp = make_table_queries();
    auto& qp = table_qp[0];  // L->R

    std::cout << "Query: " << qp.label << "\n";
    std::cout << "  start (L) = " << qp.start.transpose() << "\n";
    std::cout << "  goal  (R) = " << qp.goal.transpose() << "\n\n";
    std::cout << std::fixed;

    for (int seed = 0; seed < 10; ++seed) {
        SBFConfig cfg = make_panda_config(seed);
        cfg.max_boxes = 10000;
        cfg.max_consecutive_miss = 500;

        SBFPlanner planner(robot, table_obs, cfg);
        planner.build(qp.start, qp.goal, 300.0);

        auto& forest = planner.forest();
        int nboxes = forest.n_boxes();

        // Check containment
        const BoxNode* start_box = forest.find_containing(qp.start);
        const BoxNode* goal_box  = forest.find_containing(qp.goal);

        // Count islands (BEFORE bridging in build, but build already bridges)
        std::vector<int> all_ids;
        for (auto& [id, _] : forest.boxes()) all_ids.push_back(id);
        auto islands = find_islands(forest.adjacency(), all_ids);

        // Attach
        auto start_attach = planner.connector().attach_config(qp.start);
        auto goal_attach  = planner.connector().attach_config(qp.goal);

        // Dijkstra
        std::unordered_set<int> ss{start_attach.box_id};
        std::unordered_set<int> gs{goal_attach.box_id};
        auto dijk = dijkstra_center_distance(
            forest.adjacency(), forest.boxes(), ss, gs);

        // Also try plan() for comparison
        SBFConfig cfg2 = make_panda_config(seed);
        cfg2.max_boxes = 10000;
        cfg2.max_consecutive_miss = 500;
        SBFPlanner planner2(robot, table_obs, cfg2);
        auto plan_res = planner2.plan(qp.start, qp.goal, 300.0);

        std::cout << "seed=" << seed
                  << "  boxes=" << nboxes
                  << "  islands=" << islands.size()
                  << "  start_in=" << (start_box ? "Y" : "N")
                  << "  goal_in=" << (goal_box ? "Y" : "N")
                  << "  attach_s=" << start_attach.box_id
                  << (start_attach.inside_box ? "(in)" : "(nr)")
                  << "  attach_g=" << goal_attach.box_id
                  << (goal_attach.inside_box ? "(in)" : "(nr)")
                  << "  dijk=" << (dijk.found ? "OK" : "FAIL")
                  << "  plan=" << (plan_res.success ? "OK" : "FAIL")
                  << "\n";

        if (start_attach.box_id >= 0 && goal_attach.box_id >= 0) {
            // Check if start and goal boxes are in the same island
            int start_isle = -1, goal_isle = -1;
            int start_isle_sz = 0, goal_isle_sz = 0;
            for (auto& [isle_id, members] : islands) {
                if (members.count(start_attach.box_id)) {
                    start_isle = isle_id;
                    start_isle_sz = (int)members.size();
                }
                if (members.count(goal_attach.box_id)) {
                    goal_isle = isle_id;
                    goal_isle_sz = (int)members.size();
                }
            }
            if (!dijk.found) {
                std::cout << "    start_isle=" << start_isle << "(sz=" << start_isle_sz << ")"
                          << "  goal_isle=" << goal_isle << "(sz=" << goal_isle_sz << ")"
                          << "  same=" << (start_isle == goal_isle ? "Y" : "N") << "\n";
                // Check distance between start/goal configs
                double dist = (qp.start - qp.goal).norm();
                double start_box_dist = start_box ? 0.0
                    : forest.find_nearest(qp.start)->distance_to_config(qp.start);
                double goal_box_dist = goal_box ? 0.0
                    : forest.find_nearest(qp.goal)->distance_to_config(qp.goal);
                std::cout << "    start_goal_dist=" << std::setprecision(3) << dist
                          << "  start_nearest_dist=" << start_box_dist
                          << "  goal_nearest_dist=" << goal_box_dist << "\n";
            }
        }
    }
    return 0;
}
