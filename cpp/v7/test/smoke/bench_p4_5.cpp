/// @file bench_p4_5.cpp
/// @brief Quick perf comparison: serial vs multi-thread grower on IIWA14.

#include "sbf/core/robot.h"
#include "sbf/envelope/envelope_type.h"
#include "sbf/forest/grower.h"
#include "sbf/lect/lect.h"
#include "sbf/scene/collision.h"
#include "sbf/scene/obstacle.h"

#include <chrono>
#include <cstdio>
#include <string>

namespace {
sbf::core::Robot load_iiwa() {
    return sbf::core::Robot::from_json(
        std::string(SBF_DATA_DIR) + "/iiwa14.json");
}
std::vector<sbf::core::Interval> joint_iv(const sbf::core::Robot& r) {
    std::vector<sbf::core::Interval> iv;
    for (const auto& l : r.joint_limits().limits) iv.push_back(l);
    return iv;
}
sbf::lect::LECT make_lect(const sbf::core::Robot& r,
                          const std::vector<sbf::core::Interval>& iv) {
    sbf::envelope::EnvelopeTypeConfig cfg;
    cfg.type = sbf::envelope::EnvelopeType::LinkIAABB;
    return sbf::lect::LECT(r, iv, cfg);
}

double time_grow(int n_threads, int max_boxes) {
    auto robot = load_iiwa();
    auto iv = joint_iv(robot);
    auto lect = make_lect(robot, iv);
    sbf::scene::Obstacle far(20, 20, 20, 21, 21, 21);
    float compact[6];
    sbf::scene::pack_obstacles(&far, 1, compact);

    sbf::forest::GrowerConfig cfg;
    cfg.max_boxes = max_boxes;
    cfg.timeout_ms = 30000;
    cfg.n_threads = n_threads;
    cfg.stop_after_connect = false;
    sbf::forest::ForestGrower g(robot, lect, cfg);

    Eigen::VectorXd qs = Eigen::VectorXd::Zero(robot.n_joints());
    Eigen::VectorXd qg = Eigen::VectorXd::Zero(robot.n_joints());
    qg[0] = 0.5; qg[1] = 0.3;
    g.set_endpoints(qs, qg);

    auto t0 = std::chrono::steady_clock::now();
    auto res = g.grow(compact, 1);
    auto dt = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - t0).count();

    std::printf(
        "n_threads=%d  max_boxes=%d  built=%zu  workers=%d  islands=%d  "
        "connected=%d  bridges=%d  time=%.2fms\n",
        n_threads, max_boxes, res.boxes.size(), res.n_workers_used,
        res.adjacency_islands, (int)res.start_goal_connected,
        res.n_bridge_boxes, dt);
    return dt;
}
}  // namespace

int main() {
    std::printf("=== v7 P4.5 grower bench (IIWA14, far obstacle) ===\n");
    for (int mb : {200, 500}) {
        time_grow(1, mb);
        time_grow(4, mb);
        time_grow(8, mb);
    }
    return 0;
}
