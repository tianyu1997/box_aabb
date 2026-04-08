// SafeBoxForest v4 — SBF Planner implementation (stub)
#include "sbf/planner/sbf_planner.h"

namespace sbf {
namespace planner {

SBFPlanner::SBFPlanner(const Robot& robot, const SBFPlannerConfig& config)
    : robot_(&robot), config_(config) {}

void SBFPlanner::set_scene(const Obstacle* obstacles, int n_obs) {
    obstacles_.assign(obstacles, obstacles + n_obs);
}

PathResult SBFPlanner::plan(const Eigen::VectorXd& /*start*/,
                             const Eigen::VectorXd& /*goal*/) {
    // TODO: 实现 SBF-based 路径规划
    //   1. 构建 ForestGrower + 设置 endpoints
    //   2. grow() 生成 box forest
    //   3. 在 adjacency graph 中搜索 start_box → goal_box 路径
    //   4. 提取 waypoints
    //   5. 路径后处理 (shortcut)
    return PathResult::failure();
}

const forest::ForestGrower& SBFPlanner::grower() const { return *grower_; }
const forest::AdjacencyGraph& SBFPlanner::graph() const { return grower_->graph(); }

} // namespace planner
} // namespace sbf
