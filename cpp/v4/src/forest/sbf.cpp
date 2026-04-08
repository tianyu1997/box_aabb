// SafeBoxForest v4 — SBF builder implementation (stub)
#include "sbf/forest/sbf.h"

namespace sbf {
namespace forest {

SafeBoxForest::SafeBoxForest(const Robot& robot, const SBFConfig& config)
    : robot_(&robot), config_(config) {
    // TODO: 从 v3 sbf.cpp 迁移构造函数逻辑
}

SBFResult SafeBoxForest::build(const Obstacle* /*obstacles*/, int /*n_obs*/) {
    // TODO: 从 v3 迁移 build 实现
    return SBFResult{};
}

const LECT& SafeBoxForest::lect() const { return lect_; }
LECT& SafeBoxForest::lect_mut() { return lect_; }
const std::vector<BoxNode>& SafeBoxForest::boxes() const { return boxes_; }
int SafeBoxForest::n_boxes() const { return static_cast<int>(boxes_.size()); }
const SBFConfig& SafeBoxForest::config() const { return config_; }
bool SafeBoxForest::is_grid_envelope() const { return false; }

Eigen::VectorXd SafeBoxForest::sample_random(std::mt19937_64& /*rng*/) const {
    return Eigen::VectorXd{};
}

int SafeBoxForest::promote_all(const Obstacle* /*obs*/, int /*n_obs*/) { return 0; }
bool SafeBoxForest::try_promote_node(int /*idx*/, const Obstacle* /*obs*/, int /*n_obs*/) { return false; }
bool SafeBoxForest::try_promote_aabb(int /*idx*/, const Obstacle* /*obs*/, int /*n_obs*/) { return false; }
bool SafeBoxForest::try_promote_grid(int /*idx*/, const Obstacle* /*obs*/, int /*n_obs*/) { return false; }

} // namespace forest
} // namespace sbf
