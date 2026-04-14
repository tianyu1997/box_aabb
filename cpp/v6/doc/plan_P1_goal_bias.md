# P1: Goal Bias 连接感知改造

## 问题

**文件**: `src/forest/grower.cpp` L1375-L1383  
**现状**: goal bias 采样时随机选一棵"其他树"作为目标：
```cpp
int gi = std::uniform_int_distribution<int>(0, n_trees - 1)(rng_);
if (gi == tree_id && n_trees > 1)
    gi = (gi + 1) % n_trees;
q_rand = multi_goals_[gi];
```
这完全忽略了连接状态——当 tree_id=0 已经和 tree_id=2 连通时，仍然会向 tree2 目标点采样。浪费了 50% 的 goal bias 采样在已连通的树对之间。

## 修改方案

将 goal bias 中目标树的选择改为**只选未连通的树**：

```cpp
// RRT sample: goal-bias toward UNCONNECTED trees or uniform random
Eigen::VectorXd q_rand;
if (has_multi_goals_ && u01(rng_) < config_.rrt_goal_bias) {
    // Collect trees not yet connected to tree_id
    std::vector<int> unconnected;
    for (int t = 0; t < n_trees; ++t)
        if (t != tree_id && !tree_uf.connected(t, tree_id))
            unconnected.push_back(t);
    if (!unconnected.empty()) {
        int gi = unconnected[std::uniform_int_distribution<int>(
            0, (int)unconnected.size() - 1)(rng_)];
        q_rand = multi_goals_[gi];
    } else {
        q_rand = sample_random();  // all connected, fall back to random
    }
} else {
    q_rand = sample_random();
}
```

## 影响评估

- **影响范围**: 仅 `grow_coordinated()` 中 goal bias 的 ~8 行
- **风险**: 低。当所有树已连通时优雅回退到随机采样
- **预期效果**: goal bias 采样 100% 指向需要连接的树，直接提升跨组件连接概率
- **评级**: ★★★★★ (最高优先级)
