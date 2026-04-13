# P2: 连接驱动调度改造

## 问题

**文件**: `src/forest/grower.cpp` L1357-L1370  
**现状**: 树选择策略是 70% 选 box 数最少的树，30% 随机：
```cpp
if (u01(rng_) < 0.7) {
    int min_cnt = tree_box_count[0];
    for (int t = 1; t < n_trees; ++t) {
        if (tree_box_count[t] < min_cnt) {
            min_cnt = tree_box_count[t];
            tree_id = t;
        }
    }
} else {
    tree_id = std::uniform_int_distribution<int>(0, n_trees - 1)(rng_);
}
```
这只关心"公平"——让每棵树 box 数均匀。但实际目标是**连通所有树**。一个已经连通的大组件可能有 3 棵树，它仍然被频繁选中来补齐 box 数，而真正需要生长的孤立小组件被忽略。

## 修改方案

改为**优先选择较小连通分量中的树**，在同一分量内保持 fewest-box 偏好：

```cpp
int tree_id = 0;
if (n_trees > 0) {
    // Compute component sizes
    std::vector<int> comp_size(n_trees, 0);
    for (int t = 0; t < n_trees; ++t)
        comp_size[tree_uf.find(t)]++;

    if (u01(rng_) < 0.7) {
        // Pick tree in the smallest component (fewest boxes as tiebreak)
        int best_comp = comp_size[tree_uf.find(0)];
        int best_cnt = tree_box_count[0];
        for (int t = 1; t < n_trees; ++t) {
            int cs = comp_size[tree_uf.find(t)];
            if (cs < best_comp ||
                (cs == best_comp && tree_box_count[t] < best_cnt)) {
                best_comp = cs;
                best_cnt = tree_box_count[t];
                tree_id = t;
            }
        }
    } else {
        tree_id = std::uniform_int_distribution<int>(0, n_trees - 1)(rng_);
    }
}
```

## 影响评估

- **影响范围**: `grow_coordinated()` 中树选择逻辑 ~12 行
- **风险**: 低。最差情况（所有分量大小相同）行为等同现有逻辑
- **预期效果**: 孤立树获得更多生长资源，加速跨分量 bridge
- **评级**: ★★★★☆
