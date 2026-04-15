# Plan Step 6: Bridge 效率 + 增量邻接优化 (P1)

## 问题

`bridge_all_islands` 占 build 时间的 45%（11.8s / 26.0s）。
`compute_adjacency` 在 build 中被调用 ≥4 次，每次 O(N²)。

**瓶颈**：
1. 每次 RRT bridge 成功后，调用 `find_islands()` 做 BFS 全图遍历（O(V+E)）检查连通性
2. 每次 bridge 候选对尝试后，重建 `id_to_idx` map（O(N)）
3. `compute_adjacency()` 是 O(N²) 的全量对比，9K boxes 时约 40M 对比较

## 方案

### 6a. UnionFind 增量连通性

**文件**: `src/forest/connectivity.cpp`

- 新增 `UnionFind` 类（path compression + union by rank）
- `bridge_all_islands` 初始化时从 adj 构建 UnionFind
- 每次 bridge 成功（chain_pave 创建新 box 并加入 adj）后，增量 union
- `find(a) == find(b)` 替代 `find_islands()` 全图遍历
- O(α(N)) ≈ O(1) vs O(V+E)

```cpp
struct UnionFind {
    std::unordered_map<int, int> parent, rank;
    int find(int x);
    void unite(int x, int y);
    bool connected(int x, int y) { return find(x) == find(y); }
    int count_components();  // 仅在需要时调用
};
```

### 6b. 持久化 id_to_idx

**文件**: `src/forest/connectivity.cpp`

- 将 `id_to_idx` 作为函数参数传入或在外部维护
- `commit_box` 时同步更新，避免每次候选对尝试都 O(N) 重建

### 6c. 增量邻接维护（长期）

**文件**: `src/forest/adjacency.cpp`, `src/forest/coarsen.cpp`

- coarsen_sweep 删除 box 时直接从 adj 删除对应边
- coarsen_greedy 合并 box 时更新 adj（新 box 继承两个旧 box 的邻居）
- 消除 build 中的重复 compute_adjacency 调用

## 预期效果

| 指标 | 当前 | 目标 |
|------|------|------|
| bridge_all_islands | 11.8s | <6s |
| find_islands 调用次数 | ~10 次/build | 0（用 UnionFind）|
| compute_adjacency 调用次数 | ≥4 次/build | 1 次/build |
| Build 总时间 | 26.0s | <20s |

## 实现顺序

1. **6a** UnionFind 类 + bridge_all_islands 改造（20 min）
2. **6b** 持久化 id_to_idx（5 min）
3. 测试验证
4. **6c** 增量邻接（30 min，可选长期优化）
