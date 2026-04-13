# P3: Worker LECT 周期刷新

## 问题

**文件**: `src/forest/grower.cpp` L1295-L1299  
**现状**: Worker LECT 在进入主循环前创建一次，之后永不刷新：
```cpp
std::vector<std::unique_ptr<LECT>> worker_lects;
worker_lects.reserve(n_workers);
for (int i = 0; i < n_workers; ++i)
    worker_lects.push_back(std::make_unique<LECT>(lect_.snapshot()));
```
随着主 LECT 不断因为 `enforce_parent_adjacency()` 和新 box 的 `mark_occupied()` 而更新，worker 的 LECT 快照越来越过时。这导致：
1. Worker FFB 在已经被主 LECT 标记为 occupied 的区域尝试展开 → 浪费
2. Worker 的 collision cache 无法利用主 LECT 的最新状态

## 修改方案

每 N 个 batch 后刷新 worker LECT 快照：

```cpp
// 在主循环开始前定义刷新间隔
const int lect_refresh_interval = 50;  // 每 50 个 batch 刷新一次

// 在主循环末尾（total_batches++ 之后）添加刷新逻辑：
if (total_batches % lect_refresh_interval == 0) {
    for (int i = 0; i < n_workers; ++i)
        *worker_lects[i] = lect_.snapshot();
}
```

## 影响评估

- **影响范围**: 2 处插入，共 ~5 行
- **风险**: 低。snapshot() 是已有方法，刷新间隔可调
- **预期效果**: 减少因 stale LECT 导致的无效 FFB 调用，提高后期（box 数 > 5000+）成功率
- **评级**: ★★★☆☆ (中等优先级，对前期影响不大，后期收益显著)
