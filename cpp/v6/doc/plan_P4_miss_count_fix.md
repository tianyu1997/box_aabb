# P4: Miss Count 修正

## 问题

**文件**: `src/forest/grower.cpp` L1345, L1524  
**现状**: miss count 有两个问题：

### 问题 1: 空 batch 时虚增
```cpp
if (tasks.empty()) {
    miss_count += batch_cap;  // L1345: batch_cap 可能是 4-16
    continue;
}
```
一次空 batch 直接加 `batch_cap`（等于 n_workers），但实际只"失败"了一次尝试。这在 box 密集区域会导致 miss_count 膨胀过快。

### 问题 2: 成功/失败混合 batch
```cpp
if (batch_success > 0) miss_count = 0;
else miss_count += (int)tasks.size();  // L1524
```
只要 batch 中有 1 个成功就清零 miss_count，但 batch 中可能只有 1/8 成功。反过来，全失败时加 `tasks.size()` 也不够精确。

## 修改方案

改用更精确的连续失败计数：

```cpp
// 空 batch 时只加 1（代表一次尝试失败）
if (tasks.empty()) {
    miss_count++;
    continue;
}

// ... (collect results)

// 按实际失败数累加，成功数递减
miss_count = std::max(0, miss_count + (int)tasks.size() - batch_success * 2);
```

同时调整循环条件中的阈值比较（原来是 `max_consecutive_miss * 4`）：
```cpp
// 改为更合理的阈值
while ((int)boxes_.size() < config_.max_boxes &&
       miss_count < config_.max_consecutive_miss &&
       !deadline_reached()) {
```

## 影响评估

- **影响范围**: 3 处修改，共 ~5 行
- **风险**: 低。最差情况只是循环提前/延迟终止
- **预期效果**: 避免过早终止（miss_count 膨胀）或过晚终止（miss_count 被清零）
- **评级**: ★★★☆☆
