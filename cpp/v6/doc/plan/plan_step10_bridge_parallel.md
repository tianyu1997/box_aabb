# Plan Step 10: bridge_all_islands 并行化 (P2)

## 目标
将 bridge_all_islands 中对独立 island 对的 RRT-Connect 并行执行，降低 build 时间。当前 bridge 占 build 时间的 58% (17.7s/30.7s)。

## 基线数据
- bridge_all_islands: 17.7s（fresh cache）
- 主要成本: 多个 island-pair 的 RRT-Connect（每对 ~5000ms timeout，~87K iters）
- 5 个 island → 4 对需要桥接
- Build 总时间: 30.7s

## 修改文件
- `src/forest/connectivity.cpp` — bridge_all_islands 函数
- `include/sbf/forest/connectivity.h` — 如果需要修改签名

## 实现步骤

### 10a. 分析并行机会
每轮 bridge 中，main island 与多个 small island 的 RRT-Connect 是独立的：
- island #1 → main: RRT + chain_pave
- island #2 → main: RRT + chain_pave
- island #3 → main: RRT + chain_pave
- island #4 → main: RRT + chain_pave

这 4 对可以并行执行。chain_pave 写入 boxes_ 和 adj_，需要串行化。

### 10b. 并行 RRT + 串行 Pave 架构
```
1. 对每个 small island，并行跑 RRT-Connect（只返回 path, small_id, main_id）
2. 收集所有成功的 RRT paths
3. 串行执行 chain_pave_along_path（写入 boxes_/adj_/UF）
4. 更新 UF，检查是否所有 island 已合并
```

关键：RRT-Connect 是纯只读操作（只读 checker、robot），可安全并行。
chain_pave 修改 boxes_/adj_，必须串行。

### 10c. 使用 std::async 实现
```cpp
#include <future>

// 在每轮 bridge 中:
struct RRTResult {
    int small_id, main_id;
    std::vector<Eigen::VectorXd> path;
    double dist;
};

std::vector<std::future<RRTResult>> futures;
for (size_t ii = 1; ii < islands.size(); ++ii) {
    futures.push_back(std::async(std::launch::async, [&, ii]() -> RRTResult {
        // 找最近候选对
        // 跑 RRT-Connect
        // 返回结果
    }));
}

// 收集结果, 串行 chain_pave
for (auto& f : futures) {
    auto res = f.get();
    if (res.path.empty()) continue;
    chain_pave_along_path(res.path, res.small_id, boxes, ...);
    absorb_new_boxes(old_n);
}
```

### 10d. 线程安全考虑
- CollisionChecker: 需要每线程一个副本（内部可能有状态）
- Robot: 只读，可共享
- LECT: 只读查询，可共享
- boxes/adj: 只在主线程 chain_pave 时修改

### 10e. 候选对预计算
当前每个 small island 遍历所有 main_centers 找候选对。
并行化后，预计算所有候选对，按 island 分组，然后并行分发 RRT。

## 验证方法
```bash
rm -f ~/.sbf_cache/kuka_iiwa14_r820_*.lect
./experiments/exp2_e2e_planning --seeds 1 2>&1 | grep -E 'bridge|build=|island'
# 期望: bridge 时间从 17.7s 降至 ~5-8s（4 核）
```

## 预期效果
- bridge_all_islands: 17.7s → ~5-8s（3-4x 加速）
- Build 总时间: 30.7s → ~18-22s
- 路径质量不变（RRT 只是并行了，结果等价）

## 风险
- 中：CollisionChecker 线程安全需验证
- RRT 产生的 seed 需要每线程唯一（避免相同路径）
- 如果内存压力大，多个大 RRT 并行可能 OOM（但每个只有 ~100K 节点，应该没问题）
