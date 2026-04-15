# Plan Step 8: 多试 RRT 竞争 (P0)

## 目标
在 query 阶段的 RRT compete 环节跑多次独立 RRT（不同 seed），保留最短路径，降低随机性导致的路径质量波动。

## 基线数据
- CS→LB: RRT direct = 10.492（单次），经优化后 8.920
- 其他查询 RRT 均单次命中

## 修改文件
- `src/planner/sbf_planner.cpp` — query() 中 RRT compete 段

## 实现步骤

### 8a. 多试 RRT 循环
**位置**: `sbf_planner.cpp` ~L935-980（RRT compete 区域）

当前逻辑:
```cpp
auto direct = rrt_connect(start, goal, checker, robot_, rrt_compete);
// ... simplify & compare
```

改为:
```cpp
// 跑 N_TRIALS 次 RRT，保留最短的简化路径
const int N_TRIALS = 3;
std::vector<Eigen::VectorXd> best_direct;
double best_direct_len = std::numeric_limits<double>::max();

for (int trial = 0; trial < N_TRIALS; ++trial) {
    RRTConnectConfig rrt_compete;
    rrt_compete.timeout_ms = rrt_timeout / N_TRIALS;  // 按比例分配时间
    rrt_compete.max_iters  = 200000 / N_TRIALS;
    rrt_compete.step_size  = 0.2;
    rrt_compete.goal_bias  = 0.15;
    rrt_compete.segment_resolution = 20;
    
    auto direct = rrt_connect(start, goal, checker, robot_, rrt_compete, 42 + trial * 137);
    if (direct.empty()) continue;
    
    // 简化
    greedy_forward_simplify(direct, checker, ares_fn);
    double len = path_length(direct);
    
    if (len < best_direct_len) {
        best_direct_len = len;
        best_direct = std::move(direct);
    }
}
// 然后用 best_direct 与 chain 比较
```

### 8b. 时间预算分配
- 总 RRT 预算不变（ratio < 2→1s, < 3→3s, else→5s）
- 每次试验 = total / N_TRIALS
- N_TRIALS = 3（平衡质量与速度）

### 8c. Seed 多样性
- trial 0: seed=42
- trial 1: seed=179
- trial 2: seed=316
- 使用 `42 + trial * 137` 确保足够分散

## 验证方法
```bash
rm -f ~/.sbf_cache/kuka_iiwa14_r820_*.lect
./experiments/exp2_e2e_planning --seeds 3 2>&1 | tee /tmp/exp2_multi_rrt.txt
# 重点关注 CS→LB: 是否从 8.920 降低
```

## 预期效果
- CS→LB: 8.920 → ~7.5-8.5（多次试验更可能找到短路径）
- 总查询时间基本不变（时间已按比例分配）
- 其他查询无退化（多试只会更好或持平）

## 风险
- 低：时间预算不增加，仅增加多样性
- 单次 RRT 预算缩小可能偶尔错过长路径（但 chain 做兜底）
