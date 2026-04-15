# Step 3: Proxy RRT 加速（Issue 4）

## 问题描述

当 start 或 goal 所在 box 不在最大 island 中时，`query()` 使用 proxy 机制：在最大 island 中搜索距离最近的 box 作为代理，通过 RRT-connect 连接。

当前参数：
- **候选数量**：50 个最近 box
- **单次超时**：2000ms（2s）
- **搜索方式**：顺序遍历，第一个成功即停止

实测问题：
- TS→CS 查询耗时 ~10s（proxy 搜索花费大部分时间）
- CS→LB 查询耗时 ~10s
- 原因：前 N 个最近的候选 box 的 RRT 全部超时失败（目标点和代理点之间障碍物密集），直到第 N+k 个才成功
- 最坏情况：50 × 2000ms = 100s

## 根因分析

文件：`src/planner/sbf_planner.cpp`，`query()` 中的 proxy 搜索（约 L610-L645）

### 当前代码

```cpp
auto find_proxy = [&](const Eigen::VectorXd& q, int orig_id,
                      std::vector<Eigen::VectorXd>& link_path,
                      const char* label) -> int {
    struct Cand { int id; double dist; };
    std::vector<Cand> candidates;
    for (int lid : islands[largest_idx]) {
        auto it = box_map_iso.find(lid);
        if (it == box_map_iso.end()) continue;
        double d = (q - it->second->center()).squaredNorm();
        candidates.push_back({lid, d});
    }
    std::sort(candidates.begin(), candidates.end(),
        [](const Cand& a, const Cand& b) { return a.dist < b.dist; });
    if (candidates.size() > 50) candidates.resize(50);

    for (const auto& c : candidates) {
        auto it = box_map_iso.find(c.id);
        if (it == box_map_iso.end()) continue;
        Eigen::VectorXd target = it->second->center();

        RRTConnectConfig rrt_cfg;
        rrt_cfg.timeout_ms = 2000.0;
        rrt_cfg.max_iters = 100000;
        rrt_cfg.segment_resolution = 20;

        auto rrt_path = rrt_connect(q, target, checker, robot_, rrt_cfg, c.id);
        if (!rrt_path.empty()) {
            link_path = std::move(rrt_path);
            fprintf(stderr, "[QRY] proxy: %s box %d -> proxy %d (dist=%.3f, rrt=%dwp)\n",
                    label, orig_id, c.id, std::sqrt(c.dist), (int)link_path.size());
            return c.id;
        }
    }
    return orig_id;  // failed
};
```

### 问题分析

1. **统一超时太长**：所有 50 个候选都用 2000ms 超时。最近的几个候选如果不可行（障碍物阻挡），每个都白等 2s。
2. **候选数过多**：50 个候选中很多距离很远，RRT 成功概率极低。
3. **无提前停止**：没有基于总耗时的提前终止机制。
4. **顺序搜索**：单线程依次尝试，无法利用近距离候选快速排除。

## 修改方案：分层超时 + 减少候选 + 总时间限制

### 核心思路

将候选分为 3 个层级，近距离候选用短超时快速尝试，远距离候选才用长超时：

| 层级 | 候选排名 | 单次超时 | 策略 |
|------|----------|----------|------|
| Tier 1 | 前 5 个 | 200ms | 快速尝试最近候选 |
| Tier 2 | 第 6-15 个 | 500ms | 中等尝试 |
| Tier 3 | 第 16-25 个 | 2000ms | 远距离最后尝试 |

同时增加**总时间限制**：整个 proxy 搜索不超过 8s。

### 具体修改

#### 文件：`src/planner/sbf_planner.cpp`

**修改 proxy 搜索 lambda**：

```cpp
auto find_proxy = [&](const Eigen::VectorXd& q, int orig_id,
                      std::vector<Eigen::VectorXd>& link_path,
                      const char* label) -> int {
    struct Cand { int id; double dist; };
    std::vector<Cand> candidates;
    for (int lid : islands[largest_idx]) {
        auto it = box_map_iso.find(lid);
        if (it == box_map_iso.end()) continue;
        double d = (q - it->second->center()).squaredNorm();
        candidates.push_back({lid, d});
    }
    std::sort(candidates.begin(), candidates.end(),
        [](const Cand& a, const Cand& b) { return a.dist < b.dist; });
    if (candidates.size() > 25) candidates.resize(25);  // ← 50 → 25

    auto proxy_t0 = std::chrono::steady_clock::now();
    constexpr double total_budget_ms = 8000.0;

    for (int ci = 0; ci < (int)candidates.size(); ++ci) {
        // Check total budget
        auto now = std::chrono::steady_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(now - proxy_t0).count();
        if (elapsed_ms > total_budget_ms) {
            fprintf(stderr, "[QRY] proxy: %s budget exhausted after %d candidates (%.0fms)\n",
                    label, ci, elapsed_ms);
            break;
        }

        // Tiered timeout
        double timeout_ms;
        if (ci < 5)       timeout_ms = 200.0;   // Tier 1: fast probe
        else if (ci < 15) timeout_ms = 500.0;    // Tier 2: medium
        else               timeout_ms = 2000.0;  // Tier 3: full

        const auto& c = candidates[ci];
        auto it = box_map_iso.find(c.id);
        if (it == box_map_iso.end()) continue;
        Eigen::VectorXd target = it->second->center();

        RRTConnectConfig rrt_cfg;
        rrt_cfg.timeout_ms = timeout_ms;
        rrt_cfg.max_iters = 100000;
        rrt_cfg.segment_resolution = 20;

        auto rrt_path = rrt_connect(q, target, checker, robot_, rrt_cfg, c.id);
        if (!rrt_path.empty()) {
            link_path = std::move(rrt_path);
            fprintf(stderr, "[QRY] proxy: %s box %d -> proxy %d (dist=%.3f, rrt=%dwp, try=%d, %.0fms)\n",
                    label, orig_id, c.id, std::sqrt(c.dist), (int)link_path.size(), ci+1, elapsed_ms);
            return c.id;
        }
    }
    return orig_id;  // failed
};
```

### 不修改的部分

- Bridge 步骤（bridge_s_t）不变
- Island 检测逻辑不变
- Dijkstra/GCS 搜索不变
- RRT fallback 不变

### 参数选择依据

- **200ms Tier 1**：近距离 C-space 连接，如果 200ms 内 RRT 找不到路径，说明中间有障碍物，跳过
- **500ms Tier 2**：中等距离，给 RRT 更多探索时间
- **2000ms Tier 3**：远距离最后手段，保持原超时
- **25 候选**：实测中成功的候选通常在前 10-15 名以内
- **8s 总预算**：覆盖最坏情况 5×200+10×500+10×2000 = 26s，但有 8s cap

## 预期效果

| 指标 | 修改前 | 修改后预期 |
|------|--------|-----------|
| TS→CS proxy 时间 | ~10s | <2s |
| CS→LB proxy 时间 | ~10s | <2s |
| 最坏情况单次 proxy | 100s (50×2s) | 8s (budget cap) |
| 典型成功 proxy | 2-6s | 0.2-1s |

**注意**：如果 Step 1 和 Step 2 修复了树均衡和岛屿碎片化，proxy 可能根本不再需要触发（所有 query 端点都在最大 island 中）。此步骤作为安全网保留。

## 验证步骤

```bash
cd /home/tian/桌面/box_aabb/cpp/build
cmake --build . --target exp2_e2e_planning -j$(nproc)
./experiments/exp2_e2e_planning --seeds 1 2>&1 | grep -E "proxy|QRY.*time"
```

检查：
1. 如果有 proxy 触发，日志显示 `try=N` 和耗时
2. 单次 proxy 搜索 <2s
3. 总 query 时间 <3s per pair
4. SR=100%
5. Drake collision verification: all clean

## 风险与回退

- **风险**：200ms 对某些困难连接太短 → 会 fallback 到 Tier 2/3
- **风险**：如果 Step 1+2 已经消除了 proxy 需求，此修改不产生可观测效果 → 作为安全网保留，不影响正确性
- **回退**：如果发现 SR 下降（某些 proxy 连接在 25 候选 × 8s 内仍失败），恢复 50 候选 + 2s 统一超时
- **可选增强**：
  - 并行 RRT（std::async 同时尝试前 N 个候选）
  - 基于 C-space 路标图的快速可达性预筛选
