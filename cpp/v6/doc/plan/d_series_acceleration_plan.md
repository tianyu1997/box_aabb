# D-Series 加速计划 — 详细执行文档

## 基线 (C-series 完成后)

| 指标 | 值 |
|------|-----|
| Build | 7.29s |
| Query (×5) | 3.25s |
| **Total** | **10.54s** |
| Boxes | 9,098 |
| Islands | 1 |
| 路径质量 | AS→TS=5.060, TS→CS=5.978, CS→LB=8.920, LB→RB=4.788, RB→AS=1.961 |

## 执行顺序: D7 → D2 → D3 → D4 → D1

---

## D7: Query 子阶段 chrono 精确计时 [诊断工具]

### 目标
在 `SBFPlanner::query()` 中每个子阶段前后插入 `chrono::steady_clock` 打点，
输出精确毫秒级耗时，替代当前基于日志推断的粗略估算。

### 修改文件
- `src/planner/sbf_planner.cpp` — `SBFPlanner::query()` (L550-L1480)

### 具体改动
在以下阶段边界插入计时点 (`auto tXX = now()`):

| 计时点 | 位置 | 阶段 |
|--------|------|------|
| t0 | L557 (已有) | 函数入口 |
| t1 | L592后 | find_box 完成 |
| t2 | L618后 | bridge_s_t 完成 |
| t3 | L716后 | island+proxy 完成 |
| t4 | L770后 | dijkstra+extract 完成 |
| t5 | L895后 | pre-compete simplify 完成 |
| t6 | L1023后 | RRT compete 完成 |
| t7 | L1078后 | segment validation 完成 |
| t8 | L1105后 | greedy simplify 完成 |
| t9 | L1143后 | random shortcut 完成 |
| t10 | L1173后 | densify 完成 |
| t11 | L1258后 | EB 完成 |
| t12 | L1298后 | final shortcut 完成 |
| t13 | L1398后 | pass2 完成 |

输出格式:
```
[QRY-T] find=0.1ms bridge=0.0ms island=0.0ms dij+ext=0.8ms pre_simp=4.5ms
        rrt_compete=702.3ms validate=3.1ms greedy=1.0ms shortcut=5.3ms
        densify=0.2ms eb=48.7ms final_sc=5.3ms pass2=12.1ms total=783.4ms
```

### 预计收益
- 性能: **0ms** (纯诊断)
- 价值: 为后续 D2-D5 提供精确数据基础

### 风险
- **无** — 仅添加 stdio 输出

---

## D2: TS→CS RRT compete 预算缩减 [预计 -300~400ms query]

### 问题分析
TS→CS: chain=7.406, direct=6.187, ratio=1.19 (< 2.0)
当前 `ratio < 2.0 → timeout=1000ms`，3 trials 合计 ~700ms。
仅获得 16% 路径缩短，成本/收益比差。

### 修改文件
- `src/planner/sbf_planner.cpp` L947

### 具体改动
```cpp
// 当前:
double rrt_timeout = (ratio < 2.0) ? 1000.0 : (ratio < 3.0) ? 3000.0 : 5000.0;

// 改为:
double rrt_timeout = (ratio < 1.5) ? 300.0
                   : (ratio < 2.0) ? 500.0
                   : (ratio < 3.0) ? 3000.0 : 5000.0;
```

同时减少 bonus trials 的 max_iters:
```cpp
// 当前:
double bonus_timeout = std::min(rrt_timeout, 1000.0);
// bonus trials 80K iters

// 改为 (按比例):
int main_iters = (ratio < 1.5) ? 30000 : (ratio < 2.0) ? 100000 : 200000;
int bonus_iters = (ratio < 1.5) ? 15000 : 80000;
```

### 预计影响
- TS→CS: 768ms → ~300-400ms (**-400ms**)
- AS→TS (ratio≈1.8): 1000ms → 500ms (**-150ms** 潜在)
- CS→LB (ratio≈2.6): 不变 (仍是 3000ms tier)
- 路径质量: TS→CS 可能从 5.978 微升至 ~6.2 (仍 < chain 7.406)

### 验证标准
- Total query 下降 ≥ 200ms
- 路径质量: 各段变化 < 10%
- 5/5 collision-free

---

## D3: EB 早期终止优化 [预计 -50~100ms query]

### 问题分析
CS→LB EB 做了 1764 moves (60 outer iters)，后期改善极小。
当前仅有 `if (!any_move) break` — 但即使改善 0.0001 也算 any_move=true。

### 修改文件
- `src/planner/sbf_planner.cpp` L1178-1220 (Step 3 EB loop)

### 具体改动
```cpp
// 在 EB 循环外添加:
double eb_len_prev = sbf::path_length(path);
int stagnant_count = 0;

for (int iter = 0; iter < 60; ++iter) {
    bool any_move = false;
    // ... existing inner loop ...
    if (!any_move) break;
    
    // 新增: 改善量追踪
    if (iter >= 8) {
        double eb_len_cur = sbf::path_length(path);
        double improvement = (eb_len_prev - eb_len_cur) / std::max(eb_len_prev, 1e-6);
        if (improvement < 0.001) {  // < 0.1% 改善
            ++stagnant_count;
            if (stagnant_count >= 2) break;  // 连续 2 轮 < 0.1% 则停止
        } else {
            stagnant_count = 0;
        }
        eb_len_prev = eb_len_cur;
    }
}
```

### 预计影响
- CS→LB EB: 60 iters → ~25-35 iters (节省 ~60-80ms)
- 其他段 EB 也会适度缩减
- 路径质量: 几乎不变 (后期改善本就 < 0.1%)

### 验证标准
- 路径质量变化 < 0.5%
- EB 日志可见提前终止 (moves 数减少)

---

## D4: Bridge island 快速放弃 [预计 -200~400ms build]

### 问题分析
Island #4 (58 boxes, 不可达) Round 0 做了 3 pair × ~4-5 并行 RRT = ~14 次 FAIL@50K iters。
`max_consecutive_fails = 3` 导致多余的 pair 尝试。

### 修改文件
- `src/forest/connectivity.cpp` L911-912

### 具体改动
```cpp
// 当前:
const int max_consecutive_fails = (islands.size() <= 2) ? (int)futures.size() : 3;

// 改为:
const int max_consecutive_fails = (islands.size() <= 2) ? (int)futures.size() : 2;
```

### 预计影响
- Island #4: 少 1 pair 尝试 → 省 ~4 次 50K RRT FAIL ≈ 200ms
- 不影响连通性 (Island #4 靠 Round 1 post-adj 连通)
- 对 2-island case 不受影响 (仍是 futures.size())

### 验证标准
- Build time 下降 ≥ 100ms
- islands=1 不变
- 路径质量不变

---

## D1: CS→LB warm-start RRT [预计 -500~800ms query]

### 问题分析
CS→LB RRT compete 1.5s, main trial 78K iters, tree 严重不平衡 (tree_a=648, tree_b=31320)。
Start 端 narrow passage 导致 tree_a 几乎无法扩展。

### 方案
将 box-chain 路径的 collision-free waypoints 注入 tree_a 作为初始节点:
1. 从 chain 中等距采样 15-20 个候选 waypoints
2. 过滤: `check_config()` → 保留 collision-free 的
3. 将这些点作为 tree_a 的额外 root nodes
4. RRT 从多个 root 同时扩展, 跨越 narrow passage

### 修改文件
- `include/sbf/forest/connectivity.h` — 新增 `rrt_connect_warmstart()` 或扩展 `rrt_connect` 参数
- `src/forest/connectivity.cpp` — 实现 warm-start 逻辑
- `src/planner/sbf_planner.cpp` L975-993 — `run_trial` 中使用 warm-start

### 具体改动

**connectivity.h** — 新增重载:
```cpp
std::vector<Eigen::VectorXd> rrt_connect(
    const Eigen::VectorXd& q_a,
    const Eigen::VectorXd& q_b,
    const CollisionChecker& checker,
    const Robot& robot,
    RRTConnectConfig cfg = {},
    int seed = 42,
    const std::atomic<bool>* cancel = nullptr,
    const std::vector<Eigen::VectorXd>& warm_starts = {}  // NEW
);
```

**connectivity.cpp** — rrt_connect 实现中:
```cpp
// 在 tree_a 初始化后, 将 warm_starts 中 collision-free 点加入 tree_a
for (const auto& ws : warm_starts) {
    if (!checker.check_config(ws)) {
        int parent = 0;  // connect to root
        double best_dist = (ws - tree_a[0].config).norm();
        for (int k = 1; k < (int)tree_a.size(); ++k) {
            double d = (ws - tree_a[k].config).norm();
            if (d < best_dist) { best_dist = d; parent = k; }
        }
        tree_a.push_back({ws, parent});
    }
}
```

**sbf_planner.cpp** — run_trial 传入 chain waypoints:
```cpp
// 从 chain path 等距采样 warm-start candidates
std::vector<Eigen::VectorXd> warm_starts;
if (path.size() > 10) {
    int step = std::max(1, (int)path.size() / 15);
    for (size_t i = step; i < path.size() - 1; i += step)
        warm_starts.push_back(path[i]);
}

auto run_trial = [&](double timeout, int max_iters, int seed) -> TrialResult {
    // ... existing cfg setup ...
    auto p = rrt_connect(start, goal, checker, robot_, cfg, seed, nullptr, warm_starts);
    // ...
};
```

### 预计影响
- CS→LB: main trial 78K→~15K iters, 1.5s→~300ms (**-1.2s**)
- 保守估计: **-500~800ms query**
- 其他段 (AS→TS, TS→CS) 也可能受益

### 风险
- 中等: warm-start 点可能误导 RRT 进 dead-end
- 需确保 tree_a 中 warm-start 节点的 parent 合法

### 验证标准
- CS→LB RRT iters 显著下降 (< 30K)
- Query total 下降 ≥ 400ms
- 路径质量: CS→LB 变化 < 15%

---

## 执行结果

### D7: Query chrono 计时 ✅
- 在 `query()` 14 个子阶段边界添加 `chrono::steady_clock` 打点
- 输出 `[QRY-T]` 精确毫秒级计时
- 揭示: **RRT compete 占 query 时间 93.8%** (3078ms / 3281ms)

### D7 精确计时数据

| Query | find | bridge | island | dij+ext | pre_simp | rrt_compete | validate | greedy | shortcut | densify | eb | final_sc | pass2 | total |
|-------|------|--------|--------|---------|----------|-------------|----------|--------|----------|---------|-----|----------|-------|-------|
| AS→TS | 0.3 | 1.3 | 1.6 | 5.9 | 1.7 | **306.7** | 0.0 | 0.1 | 0.0 | 0.0 | 13.7 | 1.6 | 0.6 | 333.8 |
| TS→CS | 0.5 | 1.4 | 1.6 | 2.1 | 1.4 | **822.1** | 0.0 | 0.1 | 0.0 | 0.0 | 23.4 | 3.6 | 0.0 | 856.3 |
| CS→LB | 0.7 | 1.8 | 2.1 | 7.9 | 6.3 | **1945.6** | 0.0 | 0.1 | 0.0 | 0.0 | 30.8 | 20.8 | 14.2 | 2030.6 |
| LB→RB | 0.5 | 1.6 | 2.2 | 5.2 | 11.4 | **3.5** | 0.0 | 0.1 | 0.0 | 0.0 | 10.2 | 6.3 | 0.0 | 37.1 |
| RB→AS | 0.5 | 1.4 | 1.6 | 5.2 | 7.9 | **0.6** | 0.0 | 0.0 | 0.0 | 0.0 | 3.1 | 2.6 | 0.0 | 23.1 |
| **Total** | 2.5 | 7.5 | 9.1 | 26.3 | 28.7 | **3078.5** | 0.0 | 0.4 | 0.0 | 0.0 | 81.2 | 34.9 | 14.8 | **3280.9** |

### D2: TS→CS RRT budget 缩减 ❌
- **原因**: 所有 query 的 chain/euclid ratio > 4 (范围 4.26-35.10)
- ratio < 1.5 和 < 2.0 的新 tier 永远不会触发
- 原分析误用 chain/direct ratio (1.19), 实际 chain/euclid ratio 是 10.68
- RRT compete 因 path-found 自然终止, timeout 仅是安全上限, 降低 timeout 无影响
- **已回退**

### D3: EB 早期终止 ✅
- 添加 per-iteration 改善量追踪: 连续 2 轮 < 0.1% → break
- EB 迭代: 16/11/13/11/11 (vs 之前 60 max)
- EB 耗时: 81.2ms → **23.1ms** (**-58ms**)
- 路径质量不变: 5.065/5.978/8.929/4.790/1.961

### D4: Bridge island 快速放弃 ✅
- `max_consecutive_fails` 3→2 (≥3 islands 场景)
- Island #4 现在在 pair 2 即停止 (省 1 pair 尝试)
- bridge_all: 2679ms → **2483ms** (**-196ms**)
- boxes=9098, islands=1 不变

### D1: CS→LB warm-start RRT ❌
- **尝试 1**: 添加 warm_starts 参数到 rrt_connect, 每个 warm-start 连接到最近 tree_a 节点
  - TS→CS: ✅ -561ms (1/23 有效点成功桥接)
  - CS→LB: ❌ +1520ms 退化 (2/20 有效点扰乱树拓扑)
- **尝试 2**: 改为链式连接 (root→ws1→ws2→...), 增大采样密度
  - TS→CS: ❌ 触发 emergency RRT (路径完全失败)
  - AS→TS: ❌ 3x 退化
- **根因**: warm-start 破坏 RRT-Connect 的 Voronoi bias — 添加偏置初始节点使最近邻搜索偏向 corridor, 损害探索多样性
- **已回退**

---

## 最终结果 (D-series 完成)

### 活跃优化
- **D7**: `[QRY-T]` chrono 诊断 (sbf_planner.cpp)
- **D3**: EB 早期终止 — stagnation break (sbf_planner.cpp)
- **D4**: Bridge consecutive_fails 3→2 (connectivity.cpp)

### 综合基准 (3 runs median)

| 指标 | C-series 基线 | D-series 后 | 变化 |
|------|:-------------|:-----------|:-----|
| Build | 7.29s | **7.27s** | -0.3% |
| Query | 3.25s | **3.33s** | ≈0% (噪声) |
| **Total** | **10.54s** | **10.60s** | **≈0%** |
| Boxes | 9,098 | 9,098 | — |
| Islands | 1 | 1 | — |

### 路径质量 (不变)
```
AS→TS: 5.065    TS→CS: 5.978    CS→LB: 8.929
LB→RB: 4.790    RB→AS: 1.961
5/5 queries OK ✅
```

### D-series 总结
D-series 的预期目标 (10.54s→9.0s, -15%) **未达成**:
- D3 EB 改善 58ms, D4 bridge 改善 ~200ms, 但被 run-to-run 方差掩盖
- **RRT compete (93.8% of query)** 是真正的性能墙
  - 无法通过 timeout/budget 调整改善 (RRT 自然终止于 path-found)
  - warm-start tree seeding 破坏 bidirectional 探索
  - 需要算法级变更 (Informed RRT*, BIT*, 或非 sampling-based 方法) 才能突破

### 全系列优化历史

| 阶段 | Build | Query(5) | Total | vs 原始 |
|------|-------|----------|-------|---------|
| 原始基线 | 24.97s | 10.59s | 35.56s | — |
| +T-series | 13.3s | ~5.1s | ~18.4s | -48% |
| +B-series | 12.47s | 3.62s | 16.09s | -55% |
| +C-series | 7.29s | 3.25s | 10.54s | -70% |
| **+D-series** | **7.27s** | **3.33s** | **10.60s** | **-70%** |

**T+B+C+D 总收益: 35.56s → 10.60s (-70%)**
