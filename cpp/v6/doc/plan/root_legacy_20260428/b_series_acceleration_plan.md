# B-Series: Bridge RRT 加速计划

> 目标：将 bridge_all_islands 从 ~8.5–11s 压缩至 <4s  
> 基线：Build=13.3s, bridge 占 ~65% (8.5–11s), seed=1  
> 路径质量基准：5.063 / 5.985 / 8.920 / 4.792 / 1.966

## 最终结果

| 指标 | 基线 (T-series后) | B-series后 | 变化 |
|------|-------------------|------------|------|
| Build (warm) | 13.3s | **10.9s** | **-18%** |
| Bridge | ~8.3s | **5.7s** | **-32%** |
| 查询总时间(5q) | ~5.1s | **3.7s** | **-28%** |
| 路径质量 | 5.063/5.985/8.920/4.792/1.966 | **完全一致** | ✓ |
| Islands | 1 | 1 | ✓ |

已实施：B1 ✅, B2 ✅, B3 ✅, B6 ✅  
已跳过：B4 (KD-tree, 高维退化), B5 (commit_box, 预计<100ms)

---

## 瓶颈分析

Bridge RRT 的成本分解（以 seed=1 典型 run 为参考）：

| 组件 | 调用量(每次RRT) | 单次成本 | 备注 |
|------|-----------------|----------|------|
| `check_segment(res=20)` | ~数万次 | 21×check_config | 63次堆分配/次 |
| `check_config()` | ~数十万次 | 1×vector\<Interval\> + 1×vector\<float\> + IFK | 3次堆分配/次 |
| `rrt_nearest()` | ~数万次 | O(N) 线性扫描 | 树大小到~5000节点 |
| `commit_box()` | ~数百次 | O(N) 全box遍历 | N可达~3000 |
| `compute_fk_full()` | ~数十万次 | 区间FK(2×FLOP vs 点FK) | 点config无需区间 |

---

## B1: 消除碰撞检测堆分配 [P0, ⭐ 简单]

**问题**：`check_config()` 每次调用分配 `vector<Interval>(7)` + `vector<float>(7*6=42)`，  
`check_segment()` 每次迭代额外分配 `Eigen::VectorXd(7)`。  
bridge 过程中 check_segment 被调用数万次，每次 21 迭代 = **63次堆分配/次**。

**方案**：
1. `compute_fk_full()` 添加 `const Interval*` 指针重载，避免传 vector
2. `check_config()` 使用栈数组 `Interval[MAX_JOINTS]` + `float[MAX_TF*6]`
3. `check_segment()` 预分配 `diff` 和 `q` 在循环外复用

**涉及文件**：
- `include/sbf/core/fk_state.h` — 添加重载声明
- `src/core/fk_state.cpp` — 添加指针版 compute_fk_full
- `src/scene/collision_checker.cpp` — check_config/check_segment 改用栈数组

**预计收益**：~1–2s（消除每次 bridge RRT 中数十万次堆分配）  
**风险**：极低（纯性能优化，语义不变）

---

## B3: segment_resolution 20 → 10 [P0, ⭐ 简单]

**问题**：`check_segment(a, b, res=20)` 检查 21 个插值点。  
RRT step_size=0.1，7D空间中 0.1rad 步长下 10 个点（~0.01rad/步）已足够保守。

**方案**：
- 修改 `pair_rrt.segment_resolution = 20` → `10`（bridge_all_islands 和 bridge_s_t 两处）
- check_config 调用量直接减半

**涉及文件**：
- `src/forest/connectivity.cpp` L860, L680 — 两处 segment_resolution 赋值

**预计收益**：~2s（check_config 调用量减半）  
**风险**：中（需验证路径质量不变、islands=1）

---

## B2: 点 FK 快速路径 [P1, ⭐⭐ 中等]

**问题**：`check_config(q)` 包装 q 为零宽区间 [q,q]，然后走完整区间 FK 路径。  
区间乘法比点乘法 FLOP 多约 2×（每个 imat_mul_dh 需要 min/max 运算）。

**方案**：
1. 添加 `compute_fk_point(robot, q)` 使用标准 4×4 矩阵乘
2. `check_config()` 调用点 FK 而非区间 FK
3. 保留 `check_box()` 使用区间 FK（box 需要区间）

**涉及文件**：
- `include/sbf/core/fk_state.h` — 添加 compute_fk_point 声明
- `src/core/fk_state.cpp` — 实现 compute_fk_point
- `src/scene/collision_checker.cpp` — check_config 使用点 FK

**预计收益**：~1s（FK 计算本身提速 ~40%）  
**风险**：低（数学等价，且有方向正确性测试）

---

## B5: commit_box 空间索引 [P1, ⭐⭐ 中等]

**问题**：`commit_box()` 对所有现有 box 做 O(N) 遍历检查 shared_face/overlap，  
bridge 创建 ~200 个新 box，每个遍历 ~3000 个现有 box = ~60万次 overlap 检查。

**方案**：
- 为每个维度维护排序列表或网格索引
- 只检查与新 box 在首维有重叠的候选 box

**预计收益**：~300ms  
**风险**：中（需正确维护索引）

---

## B4: KD-tree 最近邻 [P2, ⭐⭐⭐ 复杂]

**问题**：`rrt_nearest()` 每次调用 O(N) 线性扫描。树增长到 ~5000 节点时  
每次查询 ~5000 次距离计算。

**方案**：
- 引入 nanoflann 或 简单 7D KD-tree
- O(log N) 最近邻查询
- 需要支持动态插入

**预计收益**：~1s  
**风险**：高（KD-tree 在 7D 退化严重，实际提升可能有限）

---

## B6: 渐进超时 + 目标偏置策略 [P2, ⭐⭐⭐ 复杂]

**问题**：所有岛对使用相同的 5000ms 超时。近距离pair可能只需 500ms，  
远距离pair 5000ms 仍不够。

**方案**：
- 根据 pair 距离动态调整超时：`timeout = base * (1 + dist/scale)`
- 第一轮快速 timeout（1000ms）尝试所有近距离 pair
- 第二轮对失败 pair 加大 timeout（5000ms）重试

**预计收益**：~1–3s  
**风险**：高（可能导致远距离pair失败增加）

---

## 执行顺序

| 序号 | 项目 | 状态 | 实际收益 |
|------|------|------|----------|
| 1 | B1: 消除堆分配 | ✅ 已实施 | 消除63次堆分配/check_segment |
| 2 | B3: seg_res 20→10 | ✅ 已实施 | check_config调用量减半 |
| 3 | B2: 点FK快速路径 | ✅ 已实施 | FK FLOP ~7×减少，查询-28% |
| 4 | B5: commit_box索引 | ⏭ 跳过 | 预计<100ms，ROI低 |
| 5 | B4: KD-tree最近邻 | ⏭ 跳过 | 7D退化严重，收益不确定 |
| 6 | B6: 渐进超时+取消 | ✅ 已实施 | bridge -32% (8.3→5.7s) |

### 修改文件清单

| 文件 | 修改内容 |
|------|----------|
| `include/sbf/core/fk_state.h` | B1: 添加 `compute_fk_full(Interval*, n)` 重载 |
| `src/core/fk_state.cpp` | B1: 指针版 compute_fk_full + vector版委托 |
| `include/sbf/scene/collision_checker.h` | 无修改(API不变) |
| `src/scene/collision_checker.cpp` | B1: 栈数组替代vector; B2: 点FK替代区间FK; B3: (seg_res在connectivity中) |
| `include/sbf/forest/connectivity.h` | B6: rrt_connect 添加 cancel 参数; 添加 `<atomic>` `<memory>` |
| `src/forest/connectivity.cpp` | B3: seg_res 20→10; B6: 渐进timeout/max_iters + 循环内cancel检查 |

**验证方法**：每项完成后运行：
```bash
cd /home/tian/桌面/box_aabb/cpp/build && \
rm -rf /tmp/lect_cache_kuka* && \
./experiments/exp2_e2e_planning --seeds 1 --no-viz 2>&1 | \
grep -E 'sweep|greedy|adj|bridge|build|seed.*build|AS->|TS->|CS->|LB->|RB->|Build|islands|grow='
```

确认：
1. `islands=1`（连通性不变）
2. 5条查询路径长度与基线一致
3. bridge 时间下降
