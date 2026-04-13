# Bridge 优化 + BIT* + box_seq 缩短 — 综合执行计划

## 目标
1. 将 `bridge_all_islands` 中的 RRT-Connect 替换为 OMPL BIT* — 提升桥接成功率
2. 增强 Bridge 参数（超时、候选对数、连续失败容忍） — 增加搜索覆盖
3. A* hop penalty — 减少 box_seq 中冗余跳转
4. box_seq shortcut — 后处理压缩 box 序列长度
5. 修复 ompl_adapter.h/cpp 中 v1 API 不兼容问题

## 基准 (Baseline)
- 5 seed, 16 obstacles, KUKA IIWA14 7-DOF
- boxes ≈ 5500, islands ≈ 2-5 (3个未桥接)
- box_seq mean ≈ 160 (range 2–395)
- 25/25 paths collision-free ✓

---

## Step 1: OMPL BIT* 替代 RRT-Connect (bridge)

### 1a. CMakeLists.txt 修改
- `find_package(ompl REQUIRED)` → 使用 `OMPL_LIBRARIES` + `OMPL_INCLUDE_DIRS`（v1.5.2 无 imported target）
- `sbf_forest` 添加 OMPL 链接（connectivity.cpp 依赖 OMPL）
- 保留 `sbf_planner` 的 OMPL 链接

### 1b. connectivity.h 添加 `bitstar_bridge()` 声明
- `#ifdef SBF_HAS_OMPL` 条件声明
- 签名: `std::vector<VectorXd> bitstar_bridge(q_a, q_b, checker, robot, timeout_ms, cancel)`
- 与 `rrt_connect()` 返回类型相同，可直接替换

### 1c. connectivity.cpp 实现 `bitstar_bridge()`
- 创建 OMPL RealVectorStateSpace → SpaceInformation → BITstar
- 使用 PlannerTerminationCondition 实现 timeout + cancel 检查
- 返回 simplifyMax 后的路径

### 1d. bridge_all_islands 中替换
- `#ifdef SBF_HAS_OMPL` 使用 `bitstar_bridge`，`#else` 保留 `rrt_connect`
- 无需改变 chain_pave 逻辑

### 验证
- 单 seed 构建，检查 `[BIT*]` 日志，比较 islands 数量

---

## Step 2: Bridge 参数增强

### 修改点
- `sbf_planner.cpp` L489: `5000.0, 10, 1000` → `15000.0, 50, 1000`
- `connectivity.cpp` ~L1063: `max_consecutive_fails` 2→10 (≥3 islands)

### 验证
- 比较 bridge 前后 islands 数量

---

## Step 3: A* hop penalty

### 修改点
- `dijkstra.cpp` L124: `double edge_cost = (u_repr - fc).norm();` 后添加 `edge_cost += 0.02;`
- 作用: 惩罚过多 hop，鼓励走更少更大的 box

### 验证
- 比较 box_seq 平均长度

---

## Step 4: box_seq shortcut

### 新增函数
- `dijkstra.h`: 声明 `shortcut_box_sequence(box_seq, adj)`
- `dijkstra.cpp`: 实现贪心前跳 — 对序列中每个 box，向前查找最远的相邻 box 直接跳过去

### 整合
- `sbf_planner.cpp`: 在 dijkstra_search 返回后、extract_waypoints 之前调用

### 验证
- 比较 box_seq mean 是否降到 <80

---

## Step 5: ompl_adapter.h/cpp API 修复

### Bug 清单
- `ompl_adapter.h`: `box.lo[d]` → `box.joint_intervals[d].lo` (SBFStateValidityChecker, SBFStateSampler, SBFMotionValidator)
- `ompl_adapter.h`: `box.hi[d]` → `box.joint_intervals[d].hi`
- `ompl_adapter.cpp` L40: `robot_->dof()` → `robot_->n_joints()`
- `ompl_adapter.cpp` L46-47: `robot_->joint_lo(i)/joint_hi(i)` → `robot_->joint_limits().limits[i].lo/.hi`

---

## 回退策略
每个 Step 完成后验证：
- 效果好 → 保留，继续下一步
- 效果不佳 → git revert 回退该 step，继续其他 step

## 最终验证
- `./experiments/exp2_e2e_planning --seeds 5`
- 目标: islands=1, box_seq mean<80, 25/25 collision-free

---

## 执行结果

### 最终参数
| 参数 | 原值 | 最终值 |
|------|------|--------|
| Bridge planner | RRT-Connect | OMPL BIT* |
| timeout_ms | 5000 | 5000 |
| max_pairs_per_gap | 10 | 30 |
| max_pave_ms | 1000 | 1000 |
| max_consecutive_fails | 2 | 5 (2 islands时=all) |
| A* hop penalty | 0 | +0.02 |
| shortcut_box_sequence | 无 | 启用 |
| OMPL log level | INFO | WARN |

### 5-seed 验证结果 (2025-01 final)
```
Build:   med=78.1s  mean=84.6s

AS->TS    SR=100%  t_med=0.284s  len_med=5.068
TS->CS    SR=100%  t_med=2.012s  len_med=5.985
CS->LB    SR=100%  t_med=3.460s  len_med=8.929
LB->RB    SR=100%  t_med=0.031s  len_med=4.794
RB->AS    SR=100%  t_med=0.015s  len_med=1.966

25/25 queries OK ✓
```

### 每 seed 详情
| seed | build(s) | boxes | islands | bridge(ms) | bridge_boxes |
|------|----------|-------|---------|------------|--------------|
| 0 | 93.4 | 5512 | 1 | 88863 | 141 |
| 1 | 57.0 | 5371 | 2→repair→OK | 52110 | 67 |
| 2 | 138.0 | 5220 | 1 | 132365 | 131 |
| 3 | 56.4 | 5575 | 1 | 50509 | 150 |
| 4 | 78.1 | 5212 | 2→repair→OK | 72800 | 124 |

### 关键发现
- **BIT* bridge**: 3/5 seeds 直接达到 islands=1; 2/5 seeds 剩余 2 islands 但经 repair_bridge_adjacency 修复后查询全部成功
- **bridge 耗时**: median ~73s (从初始 15s timeout 的 388s 降至 5s timeout 的 73s)
- **难桥接 island**: ~1000 boxes 的 island #3 在所有 seeds 中都难以桥接（C-space 拓扑障碍），依赖 repair 机制
- **所有步骤保留**: Step 1-5 全部有正面效果，无需回退
