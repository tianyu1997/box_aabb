# Plan Step 7: 查询优化 + 路径后处理 (P2/P3)

## 问题

1. RRT compete 固定 5s timeout，即使 box-chain 已很短也要等待 — 浪费查询时间
2. Cross-seed 仅产生 +86 boxes（0.3%），对连通性贡献甚微
3. Elastic band 碰撞时整体回退，丢弃所有优化
4. CS→LB 路径质量仍然偏高（9.59 rad）

## 方案

### 7a. 自适应 RRT Compete 预算

**文件**: `src/planner/sbf_planner.cpp`

- 计算 box-chain 路径长度 vs start→goal Euclidean 距离的比值 `r = chain_len / euclid_dist`
- 若 `r < 2.0`（路径质量好），设 RRT compete timeout = 1s（快速检查）
- 若 `r < 3.0`（中等），设 timeout = 3s
- 若 `r >= 3.0`（路径很差），设 timeout = 5s + 更大 max_iters
- 效果：好路径时查询快，差路径时给 RRT 更多机会

```cpp
double euclid = (goal - start).norm();
double ratio = chain_len / std::max(euclid, 0.01);
double rrt_timeout = (ratio < 2.0) ? 1000.0 : (ratio < 3.0) ? 3000.0 : 5000.0;
```

### 7b. Elastic Band 逐段回退

**文件**: `src/planner/sbf_planner.cpp`

- 当前：EB 后发现Any碰撞 → 整体回退到 EB 前
- 改为：EB 后逐段验证，只回退碰撞段的 waypoint 到 EB 前的值
- 保留非碰撞段的优化成果

```cpp
// 保存 pre_eb[i] for each waypoint
// EB 后逐段检查：
for (i : 1..n-2) {
    if (segment_collides(path[i-1], path[i]) || segment_collides(path[i], path[i+1]))
        path[i] = pre_eb[i];  // 只回退这一个点
}
```

### 7c. 增强 Cross-seed

**文件**: `src/forest/grower.cpp`

- 增大每树预算：100→500 boxes
- 多方向生长：向 k=3 个最近 main-component box 方向各发射生长线
- 双向 cross-seed：main component 也向小岛边界生长
- 效果：更早建立跨岛连接，减少 bridge 负担

### 7d. 二次优化 Pass

**文件**: `src/planner/sbf_planner.cpp`

- Final shortcut 后再做一轮 densify → elastic band
- 对已简化的短路径（通常 5-10 个点）做精细拉伸
- 开销很小（点少），但能进一步平滑轨迹

## 预期效果

| 指标 | 当前 | 目标 |
|------|------|------|
| LB→RB 查询时间 | 0.18s | <0.1s |
| RB→AS 查询时间 | 0.07s | <0.05s |
| CS→LB 查询时间 | 3.17s | <2.0s |
| EB collision revert | 整体回退 | 逐段回退 |
| Cross-seed boxes | +86 | +300-500 |

## 实现顺序

1. **7a** 自适应 RRT compete（10 min）
2. **7b** Elastic band 逐段回退（15 min）
3. 测试验证
4. **7c** 增强 cross-seed（20 min，可选）
5. **7d** 二次优化 pass（10 min，可选）
