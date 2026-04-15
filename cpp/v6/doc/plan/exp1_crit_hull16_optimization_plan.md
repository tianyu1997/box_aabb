# Exp1 优化计划: CritSample + Hull16_Grid

> 日期: 2026-04-12
> 目标: 在尽量短的时间内，最大化覆盖率，coarsen 后 box 数量少，连通性强
> Envelope: CritSample + Hull16_Grid (voxel_delta=0.02)

## 1. 问题诊断

### 1.1 OOM 根因
- `bridge_all_islands()` 内部硬编码 `n_rrt_threads = hardware_concurrency()-1 = 15`
- 15 个并行 RRT 线程 × (LECT snapshot + collision checker + RRT tree) ≈ 15×1.5GB
- 加上 LECT 主缓存 (165MB) + box 数据 → 超过 31GB RAM → SIGKILL (exit 137)
- **修复**: 将 bridge 线程数限制为 seed root 数量 (7-8)

### 1.2 CritSample vs IFK 对比 (历史数据)
| Metric | IFK+LinkIAABB | CritSample+Hull16_Grid |
|--------|---------------|------------------------|
| vol    | 1365          | 431                    |
| vcov   | 2.07%         | 0.65%                  |
| time   | 3.3s          | 13.3s                  |
| 原因   | IFK 宽松 → 大 box | CritSample 精确 → 小 box, 但更安全 |

### 1.3 V6 缓存的优势
- Z4 EP 持久化缓存已实现 → CritSample 结果持久化, 第二次运行无需重算
- Source isolation: IFK/CritSample 缓存条目互不干扰
- 第一次运行慢 (需计算 CritSample EP), 后续运行快 (mmap 命中)

## 2. 优化策略

### Phase A: 修复 OOM (必须先做)
1. `bridge_all_islands()` 增加 `int n_threads = 0` 参数
2. 当 `n_threads > 0` 时使用指定值; `n_threads == 0` 时回退到 `min(hw_threads-1, 8)`
3. `sbf_planner.cpp` 传 `config_.grower.n_threads` 给 bridge
4. 预期: 7 线程 vs 15 线程 → 内存减半, 不再 OOM

### Phase B: 切换 CritSample + Hull16_Grid
1. `exp1_coverage.cpp` make_config() 改用 CritSample + Hull16_Grid
2. voxel_delta = 0.02 (20mm 网格), n_samples_crit = 1000
3. 删除旧的 IFK V6 EP 缓存 (`~/.sbf_cache/bc1d61198ab31e09/ep_v6.cache`)
4. 第一次运行: 构建 CritSample EP 缓存 (慢, 但只需一次)

### Phase C: 优化 Preset 参数
CritSample 产生更紧的 envelope → box 更小 → 需要更多 box 来覆盖同样体积

**新 Preset 策略:**
- 增加 max_boxes (2500→3500) 补偿 box 体积减小
- 保持 min_edge=0.04 (太小会导致 tiny box)
- 多阶段 wavefront: 从粗到细, 先铺大 box 再填缝隙
- 激进 coarsen: target=200-300, 利用 CritSample 的精确性合并更多邻居
- enable_promotion: 允许小 box 在 LECT 中晋升到更大区域

**新 Preset I (CritSample Optimal):**
```
max_boxes=3000, min_edge=0.04
stages: {0.2→300}, {0.1→1200}, {0.05→2200}, {0.04→3000}
coarsen_target=250, max_consecutive_miss=300
n_samples_crit=1000, voxel_delta=0.02
```

**新 Preset J (CritSample Lite):**
```
max_boxes=2000, min_edge=0.04
stages: {0.2→200}, {0.1→800}, {0.05→1500}, {0.04→2000}
coarsen_target=150, max_consecutive_miss=200
```

### Phase D: 结构性优化
1. **提高 FFB 深度**: CritSample 更紧 → 需要更深的树才能生成同样大的 box
   - `ffb_config.max_depth = 70` (从 60 增加)
2. **增加 RRT step_size**: 更大步长加速 bridge RRT
3. **放宽 coarsen score_threshold**: CritSample box 更精确, 可以更激进合并
   - score_threshold: 500→200
4. **增加 cluster merge size**: max_cluster_size 12→16

## 3. 执行计划

| Step | 内容 | 预期效果 |
|------|------|----------|
| 1 | 修复 bridge_all_islands OOM | 不再 SIGKILL |
| 2 | exp1 切换 CritSample + Hull16_Grid | envelope 更精确 |
| 3 | 新增 Preset I/J, 优化参数 | 补偿 box 体积缩小 |
| 4 | 第一次运行 (EP 冷启动) | 构建 V6 EP cache |
| 5 | 第二次运行 (EP 热缓存) | 验证性能提升 |
| 6 | 根据结果迭代调参 | 最终覆盖率/时间/box数 |

## 4. 成功指标
- OOM: 不再 SIGKILL
- 覆盖率 (vcov): > 1.5% (目标追平 IFK 的 2.07%)
- 时间: < 15s (热缓存)
- box 数量: < 300 (coarsen 后)
- 连通性: 1 island (bridge 成功)
- 有效率: > 99% (box 内采样 collision-free)
