# Pipeline Benchmark v3 改进计划

> 基于 v2 实验结果（2026-03-12）的深度优化分析
> v3 改进实施完成（2026-03-13）
> 代码库：`safeboxforest/v3/`

---

## v3 基线数据（IIWA14, 50 trials, 随机分布, Batch 3）

| Source | Envelope | n_sub | Vol (m³) | Dedup Vol (m³) | Cold (ms) | Warm (ms) |
|---|---|---|---|---|---|---|
| IFK | SubAABB | 1 | 1.322 | **1.190** | 0.008 | <0.001 |
| IFK | SubAABB_Grid | 16 | 1.179 | — | 0.063 | 0.054 |
| IFK | Hull16_Grid | 1 | 1.118 | — | 1.987 | 2.043 |
| CritSample | SubAABB | 1 | 0.257 | **0.219** | 0.159 | <0.001 |
| CritSample | SubAABB_Grid | 16 | 0.208 | — | 0.220 | 0.052 |
| CritSample | Hull16_Grid | 1 | 0.182 | — | 0.718 | 0.500 |
| AnalyticalCrit | SubAABB | 1 | 0.263 | **0.226** | 265.4 | <0.001 |
| AnalyticalCrit | SubAABB_Grid | 16 | 0.214 | — | 265.5 | 0.045 |
| AnalyticalCrit | Hull16_Grid | 16 | 0.180 | — | 276.5 | 12.202 |

### v2→v3 主要变化
- **P0-1**: Hull16_Grid safety_pad 从 0.0 改为 -1.0（√3·Δ/2 自动计算）
- **P0-3**: derive_aabb_critical 预分配 FK buffer（消除堆分配）
- **P1-8**: fk_transforms_inplace 消除冗余 4×4 拷贝
- **P2-4**: Trial 宽度分布从确定性轮转改为随机 {40%narrow, 40%medium, 20%wide}
- **P2-5**: HP sweep 从 min(N,20) 增加到 min(N,30)
- **P2-8**: CSV 输出增加 stddev 列
- **认证修正**: benchmark 禁用 certified mode（认证将 AnalyticalCrit 膨胀到 IFK 边界，掩盖分析优势）

### 关键发现
1. **AnalyticalCrit 略宽于 CritSample**（SubAABB: +2.2%, Grid: +3.1%）——这是正确行为，说明解析求解器找到了更多真实极值
2. **CritSample 捕获 >97.8% 的极值点**，是实用的最优选择
3. **Hull16_Grid 带 safety_pad 后体积和磁盘增大**（CritSample: 0.182m³/36KB vs 旧 0.173m³/2KB），但保证保守性
4. **IFK Hull16 变慢**（2.1ms vs 旧 0.085ms），因为 interval FK 的宽边界 + safety_pad 产生大量体素

---

## 已完成项目

### ✅ P0-1: Hull16_Grid safety_pad=0 导致非保守结果
- **文件**: `experiments/exp_pipeline_benchmark.cpp` L316, L436
- **修复**: `VoxelGrid` 构造改 `0.0` 为 `-1.0`（触发 √3·Δ/2 自动计算）

### ✅ P0-2: AnalyticalCrit+Hull16_Grid 统一使用 fill_hull16
- **文件**: `src/forest/lect.cpp` — `derive_hull_grid()`
- **原问题**: AnalyticalCrit+Hull16 实际调用 `fill_aabb` 而非 `fill_hull16`
- **修复**: 重写 `derive_hull_grid()` 主路径，统一使用 `fill_hull16`
  从 per-link 近端/远端 endpoint-AABB 对直接构造 16 点凸包
  Conv(B_prox ∪ B_dist) ⊕ Ball(r)，不再经过 sub-AABB 中间步骤。
  `n_sub` 参数对 Hull16_Grid 不再生效。
- **效果**: Hull16_Grid 无缓存计算 3-4× 加速（IFK: 18.7s→4.5s, Crit: 10.5s→3.1s）

### ✅ P0-3: derive_aabb_critical 堆分配消除
- **文件**: `src/envelope/envelope_derive_critical.cpp` L175-192
- **修复**: 替换 `fk_link_positions` 为预分配 `tf_buf` + `fk_transforms_inplace`

### ✅ P1-8: fk_transforms_inplace 冗余拷贝消除
- **文件**: `src/robot/fk_scalar.cpp` L70-95
- **修复**: 去掉局部变量 `T`，直接 `buf[idx] = buf[idx-1] * dh_transform(...)`

### ✅ P2-4: Trial 宽度分布随机化
- **文件**: `experiments/exp_pipeline_benchmark.cpp` L530-538
- **修复**: `std::discrete_distribution<>({40, 40, 20})` 替代 `i%5`

### ✅ P2-5: HP sweep 增加 trial 数
- **文件**: `experiments/exp_pipeline_benchmark.cpp` L600
- **修复**: `n_sweep = std::min(N, 30)` (was 20)

### ✅ P2-8: 添加方差统计
- **文件**: `experiments/exp_pipeline_benchmark.cpp`
- **修复**: 追踪 `sum_volume_sq`, `sum_total_sq`；CSV 输出 `std_volume_m3`, `std_total_ms`

### ✅ 认证修正: 禁用 certified mode
- **文件**: `experiments/exp_pipeline_benchmark.cpp` L174
- **根因**: 认证步骤将分析边界膨胀到 interval FK 外边界，使 AnalyticalCrit ≡ IFK
- **修复**: `cfg.certified = false`（benchmark 测量原始分析精度，生产用认证保安全）

### ✅ P1-5: FlatBrickMap 替换 std::unordered_map
- **文件**: `include/sbf/voxel/voxel_grid.h`
- **方案**: 自实现 open-addressing 线性探测 hash map（FlatBrickMap），power-of-2 表，70% 最大负载因子
- **细节**: Entry/ConstEntry 结构体支持 C++17 structured bindings，BrickCoordHash (FNV-1a) 复用
- **验证**: 构建成功，5-trial 结果与旧版 bit-for-bit 一致

### ✅ P1-1: SubAABB 体积去重度量
- **文件**: `experiments/exp_pipeline_benchmark.cpp`
- **方案**: 新增 `compute_dedup_volume()` 辅助函数，将 sub-AABBs 光栅化到 R=64 byte grid 取 popcount
- **结果**: CSV 新增 `avg_dedup_volume_m3` 列；去重消除 10-15% 重叠
  - IFK SubAABB: 1.322 → **1.190** m³ (去重 10.0%)
  - CritSample SubAABB: 0.257 → **0.219** m³ (去重 14.8%)
  - AnalyticalCrit SubAABB: 0.263 → **0.226** m³ (去重 14.1%)
  - Hull16 仍比去重后 SubAABB 紧凑 6-20%

### ✅ P1-2: Hull16 自适应 per-link n_sub
- **文件**: `experiments/exp_pipeline_benchmark.cpp` Hull16_Grid case
- **方案**: `link_nsub = min(n_sub, ceil(diag / delta))`，短 link 自动减少细分
- **备注**: 当前 HP sweep 选出 n_sub=1 (IFK/CritSample)，自适应不触发；Analytical path 使用 fill_aabb 不受影响。作为鲁棒性改进保留

---

## 待实施项目

### 🔲 P1-6: fill_hull16 scanline 缓存优化
- **备注**: 需要重构 find_t_range_yz piecewise quadratic solver，风险高、收益不确定，暂缓

### 🔲 P3-2: AVX2/512 加速 BitBrick 操作
### 🔲 P3-3: Per-link 稀疏 grid 存储
### 🔲 P3-4: 关联端点对（correlated endpoint pairs）
### 🔲 P3-5: 二级分辨率碰撞 grid

---

## 实施日志

| 日期 | 完成项 | 备注 |
|---|---|---|
| 2026-03-12 | v2 实验完成，计划创建 | 基线 v2 数据已产出 |
| 2026-03-13 | Batch 1+2 全部完成 | P0-1/2/3, P1-8, P2-4/5/8, 认证修正 |
| 2026-03-13 | v3 benchmark 完成 | 50 trials, 新数据见上方表格 |
| 2026-03-13 | 论文 Table I 更新 | 含新叙事：AnalyticalCrit 验证 CritSample |
| 2026-03-14 | Batch 3 完成 | P1-5 FlatBrickMap, P1-1 dedup volume, P1-2 adaptive n_sub |
| 2026-03-14 | Batch 3 benchmark 完成 | 50 trials, dedup vol 消除 10-15% 重叠 |
| 2026-03-15 | Phase 3 架构重组 | LECT: scene grid pre-rasterization (2272× 加速), hull coarsening (zero-loss merge), collides_scene 两阶段 API |
| 2026-03-16 | 论文全面更新 | 8 页 403KB, 所有 TODO 已填写 (Abstract/Intro/Related/SBF Architecture V.2-V.6/Planning VI/Scalability VII.2/Conclusion), 0 个未定义引用 |
| 2026-03-16 | pybind11 绑定 | pysbf3 模块完成 (603KB .pyd), 绑定 18+ 类/枚举, 全集成测试通过 |
