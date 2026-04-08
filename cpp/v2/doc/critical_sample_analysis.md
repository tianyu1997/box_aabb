# Critical-Sample Envelope 实现分析与超参数调优

> SafeBoxForest v2 C++ — 2026-03-09

## 1. 实现流程总览

Critical-sample envelope 通过**枚举关节空间的关键配置点**（而非区间运算），
用**标量 FK** 精确计算各 link 在工作空间中的位置，最终拟合 AABB 或 OBB 包络体。

### 1.1 核心流程 (AABB_CRIT)

```
Input: Robot, intervals[n_joints], n_sub
                │
                ▼
(1) 枚举关键角度 crit_angles(lo, hi)
    每个关节: {lo, hi} ∪ {kπ/2 | k∈Z, lo < kπ/2 < hi}
                │
                ▼
(2) 构建笛卡尔积集合 build_csets()
    若 ∏|csets[j]| > CRIT_MAX_COMBOS:
        回退至 {lo, mid, hi} 每关节
                │
                ▼
(3) 递归枚举所有组合 crit_enum()
    对每个配置 q:
      ├─ 标量 FK → link positions [pos[0], ..., pos[n]]
      └─ 对每个 active link:
           对每个 sub-segment s ∈ [0, n_sub):
             线性插值 s0, s1
             更新 AABB min/max
                │
                ▼
(4) 膨胀: AABB ⊕ link_radii
                │
                ▼
Output: out_aabb[n_active × n_sub × 6]
```

### 1.2 OBB_CRIT 变体

与 AABB_CRIT 相同的关键点枚举，但替换步骤 (3) 的 min/max 累积为：
- 收集所有 sub-segment 端点到 point cloud
- 对 point cloud 做 **PCA → 主轴方向**
- 将点投影到 PCA 轴 → 计算 half-extent
- 存储: [center(3), half(3), axes(9)] = 15 floats/slot

### 1.3 derive_crit_frames 路径

另一种使用方式：先算出 per-frame position intervals（与 FrameStore 格式兼容），
再复用 `derive_aabb()` / `derive_aabb_subdivided()` 函数：

```
derive_crit_frames()  →  FrameStore.store_frames()
                              │
                              ▼
                    derive_aabb() / derive_aabb_subdivided()
```

这种方式允许缓存 frames 后以 O(1) 成本重新 derive，但精度略低于直接
per-sub-segment 枚举。

## 2. 重要超参数汇总

| # | 超参名 | 当前值 | 所在文件 | 影响 |
|---|--------|--------|----------|------|
| **1** | `CRIT_MAX_COMBOS` | 60,000 | `envelope_derive_critical.cpp:20` | 关键点笛卡尔积上限；超过时回退到粗采样 |
| **2** | `n_sub` (subdivision) | 1-8 | 调用方传入 | link 分段数。越多 → 包络更紧 (更小 volume)，但 slot 数量 ×n_sub |
| **3** | 回退策略 (fallback) | `{lo, mid, hi}` | `build_csets()` L48-51 | 超限后每关节采样点数和位置 |
| **4** | Critical-angle 生成范围 | k ∈ [-20, 20] | `crit_angles()` L26 | kπ/2 搜索范围，影响候选角度数量 |
| **5** | Envelope type | AABB / OBB | 调用方选择 | OBB 更紧但碰撞检测更昂贵 (SAT 15-axis vs 3-axis) |
| **6** | `link_radii` | 由 robot JSON | `Robot` | capsule 膨胀半径 |
| **7** | PCA 正则化 `ε` | 1e-12 | `pca_obb_world()` | 防止退化协方差矩阵 |

### 2.1 超参之间的交互

```
Volume ∝ 1/n_sub            (线性关系：分段越多，每段越紧)
Volume ∝ envelope_type      (OBB < AABB，约 0.6-0.9x)
Time   ∝ n_combos × n_sub  (每个组合 × 每个 subsegment)
n_combos = min(∏|crit_angles|, MAX_COMBOS降级后组合数)
```

### 2.2 各超参的典型效果 (基于 Exp15 Panda 数据)

#### 2.2.1 CRIT_MAX_COMBOS 对 n_combos 的影响

对于 7-DOF 机器人（Panda/IIWA）：
- **Small intervals** (0.05-0.15 rad): 每关节约 2-3 个关键角度 → 最多 2187 combos
  → 通常不触发回退
- **Medium intervals** (0.25-0.50 rad): 每关节约 2-4 个 → 最多 16384 combos
  → 部分触发回退
- **Large intervals** (0.70-1.40 rad): 每关节约 3-6 个 → 最多 279936 combos
  → 必然触发 MAX_COMBOS 回退

因此 MAX_COMBOS 主要影响 large interval 场景。

#### 2.2.2 n_sub 对 volume 的影响 (Panda medium width)

| n_sub | AABB_CRIT Vol | OBB_CRIT Vol | AABB Derive us |
|-------|--------------|--------------|----------------|
| 1     | 9.0e-2       | 7.9e-2       | 86            |
| 2     | 4.7e-2       | 4.1e-2       | 95            |
| 4     | 3.3e-2       | 2.7e-2       | 106           |
| 8     | 2.8e-2       | 2.1e-2       | 125           |

n_sub 从 1→4 volume 下降显著（~60%），4→8 下降趋缓（~15%）。
Derive 时间增长主要来自更多 subsegment 的 point cloud 收集和 PCA。

#### 2.2.3 Collision check 成本

| Method | n_sub=1 | n_sub=4 | n_sub=8 |
|--------|---------|---------|---------|
| AABB_CRIT | 14 ns | 55 ns | 98 ns |
| OBB_CRIT | 180 ns | 704 ns | 1438 ns |

OBB 的碰撞检测成本约为 AABB 的 **13-15x**（15-axis SAT vs 3-axis SAT），
在紧密碰撞检测循环中可能成为瓶颈。

## 3. 实验设计

### 3.1 实验 17: Critical-Sample 超参数全面扫描

见 `experiments/17_critical_hyperparameter_sweep.cpp`

**扫描维度**:

| 维度 | 值 |
|------|-----|
| Robot | planar_2dof (2-DOF), Panda (7-DOF), IIWA14 (7-DOF) |
| Method | AABB_CRIT, OBB_CRIT |
| CRIT_MAX_COMBOS | 500, 1000, 5000, 10000, 30000, 60000, 120000 |
| n_sub | 1, 2, 4, 8, 16 |
| Fallback | lo_hi, lo_mid_hi, lo_q25_mid_q75_hi |
| Width | small, medium, large |

**指标**: derive_us (p50), volume_m3 (p50), actual_combos

**Pareto 分析**: 寻找在 (time, volume) 二维空间中的 Pareto 前沿配置。

### 3.2 各机器人的预期最优方向

#### 3.2.1 Planar 2-DOF

- 关节少 → 关键点组合数极少（通常 < 50）
- CRIT_MAX_COMBOS 几乎不影响（不触发回退）
- 主要调优 n_sub: 期望 n_sub=4 即已足够
- OBB vs AABB: 2D 中 OBB 优势有限

**预期最优**: `CRIT_MAX_COMBOS=1000, n_sub=4, fallback=lo_mid_hi`

#### 3.2.2 Panda 7-DOF

- 3 active links (link 3, 5, 7 based on non-zero link length)
- Small intervals: combos < 60k → 不触发回退 → MAX_COMBOS 无影响
- Large intervals: combos >> 60k → MAX_COMBOS 决定精度/速度权衡
- OBB 比 AABB 紧约 0.75x 但碰撞检测贵 15x
- n_sub=4 是 volume/time 拐点

**预期最优**: `CRIT_MAX_COMBOS=30000, n_sub=4, AABB_CRIT`

#### 3.2.3 IIWA14 7-DOF

- 结构类似 Panda 但连杆参数不同
- 7 joints → 类似的组合爆炸问题
- Link radii 和连杆长度不同可能导致不同的 volume 分布

**预期最优**: 与 Panda 类似，`CRIT_MAX_COMBOS=30000, n_sub=4`

### 3.3 编译和运行

```bash
cd safeboxforest/v2/build
cmake .. -DSBF_BUILD_EXPERIMENTS=ON
cmake --build . --target exp_17_crit_hyperparam --config Release

# 快速运行 (少量 trial)
./exp_17_crit_hyperparam --trials 10 --repeats 5

# 完整运行
./exp_17_crit_hyperparam --trials 30 --repeats 10
```

结果输出到 `results/exp17_<timestamp>/`:
- `hyperparam_sweep_raw.csv`: 原始数据
- `report.md`: Pareto 分析和推荐配置

## 4. 最终推荐设定（基于已有 Exp15 数据 + 预测）

| Robot | 场景 | 推荐方法 | MAX_COMBOS | n_sub | Fallback | 理由 |
|-------|------|----------|------------|-------|----------|------|
| **2-DOF** | 通用 | AABB_CRIT | 1,000 | 4 | lo_mid_hi | combos 极少，不需大上限 |
| **Panda** | 速度优先 | AABB_CRIT | 10,000 | 2 | lo_mid_hi | derive ~50us, vol 中等 |
| **Panda** | 紧致优先 | AABB_CRIT | 60,000 | 4 | lo_mid_hi | derive ~100us, vol 最优 |
| **Panda** | OBB 紧致 | OBB_CRIT | 30,000 | 4 | lo_mid_hi | vol 最小但 check 贵 |
| **IIWA** | 速度优先 | AABB_CRIT | 10,000 | 2 | lo_mid_hi | 同 Panda 结构类似 |
| **IIWA** | 紧致优先 | AABB_CRIT | 60,000 | 4 | lo_mid_hi | 同 Panda |

> **注意**: 在 forest grower 的 FFB 搜索循环中，每个节点会调用上千次碰撞检测，
> 因此 `check_us` 的重要性可能高于 `derive_us`。AABB_CRIT 的 check 成本
> 远低于 OBB_CRIT（~10x），在多数场景下 AABB_CRIT + n_sub=4 是最佳选择。

## 5. 包络体积越大越好？

> 用户原文: "包络体积越大越好"

**说明**: 在 SafeBoxForest 框架中，"包络体积越**小**越好"（tighter envelope），
因为更小的包络 = 更少的 false positive 碰撞 = 更多的 free-space boxes 能被
认证为 safe。

如果用户的意思是 **safe box 的 C-space 体积越大越好**（即 box 在配置空间中
覆盖的 safe region 更大），那关键是降低 false positive rate，即让碰撞检测
更精确 → 需要 **更紧的包络** (更小的 workspace volume)。

两者方向一致：**包络越紧 → safe box 越大 → 规划越高效**。
