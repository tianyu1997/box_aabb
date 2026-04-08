# 实验计划：`exp_ep_source_gap`

## 一、实验目标

量化 **CritSample / Analytical / MC参考基线** 三种端点源方法，对每个活跃连杆在各轴方向上的 link iAABB 边界与参考值的差距（**signed gap**），从而：

1. 定量评估 CritSample 的危险性（是否存在负 gap = 漏判区域）
2. 定量评估 Analytical 的保守代价（正 gap = 过度膨胀）
3. 不同区间宽度、不同连杆、不同方向下的 gap 分布特征

---

## 二、"Gap"指标定义

**参考基线**：对给定区间 B，做 N_mc 次均匀随机采样 {q_i} ~ Uniform(B)，对每个连杆 l、每轴 d ∈ {x,y,z} 计算：

```
ref_hi[l][d] = max_i p_{l,d}(q_i) + r_l
ref_lo[l][d] = min_i p_{l,d}(q_i) - r_l
```

其中 p_{l,d}(q) 为连杆端点 FK 位置在 d 轴的投影，r_l 为连杆半径。

**Signed gap**（符号差距）：
```
g_hi[l][d] = method_hi[l][d] - ref_hi[l][d]
g_lo[l][d] = ref_lo[l][d]    - method_lo[l][d]
```

| 值   | 含义                                                              |
|------|-------------------------------------------------------------------|
| g > 0 | 保守（方法边界在参考之外，安全）                                 |
| g < 0 | 欠估（边界在参考之内；对 SAFE 方法不应出现；对 CritSample 是已知风险） |
| g = 0 | 与参考基线完全一致                                               |

**每试验汇总指标**：
- `max_gap`：各连杆各轴各方向中 g 的最大值（越小越紧）
- `min_gap`：最小值（负值 = 存在漏判，越负越危险）
- `mean_gap`：平均保守程度

---

## 三、被比较的三种方法

| 方法名       | API 入口                                                                                          | 安全类  | 备注                             |
|--------------|---------------------------------------------------------------------------------------------------|---------|----------------------------------|
| CritSample   | `EndpointSourceConfig::crit_sampling()` → `compute_endpoint_iaabb` → `extract_link_iaabbs`       | UNSAFE  | boundary_only（无 GCPC 缓存）    |
| Analytical   | `EndpointSourceConfig::analytical()` + `AnalyticalCriticalConfig::all_enabled()` → `compute_endpoint_iaabb` → `extract_link_iaabbs` | SAFE | 完整解析器 |
| MC（参考）   | `compute_mc_link_iaabb(robot, intervals, n_mc, seed, out)` ← **新增**                            | 参考    | 不进方法对比，只作基线           |

### MC API（新增 `include/sbf/envelope/mc_envelope.h`）

```cpp
// 新增公共 API
void compute_mc_link_iaabb(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int        n_mc,       // 采样点数，建议 50000
    uint32_t   seed,
    float*     out_iaabb); // [n_active × 6]
```

---

## 四、C++ 实验设计

**文件**：`experiments/exp_ep_source_gap.cpp`
**构建目标**：`exp_ep_source_gap`
**运行**：`.\build\Release\exp_ep_source_gap.exe [n_trials] [seed] [out_dir]`

### 4.1 实验参数

```
Robot:       iiwa14
n_trials:    200（默认）
seed:        42（默认）
n_mc:        50000（每 trial 的 MC 采样数，固定）
mc_seed:     seed + trial * 1000
宽度分类:    narrow(5-15%) / medium(20-40%) / wide(50-100%)
n_sub:       1（sub=1 LinkIAABB，三种方法可直接比较）
```

### 4.2 每 trial 流程

```
1. 生成随机区间 intervals（BoxGenerator，同 bench）
2. [参考] compute_mc_link_iaabb(robot, intervals, 50000, mc_seed, mc_aabb)
3. [CritSample] compute_endpoint_iaabb(cfg_crit, ...) → extract_link_iaabbs
4. [Analytical] compute_endpoint_iaabb(cfg_anal, ...) → extract_link_iaabbs
5. 计算 signed gap（逐连杆、逐轴、逐方向 hi/lo）
6. 写 CSV 行
```

### 4.3 CSV 输出格式

```
source,trial,width_frac,link,axis,dir,gap_m,mc_bound,method_bound
```

---

## 五、Python 可视化设计

**文件**：`scripts/viz_ep_source_gap.py`
**输出**：`results/exp_ep_source_gap_<timestamp>/`

| # | 文件名                   | 内容                                                            |
|---|--------------------------|-----------------------------------------------------------------|
| 1 | `gap_dist.html`          | CritSample/Analytical 的 gap 分布箱线图（全部聚合），区分 hi/lo |
| 2 | `gap_by_link.html`       | 按连杆分组的 gap 均值±std 条形图                                |
| 3 | `gap_by_axis.html`       | 按轴（x/y/z）分组的箱线图                                       |
| 4 | `gap_vs_width.html`      | gap 随区间宽度的散点图（narrow/medium/wide）                    |
| 5 | `neg_gap_heatmap.html`   | CritSample 负 gap 热力矩阵（x=连杆，y=轴向，color=min_gap）     |

---

## 六、文件变更清单

| 文件 | 类型 | 说明 |
|------|------|------|
| `include/sbf/envelope/mc_envelope.h`    | **新增** | `compute_mc_link_iaabb()` 声明 |
| `src/envelope/mc_envelope.cpp`          | **新增** | 实现（FK 循环 + union + radius） |
| `CMakeLists.txt`                        | **修改** | 加入 mc_envelope.cpp；新增 exp_ep_source_gap 目标 |
| `experiments/exp_ep_source_gap.cpp`     | **新增** | 主实验 |
| `scripts/viz_ep_source_gap.py`          | **新增** | 可视化脚本 |
| `tests/test_mc_envelope.cpp`            | **新增** | mc_envelope 单元测试 |
| `doc/CHANGE_LOG_CN.md`                  | **修改** | 变更记录 |

---

## 七、预期结果

| 方法       | 预期 min_gap     | 预期 max_gap | 含义                                   |
|------------|------------------|--------------|----------------------------------------|
| Analytical | ≥ 0（理论保证）  | 正值         | 总是包含 MC 基线                       |
| CritSample | **可能 < 0**     | 小正值       | 窄区间、高 DOF 方向上可能漏判         |

若 CritSample 的 `min_gap < -0.001 m`，即可直接量化其实际欠估风险。
