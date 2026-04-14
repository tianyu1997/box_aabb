# SafeBoxForest v6 论文丰满化——详细实施计划

> **目标期刊**: IEEE Transactions on Robotics (T-RO)
> **当前状态**: 英文 7 页 / 中文 14 页, 7 张表, **0 张图**, 3 条定理 (仅声明), 23 条参考文献
> **目标状态**: 16–20 页 (双栏), 7–10 张图, 8–10 张表, ≥2 种机器人, ≥3 种场景, ≥10 种子, 完整消融覆盖
> **论文路径**: `cpp/v6/doc/box_aabb_v6_paper_en.tex` / `box_aabb_v6_paper_zh.tex`

---

## 总体路线图

| 阶段 | 名称 | 目标 | 预计工期 | 前置依赖 |
|------|------|------|----------|----------|
| P1 | 图表生成 | 7 张核心图 | 3–5 天 | 无 |
| P2 | 消融补全 | 4 组新消融 + 统计增强 | 5–7 天 | 无 |
| P3 | 场景/机器人扩展 | Panda 适配 + 多场景数据 | 7–10 天 | 无 |
| P4 | 基线对比 | IRIS-NP / GCS / OMPL 完整对比 | 10–14 天 | P3 (场景) |
| P5 | 理论 + 扩写 | 完善证明, 扩写 Related Work/Discussion | 5–7 天 | P1–P4 数据 |

**关键路径**: P1 与 P2/P3 可并行; P4 依赖 P3 场景; P5 在所有数据就绪后进行。

---

## Phase 1: 图表生成 (最高优先级)

**目标**: 从 0 张图增至 7 张高质量图; 这是论文被拒的首要原因。

### Fig 1: 系统流程图 (System Overview)

- **类型**: TikZ 流程图
- **内容**: 离线阶段 (DH→FK→Envelope→LECT→FFB→RRT→Coverage) → 在线阶段 (Locate→A*→Smooth→Path)
- **数据来源**: 不需要实验数据, 基于代码架构
- **对应论文位置**: `\section{Introduction}` 或独立为 `\section{System Overview}` 之后
- **模块映射** (参考 `cpp/v6/include/sbf/` 10 个模块):
  - `core` → DH / FK / Geometry
  - `envelope` → IFK / CritSample / Analytical / BnB
  - `voxel` → BitMask / Hull16
  - `lect` → LECT 缓存树
  - `ffb` → Find Free Box
  - `forest` → RRT / Bridge / Coverage
  - `planner` → A* / Smooth / Query Pipeline
  - `adapters` → Drake / OMPL 接口
- **实施步骤**:
  1. 在 `box_aabb_v6_paper_en.tex` 中添加 TikZ 流程图代码
  2. 使用 `\tikzstyle` 定义离线/在线两种色调 (蓝系=离线, 绿系=在线)
  3. 箭头标注关键数据流 (JSON config → Forest object → Query result)
- **验收标准**: 编译通过; 双栏宽 (`\begin{figure*}`)

### Fig 2: C-space 盒森林可视化 (Box Forest Visualization)

- **类型**: 2D/3D 散点图 + 盒体
- **内容**: 2-DOF 简化场景下的 C-space 盒分布, 展示 coverage 树与盒体分裂层级
- **数据来源**: 运行 `exp1_coverage` 的 2-DOF 模式, 输出 JSON → Python 绘图
- **工具链**:
  - C++: `cpp/v6/build/experiments/exp1_coverage --scene simple_2d --output /tmp/coverage_2d.json`
  - Python: 基于 `v4/viz/aabb_viz.py` 或 `v4/viz/scene_viz.py` 改写
  - 绘图库: matplotlib (已安装 v3.10.8)
- **实施步骤**:
  1. 确认 `exp1_coverage` 支持 2-DOF 模式并输出盒体 JSON (检查 `cpp/v6/experiments/exp1_coverage.cpp` 605行源码)
  2. 如不支持, 新增一个轻量 `exp_viz_boxes.cpp` 专门输出 2D 盒体数据
  3. 编写 `cpp/v6/scripts/plot_fig2_boxes.py`:
     - 加载 JSON, 每个盒绘为半透明矩形
     - 颜色编码: 按分裂深度/所属树着色
     - 叠加 C-space 障碍边界 (碰撞区域填灰)
  4. 输出 PDF/PGF, 嵌入 LaTeX
- **验收标准**: 能清晰看到盒体覆盖自由空间, 不同树用不同颜色; 单栏宽

### Fig 3: 包络紧凑性对比 (Envelope Tightness)

- **类型**: 分组柱状图 (Grouped Bar Chart)
- **内容**: 3 种 endpoint source × 3 种 envelope 方式 的 volume 对比
- **数据来源**: `cpp/v6/experiments/results_backup_pre_accel/s1_envelope_tightness/results.json`
  - 24 行数据, 字段: `robot, endpoint, envelope, volume_mean, volume_std, time_us_mean, n_boxes, ratio_pct`
- **实施步骤**:
  1. 编写 `cpp/v6/scripts/plot_fig3_envelope.py`:
     ```python
     import json, matplotlib.pyplot as plt, numpy as np
     data = json.load(open('cpp/v6/experiments/results_backup_pre_accel/s1_envelope_tightness/results.json'))
     # 筛选 robot=panda (7-DOF) 的数据
     # X轴: endpoint source (IFK, CritSample, Analytical)
     # 每组 3 条: LinkIAABB, LinkIAABB_Grid, Hull16_Grid
     # Y轴: volume (m³)
     # 误差棒: volume_std
     ```
  2. 双 Y 轴版本: 左轴 volume, 右轴 computation time
  3. 输出 `fig3_envelope.pdf`
- **验收标准**: 清晰展示 CritSample ≈ Analytical << IFK 的 volume 差距

### Fig 4: 查询路径 3D 可视化 (Query Path in Workspace)

- **类型**: 3D 机器人构型序列 / 工作空间路径
- **内容**: IIWA14 在 combined 场景 (16 obstacles) 中的查询路径, 展示起止构型和中间 waypoints
- **数据来源**: 运行 `exp2_e2e_planning` 的 CS→LB 查询对, 输出路径 waypoints
- **工具链**:
  - 方案 A (优先): 使用 `v4/viz/planner_viz.py` + `v4/viz/aabb_viz.py` 渲染
  - 方案 B: 使用 pydrake `MeshcatVisualizer` 渲染 (pydrake 已部分安装)
  - 方案 C: matplotlib 3D scatter + 透明障碍体
- **实施步骤**:
  1. 修改 `exp2_e2e_planning` 添加 `--dump-path /tmp/path_csLB.json` 参数, 输出路径点
  2. 编写/改写 `cpp/v6/scripts/plot_fig4_path.py` 加载路径 + 场景, 3D 渲染
  3. 可选: 半透明叠加多个机器人构型 (ghosted robot)
- **验收标准**: 能看到机器人从 CS→LB 的运动, 无碰撞穿越 shelves

### Fig 5: 可扩展性曲线 (Scalability)

- **类型**: 多子图折线图 (3 子图: DOF / 障碍数 / Budget)
- **内容**: Build time 和 Query time 随参数变化的趋势
- **数据来源**: `cpp/v6/experiments/results_backup_pre_accel/s5_scalability/`
  - `s5a_2dof/`, `s5a_7dof/`: DOF 维度
  - `s5b_1obs/`, `s5b_2obs/`, `s5b_4obs/`: 障碍数维度
  - `s5c_50/`, `s5c_200/`, `s5c_500/`: Budget 维度
  - `results.json`: 汇总数据
- **实施步骤**:
  1. 编写 `cpp/v6/scripts/plot_fig5_scalability.py`:
     ```python
     fig, axes = plt.subplots(1, 3, figsize=(12, 3.5))
     # subplot 1: DOF (x=2,7) → build_time, query_time
     # subplot 2: Obstacles (x=1,2,4) → build_time
     # subplot 3: Budget (x=50,200,500) → build_time, coverage_pct
     ```
  2. 增加 DOF 数据点: 需要新增 3-DOF, 4-DOF, 5-DOF, 6-DOF 实验点 (见 P2 阶段)
  3. 误差棒: 需要多种子数据 (见 P2 阶段)
- **验收标准**: 展示 近线性 或 可控的超线性增长趋势

### Fig 6: 构建时间分解 (Build Time Breakdown)

- **类型**: 堆叠柱状图 (Stacked Bar)
- **内容**: Build 各阶段的时间占比: Envelope → LECT insert → FFB → RRT extend → Bridge
- **数据来源**: 需要增强 `exp6_build_timing` 输出分阶段时间
- **实施步骤**:
  1. 检查 `cpp/v6/experiments/exp6_build_timing.cpp` (288 行) 是否已输出分阶段时间
  2. 如果没有, 添加 per-phase timing instrumentation:
     - 在 `ForestBuilder` 的关键阶段添加 `std::chrono` 计时
     - 输出 JSON: `{envelope_ms, lect_ms, ffb_ms, rrt_ms, bridge_ms, total_ms}`
  3. 重新编译、运行实验
  4. 编写 `cpp/v6/scripts/plot_fig6_breakdown.py`
- **验收标准**: 清晰展示 Envelope 仅占 <5%, 主要时间在 RRT/Bridge

### Fig 7: 消融对比 (Ablation Comparison)

- **类型**: 分组条形图
- **内容**: P0/P2/P4 各组合下的 build time 和 query time
- **数据来源**: `output/v5v6_comparison_report.md` 中 Exp5 表格 (7 种配置)
- **实施步骤**:
  1. 编写 `cpp/v6/scripts/plot_fig7_ablation.py`:
     - X 轴: 7 种配置 (Baseline, +P0, +P2, +P4, +P0P2, +P2P4, ALL)
     - 双 Y 轴: build time (s) 和 query time (s)
     - 颜色编码: 哪些优化被启用
  2. 高亮 P4 (connect-stop) 的 5× 加速效果
- **验收标准**: 展示 P4 是最关键单一优化

### 图表生成辅助任务

- **F-A**: 建立绘图脚本框架 `cpp/v6/scripts/plot_common.py`
  - 统一配色方案 (IEEE 友好, 灰度可辨)
  - 统一字体/字号 (8pt)
  - 通用 save 函数 (PDF + PGF 双格式)
- **F-B**: 在 `CMakeLists.txt` 或 Makefile 中添加 `make plots` 目标, 一键生成所有图

---

## Phase 2: 消融实验补全

**目标**: 补齐 Z4、分裂策略、增量 FK、BitMask 四组消融; 种子数增至 10 提供误差棒。

### 2.1 Z4 旋转对称消融

- **假设**: Z4 对称利用 θ → θ+π/2 等价性减少 LECT 存储和查询次数
- **具体步骤**:
  1. 定位 Z4 相关代码:
     ```bash
     grep -rn "Z4\|z4_sym\|rotational_symmetry" cpp/v6/include/sbf/ cpp/v6/src/
     ```
  2. 确认 Z4 的开关机制 (编译期 flag 或 运行时 config)
  3. 修改 `exp5_ablation.cpp` 增加 Z4 on/off 对比组
  4. 运行实验, 收集:
     - LECT 节点数 (with Z4 vs without)
     - Build time 差异
     - Volume tightness 差异
     - Query time 差异
  5. 数据写入 JSON, 追加到 ablation 表
- **预期结论**: Z4 减少 ~75% LECT 重复计算, 对 volume 无影响
- **论文更新**: 在 Tab 4 (ablation) 中增加 Z4 行; 在 §6.4 讨论

### 2.2 分裂策略消融

- **策略候选**:
  - `ROUND_ROBIN`: 循环依次分裂各维度
  - `WIDEST_FIRST`: 选择跨度最大的维度分裂
  - `BEST_TIGHTEN`: 选择分裂后包络最紧的维度
- **具体步骤**:
  1. 定位分裂策略代码:
     ```bash
     grep -rn "split_strategy\|SplitStrategy\|ROUND_ROBIN\|WIDEST" cpp/v6/include/sbf/ cpp/v6/src/
     ```
  2. 确认是否已有 enum/config 切换机制
  3. 新建或扩展实验 `exp5_ablation.cpp`:
     - 对 combined 场景 + IIWA14
     - 分别运行 3 种策略, 各 10 seeds
     - 收集: coverage_pct, n_boxes, build_time, mean_volume, query_time, path_length
  4. 输出数据到 `results_split_strategy.json`
- **预期结论**: WIDEST_FIRST 或 BEST_TIGHTEN 优于 ROUND_ROBIN 在高维下
- **论文更新**: 新增 subsection "Split Strategy Comparison" 或合入 Tab 4

### 2.3 增量 FK 消融

- **假设**: 增量 FK 在 box 分裂时复用父节点 FK 计算, 避免从头计算
- **具体步骤**:
  1. 定位增量 FK 代码:
     ```bash
     grep -rn "incremental_fk\|IncrementalFK\|reuse_fk\|fk_cache" cpp/v6/include/sbf/ cpp/v6/src/
     ```
  2. 添加 on/off 开关 (如果没有)
  3. 在 `exp4_envelope_benchmark` 中运行两种模式
  4. 收集: envelope_time_us (with/without incremental FK)
  5. 输出 JSON
- **预期结论**: 增量 FK 节省 30-50% envelope 计算时间
- **论文更新**: 在 §4 (Envelope) 或 §10 (Experiments) 中讨论

### 2.4 BitMask (Hull16_Grid) vs LinkIAABB_Grid 消融

- **已有数据**: s1 中已有两种 envelope 的 volume 和 timing
- **补充需求**: 增加碰撞检测精度对比 (false positive rate)
- **具体步骤**:
  1. 在 `exp1_coverage` 中增加碰撞统计: 对每个 box, 记录真实碰撞检测 vs BitMask 检测的一致性
  2. 收集: false_positive_rate, false_negative_rate (应为 0, conservative), collision_check_time_ns
  3. BitMask 的空间开销 (memory_bytes per box)
- **论文更新**: 在 §5 (Sparse Voxel BitMask) 增加实验佐证

### 2.5 统计增强: 种子数 10 + 误差棒

- **适用实验**: exp5 (ablation), exp6 (build timing), exp2 (E2E)
- **具体步骤**:
  1. 修改实验脚本, 将种子数从 2 增至 10
  2. 对每个实验配置:
     - 运行 10 次 (seed 0–9)
     - 计算 mean ± std
     - 输出 min, max, median, IQR
  3. 更新所有论文表格: 在数值后添加 ± 标准差
  4. 为 Fig 5 (scalability), Fig 6 (breakdown), Fig 7 (ablation) 添加误差棒
- **预估运行时间**:
  - exp5 (7 configs × 10 seeds × ~15s build × 5 queries): ~87 min
  - exp6 (1 config × 10 seeds × ~2.5s build × 5 queries): ~5 min
  - exp2 (3 planners × 10 seeds × 2 scenes): ~30 min
  - **总计: ~120 min 实验运行时间**
- **自动化**:
  ```bash
  # cpp/v6/scripts/run_full_experiments.sh
  for seed in $(seq 0 9); do
    ./build/experiments/exp5_ablation --seed $seed --output results/exp5_seed${seed}.json
    ./build/experiments/exp6_build_timing --seed $seed --output results/exp6_seed${seed}.json
  done
  ```

---

## Phase 3: 场景 / 机器人扩展

**目标**: 从 1 个机器人 + 1 类场景扩展到 2+ 机器人 + 4 种场景。

### 3.1 Franka Panda 适配

- **现有资源**: `v4/src/aabb/configs/panda.json`
  - 字段: `name, description, dh_convention, dh_params, tool_frame, joint_limits, joint_names, coupled_pairs, coupled_triples`
- **v6 需要的格式**: 查看 `cpp/v6/` 中 IIWA14 配置嵌入方式
- **具体步骤**:
  1. 检查 v6 如何加载机器人配置:
     ```bash
     grep -rn "iiwa\|robot_config\|load_robot\|DH\|dh_params" cpp/v6/include/sbf/core/ cpp/v6/src/core/
     ```
  2. 确认 v6 是通过 JSON 文件还是硬编码加载 DH 参数
  3. **如果是 JSON**: 将 `panda.json` 复制到 `cpp/v6/src/configs/panda.json`, 适配格式差异
  4. **如果是硬编码**: 在 `core` 模块中新增 `PandaRobot` 配置, 参照 IIWA14 模式
  5. 编写单元测试验证 Panda FK 正确性
  6. 运行全套实验 (exp4, exp5, exp6, exp2) 对 Panda
- **Panda DH 参数** (from panda.json):
  - 7-DOF, MDH convention
  - active links 数量需确认 (可能 4 或 5)
- **论文更新**: 所有实验表增加 Panda 行; §10.1 Setup 增加 Panda 描述

### 3.2 多场景数据收集

- **场景定义**: `cpp/v6/experiments/marcucci_scenes.h`
  - `shelves`: 5 obstacles, 书架穿越
  - `bins`: 4 obstacles, 容器拾取
  - `table`: 3 obstacles, 桌面操作
  - `combined`: 16 obstacles, 综合场景 (当前唯一使用)
- **具体步骤**:
  1. 修改 `exp6_build_timing.cpp` 支持 `--scene` 参数 (如未支持)
  2. 逐场景运行:
     ```bash
     for scene in shelves bins table combined; do
       for seed in $(seq 0 9); do
         ./build/experiments/exp6_build_timing --scene $scene --seed $seed \
           --output results/exp6_${scene}_seed${seed}.json
       done
     done
     ```
  3. 同样修改 `exp2_e2e_planning`, `exp5_ablation` 支持多场景
  4. 收集数据, 生成 场景×机器人 交叉表
- **查询对定义**: 需要为 shelves/bins/table 单独定义查询对
  - 检查 `marcucci_scenes.h` 中的 config_C, config_L, config_R, config_AS, config_TS, config_CS, config_LB 等
  - 不同场景可能需要不同的起止构型 (部分配置在某些场景下可能碰撞)
- **论文更新**: 实验主表从单行变为 4×2 (4 场景 × 2 机器人) 矩阵

### 3.3 窄通道 / 高密度场景

- **目的**: 验证 SBF 在极端场景下的鲁棒性
- **具体步骤**:
  1. 在 `marcucci_scenes.h` 中新增:
     - `narrow_passage`: 两大障碍中间仅留 5cm 间隙
     - `dense`: 20+ 随机障碍
  2. 或使用 exp1_coverage 的随机场景生成功能
  3. 运行实验, 记录成功率和时间
- **预期**: 窄通道下 success rate < 100%, 可展示 SBF 的局限性 (honest evaluation)

---

## Phase 4: 基线对比

**目标**: 与 IRIS-NP/GCS (Drake)、OMPL (RRT/PRM/RRT*) 进行公平端到端对比。

### 4.1 OMPL 基线完善

- **当前状态**: s4_baselines 数据显示 OMPL-RRTConnect 在两个 2-DOF 场景上 0% 成功率
  - 这说明 OMPL 配置可能有问题, 或测试场景太简单/太难
- **具体步骤**:
  1. 检查 OMPL 集成代码:
     ```bash
     grep -rn "OMPL\|ompl" cpp/v6/include/sbf/adapters/ cpp/v6/src/adapters/
     ```
  2. 确认 OMPL 在 7-DOF combined 场景上的配置:
     - 碰撞检测函数是否正确挂接
     - 规划时间上限 (应 ≥ 30s)
     - State space bounds 是否匹配 joint limits
  3. 运行 OMPL 基线套件:
     - RRTConnect (双向 RRT, 默认)
     - RRT* (渐近最优)
     - PRM (概率路线图)
     - BiTRRT (如果可用)
  4. 对每种 planner, 10 seeds, 每个 query pair
  5. 收集: plan_time, path_length, success_rate, n_collision_checks
  6. 确保 **CMAKE option**: `-DSBF_WITH_OMPL=ON` 并重新编译
- **公平性**: SBF = build_time + query_time; OMPL = total_plan_time (无预处理)
- **论文更新**: 新增或扩展 Tab "E2E Comparison" (当前仅有 s3 数据)

### 4.2 IRIS-NP 基线 (Drake)

- **环境状态**: pydrake 已安装 (可 import), mosek 已安装
- **现有脚本**: `cpp/v6/scripts/gcs_pipeline.py`, `gcs_query.py`, `gcs_optimize.py`
- **现有参考**: `gcs-science-robotics/` 目录 (Marcucci 的 GCS 原始代码)
- **具体步骤**:
  1. 验证 pydrake 功能:
     ```python
     from pydrake.geometry.optimization import IrisInConfigurationSpace
     from pydrake.solvers import MosekSolver
     print(MosekSolver().enabled())  # 应返回 True
     ```
  2. 检查 `cpp/v6/scripts/gcs_pipeline.py` 的当前状态和功能
  3. 如果 pipeline 不完整, 参考 `gcs-science-robotics/gcs/` 补全:
     - IRIS-NP 区域生成 (需要 URDF + 场景)
     - GCS 路径规划 (需要 Mosek solver)
  4. 准备 IIWA14 URDF (Drake 自带, 或从 `gcs-science-robotics/models/` 获取)
  5. 复现 Marcucci 论文结果:
     - IRIS-NP: 生成凸区域 → 记录 region_time, n_regions, region_volumes
     - GCS: 在凸区域上规划 → 记录 gcs_time, path_length, path_cost
  6. 对比指标:
     | 指标 | SBF (ours) | IRIS-NP + GCS |
     |------|-----------|---------------|
     | Precompute time | build_time | iris_time |
     | Query time | query_time | gcs_time |
     | Path length | path_length | path_length |
     | Region count | n_boxes | n_iris_regions |
     | Success rate | SR% | SR% |
- **风险**: pydrake 版本可能过旧, IRIS-NP API 可能有变化
- **缓解**: 如果 Drake/IRIS 不可用, 可引用 Marcucci 论文原始数据作为 indirect comparison

### 4.3 GCS-SBF 混合管线

- **概念**: 用 SBF 生成 AABB 凸集 → 作为 GCS 的输入区域 → 求最优路径
- **具体步骤**:
  1. SBF build → 导出 AABB box 列表 (center + half-width)
  2. 将 AABB 转为 Drake `HPolyhedron`
  3. 用 `ConvexSet` 构建 GCS graph
  4. GCS shortest path (convex relaxation)
  5. 脚本: 使用/扩展 `cpp/v6/scripts/gcs_query_v2.py`
- **论文亮点**: SBF 区域覆盖 (ms级) + GCS 全局最优路径 = **最佳组合**
- **论文更新**: 在 Discussion 中作为 "SBF as GCS frontend" 讨论

---

## Phase 5: 理论补充 + 论文扩写

**目标**: 补全定理证明; 扩写 Related Work, Discussion, Conclusion 到 T-RO 标准长度。

### 5.1 定理证明补全

当前 3 条定理仅有声明 (claim), 缺少正式证明。

#### Theorem 1: Face-$k$ completeness for $k \leq 2$ (line 227)

- **声明**: 对于 ≤2 维的面, Analytical solver 的 critical-point 枚举是完备的
- **证明思路**:
  1. 1 维面: 链式曲线, 极值点满足 ∂f/∂θ = 0, 有限个解 → 完备
  2. 2 维面: 二元优化, KKT 条件 + 约束资格 → 所有 critical points 可列举
  3. 反例: k=3 时, 非线性约束可能导致遗漏
- **篇幅**: 约 0.5–1 页
- **参考**: 区间分析 (Moore, 1966), 全局优化 KKT 理论

#### Theorem 2: Monotone corner optimality (line 256)

- **声明**: BnB verifier 在单调性减少后, corner evaluation 保证最优包围
- **证明思路**:
  1. 单调维度分离: 如果 f(θ) 在某维 θ_i 上单调, 则 min/max 在端点取得
  2. 非单调维度: BnB 细分保证 ε-最优
  3. 混合: 单调维度取端点 + 非单调维度 BnB → 等价于 lower-dim BnB
- **篇幅**: 约 0.5 页

#### Theorem 3 (新增): Collision Refutation Soundness

- **声明**: 如果 box AABB 与障碍 AABB 不相交, 则该 box 内所有构型均无碰撞 (conservative)
- **证明**: AABB 是严格包络 (interval FK 上界) → 不相交 ⇒ 安全 (trivial)
- **意义**: 明确 "no false negative" 保证
- **篇幅**: 0.25 页

### 5.2 Related Work 扩写

当前约 1.5 页 (3 subsections)。目标: 2.5–3 页。

- **新增 subsection**: "Configuration Space Decomposition"
  - 综述 IRIS, IRIS-NP, IRIS-ZO, R3T, AtlasRRT
  - 与 SBF 的关系: SBF = axis-aligned vs IRIS = ellipsoidal/polytope
- **新增 subsection**: "Hierarchical Spatial Data Structures"
  - k-d tree, octree, BVH in robotics
  - SBF 的 LECT 作为 specialized BVH
- **扩展现有 subsection**:
  - Collision detection: 增加 GJK, FCL, HPP-FCL 讨论
  - Trajectory optimization: 增加 CHOMP, TrajOpt, KOMO 讨论
- **参考文献增补**: 目标从 23 条增至 40–50 条
  - [ ] Marcucci et al., GCS (Science Robotics 2024)
  - [ ] Amice et al., IRIS-NP (WAFR 2022)
  - [ ] Werner et al., IRIS-ZO (RSS 2024)
  - [ ] LaValle, Planning Algorithms (2006)
  - [ ] Karaman & Frazzoli, RRT* (IJRR 2011)
  - [ ] Ratliff et al., CHOMP (ICRA 2009)
  - [ ] Schulman et al., TrajOpt (IJRR 2014)
  - [ ] Pan et al., FCL (ICRA 2012)
  - [ ] Ericson, Real-Time Collision Detection (2004)
  - ...

### 5.3 Discussion 扩写

当前约 0.5 页。目标: 1.5–2 页。

- **新增讨论点**:
  1. **SBF vs IRIS-NP**: 计算模型差异 (interval arithmetic vs semidefinite program)
  2. **SBF 作为 GCS 前端**: AABB regions 比 IRIS ellipsoids 更易计算, 可作为 GCS warm-start
  3. **局限性** (必须诚实讨论):
     - AABB 非紧致: 不如 convex hull 或 polytope 精确
     - 维度诅咒: $2^d$ corner 指数增长
     - 无最优性保证: 路径是 feasible 但不保证 shortest
  4. **扩展方向**:
     - 动态障碍: 在线更新 LECT 节点
     - 多机器人: 组合 C-space 盒分解
     - 学习增强: 用 NN 预测 split/FFB 策略
  5. **实际部署考量**: 与 ROS2 集成, 实时性分析

### 5.4 Conclusion 扩写

当前约 0.3 页。目标: 0.5–0.75 页。

- 总结 5 大贡献
- 更新数字 (基于 P2/P3/P4 新实验数据)
- 明确 future work bullet points

### 5.5 附录扩写

- **附录 A**: 符号表 (已有, 需补全 P2-P4 新增符号)
- **附录 B**: 配置参数表 (已有, 需增加 Panda 参数)
- **新 附录 C**: 额外实验结果 (放不进正文的详细表格)
- **新 附录 D**: 复杂度分析推导

---

## 实施顺序与依赖图

```
Week 1:
  [P1: Fig1 TikZ]           ──→ 编译验证
  [P1: Fig3 envelope plot]   ──→ 已有数据, 直接绘图
  [P1: Fig7 ablation plot]   ──→ 已有数据, 直接绘图
  [P2.5: 增加种子到 10]     ──→ 运行 ~120min ──→ 更新表格

Week 2:
  [P1: Fig5 scalability]     ──→ 需 P2.5 新数据
  [P1: Fig6 breakdown]       ──→ 需代码修改 + 运行实验
  [P2.1: Z4 消融]            ──→ 代码定位 + 实验
  [P2.2: 分裂策略消融]       ──→ 代码定位 + 实验
  [P3.1: Panda 适配]         ──→ 格式转换 + FK 验证

Week 3:
  [P1: Fig2 box viz]         ──→ 需 2D 盒数据导出
  [P1: Fig4 3D path]         ──→ 需路径导出
  [P2.3: 增量 FK 消融]       ──→ 代码定位 + 实验
  [P2.4: BitMask 消融]       ──→ 碰撞统计
  [P3.2: 多场景数据]         ──→ 依赖 P3.1 (Panda)
  [P4.1: OMPL 基线修复]      ──→ 检查配置 + 重跑

Week 4:
  [P3.3: 窄通道场景]         ──→ 新场景定义
  [P4.2: IRIS-NP 基线]       ──→ pydrake 验证 + 运行
  [P4.3: GCS-SBF 混合]       ──→ 依赖 P4.2

Week 5:
  [P5.1: 定理证明]           ──→ 3 条定理
  [P5.2: Related Work]       ──→ 文献调研 + 扩写
  [P5.3: Discussion]         ──→ 基于 P1-P4 结果
  [P5.4-5.5: Conclusion+附录] ──→ 最终整合

Week 6:
  [最终整合]                  ──→ 重编译 PDF, 审稿模拟
```

---

## 文件清单 (需新建/修改)

### 新建文件

| 文件 | 用途 |
|------|------|
| `cpp/v6/scripts/plot_common.py` | 统一绘图配置 (配色/字体/保存) |
| `cpp/v6/scripts/plot_fig2_boxes.py` | Fig 2: C-space 盒可视化 |
| `cpp/v6/scripts/plot_fig3_envelope.py` | Fig 3: 包络紧凑性柱状图 |
| `cpp/v6/scripts/plot_fig4_path.py` | Fig 4: 3D 路径可视化 |
| `cpp/v6/scripts/plot_fig5_scalability.py` | Fig 5: 可扩展性折线图 |
| `cpp/v6/scripts/plot_fig6_breakdown.py` | Fig 6: 构建时间分解图 |
| `cpp/v6/scripts/plot_fig7_ablation.py` | Fig 7: 消融条形图 |
| `cpp/v6/scripts/run_full_experiments.sh` | 自动化全量实验脚本 |
| `cpp/v6/scripts/collect_results.py` | 聚合多种子 JSON → 统计摘要 |
| `cpp/v6/src/configs/panda.json` | Panda 机器人 v6 格式配置 |

### 修改文件

| 文件 | 修改内容 |
|------|----------|
| `cpp/v6/experiments/exp5_ablation.cpp` | 增加 Z4/split/incrFK 消融分组 |
| `cpp/v6/experiments/exp6_build_timing.cpp` | 增加 per-phase timing, --scene 参数 |
| `cpp/v6/experiments/exp2_e2e_planning.cpp` | 增加 --dump-path, --scene 参数 |
| `cpp/v6/experiments/exp1_coverage.cpp` | 增加 2D 盒体 JSON 导出 |
| `cpp/v6/experiments/exp4_envelope_benchmark.cpp` | 增加 BitMask 碰撞统计 |
| `cpp/v6/experiments/marcucci_scenes.h` | 增加 narrow_passage/dense 场景 |
| `cpp/v6/doc/box_aabb_v6_paper_en.tex` | 7 张图 + 扩写所有章节 |
| `cpp/v6/doc/box_aabb_v6_paper_zh.tex` | 同步更新中文版 |
| `cpp/v6/CMakeLists.txt` | OMPL/Drake ON, 新 target |

---

## 验收标准

| 维度 | 指标 | 当前 | 目标 |
|------|------|------|------|
| 页数 (EN, 双栏) | pages | 7 | 16–20 |
| 图数 | figures | 0 | 7–10 |
| 表数 | tables | 7 | 8–10 |
| 机器人 | robots | 1 (IIWA14) | 2+ (IIWA14 + Panda) |
| 场景 | scenes | 1 (combined) | 4+ (shelves, bins, table, combined) |
| 种子数 | seeds | 2 | 10 |
| 基线方法 | baselines | 1 (OMPL partial) | 3+ (RRTConnect, RRT*, IRIS-NP+GCS) |
| 消融覆盖 | ablations | 3 (P0,P2,P4) | 7+ (+Z4, split, incrFK, BitMask) |
| 参考文献 | references | 23 | 40–50 |
| 定理 (含证明) | proofs | 0 (3 claims) | 3 full proofs |
| 编译通过 | LaTeX | ✅ | ✅ |
| 语法检查 | Grammarly/ltex | ❌ | ✅ |

---

## 风险与缓解

| 风险 | 影响 | 概率 | 缓解措施 |
|------|------|------|----------|
| pydrake IRIS-NP API 不兼容 | P4.2 无法完成 | 中 | 降级为 indirect comparison (引用原文数据) |
| Panda FK 配置转换错误 | P3.1 延迟 | 低 | 先验证 2-DOF, 逐步到 7-DOF |
| OMPL 0% success 配置问题 | P4.1 无法修复 | 中 | 联调碰撞检测函数; 切换到更简单场景验证 |
| 实验运行时间过长 | 整体延迟 | 低 | 利用 10 seeds 并行 (不同 terminal) |
| 论文过长 (>20 页) | 需要删减 | 中 | 将详细表格移至附录/补充材料 |
| 窄通道场景 SBF 失败 | 暴露局限 | 高 | 诚实报告, 讨论改进方向 (这反而是亮点) |

---

## 备注

- 所有实验均在 `(sbf)` conda 环境下运行
- C++ 编译: `cd cpp/v6/build && cmake .. -DCMAKE_BUILD_TYPE=Release && make -j$(nproc)`
- 重新启用 Drake/OMPL: `cmake .. -DSBF_WITH_DRAKE=ON -DSBF_WITH_OMPL=ON`
- matplotlib 版本: 3.10.8 (已安装)
- mosek: 已安装 (GCS 求解器)
- 论文编译: `cd cpp/v6/doc && xelatex box_aabb_v6_paper_en.tex && bibtex box_aabb_v6_paper_en && xelatex ×2`
