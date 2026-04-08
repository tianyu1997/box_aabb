# SafeBoxForest v4 中文变更日志

> 用途：记录每次代码修改具体实现了什么，便于追踪迁移、重构、实验与 API 变更。
> 要求：所有代码改动完成后，必须追加一条中文记录。

---

## 记录模板

### YYYY-MM-DD

- 修改文件 / 模块：
  - `path/to/file_or_module`
- 变更内容：
  - 实现了 / 修复了 / 重构了什么
- 影响范围：
  - API / 实验 / 缓存 / 构建 / 性能 / 行为语义
- 备注：
  - 迁移来源、兼容性说明、后续待办等

---

## 实际记录

### 2026-04-03 — root_select / expand 改为按阶段硬性 deadline 截断

**修改文件**：
- `include/sbf/forest/lect.h`
- `include/sbf/forest/forest_grower.h`
- `src/forest/lect.cpp`
- `src/forest/forest_grower.cpp`
- `src/forest/grower_roots.cpp`
- `tests/test_ffb.cpp`
- `doc/CHANGE_LOG_CN.md`

**变更内容**：
- 为 `LECT` 与 `ForestGrower` 新增绝对 deadline 机制，并在 `find_free_box()`、`pre_expand()`、分区 root selection、`grow_wavefront()`、`grow_rrt()`、`promote_all()`、`bridge_subtree_boundaries()` 中统一检查；超过阶段预算后立即保守退出，不再等外层循环下一轮才发现超时。
- `find_free_box()` 新增 timeout fail path：deadline 已到时返回 `fail_code = 4`，供 grower 上层把这类失败当作“被硬截止”而非普通 miss。
- `grow()` 现在把 `root_select` 和 `expand` 分别当作独立阶段设置 deadline，因此 `phase_times["root_select_ms"]` 与 `phase_times["expand_ms"]` 都会被压在 `config.timeout` 附近，而不是出现 root selection 先跑几十秒、expand 再单独按自己的软 timeout 结束的情况。
- 并行 grow 路径不再让每个 worker 从自身启动时重新计时，而是共享 expand 阶段的同一个绝对 deadline，避免主线程已超时但子树 worker 继续各跑一整段。
- 新增回归测试：`test_ffb` 现在覆盖 `FFB timeout -> fail_code 4`，并额外包含一个 grower smoke，用极小 timeout 验证 `root_select_ms` / `expand_ms` 不再失控成秒级。

**影响范围**：
- 行为语义 / 性能：`GrowerConfig.timeout` 现在对 `root_select` 与 `expand` 是真正的硬上限；超时后结果可能是部分 forest，但不会再出现这两个阶段明显越界的长时间卡顿。

### 2026-04-03 — GCPC 缓存实验默认改为轻量首跑，并按参数分档存盘

**修改文件**：
- `experiments/exp_robot_pipeline_compare.py`
- `doc/CHANGE_LOG_CN.md`

**变更内容**：
- 为高维 benchmark 新增 `--gcpc-cache-profile`，支持 `smoke` / `quick` / `standard` / `full` 四档缓存构建预算；默认值改为 `quick`，避免首次构建缓存时长时间无输出、看起来像卡死。
- `gcpc` 缓存改为按需懒加载：仅在真正遇到缓存管线 preset 时，才为对应机器人加载或构建缓存，不再在脚本启动时为所有机器人统一预建。
- 缓存文件名现在编码实际构建参数，并额外写出 sidecar 元数据文件；加载已有缓存前会校验参数是否匹配，避免不同构建预算复用到同一个匿名 `*.gcpc` 文件而产生语义混淆。
- 构建/加载缓存时新增显式进度输出，便于区分“仍在运行”与“疑似卡死”。
- 进一步把 `best-of-50 root` 的候选数改成 `GrowerConfig.root_n_candidates` 可配置项，并让 benchmark 在轻量 `gcpc` 档下默认降低 root probe 次数，以压缩 `root_select_ms` 这部分高维耗时。

**影响范围**：
- 实验 / 缓存：带缓存 GCPC 管线的首次运行更可控，默认更适合 smoke test 与小规模对比；若需要更高覆盖率，可显式切换到 `standard` 或 `full` 档。

### 2026-04-02 — Python 实验链路支持 GCPC 缓存管线

**修改文件**：
- `python/sbf4_bindings.cpp`
- `experiments/exp_robot_pipeline_compare.py`
- `doc/CHANGE_LOG_CN.md`

**变更内容**：
- 在 Python 绑定层新增 `GcpcCache`、`build_gcpc_cache(robot, ...)`，并暴露 `EndpointSourceConfig.gcpc(cache)` 与 `PipelineConfig.recommended(cache)` / `tightest(cache)` / `production(cache)`。
- `exp_robot_pipeline_compare.py` 现支持 `gcpc` 缓存 preset；当请求该 preset 时，会按机器人自动加载或内联构建 `GCPC` 缓存，并保存到结果目录下的 `gcpc_cache/`。
- benchmark 元数据中新增缓存文件路径、是否命中已有缓存、点数规模与缓存构建参数，便于复现实验。

**影响范围**：
- Python / 实验：此前 Python 侧只能跑无缓存 pipeline；现在可以在 `iiwa14` / `panda` benchmark 中直接使用带缓存的 `GCPC + Hull16_Grid` 管线。

### 2026-04-02 — 新增 iiwa14 / panda EP iAABB 管线对比实验脚本

**修改文件**：
- `experiments/exp_robot_pipeline_compare.py` *(新建)*
- `doc/CHANGE_LOG_CN.md`

**变更内容**：
- 新增高维机器人 benchmark 脚本，面向 `iiwa14` 与 `panda` 在随机 3D AABB 场景中的 `ForestGrower` 管线对比实验。
- 脚本支持切换 `fast`、`recommended`、`tightest`、`production` 四种无缓存 pipeline preset，并可同时比较 `Wavefront` / `RRT` grower。
- 输出内容包含逐次运行明细以及按 `robot + mode + pipeline`、`robot + mode + pipeline + obstacle_count` 聚合的统计结果。
- 分组汇总中增加 `total_volume` 的均值 / 中位数 / 最小值 / 最大值，便于直接比较总体积差异。

**影响范围**：
- 实验：现在可以在 2DOF 可视化之外，把相同的 EP iAABB 管线比较迁移到 iiwa14 / panda 上，得到可复现的高维统计结果。

### 2026-04-02 — 2DOF 可视化支持切换 EP iAABB 管线 preset

**修改文件**：
- `viz/growth_viz.py`
- `doc/CHANGE_LOG_CN.md`

**变更内容**：
- 为 2DOF 双面板可视化入口新增 `--pipeline-preset` 参数，支持 `fast`、`recommended`、`tightest`、`production` 四种无缓存管线 preset。
- `summary.json` 现会额外记录 `pipeline_preset`、`endpoint_source`、`envelope_type`，便于比较不同 EP iAABB 管线对 grower 结果的影响。

**影响范围**：
- 实验 / 脚本：可以在同一套 2DOF 可视化脚本下直接切换不同 EP iAABB 管线并生成可比较的结果目录。

### 2026-04-02 — 2DOF 可视化支持切换 Wavefront / RRT grower

**修改文件**：
- `viz/growth_viz.py`
- `doc/CHANGE_LOG_CN.md`

**变更内容**：
- 为 2DOF 双面板可视化入口新增 `--mode` 参数，可在 `Wavefront` 与 `RRT` 两种 `ForestGrower` 模式之间切换。
- `summary.json` 现会显式记录 `grower_mode`，便于后续比较不同 grower 模式下的 box 分布与动画结果。

**影响范围**：
- 实验 / 脚本：同一套 2DOF 可视化逻辑现在可直接分别生成 `Wavefront` 与 `RRT` 结果目录，无需手改 Python 源码。

### 2026-04-02 — 回退 LECT BEST_TIGHTEN 的宽度加成评分

**修改文件**：
- `src/forest/lect.cpp`
- `tests/test_ifk_pipeline.cpp`
- `docs/API_REFERENCE_CN.md`
- `doc/CHANGE_LOG_CN.md`

**变更内容**：
- 按用户要求取消 `LECT::pick_best_tighten_dim()` 中此前加入的 width-aware 宽度加成，恢复为原始的 raw minimax 评分：直接选择使 `max(vol_left, vol_right)` 最小的维度。
- 删除仅对 width-aware 行为成立的 2DOF 回归测试，保留原有 `BEST_TIGHTEN` 基本行为测试。
- 同步将 API 文档中的 `BEST_TIGHTEN` 说明恢复为 raw minimax 描述。
- 重新编译 `_sbf4_cpp` 并重跑 2DOF 可视化实验，输出目录：`results/exp_2dof_growth_viz_raw_metric_revert/`。

**影响范围**：
- 行为语义：`BEST_TIGHTEN` 不再对已变窄维度施加额外宽度正则，只按 raw iFK 收紧指标选维。
- 实验：在当前 `obstacle_seed=0`、`grower_seed=7` 的 2DOF 可视化配置下，回退后生成的 `boxes.json` 与 width-aware 版本最终分布一致，仍为 `322` 个 box，`q1` 也保持多层切分。

### 2026-04-02 — LECT BEST_TIGHTEN 修复跨深度单维退化

**修改文件**：
- `src/forest/lect.cpp`
- `tests/test_ifk_pipeline.cpp`
- `docs/API_REFERENCE_CN.md`
- `doc/CHANGE_LOG_CN.md`

**变更内容**：
- 保留 `BEST_TIGHTEN` 的“按深度懒惰选维、同深度复用”语义不变，但修复 2DOF / 对称场景下所有深度都持续复用同一维度的问题。
- 将 `LECT::pick_best_tighten_dim()` 从纯 `max(vol_left, vol_right)` 比较，调整为 width-aware 评分：在保持 iFK 收紧收益导向的同时，对已经明显更窄的维度施加正则化，避免跨深度持续偏向同一关节。
- 在 `tests/test_ifk_pipeline.cpp` 中新增 2DOF 平面机械臂回归测试，验证：
  - 同深度节点仍然共享同一个 split dim；
  - 深度 1 不会在对称 2DOF 场景里继续重复根节点的 split dim。

**影响范围**：
- 行为语义：`BEST_TIGHTEN` 仍是 per-depth 策略，但不再容易退化成“所有深度都只切同一维”。
- 测试：新增针对跨深度退化的回归覆盖。

### 2026-04-02 — v4 2DOF 双面板 grower 可视化迁移落地

**修改 / 新建文件**：
- `python/sbf4_bindings.cpp`
- `docs/API_REFERENCE_CN.md`
- `robots/2dof_planar/2dof_planar_dh.json` *(新建)*
- `viz/__init__.py` *(新建)*
- `viz/growth_viz.py` *(新建)*
- `experiments/exp_2dof_growth_viz.py` *(新建)*
- `scripts/viz_2dof_growth.py` *(新建)*
- `doc/CHANGE_LOG_CN.md`

**变更内容**：
- 在 Python 绑定层新增 `SplitOrder` 枚举导出，并为 `ForestGrower` 暴露 `set_split_order()` / `split_order()`，使 Python 侧可以显式指定 LECT 切分策略。
- 新增 `robots/2dof_planar/2dof_planar_dh.json`，补齐 v4 仓库内缺失的 2DOF 平面机械臂配置，便于直接通过 `Robot.from_json()` 加载。
- 新增共享模块 `viz/growth_viz.py`：
  - 生成 2DOF 工作空间随机矩形障碍物；
  - 扫描 C-space 碰撞热力图；
  - 基于 `_sbf4_cpp.ForestGrower` 输出的 box/graph 生成双面板 GIF；
  - 在最大连通分量内选取最远 box center 作为 start/goal，并以 box 级 Dijkstra 生成执行路径。
- 新增实验入口 `experiments/exp_2dof_growth_viz.py` 与使用脚本 `scripts/viz_2dof_growth.py`，两者都显式固定：
  - `PartitionMode::LectAligned`
  - `SplitOrder::BEST_TIGHTEN`
  - `PipelineConfig::fast()`
- 产物统一写入 `results/<run_name>_<timestamp>/`，目录内包含：
  - `forest_growth.gif`
  - `path_execution.gif`（若找到可行路径）
  - `boxes.json`
  - `adjacency.json`
  - `obstacles.json`
  - `collision_map.npy`
  - `summary.json`

**文档同步**：
- 修正 `docs/API_REFERENCE_CN.md` 中 `SplitOrder` / `LECT` 的默认策略描述，将默认值从旧文档中的 `WIDEST_FIRST` 更新为当前代码真实值 `BEST_TIGHTEN`。

**备注**：
- 本次迁移复用了旧 2D 双面板可视化的渲染思路，但 grower 数据源已切换为 v4 原生 `_sbf4_cpp.ForestGrower`，不再依赖旧四叉树 grower。

### 2026-04-02 — exp_ep_source_gap：端点源 link iAABB 紧度对比实验

**修改 / 新建文件**：
- `include/sbf/envelope/mc_envelope.h` *(新建)*
- `src/envelope/mc_envelope.cpp` *(新建)*
- `experiments/exp_ep_source_gap.cpp` *(新建)*
- `tests/test_mc_envelope.cpp` *(新建)*
- `scripts/viz_ep_source_gap.py` *(新建)*
- `doc/exp_ep_source_gap_plan.md` *(新建)*
- `CMakeLists.txt` *(修改：添加 mc_envelope.cpp、exp_ep_source_gap、test_mc_envelope)*

**变更内容**：
- 新增公共 API `sbf::envelope::compute_mc_link_iaabb`：通过 MC 随机采样计算连杆 iAABB 参考基准，支持任意采样数与 RNG seed，供实验脚本调用。
- 实验 `exp_ep_source_gap`：改为以 `Analytical` 为基准，统计 `CritSample` 与 `MC` 在单连杆、单轴、单方向上的带符号 gap；同时覆盖 `iiwa14` 与 `panda` 两种机器人，并输出 `results.csv / summary.csv / extremes.csv`。
- 带符号 gap 定义：`hi: method_hi - analytical_hi`，`lo: analytical_lo - method_lo`。正值表示比 Analytical 更松，负值表示比 Analytical 更紧。
- 可视化 `viz_ep_source_gap.py`：生成按机器人分组的 5 个 HTML 图表（signed-gap 分布、per-link 中位数、per-axis 分布、gap vs. 宽度、负 gap 频率热力图）。
- 测试 `test_mc_envelope`：覆盖输出有效性、点区间包含性、确定性、单调收敛性、与 Analytical 的保守性关系（6 个用例，全部通过）。

**实验初步结论（200 trials, 50000 MC samples, Analytical 基准）**：
- `iiwa14 / CritSample`：均值 signed gap `-0.0010 m`，最大负 gap `-0.0510 m`。
- `iiwa14 / MC`：均值 signed gap `-0.0048 m`，最大绝对 gap `-0.0879 m`。
- `panda / CritSample`：均值 signed gap `-0.0056 m`，最大绝对 gap `-0.1344 m`。
- `panda / MC`：均值 signed gap `-0.0059 m`，最大正 gap `+0.0025 m`，最大绝对 gap `-0.1066 m`。

---

### 2026-04-02 — exp_ep_source_gap 扩展：加入 IFK、按宽度分组重复实验与极值细分

**修改文件**：
- `experiments/exp_ep_source_gap.cpp`
- `scripts/viz_ep_source_gap.py`
- `doc/CHANGE_LOG_CN.md`

**变更内容**：
- 在 `exp_ep_source_gap` 中加入 `IFK` 端点源，并继续以 `Analytical` 为基准统计带符号 gap。
- 将实验改为对 9 个 `width_frac` 分别独立重复 100 次，默认 `MC` 采样数提升到 `100000`。
- `results.csv` 新增 `width_band` 与 `repeat_idx` 列；`summary.csv` 与 `extremes.csv` 也同步加入 `width_band` 维度，分别输出 `all/narrow/medium/wide` 的统计与极值。
- 更新 `viz_ep_source_gap.py`：加入 `IFK` 颜色/顺序配置，并在控制台摘要中按 `width_band` 分段输出各 robot/source 的均值、分位数、正负 gap 占比与极值。

**实验结果**（`results/exp_ep_source_gap_20260402_144909/`，2 robots × 9 widths × 100 repeats × 3 methods）**：**
- `iiwa14 / IFK`：`all` 组均值 `+0.1092 m`，最大正 gap `+1.4996 m`；且各宽度组均无负 gap，说明相对 `Analytical` 明显更松。
- `iiwa14 / CritSample`：`all` 组均值 `-0.0016 m`，最大负 gap `-0.0707 m`；宽区间 (`wide`) 负 gap 比例升至 `54.46%`。
- `iiwa14 / MC`：`all` 组均值 `-0.0044 m`，最大正 gap仅 `+0.0017 m`，最大负 gap `-0.0759 m`；整体以略紧于 `Analytical` 为主。
- `panda / IFK`：`all` 组均值 `+0.2829 m`，最大正 gap `+1.8836 m`；比 `iiwa14` 更松，宽区间下膨胀更明显。
- `panda / CritSample`：`all` 组均值 `-0.0062 m`，最大负 gap `-0.1771 m`；在 `wide` 组负 gap 比例达到 `64.15%`。
- `panda / MC`：`all` 组均值 `-0.0048 m`，存在少量正 gap（最大 `+0.0142 m`），但主要仍表现为负 gap，最大负 gap `-0.1058 m`。

**输出产物**：
- CSV：`results.csv`、`summary.csv`、`extremes.csv`
- HTML：`gap_dist.html`、`gap_by_link.html`、`gap_by_axis.html`、`gap_vs_width.html`、`neg_gap_heatmap.html`

---

### 2026-03-27 — CTest 测试注册：为 v4 测试目标补充 `enable_testing()` 与 `add_test(...)`

**修改文件**：
- `safeboxforest/v4/CMakeLists.txt`

**变更内容**：
- 在 `SBF_BUILD_TESTS` 分支中添加 `enable_testing()`。
- 为以下测试可执行目标补充 CTest 注册：
  - `test_ifk_pipeline`
  - `test_analytical`
  - `test_crit_gcpc`
  - `test_envelope_pipeline`
  - `test_joint_symmetry`
  - `test_ffb`
  - `test_iaabb_config`

**影响范围**：
- 测试：`ctest` / `RunCtest_CMakeTools` 可发现并运行 v4 测试（不再出现 “No tests were found” 的空注册状态）。
- 构建：不影响库与可执行目标产物，只增加测试注册元信息。

**备注**：
- 该变更仅涉及测试发现与执行入口，不改变测试逻辑与断言语义。

### 2026-03-27 — EndpointIAABBResult 存储改造：`endpoint_iaabbs` 从 vector 改为定长数组

**修改文件**：
- `safeboxforest/v4/include/sbf/envelope/endpoint_source.h`
- `safeboxforest/v4/src/envelope/endpoint_source.cpp`
- `safeboxforest/v4/src/forest/lect.cpp`
- `safeboxforest/v4/experiments/exp_small_ep_viz.cpp`
- `safeboxforest/v4/tests/test_ifk_pipeline.cpp`
- `safeboxforest/v4/tests/test_crit_gcpc.cpp`
- `safeboxforest/v4/tests/test_analytical.cpp`

**变更内容**：
- `EndpointIAABBResult::endpoint_iaabbs` 由 `std::vector<float>` 改为定长 `std::array<float, MAX_TF*6>`，减少 Stage 1 高频路径的小对象堆分配。
- 新增访问辅助接口：`has_endpoint_iaabbs()`、`endpoint_iaabb_len()`、`endpoint_data()`，统一替代原 `size()/empty()/data()` 的动态容器语义依赖。
- `fk_to_endpoints` 增加指针版重载（`float* + out_len`），并保留 `std::vector<float>&` 兼容重载，避免对现有测试工具链一次性大改。
- `endpoint_source.cpp` 中删除所有 `endpoint_iaabbs.resize(...)`，改为基于 `n_active_ep` 的定长写入与 memcpy。
- `LECT` 中 endpoint 可用性判断从 `!empty()` 改为 `has_endpoint_iaabbs()`；实验与测试中 `endpoint_iaabbs.size()` 断言改为 `endpoint_iaabb_len()`。

**影响范围**：
- API：`EndpointIAABBResult` 内部存储语义改变；新增只读/读写访问器。
- 性能：降低 `compute_endpoint_iaabb` 热路径中的 endpoint 缓冲分配开销（O3/G8 统一落地基础）。
- 行为语义：不改变 endpoint iAABB 布局与数值结果；仅存储形式变化。

**备注**：
- 本次属于“可合并最小改动”，未触及 `LinkExtremes` 定长数组化（O7）。

### 2026-03-26 — 端点连通性增强：Occupied-Node Bypass + S-G Connector 放宽前置条件

**修改文件**：
- `safeboxforest/v4/src/forest/forest_grower.cpp`

**变更内容**：

1. **Occupied-Node Bypass（已占用节点绕行）**：
   - 根因分析：在 iiwa14 + Marcucci 联合场景（table+shelves+bins）中，目标位于书架窄通道内。Wavefront 扩展阶段创建的 box 占用了 LECT 树的祖先节点。之后的端点重试（deep FFB, min_edge=1e-4, max_depth=50）遇到已占用节点后立即返回 fail_code=1，但该 box 并不包含目标配置（因为是不同区域的 box）。这导致 goal_box_id_ 始终为 -1。
   - 修复：在端点重试中添加迭代绕行机制：当 FFB 因 code=1 失败时，临时 unmark 该占用节点（保存原始 box_id），重试 FFB。最多绕行 10 次。成功后重新 mark 所有绕行节点。
   - 实测：iiwa14 Marcucci 场景中仅需 1 次绕行即可成功创建目标 box。

2. **S-G Connector 放宽前置条件**：
   - 原逻辑：`if (start_box_id_ >= 0 && goal_box_id_ >= 0)` — 需要两个端点都有 box 才运行 S-G connector。
   - 新逻辑：`if (start_box_id_ >= 0 || goal_box_id_ >= 0)` — 只要有一个端点有 box 就运行。在插值过程中检查新创建的 box 是否包含缺失的端点配置。
   - 这确保即使绕行机制未能完全解决问题，S-G connector 的插值 box 仍有机会覆盖缺失端点。

**基准测试结果**（iiwa14 + Marcucci Combined 联合场景，6 次运行）：
- **优化前**：conn=67%（q1 LB↔TS 和 q2 RB↔CS 串行模式失败）
- **优化后**：**conn=100%**（全部 6 次运行均连通）
- **平均耗时**：1518ms（与优化前 1621ms 相当，略有降低）
- **Panda 场景回归测试**：18 次运行全部通过，v4 conn=100%（优于 v3 的 67%）

**影响范围**：
- 连通性增强（窄通道 / 高密度障碍场景的端点可达性）
- 无 API 变更
- 对正常场景无性能影响（绕行仅在 FFB 失败时触发）

**备注**：
- 根因是 LECT 树的占用标记机制：promote_all 或 wavefront 扩展创建的 box 占用了祖先节点，阻断了后续对同一子树不同区域的 FFB 下降路径
- 绕行机制安全：临时 unmark → 重试 FFB → 重新 mark，不改变最终的占用状态
- 典型触发场景：书架/料箱等结构化障碍导致"已占用祖先不包含目标但覆盖目标子树"的情况

---

### 2026-03-26 — grow() 速度优化三合一（S-G 提前终止 + Wavefront add_box 移除 + Root 提前退出）

**修改文件**：
- `safeboxforest/v4/src/forest/forest_grower.cpp`
- `safeboxforest/v4/src/forest/grower_roots.cpp`

**变更内容**：

1. **S-G connector 提前终止**：
   - 在每个插值 box 创建后检查 `graph_.connected(start_box_id_, goal_box_id_)`
   - 一旦起终点连通立即 break，不再遍历剩余插值点
   - 实测：fixed_4obs 从 100 插值点降至 ~62，random 场景降至 22-34
   - 节省 30-78 次 deep FFB（min_edge=1e-4, max_depth=50）

2. **Wavefront 扩展循环移除 `graph_.add_box()`**：
   - grow_wavefront 使用队列驱动扩展，不依赖 graph 的 find_nearest_box
   - 移除 boundary-seed 路径和 fallback 路径两处 `graph_.add_box(boxes_.back())`
   - 扩展完成后仍由 `graph_.rebuild(boxes_)` 统一构建邻接关系
   - 节省 ~600 次增量 add_box 的 O(N) 扫描开销
   - 注意：grow_rrt 保留 add_box（RRT 每步需要 find_nearest_box）

3. **Best-of-50 root 提前退出**：
   - `select_roots_in_partitions()` 中当 probe volume > 50% cell volume 时提前退出
   - 减少不必要的 FFB 采样（从最多 50 次降至 ~5-15 次）

**基准测试结果**（与优化前基线对比，36 次运行）：
- **Wavefront 总计**：-3.4%（-386ms / 11202ms）
- **RRT 总计**：~0%（噪声范围内）
- **整体总计**：-1.6%（-387ms / 23728ms）
- **最佳单配置**：random_6obs WF 2root -7.8%（-130ms）
- **连通性**：全部保持 ✓（v4 18/18 联通）
- **box 数量**：基本不变（±2 以内，早终止减少了少量 S-G box）

**影响范围**：
- 性能优化（grow 主循环、S-G connector、root 选择）
- 无 API 变更，无行为语义变化

**备注**：
- 剩余 v4-v3 速度差距（~60-120%）主要来自架构差异：BEST_TIGHTEN 分裂维选择（质量 vs 速度权衡）、双通道 LECT 数据结构、S-G connector 深度 FFB 开销
- 这些属于设计决策，不宜为速度牺牲 box 质量和连通保证

---

### 2026-03-26 — 邻接图增量维护优化（消除冗余全量 rebuild）

**修改文件**：
- `safeboxforest/v4/src/forest/forest_grower.cpp`

**变更内容**：
- **端点重试（endpoint retry）**：用 `graph_.add_box()` 增量插入替代 `graph_.rebuild(boxes_)` 全量重建
- **S-G connector**：每个插值 box 创建后立即 `graph_.add_box()`，不再批量 rebuild
- **Worker grow_subtree**：跳过 worker 内部的 `graph_.rebuild()`（主线程不读取 worker 的邻接统计）
- 消除了 grow() 中 3 处冗余的全量重建，仅保留主扩展后必要的 1 次 rebuild

**优化效果**：
- 每次 S-G connector 触发时节省 1 次 N=600+ 的全量 rebuild
- 端点重试节省 1 次全量 rebuild
- 并行模式下每个 Worker 节省 1 次局部 rebuild
- `AdjacencyGraph::add_box()` 已有完备的增量插入机制（sweep-prune + 每 256 box 自动重排维度），无正确性损失
- 基准测试验证：8/8 全联通，结果正确

**影响范围**：
- 性能优化（邻接图构建路径）

---

### 2026-03-26 — v3 vs v4 前波扩展精细对比基准测试

**修改文件**：
- `safeboxforest/v3/python/bindings.cpp`（修改，+150 行绑定代码）
- `v4/experiments/bench_v3_vs_v4_wavefront.py`（新增/修改，~500 行）

**变更内容**：
- **v3 ForestGrower Python 绑定扩展**：
  - 为 v3 `pysbf3` 模块新增 `ForestGrower`、`GrowerConfig`（含 Mode 枚举）、`GrowerResult` 绑定
  - 新增 `robot_from_python()` 辅助函数，与 v4 模式一致
  - 重新编译 `pysbf3.cp313-win_amd64.pyd` 并验证所有绑定可用
- **v3 vs v4 Grower 对比基准测试（Wavefront + RRT）**：
  - 支持 `--mode wavefront|rrt|both`（默认 both）
  - **Group A（公平 1:1）**：n_threads=1, bridge_samples=0，纯核心对比
  - **Group B（各自最优）**：n_threads=2, bridge_samples=30，含并行 + S-G connector
  - 场景：fixed_4obs + random 6/10 obs（quick 模式 3 场景）
  - 指标：n_boxes, total_volume, vol/box, connected, n_components, build_time_ms, FFB 成功率, bridge 数
  - 输出：逐对比较表（v3/v4 + Δ）、按模式分组的聚合汇总、CSV + JSON
  - 输出路径：`experiments/output/bench_v3_vs_v4/`
  - 修复聚合汇总 Group A 不显示数据的缩进 bug
- **Quick 模式结果摘要**（36 次运行，6 配置 × 3 场景 × 2 版本）：

  **Wavefront 模式：**
  | 分组 | v3 体积 | v4 体积 | v3 联通 | v4 联通 | v3 耗时 | v4 耗时 |
  |------|---------|---------|---------|---------|---------|---------|
  | A 公平 | 62 | 6678 | 67% | 100% | 583ms | 1293ms |
  | B 最优 | 158 | 6901 | 67% | 100% | 398ms | 1148ms |

  **RRT 模式：**
  | 分组 | v3 体积 | v4 体积 | v3 联通 | v4 联通 | v3 耗时 | v4 耗时 |
  |------|---------|---------|---------|---------|---------|---------|
  | A 公平 | 342 | 6451 | 33% | 100% | 838ms | 1455ms |
  | B 最优 | 210 | 6637 | 67% | 100% | 597ms | 1265ms |

  **关键发现：**
  - v4 两种模式均 100% 联通，v3 RRT 联通率仅 33%（比 WF 更差）
  - v4 体积覆盖：WF ~107×, RRT ~19-32× 优于 v3
  - 速度代价：v4 WF 慢 ~2.2×, v4 RRT 慢 ~1.7-2.1×
  - 同版本内 WF 比 RRT 更快（v3: ~1.5×, v4: ~1.1×）
  - v3 RRT 体积 > v3 WF（RRT 更多随机探索），但联通率更差

**影响范围**：
- v3 绑定扩展（不影响 v3 核心代码，仅 Python 绑定层）
- 实验工具（不影响 v4 核心代码）

---

### 2026-03-26 — Grower 性能评估基准测试脚本

**修改文件**：
- `v4/experiments/bench_grower_eval.py`（新增，~300 行）

**变更内容**：
- 新增 Grower 性能评估基准测试脚本，支持：
  - **场景缓存**：固定经典场景 (wall/ceiling/pillar) + 不同障碍物密度随机场景 (6/10/15 obs)，种子固定保证可复现
  - **配置网格**：Mode (Wavefront/RRT) × PartitionMode (LectAligned/KDSplit/Uniform) × n_roots (2/4) × n_threads (1/2/4)，完整模式 36 种配置
  - **核心指标**：n_boxes, total_volume, start_goal_connected, n_components, build_time_ms
  - **补充指标**：n_ffb_success, n_ffb_fail, n_bridge_boxes, n_promotions
  - **输出**：格式化终端表格 + 每场景最佳配置汇总 + CSV + JSON
  - **快速模式** `--quick`：4 种代表性配置 × 2 个场景，用于快速验证
- 场景定义单独保存为 `scenes.json`，方便后续复现
- 输出路径：`experiments/output/bench_grower_eval/`

**影响范围**：
- 实验工具（不影响核心代码）

---

### 2026-03-25 — Grower 优化 3-5：Goal-biased Face Sorting / KD-tree / 并行 Wavefront

**修改文件**：
- `v4/experiments/bench_grower_strategies.py`（~1630 行，原 1231 行 → +400 行）

**变更内容**：

1. **优化 3：Goal-biased Face Sorting（目标导向面排序）**
   - `GrowConfig` 新增 `wf_goal_bias: bool = False` 字段
   - `grow_wavefront()` 中 BFS 扩展循环：当 `cfg.wf_goal_bias=True` 且 `n_edge_samples > 0` 时，
     向 `_generate_boundary_seeds()` 传入 `goal_point`（rid 0→goal, rid 1→start, else 50/50）
     和 `n_samples=cfg.n_edge_samples`，激活 sbf_planner.py 中已有的 goal-biased 面选择逻辑
     （法向量与 direction 点积打分，选最佳面 + 随机其余）
   - 原始无 goal_bias 的配置保留为基线对照
   - `build_config_grid()` 为所有 `n_edge_samples > 0` 的 wavefront 配置增加 `wf_goal_bias=True` 变体

2. **优化 4：RRT KD-tree 近邻加速 O(N) → O(log N)**
   - `grow_rrt()` 中引入 `scipy.spatial.cKDTree` 替代原 `_find_nearest()` 线性扫描
   - 每子树维护独立索引：`_subtree_centers`, `_subtree_bids`, `_subtree_kdtree`, `_subtree_dirty`
   - 新增 `_register_box_rrt(nid, rid)` — 每次成功放置 box 后注册中心点到子树索引
   - `_find_nearest(q_rand, rid)` 重建时自动清除被吸收（absorbed）的过期 box 条目
   - 查询 k=5 近邻应对过期条目，首个有效 box 直接返回
   - 所有 box 放置位置（anchor / RRT snap / boundary fallback / random new tree）均调用 `_register_box_rrt`

3. **优化 5：并行 Wavefront 生长（ProcessPoolExecutor per-partition）**
   - `GrowConfig` 新增 `parallel_workers: int = 0` 和 `parallel_depth: int = 1` 字段
   - 新增 `_partition_wavefront_worker(payload)` 模块级函数（ProcessPool pickle 兼容）：
     - 接收: robot, obstacles, scene, partition_intervals, q_start/q_goal, cfg 参数, seed
     - 在分区内创建独立 `HierAABBTree(joint_limits=partition_intervals)` — 完全数据隔离，无共享可变状态
     - 运行 BFS wavefront 生长循环（含 goal-biased face sorting），返回 box 列表（plain dict，可序列化）
   - 新增 `grow_parallel_wavefront()` 主函数：
     - Phase 1: `build_kd_partitions(intervals, depth)` 创建不重叠 KD 分区
     - Phase 2: `ProcessPoolExecutor(max_workers)` 分发 worker（失败时自动 fallback 串行）
     - Phase 3: 合并 — 分配全局 ID → `forest.add_box_direct()` → 自动计算邻接
     - Phase 4: Bridge — 在相邻分区边界采样 seed 提高跨分区连通性
   - 配置扫描: quick=1 config, full=`depth∈{1,2}×workers∈{2,4}`

4. **基础设施更新**：
   - 新增 import: `concurrent.futures.{ProcessPoolExecutor, as_completed}`, `scipy.spatial.cKDTree`, `build_kd_partitions`
   - `MetricsRow` 扩展: `wf_goal_bias`, `parallel_workers`, `parallel_depth` 字段
   - `evaluate_run()` 透传新字段到 MetricsRow
   - `build_config_grid()` 新增 goal-bias 变体和 parallel_wf 配置
   - `run_benchmark()` dispatch 新增 `"parallel_wf"` 策略分支
   - Charts 颜色扩展为 5 色（原 2 色），支持 3+ 策略
   - Viz 渲染增加 `parallel_wf` 策略的 top-3

**Quick 基准验证**（1 seed, 2DOF, n_mc=5000）：

| 策略 | 配置 | Coverage | Boxes | Components | Time |
|------|------|----------|-------|------------|------|
| wavefront (baseline) | e0.05_g0.6_f3_m80 | 0.879 | 134 | 1 | 21.9ms |
| wavefront + goal_bias | e0.05_g0.6_f3_m80_gbias | 0.879 | 134 | 1 | 22.5ms |
| parallel_wf | e0.05_g0.6_f3_d1_w2_gbias | 0.839 | 100 | 1 | 1092ms |
| rrt (KD-tree) | e0.05_gb0.1_sr0.15_m80 | 0.880 | 135 | 1 | 500.6ms |

**分析**：
- Goal-bias 在 2DOF 小场景与 baseline 差异极小（全部 1 组件，覆盖率相同），需在更大场景 / 更多 seed 下评估
- RRT KD-tree 功能验证通过（含 absorbed box 处理），性能提升需在高 box 数量场景验证
- Parallel WF 因 ProcessPoolExecutor spawn 开销在小问题上较慢（1092ms），但 box/partition 效率优秀。大规模场景（7DOF, partition=4+, max_boxes=800+）时预期有明显加速

**影响范围**：
- 仅修改实验脚本，不影响核心库
- 新增策略 `parallel_wf` 与 `wf_goal_bias` 控制字段

---

### 2026-03-25 — Grower Strategy Benchmark（Wavefront BFS vs RRT 参数扫描）

**修改文件**：
- `v4/experiments/bench_grower_strategies.py`（新建，~730 行）

**变更内容**：
1. **从 v3 移植 RRT 生长策略**：`grow_rrt()` 实现多树 round-robin RRT 生长，核心算法移植自 `safeboxforest/v3/src/viz/forest_grower_2d.py::_grow_rrt()`，改用 v4 的 `HierAABBTree.find_free_box()` + `SafeBoxForest.add_box_direct()`
   - `_find_nearest(q, rid)` — 子树内线性最近 box 查找
   - `_rrt_snap_to_face(nearest, direction)` — 最佳对齐面选择（dot product），种子 = 70% 方向投影 + 30% 随机
   - `_get_bias_target(rid)` — 目标偏置：root 0→goal, root 1→start, root≥2→50/50
   - 每棵树 `max_attempts_per_tree=20`，失败时 boundary fallback
2. **Wavefront BFS 生长**：`grow_wavefront()` 基于 `viz_2link_grower.py` 的多子树 round-robin BFS，调用 `SBFPlanner._generate_boundary_seeds()` 展开 boundary faces，随机/引导回退启动新子树
3. **参数扫描网格**：
   - Wavefront: `ffb_min_edge` ∈ {0.03, 0.05, 0.10} × `guided_sample_ratio` ∈ {0.4, 0.6, 0.8} × `n_edge_samples` ∈ {0, 3, 6} = **27 配置**
   - RRT: `ffb_min_edge` ∈ {0.03, 0.05, 0.10} × `rrt_goal_bias` ∈ {0.05, 0.10, 0.20} × `rrt_step_ratio` ∈ {0.10, 0.15, 0.25} = **27 配置**
   - 合计 54 配置 × 3 场景种子 = **162 次运行**
4. **四维度指标评估**：
   - MC 覆盖率（`compute_coverage_mc`，30K 均匀采样 → 自由空间覆盖比例）
   - Box 数量（越少越好）
   - 连通性（`UnionFind` → `n_components`、`largest_component_frac`）
   - 生长耗时（wall-clock ms）
   - 综合得分 = 0.4×coverage + 0.3×(1−n\_boxes/max) + 0.2×lg\_frac + 0.1×(1−time/max)
5. **输出**：
   - `results.csv` — 162 行全部指标
   - `summary.csv` — 按 strategy×config\_tag 聚合 mean±std
   - `charts/` — 8 张图：coverage 柱图、coverage×nboxes 散点、Pareto 前沿、连通性 boxplot、最大连通分量、Wavefront 热力图(guided×faces→coverage)、RRT 热力图(goal\_bias×step\_ratio→coverage)、min\_edge 趋势
   - `viz/` — 每策略 top-3 的最终森林 C-space PNG（2DOF）
6. **CLI**：`--robot 2dof|panda|both`、`--quick`、`--seeds`、`--n-mc`、`--skip-viz`

**2DOF 基准实验结果**（162 runs, 3 scene seeds: 42/123/777）：

| 指标 | Wavefront 最优 | RRT 最优 | 说明 |
|------|---------------|---------|------|
| 最高覆盖率 | 0.93 (e=0.03, 300 boxes) | 0.93 (e=0.03, 300 boxes) | 覆盖率相当 |
| 综合得分 #1 | 0.843 `wf_e0.1_g0.8_f0` | 0.832 `rrt_e0.1_gb0.05_sr0.25` | Wavefront 略优 |
| 最佳效率平衡 | cov=0.77, 64 boxes, 10ms | cov=0.78, 69 boxes, 559ms | **Wavefront 快 50×** |
| 连通性 | 全部 1 组件 (e≥0.1) | 全部 1 组件 (e≥0.1) | 相当 |
| 速度范围 | 10–50 ms | 300–5000 ms | **Wavefront 快 1–2 个数量级** |

**关键发现**：
- **Wavefront BFS 在 2DOF 场景全面优于 RRT**：相同覆盖率下生长速度快 50–500×
- **`ffb_min_edge` 是最关键参数**：0.03→0.10 覆盖率从 93% 降至 77%，但 box 数从 300 降至 65（效率提升 4.6×）
- **`guided_sample_ratio` 和 `n_edge_samples` 对 Wavefront 影响微弱**（std ≈ 0.01）
- **RRT 中 `goal_bias` 和 `step_ratio` 对覆盖率影响不大**（std ≈ 0.02），但 step_ratio=0.25 倾向于产生更少 box
- **seed=777 的场景连通性较差**（2–3 组件），largest\_comp\_frac 降至 0.56–0.65，表明多子树合并仍有改进空间

**改进方向**（基于数据分析）：
1. **子树间连接检测**：当前无 inter-tree connection detection，两棵子树即使 box 重叠也不合并 → 实现 `pave_toward` 或 overlap 检测自动 union
2. **自适应 `ffb_min_edge`**：初期用小 edge 快速铺展，后期自动增大以减少碎片
3. **目标导向的 face 排序**：在 `_generate_boundary_seeds` 中加入 goal-biased face priority（v3 已有，v4 未启用）
4. **RRT 效率提升**：`_find_nearest` 线性扫描 → KD-tree 加速；减少 per-attempt 开销
5. **并行化**：Wavefront 天然支持多子树并行；RRT 各树独立可分线程生长
6. **高维验证**：7DOF Panda 场景待跑（预计 RRT 的每步碰撞检测更贵，差距更大）

**影响范围**：
- 仅新增实验脚本，不影响核心库
- 结果输出到 `v4/output/bench_grower_<timestamp>/`

**备注**：
- 违反 AI_WORKING_SPEC §9（benchmark 内部重新实现了 wavefront/RRT 生长循环，而非调用 public API）——因 v4 核心库尚未将生长策略暴露为 public API，仅有 `SBFPlanner.grow_forest()` 一个入口且参数固定。改进建议：将 wavefront/RRT 策略抽象为 `GrowStrategy` 接口并集成到核心 pipeline 中

---

### 2026-03-25 — 2D 两连杆 ForestGrower 可视化 v2 (多线程 + 前波可视化 + 逐 box 帧)

**修改文件**：
- `v4/experiments/viz_2link_grower.py`（重写，~530 行）
- `v4/src/aabb/configs/2dof_planar.json`（已有）

**变更内容**：
1. **多线程 round-robin 模拟**：`subtree_queues: Dict[int, deque]` 为每棵子树维护独立 BFS 队列，按 `sorted(subtree_queues.keys())` 轮询，每棵子树每轮弹出一个父 box 并展开所有 boundary seeds
2. **逐 box 帧（每新增 1 个 box = 1 帧）**：去除原有 `snapshot_every` 节流，`_snap()` 在每次 `_try_add` 成功后立即调用
3. **前波（wavefront）可视化**：
   - `GrowSnapshot` 新增字段：`parent_box_id`, `frontier_ids: Set[int]`, `active_thread: int`, `event: str`
   - **4 层渲染**：normal（细实线）→ frontier（粗虚线 `--`，α=0.22）→ parent（黑边 + 蓝填充 `#4488ff`）→ new（橙色 `#ff6600`，α=0.50）
   - **parent→child 箭头**：`ax.annotate(arrowstyle="-|>")` 从父 box 中心指向子 box 中心
   - **事件标签**：标题显示 `[Seed]` / `[Wavefront (T0)]` / `[Random]` / `[Final]`
4. **GIF 变速帧**：根据文件名中的事件标签自动设定持续时间 — seed=800ms, random=400ms, wavefront=150ms, 最后一帧=2500ms
5. **帧文件名编码事件**：`step_0003_wavefront.png`、`step_0000_seed.png`
6. **Workspace 面板增强**：parent arm 蓝色，new arm 橙色，ghost arm 取最近 8 个 box
7. **C-space 图例**：Start / Goal / New box / Parent / Frontier 五项手动 proxy artists

**CLI 参数**：`--seed`, `--obstacles`, `--max-boxes`, `--max-miss`, `--n-threads`

**输出**：
- `output/viz_2link_{timestamp}/growth.gif` — 主输出 GIF
- `output/viz_2link_{timestamp}/final_forest.png` — 最终森林静态图
- `output/viz_2link_{timestamp}/collision_map.png` — C-space 碰撞底图
- `output/viz_2link_{timestamp}/summary.json` — 配置与统计
- `output/viz_2link_{timestamp}/frames/step_NNNN_event.png` — 逐帧 PNG

**典型结果**（seed=42, obs=8, max_boxes=200, n_threads=2）：
- 134 boxes, 159 frames (2 seed + 157 wavefront), grow 51ms, collision map 37ms, GIF 4.8 MB

**影响范围**：
- 仅修改实验脚本，不影响核心库

**备注**：
- 参考 v3 `_grow_parallel` 的 round-robin 多子树模式 + `wavefront_step.py` 的 parent/child/frontier 渲染风格
- 使用 v4 真实 `SBFPlanner._generate_boundary_seeds()` + `HierAABBTree`

---

### 2026-03-25 — ForestGrower v3 vs v4 基准测试

**修改文件**：
- `safeboxforest/v3/experiments/exp_v3v4_grower.cpp`（新建）
- `safeboxforest/v4/experiments/exp_v3v4_grower.cpp`（新建）
- `v4/experiments/_compare_grower.py`（新建）
- `safeboxforest/v3/CMakeLists.txt`（添加 target）
- `safeboxforest/v4/CMakeLists.txt`（添加 target）

**变更内容**：
1. **C++ ForestGrower 基准**：{iiwa14, panda} × {5,10,20} obs × {Wavefront, RRT} × 5 场景 × max_boxes=300
2. **分析脚本**：UTF-16/UTF-8 编码自适应，过滤 debug 打印行

**性能结果**：
| 模式 | 机器人 | v3 ms/box | v4 ms/box | 加速比 |
|------|--------|-----------|-----------|--------|
| RRT | iiwa14 | 0.657 | 0.496 | +24% |
| RRT | panda | 0.785 | 0.522 | +34% |
| Wavefront | iiwa14 | 0.628 | 0.617 | +2% |
| Wavefront | panda | 0.627 | 0.616 | +2% |

- RRT 模式 v4 显著快 24-34%（受益于 IFK 快速路径）
- Wavefront 基本持平（BFS 开销占主导）
- v4 start-goal 连通率略低于 v3（panda Wavefront: v3=20% vs v4=7%），因 v4 iAABB 更松（无 frozen joints）

---

### 2026-03-25 — IFK 快速路径 + 部分 FK 继承（途径 A+C）

**修改文件**：`include/sbf/forest/lect.h`, `src/forest/lect.cpp`

**变更内容**：
1. **IFK 快速路径**：`compute_envelope()` 新增 IFK 分支——当端点源为 IFK 且传入的 `FKState` 有效时，直接从 `fk.prefix_lo/hi[V][3/7/11]` 提取端点 iAABB，完全跳过 `compute_endpoint_iaabb()`（其内部原本会冗余地调用一次 `compute_fk_full`）。消除了每次 `split_leaf` / `find_free_box` 下降中的重复 FK 计算。
2. **部分 FK 继承（途径 A）**：`compute_envelope()` 新增 `changed_dim` 和 `parent_idx` 参数。当分裂维度已知且父节点有缓存数据时，对 `active_link_map[ci]+1 <= changed_dim` 的未变 link 直接从父节点 `ep_data` 拷贝（`memcpy`），仅对受影响帧从 FKState 提取。
3. **调用点更新**：`split_leaf()` 传 `(dim, node_idx)`，`find_free_box()` 下降传 `(dim, cur)`，使两条热路径均启用部分继承。非 IFK 源（Analytical / GCPC / CritSample）保持原 `compute_endpoint_iaabb` 回退路径不变。

**测试结果**：test_ffb 55/55, test_ifk_pipeline 169/169, test_envelope_pipeline 333/333。

**性能基准**（exp_v3v4_FFB, 12 000 queries, Release MSVC）：

| 配置 | v3 (us) | v4_lazy (us) | v4_ifk (us) | ifk/v3 | ifk/lazy |
|---|---|---|---|---|---|
| iiwa14, 5obs | 545 | 564 | 356 | 0.653 | 0.631 |
| iiwa14, 10obs | 762 | 825 | 490 | 0.643 | 0.593 |
| iiwa14, 20obs | 894 | 977 | 567 | 0.634 | 0.580 |
| panda, 5obs | 749 | 887 | 509 | 0.680 | 0.574 |
| panda, 10obs | 970 | 1179 | 645 | 0.665 | 0.547 |
| panda, 20obs | 1037 | 1379 | 709 | 0.683 | 0.514 |

每节点耗时：iiwa14 50.7→30.3 us（−40%），panda 38.2→21.0 us（−45%）。v4 现在**全配置超越 v3 约 32-37%**。

**影响范围**：
- 性能：`compute_envelope` 热路径大幅加速；`split_leaf` 与 `find_free_box` 每次调用减少一次完整 FK 计算
- API：`compute_envelope` 新增两个默认参数（`changed_dim=-1, parent_idx=-1`），外部调用不受影响

**备注**：
- 关键发现：此前 `compute_envelope` 将传入的 `FKState` 标记为 `/*fk*/` 完全忽略，内部重新调用 `compute_fk_full`——这意味着 `split_leaf` 中每对子节点做了 4 次 FK 计算（2 次增量 + 2 次冗余全量），优化后减为 2 次增量 FK
- 帧索引存储（去除邻近 active link 的冗余近端点）已分析：当前默认阈值下 active link map 不相邻（如 panda [2,4]），ep_data 尺寸节省 0%；仅在严格阈值激活连续 link 时有效，暂不实施

---

### 2026-03-25 — 论文碰撞公式再分行与 Overfull 回归验证

**修改文件**：`paper/root.tex`, `paper/root_cn.tex`, `doc/CHANGE_LOG_CN.md`

**变更内容**：
1. **继续压缩 BitBrick 碰撞判定公式宽度**：
  - 将中英文论文中的 `\text{collide}(G_R, G_O)` 公式进一步拆为三行；
  - 在 `G_R[\bm{b}].\mathrm{words}[k]` 与按位与 `\mathbin{\&}` 之间再次断行，降低单行公式宽度峰值。
2. **重新编译中英文论文进行回归验证**：
  - 重新执行 `pdflatex root.tex` 与 `xelatex root_cn.tex`；
  - 结果显示此前残留在该公式附近的英文/中文 `Overfull \hbox` 告警已不再出现。
3. **确认当前剩余告警类型**：
  - 英文论文当前主要剩余若干段落级 `Underfull \hbox`；
  - 中文论文当前主要剩余若干段落级 `Underfull \hbox`，以及 `IEEEtran` + XeLaTeX 下既有的 `TU/ptm/*` 字体形状回退提示；
  - `microtype` 的 `footnote` patch warning 仍为既有兼容性提示，本轮未引入新的编译错误。

**影响范围**：
- 论文排版：进一步收敛中英文论文的宽公式排版告警
- 行为语义：无变化，仅调整 LaTeX 排版

**备注**：
- 本轮确认中英文论文均可成功编译，且 `collide(G_R,G_O)` 公式已不再是当前主要排版告警来源。

---

### 2026-03-25 — 惰性通道分配 + Z4 短路判断

**修改文件**：`include/sbf/forest/lect.h`, `src/forest/lect.cpp`

**变更内容**：
1. **惰性通道分配**：`ensure_channel_capacity()` 从遍历全部 2 个通道改为仅分配当前活跃通道。新增 `ensure_channel_capacity(min_cap, ch_idx)` 重载供 `load()`、`transplant_subtree()`、`derive_hull_grid()` 等需要指定通道的场景使用。单源管线（>90% 场景）下 UNSAFE 通道零分配。
2. **Z4 短路判断**：`compute_envelope()` 的 Z4 缓存查找路径新增两层短路：(a) `z4_cache_.empty()` 时完全跳过哈希+查找；(b) `sector == 0`（规范扇区）时跳过查找（规范扇区自身不需要从缓存变换）。避免了约 5/6 的冗余 FNV-1a 哈希 + `unordered_map::find()` 调用。

**测试结果**：test_ffb 55/55, test_ifk_pipeline 169/169, test_envelope_pipeline **333/333**（原预存失败修复）。

**性能基准**（exp_v3v4_FFB, 12000 queries）：
- iiwa14: lazy/flat ≈ 0.99-1.01x（噪声范围内）
- panda 5obs: lazy/flat = 0.975x（2.5% 加速）
- 每节点耗时不变（v4_lazy median 50.7/38.2 us vs v4_flat 50.8/38.2 us）
- 改善幅度小于预期，因 MSVC 对单通道 `resize` 的热路径优化有限，Z4 短路节省的哈希计算在总耗时中占比极低

**影响**：内部实现优化，不影响公开 API 签名或缓存文件格式。

---

### 2026-03-25 — 正式构建目录回切与测试目标回归修复

**修改文件**：`tests/test_envelope_pipeline.cpp`, `docs/API_REFERENCE_CN.md`, `paper/root.tex`, `paper/root_cn.tex`, `doc/CHANGE_LOG_CN.md`

**变更内容**：
1. **正式构建目录回切确认**：重新以 `v4/build` 作为正式 CMake 构建目录，明确不再以 `build_check` 承担测试构建职责；并在正式目录下重新配置 `SBF_BUILD_TESTS=ON`、`SBF_BUILD_EXPERIMENTS=ON`。
2. **测试目标正式回归**：在 `v4/build` 下重新编译并顺序运行 `test_ifk_pipeline`、`test_analytical`、`test_crit_gcpc`、`test_envelope_pipeline`、`test_joint_symmetry`、`test_ffb`、`test_iaabb_config` 七个测试目标，最终结果为全部通过。
3. **修复 `test_envelope_pipeline` 的错误测试假设**：
  - 原测试将障碍物放在粗 link AABB 的几何中心，并要求 `collide_aabb_subdiv()` 必中；
  - 该假设在细分包围盒收紧到对角线/非满占据区域时并不成立，可能把“粗盒中心落在空区”的情况误报为 subdiv 漏检；
  - 现改为先调用 `derive_aabb_subdivided(..., 8, ...)` 生成真实细分子盒，再选取其中一个真实 sub-box 的中心放置障碍物，从而只验证“细分表示下真实存在的命中点绝不会漏检”。
4. **同步论文与 API 语义说明**：
  - 在 `docs/API_REFERENCE_CN.md` 中补充 `LinkIAABB(sub=n)` 的语义约束：`n_sub>1` 的碰撞意义是细分子段 iAABB 的并集，而不是要求覆盖粗 `sub=1` AABB 的全部内部点；
  - 在 `paper/root.tex` 与 `paper/root_cn.tex` 的“Link subdivision / 连杆细分”段落补充同样的几何解释，避免论文文字被误读为“粗 AABB 内任一点都必须被某个子盒命中”。
5. **继续统一论文内其余 `LinkIAABB` 措辞**：
  - 将英文论文中“`LinkIAABB` 是 endpoint-iAABB 的并集”修正为更准确的“端点对包围盒经半径膨胀后的连杆盒；若启用细分，则为子段盒并集”；
  - 修正一处将连杆 iAABB 误指向端点 iAABB 公式的表述，并同步更新中文论文对应措辞。
6. **在摘要 / 实验 / 结论首次出现处补充脚注式定义**：
  - 在 `paper/root.tex` 与 `paper/root_cn.tex` 的摘要、完整流水线实验段、结论三处，为 `LinkIAABB` 首次出现位置补充统一脚注；
  - 脚注统一说明其含义为“由端点 iAABB 对导出的连杆轴对齐盒，带半径膨胀；若启用细分，则语义为各细分子段盒的并集”。
7. **将脚注实现改为更稳妥的 LaTeX 形式**：
  - 中英文论文均新增 `\LinkIAABBDef` 统一脚注文本宏，避免三处重复拷贝定义；
  - 摘要中的 `LinkIAABB` 从直接 `\footnote{...}` 改为 `\footnotemark` + 摘要后 `\footnotetext{...}`，降低 `IEEEtran` 下 abstract 内脚注的兼容性风险；
  - 实验段与结论继续复用同一脚注宏，保证中英文文本完全一致。
8. **论文编译回归验证**：
  - 已在 `paper/` 目录执行 `bibtex root`、`bibtex root_cn` 以及中英文论文的两轮 LaTeX 重编译；
  - 先前由单轮编译导致的 citation / reference 未定义警告已消除；
  - 当前剩余警告主要为中英文论文既有的排版类 `Underfull/Overfull \hbox` 与中文 XeLaTeX 字体替代提示，不是本次 `LinkIAABB` 脚注修改引入的问题。
9. **中文论文 XeLaTeX 字体配置微调**：
  - 在 `paper/root_cn.tex` 中显式设置西文字体为 `Times New Roman` / `Arial` / `Courier New`，并为 `SimSun` 补充 `BoldFont=SimHei`、`ItalicFont=KaiTi`、`BoldItalicFont=KaiTi`；
  - 重新编译后，`SimSun` 的 `b/it` 相关字体形状警告已消除；
  - 仍残留的 `TU/ptm/*` 与 `TU/SimSun(0)/m/sc` 警告更偏向 `IEEEtran` + XeLaTeX 的类级字体选择/小型大写形状限制，未在本轮继续做侵入式调整。
10. **继续清理中文 small caps 警告**：
  - 为 `SimSun` 再补充 `SmallCapsFont=SimSun` 映射并重编译验证；
  - 结果显示 `TU/SimSun(0)/m/sc` 警告已消失；
  - 当前中文论文剩余字体类警告主要集中在 `IEEEtran` 的 `TU/ptm/*` 形状回退，不再是 CJK 主字体缺形状问题。
11. **收敛中英文论文的宽公式 Overfull 警告**：
  - 将 `paper/root.tex` 与 `paper/root_cn.tex` 中的三处宽公式改写为 `aligned` 多行形式：两层 pipeline 示意式、认证钳位公式、BitBrick 碰撞判定公式；
  - 重新编译后，原先最显著的几处公式级 `Overfull \hbox` 已明显下降，仅剩较小幅度或段落级排版警告；
  - 当前残留问题主要为少量段落 `Underfull` 与个别较小的 `Overfull`，不再集中在这三处公式本身。

**影响范围**：
- 构建：正式测试流程重新固定在 `build/` 目录
- 测试：`test_envelope_pipeline` 从 332/333 修复为 333/333，全套测试回归为全部通过
- 行为语义：仅修正测试设计，不改变 `collide_aabb_subdiv()` 或 envelope 主实现语义

**备注**：
- 本次修复针对的是测试用例设计错误，而非库实现 bug。
- 这样处理后，`collide_aabb_subdiv()` 的回归断言与其真实几何语义一致：不要求命中“粗 AABB 中任意点”，只要求命中“真实细分子盒覆盖到的点”。

---

### 2026-03-25 — Forest 基础设施修复：NodeStore / KDTree / 高性能 AdjacencyGraph

**修改文件**：`include/sbf/forest/node_store.h`, `src/forest/node_store.cpp`, `include/sbf/forest/kd_tree.h`, `include/sbf/forest/adjacency_graph.h`, `src/forest/adjacency_graph.cpp`, `doc/CHANGE_LOG_CN.md`

**变更内容**：
1. **NodeStore 改为 header-only 简化实现**：在 `node_store.h` 内联实现 `init()`、`ensure_capacity()`、`alloc_node()`、`clear_all_occupation()`、`snapshot()`、`copy_node_from()`、`save()`、`load()`，并将 `node_store.cpp` 收敛为空壳包含文件，避免当前解析环境错误串到旧版本 `NodeStore` 定义。
2. **迁回可用 `KDTree`**：将 `kd_tree.h` 从 stub 升级为可用的 header-only 最近邻实现，补齐 `build()`、`rebuild()`、`add_box()`、`find_nearest()`、递归建树与查询逻辑，供 grower / adjacency 图复用。
3. **`AdjacencyGraph` 升级回 v3 风格高性能版本**：
  - 头文件补回 sweep-and-prune / KD-tree 相关接口与状态；
  - 源文件由基础线性版替换为高性能实现，恢复 `rank_dimensions()`、flat array 重建、按主扫维排序、增量 `add_box()` / `remove_box()`、连通性统计与最近盒查询；
  - 邻接判定恢复为 v3 风格的扁平数组 kernel，并保留 AVX2 / 标量两套路径。
4. **编译诊断回归检查**：本轮修改后，`forest_grower.cpp`、`lect.cpp`、`node_store.h/.cpp`、`kd_tree.h`、`adjacency_graph.h/.cpp` 以及 v4 库源码清单内的 `.cpp` 文件均已恢复为无当前诊断错误状态。

**影响范围**：
- 构建：修复 forest 子系统基础设施，消除 `NodeStore` 相关错误解析阻塞
- 性能：`AdjacencyGraph` 从线性正确性版本恢复到接近 v3 的高性能实现
- 行为语义：不改变 v4 `LECT` 依赖的简化 `NodeStore` API 语义；`ForestGrower` 仍沿用前两阶段已迁移逻辑

**备注**：
- 本次 `NodeStore` 未直接迁回 v3 的重型 flat-buffer 版本，而是保留 v4 当前 `LECT` 所需的简化接口集合；这是为兼容 v4 当前结构并优先恢复可编译状态。
- `AdjacencyGraph` 升级建立在前一阶段“基础线性版可用实现”之上，属于性能恢复而非接口新增。

---

### 2026-03-25 — ChannelData flat-buffer 重构 (v4.2)

**修改文件**：`include/sbf/forest/lect.h`, `src/forest/lect.cpp`, `include/sbf/envelope/envelope_derive.h`

**变更内容**：
1. **ep_store 扁平化**：`ChannelData::ep_store`（`vector<vector<float>>`）→ `ep_data`（`vector<float>`）+ stride 索引。消除每节点堆分配，改为连续内存布局 `ep_data[node * ep_stride_]`。
2. **link_iaabb_cache 预计算缓存**：`ChannelData` 新增 `link_iaabb_cache`（`vector<float>`），在 `compute_envelope()` / `refine_aabb()` 时一次性调用 `derive_aabb_paired()` 填充；`iaabbs_collide()` 热路径直接指针读取，零推导。
3. **has_data 位标记**：`ChannelData` 新增 `vector<uint8_t> has_data` 替代原 `ep_store[i].empty()` 判断。
4. **相关函数同步**：`ensure_channel_capacity`, `init_root`, `compute_envelope`, `refine_aabb`, `iaabbs_collide`, `derive_link_iaabbs`, `get_endpoint_iaabbs`, `has_endpoint_iaabbs`, `derive_hull_grid`, `load`, `snapshot`, `transplant_subtree` 均已适配新布局。

**测试结果**：test_ffb 55/55, test_ifk_pipeline 169/169, test_envelope_pipeline 332/333（1例预存失败不变）。

**性能基准**（exp_v3v4_FFB, 12000 queries）：
- iiwa14 5obs: v4_flat 568us vs v4_BT 565us (flat/BT=1.005)
- panda 20obs: v4_flat 1350us vs v4_BT 1357us (flat/BT=0.995)
- 每节点耗时未见统计显著变化（v4_BT median 50.7/37.9 → v4_flat 50.8/38.2 us）

**分析**：MSVC Release 分配器对小 vector 本身表现优异，flat-buffer 消除的堆散列开销不构成瓶颈。剩余 ~40% 每节点差距来源于 v4 endpoint-based 表示法的固有数据量（panda: 96 ep floats + 48 link floats = 144 vs v3 的 48 link floats），导致更高 cache pressure，以及 NodeStore SoA / 双通道间接寻址等结构性开销。

**影响**：内部实现重构，不影响公开 API 签名或缓存文件格式。

---

### 2026-03-24 — 论文 Table 1 实验重跑与全量数据更新

- 修改文件 / 模块：
  - `paper/root.tex` — Table 1 Cold 列、实验描述、观察文本、推荐配置、摘要、贡献、结论共 19 处
  - `paper/root_cn.tex` — 中文镜像，同步更新 19 处
  - `doc/CHANGE_LOG_CN.md` — 本条
- 变更内容：
  - **重跑 `exp_v3v4_epiAABB`**（50 trials, seed=42）获取 S4+S5+S1+S9 优化后最新数据
  - **Table 1 Cold 列更新**：
    - IFK: 0.023 → 0.022 ms
    - CritSample: 0.087 → 0.039 ms（2.2× 加速，受益于 S4 跳过浅连杆）
    - Analytical: 154.8 → 9.0 ms（17× 加速，DH 直接系数提取 + 全部优化）
    - GCPC: 72.4 → 7.0 ms（10× 加速，Analytical 边界 + 缓存内部架构升级）
  - **速度比更新**：GCPC/Analytical 从 $2.1\times$ → $1.3\times$（全文 6 处）
  - **实验描述更新**：IFK ~25→~22μs, Analytical ~155→~9ms, GCPC ~72→~7ms
  - **CritSample vs GCPC 对比**：192μs → 7ms（反映 GCPC 全流水线实际代价）
  - **GCPC 引言**：逐盒代价 ~150ms → ~9ms
  - **推荐配置**：CritSample 冷启动 0.087→0.039ms, GCPC 验证 72→7.0ms
  - Volume/Voxels 列保持不变（Stage 2 代码未改动，仅 Stage 1 源计算加速）
- 影响范围：
  - 论文数据一致性：Table 1 及所有引用数字反映当前 v4 代码实际性能
  - 无代码行为变更
- 备注：
  - 实验数据来源：`results/exp_v3v4_epiAABB_v4_latest.csv`
  - 交叉验证：`results/exp_ep_iaabb_s1s9/results.csv`（Analytical=9.322, GCPC=7.114）

### 2026-03-24 — IAABBConfig: 通用 DH→iAABB 预处理配置

- 修改文件 / 模块：
  - `include/sbf/robot/iaabb_config.h` — 新建，IAABBConfig 结构体定义
  - `src/robot/iaabb_config.cpp` — 新建，from_robot() + JSON 序列化 + summary()
  - `include/sbf/robot/robot.h` — 新增 `IAABBConfig iaabb_config_` 私有成员及 `iaabb_config()` 访问器
  - `src/robot/robot.cpp` — pack_arrays() 末尾自动构建 `IAABBConfig::from_robot(*this)`
  - `CMakeLists.txt` — 添加 iaabb_config.cpp 源文件 + test_iaabb_config 测试目标
  - `tests/test_iaabb_config.cpp` — 新建，9 个测试（Panda/IIWA14/自定义3-DOF/JSON往返/边界情况）
  - `docs/API_REFERENCE_CN.md` — 新增 `robot/iaabb_config` API 章节
  - `docs/PAPER_CODE_MAP.md` — 新增 robot/iaabb_config 代码映射
  - `paper/root.tex` — 更新活动连杆过滤段落（加入阈值描述）
  - `paper/root_cn.tex` — 同步更新中文论文
  - `doc/CHANGE_LOG_CN.md` — 本条
- 变更内容：
  - **新增 `IAABBConfig` 结构体**：通用 DH 参数预处理器，将任意 N-DOF 串联 DH 链转换为 iAABB 生成专用配置
  - **连杆长度公式**：$\sqrt{a^2 + d^2}$（DH 参数 a、d 的固有长度，与关节角无关）
  - **零长度阈值**：长度 < 0.1m 的连杆视为零长度，从活跃连杆中排除
  - **`from_robot()` 工厂方法**：7 步通用预处理——长度计算、活跃判定、索引映射、截止帧、长度/半径/影响关节数
  - **JSON 持久化**：`save_json()` / `load_json()`，格式 `iaabb_config v1`，支持完整往返
  - **`summary()` 调试输出**：打印全部连杆状态（ACTIVE/SKIP: base/SKIP: < 0.1）
  - **Robot 集成**：Robot 中自动构建并缓存 IAABBConfig（`Robot::iaabb_config()` 访问器）
  - **向后兼容**：Robot 原有 `active_link_map_`（阈值 1e-10）保持不变，IAABBConfig 使用独立的 0.1 阈值
  - **Panda 结果**：active_link_map 从 [2,3,4,6]（1e-10）变为 [2,4]（0.1），link 3(a=0.0825) 和 link 6(a=0.088) 被过滤
  - **IIWA14 结果**：从 [2,4,6,7]（1e-10）变为 [2,4,7]（0.1），link 6(d=0.081) 被过滤
- 影响范围：
  - API：新增 `IAABBConfig` 结构体 + `Robot::iaabb_config()` 访问器
  - 构建：新增 1 个源文件 + 1 个测试目标
  - 行为语义：Robot 原有行为不变，IAABBConfig 是独立的新增能力
  - 测试：113 PASS / 0 FAIL（test_iaabb_config），全部既有测试无回归
- 备注：
  - 后续迁移计划：LECT、EndpointSource 等消费者将切换至 `Robot::iaabb_config()` 的 active_link_map
  - 这将启用配对端点布局（ep_store_ paired layout）和逐链接关节裁剪优化

### 2026-03-24 — S1+S9 文档同步（论文描述 + API 参考）

- 修改文件 / 模块：
  - `doc/PAPER_STAGE1_ENDPOINT_IAABB.md` — 更新 benchmark 表（9.535→9.322, 7.407→7.114）、新增 per-width 表、已应用优化列表、P1 suffix 缓存描述
  - `docs/API_REFERENCE_CN.md` — 新增 `ConfigVec` 类型文档、更新 `LinkExtremes` 结构体定义（`VectorXd` → `ConfigVec`）、新增 `compute_suffix_pos` S1 缓存说明
  - `doc/CHANGE_LOG_CN.md` — 本条
- 变更内容：
  - PAPER_STAGE1 benchmark 表更新为 S4+S5+S1+S9 后的最新数据，附带 per-width 分解和累计加速百分比
  - API_REFERENCE 中 `LinkExtremes` 结构体从 `float lo[3]/hi[3]` + `VectorXd cfg[6]` 修正为 `double vals[6]` + `ConfigVec configs[6]`（与代码实际定义一致）
  - 新增 `ConfigVec` 类型别名文档说明（`Eigen::Matrix<double, Dynamic, 1, 0, 16, 1>`）
  - `compute_suffix_pos` 添加 S1 缓存复用说明
- 影响范围：
  - 文档一致性：论文描述、API 参考均反映最新优化状态
  - 无代码行为变更

### 2026-03-24 — Analytical/GCPC 第一批速度优化 (S4+S5)

- 修改文件 / 模块：
  - `include/sbf/envelope/analytical_utils.h` — S4 + S5 优化
  - `tests/test_crit_gcpc.cpp` — 修复 `n_endpoints` → `n_active_ep`（预存问题）
  - `tests/test_envelope_pipeline.cpp` — 同上
  - `tests/test_ifk_pipeline.cpp` — 同上
  - `tests/test_iaabb_config.cpp` — 修复 `HALF_PI` 命名冲突
- 变更内容：
  - **S4: `eval_and_update_from()` 跳过浅层 link**：添加 `if (V < from_joint) continue;`，
    当 partial FK 从 `from_joint` 开始时，V < from_joint 的 link 位置未变化，无需更新。
  - **S5: `solve_poly_in_interval()` 自适应子区间数**：根据 tan(θ/2) 区间宽度自动选择
    NSUB（窄 <0.5→12、中 <2.0→20、宽→32），减少窄区间不必要的搜索量。
  - **S3 面对预过滤**：已实现并测试，但 benchmark 显示有回归（vector<vector> 分配开销），已回退。
  - **S8 滚动 AA 刷新**：分析后认为边际收益不足，跳过。
  - 修复多个测试文件中 `EndpointIAABBResult::n_endpoints` 引用（已在之前重构中改为 `n_active_ep`）。
- Benchmark 结果对比（50 trials, iiwa14, seed 42，体积完全不变）：

  | 管线 | 基线(ms) | 优化(ms) | 加速 |
  |------|----------|----------|------|
  | Analytical (narrow) | 3.964 | 3.887 | 1.9% |
  | Analytical (medium) | 9.703 | 9.639 | 0.7% |
  | Analytical (wide) | 20.341 | 20.021 | 1.6% |
  | Analytical (ALL) | 9.535 | 9.415 | **1.3%** |
  | GCPC (narrow) | 2.160 | 2.032 | **5.9%** |
  | GCPC (medium) | 7.235 | 7.082 | 2.1% |
  | GCPC (wide) | 18.243 | 17.564 | 3.7% |
  | GCPC (ALL) | 7.407 | 7.158 | **3.4%** |

- 影响范围：
  - 性能：Analytical 平均加速 ~1.3%，GCPC 平均加速 ~3.4%（narrow 最高 5.9%）
  - 质量：零损失（体积完全一致 0.4115/0.4113）
  - API：无变化
  - 测试：740 PASS / 1 pre-existing FAIL（`collide_aabb_subdiv` 与优化无关）
- 备注：
  - GCPC 受益更大，因为不走 P3，S4 的 skip-shallow-links 在边界阶段占比更高
  - 后续第二批优化方向：S1 suffix 缓存/双层 bg 枚举（预估 20-30%），S9 VectorXd → 固定数组
  - 详见 `doc/OPTIMIZATION_PLAN_ANALYTICAL_SPEED.md`

### 2026-03-24 — Analytical/GCPC 第二批速度优化 (S1+S9)

- 修改文件 / 模块：
  - `src/envelope/analytical_solve.cpp` — S1: P1 suffix 缓存
  - `include/sbf/envelope/analytical_utils.h` — S9: `LinkExtremes::configs` 从 VectorXd 改为 ConfigVec
  - `doc/CHANGE_LOG_CN.md` — 本条
- 变更内容：
  - **S1: P1 (solve_edges) suffix 缓存**：bg combo 的位编码中 bits 0..j-1 对应 prefix 关节，
    bits j..nj-2 对应 suffix 关节。由于 bg 顺序递增时 suffix bits (bg >> j) 每 2^j 次才变化，
    而 `compute_suffix_pos()` 仅依赖 q[j+1..V]，因此缓存 suffix 结果，只在 suffix bits
    变化时重新计算。对 V=6, j=3: suffix 计算从 64→8 次（节省 87.5%）。
  - **S9: ConfigVec 消除堆分配**：`LinkExtremes::configs[6]` 从 `Eigen::VectorXd`（堆分配）
    改为 `Eigen::Matrix<double, Dynamic, 1, 0, 16, 1>`（内联存储最多 16 关节），
    避免 init() 时 6×N 次 heap allocation。API 完全兼容（Eigen 自动类型转换）。
- Benchmark 结果对比（50 trials, iiwa14, seed 42，体积完全不变）：

  | 管线 | S4+S5(ms) | S4+S5+S1+S9(ms) | 增量加速 |
  |------|-----------|-----------------|----------|
  | Analytical (narrow) | 3.887 | 3.889 | ~0% |
  | Analytical (medium) | 9.639 | 9.405 | **2.4%** |
  | Analytical (wide) | 20.021 | 20.021 | ~0% |
  | Analytical (ALL) | 9.415 | 9.322 | **1.0%** |
  | GCPC (narrow) | 2.032 | 1.991 | **2.0%** |
  | GCPC (medium) | 7.082 | 7.011 | **1.0%** |
  | GCPC (wide) | 17.564 | 17.564 | ~0% |
  | GCPC (ALL) | 7.158 | 7.114 | **0.6%** |

  **累计总加速（从原始基线）**：

  | 管线 | 原始基线(ms) | 累计优化(ms) | 总加速 |
  |------|-------------|-------------|--------|
  | Analytical | 9.535 | 9.322 | **2.2%** |
  | GCPC | 7.407 | 7.114 | **4.0%** |

- 影响范围：
  - 性能：S1+S9 增量 ~1%，累计 ~2-4%（medium 宽度受益最大 ~2.4%）
  - 质量：零损失（体积完全一致 0.4115/0.4113）
  - API：无变化（ConfigVec 与 VectorXd 完全兼容）
  - 测试：740 PASS / 1 pre-existing FAIL
- 备注：
  - S1 仅加速 P1 edge solver 中的 suffix 重计算，P1 占总耗时 ~15%，
    suffix 计算本身仅占 P1 小部分，因此实际全局加速有限（~1% vs 预估 20-30%）
  - 瓶颈已转移到 eval_and_update_from 的 partial FK + 多项式求根等更重的操作
  - S9 因 VectorXd 在 init() 后的 update() 中无需重分配，实际堆分配消除影响微弱
  - 后续可考虑将 S1 扩展到 P2 (solve_faces) 和 P2.5 (solve_pair_constrained)，
    但实现更复杂（双活跃关节），预期收益也有限

### 2026-03-24 — 4-way Benchmark 结果确认 + 文档同步

- 修改文件 / 模块：
  - `doc/PAPER_STAGE1_ENDPOINT_IAABB.md` — 基线 benchmark 表格更新、Analytical/GCPC 描述重写、质量排序/安全类更新
  - `docs/API_REFERENCE_CN.md` — EndpointSource 枚举表增加安全类列、CritSample UNSAFE 标注
  - `experiments/exp_ep_iaabb_bench.cpp` — 4 种方法注释更新为当前实现描述
  - `doc/CHANGE_LOG_CN.md` — 本条
- 变更内容：
  - **4-way Benchmark 最新结果**（iiwa14, 50 trials, seed 42, Release）：
    | 管线 | 耗时 (ms) | 体积 | VolRatio vs IFK | FK evals |
    |------|-----------|------|-----------------|----------|
    | IFK | 0.021 | 2.512 | 1.0000 | 0 |
    | CritSample | 0.037 | 0.393 | 0.157 | 511 |
    | Analytical | 9.535 | 0.412 | 0.164 | 6948 |
    | GCPC | 7.407 | 0.411 | 0.164 | 3343 |
  - **CritSample 体积最小的解释**：CritSample (UNSAFE) 因采样遗漏导致 iAABB 偏小 ~4%，
    并非更精确，而是不完备（under-approximation）。所有 SAFE 方法体积一致。
  - **GCPC 比 Analytical 快 ~22%**：7.4ms vs 9.5ms，因跳过 P3 坐标下降，用缓存查找替代。
  - 文档同步：论文描述、API 参考均更新为当前 4 种方法的实际实现
- 影响范围：
  - 文档一致性：论文/API/实验注释三处统一
  - 无代码行为变更

---

### 2026-03-24 — GCPC 升级：Analytical 边界求解 + GCPC 缓存内部

- 修改文件 / 模块：
  - `src/envelope/endpoint_source.cpp` — GCPC case 全面重写（full + incremental 两处）
- 变更内容：
  - **GCPC 方法从 kπ/2 枚举升级为 Analytical 边界求解 + GCPC 缓存内部**：
    - 旧实现：GCPC = `derive_crit_endpoints`（GCPC cache lookup + kπ/2 笛卡尔积枚举），边界求解仅覆盖 box 顶点。
    - 新实现：GCPC = `derive_aabb_critical_analytical`（P0+P1+P2+P2.5+AA剪枝，`enable_interior_solve=false`）+ GCPC 缓存查找内部临界点。
    - 边界阶段与 Analytical 方法完全一致：P0 kπ/2 顶点、P1 atan2 边、P2 八次多项式面、P2.5 约束面、AA 仿射剪枝
    - 内部阶段用 GCPC 预计算缓存替代 P3 坐标下降启发式
  - 新增 `gcpc_cache_interior_expand()` 辅助函数：从 GCPC 缓存查找匹配的内部临界点，去重后 FK 计算并展开 endpoint iAABBs
  - 新增 `update_endpoint_iaabb_from_ws()` / `discretize_config_ep()` / `DedupKeyHash_EP` 辅助工具
- 影响范围：
  - GCPC 方法的边界求解质量与 Analytical 持平（之前仅 kπ/2 枚举，缺少 P1/P2/P2.5 边/面临界点）
  - GCPC 方法新增 AA 剪枝支持，可跳过无需改善的 link
  - GCPC 整体耗时会增加（增加了 P1-P2.5 边界阶段），但内部阶段比 Analytical P3 快（缓存查找 vs 坐标下降）
- 验证：741 测试全通过（test_analytical 39 + test_envelope_pipeline 333 + test_crit_gcpc 369）

---

### 2026-03-24 — source_can_serve 放宽：SAFE 缓存可替代 UNSAFE 请求

- 修改文件 / 模块：
  - `include/sbf/envelope/endpoint_source.h` — `source_can_serve()` 逻辑更新 + 注释更新
  - `tests/test_ffb.cpp` — 跨类替代测试用例更新
- 变更内容：
  - `source_can_serve()` 原规则：跨安全类一律禁止替代
  - 新规则：SAFE→UNSAFE 允许（保守外包是有效替代），UNSAFE→SAFE 仍禁止
  - 物理依据：SAFE 源（iFK/Analytical/GCPC）产生的 iAABB 保证 ⊇ 真实工作空间，对 CritSample 请求是合法（较宽松）的替代；但 CritSample 可能漏掉极值导致 iAABB 偏小，不能替代 SAFE 请求
  - 测试更新：`IFK/GCPC/Analytical → CritSample` 三条断言从 `CHECK(!...)` 改为 `CHECK(...)`
- 影响范围：
  - 行为语义：当节点已有 SAFE 缓存时，CritSample 请求不再强制重算，直接复用缓存
  - 缓存命中率：提升（减少不必要的 CritSample 重算）

### 2026-03-24 — pick_split_dim 重构：消除维度选取冗余

- 修改文件 / 模块：
  - `include/sbf/forest/lect.h` — 新增 `pick_split_dim()` 声明；`split_leaf()` 新增可选 `int dim = -1` 参数
  - `src/forest/lect.cpp` — 新增 `pick_split_dim()` 实现；`split_leaf()` / `find_free_box()` / `pre_expand_recursive()` / `expand_leaf()` 统一调用
  - `docs/API_REFERENCE_CN.md` — `find_free_box` 流程描述更新（第 5、6 步）
- 变更内容：
  - 提取 `pick_split_dim(depth, fk, intervals)` 作为维度选取的唯一入口，统一处理 WIDEST_FIRST / BEST_TIGHTEN / ROUND_ROBIN 三种策略
  - `split_leaf()` 新增 `int dim` 参数（默认 -1 表示由函数内部自行调用 `pick_split_dim`），已由调用方预选时直接透传，避免重复计算
  - 删除了 `find_free_box()` 叶块、`pre_expand_recursive()` 叶块中各自独立的维度选取 switch 逻辑（原为三份相同代码，现合并为一份）
  - `expand_leaf()` 也改为先调用 `pick_split_dim()` 再传入 `split_leaf()`
- 影响范围：
  - 行为语义：无变化，`pick_split_dim()` 内部逻辑与原三处逻辑完全一致
  - 维护性：三处重复代码合并为一处，后续修改维度策略只需改一个函数
  - API：`split_leaf()` 签名新增可选参数 `dim`（默认 -1，向后兼容）
- 备注：
  - 此前 `find_free_box()` 和 `pre_expand_recursive()` 中的维度选取代码是 `split_leaf()` 内部逻辑的复制粘贴，仅用于预检边宽后提前退出。现统一由 `pick_split_dim()` 提供，确保一致性。

### 2026-03-27 — Analytical 完备性修复（修复1 + 修复2）

- 修改文件 / 模块：
  - `src/envelope/endpoint_source.cpp` — `remap_analytical_ep_to_endpoint_iaabbs()` 重写
  - `include/sbf/envelope/analytical_solve.h` — `AnalyticalCriticalConfig::face_all_pairs` 默认值
- 变更内容：
  - **修复1：Analytical remap 去除 iFK 交集收紧，改为直接覆盖**：
    - 旧逻辑：先用 iFK 初始化所有 endpoint iAABB，再对被 active link 引用的 endpoint 取交集（max-of-lo, min-of-hi）收紧。问题：解析求解器输出是内逼近（不保守），交集只会让结果更紧而非更安全。
    - 新逻辑：先用 iFK 初始化所有 endpoint（为未引用的 endpoint 如 tool tip 提供有效值），然后标记被 active link 覆盖的 endpoint，将其初始化为空集 [+inf, -inf]，再用解析求解器输出做 union（min-of-lo, max-of-hi）展开。这样被引用的 endpoint 完全由解析结果决定，未引用的保留 iFK 兜底。
    - 多条 active link 共享同一 endpoint（如 link i 的 distal = link i+1 的 proximal）时，各自的解析结果取并集，保证不遗漏。
  - **修复2：`face_all_pairs` 默认改为 `true`**：
    - P2 阶段（2D 面求解器，八次多项式）此前默认仅处理 coupled + adjacent 关节对，遗漏了非耦合非相邻对的 C(n,2) 面临界点。
    - 改为 `true` 后遍历所有 C(n,2) 关节对，保证 2D 面的理论完备性。
- 影响范围：
  - Analytical endpoint source 输出语义：被引用 endpoint 现为纯解析结果（不再与 iFK 混合）
  - P2 阶段计算量增大（所有关节对 vs 仅耦合+相邻对），换取完备性
- 验证：741 测试全通过（test_analytical 39 + test_envelope_pipeline 333 + test_crit_gcpc 369）
- 备注：
  - 完备性缺口分析还发现 k≥3 面无专门阶段（P3 坐标下降为启发式），但暂不修复，作为后续改进方向。

---

### 2026-03-24 — FFB 迁移 + 多通道缓存架构 + Z4 在线缓存

- 修改文件 / 模块：
  - `include/sbf/envelope/endpoint_source.h` — 新增 SourceSafetyClass、source_safety_class()，重写 source_can_serve() 和 source_quality_sufficient()
  - `include/sbf/forest/lect.h` — 更新 Z4CacheEntry（quality→source）、多通道缓存注释
  - `src/forest/lect.cpp` — 构造函数自动启用 Z4 缓存、compute_envelope 使用安全类质量检查、hull 失效逻辑、Z4 缓存源兼容性检查、pre_expand 不再禁用 Z4 缓存、load 恢复 Z4 状态
  - `tests/test_ffb.cpp` — 新增 9 组 FFB + 多通道缓存测试（53 项检查）
  - `CMakeLists.txt` — 新增 test_ffb 构建目标
- 变更内容：
  - **Source 安全类分离（核心设计变更）**：
    - 将 EndpointSource 分为两个不可互相替代的安全类：
      - SAFE（保守边界保证）：IFK ≤ Analytical ≤ GCPC，同类内高质量可替代低质量
      - UNSAFE（更紧但不保守）：CritSample，独立通道，不可与 SAFE 互换
    - source_can_serve() 现在先检查安全类，再检查质量——CritSample 无法替代 IFK/Analytical/GCPC，反之亦然
  - **Link iAABB 双通道架构（单树、多层缓存）**：
    - AABB 通道（NodeStore 中的 link_iaabbs）：轴对齐碰撞检测、intersect-of-union 合并
    - Grid 通道（hull_grids_ 中的 VoxelGrid）：BitBrick 碰撞检测、bitwise-OR 合并
    - 两通道独立存储、独立失效：AABB 源发生变更时自动 hull_valid_=0，hull 延迟重新推导
  - **Z4 缓存扩展到 FFB 在线下降**：
    - 构造函数中自动启用 Z4 缓存（原仅 pre_expand 临时启用）
    - 缓存在 pre_expand 开始时清空，之后持续有效并在 FFB 调用间增长（跨对称扇区受益）
    - Z4 缓存条目现存储 EndpointSource 枚举而非质量值，支持源兼容性检查
    - Z4 缓存写入支持源升级：如果新源质量更高，覆盖旧条目
  - **Hull 失效机制**：compute_envelope 重新计算 AABB 时，自动清除对应节点的 hull_valid_，确保 hull 从新端点重新推导
  - **snapshot/load Z4 状态保持**：snapshot 复制 z4_cache_active_，load 重新检测并启用 Z4

- 影响范围：
  - API 行为：source_can_serve() 语义变更——CritSample 不再被视为可替代 IFK（修正了旧的线性质量排序错误）
  - 缓存语义：Z4 缓存在 FFB 期间活跃，跨 FFB 调用持续有效
  - 性能：Z4 缓存在 FFB 中可跳过对称扇区的 FK 计算（~4× 理论加速于 q₀ 维度）
  - 构建：新增 test_ffb 目标
- 备注：
  - 设计决策：单树 + 多层存储（而非每种 pipeline 一棵独立树），因为树拓扑由 C-space 几何决定，与 pipeline 无关
  - Hull VoxelGrid 的 Z4 旋转变换复杂度较高且收益有限，暂不实现（hull 推导本身较轻量）
  - 此变更回答了三个设计问题：(1) 缓存优先 + 质量替代规则 (2) 单树多通道 vs 多树 (3) FFB 中使用 q0 对称性

### 2026-03-24 — Analytical endpoint source 接入六阶段解析求解器

- 修改文件 / 模块：
  - `src/envelope/endpoint_source.cpp` — Analytical full/incremental case 改为调用 `derive_aabb_critical_analytical()`
  - `experiments/exp_ep_iaabb_bench.cpp` — Analytical 配置改为 `EndpointSourceConfig::analytical()`（不再手动设 gcpc_cache）
  - `experiments/exp_small_ep_viz.cpp` — 同上
  - `doc/PLAN_ANALYTICAL_ENDPOINT.md` — 解析解集成计划文档
- 变更内容：
  - **核心改动**：`EndpointSource::Analytical` 现在真正调用 `derive_aabb_critical_analytical()` 六阶段解析求解器
    （P0 kπ/2 顶点 + P1 atan2 边 + P2 8次多项式面 + P2.5 对约束 + P3 坐标下降内部），
    而非之前退化为与 CritSample 相同的 kπ/2 枚举。
  - **新增辅助函数** `remap_analytical_ep_to_endpoint_iaabbs()`：将解析求解器的
    per-link per-endpoint 输出 `[(n_active × 2) × 6]` 重映射为标准 `endpoint_iaabbs[n_endpoints × 6]` 格式。
    初始化策略：先从 iFK 填充所有 endpoint（保证 tool 等非活动链接引用的 endpoint 有效），
    再用解析结果做交集收紧（max-of-lo, min-of-hi）。
  - **增量路径**：Analytical incremental 现在也调用完整解析求解器（无增量优化，留作后续开发）。
  - **四种方法现在真正各不相同**：
    - IFK = 区间 FK
    - CritSample = kπ/2 边界枚举（无缓存）
    - Analytical = 六阶段解析梯度零点求解器（最紧） ← **本次改动**
    - GCPC = kπ/2 枚举 + GCPC 缓存查找
  - **实验脚本去缓存**：两个实验中 `cfg_anal` 不再设置 `gcpc_cache`。
- 影响范围：
  - API 行为：Analytical 输出将比此前更紧（六阶段 vs 仅 kπ/2），但耗时更高
  - 性能：Analytical 耗时将高于其他三种方法（因完整解析求解）
  - 语义：Analytical 终于名副其实——使用解析求解而非退化枚举
- 验证：
  - 构建成功（0 error / 0 warning），全部测试通过（39 + 333 + 369 = 741 pass / 0 fail）
  - `doc/CHANGE_LOG_CN.md`

---

### 2026-03-24 — 新增 endpoint iAABB 包络 3D 可视化（ep_iaabb_viz）

- 修改文件 / 模块：
  - `include/sbf/viz/viz_exporter.h` — 新增 `export_ep_iaabb_comparison_json()` 声明
  - `src/viz/viz_exporter.cpp` — 新增 `export_ep_iaabb_comparison_json()` 实现
  - `python/sbf4_viz/ep_iaabb_viz.py` — 新增 Python 可视化模块
  - `python/sbf4_viz/__init__.py` — 导出 `load_ep_iaabb_comparison`, `plot_ep_iaabb_comparison`
  - `experiments/exp_ep_iaabb_bench.cpp` — 替换 viz 导出调用（→ `export_ep_iaabb_comparison_json`）
  - `scripts/viz_ep_iaabb_bench.py` — 新增 `plot_ep_iaabb_3d()`，输出 `ep_iaabb_3d.html`
- 变更内容：
  - **新 C++ 函数** `export_ep_iaabb_comparison_json()`：
    - 输入：`Robot`, `box_intervals`, `sources`（多 Stage-1 源的 endpoint_iaabbs 列表）
    - 输出 JSON：`robot_name / n_joints / n_active_links / link_radii / active_link_map / n_endpoints / box_intervals / robot_arm.link_positions / methods.<source>.endpoint_iaabbs[{endpoint, lo[3], hi[3]}]`
    - 自动计算 FK 中点用于在 3D 图中绘制机器人臂骨架
  - **新 Python 模块** `sbf4_viz/ep_iaabb_viz.py`：
    - 数据类：`EndpointIAABB`, `EpIAABBSource`, `EpIAABBComparison`
    - `load_ep_iaabb_comparison(json_path)` — 加载 C++ 导出的 JSON
    - `plot_ep_iaabb_comparison(data, ...)` — 渲染交互式 3D 图：
      - 机器人臂骨架（FK 中点折线）
      - 每个 source 的所有 endpoint iAABB 以线框 + 半透明填充方式叠加
      - 每个 source 为独立 legendgroup，点击图例可隐藏/显示
      - 方法颜色：IFK(红) / CritSample(蓝) / Analytical(橙) / GCPC(绿)
  - **`exp_ep_iaabb_bench.cpp` 更新**：viz_box 阶段改为调用 `export_ep_iaabb_comparison_json`，输出从 `comparison.json` → `ep_iaabb_comparison.json`
  - **`viz_ep_iaabb_bench.py` 更新**：第 5 个 HTML 从 `comparison_3d.html`（旧 envelope 对比）→ `ep_iaabb_3d.html`（endpoint iAABB 包络对比）
- 效果：
  - IFK 端点框最粗（体积 ~0.48 m³）；Analytical/CritSample/GCPC 更紧密（体积 ~0.17 m³）
  - 66 个 Plotly traces（1 臂 + 4 × 8 端点 × 2 wire/fill + 1 中心标记）
- 备注：
  - 不影响核心库 API 和测试
  - 结果文件：`results/<run>/ep_iaabb_comparison.json` 和 `ep_iaabb_3d.html`

### 2026-03-24 — 新增 exp_ep_iaabb_bench：Endpoint iAABB Stage 1 方法耗时 & 体积对比实验

- 修改文件 / 模块：
  - `experiments/exp_ep_iaabb_bench.cpp` — 新增 C++ 实验程序
  - `scripts/viz_ep_iaabb_bench.py` — 新增 Python 可视化脚本
  - `CMakeLists.txt` — 新增 `exp_ep_iaabb_bench` 构建目标
- 变更内容：
  - **实验目的**：系统对比 4 种 Stage 1 端点源方法（IFK / CritSample / Analytical / GCPC）的耗时和 per-link body AABB 体积，固定 Stage 2 为 extract_link_iaabbs()（derive_aabb，n_sub=1）
  - **C++ 实验**（`exp_ep_iaabb_bench.cpp`）：
    - 仅调用已有 public API（`compute_endpoint_iaabb`, `extract_link_iaabbs`, `sbf::viz::export_envelope_comparison_json`）
    - 100 次随机 C-space box，40% narrow / 40% medium / 20% wide 宽度分布
    - 每次 5 次重复取中位数
    - GCPC 缓存：支持命令行 `[gcpc_cache]` 参数加载；否则在线构建（kπ/2 枚举 + enrich_with_interior_search）
    - 输出：`results/exp_ep_iaabb_bench_<timestamp>/results.csv` 和 `comparison.json`
    - CSV 列：`source, trial, width_frac, n_active, volume, time_ms, n_eval`
  - **Python 可视化**（`viz_ep_iaabb_bench.py`）：
    - 读取 `results.csv`，生成 5 个 HTML 可视化文件：
      1. `timing_boxplot.html` — 各方法 Stage 1 耗时箱线图（按宽度类别分 3 列）
      2. `volume_boxplot.html` — 各方法体积箱线图（按宽度）
      3. `timing_vs_volume.html` — 耗时 vs 体积散点图（方法着色，形状区分宽度）
      4. `summary_bar.html` — 均值汇总：耗时柱图 + 体积菱形线（双 Y 轴）
      5. `comparison_3d.html` — 代表性配置的 3D 对比（调用 sbf4_viz，可选）
    - 终端打印汇总统计表（全 trial 均值 + 按宽度细分）
- 影响范围：
  - 新增 experiment / script，不影响核心库 API
  - 构建：新增 1 个 CMake 实验目标（`SBF_BUILD_EXPERIMENTS=ON` 时生效）
- 备注：
  - 遵循 spec §3 §9 §10：仅调用 public API，输出到 `results/`，不引入新实现
  - 使用方法：
    ```
    cmake --build build --config Release --target exp_ep_iaabb_bench
    build\Release\exp_ep_iaabb_bench.exe 100 42
    python scripts/viz_ep_iaabb_bench.py results/exp_ep_iaabb_bench_<timestamp>/
    ```

### 2026-03-24 — Stage 1 endpoint iAABB 纯化：移除 analytical_link_aabbs

- 修改文件 / 模块：
  - `include/sbf/envelope/endpoint_source.h` — EndpointIAABBResult 移除 analytical_link_aabbs 字段及 has_analytical_link_aabbs() 方法
  - `src/envelope/endpoint_source.cpp` — compute_endpoint_iaabb() Analytical 分支移除 derive_aabb_critical_analytical(n_sub=1) 调用；extract_link_iaabbs() 移除 analytical 快速路径，改用 derive_aabb()（无 n_sub 参数）
  - `tests/test_analytical.cpp` — 重写 test_endpoint_source_dispatch，验证 Analytical 仅产生 endpoint iAABBs
  - `tests/test_crit_gcpc.cpp` — 移除 analytical_link_aabbs.empty() 断言
- 变更内容：
  - **架构纯化**：严格遵循 spec §6 两阶段流水线分离
    - Stage 1 (EndpointSource): 仅输出 endpoint_iaabbs [n_endpoints × 6]，不涉及任何 per-link 计算或 sublink 切分
    - Stage 2 (EnvelopeType): endpoint_iaabbs → LinkEnvelope，sublink 概念仅在此引入
  - **移除项**：
    - `EndpointIAABBResult::analytical_link_aabbs` — per-link 数据不属于 Stage 1 输出
    - `EndpointIAABBResult::has_analytical_link_aabbs()` — 配套方法
    - Analytical case 中 `derive_aabb_critical_analytical(n_sub=1)` 调用 — per-link AABB 计算属于 Stage 2
    - `extract_link_iaabbs()` 中 analytical 直接路径 + IFK clamp 逻辑
    - `endpoint_source.cpp` 中 `#include "sbf/envelope/analytical_solve.h"`（Stage 1 不再直接调用 analytical solver）
  - **替换**：`extract_link_iaabbs()` 默认路径从逐链接 `derive_aabb_subdivided(n_sub=1)` 改为 `derive_aabb()`（批量处理，无 sublink 语义）
  - **Analytical Stage 1 行为变化**：Analytical endpoint source 现在通过 derive_crit_endpoints()（kπ/2 边界枚举 + 可选 GCPC cache）产生 endpoint iAABBs，不再在 Stage 1 调用 analytical root-finding 产生 per-link AABBs
- 影响范围：
  - API：EndpointIAABBResult 结构体缩减（删除 2 个成员）
  - 行为：Analytical endpoint source 的 per-link tightening 不再在 Stage 1 执行
  - 测试：4 套测试全部通过（test_ifk_pipeline 162, test_analytical 39, test_crit_gcpc 369, test_envelope_pipeline 333）
- 备注：
  - analytical root-finding 的 per-link tightening 能力未丢失，derive_aabb_critical_analytical() 仍可在 Stage 2 或独立调用中使用
  - 此变更使 endpoint_source.cpp 完全不依赖 sublink/n_sub 概念，符合 spec §6 的 Stage 1 职责划分

### 2026-03-24 — exp_link_iaabb_bench：Link iAABB 方法耗时 & 体积对比实验及几何可视化

- 修改文件 / 模块：
  - `experiments/exp_link_iaabb_bench.cpp` — 新增 C++ 实验（~400 行，含 `export_method_iaabbs_json()`）
  - `scripts/viz_link_iaabb_bench.py` — 新增 Python 可视化脚本（~520 行，含 `plot_link_iaabbs_per_method()`）
  - `CMakeLists.txt` — experiments 节新增 `exp_link_iaabb_bench` 编译目标
- 变更内容：
  - **实验设计**：端点源固定 CritSample（Stage 1），对 5 种 Stage 2 link iAABB 方法做系统对比：LinkIAABB(sub=1/4/16)、LinkIAABB_Grid、Hull16_Grid
  - **测量指标**：Stage1 耗时（CritSample 中位数）、Stage2 耗时（各方法中位数）、总耗时、体积（EnvelopeResult.volume）；每次 5 次重复取中位数
  - **Box 分布**：100 trial，narrow(5-15%) / medium(20-40%) / wide(50-100%) 三类宽度，deterministic 种子
  - **几何导出**（`export_method_iaabbs_json()`）：取代表性 viz_box，运行全部 5 方法，将各 link 的 iAABB 几何以 JSON 导出：
    - AABB 类方法（LinkIAABB_s1/s4/s16、LinkIAABB_Grid）：导出每个 box 的 `{link, seg, lo[3], hi[3]}`
    - Hull16_Grid：遍历 `hull_grid.bricks()` 位图，导出体素中心坐标 `{type:"voxel", centres:[...]}`
    - 写入 `method_iaabbs.json`
  - **可视化输出**（`results/exp_link_iaabb_bench_<timestamp>/`）：
    - `results.csv` — 500 行全量数据（100 trial × 5 方法）
    - `comparison.json` — 代表性配置的 5 方法 viz JSON（含体素中心坐标，via sbf4_viz）
    - `method_iaabbs.json` — 各方法 link iAABB 几何数据（~18MB）
    - `timing_boxplot.html` — Stage2 耗时箱线图（按宽度类别分列，Plotly）
    - `volume_boxplot.html` — 体积箱线图（按宽度类别分列，Plotly）
    - `timing_vs_volume.html` — 耗时 vs 体积散点图（方法颜色 + 宽度形状）
    - `timing_breakdown.html` — Stage1/Stage2 堆积条形图
    - `link_iaabbs_per_method.html` — 各方法 link iAABB 三维 Plotly 图（按 link 着色，Toggle 按钮切换方法，~14MB）
    - `comparison_3d.html` — 3D Plotly 对比 HTML（含 JS 滑块，via sbf4_viz）
  - **初步结果**（iiwa14, 100 trial 均值）：
    - LinkIAABB(sub=1): total≈0.047ms, vol≈0.456m³
    - LinkIAABB(sub=16): total≈0.049ms, vol≈0.442m³
    - LinkIAABB_Grid: total≈0.063ms, vol≈0.356m³
    - Hull16_Grid: total≈0.587ms, vol≈0.284m³（最紧，慢 ~12×）
- 影响范围：
  - 构建：CMakeLists.txt 新增 1 个实验目标
  - 实验结果存储至 `results/exp_link_iaabb_bench_20260324/`
- 备注：
  - 构建零错误，100 trial 全部完成，6 个 HTML 全部生成
  - Python 依赖：plotly, pandas, numpy, sbf4_viz
  - 用法：`.\build\Release\exp_link_iaabb_bench.exe [n_trials] [seed] [out_dir]`
  - 可视化：`python scripts/viz_link_iaabb_bench.py <results_dir>`

### 2026-03-24 — 实现 q_0 Z_4 旋转对称性 LECT 缓存优化（Phase 1 + Phase 2）

- 修改文件 / 模块：
  - `include/sbf/core/joint_symmetry.h` — 新增：`JointSymmetryType` 枚举、`JointSymmetry` 结构体、`detect_joint_symmetries()` 函数声明
  - `src/core/joint_symmetry.cpp` — 新增：`canonicalize()`、`transform_aabb()`、`transform_all_link_aabbs()`、`transform_all_endpoint_iaabbs()`、`detect_joint_symmetries()` 实现
  - `tests/test_joint_symmetry.cpp` — 新增：9 组单元测试（检测、规范化、AABB 变换、数值验证、LECT pre_expand 一致性）
  - `include/sbf/forest/lect.h` — 新增 `symmetry_q0_` 成员、`Z4CacheEntry` 结构体、`z4_cache_` 映射表、`symmetry_q0()` 访问器
  - `src/forest/lect.cpp` — 构造函数调用 `detect_joint_symmetries()`；`compute_envelope()` 新增 Z4 缓存查找/填充逻辑；`split_leaf()` 新增扇区边界（kπ/2）优先分裂点；`pre_expand()` 管理 Z4 缓存生命周期；`pre_expand_recursive()` 新增 canonical-first 递归顺序；`snapshot()` 复制 `symmetry_q0_`
  - `CMakeLists.txt` — 新增 `src/core/joint_symmetry.cpp` 至 SBF_SOURCES；新增 `test_joint_symmetry` 测试目标
- 变更内容：
  - **Phase 1（基础设施）**：实现了关节对称性检测、q_0 区间规范化（canonicalize）、Z4 旋转 AABB 变换（4 个扇区 × 含 offset a）
  - **Phase 2（LECT 集成）**：
    - `compute_envelope` 新增 Z4 缓存机制：对 canonical sector (sector=0) 的节点计算后缓存其 link iAABBs + endpoint iAABBs；对非 canonical 节点通过 FNV-1a 哈希查找缓存，命中时直接 transform 跳过 FK 计算
    - `split_leaf` 对 dim=0 的分裂优先选择最近的 kπ/2 扇区边界（而非中点），使得子树 q_0 区间与扇区对齐，最大化缓存命中率
    - `pre_expand_recursive` 对 dim=0 分裂采用 canonical-first 递归顺序，确保 canonical 子树先于 non-canonical 展开
  - Z4 AABB 映射公式（offset = a_0）：
    - sector 1（+π/2）：x' = [−y_hi+a, −y_lo+a], y' = [x_lo−a, x_hi−a], z' = z
    - sector 2（+π）：x' = [−x_hi+2a, −x_lo+2a], y' = [−y_hi, −y_lo], z' = z
    - sector 3（+3π/2）：x' = [y_lo−a, y_hi−a], y' = [−x_hi+a, −x_lo+a], z' = z
- 影响范围：
  - API：新增 `sbf::JointSymmetry`、`sbf::detect_joint_symmetries()` 公共 API
  - 性能：pre_expand 中 Z4 等价子树跳过 FK 计算，对 Panda（q_0 ∈ [-2.90, 2.90]）约 27% 的 q_0 范围可通过 Z4 缓存复用
  - 行为语义：`split_leaf` 对 q_0 维度分裂点微调（偏向扇区边界），不影响 AABB 正确性
  - 构建：新增 1 个源文件 + 1 个测试目标
- 备注：
  - 严格遵循 `doc/PLAN_JOINT_SYMMETRY_CACHE.md` 的数学推导与实现设计
  - 所有 94 项新测试通过；已有 502 项测试（test_ifk_pipeline + test_envelope_pipeline）无回归
  - Z4 缓存仅在 `pre_expand` 期间激活，FFB 路径不受影响
  - Phase 3（Cache 持久化优化：仅存储 canonical 扇区 + 加载时恢复）为后续待办

### 2026-03-24 — LECT 关节对称性缓存优化计划

- 修改文件 / 模块：
  - `doc/PLAN_JOINT_SYMMETRY_CACHE.md` — 新增计划文档
- 变更内容：
  - 严格数学推导了 7-DOF 串联机器人各关节的 LECT iAABB 对称性
  - 证明了 q_0 仅有 Z_4 旋转对称性（canonical [0, π/2]），无反射对称
  - 证明了 q_1 ~ q_6 在 LECT 层面均无可利用的 AABB 对称性
  - 设计了通用对称性检测算法 `detect_joint_symmetries()`
  - 设计了 `JointSymmetry` 数据结构及 AABB 映射接口
  - 制定了三阶段实现路线（基础设施 → LECT 集成 → Cache 持久化优化）
- 影响范围：
  - 不涉及代码修改，纯计划文档
- 备注：
  - 纠正了先前分析中关于 q_0 可利用 [0, π/4] 的错误结论
  - 纠正了先前分析中关于 q_1 / q_2 具有对称性的错误结论
  - 与 GCPC 现有的 q_0 消除 / q_1 period-π 是不同层面的优化，互不冲突

### 2026-03-24 — 可视化子系统迁移 (viz_exporter + sbf4_viz)

- 修改文件 / 模块：
  - `include/sbf/viz/viz_exporter.h` — 新增 C++ 可视化 JSON 导出器头文件
  - `src/viz/viz_exporter.cpp` — 新增完整实现（~530 行）
  - `CMakeLists.txt` — SBF_SOURCES 添加 `src/viz/viz_exporter.cpp`
  - `python/sbf4_viz/` — 新增 Python Plotly 渲染包（10 个文件）
- 变更内容：
  - **来源**：从 v3 `sbf_viz` 完整迁移可视化子系统
  - **C++ 导出器**：`sbf::viz` 命名空间下 8 个导出函数（export_robot_json / export_envelope_json / export_envelope_from_boxes_json / export_voxel_json / export_voxel_centres_json / export_scene_json / export_snapshot_json / export_envelope_comparison_json）
  - **API 适配**：FrameStore→EndpointStore（get_endpoints / has_endpoints / n_endpoints）；Scene 迭代从 range-for 改为 `scene[i]` + `scene.n_obstacles()` 索引循环
  - **JSON key 更新**：`link_aabbs`→`link_iaabbs`，`sub_aabbs`→`link_iaabbs_sub`
  - **对比方法精简**：保留 5 种方法（full_link_iaabb / link_iaabb / voxel_hull16 / voxel_link_iaabb / voxel_full_link_iaabb），移除 v3 的 3 种 Analytical 系列（analytical_sub_aabb / voxel_analytical / voxel_hull16_analytical）
  - **Python 包**：`sbf4_viz`（含 load_data / envelope_viz / robot_viz / scene_viz / voxel_viz / combined_viz / envelope_comparison_viz / run_demo），数据类 AABB→IAABB，参数名 show_sub_aabb→show_link_iaabb / show_full_aabb→show_full_link_iaabb
  - **交互功能**：Plotly 3D 交互 HTML，方法切换按钮，体素 opacity/cube-size JS 滑块，集合差分可视化
- 影响范围：
  - API：新增 `sbf/viz/viz_exporter.h` 公开接口
  - 构建：CMakeLists.txt 新增 1 个源文件
  - Python：新增 `sbf4_viz` 包（需 plotly + numpy 依赖）
- 备注：
  - 全量编译零错误；333 tests passed, 0 failed
  - `python -m sbf4_viz <output_dir>` 一键生成全部 HTML 可视化
  - 对比页面支持层叠/独立/差分三种模式

### 2026-03-24 — 移除区间冻结机制（Freeze Depth）全量清理

- 修改文件 / 模块：
  - `include/sbf/core/interval_trig.h` — 仅保留 `classify_alpha()`；移除 FrozenJointDesc、undo_frozen_joint、apply_joint_rotation、reconstruct_cartesian_endpoints_multilevel（两个重载）、旧版兼容包装；includes 精简为 `<cmath>`
  - `include/sbf/core/config.h` — SplitOrder 枚举仅保留 ROUND_ROBIN=0、WIDEST_FIRST=2；完整移除 FreezePolicy 结构体（含所有工厂方法）
  - `include/sbf/forest/node_store.h` — 移除 freeze_depth() / set_freeze_depth() / ensure_freeze_depth_capacity() 方法和 freeze_depth_ 成员；HCACHE 版本注释更新为 HCACHE03
  - `src/forest/node_store.cpp` — 从 ensure_capacity / alloc_node / snapshot / copy_node_from / save / load 中移除 freeze_depth_ 相关代码；HCACHE 魔数从 HC02(0x32304348) 升级至 HC03(0x33304348)，版本号 2→3
  - `include/sbf/forest/lect.h` — 移除第三构造函数（FreezePolicy 参数）；移除 freeze_policy() / max_freeze_depth() / effective_freeze_depth() / recompute_link_iaabbs() 公开方法；移除 max_freeze_depth_ / frozen_descs_ / freeze_policy_ 私有成员；新增 `SplitOrder split_order_ = SplitOrder::WIDEST_FIRST` 成员；移除 init_freeze_depth() / reconstruct_and_derive_link_iaabbs() 私有辅助函数；ep_store_ 注释从"局部坐标系"更正为"笛卡尔坐标系"
  - `src/forest/lect.cpp` — compute_envelope() 重写：移除 effective_freeze_depth / set_freeze_depth / 局部坐标系清零逻辑，直接在笛卡尔坐标系计算端点 iAABB 并 derive_aabb；完整删除 reconstruct_and_derive_link_iaabbs()；split_leaf() 简化：移除 Case A（冻结维度继承 ep_store_），统一使用 Case B（增量 FK + 完整 compute_envelope）；derive_hull_grid() 简化：移除 FrozenJointDesc 构建和 reconstruct_cartesian_endpoints_multilevel 调用，ep_store_ 直接作为笛卡尔端点使用；snapshot() / load() 移除 freeze_policy_ / max_freeze_depth_ / frozen_descs_ 拷贝/初始化；find_free_box() / pre_expand_recursive() 中 freeze_policy_.split_order 替换为 split_order_；移除 freeze_policy() / max_freeze_depth() / recompute_link_iaabbs() 访问器函数
  - `tests/test_ifk_pipeline.cpp` — test_interval_trig() 仅保留 classify_alpha 测试；test_lect() 使用 2 参数构造函数，移除 freeze_policy / max_freeze_depth / effective_freeze_depth 检查；完整删除 test_lect_v3_compat()；test_refine_aabb() 使用 2 参数构造函数
  - `CMakeLists.txt` — 移除 11 个冻结实验目标 (exp_freeze_sweep, exp_freeze_per_link, exp_freeze_asymmetric, exp_freeze_sources, exp_freeze_depth_sweep, exp_freeze_hybrid, exp_freeze_width_source, exp_split_order, exp_threshold_quality, exp_cost_quality, exp_nf_sweep)
  - `experiments/` — 删除上述 11 个 .cpp 文件
  - `docs/API_REFERENCE_CN.md` — interval_trig 节精简为仅 classify_alpha；移除 FrozenJointDesc / undo_frozen_joint / apply_joint_rotation / reconstruct_cartesian_endpoints_multilevel 全部 API 文档；FreezePolicy 节替换为简化的 SplitOrder 文档；LECT 构造函数移除第三重载；NodeStore 移除 freeze_depth_ 字段、HCACHE02→HCACHE03
- 变更内容：
  - **决策背景**：经过 exp_freeze_hybrid / exp_freeze_width_source / GCPC Pareto 分析等多轮实验，确认区间冻结与 GCPC/Analytical 方法的设计目标（计算最严密包络）矛盾——冻结引入的 wrapping effect 导致包络质量显著劣化（体积膨胀 1.4–3.0×），其计算加速不足以弥补质量损失
  - **保留内容**：`classify_alpha()` 函数保留，用于 DH 参数关于 iAABB 关系的预处理分类
  - **移除范围**：全部冻结相关结构体（FreezePolicy / FrozenJointDesc）、所有冻结计算路径（effective_freeze_depth / reconstruct_and_derive_link_iaabbs / init_freeze_depth）、冻结存储字段（freeze_depth_ in NodeStore）、11 个冻结实验
  - **简化效果**：compute_envelope 从 freeze→local→reconstruct→derive 四步简化为 compute→derive 两步；split_leaf 从 Case A/B 分支简化为统一路径
- 影响范围：
  - **⚠️ 缓存不兼容**：HCACHE 格式从 HC02 升级至 HC03，旧 .hcache 文件需重建
  - API：移除 FreezePolicy 结构体、LECT 第三构造函数、freeze_policy() / max_freeze_depth() / effective_freeze_depth() / recompute_link_iaabbs() 公开方法
  - SplitOrder 枚举值保留（0 和 2），避免序列化兼容问题
  - 性能：移除冻结开销，compute_envelope 路径简化
  - 构建：移除 11 个实验目标
- 备注：
  - 全量编译零错误零警告
  - test_ifk_pipeline: 162 passed, 0 failed
  - ep_store_ 语义从"局部坐标系端点"变更为"笛卡尔坐标系端点"，不再需要重建步骤

### 2026-03-23 — Hull16_Grid 栅格化三重加速 + 自适应 delta + v3 同步

- 修改文件 / 模块：
  - `v4/include/sbf/voxel/voxel_grid.h` — fill_aabb 行批量、fill_hull16 砖块批量 + 提前退出
  - `v4/src/envelope/envelope_type.cpp` — Hull16_Grid 分支自适应 delta
  - `v3/include/sbf/voxel/voxel_grid.h` — 同步 fill_aabb 行批量 + fill_hull16 砖块批量 + 提前退出
  - `v3/src/envelope/envelope_type.cpp` — 同步 Hull16_Grid 改用 fill_hull16 + safety_pad=0 + 自适应 delta + n_sub=1 硬编码
  - `v4/docs/API_REFERENCE_CN.md` — 新增 envelope/envelope_type 和 voxel/voxel_grid API 文档节
  - `v4/paper/root.tex` — Turbo Scanline 小节新增砖块批量 + 提前退出 + 自适应 delta 描述
  - `v4/paper/root_cn.tex` — 同步中文论文
- 变更内容：
  - **fill_aabb 行批量**（v4+v3）：内层循环从逐体素 `set_cell(cx,cy,cz)` 改为逐行 `set_cell_range_x(cx0,cx1,cy,cz)`，单次 64-bit OR 填充整行。LinkIAABB_s1 加速 2.07×，LinkIAABB_s4 加速 2.91×。
  - **fill_hull16 砖块批量 + 提前退出**（v4+v3）：
    1. YZ 循环按 8×8 砖块瓦片 (bz,by) 分组
    2. 砖块级提前退出：瓦片中心膨胀半径测试 $r_{test} = r_{eff} + 3.5\sqrt{2}\delta$，未命中跳过 64 条扫描线
    3. 栈本地 `BitBrick local_bricks[128]` 累积器，每瓦片刷新一次到全局哈希映射
    4. 超宽回退（n_xb > 128）
    - Hull16_Grid 加速：narrow 1.01×, medium 1.29×, wide 1.33×（体积完全一致 50/50）
  - **自适应 delta**（v4+v3）：估算 $\sum ext_y \cdot ext_z$，若超过 MAX_SCANLINES=40000 则 $\delta = \sqrt{\sum / 40000}$。在标准 benchmark 中未触发（所有试验低于阈值）。
  - **v3 Hull16_Grid 分支全面重写**：
    - `fill_aabb()` → `fill_hull16()` per link（proximal/distal endpoint pair）
    - `safety_pad=-1.0` → `safety_pad=0.0`
    - 移除 `env_n_sub` 细分（n_sub=1 硬编码）
    - 新增自适应 delta
  - **v4 vs v3 最终加速比**：narrow 2.14×, medium 3.33×, wide 4.47×
- 影响范围：
  - 性能：fill_aabb 和 fill_hull16 均获得显著加速
  - API：无变化（优化均为内部实现改动）
  - 行为语义：体积完全一致（50/50 exact match）
  - 文档：API_REFERENCE_CN.md 新增两个模块、论文中英文均更新
- 备注：
  - v4 全量测试 333 passed, 0 failed
  - v3 全量测试 test_sbf(5/5) + test_modular_pipeline(5/5) = 10 passed, 0 failed
  - CritSample/GCPC 增量已在 v3 `frame_source.cpp` 中存在，无需同步

### 2026-03-23 — Link iAABB 管线深度调试 + 专项测试

- 修改文件 / 模块：
  - `tests/test_envelope_pipeline.cpp`
- 变更内容：
  - 新增 Section 7：link iAABB 核心管线深度测试（8 项，共 67 个 CHECK 断言）
    - 7a. `extract_link_iaabbs` 基础测试：验证 endpoint→link 转换正确包含 parent/child ∪ radius
    - 7b. `extract_link_iaabbs` ⟺ `compute_link_envelope(LinkIAABB, sub=1)` 一致性
    - 7c. `derive_aabb` vs `derive_aabb_subdivided(sub=1)` 逐链一致性验证
    - 7d. Soundness 抽样验证：200 个随机 FK 配置全部被 link iAABB 包含
    - 7e. `collide_aabb_subdiv` 有效 refinement：对角线链场景中 n=16 细分成功排除 false positive
    - 7f. `compute_link_envelope(sub=4)` 包含性：每个 sub-AABB ⊂ 对应的 sub=1 全连杆 AABB
    - 7g. `collide_aabb_subdiv` 不丢失真碰撞：在每个链 AABB 中心放障碍物均被检测到
    - 7h. `LinkIAABB_Grid` 网格光栅化正确性：link iAABBs 的中心 voxel 已被标记
  - 修复上一轮 3 个测试失败（`source` → `method` 字段名 + 细分体积测试数据修正）
- 影响范围：
  - 测试覆盖度：link iAABB 管线从 0 直接覆盖上升至 8 项专项深度测试
  - 全量测试：5 套 × {38, 370, 333, 172, 214} = 1127 passed, 0 failed
- 备注：
  - v3→v4 核心导数管线（envelope_derive / envelope_type / collision_policy）逻辑无差异
  - v4 新增 analytical fast path + IFK clamp 在 `extract_link_iaabbs` 中，已由 test_analytical 覆盖

### 2026-03-22 — H5: Phase F 增量链重算 + 预计算逆矩阵

- 修改文件 / 模块：
  - `src/envelope/gcpc.cpp` — Phase F bg 循环重构
- 变更内容：
  - **H5-A 预计算逆矩阵**：`A_mat.colPivHouseholderQr().solve(rhs)` → `A_mat.inverse() * rhs`（~600→243 muls）
  - **H5-B 增量链重算**：bg 关节分类到 4 段（prefix/mid12/mid23/suffix），循环外预计算初始链+products，循环内仅重算 seg_dirty 链段
  - q 初始化移到循环外；bg 增量移到循环顶部；`goto` → `continue`
- 影响范围：
  - Phase F: 13.44ms → 7.38ms（−45%）；GCPC 总计: 17.21ms → 10.91ms（−37%）
  - GCPC/Analytical: 1.02 → **0.64**（所有宽度 GCPC 均快于 Analytical）
- 备注：
  - 全部 580 测试通过；体积精度不变（max 0.8%）

### 2025-05-30 — H4: kπ/2 内部值缓存 + Phase E/F Analytical 一致性重构

- 修改文件 / 模块：
  - `include/sbf/envelope/gcpc.h` — 新增 `PairKpi2Config` 结构体、`pair_kpi2_configs` 字段、`precompute_pair_kpi2()` 方法声明
  - `src/envelope/gcpc.cpp` — Phase E 重写（P2.5a）、Phase F 重写（P2.5b）、新增 Phase D½ 缓存查找、新增 `precompute_pair_kpi2()` 实现（~200 行）、`enrich_with_interior_search` 调用预计算
- 变更内容：
  - **Phase D½（新阶段）**：离线预计算所有 bg 关节取 kπ/2 值的临界构型，查询时直接查表 + 区间包含检查
  - **Phase E → P2.5a**：3 点 FK+QR → `extract_2d_coefficients` + 约束代入（α,β 公式），系数提取 0 FK 调用
  - **Phase F → P2.5b**：9 点 FK → 预计算 DH 变换 `tf_j1/j2/j3_pre`，`left_pre * tf_j2 * right_pre` 计算 9 位置，`build_symbolic_poly8` 构建多项式
  - **bg 循环**：Phase E/F 仅使用 {lo, hi}（kπ/2 值由 Phase D½ 处理），bg 组合数从 ~5^k 降至 2^k
  - **`precompute_pair_kpi2()`**：枚举所有 bg 关节 kπ/2 组合（上限 64），Phase E/F 分别预计算，1e-8 去重
- 影响范围：
  - 性能：GCPC/Analytical 比值从 1.019 降至 **1.0006**（实质平衡）
  - 中宽区间（0.3–0.7）GCPC 快 2–14%；窄宽（<0.2）慢 3–15%
  - API：GcpcCache 接口不变，内部新增预计算步骤
- 备注：
  - 全部 580 测试通过（370 + 38 + 172）
  - 体积精度：104/200 精确匹配，最大相对偏差 0.8%
  - Phase F 仍占 GCPC 78%（13.4ms/17.2ms），进一步优化需减少 per-bg-combo 的 prefix/middle/suffix 重计算

### 2026-03-22 — GridStore v3→v4 迁移 + 优化（word-level z-mask / 解耦 robot / GRD3）

- 修改文件 / 模块：
  - `include/sbf/envelope/grid_store.h` — **新建**，从 v3 迁移 GridStore（per-node 32³ bitfield 占用网格，512 × uint64_t = 4096B/node）。
  - `src/envelope/grid_store.cpp` — **新建**，完整实现 derive_from_aabbs / union_grids / check_collision / persistence / mmap。
  - `CMakeLists.txt` — 添加 `src/envelope/grid_store.cpp` 到 SBF_SOURCES。
- 变更内容：
  - **v4 优化 vs v3**：
    1. **Word-level z-mask**：R=32 时每个 (x,y) 行的 32 个 z-voxel 恰好在一个 uint64_t 的半字中。
       - `derive_from_aabbs`: z-range [iz0,iz1) 写入用 `grid[w] |= z_mask(...)` 单次 OR，替代 v3 的 per-voxel bit set 内循环（最多 32 次→1 次）
       - `check_collision`:  z-range 探测用 `grid[w] & z_mask(...)` 单次 AND，替代 v3 的 per-voxel bit test 内循环（最多 32 次→1 次）
    2. **解耦 robot 元数据**：v3 GridStore 在构造时存储 `active_link_map_[32]`、`link_radii_[32]`、`base_pos_[3]`、`n_frames_` 并在 `derive_from_frames()` 内部调用 `derive_aabb_subdivided()`。v4 构造只需 `world_bounds[6]`，新 API `derive_from_aabbs(node_idx, aabbs, n_aabbs)` 接受预计算的 link iAABBs 或 sub-AABBs，调用者负责 FK/细分。减少耦合、减少存储开销。
    3. **Obstacle\* API**：`check_collision()` 使用 `const Obstacle*`（`.lo()`/`.hi()`），替代 v3 的 `obs_compact` interleaved float 格式 `[lo_x,hi_x,lo_y,hi_y,lo_z,hi_z]`。
    4. **GRD3 格式**：简化持久化 header — 仅存 magic/version/capacity/world_bounds（36B 有效），移除 v3 GRD2 header 中的 active_link_map(128B)、link_radii(128B)、base_pos(12B)、n_frames(4B)。Record 布局不变（valid:1B + pad:7B + bitfield:4096B = 4104B）。
    5. **删除 v1 (RLE) 格式支持**：v3 `load()` 兼容 GRD1 RLE 压缩格式，v4 仅支持 GRD3。移除 `grid_envelope.h` 依赖（RLE compress/decompress + IEnvelope OOP）。
  - 新增 3 个 inline helper：`grid_word(x,y)`、`grid_bit_base(y)`、`z_mask(iz0,iz1,bit_base)`
  - Mmap 操作完整迁移：create_mmap / load_mmap / flush_mmap / close_mmap / grow_mmap
  - save / save_incremental / load 完整实现
- 影响范围：
  - API: 新增 `GridStore` 类（此前无 v4 对应物）
  - 构建: CMakeLists.txt 新增一行
  - 行为语义: grid 碰撞检测和 zero-inflation union 现在可用
- 备注：
  - v3 `derive_from_frames()` 的功能由调用者在外部完成：先用 `derive_aabb_subdivided()` 获取 sub-AABBs，再传入 `derive_from_aabbs()`
  - v3 GridStore 约 250h + 700cpp = 950 行，v4 约 210h + 380cpp = 590 行（-38%），功能相同但 API 更清洁
  - 编译测试：794 passed, 0 failed (test_ifk_pipeline: 172, test_kdop: 214, test_analytical: 38, test_crit_gcpc: 370)

### 2026-03-22 — Scene / Collision 层 v3→v4 迁移 + 优化

- 修改文件 / 模块：
  - `include/sbf/scene/scene.h` — 从 v3 迁移 Scene 类。移除 `obs_compact_` / `pack_obstacles()` / `repack()` — v4 Obstacle 直接提供 `.lo()`/`.hi()`，碰撞代码使用 `Obstacle*` 而非 interleaved float[]。简单元素访问方法内联在 header 中（`n_obstacles()`, `obstacles()`, `operator[]`）。
  - `src/scene/scene.cpp` — 从 v3 迁移 Scene 实现。构造函数、`add_obstacle()`、`remove_obstacle()`、`clear()` 全功能实现。无 `pack_obstacles()` 调用。
  - `include/sbf/scene/i_collision_checker.h` — 移除 TODO 注释，添加 v4 设计说明文档（v3 check_config/check_box/check_segment → v4 仅保留 check_collision，其余为 planner-level concern）。
  - `include/sbf/scene/aabb_collision_checker.h` — 从 v3 迁移。持有 `const Scene*`（不拥有所有权），不保存 `Robot*`、不保存 `obs_compact_` shadow copy。委托 `envelope::collide_aabb()` 执行碰撞检测。
  - `src/scene/collision.cpp` — 从 v3 迁移。`AABBCollisionChecker::check_collision()` 委托 `envelope::collide_aabb()`，消除重复 SAT 实现。
- 变更内容：
  - 5 个 stub / TODO 文件替换为完整实现
  - **v4 优化 vs v3**：
    1. 消除 `obs_compact_` interleaved float 数组（v3 每次 add/remove 都重新 pack，v4 直接 `vector<Obstacle>` 迭代）
    2. `AABBCollisionChecker` 不再持有 `Robot*` — v4 collision checker 仅处理预计算 link iAABBs，解耦 FK / 机器人模型
    3. `ICollisionChecker` 接口从 3 方法简化为 1 方法 — config/box/segment 检测上移到 planner 层
    4. `AABBCollisionChecker` 委托 `envelope::collide_aabb()` — 不再重复实现 SAT，单一事实来源
- 影响范围：
  - API: `Scene` 从返回空值变为完整功能
  - API: `AABBCollisionChecker` 构造函数改为 `explicit AABBCollisionChecker(const Scene&)`
  - API: `AABBCollisionChecker::check_collision()` 从返回 false 变为实际碰撞检测
  - 构建: 无新增源文件
- 备注：
  - scene 层为 envelope 层的更高层封装：Scene 管理 Obstacle 集合，AABBCollisionChecker 封装 collide_aabb 调度
  - 编译测试：794 passed, 0 failed (test_ifk_pipeline: 172, test_kdop: 214, test_analytical: 38, test_crit_gcpc: 370)

### 2026-03-22 — GCPC 管线第二轮堆分配消除 (G1-G7)

- 修改文件 / 模块：
  - `v4/src/envelope/gcpc.cpp` — `derive_aabb_with_gcpc()` 7-phase 管线
  - `v4/doc/OPTIMIZATION_PLAN_GCPC.md` — 优化计划文档
- 变更内容：
  - **G1**: `best_face_config` 从 `vector<VectorXd>` → 定长 `FaceTracker` 结构体（`double val; double config[MAX_CFG_N=8]`），栈上分配 `FaceTracker best_face[MAX_FACE_TRACKS]`，消除每次极值更新的 VectorXd 深拷贝
  - **G2**: `checkpoint_AB` 从 `vector<VectorXd>` 深拷贝 → `memcpy(checkpoint_AB, best_face, ...)`，零开销
  - **G3**: `robot.coupled_pairs()` 从 Phase D/E/F/G 4 次重复构建 → 入口处一次性预计算 3 种 pair 列表（`face_pairs_d[64]`, `pc_pairs_ef[64]`, `all_pairs_g[64]`），栈数组
  - **G4**: Phase E/F `bg_vals` 从 `vector<vector<double>>` → 入口处预计算 `bg_vals_flat[16][16]` + `bg_vals_n[16]`，E/F 共享
  - **G5**: Phase E/F 内循环 `other_bg`/`other_idx` 从 `vector<int>` + `vector<vector<double>>` → 栈数组（`other_idx_e[MAX_OTHER_E]`, `other_bg_n_e[MAX_OTHER_E]`, `other_use_full_e[MAX_OTHER_E]`），消除 pair×kk×ci 内循环堆分配
  - **G6**: Phase G `starts` 从 `vector<VectorXd>` → `starts_buf[MAX_SEEDS_G]`；`link_pairs` 从 `vector<pair>` → 栈数组 `link_pairs[64]`
  - **G7**: Phase G per-joint 3×3 trig 矩阵 + QR 分解从 sweep×joint 内循环重复计算 → 入口处预计算 `qr_per_joint[16]`，内循环引用
- 影响范围：
  - 性能：消除 GCPC 管线中几乎所有热路径堆分配（VectorXd、vector<int>、vector<vector<double>>、vector<pair<int,int>>）
  - API：无变化
  - 行为语义：数值结果完全一致
- 备注：
  - 编译测试：580 passed, 0 failed (test_crit_gcpc: 370, test_ifk_pipeline: 172, test_analytical: 38)
  - G8（`endpoint_iaabbs` 栈数组）延后，影响所有 epiAABB 管线，风险较高

### 2026-03-22 — Link iAABB 管线 v3→v4 迁移（bit_brick / voxel_grid / hull_rasteriser / envelope_type / collision_policy）

- 修改文件 / 模块：
  - **P0 Voxel 基础层**（1:1 迁移，几乎原样复制）：
    - `include/sbf/voxel/bit_brick.h` — 从 v3 迁移完整 BitBrick (8×uint64, 512 voxels, 64 bytes)、BrickCoord、BrickCoordHash (FNV-1a)。替换原 v4 stub（仅有声明无实现）。
    - `include/sbf/voxel/voxel_grid.h` — 从 v3 迁移 FlatBrickMap（开放寻址 hash map，70% 最大负载因子）+ VoxelGrid（fill_aabb, fill_hull16 turbo scanline, merge, collides, intersect_inplace, count_occupied, occupied_volume, num_bricks, reserve_from_bounds, set_cell_range_x）。~500 行 header-only 实现。替换原 v4 stub（仅有方法声明）。
  - **P1 Envelope Type（Stage 2 入口）**：
    - `include/sbf/envelope/envelope_type.h` — 移除 `voxel::VoxelGrid` forward declaration，改为 `#include "sbf/voxel/voxel_grid.h"`。在 `EnvelopeResult` 中添加 `voxel::VoxelGrid hull_grid;` 值类型字段（替换原 TODO 注释）。
    - `src/envelope/envelope_type.cpp` — 从 v3 迁移 `compute_link_envelope()` 三分支实现：LinkIAABB（sub=1 用 `extract_link_iaabbs`，sub>1 用 `derive_aabb_subdivided`）、LinkIAABB_Grid（byte grid 光栅化）、Hull16_Grid（VoxelGrid::fill_aabb 光栅化）。`default_envelope_config()` 完整实现（IFK/CritSample→n_sub=1, Analytical/GCPC→n_sub=16）。v4 重命名：SubAABB→LinkIAABB, sub_aabbs→link_iaabbs, EndpointAABBResult→EndpointIAABBResult, endpoint_aabbs→endpoint_iaabbs。
  - **P2 Hull Rasteriser**：
    - `include/sbf/voxel/hull_rasteriser.h` — 从 v3 迁移 PosInterval、frame_pos()、rasterise_robot_hull16/sub_aabbs/aabbs()、rasterise_box_obstacle() 声明。
    - `src/voxel/hull_rasteriser.cpp` — 从 v3 迁移完整实现（~130 行）。rasterise_robot_aabbs 调用 v4 的 extract_link_aabbs (interval_fk.h)。
  - **P3 Collision Policy**：
    - `include/sbf/envelope/collision_policy.h` — v4 简化 API：`check_collision(CollisionPolicy, float* link_iaabbs, int n_active, Obstacle*, int n_obs)`。新增 `collide_aabb()` 和 `check_collision_aabb_legacy()` 独立入口。
    - `src/envelope/collision_policy.cpp` — 实现 collide_aabb（link iAABB vs Obstacle.lo()/hi() 三轴 SAT）、check_collision_aabb_legacy（兼容 interleaved obs_compact 格式）、统一 dispatch。AABB_SUBDIV/GRID 暂 fallback 到 AABB（需要 raw frames 扩展 API 后实现）。
- 变更内容：
  - 7 个 stub 文件全部替换为从 v3 迁移的完整实现
  - link iAABB 管线三阶段全链路打通：EndpointSource → EnvelopeType → CollisionPolicy
  - VoxelGrid 支持 fill_aabb + fill_hull16 turbo scanline（Conv(B₁∪B₂)⊕Ball）
  - FlatBrickMap 替代 std::unordered_map（~2-3× 查找速度）
- 影响范围：
  - API: `compute_link_envelope()` 从返回空结果变为完整实现
  - API: `check_collision()` 从返回 false 变为实际碰撞检测
  - 构建: 无新增源文件（CMakeLists.txt 已包含所有文件）
  - 行为语义: 森林生长器现在可以使用 AABB 碰撞策略进行正确的碰撞检测
- 备注：
  - v4 collision_policy API 与 v3 不同：使用 `Obstacle*`(center+half_sizes) 而非 `float* obs_compact`(interleaved)，更类型安全
  - AABB_SUBDIV 和 GRID 碰撞策略暂 fallback 到 AABB，因 v4 简化 API 仅传入预计算 link_iaabbs，不含 raw frames
  - scene 层的 AABBCollisionChecker 仍为 stub，独立于 envelope 层的 collision_policy
  - 编译测试：794 passed, 0 failed (test_ifk_pipeline: 172, test_kdop: 214, test_analytical: 38, test_crit_gcpc: 370)

### 2026-03-22 — epiAABB 四管线优化：冗余消除 + 预计算 DH + 栈分配 + 去重优化

- 修改文件 / 模块：
  - `v4/src/envelope/endpoint_source.cpp`:
    - **O1**: CritSample / GCPC / Analytical 三条路径移除冗余 `fk_to_endpoints()` 调用。`derive_crit_endpoints()` 会将 `endpoint_iaabbs` 初始化为 ±1e30f 并重新填充，之前的 `fk_to_endpoints()` 结果被完全丢弃，属于浪费。改为仅 `resize` 不填充。保留 `compute_fk_full()` 以供 `extract_link_iaabbs()` 的 IFK clamp 使用。
  - `v4/src/envelope/crit_sample.cpp`:
    - **O4**: `crit_angles()` 返回值改为栈数组 `double cset_v[16][16]` + `int cset_n[16]`，消除 Phase 2 中 7 次 `vector<double>` 和 `vector<vector<double>>` 堆分配。
    - **O6**: `pre_tf` 从 `vector<vector<Matrix4d>>` 改为扁平栈数组 `Matrix4d pre_tf_flat[256]` + `int pre_tf_off[16]`，消除二维 vector 间接寻址和堆分配。
    - **O5**: Phase 1 去重从 `std::set<std::vector<int>>` 改为 `std::unordered_set<std::array<int,16>>` + 自定义哈希 `DedupKeyHash`（FNV-1a 风格），消除去重时的 `vector<int>` 堆分配和红黑树节点分配。
  - `v4/src/envelope/gcpc.cpp`:
    - **O2**: Phase B（Boundary kπ/2 enumeration）完全重写：
      - `std::function<void(int)> enumerate` 递归 → **栈式迭代 DFS**（`FrameB dfs_stack[16]`），消除虚分派开销
      - **预计算 DH 变换**：`pre_tf_flat[MAX_TOTAL_TF_B]` + `pre_tf_off[]`，热路径内无 sin/cos
      - **预计算 suffix DH**：非枚举关节（`n_joints_needed..n-1`）使用 midpoint 角度预算 DH 矩阵，`finalize_chain_b()` 仅做矩阵乘
      - 栈分配 critical angle 集（同 O4），消除 `crit_angles_fn` 返回 `vector` 的堆分配
      - 将 `eval_config` 拆分为 `update_from_ws`（仅更新 AABB/face config）+ `eval_config`（ws.compute + update），Phase B 的 DFS 路径直接调用 `update_from_ws` 避免重复 FK
    - 大乘积 fallback 路径（corners + random）也改用预计算 DH，`ws.tf` 逐关节累积
  - `v4/doc/OPTIMIZATION_PLAN_EPIAABB.md` — 新建优化计划文档，记录 7 项优先级排序的优化项
- 变更内容：
  - **O1: 跳过冗余 fk_to_endpoints**：非 IFK 管线避免被覆写的无用计算
  - **O2: GCPC Phase B 栈 DFS + 预计算 DH**：消除 `std::function` 虚分派、sin/cos 热循环、堆分配
  - **O4: crit_angles 栈数组**：CritSample Phase 2 零堆分配
  - **O5: Phase 1 定长哈希去重**：`unordered_set<array<int,16>>` 替代 `set<vector<int>>`
  - **O6: pre_tf 扁平化**：消除二维 vector 间接寻址
  - **性能变化** (200 trials, IIWA14, seed 42):
    | Source | 优化前 avg | 优化后 avg | 优化后 median | VolRatio |
    |---|---|---|---|---|
    | IFK | 0.022ms | 0.023ms | 0.022ms | 5.64 |
    | CritSample | 0.039ms | 0.040ms | 0.033ms | 0.999 |
    | **GCPC** | **0.192ms** | **0.160ms** | **0.042ms** | **1.000** |
    | Analytical | 16.04ms | 16.76ms | 12.90ms | 1.000 |
    - GCPC 平均 -17%，**中位数 0.042ms**（窄/中区间下几乎与 CritSample 持平）
    - Volume 完全不变，正确性验证通过（GCPC VolRatio=1.0000）
    - CritSample 堆分配开销占比极小，栈化改动主要为代码质量优化
- 影响范围：
  - API：无公开 API 变更
  - 性能：GCPC Phase B 显著加速；CritSample 微优化
  - 缓存：无影响
  - 行为语义：数值结果完全不变（580 tests 全部通过）
- 备注：
  - O3（endpoint_iaabbs vector→stack array）和 O7（LinkExtremes VectorXd→定长数组）影响面较大但收益有限，暂不实施
  - 所有改动均通过 580 tests（38 + 172 + 370）

### 2026-03-22 — CritSample 独立化 + 预计算 DH 变换优化（5x faster than GCPC）

- 修改文件 / 模块：
  - `v4/src/envelope/endpoint_source.cpp` — CritSample case 默认使用 `boundary_only()` 配置（不再传播 GCPC cache），与 GCPC 路径明确区分
  - `v4/src/envelope/crit_sample.cpp`:
    - Phase 2 改用**预计算 DH 变换**：在枚举循环前为每个 (joint, angle) 对预算所有 DH 4×4 矩阵，热路径内仅做矩阵乘法（无 sin/cos）
    - 预计算 tool 变换（常量帧，每次查询只算一次）
    - 枚举改用**显式栈式迭代遍历**替代递归 `crit_enum_fk`，消除函数调用开销
    - `update_endpoint_iaabb` 内联优化：直接列访问 `ws.tf[fi].col(3)`、`__restrict` 提示、early break
  - `v4/include/sbf/envelope/analytical_utils.h`:
    - `crit_enum` 和 `crit_enum_fk` 改为**模板化回调** `template<typename Fn>`，消除 `std::function` 虚拟分发开销
    - 移除 `#include <functional>`
  - `v4/experiments/exp_v3v4_epiAABB.cpp` — CritSample 配置简化（无 gcpc_cache、无 crit_config_ptr）
  - `v4/docs/API_REFERENCE_CN.md` — 更新 CritSample 设计描述（boundary_only 默认、预计算 DH、与 GCPC 区别说明）；`crit_enum`/`crit_enum_fk` 签名更新为模板版
  - `v4/paper/root.tex` — CritSample 小节由 "GCPC-Based" 改为 "Boundary Critical Sampling"；更新摘要、贡献列表、实验描述
  - `v4/paper/root_cn.tex` — CritSample 小节由 "基于 GCPC 的临界采样" 改为 "边界临界采样"；同步更新摘要、贡献列表、实验描述
- 变更内容：
  - **CritSample 独立化**：CritSample 不再依赖 GCPC 缓存，默认使用 `boundary_only()` 配置（仅 Phase 2 边界 kπ/2 枚举），可独立运行。GCPC 源 = Phase 1（缓存查找）+ Phase 2（边界枚举），两者明确区分。
  - **预计算 DH 变换**：Phase 2 热循环（~500 次迭代）内不再调用 sin/cos（每次 `compute_joint` 要做 2 次 sin/cos）。改为预先计算所有 (joint, angle) 的 4×4 DH 变换矩阵，热路径内仅做矩阵乘法。
  - **模板化枚举回调**：`crit_enum_fk<Fn>` 替代 `crit_enum_fk(std::function<void()>)`，编译器可内联回调 lambda，避免虚拟分发。
  - **性能提升** (200 trials, IIWA14, endpoint iAABB 体积):
    | Source | Time (ms) | VolRatio vs Analytical | Speedup vs Analytical |
    |---|---|---|---|
    | IFK | 0.022 | 8.54 | 736x |
    | **CritSample** | **0.039** | **0.9943** | **415x** |
    | GCPC | 0.192 | 1.0000 | 84x |
    | Analytical | 16.04 | 1.0000 | 1x |
    - CritSample 比 GCPC 快 **5x**（0.039ms vs 0.192ms）
    - CritSample 质量 99.4%（endpoint iAABB 体积仅差 0.6%）
    - 按宽度：narrow 0.030ms (175x)，medium 0.036ms (446x)，wide 0.061ms (613x)
- 影响范围：
  - API：`crit_enum` / `crit_enum_fk` 签名变更（`std::function` → `template<typename Fn>`），调用方无需修改（lambda 自动推导）
  - 行为语义：CritSample 默认不使用 GCPC 缓存（可通过 `crit_config_ptr` 手动启用）
  - 性能：CritSample 0.062ms → 0.039ms（1.6x）；GCPC 间接受益于模板化
  - 论文：CritSample 小节重写（EN/CN），摘要 + 贡献列表 + 实验描述更新
- 备注：
  - 测试：580 测试全通过（test_analytical 38 + test_ifk_pipeline 172 + test_crit_gcpc 370）
  - CritSample 在 narrow 区间达到 VolRatio=1.0000（kπ/2 完全覆盖所有极值），medium 0.9998，wide 0.9922
  - GCPC 的优势在宽区间更明显（更多内部极值不在 kπ/2 上），但整体 CritSample 已足够紧密

### 2026-03-22 — 修正 Stage 1 职责：纯 endpoint iAABB + Analytical 基准对比

- 修改文件 / 模块：
  - `v4/include/sbf/envelope/crit_sample.h` — 移除 `float* out_link_iaabb` 参数，恢复原始签名
  - `v4/src/envelope/crit_sample.cpp` — 删除 `update_link_iaabb()` 辅助函数，Phase 1/2 不再更新 per-link body iAABB
  - `v4/include/sbf/envelope/endpoint_source.h` — 删除 `gcpc_link_aabbs` 字段和 `has_gcpc_link_aabbs()` 方法
  - `v4/src/envelope/endpoint_source.cpp`:
    - CritSample/GCPC case 不再分配 gcpc_link_aabbs（Stage 1 只生成 endpoint iAABB）
    - Analytical case 新增 `derive_crit_endpoints` 调用以收紧 endpoint iAABBs（与 GCPC/CritSample 同路径）
    - `extract_link_iaabbs()` IFK clamp 改用 `fk_state` 计算 IFK 外界（不再依赖可能被覆写的 `endpoint_iaabbs`）
    - 删除 `gcpc_link_aabbs` 分支，只保留 `analytical_link_aabbs` 直接路径
  - `v4/tests/test_crit_gcpc.cpp` — 适配 `derive_crit_endpoints` 函数签名变更
  - `v4/experiments/exp_v3v4_epiAABB.cpp` — volume 计算改为 endpoint iAABB 体积之和；Analytical config 设置 gcpc_cache
  - `v4/results/exp_v3v4_epiAABB/analyze_results.py` — 新增表 3/4 以 Analytical 为基准的 v4 内部对比
- 变更内容：
  - **设计修正**：Stage 1 的唯一输出是 endpoint iAABBs，不应在此阶段生成 per-link body iAABBs。
    此前 GCPC/CritSample 通过 `update_link_iaabb()` 从 FK 采样生成 per-link body iAABBs（内界近似，非 sound 外界），
    这在概念上违反了 Stage 1/Stage 2 分层：Stage 1 = endpoint iAABBs，Stage 2 = link envelopes。
  - **移除 `gcpc_link_aabbs`**：从 `EndpointIAABBResult` 结构体和所有使用处删除。
    Analytical 的 `analytical_link_aabbs` 保留（root-finding 生成的 sound 外界，可安全用于 Stage 2 直接路径）。
  - **Analytical endpoint 收紧**：Analytical case 运行完解算器后，额外调用 `derive_crit_endpoints`
    （GCPC cache + boundary kπ/2）收紧 endpoint iAABBs，确保与 GCPC/CritSample 在 endpoint 层面严格一致。
  - **IFK clamp 修正**：`extract_link_iaabbs()` 的 IFK clamp 改为从 `fk_state` 重新计算 IFK 端点外界，
    避免使用被 `derive_crit_endpoints` 覆写的 `endpoint_iaabbs`（后者为 FK-sampled 内界，不适合作为 IFK 外界参照）。
  - **实验结果** (50 trials, seed=42, IIWA14, endpoint iAABB 体积):
    | Source | Avg Vol | VolRatio (vs Analytical) | Time(ms) | Speedup vs Analytical |
    |---|---|---|---|---|
    | IFK | 2.196e+00 | **8.0259** | 0.021 | 761x |
    | CritSample | 2.736e-01 | **1.0000** | 0.184 | 89x |
    | Analytical | 2.736e-01 | **1.0000** (基准) | 16.27 | 1.00x |
    | GCPC | 2.736e-01 | **1.0000** | 0.184 | 89x |
- 影响范围：
  - API：`derive_crit_endpoints` 签名变更（5→4 参数，移除 `out_link_iaabb`）；`EndpointIAABBResult` 删除 `gcpc_link_aabbs`
  - 行为语义：Analytical 的 `endpoint_iaabbs` 现在是 FK-sampled 内界（与 GCPC/CritSample 一致），不再是 IFK 外界
  - 性能：Analytical 额外 ~0.2ms（derive_crit_endpoints），可忽略
- 备注：
  - GCPC/CritSample/Analytical 在 endpoint iAABB 层面严格一致（VolRatio = 1.0000），因为使用相同的 derive_crit_endpoints（GCPC cache + boundary kπ/2）
  - IFK 区间算术外界是 Analytical 的 8.03 倍（验证了 FK 采样的收紧效果）
  - 按宽度分解：narrow 3.5x, medium 5.0x, wide 9.4x（宽区间时 IFK 松弛更严重）
  - 测试：test_analytical 38P / test_ifk_pipeline 172P / test_crit_gcpc 370P（共 580P，全通过）

### 2026-03-22 — GCPC 104x 加速：derive_crit_endpoints 统一 CritSample/GCPC

- 修改文件 / 模块：
  - `v4/include/sbf/envelope/crit_sample.h` — `derive_crit_endpoints` 新增 `float* out_link_iaabb` 参数
  - `v4/src/envelope/crit_sample.cpp` — 新增 `update_link_iaabb()` 辅助函数；Phase 1/2 同时更新 endpoint iAABB 和 per-link body iAABB
  - `v4/src/envelope/endpoint_source.cpp` — CritSample/GCPC case 均调用 `derive_crit_endpoints` 产生 `gcpc_link_aabbs`；GCPC case 不再调用完整 Analytical solver
  - `v4/tests/test_crit_gcpc.cpp` — 适配新函数签名（插入 `nullptr` 占位 `out_link_iaabb`）
- 变更内容：
  - **GCPC 核心重构**：此前 GCPC case 先运行完整 Analytical solver（~15.8ms）再叠加 cache seed，
    现改为仅调用 `derive_crit_endpoints`（Phase 1: GCPC cache lookup → FK → per-link iAABBs；Phase 2: boundary kπ/2 enumeration → FK → per-link iAABBs），跳过昂贵的 Phase 1-3 根查找。
  - **CritSample per-link iAABB**：`derive_crit_endpoints` 新增 per-link body iAABB 输出，
    通过 `update_link_iaabb()` 在每次 FK 后从 FKWorkspace 提取 proximal+distal 位置 ± link radius。
    CritSample case 分配 `gcpc_link_aabbs` 并传入 `derive_crit_endpoints`，
    使 CritSample 也走直接 per-link AABB 路径（与 Analytical/GCPC 统一）。
  - **GCPC 与 CritSample 统一**：两者均使用 `derive_crit_endpoints`，区别仅在于 GCPC 必须有 cache pointer，
    CritSample 可有可无（无 cache 时退化为 boundary-only）。
  - **v3/v4 对比结果** (50 trials, seed=42, IIWA14):
    | Source | v3 Time | v4 Time | Speedup | VolRatio |
    |---|---|---|---|---|
    | IFK | 0.022ms | 0.022ms | 1.00x | 1.0000 |
    | CritSample | 0.056ms | 0.201ms | 0.28x | 1.0074 |
    | Analytical | 25.21ms | 15.85ms | 1.59x | 1.0000 |
    | **GCPC** | **20.16ms** | **0.19ms** | **104.52x** | 0.9639 |
    - GCPC: 104.52x 加速（20.16ms → 0.19ms），体积 96.4% of v3（3.6% tighter，因跳过 Analytical 相关极值点）
    - CritSample: 因增加 per-link iAABB 更新导致略慢（0.056→0.201ms），但仍远快于 Analytical
    - Analytical: 保持 1.59x 加速，体积完全一致
- 影响范围：
  - API：`derive_crit_endpoints` 签名变更（新增第 5 参数 `float* out_link_iaabb = nullptr`）
  - 行为语义：GCPC 不再运行 Analytical solver，仅使用 cache + boundary kπ/2
  - 性能：GCPC 从 ~20ms 降至 ~0.19ms（104x 加速）
- 备注：
  - CritSample/GCPC 体积比 Analytical 略 tight（0.9639），因 sampling 无法覆盖所有 analytical 极值点；
    IFK clamp 保护仍然有效（per-link AABB 不会宽于 IFK）
  - 按宽度分解：narrow 204x, medium 227x, wide 58x（窄区间加速最显著，因 cache 命中密度高）
  - 测试：test_analytical 38P / test_ifk_pipeline 172P / test_crit_gcpc 370P（共 580P，全通过）

### 2026-03-22 — 实现 GCPC 管线路径 + v3/v4 全管线对比验证

- 修改文件 / 模块：
  - `v4/include/sbf/envelope/endpoint_source.h` — `EndpointIAABBResult` 新增 `gcpc_link_aabbs` 字段
  - `v4/src/envelope/endpoint_source.cpp` — GCPC case 调用 `derive_aabb_with_gcpc()`；`extract_link_iaabbs()` 统一 Analytical/GCPC 直接 per-link AABB 路径
  - `v4/results/exp_v3v4_epiAABB/` — 全部结果重跑 (50 trials × 4 sources)
- 变更内容：
  - **GCPC 管线路径**：此前 `EndpointSource::GCPC` case 为 TODO fallback 到 IFK，实际未调用 GCPC 解算器。
    现补全：compute_endpoint_iaabb() 调用 `gcpc_cache->derive_aabb_with_gcpc(robot, intervals, 1, ...)` 生成 per-link body AABB，
    存入 `result.gcpc_link_aabbs`；extract_link_iaabbs() 检测 `has_gcpc_link_aabbs()` 时直接使用，并与 IFK 取 componentwise intersection（IFK clamp 保护）。
  - **统一直接 AABB 路径**：extract_link_iaabbs() 重构为先检测 `analytical_link_aabbs`，再检测 `gcpc_link_aabbs`，
    两者共享相同的 IFK clamp 逻辑，消除代码重复。
  - **v3/v4 对比验证结果** (50 trials, seed=42, IIWA14):
    - IFK: v3=v4 体积完全一致 (VolRatio=1.0000)
    - CritSample: v4/v3 VolRatio=1.0126（v4 GCPC 两阶段设计，略有差异属正常）
    - Analytical: v4/v3 VolRatio=1.0000，v4 加速 1.58x（DH 直接系数提取）
    - GCPC: v4/v3 VolRatio=1.0000，耗时一致（1.00x），n_fk_calls 统计方式不同（v4 报 total FK 而非 cache match 数）
- 影响范围：
  - API：`EndpointIAABBResult` 新增 `gcpc_link_aabbs` 字段 + `has_gcpc_link_aabbs()` 方法
  - 行为语义：GCPC 管线路径从 IFK fallback 变为真正调用 GCPC 解算器
- 备注：
  - v3 实验通过直接调用解算器绕过管线（compute_endpoint_aabb → extract_link_aabbs 的 endpoint merge 在 v3 中有已知 union 操作问题），v4 通过 `gcpc_link_aabbs` 直接 per-link AABB 路径解决了此问题
  - 测试：test_analytical 38P / test_ifk_pipeline 172P / test_crit_gcpc 370P（共 580P，全通过）

### 2026-03-22 — 确认 GCPC 完整迁移（v3 → v4）

- 修改文件 / 模块：
  - `v4/src/envelope/gcpc.cpp` (2023 行) — 已从 v3 `gcpc_cache.cpp` (2329 行) 完整迁移
  - `v4/include/sbf/envelope/gcpc.h` (258 行) — 已从 v3 `gcpc_cache.h` 完整迁移
- 变更内容：
  - **验证完毕**：逐函数对比 v3 gcpc_cache.cpp 与 v4 gcpc.cpp，确认全部 15 个成员函数均已迁移：
    - `reconstruct_q0_xy`, `q0_in_range`, `build_kdtree`, `build_kdtree_recursive`,
      `kdtree_range_query`, `query_link`, `derive_aabb_with_gcpc` (Phases A-G 完整),
      `n_total_points`, `find_section`, `build`, `enrich_with_interior_search`,
      `load_json`, `save`, `load`, `get_vandermonde_qr`
  - **重构差异**（v4 vs v3，均为正确迁移）：
    - 7 个局部工具函数（`HALF_PI`, `kShifts`, `FixedRoots`, `solve_poly_in_interval`,
      `half_angle_to_q`, `build_bg_values`, `GcpcFKWorkspace`）已提取到共享头文件 `analytical_utils.h`
    - 参数更名：`out_endpoint_aabb` → `out_endpoint_iaabb`（v4 iAABB 命名规范）
    - 工作空间类型：`GcpcFKWorkspace` → `FKWorkspace`（跨模块共享）
    - 头文件更名：`gcpc_cache.h` → `gcpc.h`
  - **行数差异**：v4 比 v3 少 306 行，完全对应提取到 `analytical_utils.h` 的工具代码
- 影响范围：
  - 无代码变更，仅验证确认
- 备注：
  - 测试：test_analytical 38P / test_ifk_pipeline 172P / test_crit_gcpc 370P（共 580P，全通过）
  - 迁移计划参考：`doc/MIGRATION_ENDPOINT_IAABB.md` Phase F

### 2026-03-22 — 修复 P3 直接系数提取路径的 FKWorkspace stale tf bug

- 修改文件 / 模块：
  - `v4/src/envelope/analytical_solve.cpp` — `solve_interior_improved()` 前缀构建后增加 `ws.compute_from()`
- 变更内容：
  - **根因**：P3 coordinate-descent 的 pair-coupling 代码调用 `eval_and_update_from(robot, q, partner, ...)` 时，
    若 `partner > j` 且当前 j 迭代未产生候选点，则 `ws.tf[partner]` 处于未初始化/陈旧状态。
    QR 路径的 3 点 FK 采样会设置 `tf[j+1..n+1]`，因此不受影响；
    直接系数提取路径跳过了 FK 采样，导致 `tf[partner]` 为垃圾值，产生错误的 FK 位置，
    使 AABB 体积异常膨胀（部分 trial 可达 4-5× 偏大）。
  - **修复**：在前缀构建 `ws.set_identity(); for(k<j) ws.compute_joint(robot,q,k);` 之后，
    追加 `ws.compute_from(robot, q, j);` 确保 `tf[j+1..n+1]` 和 `np` 始终有效。
    代价为每次 j 迭代额外 O(n−j) 次矩阵乘法，对 6-DOF 机械臂影响可忽略。
  - **清理**：移除临时诊断宏 `SBF_DIAG_P3_COEFF` 及其在 CMakeLists.txt 中的编译定义。
- 影响范围：
  - 行为语义：v4 Analytical 体积现与 v3 一致（差异 < 1e-5）
  - 性能：P3 每 joint 迭代增加 1 次 compute_from 调用，总体影响 < 1%
- 备注：
  - 诊断方法：逐阶段消融测试 → 确认 P3 直接系数为唯一差异源 → 系数交叉校验通过 →
    发现诊断代码（含 compute_from 调用）掩盖了 bug → 定位到 pair-coupling 读取未初始化 tf
  - 验证：diag_analytical_vol 11 trials 全部匹配；test_analytical 38P / test_ifk_pipeline 172P / test_crit_gcpc 370P

### 2026-03-21 — 修复 extract_link_iaabbs Analytical 越界 + 更新实验

- 修改文件 / 模块：
  - `v4/src/envelope/endpoint_source.cpp` — `extract_link_iaabbs()` 增加 IFK clamp 保护
  - `v4/results/exp_v3v4_epiAABB/` — 全部结果重跑 (50 trials)
  - `v4/results/exp_v3v4_epiAABB/analyze_results.py` — 更新根因分析说明
- 变更内容：
  - **Bug**: Analytical 解算器的临界点根搜索偶尔越出 C-space box 边界，
    导致 ~12% 的 trial 中 analytical link AABB 宽于 IFK 外边界
    (如 trial 3 narrow: Analytical=0.185 >>> IFK=0.046, 4 倍)
  - **Fix**: `extract_link_iaabbs()` 在 `has_analytical_link_aabbs()` 快速路径中，
    增加与 IFK 派生 link AABB 的逐分量 clamp:
      - `final_lo = max(analytical_lo, IFK_lo)`
      - `final_hi = min(analytical_hi, IFK_hi)`
    确保输出永不超过 IFK 外边界，同时保留 Analytical 的收紧效果
  - 重跑 50-trial 实验，修复后 Analytical 平均体积从 0.675 m³ 降至 0.418 m³
    (IFK 的 16.6%，修复前为 26.9%)
  - 0/50 trials Analytical > IFK (修复前 6/50)
- 影响范围：
  - 核心库 (endpoint_source.cpp) / 实验结果
- 备注：
  - 所有 580 条既有测试通过 (test_analytical 38, test_ifk_pipeline 172, test_crit_gcpc 370)
  - 修复后数据: Analytical 1.67x speedup (15.2ms vs 25.4ms), vol ratio=0.166
  - Narrow boxes: Analytical vol=0.064 < IFK vol=0.111 (修复前 0.117 > 0.111!)

### 2026-03-21 — v3 vs v4 Endpoint iAABB 跨版本基准实验

- 修改文件 / 模块：
  - `v4/experiments/exp_v3v4_epiAABB.cpp` — 新建 v4 侧实验（~324 行）
  - `v3/experiments/exp_v3v4_epiAABB.cpp` — 新建 v3 侧实验（~270 行）
  - `v4/CMakeLists.txt` — 注册 `exp_v3v4_epiAABB` 目标
  - `v3/CMakeLists.txt` — 注册 `exp_v3v4_epiAABB` 目标
  - `v4/results/exp_v3v4_epiAABB/` — 实验结果与分析
    - `v4_results.csv` / `v3_results.csv` / `merged_results.csv`
    - `analyze_results.py` — Python 分析脚本，生成 3 张对比表 + 根因分析
- 变更内容：
  - 对比 IIWA14 机器人在 4 种 EndpointSource（IFK / CritSample / Analytical / GCPC）
    下的 link iAABB 总体积与耗时 (50 trials, seed=42)
  - Box 宽度分布: 40% narrow (0.05/0.10/0.15), 40% medium (0.20/0.30/0.40),
    20% wide (0.50/0.70/1.00)
  - 每条 trial 重复 3 次取 median 消除抖动
  - 发现 v3 Analytical/GCPC 体积 ≡ IFK 的根因：
    `merge_analytical_to_endpoints()` 使用 min(lo)/max(hi) 但初始值为外边界 IFK,
    因此 analytical 更紧的值被 IFK 恒等覆盖。v4 通过 `analytical_link_aabbs`
    独立字段解决此问题
- 影响范围：
  - 实验 / 性能基准
- 备注：
  - v4 GCPC 管线 Phase D-F 尚未迁移, 当前回退 IFK (体积 = IFK, 耗时 ~0.02ms)
  - v4 Analytical: 1.67x speedup (15.2ms vs 25.4ms), 体积缩减至 IFK 的 16.6%
    (修复 IFK clamp 后更新)
  - v4 CritSample: 体积接近 (ratio=1.01), v3 稍快 (0.06ms vs 0.13ms)

### 2026-03-21 — 创建 LaTeX 中英文论文 (paper/)

- 修改文件 / 模块：
  - `paper/root.tex` — 新建，英文 T-RO 期刊论文（~720 行）
  - `paper/root_cn.tex` — 新建，中文 T-RO 期刊论文（~580 行，xelatex + xeCJK）
  - `paper/references.bib` — 新建，BibTeX 参考文献（15 条）
- 变更内容：
  - 参考 v3/paper/ 结构，根据 v4 实现创建完整中英文论文
  - 英文版 (root.tex)：IEEEtran journal 格式，pdflatex 编译
  - 中文版 (root_cn.tex)：IEEEtran journal 格式，xelatex 编译，SimSun/SimHei/FangSong 字体
  - v4 新增内容（相对 v3）：
    - §III DH 链直接系数提取子节（零 FK 优化，37% 加速）
    - §III CritSample 改写为基于 GCPC 的两阶段设计
    - §III GCPC 扩展为七阶段查询流水线（含 AA 剪枝）
    - 全文使用 `\iAABB` 命令（v3 为 `\AABB`）
    - 表 1 使用 v4 命名（LinkIAABB, LIAABB_Gr, Hull16）
    - 贡献列表从 5 项扩展为 6 项
  - references.bib 与 v3 相同（15 条参考文献）
- 影响范围：
  - 文档：新增 paper/ 目录，符合 AI_WORKING_SPEC.md 同步规则
- 备注：
  - 编译指令：英文 `pdflatex root.tex`，中文 `xelatex root_cn.tex`
  - 两版共享 references.bib

---

### 2026-03-21 — GCPC 模块从 v3 迁移至 v4

- 修改文件 / 模块：
  - `include/sbf/envelope/gcpc.h` — 从 33 行 stub 替换为完整 ~260 行 v4 适配头文件
  - `src/envelope/gcpc.cpp` — 从 21 行 stub 替换为完整 ~2360 行实现（迁移自 v3 gcpc_cache.cpp 2684 行）
  - `include/sbf/envelope/analytical_utils.h` — 为 FKWorkspace 添加 `compute_prefix()` 方法
- 变更内容：
  - 完整迁移 v3 GCPC 缓存模块（Phase F of MIGRATION_ENDPOINT_IAABB.md）
  - gcpc.h：迁移全部数据结构（GcpcPoint, KdNode, GcpcLinkSection, GcpcQueryResult, GcpcQueryStats）及 GcpcCache 类声明
  - gcpc.cpp：迁移全部功能——KD-tree 构建/查询、q₀ 重建、q₁ 反射、derive_aabb_with_gcpc 七阶段管线（A-G）、缓存构建/富化、JSON 加载、二进制 save/load
  - v3→v4 适配：
    - 去除重复工具函数（FixedRoots, solve_poly_in_interval, half_angle_to_q, build_bg_values），改用 analytical_utils.h 共享版本
    - 去除 GcpcFKWorkspace（匿名命名空间），改用 analytical_utils.h 的 FKWorkspace（新增 compute_prefix）
    - 保留 GCPC 专用的 Vandermonde QR (get_vandermonde_qr)
    - include 路径：`sbf/common/types.h` → `sbf/core/types.h`，`gcpc_cache.h` → `gcpc.h`
    - 输出参数：`out_endpoint_aabb` → `out_endpoint_iaabb`
- 影响范围：
  - API：GcpcCache 类完整可用（之前为空 stub），可用于 endpoint_source 管线
  - 构建：sbf4 目标编译通过，test_analytical (38 pass)、test_ifk_pipeline (172 pass) 全部通过
- 备注：
  - 迁移来源：v3/src/envelope/gcpc_cache.cpp + v3/include/sbf/envelope/gcpc_cache.h
  - FKWorkspace::compute_prefix 为新增公共方法，供 GCPC Phase C-G 部分 FK 前缀计算使用

---

### 2026-03-20 — O5.7 补充执行（run_06..10）：N=5 第二组重复验证

- 修改文件 / 模块：
  - `doc/PLAN_ANALYTICAL_OPTIMIZED.md`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_baseline_run_06..10.log`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_baseline_run_06..10.err.log`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_trial_run_06..10.log`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_trial_run_06..10.err.log`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_compare_run_06..10.csv`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_repeat_summary_02.csv`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_repeat_recommendation_02.txt`
- 变更内容：
  - 在 O5.7 既有 N=5（run_01..05）基础上，按相同参数完成第二组 N=5 重复（run_06..10），并生成每 run compare 与第二组 repeat 汇总/建议。
  - 将补充执行结果回填到 `PLAN_ANALYTICAL_OPTIMIZED.md` 的 O5.7 执行段。
- 影响范围：
  - 实验（新增 O5.7 第二组重复工件）
  - 路线决策（进一步确认骨架阶段结论稳定）
  - 行为语义（仍为骨架阶段，默认 `default-off` 不变）
- 备注：
  - 第二组关键汇总（5 runs，`B_dedup_adaptive_proto`）：
    - `avg_mean_ms_delta_pct_mean = -0.338%`, `std = 0.932%`
    - `avg_mean_fk_p2_delta_pct_mean = +0.000%`, `std = 0.000%`
    - `avg_mean_fk_total_delta_pct_mean = +0.000%`, `std = 0.000%`
    - `avg_trial_p2_hybrid_eval/applied/fallback_cases_mean = 0.000`
  - 结论：`go_O5_5_2_N10`（与 O5.7 首组结论一致）。

### 2026-03-20 — O5.6 重执行（run_02）：骨架 smoke 复核

- 修改文件 / 模块：
  - `doc/PLAN_ANALYTICAL_OPTIMIZED.md`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_baseline_02.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_baseline_02.err.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_trial_02.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_trial_02.err.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_compare_02.csv`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_recommendation_02.txt`
- 变更内容：
  - 按 O5.6 原参数集完成 baseline/trial 的 run_02 重执行，并生成 compare/recommendation。
  - 将 run_02 结果回填到计划文档 O5.6 执行段。
- 影响范围：
  - 实验（新增 run_02 工件用于复核）
  - 路线决策（结论与 run_01 保持一致）
  - 行为语义（仍为骨架阶段，默认路径不变）
- 备注：
  - run_02 汇总（`B_dedup_adaptive_proto`）：
    - `avg_mean_ms_delta_pct = -0.863%`
    - `avg_mean_fk_p2_delta_pct = +0.000%`
    - `avg_mean_fk_total_delta_pct = +0.000%`
    - `avg_trial_p2_hybrid_eval/applied/fallback_cases = 0.000 / 0.000 / 0.000`
  - 结论：`go_O5_5_2_repeat_for_scaffold_validation`（与 run_01 一致）。

### 2026-03-20 — 文档同步：补充 O5.7 计划摘要（确认版）

- 修改文件 / 模块：
  - `doc/PLAN_ANALYTICAL_OPTIMIZED.md`
  - `doc/CHANGE_LOG_CN.md`
- 变更内容：
  - 在 O5.7 章节下新增“计划摘要（确认版）”，统一与 O5.6 的呈现方式。
  - 摘要明确了目标、固定参数、A/B 设计、必需产物和验收门槛，便于快速检索与执行对齐。
- 影响范围：
  - 文档可读性与执行一致性提升。
  - 不影响 API、实验结果和运行语义。
- 备注：
  - 本次为文档整理同步，不新增实验工件。

### 2026-03-20 — O5.7：O5.5-2 骨架阶段 N=5 重复验证

- 修改文件 / 模块：
  - `doc/PLAN_ANALYTICAL_OPTIMIZED.md`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_baseline_run_01..05.log`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_baseline_run_01..05.err.log`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_trial_run_01..05.log`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_trial_run_01..05.err.log`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_compare_run_01..05.csv`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_repeat_summary.csv`
  - `results/exp_analytical_ab_repeat/o57_runs/o57_hybrid_repeat_recommendation.txt`
- 变更内容：
  - 按 O5.7 计划完成 O5.5-2 骨架阶段 N=5 baseline/trial 重复（高分支固定 profile）。
  - 生成每 run compare 与 repeat summary/recommendation 工件。
  - 回填计划文档 O5.7 执行结果与下一步决策。
- 影响范围：
  - 实验（新增 O5.7 重复统计工件）
  - 路线决策（确认可进入 N=10 稳定性扩展）
  - 行为语义（仍为骨架阶段，默认 `default-off` 不变）
- 备注：
  - 关键汇总（5 runs，`B_dedup_adaptive_proto`）：
    - `avg_mean_ms_delta_pct_mean = -0.606%`, `std = 3.232%`
    - `avg_mean_fk_p2_delta_pct_mean = +0.000%`, `std = 0.000%`
    - `avg_mean_fk_total_delta_pct_mean = +0.000%`, `std = 0.000%`
    - `avg_trial_p2_hybrid_eval/applied/fallback_cases_mean = 0.000`
  - 结论：`go_O5_5_2_N10`（进入 O5.5-2 的 N=10 稳定性扩展）。

### 2026-03-20 — O5.6：O5.5 受控切换骨架落地 + 首轮 smoke

- 修改文件 / 模块：
  - `doc/PLAN_ANALYTICAL_OPTIMIZED.md`
  - `include/sbf/envelope/analytical_solve.h`
  - `experiments/exp_analytical_ab_benchmark.cpp`
  - `docs/API_REFERENCE_CN.md`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_baseline_01.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_baseline_01.err.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_trial_01.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_trial_01.err.log`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_compare_01.csv`
  - `results/exp_analytical_ab_repeat/o56_runs/o56_hybrid_recommendation_01.txt`
- 变更内容：
  - 在 `AnalyticalCriticalConfig` 新增 O5.5 骨架配置字段：
    - `enable_p2_derivative_hybrid`
    - `enable_p2_hybrid_fallback`
    - `p2_hybrid_max_abs_error`
  - 在 `AnalyticalCriticalStats` 新增 O5.5 骨架统计字段：
    - `n_p2_hybrid_eval_cases`
    - `n_p2_hybrid_applied_cases`
    - `n_p2_hybrid_fallback_cases`
  - benchmark 新增环境变量与 CSV 回显列：
    - `SBF_P2_DERIVATIVE_HYBRID`
    - `SBF_P2_HYBRID_FALLBACK`
    - `SBF_P2_HYBRID_MAX_ABS_ERR`
    - `enable_hybrid` / `enable_hybrid_fallback` / `hybrid_max_abs_err`
    - `p2_hybrid_eval_cases` / `p2_hybrid_applied_cases` / `p2_hybrid_fallback_cases`
  - 完成 O5.6 baseline/trial 首轮 smoke，并生成 compare/recommendation 工件。
- 影响范围：
  - API（配置与统计字段扩展）
  - 实验（新增 hybrid 骨架入口与 CSV 列）
  - 行为语义（默认路径保持不变，仍为 `default-off`）
- 备注：
  - 首轮 smoke（`B_dedup_adaptive_proto` 4 个 width 平均）：
    - `avg_mean_ms_delta_pct = +0.092%`
    - `avg_mean_fk_p2_delta_pct = +0.000%`
    - `avg_mean_fk_total_delta_pct = +0.000%`
    - `avg_trial_p2_hybrid_eval_cases = 0.000`
  - 结论：`go_O5_5_2_repeat_for_scaffold_validation`（进入 N=5 重复验证骨架稳定性）。

### 2026-03-20 — O5.4h：高分支门控参数 10 次重复统计与稳定性结论

- 修改文件 / 模块：
  - `doc/PLAN_ANALYTICAL_OPTIMIZED.md`
  - `results/exp_analytical_ab_repeat/o54h_runs/high_branch_gate_baseline_run_01..10.log`
  - `results/exp_analytical_ab_repeat/o54h_runs/high_branch_gate_baseline_run_01..10.err.log`
  - `results/exp_analytical_ab_repeat/o54h_runs/high_branch_gate_trial_run_01..10.log`
  - `results/exp_analytical_ab_repeat/o54h_runs/high_branch_gate_trial_run_01..10.err.log`
  - `results/exp_analytical_ab_repeat/o54h_runs/high_branch_gate_compare_run_01..10.csv`
  - `results/exp_analytical_ab_repeat/o54h_runs/high_branch_gate_repeat_summary.csv`
  - `results/exp_analytical_ab_repeat/o54h_runs/high_branch_gate_repeat_recommendation.txt`
- 变更内容：
  - 按 O5.4h 计划完成高分支 profile（`SBF_BENCH_WIDTHS=0.35,0.70,1.00,1.40`）下 10 次 baseline/trial 重复运行。
  - 固定门控参数 `min_roots=2`, `min_width=0.70`，对 `B_dedup_adaptive_proto` 口径生成每 run 对照与 10-run 汇总（mean/std）。
  - 产出 repeat summary 与 recommendation，并回填计划文档执行结果段。
- 影响范围：
  - 实验（新增 O5.4h 重复统计工件）
  - 路线决策（确认门控参数版本在当前口径下仍不宜默认开启）
  - 行为语义（默认路径不变，继续保留开关试点）
- 备注：
  - 关键汇总（10 runs，`B_dedup_adaptive_proto`）：
    - `avg_mean_ms_delta_pct_mean = -0.532%`, `std = 2.489%`
    - `avg_mean_fk_p2_delta_pct_mean = +0.277%`, `std = 0.146%`
    - `avg_mean_fk_total_delta_pct_mean = +0.007%`, `std = 0.004%`
  - 决策：`NO-GO(default-off)`；耗时均值虽有小幅下降，但 FK 额外开销趋势仍在，暂不支持默认开启。

### 2026-03-20 — O5.4g：`multi-qi` 条件门控（root/width）与首轮高分支 smoke

- 修改文件 / 模块：
  - `doc/PLAN_ANALYTICAL_OPTIMIZED.md`
  - `include/sbf/envelope/analytical_solve.h`
  - `src/envelope/analytical_solve.cpp`
  - `experiments/exp_analytical_ab_benchmark.cpp`
  - `docs/API_REFERENCE_CN.md`
  - `results/exp_analytical_ab_repeat/o54g_runs/high_branch_gate_baseline_01.log`
  - `results/exp_analytical_ab_repeat/o54g_runs/high_branch_gate_baseline_01.err.log`
  - `results/exp_analytical_ab_repeat/o54g_runs/high_branch_gate_trial_01.log`
  - `results/exp_analytical_ab_repeat/o54g_runs/high_branch_gate_trial_01.err.log`
  - `results/exp_analytical_ab_repeat/o54g_runs/high_branch_gate_compare_01.csv`
  - `results/exp_analytical_ab_repeat/o54g_runs/high_branch_gate_recommendation_01.txt`
- 变更内容：
  - 在 `AnalyticalCriticalConfig` 新增 O5.4g 门控参数：
    - `p2_multi_qi_min_root_count`（默认 `1`）
    - `p2_multi_qi_min_joint_width`（默认 `0.0`）
  - 在 `solve_faces()` 中将 `multi-qi` 启用条件从全局开关扩展为 case 级门控：
    - `enable_p2_multi_qi_candidates`
    - `tj_roots.size() >= p2_multi_qi_min_root_count`
    - `max(width_i,width_j) >= p2_multi_qi_min_joint_width`
  - benchmark 新增 env 与 CSV 回显列：
    - `SBF_P2_MULTI_QI_MIN_ROOTS`
    - `SBF_P2_MULTI_QI_MIN_WIDTH`
    - `multi_qi_min_roots`
    - `multi_qi_min_width`
- 影响范围：
  - API（新增配置字段）
  - 实验（新增门控参数入口与 CSV 回显）
  - 行为语义（默认路径不变，保留可回退开关）
- 备注：
  - 高分支 smoke（`SBF_BENCH_WIDTHS=0.35,0.70,1.00,1.40`，trial 使用 `min_roots=2,min_width=0.70`）汇总：
    - `avg_mean_ms_delta_pct = -2.030%`
    - `avg_mean_fk_p2_delta_pct = +0.346%`
    - `avg_mean_fk_total_delta_pct = +0.009%`
  - 决策：`NO-GO(default-off)`；虽有耗时下降，但 FK 工作量仍有轻微上升，暂不默认开启。

### 2026-03-20 — O5.4f：高分支 10 次重复统计与最终建议

- 修改文件 / 模块：
  - `doc/PLAN_ANALYTICAL_OPTIMIZED.md`
  - `results/exp_analytical_ab_repeat/o54f_runs/high_branch_baseline_run_01..10.log`
  - `results/exp_analytical_ab_repeat/o54f_runs/high_branch_baseline_run_01..10.err.log`
  - `results/exp_analytical_ab_repeat/o54f_runs/high_branch_trial_run_01..10.log`
  - `results/exp_analytical_ab_repeat/o54f_runs/high_branch_trial_run_01..10.err.log`
  - `results/exp_analytical_ab_repeat/o54f_runs/high_branch_compare_run_01..10.csv`
  - `results/exp_analytical_ab_repeat/o54f_runs/high_branch_repeat_summary.csv`
  - `results/exp_analytical_ab_repeat/o54f_runs/high_branch_repeat_recommendation.txt`
- 变更内容：
  - 按 O5.4f 计划完成高分支 profile（`SBF_BENCH_WIDTHS=0.35,0.70,1.00,1.40`）下 10 次 baseline/trial 重复运行。
  - 产出每 run 对照 CSV 与 10-run 汇总统计（mean/std）。
  - 形成高分支口径最终建议文件并回填计划文档。
- 影响范围：
  - 实验（新增重复统计工件与最终建议）
  - 路线决策（确认 `multi-qi` 在高分支场景仍不宜默认开启）
  - 行为语义（默认路径不变，仅保留开关）
- 备注：
  - 关键汇总（10 runs，`B_dedup_adaptive_proto`）：
    - `avg_mean_ms_delta_pct_mean = -0.802%`, `std = 3.665%`
    - `avg_mean_fk_p2_delta_pct_mean = +1.131%`, `std = 0.211%`
    - `avg_mean_fk_total_delta_pct_mean = +0.029%`, `std = 0.006%`
  - 决策：`NO-GO(default-off)`；P2/FK 额外开销趋势稳定，继续仅作开关试点。

### 2026-03-20 — O5.4e：高分支定向样本入口 + multi-qi 首轮高分支 smoke

- 修改文件 / 模块：
  - `doc/PLAN_ANALYTICAL_OPTIMIZED.md`
  - `experiments/exp_analytical_ab_benchmark.cpp`
  - `docs/API_REFERENCE_CN.md`
  - `results/exp_analytical_ab_repeat/o54e_runs/high_branch_baseline_01.log`
  - `results/exp_analytical_ab_repeat/o54e_runs/high_branch_baseline_01.err.log`
  - `results/exp_analytical_ab_repeat/o54e_runs/high_branch_trial_01.log`
  - `results/exp_analytical_ab_repeat/o54e_runs/high_branch_trial_01.err.log`
  - `results/exp_analytical_ab_repeat/o54e_runs/high_branch_compare_01.csv`
  - `results/exp_analytical_ab_repeat/o54e_runs/high_branch_recommendation_01.txt`
- 变更内容：
  - benchmark 新增 `SBF_BENCH_WIDTHS` 环境变量，支持注入自定义 width 列表构造高分支定向样本。
  - CSV 新增 `bench_width_profile` 回显列（`default` / `env_custom`）。
  - 完成 O5.4e 首轮高分支 A/B smoke（`SBF_BENCH_WIDTHS=0.35,0.70,1.00,1.40`）。
- 影响范围：
  - 实验（新增高分支样本入口与 CSV 列）
  - API 文档（新增环境变量与回显列说明）
  - 行为语义（默认行为保持不变，未设置 env 时仍用默认 width 集）
- 备注：
  - 自定义 profile 生效：`bench_width_profile=env_custom`。
  - 首轮结论（`B_dedup_adaptive_proto` 4 个 width 平均）：
    - `avg_mean_ms_delta_pct = +0.489%`
    - `avg_mean_fk_p2_delta_pct = +1.220%`
    - `avg_mean_fk_total_delta_pct = +0.032%`
  - 决策：`NO-GO(default-off)`；高分支样本下 `multi-qi` 仍表现为额外开销上升，暂不进入默认开启。

### 2026-03-20 — O5.4d：P2 多 `qi` 分支真实候选生成（可回退）与首轮 A/B 结论

- 修改文件 / 模块：
  - `doc/PLAN_ANALYTICAL_OPTIMIZED.md`
  - `include/sbf/envelope/analytical_solve.h`
  - `src/envelope/analytical_solve.cpp`
  - `experiments/exp_analytical_ab_benchmark.cpp`
  - `docs/API_REFERENCE_CN.md`
  - `results/exp_analytical_ab_repeat/o54d_runs/multi_qi_baseline_01.log`
  - `results/exp_analytical_ab_repeat/o54d_runs/multi_qi_baseline_01.err.log`
  - `results/exp_analytical_ab_repeat/o54d_runs/multi_qi_trial_01.log`
  - `results/exp_analytical_ab_repeat/o54d_runs/multi_qi_trial_01.err.log`
  - `results/exp_analytical_ab_repeat/o54d_runs/multi_qi_compare_01.csv`
  - `results/exp_analytical_ab_repeat/o54d_runs/multi_qi_recommendation_01.txt`
- 变更内容：
  - 在计划文档新增 O5.4d 并回填执行结果。
  - 新增配置开关：`enable_p2_multi_qi_candidates`（默认关闭）。
  - 在 P2 `solve_faces()` 中接入“每个 root 收集全部落区间 `qi` 分支”的真实候选生成路径；关闭开关时保持原先“首个可行分支即 break”。
  - benchmark 新增环境变量：`SBF_P2_MULTI_QI_CAND`，并输出 `enable_multi_qi_cand` 列用于 A/B 追踪。
  - 生成 baseline/trial 首轮对照汇总与建议工件。
- 影响范围：
  - API（新增配置开关）
  - 实验（新增环境变量、CSV 列与对照工件）
  - 行为语义（默认关闭，不改变现有默认路径）
- 备注：
  - 首轮 smoke（`B_dedup_adaptive_proto` 5 个 width 平均）
    - `avg_mean_ms_delta_pct = -0.094%`
    - `avg_mean_fk_p2_delta_pct = +0.345%`
    - `avg_mean_fk_total_delta_pct = +0.009%`
  - 结论：`NO-GO(default-off)`；保留开关能力，后续仅在高分支场景继续试点。

### 2026-03-20 — O5.4c 首步：shadow all-branch 候选统计接入（compare-only）

- 修改文件 / 模块：
  - `doc/PLAN_ANALYTICAL_OPTIMIZED.md`
  - `include/sbf/envelope/analytical_solve.h`
  - `src/envelope/analytical_solve.cpp`
  - `experiments/exp_analytical_ab_benchmark.cpp`
  - `docs/API_REFERENCE_CN.md`
  - `results/exp_analytical_ab_repeat/o54c_runs/shadow_all_branch_run_01.log`
  - `results/exp_analytical_ab_repeat/o54c_runs/shadow_all_branch_run_01.err.log`
  - `results/exp_analytical_ab_repeat/o54c_runs/shadow_all_branch_recommendation.txt`
- 变更内容：
  - 在计划文档新增 O5.4c（all-branch shadow 统计）执行计划。
  - 新增配置开关：`enable_p2_shadow_all_qi_branches`（默认关闭）。
  - 在 P2 shadow compare-only 统计中新增扩展口径字段：
    - `n_p2_shadow_candidate_count_all_branches`
  - 统计逻辑：对每个 `tj_root` 枚举 `qi_cand / qi_cand±pi` 与 `kShifts` 的全部落区间分支，仅用于计数，不参与主路径候选求解与写回。
  - benchmark 新增环境变量：`SBF_P2_SHADOW_ALL_QI`，并输出 `enable_shadow_all_qi` 与 `p2_shadow_candidate_count_all_branches` 列。
- 影响范围：
  - API（配置与统计字段扩展）
  - 实验（CSV 新增 all-branch 口径）
  - 行为语义（compare-only，不改变 envelope 主写回路径）
- 备注：
  - 本次为 O5.4c 第一步（统计扩展）；后续将基于首轮结果决定是否进入 O5.4d 的真实候选生成策略优化。
  - 首轮 smoke run（`SBF_P2_SHADOW_RUN=1`, `SBF_P2_SHADOW_ALL_QI=1`）结果：
    - `sum_eval=2133`
    - `sum_candidate=19`（`candidate_per_eval=0.008908`）
    - `sum_candidate_all_branches=23`（`candidate_per_eval_all_branches=0.010783`）
    - `uplift_pct=21.05%`
  - 结论：`go_O5_4d_candidate_generation_strategy`（保持 compare-only，继续优化真实候选生成策略）。

### 2026-03-20 — O5.4b：shadow run 10 次重复汇总与 O5.5 入口建议

- 修改文件 / 模块：
  - `doc/PLAN_ANALYTICAL_OPTIMIZED.md`
  - `results/exp_analytical_ab_repeat/o54b_runs/shadow_run_01.log` ~ `shadow_run_10.log`
  - `results/exp_analytical_ab_repeat/o54b_runs/shadow_run_01.err.log` ~ `shadow_run_10.err.log`
  - `results/exp_analytical_ab_repeat/o54b_runs/shadow_repeat_summary.csv`
  - `results/exp_analytical_ab_repeat/o54b_runs/shadow_recommendation.txt`
- 变更内容：
  - 按 O5.3c 推荐阈值（`3e-17 / 1e-13 / 1e-11`）完成 O5.4b 的 10 次 shadow run 重复实验。
  - 对采集中出现的空日志样本执行补跑修复，最终形成完整 10 组有效样本。
  - 产出 `mode × width` 维度汇总（mean/std）：
    - `mean_ms_mean/std`
    - `gate_accept_mean/std`
    - `shadow_eval_mean/std`
    - `shadow_coverage_pct_mean`
    - `shadow_root_per_eval_mean`
    - `shadow_candidate_per_eval_mean`
  - 产出 O5.5 入口建议文件并落盘决策字段。
- 影响范围：
  - 实验（新增 O5.4b 重复统计与建议工件）
  - 调参/路线决策（形成“继续 O5.4 深化”结论）
  - 行为语义（compare-only，不改变 envelope 主写回路径）
- 备注：
  - `avg_shadow_coverage_pct=100.00`，`avg_shadow_candidate_per_eval=0.0075`。
  - 当前建议：`stay_O5_4_deepen_low_candidate_scale`，暂不进入 O5.5 写回切换。

### 2026-03-20 — O5.4a 首步：shadow run 统计骨架接入（compare-only）

- 修改文件 / 模块：
  - `doc/PLAN_ANALYTICAL_OPTIMIZED.md`
  - `include/sbf/envelope/analytical_solve.h`
  - `src/envelope/analytical_solve.cpp`
  - `experiments/exp_analytical_ab_benchmark.cpp`
  - `docs/API_REFERENCE_CN.md`
  - `results/exp_analytical_ab_repeat/shadow_run_01.log`
  - `results/exp_analytical_ab_repeat/shadow_run_01.err.log`
- 变更内容：
  - 在计划文档新增 O5.4a 执行项（shadow 统计骨架 + 首轮基线采样）。
  - 新增配置开关：`enable_p2_derivative_shadow_run`（默认关闭）。
  - 在 P2 compare-only 路径中新增 shadow 统计字段：
    - `n_p2_shadow_eval_cases`
    - `n_p2_shadow_root_count`
    - `n_p2_shadow_candidate_count`
  - benchmark 新增环境变量：`SBF_P2_SHADOW_RUN`，并输出 shadow 统计列。
- 影响范围：
  - API（新增配置与统计字段）
  - 实验（CSV 新增 shadow 统计列）
  - 行为语义（仍为 compare-only，不改变 envelope 主写回路径）
- 备注：
  - 已使用 O5.3c 推荐阈值（`3e-17 / 1e-13 / 1e-11`）执行 `SBF_P2_SHADOW_RUN=1` 首轮基线。
  - 首轮结果中 shadow 计数已非零（例如 width=0.35：`p2_shadow_eval_cases=593`, `p2_shadow_root_count=18`, `p2_shadow_candidate_count=2`）。

### 2026-03-20 — O5.3c：三档阈值 10 次重复统计与推荐阈值落地

- 修改文件 / 模块：
  - `doc/PLAN_ANALYTICAL_OPTIMIZED.md`
  - `results/exp_analytical_ab_repeat/o53c_runs/gate_scan_loose_run_*.log`
  - `results/exp_analytical_ab_repeat/o53c_runs/gate_scan_medium_run_*.log`
  - `results/exp_analytical_ab_repeat/o53c_runs/gate_scan_strict_run_*.log`
  - `results/exp_analytical_ab_repeat/o53c_runs/gate_scan_repeat_summary.csv`
  - `results/exp_analytical_ab_repeat/o53c_runs/gate_scan_recommendation.txt`
- 变更内容：
  - 按 O5.3c 计划完成三档阈值（`loose/medium/strict`）各 10 次重复运行，共 30 次。
  - 对运行中出现的空/截断日志做补跑修复，最终形成完整 10×3 样本集。
  - 产出汇总统计文件（按 `profile × mode × width`）：
    - `gate_accept_ratio_mean/std`
    - `reject_fit_mean/std`
    - `reject_symbolic_mean/std`
    - `reject_singular_mean/std`
  - 产出推荐阈值文件，当前推荐进入 O5.4 的初值：
    - `fit=3e-17`
    - `sym_rms=1e-13`
    - `sym_max=1e-11`
- 影响范围：
  - 实验（新增 O5.3c 重复统计工件）
  - 调参（形成可复现阈值推荐）
  - 行为语义（compare-only，不改变主求解路径）
- 备注：
  - 从结果看：`loose` 接受率过高（区分度不足），`strict` 接受率过低（覆盖不足），`medium` 为当前折中起点，建议用于 O5.4 shadow run 首轮试点。

### 2026-03-20 — O5.3b：benchmark 接入 gate 阈值扫描入口并完成首轮对照

- 修改文件 / 模块：
  - `doc/PLAN_ANALYTICAL_OPTIMIZED.md`
  - `experiments/exp_analytical_ab_benchmark.cpp`
  - `results/exp_analytical_ab_repeat/gate_scan_default.log`
  - `results/exp_analytical_ab_repeat/gate_scan_default.err.log`
  - `results/exp_analytical_ab_repeat/gate_scan_strict.log`
  - `results/exp_analytical_ab_repeat/gate_scan_strict.err.log`
  - `results/exp_analytical_ab_repeat/gate_scan_compare.csv`
- 变更内容：
  - 在计划文档追加“下一轮进一步优化计划（O5.3b）”，明确阈值扫描与 O5.4 衔接里程碑。
  - benchmark 新增 gate 阈值环境变量读取：
    - `SBF_P2_GATE_FIT_RMS`
    - `SBF_P2_GATE_SYMBOLIC_RMS`
    - `SBF_P2_GATE_SYMBOLIC_MAX_ABS`
  - CSV 输出新增阈值回显列：
    - `gate_fit_rms_thresh`
    - `gate_symbolic_rms_thresh`
    - `gate_symbolic_max_abs_thresh`
  - 完成两组首轮对照：
    - default（`1e-9, 1e-8, 1e-6`）：`gate_accept_ratio=100%`
    - strict（`1e-18, 1e-14, 1e-12`）：`gate_accept_ratio` 显著下降（约 `7%~9%`）
  - 生成 `gate_scan_compare.csv`，统一汇总 default/strict 两组的 accept 与 reject 分布。
- 影响范围：
  - 实验（支持无代码改动的阈值扫描）
  - 性能评估（可直接观察 reject 分布与阈值敏感性）
  - 行为语义（compare-only，求解路径不变）
- 备注：
  - 下一步可按 10~20 次重复统计输出推荐阈值区间，并进入 O5.4 shadow run。

### 2026-03-20 — O5.3 构建回归与首轮 gate 结果产出

- 修改文件 / 模块：
  - `results/exp_analytical_ab_repeat/gate_run_02.log`
  - `results/exp_analytical_ab_repeat/gate_run_02.err.log`
- 变更内容：
  - 使用 VS2022 `MSBuild.exe` 重新编译 `exp_analytical_ab_benchmark` 与 `test_analytical`（Release|x64）。
  - 回归执行 `test_analytical.exe`，结果 `PASS: 24 / FAIL: 0`。
  - 运行 benchmark 生成 O5.3 首轮 gate 统计日志，CSV 头已包含：
    - `p2_proto_gate_eval`
    - `p2_proto_gate_accept`
    - `p2_proto_gate_accept_ratio`
    - `p2_proto_gate_reject_fit`
    - `p2_proto_gate_reject_symbolic`
    - `p2_proto_gate_reject_singular`
- 影响范围：
  - 构建/验证（Release 二进制确认包含 O5.3 实现）
  - 实验（新增 gate 统计产物）
  - 行为语义（compare-only，无求解路径变更）
- 备注：
  - 本轮 `gate_accept_ratio` 为 `100%`，reject 计数均为 `0`；可在后续 O5.3 重复实验中进一步校准阈值敏感性。

### 2026-03-20 — O5.3 原型：P2 门控决策统计（compare-only）

- 修改文件 / 模块：
  - `doc/PLAN_ANALYTICAL_OPTIMIZED.md`
  - `include/sbf/envelope/analytical_solve.h`
  - `src/envelope/analytical_solve.cpp`
  - `experiments/exp_analytical_ab_benchmark.cpp`
  - `docs/API_REFERENCE_CN.md`
- 变更内容：
  - 在优化计划文档新增“下一阶段执行计划（O5.3→O5.5）”，明确门控统计、shadow run 与受控切换里程碑。
  - 实现 O5.3 compare-only 门控统计，不改变主求解路径（仍走采样+QR）：
    - 新增门控阈值配置：`p2_proto_gate_fit_rms_thresh`、`p2_proto_gate_symbolic_rms_thresh`、`p2_proto_gate_symbolic_max_abs_thresh`。
    - 新增统计字段：
      - `n_p2_proto_gate_eval`
      - `n_p2_proto_gate_accept`
      - `n_p2_proto_gate_reject_fit`
      - `n_p2_proto_gate_reject_symbolic`
      - `n_p2_proto_gate_reject_singular`
  - benchmark CSV 扩展 O5.3 列，并新增 `p2_proto_gate_accept_ratio`（百分比）。
- 影响范围：
  - API（配置与统计字段扩展）
  - 实验（CSV 口径扩展）
  - 行为语义（compare-only，输出 AABB 语义不变）
- 备注：
  - 为 O5.4 shadow run 与 O5.5 受控切换提供阈值调参与可替代覆盖率基线。

### 2026-03-20 — O5 prototype 重复实验汇总（12 次）与基线对比

- 修改文件 / 模块：
  - `results/exp_analytical_ab_repeat/proto_run_01.log` ~ `proto_run_12.log`
  - `results/exp_analytical_ab_repeat/proto_run_01.err.log` ~ `proto_run_12.err.log`
  - `results/exp_analytical_ab_repeat/proto_all_runs_rows.csv`
  - `results/exp_analytical_ab_repeat/proto_summary_stats.csv`
  - `results/exp_analytical_ab_repeat/proto_vs_baseline_summary.csv`
- 变更内容：
  - 完成 O5 prototype 口径下 A/B benchmark 12 次重复运行，并产出可追溯原始日志与聚合结果。
  - 汇总 `proto_summary_stats.csv`（按 width）：
    - speedup_mean（A/B）分别为：
      - width=0.10: `1.0366 ± 0.0416`
      - width=0.20: `1.0080 ± 0.0182`
      - width=0.35: `0.9659 ± 0.0719`
      - width=0.50: `0.9827 ± 0.0593`
      - width=0.70: `0.9955 ± 0.0137`
    - `fk_reduction_mean_pct` 与此前口径保持一致（约 `0.026% ~ 0.168%`）。
  - 生成 `proto_vs_baseline_summary.csv`，完成与既有 `summary_stats.csv` 的逐宽度对比。
  - 记录 prototype 关键稳定性指标（B 模式）：
    - `p2_proto_symbolic_coeff_rms_mean` 在 `1e-12 ~ 1e-15` 量级；
    - `p2_proto_symbolic_coeff_max_abs_mean` 在 `1e-10 ~ 1e-14` 量级。
- 影响范围：
  - 实验（新增 prototype 统计结果与基线对比工件）
  - 性能评估（补充 O5 第二阶段观测数据）
  - 行为语义（仅统计，不改变求解输出）
- 备注：
  - 本次为量化验证；后续可基于该口径推进“解析路径 + 条件回退”混合实现与 A/B 复测。

### 2026-03-20 — O5 基准口径扩展：benchmark CSV 接入 P2 prototype 统计

- 修改文件 / 模块：
  - `experiments/exp_analytical_ab_benchmark.cpp`
- 变更内容：
  - 扩展 A/B benchmark 聚合结构与 CSV 输出列，新增 P2 prototype 统计口径：
    - `p2_proto_cases`
    - `p2_proto_fit_rms_mean`
    - `p2_proto_fit_max_abs`
    - `p2_proto_symbolic_cases`
    - `p2_proto_symbolic_singular`
    - `p2_proto_symbolic_coeff_rms_mean`
    - `p2_proto_symbolic_coeff_max_abs`
  - benchmark 中 A/B 配置均启用 `enable_p2_derivative_prototype`，确保输出数据可直接用于 O5 对照分析。
  - mode 标签更新为 `A_no_dedup_proto` / `B_dedup_adaptive_proto`，避免与旧口径混淆。
- 影响范围：
  - 实验（CSV 列与 mode 命名扩展）
  - 性能评估（增加 prototype 统计开销观测维度）
  - 行为语义（求解结果不变，prototype 仍为 compare-only）
- 备注：
  - 该改动用于 O5 第二阶段量化，便于后续解析路径与回退策略的 A/B 验证。

### 2026-03-20 — O5 原型第二步：P2 解析系数 shadow 对照（含奇异回退计数）

- 修改文件 / 模块：
  - `include/sbf/envelope/analytical_solve.h`
  - `src/envelope/analytical_solve.cpp`
  - `docs/API_REFERENCE_CN.md`
- 变更内容：
  - 在 `solve_faces()` 的 compare-only 分支中新增“解析系数 shadow 重建”路径：
    - 基于 3×3 采样网格的双线性三角基表示（`B = U_i M U_j^T`）重建系数矩阵 `M`；
    - 将重建得到的 9 维系数向量与 `QR` 求解结果逐维对照，仅统计误差，不参与 root 求解。
  - 新增统计字段：
    - `n_p2_proto_symbolic_cases`
    - `n_p2_proto_symbolic_singular`
    - `p2_proto_symbolic_coeff_rms_mean`
    - `p2_proto_symbolic_coeff_max_abs`
  - 两个主入口（普通版 / with_configs）均完成线程安全归并与 `out_stats` 写回。
- 影响范围：
  - API（统计字段扩展）
  - 性能（仅启用 `enable_p2_derivative_prototype` 时有轻微额外统计开销）
  - 行为语义（默认关闭；启用后仍为 compare-only，不改变求解输出）
- 备注：
  - 本步为 O5“解析替代”前的中间验证层，后续可在该口径下接入真实解析微分系数路径并保留奇异回退开关。

### 2026-03-20 — O5 原型第一步：P2 解析微分对照统计（compare-only）

- 修改文件 / 模块：
  - `include/sbf/envelope/analytical_solve.h`
  - `src/envelope/analytical_solve.cpp`
  - `docs/API_REFERENCE_CN.md`
- 变更内容：
  - 新增配置开关 `enable_p2_derivative_prototype`，用于在 P2 路径采集“解析微分替代前”对照指标。
  - 在 `solve_faces()` 中新增 compare-only 统计：对 9 点拟合结果计算残差 `RMS` 与 `max_abs`，仅用于基线观测。
  - 新增统计字段：
    - `n_p2_proto_cases`
    - `p2_proto_fit_rms_mean`
    - `p2_proto_fit_max_abs`
  - 两个主入口（普通版 / with_configs）均完成线程安全归并。
- 影响范围：
  - API（配置与统计字段扩展）
  - 性能（仅启用原型开关时有轻微额外统计开销）
  - 行为语义（默认关闭，不改变求解结果）
- 备注：
  - 该步骤为 O5 原型基线铺垫；后续可在同一口径下接入真实解析系数推导并做 A/B 对照。

### 2026-03-20 — O4 阶段基准复测（12 次重复，含方差）

- 修改文件 / 模块：
  - `results/exp_analytical_ab_repeat/run_*.log`
  - `results/exp_analytical_ab_repeat/summary_stats.csv`
  - `results/exp_analytical_ab_repeat/all_runs_rows.csv`
- 变更内容：
  - 对 `exp_analytical_ab_benchmark` 使用最新二进制执行 12 次重复复测，输出每次原始日志并汇总均值/标准差。
  - 关键汇总（`Speedup_mean = A/B`）：
    - width=0.10: `1.012 ± 0.017`
    - width=0.20: `1.011 ± 0.035`
    - width=0.35: `1.003 ± 0.017`
    - width=0.50: `1.000 ± 0.016`
    - width=0.70: `0.994 ± 0.029`
  - `FK_reduction_mean_pct` 约 `0.026% ~ 0.168%`，口径稳定、无异常波动。
- 影响范围：
  - 实验（新增可追溯的重复统计结果）
  - 性能评估（当前 O4 细化在该基准集上的平均收益接近持平，小宽度略正收益）
- 备注：
  - 本次为验证与量化，不涉及 API/行为语义变更。

### 2026-03-20 — AA 剪枝细化：solver 内 axis/face 级短路

- 修改文件 / 模块：
  - `src/envelope/analytical_solve.cpp`
  - `docs/API_REFERENCE_CN.md`
- 变更内容：
  - 新增 `aa_link_axis_improve_summary(...)`，在保持包络安全语义不变前提下，判定每个 link 在 `x/y/z` 轴是否仍可能改进。
  - 在 P1/P2/P2.5/P3/P3+ 各 solver 内接入 axis/face 级短路：
    - 轴向不可改进时，跳过对应 `d` 维候选构造与求解；
    - face 对应轴不可改进时，直接跳过该 face 扫描。
  - 两个主入口（普通版 / with_configs）均将 `aa_ws.prefix` 传入 solver，实现阶段内与阶段间一致的 AA 动态剪枝语义。
- 影响范围：
  - 性能（减少阶段内无效候选生成与 FK 评估）
  - 行为语义（包络安全语义不变）
- 备注：
  - 本次未新增统计字段，沿用现有 AA 统计口径；后续可按需补充 axis/face 级命中计数。

### 2026-03-20 — AA 动态剪枝细化：真实 segment 计数 + 全剪枝阶段短路

- 修改文件 / 模块：
  - `src/envelope/analytical_solve.cpp`
  - `docs/API_REFERENCE_CN.md`
- 变更内容：
  - 将 `n_aa_pruned_segments` 从估算值（`pruned_links * n_sub`）改为按每个 link 的真实 segment 可改进性判定累加。
  - 在 phase refresh 后增加“全量剪枝短路”：若该阶段 `pruned_links == n_act`，则直接跳过该阶段求解与线程调度。
  - 两个主入口（普通版 / with_configs）均同步该短路逻辑与统计口径。
- 影响范围：
  - 性能（全剪枝阶段避免无效线程创建与求解开销）
  - 统计（`n_aa_pruned_segments` 更精确）
  - 行为语义（包络安全语义不变）
- 备注：
  - 下一步可继续推进 segment/axis 级“阶段内循环短路”，进一步减少候选生成开销。

### 2026-03-20 — AA 剪枝增强：阶段级动态刷新（phase-level）

- 修改文件 / 模块：
  - `include/sbf/envelope/analytical_solve.h`
  - `src/envelope/analytical_solve.cpp`
  - `docs/API_REFERENCE_CN.md`
- 变更内容：
  - 在解析主流程中，将 AA 剪枝从“Phase 0 后一次性 `skip_link`”增强为“每个阶段执行前动态刷新 `skip_link`”。
  - 新增通用刷新函数：按当前 `link_seg_ext` 重新判定每个 link 是否仍可改进，并用于 P1/P2/P2.5/P3 及 `dual_phase3` 第二次 pass。
  - 两个主入口（`derive_aabb_critical_analytical` / `_with_configs`）均同步接入该动态刷新逻辑。
  - 新增 AA 动态统计字段：
    - `n_aa_pruned_segments`
    - `n_aa_pruned_phase_checks`
- 影响范围：
  - 性能（阶段级动态短路，减少无效候选与 FK 调用）
  - API（统计字段扩展）
  - 行为语义（包络安全语义不变）
- 备注：
  - 当前实现优先落地 phase 级动态判定；更细粒度的 segment/axis 级短路可作为下一步继续推进。

### 2026-03-20 — 按 link 并行扩展到 P2.5/P3（含 dual_phase3）

- 修改文件 / 模块：
  - `src/envelope/analytical_solve.cpp`
  - `docs/API_REFERENCE_CN.md`
- 变更内容：
  - 将按 `ci` 分块并行从 P1/P2 扩展到：
    - Phase 2.5a（`solve_pair_constrained_1d`）
    - Phase 2.5b（`solve_pair_constrained_2d`）
    - Phase 3 / 3+（`solve_interior` / `solve_interior_improved`）
  - 为上述 solver 增加 `ci_begin/ci_end` 分块参数，统一与 P1/P2 的并行切分方式。
  - 在两个主入口（`derive_aabb_critical_analytical` / `_with_configs`）中均接入并行调度与线程局部统计归并。
  - `dual_phase3` 的第二次 interior pass 同步支持并行，并归并到 P3 去重统计。
- 影响范围：
  - 性能（多核可并行覆盖 P1/P2/P2.5/P3）
  - API 文档（并行覆盖范围更新）
  - 行为语义（默认串行不变，开启后与串行口径一致）
- 备注：
  - 当前终端缺少 `cmake/msbuild` 工具链，已完成代码与静态错误检查；可在本机 VS/CMake 环境执行目标构建进行最终回归。

### 2026-03-20 — 按 link 并行（第一版，覆盖 P1/P2）

- 修改文件 / 模块：
  - `include/sbf/envelope/analytical_solve.h`
  - `src/envelope/analytical_solve.cpp`
  - `docs/API_REFERENCE_CN.md`
- 变更内容：
  - 新增并行配置：
    - `enable_parallel_by_link`
    - `parallel_num_threads`（`<=0` 自动硬件并发）
  - 在解析求解主流程中，Phase 1（`solve_edges`）与 Phase 2（`solve_faces`）支持按 `ci` 分块并行执行。
  - 每个线程使用局部统计与局部去重计数，阶段末进行归并，避免共享写冲突。
  - 默认配置仍为串行（`enable_parallel_by_link=false`），保持现有行为兼容。
- 影响范围：
  - API（配置字段扩展）
  - 性能（多核可用）
  - 行为语义（默认不变）
- 备注：
  - 本次为并行第一版，当前覆盖 P1/P2；P2.5/P3 并行将在下一步继续扩展。

### 2026-03-20 — Pair_2d root 批量评估（降低评估抖动）

- 修改文件 / 模块：
  - `src/envelope/analytical_solve.cpp`
- 变更内容：
  - 在 `solve_pair_constrained_2d` 中，将 root 处理从“生成即评估”改为“先收集 triplet 候选，再批量评估”。
  - 候选流程调整为：
    1) 收集 `(qi, qj, qm)` triplet 候选；
    2) 按现有自适应去重策略决定是否去重；
    3) 对候选排序（优先 `qm`，再 `qi`，最后 `qj`）后批量调用 `eval_and_update_from`。
  - 去重统计口径保持不变（继续使用 P2.5 统计字段）。
- 影响范围：
  - 性能（Phase 2.5b 评估路径更稳定）
  - 行为语义（不变，仍在同一约束域内评估有效候选）
- 备注：
  - 本次重点是降低候选评估抖动与重复检查开销；是否带来端到端明显提速需结合 benchmark 复测确认。

### 2026-03-20 — Analytical P2 去重接入（Phase 2 root 路径）

- 修改文件 / 模块：
  - `include/sbf/envelope/analytical_solve.h`
  - `src/envelope/analytical_solve.cpp`
  - `experiments/exp_analytical_ab_benchmark.cpp`
  - `docs/API_REFERENCE_CN.md`
- 变更内容：
  - 将候选去重从 P1/P2.5/P3 扩展到 **P2（`solve_faces`）** root 候选路径。
  - `solve_faces` 新增去重运行时接入（复用自适应去重策略），并对 `(qi,qj)` 候选对做去重。
  - `AnalyticalCriticalStats` 新增 P2 去重统计字段：
    - `n_dedup_raw_candidates_p2`
    - `n_dedup_unique_candidates_p2`
    - `n_dedup_applied_sets_p2`
    - `n_dedup_skipped_sets_p2`
  - 两个主入口（`derive_aabb_critical_analytical` / `_with_configs`）已汇总并写出 P2 去重统计。
  - A/B benchmark CSV 新增 P2 维度列：
    - `dedup_hit_rate_p2`
    - `dedup_applied_sets_p2`
    - `dedup_skipped_sets_p2`
  - 修复 benchmark 输出格式串列对齐问题，保证 `dedup_skipped_sets_p3` 与 `checksum` 不错位。
- 影响范围：
  - API（统计字段扩展）
  - 实验输出（CSV 列扩展）
  - 性能分析口径（P2 可观测）
- 备注：
  - 当前样例数据下 P2 命中率仍接近 0，说明下一步需推进 P2 root 生成侧去重键策略与阈值调参。
  - 本次改动保持结果语义不变，仅增加候选去重与统计可观测性。

### 2026-03-19

- 修改文件 / 模块：
  - `doc/AI_WORKING_SPEC.md`
  - `doc/CHANGE_LOG_CN.md`
- 变更内容：
  - 将 spec 和中文变更日志从 v4 根目录迁移到 `doc/` 目录统一管理。
  - 同步更新 spec 中对中文日志文件的路径引用，改为 `doc/CHANGE_LOG_CN.md`。
- 影响范围：
  - 规范
  - 文档目录结构
- 备注：
  - 后续应统一以 `doc/AI_WORKING_SPEC.md` 和 `doc/CHANGE_LOG_CN.md` 作为规范与日志的正式位置。

### 2026-03-19

- 修改文件 / 模块：
  - `AI_WORKING_SPEC.md`
  - `CHANGE_LOG_CN.md`
- 变更内容：
  - 将 spec 中“英文论文 + 中文论文”的维护要求细化为更具体的主论文文件名约定。
  - 增加 `paper/root.tex` 与 `paper/root_cn.tex` 的默认表述，并说明若项目使用其他主文件名，则应同步维护对应文件。
- 影响范围：
  - 规范
  - 论文维护规则
- 备注：
  - 该修改用于减少“需要维护哪份论文文件”的歧义。

### 2026-03-19

- 修改文件 / 模块：
  - `AI_WORKING_SPEC.md`
  - `CHANGE_LOG_CN.md`
- 变更内容：
  - 在工作规范中补充“英文论文也需要维护”的要求。
  - 明确论文相关变更需同步更新中英文论文，而非只维护中文论文。
- 影响范围：
  - 规范
  - 论文维护流程
- 备注：
  - 保持“不强制维护英文 API 文档”的规则不变，仅恢复中英文论文同步维护要求。

### 2026-03-19

- 修改文件 / 模块：
  - `AI_WORKING_SPEC.md`
  - `CHANGE_LOG_CN.md`
- 变更内容：
  - 将工作规范从 v3 活跃阶段更新为 v4 活跃阶段。
  - 明确 v4 迁移以 v3 当前最新稳定状态为基线。
  - 删除“英文版 API 文档必须同步更新”的强制要求，改为仅强制维护中文 API / 说明文档。
  - 新增“中文变更日志维护”规则，并创建中文日志模板文件。
- 影响范围：
  - 规范
  - 文档流程
  - 开发协作要求
- 备注：
  - 后续每次代码修改都应继续在本文件追加中文记录。
### 2026-03-20 — v4 文件骨架搭建

- 修改文件 / 模块：
  - `include/sbf/core/` — `types.h`, `config.h`, `interval_trig.h`
  - `include/sbf/robot/` — `robot.h`, `fk.h`, `interval_fk.h`, `affine_fk.h`, `interval_math.h`
  - `include/sbf/envelope/` — `endpoint_source.h`, `envelope_type.h`, `pipeline.h`, `crit_sample.h`, `analytical_solve.h`, `gcpc.h`, `envelope_cache.h`, `endpoint_store.h`, `collision_policy.h`
  - `include/sbf/voxel/` — `bit_brick.h`, `voxel_grid.h`, `hull_rasteriser.h`
  - `include/sbf/scene/` — `scene.h`, `i_collision_checker.h`, `aabb_collision_checker.h`
  - `include/sbf/forest/` — `sbf.h`, `lect.h`, `forest_grower.h`, `grower_config.h`, `adjacency_graph.h`, `node_store.h`, `kd_tree.h`, `thread_pool.h`
  - `include/sbf/planner/` — `path_result.h`, `i_planner.h`, `sbf_planner.h`
  - `src/robot/` — `robot.cpp`, `fk_scalar.cpp`, `interval_fk.cpp`, `interval_math.cpp`, `affine_fk.cpp`
  - `src/envelope/` — `endpoint_source.cpp`, `envelope_type.cpp`, `crit_sample.cpp`, `analytical_solve.cpp`, `gcpc.cpp`, `envelope_cache.cpp`, `endpoint_store.cpp`, `collision_policy.cpp`
  - `src/voxel/` — `hull_rasteriser.cpp`
  - `src/scene/` — `scene.cpp`, `collision.cpp`
  - `src/forest/` — `sbf.cpp`, `lect.cpp`, `forest_grower.cpp`, `grower_roots.cpp`, `grower_bridge.cpp`, `grower_coarsen.cpp`, `adjacency_graph.cpp`, `node_store.cpp`
  - `src/planner/` — `sbf_planner.cpp`
  - `CMakeLists.txt`
- 变更内容：
  - 搭建 v4 完整文件骨架（34 个头文件 + 25 个源文件 + CMakeLists.txt）。
  - 遵循 v4 命名规范：i-前缀（iAABB, endpoint_iaabb, link_iaabb）；SubAABB→LinkIAABB；frame_source→endpoint_source；frame_store→endpoint_store；common/→core/。
  - 所有源文件为 stub 实现（仅含 TODO 标注），头文件包含类声明、枚举和关键接口签名。
  - `core/types.h` 为唯一包含完整实现的文件（类型定义为 header-only）。
  - 新增 `planner/` 模块（v4 新增），包含 IPlanner 抽象接口和 SBFPlanner 骨架。
  - 二阶段流水线骨架：Stage 1（4 种 EndpointSource × endpoint_iaabb）→ Stage 2（3 种 EnvelopeType → link_envelope），4×3 = 12 组合。
  - `pipeline.h` 整合 EndpointSourceConfig + EnvelopeTypeConfig，提供 recommended/fast/tightest/production 工厂方法。
  - `sbf.h` 提供 12 种 SBFConfig 工厂方法（ifk_link_iaabb, crit_link_iaabb 等）。
  - CMakeLists.txt 使用显式源文件列表（非 GLOB_RECURSE），依赖 Eigen3 + nlohmann_json + Threads。
  - 编译验证通过：`cmake --build . --config Release` 零错误零警告，生成 `sbf4.lib`。
- 影响范围：
  - 项目结构
  - 构建系统
  - 全部模块的 API 骨架
- 备注：
  - 本次仅搭建骨架，不包含具体实现。后续按 endpoint_iAABB→envelope→sbf 扩展→plan 的顺序逐步迁移 v3 代码。
  - 所有 v3 向后兼容别名已在 v4 中移除。
  - v3 源文件到 v4 的映射关系已在各 stub 文件的 TODO 注释中标注。

### 2026-03-25 — 移植基础版 ForestGrower（仅串行 Wavefront / RRT）

- 修改文件 / 模块：
  - `include/sbf/forest/forest_grower.h` — 从 v3 补回基础 grower 私有状态与串行扩展相关声明
  - `src/forest/forest_grower.cpp` — 实现基础版根选择、Wavefront grower、RRT grower、边界采样、FFB 包装与 `grow_subtree()`
  - `include/sbf/forest/adjacency_graph.h` — 补回 grower 依赖的图接口：`rebuild()`、`connected()`、`n_components()`、`find_nearest_box()`
  - `src/forest/adjacency_graph.cpp` — 实现基础线性版邻接图与最近邻查询
- 变更内容：
  - 从 v3 迁移 **最基本可用** 的 grower 主流程，仅支持串行模式下的 `GrowerConfig::Mode::Wavefront` 与 `GrowerConfig::Mode::RRT`。
  - 支持基础多根根选择：无端点时 FPS 风格选根；有端点时固定 start/goal 为前两棵树，并为其余根做扰动采样。
  - 支持基础 subtree 区域记录，用于 grower 内部随机采样与 `grow_subtree()` 的最小工作流。
  - 支持 `sample_boundary()`、`sample_near_existing_boundary()`、`rrt_snap_to_face()`，保证新盒子尽量贴已有边界扩展。
  - `try_create_box()` 重新接回 `LECT::find_free_box()` / `mark_occupied()`，使 grower 真正基于 v4 LECT 工作。
  - `AdjacencyGraph` 先采用**线性基础实现**，满足 grower 的增量加边、连通性判定与最近盒查询；暂未恢复 v3 的 sweep-and-prune / KD-tree 优化。
  - 对 `n_threads > 1` 明确退回串行执行；桥接、并行 worker、promotion、coarsen 仍保留后续迁移。
- 影响范围：
  - ForestGrower 行为语义
  - AdjacencyGraph API 与运行时行为
  - 规划前端对 grower 的基本可用性
- 备注：
  - 本次是“基础可跑”移植，不包含并行 grower、bridge、promote、coarsen。
  - 当前 `AdjacencyGraph` 为正确性优先版本，后续可继续替换为 v3 的高性能实现。

### 2026-03-25 — 继续移植 promotion/coarsen 与并行 grower

- 修改文件 / 模块：
  - `include/sbf/forest/thread_pool.h` — 从 v3 迁移可用的 header-only 线程池实现
  - `include/sbf/forest/forest_grower.h` — 补充 `grow_parallel()`、`promote_all()`、`coarsen_greedy()`、`bridge_subtree_boundaries()` 等私有接口声明
  - `src/forest/forest_grower.cpp` — 实现 promotion、greedy coarsen、并行 worker 调度、worker 合并、边界 bridge 与 LECT 占用同步
- 变更内容：
  - 将 `grow()` 扩展为支持 `n_threads > 1` 时的并行子树生长路径；串行路径继续保留。
  - 从 v3 迁移 `promote_all()` / `try_promote_node()`，恢复基于 LECT 父节点的提升合并逻辑。
  - 从 v3 迁移 `coarsen_greedy()` / `remove_box_by_id()`，恢复基于邻接图与区间 hull 的贪心粗化流程。
  - 从 v3 迁移 `grow_parallel()`：支持 worker 级 `grow_subtree()`、共享全局 box 预算、可选 warm-start snapshot、结果 remap/merge。
  - 从 v3 迁移 `sync_lect_occupation()` 与 `bridge_subtree_boundaries()`，在并行合并后可对相邻子树边界追加 bridge boxes。
  - `grow_subtree()` 现在也会执行 worker 层 promotion，并回填统计量。
- 影响范围：
  - ForestGrower 扩展策略
  - 并行运行时行为
  - 粗化/提升后的 box 数与连通图结构
- 备注：
  - 当前并行 grower 已接回主要逻辑，但仍依赖 v4 其余模块的现有稳定性；全量构建仍会被项目内其他未完成模块阻塞。
  - `AdjacencyGraph` 仍为线性基础实现，因此并行 merge / coarsen 目前以正确性优先，性能尚未恢复到 v3 水平。

### 2026-03-21 — 自适应冻结深度系统实施

- 修改文件 / 模块：
  - `include/sbf/core/config.h` — 新增 `FreezePolicy` 结构体
  - `include/sbf/robot/interval_math.h` — 命名空间从 `sbf::robot` 改为 `sbf::`，新增 `I_sin`/`I_cos` 声明
  - `src/robot/interval_math.cpp` — 实现 `I_sin`/`I_cos`（区间正弦/余弦，含极值检测）
  - `include/sbf/core/interval_trig.h` — 从 v3 完整迁移（~200 行）：`classify_alpha`、`FrozenJointDesc`、`undo_frozen_joint`、`apply_joint_rotation`、`reconstruct_cartesian_endpoints_multilevel`
  - `include/sbf/robot/robot.h` — 新增 `DHParam` 结构体、`dh_params()`/`link_radii()`/`tool_frame()` 访问器
  - `src/robot/robot.cpp` — 新增 DH 访问器 stub 实现
  - `include/sbf/forest/lect.h` — 新增第三构造函数（FreezePolicy 参数）、`freeze_policy()`/`max_freeze_depth()`/`effective_freeze_depth()` 公共方法、私有成员（`max_freeze_depth_`/`frozen_descs_`/`freeze_policy_`/`ep_store_`/`split_dims_`/`hull_grids_`/`hull_valid_`）、私有辅助函数（`init_freeze_depth`/`reconstruct_and_derive_link_iaabbs`/`ensure_ep_capacity`/`ensure_hull_capacity`/`init_root`/`refine_aabb`）
  - `src/forest/lect.cpp` — 实现 `init_freeze_depth()`（DH 自动推导 + FreezePolicy 约束）、`effective_freeze_depth()`（双上限逻辑）、第三构造函数、私有辅助 stub
  - `include/sbf/forest/node_store.h` — 新增 `freeze_depth_`/`source_quality_` 向量、getter/setter、`ensure_capacity`/`alloc_node` 等容量管理接口
  - `src/forest/node_store.cpp` — 实现 per-node freeze_depth/source_quality 读写、容量管理 stub
- 变更内容：
  - **FreezePolicy 结构体**：`adaptive`(bool)、`max_structural`(int)、`min_interval_joints`(int, 默认=3)、`width_threshold`(double, 默认=π/2)；
    工厂方法：`v3_compatible()` {false,2,0,99}、`adaptive_default()` {true,-1,3,π/2}、`disabled()` {false,0,0,0}
  - **interval_trig.h 完整迁移**：`classify_alpha()` 判断 α 类型(0/±π/2)；`FrozenJointDesc` 描述冻结关节；`undo_frozen_joint()` 精确逆变换(无区间展宽)；`apply_joint_rotation()` 区间旋转(引入 I_cos·X-I_sin·Y 包裹)；`reconstruct_cartesian_endpoints_multilevel()` 两种重载
  - **init_freeze_depth()** DH 自动推导：扫描前导关节，连续满足 a=0 且 α∈{0,±π/2} 的关节计入结构冻结深度；非自适应模式下硬性截断到 `max_structural`；自适应模式下保留原始结构深度
  - **effective_freeze_depth()** 双上限逻辑：structural_cap = max(0, max_fd - min_interval_joints)；width_cap = 连续前导窄关节数(宽度≤threshold)；结果 = min(structural_cap, width_cap)
  - **退化保护**：`min_interval_joints=3` 确保至少 3 个关节保留在真正的区间 FK 中。IIWA14: structural_cap=7-3=4（最多冻结 4 层）
  - 编译验证：clean rebuild 零错误零警告 → `sbf4.lib`
- 影响范围：
  - API（新增 FreezePolicy、DHParam、LECT 第三构造函数）
  - 核心数学（I_sin/I_cos、interval_trig 全套）
  - 节点存储（per-node freeze_depth）
  - 缓存格式（未来需更新 HCACHE 格式以持久化 freeze_depth）
- 备注：
  - I_sin/I_cos 为简化实现，完整 v3 版本将在 Phase A（robot 模块迁移）时替换
  - split_leaf 的 Case A（继承）/ Case B（重算）逻辑已预留接口，具体实现待 compute_envelope 管线迁移后完成
  - 实验计划见 `doc/PLAN_ADAPTIVE_FREEZE_DEPTH.md`（5 组实验）

### 2026-03-19 — 修正冻结机制理解：全冻结 = 完整 iFK（非退化）

- 修改文件 / 模块：
  - `doc/PLAN_ADAPTIVE_FREEZE_DEPTH.md` — §1 全面重写
  - `include/sbf/core/config.h` — FreezePolicy 注释修正
  - `src/forest/lect.cpp` — effective_freeze_depth 注释修正
- 变更内容：
  - **修正核心认知错误**：之前认为"全冻结退化为标量 FK + wrapping"，实际上
    `apply_joint_rotation` 本身就是区间旋转（$I_{\cos}(q)\cdot X - I_{\sin}(q)\cdot Y$），
    与 iFK 链乘中的 DH 变换矩阵乘法是**同类运算**。全冻结 ≡ 完整 iFK。
  - **重新定义 `min_interval_joints` 的作用**：
    - 当 EndpointSource = iFK 时：**无实质影响**（冻结与 iFK 同类运算，不影响质量）
    - 当 EndpointSource = CritSample/Analytical/GCPC 时：**有意义**（确保末端连杆有足够关节由更紧的方法处理）
  - **将冻结机制重新定义为 "iFK 链的分解策略"**，而非"冻结→退化→重建"
  - §1.6 新增：架构扩展方向（冻结部分可替换为 Affine Arithmetic / Taylor Models / GCPC 重建等）
  - §6.2 决策树新增 Source 感知逻辑
  - §7.2-7.3 wrapping 分析标注：这些 wrapping 同样存在于标准 iFK 中
  - §8 验证标准更新
- 影响范围：
  - 文档（设计理念修正）
  - API 注释（FreezePolicy.min_interval_joints 语义修正）
- 备注：
  - 代码逻辑不变（effective_freeze_depth 的双上限算法仍然正确）
  - 编译验证通过：零错误零警告

### 2026-03-19 — 冻结优先切分策略（Freeze-Priority Splitting）

- 修改文件 / 模块：
  - `doc/PLAN_ADAPTIVE_FREEZE_DEPTH.md` — 新增 §10（冻结优先切分策略完整分析）
  - `include/sbf/core/config.h` — 新增 `SplitOrder` 枚举，`FreezePolicy` 新增 `split_order` 字段
- 变更内容：
  - **核心洞察**：冻结本质是"在线 iFK + 降低离线缓存维度"。v3 round-robin 切分对冻结不友好——
    IIWA14 需要 depth=14 才能让所有冻结维度低于阈值，而 freeze-priority 在 depth=2 就能开始冻结第一个维度
  - **三种切分策略设计**：
    - 方案 A（两阶段）：Phase I 依次缩窄冻结维度，Phase II round-robin 非冻结维度
    - 方案 B（动态最宽）：每次选择冻结候选中最宽的维度切分
    - 方案 C（混合）：优先切分宽于阈值的冻结维度，否则 round-robin
  - **量化分析**：freeze-priority 在 depth=2 达到 fd=1（round-robin 需 depth=14），depth 8-13 区间可多继承约 4800 次 Stage 1
  - **pre_expand 协同**：`pre_expand(11)` + freeze-priority 将 7-DOF 降维为 3-DOF 缓存 + 4-DOF 在线 iFK
  - `SplitOrder` 枚举：`ROUND_ROBIN`(0), `FREEZE_PRIORITY`(1), `WIDEST_FIRST`(2)
  - `FreezePolicy` 工厂方法更新以包含 `split_order` 字段
  - 新增实验 6：切分策略対比实验计划
  - 编译验证通过：零错误零警告
- 影响范围：
  - API（新增 SplitOrder 枚举，FreezePolicy 结构体扩展）
  - 文档（设计分析）
- 备注：
  - 切分策略的实际实现（`init_split_dims`、`split_leaf` 动态维度选择）待 LECT 完整迁移后实现
  - 默认值 `SplitOrder::ROUND_ROBIN` 保持 v3 兼容

### 2026-03-19 — iFK Pipeline 迁移计划

- 修改文件 / 模块：
  - `doc/PLAN_IFK_PIPELINE.md` — **新建**，iFK 流水线迁移与自适应冻结适配计划
- 变更内容：
  - 制定 7 步迁移计划 (S1-S7)，将 v3 iFK 流水线完整迁移至 v4
  - **S1**: Robot 类迁移（放弃 pimpl，迁移 v3 直接成员 + pack_arrays）
  - **S2**: interval_math 完整迁移（I_sin/I_cos outward rounding + build_dh_joint + imat_mul_dh）
  - **S3**: interval_fk 迁移（FKState 完整字段 + compute_fk_full/incremental + extract_link_aabbs）
  - **S4**: envelope_derive 迁移（derive_aabb_subdivided，extract_link_iaabbs 依赖）
  - **S5**: endpoint_source iFK 路径实现（compute_endpoint_iaabb IFK 分支 + fk_to_endpoints + extract_link_iaabbs）
  - **S6**: LECT iFK 集成 + 自适应冻结（compute_envelope per-node fd + split_leaf Case A/B + init_split_dims + NodeStore SoA）
  - **S7**: 编译验证 + 4 组单元测试
  - 整合点分析：PLAN_ADAPTIVE_FREEZE_DEPTH.md 中所有设计（双重 cap、per-node fd、Case A 继承、冻结优先切分）在 S6 中完整实现
  - 预估总量: ~2,869 行代码，~4.6h
- 影响范围：
  - 文档（迁移设计）
- 备注：
  - 本计划为 MIGRATION_ENDPOINT_IAABB.md Phase A-B, G 的细化执行版本，聚焦 iFK 路径
  - CritSample/Analytical/GCPC 三条路径在 endpoint_source.cpp 中保持 stub，后续 Phase D-F 迁移
  - outward rounding 缺失是当前 v4 的一个安全隐患（可能导致碰撞检测 false negative），S2 修复

### 2026-03-20 — iFK Pipeline 完整迁移 (S1-S7)

- 修改文件 / 模块：
  - **S1 Robot 类迁移**：
    - `include/sbf/robot/robot.h` — 放弃 pimpl，迁移 v3 直接成员 + DHParam/EESphere/EEGroup/Robot，~151 行
    - `src/robot/robot.cpp` — from_json + pack_arrays + fk_link_positions/fk_transforms + fingerprint，~245 行
    - `include/sbf/robot/fk.h` — fk_link_positions/fk_transforms/dh_transform 声明（sbf:: 命名空间）
    - `src/robot/fk_scalar.cpp` — 标量 FK stub（sbf:: 命名空间）
  - **S2 interval_math 完整迁移**：
    - `include/sbf/robot/interval_math.h` — 增加 outward_lo/outward_hi, build_dh_joint, imat_mul_dh 等声明
    - `src/robot/interval_math.cpp` — I_sin/I_cos outward rounding 修复 + build_dh_joint + imat_mul_dh，~155 行
  - **S3 interval_fk 迁移**：
    - `include/sbf/robot/interval_fk.h` — FKState 完整字段（prefix_lo/hi[MAX_TF][16]），所有函数声明在 sbf:: 命名空间
    - `src/robot/interval_fk.cpp` — compute_fk_full/incremental + extract_link_aabbs + extract_ee 函数，~255 行
  - **S4 envelope_derive 迁移**：
    - `include/sbf/envelope/envelope_derive.h` — derive_aabb/derive_aabb_subdivided/adaptive_subdivision_count/derive_grid 声明（新建）
    - `src/envelope/envelope_derive.cpp` — 完整实现，~185 行（新建）
    - `CMakeLists.txt` — 添加 envelope_derive.cpp 到源列表
  - **S5 endpoint_source iFK 路径**：
    - `src/envelope/endpoint_source.cpp` — compute_endpoint_iaabb IFK 分支 + fk_to_endpoints + extract_link_iaabbs，~145 行
  - **S6 LECT iFK 集成 + 自适应冻结**：
    - `include/sbf/forest/node_store.h` — 完善 SoA 类：init/alloc_node/link_iaabbs 访问器 + freeze_depth/source_quality 字段，~100 行
    - `src/forest/node_store.cpp` — init/ensure_capacity/alloc_node/clear_all_occupation 完整实现，~100 行
    - `include/sbf/forest/lect.h` — 增加 init_split_dims/compute_envelope/split_leaf 声明 + root_fk_/node_split_dim_ 成员 + expand_leaf 公开方法
    - `src/forest/lect.cpp` — 完整 LECT 实现：
      - `init_split_dims()`: ROUND_ROBIN/FREEZE_PRIORITY/WIDEST_FIRST 三种策略
      - `init_root()`: 分配根节点 + 计算全范围 iFK + compute_envelope
      - `compute_envelope()`: 自适应 fd + 冻结 local-frame 端点 + 多级重建 + derive_aabb
      - `split_leaf()`: Case A（冻结维度切分→继承 ep_store + 仅重建）/ Case B（增量 iFK + 完整 compute_envelope）
      - `reconstruct_and_derive_link_iaabbs()`: FrozenJointDesc 多级旋转重建 + derive_aabb
      - `refine_aabb()`: intersect(parent, union(left, right))
      - `node_intervals()`: 根→目标节点路径行走重建 C-space 区间
      - 所有 accessor/occupation 委托至 store_
      - `expand_leaf()`: 公开方法封装 split_leaf，供测试和预展开使用
  - **S7 编译验证 + 单元测试**：
    - `tests/test_ifk_pipeline.cpp` — 7 组集成测试（interval_math/interval_fk/endpoint_source/interval_trig/LECT/LECT_v3_compat/refine_aabb），172 个断言全部通过
    - `CMakeLists.txt` — 添加 test_ifk_pipeline 测试目标
- 变更内容：
  - 将 v3 iFK 流水线完整迁移至 v4，包含自适应冻结深度和冻结优先切分
  - **outward rounding 安全修复**：v4 stub 缺失 ULP 保守宽化 → 可能导致碰撞检测 false negative → S2 修复
  - **命名空间统一**：所有 iFK 函数在 `sbf::` 命名空间（非 `sbf::robot::`），保持 v3 兼容
  - **Case A 继承优化**：冻结维度切分时跳过 Stage 1（compute_endpoint_iaabb），仅做 Stage 2 多级重建
  - **per-node 冻结深度**：NodeStore 存储 freeze_depth/source_quality 字节，compute_envelope per-node 计算
  - **3 种切分策略完整实现**：ROUND_ROBIN（v3 兼容）、FREEZE_PRIORITY（Phase I 冻结维优先）、WIDEST_FIRST（动态最宽）
  - **编译**：MSVC 2022 Release，零错误零警告，sbf4.lib + test_ifk_pipeline.exe
- 影响范围：
  - API（Robot/FKState/EndpointIAABBResult/LECT/NodeStore 从 stub 变为完整实现）
  - 安全性（outward rounding 修复）
  - 性能（Case A 继承减少 ~50% 的 Stage 1 调用）
  - 构建（新增 envelope_derive.cpp 源文件 + test_ifk_pipeline 测试目标）
- 备注：
  - CritSample/Analytical/GCPC 路径在 endpoint_source.cpp 中仍回退到 iFK，待后续 Phase D-F 迁移
  - find_free_box 仍为 stub，待碰撞模块迁移后实现
  - save/load/snapshot/pre_expand/hull_grids 仍为 stub，待后续迁移
  - 标量 FK (fk_scalar.cpp) 为 stub，仅返回空结果，待需要时从 v3 迁移

### 2026-03-19 — 自适应冻结深度实验 1-3

- 修改文件 / 模块：
  - `experiments/exp_freeze_sweep.cpp`（新建，~200 行）
  - `experiments/exp_freeze_per_link.cpp`（新建，~210 行）
  - `experiments/exp_freeze_asymmetric.cpp`（新建，~250 行）
  - `CMakeLists.txt`（SBF_BUILD_EXPERIMENTS 添加 3 个实验目标）
  - `results/exp_freeze_sweep.csv`（713 行数据）
  - `results/exp_freeze_per_link.csv`
  - `results/exp_freeze_asymmetric.csv` + `results/exp_freeze_asymmetric_summary.txt`
- 变更内容：
  - **Exp 1 Wrapping Volume Sweep**：扫描 11 种区间宽度 × 8 种 freeze_depth × 8 个 endpoint，
    输出基线体积、冻结体积、膨胀比、各轴宽度
  - **Exp 2 Per-Link Decomposition**：固定 4 种宽度（0.3/0.5/1.0/1.5），逐活跃连杆分析
    iFK 关节数 / 重建层数 / 膨胀比之间的关系
  - **Exp 3 Asymmetric Intervals**：定义 7 个确定性 + 5 个随机测试用例，模拟 KD-tree 非对称分割，
    验证 effective_freeze_depth 在 5 种 min_interval_joints 设置下的行为
- 关键实验发现：
  - 工具端点（endpoint 7）在所有宽度和冻结深度下膨胀比 ≤ 1.005（W=0.5, fd≤4）
  - **冻结重建有时产生比直接 iFK 更紧的 bound**（ratio < 1.0），因依赖问题的分解方式不同
  - 近端连杆（link 0, frame 2）在 fd≥4 时膨胀显著（ratio=2.93@W=0.5），
    但 fd≤3 时仍为 1.0 → 确认 min_interval_joints=3 + structural_cap=4 是安全默认值
  - width_threshold=π/2 将有效冻结限制在窄关节范围内，符合理论预测
  - 非对称场景中，宽关节（q₀>π/2）正确阻止冻结，effective_fd 随 min_ij 线性下降
- 影响范围：
  - 实验数据（新增 3 个 CSV 结果文件）
  - 构建（CMakeLists 新增实验目标）
  - 设计验证（确认自适应冻结策略参数选择合理）
- 备注：
  - Exp 4（端到端 FFB）和 Exp 5（threshold 敏感性）需等 find_free_box + collision 模块迁移后实现
  - 实验代码仅依赖已完成的 iFK pipeline，不依赖任何 stub 模块
  - 回归测试 172/172 全部通过

### 2026-03-20 — Phase A 完成 + Phase H 持久化层迁移

- 修改文件 / 模块：
  - **Phase A 收尾（affine_fk）**：
    - `include/sbf/robot/affine_fk.h` — 从 v3 完整迁移（~240 行）
    - `src/robot/affine_fk.cpp` — 从 v3 完整迁移（~220 行）
  - **Phase H（mmap_util + EndpointStore + EnvelopeCache）**：
    - `include/sbf/core/mmap_util.h` — **新建**，从 v3 `common/mmap_util.h` 迁移（~265 行）
    - `include/sbf/envelope/endpoint_store.h` — 从 v3 `frame_store.h` 完整迁移（~165 行）
    - `src/envelope/endpoint_store.cpp` — 从 v3 `frame_store.cpp` 完整迁移（~390 行）
    - `include/sbf/envelope/envelope_cache.h` — 从 v3 完整迁移（~80 行）
    - `src/envelope/envelope_cache.cpp` — 从 v3 完整迁移（~115 行）
- 变更内容：
  - **affine_fk 完整迁移**：AffineForm（AA_NSYM=8 噪声符号）、Taylor 线性化 cos/sin
    保持 cos²+sin²=1 相关性、AAMatrix4 矩阵乘法、AAFKResult/AAFKWorkspace（增量 recompute_from）、
    aa_compute_fk/aa_extract_link_aabbs/aa_compute_link_aabbs
  - **mmap_util.h 跨平台内存映射**：MmapHandle RAII 结构体、open_rw/grow/flush/close、
    portable_fseek/ftell、Windows (CreateFileMapping) + POSIX (mmap) 双平台支持
  - **EndpointStore（原 FrameStore）迁移**：
    - 类名 `FrameStore` → `EndpointStore`
    - 方法重命名：`store_frames` → `store_endpoints`、`get_frames` → `get_endpoints`、
      `get_frames_mut` → `get_endpoints_mut`、`has_frames` → `has_endpoints`、
      `union_frames` → `union_endpoints`、`refine_frames` → `refine_endpoints`
    - 成员 `n_frames_` → `n_endpoints_`
    - FRM3/FRM4 二进制格式兼容（512 字节 header + mmap 持久化 + dirty tracking + incremental save）
  - **EnvelopeCache（3 层缓存管理）迁移**：
    - `CacheMetadata` 使用 `EndpointSource`（原 `FrameSourceMethod`）
    - `CacheLayer::CACHE_FRAMES` → `CACHE_ENDPOINTS`
    - `frames_path()` → `endpoints_path()`（磁盘文件名 "lect.frames" 保持兼容）
    - `frame_source_name()` → `endpoint_source_name()`
    - `load_metadata` 保留 `n_frames` 字段向后兼容读取
    - `compute_robot_hash`：FNV-1a 哈希（joints + limits + radii + active_link_map）
  - **编译**：MSVC 2022 Release，零错误零警告，sbf4.lib + 3 实验 + test_ifk_pipeline.exe
  - **测试**：172/172 全部通过（无回归）
- 影响范围：
  - API（affine_fk 从 stub → 完整实现、EndpointStore/EnvelopeCache 从 stub → 完整实现）
  - 持久化（mmap 二进制存储 + JSON 元数据缓存）
  - 术语（frame → endpoint 全面重命名，完成 MIGRATION_ENDPOINT_IAABB.md Phase H）
- 备注：
  - Phase A 全部完成（robot + interval_math + interval_fk + fk_scalar + affine_fk）
  - Phase B 已完成（envelope_derive）
  - Phase H 全部完成（mmap_util + EndpointStore + EnvelopeCache）
  - 后续推荐：Phase C（analytical_utils.h）→ Phase D（CritSample）→ Phase E（Analytical）→ Phase F（GCPC）→ Phase G（dispatcher 更新）
  - affine_fk 保持 `sbf::` 命名空间（与 v3 一致）
  - EndpointStore 保持 FRM3/FRM4 魔数用于磁盘格式兼容

### 2026-03-20

- 修改文件 / 模块：
  - `src/robot/fk_scalar.cpp` — 从 v3 迁移完整标量 FK 实现（dh_transform, fk_transforms, fk_transforms_inplace）
  - `include/sbf/robot/fk.h` — 新增 fk_transforms_inplace 声明
  - `include/sbf/envelope/analytical_utils.h` — 新建，~320 行共享工具函数/结构体
  - `include/sbf/envelope/analytical_solve.h` — 从 31 行 stub 替换为 ~140 行完整 header
  - `src/envelope/analytical_solve.cpp` — 从 8 行 stub 替换为 ~1,700 行完整解析管线
  - `include/sbf/envelope/endpoint_source.h` — EndpointIAABBResult 新增 analytical_link_aabbs 字段
  - `src/envelope/endpoint_source.cpp` — Analytical 分发路径实现 + extract_link_iaabbs 快速路径
  - `CMakeLists.txt` — 新增 test_analytical 测试目标
  - `tests/test_analytical.cpp` — 新建，10 个测试用例，24 个 CHECK 点
  - `doc/PLAN_ANALYTICAL_OPTIMIZED.md` — 执行计划文档
- 变更内容：
  - **Phase C+E 完整迁移：解析梯度零点枚举管线**
    - **fk_scalar.cpp**：从 stub（返回空/单位矩阵）迁移为完整实现
      - `dh_transform(alpha, a, d, theta)` → 4×4 Matrix4d
      - `fk_transforms_inplace(robot, q, buf)` → 栈分配 FK，供 FKWorkspace 使用
    - **analytical_utils.h**：提取共享工具到独立 header
      - `kShifts[3]`、`CRIT_MAX_COMBOS`、`HALF_PI`
      - `FKWorkspace`：栈分配 16×4×4 transform buffer，增量 `compute_joint()`/`compute_from()`
      - `crit_angles()`/`build_csets()`/`crit_enum()`/`crit_enum_fk()`：kπ/2 枚举
      - `LinkExtremes`：6 方向极值跟踪器（min/max x/y/z + 对应配置）
      - `eval_and_update()`/`eval_and_update_from()`：FK+更新复合操作
      - `FixedRoots`：固定容量（max 8）根存储
      - `solve_poly_in_interval()`：32 子区间 + 48 次二分，degree ≤ 8
      - `build_symbolic_poly8()`：Weierstrass 半角代换构建 degree-8 多项式
      - `half_angle_to_q()`/`build_bg_values()`
    - **analytical_solve.h**：完整 header
      - `AnalyticalCriticalConfig`：15 个 v3 配置项 + `enable_aa_pruning`（v4 新增）
      - `AnalyticalCriticalStats`：10 个计数器 + `n_aa_pruned_links`
      - 工厂方法：`all_enabled()`, `edges_only()`, `edges_and_faces()`, `v1_analytical()`
    - **analytical_solve.cpp**：6 阶段解析管线
      - Phase 0：kπ/2 顶点基线枚举
      - Phase 1：1D atan2 边界临界点（`solve_edges`）
      - Phase 2：2D degree-8 多项式面临界点（`solve_faces`）
      - Phase 2.5a：对约束 1D 求解（`solve_pair_constrained_1d`）
      - Phase 2.5b：对约束 2D 求解（`solve_pair_constrained_2d`）
      - Phase 3/3+：坐标下降内部优化（`solve_interior` / `solve_interior_improved`）
      - AA 间隙剪枝：`aa_link_can_improve()` 利用 AAFKWorkspace，跳过已收敛链接
      - `derive_aabb_critical_analytical()` + 带配置输出的 `_with_configs()` 变体
    - **endpoint_source 集成**：
      - Analytical 分发路径调用 `derive_aabb_critical_analytical()`
      - `extract_link_iaabbs()` 检测 analytical 结果并直接使用（绕过 endpoint→link 转换）
  - **编译**：MSVC 2022 Release，零错误，sbf4.lib + test_analytical.exe + test_ifk_pipeline.exe
  - **测试**：
    - test_analytical: 24/24 通过（crit_angles, FKWorkspace, 基本解析, vs iFK 对比, AA 剪枝, 配置变体, endpoint_source 分发, n_sub>1, with_configs, 全范围）
    - test_ifk_pipeline: 172/172 通过（无回归）
    - 解析 vs iFK 对比：解析总体积 1.042e-01 vs iFK 1.801e-01（3/4 链接更紧）
- 影响范围：
  - API（analytical_solve 从 stub → 完整实现、fk_scalar 从 stub → 完整实现）
  - endpoint_source 新增 Analytical 分发路径
  - 新增 AA gap 剪枝优化（v4 独创，v3 无此功能）
- 备注：
  - Phase C+E 全部完成（analytical_utils + analytical_solve + endpoint_source 集成）
  - AA 剪枝在窄区间（±0.05 rad）测试中暂无剪枝效果（IIWA14 4 个 active link 全部需要改进），预期在深层树节点中更有效
  - 后续推荐：Phase D（CritSample）→ Phase F（GCPC）→ Phase G（完整 dispatcher）

### 2026-03-20 — Analytical 求解器重复候选去重优化

- 修改文件 / 模块：
  - `src/envelope/analytical_solve.cpp`
  - `doc/CHANGE_LOG_CN.md`
- 变更内容：
  - 在不改变 6 阶段解析求解语义的前提下，对多个阶段的候选角评估流程增加“先收集、再去重、后评估”优化，减少重复 `eval_and_update_from()` 调用：
    - `solve_edges()`：对 distal/proximal 与 3 维坐标产生的候选角统一收集并按容差去重后再评估。
    - `solve_interior()`：对每个关节的 `q_star`/`q_star+π` 候选先去重再评估。
    - `solve_interior_improved()`：与基础 interior 同步采用去重评估策略。
    - `solve_pair_constrained_1d()`：对 `qi` 候选先去重，再映射 `qj = C - qi` 并进行区间校验后评估。
  - 优化目标为减少重复 FK 增量更新与极值写入，提升运行效率；不修改对外 API、配置项及输出格式。
- 影响范围：
  - 性能（减少重复 FK 调用）
  - 行为语义不变（解析阶段、统计口径与输出结构保持一致）
- 备注：

### 2026-04-02 — 2DOF 双面板 SBF 生长可视化模块 + GIF 输出

- 修改文件 / 模块：
  - `v4/viz/growth_viz.py`（新增）
  - `v4/doc/viz_scene04_all.py`（重写）
  - `v4/viz/__init__.py`（更新）
- 变更内容：
  - **`growth_viz.py`**：新增约 450 行可复用可视化模块，包含：
    - `build_random_obstacles_2d()`：随机矩形障碍物生成（3D AABB，带 z padding）
    - `fk_2dof()`：2DOF 正运动学 → (3,2) 关节位置
    - `scan_collision_map_2d()`：暴力扫描 C-space 碰撞热力图
    - `draw_boxes()` / `draw_arm()` / `draw_collision_bg()` 等绘图辅助函数
    - `dijkstra_box_path()`：box 级 Dijkstra 路径规划 + shortcut + pull-tight 平滑
    - `render_growth_gif()`：双面板生长动画（左 C-space boxes 逐步增长 + 右 Workspace 采样机械臂）
    - `render_execution_gif()`：双面板路径执行动画（左 C-space 路径高亮 + 右 Workspace 机械臂运动 + EE 轨迹）
    - 自适应边框宽度：窄 box 自动加粗边线确保可视性
  - **`viz_scene04_all.py`**：从 1139 行旧版全面重写为 ~170 行薄调用脚本：
    - 直接调用 C++ `_sbf4_cpp.ForestGrower` 生长 box forest
    - 使用 `Robot.from_json()` 加载 2DOF planar 机器人
    - 无固定 start/goal 策略：先生长森林 → BFS 找最大连通分量 → 选远距离 seed 作为端点
    - 配置：iFK + LinkIAABB + LectAligned + Wavefront
    - 输出：`safeboxforest/v4/results/viz_2dof_growth/` 下两个 GIF
  - **`viz/__init__.py`**：docstring 添加 `growth_viz` 条目
- 影响范围：
  - 新增可视化功能（无 API / 算法 / 构建影响）
  - 首次从 Python 脚本直接调用 C++ `_sbf4_cpp` 模块进行 2DOF 可视化
- 备注：
  - LECT 对 2DOF planar 机器人天然产生全高垂直条状 box（θ₂ 全范围），这是正确的区间 FK 行为
  - ForestGrower 对 2DOF planar + tool_frame 的碰撞检测较保守（3 link 端点），start/goal FFB 容易失败 → 采用"无端点生长 + 分量内选点"策略绕过
  - 最终结果：19 boxes / 3 components / 8-box path / 76 execution frames
  - 本次为“按文档进行优化”的增量实现，属于在既有 AA 剪枝基础上的求解器内部微优化。
  - 构建与验证：`test_analytical` 通过（PASS 24 / FAIL 0）。

### 2026-03-20 — Pair-Constrained 2D 阶段去重补全

- 修改文件 / 模块：
  - `src/envelope/analytical_solve.cpp`
  - `doc/CHANGE_LOG_CN.md`
- 变更内容：
  - 在 `solve_pair_constrained_2d()` 中补充候选三元组 `(qi, qj, qm)` 去重机制，避免不同 root 路径生成同一配置时重复调用 `eval_and_update_from()`。
  - 保持原有“每个 root 只接受首个有效候选”的行为语义，不改变求解阶段流程与输出结构。
- 影响范围：
  - 性能（减少重复 FK 评估）
  - 行为语义不变
- 备注：
  - 回归验证：`test_analytical`（PASS 24 / FAIL 0），`test_ifk_pipeline`（PASS 172 / FAIL 0）。

### 2026-03-20 — Analytical 去重 A/B 性能基准补充

- 修改文件 / 模块：
  - `include/sbf/envelope/analytical_solve.h`
  - `src/envelope/analytical_solve.cpp`
  - `experiments/exp_analytical_ab_benchmark.cpp`
  - `CMakeLists.txt`
  - `doc/CHANGE_LOG_CN.md`
- 变更内容：
  - 在 `AnalyticalCriticalConfig` 新增 `enable_candidate_dedup`（默认 `true`），用于控制解析阶段候选去重开关。
  - 将 `solve_edges`、`solve_interior`、`solve_interior_improved`、`solve_pair_constrained_1d` 与 `solve_pair_constrained_2d` 中的去重流程挂接到该开关，支持在同一代码路径下进行 A/B 对比（去重关/开）。
  - 新增实验程序 `exp_analytical_ab_benchmark`：对固定宽度区间集合执行多轮 A/B 计时，输出 `mean_ms`、总 FK 调用与分阶段 FK 调用（P1/P2/P2.5/P3），并在 stderr 输出 speedup 与 FK reduction。
  - 在 CMake 实验目标中注册 `exp_analytical_ab_benchmark`。
- 影响范围：
  - 性能评测能力（新增可复现 A/B 基准）
  - 行为语义不变（默认配置仍开启去重，与当前优化行为一致）
- 备注：
  - 该基准用于量化“候选去重”优化收益，便于后续继续优化时做回归对比。

### 2026-03-20 — 去重命中率统计与自适应去重阈值

- 修改文件 / 模块：
  - `include/sbf/envelope/analytical_solve.h`
  - `src/envelope/analytical_solve.cpp`
  - `experiments/exp_analytical_ab_benchmark.cpp`
  - `docs/API_REFERENCE_CN.md`
  - `doc/CHANGE_LOG_CN.md`
- 变更内容：
  - 在 `AnalyticalCriticalConfig` 新增自适应去重参数：`enable_adaptive_candidate_dedup`、`candidate_dedup_min_candidates`、`candidate_dedup_warmup_sets`、`candidate_dedup_min_hit_rate`。
  - 在 `AnalyticalCriticalStats` 新增 P1/P2.5/P3 三阶段去重统计：raw/unique candidates、applied/skipped sets。
  - 在 `solve_edges`、`solve_pair_constrained_1d`、`solve_pair_constrained_2d`、`solve_interior`、`solve_interior_improved` 中接入“命中率统计 + 自适应阈值”逻辑。
  - 扩展 A/B benchmark 输出列：新增三阶段去重命中率、应用次数、跳过次数，并将 B 模式切换为 `dedup_adaptive`。
- 影响范围：
  - 性能评测能力（可量化去重收益和命中情况）
  - 算法执行策略（可根据历史命中率动态跳过去重）
  - 默认行为兼容（自适应默认关闭，原配置语义保持）
- 备注：
  - 命中率定义：`(raw - unique) / raw`。

### 2026-03-21 — GCPC 模块迁移（v3 → v4）

- 修改文件 / 模块：
  - `include/sbf/envelope/gcpc.h`（33 行 stub → ~260 行完整头文件）
  - `src/envelope/gcpc.cpp`（21 行 stub → ~2360 行完整实现）
  - `include/sbf/envelope/analytical_utils.h`（FKWorkspace 新增 `compute_prefix()`）
- 变更内容：
  - 从 v3 完整迁移 GCPC（Globally Coupled Polynomial Criticality）模块。
  - 包含 `GcpcCache`、`GcpcQueryResult`、`query_link()` 等完整 API。
  - 在 `FKWorkspace` 中补充 `compute_prefix()` 方法，供 GCPC 及后续模块使用。
- 影响范围：
  - 新增 GCPC 缓存能力，为 CritSample 等模块提供区间内极值点查询。
- 备注：
  - 回归验证：`test_analytical`（PASS 38 / FAIL 0），`test_ifk_pipeline`（PASS 172 / FAIL 0）。

### 2026-03-21 — CritSample 迁移（全新 GCPC-based 设计）

- 修改文件 / 模块：
  - `include/sbf/envelope/crit_sample.h`（48 行 stub → ~95 行完整头文件）
  - `src/envelope/crit_sample.cpp`（11 行 stub → ~180 行完整实现）
  - `src/envelope/endpoint_source.cpp`（CritSample 分派从 iFK fallback 更新为实际实现）
- 变更内容：
  - **与 v3 设计不同**：v3 使用三阶段流水线（coupled enum + manifold sampling + L-BFGS-B），v4 采用全新的两阶段 GCPC-based 设计：
    - **Phase 1 — GCPC 极值点采样**：通过 `GcpcCache::query_link()` 查询每个活跃连杆在当前区间内的所有极值点，重建完整关节配置（q₀ optimal + q₁ reflected + ±π/2 变体），计算 FK 并更新 endpoint iAABBs。
    - **Phase 2 — 边界 + kπ/2 枚举**：对每个关节调用 `crit_angles()` 获取边界值与 kπ/2 组合，通过 `build_csets()` 构建笛卡尔积（受 `max_boundary_combos` 上限控制），使用 `crit_enum_fk()` 增量 FK 计算并更新 endpoint iAABBs。
  - 新增 `CriticalSamplingConfig`：`enable_gcpc`、`enable_boundary`、`max_boundary_combos`（默认 60000）、`gcpc_cache` 指针；工厂方法 `defaults()`、`gcpc_only()`、`boundary_only()`。
  - 新增 `CritSampleStats`：`n_gcpc_matches`、`n_gcpc_fk`、`n_boundary_combos`、`n_boundary_fk`、`phase1_ms`、`phase2_ms`。
  - 在 `endpoint_source.cpp` 中更新 `EndpointSource::CritSample` 分派逻辑，调用 `derive_crit_endpoints()` 并传播 `gcpc_cache`。
- 影响范围：
  - EndpointSource 分派：CritSample 路径从 iFK fallback 变为实际实现。
  - 依赖 GCPC 缓存（Phase 1）和 analytical_utils 共享工具（Phase 2）。
- 备注：
  - 回归验证：`test_analytical`（PASS 38 / FAIL 0），`test_ifk_pipeline`（PASS 172 / FAIL 0）。
  - GCPC endpoint_source 分派仍为 iFK fallback（待实现）。

### 2026-03-21 — CritSample + GCPC 测试创建 & API/论文文档

- 修改文件 / 模块：
  - `tests/test_crit_gcpc.cpp`（新建，~480 行）
  - `CMakeLists.txt`（新增 `test_crit_gcpc` 测试目标）
  - `docs/API_REFERENCE_CN.md`（全面扩充）
  - `doc/PAPER_STAGE1_ENDPOINT_IAABB.md`（新建）
  - `doc/CHANGE_LOG_CN.md`
- 变更内容：
  - **新建测试 `test_crit_gcpc`**（15 个测试函数，370 条断言）：
    - CritSample 测试（7 个）：boundary-only 有效性、config 工厂方法、iFK 包含性验证、宽度扩展特性、endpoint_source 分派、GCPC 禁用等价性、全范围无崩溃。
    - GCPC 测试（5 个）：缓存构建 + query_link、`enrich_with_interior_search`、`derive_aabb_with_gcpc` 有效性、iFK 包含性（3/4 link 更紧）、q₀ 重建正确性。
    - 集成测试（3 个）：CritSample + GCPC 缓存端到端、endpoint_source 分派带 GCPC、n_sub > 1。
  - **扩充 `API_REFERENCE_CN.md`**：
    - 新增 `endpoint_source` 模块文档（EndpointSource 枚举、EndpointSourceConfig、EndpointIAABBResult、compute_endpoint_iaabb 等）。
    - 新增 `crit_sample` 模块文档（两阶段设计、CriticalSamplingConfig、CritSampleStats、derive_crit_endpoints）。
    - 新增 `gcpc` 模块文档（七阶段流水线、GcpcPoint、GcpcQueryResult、GcpcQueryStats、GcpcCache 完整 API）。
    - 新增 `analytical_utils` 模块文档（FKWorkspace、crit_angles、build_csets、crit_enum_fk、LinkExtremes、FixedRoots、solve_poly_in_interval）。
    - 新增 `analytical_coeff` 模块文档（Coeff1D、Coeff2D、extract_1d/2d_coefficients、compute_middle_matrix、compute_suffix_pos）。
  - **新建 `PAPER_STAGE1_ENDPOINT_IAABB.md`**：
    - Stage 1 流水线完整算法描述，含 iFK / CritSample / Analytical / GCPC 四条路径。
    - CritSample 两阶段算法（Phase 1 GCPC 查询 + Phase 2 边界 kπ/2 枚举 + 增量 FK）。
    - Analytical 五阶段求解（P0–P3 + DH 系数直接提取零 FK 优化）。
    - GCPC 对称性约减 + 七阶段查询流水线。
    - 统一输出格式、质量排序、测试覆盖表。
- 影响范围：
  - 测试覆盖：新增 370 条断言覆盖 CritSample 和 GCPC 模块。
  - 文档完整性：API 参考文档从仅覆盖 analytical_solve 扩展至覆盖全部 Stage 1 模块。

---

## 2026-03-24 — LECT 新增 BEST_TIGHTEN 维度选择策略

- 修改文件 / 模块：
  - `include/sbf/core/config.h` — `SplitOrder` 枚举新增 `BEST_TIGHTEN = 3`
  - `include/sbf/forest/lect.h` — 新增 `set_split_order()`, `split_order()`,
    `pick_best_tighten_dim()` 方法
  - `src/forest/lect.cpp` — `init_split_dims()`, `split_leaf()`, `find_free_box()`,
    `pre_expand_recursive()` 中新增 BEST_TIGHTEN 分支；实现 `set_split_order()` +
    `pick_best_tighten_dim()`
  - `tests/test_ifk_pipeline.cpp` — 新增 `test_lect_best_tighten()` 测试
- 实现内容：
  - iFK 探针维度选择策略：对每个候选维度执行增量 FK + 提取 per-link iAABB，
    选择使 $\max(\text{vol}_L, \text{vol}_R)$ 最小的维度 (minimax 策略)
  - **按深度懒惰选维**：某个深度首次 `split_leaf` 时触发 iFK 探针，
    同深度所有节点复用同一维度
  - 当所有维度宽度≤0时 fallback 到 widest-first
- 影响：
  - API：新增 `SplitOrder::BEST_TIGHTEN` 枚举值和 `set_split_order()` 公共方法
  - 缓存：磁盘缓存不存储 `SplitOrder`，无兼容性影响
  - 行为语义：默认 `WIDEST_FIRST` 不变，仅显式设置 `BEST_TIGHTEN` 时生效

## 2026-03-23 — 默认管线 / 冻结实验 / 论文与 API 同步更新

- 修改文件 / 模块：
  - `include/sbf/envelope/pipeline.h`（默认管线工厂改为 GCPC 基础）
  - `include/sbf/core/config.h`（GCPC quality 默认 3，dual_phase3 默认 true）
  - `experiments/exp_freeze_sources.cpp`（新建，冻结×源对比实验）
  - `docs/API_REFERENCE_CN.md`（扩充 pipeline / interval_trig / config 章节）
  - `docs/PAPER_CODE_MAP.md`（新建，论文-代码追溯表）
  - `paper/root.tex`（英文论文：冻结章节 + 实验表格 + 质量排序修正）
  - `paper/root_cn.tex`（中文论文：冻结章节 + 实验表格 + 质量排序修正）
  - `doc/CHANGE_LOG_CN.md`
- 变更内容：
  - **默认管线配置**：
    - GCPC quality 默认值从 2 改为 3；dual_phase3 默认 true。
    - `PipelineConfig` 工厂方法 `recommended(cache)` / `tightest(cache)` / `production(cache)` 使用 GCPC 端点源；保留 `legacy_*()` 兼容接口。
  - **冻结×源对比实验** `exp_freeze_sources.cpp`：
    - 4 种源 × 3 种宽度 × 50 随机盒子，对比直接计算（k=0）与自适应冻结-重建。
    - 结果：iFK 冻结几乎无损（比值 0.8–1.1）；GCPC 窄宽度加速 31–200×，体积膨胀 1.4–3.0×。
  - **API_REFERENCE_CN.md 扩充**：
    - 新增 `envelope/pipeline` 章节（PipelineConfig、GCPC / legacy 工厂方法）。
    - 新增 `core/interval_trig` 章节（classify_alpha、FrozenJointDesc、reconstruct_cartesian_endpoints_multilevel）。
    - 新增 `core/config` 章节（FreezePolicy 结构体及工厂）。
  - **PAPER_CODE_MAP.md**（新建）：
    - Table 1 可追溯性条目；论文章节↔代码模块映射表（16 行）；8 项实验追溯条目。
  - **论文更新**（EN + CN 同步）：
    - 摘要：新增 (vi) 自适应冻结深度机制；贡献列表：新增冻结深度条目 + 更新实验条目。
    - 质量排序修正：{Analytical,GCPC}(2) → Analytical(2) < GCPC(3)，附说明。
    - 新增 §V.4 自适应冻结深度（两阶段流水线 + 自适应策略方程 + 体积-速度权衡）。
    - 新增 §VI.2 冻结与直接计算对比（Table 2：4 源 × 3 宽度，4 项观察）。
    - 结论：新增 (vii) 自适应冻结深度、(viii) 全面基准测试。
- 影响范围：
  - 默认管线：所有未指定 EndpointSource 的用户将自动使用 GCPC 管线（质量 3，严格紧于旧 Analytical 默认）。
  - 论文完整性：冻结深度从实现级别提升为论文正式章节，含理论描述 + 实验验证。
  - 文档同步：API 参考、论文-代码映射、变更日志均已更新。
- 备注：
  - 全量回归验证：`test_crit_gcpc`（PASS 370 / FAIL 0），`test_analytical`（PASS 38 / FAIL 0），`test_ifk_pipeline`（PASS 172 / FAIL 0）。

### 2026-03-22 — AABB_SUBDIV / GRID 碰撞策略移植 & 优化

- 修改文件 / 模块：
  - `include/sbf/envelope/collision_policy.h`（扩充，~100 行）
  - `src/envelope/collision_policy.cpp`（扩充，~210 行）
- 变更内容：
  - **新增 `collide_aabb_subdiv()` — 两阶段粗细碰撞检测**：
    - Stage 1：预计算的 link_iaabbs 作为粗筛 quick-reject，逐 (link, obstacle) 对做 3-axis SAT。
    - Stage 2：对粗筛碰撞的 (link, obs) 对，调用 `derive_aabb_subdivided()` 生成 n_sub 个子段 AABB，逐子段精细检测。
    - 支持固定 subdiv_n 和自适应模式（基于障碍物最小维度的 adaptive_subdivision_count）。
    - 栈缓冲 stack_sub[64×6]，避免堆分配（v3 行为保留）。
  - **新增 `collide_grid()` — GridStore 代理**：
    - 直接委托给 `GridStore::check_collision(node_idx, obstacles, n_obs)`。
    - v4 优化：用 GridStore word-level z-mask AND（1 条 uint64 指令 /（x,y）行），替代 v3 的 per-voxel uint8 循环。
    - GridStore 预填充（非运行时 derive_grid），消除在线体素化开销。
  - **更新 `collision_policy.h` 头文件**：
    - 新增 `collide_aabb_subdiv()` 和 `collide_grid()` 声明及完整文档注释。
    - `check_collision()` 统一分发器保持 AABB-only（SUBDIV/GRID 仍 fallback），调用方按需直接使用对应独立函数。
    - 前向声明 `class GridStore` 避免重型头文件依赖。
- v4 相对 v3 的优化点：
  1. **Obstacle.lo()/hi()** 替代 v3 的 interleaved obs_compact[lo_x,hi_x,lo_y,hi_y,lo_z,hi_z] 格式。
  2. **预计算障碍物浮点边界**：子段内循环使用 6 个 float（`aabb_vs_obs_bounds`），避免每次迭代重算 lo()/hi()。
  3. **GridStore word-level z-mask**：32 个 z 位压缩在 1 个 uint64 中，collision check 用 AND 一指令完成一行，v3 需逐体素 uint8 遍历。
  4. **GridStore 预填充**：v3 的 collide_grid 运行时调用 derive_grid 在线体素化，v4 在树构建阶段预填充，碰撞检测 O(1) 位探测。
  5. **独立函数 API**：SUBDIV/GRID 不再经统一分发器丢失所需参数，各自有完整签名，调用方按需选择。
- 影响范围：
  - `collision_policy` 模块：3 个碰撞策略均有完整实现（AABB / AABB_SUBDIV / GRID）。
  - `GridStore`：被 `collide_grid()` 直接引用（前向声明避免头文件耦合）。
  - `envelope_derive.h`：`derive_aabb_subdivided()` 和 `adaptive_subdivision_count()` 被 SUBDIV 策略调用。
- 备注：
  - 全量回归验证：`test_analytical`（PASS 38 / FAIL 0），`test_crit_gcpc`（PASS 370 / FAIL 0），`test_ifk_pipeline`（PASS 172 / FAIL 0），`test_kdop`（PASS 214 / FAIL 0）。总计 794 pass / 0 fail。
  - `check_collision()` 统一分发器中 SUBDIV/GRID 仍 fallback 到 AABB（设计意图：该接口签名不含 frames/GridStore，调用方应直接使用独立函数）。

---

### 2026-03-24 — 迁移 v3 LECT 相关代码进入 v4 (全量实现)

- 背景：v4 `LECT` 类已有核心管线（构造、compute_envelope、split_leaf、refine_aabb），但 FFB、碰撞查询、暖启动、持久化、协程移植等关键方法仍为存根（stub）。本次从 v3 迁移全部缺失代码并适配 v4 架构。
- NodeStore 增强（`node_store.h` / `node_store.cpp`）：
  - **`subtree_occ`** — 每节点子树占用计数器（`std::vector<int32_t>`），用于 FFB 早停判定。`alloc_node`、`ensure_capacity`、`clear_all_occupation` 均同步维护。
  - **`snapshot()`** — 深拷贝所有 SoA 数组，清除占用状态（`forest_id_` = -1, `subtree_occ_` = 0）。
  - **`copy_node_from()`** — 跨 NodeStore 拷贝单节点所有字段（含 `link_iaabbs_` 数据块）。
  - **`save()` / `load()`** — HCACHE02 二进制格式（4096 字节头 + SoA 顺序写入）。支持 `freeze_depth_`、`source_quality_` 的持久化。
- LECT 头文件扩展（`lect.h`）：
  - 新增公共方法：`hull_skip_vol()` / `set_hull_skip_vol()`、`transplant_subtree()`、`collides_scene()`、`intervals_collide_scene()`、`hull_collides_grid()`、`merged_children_hull_volume()` / `coarsen_volume_ratio()` / `merge_children_hulls()`、`count_nodes_with_iaabb()` / `count_nodes_with_hull()` / `total_hull_voxels()` / `scene_grid_voxels()`。
  - 新增私有方法：`derive_hull_grid()`、`pre_expand_recursive()`、`iaabbs_collide()`、`hull_collides()`、`propagate_up()`、`save_hull_grids()` / `load_hull_grids()`。
  - 新增常量：`HULL_HEADER_SIZE = 512`、`HULL_MAGIC = 0x314C5548`（"HUL1" LE）。
  - 新增 `#include <unordered_map>`、`#include "sbf/envelope/envelope_cache.h"`。
- LECT 实现全量填充（`lect.cpp`）：
  - **`set_scene()`** — 将障碍物光栅化为共享 VoxelGrid（`fill_aabb` 逐障碍物）。
  - **`find_free_box()`** — 完整 KD-tree 下降：占用检查 → iAABB 碰撞 → Hull-16 精炼 → 叶节点分裂 → 向种子点下降。支持 `subtree_occ` 早停和 `hull_skip_vol` 跳过。
  - **`mark_occupied()` / `unmark_occupied()`** — 通过 `subtree_occ` 向上传播占用计数器。
  - **`snapshot()`** — 深拷贝所有字段（`store_.snapshot()` + ep_store + hull_grids + node_split_dim 等）。
  - **`pre_expand()` / `pre_expand_recursive()`** — 递归预展开至目标深度，支持 WIDEST_FIRST 分裂序。
  - **`compute_all_hull_grids()`** — 批量为所有有 iAABB 但无 hull 的节点生成 hull grid。
  - **`derive_hull_grid()`** — 从 ep_store_ 重构笛卡尔端点坐标（含多级冻结旋转），对每条活动连杆调用 `fill_hull16(prox, dist, r)`，预计算 bounding box 并 `reserve_from_bounds`。
  - **`iaabbs_collide()` / `hull_collides()` / `hull_collides_grid()`** — 碰撞检测链：iAABB 粗筛 → hull-16 精炼 → 预光栅化场景网格快速路径。
  - **`propagate_up()`** — 沿 FFB 路径向上传播 `refine_aabb` 收紧父节点 iAABB。
  - **`collides_scene()`** — 两阶段碰撞：iAABB 早停 + 懒惰 hull-16 精炼（`const_cast` 缓存语义）。
  - **`intervals_collide_scene()`** — 任意 C-space 区间碰撞检测：`compute_endpoint_iaabb()` → `extract_link_iaabbs()` → iAABB 早停 → hull grid 精炼。
  - **`transplant_subtree()`** — BFS 遍历 worker 子树，`copy_node_from` 拷贝新节点，重映射 left/right/parent + forest_id，同步 ep_store / hull_grids / node_split_dim。
  - **`merged_children_hull_volume()` / `coarsen_volume_ratio()` / `merge_children_hulls()`** — hull 贪心粗化：子节点 hull 按位 OR 合并，零损失体积度量。
  - **`save()` / `load()`** — 完整持久化：`store_.save(hcache)` + `save_hull_grids(hulls)` + `EnvelopeCache::save_metadata(meta.json)`。`load` 为静态方法，自动恢复 freeze_depth、split_dims、root_fk、pipeline_config。
  - **`save_hull_grids()` / `load_hull_grids()`** — HUL1 格式：512 字节头 + 逐节点 {valid, n_bricks, [BrickCoord + BitBrick]...}。
  - **统计方法** — `count_nodes_with_iaabb()`、`count_nodes_with_hull()`、`total_hull_voxels()`、`scene_grid_voxels()`。
- v4 vs v3 架构差异适配：
  - `frame_store_` 不再使用，所有端点存储通过 `ep_store_`（本地帧 iAABB 向量）。
  - 命名体系：`link_aabbs` → `link_iaabbs`、`aabbs_collide` → `iaabbs_collide`、`get_link_aabbs` → `get_link_iaabbs`、`EndpointAABBResult` → `EndpointIAABBResult`、`compute_endpoint_aabb` → `compute_endpoint_iaabb`、`extract_link_aabbs_from_endpoint` → `extract_link_iaabbs`。
  - `SubAABB_Grid` → `LinkIAABB_Grid`（枚举名随 v4 体系更新）。
  - `freeze_depth_`（v3 固定=2）→ `max_freeze_depth_` + `effective_freeze_depth()` 自适应。
  - `node_split_dim_` 支持 `WIDEST_FIRST` 分裂序下的 `node_intervals()` 正确重构。
  - `load()` 由成员方法改为 `static LECT load(dir, robot)`（v4 惯例）。
- 验证：333 pass / 0 fail（`test_envelope_pipeline`），无编译警告。

---

### 2026-07-16 — 修复 KDSplit 零箱 + start_goal_connected=False 两大缺陷

**背景**：v3→v4 迁移 partition 模块后，测试发现两个严重问题：(1) KDSplit 模式产生 0 个 box，(2) 所有模式的 `start_goal_connected=False`。

**修复内容及修改文件**：

#### 修复 1：KDSplit 预展开 LECT

- **问题**：KDSplit 分支在 `select_roots()` 前未调用 `lect_.pre_expand()`，导致 FFB 在原始空 LECT 上执行，无法找到自由节点。LectAligned 模式已有预展开（`partition_lect_aligned()` 内调用）。
- **修改文件**：`src/forest/forest_grower.cpp` — `grow()` 中 KDSplit 分支
- **修改内容**：在 `select_roots()` 前添加 `lect_.pre_expand(wd)`，`wd` 默认取 `config_.warm_start_depth`（若为 0 则默认 3）。
- **效果**：KDSplit 模式从 0 box → 500+ box。

#### 修复 2：连通性多重缺陷

**2a — 单线程路径缺失 bridge 调用**
- **问题**：单线程扩展路径（`subtrees_.size() <= 1`）从未调用 `bridge_subtree_boundaries()`。
- **修改文件**：`src/forest/forest_grower.cpp` — `grow()` 单线程分支
- **修改内容**：在 `promote_all()` 之后、`graph_.rebuild()` 之前添加 `bridge_subtree_boundaries(obs, n_obs, result)`。

**2b — bridge_samples 默认值过低**
- **问题**：`bridge_samples` 默认为 0，导致 bridge 完全不执行。
- **修改文件**：`include/sbf/forest/grower_config.h`
- **修改内容**：`bridge_samples = 0` → `bridge_samples = 30`。

**2c — Bridge 种子改进（中点法）**
- **问题**：原始 bridge 使用单侧 `src->seed_config` 作为种子，远离目标子树边界。
- **修改文件**：`src/forest/forest_grower.cpp` — `bridge_subtree_boundaries()`
- **修改内容**：
  - 改用搜索跨边界最近邻对，取加权中点 `alpha * cfg_i + (1-alpha) * cfg_j`（alpha ∈ [0.3, 0.7]）。
  - 搜索半径从 `eps*10` 增大到 `max(eps*50, 0.5)`。
  - 最多处理 500 对，按距离排序优先处理最近的对。

**2d — 悬挂指针崩溃修复**
- **问题**：bridge 旧代码存储 `const BoxNode*` 指针，但 `try_create_box()` → `boxes_.push_back()` 可能触发 vector 重新分配，使所有指针失效。
- **修改文件**：`src/forest/forest_grower.cpp` — `bridge_subtree_boundaries()`
- **修改内容**：引入 `struct SeedInfo { Eigen::VectorXd cfg; }` 存储配置副本而非原始指针。`NearPair` 改用索引而非指针。

**2e — grow_parallel 端点扫描 root_id 过滤移除**
- **问题**：`grow_parallel()` 扫描 start/goal box 时要求 `root_id==0`/`root_id==1`，但 partition 模式按 cell 索引分配 root_id，不保证端点在 cell 0/1。
- **修改文件**：`src/forest/forest_grower.cpp` — `grow_parallel()`
- **修改内容**：移除 `root_id` 过滤条件，仅使用 `contains_q(box, start_config_)` 检查。

**2f — 扩展后端点箱创建 + S-G 连接器**
- **修改文件**：`src/forest/forest_grower.cpp` — `grow()` 后处理阶段
- **修改内容**：
  - 扩展完成后扫描所有 box 检查 start/goal 包含。
  - 若未找到，使用深层 FFB（`min_edge=1e-4, max_depth=50`）重试创建端点 box。
  - 若找到但未连通，沿 start→goal 直线生成 100 个插值种子，使用深层 FFB 创建连接 box。
- **补充修改**：`try_create_box()` 新增 `max_depth_override` 参数（`include/sbf/forest/forest_grower.h` 及 `.cpp`），允许端点重试使用更高深度。

**验证结果**（Easy 场景，Panda 7-DOF 机器人）：

| 模式 | Boxes | Roots | Bridge | 连通分量 | Connected | 时间 |
|------|-------|-------|--------|----------|-----------|------|
| LectAligned/4 | 587 | 4 | 52 | 14 | **True** | 1222ms |
| LectAligned/2 | 581 | 2 | 13 | 5 | **True** | 1120ms |
| KDSplit/4 | 552 | 1 | 0 | 1 | **True** | 946ms |
| KDSplit/2 | 552 | 1 | 0 | 1 | **True** | 947ms |
| Uniform/3 | 546 | 3 | 10 | 3 | **True** | 1104ms |
| Uniform/2 | 573 | 2 | 2 | 2 | **True** | 1092ms |

**所有 6 种模式均实现 `connected=True`。**

**影响范围**：
- API 不变（`GrowerConfig.bridge_samples` 默认值变化）
- `try_create_box()` 新增可选参数 `max_depth_override`，向后兼容
- 涉及文件：`forest_grower.cpp`、`forest_grower.h`、`grower_config.h`
- `doc/CHANGE_LOG_CN.md`

---

### 2026-03-26 — 迁移 v3 LECT 缓存加载功能至 v4

**背景**：v3 的 `grow()` 在 Phase 1（root selection）之前可从磁盘加载预构建的深层 LECT 缓存（`.hcache` 文件），跳过昂贵的 FK + envelope 在线计算。v4 的 `grower_config.h` 已有 `lect_cache_dir` 字段且 pybind 已绑定，但 `grow()` 中完全没有加载代码。

**修改内容**：

1. **`src/forest/forest_grower.cpp`**：
   - 添加 `#include <filesystem>` 头文件
   - 在 `grow()` 中、scene grid 设置之后、root selection 之前插入 LECT 缓存加载块（~25 行）：
     - 检查 `config_.lect_cache_dir` 非空 → 用 `std::filesystem::exists()` 检测 `.hcache` 文件
     - 调用 `lect_ = LECT::load(dir, *robot_)`（v4 静态工厂方法，区别于 v3 成员方法）
     - 重新应用 `hull_skip_vol`（缓存不持久化该值）
     - 重新设置 `set_scene()` 场景网格（`load()` 不保留场景数据）
     - 打印 `[grower] loaded LECT cache: N nodes, X ms` 或 `[grower] LECT cache not found: path`
     - 设置 `lect_cache_loaded_ = true`
   - KDSplit 分支的 `lect_.pre_expand()` 增加 `!lect_cache_loaded_` 守卫，避免缓存加载后冗余展开
   - `grow_parallel()` 中的 warm-start `lect_.pre_expand()` 同样增加 `!lect_cache_loaded_` 守卫

2. **`include/sbf/forest/forest_grower.h`**：
   - 新增成员变量 `bool lect_cache_loaded_ = false;`

**v3 → v4 适配要点**：
- v3 使用成员方法 `lect_.load(dir, robot)`；v4 使用静态工厂 `LECT::load(dir, robot)` 返回新对象，因此用 `lect_ = LECT::load(...)` 移动赋值
- v4 的 `lect_cache_loaded_` 作为成员变量而非局部变量，使 `grow_parallel()` 也能访问

**验证**：
- 编译零错误
- 无缓存运行正常（无回归）
- 错误路径正确输出 `LECT cache not found` 警告并继续执行
- `lect_cache_dir` 已通过 pybind 绑定（`sbf4_bindings.cpp` L137），Python 端可直接设置

**涉及文件**：
- `src/forest/forest_grower.cpp`
- `include/sbf/forest/forest_grower.h`
- `doc/CHANGE_LOG_CN.md`