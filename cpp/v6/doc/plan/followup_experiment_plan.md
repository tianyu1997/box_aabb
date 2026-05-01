# SafeBoxForest 后续实验与展示优化计划

日期：2026-05-02  
目标：补齐 SafeBoxForest（SBF）论文在图表标题、图注、算法伪代码框图和实验完整性上的最后缺口，为后续 TRO/IEEE RA-L/TRO 扩展版或补充材料准备可执行实验方案。

---

## 0. 总览结论

当前论文已经有 6 组核心实验：

1. endpoint interval envelope / epiAABB pipeline；
2. link-envelope pipeline；
3. matched-route cold/warm LECT replay；
4. shelf-scene combined workload 与 IRIS/PRM/BIT* 对比；
5. UR5/Panda cross-robot randomized scenes；
6. 单障碍插入后的 certificate invalidation 与 graph repair。

为了让工作更完整，建议新增或扩展以下实验，优先级从高到低：

| 优先级 | 实验 | 解决的审稿问题 | 建议位置 |
|---|---|---|---|
| P0 | E1 构建模块消融（build/grow/consolidation ablation） | 证明伪代码中每个模块必要；解释 build-time 来源 | 主文小表或补充材料主表 |
| P0 | E2 非 matched-route 的跨场景/跨进程 cache reuse | 证明 LECT 不只在相同路线 replay 中有效；回应 cache scope 限制 | 主文或补充材料重点实验 |
| P0 | E3 多查询 amortization / break-even | 证明 multi-query 是 SBF 的主要使用场景 | 主文图 |
| P0 | E4 动态场景更新扩展 | 把单障碍插入扩展到 add/remove/move/batch updates | 主文短表 + 补充细节 |
| P1 | E5 难度/窄通道压力测试 | 系统化解释 SBF、PRM、BIT*、IRIS 在 topology 上的差异 | 补充材料 |
| P1 | E6 path-quality / PathOpt 消融 | 量化 conservative box corridor 对路径长度的代价和后处理收益 | 补充材料或 discussion 表 |
| P1 | E7 并行扩展与确定性 | 证明 8-core 设置合理；说明 partitioned grower 不是默认原因 | 补充材料 |
| P1 | E8 cache 存储与 I/O profile | 给出 disk/RSS/serialization 成本，避免只报告时间 | 补充材料 |
| P2 | E9 soundness/certificate audit | 用密集碰撞检查证明 reported certificates 无 invalid segment | 补充材料 checklist |
| P2 | E10 baseline seed/hyperparameter sensitivity | 更完整解释 IRIS/PRM/BIT* 参数敏感性 | 补充材料 |

最建议进入主文的是 E1、E2、E3、E4。若页数紧张，E1 可以使用已有 `generated/tab_build_ablation.tex` 作为起点，E2/E3 用 1 张复合图，E4 保持短表。

---

## 1. 图表标题、图注、表注和伪代码框图优化计划

**执行状态（2026-05-02）：已完成第一阶段。** 已重写主文中的 workflow、2-link visualization、shelf baseline、shelf scene、cross-robot figure captions；已更新 epiAABB、link-envelope、shelf query、cross-robot、obstacle-update 表格标题和统计口径说明；已重写 `FindFreeBox`、`GrowForest`、`ConsolidateForest`、`PathOpt` 伪代码标题/输入输出/阶段注释，并新增 `IncrementalUpdate` 伪代码框图。`main.pdf` 重新编译成功，仍为 18 页，无 undefined citation/reference。

### 1.1 统一标题/图注原则

每个 caption 建议采用四段式信息结构：

1. **对象**：图/表显示什么；
2. **协议**：在哪个 experiment、scene、robot、seed/budget 下产生；
3. **统计口径**：是否 success-only、是否 matched-route replay、是否包含 query-time local processing；
4. **核心结论**：一句话指出读者应看什么。

避免 caption 中只写“comparison”或“results”。标题应能独立说明：数据来源、单位、是否 success-only、是否 cold/warm、是否 fixed budget。

### 1.2 当前图表的具体修改建议

| 对象 | 当前问题 | 建议标题/图注重点 |
|---|---|---|
| `fig:linkiaabb` workflow | caption 信息密度高但缺少“cache key / scene filter / query object”三层关系 | 标题改为 “SBF pipeline: kinematics-keyed envelope cache, scene-specific box filtering, and corridor query”。图注强调 LECT 是 robot-kinematics evidence，forest 是 obstacle-scene object，query 是 box-corridor object。 |
| `fig:overview` 2-link visualization | 子图 (a)-(f) 很有用，但 caption 可更明确 certificate invalidation/regrowth | 标题改为 “Illustrative SBF build, query, and obstacle-update cycle on a 2-link system”。注释中说明红色盒为被新障碍 invalidated 的 conservative boxes；regrowth/repair 不代表 full rebuild。 |
| `tab:epiaabb_pipeline` | endpoint-source 表应强调 upstream diagnostic，不是 planner leaderboard | 表注加入：每行统计 100 deterministic boxes/bin；IFK 体积越小越紧，Crit/Analytical/MC 更偏 reference coverage；非 planner SR。 |
| `tab:link_envelope_pipeline` | microkernel 与 cache-read 容易被误读 | 表注加入：`t_eval` 是 standalone endpoint+link envelope construction；`t_read` 是 retained cache-hit rows 中 envelope-expansion time / hit count；不含 grower-side parent-FK reuse。 |
| `tab:marcucci_envelope_build` | 目前标题过短 | 标题建议：“Matched-route cold/warm build replay on the shelf scene”。表注说明 empty tree、tree snapshot disabled、warm row requires zero EP/Grid misses。 |
| `fig:exp4_marcucci_baselines` | 需要清楚说明 BIT* 固定 10s 与 query time metric 不同 | caption 加：points are successful runs only；BIT* x-value denotes timeout budget, not measured reusable query latency。 |
| `fig:marcucci_scene` | 场景点位说明可以保留，但建议加“single shared 16-obstacle scene” | caption 加：all five queries share one reusable build。 |
| `fig:exp5_cross_robot_baselines` | cross-robot 易被解读为 universal transfer | caption 加：success-only timing/path; groups denote robot-difficulty scene families; topology-dependent transfer test。 |
| `tab:exp6_rebuild` | 应区分 invalidation/repair vs full regrowth | 表注加入：repair excludes full free-space coverage regrowth; measures surviving-box graph repair after one inserted obstacle。 |

### 1.3 伪代码框图修改建议

当前伪代码是正确的，但需要更像“可执行协议”，少一些抽象文字，多一些阶段注释和输入/输出。

#### Algorithm 1: `FindFreeBox`

建议标题：`FindFreeBox: lazy LECT descent with conservative scene validation`

建议加入三段注释：

- `// Cache materialization`：endpoint/link envelopes are materialized only if needed；
- `// Scene validation`：free/occupied decision is obstacle-scene dependent；
- `// Refinement`：split only when current node cannot certify free and depth budget remains。

建议明确输出：`validated interval I_v` 或 `failure`，不要同时称作 box `B` 和 interval `I_v` 而不解释。

#### Algorithm 2: `GrowForest`

建议标题：`GrowForest: budgeted face-adjacent conservative forest expansion`

建议把当前长行拆成 helper calls：

- `SelectFrontierComponent(F, priority)`；
- `SampleCandidate(T, F, mode)`；
- `ProjectToFace(q_cand, B_near)`；
- `AcceptIfFaceAdjacent(B_new, B_near)`；
- `UpdateComponents(F)`。

这样图注/伪代码能直接对应后续 E1 消融实验中的模块：frontier priority、unexplored sampling、face-adjacency accept、connector mode。

#### Algorithm 3: `ConsolidateForest`

建议标题：`ConsolidateForest: certificate-preserving promotion, merge, and pruning`

建议把三阶段写成显式 block：

1. `Promote collision-free sibling leaves`；
2. `Try conservative face/contact merges`；
3. `Prune boxes contained in accepted conservative regions`。

表注或算法注释中强调：每次 replace 都必须重新通过 chosen envelope representation 的 conservative collision-free check；pruning 只移除 union 中已经覆盖的 box，不缩小 certificate union。

#### Algorithm 4: `PathOpt`

当前算法缺少 `REQUIRE/ENSURE`。建议加入：

- `REQUIRE Corridor union U, initial waypoint chain <c1,...,cK> inside U`；
- `ENSURE Corridor-preserving waypoint chain inside U`。

并保留注释：waypoint elimination 与 midpoint polish 均限制在 `U` 内；任何 `U` 外局部连接不属于 conservative certificate，若使用必须单独计时和验证。

#### 建议新增一个伪代码框图：`IncrementalUpdate`

为 E4 动态更新实验服务，建议新增一个短算法：

- 输入：old forest, old graph, changed obstacle set, cached LECT；
- 标记与 changed obstacles 相交的 boxes；
- remove invalid boxes；
- repair adjacency graph among survivors；
- optional localized regrowth near cut frontiers；
- output repaired forest or failure。

这个算法可以把当前 Experiment 6 的文字描述变成方法部分可引用对象。

---

## 2. 全局实验运行设置

### 2.1 固定硬件/软件环境

所有新增实验应优先沿用论文已有环境，保证和当前表格可比：

- CPU：12th Gen Intel Core i5-12600KF；
- 内存：32 GB RAM；
- OS：Ubuntu 20.04（若在 Windows 复现实验，需要单独标注为 engineering validation，不与论文主表混合）；
- collision engine、robot model、compiler flags、OMPL/Drake version 固定并写入 run metadata；
- 所有实验禁用并发 experiment groups；
- 固定 CPU-core set 和 thread budget；默认 8 cores / 8 threads；
- 所有随机数使用显式 seed list，记录到每个结果 JSON/CSV 行。

### 2.2 统一 run metadata

每一次 trial 至少记录以下字段：

| 字段 | 含义 |
|---|---|
| `run_id` | 全局唯一 ID：experiment + robot + scene + seed + method + config |
| `timestamp` | 运行时间戳 |
| `git_commit` / artifact hash | 代码版本；若无 git，用 source archive hash |
| `robot` | IIWA14 / UR5 / Panda / others |
| `scene_id` | 场景 ID |
| `difficulty` | Easy / Medium / Hard / controlled density-clearance bin |
| `query_id` | query pair ID |
| `planner_seed` | planner-level random seed |
| `method` | SBF / PRM / BIT* / IRIS-NP+GCS / IRIS-ZO+GCS |
| `sbf_config` | envelope source, link representation, grower flags, coarsening flags |
| `cache_state` | cold / warm-matched / warm-cross-scene / no-cache |
| `time_budget_s` | method budget |
| `success` | solved and validated within budget |
| `failure_reason` | timeout / no corridor / invalid segment / baseline failure / internal error |
| `build_time_s` | defined below |
| `query_time_s` | defined below |
| `path_length_rad` | joint-space path length |
| `validated` | final returned path passed collision validation |
| `box_count`, `component_count` | SBF coverage statistics |
| `cache_hits_ep`, `cache_misses_ep`, `cache_hits_grid`, `cache_misses_grid` | cache counters |
| `disk_bytes`, `peak_rss_mb` | memory/storage profile |

### 2.3 时间统计口径

统一使用 wall-clock time，必要时同时记录 CPU time。所有计时都要用 monotonic clock。

#### SBF build time

包括：

- seed processing；
- LECT descent / lazy node refinement；
- endpoint/link envelope materialization；
- obstacle-scene validation；
- forest growth；
- consolidation；
- face-adjacency graph export；
- cache write/flush if the row claims persistent cache fill cost。

不包括：

- loading URDF/mesh assets；
- compiling kernels；
- plotting/figure generation；
- result JSON serialization unless explicitly measuring artifact overhead。

#### SBF query time

包括：

- start/goal association to forest boxes；
- graph search；
- waypoint reconstruction；
- `PathOpt`；
- any local endpoint completion or discrete validation if used。

必须单独记录：

- certified corridor-only query time；
- local endpoint completion / outside-union processing time；
- final validation time。

#### Baseline time

- PRM build：roadmap construction time；
- PRM query：second solve + simplification + final validation；
- BIT*：fixed 10 s query-budget method，没有 reusable build，报告 success/path/SR 和 budgeted runtime；
- IRIS-NP/IRIS-ZO build：region generation + GCS graph construction；
- IRIS query：GCS solve + edge validation + any charged local connector/validation。

### 2.4 成功率与路径统计口径

#### 成功率（SR）

\[
\mathrm{SR} = \frac{\#\{\text{solved and collision-validated trials}\}}{\#\{\text{all scheduled trials}\}} \times 100\%.
\]

- timeout、invalid path、内部异常、无 corridor 均计为 failure；
- SBF conservative certificate failure 与 “local completion failed” 需要分开记录，但 SR 只看最终 validated success；
- 置信区间使用 Wilson 95% CI。

#### 路径长度

- 默认 joint-space Euclidean arc length，单位 rad；
- path length 只在 successful validated runs 上报告；
- 跨方法比较建议增加 normalized path ratio：

\[
r_{m,i}=\frac{L_{m,i}}{\min_{m' \in \mathcal{M}_i} L_{m',i}},
\]

其中 \(\mathcal{M}_i\) 是同一 scene/query/seed 上成功的方法集合。

#### 时间统计

- 报告 median、IQR、mean、std；
- speedup 使用 paired trials：

\[
s_i = \frac{t_{\mathrm{cold},i}}{t_{\mathrm{warm},i}}.
\]

报告 median speedup、geometric mean speedup、bootstrap 95% CI。

#### Failure-aware 时间

为避免 success-only time 造成偏差，建议额外报告 PAR10：

- success：使用实际 runtime；
- failure/timeout：记为 \(10\times\) time budget；
- 用于补充材料，不一定放主文。

### 2.5 cache 统计口径

对 LECT 相关实验统一记录：

- EP hit rate：`cache_hits_ep / (cache_hits_ep + cache_misses_ep)`；
- Grid hit rate：`cache_hits_grid / (cache_hits_grid + cache_misses_grid)`；
- warm build speedup：paired cold/warm run；
- cache fill cost：cold run 中写入 cache 的额外耗时；
- cache read cost：warm run 中读 endpoint/grid evidence 的耗时；
- cache disk footprint：run 后 `.lect` / payload 文件总 bytes；
- cache reuse distance：warm cache 来自 same route / same scene different queries / previous scene / previous process。

---

## 3. E1：构建模块消融实验（Build/Grow/Consolidation Ablation）

### 3.1 目的

证明方法部分和伪代码中每个模块的贡献：seed-guided descent、frontier growth、bridge/connector strategy、unexplored sampling、consolidation、partitioned grower 等。该实验直接支撑伪代码框图优化。

### 3.2 实验对象

- Robot：IIWA14 shelf scene 为主；UR5-Medium 和 Panda-Hard 可作为补充。
- Scene：
  - shelf 16-obstacle combined scene；
  - cross-robot randomized Medium/Hard 各 5 scenes。
- Query：沿用当前 canonical query pairs；随机场景使用每 scene 5 query pairs。
- Seeds：每 scene 5 planner seeds；shelf 建议 10 seeds（如果算力允许）。

### 3.3 Config matrix

以当前 paper SBF config 为 baseline，做 one-factor-at-a-time 消融：

| Config | 改动 | 目的 |
|---|---|---|
| Baseline | 当前默认配置 | 对照 |
| No consolidation | 关闭 promotion/coarsening/pruning | 量化 consolidation 对 box count/query time/path 的影响 |
| No unexplored sampling | `p_u=0` | 检验 free-volume-weighted LECT walk 的覆盖收益 |
| No frontier priority | frontier uniform selection | 检验 coordinated multi-tree priority |
| No connector mode | 关闭 connector-biased sampling | 检验 disconnected components 的恢复能力 |
| Lower depth cap | `D_max` 降一档 | 检验粗 box certificate 成功率/保守性 |
| Higher depth cap | `D_max` 升一档 | 检验更细 boxes 对 SR/path/build 的权衡 |
| Partitioned grower | 使用 experimental partitioned LECT grower | 解释为什么不是默认配置 |
| LinkIAABB vs HullGrid | 保持 grower，替换 envelope representation | 连接 E2 cache 结论 |

已有 `generated/tab_build_ablation.tex` 可作为初始版本，但建议重新运行并去掉容易引起误解的列名，如 `Seed-brg`、`Brg+C2`，改成 reviewer-friendly 名称。

### 3.4 统计指标

- build time：total、grow、consolidation、graph export、cache read/write；
- query time：median over successful queries；
- SR；
- path length；
- box count；
- component count / adjacency-island count；
- average box edge length；
- number of conservative collision checks；
- invalidated or rejected candidates；
- peak RSS。

### 3.5 输出表/图

- 主表：config × build/query/SR/path/boxes/components；
- 附图：build-time stacked bar（grow vs validation vs consolidation）；
- 附图：box count vs path length / query time scatter。

### 3.6 接受标准

实验应能回答：

1. Baseline 为什么是默认配置；
2. consolidation 是否减少 boxes/query time 且不破坏 SR；
3. partitioned grower 虽然某些子计时快，但整体 wall-clock 或 merge cost 是否不适合当前默认；
4. depth cap 和 envelope representation 的权衡是否稳定。

---

## 4. E2：跨场景/跨进程 LECT cache reuse 实验

### 4.1 目的

当前论文已经谨慎说明 cold/warm 是 matched-route replay，不是 deployment-scale cache study。为了让 LECT claim 更完整，需要补一个真正的 cross-scene/cross-process cache reuse 实验。

### 4.2 实验设计

固定 robot kinematics，改变 obstacle scenes 和 query endpoints。

- Robots：IIWA14、UR5、Panda；
- Scene families：shelf-like blocked scenes、random AABB scenes、narrow-passage scenes；
- Difficulty：Easy/Medium/Hard；
- 每个 robot-difficulty：20 train scenes + 20 test scenes；
- 每 scene：5 query pairs；
- 每 query：5 planner seeds；
- Scene order：5 random permutations，用于评估 cache warm-up 顺序影响。

### 4.3 Cache conditions

| Condition | 定义 |
|---|---|
| NoCache | 禁用 persistent LECT，每次在线重算 endpoint/link evidence |
| ColdPersistent | 空 cache 开始，build 时填充并计入写入成本 |
| WarmSameSceneDifferentQuery | 同一 scene 的前若干 query/build 产生 cache，测试不同 query |
| WarmCrossScene | train scenes 预热 cache，test scenes 不共享路线，只共享 kinematic boxes/evidence |
| WarmCrossProcess | 终止进程后重新加载 cache，再跑 held-out scenes |
| TreeSnapshotWarm | 允许 tree snapshot；单独报告，不与 envelope-only warm 混合 |

### 4.4 Envelope payloads

至少比较：

- IFK + LinkIAABB(S=4)：预期 cache read 未必快；
- CritSample + LinkIAABB(S=4)：中等构造成本；
- IFK + HullGrid(δ=0.04)：grid payload 重用；
- CritSample + HullGrid(δ=0.04)：最可能显示持久化收益。

### 4.5 统计指标

- cold build time；
- warm build time；
- paired speedup；
- EP hit/miss rate；
- Grid hit/miss rate；
- cache read time / hit；
- cache write time；
- cache disk footprint；
- peak RSS；
- number of reused nodes by depth；
- route overlap diagnostic：test route boxes 与 train boxes 的 Jaccard/containment estimate。

### 4.6 关键统计口径

- WarmCrossScene 不能使用 same route；测试 scenes 必须 held-out；
- 所有 speedup 必须 paired by robot/scene/query/seed/config；
- 同时报告 success-only speedup 和 all-scheduled PAR10 speedup；
- 若 cache 使某些 run 成功/失败不同，时间 speedup 只在 paired-success 子集上报告，SR 另报。

### 4.7 输出图/表

- 主图：cache condition × build time violin/boxplot；
- 主表：payload × condition × hit rate × speedup × disk MB；
- 辅图：cache hit rate vs scene index，展示 warm-up 曲线；
- 辅图：break-even number of future builds：

\[
N_{\mathrm{break-even}} = \frac{T_{\mathrm{fill}} - T_{\mathrm{nocache,cold}}}{T_{\mathrm{nocache,repeat}} - T_{\mathrm{warm}}}.
\]

---

## 5. E3：多查询 amortization / break-even 实验

### 5.1 目的

SBF 的核心定位是 multi-query manipulation。需要展示当 query batch 变大时，一次 reusable build 如何摊薄成本，并和 PRM、IRIS+GCS、BIT* 比较。

### 5.2 实验设置

- Robots：IIWA14 shelf scene；Panda-Medium；UR5-Medium；
- 每个 scene 生成 query pool：100 start-goal pairs；
- Batch sizes：1、2、5、10、25、50、100；
- 每个 batch size：5 scene seeds × 5 query-sampling seeds；
- SBF：每 scene build 一次，然后 query batch；
- PRM：每 scene build roadmap 一次，然后 query batch；
- IRIS-NP+GCS：每 scene build regions/GCS graph 一次，然后 query batch；
- BIT*：每 query 单独 10 s budget，无 reusable build。

### 5.3 指标

- total time for batch：

\[
T_{\mathrm{batch}}(K)=T_{\mathrm{build}}+\sum_{j=1}^{K}T_{\mathrm{query},j}.
\]

- amortized time per query：

\[
\bar{T}_{\mathrm{query}}(K)=T_{\mathrm{batch}}(K)/K.
\]

- SR over all scheduled queries；
- path length success-only；
- number of successful queries per batch；
- break-even K where SBF total time becomes lower than PRM/IRIS/BIT*。

### 5.4 输出图

- line plot：batch size K vs amortized time/query；
- line plot：batch size K vs cumulative successes under wall-clock budget；
- table：break-even K for each robot/difficulty。

### 5.5 口径注意

- 如果 method build fails，batch 内所有 queries 计为 failure；
- 对 PRM/IRIS，如果 build 成功但 query 失败，按 query-level failure 记录；
- 对 BIT*，没有 build，total time 是每 query budgeted runtime 之和。

---

## 6. E4：动态场景更新扩展实验

### 6.1 目的

当前 Experiment 6 只测 “add one AABB, invalidate boxes, repair graph”。建议扩展为 add/remove/move/batch updates，证明 explicit box forest 对 dynamic scene 有系统价值。

### 6.2 更新类型

| Update type | 说明 |
|---|---|
| Add-1 | 插入一个 workspace AABB，沿用当前实验 |
| Remove-1 | 删除一个已有 AABB；允许 graph repair，但不自动声明新 free volume fully covered |
| Move-1 | 随机移动一个 AABB；等价于 remove old + add new，但记录为 single scene edit |
| Add-k | 一次加入 k 个障碍，k ∈ {2, 4, 8} |
| Sequential edits | 连续 10 次 add/remove/move，评估累计退化 |
| Localized regrowth | 在 invalidated frontier 附近有限预算 regrow；与 graph-only repair 对比 |
| Full rebuild | 从头 rebuild，作为上界/对照 |

### 6.3 场景与 seeds

- Robots：UR5-Medium、Panda-Medium、IIWA shelf；
- 每 robot-group：10 scenes；
- 每 scene：5 initial planner seeds；
- 每 initial forest：10 update events；
- 每 update event：保留同一 query set，另加 5 new queries。

### 6.4 指标

- initial build time；
- update repair time；
- full rebuild time；
- speedup：full rebuild / incremental update；
- invalidated box fraction；
- surviving box fraction；
- component count before/after；
- SR before/after；
- path length before/after；
- localized regrowth boxes added；
- failed repair reasons。

### 6.5 口径

- Remove-1 不应把新可行空间自动计入 coverage，除非运行 localized regrowth；
- Graph-only repair 只保证 surviving boxes 的 certificate；
- Localized regrowth 必须重新做 conservative envelope check；
- Sequential edits 每一步都记录 cumulative cache/forest state。

### 6.6 输出

- 表：update type × repair time × full rebuild speedup × SR delta；
- 图：invalidated fraction vs repair time；
- 图：sequential edits 中 SR/box count/repair time 的变化曲线。

---

## 7. E5：难度/窄通道压力测试

### 7.1 目的

把目前 randomized AABB scene 的 Easy/Medium/Hard 进一步参数化，解释 SBF 在 topology 和 clearance 上的边界条件。

### 7.2 Controlled scene generator

为每个 robot 生成参数化 obstacle fields：

| 因子 | 取值 |
|---|---|
| obstacle count | 8、16、32 |
| obstacle size scale | small、medium、large |
| clearance regime | wide、medium、narrow |
| bottleneck count | 0、1、2 |
| start-goal separation | short、medium、long |

为避免组合爆炸，使用 Latin hypercube / stratified sampling，每 robot 生成：

- 60 scenes：20 wide、20 medium、20 narrow；
- 每 scene 5 query pairs；
- 每 query 5 planner seeds。

### 7.3 Methods

- SBF default；
- SBF with higher depth cap；
- PRM 10s build + 2s query；
- PRM 30s build 作为 stronger roadmap control；
- BIT* 10s；
- BIT* 30s 作为 budget sensitivity；
- IRIS-NP+GCS retained config；
- IRIS-ZO+GCS optional。

### 7.4 指标

- SR with Wilson CI；
- build/query/PAR10；
- path length ratio；
- minimum sampled clearance along validated path；
- SBF box count and average edge length；
- rejected candidate ratio；
- IRIS region count and GCS solve status；
- PRM roadmap size and connected components。

### 7.5 输出

- heatmap：clearance regime × obstacle count 的 SR；
- scatter：coverage proxy / box count vs SR；
- per-method failure taxonomy。

---

## 8. E6：路径质量与 corridor post-processing 消融

### 8.1 目的

论文已经承认 SBF path 较长。该实验量化 conservative corridor 代价，并展示 `PathOpt` 是否以很小成本改善路径。

### 8.2 Configs

| Config | 说明 |
|---|---|
| Raw corridor centers | graph path 的 box centers，无 post-processing |
| Waypoint elimination only | 删除可由 corridor segment 直接连接的中间点 |
| Midpoint polish | 当前 `PathOpt` 完整版本 |
| More iterations | 增加 2/5/10 次 corridor-constrained smoothing |
| Local connector enabled | 若使用 union 外处理，单独计时并单独标注 non-certificate extension |

### 8.3 指标

- path length；
- number of waypoints；
- query post-processing time；
- certified-inside-union fraction；
- final dense validation result；
- path length ratio to PRM/BIT* best success；
- clearance margin distribution。

### 8.4 输出

- 表：post-processing config × path length × query time × waypoint count；
- 图：path length improvement vs added query time。

---

## 9. E7：并行扩展、确定性和 partitioned grower 实验

### 9.1 目的

证明当前 8-core/8-thread default 的合理性，并解释 partitioned grower/legacy parallel 为什么未作为默认主表配置。

### 9.2 设置

- Robots/scenes：IIWA shelf、Panda-Medium、UR5-Hard；
- Thread count：1、2、4、8、12、16；
- Grower：default coordinated grower、partitioned grower、legacy parallel；
- Seeds：每组 10 seeds。

### 9.3 指标

- total build time；
- grow time；
- merge/consolidation time；
- lock/wait time if available；
- speedup vs 1 thread；
- parallel efficiency：

\[
E_p = \frac{T_1}{pT_p}.
\]

- SR；
- box count variance；
- path variance；
- deterministic replay hash：同 seed 是否产生同 forest graph signature。

### 9.4 输出

- speedup curve；
- stacked timing by grower type；
- determinism table。

---

## 10. E8：cache 存储、内存和 I/O profile

### 10.1 目的

LECT 的代价不仅是 time，也包括 disk/RSS/I/O。该实验为 artifact 和 deployment readiness 提供数据。

### 10.2 设置

基于 E2 的 cache conditions，额外记录：

- disk bytes by payload type；
- number of cache nodes by depth；
- serialization time；
- deserialization time；
- write amplification；
- peak RSS；
- steady-state resident hot nodes；
- cache cleanup/compaction optional。

### 10.3 输出

- 表：payload × scenes processed × disk MB × read/write time；
- 图：cache size vs speedup；
- 图：node depth histogram。

---

## 11. E9：soundness / certificate audit

### 11.1 目的

给审稿人一个清晰的安全检查口径：SBF 返回的 certified corridor 部分必须没有碰撞。该实验不是替代理论证明，而是验证实现没有 bug。

### 11.2 设置

对所有主实验的 successful SBF paths：

- 在 joint-space path 上按固定 step size 采样；
- 使用 independent FCL collision checker 复核；
- 记录 certified corridor segment 与 local non-certificate processing segment；
- 对每个 returned box，抽样若干 configurations 做 sanity check；
- 对 boundary cases 加密采样。

### 11.3 指标

- invalid certified segment count：目标为 0；
- invalid local completion segment count：若非零，必须计入 failure 或单独报告；
- minimum clearance distribution；
- number of boxes audited；
- sample density。

### 11.4 输出

- 简短表：experiment × audited paths × sampled states × invalid certified segments；
- 目标结论：certified corridor audit found 0 invalid certified segments under dense validation。

---

## 12. E10：baseline seed/hyperparameter sensitivity

### 12.1 目的

当前论文已经谨慎说明 IRIS seed/coverage sensitivity。为完整性，可补充 sensitivity sweep，避免只比较单个 tuned setting。

### 12.2 IRIS-NP/IRIS-ZO factors

- number of region seeds：5、10、25、50；
- seed source：random collision-free、query-biased、PRM waypoint-biased、straight-line filtered；
- C-space margin：`1e-3`、`1e-4`、`1e-5`；
- GCS time budget：2s、5s、10s、30s。

### 12.3 PRM/BIT* factors

- PRM build budget：5s、10s、30s；
- PRM simplification budget：0.5s、2s、5s；
- BIT* query budget：5s、10s、30s；
- BIT* batch size / rewiring factor sweep。

### 12.4 统计

- 同一 scene/query/seed 上 paired comparison；
- report best validated config 但必须把 selection rule 写清楚；
- supplement 展示 sensitivity curve，主文保留 retained best config。

---

## 13. 推荐的主文新增图表组合

若只能加入 2 页左右，建议：

1. **Table A：Build component ablation**
   - 来自 E1；
   - columns：Config, Build, Query, SR, Path, Boxes, Components；
   - 解释 GrowForest/ConsolidateForest/PathOpt 伪代码必要性。

2. **Figure B：Cache reuse beyond matched replay**
   - 来自 E2；
   - 左：NoCache/ColdPersistent/WarmCrossScene/WarmCrossProcess build time；
   - 右：EP/Grid hit rate and disk footprint；
   - 结论：grid payloads benefit; simple IFK+AABB remains online。

3. **Figure C：Multi-query amortization**
   - 来自 E3；
   - batch size vs amortized time/query；
   - 标出 break-even K。

4. **Table D：Dynamic update repair**
   - 来自 E4；
   - update type vs repair speedup/SR delta/invalidated fraction。

其余 E5--E10 放补充材料。

---

## 14. 结果文件与生成物约定

建议新增结果目录：

```text
experiments/results_followup/
  exp_e1_build_ablation.json
  exp_e2_cache_cross_scene.json
  exp_e3_query_batch.json
  exp_e4_dynamic_updates.json
  exp_e5_topology_stress.json
  exp_e6_pathopt_ablation.json
  exp_e7_parallel_scaling.json
  exp_e8_cache_storage.json
  exp_e9_soundness_audit.json
  exp_e10_baseline_sensitivity.json

generated_followup/
  tab_build_ablation_v2.tex
  tab_cache_cross_scene.tex
  fig_cache_cross_scene.pdf
  fig_query_batch_amortization.pdf
  tab_dynamic_update.tex
  tab_soundness_audit.tex
```

每个 JSON 顶层建议包含：

- `metadata`：commit/hash、hardware、software versions、date、command-line flags；
- `schema_version`；
- `runs`：一行一个 trial；
- `aggregates`：由 analysis script 产生，不手写。

---

## 15. 最终验证 checklist

在把新增实验写入论文前，逐项检查：

1. 所有 methods 使用相同 scene/query/seed split；
2. success-only metrics 和 all-trial metrics 分开；
3. build/query/local validation/cache read/write 计时边界一致；
4. SBF certificate claim 只覆盖 validated box union 内 corridor；
5. local endpoint completion 若出现，单独计时、单独验证、单独说明；
6. cold/warm cache 明确区分 matched-route、same-scene-different-query、cross-scene、cross-process；
7. speedup 只用 paired trials；
8. SR 带 Wilson 95% CI；
9. 时间/path 带 median/IQR 和 bootstrap 95% CI；
10. 主文图表 caption 能独立说明实验协议和统计口径；
11. 伪代码框图中的每个模块都能在 E1/E4 中找到对应消融或计时；
12. `main.pdf` 构建前搜索 `TODO/TBD/Planned slot` 等占位文本。

---

## 16. 建议执行顺序

1. 先做图表标题/表注/伪代码框图重写，不需要新实验；
2. 复跑 E1 build ablation，并把现有 `tab_build_ablation` 升级为 reviewer-friendly 表；
3. 跑 E3 multi-query amortization，因为它最直接支撑论文动机；
4. 跑 E2 cross-scene cache reuse，补齐 LECT 最大实验缺口；
5. 扩展 E4 dynamic updates；
6. 根据页数决定是否把 E5--E10 放补充材料。

推荐先实现 E1/E3/E2/E4。完成这四组后，论文会从“方法和主要 benchmark 已成立”提升到“reuse、multi-query、dynamic update、module necessity 都有系统证据支撑”。
