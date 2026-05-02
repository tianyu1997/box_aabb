# SafeBoxForest 后续实验报告

日期：2026-05-02

本报告整理 `experiments/results_followup/` 与 `experiments/results_followup/heavy_matrices/` 中当前可用的 E1-E10 后续实验结果。报告口径如下：

- E1、E3、E8 来自 `12_followup_experiments.py` 对既有 paper JSON 的 retained/existing 汇总；E1 已把 raw 中已有的 `No unexplored sampling`、`ffb_depth_80`、`ffb_depth_200` 合并进 canonical summary。
- E2、E4、E5、E6 来自 `13_followup_heavy_matrices.py --full --experiments e2,e4,e5,e6,...` 的前半段有效输出。该次 full run 在进入 E7 时因 `exp6_build_timing` 工作目录不匹配中断，但 E2-E6 已经写出完整 JSON。
- E7、E9、E10 来自后续成功运行的 `13_followup_heavy_matrices.py --full --experiments e7,e9,e10`，退出码为 0。
- 结果 commit 记录为 `fbbba72`。heavy runs 使用 8 线程 paper affinity `0-7`，E7 内部线程数 sweep 另行指定。

## 1. 总览

| 实验 | 当前状态 | 主要产物 | 运行量 | 阻塞项 |
|---|---|---|---:|---:|
| E1 Build ablation | retained measured + optional Hull16 gap | `exp_e1_build_ablation.json` | 9 configs | 1 optional config |
| E2 Cache reuse | heavy measured | `heavy_matrices/exp_e2_cache_cross_scene_heavy.json` | 792 runs, 96 aggregates | 3 |
| E3 Query amortization | retained measured | `exp_e3_query_batch.json` | 4 methods x 7 batch sizes | 0 |
| E4 Dynamic updates | heavy measured existing Add-1 | `heavy_matrices/exp_e4_dynamic_updates_heavy.json` | 150 runs | 4 |
| E5 Topology stress | heavy retained full Exp.5 aggregation | `heavy_matrices/exp_e5_topology_stress_heavy.json` | 30 scenes | 1 |
| E6 PathOpt ablation | heavy measured | `heavy_matrices/exp_e6_pathopt_ablation_heavy.json` | 450 runs, 18 aggregates | 2 |
| E7 Parallel scaling | heavy measured | `heavy_matrices/exp_e7_parallel_scaling_heavy.json` | 18 runs | 0 |
| E8 Cache storage | retained measured | `exp_e8_cache_storage.json` | 14 rows | 0 |
| E9 Soundness audit | heavy measured | `heavy_matrices/exp_e9_soundness_audit_heavy.json` | 262 audited paths | 0 |
| E10 Baseline sensitivity | heavy retained + smoke | `heavy_matrices/exp_e10_baseline_sensitivity_heavy.json` | 15 rows | 1 |

整体上，E2/E6/E7/E9 给出了新的可用实测结果；E4/E5/E10 主要是把既有 full paper 数据整理成 follow-up schema，并显式标记未实现的 protocol rows。E1 的三个必补缺口现在已补齐，剩余 Hull16-Grid 不建议在 E1 中单独补跑，理由见 E2/E8 的 cache/storage tradeoff。最需要谨慎处理的是 E9：当前独立密集碰撞审计发现了非零 invalid certified segments，不能写成“0 invalid segment”的安全结论。

## 2. E1 构建模块消融

**目的。** 解释 `GrowForest`、connector、consolidation、parallel grower 等模块对 build/query/box count 的影响，支撑伪代码和方法部分的必要性。

**当前数据。** 目前是 retained/existing 汇总，不是新的 full matrix。原先缺失的 `No unexplored sampling`、`ffb_depth_80`、`ffb_depth_200` 已由 `build_ablation_sweep/raw/` 中已有 JSON 重新汇总进 `build_ablation_sweep.json` 和 `exp_e1_build_ablation.json`。9 个 configs 中 8 个已有 measured rows，仅 Hull16-Grid 作为 optional gap 保留。

| Config | 状态 | Build ms | Query s | SR | Boxes | Components | 主要观察 |
|---|---|---:|---:|---:|---:|---:|---|
| Baseline | measured | 1612 | 0.856 | 1.00 | 1036 | 3 | 当前主配置对照。 |
| No consolidation | measured | 1170 | 0.533 | 1.00 | 1803 | 3 | 更快但 boxes 更多，说明 consolidation 带来压缩而非单纯提速。 |
| No unexplored sampling | measured | 508 | 0.235 | 1.00 | 1345 | 3 | 该 workload 下不依赖 unexplored sampling 也能全成功，且 build 明显更快。 |
| Connector stress | measured | 3446 | 0.560 | 1.00 | 3218 | 3 | connector-heavy 设置 build 代价高，但 path 略短。 |
| No coordinated frontier priority | measured | 4463 | 0.684 | 1.00 | 4430 | 3 | legacy independent grower 明显更重，不适合作默认。 |
| Partitioned grower | measured | 4474 | 0.562 | 1.00 | 3717 | 3 | partitioned 也偏重，当前不支持替代 default。 |
| Lower depth cap | measured | 373 | 0.243 | 1.00 | 1222 | 3 | `ffb_depth=80` 是当前表中最快配置，未损失 SR/path。 |
| Higher depth cap | measured | 620 | 0.236 | 1.00 | 1275 | 3 | `ffb_depth=200` 比 baseline 快，但慢于 depth 80；path 略短。 |
| Hull16-Grid envelope | not measured | - | - | - | - | - | 本轮不独立补跑；E2/E8 已显示 Grid cache 有速度收益但磁盘代价高。 |

**结论。** E1 三个必补缺口已补齐。当前 shelf combined workload 下，`ffb_depth=80`、`No unexplored sampling`、`ffb_depth=200` 都保持 100% SR，且 build time 分别为 373 ms、508 ms、620 ms，均低于 baseline 的 1612 ms。该结果说明 baseline 配置更保守，并非该单一 workload 的最快配置；写作时应把 E1 表述为“模块敏感性/保守默认解释”，而不是把每个 ablation 的更快结果解释成全局替代默认。legacy/partitioned grower 仍明显不适合作默认。

## 3. E2 跨场景 LECT cache reuse

**目的。** 验证 cache reuse 是否超出 matched-route replay，尤其是跨 scene 的 kinematic evidence 复用价值。

**当前数据。** Heavy matrix 共 792 runs、96 aggregate rows。覆盖 4 个 payload：`CritSample+LinkIAABB`、`CritSample+Hull16_Grid`、`IFK+LinkIAABB`、`IFK+Hull16_Grid`；覆盖 `NoCache`、`ColdPersistent`、`TrainWarmup`、`WarmCrossScene`。仍 blocked 的协议行为包括 `WarmSameSceneDifferentQuery`、`WarmCrossProcess`、`TreeSnapshotWarm`。

| Payload | Condition | Runs | Median build s | Median disk MB | 主要观察 |
|---|---|---:|---:|---:|---|
| CritSample + Hull16_Grid | NoCache | 60 | 2.408 | - | grid payload 在线构造较贵。 |
| CritSample + Hull16_Grid | ColdPersistent | 60 | 2.456 | 34.3 | cold fill 有额外持久化成本。 |
| CritSample + Hull16_Grid | WarmCrossScene | 60 | 1.439 | 992.8 | 跨场景 warm 明显降低 build，但磁盘占用高。 |
| CritSample + LinkIAABB | NoCache | 60 | 0.681 | - | AABB payload 本身便宜。 |
| CritSample + LinkIAABB | WarmCrossScene | 60 | 0.626 | 250.9 | 有小幅收益，但收益/存储比不突出。 |
| IFK + Hull16_Grid | NoCache | 60 | 4.111 | - | IFK+grid 最贵。 |
| IFK + Hull16_Grid | WarmCrossScene | 60 | 0.571 | 878.5 | 最强 cache 加速信号。 |
| IFK + LinkIAABB | NoCache | 60 | 0.257 | - | 在线计算已经很快。 |
| IFK + LinkIAABB | WarmCrossScene | 60 | 0.447 | 263.2 | warm 反而慢于 no-cache，不应宣传该组合的 cache 收益。 |

**E2/E8 合并输出。** 已生成 cache reuse + storage tradeoff 汇总：`experiments/results_followup/cache_reuse_storage_tradeoff.json`、`doc/paper/SBF/generated_followup/tab_cache_reuse_storage_tradeoff.tex`、`doc/paper/SBF/generated_followup/fig_cache_reuse_storage_tradeoff.pdf`、`doc/paper/SBF/generated_followup/fig_cache_reuse_storage_tradeoff.png`。

| Payload | NoCache s | WarmCrossScene s | Speedup | Warm disk MB | E8 matched-route disk MB | 主要观察 |
|---|---:|---:|---:|---:|---:|---|
| CritSample + LinkIAABB | 0.68 | 0.63 | 1.09x | 250.9 | 79.4 | 只有小幅收益，存储代价不低。 |
| CritSample + Hull16-Grid | 2.41 | 1.44 | 1.67x | 992.8 | 775.4 | 有明确加速，但接近 1 GB warm cache。 |
| IFK + LinkIAABB | 0.26 | 0.45 | 0.57x | 263.2 | 79.4 | warm 反而更慢，不应宣传 cache 加速。 |
| IFK + Hull16-Grid | 4.11 | 0.57 | 7.20x | 878.5 | 578.5 | 最强 cache 加速，同时磁盘 footprint 很大。 |

**结论。** E2/E8 支持一个更细的 claim：persistent LECT cache 对 expensive Grid payload 特别有效，尤其 `IFK+Hull16_Grid` 达到 7.20x speedup，`CritSample+Hull16_Grid` 为 1.67x；但这两个配置需要约 879-993 MB 的 WarmCrossScene cache。对 cheap `IFK+LinkIAABB`，cache 使 build 从 0.26 s 变为 0.45 s，speedup 只有 0.57x，说明 I/O 成本抵消甚至超过在线计算收益。因此 Hull16-Grid 不建议再作为 E1 的单独 build-ablation 缺口补跑；它更适合在 E2/E8 中作为“速度-存储 tradeoff”讨论，而不是混入 E1 的模块消融表。

## 4. E3 多查询 amortization / break-even

**目的。** 展示 SBF 在 multi-query 场景中一次 build 多次 query 的摊销优势。

**当前数据。** 当前是 retained shelf workload 估算，不是新生成的 100-query pool。batch sizes 为 1、2、5、10、25、50、100。

| Method | SR | K=1 amortized s/query | K=10 | K=100 | 说明 |
|---|---:|---:|---:|---:|---|
| SBF | 1.00 | 1.967 | 0.600 | 0.463 | build 约 1.519 s，query median 约 0.448 s。 |
| PRM | 0.68 | 10.007 | 1.007 | 0.107 | query 极快但 SR 较低；大 batch 下 amortized time 低。 |
| BIT* | 0.84 | 10.027 | 10.027 | 10.027 | 无 reusable build，按固定 query budget。 |
| IRIS-NP+GCS | 1.00 | 602.427 | 60.807 | 6.645 | build 成本极高，但 SR 好。 |

**结论。** 对当前 retained shelf workload，SBF 在小到中等 batch 上有强优势，尤其相对 BIT* 与 IRIS。PRM 在 K=25 以后 amortized time 更低，但 SR 只有 0.68；主文若使用该图，必须同时显示 SR 或 failure-aware metric，避免把 PRM 的 success-only query time 误解为全面优势。

## 5. E4 动态场景更新

**目的。** 量化 obstacle insertion 后 certificate invalidation 与 graph repair 的价值，并区分已实现的 Add-1 与未实现的 remove/move/batch/localized regrowth。

**当前数据。** Heavy schema 汇总了现有 full Add-1 实验，共 150 runs。Remove-1、Move-1、Add-k、Localized regrowth 均 blocked。

| Robot | Difficulty | Runs | Repair median s | Full rebuild median s | Invalidated fraction median | Speedup 约 |
|---|---|---:|---:|---:|---:|---:|
| Panda | easy | 25 | 0.009 | 0.840 | 0.406 | 96x |
| Panda | medium | 25 | 0.009 | 2.910 | 0.235 | 329x |
| Panda | hard | 25 | 0.009 | 3.099 | 0.089 | 337x |
| UR5 | easy | 25 | 0.009 | 0.649 | 0.381 | 71x |
| UR5 | medium | 25 | 0.011 | 0.761 | 0.297 | 72x |
| UR5 | hard | 25 | 0.011 | 0.713 | 0.104 | 63x |

**结论。** Add-1 graph repair 的速度优势非常强，尤其 Panda medium/hard。写作时必须强调这是 surviving boxes 的 graph repair，不是完整 free-space regrowth；remove/move/batch update 还没有 API 支持，不能当作已完成结论。

## 6. E5 topology stress / cross-robot randomized scenes

**目的。** 用 UR5/Panda Easy/Medium/Hard 随机场景作为 topology stress proxy，观察 SBF、PRM、BIT*、IRIS 在不同机器人和难度下的表现。

**当前数据。** Heavy schema 保留 full Exp.5 聚合，共 30 scenes，6 个 robot-difficulty groups。Controlled narrow-passage generator 仍 blocked。

| Group | SBF(C+AABB) | SBF(IFK+AABB) | IRIS-NP+GCS | PRM | BIT* | 主要观察 |
|---|---:|---:|---:|---:|---:|---|
| Panda easy | SR 1.00 | SR 1.00 | SR 0.76 | SR 1.00 | SR 1.00 | SBF 和 PRM/BIT* 都稳，IRIS 稍弱。 |
| Panda medium | SR 1.00 | SR 1.00 | SR 0.68 | SR 1.00 | SR 0.80 | SBF 稳定，BIT*/IRIS 开始受影响。 |
| Panda hard | SR 1.00 | SR 1.00 | 数据见 JSON | 数据见 JSON | 数据见 JSON | hard 组用于突出 topology sensitivity。 |
| UR5 groups | 数据见 JSON | 数据见 JSON | 数据见 JSON | 数据见 JSON | 数据见 JSON | 当前报告保留聚合入口，详细数值可由 `aggregation.groups` 展开。 |

**结论。** E5 适合作为补充材料的 cross-robot topology proxy。它不是 controlled clearance/bottleneck sweep，因此不应写成系统窄通道实验；如果要支撑“难度边界”，仍需 E5 计划中的 controlled generator。

## 7. E6 PathOpt ablation

**目的。** 量化 corridor post-processing 对路径长度、query time 和 success rate 的影响。

**当前数据。** Heavy matrix 共 450 runs、18 aggregate rows，覆盖 `PathOpt disabled`、`Current PathOpt`、`More iterations 10`，每个 robot-difficulty group 25 runs。`Waypoint elimination only` 与 `Local connector enabled` 仍 blocked。

| Config | Group | SR | Build median s | Query median s | Path median | Boxes median | 主要观察 |
|---|---|---:|---:|---:|---:|---:|---|
| Current PathOpt | Panda easy | 0.80 | 0.670 | 0.041 | 6.004 | 3165 | 相对 disabled 路径更短。 |
| PathOpt disabled | Panda easy | 0.80 | 0.843 | 0.063 | 6.204 | 3236 | 关闭后路径变长。 |
| Current PathOpt | Panda hard | 1.00 | 3.504 | 0.291 | 7.363 | 2943 | hard 组路径改善明显。 |
| PathOpt disabled | Panda hard | 1.00 | 3.553 | 0.157 | 9.579 | 2942 | 关闭后路径显著变长。 |
| Current PathOpt | UR5 easy | 1.00 | 0.283 | 1.027 | 5.148 | 3057 | query time 较高但路径优于 disabled。 |
| More iterations 10 | UR5 easy | 1.00 | 0.281 | 1.011 | 4.589 | 3057 | 额外迭代进一步缩短路径。 |
| Current PathOpt | UR5 hard | 0.60 | 0.703 | 1.120 | 9.069 | 3163 | SR 受场景难度影响。 |

**结论。** PathOpt 对路径质量确实有价值，Panda hard 和 UR5 easy 最明显。More iterations 并非全局单调改进，例如 UR5 medium path 基本不变且 query time 变大；建议在论文里说“PathOpt offers targeted path improvement at modest query-time cost”，而不是笼统说更多迭代总是更好。

## 8. E7 并行扩展

**目的。** 比较 default coordinated grower、legacy independent grower 和 partitioned grower 在不同线程数下的 scaling，解释默认并行策略。

**当前数据。** Heavy matrix 共 18 runs，所有 query success rate 为 1.00。

| Grower | Threads | Median total s | Speedup vs T1 | Efficiency | Median boxes | 主要观察 |
|---|---:|---:|---:|---:|---:|---|
| default | 1 | 2.837 | 1.00 | 1.00 | 4765 | 单线程基线。 |
| default | 2 | 0.592 | 4.79 | 2.40 | 1452 | 因算法路径变化，表观超线性。 |
| default | 4 | 0.510 | 5.57 | 1.39 | 1420 | 最快点之一。 |
| default | 16 | 0.569 | 4.98 | 0.31 | 1276 | 16 线程仍快，但效率下降。 |
| legacy | 1 | 2.888 | 1.00 | 1.00 | 4765 | 与 default T1 接近。 |
| legacy | 16 | 2.140 | 1.35 | 0.08 | 4706 | scaling 很弱。 |
| partitioned | 1 | 3.023 | 1.00 | 1.00 | 4755 | 单线程最慢。 |
| partitioned | 12 | 1.485 | 2.04 | 0.17 | 3818 | partitioned 最佳点。 |
| partitioned | 16 | 1.635 | 1.85 | 0.12 | 3811 | 高线程退化。 |

**结论。** default coordinated grower 是当前最合理默认。legacy 几乎不能利用线程；partitioned 有一定 scaling，但整体仍慢于 default。default 的超线性 speedup 伴随 boxes 大幅减少，说明它不是纯粹固定工作量 parallel speedup，写作时应解释为 algorithmic+parallel effect。

## 9. E8 cache 存储与 I/O profile

**目的。** 给 LECT cache 的存储代价和 payload 大小一个可引用口径。

**当前数据。** E8 是 retained measured 汇总，共 14 rows。`link_envelope_pipeline` 的 LinkIAABB payload 基本为 0 bytes 级别；Hull16_Grid 单 payload 为几百到数千 bytes；`marcucci_envelope_build` 的 persistent cache 文件为 MB 级。

| Payload | Source | Mean payload / cache size | 主要观察 |
|---|---|---:|---|
| LinkIAABB | link_envelope_pipeline | 约 0 bytes optimized payload | 在线构造与存储都轻。 |
| Hull16_Grid | link_envelope_pipeline | 约 0.7-7.4 KB cache payload per row | 单节点 payload 明显更大。 |
| CritSample + AABB S=4 | marcucci_envelope_build | 约 79.4 MB | matched-route cache artifact 量级可控。 |
| CritSample + Hull16-grid d=0.04 | marcucci_envelope_build | 约 775.4 MB | grid cache 显著占磁盘。 |
| IFK + AABB S=4 | marcucci_envelope_build | 约 79.4 MB | AABB cache 与 endpoint source 无明显存储差异。 |
| IFK + Hull16-grid d=0.04 | marcucci_envelope_build | 约 578.5 MB | grid payload 仍是主要存储来源。 |

**结论。** E8 与 E2 一致：cache 收益主要来自 expensive grid payload，但代价是数百 MB 到接近 1 GB 的磁盘 footprint。新的 `fig_cache_reuse_storage_tradeoff` 已把 build-time speedup 和 disk MB 放在同一张图里，可直接作为补充材料图或主文小图的基础。

## 10. E9 soundness / certificate audit

**目的。** 用独立密集 segment collision audit 检查 successful SBF paths，发现实现层面的 invalid segment 风险。

**当前数据。** Heavy audit 共 262 paths、2485 segments，`segment_resolution=128`。当前 invalid certified segments 总数为 312，不是 0。

| Group | Method | Paths | Segments | Invalid | Invalid rate |
|---|---|---:|---:|---:|---:|
| Panda easy | sbf | 25 | 203 | 11 | 0.054 |
| Panda easy | sbf_ifk | 25 | 203 | 11 | 0.054 |
| Panda medium | sbf | 25 | 164 | 26 | 0.159 |
| Panda medium | sbf_ifk | 25 | 126 | 26 | 0.206 |
| Panda hard | sbf | 25 | 309 | 47 | 0.152 |
| Panda hard | sbf_ifk | 25 | 289 | 41 | 0.142 |
| UR5 easy | sbf | 25 | 253 | 37 | 0.146 |
| UR5 easy | sbf_ifk | 25 | 210 | 24 | 0.114 |
| UR5 medium | sbf | 15 | 175 | 12 | 0.069 |
| UR5 medium | sbf_ifk | 15 | 151 | 15 | 0.099 |
| UR5 hard | sbf | 16 | 201 | 35 | 0.174 |
| UR5 hard | sbf_ifk | 16 | 201 | 27 | 0.134 |

**结论。** 当前 E9 是最重要的风险项。它可能意味着：审计口径把 local/non-certified segment 也计入了 `invalid_certified_segments`，或者 path serialization / validation frame / scene obstacle interpretation 存在不一致，也可能是真实 certificate bug。下一步必须拆分 certified corridor segment 与 local completion segment，并对若干 invalid sample 做可视化和复核。在复核前，论文不能声称 dense audit found 0 invalid certified segments。

## 11. E10 baseline seed/hyperparameter sensitivity

**目的。** 整理 retained baseline configs 和已有 BIT* sensitivity smoke，明确 full factorial sweep 尚未实现。

**当前数据。** Heavy schema 共 15 rows：4 条 retained config，11 条 BIT* smoke；另有 1 条 blocked protocol row。

| Method | Queries | SR | Median query s | Median path | 主要观察 |
|---|---:|---:|---:|---:|---|
| ompl_prm | 25 | 0.68 | 0.007 | 2.668 | query 很快但成功率低。 |
| ompl_bitstar_budget | 25 | 0.84 | 10.029 | 3.440 | 固定 10s budget，SR 高于 PRM。 |
| iris_np_gcs | 25 | 1.00 | 0.627 | 2.438 | retained config 成功率和路径质量好。 |
| iris_zo_gcs | 25 | 0.00 | - | - | retained config 全失败。 |

**结论。** E10 当前只能作为 retained config + smoke evidence，不能支撑完整 hyperparameter sensitivity claim。若要写进补充材料，标题应明确为 “retained baseline configs and BIT* smoke sensitivity”，并保留 blocked row 说明 full IRIS/PRM/BIT* factorial runner 尚未实现。

## 12. 推荐写作使用方式

### 可进入主文或补充材料的稳妥结论

1. **E2/E8:** Cache reuse 对 grid payload 有强收益，但必须同时报告 disk footprint；`IFK+Grid` 是 7.20x speedup / 878.5 MB，`IFK+AABB` 是 0.57x / 263.2 MB。
2. **E4:** Add-1 obstacle repair 相比 full rebuild 有 60x-330x 量级 speedup，但只覆盖 graph repair / surviving certificates。
3. **E6:** PathOpt 能显著降低部分场景的 path length，尤其 Panda hard 和 UR5 easy；更多迭代收益不总是单调。
4. **E7:** Default coordinated grower 明显优于 legacy 和 partitioned，解释了当前默认设置。
5. **E1:** 三个必补缺口已补齐；`ffb_depth=80`、`No unexplored`、`ffb_depth=200` 都保持 100% SR，但这更像 workload-specific sensitivity，不应直接替代默认配置。

### 暂不建议写成强 claim 的部分

1. **E1:** Hull16-Grid 仍未作为 E1 row 独立补跑；基于 E2/E8，本轮建议不补，除非论文需要“envelope representation as build-ablation row”这一特定表述。
2. **E3:** 当前不是新 100-query pool，只是 retained workload amortization；应称为 retained-workload estimate。
3. **E5:** 当前是 randomized-scene topology proxy，不是 controlled narrow-passage stress sweep。
4. **E9:** invalid certified segments 非零，需要先诊断。
5. **E10:** full sensitivity sweep 尚未实现。

## 13. 下一步优先级

1. **最高优先级：诊断 E9 invalid segments。** 随机抽取每个 robot/difficulty/method 的 invalid segment，确认它们是否属于 certified corridor、local completion、path interpolation 或碰撞检查器坐标/障碍物解释不一致。
2. **E1 已补齐必补缺口。** 后续只需决定是否为了特定叙事补 Hull16-Grid；当前建议不补，把 Grid 证据放在 E2/E8 tradeoff 中。
3. **E2/E8 图表已生成。** 下一步可把 `fig_cache_reuse_storage_tradeoff.pdf` 接入补充材料，caption 明确 WarmCrossScene disk 与 E8 matched-route disk 是不同协议下的存储参考。
4. **把 E7 做成一张 speedup curve。** default、legacy、partitioned 三条线足够说明默认策略。
5. **E3 若要主文使用，生成真正 100-query pool。** 否则只放补充材料，标题写 retained workload amortization。
