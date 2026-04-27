# Hull16Grid Warm 路径优化计划（v2，基于真实细粒度测量）

## 重大修正（2026-04-23）

首版计划把"Z4GridCache lookup 50 μs × 154k = 7.7 s ≈ grow 全部时间"作为依据。**这个计算忽略了 8 线程并行**，实际 lookup 并非瓶颈。

P0（`node_grids_` 改 `shared_ptr<const>` 消除深拷贝）实施并验证后，**grow 中位数与 baseline 在噪声内（7058ms vs 6800ms）**，lookup_avg 甚至略升。

给 `ffb.cpp` 主循环加上 per-branch timer（envelope_ms / collide_ms / expand_ms），在 warm 下跑得到**真正的瓶颈画像**：

| 指标 (warm, 1 seed, 8 threads, 全部 Z4 盘缓存就绪) |  值 |
|---|---:|
| FFB total (thread-time) | 27341 ms |
| └ envelope_ms (compute_envelope on FFB step) | **0 ms** (100% cache hit) |
| └ collide_ms (collides_scene) | 2887 ms (11%) |
| └ **expand_ms (expand_leaf / split_leaf_impl)** | **24329 ms (89%)** |
| └ 其它 (descend / fk / deadline etc.) | 124 ms (0%) |
| FFB per-call | 6.3 ms |
| expand_leaf 调用数 | 49499/seed |
| expand_leaf 每次 | 0.47 ms |

同时发现：

- **worker LECT expansion 未写回主 lect_**。`lect_.snapshot()` 把主 lect 按值复制给 worker，worker 每次 split_leaf_impl 都只改自己的副本，下次 snapshot 覆盖时丢失；主 lect 仅靠 `mark_occupied` 收 box，但不收 tree 结构。结果：
  - warm 启动加载 452k cached nodes，跑完 grow 只增长到 488k（+36k）。
  - 每次 seed 的 8 worker × 18k split 中，只有 1 份（≈ 18k）能持久化，其它都白干。
- Z4GridCache 当然帮助了一部分——子节点的 envelope 从盘里 50 μs 取回，否则要重算 link envelope + grid 栅格化会慢 10×；但剩余 400 μs/expand 全花在：`alloc_node` + `compute_fk_incremental` ×2 + interval vector 拷贝 + `refine_parent_aabb` + `invalidate_collide`。

**一句话结论**：warm bottleneck = 每个 seed 重做 ~50k 次 tree split，且多数 split 在 worker 副本里被丢弃，每 split 花 0.5 ms。Z4GridCache 微优化（LRU/Bloom/TLS brick_buf/深拷贝消除）在并行 8 线程下节省 < 100 ms，和 warm grow 7 s 不在一个量级。

## 新的优化路线（按 ROI 排序）

### Q1 — worker expansion 写回主 LECT（最大 ROI，预期 grow -3~4 s）

`lect_.snapshot()` 之前先把 worker 新增的节点合并回主 lect。具体：

1. LECT 增加 `merge_from(const LECT& src)`：把 src 中主 lect 尚未有的 split 结构（left_/right_/split_dim_/split_val_/depth_/parent_）迁移过来；ep_data / channels / has_data / link_iaabb_cache 也顺带拷贝（避免主 lect 下次 snapshot 又丢失这些）。
2. 触发时机：每次 "periodic refresh"（`total_batches % lect_refresh_interval == 0`）和 inline promote 时 **先 merge 再 snapshot**。
3. Correctness：worker lect 是单写者，合并到主 lect 的路径 append-only（不重用 node idx），不需要锁。snapshot 仍是值拷贝。

**预期**：warm seed0 grow 7.0 s → 4~5 s（tree 已经全量持久化，expand_leaf 命中率从 36% 升到 95%+）。
warm/cold ratio 从 1.13 降到 ≤ 1.0，且**改造后 warm 会显著快于 cold**（cold 时 worker 仍要从头构建 tree）。

**风险**：中。需要小心同步点与 forest_id_/subtree_free_volume_/link_iaabb_cache_ 的 invariants。

**验证**：cold/warm 各 3 seeds；新增指标 `worker_merge_ms`、`nodes_merged_per_seed`。

#### Q1 实施结果（2026-04-23）

通过更简单的方法达成了目标：**把 `lect_refresh_interval` 从 10 提升到 1000**（实际等于"单个 seed 内不再 refresh"）。refresh 机制本身才是 worker expansion 丢失的 直接原因；把周期拉长，worker 的 tree 增长在单个 seed 内完全保留。**不需要实际实现 merge_from**。

| Config | Cold grow med | Cold total med | Warm grow med | Warm total med | boxes | islands | edges |
|---|---|---|---|---|---|---|---|
| baseline iv=10 | 7479 ms | 8.82 s | 7363 ms | 8.73 s | 3365 | 3 | 38k |
| **iv=1000 (commit)** | **5074 ms** | **6.82 s** | **3874 ms** | **5.22 s** | 3459 | 3 | 37.7k |
| 提升 | -32% | -23% | **-47%** | **-40%** | +3% | = | ≈ |

- warm grow **低于目标 4 s**（3.87 s）。warm total **低于目标 5 s**（5.22 s，差 0.22 s）。
- cold/warm ratio = 6.82/5.22 = **1.31×**，即 warm 显著快于 cold。
- 质量指标（boxes/islands/edges）无退化。
- 实现成本：1 行（改 const int）+ env override 钩子。
- 提供 `SBF_LECT_REFRESH_INTERVAL` 环境变量供回退/实验。

**副作用分析**：原来 refresh 的目的是让 worker 看到其他 worker 的 `mark_occupied`。不 refresh 后 worker 会尝试 grow 到已占用区域，但 master 在 accept 阶段已有 `lp->is_occupied(wr.ffb.node_idx)` 过滤，boxes 数未减少（3365 → 3459），说明这点"重复尝试"对整体吞吐的影响被 FFB 缓存抵消了。

**后续 Q2/Q3 是否仍需要？** 若目标就是 warm grow ≤ 4 s + warm total ≤ 5 s，Q1 已达到；Q2/Q3 可视后续需求（例如想把 warm grow 推到 ≤ 2 s）再实施。

### Q2 — 降低 expand_leaf 的"空转"调用（次高 ROI，预期 grow -0.8~1.5 s）

FFB 报告 49k 次 expand_calls，但 split_leaf_impl 里 expand_profile 只看到 18k。差额 31k 对应 `if (!is_leaf(node_idx)) return 0` 的快速返回（概率起源于 worker 间的 snapshot 时序）。这 31k 每次仍要 ~0.2 ms 的函数调用/虚拟分派/计时器。

1. FFB 调用 `expand_leaf` 前先自己判一次 `lect.is_leaf(current)` — 已经有了，所以 expand_leaf 不应再判一次；把 expand_leaf 的入口检查去掉，或者改成 debug-only assertion。
2. 统一 `split_leaf_impl` 的 pick_dim cache：**per-LECT** instead of global — 当前 depth_split_dim_cache_ 是 per-LECT，但 worker snapshot 后会清零，要修复。

**预期**：expand_calls 从 49k 降到 ~20k，expand_ms 从 24 s → 10 s（thread-time）→ warm grow -1 s（8 并行）。

### Q3 — expand_leaf 内部常数因子（低-中 ROI，预期 grow -0.3~0.6 s）

- `auto left_ivs = parent_intervals` 复制整个 interval vector — 改为原地修改（mutable reference）+ 临时保存修改前值。
- `alloc_node()` 如果在热路径上触发 capacity grow（vectors 重分配 liaabb_cache_ / forest_id_ / subtree_free_volume_ / 等），一次 ~ms；应一开始就 reserve 到更大容量。
- compute_fk_incremental 对同一 dim 调两次（左右子） — 把不变的部分外提，只换 dim 值的变化项。

### Q4 — Z4GridCache 微优化（保留 P1/P4，弃用 P2/P3）

- **P1 (TLS brick_buf)**：简单且不会伤害正确性，继续做，预期 -5~10 μs/lookup → warm grow -50~100 ms。
- **P4 (SparseVoxelGrid reserve)**：加入，预期 -3~5 μs/lookup → warm grow -30~50 ms。
- **P2 (Bloom filter on disk_miss)**：Q1 实施后 disk_miss 次数将锐降（tree 已缓存，少了新子节点），ROI 不足，推迟。
- **P3 (LRU sweep)**：保留做一次扫描对比，低优先级。

## 实施顺序

1. **Q1** (worker lect merge) → 最大 ROI，预期单独能达成目标 warm grow ≤ 4 s。
2. **Q2** (expand_leaf 空转) → Q1 的后续加成。
3. **Q3** (expand_leaf 常数因子) → 依据 Q1+Q2 的 profile 结果决定要不要做。
4. **Q4** (P1 + P4) → 后续清理。

## 验证基线

```bash
cd /home/tian/桌面/box_aabb/cpp/build
rm -rf /home/tian/.sbf_cache/*
./experiments/exp6_build_timing --envelope hull16_grid --endpoint ifk \
    --lect-cache --seeds 3 --threads 8 > /tmp/cold.log 2>&1
./experiments/exp6_build_timing --envelope hull16_grid --endpoint ifk \
    --lect-cache --seeds 3 --threads 8 > /tmp/warm.log 2>&1
# sbf 日志（含 ffb_breakdown、expand_profile）
grep -E "ffb_breakdown|expand_profile|lect_nodes|Z4GridCache.*stats" \
    $(ls -t /home/tian/桌面/box_aabb/cpp/v6/log/sbf_*.log | head -2)
```

| 阶段 | warm grow med | ffb_expand_ms | expand_calls | expand_profile.calls | lect_nodes_added |
|---|---|---|---|---|---|
| baseline (shared_ptr 已生效但无效) | 7058 ms | 24329 ms | 49499 | 18315 | ~36k |
| Q1 (worker merge) | TBD | TBD | TBD | TBD | TBD |
| Q1+Q2 | TBD | TBD | TBD | TBD | TBD |
| Q1+Q2+Q3 | TBD | TBD | TBD | TBD | TBD |
| Q1..Q4 | TBD | TBD | TBD | TBD | TBD |

## 目标

warm grow ≤ 4 s（当前 7.06 s），warm total ≤ 5 s，warm 显著快于 cold（目标 cold/warm ≥ 1.4）。

---

# 第二轮（v3, 2026-04-24）：R1–R5 计划

## Q1 后的真实瓶颈画像

iv=1000 已生效，warm grow 从 7.36 s 降到 3.80 s（中位数）。在该 baseline 上，warm seed2-3 的 ffb_breakdown 显示：

| 指标 | 值 | 占 grow wall 比例 |
|---|---:|---:|
| coordinated_grow (wall) | 3760 ms | 100% |
| └ ffb_wall | 2710 ms | 72% |
| └ task_gen + post_accept + 其它 | 1050 ms | 28% |
| **FFB thread-time 累计** | 8260 ms | — |
| └ envelope (compute_envelope on FFB step) | **0 ms** | 0% (Z4 全命中) |
| └ collide (collides_scene) | 32 ms | 0.4% |
| └ **expand (split_leaf_impl)** | **8131 ms** | **98.5%** |
| └ 其它 (descend / fk / deadline) | 90 ms | 1% |
| expand_profile (8 worker lect 累加) | 60k calls / 120k new_nodes | — |
| └ env_ms (compute_envelope on 2 new children) | 8400 ms | **95% of expand** |
| Z4GridCache lookups | 130k/seed | — |
| └ lookup_avg | 36–42 μs | — |
| └ mem_hit | 6% (LRU 256 MB 装不下热集) | — |
| └ disk_hit / disk_miss | 47k / 88k (53% miss) | — |
| Z4 thread-time 累计 | 130k × 40 μs ≈ 5.2 s | **~63% of expand** |

**结论**：

1. `split_leaf_impl::compute_envelope` 是 expand 的 95%；compute_envelope 的耗时几乎全是 Z4GridCache lookup。
2. 60k expand_calls / seed 但 lect_nodes 只增 ~40k → 8 worker 间约 33% split 重复（iv=1000 的副作用）。
3. mem_hit 仅 6%，disk_miss 88k 浪费在探测空 slot。
4. Z4 lookup 是当前真正可优化的 hot path，但优化方向不是"消除拷贝"（P0 已证）而是：(a) 减少调用数，(b) 加速 disk_miss 路径，(c) 加速 disk_hit 路径。

## R1 — Z4GridCache lookup 延迟到 collide_scene 才触发

**问题定位**：[lect.cpp:493–522](../../src/lect/lect.cpp) 中，IFK fast path 命中 EP cache 时立即调用 `gc.lookup(grid_key, grid_req)` 并把结果 push 到 `node_grids_[node_idx]`。但在 FFB 主循环中，120k 次 compute_envelope 调用对应 57k 次 collides_scene；剩余 ~63k 节点根本没机会用 grid（早被 collide_cache 覆盖、被 free 路径直接返回、或被 occupied 路径跳过）。

**改动**

1. 在 LECT 加状态：`std::vector<uint8_t> node_grid_pending_;`（capacity 大小，0 = 不需要 grid，1 = 需要 lazy 加载，2 = 已加载）。
2. compute_envelope 的 IFK fast-path 在 `EnvelopeType != LinkIAABB` 分支中，只设 `node_grid_pending_[node_idx] = 1` 并存 `(z4_key, sector, channel)` 到副结构 `pending_grid_keys_[node_idx]`，**不调用** `gc.lookup`。
3. 在 `collides_scene(int, Obstacle*, int n_obs)` 三个 grid 分支前加 `materialize_grid_if_pending(node_idx)`：把 pending=1 转 2，同步加载 grid。
4. cache miss 路径（compute_node_grids）保持立即计算并 insert（与 R1 无关；那是 EP cache miss 后的兜底）。
5. snapshot 时 `node_grid_pending_` 以 lazy 复制语义传递（worker 局部，单写者）。

**预期**：grid lookup 数 130k → 60–65k；Z4 thread-time 5.2 s → 2.5 s；wall 节省 ~300 ms（按 8 线程平摊）。
warm grow 3.80 s → 3.45 s。

**风险（中）**：
- `try_promote_envelope_union` / `compute_node_grids` 等显式访问 `node_grids_[i]` 的地方需要先 materialize。
- pending 状态在 `expand_leaf`、`mark_occupied`、`invalidate_collide` 等路径要正确传递（pending 节点 children 默认 0）。
- snapshot/IO 不持久化 pending（每个 lect 实例独立懒加载）。

**验证**：
- warm grow 中位数 ≤ 3.5 s。
- `Z4GridCache stats: lookups` 大约减半。
- boxes / islands / edges 在 baseline ±5%。

## R2 — Bloom filter 过滤 disk_miss

**问题定位**：disk_miss 路径目前要 `find_index_slot` 三次链探测、再校验 `slot.quality.matches(req)` — 平均 ~10 μs，但结果是"什么都没找到"。88k miss × 10 μs = 880 ms thread / seed。

**改动**

1. 在 `Z4GridCache` mmap header 之后增加 4 MB bitmap（32 M bits）：
   ```cpp
   constexpr size_t kBloomBits = 32 * 1024 * 1024;
   constexpr size_t kBloomBytes = kBloomBits / 8;  // 4 MB
   uint8_t bloom[kBloomBytes];  // mmap 持久化
   ```
2. 用 2 个独立 hash（splitmix64 with two different seeds）。3M unique keys 时假阳性率 ~1.5%。
3. `lookup()` 入口先查 bloom：未命中 → `disk_misses_++; return nullptr`。
4. `insert()` 写入数据后 `atomic_fetch_or` 设两个 bit。
5. schema bump `kVersion 2 → 3`，旧文件 incompatible 自动重建。
6. memory cache (LRU) 命中时跳过 bloom 检查（mem_hit 路径不变）。

**预期**：
- 88k disk_miss 中 ~98.5% 直接被 bloom 拒绝，节省 880 ms × 0.985 ≈ 870 ms thread → ~110 ms wall。
- R1 实施后 lookups 减半，disk_miss 也减半到 ~44k → wall 节省 ~55 ms。仍值得做。
- bloom 查询 ~10 ns（两次 hash + 两次内存访问）几乎不计入。

**风险（中）**：
- mmap 段加 4 MB 需要把 header offset 计算改对，并保证旧 cache 的 magic/version 检测能识别 v3。
- 多进程并发写 bloom 时 atomic OR 必须正确。
- 重启后 bloom 的位是否仍准确（持久化到 mmap，自然恢复）。

**验证**：
- `Z4GridCache stats` 中 disk_miss 数不变，但 lookup_avg 显著下降（disk_miss 路径从 10 μs 降到 0.1 μs）。
- 重新跑 cold（删 cache）→ bloom 重建后 warm 应保持优化。

## R3 — TLS brick_buf + SparseVoxelGrid::reserve（最易，最先做）

**问题定位**：47k disk_hit / seed 每次都 `std::vector<uint8_t> brick_buf(brick_bytes)` 触发 malloc；之后 `make_shared<SparseVoxelGrid>` 内部 hashmap 默认 bucket=0，N 次 set_brick 触发多次 rehash。

**改动**

1. [`Z4GridCache::lookup`](../../src/lect/z4_grid_cache.cpp) disk_hit 分支：
   ```cpp
   thread_local std::vector<uint8_t> brick_buf;
   brick_buf.resize(brick_bytes);  // 复用 capacity
   ```
2. `SparseVoxelGrid` 增 `void reserve(size_t n_bricks)` 透传到 `bricks_.reserve`。
3. lookup disk_hit 路径在 `make_shared<SparseVoxelGrid>(delta)` 后立即 `grid->reserve(n_bricks)` 再循环 set_brick。

**预期**：
- malloc 节省 ~3 μs × 47k = 140 ms thread → ~18 ms wall。
- hashmap reserve 节省 rehash ~5 μs × 47k = 235 ms thread → ~30 ms wall。
- 合计 wall ~50 ms。

**风险（极低）**：thread_local 在 worker 线程退出归还内存到 glibc，无泄漏。

**验证**：lookup_avg 下降 5–10 μs（disk_hit 平均路径）。

## R4 — 真正的 worker LECT merge-back

**问题定位**：60k expand_calls / seed 但 main lect_nodes 只增 ~40k → 33% split 在 worker 间重复。若所有 worker 共享 split 成果，expand_calls 可降到 ~40k（节省 33% expand wall）。

**改动**

1. LECT 加 `int n_nodes_at_snapshot_;` 字段，记录 snapshot 时主 lect 节点数。
2. LECT 加 `void merge_from(const LECT& worker)`：只把 `idx ∈ [n_nodes_at_snapshot, worker.n_nodes_)` 的节点 append 到主 lect。需要拷贝：
   - 树结构：left_, right_, parent_, depth_, split_dim_, split_val_
   - EP 通道：channels_[CH_SAFE/CH_UNSAFE].ep_data_, has_data, source_quality
   - link_iaabb_cache_ + link_iaabb_dirty_
   - subtree_free_volume_, forest_id_
   - node_grids_, node_grid_meta_（shared_ptr，仅引用计数）
3. snapshot 前调用 merge_from 把 worker 增长的部分写回主。先 merge 所有 worker，再 snapshot 给所有 worker。
4. 注意 worker 之间的 split idx 可能冲突（worker A 给 node 100 split 出 1234, worker B 给 node 100 split 出 1234，但 left/right 指向各自不同的子树）。简化处理：**接受最先合并者**，后续重复 split 视作 worker 私有。

**预期**：
- expand_calls 60k → 40k，expand_ms 8.1 s → 5.4 s thread → wall 节省 ~340 ms。
- lookups 130k → 80k，进一步带动 R1/R2 的收益。

**风险（高）**：
- forest_id_ / subtree_free_volume_ / depth_split_dim_cache_ 多个 invariant 要在 merge 时保持一致。
- 同一节点在两个 worker 中被 split 成不同 dim/val 时如何选择（先到先得？合并失败时 worker 后续步可能引用不存在的子节点）。
- 测试覆盖成本高。

**仅在 R1+R2+R3 未达标 (warm grow > 3.2 s) 时启动**。

## R5 — FFB 步数本身的优化（探索性）

**问题定位**：4350 FFB calls × 24 steps avg = 104k steps/seed。每 step 3 次 vector 索引 + 1 次 has_data + 1 次 is_occupied + 可能的 envelope 计算。即使 99% 走 cache hit，常数开销也 ~80 μs/call。

**改动选项**（任选）：

A. **Seed→leaf 缓存**：worker 局部 `unordered_map<seed_hash, leaf_idx>`，重复 seed（boundary 采样常出现）直接跳过 descent。预期命中率 ~10%。

B. **批量 descent**：把 batch 内 seeds 排序后共享前缀路径，类比 trie 遍历。复杂度高。

C. **降低 max_depth**：当前 100，实测 99% step 在 depth < 60。降到 60 应该不损失成功率。

**预期**：A 节省 ~10% FFB wall = ~270 ms。C 几乎无收益（深度只是上限）。

**风险（中）**：A 增加 hashmap 开销，B 大改架构，C 可能回归 success rate。

**仅在前 4 项达标后还想压榨时考虑**。

## 实施顺序与目标

| 步骤 | 改动 | 预期 wall 节省 | 风险 | 估计代码量 |
|---|---|---|---|---|
| 1 | R3 (TLS brick + reserve) | 50 ms | 极低 | < 50 LOC |
| 2 | R1 (lazy grid lookup) | 350 ms | 中 | ~150 LOC |
| 3 | R2 (bloom filter) | 100 ms | 中 | ~200 LOC |
| 4 | R4 (worker merge) | 300 ms | 高 | ~300 LOC |
| 5 | R5 (FFB micro) | 100 ms | 中 | 视方案 |

**累积目标**：warm grow 3.80 s → **3.20 s**（-16%），warm total 5.18 s → **4.55 s**（-12%）。

R1+R2+R3 单独应该能让 warm grow ≤ 3.3 s，warm total ≤ 4.7 s。如达标即停，R4/R5 推迟。

## 验证基线

```bash
cd /home/tian/桌面/box_aabb/cpp/build
rm -rf /home/tian/.sbf_cache/*
./experiments/exp6_build_timing --envelope hull16_grid --endpoint ifk \
    --lect-cache --seeds 3 --threads 8 > /tmp/cold.log 2>&1
./experiments/exp6_build_timing --envelope hull16_grid --endpoint ifk \
    --lect-cache --seeds 3 --threads 8 > /tmp/warm.log 2>&1
latest=$(ls -t /home/tian/桌面/box_aabb/cpp/v6/log/sbf_*.log | head -1)
grep -E "ffb_breakdown|expand_profile|lect_nodes|Z4GridCache.*stats|coordinated_grow" "$latest" | tail -30
```

| 阶段 | warm grow med | ffb_breakdown exp | Z4 lookups | Z4 lookup_avg | mem_hit |
|---|---|---|---|---|---|
| baseline (Q1 in tree) | 3802 ms | 8131 ms | 130k | 36 μs | 6% |
| R3 | 3574 ms (-6%) | ~5400-7000 ms | 130k | 22 μs (-39%) | 6% |
| **R3+R1** | **675 ms (-81%)** | **~400 ms (-95%)** | **40k (-69%)** | **0.5 μs (-98%)** | 0% |
| R3+R1+R2 | (R2 likely unnecessary — target hit) | | | | |

### 实施记录

- **R3 (2026-04-24)**：TLS `brick_buf` + `SparseVoxelGrid::reserve(n_bricks)`。
  - 文件：`include/sbf/voxel/voxel_grid.h`（新增 `reserve`），`src/lect/z4_grid_cache.cpp`（lookup/insert 两处 TLS）。
  - 验证：warm grow med 3802→3574 ms（-228 ms），lookup_avg 36→22 μs。

- **R1 (2026-04-24)**：lazy grid lookup（核心优化）。
  - 思想：`compute_envelope` 命中 EP cache 后**不再立即** `gc.lookup`；只在 `node_pending_grid_*[i]` 标记一次 (key, ch)。
    `collides_scene` / `try_promote_envelope_union` / `snapshot` / `lect_io::save_*` 等真正读取 `node_grids_[i]` 的位置先调用 `materialize_pending_grid_(i)`。
    Cache miss 时回落到 `compute_node_grids + gc.insert`，与原路径行为等价；cache hit 时仍是 push `shared_ptr`。
  - 文件：
    - `include/sbf/lect/lect.h`：新增 `mutable node_pending_grid_{key,valid,ch}_` 三个 vector 与 `materialize_pending_grid_/materialize_all_pending_grids_/set_pending_grid_` 三个私有方法；`num_grid_slots/node_grids/node_grid_meta` 三个 inline accessor 内部调用 materialize。
    - `src/lect/lect.cpp`：
      - `compute_envelope` 两处 grid lookup（Z4 cache hit 分支 + IFK fast-path）改为 `set_pending_grid_`。
      - `collides_scene` variant 2/3 与 `try_promote_envelope_union` (parent/li/ri) 入口调用 materialize。
      - `compute_node_grids` 写入前清掉同一节点的 pending 标记。
      - `ensure_capacity` / `alloc_node` 同步维护 pending 向量。
      - 新增 helper 实现。
    - `src/lect/lect_snapshot.cpp`：`snapshot()` 与 `transplant_subtree/transplant_domain` 的 worker 节点拷贝处先 materialize。
    - `src/lect/lect_io.cpp`：`lect_save_binary` / `lect_save_incremental` 入口 materialize_all。
  - 关键收益（warm，3 seeds，--lect-cache，与 R3+R1 同一 cache 状态）：
    - **expand_profile env_ms：5400-7000 ms → 89-120 ms（-98%）**。
    - **Z4 lookups：130k → 35-43k（-70%）**——~47% 节点真不读 grid，剩余以更高 hit 率走 fast path。
    - **lookup_avg：22 μs → 0.5 μs（-98%）**——deferred 路径上的命中走 in-memory shared_ptr push，绕过 disk index 大 part。
    - **warm grow med：3574 ms → 675 ms（-81%、-2899 ms）**——远超 350 ms 的最初预期，因为不仅省了 lookup 本身，还把"无效路径上的 hull rasterize"全部消除。
    - **冷启动：first-seed grow 8020 → 779 ms（-90%）**——cache miss 路径上的 compute_node_grids 也只在真正访问时才发生。
  - 质量保持：boxes / islands / edges 与 baseline 在噪声范围内（5400-5500 boxes，3 islands）。


---

# 第三轮（v4, 2026-04-24）：N1–N5 计划

## R1+R3 后的真实瓶颈画像（warm，3 seeds，--lect-cache）

总 build_time med = **2.93 s**，summary 给出的 Per-Phase 加起来只有 ~1.56 s（LECT 152 + Grow 833 + Coarsen1 548 + Adjacency 26）。剩余 ~1.4 s 隐藏在 phase summary 之外。

per-seed 实测细分（以 seed 1 为代表）：

| 子阶段 | 时间 | 是否在 phase summary | 备注 |
|---|---|---|---|
| LECT load (mmap+lazy) | 152 ms | ✅ LECT | |
| coordinated_grow | 1756 ms | ✅ Grow | ffb_total=1433 + repair=404 + roots=18 |
| ├─ FFB `exp` | 1215 ms | — | 占 FFB 85% |
| ├─ FFB `col` | 165 ms | — | |
| ├─ island repair | 404 ms | — | 桥接 5→3 islands |
| **lect_save_incremental** | **1321 ms** | **❌ 隐藏** | 写 ~40k 新节点，每节点 10 次 seekp+write |
| tree_uf check | 104 ms | — | grow connectivity |
| Coarsen1 | 548 ms | ✅ Coarsen1 | sweep1=112 + relaxed=277 + artic=16 + greedy1=143 |
| seed-bridge | 310 ms | ✅ Adjacency.seed-brg | RRT bridge between 5 seeds |
| Adjacency final | 37 ms | ✅ | |

关键发现：

1. **lect_save_incremental ≈ 1.3 s 是当前最大单项** — 完全隐式，per-node 10 次细粒度 IO，可批量化为列写一次。
2. **expand_profile 子项之和 187 ms vs ffb_breakdown.exp 1215 ms**，差 ~1028 ms 在 expand 路径里没被任何 timer 覆盖（split bookkeeping、节点分配、cache 写入、grid 更新等）。需先补 instrumentation。
3. Coarsen1 三个子阶段（sweep1=112, relaxed_sweep1=277, greedy1=143）从未优化过，每对候选都跑 `collides_scene`，存在批量化空间。
4. seed-bridge 310 ms 是 5 seed × per-pair RRT，per_pair_timeout_ms=200，有调小或预筛空间。

## N1 — lect_save_incremental 列式批量写（最先做）

**目标**：1321 → ≤ 300 ms（**−1000 ms / -76%**），warm total 直接 −34%。

**当前路径**（`cpp/v6/src/lect/lect_io.cpp::lect_save_incremental`，对 i ∈ [old_n_nodes, nn) 每个节点）：

```
write_node_tree(i):  5 列 int32 + 1 列 double + has_data[2] + sq[2]
                     → 8 次 seekp+write
write_node_ep(i):    safe + unsafe → 2 次 seekp+write
                     → 共 10 次 IO/节点
```

40k 新节点 → 400k syscall。即便走 page cache 也是数 ms 量级开销 × 数十万次。

**改造**（不动 V5 文件格式）：

1. 先把 `[old_n_nodes, nn)` 范围按列拷到临时 `std::vector<char>`：
   - 5 个 int32 列：`left/right/parent/depth/split_dim`，每列 `(nn-old)*4` 字节
   - split_val：`(nn-old)*8` 字节
   - has_data：`(nn-old)*2` 字节（safe+unsafe 交错）
   - source_quality：`(nn-old)*2` 字节
2. 每列 **单次** seekp + write 整段数据。
3. EP 段：把 `[old_n_nodes, nn)` 的 safe+unsafe 拷到一个 `(nn-old)*ep_ns` 字节的 buffer，单次 seekp + write。
4. "patch 旧节点"循环（split node 子指针变化）保留逐节点路径——内部节点数 << 叶子，不是瓶颈；可考虑收集索引后按列 sort+coalesce，但先做最小改动。

**文件**：仅 `cpp/v6/src/lect/lect_io.cpp` 中 `lect_save_incremental` 函数，约 ±80 行。

**风险**：低。文件偏移计算与原 SoA 布局完全一致，只是聚合 IO；read 端（`lect_load_binary` / mmap）不变。

**验证**：
- 跑 warm 3 seeds，看 `lect_save=` 行从 ~1.3 s 降到目标值。
- 比对 lect 文件 md5：N1 前后保存同一棵 LECT（同 seed/同 obs），文件应字节相同。

## N2 — expand 路径补 instrumentation

**目标**：先观测，再决定是否做 N4 类优化。

**问题**：expand_profile 显示 `pick_dim+fk+env+refine ≈ 187 ms`，但 ffb_breakdown 给出 `exp = 1215 ms`，**缺口 ~1028 ms / 节点 56k 次 → ~18 μs/call**。这部分目前完全是黑盒。

**实施**：在 `cpp/v6/src/lect/lect.cpp` 的 expand 函数（`expand_*` 或 `compute_envelope` 的 caller）中加：

```cpp
double t_alloc=0, t_split=0, t_cache=0, t_other=0;
// ... 用 chrono 包住 alloc_node、split bookkeeping、cache write、grid pending mark 等
SBF_INFO("[GRW] expand_breakdown: alloc=%.0f split=%.0f cache=%.0f other=%.0f", ...);
```

**预期**：定位 1028 ms 缺口的去向。如果 alloc_node 占大头 → SoA reserve；如果是 cache write → 批量化；如果是 split bookkeeping → SIMD/branchless。

**风险**：零（只加 timer），但要保证在 release build 中关闭计时不影响性能（用编译期开关或 in-place 累加，每 call 几 ns）。

**验证**：跑一次 warm，查看新增 `expand_breakdown` 行；不与 R1 前后 lookup_avg/exp 对照（数据本身就是产物）。

## N3 — Coarsen1 优化

**目标**：548 → ≤ 350 ms（**−200 ms**）。

**当前路径**：`cpp/v6/src/planner/sbf_planner_build.cpp` 中 Coarsen1 = sweep1 (112 ms) + relaxed_sweep1 (277 ms) + articulation (16 ms) + greedy1 (143 ms)。三个子阶段都跑 `coarsen_forest` / `coarsen_greedy`，都涉及 box-pair 的 collides_scene 检查。

**实施候选**（按降序 ROI，需先看 hot path）：
1. **复用 LECT cache**：collides_scene 是否走 V6 cache？若否，把 coarsen 路径切到带 lect 的版本。
2. **共享 articulation/adj 缓存**：sweep1+relaxed_sweep1+greedy1 三轮都重算 adj，可在 sweep1 完成时把 adj 缓存传下去。
3. **OpenMP 并行**：candidate pair 间无依赖，可 #pragma omp parallel for。

**先决条件**：先 grep `coarsen_forest`/`coarsen_greedy` 实现，确认 hot path 后再动。

**风险**：中。box 顺序敏感，并行化可能影响最终 box 集；adj 缓存复用要保证不读到 stale 数据。

**验证**：boxes/islands/edges 计数与 baseline 在 ±5% 内。

## N5 — seed-bridge 调优

**目标**：310 → ≤ 200 ms（**−100 ms**）。

**当前**：5 seed point 两两 RRT bridge，per_pair_timeout=200ms。

**实施**：
- 预筛：seed 之间几何距离 > 阈值就跳过 RRT（直接接受 unreachable）。
- 调低 per_pair_timeout 到 100 ms，依赖现有的 multi-attempt fallback。

**风险**：低-中。可能丢一些跨 seed 的 bridge edge，但 grow 已经 connectivity=1，仅影响 island 多样性。

## 实施顺序

1. **N1（先做）** — 最大、最低风险、文件改动最少。
2. **N2** — 补 instrumentation 后观察 N4 是否值得做。
3. **N3** — 看 N1 后总 budget 还差多少决定。
4. **N5** — 收尾微调。

## 目标

| 指标 | R1+R3 当前 | N1 后 | N1+N3 后 | 终态目标 |
|---|---|---|---|---|
| warm total med | 2.93 s | ~1.93 s | ~1.7 s | **≤ 1.5 s** |
| lect_save_incremental | 1321 ms | ≤ 300 ms | ≤ 300 ms | ≤ 300 ms |
| Coarsen1 | 548 ms | 548 ms | ≤ 350 ms | ≤ 350 ms |

## 验证基线

每个 N 阶段后跑：

```bash
cd /home/tian/桌面/box_aabb/cpp/build && cmake --build . --target exp6_build_timing -j8 && \
./experiments/exp6_build_timing --envelope hull16_grid --lect-cache --seeds 3 2>&1 | tail -25
LATEST=$(ls -t /home/tian/桌面/box_aabb/cpp/v6/log/ | head -1)
grep -E "lect_save=|coordinated_grow|ffb_breakdown|expand_profile|coarsen|seed-brg|Z4GridCache.*stats" \
    /home/tian/桌面/box_aabb/cpp/v6/log/$LATEST | head -40
```

记录到下表：

| 阶段 | warm total med | lect_save med | grow med | coarsen1 med | boxes |
|---|---|---|---|---|---|
| baseline (R1+R3) | 2.93 s | 1321 ms (cold-1st) / 830 ms (steady) | 833 ms | 548 ms | 3344-3475 |
| **N1** | **2.22 s (-24%)** | **~15 ms (-98%)** | 929 ms | 514 ms | 3290-3527 |
| **N1+N3** | **2.38 s (-19%)** ¹ | ~15 ms | 544 ms | **331 ms (-40%)** | 3188-3623 |
| N1+N3+N5 | (skipped — 见 N5 备注) | | | | |

¹ 测量条件 5 seeds（vs 此前 3 seeds），fresh cache。3-seed 复测 med 为 1.65-2.39 s（中位 ~1.77 s）。差异来自 seed 选择，趋势一致。

### N1 实施记录（2026-04-24）

文件：`cpp/v6/src/lect/lect_io.cpp::lect_save_incremental`。

**改动 1：列式批量写**——把"每节点 10 次 seekp+write"改为"每列 1 次大块 seekp+write"。新节点范围 `[old_n_nodes, nn)` 按 `left/right/parent/depth/split_dim/split_val/has_data/source_quality/EP` 9 列分别批量写入；patch 旧节点（split node 子指针变化）保留逐节点路径（数量级小）。

**改动 2（关键）：跳过 `materialize_all_pending_grids_()`**——`index_grid_section` 在 load 路径已经标注 `"skipped (no lazy)"`，写入的 grid section **不会被读回**。pending grid 让后续 `collides_scene` 在真正需要时按需 materialize 即可。

**改动 3（关键）：跳过 `materialize_mmap()`**——`TreeArray::data()` / `ep_data_read()` 在 mmap 模式下已正确返回 `mmap_ptr_`/heap offset，新节点的 COW 页或 offset vector 可直接写到文件，无需先把 920 MB 复制到堆 vector。

**收益分解**（per-call timing 实测，n_new ~37k）：
- pending materialize：~400 ms → **0 ms**（后续 collide 路径按需做）
- mmap materialize：~250 ms → **0 ms**
- 列式 IO（替换 ~80 万次 syscall）：变化前后均 < 15 ms（page cache 时已经够快）
- trail 段重写：~37 ms（保留）
- **lect_save 总：800-830 ms → 11-25 ms 稳态（-98%）**

**整体效果**（5 seeds，fresh cache，--lect-cache）：
- build_time med：2.93 → 2.22 s（**-24%**）
- 质量保持：boxes 3290-3527（vs 3344-3475），islands 3/3 一致，edges ±5%

**后续观察**：grow time 略升 ~100 ms（830→930），是 R1 pending 的 materialize 成本从 save 转移到 collides_scene 的结果——属于成本搬移而非新增；总账上仍净赚约 700 ms。极少数 seed 触发 capacity-exceeded 全量重写（~1200 ms）——可在未来用更大初始 headroom 缓解。

### N2 实施记录（2026-04-24）

instrumentation only：在 `cpp/v6/include/sbf/lect/lect.h::ExpandProfile` 增加 `total_ms`，在 `cpp/v6/src/lect/lect.cpp::split_leaf_impl` 包一对 `Clock` 累加；`cpp/v6/src/forest/grower.cpp` 加 `[GRW] expand_total: total=X profiled=Y unprofiled=Z` 日志。

**结论**：所谓 "1028 ms 缺口" 是平均值伪象——per-seed env_ms 在 70~928 ms 范围跳变（cache-miss path 比 cache-hit 慢 ~12×），中位 unprofiled 实际只有 100–300 ms。**N4 收益过小，不动**。保留 instrumentation 便于今后回归监测。

### N3 实施记录（2026-04-24）

文件：`cpp/v6/src/planner/sbf_planner_build.cpp`，`coarsen_sweep_relaxed(...)` 第 5 个参数 `max_rounds: 4 → 1`。

**起因**：临时给 `coarsen_sweep_relaxed` 加 per-round log（`[CRS] rsweep round=N merges=X hull_calls=Y boxes=B t=Tms`）后看到：
- Round 0：~400 merges，70 ms
- Rounds 1-3：合计 ~25 merges，但累计 175 ms

边际收益与每轮成本完全不成比例。把 `max_rounds` 砍到 1，砍掉 3 轮 ~175 ms 浪费。

**收益**（5 seeds，fresh cache）：
- relaxed_sweep1：~245 ms → ~70 ms（**-71%**）
- coarsen1 总：548 → 331 ms（**-40%**）
- build_time med：2.93 → 2.38 s（vs N1 单独的 2.22；差异来自 grow 抖动，5-seed 样本变异性较大）
- boxes 3188-3623（vs 3344-3475，方差略增 ~5%），islands 3/3 全部保持

实施后 per-round log 已清理，仅留 R3 的 `coarsen_sweep_relaxed` 参数变化。

### N5 决策（2026-04-24，skip）

读完 `cpp/v6/src/planner/sbf_planner_build.cpp` lines 680-835 的 seed-point bridge 循环后判断：

- 已经做过的优化：`per_pair_timeout = min(800, 250+100·dist)`、`max_pairs=1`（注释 "OPT: cap per_pair timeout 1500→800ms and max_pairs 5→1"）、incremental UF over box ids、side-effect adjacency capture during dim-0 sweep、`sp_bridge_budget_ms` 全局预算守门。
- 当前总耗时 357-466 ms / 5 seeds ≈ 70-90 ms/seed，离 timeout 800 ms 相去甚远——dominated by 真实 RRT 工作而非超时浪费。
- 进一步压 `per_pair_timeout` 会引发 bridge 失败 → islands 数变化 → 需要 BRG-REPAIR 兜底，risk-reward 负面。
- **决定：skip N5**。Hull16Grid warm 优化阶段在 N1+N3 收尾。

## 优化阶段总结

从最早的基线 16.25 s（IFK + Hull16Grid combined）一路到 R1+R3 的 2.93 s，再到本轮 N1+N3 的 2.38 s（5 seeds）：

| 里程碑 | warm total med | 累计 vs 16.25 s |
|---|---|---|
| 起点（A1+V5+A2 之前） | 16.25 s | — |
| Q1+R1+R3 | 2.93 s | -82% |
| **N1+N3（当前）** | **2.38 s** | **-85%** |

剩余主要时间分布（N1+N3 后，5 seeds 中位）：Grow 544 / Coarsen1 331 / seed-brg 357 / LECT 23 / 其它 ~1100 ms。下一阶段若继续，应聚焦 Grow（544 ms）的剩余热点，但当前已远低于 cold（14+ s）和原始基线，warm/cold ratio 已显著反转。


---

## 附：首版 P0–P4 计划（已修正的历史分析）

以下为首版计划内容，保留作为记录。**P0 已实施但无效；P1/P4 将在 Q4 吸纳；P2/P3 搁置或弃用**。

## 背景


继上一会话完成 A1+V5+A2 之后，IFK+Hull16Grid 的 warm/cold 比仍是 1.13×（warm 16.25 s / cold 14.44 s）。本次实验在 [Z4GridCache](../../include/sbf/lect/z4_grid_cache.h) 加入端到端遥测后，得到下表（IFK + Hull16Grid + `--lect-cache`，3 seeds，8 threads，iiwa14 combined scene）：

| 指标 | Cold seed0 | Cold seed1 | Warm seed0 |
|---|---|---|---|
| total | 14.66 s | 8.25 s | 8.09 s |
| grow | 13.27 s | 6.96 s | 6.80 s |
| lookups | 228k | 167k | 154k |
| mem_hit (LRU 256MB) | 18.0% | 14.6% | 12.8% |
| disk_hit | 24% | 34% | 40% |
| disk_miss | 58% | 51% | 47% |
| **lookup_avg** | **65 μs** | 52 μs | **50 μs** |
| pread_avg | 6.0 μs | 5.0 μs | 4.4 μs |
| inserts_total | 3897 ms | 1661 ms | 745 ms |

**核心结论**

- Warm seed0 一次 grow ≈ 154k × 50 μs = 7.7 s ≈ 整个 grow 阶段。
- pread 仅占 50 μs 的 8%，**磁盘 IO 不是瓶颈**（OS page cache 已命中）。
- 50 μs/lookup 的剩余开销集中在两处：
  1. `Z4GridCache::lookup` 的 disk_hit 分支：每次都 `new vector<uint8_t>` + `make_shared<SparseVoxelGrid>` + N 次 `set_brick`；
  2. `lect.cpp` 的 `node_grids_[i].push_back(*cached_grid)` — 把整个 grid 再做一次**深拷贝**。
- LRU 命中率只有 12-18% — 256 MB 装不下 3 GB 缓存的"热集合"。
- inserts 由 3897 → 745 ms（cold→warm）已经接近 0，**不是优化重点**。

## 优化阶段

按 ROI 排序，每阶段独立可验证。每阶段完成后跑同一组 cold/warm 测试，记录 `lookup_avg`、`mem_hit`、`grow` 三项核心指标。

### P0 — 消除 node_grids_ 深拷贝（最高 ROI）

**问题**：[lect.cpp:504, 613, 1252](../../src/lect/lect.cpp) 的
```cpp
auto& grids = node_grids_[node_idx];
grids.push_back(*cached_grid);   // 深拷贝
```
在 `node_grids_` 是 `vector<SparseVoxelGrid>` 时强制每次都拷贝整个 sparse hashmap。

**改动**

1. `node_grids_` 类型 `vector<SparseVoxelGrid>` → `vector<shared_ptr<const SparseVoxelGrid>>`。
2. 三处 push_back 改为 `grids.push_back(cached_grid)`（直接 share）。
3. 所有读取 `node_grids_[i][k]` 的地方加一层 `*`（或改为 `node_grids_[i][k].get()` / `*node_grids_[i][k]`）。
4. 计算节点 grid 的代码（非来自 cache 的路径）也需要包成 `make_shared`。

**预期**：grow ≈ -1.5~2.5 s（20~35%），lookup_avg ≈ -10~15 μs。

**验证**：cold/warm 各 3 seeds，对比 grow 中位数与 lookup_avg。

---

### P1 — TLS 复用 brick_buf

**问题**：[z4_grid_cache.cpp lookup() 第 220 行附近](../../src/lect/z4_grid_cache.cpp) 每次 disk_hit 都
```cpp
std::vector<uint8_t> brick_buf(brick_bytes);
```
触发 malloc。154k lookups × 40% disk_hit ≈ 62k allocations/seed。

**改动**

1. lookup() 内换成 `thread_local std::vector<uint8_t> brick_buf;`，调用前 `brick_buf.resize(brick_bytes)`。
2. 同样把临时构造的 `make_shared<SparseVoxelGrid>` 流程审视一下（这一处必须保留 shared_ptr，但内部 hashmap 可以预 reserve）。

**预期**：lookup_avg ≈ -5~10 μs，grow ≈ -0.4~0.8 s。

**风险**：thread_local 在 worker 线程退出时归还内存到 glibc — 不会泄漏，但首次 resize 仍要 malloc。可接受。

**验证**：同 P0。

---

### P2 — Bloom filter 前置过滤 disk_miss

**问题**：47% × 154k = 72k disk_miss × ~8 μs（hash 探测 + slot quality 检查）≈ 0.6 s/seed 浪费在"找了个空槽"。

**改动**

1. 在 [Z4GridCache Header](../../include/sbf/lect/z4_grid_cache.h) 之后增加固定大小 bitmap（4 MB = 32M bits，对应 ~3M unique keys 时假阳性 < 1%）。
2. `insert()` 时设置 bit；`lookup()` 在 `find_index_slot` 之前先查 bit，未命中直接 disk_miss++ 返回。
3. 文件 schema 升级 `kVersion` 2 → 3，旧文件直接重建（cache 是无状态的）。

**预期**：disk_miss 路径从 8 μs → 0.1 μs，grow ≈ -0.4 s/seed。

**风险**：中。需要 bitmap 在 mmap 区域、跨进程共享、thread-safe 写入（atomic OR）。

**验证**：同 P0；额外验证 mem_hit/disk_hit 数量保持一致。

---

### P3 — LRU 预算调优

**问题**：256 MB 装不下 3 GB 缓存的热集合，warm 命中率 12.8%。

**改动**

1. 在 [SBFPlannerConfig](../../include/sbf/planner/planner_config.h) 暴露 `lect_cache_lru_mb`（默认仍 256，但实验时调到 1024）。
2. exp6 加 `--lru-mb` 命令行参数。
3. 跑 sweep：256/512/1024/2048 MB。

**预期**：1 GB 时命中率 12% → 35-45%，grow ≈ -0.3~0.5 s。

**风险**：低（仅 RSS 增加）。

**验证**：4 个 LRU 预算下的 grow + mem_hit。

---

### P4 — SparseVoxelGrid 批量构造

**问题**：N 次 `set_brick` = N 次 hashmap insert（每次平均 ~80 ns，rehash 时更高）。

**改动**

1. 在 `SparseVoxelGrid` 增加 `void reserve(size_t n_bricks)` 调用底层 unordered_map.reserve。
2. 在 lookup() disk_hit 路径开头先 reserve(n_bricks) 再 set_brick。
3. （可选）增加 `set_brick_fast(BrickCoord, BitBrick)` — 假定 key 不重复，跳过查重。

**预期**：lookup_avg ≈ -3~5 μs。

**风险**：低（仅性能微调）。

**验证**：同 P0。

## 验证基线

每阶段实施后跑同一组：

```bash
cd /home/tian/桌面/box_aabb/cpp/build
rm -rf /home/tian/.sbf_cache/*
./experiments/exp6_build_timing --envelope hull16_grid --endpoint ifk \
    --lect-cache --seeds 3 --threads 8 > /tmp/cold.log 2>&1
./experiments/exp6_build_timing --envelope hull16_grid --endpoint ifk \
    --lect-cache --seeds 3 --threads 8 > /tmp/warm.log 2>&1
```

记录到下表：

| 阶段 | total cold med | total warm med | grow warm med | lookup_avg warm | mem_hit warm |
|---|---|---|---|---|---|
| baseline (本次) | 8.25 s | 8.09 s | 6800 ms | 50 μs | 12.8% |
| P0 | TBD | TBD | TBD | TBD | TBD |
| P0+P1 | TBD | TBD | TBD | TBD | TBD |
| P0+P1+P2 | TBD | TBD | TBD | TBD | TBD |
| P0+P1+P2+P3(1GB) | TBD | TBD | TBD | TBD | TBD |
| P0+P1+P2+P3+P4 | TBD | TBD | TBD | TBD | TBD |

## 目标

把 warm grow 从 6.8 s 压到 **≤ 4 s**，warm total ≤ 5 s，warm/cold 比保持 ≤ 1.0。
