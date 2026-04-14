# C-Series 加速计划

> **基线**: Build=12.47s, Query(5)=3.62s, Total=16.09s (T+B系列后)
> **目标**: Build→10.5-11s, Query→2.0-2.5s, Total→13-14s (-15~20%)
> **日期**: 2026-04-10
> 
> ### 最终结果 (2026-04-10)
> **Build=7.44s** (-40% vs T+B基线, -70% vs原始24.97s)
> **Query=3.38s** (-7% vs T+B基线, -68% vs原始10.59s)
> **Total=10.82s** (-33% vs T+B基线, -70% vs原始35.56s)
> 
> | 项 | 结果 | 收益 |
> |---|------|------|
> | C1 | ❌ 失败 | RRT compete对seg_res极敏感，路径质量严重退化 |
> | C2 | ✅ 通过 | 路径质量微提升，时间持平 |
> | C3 | ❌ 失败 | cancel杀死好trial，路径变差 |
> | C4 | ✅ **成功** | Build -5s，bridge 6.5s→2.5s |
> | C5 | ❌ 无效 | pre-check未触发，S5-A已足够 |

---

## C1: Query-time segment_resolution 20→10 [P0, ⭐]

**原理**: Build-time bridge (B3) 已验证 seg_res=10 安全且路径质量不变。Query-time 碰撞检查仍用 20，每次 `check_segment` 做 20 次 `check_config`，降至 10 可减半碰撞检查量。

**修改位置** (6处):

| # | 文件 | 行 | 上下文 |
|---|------|-----|--------|
| 1 | `include/sbf/planner/sbf_planner.h` | L91 | `rrt_segment_res = 20` → `10` (proxy RRT default) |
| 2 | `src/planner/sbf_planner.cpp` | L755 | Dijkstra-fail fallback RRT |
| 3 | `src/planner/sbf_planner.cpp` | L787 | extract-fail fallback RRT |
| 4 | `src/planner/sbf_planner.cpp` | L979 | **RRT compete** (主瓶颈!) |
| 5 | `src/planner/sbf_planner.cpp` | L1060 | segment repair RRT |
| 6 | `src/planner/sbf_planner.cpp` | L1437 | emergency RRT |

**预计收益**: Query -30% (~1100ms)
**风险**: Final validation 用 ares 高精度兜底，安全。
**状态**: ❌ **失败** — RRT compete 树拓扑对 seg_res 极度敏感。seg_res=10 导致 TS→CS 从 0.9s→3.3s（直接路径变差→触发 chain-path-optimization 失败）。seg_res=15 同样回退。仅改非 compete 的 5 处无收益（fallback RRT 几乎不触发）。

---

## C2: ares 分辨率松弛 0.005→0.01 [P2, ⭐]

**原理**: `ares(len) = max(20, ceil(len/0.005))` 对 1.0rad 段产生 200 次碰撞检查。EB 的 1765 moves 每次都做整段碰撞检查，成本高。松弛到 0.01 可减半检查但精度仍足够 (0.01rad ≈ 0.57°)。

**修改位置** (2处):

| # | 文件 | 行 | 上下文 |
|---|------|-----|--------|
| 1 | `src/planner/sbf_planner.cpp` | L895 | pre-compete simplify 的 ares_fn |
| 2 | `src/planner/sbf_planner.cpp` | L1081 | 主 ares (EB/greedy simplify/shortcut) |

**预计收益**: EB + shortcut 加速 ~15%, 约 -120ms query
**风险**: 对 KUKA 连杆尺寸 0.01 rad 仍足够精细。Final validation 独立校验。
**状态**: ✅ **通过** — 路径质量全部持平或微提升 (+0.003~0.007)。碰撞检查量减半但未体现为显著 wall-time 加速，因 simplify/EB/shortcut 不是主瓶颈。保留改动。

---

## C3: RRT compete 共享 cancel [P1, ⭐⭐]

**原理**: 3 parallel RRT compete 中，第 1 个成功的 trial 出结果后，另外 2 个仍运行到超时。CS→LB 中 2 个 trial 各超时 1s，白白浪费。共享 cancel 可让先完成的取消后续。

**修改位置**:

| # | 文件 | 行 | 修改内容 |
|---|------|-----|---------|
| 1 | `src/planner/sbf_planner.cpp` | ~L973 | 创建 `shared_ptr<atomic<bool>> cancel` |
| 2 | `src/planner/sbf_planner.cpp` | ~L974-985 | `run_trial` lambda 传入 cancel, 成功后 set cancel=true |
| 3 | `src/planner/sbf_planner.cpp` | ~L974 | 调用 `rrt_connect(..., cancel)` |

**预计收益**: -200~500ms query (CS→LB 改善最大)
**风险**: 无。Cancel 只是让其他 trial 提前退出。
**状态**: ❌ **失败** — cancel 把 "best of 3" 变成 "first of 3"，路径质量显著退化 (AS→TS 5.063→7.528, TS→CS 5.985→10.198)。bonus trial 已有 1s 短超时，cancel 节省的时间极少。

---

## C4: Bridge timeout 缩短 [P1, ⭐]

**原理**: Island #4 Round 0 的 4 个 RRT (68K iters, tree_b=26K+) 全 timeout at 3.5s。tree_b 已膨胀表明 free space 充分探索。Round 1 的 100K iters 同理。

**修改位置**:

| # | 文件 | 行 | 修改内容 |
|---|------|-----|---------|
| 1 | `src/forest/connectivity.cpp` | L881 | R0 timeout: `3500.0` → `2500.0` |
| 2 | `src/forest/connectivity.cpp` | L882 | R0 max_iters: `75000` → `50000` |
| 3 | `src/forest/connectivity.cpp` | L885 | R1 max_iters: `100000` → `60000` |

**预计收益**: -1~2s build
**风险**: 可能漏掉极困难 pair。当前场景 island #4 不可通过 RRT 连通（靠 post-adj），所以安全。
**状态**: ✅ **成功** — Build 10.5s→7.5s (-29%), bridge_all 6.5s→2.5s (-60%)。
  - 初次尝试 R1 max_iters=60000 失败 (islands=2, 关键 pair 需 ~71K iters)
  - 修正为 R1 max_iters=80000 → islands=1, 路径质量完全一致
  - 3次运行均稳定: boxes=9098, bridge=2371-2557ms

---

## C5: Pre-compete simplify 快速碰撞预检 [P2, ⭐]

**原理**: 3/5 查询 pre-compete simplify 被 REVERTED（config invalid）。在执行昂贵 simplify 前，先对 chain 几个采样点做 quick check_config，若多数碰撞则直接跳过。

**修改位置**:

| # | 文件 | 行 | 修改内容 |
|---|------|-----|---------|
| 1 | `src/planner/sbf_planner.cpp` | ~L890-910 | 在 simplify 循环前增加快速预检 |

**预计收益**: -80ms query
**风险**: 可能错过 simplify 有效的情况，但当前 3/5 无效。
**状态**: ❌ **无效** — 实现了 5-点内部采样 pre-check，但 3 个 REVERTED 链路的内部采样点碰撞率不超过半数，pre-check 从未触发。且 S5-A (inline config check) 已在 2-3 次跳跃后 early bail-out，实际浪费的碰撞检查量极小 (~5ms/query)。代码已回退。

---

## 执行顺序 (已完成)

1. **C1** → ❌ 失败 (seg_res敏感)
2. **C3** → ❌ 失败 (cancel杀好trial)
3. **C4** → ✅ **成功** (Build -29%, bridge -60%)
4. **C2** → ✅ 通过 (路径微提升，时间持平)
5. **C5** → ❌ 无效 (pre-check未触发)
6. 最终基准测试 → ✅ 完成

---

## 最终基准测试结果

**测试条件**: 3次运行，每次恢复cache baseline (`.baseline` snapshot)

```
Run 1: build=7.33s  boxes=9098  islands=1  bridge=2557ms
Run 2: build=7.44s  boxes=9098  islands=1  bridge=2371ms
Run 3: build=7.45s  boxes=9098  islands=1  bridge=2507ms
```

| Query | Time (median) | Length | Pts | 基线Length |
|-------|---------------|--------|-----|-----------|
| AS→TS | 0.41s | 5.060 | 12 | 5.063 |
| TS→CS | 0.92s | 5.978 | 7 | 5.985 |
| CS→LB | 2.10s | 8.920 | 11 | 8.920 |
| LB→RB | 0.04s | 4.788 | 6 | 4.792 |
| RB→AS | 0.025s | 1.961 | 6 | 1.966 |

**活跃改动**: C4 (connectivity.cpp bridge参数) + C2 (sbf_planner.cpp ares 0.01)

---

## 总体加速总结 (T+B+C 系列)

| 阶段 | Build | Query | Total |
|------|-------|-------|-------|
| 原始 | 24.97s | 10.59s | 35.56s |
| T+B系列后 | 12.47s | 3.62s | 16.09s |
| **C系列后** | **7.44s** | **3.38s** | **10.82s** |
| vs原始 | **-70%** | **-68%** | **-70%** |

---

## 验证标准

每项完成后运行:
```bash
cd /home/tian/桌面/box_aabb/cpp/build && cmake --build . -j$(nproc) && \
  ctest --test-dir . -R "test_collision|test_interval" --output-on-failure && \
  rm -rf /tmp/lect_cache_kuka* && \
  ./experiments/exp2_e2e_planning --seeds 1 --no-viz
```

验收标准:
- ✅ 编译无错误
- ✅ 单元测试全通过
- ✅ 5/5 queries collision-free
- ✅ 路径质量: 5.063 / 5.985 / 8.920 / 4.792 / 1.966 (允许 ±0.05)
- ✅ islands=1
