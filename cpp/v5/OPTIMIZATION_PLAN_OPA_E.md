# SafeBoxForest v5 — 后 adj 优化执行计划 (OP-A → OP-E)

**基准**: exp2 adj 优化后 (2026-04-13), 5 seeds, build_med=21.3s, SR=100%, islands=1

---

## OP-A: 缩短 grow timeout 至 7s（高优先级）

**目标**: Build 时间从 ~21s 降至 ~14s

**修改文件**:
- `experiments/exp2_e2e_planning.cpp` L90: `timeout_ms = 10000.0` → `7000.0`

**原理**: adj 优化后 pre-bridge islands 已稳定 5（完美），grow 10s 产出 35K boxes 远超必要。
7s 预计产出 ~25K boxes，足够维持 islands=5 的连通质量。

**验证**: exp2 --seeds 5, 检查 pre-bridge islands ≤ 5, SR=100%, build_med < 15s

---

## OP-B: sweep 提前终止（中优先级）

**目标**: sweep1 从 ~2s 降至 ~1s

**修改文件**:
- `include/sbf/forest/coarsen.h`: `coarsen_forest()` 加 `double target_ratio = 0.0` 参数
- `src/forest/coarsen.cpp`: `coarsen_forest()` 循环内加 early-exit: 当 `boxes.size() <= boxes_before * target_ratio` 时 break
- 同上: `coarsen_sweep_relaxed()` 加同样的 `target_ratio` 参数和 early-exit
- `src/planner/sbf_planner.cpp`: 调用处传入 `target_ratio = 0.5`

**原理**: sweep 的目标是去冗余，当已减至 50% 时大部分冗余已消除，继续扫描收益递减。

**验证**: boxes 数量变化合理，path quality 不变

---

## OP-C: 限制 adjacency 图密度 max_degree（中优先级）

**目标**: 平均度从 9.5 降至 ~6, query Dijkstra 加速 40%

**修改文件**:
- `include/sbf/forest/adjacency.h`: `compute_adjacency()` 加 `int max_degree = 0` 参数
- `src/forest/adjacency.cpp`: sweep 后对每个 box 的 adj list, 按 face area 降序排序, 截取 top-K

**方案**: 在 `compute_adjacency` 返回前加一个 pruning pass:
```cpp
if (max_degree > 0) {
    for (auto& [id, nbrs] : adj) {
        if ((int)nbrs.size() <= max_degree) continue;
        // Sort by face overlap volume (descending) → keep best K
        // 使用 shared_face 计算 face_area 太贵 → 改用 overlap_volume proxy
        // overlap_volume = Π(min(hi_d)-max(lo_d)) over non-shared dims
        ...
        nbrs.resize(max_degree);
    }
}
```

**验证**: Dijkstra query 时间降低, path quality 不变

---

## OP-D: isolated rejection 窗口自适应（低优先级）

**目标**: rejection 质量更稳定

**修改文件**:
- `src/forest/grower.cpp` L1793: `int scan_start = std::max(0, new_idx - 500)` → `std::max(0, new_idx - std::min((int)boxes_.size()/5, 1000))`

**原理**: 固定 500 在 box 少时过大（扫描浪费），box 多时可能不够。自适应更合理。

---

## OP-E: seed-point bridge max_boxes 限制（低优先级）

**目标**: 避免 seed-point bridge 产出上千 box 的极端情况

**修改文件**:
- `src/planner/sbf_planner.cpp` ~L688: `bridge_s_t()` 调用时加一个 per_pair_timeout_ms 限制
  - 或在 bridge_s_t 返回后检查 created > 500 时打 warn

**方案**: 将 seed-point bridge 的 `per_pair_timeout_ms` 从 3000 降至 1500，
同时 `max_pairs` 从 10 降至 5。这间接限制了总 box 数。

---

## 实施顺序

| 步骤 | 优化 | 预期收益 |
|---|---|---|
| 1 | OP-A (timeout 7s) | build 时间减半 |
| 2 | OP-C (max_degree) | query 加速 |
| 3 | OP-B (sweep early-exit) | build 再减 1-2s |
| 4 | OP-D (自适应窗口) | 稳定性 |
| 5 | OP-E (bridge 限制) | 极端 case 防护 |
| 6 | 全量验证 exp2 | 确认无回归 |
