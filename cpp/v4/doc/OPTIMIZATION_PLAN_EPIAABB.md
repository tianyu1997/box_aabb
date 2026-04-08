# epiAABB 四管线优化计划

> 日期：2026-03-22
> 基线 Benchmark（200 trials, seed 42, IIWA14, Release）：
>
> | 管线        | 耗时 (ms)  | VolRatio | 说明                  |
> |-------------|-----------|----------|-----------------------|
> | IFK         | 0.022     | 8.54     | 最快但最保守           |
> | CritSample  | 0.039     | 0.9943   | 边界 kπ/2 采样         |
> | GCPC        | 0.192     | 1.0000   | 缓存+边界采样           |
> | Analytical  | 16.04     | 1.0000   | 解析求根，最精确最慢    |

---

## 优化项总览

| 优先级 | 编号 | 优化项 | 影响管线 | 预估收益 | 风险 | 状态 |
|--------|------|--------|----------|----------|------|------|
| P1     | O1   | 非 IFK 管线跳过冗余 `fk_to_endpoints()` | CritSample, GCPC, Analytical | ~1-3μs/call | 极低 | ✅ 已完成 |
| P2     | O2   | GCPC Phase B: `std::function` → 模板 + 预计算 DH + 栈 DFS | GCPC | **较大** — Phase B 是 GCPC 热路径 | 中 | ✅ 已完成 |
| P3     | O3   | `endpoint_iaabbs` 从 `vector<float>` 改为栈数组 | 全部 4 管线 | 消除堆分配（48 floats = 192B） | 低 | ✅ 已完成 |
| P4     | O4   | `crit_angles()` 返回栈数组代替 `vector<double>` | CritSample, GCPC | 消除热路径堆分配 | 低 | ✅ 已完成 |
| P5     | O5   | Phase 1 去重：`std::set<vector<int>>` → 定长哈希 | CritSample+GCPC(Phase1) | 减少堆分配和比较开销 | 低 | ✅ 已完成 |
| P6     | O6   | `pre_tf[j][i]` 二维 vector → 扁平一维 | CritSample | 减少间接寻址 | 极低 | ✅ 已完成 |
| P7     | O7   | `LinkExtremes::configs[6]` 用定长数组替代 `VectorXd` | Analytical/GCPC | 消除 6 个堆分配/析构 | 低 | 暂缓 |

---

## O1: 跳过非 IFK 管线的冗余 `fk_to_endpoints()`

### 问题

`compute_endpoint_iaabb()` 中，CritSample / GCPC / Analytical 三条路径都先调用：
```cpp
result.fk_state = compute_fk_full(robot, intervals);
fk_to_endpoints(result.fk_state, robot, result.endpoint_iaabbs);   // ← 被覆盖
```
然后 `derive_crit_endpoints()` 会将 `endpoint_iaabbs` 重新初始化为 `±1e30f` 并重新填充，
导致 `fk_to_endpoints()` 的结果被完全丢弃。

### 方案

- CritSample / GCPC：保留 `compute_fk_full()`（需要 `fk_state` 供 `extract_link_iaabbs` 使用），
  移除 `fk_to_endpoints()` 调用；改为直接 `resize` + 不填充（由 `derive_crit_endpoints` 负责初始化）。
- Analytical：同理移除 `fk_to_endpoints()`。

### 涉及文件

- `v4/src/envelope/endpoint_source.cpp`：`compute_endpoint_iaabb()` 中 3 处 `fk_to_endpoints()` 调用

---

## O2: GCPC Phase B 模板化 + 预计算 DH + 栈 DFS

### 问题

`gcpc.cpp` 的 Phase B（L522）使用 `std::function<void(int)> enumerate` 做递归枚举，
每次迭代有虚分派开销，且每次 FK 调用都内含 sin/cos 计算。
CritSample Phase 2 已经做了同样的优化（模板 + 预计算 DH + 栈 DFS），GCPC 应同步。

### 方案

1. 将 GCPC Phase B 的 `enumerate` 递归改为栈 DFS（`Frame stack[16]`）。
2. 预计算 `pre_tf[j][i]` 避免 hot loop 中的 sin/cos。
3. 消除 `std::function` — 改为直接内联调用 `eval_config` lambda。

### 注意

GCPC Phase B 的 `eval_config` 不仅更新 endpoint iAABBs，还更新 link segment AABBs 和
`best_face_config` 跟踪，比 CritSample 的 `update_endpoint_iaabb` 更复杂。
需要保留完整的 `eval_config` 逻辑，只替换遍历方式。

### 涉及文件

- `v4/src/envelope/gcpc.cpp`：Phase B 段（约 L487–L573）

---

## O3: `endpoint_iaabbs` 栈数组

### 完成情况（2026-03-27）

- ✅ 已完成：`EndpointIAABBResult::endpoint_iaabbs` 已由动态容器切换为定长存储。
- ✅ 已完成：调用侧 `.size()/.empty()/data()` 语义已统一迁移到 `endpoint_iaabb_len()/has_endpoint_iaabbs()/endpoint_data()`。
- ✅ 已完成：相关测试与实验代码已同步适配并通过构建测试。

### 问题

`EndpointIAABBResult::endpoint_iaabbs` 是 `vector<float>`，最多 8 × 6 = 48 floats（192 bytes）。
每次创建 `EndpointIAABBResult` 都有一次堆分配。对于 IFK（0.022ms）来说，
堆分配的开销可能占有意义的比例。

### 方案

```cpp
struct EndpointIAABBResult {
    float endpoint_iaabbs[48];  // 固定 8 endpoints × 6, 栈分配
    int   n_endpoints = 0;
    // ...
};
```

### 注意

需要同时修改所有读取 `endpoint_iaabbs.data()` / `.size()` / `.empty()` 的地方。
`extract_link_iaabbs()` 中的 `!result.endpoint_iaabbs.empty()` 判断需要改为 `n_endpoints > 0`。

### 涉及文件

- `v4/include/sbf/envelope/endpoint_source.h`：struct 定义
- `v4/src/envelope/endpoint_source.cpp`：`fk_to_endpoints()`、`compute_endpoint_iaabb()`、`extract_link_iaabbs()`
- `v4/src/envelope/crit_sample.cpp`：接收 `float*` 已经兼容
- 所有引用 `result.endpoint_iaabbs` 的测试/实验文件

---

## O4: `crit_angles()` 栈数组

### 问题

`crit_angles()` 返回 `std::vector<double>`，每次调用堆分配。
IIWA14 有 7 个关节，每次 `derive_crit_endpoints` 调用 7 次 `crit_angles()`。
典型返回大小：2（边界）+ 0~4（kπ/2）= 2~6 个元素。

### 方案

返回栈分配的小数组：
```cpp
struct CritAngles {
    double v[16];  // 最多 16 个候选角度
    int n = 0;
};
inline CritAngles crit_angles(double lo, double hi) { ... }
```

### 涉及文件

- `v4/include/sbf/envelope/analytical_utils.h`：`crit_angles()` + `build_csets()`
- `v4/src/envelope/crit_sample.cpp`：Phase 2 调用处
- `v4/src/envelope/gcpc.cpp`：Phase B 调用处（如果仍然使用 `crit_angles`）

---

## O5: Phase 1 去重改用定长哈希

### 问题

`crit_sample.cpp` Phase 1 使用 `std::set<std::vector<int>>` 做去重，
每次插入需要 `vector<int>` 堆分配 + 红黑树节点分配 + O(n log n) 比较。

### 方案

改用 `std::unordered_set` + 定长 key（7 个 int → 直接计算哈希）或直接用
`std::array<int, 8>` + 自定义哈希。减少堆分配和比较开销。

### 涉及文件

- `v4/src/envelope/crit_sample.cpp`：Phase 1 去重段

---

## O6: `pre_tf` 扁平化

### 问题

`pre_tf` 是 `vector<vector<Matrix4d>>`（二维），内层 vector 导致额外堆分配和间接寻址。

### 方案

扁平为一维：`vector<Matrix4d> pre_tf_flat` + `int offsets[16]`。

### 涉及文件

- `v4/src/envelope/crit_sample.cpp`：Phase 2 预计算段

---

## O7: `LinkExtremes` 定长 config 数组

### 问题

`LinkExtremes::configs[6]` 是 6 个 `Eigen::VectorXd`，每个堆分配。
IIWA14 = 7D，84 bytes × 6 × n_act 次堆分配。

### 方案

```cpp
struct LinkExtremes {
    double vals[6];
    double configs[6][8];  // 最多 8 个关节
};
```

### 涉及文件

- `v4/include/sbf/envelope/analytical_utils.h`：`LinkExtremes` 定义
- 所有使用 `LinkExtremes::configs` 的 cpp 文件

---

## 执行顺序

1. **O1** — 最简单，风险最低，立即可做
2. **O3** — 影响面中等但收益高（消除所有管线的堆分配）
3. **O4 + O6** — 小改动，进一步消除堆分配
4. **O5** — Phase 1 去重优化
5. **O2** — 最大改动，GCPC Phase B 重写
6. **O7** — Analytical/GCPC 专属优化

每完成一项后：
- `cmake --build build --config Release` 零警告零错误
- 运行全部测试（580 tests: 38 + 172 + 370）
- 运行 benchmark 验证性能变化和 VolRatio 不变
- 更新 `doc/CHANGE_LOG_CN.md`
