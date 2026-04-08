# Phase B: Endpoint IAABB 管线 (Module 1)

> 依赖: Phase A (core/ + scene/)
> 状态: ✅ 已完成 (2026-04-04)
> 产出: 4 种 Endpoint Source 实现 + 统一接口

---

## 目标

实现 C-space 区间 → 工作空间 endpoint iAABB 的 4 种计算方法，提供统一的 `compute_endpoint_iaabb()` 入口。

---

## 管线概述

```
Joint intervals [n_joints × Interval]
        ↓
EndpointSource ∈ {IFK, CritSample, Analytical, GCPC}
        ↓
endpoint_iaabbs [n_active_links × 2 × 6 floats]
  每个 link 产生 2 个 endpoint (proximal + distal)
  每个 endpoint = [lo_x, lo_y, lo_z, hi_x, hi_y, hi_z]
```

### Safety Chain
```
IFK (SAFE, loosest) — 区间运算严格保守
Analytical (UNSAFE, tighter) — 可能遗漏高维内部极值
GCPC (UNSAFE, tightest) — 依赖缓存完整度
CritSample (UNSAFE) — 可能遗漏极值，不保证包含
```

---

## Step B1: IFK Source

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/envelope/ifk_source.h` | `src/envelope/ifk_source.cpp` |

### 接口
```cpp
EndpointIAABBResult compute_endpoint_iaabb_ifk(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    FKState* fk = nullptr,          // 可选: 复用/更新 FKState
    int changed_dim = -1            // ≥0 时启用增量 FK
);
```

### 算法
1. 若 `changed_dim >= 0` 且 `fk->valid`: 调用 `compute_fk_incremental()`
2. 否则: 调用 `compute_fk_full()`
3. 从 FKState 提取 endpoint iAABBs: `extract_link_iaabbs()`
4. 标记 `result.is_safe = true`

### 迁移来源
| 源版本 | 源文件 | 取什么 |
|--------|--------|--------|
| v1 | `v1/src/aabb/interval_fk.cpp` | 基础 IFK 算法 |
| v4 | `safeboxforest/v4/src/envelope/endpoint_source.cpp` | IFK 路径 + 增量 FK |

### 验收标准
- [x] 2DOF 输出维度正确: `[n_active × 2 × 6]`
- [x] 增量 FK 与 full FK 结果一致
- [x] MC 保守性: 1000 个随机 config 的 FK 点均在 iAABB 内

---

## Step B2: CritSample Source

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/envelope/crit_source.h` | `src/envelope/crit_source.cpp` |

### 接口
```cpp
EndpointIAABBResult compute_endpoint_iaabb_crit(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int n_samples = 1000,
    uint64_t seed = 42
);
```

### 算法（三阶段）
1. **关键点枚举**: 每个关节取 {lo, hi, kπ/2 within range} 的组合 → FK → 更新 AABB
2. **随机采样**: 在区间内均匀采样 `n_samples` 个 config → FK → 更新 AABB
3. **局部优化** (可选): 对每个 link 的每个坐标轴, L-BFGS-B 优化 → 更新 AABB
4. 标记 `result.is_safe = false` (UNSAFE — 可能遗漏极值)

### 迁移来源
| 源版本 | 源文件 | 取什么 |
|--------|--------|--------|
| v2 | `v2/src/aabb/strategies/critical.py` | 最完整的三阶段采样逻辑 |
| v4 | `safeboxforest/v4/src/envelope/endpoint_source.cpp` | CritSample 路径 C++ 部分 |

### 验收标准
- [x] 输出格式与 IFK 一致
- [x] AABB 通常比 IFK 更紧（但不保证包含）
- [x] `is_safe == false`

---

## Step B3: Analytical Source

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/envelope/analytical_source.h` | `src/envelope/analytical_source.cpp` |

### 接口
```cpp
EndpointIAABBResult compute_endpoint_iaabb_analytical(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int max_phase = 3              // 0=vertices only, 1=+edges, 2=+faces, 3=+interior
);
```

### 算法（多阶段）

| Phase | 方法 | 覆盖 | 复杂度 |
|-------|------|------|--------|
| 0 | kπ/2 顶点枚举 | 顶点极值 | O(2^n) |
| 1 | 1D 边: atan2 直接求解 | 边界边 | O(n × 2^(n-1)) |
| 2 | 2D 面: 8 次多项式根 | 边界面 | O(n² × ...) |
| 2.5 | Pair-constrained 1D+2D | Joint pair 约束 | O(coupled_pairs) |
| 3+ | Interior coordinate descent | 内部临界点 | Multi-start descent |

**v4 优化保留**:
- DH 系数直接提取 (bypass FK sampling + QR decomp)
- AA Gap Pruning: 跳过 link 若其 AA bounds 已在当前 AABB 内
- Candidate 去重: 移除重复临界点

### 迁移来源
| 源版本 | 源文件 | 取什么 |
|--------|--------|--------|
| v4 | `safeboxforest/v4/include/sbf/envelope/analytical_solve.h` | 完整多阶段声明 |
| v4 | `safeboxforest/v4/src/envelope/analytical_solve.cpp` | 完整实现 |

### 验收标准
- [x] Phase 0 结果 ⊆ IFK iAABB（更紧或相等）
- [x] `is_safe == false`
- [x] 2DOF Phase 1 与 MC 真值偏差 < 5%
- [x] AA Gap Pruning 实际跳过 link 数 > 0（Panda 场景, 0.1rad intervals, 1/5 links pruned）

---

## Step B4: GCPC Source

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/envelope/gcpc_source.h` | `src/envelope/gcpc_source.cpp` |

### 接口
```cpp
// 缓存管理
class GcpcCache {
public:
    static GcpcCache load(const std::string& path);
    void save(const std::string& path) const;

    // 查询: 给定区间, 返回区间内的预计算关键点
    std::vector<Eigen::VectorXd> lookup(const std::vector<Interval>& intervals) const;

private:
    // KD-tree 或 flat array 索引
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

EndpointIAABBResult compute_endpoint_iaabb_gcpc(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const GcpcCache& cache
);
```

### 算法
1. **Analytical boundary**: 调用 Phase 0-2 的 analytical solver 获得边界极值
2. **Cache interior**: 从 `GcpcCache` 查找区间内的预计算内部临界点
3. **合并**: boundary AABB ∪ interior points → 最终 AABB
4. 标记 `result.is_safe = false`

### 缓存格式（简化单层，去掉 v4 的 4-tier）
- 二进制 `.gcpc` 文件: header + flat array of critical points
- Metadata sidecar `.gcpc.meta` (JSON): robot fingerprint, n_points, generation config
- 对称性约简: q₀ 消除 (若 Z4 rotation), q₁ ∈ [0, π]

### 迁移来源
| 源版本 | 源文件 | 取什么 |
|--------|--------|--------|
| v4 | `safeboxforest/v4/include/sbf/envelope/gcpc.h` | 缓存结构 + 查询逻辑 |
| v4 | `safeboxforest/v4/src/envelope/gcpc.cpp` | 完整实现 |

### 与 v4 差异
- **去掉 4-tier**: 不区分 smoke/quick/standard/full, 统一为单层完整缓存
- **简化 lazy loading**: 一次性加载全部到内存
- **不兼容 v4 格式**: .gcpc 文件需要重新生成

### 验收标准
- [x] `GcpcCache::save` → `load` 往返一致
- [x] GCPC AABB ⊆ IFK AABB（更紧或相等）
- [x] `is_safe == false`

---

## Step B5: 统一 Endpoint Source 接口

### 文件
| 头文件 | 实现 |
|--------|------|
| `include/sbf/envelope/endpoint_source.h` | `src/envelope/endpoint_source.cpp` |

### 接口
```cpp
enum class EndpointSource : uint8_t {
    IFK         = 0,
    CritSample  = 1,
    Analytical  = 2,
    GCPC        = 3
};

struct EndpointSourceConfig {
    EndpointSource source = EndpointSource::IFK;
    int n_samples_crit = 1000;      // CritSample only
    int max_phase_analytical = 3;   // Analytical only
    const GcpcCache* gcpc_cache = nullptr;  // GCPC only
};

struct EndpointIAABBResult {
    std::vector<float> endpoint_iaabbs;  // [n_active × 2 × 6]
    EndpointSource source;
    bool is_safe;
    int n_active_links;
};

EndpointIAABBResult compute_endpoint_iaabb(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const EndpointSourceConfig& config,
    FKState* fk = nullptr,
    int changed_dim = -1
);
```

### Safety 替代规则
| 原始 Source | 可替代为 | 方向 |
|-------------|---------|------|
| IFK | — | 唯一 SAFE 源 |
| Analytical | IFK | ✓ 升级到 SAFE (更松) |
| GCPC | IFK | ✓ 升级到 SAFE (更松) |
| CritSample | IFK | ✓ 升级到 SAFE |
| SAFE → Analytical/GCPC/CritSample | **禁止** | ✗ 可能丢失极值 |

### 迁移来源
| 源版本 | 源文件 |
|--------|--------|
| v4 | `safeboxforest/v4/include/sbf/envelope/endpoint_source.h` |

---

## 测试: test_endpoint_iaabb.cpp

```
TEST_SUITE("IFK") {
    - 2DOF: 输出维度 [n_active × 2 × 6]
    - 2DOF: MC 保守性 (1000 random configs → 全在 iAABB 内)
    - 增量 FK: changed_dim=0 结果与 full 一致
}

TEST_SUITE("CritSample") {
    - 2DOF: 输出 is_safe == false
    - 2DOF: AABB volume ≤ IFK volume (通常更紧)
}

TEST_SUITE("Analytical") {
    - 2DOF: Phase 0 结果 ⊆ IFK
    - 2DOF: is_safe == true
    - 2DOF: Phase 1 与 MC 偏差 < 1%
}

TEST_SUITE("GCPC") {
    - cache save/load roundtrip
    - GCPC AABB ⊆ Analytical AABB
}

TEST_SUITE("unified interface") {
    - compute_endpoint_iaabb with each source
    - safety flag correctness
}
```
