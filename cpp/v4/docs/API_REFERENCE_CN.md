# SafeBoxForest v4 API 参考（中文）

---

## 目录

- [envelope/endpoint\_source — 端点源统一调度](#envelopeendpoint_source)
- [envelope/pipeline — 管线配置组合](#envelopepipeline)
- [envelope/envelope\_type — 连杆包络表示（Stage 2）](#envelopeenvelope_type)
- [voxel/voxel\_grid — 稀疏体素位掩码](#voxelvoxel_grid)
- [envelope/crit\_sample — 临界采样（CritSample）](#envelopecrit_sample)
- [envelope/gcpc — 全局临界点缓存（GCPC）](#envelopegcpc)
- [envelope/analytical\_solve — 解析求解器](#envelopeanalytical_solve)
- [envelope/analytical\_utils — 共享工具](#envelopeanalytical_utils)
- [envelope/analytical\_coeff — DH 系数直接提取](#envelopeanalytical_coeff)
- [forest/lect — 连杆包络碰撞树 (LECT)](#forestlect)
- [forest/node\_store — SoA 节点存储](#forestnode_store)
- [robot/iaabb\_config — DH→iAABB 预处理配置](#robotiaabb_config)

---

## envelope/endpoint_source

> 头文件：`include/sbf/envelope/endpoint_source.h`
> 实现：`src/envelope/endpoint_source.cpp`

### `EndpointSource` 枚举

Stage 1 端点源类别，4 种 iAABB 计算路径：

| 枚举值 | 编号 | 质量等级 | 安全类 | 说明 |
|--------|------|---------|------|------|
| `IFK` | 0 | 0 | SAFE | 区间 FK（最快，最保守） |
| `CritSample` | 1 | 1 | UNSAFE | 边界 kπ/2 采样（最快的采样方法，可能遗漏极值点） |
| `Analytical` | 2 | 2 | SAFE | 解析 P0–P3 梯度零点枚举 + AA 剪枝（最慢但纯在线） |
| `GCPC` | 3 | 3 | SAFE | 解析边界 P0–P2.5 + GCPC 缓存内部（推荐） |

> **质量排序**：IFK(0) < CritSample(1) < Analytical(2) < GCPC(3)。
> **安全类**：SAFE 产出保守外界（iAABB ⊇ 真实可达集）；UNSAFE (CritSample) 因采样可能遗漏极值。
> GCPC 在边界使用与 Analytical 相同的解析求解（P0+P1+P2+P2.5+AA剪枝），
> 内部用预计算缓存替代 P3 坐标下降，因此质量严格高于 Analytical。

### `EndpointSourceConfig`

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `method` | `EndpointSource` | `IFK` | 端点源选择 |
| `crit_config_ptr` | `const CriticalSamplingConfig*` | `nullptr` | CritSample 配置（可选） |
| `analytical_config_ptr` | `const AnalyticalCriticalConfig*` | `nullptr` | Analytical 配置（可选） |
| `gcpc_cache` | `const GcpcCache*` | `nullptr` | GCPC 缓存指针（不拥有生命周期） |

工厂方法：

- `EndpointSourceConfig::ifk()` — iFK 路径
- `EndpointSourceConfig::crit_sampling()` — CritSample 路径
- `EndpointSourceConfig::analytical()` — Analytical 路径
- `EndpointSourceConfig::gcpc(const GcpcCache* cache)` — GCPC 路径

### `EndpointIAABBResult`

| 字段 | 类型 | 说明 |
|------|------|------|
| `endpoint_iaabbs` | `vector<float>` | `[n_endpoints × 6]`，几何 iAABBs |
| `analytical_link_aabbs` | `vector<float>` | `[n_active × 6]`，Analytical 直出连杆 AABBs |
| `n_endpoints` | `int` | 端点数 = `n_joints + has_tool` |
| `n_active` | `int` | 活跃连杆数 |
| `fk_state` | `FKState` | iFK 状态（供增量计算） |
| `n_evaluations` | `int` | 临界配置评估次数 |

### 主要函数

```cpp
// Stage 1 入口：C-space 区间 → endpoint iAABBs
EndpointIAABBResult compute_endpoint_iaabb(
    const EndpointSourceConfig& config,
    const Robot& robot,
    const std::vector<Interval>& intervals);

// 增量入口：利用父节点 FKState 加速
EndpointIAABBResult compute_endpoint_iaabb_incremental(
    const EndpointSourceConfig& config,
    const FKState& parent_fk,
    const Robot& robot,
    const std::vector<Interval>& intervals,
    int changed_dim);

// 从 endpoint iAABBs 提取 per-link iAABBs（含半径扩展）
void extract_link_iaabbs(
    const EndpointIAABBResult& result,
    const Robot& robot,
    float* out_iaabb);  // [n_active × 6]
```

---

## envelope/pipeline

> 头文件：`include/sbf/envelope/pipeline.h`

### 设计概述

`PipelineConfig` 组合 Stage 1（EndpointSource）与 Stage 2（EnvelopeType）配置。
将 `endpoint_source.h` 与 `envelope_type.h` 组合到一个结构体中，减少头文件依赖。

### `PipelineConfig`

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `source` | `EndpointSourceConfig` | IFK | Stage 1 端点源配置 |
| `envelope` | `EnvelopeTypeConfig` | — | Stage 2 包络类型配置 |
| `aa_crossover_width` | `double` | `0.5` | AA/IA 混合交叉阈值（区间宽度 ≤ 时用 AA） |

### 工厂方法

**GCPC 工厂（推荐，需预构建 GcpcCache）：**

| 工厂 | 源 | 包络 | 用途 |
|------|-----|------|------|
| `recommended(cache)` | GCPC | Hull16_Grid | 速度+质量最优（推荐） |
| `tightest(cache)` | GCPC | Hull16_Grid | 最紧界（GCPC 比 Analytical 更紧） |
| `production(cache)` | GCPC | Hull16_Grid | 生产默认 |

**无缓存工厂（Legacy，不需 GCPC 缓存时的备选）：**

| 工厂 | 源 | 包络 | 用途 |
|------|-----|------|------|
| `recommended()` | CritSample | Hull16_Grid | 无缓存时推荐 |
| `tightest()` | Analytical | Hull16_Grid | 无缓存时最紧 |
| `production()` | IFK | Hull16_Grid | 无缓存生产默认 |
| `fast()` | IFK | LinkIAABB(sub=1) | 最快（认证用） |

```cpp
// 示例
auto cache = build_gcpc_cache(robot);
auto cfg = PipelineConfig::recommended(&cache);  // GCPC + Hull16_Grid
auto cfg_legacy = PipelineConfig::recommended();  // CritSample + Hull16_Grid
auto cfg_fast = PipelineConfig::fast();           // IFK + LinkIAABB
```

---

## envelope/envelope_type

> 头文件：`include/sbf/envelope/envelope_type.h`
> 实现：`src/envelope/envelope_type.cpp`

### 设计概述

Stage 2：从 Stage 1 的 `EndpointIAABBResult` 导出连杆包络表示。三种表示：

| 枚举值 | 编号 | 说明 |
|--------|------|------|
| `LinkIAABB` | 0 | 每连杆 iAABB（sub=n 参数化细分） |
| `LinkIAABB_Grid` | 1 | link iAABB 光栅化到 R³ byte grid |
| `Hull16_Grid` | 2 | 16 点凸包 → 稀疏 BitBrick VoxelGrid |

### `EnvelopeTypeConfig`

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `type` | `EnvelopeType` | `Hull16_Grid` | 包络类型 |
| `n_sub` | `int` | `1` | 每连杆细分段数（Hull16_Grid 固定为 1） |
| `delta` | `double` | `0.01` | Hull16_Grid 体素分辨率（米/体素） |
| `grid_R` | `int` | `64` | LinkIAABB_Grid 每轴体素数 |
| `world_bounds` | `float[6]` | `[-0.8,-1.2,0,1.8,1.2,1.4]` | LinkIAABB_Grid 世界范围 |

工厂方法：

- `EnvelopeTypeConfig::link_iaabb()` — LinkIAABB(sub=1)
- `EnvelopeTypeConfig::link_iaabb_grid()` — LinkIAABB_Grid(sub=16, R=64)
- `EnvelopeTypeConfig::hull16_grid()` — Hull16_Grid(delta=0.01, n_sub=1)

### `EnvelopeResult`

| 字段 | 类型 | 说明 |
|------|------|------|
| `link_iaabbs` | `vector<float>` | `[n_active × n_sub × 6]`，per-link iAABBs |
| `grid` | `vector<uint8_t>` | `[R³]` byte grid（LinkIAABB_Grid） |
| `grid_R` | `int` | byte grid 分辨率 |
| `hull_grid` | `voxel::VoxelGrid` | Hull16_Grid 稀疏体素网格（值类型） |
| `volume` | `double` | 体积（m³） |
| `n_voxels` | `int` | 占用体素数 |
| `n_bricks` | `int` | BitBrick 数（Hull16_Grid） |
| `valid` | `bool` | 是否有效 |

### `LinkIAABB(sub=n)` 语义说明

- `n_sub = 1` 时，`link_iaabbs` 为每条连杆 1 个完整 iAABB；它是对整条连杆工作空间的**保守外包围**。
- `n_sub > 1` 时，`link_iaabbs` 按 `[n_active × n_sub × 6]` 存储每条连杆的真实细分子段 iAABB；碰撞语义应理解为这些子段盒的**并集**。
- 因此，`sub=1` 的粗盒内部并不表示“每一点都必然被 `sub=n` 的某个子盒覆盖”。对于细长连杆，粗盒中心等内部点可能位于几何空区；`sub=n` 仅保证不漏掉**真实落在某个细分子盒覆盖范围内**的碰撞点。

### 主要函数

```cpp
// Stage 2 入口：从 endpoint iAABBs 导出连杆包络表示
EnvelopeResult compute_link_envelope(
    const EnvelopeTypeConfig& env_config,
    const EndpointIAABBResult& ep_result,
    const Robot& robot);

// 返回 (source, type) 对应的最优 EnvelopeTypeConfig
EnvelopeTypeConfig default_envelope_config(
    uint8_t source_enum, EnvelopeType type);
```

### Hull16_Grid 实现细节

**safety_pad = 0**：连杆半径直接传入 `fill_hull16()`，不额外添加 $\sqrt{3}\delta/2$ 填充（旧版使用 `safety_pad = \sqrt{3}\delta/2`，导致体积膨胀）。

**n_sub = 1 硬编码**：Hull16_Grid 的凸包扫描线对线性插值 iAABB 而言，子段并的凸包等于全段凸包，细分无效。

**自适应 delta**：当估算的总 YZ 扫描线数超过 `MAX_SCANLINES = 40000` 时，自动提升 delta：

$$\delta_{\min} = \sqrt{\frac{\sum_\ell \mathrm{ext}_y^\ell \cdot \mathrm{ext}_z^\ell}{\texttt{MAX\_SCANLINES}}}$$

若 $\delta_{\min} > \delta$，使用 $\delta_{\min}$ 以保障速度。

---

## voxel/voxel_grid

> 头文件：`include/sbf/voxel/voxel_grid.h`（header-only）
> 依赖：`include/sbf/voxel/bit_brick.h`

### 设计概述

`VoxelGrid` 封装稀疏 `FlatBrickMap`（开放寻址哈希映射，$8^3$ BitBrick 瓦片），提供：
- `fill_aabb()` — 轴对齐盒子填充
- `fill_hull16()` — 16 点凸包涡轮扫描线栅格化
- `merge()` / `collides()` — 位级 OR / AND
- `count_occupied()` / `occupied_volume()` — popcount 查询

### `FlatBrickMap`

开放寻址哈希映射（FNV-1a 键），70% 最大负载因子。
键 = `BrickCoord{bx,by,bz}`，值 = `BitBrick`（$8 \times$ `uint64`，512 体素 / 64 字节）。

### `VoxelGrid` 构造

```cpp
VoxelGrid(double delta,
          double ox = 0, double oy = 0, double oz = 0,
          double safety_pad = -1.0);
// safety_pad = -1.0 → 默认 √3·δ/2
// safety_pad =  0.0 → 不额外填充（Hull16_Grid 推荐）
```

### `fill_aabb(const float* aabb)`

填充与轴对齐盒子重叠的所有体素。

**行批量优化**：内层循环使用 `set_cell_range_x(cx0, cx1, cy, cz)` 替代逐体素 `set_cell(cx,cy,cz)`，一次 64-bit OR 填充整行，~2× 加速。

### `fill_hull16(const float* prox_iv, const float* dist_iv, double link_radius)`

栅格化 $\mathrm{Conv}(B_1 \cup B_2) \oplus \mathrm{Ball}(r)$。

**退化快速路径**：当 slopes $\approx 0$ 时退化为 `fill_aabb`。

**砖块批量 + 提前退出优化**（2026-03-23）：

1. **YZ 砖块瓦片循环**：外层按 $(b_z, b_y)$ 砖块瓦片遍历（每瓦片 $8 \times 8$ 扫描线），替代逐单元 $(c_z, c_y)$ 循环。
2. **砖块级提前退出**：对每个 $(b_y, b_z)$ 瓦片中心做膨胀半径测试 $r_{\text{test}} = r_{\text{eff}} + 3.5\sqrt{2} \cdot \delta$，若整个瓦片在凸包之外则跳过 64 条扫描线。
3. **栈本地 BitBrick 累积器**：每个 X 方向砖块使用 `BitBrick local_bricks[MAX_XB=128]`，扫描线内直接位操作，每 $(b_y,b_z)$ 瓦片只刷新一次到全局哈希映射——将哈希探测从 $O(\text{cells} \times x\_bricks)$ 降至 $O(yz\_bricks \times x\_bricks)$。
4. **超宽回退**：若 X 方向砖块数 > 128，回退到原始逐单元路径。

复杂度：$O(n_y \cdot n_z)$，每条扫描线 $O(1)$。

### 碰撞与合并

```cpp
bool collides(const VoxelGrid& other) const;  // 64-bit AND + early exit
void merge(const VoxelGrid& other);           // 64-bit OR，零损失合并
void intersect_inplace(const VoxelGrid& other); // 64-bit AND 原地交集
```

### 体积查询

```cpp
int count_occupied() const;      // popcount 统计占用体素
double occupied_volume() const;  // 占用体积 = count × δ³
int num_bricks() const;          // 分配的砖块数
```

---

## envelope/crit_sample

> 头文件：`include/sbf/envelope/crit_sample.h`
> 实现：`src/envelope/crit_sample.cpp`

### 设计概述

v4 的 CritSample 采用全新设计（与 v3 不同）：

- **v3**：三阶段流水线（coupled enum + manifold sampling + L-BFGS-B）
- **v4**：纯边界 kπ/2 采样（不依赖 GCPC 缓存，最快的采样方法）：
  - **默认模式 — boundary_only()**：仅启用 Phase 2，不使用 GCPC 缓存。CritSample 默认不需要 GCPC 缓存，可独立运行。
  - **Phase 2 — 边界 + kπ/2 枚举**：对每个关节调用 `crit_angles()` 获取 `{lo, hi, kπ/2 ∩ (lo,hi)}` 组合，通过 `build_csets()` 构建笛卡尔积（受 `max_boundary_combos` 上限控制），使用**预计算 DH 变换矩阵 + 栈式迭代遍历**评估（热路径内无 sin/cos 计算）。
  - 与 GCPC 的区别：GCPC = Phase 1（缓存查找）+ Phase 2（边界枚举）；CritSample = 仅 Phase 2。

### `CriticalSamplingConfig`

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enable_gcpc` | `bool` | `true` | 是否启用 Phase 1（GCPC 缓存查询） |
| `enable_boundary` | `bool` | `true` | 是否启用 Phase 2（边界 + kπ/2 枚举） |
| `max_boundary_combos` | `long long` | `60000` | 笛卡尔积上限 |
| `gcpc_cache` | `const GcpcCache*` | `nullptr` | GCPC 缓存指针（不拥有生命周期） |

工厂方法：

- `CriticalSamplingConfig::defaults()` — 默认（Phase 1 + Phase 2 均开启）
- `CriticalSamplingConfig::gcpc_only()` — 仅 Phase 1
- `CriticalSamplingConfig::boundary_only()` — 仅 Phase 2

### `CritSampleStats`

| 字段 | 类型 | 说明 |
|------|------|------|
| `n_gcpc_matches` | `int` | Phase 1 缓存匹配数 |
| `n_gcpc_fk` | `int` | Phase 1 FK 调用数 |
| `n_boundary_combos` | `int` | Phase 2 边界配置数 |
| `n_boundary_fk` | `int` | Phase 2 FK 调用数 |
| `phase1_ms` | `double` | Phase 1 耗时 (ms) |
| `phase2_ms` | `double` | Phase 2 耗时 (ms) |

### 主要函数

```cpp
// 主入口：计算 per-endpoint iAABBs
// 返回总 FK 评估次数
int derive_crit_endpoints(
    const Robot& robot,
    const std::vector<Interval>& intervals,
    const CriticalSamplingConfig& config,
    float* out_endpoint_iaabb,    // [n_endpoints × 6]
    CritSampleStats* out_stats = nullptr);
```

---

## envelope/gcpc

> 头文件：`include/sbf/envelope/gcpc.h`
> 实现：`src/envelope/gcpc.cpp`

### 设计概述

GCPC（Global Critical Point Cache）预计算 FK 位置的临界点，存储在 KD-tree 中供快速区间查询。

对称性约减：
- q₀ 完全消除（查询时通过 atan2 重建）
- q₁ 周期-π：仅存储 q₁ ∈ [0,π]；查询时镜像
- q₆ 条件跳过（当 d₆=0, a₆=0 且无工具时）

`derive_aabb_with_gcpc()` 八阶段流水线（含 Phase D½）：
- Phase A — 内部缓存查找（KD-tree 区间查询 + q₀/q₁ 重建）
- Phase B — 边界 kπ/2 枚举
- Phase C — 1D 边 atan2 求解（含两级 AA 剪枝）；使用 `extract_1d_coefficients` 直接系数提取（H2，0 FK）
- Phase D — 2D 面伴随矩阵求解（含 AA 剪枝）；使用 `extract_2d_coefficients` 直接系数提取（H3，0 FK）
- **Phase D½ — kπ/2 背景缓存查表**（H4 新增）：离线预计算的 `PairKpi2Config` 在查询时直接匹配区间，跳过 Phase E/F 对 kπ/2 内部值的枚举
- Phase E — 成对约束 1D（P2.5a：`extract_2d_coefficients` + 约束代入提取 α,β 系数，0 FK 调用；bg 仅 {lo,hi}）（H4 重写）
- Phase F — 成对约束 2D（P2.5b：预计算 DH 变换 + `build_symbolic_poly8` 多项式求解；H5-A 预计算逆矩阵、H5-B 增量链重算；bg 仅 {lo,hi}）（H4+H5 重写）
- Phase G — 内部坐标下降（查询局部，双模式）

**H5 Phase F 增量链重算**：bg 关节分类到 4 段（prefix / mid12 / mid23 / suffix），循环外预计算初始链与 products，循环内仅重算 dirty 链段，避免全链重构。QR solve 替换为预计算 `A_mat.inverse() * rhs`（~600→243 muls）。Phase F 耗时从 13.44ms 降至 7.38ms（−45%），GCPC 总计从 17.21ms 降至 10.91ms（−37%），所有宽度 GCPC 均快于 Analytical（比值 0.54–0.92）。

### `PairKpi2Config`（H4 新增）

离线预计算的 kπ/2 背景临界构型（Phase D½ 查表使用）。

| 字段 | 类型 | 说明 |
|------|------|------|
| `q[7]` | `double[7]` | 完整关节配置（最多 7 关节） |
| `n_joints` | `int` | 存储的关节数 |

由 `precompute_pair_kpi2()` 生成，存储在 `GcpcLinkSection::pair_kpi2_configs` 向量中。
查询时 Phase D½ 遍历这些预计算构型，检查是否落在当前查询区间内。

### `GcpcPoint`

| 字段 | 类型 | 说明 |
|------|------|------|
| `q_eff[7]` | `double[7]` | 有效关节角（q₁..qₖ，q₀ 不存储） |
| `n_eff` | `int` | 有效关节数 |
| `link_id` | `int` | 连杆 ID（0-based frame index） |
| `direction` | `int` | 0 = xy（径向 R²），1 = z |
| `A`, `B`, `C` | `double` | 预计算 FK 局部坐标 |
| `R` | `double` | √(A²+B²)，预计算径向距离 |

### `GcpcQueryResult`

| 字段 | 类型 | 说明 |
|------|------|------|
| `point` | `const GcpcPoint*` | 缓存点指针 |
| `q0_optimal` | `double` | 重建的 q₀ |
| `q1_reflected` | `bool` | 是否经过 q₁ 镜像 |
| `q1_actual` | `double` | 实际 q₁（镜像后） |

### `GcpcQueryStats`

完整的八阶段统计结构（含 Phase D½），包含：

| 阶段 | 统计字段（部分） |
|------|-----------------|
| Phase A | `n_cache_matches`, `n_q1_reflected`, `n_q0_valid` |
| Phase B | `n_boundary_kpi2` |
| Phase C | `n_boundary_atan2`, `n_aa_prune_checks`, `n_aa_pruned` |
| Phase D | `n_phase_d_faces`, `n_phase_d_fk`, `n_phase_d_pruned` |
| Phase D½ | `n_phase_d_half_checked`, `n_phase_d_half_matched`（H4 新增：kπ/2 缓存查表命中统计） |
| Phase E | `n_phase_e_pair1d`, `n_phase_e_fk`, `n_phase_e_pruned` |
| Phase F | `n_phase_f_pair2d`, `n_phase_f_fk`, `n_phase_f_pruned` |
| Phase G | `n_phase_g_interior`, `n_phase_g_fk`, `n_phase_g_pruned` |
| 总计 | `n_fk_calls` |
| 计时 | `phase_a_ms` .. `phase_g_ms`（含 `phase_d_half_ms`） |

### `GcpcCache` 类

```cpp
class GcpcCache {
public:
    // 构建
    bool load_json(const std::string& path, const Robot& robot);
    bool load(const std::string& path);     // 二进制格式
    bool save(const std::string& path) const;
    void build(const Robot& robot, const std::vector<GcpcPoint>& points);

    // 缓存充实：坐标下降发现内部极值点
    int enrich_with_interior_search(
        const Robot& robot,
        int n_random_seeds = 500,
        int max_sweeps = 5);

    // kπ/2 背景预计算（H4 新增）
    // 枚举所有 bg 关节取 kπ/2 值的临界构型，预存到各 link section
    // 由 enrich_with_interior_search 自动调用，或可在 build 后手动调用
    void precompute_pair_kpi2(const Robot& robot);

    // 查询
    void query_link(int link_id, const Interval* intervals,
                    std::vector<GcpcQueryResult>& results) const;

    // 八阶段 AABB 求解（含 Phase D½ kπ/2 缓存查表）
    void derive_aabb_with_gcpc(
        const Robot& robot,
        const std::vector<Interval>& intervals,
        int n_sub,
        float* out_aabb,               // [n_active * n_sub * 6]
        GcpcQueryStats* out_stats = nullptr,
        float* out_endpoint_iaabb = nullptr) const;

    // 访问器
    bool is_loaded() const;
    int  n_links() const;
    int  n_total_points() const;
    const GcpcLinkSection* find_section(int link_id) const;
    double d0() const;
};
```

---

## envelope/analytical_solve

### `AnalyticalCriticalConfig`（新增字段）

- `enable_candidate_dedup`：是否启用候选去重。
- `enable_adaptive_candidate_dedup`：是否启用自适应去重阈值。
- `candidate_dedup_min_candidates`：候选数达到该阈值后才考虑去重（自适应模式）。
- `candidate_dedup_warmup_sets`：命中率统计预热集合数；预热阶段按阈值执行去重。
- `candidate_dedup_min_hit_rate`：预热后若历史去重命中率低于该阈值，则跳过去重。
- `enable_parallel_by_link`：是否按 active link（`ci`）分块并行执行阶段求解（当前覆盖 P1/P2/P2.5/P3，含 `dual_phase3` 的第二次 interior pass）。
- `parallel_num_threads`：并行线程数；`<=0` 表示自动检测硬件并发数。
- `enable_p2_derivative_prototype`：P2 解析微分原型对照开关（仅采集对照统计，不改变求解结果）。
- `enable_p2_derivative_shadow_run`：O5.4 shadow run 开关（仅统计，不改变主求解写回）。
- `enable_p2_shadow_all_qi_branches`：O5.4c shadow 扩展候选统计开关；开启后对每个 root 统计全部可行 `qi` 分支（仅统计，不改变主求解行为）。
- `enable_p2_multi_qi_candidates`：O5.4d 真实候选生成优化开关；开启后 P2 对每个 root 收集全部落区间 `qi` 分支并交由现有 dedup 去重（默认关闭）。
- `p2_multi_qi_min_root_count`：O5.4g 条件门控参数；仅当单次 P2 case 的 root 数 `>=` 该阈值时，才允许启用多 `qi` 分支收集（默认 `1`，兼容 O5.4d 行为）。
- `p2_multi_qi_min_joint_width`：O5.4g 条件门控参数；仅当 `max(width_i,width_j)` `>=` 该阈值时，才允许启用多 `qi` 分支收集（默认 `0.0`）。
- `enable_p2_derivative_hybrid`：O5.5 受控切换主开关（默认关闭）。
- `enable_p2_hybrid_fallback`：O5.5 异常自动回退开关（默认开启）。
- `p2_hybrid_max_abs_error`：O5.5 回退阈值（候选差异最大绝对误差阈值，默认 `1e-6`）。
- `p2_proto_gate_fit_rms_thresh`：O5.3 门控统计中，拟合残差 RMS 的拒绝阈值（compare-only）。
- `p2_proto_gate_symbolic_rms_thresh`：O5.3 门控统计中，symbolic 系数 RMS 偏差拒绝阈值（compare-only）。
- `p2_proto_gate_symbolic_max_abs_thresh`：O5.3 门控统计中，symbolic 系数最大绝对偏差拒绝阈值（compare-only）。

### `AnalyticalCriticalStats`（新增字段）

AA 剪枝统计：

- `n_aa_pruned_links`：Phase 0 后按 link 级剪枝命中的 link 数（初始判定）。
- `n_aa_pruned_segments`：阶段内动态剪枝累计跳过的 segment 数（按每个 link 的真实 segment 可改进性判定累计）。
- `n_aa_pruned_phase_checks`：阶段内动态剪枝累计执行的 link 判定次数（每次 phase refresh 累加 `n_act`）。

阶段短路语义：

- 若某次 phase refresh 判定所有 active link 都不可改进，则该阶段直接短路跳过（该阶段统计为 0）。
- 若阶段未整体短路，solver 内部还会基于 AA 轴向可改进性执行细粒度短路：对判定不可改进的 axis / face 直接跳过候选构造与求解。

按阶段统计去重命中情况（P1 / P2 / P2.5 / P3）：

- `n_dedup_raw_candidates_*`：去重前候选总数。
- `n_dedup_unique_candidates_*`：去重后唯一候选总数。
- `n_dedup_applied_sets_*`：实际应用去重的候选集合数。
- `n_dedup_skipped_sets_*`：因自适应阈值/命中率策略而跳过去重的集合数。

其中 `*` 当前覆盖：`p1`、`p2`、`p25`、`p3`。

P2 解析微分原型（compare-only）统计：

- `n_p2_proto_cases`：P2 中完成对照统计的样本数（按 `(eval_idx, axis)` 计数）。
- `p2_proto_fit_rms_mean`：P2 拟合残差 RMS 均值（用于后续解析微分替代前的基线观测）。
- `p2_proto_fit_max_abs`：P2 拟合残差最大绝对值。
- `n_p2_proto_symbolic_cases`：P2 中完成“解析系数 shadow 重建 vs QR”对照的样本数（按 `(eval_idx, axis)` 计数）。
- `n_p2_proto_symbolic_singular`：解析系数 shadow 路径因局部基矩阵不可逆而回退统计（仅计数，不影响主求解路径）。
- `p2_proto_symbolic_coeff_rms_mean`：解析系数与 QR 系数差异的 RMS 均值（9 维系数向量）。
- `p2_proto_symbolic_coeff_max_abs`：解析系数与 QR 系数差异的最大绝对值。
- `n_p2_proto_gate_eval`：参与 O5.3 门控判定的样本数（按 `(eval_idx, axis)` 计数）。
- `n_p2_proto_gate_accept`：门控接受（理论可走解析路径）的样本数。
- `n_p2_proto_gate_reject_fit`：因拟合残差超阈值拒绝的样本数。
- `n_p2_proto_gate_reject_symbolic`：因 symbolic 系数偏差超阈值拒绝的样本数。
- `n_p2_proto_gate_reject_singular`：因 symbolic 局部基矩阵不可逆（奇异）拒绝的样本数。
- `n_p2_shadow_eval_cases`：O5.4 shadow run 中参与影子评估的样本数（gate-accept 且 shadow 开关开启）。
- `n_p2_shadow_root_count`：上述样本的 root 数累计。
- `n_p2_shadow_candidate_count`：上述样本的候选配置数累计。
- `n_p2_shadow_candidate_count_all_branches`：O5.4c 扩展口径候选数累计（每个 root 统计全部可行 `qi` 分支，仅用于 compare-only 评估上限）。
- `n_p2_hybrid_eval_cases`：O5.5 hybrid 路径评估样本数（骨架阶段用于可观测性打点）。
- `n_p2_hybrid_applied_cases`：O5.5 hybrid 实际应用样本数。
- `n_p2_hybrid_fallback_cases`：O5.5 因护栏触发回退的样本数。

可由下式得到命中率：

- `hit_rate = (raw - unique) / raw`

### `derive_aabb_critical_analytical(...)`

新增统计维度后，`out_stats` 可直接用于 A/B benchmark 输出：

- 运行时间（外部计时）
- 总 FK 调用与分阶段 FK 调用
- 去重命中率与去重应用/跳过次数

### `experiments/exp_analytical_ab_benchmark`（O5.3b 阈值扫描）

- 可通过环境变量覆盖 gate 阈值（不改代码）：
	- `SBF_P2_GATE_FIT_RMS`
	- `SBF_P2_GATE_SYMBOLIC_RMS`
	- `SBF_P2_GATE_SYMBOLIC_MAX_ABS`
- 可通过环境变量启用 shadow run 统计：
	- `SBF_P2_SHADOW_RUN`（`0/1`）
	- `SBF_P2_SHADOW_ALL_QI`（`0/1`，需配合 `SBF_P2_SHADOW_RUN=1` 使用）
- 可通过环境变量启用 O5.4d 多分支真实候选生成：
	- `SBF_P2_MULTI_QI_CAND`（`0/1`，默认 `0`）
- 可通过环境变量配置 O5.4g 条件门控阈值：
	- `SBF_P2_MULTI_QI_MIN_ROOTS`（整数，默认 `1`）
	- `SBF_P2_MULTI_QI_MIN_WIDTH`（浮点，默认 `0.0`）
- 可通过环境变量配置 O5.5 受控切换骨架：
	- `SBF_P2_DERIVATIVE_HYBRID`（`0/1`，默认 `0`）
	- `SBF_P2_HYBRID_FALLBACK`（`0/1`，默认 `1`）
	- `SBF_P2_HYBRID_MAX_ABS_ERR`（浮点，默认 `1e-6`）
- 可通过环境变量注入 benchmark width 列表（O5.4e 高分支定向样本）：
	- `SBF_BENCH_WIDTHS`（示例：`0.35,0.70,1.00,1.40`）
- CSV 会回显生效阈值列：
	- `gate_fit_rms_thresh`
	- `gate_symbolic_rms_thresh`
	- `gate_symbolic_max_abs_thresh`
- CSV 会回显优化开关列：
	- `enable_shadow_all_qi`
	- `enable_multi_qi_cand`
	- `multi_qi_min_roots`
	- `multi_qi_min_width`
	- `enable_hybrid`
	- `enable_hybrid_fallback`
	- `hybrid_max_abs_err`
	- `bench_width_profile`（`default` / `env_custom`）
---

## envelope/analytical_utils

> 头文件：`include/sbf/envelope/analytical_utils.h`

四大端点源路径（CritSample / Analytical / GCPC）共享的工具函数与数据结构。

### `FKWorkspace`

栈分配的标量 FK 工作空间（4×4 Eigen Matrix4d，最多 16 帧）。

```cpp
struct FKWorkspace {
    Eigen::Matrix4d tf[16];  // [0] = base, [1..n+1] = joint/tool frames
    int np;                   // active frame count

    void set_identity();
    void resize(int n_frames);
    void compute(const Robot& robot, const Eigen::VectorXd& q);
    void compute_prefix(const Robot& robot, const Eigen::VectorXd& q, int up_to);
    void compute_joint(const Robot& robot, const Eigen::VectorXd& q, int j);
    void compute_from(const Robot& robot, const Eigen::VectorXd& q, int from_j);
    Eigen::Vector3d pos(int frame) const;  // 提取位置列
};
```

### 临界角生成与笛卡尔积

```cpp
// 生成区间 [lo, hi] 内的临界角：{lo, hi, kπ/2 ∩ (lo,hi)}
std::vector<double> crit_angles(double lo, double hi);

// 笛卡尔积构建（含降级逻辑）
std::vector<std::vector<double>> build_csets(
    const std::vector<std::vector<double>>& per_joint,
    const std::vector<Interval>& intervals, int n);

// 递归临界配置枚举（模板化回调，零开销）
template<typename Fn>
void crit_enum(const std::vector<std::vector<double>>& csets,
               Eigen::VectorXd& q, int j, int n,
               const Fn& callback);

// 增量 FK 版枚举（模板化回调，零开销）
template<typename Fn>
void crit_enum_fk(const Robot& robot,
                  const std::vector<std::vector<double>>& csets,
                  Eigen::VectorXd& q, int j, int n,
                  FKWorkspace& ws,
                  const Fn& callback);
```

### `ConfigVec`

堆分配消除的关节配置向量（S9 优化）：

```cpp
// 固定容量 Eigen 向量（最多 16 关节），避免 VectorXd 的堆分配开销
using ConfigVec = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 16, 1>;
```

与 `Eigen::VectorXd` API 完全兼容（赋值、索引、`.size()` 等），但当 `n_joints <= 16` 时使用栈内联存储。

### `LinkExtremes`

per-link AABB 极值追踪器（含写入配置）：

```cpp
struct LinkExtremes {
    double vals[6];              // [x_min, x_max, y_min, y_max, z_min, z_max]
    ConfigVec configs[6];        // 对应极值的关节配置（栈内联存储）
    void init(int n_joints);     // 初始化极值为 ±inf，configs 为零向量
    void update(const Eigen::Vector3d& pos, const Eigen::VectorXd& q);
};
```

### 多项式求解

```cpp
struct FixedRoots { double v[8]; int n; };   // 固定容量根（最多 8 个）

FixedRoots solve_poly_in_interval(
    const double* coeffs, int degree,
    double lo, double hi);                    // 区间限制多项式求根

double half_angle_to_q(double t);             // 半角参数化 → 关节角

void build_bg_values(...);                    // pair-constrained 背景角度网格
```

---

## envelope/analytical_coeff

> 头文件：`include/sbf/envelope/analytical_coeff.h`

DH 链直接系数提取（零 FK 调用）。

### `Coeff1D`

1D 系数：三角模型 $p_d(q_j) = \alpha_d \cos q_j + \beta_d \sin q_j + \gamma_d$

```cpp
struct Coeff1D {
    double alpha[3], beta[3], gamma[3];  // per xyz axis
};

Coeff1D extract_1d_coefficients(
    const Eigen::Matrix4d& prefix,
    const Eigen::Vector3d& suffix_pos,
    const DHParam& dh_j);               // ~30 multiplications, 0 FK
```

### `Coeff2D`

2D 系数：双线性三角模型（9 系数 per axis）

$$p_d(q_i, q_j) = \sum_{k=0}^{8} a_{k,d} \cdot \phi_k(\cos q_i, \sin q_i, \cos q_j, \sin q_j)$$

```cpp
struct Coeff2D {
    double a[9][3];  // 9 coefficients × 3 axes
};

Coeff2D extract_2d_coefficients(
    const Eigen::Matrix4d& prefix,
    const DHParam& dh_i,
    const Eigen::Matrix4d& middle,
    const DHParam& dh_j,
    const Eigen::Vector3d& suffix_pos);  // ~120 multiplications, 0 FK
```

辅助函数：

```cpp
Eigen::Matrix4d compute_middle_matrix(
    const Robot& robot, const Eigen::VectorXd& q,
    int j_lo, int j_hi);

Eigen::Vector3d compute_suffix_pos(
    const Robot& robot, const Eigen::VectorXd& q,
    int j, int eval_frame);
```

> **S1 suffix 缓存优化**：`compute_suffix_pos` 仅读取 `q[j+1..eval_frame-1]`，
> 在 P1 bg 枚举中 suffix 关节角不变时可缓存复用，避免重复链乘。

---

## core/interval_trig

> 头文件：`include/sbf/core/interval_trig.h`

### 设计概述

DH α 分类工具函数，用于 DH 参数预处理。

### `classify_alpha(double alpha) → int`

DH α 分类：
- `0` → α ≈ 0（z 轴旋转）
- `1` → α ≈ −π/2（xz 平面旋转）
- `2` → α ≈ +π/2（xz 平面旋转，取反）
- `-1` → 不支持

---

## core/config

> 头文件：`include/sbf/core/config.h`

### `SplitOrder`

KD-tree 切分维度选择策略枚举：

| 值 | 名称 | 说明 |
|----|------|------|
| `0` | `ROUND_ROBIN` | 按 0→1→...→n-1 循环切分 |
| `2` | `WIDEST_FIRST` | 选择当前区间最宽的维度切分 |
| `3` | `BEST_TIGHTEN` | iFK 探针的 minimax：按深度懒惰选择使 `max(vol_left, vol_right)` 最小的维度；同深度复用同一维度 |

---

## forest/lect

> 头文件：`include/sbf/forest/lect.h`
> 实现：`src/forest/lect.cpp`

### `LECT` — 连杆包络碰撞树 (Link Envelope Collision Tree)

统一 KD-tree，缓存每节点连杆包络。基于两阶段模块化管线：

- **Stage 1**：端点 iAABB 生成（iFK / CritSample / Analytical / GCPC）
- **Stage 2**：连杆包络构造（LinkIAABB / LinkIAABB_Grid / Hull16_Grid）

**每节点缓存数据**：

| 层 | 存储 | 说明 |
|----|------|------|
| Per-link iAABBs | `NodeStore` | 每条活动连杆的 \[lo\_x, lo\_y, lo\_z, hi\_x, hi\_y, hi\_z\] |
| Endpoint iAABBs | `ep_data` (ChannelData) | 配对端点 iAABB \[n\_active × 2 × 6\]，按 \[prox0,dist0,prox1,dist1,...\] 排列 |
| Hull VoxelGrids | `hull_grids_` | 稀疏 BitBrick 体素网格 |

### 构造函数

```cpp
LECT(const Robot& robot, double voxel_delta = 0.02, int initial_cap = 1024);
LECT(const Robot& robot, const PipelineConfig& pipeline, int initial_cap = 1024);
```

构造时自动初始化根节点的 iAABB（覆盖全关节空间），当前代码默认使用 `BEST_TIGHTEN` 切分策略。
可通过 `set_split_order()` 切换到 `ROUND_ROBIN`、`WIDEST_FIRST` 或 `BEST_TIGHTEN`。

### 切分策略

```cpp
void set_split_order(SplitOrder so);   // 设置维度选择策略
SplitOrder split_order() const;        // 获取当前维度选择策略
```

### 核心操作

#### `find_free_box()`

```cpp
FFBResult find_free_box(const Eigen::VectorXd& seed,
                        const Obstacle* obstacles, int n_obs,
                        double min_edge = 1e-4, int max_depth = 30);
```

沿 KD-tree 向种子配置懒惰下降。流程：

1. **占用检查** — `is_occupied(cur)` → 返回 fail\_code=1
2. **iAABB 碰撞** — `iaabbs_collide()` 粗筛每条连杆与每个障碍物
3. **Hull-16 精炼** — 对 LinkIAABB\_Grid / Hull16\_Grid 类型，懒惰计算 `derive_hull_grid()` → `hull_collides()` 精细检测
4. **subtree\_occ 早停** — 无碰撞且子树无占用 → 立即返回 SUCCESS
5. **维度选取** — `pick_split_dim()` 统一决定二分维度（WIDEST\_FIRST / BEST\_TIGHTEN / ROUND\_ROBIN），与 `split_leaf()` / `pre_expand_recursive()` 共享同一逻辑
6. **叶节点分裂** — `split_leaf(cur, fk, intervals, dim)` 使用预选维度产生两个子节点，调用 `compute_envelope(child, fk, ivs, dim, parent)` 计算子节点包络
7. **向种子下降** — 增量 FK + `compute_envelope(child, fk, ivs, dim, cur)`

#### `compute_envelope()` — IFK 快速路径 + 部分 FK 继承

```cpp
void compute_envelope(int node_idx, const FKState& fk,
                      const std::vector<Interval>& intervals,
                      int changed_dim = -1, int parent_idx = -1);
```

为指定节点计算端点 iAABB 并导出连杆包络。三条执行路径：

1. **IFK 快速路径**（默认 iFK 源 + `fk.valid`）：
   - 直接从 `fk.prefix_lo/hi[V][3/7/11]` 提取端点坐标（V = `active_link_map[ci]`），完全跳过 `compute_endpoint_iaabb()`
   - **部分 FK 继承**（`changed_dim >= 0` 且父节点有缓存）：对 `active_link_map[ci]+1 <= changed_dim` 的未变连杆，直接从父节点 `ep_data` `memcpy`，仅对受影响帧从 FKState 提取
   - 无父节点时：从 FKState 全量提取全部活跃连杆端点
2. **Z4 缓存查找**（非规范扇区 + 缓存非空）：从 Z4 缓存变换得到端点
3. **通用回退**：调用 `compute_endpoint_iaabb()` → `derive_aabb_paired()`（用于 Analytical / GCPC / CritSample 等非 IFK 源）

#### `collides_scene()`

```cpp
bool collides_scene(int node_idx, const Obstacle* obstacles, int n_obs) const;
```

两阶段碰撞测试：iAABB 早停 → 懒惰 Hull-16 精炼。若 hull grid 未计算则按需生成。

#### `intervals_collide_scene()`

```cpp
bool intervals_collide_scene(const std::vector<Interval>& intervals,
                             const Obstacle* obstacles, int n_obs) const;
```

对任意 C-space 区间（不需要树节点）进行碰撞检测：`compute_endpoint_iaabb()` → `extract_link_iaabbs()` → iAABB 粗筛 → hull grid 精炼。

### 场景管理

```cpp
void set_scene(const Obstacle* obstacles, int n_obs);  // 光栅化障碍物为共享 VoxelGrid
void clear_scene();
bool has_scene_grid() const;
const VoxelGrid& scene_grid() const;
```

### 暖启动

```cpp
LECT snapshot() const;     // 深拷贝，清除占用
int pre_expand(int depth); // 递归预展开至目标深度
int compute_all_hull_grids(); // 批量生成 hull grid
int transplant_subtree(...);  // 并行生长后移植 worker 子树
```

### Hull 贪心粗化

```cpp
double merged_children_hull_volume(int node_idx) const;
double coarsen_volume_ratio(int node_idx) const;
bool   merge_children_hulls(int node_idx);
```

子节点 hull 按位 OR 合并，零损失体积度量。ratio = merged\_vol / max(left\_vol, right\_vol)。

### 持久化

```cpp
void save(const std::string& dir) const;
static LECT load(const std::string& dir, const Robot& robot);
```

存储布局：

| 文件 | 格式 | 内容 |
|------|------|------|
| `dir/lect.hcache` | HCACHE03 | 树结构 + per-link iAABBs（NodeStore） |
| `dir/lect.hulls` | HUL1 | hull-16 VoxelGrids（512 字节头 + 逐节点 brick 数据） |
| `dir/lect.meta.json` | JSON | pipeline 元数据（source\_method, envelope\_type, robot\_hash 等） |

### 统计

```cpp
int count_nodes_with_iaabb() const;
int count_nodes_with_hull()  const;
int total_hull_voxels()      const;
int scene_grid_voxels()      const;
```

---

## 森林生长策略（Growth Strategies）

> 当前实现位于 Python 实验层：`v4/experiments/bench_grower_strategies.py`
> **注意**：v4 核心库尚未将生长策略抽象为独立 public API 模块，下述接口仅为实验使用的内部函数。
> 未来计划将其提升为 `GrowStrategy` 接口集成到 `SBFPlanner` 中。

### 两种生长策略

#### Wavefront BFS（波前广度优先搜索）

多子树 round-robin BFS 扩展。每棵子树维护独立 `deque` 作为 BFS 队列。

```
算法流程:
1. 在 q_start / q_goal 处各放置一个锚 box（两棵子树）
2. Round-robin 轮询每棵子树：
   a. 从 BFS 队列弹出父 box
   b. 调用 _generate_boundary_seeds(box, rng, excluded_faces) 生成面种子
   c. 对每个种子调用 find_free_box() 尝试放置子 box
   d. 成功则加入 BFS 队列，记录 excluded_faces = {反面}
3. BFS 全空时，随机/引导采样启动新子树
4. 达到 max_boxes 或 max_consecutive_miss 时终止
```

**关键参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ffb_min_edge` | 0.05 | FFB 最小边长（越小→box 更多更小→覆盖率更高） |
| `guided_sample_ratio` | 0.6 | 引导采样比例（HierAABBTree 未占用区域采样） |
| `n_edge_samples` | 3 | 每面种子数（0=所有面） |
| `boundary_expand_epsilon` | 0.01 | 面外扩距离 |
| `max_consecutive_miss` | 80 | 连续失败次数上限 |

#### RRT（快速随机树）

多树 round-robin RRT 生长。每棵树从子树内最近 box 向随机/目标偏置方向延伸。

```
算法流程:
1. 在 q_start / q_goal 处各放置锚 box
2. Round-robin 轮询每棵树，每棵树最多尝试 max_attempts_per_tree 次：
   a. 以 rrt_goal_bias 概率采样目标点，否则均匀随机
   b. 在本子树内线性扫描最近 box
   c. 计算方向向量 direction = (target - nearest_center) / norm
   d. rrt_snap_to_face(nearest, direction):
      - 枚举所有面，选 dot(direction[d], face_normal) 最大的面
      - 种子 = 70% 方向投影 + 30% 随机（在面上）
   e. 调用 find_free_box() 尝试放置
3. 全树失败时 boundary fallback（随机选本树一个 box 的随机面）
4. 全轮无进展时，随机采样启动新子树
```

**关键参数**：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ffb_min_edge` | 0.05 | FFB 最小边长 |
| `rrt_goal_bias` | 0.10 | 目标偏置概率 |
| `rrt_step_ratio` | 0.15 | 步长 = step_ratio × max(q_range) |
| `rrt_max_attempts_per_tree` | 20 | 每棵树每轮最大尝试次数 |
| `max_consecutive_miss` | 80 | 连续失败次数上限 |

### 评估指标

| 指标 | 计算方式 | 优化方向 |
|------|----------|----------|
| **MC 覆盖率** | N 个均匀采样中，自由且被 box 包含的比例 | 最大化 |
| **Box 数量** | `forest.n_boxes` | 最小化 |
| **连通分量数** | `UnionFind(adjacency).n_components()` | 最小化（=1 为最佳） |
| **最大分量占比** | `largest_component / n_boxes` | 最大化（=1.0 为最佳） |
| **生长耗时** | wall-clock ms | 最小化 |
| **综合得分** | 0.4×cov + 0.3×(1−boxes/max) + 0.2×lg\_frac + 0.1×(1−t/max) | 最大化 |

### 2DOF 基准结论

> 实验：54 配置 × 3 场景 = 162 次运行，详见 `v4/output/bench_grower_<ts>/`

- **Wavefront BFS 全面优于 RRT**：覆盖率相当，速度快 50–500×
- `ffb_min_edge` 是最敏感参数：0.03 → 覆盖率 93%（300 boxes），0.10 → 77%（65 boxes）
- Wavefront 对 `guided_sample_ratio` / `n_edge_samples` 不敏感
- RRT 对 `goal_bias` / `step_ratio` 不敏感（std < 0.02）

---

## forest/node_store

> 头文件：`include/sbf/forest/node_store.h`
> 实现：`src/forest/node_store.cpp`

### `NodeStore` — SoA 平坦节点存储

KD-tree 节点的 SoA（Structure of Arrays）存储，支持树结构、per-link iAABBs、占用管理和持久化。

**SoA 字段**：

| 字段 | 类型 | 说明 |
|------|------|------|
| `left_` / `right_` / `parent_` | `vector<int>` | 树结构（-1=无） |
| `depth_` | `vector<int>` | 节点深度 |
| `split_` | `vector<double>` | 分裂值 |
| `link_iaabbs_` | `vector<float>` | 平坦数组 \[cap × n\_active × 6\] |
| `has_aabb_` | `vector<uint8_t>` | iAABB 有效标志 |
| `forest_id_` | `vector<int>` | 森林 box ID（-1=未占用） |
| `subtree_occ_` | `vector<int32_t>` | 子树占用计数器 |
| `source_quality_` | `vector<uint8_t>` | 端点源质量等级 |

**关键方法**：

```cpp
NodeStore snapshot() const;     // 深拷贝，清除占用
void copy_node_from(const NodeStore& src, int src_idx, int dst_idx);
void save(const std::string& path) const;  // HCACHE03
void load(const std::string& path);
```

---

## viz/viz_exporter — 3D 可视化 JSON 导出器

> 头文件：`include/sbf/viz/viz_exporter.h`
> 实现：`src/viz/viz_exporter.cpp`
> 命名空间：`sbf::viz`

将 C++ 端的 Robot、EndpointStore、VoxelGrid、Scene 数据导出为 JSON 文件，
供 Python `sbf4_viz` 包读取并生成交互式 Plotly 3D HTML 可视化。

### 导出函数

| 函数 | 说明 |
|------|------|
| `export_robot_json(robot, q, path)` | 导出机器人 FK 关节链（含 link_positions） |
| `export_robot_json(robot, positions, path)` | 导出预计算的关节位置 |
| `export_envelope_json(robot, store, path)` | 导出 EndpointStore 中所有节点的 link iAABBs |
| `export_envelope_from_boxes_json(robot, boxes, n_sub, path)` | 导出任意 iAABB 数组（per-link lo/hi） |
| `export_voxel_json(grid, path)` | 导出体素砖块（bricks + delta） |
| `export_voxel_centres_json(grid, path)` | 导出体素中心坐标列表 |
| `export_scene_json(scene, path)` | 导出场景障碍物（Obstacle lo/hi） |
| `export_snapshot_json(robot, q, store, node_idx, rvox, ovox, scene, path)` | 导出组合快照（robot + envelope + voxels + scene） |
| `export_envelope_comparison_json(robot, q, store, node_idx, n_sub, delta, scene, path)` | 导出 5 种方法对比 JSON |

### 对比方法（5 种）

| method key | 类型 | 说明 |
|------------|------|------|
| `full_link_iaabb` | aabb | 每条连杆的完整 iAABB（sub=1） |
| `link_iaabb` | aabb | 每条连杆的细分 iAABB（sub=n_sub） |
| `voxel_hull16` | voxel | Hull-16 栅格化体素（最紧密） |
| `voxel_link_iaabb` | voxel | Link iAABB 栅格化体素 |
| `voxel_full_link_iaabb` | voxel | Full Link iAABB 栅格化体素（最粗） |

### JSON key 命名

| JSON key | 说明 |
|----------|------|
| `link_iaabbs` | 每条活动连杆的完整 iAABB（1 per link） |
| `link_iaabbs_sub` | 细分 iAABB（n_sub per link） |
| `link_positions` | FK 关节位置序列 |
| `centres` | 体素中心坐标数组 |
| `bricks` | 体素砖块数组（含 key、mask64） |
| `obstacles` | 场景障碍物数组 |

### Python 渲染包 `sbf4_viz`

```bash
# 一键生成全部 HTML 可视化
python -m sbf4_viz <output_dir>
```

| 模块 | 功能 |
|------|------|
| `load_data` | JSON 加载器 + 数据类（IAABB / EnvelopeNode / VoxelData / SceneData） |
| `envelope_viz` | iAABB 透明盒 + 线框渲染 |
| `robot_viz` | 3D 机器人关节链渲染 |
| `scene_viz` | 障碍物可视化 |
| `voxel_viz` | 稀疏体素散点/立方体渲染 |
| `combined_viz` | 多层叠加统一视图（robot + envelope + voxel + scene） |
| `envelope_comparison_viz` | 多方法对比视图（切换按钮 + 体素 JS 滑块 + 集合差分） |
| `run_demo` | CLI 驱动器 |

---

## robot/iaabb_config

> 头文件：`include/sbf/robot/iaabb_config.h`
> 实现：`src/robot/iaabb_config.cpp`

### 设计概述

通用 DH 参数预处理器，将任意 N-DOF 串联 DH 链转换为专用于 iAABB 生成的紧凑配置结构。
该配置在 `Robot` 构造时自动生成并缓存（`Robot::iaabb_config()`），也可独立持久化为 JSON 文件，
供后续所有 iAABB 相关代码使用。

**预处理步骤**：

1. **连杆长度计算**：对每个连杆 $\ell$，从 DH 参数 $(a_\ell, d_\ell)$ 计算固有长度 $\sqrt{a_\ell^2 + d_\ell^2}$
2. **活跃连杆识别**：跳过 link 0（基座）；长度 $< \tau$（默认 $\tau = 0.1$m）的连杆视为零长度，跳过
3. **配对端点布局**：`n_active_endpoints = n_active × 2`（每条活跃连杆的近端 + 远端）
4. **逐链接关节裁剪**：`n_affecting_joints[ci]` = 影响活跃连杆 `ci` 位置的关节数
5. **iFK 截止帧**：`last_active_frame` = 最高活跃帧索引，iFK 无需超出此帧

### `IAABBConfig` 结构体

| 字段 | 类型 | 说明 |
|------|------|------|
| `zero_length_threshold` | `double` | 零长度判定阈值（默认 0.1m） |
| `n_joints` | `int` | 关节数（拷贝自 Robot） |
| `has_tool` | `bool` | 是否有工具坐标系 |
| `n_total_links` | `int` | 总连杆数 = `n_joints + has_tool` |
| `n_active` | `int` | 活跃连杆数 |
| `n_active_endpoints` | `int` | 活跃端点数 = `n_active × 2` |
| `last_active_frame` | `int` | iFK 截止帧索引 |
| `active_link_map` | `vector<int>` | 活跃连杆索引数组 |
| `active_link_radii` | `vector<double>` | 各活跃连杆的碰撞半径 |
| `active_link_lengths` | `vector<double>` | 各活跃连杆的长度 |
| `n_affecting_joints` | `vector<int>` | 各活跃连杆的影响关节数 |
| `all_link_lengths` | `vector<double>` | 全部连杆长度（含跳过的） |
| `link_is_active` | `vector<bool>` | 各连杆是否活跃 |

### 工厂方法

```cpp
// 从 Robot 构建 IAABBConfig（通用，适用于任意 N-DOF DH 链）
static IAABBConfig IAABBConfig::from_robot(
    const Robot& robot,
    double threshold = 0.1);  // 长度 < threshold 的连杆视为零长度
```

### JSON 序列化 / 反序列化

```cpp
// 保存到 JSON 文件（格式: iaabb_config v1）
void save_json(const std::string& path) const;

// 从 JSON 文件加载
static IAABBConfig load_json(const std::string& path);
```

JSON 格式包含 `format: "iaabb_config"`, `version: 1`，以及完整的配置数据。

### 摘要输出

```cpp
std::string summary() const;  // 人类可读的调试字符串
```

示例输出：
```
IAABBConfig: n_joints=7  has_tool=N  n_total_links=7  threshold=0.1
  All links (length -> status):
    link 0: len=0.333  [SKIP: base]
    link 1: len=0  [SKIP: < 0.1]
    link 2: len=0.316  [ACTIVE]
    link 3: len=0.0825  [SKIP: < 0.1]
    link 4: len=0.393  [ACTIVE]
    link 5: len=0  [SKIP: < 0.1]
    link 6: len=0.088  [SKIP: < 0.1]
  Active links: [2, 4]  (n_active=2)
  n_active_endpoints=4  last_active_frame=5
```

### `Robot` 集成

`Robot` 类在构造时（`pack_arrays()` 内部）自动调用 `IAABBConfig::from_robot(*this)` 构建配置，
并通过 `iaabb_config()` 访问器暴露：

```cpp
const IAABBConfig& Robot::iaabb_config() const;  // 自动构建，无需手动调用
```

> **向后兼容**：`Robot` 的原有 `active_link_map()`（阈值 1e-10）保持不变，
> `IAABBConfig` 使用独立的 0.1 阈值。两套活跃连杆定义共存，互不影响。

### 典型机器人预处理结果（threshold=0.1）

| 机器人 | 总连杆 | 活跃连杆 | active\_link\_map | 说明 |
|--------|--------|---------|-------------------|------|
| Panda 7-DOF（无工具） | 7 | 2 | [2, 4] | link 3(0.0825), 6(0.088) 被过滤 |
| IIWA14 7-DOF（含工具） | 8 | 3 | [2, 4, 7] | link 6(0.081) 被过滤 |