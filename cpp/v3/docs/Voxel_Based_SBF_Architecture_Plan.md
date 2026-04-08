# 基于稀疏体素位域（Sparse Voxel BitMask）的 SafeBoxForest 升级架构与实施计划

## 1. 核心动机与理论破局

在当前的 SafeBoxForest (SBF) 架构中，连杆在工作空间（W-space）中的包围盒使用的是笛卡尔 AABB。由于包围盒不仅要处理连杆的斜置，还要处理大区间下的非线性旋转（即 Wrapping Effect），导致空间浪费极其严重，直接压低了 C-space Free Region 的空间覆盖率。

**理论破局点**：
基于 GCS 的运动规划，其**“凸性限制”（Convexity）严格只作用于关节空间 (C-space)**；对于物理工作空间 (W-space)，算法只要求包装盒具有**“绝对安全性”（保守覆盖）**，而**完全不要求它是凸面体**。
因此，我们可以彻底抛弃多边形包围盒，转而使用**底层 3D 离散网格（Grid/Voxel）**来记录连杆的扫掠体积。不仅能做到物理极限上的紧致贴合，更能通过位运算（Bitwise）实现光速的碰撞检测和树节点无损合并。

---

## 2. 核心架构设计

新系统将抛弃原有的 `[xmin, xmax, ymin, ymax, zmin, zmax]` AABB，转而采用一套**“稀疏位砖的软件光栅化引擎”**。

### 2.1 存储结构：稀疏位砖 (Sparse Bit-Bricks)
*   **网格划分**：将机器人的工作物理空间划分为统一分辨率（如 $1\text{cm}^3$）的小体素。
*   **微块压缩 (Bit-Brick)**：将每 $8 \times 8 \times 8$的体素（共 512 个小格）打包为一个处理单元。512 bits 恰好可以用连续的 **$8$ 个 64位无符号整数 (`uint64_t`)** 表示。
*   **哈希映射 (Spatial Hashing)**：一个长连杆只会点亮少数几个“砖块”。在 LECT 缓存中，每个节点只存储一个极其简单的哈希表（或稀疏数组）：`Map<Brick_ID, Array<uint64_t, 8>>`。
*   **内存表现**：绝对压缩，每个包装盒仅占几百字节（远低于传统高精度感知八叉树），完美容纳于 CPU 高速缓存 (L1/L2 Cache) 中。

### 2.2 极速光栅化与保守安全认证 (Fast Rasterization & Certified Safety)
*   **生成方法**：对于 LECT 的最底层叶子节点，计算其两端 AABB（或确切边界），建立包络表面方程或者胶囊距离场 $F(x,y,z)$。
*   **降维填充 (Scanline)**：使用类似早期 3D 图形学的 X轴扫描线算法，直接计算射线穿透几何体的进入点 $X_{in}$ 和穿出点 $X_{out}$，调用块指令一次性将两个端点之间的 bits 全部置 `1`，避免 3D 的 `for` 循环遍历。
*   **绝对安全保证 (Certified)**：在刷入网格时，人为将几何体向外表面做 $R_{padding} = \sqrt{3}\Delta/2$ 的离散化膨胀（$\Delta$ 为网格边长），**从数学上提供包含所有子空间连续微小运动的绝对物理安全认证**。

### 2.3 无膨胀非凸合并 (Zero-Loss Merging)
*   以往 AABB 向上合并父节点时，会拉出一个更巨大、更空洞的大盒子。
*   在 Voxel 系统中，子节点合并为父节点的操作，**在数学上仅仅是两个不规则点集的精确并集**：直接将子节点的对应 Brick 进行按位或运算 `Parent_Brick = Left_Brick | Right_Brick`。
*   借用 AVX 等 SIMD 指令，一个时钟周期即可完成一个 $8 \times 8 \times 8$ 空腔无损且异形的“热狗状”体积融合。

### 2.4 纯内存级碰撞检测 (Bitwise Collision Kernel)
*   将静态环境障碍物在系统初始化时，进行同等分辨率的离散哈希砖块化。
*   **碰撞判断**：提取机器人扫掠位的哈希键（取有数据的区块），去场景哈希表中查询，直接按位与判断 `(Robot_Brick[i] & Scene_Brick[i]) != 0`。
*   所有的分支预测、三角函数、浮点数分离平面计算被全部剥离，纯寄存器裸奔。

---

## 3. 详细实施路线图 (4 步走)

### Phase 1: 理论验证与离线原型 (Python / C++ Demo)
**目标**：验证在特定分辨率下（如 2cm），Voxel 化是否能提供比笛卡尔 AABB 更小体积的覆盖，及验证连杆端点距离场光栅化算法正确性。
*   **Task 1.1**：基于 C++ 编写核心的 `BitBrick` 结构和 `SpatialHash` 容器。
*   **Task 1.2**：实现最基础的 Rasterization 算法，输入机械臂某一区间的 Joint 端点，结合膨胀率和胶囊半径，输出高亮体素集合。
*   **Task 1.3**：做一个简单的 3D 可视化，对比相同 C-space 区间下，原 AABB 的体积与离散体素簇逼近出的体积差异（复现我们推导出的“消除 70% 死角”的预期）。

### Phase 2: 算力压榨与高性能库重构 (Performance Engineering)
**目标**：必须将包围盒生成和检测压到几微秒级别，否则无法支持 LECT 的大规模图扩张。
*   **Task 2.1**：扫描线降维填充算法优化。丢除所有浮点数开方，改用预计算的查找表或者快速乘加完成边界极值判定。
*   **Task 2.2**：利用 Intel AVX2 / AVX-512 指令集，封装 `__m256i` 系列的位与 (`_mm256_and_si256`) 和 位或 指令，用于极速区块合并及查碰。
*   **Task 2.3**：Benchmark 压测，对等比较单纯 `if(A.max < B.min)` 和 SIMD `Bitwise_AND` 会消耗多少纳秒。

### Phase 3: SBF 架构重组与 LECT 改造
**目标**：替换核心 C++ 算法引擎中的边界载体，将数据类型进行“抽筋换骨”。
*   **Task 3.1** ✅：改写 LECT 的节点数据结构。由 `std::vector<AABB>` 变为缓存特定指针的稀疏 `BrickMap`。
    - **已完成**：LECT 已包含 `hull_grids_` (per-node VoxelGrid) 和 `scene_grid_` (预光栅化障碍物 VoxelGrid)
    - `set_scene()` 将所有障碍物一次性光栅化为共享 VoxelGrid，避免每次 FFB 查询重建
    - `hull_collides()` 自动使用预光栅化场景网格（**实测加速 2272×**）
    - `hull_collides_grid()` 公开 API 支持外部场景管理
    - `collides_scene()` 两阶段碰撞检测（AABB 早退 → Hull-16 精化）
*   **Task 3.2** ✅：改写 Greedy Coarsening 规则。原先是由于子节点 AABB 比例不佳拒绝合并，现在变为"查询合并后网集体积是否超过冗余限度"。
    - **已完成**：`merged_children_hull_volume()` 计算子节点 bitwise-OR 合并后体积
    - `coarsen_volume_ratio()` 计算 merged\_vol / max(left, right) 比率（≥1.0，越接近 1 越好）
    - `merge_children_hulls()` 执行零损失合并（parent = left | right）
    - 实测 root 节点左右子树 38% 重叠被精确消除，合并后体积 34.95 m³ < 原始 38.37 m³
*   **Task 3.3** ✅：重写 Python 接口绑定 (`pybind11`)，以便继续使用现有 Python 脚本派发配置。
    - **已完成**：`python/bindings.cpp` 创建 `pysbf3` 模块（603 KB .pyd）
    - CMakeLists.txt 添加 `SBF_WITH_PYTHON` 选项，FetchContent 自动拉取 pybind11 v2.12
    - 绑定类：Interval, Obstacle, BoxNode, JointLimits, FFBResult, PlanningResult, Edge
    - 绑定类：Robot（from_json）, Scene, VoxelGrid（fill_aabb, fill_hull16, merge, collides）
    - 绑定类：LECT 全公开 API（find_free_box, set_scene, collides_scene, coarsening, persistence）
    - 绑定枚举：StoreFormat, CollisionPolicy；配置结构：EnvelopeConfig, ForestConfig, BridgeConfig, PlannerConfig, SBFConfig
    - 全部集成测试通过（Interval/Obstacle/VoxelGrid/Robot/LECT/set_scene）

### Phase 4: 端到端 GCS 规划测试 (End-to-End Evaluation)
**目标**：用最终系统在 Marcucci Benchmark 环境下与 IRIS-NP 以及基于 AABB 的 SBF 跑满全流程规划，收集性能指标。
*   **指标评估**：
    *   C-space 覆盖率提升（预期从 24% 大幅冲刺向 IRIS-NP 的 60% 甚至更高）。
    *   LECT 按需建树时间开销（可能变长，需检验总体 Region Generation 的一秒神话是否能稳住）。
    *   规划解的路径质量及 GCS-SOCP Solver 的时间变化。

---

## 4. 潜在风险与管控策略

1.  **Resolution 颗粒度陷阱 (Curse of Dimensionality)**
    *   *风险*：分辨率过低，导致向外膨胀过大，比 AABB 还要糟糕；分辨率过高，产生数千个 Brick 的遍历拖垮时间。
    *   *管控*：加入**层次化 Grid (Hierarchical Voxel)**。对于中心实心完全确定的巨大安全区域，用更大的位块表示，只有边缘用小位块逼近（经典八叉树混合策略）。
2.  **动态障碍物的更新损耗**
    *   *风险*：论文提到了一大卖点是基于 LECT 的极速动态重构（7ms）。体素化后，如果环境变化，需要重新刷新场景哈希表。
    *   *管控*：场景体素化远快于动态建树，直接将动态物体的 AABB 做快速 $O(1)$ 包围盒 `fill_bits` 添加进入场景表即可。
3.  **Rasterization 底层耗时超预期**
    *   *风险*：哪怕怎么优化算法，计算射线穿行并执行哈希表分配，都远比查 6 个点长。
    *   *管控*：如果该环节压榨不下来，可以采取**妥协平替策略**，即回滚到我们推演过的另一个神级分支：在笛卡尔系下，只在检测阶段使用 **从两端点端点动态衍生的 k-DOP** 作为早筛分离轴，放弃离散化，留存纯数学解析的极速流派。因此这套计划在架构上拥有极强的退路保障。

---

## 5. Pipeline Benchmark v2 实测结果（2026-03-12）

### 5.1 实验设定
- 机器人：IIWA14 (7-DOF, 4 active links)
- 50 随机 C-space 盒（40% narrow 5% / 40% medium 20% / 20% wide 60%）
- 超参扫描：n_sub ∈ {1,2,4,8,16}, Δ ∈ {0.005,0.01,0.02,0.04}m, grid_R ∈ {16,32,48,64}
- 评分：纯体积（最紧凑者胜）
- 代码：`experiments/exp_pipeline_benchmark.cpp`

### 5.2 结果摘要

| Source | Envelope | n_sub | Vol (m³) | Cold (ms) | Warm (ms) | Disk/node |
|---|---|---|---|---|---|---|
| IFK | SubAABB | 1 | 1.427 | 0.005 | <0.001 | 288 B |
| IFK | SubAABB_Grid | 2 | 1.514 | 0.009 | 0.004 | 33 KB |
| IFK | Hull16_Grid | 1 | 1.187 | 0.085 | 0.055 | 5 KB |
| CritSample | SubAABB | 1 | 0.257 | 0.106 | <0.001 | 288 B |
| CritSample | SubAABB_Grid | 8 | 0.247 | 0.130 | 0.011 | 109 KB |
| CritSample | Hull16_Grid | 1 | 0.173 | 0.139 | 0.025 | 2 KB |
| AnalyticalCrit | SubAABB | 1 | 0.257 | 226.5 | <0.001 | 288 B |
| AnalyticalCrit | SubAABB_Grid | 16 | 0.232 | 226.6 | 0.071 | 258 KB |
| AnalyticalCrit | Hull16_Grid | 1 | 0.166 | 227.0 | 0.479 | 34 KB |

### 5.3 关键发现
1. **CritSample 性价比最高**：5.6× 紧于 IFK，仅 21× 的时间开销
2. **Hull16_Grid 提供最紧凑包络**：比 SubAABB 紧 32.7%（CritSample 源）
3. **AnalyticalCrit 收益递减**：比 CritSample 仅紧 4%，但慢 2160×
4. **推荐配置**：CritSample + Hull16_Grid（cold 0.139ms, warm 0.025ms, 0.173m³, 1.5KB/node）

### 5.4 已发现问题
1. **Hull16_Grid safety_pad=0.0**：实验中 VoxelGrid 构造未设置 √3Δ/2 安全填充，导致体积偏小（非保守）。需修复后重新测量。
2. **CritSample+Hull16_Grid 选择了粗网格 δ=0.04**：令人意外，待排查 safety_pad 修复后是否改变。
3. **SubAABB_Grid 存储爆炸**：n_sub=8 时 109KB/node，不适合大规模 LECT。需要稀疏化或分层策略。