# IROS 论文提纲（8页版）

**建议标题（英文）：**
> *SafeBoxForest: Certified Axis-Aligned Free-Space Decomposition via C-space Box-Conditioned Link-AABB Envelopes for Graph of Convex Sets Motion Planning*

**建议标题（备选）：**
> *Persistent Conservative C-space Decomposition for GCS Planning via C-space Box-Conditioned Link-AABB Envelopes*

---

## Abstract（~200词）

- **背景**：Graph of Convex Sets（GCS）规划框架将运动规划转化为在凸集图上的连续优化，但其性能高度依赖自由空间凸集（region）的质量与生成效率。IRIS 系列方法通过椭球/多面体近似生成区域，但不提供碰撞安全认证，且每次环境变化需全量重建。
- **方法**：提出 SafeBoxForest（SBF），一种面向 GCS 的持久化 C-space 超矩形分解框架。核心对象是**关节空间 box 对应的工作空间各机械臂连杆 link AABB 向量**。三层架构：（1）AABB 层可用区间正运动学或采样优化（Critical/Random）计算并缓存每个 C-space box 的 link AABB；（2）Forest 层以 HierAABBTree + BFS 波前扩展构建持久化无重叠 Box 图；（3）HCACHE 持久化与增量更新机制使 Forest 可跨查询、跨场景复用。
- **实验**：Franka Panda 7-DOF，与 IRIS、C-IRIS、IRIS-NP 比较区域生成时间、自由空间覆盖体积与安全性认证；含 HCACHE 热启动与增量更新实验。
- **结论**：SBF 在保守安全性认证、生成效率与多查询摊还代价上均优于现有 IRIS 系列方法，是 GCS 规划的更优 region 生成后端。

---

## I. Introduction（约 0.7 页）

**动机**
- GCS [Marcucci 2023] 提供了可证明最优路径的优化框架，但"凸集从哪里来"是核心瓶颈：region 生成慢、不可复用、无碰撞安全认证。
- 工业场景（装配流水线、协作机器人）需要高频次、重复性规划，region 每次重建代价不可接受。
- IRIS 系列采用椭球/多面体近似，依赖线性化 FK，不能保证 region ⊆ C-free。

**现有方法局限**
- **IRIS [Deits 2015]**：工作空间椭球，不直接适配高维 C-space
- **IRIS-NP [Petersen 2023]**：C-space 多面体，非线性规划初始化，不提供碰撞认证
- **C-IRIS [Werner 2023]**：SOS 认证，计算代价极高（每 region 需分钟级），难以扩展至大规模 Forest
- **共同缺陷**：障碍物变化后必须全量重建；region 不可跨场景复用；无持久化机制

**本文贡献**
1. **区间 FK 保守认证**：证明基于区间算术的 C-space 超矩形满足 $B_i \subseteq \mathcal{C}_\text{free}$，无需线性化，认证代价 $O(D \cdot L)$
2. **多策略 link AABB 框架**：以“C-space box → workspace link AABB”作为统一计算与缓存单元；支持区间 FK（认证）/关键点采样（紧致）/随机采样三种模式，并统一支持连杆分段策略，量化安全-紧致度权衡
3. **HierAABBTree + HCACHE**：层级区间树持久化，增量 FK 前缀复用节省约 43% 计算量，跨场景零代价加载
4. **SafeBoxForest（SBF）**：BFS 波前扩展 + Bridge Paving 构建持久化 Box 图，作为 GCS 输入；增量更新仅重建失效区域
5. 在 Panda 7-DOF 上全面对比 IRIS 系列，验证认证安全性、生成效率与自由空间覆盖能力

---

## II. Related Work（约 0.6 页）

**GCS 规划框架**
- GCS [Marcucci 2023]：凸集图上连续优化，可获得全局最优轨迹；依赖高质量凸集输入
- 本文专注 GCS 的 region 生成问题，下游规划器均为 GCS（公平对比）

**IRIS 系列 Region 生成方法**
- IRIS [Deits & Tedrake, IROS 2015]：工作空间交替椭球/半平面迭代，不提供 C-space 认证
- IRIS-NP [Petersen et al. 2023]：C-space 多面体，非线性规划初始化，精度更高但非认证
- C-IRIS [Werner et al. 2023]：基于 SOS 多项式的认证 C-space region，认证代价极高
- IRIS-ZO [Dai et al. 2023]：零阶优化 IRIS，减少 FK 调用，但仍不认证

**区间算术在机器人运动学中**
- 区间 FK 用于工作空间可达域分析 [Merlet 2004, Jaulin 2001]
- 本文首次将区间 FK 用于 GCS region 生成，提供轻量级认证保证

**持久化凸集结构**
- PRM [Kavraki 1996]：持久图，无认证，碰撞检测概率性
- SBF 提供认证 region + 持久化 + 增量更新的完整组合

---

## III. Conservative C-space Box Computation via Workspace Link AABB（约 1.8 页）

**问题形式化**

给定关节区间 $I = \prod_{i=1}^D [l_i, u_i]$ 和第 $\ell$ 条连杆，求其 AABB：
$$AABB_\ell(I) = \left[\min_{q \in I} p_\ell(q),\ \max_{q \in I} p_\ell(q)\right]^3$$

对每个 C-space box $B=I$，定义缓存条目为
$$\mathcal{A}(B)=\{AABB_1(I),\ldots,AABB_{L_{active}}(I)\},$$
即“该关节空间 box 对应的工作空间各连杆 link AABB 向量”。

**认证保守性（Conservative Certification）**：$\forall q \in I,\ p_\ell(q) \in AABB_\ell(I)$ 成立，则称 AABB 保守认证。

**安全性含义**：C-space Box $B = I$ 中的任意配置 $q$，其所有连杆 AABB 均不与障碍物 AABB 相交，则可保守认证 $B \subseteq \mathcal{C}_\text{free}$——**无需碰撞采样**。

### III-A. 策略一：区间正运动学（Interval FK，认证）

**区间三角函数封闭**：
$$[\underline{\sin}(\alpha, \beta),\ \overline{\sin}(\alpha, \beta)],\quad [\underline{\cos}(\alpha, \beta),\ \overline{\cos}(\alpha, \beta)]$$

**区间 DH 矩阵链积**：对每关节区间构造上下界齐次变换矩阵 $T_{lo}^i$, $T_{hi}^i$，链式乘法：
$$[T_{lo}^{1:k},\ T_{hi}^{1:k}] = [T_{lo}^{1:k-1},\ T_{hi}^{1:k-1}] \otimes [T_{lo}^k,\ T_{hi}^k]$$

**命题（认证保守性）**：区间 FK 输出的 link AABB 满足认证保守性，证明基于区间算术的包含单调性。

### III-B. 策略二：关键点枚举采样（Critical，紧致，非认证）

目标函数 $f_d(q) = p_\ell^d(q)$ 在区间顶点或鞍点取极值，三阶段流程：
1. **关键点枚举**：对相关关节子集 $\mathcal{R}_\ell$ 枚举边界组合，直接求值
2. **约束流形随机采样**：在关节耦合约束流形上采样，补充枚举遗漏极值
3. **L-BFGS-B 局部优化**：精确逼近极值

不提供认证保守性；用于紧致度对比基线，或在附加 safe margin 后使用。

### III-C. 策略三：随机采样（Random，基线，非认证）

均匀随机采样 $N$ 个关节配置，加入边界顶点组合，经 L-BFGS-B 精化。

### III-D. 通用策略：连杆分段与区间细分（Subdivision Strategies）

**连杆分段（Link Subdivision）**：为降低单条长连杆带来的 AABB 过保守性，将每条连杆在局部坐标系下等分为 $n$ 段（n_subdivisions > 1），为各段独立计算 AABB。
**区间细分（C-space Interval Subdivision）**：与连杆分段在工作空间的作用类似，在关节空间将较大的 C-space Box 细分为更小的子区间，同样能显著降低非线性映射带来的包络过保守性，使得生成的 AABB 更加紧致。
**通用性与高效性**：这两种细分策略均泛用于上述三种 AABB 生成方法。在采样方法（Critical/Random）中，各分段可共享相同的关节采样样本与优化过程；在区间方法中，可共享前置的变换矩阵计算，从而以极低的额外开销显著提升 Box 的紧致度与自由空间覆盖率。

### III-E. HierAABBTree 与 HCACHE 持久化

- **层级区间树（HierAABBTree）**：本质上是自适应细胞分解（Adaptive Cell Decomposition）在高维 C-space 的变种。通过 KD-Tree 结构对 C-space 超矩形进行递归分裂，上层节点覆盖更大区间，有效缓解高维空间的维度爆炸问题；Find Free Box (FFB) 采用 top-down 搜索快速定位最大无碰撞 Box。
- **增量 FK 前缀复用**：分裂时复用父节点累积变换，仅重算变化维度后续矩阵乘法，节省约 43% 运算。
- **HCACHE02 格式与 Lazy Load, 增量写入**：二进制持久化，仅绑定机器人运动学（与障碍物无关）；缓存内容为每个 C-space box 的 workspace link AABB 向量，支持跨场景复用。采用 Lazy Load（延迟加载）机制，仅在查询到特定节点时才从磁盘读取或计算其 AABB，极大降低了内存占用与初始化开销；增量写入仅存 dirty/new 节点。

### III-F. 策略安全性与紧致度对比

| 策略 | 认证安全 | 紧致度 | 计算代价 | Vs. IRIS |
|---|---|---|---|---|
| Interval FK | ✓ 认证 | 较低（过保守） | 低，$O(D \cdot L)$ | IRIS 不认证，代价相近 |
| Critical + Optim | ✗ 不认证 | 高 | 中 | 与 IRIS-NP 相当 |
| Random + Optim | ✗ 不认证 | 中 | 高 | 低于 IRIS-NP |

*注：连杆分段与区间细分策略可正交地应用于上述三种方法，在保持认证属性（对 Interval 而言）的同时，以少量额外计算代价显著提升紧致度。*

---

## IV. SafeBoxForest Construction and GCS Graph Generation（约 2.0 页）

**符号**：$D$-DOF 机器人，$\mathcal{C} = \prod_{i=1}^D [l_i, u_i]$，障碍物集合 $\mathcal{O}$，自由空间 $\mathcal{C}_\text{free}$，Box（C-space 超矩形）$B_i = \prod_d [l_i^d, u_i^d]$，邻接图 $G=(V,E)$。

### IV-A. 数据结构与三不变量

**定义（SafeBoxForest）**：无重叠认证 Box 集合 $\{B_1,\ldots,B_N\}$ 与邻接图 $G=(V,E)$，满足：
- **I1 非重叠**：任意 $B_i, B_j$ 内部不相交（满足 GCS 凸集不重叠假设，避免路径在重叠区域产生非物理跳跃）
- **I2 邻接对称**：共享面邻接关系对称
- **I3 缓存一致**：HierAABBTree 缓存与 Box 集合同步

**认证安全性**：基于 Interval FK 构建的所有 Box 满足 $B_i \subseteq \mathcal{C}_\text{free}$（认证成立），区别于 IRIS 的概率安全性。

### IV-B. SBF 的生长与扩展 (Forest Construction and Expansion)

SBF 的扩展以"采样生成 seed → FFB 生成 Box → 入队邻接"为基本循环。

**1. Seed 采样策略**：

- **S-G 采样（Start-Goal Sampling）**：在规划初始化阶段，优先以起点 $q_\text{start}$ 和终点 $q_\text{goal}$ 作为 seed，确保 Forest 对起终点的覆盖。
- **空闲区间权重采样（Free Interval Weighted Sampling）**：在各维度上统计当前未被 Box 覆盖的自由区间，并按区间长度加权随机采样，系统性地向覆盖不足的区域扩展，避免重复生长。
- **Box 边缘采样（Box Boundary Sampling / BFS Wavefront）**：对已生成的 Box 进行 BFS 传播，在当前 Box 外表面做 $\epsilon$-偏移采样作为 seed，引导 Forest 沿自由空间连续扩张。此过程中引入 **Goal-biased 采样**：以一定概率直接采样终点方向的偏移位置，引导 Forest 更快地向目标区域延伸，与 RRT 的 goal bias 机制类似。

**2. Find Free Box (FFB) 核心算子**：
输入 seed 配置 $q_\text{seed}$，从 HierAABBTree 根节点 top-down 搜索；无碰撞时 promotion（吸收父节点扩大 Box），碰撞时 split 后递归；**Active Split Dims** 自动排除对末端位置无影响维度（如 Panda q7），抵达叶节点返回最大认证无碰撞 Box $B^* \ni q_\text{seed}$。

**3. Bridge Paving (孤岛桥接)**：
生成sbf后，尝试将不同的sb-tree 通过三步桥接（dry-run FFB → overlap check → 正式 FFB）补充 Box；失败时调用 RRT-Connect 获取路径，获取路径后尝试通过路径上的点采样，拓展box tree，永久扩充连通性。

### IV-C. 基于 SBF 生成 GCS 规划图 (GCS Graph Generation)

**1. 节点映射与 Coarsen (粗化)**：
每个 $B_i$ 直接映射为 GCS 的一个凸集节点。为降低 GCS 求解器的计算负担，在映射前执行 **Coarsen** 操作：通过维度扫描合并相邻且对齐的 Box，显著减少 GCS 图的节点数。

**2. 边构建与邻接关系**：
当 $B_i \cap B_j \neq \emptyset$（即两个 Box 在 C-space 中共享面或有交集）时，在 GCS 图中添加一条边。由于 SBF 保证了 Box 的非重叠性（I1），GCS 边上的连续性约束可以被精确且高效地定义。

**3. 端点接入**：
将起终点配置 $q_\text{start}, q_\text{goal}$ 所在的 Box 作为端节点接入 GCS 图，完成规划问题的形式化。

### IV-D. 增量更新与持久化复用

**与 IRIS 的关键差异**：IRIS 每次障碍物变化需全量重新运行（无法识别哪些 region 失效）；SBF 精确识别并仅重建失效区域。

**增量更新流程**：
```
invalidate_against_obstacle(new_obs)  # 标记与新障碍物碰撞的 Box
remove_invalidated()                   # 移除失效节点，同步更新 GCS 图
regrow(seeds)                          # 以原 seeds 局部 FFB 回填
```
复杂度 $O(N_\text{invalid} \cdot H \cdot D \cdot L_\text{active})$，远小于 IRIS 全量重建 $O(N_\text{total} \cdot T_\text{IRIS})$。

**实现要点**：Python + Cython（`_hier_core.pyx`），SoA `NodeStore`（~20 ns/node），动态 mmap 扩容，`ProcessPoolExecutor` 并行邻接构建。

---

## V. Experiments（约 2.5 页）

**实验平台**：Franka Panda 7-DOF（$D=7$），Ubuntu 22.04，Intel Core i7，无 GPU。三个密度场景：

| 场景 | 障碍数 | 难度 |
|---|---|---|
| panda_8obs_open | 8 | 开阔 |
| panda_15obs_moderate | 15 | 中等 |
| panda_20obs_dense | 20 | 密集 |

**对比方法（聚焦 Region 生成阶段）**：

| 方法 | Region 类型 | 认证 | 持久化 |
|---|---|---|---|
| **SBF-GCS（本文）** | 超矩形（Interval/Critical/Random + 分段） | Interval 模式 ✓ | ✓ |
| IRIS-GCS | 椭球/多面体 | ✗ | ✗ |
| IRIS-NP-GCS | C-space 多面体 | ✗ | ✗ |
| C-IRIS-GCS | C-space SOS 多面体 | ✓ | ✗ |

**评估指标**：Region 生成总时间、Region 数量、自由空间覆盖率（总体积估算）、单 Region 平均体积、增量更新耗时。

---

### Exp 1：Region 生成效率与体积对比（Table I + Fig. 2）

- 固定时间预算或固定 Region 数量 $N$，对比各方法的**生成速度**与**覆盖体积**
- **Table I**：总生成时间、每 region 平均耗时、总体积（通过蒙特卡洛积分估算 C-space 占比）、认证状态
- **预期**：SBF（Interval FK）生成单个认证 Box 的速度比 C-IRIS 快 2–3 个数量级；虽然单个 Box 体积可能小于 IRIS 多面体，但在相同时间预算下，SBF 能生成海量 Box，实现极具竞争力的总自由空间覆盖率。

### Exp 2：细分策略（连杆分段与区间细分）对体积的提升（Fig. 3）

- 针对 SBF（区间与采样模式），测试不同连杆分段数（$n=1, 2, 4$）以及不同 C-space 区间细分粒度对生成结果的影响
- **对比指标**：单 Box 平均体积、总体积覆盖率、生成时间开销
- **预期**：细分策略（无论是在工作空间还是关节空间）能显著缓解过保守性，使 Box 体积呈倍数增长，而得益于计算共享，时间开销仅呈次线性增长。

### Exp 3：动态障碍物增量更新效率（Table II）

- 在已构建的 Region 集合上逐步添加或移动障碍物（1~5 个），测量：
  - SBF 增量更新时间（识别失效 + 局部回填） vs IRIS 全量重建时间
  - 更新前后的总体积保持率
- **预期**：SBF 增量更新耗时仅为全量 IRIS 重建代价的约 $1/N$，且能迅速恢复大部分自由空间覆盖。

### Exp 4：HCACHE 热启动收益（Table III）

- 首次 Region 构建 vs HCACHE 热加载的运动学/AABB 计算阶段耗时
- 跨场景（不同障碍物配置，同机器人）的缓存命中率
- **预期**：热加载将 FK 阶段耗时压缩至 < 1ms，使 Region 生成的计算瓶颈完全转移至纯几何碰撞检测，总生成时间显著缩短。

### Exp 5：消融实验（Table IV）

| 变体 | 说明 | 预期影响 |
|---|---|---|
| SBF（Interval Full） | 完整管线（基准） | 速度极快，体积中等，绝对安全 |
| SBF（Critical） | 换用 Critical 采样（非认证） | 体积更大，速度略降，无认证 |
| SBF（Random） | 换用 Random 采样（非认证） | 体积中等，速度较慢，无认证 |
| No wavefront | 关闭 BFS 波前，纯随机撒种 | 区域重叠度高，总体积覆盖率下降 |
| No coarsen | 不进行相邻 Box 合并 | Region 数量庞大，单体体积小 |

---

## VI. Conclusion（约 0.4 页）

**总结**
- 提出 SafeBoxForest（SBF），首个基于区间 FK 认证的持久化 C-space 超矩形分解框架，专为 GCS 规划设计
- Interval FK 以 $O(D \cdot L)$ 代价提供 C-space region 认证，认证效率比 C-IRIS 高约 2–3 个数量级
- HCACHE 持久化 + 增量更新，使 region 生成从"每次重建"转变为"增量维护"
- 实验验证 SBF 在认证安全性、生成效率、自由空间覆盖率与多查询摊还代价上均优于 IRIS 系列

**局限性**
- 超矩形比 IRIS 椭球/多面体体积更小（过保守），高密度障碍物场景需更多 Box 覆盖
- 当前仅支持串联机械臂 DH 模型；并联、闭链机构需扩展区间 FK

**未来工作**：仿射算术收紧包络、并联机器人支持、自适应分段策略、与 GCS 求解器联合优化

---

## References（约 20–25 篇）

关键引文：
- **GCS**：[Marcucci 2023] Science Robotics
- **IRIS 系列**：[Deits 2015] IROS, [Petersen 2023] IRIS-NP, [Werner 2023] C-IRIS, [Dai 2023] IRIS-ZO
- **区间算术**：[Jaulin 2001] Applied Interval Analysis, [Merlet 2004] Interval FK
- **运动规划基础**：[Lozano-Pérez 1979] C-space, [Kavraki 1996] PRM, [Sucan 2012] OMPL
- **碰撞检测**：[Pan 2012] FCL
- **Drake**：[Tedrake 2019]

---

## 图表规划

**图（共 4 幅）**：
1. **Fig. 1**（约 1/3 页）：整体框架图——左：区间 FK → Box 认证示意（与 IRIS 椭球对比）；右：SBF Forest 在 Panda C-space 2D 投影中的可视化
2. **Fig. 2**（约 1/4 页）：Region 生成时间对比（各方法，横轴：region 数量 $N$，纵轴：生成时间）
3. **Fig. 3**（约 1/4 页）：HCACHE 热启动收益 + 增量更新时间（柱状图）
4. **Fig. 4**（约 1/4 页）：BFS 波前扩展 vs IRIS 随机种子的覆盖质量可视化（GCS 图连通性对比）

**表（共 4 张）**：
- **Table I**：Region 生成效率（方法 × 场景 × 时间/认证状态）
- **Table II**：GCS 规划质量（方法 × 场景 × 成功率/路径长度）
- **Table III**：增量更新 vs IRIS 全量重建
- **Table IV**：消融实验

---

## 页面分配参考（8页 IEEE 双栏）

| 章节 | 预计占用 |
|---|---|
| Abstract + I. Introduction | ~0.8 页 |
| II. Related Work | ~0.6 页 |
| III. Conservative C-space Box Computation | ~1.8 页（含认证对比表） |
| IV. SafeBoxForest: Persistent Convex Decomposition | ~2.0 页 |
| V. Experiments | ~2.5 页（含图表） |
| VI. Conclusion + References | ~0.8 页 |
| **合计** | **~8.5 页（需微调压缩）** |

---
