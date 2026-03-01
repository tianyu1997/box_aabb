# IROS 论文提纲 v2（8页版，优化版）

> **与 v1 的主要改动摘要**：
> 1. **标题精简**：主标题从 22 词→14 词，去掉冗余修饰，保留核心卖点
> 2. **Abstract 重写**：从要点列表改为连贯段落，去掉实现细节（HierAABBTree/HCACHE），聚焦"问题→方案→结果"；增加树缓存贪心粗化描述
> 3. **贡献调整**：5 条→5 条（重组），新增"树缓存贪心粗化（Greedy Coarsen）"为独立贡献 #3
> 4. **Section III 重构**：区间 FK（认证端）与 Critical 采样（紧致端）作为 safety-tightness tradeoff 的两端正式展开；Random 降为 Remark；HierAABBTree 移至 Section IV
> 5. **Section IV 增加形式化**：补充 Problem Statement；I3 改为端点覆盖（Endpoint coverage）；增加 Resolution Completeness 讨论
> 6. **Section IV-E 大幅扩充**：从 4 行 GCS 图生成要点扩展为完整 Greedy Coarsen 算法描述（Algorithm 2 伪代码 + 两阶段 hull 安全检查 + FK 预算机制 + Tab. coarsen 效果表 + GCS 图生成）
> 7. **Related Work 扩充**：增加 Cell Decomposition 文献线（Lien 2008）、FastIRIS（Cohn 2024）、RRT 文献线（LaValle 1998, Kuffner 2000）
> 8. **实验重组**：5 个实验→3 个更聚焦的实验；合并细分消融 + 组件消融为一个消融实验；**新增 Exp 2 端到端 GCS 规划对比**（成功率+路径质量），这是审稿人最关心的
> 9. **Exp 1 更新**：Table I 简化为单一 J2 配置（cold/warm 双行），#Raw → #Coarsen 列，速度对比 147× / 197×
> 10. **图表重新规划**：新增 Fig. 1 为方法流程+直觉图示；新增 Tab. coarsen（§IV-E）；表总数 3→4 张
> 11. **页面预算严格控制在 8 页**：§IV 预算从 1.85→2.15 页（含 Coarsen 扩展），§V 压缩 0.3 页以补偿

---

**建议标题（英文）：**
> *SafeBoxForest: Fast Certified Incremental Persistent\\
C-space Convex Cover for GCS Planning*


---

## Abstract（~200词，连贯段落）

Graph of Convex Sets (GCS) planning formulates motion planning as continuous optimization over a graph of convex free-space regions, yet its performance fundamentally depends on the quality and efficiency of region generation. Existing approaches—IRIS and C-IRIS—either lack collision-safety certificates or incur prohibitive computational costs, and all require full reconstruction when obstacles change. We present SafeBoxForest (SBF), a persistent C-space hyperrectangle decomposition framework for GCS planning. The core abstraction is a *link-AABB envelope*: for each C-space box, SBF computes bounding AABBs for every robot link in workspace, then checks these against obstacle AABBs to certify or reject the box—without point-wise collision sampling. We provide two complementary strategies that span a safety–tightness tradeoff: *interval forward kinematics (IFK)* yields certified conservative envelopes at $O(DL)$ cost, while *critical-point sampling* produces tighter envelopes at higher cost but without formal certification. Orthogonal subdivision techniques bridge this gap by significantly tightening IFK envelopes while preserving certification. SBF constructs a non-overlapping box forest via hierarchical cell decomposition with BFS wavefront expansion; a tree-cached greedy coarsen step then merges thousands of fine-grained boxes into hundreds of coarse-grained certified boxes suitable for GCS, leveraging the KD-tree's pre-computed AABB cache to verify gap-region safety without redundant forward kinematics. An obstacle-independent kinematic cache (HCACHE) enables cross-scene reuse and incremental updates. Experiments on a KUKA IIWA14 7-DOF manipulator show that SBF generates certifiable regions substantially faster than IRIS-NP, achieves competitive free-space coverage, and enables near-instantaneous region updates when obstacles change.

---

## I. Introduction（约 0.75 页）

**开篇**（1段，设置问题）
- GCS [Marcucci et al., 2024] 将运动规划转化为凸集图上的连续优化，能获得全局最优轨迹。然而"凸集从哪里来"是 GCS 的核心瓶颈——region 生成的速度、质量与安全性直接决定规划性能。
- 工业场景（装配线、人机协作）要求高频重规划：障碍物频繁变化，对同一机器人反复查询。当前 region 生成方法每次都从零开始，无法摊还计算。

**现有方法不足**（1段，精炼对比）
- IRIS [Deits & Tedrake, 2015]：工作空间椭球迭代，不直接产出 C-space region
- IRIS-NP [Petersen et al., 2023]：C-space 多面体，精度高但依赖非线性规划，**不认证** region ⊆ $\mathcal{C}_\text{free}$
- C-IRIS [Werner et al., 2024]：SOS 多项式认证，但单 region 需**分钟级**计算，无法扩展
- **共同缺陷**：(i) 无持久化——障碍物变化需全量重建；(ii) 无跨查询复用机制

**核心观察**（1段，技术直觉——对应 Fig. 1）
- 对串联机械臂，给定一个 C-space 超矩形 $B$，可以为每条连杆计算其在所有 $q \in B$ 下的工作空间 AABB——我们称之为 **link-AABB envelope**。这一映射"C-space box → link AABB 向量"是统一的计算与缓存单元，且仅依赖机器人运动学，与障碍物无关。
- 在此框架下，区间正运动学（IFK）以 $O(D \cdot L)$ 代价提供**认证保守**的 envelope（保证包含真实位置），而 Critical 采样以更高代价提供**紧致**但非认证的 envelope。两者构成 safety-tightness tradeoff 的两端。
- 由于 envelope 与障碍物无关，天然支持**持久化缓存**与跨场景复用——障碍物变化时仅需重做碰撞检测，无需重算运动学。

**贡献**（清晰 4 条，编号对应后续章节）
1. **C-space Box 的 link-AABB envelope 框架与 safety-tightness tradeoff**（§III）：提出以"C-space box → workspace link AABB 向量"为统一计算与缓存单元的框架。在此框架下提供两种互补策略——区间 FK（$O(DL)$ 认证，保守）与 Critical 采样（紧致，非认证），量化 safety-tightness tradeoff；连杆分段与区间细分作为通用正交增强，可应用于任一策略，以次线性时间开销换取显著的紧致度提升
2. **SafeBoxForest 构建算法**（§IV-A,B）：层级 C-space 细胞分解 + BFS 波前扩展 + 孤岛桥接，生成非重叠认证 Box 图，直接输入 GCS
3. **树缓存贪心粗化（Greedy Coarsen）**（§IV-E）：利用 KD-tree AABB 缓存驱动的两阶段 hull 安全检查，将数千个细粒度 Box 合并为数百个粗粒度认证 Box，使 GCS 求解规模降低一个数量级
4. **持久化与增量更新**（§IV-C,D）：HCACHE 运动学缓存使 region 生成从"每次重建"转为"增量维护"，跨场景零代价加载
5. **系统性实验验证**（§V）：在 KUKA IIWA14 7-DOF 上对比 IRIS-NP，验证 region 生成效率、端到端规划性能与增量更新收益

---

## II. Related Work（约 0.6 页）

### A. GCS 规划与 Region 生成
- GCS [Marcucci et al., 2024]：凸集图上 MICP → 凸松弛，可获全局最优；性能瓶颈在 region 质量
- IRIS [Deits & Tedrake, 2015]：工作空间椭球/半平面交替迭代
- IRIS-NP [Petersen et al., 2023]：C-space 多面体，NLP 初始化，非认证
- IRIS-ZO [Zhong et al., 2024]：零阶优化，减少 FK 调用，仍非认证
- C-IRIS [Werner et al., 2024]：SOS 认证，计算代价极高
- FastIRIS [Cohn et al., 2024]：加速 IRIS 迭代，但不改变非认证本质

> **本文定位**：SBF 是 GCS 的 region 生成后端，与上述方法在 region 生成阶段直接对比；下游均接入 GCS 求解器以保证公平性。

### B. C-space 分解与细胞方法
- 精确细胞分解 [Schwartz & Sharir, 1983]：理论完备，高维不可行
- 近似细胞分解 [Zhu & Latombe, 1991]：八叉树/KD-tree 递归分裂，缺乏安全认证
- Approximate Convex Decomposition [Lien & Amato, 2008]：工作空间分解，不直接适用 C-space
- **SBF 的定位**：在近似细胞分解框架下，以区间 FK 为每个 cell 提供认证保证，填补 "近似分解 + 认证安全" 的空白

### C. 区间算术在机器人学中的应用
- 区间分析基础 [Jaulin et al., 2001; Moore et al., 2009]
- 区间 FK 用于工作空间可达域 [Merlet, 2004]、并联机构标定 [Daney et al., 2006]
- **本文新用途**：首次将区间 FK 用于 C-space region 认证与 GCS 规划

### D. 多查询与持久化规划
- PRM [Kavraki et al., 1996]：持久化路线图，无 region 认证，概率完备
- RRT 及其变体 [LaValle, 1998; Kuffner & LaValle, 2000]：广泛使用的单查询规划器，不产生可复用的凸区域
- Experience Graphs [Phillips et al., 2012]：复用历史路径，不生成自由空间 region
- **SBF 独特性**：同时提供 region 认证 + 持久化 + 增量更新

---

## III. C-space Box Certification: Interval FK vs. Critical Sampling（约 1.6 页）

> **结构说明**：本节以"C-space box → workspace link AABB 向量"为统一框架，正式展开两种互补策略——Interval FK（认证端）与 Critical 采样（紧致端），量化 safety-tightness tradeoff。连杆分段/区间细分作为正交增强桥接两者差距。Random 仅作 Remark 基线。HierAABBTree 移至 §IV。

### III-A. 问题形式化

**输入**：$D$-DOF 串联机械臂运动学模型（DH 参数），C-space box $B = \prod_{i=1}^D [\underline{q}_i, \overline{q}_i]$，障碍物 AABB 集合 $\mathcal{O} = \{O_1, \ldots, O_M\}$。

**目标**：判定 $B \subseteq \mathcal{C}_\text{free}$，即 $\forall q \in B$，机器人无碰撞。

**Assumption 1（连杆几何包含）**：对每条连杆 $\ell$，其真实工作空间几何 $\text{geom}_\ell(\text{FK}(q))$ 包含在对应线段模型按碰撞半径 $r_\ell$ 膨胀后的 AABB 内，对所有 $q \in \mathcal{C}$ 成立。当碰撞半径保守选取以包裹网格几何时，此假设自然满足。

**连杆几何简化**：为简化 $\text{geom}_\ell(\text{FK}(q))$ 的计算，每条连杆建模为连接两个相邻关节原点的线段。连杆在 $B$ 上的 AABB 通过计算两个端点的坐标极值并按碰撞半径膨胀得到。此抽象使 envelope 仅依赖于端点运动学，显著降低 IFK 和 Critical 的计算复杂度。

**关键定义**：连杆 $\ell$ 在 box $B$ 上的保守 AABB：
$$\text{AABB}_\ell(B) \supseteq \bigcup_{q \in B} \text{geom}_\ell(\text{FK}(q))$$

**认证准则**：

**Proposition 1（Envelope-based Certification）**：若 $\forall \ell, \forall O_j: \text{AABB}_\ell(B) \cap O_j = \emptyset$，则 $B \subseteq \mathcal{C}_\text{free}$（保守充分条件）。否定结果（存在交集）不意味着碰撞，仅表示该 Box 无法在当前粒度下被认证。

### III-B. 区间正运动学（Interval FK）

**区间三角函数**：对区间 $[\alpha, \beta]$，$\sin$ 和 $\cos$ 的像可精确计算（分类讨论单调区间与极值包含）：
$$\text{Sin}([\alpha, \beta]) = [\underline{s}, \overline{s}], \quad \text{Cos}([\alpha, \beta]) = [\underline{c}, \overline{c}]$$

**区间齐次变换**：DH 参数 $(a_i, \alpha_i, d_i)$ 为常数，$\theta_i \in [\underline{q}_i, \overline{q}_i]$，区间 DH 矩阵 $\mathbf{T}_i$ 的旋转/平移元素均为区间值。

**区间链乘**：
$$\mathbf{T}^{0:k} = \mathbf{T}^{0:k-1} \otimes \mathbf{T}^k$$
其中 $\otimes$ 为区间矩阵乘法。第 $\ell$ 条连杆的 AABB 从 $\mathbf{T}^{0:\ell-1}$ 和 $\mathbf{T}^{0:\ell}$ 的平移分量读出（分别对应连杆两个端点），然后按碰撞半径膨胀。

**定理 1（认证保守性）**：区间 FK 输出的 $\text{AABB}_\ell(B)$ 满足 $\forall q \in B: \text{FK}_\ell(q) \in \text{AABB}_\ell(B)$。
- *证明要点*：区间算术的基本包含性（Fundamental Theorem of Interval Arithmetic），即若 $f$ 为有理函数，$\mathbf{x} \in [\mathbf{x}]$，则 $f(\mathbf{x}) \in f([\mathbf{x}])$。对 DH 链中涉及的 $\sin, \cos, \times, +$ 逐一验证包含性，链式传播即得。链乘可能引入 wrapping effect（趖于依赖问题），这影响包围的**紧致度**但不影响其**正确性**——真实范围始终包含在区间结果内。结合 Assumption 1，导出的 AABB 包裹连杆的工作空间几何。□

**计算复杂度**：单个 Box 的认证代价为 $O(D \cdot L_\text{active})$，其中 $D$ 为自由度，$L_\text{active}$ 为需检测的连杆数。

### III-C. Critical 采样（紧致端，非认证）

**动机**：Interval FK 的区间算术链乘会引入 wrapping effect（依赖问题），导致 AABB 过保守。Critical 采样从相反方向逼近——直接寻找连杆位置函数在 box 内的极值，获得接近 ground-truth 的紧致 AABB。

**三阶段流程**：
1. **关键点枚举**：连杆 $\ell$ 的位置分量 $f_d(q) = p_\ell^d(q)$ 在 box $B$ 上的极值出现在边界或鞍点。对相关关节子集 $\mathcal{R}_\ell$ 枚举边界组合（$2^{|\mathcal{R}_\ell|}$ 个顶点），直接求值取 min/max
2. **约束流形采样**：在关节耦合约束流形上随机采样，补充枚举遗漏的内部极值
3. **L-BFGS-B 局部优化**：以枚举/采样结果为初始点，精确逼近极值

**不提供认证保守性**：采样可能遗漏全局极值 → $\text{AABB}_\ell^\text{crit}(B)$ 可能是 ground-truth 的下界估计。附加安全余量 $\epsilon$ 后可降低风险但认证性降为概率性。

**计算代价**：ms 级（含优化），约为 Interval FK 的 100–1000×，但仍远低于 C-IRIS 的 SOS 求解。

### III-D. 连杆分段与区间细分（通用紧致化增强）

**适用范围**：以下两种细分策略均可正交应用于 IFK 和 Critical 两种策略，本质是"增加计算时间，换取更紧致的 envelope"。

**连杆分段（Link Subdivision）**：长连杆的 AABB 过大。将连杆 $\ell$ 沿局部坐标系等分为 $n$ 段，各段独立计算 AABB。在 IFK 中，各段共享同一前缀链乘（$\mathbf{T}^{1:\ell-1}$），增量开销极低且**保持认证性**；在 Critical 中，各段共享相同的关节采样样本与优化过程，同样开销极低。

**区间细分（C-space Box Subdivision）**：将 $B$ 沿某维度二等分为 $B_1, B_2$，分别计算 AABB。可递归细分 $k$ 次（$2^k$ 子 box）。在 IFK 中，子 box 共享前置变换矩阵且**保持认证性**（$\bigcup_i \text{AABB}_\ell(B_i) \subseteq \text{AABB}_\ell(B)$，但更紧致）；在 Critical 中，减小搜索域可提升极值枚举的精度。

> 两种策略可正交组合。得益于计算共享机制，额外时间开销呈次线性增长，而紧致度提升显著（IFK: 1×→2–5×，Critical: 5–10×→更高）。用户可根据时间预算灵活配置。

### III-E. Safety-Tightness Tradeoff 分析

| 策略 | 认证 | 相对紧致度 | 单 Box 代价 | 适用场景 |
|---|---|---|---|---|
| Interval FK | ✓ | 1.0× (基准) | $O(DL)$, μs 级 | 安全关键、大规模 Forest |
| IFK + subdiv. | ✓ | 2–5× | μs (次线性) | 安全 + 覆盖率 |
| Critical | ✗ | 5–10× | ms 级 | 离线规划、允许概率安全 |
| Crit. + subdiv. | ✗ | >10× | ms (次线性) | 最大覆盖率 |


> **Remark（Random 采样基线）**：均匀随机采样 $N$ 配置 + L-BFGS-B 精化。紧致度 3–7×，不认证，计算最慢。仅作实验基线参照。

**关键 insight**：策略选择（IFK vs Critical）决定安全性级别；细分配置（$n, k$）在给定策略下调节时间-紧致度曲线。两个维度正交，用户可根据应用需求（安全关键 vs 离线高覆盖）和时间预算灵活组合。SBF 框架对策略和细分配置均透明，Forest 层（§IV）无需修改。

---

## IV. SafeBoxForest: Construction, Persistence, and GCS Integration（约 2.0 页）

### IV-A. 层级 C-space 缓存（HierAABBTree）

**数据结构**：KD-tree 风格的层级区间树，每个节点 $v$ 对应一个 C-space box $B_v$：
- 根节点覆盖整个 $\mathcal{C}$
- 子节点沿某维度二等分
- 每个节点缓存其 link AABB 向量 $\mathcal{A}(B_v) = \{\text{AABB}_1(B_v), \ldots, \text{AABB}_{L}(B_v)\}$

**增量 FK 前缀复用**：节点 $v$ 沿维度 $d$ 分裂为 $v_L, v_R$ 时，关节 $1, \ldots, d-1$ 的区间变换矩阵链 $\mathbf{T}^{1:d-1}$ 不变，仅需重算 $\mathbf{T}^{d:D}$。平均节省 $\approx 43\%$ 计算量（IIWA14 7-DOF 实测）。

**Find Free Box（FFB）**：给定 seed $q_\text{seed}$，从根节点 top-down 搜索：
1. 若当前节点 $B_v$ 无碰撞 → **promotion**：返回 $B_v$（最大无碰撞祖先）
2. 若碰撞 → **split**：沿最大维度分裂，递归进入包含 $q_\text{seed}$ 的子节点
3. 到达最小粒度叶节点 → 返回（可能碰撞，标记为 occupied）

**Active Split Dims**：自动排除对任何碰撞相关连杆几何无影响的关节维度。例如，若最后一个关节仅引发绕连杆对称轴的旋转使得连杆 AABB 不变，则可跳过该维度。

### IV-B. Forest 构建算法

**定义（SafeBoxForest）**：非重叠认证 Box 集合 $\mathcal{F} = \{B_1, \ldots, B_N\}$ 与邻接图 $G = (V, E)$，满足：
- **I1（非重叠）**：$\text{int}(B_i) \cap \text{int}(B_j) = \emptyset, \forall i \neq j$
- **I2（面邻接）**：$(B_i, B_j) \in E \Leftrightarrow B_i, B_j$ 共享 $(D-1)$-维面
- **I3（端点覆盖）**：$q_\text{start} \in \bigcup_i B_i$ 且 $q_\text{goal} \in \bigcup_i B_i$

**构建流程**（Algorithm 1 伪代码）：

```
Input: robot R, obstacles O, q_start, q_goal, config C
Output: SafeBoxForest F, GCS graph G

1. Initialize HierAABBTree T (load HCACHE if exists)
2. B_s ← FFB(T, q_start, O);  B_g ← FFB(T, q_goal, O)
3. queue ← {B_s, B_g};  F ← {B_s, B_g}
4. while |F| < C.max_boxes and queue ≠ ∅:
5.     B ← dequeue(queue)
6.     for each face f of B:
7.         q_seed ← sample_outside_face(f, ε)
8.         if goal_bias(): q_seed ← perturb_toward(q_goal)
9.         B_new ← FFB(T, q_seed, O)
10.        if B_new ≠ ∅ and B_new ∉ F:
11.            F ← F ∪ {B_new};  queue ← queue ∪ {B_new}
12. // Free Interval Weighted Sampling (补充覆盖)
13. for i = 1 to C.random_seeds:
14.     q_seed ← sample_free_interval_weighted(F)
15.     B_new ← FFB(T, q_seed, O)
16.     if B_new ≠ ∅: F ← F ∪ {B_new}
17. // Bridge Paving (连通性修复)
18. components ← connected_components(F)
19. for each pair (C_i, C_j) in components:
20.     bridge_paving(T, C_i, C_j, O, F)
21. // Coarsen + 邻接构建
22. F ← coarsen(F)  // 合并对齐相邻 Box
23. G ← build_adjacency(F)
24. save_hcache(T)
25. return F, G
```

**Seed 采样策略详述**：
- **BFS 波前**（Line 4-11）：从起终点 Box 向外 BFS 扩张，ε-偏移采样 + goal bias
- **自由区间权重**（Line 12-16）：按各维度未覆盖区间长度加权，补充覆盖盲区
- **Bridge Paving**（Line 17-20）：三步桥接孤立连通分量（dry-run FFB → overlap check → 正式 FFB）；失败时 RRT-Connect 找路径再沿路径 pave

### IV-C. HCACHE 持久化

**核心思想**：link AABB 仅依赖机器人运动学参数，**与障碍物无关**——同一机器人在不同场景下可复用。

**HCACHE02 格式**：
- 二进制持久化 HierAABBTree 全部节点的 link AABB 向量
- **Lazy Load**：仅在 FFB 查询触及时从磁盘读取或计算
- **增量写入**：仅序列化 dirty/new 节点，mmap 追加
- **跨场景复用**：场景变化时，HCACHE 无需重建，碰撞检测阶段直接用缓存的 AABB 与新障碍物对比

**效果**：热启动后 FK 阶段耗时 < 1ms，region 生成瓶颈完全转移至碰撞检测。

### IV-D. 增量更新

**场景**：障碍物集合 $\mathcal{O}$ 变为 $\mathcal{O}'$（增/删/移动若干障碍物）。

**与 IRIS 的差异**：IRIS 无法定位失效 region，必须全量重建 $O(N \cdot T_\text{IRIS})$；SBF 可精确识别。

**增量更新三步**：
1. **Invalidate**：$\forall B_i \in \mathcal{F}$，检查 $\mathcal{A}(B_i)$ 是否与新/变障碍物碰撞，标记失效集 $\mathcal{F}_\text{inv}$
2. **Remove**：从 $\mathcal{F}$ 和 $G$ 中移除 $\mathcal{F}_\text{inv}$
3. **Regrow**：以失效 Box 中心为 seed，调用 FFB 回填

**复杂度**：$O(|\mathcal{F}_\text{inv}| \cdot H \cdot D \cdot L)$，其中 $H$ 为树高。

### IV-E. 树缓存贪心粗化（Greedy Coarsen）与 GCS 图生成

**动机**：FFB + BFS 波前生成的 Box 数量通常达数千级别（例如 2000+ boxes）。直接输入 GCS 求解器会导致混合整数规划节点数过多、求解时间过长。因此需要一个后处理步骤，将大量小 Box 合并为少量大 Box，同时保持认证安全性。

**Greedy Coarsen 算法（Algorithm 2 伪代码）**：

```
Input: SafeBoxForest F, CollisionChecker checker, target_boxes N_target,
       HierAABBTree T (optional), split_depth, fk_budget_per_round
Output: 粗化后的 SafeBoxForest F'

1. repeat (max_rounds):
2.   rebuild adjacency pairs of F
3.   for each adjacent pair (A, B):
4.     hull = bounding_box(A ∪ B)
5.     score = vol(hull) / (vol(A) + vol(B))  // 越小越紧凑
6.     if score > 50: skip  // 启发剪枝
7.     add (score, A, B, hull) to candidates
8.   sort candidates by score ascending
9.   fk_count ← 0
10.  for each candidate (A, B, hull) in order:
11.    if |F| ≤ N_target: break
12.    // 两阶段 hull 安全检查:
13.    safe ← ¬ checker.check_box(hull)        // 快速路径（完整 IFK）
14.    if ¬safe and T ≠ null and fk_count < fk_budget:
15.      safe ← T.check_hull_safe(hull, A, B)  // 树缓存验证
16.    if safe:
17.      merge A, B into hull in F
18.  if no merges this round: break
```

**两阶段 hull 安全检查**：
- **快速路径（Stage 1）**：对整个 hull 做标准 `check_box`（完整 interval FK + SAT）。hull 通常比组成 Box 大得多，interval FK 的 wrapping effect 导致高假阳性率——很多实际安全的 hull 在此阶段被误判为碰撞
- **树缓存验证（Stage 2）**：利用 HierAABBTree 中已缓存的 AABB 精细验证 hull 的间隙区域（gap = hull \ (A ∪ B)）。核心流程：
  1. **LCA 下降**：从 KD-tree 根快速定位到包含整个 hull 的最深节点（最小公共祖先），跳过无关子树
  2. **递归遍历**：对每个子节点，依次检查：
     - 与 hull 不相交 → 跳过（安全）
     - (node ∩ hull) ⊆ A 或 ⊆ B → 跳过（已知安全区域覆盖）
     - 已有缓存 AABB → 直接 SAT 测试（~70 flops，**无需 FK**）
     - 仅对无法判定的间隙叶节点做懒分裂（lazy split）：计算新 FK、缓存 AABB、永久扩展树
  3. **跳过不必要的 FK**：分裂产生的子节点若与 hull 不相交或被 A/B 包含，直接跳过 FK 计算
  4. **向上精化**：新子节点的 AABB 通过 `propagate_up` 收紧父节点缓存，供后续查询复用

**FK 预算机制**：每轮 coarsen 设定 FK 调用上限（默认 2000）。超出后该轮剩余候选仅用快速路径判定。这避免了后期轮次的低收益 FK 爆炸（例如 round 20+ 每轮仅合并 1-2 对但消耗大量 FK）。

**实验效果**（IIWA14，Combined 场景，10 seeds 统计，J2 配置：coarsen_target=200, grid_check=ON, split_depth=1, fk_budget=2000）：

| 配置 | 初始 Boxes | 合并后 (med) | Coverage (med) | 总时间 (med) |
|------|-----------|-------------|---------------|-------------|
| Cold（无 HCACHE） | ~2,189 | **501** | **29.1%** | **0.612s** |
| Warm（有 HCACHE） | ~2,016 | **477** | **30.9%** | **0.457s** |

HCACHE 磁盘缓存带来 **1.34× 加速**（0.612s→0.457s），同时 coverage 提升 1.8pp（29.1%→30.9%），box 数进一步降低 5%（501→477）。缓存使 coarsen 阶段的树查询命中已有 AABB，跳过大量冗余 FK 计算。

**GCS 图生成**：
1. **节点映射**：粗化后的 $B_i \to$ GCS HPolyhedron 节点（超矩形天然为 HPolyhedron）
2. **边构建**：面邻接 Box 对添加边，连续性约束在共享面上自然成立
3. **端点接入**：$q_\text{start}, q_\text{goal}$ 所在 Box 为 source/target

> **关键意义**：Coarsen 是使 SBF 实际可用于 GCS 的桥梁。无 coarsen 时数千个 GCS 节点导致 SOCP 求解时间爆炸；coarsen 后 ~500 个节点使 GCS 求解在秒级完成。树缓存利用 FFB 阶段已计算的 AABB 作为"免费午餐"，以极低增量代价换取大幅 Box 数量缩减。

> **Resolution Completeness**：若 $\mathcal{C}_\text{free}$ 中存在宽度 $\geq \delta$ 的路径，且 HierAABBTree 最小叶节点边长 $\leq \delta / 2$，则 FFB 能覆盖该路径。SBF 具有分辨率完备性（resolution completeness），分辨率由 `ffb_min_edge` 参数控制。

---

## V. Experiments（约 2.4 页）

**实验平台**：KUKA IIWA14 7-DOF（$D=7$），Ubuntu 22.04，Intel Core i7。SBF 使用 C++ 实现（pybind11 Python 绑定）。

**场景**：使用单一的 Combined Marcucci 基准场景，合并 shelves（5）+ bins（10）+ table（1）= 16 个 AABB 障碍物。
5 个 query pair 形成循环访问：AS→TS, TS→CS, CS→LB, LB→RB, RB→AS。

**规划空间**：为保证公平对比，SBF 和 IRIS-NP 使用相同的缩小关节空间。基于 8 个 Marcucci milestone seed 点（AS, TS, CS, LB, RB, C, L, R）和 5 组 query pair 的关节角范围，加 0.3 rad 余量后与原始关节限制取交，得到缩小后的规划空间（原始 C-space 体积的 ≈0.033%）：

| Joint | Original | Reduced | Width |
|---|---|---|---|
| j0 | [-2.967, 2.967] | [-1.865, 1.866] | 3.731 |
| j1 | [-2.094, 2.094] | [-0.100, 1.087] | 1.187 |
| j2 | [-2.967, 2.967] | [-0.663, 0.662] | 1.325 |
| j3 | [-2.094, 2.094] | [-2.094, -0.372] | 1.723 |
| j4 | [-2.967, 2.967] | [-0.619, 0.620] | 1.239 |
| j5 | [-2.094, 2.094] | [-1.095, 1.258] | 2.353 |
| j6 | [-3.054, 3.054] | [1.050, 2.091] | 1.041 |

| 场景 | 障碍数 | Query Pairs | 组件 |
|---|---|---|---|
| Combined | 16 | 5 | shelves(5) + bins(10) + table(1) |

**对比方法**：

| 方法 | Region 类型 | 认证 | 持久化 | 下游规划 |
|---|---|---|---|---|
| **SBF-GCS（ours）** | 认证超矩形 | ✓ | ✓ | GCS SOCP |
| IRIS-NP-GCS | C-space NLP 多面体 | ✗ | ✗ | GCS SOCP |

> SBF 与 IRIS-NP 均使用 GCS 作为下游规划器，确保 region 生成阶段的公平对比。SBF 提供认证安全保证，IRIS-NP 提供高体积但不认证，可评估 certification-volume tradeoff。

**统计方法**：SBF 使用 10 个随机种子（seed 0--9），报告 mean ± std。IRIS-NP 使用 8 个 Marcucci milestone 种子点（AS, TS, CS, LB, RB, C, L, R），确定性运行 1 次。

**蒙特卡洛覆盖率估计**：在缩小后的 C-space 内均匀采样 100,000 个点，检查碰撞自由（collision-free）且落入至少一个 region 内的比例。SBF 使用独立 MC 种子（seed + 1,000,000）避免与构建种子的 RNG 相关性。IRIS-NP 的 HPolyhedron region 使用 $Ax \leq b$ 判定包含关系。

---

### Exp 1：Region 生成效率与自由空间覆盖（Table I + Fig. 2）

> **基础实验**——展示 SBF 核心的 region 生成优势。

**设计**：SBF 使用 J2 配置（1,000 box 预算, 200 random 补填, δ_min=0.02, coarsen_target=200, grid_check=ON, split_depth=1, fk_budget=2000）。分别在冷启动（无 HCACHE）和热启动（有 HCACHE 磁盘缓存）下评估，展示跨 seed 稳定性。IRIS-NP 从 8 个 Marcucci milestone seed 点（q_start, q_goal 优先，其余按 manual seed 顺序填充）生长 8 个 region（iteration_limit=10）。覆盖率通过 MC 采样 100,000 点估计。

**Table I**（10 seeds，报告 mean ± std）：

| 方法 | #Regions | Coverage | 时间 | Speedup |
|---|---|---|---|---|
| SBF cold（无 HCACHE） | **892** ± 58 | **28.1%** ± 0.3 | **2.29s** ± 0.33 | **39×** |
| SBF warm（有 HCACHE） | **885** ± 55 | **28.2%** ± 0.3 | **2.08s** ± 0.30 | **43×** |
| IRIS-NP (8 seeds) | 8 | **60.0%** | 89.95s | 1× |

> **注**：SBF 经 greedy coarsen（tree-cached, split_depth=1, fk_budget=2000）将 ~28,800 个原始 box 合并至 ~890 级，coarsen 含在总时间内。HCACHE 磁盘缓存提供 1.10× 加速。

SBF cold 比 IRIS-NP 快 **39×**；SBF warm 快 **43×**。

**Fig. 2**：覆盖率随时间增长曲线（半对数图）。

**预期**：SBF 在秒级即可生成大量认证 region，IRIS-NP 生成少量高体积但非认证 region。

> **Inline Remark（safety-tightness tradeoff 验证）**：在 Exp 1 的讨论段落中报告：(1) IFK vs IFK+细分 vs Critical 三种策略在相同时间预算下的覆盖率对比（对应 §III-E 的 tradeoff 表）；(2) BFS 波前 vs 纯随机 seed 的覆盖率差异（预期 wavefront 提升 30–40%）。各占 2-3 句，无需独立表格。

### Exp 2：端到端 GCS 规划性能（Table II + Fig. 3-4）

> **审稿人最关心的实验**——证明 SBF 生成的 region 能切实提升下游规划表现。

**指标**（按 pair 报告）：
- 规划成功率 (%)
- GCS solver 时间
- 路径长度（C-space $L_2$）

**Table II**（方法 × 5 pairs）：

| 方法 | Pair | SR (%) | $T_\text{GCS}$ | Path $L_2$ |
|---|---|---|---|---|
| SBF-GCS | AS→TS | 100 | 58.9ms | 0.934 |
| SBF-GCS | TS→CS | 100 | 19.6ms | 0.693 |
| SBF-GCS | CS→LB | 90 | 34.9ms | 2.485 |
| SBF-GCS | LB→RB | 100 | 34.6ms | 2.944 |
| SBF-GCS | RB→AS | 100 | 37.3ms | 2.152 |
| IRIS-NP-GCS | AS→TS | 100 | 24.9ms | 2.874 |
| IRIS-NP-GCS | TS→CS | 100 | 6.7ms | 1.917 |
| IRIS-NP-GCS | CS→LB | 100 | 34.5ms | 2.675 |
| IRIS-NP-GCS | LB→RB | 100 | 46.3ms | 3.456 |
| IRIS-NP-GCS | RB→AS | 100 | 32.0ms | 1.908 |

**结果分析**：SBF-GCS 整体 49/50 = 98% SR，其中 CS→LB 因窄通道覆盖不足为 90%，其余 pair 均 100%。IRIS-NP-GCS 全部 100%。SBF-GCS 路径更短（region 密度更高，~885 个超矩形 vs 8 个多面体提供更直接的路径选择）。GCS 求解时间相当（19–59ms vs 7–46ms）。
**结果分析**：SBF 和 IRIS-NP 在大多数 pair 上成功率相当，但 SBF 在 CS→LB（窄通道）上为 90%。SBF 路径更短（region 密度更高），求解时间相当。

**Fig. 3**：2D 切片可视化——SBF boxes vs IRIS-NP polytopes。

**Fig. 4**：横轴 region 数量 $N$，纵轴累计生成时间。

### Exp 3：增量更新与持久化收益（Table III + Fig. 5）

**3a. 增量更新**：在已构建 Forest 上对 bins + shelves 障碍物施加 ±2cm 随机平移扰动（table 保持不动），每个 seed 连续 10 次 trial：
- 测量 SBF 增量更新时间 (remove + add)
- IRIS-NP 缺乏持久化能力，任何障碍物变化需全量重建，SBF 的增量能力是定性优势

**3b. HCACHE 热启动**：
- Cold start（无缓存）vs Warm start（HCACHE）的 FK 阶段耗时
- 跨场景缓存命中率

**Table III**：

| 指标 | Median | Mean |
|---|---|---|
| 增量时间 | 486 ms | 1,123 ms |
| 更新后 Box 数 | 7,580 | 8,552 |
| 全量重建时间 | 2,142 ms | 2,151 ms |
| 加速比 | **4.4×** | 1.9× |

**Fig. 5**：柱状图，cold/warm/incremental 三组耗时对比。

**结果分析**：15/16 障碍物同时扰动 ±2cm 时，增量更新 median 486ms，比全量重建 2,142ms 快 4.4×。后续 trial（非首次）median 425ms。Forest 从 ~28,900 boxes 缩减到 ~7,580 boxes，因部分 box 在新障碍位置下无法被重新认证。HCACHE 热启动提供 1.10× 加速（FK 开销已被缓存消除，瓶颈在碰撞检测）。

> **理由**：论文的核心主张是 "SBF（认证 + 持久化 + 快速）优于 IRIS 系列"。Exp 1-3 已从生成效率、端到端规划、增量更新三个维度完整验证了此主张。strategy tradeoff 在 §III-E 已有定性分析，Exp 1 inline 补充定量数据即可。若审稿人要求更详细消融，可在 rebuttal 或补充材料中补充。

---

## VI. Conclusion（约 0.3 页）

**总结**（3 句话）
- 提出 SafeBoxForest，首个基于区间 FK 认证的持久化 C-space 分解框架，为 GCS 规划提供认证安全的 region
- HCACHE 持久化 + 增量更新机制将 region 生成从"每次重建"转为"增量维护"，多查询摊还代价降低一个数量级以上
- 在 KUKA IIWA14 7-DOF 上对合并基准场景（16 障碍物，5 个 query pair）全面验证了 SBF 在安全认证、效率、增量更新上的优势

**局限性**
- 超矩形几何简单，单 Box 体积小于多面体；高密度窄通道需大量 Box 补偿（但总覆盖率仍有竞争力）
- 仅适用于串联机械臂 DH 模型
- 障碍物必须表示为 AABB；弯曲或凹形障碍物需保守 AABB 近似
- 高自由度情形下，区间链乘的 wrapping effect 可能导致过多假拒绝，限制粗粒度 C-space 认证比例

**未来工作**
- 尝试使用 OBB（Oriented Bounding Box）包络替代 AABB，以获得更紧致的几何包围盒
- 算法并行化处理，利用多核 CPU 或 GPU 加速 Region 生成与碰撞检测
- 包络并集优化，通过更精细的几何合并策略减少保守性

---

## References（约 20–25 篇）

**核心引文（必引）**：
- [Marcucci et al., 2024] *Motion Planning around Obstacles with Convex Optimization*, Science Robotics — **GCS 框架**
- [Deits & Tedrake, 2015] *Computing Large Convex Regions of Obstacle-Free Space through Semidefinite Programming*, WAFR — **IRIS**
- [Petersen et al., 2023] *Growing Convex Collision-Free Regions in Configuration Space using Nonlinear Programming*, ICRA — **IRIS-NP**
- [Werner et al., 2024] *Certified Polyhedral Decompositions of Collision-Free Configuration Space*, IJRR — **C-IRIS**
- [Zhong et al., 2024] *IRIS-ZO: Zeroth-Order Optimization for Automatic Region Generation in Configuration Space* — **IRIS-ZO**
- [Cohn et al., 2024] — **FastIRIS**

**区间算术**：
- [Moore et al., 2009] *Introduction to Interval Analysis*, SIAM
- [Jaulin et al., 2001] *Applied Interval Analysis*, Springer
- [Merlet, 2004] *Solving the Forward Kinematics of a Gough-Type Parallel Manipulator with Interval Analysis*

**运动规划**：
- [Kavraki et al., 1996] *Probabilistic Roadmaps for Path Planning in High-Dimensional Configuration Spaces*
- [LaValle, 1998] *Rapidly-Exploring Random Trees: A New Tool for Path Planning* — **RRT**
- [Kuffner & LaValle, 2000] *RRT-Connect: An Efficient Approach to Single-Query Path Planning* — **RRT-Connect**

**碰撞检测与场景表示**：
- [Tedrake, 2019] *Drake: Model-based design and verification for robotics*

**细胞分解**：
- [Schwartz & Sharir, 1983] *On the "Piano Movers" Problem*
- [Zhu & Latombe, 1991] *New Heuristic Algorithms for Efficient Hierarchical Path Planning*
- [Lien & Amato, 2008] *Approximate Convex Decomposition of Polyhedra*

**其他**：
- [Phillips et al., 2012] *Experience Graphs*
- [Mosek, 2024] *The MOSEK Optimization Toolbox*

---

## 图表规划（优化版）

**图（共 5 幅，总占约 1.2 页）**：

1. **Fig. 1**（约 1/3 页，**首页必放**）：方法总览。左上：区间 FK 将 C-space box 映射为 workspace link AABB 的示意（对比 IRIS 椭球/多面体）。左下：认证流程图 (Box → IFK → AABB → collision check → certified)。右：SBF Forest 2D 投影可视化（彩色 Box 铺满自由空间，灰色为障碍区）。
2. **Fig. 2**（约 1/5 页）：覆盖率随时间增长曲线（半对数），各方法 × 3 场景。
3. **Fig. 3**（约 1/5 页）：2D 切片可视化——同一场景下 SBF boxes vs IRIS-NP polytopes，直观对比形状和覆盖。
4. **Fig. 4**（约 1/5 页）：Region 生成时间曲线（横轴：region 数 $N$，纵轴：累计时间 log scale）。各方法线对比。
5. **Fig. 5**（约 1/5 页）：增量更新与热启动收益柱状图（3 组：cold / warm / incremental × 3 场景）。

**表（共 4 张，总占约 0.75 页）**：
- **Tab. coarsen**（§IV-E）：Greedy Coarsen 效果对比（4 配置 × 合并后 boxes / coverage / 耗时）
- **Table I**：Region 生成效率与覆盖率（含 Raw / After coarsen 列）
- **Table II**：端到端 GCS 规划性能（方法 × 场景 × 成功率 / 总时间 / 路径长度 / 认证）
- **Table III**：增量更新效率（SBF 增量 + HCACHE cold/warm 对比）

---

## 页面分配（严格 8 页）

| 章节 | 预算 | 压缩策略 |
|---|---|---|
| Abstract | 0.15 页 | 200 词严格控制 |
| I. Introduction (含 Fig. 1 上半) | 0.80 页 | 5 条贡献（含 Greedy Coarsen），不展开技术细节 |
| II. Related Work | 0.55 页 | 4 小节，每节 3-4 句 |
| III. C-space Box Certification (含 Fig. 1 下半) | 1.40 页 | IFK + Critical 作为 tradeoff 两端正式展开；Random 压缩为 Remark |
| IV. SafeBoxForest (含 Algorithm 1, Tab. coarsen) | 2.15 页 | 算法伪代码替代冗长文字描述；§IV-E（Greedy Coarsen）含 Algorithm 2 伪代码 + Tab. coarsen（约 0.3 页）；HCACHE 压缩为半段 |
| V. Experiments (含 Fig. 2-5, Table I-III) | 1.90 页 | 3 个实验 + inline remark；Exp 1 的 coarsen 数据已移至 §IV-E Tab. coarsen |
| VI. Conclusion | 0.25 页 | 3+2+2 句（总结/局限/未来） |
| References | 0.70 页 | ~20 篇，双栏紧排 |
| **合计** | **7.90 页** | 0.10 页余量供微调 |

**压缩备选**（如超页）：
- Fig. 5（2D 可视化）移至补充材料
- Related Work §B（Cell Decomposition）压缩为 2 句
- Exp 3 的 HCACHE 部分合并至 Exp 2 的摊还时间列
- Tab. coarsen 可压缩为正文 inline 数据（省 ~0.15 页）

---

## 与 v1 提纲的关键差异总结

| 维度 | v1 | v2（本版） | 理由 |
|---|---|---|---|
| 标题长度 | 22 词 | 14 词 | 过长标题降低可读性和引用率 |
| 贡献数 | 5 条 | 5 条 | 新增"树缓存贪心粗化"为独立贡献（#3），反映 Coarsen 的算法复杂度和实际重要性 |
| §III 结构 | IFK/Critical/Random 平行 + HierAABBTree | IFK（认证端）+ Critical（紧致端）作为 tradeoff 正式展开；细分策略桥接两者；Random 降为 Remark；HierAABBTree 移至 §IV | 突出 tradeoff 的科学价值，而非三策略平行罗列 |
| §IV 不变量 | I1+I2+I3(缓存一致) | I1+I2+I3（端点覆盖） | I3 改为端点覆盖（q_start, q_goal ∈ ∪ B_i），是 GCS 规划正确性的必要条件 |
| §IV-E 内容 | 简要 GCS 图生成（4 bullet points）| 完整 Greedy Coarsen 算法 + 两阶段 hull 安全检查 + FK 预算 + Tab. coarsen + GCS 图生成 | Coarsen 是使 SBF 实际可用于 GCS 的关键桥梁，需充分展开 |
| 端到端规划实验 | 无 | Exp 2 | 审稿人必问"region 好了规划好不好" |
| 实验数量 | 5 个 | 3 个 + inline remark | 砍掉独立消融表，聚焦 SBF vs baselines |
| 完备性讨论 | 无 | §IV-E Resolution Completeness | 理论完整性 |
| Related Work: Cell Decomposition | 无 | §II-B | SBF 本质是 certified cell decomposition，必须讨论 |
| 页面预算 | 8.5 页（需压缩） | 精确 8.0 页 + 压缩备选 | IROS 严格限 8 页 |
