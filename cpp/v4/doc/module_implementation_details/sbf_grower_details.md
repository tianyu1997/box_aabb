# SBF Grower 实现细节（以 ForestGrower 为主）

## 1. 当前实现状态说明

v4 中实际可用的建森林主流程在 ForestGrower。
SafeBoxForest 类存在但仍以 stub 为主。

核心文件：
- include/sbf/forest/forest_grower.h
- src/forest/forest_grower.cpp
- src/forest/grower_roots.cpp
- include/sbf/forest/sbf.h
- src/forest/sbf.cpp

## 2. 模块职责

ForestGrower 负责：
- root 选择
- 子树分区
- 迭代扩展（wavefront / rrt）
- 通过 FFB 创建盒子
- promotion 与 coarsen
- 可选并行子树构建与合并

## 3. grow 端到端流程

grow(obs, n_obs)：
1. 清理状态（boxes、graph、occupation、计数器）
2. 按 envelope 类型设置 scene grid
3. 可选加载 LECT cache
4. root 选择与 partition
5. 扩展（wavefront 或 rrt）
6. promote_all
7. 单线程路径执行 subtree bridge
8. graph 重建并汇总统计
9. 可选 coarsen_greedy
10. 若启用 start/goal，做恢复与连通补偿

## 4. root 选择与分区

### 4.1 分区策略

- partition_uniform：按最宽维重复二分
- partition_lect_aligned：按 LECT 实际 split 维和值对齐
- kd-split 路径：先根后分区

### 4.2 端点引导 root

若有 start/goal：
- 先尝试在 start/goal 建盒
- 再用偏置与多样性策略补齐额外 roots

## 5. 扩展模式

### 5.1 Wavefront

- 以体积优先队列驱动
- 在边界面采样扩张
- 支持 coarse/fine 分阶段 min_edge

### 5.2 RRT 风格

- 随机或目标偏置采样
- 图上找最近盒
- 沿方向做面 snap
- 调用 FFB 创建候选盒

## 6. 盒子创建主路径

try_create_box：
1. 调用 lect.find_free_box
2. FFB 失败或节点已占用则拒绝
3. 从节点恢复 joint intervals
4. 构造 BoxNode
5. 标记 LECT 节点占用
6. 维护 boxes 与 id 索引映射

这是绝大多数新盒的统一入口。

## 7. Promotion 与 Coarsen

### 7.1 Promotion（树结构内）

promote_all 循环扫描内部节点。
try_promote_node 主要条件：
- 左右子都是叶子
- 两子都被占用
- 父未占用
- 父包络碰撞检查通过

通过后把两子提升为父盒。

### 7.2 Coarsen（图层级）

coarsen_greedy：
- 基于图邻接构造合并候选
- 分数由 hull_volume / sum_volume 决定
- 用 intervals_collide_scene 验证合并盒可行性
- 用新盒替换旧对

## 8. 并行模式

grow_parallel：
- 每个子树由单线程 worker 构建
- 可用 LECT snapshot 做 warm-start
- 合并时做 box id remap
- 条件满足时把 worker 子树回灌到 coordinator LECT

## 9. bridge 与 coarsen 拆分状态

当前 grower_bridge.cpp 与 grower_coarsen.cpp 仍是占位文件。
对应核心逻辑仍在 forest_grower.cpp 中。
这说明该重构拆分处于未完全收口状态。

## 10. 关键运行时数据结构

- boxes_：盒子主存储
- box_id_to_idx_：向量可变场景下的 O(1) 映射
- graph_：邻接与连通域信息
- lect_：几何判定与占用真实源
- subtrees_：区域采样约束

## 11. 行为边界与注意事项

- endpoint 恢复阶段可能临时 unmark occupied 节点再重试，之后恢复
- 并行合并后一致性依赖 remap 与 transplant 正确性
- coarsen 阈值过激会改变连通结构与解空间覆盖

## 12. 建议增强测试

- 固定随机种子的可复现实验
- promotion/coarsen 前后连通性不变量检查
- lect 占用状态与 boxes 映射一致性检查
- 并行与串行在同配置下的结果差异分析
