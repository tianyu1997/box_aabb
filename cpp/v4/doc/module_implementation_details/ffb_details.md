# FFB（find_free_box）实现细节

## 1. 模块定位

FFB 是围绕 seed 在 KD 树中查找可用自由盒的核心过程。
实现位于 LECT::find_free_box，输出 FFBResult。

核心文件：
- include/sbf/core/types.h
- src/forest/lect.cpp

## 2. 输出契约（FFBResult）

字段：
- node_idx：选中的自由节点
- path：从根到终点的遍历路径
- fail_code：失败原因码
- n_new_nodes：过程中新增节点数
- n_fk_calls：下降中增量 FK 调用次数

成功条件：
- fail_code == 0 且 node_idx >= 0

## 3. 主算法流程

输入：
- seed
- obstacles
- min_edge
- max_depth

循环步骤：
1. 将当前节点加入 path
2. 占用检查
3. 碰撞检查栈：
   - link iAABB 宽相
   - 需要时 hull 精化
4. 若当前节点可接受且子树占用为 0，直接成功返回
5. 若是叶子且不可接受：
   - 检查深度与边长停止条件
   - 执行 split 并计算子节点包络
6. 按 seed 相对 split 的位置选择子节点下降
7. 计算子节点增量 FK
8. 继续循环

## 4. 失败码语义（按当前实现）

- fail_code 1：被 occupied 节点阻塞
- fail_code 2：达到 max_depth
- fail_code 3：分裂边长 <= min_edge

这些失败码会被 grower 的重试和绕开策略使用。

## 5. 与包络流水线的耦合方式

FFB 自身不做重几何求解，而依赖 LECT 的节点缓存：
- 已有节点：直接读缓存
- 新 split 子节点：按需触发 compute_envelope

因此单次查询成本主要由路径长度与局部扩展决定。

## 6. 与占用状态的关系

- occupied 节点不可直接分配
- subtree occupancy 控制是否可提前接受
- 成功返回后会触发祖先 refine

## 7. 性能特征

- 缓存命中且远离障碍时非常快
- 近障碍区域会因精化判定变重
- 对 split 策略与 seed 质量敏感

## 8. 建议观测指标

推荐持续记录：
- n_fk_calls
- n_new_nodes
- path 深度
- fail_code 分布

用于区分瓶颈来源：
- 几何冲突
- 深度/边长停止
- 占用冲突

## 9. 可扩展方向

- 随深度或局部密度自适应 min_edge
- 细化 fail_code 分类
- 增加边界 seed 的下降 tie-break 规则
