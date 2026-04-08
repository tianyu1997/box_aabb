# LECT 模块实现细节

## 1. 模块定位

LECT（Link Envelope Collision Tree）是核心 KD 树组件，负责：
- 惰性 C 空间划分
- 节点级包络缓存
- 碰撞宽相/精化
- FFB 查询支撑

核心文件：
- include/sbf/forest/lect.h
- src/forest/lect.cpp

## 2. 架构要点

### 2.1 双通道模型

ChannelIdx：
- SAFE
- UNSAFE

每个通道都维护扁平化节点缓存：
- ep_data
- link_iaabb_cache
- source_quality
- has_data
- hull_grids
- hull_valid

目的：
- 安全语义隔离
- 防止 UNSAFE 结果误用于 SAFE 证书

### 2.2 扁平 stride

在 init_root 一次性设置：
- ep_stride = n_active_endpoints * 6
- liaabb_stride = n_active_links * 6

节点 i 的地址偏移：
- ep_data + i * ep_stride
- link_iaabb_cache + i * liaabb_stride

优点是缓存友好、索引简单、拷贝高效。

## 3. 节点包络构建

compute_envelope(node, fk, intervals, changed_dim, parent_idx) 主流程：

1. 检查缓存 source_quality 是否可服务当前请求
2. 可选 Z4 缓存命中
3. IFK fast path（若 source=IFK 且 fk.valid）：
   - 全量抽取，或
   - 基于 changed_dim + parent_idx 做部分继承
4. 非 IFK 走 Stage 1 统一入口 compute_endpoint_iaabb
5. derive_aabb_paired 派生并写入 link_iaabb_cache
6. 失效对应 hull_valid
7. 可选写入 Z4 canonical 缓存

## 4. 分裂策略

支持：
- ROUND_ROBIN
- WIDEST_FIRST
- BEST_TIGHTEN

BEST_TIGHTEN 的核心：
- 对每个候选维度构造左右子区间
- 用增量 FK + link AABB 估计左右体积
- 选择 min(max(vol_left, vol_right)) 的维度

## 5. 收紧与回传

refine_aabb(parent)：
- parent_ep = intersect(parent_ep, union(left_ep, right_ep))
- 然后重算父节点 link_iaabb_cache

propagate_up(path)：
- 在 FFB 成功后自底向上调用 refine_aabb
- 持续收紧祖先节点包络

## 6. 碰撞检测栈

### 6.1 宽相

iaabbs_collide(node, obstacles)：
- 直接读取 link_iaabb_cache
- 做 AABB overlap

### 6.2 精化

若宽相命中且配置要求精化：
- 计算或读取 hull grid
- 与 scene grid 或临时 obstacle grid 做体素碰撞

## 7. 与 FFB 的关系

find_free_box 中，LECT 负责：
- 节点占用判断
- 宽相/精化碰撞判断
- 叶子按需 split
- 按 seed 导航下降

FFBResult 返回完整路径与诊断信息。

## 8. 持久化

- 树结构：lect.hcache
- hull：lect.hulls（支持双通道格式）
- metadata：用于恢复 pipeline 配置和有效性语义

## 9. 对称性优化（Z4）

当 q0 具旋转对称时启用 Z4 cache：
- 先 canonicalize q0 区间扇区
- 对 canonical 区间做 hash
- 非 canonical 扇区可通过变换复用 endpoint 包络

## 10. 当前边界

- 通道间替代严格受安全规则约束
- hull 为惰性生成，缺失时行为偏保守
- load 路径对 has_data 的恢复依赖持久化上下文

## 11. 建议测试点

- source 替代矩阵正确性
- IFK 部分继承在 split 后的一致性
- BEST_TIGHTEN 与基线策略效果对比
- Z4 hit/miss 与扇区映射正确性
- refine_aabb 单调收紧性质
