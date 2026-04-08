# Link iAABB 模块实现细节

## 1. 模块定位

Link iAABB 模块是包络流水线的 Stage 2。
它把 endpoint 区间包络转换为每条 active link 的 AABB，并支持更细粒度表示（subdivide、grid）。

核心文件：
- include/sbf/envelope/envelope_derive.h
- src/envelope/envelope_derive.cpp

## 2. 数据模型

### 2.1 通用 frame 布局

derive_aabb 使用通用 frames + active_link_map 的传统路径。

### 2.2 paired active 布局（主路径）

derive_aabb_paired 直接消费 Stage 1 的 paired endpoint 结果：
- 输入：n_active * 2 * 6
- 输出：n_active * 6

每条 link 的处理：
- 轴向合并 proximal 与 distal 区间
- 再做半径膨胀

这条路径避免额外索引跳转，和 LECT 缓存布局天然匹配。

## 3. 主要函数

### 3.1 derive_aabb_paired

每条 active link 的计算步骤：
1. 读 proximal endpoint AABB
2. 读 distal endpoint AABB
3. lo 取逐轴最小
4. hi 取逐轴最大
5. 按 link radius 膨胀

### 3.2 derive_aabb_subdivided

把一条 link 沿参数 t 线性分段，生成 n_sub 个子段 AABB。
用于更细包络近似（尤其是 grid/hull 流程）。

### 3.3 adaptive_subdivision_count

依据几何对角线与体素尺寸估算子段数量，并受 max_sub 限制。

### 3.4 derive_grid

把子段 AABB 光栅化到体素网格。
支持固定子段数或自适应子段数。

## 4. 与碰撞流程的关系

- LinkIAABB：直接用 per-link AABB 做宽相筛选
- LinkIAABB_Grid / Hull16_Grid：宽相后再做 grid/hull 精化

## 5. 几何与数值要点

- 半径膨胀在区间合并后进行
- 子段由 t 从 0 到 1 做线性插值
- 体素映射使用 floor/ceil，再夹到 world bounds

## 6. 性能要点

- derive_aabb_paired 连续内存 + 低分支，速度稳定
- LECT 预缓存 link_iaabb_cache，热路径避免重复派生
- grid 派生成本更高，通常按需惰性触发

## 7. 扩展建议

- 为 derive_aabb_paired 增加 SIMD 路径
- 子段策略可引入曲率/姿态敏感启发式
- grid 表示可考虑压缩存储或块级稀疏优化
