# RRT Grower Volume Bonus 优化计划

## 背景

当前 `ForestGrower::grow_rrt` 在最近 box 搜索中只用 wrap-aware 平方距离 + per-box tabu 惩罚。
当域内同时存在小 box（贴近障碍）和大 box（占据空旷区）时，几何最近搜索经常选中靠近样本的小 box，
触发 FFB 失败、tabu 累计、再切换。这浪费迭代且让小 box 反复被试探。

引入**体积奖励**：体积越大的 box 越容易被选为父节点，引导 RRT 优先从空旷区扩张。

## 问题：跨维度体积量纲

直接用 V = Πwidth_d 在不同 nd 下数值指数级膨胀/萎缩（2D 域 ~25，3D ~125，14D ~1e10）。
必须做**维度无关归一化**，且单位要与 `cspace_squared_dist_flat`（平方距离）一致。

## 方案

### 公式

设 box i 在域中：

- `V_i = Π_d width_d(box_i)`
- `V_dom = Π_d limits[d].width()`（域归一化基准）
- `r_i = V_i / V_dom ∈ (0, 1]` 无量纲体积比
- `diag_sq = Σ_d limits[d].width()²` 域对角线平方

体积奖励项（平方长度，与 d、tabu_step_sq 同量纲）：

$$
\text{vol\_bonus}_i = \alpha_{\text{vol}} \cdot \text{diag\_sq} \cdot r_i^{2/n_d}
$$

最近搜索打分：

$$
\text{scored}_i = d_i + \text{tabu\_step\_sq} \cdot \text{fail}_i - \text{vol\_bonus}_i
$$

### 关键设计

- `r_i^{1/nd}` ＝ box 与域的**等效边长比**，∈(0,1]，与维度无关
- 平方后乘 `diag_sq` ＝ 平方长度，与 `d`、`tabu_step_sq` 单位匹配
- 维度自适应：2D/3D/6D/14D 下奖励幅度始终落在 `(0, α·diag_sq]`
- α_vol = 0.05 默认（与 tabu 系数 0.0025 比 = 20:1，tabu 仍主导，体积只是软偏好）
- 对数空间维护 `box_log_volume`，避免 14D 下 V 数值溢出

## CLI 暴露

`exp_2d_trace --vol-bonus-alpha <float>`，默认 0.05；0 = 关闭。
后续可做 α ∈ {0, 0.01, 0.05, 0.1, 0.2, 0.5} 扫描实验。

## 改动文件

1. **`include/sbf/forest/grower.h`**
   - `GrowerConfig` 新增 `double vol_bonus_alpha = 0.05;`

2. **`src/forest/grower.cpp` 中 `grow_rrt`**
   - anti-stuck 块新增：
     - `log_V_dom = Σ log(limits[d].width())`
     - `box_log_volume` 数组（与 `boxes_` 同步），初始化遍历 `boxes_`
     - 读取 `config_.vol_bonus_alpha`
   - 两处最近搜索循环（line ~698 / ~710）：
     - `r_pow = exp((2.0/nd) * (box_log_volume[i] - log_V_dom))`
     - `scored = d + tabu_step_sq * box_fail_count[i] - vol_bonus * r_pow`
   - 新 box push 分支：`box_log_volume.push_back(log_V_new)`
   - SBF_TRACE 增加 `vol_r=%.4f` 字段

3. **`experiments/exp_2d_trace.cpp`**
   - CLI 新增 `--vol-bonus-alpha`
   - 透传到 `cfg.grower.vol_bonus_alpha`

4. **`scripts/run_2d_trace_log.py`**
   - argparse 新增 `--vol-bonus-alpha`，透传到二进制

## 验证步骤

1. 直接二进制基线：`α=0`（关闭）应与当前结果一致：boxes=60, success=60, fail=17, islands=1
2. 默认 α=0.05：期望 fail 下降、success/boxes 不退化、islands=1
3. 强偏好 α=0.5：观察是否过早终止小角落探索
4. 跑 MP4 管线 `α=0.05`，目视对比覆盖度
5. 多维度兼容：跑 `unit_envelope`(3D) 或现有 6D smoke 确认不退化

## 范围排除

- 不动 `snap_to_face`（保持 parent-face 邻接）
- 不动 `enforce_parent_adjacency`、`sample_unexplored`
- 不修改 `grow_wavefront`
