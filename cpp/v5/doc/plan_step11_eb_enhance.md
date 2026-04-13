# Plan Step 11: 弹性带增强 (P3)

## 目标
增加弹性带优化的迭代次数和 alpha 梯度，挤出更多路径缩短。

## 基线数据
| 查询 | EB 前 | EB 后 | Pass2 后 | 总压缩 |
|------|-------|-------|----------|--------|
| AS→TS | 6.767 | 6.232 | 6.083 | −10.1% |
| TS→CS | 6.187 | 5.985 | — | −3.3% |
| CS→LB | 10.492 | 9.590 | 8.920 | −15.0% |
| LB→RB | 5.596 | 4.883 | — | −12.7% |
| RB→AS | 2.196 | 2.050 | — | −6.6% |

## 修改文件
- `src/planner/sbf_planner.cpp` — EB 主循环 + Pass2 EB

## 实现步骤

### 11a. 主 EB 增强
**位置**: `sbf_planner.cpp` ~L1128-1170（elastic band 主循环）

当前: 60 iter, alphas = [0.5, 0.3, 0.15, 0.05]

改为:
```cpp
// 增加到 100 次迭代
for (int iter = 0; iter < 100; ++iter) {
    // ...
    // 增加更细粒度的 alpha
    static const double alphas[] = {0.5, 0.3, 0.15, 0.08, 0.03};
    // ...
}
```

### 11b. Pass2 EB 增强
**位置**: ~L1295-1330

当前: 30 iter, alphas = [0.4, 0.2, 0.08]

改为:
```cpp
for (int iter = 0; iter < 50; ++iter) {
    // ...
    static const double alphas2[] = {0.4, 0.2, 0.1, 0.04};
    // ...
}
```

### 11c. Pass3 超精细 EB (可选)
在 pass2 后添加 pass3:
```cpp
// Pass3: ultra-fine EB (20 iter, tiny steps)
{
    double len_before3 = path_length(path);
    int moves3 = 0;
    for (int iter = 0; iter < 20; ++iter) {
        bool any = false;
        for (size_t i = 1; i + 1 < path.size(); ++i) {
            // ... 与 pass2 相同结构
            static const double alphas3[] = {0.02, 0.01};
            // ...
        }
        if (!any) break;
    }
    // 碰撞验证 + 回退
}
```

### 11d. EB 收敛检测优化
当前每 iter 检查 any_move。增加：如果连续 3 iter 的 len 减少 < 0.001，提前退出。
```cpp
double prev_len = path_length(path);
int stagnant = 0;
for (int iter = 0; iter < 100; ++iter) {
    // ... EB moves
    double cur_len = path_length(path);
    if (prev_len - cur_len < 0.001) stagnant++;
    else stagnant = 0;
    if (stagnant >= 3) break;
    prev_len = cur_len;
}
```

## 验证方法
```bash
rm -f ~/.sbf_cache/kuka_iiwa14_r820_*.lect
./experiments/exp2_e2e_planning --seeds 1 2>&1 | grep -E 'elastic|pass2|pass3|VAL|seed=|q='
# 对比 EB moves 数量和 len 变化
```

## 预期效果
- 额外 1-3% 路径缩短
- CS→LB: 8.920 → ~8.6-8.8（更多 EB 迭代挤出余量）
- 计算成本增加 ~20-50ms（EB 本身很轻量，点数少）

## 风险
- 极低：EB 只向更优移动，有碰撞回退保护
- 更多迭代 = 更多碰撞检查 = 微量时间增加
- Pass3 价值可能很小（如果 pass2 已收敛）
