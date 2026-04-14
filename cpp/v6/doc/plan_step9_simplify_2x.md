# Plan Step 9: 预竞争简化用 2x 分辨率 (P1)

## 目标
让 pre-compete simplify 真正起效。当前 5/5 查询的简化结果都被 2x 验证回退（因为 1x 分辨率简化产生的段在 2x 分辨率下碰撞），完全浪费了简化计算。

## 基线数据
- 5/5 查询 pre-compete simplify 全部 REVERTED
- 简化后 chain 保持原始长度（16-27 rad），RRT 靠自身获胜

## 修改文件
- `src/planner/sbf_planner.cpp` — pre-compete simplify 段

## 实现步骤

### 9a. 简化时直接使用 2x 分辨率
**位置**: `sbf_planner.cpp` ~L895-920（pre-compete simplify 区域）

当前逻辑:
```cpp
auto ares_fn = [&](const VectorXd& a, const VectorXd& b) -> int {
    double len = (b - a).norm();
    return std::max(20, static_cast<int>(std::ceil(len / 0.005)));
};
// ... check_segment(path[cur], path[j], ares_fn(path[cur], path[j]))
```

改为: 在简化 check_segment 时使用 2 倍分辨率:
```cpp
// 简化用 2x 分辨率，确保结果在后续验证也通过
if (!checker.check_segment(path[cur], path[j],
                           2 * ares_fn(path[cur], path[j]))) {
    best = j;
    break;
}
```

### 9b. 移除冗余的 2x 后验证
**位置**: ~L910-930

由于简化本身已经使用 2x 分辨率，不再需要 simplify_valid 验证步骤。
但保留为 **3x 安全验证**以防万一:
```cpp
// 3x safety validation (since simplify used 2x)
if (checker.check_segment(s[i], s[i+1], 3 * ares_fn(s[i], s[i+1]))) {
    simplify_valid = false;
    break;
}
```

### 9c. 自适应跳跃回退
如果最远可达点距离 > 3 rad，尝试回退 1 步以增加安全余量:
```cpp
if (best > cur + 2 && (path[best] - path[cur]).norm() > 3.0) {
    // 验证 best-1 也可到达
    if (!checker.check_segment(path[cur], path[best-1],
                               2 * ares_fn(path[cur], path[best-1]))) {
        // best-1 同样可达且更保守，但仍用 best（更短）
    }
}
```
注: 此步骤可选，先实现 9a+9b 看效果。

## 验证方法
```bash
rm -f ~/.sbf_cache/kuka_iiwa14_r820_*.lect
./experiments/exp2_e2e_planning --seeds 1 2>&1 | grep -E 'simplify|compete|VAL|seed=|q='
# 期望: 部分查询的 simplify 不再 REVERTED
# 期望: chain_len 降低 → RRT 更容易赢
```

## 预期效果
- 2-4/5 查询的简化保留生效（chain 从 16-27 rad 降至 7-11 rad）
- RRT 仍可能赢（但 chain 做了有效竞争）
- 无碰撞退化（2x 分辨率保证安全）

## 风险
- 低：简化更保守 = 跳跃距离略短，但仍比原始 chain 短很多
- 极端情况: 某些窄走廊 2x 也不够 → 3x 后验证捕获
