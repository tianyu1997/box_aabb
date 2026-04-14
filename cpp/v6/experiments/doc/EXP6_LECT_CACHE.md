# EXP6: LECT 缓存有效性 (LECT Cache Effectiveness)

> 论文章节: §LECT Cache Effectiveness (sec:exp_lect)
> 论文产出: **Table 6** (`tab6_lect_expansion.tex`)
> 代码入口: **独立 bench 脚本** (非 S1-S5, 从 bench_endpoint_iaabb 生成)
> 数据路径: 从 C++ 诊断输出解析 (非标准 JSON pipeline)
> **Table 6 已有完整数据** ✅

---

## 1. 实验目标

测量 LECT (Lifelong Envelope Cache Tree) 中每个节点的展开成本:
- **Cold start**: 根节点构建 (无父节点, 无缓存)
- **Hot expansion**: 深层展开 (depth ≥ 2, 利用 Z4 缓存 + 部分 FK 继承)

量化 Z4 caching 和 incremental FK 的缓存收益。

### 论文核心结论
1. **IFK 受益最大**: 7DOF Panda hot speedup 2.0-2.5×; 2DOF 达 3-9×
2. **CritSample 中等收益**: hot speedup 1.1-1.3×
3. **Analytical/GCPC: 计算主导**: speedup ≈ 1.0× (solver 每次从头计算)
4. **IFK envelopes 最松**: root volume 比 CritSample/Analytical 大 7-12×
5. **窄 intervals 更受益于细分**: w=0.10 leaf volume ~8× smaller than root

---

## 2. 实验矩阵

### 自变量

| 维度 | 取值 |
|------|------|
| Robot | Panda 7DOF, 2DOF planar |
| Endpoint Source | IFK, CritSample, Analytical, GCPC |
| Width (rad) | 0.10, 0.30, 0.50, 1.00 (Panda); +2.00 (2DOF) |

### 参数设置

| 参数 | 值 |
|------|------|
| 随机 intervals/width | 3 |
| LECT 展开深度 | 到叶节点 (取决于 min_edge) |
| 测量 | Cold = 根节点; Hot = depth ≥ 2 节点均值 |
| Build 配置 | Release MSVC 2022 /O2 |

---

## 3. 实验流程

```
对每个 robot ∈ {panda, 2dof}:
    对每个 source ∈ {IFK, CritSample, Analytical, GCPC}:
        对每个 width ∈ {0.10, 0.30, 0.50, 1.00, [2.00]}:
            重复 3 次 (随机 intervals):
                构建 LECT 树:
                    创建根节点 → 记录 cold_time_us
                    递归展开到叶 → 记录 每个 hot 节点 time_us
                    记录 root volume, leaf volume, root extent, leaf extent
            
            汇总:
                cold_us = mean(3 × root_time)
                hot_us = mean(all deeper nodes across 3 repeats)
                speedup = cold_us / hot_us
                root_vol, leaf_vol, root_ext, leaf_ext
```

---

## 4. 输出指标

| 指标 | 类型 | 含义 |
|------|------|------|
| `source` | str | Endpoint source |
| `width` | float (rad) | Box 宽度 |
| `cold_us` | float (μs) | 根节点构建时间 |
| `hot_us` | float (μs) | 深层展开时间 (均值) |
| `speedup` | float | cold / hot |
| `root_vol` | float (m³) | 根节点 envelope volume |
| `leaf_vol` | float (m³) | 叶节点 envelope volume |
| `root_ext` | float (m) | 根节点 envelope extent (最大边长) |
| `leaf_ext` | float (m) | 叶节点 envelope extent |

---

## 5. Table 6 当前数据 (已完整)

Table 6 已从 `bench_endpoint_iaabb` 独立测试生成, 包含完整数据:

### Panda 7-DOF 关键数据

| Source | Width | Cold (μs) | Hot (μs) | Speedup | Root Vol | Leaf Vol |
|--------|-------|-----------|----------|---------|----------|----------|
| IFK | 0.10 | 73.8 | 29.6 | 2.49× | 0.016 | 0.002 |
| IFK | 1.00 | 61.6 | 31.3 | 1.97× | 26.30 | 3.726 |
| CritSample | 0.10 | 296.7 | 269.1 | 1.10× | 0.002 | <0.001 |
| Analytical | 0.10 | 14,791 | 15,959 | 0.93× | 0.002 | <0.001 |
| GCPC | 0.10 | 12,606 | 13,283 | 0.95× | 0.002 | <0.001 |

### 2-DOF 关键数据

| Source | Width | Cold (μs) | Hot (μs) | Speedup |
|--------|-------|-----------|----------|---------|
| IFK | 0.10 | 39.9 | 5.8 | 6.83× |
| IFK | 2.00 | 61.8 | 6.8 | 9.11× |
| GCPC | 0.10 | 34.7 | 12.3 | 2.81× |

---

## 6. 当前状态

| 项目 | 状态 | 说明 |
|------|------|------|
| Table 6 LaTeX | ✅ 完整 | `tab6_lect_expansion.tex` 已生成 |
| 数据来源 | ✅ bench_endpoint_iaabb | 从 C++ 独立 bench 生成 |
| 集成到 run_all_experiments.py | ❌ 未集成 | 以独立脚本执行 |
| JSON 数据文件 | ❌ 无标准 JSON | 数据在 LaTeX 中, 未存为 JSON |

---

## 7. 实现差距

### 若需要重新生成数据:

1. **C++ bench 程序**: 需要独立的 LECT bench 可执行文件或 Python binding
   - 当前 `bench_endpoint_iaabb` 是 C++ 独立程序
   - 可能位于 build 目标中 (检查 CMakeLists.txt)

2. **Python 集成**: 创建 `run_s6_lect()` 函数
   - 需要 Python binding 暴露 LECT 逐节点展开 API
   - 或解析 C++ bench 程序的 stdout

3. **数据标准化**: 将 LaTeX 表格数据反解析为 JSON
   - `results/paper/s6_lect_cache/results.json`

### C++ bench 位置检查

```powershell
# 查找 bench_endpoint_iaabb 相关文件
Get-ChildItem -Recurse -Filter "*bench*" v5/
```

---

## 8. 执行命令

### 若使用已有数据
Table 6 已经完整, 无需重新执行。直接使用现有 `tab6_lect_expansion.tex`。

### 若需重新生成
```powershell
# 方案 A: 运行 C++ bench 程序 (需确认可执行文件位置)
cd build_x64/Release
./bench_endpoint_iaabb.exe --lect-expansion --output lect_results.json

# 方案 B: 通过 Python binding (若已暴露)
python -c "
import sbf5
robot = sbf5.Robot.from_json('data/panda.json')
# ... LECT expansion benchmark
"
```

---

## 9. 风险与注意事项

1. **数据已完成**: Table 6 数据充分, 无需重跑 (除非修改了 LECT 核心代码)
2. **bench_endpoint_iaabb 可能过期**: 若 LECT 代码有重大修改, 数据可能不再准确
3. **subtree_occ 修改影响**: 之前在 FFB 中移除了 subtree_occ → LECT 中仍保留 (但不影响 bench)
4. **论文一致性**: 确认 bench 使用的机器人与论文描述一致 (IIWA14 vs Panda)
