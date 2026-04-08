# EXP5: 冻结 vs 直接计算对比 (Freeze vs. Direct Comparison)

> 论文章节: §Freeze vs. Direct Comparison (sec:exp_freeze)
> 论文产出: **Table 5** (`tab5_freeze_comparison.tex`)
> 代码入口: **尚未实现** (不在 S1-S5 code phases 中)
> 数据路径: `results/paper/s5_freeze_comparison/results.json` (待创建)
> **Table 5 当前为 STUB (data not yet generated)**

---

## 1. 实验目标

对比 **直接计算** (freeze depth k=0) 和 **自适应冻结-重构 pipeline**
在四种 endpoint source 下的性能权衡:
- 体积膨胀率 (Ratio): 冻结后 volume / 直接 volume
- 加速比 (Speedup): 直接 time / 冻结 time

### 冻结策略参数
- 最小自由关节数 m = 3
- 宽度阈值 θ = π/2
- 自适应冻结深度: k* = 4 (窄 box) 或 k* = 0 (宽 box)

### 论文核心结论
1. **IFK: 冻结几乎无额外开销** — Ratio ≈ 0.8-1.1 (冻结 = iFK 链分解)
2. **CritSample/Analytical/GCPC: 中等宽度有膨胀** — Ratio 2-3×
3. **GCPC/Analytical 窄宽度: 巨大加速** — GCPC 31-107×, Analytical 6-9×
4. **宽宽度自适应关闭** — k*=0 时结果完全一致

---

## 2. 实验矩阵

### 自变量

| 维度 | 取值 | 说明 |
|------|------|------|
| Endpoint Source | IFK, CritSample, Analytical, GCPC | 4 种 |
| Box 宽度 (w) | 0.10, 0.20, 0.40, 0.80, 1.60 rad | 5 个宽度级别 |
| 冻结模式 | Direct (k=0), Adaptive (k=k*) | 2 种 |

### 参数设置

| 参数 | 值 |
|------|------|
| Robot | 7DOF Panda (IIWA14) |
| n_boxes | 50 per width |
| 总组合 | 4 sources × 5 widths × 2 modes × 50 boxes = 2000 次 compute |
| 随机种子 | 42 |
| Envelope type | LinkIAABB (固定, 隔离冻结效应) |
| Freeze params | m=3, θ=π/2 |

---

## 3. 实验流程

```
对每个 endpoint_source ∈ {IFK, CritSample, Analytical, GCPC}:
    对每个 width ∈ {0.10, 0.20, 0.40, 0.80, 1.60}:
        rng = RandomState(42)
        volumes_direct, volumes_freeze = [], []
        times_direct, times_freeze = [], []
        
        对每个 box ∈ range(50):
            intervals = 随机采样 (width 固定)
            
            # 直接计算 (k=0)
            info_direct = sbf5.compute_envelope_info(
                robot, intervals, ep_cfg, env_cfg, gcpc,
                freeze_depth=0)
            volumes_direct.append(info_direct["volume"])
            times_direct.append(info_direct["total_time_us"])
            
            # 自适应冻结 (k=k*)
            info_freeze = sbf5.compute_envelope_info(
                robot, intervals, ep_cfg, env_cfg, gcpc,
                freeze_depth="adaptive")  # or -1 for auto
            volumes_freeze.append(info_freeze["volume"])
            times_freeze.append(info_freeze["total_time_us"])
        
        记录:
            ratio = mean(volumes_freeze) / mean(volumes_direct)
            speedup = mean(times_direct) / mean(times_freeze)
            k_star = 自适应策略选择的冻结深度
```

---

## 4. 输出指标

| 指标 | 类型 | 含义 |
|------|------|------|
| `source` | str | endpoint source 名 |
| `width` | float (rad) | box 宽度 |
| `k_star` | int | 自适应冻结深度 |
| `volume_direct` | float (m³) | 直接计算 volume 均值 |
| `volume_freeze` | float (m³) | 冻结计算 volume 均值 |
| `ratio` | float | volume_freeze / volume_direct |
| `time_direct_us` | float (μs) | 直接计算耗时 |
| `time_freeze_us` | float (μs) | 冻结计算耗时 |
| `speedup` | float | time_direct / time_freeze |

---

## 5. Table 5 格式

```
Source     | Width | k* | Ratio | Speedup
IFK        | 0.10  | 4  | 0.83  | 1.0×
IFK        | 0.80  | 0  | 1.00  | 1.0×
CritSample | 0.10  | 4  | 2.31  | 3.2×
Analytical | 0.10  | 4  | 2.15  | 6.8×
GCPC       | 0.10  | 4  | 2.08  | 107×
GCPC       | 0.80  | 0  | 1.00  | 1.0×
```

---

## 6. 预期结果

| Source | 窄 (w=0.1) | 中 (w=0.4) | 宽 (w=1.6) |
|--------|------------|------------|------------|
| IFK Ratio | 0.8-1.1 | 1.0 | 1.0 |
| IFK Speedup | ~1.0× | 1.0× | 1.0× |
| CritSample Ratio | 2-3× | 1.5× | 1.0× |
| Analytical Speedup | 6-9× | 2-3× | 1.0× |
| GCPC Speedup | **31-107×** | 5-10× | 1.0× |

---

## 7. 实现差距分析

### 需要新增的代码

1. **C++ 端: freeze_depth 参数暴露**
   - `compute_envelope_info()` 需接受 `freeze_depth` 参数
   - 或通过 `EndpointSourceConfig` 的 freeze 相关字段控制
   - 当前是否已有 freeze_depth 支持? → 需检查 C++ bindings

2. **Python 端: 独立实验函数**
   - 在 `run_all_experiments.py` 中新增 `run_s5_freeze()` 或
   - 创建独立脚本 `exp_freeze_comparison.py`

3. **gen_tables.py 更新**
   - `gen_tables.py` 已有 `tab5_freeze_comparison.tex` 的写入逻辑 (line 95)
   - 但内容为 stub, 需接入实际数据

### 可能的 C++ 检查点

```python
# 检查 EndpointSourceConfig 是否有 freeze 相关属性
import sbf5
cfg = sbf5.EndpointSourceConfig()
# 查看 dir(cfg) 是否包含 freeze_depth, max_freeze_joints 等
```

---

## 8. 执行命令 (待实现后)

```powershell
# 独立执行冻结对比实验
python python/scripts/exp_freeze_comparison.py

# 或集成后
python -c "
import sys; sys.path.insert(0, 'python')
from scripts.run_all_experiments import run_s5_freeze
run_s5_freeze()
"
```

### 预估运行时间
- 4 sources × 5 widths × 50 boxes × 2 modes = 2000 compute
- IFK: ~5μs × 500 = ~2.5ms
- CritSample: ~270μs × 500 = ~0.14s
- Analytical: ~15ms × 500 = ~7.5s
- GCPC: ~13ms × 500 = ~6.5s
- **总计: ~30 秒** (非常快, 因为只有单 box compute, 无 planning)

---

## 9. 风险与注意事项

1. **freeze_depth 可能未在 Python bindings 中暴露**: 需检查 C++ 侧
   - 若未暴露, 需修改 `sbf5_bindings.cpp` 添加 freeze_depth 参数
2. **自适应策略逻辑**: k* 的计算可能在 C++ 内部, 需确认可从 Python 控制
3. **IFK 不受冻结影响**: IFK 本身就是链式间隔运算, freeze 等价于子链分拆
4. **Tab 6 (LECT) 已有完整数据**: 说明底层 C++ freeze 机制工作正常
5. **论文措辞**: "IIWA14 manipulator" — 确认实验用的是 IIWA14 还是 Panda
   (代码用 panda.json, 论文说 IIWA14, 可能需统一)
