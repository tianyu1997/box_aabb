# EXP2: Envelope 计算耗时 (Envelope Computation Timing)

> 论文章节: §Envelope Volume and Timing (sec:exp_pipeline)
> 论文产出: **Table 2** (`tab2_computation_timing.tex`)
> 代码入口: `run_all_experiments.py → run_s2()`
> 数据路径: `results/paper/s2_envelope_timing/results.json`

---

## 1. 实验目标

精确测量四种 endpoint-iAABB source 和三种 envelope representation 在不同组合下的计算速度。
分离 **endpoint 计算时间** 和 **envelope 光栅化时间**, 计算相对 Analytical-LinkIAABB 的加速比。

论文核心结论:
- IFK+LinkIAABB 最快: **4.8 μs** (加速 **3174×**)
- CritSample: **264 μs** (加速 **57×**)
- Analytical: **15,199 μs** (基准 1.0×)
- GCPC: **12,788 μs** (加速 **1.2×**)
- Hull16_Grid 的 envelope 光栅化: ~22 μs (额外开销有限)

---

## 2. 实验矩阵

### 自变量

| 维度 | 取值 | 说明 |
|------|------|------|
| Robot | `2dof`, `panda` | 2DOF planar + 7DOF Panda |
| Endpoint Source | `IFK`, `CritSample`, `Analytical`, `GCPC` | 4 种 |
| Envelope Type | `LinkIAABB`, `LinkIAABB_Grid`, `Hull16_Grid` | 3 种 |

### 参数设置

| 参数 | Quick | Lite | **Full** |
|------|-------|------|----------|
| n_boxes | 100 | 300 | **1000** |
| n_repeats | 10 | 20 | **50** |
| EP sources | 2 | 4 | **4** |
| Envelope types | 2 | 3 | **3** |
| 总样本/config | 1,000 | 6,000 | **50,000** |
| 随机种子 | 123 | 123 | **123** |

### 依赖数据
- `data/2dof_planar.json`, `data/panda.json`
- `data/2dof_500.gcpc`, `data/panda_5000.gcpc` (GCPC source 需要)

---

## 3. 实验流程

```
对每个 robot ∈ {2dof, panda}:
    rng = RandomState(123)
    预采样 n_boxes=1000 个随机 box
    
    对每个 endpoint_source:
        对每个 envelope_type:
            ep_times, env_times, total_times = [], [], []
            
            对每个 box (1000 个):
                重复 n_repeats=50 次:
                    info = sbf5.compute_envelope_info(...)
                    ep_times.append(info["ep_time_us"])
                    env_times.append(info["env_time_us"])
                    total_times.append(info["total_time_us"])
            
            记录 row: {ep_us_mean, ep_us_std, env_us_mean, env_us_std,
                       total_us_mean, total_us_std, n_samples}

计算 speedup = Analytical-LinkIAABB.total / this.total (per robot)
```

---

## 4. 输出指标

| 指标 | 类型 | 含义 |
|------|------|------|
| `ep_us_mean` | float (μs) | endpoint-iAABB 源计算耗时均值 |
| `ep_us_std` | float | endpoint 耗时标准差 |
| `env_us_mean` | float (μs) | envelope 光栅化耗时均值 |
| `env_us_std` | float | envelope 耗时标准差 |
| `total_us_mean` | float (μs) | 总耗时 = ep + env |
| `total_us_std` | float | 总耗时标准差 |
| `speedup` | float | 相对 Analytical-LinkIAABB 加速比 |
| `n_samples` | int | 总样本数 (1000 × 50 = 50000) |

---

## 5. 输出格式

```json
{
  "rows": [
    {
      "robot": "panda",
      "endpoint": "IFK",
      "envelope": "LinkIAABB",
      "ep_us_mean": 4.8,
      "ep_us_std": 1.2,
      "env_us_mean": 0.0,
      "env_us_std": 0.0,
      "total_us_mean": 4.8,
      "total_us_std": 1.2,
      "n_samples": 50000,
      "speedup": 3174.0
    },
    // ... 23 more rows
  ]
}
```

---

## 6. 预期结果验证

| 检查点 | 预期 |
|--------|------|
| EP 速度排序 | IFK < CritSample ≪ GCPC < Analytical |
| ENV 速度排序 | LinkIAABB < LinkIAABB_Grid < Hull16_Grid |
| IFK 总耗时 | ~5 μs (7DOF), ~2 μs (2DOF) |
| Analytical 总耗时 | ~15 ms (7DOF) |
| GCPC 总耗时 | ~13 ms (7DOF) |
| CritSample 总耗时 | ~265 μs (7DOF) |
| Hull16_Grid env 增量 | ~20 μs |
| speedup 范围 | 1.0× ~ 3174× |

---

## 7. 当前状态与差距

| 项目 | 当前 | 目标 | 差距 |
|------|------|------|------|
| n_boxes × n_repeats | 100 × 10 = 1,000 | **1000 × 50 = 50,000** | 50× 更多样本 |
| EP sources | IFK, CritSample | **4 个全部** | 缺 Analytical, GCPC |
| Envelope types | LinkIAABB, LinkIAABB_Grid | **3 个全部** | 缺 Hull16_Grid |
| 总行数 | 8 | **24** | 缺 16 行 |
| speedup 基准 | 未计算 (无 Analytical) | 有 | 需重跑 |

---

## 8. 执行命令

```powershell
# 删除旧结果, 执行完整 S2
Remove-Item "results/paper/s2_envelope_timing/results.json" -Force

# Full 模式 (~15 分钟)
# Analytical: 1000 boxes × 50 reps × ~15ms = ~750s per config
# 总计: 2 robots × 12 configs × (IFK ~0.05s + CritSample ~13s + Analytical ~750s + GCPC ~640s) ≈ 30+ min
python -c "
import sys; sys.path.insert(0, 'python')
from scripts.run_all_experiments import run_s2
run_s2(quick=False, lite=False)
"
```

---

## 9. Table 2 LaTeX 格式参考

```
Source / Envelope | EP (μs) | Env (μs) | Total (μs) | Speedup
IFK / LinkIAABB   | 4.8     | 0.00     | 4.8        | 3,174×
Analytical / LIAABB| 15,198  | 0.3      | 15,199     | 1.0×
```

由 `gen_paper_tables.py → gen_tab2()` 生成 → `paper/generated/tab2_computation_timing.tex`

---

## 10. 风险与注意事项

1. **Analytical 极慢**: 1000 × 50 × 15ms = 750s/config, 但只有 3 个 Analytical config
2. **GCPC 慢**: 1000 × 50 × 13ms = 650s/config
3. **总运行时间**: Full 模式约 ~30-45 分钟 (主要是 Analytical + GCPC)
4. **Lite 模式折衷**: 300 × 20 = 6000 样本, 约 ~10 分钟
5. **预热效应**: 前几次调用可能较慢 (cache 预热), n_repeats=50 可稀释
6. **内存**: 50k 样本 × 3 数组 × 24 rows = ~3.6M 浮点数, 无问题
