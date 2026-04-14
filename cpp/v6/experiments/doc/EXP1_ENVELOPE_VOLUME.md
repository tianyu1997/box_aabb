# EXP1: Envelope 紧密度对比 (Envelope Volume Tightness)

> 论文章节: §Envelope Volume and Timing (sec:exp_pipeline)
> 论文产出: **Table 1** (`tab1_envelope_volume.tex`)
> 代码入口: `run_all_experiments.py → run_s1()`
> 数据路径: `results/paper/s1_envelope_tightness/results.json`

---

## 1. 实验目标

量化 4×3 = 12 pipeline 配置产生的 envelope 紧密程度差异。
紧密度以 **envelope volume (m³)** 衡量——越小表示包络越贴合真实扫掠体积。

论文核心结论:
- CritSample+LinkIAABB 比 IFK+LinkIAABB 紧 **2.6×**
- Hull16_Grid 比 LinkIAABB 紧 **42%**
- Analytical ≈ GCPC (差异 < 1.4%)
- 推荐配置: **CritSample + Hull16_Grid** (V = 0.117 m³)

---

## 2. 实验矩阵

### 自变量

| 维度 | 取值 | 说明 |
|------|------|------|
| Robot | `2dof`, `panda` | 2DOF planar + 7DOF Panda |
| Endpoint Source | `IFK`, `CritSample`, `Analytical`, `GCPC` | 4 种 endpoint-iAABB 源 |
| Envelope Type | `LinkIAABB`, `LinkIAABB_Grid`, `Hull16_Grid` | 3 种包络表示 |

### 参数设置

| 参数 | Quick | Lite | **Full** |
|------|-------|------|----------|
| n_boxes | 50 | 200 | **500** |
| EP sources | IFK, CritSample | 全部 4 个 | **全部 4 个** |
| Envelope types | LinkIAABB, LinkIAABB_Grid | 全部 3 个 | **全部 3 个** |
| 总行数 | 2×2×2 = 8 | 2×4×3 = 24 | **2×4×3 = 24** |
| 随机种子 | 42 | 42 | **42** |
| Box 宽度 | [0.1, 0.5] rad | [0.1, 0.5] rad | **[0.1, 0.5] rad** |

### 依赖数据
- `data/2dof_planar.json` — 2DOF 机器人模型
- `data/panda.json` — 7DOF Panda 机器人模型
- `data/2dof_500.gcpc` — 2DOF GCPC 缓存 (GCPC source 需要)
- `data/panda_5000.gcpc` — Panda GCPC 缓存 (GCPC source 需要)

---

## 3. 实验流程

```
对每个 robot ∈ {2dof, panda}:
    rng = RandomState(42)
    加载 GCPC cache (若有)
    
    对每个 endpoint_source ∈ {IFK, CritSample, Analytical, GCPC}:
        对每个 envelope_type ∈ {LinkIAABB, LinkIAABB_Grid, Hull16_Grid}:
            volumes, safe_flags, times = [], [], []
            
            对每个 box_idx ∈ range(500):
                intervals = 随机采样 joint intervals (width ∈ [0.1, 0.5])
                info = sbf5.compute_envelope_info(robot, intervals, ep_cfg, env_cfg, gcpc)
                volumes.append(info["volume"])
                safe_flags.append(info["is_safe"])
                times.append(info["total_time_us"])
            
            记录 row: {robot, endpoint, envelope, volume_mean, volume_std, safe, time_us_mean}

计算 ratio_pct = 100 × (volume_mean / IFK-LinkIAABB baseline) per robot
```

---

## 4. 输出指标

| 指标 | 类型 | 含义 |
|------|------|------|
| `volume_mean` | float (m³) | 500 个 box 的平均 envelope 体积 |
| `volume_std` | float | volume 标准差 |
| `safe` | bool | 所有 box 是否均为保守安全包络 |
| `time_us_mean` | float (μs) | 平均计算耗时 |
| `ratio_pct` | float (%) | 相对 IFK-LinkIAABB 基准的体积比 |
| `n_boxes` | int | 采样 box 数 |

---

## 5. 输出格式

```json
{
  "rows": [
    {
      "robot": "panda",
      "endpoint": "CritSample",
      "envelope": "Hull16_Grid",
      "volume_mean": 0.117,
      "volume_std": 0.03,
      "safe": false,
      "time_us_mean": 278.0,
      "n_boxes": 500,
      "ratio_pct": 22.6
    },
    // ... 23 more rows
  ]
}
```

---

## 6. 预期结果验证

| 检查点 | 预期 |
|--------|------|
| IFK volume 最大 | ✓ (最保守) |
| CritSample ≤ Analytical ≈ GCPC ≤ IFK (volume) | ✓ |
| Hull16_Grid ≤ LinkIAABB_Grid ≤ LinkIAABB (volume) | ✓ |
| IFK/Analytical/GCPC: `safe = true` | ✓ (保守安全) |
| CritSample: `safe = false` | ✓ (非保守, 边界采样) |
| ratio_pct 范围 | 22% ~ 100% |
| Analytical vs GCPC volume 差异 | < 1.4% |

---

## 7. 当前状态与差距

| 项目 | 当前 | 目标 | 差距 |
|------|------|------|------|
| n_boxes | 50 (quick) | **500** | 需删除旧 results.json 重跑 |
| EP sources | IFK, CritSample | **4 个全部** | 缺 Analytical, GCPC |
| Envelope types | LinkIAABB, LinkIAABB_Grid | **3 个全部** | 缺 Hull16_Grid |
| 总行数 | 8 | **24** | 缺 16 行 |
| GCPC cache | 未验证 | 需检查 | 确认 data/*.gcpc 是否存在 |

---

## 8. 执行命令

```powershell
cd safeboxforest/v5/
$env:PYTHONPATH = "build_x64/Release;python"

# 删除旧的 quick 模式结果
Remove-Item "results/paper/s1_envelope_tightness/results.json" -Force

# 执行完整实验 (约 10 分钟)
python -u python/scripts/run_all_experiments.py --skip-figures 2>&1 | Tee-Object -FilePath s1_log.txt
# 或单独执行 S1:
python -c "
import sys; sys.path.insert(0, 'python')
from scripts.run_all_experiments import run_s1
run_s1(quick=False, lite=False)
"
```

---

## 9. Table 1 LaTeX 格式参考

```
Source × Envelope | 2-DOF V | 2-DOF Ratio | 7-DOF V | 7-DOF Ratio | Cert.
IFK / LinkIAABB   | 0.172   | 100.0%      | 0.521   | 100.0%      | ✓
CritSample/Hull16 | 0.097   | 56.2%       | 0.117   | 22.6%       |
...
```

由 `gen_paper_tables.py → gen_tab1()` 生成 → `paper/generated/tab1_envelope_volume.tex`

---

## 10. 风险与注意事项

1. **GCPC cache 缺失**: 若 `data/panda_5000.gcpc` 不存在, GCPC 行将跳过
2. **Analytical source 慢**: 每个 box ~15ms, 500 boxes × 2 robots = ~30s
3. **GCPC source 慢**: 每个 box ~13ms, 500 boxes × 2 robots = ~26s
4. **Quick 模式残留**: 旧的 8 行 results.json 会导致跳过, 必须先删除
5. **随机种子固定**: seed=42, 确保可复现
