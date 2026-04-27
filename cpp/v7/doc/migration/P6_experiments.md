# P6 — 实验系统：11 脚本整合、--quick 旗标与夜间 CI

> **目标**: 整合 v6 的所有实验脚本（~30 个散落的 `.py`），
> 统一到 11 个标准实验脚本，添加 `--quick` 旗标（D3 决策），
> 配置夜间全量实验 CI。消除所有孤儿实验脚本。
>
> **预计工时**: 3-4 天  
> **前置条件**: P5 完成（端到端 SR=100%）  
> **完成标准**: `make run_quick_all` ≤5 分钟完成；夜间 CI pipeline 可触发

---

## 0. 架构决策 D3 在本 Phase 的核心作用

> **决策 D3**: CI gate 仅跑 `--quick`（≤5 min），夜间机器跑全量实验。

**实施要求**:
- 每个实验脚本**必须**接受 `--quick` CLI flag（`argparse`）
- `--quick` 行为：`n_trials=3, seed=[0,1,2], timeout=30s`
- 全量行为：`n_trials=20, seed=[0..19]`
- 脚本内**禁止**硬编码 quick/full 模式（必须通过 flag 控制）
- `results_nightly/` 存全量结果；`results_quick/` 存 CI 结果

---

## 1. v6 实验脚本盘点

v6 的实验脚本分布在多个目录：

| 分类 | 脚本数 | 说明 |
|------|--------|------|
| **MUST KEEP** | 9 | B1-B6 实验基础 + 主对比实验 |
| **APPENDIX** | 4 | 论文附录实验 |
| **ORPHAN** | ~17 | 调试脚本，无论文对应 |

**ORPHAN 脚本处理原则**: 归档到 `v7/experiments/archive/`（不删除），
不迁移到 `v7/scripts/`。

---

## 2. v7 的 11 个标准实验脚本

```
scripts/
  run_exp_main.py          # EXP1: SBF Full vs 消融基准（主实验）
  run_exp_ablation.py      # EXP2: 4 消融条件 × 4 场景（B3 数据源）
  run_exp_crossrobot.py    # EXP3: Panda 7-DoF 跨机器人（B1 数据源）
  run_exp_sensitivity.py   # EXP4: goal_bias 敏感性（B4 数据源）
  run_exp_irisnp.py        # EXP5: IRIS-NP+GCS 对比（B5/B6 数据源）
  run_exp_iriszo.py        # EXP6: IRIS-ZO 对比
  run_exp_prm.py           # EXP7: PRM sweep（样本数 vs SR）
  run_exp_pathopt.py       # EXP8: PathOpt 各步骤贡献分析
  run_exp_appendix_a.py    # APP-A: 计算复杂度分析
  run_exp_appendix_b.py    # APP-B: 参数影响
  build_tables.py          # 后处理：JSON → LaTeX 表格
```

---

## 3. 统一脚本模板

所有 11 个脚本遵循相同的 CLI 结构：

```python
#!/usr/bin/env python3
"""
run_exp_main.py - 主实验：SBF Full vs 消融基准

用法:
    python run_exp_main.py [--quick] [--seed SEED [SEED ...]] [--output DIR]
"""

import argparse
import json
import sys
from pathlib import Path

# 导入 v7 Python binding
from sbf import SbfPlanner, PlannerConfig, Scene, UrdfEndpointSource, CollisionConfig

def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--quick",
        action="store_true",
        help="快速模式：n_trials=3, seed=[0,1,2], timeout=30s（CI 用）"
    )
    parser.add_argument(
        "--seed",
        type=int,
        nargs="+",
        default=None,
        help="指定 seed 列表，覆盖 --quick/full 默认值"
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="结果输出目录（默认: results_quick/ 或 results_nightly/）"
    )
    parser.add_argument(
        "--config",
        type=str,
        default="configs/iiwa14_scenes.yaml",
        help="实验配置文件"
    )
    return parser.parse_args()


def get_experiment_params(args):
    """根据 --quick 确定实验参数（集中在此函数，不散落各处）"""
    if args.quick:
        return {
            "n_trials": 3,
            "seeds": list(range(3)),
            "timeout_s": 30,
            "output_dir": Path("results_quick"),
        }
    else:
        return {
            "n_trials": 20,
            "seeds": list(range(20)),
            "timeout_s": 120,
            "output_dir": Path("results_nightly"),
        }


def run_experiment(params, config_path):
    """主实验逻辑（不含 CLI 解析）"""
    results = []
    # ... 实验实现 ...
    return results


def main():
    args = parse_args()
    params = get_experiment_params(args)

    # 若用户指定了 --seed，覆盖默认值
    if args.seed is not None:
        params["seeds"] = args.seed
        params["n_trials"] = len(args.seed)

    # 输出目录
    output_dir = Path(args.output) if args.output else params["output_dir"]
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"[实验] 模式: {'quick' if args.quick else 'full'}, "
          f"trials: {params['n_trials']}, seeds: {params['seeds']}")

    results = run_experiment(params, args.config)

    output_file = output_dir / "exp_main.json"
    with open(output_file, "w") as f:
        json.dump(results, f, indent=2)
    print(f"[完成] 结果写入: {output_file}")


if __name__ == "__main__":
    main()
```

---

## 4. build_tables.py — 后处理管道

```python
#!/usr/bin/env python3
"""
build_tables.py - 从 JSON 结果生成 LaTeX 表格

用法:
    python build_tables.py --source results_nightly/ --output ../doc/generated/
    python build_tables.py --source results_quick/ --output ../doc/generated/ --draft
"""

import argparse
import json
from pathlib import Path

# 表格生成器映射（每个实验对应一个生成函数）
TABLE_GENERATORS = {
    "exp_main":       generate_tab_main,
    "exp_ablation":   generate_tab_b3,
    "exp_crossrobot": generate_tab_b1,
    "exp_sensitivity":generate_tab_b4,
    "exp_irisnp":     generate_tab_b5_b6,
    # ...
}
```

**重要约束**:
- `doc/generated/` 下的所有 `.tex` 文件由此脚本生成，**禁止手改**
- draft 模式：用 `??` 替换来自 quick 结果（仅 3 trials）的统计量
- 完整模式：仅接受 full 结果（20 trials），否则报错

---

## 5. CI 配置更新（D3 实施）

### 5.1 `--quick` CI job

在 `cpp/v7/.github/workflows/ci.yml` 中添加：

```yaml
  experiment-quick:
    runs-on: ubuntu-22.04
    timeout-minutes: 8   # 5分钟实验 + 3分钟余量
    needs: quick-build-test
    steps:
      - name: Run Quick Experiments
        run: |
          cd cpp/v7
          for script in scripts/run_exp_*.py; do
            echo "Running: $script --quick"
            python "$script" --quick --output experiments/results_quick/
          done
      - name: Verify Quick Results
        run: |
          python cpp/v7/scripts/verify_quick_results.py \
            --results cpp/v7/experiments/results_quick/ \
            --min-sr 1.0  # SR=100% 必须满足
```

### 5.2 夜间全量 job（D3：夜间机器跑全量）

```yaml
name: v7-nightly

on:
  schedule:
    - cron: '0 2 * * *'   # 每天凌晨 2 点（UTC）

jobs:
  full-experiments:
    runs-on: [self-hosted, nightly-machine]
    timeout-minutes: 480  # 8 小时
    steps:
      - name: Run Full Experiments
        run: |
          cd cpp/v7
          for script in scripts/run_exp_*.py; do
            python "$script" --output experiments/results_nightly/
          done
      - name: Build LaTeX Tables
        run: |
          cd cpp/v7
          python scripts/build_tables.py \
            --source experiments/results_nightly/ \
            --output doc/generated/
      - name: Upload Results
        uses: actions/upload-artifact@v4
        with:
          name: nightly-results-${{ github.run_number }}
          path: cpp/v7/experiments/results_nightly/
```

---

## 6. verify_quick_results.py

CI 验证脚本（保证 `--quick` 结果满足最低要求）：

```python
#!/usr/bin/env python3
"""验证 --quick 实验结果满足 CI 门槛"""

import json
import argparse
from pathlib import Path
import sys

CI_THRESHOLDS = {
    "sr":           1.0,    # 成功率必须 100%
    "max_build_s":  30.0,   # 最大构建时间（秒）
    "max_opt_s":    10.0,   # 最大路径优化时间（秒）
}

def verify(results_dir: Path):
    ok = True
    for json_file in results_dir.glob("*.json"):
        data = json.loads(json_file.read_text())
        sr = data.get("success_rate", 0)
        if sr < CI_THRESHOLDS["sr"]:
            print(f"[FAIL] {json_file.name}: SR={sr:.2f} < {CI_THRESHOLDS['sr']}")
            ok = False
    return ok

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--results", required=True)
    parser.add_argument("--min-sr", type=float, default=1.0)
    args = parser.parse_args()

    if not verify(Path(args.results)):
        sys.exit(1)
    print("[OK] 所有 quick 实验结果满足 CI 门槛")
```

---

## 7. 场景配置文件整理

v6 的 configs 从多个位置（`v6/configs/`, `v3/scripts/`, `v4/experiments/`）读取。
v7 统一到：

```
experiments/configs/
  iiwa14_scene1.yaml     # IIWA14: bookshelf
  iiwa14_scene2.yaml     # IIWA14: narrow passage
  iiwa14_scene3.yaml     # IIWA14: cluttered
  iiwa14_scene4.yaml     # IIWA14: open
  panda_scene1.yaml      # Panda: bookshelf
  robots/
    iiwa14.urdf          # 机器人 URDF（软链接到 Drake 安装路径）
    panda.urdf
```

**迁移任务**：
- 读取 v6 的 4 个 IIWA14 场景配置，转换为 v7 YAML 格式
- 读取 v6 的 Panda 场景配置，转换
- 验证：`Scene::load_yaml(path)` 能正确加载所有 5 个场景，障碍物数量一致

---

## 8. 孤儿脚本归档

以下 v6 脚本归档到 `experiments/archive/`（不迁移，不删除）：

| 文件 | 归档原因 |
|------|---------|
| `_fix_*.py` (v3-v6) | 临时修复脚本，无对应论文实验 |
| `_test_*.py` (v3-v4) | 早期探索，被正式实验替代 |
| `_diagnose_*.py` | 调试辅助，无实验价值 |
| `_show_*.py` | 可视化快照脚本，功能已集成 |
| `_check_*.py` | 单次验证脚本，无迭代价值 |

**归档命令**:
```bash
# 先确认文件列表再执行
mkdir -p cpp/v7/experiments/archive
cp cpp/v6/*.py cpp/v7/experiments/archive/
# 注意：是 cp，不是 mv（v6 保持不变）
```

---

## 9. 迁移任务列表

### Task 6.1 — 整理 11 个脚本的实验设计

阅读 v6 对应脚本，明确每个实验的：
- 输入参数
- 输出 JSON 字段（与 `build_tables.py` 对接）
- 回归基准（与 v6 结果的比较点）

### Task 6.2 — 实现 `run_exp_main.py`

- `--quick` 旗标（3 trials, seed 0-2）
- 全量（20 trials, seed 0-19）
- 输出字段：`success_rate`, `build_time_mean`, `path_length_mean`, `path_length_std`

### Task 6.3 — 实现其余 10 个脚本

（按优先级：ablation → crossrobot → sensitivity → irisnp → 其余对比实验）

### Task 6.4 — 实现 `build_tables.py`

- 读取各实验 JSON → 生成 `doc/generated/*.tex`
- 与 v6 的 `build_b1_b3_b5_tables.py` 对比，确保迁移的数值完全一致

### Task 6.5 — 配置 CI yaml

- quick job（≤8 分钟）
- nightly job（≤8 小时）
- `verify_quick_results.py` 集成

### Task 6.6 — 场景配置文件迁移

- 5 个 YAML 场景文件
- 单元测试：`Scene::load_yaml` 加载所有 5 个，障碍物数量 == v6

---

## 10. 测试矩阵

| 测试名 | 内容 | 超时 |
|--------|------|------|
| `smoke_P6_script_imports` | 11 个脚本可导入，无报错 | 10s |
| `smoke_P6_quick_flag` | 每个脚本 --quick，≤30s 完成，生成 JSON | 5min |
| `smoke_P6_scene_load` | 5 个场景 YAML 加载，障碍物数匹配 | 5s |
| `smoke_P6_build_tables` | build_tables.py 从 quick 结果生成 tex 文件 | 10s |
| `unit_quick_sr` | quick 模式 SR=100%（IIWA14 3 seeds） | 2min |

---

## 11. Definition of Done

- [ ] 11 个标准实验脚本就位，每个均有 `--quick` 测试通过
- [ ] `smoke_P6_*` 全部通过
- [ ] `make run_quick_all` ≤5 分钟（11 个脚本全部 `--quick`）
- [ ] 场景 YAML 完整（5 个），`Scene::load_yaml` 全部通过
- [ ] CI yaml 更新（quick job + nightly job）
- [ ] 孤儿脚本已归档到 `experiments/archive/`（非删除）
- [ ] `doc/generated/` 下无手改文件（全部由 `build_tables.py` 生成）
- [ ] 已更新 `/memories/session/plan.md` P6 打勾

---

*Phase: P6 | 依赖: P5 | 解锁: P7*
