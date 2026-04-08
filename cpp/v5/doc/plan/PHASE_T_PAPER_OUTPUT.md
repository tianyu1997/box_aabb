# Phase T: 论文输出生成

> 依赖: Phase S (5 组核心实验数据)
> 状态: 🔲 未开始
> 产出: LaTeX 表格 (.tex) + 矢量图 (.pdf) + 统计分析 + 一键复现脚本
> 预计: ~250 LOC (Python 输出脚本 + report 扩展)

---

## 目标

将 Phase S 产出的 JSON 实验数据转化为可直接插入 T-RO 论文的:
1. **4 张 LaTeX 表格** (booktabs 格式) 
2. **4 张论文配图** (PDF 矢量 + PNG 位图)
3. **统计显著性标注** (Wilcoxon signed-rank)
4. **一键复现脚本** (从零到全部输出)

---

## Step T1: LaTeX 表格生成

### 文件
修改: `python/sbf5_bench/report.py`
新建: `python/scripts/gen_tables.py`

### Table 1: Envelope Tightness (S1 数据)

```latex
\begin{table}[t]
\centering
\caption{Envelope volume comparison across 12 pipeline configurations.
$V$ = mean envelope volume (m³ for 2DOF, m⁶ for 7DOF).
$R$ = compression ratio vs IFK-LinkIAABB baseline.}
\label{tab:envelope_volume}
\begin{tabular}{llccc}
\toprule
Endpoint & Envelope & $V$ (7DOF) & $R$ (\%) & Safe? \\
\midrule
IFK        & LinkIAABB      & 12.5 & 100.0 & \cmark \\
IFK        & LinkIAABB\_Grid & 10.2 & 81.6  & \cmark \\
IFK        & Hull16\_Grid   & 8.7  & 69.6  & \cmark \\
CritSample & LinkIAABB      & 11.8 & 94.4  & \xmark \\
...        & ...            & ...  & ...   & ...    \\
GCPC       & Hull16\_Grid   & \textbf{7.1} & \textbf{56.8} & \cmark \\
\bottomrule
\end{tabular}
\end{table}
```

### Table 2: Computation Timing (S2 数据)

```latex
\begin{table}[t]
\centering
\caption{Envelope computation time (μs) per box.}
\label{tab:timing}
\begin{tabular}{llrrrr}
\toprule
Endpoint & Envelope & EP (μs) & Env (μs) & Total (μs) & Speedup \\
\midrule
IFK        & LinkIAABB    & 15 & 2  & \textbf{17}  & 35× \\
Analytical & LinkIAABB    & 580 & 2  & 582 & 1.0× \\
GCPC       & LinkIAABB\_Grid & 45 & 30 & 75 & 7.8× \\
...        & ...          & ... & ... & ... & ... \\
\bottomrule
\end{tabular}
\end{table}
```

### Table 3: End-to-End Planning (S3 数据)

```latex
\begin{table*}[t]
\centering
\caption{Planning performance: 12 configurations × 6 scenes (30 seeds each).
Bold = best per scene; gray = worst.}
\label{tab:e2e}
\begin{tabular}{ll cccc cccc}
\toprule
& & \multicolumn{4}{c}{2DOF Scenes} & \multicolumn{4}{c}{7DOF Scenes} \\
\cmidrule(lr){3-6} \cmidrule(lr){7-10}
Endpoint & Envelope & simple & narrow & clutter & \textit{avg} 
         & tabletop & shelf & multi & \textit{avg} \\
\midrule
\multicolumn{10}{c}{\textit{Success Rate (\%)}} \\
\midrule
IFK & LinkIAABB & 100 & 100 & 100 & 100 & 83 & 47 & 73 & 68 \\
... & ... & ... & ... & ... & ... & ... & ... & ... & ... \\
\midrule
\multicolumn{10}{c}{\textit{Planning Time (s)}} \\
\midrule
... \\
\bottomrule
\end{tabular}
\end{table*}
```

### Table 4: Baseline Comparison (S4 数据)

```latex
\begin{table}[t]
\centering
\caption{SBF vs sampling-based planners on 4 representative scenes.}
\label{tab:baselines}
\begin{tabular}{lcccc}
\toprule
Planner & Success\% & Time (s) & PathLen & Smooth \\
\midrule
\textbf{SBF-GCPC-Grid} & \textbf{82} & 1.2 & 5.8 & \textbf{0.15} \\
RRTConnect & 95 & \textbf{0.8} & 8.2 & 0.85 \\
RRT* & 88 & 5.2 & \textbf{4.9} & 0.42 \\
BIT* & 90 & 8.1 & 5.1 & 0.38 \\
\bottomrule
\end{tabular}
\end{table}
```

### report.py 扩展

```python
def envelope_volume_table(s1_data: dict) -> str:
    """Generate Table 1 LaTeX: envelope volume comparison."""
    ...

def timing_table(s2_data: dict) -> str:
    """Generate Table 2 LaTeX: computation timing."""
    ...

def e2e_table(s3_results: ExperimentResults) -> str:
    """Generate Table 3 LaTeX: end-to-end planning (double-column)."""
    ...

def baseline_table(s4_results: ExperimentResults) -> str:
    """Generate Table 4 LaTeX: baseline comparison."""
    ...
```

### 格式要求
- `\usepackage{booktabs}` 
- 最优值 `\textbf{}`
- 最差值 `\color{gray}`
- 数值精度: 时间 2位, 百分比 0位, volume 1位
- 统计显著性用 `$^*$` 标注 (见 T3)

---

## Step T2: 论文配图

### 依赖
```bash
pip install kaleido plotly
```

### 文件
新建: `python/scripts/gen_figures.py`

### Figure 1: Envelope Timing Bar Chart (S2)

```python
import plotly.graph_objects as go

def fig_envelope_timing(s2_data: dict) -> go.Figure:
    """Grouped bar: endpoint sources × envelope types, y = total time (μs)."""
    # 4 groups (EP source), 3 bars each (Env type)
    # Color scheme: LinkIAABB=blue, LinkIAABB_Grid=orange, Hull16_Grid=green
    ...
```

- **Size**: single-column (3.5in × 2.5in)
- **Font**: 10pt, matching LaTeX
- **Export**: PDF (vector) + PNG (300 DPI)

### Figure 2: E2E Planning Heatmap (S3)

```python
def fig_e2e_heatmap(s3_results: ExperimentResults) -> go.Figure:
    """Heatmap: rows = 12 configs, cols = 6 scenes.
    Color = planning_time_s (log scale).
    Annotation = success_rate%."""
    ...
```

- **Size**: double-column (7in × 3in)
- **Color**: sequential (viridis), log scale
- **Annotation**: success rate in each cell

### Figure 3: Baseline Pareto Plot (S4)

```python
def fig_baseline_pareto(s4_results: ExperimentResults) -> go.Figure:
    """Scatter: x = planning_time, y = path_quality.
    Each point = one planner on one scene (mean ± std error bars)."""
    ...
```

- **Size**: single-column (3.5in × 3in)
- **Markers**: different shapes per planner, colors per scene
- **Error bars**: std across seeds

### Figure 4: Scalability Line Plots (S5)

```python
def fig_scalability(s5_data: dict) -> go.Figure:
    """3 subplots:
    (a) time vs DOF
    (b) success_rate vs obstacle_count
    (c) success_rate vs max_boxes"""
    from plotly.subplots import make_subplots
    fig = make_subplots(rows=1, cols=3,
                        subplot_titles=["(a) DOF", "(b) Obstacles", "(c) Budget"])
    ...
```

- **Size**: double-column (7in × 2.5in)
- **Error bands**: mean ± std shaded area
- **Line styles**: solid/dashed for different metrics

### 配色方案 (统一)

```python
COLORS = {
    "SBF-GCPC-Grid":     "#1f77b4",  # blue
    "SBF-Analytical-Box": "#ff7f0e", # orange
    "SBF-IFK-Box":       "#2ca02c",  # green
    "RRTConnect":        "#d62728",  # red
    "RRT*":              "#9467bd",  # purple
    "BIT*":              "#8c564b",  # brown
}

ENVELOPE_COLORS = {
    "LinkIAABB":      "#4c78a8",
    "LinkIAABB_Grid": "#f58518",
    "Hull16_Grid":    "#54a24b",
}
```

### 输出

```
results/paper/figures/
    ├── fig1_envelope_timing.pdf
    ├── fig1_envelope_timing.png
    ├── fig2_e2e_heatmap.pdf
    ├── fig2_e2e_heatmap.png
    ├── fig3_baseline_pareto.pdf
    ├── fig3_baseline_pareto.png
    ├── fig4_scalability.pdf
    └── fig4_scalability.png
```

---

## Step T3: 统计显著性分析

### 文件
新建: `python/sbf5_bench/stats.py`

### 方法

```python
from scipy.stats import wilcoxon

def pairwise_significance(
    results: ExperimentResults,
    metric: str = "planning_time_s",
    alpha: float = 0.05,
) -> dict:
    """Wilcoxon signed-rank test between all planner pairs.
    
    Returns dict[(planner_a, planner_b)] -> {
        "statistic": W,
        "p_value": p,
        "significant": p < alpha,
        "effect_direction": "a < b" | "a > b" | "ns"
    }
    """
    ...

def annotate_table_significance(
    latex_str: str,
    sig_results: dict,
    reference_planner: str = "SBF-GCPC-LinkIAABB_Grid",
) -> str:
    """Add $^*$ markers to LaTeX table for statistically significant
    improvements over reference planner."""
    ...
```

### 应用位置
- Table 3: 标注相对 SBF-GCPC-Grid 显著差异的 configs
- Table 4: 标注相对 RRTConnect 显著差异的 planners

---

## Step T4: 一键复现脚本

### 文件
新建: `python/scripts/run_all_experiments.py`

### 脚本结构

```python
#!/usr/bin/env python3
"""
Run all experiments and generate all paper outputs.

Usage:
    python python/scripts/run_all_experiments.py [--skip-experiments] [--skip-figures]

Outputs → results/paper/
"""
import argparse
import logging
import sys
import os

logging.basicConfig(level=logging.INFO, format="%(asctime)s %(message)s")
logger = logging.getLogger(__name__)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--skip-experiments", action="store_true",
                        help="Only regenerate tables/figures from existing data")
    parser.add_argument("--skip-figures", action="store_true",
                        help="Only run experiments, skip output generation")
    parser.add_argument("--quick", action="store_true",
                        help="Quick mode: 3 seeds, 2 scenes (for testing)")
    args = parser.parse_args()

    os.makedirs("results/paper", exist_ok=True)

    if not args.skip_experiments:
        logger.info("═══ Phase S1: Envelope Tightness ═══")
        run_s1(quick=args.quick)

        logger.info("═══ Phase S2: Envelope Timing ═══")
        run_s2(quick=args.quick)

        logger.info("═══ Phase S3: End-to-End Benchmark ═══")
        run_s3(quick=args.quick)

        logger.info("═══ Phase S4: Baselines ═══")
        run_s4(quick=args.quick)

        logger.info("═══ Phase S5: Scalability ═══")
        run_s5(quick=args.quick)

    if not args.skip_figures:
        logger.info("═══ Phase T: Generating Tables & Figures ═══")
        gen_tables()
        gen_figures()
        gen_stats()

    logger.info("═══ ALL DONE ═══")
    logger.info("Tables  → results/paper/tables/")
    logger.info("Figures → results/paper/figures/")
    logger.info("Data    → results/paper/s{1-5}_*/")

if __name__ == "__main__":
    main()
```

### 功能
- `--quick` 模式: 3 seeds × 2 scenes × 3 configs (用于 CI 或调试)
- `--skip-experiments`: 只从已有 JSON 生成输出 (调整格式时无需重跑)
- `--skip-figures`: 只跑实验不生成输出
- Checkpoint/resume: S3 中断后可直接重跑, 自动从 checkpoint 续跑
- 所有输出在 `results/paper/` 下, 可直接 `\input{}` 到 LaTeX

### 运行命令

```powershell
# 完整运行 (约 2-3 小时)
cd v5
$env:PYTHONPATH = "build_x64/Release;python"
python python/scripts/run_all_experiments.py

# 快速验证 (约 5 分钟)
python python/scripts/run_all_experiments.py --quick

# 只重新生成图表 (几秒)
python python/scripts/run_all_experiments.py --skip-experiments
```

---

## Step T5: 论文结果目录结构

### 最终产出

```
results/paper/
├── tables/
│   ├── tab1_envelope_volume.tex
│   ├── tab2_computation_timing.tex
│   ├── tab3_e2e_planning.tex
│   └── tab4_baselines.tex
├── figures/
│   ├── fig1_envelope_timing.pdf
│   ├── fig1_envelope_timing.png
│   ├── fig2_e2e_heatmap.pdf
│   ├── fig2_e2e_heatmap.png
│   ├── fig3_baseline_pareto.pdf
│   ├── fig3_baseline_pareto.png
│   ├── fig4_scalability.pdf
│   └── fig4_scalability.png
├── stats/
│   ├── pairwise_s3.json
│   └── pairwise_s4.json
├── s1_envelope_tightness/
│   └── *.json
├── s2_envelope_timing/
│   └── *.json
├── s3_e2e/
│   └── *.json
├── s4_baselines/
│   └── *.json
└── s5_scalability/
    └── *.json
```

### LaTeX 插入方式

```latex
% main.tex
\input{results/paper/tables/tab1_envelope_volume.tex}
\input{results/paper/tables/tab2_computation_timing.tex}
\input{results/paper/tables/tab3_e2e_planning.tex}
\input{results/paper/tables/tab4_baselines.tex}

\begin{figure}[t]
\centering
\includegraphics[width=\columnwidth]{results/paper/figures/fig1_envelope_timing.pdf}
\caption{Envelope computation time breakdown.}
\end{figure}
```

---

## 测试

| 用例 | 文件 | 描述 |
|------|------|------|
| `test_table_generation` | `python/tests/test_report.py` (追加) | 用 mock data 生成 4 张表, 验证 `\begin{table}` 存在 |
| `test_figure_generation` | `python/tests/test_report.py` (追加) | 用 mock data 生成 4 张 figure, 验证 `go.Figure` 有 traces |
| `test_stats` | `python/tests/test_report.py` (追加) | Wilcoxon test 返回 p-value |
| `test_quick_mode` | `python/tests/test_experiments.py` (追加) | `--quick` 运行全流程无异常 |

---

## 验收标准

- [ ] 4 张 `.tex` 表格可被 `pdflatex` 编译
- [ ] 4 张 `.pdf` 图清晰可读 (单列 3.5in / 双列 7in)
- [ ] 统计显著性: 至少 Table 3 中有 `$^*$` 标注
- [ ] `run_all_experiments.py --quick` 在 5 分钟内跑完
- [ ] `run_all_experiments.py --skip-experiments` 在 10 秒内跑完
- [ ] 所有 `.pdf` 是矢量图 (可 zoom 无锯齿)
- [ ] `.png` 分辨率 ≥ 300 DPI
- [ ] JSON → LaTeX → PDF 全链路无断裂

---

## 文件改动清单

| 文件 | 类型 | 改动量 |
|------|------|--------|
| `python/sbf5_bench/report.py` | 修改 | +80 LOC (4 个 table 函数) |
| `python/sbf5_bench/stats.py` | 新建 | ~60 LOC |
| `python/scripts/gen_tables.py` | 新建 | ~40 LOC |
| `python/scripts/gen_figures.py` | 新建 | ~100 LOC (4 个 figure 函数) |
| `python/scripts/run_all_experiments.py` | 新建 | ~80 LOC |
| `python/tests/test_report.py` | 修改 | +30 LOC |
| **总计** | | **~390 LOC** |
