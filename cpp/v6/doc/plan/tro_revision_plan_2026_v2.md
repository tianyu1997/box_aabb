# SafeBoxForest v6 — TRO 大修执行计划 (Plan v2, 2026-04-24)

## 目标
按 TRO 审稿标准对 `cpp/v6/doc/box_aabb_v6_paper_en.tex` 与同步对应的
`box_aabb_v6_paper_zh.tex` 完成大修。本计划取代 `tro_revision_plan_2026.md`，
是当前会话的唯一权威。

## 主统计协议（Master Protocol，全文一律遵守）
- **种子数**：N = 10 seeds（cold cache，每 seed 独立清空 `~/.sbf_cache/`）
- **统计量**：median + IQR (P25–P75)；附 min–max 作为补充
- **并行**：16 worker threads（主协议）；scalability 表里另列 1/4/8
  对照
- **硬件 / 软件**：i5-12600KF / Ubuntu 22.04 / GCC 11.4 / Drake 1.31 / MOSEK
- **碰撞后端**：所有基线（SBF / IRIS-NP / RRT-Connect / PRM）统一 Drake
  `SceneGraphCollisionChecker`
- **正文报告口径**：median；任何与 median 偏差 > 5 % 的均值数字必须
  以脚注形式给出
- **数字单一来源**：所有可被实验生成的数字一律 `\input{generated/...}`，
  正文不再出现手写数字

数据来源 binary（既有，无需重写）：
- `cpp/build/experiments/exp6_build_timing` (build / per-query, C++ + LECT)
- `cpp/build/experiments/exp5_ablation` (ablation 表)
- `cpp/v6/scripts/run_baselines.py` (PRM / RRT-C / IRIS-NP+GCS via Drake)
- `cpp/v6/scripts/gcs_pipeline.py` (SBF→GCS post-opt)

---

## Phase 1 — 数字与术语止血（无新实验）

只做对齐与术语统一，不动任何 binary。

### P1.1 术语统一
- `Link Envelope Collision Tree` → `Lifelong Envelope Cache Tree`
  - 文件位置：`box_aabb_v6_paper_en.tex` §LECT 标题；`zh.tex` 同步
- 全文检查 LECT、epiAABB、LinkIAABB、Hull16-Grid 大小写

### P1.2 修正交叉数字冲突（按对应表数字回写文字）
| 项 | 现状 | 处理 |
|---|---|---|
| CS→LB 时间 | 表 0.701 vs 图注 0.654 | 图注以表为准 |
| 平均查询 | 0.259 / 0.256 / 0.251 三处 | 全文统一为 `tab:query` 中的 0.259 |
| Build mean | 摘要 1.32 / 表 1.51 / 文字 1.38 | 全文用 `tab:build` median，目前 1.32 |
| Boxes ~3,468 | abstract / ablation | 与 build 表对齐 |
| 消融 WidestFirst 矛盾 | 文字 1.38 vs 表 1.52 | 改文字以匹配表 |

### P1.3 参考文献清理
- 删除重复的第二个 `ericson2004real` (line 2810)

### P1.4 Conclusion / Abstract 术语口径
- “matching or better path quality” 改为
  “raw paths $17.4\%$ longer than IRIS-NP+GCS in $L_2$;
  $15\%$ shorter once SBF→GCS post-optimisation is enabled”

**交付**：仅文字 diff（< 100 行），无须重新跑实验

---

## Phase 2 — 主协议落地：10 seeds 数据重生成

### P2.1 重跑 build / query (exp6_build_timing)
```
cd cpp/build
rm -rf ~/.sbf_cache/*
./experiments/exp6_build_timing \
   --envelope hull16_grid --endpoint ifk --lect-cache \
   --seeds 10 --threads 16 \
   --json /home/tian/桌面/box_aabb/cpp/v6/experiments/results_new/exp6_10seed_16thr.json
```
覆盖既有 `exp6_timing_10seed.json`（实际只是 5 seed 的旧文件），
并新写一份 `exp6_10seed_8thr.json`。

### P2.2 重跑 ablation (exp5_ablation)
```
./experiments/exp5_ablation --seeds 10 --threads 16 \
   --json .../results_new/exp5_10seed_16thr.json
```

### P2.3 写 IQR 计算脚本
新建 `cpp/v6/scripts/regen_paper_tables.py`：
- 读取 `exp6_10seed_16thr.json`、`exp5_10seed_16thr.json`、
  `online_query_comparison.json`
- 计算 median / P25 / P75 / min / max
- 直接生成：
  - `generated/tab_build.tex`
  - `generated/tab_query.tex`
  - `generated/tab_ablation.tex`
- 同步刷新 `generated/canonical_numbers.csv`

### P2.4 正文改用 `\input{...}`
将 `tab:build`、`tab:query`、`tab:ablation` 由手写改为
`\input{generated/tab_build.tex}` 等。

**交付**：3 张主表完全自动化、IQR 列出现在所有时间数字

---

## Phase 3 — 基线公平性

### P3.1 重跑 baselines（10 seeds，统一 16 threads）
```
python3 cpp/v6/scripts/run_baselines.py --seeds 10 \
   --json cpp/v6/experiments/results_new/baselines_10seed_16thr.json
```

### P3.2 新增「等预算基线表」`tab:baselines_isobudget`
列：方法 × 预算档（5 s / 30 s / 130 s 三档 wall-clock）
内容：每档下的 SR / median query / path

数据来源：
- SBF：用 `--max-time` 控制 build；从 `exp6_build_timing` 加新 flag
- IRIS-NP：限制 region 数量 (n=2 / 8 / 16)
- PRM：限制 N (1k / 10k / 50k)；用既有 `B2_prm_sweep.json` 扩展
- RRT-Connect：固定 0 s precompute（无预算依赖）

由 `regen_paper_tables.py` 生成 `generated/tab_baselines_isobudget.tex`

### P3.3 PRM sweep 升级
```
python3 cpp/v6/scripts/run_baselines.py --seeds 10 --prm-only \
   --prm-nodes 1000,3000,10000,30000,50000 \
   --json cpp/v6/experiments/results_new/prm_sweep_10seed.json
```
覆盖现有 `tab_prm_sweep.tex`。

### P3.4 重写 §exp_baselines 与 Discussion 中的“98×”叙事
- 弱化“98×”单数字，写成“two orders of magnitude under matched
  precompute budgets (Table N)”
- 明确二阶段 path quality：raw +17.4 %，post-opt –15 %

**交付**：1 张新主表 + 现有 `tab_baselines_v2` 升级 IQR + 文字重写

---

## Phase 4 — 理论表述与限制扩展

### P4.1 M1 表头与说明改写
- `False-Acc` → `Sample-witness violations (must be 0 by Thm.~\ref{thm:certify})`
- `False-Rej / FR%` → `Sample-disagreement rate (over-approx proxy,
  not a formal type-II error)`
- 在 `tab:m1_erosion` 之后追加一句免责声明

### P4.2 Limitations 扩展（4-obstacle failure mode）
- 解释为何 axis-aligned 编码下 4-obs 窄通道是已知失败模式
- 给出 fallback 思路：(i) OBB / 旋转 box；(ii) 与 IRIS-NP 混合

### P4.3 Reproducibility 段强化
新增列表：
- 提交 SHA、Drake 版本、MOSEK 版本、CPU 型号
- 每张表对应的 binary + flags + JSON 文件名
- “Code & cache release upon acceptance”

### P4.4 写作小修
- Abstract 中 “$98\times$” 数字下沉到表注
- Conclusion 段同步两阶段 path-quality

**交付**：仅文字 diff

---

## Phase 5 — 中文版同步与一致性校验

### P5.1 `box_aabb_v6_paper_zh.tex` 同步
按 P1–P4 逐段对中文版应用同样改动；术语对照：
- LECT → 终身包络缓存树（保留缩写 LECT）
- iso-budget baseline → 等预算基线
- 二阶段叙事翻译保持一致

### P5.2 `scripts/check_paper_consistency.py` 跑通
确保所有 `\input{generated/...}` 与 CSV 的 single source of truth 对得上。

### P5.3 `scripts/diff_en_zh.py` 跑通
中英两版同结构。

---

## 里程碑与提交节点

| 节点 | 交付物 |
|---|---|
| M1 | Phase 1 完成；术语 + 数字一致 |
| M2 | Phase 2 完成；3 张主表自动化、IQR 上线 |
| M3 | Phase 3 完成；新基线表 + 重写讨论 |
| M4 | Phase 4 完成；理论与限制扩写 |
| M5 | Phase 5 完成；中英同步、一致性脚本通过 |

---

## 风险与对策
- **IRIS-NP 单 region 210 s 太慢**：只跑 (2 / 8 / 16) region 三档，
  其余 IQR 由现有 cache 内插
- **MOSEK license 限制**：所有 GCS 实验在单机串行跑
- **磁盘缓存膨胀**：每个 Phase 跑前 `rm -rf ~/.sbf_cache/*`
