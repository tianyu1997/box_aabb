# P7 — 论文最终版与开源发布

> **目标**: 准备 TRO 投稿版本论文和代码开源版本。
> 清理论文中所有版本号引用，更新所有实验数据（来自 P6 夜间全量结果），
> 编写开源 README，配置 LICENSE 和 code hygiene。
>
> **预计工时**: 3-4 天  
> **前置条件**: P6 完成（夜间全量实验已跑完，所有 JSON 结果就绪）  
> **完成标准**: 论文 pdflatex/xelatex 编译零 warning；代码可从 clean git clone 构建

---

## 1. 论文处理原则

> 重申用户记忆原则：
> - 论文文件：`cpp/v7/doc/box_aabb_v7_paper_en.tex` 和 `box_aabb_v7_paper_zh.tex`
> - 中英文版同步更新
> - 所有数值来自 `doc/generated/*.tex`（机器生成），禁止手改

---

## 2. 清理清单：论文中要删除的内容

### 2.1 版本号引用（"v6" / "v7" / "version" 字样）

```bash
# 检查
grep -n "v6\|v7\|version [0-9]" cpp/v7/doc/box_aabb_v7_paper_en.tex
```

所有算法描述中的版本号引用全部删除，替换为描述性名称：
- "v6 LECT" → "LECT"
- "v7 architecture" → （删除，不提版本）
- "compared to v5" → （删除或改为"prior work [ref]"）

### 2.2 调试注释和 TODO

```bash
grep -n "TODO\|FIXME\|DEBUG\|HACK\|XXX" \
    cpp/v7/doc/box_aabb_v7_paper_en.tex \
    cpp/v7/doc/box_aabb_v7_paper_zh.tex
```

全部删除或处理。

### 2.3 占位符

- `??` → 必须全部替换为夜间实验实际数值
- `\textbf{[TODO}` → 处理
- `% PLACEHOLDER` → 删除

---

## 3. 论文数值更新流程

```
P6 夜间全量实验 (seed 0-19)
  → experiments/results_nightly/*.json
  → scripts/build_tables.py --source results_nightly/ --output doc/generated/
  → doc/generated/tab_*.tex (自动生成)
  → paper_en.tex 和 paper_zh.tex 中 \input{} 引用
```

**非自动更新的部分**（需手工更新）：
1. Abstract 中的摘要性数字（SR, build_time, path_quality 的代表值）
2. Introduction 中的定性描述
3. Conclusion 中的结论句

**手工更新规则**：
- 数值必须与 `results_nightly/` 中的统计量一致（可交叉验证）
- 有效数字：SR 保留 1 位小数（如 "100.0%"），时间保留 2 位小数（如 "1.38s"）
- 中英文版必须同步（同一天更新）

---

## 4. 论文编译验证

### 4.1 EN 版（pdflatex）

```bash
cd cpp/v7/doc
pdflatex -interaction=nonstopmode box_aabb_v7_paper_en.tex
bibtex box_aabb_v7_paper_en
pdflatex -interaction=nonstopmode box_aabb_v7_paper_en.tex
pdflatex -interaction=nonstopmode box_aabb_v7_paper_en.tex

# 检查 warning
grep -i "warning\|undefined\|multiply" box_aabb_v7_paper_en.log | grep -v "^$"
# 目标：0 warning（新引入的，v6 已有的 \Cspace warning 除外）
```

### 4.2 ZH 版（xelatex）

```bash
cd cpp/v7/doc
xelatex -interaction=nonstopmode box_aabb_v7_paper_zh.tex
bibtex box_aabb_v7_paper_zh
xelatex -interaction=nonstopmode box_aabb_v7_paper_zh.tex
xelatex -interaction=nonstopmode box_aabb_v7_paper_zh.tex

grep -i "warning\|undefined\|未定义" box_aabb_v7_paper_zh.log | grep -v "^$"
```

### 4.3 页数验证

| 指标 | EN | ZH |
|------|----|----|
| 最小页数 | 26 pp | 28 pp |
| 最大页数 | 30 pp | 34 pp |
| 图表标签数 | ≥ 22 | ≥ 19 |

---

## 5. 开源代码清理

### 5.1 需要从开源版本删除的文件

```bash
# 删除开发过程文件（不进公开仓库）
cpp/v7/experiments/archive/   → 不发布（内含调试脚本）
cpp/v7/doc/migration/         → 不发布（内含迁移计划文档）
cpp/v7/experiments/results_nightly/  → 不发布（大数据文件）
.github/workflows/nightly.yml → 不发布（含内部 self-hosted runner 配置）
```

**实现方式**: `.gitignore` + `release_filter.sh` 脚本（生成 release 压缩包时自动排除）。

### 5.2 README.md

创建 `cpp/v7/README.md`，必须包含：

```markdown
# SBF: Spatial Box Forest for Robot Motion Planning

## Requirements
- CMake ≥ 3.18
- Eigen3 ≥ 3.4
- Drake (see drake_version.txt for tested version)
- Python ≥ 3.9 (for experiments and paper tables)

## Quick Build
```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
```

## Running Experiments
```bash
# Quick smoke test (≤5 min)
python scripts/run_exp_main.py --quick

# Full experiment (nightly, ~2h)
python scripts/run_exp_main.py
```

## Citation
[TRO citation 占位符]
```

### 5.3 LICENSE

确认使用 MIT 或 BSD-3-Clause（与 Drake 许可证兼容）。
在 `cpp/v7/LICENSE` 中写入完整许可证文本。

### 5.4 代码卫生检查

```bash
# 检查调试 print 和 logging 遗留
grep -rn "printf\|std::cout\|std::cerr\|spdlog::debug" \
    cpp/v7/src/ --include="*.cpp" | grep -v "// debug ok"

# 检查 hardcoded path
grep -rn '"/home/\|"/root/\|"C:\\\\' cpp/v7/src/ cpp/v7/scripts/

# 检查 TODO/FIXME
grep -rn "TODO\|FIXME\|HACK\|XXX" cpp/v7/src/ cpp/v7/include/
```

全部清理后，`git diff --stat HEAD` 应该只含预期的改动。

---

## 6. 版本文件

### 6.1 `cpp/v7/VERSION`

```
7.0.0
```

### 6.2 `cpp/v7/drake_version.txt`

```
# Tested Drake version for SBF v7
drake_version = "1.x.x"  # 填入实际版本，参见 /memories/repo/drake_version_iriszo.md
```

### 6.3 `cpp/v7/CHANGELOG.md`

```markdown
# Changelog

## v7.0.0 (2026-xx-xx)
### Architecture Changes
- Radii=0 envelope cache (D1): LECT cache now stores zero-radius geometry
- Force face-overlap bridging (D2): Bridge boxes must overlap parent face area
- Strict parent-adjacency for GCS edges (fix goal bias)

### Performance
- Memory peak reduced ~60% via true lazy load (SubtreeLease)
- Build time maintained within ±10% of v6

### Breaking Changes
- Python module renamed: _sbf5_cpp → _sbf7_cpp
- LECT file format v7 (can read v6 format, write v7 only)
- CacheKey no longer includes link_radii (shared cache for all radii configs)
```

---

## 7. 开源发布检查清单

### 7.1 安全检查（OWASP 相关）

```bash
# 检查是否有密钥、token、credential 泄露
git log --all --oneline | head -20
grep -rn "password\|secret\|api_key\|token\|credential" \
    cpp/v7/ --include="*.py" --include="*.cpp" --include="*.yaml"
```

### 7.2 依赖版本锁定

```
cpp/v7/requirements.txt:
  numpy>=1.24
  scipy>=1.10
  pyyaml>=6.0
  matplotlib>=3.7  # 仅 visualization
```

### 7.3 最终构建测试（在 clean 环境中）

```bash
# 模拟 clean clone
git clone <repo> /tmp/sbf_v7_test
cd /tmp/sbf_v7_test/cpp/v7
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON
make -j4
ctest --output-on-failure
# 期望：所有 smoke_P* 测试通过
```

---

## 8. TRO 投稿准备

### 8.1 投稿材料清单

| 材料 | 来源 | 状态 |
|------|------|------|
| 主论文 PDF | `box_aabb_v7_paper_en.pdf` | P7 生成 |
| 补充材料 | `box_aabb_v7_supplementary.pdf` | P7 生成 |
| 代码压缩包 | `release_filter.sh` 生成 | P7 生成 |
| 视频（可选）| B8 实验（未来） | ❌ 不在 v7 范围 |

### 8.2 投稿前最终验证

```bash
# 验证所有图表标签已解决
grep "??" cpp/v7/doc/box_aabb_v7_paper_en.log

# 验证页面不超限（TRO 通常 ≤ 12 pages + appendix）
# 注意：pdflatex 输出的完整版可能更长，投稿版可能需要精简
```

---

## 9. 迁移任务列表

### Task 7.1 — 运行夜间全量实验（P6 完成后执行）

```bash
cd cpp/v7
for script in scripts/run_exp_*.py; do
    python "$script" --output experiments/results_nightly/
done
```

### Task 7.2 — 生成最终论文表格

```bash
python scripts/build_tables.py \
    --source experiments/results_nightly/ \
    --output doc/generated/
```

### Task 7.3 — 更新论文 Abstract 和 Introduction 中的摘要数字

- 读取 `results_nightly/exp_main.json`，提取代表性数字
- 在中英文版中同步更新

### Task 7.4 — 清理版本号引用

```bash
grep -n "v6\|v7\|version [0-9]" doc/box_aabb_v7_paper_en.tex
# 逐一处理
```

### Task 7.5 — 代码卫生清理

- `printf` / `std::cout` → 替换为 `spdlog::info` 或删除
- hardcoded path → 替换为相对路径或配置参数

### Task 7.6 — 编写 README.md

### Task 7.7 — 配置 LICENSE 和 CHANGELOG.md

### Task 7.8 — Clean build 验证（见 7.3 节）

---

## 10. 测试矩阵

| 测试名 | 内容 | 超时 |
|--------|------|------|
| `smoke_P7_paper_en` | pdflatex 编译，0 new warning | 30s |
| `smoke_P7_paper_zh` | xelatex 编译，0 new warning | 30s |
| `smoke_P7_no_version_refs` | 论文中无 v6/v7 字样 | 5s |
| `smoke_P7_no_placeholder` | 论文中无 ?? 占位符 | 5s |
| `smoke_P7_clean_build` | clean mkdir + cmake + make + ctest | 5min |
| `smoke_P7_code_hygiene` | 无 hardcoded path，无裸 printf | 10s |

---

## 11. Definition of Done

- [ ] EN/ZH 论文编译零 warning（新引入的）
- [ ] 论文中无 `??` 占位符，无版本号引用
- [ ] 所有实验数值来自 `results_nightly/`（20 trials）
- [ ] README.md 完整，可从 zero 构建
- [ ] LICENSE 文件就位
- [ ] CHANGELOG.md 记录 v7 的 breaking changes
- [ ] clean clone + build + ctest 全部通过
- [ ] `release_filter.sh` 生成的压缩包不含敏感文件
- [ ] 已更新 `/memories/session/plan.md` P7 打勾
- [ ] 已创建 `/memories/repo/v7_module_status.md`（记录各模块完成状态）

---

*Phase: P7 | 依赖: P6 | 解锁: TRO 投稿 + 开源发布*
