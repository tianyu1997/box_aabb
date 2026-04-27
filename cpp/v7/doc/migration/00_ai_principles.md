# v7 迁移 — AI 工作原则手册

> **范围**: 本手册约束 AI（GitHub Copilot / Claude）在 v7 迁移全程的行为。
> 每次开始新任务前必须重读本文件。

---

## A. 工程纪律

### A1 — "读后再改"强制原则
- **任何**编辑操作前，必须先读取目标文件 **≥40 行上下文**。
- 若文件超过 200 行，先 grep 定位目标位置，再读取 ±20 行片段。
- 禁止"猜测文件内容后直接写入"。

### A2 — 最小变更原则
- 每次 `replace_string_in_file` 仅做一处改动。
- 不允许"顺手"修复无关代码风格、注释或格式。
- 如发现额外改进点，记录到 `TODO.md` 留待下次迭代，不要在当前任务中实施。

### A3 — 绝不伪造数据
- 所有数值（实验结果、性能指标、LOC）必须来自实际文件读取或实际命令输出。
- 禁止"根据趋势估算"填写表格。
- 若数据缺失，用 `??` 占位并记录 issue，不允许填写看似合理的假值。

### A4 — 工具调用诚信
- 工具返回 `error` 时，禁止假设工具已成功执行后继续。
- 相同工具调用失败两次后，换策略（不重试第三次相同调用）。

---

## B. 架构决策遵从

> **这三条决策不再讨论，直接执行。**

### B1 — Radii=0 Envelope Cache（决策 D1）

**是什么**: LECT 缓存的 EndpointEnvelope 以 `link_radii=0` 计算（纯扫掠几何），
碰撞检测时在线膨胀。

**执行规则**:
```
EndpointSource::compute(q)
  → raw_aabb (link_radii all zero)
  → cache raw_aabb

CollisionChecker::check(raw_aabb, link_i_radius)
  → inflate raw_aabb_i by radius_i (Minkowski)
  → test against scene
```

- `BoxNode` 仅存储零半径 AABB；`link_radii` 字段从 `BoxNode` 删除。
- LECT `CacheKey` hash **不含** link_radii；同一构型不同机械臂半径配置共享缓存。
- 若需调整安全余量，修改调用者的 `link_radii` 参数，无需重建缓存。
- 代码中所有 `with_radii` / `inflate_envelope` 逻辑集中在 `sbf_collision.cpp`，
  其余模块不知道"半径"的存在。

### B2 — 强制 Face-Overlap 桥接（决策 D2）

**是什么**: 桥接 box 必须与父 leaf-box 有 **面重叠**（overlap，不允许仅点切/边切）。
若 RRT 采样不满足条件，细分直到满足。

**执行规则**:
- `BridgeBuilder::try_extend(parent, sample)` 返回 `REJECT` 若接触面积 `< ε_face`。
- `ε_face = 1e-3 * min(parent.side)^2`（单边最小面积的 0.1%）。
- 连续 `K=8` 次 REJECT → 触发 `subdivide_rrt(parent)` 后重试。
- 桥接成功判定：`face_overlap_area(child, parent) ≥ ε_face` **且** `interior_dist > 0`。
- 严禁降低此门槛以换取成功率。

### B3 — CI --quick + 夜间全量（决策 D3）

**是什么**: CI gate 仅跑 `--quick`（≤5 min），夜间机器跑全量实验。

**执行规则**:
- `--quick` flag：每个实验固定 `n_trials=3, seed=[0,1,2], timeout=30s`。
- CI 失败条件：`--quick` 下 SR < 100% 或 build_time > 5× baseline。
- 夜间全量：`n_trials=20, seed=[0..19]`，结果写入 `results_nightly/`。
- 实验脚本必须接受 `--quick` CLI flag，**不允许**在脚本内硬编码 quick 模式。

---

## C. 代码质量红线

### C1 — 单函数 500 行上限
- 新增或重构函数体不超过 **500 行**。
- 若迁移过程中发现 v6 大函数（`grower_coordinated.cpp:1200行`），
  **必须**在同一 Phase 内拆分，不允许原样复制。

### C2 — 禁止假 Lazy Load
- v6 的 "lazy load" 实为预分配全部坐标 + 按需标记。v7 禁止此模式。
- 真正的惰性：`load_subtree(node_id)` 仅在 `node_id` 被首次访问时触发 I/O。
- `LectLoader::is_loaded(node_id)` == false 意味着该节点数据**从未**进入内存。

### C3 — OOM 守卫必须在存入前检查
```cpp
// 正确
if (cache_.memory_used() + estimated_size > kMaxCacheBytes) {
    evict_lru();
}
cache_.insert(key, data);

// 错误（v6 风格 — 先存后检查）
cache_.insert(key, data);
if (cache_.memory_used() > kMaxCacheBytes) { evict(); }
```

### C4 — 线程安全写法
- 每个 grower 线程拥有独立 `LocalSubtreeArena`；写操作仅在本线程进行。
- 合并子树到全局树发生在 **join 阶段**，持有全局写锁（≤100ms）。
- 禁止 grower 线程持有全局写锁超过 1ms（测量值，非估计值）。

---

## D. 测试纪律

### D1 — 每个 Phase 必须有冒烟测试
- Phase 完成门槛：`ctest --output-on-failure -R smoke_P<N>` 全部通过。
- 冒烟测试必须在 CI `--quick` 环境中 ≤60s 完成。

### D2 — 回归基准不降
- 每个 Phase 结束后运行 `make regression_bench`（对应 v6 基准 JSON）。
- `build_time` 可上升 ≤10%，`path_quality` 不允许下降，`SR` 不允许下降。
- 若回归，**不允许**进入下一 Phase。

### D3 — 接口测试先于实现
- 新模块先写头文件 + 接口测试（Google Test mock），再写实现。
- 不允许"先实现再补测试"。

---

## E. 文档与内存管理

### E1 — 决策变更必须更新 repo memory
- 任何架构决策变更，5 分钟内写入 `/memories/repo/` 对应文件。
- 不允许"口头承诺但不写入"。

### E2 — Phase 完成后更新 session plan
- 每个 Phase 完成后，在 `/memories/session/plan.md` 打勾并写入 `completed_at` 时间戳。

### E3 — 实验结果永不手写
- 所有进入论文的数值来自：`results_nightly/*.json` → `scripts/build_tables.py` → `doc/generated/*.tex`。
- 禁止手动编辑 `doc/generated/` 下的任何文件。

---

## F. 边界条件保持

### F1 — 严格父邻接
- GCS 中每个节点只与 **直接父节点**（Parent-Leaf）及 **已知相邻叶节点** 建立边。
- 不允许建立"跨层"边（grandparent-to-grandchild）。
- 违反此规则会导致 GCS 目标偏差（v6 已知 bug，v7 必须修复）。

### F2 — 目标节点处理
- 目标点 q_goal 必须在规划前完成碰撞检测，失败时立即返回 `INFEASIBLE`。
- 不允许在树展开中"顺便"碰撞检测目标点。

---

## G. 文档输出格式约定

| 文档类型 | 位置 | 语言 |
|---------|------|------|
| 迁移计划 phase 文档 | `cpp/v7/doc/migration/P*.md` | 中文 |
| API 头文件注释 | `cpp/v7/include/**/*.h` | English |
| 实验脚本说明 | `cpp/v7/scripts/*.py` 顶部 docstring | 中文 |
| 论文 | `cpp/v7/doc/*.tex` | 中文版 + 英文版同步 |

---

## H. 本次迁移的"不做"清单

以下是 **明确不做** 的事项，AI 不得擅自添加：

- [ ] 不实现 GPU 加速（留 v8）
- [ ] 不添加 Python async API（留 v8）
- [ ] 不实现分布式多机规划（留 v8）
- [ ] 不引入新的外部依赖（除 yaml-cpp 已存在外）
- [ ] 不修改 GCS solver 本身（使用 Drake 现有接口）
- [ ] 不改变 AABB 碰撞检测的数学定义（仅工程优化）
- [ ] 不在 v7 中实现 B7/B8 实验（需新硬件/机器人模型）

---

*最后更新: 2026-05（v7迁移启动）*
